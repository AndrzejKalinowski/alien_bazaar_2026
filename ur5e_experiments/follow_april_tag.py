"""
Pick things marked with AprilTags using a wrist-mounted webcam and the suction cup.

How it works:
  1. The webcam (mounted on the end effector) looks for AprilTags.
  2. Each tag's pose is estimated with solvePnP and transformed into the robot
     base frame:  T_base_tag = T_base_tcp @ T_tcp_cam @ T_cam_tag
  3. On "pick", the robot servos (speedL, recomputed every video frame) so the
     camera is above the tag, then so the suction cup is just above the tag,
     turns the vacuum on and slowly descends until it feels contact through the
     UR5e's force/torque sensor (or reaches the max depth), then lifts.
     The tag is re-detected on every frame the whole time, so the target keeps
     updating (and follows the tag if it moves). When the tag drops out of view
     (e.g. the camera is too close), the last estimate is used.

Keys (in the video window) / gamepad:
  p  / Y (north)   pick the tag closest to the image center (or TARGET_TAG_ID)
  s  / A (south)   stop / abort the current motion
  g  / B (east)    grip (vacuum on)
  r  / X (west)    release (vacuum off)
  h  / Start       go home
  q/Esc            quit

Manual jogging with the gamepad works like gamepad_robot_teleop.py:
  left stick = move in X/Y, triggers = move in Z, hold RB = rotate instead.
Touching the sticks/triggers during a pick aborts it (manual override).

Setup before first use:
  * Set the TCP on the pendant to the tip of the suction cup, so that
    getActualTCPPose() is the point that touches the object.
  * Measure CAMERA_OFFSET (camera lens position relative to that TCP, in the
    tool frame) and CAMERA_YAW_DEG (how the image is rotated around tool Z).
  * Set TAG_SIZE to the printed size of the tag's black square.
  * For better accuracy, put a camera_calibration.npz (keys camera_matrix and
    dist_coeffs) next to this script; otherwise intrinsics are guessed from
    CAMERA_HFOV_DEG.

Sanity check for the camera mounting: with a tag lying still on the table, jog
the robot around (teleop or freedrive). The "base" position shown in the
window should stay (nearly) constant. If it moves with the robot, the
CAMERA_OFFSET / CAMERA_YAW_DEG values are wrong.

Requires: pip install opencv-python  (AprilTag detection is built into cv2.aruco)
"""

import os
import threading
import time

import cv2
import numpy as np
import rtde_control
import rtde_receive
from inputs import devices
from pygamepad.gamepads import Gamepad
import math

from gamepad_robot_teleop import (MAX_LINEAR_SPEED, MAX_ROTATION_SPEED, MAX_Z_SPEED,
                                  SPEED_ACCEL, apply_deadzone)
from suction import Suction

IP = "192.168.1.20"

# --- camera -----------------------------------------------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_HFOV_DEG = 70.0   # only used when there is no calibration file
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "camera_calibration.npz")
CAMERA_TIMEOUT = 0.5     # s without a new frame -> stop the robot

# Camera pose relative to the TCP (suction cup tip), in the tool frame.
# Assumes the camera looks along tool +Z (same direction as the suction cup).
CAMERA_OFFSET = [0.05, 0.0, 0.05]   # meters, [x, y, z] in tool frame
CAMERA_YAW_DEG = 90                # rotation of camera around tool Z

# --- tags -------------------------------------------------------------------
TAG_DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE = 0.04          # meters, edge of the black square
TARGET_TAG_ID = None     # None = pick the tag closest to the image center
AXES_LENGTH = 0.75       # drawn axes length, as a fraction of TAG_SIZE

TRACK_SMOOTHING = 0.3    # 0..1, weight of each new measurement in the running estimate
TRACK_MAX_JUMP = 0.05    # m, ignore measurements this far from the estimate (misdetections)

# --- motion -----------------------------------------------------------------
HOME_Q = [0, -1.57, 1.57, -1.57, -1.57, 0]   # tool pointing down
HOME_SPEED = 1.0
HOME_ACCEL = 1.0

SERVO_GAIN = 0.8     # 1/s, speed = gain * distance to target
SERVO_MAX_SPEED = 0.15   # m/s
SERVO_ACCEL = 0.5        # m/s^2
SERVO_TOLERANCE = 0.005  # m, "arrived" when closer than this

HOVER_HEIGHT = 0.25      # camera height above the tag for the first approach
APPROACH_HEIGHT = 0.05   # suction cup height above the tag before descending
LIFT_HEIGHT = 0.15       # how far to lift after gripping

DESCEND_SPEED = 0.02         # m/s, slow final approach
DESCEND_ACCEL = 0.2
CONTACT_FORCE = 12.0         # N, stop descending above this
MAX_OVERSHOOT = 0.02         # m, descend at most this far below the estimated tag
MIN_TCP_Z = -0.05            # m in base frame, never go lower than this (table guard)

GRIP_DWELL = 0.5         # s, let the vacuum build up before lifting

# Always pass a time to speedL: with time=0 the speed loop in the ur_rtde control
# script spins without pausing and the robot protective-stops with
# "C271A1: Low level real-time thread: Runtime is too much behind".
# The robot keeps repeating the latest command every SPEED_CMD_TIME.
SPEED_CMD_TIME = 0.02    # s
FT_SETTLE = 0.2          # s, wait after zeroing the force sensor

WINDOW = "april tag pick"


# --- math helpers -------------------------------------------------------------

def pose_to_matrix(pose):
    """UR pose [x, y, z, rx, ry, rz] (axis-angle) -> 4x4 matrix."""
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(pose[3:], dtype=float))
    T[:3, 3] = pose[:3]
    return T


def tcp_to_camera_matrix():
    yaw = np.radians(CAMERA_YAW_DEG)
    T = np.eye(4)
    T[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0],
                 [np.sin(yaw), np.cos(yaw), 0],
                 [0, 0, 1]]
    T[:3, 3] = CAMERA_OFFSET
    return T


T_TCP_CAM = tcp_to_camera_matrix()


# --- camera -----------------------------------------------------------------

class Camera:
    """Reads frames in a background thread so we always get the newest one."""

    def __init__(self, index=CAMERA_INDEX):
        self._cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera {index}")

        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

        while self._frame is None:
            time.sleep(0.01)
        h, w = self._frame.shape[:2]
        self.camera_matrix, self.dist_coeffs = load_intrinsics(w, h)

    def _reader(self):
        while self._running:
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
                    self._stamp = time.time()

    def read(self, newer_than=0.0, timeout=CAMERA_TIMEOUT):
        """Return (frame, timestamp) newer than `newer_than`, or (None, newer_than) on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._stamp > newer_than:
                    return self._frame.copy(), self._stamp
            time.sleep(0.005)
        return None, newer_than

    def close(self):
        self._running = False
        self._thread.join()
        self._cap.release()


def load_intrinsics(width, height):
    if os.path.exists(CALIBRATION_FILE):
        data = np.load(CALIBRATION_FILE)
        print(f"Loaded camera calibration from {CALIBRATION_FILE} (run calibrate_camera.py to redo)")
        camera_matrix = data["camera_matrix"].copy()
        if "image_size" in data:
            calib_w, calib_h = data["image_size"]
            if (calib_w, calib_h) != (width, height):
                # Intrinsics scale with resolution (only valid if the aspect ratio is the same)
                print(f"WARNING: calibrated at {calib_w}x{calib_h} but camera gives "
                      f"{width}x{height}, scaling. Recalibrate for best accuracy.")
                camera_matrix[0] *= width / calib_w
                camera_matrix[1] *= height / calib_h
        return camera_matrix, data["dist_coeffs"]

    print(f"No calibration file, guessing intrinsics from {CAMERA_HFOV_DEG} deg HFOV")
    f = (width / 2) / np.tan(np.radians(CAMERA_HFOV_DEG) / 2)
    camera_matrix = np.array([[f, 0, width / 2],
                              [0, f, height / 2],
                              [0, 0, 1]])
    return camera_matrix, np.zeros(5)


# --- tag detection ------------------------------------------------------------

class TagDetector:
    def __init__(self, camera_matrix, dist_coeffs):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(TAG_DICTIONARY), params)
        self._K = camera_matrix
        self._dist = dist_coeffs
        s = TAG_SIZE / 2
        # Same corner order as cv2.aruco: top-left, top-right, bottom-right, bottom-left
        self._object_points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])

    def detect(self, frame):
        """Return a list of (tag_id, corners, T_cam_tag)."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        tags = []
        if ids is None:
            return tags
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            ok, rvec, tvec = cv2.solvePnP(self._object_points, tag_corners[0],
                                          self._K, self._dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            T = np.eye(4)
            T[:3, :3], _ = cv2.Rodrigues(rvec)
            T[:3, 3] = tvec.flatten()
            tags.append((int(tag_id), tag_corners[0], T))
        return tags

    def choose(self, tags, frame_shape):
        """Pick TARGET_TAG_ID, or the tag closest to the image center."""
        if TARGET_TAG_ID is not None:
            tags = [t for t in tags if t[0] == TARGET_TAG_ID]
        if not tags:
            return None
        h, w = frame_shape[:2]
        center = np.array([w / 2, h / 2])
        return min(tags, key=lambda t: np.linalg.norm(t[1].mean(axis=0) - center))


def tag_position_in_base(tcp_pose, T_cam_tag):
    return (pose_to_matrix(tcp_pose) @ T_TCP_CAM @ T_cam_tag)[:3, 3]


# --- robot tasks ----------------------------------------------------------------
# Each task is advanced once per video frame by calling update(), which sends
# a new command to the robot and returns quickly, so the video never freezes.

class PickTask:
    def __init__(self, r, c, suction, tag_id, tag_pos, tcp_pose):
        self.r = r
        self.c = c
        self.suction = suction
        self.tag_id = tag_id
        self.tag_pos = np.asarray(tag_pos)
        self.orientation = tcp_pose[3:]   # keep the tool orientation (pointing down)
        self.cam_offset_base = pose_to_matrix(tcp_pose)[:3, :3] @ T_TCP_CAM[:3, 3]
        self.stage = "hover"
        self.stage_start = time.time()
        self.lift_target = None
        self.done = False
        self.status = ""

    def _next(self, stage):
        self.c.speedStop()
        self.stage = stage
        self.stage_start = time.time()

    def _track(self, tags, tcp_pose):
        """Update the running estimate of the tag position from this frame."""
        for tag_id, _, T_cam_tag in tags:
            if tag_id != self.tag_id:
                continue
            measured = tag_position_in_base(tcp_pose, T_cam_tag)
            if np.linalg.norm(measured - self.tag_pos) > TRACK_MAX_JUMP:
                return False
            self.tag_pos += TRACK_SMOOTHING * (measured - self.tag_pos)
            return True
        return False

    def _servo(self, tcp_pose, target):
        """One speedL step towards target; returns remaining distance."""
        target = np.array(target, dtype=float)
        target[2] = max(target[2], MIN_TCP_Z)
        error = target - np.asarray(tcp_pose[:3])
        dist = np.linalg.norm(error)
        velocity = SERVO_GAIN * error
        speed = np.linalg.norm(velocity)
        if speed > SERVO_MAX_SPEED:
            velocity *= SERVO_MAX_SPEED / speed
        self.c.speedL(list(velocity) + [0, 0, 0], SERVO_ACCEL, SPEED_CMD_TIME)
        return dist

    def update(self, tags, tcp_pose):
        seen = self._track(tags, tcp_pose)
        tag_info = "tag seen" if seen else "tag not seen, using last estimate"

        if self.stage == "hover":
            # Camera straight above the tag, so it's well centered in view
            target = self.tag_pos + [0, 0, HOVER_HEIGHT] - self.cam_offset_base
            dist = self._servo(tcp_pose, target)
            self.status = f"hover: {dist * 1000:.0f} mm to go ({tag_info})"
            if dist < SERVO_TOLERANCE:
                self._next("approach")

        elif self.stage == "approach":
            target = self.tag_pos + [0, 0, APPROACH_HEIGHT]
            dist = self._servo(tcp_pose, target)
            self.status = f"approach: {dist * 1000:.0f} mm to go ({tag_info})"
            if dist < SERVO_TOLERANCE:
                self._next("zero_ft")
                self.c.zeroFtSensor()
                self.suction.grip()

        elif self.stage == "zero_ft":
            self.status = "zeroing force sensor, vacuum on"
            if time.time() - self.stage_start > FT_SETTLE:
                self._next("descend")

        elif self.stage == "descend":
            force = np.linalg.norm(self.r.getActualTCPForce()[:3])
            min_z = max(self.tag_pos[2] - MAX_OVERSHOOT, MIN_TCP_Z)
            self.status = f"descend: force {force:.1f} N"
            if force > CONTACT_FORCE:
                print("Contact")
                self._next("dwell")
            elif tcp_pose[2] <= min_z:
                print("No contact detected, reached max depth")
                self._next("dwell")
            else:
                self.c.speedL([0, 0, -DESCEND_SPEED, 0, 0, 0], DESCEND_ACCEL, SPEED_CMD_TIME)

        elif self.stage == "dwell":
            self.status = "gripping"
            if time.time() - self.stage_start > GRIP_DWELL:
                self.lift_target = np.asarray(tcp_pose[:3]) + [0, 0, LIFT_HEIGHT]
                self._next("lift")

        elif self.stage == "lift":
            dist = self._servo(tcp_pose, self.lift_target)
            self.status = f"lift: {dist * 1000:.0f} mm to go"
            if dist < SERVO_TOLERANCE:
                self.c.speedStop()
                self.status = "picked (r to release)"
                self.done = True

    def abort(self):
        self.c.speedStop()


class HomeTask:
    def __init__(self, c):
        self.c = c
        self.start = time.time()
        self.done = False
        self.status = "going home..."
        c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL, True)   # asynchronous

    def update(self, tags, tcp_pose):
        # Give the async move a moment to start before checking if it finished
        if time.time() - self.start > 0.2 and self.c.getAsyncOperationProgress() < 0:
            self.status = "home"
            self.done = True

    def abort(self):
        self.c.stopJ()


# --- gamepad ------------------------------------------------------------------

# Gamepad button -> the same action as this keyboard key
GAMEPAD_KEYS = {
    "BTN_NORTH": "p",
    "BTN_SOUTH": "s",
    "BTN_EAST": "g",
    "BTN_WEST": "r",
    "BTN_START": "h",
}


class GamepadControl:
    def __init__(self):
        if not devices.gamepads:
            print("No gamepad found, keyboard only")
        self._gamepad = Gamepad()
        self._gamepad.listen()
        self._was_pressed = {name: False for name in GAMEPAD_KEYS}

    def poll_keys(self):
        """Keys for gamepad buttons pressed since the last call.

        pygamepad's is_just_pressed only lasts 20 ms, shorter than one video
        frame, so we detect the press edges ourselves.
        """
        keys = []
        for name, key in GAMEPAD_KEYS.items():
            pressed = bool(getattr(self._gamepad.buttons, name).value)
            if pressed and not self._was_pressed[name]:
                keys.append(key)
            self._was_pressed[name] = pressed
        return keys

    def jog_speed(self):
        """speedL vector from the sticks, same mapping as gamepad_robot_teleop.py, or None."""
        b = self._gamepad.buttons
        x = apply_deadzone(b.ABS_X.value)
        y = apply_deadzone(b.ABS_Y.value)
        trigger = apply_deadzone(b.ABS_RZ.value - b.ABS_Z.value)
        if x == 0 and y == 0 and trigger == 0:
            return None

        speed = [0.0] * 6
        if b.BTN_TR.value:
            speed[3] = x * MAX_ROTATION_SPEED
            speed[4] = y * MAX_ROTATION_SPEED
            speed[5] = trigger * MAX_ROTATION_SPEED
        else:
            speed[0] = y * MAX_LINEAR_SPEED
            speed[1] = x * MAX_LINEAR_SPEED
            speed[2] = trigger * MAX_Z_SPEED
        return speed

    def close(self):
        self._gamepad.stop_listening()


# --- display ------------------------------------------------------------------

def draw(frame, tags, chosen, tcp_pose, status, camera_matrix, dist_coeffs):
    for tag_id, corners, T_cam_tag in tags:
        color = (0, 255, 0) if chosen is not None and tag_id == chosen[0] else (0, 180, 255)
        pts = corners.astype(int)
        cv2.polylines(frame, [pts], True, color, 2)
        # Tag axes: X red, Y green, Z blue (Z points out of the tag, towards the camera)
        rvec, _ = cv2.Rodrigues(T_cam_tag[:3, :3])
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, T_cam_tag[:3, 3],
                          TAG_SIZE * AXES_LENGTH, 2)
        base = tag_position_in_base(tcp_pose, T_cam_tag)
        x, y = pts[0]
        cv2.putText(frame, f"id {tag_id}  cam z {T_cam_tag[2, 3]:.3f}", (x, y - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, "base {:.3f} {:.3f} {:.3f}".format(*base), (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, "p: pick  s: stop  g: grip  r: release  h: home  q: quit", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


def main():
    camera = Camera()
    detector = TagDetector(camera.camera_matrix, camera.dist_coeffs)
    suction = Suction()
    gamepad = GamepadControl()

    r = rtde_receive.RTDEReceiveInterface(IP)
    c = rtde_control.RTDEControlInterface(IP)
    print("Connected to robot. TCP pose:", r.getActualTCPPose())

    task = None
    jogging = False
    status = "ready"
    stamp = 0.0

    try:
        while True:
            frame, stamp = camera.read(newer_than=stamp)
            if frame is None:
                # Never keep moving blind
                if task is not None:
                    task.abort()
                    task = None
                c.speedStop()
                jogging = False
                status = "camera timeout, stopped"
                print(status)
                continue

            tcp_pose = r.getActualTCPPose()
            tags = detector.detect(frame)
            chosen = detector.choose(tags, frame.shape)

            keys = gamepad.poll_keys()
            key = cv2.waitKey(1) & 0xFF
            if key != 0xFF:
                keys.append(chr(key))
            if "q" in keys or "\x1b" in keys:
                break

            try:
                for key in keys:
                    if key == "s":
                        if task is not None:
                            task.abort()
                            task = None
                        c.speedStop()
                        status = "stopped"
                    elif key == "p" and task is None and not jogging:
                        if chosen is None:
                            status = "no tag in view"
                        else:
                            tag_id, _, T_cam_tag = chosen
                            print(f"Picking tag {tag_id}")
                            task = PickTask(r, c, suction, tag_id,
                                            tag_position_in_base(tcp_pose, T_cam_tag), tcp_pose)
                    elif key == "h" and task is None and not jogging:
                        task = HomeTask(c)
                    elif key == "g":
                        suction.grip()
                        status = "gripped"
                    elif key == "r":
                        suction.release()
                        status = "released"

                jog = gamepad.jog_speed()
                if jog is not None:
                    if task is not None:
                        task.abort()
                        task = None
                        print("Manual override, task aborted")
                    c.speedL(jog, SPEED_ACCEL, SPEED_CMD_TIME)
                    jogging = True
                    status = "manual"
                elif jogging:
                    c.speedStop()
                    jogging = False

                if task is not None:
                    task.update(tags, tcp_pose)
                    status = task.status
                    if task.done:
                        task = None
            except Exception as e:
                print(f"Robot error: {e}")
                status = f"error: {e}"
                task = None
                jogging = False
                try:
                    c.speedStop()
                    c.reuploadScript()
                except Exception as e2:
                    print(f"Recovery failed: {e2}")

            draw(frame, tags, chosen, tcp_pose, status,
                 camera.camera_matrix, camera.dist_coeffs)
            cv2.imshow(WINDOW, frame)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            c.speedStop()
            c.stopScript()
        except Exception:
            pass
        gamepad.close()
        suction.close()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
