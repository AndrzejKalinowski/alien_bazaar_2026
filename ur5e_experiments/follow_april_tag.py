"""
Pick things marked with AprilTags using a wrist-mounted webcam and the suction cup.

How it works:
  1. The webcam (mounted on the end effector) looks for AprilTags.
  2. Each tag's pose is estimated with solvePnP and transformed into the robot
     base frame:  T_base_tag = T_base_tcp @ T_tcp_cam @ T_cam_tag
  3. On "pick", the robot servos (speedL, recomputed every video frame) so the
     tool points into the tag along its surface normal (works for tags lying
     flat, tilted or on vertical faces) with the camera in front of the tag,
     then so the suction cup is just in front of the tag, turns the vacuum on
     and slowly pushes along the normal until it feels contact through the
     UR5e's force/torque sensor (or reaches the max depth), then backs off
     along the normal once the suction controller reports GRIP OK (or releases
     and gives up after GRIP_CONFIRM_TIMEOUT).
     The tool's rotation around the approach axis is kept as close as possible
     to where it started, so the wrist turns as little as possible.
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
  A command that can't run right now (g during the 1.5 s release pulse or
  while a task runs, p / h while a task runs or jogging, ...) is refused with a WARNING
  in the console and the window; the program keeps running.

Manual jogging with the gamepad (see gamepad_jog.py):
  left stick = move in X/Y, triggers = move in Z, hold RB = rotate instead,
  hold LB = fine.
Touching the sticks/triggers during a pick aborts it (manual override).

Motion watchdog (robot_watchdog.py): the robot stops by itself if the main
loop sends nothing for 0.2 s (stalled loop, camera hang, breakpoint). The
next loop iteration then re-uploads the control script and aborts the task.

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
import math

import safe_motion
from gamepad_jog import SPEED_ACCEL, GamepadControl, limit_z_speed
from safe_motion import CEILING_MARGIN, MAX_TCP_Z   # MAX_TCP_Z is re-exported to the other scripts
from robot_watchdog import RobotWatchdog
from serial import SerialException
from suction import Suction, key_command

IP = "192.168.1.20"

# --- camera -----------------------------------------------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_HFOV_DEG = 70.0   # only used when there is no calibration file
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "camera_calibration.npz")
CAMERA_TIMEOUT = 0.5     # s without a new frame -> stop the robot

# Camera pose relative to the TCP (suction cup tip), in the tool frame.
# hand_eye_calibration.py measures it automatically and writes HAND_EYE_FILE,
# which is used instead of the manual values below when it exists.
HAND_EYE_FILE = os.path.join(os.path.dirname(__file__), "hand_eye.npz")
# Manual fallback. Assumes the camera looks along tool +Z (same direction as the suction cup).
CAMERA_OFFSET = [0.05, 0.0, 0.1]   # meters, [x, y, z] in tool frame
CAMERA_YAW_DEG = 0                # rotation of camera around tool Z

# --- tags -------------------------------------------------------------------
TAG_DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE = 0.08          # meters, edge of the black square
TARGET_TAG_ID = None     # None = pick the tag closest to the image center
AXES_LENGTH = 0.75       # drawn axes length, as a fraction of TAG_SIZE

TRACK_SMOOTHING = 0.3    # 0..1, weight of each new measurement in the running estimate
TRACK_MAX_JUMP = 0.05    # m, ignore measurements this far from the estimate (misdetections)
TRACK_MAX_TURN_DEG = 25  # deg, ignore measurements whose normal is this far off the estimate
# Tag normals within this angle of vertical are treated as exactly vertical, so
# flat tags are picked straight down despite pose-estimation noise. 0 = off.
NORMAL_SNAP_DEG = 8
MAX_TILT_DEG = 100       # deg from vertical, refuse to pick tags tilted further (facing down)

# --- motion -----------------------------------------------------------------
HOME_Q = [0, -1.57, 1.57, -1.57, -1.57, 0]   # tool pointing down
HOME_SPEED = 1.0
HOME_ACCEL = 1.0

SERVO_GAIN = 0.8     # 1/s, speed = gain * distance to target
SERVO_MAX_SPEED = 0.15   # m/s
SERVO_ACCEL = 0.5        # m/s^2
SERVO_TOLERANCE = 0.005  # m, "arrived" when closer than this
SERVO_ROT_GAIN = 1.0         # 1/s, rotation speed = gain * angle to target
SERVO_MAX_ROT_SPEED = 0.5    # rad/s
SERVO_ROT_TOLERANCE_DEG = 2  # deg, "arrived" when the orientation is closer than this

HOVER_HEIGHT = 0.25      # camera distance from the tag (along its normal) for the first approach
APPROACH_HEIGHT = 0.05   # suction cup distance from the tag before pushing in
LIFT_HEIGHT = 0.15       # how far to back off along the tag normal after gripping

DESCEND_SPEED = 0.02         # m/s, slow final approach
DESCEND_ACCEL = 0.2
CONTACT_FORCE = 12.0         # N, stop descending above this
MAX_OVERSHOOT = 0.02         # m, push at most this far past the estimated tag surface
MIN_TCP_Z = -0.05            # m in base frame, never go lower than this (table guard)

GRIP_DWELL = 0.5         # s, let the vacuum build up before lifting
# s after contact to wait for the controller's "GRIP OK" before giving up.
# Objects sealed 0.9-5.4 s after GRIP in the controller's tests.
GRIP_CONFIRM_TIMEOUT = 6.0

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
    if os.path.exists(HAND_EYE_FILE):
        print(f"Loaded camera mounting from {HAND_EYE_FILE} "
              "(CAMERA_OFFSET / CAMERA_YAW_DEG ignored, run hand_eye_calibration.py to redo)")
        return np.load(HAND_EYE_FILE)["T_tcp_cam"]

    yaw = np.radians(CAMERA_YAW_DEG)
    T = np.eye(4)
    T[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0],
                 [np.sin(yaw), np.cos(yaw), 0],
                 [0, 0, 1]]
    T[:3, 3] = CAMERA_OFFSET
    return T


def rotvec_between(a, b):
    """Axis-angle of the smallest rotation turning unit vector a into unit vector b."""
    axis = np.cross(a, b)
    s = np.linalg.norm(axis)
    c = np.dot(a, b)
    if s < 1e-9:
        if c > 0:
            return np.zeros(3)
        # Opposite vectors: turn 180 deg around any axis perpendicular to a
        axis = np.cross(a, [1, 0, 0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0, 1, 0])
        return axis / np.linalg.norm(axis) * np.pi
    return axis / s * np.arctan2(s, c)


def rotation_between(a, b):
    return cv2.Rodrigues(rotvec_between(a, b))[0]


def rotation_error(R_target, R_current):
    """Axis-angle (base frame) that turns R_current into R_target."""
    return cv2.Rodrigues(R_target @ R_current.T)[0].flatten()


def angle_between(a, b):
    return np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))


def snap_normal(normal):
    """Treat nearly vertical normals as exactly vertical (see NORMAL_SNAP_DEG)."""
    up = np.array([0.0, 0.0, 1.0])
    if angle_between(normal, up) < np.radians(NORMAL_SNAP_DEG):
        return up
    return normal


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


def tag_pose_in_base(tcp_pose, T_cam_tag):
    return pose_to_matrix(tcp_pose) @ T_TCP_CAM @ T_cam_tag


def tag_position_in_base(tcp_pose, T_cam_tag):
    return tag_pose_in_base(tcp_pose, T_cam_tag)[:3, 3]


def tag_tilt_deg(T_base_tag):
    """Angle between the tag normal (its Z axis, out of the printed face) and vertical."""
    return np.degrees(angle_between(T_base_tag[:3, 2], [0, 0, 1]))


# --- robot tasks ----------------------------------------------------------------
# Each task is advanced once per video frame by calling update(), which sends
# a new command to the robot and returns quickly, so the video never freezes.

class PickTask:
    def __init__(self, r, c, suction, tag_id, T_base_tag, tcp_pose):
        self.r = r
        self.c = c
        self.suction = suction
        self.tag_id = tag_id
        self.tag_pos = T_base_tag[:3, 3].copy()
        self.normal = T_base_tag[:3, 2].copy()   # out of the tag face, towards the camera
        # Target tool orientation: tool Z (suction direction) into the tag.
        # Start from the current orientation and turn it the least possible.
        self.R_target = pose_to_matrix(tcp_pose)[:3, :3]
        self._align_target()
        self.stage = "hover"
        self.stage_start = time.time()
        self.push_dir = None
        self.lift_target = None
        self.done = False
        self.status = ""

    def _next(self, stage):
        self.c.speedStop()
        self.stage = stage
        self.stage_start = time.time()

    def _approach_normal(self):
        return snap_normal(self.normal)

    def _align_target(self):
        """Turn R_target the least possible so tool Z points against the tag normal."""
        n = self._approach_normal()
        self.R_target = rotation_between(self.R_target[:, 2], -n) @ self.R_target

    def _track(self, tags, tcp_pose):
        """Update the running estimate of the tag pose from this frame."""
        for tag_id, _, T_cam_tag in tags:
            if tag_id != self.tag_id:
                continue
            T = tag_pose_in_base(tcp_pose, T_cam_tag)
            measured, normal = T[:3, 3], T[:3, 2]
            if np.linalg.norm(measured - self.tag_pos) > TRACK_MAX_JUMP:
                return False
            if angle_between(normal, self.normal) > np.radians(TRACK_MAX_TURN_DEG):
                return False
            self.tag_pos += TRACK_SMOOTHING * (measured - self.tag_pos)
            self.normal += TRACK_SMOOTHING * (normal - self.normal)
            self.normal /= np.linalg.norm(self.normal)
            self._align_target()
            return True
        return False

    def _servo(self, tcp_pose, target, R_target=None):
        """One speedL step towards target (and R_target); returns (distance, angle) left."""
        target = np.array(target, dtype=float)
        target[2] = min(max(target[2], MIN_TCP_Z), MAX_TCP_Z)
        error = target - np.asarray(tcp_pose[:3])
        dist = np.linalg.norm(error)
        velocity = SERVO_GAIN * error
        speed = np.linalg.norm(velocity)
        if speed > SERVO_MAX_SPEED:
            velocity *= SERVO_MAX_SPEED / speed

        omega = np.zeros(3)
        angle = 0.0
        if R_target is not None:
            rot_error = rotation_error(R_target, pose_to_matrix(tcp_pose)[:3, :3])
            angle = np.linalg.norm(rot_error)
            omega = SERVO_ROT_GAIN * rot_error
            rot_speed = np.linalg.norm(omega)
            if rot_speed > SERVO_MAX_ROT_SPEED:
                omega *= SERVO_MAX_ROT_SPEED / rot_speed

        self.c.speedL(list(velocity) + list(omega), SERVO_ACCEL, SPEED_CMD_TIME)
        return dist, angle

    def _arrived(self, dist, angle):
        return dist < SERVO_TOLERANCE and angle < np.radians(SERVO_ROT_TOLERANCE_DEG)

    def update(self, tags, tcp_pose):
        seen = self._track(tags, tcp_pose)
        tag_info = "tag seen" if seen else "tag not seen, using last estimate"
        n = self._approach_normal()

        if self.stage == "hover":
            # Camera in front of the tag along its normal, so it's well centered in view
            cam_offset_base = self.R_target @ T_TCP_CAM[:3, 3]
            target = self.tag_pos + n * HOVER_HEIGHT - cam_offset_base
            dist, angle = self._servo(tcp_pose, target, self.R_target)
            self.status = (f"hover: {dist * 1000:.0f} mm, {np.degrees(angle):.0f} deg "
                           f"to go ({tag_info})")
            if self._arrived(dist, angle):
                self._next("approach")

        elif self.stage == "approach":
            target = self.tag_pos + n * APPROACH_HEIGHT
            dist, angle = self._servo(tcp_pose, target, self.R_target)
            self.status = (f"approach: {dist * 1000:.0f} mm, {np.degrees(angle):.0f} deg "
                           f"to go ({tag_info})")
            if self._arrived(dist, angle):
                if self.suction.grip():
                    self._next("zero_ft")
                    self.push_dir = -n   # freeze the direction for the final push
                    self.c.zeroFtSensor()
                else:
                    # Keep holding the approach pose until the release pulse ends
                    self.status = "waiting for the release pulse to end"

        elif self.stage == "zero_ft":
            self.status = "zeroing force sensor, vacuum on"
            if time.time() - self.stage_start > FT_SETTLE:
                self._next("descend")

        elif self.stage == "descend":
            force = np.linalg.norm(self.r.getActualTCPForce()[:3])
            # How far the cup is past the tag surface, along the push direction
            depth = np.dot(np.asarray(tcp_pose[:3]) - self.tag_pos, self.push_dir)
            self.status = f"descend: force {force:.1f} N"
            if force > CONTACT_FORCE:
                print("Contact")
                self._next("dwell")
            elif depth >= MAX_OVERSHOOT or (self.push_dir[2] < 0 and tcp_pose[2] <= MIN_TCP_Z):
                print("No contact detected, reached max depth")
                self._next("dwell")
            else:
                self.c.speedL(list(self.push_dir * DESCEND_SPEED) + [0, 0, 0],
                              DESCEND_ACCEL, SPEED_CMD_TIME)

        elif self.stage == "dwell":
            # Lift only once the controller confirms the object is held
            # ("UNKNOWN" = no pressure sensor, so lift after the dwell anyway)
            result = self.suction.grip_result()
            elapsed = time.time() - self.stage_start
            self.status = f"gripping, waiting for vacuum ({result or 'no report yet'})"
            if elapsed > GRIP_DWELL and result in ("OK", "UNKNOWN"):
                # Back off the way we came in
                self.lift_target = np.asarray(tcp_pose[:3]) - self.push_dir * LIFT_HEIGHT
                self._next("lift")
            elif elapsed > GRIP_CONFIRM_TIMEOUT:
                print(f"Grip not confirmed (last report: {result}), releasing")
                self.suction.release()
                self.c.speedStop()
                self.status = "grip failed, vacuum off (p to retry)"
                self.done = True

        elif self.stage == "lift":
            dist, _ = self._servo(tcp_pose, self.lift_target)
            self.status = f"lift: {dist * 1000:.0f} mm to go"
            if self.suction.grip_result() == "LOST":
                print("Object lost while lifting")
                self.c.speedStop()
                self.status = "object lost while lifting (r to release)"
                self.done = True
            elif dist < SERVO_TOLERANCE:
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
        home_z = c.getForwardKinematics(HOME_Q)[2]
        if home_z > MAX_TCP_Z:
            self.status = f"home is above the ceiling ({home_z:.3f} > {MAX_TCP_Z:.3f} m), not moving"
            self.done = True
            return
        self.status = "going home..."
        try:
            c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL, True)   # asynchronous
        except safe_motion.MotionRefused as e:
            self.status = str(e)
            self.done = True

    def update(self, tags, tcp_pose):
        # Give the async move a moment to start before checking if it finished
        if time.time() - self.start > 0.2 and self.c.getAsyncOperationProgress() < 0:
            self.status = "home"
            self.done = True

    def abort(self):
        self.c.stopJ()


# --- suction ------------------------------------------------------------------

class NoSuction:
    """Stand-in when the suction controller isn't connected: only prints."""

    def grip(self):
        print("(no gripper) grip")
        return True

    def grip_result(self):
        return "UNKNOWN"   # like the controller without a pressure sensor

    def release(self, wait=False):
        print("(no gripper) release")
        return True

    def close(self):
        pass


def connect_suction():
    try:
        return Suction()
    except (SerialException, OSError) as e:
        print(f"Suction gripper not available ({e}), running without it")
        return NoSuction()


# --- gamepad ------------------------------------------------------------------

# Gamepad button -> the same action as this keyboard key
GAMEPAD_KEYS = {
    "BTN_NORTH": "p",
    "BTN_SOUTH": "s",
    "BTN_EAST": "g",
    "BTN_WEST": "r",
    "BTN_START": "h",
}


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
        tcp = (T_TCP_CAM @ T_cam_tag)[:3, 3]   # tag position in the tool frame
        x, y = pts[0]
        cv2.putText(frame, f"id {tag_id}  cam z {T_cam_tag[2, 3]:.3f}", (x, y - 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, "base {:.3f} {:.3f} {:.3f}".format(*base), (x, y - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, "tcp  {:.3f} {:.3f} {:.3f}".format(*tcp), (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, "p: pick  s: stop  g: grip  r: release  h: home  q: quit", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


def warn(message):
    """A refused operator command: printed and shown in the window, the loop goes on."""
    print(f"WARNING: {message}")
    return message


def main():
    camera = Camera()
    detector = TagDetector(camera.camera_matrix, camera.dist_coeffs)
    suction = connect_suction()
    gamepad = GamepadControl(GAMEPAD_KEYS)

    r, c = safe_motion.connect(IP)   # caps upward jogging near the ceiling
    print("Connected to robot. TCP pose:", r.getActualTCPPose())

    task = None
    jogging = False
    status = "ready"
    stamp = 0.0

    watchdog = RobotWatchdog(c)
    try:
        while True:
            frame, stamp = camera.read(newer_than=stamp)
            if not watchdog.kick():
                if task is not None:
                    task.abort()
                    task = None
                jogging = False
                status = "robot was stopped (loop stall / protective stop), task aborted"
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
                    elif key in ("p", "h") and (task is not None or jogging):
                        status = warn(f"task running, {key} ignored (s stops the task)" if task else
                                      f"jogging, {key} ignored (release the sticks first)")
                    elif key == "p":
                        if chosen is None:
                            status = warn("no tag in view, p ignored")
                        else:
                            tag_id, _, T_cam_tag = chosen
                            T_base_tag = tag_pose_in_base(tcp_pose, T_cam_tag)
                            tilt = tag_tilt_deg(T_base_tag)
                            if tilt > MAX_TILT_DEG:
                                status = warn(f"tag {tag_id} tilted {tilt:.0f} deg, too far to pick")
                            else:
                                print(f"Picking tag {tag_id} (tilt {tilt:.0f} deg)")
                                task = PickTask(r, c, suction, tag_id, T_base_tag, tcp_pose)
                    elif key == "h":
                        task = HomeTask(c)
                    elif key == "g" and task is not None:
                        # The pick task controls the vacuum and waits for its GRIP result
                        status = warn("task running, g ignored (s stops the task)")
                    elif key in ("g", "r"):
                        status = key_command(suction, key)

                jog = gamepad.jog_speed()
                if jog is not None:
                    # Slow down near the ceiling / table guard, only allow the way back beyond them
                    limit_z_speed(jog, tcp_pose[2], MIN_TCP_Z, MAX_TCP_Z)
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

                # Last line of defence for tasks (e.g. a moveJ home arcing upwards).
                # Jogging is not stopped here: SafeControl.speedL already caps it,
                # and the user has to be able to jog back down.
                if task is not None and tcp_pose[2] > MAX_TCP_Z + CEILING_MARGIN:
                    task.abort()
                    task = None
                    status = f"above ceiling ({tcp_pose[2]:.3f} m), stopped - jog down"
                    print(status)
            except Exception as e:
                print(f"Robot error: {e}")
                status = f"error: {e}"
                task = None
                jogging = False
                try:
                    c.speedStop()
                    c.reuploadScript()
                    watchdog.arm()   # the new script has no watchdog
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
