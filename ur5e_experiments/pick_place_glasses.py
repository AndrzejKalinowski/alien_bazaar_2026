"""
Pick an upside-down glass found by the overhead camera with the suction cup
and put it down on an AprilTag lying on the table.

Uses everything from find_glasses.py (camera calibration, detection area,
colour filter, trackbars); the glasses stand upside down, so the circle seen
from above is the foot. GLASS_HEIGHT (table to top of the foot) is both the
height of that circle for the detection and where the suction cup grabs; it is
separate from find_glasses.RIM_HEIGHT, so tuning one script can't break the other.

Sequence (p / gamepad Y):
  1. Measure the glasses and the place tag (averaged over several frames; the
     robot should not block the camera's view of them).
  2. Pick the glass closest to the tag (glasses already standing on the tag
     are skipped).
  3. Up to CARRY_Z, over the glass, down to APPROACH_GAP above the foot, vacuum
     on, slowly down until the force sensor feels contact, wait for "GRIP OK".
  4. Up to CARRY_Z, over the tag, down to APPROACH_GAP above the placing
     height, slowly down until the glass touches the table, release.
  5. Up to CARRY_Z and back to the start pose (keep it out of the camera view).
  The tool keeps the orientation it has at the start, which must be pointing
  down (e.g. after h / home).

Side grip (PICK_FROM_SIDE = True): the suction cup grabs the glass wall at
SIDE_GRIP_HEIGHT above the table instead of the foot on top.
  3. Up, then the tool turns horizontal (pointing from the robot base towards
     the glass, turned by SIDE_APPROACH_YAW_DEG), over to SIDE_STANDOFF in front
     of the wall at carry height, down to SIDE_GRIP_HEIGHT, vacuum on, slowly
     sideways into the wall. A glass slides at ~1 N, long before the force
     sensor notices, so this stops at the expected wall + SIDE_MAX_PRESS (the
     force limit only guards against hitting something solid). Wait for
     "GRIP OK".
  4. Lift, carry so the glass axis is over the tag (the tool keeps its
     orientation), slowly down until the glass touches the table, release,
     back off SIDE_STANDOFF sideways, up.
  5. Over the start position the tool turns back to its start orientation.
  The start orientation can be anything (no pointing-down check), e.g. already
  sideways; the turn to the side orientation happens over the start position.
  The grip orientation is taught: jog the cup onto a glass wall exactly as it
  should grip (hold RB for rotation) and press o. It prints SIDE_GRIP_ROTATION
  (the tool orientation in the base frame, used as it is for every glass, the
  approach runs along its tool z axis) and SIDE_GRIP_HEIGHT; copy them into
  the constants. With SIDE_GRIP_ROTATION = None the tool is held level instead,
  set by SIDE_APPROACH_YAW_DEG (relative to the base -> glass direction) and
  SIDE_ROLL_DEG (the turn around the tool's own axis: 0 = tool x axis pointing
  straight down, positive = right-hand around the approach direction); o
  prints those too when the tool is roughly level.
  The wall radius at the grip height is SIDE_GRIP_RADIUS, or half the measured
  foot diameter for straight-sided glasses. The approach path is not checked:
  the gripper must fit between the glasses on the robot side of the target.

Keys (video window) / gamepad:
  p  / Y (north)   pick & place one glass
  s  / A (south)   stop / abort (vacuum stays as it is)
  g  / B (east)    grip (vacuum on)
  r  / X (west)    release
  h  / Start       go home (HOME_Q from follow_april_tag.py, tool pointing down)
  o                print the side grip constants of the current tool pose
  q / Esc          quit
  Sticks jog the robot (gamepad_jog.py); touching them aborts a running task.
  The mouse edits the detection area as in find_glasses.py.
  A command that can't run right now (g during the 1.5 s release pulse or
  while a task runs, p / h while a task runs, ...) is refused with a WARNING
  in the console and the window; the program keeps running.

All motion goes through safe_motion.py: the TCP never goes above MAX_TCP_Z
(0.60 m above the base). A move that would is refused and the task stops.

The place tag (PLACE_TAG_ID, any 36h11 size) lies flat on the table inside
the camera view; its last seen position is kept, since the placed glass covers
it. It is shown as a cyan square in the video (with its id) and on the map.
With PLACE_TAG_ID = None the lowest id in view is used; p warns when more
than one tag is in view (e.g. a calibration tag left on the table).

Motion watchdog (robot_watchdog.py): the robot stops by itself if the main
loop sends nothing for 0.2 s (stalled loop, camera hang, breakpoint). The
next loop iteration then re-uploads the control script and aborts the task.

Faults are recovered in place, without restarting (which would also reset the
gripper, 2 s): any error in the loop (a robot call failing after a protective
stop, a lost RTDE connection, no camera frame for FRAME_TIMEOUT) stops all
motion, aborts the task, prints the traceback and a WARNING, reconnects RTDE
if needed and carries on. After a protective stop, clear it on the pendant;
the control script is then re-uploaded by itself. Ctrl+C, q / Esc or closing
the window still quit.

Requires: pip install opencv-python ur_rtde pyserial numpy
"""

import time
import traceback

import cv2
import numpy as np

import find_glasses as fg
import safe_motion
from follow_april_tag import HOME_Q, IP, MAX_TCP_Z, MIN_TCP_Z, connect_suction
from gamepad_jog import GamepadControl, Jogger
from robot_watchdog import RobotWatchdog
from safe_motion import MotionRefused
from serial import SerialException
from suction import key_command

PLACE_TAG_ID = None          # None = the lowest id in view
PLACED_RADIUS = 0.04         # m, a glass this close to the tag already stands on it
GLASS_HEIGHT = 0.075         # m, upside-down glass: table to top of the foot (seen circle, cup lands here)

CARRY_CLEARANCE = 0.05       # m, gap under the carried glass over the other glasses
APPROACH_GAP = 0.02          # m, stop this far above the foot / placing height, then go slowly
MAX_OVERSHOOT = 0.015        # m, push at most this far past the expected height
MAX_TILT_DEG = 10            # tool must point down within this at the start

# --- side grip ------------------------------------------------------------------
PICK_FROM_SIDE = True        # grab the glass wall with the tool horizontal instead of the foot
SIDE_GRIP_HEIGHT = 0.035    # m above the table, cup center (TCP) on the wall (taught pose); the gripper must clear the table here
SIDE_GRIP_RADIUS = None      # m, glass radius at SIDE_GRIP_HEIGHT, None = measured foot diameter / 2
# Taught grip orientation, axis-angle in the base frame (o key prints it), used for
# every glass; the approach runs along its tool z axis (here base -y, 0.7 deg down).
# None = level tool built from SIDE_APPROACH_YAW_DEG / SIDE_ROLL_DEG instead.
SIDE_GRIP_ROTATION = [1.48600, -0.65077, 0.65623]
SIDE_APPROACH_YAW_DEG = 8    # deg, turn the approach from radial (base -> glass) around vertical
SIDE_ROLL_DEG = 138          # deg, tool turned around its own axis: 0 = tool x straight down (o key reads it off)
SIDE_STANDOFF = 0.03         # m, gap between cup and wall before the slow approach and after release
SIDE_MAX_PRESS = 0.006       # m, go at most this far past the expected wall (camera error, cup compression)
SIDE_CONTACT_FORCE = 5.0     # N, stop the sideways approach early (a free glass slides before this)

MOVE_SPEED = 0.15            # m/s, moveL
APPROACH_SPEED = 0.05        # m/s, moveL over to the glass and down next to it / onto the tag
MOVE_ACCEL = 0.6             # m/s^2 (keep low enough for the vacuum to hold the glass)
DESCEND_SPEED = 0.015        # m/s, slow final approach
DESCEND_ACCEL = 0.2
CONTACT_FORCE = 10.0         # N, touching the glass foot
PLACE_FORCE = 8.0            # N, glass touching the table
FT_SETTLE = 0.2              # s, wait after zeroing the force sensor
GRIP_DWELL = 0.5             # s, let the vacuum build up before lifting
GRIP_CONFIRM_TIMEOUT = 6.0   # s
RELEASE_TIME = 1.7           # s, the release pulse lasts 1.5 s
SPEED_CMD_TIME = 0.02        # s
STOP_DECEL = 1.0             # m/s^2
HOME_SPEED = 1.0
HOME_ACCEL = 1.0
RECONNECT_DELAY = 1.0        # s, wait after a failed reconnect before the loop retries

GAMEPAD_KEYS = {"BTN_NORTH": "p", "BTN_SOUTH": "s", "BTN_EAST": "g",
                "BTN_WEST": "r", "BTN_START": "h"}


class TaskFailed(Exception):
    pass


def warn(message):
    """A refused operator command: printed and shown in the window, the loop goes on."""
    print(f"WARNING: {message}")
    return message


class Task:
    """Runs a generator one step per video frame; each step returns quickly."""

    def __init__(self, r, c):
        self.r = r
        self.c = c
        self.status = ""
        self.done = False
        self._steps = self.run()

    def run(self):
        yield from ()

    def update(self):
        try:
            self.status = next(self._steps)
        except StopIteration:
            self.done = True
        except (TaskFailed, MotionRefused) as e:
            print(e)
            self.c.speedStop(STOP_DECEL)
            self.status = str(e)
            self.done = True
        except (SerialException, OSError) as e:
            # Gripper unplugged mid-task: stop this task, not the whole program
            self.c.speedStop(STOP_DECEL)
            self.c.stopL(STOP_DECEL)
            self.status = warn(f"gripper not responding ({e}), task aborted")
            self.done = True

    def abort(self):
        self._steps.close()
        self.c.speedStop(STOP_DECEL)
        self.c.stopL(STOP_DECEL)

    def wait_move(self, label):
        """Wait for the asynchronous move started just before."""
        start = time.time()
        while True:
            # Give the async move a moment to start before checking if it finished
            if time.time() - start > 0.2 and self.c.getAsyncOperationProgress() < 0:
                return
            yield label

    def wait(self, seconds, label):
        end = time.time() + seconds
        while time.time() < end:
            yield label


class HomeTask(Task):
    def run(self):
        home_z = self.c.getForwardKinematics(HOME_Q)[2]
        if home_z > MAX_TCP_Z:
            raise TaskFailed(f"home is above the ceiling ({home_z:.3f} > {MAX_TCP_Z:.3f} m), not moving")
        self.c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL, True)
        yield from self.wait_move("going home...")
        self.status = "home"

    def abort(self):
        self._steps.close()
        self.c.stopJ(STOP_DECEL)


class PickPlaceTask(Task):
    def __init__(self, r, c, suction, glass, place_xy, table_z, glass_height):
        self.suction = suction
        self.glass = glass
        self.place_xy = np.asarray(place_xy)
        self.table_z = table_z
        self.glass_height = glass_height
        super().__init__(r, c)

    def move_to(self, xyz, label, holding=False, rotation=None, speed=MOVE_SPEED):
        xyz = [xyz[0], xyz[1], min(max(xyz[2], MIN_TCP_Z), MAX_TCP_Z)]
        rotation = self.rotation if rotation is None else rotation
        self.c.moveL(xyz + list(rotation), speed, MOVE_ACCEL, True)
        for status in self.wait_move(label):
            if holding and self.suction.grip_result() == "LOST":
                self.c.stopL(STOP_DECEL)
                raise TaskFailed("glass lost while carrying (vacuum still on, r to release)")
            yield status

    def push(self, direction, max_travel, force_limit, label):
        """Slowly along direction until the force sensor feels contact or max_travel is covered."""
        direction = np.asarray(direction, dtype=float)
        direction /= np.linalg.norm(direction)
        self.c.zeroFtSensor()
        yield from self.wait(FT_SETTLE, f"{label}: zeroing force sensor")
        start = np.asarray(self.r.getActualTCPPose()[:3])
        while True:
            force = np.linalg.norm(self.r.getActualTCPForce()[:3])
            pos = np.asarray(self.r.getActualTCPPose()[:3])
            if force > force_limit:
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: contact ({force:.1f} N)")
                return
            if (pos - start) @ direction >= max_travel or (direction[2] < 0 and pos[2] <= MIN_TCP_Z):
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: no contact felt, reached max depth")
                return
            self.c.speedL(list(direction * DESCEND_SPEED) + [0, 0, 0], DESCEND_ACCEL, SPEED_CMD_TIME)
            yield f"{label}: force {force:.1f} N"

    def push_down(self, stop_z, force_limit, label):
        """Slowly down until the force sensor feels contact or stop_z is reached."""
        z = self.r.getActualTCPPose()[2]
        yield from self.push([0, 0, -1], z - stop_z, force_limit, label)

    def wait_for_grip(self, back_off):
        """Wait for the vacuum; on failure release and move through the back_off points."""
        grip_start = time.time()
        while True:
            result = self.suction.grip_result()
            elapsed = time.time() - grip_start
            if elapsed > GRIP_DWELL and result in ("OK", "UNKNOWN"):
                return
            if elapsed > GRIP_CONFIRM_TIMEOUT:
                self.suction.release()
                for xyz in back_off:
                    yield from self.move_to(xyz, "grip failed, backing off")
                raise TaskFailed(f"grip not confirmed ({result}), released (p to retry)")
            yield f"gripping, waiting for vacuum ({result or 'no report yet'})"

    def check_start(self):
        """Start pose, after checking that the tool points down."""
        start = self.r.getActualTCPPose()
        R = cv2.Rodrigues(np.asarray(start[3:], dtype=float))[0]
        tilt = np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))
        if tilt > MAX_TILT_DEG:
            raise TaskFailed(f"tool is {tilt:.0f} deg from pointing down, go home (h) first")
        self.rotation = list(start[3:])
        return start

    def run(self):
        start = self.check_start()
        g = self.glass
        foot_z = self.table_z + self.glass_height       # tip height on top of the glass
        # The carried glass hangs glass_height below the tip, over glasses glass_height tall
        carry_z = self.table_z + 2 * self.glass_height + CARRY_CLEARANCE
        if carry_z > MAX_TCP_Z:
            raise TaskFailed(f"carry height {carry_z:.3f} m is above MAX_TCP_Z")
        tx, ty = self.place_xy
        print(f"Pick glass at {g.x * 1000:.0f}, {g.y * 1000:.0f} mm, "
              f"place at {tx * 1000:.0f}, {ty * 1000:.0f} mm")

        # Pick
        yield from self.move_to([start[0], start[1], max(start[2], carry_z)], "up")
        yield from self.move_to([g.x, g.y, carry_z], "to glass", speed=APPROACH_SPEED)
        yield from self.move_to([g.x, g.y, foot_z + APPROACH_GAP], "approach glass", speed=APPROACH_SPEED)
        while not self.suction.grip():
            yield "waiting for the release pulse to end"
        yield from self.push_down(foot_z - MAX_OVERSHOOT, CONTACT_FORCE, "pick")

        yield from self.wait_for_grip([[g.x, g.y, carry_z]])

        # Carry and place
        yield from self.move_to([g.x, g.y, carry_z], "lift", holding=True)
        yield from self.move_to([tx, ty, carry_z], "carry to tag", holding=True)
        yield from self.move_to([tx, ty, foot_z + APPROACH_GAP], "lower", holding=True, speed=APPROACH_SPEED)
        yield from self.push_down(foot_z - MAX_OVERSHOOT, PLACE_FORCE, "place")
        self.suction.release()
        yield from self.wait(RELEASE_TIME, "releasing")

        # Out of the camera's way
        yield from self.move_to([tx, ty, carry_z], "up")
        yield from self.move_to([start[0], start[1], max(start[2], carry_z)], "back")
        yield from self.move_to(start[:3], "back")
        self.status = "placed"


def recover(r, c, watchdog):
    """After an exception in the main loop: stop all motion, reconnect what dropped out.

    The control script itself is re-uploaded by watchdog.kick() on the next
    frame, once the robot is not protective- or emergency-stopped any more.
    """
    for stop in (c.speedStop, c.stopL, c.stopJ):
        try:
            stop(STOP_DECEL)
        except Exception:
            pass
    try:
        if not r.isConnected():
            print("RTDE receive connection lost, reconnecting")
            r.reconnect()
        if not c.isConnected():
            print("RTDE control connection lost, reconnecting")
            c.reconnect()
            watchdog.arm()
    except Exception as e:
        print(f"Reconnect failed ({e}), retrying on the next error")
        time.sleep(RECONNECT_DELAY)


def side_rotation(direction, roll_deg):
    """Axis-angle rotation with the tool z axis along the horizontal direction and the
    tool x axis turned roll_deg (right-hand around tool z) away from straight down."""
    z = np.array([direction[0], direction[1], 0.0])
    z /= np.linalg.norm(z)
    down = np.array([0.0, 0.0, -1.0])
    roll = np.radians(roll_deg)
    x = np.cos(roll) * down + np.sin(roll) * np.cross(z, down)
    R = np.column_stack([x, np.cross(z, x), z])
    return list(cv2.Rodrigues(R)[0].ravel())


def read_side_orientation(tcp):
    """SIDE_APPROACH_YAW_DEG and SIDE_ROLL_DEG of a jogged pose, as side_rotation() defines
    them (yaw relative to the radial direction at the TCP), or None if the tool is not
    roughly horizontal."""
    R = cv2.Rodrigues(np.asarray(tcp[3:], dtype=float))[0]
    z = R[:, 2]
    if abs(z[2]) > np.sin(np.radians(MAX_TILT_DEG)) or np.hypot(tcp[0], tcp[1]) < 0.1:
        return None
    z = np.array([z[0], z[1], 0.0]) / np.hypot(z[0], z[1])
    down = np.array([0.0, 0.0, -1.0])
    yaw = np.degrees(np.arctan2(z[1], z[0]) - np.arctan2(tcp[1], tcp[0]))
    roll = np.degrees(np.arctan2(R[:, 0] @ np.cross(z, down), R[:, 0] @ down))
    return (yaw + 180) % 360 - 180, roll


class SidePickPlaceTask(PickPlaceTask):
    """Like PickPlaceTask, but the cup grabs the glass wall with the tool horizontal."""

    def run(self):
        # Any start orientation: the side orientation is built from scratch and the
        # tool only turns back to the start orientation at the end
        start = self.r.getActualTCPPose()
        start_rotation = list(start[3:])
        g = self.glass
        glass_xy = np.array([g.x, g.y])
        radius = SIDE_GRIP_RADIUS if SIDE_GRIP_RADIUS is not None else g.diameter / 2
        if np.linalg.norm(glass_xy) < 0.1:
            raise TaskFailed("glass is too close to the robot base for a side grip")

        if SIDE_GRIP_ROTATION is not None:
            # Taught orientation as it is, approach along its tool z axis (may dip a little)
            self.rotation = list(SIDE_GRIP_ROTATION)
            approach = cv2.Rodrigues(np.asarray(SIDE_GRIP_ROTATION, dtype=float))[0][:, 2]
            d = approach[:2] / np.linalg.norm(approach[:2])
        else:
            # Radial approach: tilting the tool outwards is a wrist 1 move, far from the
            # wrist singularity (wrist 2 stays near -90 deg as in HOME_Q)
            yaw = np.radians(SIDE_APPROACH_YAW_DEG)
            radial = glass_xy / np.linalg.norm(glass_xy)
            d = np.array([radial[0] * np.cos(yaw) - radial[1] * np.sin(yaw),
                          radial[0] * np.sin(yaw) + radial[1] * np.cos(yaw)])
            approach = np.array([d[0], d[1], 0.0])
            self.rotation = side_rotation(d, SIDE_ROLL_DEG)

        grip_z = self.table_z + SIDE_GRIP_HEIGHT
        # The carried glass hangs SIDE_GRIP_HEIGHT below the tip, over glasses glass_height tall
        carry_z = self.table_z + self.glass_height + SIDE_GRIP_HEIGHT + CARRY_CLEARANCE
        if carry_z > MAX_TCP_Z:
            raise TaskFailed(f"carry height {carry_z:.3f} m is above MAX_TCP_Z")
        safe_z = max(start[2], carry_z)
        # SIDE_STANDOFF back along the approach from the wall point at grip height
        wall = np.array([*(glass_xy - d * radius), grip_z])
        standoff = wall - approach * SIDE_STANDOFF
        tx, ty = self.place_xy - d * radius      # tip position with the glass axis over the tag
        print(f"Side pick glass at {g.x * 1000:.0f}, {g.y * 1000:.0f} mm "
              f"(radius {radius * 1000:.0f} mm), place at "
              f"{self.place_xy[0] * 1000:.0f}, {self.place_xy[1] * 1000:.0f} mm")

        # Turn the tool horizontal high up, away from the glasses
        yield from self.move_to([start[0], start[1], safe_z], "up", rotation=start_rotation)
        yield from self.move_to([start[0], start[1], safe_z], "turn tool sideways")
        yield from self.move_to([standoff[0], standoff[1], carry_z], "to glass", speed=APPROACH_SPEED)
        yield from self.move_to(list(standoff), "down beside glass", speed=APPROACH_SPEED)
        while not self.suction.grip():
            yield "waiting for the release pulse to end"
        yield from self.push(approach, SIDE_STANDOFF + SIDE_MAX_PRESS, SIDE_CONTACT_FORCE, "pick")
        yield from self.wait_for_grip([list(standoff),
                                       [standoff[0], standoff[1], carry_z]])

        # Carry and place
        tcp = self.r.getActualTCPPose()
        yield from self.move_to([tcp[0], tcp[1], carry_z], "lift", holding=True)
        yield from self.move_to([tx, ty, carry_z], "carry to tag", holding=True)
        yield from self.move_to([tx, ty, grip_z + APPROACH_GAP], "lower", holding=True, speed=APPROACH_SPEED)
        yield from self.push_down(grip_z - MAX_OVERSHOOT, PLACE_FORCE, "place")
        self.suction.release()
        yield from self.wait(RELEASE_TIME, "releasing")

        # Back off sideways before going up so the cup does not drag the glass
        tcp = self.r.getActualTCPPose()
        away = np.array(tcp[:3]) - approach * SIDE_STANDOFF
        yield from self.move_to(list(away), "back off")
        yield from self.move_to([away[0], away[1], carry_z], "up")
        yield from self.move_to([start[0], start[1], safe_z], "back")
        yield from self.move_to([start[0], start[1], safe_z], "turn tool back", rotation=start_rotation)
        yield from self.move_to(start[:3], "back", rotation=start_rotation)
        self.status = "placed"


def find_place_tag(detector, finder, image):
    """(base x, y of the place tag's center on the table, its id, all ids in view), or None."""
    centers = fg.tag_centers(detector, image)
    ids = [PLACE_TAG_ID] if PLACE_TAG_ID is not None else sorted(centers)
    for tag_id in ids:
        if tag_id in centers:
            p = fg.pixel_to_plane(finder.K, finder.T_base_cam, centers[tag_id], finder.table_z)
            if p is not None:
                return p[:2], tag_id, sorted(centers)
    return None


def choose_glass(glasses, place_xy):
    free = [g for g in glasses if np.hypot(g.x - place_xy[0], g.y - place_xy[1]) > PLACED_RADIUS]
    if len(free) < len(glasses):
        print("(skipping the glass already standing on the tag)")
    return min(free, key=lambda g: np.hypot(g.x - place_xy[0], g.y - place_xy[1]), default=None)


def draw_place(image, finder, place_xy, tag_id, seen):
    T_cam_base = np.linalg.inv(finder.T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    point = np.array([[place_xy[0], place_xy[1], finder.table_z]])
    pixel, _ = cv2.projectPoints(point, rvec, T_cam_base[:3, 3], finder.K, None)
    p = tuple(int(v) for v in pixel.ravel())
    color = (255, 255, 0) if seen else (160, 160, 0)
    cv2.drawMarker(image, p, color, cv2.MARKER_SQUARE, 30, 2)
    label = f"place: tag {tag_id}" + ("" if seen else " (last seen)")
    cv2.putText(image, label, (p[0] + 18, p[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    cap, width, height = fg.open_camera()
    finder = fg.GlassFinder.load(width, height, rim_height=GLASS_HEIGHT)
    detector = fg.make_tag_detector()
    editor = fg.AreaEditor(finder)
    fg.setup_window(editor)

    suction = connect_suction()
    r, c = safe_motion.connect(IP)
    gamepad = GamepadControl(GAMEPAD_KEYS)
    jogger = Jogger(c, gamepad, MIN_TCP_Z, MAX_TCP_Z)
    print("Connected to robot. TCP pose:", r.getActualTCPPose())

    task = None
    place_xy = None
    place_id = None
    tags_in_view = []
    status = "p: pick & place  s: stop  g/r: grip/release  h: home  q: quit"
    watchdog = RobotWatchdog(c)
    try:
        while True:
            if fg.window_closed():
                break
            try:
                fg.read_trackbars(finder)
                image = finder.undistort(fg.read_frame(cap))
                if not watchdog.kick():
                    if task is not None:
                        task.abort()
                        task = None
                    jogger.stop()
                    status = "robot was stopped (loop stall / protective stop), task aborted"
                glasses = finder.detect(image)
                seen = find_place_tag(detector, finder, image)
                if seen is not None:
                    place_xy, place_id, tags_in_view = seen
                tcp = r.getActualTCPPose()

                if task is not None:
                    task.update()
                    status = task.status
                    if task.done:
                        task = None
                # Backstop for moves that got above the ceiling anyway (safe_motion.py)
                if task is not None and c.over_ceiling():
                    task.abort()
                    task = None
                    status = warn(f"above the {MAX_TCP_Z:.2f} m ceiling, task stopped (jog down)")

                fg.draw_overlay(image, finder, editor, glasses, status, tcp[:2], place_xy)
                if place_xy is not None:
                    draw_place(image, finder, place_xy, place_id, seen is not None)
                cv2.imshow(fg.WINDOW, image)

                if gamepad.jog_speed() is not None and task is not None:
                    task.abort()
                    task = None
                    status = "manual override, task aborted"
                    print(status)
                if task is None:
                    jogger.update(tcp)

                keys = fg.read_keys(gamepad)
                if "q" in keys or "\x1b" in keys:
                    break
                for key in keys:
                    if key == "s":
                        if task is not None:
                            task.abort()
                            task = None
                        jogger.stop()
                        c.speedStop(STOP_DECEL)
                        status = "stopped"
                    elif key == "g" and task is not None:
                        # The pick task controls the vacuum and waits for its GRIP result
                        status = warn("task running, g ignored (s stops the task)")
                    elif key in ("g", "r"):
                        status = key_command(suction, key)
                    elif key == "o":
                        pose = r.getActualTCPPose()
                        rotation = ", ".join(f"{v:.5f}" for v in pose[3:])
                        print(f"SIDE_GRIP_ROTATION = [{rotation}]\n"
                              f"SIDE_GRIP_HEIGHT = {pose[2] - finder.table_z:.4f}")
                        side = read_side_orientation(pose)
                        if side is None:
                            status = "o: tool not horizontal, only SIDE_GRIP_ROTATION printed"
                        else:
                            status = f"SIDE_APPROACH_YAW_DEG = {side[0]:.0f}  SIDE_ROLL_DEG = {side[1]:.0f}"
                        print(status)
                    elif key in ("h", "p") and task is not None:
                        status = warn(f"task running ({task.status}), {key} ignored (s stops the task)")
                    elif key == "h":
                        jogger.stop()
                        task = HomeTask(r, c)
                    elif key == "p":
                        jogger.stop()
                        if place_xy is None:
                            status = warn(f"no place tag {PLACE_TAG_ID if PLACE_TAG_ID is not None else ''} "
                                          "in view, p ignored")
                            continue
                        if PLACE_TAG_ID is None and len(tags_in_view) > 1:
                            # Not refused: the id of the place tag is not fixed, so the lowest wins
                            warn(f"tags {tags_in_view} in view, placing on the lowest id {place_id}")
                        status = "measuring..."
                        glass = choose_glass(finder.measure(cap, kick=watchdog.kick), place_xy)
                        if glass is None:
                            status = warn("no glass found, p ignored")
                            continue
                        task_class = SidePickPlaceTask if PICK_FROM_SIDE else PickPlaceTask
                        task = task_class(r, c, suction, glass, place_xy,
                                          finder.table_z, GLASS_HEIGHT)
            except Exception as e:
                # Robot fault, lost connection, camera hiccup...: stop and recover in
                # place. Restarting would also cost the gripper's 2 s serial reset.
                traceback.print_exc()
                status = warn(f"error: {e} - stopped, recovering")
                task = None
                recover(r, c, watchdog)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):   # keep the window alive, allow quit
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if task is not None:
                task.abort()
            c.speedStop(STOP_DECEL)
            c.stopScript()
        except Exception:
            pass
        gamepad.close()
        suction.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
