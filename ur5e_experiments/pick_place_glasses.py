"""
Pick an upside-down glass found by the overhead camera with the suction cup
and put it down on an AprilTag lying on the table.

Uses everything from find_glasses.py (camera calibration, detection area,
colour filter, trackbars); the glasses stand upside down, so the circle seen
from above is the foot and RIM_HEIGHT there must be the glass height (table to
top of the foot), which is where the suction cup grabs.

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

Keys (video window) / gamepad:
  p  / Y (north)   pick & place one glass
  s  / A (south)   stop / abort (vacuum stays as it is)
  g  / B (east)    grip (vacuum on)
  r  / X (west)    release
  h  / Start       go home (HOME_Q from follow_april_tag.py, tool pointing down)
  q / Esc          quit
  Sticks jog the robot (gamepad_jog.py); touching them aborts a running task.
  The mouse edits the detection area as in find_glasses.py.

The place tag (PLACE_TAG_ID, any 36h11 size) lies flat on the table inside
the camera view; its last seen position is kept, since the placed glass covers
it. It is shown as a cyan square in the video and on the map.

Requires: pip install opencv-python ur_rtde pyserial
"""

import time

import cv2
import numpy as np
import rtde_control
import rtde_receive

import find_glasses as fg
from follow_april_tag import HOME_Q, IP, MAX_TCP_Z, MIN_TCP_Z, connect_suction
from gamepad_jog import GamepadControl, Jogger

PLACE_TAG_ID = None          # None = the lowest id in view
PLACED_RADIUS = 0.04         # m, a glass this close to the tag already stands on it

CARRY_CLEARANCE = 0.05       # m, gap under the carried glass over the other glasses
APPROACH_GAP = 0.02          # m, stop this far above the foot / placing height, then go slowly
MAX_OVERSHOOT = 0.015        # m, push at most this far past the expected height
MAX_TILT_DEG = 10            # tool must point down within this at the start

MOVE_SPEED = 0.8            # m/s, moveL
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

GAMEPAD_KEYS = {"BTN_NORTH": "p", "BTN_SOUTH": "s", "BTN_EAST": "g",
                "BTN_WEST": "r", "BTN_START": "h"}


class TaskFailed(Exception):
    pass


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
        except TaskFailed as e:
            print(e)
            self.c.speedStop(STOP_DECEL)
            self.status = str(e)
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
        self.c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL, True)
        yield from self.wait_move("going home...")
        self.status = "home"

    def abort(self):
        self._steps.close()
        self.c.stopJ(STOP_DECEL)


class PickPlaceTask(Task):
    def __init__(self, r, c, suction, glass, place_xy, table_z, rim_height):
        self.suction = suction
        self.glass = glass
        self.place_xy = np.asarray(place_xy)
        self.table_z = table_z
        self.rim_height = rim_height
        super().__init__(r, c)

    def move_to(self, xyz, label, holding=False):
        xyz = [xyz[0], xyz[1], min(max(xyz[2], MIN_TCP_Z), MAX_TCP_Z)]
        self.c.moveL(xyz + self.rotation, MOVE_SPEED, MOVE_ACCEL, True)
        for status in self.wait_move(label):
            if holding and self.suction.grip_result() == "LOST":
                self.c.stopL(STOP_DECEL)
                raise TaskFailed("glass lost while carrying (vacuum still on, r to release)")
            yield status

    def push_down(self, stop_z, force_limit, label):
        """Slowly down until the force sensor feels contact or stop_z is reached."""
        self.c.zeroFtSensor()
        yield from self.wait(FT_SETTLE, f"{label}: zeroing force sensor")
        while True:
            force = np.linalg.norm(self.r.getActualTCPForce()[:3])
            z = self.r.getActualTCPPose()[2]
            if force > force_limit:
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: contact ({force:.1f} N)")
                return
            if z <= max(stop_z, MIN_TCP_Z):
                self.c.speedStop(STOP_DECEL)
                print(f"{label}: no contact felt, reached max depth")
                return
            self.c.speedL([0, 0, -DESCEND_SPEED, 0, 0, 0], DESCEND_ACCEL, SPEED_CMD_TIME)
            yield f"{label}: force {force:.1f} N"

    def run(self):
        start = self.r.getActualTCPPose()
        R = cv2.Rodrigues(np.asarray(start[3:], dtype=float))[0]
        tilt = np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))
        if tilt > MAX_TILT_DEG:
            raise TaskFailed(f"tool is {tilt:.0f} deg from pointing down, go home (h) first")
        self.rotation = list(start[3:])

        g = self.glass
        foot_z = self.table_z + self.rim_height       # tip height on top of the glass
        # The carried glass hangs rim_height below the tip, over glasses rim_height tall
        carry_z = self.table_z + 2 * self.rim_height + CARRY_CLEARANCE
        if carry_z > MAX_TCP_Z:
            raise TaskFailed(f"carry height {carry_z:.3f} m is above MAX_TCP_Z")
        tx, ty = self.place_xy
        print(f"Pick glass at {g.x * 1000:.0f}, {g.y * 1000:.0f} mm, "
              f"place at {tx * 1000:.0f}, {ty * 1000:.0f} mm")

        # Pick
        yield from self.move_to([start[0], start[1], max(start[2], carry_z)], "up")
        yield from self.move_to([g.x, g.y, carry_z], "to glass")
        yield from self.move_to([g.x, g.y, foot_z + APPROACH_GAP], "approach glass")
        self.suction.grip()
        yield from self.push_down(foot_z - MAX_OVERSHOOT, CONTACT_FORCE, "pick")

        grip_start = time.time()
        while True:
            result = self.suction.grip_result()
            elapsed = time.time() - grip_start
            if elapsed > GRIP_DWELL and result in ("OK", "UNKNOWN"):
                break
            if elapsed > GRIP_CONFIRM_TIMEOUT:
                self.suction.release()
                yield from self.move_to([g.x, g.y, carry_z], "grip failed, backing off")
                raise TaskFailed(f"grip not confirmed ({result}), released (p to retry)")
            yield f"gripping, waiting for vacuum ({result or 'no report yet'})"

        # Carry and place
        yield from self.move_to([g.x, g.y, carry_z], "lift", holding=True)
        yield from self.move_to([tx, ty, carry_z], "carry to tag", holding=True)
        yield from self.move_to([tx, ty, foot_z + APPROACH_GAP], "lower", holding=True)
        yield from self.push_down(foot_z - MAX_OVERSHOOT, PLACE_FORCE, "place")
        self.suction.release()
        yield from self.wait(RELEASE_TIME, "releasing")

        # Out of the camera's way
        yield from self.move_to([tx, ty, carry_z], "up")
        yield from self.move_to([start[0], start[1], max(start[2], carry_z)], "back")
        yield from self.move_to(start[:3], "back")
        self.status = "placed"


def find_place_tag(detector, finder, image):
    """Base x, y of the place tag's center on the table, or None."""
    centers = fg.tag_centers(detector, image)
    ids = [PLACE_TAG_ID] if PLACE_TAG_ID is not None else sorted(centers)
    for tag_id in ids:
        if tag_id in centers:
            p = fg.pixel_to_plane(finder.K, finder.T_base_cam, centers[tag_id], finder.table_z)
            if p is not None:
                return p[:2]
    return None


def choose_glass(glasses, place_xy):
    free = [g for g in glasses if np.hypot(g.x - place_xy[0], g.y - place_xy[1]) > PLACED_RADIUS]
    if len(free) < len(glasses):
        print("(skipping the glass already standing on the tag)")
    return min(free, key=lambda g: np.hypot(g.x - place_xy[0], g.y - place_xy[1]), default=None)


def draw_place(image, finder, place_xy, seen):
    T_cam_base = np.linalg.inv(finder.T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    point = np.array([[place_xy[0], place_xy[1], finder.table_z]])
    pixel, _ = cv2.projectPoints(point, rvec, T_cam_base[:3, 3], finder.K, None)
    p = tuple(int(v) for v in pixel.ravel())
    color = (255, 255, 0) if seen else (160, 160, 0)
    cv2.drawMarker(image, p, color, cv2.MARKER_SQUARE, 30, 2)
    cv2.putText(image, "place" if seen else "place (last seen)", (p[0] + 18, p[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    cap, width, height = fg.open_camera()
    finder = fg.GlassFinder.load(width, height)
    detector = fg.make_tag_detector()
    editor = fg.AreaEditor(finder)
    fg.setup_window(editor)

    suction = connect_suction()
    r = rtde_receive.RTDEReceiveInterface(IP)
    c = rtde_control.RTDEControlInterface(IP)
    gamepad = GamepadControl(GAMEPAD_KEYS)
    jogger = Jogger(c, gamepad)
    print("Connected to robot. TCP pose:", r.getActualTCPPose())

    task = None
    place_xy = None
    status = "p: pick & place  s: stop  g/r: grip/release  h: home  q: quit"
    try:
        while True:
            if fg.window_closed():
                break
            fg.read_trackbars(finder)
            image = finder.undistort(fg.read_frame(cap))
            glasses = finder.detect(image)
            seen = find_place_tag(detector, finder, image)
            if seen is not None:
                place_xy = seen
            tcp = r.getActualTCPPose()

            if task is not None:
                task.update()
                status = task.status
                if task.done:
                    task = None

            fg.draw_overlay(image, finder, editor, glasses, status, tcp[:2], place_xy)
            if place_xy is not None:
                draw_place(image, finder, place_xy, seen is not None)
            cv2.imshow(fg.WINDOW, image)

            if gamepad.jog_speed() is not None and task is not None:
                task.abort()
                task = None
                status = "manual override, task aborted"
                print(status)
            if task is None:
                jogger.update()

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
                elif key == "g":
                    suction.grip()
                    status = "vacuum on"
                elif key == "r":
                    suction.release()
                    status = "released"
                elif key == "h" and task is None:
                    jogger.stop()
                    task = HomeTask(r, c)
                elif key == "p" and task is None:
                    jogger.stop()
                    if place_xy is None:
                        status = f"no place tag {PLACE_TAG_ID if PLACE_TAG_ID is not None else ''} in view"
                        continue
                    status = "measuring..."
                    glass = choose_glass(finder.measure(cap), place_xy)
                    if glass is None:
                        status = "no glass found"
                        continue
                    task = PickPlaceTask(r, c, suction, glass, place_xy,
                                         finder.table_z, fg.RIM_HEIGHT)
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
