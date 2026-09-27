"""Hardware DeviceAdapter: UR5e through SafeControl, gripper and servo workers.

NOT RUN ON HARDWARE YET. Everything here was only exercised against fakes
(tests/fake_robot.py). The first real run needs the steps in docs/supervisor.md.

Ownership: HardwareDevices is called only from the ControlLoop owner thread.
service() runs once per loop step and kicks the RTDE watchdog (RobotWatchdog,
armed by arm() right before the loop starts). A stalled loop therefore stops
the robot after 0.2 s. RTDE calls here are non-blocking: moveL / moveJ are
async, contact moves are speedL with SPEED_CMD_TIME refreshed every tick,
stops use the asynchronous stopL / stopJ. USB goes through system_io workers.

Each recipe step is a plan: a list of actions (async moveL / moveJ, contact
push, gripper and servo jobs). validate_motion() builds it from the taught
poses (system_teach.py) and checks the TCP polyline against the measured
station layout (system_geometry.py). begin() refuses a plan when the robot
moved since validation. Postconditions are checked, never assumed:
  moves      finished (async progress < 0) AND the actual pose is within
             POSITION_TOLERANCE / ROTATION_TOLERANCE_DEG of the target
  contact    ends with "contact" (force above the limit) or "limit" (travel
             used up, or MIN_TCP_Z). PICK and LOWER need "contact": reaching the
             travel limit is not support, so no RELEASE follows it
  grip       OK only with GRIP OK and a live HOLD YES (see system_io)
  release    the controller reports IDLE after the pulse
  servos     every commanded angle reached, no status error
The side grip keeps the glass axis on the tool z axis, so FLIP is a 180 deg
turn of wrist 3 only (moveJ, checked by SafeControl against the ceiling):
the TCP and the glass centre stay in place. It needs a TCP offset without x/y.
The tool+glass sphere of the layout must cover the glass in every orientation.

STOP: pending jobs are invalidated first, then speedStop + stopL/stopJ
(async) and the servo stop; the stop is acknowledged only when the TCP speed
is below STOPPED_SPEED, no async move runs and the servos report stopped.
The vacuum is never released by a stop or a failure.
Requires: numpy, ur_rtde (via safe_motion.connect), pyserial.
"""

from dataclasses import dataclass
from math import radians, sin
from time import monotonic
from uuid import uuid4

import numpy as np

import bus_servos
from robot_watchdog import RobotWatchdog
from safe_motion import MotionRefused
from system_geometry import STATION_ACCESS, CollisionRefused
from system_io import SpongeProfile, SprayProfile
from system_model import Grip, Outcome, Reply, Result, State, Step, Telemetry
from system_teach import TeachError

# --- motion (values from pick_place_glasses.py, measured there) --------------------
MOVE_SPEED = 0.15            # m/s, moveL in free space
APPROACH_SPEED = 0.05        # m/s, near glasses, stations and outputs
MOVE_ACCEL = 0.6             # m/s^2 (low enough for the vacuum to hold the glass)
DESCEND_SPEED = 0.015        # m/s, contact moves
DESCEND_ACCEL = 0.2          # m/s^2
SPEED_CMD_TIME = 0.02        # s, speedL time, never 0 (C271A1 protective stop)
STOP_DECEL = 1.0             # m/s^2
FT_SETTLE = 0.2              # s after zeroFtSensor
SIDE_STANDOFF = 0.03         # m, cup to wall before the slow approach / after release
SIDE_MAX_PRESS = 0.006       # m past the expected wall
SIDE_CONTACT_FORCE = 5.0     # N, a free glass slides before this
APPROACH_GAP = 0.02          # m above the taught output pose before the contact move
MAX_OVERSHOOT = 0.015        # m past the taught output pose
PLACE_FORCE = 8.0            # N, glass touching the output surface
GRIP_DWELL = 0.5             # s, vacuum build-up before GRIP OK counts
GRIP_CONFIRM_TIMEOUT = 6.0   # s (firmware reports FAIL after 8 s)
MIN_GLASS_DISTANCE = 0.1     # m from the base axis, side grip refused closer

# --- new for the washing cycle: TO BE MEASURED on the cell ------------------------
FLIP_SPEED = 0.5             # rad/s, wrist 3 turn with the glass held
FLIP_ACCEL = 0.5             # rad/s^2
WIPE_SPEED = 0.03            # m/s along the taught wiping strokes
WIPE_MAX_FORCE = 15.0        # N, a stroke pressing harder than this fails
MAX_TCP_XY_OFFSET = 0.005    # m, TCP x/y offset allowed for the wrist-3 flip
MAX_APPROACH_TILT_DEG = 10   # deg, taught side-grip approach from horizontal
SPRAY_1_STROKES = 3          # pump strokes, first pass
SPRAY_2_STROKES = 2          # pump strokes, second pass
SPONGE_POSITIONS_DEG = (150.0, 210.0)   # deg, sponge servo goals per cycle
SPONGE_REPEATS = 5
SPONGE_SPEED = 1500          # steps/s

# --- checks ------------------------------------------------------------------------
POSITION_TOLERANCE = 0.003   # m, move finished at its target
ROTATION_TOLERANCE_DEG = 2.0 # deg
JOINT_TOLERANCE_DEG = 1.0    # deg, moveJ finished
MOVE_START_GRACE = 0.2       # s before "no async operation" can mean finished
PREPARED_TOLERANCE = 0.005   # m, robot may not move between validation and begin
STOPPED_SPEED = 0.002        # m/s, TCP speed counted as stopped


class ActionFailed(RuntimeError):
    pass


def rotation_matrix(rotvec):
    """Axis-angle (UR pose rotation) -> 3x3 matrix."""
    r = np.asarray(rotvec, dtype=float)
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K


def pose_error(actual, target):
    """(position error m, rotation error rad) between two UR poses."""
    position = float(np.linalg.norm(np.subtract(actual[:3], target[:3])))
    R = rotation_matrix(actual[3:]).T @ rotation_matrix(target[3:])
    angle = float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))
    return position, angle


def flip_joints(q):
    """Same joints with wrist 3 turned by 180 deg, towards zero (joint range +-360 deg)."""
    q = list(q)
    q[5] = q[5] - np.pi if q[5] > 0 else q[5] + np.pi
    return q


# --- actions -------------------------------------------------------------------------

class Action:
    label = ""

    def start(self, hw, now):
        pass

    def poll(self, hw, now):
        """None while running, a detail string when done; raise ActionFailed."""
        return ""


@dataclass
class MoveL(Action):
    pose: list
    speed: float
    label: str
    force_limit: float | None = None

    def start(self, hw, now):
        self.started = now
        hw.c.moveL(list(self.pose), self.speed, MOVE_ACCEL, True)

    def poll(self, hw, now):
        if self.force_limit is not None:
            force = float(np.linalg.norm(hw.r.getActualTCPForce()[:3]))
            if force > self.force_limit:
                raise ActionFailed(f"{self.label}: force {force:.1f} N above {self.force_limit:.0f} N")
        if now - self.started < MOVE_START_GRACE or hw.c.getAsyncOperationProgress() >= 0:
            return None
        position, angle = pose_error(hw.r.getActualTCPPose(), self.pose)
        if position > POSITION_TOLERANCE or angle > radians(ROTATION_TOLERANCE_DEG):
            raise ActionFailed(f"{self.label}: move ended {position * 1000:.1f} mm / "
                               f"{np.degrees(angle):.1f} deg from its target")
        return self.label


@dataclass
class FlipWrist(Action):
    label: str = "flip"

    def start(self, hw, now):
        self.started = now
        self.tcp = list(hw.r.getActualTCPPose())
        self.q = flip_joints(hw.r.getActualQ())
        hw.moving_joints = True
        hw.c.moveJ(self.q, FLIP_SPEED, FLIP_ACCEL, True)

    def poll(self, hw, now):
        if now - self.started < MOVE_START_GRACE or hw.c.getAsyncOperationProgress() >= 0:
            return None
        hw.moving_joints = False
        error = max(abs(a - b) for a, b in zip(hw.r.getActualQ(), self.q))
        if error > radians(JOINT_TOLERANCE_DEG):
            raise ActionFailed(f"flip ended {np.degrees(error):.1f} deg from the target joints")
        position, _ = pose_error(hw.r.getActualTCPPose(), self.tcp)
        if position > POSITION_TOLERANCE:
            raise ActionFailed(f"TCP moved {position * 1000:.1f} mm during the wrist flip")
        return "glass turned"


@dataclass
class Contact(Action):
    direction: tuple
    travel: float
    force_limit: float
    label: str

    def start(self, hw, now):
        d = np.asarray(self.direction, dtype=float)
        self.d = d / np.linalg.norm(d)
        hw.c.zeroFtSensor()
        self.settled = now + FT_SETTLE
        self.origin = None
        hw.contact_reason = None

    def poll(self, hw, now):
        if now < self.settled:
            return None
        pose = hw.r.getActualTCPPose()
        if self.origin is None:
            self.origin = np.asarray(pose[:3], dtype=float)
        force = float(np.linalg.norm(hw.r.getActualTCPForce()[:3]))
        travelled = float((np.asarray(pose[:3]) - self.origin) @ self.d)
        reason = None
        if force > self.force_limit:
            reason = "contact"
        elif travelled >= self.travel or (self.d[2] < 0 and pose[2] <= hw.min_tcp_z):
            reason = "limit"
        if reason is None:
            hw.c.speedL(list(self.d * DESCEND_SPEED) + [0.0, 0.0, 0.0], DESCEND_ACCEL, SPEED_CMD_TIME)
            return None
        hw.c.speedStop(STOP_DECEL)
        hw.contact_reason = reason
        if reason != "contact":
            raise ActionFailed(f"{self.label}: no contact within {self.travel * 1000:.0f} mm")
        return f"{self.label}: contact at {force:.1f} N"


@dataclass
class DeviceJob(Action):
    worker: str        # "gripper" or "servos"
    kind: str
    label: str
    params: dict | None = None

    def start(self, hw, now):
        self.job_id = f"{hw.command_id}:{self.kind}"
        getattr(hw, self.worker).submit(self.job_id, self.kind, **(self.params or {}))

    def poll(self, hw, now):
        result = getattr(hw, self.worker).result(self.job_id)
        if result is None:
            return None
        if result.outcome != Outcome.SUCCEEDED:
            raise ActionFailed(f"{self.label}: {result.outcome.value} {result.detail}")
        return result.detail or self.label


@dataclass
class WaitGrip(Action):
    label: str = "grip confirmation"

    def start(self, hw, now):
        self.started = now

    def poll(self, hw, now):
        grip = hw.gripper.snapshot()["grip"]
        if now - self.started >= GRIP_DWELL and grip == Grip.OK:
            return "GRIP OK"
        if grip == Grip.LOST:
            raise ActionFailed("glass lost while gripping")
        if now - self.started > GRIP_CONFIRM_TIMEOUT:
            raise ActionFailed(f"grip not confirmed ({grip.value}); vacuum left on")
        return None


@dataclass
class Plan:
    command: object
    start: list
    path: tuple
    actions: list


# --- adapter -------------------------------------------------------------------------

class HardwareDevices:
    def __init__(self, rtde_r, control, gripper, servos, teach, vision, min_tcp_z, max_tcp_z,
                 clock=monotonic, watchdog_factory=RobotWatchdog):
        self.r, self.c = rtde_r, control
        self.gripper, self.servos = gripper, servos
        self.teach, self.vision = teach, vision
        self.min_tcp_z, self.max_tcp_z = min_tcp_z, max_tcp_z
        self.clock = clock
        self.watchdog_factory = watchdog_factory
        self.watchdog = None
        self.robot_fault = ""
        self.config_problems = []
        self.workspace = None
        self.supported = False
        self.contact_reason = None
        self.moving_joints = False
        self.freedrive = False
        self.command_id = None
        self._prepared = None
        self._active = None      # [plan, index, action started]
        self._stop = None        # (stop_id, servo stop id)
        self.reload_teach()

    # --- lifecycle --------------------------------------------------------------------

    def reload_teach(self):
        try:
            self.teach.check_tcp_offset(self.c.getTCPOffset())
            offset = self.c.getTCPOffset()
            problems = self.teach.missing()
            if max(abs(offset[0]), abs(offset[1])) > MAX_TCP_XY_OFFSET:
                problems.append("TCP offset has x/y; the wrist-3 flip would move the glass")
            self.workspace = None if problems else self.teach.workspace(self.min_tcp_z, self.max_tcp_z)
        except TeachError as exc:
            problems = [str(exc)]
            self.workspace = None
        self.config_problems = problems

    def arm(self):
        """Arm the RTDE watchdog; call right before the owner loop starts."""
        self.watchdog = self.watchdog_factory(self.c)

    def service(self, now):
        """Once per owner-loop step, before supervisor.tick()."""
        try:
            alive = self.watchdog.kick() if self.watchdog is not None else False
            self.robot_fault = "" if alive else "robot stopped (watchdog, protective or emergency stop)"
            if alive and self.c.over_ceiling():
                self._stop_robot()
                self.robot_fault = "TCP above the ceiling"
        except Exception as exc:
            self.robot_fault = f"robot: {exc}"

    # --- DeviceAdapter ----------------------------------------------------------------

    def _robot_stopped(self):
        speed = float(np.linalg.norm(self.r.getActualTCPSpeed()[:3]))
        return speed < STOPPED_SPEED and self.c.getAsyncOperationProgress() < 0

    def telemetry(self, now):
        g, s = self.gripper.snapshot(), self.servos.snapshot()
        connected = self.r.isConnected() and self.c.isConnected() and g["connected"] and s["connected"]
        fault = (self.robot_fault or g["fault"] or s["fault"]
                 or ("freedrive is on" if self.freedrive else "")
                 or ("; ".join(s["errors"]["SPRAYER"] + s["errors"]["SPONGE"]))
                 or ("not ready: " + "; ".join(self.config_problems) if self.config_problems else ""))
        observed_at = min(now, g["observed_at"], s["observed_at"])
        return Telemetry(observed_at, connected, g["grip"], self.supported,
                         self._active is None and self._robot_stopped(), s["stopped"], fault)

    def observe(self, now):
        if self.vision is None:
            raise ValueError("hardware mode needs the overhead camera")
        return self.vision.observe(now)

    def validate_motion(self, command):
        self._prepared = None
        if self.workspace is None:
            raise CollisionRefused("not ready: " + "; ".join(self.config_problems))
        start = list(self.r.getActualTCPPose())
        path, actions = self._plan(command, start)
        checked = self.workspace.validate_path(path, command.step)
        self._prepared = Plan(command, start, checked, actions)

    def begin(self, command):
        plan = self._prepared
        self._prepared = None
        if self._active is not None or self._stop is not None:
            raise RuntimeError("a robot operation is already active")
        if plan is None or plan.command != command:
            raise CollisionRefused("motion plan must be validated before dispatch")
        moved, _ = pose_error(self.r.getActualTCPPose(), plan.start)
        if moved > PREPARED_TOLERANCE:
            raise CollisionRefused("robot moved since the plan was validated")
        if command.step in (Step.LIFT, Step.RETREAT):
            self.supported = False
        self.command_id = command.id
        self._active = [plan, 0, False]

    def poll(self, command_id, now):
        if self._active is None or self._active[0].command.id != command_id:
            return None
        plan = self._active[0]
        try:
            while self._active[1] < len(plan.actions):
                action = plan.actions[self._active[1]]
                if not self._active[2]:
                    action.start(self, now)
                    self._active[2] = True
                if action.poll(self, now) is None:
                    return None
                self._active[1] += 1
                self._active[2] = False
        except (ActionFailed, MotionRefused, RuntimeError, TeachError) as exc:
            self._active = None
            self._stop_robot()
            return Result(command_id, Outcome.FAILED, str(exc))
        self._active = None
        step = plan.command.step
        if step == Step.PICK:
            self.supported = True   # the glass still stands on the table
        elif step == Step.LOWER:
            self.supported = self.contact_reason == "contact"
        return Result(command_id, Outcome.SUCCEEDED)

    def _stop_robot(self):
        for stop in (lambda: self.c.speedStop(STOP_DECEL), lambda: self.c.stopL(STOP_DECEL, True),
                     lambda: self.c.stopJ(STOP_DECEL, True)):
            try:
                stop()
            except Exception:
                pass
        self.moving_joints = False

    def begin_stop(self, stop_id, now):
        self._prepared = None
        self._active = None      # late results of the old plan are never evaluated
        self.servos.request_stop(f"{stop_id}:servos")
        self.gripper.request_stop(f"{stop_id}:gripper")  # cancels jobs, keeps the vacuum
        self._stop_robot()
        self._stop = (stop_id, f"{stop_id}:servos", False)

    def poll_stop(self, stop_id, now):
        if self._stop is None or self._stop[0] != stop_id:
            return None
        _, servo_stop, servos_done = self._stop
        if not servos_done:
            result = self.servos.result(servo_stop)
            if result is None:
                return None
            if result.outcome != Outcome.SUCCEEDED:
                self._stop = None
                return Result(stop_id, Outcome.FAILED, f"servo stop: {result.detail}")
            self._stop = (stop_id, servo_stop, True)
        if self.robot_fault or self._robot_stopped():
            self._stop = None
            return Result(stop_id, Outcome.SUCCEEDED)
        return None

    # --- teaching ---------------------------------------------------------------------

    def capture(self, name):
        if self._active is not None or not self._robot_stopped():
            raise TeachError("the robot must stand still to capture a pose")
        self.teach.capture(name, self.r.getActualTCPPose(), self.r.getActualQ(), self.c.getTCPOffset())
        self.reload_teach()
        return f"captured {name}"

    def set_freedrive(self, on):
        if on:
            if self._active is not None or not self._robot_stopped():
                raise TeachError("freedrive only with the robot standing still")
            self.c.teachMode()
        else:
            self.c.endTeachMode()
        self.freedrive = on
        return "freedrive on: move the arm by hand" if on else "freedrive off"

    def teach_gripper(self, action):
        self.gripper.submit(f"teach-{uuid4().hex}", action)
        return f"gripper {action} sent (see gripper state)"

    def close(self):
        """Stop motion and the control script, then close the serial workers."""
        self._stop_robot()
        for step in (self.c.endTeachMode if self.freedrive else None, self.c.stopScript):
            try:
                if step is not None:
                    step()
            except Exception:
                pass
        self.servos.close()
        self.gripper.close()

    def panel_status(self):
        return {"teach": self.teach.status(), "config_problems": list(self.config_problems),
                "gripper": {k: (v.value if isinstance(v, Grip) else v)
                            for k, v in self.gripper.snapshot().items() if k != "observed_at"},
                "servos": {k: v for k, v in self.servos.snapshot().items() if k != "observed_at"},
                "robot_fault": self.robot_fault, "freedrive": self.freedrive}

    # --- plans ------------------------------------------------------------------------

    def _transfer(self, start, destination, speed=MOVE_SPEED):
        """Up to the crossing height, across, down; orientation turns while crossing."""
        h = self.workspace.transport_height(start[:3], destination[:3])
        up = [start[0], start[1], h, *start[3:]]
        over = [destination[0], destination[1], h, *destination[3:]]
        path = [start[:3], up[:3], over[:3], destination[:3]]
        actions = [MoveL(up, MOVE_SPEED, "up to crossing height"),
                   MoveL(over, MOVE_SPEED, "across"),
                   MoveL(list(destination), speed, "down")]
        return path, actions

    def _plan(self, command, start):
        step, teach = command.step, self.teach
        if step == Step.PICK:
            return self._plan_pick(command, start)
        if step in (Step.LIFT, Step.RETREAT):
            path, actions = [start[:3]], []
            current = list(start)
            if step == Step.RETREAT:
                # Back off along the cup axis first so the cup does not drag the glass.
                a = rotation_matrix(start[3:])[:, 2]
                current = [*(np.asarray(start[:3]) - a * SIDE_STANDOFF), *start[3:]]
                path.append(current[:3])
                actions.append(MoveL(current, APPROACH_SPEED, "back off the glass"))
            h = self.workspace.transport_height(current[:3])
            up = [current[0], current[1], h, *start[3:]]
            path.append(up[:3])
            actions.append(MoveL(up, MOVE_SPEED, "lift"))
            return path, actions
        if step == Step.FLIP:
            flip = teach.pose("flip")
            path, actions = self._transfer(start, flip)
            return path, actions + [FlipWrist()]
        if step.value.startswith("TO_") and step != Step.TO_OUTPUT:
            station = self._station(step)
            approach, work = teach.pose(f"{station}.approach"), teach.pose(f"{station}.work")
            path, actions = self._transfer(start, approach, APPROACH_SPEED)
            return path + [work[:3]], actions + [MoveL(work, APPROACH_SPEED, f"into {station}")]
        if step.value.startswith("LEAVE_"):
            station = self._station(step)
            approach = teach.pose(f"{station}.approach")
            h = self.workspace.transport_height(approach[:3])
            up = [approach[0], approach[1], h, *approach[3:]]
            return ([start[:3], approach[:3], up[:3]],
                    [MoveL(approach, APPROACH_SPEED, f"out of {station}"), MoveL(up, MOVE_SPEED, "up")])
        if step in (Step.SPRAY_1, Step.SPRAY_2):
            strokes = SPRAY_1_STROKES if step == Step.SPRAY_1 else SPRAY_2_STROKES
            profile = SprayProfile(bus_servos.SPRAYER_REST_DEG, bus_servos.SPRAYER_PRESS_DEG, strokes,
                                   bus_servos.SPRAY_HOLD, bus_servos.SPRAY_PERIOD, bus_servos.SPRAYER_SPEED)
            return [start[:3]], [DeviceJob("servos", "spray", "spray", {"profile": profile})]
        if step == Step.SPONGE:
            profile = SpongeProfile(SPONGE_POSITIONS_DEG, SPONGE_REPEATS, SPONGE_SPEED)
            return [start[:3]], [DeviceJob("servos", "sponge", "sponge", {"profile": profile})]
        if step == Step.WIPE:
            strokes = [teach.pose(f"wiper.stroke.{i}") for i in range(1, teach.wipe_strokes + 1)]
            work = teach.pose("wiper.work")
            moves = [MoveL(p, WIPE_SPEED, f"wipe {i + 1}", WIPE_MAX_FORCE) for i, p in enumerate(strokes)]
            moves.append(MoveL(work, WIPE_SPEED, "wipe end", WIPE_MAX_FORCE))
            return [start[:3], *(p[:3] for p in strokes), work[:3]], moves
        if step == Step.TO_OUTPUT:
            place = teach.pose(f"output.{command.slot_id}")
            above = [place[0], place[1], place[2] + APPROACH_GAP, *place[3:]]
            return self._transfer(start, above, APPROACH_SPEED)
        if step == Step.LOWER:
            travel = APPROACH_GAP + MAX_OVERSHOOT
            bottom = [start[0], start[1], start[2] - travel]
            return [start[:3], bottom], [Contact((0, 0, -1), travel, PLACE_FORCE, "place")]
        if step == Step.RELEASE:
            return [start[:3]], [DeviceJob("gripper", "release", "release")]
        if step == Step.OBSERVE:
            return self._transfer(start, teach.pose("observe"))
        raise CollisionRefused(f"no hardware plan for {step.value}")

    @staticmethod
    def _station(step):
        return STATION_ACCESS[step][0]

    def _plan_pick(self, command, start):
        side = self.teach.pose("side_grip")
        a = rotation_matrix(side[3:])[:, 2]
        if abs(a[2]) > sin(radians(MAX_APPROACH_TILT_DEG)):
            raise TeachError("taught side_grip is not horizontal enough")
        glass = np.array([command.target.x, command.target.y])
        if np.linalg.norm(glass) < MIN_GLASS_DISTANCE:
            raise CollisionRefused("glass is too close to the robot base for a side grip")
        d = a[:2] / np.linalg.norm(a[:2])
        radius = float(self.teach.data["glass"]["radius"])
        wall = np.array([*(glass - d * radius), side[2]])
        standoff = wall - a * SIDE_STANDOFF
        press = standoff + a * (SIDE_STANDOFF + SIDE_MAX_PRESS)
        path, actions = self._transfer(start, [*standoff, *side[3:]], APPROACH_SPEED)
        actions += [DeviceJob("gripper", "grip", "vacuum on"),
                    Contact(tuple(a), SIDE_STANDOFF + SIDE_MAX_PRESS, SIDE_CONTACT_FORCE, "pick"),
                    WaitGrip()]
        return path + [list(press)], actions



TEACH_STATES = (State.READY, State.STOPPED, State.FAULT, State.COMPLETED)


def teach_actions(hw, supervisor):
    """Owner-thread actions for the panel's teach mode; refused during a batch."""
    def guarded(function):
        def run(payload):
            if supervisor.state not in TEACH_STATES:
                return Reply(False, f"teaching is refused in {supervisor.state.value}")
            return Reply(True, function(payload))
        return run
    return {"teach_capture": guarded(lambda p: hw.capture(p["name"])),
            "teach_freedrive": guarded(lambda p: hw.set_freedrive(p["on"])),
            "teach_gripper": guarded(lambda p: hw.teach_gripper(p["action"]))}


def connect(slot_ids, vision, teach_path=None):
    """Open robot, gripper and servos. THE ARM CAN MOVE from here on.

    Ports open before the loop is armed (the gripper port may reset the
    XIAO: only do this with no glass held). Limits come from SafeControl
    and follow_april_tag, never from the layout file.
    """
    import safe_motion
    from follow_april_tag import IP, MIN_TCP_Z   # loads hand_eye.npz; no connection
    from system_io import GripperWorker, ServoWorker, open_servo_bus, open_suction
    from system_teach import TeachStore

    teach = TeachStore(slot_ids, teach_path)
    gripper = GripperWorker(open_suction)
    servos = ServoWorker(open_servo_bus, bus_servos.SPRAYER_ID, bus_servos.ROTATOR_ID,
                         bus_servos.SPRAYER_REST_DEG)
    gripper.start()
    try:
        servos.start()
        try:
            r, c = safe_motion.connect(IP)
        except Exception:
            servos.close()
            raise
    except Exception:
        gripper.close()
        raise
    return HardwareDevices(r, c, gripper, servos, teach, vision, MIN_TCP_Z, safe_motion.MAX_TCP_Z)
