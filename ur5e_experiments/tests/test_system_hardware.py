"""Hardware adapter against fakes: plans, postconditions, STOP and teaching.

The layout below is test data only (not the real cell). SafeControl wraps the
fake robot, so ceiling checks are the real ones.
"""

import json
from math import pi

import numpy as np
import pytest

from fake_robot import SIDE, FakeGripper, FakeRobot, FakeServos
from robot_watchdog import RobotWatchdog
from safe_motion import MAX_TCP_Z, SafeControl
from system_batch import OutputSlot
from system_controller import Supervisor
from system_hardware import (GRIP_CONFIRM_TIMEOUT, HardwareDevices, flip_joints, pose_error,
                             rotation_matrix, teach_actions)
from system_model import GlassTarget, Grip, Scene, SlotState, State, Step
from system_settings import TICK_PERIOD
from system_simulator import SimulationClock
from system_teach import TeachError, TeachStore, required_poses

Q = [0, -1.57, 1.57, -1.57, -1.57, 0.5]
STATION_X = {"sprayer": 0.3, "sponge": 0.6, "wiper": 0.9}
GLASS = (0.4, -0.3)
OUTPUT = (0.6, -0.4, 0.06)


def teach_data():
    poses = {}

    def put(name, xyz):
        poses[name] = {"pose": [*xyz, *SIDE], "q": Q, "captured": "test"}

    put("observe", (0.3, -0.5, 0.35))
    put("side_grip", (0.3, -0.3, 0.05))
    put("flip", (0.4, -0.1, 0.35))
    stations = {}
    for name, x in STATION_X.items():
        put(f"{name}.approach", (x, 0.1, 0.15))
        put(f"{name}.work", (x, 0.3, 0.15))
        stations[name] = {"low": [x - 0.1, 0.25, 0.0], "high": [x + 0.1, 0.45, 0.2],
                          "corridor": {"low": [x - 0.11, 0.04, 0.04], "high": [x + 0.11, 0.41, 0.26]}}
    put("wiper.stroke.1", (0.895, 0.3, 0.15))
    put("wiper.stroke.2", (0.905, 0.3, 0.15))
    put("output.tag-1", OUTPUT)
    return {"version": 1, "tcp_offset": [0, 0, 0.2, 0, 0, 0], "poses": poses, "glass": {"radius": 0.03},
            "layout": {"tool_radius": 0.08, "clearance": 0.02, "stations": stations}}


def contact_force(pose):
    x, y, z = pose[:3]
    force = 0.0
    wall = GLASS[0] - 0.03
    if abs(y - GLASS[1]) < 0.02 and z < 0.1 and x > wall:
        force += 5000 * (x - wall)
    if abs(x - OUTPUT[0]) < 0.02 and abs(y - OUTPUT[1]) < 0.02 and z < OUTPUT[2]:
        force += 5000 * (OUTPUT[2] - z)
    return force


class FakeVision:
    def __init__(self, targets):
        self.targets = tuple(targets)
        self.sequence = 0

    def observe(self, now):
        self.sequence += 1
        return Scene(self.sequence, now, self.targets)


class System:
    def __init__(self, tmp_path, data=None, holds=True, force=True):
        path = tmp_path / "teach.json"
        path.write_text(json.dumps(data or teach_data()), encoding="utf-8")
        self.clock = SimulationClock()
        self.robot = FakeRobot()
        if force:
            self.robot.force_fn = contact_force
        self.gripper = FakeGripper(self.clock, holds)
        self.servos = FakeServos(self.clock)
        self.store = TeachStore(["tag-1"], str(path))
        self.hw = HardwareDevices(self.robot, SafeControl(self.robot, self.robot), self.gripper, self.servos,
                                  self.store, FakeVision([GlassTarget("g1", *GLASS)]), -0.05, MAX_TCP_Z,
                                  clock=self.clock, watchdog_factory=RobotWatchdog)
        self.hw.arm()
        self.sup = Supervisor(self.hw, [OutputSlot("tag-1", SlotState.FREE)], self.clock)

    def tick(self):
        self.clock.advance(TICK_PERIOD)
        self.robot.advance(TICK_PERIOD)
        self.hw.service(self.clock())
        self.sup.tick()

    def run_until(self, predicate, limit=60000):
        for _ in range(limit):
            if predicate():
                return
            self.tick()
        pytest.fail(f"not reached: {self.sup.status()}")

    def run_to(self, step):
        self.run_until(lambda: self.sup.command is not None and self.sup.command.step == step)


# --- pure helpers -------------------------------------------------------------------

def test_rotation_and_pose_error():
    assert np.allclose(rotation_matrix(SIDE)[:, 2], [1, 0, 0], atol=1e-12)
    position, angle = pose_error([0, 0, 0, *SIDE], [0.003, 0.004, 0, 0, pi / 2 + 0.01, 0])
    assert position == pytest.approx(0.005) and angle == pytest.approx(0.01)


def test_flip_turns_only_wrist_3_towards_zero():
    assert flip_joints(Q)[:5] == Q[:5]
    assert flip_joints(Q)[5] == pytest.approx(0.5 - pi)
    assert flip_joints([0, 0, 0, 0, 0, -1.0])[5] == pytest.approx(pi - 1.0)


# --- full cycle ---------------------------------------------------------------------

def test_full_cycle_on_fakes_places_the_glass_and_checks_every_postcondition(tmp_path):
    s = System(tmp_path)
    assert s.sup.start("s").accepted, s.sup.status()
    s.run_until(lambda: s.sup.state in (State.COMPLETED, State.FAULT))
    assert s.sup.state == State.COMPLETED, s.sup.fault
    assert s.sup.outputs.slots["tag-1"].state == SlotState.OCCUPIED
    assert s.gripper.jobs == ["grip", "release"]
    assert [kind for kind, _ in s.servos.jobs] == ["spray", "sponge", "spray"]
    strokes = [profile.strokes for kind, profile in s.servos.jobs if kind == "spray"]
    assert strokes == [3, 2]
    kinds = [call[0] for call in s.robot.calls]
    assert kinds.count("moveJ") == 1 and "speedL" in kinds
    assert all(call[1][2] <= MAX_TCP_Z for call in s.robot.calls if call[0] == "moveL")


def test_lowering_without_contact_never_releases(tmp_path):
    s = System(tmp_path)
    s.robot.force_fn = lambda pose: contact_force(pose) if pose[1] > -0.35 else 0.0  # nothing at the output
    s.sup.start("s")
    s.run_until(lambda: s.sup.state in (State.FAULT, State.STOPPED, State.COMPLETED))
    assert s.sup.state == State.FAULT and "no contact" in s.sup.fault
    assert "release" not in s.gripper.jobs
    assert s.sup.outputs.slots["tag-1"].state == SlotState.UNKNOWN


def test_unconfirmed_grip_fails_the_pick_and_keeps_the_vacuum(tmp_path):
    s = System(tmp_path, holds=False)
    s.sup.start("s")
    s.run_until(lambda: s.sup.state == State.FAULT, limit=int((GRIP_CONFIRM_TIMEOUT + 40) / TICK_PERIOD))
    assert "grip not confirmed" in s.sup.fault
    assert s.gripper.jobs == ["grip"]
    assert not any(call[0] == "moveL" and call[1][2] > 0.1 for call in s.robot.calls[-3:])


def test_no_contact_at_the_glass_fails_the_pick(tmp_path):
    s = System(tmp_path, force=False)
    s.sup.start("s")
    s.run_until(lambda: s.sup.state == State.FAULT)
    assert "pick: no contact" in s.sup.fault


def test_move_ending_away_from_its_target_fails(tmp_path):
    s = System(tmp_path)
    s.robot.stop_short = 0.01
    s.sup.start("s")
    s.run_until(lambda: s.sup.state == State.FAULT)
    assert "from its target" in s.sup.fault


def test_station_servo_failure_stops_the_batch(tmp_path):
    s = System(tmp_path)
    s.servos.fail = "sponge"
    s.sup.start("s")
    s.run_until(lambda: s.sup.state == State.FAULT)
    assert "sponge" in s.sup.fault and s.gripper.grip == Grip.OK


def test_stop_during_a_move_stops_async_and_keeps_the_vacuum(tmp_path):
    s = System(tmp_path)
    s.sup.start("s")
    s.run_to(Step.TO_SPONGE)
    s.tick()
    s.sup.request_stop()
    s.run_until(lambda: s.sup.state != State.STOPPING)
    assert s.sup.state == State.STOPPED
    assert ("stopL", True) in s.robot.calls and s.servos.stops and s.gripper.stops
    assert s.gripper.grip == Grip.OK and "release" not in s.gripper.jobs


def test_watchdog_trip_faults_the_batch(tmp_path):
    s = System(tmp_path)
    s.sup.start("s")
    s.run_to(Step.LIFT)
    s.robot.program_running = False  # controller stopped the script: loop stalled
    s.run_until(lambda: s.sup.state == State.FAULT)
    assert "watchdog" in s.sup.fault


def test_robot_moved_after_validation_is_refused(tmp_path):
    s = System(tmp_path)
    s.robot.pose = [0.3, -0.5, 0.35, *SIDE]
    from system_model import Command
    command = Command("c", "b", GlassTarget("g1", *GLASS), "tag-1", Step.OBSERVE, 0.0)
    s.hw.validate_motion(command)
    s.robot.pose[0] += 0.01
    with pytest.raises(ValueError, match="moved"):
        s.hw.begin(command)


def test_path_through_a_station_is_refused_before_motion(tmp_path):
    data = teach_data()
    data["poses"]["observe"]["pose"][:3] = [0.3, 0.35, 0.1]   # inside the sprayer
    s = System(tmp_path, data)
    from system_model import Command
    command = Command("c", "b", GlassTarget("g1", *GLASS), "tag-1", Step.OBSERVE, 0.0)
    with pytest.raises(ValueError):
        s.hw.validate_motion(command)
    assert not s.robot.calls


# --- readiness and teaching ---------------------------------------------------------

def test_incomplete_teach_file_refuses_start(tmp_path):
    data = teach_data()
    del data["poses"]["sponge.work"]
    s = System(tmp_path, data)
    reply = s.sup.start("s")
    assert not reply.accepted and "sponge.work" in reply.message
    assert not s.robot.calls


def test_changed_tcp_offset_refuses_start(tmp_path):
    s = System(tmp_path)
    s.robot.tcp_offset = [0, 0, 0.21, 0, 0, 0]
    s.hw.reload_teach()
    assert not s.sup.start("s").accepted


def test_tcp_offset_with_xy_blocks_the_wrist_flip(tmp_path):
    data = teach_data()
    data["tcp_offset"] = [0.02, 0, 0.2, 0, 0, 0]
    s = System(tmp_path, data)
    s.robot.tcp_offset = [0.02, 0, 0.2, 0, 0, 0]
    s.hw.reload_teach()
    assert any("x/y" in p for p in s.hw.config_problems)


def test_freedrive_blocks_start(tmp_path):
    s = System(tmp_path)
    s.hw.set_freedrive(True)
    assert not s.sup.start("s").accepted
    s.hw.set_freedrive(False)
    assert s.sup.start("s2").accepted


def test_capture_writes_the_file_atomically_and_only_outside_batches(tmp_path):
    s = System(tmp_path)
    actions = teach_actions(s.hw, s.sup)
    s.robot.pose = [0.31, -0.5, 0.36, *SIDE]
    assert actions["teach_capture"]({"name": "observe"}).accepted
    saved = json.loads((tmp_path / "teach.json").read_text(encoding="utf-8"))
    assert saved["poses"]["observe"]["pose"][0] == pytest.approx(0.31)
    assert not (tmp_path / "teach.json.tmp").exists()
    with pytest.raises(TeachError):
        actions["teach_capture"]({"name": "../evil"})
    s.sup.start("s")
    s.tick()
    assert not actions["teach_capture"]({"name": "observe"}).accepted


def test_wipe_strokes_are_taught_in_order():
    store = TeachStore(["tag-1"], "does-not-exist.json")
    assert store.allowed("wiper.stroke.1") and not store.allowed("wiper.stroke.2")
    assert store.allowed("output.tag-1") and not store.allowed("output.tag-9")
    assert "output.tag-1" in required_poses(["tag-1"], 1)
    assert any("glass.radius" in p for p in store.missing())


# --- test flip (teach mode) ---------------------------------------------------------

def untaught():
    data = teach_data()
    data["poses"], data["layout"]["stations"] = {}, {}
    return data


def test_test_flip_turns_wrist_3_in_place_without_a_teach_file_and_back(tmp_path):
    s = System(tmp_path, untaught())
    actions = teach_actions(s.hw, s.sup)
    assert s.hw.config_problems and not s.sup.start("s").accepted
    tcp = s.robot.getActualTCPPose()
    assert actions["teach_flip"]({}).accepted
    s.run_until(lambda: "done" in s.hw.test_status, limit=100)
    assert s.robot.q[5] == pytest.approx(0.5 - pi) and s.robot.q[:5] == Q[:5]
    assert s.robot.getActualTCPPose() == tcp
    assert actions["teach_flip"]({}).accepted
    s.run_until(lambda: s.hw._test is None, limit=100)
    assert s.robot.q[5] == pytest.approx(0.5)


def test_stop_cancels_the_test_flip(tmp_path):
    s = System(tmp_path, untaught())
    s.hw.test_flip()
    assert not s.hw.telemetry(s.clock()).robot_stopped
    s.sup.request_stop()
    assert ("stopJ", True) in s.robot.calls and s.hw._test is None
    assert s.hw.test_status == "test flip stopped"


def test_test_flip_is_refused_in_freedrive_with_xy_offset_and_during_a_batch(tmp_path):
    s = System(tmp_path, untaught())
    s.hw.set_freedrive(True)
    with pytest.raises(TeachError, match="freedrive"):
        s.hw.test_flip()
    s.hw.set_freedrive(False)
    s.robot.tcp_offset = [0.02, 0, 0.2, 0, 0, 0]
    with pytest.raises(TeachError, match="x/y"):
        s.hw.test_flip()
    assert not any(call[0] == "moveJ" for call in s.robot.calls)
    s = System(tmp_path)
    s.sup.start("s")
    s.tick()
    assert not teach_actions(s.hw, s.sup)["teach_flip"]({}).accepted


def test_freedrive_and_capture_wait_for_the_test_flip(tmp_path):
    s = System(tmp_path, untaught())
    s.hw.test_flip()
    with pytest.raises(TeachError):
        s.hw.set_freedrive(True)
    with pytest.raises(TeachError):
        s.hw.capture("observe")
