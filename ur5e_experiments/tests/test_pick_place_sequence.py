"""pick_place_glasses.SequenceTask on the fake robot: step checks, the flip cycle."""

from math import pi
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import pick_place_glasses as ppg
from fake_robot import FakeRobot
from safe_motion import MAX_TCP_Z, SafeControl

DOWN = [pi, 0.0, 0.0]        # axis-angle: tool z along base -z
DT = 0.02                    # s per loop step


class FakeSuction:
    def __init__(self, robot, lose_on_flip=False):
        self.robot = robot
        self.lose_on_flip = lose_on_flip
        self.calls = []
        self.result = None

    def grip(self):
        self.calls.append("grip")
        self.result = "OK"
        return True

    def release(self):
        self.calls.append("release")
        self.result = None
        return True

    def grip_result(self):
        # The flip is the moveJ that turns wrist 3 away from the start (the turns keep it)
        if self.lose_on_flip and any(call[0] == "moveJ" and abs(call[1][5] - 0.5) > 1
                                     for call in self.robot.calls):
            return "LOST"
        return self.result


@pytest.fixture(autouse=True)
def untaught_arm_configuration(monkeypatch):
    # Turns anchored to the start joints unless a test teaches its own
    monkeypatch.setattr(ppg, "SIDE_GRIP_Q", None)
    monkeypatch.setattr(ppg, "TOP_GRIP_Q", None)
    # The fake robot's joints are no real arm: its self-collision gap is meaningless
    # (tested separately with the real joints below)
    monkeypatch.setattr(ppg, "self_gap", lambda q, T_flange_tcp: 0.05)


@pytest.fixture
def clock(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(ppg, "time", SimpleNamespace(time=lambda: now[0]))
    return now


def run(task, robot, clock, limit=20000):
    for _ in range(limit):
        task.update()
        if task.done:
            return task.status
        clock[0] += DT
        robot.advance(DT)
    pytest.fail(f"task did not finish: {task.status}")


class FakeSprayer:
    """bus_servos.Sprayer: runs for a few loop steps after start()."""

    def __init__(self, polls=3, error=None):
        self.polls = polls
        self.fail_with = error
        self.started = []
        self.stop_requests = 0
        self.strokes = 0
        self.error = None
        self._left = 0

    def start(self, period=None, count=None):
        self.started.append(count)
        self._left = self.polls
        self.error = None

    @property
    def running(self):
        if self._left > 0:
            self._left -= 1
            self.strokes = self.polls - self._left
            if self._left == 0:
                self.error = self.fail_with
        return self._left > 0

    def request_stop(self):
        self.stop_requests += 1


def make_task(sequence, lose_on_flip=False, sprayer=None):
    robot = FakeRobot(pose=(0.3, -0.3, 0.3, *DOWN))
    suction = FakeSuction(robot, lose_on_flip)
    glass = SimpleNamespace(x=0.35, y=-0.45, diameter=0.07)
    task = ppg.SequenceTask(robot, SafeControl(robot, robot), suction, glass,
                            place_xy=(0.1, -0.5), table_z=0.0, glass_height=0.075,
                            sequence=sequence, sprayer=FakeSprayer() if sprayer is None else sprayer)
    return task, robot, suction


@pytest.mark.parametrize("sequence, problem", [
    ([], "empty"),
    (["pick_side", "flip", "twirl", "place"], "unknown"),
    (["pick_top", "flip", "place"], "needs a side grip"),
    (["pick_side", "pick_top", "place"], "needs nothing held"),
    (["place"], "needs a glass held"),
    (["pick_side", "flip"], "still held"),
])
def test_bad_sequences_are_refused(sequence, problem):
    with pytest.raises(ppg.TaskFailed, match=problem):
        ppg.check_sequence(sequence)


def test_default_and_plain_sequences_are_valid():
    for sequence in (ppg.SEQUENCE, ["pick_side", "place"], ["pick_top", "place"],
                     ["pick_side", "flip", "set_down", "pick_side", "flip", "place"]):
        ppg.check_sequence(sequence)


def test_bad_sequence_is_refused_before_anything_moves(clock):
    task, robot, suction = make_task(["pick_top", "flip", "place"])
    run(task, robot, clock)
    assert "needs a side grip" in task.status
    assert not [call for call in robot.calls if call[0] in ("moveL", "moveJ", "speedL")]
    assert suction.calls == []


def test_flip_sequence_regrips_from_the_top_and_places_on_the_tag(clock):
    task, robot, suction = make_task(ppg.SEQUENCE)
    q5 = robot.q[5]
    start_q = list(robot.q)
    assert run(task, robot, clock) == "done"
    assert suction.calls == ["grip", "release", "grip", "release"]
    # Tool turns are joint moves to IK solutions near the start joints: sideways, the
    # flip turns wrist 3 by 180 deg, down (wrist 3 back), back to the start orientation
    wrist = [call[1][5] for call in robot.calls if call[0] == "moveJ"]
    assert wrist[:3] == [pytest.approx(q5), pytest.approx(q5 - pi), pytest.approx(q5)]
    # then the stations: sprayer, dryer (entry, DRY_SWINGS x right-left, middle), back to
    # the joints before the sprayer, and the last turn back to the start orientation
    dry_q5 = ppg.DRY_Q[5]
    swing = ppg.DRY_FIRST_DIRECTION * np.radians(ppg.DRY_ANGLE_DEG)
    assert wrist[3] == pytest.approx(ppg.SPRAY_Q[5])
    assert wrist[4] == pytest.approx(dry_q5)
    assert wrist[5:5 + 2 * ppg.DRY_SWINGS] == [pytest.approx(dry_q5 + swing), pytest.approx(dry_q5 - swing)] * ppg.DRY_SWINGS
    assert wrist[5 + 2 * ppg.DRY_SWINGS:] == [pytest.approx(dry_q5), pytest.approx(q5), pytest.approx(q5)]
    assert len(robot.ik_calls) == 5
    assert task.sprayer.started == [ppg.SPRAY_STROKES] and task.sprayer.stop_requests == 1
    turn_anchors = [qnear for i, (qnear, *_) in enumerate(robot.ik_calls) if i not in (2, 3)]   # stations
    assert all(qnear == pytest.approx(start_q, abs=1e-4) for qnear in turn_anchors)
    assert all(None not in errors for _, *errors in robot.ik_calls)
    assert all(call[1][2] <= MAX_TCP_Z for call in robot.calls if call[0] == "moveL")
    # The top pick goes to where the cup left the glass, not the camera position: with
    # no force the side push ran SIDE_MAX_PRESS past the expected wall
    approach = cv2.Rodrigues(np.asarray(ppg.SIDE_GRIP_ROTATION))[0][:, 2]
    expected = np.array([0.35, -0.45]) + approach[:2] * ppg.SIDE_MAX_PRESS
    top_z = 0.075 + ppg.APPROACH_GAP
    top_pick, lower_on_tag = [call[1] for call in robot.calls if call[0] == "moveL"
                              and call[1][2] == pytest.approx(top_z) and np.allclose(call[1][3:], ppg.TOP_GRIP_ROTATION)]
    # the cup TOP_GRIP_OFFSET off the glass axis, picking and placing (the glass on the tag)
    offset = np.asarray(ppg.TOP_GRIP_OFFSET)
    assert top_pick[:2] == pytest.approx(expected + offset, abs=0.001)
    assert lower_on_tag[:2] == pytest.approx(np.add([0.1, -0.5], offset))
    # The glass ends up on the tag, the robot back at the start pose
    assert task.glass_xy == pytest.approx([0.1, -0.5], abs=0.002)
    assert robot.pose[:3] == pytest.approx([0.3, -0.3, 0.3], abs=1e-6)


def test_flipped_glass_is_set_down_with_the_tip_at_its_other_end(clock):
    task, robot, suction = make_task(["pick_side", "flip", "set_down"])
    lowest = []
    real_speedL = robot.speedL

    def speedL(xd, accel, time):
        lowest.append(robot.pose[2])
        real_speedL(xd, accel, time)

    robot.speedL = speedL
    assert run(task, robot, clock) == "done"
    # Side grip 35 mm above the table; flipped, the tip is 75 - 35 = 40 mm above it
    # and the put-down pushes at most MAX_OVERSHOOT below that
    assert min(lowest) >= 0.040 - ppg.MAX_OVERSHOOT - 0.001
    assert task.hang == pytest.approx(0.040)


def test_glass_lost_during_the_flip_stops_the_wrist(clock):
    task, robot, suction = make_task(ppg.SEQUENCE, lose_on_flip=True)
    run(task, robot, clock)
    assert "glass lost" in task.status
    assert ("stopJ", False) in robot.calls
    assert suction.calls == ["grip"]              # vacuum left on for the operator


def test_forward_kinematics_always_gets_the_active_tcp_offset(clock, monkeypatch):
    # ur_rtde 1.6.5 reads a missing offset from stale input registers (safe_motion.py)
    monkeypatch.setattr(ppg, "TOP_GRIP_ROTATION", None)
    task, robot, suction = make_task(ppg.SEQUENCE)
    robot.pose[3:] = ppg.SIDE_GRIP_ROTATION     # not pointing down: pick_top uses FK(START_Q)
    fake_fk = robot.getForwardKinematics

    def fk(q=None, tcp_offset=None):
        pose = fake_fk(q, tcp_offset)
        return pose[:3] + DOWN if list(q) == ppg.START_Q else pose

    robot.getForwardKinematics = fk
    assert run(task, robot, clock) == "done"
    assert robot.fk_offsets and all(offset == robot.tcp_offset for offset in robot.fk_offsets)


def test_wound_up_joint_is_reported():
    assert ppg.joint_near_limit(np.radians([-110, -103, -125, 360.8, 104, -148])) == ("wrist 1", pytest.approx(360.8))
    assert ppg.joint_near_limit(np.radians([-110, -103, -125, 0.8, 104, -148])) is None
    assert ppg.joint_near_limit(ppg.START_Q) is None


def test_turns_use_the_taught_arm_configuration(clock, monkeypatch):
    side_q = [0.1, -1.2, 1.9, -0.9, -1.1, -1.4]
    top_q = [0.2, -1.3, 1.8, -1.6, -1.5, 0.3]
    monkeypatch.setattr(ppg, "SIDE_GRIP_Q", side_q)
    monkeypatch.setattr(ppg, "TOP_GRIP_Q", top_q)
    task, robot, suction = make_task(ppg.SEQUENCE)
    start_q = list(robot.q)
    assert run(task, robot, clock) == "done"
    # sideways, down, back to the start orientation
    # turn sideways, configuration check before going down, the same for the top grip,
    # spray, dry, back to the start orientation
    anchors = [list(qnear) for qnear, *_ in robot.ik_calls]
    assert len(anchors) == 7
    assert [anchors[i] for i in (0, 1, 2, 3, 6)] == [pytest.approx(side_q), pytest.approx(side_q),
                                                      pytest.approx(top_q), pytest.approx(top_q),
                                                      pytest.approx(start_q)]
    # The spray pose: SPRAY_Q shifted by whole turns only
    turns = (np.subtract(anchors[4], ppg.SPRAY_Q)) / (2 * pi)
    assert turns == pytest.approx(np.round(turns))


def test_a_wound_up_wrist_is_unwound_at_the_start(clock, monkeypatch):
    side_q = [0.1, -1.2, 1.9, -1.6, -1.1, -1.4]
    monkeypatch.setattr(ppg, "SIDE_GRIP_Q", side_q)
    task, robot, suction = make_task(["pick_side", "place"])
    robot.q[3] += 2 * pi                       # wrist 1 at 270 deg: same pose, one turn wound
    wound_q = list(robot.q)
    assert run(task, robot, clock) == "done"
    # Before any grip: wrist 2 parked (tool out along the wrist-1 axis), wrist 1 a full
    # turn back, wrist 2 back, each its own moveJ
    moves = [call[1] for call in robot.calls if call[0] == "moveJ"][:3]
    assert moves[0] == pytest.approx(wound_q[:4] + [0.0] + wound_q[5:])
    assert moves[1] == pytest.approx(wound_q[:3] + [wound_q[3] - 2 * pi, 0.0] + wound_q[5:])
    assert moves[2] == pytest.approx(wound_q[:3] + [wound_q[3] - 2 * pi] + wound_q[4:])
    # ... so the side turn is anchored to the taught joints as they are
    assert list(robot.ik_calls[0][0]) == pytest.approx(side_q)
    assert suction.calls == ["grip", "release"]


def test_a_turn_near_a_joint_limit_takes_the_unwound_way(clock):
    task, robot, suction = make_task(["pick_top", "place"])
    task.start = robot.getActualTCPPose()
    task.start_q = list(robot.q)
    task.min_self_gap = -1.0
    robot.q[3] = np.radians(250)
    q, _ = task.turn_plan(robot.pose[:3] + DOWN, [0, -1.57, 1.57, np.radians(-70), -1.57, 0.5])
    # shifted to the current turn it would be 290 deg... fine; one more turn would be 430
    assert abs(np.degrees(q[3])) < 360 - ppg.JOINT_LIMIT_MARGIN_DEG
    robot.q[3] = np.radians(330)
    q, _ = task.turn_plan(robot.pose[:3] + DOWN, [0, -1.57, 1.57, np.radians(-20), -1.57, 0.5])
    # 340 deg would be near the limit: -20 deg instead (a full turn back on this move)
    assert np.degrees(q[3]) == pytest.approx(-20, abs=0.01)


def test_a_sagging_turn_is_done_higher_up(clock):
    # Between two arm shapes the TCP sags on the joint arc: 50 mm here, more than
    # TURN_MAX_DIP, so the turn must happen higher, not be refused
    task, robot, suction = make_task(["pick_side", "place"])
    fake_fk = robot.getForwardKinematics

    def fk(q=None, tcp_offset=None):
        if q is None or tuple(q) in robot.ik_poses:
            return fake_fk(q, tcp_offset)
        pose = fake_fk(q, tcp_offset)                   # on the arc: below the latest IK target
        pose[2] = list(robot.ik_poses.values())[-1][2] - 0.05
        return pose

    robot.getForwardKinematics = fk
    assert run(task, robot, clock) == "done"
    safe_z = max(0.3, task.carry_z)
    raised = [call[1] for call in robot.calls if call[0] == "moveL" and call[1][2] > safe_z + 0.02]
    assert raised, "the turn was not raised"
    assert all(z <= MAX_TCP_Z for z in (call[1][2] for call in robot.calls if call[0] == "moveL"))
    assert [call[0] for call in robot.calls].count("moveJ") == 2    # sideways, back


def test_spray_needs_the_sprayer_before_anything_moves(clock):
    task, robot, suction = make_task(ppg.SEQUENCE)
    task.sprayer = None
    run(task, robot, clock)
    assert "sprayer" in task.status and "not connected" in task.status
    assert not [call for call in robot.calls if call[0] in ("moveL", "moveJ", "speedL")]


def test_sprayer_failure_keeps_the_glass(clock):
    task, robot, suction = make_task(["pick_top", "spray", "place"],
                                     sprayer=FakeSprayer(error=TimeoutError("no reply from servo")))
    run(task, robot, clock)
    assert "sprayer failed" in task.status
    assert suction.calls == ["grip"]                 # not released over the sprayer
    assert task.sprayer.stop_requests == 1


def test_spray_holds_the_taught_pose(clock):
    task, robot, suction = make_task(["pick_top", "spray", "place"])
    assert run(task, robot, clock) == "done"
    into = [call[1] for call in robot.calls if call[0] == "moveL" and call[1][:3] == pytest.approx(ppg.SPRAY_POSE[:3])]
    assert len(into) == 1 and into[0][3:] == pytest.approx(ppg.SPRAY_POSE[3:])



# --- self-collision model and unwinding, real UR5e geometry -------------------------

REAL_TCP = ppg.system_arm.pose_matrix([0, 0, 0.0766, 0, 0, 0])       # measured on the robot
WOUND = np.radians([-61.6, -91.0, -133.9, 222.9, 296.7, 47.1])        # read from the robot
real_self_gap = ppg.self_gap                  # taken at import, before the fixture swaps it
TAUGHT_Q = (ppg.SIDE_GRIP_Q, ppg.TOP_GRIP_Q, ppg.SPRAY_Q)     # likewise


NOW = np.radians([-126.8, -98.3, -92.1, 80.4, 335.2, -13.1])         # read later: wrist 2 wound
TAUGHT_GAP = min(real_self_gap(q, REAL_TCP) for q in TAUGHT_Q) - ppg.SELF_GAP_MARGIN


def plan(q):
    return ppg.unwind_waypoints(q, lambda x: real_self_gap(x, REAL_TCP),
                                lambda x: ppg.system_arm.tcp_matrix(list(x), REAL_TCP)[2, 3], TAUGHT_GAP)


@pytest.mark.parametrize("q", [WOUND, NOW])
def test_unwinding_goes_one_wrist_at_a_time_and_stays_clear(q):
    path = plan(q)
    assert path is not None
    end = np.degrees(path[-1])
    assert end[3:] == pytest.approx([(a + 180) % 360 - 180 if abs(a) > ppg.UNWIND_ABOVE_DEG else a
                                     for a in np.degrees(q[3:])])
    assert path[-1][:3] == pytest.approx(q[:3])
    for a, b in zip(path, path[1:]):
        assert np.count_nonzero(np.abs(b - a) > 1e-9) == 1
    assert min(real_self_gap(x, REAL_TCP) for x in ppg.joint_samples(path)) >= TAUGHT_GAP


@pytest.mark.parametrize("q", [WOUND, NOW])
def test_turning_the_wrists_at_once_would_hit_the_arm(q):
    # What the pendant stopped: both at once, or wrist 2 alone from where it stood
    straight = [q, plan(q)[-1]]
    assert min(real_self_gap(x, REAL_TCP) for x in ppg.joint_samples(straight)) < TAUGHT_GAP - 0.05



def test_right_orientation_in_the_wrong_configuration_still_turns(clock, monkeypatch):
    # The arm stood in the side orientation, wrist flipped: the turn was skipped and the
    # gripper went down into the table
    side_q = [0.1, -1.2, 1.9, -1.6, -1.1, -1.4]
    monkeypatch.setattr(ppg, "SIDE_GRIP_Q", side_q)
    task, robot, suction = make_task(["pick_side", "place"])
    robot.pose[3:] = list(ppg.SIDE_GRIP_ROTATION)
    flipped = [0.1, -1.2, 1.9, -1.6 + pi, 1.1, -1.4 + pi]
    robot.q = list(flipped)
    assert run(task, robot, clock) == "done"
    first_move_j = next(call[1] for call in robot.calls if call[0] == "moveJ")
    assert first_move_j[3:] == pytest.approx(side_q[3:], abs=1e-4)
    grip = next(i for i, call in enumerate(robot.calls) if call[0] == "speedL")
    assert any(call[0] == "moveJ" for call in robot.calls[:grip])


def test_going_down_in_the_wrong_configuration_is_refused(clock, monkeypatch):
    side_q = [0.1, -1.2, 1.9, -1.6, -1.1, -1.4]
    task, robot, suction = make_task(["pick_side", "place"])
    robot.q = [0.1, -1.2, 1.9, -1.6 + pi, 1.1, -1.4 + pi]      # wrist flipped
    # The fake IK answers in the configuration asked for, like the robot's
    with pytest.raises(ppg.TaskFailed, match="not in the taught side grip configuration"):
        task.check_configuration(side_q, "side grip")
    robot.q = list(side_q)
    task.check_configuration(side_q, "side grip")



def test_dry_comes_in_along_the_tool_axis_and_swings_about_it(clock):
    task, robot, suction = make_task(["pick_top", "dry", "place"])
    assert run(task, robot, clock) == "done"
    tool_z = cv2.Rodrigues(np.asarray(ppg.DRY_POSE[3:]))[0][:, 2]
    entry = np.subtract(ppg.DRY_POSE[:3], ppg.DRY_APPROACH * tool_z)
    # joint move to the entry (the fake IK hands out that pose), straight in, straight out
    assert list(robot.ik_poses.values())[1][:3] == pytest.approx(list(entry))
    into = [call[1] for call in robot.calls if call[0] == "moveL" and call[1][:3] == pytest.approx(ppg.DRY_POSE[:3])]
    assert len(into) == 1 and into[0][3:] == pytest.approx(ppg.DRY_POSE[3:])
    # the swings only turn wrist 3
    swings = [call[1] for call in robot.calls if call[0] == "moveJ"][2:2 + 2 * ppg.DRY_SWINGS]
    assert all(q[:5] == pytest.approx(swings[0][:5]) for q in swings)
    assert suction.calls == ["grip", "release"]


def test_stations_chain_and_return_once(clock):
    task, robot, suction = make_task(["pick_top", "spray", "dry", "place"])
    assert run(task, robot, clock) == "done"
    # after the pick: sprayer, dryer, ... one move back to the joints before the sprayer
    back = [call[1] for call in robot.calls if call[0] == "moveJ"]
    returns = [q for q in back if q == pytest.approx(back[0])]
    assert len(returns) == 2        # the turn down itself, and the one return before placing



def test_the_next_move_follows_as_soon_as_one_ends(clock):
    # No fixed 0.2 s start-up wait any more: the controller's async change count says
    # when the move started and ended
    task, robot, suction = make_task(["pick_top", "place"])
    task.rotation = DOWN
    target = list(np.add(robot.pose[:3], [0.0, 0.0, 0.001]))       # 1 mm: done in one step
    steps = 0
    for _ in task.move_to(target, "short"):
        steps += 1
        clock[0] += DT
        robot.advance(DT)
    assert steps <= 2 + int(np.ceil(ppg.MOVE_SETTLE / DT))      # + the settle after it
    assert robot.pose[:3] == pytest.approx(target)


def test_a_move_the_controller_never_reports_ends_after_the_timeout(clock):
    task, robot, suction = make_task(["pick_top", "place"])
    task.rotation = DOWN
    robot.moveL = lambda pose, speed, accel, asynchronous=False: True     # no change count
    steps = 0
    for _ in task.move_to(robot.pose[:3], "nowhere"):
        steps += 1
        clock[0] += DT
    assert steps == pytest.approx(ppg.MOVE_START_TIMEOUT / DT, abs=2)


def test_stopping_ends_the_move_thread_before_speed_stop(clock):
    # speedStop while the robot script's async move thread still runs stops the program
    # ("another thread is already controlling the robot")
    task, robot, suction = make_task(["pick_top", "place"])
    for _ in range(5):
        task.update()
        clock[0] += DT
        robot.advance(DT)
    robot.calls.clear()
    task.abort()
    kinds = [call[0] for call in robot.calls]
    assert kinds == ["stopL", "stopJ", "speedStop"]


def test_the_next_command_waits_for_the_move_thread_to_exit(clock):
    task, robot, suction = make_task(["pick_top", "place"])
    task.rotation = DOWN
    target = list(np.add(robot.pose[:3], [0.0, 0.0, 0.001]))
    finished_at = None
    for _ in task.move_to(target, "short"):
        if finished_at is None and robot.target is None:
            finished_at = clock[0]
        clock[0] += DT
        robot.advance(DT)
    assert clock[0] - finished_at >= ppg.MOVE_SETTLE
