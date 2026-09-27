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
    # No zones from the real cell (the zone tests set their own)
    monkeypatch.setattr(ppg, "EXCLUSION_ZONES", [])


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


class FakeSponge:
    """bus_servos.BusServo in wheel mode."""

    def __init__(self, error=None):
        self.speeds = []
        self.stops = 0
        self.error = error

    def spin(self, speed, acceleration=None):
        if self.error is not None:
            raise self.error
        self.speeds.append(speed)

    def stop(self):
        self.stops += 1


def make_task(sequence, lose_on_flip=False, sprayer=None, sponge=None):
    robot = FakeRobot(pose=(0.3, -0.3, 0.3, *DOWN))
    suction = FakeSuction(robot, lose_on_flip)
    glass = SimpleNamespace(x=0.35, y=-0.45, diameter=0.07)
    task = ppg.SequenceTask(robot, SafeControl(robot, robot), suction, glass,
                            place_xy=(0.1, -0.5), table_z=0.0, glass_height=0.075,
                            sequence=sequence, sprayer=FakeSprayer() if sprayer is None else sprayer,
                            sponge=FakeSponge() if sponge is None else sponge)
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
    turns = [pytest.approx(dry_q5 + np.radians(a)) for a in ppg.DRY_TURNS_DEG]
    # (from station to station directly: the checks pass here)
    assert wrist[3] == pytest.approx(ppg.SPRAY_Q[5])
    assert wrist[4] == pytest.approx(ppg.SPONGE_Q[5])
    assert wrist[5] == pytest.approx(dry_q5)
    assert wrist[6:6 + len(turns)] == turns
    assert wrist[6 + len(turns):] == [pytest.approx(q5), pytest.approx(q5)]
    assert len(robot.ik_calls) == 6
    assert task.sponge.speeds == [ppg.SPONGE_SPEED, -ppg.SPONGE_SPEED] and task.sponge.stops == 1
    assert task.sprayer.started == [ppg.SPRAY_STROKES] and task.sprayer.stop_requests == 1
    turn_anchors = [qnear for i, (qnear, *_) in enumerate(robot.ik_calls) if i not in (2, 3, 4)]   # stations
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
    assert len(anchors) == 8
    assert [anchors[i] for i in (0, 1, 2, 3, 7)] == [pytest.approx(side_q), pytest.approx(side_q),
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
TAUGHT_Q = (ppg.SIDE_GRIP_Q, ppg.TOP_GRIP_Q, ppg.SPRAY_Q, ppg.DRY_Q)     # likewise


NOW = np.radians([-126.8, -98.3, -92.1, 80.4, 335.2, -13.1])         # read later: wrist 2 wound
TAUGHT_GAP = min(ppg.SELF_GAP_PROVEN, *(real_self_gap(q, REAL_TCP) for q in TAUGHT_Q)) - ppg.SELF_GAP_MARGIN


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



def test_dry_comes_in_along_the_tool_axis_and_turns_about_it(clock):
    task, robot, suction = make_task(["pick_top", "dry", "place"])
    assert run(task, robot, clock) == "done"
    tool_z = cv2.Rodrigues(np.asarray(ppg.DRY_POSE[3:]))[0][:, 2]
    deeper = np.add(ppg.DRY_POSE[:3], ppg.DRY_DEPTH * tool_z)       # DRY_DEPTH along the tool z axis
    entry = deeper - ppg.DRY_APPROACH * tool_z
    # joint move to the entry (the fake IK hands out that pose), straight in, straight out
    assert list(robot.ik_poses.values())[1][:3] == pytest.approx(list(entry))
    into = [call[1] for call in robot.calls if call[0] == "moveL" and call[1][:3] == pytest.approx(list(deeper))]
    assert len(into) == 1 and into[0][3:] == pytest.approx(ppg.DRY_POSE[3:])
    # the turns only turn wrist 3: +180 deg from DRY_Q's own angle (never a turn wound)
    moves = [call[1] for call in robot.calls if call[0] == "moveJ"]
    entry_q, turned = moves[1], moves[2:2 + len(ppg.DRY_TURNS_DEG)]
    assert entry_q[5] == pytest.approx(ppg.DRY_Q[5])
    assert [q[5] for q in turned] == [pytest.approx(ppg.DRY_Q[5] + np.radians(a)) for a in ppg.DRY_TURNS_DEG]
    assert all(q[:5] == pytest.approx(entry_q[:5]) for q in turned)
    assert ppg.joint_near_limit(turned[-1]) is None
    assert suction.calls == ["grip", "release"]


def test_stations_go_direct_when_the_checks_pass(clock):
    task, robot, suction = make_task(["pick_top", "spray", "dry", "place"])
    assert run(task, robot, clock) == "done"
    moves = [call[1] for call in robot.calls if call[0] == "moveJ"]
    lifted = moves[0]
    returns = [i for i, q in enumerate(moves) if q == pytest.approx(lifted)]
    assert len(returns) == 2        # the turn down itself, and only before placing
    assert moves[1][5] == pytest.approx(ppg.SPRAY_Q[5]) and moves[2][5] == pytest.approx(ppg.DRY_Q[5])


def test_stations_go_through_the_lifted_joints_when_the_direct_move_is_refused(clock):
    # the direct sprayer -> dryer move swung the gripper into the arm (model)
    task, robot, suction = make_task(["pick_top", "spray", "dry", "place"])
    plan = task.station_plan

    def refuse_direct(entry, q_ref, name, keep_turns=False):
        if name == "drying" and task.station_return_q is not None and not refuse_direct.refused:
            refuse_direct.refused = True
            raise ppg.TaskFailed("drying: too close to the arm")
        return plan(entry, q_ref, name, keep_turns)

    refuse_direct.refused = False
    task.station_plan = refuse_direct
    assert run(task, robot, clock) == "done"
    moves = [call[1] for call in robot.calls if call[0] == "moveJ"]
    lifted = moves[0]
    returns = [i for i, q in enumerate(moves) if q == pytest.approx(lifted)]
    assert len(returns) == 3        # the turn down itself, between the stations, before placing
    assert moves[returns[1] + 1][5] == pytest.approx(ppg.DRY_Q[5])     # next: the dryer


def test_tools_turn_where_they_are(clock, monkeypatch):
    task, robot, suction = make_task(["pick_side", "place"])
    assert run(task, robot, clock) == "done"
    # no move back over the start position before the side turn: the first moveJ
    # (the turn) starts where the tool went up, over the start of the task
    first_turn = next(i for i, call in enumerate(robot.calls) if call[0] == "moveJ")
    before = [call[1] for call in robot.calls[:first_turn] if call[0] == "moveL"]
    assert before and all(p[:2] == pytest.approx([0.3, -0.3]) for p in before)
    # the old way still works
    monkeypatch.setattr(ppg, "TURN_OVER_START", True)
    task, robot, suction = make_task(["pick_side", "place"])
    assert run(task, robot, clock) == "done"



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


def test_spray_button_starts_and_stops_a_burst(capsys):
    sprayer = FakeSprayer(polls=100)
    assert "no sprayer" in ppg.spray_command(None, None)
    assert "task running" in ppg.spray_command(sprayer, task=object())
    assert sprayer.started == []
    assert "spraying" in ppg.spray_command(sprayer, None)
    assert sprayer.started == [ppg.MANUAL_SPRAY_STROKES]
    assert "stopping" in ppg.spray_command(sprayer, None)       # pressed again while spraying
    assert sprayer.stop_requests == 1 and sprayer.started == [ppg.MANUAL_SPRAY_STROKES]


def test_the_task_spray_waits_for_a_manual_burst(clock):
    sprayer = FakeSprayer()
    task, robot, suction = make_task(["pick_top", "spray", "place"], sprayer=sprayer)
    sprayer.start(count=ppg.MANUAL_SPRAY_STROKES)                  # y pressed just before p
    assert run(task, robot, clock) == "done"
    assert sprayer.started == [ppg.MANUAL_SPRAY_STROKES, ppg.SPRAY_STROKES]


def test_routes_to_the_taught_stations_stay_clear():
    # Real UR5e model, real TCP: lifted top grip -> each station entry and back
    A = ppg.system_arm
    carry = 0.006 + 0.075 + ppg.CARRY_CLEARANCE + 0.075
    lifted = A.solve_ik(A.pose_matrix([-0.13, -0.68, carry, *ppg.TOP_GRIP_ROTATION]), list(TAUGHT_Q[1]), REAL_TCP)
    tool_z = cv2.Rodrigues(np.asarray(ppg.DRY_POSE[3:]))[0][:, 2]
    dry = np.add(ppg.DRY_POSE[:3], ppg.DRY_DEPTH * tool_z) - ppg.DRY_APPROACH * tool_z
    entries = {"spray": ([*np.add(ppg.SPRAY_POSE[:3], ppg.SPRAY_APPROACH), *ppg.SPRAY_POSE[3:]], TAUGHT_Q[2]),
               "sponge": ([*np.add(ppg.SPONGE_POSE[:3], ppg.SPONGE_APPROACH), *ppg.SPONGE_POSE[3:]], ppg.SPONGE_Q),
               "dry": ([*dry, *ppg.DRY_POSE[3:]], TAUGHT_Q[3])}
    q_entry = {}
    for name, (pose, q_ref) in entries.items():
        q_entry[name] = A.solve_ik(A.pose_matrix(pose), list(q_ref), REAL_TCP)
        samples = list(ppg.joint_samples([lifted, q_entry[name], lifted]))
        assert min(real_self_gap(q, REAL_TCP) for q in samples) >= TAUGHT_GAP, name
        z = [A.tcp_matrix(list(q), REAL_TCP)[2, 3] for q in samples]
        assert min(z) >= min(carry, pose[2]) - ppg.TURN_MAX_DIP, name
    # ... and the direct moves from station to station (sprayer -> sponge -> dryer)
    for a, b in (("spray", "sponge"), ("sponge", "dry")):
        samples = list(ppg.joint_samples([q_entry[a], q_entry[b]]))
        assert min(real_self_gap(q, REAL_TCP) for q in samples) >= TAUGHT_GAP, (a, b)



def test_sponge_comes_from_above_and_turns_both_ways(clock):
    task, robot, suction = make_task(["pick_top", "sponge", "place"])
    started = clock[0]
    assert run(task, robot, clock) == "done"
    entry = list(np.add(ppg.SPONGE_POSE[:3], ppg.SPONGE_APPROACH))
    assert list(robot.ik_poses.values())[1][:3] == pytest.approx(entry)       # joint move above it
    into = [call[1] for call in robot.calls if call[0] == "moveL" and call[1][:3] == pytest.approx(ppg.SPONGE_POSE[:3])]
    assert len(into) == 1 and into[0][3:] == pytest.approx(ppg.SPONGE_POSE[3:])
    assert task.sponge.speeds == [ppg.SPONGE_SPEED, -ppg.SPONGE_SPEED] and task.sponge.stops == 1
    assert clock[0] - started >= 2 * ppg.SPONGE_SPIN_TIME


def test_sponge_servo_failure_keeps_the_glass_and_stops_it(clock):
    task, robot, suction = make_task(["pick_top", "sponge", "place"],
                                     sponge=FakeSponge(error=TimeoutError("no reply from servo")))
    run(task, robot, clock)
    assert "sponge servo not answering" in task.status
    assert suction.calls == ["grip"] and task.sponge.stops == 1


def test_sponge_needs_its_servo_before_anything_moves(clock):
    task, robot, suction = make_task(ppg.SEQUENCE)
    task.sponge = None
    run(task, robot, clock)
    assert "sponge servo" in task.status and "not connected" in task.status
    assert not [call for call in robot.calls if call[0] in ("moveL", "moveJ", "speedL")]


# --- exclusion zones --------------------------------------------------------------------

ZONE = ([0.0, -0.40, 0.0], [0.10, -0.20, 0.40])       # a box between the start and the glass


def test_segment_box_test():
    lo, hi = ZONE
    assert ppg.segment_hits_box([-0.1, -0.3, 0.2], [0.2, -0.3, 0.2], lo, hi, 0.0)     # straight through
    assert not ppg.segment_hits_box([-0.1, -0.3, 0.5], [0.2, -0.3, 0.5], lo, hi, 0.0)  # over it
    assert ppg.segment_hits_box([-0.1, -0.3, 0.42], [0.2, -0.3, 0.42], lo, hi, 0.03)  # within the margin
    assert ppg.segment_hits_box([0.05, -0.3, 0.2], [0.05, -0.3, 0.2], lo, hi, 0.0)    # a point inside


def test_leaving_a_zone_is_allowed_entering_is_not(monkeypatch):
    monkeypatch.setattr(ppg, "EXCLUSION_ZONES", [ZONE])
    inside, outside = [0.05, -0.3, 0.2], [0.3, -0.3, 0.2]
    assert ppg.zone_hit([inside, outside], 0.03) is None
    assert ppg.zone_hit([outside, inside], 0.03) == 0


def test_traversing_into_a_zone_is_refused_before_moving(clock, monkeypatch):
    # start (0.3, -0.3, 0.3), glass (0.35, -0.45): a zone across the way over to it
    lo, hi = [0.28, -0.42, 0.10], [0.40, -0.36, 0.30]
    monkeypatch.setattr(ppg, "EXCLUSION_ZONES", [(lo, hi)])
    task, robot, suction = make_task(["pick_top", "place"])
    run(task, robot, clock)
    assert "to glass would enter exclusion zone 0" in task.status
    sent = [call[1][:3] for call in robot.calls if call[0] == "moveL"]
    assert all(not ppg.segment_hits_box(p, p, lo, hi, 0.0) for p in sent)
    assert "speedL" not in [call[0] for call in robot.calls]      # never went down to pick


def test_station_moves_may_enter_a_zone(clock, monkeypatch):
    # a zone around the sponge: the station goes in, traversal stays out
    lo = np.subtract(ppg.SPONGE_POSE[:3], 0.05)
    hi = np.add(ppg.SPONGE_POSE[:3], [0.05, 0.05, 0.12])
    monkeypatch.setattr(ppg, "EXCLUSION_ZONES", [(list(lo), list(hi))])
    task, robot, suction = make_task(["pick_top", "sponge", "place"])
    assert run(task, robot, clock) == "done"
    assert task.sponge.speeds == [ppg.SPONGE_SPEED, -ppg.SPONGE_SPEED]


def test_a_held_glass_keeps_further_away(clock):
    task, robot, suction = make_task(["pick_top", "place"])
    empty = task.zone_reach()
    task.held = "top"
    assert task.zone_reach() == pytest.approx(empty + np.hypot(task.glass_height, task.radius))


def test_after_the_sprayer_the_glass_goes_straight_up_first(clock):
    task, robot, suction = make_task(["pick_top", "spray", "sponge", "place"])
    assert run(task, robot, clock) == "done"
    exit_xy = np.add(ppg.SPRAY_POSE[:3], ppg.SPRAY_APPROACH)[:2]
    moves = [call for call in robot.calls if call[0] in ("moveL", "moveJ")]
    out = next(i for i, call in enumerate(moves) if call[0] == "moveL"
               and call[1][:2] == pytest.approx(list(exit_xy)) and call[1][2] == pytest.approx(ppg.SPRAY_POSE[2]))
    up = moves[out + 1]
    assert up[0] == "moveL" and up[1][:2] == pytest.approx(list(exit_xy))
    assert up[1][2] == pytest.approx(task.table_z + ppg.TRAVERSE_HEIGHT)
    assert moves[out + 2][0] == "moveJ"         # only then over to the sponge


# --- several glasses, several place tags ------------------------------------------------

def glass(x, y):
    return SimpleNamespace(x=x, y=y, diameter=0.07)


def test_the_glass_closest_to_the_base_is_picked_not_those_on_a_tag():
    tags = {3: np.array([-0.2, -0.99]), 7: np.array([0.0, -0.99])}
    near_base, far, placed = glass(0.05, -0.45), glass(-0.2, -0.8), glass(-0.2, -0.985)
    assert ppg.choose_glass([far, placed, near_base], tags) is near_base
    assert ppg.choose_glass([placed], tags) is None


def test_tags_fill_up_in_id_order_and_c_starts_over(monkeypatch):
    tags = {10: np.array([0.1, -0.99]), 3: np.array([-0.2, -0.99]), 7: np.array([0.0, -0.99])}
    used = set()
    assert ppg.next_place_tag(tags, used, []) == 3
    used.add(3)
    assert ppg.next_place_tag(tags, used, []) == 7
    # a glass already standing on tag 7 (put there by hand): skipped too
    assert ppg.next_place_tag(tags, used, [glass(0.0, -0.99)]) == 10
    used |= {7, 10}
    assert ppg.next_place_tag(tags, used, []) is None
    used.clear()                                        # c
    assert ppg.next_place_tag(tags, used, []) == 3
    monkeypatch.setattr(ppg, "PLACE_TAG_IDS", [10, 3])  # only these, in this order
    assert ppg.next_place_tag(tags, set(), []) == 10


def test_a_tag_counts_as_used_once_the_glass_is_down(clock):
    placed = []
    task, robot, suction = make_task(["pick_top", "place"])
    task.on_placed = lambda: placed.append(robot.pose[:2])
    assert run(task, robot, clock) == "done"
    assert len(placed) == 1 and placed[0] == pytest.approx(np.add([0.1, -0.5], ppg.TOP_GRIP_OFFSET), abs=0.002)


def test_a_task_failing_before_the_glass_is_down_leaves_the_tag_free(clock):
    placed = []
    task, robot, suction = make_task(ppg.SEQUENCE, lose_on_flip=True)
    task.on_placed = lambda: placed.append(True)
    run(task, robot, clock)
    assert "glass lost" in task.status and placed == []
