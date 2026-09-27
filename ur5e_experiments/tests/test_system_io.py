"""Serial device workers against in-memory drivers (real threads, short timeouts)."""

import time

import pytest

import system_io
from system_io import GripperWorker, ServoWorker, SpongeProfile, SprayProfile, grip_from_reports
from system_model import Grip, Outcome


def wait_result(worker, job_id, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = worker.result(job_id)
        if result is not None:
            return result
        time.sleep(0.005)
    pytest.fail(f"no result for {job_id}")


@pytest.mark.parametrize("state,report,holding,grip", [
    ("IDLE", None, None, Grip.NO),
    ("RELEASING", None, None, Grip.NO),
    ("GRIPPING", None, None, Grip.UNKNOWN),
    ("GRIPPING", "FAIL", False, Grip.UNKNOWN),
    ("GRIPPING", "OK", True, Grip.OK),
    ("GRIPPING", "OK", None, Grip.UNKNOWN),     # sensor missing: not confirmed
    ("GRIPPING", "OK", False, Grip.LOST),
    ("GRIPPING", "LOST", True, Grip.LOST),
    (None, None, None, Grip.UNKNOWN),
])
def test_grip_mapping_needs_report_and_live_hold(state, report, holding, grip):
    assert grip_from_reports(state, report, holding) == grip


class FakeSuction:
    def __init__(self):
        self.state = "IDLE"
        self.report = None
        self.hold = None
        self.pulse_until = 0.0
        self.pulse_ends = True
        self.fail = False
        self.sent = []

    def _update(self):
        if self.fail:
            raise OSError("USB unplugged")
        if self.state == "RELEASING" and self.pulse_ends and time.monotonic() > self.pulse_until:
            self.state = "IDLE"

    def status(self):
        self._update()
        return self.state

    def grip_result(self):
        return self.report

    def is_holding(self):
        return self.hold

    def grip(self):
        self._update()
        if self.state == "RELEASING":
            return False
        self.sent.append("GRIP")
        self.state, self.report, self.hold = "GRIPPING", "OK", True
        return True

    def release(self):
        self.sent.append("RELEASE")
        self.state, self.report, self.hold = "RELEASING", None, None
        self.pulse_until = time.monotonic() + 0.05
        return True

    def close(self):
        pass


@pytest.fixture
def gripper():
    suction = FakeSuction()
    worker = GripperWorker(lambda: suction)
    worker.start()
    yield worker, suction
    worker.close()


def test_grip_then_release_is_confirmed_by_idle_state(gripper):
    worker, suction = gripper
    worker.submit("g", "grip")
    assert wait_result(worker, "g").outcome == Outcome.SUCCEEDED
    assert worker.snapshot()["grip"] == Grip.OK
    worker.submit("r", "release")
    assert wait_result(worker, "r").outcome == Outcome.SUCCEEDED
    assert worker.snapshot()["grip"] == Grip.NO and suction.sent == ["GRIP", "RELEASE"]


def test_release_without_end_of_pulse_is_a_failure(gripper, monkeypatch):
    worker, suction = gripper
    monkeypatch.setattr(system_io, "RELEASE_CONFIRM_TIMEOUT", 0.1)
    suction.pulse_ends = False
    worker.submit("r", "release")
    result = wait_result(worker, "r")
    assert result.outcome == Outcome.FAILED and "unknown" in result.detail


def test_stop_never_releases_the_vacuum(gripper):
    worker, suction = gripper
    worker.submit("g", "grip")
    wait_result(worker, "g")
    worker.request_stop("stop")
    assert wait_result(worker, "stop").outcome == Outcome.SUCCEEDED
    assert suction.sent == ["GRIP"] and worker.snapshot()["grip"] == Grip.OK


def test_serial_error_latches_and_refuses_new_jobs(gripper):
    worker, suction = gripper
    suction.fail = True
    deadline = time.monotonic() + 2
    while worker.connected and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not worker.connected and "USB unplugged" in worker.fault
    with pytest.raises(RuntimeError):
        worker.submit("g", "grip")


class FakeServo:
    """BusServo stand-in: reaches a goal after `lag` s unless stuck."""

    def __init__(self, bus, servo_id, *args, **kwargs):
        self.id = servo_id
        self.bus = bus
        self.goal = self.position_deg = 280.0
        self.moved_at = 0.0
        self.bus.servos[servo_id] = self

    def ping(self):
        return True

    def move_to(self, degrees, speed=None, acceleration=None):
        self.bus.log.append((self.id, degrees))
        self.goal, self.moved_at = degrees, time.monotonic()

    def stop(self):
        self.goal = self.position_deg

    def status(self):
        if self.id in self.bus.stuck:
            moving = True
        else:
            moving = time.monotonic() - self.moved_at < self.bus.lag
            if not moving:
                self.position_deg = self.goal
        return {"position": round(self.position_deg / 360 * 4096), "moving": moving,
                "errors": list(self.bus.errors.get(self.id, []))}


class FakeBus:
    def __init__(self):
        self.servos, self.log, self.stuck, self.errors, self.lag = {}, [], set(), {}, 0.01

    def close(self):
        pass


@pytest.fixture
def servos(monkeypatch):
    import bus_servos
    monkeypatch.setattr(bus_servos, "BusServo", FakeServo)
    monkeypatch.setattr(system_io, "SERVO_MOVE_TIMEOUT", 0.2)
    bus = FakeBus()
    worker = ServoWorker(lambda: bus, sprayer_id=1, sponge_id=2, spray_rest_deg=280.0)
    worker.start()
    yield worker, bus
    worker.close()


SPRAY = SprayProfile(rest_deg=280.0, press_deg=256.0, strokes=2, hold=0.01, period=0.05, speed=5500)


def test_spray_job_confirms_every_position(servos):
    worker, bus = servos
    worker.submit("s", "spray", profile=SPRAY)
    assert wait_result(worker, "s").outcome == Outcome.SUCCEEDED
    assert bus.log == [(1, 256.0), (1, 280.0)] * 2


def test_sponge_job_runs_the_profile(servos):
    worker, bus = servos
    worker.submit("p", "sponge", profile=SpongeProfile((150.0, 210.0), 2, 1500))
    assert wait_result(worker, "p").outcome == Outcome.SUCCEEDED
    assert [goal for servo_id, goal in bus.log if servo_id == 2] == [150.0, 210.0] * 2


@pytest.mark.parametrize("fault", ["stuck", "error"])
def test_servo_that_does_not_arrive_or_reports_an_error_fails_the_job(servos, fault):
    worker, bus = servos
    if fault == "stuck":
        bus.stuck.add(1)
    else:
        bus.errors[1] = ["overload"]
    worker.submit("s", "spray", profile=SPRAY)
    result = wait_result(worker, "s")
    assert result.outcome == Outcome.FAILED
    assert ("did not reach" if fault == "stuck" else "overload") in result.detail


def test_stop_cancels_the_job_and_rests_the_sprayer(servos):
    worker, bus = servos
    worker.submit("s", "spray", profile=SprayProfile(280.0, 256.0, 50, 0.05, 0.2, 5500))
    time.sleep(0.05)
    worker.request_stop("stop")
    assert wait_result(worker, "s").outcome == Outcome.CANCELLED
    assert wait_result(worker, "stop").outcome == Outcome.SUCCEEDED
    assert bus.log[-1] == (1, 280.0) and worker.snapshot()["stopped"]


def test_stop_that_is_not_confirmed_fails(servos, monkeypatch):
    worker, bus = servos
    monkeypatch.setattr(system_io, "SERVO_STOP_TIMEOUT", 0.1)
    bus.stuck.add(2)
    worker.request_stop("stop")
    assert wait_result(worker, "stop").outcome == Outcome.FAILED


def test_same_servo_for_two_roles_is_refused():
    with pytest.raises(ValueError):
        ServoWorker(lambda: None, 1, 1, 280.0)
