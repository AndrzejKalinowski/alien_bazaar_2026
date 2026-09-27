"""Motion watchdog, jog Z limits and the bus servo packets, with fake hardware."""

import pytest

import bus_servos as bs
import gamepad_jog as gj
from robot_watchdog import RobotWatchdog

MIN_Z, MAX_Z = -0.05, 0.60


# --- robot_watchdog -------------------------------------------------------------

class FakeControl:
    def __init__(self):
        self.running = True
        self.protective_stop = False
        self.calls = []

    def setWatchdog(self, frequency):
        self.calls.append(("set", frequency))
        return self.running

    def kickWatchdog(self):
        self.calls.append("kick")
        return self.running and not self.protective_stop

    def isProgramRunning(self):
        return self.running

    def isProtectiveStopped(self):
        return self.protective_stop

    def isEmergencyStopped(self):
        return False

    def reuploadScript(self):
        self.calls.append("reupload")
        self.running = True
        return True


def test_watchdog_arms_and_kicks():
    c = FakeControl()
    watchdog = RobotWatchdog(c)
    assert c.calls == [("set", 5.0)]
    assert watchdog.kick() is True


def test_watchdog_trip_reuploads_and_rearms():
    c = FakeControl()
    watchdog = RobotWatchdog(c)
    c.running = False
    assert watchdog.kick() is False
    assert c.calls[-2:] == ["reupload", ("set", 5.0)]
    assert watchdog.kick() is True


def test_watchdog_waits_for_protective_stop_to_clear():
    c = FakeControl()
    watchdog = RobotWatchdog(c)
    c.running, c.protective_stop = False, True
    before = len(c.calls)
    assert watchdog.kick() is False
    assert watchdog.kick() is False
    assert "reupload" not in c.calls[before:]
    c.protective_stop = False
    assert watchdog.kick() is False            # re-uploads now
    assert watchdog.kick() is True


# --- gamepad_jog ---------------------------------------------------------------

def z_speed(vz, tcp_z):
    return gj.limit_z_speed([0, 0, vz, 0, 0, 0], tcp_z, MIN_Z, MAX_Z)[2]


def test_jog_is_free_between_the_limits():
    assert z_speed(gj.MAX_Z_SPEED, 0.3) == gj.MAX_Z_SPEED
    assert z_speed(-gj.MAX_Z_SPEED, 0.3) == -gj.MAX_Z_SPEED


def test_jog_slows_down_near_the_limits():
    assert z_speed(gj.MAX_Z_SPEED, MAX_Z - 0.01) == pytest.approx(gj.Z_LIMIT_GAIN * 0.01)
    assert z_speed(-gj.MAX_Z_SPEED, MIN_Z + 0.01) == pytest.approx(-gj.Z_LIMIT_GAIN * 0.01)


def test_jog_beyond_a_limit_only_allows_the_way_back():
    assert z_speed(gj.MAX_Z_SPEED, MAX_Z + 0.01) == 0
    assert z_speed(-gj.MAX_Z_SPEED, MAX_Z + 0.01) == -gj.MAX_Z_SPEED
    assert z_speed(-gj.MAX_Z_SPEED, MIN_Z - 0.01) == 0
    assert z_speed(gj.MAX_Z_SPEED, MIN_Z - 0.01) == gj.MAX_Z_SPEED


def test_jogger_sends_the_capped_speed():
    class Pad:
        def jog_speed(self):
            return [0.01, 0, gj.MAX_Z_SPEED, 0, 0, 0]

    class Control:
        def speedL(self, speed, accel, time):
            assert time > 0                    # time=0 protective-stops the robot (C271A1)
            self.speed = speed

    c = Control()
    gj.Jogger(c, Pad(), MIN_Z, MAX_Z).update([0, 0, MAX_Z + 0.005, 0, 0, 0])
    assert c.speed[2] == 0 and c.speed[0] == 0.01


# --- bus_servos ----------------------------------------------------------------

def test_ping_packet_matches_the_sts_protocol():
    # PING to ID 1: FF FF 01 02 01 FB (checksum = ~(1 + 2 + 1) & 0xFF)
    assert bs._packet(1, 0x01) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


@pytest.mark.parametrize("value", [0, 1, 450, -1, -450, 32767, -32767])
def test_sign_magnitude_round_trip(value):
    encoded = bs._to_sign_magnitude(value)
    assert 0 <= encoded < 1 << 16
    assert bs._from_sign_magnitude(encoded) == value


def test_sign_magnitude_clamps_the_magnitude():
    assert bs._from_sign_magnitude(bs._to_sign_magnitude(-100000)) == -32767


class DeadServo:
    def move_to(self, degrees, speed=None, acceleration=None):
        raise TimeoutError("no reply from servo")


def test_sprayer_reports_a_servo_that_stops_answering():
    sprayer = bs.Sprayer(DeadServo(), period=0.01, hold=0.0)
    sprayer.start(count=3)
    sprayer.wait()
    assert not sprayer.running
    assert isinstance(sprayer.error, TimeoutError) and sprayer.strokes == 0
    sprayer.request_stop()                      # never blocks, also after the thread ended


class ControlOnly(FakeControl):
    """Like the real RTDEControlInterface: no protective / emergency stop state."""
    isProtectiveStopped = property(lambda self: (_ for _ in ()).throw(AttributeError("isProtectiveStopped")))
    isEmergencyStopped = property(lambda self: (_ for _ in ()).throw(AttributeError("isEmergencyStopped")))


class Receive:
    def __init__(self):
        self.protective_stop = False

    def isProtectiveStopped(self):
        return self.protective_stop

    def isEmergencyStopped(self):
        return False


def test_watchdog_reads_the_stop_state_from_the_receive_interface():
    c, r = ControlOnly(), Receive()
    watchdog = RobotWatchdog(c, r)
    c.running, r.protective_stop = False, True
    assert watchdog.kick() is False                 # no AttributeError, waits for the clear
    assert "reupload" not in c.calls
    r.protective_stop = False
    assert watchdog.kick() is False                 # re-uploads now
    assert watchdog.kick() is True
