"""Suction driver against a fake controller that answers like Gripper/src/main.cpp."""

import time

import pytest
import serial

import suction as S


class FakeController:
    """Serial port stand-in replying like handleCommand() in the firmware."""

    def __init__(self, *args, **kwargs):
        self.rx = b""
        self.state = "IDLE"
        self.sensor = True
        self.fail_writes = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        data, self.rx = self.rx[:n], self.rx[n:]
        return data

    def say(self, line):
        self.rx += line.encode() + b"\n"

    def write(self, data):
        if self.fail_writes:
            raise serial.SerialException("port gone")
        command = data.decode().strip()
        if command == "GRIP":
            if self.state != "IDLE":
                return self.say("ERR BUSY")
            self.state = "GRIPPING"
            self.say("DONE GRIP")
            if not self.sensor:
                self.say("GRIP UNKNOWN")
        elif command == "RELEASE":
            if self.state == "RELEASING":
                return self.say("ERR BUSY")
            self.state = "RELEASING"

    def end_release(self):
        self.state = "IDLE"
        self.say("DONE RELEASE")

    def close(self):
        pass


@pytest.fixture
def suction(monkeypatch):
    monkeypatch.setattr(S.serial, "Serial", FakeController)
    monkeypatch.setattr(S, "sleep", lambda seconds: None)   # skip the 2 s controller reset
    return S.Suction()


def test_grip_during_release_pulse_is_refused_without_blocking(suction):
    assert suction.release()
    start = time.monotonic()
    assert suction.grip() is False
    assert time.monotonic() - start < 0.05
    assert suction.release() is False          # a second release is ignored too
    suction._serial.end_release()
    assert suction.grip() is True


def test_lost_done_release_does_not_refuse_forever(suction):
    suction.release()
    suction._release_started -= S.RELEASE_TIMEOUT + 0.1
    assert suction.grip() is True


def test_grip_result_follows_the_controller(suction):
    fw = suction._serial
    suction.grip()
    assert suction.grip_result() is None       # DONE GRIP, no report yet
    fw.say("GRIP OK")
    assert suction.grip_result() == "OK"
    fw.say("GRIP LOST")
    assert suction.grip_result() == "LOST"


def test_err_busy_keeps_the_session_result(suction):
    """AUDIT #3: a GRIP while gripping (ERR BUSY) must not clear the last report."""
    fw = suction._serial
    fw.sensor = False
    suction.grip()
    assert suction.grip_result() == "UNKNOWN"
    suction.grip()                             # vacuum already on -> ERR BUSY
    assert suction.grip_result() == "UNKNOWN"
    fw.sensor = True
    fw.say("GRIP LOST")
    suction.grip()
    assert suction.grip_result() == "LOST"
    fw.say("GRIP OK")                          # re-sealed on the next glass
    assert suction.grip_result() == "OK"


def test_new_session_clears_a_stale_result(suction):
    fw = suction._serial
    suction.grip()
    fw.say("GRIP OK")
    assert suction.grip_result() == "OK"
    fw.state = "IDLE"                          # controller reset behind our back
    suction.grip()
    assert suction.grip_result() is None
    suction.release()
    assert suction.grip_result() is None


def test_key_command_warns_instead_of_raising(suction, capsys):
    suction.release()
    assert S.key_command(suction, "g").startswith("release pulse still running")
    suction._serial.end_release()
    assert S.key_command(suction, "g") == "vacuum on"
    suction._serial.fail_writes = True
    assert S.key_command(suction, "r").startswith("gripper not responding")
    assert "WARNING" in capsys.readouterr().out
