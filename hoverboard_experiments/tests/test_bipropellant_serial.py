"""bipropellant framing, byte for byte against the protocol's reference client.

reference_compile_message() is compileMessage() from
https://github.com/bipropellant/bipropellant-protocol examples/debugMachineProtocol.py
(master, checked 2026-09-26), reduced to what WRITEVAL without ACK uses.

Run from the repo root:  python -m pytest hoverboard_experiments/tests
"""

import os
import struct
import sys

import pytest

cobsr = pytest.importorskip("cobs.cobsr")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bipropellant_serial as bp  # noqa: E402


def reference_compile_message(cmd, code, data, send_ci):
    msg = bytearray()
    msg += b"\x00"
    msg.append(ord(cmd))          # ACK = 0
    msg.append(send_ci)
    msg.append(len(data))
    msg.append(code)
    msg += data
    check = (256 - (sum(msg) % 256)) % 256
    msg.append(check)
    msg = msg[:4] + cobsr.encode(bytes(msg[4:]))
    msg[3] = len(msg[4:])
    return bytes(msg)


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))

    def close(self):
        pass


@pytest.fixture
def board(monkeypatch):
    monkeypatch.setattr(bp.serial, "Serial", FakeSerial)
    return bp.BipropellantSerial("COM_TEST")


def sent_ci(frame):
    return frame[2]


@pytest.mark.parametrize("left, right", [(0, 0), (300, -300), (-600, 600), (1, -1)])
def test_pwm_frame_matches_reference(board, left, right):
    board.set_pwm(left, right)
    frame = board._ser.written[-1]
    data = struct.pack("<iiiii", left, right, 600, -600, 40)
    assert frame == reference_compile_message("W", bp.PARAM_PWM, data, sent_ci(frame))


@pytest.mark.parametrize("enabled", [True, False])
def test_enable_frame_matches_reference(board, enabled):
    board.set_enabled(enabled)
    frame = board._ser.written[-1]
    assert frame == reference_compile_message("W", bp.PARAM_ENABLE, bytes([int(enabled)]), sent_ci(frame))


def test_continuity_indicator_is_never_zero(board):
    for _ in range(600):   # wraps around several times
        board.set_enabled(True)
    assert all(1 <= sent_ci(frame) <= 255 for frame in board._ser.written)
