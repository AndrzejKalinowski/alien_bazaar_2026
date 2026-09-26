"""Minimal client for the bipropellant binary machine protocol.

Protocol reference: https://github.com/bipropellant/bipropellant-protocol
Frame layout and checksum match protocol.h / protocolfunctions.c and the
official examples/debugMachineProtocol.py reference client:

    SOM(0x00) cmd CI len [ COBS/R encoded: code, payload..., checksum ]

`checksum` is chosen so the byte-sum of the whole decoded frame (SOM, cmd,
CI, len, code, payload, checksum) mod 256 is zero, with `len` still holding
the raw payload length at that point; `len` is then overwritten with the
length of the COBS/R encoded part. Checked byte for byte against the
reference client in tests/test_bipropellant_serial.py.

Only WRITEVAL is implemented (ACK not requested) since that's all a gamepad
teleop needs: push PWM values continuously, don't block on replies.
"""

import random
import struct

import serial
from cobs import cobsr

# param codes registered in protocolfunctions.c
PARAM_ENABLE = 0x09
PARAM_PWM = 0x0D

CMD_WRITEVAL = ord("W")


class BipropellantSerial:
    def __init__(self, port: str, baudrate: int = 115200):
        self._ser = serial.Serial(port, baudrate=baudrate)
        self._ci = random.randrange(254)

    def close(self):
        self._ser.close()

    def _next_ci(self) -> int:
        self._ci = (self._ci + 1) % 254
        return self._ci + 1  # CI ranges 1..255, never 0

    def _write_message(self, code: int, data: bytes):
        msg = bytearray()
        msg += b"\x00"                      # SOM
        msg.append(CMD_WRITEVAL)            # cmd, no ACK requested
        msg.append(self._next_ci())         # CI
        msg.append(len(data))               # placeholder, overwritten below
        msg.append(code)
        msg += data

        checksum = (256 - (sum(msg) % 256)) % 256
        msg.append(checksum)

        encoded = msg[:4] + cobsr.encode(bytes(msg[4:]))
        encoded[3] = len(encoded) - 4

        self._ser.write(bytes(encoded))

    def set_enabled(self, enabled: bool):
        """Write PARAM_ENABLE (0x09). Motor output is forced to zero unless
        this is set, regardless of PWM/speed control mode."""
        self._write_message(PARAM_ENABLE, bytes([1 if enabled else 0]))

    def set_pwm(
        self,
        pwm_left: int,
        pwm_right: int,
        max_power: int = 600,
        min_power: int = -600,
        minimum_pwm: int = 40,
    ):
        """Write PROTOCOL_PWM_DATA (code 0x0D): switches the board into
        CONTROL_TYPE_PWM and drives the motors directly at the given PWM
        values (roughly -1000..1000, clamped/deadzoned on the firmware side
        by max_power/min_power/minimum_pwm)."""
        data = struct.pack(
            "<iiiii", pwm_left, pwm_right, max_power, min_power, minimum_pwm
        )
        self._write_message(PARAM_PWM, data)
