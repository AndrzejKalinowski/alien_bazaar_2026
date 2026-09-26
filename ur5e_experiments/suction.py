"""
Suction cup gripper control over the XIAO C3's serial protocol.

Sends "GRIP\n" / "RELEASE\n" as UTF-8 at 115200 baud
(8 data bits, no parity, 1 stop bit, no flow control).

After GRIP the controller reports whether an object was picked up, based on
the vacuum measured by its pressure sensor: "GRIP OK" when the object is held,
"GRIP FAIL" if nothing is held within 8 s, "GRIP LOST" if the object drops
while gripping, and "GRIP UNKNOWN" if the sensor is missing.
"""

import serial
from time import monotonic, sleep

PORT = "COM9"
BAUDRATE = 115200
GRIP_RESULT_TIMEOUT = 10.0  # the controller itself reports GRIP FAIL after 8 s


class Suction:
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self._serial = serial.Serial(port, baudrate, timeout=0)
        self._buffer = b""
        sleep(2)  # let the controller reset after opening the serial port

    def grip(self):
        """Switch the vacuum on without waiting for the result."""
        self._serial.reset_input_buffer()
        self._buffer = b""
        self._serial.write(b"GRIP\n")

    def wait_for_grip(self, timeout=GRIP_RESULT_TIMEOUT):
        """Wait for the result of the last grip().

        Returns "OK", "FAIL", "LOST" or "UNKNOWN", or None on timeout.
        """
        line = self._wait_for_line(("GRIP ", "ERR"), timeout)
        if line is None or line.startswith("ERR"):
            return None
        return line.split()[1]

    def grip_and_wait(self, timeout=GRIP_RESULT_TIMEOUT):
        """Grip and return True once the object is held, False otherwise.

        On False the vacuum stays on; call release() to switch it off.
        """
        self.grip()
        return self.wait_for_grip(timeout) == "OK"

    def is_holding(self, timeout=1.0):
        """Ask the controller whether an object is held right now."""
        self._serial.write(b"HOLD\n")
        return self._wait_for_line(("HOLD ", "ERR"), timeout) == "HOLD YES"

    def release(self):
        self._serial.write(b"RELEASE\n")

    def status(self):
        self._serial.write(b"STATUS\n")

    def close(self):
        self._serial.close()

    def _wait_for_line(self, prefixes, timeout):
        """Return the first reply line starting with one of prefixes."""
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            self._buffer += self._serial.read(self._serial.in_waiting or 1)
            while b"\n" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line.startswith(prefixes):
                    return line
            sleep(0.01)
        return None
