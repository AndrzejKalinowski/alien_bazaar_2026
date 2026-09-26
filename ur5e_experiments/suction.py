"""
Suction cup gripper control over the XIAO C3's serial protocol
(see Gripper/README.md and Gripper/src/main.cpp).

Sends uppercase commands terminated by "\n" as UTF-8 at 115200 baud
(8 data bits, no parity, 1 stop bit, no flow control).

After GRIP the controller reports whether an object was picked up, based on
the vacuum measured by its pressure sensor: "GRIP OK" when the object is held,
"GRIP FAIL" if nothing is held within 8 s (the vacuum stays on, and "GRIP OK"
can still follow), "GRIP LOST" if the object drops while gripping, and
"GRIP UNKNOWN" if the sensor is missing.

RELEASE fires a 1.5 s release pulse and replies "DONE RELEASE" when it ends.
During the pulse the controller answers GRIP and RELEASE with "ERR BUSY", so
grip() waits for a pending release to finish first. A repeated GRIP while
already gripping also returns "ERR BUSY".
"""

import serial
from time import monotonic, sleep

PORT = "COM9"
BAUDRATE = 115200
GRIP_RESULT_TIMEOUT = 10.0  # the controller itself reports GRIP FAIL after 8 s
RELEASE_TIMEOUT = 3.0       # the release pulse lasts 1.5 s
REPLY_TIMEOUT = 1.0


class Suction:
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self._serial = serial.Serial(port, baudrate, timeout=0)
        self._buffer = b""
        self._release_pending = False
        self._grip_result = None
        sleep(2)  # let the controller reset after opening the serial port

    def grip(self):
        """Switch the vacuum on without waiting for the result.

        If a release pulse is still running, waits for it to end first
        (at most RELEASE_TIMEOUT), since the controller rejects GRIP meanwhile.
        """
        if self._release_pending:
            self.wait_for_release()
        self._serial.reset_input_buffer()
        self._buffer = b""
        self._grip_result = None
        self._serial.write(b"GRIP\n")

    def grip_result(self):
        """Non-blocking: the latest grip report since the last grip().

        Returns "OK", "FAIL", "LOST" or "UNKNOWN", or None if nothing was
        reported yet. After "FAIL" the vacuum stays on, and "OK" can still
        follow if the object seals late.
        """
        self._read_lines(())
        return self._grip_result

    def wait_for_grip(self, timeout=GRIP_RESULT_TIMEOUT):
        """Wait for the result of the last grip().

        Returns "OK", "FAIL", "LOST" or "UNKNOWN", or None on timeout or if
        the controller rejected the command (e.g. ERR BUSY when already
        gripping).
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

    def release(self, wait=False):
        """Switch the vacuum off and fire the release pulse.

        With wait=True, blocks until the pulse ends and returns True on
        "DONE RELEASE"; otherwise returns immediately.
        """
        self._serial.write(b"RELEASE\n")
        self._release_pending = True
        if wait:
            return self.wait_for_release()

    def wait_for_release(self, timeout=RELEASE_TIMEOUT):
        """Wait for the running release pulse to end. Returns True on success."""
        if self._release_pending:
            self._wait_for_line(("DONE RELEASE",), timeout)
        done = not self._release_pending
        self._release_pending = False  # don't block again on a lost reply
        return done

    def is_holding(self, timeout=REPLY_TIMEOUT):
        """Ask the controller whether an object is held right now.

        Returns True or False, or None if the pressure sensor is missing
        or there was no reply.
        """
        self._serial.write(b"HOLD\n")
        line = self._wait_for_line(("HOLD ", "ERR"), timeout)
        if line is None or line.startswith("ERR"):
            return None
        return line == "HOLD YES"

    def status(self, timeout=REPLY_TIMEOUT):
        """Return the controller state: "IDLE", "GRIPPING" or "RELEASING"."""
        self._serial.write(b"STATUS\n")
        line = self._wait_for_line(("STATE ",), timeout)
        return None if line is None else line.split()[1]

    def pressure(self, timeout=REPLY_TIMEOUT):
        """Read the pressure sensor, for calibrating the hold thresholds.

        Returns (pressure, baseline, vacuum) in hPa; baseline and vacuum are
        None unless gripping. Returns None if the sensor is missing.
        """
        self._serial.write(b"PRESSURE\n")
        line = self._wait_for_line(("PRESSURE ", "ERR"), timeout)
        if line is None or line.startswith("ERR"):
            return None
        values = line.split()
        pressure = float(values[1])
        if len(values) >= 6:
            return pressure, float(values[3]), float(values[5])
        return pressure, None, None

    def close(self):
        self._serial.close()

    def _wait_for_line(self, prefixes, timeout):
        """Return the first reply line starting with one of prefixes.

        Other lines are dropped, since the controller sends GRIP ... messages
        on its own between replies.
        """
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            line = self._read_lines(prefixes)
            if line is not None:
                return line
            sleep(0.01)
        return None

    def _read_lines(self, prefixes):
        """Process the lines received so far, without waiting.

        Returns the first line starting with one of prefixes (later lines stay
        in the buffer), or None.
        """
        self._buffer += self._serial.read(self._serial.in_waiting)
        while b"\n" in self._buffer:
            raw, self._buffer = self._buffer.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if line == "DONE RELEASE":
                self._release_pending = False
            elif line.startswith("GRIP "):
                self._grip_result = line.split()[1]
            if prefixes and line.startswith(prefixes):
                return line
        return None


if __name__ == "__main__":
    suction = Suction()
    try:
        print("state:", suction.status())
        print("pressure:", suction.pressure())
        print("grip result:", "OK" if suction.grip_and_wait() else "not held")
        print("pressure:", suction.pressure())
        print("holding:", suction.is_holding())
        print("released:", suction.release(wait=True))
    finally:
        suction.close()
