"""
Suction cup gripper control over the XIAO C3's serial protocol
(see Gripper/README.md and Gripper/src/main.cpp).

Sends uppercase commands terminated by "\n" as UTF-8 at 115200 baud
(8 data bits, no parity, 1 stop bit, no flow control).

After GRIP the controller reports whether an object was picked up, based on
the vacuum measured by its pressure sensor: "GRIP OK" when the object is held,
"GRIP FAIL" if nothing is held within 8 s (the vacuum stays on, and "GRIP OK"
can still follow), "GRIP LOST" if the object drops while gripping, and
"GRIP UNKNOWN" if the sensor is missing or stops giving valid readings
while gripping (the tasks then carry on blind, as without a sensor).

RELEASE fires a 1.5 s release pulse and replies "DONE RELEASE" when it ends.
During the pulse the controller answers GRIP and RELEASE with "ERR BUSY", so
grip() and release() refuse (return False, send nothing) while a pulse is
still running: they are called from video loops that must not block for 1.5 s.
Tasks retry grip() every step until it goes through; grip_and_wait() waits
for the pulse instead.

A repeated GRIP while already gripping returns "ERR BUSY" and changes
nothing: the vacuum stays on with the first GRIP's baseline, and GRIP OK /
LOST still follow on every change of the held state. So grip_result() keeps
the last report until the controller confirms a new session with
"DONE GRIP" (or release() is called), and a pick after a manual g works.
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
        self._release_started = 0.0
        self._grip_result = None
        sleep(2)  # let the controller reset after opening the serial port

    def release_pending(self):
        """Non-blocking: True while the release pulse is running."""
        self._read_lines(())
        if self._release_pending and monotonic() - self._release_started > RELEASE_TIMEOUT:
            self._release_pending = False  # "DONE RELEASE" was lost, don't refuse forever
        return self._release_pending

    def grip(self):
        """Switch the vacuum on without waiting for the result.

        Returns False and sends nothing while a release pulse is still
        running (the controller would answer ERR BUSY); try again later.
        """
        if self.release_pending():
            return False
        # The result is not cleared here but on "DONE GRIP": if the vacuum is
        # already on, the controller answers ERR BUSY and keeps its grip
        # session (and baseline), so the last report (UNKNOWN, LOST, OK)
        # stays valid. Clearing it would wait forever for a GRIP OK that
        # only comes on a change of the held state.
        self._serial.write(b"GRIP\n")
        return True

    def grip_result(self):
        """Non-blocking: the latest grip report of the current grip session.

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
        self.wait_for_release()
        self.grip()
        return self.wait_for_grip(timeout) == "OK"

    def release(self, wait=False):
        """Switch the vacuum off and fire the release pulse.

        Returns False and sends nothing if a release pulse is already running.
        With wait=True, blocks until the pulse ends and returns True on
        "DONE RELEASE"; otherwise returns True right away.
        """
        if self.release_pending():
            return False
        self._serial.write(b"RELEASE\n")
        self._grip_result = None
        self._release_pending = True
        self._release_started = monotonic()
        if wait:
            return self.wait_for_release()
        return True

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
            elif line == "DONE GRIP":
                self._grip_result = None   # a new grip session with a fresh baseline
            elif line.startswith("GRIP "):
                self._grip_result = line.split()[1]
            if prefixes and line.startswith(prefixes):
                return line
        return None


def key_command(suction, key):
    """The g (grip) / r (release) operator keys, for the video loops.

    Prints and returns a status line. A command that can't be carried out is
    refused with a warning instead of blocking or raising, so the loop (and
    the robot watchdog) keeps running.
    """
    try:
        if key == "g":
            if suction.grip():
                print("vacuum on")
                return "vacuum on"
            message = "release pulse still running, grip refused - try again in a moment"
        else:
            if suction.release():
                print("released")
                return "released"
            message = "release pulse already running, release ignored"
    except (serial.SerialException, OSError) as e:
        message = f"gripper not responding ({e}), command not sent"
    print(f"WARNING: {message}")
    return message


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
