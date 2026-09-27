"""
Motion watchdog: the robot stops by itself when the Python main loop stalls.

speedL keeps executing the last velocity until the next command or speedStop,
so a loop that hangs (camera read blocking, a USB glitch, a debugger
breakpoint) would otherwise leave the robot moving. ur_rtde's setWatchdog()
makes the control script call rtde_set_watchdog(..., "stop"): when no RTDE
input arrives for 1 / WATCHDOG_MIN_FREQUENCY seconds, the controller stops
the control script and with it every motion. Every command sent counts as
input; kick() covers the loop iterations that send nothing.

Things to know (checked in the ur_rtde 1.6 sources):
  - A blocking move (moveJ / moveL without async=True) sends nothing while it
    runs, so it trips the watchdog. Use async moves and keep kicking.
  - Anything else that blocks the loop longer than the period (e.g. averaging
    several camera frames) must call kick() in between.
  - The watchdog belongs to the control script: reuploadScript() removes it,
    call arm() again afterwards.
  - After a trip the control script is gone and every command fails. The next
    kick() re-uploads it, re-arms and returns False, so the caller aborts its
    task and tells the user. The robot does not move again on its own.
  - During a protective stop or e-stop kick() returns False until it is
    cleared on the pendant, then re-uploads the script as above. Those two
    states are only readable from RTDEReceiveInterface, hence rtde_r.
  - A trip shows on the pendant as protective stop C207A0 "fieldbus input
    disconnected". kick() prints a WARNING for every gap longer than
    SLOW_KICK_FRACTION of the period, so the console shows how long the loop
    stalled and where (the lines printed just before). Stalls from outside
    the code on Windows: cv2.waitKey(1) (blocked ~480 ms every 1-2 s here,
    use cv2.pollKey()), dragging / resizing the OpenCV window (HighGUI blocks
    while it moves) and a selection in the console (QuickEdit blocks the next
    print until Esc).

Usage:
  watchdog = RobotWatchdog(rtde_c, rtde_r)   # right before the main loop
  while True:
      if not watchdog.kick(): ...    # once per iteration; False = robot was stopped

Requires: pip install ur_rtde
"""

import time

# 1 / this = 0.2 s without input stops the robot. One camera frame alone
# takes 33 ms at 30 fps, plus detection and drawing, so this leaves room for
# hiccups; at the max jog speed (0.08 m/s) a stall moves the tool < 2 cm.
WATCHDOG_MIN_FREQUENCY = 5.0   # Hz
SLOW_KICK_FRACTION = 0.6       # a gap between kicks longer than this share of the period is reported


class RobotWatchdog:
    def __init__(self, rtde_c, rtde_r, min_frequency=WATCHDOG_MIN_FREQUENCY, clock=time.monotonic):
        self._c = rtde_c
        self._r = rtde_r
        self._min_frequency = min_frequency
        self._clock = clock
        self._waiting_for_clear = False
        self.arm()

    def arm(self):
        """Enable the watchdog. Needed again after every reuploadScript()."""
        if not self._c.setWatchdog(self._min_frequency):
            raise RuntimeError("could not enable the RTDE watchdog")
        self._last_kick = self._clock()

    def kick(self):
        """Call once per loop iteration. False = the robot was stopped, abort any task."""
        now = self._clock()
        gap = now - self._last_kick
        self._last_kick = now
        if gap > SLOW_KICK_FRACTION / self._min_frequency and not self._waiting_for_clear:
            print(f"WARNING: {gap * 1000:.0f} ms since the last watchdog kick "
                  f"(the robot stops after {1000 / self._min_frequency:.0f} ms)")
        # isProgramRunning first: kickWatchdog() on a dead script prints an error every call
        if self._c.isProgramRunning() and self._c.kickWatchdog():
            self._waiting_for_clear = False
            return True
        if self._r.isProtectiveStopped() or self._r.isEmergencyStopped():
            if not self._waiting_for_clear:
                print("Robot is protective / emergency stopped, clear it on the pendant")
                self._waiting_for_clear = True
            return False
        print("Robot control script stopped (main loop stalled?), re-uploading it")
        if not self._c.reuploadScript():
            raise RuntimeError("could not re-upload the RTDE control script")
        self.arm()
        self._waiting_for_clear = False
        return False
