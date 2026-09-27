"""
Motion controller with a software ceiling. Scripts use it instead of calling
rtde_control.RTDEControlInterface directly.

SafeControl wraps an RTDEControlInterface and checks every command that moves
the arm against MAX_TCP_Z (height of the TCP, the suction-cup tip, above the
robot base):
  moveL    refused if the target is above the ceiling. The TCP moves in a
           straight line, so a target under the ceiling keeps the whole path
           under it (starting above it, the move only goes down).
  moveJ    the joints move in a straight line in joint space, so the TCP moves
           along an arc that can rise above both ends. The arc is sampled at
           MOVEJ_CHECK_STEPS points with forward kinematics; refused if the
           target or any point is above the ceiling (or, starting above the
           ceiling, if the arc rises above the start).
  speedL   the upward speed is capped to CEILING_JOG_GAIN * distance left to the
           ceiling, so jogging and servoing slow down and stop under it; above
           it only downward motion is allowed. Never refused, so the arm can
           always be jogged back down. time must be > 0 (time=0 makes the robot
           protective-stop, C271A1).
Other motion commands (servoL, speedJ, moveP, forceMode, ...) are refused
because they are not checked. Everything else (stopL, speedStop, zeroFtSensor,
teachMode, getAsyncOperationProgress, ...) is passed through unchanged.

A refused command raises MotionRefused before anything is sent to the robot, so
the arm keeps doing what it was doing: the caller must catch it and stop.

over_ceiling() is the backstop for what the checks can miss (the arc between
moveJ samples, blends, freedrive): call it every loop and abort the running
task when it is True.

What this does NOT do:
  * It is not a safety function. A bug, a direct rtde_control call or a script
    that does not use this module bypasses it. Also set a safety plane on the
    pendant (Installation > Safety > Planes); the controller enforces that one.
  * Only the TCP is limited. With the tool pointing down, the flange, camera
    and wrist are above the TCP, and the elbow can be higher still.
  * Freedrive (teachMode) is not limited.

Usage:
  import safe_motion
  r, c = safe_motion.connect(IP)       # RTDEReceiveInterface, SafeControl
  c.moveL(pose, speed, accel, True)    # same calls as RTDEControlInterface

Requires: pip install ur_rtde numpy
"""

import numpy as np

# --- ceiling --------------------------------------------------------------------
MAX_TCP_Z = 0.85             # m in base frame, the TCP never goes higher than this (was 0.60, raised on request)
CEILING_MARGIN = 0.02        # m, over_ceiling() is True this far above MAX_TCP_Z
CEILING_JOG_GAIN = 2.0       # 1/s, speedL upward speed is capped to gain * distance left to the ceiling
MOVEJ_CHECK_STEPS = 20       # forward-kinematics samples along a moveJ arc

# Commands that move the arm but are not checked here
UNCHECKED_MOTION = {"moveC", "moveP", "movePath", "moveJ_IK", "moveL_FK", "moveUntilContact",
                    "servoJ", "servoL", "servoC", "speedJ", "jogStart", "forceMode"}


class MotionRefused(Exception):
    """A motion command was not sent because it would take the TCP above the ceiling."""


def _is_path(arg):
    """moveL / moveJ also take a list of waypoints, which are not checked."""
    return len(arg) > 0 and np.ndim(arg[0]) > 0


class SafeControl:
    """RTDEControlInterface with the moves checked against MAX_TCP_Z."""

    def __init__(self, rtde_c, rtde_r):
        self._c = rtde_c
        self._r = rtde_r

    def __getattr__(self, name):
        if name in UNCHECKED_MOTION:
            raise MotionRefused(f"{name} is not checked against the ceiling, use moveL / moveJ / speedL")
        return getattr(self._c, name)

    def tcp_z(self):
        return self._r.getActualTCPPose()[2]

    def over_ceiling(self):
        """True when the TCP is more than CEILING_MARGIN above MAX_TCP_Z."""
        return self.tcp_z() > MAX_TCP_Z + CEILING_MARGIN

    def moveL(self, pose, *args, **kwargs):
        if _is_path(pose):
            raise MotionRefused("moveL with a path is not checked against the ceiling")
        if pose[2] > MAX_TCP_Z:
            raise MotionRefused(f"moveL target z {pose[2]:.3f} m is above the ceiling ({MAX_TCP_Z:.2f} m)")
        return self._c.moveL(pose, *args, **kwargs)

    def moveJ(self, q, *args, **kwargs):
        if _is_path(q):
            raise MotionRefused("moveJ with a path is not checked against the ceiling")
        highest, end = self._moveJ_heights(q)
        if end > MAX_TCP_Z or highest > max(MAX_TCP_Z, self.tcp_z()):
            raise MotionRefused(f"moveJ would take the TCP to z {highest:.3f} m, "
                                f"above the ceiling ({MAX_TCP_Z:.2f} m)")
        return self._c.moveJ(q, *args, **kwargs)

    def speedL(self, xd, acceleration=0.25, time=0.0):
        if time <= 0:
            raise ValueError("speedL needs time > 0 (time=0 protective-stops the robot, C271A1)")
        xd = list(xd)
        xd[2] = min(xd[2], max(0.0, CEILING_JOG_GAIN * (MAX_TCP_Z - self.tcp_z())))
        return self._c.speedL(xd, acceleration, time)

    def getForwardKinematics(self, q=None, tcp_offset=None):
        """TCP pose at joints q (default: the actual ones) with the active TCP offset.

        ur_rtde 1.6.5 getForwardKinematics(q) without tcp_offset is wrong: its robot
        script (cmd 45) always reads the offset from input registers 6-11, which then
        hold whatever the previous command left there (a moveL's speed and accel...).
        A wrist-3 flip with the TCP on the wrist axis came out 0.45 m high. So the
        active offset is always passed explicitly.
        """
        if q is None:
            return self._c.getForwardKinematics()
        if tcp_offset is None:
            tcp_offset = self._c.getTCPOffset()
        return self._c.getForwardKinematics(list(q), list(tcp_offset))

    def _moveJ_heights(self, q):
        """Highest and final TCP z along a moveJ to q."""
        q0 = np.asarray(self._r.getActualQ(), dtype=float)
        q1 = np.asarray(q, dtype=float)
        tcp_offset = self._c.getTCPOffset()
        z = [self.getForwardKinematics(q0 + t * (q1 - q0), tcp_offset)[2]
             for t in np.linspace(0, 1, MOVEJ_CHECK_STEPS + 1)[1:]]
        return max(z), z[-1]


def connect(ip):
    """(RTDEReceiveInterface, SafeControl) for the robot at ip. The arm can move from here on."""
    # Imported here so the checks above can be tested without ur_rtde
    import rtde_control
    import rtde_receive
    r = rtde_receive.RTDEReceiveInterface(ip)
    return r, SafeControl(rtde_control.RTDEControlInterface(ip), r)
