"""Test stand-ins for RTDE receive + control and the gripper/servo workers.

FakeRobot moves only the TCP pose (no kinematics): an async moveL goes
straight to its target at its speed, moveJ turns the joints (TCP stays), a
speedL keeps its velocity until the next command or speedStop, exactly the
behaviour the watchdog is there for. Force comes from force_fn(pose).
advance(dt) is driven by the test's virtual clock. Nothing opens a port.
"""

from math import pi

import numpy as np

from system_model import Grip, Outcome, Result

SIDE = [0.0, pi / 2, 0.0]   # axis-angle: tool z along base +x (horizontal side grip)


class FakeRobot:
    def __init__(self, pose=(0.3, -0.5, 0.35, *SIDE), q=(0, -1.57, 1.57, -1.57, -1.57, 0.5)):
        self.pose = list(pose)
        self.q = list(q)
        self.target = None          # (pose, speed)
        self.joint_target = None
        self.velocity = np.zeros(3)
        self.force_fn = lambda pose: 0.0
        self.force_offset = 0.0
        self.stop_short = 0.0       # m, moveL ends this far before its target
        self.calls = []
        self.connected = True
        self.program_running = True
        self.protective_stop = False
        self.tcp_offset = [0, 0, 0.2, 0, 0, 0]
        self.speed = 0.0

    # --- simulation ---------------------------------------------------------------
    def advance(self, dt):
        position = np.asarray(self.pose[:3], dtype=float)
        before = position.copy()
        if self.target is not None:
            goal = np.asarray(self.target[0][:3], dtype=float)
            delta = goal - position
            distance = np.linalg.norm(delta)
            step = self.target[1] * dt
            end = distance - self.stop_short
            if step >= end:
                position = position + (delta / distance * end if distance else 0)
                self.pose[3:] = list(self.target[0][3:])
                self.target = None
            else:
                position = position + delta / distance * step
        elif self.joint_target is not None:
            self.q = list(self.joint_target)
            self.joint_target = None
        elif np.any(self.velocity):
            position = position + self.velocity * dt
        self.pose[:3] = list(position)
        self.speed = float(np.linalg.norm(position - before) / dt) if dt else 0.0

    # --- receive ------------------------------------------------------------------
    def getActualTCPPose(self):
        return list(self.pose)

    def getActualQ(self):
        return list(self.q)

    def getActualTCPSpeed(self):
        return [self.speed, 0, 0, 0, 0, 0]

    def getActualTCPForce(self):
        return [self.force_fn(self.pose) - self.force_offset, 0, 0, 0, 0, 0]

    def isConnected(self):
        return self.connected

    # --- control ------------------------------------------------------------------
    def moveL(self, pose, speed, accel, asynchronous=False):
        self.calls.append(("moveL", list(pose)))
        self.velocity[:] = 0
        self.target = (list(pose), speed)
        return True

    def moveJ(self, q, speed, accel, asynchronous=False):
        self.calls.append(("moveJ", list(q)))
        self.joint_target = list(q)
        return True

    def getForwardKinematics(self, q=None):
        return list(self.pose)

    def speedL(self, xd, accel, time):
        assert time > 0, "speedL time must be > 0"
        self.calls.append(("speedL", list(xd)))
        self.velocity = np.asarray(xd[:3], dtype=float)

    def speedStop(self, accel=10.0):
        self.calls.append(("speedStop",))
        self.velocity[:] = 0

    def stopL(self, accel=10.0, asynchronous=False):
        self.calls.append(("stopL", asynchronous))
        self.target = None

    def stopJ(self, accel=10.0, asynchronous=False):
        self.calls.append(("stopJ", asynchronous))
        self.joint_target = None

    def getAsyncOperationProgress(self):
        return 0 if (self.target is not None or self.joint_target is not None) else -1

    def zeroFtSensor(self):
        self.force_offset = self.force_fn(self.pose)

    def getTCPOffset(self):
        return list(self.tcp_offset)

    def setWatchdog(self, frequency):
        return True

    def kickWatchdog(self):
        return self.program_running and not self.protective_stop

    def isProgramRunning(self):
        return self.program_running

    def isProtectiveStopped(self):
        return self.protective_stop

    def isEmergencyStopped(self):
        return False

    def reuploadScript(self):
        self.program_running = True
        return True

    def teachMode(self):
        self.calls.append(("teachMode",))

    def endTeachMode(self):
        self.calls.append(("endTeachMode",))

    def stopScript(self):
        self.calls.append(("stopScript",))


class FakeGripper:
    """GripperWorker interface; jobs finish when the test's result() is read."""

    def __init__(self, clock, holds=True):
        self.clock = clock
        self.holds = holds
        self.grip = Grip.NO
        self.jobs = []
        self.stops = []
        self._results = {}
        self.fault = ""

    def snapshot(self):
        return {"connected": True, "fault": self.fault, "observed_at": self.clock(), "grip": self.grip}

    def submit(self, job_id, kind, **params):
        self.jobs.append(kind)
        if kind == "grip":
            self.grip = Grip.OK if self.holds else Grip.UNKNOWN
        elif kind == "release":
            self.grip = Grip.NO
        self._results[job_id] = Result(job_id, Outcome.SUCCEEDED)

    def result(self, job_id):
        return self._results.pop(job_id, None)

    def request_stop(self, stop_id):
        self.stops.append(stop_id)

    def close(self):
        pass


class FakeServos:
    def __init__(self, clock):
        self.clock = clock
        self.jobs = []
        self.stops = []
        self._results = {}
        self.fail = None

    def snapshot(self):
        return {"connected": True, "fault": "", "observed_at": self.clock(), "stopped": True,
                "moving": {}, "errors": {"SPRAYER": [], "SPONGE": []}}

    def submit(self, job_id, kind, **params):
        self.jobs.append((kind, params.get("profile")))
        outcome = Outcome.FAILED if kind == self.fail else Outcome.SUCCEEDED
        self._results[job_id] = Result(job_id, outcome, "injected" if self.fail == kind else "")

    def result(self, job_id):
        return self._results.pop(job_id, None)

    def request_stop(self, stop_id):
        self.stops.append(stop_id)
        self._results[stop_id] = Result(stop_id, Outcome.SUCCEEDED)

    def close(self):
        pass
