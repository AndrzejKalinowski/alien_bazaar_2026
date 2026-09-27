"""Deterministic, in-memory devices for supervisor development without hardware.

This is a logical simulator with station-box/tool-envelope path checks. Its
example layout is FICTIONAL and must never be used as measured robot geometry.
It does not simulate UR links, physical forces or actual motion tracking. PICK
removes a target from the input scene; FLIP records mouth-down orientation;
LOWER confirms support and RELEASE records the occupied output. stop preserves
vacuum and invalidates the pending command. Optional failures occur before
the requested operation's effects. No camera, serial or RTDE libraries used.
Requires: Python standard library.
"""

from math import isfinite

from system_geometry import Box, CollisionRefused, STATION_ACCESS, Station, Workspace
from system_model import Grip, Outcome, Result, Scene, Step, Telemetry
from system_settings import SIM_OPERATION_TIME

SIM_TOOL_RADIUS = 0.07    # m, fictional tool + glass sphere for all orientations
SIM_CLEARANCE = 0.02      # m, fictional additional station clearance
SIM_MIN_TCP_Z = 0.0       # m, simulation only
SIM_MAX_TCP_Z = 0.60      # m, simulation only; real adapter must use SafeControl limits
SIM_PICK_Z = 0.10         # m, fictional glass contact height
SIM_OUTPUT_Z = 0.10       # m, fictional output contact height
SIM_OUTPUT_APPROACH = 0.05  # m above fictional output contact height
SIM_OBSERVATION_POSE = (0.0, -0.4, 0.45)  # m, fictional TCP point


def example_workspace():
    """Three different station heights, for offline tests and the CLI only."""
    stations = []
    # Absolute base-frame heights, not offsets from the camera or TCP (metres).
    for name, x, height in (("sprayer", 0.2, 0.10), ("sponge", 0.8, 0.20), ("wiper", 1.4, 0.30)):
        stations.append(Station(
            name, Box((x - 0.22, 0.1, 0.0), (x + 0.22, 0.5, height)),
            Box((x - 0.14, 0.16, height - 0.07), (x + 0.14, 0.44, height + 0.30)),
            (x, 0.3, height + 0.03)))
    return Workspace(tuple(stations), SIM_TOOL_RADIUS, SIM_CLEARANCE, SIM_MIN_TCP_Z, SIM_MAX_TCP_Z)


class SimulationClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        if not isfinite(seconds) or seconds < 0:
            raise ValueError("simulation time must advance by a finite non-negative duration")
        self.now += seconds


class SimulatedDevices:
    def __init__(self, targets, operation_time=SIM_OPERATION_TIME, fail_at=None, workspace=None):
        if not isfinite(operation_time) or operation_time < 0:
            raise ValueError("operation_time must be finite and non-negative")
        self.targets = {target.id: target for target in targets}
        self.operation_time = operation_time
        self.fail_at = fail_at
        self.workspace = example_workspace() if workspace is None else workspace
        self.tcp = SIM_OBSERVATION_POSE
        self.connected = True
        self.grip = Grip.NO
        self.supported = False
        self.robot_stopped = True
        self.stations_stopped = True
        self.device_fault = ""
        self.orientation = "up"
        self.history = []
        self.paths = []
        self.placements = []
        self.stop_requests = []
        self._pending = None
        self._stopping = None
        self._sequence = 0
        self._prepared = None
        self._active_path = None
        self._output_points = {}

    def telemetry(self, now):
        return Telemetry(now, self.connected, self.grip, self.supported,
                         self.robot_stopped, self.stations_stopped, self.device_fault)

    def observe(self, now):
        self._sequence += 1
        return Scene(self._sequence, now, tuple(self.targets.values()))

    def validate_motion(self, command):
        """Retain the exact checked path; begin refuses stale/unvalidated plans."""
        self._prepared = None
        world, start, step = self.workspace, self.tcp, command.step
        access = STATION_ACCESS.get(step)
        if access:
            station = world.station(access[0])
            if access[1] == "enter":
                path = world.transfer_path(start, station.work_point, step)
            elif access[1] == "exit":
                path = (start, (start[0], start[1], world.transport_height(start)))
            elif step == Step.WIPE:
                path = (start, (start[0] + 0.01, start[1], start[2]), start)
            else:
                path = (start,)
        elif step == Step.PICK:
            path = world.transfer_path(start, (command.target.x, command.target.y, SIM_PICK_Z), step)
        elif step in (Step.LIFT, Step.RETREAT):
            path = (start, (start[0], start[1], world.transport_height(start)))
        elif step == Step.TO_OUTPUT:
            output = self._output_points.setdefault(
                command.slot_id, (0.1 + len(self._output_points) * 0.2, -0.8, SIM_OUTPUT_Z))
            path = world.transfer_path(start, (output[0], output[1], output[2] + SIM_OUTPUT_APPROACH), step)
        elif step == Step.LOWER:
            path = (start, self._output_points[command.slot_id])
        elif step == Step.OBSERVE:
            path = world.transfer_path(start, SIM_OBSERVATION_POSE, step)
        elif step in (Step.FLIP, Step.RELEASE):
            # The sphere includes all intermediate glass orientations during FLIP.
            path = (start,)
        else:
            raise CollisionRefused(f"no motion plan for {step.value}")
        checked = world.validate_path(path, step)
        self._prepared = (command, start, world, checked)

    def begin(self, command):
        if self._pending is not None or self._stopping is not None:
            raise RuntimeError("a device operation is already active")
        if not self.connected:
            raise RuntimeError("simulated devices disconnected")
        if command.step == Step.PICK and command.target.id not in self.targets:
            raise RuntimeError("pick target no longer present")
        if command.step == Step.RELEASE and not self.supported:
            raise RuntimeError("cannot release an unsupported glass")
        if (self._prepared is None or self._prepared[0] != command
                or self._prepared[1] != self.tcp or self._prepared[2] != self.workspace):
            raise CollisionRefused("motion plan must be validated for current pose and layout before dispatch")
        self._active_path = self._prepared[3]
        self._prepared = None
        self._pending = command
        self.history.append(command)
        self.paths.append((command.id, self._active_path))
        stationary = (Step.SPRAY_1, Step.SPONGE, Step.SPRAY_2, Step.RELEASE, Step.OBSERVE)
        self.robot_stopped = command.step in stationary
        self.stations_stopped = command.step not in (Step.SPRAY_1, Step.SPONGE, Step.SPRAY_2)

    def poll(self, command_id, now):
        command = self._pending
        if command is None or command.id != command_id:
            return None
        if now - command.issued_at < self.operation_time:
            return None
        self._pending = None
        self.robot_stopped = self.stations_stopped = True
        if command.step == self.fail_at:
            return Result(command.id, Outcome.FAILED, "injected simulated failure")
        self.tcp = self._active_path[-1]
        self._active_path = None
        if command.step == Step.PICK:
            self.targets.pop(command.target.id)
            self.grip = Grip.OK
            self.orientation = "up"
            self.supported = True
        elif command.step == Step.LIFT:
            self.supported = False
        elif command.step == Step.FLIP:
            self.orientation = "down"
        elif command.step == Step.LOWER:
            self.supported = True
        elif command.step == Step.RELEASE:
            self.grip = Grip.NO
            self.placements.append({"target_id": command.target.id, "slot_id": command.slot_id,
                                    "orientation": self.orientation})
        elif command.step == Step.RETREAT:
            self.supported = False
        return Result(command.id, Outcome.SUCCEEDED)

    def begin_stop(self, stop_id, now):
        self._pending = None  # prevents late completions from changing the world
        self._prepared = self._active_path = None
        self._stopping = (stop_id, now + self.operation_time)
        self.stop_requests.append(stop_id)

    def poll_stop(self, stop_id, now):
        if self._stopping is None or self._stopping[0] != stop_id or now < self._stopping[1]:
            return None
        self._stopping = None
        self.robot_stopped = self.stations_stopped = True
        return Result(stop_id, Outcome.SUCCEEDED)
