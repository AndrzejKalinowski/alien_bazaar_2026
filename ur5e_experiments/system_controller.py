"""Single-owner, tick-driven supervisor; currently exercised with fake devices.

Call start(), tick(), request_stop(), reset_fault() and confirm_output_cleared()
from ONE owner thread. A future web server must enqueue commands to that owner.
There are no sleeps, hardware imports or reconnects here. The monotonic clock
is injectable so deadlines, STOP and failures can be tested without wall time.

The batch recipe is fixed. A missing target is skipped after three fresh
observations; newly added targets are excluded. Output capacity is reserved
before PICK. STOP never releases vacuum and needs an explicit acknowledgement.
After STOP/fault, reset only restores READY; it never resumes a suspended step.
Drain events after each tick; the in-memory diagnostic buffer is bounded.
Requires: Python standard library.
"""

from collections import deque
from math import isfinite
from time import monotonic
from uuid import uuid4

from system_batch import Batch, OutputRegistry, validate_targets
from system_model import Command, Grip, Outcome, Reply, SlotState, State, Step, TargetState
from system_operations import DeviceAdapter, RECIPE
from system_settings import (FUTURE_TOLERANCE, MAX_PENDING_EVENTS, MAX_START_REQUESTS,
                             SCENE_MAX_AGE, STOP_TIMEOUT, TELEMETRY_MAX_AGE)

MISSING_TARGET_RETRIES = 3  # distinct fresh observations before skipping a target


class Supervisor:
    def __init__(self, devices: DeviceAdapter, slots, clock=monotonic):
        self.devices = devices
        self.clock = clock
        self.outputs = OutputRegistry(slots)
        self.state = State.READY
        self.batch = None
        self.item = None
        self.slot = None
        self.command = None
        self.step_index = 0
        self.fault = ""
        self._holding = False
        self._events = deque(maxlen=MAX_PENDING_EVENTS)
        self._event_sequence = 0
        self._start_requests = {}
        self._scene_sequence = -1
        self._last_scene = None
        self._missing_count = 0
        self._stop_id = None
        self._stop_started = 0.0
        self._stop_target = State.STOPPED
        self._stop_ack = False
        self._stop_confirmed = False
        self._stop_error = ""
        self._emit("ready", "supervisor ready")

    def _emit(self, event, message, **details):
        self._event_sequence += 1
        self._events.append({
            "sequence": self._event_sequence, "time": self.clock(), "event": event,
            "state": self.state.value, "message": message,
            "batch_id": self.batch.id if self.batch else None,
            "target_id": self.item.target.id if self.item else None, **details,
        })

    def drain_events(self):
        events = list(self._events)
        self._events.clear()
        return events

    @staticmethod
    def _fresh(timestamp, now, max_age, name):
        if not isfinite(timestamp) or not -FUTURE_TOLERANCE <= now - timestamp <= max_age:
            raise ValueError(f"{name} is stale or has an invalid timestamp")

    def _telemetry(self, now, allow_fault=False):
        sample = self.devices.telemetry(now)
        self._fresh(sample.observed_at, now, TELEMETRY_MAX_AGE, "telemetry")
        if not sample.connected:
            raise ValueError("device disconnected")
        if sample.fault and not allow_fault:
            raise ValueError(sample.fault)
        if not isinstance(sample.grip, Grip):
            raise ValueError("invalid grip telemetry")
        return sample

    def _scene(self, now):
        scene = self.devices.observe(now)
        self._fresh(scene.observed_at, now, SCENE_MAX_AGE, "camera observation")
        validate_targets(scene.targets)
        if not isinstance(scene.sequence, int) or scene.sequence < 0:
            raise ValueError("invalid camera observation sequence")
        if self._last_scene is not None:
            if scene.sequence < self._last_scene.sequence:
                raise ValueError("camera observation sequence went backwards")
            if scene.sequence == self._last_scene.sequence and scene != self._last_scene:
                raise ValueError("cached camera observation changed without a new frame")
        self._last_scene = scene
        return scene

    @staticmethod
    def _idle(sample):
        if sample.grip != Grip.NO:
            raise ValueError("gripper must be confirmed empty before START/reset")
        if not sample.robot_stopped or not sample.stations_stopped:
            raise ValueError("robot and station drives must be stopped")

    def start(self, request_id):
        """Idempotent START. Refusal never changes the running batch."""
        if not isinstance(request_id, str) or not request_id.strip():
            return Reply(False, "START requires a non-empty request ID")
        if request_id in self._start_requests:
            return self._start_requests[request_id]
        # Retain IDs for this entire process lifetime: evicting one could execute
        # a late duplicate after a reset. A full cache refuses new START requests.
        if len(self._start_requests) >= MAX_START_REQUESTS:
            return Reply(False, "START request cache full; restart after finishing and checking the system")
        if self.state not in (State.READY, State.COMPLETED):
            reply = Reply(False, f"START refused in {self.state.value}")
        else:
            try:
                now = self.clock()
                self._idle(self._telemetry(now))
                # Optional adapter hook: conditions that block START but not RESET.
                problems = getattr(self.devices, "start_problems", lambda: [])()
                if problems:
                    raise ValueError("not ready: " + "; ".join(problems))
                scene = self._scene(now)
                if not scene.targets:
                    raise ValueError("no glasses detected")
                if any(s.state in (SlotState.UNKNOWN, SlotState.RESERVED)
                       for s in self.outputs.slots.values()):
                    raise ValueError("output occupancy requires operator confirmation")
                self.batch = Batch(uuid4().hex, scene.targets)
                self.item = self.slot = self.command = None
                self._scene_sequence = scene.sequence
                self._missing_count = 0
                self._holding = False
                self.fault = ""
                self.state = State.RUNNING
                self._emit("batch_started", f"batch contains {len(scene.targets)} glasses")
                reply = Reply(True, "batch started", self.batch.id)
            except Exception as exc:
                reply = Reply(False, f"START refused: {exc}")
        self._start_requests[request_id] = reply
        return reply

    def _select_target(self, scene):
        item = self.batch.pending()
        if item is None:
            self.state = State.COMPLETED
            self._emit("batch_completed", "batch accounted for", counts=self.batch.summary())
            return
        if not any(s.state == SlotState.FREE for s in self.outputs.slots.values()):
            self.state = State.WAITING_OUTPUT
            self._emit("waiting_output", "clear and confirm output before the next pick")
            return
        if scene.sequence <= self._scene_sequence:
            return  # wait for a NEW observation, not another read of the same frame
        self._scene_sequence = scene.sequence
        observed = next((target for target in scene.targets if target.id == item.target.id), None)
        if observed is None:
            self._missing_count += 1
            if self._missing_count >= MISSING_TARGET_RETRIES:
                item.state = TargetState.SKIPPED
                item.detail = "target missing in fresh observations"
                self._missing_count = 0
                self._emit("target_skipped", item.detail, target_id=item.target.id)
            return
        self._missing_count = 0
        item.target = observed
        slot = self.outputs.reserve(item.target.id)
        self.item, self.slot = item, slot
        item.state = TargetState.ACTIVE
        self.step_index = 0
        self._emit("output_reserved", f"reserved output {slot.id}", slot_id=slot.id)

    def _monitor_grip(self, sample):
        if not self._holding:
            return
        releasing = self.command is not None and self.command.step == Step.RELEASE
        if releasing and sample.grip == Grip.NO and sample.supported:
            return
        if sample.grip != Grip.OK:
            raise ValueError(f"grip is {sample.grip.value} while holding a glass")

    def tick(self):
        """Advance at most one operation; transport errors latch a fault."""
        now = self.clock()
        if self.state == State.STOPPING:
            self._tick_stop(now)
            return
        if self.state not in (State.RUNNING, State.WAITING_OUTPUT):
            return
        try:
            sample = self._telemetry(now)
            scene = self._scene(now)
            self._monitor_grip(sample)
            if self.state == State.WAITING_OUTPUT:
                self._idle(sample)
                return
            if self.item is None:
                self._idle(sample)
                self._select_target(scene)
                if self.item is None:
                    return
            spec = RECIPE[self.step_index]
            if self.command is None:
                if spec.step == Step.PICK:
                    self._idle(sample)
                if spec.step == Step.RELEASE and not sample.supported:
                    raise ValueError("release refused: glass support not confirmed")
                self.command = Command(uuid4().hex, self.batch.id, self.item.target,
                                       self.slot.id, spec.step, now)
                self.devices.validate_motion(self.command)
                self.devices.begin(self.command)
                self._emit("operation_started", spec.step.value, command_id=self.command.id,
                           step=spec.step.value)
                return
            if now - self.command.issued_at >= spec.timeout:
                raise TimeoutError(f"{spec.step.value} timed out")
            result = self.devices.poll(self.command.id, now)
            if result is None or result.command_id != self.command.id:
                return  # a late reply cannot complete a different operation
            if result.outcome != Outcome.SUCCEEDED:
                raise ValueError(f"{spec.step.value}: {result.outcome.value}: {result.detail}")
            # A poll may publish a NEW device state, e.g. after pick/release.
            sample = self._telemetry(now)
            if spec.step == Step.PICK:
                if sample.grip != Grip.OK:
                    raise ValueError(f"pick not confirmed: {sample.grip.value}")
                self._holding = True
            self._monitor_grip(sample)
            if spec.step == Step.LOWER and not sample.supported:
                raise ValueError("lowering finished without confirmed support")
            if spec.step == Step.RELEASE:
                if sample.grip != Grip.NO or not sample.supported:
                    raise ValueError("release was not confirmed on a supported glass")
                self._holding = False
                self.slot.state = SlotState.OCCUPIED
            self._emit("operation_completed", spec.step.value, command_id=self.command.id,
                       step=spec.step.value)
            self.command = None
            self.step_index += 1
            if self.step_index == len(RECIPE):
                self.item.state = TargetState.COMPLETED
                self._emit("glass_completed", "glass placed mouth-down", slot_id=self.slot.id)
                self.item = self.slot = None
        except Exception as exc:
            self._halt(str(exc), State.FAULT)

    def _halt(self, reason, terminal):
        self.fault = reason
        if self.command is not None:
            self._emit("operation_cancelled", reason, command_id=self.command.id,
                       step=self.command.step.value)
        self.command = None
        if self.item is not None:
            self.item.state = TargetState.INTERRUPTED
            self.item.detail = reason
        if self.slot is not None and self.slot.state == SlotState.RESERVED:
            self.slot.state = SlotState.UNKNOWN
        self.state = State.STOPPING
        self._stop_target = terminal
        self._stop_started = self.clock()
        self._stop_id = uuid4().hex
        self._stop_ack = self._stop_confirmed = False
        self._stop_error = ""
        self._emit("stop_requested", reason, stop_id=self._stop_id)
        try:
            self.devices.begin_stop(self._stop_id, self._stop_started)
        except Exception as exc:
            self.fault = f"{reason}; stop dispatch failed: {exc}"
            self.state = State.FAULT
            self._emit("fault", self.fault)

    def request_stop(self, reason="operator STOP"):
        if self.state == State.STOPPING:
            return Reply(True, "stop already requested", self.batch.id if self.batch else None)
        # Retrying a failed stop cannot downgrade an existing fault to STOPPED.
        terminal = State.FAULT if self.state == State.FAULT else State.STOPPED
        if terminal == State.FAULT and self.fault:
            reason = f"{self.fault}; {reason}"
        self._halt(reason, terminal)
        return Reply(True, "stop requested", self.batch.id if self.batch else None)

    def _tick_stop(self, now):
        if now - self._stop_started >= STOP_TIMEOUT:
            self.state = State.FAULT
            self.fault += "; stopping was not confirmed before timeout"
            if self._stop_error:
                self.fault += f": {self._stop_error}"
            self._emit("fault", self.fault)
            return
        try:
            result = self.devices.poll_stop(self._stop_id, now)
            if result is not None and result.command_id == self._stop_id:
                if result.outcome != Outcome.SUCCEEDED:
                    raise ValueError(result.detail or "stop failed")
                self._stop_ack = True
            sample = self._telemetry(now, allow_fault=True)
            if self._stop_ack and sample.robot_stopped and sample.stations_stopped:
                self._stop_confirmed = True
                self.state = self._stop_target
                self._emit("stopped", self.fault, stop_id=self._stop_id)
        except Exception as exc:
            # A disconnected device may recover within the deadline. Until then
            # STOPPING remains visible; no operation or RELEASE can be issued.
            self._stop_error = str(exc)

    def confirm_output_cleared(self, slot_ids):
        if self.state in (State.RUNNING, State.STOPPING):
            return Reply(False, "output confirmation refused during motion/automatic cycle")
        try:
            self.outputs.confirm_cleared(slot_ids)
        except ValueError as exc:
            return Reply(False, str(exc))
        self._emit("output_cleared", "operator confirmed output clear")
        if self.state == State.WAITING_OUTPUT:
            self.state = State.RUNNING
        return Reply(True, "output cleared")

    def reset_fault(self):
        if self.state not in (State.STOPPED, State.FAULT):
            return Reply(False, "reset requires STOPPED or FAULT")
        try:
            if not self._stop_confirmed:
                raise ValueError("stopping has not been confirmed; retry STOP after checking devices")
            self._idle(self._telemetry(self.clock()))
            if any(s.state in (SlotState.UNKNOWN, SlotState.RESERVED)
                   for s in self.outputs.slots.values()):
                raise ValueError("resolve unknown output occupancy first")
        except Exception as exc:
            return Reply(False, f"reset refused: {exc}")
        self.item = self.slot = self.command = None
        self._holding = False
        # A repaired/restarted vision adapter may start a new frame sequence.
        # This is only accepted after a checked reset, never during a batch.
        self._last_scene = None
        self._scene_sequence = -1
        self.fault = ""
        self.state = State.READY
        self._emit("reset", "ready; a new START is required")
        return Reply(True, "ready; a new START is required")

    def status(self):
        return {
            "state": self.state.value, "fault": self.fault,
            "batch_id": self.batch.id if self.batch else None,
            "counts": self.batch.summary() if self.batch else {},
            "target_id": self.item.target.id if self.item else None,
            "operation": self.command.step.value if self.command else None,
            "step_index": self.step_index if self.item else None,
            "stop_confirmed": self._stop_confirmed,
            "outputs": self.outputs.snapshot(),
            "targets": [{"id": item.target.id, "state": item.state.value, "detail": item.detail}
                        for item in self.batch.items] if self.batch else [],
        }
