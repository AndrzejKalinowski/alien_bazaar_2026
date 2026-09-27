"""Serial device owners for the hardware supervisor: gripper and servo bus.

Each worker is ONE thread that alone uses its port. The supervisor's owner
thread never waits on USB: it submits jobs (bounded queue), reads results by
job ID and reads a timestamped snapshot. Blocking driver calls (Suction.status()
up to 1 s, servo transactions, stroke waits) happen only in the worker thread.

Ports are opened once, before the batch loop is armed, and never reopened
automatically: opening the XIAO port can reset the controller and drop the
vacuum on a held glass. After a serial error the worker reports disconnected
with a latched fault; the operator decides what to do.

GripperWorker
  snapshot grip: IDLE or RELEASING -> NO (vacuum off); GRIPPING -> OK only
  with a "GRIP OK" report of this session AND a live "HOLD YES"; LOST on
  "GRIP LOST" or "HOLD NO" after OK; otherwise UNKNOWN (vacuum on, not
  confirmed, or no sensor). observed_at is the time of the last STATE reply.
  Jobs: "grip" (sent once the release pulse is over), "release" (SUCCEEDED
  only when the controller is back to IDLE: the pulse really ended; timeout
  is FAILED with the state unknown). Stopping never releases the vacuum.

ServoWorker (Feetech STS bus, bus_servos.py)
  Roles: SPRAYER works the pump, SPONGE drives the sponge. Jobs: "spray"
  (count strokes rest -> press -> rest, each position confirmed) and "sponge"
  (a list of absolute angles, repeated). A servo that does not reach its goal
  in time, reports a status error or stops answering fails the job.
  request_stop(): priority flag; the running job aborts at its next check,
  then the sprayer goes to rest, the sponge holds, and the stop result is
  SUCCEEDED only after both report not moving. The servos themselves have no
  timeout: a PC or USB crash leaves them at the last goal (position mode
  only, so they do not keep turning; continuous sponge rotation needs a
  device-side limit first, see docs/system_supervisor_plan.md §6).

Requires: pyserial (suction.py, bus_servos.py), imported by the open_* helpers.
"""

from dataclasses import dataclass
import queue
import threading
from time import monotonic

from system_model import Grip, Outcome, Result

# --- workers ---------------------------------------------------------------------
JOB_QUEUE_SIZE = 4             # jobs waiting per device
RESULTS_KEPT = 64              # finished job results kept until read
GRIPPER_POLL = 0.1             # s between STATE/HOLD queries
SERVO_POLL = 0.1               # s between servo status reads while idle
CHECK_PERIOD = 0.02            # s, job progress checks (and STOP reaction)

# --- gripper ---------------------------------------------------------------------
GRIP_SEND_TIMEOUT = 2.0        # s, wait for a running release pulse (1.5 s) before GRIP
RELEASE_CONFIRM_TIMEOUT = 2.5  # s, pulse is 1.5 s (firmware RELEASE_PULSE_MS)

# --- servos ----------------------------------------------------------------------
SERVO_TOLERANCE_DEG = 3.0      # deg, goal reached
SERVO_MOVE_TIMEOUT = 2.0       # s per position move
SERVO_STOP_TIMEOUT = 1.0       # s, both servos not moving after a stop


@dataclass(frozen=True)
class SprayProfile:
    rest_deg: float          # deg, pump released
    press_deg: float         # deg, pump pressed
    strokes: int
    hold: float              # s pressed
    period: float            # s between stroke starts
    speed: int               # steps/s


@dataclass(frozen=True)
class SpongeProfile:
    positions_deg: tuple     # deg, absolute goals in order
    repeats: int
    speed: int               # steps/s


class JobCancelled(Exception):
    pass


class DeviceWorker:
    """One thread per device. Subclasses implement _open, _poll, _run_job, _stop_devices."""

    name = "device"

    def __init__(self, opener, clock=monotonic):
        self._opener = opener
        self.clock = clock
        self._device = None
        self._jobs = queue.Queue(maxsize=JOB_QUEUE_SIZE)
        self._lock = threading.Lock()
        self._results = {}
        self._cancel = threading.Event()
        self._closed = threading.Event()
        self._stop_id = None
        self._thread = None
        self.connected = False
        self.fault = ""
        self.observed_at = float("-inf")

    # --- owner thread -------------------------------------------------------------

    def start(self):
        """Open the port in the caller's thread (may take seconds), then run."""
        self._device = self._opener()
        try:
            self._after_open()
            self._poll()
        except Exception:
            self._device.close()
            raise
        self.connected = True
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def _after_open(self):
        pass

    def submit(self, job_id, kind, **params):
        if self._closed.is_set() or not self.connected:
            raise RuntimeError(f"{self.name} is not available: {self.fault or 'closed'}")
        try:
            self._jobs.put_nowait((job_id, kind, params))
        except queue.Full:
            raise RuntimeError(f"{self.name} job queue full") from None

    def result(self, job_id):
        with self._lock:
            return self._results.pop(job_id, None)

    def request_stop(self, stop_id):
        """Invalidate queued and running jobs, then stop the devices."""
        with self._lock:
            self._stop_id = stop_id
        self._cancel.set()
        self._drain("cancelled by STOP")

    def close(self):
        self._closed.set()
        self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass

    # --- worker thread ------------------------------------------------------------

    def _finish(self, job_id, outcome, detail=""):
        with self._lock:
            self._results[job_id] = Result(job_id, outcome, detail)
            while len(self._results) > RESULTS_KEPT:
                self._results.pop(next(iter(self._results)))

    def _drain(self, message):
        while True:
            try:
                job_id, _, _ = self._jobs.get_nowait()
            except queue.Empty:
                return
            self._finish(job_id, Outcome.CANCELLED, message)

    def _check(self):
        """Called often inside jobs; raises when a STOP arrived."""
        if self._cancel.is_set():
            raise JobCancelled()

    def _sleep(self, seconds):
        if self._cancel.wait(seconds):
            raise JobCancelled()

    def _run(self):
        next_poll = 0.0
        while not self._closed.is_set():
            try:
                if self._cancel.is_set():
                    self._handle_stop()
                    continue
                try:
                    job_id, kind, params = self._jobs.get(timeout=CHECK_PERIOD)
                except queue.Empty:
                    if self.clock() >= next_poll:
                        self._poll()
                        next_poll = self.clock() + self.poll_period
                    continue
                try:
                    detail = self._run_job(kind, params)
                    self._finish(job_id, Outcome.SUCCEEDED, detail or "")
                except JobCancelled:
                    self._finish(job_id, Outcome.CANCELLED, "cancelled by STOP")
                except (TimeoutError, RuntimeError, ValueError) as exc:
                    self._finish(job_id, Outcome.FAILED, str(exc))
            except Exception as exc:  # serial errors: latch, never reopen here
                self.connected = False
                self.fault = f"{self.name}: {exc}"
                self._drain(self.fault)
                with self._lock:
                    stop_id, self._stop_id = self._stop_id, None
                if stop_id is not None:
                    self._finish(stop_id, Outcome.FAILED, self.fault)
                self._closed.wait()

    def _handle_stop(self):
        with self._lock:
            stop_id = self._stop_id
        self._cancel.clear()
        self._drain("cancelled by STOP")
        if stop_id is None:
            return
        try:
            self._stop_devices()
            outcome, detail = Outcome.SUCCEEDED, ""
        except (TimeoutError, RuntimeError, ValueError) as exc:
            outcome, detail = Outcome.FAILED, str(exc)
        with self._lock:
            if self._stop_id == stop_id:
                self._stop_id = None
        self._finish(stop_id, outcome, detail)
        self._poll()

    poll_period = 0.1

    def _poll(self):
        raise NotImplementedError

    def _run_job(self, kind, params):
        raise NotImplementedError

    def _stop_devices(self):
        pass


def grip_from_reports(state, result, holding):
    """Map controller STATE, last GRIP report and HOLD answer to Grip."""
    if state in ("IDLE", "RELEASING"):
        return Grip.NO
    if state != "GRIPPING":
        return Grip.UNKNOWN
    if result == "LOST" or (result == "OK" and holding is False):
        return Grip.LOST
    if result == "OK" and holding is True:
        return Grip.OK
    return Grip.UNKNOWN


class GripperWorker(DeviceWorker):
    """Owns suction.Suction. snapshot() is safe from any thread."""

    name = "gripper"
    poll_period = GRIPPER_POLL

    def __init__(self, opener, clock=monotonic):
        super().__init__(opener, clock)
        self.state = None
        self.report = None
        self.holding = None

    def snapshot(self):
        with self._lock:
            return {"connected": self.connected, "fault": self.fault, "observed_at": self.observed_at,
                    "grip": grip_from_reports(self.state, self.report, self.holding),
                    "state": self.state, "report": self.report, "holding": self.holding}

    def _poll(self):
        suction = self._device
        state = suction.status()
        report = suction.grip_result()
        holding = suction.is_holding() if state == "GRIPPING" else None
        with self._lock:
            if state is not None:
                self.state, self.report, self.holding = state, report, holding
                self.observed_at = self.clock()

    def _run_job(self, kind, params):
        suction = self._device
        if kind == "grip":
            deadline = self.clock() + GRIP_SEND_TIMEOUT
            while not suction.grip():
                if self.clock() > deadline:
                    raise TimeoutError("release pulse did not end, GRIP not sent")
                self._sleep(CHECK_PERIOD)
            self._poll()
            return "vacuum on"
        if kind == "release":
            if not suction.release():
                raise RuntimeError("a release pulse is already running")
            deadline = self.clock() + RELEASE_CONFIRM_TIMEOUT
            while True:
                # STATE IDLE is the controller's own word that the pulse ended;
                # Suction.release_pending() also turns False on its own timeout.
                self._poll()
                if self.state == "IDLE":
                    return "release pulse finished"
                if self.clock() > deadline:
                    raise TimeoutError(f"release not confirmed (state {self.state}), gripper state unknown")
                self._sleep(CHECK_PERIOD)
        raise ValueError(f"unknown gripper job {kind!r}")


class ServoWorker(DeviceWorker):
    """Owns bus_servos.ServoBus with the sprayer and sponge servos."""

    name = "servos"
    poll_period = SERVO_POLL

    def __init__(self, opener, sprayer_id, sponge_id, spray_rest_deg, clock=monotonic):
        super().__init__(opener, clock)
        if sprayer_id == sponge_id:
            raise ValueError("sprayer and sponge need different servo IDs")
        self.ids = {"SPRAYER": sprayer_id, "SPONGE": sponge_id}
        self.spray_rest_deg = spray_rest_deg
        self.servos = {}
        self.moving = {"SPRAYER": None, "SPONGE": None}
        self.errors = {"SPRAYER": [], "SPONGE": []}

    def _after_open(self):
        from bus_servos import BusServo  # pyserial only in hardware mode
        self.servos = {role: BusServo(self._device, servo_id) for role, servo_id in self.ids.items()}
        missing = [f"{role} (ID {self.ids[role]})" for role, servo in self.servos.items() if not servo.ping()]
        if missing:
            raise RuntimeError(f"servos not answering: {', '.join(missing)}")

    @property
    def stopped(self):
        with self._lock:
            return all(m is False for m in self.moving.values())

    def snapshot(self):
        with self._lock:
            return {"connected": self.connected, "fault": self.fault, "observed_at": self.observed_at,
                    "moving": dict(self.moving), "errors": {k: list(v) for k, v in self.errors.items()},
                    "stopped": all(m is False for m in self.moving.values())}

    def _poll(self):
        readings = {role: servo.status() for role, servo in self.servos.items()}
        with self._lock:
            for role, status in readings.items():
                self.moving[role] = status["moving"]
                self.errors[role] = status["errors"]
            self.observed_at = self.clock()

    def _goto(self, role, degrees, speed):
        servo = self.servos[role]
        servo.move_to(degrees, speed)
        deadline = self.clock() + SERVO_MOVE_TIMEOUT
        self._sleep(CHECK_PERIOD)
        while True:
            status = servo.status()
            with self._lock:
                self.moving[role], self.errors[role] = status["moving"], status["errors"]
                self.observed_at = self.clock()
            if status["errors"]:
                raise RuntimeError(f"{role} servo error: {', '.join(status['errors'])}")
            angle = status["position"] * 360 / 4096
            if not status["moving"] and abs(angle - degrees) <= SERVO_TOLERANCE_DEG:
                return
            if self.clock() > deadline:
                raise TimeoutError(f"{role} servo did not reach {degrees:.0f} deg (at {angle:.0f})")
            self._sleep(CHECK_PERIOD)

    def _run_job(self, kind, params):
        if kind == "spray":
            profile = params["profile"]
            for stroke in range(profile.strokes):
                started = self.clock()
                self._goto("SPRAYER", profile.press_deg, profile.speed)
                self._sleep(profile.hold)
                self._goto("SPRAYER", profile.rest_deg, profile.speed)
                if stroke + 1 < profile.strokes:
                    self._sleep(max(0.0, profile.period - (self.clock() - started)))
            return f"{profile.strokes} strokes"
        if kind == "sponge":
            profile = params["profile"]
            for _ in range(profile.repeats):
                for degrees in profile.positions_deg:
                    self._goto("SPONGE", degrees, profile.speed)
            return f"{profile.repeats} sponge cycles"
        raise ValueError(f"unknown servo job {kind!r}")

    def _stop_devices(self):
        # Pump released and sponge held where it is; neither keeps moving.
        self.servos["SPONGE"].stop()
        self.servos["SPRAYER"].move_to(self.spray_rest_deg)
        deadline = self.clock() + SERVO_STOP_TIMEOUT
        while True:
            self._poll()
            if self.stopped:
                return
            if self.clock() > deadline:
                raise TimeoutError("servos still moving after stop")
            self._closed.wait(CHECK_PERIOD)


def open_suction():
    from suction import Suction
    return Suction()


def open_servo_bus():
    from bus_servos import ServoBus
    return ServoBus()
