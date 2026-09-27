"""Hardware-independent messages for the supervisor and future I/O adapters.

All timestamps use the same monotonic clock. Telemetry.observed_at is the
OLDEST contributing device sample, never the time a cached snapshot was read.
Scene target IDs are stable within a batch; the future vision adapter must
associate fresh detections before publishing them. Positions are metres.
Importing this module opens no connections. Requires: Python standard library.
"""

from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    READY = "READY"
    RUNNING = "RUNNING"
    WAITING_OUTPUT = "WAITING_OUTPUT"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"
    COMPLETED = "COMPLETED"


class Grip(str, Enum):
    NO = "NO"
    OK = "OK"
    LOST = "LOST"
    UNKNOWN = "UNKNOWN"


class SlotState(str, Enum):
    FREE = "FREE"
    RESERVED = "RESERVED"
    OCCUPIED = "OCCUPIED"
    UNKNOWN = "UNKNOWN"


class TargetState(str, Enum):
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    SKIPPED = "SKIPPED"
    INTERRUPTED = "INTERRUPTED"


class Outcome(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class Step(str, Enum):
    PICK = "PICK"
    LIFT = "LIFT"
    FLIP = "FLIP"
    TO_SPRAYER_1 = "TO_SPRAYER_1"
    SPRAY_1 = "SPRAY_1"
    LEAVE_SPRAYER_1 = "LEAVE_SPRAYER_1"
    TO_SPONGE = "TO_SPONGE"
    SPONGE = "SPONGE"
    LEAVE_SPONGE = "LEAVE_SPONGE"
    TO_SPRAYER_2 = "TO_SPRAYER_2"
    SPRAY_2 = "SPRAY_2"
    LEAVE_SPRAYER_2 = "LEAVE_SPRAYER_2"
    TO_WIPER = "TO_WIPER"
    WIPE = "WIPE"
    LEAVE_WIPER = "LEAVE_WIPER"
    TO_OUTPUT = "TO_OUTPUT"
    LOWER = "LOWER"
    RELEASE = "RELEASE"
    RETREAT = "RETREAT"
    OBSERVE = "OBSERVE"


@dataclass(frozen=True)
class GlassTarget:
    id: str
    x: float
    y: float


@dataclass(frozen=True)
class Scene:
    sequence: int
    observed_at: float
    targets: tuple[GlassTarget, ...]


@dataclass(frozen=True)
class Telemetry:
    observed_at: float
    connected: bool
    grip: Grip
    supported: bool
    robot_stopped: bool
    stations_stopped: bool
    fault: str = ""


@dataclass(frozen=True)
class Command:
    id: str
    batch_id: str
    target: GlassTarget
    slot_id: str
    step: Step
    issued_at: float


@dataclass(frozen=True)
class Result:
    command_id: str
    outcome: Outcome
    detail: str = ""


@dataclass(frozen=True)
class Reply:
    accepted: bool
    message: str
    batch_id: str | None = None
