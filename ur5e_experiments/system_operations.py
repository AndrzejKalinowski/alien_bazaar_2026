"""Non-blocking operation contract and recipe for one upright glass.

Each tick only starts or polls an operation; adapters must never wait for a
move, USB reply, thread join, or image here. A successful result means the
operation's physical postconditions were verified by the adapter. In this
milestone only the simulator implements this contract. Real motion will use
SafeControl and the controller-owned watchdog, not direct RTDE calls.

The two spray steps share an operation type and differ by recipe phase.
Each station operation is followed by an explicit LEAVE step. Its checked
withdrawal must finish before the next station's approach can start.
Requires: Python standard library.
"""

from dataclasses import dataclass
from typing import Protocol

from system_model import Command, Result, Scene, Step, Telemetry
from system_settings import MOTION_TIMEOUT, PICK_TIMEOUT, RELEASE_TIMEOUT, STATION_TIMEOUT


@dataclass(frozen=True)
class OperationSpec:
    step: Step
    timeout: float


RECIPE = tuple(
    OperationSpec(step, PICK_TIMEOUT if step == Step.PICK else
                  RELEASE_TIMEOUT if step == Step.RELEASE else
                  STATION_TIMEOUT if step in (Step.SPRAY_1, Step.SPONGE, Step.SPRAY_2, Step.WIPE)
                  else MOTION_TIMEOUT)
    for step in Step
)


class DeviceAdapter(Protocol):
    """Single-owner interface; every method must return promptly.

    begin_stop must invalidate pending device commands BEFORE requesting a
    stop. Its acknowledgement is separate from dispatch. No stop method may
    release vacuum. A future hardware adapter must confirm stopping both
    servos as well as the robot, and propagate device errors as failures.
    """

    def telemetry(self, now: float) -> Telemetry: ...

    def observe(self, now: float) -> Scene: ...

    def validate_motion(self, command: Command) -> None:
        """Check and retain the exact planned path before begin.

        Include station heights, tool/glass swept volume, approach/withdrawal
        and always-solid fixtures. A hardware adapter also needs full-arm
        checks; replanning or changing geometry invalidates the approval.
        """
        ...

    def begin(self, command: Command) -> None: ...

    def poll(self, command_id: str, now: float) -> Result | None: ...

    def begin_stop(self, stop_id: str, now: float) -> None: ...

    def poll_stop(self, stop_id: str, now: float) -> Result | None: ...
