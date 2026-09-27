"""Batch snapshot and exclusive output reservations, with no hardware imports.

Output slots start UNKNOWN unless the caller explicitly supplies verified free
slots (the simulator does). A reservation interrupted by STOP becomes UNKNOWN.
Only explicit operator confirmation can make an occupied/unknown slot free.
New detections after START are not appended to the current batch.
Requires: Python standard library.
"""

from dataclasses import dataclass
from math import isfinite

from system_model import GlassTarget, SlotState, TargetState


@dataclass
class OutputSlot:
    id: str
    state: SlotState = SlotState.UNKNOWN
    target_id: str | None = None


@dataclass
class BatchItem:
    target: GlassTarget
    state: TargetState = TargetState.PENDING
    detail: str = ""


def validate_targets(targets):
    ids = set()
    for target in targets:
        if not target.id or target.id in ids:
            raise ValueError("target IDs must be non-empty and unique")
        if not isfinite(target.x) or not isfinite(target.y):
            raise ValueError("target coordinates must be finite")
        ids.add(target.id)


class Batch:
    def __init__(self, batch_id, targets):
        validate_targets(targets)
        self.id = batch_id
        self.items = [BatchItem(target) for target in targets]

    def pending(self):
        return next((item for item in self.items if item.state == TargetState.PENDING), None)

    def summary(self):
        return {state.value: sum(item.state == state for item in self.items)
                for state in TargetState}


class OutputRegistry:
    def __init__(self, slots):
        slots = list(slots)
        if not slots or any(not s.id for s in slots) or len({s.id for s in slots}) != len(slots):
            raise ValueError("output slot IDs must be non-empty and unique; at least one required")
        # Do not share mutable reservations with the caller's configuration.
        self.slots = {s.id: OutputSlot(s.id, s.state, s.target_id) for s in slots}

    def reserve(self, target_id):
        slot = next((s for s in self.slots.values() if s.state == SlotState.FREE), None)
        if slot is not None:
            slot.state = SlotState.RESERVED
            slot.target_id = target_id
        return slot

    def confirm_cleared(self, slot_ids):
        ids = list(slot_ids)
        if not ids or any(i not in self.slots for i in ids):
            raise ValueError("provide known output slot IDs")
        if any(self.slots[i].state == SlotState.RESERVED for i in ids):
            raise ValueError("cannot clear a reserved output slot")
        for slot_id in ids:
            slot = self.slots[slot_id]
            slot.state, slot.target_id = SlotState.FREE, None

    def snapshot(self):
        return {key: {"state": s.state.value, "target_id": s.target_id}
                for key, s in self.slots.items()}
