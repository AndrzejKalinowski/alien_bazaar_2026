"""Batch snapshot and exclusive output reservations, with no hardware imports.

Output slots start UNKNOWN unless the caller explicitly supplies verified free
slots (the simulator does). A reservation interrupted by STOP becomes UNKNOWN.
Only explicit operator confirmation can make an occupied/unknown slot free.
New detections after START are not appended to the current batch.

OutputStore keeps the occupancy of real output places across restarts
(system_outputs.json, atomic writes). A restart never frees a place:
RESERVED comes back as UNKNOWN, and after an unclean exit (crash, power
loss, a batch still active) FREE also comes back as UNKNOWN, so the
operator checks the output again. A missing or unreadable file is all UNKNOWN.
Requires: Python standard library.
"""

from dataclasses import dataclass
import json
from math import isfinite
import os

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


OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "system_outputs.json")


class OutputStore:
    """Output occupancy on disk. Owner thread only."""

    def __init__(self, path=None):
        self.path = path or OUTPUT_FILE
        self.warning = ""
        self._last = None

    def load_slots(self, slot_ids):
        data = {}
        if os.path.exists(self.path):
            try:
                with open(self.path, encoding="utf-8") as f:
                    data = json.load(f)
                if not isinstance(data, dict):
                    raise ValueError("not a JSON object")
            except (OSError, ValueError) as exc:
                self.warning = f"{self.path} unreadable ({exc}); all output places UNKNOWN"
                data = {}
        clean = data.get("clean_shutdown") is True
        saved = data.get("slots") if isinstance(data.get("slots"), dict) else {}
        slots = []
        for slot_id in slot_ids:
            entry = saved.get(slot_id) if isinstance(saved.get(slot_id), dict) else {}
            try:
                state = SlotState(entry.get("state"))
            except ValueError:
                state = SlotState.UNKNOWN
            if state == SlotState.RESERVED or (state == SlotState.FREE and not clean):
                state = SlotState.UNKNOWN
            target = entry.get("target_id") if state == SlotState.OCCUPIED else None
            slots.append(OutputSlot(slot_id, state, target if isinstance(target, str) else None))
        # From now on the session counts as unclean until close() says otherwise.
        self.save({s.id: {"state": s.state.value, "target_id": s.target_id} for s in slots})
        return slots

    def save(self, snapshot, clean=False):
        record = {"clean_shutdown": clean, "slots": snapshot}
        if record == self._last:
            return
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(record, f, indent=2)
        os.replace(tmp, self.path)
        self._last = record

    def sink(self, supervisor):
        """ControlLoop sink: save after any tick that produced events."""
        def write(events):
            if events:
                self.save(supervisor.outputs.snapshot())
        return write

    def close(self, supervisor, stopped):
        """stopped: the devices were confirmed stopped and no batch is active."""
        self.save(supervisor.outputs.snapshot(), clean=stopped)
