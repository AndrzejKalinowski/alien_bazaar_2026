"""Taught robot poses and measured station layout for the hardware supervisor.

One JSON file (system_teach.json next to this script) holds what only the
real cell can provide:
  poses     TCP poses captured from RTDE while the operator holds the arm
            there (pendant jog or freedrive), with the joint angles. Captured
            with POST /api/teach/capture from the panel (--teach), never
            during a batch. Station poses are taught WITH a gripped glass in
            its working orientation (mouth down), so they already include
            the glass-in-cup geometry; the grip must be repeatable for that.
  glass     measured glass profile: radius of the wall at the cup height.
  layout    measured station boxes (m, robot base frame, absolute z; a height
            measured from the table needs table_z added), work corridors,
            solid parts, and the tool+glass sphere radius and clearance.
            Edited by hand after measuring; read at start-up only.
  tcp_offset  getTCPOffset() when the poses were taught. A different TCP on
            the pendant makes every pose wrong, so hardware START is refused.

Required pose names (see required_poses()):
  observe          arm out of the overhead camera's view, before each pick
  side_grip        cup on the wall of an upright glass, tool horizontal; only
                   its orientation and height are used (the x, y come from the
                   camera). Tool z (approach) must be horizontal.
  flip             where the held glass is turned mouth-down by wrist 3
  <station>.approach, <station>.work   for sprayer, sponge and wiper
  wiper.stroke.1 .. wiper.stroke.N      wiping moves over the cloth, in order
  output.<slot>    TCP when the mouth-down glass stands on that output place

The file is data the operator edits at runtime, so it is JSON (repo
convention); program constants stay in Python. Writes are atomic (tmp +
os.replace). Nothing here talks to hardware. Requires: Python stdlib.
"""

from datetime import datetime, timezone
import json
from math import isfinite
import os
import re

from system_geometry import Box, Station, Workspace

TEACH_FILE = os.path.join(os.path.dirname(__file__), "system_teach.json")
TEACH_VERSION = 1
STATIONS = ("sprayer", "sponge", "wiper")
MAX_WIPE_STROKES = 20
TCP_OFFSET_TOLERANCE = 0.001      # m and rad, taught vs current TCP offset
NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")


class TeachError(ValueError):
    """The taught data is missing, malformed or does not match the robot."""


def _vector(value, length, what):
    try:
        result = [float(v) for v in value]
    except (TypeError, ValueError):
        raise TeachError(f"{what} must be a list of {length} numbers") from None
    if len(result) != length or not all(isfinite(v) for v in result):
        raise TeachError(f"{what} must be a list of {length} finite numbers")
    return result


def required_poses(slot_ids, wipe_strokes):
    names = ["observe", "side_grip", "flip"]
    for station in STATIONS:
        names += [f"{station}.approach", f"{station}.work"]
    names += [f"wiper.stroke.{i}" for i in range(1, wipe_strokes + 1)]
    names += [f"output.{slot}" for slot in slot_ids]
    return names


def empty_data():
    return {"version": TEACH_VERSION, "tcp_offset": None, "poses": {},
            "glass": {"radius": None},
            "layout": {"tool_radius": None, "clearance": None, "stations": {}}}


class TeachStore:
    """Load, check, capture and save the taught data. Owner thread only."""

    def __init__(self, slot_ids, path=None):
        self.slot_ids = tuple(slot_ids)
        self.path = path or TEACH_FILE
        self.data = empty_data()
        if os.path.exists(self.path):
            with open(self.path, encoding="utf-8") as f:
                try:
                    data = json.load(f)
                except ValueError as exc:
                    raise TeachError(f"{self.path} is not valid JSON: {exc}") from None
            if not isinstance(data, dict) or data.get("version") != TEACH_VERSION:
                raise TeachError(f"{self.path}: expected version {TEACH_VERSION}")
            self.data = {**empty_data(), **data}

    # --- poses --------------------------------------------------------------------

    @property
    def wipe_strokes(self):
        count = 0
        while f"wiper.stroke.{count + 1}" in self.data["poses"]:
            count += 1
        return count

    def allowed(self, name):
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            return False
        stroke = re.fullmatch(r"wiper\.stroke\.(\d+)", name)
        if stroke:
            number = int(stroke.group(1))
            # strokes are taught in order; the next one may be added
            return 1 <= number <= min(self.wipe_strokes + 1, MAX_WIPE_STROKES)
        return name in required_poses(self.slot_ids, 0)

    def pose(self, name):
        entry = self.data["poses"].get(name)
        if entry is None:
            raise TeachError(f"pose {name!r} is not taught")
        return _vector(entry["pose"], 6, name)

    def joints(self, name):
        entry = self.data["poses"].get(name)
        if entry is None:
            raise TeachError(f"pose {name!r} is not taught")
        return _vector(entry["q"], 6, f"{name} joints")

    def capture(self, name, pose, q, tcp_offset):
        """Store the current robot pose under name and save the file."""
        if not self.allowed(name):
            raise TeachError(f"{name!r} is not a pose name that can be taught now")
        pose, q = _vector(pose, 6, "TCP pose"), _vector(q, 6, "joint angles")
        tcp_offset = _vector(tcp_offset, 6, "TCP offset")
        taught = self.data.get("tcp_offset")
        if taught is not None and max(abs(a - b) for a, b in zip(taught, tcp_offset)) > TCP_OFFSET_TOLERANCE:
            raise TeachError("the TCP offset changed since the other poses were taught; "
                             "restore it on the pendant or start a new teach file")
        self.data["tcp_offset"] = tcp_offset
        self.data["poses"][name] = {"pose": pose, "q": q,
                                    "captured": datetime.now(timezone.utc).isoformat(timespec="seconds")}
        self.save()

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2)
        os.replace(tmp, self.path)

    # --- checks -------------------------------------------------------------------

    def missing(self):
        """Human-readable list of what still blocks an automatic cycle."""
        problems = [f"pose {name}" for name in required_poses(self.slot_ids, max(1, self.wipe_strokes))
                    if name not in self.data["poses"]]
        radius = self.data["glass"].get("radius")
        if not isinstance(radius, (int, float)) or not 0 < radius < 0.2:
            problems.append("glass.radius (m, wall radius at the cup height)")
        try:
            self.workspace(-1.0, 1.0)
        except TeachError as exc:
            problems.append(str(exc))
        return problems

    def check_tcp_offset(self, tcp_offset):
        taught = self.data.get("tcp_offset")
        if taught is None:
            return
        current = _vector(tcp_offset, 6, "TCP offset")
        if max(abs(a - b) for a, b in zip(taught, current)) > TCP_OFFSET_TOLERANCE:
            raise TeachError(f"TCP offset on the robot {current} differs from the taught {taught}")

    def workspace(self, min_tcp_z, max_tcp_z):
        """system_geometry.Workspace from the measured layout (limits from the caller)."""
        layout = self.data["layout"]
        try:
            stations = []
            for station_id in STATIONS:
                entry = layout["stations"].get(station_id)
                if entry is None:
                    raise TeachError(f"layout.stations.{station_id} is not measured")
                bounds = Box(entry["low"], entry["high"])
                corridor = Box(**entry["corridor"])
                solids = tuple(Box(**solid) for solid in entry.get("solid", ()))
                stations.append(Station(station_id, bounds, corridor,
                                        self.pose(f"{station_id}.work")[:3]
                                        if f"{station_id}.work" in self.data["poses"]
                                        else corridor_center(corridor), solids))
            return Workspace(tuple(stations), float(layout["tool_radius"]), float(layout["clearance"]),
                             min_tcp_z, max_tcp_z)
        except TeachError:
            raise
        except (KeyError, TypeError, ValueError) as exc:
            raise TeachError(f"layout: {exc}") from None

    def status(self):
        names = required_poses(self.slot_ids, self.wipe_strokes + 1)
        return {"path": os.path.basename(self.path),
                "poses": {name: name in self.data["poses"] for name in names},
                "missing": self.missing()}


def corridor_center(box):
    return tuple((a + b) / 2 for a, b in zip(box.low, box.high))
