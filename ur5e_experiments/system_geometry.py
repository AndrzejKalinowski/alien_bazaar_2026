"""Conservative station clearance checks for TCP polylines, in base-frame metres.

A station has an occupied bounding box (including its maximum servo sweep),
an optional taught work corridor and optional always-solid parts. The moving
tool/glass is enclosed by a sphere about the TCP that covers EVERY orientation,
including the full 180-degree flip. Inflating boxes by radius + clearance gives
a conservative continuous segment test; checking just waypoint endpoints is
insufficient. Touching an inflated obstacle counts as a collision.

Only the corresponding approach/work/withdraw stage may use a station's work
corridor. Other stations and solid parts remain obstacles in every stage.
The corridor is an explicitly approved swept tool volume, not an exemption
for the whole station. Real dimensions and corridors must be taught/validated.

This checks tool/glass vs modelled stations. It does NOT check UR links,
joint-space moveJ arcs, cables, humans, unknown obstacles or real tracking
error. A future hardware adapter must also use SafeControl and validate the
full robot path before enabling motion. Requires: Python standard library.
"""

from dataclasses import dataclass
from math import isfinite

from system_model import Step

Point = tuple[float, float, float]
GEOMETRY_EPSILON = 1e-9  # m, numerical separation in addition to configured clearance


class CollisionRefused(ValueError):
    """A planned path cannot be dispatched with the current layout."""


def point(value):
    result = tuple(float(v) for v in value)
    if len(result) != 3 or not all(isfinite(v) for v in result):
        raise ValueError("a point must have three finite coordinates in metres")
    return result


def _between(start, end, t):
    return tuple(a + (b - a) * t for a, b in zip(start, end))


@dataclass(frozen=True)
class Box:
    low: Point
    high: Point

    def __post_init__(self):
        object.__setattr__(self, "low", point(self.low))
        object.__setattr__(self, "high", point(self.high))
        if any(a >= b for a, b in zip(self.low, self.high)):
            raise ValueError("box minimum must be below maximum on every axis")

    def contains_envelope(self, center, extent):
        return all(lo <= v - extent and v + extent <= hi
                   for lo, v, hi in zip(self.low, center, self.high))

    def segment_interval(self, start, end, padding):
        """Closed [entry, exit] parameters inside an inflated box, or None."""
        enter, leave = 0.0, 1.0
        for lo, hi, a, b in zip(self.low, self.high, start, end):
            lo, hi = lo - padding, hi + padding
            delta = b - a
            if delta == 0:
                if a < lo or a > hi:
                    return None
                continue
            first, last = sorted(((lo - a) / delta, (hi - a) / delta))
            enter, leave = max(enter, first), min(leave, last)
            if enter > leave:
                return None
        return enter, leave


@dataclass(frozen=True)
class Station:
    id: str
    bounds: Box
    work_corridor: Box | None = None
    work_point: Point | None = None
    solid_parts: tuple[Box, ...] = ()

    def __post_init__(self):
        if not self.id:
            raise ValueError("station ID is required")
        if (self.work_corridor is None) != (self.work_point is None):
            raise ValueError("a station access needs both a work corridor and a work point")
        if self.work_point is not None:
            object.__setattr__(self, "work_point", point(self.work_point))
        object.__setattr__(self, "solid_parts", tuple(self.solid_parts))
        for part in self.solid_parts:
            if not (self.bounds.contains_envelope(part.low, 0)
                    and self.bounds.contains_envelope(part.high, 0)):
                raise ValueError("station bounds must include every solid part")

    @property
    def top_z(self):
        """Absolute height of the top in the robot base frame (not above TCP)."""
        return self.bounds.high[2]


# Access cannot be supplied as an arbitrary 'ignore obstacle' flag by callers.
STATION_ACCESS = {
    Step.TO_SPRAYER_1: ("sprayer", "enter"),
    Step.SPRAY_1: ("sprayer", "work"),
    Step.LEAVE_SPRAYER_1: ("sprayer", "exit"),
    Step.TO_SPONGE: ("sponge", "enter"),
    Step.SPONGE: ("sponge", "work"),
    Step.LEAVE_SPONGE: ("sponge", "exit"),
    Step.TO_SPRAYER_2: ("sprayer", "enter"),
    Step.SPRAY_2: ("sprayer", "work"),
    Step.LEAVE_SPRAYER_2: ("sprayer", "exit"),
    Step.TO_WIPER: ("wiper", "enter"),
    Step.WIPE: ("wiper", "work"),
    Step.LEAVE_WIPER: ("wiper", "exit"),
}


@dataclass(frozen=True)
class Workspace:
    stations: tuple[Station, ...]
    tool_radius: float       # m, bounds gripper + glass for every rotation
    clearance: float         # m, additional clearance from station obstacles
    min_tcp_z: float          # m, preserve the actual robot configuration
    max_tcp_z: float          # m, must not exceed SafeControl's ceiling in hardware

    def __post_init__(self):
        object.__setattr__(self, "stations", tuple(self.stations))
        if not all(isfinite(v) for v in (self.tool_radius, self.clearance,
                                         self.min_tcp_z, self.max_tcp_z)):
            raise ValueError("workspace dimensions must be finite")
        if self.tool_radius <= 0 or self.clearance < 0 or self.min_tcp_z >= self.max_tcp_z:
            raise ValueError("invalid tool radius, clearance or TCP height limits")
        if len({s.id for s in self.stations}) != len(self.stations):
            raise ValueError("station IDs must be unique")
        for station in self.stations:
            if station.work_point is not None and not station.work_corridor.contains_envelope(
                    station.work_point, self.extent):
                raise ValueError(f"{station.id}: work corridor does not fit tool + clearance")

    @property
    def extent(self):
        return self.tool_radius + self.clearance

    def station(self, station_id):
        for station in self.stations:
            if station.id == station_id:
                return station
        raise CollisionRefused(f"no geometry for station {station_id}")

    def transport_height(self, *poses):
        """Conservative common crossing height; never clamp an unsafe height."""
        top = max((s.top_z + self.extent + GEOMETRY_EPSILON for s in self.stations),
                  default=self.min_tcp_z)
        height = max([top, *(point(p)[2] for p in poses)])
        if height > self.max_tcp_z:
            raise CollisionRefused(f"station crossing requires TCP z={height:.3f} m, "
                                   f"above ceiling {self.max_tcp_z:.3f} m")
        return height

    def transfer_path(self, start, destination, step):
        """Raise, cross, descend; validation also checks the vertical legs."""
        start, destination = point(start), point(destination)
        height = self.transport_height(start, destination)
        path = (start, (start[0], start[1], height),
                (destination[0], destination[1], height), destination)
        return self.validate_path(path, step)

    def validate_path(self, path, step):
        """Return the checked polyline or raise before any motion is dispatched."""
        if not isinstance(step, Step):
            raise CollisionRefused("unknown operation stage")
        path = tuple(point(p) for p in path)
        if not path:
            raise CollisionRefused("empty path is not a checked motion")
        if any(not self.min_tcp_z <= p[2] <= self.max_tcp_z for p in path):
            raise CollisionRefused("planned TCP height is outside configured limits")
        access = STATION_ACCESS.get(step)
        active, mode = (self.station(access[0]), access[1]) if access else (None, None)
        if active is not None:
            corridor = active.work_corridor
            if corridor is None:
                raise CollisionRefused(f"{active.id}: no taught work corridor")
            required = path if mode == "work" else (path[-1] if mode == "enter" else path[0],)
            if not all(corridor.contains_envelope(p, self.extent) for p in required):
                raise CollisionRefused(f"{active.id}: tool is outside the taught work corridor")
            if mode == "exit" and active.bounds.segment_interval(path[-1], path[-1], self.extent):
                raise CollisionRefused(f"{active.id}: withdrawal does not clear the station")
        segments = tuple(zip(path, path[1:])) or ((path[0], path[0]),)
        for index, (start, end) in enumerate(segments):
            for station in self.stations:
                for solid in station.solid_parts:
                    if solid.segment_interval(start, end, self.extent) is not None:
                        raise CollisionRefused(f"{step.value}: collision with solid part of {station.id}")
                interval = station.bounds.segment_interval(start, end, self.extent)
                if interval is None:
                    continue
                permitted = station is active and (mode == "work"
                            or (mode == "enter" and index == len(segments) - 1)
                            or (mode == "exit" and index == 0))
                if permitted and all(station.work_corridor.contains_envelope(
                        _between(start, end, t), self.extent) for t in interval):
                    continue
                raise CollisionRefused(f"{step.value}: collision with station {station.id}")
        return path
