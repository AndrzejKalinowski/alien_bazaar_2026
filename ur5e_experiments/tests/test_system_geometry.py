"""Station height/swept-volume regressions; no real robot, camera or serial I/O."""

from dataclasses import replace
from math import inf, nan

import pytest

from system_batch import OutputSlot
from system_controller import Supervisor
from system_geometry import Box, CollisionRefused, Station, Workspace
from system_model import Command, GlassTarget, SlotState, State, Step
from system_simulator import (SIM_OBSERVATION_POSE, SimulatedDevices, SimulationClock,
                              example_workspace)


@pytest.fixture
def obstacle_world():
    return Workspace((Station("rack", Box((-0.1, -0.1, 0.0), (0.1, 0.1, 0.2))),),
                     tool_radius=0.04, clearance=0.01, min_tcp_z=0.0, max_tcp_z=0.6)


def test_clear_endpoints_do_not_make_a_crossing_segment_safe(obstacle_world):
    start, end = (-1.0, 0.0, 0.1), (1.0, 0.0, 0.1)
    obstacle_world.validate_path((start,), Step.TO_OUTPUT)
    obstacle_world.validate_path((end,), Step.TO_OUTPUT)
    with pytest.raises(CollisionRefused, match="rack"):
        obstacle_world.validate_path((start, end), Step.TO_OUTPUT)


@pytest.mark.parametrize("height", [0.20, 0.23, 0.25])
def test_tcp_above_station_is_insufficient_and_touching_margin_is_refused(obstacle_world, height):
    with pytest.raises(CollisionRefused):
        obstacle_world.validate_path(((-0.5, 0.0, height), (0.5, 0.0, height)), Step.TO_OUTPUT)


def test_transfer_raises_whole_envelope_over_station(obstacle_world):
    path = obstacle_world.transfer_path((-0.5, 0.0, 0.1), (0.5, 0.0, 0.1), Step.TO_OUTPUT)
    assert path[1][2] > 0.25
    assert path[1][2] == path[2][2]
    assert path[-1][2] == 0.1


def test_transport_uses_the_highest_station():
    world = example_workspace()
    assert world.transport_height() > world.station("wiper").top_z + world.extent
    assert world.transport_height() < world.max_tcp_z


def test_height_above_ceiling_is_refused_instead_of_clamped(obstacle_world):
    world = replace(obstacle_world, max_tcp_z=0.24)
    with pytest.raises(CollisionRefused, match="ceiling"):
        world.transfer_path((-0.5, 0.0, 0.1), (0.5, 0.0, 0.1), Step.TO_OUTPUT)


def test_vertical_departure_from_obstacle_is_also_checked(obstacle_world):
    with pytest.raises(CollisionRefused, match="rack"):
        obstacle_world.transfer_path((0.0, 0.0, 0.1), (0.5, 0.0, 0.1), Step.TO_OUTPUT)


def test_taught_side_detour_can_pass_below_station_top(obstacle_world):
    path = ((-0.5, 0.0, 0.1), (-0.5, 0.3, 0.1), (0.5, 0.3, 0.1), (0.5, 0.0, 0.1))
    assert obstacle_world.validate_path(path, Step.TO_OUTPUT) == path


def test_flip_checks_full_orientation_envelope_even_without_tcp_translation(obstacle_world):
    with pytest.raises(CollisionRefused):
        obstacle_world.validate_path(((0.0, 0.0, 0.23),), Step.FLIP)
    obstacle_world.validate_path(((0.0, 0.0, 0.3),), Step.FLIP)


@pytest.mark.parametrize("step", [Step.PICK, Step.FLIP, Step.TO_OUTPUT, Step.RETREAT, Step.OBSERVE])
def test_work_corridor_is_forbidden_during_unrelated_operations(step):
    world = example_workspace()
    with pytest.raises(CollisionRefused, match="sprayer"):
        world.validate_path((world.station("sprayer").work_point,), step)


def test_approach_work_and_exit_only_use_their_station_corridor():
    world = example_workspace()
    work = world.station("sprayer").work_point
    approach = world.transfer_path(SIM_OBSERVATION_POSE, work, Step.TO_SPRAYER_1)
    world.validate_path((work,), Step.SPRAY_1)
    above = (work[0], work[1], world.transport_height(work))
    world.validate_path((work, above), Step.LEAVE_SPRAYER_1)
    with pytest.raises(CollisionRefused):
        world.validate_path(approach, Step.TO_SPONGE)
    with pytest.raises(CollisionRefused):
        world.validate_path((work,), Step.SPONGE)


def test_approach_permission_does_not_disable_collision_checks_for_transit_segments():
    world = example_workspace()
    work = world.station("sprayer").work_point
    above = (work[0], work[1], 0.45)
    with pytest.raises(CollisionRefused, match="collision"):
        world.validate_path((above, work, above, work), Step.TO_SPRAYER_1)


def test_withdrawal_must_end_outside_station():
    world = example_workspace()
    work = world.station("sprayer").work_point
    with pytest.raises(CollisionRefused, match="does not clear"):
        world.validate_path((work, (work[0], work[1], work[2] + 0.01)), Step.LEAVE_SPRAYER_1)


def test_work_operation_must_stay_inside_taught_corridor():
    world = example_workspace()
    work = world.station("sprayer").work_point
    with pytest.raises(CollisionRefused, match="outside"):
        world.validate_path((work, (work[0] + 0.2, work[1], work[2])), Step.SPRAY_1)


def test_active_station_solid_parts_remain_forbidden():
    world = example_workspace()
    station = world.station("sprayer")
    x, y, _ = station.work_point
    solid = Box((x - 0.01, y - 0.01, 0.08), (x + 0.01, y + 0.01, 0.10))
    modified = replace(station, solid_parts=(solid,))
    world = replace(world, stations=(modified, *world.stations[1:]))
    with pytest.raises(CollisionRefused, match="solid part"):
        world.validate_path((station.work_point,), Step.SPRAY_1)


def test_other_stations_remain_forbidden_inside_active_station_corridor():
    world = example_workspace()
    work = world.station("sprayer").work_point
    obstacle = Station("another fixture", Box((0.19, 0.29, 0.05), (0.21, 0.31, 0.15)))
    world = replace(world, stations=(*world.stations, obstacle))
    with pytest.raises(CollisionRefused, match="another fixture"):
        world.validate_path((work,), Step.SPRAY_1)


@pytest.mark.parametrize("bad", [nan, inf, -inf])
def test_nonfinite_geometry_is_rejected(bad, obstacle_world):
    with pytest.raises(ValueError):
        Box((0.0, 0.0, 0.0), (1.0, 1.0, bad))
    with pytest.raises(ValueError):
        obstacle_world.validate_path(((0.0, bad, 0.3),), Step.FLIP)
    with pytest.raises(ValueError):
        replace(obstacle_world, clearance=bad)


def test_oversized_tool_cannot_be_squeezed_into_corridor():
    with pytest.raises(ValueError, match="does not fit"):
        replace(example_workspace(), tool_radius=0.3)


def _controller_for(target, world=None):
    clock = SimulationClock()
    devices = SimulatedDevices((target,), workspace=world)
    controller = Supervisor(devices, [OutputSlot("slot", SlotState.FREE)], clock)
    assert controller.start("start").accepted
    return controller, devices


def test_supervisor_refuses_pick_inside_station_before_dispatch():
    controller, devices = _controller_for(GlassTarget("glass", 0.2, 0.3))
    controller.tick()
    assert controller.state == State.STOPPING
    assert "collision with station sprayer" in controller.fault
    assert not devices.history


def test_supervisor_refuses_impossible_transit_height_before_dispatch():
    world = example_workspace()
    high = Station("tall station", Box((2.0, 2.0, 0.0), (2.2, 2.2, 0.57)))
    world = replace(world, stations=(*world.stations, high))
    controller, devices = _controller_for(GlassTarget("glass", 0.2, -0.4), world)
    controller.tick()
    assert controller.state == State.STOPPING and "ceiling" in controller.fault
    assert not devices.history


@pytest.mark.parametrize("change", ["unvalidated", "pose", "layout", "command"])
def test_unvalidated_or_changed_plan_cannot_be_dispatched(change):
    target = GlassTarget("glass", 0.2, -0.4)
    devices = SimulatedDevices((target,))
    command = Command("id", "batch", target, "slot", Step.PICK, 0.0)
    if change != "unvalidated":
        devices.validate_motion(command)
    if change == "pose":
        devices.tcp = (0.01, -0.4, 0.45)
    elif change == "layout":
        devices.workspace = replace(devices.workspace, clearance=0.021)
    elif change == "command":
        command = replace(command, id="different")
    with pytest.raises(CollisionRefused, match="must be validated"):
        devices.begin(command)
    assert not devices.history
