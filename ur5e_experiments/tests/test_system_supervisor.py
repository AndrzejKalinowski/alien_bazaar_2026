"""Supervisor process invariants on fake devices; never connect to hardware."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import pytest

from system_batch import OutputSlot
from system_controller import Supervisor
from system_main import main
from system_model import GlassTarget, Grip, Outcome, Result, SlotState, State, Step
from system_operations import RECIPE
from system_settings import SCENE_MAX_AGE, STOP_TIMEOUT, TELEMETRY_MAX_AGE, TICK_PERIOD
from system_simulator import SimulatedDevices, SimulationClock


def make_system(count=1, capacity=None, **device_options):
    clock = SimulationClock()
    targets = [GlassTarget(f"g{i}", 0.2 + i * 0.1, -0.4) for i in range(count)]
    slots = [OutputSlot(f"t{i}", SlotState.FREE) for i in range(capacity or count)]
    devices = SimulatedDevices(targets, **device_options)
    supervisor = Supervisor(devices, slots, clock)
    return supervisor, devices, clock


def tick_until(system, predicate, limit=5000):
    supervisor, devices, clock = system
    for _ in range(limit):
        if predicate():
            return
        clock.advance(TICK_PERIOD)
        supervisor.tick()
    pytest.fail(f"condition not reached: {supervisor.status()}")


def run_to_step(system, step):
    supervisor, _, _ = system
    assert supervisor.start("start").accepted
    tick_until(system, lambda: supervisor.command is not None and supervisor.command.step == step)


def test_full_batch_order_and_unique_output_reservations():
    system = make_system(3)
    supervisor, devices, _ = system
    assert supervisor.start("start").accepted
    tick_until(system, lambda: supervisor.state == State.COMPLETED)
    expected = ["PICK", "LIFT", "FLIP", "TO_SPRAYER_1", "SPRAY_1", "LEAVE_SPRAYER_1",
                "TO_SPONGE", "SPONGE", "LEAVE_SPONGE", "TO_SPRAYER_2", "SPRAY_2",
                "LEAVE_SPRAYER_2", "TO_WIPER", "WIPE", "LEAVE_WIPER", "TO_OUTPUT", "LOWER",
                "RELEASE", "RETREAT", "OBSERVE"]
    for target_id in ("g0", "g1", "g2"):
        assert [c.step.value for c in devices.history if c.target.id == target_id] == expected
    assert devices.placements == [
        {"target_id": f"g{i}", "slot_id": f"t{i}", "orientation": "down"} for i in range(3)]
    assert len({c.id for c in devices.history}) == len(devices.history)
    assert all(s.state == SlotState.OCCUPIED for s in supervisor.outputs.slots.values())
    assert supervisor.status()["counts"]["COMPLETED"] == 3
    assert not devices.targets and devices.grip == Grip.NO


def test_full_output_waits_before_pick_and_operator_clear_continues_same_batch():
    system = make_system(2, capacity=1)
    supervisor, devices, _ = system
    batch_id = supervisor.start("start").batch_id
    tick_until(system, lambda: supervisor.state == State.WAITING_OUTPUT)
    assert [c.target.id for c in devices.history if c.step == Step.PICK] == ["g0"]
    assert devices.grip == Grip.NO
    assert supervisor.confirm_output_cleared(["t0"]).accepted
    tick_until(system, lambda: supervisor.state == State.COMPLETED)
    assert supervisor.batch.id == batch_id
    assert len(devices.placements) == 2


def test_start_is_idempotent_even_after_completion_and_busy_start_is_refused():
    system = make_system()
    supervisor, devices, _ = system
    reply = supervisor.start("same-id")
    assert supervisor.start("same-id") == reply
    assert not supervisor.start("another-id").accepted
    tick_until(system, lambda: supervisor.state == State.COMPLETED)
    size = len(devices.history)
    assert supervisor.start("same-id") == reply
    supervisor.tick()
    assert supervisor.state == State.COMPLETED and len(devices.history) == size


@pytest.mark.parametrize("step", list(Step))
def test_stop_at_every_stage_invalidates_commands_and_preserves_vacuum(step):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, step)
    command = supervisor.command
    grip = devices.grip
    history_size = len(devices.history)
    supervisor.request_stop()
    supervisor.request_stop()  # repeated STOP does not enqueue another stop
    assert supervisor.state == State.STOPPING
    assert len(devices.stop_requests) == 1
    assert not supervisor.start("late-start").accepted
    tick_until(system, lambda: supervisor.state == State.STOPPED)
    assert supervisor.status()["stop_confirmed"]
    assert devices.grip == grip
    assert len(devices.history) == history_size
    assert devices.poll(command.id, clock() + 100) is None
    expected = SlotState.OCCUPIED if step in (Step.RETREAT, Step.OBSERVE) else SlotState.UNKNOWN
    assert supervisor.outputs.slots["t0"].state == expected


@pytest.mark.parametrize("step", list(Step))
def test_failure_never_starts_the_next_operation(step):
    system = make_system(fail_at=step)
    supervisor, devices, _ = system
    supervisor.start("start")
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert devices.history[-1].step == step
    assert "injected simulated failure" in supervisor.fault
    assert supervisor.status()["counts"]["INTERRUPTED"] == 1
    assert not supervisor.start("restart-without-reset").accepted


@pytest.mark.parametrize("grip", [Grip.UNKNOWN, Grip.NO, Grip.LOST])
def test_pick_success_without_vacuum_confirmation_never_lifts(grip, monkeypatch):
    system = make_system()
    supervisor, devices, _ = system
    original = devices.poll

    def poll(command_id, now):
        result = original(command_id, now)
        if result is not None:
            devices.grip = grip
        return result

    monkeypatch.setattr(devices, "poll", poll)
    supervisor.start("start")
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert [c.step for c in devices.history] == [Step.PICK]
    assert "pick not confirmed" in supervisor.fault


@pytest.mark.parametrize("step", [Step.LIFT, Step.FLIP, Step.SPRAY_1, Step.SPONGE, Step.WIPE])
@pytest.mark.parametrize("grip", [Grip.UNKNOWN, Grip.LOST])
def test_grip_monitor_also_covers_station_waits(step, grip):
    system = make_system()
    supervisor, devices, _ = system
    run_to_step(system, step)
    devices.grip = grip
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert devices.history[-1].step == step
    assert not devices.placements
    assert devices.grip == grip  # STOP never sent RELEASE


def test_lower_limit_without_support_does_not_release(monkeypatch):
    system = make_system()
    supervisor, devices, _ = system
    original = devices.poll

    def poll(command_id, now):
        result = original(command_id, now)
        if result is not None and devices.history[-1].step == Step.LOWER:
            devices.supported = False
        return result

    monkeypatch.setattr(devices, "poll", poll)
    supervisor.start("start")
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert devices.history[-1].step == Step.LOWER
    assert devices.grip == Grip.OK and not devices.placements


def test_release_needs_confirmed_empty_grip(monkeypatch):
    system = make_system()
    supervisor, devices, _ = system
    run_to_step(system, Step.RELEASE)
    monkeypatch.setattr(devices, "poll", lambda command_id, now: Result(command_id, Outcome.SUCCEEDED))
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert devices.history[-1].step == Step.RELEASE
    assert supervisor.outputs.slots["t0"].state == SlotState.UNKNOWN


def test_timeout_precedes_late_success(monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, Step.PICK)
    monkeypatch.setattr(devices, "poll", lambda command_id, now: pytest.fail("expired command polled"))
    clock.advance(RECIPE[0].timeout)
    supervisor.tick()
    assert supervisor.state == State.STOPPING and "timed out" in supervisor.fault
    tick_until(system, lambda: supervisor.state == State.FAULT)


def test_wrong_command_reply_cannot_advance_operation(monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, Step.PICK)
    monkeypatch.setattr(devices, "poll", lambda command_id, now: Result("old-id", Outcome.SUCCEEDED))
    clock.advance(0.2)
    supervisor.tick()
    assert supervisor.state == State.RUNNING and supervisor.command.step == Step.PICK
    clock.advance(RECIPE[0].timeout)
    supervisor.tick()
    assert supervisor.state == State.STOPPING


@pytest.mark.parametrize("source", ["telemetry", "camera"])
def test_stale_observations_stop_an_active_cycle(source, monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, Step.SPONGE)
    if source == "telemetry":
        method = devices.telemetry
        monkeypatch.setattr(devices, "telemetry", lambda now: replace(
            method(now), observed_at=now - TELEMETRY_MAX_AGE - 1))
    else:
        method = devices.observe
        monkeypatch.setattr(devices, "observe", lambda now: replace(
            method(now), observed_at=now - SCENE_MAX_AGE - 1))
    supervisor.tick()
    assert supervisor.state == State.STOPPING and "stale" in supervisor.fault
    clock.advance(STOP_TIMEOUT + 0.1)
    supervisor.tick()
    assert supervisor.state == State.FAULT


def test_unacknowledged_stop_cannot_reset(monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, Step.PICK)
    monkeypatch.setattr(devices, "poll_stop", lambda stop_id, now: None)
    supervisor.request_stop()
    clock.advance(STOP_TIMEOUT)
    supervisor.tick()
    assert supervisor.state == State.FAULT
    assert not supervisor.reset_fault().accepted
    assert not supervisor.status()["stop_confirmed"]


def test_stop_ack_without_station_stop_does_not_report_stopped(monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    run_to_step(system, Step.SPONGE)
    monkeypatch.setattr(devices, "poll_stop", lambda stop_id, now: Result(stop_id, Outcome.SUCCEEDED))
    supervisor.request_stop()
    supervisor.tick()
    assert supervisor.state == State.STOPPING
    clock.advance(STOP_TIMEOUT)
    supervisor.tick()
    assert supervisor.state == State.FAULT


def test_reset_requires_empty_grip_and_resolved_output_then_waits_for_start():
    system = make_system()
    supervisor, devices, _ = system
    run_to_step(system, Step.SPONGE)
    supervisor.request_stop()
    tick_until(system, lambda: supervisor.state == State.STOPPED)
    assert not supervisor.reset_fault().accepted
    # Simulate operator removal on a support, not an automatic release/reset.
    devices.grip = Grip.NO
    assert not supervisor.reset_fault().accepted
    assert supervisor.confirm_output_cleared(["t0"]).accepted
    assert supervisor.reset_fault().accepted
    size = len(devices.history)
    supervisor.tick()
    assert supervisor.state == State.READY and len(devices.history) == size
    assert supervisor.start("start").accepted  # duplicate ID returns old receipt only
    assert supervisor.state == State.READY


def test_new_targets_are_excluded_and_existing_target_position_is_refreshed():
    system = make_system()
    supervisor, devices, _ = system
    supervisor.start("start")
    devices.targets["g0"] = GlassTarget("g0", 0.25, -0.3)
    devices.targets["new"] = GlassTarget("new", 0.6, -0.3)
    tick_until(system, lambda: supervisor.state == State.COMPLETED)
    pick = next(c for c in devices.history if c.step == Step.PICK)
    assert pick.target.x == 0.25 and pick.target.y == -0.3
    assert [p["target_id"] for p in devices.placements] == ["g0"]
    assert "new" in devices.targets


def test_missing_target_is_accounted_for_without_picking_another_glass():
    system = make_system()
    supervisor, devices, _ = system
    supervisor.start("start")
    devices.targets.clear()
    tick_until(system, lambda: supervisor.state == State.COMPLETED)
    assert supervisor.status()["counts"]["SKIPPED"] == 1
    assert not devices.history
    assert supervisor.outputs.slots["t0"].state == SlotState.FREE


def test_unchanged_frame_is_not_counted_as_multiple_observations(monkeypatch):
    system = make_system()
    supervisor, devices, clock = system
    scene = devices.observe(clock())
    monkeypatch.setattr(devices, "observe", lambda now: scene)
    assert supervisor.start("start").accepted
    for _ in range(3):
        clock.advance(TICK_PERIOD)
        supervisor.tick()
    assert not devices.history
    clock.advance(SCENE_MAX_AGE)
    supervisor.tick()
    assert supervisor.state == State.STOPPING


@pytest.mark.parametrize("condition", ["disconnected", "unknown_grip", "moving", "unknown_output"])
def test_start_preflight_refusal_does_not_send_commands(condition):
    supervisor, devices, _ = make_system()
    if condition == "disconnected":
        devices.connected = False
    elif condition == "unknown_grip":
        devices.grip = Grip.UNKNOWN
    elif condition == "moving":
        devices.robot_stopped = False
    else:
        supervisor.outputs.slots["t0"].state = SlotState.UNKNOWN
    assert not supervisor.start("start").accepted
    assert supervisor.state == State.READY and not devices.history


def test_clear_output_validation_is_atomic():
    supervisor, _, _ = make_system(2)
    supervisor.outputs.slots["t0"].state = SlotState.OCCUPIED
    assert not supervisor.confirm_output_cleared(["t0", "absent"]).accepted
    assert supervisor.outputs.slots["t0"].state == SlotState.OCCUPIED
    supervisor.start("start")
    assert not supervisor.confirm_output_cleared(["t0"]).accepted


def test_empty_scene_refuses_start_without_moving():
    supervisor, devices, _ = make_system()
    devices.targets.clear()
    assert not supervisor.start("start").accepted
    assert supervisor.state == State.READY and not devices.history


def test_checked_reset_accepts_restarted_camera_sequence():
    system = make_system()
    supervisor, devices, _ = system
    run_to_step(system, Step.PICK)
    devices._sequence = -1  # the camera process restarts before pick completes
    supervisor.tick()
    assert supervisor.state == State.STOPPING
    tick_until(system, lambda: supervisor.state == State.FAULT)
    assert "sequence went backwards" in supervisor.fault
    assert supervisor.confirm_output_cleared(["t0"]).accepted
    assert supervisor.reset_fault().accepted
    assert supervisor.start("new-start").accepted
    tick_until(system, lambda: supervisor.state == State.COMPLETED)


def test_return_of_connection_never_resumes_interrupted_motion():
    system = make_system()
    supervisor, devices, _ = system
    run_to_step(system, Step.WIPE)
    size = len(devices.history)
    devices.connected = False
    supervisor.tick()
    assert supervisor.state == State.STOPPING
    devices.connected = True
    tick_until(system, lambda: supervisor.state == State.FAULT)
    supervisor.tick()
    assert len(devices.history) == size
    assert devices.grip == Grip.OK


def test_dispatch_and_stop_exceptions_latch_fault_without_escaping(monkeypatch):
    system = make_system()
    supervisor, devices, _ = system

    def fail(*args):
        raise OSError("serial transport failed")

    monkeypatch.setattr(devices, "begin", fail)
    monkeypatch.setattr(devices, "begin_stop", fail)
    assert supervisor.start("start").accepted
    supervisor.tick()
    assert supervisor.state == State.FAULT
    assert "stop dispatch failed" in supervisor.fault
    assert not supervisor.reset_fault().accepted
    assert supervisor.outputs.slots["t0"].state == SlotState.UNKNOWN


@pytest.mark.parametrize("options,code,state", [
    ([], 0, "COMPLETED"),
    (["--slots", "1"], 3, "WAITING_OUTPUT"),
    (["--slots", "1", "--auto-clear-output"], 0, "COMPLETED"),
    (["--fail-at", "SPONGE"], 2, "FAULT"),
    (["--stop-at", "WIPE"], 4, "STOPPED"),
])
def test_cli_scenarios(options, code, state, capsys):
    assert main(["--simulate", "--fast", "--quiet", *options]) == code
    assert json.loads(capsys.readouterr().out)["state"] == state


def test_cli_requires_explicit_simulation(capsys):
    with pytest.raises(SystemExit) as error:
        main([])
    assert error.value.code == 2
    assert "--simulate" in capsys.readouterr().err


def test_journal_has_ordered_events_and_is_never_overwritten(tmp_path, capsys):
    path = tmp_path / "events.jsonl"
    args = ["--simulate", "--fast", "--quiet", "--journal", str(path)]
    assert main(args) == 0
    original = path.read_text(encoding="utf-8")
    events = [json.loads(line) for line in original.splitlines()]
    assert [e["sequence"] for e in events] == list(range(1, len(events) + 1))
    assert events[-1]["event"] == "batch_completed"
    assert main(args) == 1
    assert path.read_text(encoding="utf-8") == original
    capsys.readouterr()


def test_entry_point_does_not_import_hardware_or_vision_libraries():
    module_dir = Path(__file__).resolve().parents[1]
    code = (f"import sys; sys.path.insert(0, {str(module_dir)!r}); import system_main; "
            "assert not {'serial', 'cv2', 'rtde_control', 'rtde_receive', 'find_glasses', "
            "'glass_classifier', 'table_background'} & set(sys.modules)")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
