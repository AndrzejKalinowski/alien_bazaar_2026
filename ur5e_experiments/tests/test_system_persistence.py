"""Output occupancy across restarts and the single hardware instance lock."""

import json

import pytest

from system_batch import OutputStore
from system_controller import Supervisor
from system_main import instance_lock
from system_model import SlotState, State
from system_simulator import SimulatedDevices, SimulationClock

IDS = ["tag-1", "tag-2", "tag-3"]


def states(slots):
    return [s.state for s in slots]


def test_missing_file_starts_unknown_and_marks_the_session_unclean(tmp_path):
    store = OutputStore(str(tmp_path / "outputs.json"))
    assert states(store.load_slots(IDS)) == [SlotState.UNKNOWN] * 3
    assert json.loads((tmp_path / "outputs.json").read_text())["clean_shutdown"] is False


def test_clean_restart_keeps_free_and_occupied(tmp_path):
    path = tmp_path / "outputs.json"
    path.write_text(json.dumps({"clean_shutdown": True, "slots": {
        "tag-1": {"state": "FREE"}, "tag-2": {"state": "OCCUPIED", "target_id": "glass-4"},
        "tag-3": {"state": "RESERVED", "target_id": "glass-5"}}}))
    slots = OutputStore(str(path)).load_slots(IDS)
    assert states(slots) == [SlotState.FREE, SlotState.OCCUPIED, SlotState.UNKNOWN]
    assert slots[1].target_id == "glass-4" and slots[2].target_id is None


def test_unclean_restart_never_frees_a_place(tmp_path):
    path = tmp_path / "outputs.json"
    path.write_text(json.dumps({"clean_shutdown": False, "slots": {
        "tag-1": {"state": "FREE"}, "tag-2": {"state": "OCCUPIED"}}}))
    assert states(OutputStore(str(path)).load_slots(IDS)) == [
        SlotState.UNKNOWN, SlotState.OCCUPIED, SlotState.UNKNOWN]


@pytest.mark.parametrize("content", ["not json", "[1, 2]", '{"slots": {"tag-1": {"state": "BOGUS"}}}'])
def test_unreadable_or_bad_file_is_all_unknown(tmp_path, content):
    path = tmp_path / "outputs.json"
    path.write_text(content)
    store = OutputStore(str(path))
    assert states(store.load_slots(IDS)) == [SlotState.UNKNOWN] * 3


def test_supervisor_changes_are_saved_and_a_clean_close_is_recorded(tmp_path):
    path = tmp_path / "outputs.json"
    store = OutputStore(str(path))
    slots = store.load_slots(["tag-1"])
    clock = SimulationClock()
    supervisor = Supervisor(SimulatedDevices([]), slots, clock)
    sink = store.sink(supervisor)
    supervisor.confirm_output_cleared(["tag-1"])
    sink(supervisor.drain_events())
    saved = json.loads(path.read_text())
    assert saved["slots"]["tag-1"]["state"] == "FREE" and saved["clean_shutdown"] is False
    store.close(supervisor, stopped=supervisor.state == State.READY)
    assert json.loads(path.read_text())["clean_shutdown"] is True
    assert states(OutputStore(str(path)).load_slots(["tag-1"])) == [SlotState.FREE]
    assert not (tmp_path / "outputs.json.tmp").exists()


def test_second_hardware_instance_is_refused(tmp_path):
    path = str(tmp_path / "supervisor.lock")
    first = instance_lock(path)
    try:
        with pytest.raises(RuntimeError, match="another hardware supervisor"):
            instance_lock(path)
    finally:
        first.close()
    instance_lock(path).close()  # released with the process / handle


def test_hardware_mode_wiring_with_fake_devices(tmp_path, monkeypatch):
    """system_main --hardware end to end, with connect() returning fakes."""
    import socket
    import threading

    import system_batch
    import system_hardware
    import system_main
    import system_web
    from test_system_hardware import System

    fake = System(tmp_path)
    monkeypatch.setattr(system_hardware, "connect", lambda slot_ids, vision: fake.hw)
    monkeypatch.setattr(system_main, "LOCK_FILE", str(tmp_path / "supervisor.lock"))
    monkeypatch.setattr(system_main, "LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setattr(system_batch, "OUTPUT_FILE", str(tmp_path / "outputs.json"))
    original_run = system_web.ControlLoop.run

    def short_run(loop):
        threading.Timer(0.3, loop.shutdown).start()
        original_run(loop)

    monkeypatch.setattr(system_web.ControlLoop, "run", short_run)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    assert system_main.main(["--hardware", "--web", "--teach", "--port", str(port), "--slots", "1"]) == 0
    assert json.loads((tmp_path / "outputs.json").read_text())["clean_shutdown"] is True
    assert list((tmp_path / "logs").glob("supervisor-*.jsonl"))
    assert ("stopScript",) in fake.robot.calls   # devices.close() on the way out
