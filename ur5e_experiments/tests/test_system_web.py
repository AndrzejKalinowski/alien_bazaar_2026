"""Control loop ownership and the web panel API, on simulated devices only.

The HTTP tests bind an ephemeral loopback port; nothing touches hardware.
"""

import http.client
import json
import socket
import threading
import time

import pytest

from system_batch import OutputSlot
from system_controller import Supervisor
from system_main import main
from system_model import GlassTarget, Reply, SlotState, State
import system_web
from system_web import COMMAND_QUEUE_SIZE, ControlLoop, FrameHub, serve
from system_settings import TICK_PERIOD
from system_simulator import SimulatedDevices, SimulationClock



def make_loop(count=2, capacity=None, clock=None, **device_options):
    clock = clock or SimulationClock()
    targets = [GlassTarget(f"g{i}", 0.2 + i * 0.1, -0.4) for i in range(count)]
    slots = [OutputSlot(f"t{i}", SlotState.FREE) for i in range(capacity or count)]
    devices = SimulatedDevices(targets, **device_options)
    supervisor = Supervisor(devices, slots, clock)
    actions = {"add_glasses": lambda p: Reply(True, str(devices.add_targets(p["count"])))}
    loop = ControlLoop(supervisor, actions=actions, world=devices.snapshot, wall_clock=clock)
    return loop, devices, clock


def step_until(loop, clock, predicate, limit=5000):
    for _ in range(limit):
        if predicate():
            return
        clock.advance(TICK_PERIOD)
        loop.step()
    pytest.fail(f"condition not reached: {loop.status()['state']}")


# --- ControlLoop (single thread, virtual time) ----------------------------------

def test_commands_run_in_the_owner_step_and_report_the_supervisor_reply():
    loop, devices, clock = make_loop()
    record = loop.submit("start", command_id="s1")
    assert record["status"] == "queued" and loop.supervisor.state == State.READY
    loop.step()
    assert loop.result("s1") | {} == {"id": "s1", "kind": "start", "status": "done", "accepted": True,
                                      "message": "batch started", "batch_id": loop.supervisor.batch.id}
    step_until(loop, clock, lambda: loop.supervisor.state == State.COMPLETED)
    status = loop.status()
    assert status["counts"]["COMPLETED"] == 2
    assert status["recipe"][0] == "PICK" and status["step_index"] is None
    assert status["world"]["simulated"] and len(status["world"]["outputs"]) == 2


def test_retried_command_id_is_not_queued_twice():
    loop, _, _ = make_loop()
    first = loop.submit("start", command_id="same")
    assert loop.submit("start", command_id="same") == first
    loop.step()
    loop.step()
    assert loop.result("same")["accepted"]
    assert loop.submit("start", command_id="same")["status"] == "done"


def test_stop_takes_priority_and_cancels_commands_queued_before_it():
    loop, devices, _ = make_loop()
    loop.submit("start", command_id="late-start")
    stop = loop.stop()
    loop.step()
    assert loop.result("late-start")["status"] == "cancelled"
    assert loop.result(stop["id"])["status"] == "done"
    assert not devices.history, "a START queued before STOP must never run"
    assert loop.supervisor.state in (State.STOPPING, State.STOPPED)


def test_stop_during_a_batch_preserves_vacuum_and_needs_reset():
    loop, devices, clock = make_loop()
    loop.submit("start", command_id="s")
    step_until(loop, clock, lambda: loop.supervisor.status()["operation"] == "SPONGE")
    loop.stop()
    step_until(loop, clock, lambda: loop.supervisor.state == State.STOPPED)
    assert devices.grip.value == "OK"
    loop.submit("start", command_id="s2")
    loop.step()
    assert not loop.result("s2")["accepted"]


def test_full_queue_refuses_instead_of_blocking():
    loop, _, _ = make_loop()
    for i in range(COMMAND_QUEUE_SIZE):
        assert loop.submit("reset", command_id=f"r{i}")["status"] == "queued"
    refused = loop.submit("reset", command_id="overflow")
    assert refused["status"] == "refused" and not refused["accepted"]


def test_failing_action_is_refused_and_the_loop_keeps_running():
    loop, _, _ = make_loop()
    loop.submit("add_glasses", {"count": 999}, command_id="a")
    loop.step()
    assert loop.result("a")["status"] == "done" and not loop.result("a")["accepted"]
    loop.submit("add_glasses", {"count": 2}, command_id="b")
    loop.step()
    assert loop.result("b")["accepted"]
    assert len(loop.status()["world"]["targets"]) == 4


def test_unknown_or_unregistered_commands_are_rejected():
    loop, _, _ = make_loop()
    with pytest.raises(ValueError):
        loop.submit("move_robot")
    bare = ControlLoop(loop.supervisor)
    with pytest.raises(ValueError):
        bare.submit("add_glasses", {"count": 1})


def test_loop_status_reports_a_stall():
    loop, _, clock = make_loop()
    assert loop.status()["loop"]["alive"]
    clock.advance(system_web.LOOP_STALL + 0.1)
    assert not loop.status()["loop"]["alive"]


def test_events_are_published_in_order_and_gaps_are_flagged():
    hub = system_web.EventHub(size=3)
    hub.publish([{"sequence": i} for i in range(1, 6)])
    events, gap = hub.since(0, 0)
    assert [e["sequence"] for e in events] == [3, 4, 5] and gap
    events, gap = hub.since(4, 0)
    assert [e["sequence"] for e in events] == [5] and not gap


def test_frame_hub_keeps_only_the_latest_frame():
    frames = FrameHub()
    frames.publish(1, 0.0, b"a")
    frames.publish(2, 0.1, b"b")
    assert frames.newer(0, 0) == (2, 0.1, b"b")
    assert frames.newer(2, 0) is None


# --- HTTP (real threads, loopback, simulated devices) ---------------------------

@pytest.fixture
def panel():
    loop, devices, _ = make_loop(count=2, clock=time.monotonic, operation_time=0.0)
    frames = FrameHub()
    loop.period = 0.002  # s, faster than TICK_PERIOD so the test batch is quick
    server = serve(loop, "127.0.0.1", 0, frames)
    owner = threading.Thread(target=loop.run, daemon=True)
    owner.start()
    threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()
    yield loop, devices, server, frames
    loop.shutdown()
    owner.join(5)
    frames.close()
    server.shutdown()
    server.server_close()


def request(server, method, path, body=None, headers=None):
    connection = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
    data = None if body is None else json.dumps(body)
    base = {"Content-Type": "application/json"} if method == "POST" else {}
    connection.request(method, path, data, {**base, **(headers or {})})
    response = connection.getresponse()
    payload = response.read()
    connection.close()
    return response, payload


def wait_command(server, command_id):
    for _ in range(100):
        response, payload = request(server, "GET", f"/api/commands/{command_id}")
        record = json.loads(payload)
        if record["status"] != "queued":
            return record
        time.sleep(0.02)
    pytest.fail("command not handled")


def test_panel_page_and_status_are_served_with_security_headers(panel):
    _, _, server, _ = panel
    response, body = request(server, "GET", "/")
    assert response.status == 200 and b"<title>" in body
    assert "script-src 'self'" in response.getheader("Content-Security-Policy")
    response, body = request(server, "GET", "/api/status")
    status = json.loads(body)
    assert status["state"] == "READY" and status["loop"]["alive"] and status["camera"]["available"]
    assert request(server, "GET", "/../system_web.py")[0].status == 404


@pytest.mark.parametrize("headers,code", [
    ({"Origin": "http://evil.example"}, 403),
    ({"Content-Type": "text/plain"}, 415),
])
def test_commands_need_same_origin_and_json(panel, headers, code):
    loop, devices, server, _ = panel
    response, _ = request(server, "POST", "/api/batch/start", {"request_id": "x"}, headers)
    assert response.status == code
    assert loop.supervisor.state == State.READY and not devices.history


def test_start_over_http_runs_a_batch_and_is_idempotent(panel):
    loop, devices, server, _ = panel
    port = server.server_address[1]
    response, body = request(server, "POST", "/api/batch/start", {"request_id": "web-1"},
                             {"Origin": f"http://127.0.0.1:{port}"})
    assert response.status == 202
    assert json.loads(body)["id"] == "web-1"
    record = wait_command(server, "web-1")
    assert record["accepted"], record
    request(server, "POST", "/api/batch/start", {"request_id": "web-1"})
    for _ in range(500):
        if loop.status()["state"] == "COMPLETED":
            break
        time.sleep(0.01)
    assert loop.status()["counts"]["COMPLETED"] == 2
    assert len(devices.placements) == 2  # the retried START did not start a second batch


@pytest.mark.parametrize("path,body", [
    ("/api/batch/start", {}),
    ("/api/batch/start", {"request_id": "x" * 101}),
    ("/api/output/confirm-cleared", {"slot_ids": "t0"}),
    ("/api/sim/add-glasses", {"count": "3"}),
    ("/api/nope", {}),
])
def test_bad_command_bodies_are_rejected(panel, path, body):
    _, _, server, _ = panel
    assert request(server, "POST", path, body)[0].status in (400, 404)


def test_stop_and_clear_over_http(panel):
    loop, _, server, _ = panel
    response, body = request(server, "POST", "/api/stop", {})
    assert response.status == 202
    assert wait_command(server, json.loads(body)["id"])["accepted"]
    for _ in range(200):
        if loop.status()["state"] == "STOPPED":
            break
        time.sleep(0.01)
    assert loop.status()["state"] == "STOPPED"
    _, body = request(server, "POST", "/api/output/confirm-cleared", {"slot_ids": ["t0"]})
    assert wait_command(server, json.loads(body)["id"])["accepted"]


def test_event_stream_sends_status(panel):
    _, _, server, _ = panel
    connection = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
    connection.request("GET", "/api/events")
    response = connection.getresponse()
    assert response.getheader("Content-Type") == "text/event-stream"
    assert response.readline() == b"event: status\n"
    status = json.loads(response.readline().removeprefix(b"data: "))
    assert status["state"] == "READY"
    connection.close()


def test_mjpeg_stream_sends_the_latest_frame(panel):
    _, _, server, frames = panel
    frames.publish(1, time.monotonic(), b"\xff\xd8jpeg")
    connection = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
    connection.request("GET", "/camera.mjpg")
    response = connection.getresponse()
    assert response.getheader("Content-Type").startswith("multipart/x-mixed-replace")
    assert response.readline() == b"--frame\r\n"
    connection.close()


def test_second_server_on_the_same_port_fails(panel):
    loop, _, server, _ = panel
    with pytest.raises(OSError):
        serve(loop, "127.0.0.1", server.server_address[1])


def test_shutdown_stops_a_running_batch_before_the_loop_ends():
    # The simulator acknowledges a stop after operation_time, within STOP_TIMEOUT.
    loop, devices, _ = make_loop(count=1, clock=time.monotonic, operation_time=0.3)
    owner = threading.Thread(target=loop.run, daemon=True)
    owner.start()
    loop.submit("start", command_id="s")
    for _ in range(200):
        if devices.history:
            break
        time.sleep(0.01)
    loop.shutdown()
    owner.join(5)
    assert not owner.is_alive()
    assert loop.supervisor.state == State.STOPPED and devices.stop_requests
    assert loop.submit("reset")["status"] == "refused"


def test_shutdown_with_unconfirmed_stop_ends_in_fault():
    clock = SimulationClock()
    loop, devices, _ = make_loop(count=1, clock=clock, operation_time=60.0)
    loop.submit("start", command_id="s")
    step_until(loop, clock, lambda: bool(devices.history))
    loop.period = 0.0  # _finish_stop sleeps period per tick; virtual time advances below
    original_tick = loop.supervisor.tick

    def tick_and_advance():
        clock.advance(0.1)
        original_tick()

    loop.supervisor.tick = tick_and_advance
    loop.wall_clock = clock
    loop._finish_stop("test shutdown")
    assert loop.supervisor.state == State.FAULT
    assert "not confirmed" in loop.supervisor.fault


def test_web_cli_options_are_checked(capsys):
    for options in (["--web", "--fast"], ["--web", "--stop-at", "WIPE"], ["--camera"]):
        with pytest.raises(SystemExit) as error:
            main(["--simulate", *options])
        assert error.value.code == 2
    capsys.readouterr()


def test_web_cli_reports_a_port_in_use(capsys):
    blocker = socket.socket()
    blocker.bind(("127.0.0.1", 0))
    blocker.listen()
    try:
        assert main(["--simulate", "--web", "--port", str(blocker.getsockname()[1])]) == 1
    finally:
        blocker.close()
    assert "supervisor runner" in capsys.readouterr().err


def test_service_runs_before_every_tick_and_while_stopping_at_shutdown():
    loop, devices, clock = make_loop(count=1)
    calls = []
    loop.service = calls.append
    loop.submit("start", command_id="s")
    step_until(loop, clock, lambda: bool(devices.history))
    assert len(calls) == loop._ticks
    loop.period = 0.0
    before = len(calls)
    loop.service = lambda now: (calls.append(now), clock.advance(TICK_PERIOD))  # virtual time moves
    loop._finish_stop("test")
    assert len(calls) > before and loop.supervisor.state == State.STOPPED


def test_extra_status_and_actions_are_published():
    loop, _, _ = make_loop()
    loop.extra_status = lambda: {"teach": {"poses": {}}}
    loop.step()
    status = loop.status()
    assert status["teach"] == {"poses": {}} and status["actions"] == ["add_glasses"]


@pytest.mark.parametrize("path,body,code", [
    ("/api/teach/capture", {"name": "observe"}, 404),     # no --teach
])
def test_teach_routes_need_teach_mode(panel, path, body, code):
    _, _, server, _ = panel
    assert request(server, "POST", path, body)[0].status == code


def test_teach_routes_validate_bodies(panel):
    loop, _, server, _ = panel
    loop.actions.update({"teach_capture": lambda p: Reply(True, p["name"]),
                         "teach_freedrive": lambda p: Reply(True, "fd"),
                         "teach_gripper": lambda p: Reply(True, "g")})
    for path, body in (("/api/teach/capture", {}), ("/api/teach/freedrive", {"on": "yes"}),
                       ("/api/teach/gripper", {"action": "blow"})):
        assert request(server, "POST", path, body)[0].status == 400
    response, body = request(server, "POST", "/api/teach/capture", {"name": "observe"})
    assert response.status == 202
    assert wait_command(server, json.loads(body)["id"])["message"] == "observe"


@pytest.mark.parametrize("options", [
    ["--hardware"],                                  # needs --web
    ["--hardware", "--web"],                         # needs --camera or --teach
    ["--hardware", "--web", "--teach", "--fail-at", "PICK"],
    ["--simulate", "--web", "--teach"],              # teach is hardware only
    ["--simulate", "--hardware", "--web"],           # exclusive modes
])
def test_hardware_cli_options_are_checked_before_connecting(options, capsys):
    with pytest.raises(SystemExit) as error:
        main(options)
    assert error.value.code == 2
    capsys.readouterr()
