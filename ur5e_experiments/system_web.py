"""Web panel for the supervisor: one control-loop owner and an HTTP front end.

ControlLoop is the ONLY caller of the Supervisor. It runs in one thread and,
once per tick: handles a pending STOP first, then at most one queued operator
command, then supervisor.tick(), then publishes events and a status snapshot.
HTTP handler threads never touch the supervisor or the devices. They enqueue
commands (bounded queue, refused when full) and read published snapshots.
A slow or stalled browser only blocks its own handler thread.

STOP has a separate priority flag. It cancels every command queued before it
(also a START still waiting in the queue), then calls request_stop(). An
accepted HTTP request (202) is not an executed command: GET /api/commands/{id}
reports "queued", "done" (with the supervisor's accepted/message reply) or
"cancelled". START is idempotent by its request ID, like Supervisor.start().

Endpoints (the plan's WebSocket is Server-Sent Events here; the panel only
needs server -> browser updates and commands already use HTTP):
    GET  /                          panel (web/index.html, app.js, style.css)
    GET  /api/status                full status snapshot, also after a reload
    GET  /api/events                SSE: supervisor events + status every 0.5 s
    GET  /api/commands/{id}         result of a queued command
    GET  /camera.mjpg               latest annotated camera frame (camera mode)
    POST /api/batch/start           {"request_id": "..."}
    POST /api/stop                  {"reason": "..."} (optional)
    POST /api/fault/reset           {}
    POST /api/output/confirm-cleared  {"slot_ids": ["tag-1", ...]}
    POST /api/sim/add-glasses       {"count": 3} (simulation only)
    POST /api/teach/capture         {"name": "sprayer.work"}   (--teach only)
    POST /api/teach/freedrive       {"on": true}                (--teach only)
    POST /api/teach/gripper         {"action": "grip"|"release"} (--teach only)

Access: GET is open (viewing needs no control rights). POST needs the session
token in the X-Supervisor-Token header, a JSON body and, if the browser sends
an Origin header, the same origin as the Host. The default bind is loopback;
binding to the LAN is an explicit option. Plain HTTP: the token is not secret
from someone who can sniff the network. The listening socket is exclusive, so
a second supervisor on the same port fails to start instead of sharing it.

Closing the browser does not stop a batch; reopening it restores the state
from /api/status. Stopping the process (Ctrl+C) stops the devices first.
Threads, not processes, for now: the camera adapter runs in its own thread.
Requires: Python 3.12+ standard library (camera mode: see system_vision.py).
"""

from collections import OrderedDict, deque
import hmac
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import secrets
import socket
import threading
from time import monotonic, sleep
from urllib.parse import urlsplit
from uuid import uuid4

from system_model import Reply, State
from system_operations import RECIPE
from system_settings import STOP_TIMEOUT, TICK_PERIOD

# --- control loop ----------------------------------------------------------------
COMMAND_QUEUE_SIZE = 16        # operator commands waiting for the owner thread
COMMAND_RESULTS_KEPT = 256     # results kept for GET /api/commands/{id}
EVENT_BUFFER = 1000            # events kept for (re)connecting SSE clients
LOOP_STALL = 1.0               # s without a tick -> status reports the loop stalled

# --- web server ------------------------------------------------------------------
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
MAX_BODY = 4096                # bytes, JSON command body
MAX_STREAM_CLIENTS = 8         # concurrent SSE + MJPEG connections
STATUS_PERIOD = 0.5            # s, SSE status push
CLIENT_TIMEOUT = 10.0          # s, a client that does not take data is dropped
MAX_ID_LENGTH = 100            # characters, request/command IDs
WEB_DIR = Path(__file__).resolve().parent / "web"
STATIC_FILES = {"/": ("index.html", "text/html; charset=utf-8"),
                "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                "/style.css": ("style.css", "text/css; charset=utf-8")}
CONTENT_SECURITY_POLICY = ("default-src 'self'; img-src 'self'; style-src 'self'; "
                           "script-src 'self'; connect-src 'self'; frame-ancestors 'none'")

BUILTIN_COMMANDS = ("start", "reset", "confirm_cleared")


class EventHub:
    """Bounded event history; readers wait without holding up the publisher."""

    def __init__(self, size=EVENT_BUFFER):
        self._condition = threading.Condition()
        self._events = deque(maxlen=size)
        self._last = 0
        self.closed = False

    def publish(self, events):
        if not events:
            return
        with self._condition:
            self._events.extend(events)
            self._last = events[-1]["sequence"]
            self._condition.notify_all()

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()

    @property
    def last(self):
        with self._condition:
            return self._last

    def since(self, sequence, timeout):
        """Events after `sequence`; gap=True when older ones were already dropped."""
        with self._condition:
            self._condition.wait_for(lambda: self._last > sequence or self.closed, timeout)
            events = [e for e in self._events if e["sequence"] > sequence]
            gap = bool(events) and events[0]["sequence"] > sequence + 1
            return events, gap


class FrameHub:
    """Latest JPEG only: a slow viewer skips frames instead of queueing video."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None      # (sequence, monotonic time, jpeg bytes)
        self.closed = False

    def publish(self, sequence, captured_at, jpeg):
        with self._condition:
            self._frame = (sequence, captured_at, jpeg)
            self._condition.notify_all()

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()

    def latest(self):
        with self._condition:
            return self._frame

    def newer(self, sequence, timeout):
        with self._condition:
            self._condition.wait_for(
                lambda: self.closed or (self._frame is not None and self._frame[0] > sequence), timeout)
            return self._frame if self._frame is not None and self._frame[0] > sequence else None


class ControlLoop:
    """Owner of the supervisor. step() is one tick; run() repeats it in a thread.

    actions: extra owner-thread commands, e.g. {"add_glasses": callable(payload)
    -> Reply}. world: callable returning a JSON-ready dict for the panel map.
    sinks: callables that get each list of drained events (journal writer).
    service: callable(now) run in every step BEFORE supervisor.tick(), also
    while stopping at shutdown (hardware: RTDE watchdog kick, ceiling check).
    extra_status: callable returning a dict merged into the published status.
    """

    def __init__(self, supervisor, period=TICK_PERIOD, actions=None, world=None, sinks=(),
                 wall_clock=monotonic, service=None, extra_status=None):
        self.supervisor = supervisor
        self.period = period
        self.actions = dict(actions or {})
        self.world = world
        self.sinks = tuple(sinks)
        self.service = service
        self.extra_status = extra_status
        self.wall_clock = wall_clock
        self.events = EventHub()
        self.error = ""
        self._queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
        self._lock = threading.Lock()
        self._results = OrderedDict()
        self._stop_ids = []
        self._stop_reason = ""
        self._stop_flag = threading.Event()
        self._shutdown = threading.Event()
        self._ticks = 0
        self._heartbeat = wall_clock()
        self._max_tick = 0.0
        self._status = {}
        self._publish()

    # --- called from any thread ---------------------------------------------------

    def _record(self, command_id, kind, status, **details):
        """Caller holds self._lock."""
        self._results[command_id] = {"id": command_id, "kind": kind, "status": status, **details}
        self._results.move_to_end(command_id)
        while len(self._results) > COMMAND_RESULTS_KEPT:
            oldest = next(iter(self._results))
            if self._results[oldest]["status"] == "queued":
                break  # never forget a command the owner still has to answer
            self._results.popitem(last=False)

    def submit(self, kind, payload=None, command_id=None):
        """Queue an operator command; returns its result record (a copy)."""
        if kind not in BUILTIN_COMMANDS and kind not in self.actions:
            raise ValueError(f"unknown command {kind!r}")
        command_id = command_id or uuid4().hex
        with self._lock:
            if command_id in self._results:
                return dict(self._results[command_id])  # a retried POST is not a new command
            if self._shutdown.is_set():
                self._record(command_id, kind, "refused", accepted=False, message="supervisor is shutting down")
                return dict(self._results[command_id])
            try:
                self._queue.put_nowait((command_id, kind, payload or {}))
            except queue.Full:
                return {"id": command_id, "kind": kind, "status": "refused", "accepted": False,
                        "message": "command queue full, retry"}
            self._record(command_id, kind, "queued")
            return dict(self._results[command_id])

    def stop(self, reason="operator STOP"):
        """Priority STOP; handled before any queued command on the next tick."""
        command_id = uuid4().hex
        with self._lock:
            self._stop_ids.append(command_id)
            self._stop_reason = reason
            self._record(command_id, "stop", "queued")
            self._stop_flag.set()
            return dict(self._results[command_id])

    def result(self, command_id):
        with self._lock:
            record = self._results.get(command_id)
            return dict(record) if record else None

    def status(self):
        with self._lock:
            status = dict(self._status)
        age = self.wall_clock() - status["loop"]["heartbeat"]
        status["loop"] = {**status["loop"], "age": age,
                          "alive": not self.error and age < LOOP_STALL and not self._shutdown.is_set()}
        return status

    def shutdown(self):
        self._shutdown.set()

    # --- owner thread only --------------------------------------------------------

    def _finish(self, command_id, kind, reply):
        with self._lock:
            self._record(command_id, kind, "done", accepted=reply.accepted, message=reply.message,
                         batch_id=reply.batch_id)

    def _execute(self, command_id, kind, payload):
        sup = self.supervisor
        if kind == "start":
            return sup.start(command_id)
        if kind == "reset":
            return sup.reset_fault()
        if kind == "confirm_cleared":
            return sup.confirm_output_cleared(payload.get("slot_ids") or ())
        return self.actions[kind](payload)

    def _cancel_queued(self, message):
        while True:
            try:
                command_id, kind, _ = self._queue.get_nowait()
            except queue.Empty:
                return
            with self._lock:
                self._record(command_id, kind, "cancelled", accepted=False, message=message)

    def step(self):
        started = self.wall_clock()
        if self._stop_flag.is_set():
            with self._lock:
                self._stop_flag.clear()
                stop_ids, self._stop_ids = self._stop_ids, []
                reason = self._stop_reason
            self._cancel_queued("cancelled by STOP")
            reply = self.supervisor.request_stop(reason)
            for command_id in stop_ids:
                self._finish(command_id, "stop", reply)
        else:
            try:
                command_id, kind, payload = self._queue.get_nowait()
            except queue.Empty:
                pass
            else:
                try:
                    reply = self._execute(command_id, kind, payload)
                except Exception as exc:  # refuse, never let an operator command end the loop
                    reply = Reply(False, f"{kind} refused: {exc}")
                self._finish(command_id, kind, reply)
        self._service()
        self.supervisor.tick()
        self._ticks += 1
        self._max_tick = max(self._max_tick, self.wall_clock() - started)
        self._publish()

    def _service(self):
        if self.service is not None:
            self.service(self.wall_clock())

    def _publish(self):
        events = self.supervisor.drain_events()
        for sink in self.sinks:
            sink(events)
        self.events.publish(events)
        status = self.supervisor.status()
        status["event_sequence"] = self.events.last
        status["recipe"] = [spec.step.value for spec in RECIPE]
        status["actions"] = sorted(self.actions)
        status["loop"] = {"heartbeat": self.wall_clock(), "ticks": self._ticks,
                          "max_tick": self._max_tick, "error": self.error,
                          "pending_commands": self._queue.qsize()}
        status["world"] = self.world() if self.world else None
        if self.extra_status is not None:
            status.update(self.extra_status())
        with self._lock:
            self._status = status

    def _finish_stop(self, reason):
        """Stop devices before the process exits; bounded by STOP_TIMEOUT."""
        sup = self.supervisor
        if sup.state not in (State.RUNNING, State.WAITING_OUTPUT, State.STOPPING):
            return
        if sup.state != State.STOPPING:
            sup.request_stop(reason)
        deadline = self.wall_clock() + STOP_TIMEOUT + 4 * self.period
        while sup.state == State.STOPPING and self.wall_clock() < deadline:
            sleep(self.period)
            self._service()
            sup.tick()
        self._publish()

    def run(self):
        try:
            while not self._shutdown.is_set():
                started = self.wall_clock()
                self.step()
                sleep(max(0.0, self.period - (self.wall_clock() - started)))
        except BaseException as exc:
            self.error = f"control loop failed: {exc!r}"
            raise
        finally:
            self._shutdown.set()
            self._cancel_queued("supervisor is shutting down")
            try:
                self._finish_stop(self.error or "supervisor shutdown")
            finally:
                self._publish()
                self.events.close()


# --- HTTP ------------------------------------------------------------------------

class PanelServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False   # a second instance must fail, not share the port

    def __init__(self, address, loop, token, frames=None, verbose=False):
        self.loop = loop
        self.token = token
        self.frames = frames
        self.verbose = verbose
        self.streams = threading.BoundedSemaphore(MAX_STREAM_CLIENTS)
        super().__init__(address, PanelHandler)

    def server_bind(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):  # Windows: stop another socket binding the same port
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class RequestError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def valid_id(value):
    return isinstance(value, str) and 0 < len(value) <= MAX_ID_LENGTH and value.isprintable()


class PanelHandler(BaseHTTPRequestHandler):
    server: PanelServer
    timeout = CLIENT_TIMEOUT
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        if self.server.verbose:
            super().log_message(format, *args)

    def _headers(self, status, content_type, length=None, extra=()):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", CONTENT_SECURITY_POLICY)
        if length is not None:
            self.send_header("Content-Length", str(length))
        for name, value in extra:
            self.send_header(name, value)
        self.end_headers()

    def _json(self, status, data):
        body = json.dumps(data, ensure_ascii=True).encode("ascii")
        self._headers(status, "application/json", len(body))
        self.wfile.write(body)

    def _error(self, status, message):
        self._json(status, {"error": message})

    # --- GET ----------------------------------------------------------------------

    def do_GET(self):
        path = urlsplit(self.path).path
        loop = self.server.loop
        if path in STATIC_FILES:
            name, content_type = STATIC_FILES[path]
            body = (WEB_DIR / name).read_bytes()
            self._headers(HTTPStatus.OK, content_type, len(body))
            self.wfile.write(body)
        elif path == "/api/status":
            status = loop.status()
            status["camera"] = self._camera_status()
            self._json(HTTPStatus.OK, status)
        elif path.startswith("/api/commands/"):
            record = loop.result(path.removeprefix("/api/commands/"))
            if record is None:
                self._error(HTTPStatus.NOT_FOUND, "unknown or expired command ID")
            else:
                self._json(HTTPStatus.OK, record)
        elif path == "/api/events":
            self._stream(self._events)
        elif path == "/camera.mjpg":
            if self.server.frames is None:
                self._error(HTTPStatus.NOT_FOUND, "no camera in this mode")
            else:
                self._stream(self._mjpeg)
        else:
            self._error(HTTPStatus.NOT_FOUND, "not found")

    def _camera_status(self):
        frames = self.server.frames
        if frames is None:
            return {"available": False}
        frame = frames.latest()
        if frame is None:
            return {"available": True, "sequence": None, "age": None}
        return {"available": True, "sequence": frame[0],
                "age": self.server.loop.wall_clock() - frame[1]}

    def _stream(self, writer):
        if not self.server.streams.acquire(blocking=False):
            self._error(HTTPStatus.SERVICE_UNAVAILABLE, "too many open streams")
            return
        try:
            writer()
        except (ConnectionError, TimeoutError, OSError):
            pass  # the browser went away or stopped reading; only this thread is affected
        finally:
            self.server.streams.release()
            self.close_connection = True

    def _events(self):
        self._headers(HTTPStatus.OK, "text/event-stream", extra=(("Connection", "close"),))
        hub, loop = self.server.loop.events, self.server.loop
        sequence = hub.last  # history is in /api/status; stream only new events
        status_due = 0.0
        events, gap = [], False  # first message: the current status, without waiting
        while not hub.closed:
            chunks = []
            if gap:
                chunks.append("event: gap\ndata: {}\n\n")
            for event in events:
                sequence = event["sequence"]
                chunks.append(f"event: supervisor\ndata: {json.dumps(event, ensure_ascii=True)}\n\n")
            if events or loop.wall_clock() >= status_due:
                status = loop.status()
                status["camera"] = self._camera_status()
                chunks.append(f"event: status\ndata: {json.dumps(status, ensure_ascii=True)}\n\n")
                status_due = loop.wall_clock() + STATUS_PERIOD
            self.wfile.write("".join(chunks).encode("ascii"))
            self.wfile.flush()
            events, gap = hub.since(sequence, STATUS_PERIOD)

    def _mjpeg(self):
        self._headers(HTTPStatus.OK, "multipart/x-mixed-replace; boundary=frame",
                      extra=(("Connection", "close"),))
        frames, sequence = self.server.frames, -1
        while not frames.closed:
            frame = frames.newer(sequence, 1.0)
            if frame is None:
                continue
            sequence, _, jpeg = frame
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                             + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            self.wfile.flush()

    # --- POST ---------------------------------------------------------------------

    def _authorize(self):
        token = self.headers.get("X-Supervisor-Token", "")
        if not hmac.compare_digest(token.encode(), self.server.token.encode()):
            raise RequestError(HTTPStatus.FORBIDDEN, "missing or wrong control token")
        origin = self.headers.get("Origin")
        if origin is not None and urlsplit(origin).netloc != self.headers.get("Host", ""):
            raise RequestError(HTTPStatus.FORBIDDEN, "cross-origin request refused")
        if self.headers.get_content_type() != "application/json":
            raise RequestError(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "send application/json")

    def _body(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise RequestError(HTTPStatus.BAD_REQUEST, "bad Content-Length") from None
        if not 0 <= length <= MAX_BODY:
            raise RequestError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body too large")
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(raw)
        except (UnicodeDecodeError, ValueError):
            raise RequestError(HTTPStatus.BAD_REQUEST, "body is not JSON") from None
        if not isinstance(data, dict):
            raise RequestError(HTTPStatus.BAD_REQUEST, "body must be a JSON object")
        return data

    def do_POST(self):
        path = urlsplit(self.path).path
        loop = self.server.loop
        try:
            self._authorize()
            body = self._body()
            if path == "/api/stop":
                reason = body.get("reason", "operator STOP (panel)")
                if not isinstance(reason, str) or len(reason) > 200:
                    raise RequestError(HTTPStatus.BAD_REQUEST, "reason must be a short string")
                record = loop.stop(reason)
            elif path == "/api/batch/start":
                request_id = body.get("request_id")
                if not valid_id(request_id):
                    raise RequestError(HTTPStatus.BAD_REQUEST, "request_id is required")
                record = loop.submit("start", command_id=request_id)
            elif path == "/api/fault/reset":
                record = loop.submit("reset")
            elif path == "/api/output/confirm-cleared":
                slot_ids = body.get("slot_ids")
                if (not isinstance(slot_ids, list) or not slot_ids or len(slot_ids) > 100
                        or not all(valid_id(s) for s in slot_ids)):
                    raise RequestError(HTTPStatus.BAD_REQUEST, "slot_ids must list output slot IDs")
                record = loop.submit("confirm_cleared", {"slot_ids": slot_ids})
            elif path == "/api/sim/add-glasses":
                if "add_glasses" not in loop.actions:
                    raise RequestError(HTTPStatus.NOT_FOUND, "only available with simulated targets")
                count = body.get("count", 1)
                if not isinstance(count, int) or isinstance(count, bool):
                    raise RequestError(HTTPStatus.BAD_REQUEST, "count must be an integer")
                record = loop.submit("add_glasses", {"count": count})
            elif path.startswith("/api/teach/"):
                record = self._teach(path.removeprefix("/api/teach/"), body)
            else:
                raise RequestError(HTTPStatus.NOT_FOUND, "not found")
        except RequestError as exc:
            self._error(exc.status, str(exc))
            return
        status = HTTPStatus.SERVICE_UNAVAILABLE if record["status"] == "refused" else HTTPStatus.ACCEPTED
        self._json(status, record)


    def _teach(self, action, body):
        loop = self.server.loop
        kind = f"teach_{action}"
        if kind not in loop.actions:
            raise RequestError(HTTPStatus.NOT_FOUND, "teaching needs --teach")
        if action == "capture" and not valid_id(body.get("name")):
            raise RequestError(HTTPStatus.BAD_REQUEST, "name is required")
        if action == "freedrive" and not isinstance(body.get("on"), bool):
            raise RequestError(HTTPStatus.BAD_REQUEST, "on must be true or false")
        if action == "gripper" and body.get("action") not in ("grip", "release"):
            raise RequestError(HTTPStatus.BAD_REQUEST, "action must be grip or release")
        return loop.submit(kind, body)


def serve(loop, host=DEFAULT_HOST, port=DEFAULT_PORT, token=None, frames=None, verbose=False):
    """Create the server (binds immediately); call serve_forever() in a thread."""
    return PanelServer((host, port), loop, token or secrets.token_urlsafe(16), frames, verbose)
