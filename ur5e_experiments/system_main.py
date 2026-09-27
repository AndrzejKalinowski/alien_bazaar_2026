"""Run the supervisor: simulated devices (CLI or web panel) or the real cell.

From the repository root:
    python ur5e_experiments/system_main.py --simulate --fast
    python ur5e_experiments/system_main.py --simulate --glasses 3 --slots 1
    python ur5e_experiments/system_main.py --simulate --fast --fail-at SPONGE
    python ur5e_experiments/system_main.py --simulate --fast --stop-at WIPE
    python ur5e_experiments/system_main.py --simulate --web
    python ur5e_experiments/system_main.py --simulate --web --camera
    python ur5e_experiments/system_main.py --hardware --web --teach
    python ur5e_experiments/system_main.py --hardware --web --camera

--web serves the panel (system_web.py) and waits for START from the browser;
it prints the panel link with the control token. --host 0.0.0.0 opens it to
the LAN (plain HTTP, token-protected commands). --step-time slows the fake
operations so the cycle can be followed. The panel can add fictional glasses.
--camera takes the targets from the overhead camera with the classic detector
(system_vision.py, needs OpenCV and the overhead calibration); the robot,
gripper and stations stay simulated. Ctrl+C stops the devices, then exits.

Exactly one of --simulate / --hardware is required; nothing connects by default.
--hardware (system_hardware.py, NOT YET TRIED ON THE ROBOT) connects the UR5e
through SafeControl, the gripper and the servo bus, and needs --web. The arm
can move once it has started. It needs --camera for batches; --teach adds
freedrive, manual grip/release and pose capture to the panel (outside
batches), to fill system_teach.json. --gamepad: any stick input STOPs a
batch; when idle the sticks jog the arm (gamepad_jog.py). Output occupancy
persists in system_outputs.json: a restart never frees a place, and after an
unclean exit every free place comes back UNKNOWN (confirm it in the panel).
START stays refused while the teach file or layout is incomplete, the TCP
offset changed, freedrive is on or the arm is not at the observe pose.
One --hardware process at a time (supervisor.lock). Events always go to
logs/supervisor-<date>.jsonl unless --journal names another new file.
--fast advances a virtual monotonic clock.
Without it the same fake sequence runs in wall time; Ctrl+C requests STOP.
--auto-clear-output simulates an operator emptying the output between glasses.
--journal PATH writes JSON Lines to a NEW file; existing files are not replaced.
--quiet suppresses event output, but still prints the final JSON status.

Exit codes: 0 complete (all glasses done), 1 runtime/logging error, 2 fault or
invalid arguments, 3 output full, 4 stopped, 5 complete with skipped targets.
With --web: 0 after Ctrl+C with confirmed stopped devices, 1 start-up error
(e.g. port in use: another supervisor is running), 2 fault or loop failure.
Paths are checked against fictional station bounds and a tool/glass envelope;
these example dimensions are not a calibration for real hardware.
Requires: Python 3.12+ standard library only (--camera: OpenCV;
--hardware: ur_rtde, numpy, pyserial).
"""

import argparse
from contextlib import ExitStack
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import threading
from time import monotonic, sleep

from system_batch import OutputSlot, OutputStore
from system_controller import Supervisor
from system_model import GlassTarget, Reply, SlotState, State, Step
from system_settings import SIM_OPERATION_TIME, STOP_TIMEOUT, TICK_PERIOD
from system_simulator import SimulatedDevices, SimulationClock
import system_web


def positive_count(value):
    value = int(value)
    if not 1 <= value <= 1000:
        raise argparse.ArgumentTypeError("count must be between 1 and 1000")
    return value


def step_time(value):
    value = float(value)
    if not 0 <= value <= 10:
        raise argparse.ArgumentTypeError("step time must be 0..10 s")
    return value


def port_number(value):
    value = int(value)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError("port must be 1..65535")
    return value


HERE = os.path.dirname(os.path.abspath(__file__))
LOCK_FILE = os.path.join(HERE, "supervisor.lock")
LOG_DIR = os.path.join(HERE, "logs")


def open_journal(stack, path):
    return stack.enter_context(path.open("x", encoding="utf-8")) if path else None


def instance_lock(path=None):
    """Held for the process lifetime; the OS drops it if the process dies."""
    path = path or LOCK_FILE
    handle = open(path, "a+")
    try:
        if os.name == "nt":
            import msvcrt
            handle.seek(0)
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(f"another hardware supervisor is running ({path} is locked)") from None
    return handle


def default_journal():
    os.makedirs(LOG_DIR, exist_ok=True)
    return Path(LOG_DIR) / f"supervisor-{datetime.now():%Y%m%d-%H%M%S}.jsonl"


def run_web(args, targets, slots):
    frames = vision = store = None
    try:
        with ExitStack() as stack:
            if args.hardware:
                stack.callback(instance_lock().close)
                store = OutputStore()
                slots = store.load_slots([slot.id for slot in slots])
                if store.warning:
                    print(f"WARNING: {store.warning}", file=sys.stderr, flush=True)
                args.journal = args.journal or default_journal()
            journal = open_journal(stack, args.journal)
            if args.camera:
                from system_vision import CameraVision  # OpenCV only in camera mode
                frames = system_web.FrameHub()
                vision = CameraVision(frames)
                vision.start()
                stack.callback(vision.close)
                stack.callback(frames.close)
            def write_events(events):
                for event in events:
                    journal.write(json.dumps(event, ensure_ascii=True) + "\n")
                if events:
                    journal.flush()

            sinks = [write_events] if journal else []
            if args.hardware:
                import system_hardware  # ur_rtde, numpy, pyserial only in this mode
                devices = system_hardware.connect([slot.id for slot in slots], vision)
                stack.callback(devices.close)
                supervisor = Supervisor(devices, slots)
                override = None

                def service(now):
                    devices.service(now)
                    if override is not None:
                        override.update(now)

                loop = system_web.ControlLoop(
                    supervisor, sinks=[*sinks, store.sink(supervisor)], service=service,
                    extra_status=devices.panel_status,
                    actions=system_hardware.teach_actions(devices, supervisor) if args.teach else {})
                if args.gamepad:
                    from gamepad_jog import GamepadControl
                    override = system_hardware.GamepadOverride(
                        devices, GamepadControl(), lambda: supervisor.state, loop.stop)
                    stack.callback(override.close)
            else:
                devices = SimulatedDevices([] if args.camera else targets, operation_time=args.step_time,
                                           fail_at=Step(args.fail_at) if args.fail_at else None,
                                           scene_source=vision)
                supervisor = Supervisor(devices, slots)

                def add_glasses(payload):
                    return Reply(True, "added " + ", ".join(devices.add_targets(payload.get("count"))))

                loop = system_web.ControlLoop(
                    supervisor, actions={} if args.camera else {"add_glasses": add_glasses},
                    world=devices.snapshot, sinks=sinks)
            server = system_web.serve(loop, args.host, args.port, args.token, frames)
            stack.callback(server.server_close)
            if args.hardware:
                devices.arm()  # watchdog: the owner loop must start right now
            owner = threading.Thread(target=loop.run, name="supervisor-owner")
            owner.start()
            threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
            host = "127.0.0.1" if args.host in ("0.0.0.0", "") else args.host
            print(f"Panel: http://{host}:{server.server_address[1]}/#token={server.token}", flush=True)
            if args.host not in ("127.0.0.1", "localhost", "::1"):
                print("WARNING: panel reachable from the network over plain HTTP; "
                      "only the token protects the controls.", file=sys.stderr, flush=True)
            try:
                while owner.is_alive():
                    owner.join(0.2)
            except KeyboardInterrupt:
                print("Stopping devices...", file=sys.stderr, flush=True)
            finally:
                loop.shutdown()
                owner.join()
                server.shutdown()
                if store is not None:
                    store.close(supervisor, stopped=not loop.error and supervisor.state not in (
                        State.RUNNING, State.WAITING_OUTPUT, State.STOPPING))
            print(json.dumps(loop.status(), ensure_ascii=True))
            if loop.error or supervisor.state == State.FAULT:
                print(loop.error or supervisor.fault, file=sys.stderr)
                return 2
            return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"supervisor runner: {exc}", file=sys.stderr)
        return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true", help="in-memory robot, gripper and servos")
    mode.add_argument("--hardware", action="store_true",
                      help="the real cell (needs --web; the arm can move)")
    parser.add_argument("--fast", action="store_true", help="run with virtual time")
    parser.add_argument("--glasses", type=positive_count, default=3)
    parser.add_argument("--slots", type=positive_count, default=3)
    parser.add_argument("--fail-at", choices=[step.value for step in Step])
    parser.add_argument("--stop-at", choices=[step.value for step in Step])
    parser.add_argument("--auto-clear-output", action="store_true")
    parser.add_argument("--journal", type=Path)
    parser.add_argument("--quiet", action="store_true")
    web = parser.add_argument_group("web panel")
    web.add_argument("--web", action="store_true", help="serve the panel and wait for START")
    web.add_argument("--host", default=system_web.DEFAULT_HOST)
    web.add_argument("--port", type=port_number, default=system_web.DEFAULT_PORT)
    web.add_argument("--token", help="control token (default: random per run)")
    web.add_argument("--camera", action="store_true", help="targets from the overhead camera")
    web.add_argument("--step-time", type=step_time, default=SIM_OPERATION_TIME,
                     help="s per simulated operation")
    web.add_argument("--teach", action="store_true", help="hardware: freedrive and pose capture in the panel")
    web.add_argument("--gamepad", action="store_true", help="hardware: sticks STOP a batch, jog when idle")
    args = parser.parse_args(argv)
    if args.hardware:
        if not args.web:
            parser.error("--hardware needs --web (STOP and status come from the panel)")
        if not (args.camera or args.teach):
            parser.error("--hardware needs --camera for batches (or --teach to teach poses)")
        if args.fail_at or args.step_time != SIM_OPERATION_TIME:
            parser.error("--fail-at and --step-time are simulation options")
    elif args.teach or args.gamepad:
        parser.error("--teach and --gamepad need --hardware")
    if args.web:
        if args.fast or args.stop_at or args.auto_clear_output:
            parser.error("--fast, --stop-at and --auto-clear-output are CLI-only; use the panel")
        if args.token is not None and not system_web.valid_id(args.token):
            parser.error("--token must be 1..100 printable characters")
    elif args.camera or args.token is not None:
        parser.error("--camera and --token need --web")

    clock = SimulationClock() if args.fast else monotonic
    targets = [GlassTarget(f"glass-{i + 1}", 0.1 + i * 0.1, -0.4) for i in range(args.glasses)]
    # Hardware: replaced by the persisted occupancy (OutputStore) in run_web.
    slots = [OutputSlot(f"tag-{i + 1}", SlotState.UNKNOWN if args.hardware else SlotState.FREE)
             for i in range(args.slots)]
    if args.web:
        return run_web(args, targets, slots)
    devices = SimulatedDevices(targets, operation_time=args.step_time,
                               fail_at=Step(args.fail_at) if args.fail_at else None)
    supervisor = Supervisor(devices, slots, clock)
    stop_sent = False

    def advance():
        if args.fast:
            clock.advance(TICK_PERIOD)
        else:
            sleep(TICK_PERIOD)

    def finish_stop():
        supervisor.request_stop("runner shutdown")
        for _ in range(int(STOP_TIMEOUT / TICK_PERIOD) + 2):
            advance()
            supervisor.tick()
            if supervisor.state != State.STOPPING:
                break

    try:
        with ExitStack() as stack:
            journal = open_journal(stack, args.journal)

            def flush_events():
                for event in supervisor.drain_events():
                    line = json.dumps(event, ensure_ascii=True)
                    if not args.quiet:
                        print(line)
                    if journal is not None:
                        journal.write(line + "\n")
                if journal is not None:
                    journal.flush()

            reply = supervisor.start("cli-start")
            if not reply.accepted:
                print(reply.message, file=sys.stderr)
                return 2
            try:
                while supervisor.state in (State.RUNNING, State.WAITING_OUTPUT, State.STOPPING):
                    supervisor.tick()
                    if (not stop_sent and args.stop_at and supervisor.command is not None
                            and supervisor.command.step.value == args.stop_at):
                        supervisor.request_stop("simulated operator STOP")
                        stop_sent = True
                    flush_events()
                    if supervisor.state == State.WAITING_OUTPUT:
                        if args.auto_clear_output:
                            supervisor.confirm_output_cleared(list(supervisor.outputs.slots))
                        else:
                            break
                    advance()
            except KeyboardInterrupt:
                finish_stop()
            flush_events()
            status = supervisor.status()
            print(json.dumps(status, ensure_ascii=True))
            if supervisor.state == State.COMPLETED:
                return 5 if status["counts"]["SKIPPED"] else 0
            return {State.FAULT: 2, State.WAITING_OUTPUT: 3, State.STOPPED: 4}.get(supervisor.state, 1)
    except (OSError, ValueError) as exc:
        print(f"supervisor runner: {exc}", file=sys.stderr)
        return 1
    finally:
        if supervisor.state in (State.RUNNING, State.STOPPING):
            finish_stop()


if __name__ == "__main__":
    raise SystemExit(main())
