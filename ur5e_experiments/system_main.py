"""Run the first supervisor milestone: a complete batch with simulated devices.

From the repository root:
    python ur5e_experiments/system_main.py --simulate --fast
    python ur5e_experiments/system_main.py --simulate --glasses 3 --slots 1
    python ur5e_experiments/system_main.py --simulate --fast --fail-at SPONGE
    python ur5e_experiments/system_main.py --simulate --fast --stop-at WIPE

--simulate is required. There is no hardware mode and no implicit connection
to robot, camera or serial ports. --fast advances a virtual monotonic clock.
Without it the same fake sequence runs in wall time; Ctrl+C requests STOP.
--auto-clear-output simulates an operator emptying the output between glasses.
--journal PATH writes JSON Lines to a NEW file; existing files are not replaced.
--quiet suppresses event output, but still prints the final JSON status.

Exit codes: 0 complete (all glasses done), 1 runtime/logging error, 2 fault or
invalid arguments, 3 output full, 4 stopped, 5 complete with skipped targets.
This runner intentionally has no GUI or network server yet.
Paths are checked against fictional station bounds and a tool/glass envelope;
these example dimensions are not a calibration for real hardware.
Requires: Python 3.12+ standard library only.
"""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import sys
from time import monotonic, sleep

from system_batch import OutputSlot
from system_controller import Supervisor
from system_model import GlassTarget, SlotState, State, Step
from system_settings import STOP_TIMEOUT, TICK_PERIOD
from system_simulator import SimulatedDevices, SimulationClock


def positive_count(value):
    value = int(value)
    if not 1 <= value <= 1000:
        raise argparse.ArgumentTypeError("count must be between 1 and 1000")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--simulate", action="store_true", required=True,
                        help="use in-memory devices; hardware mode is not implemented")
    parser.add_argument("--fast", action="store_true", help="run with virtual time")
    parser.add_argument("--glasses", type=positive_count, default=3)
    parser.add_argument("--slots", type=positive_count, default=3)
    parser.add_argument("--fail-at", choices=[step.value for step in Step])
    parser.add_argument("--stop-at", choices=[step.value for step in Step])
    parser.add_argument("--auto-clear-output", action="store_true")
    parser.add_argument("--journal", type=Path)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    clock = SimulationClock() if args.fast else monotonic
    targets = [GlassTarget(f"glass-{i + 1}", 0.1 + i * 0.1, -0.4) for i in range(args.glasses)]
    slots = [OutputSlot(f"tag-{i + 1}", SlotState.FREE) for i in range(args.slots)]
    devices = SimulatedDevices(targets, fail_at=Step(args.fail_at) if args.fail_at else None)
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
            journal = stack.enter_context(args.journal.open("x", encoding="utf-8")) if args.journal else None

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
