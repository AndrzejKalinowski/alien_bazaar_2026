"""
Waveshare ST/STS bus servos (e.g. ST3215) driven through the Seeed Studio
Bus Servo Driver Board, connected to the PC over USB serial.

The servos speak the Feetech STS protocol (half duplex, 1 Mbaud, 8N1):
    request:  FF FF <id> <len> <instr> <params...> <checksum>
    reply:    FF FF <id> <len> <error> <params...> <checksum>
    checksum = ~(id + len + instr/error + params) & 0xFF

Two servos share the bus:
  * the rotator (ROTATOR_ID) turns on command - to an absolute angle
    (position mode), by a relative angle, possibly more than one turn (step
    mode), or continuously (wheel mode);
  * the sprayer (SPRAYER_ID) moves back and forth between a rest and a press
    angle every SPRAY_PERIOD seconds to work a pump/sprayer (background thread).

New servos all ship with ID 1, so connect them one at a time and give the
sprayer its own ID first:   python bus_servos.py set-id 1 2

Use as a module:
    from bus_servos import ServoBus, BusServo, Sprayer
    bus = ServoBus("COM10")
    rotator = BusServo(bus, 1)
    sprayer = Sprayer(BusServo(bus, 2))
    sprayer.start()
    rotator.rotate_by(90)
    ...
    sprayer.stop()
    bus.close()

Standalone test (see --help for all commands):
    python bus_servos.py --port COM10 scan
    python bus_servos.py info 1
    python bus_servos.py move 1 180          # absolute angle, 0..360
    python bus_servos.py rotate 1 -450       # relative angle, multi-turn
    python bus_servos.py spin 1 800 --time 3 # steps/s, negative = reverse
    python bus_servos.py spray 2 --count 5
    python bus_servos.py watch 1             # torque off, print position while
                                             # you turn it by hand (--hold, --keep-torque)
    python bus_servos.py demo                # interactive: both servos

Requires: pip install pyserial
"""

import argparse
import threading
import time

import serial

PORT = "COM10"
BAUDRATE = 1_000_000

ROTATOR_ID = 2
SPRAYER_ID = 1

DEFAULT_SPEED = 1500       # steps/s (4096 steps per turn, ST3215 max ~3400)
DEFAULT_ACCELERATION = 50  # units of 100 steps/s^2, 0 = maximum

SPRAYER_REST_DEG = 280.0   # servo angle with the pump released
SPRAYER_PRESS_DEG = 250.0  # servo angle with the pump pressed
SPRAY_PERIOD = 2.0         # seconds between the starts of two strokes
SPRAY_HOLD = 0.2           # seconds to hold the pump pressed
SPRAYER_SPEED = 5500

STEPS_PER_REV = 4096
BROADCAST_ID = 0xFE

# Instructions
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# Control table (STS series)
REG_ID = 5
REG_MODE = 33              # 0 position, 1 wheel (speed), 2 PWM, 3 step
REG_TORQUE_ENABLE = 40
REG_ACCELERATION = 41
REG_GOAL_POSITION = 42
REG_GOAL_SPEED = 46
REG_LOCK = 55              # 0 = EEPROM writes are saved, 1 = not saved
REG_PRESENT_POSITION = 56
REG_PRESENT_SPEED = 58
REG_PRESENT_LOAD = 60
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMPERATURE = 63
REG_MOVING = 66
REG_PRESENT_CURRENT = 69

MODE_POSITION = 0
MODE_WHEEL = 1
MODE_STEP = 3

STATUS_ERRORS = {
    0x01: "voltage",
    0x02: "sensor",
    0x04: "overheat",
    0x08: "overcurrent",
    0x20: "overload",
}


class ServoError(Exception):
    pass


def _checksum(body):
    return (~sum(body)) & 0xFF


def _packet(servo_id, instruction, params=b""):
    body = bytes([servo_id, len(params) + 2, instruction]) + bytes(params)
    return b"\xff\xff" + body + bytes([_checksum(body)])


def _to_sign_magnitude(value, sign_bit=15):
    """STS registers store negative numbers as magnitude + sign bit."""
    magnitude = min(abs(int(value)), (1 << sign_bit) - 1)
    return magnitude | (1 << sign_bit) if value < 0 else magnitude


def _from_sign_magnitude(value, sign_bit=15):
    if value & (1 << sign_bit):
        return -(value & ((1 << sign_bit) - 1))
    return value


def _u16(value):
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


class ServoBus:
    """The serial link to the driver board, shared by all servos on it.

    Thread safe: each request/reply is done under a lock, so e.g. the sprayer
    thread and the main program can use the bus at the same time.
    """

    def __init__(self, port=PORT, baudrate=BAUDRATE, timeout=0.05):
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._lock = threading.Lock()
        self.last_error = 0  # error byte of the last status reply

    def close(self):
        self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _read_exact(self, n):
        data = self._serial.read(n)
        if len(data) != n:
            raise TimeoutError("no reply from servo")
        return data

    def _read_packet(self):
        prev = None
        while True:
            byte = self._read_exact(1)[0]
            if prev == 0xFF and byte == 0xFF:
                break
            prev = byte
        servo_id = self._read_exact(1)[0]
        while servo_id == 0xFF:  # tolerate extra header bytes
            servo_id = self._read_exact(1)[0]
        length = self._read_exact(1)[0]
        rest = self._read_exact(length)
        body = bytes([servo_id, length]) + rest[:-1]
        if _checksum(body) != rest[-1]:
            raise ServoError("bad checksum in reply")
        return b"\xff\xff" + body + rest[-1:]

    def transact(self, servo_id, instruction, params=b""):
        """Send one instruction and return the reply's parameter bytes."""
        request = _packet(servo_id, instruction, params)
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(request)
            if servo_id == BROADCAST_ID:
                return b""
            reply = self._read_packet()
            if reply == request:  # some adapters echo the half-duplex line
                reply = self._read_packet()
        if reply[2] != servo_id:
            raise ServoError(f"reply from ID {reply[2]}, expected {servo_id}")
        self.last_error = reply[4]
        return reply[5:-1]

    def ping(self, servo_id):
        try:
            self.transact(servo_id, INST_PING)
            return True
        except (TimeoutError, ServoError):
            return False

    def read(self, servo_id, address, length):
        data = self.transact(servo_id, INST_READ, bytes([address, length]))
        if len(data) != length:
            raise ServoError(f"expected {length} bytes, got {len(data)}")
        return data

    def read_u8(self, servo_id, address):
        return self.read(servo_id, address, 1)[0]

    def read_u16(self, servo_id, address):
        data = self.read(servo_id, address, 2)
        return data[0] | (data[1] << 8)

    def write(self, servo_id, address, data):
        self.transact(servo_id, INST_WRITE, bytes([address]) + bytes(data))

    def scan(self, ids=range(0, 21)):
        return [servo_id for servo_id in ids if self.ping(servo_id)]

    def set_id(self, old_id, new_id):
        """Permanently change a servo's ID (only that servo must answer to old_id)."""
        self.write(old_id, REG_LOCK, [0])
        self.write(old_id, REG_ID, [new_id])
        self.write(new_id, REG_LOCK, [1])


class BusServo:
    """One servo on the bus. Angles are in degrees, speeds in steps/s."""

    def __init__(self, bus, servo_id, speed=DEFAULT_SPEED, acceleration=DEFAULT_ACCELERATION):
        self.bus = bus
        self.id = servo_id
        self.speed = speed
        self.acceleration = acceleration
        self._mode = None

    def ping(self):
        return self.bus.ping(self.id)

    def set_mode(self, mode):
        # With the EEPROM locked (the default) this only lasts until power off,
        # which is what we want: no EEPROM wear from switching modes.
        if mode != self._mode:
            self.bus.write(self.id, REG_MODE, [mode])
            self._mode = mode

    def torque(self, enabled):
        self.bus.write(self.id, REG_TORQUE_ENABLE, [1 if enabled else 0])

    def _write_goal(self, position, speed, acceleration):
        speed = self.speed if speed is None else speed
        acceleration = self.acceleration if acceleration is None else acceleration
        # acceleration, goal position, goal time (unused), goal speed
        data = (bytes([acceleration]) + _u16(_to_sign_magnitude(position))
                + _u16(0) + _u16(abs(int(speed))))
        self.bus.write(self.id, REG_ACCELERATION, data)

    def move_to_steps(self, position, speed=None, acceleration=None):
        """Go to an absolute position, 0..4095 steps."""
        self.set_mode(MODE_POSITION)
        self._write_goal(max(0, min(STEPS_PER_REV - 1, int(position))), speed, acceleration)

    def move_to(self, degrees, speed=None, acceleration=None):
        """Go to an absolute angle, 0..360 degrees."""
        self.move_to_steps(round(degrees / 360 * STEPS_PER_REV), speed, acceleration)

    def rotate_by(self, degrees, speed=None, acceleration=None):
        """Turn by a relative angle (step mode, may be more than one turn)."""
        self.set_mode(MODE_STEP)
        self._write_goal(round(degrees / 360 * STEPS_PER_REV), speed, acceleration)

    def spin(self, speed, acceleration=None):
        """Turn continuously at speed steps/s (negative = reverse, 0 = stop)."""
        self.set_mode(MODE_WHEEL)
        acceleration = self.acceleration if acceleration is None else acceleration
        self.bus.write(self.id, REG_ACCELERATION, [acceleration])
        self.bus.write(self.id, REG_GOAL_SPEED, _u16(_to_sign_magnitude(speed)))

    def stop(self):
        """Stop where it is and hold the position."""
        if self._mode == MODE_WHEEL:
            self.spin(0)
        else:
            self.move_to_steps(self.position())

    def position(self):
        """Current position in steps (0..4095 in position mode)."""
        return _from_sign_magnitude(self.bus.read_u16(self.id, REG_PRESENT_POSITION))

    def angle(self):
        return self.position() * 360 / STEPS_PER_REV

    def is_moving(self):
        return self.bus.read_u8(self.id, REG_MOVING) != 0

    def wait_until_stopped(self, timeout=10.0, poll=0.02):
        """Block until the servo stops moving. Returns False on timeout."""
        time.sleep(0.05)  # give the servo time to start moving
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.is_moving():
                return True
            time.sleep(poll)
        return False

    def status(self):
        data = self.bus.read(self.id, REG_PRESENT_POSITION, REG_PRESENT_CURRENT + 2 - REG_PRESENT_POSITION)

        def u16(address):
            i = address - REG_PRESENT_POSITION
            return data[i] | (data[i + 1] << 8)

        error = self.bus.last_error
        return {
            "position": _from_sign_magnitude(u16(REG_PRESENT_POSITION)),
            "speed": _from_sign_magnitude(u16(REG_PRESENT_SPEED)),
            "load_percent": _from_sign_magnitude(u16(REG_PRESENT_LOAD), 10) / 10,
            "voltage": data[REG_PRESENT_VOLTAGE - REG_PRESENT_POSITION] / 10,
            "temperature": data[REG_PRESENT_TEMPERATURE - REG_PRESENT_POSITION],
            "moving": bool(data[REG_MOVING - REG_PRESENT_POSITION]),
            "current_mA": _from_sign_magnitude(u16(REG_PRESENT_CURRENT)) * 6.5,
            "errors": [name for bit, name in STATUS_ERRORS.items() if error & bit],
        }


class Sprayer:
    """Works a pump by moving a servo rest -> press -> rest every `period` s."""

    def __init__(self, servo, rest_deg=SPRAYER_REST_DEG, press_deg=SPRAYER_PRESS_DEG,
                 period=SPRAY_PERIOD, hold=SPRAY_HOLD, speed=SPRAYER_SPEED):
        self.servo = servo
        self.rest_deg = rest_deg
        self.press_deg = press_deg
        self.period = period
        self.hold = hold
        self.speed = speed
        self._stop = threading.Event()
        self._thread = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def rest(self):
        self.servo.move_to(self.rest_deg, self.speed)

    def stroke(self):
        """One full press and release (blocking)."""
        self.servo.move_to(self.press_deg, self.speed)
        self.servo.wait_until_stopped(timeout=self.period)
        self._stop.wait(self.hold)
        self.servo.move_to(self.rest_deg, self.speed)
        self.servo.wait_until_stopped(timeout=self.period)

    def start(self, period=None, count=None):
        """Start spraying in the background (forever, or `count` strokes)."""
        if period is not None:
            self.period = period
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(count,), daemon=True)
        self._thread.start()

    def stop(self):
        """Stop after the current stroke and go back to rest."""
        if self.running:
            self._stop.set()
            self._thread.join()

    def wait(self):
        if self._thread is not None:
            self._thread.join()

    def _run(self, count):
        strokes = 0
        while not self._stop.is_set() and (count is None or strokes < count):
            started = time.monotonic()
            self.stroke()
            strokes += 1
            self._stop.wait(max(0.0, self.period - (time.monotonic() - started)))
        self.rest()


def _watch(bus, ids, interval, keep_torque, hold):
    """Print the positions of `ids` until Ctrl+C, e.g. to find the angles for
    SPRAYER_REST_DEG / SPRAYER_PRESS_DEG by turning the servo by hand."""
    servos = [BusServo(bus, servo_id) for servo_id in ids]
    if not keep_torque:
        for servo in servos:
            servo.torque(False)
    print("Turn the servos by hand, Ctrl+C to stop.")
    try:
        while True:
            parts = []
            for servo in servos:
                try:
                    steps = servo.position()
                    parts.append(f"ID {servo.id}: {steps:5d} steps {steps * 360 / STEPS_PER_REV:6.1f} deg")
                except (TimeoutError, ServoError) as e:
                    parts.append(f"ID {servo.id}: {e}")
            print("\r" + "   ".join(parts) + "   ", end="", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print()
    finally:
        if hold:
            for servo in servos:
                # Set the goal to where it is now first, otherwise torque on
                # would snap it back to the last commanded goal.
                servo.move_to_steps(servo.position())
                servo.torque(True)


def _demo(bus):
    rotator = BusServo(bus, ROTATOR_ID)
    sprayer = Sprayer(BusServo(bus, SPRAYER_ID))
    sprayer.rest()
    print("Commands: <degrees> rotate by, a <degrees> go to angle, "
          "w <speed> spin (0 stops), s toggle sprayer, i info, q quit")
    try:
        while True:
            parts = input("> ").split()
            if not parts:
                continue
            cmd = parts[0].lower()
            try:
                if cmd == "q":
                    break
                elif cmd == "s":
                    if sprayer.running:
                        sprayer.stop()
                        print("sprayer stopped")
                    else:
                        sprayer.start()
                        print(f"sprayer running, period {sprayer.period} s")
                elif cmd == "a":
                    rotator.move_to(float(parts[1]))
                elif cmd == "w":
                    rotator.spin(int(parts[1]))
                elif cmd == "i":
                    print("rotator:", rotator.status())
                    print("sprayer:", sprayer.servo.status())
                else:
                    rotator.rotate_by(float(cmd))
            except (ValueError, IndexError):
                print("?")
            except (TimeoutError, ServoError) as e:
                print("error:", e)
    finally:
        sprayer.stop()
        rotator.stop()


def main():
    parser = argparse.ArgumentParser(description="Test Waveshare bus servos.")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUDRATE)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("scan", help="list servo IDs that answer")
    p.add_argument("--all", action="store_true", help="scan IDs 0..253 instead of 0..20")
    sub.add_parser("ping").add_argument("id", type=int)
    sub.add_parser("info").add_argument("id", type=int)

    p = sub.add_parser("move", help="go to an absolute angle (0..360)")
    p.add_argument("id", type=int)
    p.add_argument("degrees", type=float)
    p.add_argument("--speed", type=int, default=DEFAULT_SPEED)

    p = sub.add_parser("rotate", help="turn by a relative angle (multi-turn)")
    p.add_argument("id", type=int)
    p.add_argument("degrees", type=float)
    p.add_argument("--speed", type=int, default=DEFAULT_SPEED)

    p = sub.add_parser("spin", help="turn continuously")
    p.add_argument("id", type=int)
    p.add_argument("speed", type=int, help="steps/s, negative = reverse")
    p.add_argument("--time", type=float, default=2.0, help="seconds, then stop")

    p = sub.add_parser("spray", help="run the sprayer strokes")
    p.add_argument("id", type=int, nargs="?", default=SPRAYER_ID)
    p.add_argument("--period", type=float, default=SPRAY_PERIOD)
    p.add_argument("--count", type=int, default=5)
    p.add_argument("--rest", type=float, default=SPRAYER_REST_DEG)
    p.add_argument("--press", type=float, default=SPRAYER_PRESS_DEG)

    p = sub.add_parser("torque", help="enable/disable holding torque")
    p.add_argument("id", type=int)
    p.add_argument("state", choices=["on", "off"])

    p = sub.add_parser("set-id", help="permanently change a servo's ID")
    p.add_argument("old_id", type=int)
    p.add_argument("new_id", type=int)

    p = sub.add_parser("watch", help="torque off, print positions while you turn servos by hand")
    p.add_argument("ids", type=int, nargs="*", default=[ROTATOR_ID, SPRAYER_ID])
    p.add_argument("--interval", type=float, default=0.1, help="seconds between readings")
    p.add_argument("--keep-torque", action="store_true", help="don't switch torque off")
    p.add_argument("--hold", action="store_true",
                   help="on exit, hold the position the servo was left at (torque on)")

    sub.add_parser("demo", help="interactive test of rotator + sprayer")
    args = parser.parse_args()

    with ServoBus(args.port, args.baud) as bus:
        if args.command == "scan":
            print("found IDs:", bus.scan(range(0, 254) if args.all else range(0, 21)))
        elif args.command == "ping":
            print("OK" if bus.ping(args.id) else "no answer")
        elif args.command == "info":
            for key, value in BusServo(bus, args.id).status().items():
                print(f"{key:>12}: {value}")
        elif args.command in ("move", "rotate"):
            servo = BusServo(bus, args.id, speed=args.speed)
            if args.command == "move":
                servo.move_to(args.degrees)
            else:
                servo.rotate_by(args.degrees)
            servo.wait_until_stopped()
            print(f"at {servo.angle():.1f} deg")
        elif args.command == "spin":
            servo = BusServo(bus, args.id)
            servo.spin(args.speed)
            try:
                time.sleep(args.time)
            finally:
                servo.spin(0)
        elif args.command == "spray":
            sprayer = Sprayer(BusServo(bus, args.id), args.rest, args.press, args.period)
            sprayer.start(count=args.count)
            try:
                sprayer.wait()
            except KeyboardInterrupt:
                sprayer.stop()
        elif args.command == "torque":
            BusServo(bus, args.id).torque(args.state == "on")
        elif args.command == "set-id":
            if not bus.ping(args.old_id):
                raise SystemExit(f"no servo with ID {args.old_id}")
            bus.set_id(args.old_id, args.new_id)
            print("OK" if bus.ping(args.new_id) else "servo does not answer to the new ID")
        elif args.command == "watch":
            _watch(bus, args.ids, args.interval, args.keep_torque, args.hold)
        elif args.command == "demo":
            _demo(bus)


if __name__ == "__main__":
    main()
