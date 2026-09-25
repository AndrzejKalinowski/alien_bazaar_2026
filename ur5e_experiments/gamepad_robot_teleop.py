import rtde_control, rtde_receive
import serial
from time import sleep
from pygamepad.gamepads import Gamepad

IP = "192.168.1.20"

SUCTION_PORT = "COM9"
SUCTION_BAUDRATE = 115200

HOME_Q = [0, -1.57, 1.57, -1.57, -1.57, 0]

MAX_LINEAR_SPEED = 0.15    # m/s at full stick deflection
MAX_ROTATION_SPEED = 0.4   # rad/s at full stick deflection
MAX_Z_SPEED = 0.15         # m/s at full trigger press

SPEED_ACCEL = 0.6       # m/s^2 (or rad/s^2), smooths out stick changes
LOOP_DT = 0.02          # seconds per control loop tick

HOME_SPEED = 1.0
HOME_ACCEL = 1.0

DEADZONE = 0.15

RECONNECT_DELAY = 1.0   # seconds to wait between reconnect attempts
ERROR_RETRY_DELAY = 0.5  # seconds to wait after a non-connection error


def apply_deadzone(value):
    return value if abs(value) > DEADZONE else 0.0


def connect_rtde():
    while True:
        try:
            r = rtde_receive.RTDEReceiveInterface(IP)
            c = rtde_control.RTDEControlInterface(IP)
            print("Connected to robot. TCP pose:", r.getActualTCPPose())
            return r, c
        except Exception as e:
            print(f"Failed to connect to robot ({e}), retrying in {RECONNECT_DELAY}s...")
            sleep(RECONNECT_DELAY)


def recover_from_fault(r, c):
    """Try to clear whatever's wrong with the robot without restarting the script."""
    try:
        if not (r.isConnected() and c.isConnected()):
            raise ConnectionError("lost RTDE connection")

        # The control script on the pendant stops running after a protective
        # stop, an e-stop, switching to local mode, etc. Re-uploading it is
        # what actually clears "RTDE control script is not running!".
        try:
            c.reuploadScript()
        except Exception as e:
            print(f"reuploadScript failed ({e}), will check robot state...")

        try:
            while c.isProtectiveStopped():
                print("Robot is protective-stopped, waiting for it to clear...")
                sleep(RECONNECT_DELAY)
        except Exception as e:
            print(f"Could not query protective stop state ({e})")

        try:
            while r.isEmergencyStopped():
                print("Robot is emergency-stopped, waiting for the e-stop to be released...")
                sleep(RECONNECT_DELAY)
        except Exception as e:
            print(f"Could not query emergency stop state ({e})")

        # State may have changed while waiting above, so make sure the
        # control script is actually running before handing control back.
        c.reuploadScript()

        return r, c
    except Exception as e:
        print(f"Recovery failed ({e}), reconnecting from scratch...")
        try:
            c.disconnect()
        except Exception:
            pass
        try:
            r.disconnect()
        except Exception:
            pass
        return connect_rtde()


def main():
    gamepad = Gamepad()
    gamepad.listen()

    suction = serial.Serial(SUCTION_PORT, SUCTION_BAUDRATE, timeout=0)
    sleep(2)  # let the suction controller reset after opening the serial port

    r, c = connect_rtde()

    try:
        while True:
            try:
                b = gamepad.buttons

                if b.BTN_START.is_just_pressed:
                    print("Go home")
                    c.speedStop()
                    c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL)
                    sleep(0.1)
                    continue

                if b.BTN_EAST.is_just_pressed:
                    print("Grip")
                    suction.write(b"GRIP\n")
                    sleep(0.1)
                    continue

                if b.BTN_WEST.is_just_pressed:
                    print("Release")
                    suction.write(b"RELEASE\n")
                    sleep(0.1)
                    continue

                rotation_mode = b.BTN_TR.value

                x = apply_deadzone(b.ABS_X.value)
                y = apply_deadzone(b.ABS_Y.value)
                trigger = apply_deadzone(b.ABS_RZ.value - b.ABS_Z.value)

                if x == 0 and y == 0 and trigger == 0:
                    c.speedStop()
                    sleep(LOOP_DT)
                    continue

                speed = [0.0] * 6

                if rotation_mode:
                    speed[3] = x * MAX_ROTATION_SPEED
                    speed[4] = y * MAX_ROTATION_SPEED
                    speed[5] = trigger * MAX_ROTATION_SPEED
                else:
                    speed[0] = y * MAX_LINEAR_SPEED
                    speed[1] = x * MAX_LINEAR_SPEED
                    speed[2] = trigger * MAX_Z_SPEED

                c.speedL(speed, SPEED_ACCEL, LOOP_DT)
                sleep(LOOP_DT)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as e:
                print(f"Robot error: {e}. Attempting to recover without restarting...")
                sleep(ERROR_RETRY_DELAY)
                r, c = recover_from_fault(r, c)
    except (KeyboardInterrupt, SystemExit):
        try:
            c.speedStop()
            c.stopScript()
        except Exception:
            pass
        gamepad.stop_listening()
        suction.close()
        exit()


if __name__ == "__main__":
    main()
