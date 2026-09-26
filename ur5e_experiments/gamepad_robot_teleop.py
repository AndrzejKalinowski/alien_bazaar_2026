import rtde_control, rtde_receive
from time import sleep, time
from gamepad_jog import GamepadControl, Jogger
from robot_watchdog import RobotWatchdog
from suction import Suction, key_command

IP = "192.168.1.20"

HOME_Q = [0, -1.57, 1.57, -1.57, -1.57, 0]

LOOP_DT = 0.02          # seconds per control loop tick

HOME_SPEED = 1.0
HOME_ACCEL = 1.0

RECONNECT_DELAY = 1.0   # seconds to wait between reconnect attempts
ERROR_RETRY_DELAY = 0.5  # seconds to wait after a non-connection error

# Gamepad button -> key character: START = home, B = grip, X = release.
GAMEPAD_KEYS = {"BTN_START": "h", "BTN_EAST": "g", "BTN_WEST": "r"}


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
    gamepad = GamepadControl(GAMEPAD_KEYS)

    suction = Suction()

    r, c = connect_rtde()
    jogger = Jogger(c, gamepad)
    watchdog = RobotWatchdog(c)
    home_started = None   # time the home move started, None = not homing

    try:
        while True:
            try:
                if not watchdog.kick():
                    jogger.stop()
                    home_started = None

                keys = gamepad.poll_keys()

                if "h" in keys and home_started is not None:
                    print("WARNING: already going home, h ignored")
                elif "h" in keys:
                    print("Go home")
                    jogger.stop()
                    # Asynchronous: a blocking moveJ sends nothing and would trip the watchdog
                    c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL, True)
                    home_started = time()

                for key in ("g", "r"):
                    if key in keys:
                        # Refused commands only warn, so they never reach the robot recovery below
                        key_command(suction, key)

                if home_started is not None:
                    if gamepad.jog_speed() is not None:
                        print("Manual override, home move aborted")
                        c.stopJ()
                        home_started = None
                    # Give the async move a moment to start before checking if it finished
                    elif time() - home_started > 0.2 and c.getAsyncOperationProgress() < 0:
                        home_started = None
                if home_started is None:
                    jogger.update()
                sleep(LOOP_DT)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as e:
                print(f"Robot error: {e}. Attempting to recover without restarting...")
                sleep(ERROR_RETRY_DELAY)
                r, c = recover_from_fault(r, c)
                jogger = Jogger(c, gamepad)
                watchdog = RobotWatchdog(c)   # the re-uploaded script has no watchdog
                home_started = None
    except (KeyboardInterrupt, SystemExit):
        try:
            c.speedStop()
            c.stopScript()
        except Exception:
            pass
        gamepad.close()
        suction.close()
        exit()


if __name__ == "__main__":
    main()
