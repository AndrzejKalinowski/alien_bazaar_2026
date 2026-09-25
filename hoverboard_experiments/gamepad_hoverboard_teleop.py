from time import sleep, time

from pygamepad.gamepads import Gamepad

from bipropellant_serial import BipropellantSerial

PORT = "COM6"
BAUDRATE = 115200

MAX_PWM = 600      # matches firmware default speed_max_power
MIN_MOVING_PWM = 40  # matches firmware default speed_minimum_pwm

DEADZONE = 0.15

LOOP_DELAY = 0.02  # 50 Hz


def apply_deadzone(value):
    return value if abs(value) > DEADZONE else 0.0


PRINT_EVERY = 0.2  # seconds between status prints, so the loop isn't spammed at 50Hz


def main():
    print(f"Connecting to hoverboard on {PORT} @ {BAUDRATE} baud...")
    board = BipropellantSerial(PORT, BAUDRATE)
    board.set_enabled(True)
    print("Motors enabled. Waiting for gamepad input...")

    gamepad = Gamepad()
    gamepad.listen()

    last_print = 0.0
    last_pwm = (None, None)

    try:
        while True:
            b = gamepad.buttons

            if b.BTN_START.is_just_pressed:
                print("START pressed -> zeroing PWM")
                board.set_pwm(0, 0)
                sleep(0.1)
                continue

            throttle = -apply_deadzone(b.ABS_Y.value)
            steer = apply_deadzone(b.ABS_X.value)

            pwm_left = throttle - steer
            pwm_right = throttle + steer

            pwm_left = int(max(-1.0, min(1.0, pwm_left)) * MAX_PWM)
            pwm_right = int(max(-1.0, min(1.0, pwm_right)) * MAX_PWM)

            board.set_pwm(pwm_left, pwm_right, max_power=MAX_PWM, min_power=-MAX_PWM,
                          minimum_pwm=MIN_MOVING_PWM)

            now = time()
            if (pwm_left, pwm_right) != last_pwm and now - last_print >= PRINT_EVERY:
                print(f"throttle={throttle:+.2f} steer={steer:+.2f} "
                      f"-> pwm_left={pwm_left:+d} pwm_right={pwm_right:+d}")
                last_print = now
                last_pwm = (pwm_left, pwm_right)

            sleep(LOOP_DELAY)
    except (KeyboardInterrupt, SystemExit):
        print("\nStopping: zeroing PWM and disabling motors")
        board.set_pwm(0, 0)
        board.set_enabled(False)
        board.close()
        gamepad.stop_listening()
        exit()


if __name__ == "__main__":
    main()
