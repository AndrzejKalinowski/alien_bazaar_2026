"""
Drive the hoverboard via the Xiao's simple ASCII serial interface
(xiao_send_pwm.ino), using a gamepad.

Sends lines like "a100 b20\n" over USB serial:
  a<value> -> speed/pwm
  b<value> -> steer/turn
"""

from time import sleep, time

import serial
from pygamepad.gamepads import Gamepad

PORT = "COM8"
BAUDRATE = 115200

MAX_PWM = 500
MAX_STEER = 400

DEADZONE = 0.15

LOOP_DELAY = 0.03  # matches the Arduino's loop delay / well under its 500ms timeout

PRINT_EVERY = 0.2  # seconds between status prints, so the loop isn't spammed


def apply_deadzone(value):
    return value if abs(value) > DEADZONE else 0.0


def main():
    print(f"Connecting to Xiao on {PORT} @ {BAUDRATE} baud...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    sleep(2)  # let the board reset after opening the serial port

    gamepad = Gamepad()
    gamepad.listen()
    print("Waiting for gamepad input...")

    last_print = 0.0
    last_cmd = (None, None)

    try:
        while True:
            b = gamepad.buttons

            speed = apply_deadzone(b.ABS_Y.value)
            steer = apply_deadzone(b.ABS_X.value)

            pwm_speed = int(speed * MAX_PWM)
            pwm_steer = int(steer * MAX_STEER)

            ser.write(f"a{pwm_speed} b{pwm_steer}\n".encode("ascii"))

            now = time()
            if (pwm_speed, pwm_steer) != last_cmd and now - last_print >= PRINT_EVERY:
                print(f"speed={speed:+.2f} steer={steer:+.2f} -> a{pwm_speed} b{pwm_steer}")
                last_print = now
                last_cmd = (pwm_speed, pwm_steer)

            sleep(LOOP_DELAY)
    except (KeyboardInterrupt, SystemExit):
        print("\nStopping: zeroing PWM")
        ser.write(b"a0 b0\n")
        sleep(0.1)
        ser.close()
        gamepad.stop_listening()
        exit()


if __name__ == "__main__":
    main()
