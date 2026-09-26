"""
Jog the robot with a gamepad, for use inside other scripts' main loops.

Stick mapping (same everywhere):
  left stick      move in base X / Y
  triggers        move in Z (RT up, LT down)
  hold RB         rotate instead of move (stick = rx / ry, triggers = rz)
  hold LB         fine mode, FINE_SCALE of the normal speed

Usage:
  gamepad = GamepadControl({"BTN_SOUTH": " ", "BTN_START": "h"})   # button -> key
  jogger = Jogger(rtde_c, gamepad)
  while True:
      for key in gamepad.poll_keys(): ...     # buttons pressed since the last call
      jogger.update()                         # speedL from the sticks, stops when released
  jogger.stop(); gamepad.close()

Call update() at least every ~50 ms while jogging; before anything that blocks
the loop for longer, call jogger.stop(). Use it together with
robot_watchdog.RobotWatchdog, which stops the robot if the loop stalls anyway.
"""

from inputs import devices
from pygamepad.gamepads import Gamepad

MAX_LINEAR_SPEED = 0.08    # m/s at full stick deflection
MAX_ROTATION_SPEED = 0.3   # rad/s at full stick deflection
MAX_Z_SPEED = 0.08         # m/s at full trigger press
FINE_SCALE = 0.2           # speed factor while LB is held

SPEED_ACCEL = 0.4       # m/s^2 (or rad/s^2), smooths out stick changes
SPEED_CMD_TIME = 0.02   # s, speedL command time
STOP_DECEL = 1.0        # m/s^2 when the sticks are released

DEADZONE = 0.15


def apply_deadzone(value):
    return value if abs(value) > DEADZONE else 0.0


class GamepadControl:
    def __init__(self, button_keys=None):
        """button_keys: {pygamepad button name: key string} reported by poll_keys()."""
        if not devices.gamepads:
            print("No gamepad found, keyboard only")
        self._gamepad = Gamepad()
        self._gamepad.listen()
        self._button_keys = dict(button_keys or {})
        self._was_pressed = {name: False for name in self._button_keys}

    def poll_keys(self):
        """Keys for gamepad buttons pressed since the last call.

        pygamepad's is_just_pressed only lasts 20 ms, shorter than one video
        frame, so we detect the press edges ourselves.
        """
        keys = []
        for name, key in self._button_keys.items():
            pressed = bool(getattr(self._gamepad.buttons, name).value)
            if pressed and not self._was_pressed[name]:
                keys.append(key)
            self._was_pressed[name] = pressed
        return keys

    def jog_speed(self):
        """speedL vector [vx, vy, vz, wx, wy, wz] from the sticks, or None when released."""
        b = self._gamepad.buttons
        x = apply_deadzone(b.ABS_X.value)
        y = apply_deadzone(b.ABS_Y.value)
        trigger = apply_deadzone(b.ABS_RZ.value - b.ABS_Z.value)
        if x == 0 and y == 0 and trigger == 0:
            return None

        scale = FINE_SCALE if b.BTN_TL.value else 1.0
        speed = [0.0] * 6
        if b.BTN_TR.value:
            speed[3] = x * MAX_ROTATION_SPEED * scale
            speed[4] = y * MAX_ROTATION_SPEED * scale
            speed[5] = trigger * MAX_ROTATION_SPEED * scale
        else:
            speed[0] = y * MAX_LINEAR_SPEED * scale
            speed[1] = x * MAX_LINEAR_SPEED * scale
            speed[2] = trigger * MAX_Z_SPEED * scale
        return speed

    def close(self):
        self._gamepad.stop_listening()


class Jogger:
    """Sends speedL from the gamepad sticks, and speedStop once when they are released."""

    def __init__(self, rtde_c, gamepad):
        self._c = rtde_c
        self._gamepad = gamepad
        self.jogging = False

    def update(self, enabled=True):
        """Call every loop. Returns True while jogging."""
        speed = self._gamepad.jog_speed() if enabled else None
        if speed is not None:
            self._c.speedL(speed, SPEED_ACCEL, SPEED_CMD_TIME)
            self.jogging = True
        elif self.jogging:
            self.stop()
        return self.jogging

    def stop(self):
        if self.jogging:
            self._c.speedStop(STOP_DECEL)
        self.jogging = False
