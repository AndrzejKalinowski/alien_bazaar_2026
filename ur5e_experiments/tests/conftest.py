"""
Offline tests: no robot, gripper, servos, gamepad or camera needed.

The scripts are flat modules imported from ur5e_experiments/, so that
directory goes on sys.path. Hardware-only libraries (ur_rtde, the gamepad
libraries) are replaced by empty stand-ins when they are not installed, so
the modules can be imported; nothing here may construct an RTDE interface.

Run from the repo root:  python -m pytest ur5e_experiments/tests
"""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


def _stub(name, **attributes):
    try:
        __import__(name)
    except ImportError:
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        sys.modules[name] = module


def _not_in_tests(*args, **kwargs):
    raise RuntimeError("tests must not open robot or gamepad connections")


_stub("rtde_control", RTDEControlInterface=_not_in_tests)
_stub("rtde_receive", RTDEReceiveInterface=_not_in_tests)
_stub("inputs", devices=types.SimpleNamespace(gamepads=[]))
_stub("pygamepad")
_stub("pygamepad.gamepads", Gamepad=_not_in_tests)
