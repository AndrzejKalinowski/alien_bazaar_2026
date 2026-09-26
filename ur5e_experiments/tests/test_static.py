"""Static checks over every script, without running (or importing) any of them.

Undefined names only show up when the code line runs, which for a robot
script can be mid-demo: AUDIT #1 was a NameError in main(). pyflakes finds
them without executing anything. Other pyflakes messages (unused imports...)
are not failures here.
"""

import glob
import os

import pytest

pyflakes_api = pytest.importorskip("pyflakes.api")
from pyflakes import messages  # noqa: E402
from pyflakes.reporter import Reporter  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = sorted(
    path for folder in ("ur5e_experiments", "hoverboard_experiments")
    for path in glob.glob(os.path.join(REPO, folder, "*.py"))
)
UNDEFINED = (messages.UndefinedName, messages.UndefinedLocal, messages.UndefinedExport)


class Collector(Reporter):
    def __init__(self):
        super().__init__(None, None)
        self.problems = []

    def flake(self, message):
        if isinstance(message, UNDEFINED):
            self.problems.append(str(message))

    def syntaxError(self, filename, msg, lineno, offset, text):
        self.problems.append(f"{filename}:{lineno}: syntax error: {msg}")

    def unexpectedError(self, filename, msg):
        self.problems.append(f"{filename}: {msg}")


def test_found_the_scripts():
    names = {os.path.basename(p) for p in SCRIPTS}
    assert {"pick_place_glasses.py", "find_glasses.py", "gamepad_robot_teleop.py"} <= names


@pytest.mark.parametrize("path", SCRIPTS, ids=os.path.basename)
def test_no_undefined_names(path):
    collector = Collector()
    with open(path, encoding="utf-8") as f:
        pyflakes_api.check(f.read(), path, collector)
    assert not collector.problems, "\n".join(collector.problems)
