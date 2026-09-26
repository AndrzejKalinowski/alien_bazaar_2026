"""pick_place_glasses: which glass p picks, and the side grip radius by orientation.

Importing pick_place_glasses opens no connection (that is all in main()).
"""

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("serial")

import find_glasses as fg  # noqa: E402
import glass_classifier as gc  # noqa: E402
import pick_place_glasses as ppg  # noqa: E402

PLACE = (0.5, 0.0)


def glass(x, y, orientation=None, diameter=0.06):
    return fg.Glass(x, y, 0.075, diameter, (0, 0, 30), orientation=orientation)


def obstruction(x, y, size=0.1):
    square = np.array([[x, y], [x + size, y], [x + size, y + size], [x, y + size]])
    return gc.TableObject("obstruction", square, (x + size / 2, y + size / 2), size ** 2, square)


def test_wall_radius_follows_orientation():
    h = ppg.SIDE_GRIP_HEIGHT
    down = ppg.wall_radius(glass(0, 0, gc.DOWN), h)
    up = ppg.wall_radius(glass(0, 0, gc.UP), h)
    # Low on the glass: the mouth when upside down, the foot when upright
    assert up < down
    assert down == pytest.approx(ppg.GLASS_SHAPE.radius_at(gc.DOWN, h))
    assert ppg.GLASS_FOOT_DIAMETER / 2 <= up <= down <= ppg.GLASS_MOUTH_DIAMETER / 2


def test_wall_radius_classic_mode_uses_measured_diameter(monkeypatch):
    monkeypatch.setattr(ppg, "SIDE_GRIP_RADIUS", None)
    assert ppg.wall_radius(glass(0, 0, None, diameter=0.066)) == pytest.approx(0.033)


def test_closest_pickable_glass_is_chosen():
    glasses = [glass(0.3, 0.0, gc.DOWN), glass(0.45, 0.0, gc.UP), glass(0.5, 0.01, gc.DOWN)]
    chosen, skipped = ppg.choose_glass(glasses, PLACE, from_side=True)
    assert (chosen.x, chosen.y) == (0.45, 0.0)
    assert skipped == ["already on the tag"]


def test_unclear_orientation_is_skipped():
    chosen, skipped = ppg.choose_glass([glass(0.4, 0.0, gc.UNSURE), glass(0.2, 0.0, gc.DOWN)], PLACE)
    assert chosen.x == 0.2
    assert "orientation unclear" in skipped[0]


def test_top_grip_skips_upright_glasses():
    glasses = [glass(0.45, 0.0, gc.UP), glass(0.3, 0.0, gc.DOWN)]
    assert ppg.choose_glass(glasses, PLACE, from_side=False)[0].x == 0.3
    assert ppg.choose_glass(glasses, PLACE, from_side=True)[0].x == 0.45


def test_glass_next_to_an_obstruction_is_skipped():
    glasses = [glass(0.45, 0.0, gc.DOWN), glass(0.2, 0.0, gc.DOWN)]
    arm = [obstruction(0.47, -0.05)]   # 3 cm from the first glass's wall
    chosen, skipped = ppg.choose_glass(glasses, PLACE, arm)
    assert chosen.x == 0.2
    assert "obstruction" in skipped[0]


def test_classic_mode_glasses_are_picked_as_before():
    chosen, skipped = ppg.choose_glass([glass(0.3, 0.0), glass(0.4, 0.0)], PLACE)
    assert chosen.x == 0.4 and skipped == []


def test_nothing_pickable():
    chosen, skipped = ppg.choose_glass([glass(0.3, 0.0, gc.UNSURE)], PLACE)
    assert chosen is None and len(skipped) == 1
