"""GlassFinder in diff mode: background capture, detection with orientation, measure() voting.

The camera is a fake that returns synthetic, already undistorted frames; the
real calibration / settings / area files are replaced with temporary ones.
"""

import itertools

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

import find_glasses as fg  # noqa: E402
import glass_classifier as gc  # noqa: E402
import synthetic_table as st  # noqa: E402
import table_background as tb  # noqa: E402


class FakeCamera:
    def __init__(self, scene):
        self.scene = scene
        self.seeds = itertools.count(200)

    def read(self):
        return True, st.photo(self.scene, seed=next(self.seeds))


@pytest.fixture
def finder(tmp_path, monkeypatch):
    monkeypatch.setattr(fg, "SETTINGS_FILE", str(tmp_path / "settings.json"))
    monkeypatch.setattr(fg, "AREA_FILE", str(tmp_path / "area.json"))
    monkeypatch.setattr(tb, "BACKGROUND_FILE", str(tmp_path / "background.npz"))
    monkeypatch.setattr(tb, "BACKGROUND_FRAMES", 8)
    monkeypatch.setattr(fg, "MEASURE_FRAMES", 4)
    finder = fg.GlassFinder(st.K, st.T_BASE_CAM, st.TABLE_Z, *st.SIZE, rim_height=st.HEIGHT,
                            mouth_diameter=st.MOUTH, foot_diameter=st.FOOT)
    finder.undistort = lambda frame: frame   # the fake camera has no lens distortion
    return finder


def test_starts_classic_without_background(finder):
    assert finder.mode == fg.CLASSIC
    assert "b" in finder.background_status()
    assert "no table background" in fg.diff_key(finder, "d", None)


def test_capture_then_detect_with_orientation(finder, tmp_path):
    empty = st.mat()
    kicks = []
    status = fg.diff_key(finder, "b", FakeCamera(empty), kick=lambda: kicks.append(1))
    assert "captured" in status and finder.mode == fg.DIFF
    assert (tmp_path / "background.npz").exists()
    assert len(kicks) >= tb.BACKGROUND_FRAMES   # the watchdog is kicked while capturing

    scene = empty.copy()
    placed = [((0.2, 0.25), gc.DOWN), ((0.65, -0.05), gc.UP)]
    for xy, orientation in placed:
        st.draw_glass(scene, xy, orientation)
    glasses = finder.detect(st.photo(scene, seed=3))
    assert sorted(g.orientation for g in glasses) == [gc.DOWN, gc.UP]

    measured = finder.measure(FakeCamera(scene))
    for xy, orientation in placed:
        g = min(measured, key=lambda g: np.hypot(g.x - xy[0], g.y - xy[1]))
        assert np.hypot(g.x - xy[0], g.y - xy[1]) < 0.003
        assert g.orientation == orientation
        assert g.z == pytest.approx(st.TABLE_Z + st.HEIGHT)

    # A new finder picks the saved background up by itself
    again = fg.GlassFinder(st.K, st.T_BASE_CAM, st.TABLE_Z, *st.SIZE, rim_height=st.HEIGHT)
    assert again.mode == fg.DIFF


def test_detection_area_filters_glasses(finder):
    empty = st.mat()
    fg.diff_key(finder, "b", FakeCamera(empty))
    scene = empty.copy()
    st.draw_glass(scene, (0.2, 0.25), gc.DOWN)
    st.draw_glass(scene, (0.65, -0.05), gc.UP)
    finder.area = np.array([[0.5, -0.2], [0.8, -0.2], [0.8, 0.1], [0.5, 0.1]])
    glasses = finder.detect(st.photo(scene, seed=3))
    assert [g.orientation for g in glasses] == [gc.UP]


def test_out_of_time_reports_unsearched_changes(finder, monkeypatch):
    empty = st.mat()
    fg.diff_key(finder, "b", FakeCamera(empty))
    scene = empty.copy()
    st.draw_glass(scene, (0.2, 0.25), gc.DOWN)
    monkeypatch.setattr(gc, "CLASSIFY_BUDGET", -1.0)
    assert finder.detect(st.photo(scene, seed=3)) == []
    assert finder.unsearched == 1
    assert "1 changes not searched" in finder.background_status()


def test_stale_background_finds_nothing(finder):
    fg.diff_key(finder, "b", FakeCamera(st.mat()))
    assert finder.detect(st.photo(st.mat(seed=9), seed=3)) == []
    assert "STALE" in finder.background_status()


def test_d_switches_modes_and_v_the_mask(finder):
    fg.diff_key(finder, "b", FakeCamera(st.mat()))
    fg.diff_key(finder, "d", None)
    assert finder.mode == fg.CLASSIC
    fg.diff_key(finder, "d", None)
    assert finder.mode == fg.DIFF
    fg.diff_key(finder, "v", None)
    assert finder.show_mask


@pytest.mark.parametrize("votes, expected", [
    ([None, None], None),
    ([gc.UP] * 4, gc.UP),
    ([gc.DOWN, gc.DOWN, gc.DOWN, gc.UNSURE], gc.DOWN),
    ([gc.UP, gc.DOWN, gc.UP, gc.DOWN], gc.UNSURE),
    ([gc.UNSURE, gc.UNSURE, gc.UP], gc.UNSURE),
])
def test_vote_orientation(votes, expected):
    cluster = [fg.Glass(0, 0, 0, 0.06, (0, 0, 1), orientation=v) for v in votes]
    assert fg.vote_orientation(cluster) == expected
