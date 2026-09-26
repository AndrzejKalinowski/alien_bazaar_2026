"""Empty-table background and change mask, on synthetic scenes (tests/synthetic_table.py)."""

import copy

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

import synthetic_table as st  # noqa: E402
import table_background as tb  # noqa: E402


@pytest.fixture(scope="module")
def empty():
    return st.mat()


@pytest.fixture(scope="module")
def captured(empty):
    return tb.TableBackground.capture(st.frames(empty), st.SIZE, st.T_BASE_CAM,
                                      frames=10)


@pytest.fixture
def background(captured):
    return copy.deepcopy(captured)   # compare() adapts it


def blob_count(mask):
    return cv2.connectedComponents(mask)[0] - 1


def test_empty_table_is_unchanged(background, empty):
    change = background.compare(st.photo(empty, seed=7))
    assert change.mask.shape == (720, 1280)
    assert blob_count(change.mask) == 0
    assert not change.stale


@pytest.mark.parametrize("gain", [0.75, 1.3])
def test_auto_exposure_is_not_a_change(background, empty, gain):
    change = background.compare(st.photo(empty, gain=gain, seed=7))
    assert abs(change.gain - gain) < 0.05
    assert change.changed_fraction < 0.01


def test_glasses_are_changes(background, empty):
    scene = empty.copy()
    places = [(0.4, 0.1), (0.2, 0.25), (0.62, -0.05)]
    for xy, orientation in zip(places, ["down", "up", "down"]):
        st.draw_glass(scene, xy, orientation)
    change = background.compare(st.photo(scene, gain=1.2, seed=7))
    assert blob_count(change.mask) == 3
    for xy, orientation in zip(places, ["down", "up", "down"]):
        hull = st.silhouette(xy, orientation)
        inside = np.zeros((720, 1280), np.uint8)
        cv2.fillPoly(inside, [np.round(hull).astype(np.int32)], 1)
        # Filled solid, not just the rims
        assert np.mean(change.mask[inside > 0] > 0) > 0.9


def test_shadow_is_not_a_change_but_glass_in_it_is(background, empty):
    scene = empty.copy()
    st.draw_glass(scene, (0.3, 0.0), "down")
    area = [(150, 250), (900, 250), (900, 650), (150, 650)]
    scene = st.shadow(scene, area, 0.6)
    change = background.compare(st.photo(scene, seed=7))
    assert blob_count(change.mask) == 1
    center = np.round(st.project([[0.3, 0.0, st.HEIGHT / 2]])[0]).astype(int)
    assert change.mask[center[1], center[0]] == 255


def test_arm_is_a_change(background, empty):
    change = background.compare(st.photo(st.arm(empty.copy()), seed=7))
    assert change.mask[320, 1200] == 255


def test_stale_when_most_of_the_area_changed(background):
    other = st.mat(seed=5)   # a different table / the camera moved
    change = background.compare(st.photo(other, seed=7))
    assert change.stale


def test_stale_background_does_not_adapt(background):
    before = background.reference.copy()
    background.compare(st.photo(st.mat(seed=5), seed=7))
    assert np.array_equal(background.reference, before)


def test_adapts_to_slow_light_change_but_not_to_glass(background, empty, monkeypatch):
    monkeypatch.setattr(tb, "ADAPT_RATE", 0.3)   # fewer frames needed
    scene = empty.copy()
    st.draw_glass(scene, (0.4, 0.1), "down")
    # Light slowly getting warmer: not a pure exposure change, so it must adapt
    warmer = scene * np.array([0.97, 1.0, 1.04], np.float32)
    for seed in range(15):
        change = background.compare(st.photo(warmer, seed=seed))
    assert blob_count(change.mask) == 1
    center = np.round(st.project([[0.4, 0.1, st.HEIGHT / 2]])[0]).astype(int)
    assert change.mask[center[1], center[0]] == 255


def test_save_load_round_trip(tmp_path, background):
    path = str(tmp_path / "bg.npz")
    background.save(path)
    loaded = tb.TableBackground.load(st.SIZE, st.T_BASE_CAM, path)
    assert np.array_equal(loaded.background, background.background)
    assert np.allclose(loaded.noise, background.noise)
    assert not (tmp_path / "bg.npz.tmp.npz").exists()


def test_load_refuses_other_camera_pose(tmp_path, background, capsys):
    path = str(tmp_path / "bg.npz")
    background.save(path)
    moved = st.T_BASE_CAM.copy()
    moved[0, 3] += 0.01
    assert tb.TableBackground.load(st.SIZE, moved, path) is None
    assert "calibrated again" in capsys.readouterr().out
    assert tb.TableBackground.load((640, 480), st.T_BASE_CAM, path) is None


def test_load_missing_file_is_none(tmp_path):
    assert tb.TableBackground.load(st.SIZE, st.T_BASE_CAM, str(tmp_path / "none.npz")) is None
