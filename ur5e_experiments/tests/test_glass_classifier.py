"""Glass orientation / object classification, on synthetic scenes (tests/synthetic_table.py)."""

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

import glass_classifier as gc  # noqa: E402
import synthetic_table as st  # noqa: E402
import table_background as tb  # noqa: E402

CAM = gc.TableCamera(st.K, st.T_BASE_CAM, st.TABLE_Z)
SHAPE = gc.GlassShape(st.HEIGHT, st.MOUTH, st.FOOT)
EDGE, ROUNDNESS = 150, 0.8   # the find_glasses.py trackbar defaults

# Under the camera (no parallax), next to it, and far out in every direction
SCENE = [((0.40, 0.10), gc.DOWN), ((0.50, 0.10), gc.UP), ((0.30, 0.10), gc.UP),
         ((0.15, 0.30), gc.DOWN), ((0.15, -0.10), gc.UP), ((0.70, 0.30), gc.UP),
         ((0.72, -0.12), gc.DOWN)]


@pytest.fixture(scope="module")
def empty():
    return st.mat()


@pytest.fixture(scope="module")
def background(empty):
    return tb.TableBackground.capture(st.frames(empty), st.SIZE, st.T_BASE_CAM, frames=10)


def classify(background, scene, gain=1.0):
    frame = st.photo(scene, gain=gain, seed=3)
    change = background.compare(frame, adapt=False)
    return gc.classify(frame, change.mask, CAM, SHAPE, EDGE, ROUNDNESS)


def match(glasses, xy):
    return min(glasses, key=lambda g: np.linalg.norm(g.xy - xy))


# --- geometry ---------------------------------------------------------------------

def test_to_plane_inverts_project():
    points = np.column_stack([np.linspace(0.1, 0.7, 9), np.linspace(-0.2, 0.4, 9),
                              np.linspace(0, 0.1, 9)])
    for p in points:
        assert np.allclose(CAM.to_plane(CAM.project(p), p[2])[0], p, atol=1e-9)


@pytest.mark.parametrize("xy", [(0.4, 0.1), (0.15, -0.1), (0.72, 0.3)])
def test_circle_on_plane_measures_a_projected_ring(xy):
    pixels = CAM.ring(xy, st.HEIGHT, 0.07, n=360).astype(np.float32)
    (u, v), r = cv2.minEnclosingCircle(pixels)
    center, diameter = CAM.circle_on_plane((u, v, r), st.HEIGHT)
    assert np.linalg.norm(center - xy) < 0.002
    assert abs(diameter - 0.07) < 0.002


def test_radius_range_covers_both_ends():
    lo, hi = gc.radius_range(CAM, SHAPE)
    foot_on_table = 900 * st.FOOT / 2 / 1.0
    mouth_on_top = 900 * st.MOUTH / 2 / (1.0 - st.HEIGHT)
    assert lo < foot_on_table and hi > mouth_on_top


def test_radius_at_follows_the_taper():
    assert SHAPE.radius_at(gc.DOWN, 0) == pytest.approx(st.MOUTH / 2)
    assert SHAPE.radius_at(gc.DOWN, st.HEIGHT) == pytest.approx(st.FOOT / 2)
    assert SHAPE.radius_at(gc.UP, 0) == pytest.approx(st.FOOT / 2)
    assert SHAPE.radius_at(gc.UP, st.HEIGHT / 2) == pytest.approx((st.MOUTH + st.FOOT) / 4)


# --- orientation from exact circles -------------------------------------------------

def exact_circles(xy, orientation):
    bottom, top = st.glass_ends(orientation)
    circles = []
    for z, d in ((st.TABLE_Z, bottom), (st.HEIGHT, top)):
        (u, v), r = cv2.minEnclosingCircle(CAM.ring(xy, z, d, n=360).astype(np.float32))
        circles.append((float(u), float(v), float(r)))
    return circles


def full_blob(xy, orientation):
    mask = np.zeros((720, 1280), np.uint8)
    cv2.fillPoly(mask, [np.round(st.silhouette(xy, orientation)).astype(np.int32)], 255)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    return gc.Blob(labels, 1, stats[1], st.SIZE)


@pytest.mark.parametrize("orientation", [gc.UP, gc.DOWN])
@pytest.mark.parametrize("xy", [(0.4, 0.1), (0.42, 0.12), (0.2, 0.3), (0.7, -0.1)])
def test_pair_of_circles_gives_orientation_and_axis(xy, orientation):
    g = gc.classify_candidate(CAM, SHAPE, full_blob(xy, orientation), exact_circles(xy, orientation))
    assert g.orientation == orientation
    assert np.linalg.norm(g.xy - xy) < 0.002


def test_single_circle_far_out_still_gives_orientation():
    # Parallax: the outline around one circle only fits one way
    xy = (0.72, -0.12)
    for orientation in (gc.UP, gc.DOWN):
        top = exact_circles(xy, orientation)[1]
        g = gc.classify_candidate(CAM, SHAPE, full_blob(xy, orientation), [top])
        assert g.orientation == orientation


def test_wrong_size_circles_are_not_a_glass():
    xy = (0.3, 0.2)
    circles = [(u, v, r * 1.6) for u, v, r in exact_circles(xy, gc.UP)]
    assert gc.classify_candidate(CAM, SHAPE, full_blob(xy, gc.UP), circles) is None


# --- whole scenes -----------------------------------------------------------------

def test_scene_orientations_and_positions(background, empty):
    scene = empty.copy()
    for xy, orientation in SCENE:
        st.draw_glass(scene, xy, orientation)
    glasses, objects = classify(background, scene, gain=1.15)
    assert len(glasses) == len(SCENE)
    for xy, orientation in SCENE:
        g = match(glasses, xy)
        assert np.linalg.norm(g.xy - xy) < 0.003, (xy, g.xy)
        assert g.orientation == orientation, (xy, g.orientation, g.margin)
    assert objects == []


def test_ring_printed_on_the_mat_is_not_a_glass(empty):
    # Was on the table when it was empty: HoughCircles would find it, the diff does not
    table = empty.copy()
    cv2.circle(table, (300, 200), 35, (220, 220, 220), 3)
    bg = tb.TableBackground.capture(st.frames(table, n=5), st.SIZE, st.T_BASE_CAM, frames=5)
    glasses, objects = classify(bg, table)
    assert glasses == [] and objects == []


def test_box_is_an_unknown_object(background, empty):
    scene = empty.copy()
    st.draw_glass(scene, (0.2, 0.0), gc.DOWN)
    cv2.rectangle(scene, (800, 150), (860, 210), (40, 90, 160), -1)   # ~6.5 cm square
    glasses, objects = classify(background, scene)
    assert len(glasses) == 1 and glasses[0].orientation == gc.DOWN
    assert [o.label for o in objects] == ["unknown"]
    assert 0.002 < objects[0].area < 0.008


def test_arm_is_an_obstruction_and_blocks_the_glass_next_to_it(background, empty):
    scene = empty.copy()
    st.draw_glass(scene, (0.2, 0.0), gc.DOWN)
    far = (0.2, 0.0)
    near_arm = CAM.to_plane([(960, 330)], st.TABLE_Z)[0, :2]   # just left of the arm
    st.draw_glass(scene, near_arm, gc.UP)
    glasses, objects = classify(background, st.arm(scene))
    assert "obstruction" in [o.label for o in objects]
    reach = st.MOUTH / 2 + 0.03   # axis to wall, plus a 3 cm gap
    assert gc.near(np.array(far), objects, reach) == []
    assert gc.near(np.array(near_arm), objects, reach) != []
