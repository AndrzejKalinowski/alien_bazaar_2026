"""Frames, rotations and the overhead-camera back-projection, on a synthetic camera."""

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

import find_glasses as fg  # noqa: E402
import follow_april_tag as fat  # noqa: E402

rng = np.random.default_rng(0)

# Synthetic overhead camera: 1 m above the table, looking straight down, 1280x720
K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
T_BASE_CAM = np.eye(4)
T_BASE_CAM[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]   # camera Z points down
T_BASE_CAM[:3, 3] = [0.4, 0.1, 1.0]


def project(points):
    T_cam_base = np.linalg.inv(T_BASE_CAM)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    pixels, _ = cv2.projectPoints(np.asarray(points, float), rvec, T_cam_base[:3, 3], K, None)
    return pixels.reshape(-1, 2)


def random_unit():
    v = rng.normal(size=3)
    return v / np.linalg.norm(v)


def test_pose_to_matrix():
    pose = [0.1, -0.2, 0.3, 0, 0, np.pi / 2]   # 90 deg around Z
    T = fat.pose_to_matrix(pose)
    assert np.allclose(T[:3, 3], [0.1, -0.2, 0.3])
    assert np.allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(T[3], [0, 0, 0, 1])


@pytest.mark.parametrize("case", ["random", "same", "opposite"])
def test_rotvec_between(case):
    for _ in range(20):
        a = random_unit()
        b = {"random": random_unit(), "same": a, "opposite": -a}[case]
        R = cv2.Rodrigues(fat.rotvec_between(a, b))[0]
        assert np.allclose(R @ a, b, atol=1e-9)


def test_rotation_error_turns_current_into_target():
    for _ in range(20):
        R_target = cv2.Rodrigues(rng.normal(size=3))[0]
        R_current = cv2.Rodrigues(rng.normal(size=3))[0]
        R_err = cv2.Rodrigues(fat.rotation_error(R_target, R_current))[0]
        assert np.allclose(R_err @ R_current, R_target, atol=1e-6)


def test_pixel_to_plane_inverts_projection():
    points = np.column_stack([rng.uniform(0.1, 0.7, 20), rng.uniform(-0.2, 0.4, 20),
                              rng.uniform(-0.05, 0.1, 20)])
    for point, pixel in zip(points, project(points)):
        hit = fg.pixel_to_plane(K, T_BASE_CAM, pixel, point[2])
        assert np.allclose(hit, point, atol=1e-9)


def test_pixel_to_plane_behind_camera_is_none():
    # A plane above the camera cannot be seen by a camera looking down
    assert fg.pixel_to_plane(K, T_BASE_CAM, (640, 360), 2.0) is None


def test_tag_centers_finds_rendered_tags():
    dictionary = cv2.aruco.getPredefinedDictionary(fat.TAG_DICTIONARY)
    image = np.full((720, 1280, 3), 255, np.uint8)
    expected = {}
    for tag_id, (x, y) in {3: (200, 150), 7: (900, 500)}.items():
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 120)
        image[y:y + 120, x:x + 120] = marker[..., None]
        expected[tag_id] = np.array([x + 59.5, y + 59.5])   # pixel centers
    centers = fg.tag_centers(fg.make_tag_detector(), image)
    assert set(centers) == set(expected)
    for tag_id, center in expected.items():
        assert np.linalg.norm(centers[tag_id] - center) < 1.0


def test_solve_camera_pose_recovers_synthetic_camera(capsys):
    base = np.column_stack([rng.uniform(0.1, 0.7, 12), rng.uniform(-0.2, 0.4, 12),
                            rng.choice([0.0, 0.03, 0.08], 12)])   # some raised on blocks
    points = list(zip(project(base), base))
    T, max_mm = fg.solve_camera_pose(points, K)
    assert np.allclose(T, T_BASE_CAM, atol=1e-6)
    assert max_mm < 0.01
