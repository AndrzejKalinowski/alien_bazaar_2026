"""Hand-eye solver on synthetic AX = XB data with a known camera mounting X."""

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

import follow_april_tag as fat  # noqa: E402
import hand_eye_calibration as he  # noqa: E402

rng = np.random.default_rng(1)


def transform(rotvec, translation):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rotvec, float))[0]
    T[:3, 3] = translation
    return T


def synthetic_samples(X, count=8):
    """(tcp pose, T_cam_tag) pairs for a tag fixed in the base frame, seen through X = T_tcp_cam."""
    T_base_tag = transform([0.1, -0.2, 0.3], [0.5, 0.1, 0.0])
    samples = []
    for _ in range(count):
        T_base_tcp = transform(rng.normal(scale=0.4, size=3), rng.uniform(-0.1, 0.1, 3) + [0.4, 0, 0.4])
        T_cam_tag = np.linalg.inv(T_base_tcp @ X) @ T_base_tag
        samples.append((he.matrix_to_pose(T_base_tcp), T_cam_tag))
    return samples


def test_matrix_to_pose_round_trip():
    T = transform([0.3, -0.1, 0.7], [0.1, 0.2, 0.3])
    assert np.allclose(fat.pose_to_matrix(he.matrix_to_pose(T)), T)


def test_park_martin_recovers_known_mounting():
    X = transform([0.05, -0.02, 1.5], [0.05, 0.0, 0.1])   # like CAMERA_OFFSET, rotated about Z
    solved = he.park_martin(synthetic_samples(X))
    assert np.allclose(solved, X, atol=1e-6)


def test_average_rotation_of_noisy_copies():
    R = cv2.Rodrigues(np.array([0.4, 0.1, -0.3]))[0]
    noisy = [cv2.Rodrigues(rng.normal(scale=0.01, size=3))[0] @ R for _ in range(50)]
    mean = he.average_rotation(noisy)
    assert np.allclose(mean @ mean.T, np.eye(3), atol=1e-9)
    assert np.degrees(np.linalg.norm(cv2.Rodrigues(mean @ R.T)[0])) < 0.5
