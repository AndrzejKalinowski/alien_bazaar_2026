"""Synthetic overhead-camera scenes: a textured mat, tapered glasses, shadows, an arm.

Not a test module (no test_ prefix); imported by the table / classifier tests.
The camera is the one of test_geometry.py: 1 m above the table, looking
straight down, 1280x720, already undistorted.
"""

import cv2
import numpy as np

K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
T_BASE_CAM = np.eye(4)
T_BASE_CAM[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]   # camera Z points down
T_BASE_CAM[:3, 3] = [0.4, 0.1, 1.0]
SIZE = (1280, 720)
TABLE_Z = 0.0
NADIR = (0.4, 0.1)            # base x, y straight under the camera

HEIGHT = 0.075                # m, glass
MOUTH = 0.080                 # m, diameter of the open end
FOOT = 0.060                  # m, diameter of the closed end


def project(points):
    T_cam_base = np.linalg.inv(T_BASE_CAM)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    pixels, _ = cv2.projectPoints(np.asarray(points, float).reshape(-1, 3), rvec,
                                  T_cam_base[:3, 3], K, None)
    return pixels.reshape(-1, 2)


def ring(xy, z, diameter, n=120):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    points = np.column_stack([xy[0] + diameter / 2 * np.cos(a), xy[1] + diameter / 2 * np.sin(a),
                              np.full(n, z)])
    return project(points)


def mat(seed=0):
    """Dark, slightly textured table mat (what find_glasses.py recommends)."""
    rng = np.random.default_rng(seed)
    h, w = SIZE[1], SIZE[0]
    coarse = cv2.resize(rng.normal(0, 1, (h // 16, w // 16)).astype(np.float32), (w, h),
                        interpolation=cv2.INTER_CUBIC)
    fine = cv2.GaussianBlur(rng.normal(0, 1, (h, w)).astype(np.float32), (0, 0), 1.5)
    gray = 60 + 10 * coarse + 12 * fine
    image = np.dstack([gray * 1.05, gray, gray * 0.95])
    return np.clip(image, 0, 255).astype(np.float32)


def glass_ends(orientation):
    """(bottom diameter, top diameter): 'down' stands on its mouth."""
    return (MOUTH, FOOT) if orientation == "down" else (FOOT, MOUTH)


def silhouette(xy, orientation):
    bottom, top = glass_ends(orientation)
    points = np.vstack([ring(xy, TABLE_Z, bottom), ring(xy, TABLE_Z + HEIGHT, top)])
    return cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)


def draw_glass(image, xy, orientation):
    """Transparent glass: refracted (shifted, lighter) mat inside, bright rims."""
    hull = silhouette(xy, orientation)
    inside = np.zeros(image.shape[:2], np.uint8)
    cv2.fillPoly(inside, [np.round(hull).astype(np.int32)], 1)
    shifted = np.roll(image, (7, -5), axis=(0, 1)) * 1.1 + 12
    image[inside > 0] = shifted[inside > 0]
    bottom, top = glass_ends(orientation)
    # The closed end is thick glass (strong edge), the open end a thin bright rim
    for z, d in ((TABLE_Z, bottom), (TABLE_Z + HEIGHT, top)):
        closed = d == FOOT
        pts = np.round(ring(xy, z, d) * 4).astype(np.int32)
        cv2.polylines(image, [pts], True, (200, 200, 200) if closed else (230, 230, 230),
                      3 if closed else 2, cv2.LINE_AA, shift=2)
    cv2.polylines(image, [np.round(hull * 4).astype(np.int32)], True, (150, 150, 150), 1,
                  cv2.LINE_AA, shift=2)
    return image


def shadow(image, polygon_px, ratio):
    inside = np.zeros(image.shape[:2], np.float32)
    cv2.fillPoly(inside, [np.asarray(polygon_px, np.int32)], 1.0)
    inside = cv2.GaussianBlur(inside, (0, 0), 8)   # soft edge
    return image * (1 - (1 - ratio) * inside[..., None])


def arm(image):
    """A robot arm reaching in from the right edge: big, grey-orange, touching the border."""
    cv2.rectangle(image, (1000, 250), (1279, 400), (60, 120, 180), -1)
    return image


def photo(scene, gain=1.0, seed=1, noise=2.0):
    """A camera frame of `scene` (float image): exposure gain and sensor noise."""
    rng = np.random.default_rng(seed)
    image = scene * gain + noise * rng.standard_normal(scene.shape, dtype=np.float32)
    return np.clip(image, 0, 255).astype(np.uint8)


def frames(scene, n=10, **kwargs):
    """read() for TableBackground.capture: n noisy frames of the scene."""
    seeds = iter(range(100, 100 + n))
    return lambda: photo(scene, seed=next(seeds), **kwargs)
