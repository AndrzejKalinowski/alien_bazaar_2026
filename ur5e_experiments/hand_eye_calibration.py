"""
Measure where the camera is on the tool (hand-eye calibration), automatically.

The robot moves through a set of small poses around its start pose while
looking at an AprilTag lying still on the table. At every pose it records the
robot TCP pose and the tag pose seen by the camera. cv2.calibrateHandEye then
solves for the camera pose in the tool frame (full rotation, including tilt,
plus offset) and writes hand_eye.npz, which follow_april_tag.py loads instead of
CAMERA_OFFSET / CAMERA_YAW_DEG.

Before running:
  * Calibrate the camera first (calibrate_camera.py); a wrong lens model gives
    a wrong hand-eye result.
  * TAG_SIZE in follow_april_tag.py must be the exact printed tag size.
  * TCP on the pendant set to the suction cup tip (the result is relative to it).
  * Put a tag flat on the table, jog the robot so the tag is roughly in the
    middle of the image, 20-35 cm below the camera, tool pointing down.
  * Keep the area clear: the robot tilts up to ~12 deg and shifts a few cm.

Keys: SPACE start, q/Esc abort.
"""

import time

import cv2
import numpy as np
import rtde_control
import rtde_receive

from follow_april_tag import HAND_EYE_FILE, IP, TAG_SIZE, Camera, TagDetector, pose_to_matrix

MOVE_SPEED = 0.05        # m/s
MOVE_ACCEL = 0.2         # m/s^2
SETTLE_TIME = 0.7        # s, wait after each move before measuring
MEASURE_FRAMES = 15      # tag detections averaged per pose
MEASURE_TIMEOUT = 3.0    # s
MIN_GOOD_POSES = 8

# Pose offsets in the tool frame: [dx, dy, dz] in m, [rx, ry, rz] in degrees.
# Rotations pivot around a point on the tool Z axis at the tag's distance, so
# the tag stays in view. Hand-eye needs rotations about different axes.
POSE_OFFSETS = [
    ([0, 0, 0], [0, 0, 0]),
    ([0.03, 0, 0], [0, 0, 0]),
    ([0, 0.03, 0], [0, 0, 0]),
    ([0, 0, -0.04], [0, 0, 0]),
    ([0, 0, 0], [10, 0, 0]),
    ([0, 0, 0], [-10, 0, 0]),
    ([0, 0, 0], [0, 10, 0]),
    ([0, 0, 0], [0, -10, 0]),
    ([0, 0, 0], [0, 0, 25]),
    ([0, 0, 0], [0, 0, -25]),
    ([0, 0, 0], [8, 8, 15]),
    ([0, 0, 0], [-8, 8, -15]),
    ([0, 0, 0], [8, -8, -15]),
    ([0, 0, 0], [-8, -8, 15]),
    ([0.02, -0.02, -0.03], [12, 0, 10]),
    ([-0.02, 0.02, -0.03], [0, 12, -10]),
    ([0, 0, 0], [0, 0, 0]),
]

METHODS = {
    "Tsai": cv2.CALIB_HAND_EYE_TSAI,
    "Park": cv2.CALIB_HAND_EYE_PARK,
    "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

WINDOW = "hand-eye calibration"


def matrix_to_pose(T):
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return list(T[:3, 3]) + list(rvec.ravel())


def offset_matrix(translation, rotation_deg, pivot_dist):
    """Tool-frame offset: rotate about a pivot on tool Z, then translate."""
    R, _ = cv2.Rodrigues(np.radians(rotation_deg).astype(float))
    pivot = np.array([0, 0, pivot_dist])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pivot - R @ pivot + translation
    return T


def average_rotation(rotations):
    U, _, Vt = np.linalg.svd(np.sum(rotations, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def show(camera, detector, text, tag_id=None):
    frame, _ = camera.read()
    if frame is None:
        return -1
    for tid, corners, T_cam_tag in detector.detect(frame):
        color = (0, 255, 0) if tag_id in (None, tid) else (0, 180, 255)
        cv2.polylines(frame, [corners.astype(int)], True, color, 2)
        rvec, _ = cv2.Rodrigues(T_cam_tag[:3, :3])
        cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec,
                          T_cam_tag[:3, 3], TAG_SIZE * 0.75, 2)
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow(WINDOW, frame)
    return cv2.waitKey(1) & 0xFF


def measure(camera, detector, tag_id):
    """Averaged T_cam_tag of tag_id over several fresh frames, or None."""
    stamp = time.time()
    rotations, translations = [], []
    deadline = time.time() + MEASURE_TIMEOUT
    while len(translations) < MEASURE_FRAMES and time.time() < deadline:
        frame, stamp = camera.read(newer_than=stamp)
        if frame is None:
            continue
        for tid, _, T_cam_tag in detector.detect(frame):
            if tid == tag_id:
                rotations.append(T_cam_tag[:3, :3])
                translations.append(T_cam_tag[:3, 3])
    if len(translations) < MEASURE_FRAMES // 2:
        return None
    T = np.eye(4)
    T[:3, :3] = average_rotation(rotations)
    T[:3, 3] = np.median(translations, axis=0)
    return T


def move_and_watch(c, camera, detector, target, text, tag_id):
    """Asynchronous moveL, keeping the video live. Returns False if aborted."""
    c.moveL(target, MOVE_SPEED, MOVE_ACCEL, True)
    start = time.time()
    while True:
        if show(camera, detector, text, tag_id) in (ord("q"), 27):
            c.stopL(1.0)
            return False
        if time.time() - start > 0.2 and c.getAsyncOperationProgress() < 0:
            return True


def park_martin(samples):
    """Solve AX = XB (Park & Martin 1994) over all pairs of poses.

    A = robot motion between two poses (in the tool frame), B = the same motion
    seen by the camera, X = camera pose in the tool frame.
    """
    tcp = [pose_to_matrix(p) for p, _ in samples]
    cam = [T for _, T in samples]
    pairs = []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            A = np.linalg.inv(tcp[i]) @ tcp[j]
            B = cam[i] @ np.linalg.inv(cam[j])
            pairs.append((A, B))

    # Rotation: align the rotation axes of the A and B motions
    M = np.zeros((3, 3))
    for A, B in pairs:
        alpha = cv2.Rodrigues(A[:3, :3])[0].ravel()
        beta = cv2.Rodrigues(B[:3, :3])[0].ravel()
        M += np.outer(beta, alpha)
    w, V = np.linalg.eigh(M.T @ M)
    R = V @ np.diag(1 / np.sqrt(w)) @ V.T @ M.T

    # Translation: least squares of (R_A - I) t = R t_B - t_A
    C = np.vstack([A[:3, :3] - np.eye(3) for A, _ in pairs])
    d = np.concatenate([R @ B[:3, 3] - A[:3, 3] for A, B in pairs])
    t = np.linalg.lstsq(C, d, rcond=None)[0]

    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = t
    return X


def solve(samples):
    candidates = [("Park-Martin", park_martin(samples))]
    # OpenCV's solvers too, when this OpenCV build has them (5.0 doesn't)
    if hasattr(cv2, "calibrateHandEye"):
        R_g2b = [pose_to_matrix(p)[:3, :3] for p, _ in samples]
        t_g2b = [pose_to_matrix(p)[:3, 3] for p, _ in samples]
        R_t2c = [T[:3, :3] for _, T in samples]
        t_t2c = [T[:3, 3] for _, T in samples]
        for name, method in METHODS.items():
            try:
                R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
            except cv2.error as e:
                print(f"{name}: failed ({e})")
                continue
            X = np.eye(4)
            X[:3, :3] = R
            X[:3, 3] = t.ravel()
            candidates.append((name, X))

    results = []
    for name, X in candidates:
        # The tag didn't move, so with the right X every pose must put it at
        # the same place in the base frame. The spread is the error.
        tag_in_base = np.array([(pose_to_matrix(p) @ X @ T)[:3, 3] for p, T in samples])
        spread = np.linalg.norm(tag_in_base - tag_in_base.mean(axis=0), axis=1)
        print(f"{name:10s}: tag position spread {spread.mean() * 1000:.1f} mm mean, "
              f"{spread.max() * 1000:.1f} mm max")
        results.append((spread.mean(), name, X))
    return min(results, key=lambda r: r[0]) if results else None


def describe(X):
    t = X[:3, 3] * 1000
    R = X[:3, :3]
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
    print(f"Camera offset in tool frame: x {t[0]:.1f}  y {t[1]:.1f}  z {t[2]:.1f} mm")
    print(f"Camera yaw around tool Z: {yaw:.1f} deg, tilt from tool Z: {tilt:.1f} deg")


def main():
    camera = Camera()
    detector = TagDetector(camera.camera_matrix, camera.dist_coeffs)
    r = rtde_receive.RTDEReceiveInterface(IP)
    c = rtde_control.RTDEControlInterface(IP)

    try:
        # Wait for the user to confirm, with a tag in view
        while True:
            frame, _ = camera.read()
            tags = detector.detect(frame) if frame is not None else []
            key = show(camera, detector,
                       "SPACE: start (robot will move!)   q: quit" if tags else "no tag in view")
            if key in (ord("q"), 27):
                return
            if key == ord(" ") and tags:
                break

        tag_id = detector.choose(tags, frame.shape)[0]
        first = measure(camera, detector, tag_id)
        if first is None:
            print("Could not measure the tag, aborting")
            return
        pivot_dist = first[2, 3]
        print(f"Using tag {tag_id}, {pivot_dist * 100:.0f} cm from the camera")

        start_pose = r.getActualTCPPose()
        T_start = pose_to_matrix(start_pose)
        samples = []

        for i, (translation, rotation) in enumerate(POSE_OFFSETS):
            target = matrix_to_pose(T_start @ offset_matrix(translation, rotation, pivot_dist))
            text = f"pose {i + 1}/{len(POSE_OFFSETS)}   good: {len(samples)}   q: abort"
            if not move_and_watch(c, camera, detector, target, text, tag_id):
                print("Aborted")
                return
            time.sleep(SETTLE_TIME)
            T_cam_tag = measure(camera, detector, tag_id)
            if T_cam_tag is None:
                print(f"Pose {i + 1}: tag not visible, skipped")
                continue
            samples.append((r.getActualTCPPose(), T_cam_tag))
            print(f"Pose {i + 1}: ok")

        print("Returning to start pose")
        move_and_watch(c, camera, detector, start_pose, "returning to start", tag_id)

        if len(samples) < MIN_GOOD_POSES:
            print(f"Only {len(samples)} good poses (need {MIN_GOOD_POSES}), "
                  "move the tag closer to the image center and retry")
            return

        print(f"\nSolving hand-eye from {len(samples)} poses:")
        best = solve(samples)
        if best is None:
            return
        spread, name, X = best
        print(f"\nBest: {name}")
        describe(X)
        if spread > 0.005:
            print("WARNING: spread above 5 mm; check camera calibration, TAG_SIZE, "
                  "and that the camera and tag didn't move")
        np.savez(HAND_EYE_FILE, T_tcp_cam=X)
        print(f"Saved {HAND_EYE_FILE}")
    finally:
        try:
            c.stopScript()
        except Exception:
            pass
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
