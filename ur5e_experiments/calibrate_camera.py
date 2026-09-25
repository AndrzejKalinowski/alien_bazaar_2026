"""
Calibrate the webcam and write camera_calibration.npz for follow_april_tag.py.

Uses a ChArUco board (a chessboard with ArUco markers in the white squares),
which works even when only part of the board is visible, so it's easy to
cover the corners of the image where lens distortion is strongest.

Steps:
  1. Use the printed 5x7 board (DICT_6X6 markers, ids 0-16), or
     python calibrate_camera.py --print
       writes charuco_board.png with the same layout; print it and
       glue/tape it flat onto something stiff (cardboard, clipboard).
  2. python calibrate_camera.py
       live view; detected corners are drawn on the board.
       SPACE  save the current view (needs at least MIN_CORNERS corners)
       c      calibrate from the saved views and write camera_calibration.npz
       q/Esc  quit
     Take ~20 views: board near and far, tilted in different directions
     (~30-45 deg), and in every part of the image, especially the corners.
     Hold still when pressing SPACE (motion blur hurts accuracy).
  3. Check the printed reprojection error: below ~0.5 px is good, above
     ~1 px means some views were blurry or the board isn't flat.

Uses the same CAMERA_INDEX / FRAME_WIDTH / FRAME_HEIGHT as follow_april_tag.py;
the calibration is only valid for that resolution.
"""

import os
import sys

import cv2
import numpy as np

from follow_april_tag import CALIBRATION_FILE, CAMERA_INDEX, FRAME_HEIGHT, FRAME_WIDTH

# Board layout. The absolute square size doesn't affect the intrinsics, only
# the marker/square ratio has to match the printed board (it does unless you
# print it stretched).
BOARD_COLS = 5
BOARD_ROWS = 7
SQUARE_SIZE = 0.030      # m
MARKER_SIZE = 0.018      # m, markers are 0.6 x the square size on the printed board
# Different family than the AprilTags, so the board can't be mistaken for a tag
BOARD_DICTIONARY = cv2.aruco.DICT_6X6_250

PRINT_DPI = 300
BOARD_IMAGE = os.path.join(os.path.dirname(__file__), "charuco_board.png")

MIN_CORNERS = 8          # per view
MIN_VIEWS = 10

WINDOW = "camera calibration"


def make_board():
    dictionary = cv2.aruco.getPredefinedDictionary(BOARD_DICTIONARY)
    return cv2.aruco.CharucoBoard((BOARD_COLS, BOARD_ROWS), SQUARE_SIZE, MARKER_SIZE, dictionary)


def print_board():
    px_per_m = PRINT_DPI / 0.0254
    square_px = round(SQUARE_SIZE * px_per_m)
    image = make_board().generateImage((BOARD_COLS * square_px, BOARD_ROWS * square_px))
    margin = round(0.01 * px_per_m)   # 1 cm white border
    image = cv2.copyMakeBorder(image, margin, margin, margin, margin,
                               cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(BOARD_IMAGE, image)
    print(f"Wrote {BOARD_IMAGE} ({PRINT_DPI} dpi, "
          f"{image.shape[1] / px_per_m * 1000:.0f} x {image.shape[0] / px_per_m * 1000:.0f} mm). "
          "Print it at 100% scale.")


def calibrate(board, views, image_size):
    object_points, image_points = [], []
    for corners, ids in views:
        obj, img = board.matchImagePoints(corners, ids)
        object_points.append(obj)
        image_points.append(img)

    error, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)

    print(f"\nCalibrated from {len(views)} views, image size {image_size[0]}x{image_size[1]}")
    print(f"Reprojection error: {error:.3f} px "
          f"({'good' if error < 0.5 else 'ok' if error < 1.0 else 'poor, retake views'})")
    print("Camera matrix:\n", np.round(camera_matrix, 1))
    print("Distortion coefficients:", np.round(dist_coeffs.ravel(), 4))
    hfov = np.degrees(2 * np.arctan(image_size[0] / (2 * camera_matrix[0, 0])))
    print(f"Horizontal field of view: {hfov:.1f} deg")

    np.savez(CALIBRATION_FILE, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
             image_size=np.array(image_size))
    print(f"Saved {CALIBRATION_FILE}")
    return error


def main():
    if "--print" in sys.argv:
        print_board()
        return

    board = make_board()
    detector = cv2.aruco.CharucoDetector(board)

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {CAMERA_INDEX}")

    views = []
    # Where saved views had corners, so you can see which parts of the image are covered
    coverage = None
    status = "SPACE: save view   c: calibrate   q: quit"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            if coverage is None:
                coverage = np.zeros_like(frame)
            image_size = (frame.shape[1], frame.shape[0])

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
            n = 0 if ids is None else len(ids)
            if n > 0:
                # OpenCV 5 returns (N, 2) corners, but drawDetectedCornersCharuco
                # asserts on anything other than one point per id, i.e. (N, 1, 2)
                corners = corners.reshape(-1, 1, 2)

            display = cv2.addWeighted(frame, 1.0, coverage, 0.5, 0)
            if n > 0:
                cv2.aruco.drawDetectedCornersCharuco(display, corners, ids, (0, 255, 0))
            color = (0, 255, 0) if n >= MIN_CORNERS else (0, 0, 255)
            cv2.putText(display, f"corners: {n}   views: {len(views)}/{MIN_VIEWS}+",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display, status, (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(WINDOW, display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                if n >= MIN_CORNERS:
                    views.append((corners, ids))
                    for x, y in corners.reshape(-1, 2).astype(int):
                        cv2.circle(coverage, (x, y), 6, (255, 128, 0), -1)
                    status = f"saved view {len(views)}"
                else:
                    status = f"need at least {MIN_CORNERS} corners"
            elif key == ord("c"):
                if len(views) < MIN_VIEWS:
                    status = f"need at least {MIN_VIEWS} views"
                else:
                    error = calibrate(board, views, image_size)
                    status = f"saved, error {error:.2f} px (q to quit)"
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
