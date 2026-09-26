"""
Calibrate the webcam and write camera_calibration.npz for follow_april_tag.py.

Uses a ChArUco board (a chessboard with ArUco markers in the white squares),
which works even when only part of the board is visible, so it's easy to
cover the corners of the image where lens distortion is strongest.

Steps:
  1. Use the printed calibrx 7x5 ChArUco board (see BOARD_* below), or
     python calibrate_camera.py --print
       writes charuco_board.png with the same layout; print it and
       glue/tape it flat onto something stiff (cardboard, clipboard).
  2. python calibrate_camera.py [board options]
       live view; detected corners are drawn on the board.
       SPACE  save the current view (needs at least MIN_CORNERS corners)
       c      calibrate from the saved views and write camera_calibration.npz
       q/Esc  quit
     Take ~20 views: board near and far, tilted in different directions
     (~30-45 deg), and in every part of the image, especially the corners.
     Hold still when pressing SPACE (motion blur hurts accuracy).
  3. Check the printed reprojection error: below ~0.5 px is good, above
     ~1 px means some views were blurry or the board isn't flat.

Board options, for a board printed at a different scale (all in mm, measured
on your printout with a ruler):
  --square 21.3        size of one chessboard square
  --marker 15.3        size of one marker (default: square x 18/25, the ratio
                       of the calibrx board, which printing scale doesn't change)
  --scale-bar 42.6     length of the "50 mm" bar printed above the calibrx
                       board; square and marker are scaled from it
  --cols 7 --rows 5    number of squares across / down
Tip: measure across several squares and divide, e.g. 7 squares = 149.1 mm
-> --square 21.3; that's more precise than measuring one square.

Uses the same CAMERA_INDEX / FRAME_WIDTH / FRAME_HEIGHT as follow_april_tag.py;
the calibration is only valid for that resolution.
"""

import argparse
import os

import cv2
import numpy as np

from follow_april_tag import CALIBRATION_FILE, CAMERA_INDEX, FRAME_HEIGHT, FRAME_WIDTH

# Default board layout, override with command line options (see above).
# Printed board: calibrx-charuco-175x125mm.svg (calibrx.io), measured from the
# SVG: 7 x 5 squares of 25.00 mm (175 x 125 mm, black square top-left),
# 18.00 mm DICT_6X6 markers (ids 0-16) centered in the white squares.
BOARD_COLS = 7
BOARD_ROWS = 5
SQUARE_SIZE = 25.0       # mm
MARKER_SIZE = 18.0       # mm
SCALE_BAR = 50.0         # mm, nominal length of the scale bar on the calibrx print
# Different family than the AprilTags, so the board can't be mistaken for a tag
BOARD_DICTIONARY = cv2.aruco.DICT_6X6_250

PRINT_DPI = 300
BOARD_IMAGE = os.path.join(os.path.dirname(__file__), "charuco_board.png")

MIN_CORNERS = 8          # per view
MIN_VIEWS = 10

WINDOW = "camera calibration"


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate the webcam with a ChArUco board.")
    parser.add_argument("--print", action="store_true", help="write charuco_board.png and exit")
    parser.add_argument("--cols", type=int, default=BOARD_COLS, help="squares across")
    parser.add_argument("--rows", type=int, default=BOARD_ROWS, help="squares down")
    parser.add_argument("--square", type=float, help="measured square size in mm")
    parser.add_argument("--marker", type=float, help="measured marker size in mm")
    parser.add_argument("--scale-bar", type=float,
                        help=f"measured length in mm of the {SCALE_BAR:.0f} mm scale bar")
    args = parser.parse_args()

    if args.scale_bar is not None and args.square is not None:
        parser.error("use either --scale-bar or --square, not both")
    if args.scale_bar is not None:
        scale = args.scale_bar / SCALE_BAR
        args.square = SQUARE_SIZE * scale
        if args.marker is None:
            args.marker = MARKER_SIZE * scale
    if args.square is None:
        args.square = SQUARE_SIZE
    if args.marker is None:
        # Printing scales square and marker together, so keep the ratio
        args.marker = args.square * MARKER_SIZE / SQUARE_SIZE
    if not 0 < args.marker < args.square:
        parser.error("marker must be smaller than square")
    return args


def make_board(args):
    dictionary = cv2.aruco.getPredefinedDictionary(BOARD_DICTIONARY)
    # OpenCV wants meters
    return cv2.aruco.CharucoBoard((args.cols, args.rows), args.square / 1000,
                                  args.marker / 1000, dictionary)


def print_board(args):
    px_per_m = PRINT_DPI / 0.0254
    square_px = round(args.square / 1000 * px_per_m)
    image = make_board(args).generateImage((args.cols * square_px, args.rows * square_px))
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
    args = parse_args()
    if args.print:
        print_board(args)
        return

    print(f"Board: {args.cols} x {args.rows} squares, square {args.square:.2f} mm, "
          f"marker {args.marker:.2f} mm")
    board = make_board(args)
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
