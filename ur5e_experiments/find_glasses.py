"""
Find glasses (circles) with a fixed overhead camera and report their position
in the robot base frame.

How it works:
  1. The frame is undistorted with the overhead camera's intrinsics
     (overhead_camera_calibration.npz, made with calibrate_camera.py).
  2. Circles are found with cv2.HoughCircles (HOUGH_GRADIENT_ALT). The allowed
     radius range in pixels is computed from GLASS_MIN/MAX_DIAMETER and the
     camera's distance to the rim plane, so only glass-sized circles are kept.
     Circles that are too colourful or too dark along their rim are rejected
     (glass is colourless and its rim shows as a bright ring), see "Colour
     filter" below. Then smaller circles inside a bigger one (the glass bottom
     seen through the glass, reflections) are dropped.
  3. Each circle center is turned into a viewing ray and intersected with the
     horizontal plane z = table_z + RIM_HEIGHT in the base frame. For an
     upright glass that point is on the glass axis, so its x, y is where the
     glass stands. Upside-down glasses work the same way (the foot is the
     circle then), as long as RIM_HEIGHT is the height of the circle you see.

  RIM_HEIGHT matters: a wrong height shifts x, y away from the point under the
  camera by  (distance from that point) * (height error) / (camera height).
  E.g. camera 1 m up, glass 40 cm off-center, 1 cm height error -> 4 mm error.

Calibration (where the camera is relative to the robot):
  python find_glasses.py --calibrate
  Uses AprilTags (36h11, any size) lying in the work area as reference points:
    1. Put 1..8 tags in the work area, spread out (corners of the area and the
       middle). A few raised on blocks of different heights improve accuracy.
       Move the robot out of the camera view.
    2. SPACE: the tag centers are measured in the image (averaged).
    3. The program asks for the tags one by one: put the suction cup tip (the
       TCP) on the center of that tag and press SPACE. Drive the robot there
       with the gamepad (see gamepad_jog.py, hold LB for fine moves) or use
       freedrive (f). n skips a tag.
    Gamepad buttons: A = SPACE, X = n (skip), B = f (freedrive), Y = c (solve).
    4. Repeat 1-3 (move the tags to new places) until you have >= 6 points,
       better 10-15 covering the whole area. Then press c: the camera pose is
       solved with solvePnP and saved to overhead_camera_pose.npz, with the
       per-point error in mm. Aim for < 2-3 mm.
  The lowest touched point is taken as the table height (override TABLE_Z).
  Progress (touched points and measured tags still to touch) is saved to
  overhead_calibration_points.json after every step, so if the robot or the
  script restarts, running --calibrate again continues where it stopped.
  --fresh starts over (the old file is kept as .bak). The file is plain JSON,
  delete a bad point from it by hand if needed.

Detection mode:
  python find_glasses.py [--robot]
    p      measure glasses over several frames and print their base positions
    m      (--robot) move the suction tip HOVER_CHECK above the glass nearest
           to the image center, to check the calibration
    s      stop robot motion
    q/Esc  quit
  With --robot the gamepad jogs the robot too; buttons: X = p, Y = m, A = s,
  B = t (add the tip position as an area corner).
  Keys that can't run (m / s / t without --robot, m with no glass found, c
  with bad points) are refused with a WARNING; the program keeps running.
  The trackbars tune edge threshold and roundness for your lighting. All
  trackbar values are saved to detection_settings.json whenever they change
  and loaded on the next start (also by GlassFinder in other scripts); delete
  the file to go back to the defaults below.
  With --robot (and in --calibrate) the robot_watchdog.py motion watchdog is
  on: the robot stops if the loop stalls for 0.2 s, and read_frame() raises
  after FRAME_TIMEOUT without a camera frame.

Colour filter:
  For every circle a thin ring of pixels along its edge is converted to HSV.
  The saturation (0 = gray/white/clear, 255 = strong colour) and brightness
  reached by the top RIM_FRACTION of that ring are compared with the "max
  saturation" and "min brightness" trackbars, so a coloured rim counts even if
  it is thin. Each circle shows its values as "S.. V..": glasses in green,
  rejected circles in red with the reason. Set max saturation a bit above what
  your glasses show and below the other objects. The ring also sees the table
  through and around the glass, so use a gray/black (unsaturated) mat.

Detection area:
  Only glasses standing inside this polygon on the table are reported; outside
  it the image is dimmed, and its corners are labelled with base x, y in mm.
  Edit it with the mouse at any time, every change is saved immediately to
  detection_area.json (in base coordinates, so it stays valid when the camera
  is recalibrated):
    drag on empty space      draw a new rectangle (replaces the area)
    drag a corner            move it
    click on an edge         add a corner there (and drag it right away)
    right click a corner     delete it
    t / B                    (--robot) add the suction tip's x, y as a corner;
                             jog the tip to each table corner for exact edges
    u                        undo the last change
    x                        clear the area (whole image)
  The map in the top right corner shows the same from above in base
  coordinates: robot base (+X red, +Y green), camera view on the table (gray),
  detection area (magenta), glasses (green) and, with --robot, the tip (yellow).

For use from other scripts:
  finder = GlassFinder.load()
  glasses = finder.measure(cap)   # list of Glass(x, y, z, diameter, pixel)

Tips: glasses are transparent, so give them contrast: a dark matte mat on the
table and diffuse light from above/side makes the rims show as bright rings.
Mount the camera looking straight down, above the middle of the work area.

Requires: pip install opencv-python ur_rtde
"""

import argparse
import json
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np

from follow_april_tag import IP, MAX_TCP_Z, MIN_TCP_Z, TAG_DICTIONARY, pose_to_matrix
from gamepad_jog import GamepadControl, Jogger
from robot_watchdog import RobotWatchdog

# --- camera -------------------------------------------------------------------

OVERHEAD_CAMERA_INDEX = 2
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_HFOV_DEG = 70.0   # only used when there is no intrinsics file
FRAME_TIMEOUT = 0.5      # s without a frame -> error, so the finally blocks stop the robot
HERE = os.path.dirname(__file__)
INTRINSICS_FILE = os.path.join(HERE, "overhead_camera_calibration.npz")
POSE_FILE = os.path.join(HERE, "overhead_camera_pose.npz")
PROGRESS_FILE = os.path.join(HERE, "overhead_calibration_points.json")
AREA_FILE = os.path.join(HERE, "detection_area.json")
SETTINGS_FILE = os.path.join(HERE, "detection_settings.json")

# --- glasses ------------------------------------------------------------------

TABLE_Z = None               # m in base frame, None = take it from the calibration
RIM_HEIGHT = 0.075            # m above the table of the circle seen from above
GLASS_MIN_DIAMETER = 0.05    # m
GLASS_MAX_DIAMETER = 0.10    # m
RADIUS_MARGIN = 0.15         # widen the pixel radius range by this fraction

EDGE_THRESHOLD = 150         # Canny high threshold for HoughCircles, trackbar
ROUNDNESS = 80               # %, how perfect a circle must be (param2 of HOUGH_GRADIENT_ALT), trackbar
BLUR = 5                     # px, median blur before detection (odd)

MAX_SATURATION = 60          # 0-255, reject circles more colourful than this along the rim, trackbar
MIN_BRIGHTNESS = 0           # 0-255, reject circles darker than this along the rim (0 = off), trackbar
RING_WIDTH = 0.12            # colour is sampled in the ring r * (1 +- RING_WIDTH)
RIM_FRACTION = 0.25          # the rim must cover this fraction of that ring to count

MEASURE_FRAMES = 15          # frames averaged by measure()
MIN_SEEN_FRACTION = 0.5      # a glass must be found in this fraction of those frames

# --- calibration ------------------------------------------------------------

CALIB_FRAMES = 20            # frames averaged per tag measurement
MIN_POINTS = 6

# --- robot check move ---------------------------------------------------------

HOVER_CHECK = 0.05           # m, tip height above the rim for the check move
CHECK_SPEED = 0.1            # m/s
CHECK_ACCEL = 0.3

WINDOW = "find glasses"
MAP_SIZE = 300               # px, top-down map in the corner of the video
GRAB_DISTANCE = 12           # px, how close to a corner / edge the mouse must be to grab it
MIN_RECT_DRAG = 15           # px, shorter drags on empty space are ignored (plain clicks)

# Gamepad button -> the same action as this keyboard key
CALIBRATE_GAMEPAD_KEYS = {"BTN_SOUTH": " ", "BTN_WEST": "n", "BTN_EAST": "f", "BTN_NORTH": "c"}
RUN_GAMEPAD_KEYS = {"BTN_WEST": "p", "BTN_NORTH": "m", "BTN_SOUTH": "s", "BTN_EAST": "t"}


def window_closed():
    """True once the window was closed with its X button (the trackbars are gone then)."""
    return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1


def warn(message):
    """A refused operator command: printed and shown in the window, the loop goes on."""
    print(f"WARNING: {message}")
    return message


def read_keys(gamepad):
    """Keys pressed since the last call, from the gamepad and the OpenCV window."""
    keys = gamepad.poll_keys() if gamepad else []
    key = cv2.waitKey(1) & 0xFF
    if key != 0xFF:
        keys.append(chr(key))
    return keys


# --- geometry -------------------------------------------------------------------

def load_intrinsics(width, height):
    if not os.path.exists(INTRINSICS_FILE):
        print(f"WARNING: no {INTRINSICS_FILE}, guessing intrinsics from {CAMERA_HFOV_DEG} deg HFOV.\n"
              f"  Run: python calibrate_camera.py --camera {OVERHEAD_CAMERA_INDEX} "
              f"--output {os.path.basename(INTRINSICS_FILE)}")
        f = (width / 2) / np.tan(np.radians(CAMERA_HFOV_DEG) / 2)
        return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]]), np.zeros(5)

    data = np.load(INTRINSICS_FILE)
    camera_matrix = data["camera_matrix"].copy()
    calib_w, calib_h = data["image_size"]
    if (calib_w, calib_h) != (width, height):
        print(f"WARNING: intrinsics calibrated at {calib_w}x{calib_h} but camera gives "
              f"{width}x{height}, scaling. Recalibrate for best accuracy.")
        camera_matrix[0] *= width / calib_w
        camera_matrix[1] *= height / calib_h
    return camera_matrix, data["dist_coeffs"]


class Undistorter:
    """Removes lens distortion; afterwards the pinhole model with K holds exactly."""

    def __init__(self, width, height):
        self.K, dist = load_intrinsics(width, height)
        self._maps = cv2.initUndistortRectifyMap(self.K, dist, None, self.K,
                                                 (width, height), cv2.CV_16SC2)

    def __call__(self, frame):
        return cv2.remap(frame, *self._maps, cv2.INTER_LINEAR)


def pixel_to_plane(K, T_base_cam, pixel, plane_z):
    """Intersect the viewing ray through `pixel` with the plane z = plane_z (base frame)."""
    ray_cam = np.linalg.solve(K, [pixel[0], pixel[1], 1.0])
    ray = T_base_cam[:3, :3] @ ray_cam
    origin = T_base_cam[:3, 3]
    if abs(ray[2]) < 1e-9:
        return None
    s = (plane_z - origin[2]) / ray[2]
    return origin + s * ray if s > 0 else None


def open_camera():
    cap = cv2.VideoCapture(OVERHEAD_CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {OVERHEAD_CAMERA_INDEX}")
    for _ in range(100):
        ok, frame = cap.read()
        if ok:
            return cap, frame.shape[1], frame.shape[0]
    raise RuntimeError("Camera gives no frames")


def read_frame(cap, timeout=FRAME_TIMEOUT):
    # Looping forever here would keep a running speedL going (unplugged camera)
    deadline = time.time() + timeout
    while True:
        ok, frame = cap.read()
        if ok:
            return frame
        if time.time() > deadline:
            raise RuntimeError(f"no frame from camera {OVERHEAD_CAMERA_INDEX} for {timeout} s")


def draw_base_axes(image, K, T_base_cam, origin_z, length=0.1):
    """Draw the robot base X (red) / Y (green) axes on the table, as a sanity check."""
    T_cam_base = np.linalg.inv(T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    points = np.array([[0, 0, origin_z], [length, 0, origin_z], [0, length, origin_z]])
    pixels, _ = cv2.projectPoints(points, rvec, T_cam_base[:3, 3], K, None)
    o, x, y = pixels.reshape(-1, 2).astype(int)
    cv2.arrowedLine(image, o, x, (0, 0, 255), 2)
    cv2.arrowedLine(image, o, y, (0, 255, 0), 2)
    cv2.putText(image, "base", o + 5, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


# --- glass detection ------------------------------------------------------------

def ring_color(hsv, u, v, r):
    """Saturation and brightness of the rim along the circle edge.

    Takes the (1 - RIM_FRACTION) quantile over a thin ring, so the rim counts as
    soon as it covers RIM_FRACTION of the ring: a thin coloured rim on a gray
    table is coloured, and a bright glass rim is bright.
    """
    h, w = hsv.shape[:2]
    outer = r * (1 + RING_WIDTH)
    x0, x1 = max(int(u - outer), 0), min(int(u + outer) + 1, w)
    y0, y1 = max(int(v - outer), 0), min(int(v + outer) + 1, h)
    if x1 <= x0 or y1 <= y0:
        return 255.0, 0.0
    yy, xx = np.mgrid[y0:y1, x0:x1]
    d = np.hypot(xx - u, yy - v)
    ring = hsv[y0:y1, x0:x1][(d >= r * (1 - RING_WIDTH)) & (d <= outer)]
    if len(ring) == 0:
        return 255.0, 0.0
    return tuple(float(np.quantile(ring[:, i], 1 - RIM_FRACTION)) for i in (1, 2))


@dataclass
class Glass:
    x: float          # m, base frame, glass axis
    y: float
    z: float          # m, height of the detected rim
    diameter: float   # m, measured
    pixel: tuple      # (u, v, r) in the undistorted image
    saturation: float = 0.0   # median HSV saturation along the rim, 0-255
    brightness: float = 0.0   # median HSV value along the rim, 0-255


class GlassFinder:
    def __init__(self, K, T_base_cam, table_z, width, height):
        self.K = K
        self.T_base_cam = T_base_cam
        self.table_z = table_z
        self.rim_z = table_z + RIM_HEIGHT
        self.undistort = Undistorter(width, height)
        settings = load_settings()
        self.edge_threshold = settings["edge"]
        self.roundness = settings["roundness %"] / 100
        self.max_saturation = settings["max saturation"]
        self.min_brightness = settings["min brightness"]
        self.rejected = []   # (u, v, r, reason) from the last detect(), for display
        self.area = load_area()   # (N, 2) base x, y polygon, or None = everywhere

        # Expected radius in pixels from the depth of the rim plane at the image center
        center = pixel_to_plane(K, T_base_cam, (width / 2, height / 2), self.rim_z)
        if center is None:
            raise RuntimeError("Rim plane not in front of the camera, check the calibration")
        depth = (np.linalg.inv(T_base_cam) @ np.append(center, 1))[2]
        f = K[0, 0]
        # Off-center rims are farther away and look smaller, hence the margin
        self.min_radius = int(f * GLASS_MIN_DIAMETER / 2 / depth * (1 - RADIUS_MARGIN))
        self.max_radius = int(np.ceil(f * GLASS_MAX_DIAMETER / 2 / depth * (1 + RADIUS_MARGIN)))
        print(f"Camera {depth:.2f} m above the rims, glass radius {self.min_radius}-{self.max_radius} px")

    @classmethod
    def load(cls, width=FRAME_WIDTH, height=FRAME_HEIGHT):
        if not os.path.exists(POSE_FILE):
            raise RuntimeError(f"No {POSE_FILE}, run: python find_glasses.py --calibrate")
        data = np.load(POSE_FILE)
        if tuple(data["image_size"]) != (width, height):
            raise RuntimeError(f"Camera pose was calibrated at {tuple(data['image_size'])}, "
                               f"camera gives {(width, height)}")
        table_z = float(data["table_z"]) if TABLE_Z is None else TABLE_Z
        K = Undistorter(width, height).K
        return cls(K, data["T_base_cam"], table_z, width, height)

    def detect(self, undistorted):
        """Glasses in one undistorted frame."""
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, BLUR)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT_ALT, dp=1.5,
                                   minDist=self.min_radius,
                                   param1=max(self.edge_threshold, 1),
                                   param2=min(max(self.roundness, 0.05), 0.99),
                                   minRadius=self.min_radius, maxRadius=self.max_radius)
        if circles is None:
            return []

        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
        self.rejected = []
        candidates = []
        for u, v, r in circles.reshape(-1, 3):
            center = pixel_to_plane(self.K, self.T_base_cam, (u, v), self.rim_z)
            edge = pixel_to_plane(self.K, self.T_base_cam, (u + r, v), self.rim_z)
            if center is None or edge is None or not self.in_area(center[:2]):
                continue
            saturation, brightness = ring_color(hsv, u, v, r)
            reason = None
            if saturation > self.max_saturation:
                reason = f"colour S{saturation:.0f}"
            elif brightness < self.min_brightness:
                reason = f"dark V{brightness:.0f}"
            if reason:
                self.rejected.append((u, v, r, reason))
                continue
            diameter = 2 * np.linalg.norm(edge - center)
            candidates.append(Glass(center[0], center[1], center[2], diameter,
                                    (float(u), float(v), float(r)), saturation, brightness))

        # Biggest first; drop circles whose center lies inside an accepted one
        glasses = []
        for g in sorted(candidates, key=lambda g: -g.pixel[2]):
            u, v, _ = g.pixel
            if all(np.hypot(u - k.pixel[0], v - k.pixel[1]) > k.pixel[2] for k in glasses):
                glasses.append(g)
        return glasses

    def in_area(self, xy):
        if self.area is None:
            return True
        return cv2.pointPolygonTest(self.area.astype(np.float32),
                                    (float(xy[0]), float(xy[1])), False) >= 0

    def area_pixels(self, polygon=None):
        """Polygon (default: the detection area) on the table, projected into the image."""
        polygon = self.area if polygon is None else np.asarray(polygon, dtype=float)
        T_cam_base = np.linalg.inv(self.T_base_cam)
        rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
        points = np.hstack([polygon, np.full((len(polygon), 1), self.table_z)])
        pixels, _ = cv2.projectPoints(points, rvec, T_cam_base[:3, 3], self.K, None)
        return pixels.reshape(-1, 2).astype(np.int32)

    def measure(self, cap, frames=MEASURE_FRAMES, kick=None):
        """Glasses found consistently over several frames, positions averaged (median).

        kick: called after every frame (RobotWatchdog.kick), this blocks for ~0.5 s.
        """
        clusters = []   # lists of Glass belonging to the same physical glass
        for _ in range(frames):
            frame = read_frame(cap)
            if kick:
                kick()
            for g in self.detect(self.undistort(frame)):
                for cluster in clusters:
                    ref = cluster[0]
                    if np.hypot(g.x - ref.x, g.y - ref.y) < ref.diameter / 4:
                        cluster.append(g)
                        break
                else:
                    clusters.append([g])

        result = []
        for cluster in clusters:
            if len(cluster) < frames * MIN_SEEN_FRACTION:
                continue
            m = lambda attr: float(np.median([getattr(g, attr) for g in cluster]))
            pixel = tuple(np.median([g.pixel for g in cluster], axis=0))
            result.append(Glass(m("x"), m("y"), m("z"), m("diameter"), pixel,
                                m("saturation"), m("brightness")))
        return result


def load_area():
    if not os.path.exists(AREA_FILE):
        return None
    with open(AREA_FILE) as f:
        polygon = json.load(f)["polygon"]
    return np.array(polygon, dtype=float) if len(polygon) >= 3 else None


def save_area(polygon):
    with open(AREA_FILE, "w") as f:
        json.dump({"polygon": [[float(x), float(y)] for x, y in polygon]}, f, indent=1)


def point_segment_distance(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
    return np.linalg.norm(p - (a + t * ab))


class AreaEditor:
    """Edit the detection area with the mouse in the video window; every change is saved."""

    def __init__(self, finder):
        self.finder = finder
        self.points = [] if finder.area is None else [p.copy() for p in finder.area]
        self.history = []
        self.hover = None    # corner index under the mouse
        self.drag = None     # ("corner", index) or ("rect", start pixel, end pixel)

    # -- geometry --

    def _to_table(self, u, v):
        p = pixel_to_plane(self.finder.K, self.finder.T_base_cam, (u, v), self.finder.table_z)
        return None if p is None else p[:2]

    def _pixels(self):
        return self.finder.area_pixels(self.points) if self.points else np.zeros((0, 2), int)

    def _corner_at(self, u, v):
        pixels = self._pixels()
        if len(pixels) == 0:
            return None
        d = np.linalg.norm(pixels - (u, v), axis=1)
        return int(np.argmin(d)) if d.min() < GRAB_DISTANCE else None

    def _edge_at(self, u, v):
        """Index i of the edge points[i] -> points[i + 1] under the mouse, or None."""
        pixels = self._pixels().astype(float)
        if len(pixels) < 3:
            return None
        n = len(pixels)
        d = [point_segment_distance(np.array((u, v), float), pixels[i], pixels[(i + 1) % n])
             for i in range(n)]
        return int(np.argmin(d)) if min(d) < GRAB_DISTANCE else None

    # -- changes --

    def _push(self):
        self.history.append([p.copy() for p in self.points])

    def _save(self):
        if len(self.points) >= 3:
            save_area(self.points)
            self.finder.area = np.array(self.points)
        elif not self.points:
            save_area([])
            self.finder.area = None
        # 1-2 points: unfinished (added with the tip), keep the saved area for now

    def add_point(self, xy):
        """Add a corner where it lengthens the outline the least (so order doesn't matter)."""
        self._push()
        xy = np.asarray(xy, dtype=float)
        if len(self.points) < 3:
            self.points.append(xy)
        else:
            n = len(self.points)
            cost = [np.linalg.norm(xy - self.points[i]) + np.linalg.norm(xy - self.points[(i + 1) % n])
                    - np.linalg.norm(self.points[i] - self.points[(i + 1) % n]) for i in range(n)]
            self.points.insert(int(np.argmin(cost)) + 1, xy)
        self._save()

    def clear(self):
        self._push()
        self.points = []
        self._save()

    def undo(self):
        if self.history:
            self.points = self.history.pop()
            self._save()

    def on_mouse(self, event, u, v, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            if self.drag and self.drag[0] == "corner":
                p = self._to_table(u, v)
                if p is not None:
                    self.points[self.drag[1]] = p
            elif self.drag and self.drag[0] == "rect":
                self.drag = ("rect", self.drag[1], (u, v))
            else:
                self.hover = self._corner_at(u, v)
        elif event == cv2.EVENT_LBUTTONDOWN:
            i = self._corner_at(u, v)
            edge = self._edge_at(u, v) if i is None else None
            if i is not None:
                self._push()
                self.drag = ("corner", i)
            elif edge is not None:
                p = self._to_table(u, v)
                if p is not None:
                    self._push()
                    self.points.insert(edge + 1, p)
                    self.drag = ("corner", edge + 1)
            else:
                self.drag = ("rect", (u, v), (u, v))
        elif event == cv2.EVENT_LBUTTONUP and self.drag:
            if self.drag[0] == "corner":
                self._save()
            else:
                (u0, v0), (u1, v1) = self.drag[1], self.drag[2]
                if abs(u1 - u0) > MIN_RECT_DRAG and abs(v1 - v0) > MIN_RECT_DRAG:
                    corners = [self._to_table(*p) for p in [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]]
                    if all(c is not None for c in corners):
                        self._push()
                        self.points = corners
                        self._save()
            self.drag = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            i = self._corner_at(u, v)
            if i is not None:
                if len(self.points) <= 3:
                    print("An area needs at least 3 corners (x clears it)")
                else:
                    self._push()
                    del self.points[i]
                    self._save()

    # -- drawing --

    def draw(self, image):
        """Dim outside the area, outline it, corner handles with base x, y in mm."""
        pixels = self._pixels()
        if len(pixels) >= 3:
            mask = np.zeros(image.shape[:2], np.uint8)
            cv2.fillPoly(mask, [pixels], 255)
            image[mask == 0] //= 3
        if len(pixels) >= 2:
            cv2.polylines(image, [pixels], len(pixels) >= 3, (255, 0, 255), 2)
        for i, ((x, y), p) in enumerate(zip(self.points, pixels)):
            p = tuple(int(c) for c in p)
            active = i == self.hover or (self.drag and self.drag[0] == "corner" and self.drag[1] == i)
            cv2.circle(image, p, 9 if active else 6, (255, 255, 255) if active else (255, 0, 255), -1)
            cv2.putText(image, f"{x * 1000:.0f}, {y * 1000:.0f}", (p[0] + 10, p[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
        if self.drag and self.drag[0] == "rect":
            cv2.rectangle(image, self.drag[1], self.drag[2], (255, 255, 255), 1)


def draw_map(image, finder, glasses, points=None, tip_xy=None, place_xy=None):
    """Top-down map in base coordinates (X right, Y up) in the top right corner of the image."""
    h, w = image.shape[:2]
    footprint = [pixel_to_plane(finder.K, finder.T_base_cam, p, finder.table_z)
                 for p in [(0, 0), (w, 0), (w, h), (0, h)]]
    footprint = np.array([p[:2] for p in footprint if p is not None])
    area = np.array(points) if points else None

    # Fit everything in view, same scale on both axes
    shown = [np.zeros((1, 2)), footprint]
    shown += [np.array([p]) for p in (tip_xy, place_xy) if p is not None]
    if area is not None:
        shown.append(area)
    shown = np.vstack(shown)
    lo, hi = shown.min(axis=0), shown.max(axis=0)
    margin = 20
    scale = (MAP_SIZE - 2 * margin) / max(hi - lo)
    center = (lo + hi) / 2
    x0, y0 = w - MAP_SIZE - 10, 10

    def px(xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        u = x0 + MAP_SIZE / 2 + (xy[:, 0] - center[0]) * scale
        v = y0 + MAP_SIZE / 2 - (xy[:, 1] - center[1]) * scale
        return np.stack([u, v], axis=1).astype(np.int32)

    panel = image[y0:y0 + MAP_SIZE, x0:x0 + MAP_SIZE]
    panel[:] = panel // 4
    cv2.rectangle(image, (x0, y0), (x0 + MAP_SIZE, y0 + MAP_SIZE), (200, 200, 200), 1)

    if len(footprint) == 4:
        cv2.polylines(image, [px(footprint)], True, (140, 140, 140), 1)
    if area is not None:
        cv2.polylines(image, [px(area)], len(area) >= 3, (255, 0, 255), 2)
        for p in px(area):
            cv2.circle(image, tuple(p), 3, (255, 0, 255), -1)
    for g in glasses:
        cv2.circle(image, tuple(px((g.x, g.y))[0]), max(2, int(g.diameter / 2 * scale)),
                   (0, 255, 0), -1)
    if place_xy is not None:
        cv2.drawMarker(image, tuple(px(place_xy)[0]), (255, 255, 0), cv2.MARKER_SQUARE, 14, 2)
    if tip_xy is not None:
        cv2.drawMarker(image, tuple(px(tip_xy)[0]), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 12, 2)

    # Robot base with its axes, 10 cm long
    o, ax, ay = px([(0, 0), (0.1, 0), (0, 0.1)])
    cv2.circle(image, tuple(o), max(4, int(0.08 * scale)), (255, 255, 255), 1)
    cv2.arrowedLine(image, tuple(o), tuple(ax), (0, 0, 255), 2, tipLength=0.25)
    cv2.arrowedLine(image, tuple(o), tuple(ay), (0, 255, 0), 2, tipLength=0.25)
    cv2.putText(image, "top view, ticks 10 cm", (x0 + 5, y0 + MAP_SIZE - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # 10 cm ticks along the border
    for gx in np.arange(np.floor((center[0] - MAP_SIZE / 2 / scale) * 10) / 10,
                        center[0] + MAP_SIZE / 2 / scale, 0.1):
        u = px((gx, center[1]))[0][0]
        if x0 < u < x0 + MAP_SIZE:
            cv2.line(image, (u, y0), (u, y0 + 5), (200, 200, 200), 1)
    for gy in np.arange(np.floor((center[1] - MAP_SIZE / 2 / scale) * 10) / 10,
                        center[1] + MAP_SIZE / 2 / scale, 0.1):
        v = px((center[0], gy))[0][1]
        if y0 < v < y0 + MAP_SIZE:
            cv2.line(image, (x0, v), (x0 + 5, v), (200, 200, 200), 1)


def draw_glasses(image, glasses, rejected=(), color=(0, 255, 0)):
    for u, v, r, reason in rejected:
        u, v, r = int(round(u)), int(round(v)), int(round(r))
        cv2.circle(image, (u, v), r, (0, 0, 255), 1)
        cv2.putText(image, reason, (u - r, v - r - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    for g in glasses:
        u, v, r = (int(round(p)) for p in g.pixel)
        cv2.circle(image, (u, v), r, color, 2)
        cv2.drawMarker(image, (u, v), color, cv2.MARKER_CROSS, 12, 2)
        cv2.putText(image, f"{g.x * 1000:.0f}, {g.y * 1000:.0f} mm  d{g.diameter * 1000:.0f}",
                    (u - r, v - r - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(image, f"S{g.saturation:.0f} V{g.brightness:.0f}", (u - r, v + r + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


# --- detection window (shared with pick_place_glasses.py) -------------------------

# Trackbar name -> (default, maximum)
TRACKBARS = {
    "edge": (EDGE_THRESHOLD, 500),
    "roundness %": (ROUNDNESS, 99),
    "max saturation": (MAX_SATURATION, 255),
    "min brightness": (MIN_BRIGHTNESS, 255),
}


def load_settings():
    """Trackbar values: saved ones from SETTINGS_FILE, defaults for the rest."""
    settings = {name: default for name, (default, _) in TRACKBARS.items()}
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE) as f:
                saved = json.load(f)
            settings.update({k: int(v) for k, v in saved.items() if k in TRACKBARS})
        except (ValueError, OSError) as e:
            print(f"Could not read {SETTINGS_FILE} ({e}), using defaults")
    return settings


def save_settings(settings):
    tmp = SETTINGS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(settings, f, indent=1)
    os.replace(tmp, SETTINGS_FILE)


_saved_settings = None


def setup_window(editor):
    """Video window with the area editor on the mouse and the detection trackbars."""
    global _saved_settings
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, editor.on_mouse)
    _saved_settings = load_settings()
    for name, (_, maximum) in TRACKBARS.items():
        cv2.createTrackbar(name, WINDOW, _saved_settings[name], maximum, lambda v: None)


def read_trackbars(finder):
    """Apply the trackbars to the finder, and save them when they changed."""
    global _saved_settings
    settings = {name: cv2.getTrackbarPos(name, WINDOW) for name in TRACKBARS}
    finder.edge_threshold = settings["edge"]
    finder.roundness = settings["roundness %"] / 100
    finder.max_saturation = settings["max saturation"]
    finder.min_brightness = settings["min brightness"]
    if settings != _saved_settings:
        save_settings(settings)
        _saved_settings = settings


def draw_overlay(image, finder, editor, glasses, status, tip_xy=None, place_xy=None):
    """Area, base axes, glasses, map and text; `image` must be undistorted."""
    editor.draw(image)
    draw_base_axes(image, finder.K, finder.T_base_cam, finder.table_z)
    draw_glasses(image, glasses, finder.rejected)
    draw_map(image, finder, glasses, editor.points, tip_xy, place_xy)
    cv2.putText(image, f"glasses: {len(glasses)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(image, status, (10, image.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def make_tag_detector():
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICTIONARY),
                                   cv2.aruco.DetectorParameters())


# --- calibration ----------------------------------------------------------------

def tag_centers(detector, undistorted):
    """{tag_id: center pixel}; center = intersection of the diagonals (exact under perspective)."""
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY))
    centers = {}
    if ids is None:
        return centers
    for c, tag_id in zip(corners, ids.flatten()):
        p = np.hstack([c[0], np.ones((4, 1))])
        center = np.cross(np.cross(p[0], p[2]), np.cross(p[1], p[3]))
        centers[int(tag_id)] = center[:2] / center[2]
    return centers


def solve_camera_pose(points, K):
    base = np.array([b for _, b in points], dtype=np.float64)
    pixels = np.array([p for p, _ in points], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(base, pixels, K, None, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(base, pixels, K, None, rvec, tvec)
    T_cam_base = np.eye(4)
    T_cam_base[:3, :3], _ = cv2.Rodrigues(rvec)
    T_cam_base[:3, 3] = tvec.ravel()
    T_base_cam = np.linalg.inv(T_cam_base)

    projected, _ = cv2.projectPoints(base, rvec, tvec, K, None)
    px_errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
    print("\npoint   base x, y, z [mm]            error [px]  error on table [mm]")
    mm_errors = []
    for i, ((pixel, b), e) in enumerate(zip(points, px_errors)):
        hit = pixel_to_plane(K, T_base_cam, pixel, b[2])
        mm = np.linalg.norm(hit[:2] - b[:2]) * 1000
        mm_errors.append(mm)
        print(f"{i:5d}   {b[0] * 1000:7.1f} {b[1] * 1000:7.1f} {b[2] * 1000:7.1f}"
              f"      {e:6.2f}      {mm:6.2f}")
    print(f"RMS error {np.sqrt(np.mean(px_errors ** 2)):.2f} px, "
          f"mean {np.mean(mm_errors):.2f} mm, max {np.max(mm_errors):.2f} mm")

    cam = T_base_cam[:3, 3]
    tilt = np.degrees(np.arccos(np.clip(-T_base_cam[2, 2], -1, 1)))
    print(f"Camera at {np.round(cam * 1000).astype(int)} mm in base frame, "
          f"{tilt:.1f} deg from looking straight down")
    return T_base_cam, float(np.max(mm_errors))


def save_progress(points, pending, width, height):
    """Write atomically, so a crash mid-write can't destroy the saved points."""
    data = {
        "image_size": [width, height],
        "points": [{"pixel": [float(v) for v in p], "base": [float(v) for v in b]}
                   for p, b in points],
        "pending": [{"tag_id": tag_id, "pixel": [float(v) for v in p]} for tag_id, p in pending],
    }
    tmp = PROGRESS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, PROGRESS_FILE)


def load_progress(width, height, fresh):
    if not os.path.exists(PROGRESS_FILE):
        return [], []
    if fresh:
        os.replace(PROGRESS_FILE, PROGRESS_FILE + ".bak")
        print(f"Starting fresh, old progress moved to {PROGRESS_FILE}.bak")
        return [], []
    with open(PROGRESS_FILE) as f:
        data = json.load(f)
    if tuple(data["image_size"]) != (width, height):
        raise RuntimeError(f"{PROGRESS_FILE} was recorded at {tuple(data['image_size'])}, camera "
                           f"gives {(width, height)}. Use --fresh to start over.")
    points = [(np.array(p["pixel"]), np.array(p["base"])) for p in data["points"]]
    pending = [(p["tag_id"], np.array(p["pixel"])) for p in data["pending"]]
    print(f"Resuming from {PROGRESS_FILE}: {len(points)} points, {len(pending)} tags to touch "
          "(--fresh to start over)")
    return points, pending


def calibrate(args):
    import rtde_control
    import rtde_receive

    rtde_r = rtde_receive.RTDEReceiveInterface(IP)
    rtde_c = rtde_control.RTDEControlInterface(IP)
    gamepad = GamepadControl(CALIBRATE_GAMEPAD_KEYS)
    jogger = Jogger(rtde_c, gamepad, MIN_TCP_Z, MAX_TCP_Z)
    cap, width, height = open_camera()
    undistort = Undistorter(width, height)
    detector = make_tag_detector()

    # points: (pixel, base xyz); pending: (tag_id, pixel) measured, waiting to be touched
    points, pending = load_progress(width, height, args.fresh)
    freedrive = False
    status = "Robot out of view, SPACE / A: measure tags"
    watchdog = RobotWatchdog(rtde_c)
    try:
        while True:
            image = undistort(read_frame(cap))
            if not watchdog.kick():
                jogger.stop()
                freedrive = False   # the new control script is not in teach mode
                status = "Robot was stopped (loop stall / protective stop)"
            visible = tag_centers(detector, image)
            for tag_id, c in visible.items():
                cv2.drawMarker(image, c.astype(int), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(image, str(tag_id), c.astype(int) + 8,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            for pixel, _ in points:
                cv2.circle(image, pixel.astype(int), 6, (0, 255, 0), -1)
            if pending:
                tag_id, pixel = pending[0]
                cv2.circle(image, pixel.astype(int), 25, (0, 0, 255), 3)
                status = (f"Touch tag {tag_id} center with the tip, SPACE / A   "
                          "(n / X: skip, f / B: freedrive)")
            cv2.putText(image, f"points: {len(points)}  (c / Y: solve, >= {MIN_POINTS})  "
                        f"{'FREEDRIVE' if freedrive else ''}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image, status, (10, height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(WINDOW, image)

            jogger.update(rtde_r.getActualTCPPose(), enabled=not freedrive)
            keys = read_keys(gamepad)
            if "q" in keys or "\x1b" in keys or window_closed():
                break
            for key in keys:
                if key == "f":
                    jogger.stop()
                    freedrive = not freedrive
                    rtde_c.teachMode() if freedrive else rtde_c.endTeachMode()
                elif key == "n" and pending:
                    pending.pop(0)
                    save_progress(points, pending, width, height)
                    if not pending:
                        status = "Move tags / robot out of view, SPACE / A: measure tags"
                elif key == " " and pending:
                    jogger.stop()
                    tcp = rtde_r.getActualTCPPose()
                    points.append((pending.pop(0)[1], np.array(tcp[:3])))
                    print(f"point {len(points) - 1}: pixel {np.round(points[-1][0], 1)}, "
                          f"base {np.round(np.array(tcp[:3]) * 1000, 1)} mm")
                    save_progress(points, pending, width, height)
                    if not pending:
                        status = "Move tags / robot out of view, SPACE / A: measure tags"
                elif key == " ":
                    jogger.stop()   # measuring blocks the loop for a moment
                    samples = {}
                    for _ in range(CALIB_FRAMES):
                        frame = read_frame(cap)
                        watchdog.kick()
                        for tag_id, c in tag_centers(detector, undistort(frame)).items():
                            samples.setdefault(tag_id, []).append(c)
                    pending = [(tag_id, np.median(s, axis=0)) for tag_id, s in sorted(samples.items())
                               if len(s) >= CALIB_FRAMES // 2]
                    save_progress(points, pending, width, height)
                    if not pending:
                        status = "No tags found"
                elif key == "c":
                    if len(points) < MIN_POINTS:
                        status = warn(f"Need at least {MIN_POINTS} points, have {len(points)}")
                        continue
                    try:
                        T_base_cam, max_mm = solve_camera_pose(points, undistort.K)
                    except (RuntimeError, TypeError, cv2.error) as e:
                        # Degenerate points (e.g. all in a line); the progress file is kept
                        status = warn(f"Solving failed ({e}), add or fix points")
                        continue
                    table_z = min(b[2] for _, b in points)
                    np.savez(POSE_FILE, T_base_cam=T_base_cam, table_z=table_z,
                             image_size=np.array([width, height]),
                             pixels=np.array([p for p, _ in points]),
                             base_points=np.array([b for _, b in points]))
                    print(f"Table z = {table_z * 1000:.1f} mm. Saved {POSE_FILE}")
                    status = f"Saved, max error {max_mm:.1f} mm. Add points or q to quit"
    finally:
        try:
            jogger.stop()
            if freedrive:
                rtde_c.endTeachMode()
            rtde_c.stopScript()
        except Exception:
            pass
        gamepad.close()
        cap.release()
        cv2.destroyAllWindows()


# --- detection mode ---------------------------------------------------------------

def run(args):
    cap, width, height = open_camera()
    finder = GlassFinder.load(width, height)

    rtde_c = rtde_r = gamepad = jogger = None
    if args.robot:
        import rtde_control
        import rtde_receive
        rtde_r = rtde_receive.RTDEReceiveInterface(IP)
        rtde_c = rtde_control.RTDEControlInterface(IP)
        gamepad = GamepadControl(RUN_GAMEPAD_KEYS)
        jogger = Jogger(rtde_c, gamepad, MIN_TCP_Z, MAX_TCP_Z)

    editor = AreaEditor(finder)
    setup_window(editor)
    status = ("p: measure  " + ("m: move above glass  s: stop  t: tip as corner  " if args.robot else "")
              + "area: drag / u: undo / x: clear   q: quit")
    watchdog = RobotWatchdog(rtde_c) if rtde_c else None
    try:
        while True:
            if window_closed():
                break
            read_trackbars(finder)
            image = finder.undistort(read_frame(cap))
            if watchdog and not watchdog.kick():
                jogger.stop()
                status = "Robot was stopped (loop stall / protective stop)"
            glasses = finder.detect(image)
            tcp = rtde_r.getActualTCPPose() if rtde_r else None
            tip_xy = tcp[:2] if tcp else None
            draw_overlay(image, finder, editor, glasses, status, tip_xy)
            cv2.imshow(WINDOW, image)

            if jogger:
                jogger.update(tcp)
            keys = read_keys(gamepad)
            if "q" in keys or "\x1b" in keys:
                break
            for key in keys:
                if key in ("t", "s", "m") and not rtde_c:
                    status = warn(f"{key} needs --robot, ignored")
                elif key == "t":
                    editor.add_point(rtde_r.getActualTCPPose()[:2])
                elif key == "u":
                    editor.undo()
                elif key == "x":
                    editor.clear()
                elif key == "s":
                    jogger.stop()
                    rtde_c.stopL(1.0)
                elif key in ("p", "m"):
                    if jogger:
                        jogger.stop()   # measuring blocks the loop for a moment
                    measured = finder.measure(cap, kick=watchdog.kick if watchdog else None)
                    print(f"\n{len(measured)} glasses (base frame, rim z = {finder.rim_z * 1000:.0f} mm):")
                    for g in measured:
                        print(f"  x {g.x * 1000:7.1f}  y {g.y * 1000:7.1f} mm   diameter {g.diameter * 1000:.0f} mm")
                    if key == "m" and not measured:
                        status = warn("no glass found, m ignored")
                    elif key == "m":
                        c = np.array([width / 2, height / 2])
                        g = min(measured, key=lambda g: np.linalg.norm(np.array(g.pixel[:2]) - c))
                        tcp = rtde_r.getActualTCPPose()
                        target = [g.x, g.y, min(max(g.z + HOVER_CHECK, MIN_TCP_Z), MAX_TCP_Z)] + list(tcp[3:])
                        print(f"Moving tip above glass at {g.x * 1000:.0f}, {g.y * 1000:.0f} mm")
                        rtde_c.moveL(target, CHECK_SPEED, CHECK_ACCEL, True)
    finally:
        if rtde_c:
            try:
                rtde_c.stopL(1.0)
                rtde_c.stopScript()
            except Exception:
                pass
            gamepad.close()
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Find glasses with the overhead camera.")
    parser.add_argument("--calibrate", action="store_true",
                        help="measure the camera pose by touching AprilTags with the TCP")
    parser.add_argument("--fresh", action="store_true",
                        help="with --calibrate: discard saved calibration progress and start over")
    parser.add_argument("--robot", action="store_true",
                        help="connect to the robot, enables the 'm' check move")
    args = parser.parse_args()
    calibrate(args) if args.calibrate else run(args)


if __name__ == "__main__":
    main()
