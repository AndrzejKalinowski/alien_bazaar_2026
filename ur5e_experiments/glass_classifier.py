"""
Classifies what the change mask (table_background.py) found on the table:
tapered glasses standing upright ("up") or upside down ("down"), and other
things (unknown objects, obstructions like the robot arm or a hand).

Used by find_glasses.GlassFinder in diff mode; no robot or camera code here.

How it works:
  1. Every blob of the change mask is searched for circles (HoughCircles, only
     inside the blob, radius range from the glass shape). A circle counts only
     if most of its ring lies on changed pixels, so edges in the mat, tape and
     tags can't make a glass any more.
  2. A glass is a truncated cone: GlassShape(height, mouth, foot). Seen from
     above it shows two circles, one on the table (z = table) and one on top
     (z = table + height). The top circle is at the same height either way;
     upright it is the mouth (wide), upside down the foot (narrow).
  3. Each pair of circles, and each single circle, is tried as both
     orientations: every circle is back-projected onto the height it would
     have, and the fit costs (in sigmas, squared and averaged)
       - axis: both circle centres must be above the same table point
         (AXIS_TOLERANCE). Away from the point under the camera this is the
         strong cue: parallax shifts the top circle outwards by ~2 cm at 30 cm
         off-centre, and the wrong orientation puts the ends 2x that apart.
       - diameters: each circle must match its end (DIAMETER_TOLERANCE).
         Under the camera, where parallax is gone, this still separates them:
         a circle assumed on the wrong height comes out ~8 % off in size.
       - coverage: the predicted outline (hull of both ends) must lie on
         changed pixels (COVERAGE_TOLERANCE).
     The orientation with the lower sum wins; if the other one is less than
     MIN_ORIENTATION_MARGIN worse, the orientation is "?" (not picked). Fits
     above MAX_FIT_COST (per term) are not glasses. With only one circle
     found, near the point under the camera, "?" is expected.
  4. Pairs are accepted first (best fit first), then single circles; circles
     inside an accepted glass (reflections, the far end seen through the
     glass) are not a second glass. When a glass-sized blob has no circle at
     all, its enclosing circle is tried as a single circle.
  5. What is left of each blob after taking out the glasses' outlines is an
     object: an "obstruction" when it touches the image border (the robot
     arm, a hand) or is bigger than OBSTRUCTION_AREA, else "unknown".
  6. Time limit: the caller's main loop kicks the robot watchdog (200 ms), and
     a busy mask (reflective table, stale background) took 250-550 ms per
     frame. So blobs are searched glass-sized first, blobs outside `search`
     (the detection area) not at all, and none after `deadline`. Blobs that
     are not searched still become objects with the same labels (so
     obstructions are always reported), with checked=False when the deadline
     cut them off.

Requires: pip install opencv-python numpy
"""

import time
from dataclasses import dataclass
from itertools import combinations

import cv2
import numpy as np

UP = "up"          # standing on the foot, open end on top
DOWN = "down"      # standing on the mouth (how the demo presents them)
UNSURE = "?"

# --- circles --------------------------------------------------------------------

RADIUS_MARGIN = 0.15         # widen the expected pixel radius range by this fraction
ROI_PADDING = 10             # px around a blob searched for circles
BLUR = 5                     # px, median blur before HoughCircles (odd)
DUPLICATE_CIRCLE = 0.15      # circles closer than this fraction of the radius (centre and radius) are one
RING_SLACK = 4               # px, the change mask is widened by this for the ring check
MIN_RING_CHANGED = 0.6       # fraction of a circle's ring that must be on changed pixels
RING_SAMPLES = 64            # points checked along the ring
MAX_CIRCLES = 12             # per blob, the strongest are kept
REFINE_SEARCH = 5            # px, rim edges are taken this far in / out of the Hough circle
REFINE_INLIER = 2.5          # px, then this far from the first fit (half a rim + noise)
REFINE_SECTORS = 16          # the rim must be seen in REFINE_MIN_SECTORS of these angle sectors
REFINE_MIN_SECTORS = 0.6

# --- fitting --------------------------------------------------------------------

AXIS_TOLERANCE = 0.005       # m, 1 sigma, top and bottom centre of one glass back-projected
DIAMETER_TOLERANCE = 0.003   # m, 1 sigma, measured circle vs its end of the glass (1 px ~ 2 mm)
COVERAGE_TOLERANCE = 0.15    # 1 sigma, fraction of the predicted outline not on changed pixels
MAX_FIT_COST = 4.0           # mean squared sigmas per fit term, worse is not a glass
MIN_ORIENTATION_MARGIN = 4.0  # chi2 (summed squared sigmas) the other orientation must be worse by, else "?"
OUTLINE_POINTS = 48          # points per end circle of the predicted outline
CLASSIFY_BUDGET = 0.06       # s per frame, change mask + glass search (find_glasses sets the deadline)

# --- objects --------------------------------------------------------------------

MIN_OBJECT_AREA = 0.0004     # m^2 on the table (2 x 2 cm), smaller leftovers are ignored
OBSTRUCTION_AREA = 0.03      # m^2, bigger leftovers are obstructions (also when touching the border)
EXPLAINED_MARGIN = 6         # px, a glass outline is widened by this before it is taken out of its blob
BORDER = 3                   # px, a blob this close to the image edge touches it


@dataclass
class GlassShape:
    height: float     # m, table to top
    mouth: float      # m, diameter of the open end
    foot: float       # m, diameter of the closed end (bottom of an upright glass)

    def ends(self, orientation):
        """(diameter on the table, diameter on top)."""
        return (self.mouth, self.foot) if orientation == DOWN else (self.foot, self.mouth)

    def radius_at(self, orientation, z):
        """Wall radius at height z above the table (straight taper)."""
        bottom, top = self.ends(orientation)
        t = min(max(z / self.height, 0.0), 1.0)
        return (bottom + (top - bottom) * t) / 2


@dataclass
class FoundGlass:
    xy: np.ndarray        # m, base x, y of the glass axis
    orientation: str      # UP, DOWN or UNSURE
    margin: float         # how much worse the other orientation fits (chi2), confidence
    cost: float           # fit cost of the chosen orientation, squared sigmas per term
    top_diameter: float   # m, the top circle as measured (or the model's, if not seen)
    top_pixel: tuple      # (u, v, r) of the top circle in the image
    outline: np.ndarray   # (N, 2) px, predicted outline of the whole glass


@dataclass
class TableObject:
    label: str            # "unknown" or "obstruction"
    contour: np.ndarray   # (N, 2) px
    xy: tuple             # m, table point under the contour's centre
    area: float           # m^2 on the table
    table_contour: np.ndarray = None   # (N, 2) m, the contour on the table plane
    checked: bool = True  # False: not (fully) searched for glasses, the time budget ran out


class TableCamera:
    """Back-projection and projection for the undistorted overhead camera."""

    def __init__(self, K, T_base_cam, table_z):
        self.K = np.asarray(K, dtype=float)
        self.T_base_cam = np.asarray(T_base_cam, dtype=float)
        self.table_z = table_z
        T_cam_base = np.linalg.inv(self.T_base_cam)
        self._rvec = cv2.Rodrigues(T_cam_base[:3, :3])[0]
        self._tvec = T_cam_base[:3, 3]
        self._K_inv = np.linalg.inv(self.K)

    def to_plane(self, pixels, z):
        """(N, 2) pixels -> (N, 3) base points on the plane at height z (NaN if not in front)."""
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        rays = self.T_base_cam[:3, :3] @ (self._K_inv @ np.column_stack([pixels, np.ones(len(pixels))]).T)
        origin = self.T_base_cam[:3, 3]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (z - origin[2]) / rays[2]
        s[~(s > 0)] = np.nan
        return (origin[:, None] + s * rays).T

    def project(self, points):
        pixels, _ = cv2.projectPoints(np.asarray(points, dtype=float).reshape(-1, 3),
                                      self._rvec, self._tvec, self.K, None)
        return pixels.reshape(-1, 2)

    def depth(self, z):
        """Distance along the optical axis to the plane at height z, at the image centre."""
        point = self.to_plane([self.K[:2, 2]], z)[0]
        return (np.linalg.inv(self.T_base_cam) @ np.append(point, 1))[2]

    def ring(self, xy, z, diameter, n=OUTLINE_POINTS):
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        return self.project(np.column_stack([xy[0] + diameter / 2 * np.cos(a),
                                             xy[1] + diameter / 2 * np.sin(a), np.full(n, z)]))

    def circle_on_plane(self, circle, z):
        """Image circle (u, v, r) -> (x, y centre, diameter) on the plane at height z."""
        u, v, r = circle
        points = self.to_plane([(u, v), (u + r, v), (u - r, v), (u, v + r), (u, v - r)], z)
        center = points[0]
        diameter = 2 * np.mean(np.linalg.norm(points[1:, :2] - center[:2], axis=1))
        return center[:2], diameter

    def table_area(self, contour):
        """m^2 on the table inside an image contour."""
        p = self.to_plane(contour, self.table_z)[:, :2]
        if np.isnan(p).any():
            return np.inf
        return 0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], 1)) - np.dot(p[:, 1], np.roll(p[:, 0], 1)))


def outline(cam, shape, xy, orientation):
    """Predicted image outline (convex hull of both ends) of a glass standing at xy."""
    bottom, top = shape.ends(orientation)
    points = np.vstack([cam.ring(xy, cam.table_z, bottom),
                        cam.ring(xy, cam.table_z + shape.height, top)])
    return cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)


def radius_range(cam, shape):
    """Pixel radius range of the glass ends; off-centre ends are farther and look smaller."""
    f = cam.K[0, 0]
    smallest = f * min(shape.foot, shape.mouth) / 2 / cam.depth(cam.table_z)
    biggest = f * max(shape.foot, shape.mouth) / 2 / cam.depth(cam.table_z + shape.height)
    return max(int(smallest * (1 - RADIUS_MARGIN)), 3), int(np.ceil(biggest * (1 + RADIUS_MARGIN)))


class Blob:
    """One connected blob of the change mask, in a padded region of interest."""

    def __init__(self, labels, index, stats, image_size):
        w, h = image_size
        x, y, bw, bh = stats[:4]
        self.x0, self.y0 = max(x - ROI_PADDING, 0), max(y - ROI_PADDING, 0)
        self.x1, self.y1 = min(x + bw + ROI_PADDING, w), min(y + bh + ROI_PADDING, h)
        self.mask = np.where(labels[self.y0:self.y1, self.x0:self.x1] == index, 255, 0).astype(np.uint8)
        self.touches_border = (x <= BORDER or y <= BORDER
                               or x + bw >= w - BORDER or y + bh >= h - BORDER)
        slack = 2 * RING_SLACK + 1
        self.wide = cv2.dilate(self.mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (slack, slack)))

    def local(self, pixels):
        return np.asarray(pixels, dtype=float) - (self.x0, self.y0)

    def coverage(self, polygon):
        """Fraction of the polygon (image pixels) that is changed."""
        canvas = np.zeros_like(self.mask)
        cv2.fillPoly(canvas, [np.round(self.local(polygon)).astype(np.int32)], 255)
        inside = np.count_nonzero(canvas)
        return np.count_nonzero(canvas & self.mask) / inside if inside else 0.0

    def ring_changed(self, circle):
        u, v, r = circle
        a = np.linspace(0, 2 * np.pi, RING_SAMPLES, endpoint=False)
        pts = np.round(self.local(np.column_stack([u + r * np.cos(a), v + r * np.sin(a)]))).astype(int)
        h, w = self.mask.shape
        ok = (pts[:, 0] >= 0) & (pts[:, 0] < w) & (pts[:, 1] >= 0) & (pts[:, 1] < h)
        return np.count_nonzero(self.wide[pts[ok, 1], pts[ok, 0]]) / RING_SAMPLES


def fit_circle(points):
    """Least-squares circle (u, v, r) through (N, 2) points (Kasa fit)."""
    x, y = points[:, 0], points[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    (a, b, c), *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    u, v = a / 2, b / 2
    return u, v, np.sqrt(max(c + u ** 2 + v ** 2, 0.0))


def refine_circle(edges, circle):
    """Sub-pixel circle fitted to the edge pixels of its rim.

    HoughCircles is often 1-2 px off in radius (and more in the centre next to
    other glasses), and near the point under the camera 2 px is the whole
    difference between the orientations. edges: (N, 2) Canny edge pixels of the
    blob. The edges within REFINE_SEARCH of the circle are fitted, then those
    within REFINE_INLIER of that fit again. A rim has an inner and an outer
    edge; fitting both gives the middle of the rim. Returns the Hough circle
    unchanged when the rim is not seen around most of the circle.
    """
    u, v, r = circle
    for band in (REFINE_SEARCH, REFINE_INLIER):
        d = np.hypot(edges[:, 0] - u, edges[:, 1] - v)
        points = edges[np.abs(d - r) < band]
        angles = np.arctan2(points[:, 1] - v, points[:, 0] - u)
        bins = np.unique(((angles + np.pi) / (2 * np.pi) * REFINE_SECTORS).astype(int) % REFINE_SECTORS)
        if len(bins) < REFINE_MIN_SECTORS * REFINE_SECTORS:
            return circle
        u, v, r = fit_circle(points)
    if np.hypot(u - circle[0], v - circle[1]) > REFINE_SEARCH or abs(r - circle[2]) > REFINE_SEARCH:
        return circle
    return float(u), float(v), float(r)


def find_circles(gray, blob, radii, edge_threshold, roundness):
    """Circles (u, v, r) in image pixels inside the blob, strongest first, duplicates merged."""
    roi = cv2.medianBlur(gray[blob.y0:blob.y1, blob.x0:blob.x1], BLUR)
    if min(roi.shape) < 2 * radii[0]:
        return []
    # Same thresholds as HoughCircles uses inside (param1 = Canny high, low = half)
    edge_y, edge_x = np.nonzero(cv2.Canny(roi, max(edge_threshold, 1) / 2, max(edge_threshold, 1)))
    edges = np.column_stack([edge_x + blob.x0, edge_y + blob.y0]).astype(float)
    found = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT_ALT, dp=1.5, minDist=2,
                             param1=max(edge_threshold, 1), param2=min(max(roundness, 0.05), 0.99),
                             minRadius=radii[0], maxRadius=radii[1])
    circles = []
    if found is None:
        return circles
    for u, v, r in found.reshape(-1, 3):
        c = refine_circle(edges, (float(u + blob.x0), float(v + blob.y0), float(r)))
        if any(np.hypot(c[0] - k[0], c[1] - k[1]) < DUPLICATE_CIRCLE * k[2]
               and abs(c[2] - k[2]) < DUPLICATE_CIRCLE * k[2] for k in circles):
            continue
        if blob.ring_changed(c) >= MIN_RING_CHANGED:
            circles.append(c)
        if len(circles) >= MAX_CIRCLES:
            break
    return circles


def fit(cam, shape, blob, circles, orientation):
    """(chi2, terms, xy, top circle or None, top diameter, outline) for circles as this
    orientation; chi2 = sum of the squared sigmas of the `terms` fit terms.

    Two circles: the bigger is the mouth. One circle: tried as either end.
    """
    bottom_d, top_d = shape.ends(orientation)
    top_z, bottom_z = cam.table_z + shape.height, cam.table_z
    if len(circles) == 2:
        big, small = sorted(circles, key=lambda c: -c[2])
        mouth_on_top = orientation == UP
        options = [(big, small) if mouth_on_top else (small, big)]   # (top, bottom)
    else:
        options = [(circles[0], None), (None, circles[0])]

    best = None
    for top, bottom in options:
        terms, centers = [], []
        for circle, z, expected in ((top, top_z, top_d), (bottom, bottom_z, bottom_d)):
            if circle is None:
                continue
            xy, d = cam.circle_on_plane(circle, z)
            if np.isnan(xy).any():
                break
            centers.append(xy)
            terms.append(((d - expected) / DIAMETER_TOLERANCE) ** 2)
        else:
            if len(centers) == 2:
                terms.append((np.linalg.norm(centers[0] - centers[1]) / AXIS_TOLERANCE) ** 2)
            xy = np.mean(centers, axis=0)
            shape_outline = outline(cam, shape, xy, orientation)
            terms.append(((1 - blob.coverage(shape_outline)) / COVERAGE_TOLERANCE) ** 2)
            chi2 = float(np.sum(terms))
            if best is None or chi2 < best[0]:
                top_diameter = cam.circle_on_plane(top, top_z)[1] if top is not None else top_d
                best = (chi2, len(terms), xy, top, top_diameter, shape_outline)
    return best


def classify_candidate(cam, shape, blob, circles):
    """FoundGlass for one or two circles, or None when neither orientation fits."""
    fits = {o: fit(cam, shape, blob, circles, o) for o in (UP, DOWN)}
    fits = {o: f for o, f in fits.items() if f is not None}
    if not fits:
        return None
    best = min(fits, key=lambda o: fits[o][0])
    chi2, terms, xy, top, top_diameter, shape_outline = fits[best]
    cost = chi2 / terms   # per term, so pairs and single circles compare
    if cost > MAX_FIT_COST:
        return None
    # Same terms for both orientations: the chi2 difference is the log likelihood ratio (x2)
    others = [f[0] for o, f in fits.items() if o != best]
    margin = others[0] - chi2 if others else np.inf
    if top is None:
        # Not seen: the model's top circle, as a circle in the image
        ring = cam.ring(xy, cam.table_z + shape.height, shape.ends(best)[1])
        (u, v), r = cv2.minEnclosingCircle(ring.astype(np.float32))
        top = (float(u), float(v), float(r))
    orientation = best if margin >= MIN_ORIENTATION_MARGIN else UNSURE
    return FoundGlass(xy, orientation, float(margin), cost, float(top_diameter), top, shape_outline)


def past(deadline):
    return deadline is not None and time.perf_counter() > deadline


def classify_blob(cam, shape, blob, circles, deadline=None):
    """Glasses in one blob: pairs first (best fit first), then single circles.
    Past the deadline no more candidates are tried (up to 78 fits for MAX_CIRCLES)."""
    candidates = []
    for pair in combinations(circles, 2):
        if past(deadline):
            break
        g = classify_candidate(cam, shape, blob, list(pair))
        if g is not None:
            candidates.append((0, g.cost, pair, g))
    for c in circles:
        if past(deadline):
            break
        g = classify_candidate(cam, shape, blob, [c])
        if g is not None:
            candidates.append((1, g.cost, (c,), g))
    candidates.sort(key=lambda item: item[:2])

    glasses, used = [], set()
    spacing = min(shape.foot, shape.mouth)   # two glasses' axes can't be closer
    for _, _, members, g in candidates:
        if used & set(members):
            continue
        if any(np.linalg.norm(g.xy - k.xy) < spacing for k in glasses):
            continue
        # A circle inside an accepted glass is part of it (far end, reflection)
        if any(cv2.pointPolygonTest(k.outline.astype(np.float32), c[:2], False) >= 0
               for k in glasses for c in members):
            continue
        glasses.append(g)
        used |= set(members)
    return glasses


def leftover_objects(cam, blob, glasses, min_object_area=MIN_OBJECT_AREA, checked=True):
    """What of the blob the glasses don't explain, as TableObjects."""
    rest = blob.mask.copy()
    for g in glasses:
        explained = np.zeros_like(rest)
        cv2.fillPoly(explained, [np.round(blob.local(g.outline)).astype(np.int32)], 255)
        k = 2 * EXPLAINED_MARGIN + 1
        explained = cv2.dilate(explained, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        rest[explained > 0] = 0
    objects = []
    contours, _ = cv2.findContours(rest, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour = contour.reshape(-1, 2) + (blob.x0, blob.y0)
        if len(contour) < 3:
            continue
        area = cam.table_area(contour)
        if area < min_object_area:
            continue
        m = cv2.moments(contour.astype(np.float32))
        center = (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] else contour.mean(axis=0)
        xy = cam.to_plane([center], cam.table_z)[0, :2]
        big = area > OBSTRUCTION_AREA or blob.touches_border
        objects.append(TableObject("obstruction" if big else "unknown", contour,
                                   (float(xy[0]), float(xy[1])), float(area),
                                   cam.to_plane(contour, cam.table_z)[:, :2], checked))
    return objects


def classify(undistorted, mask, cam, shape, edge_threshold, roundness, search=None, deadline=None):
    """(glasses, objects) on the table, from an undistorted BGR frame and its change mask.

    search: uint8 mask, glasses are only searched in blobs that overlap it (None =
    everywhere); deadline: time.perf_counter() after which no blob is searched.
    """
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    image_size = (mask.shape[1], mask.shape[0])
    radii = radius_range(cam, shape)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    # Glass-sized blobs first: when the deadline cuts in, it hits the big ones
    # (arm, cables, reflections), which also cost the most Hough time
    glass_area = np.pi * radii[1] ** 2
    order = sorted(range(1, count),
                   key=lambda i: abs(np.log(max(stats[i, cv2.CC_STAT_AREA], 1) / glass_area)))
    glasses, objects = [], []
    for index in order:
        blob = Blob(labels, index, stats[index], image_size)
        if search is not None and not np.any(search[blob.y0:blob.y1, blob.x0:blob.x1][blob.mask > 0]):
            objects += leftover_objects(cam, blob, [])
            continue
        if past(deadline):
            objects += leftover_objects(cam, blob, [], checked=False)
            continue
        circles = find_circles(gray, blob, radii, edge_threshold, roundness)
        found = classify_blob(cam, shape, blob, circles, deadline)
        if not found and not blob.touches_border and not past(deadline):
            # Hough found nothing usable: try the blob's enclosing circle as one end
            contours, _ = cv2.findContours(blob.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            (u, v), r = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            if radii[0] <= r <= radii[1]:
                found = classify_blob(cam, shape, blob, [(u + blob.x0, v + blob.y0, r)], deadline)
        glasses += found
        objects += leftover_objects(cam, blob, found, checked=not past(deadline))
    return glasses, objects


def near(glass_xy, objects, clearance, labels=("obstruction",)):
    """Objects with one of the labels whose outline on the table comes within clearance (m)
    of glass_xy (a glass axis), or contains it."""
    result = []
    for obj in objects:
        if obj.label not in labels:
            continue
        outline_m = obj.table_contour.astype(np.float32)
        # Signed distance: positive inside the outline
        d = cv2.pointPolygonTest(outline_m, (float(glass_xy[0]), float(glass_xy[1])), True)
        if d > -clearance:
            result.append(obj)
    return result
