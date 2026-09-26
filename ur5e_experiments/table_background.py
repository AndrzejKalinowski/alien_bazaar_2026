"""
Empty-table background for the overhead camera, and the change mask: which
pixels show something that was not on the table when it was empty.

Used by find_glasses.GlassFinder (diff mode); no robot or camera code here.

How it works:
  Capture (b in find_glasses.py / pick_place_glasses.py): with the table
  empty and the robot out of view, BACKGROUND_FRAMES undistorted frames are
  taken. Their per-pixel median is the background, and the mean absolute
  deviation from it is each pixel's noise (a glossy spot or a flickering
  reflection gets a higher noise, so it does not show up as a change). It is
  all computed at DIFF_SCALE of the camera resolution, for speed, and saved to
  table_background.npz together with the camera pose it belongs to: after a
  new overhead calibration (or a different DIFF_SCALE) the file is refused and
  the background must be captured again.

  Every frame is compared with the background in four features: brightness
  (Lab L), colour (Lab a, b) and texture (gradient magnitude of L). Glass is
  transparent, so brightness alone hardly changes; it is the texture of the
  mat seen through the glass (refraction) and the edges / highlights that
  show it. Each difference is divided by the pixel's noise, so the threshold
  (DIFF_THRESHOLD, "diff" trackbar) is in noise sigmas.

  The webcam's auto exposure and white balance cannot be locked, so before
  comparing, the frame's overall brightness ratio to the background (gain,
  median over the detection area) and its colour offset are taken out.

  Shadows (of the robot, of people) only darken: a pixel whose smoothed
  brightness ratio is between SHADOW_MIN_RATIO and SHADOW_MAX_RATIO and whose
  colour did not change is compared with the background darkened by that
  ratio, in brightness and texture. A glass inside a shadow still shows by its
  texture change.

  The thresholded pixels are cleaned (small specks removed, gaps closed,
  holes filled) into the change mask. Unchanged pixels slowly follow the
  current image (ADAPT_RATE), so the background keeps up with slowly changing
  daylight; pixels near a change never do, so a glass never fades into it.

  When more than STALE_FRACTION of the detection area changed, or the gain is
  outside GAIN_LIMITS, the background is considered stale (camera moved,
  lights switched): capture it again with an empty table.

Requires: pip install opencv-python numpy
"""

import os
from dataclasses import dataclass

import cv2
import numpy as np

HERE = os.path.dirname(__file__)
BACKGROUND_FILE = os.path.join(HERE, "table_background.npz")

# --- capture --------------------------------------------------------------------

BACKGROUND_FRAMES = 30       # frames, median over them (~1 s)
CAPTURE_BANDS = 8            # the median is computed in this many bands, kicking the watchdog in between
DIFF_SCALE = 0.5             # the mask is computed at this fraction of the camera resolution
POSE_TOLERANCE = 1e-4        # m / rotation matrix entries, background belongs to this camera pose

# --- difference -------------------------------------------------------------------

BLUR = 5                     # px (at DIFF_SCALE), Gaussian before comparing, odd
# Minimum noise per feature (L, a, b, gradient; 0-255 units), so a perfectly
# still pixel in the capture does not become hair-trigger afterwards
NOISE_FLOOR = (2.0, 1.5, 1.5, 3.0)
DIFF_THRESHOLD = 6           # noise sigmas, a pixel differing more is changed, trackbar
SHADOW_MIN_RATIO = 0.35      # darker than this (fraction of the background) is not a shadow any more
SHADOW_MAX_RATIO = 0.93      # brighter than this is not a shadow (just noise / exposure)
SHADOW_SMOOTH = 15           # px (at DIFF_SCALE), shadows are smooth over this size
GAIN_SAMPLE_STEP = 4         # px, every n-th pixel is used for the gain and colour offset

# --- mask cleanup -----------------------------------------------------------------

OPEN_RADIUS = 1              # px (at DIFF_SCALE), removes single-pixel specks
CLOSE_RADIUS = 4             # px (at DIFF_SCALE), joins the edges of a glass into one blob
MIN_BLOB_PIXELS = 30         # px (at DIFF_SCALE), smaller blobs are noise

# --- keeping it current -----------------------------------------------------------

ADAPT_RATE = 0.02            # per frame, unchanged pixels move this fraction towards the frame
ADAPT_MARGIN = 10            # px (at DIFF_SCALE), pixels this close to a change do not adapt
STALE_FRACTION = 0.4         # more of the area changed -> background stale
GAIN_LIMITS = (0.5, 2.0)     # exposure ratio outside -> background stale


def features(bgr):
    """[L, a, b, gradient], (h, w) float32 each: Lab (0-255) and the gradient magnitude of L."""
    blurred = cv2.GaussianBlur(bgr, (BLUR, BLUR), 0)
    L, a, b = (c.astype(np.float32) for c in cv2.split(cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)))
    # Sobel 3x3 is 4x a unit step; scaled so the gradient is on the same scale as L
    gx = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3, scale=0.25)
    gy = cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3, scale=0.25)
    return [L, a, b, cv2.magnitude(gx, gy)]


def small_size(width, height):
    return int(round(width * DIFF_SCALE)), int(round(height * DIFF_SCALE))


def disk(radius):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))


def fill_holes(mask):
    """Everything inside an outer contour set, so a glass is one solid blob."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
    return filled


def remove_small(mask, min_pixels):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = np.zeros(n, bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_pixels
    return np.where(keep[labels], 255, 0).astype(np.uint8)


@dataclass
class Change:
    mask: np.ndarray            # uint8 0 / 255, full camera resolution
    score: np.ndarray           # float32 noise sigmas, at DIFF_SCALE (for display)
    gain: float                 # frame / background brightness
    changed_fraction: float     # of the detection area
    stale: bool                 # background no longer matches the empty table


class TableBackground:
    def __init__(self, background, noise, image_size, T_base_cam):
        """background: BGR uint8 at DIFF_SCALE; noise: (h, w, 4) float32 per feature."""
        self.background = background
        self.noise = np.maximum(noise, np.array(NOISE_FLOOR, np.float32)).astype(np.float32)
        self._inverse_noise = cv2.split(1 / self.noise)   # per feature, multiplying is faster
        self.image_size = tuple(int(v) for v in image_size)
        self.T_base_cam = np.asarray(T_base_cam, dtype=float)
        self.reference = features(background)   # adapts slowly, the saved background does not
        self.threshold = DIFF_THRESHOLD

    # -- capture / files --

    @classmethod
    def capture(cls, read, image_size, T_base_cam, frames=BACKGROUND_FRAMES, kick=None):
        """Median of `frames` undistorted frames from read(); kick() after each (watchdog).

        Blocks for about frames / camera fps; stop any jogging first.
        """
        kick = kick or (lambda: None)
        size = small_size(*image_size)
        stack = []
        for _ in range(frames):
            stack.append(cv2.resize(read(), size, interpolation=cv2.INTER_AREA))
            kick()
        stack = np.stack(stack)
        # In bands of rows with a kick in between: in one go the median takes
        # longer than the watchdog allows
        background = np.empty(stack.shape[1:], np.uint8)
        edges = np.linspace(0, size[1], CAPTURE_BANDS + 1).astype(int)
        for top, bottom in zip(edges[:-1], edges[1:]):
            background[top:bottom] = np.median(stack[:, top:bottom], axis=0)
            kick()
        reference = features(background)
        deviation = [np.zeros_like(c) for c in reference]
        for frame in stack:
            for total, c, r in zip(deviation, features(frame), reference):
                total += cv2.absdiff(c, r)
            kick()
        # Mean absolute deviation * 1.25 = sigma for Gaussian noise
        noise = np.dstack(deviation) * (1.25 / len(stack))
        return cls(background, noise, image_size, T_base_cam)

    def save(self, path=None):
        path = path or BACKGROUND_FILE   # looked up now, so tests can point it elsewhere
        tmp = path + ".tmp.npz"   # np.savez appends .npz to names without it
        np.savez_compressed(tmp, background=self.background, noise=self.noise,
                            image_size=np.array(self.image_size), T_base_cam=self.T_base_cam,
                            diff_scale=DIFF_SCALE)
        os.replace(tmp, path)

    @classmethod
    def load(cls, image_size, T_base_cam, path=None):
        """The saved background, or None (with the reason printed) if missing or not valid now."""
        path = path or BACKGROUND_FILE
        if not os.path.exists(path):
            return None
        try:
            data = np.load(path)
            reason = None
            if tuple(data["image_size"]) != tuple(image_size):
                reason = f"recorded at {tuple(data['image_size'])}, camera gives {tuple(image_size)}"
            elif float(data["diff_scale"]) != DIFF_SCALE:
                reason = f"recorded at DIFF_SCALE {float(data['diff_scale'])}, now {DIFF_SCALE}"
            elif not np.allclose(data["T_base_cam"], T_base_cam, atol=POSE_TOLERANCE):
                reason = "the overhead camera was calibrated again since"
            if reason:
                print(f"Table background {path} not used: {reason}. Capture it again (b).")
                return None
            return cls(data["background"], data["noise"], data["image_size"], data["T_base_cam"])
        except (OSError, KeyError, ValueError) as e:
            print(f"Could not read {path} ({e}), capture the background again (b)")
            return None

    # -- comparing --

    def compare(self, undistorted, area_pixels=None, adapt=True):
        """Change mask of one undistorted frame (full resolution).

        area_pixels: detection area polygon in full-resolution pixels, or None
        (whole image). It is only used for the gain and the stale check: the
        mask covers the whole image, since a glass standing in the area can
        reach out of it in the image (parallax).
        adapt: let unchanged pixels follow the frame (off while something big,
        like the robot, is in view, so it can't slowly become background).
        """
        width, height = self.image_size
        size = small_size(width, height)
        L, a, b, G = features(cv2.resize(undistorted, size, interpolation=cv2.INTER_AREA))
        rL, ra, rb, rG = self.reference
        iL, ia, ib, iG = self._inverse_noise

        area = np.ones(L.shape, bool)
        if area_pixels is not None and len(area_pixels) >= 3:
            area = np.zeros(L.shape, np.uint8)
            cv2.fillPoly(area, [np.round(np.asarray(area_pixels) * DIFF_SCALE).astype(np.int32)], 1)
            area = area.astype(bool)
        step = GAIN_SAMPLE_STEP
        sample = area[::step, ::step]
        if not sample.any():
            step, sample = 1, area

        def sampled(channel):
            return channel[::step, ::step][sample]

        # Auto exposure / white balance: overall ratio and colour offset
        gain = max(float(np.median(sampled(L) / np.maximum(sampled(rL), 1))), 1e-3)
        offset_a = float(np.median(sampled(a) - sampled(ra)))
        offset_b = float(np.median(sampled(b) - sampled(rb)))

        d_color = cv2.magnitude((a - ra - offset_a) * ia, (b - rb - offset_b) * ib)
        d_L = np.abs(L - gain * rL) * (iL / gain)
        d_G = np.abs(G - gain * rG) * (iG / gain)
        score = np.maximum(d_color, np.maximum(d_L, d_G))

        # Shadows: compare with the background darkened by the local ratio.
        # Not with a darkened noise: the sensor noise does not drop with the light.
        k = 2 * SHADOW_SMOOTH + 1
        shade = cv2.GaussianBlur(L / (gain * np.maximum(rL, 1)), (k, k), 0)
        shadow = (shade > SHADOW_MIN_RATIO) & (shade < SHADOW_MAX_RATIO) & (d_color < self.threshold)
        if shadow.any():
            d_L = np.abs(L - shade * gain * rL) * (iL / gain)
            d_G = np.abs(G - shade * gain * rG) * (iG / gain)
            np.copyto(score, np.maximum(d_color, np.maximum(d_L, d_G)), where=shadow)

        mask = np.where(score > self.threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, disk(OPEN_RADIUS))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, disk(CLOSE_RADIUS))
        mask = remove_small(fill_holes(mask), MIN_BLOB_PIXELS)

        changed_fraction = float(np.mean(mask[area] > 0))
        stale = changed_fraction > STALE_FRACTION or not GAIN_LIMITS[0] <= gain <= GAIN_LIMITS[1]

        if adapt and not stale:
            still = np.where(cv2.dilate(mask, disk(ADAPT_MARGIN)) == 0, 255, 0).astype(np.uint8)
            for c, r in zip([L / gain, a - offset_a, b - offset_b, G / gain], self.reference):
                cv2.accumulateWeighted(c, r, ADAPT_RATE, mask=still)

        full = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        return Change(full, score, gain, changed_fraction, stale)
