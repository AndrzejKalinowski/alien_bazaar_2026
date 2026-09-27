"""Classic overhead-camera observer for the supervisor (find_glasses.py, master).

CameraVision owns the camera in its own thread: read frame -> undistort ->
GlassFinder.detect() (Hough circles, colour filter, detection area polygon)
-> WindowAccumulator -> TargetTracker -> Scene. The supervisor's owner thread
only calls observe(), which returns the last published Scene without waiting.
No robot connection: find_glasses is imported for its detector only.

Scenes are published once per WINDOW_PERIOD from the frames in that window,
not per frame. A glass is reported at its median position if it was detected
in at least MIN_SEEN_FRACTION of the window's frames (like
GlassFinder.measure(), which blocks and so cannot run in a control loop).
A new Scene is therefore an independent measurement: the supervisor skips a
target after three of them without it, i.e. about 1 s of absence, not three
frames. Too few frames in a window publish nothing, so the scene goes stale
and the supervisor stops the batch. observed_at is the window's last frame.

IDs: TargetTracker keeps an ID while each published position is within
MATCH_RADIUS of the previous one. Ambiguous matches (two glasses near one
track, or one glass near two tracks) are left out of that Scene instead of
guessing; a glass that jumped farther gets a new ID. The supervisor never
swaps a batch target for a new ID, so a moved glass is skipped, not mixed up.
Glasses are assumed upright (the classic detector cannot tell orientation).

Only the detection area (detection_area.json, edited in find_glasses.py)
counts; it must exclude the stations and the output tags. Camera index,
resolution and calibration files are the constants of find_glasses.py.
A future hardware OBSERVE must wait for a window that started after the arm
left the camera view; the simulated robot never occludes the real table.
Requires: Python standard library for the tracker; CameraVision needs
opencv-python, numpy and the overhead calibration (see find_glasses.py).
"""

from collections import deque
from math import ceil, hypot, isfinite
from statistics import median
import threading
from time import monotonic

from system_model import GlassTarget, Scene

# --- measurement window ----------------------------------------------------------
WINDOW_PERIOD = 0.3          # s, one published Scene per window (below SCENE_MAX_AGE)
MIN_WINDOW_FRAMES = 5        # frames; fewer (slow/stalled camera) publish nothing
MIN_SEEN_FRACTION = 0.5      # of the window's frames a glass must be detected in
CLUSTER_RADIUS = 0.015       # m, detections of one glass within a window

# --- identity --------------------------------------------------------------------
MATCH_RADIUS = 0.03          # m, max move between windows that keeps a glass's ID
MAX_TRACKS = 64              # IDs kept; then new glasses are not published
FORGET_WINDOWS = 100         # windows (~30 s) unseen before an ID is dropped

# --- preview ---------------------------------------------------------------------
PREVIEW_PERIOD = 0.2         # s, JPEG for the panel (5 frames/s)
PREVIEW_WIDTH = 960          # px, preview only; detection uses the calibrated frame
JPEG_QUALITY = 70            # %


def _finite(x, y):
    if not (isfinite(x) and isfinite(y)):
        raise ValueError("detections must have finite coordinates")


class WindowAccumulator:
    """Groups per-frame detections [(x, y, extra), ...] into timed windows."""

    def __init__(self, period=WINDOW_PERIOD, min_frames=MIN_WINDOW_FRAMES,
                 fraction=MIN_SEEN_FRACTION, radius=CLUSTER_RADIUS):
        self.period, self.min_frames = period, min_frames
        self.fraction, self.radius = fraction, radius
        self._start = None
        self._frames = []

    def add(self, points, timestamp):
        """Add one frame; returns (stable points, last timestamp) when a window closes."""
        for x, y, _ in points:
            _finite(x, y)
        if self._start is None:
            self._start = timestamp
        self._frames.append((timestamp, tuple(points)))
        if timestamp - self._start < self.period:
            return None
        frames, self._frames, self._start = self._frames, [], None
        if len(frames) < self.min_frames:
            return [], frames[-1][0]
        clusters = []   # [reference (x, y), {frame index}, [points]]
        for index, (_, detections) in enumerate(frames):
            for p in detections:
                cluster = next((c for c in clusters
                                if hypot(p[0] - c[0][0], p[1] - c[0][1]) <= self.radius), None)
                if cluster is None:
                    clusters.append([(p[0], p[1]), {index}, [p]])
                else:
                    cluster[1].add(index)
                    cluster[2].append(p)
        need = ceil(len(frames) * self.fraction)
        stable = [(median(p[0] for p in c[2]), median(p[1] for p in c[2]), c[2][-1][2])
                  for c in clusters if len(c[1]) >= need]
        return stable, frames[-1][0]


class TargetTracker:
    """Stable IDs for glasses between windows; conservative on ambiguity."""

    def __init__(self, radius=MATCH_RADIUS, max_tracks=MAX_TRACKS, forget=FORGET_WINDOWS):
        self.radius, self.max_tracks, self.forget = radius, max_tracks, forget
        self._tracks = {}      # id -> [x, y, windows since seen]
        self._created = 0

    def update(self, points):
        """points: [(x, y), ...] of one window -> IDs aligned with points (None = not published)."""
        for x, y in points:
            _finite(x, y)
        near_track = {tid: [i for i, p in enumerate(points)
                            if hypot(p[0] - t[0], p[1] - t[1]) <= self.radius]
                      for tid, t in self._tracks.items()}
        near_point = [[tid for tid, t in self._tracks.items()
                       if hypot(p[0] - t[0], p[1] - t[1]) <= self.radius] for p in points]
        ids = [None] * len(points)
        for tid, track in self._tracks.items():
            found = near_track[tid]
            if len(found) == 1 and len(near_point[found[0]]) == 1:
                ids[found[0]] = tid
                track[0], track[1], track[2] = points[found[0]][0], points[found[0]][1], 0
            else:
                track[2] += 1
        for tid in [tid for tid, t in self._tracks.items() if t[2] > self.forget]:
            del self._tracks[tid]
        for i, p in enumerate(points):
            if not near_point[i] and len(self._tracks) < self.max_tracks:
                self._created += 1
                tid = f"glass-{self._created}"
                self._tracks[tid] = [p[0], p[1], 0]
                ids[i] = tid
        return ids


class CameraVision:
    """Camera thread publishing Scenes and preview JPEGs; observe() never blocks.

    frames: a system_web.FrameHub for the panel preview, or None.
    """

    def __init__(self, frames=None, clock=monotonic):
        import cv2
        import find_glasses as fg
        self._cv2, self._fg = cv2, fg
        self.frames = frames
        self.clock = clock
        self.error = ""
        self._lock = threading.Lock()
        self._scene = None
        self._sequence = 0
        self._stop = threading.Event()
        self._thread = None
        self._cap = None
        self._finder = None
        self._window = WindowAccumulator()
        self._tracker = TargetTracker()
        self._labels = []   # (pixel, id) of the last Scene, for the preview
        self._previews = 0

    def start(self):
        """Open the camera and calibration (may take seconds); then run the thread."""
        cap, width, height = self._fg.open_camera()
        try:
            self._finder = self._fg.GlassFinder.load(width, height)
        except Exception:
            cap.release()
            raise
        self._cap = cap
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def observe(self, now):
        with self._lock:
            scene = self._scene
        if scene is None:
            raise ValueError(f"no camera observation yet{': ' + self.error if self.error else ''}")
        return scene  # staleness is the supervisor's check (SCENE_MAX_AGE)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()

    def _run(self):
        next_preview = 0.0
        try:
            while not self._stop.is_set():
                frame = self._fg.read_frame(self._cap)
                now = self.clock()
                image = self._finder.undistort(frame)
                glasses = self._finder.detect(image)
                closed = self._window.add([(g.x, g.y, g.pixel) for g in glasses], now)
                if closed is not None:
                    self._publish(*closed)
                if self.frames is not None and now >= next_preview:
                    next_preview = now + PREVIEW_PERIOD
                    self._preview(image, glasses, now)
        except Exception as exc:
            # No new Scenes: the supervisor sees a stale camera and stops the batch.
            self.error = f"camera stopped: {exc}"

    def _publish(self, stable, observed_at):
        ids = self._tracker.update([(x, y) for x, y, _ in stable])
        targets = tuple(GlassTarget(tid, float(x), float(y))
                        for tid, (x, y, _) in zip(ids, stable) if tid is not None)
        with self._lock:
            self._sequence += 1
            self._scene = Scene(self._sequence, observed_at, targets)
            self._labels = [(pixel, tid) for tid, (_, _, pixel) in zip(ids, stable) if tid is not None]

    def _preview(self, image, glasses, now):
        cv2, fg = self._cv2, self._fg
        if self._finder.area is not None:
            cv2.polylines(image, [self._finder.area_pixels()], True, (255, 200, 0), 2)
        fg.draw_glasses(image, glasses, self._finder.rejected)
        with self._lock:
            labels = list(self._labels)
        for (u, v, r), tid in labels:
            cv2.putText(image, tid, (int(u - r), int(v + r + 34)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
        scale = PREVIEW_WIDTH / image.shape[1]
        if scale < 1:
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self._previews += 1
            self.frames.publish(self._previews, now, jpeg.tobytes())
