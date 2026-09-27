"""Classic overhead-camera observer for the supervisor (find_glasses.py, master).

The camera runs in its OWN PROCESS (multiprocessing, spawn): read frame ->
undistort -> GlassFinder.detect() (Hough circles, colour filter, detection
area polygon) -> WindowAccumulator -> TargetTracker. Only finished Scenes and
preview JPEGs cross the pipe, so image processing never competes with the
control loop for the GIL. In the supervisor process a reader thread keeps the
latest Scene; the owner thread only calls observe(), which never waits.
If the camera process dies or stops sending, no new Scene arrives, the
scene goes stale and the supervisor stops the batch. Timestamps are
time.monotonic() of the camera process (a system-wide clock on Windows and
Linux); the supervisor also rejects timestamps from the future.
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
window_start is the first frame of the window: the hardware OBSERVE step
waits for a Scene whose window started after the arm reached the observe
pose, so no frame of it can show the arm over the table.
Requires: Python standard library for the tracker; CameraVision needs
opencv-python, numpy and the overhead calibration (see find_glasses.py).
"""

from collections import deque
from math import ceil, hypot, isfinite
from statistics import median
import multiprocessing
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

# --- camera process --------------------------------------------------------------
CAMERA_START_TIMEOUT = 20.0  # s, open camera + calibration in the child process
CAMERA_STOP_TIMEOUT = 2.0    # s, then the process is terminated


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
        """Add one frame; returns (stable points, last, first timestamp) when a window closes."""
        for x, y, _ in points:
            _finite(x, y)
        if self._start is None:
            self._start = timestamp
        self._frames.append((timestamp, tuple(points)))
        if timestamp - self._start < self.period:
            return None
        frames, self._frames, self._start = self._frames, [], None
        if len(frames) < self.min_frames:
            return [], frames[-1][0], frames[0][0]
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
        return stable, frames[-1][0], frames[0][0]


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


class CameraPipeline:
    """Camera process side: frames in, ("scene", ...) and ("frame", ...) messages out."""

    def __init__(self, send, clock=monotonic):
        import cv2
        import find_glasses as fg
        self._cv2, self._fg = cv2, fg
        self.send = send
        self.clock = clock
        self._cap = None
        self._finder = None
        self._window = WindowAccumulator()
        self._tracker = TargetTracker()
        self._sequence = 0
        self._labels = []   # (pixel, id) of the last Scene, for the preview
        self._previews = 0

    def open(self):
        cap, width, height = self._fg.open_camera()
        try:
            self._finder = self._fg.GlassFinder.load(width, height)
        except Exception:
            cap.release()
            raise
        self._cap = cap

    def close(self):
        if self._cap is not None:
            self._cap.release()

    def run(self, stop):
        next_preview = 0.0
        while not stop.is_set():
            frame = self._fg.read_frame(self._cap)
            now = self.clock()
            image = self._finder.undistort(frame)
            glasses = self._finder.detect(image)
            closed = self._window.add([(g.x, g.y, g.pixel) for g in glasses], now)
            if closed is not None:
                self._publish(*closed)
            if now >= next_preview:
                next_preview = now + PREVIEW_PERIOD
                self._preview(image, glasses, now)

    def _publish(self, stable, observed_at, window_start):
        ids = self._tracker.update([(x, y) for x, y, _ in stable])
        self._sequence += 1
        targets = [(tid, float(x), float(y)) for tid, (x, y, _) in zip(ids, stable) if tid is not None]
        self._labels = [(pixel, tid) for tid, (_, _, pixel) in zip(ids, stable) if tid is not None]
        self.send(("scene", self._sequence, observed_at, window_start, targets))

    def _preview(self, image, glasses, now):
        cv2, fg = self._cv2, self._fg
        if self._finder.area is not None:
            cv2.polylines(image, [self._finder.area_pixels()], True, (255, 200, 0), 2)
        fg.draw_glasses(image, glasses, self._finder.rejected)
        for (u, v, r), tid in self._labels:
            cv2.putText(image, tid, (int(u - r), int(v + r + 34)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
        scale = PREVIEW_WIDTH / image.shape[1]
        if scale < 1:
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self._previews += 1
            self.send(("frame", self._previews, now, jpeg.tobytes()))


def camera_process(conn, stop):
    """Entry point of the camera process (module level, so spawn can import it)."""
    pipeline = None
    try:
        pipeline = CameraPipeline(conn.send)
        pipeline.open()
        conn.send(("ready",))
        pipeline.run(stop)
    except Exception as exc:
        try:
            conn.send(("error", f"{type(exc).__name__}: {exc}"))
        except OSError:
            pass
    finally:
        if pipeline is not None:
            pipeline.close()
        conn.close()


class CameraVision:
    """Supervisor side of the camera process; observe() never blocks.

    frames: a system_web.FrameHub for the panel preview, or None.
    target: the process entry point (tests pass a fake camera).
    """

    def __init__(self, frames=None, target=camera_process, start_timeout=CAMERA_START_TIMEOUT):
        self.frames = frames
        self.target = target
        self.start_timeout = start_timeout
        self.error = ""
        self._lock = threading.Lock()
        self._scene = None
        self._process = None
        self._conn = None
        self._stop = None
        self._reader = None

    def start(self):
        """Start the camera process and wait until it has opened the camera."""
        context = multiprocessing.get_context("spawn")
        receive, send = context.Pipe(duplex=False)
        self._stop = context.Event()
        self._process = context.Process(target=self.target, args=(send, self._stop),
                                        name="camera", daemon=True)
        self._process.start()
        send.close()   # only the child writes; its exit then gives EOF here
        self._conn = receive
        try:
            if not receive.poll(self.start_timeout):
                raise RuntimeError("camera process did not start in time")
            message = receive.recv()
        except (EOFError, OSError):
            message = ("error", "camera process ended during start-up")
        except RuntimeError as exc:
            message = ("error", str(exc))
        if message[0] != "ready":
            self.close()
            raise RuntimeError(f"camera: {message[1]}")
        self._reader = threading.Thread(target=self._read, name="camera-reader", daemon=True)
        self._reader.start()

    def _read(self):
        try:
            while True:
                message = self._conn.recv()
                if message[0] == "scene":
                    _, sequence, observed_at, window_start, targets = message
                    scene = Scene(sequence, observed_at,
                                  tuple(GlassTarget(tid, x, y) for tid, x, y in targets), window_start)
                    with self._lock:
                        self._scene = scene
                elif message[0] == "frame" and self.frames is not None:
                    self.frames.publish(*message[1:])
                elif message[0] == "error":
                    self.error = f"camera stopped: {message[1]}"
        except (EOFError, OSError):
            if not self.error:
                self.error = "camera process ended"

    @property
    def alive(self):
        return self._process is not None and self._process.is_alive()

    def observe(self, now):
        with self._lock:
            scene = self._scene
        if scene is None:
            suffix = f": {self.error}" if self.error else ""
            raise ValueError(f"no camera observation yet{suffix}")
        return scene  # staleness is the supervisor's check (SCENE_MAX_AGE)

    def close(self):
        if self._stop is not None:
            self._stop.set()
        if self._process is not None:
            self._process.join(CAMERA_STOP_TIMEOUT)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(CAMERA_STOP_TIMEOUT)
        if self._conn is not None:
            self._conn.close()
        if self._reader is not None:
            self._reader.join(CAMERA_STOP_TIMEOUT)
