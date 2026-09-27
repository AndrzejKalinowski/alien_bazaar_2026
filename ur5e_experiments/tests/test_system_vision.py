"""Window statistics and target identity for the camera observer (no camera)."""

import pytest

from system_model import GlassTarget
from system_simulator import SimulatedDevices
from system_vision import MATCH_RADIUS, TargetTracker, WindowAccumulator


def feed(window, frames, dt=0.05):
    """frames: list of per-frame detections [(x, y), ...]; returns every closed window."""
    closed = []
    for i, points in enumerate(frames):
        result = window.add([(x, y, None) for x, y in points], i * dt)
        if result is not None:
            closed.append(result)
    return closed


def test_window_publishes_medians_of_glasses_seen_often_enough():
    window = WindowAccumulator(period=0.3, min_frames=5, fraction=0.5, radius=0.015)
    frames = [[(0.100 + 0.001 * (i % 3), 0.2), (0.4, 0.4)] if i % 2 else [(0.101, 0.2)]
              for i in range(7)]
    frames[3].append((0.7, 0.7))  # one-frame reflection
    (stable, observed_at), = feed(window, frames)
    assert observed_at == pytest.approx(0.3)
    assert [(round(x, 3), y) for x, y, _ in stable] == [(0.101, 0.2)]  # 0.4 seen in 3/7


def test_too_few_frames_in_a_window_publish_nothing():
    window = WindowAccumulator(period=0.3, min_frames=5)
    (stable, _), = feed(window, [[(0.1, 0.2)]] * 3, dt=0.15)
    assert stable == []


def test_window_refuses_invalid_coordinates():
    with pytest.raises(ValueError):
        WindowAccumulator().add([(float("nan"), 0.0, None)], 0.0)


def test_tracker_keeps_ids_for_small_moves_and_renews_them_for_jumps():
    tracker = TargetTracker(radius=0.03)
    assert tracker.update([(0.1, 0.1), (0.3, 0.1)]) == ["glass-1", "glass-2"]
    assert tracker.update([(0.31, 0.1), (0.11, 0.1)]) == ["glass-2", "glass-1"]
    assert tracker.update([(0.2, 0.3), (0.31, 0.1)]) == ["glass-3", "glass-2"]


def test_tracker_leaves_ambiguous_matches_out():
    tracker = TargetTracker(radius=0.03)
    tracker.update([(0.1, 0.1)])
    assert tracker.update([(0.08, 0.1), (0.12, 0.1)]) == [None, None]
    tracker = TargetTracker(radius=0.03)
    tracker.update([(0.1, 0.1), (0.14, 0.1)])
    assert tracker.update([(0.12, 0.1)]) == [None]


def test_tracker_forgets_old_ids_and_is_bounded():
    tracker = TargetTracker(radius=MATCH_RADIUS, max_tracks=2, forget=2)
    tracker.update([(0.1, 0.1), (0.3, 0.1)])
    assert tracker.update([(0.5, 0.1)]) == [None]  # full: not published
    for _ in range(3):
        tracker.update([])
    assert tracker.update([(0.5, 0.1)]) == ["glass-3"]


class FixedScene:
    def __init__(self, scene):
        self.scene = scene

    def observe(self, now):
        return self.scene


def test_simulated_robot_can_use_an_external_scene():
    from system_model import Scene
    scene = Scene(1, 0.0, (GlassTarget("glass-7", 0.2, -0.4),))
    devices = SimulatedDevices([], scene_source=FixedScene(scene))
    assert devices.observe(0.0) is scene
    assert devices.snapshot()["camera_targets"]
    assert devices._present("glass-7") and not devices._present("glass-1")
    with pytest.raises(ValueError):
        devices.add_targets(1)
