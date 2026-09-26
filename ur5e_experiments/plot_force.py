"""
Live plot of the UR5e's built-in force/torque sensor.

Two stacked charts share a scrolling time axis (the last PLOT_WINDOW seconds):
  top     force  Fx, Fy, Fz and |F|   [N]
  bottom  torque Tx, Ty, Tz           [Nm]
Forces and torques have different units, so they get separate charts instead of
two y-scales on one. The panel right of each chart shows the current values and
doubles as the legend. The y-axes autoscale but never shrink below MIN_*_SPAN,
so sensor noise at rest does not fill the whole chart.

Values come from getActualTCPForce(): the wrench at the TCP, expressed in the
base frame, relative to the last zeroFtSensor() (the pick tasks zero it before
each contact move, so expect a few N offset at rest). A background thread
samples at SAMPLE_PERIOD, independent of the drawing rate. Only the receive
interface is used, so this script never moves the robot and can run alongside
the pendant or another script.

Keys (in the plot window):
  space   pause / resume (sampling continues, the plot freezes)
  q       quit (matplotlib default)

Usage:
  python plot_force.py          connect to the robot at IP
  python plot_force.py --demo   synthetic data, no robot needed

Requires: pip install ur_rtde matplotlib numpy
"""

import argparse
import collections
import math
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

IP = "192.168.1.20"  # same as follow_april_tag.IP (not imported: that module has side effects)

# --- sampling / plotting ---------------------------------------------------
SAMPLE_PERIOD = 0.01     # s (RTDE publishes at 500 Hz; 100 Hz is plenty to see contacts)
PLOT_WINDOW = 10.0       # s of history shown
FRAME_INTERVAL = 50      # ms between redraws
STALE_TIMEOUT = 0.5      # s without a new sample -> show "no data"
MIN_FORCE_SPAN = 10.0    # N, smallest y-range of the force chart
MIN_TORQUE_SPAN = 1.0    # Nm, smallest y-range of the torque chart

# --- style (light chart surface, validated categorical slots 1-3) ----------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
CRITICAL = "#d03b3b"
AXIS_COLORS = ("#2a78d6", "#eb6834", "#1baf7a")  # x blue, y orange, z aqua


class Sampler(threading.Thread):
    """Reads the wrench in the background into a ring buffer of (t, wrench)."""

    def __init__(self, read_wrench):
        super().__init__(daemon=True)
        self.read_wrench = read_wrench
        self.buf = collections.deque(maxlen=int(PLOT_WINDOW / SAMPLE_PERIOD) + 10)
        self.lock = threading.Lock()
        self.running = True
        self.error = None

    def run(self):
        next_t = time.monotonic()
        while self.running:
            try:
                w = self.read_wrench()
            except Exception as e:  # keep the window alive and show the error
                self.error = str(e)
                time.sleep(0.2)
                continue
            with self.lock:
                self.buf.append((time.monotonic(), w))
            next_t += SAMPLE_PERIOD
            time.sleep(max(0.0, next_t - time.monotonic()))

    def snapshot(self):
        with self.lock:
            data = list(self.buf)
        if not data:
            return np.empty(0), np.empty((0, 6))
        t, w = zip(*data)
        return np.array(t), np.array(w, dtype=float)


def demo_wrench():
    """Noise plus a periodic 'press down' contact, roughly like a pick."""
    t = time.monotonic()
    phase = t % 6.0
    press = 25.0 * math.sin(math.pi * (phase - 3.0) / 1.5) if 3.0 < phase < 4.5 else 0.0
    n = np.random.normal(0.0, [0.4, 0.4, 0.5, 0.02, 0.02, 0.01])
    return [1.5 + n[0], -0.8 + n[1], -2.0 - press + n[2],
            0.05 + 0.02 * press / 25 + n[3], -0.1 + n[4], n[5]]


def style_axes(ax, unit):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(BASELINE)
    ax.grid(True, axis="y", color=GRID, linewidth=0.8)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)
    ax.set_ylabel(unit, color=MUTED, fontsize=9, rotation=0, ha="right", va="center")
    ax.axhline(0.0, color=BASELINE, linewidth=0.8, zorder=1)
    ax.set_xlim(-PLOT_WINDOW, 0.0)


def make_readout(ax, entries):
    """Right-hand panel: colored swatch + name + live value, one row per series.

    Text stays in ink colors; the swatch carries the series identity.
    """
    values = []
    for i, (name, color, dashed) in enumerate(entries):
        y = 0.9 - i * 0.2
        ax.plot([1.03, 1.08], [y, y], transform=ax.transAxes, clip_on=False,
                color=color, linewidth=2.5, linestyle="--" if dashed else "-",
                solid_capstyle="round")
        ax.text(1.10, y, name, transform=ax.transAxes, va="center",
                color=INK_SECONDARY, fontsize=10)
        values.append(ax.text(1.42, y, "", transform=ax.transAxes, va="center",
                              ha="right", color=INK, fontsize=11,
                              family="monospace", fontweight="bold"))
    return values


def autoscale(ax, arrays, min_span):
    lo = min(0.0, *(a.min() for a in arrays))
    hi = max(0.0, *(a.max() for a in arrays))
    pad = 0.1 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    if hi - lo < min_span:
        mid = 0.5 * (hi + lo)
        lo, hi = mid - min_span / 2, mid + min_span / 2
    ax.set_ylim(lo, hi)


def main():
    parser = argparse.ArgumentParser(description="Live plot of the UR5e F/T sensor.")
    parser.add_argument("--demo", action="store_true", help="synthetic data, no robot")
    args = parser.parse_args()

    r = None
    if args.demo:
        read_wrench = demo_wrench
        source = "demo data"
    else:
        import rtde_receive
        r = rtde_receive.RTDEReceiveInterface(IP)
        read_wrench = r.getActualTCPForce
        source = IP

    sampler = Sampler(read_wrench)
    sampler.start()

    fig, (ax_f, ax_t) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=0.07, right=0.72, top=0.86, bottom=0.08, hspace=0.25)
    fig.canvas.manager.set_window_title("UR5e force/torque")
    fig.text(0.07, 0.945, "TCP force / torque", fontsize=15, color=INK,
             fontweight="bold")
    status = fig.text(0.07, 0.905, "", fontsize=9, color=INK_SECONDARY)

    style_axes(ax_f, "N")
    style_axes(ax_t, "Nm")
    ax_t.set_xlabel("time [s]", color=MUTED, fontsize=9)
    ax_f.set_title("Force", loc="left", color=INK_SECONDARY, fontsize=10)
    ax_t.set_title("Torque", loc="left", color=INK_SECONDARY, fontsize=10)

    f_lines = [ax_f.plot([], [], color=c, linewidth=1.5, zorder=3)[0] for c in AXIS_COLORS]
    norm_line = ax_f.plot([], [], color=INK_SECONDARY, linewidth=1.5,
                          linestyle="--", zorder=2)[0]
    t_lines = [ax_t.plot([], [], color=c, linewidth=1.5, zorder=3)[0] for c in AXIS_COLORS]

    f_vals = make_readout(ax_f, [("Fx", AXIS_COLORS[0], False),
                                 ("Fy", AXIS_COLORS[1], False),
                                 ("Fz", AXIS_COLORS[2], False),
                                 ("|F|", INK_SECONDARY, True)])
    t_vals = make_readout(ax_t, [("Tx", AXIS_COLORS[0], False),
                                 ("Ty", AXIS_COLORS[1], False),
                                 ("Tz", AXIS_COLORS[2], False)])

    state = {"paused": False}

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        t, w = sampler.snapshot()
        now = time.monotonic()
        stale = len(t) == 0 or now - t[-1] > STALE_TIMEOUT
        if sampler.error and stale:
            status.set_text(f"{source}  ·  no data: {sampler.error}")
            status.set_color(CRITICAL)
        elif stale:
            status.set_text(f"{source}  ·  no data")
            status.set_color(CRITICAL)
        else:
            status.set_text(f"{source}  ·  base frame  ·  "
                            f"{'PAUSED (space to resume)' if state['paused'] else 'live (space to pause)'}")
            status.set_color(INK_SECONDARY)
        if state["paused"] or len(t) == 0:
            return

        x = t - now
        norm = np.linalg.norm(w[:, :3], axis=1)
        for i in range(3):
            f_lines[i].set_data(x, w[:, i])
            t_lines[i].set_data(x, w[:, 3 + i])
            f_vals[i].set_text(f"{w[-1, i]:+7.2f} N")
            t_vals[i].set_text(f"{w[-1, 3 + i]:+6.3f} Nm")
        norm_line.set_data(x, norm)
        f_vals[3].set_text(f"{norm[-1]:7.2f} N")

        autoscale(ax_f, [w[:, 0], w[:, 1], w[:, 2], norm], MIN_FORCE_SPAN)
        autoscale(ax_t, [w[:, 3], w[:, 4], w[:, 5]], MIN_TORQUE_SPAN)

    anim = FuncAnimation(fig, update, interval=FRAME_INTERVAL, cache_frame_data=False)
    try:
        plt.show()
    finally:
        sampler.running = False
        if r is not None:
            r.disconnect()
    del anim


if __name__ == "__main__":
    main()
