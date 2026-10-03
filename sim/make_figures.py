#!/usr/bin/env python3
"""Regenerate every picture in docs/images/ from simulated runs.

    pip install matplotlib numpy pillow
    python3 sim/make_figures.py

Each figure is made by running the real program file through the simulator
(sim/run.py) and plotting what the virtual robot did.
"""
import bisect
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import animation, patches  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)

from run import run_program, summary  # noqa: E402
from world import Tape  # noqa: E402

OUT = os.path.join(ROOT, "docs", "images")
JARVIS = os.path.join(ROOT, "jarvis", "jarvis.py")
JARVIS_II = os.path.join(ROOT, "jarvis", "jarvis_ii.py")
DEMOS = os.path.join(ROOT, "demos")

# ---- shared look ----------------------------------------------------------
INK = "#1f2937"
MUTED = "#6b7280"
GRID = "#e5e7eb"
BLUE = "#2563eb"
ORANGE = "#ea580c"
GREEN = "#16a34a"
RED = "#dc2626"
TAPE = "#3b82f6"
TILE_A, TILE_B = "#f8fafc", "#eef2f7"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.edgecolor": MUTED,
    "axes.labelcolor": INK,
    "axes.titleweight": "bold",
    "axes.titlesize": 12,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
})

# Retrace starts at this marker, injected in front of the program's final
# "go home" call so the plots can colour the outbound and return legs.
MARK_RETRACE = ("return_home_by_retrace(path, state)\n",
                "sim_mark('retrace')\nreturn_home_by_retrace(path, state)\n")

# Jarvis arena: 6 x 6 tiles of 600 mm. The robot's start point is the origin.
# The tape sits just outside the swept area so a nominal run never touches it.
JARVIS_TAPE = Tape(-150, -150, 3800, 3600)


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))


def mark_time(w, name):
    for t, kind, payload in w.events:
        if kind == "mark" and payload == name:
            return t
    return None


def split_trace(w, t_split):
    tr = np.array(w.trace)
    if t_split is None:
        return tr, tr[:0]
    i = int(np.searchsorted(tr[:, 0], t_split))
    return tr[: i + 1], tr[i:]


def draw_tiles(ax, x0, y0, nx, ny, size=600):
    for i in range(nx):
        for j in range(ny):
            ax.add_patch(patches.Rectangle(
                (x0 + i * size, y0 + j * size), size, size,
                facecolor=TILE_A if (i + j) % 2 == 0 else TILE_B,
                edgecolor="#d1d5db", lw=0.6, zorder=0))


def draw_tape(ax, tape):
    w = tape.width
    ax.add_patch(patches.Rectangle(
        (tape.x0 - w, tape.y0 - w), tape.x1 - tape.x0 + 2 * w, tape.y1 - tape.y0 + 2 * w,
        fill=False, edgecolor=TAPE, lw=3, zorder=1))


def robot_marker(ax, x, y, heading_cw, color=INK, size=110, zorder=6):
    """Small chassis outline with an arrow showing the direction of travel."""
    h = math.radians(-heading_cw)
    c, s = math.cos(h), math.sin(h)
    body = np.array([[-size * .7, -size * .5], [size * .5, -size * .5],
                     [size * .5, size * .5], [-size * .7, size * .5]])
    rot = body @ np.array([[c, s], [-s, c]]) + [x, y]
    ax.add_patch(patches.Polygon(rot, closed=True, facecolor="white",
                                 edgecolor=color, lw=1.5, zorder=zorder))
    ax.annotate("", xy=(x + c * size * 1.1, y + s * size * 1.1), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.5), zorder=zorder + 1)


def colored_path(ax, tr, cmap, lw=2.0, zorder=3, vmin=None, vmax=None):
    pts = tr[:, 1:3].reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, lw=lw, zorder=zorder)
    lc.set_array(tr[:-1, 0] / 1000.0)
    if vmin is not None:
        lc.set_clim(vmin, vmax)
    ax.add_collection(lc)
    return lc


def jarvis_planned_path():
    """The intended route of jarvis.py, worked out by hand from its constants."""
    pts = [(0, 0)]
    for lane in range(6):
        y = lane * 300
        east = lane % 2 == 0
        if lane == 5:  # westbound lane with the obstacle detour
            pts += [(2400, y), (2400, y - 300), (1200, y - 300), (1200, y), (0, y)]
        else:
            pts.append((3600, y) if east else (0, y))
            pts.append((3600, y + 300) if east else (0, y + 300))
    return np.array(pts)


# ---- figures ----------------------------------------------------------------

def fig_hardware():
    fig, ax = plt.subplots(figsize=(10, 5.6))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 56)
    ax.axis("off")

    ax.add_patch(patches.FancyBboxPatch((38, 16), 24, 26, boxstyle="round,pad=0.6,rounding_size=2",
                                        facecolor="#f3f4f6", edgecolor=INK, lw=2))
    ax.add_patch(patches.Rectangle((42, 30), 16, 9, facecolor="#cbd5e1", edgecolor=INK))
    ax.text(50, 34.5, "EV3 brick\nev3dev + Pybricks", ha="center", va="center", fontsize=9, color=INK)
    for i, (dx, dy) in enumerate([(0, 0), (-3, -3), (3, -3), (0, -6)]):
        ax.add_patch(patches.Circle((50 + dx, 24 + dy), 1.2, facecolor="white", edgecolor=INK))
    ax.text(50, 14.2, "speaker · status light · buttons", ha="center", fontsize=8, color=MUTED)

    def box(x, y, title, sub, color):
        ax.add_patch(patches.FancyBboxPatch((x, y), 24, 8, boxstyle="round,pad=0.4,rounding_size=1.2",
                                            facecolor="white", edgecolor=color, lw=2))
        ax.text(x + 12, y + 5.4, title, ha="center", va="center", fontsize=10, fontweight="bold", color=INK)
        ax.text(x + 12, y + 2.4, sub, ha="center", va="center", fontsize=8, color=MUTED)

    outs = [  # (y, port, title, sub)
        (44, "A", "Right drive motor", "Motor(Port.A)"),
        (30, "D", "Left drive motor", "Motor(Port.D)"),
        (16, "C", "Claw motor (worm gear)", "Motor(Port.C) · run_target(200°)"),
    ]
    for y, port, title, sub in outs:
        box(2, y, title, sub, ORANGE)
        ax.annotate("", xy=(37.4, 29 + (y - 30) * 0.45 + 4), xytext=(26.4, y + 4),
                    arrowprops=dict(arrowstyle="-", color=ORANGE, lw=2))
        ax.text(36.2, 29 + (y - 30) * 0.45 + 5, port, fontsize=10, fontweight="bold", color=ORANGE)

    ins = [
        (40, "S3", "Gyro sensor", "GyroSensor(Port.S3) · heading"),
        (24, "S4", "Color sensor", "ColorSensor(Port.S4) · blue tape"),
    ]
    for y, port, title, sub in ins:
        box(74, y, title, sub, BLUE)
        ax.annotate("", xy=(62.6, 29 + (y - 32) * 0.6 + 4), xytext=(73.6, y + 4),
                    arrowprops=dict(arrowstyle="-", color=BLUE, lw=2))
        ax.text(62.8, 29 + (y - 32) * 0.6 + 5, port, fontsize=10, fontweight="bold", color=BLUE)

    ax.text(14, 54, "Outputs (motors)", ha="center", fontsize=11, fontweight="bold", color=ORANGE)
    ax.text(86, 54, "Inputs (sensors)", ha="center", fontsize=11, fontweight="bold", color=BLUE)
    ax.text(50, 4, "DriveBase(left=D, right=A, wheel_diameter=57 mm, axle_track=134 mm)",
            ha="center", fontsize=10, family="DejaVu Sans Mono", color=INK)
    ax.text(50, 0.8, "Ports as wired in jarvis.py and jarvis_ii.py", ha="center", fontsize=8, color=MUTED)
    save(fig, "hardware_ports.png")


def run_jarvis(**kw):
    kw.setdefault("tape", JARVIS_TAPE)
    patches_ = [MARK_RETRACE] + list(kw.pop("patches", []))
    return run_program(JARVIS, patches=patches_, **kw)


def fig_jarvis_path():
    w = run_jarvis()
    print(summary(w))
    out, back = split_trace(w, mark_time(w, "retrace"))

    fig, ax = plt.subplots(figsize=(11, 7))
    draw_tiles(ax, 0, 0, 7, 4)
    draw_tape(ax, Tape(-150, -150, 3800, 2250))
    ax.add_patch(patches.Rectangle((1200, 1500), 1200, 300, facecolor=RED, alpha=0.15,
                                   edgecolor=RED, hatch="//", lw=1.2, zorder=1))
    ax.text(1800, 1650, "obstacle zone\n(tiles 3–4, x 1200–2400)", ha="center", va="center",
            fontsize=9, color=RED, zorder=2)

    plan = jarvis_planned_path()
    ax.plot(plan[:, 0], plan[:, 1], ls="--", color=MUTED, lw=1.2, zorder=2, label="planned route")
    ax.plot(out[:, 1], out[:, 2], color=BLUE, lw=2.2, zorder=3, label="simulated sweep")
    ax.plot(back[:, 1], back[:, 2], color=ORANGE, lw=1.2, ls=(0, (4, 3)), zorder=4,
            label="simulated retrace home")

    for lane in range(6):
        ax.text(-120 if lane % 2 else 3720, lane * 300, "lane %d" % lane, va="center",
                ha="right" if lane % 2 else "left", fontsize=8, color=MUTED)
    robot_marker(ax, 0, 0, 0, color=GREEN)
    ax.text(0, -110, "START / HOME", ha="center", va="top", fontsize=8, color=GREEN, fontweight="bold")

    ax.set_xlim(-450, 4100)
    ax.set_ylim(-300, 2450)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm) — east")
    ax.set_ylabel("y (mm) — north")
    ax.set_title("jarvis.py — six-lane sweep with obstacle detour (simulated)")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
    save(fig, "jarvis_path.png")
    return w


def fig_jarvis_anim(w):
    tr = np.array(w.trace)
    t_ret = mark_time(w, "retrace")
    step = max(1, len(tr) // 160)
    idx = list(range(0, len(tr), step)) + [len(tr) - 1]

    fig, ax = plt.subplots(figsize=(8, 4.8))
    draw_tiles(ax, 0, 0, 7, 4)
    draw_tape(ax, Tape(-150, -150, 3800, 2250))
    ax.add_patch(patches.Rectangle((1200, 1500), 1200, 300, facecolor=RED, alpha=0.15,
                                   edgecolor=RED, hatch="//", lw=1, zorder=1))
    ax.set_xlim(-300, 4000)
    ax.set_ylim(-300, 2250)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("jarvis.py in the simulator", fontsize=11)
    line_out, = ax.plot([], [], color=BLUE, lw=2, zorder=3)
    line_back, = ax.plot([], [], color=ORANGE, lw=1.5, zorder=4)
    body = patches.Polygon(np.zeros((4, 2)), closed=True, facecolor="white", edgecolor=INK, lw=1.5, zorder=6)
    ax.add_patch(body)
    nose, = ax.plot([], [], color=INK, lw=2, zorder=7)
    label = ax.text(0.01, 0.98, "", transform=ax.transAxes, va="top", fontsize=9,
                    family="DejaVu Sans Mono", color=INK)

    def frame(k):
        i = idx[k]
        t, x, y, hd = tr[i]
        if t_ret is None or t <= t_ret:
            line_out.set_data(tr[: i + 1, 1], tr[: i + 1, 2])
            line_back.set_data([], [])
            phase = "sweep"
        else:
            j = int(np.searchsorted(tr[:, 0], t_ret))
            line_out.set_data(tr[: j + 1, 1], tr[: j + 1, 2])
            line_back.set_data(tr[j: i + 1, 1], tr[j: i + 1, 2])
            phase = "retrace home"
        h = math.radians(-hd)
        c, s = math.cos(h), math.sin(h)
        pts = np.array([[-90, -70], [70, -70], [70, 70], [-90, 70]])
        body.set_xy(pts @ np.array([[c, s], [-s, c]]) + [x, y])
        nose.set_data([x, x + 150 * c], [y, y + 150 * s])
        label.set_text("t = %5.1f s   %s" % (t / 1000.0, phase))
        return line_out, line_back, body, nose, label

    anim = animation.FuncAnimation(fig, frame, frames=len(idx), blit=False)
    path = os.path.join(OUT, "jarvis_run.gif")
    anim.save(path, writer=animation.PillowWriter(fps=16), dpi=80)
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))


def fig_turn_controller(w):
    """Zoom in on the first left turn of the jarvis.py run."""
    # The first gyro reset is the startup one; the next is the first turn.
    t0 = next(t for t, k, _ in w.events if k == "gyro_reset" and t > 1000)
    t1 = t0 + 1300
    tr = np.array(w.trace)
    ml = np.array(w.motor_log)
    sel = (tr[:, 0] >= t0) & (tr[:, 0] <= t1)
    msel = (ml[:, 0] >= t0) & (ml[:, 0] <= t1)
    h0 = tr[sel][0, 3]
    t = (tr[sel][:, 0] - t0) / 1000.0
    turned = tr[sel][:, 3] - h0
    gyro = np.round(turned)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6.4), sharex=True,
                                 gridspec_kw=dict(height_ratios=[3, 2], hspace=0.08))
    a1.axhspan(-92, -88, color=GREEN, alpha=0.12, label="±2° tolerance (TURN_TOL)")
    a1.axhspan(-110, -92, color=ORANGE, alpha=0.08, label="slow zone, |error| ≤ 20°")
    a1.axhline(-90, color=GREEN, lw=1, ls="--")
    a1.plot(t, gyro, color=BLUE, lw=2, drawstyle="steps-post", label="gyro.angle() (whole degrees)")
    a1.set_ylabel("heading change (°)")
    a1.set_ylim(-100, 5)
    a1.legend(loc="upper right", fontsize=8)
    a1.set_title("turn_gyro(-90): proportional turn with a slow zone (simulated)")
    a1.grid(color=GRID)
    a1.annotate("stops at %.1f°" % turned[-1], xy=(t[-1], turned[-1]), xytext=(t[-1] - 0.35, -55),
                arrowprops=dict(arrowstyle="->", color=MUTED), fontsize=9, color=INK)

    mt = (ml[msel][:, 0] - t0) / 1000.0
    a2.plot(mt, ml[msel][:, 1], color=ORANGE, lw=1.4, ls="--", label="left motor command")
    a2.plot(mt, ml[msel][:, 2], color=ORANGE, lw=2, label="left motor actual")
    a2.plot(mt, ml[msel][:, 3], color=BLUE, lw=1.4, ls="--", label="right motor command")
    a2.plot(mt, ml[msel][:, 4], color=BLUE, lw=2, label="right motor actual")
    a2.axhline(0, color=MUTED, lw=0.8)
    a2.set_ylabel("motor speed (°/s)")
    a2.set_xlabel("time since turn started (s)")
    a2.legend(loc="lower right", fontsize=8, ncol=2)
    a2.grid(color=GRID)
    save(fig, "turn_controller.png")


def fig_drive_profile():
    """Gyro heading hold vs. no correction, on a robot with a 3% weaker right wheel."""
    weak = dict(right_wheel_scale=0.99)
    with_gyro = run_jarvis(**weak)
    no_gyro = run_jarvis(patches=[("KP_TURNRATE = 8.0", "KP_TURNRATE = 0.0")], **weak)

    fig = plt.figure(figsize=(11, 7.6))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1], hspace=0.35, wspace=0.18)
    for k, (w, title) in enumerate([(with_gyro, "with gyro heading hold (KP_TURNRATE = 8)"),
                                    (no_gyro, "without correction (KP_TURNRATE = 0)")]):
        ax = fig.add_subplot(gs[0, k])
        out, _ = split_trace(w, mark_time(w, "retrace"))
        draw_tiles(ax, -150, -150, 7, 5)
        ax.plot(jarvis_planned_path()[:, 0], jarvis_planned_path()[:, 1], ls="--", color=MUTED, lw=1)
        ax.plot(out[:, 1], out[:, 2], color=BLUE if k == 0 else RED, lw=1.8)
        ax.set_xlim(-300, 4200)
        ax.set_ylim(-400, 2700)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10)
        ax.set_xticks([0, 1200, 2400, 3600])
        ax.tick_params(labelsize=8)

    def lane0(w):
        """Trace of the first lane: from the end of the startup wait to the first turn."""
        tr = np.array(w.trace)
        t_end = next(t for t, k, _ in w.events if k == "gyro_reset" and t > 1000)
        seg = tr[(tr[:, 0] >= 2700) & (tr[:, 0] < t_end)]
        dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])))])
        return seg, dist

    seg, dist = lane0(with_gyro)
    v = np.gradient(dist, seg[:, 0] / 1000.0)

    ax = fig.add_subplot(gs[1, 0])
    ax.plot(dist, v, color=BLUE, lw=2)
    ax.axvspan(3600 - 600, 3600, color=ORANGE, alpha=0.12)
    ax.text(3300, 150, "SLOW_ZONE\n(last 600 mm)", ha="center", fontsize=8, color=ORANGE)
    ax.set_xlabel("distance along lane 0 (mm)")
    ax.set_ylabel("speed (mm/s)")
    ax.set_title("Speed profile: cruise 300 mm/s, ramp to ≥120", fontsize=10)
    ax.grid(color=GRID)
    ax.set_ylim(0, 340)

    ax = fig.add_subplot(gs[1, 1])
    ax.plot(dist, seg[:, 3] - seg[0, 3], color=BLUE, lw=1.6, label="with gyro hold")
    nseg, ndist = lane0(no_gyro)
    ax.plot(ndist, nseg[:, 3] - nseg[0, 3], color=RED, lw=1.6, label="no correction")
    ax.axhspan(-0.8, 0.8, color=GREEN, alpha=0.15, label="±0.8° deadband")
    ax.set_xlabel("distance along lane 0 (mm)")
    ax.set_ylabel("heading error (°)")
    ax.set_title("Heading error on lane 0", fontsize=10)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(color=GRID)
    fig.suptitle("drive_straight_gyro() on a robot whose right wheel is 1% weaker (simulated)",
                 fontweight="bold", y=0.98)
    save(fig, "drive_straight_gyro.png")


def fig_edge_failsafe():
    tape = Tape(-150, -150, 3300, 2400)
    w = run_jarvis(tape=tape, time_limit_ms=60000)
    tr = np.array(w.trace)
    hits = [p for t, k, p in w.events if k == "tape"]
    log = w.globals["path"]

    fig, (ax, tx) = plt.subplots(1, 2, figsize=(13, 5.6), gridspec_kw=dict(width_ratios=[1.7, 1], wspace=0.22))
    draw_tiles(ax, 0, 0, 6, 4)
    draw_tape(ax, Tape(-150, -150, 3300, 2250))
    lc = colored_path(ax, tr, "viridis", lw=2.4)
    # Several hits land on the same spot; label each spot with its hit numbers.
    groups = []
    for n, (sx, sy) in enumerate(hits, 1):
        for g in groups:
            if math.hypot(g[0] - sx, g[1] - sy) < 40:
                g[2].append(n)
                break
        else:
            groups.append((sx, sy, [n]))
    for sx, sy, ns in groups:
        label = str(ns[0]) if len(ns) == 1 else "%d–%d" % (ns[0], ns[-1])
        ax.plot(sx, sy, "o", ms=22, mfc="white", mec=RED, mew=2, zorder=8)
        ax.text(sx, sy, label, ha="center", va="center", fontsize=7.5, fontweight="bold", color=RED, zorder=9)
    robot_marker(ax, 0, 0, 0, color=GREEN)
    ax.set_xlim(-400, 3600)
    ax.set_ylim(-400, 2300)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Tape moved in to x = 3300 mm (first 60 s)", fontsize=10)
    cb = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.02)
    cb.set_label("time (s)")

    tx.axis("off")
    lines = [
        ("What happens at hit 1", True),
        ("· colour sensor reads BLUE on two loops in a row", False),
        ("  (COLOR_DEBOUNCE = 2) → the drive loop stops", False),
        ("· back off EDGE_BACKOFF_MM = 80 mm", False),
        ("· turn left 90°", False),
        ("", False),
        ("Then the main loop carries on as planned", True),
        ("· it doesn't know about the extra left turn,", False),
        ("  so its own U-turn sends the robot along the", False),
        ("  wrong axis and it meets the tape again", False),
        ("· the failsafe keeps it inside the tape every", False),
        ("  time, but the sweep pattern is now rotated", False),
        ("", False),
        ("First moves in the recorded path:", True),
    ]
    lines += [("  " + repr(m), False) for m in log[:8]]
    for i, (text, bold) in enumerate(lines):
        tx.text(0.02, 0.98 - i * 0.047, text, transform=tx.transAxes, va="top", fontsize=9,
                fontweight="bold" if bold else "normal",
                family="DejaVu Sans Mono" if text.startswith("  (") or text.startswith("  ('") else "DejaVu Sans",
                color=INK)
    fig.suptitle("Colour edge failsafe in drive_straight_gyro() (simulated)", fontweight="bold")
    save(fig, "edge_failsafe.png")


def triangle_background(ax):
    draw_tiles(ax, 0, 0, 6, 6)
    ax.add_patch(patches.Polygon([[0, 0], [3600, 0], [3600, 3600]], closed=True,
                                 facecolor=GREEN, alpha=0.08, edgecolor=GREEN, lw=1.5, zorder=1))
    ax.plot([0, 3600], [0, 3600], color=GREEN, lw=1.5, zorder=1)
    ax.plot([100, 3600], [0, 3500], color=GREEN, lw=1, ls=":", zorder=1)
    ax.plot([150, 3450, 3450], [150, 150, 3350], color=GREEN, lw=1, ls=":", zorder=1)
    ax.text(1350, 1650, "x = y diagonal", rotation=45, fontsize=8, color=GREEN)
    ax.text(2600, 900, "sweep region\nx ≥ y", ha="center", fontsize=9, color=GREEN)


def fig_jarvis_ii():
    tape = Tape(-75, -75, 3600, 3600)
    as_written = run_program(JARVIS_II, patches=[MARK_RETRACE], tape=tape)
    fixed = run_program(JARVIS_II, patches=[MARK_RETRACE, ("drive_segment(-lane_len", "drive_segment(lane_len")],
                        tape=tape)
    print(summary(as_written))
    print(summary(fixed))

    fig, axes = plt.subplots(1, 2, figsize=(13, 6.4))
    for ax, w, title in [(axes[0], as_written, "As written: odd lanes use drive_segment(-lane_len)"),
                         (axes[1], fixed, "With the sign flipped: drive_segment(lane_len)")]:
        triangle_background(ax)
        draw_tape(ax, Tape(-75, -75, 3600, 3600))
        out, back = split_trace(w, mark_time(w, "retrace"))
        ax.plot(out[:, 1], out[:, 2], color=BLUE, lw=2, zorder=3, label="sweep")
        ax.plot(back[:, 1], back[:, 2], color=ORANGE, lw=1.1, ls=(0, (4, 3)), zorder=4, label="retrace home")
        robot_marker(ax, 0, 0, 0, color=GREEN, size=90)
        ax.set_xlim(-300, 4300)
        ax.set_ylim(-300, 3900)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
    far = max(p[1] for p in as_written.trace)
    axes[0].annotate("backs out of the arena\n(reaches x ≈ %.0f m)" % (far / 1000.0),
                     xy=(4250, 300), xytext=(2300, 1800), fontsize=9, color=RED,
                     arrowprops=dict(arrowstyle="->", color=RED))
    fig.suptitle("jarvis_ii.py — triangle sweep (simulated)", fontweight="bold")
    save(fig, "jarvis_ii_path.png")


def fig_retrace_log():
    """A 'terminal screenshot' of the move log jarvis.py records and replays."""
    w = run_jarvis()
    log = w.globals["path"]
    lines = ["$ python3 sim/run.py jarvis/jarvis.py", ""]
    lines += summary(w).splitlines()
    lines += ["", ">>> path[:9]      # recorded on the way out"]
    lines += ["    %-22s" % (repr(m),) for m in log[:9]]
    lines += ["    ...  (%d moves total)" % len(log), "",
              ">>> [invert_move(m) for m in reversed(path)][:5]   # replayed going home"]
    inv = {"drive": lambda v: ("drive", -v), "turnL": lambda v: ("turnR", 0), "turnR": lambda v: ("turnL", 0)}
    lines += ["    %r" % (inv[k](v),) for k, v in list(reversed(log))[:5]]

    fig = plt.figure(figsize=(9.2, 0.25 * len(lines) + 0.9))
    fig.patch.set_facecolor("#0f172a")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.add_patch(patches.Rectangle((0, 0.965), 1, 0.035, transform=ax.transAxes, color="#1e293b"))
    for i, c in enumerate(["#ef4444", "#f59e0b", "#22c55e"]):
        ax.add_patch(patches.Circle((0.02 + i * 0.022, 0.982), 0.006, transform=ax.transAxes, color=c))
    for i, line in enumerate(lines):
        color = "#a5f3fc" if line.startswith(("$", ">>>")) else "#e2e8f0"
        ax.text(0.02, 0.93 - i * (0.9 / len(lines)), line, transform=ax.transAxes,
                family="DejaVu Sans Mono", fontsize=9.5, color=color, va="top")
    save(fig, "retrace_log.png")


def fig_daisy():
    # No DriveBase in this demo, so tell the world which motors are the wheels.
    w = run_program(os.path.join(DEMOS, "daisy_dance.py"), forward_sign={"D": -1}, drive_ports=("D", "A"),
                    wheel_diameter=56, axle_track=120)
    block = w.globals["BASE_BLOCK"]
    scales = w.globals["SCALES"]
    lights = w.globals["COLORS"]
    names = ["forward", "back", "spin L", "spin R", "shimmy", "", "", "", "arc left", "arc right"]

    fig = plt.figure(figsize=(13, 6.2))
    gs = fig.add_gridspec(2, 2, width_ratios=[2.1, 1], height_ratios=[3, 1.2], hspace=0.45, wspace=0.18)

    ax = fig.add_subplot(gs[0, 0])
    t = 0
    for (ls, rs, dur), name in zip(block, names):
        ax.add_patch(patches.Rectangle((t / 1000, 0), dur / 1000, ls, color=ORANGE, alpha=0.85))
        ax.add_patch(patches.Rectangle((t / 1000, 0), dur / 1000, rs, color=BLUE, alpha=0.55))
        if name:
            ax.text((t + dur / 2) / 1000 if name != "shimmy" else (t + 500) / 1000, 720, name,
                    ha="center", fontsize=8.5, color=INK)
        t += dur
    ax.axhline(0, color=MUTED, lw=0.8)
    ax.set_xlim(0, 5)
    ax.set_ylim(-800, 800)
    ax.set_xlabel("time within one block (s)")
    ax.set_ylabel("motor speed (°/s)")
    ax.legend(handles=[patches.Patch(color=ORANGE, label="left (Port D)"),
                       patches.Patch(color=BLUE, alpha=0.55, label="right (Port A)")],
              loc="lower right", fontsize=8)
    ax.set_title("BASE_BLOCK — one 5-second move sequence (scale 1.0)", fontsize=10)
    ax.grid(color=GRID)

    cmap = {"GREEN": "#22c55e", "YELLOW": "#facc15", "ORANGE": "#fb923c", "RED": "#ef4444"}
    lax = fig.add_subplot(gs[1, 0])
    for i, (sc, c) in enumerate(zip(scales, lights)):
        lax.add_patch(patches.Rectangle((i * 5, 0), 5, 1, color=cmap[c], ec="white", lw=2))
        lax.text(i * 5 + 2.5, 0.5, "×%.1f" % sc, ha="center", va="center", fontsize=9, color=INK)
    lax.set_xlim(0, 40)
    lax.set_ylim(0, 1)
    lax.set_yticks([])
    lax.set_xticks(range(0, 41, 5))
    lax.set_xlabel("time (s) — the block repeats 8 times; status light colour and speed scale per cycle")

    tax = fig.add_subplot(gs[:, 1])
    tr = np.array(w.trace)
    lc = colored_path(tax, tr, "plasma", lw=1.4)
    tax.autoscale()
    tax.margins(0.08)
    tax.set_aspect("equal")
    tax.set_title("Path over 40 s (simulated)", fontsize=10)
    tax.set_xlabel("x (mm)")
    tax.set_ylabel("y (mm)")
    tax.grid(color=GRID)
    fig.colorbar(lc, ax=tax, fraction=0.05, pad=0.03).set_label("time (s)")
    fig.suptitle("daisy_dance.py", fontweight="bold")
    save(fig, "daisy_dance.png")


def fig_donut():
    """Planned donut layout, worked out from the constants in donut_field.py.

    The simulator's motor model is too rough for full-speed spins (the real
    DriveBase saturates and recovers differently), so this figure shows the
    intended pattern rather than a simulated trace.
    """
    area_x, area_y, margin, step_x, step_y = 2500, 1400, 200, 500, 350
    cols = int((area_x - 2 * margin) // step_x) + 1
    rows = int((area_y - 2 * margin) // step_y) + 1

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.add_patch(patches.Rectangle((0, 0), area_x, area_y, facecolor="#f8fafc", edgecolor=MUTED, lw=1.5))
    ax.add_patch(patches.Rectangle((margin, margin), area_x - 2 * margin, area_y - 2 * margin,
                                   fill=False, edgecolor=MUTED, ls="--", lw=1))
    ax.text(area_x - margin - 10, margin + 15, "MARGIN = 200 mm", ha="right", fontsize=8, color=MUTED)

    route = []
    n = 0
    for r in range(rows):
        y = area_y - margin - r * step_y
        xs = [margin + c * step_x for c in range(cols)]
        if r % 2:
            xs = xs[::-1]
        for x in xs:
            route.append((x, y))
            n += 1
            for k, col in [(1, BLUE), (-1, ORANGE)]:
                ax.add_patch(patches.Circle((x + k * 45, y), 45, fill=False, edgecolor=col, lw=1.4))
            ax.text(x, y - 95, str(n), ha="center", va="top", fontsize=8, color=INK)
    route = np.array(route)
    ax.plot(route[:, 0], route[:, 1], color=MUTED, lw=1.2, ls="-", zorder=1)
    for a, b in zip(route[:-1], route[1:]):
        mid = (a + b) / 2
        ax.annotate("", xy=mid + (b - a) * 0.05, xytext=mid - (b - a) * 0.05,
                    arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.2))
    robot_marker(ax, margin - 130, area_y - margin, 0, color=GREEN, size=60)
    ax.text(margin - 130, area_y - margin + 90, "start", ha="center", fontsize=8, color=GREEN)
    ax.legend(handles=[patches.Patch(edgecolor=BLUE, fill=False, label="donut(+1): spin clockwise"),
                       patches.Patch(edgecolor=ORANGE, fill=False, label="donut(-1): spin counter-clockwise")],
              loc="lower left", fontsize=8, bbox_to_anchor=(0.0, -0.02))
    ax.set_xlim(-200, 2700)
    ax.set_ylim(-150, 1550)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("donut_field.py — %d × %d donut spots in lawn-mower order (planned layout)" % (cols, rows))
    save(fig, "donut_field.png")


def fig_ymca():
    w = run_program(os.path.join(DEMOS, "ymca_dance.py"), forward_sign={"D": -1}, drive_ports=("D", "A"),
                    time_limit_ms=4000)
    ml = np.array(w.motor_log)
    t = ml[:, 0] / 1000
    fig, ax = plt.subplots(figsize=(10, 3.6))
    # Port D is declared COUNTERCLOCKWISE, so flip its physical angle back to
    # the sign the program uses.
    ax.plot(t, -ml[:, 5], color=ORANGE, lw=2, label="left motor angle (Port D)")
    ax.plot(t, ml[:, 6], color=BLUE, lw=2, label="right motor angle (Port A)")
    ax.axhline(0, color=MUTED, lw=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("angle (°)")
    ax.set_ylim(-60, 60)
    ax.set_xlim(0, 4)
    ax.grid(color=GRID)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("ymca_dance.py — each wheel rocks ±45°, one at a time, forever")
    save(fig, "ymca_dance.png")


def main():
    os.makedirs(OUT, exist_ok=True)
    fig_hardware()
    w = fig_jarvis_path()
    fig_turn_controller(w)
    fig_jarvis_anim(w)
    fig_retrace_log()
    fig_drive_profile()
    fig_edge_failsafe()
    fig_jarvis_ii()
    fig_daisy()
    fig_donut()
    fig_ymca()


if __name__ == "__main__":
    main()
