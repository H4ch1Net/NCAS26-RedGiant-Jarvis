#!/usr/bin/env python3
"""Run a Jarvis program against the simulated EV3.

    python3 sim/run.py jarvis/jarvis.py
    python3 sim/run.py jarvis/jarvis_ii.py --plot out.png

The program file is executed unchanged; only the ``pybricks`` imports are
redirected to the mock package in this folder.
"""
import argparse
import copy
import math
import os
import runpy
import sys
import types

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

from world import world, SimTimeout, Tape  # noqa: E402


def _install_thread_stub():
    # The dance demos start background music threads. The simulator has no
    # audio, so record the call instead of spawning a real thread.
    stub = types.ModuleType("_thread")
    stub.start_new_thread = lambda fn, args: world.event("thread", fn.__name__)
    sys.modules["_thread"] = stub


def run_program(path, patches=(), **world_kwargs):
    """Execute ``path`` in the simulator and return the world afterwards.

    ``patches`` is an optional list of (old, new) pairs applied to the source
    text before running -- used to try a proposed fix or a different tuning
    value without editing the real file. The program's globals (``path``,
    ``state`` ...) are left in ``world.globals`` for inspection.
    """
    world.reset(**world_kwargs)
    world.label = os.path.basename(path)
    real_thread = sys.modules.get("_thread")
    _install_thread_stub()
    try:
        with open(path) as f:
            src = f.read()
        for old, new in patches:
            assert old in src, "patch target not found in %s: %r" % (path, old)
            src = src.replace(old, new)
        code = compile(src, path, "exec")
        world.globals = {"__name__": "__main__", "__file__": path,
                         "sim_mark": lambda name: world.event("mark", name)}
        try:
            exec(code, world.globals)
        except SimTimeout:
            world.event("timeout")
    finally:
        if real_thread is not None:
            sys.modules["_thread"] = real_thread
    # reset() swaps in fresh lists, so a shallow copy keeps this run's data
    # intact when the next run starts.
    return copy.copy(world)


def summary(w):
    xs = [p[1] for p in w.trace]
    ys = [p[2] for p in w.trace]
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(w.trace, w.trace[1:]))
    end = w.trace[-1] if w.trace else (0, 0, 0, 0)
    lines = [
        "program        : %s" % w.label,
        "simulated time : %.1f s" % (w.t_ms / 1000.0),
        "distance driven: %.2f m" % (dist / 1000.0),
        "x range        : %.0f .. %.0f mm" % (min(xs), max(xs)),
        "y range        : %.0f .. %.0f mm" % (min(ys), max(ys)),
        "final pose     : x=%.0f mm, y=%.0f mm, heading=%.1f deg" % (end[1], end[2], end[3]),
        "tape crossings : %d" % sum(1 for e in w.events if e[1] == "tape"),
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("program")
    ap.add_argument("--plot", help="save a top-down path plot to this PNG")
    ap.add_argument("--tape", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"),
                    help="put blue boundary tape around this rectangle (mm)")
    ap.add_argument("--drift", type=float, default=1.0,
                    help="right-wheel strength factor, e.g. 0.98 for a 2%% weaker wheel")
    args = ap.parse_args()

    tape = Tape(*args.tape) if args.tape else None
    w = run_program(args.program, tape=tape, right_wheel_scale=args.drift)
    print(summary(w))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.plot([p[1] for p in w.trace], [p[2] for p in w.trace], lw=1.2)
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title(w.label)
        ax.grid(alpha=0.3)
        fig.savefig(args.plot, dpi=120, bbox_inches="tight")
        print("saved", args.plot)


if __name__ == "__main__":
    main()
