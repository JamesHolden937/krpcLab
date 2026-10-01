#!/usr/bin/env python3
"""One-step error of a recording, by flight regime.

    ./kspSim/tools/regimes.py logs/kspsim/ft/a_qs_entry_1.jsonl [--model qs_entry]

For long recordings -- a whole autopilot flight recorded with
``flighttest.py --attach`` -- the one-step comparison of ``fidelity.py``
binned by Mach: per bin the mean and rms of (sim - game) for the aero force
and the torque, normalised by dynamic pressure where that is meaningful, and
the mean alpha.  A bias that lives in one regime shows up in its bin.
"""
import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from kspSim.tools import fidelity as F  # noqa: E402

BINS = [0.0, 0.3, 0.6, 0.9, 1.2, 1.6, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 30.0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--model", default=None)
    args = ap.parse_args()
    h, rows = F.read(args.recording)
    rp = F.Replay(args.model or h["save"], h, rows)
    groups = {}
    for f in rp.forces():
        if f["q"] < 20.0:
            continue
        for lo, hi in zip(BINS, BINS[1:]):
            if lo <= f["mach"] < hi:
                groups.setdefault(lo, []).append(f)
                break
    print("%-10s %6s %6s %7s | %-32s | %-40s" % ("Mach", "n", "alpha", "q", "aero F (sim-game)/q mean  x y z",
                                                 "torque (sim-game) mean / rms  pitch roll yaw"))
    for lo in sorted(groups):
        g = groups[lo]
        n = len(g)
        fa = [sum((f["fa_sim"][k] - f["fa_game"][k]) / f["q"] for f in g) / n for k in range(3)]
        tm = [sum(f["t_sim"][k] - f["t_game"][k] for f in g) / n for k in range(3)]
        tr = [math.sqrt(sum((f["t_sim"][k] - f["t_game"][k]) ** 2 for f in g) / n) for k in range(3)]
        tg = [math.sqrt(sum(f["t_game"][k] ** 2 for f in g) / n) for k in range(3)]
        print("%4.1f-%-5.1f %6d %6.1f %7.0f | %+7.3f %+7.3f %+7.3f          | %+7.0f/%6.0f %+6.0f/%6.0f %+6.0f/%6.0f  (game rms %6.0f %6.0f %6.0f)"
              % (lo, BINS[BINS.index(lo) + 1], n, sum(f["alpha"] for f in g) / n,
                 sum(f["q"] for f in g) / n, fa[0], fa[1], fa[2],
                 tm[0], tr[0], tm[1], tr[1], tm[2], tr[2], tg[0], tg[1], tg[2]))


if __name__ == "__main__":
    main()
