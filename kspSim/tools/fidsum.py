#!/usr/bin/env python3
"""One line per recording: how far the simulator is from the game.

    ./kspSim/tools/fidsum.py logs/kspsim/ft/g_*.jsonl

Columns: the one-step error of the aerodynamic force (rms as % of the
game's median magnitude, body x/y/z) and of the torque (pitch/roll/yaw),
then the median attitude and position error of restarts after 2 and 5 s of
flying the game's own inputs (``fidelity.py`` for the detail).
"""
import math
import os
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from kspSim.tools import fidelity as F  # noqa: E402


def pct(fs, kg, ks, k):
    g = [f[kg][k] for f in fs]
    e = [f[ks][k] - f[kg][k] for f in fs]
    mag = statistics.median(abs(x) for x in g) or 1e-9
    rms = math.sqrt(sum(x * x for x in e) / len(e))
    return 100.0 * rms / max(mag, 1e-9), rms


def main():
    print("%-34s %-20s %-26s %-17s %-17s" % ("recording", "aero F rms %", "torque rms N m",
                                            "2 s att/pos", "5 s att/pos"))
    for path in sys.argv[1:]:
        h, rows = F.read(path)
        try:
            rp = F.Replay(h["save"], h, rows)
        except Exception as exc:                        # noqa: BLE001
            print("%-34s %s" % (os.path.basename(path), exc))
            continue
        fs = rp.forces()
        fa = [f for f in fs if f["q"] > 1.0]
        aero = "/".join("%.0f" % pct(fa, "fa_game", "fa_sim", k)[0] for k in range(3)) if fa else "-"
        tq = "/".join("%.0f" % pct(fs, "t_game", "t_sim", k)[1] for k in range(3))
        span = rows[-1]["t"] - rows[0]["t"]
        hs = [x for x in (2.0, 5.0) if x < span]
        ps = rp.predict(hs, 1.0) if hs else []
        cols = []
        for x in (2.0, 5.0):
            r = [p[x] for p in ps if p.get(x)]
            cols.append("%5.2f deg %6.2f m" % (statistics.median(q["datt"] for q in r),
                                             statistics.median(q["dr"] for q in r)) if r else "-")
        print("%-34s %-20s %-26s %-17s %-17s" % (os.path.basename(path)[2:-6], aero, tq, *cols))


if __name__ == "__main__":
    main()
