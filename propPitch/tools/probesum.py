#!/usr/bin/env python3
"""Read a prop log: how well the blade model's thrust explains the measured
forward aero force.  Per tick, ``J - model`` is the airframe's drag plus the
model's error; it should depend on speed, not on the angle flown.

    ./propPitch/tools/probesum.py logs/LOG15200     # fly it with LOG_INTERVAL_UT=0

Prints, per angle flown (rounded), the mean J, model and J - model in
speed bands, then the least-squares slope of J on the model (1 if the model
is right up to a pitch-independent drag) over the ticks of each band.
"""
import re
import sys
from collections import defaultdict

PAT = re.compile(r"TRACK spd=\s*([\d.]+).*?pitch=\s*([\d.]+) opt=\s*([\d.]+) "
                 r"J=\s*([-+\d.]+) model=\s*([-+\d.]+) rpm=\s*([\d.]+)")


def main():
    rows = []
    last = None
    for line in open(sys.argv[1]):
        m = PAT.search(line)
        if m:
            row = tuple(float(x) for x in m.groups())
            # J is read before the tick's command: keep ticks whose angle
            # did not change, so J and the model describe the same angle.
            if last is not None and abs(row[1] - last) < 0.05:
                rows.append(row)
            last = row[1]
    bands = defaultdict(list)
    for spd, pitch, opt, j, model, rpm in rows:
        bands[int(spd // 20) * 20].append((pitch, opt, j, model, rpm))
    for band in sorted(bands):
        pts = bands[band]
        print("speed %3d-%3d m/s, %d ticks" % (band, band + 20, len(pts)))
        by = defaultdict(list)
        for p, o, j, mo, rpm in pts:
            by["opt" if abs(p - o) < 0.05 else "%.0f" % p].append((j, mo, rpm))
        for k in sorted(by, key=lambda k: (k != "opt", float(k) if k != "opt" else 0)):
            v = by[k]
            n = len(v)
            print("   %4s  n=%3d  J=%+7.2f  model=%+7.2f  J-model=%+6.2f  rpm=%4.0f"
                  % (k, n, sum(x[0] for x in v) / n, sum(x[1] for x in v) / n,
                     sum(x[0] - x[1] for x in v) / n, sum(x[2] for x in v) / n))
        n = len(pts)
        if n > 2:
            mx = sum(p[3] for p in pts) / n
            my = sum(p[2] for p in pts) / n
            sxx = sum((p[3] - mx) ** 2 for p in pts)
            sxy = sum((p[3] - mx) * (p[2] - my) for p in pts)
            if sxx > 0:
                print("   slope J/model %.2f" % (sxy / sxx))


if __name__ == "__main__":
    main()
