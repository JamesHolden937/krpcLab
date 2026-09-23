#!/usr/bin/env python3
"""What angle of attack did this airframe actually hold, against ``q``?

The reader for a ``BROADSIDE_PROBE_DEG`` flight: command 90 degrees the
whole way down and bin what comes back by dynamic pressure.  docs/spaceplane
has the old craft's curve, taken this way on LOG1615 -- 90 degrees to about
500 Pa, 48 at 1 kPa, 26 at 4 kPa, "available above roughly 45 km and nowhere
below it".  That curve is a property of *that* airframe and 15 kN m of
reaction wheel; a craft with working control surfaces should hold it deeper,
because a surface's authority grows with ``q`` exactly as the disturbance
does while a wheel's does not.

    ./spaceplane/tools/alphaceiling.py logs/LOG2881
"""
import re
import sys

LINE = re.compile(r'aoa=\s*([\d.-]+)/\s*([\d.-]+).*?q=\s*(\d+)')
BINS = [0, 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 5000,
        6500, 8000, 10000, 15000, 10 ** 9]


def main(paths):
    print("%-14s %9s %5s %7s %7s" % ("log", "q band", "n", "cmd", "held"))
    for path in paths:
        rows = []
        for line in open(path, errors="ignore"):
            m = LINE.search(line)
            if m:
                rows.append((float(m.group(3)), float(m.group(1)),
                             float(m.group(2))))
        for lo, hi in zip(BINS, BINS[1:]):
            sel = [r for r in rows if lo <= r[0] < hi]
            if not sel:
                continue
            # The *best* angle held in the band, which is what a ceiling is:
            # a low sample is a transient, the same argument ``Holdable``
            # makes for keeping the highest saturated sample per bin.
            held = max(r[2] for r in sel)
            cmd = max(r[1] for r in sel)
            print("%-14s %4d-%-4d %5d %7.1f %7.1f"
                  % (path.split("/")[-1], lo, min(hi, 99999), len(sel),
                     cmd, held))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
