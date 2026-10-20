#!/usr/bin/env python3
"""The turn-shape model's predicted flight against the flight itself.

    ./ascent/tools/shapetrack.py LOG13001 [LOG13002 ...]

Reads a log's ``turn shape track:`` (t:height/downrange/speed from liftoff)
and its ASCENT telemetry, and prints both at every predicted point with the
difference, then the predicted and flown separation.
"""

import math
import re
import sys

TRACK = re.compile(r"turn shape track: (.*)")
SOLVE = re.compile(r"turn shape: solved (\S+) \(orbit (\S+) t, sep (\S+) m "
                   r"(\S+) m out at (\S+) s")
LIFT = re.compile(r"^\[\s*(\S+)\] phase LAUNCH -> ASCENT")
TEL = re.compile(r"^\[\s*(\S+)\] (?:ASCENT|THROTTLE_DOWN)\s+alt=\s*(\S+) "
                 r"dr=\s*(\S+) va=\s*(\S+) vu=\s*(\S+)")
SEP = re.compile(r"^\[\s*(\S+)\] separating: alt=(\S+) downrange=(\S+)")
ORBIT = re.compile(r"orbit: .* mass (\S+) t")


def read(path):
    track, t0, tel, solve, sep, orbit = [], None, [], None, None, None
    for line in open(path, errors="replace"):
        m = TRACK.search(line)
        if m:
            for p in m.group(1).split():
                t, rest = p.split(":")
                h, dr, v = rest.split("/")
                track.append((float(t), float(h), float(dr), float(v)))
            continue
        m = SOLVE.search(line)
        if m:
            solve = tuple(float(x) for x in m.groups())
        m = LIFT.search(line)
        if m:
            t0 = float(m.group(1))
        m = TEL.search(line)
        if m and t0 is not None:
            ut, alt, dr, va, vu = (float(x) for x in m.groups())
            tel.append((ut - t0, alt, dr, math.hypot(va, vu)))
        m = SEP.search(line)
        if m and t0 is not None:
            sep = (float(m.group(1)) - t0, float(m.group(2)),
                   float(m.group(3)))
        m = ORBIT.search(line)
        if m:
            orbit = float(m.group(1))
    return track, tel, solve, sep, orbit


def at(tel, t):
    for a, b in zip(tel, tel[1:]):
        if a[0] <= t <= b[0]:
            f = (t - a[0]) / max(1e-9, b[0] - a[0])
            return tuple(x + (y - x) * f for x, y in zip(a[1:], b[1:]))
    return None


def main(paths):
    for path in paths:
        track, tel, solve, sep, orbit = read(path)
        print("== %s" % path)
        if not track or not tel:
            print("   no track or no telemetry")
            continue
        print("   %5s %8s %8s %6s | %8s %8s %6s | %7s %7s %5s"
              % ("t", "h mod", "dr mod", "v mod", "h fly", "dr fly", "v fly",
                 "dh", "ddr", "dv"))
        for t, h, dr, v in track:
            f = at(tel, t)
            if f is None:
                continue
            print("   %5.0f %8.0f %8.0f %6.0f | %8.0f %8.0f %6.0f | %+7.0f "
                  "%+7.0f %+5.0f" % (t, h, dr, v, f[0], f[1], f[2],
                                     h - f[0], dr - f[1], v - f[2]))
        if solve:
            print("   sep model: %.0f s, %.0f m, %.0f m out; orbit %.2f t "
                  "(shape %.3f)" % (solve[4], solve[2], solve[3], solve[1],
                                    solve[0]))
        if sep:
            print("   sep flown: %.0f s, %.0f m, %.0f m out; orbit %s t"
                  % (sep[0], sep[1], sep[2],
                     "%.2f" % orbit if orbit else "-"))


if __name__ == "__main__":
    main(sys.argv[1:])
