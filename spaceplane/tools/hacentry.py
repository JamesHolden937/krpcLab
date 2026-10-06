#!/usr/bin/env python3
"""The glide-to-cone handover, one line per flight, grouped by arm.

The bank the glide was flying, the cone's first commands, whether the
vehicle departed (flown bank past +-130 deg in the cone's first twenty
telemetry lines) and where it stopped.  Written for the departures the
cone's weave caused by opening with a bank reversal at alpha ~42
(``HAC_WEAVE_FIRST_WITH_BANK``).

    ./spaceplane/tools/hacentry.py logs/LOG66{32..61}
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from conexit import arm_of  # noqa: E402

BANK = re.compile(r"bank=\s*([-+]?\d+\.\d)")
FLOWN = re.compile(r"bnk=\s*([-+]?\d+\.\d)")
AOA = re.compile(r"aoa=\s*(\d+\.\d)")
DOWN = re.compile(r"DOWN: .*?\(along ([+-]\d+), across ([+-]\d+)\)")
HALF_LENGTH, HALF_WIDTH = 1200, 35


def one(path):
    glide = alpha = None
    cone = []
    departed = False
    seen = False
    for line in open(path, errors="replace"):
        if "GLIDE -> HAC" in line:
            seen = True
            continue
        if not seen and "] GLIDE " in line:
            b, a = BANK.search(line), AOA.search(line)
            glide = float(b.group(1)) if b else glide
            alpha = float(a.group(1)) if a else alpha
        elif seen and "] HAC " in line and len(cone) < 20:
            b, f = BANK.search(line), FLOWN.search(line)
            if b:
                cone.append(float(b.group(1)))
            if f and abs(float(f.group(1))) > 130.0:
                departed = True
    if not seen:
        return None
    text = open(path, errors="replace").read()
    d = DOWN.search(text)
    stop = (int(d.group(1)), int(d.group(2))) if d else None
    return glide, alpha, cone[1:3], departed, stop, arm_of(text)


def main(paths):
    arms = {}
    for path in paths:
        got = one(path)
        name = os.path.basename(path)
        if got is None:
            print("%-8s no cone" % name)
            continue
        glide, alpha, first, departed, stop, arm = got
        on = (stop is not None and abs(stop[0]) <= HALF_LENGTH
              and abs(stop[1]) <= HALF_WIDTH)
        arms.setdefault(arm, []).append((departed, on, stop is not None))
        print("%-8s glide %+6.1f at %4.1f  cone %-14s %-6s stop %s%s"
              % (name, glide or 0.0, alpha or 0.0,
                 " ".join("%+.0f" % b for b in first),
                 "DEPART" if departed else "",
                 "%+d/%+d" % stop if stop else "-", "  RWY" if on else ""))
    for arm in sorted(arms):
        rows = arms[arm]
        print("  %s\n    n=%d departed %d, stopped %d, on the runway %d"
              % (arm, len(rows), sum(r[0] for r in rows),
                 sum(r[2] for r in rows), sum(r[1] for r in rows)))


if __name__ == "__main__":
    main(sys.argv[1:])
