#!/usr/bin/env python3
"""The polar the airframe actually flies, out of the logs it has flown.

TEMPORARY TEST HARNESS -- not part of the flight software, and it prints to
stdout deliberately.  ``aeroaudit.py`` asks whether the *model* matches the
flight; this asks what the *airframe* is, which is a different question and
the one every terminal constant in ``config.py`` is an answer to.

    ./spaceplane/tools/polar.py                       # every log, subsonic
    ./spaceplane/tools/polar.py --mach 0.45 logs/LOG77*

It exists because ``planeprobe`` was wrong by a factor of 1.8 subsonically
and nothing could contradict it: its numbers were transcribed into
``config.py`` by hand, so the measurement had no consumer that could
disagree with it.  See spaceplane failure 13.  This one is free -- it reads
logs already on disk -- so the polar can be re-taken whenever the airframe,
its mass or its gear state changes, and a constant derived from it can be
checked without a flight.

``act=ClA/CdA`` in the telemetry is recovered from the aerodynamic force kRPC
reports, resolved about the relative wind, so it is a measurement of the
vehicle and not of the table.  Binned on the angle of attack **achieved**,
from kRPC's own ``angle_of_attack``, because what a wing does is a function
of where it is pointed and not of what it was asked for.
"""
import argparse
import glob
import os
import re
import sys

FIELD = re.compile(r"(\w+)=\s*([-+]?[\d.]+)")
ACT = re.compile(r"act=\s*([\d.]+)/\s*([\d.]+)")
AOA = re.compile(r"aoa=\s*([\d.]+)/\s*([-\d.]+)")
PHASES = (" GLIDE ", " APPROACH ", " FLARE ")
SEA_LEVEL_RHO = 1.225


def samples(paths, mach, mass):
    bins = {}
    for path in paths:
        try:
            handle = open(path, errors="replace")
        except OSError:
            continue
        with handle:
            for line in handle:
                if not any(p in line for p in PHASES):
                    continue
                act, aoa = ACT.search(line), AOA.search(line)
                if not act or not aoa:
                    continue
                row = {k: float(v) for k, v in FIELD.findall(line)}
                if row.get("M", 9.0) > mach:
                    continue
                cla, cda = float(act.group(1)), float(act.group(2))
                # A vehicle making no lift at all is one that is not flying:
                # a wreck, or a tick before the force stream settles.  It is
                # not a data point about the wing.
                if cda < 0.5 or cla < 0.1:
                    continue
                key = int(round(float(aoa.group(2)) / 2.0)) * 2
                bins.setdefault(key, []).append((cla / cda, cla))
    return bins


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("logs", nargs="*", default=None)
    p.add_argument("--mach", type=float, default=0.45,
                   help="only samples at or below this Mach (default 0.45)")
    p.add_argument("--mass", type=float, default=6690.0,
                   help="kg, for the 1 g speed column")
    p.add_argument("--min-samples", type=int, default=20)
    args = p.parse_args()

    paths = args.logs or [p for p in sorted(glob.glob("logs/LOG*"))
                          if os.path.basename(p)[3:].isdigit()]
    bins = samples(paths, args.mach, args.mass)
    weight = args.mass * 9.81

    print("the flown polar at M <= %.2f, %d logs, by *achieved* angle of attack"
          % (args.mach, len(paths)))
    print("%6s %8s %8s %8s %8s %8s %10s"
          % ("aoa", "n", "L/D med", "p25", "p75", "ClA med", "1g at s.l."))
    best = None
    for key in sorted(bins):
        rows = bins[key]
        if len(rows) < args.min_samples:
            continue
        ratios = sorted(r[0] for r in rows)
        clas = sorted(r[1] for r in rows)
        middle = ratios[len(ratios) // 2]
        cla = clas[len(clas) // 2]
        speed = (2.0 * weight / (SEA_LEVEL_RHO * cla)) ** 0.5 if cla > 0 else 0.0
        print("%6d %8d %8.2f %8.2f %8.2f %8.1f %10.1f"
              % (key, len(rows), middle, ratios[len(ratios) // 4],
                 ratios[3 * len(ratios) // 4], cla, speed))
        if best is None or middle > best[1]:
            best = (key, middle, cla, speed)
    if best is None:
        # **Not a shrug and not a zero.**  No usable samples is a different
        # answer from "the wing has no lift", and only one of them should be
        # allowed to reach a constant in config.py.
        print("\nno bin reached %d samples -- nothing measured here"
              % args.min_samples)
        return 1
    print("\nbest glide L/D %.2f at %d deg, %.1f m/s at sea level and %.0f kg"
          % (best[1], best[0], best[3], args.mass))
    print("glide angle %.1f deg" % __import__("math").degrees(
        __import__("math").atan(1.0 / best[1])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
