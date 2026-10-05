#!/usr/bin/env python3
"""Per savefly batch: where each flight touched down and stopped, by arm.

Touchdown is the first ROLLOUT tick whose ``xt`` has been computed (the
very first reads +0.0); the stop is the batch file's own along/across.
"ok" = all parts kept and stopped within half the runway length along and
half its width across (``Config.RUNWAY_LENGTH_M`` / ``RUNWAY_WIDTH_M``).

    ./spaceplane/tools/rwysum.py logs/sav-flarelat-1005.txt
"""
import os
import re
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from spaceplane.config import Config  # noqa: E402

LINE = re.compile(r"^(arm\d+) ksp\d+ +\d+ +(\S+) .*along +([-+]\d+) +across +"
                  r"([-+]\d+) .* (\d+) parts .*(LOG\d+)")


def parts_at_start(log):
    try:
        for line in open(os.path.join(ROOT, "logs", log), errors="replace"):
            m = re.search(r"vessel: .* (\d+) parts", line)
            if m:
                return int(m.group(1))
    except OSError:
        pass
    return 31


def touchdown(log):
    try:
        for line in open(os.path.join(ROOT, "logs", log), errors="replace"):
            if "ROLLOUT  alt" not in line:
                continue
            xt = re.search(r"xt= *([-+\d.]+)", line)
            rwy = re.search(r"rwy= *([-\d.]+)", line)
            if xt and float(xt.group(1)) != 0.0:
                return float(rwy.group(1)) if rwy else None, float(xt.group(1))
    except OSError:
        pass
    return None, None


def main():
    cfg = Config()
    half_l, half_w = cfg.RUNWAY_LENGTH_M / 2, cfg.RUNWAY_WIDTH_M / 2
    arms, rows = {}, []
    for path in sys.argv[1:]:
        for line in open(path):
            m = re.match(r"^(arm\d+)='(.*)'", line)
            if m:
                arms[m.group(1)] = m.group(2)
            m = LINE.match(line)
            if m:
                arm, save, al, ac, parts, log = m.groups()
                rwy, xt = touchdown(log)
                ok = (int(parts) == parts_at_start(log) and abs(int(al)) <= half_l
                      and abs(int(ac)) <= half_w)
                rows.append((arm, save, int(al), int(ac), int(parts), log,
                             rwy, xt, ok))
    for arm in sorted(arms):
        r = [x for x in rows if x[0] == arm]
        print("%s  %s" % (arm, arms[arm]))
        for x in sorted(r, key=lambda x: x[1]):
            print("   %-15s stop %+6d %+5d  parts %2d  td rwy %6s xt %7s  %s  %s"
                  % (x[1], x[2], x[3], x[4],
                     "-" if x[6] is None else "%.0f" % x[6],
                     "-" if x[7] is None else "%+.1f" % x[7], x[5],
                     "OK" if x[8] else ""))
        xts = [abs(x[7]) for x in r if x[7] is not None]
        print("   => ok %d/%d   |td xt| median %s" % (
            sum(x[8] for x in r), len(r),
            "%.0f" % st.median(xts) if xts else "-"))


if __name__ == "__main__":
    main()
