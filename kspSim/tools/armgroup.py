#!/usr/bin/env python3
"""armgroup.py LOG... -- flights grouped by the configuration that flew them.

TEMPORARY TEST HARNESS -- not part of the flight software.  Reads each log's
own ``config:`` line (so concurrent flights cannot be misattributed, as the
harness's "?LOG4186|LOG4187" guesses can be), drops the harness's per-run
fields, and prints per arm: n, intact (all parts), on the runway (|along| <=
1200, |across| <= 40), the cone's exit surplus, and contact speed/sink.
"""
import re
import sys
from collections import defaultdict

HARNESS = ("SAVE_NAME", "LOOP_PACING_GAME_TIME", "TIMESCALE_GOVERNOR",
           "LOG_DIR", "INSTANCE")


def read(path):
    cfg, parts, total, along, across, exit_m, td = None, None, None, None, None, None, None
    for line in open(path, errors="replace"):
        if cfg is None and "config:" in line:
            body = line.split("config:", 1)[1]
            items = [x.strip() for x in body.split(",")]
            cfg = ", ".join(x for x in items if x and not x.startswith(HARNESS)
                            and not x.startswith("defaults_fingerprint"))
        m = re.search(r"h=(\d+) \(needed (\d+)\)", line)
        if m and "HAC -> APPROACH" in line:
            exit_m = int(m.group(1)) - int(m.group(2))
        m = re.search(r"FLARE -> ROLLOUT sink=([-\d.]+) speed=([\d.]+)", line)
        if m:
            td = (float(m.group(2)), float(m.group(1)))
        m = re.search(r"along ([-+\d]+), across ([-+\d]+)\).*?(\d+) of (\d+) parts", line)
        if m and "DOWN" in line:
            along, across = int(m.group(1)), int(m.group(2))
            parts, total = int(m.group(3)), int(m.group(4))
    return cfg, parts, total, along, across, exit_m, td


def main():
    arms = defaultdict(list)
    for path in sys.argv[1:]:
        cfg, *rest = read(path)
        if cfg is None:
            continue
        arms[cfg].append((path.split("/")[-1],) + tuple(rest))
    for cfg, rows in arms.items():
        intact = sum(1 for r in rows if r[1] is not None and r[1] == r[2])
        runway = sum(1 for r in rows if r[1] == r[2] and r[3] is not None
                     and abs(r[3]) <= 1200 and abs(r[4]) <= 40)
        exits = [r[5] for r in rows if r[5] is not None]
        print("== %s" % (cfg or "(defaults)"))
        print("   n=%d intact=%d on-runway=%d  exit %s" % (
            len(rows), intact, runway,
            " ".join("%+d" % e for e in sorted(exits))))
        for r in rows:
            td = "td %.0f m/s sink %.1f" % r[6] if r[6] else "td -"
            pos = ("along %+d across %+d %d/%d" % (r[3], r[4], r[1], r[2])
                   if r[1] is not None else "destroyed")
            print("   %-8s exit %6s  %-24s %s" % (
                r[0], "%+d" % r[5] if r[5] is not None else "-", td, pos))


if __name__ == "__main__":
    main()
