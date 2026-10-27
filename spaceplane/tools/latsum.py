#!/usr/bin/env python3
"""latsum.py LOG... -- command latency per arm and phase.

Reads each log's ``latency`` line (``common.pacing.Latency``) and its own
``config:`` line, and prints per arm and phase the median over flights of
each flight's median and p90: game ms from the telemetry's tick to the
command's tick, the wall ms the loop spent between, and the metres the
vehicle flew meanwhile.  Also each arm's wall seconds per flight (the
``wall per phase`` total), which is what a latency fix may cost the farm.
"""
import re
import statistics
import sys
from collections import defaultdict

HARNESS = ("SAVE_NAME", "LOOP_PACING_GAME_TIME", "TIMESCALE_GOVERNOR",
           "LOG_DIR", "INSTANCE", "defaults_fingerprint")
ROW = re.compile(r"(\w+) n=(\d+) game ([\d.]+)/([\d.]+)/([\d.]+) ms "
                 r"wall ([\d.]+)/([\d.]+)/([\d.]+) ms "
                 r"([\d.]+)/([\d.]+)/([\d.]+) m")


def read(path):
    cfg, phases, wall = None, {}, None
    for line in open(path, errors="replace"):
        if cfg is None and "config:" in line:
            items = [x.strip() for x in line.split("config:", 1)[1].split(",")]
            cfg = ", ".join(x for x in items
                            if x and not x.startswith(HARNESS))
        if "latency, telemetry tick" in line:
            for m in ROW.finditer(line):
                phases[m.group(1)] = [float(x) for x in m.groups()[1:]]
        m = re.search(r"wall per phase .*?total (\d+)s", line)
        if m:
            wall = int(m.group(1))
    return cfg, phases, wall


def main():
    arms = defaultdict(list)
    for path in sys.argv[1:]:
        cfg, phases, wall = read(path)
        if phases:
            arms[cfg or "(defaults)"].append((phases, wall))
    for cfg, flights in arms.items():
        walls = [w for _, w in flights if w is not None]
        print("== %s  n=%d  wall/flight %s s" % (
            cfg, len(flights),
            "%.0f" % statistics.median(walls) if walls else "-"))
        order = []
        for phases, _ in flights:
            order += [p for p in phases if p not in order]
        print("   %-9s %4s  %17s  %17s  %13s" % (
            "phase", "n", "game ms med/p90", "wall ms med/p90",
            "m med/p90"))
        for phase in order:
            rows = [f[0][phase] for f in flights if phase in f[0]]
            med = lambda i: statistics.median(r[i] for r in rows)  # noqa: E731
            print("   %-9s %4d  %8.0f %8.0f  %8.0f %8.0f  %6.1f %6.1f" % (
                phase, len(rows), med(1), med(2), med(4), med(5),
                med(7), med(8)))


if __name__ == "__main__":
    main()
