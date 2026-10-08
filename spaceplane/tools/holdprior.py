#!/usr/bin/env python3
"""Write ``HOLDABLE_PRIOR``'s file: the alpha each craft's glide held while
saturated, per ``Holdable`` q bin, from its own logs.

    ./spaceplane/tools/holdprior.py logs/LOG7[0-4]??          # write
    ./spaceplane/tools/holdprior.py --dry-run logs/LOG73??    # print only

Logs are grouped by craft (``vessel:`` line: name and part count, the key the
autopilot looks itself up by).  A tick counts when it is GLIDE, at or above
``--mach`` and ``HOLDABLE_MIN_Q``, and short of its command by more than
``HOLDABLE_SATURATED_DEG`` -- the same test ``Holdable.observe`` uses -- and a
bin is written when it has ``--min-samples`` of them; its value is the median.
The ceiling depends on how the vehicle is flown (failure 23): re-run after
anything that changes the attitude control, from logs flown since.

Writes ``logs/holdprior/<craft>.json``; nothing tracked (it is regenerated
from the logs).
"""
import argparse
import collections
import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from spaceplane.config import Config  # noqa: E402
from spaceplane.trajectory import holdprior_key, holdprior_slug  # noqa: E402

VESSEL = re.compile(r"vessel: (.*?)\s+(\d+) parts")
AOA = re.compile(r"aoa=\s*([-0-9.]+)/\s*([-0-9.]+)")
Q = re.compile(r"\bq=\s*([0-9.]+)")
MACH = re.compile(r"\bM=\s*([0-9.]+)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--mach", type=float, default=1.5)
    ap.add_argument("--min-samples", type=int, default=100)
    ap.add_argument("--dry-run", action="store_true")
    # Bin by Mach band (``HOLDABLE_MACH_EDGES``) as well as q: the trim
    # limit moves with Mach, and a q-only median answers Mach 4 with what
    # Mach 2 held.  Pair with ``HOLDABLE_BY_MACH`` in flight.
    ap.add_argument("--by-mach", action="store_true")
    args = ap.parse_args(argv)
    cfg = Config()
    per = max(1, int(cfg.HOLDABLE_Q_DECADE_BINS))
    sat = float(cfg.HOLDABLE_SATURATED_DEG)
    qmin = float(cfg.HOLDABLE_MIN_Q)
    crafts = collections.defaultdict(lambda: collections.defaultdict(list))
    used = collections.Counter()
    for path in args.logs:
        key = None
        took = False
        for line in open(path, errors="replace"):
            if key is None:
                m = VESSEL.search(line)
                if m:
                    key = holdprior_key(m.group(1), int(m.group(2)))
                continue
            if "GLIDE    alt" not in line:
                continue
            a, q, mach = AOA.search(line), Q.search(line), MACH.search(line)
            if not (a and q and mach):
                continue
            cmd, ach = float(a.group(1)), float(a.group(2))
            q, mach = float(q.group(1)), float(mach.group(1))
            if q < qmin or mach < args.mach or cmd - ach <= sat:
                continue
            index = int(math.floor(per * math.log10(max(1.0, q))))
            band = (sum(1 for e in cfg.HOLDABLE_MACH_EDGES if mach >= e)
                    if args.by_mach else -1)
            crafts[key][(band, index)].append(ach)
            took = True
        if took:
            used[key] += 1
    for key, cells in sorted(crafts.items()):
        bins = []
        edges = (0.0,) + tuple(cfg.HOLDABLE_MACH_EDGES) + (99.0,)
        for band, index in sorted(cells):
            values = sorted(cells[(band, index)])
            if len(values) < args.min_samples:
                continue
            lo, hi = 10 ** (index / float(per)), 10 ** ((index + 1) / float(per))
            cell = [round(lo, 1), round(hi, 1),
                    round(values[len(values) // 2], 2), len(values)]
            if band >= 0:
                cell += [max(args.mach, edges[band]), edges[band + 1]]
            bins.append(cell)
        print("%s  (%d logs)" % (key, used[key]))
        for cell in bins:
            print("  q %6.0f-%6.0f  held %5.1f  n=%d%s" % (
                tuple(cell[:4]) + (("  M %g-%g" % tuple(cell[4:6]))
                                   if len(cell) > 4 else "",)))
        if args.dry_run or not bins:
            continue
        out = os.path.join(ROOT, "logs", "holdprior", holdprior_slug(key) + ".json")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w") as fh:
            json.dump({"key": key, "mach_floor": args.mach, "logs": used[key],
                       "bins": bins}, fh, indent=1)
        print("  -> %s" % out)


if __name__ == "__main__":
    main()
