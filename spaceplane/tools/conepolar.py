#!/usr/bin/env python3
"""Write ``HAC_LD_FLOWN_POLAR``'s file: the subsonic polar each craft's cone
actually flew -- measured ClA and CdA per bin of *achieved* alpha.

    ./spaceplane/tools/conepolar.py logs/LOG8[5-6]??          # write
    ./spaceplane/tools/conepolar.py --dry-run logs/LOG86??    # print only

The cone's ladder priced its future on the swept table, and the table is
re-probed in flight with the surfaces at their *present* deflection: at cone
entry the subsonic rows read L/D 0.85-1.72 at 2.5 km where the cone then
flies 3.3-3.7 (rot-ladder-1008).  What the plan needs is the airframe
*trimmed at each alpha*, which only flying it measures.  A tick counts when
it is HAC, below ``--mach``, above ``--min-q``, and the force read is a wing
making lift; a bin is written with ``--min-samples`` of them, its values the
medians.  Like the alpha prior it describes the vehicle as flown (failure
23): re-run after anything that changes how the cone is trimmed, from logs
flown since.

Writes ``logs/conepolar/<craft>.json``; nothing tracked (it is regenerated
from logs).
"""
import argparse
import collections
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from spaceplane.trajectory import holdprior_key, holdprior_slug  # noqa: E402

VESSEL = re.compile(r"vessel: (.*?)\s+(\d+) parts")
AOA = re.compile(r"aoa=\s*[-0-9.]+/\s*([-0-9.]+)")
ACT = re.compile(r"act=\s*([0-9.]+)/\s*([0-9.]+)")
Q = re.compile(r"\bq=\s*([0-9.]+)")
MACH = re.compile(r"\bM=\s*([0-9.]+)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--mach", type=float, default=0.9)
    ap.add_argument("--min-q", type=float, default=300.0)
    ap.add_argument("--bin", type=float, default=2.0)
    ap.add_argument("--min-samples", type=int, default=50)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
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
            if "] HAC " not in line:
                continue
            a, act, q, mach = (AOA.search(line), ACT.search(line),
                               Q.search(line), MACH.search(line))
            if not (a and act and q and mach):
                continue
            if float(mach.group(1)) >= args.mach or float(q.group(1)) < args.min_q:
                continue
            cla, cda = float(act.group(1)), float(act.group(2))
            if cda < 0.5 or cla < 0.1:
                continue
            index = int(float(a.group(1)) // args.bin)
            crafts[key][index].append((cla, cda))
            took = True
        if took:
            used[key] += 1
    for key, cells in sorted(crafts.items()):
        bins = []
        for index in sorted(cells):
            values = cells[index]
            if len(values) < args.min_samples:
                continue
            cla = sorted(v[0] for v in values)[len(values) // 2]
            cda = sorted(v[1] for v in values)[len(values) // 2]
            bins.append([round((index + 0.5) * args.bin, 2), round(cla, 2),
                         round(cda, 2), len(values)])
        print("%s  (%d logs)" % (key, used[key]))
        for alpha, cla, cda, n in bins:
            print("  alpha %5.1f  ClA %6.1f  CdA %6.1f  L/D %4.2f  n=%d"
                  % (alpha, cla, cda, cla / cda, n))
        if args.dry_run or not bins:
            continue
        out = os.path.join(ROOT, "logs", "conepolar",
                           holdprior_slug(key) + ".json")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w") as fh:
            json.dump({"key": key, "mach_max": args.mach, "logs": used[key],
                       "bins": bins}, fh, indent=1)
        print("  -> %s" % out)


if __name__ == "__main__":
    main()
