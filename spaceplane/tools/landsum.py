#!/usr/bin/env python3
"""The whole landing chain on one line per flight: arrival, handover, wheels.

TEMPORARY TEST HARNESS -- not part of the flight software.  ``glidesum.py``
reports the entry, ``conesum.py`` the cone; this reports what they hand to
each other and what comes out of the end, because the failures that survived
longest here were all *between* phases and no single-phase summary could see
them.

Four columns and the relations between them are the whole diagnosis:

    arrival   the predicted along-track miss at GLIDE -> HAC, which is what
              the entry delivered and the cone has to absorb
    surplus   height at the rollout minus what the *approach* needs from
              there (``approach_needed``), so zero is a handover the next
              phase can fly and positive is runway given away at about two
              metres per metre
    ratio     ground from the rollout to the first wheel contact over the
              height there -- the number ``APPROACH_BEST_LD`` and
              ``GATE_ALT_M`` are both sized against, and which has been
              re-measured wrongly twice because it was taken over APPROACH
              alone and stopped at the flare (failure 23, spaceplane 39)
    stopped   along and across from the runway midpoint.  The runway is
              +/-1200 m along and +/-35 m across, and those two bounds are
              the only pass/fail in this project

    ./spaceplane/tools/landsum.py logs/LOG16*
    ./spaceplane/tools/landsum.py --arrivals-within 5000 logs/LOG16*   # only the ones the
                                                      # entry did its job on

**The last line is the one to read.** ``stopped - arrival`` is the bias the
landing chain adds on top of whatever the entry did, and it is the quantity
that decides whether a good arrival becomes a landing: measured on 2026-09-17
it is **+1.75 km with sd 0.8**, against a runway that accepts +/-1.2 km. The
entry and the landing have to be judged separately or each hides the other.
"""
import argparse
import glob
import os
import re
import statistics
import sys

ARR = re.compile(r'GLIDE -> HAC over the field long=([+-]\d+) cross=([+-]\d+)')
EXIT = re.compile(r'HAC -> APPROACH (\w[\w ]*?) turn=(-?\d+) h=(-?\d+) '
                  r'\(needed (-?\d+)\) gate=(-?\d+)')
TOUCH = re.compile(r'FLARE -> ROLLOUT sink=(-?[\d.]+) speed=([\d.]+)')
ROLL = re.compile(r'^\[\s*([\d.]+)\]\s+ROLLOUT\s+alt=.*?rwy=\s*(\d+)', re.M)
DOWN = re.compile(r'DOWN: (.*?) -- (\d+) m from the runway midpoint '
                  r'\(along ([+-]\d+), across ([+-]\d+)\).*?'
                  r'(\d+) of (\d+) parts')
CFG = re.compile(r'config: (.*?) \[defaults (\w+)\]')


def _flown(text, field, fallback):
    """The value *this flight* flew, not the one the tool was told to assume.

    ``ratio`` is a geometry, and it is assembled from ``GATE_DIST_M`` and
    ``TOUCHDOWN_AIM_M``.  Taking those from a command-line default read
    eighteen flights of ``TOUCHDOWN_AIM_M=2400`` as if they had flown 200 and
    reported the rollout-to-wheels ratio 1.1 low -- under ``APPROACH_BEST_LD``
    where the truth is over it, which inverts the one comparison the column
    exists to make.  Every log states its own configuration on line two;
    read it from there.  CLAUDE.md failure 13.
    """
    found = re.search(r'\b%s=(-?[\d.]+)' % field, text)
    return float(found.group(1)) if found else fallback


def read(path, gate_dist, aim):
    text = open(path, errors="replace").read()
    gate_dist = _flown(text, "GATE_DIST_M", gate_dist)
    aim = _flown(text, "TOUCHDOWN_AIM_M", aim)
    if "# closed" not in text:
        return None
    arrival = ARR.search(text)
    if arrival is None:
        return None
    exit_ = EXIT.search(text)
    down = DOWN.search(text)
    touch = TOUCH.search(text)
    rolls = ROLL.findall(text)
    cfg = CFG.search(text)
    ratio = None
    if exit_ and rolls:
        height = float(exit_.group(3))
        ground = float(exit_.group(5)) + gate_dist + aim + float(rolls[0][1])
        if height > 1.0:
            ratio = ground / height
    return dict(
        log=os.path.basename(path),
        along=int(arrival.group(1)), cross=int(arrival.group(2)),
        why=exit_.group(1).strip() if exit_ else "-",
        surplus=(float(exit_.group(3)) - float(exit_.group(4)))
        if exit_ else None,
        ratio=ratio,
        speed=float(touch.group(2)) if touch else None,
        stopped_along=int(down.group(3)) if down else None,
        stopped_across=int(down.group(4)) if down else None,
        parts=int(down.group(5)) if down else None,
        total=int(down.group(6)) if down else None,
        config=cfg.group(1) if cfg else "?",
        fingerprint=cfg.group(2) if cfg else "?")


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--arrivals-within", type=float, default=None,
                    metavar="M",
                    help="only flights the entry delivered inside this, so "
                         "the landing chain is judged on its own")
    ap.add_argument("--gate-dist", type=float, default=4000.0,
                    help="fallback only; a log that names GATE_DIST_M wins")
    ap.add_argument("--aim", type=float, default=200.0,
                    help="fallback only; a log that names TOUCHDOWN_AIM_M wins")
    args = ap.parse_args(argv)

    paths = []
    for item in args.logs:
        paths.extend(sorted(glob.glob(item)) if any(c in item for c in "*?[")
                     else [item])
    rows = []
    for path in sorted(paths,
                       key=lambda p: int(re.search(r'(\d+)$', p).group(1))):
        row = read(path, args.gate_dist, args.aim)
        if row is None:
            continue
        if (args.arrivals_within is not None
                and abs(row["along"]) > args.arrivals_within):
            continue
        rows.append(row)

    print("%-11s %8s %7s %8s %6s %6s %8s %7s %s"
          % ("log", "arrival", "cross", "surplus", "ratio", "td m/s",
             "along", "across", "parts"))
    for r in rows:
        print("%-11s %+8d %+7d %8s %6s %6s %8s %7s %s"
              % (r["log"], r["along"], r["cross"],
                 "%+.0f" % r["surplus"] if r["surplus"] is not None else "-",
                 "%.2f" % r["ratio"] if r["ratio"] else "-",
                 "%.0f" % r["speed"] if r["speed"] else "-",
                 "%+d" % r["stopped_along"]
                 if r["stopped_along"] is not None else "-",
                 "%+d" % r["stopped_across"]
                 if r["stopped_across"] is not None else "-",
                 "%d/%d" % (r["parts"], r["total"]) if r["parts"] else "lost"))

    if not rows:
        return 0
    arrivals = [r["along"] for r in rows]
    landed = [r for r in rows if r["stopped_along"] is not None]
    print("\n%d flights, %d configurations"
          % (len(rows), len({r["fingerprint"] for r in rows})))
    print("  arrival        mean %+8.0f  sd %7.0f   inside 5 km: %d"
          % (statistics.mean(arrivals),
             statistics.stdev(arrivals) if len(arrivals) > 1 else 0.0,
             sum(1 for x in arrivals if abs(x) <= 5000)))
    ratios = [r["ratio"] for r in rows if r["ratio"]]
    if ratios:
        print("  rollout->wheels ratio mean %.2f sd %.2f  (sizes "
              "APPROACH_BEST_LD and GATE_ALT_M)"
              % (statistics.mean(ratios),
                 statistics.stdev(ratios) if len(ratios) > 1 else 0.0))
    if landed:
        bias = [r["stopped_along"] - r["along"] for r in landed]
        across = [abs(r["stopped_across"]) for r in landed]
        print("  stopped along  mean %+8.0f  sd %7.0f   on the runway "
              "(|along| <= 1200): %d of %d"
              % (statistics.mean(r["stopped_along"] for r in landed),
                 statistics.stdev([r["stopped_along"] for r in landed])
                 if len(landed) > 1 else 0.0,
                 sum(1 for r in landed if abs(r["stopped_along"]) <= 1200),
                 len(landed)))
        print("  stopped across |mean| %6.0f  max %5d          on the strip "
              "(|across| <= 35): %d of %d"
              % (statistics.mean(across), max(across),
                 sum(1 for x in across if x <= 35), len(landed)))
        print("  **landing bias (stopped - arrival) mean %+.0f m  sd %.0f** "
              "-- what the chain adds on its own"
              % (statistics.mean(bias),
                 statistics.stdev(bias) if len(bias) > 1 else 0.0))
    intact = [r for r in rows if r["parts"] and r["parts"] >= 18]
    print("  kept 18+ parts: %d of %d" % (len(intact), len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
