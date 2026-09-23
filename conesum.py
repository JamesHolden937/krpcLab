#!/usr/bin/env python3
"""What the heading alignment cone actually costs, read back off the logs.

TEMPORARY TEST HARNESS -- not part of the flight software.  ``glidesum.py``
is the entry's equivalent and reads the same telemetry lines.

``HAC_LD`` is the one number the cone's whole energy model divides by: the
path it plans is ``(height - GATE_ALT_M) * HAC_LD``, so a cone planned
against the wrong value arrives over the gate with height it has nowhere to
put -- and the approach turns every metre of that into two metres of runway
overshoot.  It is not a polar measurement.  It is the *planned ideal path per
metre of height*, which is the achieved glide ratio divided by how much
further than its own plan the vehicle flies while chasing a circle that is
re-solved every tick.  Both halves are in the logs, so nothing here is
transcribed:

    LD_flown  = sum(v dt) / height lost, over the HAC phase
    tracking  = sum(v dt) / the path the cone planned when it first planned one
    HAC_LD    = LD_flown / tracking

Failure 13's rule, applied one phase later: a measurement whose only consumer
is a hand-copied constant cannot be contradicted.  Run this after anything
that changes how the cone is flown -- the attitude controller, the bank
limit, the speed the turn is held at -- because every one of those changes
both halves.

    ./conesum.py logs/LOG13*
    ./conesum.py --settled logs/LOG13*    # only flights whose plan was stable
"""
import argparse
import os
import re
import sys

LINE = re.compile(r'^\[\s*([\d.]+)\]\s+HAC\s+alt=\s*(-?\d+)\s+h=\s*(-?[\d.]+)'
                  r'\s+v=\s*([\d.]+)')
PLAN = re.compile(r'path=\s*(\d+)\s+need=\s*(\d+)')
EXIT = re.compile(r'HAC -> APPROACH (\w[\w ]*?) turn=(-?\d+) h=(-?\d+) '
                  r'\(needed (-?\d+)\) gate=(-?\d+) \(circle (-?\d+)\) '
                  r'laps=(\d+)')
DOWN = re.compile(r'DOWN: (.*?) -- (\d+) m from the runway midpoint '
                  r'\(along ([+-]\d+), across ([+-]\d+)\)')


def read(path):
    rows, plans, exit_line, down = [], [], None, None
    for line in open(path, errors="replace"):
        m = LINE.match(line)
        if m:
            rows.append(tuple(float(x) for x in m.groups()))
            p = PLAN.search(line)
            plans.append(float(p.group(1)) if p else None)
            continue
        m = EXIT.search(line)
        if m:
            exit_line = m.groups()
            continue
        m = DOWN.search(line)
        if m:
            down = m.groups()
    return rows, plans, exit_line, down


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--settled", action="store_true",
                    help="only flights whose first plan was within 30%% of "
                         "the path flown -- the ones whose tracking ratio "
                         "means anything")
    args = ap.parse_args(argv)

    implied = []
    flown_ld = []
    for path in sorted(args.logs,
                       key=lambda p: int(re.search(r'(\d+)$', p).group(1))):
        rows, plans, exit_line, down = read(path)
        if len(rows) < 6:
            continue
        first = next((k for k, p in enumerate(plans) if p), None)
        if first is None:
            continue
        flown = sum(0.5 * (a[3] + b[3]) * (b[0] - a[0])
                    for a, b in zip(rows[first:], rows[first + 1:]))
        used = rows[first][2] - rows[-1][2]
        if used < 500.0 or flown < 1000.0:
            continue
        planned = plans[first]
        tracking = flown / max(1.0, planned)
        if args.settled and not 0.7 <= tracking <= 1.3:
            continue
        ld = flown / used
        implied.append(ld / tracking)
        flown_ld.append(ld)
        bits = ["%-12s" % os.path.basename(path),
                "h %6.0f->%6.0f" % (rows[first][2], rows[-1][2]),
                "flown=%6.0f plan=%6.0f" % (flown, planned),
                "LD=%.2f track=%.2f -> HAC_LD=%.2f" % (ld, tracking,
                                                       ld / tracking)]
        if exit_line:
            why, turn, h, need, gate, circle, laps = exit_line
            bits.append("exit %s surplus=%+6.0f gate=%5s R=%5s laps=%s"
                        % (why.strip(), float(h) - float(need), gate, circle,
                           laps))
        if down:
            bits.append("down along=%s across=%s" % (down[2], down[3]))
        print("  ".join(bits))

    if implied:
        implied.sort()
        n = len(implied)
        mean = sum(implied) / n
        sd = (sum((x - mean) ** 2 for x in implied) / max(1, n - 1)) ** 0.5
        print("\n%d flights: HAC_LD mean %.2f sd %.2f median %.2f "
              "[%.2f .. %.2f]   (glide ratio flown: mean %.2f)"
              % (n, mean, sd, implied[n // 2], implied[0], implied[-1],
                 sum(flown_ld) / n))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
