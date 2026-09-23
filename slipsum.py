#!/usr/bin/env python3
"""slipsum.py LOG... -- what sideslip can this airframe actually hold?

The yaw-axis twin of the alpha ceiling.  ``SLIP_PROBE_DEG`` commands a fixed
sideslip from COAST onward and every tick logs what was asked (``slipc=``),
what the vehicle achieved (``slip=``) and the dynamic pressure it happened at
(``q=``).  The plant is a saturation -- ``achieved = min(command, holdable(q))``
-- so binning the achieved angle against ``q`` *is* the ceiling curve, the
same way ``Holdable`` learns it for the angle of attack.

Read it as: at each dynamic pressure, how much of the commanded slip did the
reaction wheels actually deliver?  A row where achieved tracks commanded is a
slip the vehicle can hold and therefore drag it can make; a row where it
collapses is the wheels losing to the weathercock moment.

    ./slipsum.py logs/LOG28*
"""
import glob, re, sys
from collections import defaultdict

ROW = re.compile(r"\]\s+(\w+)\s+alt=.*?slip=\s*([+-][0-9.]+).*?slipc=\s*([+-][0-9.]+)"
                 r".*?\bq=\s*([0-9.]+)")
# The drag the slip actually bought, off the same line: ``act=ClA/CdA`` is
# measured from the achieved aerodynamic force, so it needs no model and no
# density assumption (see docs/krpc.md on ``Flight.aerodynamic_force``).
ACT = re.compile(r"\bact=\s*([0-9.]+)/\s*([0-9.]+)")
AOA = re.compile(r"\baoa=\s*([0-9.]+)/\s*([0-9.]+)")
MACH = re.compile(r"\bM=\s*([0-9.]+)")
# q bins, geometric: the ceiling falls with q and the interesting decade is 1e3-1e4
EDGES = [0, 500, 1000, 1500, 2000, 3000, 4000, 5000, 7000, 10000, 1e9]

def main(argv):
    files = sorted(set(sum((glob.glob(a) for a in argv), [])))
    per_cmd = defaultdict(lambda: defaultdict(list))
    phases = defaultdict(set)
    for path in files:
        for line in open(path, errors="ignore"):
            m = ROW.search(line)
            if not m:
                continue
            phase, achieved, commanded, q = (m.group(1), abs(float(m.group(2))),
                                             abs(float(m.group(3))), float(m.group(4)))
            if commanded < 0.01:
                continue
            b = next(i for i in range(len(EDGES) - 1) if EDGES[i] <= q < EDGES[i + 1])
            per_cmd[round(commanded)][b].append(achieved)
            phases[round(commanded)].add(phase)
    if not per_cmd:
        print("no probed ticks: no log carries slipc=")
        return 1
    drag_report(files)
    for cmd in sorted(per_cmd):
        print("commanded %g deg   (%s)" % (cmd, ", ".join(sorted(phases[cmd]))))
        print("   %-14s %6s %8s %8s %8s" % ("q (Pa)", "ticks", "held", "of cmd", "verdict"))
        for b, vals in sorted(per_cmd[cmd].items()):
            vals.sort()
            med = vals[len(vals) // 2]
            frac = med / cmd if cmd else 0.0
            verdict = ("holds" if frac > 0.85 else
                       "partial" if frac > 0.4 else "lost")
            print("   %-14s %6d %8.1f %7.0f%% %8s"
                  % ("%d-%d" % (EDGES[b], EDGES[b + 1]) if EDGES[b + 1] < 1e8
                     else "%d+" % EDGES[b], len(vals), med, 100 * frac, verdict))
        print()
    return 0



def drag_report(files):
    """What the slip cost in drag, against unprobed flights at the same alpha.

    **The point of the whole exercise.**  A held sideslip is only worth
    having if the fuselage's side area actually brakes: the swept table says
    ``CdA`` 29-33 m^2 broadside against 0.8 at zero, but the table is the one
    that was wrong by 1.8x subsonically (failure 13), so it is the flown
    ``act=`` column that decides.  Binned on the *achieved* slip and on the
    achieved angle of attack, because drag rises with both and a comparison
    that mixes them measures neither.
    """
    from collections import defaultdict
    bins = defaultdict(list)
    for path in files:
        for line in open(path, errors="ignore"):
            if " GLIDE " not in line and " HAC " not in line and \
               " APPROACH " not in line:
                continue
            a, act, mach = AOA.search(line), ACT.search(line), MACH.search(line)
            sl = re.search(r"slip=\s*([+-][0-9.]+)", line)
            if not (a and act and mach and sl):
                continue
            if float(mach.group(1)) > 1.0:
                continue                    # subsonic only: CdA is not one curve
            alpha = round(float(a.group(2)) / 4.0) * 4      # 4-degree bins
            slip = round(abs(float(sl.group(1))) / 5.0) * 5  # 5-degree bins
            bins[(alpha, slip)].append(float(act.group(2)))
    if not bins:
        return
    print("subsonic CdA against sideslip, from the achieved aerodynamic force")
    print("   %-8s %-8s %6s %8s" % ("aoa", "|slip|", "ticks", "CdA"))
    for (alpha, slip) in sorted(bins):
        vals = sorted(bins[(alpha, slip)])
        if len(vals) < 5:
            continue
        print("   %-8.0f %-8.0f %6d %8.2f"
              % (alpha, slip, len(vals), vals[len(vals) // 2]))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:] or ["logs/LOG*"]))
