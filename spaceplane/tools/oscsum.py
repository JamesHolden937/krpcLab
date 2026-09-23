#!/usr/bin/env python3
"""How hard is the vehicle wallowing, per log.

``glidesum.py`` counts bank reversals and ``landsum.py`` reads the arrival;
neither says whether the airframe was *pointed* on the way down.  This does:
the spread of the achieved angle of attack about its command, and the peak
to peak sideslip, per phase.

It exists because a second airframe flew the committed configuration into a
coupled pitch/yaw oscillation -- commanded 25 degrees of alpha, achieving 21
to 52, with sideslip swinging +-30 at Mach 7 -- and arrived 205 km short,
while every summary in the tree reported only the 205 km.  An attitude that
is not held is a drag number nobody planned, and it has to be readable.

    ./spaceplane/tools/oscsum.py logs/LOG287*
"""
import re
import statistics
import sys

LINE = re.compile(
    r'^\[\s*([\d.]+)\]\s+(\w+)\s+.*aoa=\s*([\d.-]+)/\s*([\d.-]+)'
    r'.*?slip=\s*([+-][\d.]+).*?q=\s*(\d+)')
ARRIVE = re.compile(r'GLIDE -> HAC .*?long=(-?\d+)')
CONFIG = re.compile(r'config: (.*?)\s*\[defaults')


def read(path):
    phases, arrival, config = {}, None, ""
    for line in open(path, errors="ignore"):
        m = CONFIG.search(line)
        if m:
            config = m.group(1)
        m = ARRIVE.search(line)
        if m:
            arrival = int(m.group(1))
        m = LINE.match(line)
        if m:
            _, phase, cmd, got, slip, q = m.groups()
            phases.setdefault(phase, []).append(
                (float(cmd), float(got), float(slip), float(q)))
    return config, arrival, phases


def main(paths):
    print("%-14s %-9s %5s %7s %7s %8s %9s"
          % ("log", "phase", "n", "aoa err", "aoa sd", "slip p-p", "arrival"))
    for path in paths:
        config, arrival, phases = read(path)
        if config:
            print("  %s" % config[:96])
        for phase in ("GLIDE", "HAC", "APPROACH"):
            rows = phases.get(phase)
            if not rows or len(rows) < 3:
                continue
            err = statistics.fmean(g - c for c, g, _, _ in rows)
            sd = statistics.pstdev([g - c for c, g, _, _ in rows])
            slips = [s for _, _, s, _ in rows]
            print("%-14s %-9s %5d %+7.1f %7.1f %8.1f %9s"
                  % (path.split("/")[-1], phase, len(rows), err, sd,
                     max(slips) - min(slips),
                     "%+d" % arrival if arrival is not None and
                     phase == "GLIDE" else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or ["logs/LOG2872"]))
