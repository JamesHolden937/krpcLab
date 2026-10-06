#!/usr/bin/env python3
"""The cone's handover, one line per flight, and its spread.

What the approach is handed: the exit kind, the height above what the
approach needs at the rollout (``surplus``), the distance to the gate, the
laps flown, and where the flight stopped.  For screening cone changes,
where a landing number mixes in everything after the cone.

    ./spaceplane/tools/conexit.py logs/LOG65{27..34}

Flights are grouped into arms by their own ``config:`` line (the fields
that differ from the defaults, less the per-instance plumbing), so a sim
screen maps logs to arms exactly rather than by LOG number.
"""
import re
import statistics as st
import sys

EXIT = re.compile(r"HAC -> APPROACH (\w[\w ]*?) turn=(-?\d+) h=(-?\d+) "
                  r"\(needed (-?\d+)\) gate=(-?\d+) \(circle (-?\d+)\) "
                  r"laps=(\d+)")
ENTRY = re.compile(r"GLIDE -> HAC .*?d=(\d+) h=(\d+)")
DOWN = re.compile(r"DOWN: .*?\(along ([+-]\d+), across ([+-]\d+)\)")
CONFIG = re.compile(r"config: (.*?) \[defaults (\w+)\]")
PLUMBING = ("SAVE_NAME", "LOOP_PACING_GAME_TIME", "TIMESCALE_GOVERNOR",
            "LOG_INTERVAL_UT")


def arm_of(text):
    """The arm a log flew: its non-plumbing config differences, sorted."""
    m = CONFIG.search(text)
    if not m:
        return "?"
    keep = sorted(f.strip() for f in m.group(1).split(",")
                  if f.strip() and f.split("=")[0].strip() not in PLUMBING)
    save = re.search(r"SAVE_NAME=([^,]+)", m.group(1))
    return "%s [%s] %s" % (save.group(1).strip() if save else "?",
                           m.group(2), " ".join(keep) or "(default)")


def spread(label, surplus):
    print("%s n=%d surplus median %+d, |surplus|<500: %d, mean %+d sd %d"
          % (label, len(surplus), st.median(surplus),
             sum(1 for s in surplus if abs(s) < 500),
             st.mean(surplus),
             st.pstdev(surplus) if len(surplus) > 1 else 0))


def main(paths):
    surplus = []
    arms = {}
    for path in paths:
        text = open(path, errors="replace").read()
        arm = arms.setdefault(arm_of(text), [])
        e, x, d = ENTRY.search(text), EXIT.search(text), DOWN.search(text)
        if not x:
            print("%-14s no cone exit" % path.split("/")[-1])
            continue
        kind, turn, h, need, gate, circle, laps = x.groups()
        s = int(h) - int(need)
        surplus.append(s)
        arm.append(s)
        print("%-14s in h=%5s  %-13s surplus %+6d gate %5s R %5s laps %s  "
              "stop %s" % (path.split("/")[-1], e.group(2) if e else "-",
                           kind, s, gate, circle, laps,
                           "%s/%s" % d.groups() if d else "-"))
    if len(arms) > 1:
        for name in sorted(arms):
            if arms[name]:
                spread("  " + name + "\n   ", arms[name])
    if surplus:
        spread("all", surplus)

if __name__ == "__main__":
    main(sys.argv[1:])
