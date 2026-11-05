#!/usr/bin/env python3
"""One line per prop-brake (or throttle-cut) flight: the speed Brakes
went on (or the props feathered) at, the time to
stand still, the worst sideslip (|va x|), the worst nose-up (va z, the air
from below in the vessel's axes), the height gained over the runway, the
tightest tip bound, the hill climb's back-offs, and whether it broke up.

    ./propPitch/tools/brakesum.py logs/LOG15240 logs/LOG15241 ...
"""
import re
import sys


def summary(path):
    on = still = None
    v_on = None
    slip = nose = 0.0
    alt0 = alt_max = None
    tips = 0
    tightest = None
    fatal = False
    config = ""
    for line in open(path):
        m = re.match(r"\[\s*([\d.]+)\]", line)
        ut = float(m.group(1)) if m else None
        if "config:" in line:
            config = line.split("config:")[1].split("defaults_fingerprint")[0]
        if "FATAL" in line:
            fatal = True
        if on is None and ("prop brake on" in line or "mode=feather" in line):
            on = ut                     # Brakes, or a throttle cut
        if on is not None and still is None and (
                "props stopped" in line
                or re.search(r"spd=\s*0\.[0-4] ", line)):
            still = ut
        if "tip: " in line:
            tips += 1
        t = re.search(r"spd=\s*([\d.]+) alt=\s*(-?[\d.]+).*va=\s*([-+\d.]+),"
                      r"\s*([-+\d.]+),\s*([-+\d.]+).*tip=(\S+)", line)
        if not t:
            continue
        spd, alt = float(t.group(1)), float(t.group(2))
        if on is None:
            alt0 = alt
            v_on = spd
            continue
        if still is None:
            slip = max(slip, abs(float(t.group(3))))
            nose = max(nose, float(t.group(5)))
            alt_max = alt if alt_max is None else max(alt_max, alt)
            if t.group(6) != "-":
                kn = float(t.group(6).rstrip("kN"))
                tightest = kn if tightest is None else max(tightest, kn)
    if on is None:
        return "%s: no brake or cut" % path
    return ("%s: brake at %5.1f m/s, still %s, slip %4.1f, nose-up %4.1f m/s, "
            "climbed %4.1f m, tightest %s, %d back-offs%s |%s"
            % (path, v_on or 0, "%.1f s" % (still - on) if still else "never",
               slip, nose, (alt_max or 0) - (alt0 or 0),
               "%+.0f kN" % tightest if tightest is not None else "-",
               tips, " BROKE" if fatal else "", config.strip()))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(summary(p))
