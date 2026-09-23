#!/usr/bin/env python3
"""One line per spaceplane flight, and the bank-reversal trace behind it.

TEMPORARY TEST HARNESS -- not part of the flight software.  ``logsum.py`` is
the booster's equivalent and reads a different telemetry line.

The spaceplane's open problem is **repeatability**, not aim (see
`docs/spaceplane-failures.md` 10c/10d), and the quantity that turned out to
separate two otherwise identical flights is not in any existing summary: how
many times the bank reversed, and where.  The sign of the bank is the only
cross-track authority in the flight, so every reversal is also a dozen
seconds spent rolling through wings-level -- where the lift is nearly all
vertical and the vehicle stops sinking.  A flight that reverses twenty times
has spent half its entry in that transient, and *which* transient it is in
when the air thickens is worth tens of kilometres at the ground.

    ./glidesum.py logs/LOG706 logs/LOG710
    ./glidesum.py --trace logs/LOG706
    ./glidesum.py --band 33000 26000 logs/LOG7*      # just the level-off

What it counts is a *sign change of the commanded bank* between consecutive
telemetry lines.  That is the reversal itself and not the solve's intent: in
the logs that motivated this tool the command through the level-off is not
the solve's answer at all, it is the rate limiter slewing at exactly
``BANK_RATE_DEG_S`` between the stops.
"""
import argparse
import os
import re
import sys

FIELD = re.compile(r"(\w+)=\s*([-+]?[\d.]+)")
STAMP = re.compile(r"^\[\s*([\d.]+)\]")


def rows(path, phase="GLIDE"):
    out = []
    for line in open(path, errors="replace"):
        if " %s " % phase not in line:
            continue
        stamp = STAMP.match(line)
        if not stamp:
            continue
        d = dict(FIELD.findall(line))
        if "bank" not in d or "alt" not in d:
            continue
        d["ut"] = stamp.group(1)
        out.append({k: float(v) for k, v in d.items()})
    return out


def reversals(samples, low=None, high=None):
    """Sign changes of the commanded bank, with the row each landed on.

    A pass through exactly zero is not special-cased: the command is
    rate-limited, so it steps through zero on some tick and that tick is as
    good a marker of the reversal as any.
    """
    out = []
    for a, b in zip(samples, samples[1:]):
        if a["bank"] * b["bank"] >= 0.0:
            continue
        if low is not None and not (low <= b["alt"] <= high):
            continue
        out.append(b)
    return out


def landed(path):
    """The shutdown line's own account, rather than a reconstruction."""
    last = None
    for line in open(path, errors="replace"):
        if "shutdown:" in line:
            last = line.strip()
    return last


def config_line(path):
    for line in open(path, errors="replace"):
        m = re.search(r"config: (.*)$", line)
        if m:
            return m.group(1).strip()
    return "?"


def summarise(path, low, high):
    samples = rows(path)
    if not samples:
        return "%-16s  no GLIDE telemetry" % os.path.basename(path)
    inside = [s for s in samples if low <= s["alt"] <= high]
    revs = reversals(samples)
    banded = reversals(samples, low, high)
    crosses = sorted(abs(s.get("cross", 0.0)) for s in inside) or [0.0]
    longs = [s.get("long", 0.0) for s in inside]
    span = samples[-1]["ut"] - samples[0]["ut"]
    # Seconds of the banded window spent inside a reversal, which is what the
    # count is a proxy for: the roll takes 2*|bank|/BANK_RATE_DEG_S and the
    # vehicle is not flying the commanded bank for any of it.
    return ("%-16s  %5.0f s glide  %3d reversals (%2d in band)  "
            "|cross| med %5.0f p90 %6.0f  |long| med %6.0f  %s"
            % (os.path.basename(path), span, len(revs), len(banded),
               crosses[len(crosses) // 2], crosses[int(len(crosses) * 0.9)],
               sorted(abs(x) for x in longs)[len(longs) // 2] if longs else 0.0,
               config_line(path)))


def trace(path, low, high):
    print("=== %s   %s" % (path, config_line(path)))
    print("    %8s %7s %6s %7s %7s %9s %9s"
          % ("ut", "alt", "v", "vs", "bank", "long", "cross"))
    previous = None
    for s in rows(path):
        mark = " "
        if previous is not None and previous * s["bank"] < 0.0:
            mark = "*"
        previous = s["bank"]
        if not (low <= s["alt"] <= high):
            continue
        print("  %s %8.1f %7.0f %6.0f %7.1f %7.1f %9.0f %9.0f"
              % (mark, s["ut"], s["alt"], s["v"], s.get("vs", 0.0),
                 s["bank"], s.get("long", 0.0), s.get("cross", 0.0)))
    end = landed(path)
    if end:
        print("    %s" % end)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("logs", nargs="+")
    p.add_argument("--trace", action="store_true",
                   help="every glide line in the band, reversals marked *")
    p.add_argument("--band", nargs=2, type=float, default=(0.0, 1.0e6),
                   metavar=("LOW", "HIGH"),
                   help="altitude window to count and trace in, metres")
    a = p.parse_args(argv)
    low, high = min(a.band), max(a.band)
    for path in a.logs:
        if a.trace:
            trace(path, low, high)
        else:
            print(summarise(path, low, high))
    return 0


if __name__ == "__main__":
    sys.exit(main())
