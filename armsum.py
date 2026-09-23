#!/usr/bin/env python3
"""Group flights by the configuration that flew them, and put error bars on.

TEMPORARY TEST HARNESS -- not part of the flight software.  ``glidesum.py``
reports one line per flight; this reports one line per *arm*, which is the
unit a comparison is actually made in.

It exists because of spaceplane failure 27.  Three flights of one arm against
three of another looked like a seven-fold improvement, was adopted, written
into ``config.py`` with its table, and dissolved at n=22: the along-track
scatter is **sd ~10 km**, so an arm of three has a standard error of six and
two such arms cannot resolve anything smaller than fifteen.  Reading a mean
without its standard error is what made that mistake invisible.

    ./armsum.py logs/LOG10*                  # every arm it can find
    ./armsum.py --by GLIDE_RESERVE_ON logs/LOG1*
    ./armsum.py --against '(defaults)' logs/LOG1*

The grouping key is the log's own ``config:`` line, including the
``[defaults ...]`` fingerprint -- two logs with different fingerprints are
not the same experiment however identical their overrides look (failure 28).

What is reported is the **signed along-track miss at the gate handover**,
because that is the landing miss on this vehicle to within a kilometre or
two, and because a great-circle distance cannot tell an overshoot from an
undershoot (CLAUDE.md).
"""
import argparse
import glob
import math
import os
import re
import sys

CONFIG = re.compile(r"config: (.*?)\s*(?:\[defaults ([0-9a-f]+)\])?\s*$")
# **Both spellings of the same handover.**  The phase machine gained the
# heading-alignment cone and the transition became ``GLIDE -> HAC``; this
# still matched only the old ``GLIDE -> APPROACH`` and so reported "no
# flights reached the gate" for every flight flown since -- silently, which
# is the failure mode a summary tool has.  The quantity is the same one
# either way: the along-track miss the entry delivers and the cone inherits.
HANDOVER = re.compile(r"GLIDE -> (?:APPROACH|HAC).*?long=([-+0-9]+)"
                      r".*?cross=([-+0-9]+)")


def read(path):
    """(config, defaults, long, cross) for one log, or None if it never flew."""
    config = defaults = None
    miss = None
    for line in open(path, errors="replace"):
        if config is None and "config:" in line:
            m = CONFIG.search(line.strip())
            if m:
                # **The governor's path is per-instance, so leaving it in the
                # key makes every flight its own arm** -- twelve arms of one,
                # each with a standard error of zero, which is the exact
                # failure this tool was written to prevent (failure 27: a
                # mean without its error bar).  The save stays in the key:
                # the same configuration on two entry states is two
                # experiments.
                drop = ("TIMESCALE_GOVERNOR=", "TIMESCALE_GOVERNOR_MAX=",
                        "LOOP_PACING_GAME_TIME=")
                bits = [b.strip() for b in m.group(1).split(",")
                        if b.strip() and not b.strip().startswith(drop)]
                config = ", ".join(bits) or "(defaults)"
                defaults = m.group(2) or "?"
        m = HANDOVER.search(line)
        if m:
            miss = (int(m.group(1)), int(m.group(2)))
    if config is None or miss is None:
        return None
    return config, defaults, miss[0], miss[1]


def stats(values):
    n = len(values)
    mean = sum(values) / n
    if n < 2:
        return mean, 0.0, 0.0
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    sd = math.sqrt(var)
    return mean, sd, sd / math.sqrt(n)


def median(values):
    s = sorted(values)
    half = len(s) // 2
    return s[half] if len(s) % 2 else 0.5 * (s[half - 1] + s[half])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("logs", nargs="+")
    p.add_argument("--by", default=None,
                   help="group on one field's value rather than the whole line")
    p.add_argument("--against", default=None,
                   help="the arm to difference the others against")
    args = p.parse_args(argv)

    paths = []
    for pattern in args.logs:
        paths.extend(sorted(glob.glob(pattern)) if "*" in pattern else [pattern])

    arms = {}
    for path in paths:
        got = read(path)
        if got is None:
            continue
        config, defaults, along, cross = got
        if args.by:
            m = re.search(r"\b%s=(\S+?)[,\]]?$|\b%s=([^,]+)"
                          % (args.by, args.by), config)
            key = (m.group(1) or m.group(2)).strip() if m else "(default)"
            key = "%s=%s" % (args.by, key)
        else:
            key = "%s [%s]" % (config, defaults)
        arms.setdefault(key, []).append((os.path.basename(path), along, cross))

    if not arms:
        print("no flights reached the gate in those logs")
        return 1

    # **Say so when the logs cannot answer the question being asked of
    # them.**  A log written before ``defaults_fingerprint`` existed carries
    # no baseline, so an arm keyed on "the field is not overridden" may be
    # two different experiments stacked on top of each other -- which is
    # exactly how failure 28 happened.  Silence here would reproduce it.
    blind = sum(1 for path in paths
                for got in [read(path)] if got and got[1] == "?")
    if blind:
        print("-- %d of %d logs predate the defaults fingerprint: an arm of "
              "theirs\n   is only as trustworthy as the defaults being "
              "unchanged across them.\n" % (blind, len(paths)))

    width = max(len(k) for k in arms)
    print("%-*s   n    along mean     sd     se   median   |cross| med"
          % (width, "arm"))
    summary = {}
    for key in sorted(arms, key=lambda k: -len(arms[k])):
        rows = arms[key]
        along = [r[1] / 1000.0 for r in rows]
        cross = [abs(r[2]) / 1000.0 for r in rows]
        mean, sd, se = stats(along)
        summary[key] = (mean, se, len(along))
        print("%-*s %3d   %+8.1f %6.1f %6.1f %+8.1f %10.2f"
              % (width, key, len(along), mean, sd, se, median(along),
                 median(cross)))

    if args.against:
        base = None
        for key in summary:
            if args.against in key:
                base = key
                break
        if base is None:
            print("\n-- no arm matching %r to difference against" % args.against)
            return 0
        bm, bse, bn = summary[base]
        print("\ndifference against %r (n=%d):" % (base, bn))
        for key, (mean, se, n) in summary.items():
            if key == base:
                continue
            delta = mean - bm
            sigma = math.sqrt(se * se + bse * bse)
            # Two standard errors is the least that should move a default.
            verdict = ("inside the scatter" if abs(delta) < 2.0 * sigma
                       else "OUTSIDE the scatter")
            print("  %-*s %+7.1f km  +- %.1f  (%s)"
                  % (width, key, delta, sigma, verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
