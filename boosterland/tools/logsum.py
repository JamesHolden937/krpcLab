#!/usr/bin/env python3
"""One-line-per-flight summary of a boosterland log, plus the miss trace.

TEMPORARY TEST HARNESS -- not part of the flight software.

The logs are dense and meant to be read whole, but comparing twenty flights
needs the four numbers that separate them: where boostback stopped, what the
coast did to the prediction, and where the booster ended up.

    ./boosterland/tools/logsum.py logs/LOG1 logs/LOG2
    ./boosterland/tools/logsum.py --trace logs/LOG1
"""
import argparse
import re
import sys

TELEM = re.compile(r"^\[ *([\d.]+)\] (\w+) +alt= *(-?[\d.]+).*?"
                   r"pad= *([\d.]+) +miss= *([\d.]+|nan).*?cda= *([\d.]+) +M=([\d.]+)")
SIGNED = re.compile(r"^\[ *([\d.]+)\] (\w+) +alt= *(-?[\d.]+).*?hs= *([\d.]+).*?"
                    r"long= *([-+][\d.]+) +cross= *([-+][\d.]+)")
PHASE = re.compile(r"^\[ *([\d.]+)\] phase (\w+) -> (\w+) ?(.*)$")
DONE = re.compile(r"shutdown: (\S+) ([\d.]+) m from pad, alt=(-?[\d.]+) spd=([\d.]+)")
RESWEEP = re.compile(r"^\[ *([\d.]+)\] drag curve re-swept")
CONFIG = re.compile(r"^\[ *[\d.]+\] config: (.*)$")


def segments(path):
    """Where the miss stands at each hand-over, signed.

    A flight is four segments -- the burn, the coast, the corrections, the
    landing burn -- and a single distance at the end cannot say which of them
    spent the error.  Read across one row and it is usually obvious: the
    boostback exit is a deliberate *lead* (`long` about -175 m on this
    booster, negative meaning short), the coast is what spends it, and the
    landing burn moves the answer by ten metres or so.  When `burn@` and the
    final distance agree, nothing after the landing-burn trigger mattered and
    the fix belongs earlier in the flight.
    """
    marks, seen, done, hs = {}, set(), None, []
    for line in open(path, errors="replace"):
        m = SIGNED.match(line)
        if m:
            phase = m.group(2)
            if phase not in seen:
                seen.add(phase)
                marks[phase] = (float(m.group(5)), float(m.group(6)))
            if phase == "LANDING_BURN":
                hs.append(float(m.group(4)))
        m = DONE.search(line)
        if m:
            done = (m.group(1), float(m.group(2)))
    return marks, done, hs[-3:]


def summarise(path):
    rows, phases, done, resweep, config = [], [], None, None, ""
    for line in open(path, errors="replace"):
        m = CONFIG.match(line)
        if m:
            config = m.group(1)
        m = TELEM.match(line)
        if m:
            rows.append((float(m.group(1)), m.group(2), float(m.group(3)),
                         float(m.group(4)),
                         float("nan") if m.group(5) == "nan" else float(m.group(5)),
                         float(m.group(6)), float(m.group(7))))
        m = PHASE.match(line)
        if m:
            phases.append((float(m.group(1)), m.group(3), m.group(4)))
        m = RESWEEP.match(line)
        if m and resweep is None:
            resweep = float(m.group(1))
        m = DONE.search(line)
        if m:
            done = (m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4)))
    return rows, phases, done, resweep, config


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logs", nargs="+")
    p.add_argument("--trace", action="store_true",
                   help="print the coast's predicted miss against altitude")
    p.add_argument("--segments", action="store_true",
                   help="one row per flight: the signed miss at each phase "
                        "hand-over, plus the last three hs= readings.  This "
                        "is the view that says which segment spent the error")
    args = p.parse_args(sys.argv[1:] if argv is None else argv)

    if args.segments:
        print("%-9s %-34s %16s %16s %8s  %s"
              % ("log", "config", "coast entry", "burn entry", "landed",
                 "hs= last three"))
        for path in args.logs:
            marks, done, hs = segments(path)
            if done is None:
                continue
            _, _, _, _, config = summarise(path)
            # A sweep sets the same AIM_BIAS on every flight, so it is noise
            # in this table; what distinguishes the columns is the rest.
            config = " ".join(f for f in config.split()
                              if not f.startswith("AIM_BIAS_"))
            co = marks.get("COAST", (float("nan"),) * 2)
            lb = marks.get("LANDING_BURN", (float("nan"),) * 2)
            print("%-9s %-34s %+8.0f%+8.0f %+8.0f%+8.0f %8.0f  %s"
                  % (path.split("/")[-1], (config or "?")[:34],
                     co[0], co[1], lb[0], lb[1], done[1],
                     " ".join("%.1f" % h for h in hs)))
        return 0

    for path in args.logs:
        rows, phases, done, resweep, config = summarise(path)
        # The *first* entry into COAST is the boostback exit; a later one is
        # a correction burn ending, which is a different event entirely.
        bb = next(((ut, note) for ut, n, note in phases if n == "COAST"),
                  (float("nan"), ""))
        corr = sum(1 for _, n, _ in phases if n == "CORRECTION")
        coast = [r for r in rows if r[1] in ("COAST", "CORRECTION")]
        peak = max((r[4] for r in coast if r[4] == r[4]), default=float("nan"))
        print("%-14s bb_exit ut=%.2f %-14s corrections=%d peak_miss=%.0f "
              "resweep=%s  ->  %s"
              % (path.split("/")[-1], bb[0], bb[1], corr, peak,
                 ("ut=%.2f" % resweep) if resweep else "no",
                 ("%s %.0f m at %.1f m/s" % (done[0].split(".")[-1], done[1], done[3]))
                 if done else "(no exit line)"))
        # Which --set flew it.  A sweep is twenty logs that differ only here.
        print("               config: %s" % (config or "(not recorded)"))
        if args.trace:
            for ut, phase, alt, pad, miss, cda, mach in rows:
                if phase in ("COAST", "CORRECTION", "LANDING_BURN"):
                    print("   %9.2f %-12s alt=%7.0f pad=%7.0f miss=%7.0f "
                          "cda=%5.1f M=%.2f" % (ut, phase, alt, pad, miss, cda, mach))
    return 0


if __name__ == "__main__":
    sys.exit(main())
