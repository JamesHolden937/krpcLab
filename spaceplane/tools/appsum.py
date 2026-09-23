#!/usr/bin/env python3
"""What the approach flew, and whether it flew the descent it asked for.

TEMPORARY TEST HARNESS -- not part of the flight software.

`landsum.py` reports the landing geometry and `rollsum.py` the rollout; this
reports the stretch between them, because that is where the speed loop either
holds the profile or runs away from it, and the telemetry has been printing
the evidence all along in a column nothing read.

The APPROACH line carries ``sink=<achieved>/<wanted>``.  Those two numbers
being far apart is the whole diagnosis, and the third column is why:

    bank     mean |bank| over the approach, degrees.  The S-turn's limit is
             ``APPROACH_BANK_MAX_DEG`` (40).
    sink     mean achieved sink, and mean wanted, m/s.
    excess   achieved minus wanted.  A vehicle descending 30 m/s faster than
             its own profile asks is not flying the profile.
    flare    speed and height at ``APPROACH -> FLARE``.  The flare is
             documented to complete from 83-91 m/s; above about 94 the
             lateral capture runs out of time and the miss grows with speed.
    td       touchdown speed.

Why it matters: ``alpha_for_speed`` solves for a load in the vertical plane
and nothing divided it by ``cos(bank)``, so a vehicle at the 40 degree S-turn
limit was handed 77% of the load it computed -- in exactly the moments it had
decided it was high.  The prediction is that ``excess`` should track ``bank``,
and that compensating should collapse it.  **Check the mechanism moved before
reading the outcome** (CLAUDE.md): a null on the landings with ``excess``
unchanged means the knob never reached the decision.

    ./spaceplane/tools/appsum.py logs/LOG24*
    ./spaceplane/tools/appsum.py --by-arm logs/LOG24*
"""
import argparse
import glob
import os
import re

APPROACH = re.compile(r"^\[\s*[\d.]+\]\s+APPROACH\s+.*?\bv=\s*([-\d.]+)"
                      r".*?\bbank=\s*([-+\d.]+).*?\bsink=\s*([-\d.]+)/\s*([-\d.]+)")
TO_FLARE = re.compile(r"APPROACH -> FLARE h=([\d.]+) v=([\d.]+) sink=([-\d.]+)")
TOUCHDOWN = re.compile(r"FLARE -> ROLLOUT sink=([-+\d.]+) speed=([\d.]+)")
CONFIG = re.compile(r"^\[\s*[\d.]+\]\s+config:\s*(.*?)\s*\[defaults ([0-9a-f]+)\]")
SAVE = re.compile(r"\bSAVE_NAME=(\S+?)(?:,|\s|$)")
DROP = ("SAVE_NAME=", "TIMESCALE_GOVERNOR=", "TIMESCALE_GOVERNOR_MAX=",
        "LOOP_PACING_GAME_TIME=")


def read(path):
    banks, sinks, wants = [], [], []
    flare = touchdown = arm = fingerprint = save = None
    with open(path, errors="replace") as fh:
        for line in fh:
            if arm is None and "config:" in line:
                m = SAVE.search(line)
                if m:
                    save = m.group(1)
                m = CONFIG.match(line)
                if m:
                    bits = [b.strip() for b in m.group(1).split(",")
                            if b.strip() and not b.strip().startswith(DROP)]
                    arm, fingerprint = ", ".join(bits) or "(defaults)", m.group(2)
            m = APPROACH.match(line)
            if m:
                banks.append(abs(float(m.group(2))))
                sinks.append(float(m.group(3)))
                wants.append(float(m.group(4)))
                continue
            m = TO_FLARE.search(line)
            if m:
                flare = (float(m.group(1)), float(m.group(2)))
            m = TOUCHDOWN.search(line)
            if m:
                touchdown = float(m.group(2))
    if not banks:
        return None
    return {"log": os.path.basename(path), "save": save, "arm": arm,
            "fingerprint": fingerprint, "n": len(banks),
            "bank": sum(banks) / len(banks),
            "sink": sum(sinks) / len(sinks),
            "want": sum(wants) / len(wants),
            "flare": flare, "td": touchdown}


def stat(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return float("nan"), float("nan"), 0
    m = sum(xs) / len(xs)
    if len(xs) < 2:
        return m, 0.0, 1
    return m, (sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5, len(xs)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--by-arm", action="store_true")
    args = ap.parse_args(argv)

    paths = []
    for pattern in args.logs:
        paths.extend(sorted(glob.glob(pattern)) if "*" in pattern else [pattern])
    rows = [r for r in (read(p) for p in paths) if r]

    print("%-9s %5s | %5s | %6s %6s %7s | %6s %6s | %5s | %s"
          % ("log", "ticks", "bank", "sink", "wanted", "excess",
             "flare v", "flare h", "td", "arm"))
    for r in rows:
        fv, fh = (r["flare"][1], r["flare"][0]) if r["flare"] else (None, None)
        print("%-9s %5d | %5.1f | %6.1f %6.1f %+7.1f | %6s %6s | %5s | %s"
              % (r["log"], r["n"], r["bank"], r["sink"], r["want"],
                 r["sink"] - r["want"],
                 "%.1f" % fv if fv else "-", "%.0f" % fh if fh else "-",
                 "%.1f" % r["td"] if r["td"] else "-",
                 (r["arm"] or "?")[:34]))

    if args.by_arm or len({(r["arm"], r["fingerprint"]) for r in rows}) > 1:
        arms = {}
        for r in rows:
            arms.setdefault((r["save"] or "?", r["arm"] or "?",
                             r["fingerprint"] or "?"), []).append(r)
        print("\n%-4s | %-13s | %-15s | %-15s | %-13s | %s"
              % ("n", "mean |bank|", "sink excess", "flare speed",
                 "touchdown", "arm"))
        for (save, arm, fp), g in sorted(arms.items(), key=lambda kv: -len(kv[1])):
            bm, bs, _ = stat([r["bank"] for r in g])
            em, es, _ = stat([r["sink"] - r["want"] for r in g])
            fm, fs, fn = stat([r["flare"][1] for r in g if r["flare"]])
            tm, ts, tn = stat([r["td"] for r in g if r["td"]])
            print("%-4d | %5.1f sd %-5.1f | %+6.1f sd %-5.1f | %5.1f sd %-5.1f | "
                  "%5.1f sd %-5.1f | %s/%s [%s]"
                  % (len(g), bm, bs, em, es, fm, fs, tm, ts,
                     (save or "?")[:12], arm[:40], fp))
        print("\nsink excess is achieved minus the profile's own wanted sink.\n"
              "The flare is documented to complete from 83-91 m/s.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
