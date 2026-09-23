#!/usr/bin/env python3
"""What the rollout was handed, and whether it kept the vehicle.

TEMPORARY TEST HARNESS -- not part of the flight software.  ``landsum.py``
reports the landing *geometry* (arrival, surplus, ratio, where it stopped);
this reports the four seconds after the wheels touch, because that is where
this vehicle is actually being destroyed and no summary could see it.

The thing that made this necessary: every breakup on record begins **two to
four seconds after** ``FLARE -> ROLLOUT``, not on contact.  Read as
"destroyed on touchdown" for a long time, and it is not -- the vehicle
arrives intact, rolls, and then departs.  So the columns are the state at the
handover and the state one and two ticks later, side by side with what
survived:

    td       speed and sink at ``FLARE -> ROLLOUT``
    slip0/1  sideslip at the first and second rollout ticks, in degrees.
             On the ground this is the angle between where the wheels point
             and where the vehicle is going; it has been seen at 58 deg.
    aoa1     *achieved* body angle at the second tick.  60-130 deg is the
             vehicle broadside, which is a departure and not an attitude.
    xt0/1    cross-track, metres.  A pair that walks outward while the
             vehicle barely moves forward is a sideways slide.
    dec      mean deceleration over the first two ticks, m/s^2, computed
             **up to the first part loss** so that it is a cause and not a
             consequence -- debris and a broadside body decelerate hard, so
             a figure taken across the breakup measures the breakup.
    lost@    seconds from touchdown to the first part loss, and what went.
    parts    kept of total.

    ./spaceplane/tools/rollsum.py logs/LOG23*
    ./spaceplane/tools/rollsum.py --broken logs/LOG23*      # only the ones that lost parts

Read ``dec`` and ``slip`` together.  Gear-down drag on this airframe is about
1 m/s^2 and the brakes are worth a few more; anything in double figures is
the body, not the wheels.
"""
import argparse
import glob
import os
import re

# "[  40742.34] ROLLOUT  alt= ... v=  43.6 ... slip=-58.0 ... aoa= 7.1/58.0"
TICK = re.compile(r"^\[\s*([\d.]+)\]\s+ROLLOUT\s+.*?\bv=\s*([-\d.]+).*?"
                  r"\baoa=\s*([-\d.]+)/\s*([-\d.]+).*?\bslip=\s*([-+\d.]+)"
                  r".*?\bxt=\s*([-+\d.]+)")
HANDOVER = re.compile(r"^\[\s*([\d.]+)\]\s+FLARE -> ROLLOUT\s+"
                      r"sink=([-+\d.]+)\s+speed=([\d.]+)")
LOST = re.compile(r"^\[\s*([\d.]+)\]\s+lost (\d+) of (\d+) parts")
SILENT = re.compile(r"^\[\s*([\d.]+)\]\s+\d+ skin sensor\(s\) have gone "
                    r"silent[^:]*:\s*(.*)$")
DOWN = re.compile(r"^\[\s*([\d.]+)\]\s+(.*?) -- .*?\(along ([-+\d]+), "
                  r"across ([-+\d]+)\).*?(\d+) of (\d+) parts")
SAVE = re.compile(r"\bSAVE_NAME=(\S+?)(?:,|\s|$)")
CONFIG = re.compile(r"^\[\s*[\d.]+\]\s+config:\s*(.*?)\s*\[defaults ([0-9a-f]+)\]")
DESTROYED = re.compile(r"shutdown: destroyed in (\w+)")


def read(path):
    """One flight's rollout, or ``None`` if it never reached the ground."""
    handover = None
    ticks = []
    first_loss = None            # (ut, what)
    down = None
    save = None
    arm = None
    fingerprint = None
    destroyed = None
    with open(path, errors="replace") as fh:
        for line in fh:
            if arm is None and "config:" in line:
                m = SAVE.search(line)
                if m:
                    save = m.group(1)
                m = CONFIG.match(line)
                if m:
                    # The arm is the difference list with the save and the
                    # harness's own plumbing taken out: SAVE_NAME and the
                    # governor's per-instance path are not the experiment.
                    drop = ("SAVE_NAME=", "TIMESCALE_GOVERNOR=",
                            "TIMESCALE_GOVERNOR_MAX=", "LOOP_PACING_GAME_TIME=")
                    bits = [b.strip() for b in m.group(1).split(",")
                            if b.strip() and not b.strip().startswith(drop)]
                    arm = ", ".join(bits) or "(defaults)"
                    fingerprint = m.group(2)
            m = HANDOVER.match(line)
            if m:
                handover = (float(m.group(1)), float(m.group(2)),
                            float(m.group(3)))
                continue
            m = TICK.match(line)
            if m:
                ticks.append({"ut": float(m.group(1)),
                              "v": float(m.group(2)),
                              "aoa_cmd": float(m.group(3)),
                              "aoa": float(m.group(4)),
                              "slip": float(m.group(5)),
                              "xt": float(m.group(6))})
                continue
            if first_loss is None:
                m = SILENT.match(line)
                if m:
                    first_loss = (float(m.group(1)), m.group(2).strip())
                else:
                    m = LOST.match(line)
                    if m:
                        first_loss = (float(m.group(1)), "?")
            m = DESTROYED.search(line)
            if m:
                destroyed = m.group(1)
            m = DOWN.match(line)
            if m and ("runway" in m.group(2) or "DOWN" in line):
                down = {"verdict": m.group(2),
                        "along": float(m.group(3)),
                        "across": float(m.group(4)),
                        "kept": int(m.group(5)),
                        "total": int(m.group(6))}
    if handover is None:
        return None
    return {"log": os.path.basename(path), "save": save, "arm": arm,
            "fingerprint": fingerprint, "destroyed": destroyed,
            "handover": handover, "ticks": ticks,
            "first_loss": first_loss, "down": down}


def decel(flight):
    """Mean deceleration from touchdown, stopped at the first part loss.

    A figure taken *through* a breakup is a measurement of the breakup: the
    body goes broadside and the debris stops. So the window ends at the first
    silent sensor, and a flight that lost a part inside one tick reports
    whatever it had before that.
    """
    ut0, _sink, v0 = flight["handover"]
    cut = flight["first_loss"][0] if flight["first_loss"] else None
    last = None
    for t in flight["ticks"]:
        if cut is not None and t["ut"] > cut:
            break
        last = t
    if last is None or last["ut"] - ut0 < 0.3:
        return None
    return (v0 - last["v"]) / (last["ut"] - ut0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--broken", action="store_true",
                    help="only flights that lost a part")
    ap.add_argument("--intact", action="store_true",
                    help="only flights that kept everything")
    ap.add_argument("--by-arm", action="store_true",
                    help="always print the per-arm table, even for one arm")
    ap.add_argument("--near", type=float, default=None,
                    help="only flights that came to rest within this many "
                         "metres along-track of the runway midpoint.  A "
                         "vehicle that hit the sea four kilometres out did "
                         "not have a touchdown and its sink rate is not a "
                         "measurement of a flare.")
    args = ap.parse_args(argv)

    paths = []
    for pattern in args.logs:
        paths.extend(sorted(glob.glob(pattern)) if "*" in pattern else [pattern])

    rows = []
    for path in paths:
        try:
            f = read(path)
        except OSError:
            continue
        if not f:
            continue
        d = f["down"]
        broke = bool(d and d["kept"] < d["total"])
        if args.broken and not broke:
            continue
        if args.intact and broke:
            continue
        if args.near is not None:
            # No DOWN line means no vessel left to file one, which is a long
            # way from the runway by any reading -- excluded, not kept.
            if d is None or abs(d["along"]) > args.near:
                continue
        rows.append(f)

    print("%-9s %-13s %5s %5s | %6s %6s %5s | %6s %6s | %6s | %-7s %-4s %s"
          % ("log", "save", "td", "sink", "slip0", "slip1", "aoa1",
             "xt0", "xt1", "dec", "lost@", "kept", "first parts lost"))
    kept_dec, lost_dec = [], []
    for f in rows:
        ut0, sink, v0 = f["handover"]
        t0 = f["ticks"][0] if f["ticks"] else None
        t1 = f["ticks"][1] if len(f["ticks"]) > 1 else None
        d = f["down"]
        dec = decel(f)
        loss = f["first_loss"]
        broke = bool(d and d["kept"] < d["total"])
        (lost_dec if broke else kept_dec).append(dec)
        print("%-9s %-13s %5.1f %5.1f | %6s %6s %5s | %6s %6s | %6s | %-7s %-4s %s"
              % (f["log"], (f["save"] or "?")[:13], v0, sink,
                 "%+.1f" % t0["slip"] if t0 else "-",
                 "%+.1f" % t1["slip"] if t1 else "-",
                 "%.0f" % t1["aoa"] if t1 else "-",
                 "%+.0f" % t0["xt"] if t0 else "-",
                 "%+.0f" % t1["xt"] if t1 else "-",
                 "%.1f" % dec if dec is not None else "-",
                 ("%.1fs" % (loss[0] - ut0)) if loss else "-",
                 ("%d/%d" % (d["kept"], d["total"])) if d else "-",
                 (loss[1][:44] if loss else "")))

    def stat(xs):
        xs = [x for x in xs if x is not None]
        if not xs:
            return float("nan"), float("nan"), 0
        m = sum(xs) / len(xs)
        if len(xs) < 2:
            return m, 0.0, len(xs)
        var = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
        return m, var ** 0.5, len(xs)

    # **Classify on the part loss, not on the DOWN line.**  A vehicle that
    # comes apart badly enough never files one -- there is no vessel left to
    # report -- so scoring "kept everything" by the absence of a DOWN line
    # counts the worst flights as the best.  That is what the first version
    # of this did, and it read 68 of 81 intact when the real figure is 31.
    groups = {"intact": [], "on contact": [], "in rollout": []}
    for f in rows:
        loss = f["first_loss"]
        if loss is None:
            groups["intact"].append(f)
        elif loss[0] - f["handover"][0] < 1.5:
            groups["on contact"].append(f)
        else:
            groups["in rollout"].append(f)

    print("\n%d flights" % len(rows))
    print("%-12s %5s | %-17s | %-17s | %-17s"
          % ("", "n", "sink at handover", "touchdown speed", "decel to loss"))
    for name in ("intact", "on contact", "in rollout"):
        g = groups[name]
        sm, ss, sn = stat([f["handover"][1] for f in g])
        vm, vs, _ = stat([f["handover"][2] for f in g])
        dm, ds, _ = stat([decel(f) for f in g])
        print("%-12s %5d | %+7.1f sd %-6.1f | %7.1f sd %-6.1f | %7.1f sd %-5.1f"
              % (name, len(g), sm, ss, vm, vs, dm, ds))
    print("\nsink is the value on the FLARE -> ROLLOUT line: positive is still\n"
          "descending, negative is the flare having arrested past level.")

    # **Per arm, and the number is survival.**  ``armsum.py`` groups the same
    # way but reports the along-track miss at the gate, which is the entry's
    # figure of merit; this vehicle's open problem is that it reaches the
    # runway and comes apart on it, and no summary reported that per arm.
    #
    # The denominator is every flight that reached the rollout, so a
    # configuration that stops arriving cannot win by not being scored.
    arms = {}
    for f in rows:
        # The save is part of the arm: the same configuration on two entry
        # states is two experiments, and this project has misfiled a batch
        # for less.
        arms.setdefault((f["save"] or "?", f["arm"] or "?",
                         f["fingerprint"] or "?"), []).append(f)
    if len(arms) > 1 or args.by_arm:
        print("\n%-4s %-5s %-5s | %-17s | %-16s | %-14s | %s"
              % ("n", "whole", "wreck", "parts kept, mean", "stopped along",
                 "save", "arm"))
        for (save, arm, fp), g in sorted(arms.items(),
                                         key=lambda kv: -len(kv[1])):
            whole = sum(1 for f in g if f["first_loss"] is None)
            gone = sum(1 for f in g if f["destroyed"])
            kept = [f["down"]["kept"] for f in g if f["down"]]
            mk = sum(kept) / len(kept) if kept else 0.0
            # Where it stopped, from the runway midpoint: the runway is
            # +/-1200 m, so this is the pass/fail as well as the thing the
            # landing is being moved earlier *along*.
            am, asd, an = stat([f["down"]["along"] for f in g if f["down"]])
            print("%-4d %-5s %-5s | %-17s | %-16s | %-14s | %s [%s]"
                  % (len(g), "%d" % whole, "%d" % gone,
                     "%.1f of 23 (n=%d)" % (mk, len(kept)),
                     ("%+.0f sd %.0f" % (am, asd)) if an else "-",
                     save[:14], arm[:60], fp))
        print("\nwhole = lost no part at all; wreck = shutdown: destroyed.\n"
              "A flight that was destroyed files no DOWN line, so it is "
              "absent from\nthe parts-kept mean and present in the wreck "
              "count -- read both.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
