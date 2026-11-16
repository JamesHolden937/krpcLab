#!/usr/bin/env python3
"""The landing burn the prediction flew against the one the booster flew.

TEMPORARY DIAGNOSTIC HARNESS -- not part of the flight software.

Reads a ``DIAG_BURN`` log: the ``burn plan`` line (the propagator's burn from
the first burn tick, ``trajectory._fly_landing_burn``) and the ``burn tick``
lines (the flown burn on the same clock).  Prints, per flight, the horizontal
distance each covered and how long each took, then -- with ``--rows`` -- the
two side by side every ``--every`` seconds: height, horizontal and vertical
speed, the throttle each asked for, what the engines delivered, and how far
the nose sat off the braking direction.

    ./boosterland/tools/burnplan.py logs/LOG17600 logs/LOG17601
    ./boosterland/tools/burnplan.py --rows logs/LOG17600
"""

import argparse
import re

PLAN = re.compile(r"burn plan m=(\S+) acc=(\S+) .*samples=\[([^\]]*)\]")
TICK = re.compile(r"\[\s*([\d.]+)\] burn tick (.*)")


def parse(path):
    plan, ticks, meta = None, [], {}
    for line in open(path):
        m = PLAN.search(line)
        if m and plan is None:
            meta = {"m": float(m.group(1)), "acc": float(m.group(2))}
            plan = [tuple(float(x) for x in s.split(":"))
                    for s in m.group(3).split()]
            continue
        m = TICK.search(line)
        if m:
            ticks.append({k: float(v) for k, v in
                          (kv.split("=") for kv in m.group(2).split())})
    return plan, ticks, meta


def interp(plan, t, col):
    if t <= plan[0][0]:
        return plan[0][col]
    for a, b in zip(plan, plan[1:]):
        if a[0] <= t <= b[0]:
            f = (t - a[0]) / max(1e-9, b[0] - a[0])
            return a[col] * (1 - f) + b[col] * f
    return None


def travel(points):
    """Horizontal distance from (t, hs) pairs, trapezoids."""
    return sum(0.5 * (a[1] + b[1]) * (b[0] - a[0])
               for a, b in zip(points, points[1:]))


def at_height(rows, h, key_h, key):
    """The value of ``key`` where the series crosses height ``h`` (falling)."""
    for a, b in zip(rows, rows[1:]):
        if a[key_h] >= h >= b[key_h]:
            f = (a[key_h] - h) / max(1e-9, a[key_h] - b[key_h])
            return a[key] * (1 - f) + b[key] * f
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--rows", action="store_true")
    ap.add_argument("--every", type=float, default=1.0)
    a = ap.parse_args()
    print("%-9s %6s %6s %7s %7s %6s %6s %6s %6s"
          % ("log", "t_pln", "t_fly", "x_pln", "x_fly", "dx", "h0", "hs0",
             "acc0"))
    for path in a.logs:
        plan, ticks, meta = parse(path)
        if not plan or not ticks:
            print("%-9s no DIAG_BURN data" % path.split("/")[-1])
            continue
        # The flown burn up to contact (the plan stops at its own contact).
        flown = [x for x in ticks if x["h"] > 0.0] or ticks
        x_plan = travel([(p[0], p[2]) for p in plan])
        x_fly = travel([(x["t"], x["hs"]) for x in flown])
        print("%-9s %6.1f %6.1f %7.0f %7.0f %+6.0f %6.0f %6.1f %6.2f"
              % (path.split("/")[-1], plan[-1][0], flown[-1]["t"], x_plan,
                 x_fly, x_fly - x_plan, plan[0][1], plan[0][2], meta["acc"]))
        if not a.rows:
            continue
        print("   t |    h pln/fly | hs pln/fly | vs pln/fly | thr pln/cmd/got"
              " | dlv/max  off  M    cda")
        nxt = 0.0
        for x in ticks:
            if x["t"] + 1e-6 < nxt:
                continue
            nxt = x["t"] + a.every
            ph, phs, pvs, pthr = (interp(plan, x["t"], c) for c in (1, 2, 3, 4))
            fmt = lambda v, f: (f % v) if v is not None else "-"
            print("%5.1f | %6s %6.0f | %5s %5.1f | %5s %6.1f | %5s %5.2f %5.2f"
                  " | %5.1f/%5.1f %4.1f %4.2f %5.2f"
                  % (x["t"], fmt(ph, "%.0f"), x["h"], fmt(phs, "%.1f"),
                     x["hs"], fmt(pvs, "%.1f"), x["vs"], fmt(pthr, "%.2f"),
                     x["thr"], x["got"], x["acc"], x["max"], x["off"], x["M"],
                     x["cda"]))
        # The same two burns read by height: where they part.
        prow = [{"h": p[1], "hs": p[2], "vs": p[3], "t": p[0]} for p in plan]
        print("   by height:   h | hs pln/fly | vs pln/fly | t pln/fly")
        for h in (4000, 3000, 2000, 1500, 1000, 600, 300, 150, 50):
            vals = [at_height(prow, h, "h", k) for k in ("hs", "vs", "t")]
            fvals = [at_height(ticks, h, "h", k) for k in ("hs", "vs", "t")]
            if None in vals or None in fvals:
                continue
            print("              %5d | %5.1f %5.1f | %6.1f %6.1f | %5.1f %5.1f"
                  % (h, vals[0], fvals[0], vals[1], fvals[1], vals[2], fvals[2]))


if __name__ == "__main__":
    main()
