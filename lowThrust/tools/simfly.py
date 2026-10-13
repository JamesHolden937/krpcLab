#!/usr/bin/env python3
"""Plan and fly an apsides change in the stand-in simulator -- no KSP.

    ./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500"
    ./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=2000" --within 6h
    ./lowThrust/tools/simfly.py ... --table           # every cutoff the planner flew

Altitudes in km above the surface; an apsis left out of --target is free.
--within takes s, m, h or d (a Kerbin day, 6 h).  Prints the plan, the
two-burn impulsive dv it is measured against, and the burn schedule.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from lowThrust import config, guidance, orbit, plan, sim  # noqa: E402

UNITS = {"s": 1, "m": 60, "h": 3600, "d": 6 * 3600}


def parse_orbit(text):
    out = {}
    for item in text.replace(",", " ").split():
        k, _, val = item.partition("=")
        if k not in ("ap", "pe"):
            raise SystemExit(f"unknown orbit field {k!r} (ap, pe)")
        out[k] = float(val) * 1000
    return out


def parse_duration(text):
    unit = text[-1]
    if unit in UNITS:
        return float(text[:-1]) * UNITS[unit]
    return float(text)


def hms(s):
    d, s = divmod(s, 6 * 3600)
    h, s = divmod(s, 3600)
    m, s = divmod(s, 60)
    return (f"{int(d)}d " if d else "") + f"{int(h)}h{int(m):02d}m{int(s):02d}s"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--start", required=True)
    ap.add_argument("--target", required=True)
    ap.add_argument("--within", help="time budget, e.g. 90m, 6h, 2d")
    ap.add_argument("--dry", type=float, default=sim.ION_PROBE.dry_kg)
    ap.add_argument("--prop", type=float, default=sim.ION_PROBE.prop_kg)
    ap.add_argument("--thrust", type=float, default=sim.ION_PROBE.thrust_n)
    ap.add_argument("--isp", type=float, default=sim.ION_PROBE.isp_s)
    ap.add_argument("--table", action="store_true",
                    help="list every cutoff the planner flew")
    ap.add_argument("--burns", type=int, default=8, help="burns to list")
    ap.add_argument("--set", action="append", default=[])
    args = ap.parse_args()

    cfg = config.apply_overrides(config.Config(), args.set)
    body = sim.KERBIN
    vessel = sim.Vessel(args.dry, args.prop, args.thrust, args.isp)
    start = parse_orbit(args.start)
    s_ap = start.get("ap", start.get("pe"))
    s_pe = start.get("pe", s_ap)
    r, v = sim.state_at_periapsis(body, s_ap, s_pe)
    tgt = parse_orbit(args.target)
    target = guidance.Target(
        body.radius + tgt["ap"] if "ap" in tgt else None,
        body.radius + tgt["pe"] if "pe" in tgt else None)
    el0 = orbit.elements(r, v, body.mu)

    print(f"config: {' '.join(config.differences(cfg)) or '(defaults)'} "
          f"defaults_fingerprint={config.defaults_fingerprint()}")
    print(f"{body.name}: start ap {(el0.ra - body.radius) / 1e3:.0f} km "
          f"pe {(el0.rp - body.radius) / 1e3:.0f} km  ->  target "
          + " ".join(f"{k} {val / 1e3:.0f} km" for k, val in tgt.items()))
    print(f"vessel {vessel.dry_kg + vessel.prop_kg:.0f} kg, "
          f"{vessel.thrust_n:.0f} N, accel "
          f"{vessel.thrust_n / (vessel.dry_kg + vessel.prop_kg):.3f} m/s^2")

    within = parse_duration(args.within) if args.within else None
    p = plan.plan(body, vessel, r, v, target, cfg, within)
    print(f"budget {hms(within) if within else 'none (cheapest)'}")
    res = p.result
    ra1 = target.ra if target.ra is not None else res.el.ra
    rp1 = target.rp if target.rp is not None else res.el.rp
    ref = orbit.impulsive_dv(body.mu, el0.ra, el0.rp, ra1, rp1)

    print(f"{'PLAN' if p.feasible else 'NO PLAN'}: {p.reason}")
    if p.table and args.table:
        print("  cutoffs flown (eta: time, dv):")
        for eta, done, t, dv in p.table:
            print(f"    {eta:.3f}: {hms(t):>14}  {dv:7.1f} m/s"
                  + ("" if done else "  (did not arrive)"))
    print(f"  eta cutoff {p.eta:.3f}  time {hms(res.time)} "
          f"({res.orbits:.1f} start orbits)  dv {res.dv:.1f} m/s  "
          f"xenon {res.prop_used:.2f} kg")
    print(f"  two-burn impulsive {ref:.1f} m/s  ->  low-thrust costs "
          f"{(res.dv / ref - 1) * 100 if ref else 0:+.1f}%")
    if res.el:
        print(f"  final ap {(res.el.ra - body.radius) / 1e3:.3f} km  "
              f"pe {(res.el.rp - body.radius) / 1e3:.3f} km  "
              f"({res.reason})")
    print(f"  {len(res.burns)} burns")
    for b in res.burns[:args.burns]:
        print(f"    t={hms(b.t0):>14}  {b.t1 - b.t0:7.0f} s  "
              f"{b.dv:6.2f} m/s  nu {math.degrees(b.nu0):5.0f} -> "
              f"{math.degrees(b.nu1):4.0f} deg")
    if len(res.burns) > args.burns:
        print(f"    ... {len(res.burns) - args.burns} more")


if __name__ == "__main__":
    main()
