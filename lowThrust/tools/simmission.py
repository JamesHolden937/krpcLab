#!/usr/bin/env python3
"""Plan a low-thrust transfer between bodies in the stand-in solar system and
list the maneuvers it flies -- no KSP.

    ./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "mun: ap=50 pe=50"
    ./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "duna: ap=200 pe=200 i=0"
    ./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "ike: ap=50 pe=50"
    ./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "kerbin: ap=500 pe=500 i=6"
    ./lowThrust/tools/simmission.py ... --thrust 200 --mass 5000 --isp 4200 --ut 0

Orbits: ``body: ap= pe= [i= lan= argpe=]`` (km and degrees; anything left out
of --to is free).  Bodies: Moho, Eve, Kerbin, Mun, Minmus, Duna, Ike, Dres,
Jool, Eeloo.  Prints the phase timeline (Kerbin dates, 6 h days, 426 d years),
every burn (phase, body, start, duration, dv, direction), and the totals.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from lowThrust import bodies, config, guidance, mission, orbit, sim  # noqa: E402


def parse(text, system, start):
    name, _, rest = text.partition(":")
    body = system.get(name.strip().capitalize())
    if body is None:
        raise SystemExit(f"unknown body {name!r}")
    f = {}
    for item in rest.replace(",", " ").split():
        k, _, val = item.partition("=")
        allowed = ("ap", "pe", "i", "lan", "argpe") if start else \
            ("ap", "pe", "i", "lan")
        if k not in allowed:
            raise SystemExit(f"unknown field {k!r} ({', '.join(allowed)})")
        f[k] = float(val)
    return body, f


def dur(s):
    d, s = divmod(s, 21600)
    h, s = divmod(s, 3600)
    m, s = divmod(s, 60)
    return (f"{int(d)}d " if d else "") + f"{int(h)}h{int(m):02d}m{int(s):02d}s"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--from", dest="start", required=True)
    ap.add_argument("--to", dest="dest", required=True)
    ap.add_argument("--ut", type=float, default=0.0, help="start UT, s")
    ap.add_argument("--mass", type=float, default=sim.ION_PROBE.mass_kg,
                    help="kg, constant (propellant is unlimited)")
    ap.add_argument("--thrust", type=float, default=sim.ION_PROBE.thrust_n)
    ap.add_argument("--isp", type=float, default=sim.ION_PROBE.isp_s)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--burns", type=int, default=40, help="burns to list")
    args = ap.parse_args()

    cfg = config.apply_overrides(config.Config(), args.set)
    system = bodies.stock()
    b0, s = parse(args.start, system, True)
    b1, t = parse(args.dest, system, False)
    s_ap = s.get("ap", s.get("pe", 100.0)) * 1e3
    s_pe = s.get("pe", s.get("ap", 100.0)) * 1e3
    ra, rp = b0.radius + max(s_ap, s_pe), b0.radius + min(s_ap, s_pe)
    r, v = orbit.state_from_elements(
        b0.mu, (ra + rp) / 2, (ra - rp) / (ra + rp), math.radians(s.get("i", 0)),
        math.radians(s.get("lan", 0)), math.radians(s.get("argpe", 0)), 0.0)
    target = guidance.Target(
        b1.radius + t["ap"] * 1e3 if "ap" in t else None,
        b1.radius + t["pe"] * 1e3 if "pe" in t else None,
        math.radians(t["i"]) if "i" in t else None,
        math.radians(t["lan"]) if "lan" in t else None)
    vessel = sim.Vessel(args.mass, args.thrust, args.isp)

    print(f"config: {' '.join(config.differences(cfg)) or '(defaults)'} "
          f"defaults_fingerprint={config.defaults_fingerprint()}")
    print(f"vessel {args.mass:.0f} kg, {args.thrust:.0f} N "
          f"({args.thrust / args.mass:.3f} m/s^2, TWR "
          f"{args.thrust / args.mass / 9.81:.4f}), Isp {args.isp:.0f} s "
          "(mass constant, propellant unlimited)")
    print(f"route: {b0.name} -> " + " -> ".join(
        x.name for x in mission.route(b0, b1)))
    res = mission.fly(system, vessel, b0, r, v, args.ut, b1, target, cfg)

    print("\ntimeline")
    for e in res.events:
        print(f"  {mission.fmt_ut(e.t):>16}  {e.text}")
    print(f"\nburns ({len(res.maneuvers)}; direction = prograde/normal/"
          "radial share of the dv)")
    for m in res.maneuvers[:args.burns]:
        print(f"  {m.phase:9} {m.body:7} {mission.fmt_ut(m.t0):>16}  "
              f"{dur(m.t1 - m.t0):>12}  {m.dv:8.2f} m/s  "
              f"({m.prograde:+.2f} {m.normal:+.2f} {m.radial:+.2f})")
    if len(res.maneuvers) > args.burns:
        rest = res.maneuvers[args.burns:]
        print(f"  ... {len(rest)} more, {sum(m.dv for m in rest):.1f} m/s")
    by = {}
    for m in res.maneuvers:
        by[(m.body, m.phase)] = by.get((m.body, m.phase), 0.0) + m.dv
    print("\ndv by phase")
    for (body, phase), dv in by.items():
        print(f"  {body:7} {phase:9} {dv:8.1f} m/s")
    c = res.final
    print(f"\n{'ARRIVED' if res.done else 'FAILED'}: {res.reason}")
    print(f"  {res.body.name}: ap {(c.ra - res.body.radius) / 1e3:.1f} km  "
          f"pe {(c.rp - res.body.radius) / 1e3:.1f} km  "
          f"i {math.degrees(orbit.inclination_lan(c.rvec, c.vvec)[0]):.2f} deg")
    print(f"  total {res.dv:.1f} m/s, {res.prop_used:.1f} kg xenon, "
          f"{dur(res.t - res.t0)} ({mission.fmt_ut(res.t0)} -> "
          f"{mission.fmt_ut(res.t)})")


if __name__ == "__main__":
    main()
