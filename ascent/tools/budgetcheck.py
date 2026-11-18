#!/usr/bin/env python3
"""The ascent's separation budget against flown booster logs.

    ./ascent/tools/budgetcheck.py logs/LOG17830 [...] [--engines 13]

For each boosterland log: ``budget.recovery`` from its SEPARATION line, its
fall on (a) the booster's ``dragcache`` curve -- what the ascent budgets
with -- and (b) the Cd*A the log's own coast flew (its ``cda`` column by
Mach, falling); against the flight's mass at the coast, the speed the
landing burn lit at, and the propellant the coast and the burn spent.

The engines are the booster's (``--engines`` KS-25s by default: vacuum
1000 kN / Isp 315, pad 937.3 kN / 295); the air is kspSim's Kerbin at the
equator, half-lit.  The drag cache entry is the one whose note names a
log given here (the booster that log flew), else ``--key``.
"""

import argparse
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from ascent import budget                       # noqa: E402
from boosterland import dragcache               # noqa: E402
from kspSim.atmosphere import Atmosphere        # noqa: E402

PAD_ALT = 72.4
R_PAD = 600072.4
MU = 3.5316e12


def air():
    atm = Atmosphere()
    rho, sound = [], []
    for k in range(72):
        alt = PAD_ALT + k * 1000.0
        p = atm.pressure(alt)
        r = atm.density(p, atm.temperature(alt, 0.0, 0.5))
        rho.append(r)
        sound.append(atm.sound_speed(p, r) if r > 0 else 300.0)
    return budget.Body(mu=MU, radius=R_PAD, density_pad=rho[0],
                       density=tuple(rho), step=1000.0), sound


def by_mach(machs, areas, sound):
    def cda(h, v):
        k = min(len(sound) - 1, max(0, int(h / 1000.0)))
        return dragcache.interp(machs, areas, v / max(1.0, sound[k]))
    return cda


def field(key, line):
    m = re.search(r"\b" + key + r"=\s*([-+\d.]+)", line)
    return float(m.group(1)) if m else None


def read(path):
    lines = open(path, errors="replace").read().splitlines()
    body = [ln.split("] ", 1)[-1] for ln in lines]
    sep = next(ln for ln, b in zip(lines, body) if b.startswith("SEPARATION "))
    prop = {}
    for ln in lines:
        m = re.search(r"propellant ([\d.]+) t .* dry ([\d.]+) t \((\w+)\)", ln)
        if m:
            prop[m.group(3)] = (float(m.group(1)), float(m.group(2)))
    lit = next(i for i, ln in enumerate(lines)
               if "phase COAST -> LANDING_BURN" in ln)
    coast = [ln for ln in lines[:lit] if "] COAST " in ln and "alt=" in ln]
    pts = {}
    for ln in coast:
        if field("vs", ln) < -100.0:
            pts[round(field("M", ln) * 20.0) / 20.0] = field("cda", ln)
    machs = sorted(pts)
    last = coast[-1]
    return dict(sep=sep, prop=prop, machs=machs,
                areas=[pts[m] for m in machs],
                lit_v=math.hypot(field("hs", last), field("vs", last)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--engines", type=int, default=13)
    ap.add_argument("--key", help="dragcache key (default: the one entry)")
    args = ap.parse_args()
    body, sound = air()
    cache = dragcache.load()
    names = {os.path.basename(x) for x in args.logs}
    mine = [k for k, v in cache.items() if v.get("note") in names]
    key = args.key or (mine[0] if len(mine) == 1 else
                       next(iter(cache)) if len(cache) == 1 else None)
    if key is None:
        notes = {k: v.get("note", "") for k, v in cache.items()}
        sys.exit("several dragcache entries, pass --key: %s" % notes)
    cached = by_mach(cache[key]["machs"], cache[key]["areas"], sound)
    n = args.engines
    print("%-9s %18s | %26s | %26s | %s" % (
        "log", "flown coast/lit", "budget, cache: coast lit land",
        "budget, own fall", "coast spent, burn spent t"))
    for path in args.logs:
        f = read(path)
        sep = f["sep"]
        p_coast, dry = f["prop"]["coast"]
        p_burn, _ = f["prop"]["landing_burn"]
        mass = field("m", sep) * 1000.0
        booster = budget.Booster(
            mass=mass, dry_mass=dry * 1000.0, thrust_vac=n * 1.0e6,
            thrust_sl=n * 0.9373e6, isp_vac=315.0, isp_sl=295.0, radius=2.61,
            gimbal_accel=0.0, fixed_accel=0.059)
        state = budget.State(field("alt", sep) - PAD_ALT, field("pad", sep),
                             field("hs", sep), 0.0, field("vs", sep))
        cols = []
        for cda in (cached, by_mach(f["machs"], f["areas"], sound)):
            rec = budget.recovery(body, booster, state, cda=cda)
            m_bb = mass * math.exp(-rec.boostback_dv / (315.0 * budget.G0))
            land = m_bb - rec.propellant_left - dry * 1000.0
            cols.append("%6.2f t %4.0f %5.2f t" % (m_bb / 1000.0, rec.terminal,
                                                  land / 1000.0))
        print("%-9s %8.2f t %4.0f m/s | %26s | %26s | %5.2f %5.2f" % (
            os.path.basename(path), dry + p_coast, f["lit_v"], cols[0],
            cols[1], p_coast - p_burn, p_burn))


if __name__ == "__main__":
    main()
