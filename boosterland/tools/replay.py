#!/usr/bin/env python3
"""Re-fly a real flight's predictions offline, from a DIAG_STATE log.

TEMPORARY DIAGNOSTIC HARNESS -- not part of the flight software.

The question this exists to answer: on the real vehicle the predicted miss
walks monotonically out during a *ballistic* coast -- LOG49 goes 41 m at
32 km to 620 m at the landing burn, with the engines off the whole way --
while the same coast in ``boosterland/tests/fakeksp`` oscillates around zero and lands
at 18 m.  A fresh propagation of an unpowered arc should not change its
answer, so either the air the propagator is reading is being revised under
it, or the propagation disagrees with the vehicle.  The log could not tell
those apart, because ``cda=`` records only the Cd*A at the Mach the booster
happened to be doing and the propagator reads a whole curve.

``Config.DIAG_STATE`` logs the state vector, the whole curve and the
atmosphere tables; this reads them back and re-propagates, so a tick can be
re-flown with air it did not have:

    ./boosterland/tools/replay.py logs/LOG54                  # walk of the prediction
    ./boosterland/tools/replay.py logs/LOG54 --curve-from -1  # every tick, with the final curve
    ./boosterland/tools/replay.py logs/LOG54 --scale 0.8      # ... or with the curve scaled

If re-flying the early ticks with the *late* curve removes the walk, the
curve was being revised and the estimator is what to fix.  If the walk
survives, the propagation itself disagrees with the vehicle and the curve is
innocent.
"""

import os
import argparse
import math
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from boosterland import trajectory
from common import vec
from boosterland.config import Config, apply_overrides

ENV = re.compile(
    r"diag env target=(\S+) pad=(\S+) target_radius=(\S+) mu=(\S+) R=(\S+) "
    r"atmo=(\S+) alt_step=(\S+) omega=(\S+)")
TICK = re.compile(
    r"\[\s*([\d.]+)\] diag r=(\S+) v=(\S+) m=(\S+) acc=(\S+) pred=(\S+) "
    r"ttl=(\S+)(?: probe=(\S+))? curve=\[([^\]]*)\]")
TABLE = re.compile(r"diag (rho|sound)=\[([^\]]*)\]")
COMPACT = re.compile(
    r"\[\s*([\d.]+)\] (\w+)\s+alt=\s*(-?[\d.]+).*?pad=\s*(-?[\d.]+) "
    r"miss=\s*(-?[\d.]+)")


def _triple(text):
    return tuple(float(x) for x in text.split(","))


class ReplayEnv(object):
    """An ``Environment`` rebuilt from a log: same tables, frozen curve."""

    def __init__(self, header, rho, sound, curve, scale=1.0):
        self.target = header["target"]
        self.pad = header["pad"]
        self.target_radius = header["target_radius"]
        self.mu = header["mu"]
        self.equatorial_radius = header["R"]
        self.atmosphere_depth = header["atmo"]
        self.omega = header["omega"]
        self._alt_step = header["alt_step"]
        self._rho = rho
        self._sound = sound
        self.set_curve(curve, scale)

    def set_curve(self, curve, scale=1.0):
        self._machs = [m for m, _ in curve]
        self._areas = [a * scale for _, a in curve]
        self.drag_area = self._areas[0] if self._areas else 0.0

    def _interp(self, table, altitude):
        if len(table) < 2:
            return table[0] if table else 0.0
        x = max(0.0, altitude) / self._alt_step
        i = int(x)
        if i >= len(table) - 1:
            return table[-1]
        f = x - i
        return table[i] * (1.0 - f) + table[i + 1] * f

    def density(self, altitude):
        if len(self._rho) < 2 or altitude >= self.atmosphere_depth:
            return 0.0
        return self._interp(self._rho, altitude)

    def speed_of_sound(self, altitude):
        return self._interp(self._sound, altitude)

    def mach(self, speed, altitude):
        c = self.speed_of_sound(altitude)
        return speed / c if c > 0.0 else 0.0

    def drag_area_at(self, speed, altitude):
        if not self._machs:
            return 0.0
        mach = self.mach(speed, altitude)
        if mach <= self._machs[0]:
            return self._areas[0]
        if mach >= self._machs[-1]:
            return self._areas[-1]
        for i in range(1, len(self._machs)):
            if self._machs[i] >= mach:
                lo, hi = self._machs[i - 1], self._machs[i]
                f = (mach - lo) / (hi - lo) if hi > lo else 0.0
                return self._areas[i - 1] * (1.0 - f) + self._areas[i] * f
        return self._areas[-1]


def read(path):
    header, rho, sound, ticks, compact = None, [], [], [], []
    for line in open(path):
        m = ENV.search(line)
        if m:
            header = {
                "target": _triple(m.group(1)), "pad": _triple(m.group(2)),
                "target_radius": float(m.group(3)), "mu": float(m.group(4)),
                "R": float(m.group(5)), "atmo": float(m.group(6)),
                "alt_step": float(m.group(7)), "omega": _triple(m.group(8)),
            }
            continue
        m = TABLE.search(line)
        if m:
            values = [float(x) for x in m.group(2).split()]
            (rho if m.group(1) == "rho" else sound).extend(values)
            continue
        m = TICK.search(line)
        if m:
            curve = []
            for pair in m.group(9).split():
                mach, area = pair.split(":")
                curve.append((float(mach), float(area)))
            ticks.append({
                "ut": float(m.group(1)), "r": _triple(m.group(2)),
                "v": _triple(m.group(3)), "mass": float(m.group(4)),
                "accel": float(m.group(5)), "pred": _triple(m.group(6)),
                "ttl": float(m.group(7)), "probe": m.group(8),
                "curve": curve,
            })
            continue
        m = COMPACT.search(line)
        if m:
            compact.append({"ut": float(m.group(1)), "phase": m.group(2),
                            "alt": float(m.group(3)),
                            "pad": float(m.group(4)),
                            "miss": float(m.group(5))})
    return header, rho, sound, ticks, compact


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("log")
    p.add_argument("--curve-from", type=int, default=None, metavar="N",
                   help="re-fly every tick with tick N's curve (-1 = last)")
    p.add_argument("--scale", type=float, default=1.0,
                   help="multiply the whole Cd*A curve by this")
    p.add_argument("--powered", action="store_true",
                   help="propagate with the landing burn (as the flight did)")
    p.add_argument("--omega", choices=("measured", "negated", "zero"),
                   default="measured",
                   help="the rotating-frame terms are the one part of the "
                        "propagator boosterland/tests/fakeksp cannot exercise -- it sets "
                        "rotational_speed = 0 -- so a sign error there would "
                        "be invisible offline and would grow with the "
                        "prediction horizon, which is the shape of the walk")
    p.add_argument("--set", action="append", default=[], metavar="K=V")
    args = p.parse_args(argv)

    cfg = Config()
    apply_overrides(cfg, args.set)
    header, rho, sound, ticks, compact = read(args.log)
    if header is None or not ticks:
        sys.exit("%s has no DIAG_STATE lines -- fly with --set DIAG_STATE=True"
                 % args.log)

    fixed = None
    if args.curve_from is not None:
        fixed = ticks[args.curve_from]["curve"]
    env = ReplayEnv(header, rho, sound, fixed or ticks[0]["curve"], args.scale)
    if args.omega == "negated":
        env.omega = vec.scale(env.omega, -1.0)
    elif args.omega == "zero":
        env.omega = (0.0, 0.0, 0.0)

    by_ut = {round(c["ut"], 2): c for c in compact}
    print("# %s   %d ticks, %d curve bins, rho %d sound %d"
          % (args.log, len(ticks), len(ticks[0]["curve"]), len(rho),
             len(sound)))
    print("#   ut      phase        alt     pad   logged  replayed   delta  from_pad")
    for tick in ticks:
        env.set_curve(fixed or tick["curve"], args.scale)
        accel = tick["accel"] if args.powered else 0.0
        pred = trajectory.predict_landing(
            env, tick["r"], tick["v"], tick["mass"], accel, cfg)
        miss = trajectory.surface_distance(env, pred.position, env.target)
        # Where the propagation says it will touch down relative to the *pad*
        # is the column that can be checked against the flight: the booster
        # really did land 12 m from the pad, whatever the aim bias was.
        from_pad = trajectory.surface_distance(env, pred.position, env.pad)
        row = by_ut.get(round(tick["ut"], 2))
        logged = row["miss"] if row else float("nan")
        print("%9.2f %-11s %8.0f %7.0f %8.0f %9.0f %7.0f %9.0f"
              % (tick["ut"], row["phase"] if row else "?",
                 row["alt"] if row else float("nan"),
                 row["pad"] if row else float("nan"),
                 logged, miss, miss - logged, from_pad))


if __name__ == "__main__":
    main()
