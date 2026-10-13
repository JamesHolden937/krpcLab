"""The stand-in: a point-mass vessel with a constant-thrust engine around a
spherical planet.  No kRPC, no KSP -- the physics here is the physics KSP
uses in vacuum (two-body gravity, rocket equation), so the guidance can be
developed against it and trusted before it is translated to the game.

Each tick the guidance is asked once and its command held for the tick, as
the live pilot will hold a command between kRPC calls.  A coasting tick is
propagated exactly (``orbit.coast``); a thrusting one by RK4.
"""

import math
from collections import namedtuple
from dataclasses import dataclass

from common import vec
from lowThrust import guidance, orbit

G0 = 9.80665


@dataclass
class Body:
    name: str
    mu: float           # m^3/s^2
    radius: float       # m
    atmosphere: float   # m above the surface; periapsis must stay above it


@dataclass
class Vessel:
    dry_kg: float
    prop_kg: float
    thrust_n: float
    isp_s: float


# Kerbin's numbers, so what works here is what the game will need.
KERBIN = Body("Kerbin", 3.5316e12, 600e3, 70e3)
# One IX-6315 "Dawn" (2 kN, Isp 4200 s vacuum) on a 4 t probe with 1 t of
# xenon: 0.4 m/s^2 full, ~9 km/s.
ION_PROBE = Vessel(dry_kg=4000.0, prop_kg=1000.0, thrust_n=2000.0, isp_s=4200.0)

Burn = namedtuple("Burn", "t0 t1 dv nu0 nu1")
Result = namedtuple(
    "Result", "done reason time dv prop_used orbits burns el end_r end_v "
              "end_mass")


def state_at_periapsis(body, ap_alt, pe_alt, inc=0.0, lan=0.0, argpe=0.0):
    """(r, v) at periapsis of the orbit with these altitudes (m) and angles
    (rad: inclination, longitude of the ascending node, argument of
    periapsis), in the body's frame (equator = x-y)."""
    ra, rp = body.radius + ap_alt, body.radius + pe_alt
    if ra < rp:
        ra, rp = rp, ra
    r, v = (rp, 0.0, 0.0), (0.0, orbit.speed_at(body.mu, rp, ra, rp), 0.0)

    def rot(x):          # Rz(lan) Rx(inc) Rz(argpe)
        c, s = math.cos(argpe), math.sin(argpe)
        x = (c * x[0] - s * x[1], s * x[0] + c * x[1], x[2])
        c, s = math.cos(inc), math.sin(inc)
        x = (x[0], c * x[1] - s * x[2], s * x[1] + c * x[2])
        c, s = math.cos(lan), math.sin(lan)
        return (c * x[0] - s * x[1], s * x[0] + c * x[1], x[2])
    return rot(r), rot(v)


def _rk4(r, v, m, mu, force, mdot, dt):
    """One RK4 step of gravity plus thrust ``force`` (inertial vector, N)."""
    def deriv(r_, v_, m_):
        rm = vec.norm(r_)
        g = vec.scale(r_, -mu / rm ** 3)
        return v_, vec.add(g, vec.scale(force, 1 / m_)), -mdot

    k1 = deriv(r, v, m)
    k2 = deriv(vec.add(r, vec.scale(k1[0], dt / 2)),
               vec.add(v, vec.scale(k1[1], dt / 2)), m + k1[2] * dt / 2)
    k3 = deriv(vec.add(r, vec.scale(k2[0], dt / 2)),
               vec.add(v, vec.scale(k2[1], dt / 2)), m + k2[2] * dt / 2)
    k4 = deriv(vec.add(r, vec.scale(k3[0], dt)),
               vec.add(v, vec.scale(k3[1], dt)), m + k3[2] * dt)

    def comb(i):
        return vec.add(vec.add(k1[i], vec.scale(k2[i], 2)),
                       vec.add(vec.scale(k3[i], 2), k4[i]))
    return (vec.add(r, vec.scale(comb(0), dt / 6)),
            vec.add(v, vec.scale(comb(1), dt / 6)),
            m + (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) * dt / 6)


def fly(body, vessel, r, v, target, eta_cut, cfg, record=True):
    """Fly the transfer to ``target`` (a ``guidance.Target``) from (r, v)
    with effectivity cutoff ``eta_cut``.  Returns a ``Result``; ``record``
    keeps the burn list.
    """
    mu = body.mu
    dry = vessel.dry_kg
    m = dry + vessel.prop_kg
    mdot_full = vessel.thrust_n / (vessel.isp_s * G0)
    t = dv = 0.0
    burns = []
    burn_open = None
    el = orbit.elements(r, v, mu)
    period0 = el.period if el else 0.0

    floor = body.radius + body.atmosphere

    def finish(done, reason):
        if burn_open is not None and record:
            burns.append(Burn(burn_open[0], t, burn_open[1], burn_open[2],
                              el.nu if el else 0.0))
        return Result(done, reason, t, dv, dry + vessel.prop_kg - m,
                      t / period0 if period0 else 0.0, burns, el, r, v, m)

    while True:
        el = orbit.elements(r, v, mu)
        if el is None:
            return finish(False, "escaped")
        if el.rp < floor:
            return finish(False, "periapsis inside the atmosphere")
        if t > cfg.MAX_ORBITS * period0:
            return finish(False, "orbit limit")
        tick = el.period / cfg.TICKS_PER_ORBIT
        accel = vessel.thrust_n / m
        target = guidance.lock(target, el, cfg, coasting=burn_open is None)
        cmd = guidance.command(el, target, accel, tick, eta_cut, cfg, mu)
        if cmd.done:
            return finish(True, "arrived")
        if m - dry <= 0 and cmd.throttle > 0:
            return finish(False, "out of propellant")

        if cmd.throttle <= 0:
            if burn_open is not None:
                if record:
                    burns.append(Burn(burn_open[0], t, burn_open[1],
                                      burn_open[2], el.nu))
                burn_open = None
            r, v = orbit.coast(r, v, mu, tick)
            t += tick
            continue

        _, T, N = orbit.rtn(r, v)
        force = vec.scale(vec.add(vec.scale(T, cmd.prograde),
                                  vec.scale(N, cmd.normal)),
                          vessel.thrust_n * cmd.throttle)
        mdot = mdot_full * cmd.throttle
        m0 = m
        h = tick / cfg.THRUST_SUBSTEPS
        for _ in range(cfg.THRUST_SUBSTEPS):
            r, v, m = _rk4(r, v, m, mu, force, mdot, h)
        m = max(m, dry)
        burn_dv = vessel.isp_s * G0 * math.log(m0 / m)
        dv += burn_dv
        t += tick
        if burn_open is None:
            burn_open = [t - tick, 0.0, el.nu]
        burn_open[1] += burn_dv
