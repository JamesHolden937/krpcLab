"""The solver: the Hohmann transfer split into many small burns.  Pure
functions, no kRPC.

Every tick, from the current orbit alone, it answers: thrust or coast,
prograde or retrograde, how hard.  There is no stored schedule -- a lost
tick, a weak engine or a shadow pass is corrected on the next pass by the
same law.

**Decoupled.**  A tangential burn at periapsis moves the apoapsis and, to
first order, not the periapsis; one at apoapsis moves only the periapsis.
So each apsis arc fixes one element: at the periapsis arc the apoapsis is
driven to its target, at the apoapsis arc the periapsis.  The last pass
nulls its element exactly, so the end is deadbeat.

**Fixed apse line.**  The two arcs sit on an apse line held fixed in
inertial space (``Target.apse``) through each burn, and the law controls
the orbit's radius *toward* those two directions, r(phi) = p / (1 + e.u),
which is smooth through circular.  Between burns the line is re-locked to
the real periapsis (``lock_apse``).

**Arcs, and the time budget.**  The arcs are centred on the two directions
with half-width acos(eta_cut) (a tangential burn's effectivity off-apsis is
about the cosine of the angle).  A burn starts half its duration before
the apsis and runs at full throttle until it is done or the arc ends, so a
burn is centred and no single pass is longer than the arc.  Narrow arcs are
many small near-impulsive burns, within a few percent of the impulsive
Hohmann; wide arcs deliver more per pass and finish sooner for more dv.
The planner (``plan.py``) picks the narrowest arcs that meet the deadline.

**Order.**  Outward first and inward first, as Hohmann: the periapsis is
raised only once the apoapsis is up (a periapsis raise is cheapest at the
final apoapsis), the apoapsis lowered only once the periapsis is down.

Thrust is prograde or retrograde only -- what plain SAS modes can hold.
"""

import math
from collections import namedtuple

from common import vec
from lowThrust import orbit

# radii, m (None = free); ``apse`` the in-plane inertial unit vector the
# periapsis arc is centred on (``lock_apse`` sets it).
Target = namedtuple("Target", "ra rp apse", defaults=(None,))
# ``prograde`` +1 / -1 / 0; ``eta`` this point's effectivity, ``err`` the
# error (m) of the radius this arc is driving.
Command = namedtuple("Command", "done throttle prograde eta err")


def lock_apse(target, el, coasting=True):
    """The target with its apse line set.

    Re-locked to the osculating periapsis whenever the vessel is coasting,
    held through a burn.  Within a burn the line must not move: near-
    circular, the osculating apse turns as fast as a vessel thrusting short
    of apoapsis moves, and a law chasing it rode its arc's edge for weeks.
    Between burns it must follow the real orbit: held for a whole transfer,
    the burns cannot correct the eccentricity across the line, and a 500 km
    circular target ended 628 x 396.  On a circular orbit with no line yet,
    the vessel's position.
    """
    if not coasting and target.apse is not None:
        return target
    if el.e > 1e-6:
        return target._replace(apse=vec.unit(el.evec))
    if target.apse is None:
        return target._replace(apse=vec.unit(el.rvec))
    return target


def arrived(el, target, cfg):
    """Every targeted apsis within tolerance."""
    return ((target.ra is None or abs(el.ra - target.ra) < cfg.TOLERANCE_M)
            and (target.rp is None
                 or abs(el.rp - target.rp) < cfg.TOLERANCE_M))


def command(el, target, accel, tick_s, eta_cut, cfg, mu):
    """This tick's thrust command.

    ``el`` the current elements (``orbit.elements``), ``target`` with its
    apse locked, ``accel`` the acceleration the engine gives at full
    throttle now (m/s^2, measured, never the rated figure), ``tick_s`` how
    long the command will be held, ``eta_cut`` the arc cutoff.
    """
    if arrived(el, target, cfg):
        return Command(True, 0.0, 0, 1.0, 0.0)
    apse = target.apse
    half = math.acos(max(-1.0, min(1.0, eta_cut)))
    theta = orbit.plane_angle(el.rvec, el.vvec, apse)
    d_ap = math.atan2(math.sin(theta - math.pi), math.cos(theta - math.pi))
    at_pe = abs(theta) <= abs(d_ap)          # this arc moves the apoapsis
    if at_pe:
        off, goal, far = theta, target.ra, vec.scale(apse, -1.0)
    else:
        off, goal, far = d_ap, target.rp, apse
    eta = math.cos(off)
    if abs(off) > half or goal is None:
        return Command(False, 0.0, 0, eta, 0.0)

    # Hohmann's order (an explicit flag, not ``goal is target.rp``: under
    # PyPy ``is`` compares floats by value, and a circular target deadlocked)
    tol = cfg.TOLERANCE_M
    if not at_pe and target.ra is not None:
        if el.rp < goal and el.ra < target.ra - tol:
            return Command(False, 0.0, 0, eta, 0.0)
    if at_pe and target.rp is not None:
        if el.ra > goal and el.rp > target.rp + tol:
            return Command(False, 0.0, 0, eta, 0.0)

    # How the radius toward ``far`` moves per m/s of prograde, numerically
    # (exact for the orbit, and no singularity at e = 0).
    _, T, _ = orbit.rtn(el.rvec, el.vvec)
    radius = orbit.radius_toward(el.p, el.evec, far)
    err = radius - goal
    bump = orbit.elements(el.rvec, vec.add(el.vvec, vec.scale(T, cfg.RATE_DV)),
                          mu)
    rate = (orbit.radius_toward(bump.p, bump.evec, far) - radius) / cfg.RATE_DV
    if rate == 0 or abs(err) < tol:
        return Command(False, 0.0, 0, eta, err)
    prograde = -1 if err * rate > 0 else 1
    need = abs(err) / (abs(rate) * accel)          # seconds at full thrust

    # Start half the burn's duration before the apsis, so it is centred.
    # Consistent while burning: a second of burn takes a second off the time
    # to the apsis and half a second off the half-duration.  (Spreading it
    # over the arc front-loaded long burns and overshot the far apsis.)
    if off < 0 and orbit.time_to(el, el.nu - off, mu) > need / 2:
        return Command(False, 0.0, 0, eta, err)
    return Command(False, min(1.0, need / tick_s), prograde, eta, err)
