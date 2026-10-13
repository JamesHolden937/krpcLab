"""Two-body orbit maths: elements from a state, exact coasting, radii toward
fixed directions, the impulsive reference.  Pure functions, no kRPC.

Vectors are plain 3-tuples (``common.vec``) in an inertial frame centred on
the body.  Everything here is for bound (elliptic) orbits; escape is a later
stage.
"""

import math
from collections import namedtuple

from common import vec

Elements = namedtuple("Elements", "a e p h ra rp nu r period rvec vvec evec")


def elements(r, v, mu):
    """The osculating elements of state ``(r, v)``, or None if not bound."""
    rmag = vec.norm(r)
    hvec = vec.cross(r, v)
    h = vec.norm(hvec)
    energy = vec.dot(v, v) / 2 - mu / rmag
    if energy >= 0 or h == 0:
        return None
    a = -mu / (2 * energy)
    # e vector = (v x h)/mu - r/|r|
    evec = vec.sub(vec.scale(vec.cross(v, hvec), 1 / mu), vec.scale(r, 1 / rmag))
    e = vec.norm(evec)
    p = h * h / mu
    if e > 1e-10:
        cosnu = vec.dot(evec, r) / (e * rmag)
        nu = math.acos(max(-1.0, min(1.0, cosnu)))
        if vec.dot(r, v) < 0:
            nu = 2 * math.pi - nu
    else:
        nu = 0.0      # circular: every point is a periapsis
    return Elements(a, e, p, h, a * (1 + e), a * (1 - e), nu, rmag,
                    2 * math.pi * math.sqrt(a ** 3 / mu), r, v, evec)


def rtn(r, v):
    """Unit radial, transverse (along-track, in plane) and normal vectors."""
    R = vec.unit(r)
    N = vec.unit(vec.cross(r, v))
    return R, vec.cross(N, R), N


def radius_toward(p, evec, direction):
    """The orbit's radius in inertial ``direction`` (a unit vector in the
    plane): p / (1 + e.u).  Smooth through circular, unlike the apsides."""
    return p / (1 + vec.dot(evec, direction))


def plane_angle(r, v, ref):
    """Angle (rad, -pi..pi) from in-plane unit ``ref`` to position ``r``,
    positive in the direction of motion."""
    N = vec.unit(vec.cross(r, v))
    u = vec.unit(r)
    return math.atan2(vec.dot(vec.cross(ref, u), N), vec.dot(ref, u))


def time_to(el, nu_to, mu):
    """Seconds from the current true anomaly forward to ``nu_to``."""
    def mean_anomaly(nu):
        E = 2 * math.atan2(math.sqrt(1 - el.e) * math.sin(nu / 2),
                           math.sqrt(1 + el.e) * math.cos(nu / 2))
        return E - el.e * math.sin(E)
    dM = (mean_anomaly(nu_to) - mean_anomaly(el.nu)) % (2 * math.pi)
    return dM / (2 * math.pi) * el.period


def coast(r, v, mu, dt):
    """Propagate an elliptic orbit ``dt`` seconds exactly (f and g series)."""
    r0 = vec.norm(r)
    a = 1 / (2 / r0 - vec.dot(v, v) / mu)
    sqmu, sqa = math.sqrt(mu), math.sqrt(a)
    sigma0 = vec.dot(r, v) / sqmu
    M = math.sqrt(mu / a ** 3) * dt
    # Kepler's equation in the eccentric-anomaly change dE:
    #   M = dE - (1 - r0/a) sin dE + sigma0/sqrt(a) (1 - cos dE)
    k1, k2 = 1 - r0 / a, sigma0 / sqa
    dE = M
    for _ in range(50):
        s, c = math.sin(dE), math.cos(dE)
        f = dE - k1 * s + k2 * (1 - c) - M
        step = f / (1 - k1 * c + k2 * s)
        dE -= step
        if abs(step) < 1e-13:
            break
    s, c = math.sin(dE), math.cos(dE)
    rr = a + (r0 - a) * c + sigma0 * sqa * s
    f = 1 - a / r0 * (1 - c)
    g = a * sigma0 / sqmu * (1 - c) + r0 * math.sqrt(a / mu) * s
    fd = -math.sqrt(mu * a) / (rr * r0) * s
    gd = 1 - a / rr * (1 - c)
    return (vec.add(vec.scale(r, f), vec.scale(v, g)),
            vec.add(vec.scale(r, fd), vec.scale(v, gd)))


def speed_at(mu, r, ra, rp):
    """Vis-viva: the speed at radius ``r`` on the orbit with these apsides."""
    return math.sqrt(mu * (2 / r - 2 / (ra + rp)))


def impulsive_dv(mu, ra0, rp0, ra1, rp1):
    """Two-burn impulsive reference for an apsides change (Hohmann-like).

    One burn at an apsis of the start orbit sets the opposite apsis, a second
    at that new apsis sets the remaining one.  The cheaper of the two orders.
    It is the yardstick the low-thrust transfer's dv is compared against.
    """
    def order(first_at, tgt_other, tgt_first):
        # burn at radius first_at (an apsis of the start orbit), moving the
        # opposite apsis to tgt_other; then there, move first_at to tgt_first.
        dv1 = abs(speed_at(mu, first_at, first_at, tgt_other)
                  - speed_at(mu, first_at, ra0, rp0))
        dv2 = abs(speed_at(mu, tgt_other, tgt_other, tgt_first)
                  - speed_at(mu, tgt_other, tgt_other, first_at))
        return dv1 + dv2
    return min(order(rp0, ra1, rp1), order(ra0, rp1, ra1))
