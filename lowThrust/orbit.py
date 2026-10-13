"""Two-body orbit maths: elements from a state, exact coasting, radii toward
fixed directions, the impulsive reference.  Pure functions, no kRPC.

Vectors are plain 3-tuples (``common.vec``) in an inertial frame centred on
the body.  ``elements`` is for bound orbits (None otherwise); ``conic``,
``propagate``, ``time_to_periapsis`` and ``lambert`` take any conic.
"""

import math
from collections import namedtuple

from common import vec

Elements = namedtuple("Elements", "a e p h ra rp nu r period rvec vvec evec")
# Any conic: a < 0 and ra = period = inf for a hyperbola; nu in -pi..pi
# (negative inbound); vinf the hyperbolic excess speed (0 if bound).
Conic = namedtuple("Conic", "a e p h rp ra nu r period energy vinf rvec vvec "
                            "evec")


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


def conic(r, v, mu):
    """The osculating conic of state (r, v), bound or not."""
    rmag = vec.norm(r)
    hvec = vec.cross(r, v)
    h = vec.norm(hvec)
    energy = vec.dot(v, v) / 2 - mu / rmag
    evec = vec.sub(vec.scale(vec.cross(v, hvec), 1 / mu), vec.scale(r, 1 / rmag))
    e = vec.norm(evec)
    p = h * h / mu
    if e > 1e-10:
        nu = math.acos(max(-1.0, min(1.0, vec.dot(evec, r) / (e * rmag))))
        if vec.dot(r, v) < 0:
            nu = -nu
    else:
        nu = 0.0
    bound = energy < 0
    a = -mu / (2 * energy) if energy != 0 else math.inf
    return Conic(a, e, p, h, p / (1 + e), a * (1 + e) if bound else math.inf,
                 nu, rmag,
                 2 * math.pi * math.sqrt(a ** 3 / mu) if bound else math.inf,
                 energy, 0.0 if bound else math.sqrt(2 * energy), r, v, evec)


def time_to_periapsis(c, mu):
    """Seconds from now to the next periapsis (a hyperbola outbound: the one
    just passed, negative)."""
    if c.e < 1:
        el = Elements(c.a, c.e, c.p, c.h, c.ra, c.rp, c.nu % (2 * math.pi),
                      c.r, c.period, c.rvec, c.vvec, c.evec)
        return time_to(el, 0.0, mu)
    F = 2 * math.atanh(math.sqrt((c.e - 1) / (c.e + 1)) * math.tan(c.nu / 2))
    M = c.e * math.sinh(F) - F
    return -M / math.sqrt(mu / (-c.a) ** 3)


def _stumpff(z):
    if z > 1e-8:
        sz = math.sqrt(z)
        return (1 - math.cos(sz)) / z, (sz - math.sin(sz)) / sz ** 3
    if z < -1e-8:
        sz = math.sqrt(-z)
        return (math.cosh(sz) - 1) / -z, (math.sinh(sz) - sz) / sz ** 3
    return 0.5 - z / 24, 1 / 6 - z / 120


def propagate(r, v, mu, dt):
    """Propagate any conic ``dt`` seconds exactly (universal variables)."""
    if dt == 0:
        return r, v
    r0 = vec.norm(r)
    vr0 = vec.dot(r, v) / r0
    alpha = 2 / r0 - vec.dot(v, v) / mu
    sqmu = math.sqrt(mu)
    def residual(x):
        try:
            z = alpha * x * x
            C, S = _stumpff(z)
            return abs(r0 * vr0 / sqmu * x * x * C
                       + (1 - alpha * r0) * x ** 3 * S + r0 * x - sqmu * dt)
        except OverflowError:
            return math.inf

    x = sqmu * dt / r0
    if alpha > 1e-12:
        x = sqmu * alpha * dt
    elif alpha < -1e-12:
        # The long-arc guess, unless the short-arc one is closer: from a
        # hyperbola 72 Mm out, a 63 s coast from the long-arc guess ran
        # Newton off along cosh to 1e17 m.
        a = 1 / alpha
        sgn = 1 if dt > 0 else -1
        arg = ((-2 * mu * alpha * dt)
               / (vec.dot(r, v) + sgn * math.sqrt(-mu * a) * (1 - r0 * alpha)))
        if arg > 0:
            x_long = sgn * math.sqrt(-a) * math.log(arg)
            if residual(x_long) < residual(x):
                x = x_long
    for _ in range(200):
        z = alpha * x * x
        C, S = _stumpff(z)
        F = (r0 * vr0 / sqmu * x * x * C + (1 - alpha * r0) * x ** 3 * S
             + r0 * x - sqmu * dt)
        dF = (r0 * vr0 / sqmu * x * (1 - z * S) + (1 - alpha * r0) * x * x * C
              + r0)
        step = F / dF
        x -= step
        if abs(step) < 1e-9 * max(1.0, abs(x)):
            break
    z = alpha * x * x
    C, S = _stumpff(z)
    f = 1 - x * x / r0 * C
    g = dt - x ** 3 * S / sqmu
    r1 = vec.add(vec.scale(r, f), vec.scale(v, g))
    rm = vec.norm(r1)
    fd = sqmu / (rm * r0) * (alpha * x ** 3 * S - x)
    gd = 1 - x * x / rm * C
    return r1, vec.add(vec.scale(r, fd), vec.scale(v, gd))


def lambert(r1, r2, dt, mu, normal=(0.0, 0.0, 1.0)):
    """Velocities (v1, v2) of the single-revolution conic from r1 to r2 in
    ``dt`` seconds, moving prograde about ``normal``; None if there is none.
    Universal variables, bisection on z (Curtis, algorithm 5.2).  Singular at
    a transfer angle of exactly 180 deg."""
    m1, m2 = vec.norm(r1), vec.norm(r2)
    cos_t = max(-1.0, min(1.0, vec.dot(r1, r2) / (m1 * m2)))
    theta = math.acos(cos_t)
    if vec.dot(vec.cross(r1, r2), normal) < 0:
        theta = 2 * math.pi - theta
    if abs(1 - math.cos(theta)) < 1e-12:
        return None
    A = math.sin(theta) * math.sqrt(m1 * m2 / (1 - math.cos(theta)))
    if abs(A) < 1e-9:
        return None
    sqmu = math.sqrt(mu)

    def y(z):
        C, S = _stumpff(z)
        return m1 + m2 + A * (z * S - 1) / math.sqrt(C)

    def F(z):
        C, S = _stumpff(z)
        yz = y(z)
        return (yz / C) ** 1.5 * S + A * math.sqrt(yz) - sqmu * dt

    # F rises with z.  Upper end: just under (2 pi)^2, where F -> +inf.
    # Lower end: where y reaches 0 (F there is -sqrt(mu) dt < 0), or far
    # enough down for a fast hyperbolic transfer.
    hi = 4 * math.pi ** 2 - 1e-6
    if F(hi) < 0:
        return None                       # slower than one revolution allows

    def y_root(a_, b_):                   # y(a_) < 0 <= y(b_)
        for _ in range(200):
            mid = (a_ + b_) / 2
            if y(mid) < 0:
                a_ = mid
            else:
                b_ = mid
        return b_
    lo = -4 * math.pi ** 2
    while True:
        if lo < -4e5:                     # cosh overflows past z ~ -5e5
            return None
        if y(lo) < 0:
            lo = y_root(lo, hi)
            break
        if F(lo) < 0:
            break
        lo *= 2
    for _ in range(200):
        mid = (lo + hi) / 2
        if F(mid) > 0:
            hi = mid
        else:
            lo = mid
        if hi - lo < 1e-12:
            break
    yz = y((lo + hi) / 2)
    f = 1 - yz / m1
    g = A * math.sqrt(yz / mu)
    gd = 1 - yz / m2
    v1 = vec.scale(vec.sub(r2, vec.scale(r1, f)), 1 / g)
    v2 = vec.scale(vec.sub(vec.scale(r2, gd), r1), 1 / g)
    return v1, v2


def state_from_elements(mu, a, e, inc, lan, argpe, nu):
    """(r, v) on the conic with these elements (rad) at true anomaly nu."""
    p = a * (1 - e * e)
    rr = p / (1 + e * math.cos(nu))
    pos = (rr * math.cos(nu), rr * math.sin(nu), 0.0)
    k = math.sqrt(mu / p)
    vel = (-k * math.sin(nu), k * (e + math.cos(nu)), 0.0)

    def rot(x):          # Rz(lan) Rx(inc) Rz(argpe)
        c, s_ = math.cos(argpe), math.sin(argpe)
        x = (c * x[0] - s_ * x[1], s_ * x[0] + c * x[1], x[2])
        c, s_ = math.cos(inc), math.sin(inc)
        x = (x[0], c * x[1] - s_ * x[2], s_ * x[1] + c * x[2])
        c, s_ = math.cos(lan), math.sin(lan)
        return (c * x[0] - s_ * x[1], s_ * x[0] + c * x[1], x[2])
    return rot(pos), rot(vel)


def true_from_mean(M, e):
    """True anomaly for mean anomaly M on an ellipse."""
    E = M if e < 0.8 else math.pi
    for _ in range(50):
        d = (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
        E -= d
        if abs(d) < 1e-14:
            break
    return 2 * math.atan2(math.sqrt(1 + e) * math.sin(E / 2),
                          math.sqrt(1 - e) * math.cos(E / 2))


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


def plane_normal(inc, lan):
    """Unit orbit normal for inclination and longitude of the ascending node
    (rad), in the body's frame (equator = x-y, node longitude from +x)."""
    return (math.sin(inc) * math.sin(lan), -math.sin(inc) * math.cos(lan),
            math.cos(inc))


def inclination_lan(r, v):
    """(inclination, lan) of the orbit through (r, v), rad; lan is 0 for an
    equatorial orbit."""
    n = vec.unit(vec.cross(r, v))
    inc = math.acos(max(-1.0, min(1.0, n[2])))
    lan = math.atan2(n[0], -n[1]) if math.hypot(n[0], n[1]) > 1e-12 else 0.0
    return inc, lan % (2 * math.pi)


def plane_change_dv(el, normal_from, normal_to):
    """Impulsive cost of rotating the orbit through (el) from one plane to
    the other at the higher of its two nodes: only the horizontal velocity
    turns, 2 (h / r_node) sin(delta / 2)."""
    delta = math.acos(max(-1.0, min(1.0, vec.dot(normal_from, normal_to))))
    if delta == 0:
        return 0.0
    nodes = vec.unit(vec.cross(normal_from, normal_to))
    r_node = max(radius_toward(el.p, el.evec, nodes),
                 radius_toward(el.p, el.evec, vec.scale(nodes, -1.0)))
    return 2 * el.h / r_node * math.sin(delta / 2)


def impulsive_dv(mu, ra0, rp0, ra1, rp1, di=0.0):
    """Two-burn impulsive reference for an apsides change (Hohmann-like),
    with a plane change of ``di`` (rad) folded into the burn at the larger
    radius (where it is cheapest).

    One burn at an apsis of the start orbit sets the opposite apsis, a second
    at that new apsis sets the remaining one.  The cheaper of the two orders.
    It assumes the nodes can be put at that apsis, so it is a lower bound
    when they are not.  The yardstick the low-thrust dv is compared against.
    """
    def burn(va, vb, plane):
        if not plane:
            return abs(vb - va)
        return math.sqrt(max(0.0, va * va + vb * vb
                             - 2 * va * vb * math.cos(di)))

    def order(first_at, tgt_other, tgt_first):
        # burn at radius first_at (an apsis of the start orbit), moving the
        # opposite apsis to tgt_other; then there, move first_at to tgt_first.
        plane_first = first_at >= tgt_other
        dv1 = burn(speed_at(mu, first_at, ra0, rp0),
                   speed_at(mu, first_at, first_at, tgt_other), plane_first)
        dv2 = burn(speed_at(mu, tgt_other, tgt_other, first_at),
                   speed_at(mu, tgt_other, tgt_other, tgt_first),
                   not plane_first)
        return dv1 + dv2
    return min(order(rp0, ra1, rp1), order(ra0, rp1, ra1))
