"""Targeting for transfers out of the current body's orbit: to one of its
moons, or (through escape) to a sibling planet.  Pure functions, no kRPC.

Everything is predicted by propagating the two-body coast exactly
(``orbit.propagate``) against the target body on its rails -- the same
"decide from a fresh prediction" shape as the rest of the repo.

- ``closest_approach`` / ``miss``: where the coast passes the target, as a
  B-plane miss: the component of (vessel - target) across the relative
  velocity, against the aim point ``b`` on the side that makes the passage
  prograde about the target's orbit normal.
- ``correction``: the smallest impulse that zeroes that miss (numerical
  Jacobian, minimum-norm solve) -- mid-course corrections.
- ``push_dv``: the prograde impulse at a periapsis that puts a moon
  encounter on the aim point -- the final push of a moon transfer.
- ``window``: the heliocentric departure (Lambert over a grid of dates and
  flight times) -- the interplanetary window.
- ``parking_orientation``: the parking ellipse whose final push leaves on
  the window's departure asymptote.
- ``impact_parameter`` / capture feasibility.
"""

import math
from collections import namedtuple

from common import vec
from lowThrust import orbit

Approach = namedtuple("Approach", "t d vrel")       # time, vessel - target
Window = namedtuple("Window", "t_dep tof vinf_dep vinf_arr")


def impact_parameter(rp, vinf, mu):
    """The asymptotic miss distance that gives periapsis ``rp``."""
    return rp * math.sqrt(1 + 2 * mu / (rp * vinf * vinf))


def closest_approach(r, v, t, mu, target, horizon, samples=400, guess=None):
    """Closest approach of the coast from (r, v) at UT t to ``target`` (a
    body orbiting the same centre) within ``horizon`` seconds; with a
    ``guess`` UT, only a window round it is searched."""
    def dist(tt):
        rv, _ = orbit.propagate(r, v, mu, tt - t)
        return vec.norm(vec.sub(rv, target.position(tt)))

    if guess is not None:
        span = min(horizon, max(abs(guess - t) * 0.2, 600.0))
        lo_t, hi_t = max(t, guess - span), min(t + horizon, guess + span)
        n = 40
    else:
        lo_t, hi_t, n = t, t + horizon, samples
    step = (hi_t - lo_t) / n
    best_i, best_d = 0, math.inf
    for i in range(n + 1):
        d = dist(lo_t + i * step)
        if d < best_d:
            best_i, best_d = i, d
    a = lo_t + max(0, best_i - 1) * step
    b = lo_t + min(n, best_i + 1) * step
    g = (math.sqrt(5) - 1) / 2
    for _ in range(60):                      # golden section
        c1, c2 = b - g * (b - a), a + g * (b - a)
        if dist(c1) < dist(c2):
            b = c2
        else:
            a = c1
        if b - a < 1e-3:
            break
    tc = (a + b) / 2
    rv, vv = orbit.propagate(r, v, mu, tc - t)
    rt, vt = target.state(tc)
    return Approach(tc, vec.sub(rv, rt), vec.sub(vv, vt))


def miss(ca, b, normal):
    """B-plane miss: the across-track component of the closest-approach
    offset minus the aim point -- ``b`` on the side whose passage is
    prograde about ``normal`` (angular momentum d x v along it)."""
    u = vec.unit(ca.vrel)
    d_perp = vec.sub(ca.d, vec.scale(u, vec.dot(ca.d, u)))
    aim = vec.scale(vec.unit(vec.cross(u, normal)), b)
    return vec.sub(d_perp, aim), u


def signed_miss(ca, normal):
    """The closest-approach distance, positive on the prograde side.  The
    whole distance, not its across-track part: a minimum at the edge of the
    search window is no encounter, and its across-track part alone once read
    557 km for a pass 10400 km away."""
    u = vec.unit(ca.vrel)
    s = vec.norm(ca.d)
    return s if vec.dot(vec.cross(ca.d, u), normal) > 0 else -s


def _solve3(A, y):
    """x with A x = y for a 3x3 A (Cramer); None if singular."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(A)
    if d == 0:
        return None
    out = []
    for k in range(3):
        m = [list(row) for row in A]
        for i in range(3):
            m[i][k] = y[i]
        out.append(det(m) / d)
    return out


def correction(r, v, t, mu, target, horizon, b, normal, dv_probe=0.01,
               vinf_max=None):
    """The impulse (inertial vector, m/s) that zeroes the B-plane miss, from
    a numerical Jacobian of the miss against the three impulse components:
    minimum-norm for the two miss components; with ``vinf_max`` and an
    arrival faster than it, also the arrival speed (a square solve) -- a
    weak engine has to arrive slowly enough to capture in one pass.
    Returns (impulse, the closest approach it was computed from)."""
    ca = closest_approach(r, v, t, mu, target, horizon)
    err, u = miss(ca, b, normal)
    e1 = vec.unit(vec.cross(u, normal))
    e2 = vec.cross(u, e1)
    # the speed constraint in metres: an m/s of arrival speed weighed as the
    # distance it moves in the time left
    w = max(ca.t - t, 1.0)
    slow = vinf_max is not None and vec.norm(ca.vrel) > vinf_max
    E = [vec.dot(err, e1), vec.dot(err, e2)]
    if slow:
        E.append((vec.norm(ca.vrel) - vinf_max) * w)
    J = []                                    # columns per impulse axis
    for k in range(3):
        dv = [0.0, 0.0, 0.0]
        dv[k] = dv_probe
        ca_k = closest_approach(r, vec.add(v, tuple(dv)), t, mu, target,
                                horizon, guess=ca.t)
        err_k, _ = miss(ca_k, b, normal)
        col = [(vec.dot(err_k, e1) - E[0]) / dv_probe,
               (vec.dot(err_k, e2) - E[1]) / dv_probe]
        if slow:
            col.append(((vec.norm(ca_k.vrel) - vinf_max) * w - E[2])
                       / dv_probe)
        J.append(col)
    Jr = [[J[k][i] for k in range(3)] for i in range(len(E))]
    if slow:
        x = _solve3(Jr, [-e for e in E])
        return ((0.0, 0.0, 0.0) if x is None else tuple(x)), ca
    a11 = sum(x * x for x in Jr[0])
    a12 = sum(x * y for x, y in zip(Jr[0], Jr[1]))
    a22 = sum(x * x for x in Jr[1])
    det = a11 * a22 - a12 * a12
    if det == 0:
        return (0.0, 0.0, 0.0), ca
    # dv = -J^T (J J^T)^-1 E
    l1 = -(a22 * E[0] - a12 * E[1]) / det
    l2 = -(-a12 * E[0] + a11 * E[1]) / det
    dv = tuple(Jr[0][k] * l1 + Jr[1][k] * l2 for k in range(3))
    return dv, ca


def _aimed(r, t, mu, target, ta, normal, rc):
    """Lambert to the B-plane aim point at UT ta: the target's centre plus
    the impact parameter for periapsis ``rc`` on the prograde side (two
    passes: the offset depends on the arrival velocity).  Aimed at the
    centre, the arc and the B-plane correction disagreed, and a Moho cruise
    re-targeted itself sixty times."""
    rt, vt = target.state(ta)
    sol = orbit.lambert(r, rt, ta - t, mu, normal)
    if sol is None or rc is None:
        return sol, vt
    for _ in range(2):
        vrel = vec.sub(sol[1], vt)
        u = vec.unit(vrel)
        b = impact_parameter(rc, max(vec.norm(vrel), 1.0), target.mu)
        aim = vec.add(rt, vec.scale(vec.unit(vec.cross(u, target.normal)), b))
        s2 = orbit.lambert(r, aim, ta - t, mu, normal)
        if s2 is None:
            break
        sol = s2
    return sol, vt


def retarget(r, v, t, mu, target, t_lo, t_hi, normal, n=24, rc=None):
    """A new transfer from (r, v, t) to ``target``'s centre: Lambert over
    arrival times in [t_lo, t_hi], the cheapest in impulse plus arrival
    speed.  Returns (impulse vector, arrival UT), or None.  For a miss far
    too big for a linear correction (a short departure left a Moho transfer
    4 million km wide, and the correction asked for 5-17 km/s a tick)."""
    best = None
    for i in range(n + 1):
        ta = t_lo + (t_hi - t_lo) * i / n
        if ta <= t + 60:
            continue
        rt, vt = target.state(ta)
        # not near a 180 deg transfer: the arc's plane is ill-conditioned
        # there, and a finite burn onto one landed a Moho transfer farther
        # off than it started
        ang = math.acos(max(-1.0, min(1.0, vec.dot(r, rt)
                                      / (vec.norm(r) * vec.norm(rt)))))
        if abs(ang - math.pi) < math.radians(15):
            continue
        sol, vt = _aimed(r, t, mu, target, ta, normal, rc)
        if sol is None:
            continue
        dv = vec.sub(sol[0], v)
        cost = vec.norm(dv) + vec.norm(vec.sub(sol[1], vt))
        if best is None or cost < best[0]:
            best = (cost, dv, ta)
    return None if best is None else (best[1], best[2])


def lambert_dv(r, v, t, mu, target, ta, normal, rc=None):
    """The impulse onto the Lambert arc to ``target``'s aim point at UT ta."""
    sol, _ = _aimed(r, t, mu, target, ta, normal, rc)
    return None if sol is None else vec.sub(sol[0], v)


def capturable_vinf(body, accel, rc_min, ra_max, arc, fill, ra_loose=None):
    """The fastest arrival an engine of ``accel`` can capture in one pass
    (0 if none)."""
    if capture_orbit(body, 1.0, accel, rc_min, ra_max, arc, fill,
                     ra_loose) is None:
        return 0.0
    lo, hi = 1.0, 5000.0
    if capture_orbit(body, hi, accel, rc_min, ra_max, arc, fill,
                     ra_loose) is not None:
        return hi
    for _ in range(30):
        mid = (lo + hi) / 2
        if capture_orbit(body, mid, accel, rc_min, ra_max, arc, fill,
                         ra_loose) is not None:
            lo = mid
        else:
            hi = mid
    return lo


def push_dv(r, v, t, mu, target, b, normal, dv_max, horizon_fn, n=40,
            dv_min=0.0):
    """The smallest prograde impulse in (dv_min, dv_max] at (r, v, t) whose
    coast passes ``target`` on the prograde side at ``b``; None if there is
    none.  ``horizon_fn(r, v)`` gives how far ahead to search after the
    impulse."""
    u = vec.unit(v)

    def f(dv):
        v1 = vec.add(v, vec.scale(u, dv))
        h = horizon_fn(r, v1)
        ca = closest_approach(r, v1, t, mu, target, h, 200)
        if ca.t > t + 0.999 * h:             # no encounter inside the window
            return abs(signed_miss(ca, normal)) + b, ca
        return signed_miss(ca, normal) - b, ca

    prev_dv, (prev_f, _) = dv_min, f(dv_min)
    for i in range(1, n + 1):
        dv = dv_min + (dv_max - dv_min) * i / n
        fv, ca = f(dv)
        # a genuine crossing passes through the aim, not across the far side
        if (prev_f < 0) != (fv < 0) and abs(fv) < 20 * b + abs(prev_f):
            lo, hi, flo = prev_dv, dv, prev_f
            for _ in range(40):
                mid = (lo + hi) / 2
                fm, _ = f(mid)
                if (fm < 0) == (flo < 0):
                    lo, flo = mid, fm
                else:
                    hi = mid
            fm, ca = f((lo + hi) / 2)
            if abs(fm) < max(0.2 * b, 1e3):
                return (lo + hi) / 2
        prev_dv, prev_f = dv, fv
    return None


def first_hazard(r, v, t, mu, others, t_end, margin=1.2):
    """The first of ``others`` (bodies about the same centre) whose SOI the
    coast from (r, v, t) enters before UT ``t_end``; None if none."""
    hit = None
    for b in others:
        if t_end <= t:
            break
        ca = closest_approach(r, v, t, mu, b, t_end - t, 300)
        if vec.norm(ca.d) < margin * b.soi and (hit is None or ca.t < hit[0]):
            hit = (ca.t, b)
    return hit[1] if hit else None


def window(origin, dest, t_from, mu_sun, span, grid=(90, 30),
           arrival_weight=1.0):
    """The cheapest Lambert departure from ``origin`` to ``dest`` (siblings
    about the same star) leaving within ``span`` seconds of ``t_from``:
    least |v_inf out| + weight |v_inf in|, over flight times 0.5-1.5 of the
    Hohmann's."""
    a_t = (origin.sma + dest.sma) / 2
    t_h = math.pi * math.sqrt(a_t ** 3 / mu_sun)

    def cost(td, tof):
        r1, v1p = origin.state(td)
        r2, v2p = dest.state(td + tof)
        # prograde about the origin's orbit normal
        sol = orbit.lambert(r1, r2, tof, mu_sun, origin.normal)
        if sol is None:
            return math.inf, None
        vd = vec.sub(sol[0], v1p)
        va = vec.sub(sol[1], v2p)
        return vec.norm(vd) + arrival_weight * vec.norm(va), (vd, va)

    best = (math.inf, None, None, None)
    nd, nt = grid
    for i in range(nd + 1):
        td = t_from + span * i / nd
        for j in range(nt + 1):
            tof = t_h * (0.5 + j / nt)
            c, vv = cost(td, tof)
            if c < best[0]:
                best = (c, td, tof, vv)
    # refine round the best cell
    _, td0, tof0, _ = best
    dtd, dtof = span / nd, t_h / nt
    for _ in range(3):
        for i in range(-5, 6):
            for j in range(-5, 6):
                td = td0 + dtd * i / 5
                tof = tof0 + dtof * j / 5
                if td < t_from or tof <= 0:
                    continue
                c, vv = cost(td, tof)
                if c < best[0]:
                    best = (c, td, tof, vv)
        _, td0, tof0, _ = best
        dtd, dtof = dtd / 5, dtof / 5
    c, td, tof, vv = best
    if vv is None:
        return None
    return Window(td, tof, vv[0], vv[1])


def departure_hyperbola(mu, rp, vinf):
    """(eccentricity, true anomaly of the outgoing asymptote) of the escape
    hyperbola with periapsis ``rp`` and excess speed ``vinf``."""
    e = 1 + rp * vinf * vinf / mu
    return e, math.acos(-1 / e)


def parking_orientation(mu, rp, ra, vinf_vec, normal_now, steps=360):
    """The parking ellipse (rp, ra) whose periapsis push leaves on
    ``vinf_vec``: its plane must contain the asymptote, and of that family
    of planes the one whose plane change from ``normal_now`` -- made on the
    ellipse, at its higher node -- costs least (prograde planes only).
    Returns (normal, periapsis direction, the plane change's dv)."""
    vinf = vec.norm(vinf_vec)
    u = vec.unit(vinf_vec)
    _, nu_inf = departure_hyperbola(mu, rp, vinf)
    ref = (0.0, 0.0, 1.0) if abs(u[2]) < 0.9 else (1.0, 0.0, 0.0)
    a_ = vec.unit(vec.cross(u, ref))
    b_ = vec.cross(u, a_)
    e = (ra - rp) / (ra + rp)
    p = rp * (1 + e)
    h = math.sqrt(mu * p)
    best = None
    for k in range(steps):
        psi = 2 * math.pi * k / steps
        n = vec.add(vec.scale(a_, math.cos(psi)), vec.scale(b_, math.sin(psi)))
        if vec.dot(n, normal_now) <= 0:
            continue
        P = vec.sub(vec.scale(u, math.cos(nu_inf)),
                    vec.scale(vec.cross(n, u), math.sin(nu_inf)))
        delta = math.acos(max(-1.0, min(1.0, vec.dot(n, normal_now))))
        if delta < 1e-9:
            cost = 0.0
        else:
            nodes = vec.unit(vec.cross(normal_now, n))
            evec = vec.scale(P, e)
            r_node = max(orbit.radius_toward(p, evec, nodes),
                         orbit.radius_toward(p, evec, vec.scale(nodes, -1.0)))
            cost = 2 * h / r_node * math.sin(delta / 2)
        if best is None or cost < best[2]:
            best = (n, vec.unit(P), cost)
    return best


def capture_dv(mu, rp, ra, vinf):
    """Impulse at periapsis ``rp`` from a hyperbola of excess ``vinf`` to
    the ellipse (rp, ra)."""
    v_hyp = math.sqrt(vinf * vinf + 2 * mu / rp)
    v_ell = math.sqrt(mu * (2 / rp - 2 / (rp + ra)))
    return v_hyp - v_ell


def pass_time(mu, rp, vinf, arc):
    """Seconds a hyperbola of periapsis ``rp`` and excess ``vinf`` spends
    within +-``arc`` rad of periapsis."""
    e = 1 + rp * vinf * vinf / mu
    a = -mu / (vinf * vinf)
    arc = min(arc, 0.95 * math.acos(-1 / e))
    F = 2 * math.atanh(math.sqrt((e - 1) / (e + 1)) * math.tan(arc / 2))
    M = e * math.sinh(F) - F
    return 2 * M / math.sqrt(mu / (-a) ** 3)


def moon_arrival_vinf(mu, rp, r_moon, v_moon):
    """Excess speed at a moon of circular-orbit speed ``v_moon`` and radius
    ``r_moon``, arriving at apoapsis of the ellipse (rp, r_moon)."""
    v_ap = math.sqrt(mu * 2 * rp / (r_moon * (r_moon + rp)))
    return abs(v_moon - v_ap)


def capture_orbit(body, vinf, accel, rc_min, ra_max, arc, fill,
                  ra_loose=None):
    """The lowest capture periapsis (>= rc_min) at which an engine of
    ``accel`` can shed ``vinf`` into the loosest ellipse (apoapsis ra_max)
    in one pass: burn time at most ``fill`` of the time within +-``arc`` of
    periapsis.  A weak engine captures high and loose and spirals down
    after -- at 0.04 m/s^2 even a v_inf of 0 cannot be captured at 50 km
    over the Mun.  When ``ra_loose`` is given and ``ra_max`` admits no
    capture, the looser ellipse is tried.  Returns (rc, ra), or None."""
    vinf = max(vinf, 1.0)
    for ra in (ra_max,) if ra_loose is None else (ra_max, ra_loose):
        rc = rc_min
        while rc < 0.5 * ra:
            need = capture_dv(body.mu, rc, ra, vinf) / accel
            if need <= fill * pass_time(body.mu, rc, vinf, arc):
                return rc, ra
            rc *= 1.1
    return None


def clear_of(r, others, margin):
    """Whether radius ``r`` is outside every other moon's band of orbits
    (its periapsis to apoapsis, widened by ``margin`` SOIs)."""
    for b in others:
        lo = b.sma * (1 - b.ecc) - margin * b.soi
        hi = b.sma * (1 + b.ecc) + margin * b.soi
        if lo < r < hi:
            return False
    return True


def wait_periapsis(mu, moon, rp_min, rc, ra_cap, accel, arc, fill):
    """The lowest waiting-orbit periapsis (>= rp_min) from which a moon
    arrival can be captured in one periapsis pass: capture burn time at most
    ``fill`` of the time within +-``arc`` of the capture periapsis.  A weak
    engine raises it, which slows the arrival relative to the moon."""
    r_m = moon.sma
    v_m = math.sqrt(mu / r_m)
    rp = rp_min
    top = 0.9 * (r_m - moon.soi)
    while rp < top:
        vinf = moon_arrival_vinf(mu, rp, r_m, v_m)
        need = capture_dv(moon.mu, rc, ra_cap, max(vinf, 1.0)) / accel
        if need <= fill * pass_time(moon.mu, rc, max(vinf, 1.0), arc):
            return rp
        rp += 0.02 * (top - rp_min) + 1e3
    return None
