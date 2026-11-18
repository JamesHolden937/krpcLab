"""The recon pilot's laws, as pure functions (tests/testGuidance.py).

Angles in radians, distances in metres, speeds in m/s.  Headings are
clockwise from north.  The body is a sphere of radius ``R`` (Kerbin).
"""

import math

G0 = 9.80665


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap(a):
    """An angle into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# -- the sphere -------------------------------------------------------------
def distance(lat1, lon1, lat2, lon2, radius):
    """Great-circle distance (haversine), metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2):
    """Initial great-circle course from 1 to 2, radians from north."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.atan2(y, x) % (2.0 * math.pi)


def offset(lat, lon, course, dist, radius):
    """The point ``dist`` along ``course`` from (lat, lon): (lat, lon) deg."""
    p1, l1 = math.radians(lat), math.radians(lon)
    d = dist / radius
    p2 = math.asin(math.sin(p1) * math.cos(d)
                   + math.cos(p1) * math.sin(d) * math.cos(course))
    l2 = l1 + math.atan2(math.sin(course) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), (math.degrees(l2) + 540.0) % 360.0 - 180.0


def along_cross(lat, lon, lat0, lon0, course, radius):
    """Where (lat, lon) is from the line through (lat0, lon0) on
    ``course``: (along, cross) metres, cross positive to the right."""
    d = distance(lat0, lon0, lat, lon, radius)
    if d < 1e-6:
        return 0.0, 0.0
    b = bearing(lat0, lon0, lat, lon)
    rel = wrap(b - course)
    return d * math.cos(rel), d * math.sin(rel)


# -- load ---------------------------------------------------------------------
def normal_g_budget(g_limit, axial_g):
    """How much load the wings may pull (g) with ``axial_g`` along the
    flight path already on the pilot: |total| <= g_limit."""
    return math.sqrt(max(0.0, g_limit * g_limit - axial_g * axial_g))


def bank_for_turn(turn_rate, speed, normal_g):
    """The bank that turns at ``turn_rate`` (rad/s) at ``speed`` in a
    level coordinated turn, no steeper than ``normal_g`` of load allows
    (tan(phi) <= sqrt(n^2 - 1))."""
    want = math.atan(speed * turn_rate / G0)
    cap = math.atan(math.sqrt(max(0.0, normal_g * normal_g - 1.0)))
    return clamp(want, -cap, cap)


def turn_radius(speed, normal_g):
    """A level turn's radius at ``normal_g`` of load."""
    lat = math.sqrt(max(1e-6, normal_g * normal_g - 1.0))
    return speed * speed / (G0 * lat)


# -- heat ---------------------------------------------------------------------
def heat_throttle(frac, rate, frac_max, lookahead, band):
    """The throttle that keeps the hottest part under ``frac_max`` of its
    limit: proportional on where it will be ``lookahead`` s from now,
    full at ``band`` below the ceiling, shut at it."""
    ahead = frac + max(0.0, rate) * lookahead
    return clamp((frac_max - ahead) / band, 0.0, 1.0)


# -- energy on the way home ----------------------------------------------------
def coast_speed(v_end, dist, drag_k):
    """The speed from which coasting ``dist`` (drag only, dV/dx = -k V,
    k = rho CdA / 2m) slows to ``v_end``."""
    return v_end * math.exp(max(0.0, drag_k) * max(0.0, dist))


def glide_altitude(dist, h_end, slope, h_max):
    """The descent's altitude ``dist`` from its end."""
    return min(h_max, h_end + max(0.0, dist) * slope)


def flight_path_command(h_cmd, h, speed, tau, gamma_max, gamma_min):
    """Flight-path angle closing the altitude error in ``tau`` seconds."""
    vs = (h_cmd - h) / tau
    s = clamp(vs / max(speed, 1.0), -1.0, 1.0)
    return clamp(math.asin(s), gamma_min, gamma_max)


def localizer_course(rwy_course, cross, speed, tau, max_cut):
    """The course that brings ``cross`` (m, right of the centreline) to
    zero in about ``tau`` s, cut at most ``max_cut``."""
    lead = max(speed * tau, 1.0)
    return rwy_course - clamp(math.atan2(cross, lead), -max_cut, max_cut)



def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return tuple(x / n for x in a) if n > 1e-9 else None


def path_attitude(vel, a_up, a_right, alpha):
    """The nose and roof for flying ``vel`` (surface frame: x up, y north,
    z east) with a normal acceleration of ``a_up`` (in the vertical plane
    through the velocity, upward positive) and ``a_right`` (horizontal, to
    the right), at angle of attack ``alpha``: the lift points along that
    acceleration, the nose ``alpha`` above the velocity toward it, the
    roof along the lift.  Returns (nose, roof, bank) or None for no
    velocity."""
    v = _unit(vel)
    if v is None:
        return None
    up = (1.0, 0.0, 0.0)
    right = _unit(_cross(up, v)) or (0.0, 0.0, 1.0)
    n_up = _cross(v, right)
    lift = _unit(tuple(a_up * u + a_right * r for u, r in zip(n_up, right)))
    if lift is None:
        lift = n_up
    nose = tuple(math.cos(alpha) * x + math.sin(alpha) * l
                 for x, l in zip(v, lift))
    roof = tuple(math.cos(alpha) * l - math.sin(alpha) * x
                 for x, l in zip(v, lift))
    bank = math.atan2(a_right, a_up)
    return nose, roof, bank


def normal_accel(speed, gamma, gamma_cmd, heading_err, tau_path, tau_heading,
                 a_max, bank_cap):
    """The acceleration square to the path that flies it: up, to close the
    path angle in ``tau_path`` against gravity's share; right, the turn
    closing the heading in ``tau_heading``.  The total held to ``a_max``,
    the vertical first; the turn's share to ``bank_cap`` of bank."""
    g = G0
    a_up = speed * (gamma_cmd - gamma) / tau_path + g * math.cos(gamma)
    a_up = clamp(a_up, -a_max, a_max)
    a_right = speed * heading_err / tau_heading
    room = math.sqrt(max(0.0, a_max * a_max - a_up * a_up))
    cap = abs(a_up) * math.tan(bank_cap) if a_up > 0 else room
    a_right = clamp(a_right, -min(room, cap), min(room, cap))
    return a_up, a_right
