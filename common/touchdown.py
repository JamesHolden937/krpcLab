"""How hard a landing a craft can take: its lowest parts' impact tolerance.

KSP destroys a part that strikes something faster than its ``crashTolerance``
(kRPC ``Part.impact_tolerance``, m/s).  On a landing the parts that strike
are the lowest ones along the landing axis, so the craft's limit is the
smallest tolerance among the parts within ``band`` metres of its lowest
point.  It ignores leg suspension (legs survive somewhat more), tip-over and
joint loads; it is the speed above which the first contact breaks a part.
"""

import math


def touchdown_limit(vessel, axis=(0.0, 1.0, 0.0), band=1.0):
    """``(speed, part title, n parts that touch)`` or None.

    The legs if the craft has any, else the parts within ``band`` of its
    lowest point along ``axis`` (*up* the vessel in its own frame; +y is a
    booster's nose).  A few kRPC calls per part: call it once, before the
    flight, never in a control loop.
    """
    frame = vessel.reference_frame
    rows = []
    try:
        legs = [leg.part for leg in vessel.parts.legs]
    except Exception:                                       # noqa: BLE001
        legs = []
    if legs:
        # Deployed legs are what touches, wherever they sit before deploying.
        return _weakest(legs)
    try:
        for part in vessel.parts.all:
            lower, upper = part.bounding_box(frame)
            # the box's lowest corner along the axis
            low = sum(min(lower[i] * axis[i], upper[i] * axis[i])
                      for i in range(3))
            rows.append((low, part))
    except Exception:                                       # noqa: BLE001
        return None
    if not rows:
        return None
    bottom = min(r[0] for r in rows)
    return _weakest([p for low, p in rows if low <= bottom + band])


def _weakest(hit):
    best = None
    for part in hit:
        try:
            tol = part.impact_tolerance
        except Exception:                                   # noqa: BLE001
            continue
        if best is None or tol < best[0]:
            best = (tol, part.title)
    return (best[0], best[1], len(hit)) if best else None


# --- Landing limits from the legs' live suspension ---------------------------

LEGS_SERVICE = "krpc_lab_legs"
PHYSICS_DT = 0.02           # KSP's physics step; ModuleWheelDamage compares
                            # the down-force change across one of them
G0 = 9.81                   # PhysicsGlobals.GravitationalAcceleration


def read_legs(conn):
    """The window mod's per-leg rows (``launcherMod/Legs.cs``) or None."""
    service = getattr(conn, LEGS_SERVICE, None)
    if service is None:
        return None
    try:
        rows = service.legs()
    except Exception:                                       # noqa: BLE001
        return None
    keys = ("title", "x", "y", "z", "travel", "spring", "damper",
            "spring_ratio", "damper_ratio", "impact_tol", "explode_mult",
            "crash_tol", "friction", "wheel_radius", "deployed")
    legs = []
    for row in rows:
        f = row.split("|")
        if len(f) != len(keys):
            continue
        leg = {"title": f[0]}
        try:
            for k, v in zip(keys[1:], f[1:]):
                leg[k] = float(v)
        except ValueError:
            continue
        legs.append(leg)
    return legs


def _hull(points):
    """Convex hull of 2-D points, counter-clockwise (monotone chain)."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_margin(points):
    """Distance from the COM's plumb line (the origin) to the nearest edge
    of the support polygon -- the tipping lever in the worst direction --
    or 0 if the origin is not inside it."""
    hull = _hull(points)
    if len(hull) < 3:
        return 0.0
    best = None
    for i, a in enumerate(hull):
        b = hull[(i + 1) % len(hull)]
        ex, ez = b[0] - a[0], b[1] - a[1]
        length = math.hypot(ex, ez)
        if length < 1e-9:
            continue
        # signed distance, positive inside a counter-clockwise hull
        d = (ex * (0.0 - a[1]) - ez * (0.0 - a[0])) / length
        best = d if best is None else min(best, d)
    return max(0.0, best or 0.0)


def landing_limits(mass, inertia, g, legs):
    """Vertical and lateral touchdown limits of a legged craft, or None.

    ``mass`` kg, ``inertia`` the transverse moment of inertia about the COM
    (kg m^2, the smaller of the two), ``g`` local gravity, ``legs`` from
    :func:`read_legs` with the feet where they will touch (deployed).

    Vertical: a leg explodes when its down force jumps by ``impact_tol x
    explode_mult x g0/g`` (kN) within one physics step.  The damper is what
    jumps -- from nothing to ``damper x v`` on the contact tick, the spring
    adding ``spring x v x dt`` -- so ``v = tol / (damper + spring dt)``; and
    the stroke bottoms out above ``travel x sqrt(spring_ratio x g)`` (the
    spring is ``spring_ratio`` x the leg's share of the weight).

    Lateral, the foot stopped by friction: the craft pivots about the
    leading feet, angular momentum about them conserved through the contact
    however soft the dampers make it, and it topples if that rotation's
    energy lifts the COM over the pivot.  The leading leg compresses and the
    trailing one extends as the weight moves onto the pivot -- the weight of
    the whole craft on springs set for its n-th share -- which tilts the
    craft ``atan(n/spring_ratio / 2b)`` toward the tip before it starts.  The
    vertical speed at contact pushes the other way and is left out, and so is
    the damper's dissipation during the rotation: both make the real limit
    higher.  If ``friction x h > b`` the craft can also be tipped by the foot
    *sliding* (friction torque beats the restoring torque), whatever the speed.
    """
    feet = [l for l in legs if math.isfinite(l["y"])]
    if len(feet) < 3 or mass <= 0.0 or g <= 0.0:
        return None
    radius = max(l["wheel_radius"] for l in feet)
    h = -min(l["y"] for l in feet) + radius          # COM above the ground
    b = support_margin([(l["x"], l["z"]) for l in feet])
    if h <= 0.0 or b <= 0.0:
        return {"h": h, "b": b, "vertical": None, "lateral": 0.0,
                "slide_tips": True, "n": len(feet)}
    n = len(feet)
    sr = min(l["spring_ratio"] for l in feet)
    travel = min(l["travel"] for l in feet)
    tilt = math.atan(min(n / sr, travel) / (2.0 * b))
    phi = math.atan2(b, h) - tilt
    r = math.hypot(h, b)
    if phi <= 0.0:
        lateral = 0.0
    else:
        pivot = inertia + mass * r * r
        lateral = (math.sqrt(2.0 * pivot * mass * g * r * (1.0 - math.cos(phi)))
                   / (mass * r * math.cos(phi)))
    vertical = []
    for l in feet:
        tol = l["impact_tol"] * l["explode_mult"] * G0 / g
        jump = l["damper"] + l["spring"] * PHYSICS_DT
        if tol > 0.0 and jump > 0.0:
            vertical.append(tol / jump)
    bottom = travel * math.sqrt(sr * g)
    v = min(vertical + [bottom]) if vertical else bottom
    mu = min(l["friction"] for l in feet)
    return {"h": h, "b": b, "n": n, "tilt_deg": math.degrees(tilt),
            "vertical": v, "vertical_damper": min(vertical) if vertical else None,
            "vertical_bottom": bottom, "lateral": lateral,
            "friction": mu, "slide_tips": mu * h > b,
            "deployed": all(l["deployed"] for l in feet)}
