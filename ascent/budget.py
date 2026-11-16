"""What the booster needs to fly home, from the state it would separate in.

Pure functions, no kRPC.  ``recovery`` answers one question per tick of the
ascent: *if the booster separated now, how much propellant would it land
with?*  The ascent separates when that answer reaches the reserve, so the
booster burns everything it does not need for the trip back on lifting the
upper stage.

Nothing here is fitted to a vehicle.  The booster enters as its masses, its
engines' thrust and Isp, and its body radius; the state as its height,
distance from the pad and velocity.  Two parts:

**Boostback.**  A finite burn, integrated in the vertical plane through the
pad, flip included (see ``boostback``).  Every step asks what horizontal velocity would carry a ballistic
booster from where it is now to the pad (``return_speed``: flat ground,
vacuum, gravity at the current height) and thrusts at the full engine
acceleration along the nose, which turns towards the pad.  The burn ends when the velocity
matches.  Drag is left out on purpose: the burn is high, and on the way down
drag mostly takes vertical speed, where the horizontal speed is already
small.  The heavy booster's real boostback (LOG12758: separated at 22.6 km,
25.5 km out, 791 m/s away and 394 up) burned 1251 m/s; this model gives
within a few percent of that (``tests/testBudget``).

**Landing.**  The booster falls from the boostback's burn-out through the
atmosphere (``fall``: the body's own density profile, gravity, the
horizontal speed the boostback left it), and the landing burn is the one
boosterland flies: lit at the height its law asks for
(``trajectory.landing_burn_state``: where the booster meets its descent
profile, read backwards; no reaction time or attitude rate is known before
the flight, so none is counted), then a constant deceleration to the ground.  Over that height the
burn's dv is the speed plus the gravity it holds up: ``v + 2 g s H / v``,
``s`` the vertical share of the velocity.  The fall's drag is the craft's
own Cd*A at the speed (``cda``: the ascent's zero-alpha table of the
stack), else ``cd`` times the frontal area.  The Heavy Shuttle booster
(LOG16184) lit at 5.2 km doing 680 m/s and needed ~850 m/s; a sea-level
terminal speed put it at 405 and the booster flamed out at 29 m.

The two burns spend propellant at their own Isp -- vacuum for the boostback,
the pad's for the landing -- and ``recovery`` returns the propellant left
after both, before the margin.
"""

import math
from dataclasses import dataclass

from boosterland import trajectory
from boosterland.guidance import drift_rate, flip_throttle

from boosterland.config import Config as _BoosterConfig

_BCFG = _BoosterConfig()
FLIP_ALIGN_DEG = math.degrees(math.acos(1.0 - _BCFG.FLIP_THRUST_LOSS))    # the flip is over inside
# boosterland's landing law (trajectory.landing_burn_state), which the
# budget has to fly:
LANDING_THROTTLE_CAP = _BCFG.LANDING_THROTTLE_CAP


class _Unmeasured:
    """boosterland's in-flight measurements, none taken yet."""
    touchdown_tolerance = None
    attitude_rate = None
    engine_spool_s = None
    control_interval = None

G0 = 9.80665


@dataclass
class Booster:
    mass: float             # kg, everything that separates, propellant included
    dry_mass: float         # kg, mass with the main propellant gone
    thrust_vac: float       # N, all its engines at full throttle in vacuum
    thrust_sl: float        # N, ... at the pad's pressure
    isp_vac: float          # s
    isp_sl: float           # s
    radius: float           # m, body radius for the frontal area
    # For a flip flown on boosterland's FLIP_THROTTLE_LAW (None: the old
    # full-throttle flip at a fixed rate):
    gimbal_accel: float = None  # rad/s^2 per unit throttle
    fixed_accel: float = 0.0    # rad/s^2 from RCS and wheels


@dataclass
class Body:
    mu: float               # m^3/s^2
    radius: float           # m, to the pad's surface
    density_pad: float      # kg/m^3 at the pad
    # kg/m^3 every ``step`` m above the pad, from 0 up; required when
    # density_pad > 0 (the fall's drag is taken at its height)
    density: tuple = ()
    step: float = 1000.0


@dataclass
class State:
    height: float           # m above the pad
    downrange: float        # m from the pad along the ground, >= 0
    v_away: float           # m/s horizontal, away from the pad (+)
    v_cross: float          # m/s horizontal, across that line (any sign)
    v_up: float             # m/s vertical


@dataclass
class Recovery:
    boostback_dv: float     # m/s
    landing_dv: float       # m/s
    burn_s: float           # s, the boostback's duration
    terminal: float         # m/s, the speed the landing burn starts from
    apex: float             # m, above the pad, after the boostback
    propellant_left: float  # kg after both burns; negative = cannot return
    propellant: float       # kg aboard now


def gravity(body, height):
    r = body.radius + max(0.0, height)
    return body.mu / (r * r)


def fall_time(height, v_up, g):
    """Seconds until a vacuum ballistic body at ``height`` going up at
    ``v_up`` reaches the ground (flat, constant ``g``)."""
    h = max(0.0, height)
    return (v_up + math.sqrt(v_up * v_up + 2.0 * g * h)) / g


def return_speed(downrange, height, v_up, g):
    """Horizontal speed towards the pad that lands a ballistic body on it."""
    return downrange / max(1.0, fall_time(height, v_up, g))


def boostback(body, booster, state, flip_rate_deg=30.0, dt=0.1,
              max_s=600.0):
    """Integrate the boostback burn, flip included.  Returns (dv, mass
    after, seconds, the state at burn-out).

    The engines light pointing prograde and the nose swings over the top
    towards the pad at ``flip_rate_deg`` while they burn, as boosterland
    flies it (``FLIP_UNDER_POWER``): the thrust spent turning is spent, and
    the part of it that pushes upward raises the apex, which lengthens the
    fall and lowers the return speed.  The cross-track velocity is nulled
    in quadrature with the along-track change.
    """
    x = state.downrange
    h = state.height
    u = state.v_away            # + away from the pad
    w = state.v_up
    m = booster.mass
    mdot = booster.thrust_vac / (booster.isp_vac * G0)
    rate = math.radians(max(1e-3, flip_rate_deg))
    point = math.atan2(w, u)    # nose angle in the (away, up) plane
    spin = 0.0                  # rad/s, the law's turn
    law = booster.gimbal_accel is not None
    flipping = True
    dv = 0.0
    t = 0.0
    while t < max_s and m > booster.dry_mass:
        g = gravity(body, h)
        want = -return_speed(x, h, w, g)        # towards the pad
        if u <= want:
            break
        target = math.pi                        # horizontal, towards the pad
        a = booster.thrust_vac / m
        throttle = 1.0
        if law:
            theta = max(0.0, target - point)
            if flipping and theta <= math.radians(FLIP_ALIGN_DEG):
                flipping = False
            if flipping:
                fall = fall_time(h, w, g)
                throttle, _, _ = flip_throttle(
                    theta, spin, booster.gimbal_accel, booster.fixed_accel,
                    a, drift_rate(x, u, fall))
            alpha = throttle * booster.gimbal_accel + booster.fixed_accel
            if spin * spin / (2.0 * max(alpha, 1e-9)) >= theta:
                spin = max(0.0, spin - alpha * dt)
            else:
                spin += alpha * dt
            point = min(target, point + spin * dt)
        elif point < target:
            point = min(target, point + rate * dt)
        step = a * dt * throttle
        if step <= 0.0:
            # coasting through the flip on RCS and wheels
            x += u * dt
            h += w * dt
            w -= g * dt
            t += dt
            continue
        du = step * math.cos(point)
        if du < 0.0 and u + du < want:          # matched inside this step
            frac = (want - u) / du
            step *= frac
            du *= frac
        else:
            frac = 1.0
        u += du
        w += step * math.sin(point) - g * dt * frac
        x += u * dt * frac
        h += w * dt * frac
        m -= mdot * dt * frac * throttle
        dv += step
        t += dt * frac
    # The cross-track has to go too; it is small, so it is charged in
    # quadrature rather than flown.
    cross = abs(state.v_cross)
    dv = math.hypot(dv, cross)
    m_after = booster.mass * math.exp(-dv / (booster.isp_vac * G0))
    return dv, m_after, t, State(h, x, u, 0.0, w)


def density(body, height):
    if body.density_pad <= 0.0:
        return 0.0
    table = body.density
    if not table:
        raise ValueError("budget.Body has air but no density profile")
    k = max(0.0, height) / body.step
    i = int(k)
    if i >= len(table) - 1:
        return table[-1]
    f = k - i
    return table[i] * (1.0 - f) + table[i + 1] * f


def ignition(speed, accel, g):
    """The height boosterland lights the landing burn at, for a booster at
    ``speed`` with ``accel`` of full thrust; None when the engines cannot
    hold it up."""
    if trajectory.planned_net(accel, g, _BCFG) <= 0.0:
        return None
    profile = trajectory.descent_profile(_Unmeasured, _BCFG, accel, g)
    return trajectory.profile_height(profile, speed) + _BCFG.TOUCHDOWN_ALT_M


def fall(body, booster, mass, after, cd, cda=None, dt=0.5, max_s=900.0):
    """Fall from the boostback's burn-out to the landing burn's ignition.
    Returns (dv of the landing burn, the speed it lights at, its height)."""
    area = math.pi * booster.radius * booster.radius
    a = booster.thrust_sl / max(1.0, mass)
    h, u, w = after.height, abs(after.v_away), after.v_up
    t = 0.0
    while t < max_s:
        g = gravity(body, h)
        v = math.hypot(u, w)
        if w < 0.0 or h <= 0.0:
            need = ignition(v, a, g)
            if need is None:
                return float("inf"), v, h
            if h <= need or h <= 0.0:
                s = -w / max(1e-9, v)
                height = max(need, 1e-6)
                return v + 2.0 * g * s * height / max(1e-9, v), v, h
        rho = density(body, h)
        if rho > 0.0 and v > 0.0:
            k = cda(h, v) if cda is not None else cd * area
            d = 0.5 * rho * v * k / mass        # drag / speed
            u -= d * u * dt
            w -= d * w * dt
        w -= g * dt
        h += w * dt
        t += dt
    return float("inf"), math.hypot(u, w), h


class PlaneEnv(_Unmeasured):
    """``body`` as boosterland's propagator reads an ``Environment``: the
    pad at (R, 0, 0), the booster's flight in the x-y plane, the air the
    ascent read off the game, the stack's Cd*A.  Heights are above the pad
    (``Body.radius``, as everywhere here); the body does not turn (the
    states are already ground-relative)."""

    def __init__(self, body, drag):
        self.mu = body.mu
        self.equatorial_radius = body.radius
        self.target_radius = body.radius
        self.target = (body.radius, 0.0, 0.0)
        self.omega = (0.0, 0.0, 0.0)
        self.atmosphere_depth = (len(body.density) * body.step
                                 if body.density_pad > 0.0 else 0.0)
        self._body = body
        self._drag = drag
        self.leg_clearance = None

    def density(self, altitude):
        return density(self._body, altitude)

    def drag_area_at(self, speed, altitude):
        return self._drag(altitude, speed)


def _plane_state(body, state, extra_u=0.0):
    """``state`` as (r, v) in PlaneEnv's frame; ``extra_u`` m/s more
    towards the pad."""
    theta = state.downrange / body.radius
    up = (math.cos(theta), math.sin(theta), 0.0)
    away = (-math.sin(theta), math.cos(theta), 0.0)
    rr = body.radius + max(0.0, state.height)
    u = state.v_away - extra_u
    r = tuple(rr * c for c in up)
    v = tuple(u * a + state.v_up * b for a, b in zip(away, up))
    return r, v


def predicted_landing(body, booster, mass, after, drag, extra_u=0.0):
    """boosterland's own prediction from the boostback's burn-out: the fall
    with drag, the landing burn lit and flown on its law, its propellant
    spent at the pad's Isp with no floor.  Returns (downrange of the
    touchdown, + beyond the pad on the separation's side; the mass left;
    the speed the burn lights at)."""
    env = PlaneEnv(body, drag)
    env.burn_isp = booster.isp_sl
    r, v = _plane_state(body, after, extra_u)
    accel = booster.thrust_sl / max(1.0, mass)
    p = trajectory.predict_landing(env, r, v, mass, accel, _BCFG)
    x = math.atan2(p.position[1], p.position[0]) * body.radius
    m = p.final_mass if p.powered and p.final_mass is not None else mass
    return x, m, p.entry_speed, p.powered


def recovery(body, booster, state, cd=1.0, flip_rate_deg=30.0, cda=None):
    """The propellant the booster would land with if it separated in
    ``state``.  ``cda(height, speed)``: the falling booster's Cd*A, m^2
    (None: ``cd`` x the frontal area).

    The boostback is the burn above, aimed by a vacuum return; boosterland
    aims it by its prediction, which falls through air: the drag takes
    horizontal speed on the way down and the vacuum aim lands short (the
    fin booster's real boostback spent ~4 t more than this burn).  So the
    burn-out is carried on (``extra``, m/s more towards the pad, solved by
    secant) until boosterland's own prediction -- the fall, the trigger,
    the landing burn on its law, its propellant spent -- touches down on
    the pad, and what that burn leaves is what the booster lands with."""
    propellant = booster.mass - booster.dry_mass
    bb_dv, m1, burn_s, after = boostback(body, booster, state, flip_rate_deg)
    g = gravity(body, after.height)
    drag = cda if cda is not None else (
        lambda h, v, k=cd * math.pi * booster.radius ** 2: k)
    if not math.isfinite(m1) or m1 <= 0.0:
        return Recovery(bb_dv, float("inf"), burn_s, 0.0, after.height,
                        -float("inf"), propellant)

    def landed(extra):
        m = m1 * math.exp(-abs(extra) / (booster.isp_vac * G0))
        return (predicted_landing(body, booster, m, after, drag, extra), m)

    extra = 0.0
    (x0, m_end, v_t, powered), m_bb = landed(extra)
    # A miss of x needs ~ x / (fall time) of horizontal speed; the secant
    # takes over from that first step.
    fall_s = max(1.0, fall_time(after.height, after.v_up, g))
    e_prev, x_prev = extra, x0
    extra = x0 / fall_s
    for _ in range(SOLVE_ITERATIONS):
        (x, m_end, v_t, powered), m_bb = landed(extra)
        if abs(x) <= LANDING_TOLERANCE_M or x == x_prev:
            break
        step = (extra - e_prev) * x / (x - x_prev)
        e_prev, x_prev = extra, x
        extra -= step
    bb_dv += abs(extra)
    apex = after.height + max(0.0, after.v_up) ** 2 / (2.0 * g)
    if powered and m_end > 0.0:
        land_dv = booster.isp_sl * G0 * math.log(m_bb / m_end)
    else:
        land_dv, m_end = float("inf"), -float("inf")
    return Recovery(boostback_dv=bb_dv, landing_dv=land_dv, burn_s=burn_s,
                    terminal=v_t, apex=apex,
                    propellant_left=m_end - booster.dry_mass,
                    propellant=propellant)


# The secant on the boostback's last metres: a few predictions per
# recovery, and a touchdown within a booster's own length of the pad (the
# margin is the reserve's job, not the aim's).
SOLVE_ITERATIONS = 4
LANDING_TOLERANCE_M = 20.0


def separation_target(upper_accel, booster_mass, booster_thrust_now, ratio):
    """The booster's thrust at separation, and the fraction of its present
    thrust that is: its acceleration once free must be ``ratio`` of the
    upper stage's, so the upper stage pulls away from it.  ``None`` when the
    booster makes no thrust to cut."""
    target = max(0.0, ratio * upper_accel * booster_mass)
    if booster_thrust_now <= 0.0:
        return target, None
    return target, min(1.0, target / booster_thrust_now)
