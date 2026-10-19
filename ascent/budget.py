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

**Landing.**  The booster falls tail first at its terminal velocity and the
landing burn takes that out at full thrust: ``dv = v_t * a / (a - g)``, the
gravity loss of a constant-thrust stop.  ``v_t`` comes from the booster's
own mass and frontal area at the pad's air density, with a drag coefficient
from ``Config`` (1.0: a bare cylinder falls faster than one with airbrakes
open, so the estimate errs towards keeping propellant).  It is capped by the
speed a vacuum fall from the apex would reach, for bodies with thin or no
air.  The heavy fell at ~242 m/s and burned ~330 m/s stopping; the model
gives 294 and 334.

The two burns spend propellant at their own Isp -- vacuum for the boostback,
the pad's for the landing -- and ``recovery`` returns the propellant left
after both, before the margin.
"""

import math
from dataclasses import dataclass

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


@dataclass
class Body:
    mu: float               # m^3/s^2
    radius: float           # m, to the pad's surface
    density_pad: float      # kg/m^3 at the pad


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
    dv = 0.0
    t = 0.0
    while t < max_s and m > booster.dry_mass:
        g = gravity(body, h)
        want = -return_speed(x, h, w, g)        # towards the pad
        if u <= want:
            break
        target = math.pi                        # horizontal, towards the pad
        if point < target:
            point = min(target, point + rate * dt)
        a = booster.thrust_vac / m
        step = a * dt
        du = step * math.cos(point)
        if u + du < want:                       # matched inside this step
            frac = (want - u) / du
            step *= frac
            du *= frac
        else:
            frac = 1.0
        u += du
        w += step * math.sin(point) - g * dt * frac
        x += u * dt * frac
        h += w * dt * frac
        m -= mdot * dt * frac
        dv += step
        t += dt * frac
    # The cross-track has to go too; it is small, so it is charged in
    # quadrature rather than flown.
    cross = abs(state.v_cross)
    dv = math.hypot(dv, cross)
    m_after = booster.mass * math.exp(-dv / (booster.isp_vac * G0))
    return dv, m_after, t, State(h, x, u, 0.0, w)


def terminal_speed(body, booster, mass, cd):
    area = math.pi * booster.radius * booster.radius
    g = gravity(body, 0.0)
    if body.density_pad <= 0.0 or area <= 0.0 or cd <= 0.0:
        return float("inf")
    return math.sqrt(2.0 * mass * g / (body.density_pad * cd * area))


def landing_dv(body, booster, mass, apex, cd):
    """dv of the landing burn for a booster of ``mass`` falling from
    ``apex``.  Returns (dv, the speed it starts from)."""
    g = gravity(body, 0.0)
    v_vacuum = math.sqrt(2.0 * g * max(0.0, apex))
    v = min(v_vacuum, terminal_speed(body, booster, mass, cd))
    a = booster.thrust_sl / max(1.0, mass)
    if a <= g:
        return float("inf"), v
    return v * a / (a - g), v


def recovery(body, booster, state, cd=1.0, flip_rate_deg=30.0):
    """The propellant the booster would land with if it separated in
    ``state``."""
    propellant = booster.mass - booster.dry_mass
    bb_dv, m1, burn_s, after = boostback(body, booster, state, flip_rate_deg)
    g = gravity(body, after.height)
    apex = after.height + max(0.0, after.v_up) ** 2 / (2.0 * g)
    # The landing burn's own mass: iterate once, starting from the dry mass
    # plus what the burn will use -- the stop is short, so one pass is enough.
    land_dv, v_t = landing_dv(body, booster, m1, apex, cd)
    if math.isfinite(land_dv):
        m2 = m1 * math.exp(-land_dv / (booster.isp_sl * G0))
        land_dv, v_t = landing_dv(body, booster, 0.5 * (m1 + m2), apex, cd)
        m2 = m1 * math.exp(-land_dv / (booster.isp_sl * G0))
    else:
        m2 = -float("inf")
    return Recovery(boostback_dv=bb_dv, landing_dv=land_dv, burn_s=burn_s,
                    terminal=v_t, apex=apex,
                    propellant_left=m2 - booster.dry_mass,
                    propellant=propellant)


def separation_target(upper_accel, booster_mass, booster_thrust_now, ratio):
    """The booster's thrust at separation, and the fraction of its present
    thrust that is: its acceleration once free must be ``ratio`` of the
    upper stage's, so the upper stage pulls away from it.  ``None`` when the
    booster makes no thrust to cut."""
    target = max(0.0, ratio * upper_accel * booster_mass)
    if booster_thrust_now <= 0.0:
        return target, None
    return target, min(1.0, target / booster_thrust_now)
