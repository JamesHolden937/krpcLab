"""MechJeb classic's turn shape, solved from the craft.

Pure, no kRPC.  MechJeb's classic ascent flies a pitch that is a function of
altitude alone:

    pitch(h) = 90                                         h < start
             = 90 - ((h - start) / (end - start))**shape * (90 - end_angle)
             = end_angle                                  h > end

and burns until the apoapsis reaches the target, coasts, and circularizes.
``shape`` (MechJeb's "turn shape" percentage / 100) is the one free choice:
low turns early and flat, high climbs steeply and turns late.

A steep climb loses to gravity; a flat one loses to drag and carries the
booster further downrange, so it must keep more propellant for the trip back
and leave earlier.  The ascent separates by ``budget.recovery`` (the booster
keeps exactly its return propellant plus the reserve), so both costs land in
one number: **the upper stage's mass in the target orbit**.  ``solve``
integrates the whole flight for a shape -- the stack under thrust, the
separation by the same rule the pilot flies, the upper stage to the
apoapsis, the coast and the circularization -- and maximizes that mass.

The flight is a point mass in the equatorial plane of a rotating body:
thrust along the commanded pitch, drag against the air-relative velocity
from the craft's own Cd*A table (sampled from the game at zero angle of
attack), Isp and thrust between their sea-level and vacuum values by the
pressure, mass flow constant.  The booster's throttle-down before
separation and the coast's drag are left out.
"""

import math
from dataclasses import dataclass, field

from . import budget

G0 = 9.80665


@dataclass
class Engines:
    thrust_vac: float       # N
    isp_vac: float          # s
    isp_sl: float           # s, at 1 atm

    def thrust(self, p_atm):
        isp = self.isp_vac + (self.isp_sl - self.isp_vac) * min(1.0, p_atm)
        return self.thrust_vac * isp / self.isp_vac

    @property
    def mdot(self):
        return self.thrust_vac / (self.isp_vac * G0)


@dataclass
class Craft:
    booster: budget.Booster     # as the ascent builds it, propellant full
    booster_engines: Engines
    upper_mass: float           # kg, everything above the booster
    upper_propellant: float     # kg of it
    upper_engines: Engines
    upper_lit: bool             # upper engines burn from liftoff
    # Cd*A (m^2) on a grid: cda[i][j] at heights[i] (m), speeds[j] (m/s)
    heights: list = field(default_factory=list)
    speeds: list = field(default_factory=list)
    cda: list = field(default_factory=list)


@dataclass
class World:
    body: budget.Body           # mu, pad radius, pad density
    omega: float                # rad/s, the body's rotation
    pressure: list              # atm at every PROFILE_STEP_M from the pad
    density: list               # kg/m^3, the same
    step: float = 1000.0        # m between profile samples


@dataclass
class Path:
    start: float                # m above the pad: the vertical climb's top
    end: float
    # m/s surface speed that also ends the vertical climb (MechJeb's
    # turn-start velocity); the turn is then measured from the height it
    # began at, not from ``start``
    start_speed: float = float("inf")
    end_angle: float = 0.0      # deg
    orbit: float = 80000.0      # m, apoapsis and circular target
    reserve: float = 0.15       # RESERVE_FRACTION
    landing_cd: float = 1.0
    flip_rate_deg: float = 30.0


@dataclass
class Result:
    shape: float
    mass: float                 # kg in orbit; -inf when it does not get there
    sep_height: float = float("nan")
    sep_downrange: float = float("nan")
    sep_time: float = float("nan")
    circ_dv: float = float("nan")
    reason: str = ""
    # (t, height, ground downrange, air speed) every ``track_every`` s
    track: list = field(default_factory=list)


def pitch_deg(h, path, shape, turn_start=None):
    """MechJeb classic's pitch at height h.  ``turn_start`` is the height
    the vertical climb ended at (``path.start`` or lower, when the speed
    ended it first); None while still climbing vertically."""
    h0 = path.start if turn_start is None else turn_start
    if h <= h0:
        return 90.0
    if h >= path.end:
        return path.end_angle
    x = (h - h0) / (path.end - h0)
    return 90.0 - x ** shape * (90.0 - path.end_angle)


def _profile(table, h, step):
    if not table:
        return 0.0
    k = max(0.0, h) / step
    i = int(k)
    if i >= len(table) - 1:
        return table[-1]
    f = k - i
    return table[i] * (1.0 - f) + table[i + 1] * f


def _interp(xs, x):
    """Index and fraction of x in the ascending list xs, clamped."""
    if x <= xs[0]:
        return 0, 0.0
    if x >= xs[-1]:
        return len(xs) - 2, 1.0
    lo = 0
    while xs[lo + 1] < x:
        lo += 1
    return lo, (x - xs[lo]) / (xs[lo + 1] - xs[lo])


def cda_at(craft, h, v):
    if not craft.cda:
        return 0.0
    if len(craft.heights) == 1:
        row = craft.cda[0]
        j, g = _interp(craft.speeds, v)
        return row[j] * (1 - g) + row[j + 1] * g
    i, f = _interp(craft.heights, h)
    j, g = _interp(craft.speeds, v)
    a = craft.cda[i][j] * (1 - g) + craft.cda[i][j + 1] * g
    b = craft.cda[i + 1][j] * (1 - g) + craft.cda[i + 1][j + 1] * g
    return a * (1 - f) + b * f


def apsides(world, r, vr, vt):
    """(apoapsis radius, speed there) of the inertial orbit; apoapsis None
    when the orbit is open."""
    mu = world.body.mu
    v2 = vr * vr + vt * vt
    energy = 0.5 * v2 - mu / r
    if energy >= 0.0:
        return None, None
    a = -mu / (2.0 * energy)
    h = r * vt
    e = math.sqrt(max(0.0, 1.0 + 2.0 * energy * h * h / (mu * mu)))
    ra = a * (1.0 + e)
    return ra, h / ra


def fly(craft, world, path, shape, dt=0.2, max_s=900.0, recover_every=1.0,
        track_every=None):
    """One ascent for ``shape``; returns a ``Result``.  ``track_every`` (s)
    records the trajectory for comparison with a flight's telemetry.

    Downrange is along the ground: theta is inertial, and the pad turns with
    the body, so the pad's own angle omega*t comes off."""
    R = world.body.radius
    mu = world.body.mu
    r, theta = R, 0.0
    vr, vt = 0.0, world.omega * R
    b = craft.booster
    b_prop = b.mass - b.dry_mass
    u_prop = craft.upper_propellant
    attached = True
    t = 0.0
    next_check = 0.0
    sep = None
    snap = None                     # state at the last coarse check
    fine = False                    # re-flying the last interval per step
    track = []
    next_track = 0.0
    turn_start = None               # height the vertical climb ended at
    target_r = R + path.orbit       # the pad is the reference (as budget)
    while t < max_s:
        h = r - R
        p = _profile(world.pressure, h, world.step)
        rho = _profile(world.density, h, world.step)
        # air-relative velocity: the atmosphere turns with the body
        ur, ut = vr, vt - world.omega * r
        speed = math.hypot(ur, ut)
        ground = R * (theta - world.omega * t)
        if track_every and t >= next_track and not fine:
            next_track = t + track_every
            track.append((t, h, ground, speed))
        if attached and (fine or t >= next_check):
            next_check = t + recover_every
            state = budget.State(height=h, downrange=ground, v_away=ut,
                                 v_cross=0.0, v_up=ur)
            bb = budget.Booster(mass=b.dry_mass + b_prop,
                                dry_mass=b.dry_mass,
                                thrust_vac=b.thrust_vac, thrust_sl=b.thrust_sl,
                                isp_vac=b.isp_vac, isp_sl=b.isp_sl,
                                radius=b.radius, gimbal_accel=b.gimbal_accel,
                                fixed_accel=b.fixed_accel)
            rec = budget.recovery(world.body, bb, state, cd=path.landing_cd,
                                  flip_rate_deg=path.flip_rate_deg)
            need = rec.propellant - rec.propellant_left
            if (h > 1000.0 and rec.propellant_left
                    <= path.reserve * need) or b_prop <= 0.0:
                if not fine and snap is not None:
                    # crossed inside the last interval: go back and find
                    # the step (the booster burns ~1.6 t/s on the heavy)
                    fine = True
                    r, theta, vr, vt, b_prop, u_prop, t = snap
                    continue
                attached = False
                sep = (h, ground, t)
            elif not fine:
                snap = (r, theta, vr, vt, b_prop, u_prop, t)
        mass = (b.dry_mass + b_prop if attached else 0.0) \
            + craft.upper_mass - (craft.upper_propellant - u_prop)
        thrust = 0.0
        if attached:
            thrust += craft.booster_engines.thrust(p)
            b_prop -= craft.booster_engines.mdot * dt
        if (not attached or craft.upper_lit) and u_prop > 0.0:
            thrust += craft.upper_engines.thrust(p)
            u_prop -= craft.upper_engines.mdot * dt
            if u_prop <= 0.0:
                return Result(shape, -float("inf"), *_sep(sep), track=track,
                              reason="upper stage out of propellant")
        ra, va = apsides(world, r, vr, vt)
        if not attached and ra is not None and ra >= target_r:
            if h < world.step * (len(world.pressure) - 1):
                # apoapsis made inside the air: MechJeb coasts and tops it
                # up; counted as reached
                pass
            v_circ = math.sqrt(mu / ra)
            dv = max(0.0, v_circ - va)
            isp = craft.upper_engines.isp_vac
            m_orbit = mass * math.exp(-dv / (isp * G0))
            m_need = mass - u_prop
            if m_orbit < m_need:
                return Result(shape, -float("inf"), *_sep(sep), track=track, circ_dv=dv,
                              reason="no propellant to circularize")
            return Result(shape, m_orbit, *_sep(sep), track=track, circ_dv=dv)
        if turn_start is None and (h >= path.start
                                   or speed >= path.start_speed):
            turn_start = min(h, path.start)
        pitch = math.radians(pitch_deg(h, path, shape, turn_start))
        a_t = thrust / mass
        drag = 0.5 * rho * speed * speed * cda_at(craft, h, speed) / mass
        dr_ = -drag * ur / speed if speed > 1e-6 else 0.0
        dt_ = -drag * ut / speed if speed > 1e-6 else 0.0
        g = mu / (r * r)
        # polar: r'' = vt^2/r - g + a_r ; d(vt)/dt = -vr vt / r + a_t
        ar = a_t * math.sin(pitch) + dr_ - g + vt * vt / r
        at = a_t * math.cos(pitch) + dt_ - vr * vt / r
        vr += ar * dt
        vt += at * dt
        r += vr * dt
        theta += vt / r * dt
        t += dt
        if r < R - 1.0:
            return Result(shape, -float("inf"), *_sep(sep), track=track,
                          reason="fell back")
    return Result(shape, -float("inf"), *_sep(sep), track=track, reason="timeout")


def _sep(sep):
    if sep is None:
        return float("nan"), float("nan"), float("nan")
    return sep


def solve(craft, world, path, lo=0.1, hi=1.0, tol=0.01, coarse=10):
    """The shape in [lo, hi] that leaves the most mass in orbit: a coarse
    scan (the curve need not be unimodal where the booster's separation
    jumps), then a golden-section refinement around the best cell.
    Returns (best Result, every Result flown)."""
    flown = []
    xs = [lo + (hi - lo) * k / coarse for k in range(coarse + 1)]
    for x in xs:
        flown.append(fly(craft, world, path, x))
    best = max(flown, key=lambda r: r.mass)
    if not math.isfinite(best.mass):
        return best, flown
    k = xs.index(best.shape)
    a, c = xs[max(0, k - 1)], xs[min(coarse, k + 1)]
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    x1, x2 = c - phi * (c - a), a + phi * (c - a)
    r1, r2 = fly(craft, world, path, x1), fly(craft, world, path, x2)
    flown += [r1, r2]
    while c - a > tol:
        if r1.mass >= r2.mass:
            c, x2, r2 = x2, x1, r1
            x1 = c - phi * (c - a)
            r1 = fly(craft, world, path, x1)
            flown.append(r1)
        else:
            a, x1, r1 = x1, x2, r2
            x2 = a + phi * (c - a)
            r2 = fly(craft, world, path, x2)
            flown.append(r2)
    best = max(flown, key=lambda r: r.mass)
    return best, flown
