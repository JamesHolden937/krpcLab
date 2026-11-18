"""Steering and throttle laws for each phase of the return.

Boostback logic: changing horizontal velocity by ``dv`` shifts the predicted
touchdown point by roughly ``dv * time_to_land``, so the burn direction is the
horizontal miss vector reversed and the remaining dv is the miss divided by the
time left in the flight.  Both are re-derived from a fresh prediction every
tick, which makes the burn self-correcting -- it simply stops when the
prediction lands inside its own noise (``Environment.miss_band``).
"""

import math
from dataclasses import dataclass

from common import vec
from . import trajectory
from .trajectory import landing_command, miss_vector

G0 = 9.80665


@dataclass
class BoostbackSolution:
    aim: tuple            # attitude vector, body frame
    miss: tuple           # predicted touchdown minus target
    miss_distance: float
    dv: float             # remaining dv to null the miss
    burn_time: float      # seconds of burn left at full throttle
    throttle: float


def turn_time(theta, omega, accel):
    """Seconds to turn ``theta`` rad and stop, starting at ``omega`` rad/s
    towards the target, at a constant angular acceleration ``accel``: the
    bang-bang minimum.  Already going too fast to stop in time counts as the
    time to brake (the overshoot is what the next tick re-solves)."""
    if accel <= 0.0:
        return float("inf")
    theta = max(0.0, theta)
    w = max(0.0, omega)
    if w * w / (2.0 * accel) >= theta:
        return w / accel
    return (2.0 * math.sqrt(accel * theta + 0.5 * w * w) - w) / accel


def flip_throttle(theta, omega, gimbal_accel, fixed_accel, thrust_accel,
                  drift_rate, steps=40):
    """The throttle that makes the rest of the flip cheapest.

    Thrust lit during the flip buys gimbal authority (``gimbal_accel``,
    rad/s^2 per unit throttle) on top of whatever RCS and wheels give
    (``fixed_accel``), so a higher throttle turns faster -- but only the part
    of it along the target is any use.  Over a turn of ``theta`` that is
    ``sin(theta)/theta`` of it; the rest is spent.  And every second of the
    turn the booster drifts on, which the burn has to buy back at
    ``drift_rate`` m/s per second.  So

        cost(u) = u * thrust_accel * t(u) * (1 - sin(theta)/theta)
                  + drift_rate * t(u),      t(u) = turn_time(alpha(u))

    -- the waste rises with u and the drift falls with it.  Returns
    ``(u, t, cost)`` for the cheapest u on a grid over [0, 1]; u = 0 is a
    coast flip on RCS and wheels alone, when they can do it.
    """
    theta = max(0.0, theta)
    useless = 1.0 - (math.sin(theta) / theta if theta > 1e-6 else 1.0)
    best = None
    for k in range(steps + 1):
        u = k / float(steps)
        a = u * gimbal_accel + fixed_accel
        t = turn_time(theta, omega, a)
        if not math.isfinite(t):
            continue
        cost = u * thrust_accel * t * useless + drift_rate * t
        if best is None or cost < best[2] - 1e-9:
            best = (u, t, cost)
    return best if best is not None else (1.0, float("inf"), float("inf"))


def drift_rate(x, v_away, time_to_land):
    """m/s of boostback dv that each second of not burning adds: the return
    speed ``x/T`` grows as the booster flies on (``v_away/T``) and as the
    time it has left to get back shrinks (``x/T^2``)."""
    t = max(1.0, time_to_land)
    return max(0.0, v_away) / t + max(0.0, x) / (t * t)


def boostback_solution(env, r, v, mass, max_accel, isp, prediction, cfg):
    miss = miss_vector(env, prediction)
    up = vec.unit(r)
    horizontal_miss = vec.project_out(miss, up)
    distance = vec.norm(horizontal_miss)

    # Burn opposite the miss.
    if distance > 1e-6:
        aim = vec.scale(vec.unit(horizontal_miss), -1.0)
    else:
        aim = vec.scale(vec.unit(v), -1.0)

    time_left = max(1.0, prediction.time_to_land)
    dv = distance / time_left

    burn_time = 0.0
    if max_accel > 0.0 and isp > 0.0:
        exhaust = isp * G0
        thrust = max_accel * mass
        mass_flow = thrust / exhaust
        burn_time = mass / mass_flow * (1.0 - math.exp(-dv / exhaust))

    throttle = taper_throttle(env, cfg, dv, max_accel)

    return BoostbackSolution(aim=aim, miss=miss, miss_distance=distance,
                             dv=dv, burn_time=burn_time, throttle=throttle)


def taper_throttle(env, cfg, dv, max_accel):
    """The boostback's throttle for ``dv`` still to burn: full until the dv
    left is what full thrust delivers in ``SETTLE_LAGS`` measured reaction
    times (spool-up plus a control tick, ``trajectory.reaction_time``), then
    proportional, so the dv closes exponentially over that time and the
    burn can stop inside the prediction's noise instead of a tick's worth of
    full thrust past it.  Unmeasured: full."""
    if dv <= 0.0:
        return 0.0
    span = max_accel * cfg.SETTLE_LAGS * trajectory.reaction_time(env)
    if span <= 0.0:
        return 1.0
    return vec.clamp(dv / span, 0.0, 1.0)


def held_boostback(env, r, prediction, heading, cfg, max_accel):
    """The boostback taper flown on a fixed ``heading``.

    ``boostback_solution`` re-aims opposite the miss every tick, which is
    right while the miss is kilometres and wrong at the end: as the
    along-track part closes, whatever cross-track is left sets the direction,
    and the nose swings sideways at a few per cent throttle chasing a miss
    the burn can no longer make any real difference to.  Here the heading
    is the one taken when the taper began and only the miss *along* it is
    burnt off, so the throttle still tapers to zero and the nose holds still.

    Returns ``(throttle, remaining)``: ``remaining`` is the miss left along
    the heading's horizontal part, positive while the burn still has range
    to take off.
    """
    up = vec.unit(r)
    flat = vec.project_out(heading, up)
    miss = vec.project_out(miss_vector(env, prediction), up)
    if vec.norm(flat) < 1e-9:
        return 0.0, 0.0
    remaining = -vec.dot(miss, vec.unit(flat))
    dv = max(0.0, remaining) / max(1.0, prediction.time_to_land)
    return taper_throttle(env, cfg, dv, max_accel), remaining


def rcs_trim_command(cross, left, deadband, accel, to_go):
    """The world-frame translation that walks the cross-track miss to zero.

    A sideways acceleration ``a`` held for the time left ``t`` moves the
    touchdown by ``a t^2 / 2``, so the input that nulls ``cross`` exactly is
    ``|cross| / (accel t^2 / 2)`` (re-solved every tick, capped at 1); with
    the RCS unread, full input.  Pushing toward ``-left`` takes a positive
    ``cross`` down.  Nothing inside ``deadband`` (the prediction's noise).
    Returns a vector of length 0..1, or None for no translation.
    """
    if cross != cross or abs(cross) <= deadband:
        return None
    level = 1.0
    if accel and to_go and to_go > 0.0:
        level = min(1.0, abs(cross) / (0.5 * accel * to_go * to_go))
    return vec.scale(vec.unit(left), -math.copysign(level, cross))


def coast_attitude(v):
    """Retrograde: the unsteered coast leaves the miss to the wing, which acts
    once the descent re-sweep has made the prediction worth acting on."""
    return vec.unit(vec.scale(v, -1.0))


def steering_plane(r, v):
    """Unit normal of the trajectory plane, or None where it is degenerate."""
    n = vec.cross(r, v)
    return vec.unit(n) if vec.norm(n) > 1e-6 else None


def steer_attitude(r, v, steer):
    """The nose direction that flies ``steer``: retrograde, rotated by aoa.

    Exactly the construction ``Environment.probe_lift_slope`` measured the
    slope at, which is what makes the sign of the measurement and the sign of
    the command the same sign.
    """
    side = trajectory.lift_direction(steer, v)
    retrograde = vec.unit(vec.scale(v, -1.0))
    if side is None or vec.norm(retrograde) < 1e-9:
        return retrograde
    return vec.unit(vec.add(vec.scale(retrograde, math.cos(steer.aoa)),
                            vec.scale(side, math.sin(steer.aoa))))


def solve_steer(env, r, v, mass, max_accel, cfg):
    """The angle of attack that lands the prediction on the target.

    Every other correction in this flight spends propellant and an engine
    relight, works in one direction only (see failure 16), and acts for the
    few seconds a burn lasts.  This one is a *trim*: the booster is already
    falling through air thick enough to decelerate it at more than a g, so a
    few degrees off retrograde is a large sideforce acting for the whole
    minute of the descent -- and it is as happy to lengthen the flight as to
    shorten it.

    It is solved rather than tuned.  Propagations at zero and at
    the holdable angle measure how far the touchdown point moves,
    and the answer is the angle that puts the miss at zero, clamped to what
    the vehicle can hold.  There is no gain to fit, and a vehicle with a
    different shape or mass gets a different answer from the same code,
    because the sensitivity is measured in flight.

    With ``AERO_STEER_CROSS`` the lift is also *rolled* about the velocity,
    which makes it a two-axis actuator and gives the **cross-track** miss the
    only authority it has anywhere in this flight.  A third propagation
    measures the out-of-plane sensitivity and the 2x2 is inverted; see
    :func:`steer_for` for why the roll is stored as a rotated normal.

    The inversion is **quadratic**, because the plant is: the sideforce goes
    as ``a|a|`` (see :class:`trajectory.Steer`), so the miss moves as ``a|a|``
    too, and a linear interpolation between two propagations under-commands
    small angles and overshoots large ones.  Solving the shape the vehicle
    actually has costs nothing -- it is the same two propagations and a square
    root.

    Returns ``(steer, prediction)`` with the prediction taken *at the
    commanded angle*, so the caller stores a prediction of the trajectory the
    booster is about to fly rather than one of a trajectory nobody flies.
    Returns ``(None, None)`` when the vehicle cannot steer: no measured
    slope, no usable plane, or no air.
    """
    normal = steering_plane(r, v)
    if normal is None or not cfg.AERO_STEER:
        return None, None
    slope = env.lift_slope_at(vec.norm(v),
                              vec.norm(r) - env.equatorial_radius)
    if not slope:
        return None, None

    def fly(u):
        """Propagate under the steering vector ``u``, in plane coordinates."""
        steer = steer_for(normal, v, u, env.lift_slope_at, holdable(env, cfg))
        prediction = trajectory.predict_landing(env, r, v, mass, max_accel,
                                                cfg, steer=steer)
        long, cross = trajectory.miss_components(env, prediction.position,
                                                 env.target, r)
        return steer, prediction, (long, cross)

    hold = holdable(env, cfg)
    most = hold(vec.norm(v), vec.norm(r) - env.equatorial_radius) \
        if hold is not None else None
    if most is None:
        # Command no more than the vehicle can hold here, and no more than
        # its angle of peak lift (Environment.holdable_aoa): past that, more
        # angle is less lift and the solve's model runs backwards.  An
        # unmeasured limit is not a licence to steer.  The probes below
        # still measure the sensitivity at their own angle, and the
        # propagator caps each of them at the hold limit step by step.
        return None, None
    limit = most
    zero, flat, m0 = fly((0.0, 0.0))
    # Probe at the limit itself: the plant is quadratic in the angle, so any
    # probe inside the holdable range reads the same coefficient, and the
    # largest reads it furthest above the propagation's noise.  Past the
    # limit it would be capped by the propagator and under-read.
    probe = most
    if probe <= 1e-3:
        return zero, flat

    # Do not act on the prediction until the prediction is worth acting on.
    # At the boostback exit it reads about -175 m on this booster and the
    # *unsteered* coast walks it to zero (`qs_north`: coast entry -181, burn
    # entry -5, landed 13 m).  That is not a miss, it is what is left of
    # failure 13's walk, and the drag re-sweep clears it a few tens of
    # seconds later.  The wing, acting immediately, takes it at face value
    # and steers out an error that was going to correct itself -- which is
    # why the first grid of this had it winning by 100 m on the two saves
    # that really do overshoot and losing by 80-100 m on the two that were
    # already landing at 8 m, always to the *short* side.
    # So the wing acts only on a prediction the descent-attitude re-sweep has
    # replaced: an event, not an altitude, so it holds on any booster and any
    # body (breadth2-1112: median 34 v 56 under a 25 km gate, the heavy 24 v 65).
    if getattr(env, "reswept_ut", None) is None:
        return zero, flat

    band = getattr(env, "miss_band", None)
    if band is None or math.hypot(m0[0], m0[1]) <= band:
        # Already on the pad.  Commanding an angle against sub-metre noise is
        # how the solve spends its authority on nothing.
        return zero, flat
    if not cfg.AERO_STEER_CROSS:
        # One axis: the lift stays in the trajectory plane and only the
        # downrange miss is solved.  This is the original law, kept because
        # it is the one with flight time on it.
        _, _, m1 = fly((probe, 0.0))
        per_square = (m1[0] - m0[0]) / (probe * abs(probe))
        if abs(per_square) * limit * limit < band:
            return zero, flat
        wanted = -m0[0] / per_square
        u = (vec.clamp(math.copysign(math.sqrt(abs(wanted)), wanted),
                       -limit, limit), 0.0)
        return verified(env, r, v, mass, max_accel, cfg, normal, u, m0,
                        zero, flat)

    # Two axes.  The wing is a cylinder in crossflow and the vehicle is
    # axisymmetric, so the same measured slope applies whichever way the lift
    # is rolled -- which means the *cross-track* miss has an actuator too, and
    # it is the same one.  Nothing else in this flight can move it but the RCS
    # trim, and the landing burn has ten metres of authority.  Cross-track errors of 15-28 m were the
    # single largest surviving term when this was written, and they sat
    # unchanged from 28 km to the ground.
    #
    # Two more propagations give the 2x2 sensitivity, measured rather than
    # assumed, in exactly the quadratic variable the plant is linear in.
    _, _, m1 = fly((probe, 0.0))
    _, _, m2 = fly((0.0, probe))
    q = probe * abs(probe)                 # the a|a| each probe applied
    # J maps the steering vector's a|a| onto (long, cross).
    j = ((m1[0] - m0[0]) / q, (m2[0] - m0[0]) / q,
         (m1[1] - m0[1]) / q, (m2[1] - m0[1]) / q)
    det = j[0] * j[3] - j[1] * j[2]
    span = limit * limit
    if abs(det) * span * span < band ** 2:
        # Singular or feeble: the two axes are not independent enough to
        # invert, or the air cannot move the landing point from here.  Fall
        # back to the downrange axis alone rather than divide by nothing --
        # a badly conditioned inverse commands a large angle on no evidence.
        per_square = j[0]
        if abs(per_square) * span < band:
            return zero, flat
        wanted = -m0[0] / per_square
        u = (vec.clamp(math.copysign(math.sqrt(abs(wanted)), wanted),
                       -limit, limit), 0.0)
    else:
        # w is the a|a| vector that nulls both components at once.
        w = ((-j[3] * m0[0] + j[1] * m0[1]) / det,
             (j[2] * m0[0] - j[0] * m0[1]) / det)
        mag = math.hypot(w[0], w[1])
        if mag < 1e-12:
            return zero, flat
        # |u| = sqrt(|w|), and u keeps w's direction: the plant is |u| u.
        aoa = min(math.sqrt(mag), limit)
        u = (w[0] / mag * aoa, w[1] / mag * aoa)

    return verified(env, r, v, mass, max_accel, cfg, normal, u, m0,
                    zero, flat)


def verified(env, r, v, mass, max_accel, cfg, normal, u, m0, zero, flat):
    """Command ``u`` only if propagating it actually lands nearer.

    The solve inverts a sensitivity measured by two probes, and near the
    ground that sensitivity collapses: ``wanted = -miss / per_square`` with
    ``per_square`` going to zero answers an *already closed* miss with the
    maximum angle.  LOG399 and LOG403 held 4 deg for the whole coast with the
    prediction pinned at 0 m, saturated at 10 deg on the last tick, and the
    predicted miss went 0 -> -19 m in one step -- the solver manufacturing
    the error it could no longer fix.  LOG402 saturated from 1.2 km and went
    -62 -> -91.

    The guard is free, because the prediction at the commanded angle is
    computed anyway to hand back to the caller.  So compare it against the
    do-nothing prediction that was computed first: if steering is not an
    improvement, halve the angle and look again, and if that still is not,
    hold retrograde.  This is the same shape as ``correction_gradient``
    refusing a burn that would make things worse, and it is why the altitude
    floor above is not needed -- a command that helps at 1.5 km is kept, and
    only one that does not is dropped.
    """
    flat_miss = math.hypot(m0[0], m0[1])
    best = None
    for scale in (1.0, 0.5):
        trial = (u[0] * scale, u[1] * scale)
        steer = steer_for(normal, v, trial, env.lift_slope_at,
                         holdable(env, cfg))
        prediction = trajectory.predict_landing(env, r, v, mass, max_accel,
                                                cfg, steer=steer)
        long, cross = trajectory.miss_components(env, prediction.position,
                                                 env.target, r)
        miss = math.hypot(long, cross)
        if miss <= flat_miss:
            return steer, prediction
        if best is None or miss < best[0]:
            best = (miss, steer, prediction)
    # Every angle tried is worse than flying straight.  Do that.
    return zero, flat


def holdable(env, cfg):
    """The propagator's hold limit (capped at the lift peak), or None."""
    return getattr(env, "holdable_aoa", None)


def steer_for(normal, v, u, slope=None, hold=None):
    """A :class:`trajectory.Steer` flying the plane-coordinate command ``u``.

    ``u`` is ``(in_plane, out_of_plane)`` radians of angle of attack.  The
    out-of-plane part is delivered by *rolling the trajectory-plane normal
    about the velocity*, which is the one construction that keeps the lift
    perpendicular to the velocity for the whole propagation: ``Steer`` holds
    its normal fixed and takes ``cross(normal, v)`` every step, so a normal
    stored already-rotated stays correct as the velocity turns, where a
    stored lift *direction* would not.
    """
    aoa = math.hypot(u[0], u[1])
    if aoa < 1e-12:
        return trajectory.Steer(0.0, normal, slope, hold)
    phi = math.atan2(u[1], u[0])
    axis = vec.unit(v)
    if abs(phi) > 1e-12 and vec.norm(axis) > 0.0:
        normal = vec.quat_rotate(vec.quat_axis_angle(axis, phi), normal)
    return trajectory.Steer(aoa, normal, slope, hold)


@dataclass
class TerminalCommand:
    direction: tuple
    demand: float       # lateral acceleration the law wants, m/s^2


def burn_lift_coefficient(env, r, v, mass, cfg):
    """The burn's sideways lift per radian squared of tilt off retrograde,
    m/s^2: ``q * ClA / m`` from the probed lift curve
    (``Environment.lift_slope_held``) at this Mach, signed as the probe
    signs it (negative: tilting the thrust one way lifts the body the
    other).  ``None`` with ``LANDING_AERO_DIVERT`` off, no mass, or no
    measured curve -- then the divert credits the thrust alone."""
    if not cfg.LANDING_AERO_DIVERT or not mass or mass <= 0.0:
        return None
    held = getattr(env, "lift_slope_held", None)
    if held is None:
        return None
    speed = vec.norm(v)
    altitude = vec.norm(r) - env.equatorial_radius
    try:
        slope = held(speed, altitude)
    except (ValueError, IndexError, ZeroDivisionError):
        return None
    if not slope:
        return None
    rho = env.density(altitude)
    return 0.5 * rho * speed * speed * slope / mass


def tilt_lateral(a, c, t):
    """Sideways acceleration of a tilt ``t`` (rad, signed) off retrograde:
    the thrust's ``a sin t`` and the body's lift ``c t|t|``."""
    return a * math.sin(t) + (c * t * abs(t) if c else 0.0)


def solve_tilt(a, c, demand, limit, samples=48):
    """The tilt inside ``[-limit, limit]`` whose sideways acceleration is
    ``demand``, the least such tilt; when none reaches it, the tilt that
    gives the most.  Supersonic the lift ``c`` can cancel the thrust a few
    degrees off retrograde and reverse it beyond, so the divert is then
    flown by tilting the *other* way, as the coast's aero steer does.
    Returns ``(tilt, authority)``, authority the most any tilt gives."""
    if limit <= 0.0:
        return 0.0, 0.0
    ts = [-limit + 2.0 * limit * i / samples for i in range(samples + 1)]
    fs = [tilt_lateral(a, c, t) for t in ts]
    best = max(range(len(ts)), key=lambda i: fs[i])
    authority = max(0.0, fs[best])
    root = None
    for i in range(len(ts) - 1):
        lo, hi = ts[i], ts[i + 1]
        flo, fhi = fs[i] - demand, fs[i + 1] - demand
        if flo * fhi > 0.0:
            continue
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            fm = tilt_lateral(a, c, mid) - demand
            if (fm <= 0.0) == (flo <= 0.0):
                lo, flo = mid, fm
            else:
                hi = mid
        t = 0.5 * (lo + hi)
        if root is None or abs(t) < abs(root):
            root = t
    return (root if root is not None else ts[best]), authority


def one_law_command(env, r, v, time_to_land, thrust_accel, cfg, zem,
                    tilt_budget_deg, rate, mass=None):
    """The burn's one steering law, ignition to touchdown .

    The base is ``trajectory.base_direction``: retrograde, scaled toward the
    vertical as far as its implied sideways null rate would outrun the
    attitude ``rate``.  On top, the zero-effort-miss law on the propagator's
    prediction of that same base -- ``ZEM_GAIN / t_go^2`` -- with its gain
    capped at ``rate^2``: the aiming loop never closes faster than the nose
    that executes it, so near the ground it fades into a slow position hold
    rather than switching off at a height; and it never asks for a sideways
    speed the craft could not brake to zero by touchdown.  The tilt the miss
    adds is held inside ``LANDING_MAX_TILT_DEG`` and the throttle's budget, as the divert's
    was.
    """
    up = vec.unit(r)
    a = max(0.0, thrust_accel)
    base = trajectory.base_direction(v, up, a, rate)
    if zem is None or a <= 0.0:
        return TerminalCommand(base, 0.0)
    t_go = max(1e-3, time_to_land)
    gain = min(cfg.LANDING_ZEM_GAIN / (t_go * t_go), rate * rate)
    accel = vec.scale(vec.project_out(zem, up), -gain)
    # The aim, with the base's null, is a = rate * (v_aim - v): it asks for a
    # sideways speed v_aim = accel / rate.  No more may be asked for than
    # the craft can still lose before touchdown: the base nulls sideways
    # speed exponentially at ``rate``, so v_aim decays to v_aim exp(-rate
    # t_go) by contact, and that must be inside what the craft survives
    # sideways (``trajectory.lateral_allowance``).  Nor more than its lateral
    # authority, thrust * sin(LANDING_MAX_TILT_DEG), can brake in the time
    # left after one nose response.  Capped only by the authority, the aim
    # landed fin2 crossing the pad at 12-22 m/s (nose-1115).
    lift = burn_lift_coefficient(env, r, v, mass, cfg)
    limit = cfg.LANDING_MAX_TILT_DEG
    if tilt_budget_deg is not None:
        limit = min(limit, tilt_budget_deg)
    limit = math.radians(max(0.0, limit))
    if lift is None:
        authority = a * math.sin(math.radians(cfg.LANDING_MAX_TILT_DEG))
    else:
        authority = solve_tilt(a, lift, float("inf"), limit)[1]
    # The sideways speed is braked late, subsonic, where the lift is gone:
    # size what may be asked for on the thrust's own authority, never the
    # lift-boosted one, or the divert builds a speed it cannot straighten
    # out of before touchdown.
    braking = min(authority, a * math.sin(limit))
    v_max = min(braking * max(0.0, t_go - 1.0 / rate
                              - stand_up_time(cfg, rate)),
                trajectory.lateral_allowance(env, cfg)
                * math.exp(min(50.0, rate * t_go)))
    asked = vec.norm(accel) / rate
    if asked > v_max:
        accel = vec.scale(accel, v_max / asked)
    accel = vec.project_out(accel, base)
    demand = vec.norm(accel)
    if demand < 1e-6:
        return TerminalCommand(base, 0.0)
    if lift is None:
        tilt = min(math.atan2(demand, a), limit)
    else:
        tilt = solve_tilt(a, lift, demand, limit)[0]
    return TerminalCommand(
        vec.unit(vec.add(base, vec.scale(vec.unit(accel), math.tan(tilt)))),
        demand)


def terminal_command(env, r, v, height, time_to_land, thrust_accel, cfg,
                     zem=None, tilt_budget_deg=None, null_rate=None,
                     mass=None):
    """The landing burn's command, ignition to touchdown: ``one_law_command``,
    stood up over the last nose response (``1 / rate``) inside a cone that
    narrows at touchdown to ``SURVIVAL_SHARE`` of the legs' measured tip
    angle (atan(support margin / centre-of-mass height); none known: upright) -- a lean at contact
    pries a booster over on its legs (tip1-1018, LOG12086).  With no attitude
    rate read there is no bandwidth to steer at, so it flies retrograde.
    """
    retrograde = vec.unit(vec.scale(v, -1.0))
    if vec.norm(retrograde) < 1e-9:
        return TerminalCommand(vec.unit(r), 0.0)
    if not cfg.LANDING_TERMINAL_GUIDANCE or not null_rate:
        return TerminalCommand(retrograde, 0.0)
    if not cfg.LANDING_BURN_AIM:
        zem = None
    cmd = one_law_command(env, r, v, time_to_land, thrust_accel, cfg, zem,
                          tilt_budget_deg, null_rate, mass=mass)
    floor = cfg.SURVIVAL_SHARE * (getattr(env, "tip_angle_deg", None) or 0.0)
    if cfg.LANDING_STAND_UP:
        allowed = stand_up_cone(cfg, null_rate, time_to_land, floor)
        if allowed >= 90.0:
            return cmd
        return TerminalCommand(
            within_cone(cmd.direction, vec.unit(r), allowed), cmd.demand)
    frac = min(1.0, max(0.0, time_to_land) * null_rate)
    if frac >= 1.0:
        return cmd
    full = cfg.LANDING_MAX_TILT_DEG * 2.0
    return TerminalCommand(
        within_cone(cmd.direction, vec.unit(r), floor + (full - floor) * frac),
        cmd.demand)


def nose_rate(cfg, null_rate):
    """The nose's measured closing rate (1/s, one over its lag behind the
    command), recovered from the lateral null's rate, which is
    ``LOOP_BANDWIDTH_SHARE`` of it (``Autoland.null_rate``)."""
    return null_rate / cfg.LOOP_BANDWIDTH_SHARE


def stand_up_cone(cfg, null_rate, time_to_land, floor):
    """deg off vertical the command may lean with ``time_to_land`` left, so
    that the nose is upright (within ``floor``) at contact: the command moves
    at most the burn's slew, ``lam * LANDING_MAX_TILT_DEG`` deg/s
    (``Autoland.burn_slew_rate``), and the nose follows it one lag ``1/lam``
    behind.  So the lean allowed is the slew times the time left after that
    lag -- a ramp, no switch at a height."""
    lam = nose_rate(cfg, null_rate)
    return floor + cfg.LANDING_MAX_TILT_DEG * max(
        0.0, lam * max(0.0, time_to_land) - 1.0)


def stand_up_time(cfg, rate):
    """Seconds the stand-up takes from a full divert's tilt
    (``stand_up_cone`` inverted at ``LANDING_MAX_TILT_DEG``): the sideways
    speed must be braked before it starts.  0 with ``LANDING_STAND_UP``
    off."""
    if not cfg.LANDING_STAND_UP:
        return 0.0
    return 2.0 / nose_rate(cfg, rate)


def slew(previous, target, max_deg):
    """``target``, or the direction ``max_deg`` from ``previous`` toward it.

    A command that steps is a command no vehicle flies: the landing burn's
    reversed 60 deg in a tick as the sideways speed changed sign, and the
    booster chased it into an oscillation (cmd1-1017, cw= 57-115 deg/s)."""
    if previous is None:
        return tuple(target)
    a, b = vec.unit(previous), vec.unit(target)
    angle = vec.angle_between(a, b)
    if angle <= max_deg or angle < 1e-9:
        return tuple(b)
    axis = vec.cross(a, b)
    if vec.norm(axis) < 1e-9:
        return tuple(b)
    q = vec.quat_axis_angle(vec.unit(axis), math.radians(max_deg))
    return vec.unit(vec.quat_rotate(q, a))


def within_cone(direction, axis, max_deg):
    """``direction`` rotated toward ``axis`` until it is within ``max_deg``.

    A direction on the far side of the axis (more than 90 deg off) is
    answered with the axis itself: a booster going up is stood upright, not
    leaned over at the cone's edge toward wherever it happens to be moving.
    """
    d, a = vec.unit(direction), vec.unit(axis)
    cos = vec.dot(d, a)
    if cos <= 0.0:
        return a
    if cos >= math.cos(math.radians(max_deg)):
        return d
    side = vec.unit(vec.project_out(d, a))
    if vec.norm(side) < 1e-9:
        return a
    t = math.radians(max_deg)
    return vec.unit(vec.add(vec.scale(a, math.cos(t)), vec.scale(side, math.sin(t))))


@dataclass
class LandingThrottle:
    throttle: float
    target_speed: float
    height: float


def landing_throttle(env, r, v, max_accel, cfg, height=None):
    """Track a constant-deceleration descent profile down to touchdown.

    ``height`` is the distance the legs still have to fall; pass the
    terrain-relative one so the profile flares against the actual ground
    rather than against the pad's radius.
    """
    radius = vec.norm(r)
    if height is None:
        height = radius - env.target_radius
    # The law lives in trajectory.landing_command so that the propagator can
    # fly the same profile the booster does; this is the vector wrapper.
    throttle, target_speed = landing_command(env, radius, vec.norm(v), height,
                                             max_accel, cfg,
                                             vec.dot(v, vec.unit(r)))
    return LandingThrottle(throttle, target_speed, height)


def transport_roof(roof, from_nose, to_nose, normal):
    """``roof`` carried by the rotation that takes ``from_nose`` to
    ``to_nose`` -- what the roof becomes if the turn is pure pitch.

    The rotation is about ``from_nose x to_nose``; a rotation by theta and
    by theta - 360 is the same rotation, so the answer does not depend on
    which way round the flip goes.  Near-opposite noses leave that axis
    undefined, and ``normal`` (the trajectory plane's) stands in for it.
    """
    a, b = vec.unit(from_nose), vec.unit(to_nose)
    axis = vec.cross(a, b)
    angle = math.atan2(vec.norm(axis), vec.dot(a, b))
    if vec.norm(axis) < 1e-6:
        if vec.dot(a, b) > 0.0:
            return roof
        axis = vec.project_out(normal, a)
        if vec.norm(axis) < 1e-9:
            return roof
    q = vec.quat_axis_angle(vec.unit(axis), angle)
    return vec.quat_rotate(q, roof)


def roll_reference(r, v, aim, previous=None, min_plane_speed=0.0,
                   previous_nose=None):
    """The ``up`` vector to roll the vehicle's roof toward, body frame.

    Left to itself the autopilot only damps the roll rate (``target_roll``
    NaN), so the booster keeps whatever roll the flip happened to leave it
    with and drifts from there.  Every steering command this script issues
    lives in the plane of the trajectory -- the boostback aim, the coast
    bias, the correction tilt -- so the useful orientation is the one that
    puts the vehicle's pitch axis in that plane: grid fins working in the
    plane they are being asked to steer in, and legs and fins in the same
    place relative to the airstream every flight rather than wherever the
    flip stopped.

    So the roof is rolled toward the in-plane direction perpendicular to the
    nose, on the side away from the body.  Taking it as ``n x nose``, with
    ``n`` the trajectory-plane normal, keeps it well defined for *every* nose
    direction the profile commands, including straight up during the landing
    burn -- which the autopilot's default zenith reference is not.

    kRPC frames are left-handed, so the sign of the cross product is not
    obvious; it is resolved by testing against the zenith rather than
    assumed, the same way ``Environment._measure_omega`` does.  That test
    only picks the *first* reference, though.  ``n x nose`` reverses with the
    nose, and the nose sweeps through the zenith during the landing burn and
    through 180 deg during the flip, so re-deciding the sign from the zenith
    every tick would snap the reference to the other side mid-flight and
    command a barrel roll.  Once seeded, the side is carried forward from
    ``previous`` instead: the roof stays where it is and the vehicle pitches
    around it, which is the whole point of holding a roll.

    ``previous_nose``: the nose ``previous`` was the roof for.  Given, the
    side is taken from ``previous`` *carried through the turn* (see
    ``transport_roof``) rather than ``previous`` itself, so a flip over the
    top -- which takes a roof that pointed up to one that points down -- is
    commanded as pitch alone and not as pitch plus a half roll.

    ``min_plane_speed``: below this horizontal speed the trajectory plane is
    whatever the last few m/s of drift make it -- in the landing burn it
    swings with them and the roof command whips round -- so the roof is
    carried from ``previous``, made perpendicular to the new nose.
    """
    nose = vec.unit(aim)
    if previous is not None and previous_nose is not None:
        previous = transport_roof(previous, previous_nose, nose,
                                  vec.cross(r, v))
    if previous is not None and min_plane_speed > 0.0:
        horizontal = vec.project_out(v, vec.unit(r))
        if vec.norm(horizontal) < min_plane_speed:
            up = vec.project_out(previous, nose)
            if vec.norm(up) > 1e-6:
                return vec.unit(up)
    normal = vec.cross(r, v)
    if vec.norm(normal) < 1e-6:            # straight up or down: no plane
        normal = vec.cross(r, previous or (1.0, 0.0, 0.0))
    up = vec.project_out(vec.cross(normal, nose), nose)
    if vec.norm(up) < 1e-6:                # nose along the normal: no roof
        return previous
    up = vec.unit(up)

    if previous is None:
        side = vec.dot(up, vec.unit(r))     # seed: roof away from the body
    else:
        side = vec.dot(up, previous)        # then: stay on the side we are on
    if abs(side) < 1e-9:                    # exactly 90 deg from last tick
        return up
    return up if side > 0.0 else vec.scale(up, -1.0)


def flip_through_vertical(current, target, up, min_turn_deg, past_deg):
    """The attitude to command *now* so a big turn is two pitches through
    the vertical.

    While the nose is on the far side of the vertical from the target
    (their horizontal parts point apart), command the zenith tilted
    ``past_deg`` toward the target's side: the nose pitches up in its own
    vertical plane, crosses the vertical, and the moment its horizontal part
    points the target's way the routing releases and the nose pitches down
    to the target in the target's vertical plane.  With the roof held in
    the trajectory plane (``roll_reference``) both legs are pitch alone --
    no yaw through the airstream, no rolling while the engines swing.

    Turns under ``min_turn_deg`` are commanded directly.  Memoryless:
    ``current`` is the real nose, so it releases itself.
    """
    nose = vec.unit(current)
    aim = vec.unit(target)
    zenith = vec.unit(up)
    if vec.angle_between(nose, aim) < min_turn_deg:
        return tuple(aim)
    h_aim = vec.project_out(aim, zenith)
    if vec.norm(h_aim) < 1e-6:
        return tuple(aim)               # the target is the vertical
    h_aim = vec.unit(h_aim)
    if vec.dot(vec.project_out(nose, zenith), h_aim) > 0.0:
        return tuple(aim)               # already over: pitch down to it
    tilt = math.radians(past_deg)
    return vec.unit(vec.add(vec.scale(zenith, math.cos(tilt)),
                            vec.scale(h_aim, math.sin(tilt))))

