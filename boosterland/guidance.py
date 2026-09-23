"""Steering and throttle laws for each phase of the return.

Boostback logic: changing horizontal velocity by ``dv`` shifts the predicted
touchdown point by roughly ``dv * time_to_land``, so the burn direction is the
horizontal miss vector reversed and the remaining dv is the miss divided by the
time left in the flight.  Both are re-derived from a fresh prediction every
tick, which makes the burn self-correcting -- it simply stops when the
prediction lands inside ``BOOSTBACK_TOLERANCE_M``.
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


def boostback_solution(env, r, v, mass, max_accel, isp, prediction, cfg):
    miss = miss_vector(env, prediction)
    up = vec.unit(r)
    horizontal_miss = vec.project_out(miss, up)
    distance = vec.norm(horizontal_miss)

    # Burn opposite the miss, optionally pitched up off the local horizon.
    if distance > 1e-6:
        aim = vec.scale(vec.unit(horizontal_miss), -1.0)
    else:
        aim = vec.scale(vec.unit(v), -1.0)
    pitch = math.radians(cfg.BOOSTBACK_PITCH_DEG)
    if pitch:
        aim = vec.unit(vec.add(vec.scale(aim, math.cos(pitch)),
                               vec.scale(up, math.sin(pitch))))

    time_left = max(1.0, prediction.time_to_land)
    dv = distance / time_left

    burn_time = 0.0
    if max_accel > 0.0 and isp > 0.0:
        exhaust = isp * G0
        thrust = max_accel * mass
        mass_flow = thrust / exhaust
        burn_time = mass / mass_flow * (1.0 - math.exp(-dv / exhaust))

    if dv >= cfg.BOOSTBACK_TAPER_DV:
        throttle = 1.0
    else:
        throttle = vec.clamp(dv / cfg.BOOSTBACK_TAPER_DV,
                             cfg.BOOSTBACK_MIN_THROTTLE, 1.0)

    return BoostbackSolution(aim=aim, miss=miss, miss_distance=distance,
                             dv=dv, burn_time=burn_time, throttle=throttle)


def _tilt_from_retrograde(env, r, v, miss, max_tilt_deg, gain):
    """Retrograde, tilted to push the predicted landing point onto the pad."""
    retrograde = vec.unit(vec.scale(v, -1.0))
    if vec.norm(retrograde) < 1e-9:
        return vec.unit(r)
    distance = vec.norm(vec.project_out(miss, vec.unit(r)))
    if distance < 1e-3:
        return retrograde
    correction = vec.unit(vec.project_out(vec.scale(miss, -1.0), retrograde))
    tilt = math.radians(min(max_tilt_deg, gain * distance))
    return vec.unit(vec.add(retrograde, vec.scale(correction, math.tan(tilt))))


def coast_attitude(env, r, v, prediction, cfg):
    """Retrograde hold during the ballistic arc, with a light aero bias."""
    return _tilt_from_retrograde(env, r, v, miss_vector(env, prediction),
                                 cfg.COAST_MAX_TILT_DEG, cfg.COAST_STEER_GAIN)


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
    ``AERO_STEER_SOLVE_AOA_DEG`` measure how far the touchdown point moves,
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
        steer = steer_for(normal, v, u, env.lift_slope_at)
        prediction = trajectory.predict_landing(env, r, v, mass, max_accel,
                                                cfg, steer=steer)
        long, cross = trajectory.miss_components(env, prediction.position,
                                                 env.target, r)
        return steer, prediction, (long, cross)

    limit = math.radians(cfg.AERO_STEER_MAX_AOA_DEG)
    probe = math.radians(cfg.AERO_STEER_SOLVE_AOA_DEG)
    zero, flat, m0 = fly((0.0, 0.0))

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
    if vec.norm(r) - env.target_radius > cfg.AERO_STEER_MAX_ALT_M:
        return zero, flat

    # An altitude floor is available but defaults off, because it is the
    # wrong shape of guard and that was measured.  Cutting the steering below
    # 2 km cost `quicksave` 13 m (5/5 m became 18/19) while the flights it
    # was meant to save were saved by the check at the bottom of this
    # function instead.  The last two kilometres do real work; what they must
    # not do is saturate.
    if vec.norm(r) - env.target_radius <= cfg.AERO_STEER_MIN_ALT_M:
        return zero, flat
    if math.hypot(m0[0], m0[1]) <= cfg.AERO_STEER_DEADBAND_M:
        # Already on the pad.  Commanding an angle against sub-metre noise is
        # how the solve spends its authority on nothing.
        return zero, flat
    if not cfg.AERO_STEER_CROSS:
        # One axis: the lift stays in the trajectory plane and only the
        # downrange miss is solved.  This is the original law, kept because
        # it is the one with flight time on it.
        _, _, m1 = fly((probe, 0.0))
        per_square = (m1[0] - m0[0]) / (probe * abs(probe))
        if abs(per_square) * limit * limit < cfg.AERO_STEER_MIN_AUTHORITY_M:
            return zero, flat
        wanted = -m0[0] / per_square
        u = (vec.clamp(math.copysign(math.sqrt(abs(wanted)), wanted),
                       -limit, limit), 0.0)
        return verified(env, r, v, mass, max_accel, cfg, normal, u, m0,
                        zero, flat)

    # Two axes.  The wing is a cylinder in crossflow and the vehicle is
    # axisymmetric, so the same measured slope applies whichever way the lift
    # is rolled -- which means the *cross-track* miss has an actuator too, and
    # it is the same one.  Nothing else in this flight can move it: CORRECTION
    # tilts off retrograde, which is a downrange lever, and the landing burn
    # has ten metres of authority.  Cross-track errors of 15-28 m were the
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
    if abs(det) * span * span < cfg.AERO_STEER_MIN_AUTHORITY_M ** 2:
        # Singular or feeble: the two axes are not independent enough to
        # invert, or the air cannot move the landing point from here.  Fall
        # back to the downrange axis alone rather than divide by nothing --
        # a badly conditioned inverse commands a large angle on no evidence.
        per_square = j[0]
        if abs(per_square) * span < cfg.AERO_STEER_MIN_AUTHORITY_M:
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
        steer = steer_for(normal, v, trial, env.lift_slope_at)
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


def steer_for(normal, v, u, slope=None):
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
        return trajectory.Steer(0.0, normal, slope)
    phi = math.atan2(u[1], u[0])
    axis = vec.unit(v)
    if abs(phi) > 1e-12 and vec.norm(axis) > 0.0:
        normal = vec.quat_rotate(vec.quat_axis_angle(axis, phi), normal)
    return trajectory.Steer(aoa, normal, slope)


def correction_attitude(env, r, v, prediction, cfg, height=None):
    """Steer a mid-flight correction *relative to retrograde*.

    ...except up high, where it steers like boostback and can therefore
    correct in **both** directions.  That asymmetry is worth the extra
    parameter: a tilt bounded off retrograde can only ever make the booster
    land *shorter*, so an undershoot is a miss no phase after boostback can
    touch, and `qs_steep` lands ~130 m short for exactly that reason -- the
    prediction says so at 34 km, with the vehicle in thin air and holding
    fuel, and CORRECTION declines because ``miss_gradient`` correctly reports
    that its own steering would make things worse.  Above
    ``CORRECTION_FREE_AIM_ALT_M`` the air is thin enough that the vehicle can
    hold the horizontal attitude that LOG7 proved it cannot hold at 8 km
    (failure 6), so the aim there is boostback's: opposite the horizontal
    miss, which points prograde for an undershoot and retrograde for an
    overshoot.  The gradient gate is unchanged and still refuses a burn that
    does not help, so this widens what the phase *can* fix without widening
    what it will attempt.  0 disables it and every correction steers off
    retrograde.

    Boostback aims along the horizontal miss vector, which is the right answer
    when the booster is above the atmosphere with time to turn.  It is the
    wrong answer during the coast: by then the vehicle is falling steeply
    through thick air, retrograde is 45-70 deg off horizontal, and a purely
    horizontal command is an attitude it cannot hold.  LOG7 shows the cost --
    CORRECTION held the throttle closed for 36 s waiting for an alignment that
    was never going to happen, timed out, and let the miss grow 341 -> 966 m.

    Tilting off retrograde instead is always reachable, and it is not much
    weaker: retrograde thrust already works against a downrange overshoot, so
    the tilt only has to bias it.
    """
    miss = miss_vector(env, prediction)
    free = cfg.CORRECTION_FREE_AIM_ALT_M
    if free > 0.0 and height is not None and height >= free:
        horizontal = vec.project_out(miss, vec.unit(r))
        if vec.norm(horizontal) > 1e-6:
            return vec.scale(vec.unit(horizontal), -1.0)
    return _tilt_from_retrograde(env, r, v, miss,
                                 cfg.CORRECTION_MAX_TILT_DEG,
                                 cfg.CORRECTION_TILT_GAIN)


@dataclass
class TerminalCommand:
    direction: tuple
    demand: float       # lateral acceleration the law wants, m/s^2


def terminal_attitude(env, r, v, height, time_to_land, thrust_accel, cfg):
    """Just the direction; see :func:`terminal_command`."""
    return terminal_command(env, r, v, height, time_to_land,
                            thrust_accel, cfg).direction


def terminal_command(env, r, v, height, time_to_land, thrust_accel, cfg,
                     zem=None, tilt_budget_deg=None):
    """Steer the landing burn onto the pad, arriving with no sideways speed.

    The old divert was position feedback alone -- tilt proportional to the miss
    -- and that is why it had to be switched off.  A law with no velocity term
    cannot do anything *but* arrive moving: LOG7 held 20 deg for nine seconds,
    closed 36 m of a 1190 m miss, and put the booster on the ground doing
    10 m/s sideways, which tipped it over (failure 5).  Turning that gain back
    up would reproduce it exactly.

    So this targets both ends of the trajectory at once.  Horizontally the
    booster is a double integrator, and the minimum-energy steering that
    brings *position and velocity* to zero together at ``t_go`` is

        a = -(6/t_go^2) e - (4/t_go) v_h

    with ``e`` the horizontal offset from the pad and ``v_h`` the horizontal
    velocity.  The velocity term is the whole point: as the offset closes, the
    law is already pulling the lateral speed out, so the vehicle arrives over
    the pad *stopped* rather than crossing it.  Both gains come from the
    closed-form solution rather than being tuned, which is why they are 6 and
    4 and not round numbers.

    Three guards, and each is load-bearing:

    * ``t_go`` is floored.  Both gains go to infinity at touchdown, and an
      unbounded command in the last second is a vehicle tipping over on the
      pad -- the LOG7 failure by a different route.
    * the tilt is bounded by ``LANDING_MAX_TILT_DEG`` **and** by
      ``tilt_budget_deg``, the angle the throttle can actually pay for.  The
      fixed bound alone is not enough and that was measured, twice.  Thrust
      tilted by theta brakes with only its cosine, so the caller has to open
      the throttle by 1/cos(theta) to stand still -- and when the profile is
      already asking for the cap there is nothing left to open.  The tilt then
      comes straight out of the braking, silently, for as long as it steers.
      Three round-4 flights arrived at 17.6, 35.7 and 43.7 m/s carrying
      17-42 m/s sideways, one of them still descending at 36 m/s a metre and a
      half up: that is failure 5 again, by way of the throttle cap.  The
      caller passes ``acos(demanded / cap)``, which goes to zero exactly when
      the vehicle needs all of its thrust to stop.
    * below ``LANDING_NULL_ALT_M`` the position term is dropped and the law
      steers on the lateral *velocity* alone, and it keeps doing that all the
      way down to ``LANDING_DIVERT_MIN_ALT_M``.  This is the half that was
      missing, and it is what the in-game divert died of: the previous shape
      chased the offset down to 15 m and then stopped steering outright, so
      whatever lateral speed the chase had built up was still on the vehicle
      when the legs arrived -- 3.4 m/s at 16 m up on ``quicksave``, which
      topples a booster, and the still-lit engine then threw it back to
      950 m.  Aiming and stopping are different jobs and the last stretch
      belongs to the second one: the offset is whatever it is by 250 m up,
      and landing upright beats landing close (failure 5, which is the same
      lesson one layer down).  ``LANDING_DIVERT_MIN_ALT_M`` is now only the
      floor under *that*, held for the last few metres so the vehicle is
      standing on retrograde when it touches.

    By default the error is measured from the *vehicle's own position*, so
    this stays a closed loop on the real state and does not care that the
    propagator flies the burn without a divert.  Pass ``zem`` -- the predicted
    touchdown point minus the pad -- to close the loop on the propagator's
    answer instead; see the ``zem`` branch below for why that is the better
    signal on a booster still doing 200 m/s through thick air, and
    ``LANDING_DIVERT_ON_PREDICTION`` for the flag that supplies it.

    Two things about ``thrust_accel`` that cost a sweep to find.  It is the
    acceleration the vehicle is *actually making* -- throttle times the best
    it has -- not its maximum.  Sizing the tilt against the maximum quietly
    under-delivers whenever the throttle is part open: on the 60 km entry
    state the law asked for 9.6 m/s^2 of lateral at ``thr=0.46`` and got 4.2,
    so the offset never closed and the command sat on the bound the whole
    way down.  And when the throttle is shut there is no steering to be had
    at any attitude, so below ``LANDING_DIVERT_MIN_ACCEL`` this does not
    pretend otherwise.

    ``LANDING_DIVERT_MAX_ALT_M`` is the other half.  The law above is a flat
    double integrator: no drag, no gravity turn, no curvature.  That is a
    good description of the last kilometre and a poor one of a burn that
    starts at 27 km and spends half of it coasting at zero throttle, where
    the predictor -- which models all three -- is the better authority.  So
    high up the burn stays retrograde and the trajectory is left to the
    propagator that shaped it.
    """
    retrograde = vec.unit(vec.scale(v, -1.0))
    if vec.norm(retrograde) < 1e-9:
        return TerminalCommand(vec.unit(r), 0.0)
    if not cfg.LANDING_TERMINAL_GUIDANCE:
        return TerminalCommand(retrograde, 0.0)
    if not (cfg.LANDING_DIVERT_MIN_ALT_M < height
            <= cfg.LANDING_DIVERT_MAX_ALT_M):
        return TerminalCommand(retrograde, 0.0)

    up = vec.unit(r)
    offset = vec.project_out(vec.sub(r, env.target), up)    # where we are, vs the pad
    lateral = vec.project_out(v, up)
    t_go = max(cfg.LANDING_TGO_MIN_S, time_to_land)

    if height <= cfg.LANDING_NULL_ALT_M:
        # The last stretch is for stopping, not for aiming.  Drop the position
        # term entirely -- whichever signal was supplying it -- and drive the
        # lateral speed to zero, so the legs arrive over whatever point the
        # aiming left them above rather than crossing it sideways.  The gain
        # is the same 4/t_go the full law uses, so this is not a separate
        # controller, it is the same one with its other half switched off.
        if vec.norm(lateral) <= cfg.LANDING_HVEL_TOL:
            return TerminalCommand(retrograde, 0.0)
        accel = vec.scale(lateral, -cfg.LANDING_ZEV_GAIN / t_go)
    elif zem is None:
        accel = vec.add(vec.scale(offset, -cfg.LANDING_ZEM_GAIN / (t_go * t_go)),
                        vec.scale(lateral, -cfg.LANDING_ZEV_GAIN / t_go))
    else:
        # ``zem`` is the propagator's own answer to "where does this land if I
        # steer no further" -- the predicted touchdown point minus the pad.
        # The two terms above are a *linearisation* of that same question:
        # position plus velocity times the time left, with no drag, no gravity
        # turn, and no model of the braking profile that is about to eat most
        # of the horizontal speed.  On a booster arriving at 200 m/s through
        # thick air those three are not small corrections, and the propagator
        # models all of them and is called every tick anyway.
        #
        # The velocity term goes with them.  It exists in the linear law to
        # stop the vehicle crossing the pad still moving; here the prediction
        # already flies the burn to a stop, so its residual lateral speed is
        # inside the zero-effort miss rather than beside it, and adding a
        # separate term for it would count it twice.
        accel = vec.scale(vec.project_out(zem, up),
                          -cfg.LANDING_ZEM_GAIN / (t_go * t_go))
    accel = vec.project_out(accel, retrograde)   # only the part that tilts us
    demand = vec.norm(accel)
    if demand < 1e-6:
        return TerminalCommand(retrograde, 0.0)
    # The demand is a property of the trajectory, so it is reported even when
    # there is no thrust to meet it -- that is exactly the case the caller
    # needs to know about, because it can open the throttle and create some.
    if thrust_accel < cfg.LANDING_DIVERT_MIN_ACCEL:
        return TerminalCommand(retrograde, demand)

    limit = cfg.LANDING_MAX_TILT_DEG
    if tilt_budget_deg is not None:
        limit = min(limit, tilt_budget_deg)
    tilt = math.atan2(demand, thrust_accel)
    tilt = min(tilt, math.radians(max(0.0, limit)))
    return TerminalCommand(
        vec.unit(vec.add(retrograde,
                         vec.scale(vec.unit(accel), math.tan(tilt)))), demand)


def landing_attitude(env, r, v, prediction, cfg):
    """Thrust straight anti-velocity.

    ``LANDING_MAX_TILT_DEG`` defaults to 0, i.e. pure retrograde, because
    tilting off it during the landing burn made things worse in every way
    (LOG7).  A tilt of 20 deg spent nine seconds trying to close a 1190 m miss,
    closed 36 m of it, and put the booster on the ground doing 10 m/s
    *sideways* -- the thrust it diverted was thrust it needed to stop with, and
    the lateral velocity it built up had nothing left to cancel it.  Retrograde
    thrust cancels horizontal velocity as a side effect of stopping, which is
    the behaviour that keeps the vehicle on its legs.

    The miss is boostback's and CORRECTION's job; by the landing burn there are
    seconds left and no authority to spare.  Raise the tilt to re-enable the
    divert if a vehicle ever has the margin for it.
    """
    return _tilt_from_retrograde(env, r, v, miss_vector(env, prediction),
                                 cfg.LANDING_MAX_TILT_DEG,
                                 cfg.LANDING_DIVERT_GAIN)


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


def roll_reference(r, v, aim, previous=None):
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
    """
    nose = vec.unit(aim)
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


def flip_waypoint(current, target, up, min_turn_deg):
    """The attitude to command *now* so a big turn goes over the top.

    kRPC's autopilot rotates along the shortest arc between where the nose is
    and where it is told to point.  For a small change that is the only
    sensible path, but the boostback-to-coast and boostback-to-correction
    transitions are 150 deg or more, and the shortest arc between two
    near-opposite vectors is a coin toss that can just as easily swing the
    nose down through the airstream as up over the top.  Nose-down is the
    wrong half: it points the engines into the flow, and it is the half the
    vehicle has least authority to come back from.

    So for turns beyond ``min_turn_deg`` the arc is checked, and if it dives
    below the horizon the turn is routed through the far side instead, by
    commanding the up-side point on the same great circle first.  The nose
    then passes as near the vertical as these two vectors allow -- through it
    when they straddle it, over the top of the arc when they do not -- and
    the routing releases itself as soon as the remaining turn is small enough
    to be unambiguous, because ``current`` is the vehicle's real attitude.

    Costs about 20 deg of extra travel on a 170 deg flip.  ``min_turn_deg``
    is what keeps that off the small corrections, where the direct arc is
    already the closest approach to the vertical the endpoints allow.
    """
    nose = vec.unit(current)
    aim = vec.unit(target)
    zenith = vec.unit(up)
    if vec.angle_between(nose, aim) < min_turn_deg:
        return tuple(aim)

    middle = vec.add(nose, aim)
    if vec.norm(middle) < 1e-6:
        # Exactly opposite: every path is the same length, so pick the plane
        # that contains the zenith and climb.
        climb = vec.project_out(zenith, nose)
        return tuple(aim) if vec.norm(climb) < 1e-6 else vec.unit(climb)

    middle = vec.unit(middle)
    if vec.dot(middle, zenith) >= 0.0:
        return tuple(aim)               # the short way already goes over
    return vec.scale(middle, -1.0)      # the same arc, the other way round
