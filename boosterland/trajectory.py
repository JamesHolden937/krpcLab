"""Local trajectory propagation and landing-point prediction.

The propagator runs entirely in the body's rotating reference frame -- the
same frame the atmosphere rotates with, so ``v`` is already air-relative and
drag needs no extra term.  Working in a rotating frame means the Coriolis and
centrifugal accelerations are explicit; ``Environment.omega`` supplies the
rotation vector with a sign measured in-game.

``predict_landing`` does not report the ballistic impact point.  It stops the
integration where the landing burn would have to start, then adds the downrange
distance the booster still covers while that burn kills its velocity.  Because
braking sheds horizontal speed, the result lands *short* of where an unpowered
rock would hit -- by kilometres for a fast, weakly-braking booster -- so
boostback aims at the point the vehicle will actually touch down on.
"""

import math
from dataclasses import dataclass

from . import vec


@dataclass
class Prediction:
    position: tuple          # predicted touchdown point, body frame
    time_to_land: float      # seconds from now until touchdown
    burn_time: float         # seconds of landing burn included above
    burn_altitude: float     # height above pad where the burn would start
    entry_speed: float       # speed at the start of that burn
    powered: bool            # False => pure ballistic impact (no thrust)
    steps: int
    profile: tuple = ()      # (speed, altitude) along the way down


def gravity_at(env, r):
    d = vec.norm(r)
    return vec.scale(r, -env.mu / (d * d * d))


def drag_lookup(env, drag_area=None):
    """Cd*A as a function of ``(speed, altitude)``.

    A constant is a bad model of a booster's drag: Cd has a transonic hump,
    and one propagation runs the vehicle through Mach 3 to Mach 0.  Passing an
    explicit ``drag_area`` pins it anyway, which is what the gradient probes
    and the offline tests want -- they compare two propagations and only need
    both to see the same air.
    """
    if drag_area is not None:
        return lambda speed, altitude: drag_area
    return env.drag_area_at


@dataclass
class Steer:
    """A commanded angle of attack, and the plane it is flown in.

    The booster falls nose-retrograde through 30 km of thickening air at
    200-500 m/s with its grid fins out.  Held exactly on retrograde that is a
    pure brake; held a few degrees off it, it is a *wing* -- and a wing is the
    one actuator in this flight that works in both directions, costs no
    propellant and no engine relights, and acts for the whole minute during
    which the miss is already known.

    ``aoa`` is signed in radians: positive tilts the nose off retrograde
    toward ``+cross(normal, v)``, and the lift is applied along that same
    direction scaled by a *measured* coefficient, so the sign convention
    cancels the way ``Environment._measure_omega`` makes the frame's
    handedness cancel.

    The force goes as ``aoa * |aoa|``, not as ``aoa``.  That is measured, not
    assumed: this booster's sideforce per radian varies eightfold between 2
    and 20 degrees while its sideforce per radian *squared* holds within 9%
    over the range the vehicle is ever asked to fly.  A cylinder in crossflow
    is not a wing with a linear lift curve.  ``normal`` is the trajectory plane's normal, held fixed for a
    propagation: it barely moves during a coast, and recomputing it as the
    velocity turns vertical on final is a degeneracy for nothing.
    """
    aoa: float
    normal: tuple
    slope: object = None        # (speed, altitude) -> Cl*A per radian


def lift_direction(steer, v):
    """Perpendicular to ``v``, in the steering plane; the way lift acts."""
    if steer is None:
        return None
    d = vec.cross(steer.normal, v)
    return vec.unit(d) if vec.norm(d) > 1e-9 else None


def acceleration(env, r, v, mass, drag, steer=None):
    """Total acceleration in the rotating body frame.

    ``drag`` is a ``(speed, altitude) -> Cd*A`` lookup; see :func:`drag_lookup`.
    ``steer`` adds the lift of a commanded angle of attack; see :class:`Steer`.
    """
    a = gravity_at(env, r)
    if mass > 0.0:
        altitude = vec.norm(r) - env.equatorial_radius
        rho = env.density(altitude)
        if rho > 0.0:
            speed = vec.norm(v)
            drag_area = drag(speed, altitude) if speed > 0.0 else 0.0
            if speed > 0.0 and drag_area > 0.0:
                # a_drag = -(1/2) rho Cd A |v| v / m
                q = 0.5 * rho * drag_area * speed / mass
                a = vec.add(a, vec.scale(v, -q))
            if speed > 0.0 and steer is not None and steer.aoa and steer.slope:
                slope = steer.slope(speed, altitude)
                side = lift_direction(steer, v)
                if slope and side is not None:
                    # a_lift = (1/2) rho |v|^2 K alpha|alpha| / m
                    lift = (0.5 * rho * speed * speed * slope
                            * steer.aoa * abs(steer.aoa) / mass)
                    a = vec.add(a, vec.scale(side, lift))
    w = env.omega
    a = vec.add(a, vec.scale(vec.cross(w, v), -2.0))       # Coriolis
    a = vec.add(a, vec.scale(vec.cross(w, vec.cross(w, r)), -1.0))  # centrifugal
    return a


def stopping_distance(speed, net_accel):
    if net_accel <= 0.0:
        return float("inf")
    return speed * speed / (2.0 * net_accel)


def landing_burn_state(env, r, v, max_accel, cfg, height=None, miss=0.0):
    """(height, height needed to stop, net decel) right now.

    ``height`` defaults to the height above the pad's radius, but the caller
    should pass the terrain-relative leg height when it has one: the pad radius
    is only the ground under the booster when it is coming down on the pad.

    ``miss`` buys an early start, in proportion to how far off the prediction
    is, so a burn that would otherwise begin at the last possible metre has
    time left to translate.  ``LANDING_DIVERT_LEAD`` defaults to 0 because the
    landing burn no longer translates: with ``LANDING_MAX_TILT_DEG`` at 0 it
    thrusts straight retrograde, so an early trigger buys nothing and only
    moves the phase change away from the moment the engines light.  LOG7 showed
    what that costs in legibility -- LANDING_BURN was announced at 2272 m and
    the throttle then sat at zero for eight seconds and 1900 m of fall.  Raise
    it alongside the tilt if the divert is ever re-enabled.
    """
    if height is None:
        height = vec.norm(r) - env.target_radius
    speed = vec.norm(v)
    g = env.mu / (vec.norm(r) ** 2)
    net = max_accel * cfg.LANDING_THROTTLE_CAP - g
    needed = stopping_distance(speed, net) * cfg.SUICIDE_MARGIN
    needed += speed * cfg.SUICIDE_REACTION_S
    needed += min(miss * cfg.LANDING_DIVERT_LEAD, cfg.LANDING_MAX_EARLY_M)
    return height, needed, net


def landing_command(env, radius, speed, height, max_accel, cfg,
                    vertical_speed=None):
    """(throttle, target speed) for the constant-deceleration descent profile.

    The law itself, as a scalar function of where the vehicle is and how fast
    it is going.  ``guidance.landing_throttle`` is the vector wrapper the
    control loop calls; ``predict_landing`` integrates against this same
    function, so what the propagator flies and what the booster flies cannot
    drift apart -- the same reason the suicide-burn trigger and the
    prediction share ``landing_burn_state``.

    ``vertical_speed`` is signed, positive up, and is what stops the burn
    flying the booster back into the sky; see the guard at the bottom.  Both
    callers have the velocity vector, so both pass it.
    """
    g = env.mu / (radius * radius)
    net = max(0.1, max_accel * cfg.LANDING_THROTTLE_CAP - g)

    remaining = max(0.0, height - cfg.TOUCHDOWN_ALT_M)
    target_speed = math.sqrt(2.0 * net * remaining) + cfg.TOUCHDOWN_SPEED
    # A pure constant-deceleration profile is still doing several m/s a couple
    # of metres up -- it assumes the vehicle can brake at full authority right
    # into the ground.  Taper the last stretch into a gentle constant-rate
    # descent so the legs meet the pad at walking pace.
    target_speed = min(target_speed,
                       cfg.TOUCHDOWN_SPEED + cfg.FINAL_APPROACH_RATE * remaining)
    if max_accel <= 0.0:
        return 0.0, target_speed

    # Feedforward: the deceleration that reaches touchdown speed exactly.
    span = max(1.0, remaining)
    a_required = (speed * speed - cfg.TOUCHDOWN_SPEED ** 2) / (2.0 * span)
    throttle = (a_required + g) / max_accel
    throttle += cfg.LANDING_THROTTLE_KP * (speed - target_speed)

    # A booster near the ground must never be thrusting while it is going
    # *up*.  ``span`` is floored at a metre, so a height that reads near zero
    # while the vehicle is still doing several m/s asks for
    # ``(speed^2 - 4) / 2`` of deceleration -- 26 m/s^2 at 7 m/s -- and the
    # cap is the only thing between that and the engine.  A nearly empty
    # booster has 3.5 g in hand, so what comes out is not a hard landing, it
    # is a *launch*: LOG429 was on a clean final approach at 11 m and 0.22
    # throttle, went to 0.95, and left the ground at 101 m/s.  It coasted to
    # 951 m, fell back, burned again, and landed 1106 m from the pad having
    # spent 3.8 t -- from a burn that was 132 m out when it started.
    #
    # Which input lied does not matter and is not knowable from one tick: a
    # height is a terrain sample offset by a bounding box, and bad ones have
    # ended five flights now.  What is unambiguous is the sign of the
    # vertical speed.  Ascending this low, the burn has already done its job
    # and any thrust at all is making things worse, so there is nothing to
    # trade away by cutting it -- the vehicle falls back onto a profile that
    # knows what to do with it.
    if (vertical_speed is not None
            and vertical_speed >= 0.0
            and height <= cfg.LANDING_GOAROUND_ALT_M):
        throttle = 0.0
    # The same floor bites a metre up without any ascent to give it away, and
    # there it is worse, because the vehicle is about to touch.  LOG446 was
    # textbook -- 22 m at 0.30 throttle, 5.7 m at 0.30, `pad=17` throughout --
    # and then at **1.6 m, descending at 3.5 m/s**, the feedforward asked for
    # 0.95.  A 3.5 g booster does not land under that; it tips, and the lit
    # engine drove it sideways at 30 m/s.  It spent the next twenty seconds
    # circling the pad between 12 and 19 m, throttle flapping against the
    # guard above, and ran the tanks dry.
    #
    # A vehicle this low and already this slow cannot need more than a hover:
    # it is within a second of the ground at walking pace, and the extra
    # thrust has nowhere to go but up.  A *fast* arrival is left alone --
    # that one still needs everything it has, and the cap is not what is
    # wrong with it.
    if (height <= cfg.LANDING_FLARE_ALT_M
            and speed <= cfg.LANDING_FLARE_SPEED):
        hover = g / max_accel
        throttle = min(throttle, hover + cfg.LANDING_FLARE_MARGIN)
    return vec.clamp(throttle, 0.0, cfg.LANDING_THROTTLE_CAP), target_speed


def _rk4(env, r, v, mass, drag, dt, thrust=None, steer=None):
    """One RK4 step of (r, v)' = (v, a(r, v)).

    Semi-implicit Euler is what this replaced, and the step size it needs is
    not a detail: the drag term goes as v^2 through a density that changes by
    a factor of e every 5 km, so a first-order step accumulates a real bias
    over a hundred seconds of descent rather than noise.  Measured against
    LOG243 -- a flight that landed 109 m from the pad -- the prediction from
    20 km read 48 m at ``PREDICT_DT_ATMO`` 0.5 and 106 m at 0.05, with the
    vehicle doing nothing in between: the walk the coast was famous for was
    the integrator, not the air.  RK4 gets the same answer as a converged
    Euler at a step ten times longer, which is what makes it affordable in
    the control loop; see CLAUDE.md failure 16.

    ``thrust`` is an extra ``(r, v) -> acceleration`` term, so the landing
    burn integrates on the same scheme as the coast.
    """
    def deriv(rr, vv):
        a = acceleration(env, rr, vv, mass, drag, steer)
        if thrust is not None:
            a = vec.add(a, thrust(rr, vv))
        return vv, a

    k1r, k1v = deriv(r, v)
    k2r, k2v = deriv(vec.add(r, vec.scale(k1r, dt * 0.5)),
                     vec.add(v, vec.scale(k1v, dt * 0.5)))
    k3r, k3v = deriv(vec.add(r, vec.scale(k2r, dt * 0.5)),
                     vec.add(v, vec.scale(k2v, dt * 0.5)))
    k4r, k4v = deriv(vec.add(r, vec.scale(k3r, dt)),
                     vec.add(v, vec.scale(k3v, dt)))
    w = dt / 6.0
    r = vec.add(r, vec.scale(vec.add(vec.add(k1r, vec.scale(k2r, 2.0)),
                                     vec.add(vec.scale(k3r, 2.0), k4r)), w))
    v = vec.add(v, vec.scale(vec.add(vec.add(k1v, vec.scale(k2v, 2.0)),
                                     vec.add(vec.scale(k3v, 2.0), k4v)), w))
    return r, v


def _step(env, r, v, mass, drag, dt, cfg, thrust=None, steer=None):
    """Advance one step with the configured integrator."""
    if cfg.PREDICT_RK4:
        return _rk4(env, r, v, mass, drag, dt, thrust, steer)
    a = acceleration(env, r, v, mass, drag, steer)
    if thrust is not None:
        a = vec.add(a, thrust(r, v))
    v = vec.add(v, vec.scale(a, dt))
    r = vec.add(r, vec.scale(v, dt))
    return r, v


def predict_landing(env, r0, v0, mass, max_accel, cfg, drag_area=None,
                    steer=None):
    """Propagate to touchdown, including the landing burn's downrange creep.

    ``steer`` is the angle of attack the vehicle is going to fly (see
    :class:`Steer`).  It has to be passed in rather than assumed, for the same
    reason ``landing_command`` is shared with the control loop: a prediction
    that does not include the steering the booster is about to fly is a
    prediction of a trajectory nobody flies.
    """
    drag = drag_lookup(env, drag_area)
    r = tuple(r0)
    v = tuple(v0)
    t = 0.0
    steps = 0
    powered = False
    net = 0.0
    profile = []

    while t < cfg.PREDICT_MAX_TIME_S:
        radius = vec.norm(r)
        height = radius - env.target_radius
        up = vec.scale(r, 1.0 / radius)
        speed = vec.norm(v)
        descending = vec.dot(v, up) < 0.0

        if max_accel > 0.0:
            # Stop where the *trigger* will stop, not where a theoretical
            # full-throttle stop would: landing_burn_state adds
            # SUICIDE_MARGIN and the reaction allowance, so the real burn
            # starts a fifth of its stopping distance higher.  Predicting the
            # tighter burn quietly under-states how far the vehicle creeps
            # downrange while braking -- which is the whole reason this
            # function exists.
            _, needed, net = landing_burn_state(
                env, r, v, max_accel, cfg, height=height)
            if descending and height <= needed:
                powered = True
                break
        if height <= 0.0:
            break

        altitude = radius - env.equatorial_radius
        dt = (cfg.PREDICT_DT_ATMO if altitude < env.atmosphere_depth
              else cfg.PREDICT_DT_VACUUM)
        if speed > 1.0:
            dt = min(dt, max(0.05, 0.25 * height / speed))

        # Where the booster will be when it is going this fast.  Cd*A is not
        # a function of Mach alone -- KSP scales drag by a pseudo-Reynolds
        # term as well, so the same Mach reads ~15-20% lower down low than it
        # does at 30 km -- and a probe is free to ask about any position it
        # likes.  Recording the descent's own (speed, altitude) pairs lets
        # each bin be probed in the air it is going to be *used* in rather
        # than the air the booster happens to be in; see
        # Environment.probe_altitude.
        if descending:
            profile.append((speed, altitude))

        r, v = _step(env, r, v, mass, drag, dt, cfg, steer=steer)
        t += dt
        steps += 1

    radius = vec.norm(r)
    speed = vec.norm(v)
    burn_time = 0.0
    burn_altitude = radius - env.target_radius
    if powered and net > 0.0:
        # Fly the burn rather than estimating it.  The closed form this
        # replaced -- horizontal speed times half of speed/net -- assumed a
        # full-throttle stop with no drag and no gravity turn, and it landed
        # the prediction hundreds of metres short on a fast entry, which is
        # then exactly where boostback and CORRECTION aimed.  A suicide burn
        # is only a minute long, so integrating it costs a couple of hundred
        # steps and removes the guess.
        # No ``steer`` here, deliberately.  The landing burn holds straight
        # anti-velocity -- ``LANDING_MAX_TILT_DEG`` is 0 since failure 5 --
        # so the vehicle makes no lift during it, and a propagation that
        # flew the coast's angle of attack on down through the burn would be
        # predicting a wing nobody is flying.  Left in, it costs the whole
        # benefit on a short coast: the 60 km entry state predicted a miss of
        # 0 m on every tick of the descent and landed 119 m out.
        r, v, burn_time, burn_steps = _fly_landing_burn(
            env, r, v, mass, max_accel, cfg, drag)
        steps += burn_steps

    return Prediction(
        position=vec.scale(vec.unit(r), env.target_radius),
        time_to_land=t + burn_time,
        burn_time=burn_time,
        burn_altitude=burn_altitude if powered else 0.0,
        entry_speed=speed,
        powered=powered,
        steps=steps,
        profile=tuple(profile),
    )


def _fly_landing_burn(env, r, v, mass, max_accel, cfg, drag, steer=None):
    """Integrate the suicide burn itself: thrust anti-velocity on the profile.

    Returns the touchdown state, how long the burn took and the steps spent.
    Gravity and drag are the same terms the ballistic phase uses, so the
    trajectory bends toward the vertical as the horizontal speed is eaten --
    which is the part a closed form gets wrong, and the part that decides
    where the booster actually ends up.
    """
    t = 0.0
    steps = 0
    while t < cfg.PREDICT_MAX_TIME_S:
        radius = vec.norm(r)
        height = radius - env.target_radius
        speed = vec.norm(v)
        if height <= 0.0 or speed < cfg.TOUCHDOWN_SPEED:
            break

        throttle, _ = landing_command(env, radius, speed, height, max_accel,
                                      cfg, vec.dot(v, vec.unit(r)))
        dt = min(cfg.PREDICT_DT_ATMO, max(0.05, 0.25 * height / speed))

        r, v = _step(env, r, v, mass, drag, dt, cfg,
                     lambda rr, vv: vec.scale(vec.unit(vv),
                                              -throttle * max_accel),
                     steer=steer)
        t += dt
        steps += 1
    return r, v, t, steps


def miss_gradient(env, r, v, mass, aim, cfg, probe_dv, max_accel=0.0):
    """Change in predicted miss per m/s of dv spent along ``aim``.

    Negative means burning that way walks the landing point toward the pad;
    positive means it walks away.  Steering that is constrained to point near
    retrograde -- which is all a booster can hold in thick air -- can only ever
    make the vehicle land *shorter*, so for an undershoot every reachable
    direction is the wrong one.  Without this check the correction burns anyway
    and digs in: a 25 m/s coast kick took the predicted miss from 2591 m to
    17277 m over three burns, each one "correcting" harder than the last.

    ``max_accel`` at 0 makes both propagations ballistic, which is cheap and
    was the original behaviour.  **It is also why low corrections backfired.**
    A ballistic propagation from 30 km is a fair description of what the
    booster is about to do, because the landing burn is a small tail on a long
    coast.  From 10 km it is not a description of anything: nearly all of the
    remaining flight *is* the burn, and the burn eats horizontal speed, so a
    ballistic probe measures the sensitivity of a trajectory the vehicle will
    never fly.  Two round-2 flights show the cost -- corrections entered at
    9-11 km reported ``grad`` of 12.5 and 14.5 (comfortably "helping") on
    every tick while the miss ran 400 -> 527 and 445 -> 585, and both were cut
    by ``correction_making_it_worse`` rather than by the gradient that was
    supposed to see it coming.  Passing the vehicle's real acceleration makes
    both propagations fly the burn, which is what the vehicle is going to do.
    """
    direction = vec.unit(aim)
    if vec.norm(direction) < 1e-9 or probe_dv <= 0.0:
        return 0.0
    here = predict_landing(env, r, v, mass, max_accel, cfg)
    there = predict_landing(env, r, vec.add(v, vec.scale(direction, probe_dv)),
                            mass, max_accel, cfg)
    return (surface_distance(env, there.position, env.target)
            - surface_distance(env, here.position, env.target)) / probe_dv


def miss_vector(env, prediction):
    """Predicted touchdown minus target, as a vector in the body frame."""
    return vec.sub(prediction.position, env.target)


def surface_distance(env, a, b):
    """Great-circle distance between two points at pad radius."""
    ua, ub = vec.unit(a), vec.unit(b)
    c = max(-1.0, min(1.0, vec.dot(ua, ub)))
    return math.acos(c) * env.target_radius


def miss_components(env, predicted, target, from_position):
    """The predicted miss as ``(long, cross)`` metres, signed.

    ``surface_distance`` cannot tell an overshoot from an undershoot, and that
    ambiguity has cost this project real sessions: a systematic 105 m westward
    bias read as imprecision for most of one, and a correction burn's gradient
    sign can only be argued about with a signed miss in front of you.  So the
    error is resolved into the two directions that mean something.

    ``long`` is along the booster's approach -- the horizontal direction from
    where it is now toward the target -- and is **positive when the predicted
    touchdown is beyond the target**, i.e. the booster flies past.  ``cross``
    is the perpendicular, positive to the left of that approach as seen from
    outside the body (the sign is a convention; the magnitude is what matters,
    and the sign is at least consistent within a flight).

    All three points are treated as horizontal vectors about the target's own
    up direction, so the answer is a ground-plane offset rather than a chord
    through the body.
    """
    up = vec.unit(target)
    err = vec.project_out(vec.sub(predicted, target), up)
    approach = vec.project_out(vec.sub(target, from_position), up)
    if vec.norm(approach) < 1e-6:
        return vec.norm(err), 0.0
    along = vec.unit(approach)
    left = vec.cross(up, along)
    return vec.dot(err, along), vec.dot(err, left)
