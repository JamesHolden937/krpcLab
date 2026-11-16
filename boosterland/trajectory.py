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

from common import vec
from common.geo import gravity_at, surface_distance  # noqa: F401


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




def drag_lookup(env, drag_area=None, scale=1.0):
    """Cd*A as a function of ``(speed, altitude)``.

    A constant is a bad model of a booster's drag: Cd has a transonic hump,
    and one propagation runs the vehicle through Mach 3 to Mach 0.  Passing an
    explicit ``drag_area`` pins it anyway, which is what the gradient probes
    and the offline tests want -- they compare two propagations and only need
    both to see the same air.
    """
    if drag_area is not None:
        return lambda speed, altitude: drag_area * scale
    if scale != 1.0:
        return lambda speed, altitude: env.drag_area_at(speed, altitude) * scale
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
    hold: object = None         # (speed, altitude) -> the most the vehicle
                                # can hold, radians, or None; the lift is
                                # credited at no more than that


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
                    aoa = steer.aoa
                    if steer.hold is not None:
                        most = steer.hold(speed, altitude)
                        if most is not None and abs(aoa) > most:
                            aoa = math.copysign(most, aoa)
                    lift = (0.5 * rho * speed * speed * slope
                            * aoa * abs(aoa) / mass)
                    a = vec.add(a, vec.scale(side, lift))
    w = env.omega
    a = vec.add(a, vec.scale(vec.cross(w, v), -2.0))       # Coriolis
    a = vec.add(a, vec.scale(vec.cross(w, vec.cross(w, r)), -1.0))  # centrifugal
    return a


def stopping_distance(speed, net_accel):
    if net_accel <= 0.0:
        return float("inf")
    return speed * speed / (2.0 * net_accel)


def planned_net(max_accel, g, cfg):
    """The net deceleration the vertical profile plans with.

    The profile takes only ``cos(LANDING_MAX_TILT_DEG)`` of the cap, so the throttle it asks
    for leaves the divert its full tilt (``acos(throttle / cap)``) down to
    the ground: planned at the cap, the throttle pinned from ~600 m and the
    burn had no lateral authority for its last 600 m whatever its TWR.
    """
    share = math.cos(math.radians(cfg.LANDING_MAX_TILT_DEG))
    return max_accel * cfg.LANDING_THROTTLE_CAP * share - g


def leg_clearance(env):
    """Centre of mass to the bottom of the vessel (``env.leg_clearance``,
    measured off the bounding box).  The real trigger and throttle fly the
    *legs'* height (``Snapshot.landing_height``); a propagation flown on the
    centre of mass lit the burn later and landed that much lower, ~11 m on
    the fin boosters, and read 3-5 s of time-to-go at contact."""
    c = getattr(env, "leg_clearance", None)
    return c if c and c > 0.0 else 0.0


def one_law_rate(env, cfg):
    """The attitude rate (1/s) the burn's law closes at (``env.attitude_rate``,
    ``Autoland.null_rate``), or None while unread."""
    rate = getattr(env, "attitude_rate", None)
    return rate if rate and rate > 0.0 else None


# KSP's own Part.crashTolerance before a part's cfg sets one (Assembly-CSharp,
# Part's constructor): the tolerance of a craft whose parts cannot be read.
GAME_DEFAULT_CRASH_TOLERANCE = 9.0


def touchdown_speed(env, cfg):
    """The descent rate the profile lands at: ``TOUCHDOWN_SPEED_SHARE`` of
    what the craft survives (``env.touchdown_tolerance``: the weakest part
    that touches, or the legs' suspension when lower), read in flight."""
    tol = getattr(env, "touchdown_tolerance", None)
    if not tol or tol <= 0.0:
        tol = GAME_DEFAULT_CRASH_TOLERANCE
    return cfg.TOUCHDOWN_SPEED_SHARE * tol


def base_direction(v, up, thrust_accel, rate):
    """The landing burn's braking direction.

    Retrograde nulls the sideways speed for free, at a rate of
    ``thrust / speed`` per second: negligible at 700 m/s, ~3/s at the last
    few m/s, far faster than the nose can follow (the dv3-1115 last-20 m
    oscillation).  So the angle retrograde makes with the vertical is scaled
    by ``min(1, rate * speed / thrust)``: exactly retrograde while its
    implied rate is under the attitude ``rate``, and continuously a vertical
    reference nulling the sideways speed at ``rate`` once it is not (small
    angle: lateral accel = thrust * angle = rate * v_h).  Nothing switches at
    a height.  Going up, the same rate from vertical; with no thrust,
    retrograde (or vertical when climbing), ready for the throttle.
    """
    speed = vec.norm(v)
    if speed < 1e-6:
        return tuple(up)
    retro = vec.scale(v, -1.0 / speed)
    lateral = vec.project_out(v, up)
    h = vec.norm(lateral)
    descending = vec.dot(v, up) < 0.0
    if thrust_accel <= 1e-6:
        return retro if descending else tuple(up)
    if h < 1e-6:
        return tuple(up)
    if descending:
        gamma = math.radians(vec.angle_between(retro, up))
        theta = gamma * min(1.0, rate * speed / thrust_accel)
    else:
        theta = math.atan(min(1.0 / speed, rate / thrust_accel) * h)
    side = vec.scale(lateral, -1.0 / h)
    return vec.unit(vec.add(vec.scale(up, math.cos(theta)),
                            vec.scale(side, math.sin(theta))))


def throttle_gain(env, cfg):
    """The descent-rate trim's gain, 1/s (m/s^2 per m/s of speed error):
    ``LOOP_BANDWIDTH_SHARE`` of the throttle's own bandwidth, one over its
    measured reaction (``reaction_time``: spool-up plus one control tick).
    A tick of 0.04 game-s and instant engines give 6.25 /s."""
    lag = reaction_time(env)
    if lag <= 0.0:
        lag = cfg.LOOP_SLEEP_S
    return cfg.LOOP_BANDWIDTH_SHARE / lag


def reaction_time(env):
    """Seconds from the trigger to braking thrust, as measured in flight: the
    engines' spool-up (``env.engine_spool_s``, timed at the boostback's
    ignition) plus one control interval (``env.control_interval``: the
    trigger is only looked at once a tick).  Unread parts count 0."""
    return ((getattr(env, "engine_spool_s", None) or 0.0)
            + (getattr(env, "control_interval", None) or 0.0))


def profile_speed(env, cfg, speed, vertical_speed, max_accel):
    """The speed the descent profile is flown on: the descent rate plus the
    share of the sideways speed the thrust along the profile brakes.

    The profile is a *descent* profile.  On total speed, a booster sliding
    15 m/s sideways at 20 m reads 15 m/s against a 3 m/s target, opens to the
    cap (which zeroes the divert's tilt budget), climbs, is cut by the
    go-around guard, falls, and repeats -- every tick, on both boosters,
    drifting 100-750 m (dlaw1-1016).  Thrust along the vertical cannot take
    out sideways speed; the steering law can, given throttle that is neither
    0 nor the cap.  Higher up the sideways speed has to be braked like any
    other (the closed-loop sim hit at 54 m/s on descent rate alone): all of
    it while the base is retrograde, the rest left to the law once the base
    nulls it at ``rate`` (base_direction).  With no rate read, the base is
    retrograde: all of it."""
    if not cfg.LANDING_THROTTLE_VERTICAL or vertical_speed is None:
        return speed
    rate = one_law_rate(env, cfg)
    down = max(0.0, -vertical_speed)
    side_sq = max(0.0, speed * speed - vertical_speed * vertical_speed)
    w = (1.0 if rate is None
         else min(1.0, rate * speed / max(1e-6, max_accel)))
    return math.sqrt(down * down + w * w * side_sq)


def descent_profile(env, cfg, max_accel, g):
    """``(v_td, k, net, v_join, h_join)``: the descent profile's shape.

    Constant deceleration ``net`` from high up, joined where ``k v`` reaches
    ``net`` to the taper ``v = v_td + k h``, which decelerates at ``k v`` and
    lands at the touchdown speed.  k is the attitude rate the lateral null
    closes at (``one_law_rate``): the descent slows over the same time
    constant the sideways speed is nulled in, so both arrive together.  With
    no rate measured, no taper (k None): constant deceleration to v_td."""
    net = max(0.1, planned_net(max_accel, g, cfg))
    v_td = touchdown_speed(env, cfg)
    k = one_law_rate(env, cfg)
    if k is not None:
        # ... and no faster than the descent-rate trim that flies it can
        # follow (the same cascade share of its gain): a nose fast enough to
        # read as instant left no taper at all, and the sim hit at 8 m/s.
        k = min(k, cfg.LOOP_BANDWIDTH_SHARE * throttle_gain(env, cfg))
    if k is None:
        return v_td, None, net, v_td, 0.0
    v_join = max(v_td, net / k)
    return v_td, k, net, v_join, (v_join - v_td) / k


def profile_height(profile, speed):
    """The height above the cutoff at which the profile asks for ``speed``:
    the profile read backwards (0 at or under the touchdown speed)."""
    v_td, k, net, v_join, h_join = profile
    if speed <= v_td:
        return 0.0
    if k is not None and speed <= v_join:
        return (speed - v_td) / k
    return h_join + (speed * speed - v_join * v_join) / (2.0 * net)


def landing_burn_state(env, r, v, max_accel, cfg, height=None):
    """(height, height needed to stop, net decel) right now.

    The burn is lit where the vehicle meets its own descent profile -- the
    profile ``landing_command`` flies, read backwards at the speed it would
    fly it on -- plus the distance covered in the measured reaction time.
    Lit any higher, the profile's target is above the vehicle's speed and the
    throttle sits at zero until the two meet: a fixed 18% margin and 1.5 s
    allowance coasted ~1.5 s inside the burn on every flight (LOG17300).

    ``height`` defaults to the height above the pad's radius, but the caller
    should pass the terrain-relative leg height when it has one: the pad radius
    is only the ground under the booster when it is coming down on the pad.
    """
    radius = vec.norm(r)
    if height is None:
        height = radius - env.target_radius
    speed = vec.norm(v)
    g = env.mu / (radius * radius)
    profile = descent_profile(env, cfg, max_accel, g)
    flown = profile_speed(env, cfg, speed, vec.dot(v, r) / radius, max_accel)
    needed = (profile_height(profile, flown) + cfg.TOUCHDOWN_ALT_M
              + speed * reaction_time(env))
    return height, needed, profile[2]


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
    speed = profile_speed(env, cfg, speed, vertical_speed, max_accel)
    v_td, k, net, v_join, h_join = descent_profile(env, cfg, max_accel, g)
    remaining = max(0.0, height - cfg.TOUCHDOWN_ALT_M)
    taper = k is not None and remaining <= h_join
    if taper:
        target_speed = v_td + k * remaining
    else:
        target_speed = math.sqrt(v_join * v_join
                                 + 2.0 * net * (remaining - h_join))
    if max_accel <= 0.0:
        return 0.0, target_speed

    # Feedforward: the deceleration that reaches touchdown speed exactly,
    # above the taper; in it, the taper's own deceleration k * v_target,
    # which falls to k * v_td at the ground.  The exact form divides by the
    # height left, and a metre up at a few m/s it asked for full thrust on a
    # vehicle that wanted a hover (LOG446: 0.95 at 1.6 m, 3.5 m/s).  The
    # trim below takes any speed error, at the same gain at any height.
    if taper:
        a_required = k * target_speed
    else:
        span = max(1e-3, remaining)
        a_required = (speed * speed - v_td ** 2) / (2.0 * span)
    throttle = (a_required + g) / max_accel
    # The trim is an acceleration, so the loop gain is the same whatever the
    # thrust-to-weight: a throttle gain fitted to one booster is a loop that
    # oscillates on a stronger one.
    throttle += throttle_gain(env, cfg) * (speed - target_speed) / max_accel

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
    #
    # At any height: climbing, the burn's base is within the attitude rate of
    # vertical (base_direction), so thrust only adds climb.
    if vertical_speed is not None and vertical_speed >= 0.0:
        throttle = 0.0
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
                    steer=None, drag_scale=1.0):
    """Propagate to touchdown, including the landing burn's downrange creep.

    ``steer`` is the angle of attack the vehicle is going to fly (see
    :class:`Steer`).  It has to be passed in rather than assumed, for the same
    reason ``landing_command`` is shared with the control loop: a prediction
    that does not include the steering the booster is about to fly is a
    prediction of a trajectory nobody flies.

    """
    drag = drag_lookup(env, drag_area, drag_scale)
    r = tuple(r0)
    v = tuple(v0)
    t = 0.0
    steps = 0
    powered = False
    net = 0.0
    profile = []

    while t < cfg.PREDICT_MAX_TIME_S:
        radius = vec.norm(r)
        height = radius - env.target_radius - leg_clearance(env)
        up = vec.scale(r, 1.0 / radius)
        speed = vec.norm(v)
        descending = vec.dot(v, up) < 0.0

        if max_accel > 0.0:
            # Light where the *trigger* will light (landing_burn_state: the
            # profile read backwards, plus the reaction time), so the
            # prediction flies the burn that will be flown.
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
    burn_altitude = radius - env.target_radius - leg_clearance(env)
    if powered and net > 0.0:
        # Fly the burn rather than estimating it.  The closed form this
        # replaced -- horizontal speed times half of speed/net -- assumed a
        # full-throttle stop with no drag and no gravity turn, and it landed
        # the prediction hundreds of metres short on a fast entry, which is
        # then exactly where boostback and CORRECTION aimed.  A suicide burn
        # is only a minute long, so integrating it costs a couple of hundred
        # steps and removes the guess.
        # No ``steer`` here, deliberately.  The landing burn flies within
        # ``LANDING_MAX_TILT_DEG`` of retrograde, close enough that the
        # vehicle makes little lift during it, and a propagation that
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


def _fly_landing_burn(env, r, v, mass, max_accel, cfg, drag):
    """Integrate the suicide burn itself: thrust along the law's base
    direction, on the profile.

    Returns the touchdown state, how long the burn took and the steps spent.
    Gravity and drag are the same terms the ballistic phase uses, so the
    trajectory bends toward the vertical as the horizontal speed is eaten --
    which is the part a closed form gets wrong, and the part that decides
    where the booster actually ends up.  The nose is taken to be on the
    command: the coast unwinds its angle of attack before ignition
    (``COAST_UNWIND_LAGS``) so that it is.
    """
    t = 0.0
    steps = 0
    rate = one_law_rate(env, cfg)

    def thrust_dir(vv, rr, accel):
        if rate is not None:
            return base_direction(vv, vec.unit(rr), accel, rate)
        return vec.scale(vec.unit(vv), -1.0)

    while t < cfg.PREDICT_MAX_TIME_S:
        radius = vec.norm(r)
        height = radius - env.target_radius - leg_clearance(env)
        speed = vec.norm(v)
        if height <= 0.0 or speed < touchdown_speed(env, cfg):
            break

        throttle, _ = landing_command(env, radius, speed, height, max_accel,
                                      cfg, vec.dot(v, vec.unit(r)))
        dt = min(cfg.PREDICT_DT_ATMO, max(0.05, 0.25 * height / speed))
        accel = throttle * max_accel
        r, v = _step(env, r, v, mass, drag, dt, cfg,
                     lambda rr, vv: vec.scale(thrust_dir(vv, rr, accel),
                                              accel))
        t += dt
        steps += 1
    return r, v, t, steps


def miss_vector(env, prediction):
    """Predicted touchdown minus target, as a vector in the body frame."""
    return vec.sub(prediction.position, env.target)




def miss_axes(target, from_position):
    """``(along, left)``: the unit ground directions ``miss_components``
    resolves the miss onto, or None where the approach is undefined."""
    up = vec.unit(target)
    approach = vec.project_out(vec.sub(target, from_position), up)
    if vec.norm(approach) < 1e-6:
        return None
    along = vec.unit(approach)
    return along, vec.cross(up, along)


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
