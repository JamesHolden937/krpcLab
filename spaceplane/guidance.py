"""Steering and throttle laws.  Pure functions of a state and a config.

Three laws, and they hand over to each other at altitudes rather than at
misses:

``solve_glide``   entry and glide: the angle of attack and bank that put the
                  predicted gate crossing on the gate, solved against the
                  propagator rather than tuned.
``approach``      from the gate in: a geometric path to the threshold, flown
                  on the angle of attack that carries the weight.
``flare``         the last fifteen metres, which are their own problem.

The boundary between the first two is the thing that makes the whole scheme
safe, and it is boosterland's "Aim, then stop" one phase earlier: a solve that
runs all the way to the tarmac is solving a manoeuvre it has no model of.
"""
import math
from dataclasses import replace

from common import vec
from . import airframe, trajectory
from .trajectory import Steer


# -- entry and glide -------------------------------------------------------
def _fly(env, r, v, mass, cfg, end, gate, alpha, bank):
    """Propagate at one pair of controls and report ``(high, cross)``.

    ``long`` is the along-track miss where the arc crosses the gate's
    altitude, positive when the vehicle will arrive past the gate, and it is
    what the glide is solved on.  ``cross`` is the lateral miss at the same
    point, which the bank's sign takes.
    """
    # ``GLIDE_BANK_SWEEP``: the glide solve flies the planned slow reversal
    # (``solve_glide(plan=...)`` posts it for the duration of one solve).
    steer = Steer(alpha=alpha, bank=bank, cfg=cfg, mass=mass,
                  plan=getattr(env, "bank_plan", None))
    prediction = trajectory.predict(env, r, v, mass, cfg, steer=steer,
                                    gate=gate, end=end,
                                    target_radius=vec.norm(gate))
    return steer, prediction, (prediction.long, prediction.cross)


def max_range(env, r, v, mass, cfg, end, gate, alpha0, bank0, floor, top,
              flat):
    """The command that goes furthest, for when the gate is out of reach.

    ``solve_glide`` nulls a *miss*, and a miss only exists if the arc gets to
    the gate's altitude at all.  When it does not -- when the propagation
    runs out of sky first -- there is no miss to null, and what the solve did
    about that was return the command it was handed.  Measured, that is not a
    graceful degradation, it is the vehicle letting go of the controls at the
    exact moment it is losing the flight:

    ``logs/LOG1877`` holds ``long`` inside a few tens of metres all the way
    down from the interface, reaches 23.5 km, and then flies **thirty
    consecutive ticks at alpha 18.0 and bank +2.3** -- byte-identical
    commands -- while the predicted arrival walks from -0.7 km to -4.1 km and
    the vehicle arrives 5.4 km short.  Every flight of the batch does the
    same thing, and that shortfall is the whole of the landing bias: across
    eighteen flights the touchdown is ``+118 m + 0.324 * arrival`` with a
    residual of 258 m, so an arrival centred on zero lands on the runway and
    nothing else has to change.

    The frozen command is not even a good one.  At 18 km the vehicle is
    gliding at L/D 0.75 with 13.8 degrees achieved, on an airframe whose
    best glide at that Mach is nearer 20 degrees -- so it is short, it knows
    it is short, and it is flying below its own best-glide angle while being
    short.  Freezing is the worst of the three available answers: it neither
    reaches the gate nor goes as far as it could.

    So when the arc grounds, change the *objective* rather than giving up on
    it.  The propagator that could not find a miss can still say how far each
    candidate gets, so bracket the angle of attack across the range the solve
    is allowed and fly whichever arc travels furthest.  A reaching arc beats
    a grounded one outright -- it got to the gate, which is the thing being
    asked for -- and among arcs of the same kind the furthest wins.

    Three properties make this a law rather than a tuning:

    * **Nothing is transcribed.**  The best-glide angle is not a constant
      here; it is whatever the vehicle's own swept aero table and the
      propagator say goes furthest *at this Mach and this altitude*, which
      is the quantity that actually varies and the one CLAUDE.md's failure 13
      is about.  A different airframe gets a different answer for free.
    * **It cannot walk the wrong way.**  Range against angle of attack has an
      interior optimum on this airframe and a gradient step is free to
      descend away from the answer -- which is what ``SOLVE_ALPHA_MIN_DEG``
      exists to prevent.  A bracket over the whole permitted span has no such
      failure mode, and it is why ``_solve_range`` brackets too.
    * **It hands back cleanly.**  The moment a candidate reaches the gate the
      normal solve has a miss to work with again on the next tick, so this is
      a mode the vehicle falls into and climbs out of, not a latch.

    Bank is held at the floor the cross-track demands and no more.  Rolling
    lift off the vertical always shortens the flight, so a vehicle that is
    short may not spend range on anything the lateral miss is not actually
    asking for -- and inside the deadband that floor is exactly zero.
    """
    lean = cross_bank_floor(env, cfg, r, gate, flat.cross)
    magnitude = min(lean, align_bank_cap(env, cfg, r, gate))
    sign = _bank_sign(env, cfg, r, v, gate, bank0, flat.cross)
    bank = sign * magnitude

    # **The bracket opens all the way down, and that is the point.**
    # ``SOLVE_ALPHA_MIN_DEG`` floors the ordinary solve at 20 degrees to keep
    # a *gradient* step out of the range curve's interior minimum.  A bracket
    # has no such failure mode -- it evaluates the whole span and takes the
    # best of it -- so the floor buys nothing here and costs a great deal:
    # offline the range curve reads 1265 km at 20 degrees and 1610 km at 5,
    # so the guard against walking downhill is also a guard against the
    # shorter side of the only hill that can still help.
    #
    # It has to open, because the two limits collide exactly where the
    # shortfall is.  ``alpha_ceiling`` ratchets down to what the airframe
    # will hold -- 18 degrees by 23 km on this vehicle -- while the floor
    # stays at 20, and ``floor = min(floor, top)`` then collapses the
    # ordinary solve's bracket to the *single point* 18.0.  That is the
    # frozen command in ``logs/LOG1877``: not a solve that chose to hold,
    # but a solve with no remaining degree of freedom, running every tick and
    # able to return only the one angle it started with.
    low = min(cfg.ALPHA_MIN_DEG, top)
    probes = max(2, int(getattr(cfg, "SOLVE_MAX_RANGE_PROBES", 6)))
    best = None
    for i in range(probes + 1):
        share = i / float(probes)
        alpha = low + (top - low) * share
        steer, prediction, _ = _fly(env, r, v, mass, cfg, end, gate,
                                    alpha, bank)
        if prediction.skipped:
            # Went back out of the atmosphere: the arc is not one the
            # vehicle is flying and its along-track is not comparable.
            continue
        # A reaching arc outranks a grounded one whatever their misses say:
        # the two numbers are measured at different altitudes and are not
        # the same quantity.  ``boosterland`` failure 5's shape.
        rank = (1 if prediction.reached else 0, prediction.long)
        if best is None or rank > best[0]:
            best = (rank, steer, prediction)

    if best is None:
        return Steer(alpha=alpha0, bank=bank0, cfg=cfg, mass=mass), flat
    _, steer, prediction = best
    prediction.max_range = True
    return steer, prediction


def alpha_floor(env, cfg, speed, altitude, mass=None, gravity=None):
    """The lowest angle of attack the solve may ask for, against Mach.

    ``SOLVE_ALPHA_MIN_DEG`` keeps the hypersonic solve on the monotone branch
    of a range curve whose minimum sits at 20 deg.  That is a fact about Mach
    5-7 and it does not survive into the subsonic glide, where the polar has
    no interior optimum and the measured best glide is 10 deg at L/D 3.39.
    Held at 20 all the way down, the vehicle arrives at Mach 0.9 gliding at
    L/D 0.41 -- falling, not gliding -- which is where the shortfall is
    spent.
    """
    try:
        mach = env.mach(speed, altitude)
    except Exception:                                   # noqa: BLE001
        return cfg.SOLVE_ALPHA_MIN_DEG
    if mach >= cfg.SOLVE_ALPHA_SUB_MACH:
        return cfg.SOLVE_ALPHA_MIN_DEG
    # Subsonically the Mach floor relaxes to best glide -- and the speed
    # floor, when it is on, takes over from there.  The solve has to see the
    # same floor the propagator enforces, or it spends its probes below it
    # measuring a gradient that is not there.
    floor = cfg.SOLVE_ALPHA_MIN_SUB_DEG
    if mass is not None and gravity is not None:
        floor = max(floor, trajectory.alpha_floor_for_speed(
            env, cfg, speed, altitude, mass, gravity))
    return floor


def glide_reserve(env, cfg, r):
    """How far past the gate the glide should currently be aiming, in metres.

    **The glide's target is the gate plus a reserve that decays to nothing.**
    Solving straight at the gate at every altitude puts the vehicle on the
    exact boundary of its own authority for the whole entry, and the errors
    it then has to absorb are one-sided: the propagator over-predicts range
    at the shallow end (failure 8) and the airframe stops holding the angle
    of attack that would stretch the arc exactly where a shortfall appears
    (``Holdable``; ``logs/LOG1015`` learned 14.9 deg at 10 kPa against a
    solve floored at 20).  Arriving short has no answer.  Arriving long has a
    cheap one -- bank, and drag going as ``rho v^2``, both of which get
    *stronger* all the way down.

    So the vehicle is flown deliberately long and gives the margin back on a
    schedule: full at ``GLIDE_RESERVE_FROM_ALT_M``, linearly to zero at
    ``GLIDE_RESERVE_TO_ALT_M``, and zero from there to the gate.  The decay
    is not decoration -- a reserve held to the end is just a long landing,
    which is no more recoverable than a short one.
    """
    if not cfg.GLIDE_RESERVE_ON or cfg.GLIDE_RESERVE_M <= 0.0:
        return 0.0
    altitude = vec.norm(r) - env.equatorial_radius
    top = cfg.GLIDE_RESERVE_FROM_ALT_M
    bottom = cfg.GLIDE_RESERVE_TO_ALT_M
    if altitude >= top:
        return cfg.GLIDE_RESERVE_M
    if altitude <= bottom or top <= bottom:
        return 0.0
    return cfg.GLIDE_RESERVE_M * (altitude - bottom) / (top - bottom)


def solve_glide(env, r, v, mass, cfg, end, alpha0, bank0, ceiling=None,
                plan=None):
    """The angle of attack and bank under ``plan`` (``GLIDE_BANK_SWEEP``).

    With a :class:`trajectory.BankPlan` every propagation of the solve
    flies it, so the magnitude is solved for the slow reversal the vehicle
    will fly -- time near wings level included -- rather than for the mean
    of a reversing entry.  The sign the solve returns is then the relay's
    and is the caller's to replace.  See ``_solve_glide`` for the rest.
    """
    if plan is None:
        return _solve_glide(env, r, v, mass, cfg, end, alpha0, bank0,
                            ceiling)
    env.bank_plan = plan
    try:
        return _solve_glide(env, r, v, mass, cfg, end, alpha0, bank0,
                            ceiling)
    finally:
        env.bank_plan = None


def _solve_glide(env, r, v, mass, cfg, end, alpha0, bank0, ceiling=None):
    """The angle of attack and bank that land the prediction on the gate.

    **Decoupled, not a 2x2.**  The first version inverted a measured Jacobian
    for both controls at once, the way ``boosterland.guidance.solve_steer``
    does, and it does not work on this plant for two measured reasons:

    * ``d(long)/d(alpha)`` **changes sign during the entry**.  From 80 km an
      extra degree lengthens the flight (more lift, the vehicle stays high in
      thin air and glides on); by 45 km an extra degree shortens it (drag has
      taken over).  A Newton step trusts a local gradient to be locally
      informative, and this one is, but the *answer* it points at is on the
      far side of an interior optimum.
    * Angle of attack spends most of the entry against its stop.  When the
      inversion's alpha clamps, the bank it solved alongside is no longer the
      right partner for it -- and nothing re-solved it, so the command froze.
      Offline that sat at 32 deg and -4 deg of bank for five hundred seconds
      with a 72 km miss in front of it, correcting nothing.

    So the controls are solved in the order they have authority, each against
    the *residual* the one before it leaves:

    1. **alpha** takes the downrange miss, clamped to the monotone branch
       above the range minimum (see ``SOLVE_ALPHA_MIN_DEG``).
    2. **bank magnitude** takes what is left of it.  Rolling lift off the
       vertical lets the vehicle sink into thicker air, which always shortens
       the flight -- one sign, no interior optimum.
    3. **bank sign** takes the cross-track, by reversal.  It is the only
       cross-track authority the vehicle has, and it is free: a reversal
       changes which way the lift leans without changing how much of it is
       vertical, so it costs the range budget nothing.

    That division is not an invention; it is how a shuttle entry is flown, and
    the reason is the same one the measurements give here -- angle of attack
    is spoken for by the need to decelerate, so bank is what is left to steer
    with.
    """
    gate = env.runway.gate(end)
    # **Solve inside the ceiling the vehicle has demonstrated**, not up to
    # the airframe's nominal maximum.  ``ratchet_alpha`` learns the angle of
    # attack this vehicle can actually hold, from live telemetry, and the
    # control loop clamps its *command* to it -- but the solve was exploring
    # up to ``ALPHA_MAX_DEG`` regardless, so the trajectory being predicted
    # was flown at an attitude already known to be unreachable.  That is
    # "predict the law you fly" again, in the one place where the plant limit
    # is actually known.
    #
    # A *ceiling* is the right shape here where a gain was not: a saturation
    # cannot be undone by commanding more, whereas ``ALPHA_TRACKING``'s
    # proportional factor was cancelled by the solve asking for more and
    # made the shortfall twice as bad.  See that config entry.
    top = trajectory.glide_alpha_max(cfg, env, vec.norm(v),
                                     vec.norm(r) - env.equatorial_radius)
    top = top if ceiling is None else min(top, ceiling)
    floor = alpha_floor(env, cfg, vec.norm(v),
                        vec.norm(r) - env.equatorial_radius,
                        mass, env.mu / (vec.norm(r) ** 2))
    floor = min(floor, top)
    base = _fly(env, r, v, mass, cfg, end, gate, alpha0, bank0)
    _, flat, m0 = base

    if not env.ready():
        return Steer(alpha=alpha0, bank=bank0, cfg=cfg, mass=mass), flat
    if not flat.reached:
        # **"Did not reach" is not "fell short", and treating them alike
        # sends the vehicle the wrong way.**  ``predict`` distinguishes them
        # and says so: ``grounded`` ran out of sky, which is a shortfall;
        # anything else that fails to reach ran out of *time*, which on a
        # long shallow arc means the vehicle is enormously long.
        #
        # Measured, hooking this branch on ``not reached`` alone fired
        # ``max_range`` at 40 km with the prediction reading **+68 km** and
        # made a vehicle that was already 68 kilometres long fly further
        # still (``logs/LOG1887``).  The shortfall case is handled where the
        # evidence for it actually is -- in ``verified``, on the sign of the
        # residual -- and this branch keeps the old hold.
        if getattr(cfg, "SOLVE_MAX_RANGE_ON", False) and flat.grounded:
            return max_range(env, r, v, mass, cfg, end, gate,
                             alpha0, bank0, floor, top, flat)
        return Steer(alpha=alpha0, bank=bank0, cfg=cfg, mass=mass), flat
    # The cross-track's own claim on the bank, taken *before* the range
    # solve so the angle of attack is solved against the trajectory the
    # vehicle will actually fly, rather than against one with no lean in it.
    lean = cross_bank_floor(env, cfg, r, gate, m0[1])
    bank0 = (1.0 if bank0 >= 0.0 else -1.0) * max(abs(bank0), lean)
    if lean > 0.0:
        _, flat, m0 = _fly(env, r, v, mass, cfg, end, gate, alpha0, bank0)
        if not flat.reached:
            return Steer(alpha=alpha0, bank=bank0, cfg=cfg, mass=mass), flat
    # The glide aims past the gate by a reserve that decays to nothing --
    # deliberately long, spending the margin through the altitudes where
    # shedding is cheap.  See ``glide_reserve``.
    target = glide_reserve(env, cfg, r)
    if abs(m0[0] - target) <= cfg.SOLVE_DEADBAND_M:
        alpha, magnitude = alpha0, abs(bank0)
    else:
        alpha, magnitude = _solve_range(env, r, v, mass, cfg, end, gate,
                                        alpha0, bank0, m0, floor, top, target)
    magnitude = max(magnitude, lean)

    sign = _bank_sign(env, cfg, r, v, gate, bank0, m0[1])
    bank = sign * magnitude
    return verified(env, r, v, mass, cfg, end, gate, alpha, bank, m0, flat,
                    alpha0, bank0, floor, target, top)


def _solve_range(env, r, v, mass, cfg, end, gate, alpha0, bank0, m0,
                 floor=None, top=None, target=0.0):
    """Angle of attack first, then bank magnitude on what is left of the miss."""
    if floor is None:
        floor = cfg.SOLVE_ALPHA_MIN_DEG
    if top is None:
        top = cfg.ALPHA_MAX_DEG
    magnitude = abs(bank0)
    sign = 1.0 if bank0 >= 0.0 else -1.0

    # **Bracket, do not differentiate.**  Range against angle of attack has an
    # interior optimum on this airframe -- offline, at bank 0, it runs
    # 1610 km at 5 deg, a *minimum* of 1265 at 20, then 1730 at 30 and 1823
    # at 32 -- and a Newton step trusts a local slope to point at the answer,
    # which near a turning point it does not.  ``SOLVE_ALPHA_MIN_DEG`` was
    # the first attempt at a guard and it is worse than nothing at the wrong
    # moment: the floor sits *at the bottom of the bowl*, so a solve that
    # walks downhill settles there and stays.  Measured in game, a flight
    # 23 km short at 31 km and Mach 5.7 was commanding exactly 20.0 deg --
    # the worst range the vehicle can fly -- while trying to stretch.
    #
    # So sample the span instead and take the best.  Three or four
    # propagations against two, no reliance on a derivative, and immune to
    # an optimum wherever it sits.  The candidates are the current command,
    # both ends of what is allowed, and the Newton step as a refinement --
    # which is still worth having where the curve *is* locally monotone.
    alpha = alpha0
    best = abs(m0[0] - target)
    da = cfg.SOLVE_ALPHA_PROBE_DEG
    if alpha0 + da > top:
        da = -da
    _, _, m_alpha = _fly(env, r, v, mass, cfg, end, gate, alpha0 + da,
                         sign * magnitude)
    slope = (m_alpha[0] - m0[0]) / da
    candidates = [floor, top]
    # **And best glide, which the ends do not bracket.**  The sampling above
    # takes both stops and a Newton step, which is proof against the
    # *hypersonic* range minimum at 20 degrees that ``SOLVE_ALPHA_MIN_DEG``
    # guards.  Subsonically the curve turns the other way -- it has an
    # interior **maximum** at best glide, measured at 12 degrees and L/D 2.1
    # against 1.19 at 8 and 1.33 at 20 -- so an entry that is short and
    # sampling only the ends chooses whichever stop is less bad and never
    # tries the one angle that would stretch it.
    #
    # Measured, ``logs/LOG880``: 7 km short at 24 km altitude, angle of
    # attack pinned at exactly 20.0 and bank at zero for the whole terminal
    # glide, arriving 15.5 km short with neither control moving.  It was not
    # saturated -- it was at a stop it had chosen.
    #
    # The angle comes from ``airframe.measure``, off the table this vehicle
    # swept for itself, so it is not a constant and it follows the aircraft.
    glide_alpha = getattr(env, "best_alpha", None)
    if glide_alpha is not None and floor <= glide_alpha <= top:
        candidates.append(glide_alpha)
    # **And the middle, because two ends do not sample a span.**  The comment
    # above says "sample the span instead and take the best"; what it does is
    # take the two stops and a Newton step, which brackets nothing.  That is
    # enough against the hypersonic range *minimum* at 20 degrees -- either
    # stop beats the middle there -- and it is exactly wrong against the
    # subsonic range *maximum* at best glide, where the middle beats both
    # stops and is never tried.  ``logs/LOG880`` sat at 20.0 degrees for its
    # whole terminal glide and arrived 15.5 km short.
    #
    # Reading the best-glide angle off the swept table does not fix it here:
    # the table puts it at 8 degrees, which is already the subsonic floor and
    # therefore already a candidate, while the airframe flies it at 12 (the
    # ``airframe DISAGREES`` line says so on every flight).  So sample the
    # interior rather than trust either number -- one more propagation, in a
    # phase that ticks once a second.
    if top - floor > 2.0 * cfg.SOLVE_ALPHA_PROBE_DEG:
        candidates.append(0.5 * (floor + top))
    if slope and abs(slope) * (top - floor) \
            > cfg.SOLVE_MIN_AUTHORITY_M:
        candidates.append(vec.clamp(alpha0 - (m0[0] - target) / slope,
                                    floor, top))
    for candidate in candidates:
        if abs(candidate - alpha0) < 0.05:
            continue
        _, probe, miss = _fly(env, r, v, mass, cfg, end, gate, candidate,
                              sign * magnitude)
        if probe.skipped or not probe.reached:
            continue
        if abs(miss[0] - target) < best:
            alpha, best = candidate, abs(miss[0] - target)

    # What the angle of attack could not take.  Re-flown rather than
    # extrapolated, because ``alpha`` has usually just moved to a stop and the
    # slope measured at ``alpha0`` is not the slope there.
    _, _, m1 = _fly(env, r, v, mass, cfg, end, gate, alpha, sign * magnitude)
    if abs(m1[0] - target) <= cfg.SOLVE_DEADBAND_M:
        return alpha, magnitude

    least = glide_bank_min(cfg)
    magnitude = max(magnitude, least)
    db = cfg.SOLVE_BANK_PROBE_DEG
    if (magnitude + db > cfg.BANK_MAX_DEG
            or magnitude + db < least):
        db = -db
    _, _, m2 = _fly(env, r, v, mass, cfg, end, gate, alpha,
                    sign * (magnitude + db))
    slope = (m2[0] - m1[0]) / db
    if abs(slope) * cfg.BANK_MAX_DEG > cfg.SOLVE_MIN_AUTHORITY_M:
        magnitude = vec.clamp(magnitude - (m1[0] - target) / slope,
                              least, cfg.BANK_MAX_DEG)
    return alpha, magnitude


def glide_bank_min(cfg):
    """The least lean the glide's range solve will fly.

    ``Config.GLIDE_BANK_MIN_DEG`` when set, else ``SOLVE_BANK_MIN_DEG``,
    which the deorbit search and the coast also use -- so this one moves
    the glide alone.
    """
    least = float(getattr(cfg, "GLIDE_BANK_MIN_DEG", 0.0) or 0.0)
    return least if least > 0.0 else cfg.SOLVE_BANK_MIN_DEG


def bank_in_transit(cfg, command, wanted, side, dt, rate=None):
    """Is the lean still travelling between the stops?

    The honest form of ``SOLVE_HOLD_THROUGH_REVERSAL_DEG``'s test.  Its job
    (failure 16) is to stop the angle of attack being yanked in response to
    the wings-level prediction a *transit* produces -- and the first version
    asked "is the commanded magnitude small", on the argument that a lean
    below ``SOLVE_BANK_MIN_DEG`` can only be a reversal in progress.

    It is not.  ``cross_bank_floor`` narrows the band with the range to run,
    so the terminal glide asks for a few degrees of lean and *holds* it, and
    the magnitude test then reads a settled vehicle as permanently
    mid-reversal: measured, 57-83% of every entry's GLIDE ticks, from about
    27 km down.  The angle of attack -- the range solve's primary control --
    was frozen for all of it.  Failure 25.

    So ask the question the guard actually means.  ``command`` is the
    rate-limited lean the vehicle is flying, ``wanted`` the magnitude the
    solve has just asked for and ``side`` the sign the reversal logic is
    committed to; the vehicle is between the stops exactly when the command
    has not yet arrived at ``side * |wanted|``.  During a reversal that stays
    true for the whole slew and becomes false the moment the new lean is
    established, whatever its size -- which is what "between the stops"
    means, and it no longer depends on the lean being a large one.

    ``side`` of ``None`` is the first tick, before anything is committed:
    nothing is in transit yet.
    """
    if side is None:
        return False
    target = side * abs(wanted)
    if rate is None:
        rate = cfg.BANK_RATE_DEG_S
    reach = max(cfg.SOLVE_HOLD_TRANSIT_DEG, rate * max(0.0, dt))
    return abs(target - command) > reach


def align_bank_cap(env, cfg, r, gate):
    """The most lean the glide may still be carrying, this close to the gate.

    Full authority until ``GLIDE_ALIGN_RANGE_M`` out, then a linear taper to
    ``GLIDE_ALIGN_BANK_DEG`` at the gate itself.  See that config entry: the
    approach's turn radius at its bank limit is 2.4 km and it is handed 2.6,
    so a vehicle that arrives mid-turn arrives pointing somewhere it cannot
    turn back from, whatever the capture law does afterwards.

    A cap, not a target -- the glide is free to fly wings level inside it.
    """
    reach = cfg.GLIDE_ALIGN_RANGE_M
    if reach <= 0.0:
        return cfg.BANK_MAX_DEG
    distance = trajectory.surface_distance(env, r, gate)
    if distance >= reach:
        return cfg.BANK_MAX_DEG
    share = max(0.0, distance) / reach
    return (cfg.GLIDE_ALIGN_BANK_DEG
            + share * (cfg.BANK_MAX_DEG - cfg.GLIDE_ALIGN_BANK_DEG))


def cross_band(cfg, distance):
    """How much predicted cross-track is worth reacting to, at this range.

    One definition, shared by the reversal test and by the bank floor, for
    the same reason ``deorbit_aim`` is shared by the search and the burn's
    stop test: two thresholds that are meant to be the same threshold must
    not be able to drift apart.
    """
    return vec.clamp(cfg.CROSS_DEADBAND_PER_KM * distance / 1000.0,
                     cfg.CROSS_DEADBAND_MIN_M, cfg.CROSS_DEADBAND_MAX_M)


def cross_bank_floor(env, cfg, r, gate, cross):
    """The bank magnitude the cross-track needs, whatever the range wants.

    See ``Config.CROSS_BANK_ON``.  Zero inside the band, and zero when the
    feature is off -- not a small number, because a floor of "a little bank
    always" is a range error on every flight rather than a correction on the
    ones that need it.
    """
    if not getattr(cfg, "CROSS_BANK_ON", False):
        return 0.0
    band = cross_band(cfg, trajectory.surface_distance(env, r, gate))
    outside = abs(cross) - band
    if outside <= 0.0:
        return 0.0
    return min(cfg.CROSS_BANK_KP * outside, cfg.CROSS_BANK_MAX_DEG)


def _bank_sign(env, cfg, r, v, gate, bank0, cross=0.0):
    """Which way to lean: an azimuth deadband, not a prediction.

    The bank's *magnitude* is solved against the propagator because it sets
    the sink rate and therefore the range.  Its *sign* cannot be, and the
    reason is structural: the propagation models the mean of a reversing entry
    (see ``trajectory.acceleration``), so it has no opinion about which way the
    next lean should go -- deliberately, because a propagation of one constant
    lean predicts an 80 km curving miss that is an artefact of the model
    rather than anything the vehicle would do.

    So the sign is chosen the way a shuttle chooses it: hold the current lean
    until the angle between the ground track and the bearing to the runway
    leaves a deadband, then reverse.  **Both** deadbands narrow as the range
    closes -- fifteen degrees of azimuth error is nothing with a thousand
    kilometres to run and is the whole budget with fifty, and so is three
    kilometres of predicted cross-track -- so the entry reverses a few times
    early and converges without chattering.  A band that does not narrow is a
    band that chatters at one end or gives up at the other, and this one did
    both: see failure 10e.

    Which sign of bank turns which way is *measured*, by asking
    ``bank_toward`` for the lean whose lift leans toward the runway.  kRPC's
    frames are left-handed and this project does not assume a handedness
    anywhere.
    """
    up = vec.unit(r)
    track = vec.project_out(v, up)
    toward = vec.project_out(vec.sub(gate, r), up)
    sign = 1.0 if bank0 >= 0.0 else -1.0
    if vec.norm(track) < 1.0 or vec.norm(toward) < 1.0:
        return sign
    track = vec.unit(track)
    toward = vec.unit(toward)
    error = vec.angle_between(track, toward)
    distance = trajectory.surface_distance(env, r, gate)
    deadband = vec.clamp(cfg.AZIMUTH_DEADBAND_PER_KM * distance / 1000.0,
                         cfg.AZIMUTH_DEADBAND_MIN_DEG,
                         cfg.AZIMUTH_DEADBAND_MAX_DEG)
    # Either test may call for a reversal.  The azimuth one alone cannot:
    # its deadband widens with range to go, which permits a cross-track
    # proportional to range -- 23 km of offset at 500 km is 2.6 deg against a
    # 6 deg deadband, so in game the lean held one sign all the way down
    # while the offset grew from 469 m to 23 km.
    #
    # The cross-track threshold scales with range to run too, and at a fixed
    # 500 m it did not.  ``cross`` is the offset predicted at the *gate* by a
    # propagation that models a reversing entry, so it carries no lateral
    # lift at all: it is the vehicle's current lateral velocity carried
    # forward, and relay-testing that against a constant is a bang-bang loop
    # whose actuator takes thirteen seconds to roll between the stops.  In
    # game it reversed on an 18-second period through the level-off and
    # overshot the 500 m band to 3.4 km on every swing.  See failure 10e.
    band = cross_band(cfg, distance)
    if error <= deadband and abs(cross) <= band:
        return sign
    wanted = bank_toward(r, v, vec.project_out(toward, track), 1.0)
    return wanted if wanted else sign


def _plan_cross(env, cfg, r, v, mass, end, alpha, magnitude, plan):
    """The cross-track at the gate under ``plan``, or ``None``."""
    gate = env.runway.gate(end)
    steer = Steer(alpha=alpha, bank=magnitude, cfg=cfg, mass=mass, plan=plan)
    p = trajectory.predict(env, r, v, mass, cfg, steer=steer, gate=gate,
                           end=end, target_radius=vec.norm(gate))
    return p.cross if p.reached and not p.skipped else None


def _bracket_root(f, x0, lo, hi, step, tolerance, budget):
    """``(x, f(x))`` nearest a root of a monotone ``f`` on ``[lo, hi]``.

    Local, not global: the curves this serves are steep steps between
    plateaus (a sweep rate that reaches the stop in seconds, a start time
    past the end of the flight), and a global regula falsi stalls on them
    -- 116 km off, offline.  So from ``x0`` step outward, doubling, until
    the sign changes, then Illinois inside that bracket.  A bracket that
    reaches an end without a sign change returns that end.  ``None`` from
    ``f`` (a propagation that did not reach the gate) returns the best
    point so far, or ``(x0, None)``.
    """
    calls = [budget]

    def g(x):
        calls[0] -= 1
        return f(x)

    a = vec.clamp(x0, lo, hi)
    fa = g(a)
    if fa is None or abs(fa) <= tolerance:
        return a, fa
    b = vec.clamp(a + step, lo, hi)
    if b == a:
        b = vec.clamp(a - step, lo, hi)
    fb = g(b)
    if fb is None:
        return a, fa
    if fa * fb > 0.0:
        if abs(fb) > abs(fa):
            a, fa, b, fb = b, fb, a, fa
        while fa * fb > 0.0 and calls[0] > 0:
            direction = 1.0 if b >= a else -1.0
            if (direction > 0 and b >= hi) or (direction < 0 and b <= lo):
                return b, fb
            step *= 2.0
            a, fa = b, fb
            b = vec.clamp(b + direction * step, lo, hi)
            fb = g(b)
            if fb is None:
                return a, fa
            if abs(fb) <= tolerance:
                return b, fb
        if fa * fb > 0.0:
            return (a, fa) if abs(fa) < abs(fb) else (b, fb)
    # Illinois: plain regula falsi keeps one end and crawls where the curve
    # bends, so halve the kept end's value whenever it survives twice.
    x, fx = (a, fa) if abs(fa) < abs(fb) else (b, fb)
    kept = 0
    while calls[0] > 0 and fb != fa:
        c = a - fa * (b - a) / (fb - fa)
        fc = g(c)
        if fc is None:
            break
        if abs(fc) < abs(fx):
            x, fx = c, fc
        if abs(fc) <= tolerance:
            break
        if fc * fa > 0.0:
            a, fa = c, fc
            if kept == 1:
                fb *= 0.5
            kept = 1
        else:
            b, fb = c, fc
            if kept == -1:
                fa *= 0.5
            kept = -1
    return x, fx


def sweep_start(env, cfg, r, v, mass, end, alpha, magnitude, lean, side,
                approach, guess):
    """``(start, cross)``: when the slow reversal should begin.

    ``Config.GLIDE_BANK_SWEEP``, holding.  The plan is: lean on ``side`` at
    ``magnitude``, then at ``start`` seconds cross to the other side at
    ``GLIDE_BANK_SWEEP_RATE_DEG_S`` and stay there.  The cross-track at the
    gate is monotone in ``start`` -- the longer the lean is held, the
    further the track goes that way -- and ``start`` is the one that puts it
    on zero, searched over ``[0, GLIDE_BANK_SWEEP_HORIZON_S]``.  A ``start``
    at 0 is the crossing being due now; when no start nulls it the nearer
    end comes back (0: cross now, the most it can do; the horizon: hold).
    """
    def f(start):
        plan = trajectory.BankPlan(
            lean=lean, hold=side, start=start, toward=-side,
            rate=cfg.GLIDE_BANK_SWEEP_RATE_DEG_S, approach=approach)
        return _plan_cross(env, cfg, r, v, mass, end, alpha, magnitude, plan)
    return _bracket_root(f, guess, 0.0, cfg.GLIDE_BANK_SWEEP_HORIZON_S,
                         cfg.GLIDE_BANK_SWEEP_START_STEP_S,
                         cfg.GLIDE_BANK_SWEEP_TOL_M,
                         cfg.GLIDE_BANK_SWEEP_ITERATIONS)


def sweep_rate(env, cfg, r, v, mass, end, alpha, magnitude, lean, toward,
               guess):
    """``(rate, cross)``: how fast the crossing under way should go.

    ``Config.GLIDE_BANK_SWEEP``, crossing.  The lean moves from ``lean``
    toward ``toward * magnitude`` and stays there; the faster it gets
    there, the further the track goes that way, so the cross-track at the
    gate is monotone in the rate, searched over
    ``[GLIDE_BANK_SWEEP_RATE_MIN_DEG_S, GLIDE_BANK_SWEEP_RATE_MAX_DEG_S]``.
    """
    def f(rate):
        plan = trajectory.BankPlan(lean=lean, hold=0.0, toward=toward,
                                   rate=rate)
        return _plan_cross(env, cfg, r, v, mass, end, alpha, magnitude, plan)
    return _bracket_root(f, guess, cfg.GLIDE_BANK_SWEEP_RATE_MIN_DEG_S,
                         cfg.GLIDE_BANK_SWEEP_RATE_MAX_DEG_S,
                         cfg.GLIDE_BANK_SWEEP_STEP_DEG_S,
                         cfg.GLIDE_BANK_SWEEP_TOL_M,
                         cfg.GLIDE_BANK_SWEEP_ITERATIONS)


def single_reversal_sign(env, cfg, r, v, mass, end, alpha, magnitude,
                         side, flipped, first=False):
    """``(side, flipped, hold, flip)``: the one-reversal sign law.

    ``Config.GLIDE_SINGLE_REVERSAL``.  ``hold`` is the cross-track predicted
    at the gate if the current lean is held all the way there, ``flip`` the
    same if it is reversed now and that is held; both with the lateral lift
    in (``reversing=False``), which is the trajectory this law really flies.
    The flip is due once ``flip`` has come round to the same side as
    ``hold`` and is no longer worse: holding any longer would put the
    reversed arc past the centreline.  While the gate is outside both
    (``hold`` and ``flip`` on one side, ``hold`` the nearer) the lean stays
    toward it.  Once ``flipped`` the side is held; the caller hands the
    sign back to ``_bank_sign`` below the trim Mach.

    ``first`` is the first tick, ``side`` the lean already held: it is kept
    when the gate is between the two arcs (either side works) and turned
    toward the gate when it is not -- without spending the one reversal.
    Either prediction missing (a skip, the arc not reaching the gate) keeps
    the side: a missing answer must not look like a zero crossing.
    """
    gate = env.runway.gate(end)
    magnitude = max(abs(magnitude), cfg.SOLVE_BANK_MIN_DEG)

    def cross(sign):
        steer = Steer(alpha=alpha, bank=sign * magnitude, cfg=cfg, mass=mass,
                      reversing=False)
        p = trajectory.predict(env, r, v, mass, cfg, steer=steer, gate=gate,
                               end=end, target_radius=vec.norm(gate))
        return p.cross if p.reached and not p.skipped else None

    hold, flip = cross(side), cross(-side)
    if flipped or hold is None or flip is None:
        return side, flipped, hold, flip
    if hold * flip > 0.0 and abs(flip) <= abs(hold):
        # Both on one side and reversing is the nearer: the drift has
        # carried the reversed arc onto the centreline -- or, on the first
        # tick, the gate is outside the band and the lean points away.
        return -side, not first, hold, flip
    return side, flipped, hold, flip


def verified(env, r, v, mass, cfg, end, gate, alpha, bank, m0, flat,
             alpha0, bank0, floor=None, target=0.0, top=None):
    """Command the solved pair only if propagating it lands nearer.

    The inversion divides the miss by a sensitivity measured from two probes,
    and near the gate that sensitivity collapses -- so an *already closed*
    miss divided by nearly nothing comes back as a control against its stop.
    In boosterland the same arithmetic held the booster at a saturated angle
    for the last three ticks of a coast and took a closed miss from 0 to
    -91 m; the guard that fixed it is this one, and it is free, because the
    prediction at the commanded angles is computed anyway to hand back.
    """
    if floor is None:
        floor = cfg.SOLVE_ALPHA_MIN_DEG
    was = abs(m0[0] - target)
    for scale in (1.0, 0.5):
        a = alpha0 + scale * (alpha - alpha0)
        b = bank0 + scale * (bank - bank0)
        steer, prediction, miss = _fly(env, r, v, mass, cfg, end, gate, a, b)
        if prediction.skipped:
            # **Stretching by ballooning is not stretching.**  When the entry
            # is short the solve wants lift, and the cheapest lift is bank
            # zero -- every bit of it vertical.  Deep enough in, that is over
            # a g on this airframe and the vehicle leaves the atmosphere
            # again: in game, through periapsis at 23.5 km at Mach 4.1 and
            # climbing at 71 m/s, with the command reading bank -0.1.  The
            # arc does come down eventually and the propagation reports a
            # range for it, so the miss test cannot see this; only the
            # trajectory can.  Refuse it and let the fallbacks bank instead,
            # which is the control that sinks the vehicle.
            continue
        if prediction.reached and abs(miss[0] - target) < was:
            return steer, prediction
    # **Nothing offered was an improvement, and there are two reasons for
    # that which want opposite answers.**
    #
    # Near the gate the miss is already closed and the sensitivity has
    # collapsed, so holding is right and is what this guard was built for.
    # But the same branch is reached when the vehicle is *short and out of
    # authority*, and there holding means flying the last eight kilometres of
    # altitude on whatever angle happened to be commanded when the shortfall
    # began -- 18.0 degrees for thirty consecutive ticks in ``logs/LOG1877``,
    # at L/D 0.75, while the arrival walked out to -4 km.  A vehicle that
    # cannot close the miss should be going as far as it can; those are the
    # same command only when it is already doing so.
    #
    # The two cases are told apart by the residual itself: short by more than
    # the deadband is the range-limited one, and a closed miss is not.
    if (getattr(cfg, "SOLVE_MAX_RANGE_ON", False)
            and (m0[0] - target) < -cfg.SOLVE_DEADBAND_M):
        ceiling = cfg.ALPHA_MAX_DEG if top is None else top
        return max_range(env, r, v, mass, cfg, end, gate, alpha0, bank0,
                         floor, ceiling, flat)
    # Hold, rather than commit a control on no evidence -- but keep any
    # reversal, because the sign costs the range budget nothing and the
    # cross-track has no other actuator anywhere in this flight.
    hold = math.copysign(abs(bank0), bank if bank else bank0)
    steer, prediction, _ = _fly(env, r, v, mass, cfg, end, gate, alpha0, hold)
    if prediction.skipped:
        # Even holding balloons.  The vehicle is carrying more energy than the
        # current attitude can keep in the atmosphere, so put the lift on its
        # side: ``SOLVE_BANK_MIN_DEG`` of bank is the one control that always
        # sinks it, and arriving short having stayed in the air beats
        # arriving on a later pass having left it.
        sunk = math.copysign(max(abs(hold), cfg.SOLVE_BANK_MIN_DEG),
                             hold if hold else 1.0)
        steer, prediction, _ = _fly(env, r, v, mass, cfg, end, gate,
                                    min(alpha0, floor), sunk)
    return steer, prediction

# -- reading the table back ------------------------------------------------
# ``alpha_for_load`` lives in trajectory, because the propagator needs it too:
# the arrival-speed cap is part of the flown law and therefore part of the
# predicted one.  Re-exported here so the control laws read naturally.
alpha_for_load = trajectory.alpha_for_load
alpha_limit_for_speed = trajectory.alpha_limit_for_speed


def bank_toward(r, v, want, magnitude):
    """A signed bank whose lift leans the vehicle toward ``want``.

    The sign is *computed* from the geometry rather than reasoned about.
    ``lift_frame`` returns a pair whose handedness depends on kRPC's, which
    this project does not assume anywhere: the lift at bank ``b`` is
    ``up_perp cos b + side sin b``, so its component along a horizontal
    ``want`` is dominated by ``sin b`` times ``dot(side, want)``, and the sign
    of that dot product is the answer.  Same tactic as
    ``Environment._measure_omega``.
    """
    up_perp, side = trajectory.lift_frame(r, v)
    if up_perp is None or vec.norm(want) < 1e-9:
        return 0.0
    lean = vec.dot(side, vec.unit(want))
    if abs(lean) < 1e-6:
        return 0.0
    return math.copysign(abs(magnitude), lean)


# -- the approach ----------------------------------------------------------
class ApproachCommand:
    def __init__(self, alpha, bank, sink, wanted_sink, cross, heading_error,
                 distance, height, speed):
        self.alpha = alpha
        self.bank = bank
        self.sink = sink
        self.wanted_sink = wanted_sink
        self.cross = cross
        self.heading_error = heading_error
        self.distance = distance
        self.height = height
        self.speed = speed
        self.excess = 0.0
        self.scurve_deg = 0.0
        self.target_speed = speed
        self.speed_floor = 0.0
        # **The lateral state, published so a second lateral authority can
        # reason about it the way the capture does.**  ``cross`` alone says
        # where the vehicle is; these two say where it is *going to be* at
        # the flare's door, which is the only place the cross-track has to be
        # small.  ``APPROACH_SLIP_FOR_ENERGY``'s side force is worth ~50 m
        # per degree and was arriving rather than arresting when it chose its
        # side from the offset now.  Zero when the capture is not running,
        # which degrades the predictor to the offset itself.
        self.cross_rate = 0.0
        self.cross_time = 0.0


def alpha_for_speed(env, cfg, speed, sink, height, mass, gravity, target,
                    trim, bank_deg=0.0, climb_ok=False):
    """The angle of attack that holds ``target`` airspeed, on a glider.

    **A glider cannot choose its speed and its path independently**, which the
    approach has always known -- but the law it drew from that was
    ``trim + gain * (speed - target)``, floored at trim so it could never
    unload the wing.  That loop only works in one direction.  It bleeds off a
    surplus, and faced with a *deficit* it commands one g and waits, and one g
    is not neutral: a vehicle descending at its best glide angle of 18 degrees
    flies a straight path at ``cos(18) = 0.95``, so holding a full g is a
    standing pull-up.  It flattens, it slows, ``trim`` rises with the slowing,
    drag rises with ``trim``, and the vehicle arrives at the flare below the
    speed the flare needs -- measured on ``logs/LOG2246``, 96.6 m/s at 830 m
    decaying monotonically to **59.9** at the flare against a window of 83-91,
    three kilometres past the aim point.  Nothing in the law was fighting it;
    the law had reached its floor and stopped.

    So command the descent instead.  In a straight glide

        dv/dt = g sin(theta) - D/m          theta below the horizon
        L     = m g cos(theta)

    -- the first line says which ``theta`` holds the speed and the second says
    what load flies that ``theta``.  ``alpha_for_load`` turns the load into an
    angle out of the swept table, so the whole thing is arithmetic on measured
    coefficients with one time constant in it (``APPROACH_SPEED_TAU_S``, how
    long a speed error is given to disappear) and no fitted gain at all.

    Drag depends on the angle this is solving for, so it is iterated from the
    trim angle; two passes are plenty, because ``Cd`` moves slowly over the
    few degrees between the answer and the start.

    **And the load has to be flown against the path the vehicle is on, not
    the path it would settle into.**  ``L = m g cos(theta)`` is a statement
    about a *steady* glide, and commanding it open-loop is a positive
    feedback: this airframe delivers about 85% of the angle it is commanded,
    so the load arrives short, the path steepens past the target, and the
    table then answers the higher speed with a *lower* angle still.  Measured
    on ``logs/LOG2247`` with the outer loop alone: 104 m/s and 44 m/s of sink
    at the gate, running away to **124 m/s and 72 m/s of sink** by 350 m, and
    into the ground at 98.  So the commanded load is the steady-state one
    plus ``APPROACH_PATH_KN`` per radian of the difference between the
    descent the vehicle *has* and the one it was asked for -- the ordinary
    inner loop of a two-loop controller, and the reason the outer one is
    allowed to be a slow energy argument.

    **And the load it asks for is in the vertical plane, while the vehicle
    is in a turn.**  ``L = m g cos(theta)`` holds the path only with the
    wings level; banked by ``phi`` the vertical component is ``L cos(phi)``,
    so the wing has to carry ``1 / cos(phi)`` times as much to fly the same
    descent.  Nothing divided by it, and the S-turn banks to 40 degrees --
    ``1/cos(40) = 1.31`` -- so the approach was handed 77% of the load it had
    computed in exactly the moments it had decided it was high.

    What that produces is the opposite of what the S-turn is for.  The weave
    exists to *spend* surplus height; unaccounted, the bank spends it into
    airspeed instead.  Measured on ``logs/LOG2394`` (`qs_plane_inc`), the
    approach at 40 degrees of bank sinks at **56-59 m/s where its own profile
    asks for 25**, reaches the flare at 111 m/s and 190 m instead of the
    83-91 the flare is documented to complete from, and touches down at
    **76.7 m/s** -- a bounce, and the vehicle destroyed two seconds later.

    It also explains failure 68's inversion, which was left as "a knob
    upstream of a phase with no authority is not a lever".  Moving
    ``TOUCHDOWN_AIM_M`` nearer raises the computed surplus, a larger surplus
    is a larger weave, a larger weave is more uncompensated bank, and more
    bank is a *faster, steeper* arrival that floats further -- so aiming
    short landed it nine hundred metres long.  The knob reached the decision
    perfectly well; the decision was wired backwards.

    The inner loop feels some of this -- ``flying`` is the descent the
    vehicle actually has, so the load rises as the path steepens -- but it
    converges to a standing error, because whatever it commands is divided by
    ``cos(phi)`` again on the way to the vertical.

    The dive is bounded (``APPROACH_DIVE_MAX_DEG``).  That bound is the same
    caution ``APPROACH_TRIM_FLOOR`` was written for -- 27 flights that unloaded
    to 4-6 degrees and arrived nose-down at 40-65 degrees below the horizon --
    but expressed where it belongs, on the *path* rather than on the angle: a
    load floor of ``cos(35)`` is a 35 degree dive and no steeper, whatever the
    speed, the mass or the air does.
    """
    rho = env.density(height)
    q = 0.5 * rho * speed * speed
    if q <= 0.0 or mass <= 0.0 or speed <= 1.0:
        return trim
    tau = max(0.5, float(getattr(cfg, "APPROACH_SPEED_TAU_S", 6.0)))
    steepest = math.sin(math.radians(
        float(getattr(cfg, "APPROACH_DIVE_MAX_DEG", 35.0))))
    # ``APPROACH_MUSH_RECOVERY``: well below the target and with height in
    # hand, the dive bound -- and the load floor it sets -- yields, so the
    # wing can unload and the vehicle accelerate out of the back side.  See
    # the config entry (LOG4053).
    if (getattr(cfg, "APPROACH_MUSH_RECOVERY", False)
            and speed < float(cfg.APPROACH_MUSH_SPEED_FRAC) * target
            and height > float(cfg.APPROACH_MUSH_MIN_H_M)):
        steepest = max(steepest, math.sin(math.radians(
            float(cfg.APPROACH_MUSH_DIVE_DEG))))
    gain = float(getattr(cfg, "APPROACH_PATH_KN", 1.5))
    flying = math.asin(vec.clamp(sink / max(1.0, speed), -1.0, 1.0))
    alpha = trim
    for _ in range(2):
        _, cda = env.coefficients(alpha, speed, height)
        drag = cda * q / mass
        # Hold the speed against drag, and spend the difference on the error.
        wanted = (drag + (target - speed) / tau) / gravity
        # **A glider with too much speed can climb.**  The floor at zero
        # meant the law's answer to 270 m/s against a 128 target was "hold
        # the path level" and the surplus went into drag at 22 deg of alpha;
        # (270^2 - 128^2) / 2g is 2.9 km of height, against the 0.6-1.5 km
        # the shuttle's cone ran short by on every flight.  ``climb_ok``
        # lets the same law ask for the negative descent -- a zoom -- bounded
        # like the dive.
        lowest = -math.sin(math.radians(float(getattr(
            cfg, "SPEED_PATH_CLIMB_MAX_DEG", 0.0)))) if climb_ok else 0.0
        wanted = vec.clamp(wanted, lowest, steepest)
        theta = math.asin(wanted)
        load = math.cos(theta) + gain * (flying - theta)
        # (``theta`` negative is a climb: ``flying - theta`` then asks for
        # the pull-up that turns a dive into it, through the same gain.)
        # The wing carries the turn as well as the path.  Bounded, because
        # ``1/cos`` runs away at the vertical and a bank limit that has been
        # exceeded should not become an infinite load demand.
        if getattr(cfg, "APPROACH_BANK_COMPENSATION", False):
            lift_share = math.cos(math.radians(vec.clamp(
                bank_deg, -cfg.APPROACH_BANK_COMP_MAX_DEG,
                cfg.APPROACH_BANK_COMP_MAX_DEG)))
            load /= max(0.1, lift_share)
        # Never more than the flare is allowed to ask for, and never less
        # than the bounded dive: the first keeps this out of the manoeuvre
        # the next phase owns, the second is the old trim floor's caution.
        load = vec.clamp(load, math.cos(math.asin(steepest)),
                         float(getattr(cfg, "APPROACH_LOAD_MAX", 1.6)))
        found = alpha_for_load(env, speed, height, mass, gravity, load)
        if found is None:
            break
        alpha = found
    return alpha


def polar_speed(env, cfg, ratio, height, mass, gravity, stall):
    """The speed whose wings-level, one-g glide ratio is ``ratio``.

    On the fast side of best glide, where speed is stable to fly: best-glide
    speed when ``ratio`` is more than the airframe has (low -- stretch), the
    first faster speed whose ratio has fallen to ``ratio`` otherwise, and
    the top of the scan when even that is too flat (high -- the S-turn's
    job).  Off the table (``airframe.turning_ld``); ``None`` if it cannot
    answer.  The scan's bounds are multiples of the stall only as limits.
    """
    best = None
    curve = []
    # The bounds are equivalent airspeeds -- the stall is one -- so they are
    # scaled to true airspeed at this height (the cone flies 12 km up, where
    # best glide is 1.8x its sea-level speed).
    try:
        rho0, rho = env.density(0.0), env.density(max(0.0, height))
        scale = math.sqrt(rho0 / rho) if rho0 > 0.0 and rho > 0.0 else 1.0
    except Exception:                                       # noqa: BLE001
        scale = 1.0
    step = max(0.5, float(getattr(cfg, "APPROACH_POLAR_STEP_M_S", 2.0))
               * scale)
    v = 1.2 * stall * scale
    while v <= 3.0 * stall * scale:
        ld = airframe.turning_ld(env, cfg, v, height, mass, gravity, 0.0)
        if ld is not None and ld > 0.0:
            ld *= airframe.PLANNING_BIAS      # a straight final, see approach_ld
            curve.append((v, ld))
            if best is None or ld > best[1]:
                best = (v, ld)
        v += step
    if best is None:
        return None
    if ratio >= best[1]:
        return best[0]
    for v, ld in curve:
        if v > best[0] and ld <= ratio:
            return v
    return curve[-1][0]


def approach(env, cfg, end, r, v, mass, gravity, height, weave=0.0,
             heading_lead=0.0):
    """Geometric final: hold the speed, track the centreline, spend the excess.

    No prediction at all, on purpose.  From the gate in, the vehicle is under
    a minute from the ground with a glide ratio of 3.4; there is nothing left
    to trade and nothing a propagation would say that the geometry does not.

    **Angle of attack holds the speed.**  That is the whole design and it is
    the opposite of what an aircraft with a throttle does.  A glider cannot
    choose its speed and its flight path independently -- pick an angle of
    attack and the polar gives you both -- so the one that has to be chosen is
    the speed, because arriving slow is unrecoverable: the measured flare
    *stalls* from 1.3 and 1.5 times the stall speed and only completes from
    1.7 up.  Height is the free variable, and the plan is to have too much of
    it (see ``GATE_ALT_M``) and spend the surplus.

    Spending it is the S-turn.  This airframe has no speedbrake, and its
    usable angle-of-attack window on final is narrow -- between 8 and 12
    degrees the glide ratio only moves from 3.39 to 3.26 while the speed moves
    from 76 to 57 m/s -- so there is very little steepening available before
    the speed floor bites.  Rolling the lift off the vertical is the only
    other drag there is.

    **And for a long time the S-turn did not turn.**  It set the *magnitude*
    of the bank and left the *direction* to the centreline capture, which
    points at the centreline by construction -- so the vehicle banked to its
    40 degree limit, crossed, banked 40 the other way, and flew a limit
    cycle 30 m wide at 10 degrees of track error.  ``1/cos(10 deg)`` is 1.5%
    of extra path.  Measured on ``logs/LOG1315``: twenty consecutive ticks
    at +/-40 degrees of bank, cross-track never outside +/-30 m, and the
    vehicle overflew the aim by 2.5 km with 1.7 km of height still in hand.
    A knob that changes nothing may be disconnected rather than powerless --
    this one was disconnected.

    What an S-turn actually commands is a *track angle* off the centreline,
    ``theta = acos(straight path / path the height can pay for)``, which is
    the cone's weave one phase later and the same arithmetic.  It is asked
    for as a lateral *rate* rather than a bank so that the capture law below
    is untouched -- and so that the same ``sqrt(2 a s)`` that stops the
    vehicle reaching the centreline too fast also stops the weave leaving a
    cross-track it cannot get back from.
    """
    speed = vec.norm(v)
    up = vec.unit(r)
    along = env.runway.horizontal(end, end["along"])
    across = vec.unit(vec.cross(up, along))
    aim = vec.add(end["threshold"],
                  vec.scale(along, airframe.touchdown_aim(env, cfg)))

    offset = vec.sub(vec.scale(vec.unit(r), vec.norm(end["threshold"])), aim)
    distance = -vec.dot(offset, along)          # positive: still short of aim
    cross = vec.dot(offset, across)

    track = vec.project_out(v, up)
    heading_error = 0.0
    if vec.norm(track) > 1.0:
        track = vec.unit(track)
        heading_error = math.degrees(math.atan2(vec.dot(track, across),
                                                vec.dot(track, along)))
        # ``APPROACH_HEADING_LEAD``: the heading the vehicle will have once
        # it has rolled level (``Autopilot.approach_heading_lead``), so the
        # capture does not command level while the turn is still running.
        heading_error += heading_lead

    sink = -vec.dot(v, up)
    # **The speed the flare needs is a speed at one point, not a speed to
    # hold.**  Flown, the flare is entered anywhere from 67 to 104 m/s and
    # only 83-91 lands on the centreline with every part attached: above 94
    # the lateral capture runs out of time and the miss grows monotonically
    # with speed (``corr = -0.99`` between the offset at flare entry and the
    # offset at rest), and below about 82 -- 1.7 times the stall -- the flare
    # stalls.  See spaceplane failure 48.
    #
    # The approach could only ever command a constant, so the good landings
    # were the ones where the vehicle was too starved to obey it.  Commanding
    # a *lower* constant is not the fix and was measured: ``APPROACH_FACTOR``
    # at 1.85 flew -3.9 km and 56.6 m/s three times out of three, all
    # destroyed, because ``HAC_SPEED_FACTOR`` is a multiple of this and the
    # cut starves the cone as well.
    #
    # So schedule it instead: carry the approach speed early, where it is
    # what the range is made of, and arrive at the flare's own requirement by
    # the height the flare actually triggers at.  ``APPROACH_FACTOR`` is
    # untouched, so the cone is untouched; only the last two thousand metres
    # change, which is the stretch that owns the problem.
    # Off the swept table when ``AIRFRAME_DERIVED``; the configured constant
    # otherwise.  Every speed on final is a multiple of this one number, so
    # it is read once here and the multiples are unchanged.
    stall = airframe.stall(env, cfg)
    # **The approach's ground-per-height, not best glide** -- see
    # ``airframe.approach_ld``.  The approach flies 2.25 x stall, not the
    # 78 m/s best glide sits at, and the two ratios differ by a third.
    best_ld = airframe.approach_ld(env, cfg, height, mass, gravity)
    fast = getattr(cfg, "FLARE_SHALLOW", False)
    target = (cfg.FLARE_SHALLOW_APPROACH_FACTOR if fast
              else cfg.APPROACH_FACTOR) * stall
    floor = cfg.APPROACH_SPEED_FLOOR_FACTOR * stall
    if getattr(cfg, "APPROACH_POLAR_SPEED", False) and height > 1.0:
        # ``APPROACH_POLAR_SPEED``: the speed whose glide ratio is the one
        # still needed to the aim, off the polar -- see ``polar_speed``.
        # Replaces ``APPROACH_FACTOR`` (2.25 x stall, the old craft's) and
        # puts the floor under it: on the shuttle 108 m/s is L/D 3.0, so on
        # a final needing 4.2 it read itself low all the way down and dove
        # at 1.4 deg of alpha to hold a speed it could not afford (LOG4281:
        # exc -360..-620, flare at 91 m/s).
        polar = polar_speed(env, cfg, max(0.0, distance) / height, height,
                            mass, gravity, stall)
        if polar is not None:
            # **One-sided: it may only slow the approach toward best
            # glide.**  On the fast side the polar is flat near its top, so
            # a final needing ~4.2 against a best of 4.15 inverted to ~140
            # m/s: LOG4354 crossed the threshold at 138 m/s 700 m up and
            # floated 3.6 km.  Speed stretches a low final; a high one is
            # the S-turn's.
            target = min(target, polar)
            floor = min(floor, target)
    if getattr(cfg, "APPROACH_SPEED_PROFILE", False):
        # The height the flare will fire at, from the same expression the
        # flare's own trigger uses -- shared rather than re-derived, for the
        # reason ``deorbit_aim`` is shared by the search and the stop test.
        trigger = flare_door(cfg, sink, speed, env)
        # **Reach the target above the trigger and hold it there.**  The
        # first version ramped to its target *at* the trigger height, so the
        # vehicle arrived at the flare still decelerating and carried on
        # through the target into the stall -- 65-76 m/s against a window of
        # 83-91, two of five destroyed (failure 49).  A glider cannot make
        # speed, so a target it is still descending towards is a target it
        # will pass.  Finishing the ramp ``APPROACH_PROFILE_HOLD_M`` above
        # the trigger leaves a stretch where the command is constant and the
        # speed loop can settle on it.
        hold = max(0.0, float(getattr(cfg, "APPROACH_PROFILE_HOLD_M", 0.0)))
        span = max(1.0, cfg.GATE_ALT_M - trigger - hold)
        share = vec.clamp((height - trigger - hold) / span, 0.0, 1.0)
        wanted = (cfg.FLARE_SHALLOW_DOOR_FACTOR if fast
                  else cfg.APPROACH_FLARE_FACTOR) * stall
        target = wanted + share * (target - wanted)
        # The floor has to come down with it, or it fights the deceleration
        # it is supposed to be protecting: at a fixed 2.0 x stall it clamps
        # the angle of attack to trim the moment the vehicle goes below 96,
        # which is *above* the speed the flare wants.  Its bottom is the
        # measured one -- the angle the flare stops completing at.
        bottom = cfg.APPROACH_FLARE_FLOOR_FACTOR * stall
        floor = bottom + share * (floor - bottom)
    trim = alpha_for_load(env, speed, height, mass, gravity, 1.0)
    if trim is None:
        trim = cfg.GLIDE_ALPHA_DEG

    # How much height there is over the best glide to the aim point.  Positive
    # is surplus, which is the side to be on.
    reachable = max(0.0, distance) / max(0.1, best_ld)
    excess = height - reachable
    if getattr(cfg, "APPROACH_ENERGY_EXCESS", False) and gravity > 0.0:
        # ``APPROACH_ENERGY_EXCESS``: the speed over the flare's door is
        # height the final has to spend too, and it has a target -- the
        # door.  Counted in height only, LOG4383 left the cone at 133 m/s
        # (650 m over the door), read itself +560 m high only as it slowed,
        # then dove to spend it and reached the door at 81 m/s.
        door = cfg.APPROACH_FLARE_FACTOR * stall
        excess += (speed * speed - door * door) / (2.0 * gravity)
    target, floor = spend_as_speed(cfg, target, floor, excess, stall,
                                   gravity, fast)
    wanted_sink = (height * max(1.0, math.sqrt(max(0.0, speed * speed
                                                   - sink * sink)))
                   / distance) if distance > 50.0 else sink

    # **The speed loop is two-sided, or it is not a loop.**  See
    # ``APPROACH_SPEED_PATH``: ``trim + gain * error`` can only ever bleed,
    # because the floor that stops it unloading the wing is one g -- and one g
    # is a *pull-up* on a vehicle descending at 18 degrees, which needs
    # ``cos(18) = 0.95``.  So the command is the descent angle that holds the
    # speed, flown as the load that flies that angle.
    two_sided = getattr(cfg, "APPROACH_SPEED_PATH", False)
    if two_sided:
        # The floor is not a clamp on the angle any more, it is a floor under
        # the *target*: "never slower than this" is a speed the law can fly
        # to, where "never less alpha than trim" was a command it could not
        # recover from.
        alpha = alpha_for_speed(env, cfg, speed, sink, height, mass, gravity,
                                max(target, floor), trim)
        alpha += cfg.APPROACH_PATH_KP * vec.clamp(excess,
                                                  -cfg.APPROACH_PATH_LIMIT_M,
                                                  cfg.APPROACH_PATH_LIMIT_M)
    else:
        alpha = (trim
                 + cfg.APPROACH_SPEED_KP * (speed - target)
                 + cfg.APPROACH_PATH_KP * vec.clamp(
                     excess, -cfg.APPROACH_PATH_LIMIT_M,
                     cfg.APPROACH_PATH_LIMIT_M))

    # The speed floor outranks the path, always.  Arriving at the flare too
    # slow cannot be fixed -- the manoeuvre needs more airspeed than the
    # margin contains and the vehicle stalls in it -- while arriving fast only
    # costs runway, of which there is plenty (46 m/s in 2400 m is 0.44 m/s^2
    # and the gear-down drag alone gives about one).
    if speed < floor and not two_sided:
        alpha = min(alpha, trim)
    # **And never less lift than the vehicle weighs.**
    #
    # The path term subtracts from ``trim`` when the approach is high, and
    # nothing stopped it subtracting all of it: the floor was
    # ``ALPHA_MIN_DEG``, which is zero.  Unloading the wing does not spend
    # energy -- it converts height into speed and sink -- so on a vehicle
    # with two kilometres left it does not reach any glide at all, it falls.
    # Measured over 27 flights (LOG1626-1652), *every* flare was entered
    # between 41 and 98 m/s of sink at 74-110 m/s of airspeed: 40 to 65
    # degrees below the horizon, where the polar says 15 to 30.  At the
    # commanded 4-6 degrees this airframe's Cl*A is about 17, which at 74 m/s
    # is 57 kN of lift under 66 kN of weight.  Lift below weight is not a
    # steep glide, it is an accelerating fall, and no flare arrests one in
    # 200 m.  None of the 27 kept its parts.
    #
    # ``trim`` is the angle that holds one g *for this airframe at this
    # mass and this speed*, read from the swept table -- so this is a law
    # rather than a number, and it transfers to a vehicle nobody has flown.
    # Surplus height is then spent the way a glider spends it, with the
    # S-turn above, and the worst case is landing long instead of arriving
    # nose-down.
    if cfg.APPROACH_TRIM_FLOOR and not two_sided:
        alpha = max(alpha, min(trim, cfg.APPROACH_ALPHA_MAX_DEG))
    alpha = vec.clamp(alpha, cfg.ALPHA_MIN_DEG, cfg.APPROACH_ALPHA_MAX_DEG)

    # **How far off the centreline to fly, to spend the height that is
    # left.**  Serpentining at ``theta`` flies ``1/cos(theta)`` times as far
    # for the same progress, and the shortfall has exactly that shape:
    # ``distance`` is the path a straight-in final would fly and
    # ``height * APPROACH_BEST_LD`` is the path the height can pay for.
    # Zero by construction the moment the vehicle is on profile, and off
    # near the ground and near the aim, where the cross-track it creates has
    # no time left to be taken back.
    # **The weave's stopping point is measured from the aim, which is what
    # inverts the aim into a lever that works backwards.**  Failure 68 moved
    # ``TOUCHDOWN_AIM_M`` 2400 -> 400 to bias out an overshoot and the vehicle
    # stopped nine hundred metres *later*, and read that as "a knob upstream
    # of a phase with no authority over the quantity is not a lever".  The
    # mechanism is narrower than that and it is here: ``distance`` is
    # measured to the aim, so moving the aim two kilometres nearer also moves
    # the point where dissipation *stops* two kilometres further out.  With
    # the aim at the far threshold the weave runs until 900 m past the near
    # one; with the aim at 400 it stops 1100 m *before* it, and the surplus
    # that used to be dumped low and late is flown out straight instead.
    #
    # What the condition is actually about is whether there is still time to
    # take the cross-track back before the flare freezes it -- which is a
    # time, and the capture below already computes it.  Written as one, the
    # aim stops dragging the dissipation around behind it and becomes a
    # geometry anchor again, which is all it ever claimed to be.
    #
    # ``APPROACH_SCURVE_STOP_S`` is set to what the old constant was
    # delivering at the operating point it was fitted at (7.3 s on
    # ``logs/LOG2384``), so turning this on is meant to change nothing by
    # itself.  That is the point: it has to be verified neutral before the
    # aim can be moved against it.
    scurve_stop = distance > cfg.APPROACH_SCURVE_STOP_M
    if getattr(cfg, "APPROACH_SCURVE_STOP_BY_TIME", False):
        stop_trigger = flare_door(cfg, sink, speed, env)
        scurve_stop = ((max(0.0, height - stop_trigger) / max(1.0, sink))
                       > cfg.APPROACH_SCURVE_STOP_S)
    scurve_deg = 0.0
    if (cfg.APPROACH_SCURVE_TRACK and cfg.APPROACH_LATERAL_CAPTURE
            and excess > cfg.APPROACH_SCURVE_M
            and scurve_stop
            and height > 150.0):
        affordable = max(1.0, height * best_ld)
        ratio = vec.clamp(max(0.0, distance) / affordable, 0.0, 1.0)
        scurve_deg = min(cfg.APPROACH_SCURVE_MAX_DEG,
                         math.degrees(math.acos(ratio)))

    # Bank: the centreline first, then whatever S-turn the surplus height
    # needs on top.  Never below 150 m, where a wing down is worse than
    # anything it could buy.
    cross_rate = 0.0
    cross_time = 0.0
    if cfg.APPROACH_LATERAL_CAPTURE:
        # The lateral closing rate that can still be arrested inside the
        # offset that is left, against the lateral acceleration the bank
        # limit affords: ``v^2 = 2 a s``.  Then bank on the difference
        # between that and the rate the vehicle has.  Every term is a speed
        # or an acceleration, so it does not need re-fitting when the
        # approach speed changes -- which is exactly what went wrong with the
        # two proportional gains it replaces.  See ``APPROACH_LATERAL_CAPTURE``.
        lateral = (cfg.APPROACH_CAPTURE_MARGIN * gravity
                   * math.tan(math.radians(cfg.APPROACH_BANK_MAX_DEG)))
        stoppable = math.sqrt(2.0 * lateral * abs(cross))
        # **And a rate the vehicle still has time to spend.**  "Arrest it in
        # the offset that is left" permits 9 m/s fourteen metres out, which
        # is a closure the *flare* then inherits and flies straight for eight
        # seconds with the wings level -- measured on ``logs/LOG1366``, a
        # handover at ``cross=+14`` reached the first rollout tick at
        # **-72 m** and came to rest at -86, shedding both elevons and a wing
        # on the grass beside a runway it had landed on. The capture is not
        # overshooting; it is being interrupted, and a gentler gain would
        # hand over *more* rate rather than less.
        #
        # So the rate asked for is also the one that puts the cross-track at
        # zero when the wheels arrive: ``|cross| / time to the ground``. The
        # drift through the flare then *is* the last of the correction
        # instead of a departure from it. This is the open item recorded at
        # ``APPROACH_CAPTURE_MARGIN``, and it is a time, not a gain.
        #
        # **And the time to the ground is not the height over the sink**,
        # because the flare spends the last two hundred metres arresting
        # exactly that sink.  Dividing by the approach's rate runs the
        # closure for about twice as long as it was sized for and overshoots
        # by as much: ``logs/LOG1398`` handed over near the centreline, drew
        # 6.8 m/s of closure for it, and touched down at **+45 m** and still
        # going -- +80 at rest, both elevons gone in the grass beside the
        # runway.  The flare's own trigger is the boundary and its average
        # sink is about half, so the remaining time is written against the
        # same constants the flare is triggered on.  A phase that cannot
        # correct still has to be *handed* a state it can fly out.
        trigger = flare_door(cfg, sink, speed, env)
        to_flare = max(0.0, height - trigger) / max(1.0, sink)
        in_flare = 2.0 * min(height, trigger) / max(1.0, sink)
        # **Be centred at the flare's door, not at the wheels.**  Spending
        # the last of the correction inside the flare is what the ``in_flare``
        # term was for, and it works -- the cross-track at rest is within
        # 33 m on every flight that stops.  What it costs is the *wing*: the
        # flare is eight seconds long and the only way to move sideways in it
        # is to put a tip down, at fifty metres, on an airframe whose wings
        # are the lowest thing on it.  Across eighteen flights of the working
        # arm the split is clean and it is on this number alone -- every
        # flight entering the flare inside 70 m of the centreline kept its
        # parts, every flight outside it was ``destroyed in ROLLOUT`` with a
        # ``Structural Wing Type A`` and an elevon the first things to go.
        #
        # So ask for the closure that is finished when the flare starts.  It
        # is a *faster* closure, but it is spent at 150 m and above, where a
        # wing down is free, instead of at 50 m where it is the aircraft.
        # **And the two ends of that are both wrong, so it is a share.**
        # Finishing at the wheels (share 1) puts the cross-track at rest
        # inside 33 m and takes a wingtip doing it; finishing at the door
        # (share 0) starts the flare on the centreline -- measured, +1 m and
        # +0 m -- and then *drifts*, because arriving at zero offset is not
        # arriving at zero rate, and the flare has no authority to take the
        # last of it out.  Seven of seven on the runway either way; what
        # moves is which of the two windows is missed.  The share is how much
        # of the flare the capture is still allowed to use.
        in_flare *= vec.clamp(
            float(getattr(cfg, "APPROACH_CAPTURE_FLARE_SHARE", 1.0)), 0.0, 1.0)
        if getattr(cfg, "APPROACH_CAPTURE_BY_FLARE", False):
            in_flare = 0.0
        cross_time = to_flare + in_flare
        timely = abs(cross) / max(1.0, cross_time)
        wanted_rate = -math.copysign(min(stoppable, timely, speed), cross)
        if scurve_deg > 0.0:
            # Which way this half of the weave leans: the clock, unless the
            # band is used up, in which case the band wins.  The clock is
            # what makes it a manoeuvre rather than a relay (see
            # ``weave_sign``); the band is what keeps the excursion inside
            # an offset the capture can still take back before the flare.
            lean_side = 1.0 if weave >= 0.0 else -1.0
            if lean_side * cross > cfg.APPROACH_SCURVE_CROSS_M:
                lean_side = -lean_side
            room = max(0.0, cfg.APPROACH_SCURVE_CROSS_M - lean_side * cross)
            wanted_rate = lean_side * min(
                speed * math.sin(math.radians(scurve_deg)),
                math.sqrt(2.0 * lateral * room))
        rate = vec.dot(v, across)
        cross_rate = rate
        error = wanted_rate - rate
        magnitude = abs(cfg.APPROACH_CAPTURE_KP * error)
        # Lift toward +across accelerates the vehicle toward +across, so the
        # side to lean is the sign of the rate error and not of the offset.
        want = vec.scale(across, 1.0 if error > 0.0 else -1.0)
        if abs(cross) < 8.0 and abs(error) < 1.0 and scurve_deg <= 0.0:
            magnitude = 0.0
    else:
        magnitude = abs(cfg.APPROACH_CROSS_KP * cross
                        + cfg.APPROACH_HEADING_KP * heading_error)
        want = vec.scale(across, -1.0 if (cross + 12.0 * heading_error) > 0.0
                         else 1.0)
        if abs(cross) < 8.0 and abs(heading_error) < 1.0:
            magnitude = 0.0
    if (not cfg.APPROACH_SCURVE_TRACK
            and excess > cfg.APPROACH_SCURVE_M and height > 150.0):
        magnitude = max(magnitude,
                        cfg.APPROACH_SCURVE_KP
                        * (excess - cfg.APPROACH_SCURVE_M))
        if vec.norm(want) < 0.5:
            want = across
    if height <= 150.0:
        magnitude = min(magnitude, 10.0)
    magnitude = min(magnitude, cfg.APPROACH_BANK_MAX_DEG)
    bank = bank_toward(r, v, want, magnitude) if magnitude > 0.1 else 0.0
    # **Now that the bank is known, ask the wing for the load the turn
    # actually costs.**  The bank falls out of the lateral logic below the
    # speed loop, so the first pass necessarily solved for a wings-level
    # vehicle; this is the same call with the answer fed back, not a second
    # law.  One pass, because the bank does not depend on alpha -- there is
    # no loop to converge, only an ordering to undo.
    if (two_sided and abs(bank) > 1.0
            and getattr(cfg, "APPROACH_BANK_COMPENSATION", False)):
        alpha = alpha_for_speed(env, cfg, speed, sink, height, mass, gravity,
                                max(target, floor), trim, bank_deg=bank)
        alpha += cfg.APPROACH_PATH_KP * vec.clamp(excess,
                                                  -cfg.APPROACH_PATH_LIMIT_M,
                                                  cfg.APPROACH_PATH_LIMIT_M)
        alpha = vec.clamp(alpha, cfg.ALPHA_MIN_DEG, cfg.APPROACH_ALPHA_MAX_DEG)

    command = ApproachCommand(alpha, bank, sink, wanted_sink, cross,
                              heading_error, distance, height, speed)
    command.cross_rate = cross_rate
    command.cross_time = cross_time
    command.excess = excess
    command.scurve_deg = scurve_deg
    # **The speed the approach is trying to hold, published.**  Anything that
    # adds drag has to know it: a glider that ends up below this can only get
    # back by diving, and a dive near the flare door is what destroyed two
    # vehicles the first time the airbrake flew (LOG2824-2825, the flare
    # entered at 250 m with 79-82 m/s of sink against 29-36 unbraked).  See
    # ``spaceplane.airbrake``.
    command.target_speed = target
    command.speed_floor = floor
    return command


# -- the heading alignment cone --------------------------------------------
class HacCommand:
    def __init__(self, alpha, bank, turn_deg, distance, radius, laps,
                 side, height, needed_height, speed, sink):
        self.alpha = alpha
        self.bank = bank
        self.turn_deg = turn_deg
        self.distance = distance       # from the turn centre, now
        self.radius = radius           # of the circle the energy asks for
        self.laps = laps
        self.side = side
        self.height = height
        self.needed_height = needed_height
        self.speed = speed
        self.sink = sink


def hac_frame(env, cfg, end):
    """``(gate, along, across, up)`` in the tangent plane at the low gate.

    One basis for the whole cone, taken at the gate rather than under the
    vehicle: the circle is a *fixed* piece of ground geometry and computing
    it in a frame that slides with the vehicle would make its centre move
    while the vehicle flew round it.
    """
    gate = env.runway.low_gate(end)
    up = vec.unit(gate)
    along = env.runway.horizontal(end, end["along"])
    across = vec.unit(vec.cross(up, along))
    return gate, along, across, up


def hac_cost(env, cfg, end, r, v, side, radius=None):
    """Path plus the turn to start it, for one end and one hand.

    The path model alone is positional -- it says nothing about which way
    the vehicle is *pointing*, and a vehicle arriving backwards down a
    tangent it is already on is costed nothing.  The initial heading change
    is charged at ``radius`` per radian, which is what turning through it
    actually costs.
    """
    radius = cfg.HAC_RADIUS_M if radius is None else radius
    distance, angle, exit_angle, centre, along, across = hac_state(
        env, cfg, end, r, side, radius)
    if distance < 1e-6:
        return None
    path, turn, tangent = hac_path(cfg, distance, angle, exit_angle, side,
                                   radius)
    ux, uy = math.cos(angle), math.sin(angle)
    if tangent is not None:
        wx = radius * math.cos(tangent) - distance * ux
        wy = radius * math.sin(tangent) - distance * uy
    else:
        wx, wy = -side * uy, side * ux
    norm = math.hypot(wx, wy)
    if norm < 1e-9:
        return None
    wx, wy = wx / norm, wy / norm
    up = vec.unit(r)
    track = vec.project_out(v, up)
    swing = 0.0
    if vec.norm(track) > 1.0:
        track = vec.unit(track)
        cx, cy = vec.dot(track, along), vec.dot(track, across)
        swing = abs(math.atan2(cx * wy - cy * wx, cx * wx + cy * wy))
    return path + radius * swing


def hac_choose(env, cfg, runway, r, v):
    """``(end, side)``: which way round, **and which way down the runway**.

    Both are free and both were being decided by something other than what
    they cost.  The end was picked once at the deorbit, on the arrival
    bearing from fifteen hundred kilometres away, and never revisited; the
    hand was picked on which side of the centreline the vehicle happened to
    be.  Neither is a bad heuristic at range and both are the wrong question
    over the field, where the only thing that matters is how much flying is
    left -- and the four combinations differ by most of a lap.  An arrival
    facing 345 degrees of turn to one threshold faces about 165 to the
    other, and the far hand halves it again.

    Nothing downstream cares which end it lands on.  ``RUNWAY_BOTH_ENDS``
    still pins runway 09 when something does.
    """
    ends = (list(runway.ends.values()) if cfg.RUNWAY_BOTH_ENDS
            else [runway.ends["09"]])
    best = None
    for end in ends:
        for side in (1.0, -1.0):
            cost = hac_cost(env, cfg, end, r, v, side)
            if cost is None:
                continue
            if best is None or cost < best[0]:
                best = (cost, end, side)
    if best is None:
        return runway.ends["09"], 1.0
    return best[1], best[2]


def hac_side(env, cfg, end, r, v=None):
    """Which way round the cone, for one end: whichever costs less to fly.

    Kept for the callers that have already committed to an end.  With no
    velocity to judge by it falls back to the side the vehicle is on, which
    is what this was before ``hac_cost`` existed: joining the near side at
    least does not cross the centreline at the moment the approach is being
    set up.
    """
    if v is not None:
        costs = [(hac_cost(env, cfg, end, r, v, side), side)
                 for side in (1.0, -1.0)]
        costs = [c for c in costs if c[0] is not None]
        if costs:
            return min(costs)[1]
    gate, along, across, up = hac_frame(env, cfg, end)
    here = vec.scale(vec.unit(r), vec.norm(gate))
    offset = vec.dot(vec.sub(here, gate), across)
    return 1.0 if offset >= 0.0 else -1.0


def hac_state(env, cfg, end, r, side, radius):
    """Where the vehicle is relative to the circle of this radius.

    ``(distance, angle, exit_angle, centre, along, across)`` -- the distance
    from that circle's centre and the angle round it, in the tangent plane at
    the gate.

    **The centre moves with the radius, and that is the whole definition of
    the cone.**  A circle is only a heading alignment cone if it is tangent
    to the final approach course *at the gate*; pin the centre eight
    kilometres abeam and vary the radius and every circle but one misses the
    gate by ``8 km - R`` sideways.  Flown, that is where the cross-track
    came from: three arrivals rolled out on 3.0, 4.6 and 5.7 km circles
    about a centre placed for an 8 km one, and landed 0.2, 1.2 and 1.6 km
    off the centreline on a runway 70 m wide.

    So the radius chooses the circle *and* its centre together, and the
    rollout point is the gate for every candidate.
    """
    gate, along, across, _ = hac_frame(env, cfg, end)
    centre = vec.add(gate, vec.scale(across, side * radius))
    here = vec.scale(vec.unit(r), vec.norm(gate))
    offset = vec.sub(here, centre)
    x = vec.dot(offset, along)
    y = vec.dot(offset, across)
    # The rollout radius points from the centre back at the gate, which is
    # ``-side`` across.  ``side`` is also the turn's sense: +1 circles the
    # centre the way the angle increases.
    return (math.hypot(x, y), math.atan2(y, x),
            math.atan2(-side * radius, 0.0), centre, along, across)


def gate_dist(env, cfg):
    """The low gate's distance before the threshold: the runway's own
    (``GATE_FROM_APPROACH``) when it has one, else ``GATE_DIST_M``."""
    runway = getattr(env, "runway", None)
    got = getattr(runway, "gate_dist", None)
    return got() if callable(got) else cfg.GATE_DIST_M


def hac_turn(cfg, angle, exit_angle, side):
    """How much turn is left, in ``[0, 2pi)`` -- with the wrap in the right place.

    **A vehicle sitting exactly on the rollout reads either 0 or 2pi**
    depending on the last bit of the arithmetic, and the difference is a
    whole extra lap.  Flown, that is not a subtle error: the exit test never
    fires, the vehicle rolls wings-level over the gate, reads 359.99 degrees
    of turn still to go and sets off round again with the height it needed to
    land.  Anything inside ``tolerance`` of the rollout has arrived at it,
    from either side, which is also what the exit test means.
    """
    turn = (side * (exit_angle - angle)) % (2.0 * math.pi)
    # A little past the rollout is arrived, not a lap to go -- see
    # ``Config.HAC_OVERSHOOT_DEG``.
    past = max(cfg.HAC_EXIT_TURN_DEG, getattr(cfg, "HAC_OVERSHOOT_DEG", 0.0))
    if turn > 2.0 * math.pi - math.radians(past):
        return 0.0
    return turn


def hac_path(cfg, distance, angle, exit_angle, side, radius):
    """``(path, turn, tangent_angle)`` to the rollout, flying this radius.

    Two legs, because the vehicle does not start on the circle: a straight
    run to the point where it becomes tangent, then the arc from there to the
    rollout.  Modelling only the arc -- which the first version did -- makes
    the path go to zero for a vehicle lined up thirty kilometres out, so the
    guidance reads "no path left" exactly when there is the most of it and
    calls for extra laps to use up height it is about to need.

    Inside the circle there is no tangent; the honest answer there is the arc
    at the radius the vehicle actually has.
    """
    # **Never less than the straight line to the gate.**  Inside the circle
    # the arc model is ``d * turn``, which goes to zero as the vehicle lines
    # up -- so a vehicle inside a 10 km circle, aligned, and still three
    # kilometres from the gate was costed *no path at all*.  Three flights
    # of one batch duly flew themselves down to the gate's altitude three to
    # four kilometres short of it and landed there.  The distance to the
    # gate is a floor no geometry can get under, and it costs one
    # hypotenuse.
    to_gate = math.hypot(distance * math.cos(angle),
                         distance * math.sin(angle)
                         - radius * math.sin(exit_angle))
    if distance <= radius:
        turn = hac_turn(cfg, angle, exit_angle, side)
        return max(distance * turn, to_gate), turn, None
    lead = math.sqrt(max(0.0, distance * distance - radius * radius))
    # **The tangent point is ``acos(R/d)`` round from the vehicle *with* the
    # turn, not against it.**  Both offsets are tangent points; only one is
    # reached travelling the way the turn goes, and the other is reached
    # travelling backwards along the circle.  Taken with the wrong sign the
    # arithmetic still produces a plausible number, which is why this
    # survived: a vehicle lined up thirty kilometres out on the extended
    # centreline -- the best arrival there is -- was costed a **149 degree
    # turn** instead of none, so every flight read as short of height,
    # abandoned the cone, and cut in from wherever it happened to be.
    tangent = angle + side * math.acos(vec.clamp(radius / distance, -1.0, 1.0))
    turn = hac_turn(cfg, tangent, exit_angle, side)
    # ``HAC_PATH_WRAP_TO_GATE``: **a tangent point past the rollout is not
    # path.**  Just outside the circle and lined up, the tangent point lies
    # a few degrees *beyond* the rollout; ``hac_turn`` rightly calls that
    # arrived, but ``lead`` still runs to it -- ``sqrt(x^2 + 2 R dy)``, so
    # 190 m outside a 16 km circle 955 m before the gate costs 2651 m.  The
    # scan takes the longest path that fits, so it *chose* those radii and
    # the cone read on-profile while 1.2-2.8 km high: LOG4056 ``gate=955
    # path=2650``, LOG4051 ``gate=6207 path=9484``.  The vehicle rolls out
    # at the gate, so the gate is the path.
    if (getattr(cfg, "HAC_PATH_WRAP_TO_GATE", False) and turn == 0.0
            and (side * (exit_angle - tangent)) % (2.0 * math.pi) > math.pi):
        return to_gate, turn, tangent
    # ``HAC_PAST_BEFORE_GATE_DEG``: **a few degrees past the rollout is not
    # a lap while the gate is still ahead.**  ``hac_turn`` forgives 12 deg;
    # one degree more and the same vehicle is costed a whole circle.  LOG4152
    # joined over the field lined up 16 km before the gate, the weave's
    # first swing put the tangent point 13 deg past the rollout, and the
    # plan read 347 deg / 106 km to go against 59 km affordable -- *short*
    # -- so the weave stopped, the wings went level and it flew 13 km
    # straight at the gate, arriving 5.4 km high (every high exit on
    # qs_shuttle2 has this shape, laps=0).  Before the gate (``x < 0``) the
    # path is the run to it; a lap that is really needed is the scan's
    # ``laps``, priced as a lap.  Past the gate the wrap stands: that is a
    # lap being flown.
    past_deg = float(getattr(cfg, "HAC_PAST_BEFORE_GATE_DEG", 0.0))
    if (past_deg > 0.0
            and turn > 2.0 * math.pi - math.radians(past_deg)
            and distance * math.cos(angle) < 0.0):
        return to_gate, 0.0, tangent
    return max(lead + radius * turn, to_gate), turn, tangent


def hac_hold_radius(cfg, speed, gravity=9.81, load=None):
    """The tightest circle the airframe can hold at ``speed``.

    ``R = v^2 / (g tan(bank))`` with ``HAC_HOLD_MARGIN`` on top, which is
    the same expression ``hac_radius`` floors its scan with -- named here so
    that the phase transition and the solver ask the *same* question.  They
    did not, and that is how a vehicle came to be handed a circle at entry
    that it could not fly: ``logs/LOG2756`` entered at 441 m/s, where this
    returns **25.7 km**, and was given 16.0 km on its first tick because the
    solver clamps its own floor to ``HAC_RADIUS_MAX_M``.  The log printed
    ``R=16000`` and nothing anywhere said the number was impossible --
    failure 33's shape, a clamp making two configurations look identical
    while faithfully reporting the one that was ignored.
    """
    bank = math.radians(cfg.HAC_BANK_MAX_DEG)
    if getattr(cfg, "HAC_LOAD_MODEL", False) and load is not None:
        # **The honest radius.**  Lateral acceleration is ``n g sin(bank)``,
        # not ``g tan(bank)`` -- the two agree only when the wing is pulling
        # the ``1/cos(bank)`` that holds altitude, and this one pulls about
        # 1.06 g while the formula assumes 1.41 at the committed cap.  The
        # ``tan`` form is optimistic at every bank angle and increasingly so
        # as bank rises (1.33x too tight at 45 degrees, 1.89x at 60, 2.8x at
        # 70), which is exactly the wrong direction for raising the cap.
        #
        # It also makes an impossible bank *say so*.  Where the ``tan`` form
        # shrinks the planned circle without limit, this one blows the radius
        # up when the wing cannot pay for the turn, so ``hac_enterable``
        # refuses instead of printing a number nothing can fly -- failure
        # 33's clamp, closed rather than rediscovered.
        return (cfg.HAC_HOLD_MARGIN * speed * speed
                / max(0.1, load * gravity * math.sin(bank)))
    return (cfg.HAC_HOLD_MARGIN * speed * speed
            / max(0.1, gravity * math.tan(bank)))


def hac_enterable(cfg, speed, gravity=9.81, load=None):
    """Is the vehicle manoeuvrable enough for there to be a cone to fly?

    **The calculated form of ``HAC_ENTRY_MACH``.**  That constant is a Mach
    number standing in for a turn radius -- its own comment says so and does
    the arithmetic in prose ("9 km at Mach 0.9, 30 km at Mach 1.8, 250 km at
    Mach 4.7").  A Mach number cannot express it, because what decides
    whether a circle exists is the speed against the *cone's* size, and the
    speed of sound has nothing to do with either.

    So: the cone is enterable the first moment the circle the airframe can
    hold fits inside the one the cone is allowed to fly.  With the committed
    ``HAC_HOLD_MARGIN`` and ``HAC_RADIUS_MAX_M`` that is 347 m/s, which is
    Mach 1.08-1.11 over the altitudes it happens at -- and ``HAC_ENTRY_MACH``
    was **1.10** until it was raised to 1.50.  The derivation recovers the
    original fit; the raise was an override of it.

    **Which direction this moves the handover is not the obvious one.**  It
    is stricter than the committed 1.50 -- but the flights that land do not
    use that latitude anyway: ``logs/LOG2747`` entered at Mach 0.9 and 279
    m/s, needing 10.3 km of radius against 16.0 available, and it entered on
    the ``HAC_ENTRY_DIST_M`` backstop rather than on either speed test.  The
    latitude between Mach 0.9 and this bound is real and unspent, and it is
    height the cone could be starting with.  What the veto at 1.50 buys is
    not an earlier entry for the flights that work; it is permission for the
    flights that do not.
    """
    if not getattr(cfg, "HAC_ENTRY_DERIVED", False):
        return None                 # the caller keeps its Mach test
    return hac_hold_radius(cfg, speed, gravity, load) <= cfg.HAC_RADIUS_MAX_M


def hac_radius(env, cfg, end, r, side, available, speed=None,
               gravity=9.81, load=None, lap_speed=None):
    """The radius (and extra laps) whose path spends exactly the height left.

    A scan rather than an inversion.  ``path`` is not monotone in the radius
    -- a wider circle shortens the run in to the tangent and lengthens the
    arc -- so there is nothing to invert, and twenty-five evaluations of a
    closed-form expression cost nothing next to one propagation.

    **The lap count is decided first, and it is the smallest one that can
    work.**  Letting the scan choose laps and radius together picks whichever
    pair happens to land nearest the available path, and a tight circle flown
    twice matches almost any number -- so a comfortable arrival came back as
    "3.5 km radius, two laps" where one wide turn would have done.  Laps are
    the expensive answer: each is two more minutes at 45 degrees of bank with
    the ground getting closer.  So: add a lap only when even the widest
    circle is short of path, then fit the radius inside that.
    """
    def costed(radius):
        state = hac_state(env, cfg, end, r, side, radius)
        return hac_path(cfg, state[0], state[1], state[2], side, radius)

    # **The tightest circle is the one the airframe can hold *now*, not a
    # constant.**  ``R = v^2 / (g tan(bank))`` is 6.4 km at 250 m/s and
    # 1.5 km at 120, so a cone entered fast has to be flown wide and tighten
    # as the speed comes off -- which is why it is a cone and not a
    # cylinder.  Scanning down to a fixed 2 km offers the guidance circles
    # that do not exist at entry speed, and it takes them: one flight
    # committed to a 2 km circle at Mach 0.9, could not hold it, and spent
    # the descent somewhere between the plan and the vehicle.
    floor = cfg.HAC_RADIUS_MIN_M
    if speed is not None:
        floor = max(floor, hac_hold_radius(cfg, speed, gravity, load))
    # **The clamp that hid the impossible case.**  When the airframe's own
    # floor is wider than the cone is allowed to be, taking the minimum
    # throws the airframe away and offers a circle it cannot hold.  Under
    # ``HAC_ENTRY_DERIVED`` the phase is not entered until the floor fits,
    # and speed only falls inside the cone, so this can no longer bind and
    # keeping it would only restore the silence.  With the flag off it is
    # exactly the clamp it always was.
    if not getattr(cfg, "HAC_ENTRY_DERIVED", False):
        floor = min(floor, cfg.HAC_RADIUS_MAX_M)

    # ``HAC_LAP_AT_TARGET_SPEED``: **a lap is flown at the cone's speed,
    # not the entry's.**  The floor above is the circle the vehicle can hold
    # *now*; at 270 m/s that is 7.4 km and a lap on it is 46 km of path, so
    # an arrival 7 km high never fits one and exits the gate +5 km (LOG4171,
    # 4152 -- the scan read laps=0 throughout).  By the time a lap is flown
    # the speed law has brought it to ``lap_speed``; price the laps there.
    lap_floor = floor
    if (getattr(cfg, "HAC_LAP_AT_TARGET_SPEED", False)
            and lap_speed is not None and speed is not None):
        lap_floor = max(cfg.HAC_RADIUS_MIN_M,
                        hac_hold_radius(cfg, min(speed, lap_speed), gravity,
                                        load))
        lap_floor = min(lap_floor, floor)

    def scan(laps):
        """The best radius at this lap count, and how well it fits.

        **Over budget is not the same as under budget**, and scoring them
        the same cost a flight.  A candidate longer than the height can pay
        for arrives *short of the gate*, which nothing downstream can undo;
        one shorter arrives high, which the approach's S-turn and
        ``HAC_EXIT_SURPLUS_M`` exist to absorb.  Scored on ``abs``, an
        entry with 19.1 km of path in hand picked a 21.5 km plan over a
        10.2 km one because 2.4 km of error beats 8.9 -- and was already
        1.8 km of height short at the handover.  So: the longest path that
        still fits, and only if nothing fits, the shortest one.
        """
        fits = None
        over = None
        steps = 24
        low = lap_floor if laps > 0 else floor
        for i in range(steps + 1):
            radius = low + (cfg.HAC_RADIUS_MAX_M - low) * i / float(steps)
            path, turn, tangent = costed(radius)
            total = path + laps * 2.0 * math.pi * radius
            entry = (abs(total - available), radius, laps, turn, tangent,
                     total)
            if total <= available:
                if fits is None or total > fits[5]:
                    fits = entry
            elif over is None or total < over[5]:
                over = entry
        return fits if fits is not None else over

    # **The lap count obeys the same asymmetry as the radius, and ties go
    # to fewer laps.**  Three rules were tried here and the first two failed
    # in flight.  Letting the scan choose radius and laps together picks
    # whichever pair lands nearest and a tight circle flown twice matches
    # almost any number, so a comfortable arrival came back as "3.5 km
    # radius, two laps".  Comparing ``available`` against the widest
    # circle's path instead reads a vehicle 300 m from the gate with 310 m
    # in hand as short of path and sells it a lap it can then never finish.
    # Scoring laps on ``abs`` -- while the radius scan inside them scored on
    # fit -- let an over-budget lap win anyway, which is the bug this
    # replaces: the lap loop has to prefer what fits for the same reason the
    # radius scan does.
    candidates = [scan(laps) for laps in range(int(cfg.HAC_MAX_LAPS) + 1)]
    candidates = [c for c in candidates if c is not None]
    fitting = [c for c in candidates if c[5] <= available]
    if fitting:
        best = max(fitting, key=lambda c: c[5])
        for c in fitting:
            # Fewer laps wins anything within a margin worth flying for: a
            # lap is minutes at the bank limit with the ground coming up.
            if c[2] < best[2] and c[5] >= best[5] - cfg.HAC_LAP_MARGIN_M:
                best = c
        # **And among what is left, the tightest circle.**  Every fitting
        # plan spends the same height; they differ in how far from the field
        # they spend it.  Taking the longest total outright picked a 20 km
        # circle and an 87 km path, and the vehicle spent the descent flying
        # away from the runway.  A tighter circle that spends nearly as much
        # keeps the arrival in reach of the gate it is aiming at.
        for c in fitting:
            if (c[2] == best[2] and c[1] < best[1]
                    and c[5] >= best[5] - cfg.HAC_LAP_MARGIN_M):
                best = c
    else:
        best = min(candidates, key=lambda c: c[5])
    return best[1], best[2], best[3], best[4], best[5]


def weave_sign(cfg, elapsed, period=None):
    """Which way a serpentine is leaning, ``elapsed`` into the phase.

    On a clock and not on a cross-track band, because the offset a band
    would watch is the thing the weave is deliberately creating -- a relay
    on it would cancel the manoeuvre it is meant to time.  A pure function
    of the phase clock, so the log says what it was doing.

    ``period`` lets the approach's own S-turn share the clock without
    sharing the cone's period: the two weaves spend the same quantity by
    the same trick, but one has a whole circle to do it in and the other a
    few kilometres of final.
    """
    period = max(1.0, cfg.HAC_WEAVE_PERIOD_S if period is None else period)
    return 1.0 if int(max(0.0, elapsed) / period) % 2 == 0 else -1.0


def weave_bank(cfg):
    """The bank a held weave reverses at (``HAC_WEAVE_BANK_DEG``)."""
    got = float(getattr(cfg, "HAC_WEAVE_BANK_DEG", 0.0) or 0.0)
    return got if got > 0.0 else cfg.HAC_BANK_MAX_DEG


def weave_reversal_s(cfg, theta_deg, speed, gravity=9.81, roll_rate=None):
    """Seconds to swing the track from ``+theta`` to ``-theta``: the turn at
    the weave's bank plus rolling through twice that bank."""
    bank = weave_bank(cfg)
    omega = gravity * math.tan(math.radians(bank)) / max(1.0, speed)
    rate = roll_rate if roll_rate and roll_rate > 0.5 else \
        cfg.HAC_WEAVE_ROLL_RATE_DEG_S
    return (2.0 * math.radians(theta_deg) / max(1e-3, omega)
            + 2.0 * bank / max(0.5, rate))


def weave_efficiency(theta_deg, reversal_s, hold_s):
    """Progress per metre flown over one swing: held at ``theta`` for
    ``hold_s``, then a reversal whose track sweeps ``+-theta`` uniformly
    (mean ``sin(t)/t``)."""
    t = math.radians(theta_deg)
    sweep = math.sin(t) / t if t > 1e-6 else 1.0
    total = hold_s + reversal_s
    if total <= 0.0:
        return 1.0
    return (hold_s * math.cos(t) + reversal_s * sweep) / total


def weave_angle(cfg, ratio, speed, gravity=9.81, roll_rate=None,
                time_left=None):
    """``HAC_WEAVE_HELD``: ``(theta_deg, half_period_s)`` whose swing flies
    ``1/ratio`` times the progress.

    The smallest angle whose *effective* ratio reaches ``ratio`` (the
    largest allowed if none does).  ``time_left`` (s of path to the gate)
    shrinks the hold, and then the angle, so the last swing fits.
    """
    hold = cfg.HAC_WEAVE_HOLD_S
    top = cfg.HAC_WEAVE_MAX_DEG
    best = (0.0, max(1.0, hold))
    for i in range(1, int(top) + 1):
        theta = float(i)
        rev = weave_reversal_s(cfg, theta, speed, gravity, roll_rate)
        h = hold
        if time_left is not None:
            if rev > time_left:
                break
            h = min(hold, time_left - rev)
        best = (theta, rev + h)
        if weave_efficiency(theta, rev, h) <= ratio:
            break
    return best


def _hac_planned_ld(env, cfg, speed, height, mass, gravity, radius, share):
    """Path per metre of height at ``height``, flying ``speed``, with
    ``share`` of the path on the arc of ``radius`` (at the bank that holds
    it) and the rest wings level; combined as height, which is what adds.
    ``None`` if the table cannot answer."""
    if speed <= 1.0:
        return None
    bank = min(cfg.HAC_BANK_MAX_DEG, math.degrees(
        math.atan(speed * speed / max(1.0, gravity * radius))))
    level = airframe.turning_ld(env, cfg, speed, height, mass, gravity, 0.0)
    banked = airframe.turning_ld(env, cfg, speed, height, mass, gravity, bank)
    if level is None or banked is None or level <= 0.0 or banked <= 0.0:
        return None
    return 1.0 / ((1.0 - share) / level + share / banked)


def hac_ladder(env, cfg, height, mass, gravity, reference, radius, turn,
               laps, total):
    """``HAC_LD_AT_TARGET``: path the height still pays for, slice by slice.

    ``[(h, path from GATE_ALT_M up to h)]`` in ``HAC_LADDER_STEP_M`` steps
    to ``height``, each slice priced at the cone's target speed *at that
    height* (``reference(h)``) and the current plan's arc share.  A single
    ratio taken where the vehicle is prices a descent into denser air at
    the thin air's glide: the shuttle read 1.16 at 13 km, called itself
    short, and found its 2 km of surplus below 7 km with 5 km of path left
    (LOG4091).  ``None`` if any slice cannot be answered.
    """
    arc = radius * (turn + laps * 2.0 * math.pi)
    share = vec.clamp(arc / total, 0.0, 1.0) if total > 0.0 else 0.0
    step = max(50.0, float(getattr(cfg, "HAC_LADDER_STEP_M", 500.0)))
    rungs = [(cfg.GATE_ALT_M, 0.0)]
    low, path = cfg.GATE_ALT_M, 0.0
    while low < height:
        high = min(height, low + step)
        mid = 0.5 * (low + high)
        ld = _hac_planned_ld(env, cfg, reference(mid), mid, mass, gravity,
                             radius, share)
        if ld is None:
            return None
        path += ld * (high - low)
        rungs.append((high, path))
        low = high
    return rungs


def straight_in_reach(env, cfg, mass, gravity):
    """``HAC_AIM_DERIVED``: the entry aim's ground per metre of height --
    the middle of what the cone can fly from ``HAC_ALT_M`` to the gate (see
    below), off the table, or ``None``.

    What ``HAC_GATE_LD`` (1.35, fitted on the old craft) stands in for: the
    entry aim should put the cone where a straight-in at the cone's speed
    reaches the gate, so the weave spends what is high and best-glide speed
    stretches what is low.  At 1.35 the shuttle, which glides 2.1-2.6 in
    the cone, entered high by construction on every flight (LOG4321: 14.6
    km at 20 km, weave pinned, out +4.2 km).
    """
    stall = airframe.stall(env, cfg)
    base = cfg.HAC_SPEED_FACTOR * cfg.APPROACH_FACTOR * stall
    rungs = hac_ladder(env, cfg, cfg.HAC_ALT_M, mass, gravity,
                       lambda h: base * eas_scale(env, cfg, h),
                       1.0, 0.0, 0, 1.0)
    if rungs is None or len(rungs) < 2:
        return None
    longest = rungs[-1][1] / max(1.0, rungs[-1][0] - cfg.GATE_ALT_M)
    # **The middle of the cone's authority, not its edge.**  ``longest`` is
    # the flattest the cone can glide (wings level at its own speed); aimed
    # there, nothing is left for a short arrival: LOG4379 arrived on target
    # at ratio 3.57 and ran out of height.  The steepest is the drag at the
    # cone's alpha limit times the held weave's least efficient swing.  The
    # aim is their midpoint in height per metre (the harmonic mean), so equal
    # height errors either way are absorbed.
    mid_h = 0.5 * (cfg.HAC_ALT_M + cfg.GATE_ALT_M)
    speed = base * eas_scale(env, cfg, mid_h)
    try:
        cla, cda = env.coefficients(cfg.HAC_ALPHA_MAX_DEG, speed, mid_h)
    except Exception:                                       # noqa: BLE001
        return longest
    if cla <= 0.0 or cda <= 0.0:
        return longest
    theta = cfg.HAC_WEAVE_MAX_DEG
    rev = weave_reversal_s(cfg, theta, speed, gravity)
    steepest = (cla / cda) * weave_efficiency(theta, rev,
                                              cfg.HAC_WEAVE_HOLD_S)
    if steepest >= longest:
        return longest
    return 2.0 / (1.0 / steepest + 1.0 / longest)


def ladder_height(rungs, path, fallback_ld):
    """The height on ``rungs`` that pays for ``path``; extrapolated past the
    top at the top slice's ratio (``fallback_ld`` with no slice at all)."""
    for (h0, p0), (h1, p1) in zip(rungs, rungs[1:]):
        if p1 >= path:
            return h0 + (h1 - h0) * (path - p0) / max(1e-9, p1 - p0)
    if len(rungs) >= 2:
        (h0, p0), (h1, p1) = rungs[-2], rungs[-1]
        return h1 + (path - p1) * (h1 - h0) / max(1e-9, p1 - p0)
    return rungs[-1][0] + (path - rungs[-1][1]) / max(0.1, fallback_ld)


def eas_scale(env, cfg, height):
    """True airspeed per unit of equivalent airspeed at ``height``.

    **A stall speed is an equivalent airspeed.**  ``STALL_SPEED_M_S`` and
    every factor built on it were measured near the runway, and the cone
    applied them as *true* airspeed nine kilometres up, where the air is a
    third as dense: 108 m/s true there is about 66 equivalent, down by the
    stall.  The shuttle held it with 19-22 deg of alpha and all its drag,
    and with ``HAC_CLIMB`` took 118 m/s for *fast* and pulled up to 88
    (LOG3071).  The same wing at the same angle flies ``sqrt(rho0 / rho)``
    faster in thin air; ``HAC_SPEED_EAS`` says so.  1 when off.
    """
    if not getattr(cfg, "HAC_SPEED_EAS", False):
        return 1.0
    try:
        rho0 = env.density(0.0)
        rho = env.density(max(0.0, height))
    except Exception:                                       # noqa: BLE001
        return 1.0
    if rho <= 0.0 or rho0 <= 0.0:
        return 1.0
    return math.sqrt(rho0 / rho)


def hac(env, cfg, end, r, v, mass, gravity, height, side,
        previous=None, max_step=None, weave=0.0, roll_rate=None,
        ld_scale=None):
    """Circle down to the gate, and let the radius carry the energy error.

    The one control decision here is **how wide to turn**.  The path still to
    be flown is a run in to the tangent plus an arc round to the rollout; the
    path the vehicle can still fly is ``(height - GATE_ALT_M) * HAC_LD``;
    the radius that makes the two equal is the radius to fly, and everything
    else is tracking it.

    That inverts the problem the rest of the entry has.  A straight-in
    approach converts an energy error into a *miss*, which nothing downstream
    can undo; the cone converts it into a radius, and a radius is a thing the
    vehicle can simply fly.  When even the widest circle is short of path the
    answer is another lap, which is 50 km of it.

    Returns ``None`` when the geometry is degenerate (over the centre, or
    stopped), which the caller must treat as "no answer", never as zero.
    """
    speed = vec.norm(v)
    if speed < 1.0:
        return None
    up = vec.unit(r)
    sink = -vec.dot(v, up)

    # **What the cone has to spend is energy, not height.**
    #
    # Budgeted in height alone, a vehicle sitting at the right altitude and
    # thirty-five metres a second too fast reads as *on profile*: it plans no
    # extra path, flies ``laps=0``, rolls out, and hands the approach a
    # surplus in the one currency the approach cannot spend.  Measured over
    # seventeen flights once the along-track was centred (failure 47), the
    # correlation between the arrival and the speed at flare entry is
    # **+0.77** -- the better the arrival, the hotter the flare -- and a
    # flare entered at 103 m/s with 42 m/s of sink lands 29 to 44 m off the
    # centreline, because ``corr(cross-track at flare entry, final
    # cross-track)`` is **-0.99** and there is no time left to finish the
    # lateral capture.
    #
    # The whole configuration downstream had been fitted while the entry
    # delivered a five kilometre shortfall, which was quietly absorbing that
    # energy.  Take the shortfall away and it has to go somewhere.
    #
    # Specific energy is the same arithmetic with one more term: height plus
    # ``v^2 / 2g`` over the speed the next phase wants, which is the speed
    # the cone is already flying its angle of attack to.  Nothing here is
    # measured about this airframe, and it is what a heading alignment cone
    # manages on the vehicle the idea was borrowed from.
    stall = airframe.stall(env, cfg)
    # The cone's own glide ratio, derived at the bank it holds and the speed
    # it is flown at rather than transcribed -- see ``airframe.turning_ld``.
    cone_ld = airframe.cone_ld(env, cfg, speed, height, mass, gravity)
    rungs = None
    reference = (cfg.HAC_SPEED_FACTOR * cfg.APPROACH_FACTOR * stall
                 * eas_scale(env, cfg, height))
    excess_height = 0.0
    if getattr(cfg, "HAC_ENERGY_BUDGET", False) and gravity > 0.0:
        excess_height = max(0.0, (speed * speed - reference * reference)
                            / (2.0 * gravity))
    available = (max(0.0, height + excess_height - cfg.GATE_ALT_M)
                 * cone_ld)
    # **What the wing can pay for, here.**  See ``airframe.turn_load``: the
    # radius model below is a level-turn formula and the cone does not fly a
    # level turn, so the load it assumes has to come off the airframe rather
    # than out of ``1/cos(bank)``.  ``None`` leaves the old ``tan`` form,
    # which is what ``HAC_LOAD_MODEL`` being off means.
    load = airframe.turn_load(env, cfg, speed, height, mass, gravity)
    radius, laps, turn, tangent, total = hac_radius(env, cfg, end, r, side,
                                                    available, speed,
                                                    gravity, load,
                                                    lap_speed=reference)
    measured = getattr(cfg, "HAC_LD_MEASURED", False)
    if getattr(cfg, "HAC_LD_AT_TARGET", False) or measured:
        # ``HAC_LD_AT_TARGET``: price the path still to fly at the ratio the
        # vehicle will fly it at -- the swept table at the cone's own
        # target speed, wings level on the straight legs and at the circle's
        # bank on the arc -- then plan again.  ``HAC_LD`` 1.86 is a
        # whole-cone average of a Mach 0.7 entry at 22 deg of alpha (1.4)
        # and a subsonic straight-in at 6 (2.7-3.0); planned on it, the cone
        # read on-profile mid-way and rolled out 1.1-2.8 km high (LOG4051,
        # 4086).  ``None`` from the table keeps the first plan.
        # Priced slice by slice down to the gate (``hac_ladder``): the
        # descent is flown into denser air, where the ratio climbs.
        base = (cfg.HAC_SPEED_FACTOR * cfg.APPROACH_FACTOR * stall)
        rungs = hac_ladder(env, cfg, height, mass, gravity,
                           lambda h: base * eas_scale(env, cfg, h),
                           radius, turn, laps, total)
        # ``HAC_LD_MEASURED``: the table gives the curve's *shape* with
        # height; the vehicle's own L/D against the table's at the alpha it
        # is flying (``Autopilot.hac_ld_scale``) gives its scale.  A curve
        # corrected by a measurement, where ``HAC_LD`` was one airframe's
        # whole-cone average (1.86 on the old craft; the shuttle's cones
        # fly 2.1-2.6 per planned metre, conesum).
        if rungs is not None and measured and ld_scale:
            rungs = [(h, p * ld_scale) for h, p in rungs]
        if rungs is not None and len(rungs) >= 2:
            top = rungs[-1][1] / max(1.0, rungs[-1][0] - cfg.GATE_ALT_M)
            available = rungs[-1][1] + excess_height * top
            radius, laps, turn, tangent, total = hac_radius(
                env, cfg, end, r, side, available, speed, gravity, load)
    # **The plan is a commanded geometry, and a commanded geometry that
    # steps is a commanded bank that steps.**  Unlimited, the scan flips
    # between two qualitatively different manoeuvres -- a 2 km circle with
    # 340 degrees to fly and a 15 km one with none -- and the bank reverses
    # by 90 degrees with it.  One flight did that four times on the way
    # down, flew nothing in particular, and reached the floor eight
    # kilometres from the gate.  Rate-limiting the radius makes the vehicle
    # commit to a circle for long enough to fly some of it, which is what
    # lets the turn unwind at all.
    if previous is not None and max_step is not None and max_step > 0.0:
        radius = vec.clamp(radius, previous - max_step, previous + max_step)
        path, turn, tangent = hac_path(
            cfg, *hac_state(env, cfg, end, r, side, radius)[:3], side=side,
            radius=radius)
        total = path + laps * 2.0 * math.pi * radius
    distance, angle, exit_angle, centre, along, across = hac_state(
        env, cfg, end, r, side, radius)
    if distance < 1e-6:
        return None

    # **Short of height, stop flying the cone and cut at the gate.**  The
    # circle is a device for *spending* path and every metre of it is a
    # metre the vehicle has to be able to afford; a vehicle that cannot
    # afford the smallest circle is not helped by being flown round it.  This
    # is the one arrival the cone does not absorb, which is why the deorbit
    # is biased the way it is -- but when it happens the shortest path to the
    # gate is the only thing left to want.
    tight = hac_state(env, cfg, end, r, side, cfg.HAC_RADIUS_MIN_M)
    shortest, _, _ = hac_path(cfg, tight[0], tight[1], tight[2], side,
                              cfg.HAC_RADIUS_MIN_M)
    short = available < shortest

    # Where to point.  Outside the circle that is the tangent point, which is
    # the leg the path model just costed -- steering at anything else would
    # fly a distance the energy calculation did not pay for.  On the circle
    # it is the tangent direction, cut inward or outward by the radial error.
    #
    # **Being short does not change where to point**, and the version that
    # thought it did cost a batch.  Cutting straight at the gate is shorter
    # in distance and arrives across the runway, which the four-kilometre
    # approach cannot undo; and for a vehicle that is already near the gate
    # it is not even shorter -- one flight was sent at a gate it was sitting
    # on, flew past it, and spiralled back from 10 km out.  The tightest
    # circle *is* the shortest path that still ends lined up, and the radius
    # scan already picks it when nothing fits: when every candidate path
    # exceeds the height, the smallest one is the closest match.  So short
    # is a flag for the log, not a mode.
    ux, uy = math.cos(angle), math.sin(angle)
    lead = math.sqrt(max(0.0, distance * distance - radius * radius))
    if lead > cfg.HAC_JOIN_M and tangent is not None:
        tx = radius * math.cos(tangent)
        ty = radius * math.sin(tangent)
        wx, wy = tx - distance * ux, ty - distance * uy
    else:
        tx, ty = -side * uy, side * ux
        cut = vec.clamp((distance - radius) / max(1.0, cfg.HAC_CAPTURE_M),
                        -1.0, 1.0)
        wx, wy = tx - cut * ux, ty - cut * uy
    norm = math.hypot(wx, wy)
    if norm < 1e-9:
        return None
    wx, wy = wx / norm, wy / norm

    # **Spend whatever the circle could not.**  ``total`` is the path the
    # geometry plans and ``available`` is the path the height can pay for;
    # the plan is never longer than the height (see ``hac_radius``), so any
    # difference is height with nowhere to go.  Serpentining at ``theta``
    # off the intended track flies ``1/cos(theta)`` times as far for the
    # same progress, which is exactly the shape of the shortfall.
    surplus = max(0.0, available - total)
    weave_deg = 0.0
    weave_half_s = cfg.HAC_WEAVE_PERIOD_S
    held = getattr(cfg, "HAC_WEAVE_HELD", False)
    # A serpentine spends its surplus over whole cycles; one begun with less
    # than a cycle of path left is a lateral excursion into the gate.  See
    # ``Config.HAC_WEAVE_WHOLE_CYCLE``.  Held, ``weave_angle`` fits the
    # last swing to the path instead.
    cycle_ok = (held or not getattr(cfg, "HAC_WEAVE_WHOLE_CYCLE", False)
                or total >= speed * cfg.HAC_WEAVE_PERIOD_S)
    if (cfg.HAC_WEAVE_ON and cycle_ok and surplus > cfg.HAC_WEAVE_DEADBAND_M
            and total > 1.0):
        ratio = vec.clamp(total / max(1.0, available), 0.0, 1.0)
        if held:
            weave_deg, weave_half_s = weave_angle(
                cfg, ratio, speed, gravity, roll_rate,
                time_left=total / max(1.0, speed))
        else:
            weave_deg = min(cfg.HAC_WEAVE_MAX_DEG,
                            math.degrees(math.acos(ratio)))
        phi = math.radians(weave_deg) * (1.0 if weave >= 0.0 else -1.0)
        wx, wy = (wx * math.cos(phi) - wy * math.sin(phi),
                  wx * math.sin(phi) + wy * math.cos(phi))

    track = vec.project_out(v, up)
    if vec.norm(track) < 1.0:
        return None
    track = vec.unit(track)
    cx, cy = vec.dot(track, along), vec.dot(track, across)
    # Signed track error, positive counter-clockwise in the gate's frame.
    error = math.degrees(math.atan2(cx * wy - cy * wx, cx * wx + cy * wy))
    # **Holding a circle is a standing bank, not a heading error.**  With
    # only the proportional term the command is zero exactly when the vehicle
    # is pointing along the tangent -- so it flies straight off the circle,
    # waits for the error to build, banks, overshoots, and hunts.  The
    # feed-forward is the bank the turn itself needs, ``atan(v^2 / gR)``, in
    # the turn's own sense; the proportional term is then only correcting the
    # residual, which is what a proportional term is for.  It is off during
    # the join, because a tangent leg is a straight line.
    forward = 0.0
    if lead <= cfg.HAC_JOIN_M:
        forward = side * math.degrees(
            math.atan(speed * speed / max(1.0, gravity * radius)))
    cap = cfg.HAC_BANK_MAX_DEG
    if held and weave_deg > 0.0:
        cap = weave_bank(cfg)
    signed = vec.clamp(forward + cfg.HAC_HEADING_KP * error, -cap, cap)
    magnitude = abs(signed)
    error = signed
    # ``cross(up, track)`` is the left of the vehicle's own track, and lift
    # leaned that way turns it that way.  The sign is taken from the
    # vehicle's geometry rather than from the runway's frame on purpose:
    # ``across`` is a fixed direction at the gate and the vehicle spends half
    # of every lap pointing the other way, so a sign argued in the gate's
    # frame is right for half the circle and exactly backwards for the rest.
    lean = vec.cross(up, track)
    if error < 0.0:
        lean = vec.scale(lean, -1.0)
    bank = bank_toward(r, v, lean, magnitude) if magnitude > 0.1 else 0.0

    # Angle of attack holds the speed, as everywhere else in this flight --
    # and the load it has to trim for is the *banked* one, because a turn
    # flown at level-flight trim descends through the circle instead of
    # round it.
    load = 1.0 / max(0.2, math.cos(math.radians(min(abs(bank), 75.0))))
    trim = alpha_for_load(env, speed, height, mass, gravity, load)
    if trim is None:
        trim = cfg.GLIDE_ALPHA_DEG
    target = (cfg.HAC_SPEED_FACTOR * cfg.APPROACH_FACTOR
              * stall * math.sqrt(load)) * eas_scale(env, cfg, height)
    if getattr(cfg, "HAC_POLAR_SPEED", False):
        # ``HAC_POLAR_SPEED``: the speed whose glide ratio is the one the
        # plan needs (path over energy height to the gate), off the polar --
        # the approach's law (``polar_speed``) one phase earlier.  Short, it
        # slows to best glide and stretches; high, it flies faster and the
        # weave takes the rest.  At the banked load, as the fixed target was.
        spend = height + excess_height - cfg.GATE_ALT_M
        if spend > 1.0:
            polar = polar_speed(env, cfg, total / spend, height, mass,
                                gravity, stall)
            if polar is not None:
                target = polar * math.sqrt(load)
    if getattr(cfg, "HAC_SPEND_AS_SPEED", False):
        # ``HAC_SPEND_AS_SPEED``: height over the cone's own profile flies
        # the cone slower -- more alpha, less L/D, a steeper circle -- down
        # to the flare's door speed scaled for the bank.  See the config.
        profile = cfg.GATE_ALT_M + total / max(0.1, cone_ld)
        if rungs is not None:
            profile = ladder_height(rungs, total, cone_ld)
        lowest = (cfg.APPROACH_FLARE_FACTOR * stall * math.sqrt(load)
                  * eas_scale(env, cfg, height))
        target = spend_as_speed(cfg, target, target, height - profile,
                                stall, gravity, lowest=lowest)[0]
    # **The one-sided speed law, one phase earlier.**  ``trim + KP * (v -
    # target)`` bleeds a surplus and answers a deficit by holding trim, and
    # trim *rises* as the vehicle slows -- the loop failure 65 took out of
    # the approach.  The old craft never showed it here because it enters
    # the cone with energy to spare.  The shuttle does not: LOG3029 enters
    # at 227 m/s and decays to 55-80 at 14-20 degrees of alpha, where its
    # flown L/D is 1.2-1.5 against 3.3 at 2 degrees, so the cone spends its
    # height on drag and hands over "out of height" 200 degrees off the
    # runway.  ``alpha_for_speed`` commands the descent that holds the
    # speed, which can unload the wing to *make* speed -- the same law, so
    # the two phases cannot disagree about what holding a speed means.
    if getattr(cfg, "HAC_SPEED_PATH", False):
        alpha = alpha_for_speed(env, cfg, speed, sink, height, mass, gravity,
                                target, trim, bank_deg=abs(bank),
                                climb_ok=getattr(cfg, "HAC_CLIMB", False))
    else:
        alpha = trim + cfg.HAC_SPEED_KP * (speed - target)
    alpha = vec.clamp(alpha, cfg.ALPHA_MIN_DEG, cfg.HAC_ALPHA_MAX_DEG)

    needed = cfg.GATE_ALT_M + total / max(0.1, cone_ld)
    if rungs is not None:
        needed = ladder_height(rungs, total, cone_ld)
    # **And what the *approach* needs from here, which is a different
    # number and is the one the handover has to satisfy.**  ``needed`` above
    # is the cone's own profile: the gate's altitude plus the circling path
    # still to fly, divided by the ratio a *turning* descent achieves.  The
    # moment the vehicle leaves the cone it stops flying that path and that
    # ratio -- it flies a straight line to the touchdown aim at the ratio
    # the approach achieves, 1.95 against 1.70.
    #
    # Measured on ``logs/LOG1500``, an arrival 19 m from the gate that rolled
    # out **12 m** above the cone's profile and still touched down 1.9 km
    # past the aim, 2.1 km along a 2.4 km runway: the cone's profile is
    # about 560 m of height -- 1.1 km of ground -- higher than the approach's
    # at the rollout, and the approach cannot spend that (its ground ratio
    # barely moves, high or low).  A phase must hand over a state the next
    # phase can fly, and until now the cone was checking that against its
    # own arithmetic.
    # **How far the vehicle actually is from the rollout point.**  Neither
    # ``turn`` nor ``lead`` answers that on its own, and both were tried.
    # ``lead`` is zero everywhere *inside* the circle, so a vehicle sitting
    # on the abeam line four kilometres to one side of the gate reads
    # ``turn=0, lead=0`` and takes the rollout -- three flights of one batch
    # did exactly that and landed 1.3 to 1.6 km across a runway 70 m wide.
    # The gate is at ``(0, -side * radius)`` in this circle's frame, so the
    # range to it is one hypotenuse and it is not ambiguous about anything.
    gate_range = math.hypot(distance * ux, side * radius + distance * uy)
    approach_needed = ((gate_range + gate_dist(env, cfg)
                        + airframe.touchdown_aim(env, cfg))
                       / max(0.1, airframe.approach_ld(env, cfg, height,
                                                       mass, gravity)))
    command = HacCommand(alpha, bank, math.degrees(turn), distance, radius,
                         laps, side, height, needed, speed, sink)
    command.path = total
    command.short = short
    command.weave_deg = weave_deg
    command.weave_half_s = weave_half_s
    command.alpha_target_speed = target
    command.plan_ld = ((rungs[-1][1] / max(1.0, rungs[-1][0] - cfg.GATE_ALT_M))
                       if rungs is not None and len(rungs) >= 2 else cone_ld)
    command.surplus = surplus
    command.excess_height = excess_height
    command.lead = lead
    command.gate_range = gate_range
    command.approach_needed = approach_needed
    # **On the circle means at the tangent point, not near the radius.**
    # Being 314 m outside a 16 km circle puts the tangent point *ten
    # kilometres* away: one flight read ``turn=0`` there, took the rollout,
    # and handed the approach a vehicle 10 km from the gate at 2.9 km.  The
    # run in to the tangent is the distance that matters, and at ``turn=0``
    # it is also the distance to the gate.
    command.on_circle = lead <= cfg.HAC_JOIN_M
    return command


def touchdown_alpha_cap(cfg, height, sink=0.0):
    """The most angle of attack the vehicle may be in this close to the ground.

    **There is an attitude this airframe cannot land in**, and the flare has
    no term that knows it: the arrest demand is about the sink rate and the
    ground clearance is about the geometry.  Every landing on record touched
    down at 15-17 degrees achieved and dragged its tail -- the best of them
    (``logs/LOG1309``) kept 19 of 23 parts and the rest kept none.

    Tapered rather than stepped, between twice ``FLARE_TOUCHDOWN_ALT_M`` and
    ``FLARE_TOUCHDOWN_ALT_M``, because a cap that arrives in one tick is a
    pitch input in the last second of the flight -- which is the mistake
    failures 31 and 34 are both instances of.
    """
    low = max(0.0, cfg.FLARE_TOUCHDOWN_ALT_M)
    share = vec.clamp((height - low) / max(1.0, low), 0.0, 1.0)
    # **And it yields to a sink rate that is still there.**  The cap is about
    # the attitude to *settle* in, not a limit during the arrest: a vehicle
    # twenty metres up still coming down at 30 m/s needs every degree it
    # has, and capping it there is not a gentler landing, it is an arrival
    # at 21 m/s of sink -- flown, twice, in ``logs/LOG1358`` and
    # ``logs/LOG1361``.  Above ``FLARE_TOUCHDOWN_SINK_M_S`` the flare gets
    # its authority back, tapered so the handover is not itself a step.
    urgent = vec.clamp((sink - cfg.FLARE_TOUCHDOWN_SINK_M_S)
                       / max(1.0, cfg.FLARE_TOUCHDOWN_SINK_M_S), 0.0, 1.0)
    share = max(share, urgent)
    return (cfg.FLARE_TOUCHDOWN_ALPHA_DEG
            + share * (cfg.FLARE_ALPHA_DEG - cfg.FLARE_TOUCHDOWN_ALPHA_DEG))


def rollout_alpha(cfg, speed, elapsed=None, entry_alpha=None, env=None):
    """The angle of attack to hold on the ground at ``speed``.

    **The schedule lives here so that the test and the flight cannot use
    different ones**, which is how it came to be written against a touchdown
    speed the vehicle no longer makes: the ramp was scheduled from 55 m/s
    and every touchdown since has been at 41-51, so it delivered
    ``ROLLOUT_ALPHA_DEG`` on the first tick -- the step it exists to
    prevent.  Both speeds are now fractions of the stall, which is the speed
    the flare ends at and therefore the one this actually depends on.

    ``elapsed``/``entry_alpha`` ramp the command out of whatever attitude the
    flare finished in, because arriving at the schedule is itself a step if
    it happens in one tick.
    """
    # ``env`` is optional here alone: the offline tests call this
    # directly with no environment, and with none it reads exactly the
    # constant it always read.
    stall = max(1.0, airframe.stall(env, cfg))
    full_down = cfg.ROLLOUT_DEROTATE_FACTOR * stall
    band = max(1.0, cfg.ROLLOUT_DEROTATE_BAND_FACTOR * stall)
    share = vec.clamp((speed - full_down) / band, 0.0, 1.0)
    alpha = (cfg.ROLLOUT_ALPHA_DEG
             + share * (cfg.ROLLOUT_HOLD_ALPHA_DEG - cfg.ROLLOUT_ALPHA_DEG))
    if entry_alpha is None or elapsed is None:
        return alpha
    ramp = vec.clamp(elapsed / max(0.01, cfg.ROLLOUT_RAMP_S), 0.0, 1.0)
    return entry_alpha + ramp * (alpha - entry_alpha)


def brake_fraction(cfg, speed, remaining, free_decel):
    """How hard to brake, from the tarmac that is left rather than a constant.

    **The rollout brakes about twenty times harder than it needs to, and it
    is tearing the aircraft apart.**  ``run_rollout``'s own docstring does the
    arithmetic -- 2400 m and a touchdown at 46-54 m/s needs 0.44-0.61 m/s^2,
    and gear-down drag alone gives about one -- and then the code sets
    ``brakes = speed < BRAKE_SPEED_M_S`` with that constant at 200 m/s, which
    is "always".  Measured over 41 flights that reached the runway, the first
    two seconds of rollout decelerate at **12.3 m/s^2**: on 6.9 t that is
    83 kN, more than the vehicle's own weight, through two small gear legs.

    That it is the *loads* and not the touchdown is what the measurement
    says.  Sink at the handover, touchdown speed and deceleration all fail to
    separate the flights that keep their parts from the ones that do not
    (-2.1 vs -2.7 m/s, 46.5 vs 44.6 m/s, and the deceleration is the same
    inside a sigma) -- so the thing that varies is not the arrival, and the
    thing that does not vary is the braking, which is applied flat out on
    every flight regardless of how much runway is in front of it.

    So brake for the distance that is left:

        needed = v^2 / (2 * remaining)

    less whatever drag is already delivering, as a fraction of what the
    wheels can do.  Two properties matter more than the constant:

    * a vehicle that touched down early and has two kilometres in hand asks
      for almost nothing and rolls on its drag, which is the case this
      exists to create; and
    * a vehicle that floated to the far end asks for everything and gets it,
      which is the case the old behaviour was accidentally right about.

    ``remaining`` is signed and may be negative -- past the end of the
    tarmac, where the answer is all of it.  ``free_decel`` is the airframe's
    own ``cda * q / mass``, the same quantity the telemetry line prints as
    ``dec``, so this is not a second drag model.
    """
    if remaining is None:
        return 1.0
    # Below the floor the energy left is small and the loads with it, and
    # rolling friction alone will not stop a vehicle in any useful distance.
    # This is what ``BRAKE_SPEED_M_S`` was always documented to mean.
    if speed < cfg.BRAKE_SPEED_M_S:
        return 1.0
    stop_in = remaining - cfg.ROLLOUT_STOP_RESERVE_M
    if stop_in <= 1.0:
        return 1.0
    needed = (speed * speed) / (2.0 * stop_in)
    full = max(0.1, cfg.BRAKE_DECEL_FULL_M_S2)
    return vec.clamp((needed - max(0.0, free_decel)) / full, 0.0, 1.0)


def spend_as_speed(cfg, target, floor, excess, stall, gravity=9.81,
                   fast=False, lowest=None):
    """``APPROACH_SPEND_AS_SPEED``: a high approach flies slower.

    Returns ``(target, floor)``.  Surplus height past the S-turn's own
    ``APPROACH_SCURVE_M`` is first taken out as speed -- the target falls to
    ``sqrt(target^2 - 2 g surplus)`` -- and never below the speed the flare
    wants at its door, which is where the profile was going to take it
    anyway.  The floor comes down with it, or ``max(target, floor)`` would
    hold the old speed regardless.
    """
    if lowest is None and not getattr(cfg, "APPROACH_SPEND_AS_SPEED", False):
        return target, floor
    spend = excess - float(cfg.APPROACH_SCURVE_M)
    if spend <= 0.0:
        return target, floor
    door = (cfg.FLARE_SHALLOW_DOOR_FACTOR if fast
            else cfg.APPROACH_FLARE_FACTOR) * stall
    if lowest is not None:
        door = lowest
    slow = math.sqrt(max(door * door, target * target - 2.0 * gravity * spend))
    if slow >= target:
        return target, floor
    return slow, min(floor, slow)


def flare_door(cfg, sink, speed=None, env=None):
    """The height the flare starts at, for a vehicle sinking at ``sink``.

    One function for every place that asks -- the phase change, the speed
    profile's end, the S-turn's stop, the lateral capture's clock, the brake
    stow -- so they cannot disagree about where the door is.

    Off (``FLARE_SHALLOW`` False) it is the old ``FLARE_ALT_M +
    FLARE_LEAD_S * sink``.  On, it is the height the pull-up needs: the
    sink brought down to the inner glide's at ``FLARE_SHALLOW_PULL_LOAD``,
    ``FLARE_TRACK_TAU_S`` of the tracker's lag at the present sink, and
    ``FLARE_ALT_M`` of inner glide below that.  See ``Config.FLARE_SHALLOW``.
    """
    sink = max(0.0, sink)
    exp_tau = float(getattr(cfg, "FLARE_EXP_TAU_S", 0.0))
    if (getattr(cfg, "FLARE_DOOR_FROM_SCHEDULE", False) and exp_tau > 0.0
            and not getattr(cfg, "FLARE_SHALLOW", False)):
        # Open where the exponential schedule starts to bind, read one
        # pitch response ahead: ``tau (sink - td) + T sink``.  Never lower
        # than the old door.  See the config entry.
        lag = getattr(env, "pitch_response_s", None)
        if lag is None or lag <= 0.0:
            lag = cfg.FLARE_LEAD_S
        binds = (exp_tau * max(0.0, sink - float(cfg.FLARE_EXP_TOUCHDOWN_M_S))
                 + lag * sink)
        return max(cfg.FLARE_ALT_M + cfg.FLARE_LEAD_S * sink, binds)
    if (getattr(cfg, "FLARE_DOOR_FROM_RESPONSE", False)
            and not getattr(cfg, "FLARE_SHALLOW", False)):
        # The pitch axis's own response time at the present sink, then the
        # pull-up at the most the flare may ask for.  See the config entry.
        lag = getattr(env, "pitch_response_s", None)
        if lag is None or lag <= 0.0:
            lag = cfg.FLARE_LEAD_S
        touchdown = float(cfg.FLARE_TOUCHDOWN_SINK_M_S)
        pull = max(0.05, float(cfg.FLARE_TRACK_LOAD_MAX) - 1.0) * 9.81
        arrest = max(0.0, sink * sink - touchdown * touchdown) / (2.0 * pull)
        return cfg.FLARE_ALT_M + lag * sink + arrest
    if not getattr(cfg, "FLARE_SHALLOW", False):
        return cfg.FLARE_ALT_M + cfg.FLARE_LEAD_S * sink
    inner = inner_glide_sink(cfg, speed if speed is not None else 0.0)
    pull = max(0.05, float(cfg.FLARE_SHALLOW_PULL_LOAD) - 1.0) * 9.81
    arrest = max(0.0, sink * sink - inner * inner) / (2.0 * pull)
    lag = float(getattr(cfg, "FLARE_TRACK_TAU_S", 2.0)) * max(0.0, sink - inner)
    return cfg.FLARE_ALT_M + arrest + lag


def inner_glide_sink(cfg, speed):
    """The sink rate of the shallow inner glide at ``speed``."""
    return max(0.0, speed) * math.sin(math.radians(cfg.FLARE_INNER_GLIDE_DEG))


def flare(env, cfg, r, v, mass, gravity, height, elapsed, lead_s=0.0):
    """Arrest the sink, and do not ask for more than the wing has.

    The angle is ramped rather than snapped to maximum lift.  At 16 m and
    17 m/s of sink the vehicle is a second from the ground, and a step input
    to 30 degrees is a pitch rate the airframe answers with an overshoot --
    which on a low-wing glider means ballooning, and then arriving slow with
    nothing left.  The ramp is short (about a second) because there is not
    much more than that available.
    """
    speed = vec.norm(v)
    up = vec.unit(r)
    sink = -vec.dot(v, up)
    # How much load it takes to stop the descent in the height that is left.
    #
    # **The height that is left goes to zero, and the demand goes to
    # infinity with it.**  At ten metres with 15 m/s still to arrest this
    # asks for 2.5 g and gets ``FLARE_ALPHA_DEG`` -- maximum lift -- so
    # *every* flare, whatever it was doing a second earlier, ends commanding
    # the largest angle of attack the vehicle has.  Measured, they all touch
    # down within a degree of the same attitude (16.0, 15.3, 15.6, 16.2,
    # 17.2 achieved) and scrape along the runway shedding parts, on the
    # centreline and on the tarmac.  The floor keeps the last few metres
    # from asking for what nothing can deliver anyway.
    needed = 1.0
    if height > 0.5 and sink > 0.5:
        # The load that arrests this sink rate in the height that is left,
        # with a margin.  Without the margin the law asks for exactly enough
        # and gets less, because the angle takes time to reach and the vehicle
        # keeps descending while it does -- and because "exactly enough" has
        # no answer for the drag the manoeuvre itself costs.
        arrest = max(height, cfg.FLARE_ARREST_FLOOR_M)
        needed = 1.0 + (sink * sink) / (2.0 * arrest * gravity)
        needed = 1.0 + cfg.FLARE_MARGIN * (needed - 1.0)
    # **An arrest is an open-loop equilibrium too, and it over-arrests.**
    #
    # The demand above says what load *would* stop the descent in the height
    # that is left, and nothing above corrects it when the descent has already
    # stopped.  Measured on ``logs/LOG2259``, entering at 90 m/s and 27 m/s of
    # sink from 115 m: the vehicle is level (``vs = -2.4``) at **50 m**, holds
    # there for four seconds bleeding 62 m/s to 46, stalls out of the float
    # and falls the last fifty metres -- touching down at 41 m/s where its
    # capped angle needs 50 to fly, with the achieved angle of attack eight
    # degrees past the command.  Half the flare is spent arriving and half is
    # spent falling out of the sky.
    #
    # So fly a *sink schedule* instead: the descent rate that can still be
    # brought to ``FLARE_TOUCHDOWN_SINK_M_S`` in the height that is left,
    #
    #     wanted = sqrt(touchdown^2 + 2 a h),   a = (FLARE_TRACK_LOAD - 1) g
    #
    # and command the load that puts the sink back on it.  Memoryless -- it
    # asks only where the vehicle is now -- and self-correcting in the
    # direction that matters: a vehicle that has arrested early is *above*
    # the schedule's arrival and the law unloads and lets it down, which is
    # the float this exists to delete.  Failure 67, one phase later.
    if getattr(cfg, "FLARE_SINK_TRACK", False):
        rise = max(0.05, float(getattr(cfg, "FLARE_TRACK_LOAD", 1.5)) - 1.0)
        touchdown = float(getattr(cfg, "FLARE_TOUCHDOWN_SINK_M_S", 8.0))
        # ``FLARE_LEAD_BY_RESPONSE``: the schedule read at the height the
        # vehicle will be at once its pitch has answered -- ``lead_s`` of
        # the present sink lower.  See the config entry.
        scheduled = max(0.0, height - max(0.0, lead_s) * max(0.0, sink))
        wanted = math.sqrt(touchdown * touchdown
                           + 2.0 * rise * gravity * scheduled)
        if getattr(cfg, "FLARE_SHALLOW", False):
            # The inner glide, and a gentler bottom under it: the pull-up is
            # the part of this schedule the vehicle is furthest behind at the
            # door, the glide is the cap, and the last few metres are drawn
            # at ``FLARE_SHALLOW_FINAL_LOAD`` so the touchdown is a settle.
            final = max(0.01, float(cfg.FLARE_SHALLOW_FINAL_LOAD) - 1.0)
            low = float(cfg.FLARE_SHALLOW_TOUCHDOWN_SINK_M_S)
            bottom = math.sqrt(low * low
                               + 2.0 * final * gravity * scheduled)
            wanted = min(bottom, max(low, inner_glide_sink(cfg, speed)))
        # ``FLARE_EXP_TAU_S``: and no faster than an exponential flare,
        # ``td + h / tau`` -- the square root is steepest at the ground and
        # lands at ``touchdown`` plus the loop's lag by construction (the
        # shuttle, 9-17 m/s at contact, wings and engine lost).
        exp_tau = float(getattr(cfg, "FLARE_EXP_TAU_S", 0.0))
        if exp_tau > 0.0:
            wanted = min(wanted, float(cfg.FLARE_EXP_TOUCHDOWN_M_S)
                         + scheduled / exp_tau)
        tau = max(0.2, float(getattr(cfg, "FLARE_TRACK_TAU_S", 2.0)))
        needed = 1.0 + (sink - wanted) / (tau * gravity)
        needed = vec.clamp(needed,
                           float(getattr(cfg, "FLARE_TRACK_LOAD_MIN", 0.85)),
                           float(getattr(cfg, "FLARE_TRACK_LOAD_MAX", 2.5)))
    target = alpha_for_load(env, speed, height, mass, gravity, needed)
    if target is None:
        target = cfg.FLARE_ALPHA_DEG
    target = min(target, touchdown_alpha_cap(cfg, height, sink))
    ramp = vec.clamp(elapsed / max(0.1, cfg.FLARE_RAMP_S), 0.0, 1.0)
    trim = alpha_for_load(env, speed, height, mass, gravity, 1.0)
    if trim is None:
        trim = cfg.GLIDE_ALPHA_DEG
    return trim + ramp * (target - trim), sink, needed


# -- the deorbit burn ------------------------------------------------------
def deorbit_window(env, r, v, mass, cfg, end, gate):
    """The span of arrivals the glide could still reach from this state.

    Two propagations at the *corners of the solve's own search box*: the
    shortest entry the glide is allowed to fly (``SOLVE_ALPHA_MIN_DEG`` at
    ``BANK_MAX_DEG``) and the longest (``ALPHA_MAX_DEG`` at
    ``SOLVE_BANK_MIN_DEG``).  Those corners are not a guess about the
    airframe -- they are exactly the range of commands ``solve_glide`` may
    issue, so the window is the authority the glide actually has.

    Returns ``(shortest, longest, needed)`` as forward arcs from ``r``, or
    ``None`` when either corner is not an entry at all (never reaches the
    gate's altitude, skips back out, or arrives a revolution late).  ``None``
    and not a wide window: a corner that does not fly is not a bound.

    This exists to delete ``DEORBIT_LONG_BIAS_FRACTION``.  That constant was
    re-fitted five times in one session, every time forced by a change
    somewhere else, because it stands in for the difference between the entry
    the search propagates and the entry the glide flies -- it is not a
    property of the airframe, the orbit or the runway, and a value for it is
    a measurement of whatever the rest of the configuration happened to be
    that afternoon.  The window replaces it with a quantity the search can
    compute for itself.
    """
    # **The corners are bounds, so the tracking model belongs on them.**
    # ``ALPHA_TRACKING`` is a measured fact -- above about 2 kPa this airframe
    # delivers ~85% of the angle of attack it is commanded -- and it is off
    # by default because the *solve* exploits it: told it will get less lift,
    # the search asks for a shallower entry whose extra range does not
    # materialise (failure 10's ``ALPHA_TRACKING`` entry).
    #
    # A bound has no such exploit.  Nothing solves against these two
    # propagations; they only say how far the glide could go if it tried, and
    # believing a command the airframe cannot hold makes the long end of that
    # span too long.  Measured: the centred window landed 19.7 km short with
    # the corners optimistic, on a span only 47 km wide.
    honest = replace(cfg, ALPHA_TRACKING_ON=True) if replace else cfg
    corners = []
    # **Which diagonal of the command box is the real one.**  See
    # ``DEORBIT_WINDOW_CORNERS_FIXED``: range falls with alpha over the whole
    # solve range on this airframe, so the shortest entry is the one flown at
    # *both* stops (max alpha, max bank) and the longest at neither.  The old
    # pairing crossed them and spanned a sliver.
    if getattr(cfg, "DEORBIT_WINDOW_CORNERS_FIXED", False):
        box = ((cfg.ALPHA_MAX_DEG, cfg.BANK_MAX_DEG),
               (cfg.SOLVE_ALPHA_MIN_DEG, cfg.SOLVE_BANK_MIN_DEG))
    else:
        box = ((cfg.SOLVE_ALPHA_MIN_DEG, cfg.BANK_MAX_DEG),
               (cfg.ALPHA_MAX_DEG, cfg.SOLVE_BANK_MIN_DEG))
    for index, (alpha, bank) in enumerate(box):
        steer = Steer(alpha=alpha, bank=bank, cfg=honest, mass=mass)
        prediction = trajectory.predict(env, r, v, mass, cfg, steer=steer,
                                        target_radius=vec.norm(gate))
        if not prediction.reached or prediction.skipped:
            return None
        # **Both corners, and the long one is load-bearing for a reason it
        # does not state.**  Read literally, ``DEORBIT_MAX_TIME_TO_GO_S``
        # rejects an arrival a revolution away, and the long corner is by
        # construction the slowest entry the glide could choose -- a
        # trajectory nobody flies -- so testing it looks like a mistake.  It
        # was vetoing perfectly good burns: the long corner takes 1593 s
        # against the 1500 s limit at the dv that would centre the gate, so
        # the search sat pinned at the shallowest burn that happened to pass,
        # 20 km short, with ``DEORBIT_WINDOW_BIAS`` connected to nothing.
        #
        # Removed, and it is **much** worse.  Free to centre, the search
        # committed 300 km earlier on 28 m/s and three flights landed 95,
        # 103 and 132 km short.  The clock was standing in for a constraint
        # nobody had written down -- *do not commit to an entry so shallow
        # that the propagator cannot be trusted on it* (the shallow end is
        # where the skip lives and where the range over-prediction is worst,
        # failure 8) -- and on this vehicle a 1500 s stretch bound happens to
        # mark that line.  It is kept, and the reason is now recorded rather
        # than inferred: boosterland failure 17's rule, one project over.
        long_corner = (index == 1)
        if prediction.time_to_go > cfg.DEORBIT_MAX_TIME_TO_GO_S \
                and (cfg.DEORBIT_WINDOW_TIME_ON_LONG or not long_corner):
            return None
        corners.append(trajectory.forward_arc(env, r, v, prediction.position))
    shortest, longest = min(corners), max(corners)
    return shortest, longest, trajectory.forward_arc(env, r, v, gate)


def deorbit_centring(env, r, v, mass, cfg, end, gate, window_note=None):
    """How far the gate sits from the centre of that window, doubled.

    Signed so that it *falls* as retrograde dv is added, which is the
    convention ``deorbit_remaining`` divides by: more dv takes energy out, so
    both ends of the window come closer and the gate moves toward -- and then
    past -- the centre.

    Zero is the burn to take.  Centring is the whole rule and it needs no
    constant: it is the arrival from which the glide has the most room to be
    wrong in either direction, which is the only thing the old one-sided aim
    and its tolerance band were ever trying to express.
    """
    window = deorbit_window(env, r, v, mass, cfg, end, gate)
    if window is None:
        return None
    shortest, longest, needed = window
    # **This airframe has a minimum entry range, and the propagator does not
    # believe in it.**  The deorbit waits for a pass and commits to the first
    # one it can centre; some passes put the runway 2133-2184 km ahead
    # instead of the usual 2244-2254, and the search takes them, because
    # from inside the model a 28 m/s burn centres that arrival as neatly as
    # a 27 m/s burn centres the other.  Flown, 13 of 41 flights took such a
    # pass and **every one of them arrived long, mean +36 km**, against
    # +5.3 km for the 28 that did not.  Refusing them alone takes the
    # arrival scatter from sd 20.2 km to 10.3 and the bias from +14.9 km to
    # +5.3.
    #
    # So this is a floor under the *arc the glide is asked to fly*, and it
    # stands in for the propagator's over-prediction of how short an entry
    # this vehicle can make -- which is the real missing model, in failure
    # 19's sense.  What would contradict it: a flight that lands on the
    # runway from a pass under it, or a re-measurement of the glide's
    # shortest achievable entry.  ``conesum.py``'s sibling for the entry
    # does not exist yet; until it does, the number is the split in the
    # data (2184 against 2244) and is written where it can be seen.
    if needed < cfg.DEORBIT_COMMIT_ARC_MIN_M:
        return None
    # **The width of this window is not a measure of its quality, and the
    # obvious reading of it is exactly backwards.**  Reaching for it is
    # tempting: the burn this search picks leaves a window **2 to 4 km wide
    # over a 2250 km entry**, which looks like a glide with no authority at
    # all.  Measured over 41 flights, the passes that come with a *wider*
    # window -- 11 to 22 km -- are the ones that land **+36 km** out, and
    # the 2-4 km ones land +5.3.  A minimum-width requirement would have
    # selected the bad pass every time.  The width tracks how steep the
    # entry is, not how well it is known.
    if longest - shortest < cfg.DEORBIT_MIN_WINDOW_M:
        return None
    # ``DEORBIT_WINDOW_BIAS`` moves the target off centre as a share of the
    # window's *own* width, which is what makes it a policy rather than a
    # fit: the same number means the same thing on an airframe with twice the
    # crossrange or a runway twice as far round the planet.
    # **Positive is the safe side, and getting that backwards is easy.**  The
    # gate sitting near the *long* edge of the window means the vehicle can
    # only just reach it -- it must fly its longest -- which is aiming
    # short.  Near the short edge it overflies unless it shortens, which is
    # the recoverable side and what "aim long" means.  So a positive bias
    # moves the *target arrival* down, and the burn that meets it is a
    # smaller one.
    want = 0.5 * (shortest + longest) - \
        0.5 * cfg.DEORBIT_WINDOW_BIAS * (longest - shortest)
    # **An offset in metres, because the window is too narrow to express one
    # as a share of itself.**  ``DEORBIT_WINDOW_BIAS`` is a fraction of the
    # width and the width is 2-4 km, so the whole knob can move the target
    # by half a kilometre.  The measured bias it has to answer is **+5.3 km**
    # over 28 flights -- the vehicle flies further than the propagator says
    # it will, which is the same over-prediction ``DEORBIT_LONG_BIAS_M`` was
    # fitted against before the window replaced it, and the window path
    # left that constant connected only to the burn's *stop test* and not to
    # which burn is chosen at all.
    #
    # Positive aims **short**: it asks the search for an arrival that many
    # metres inside the gate, which is the direction a vehicle that lands
    # long needs.  A constant offset does not change the slope, so
    # ``deorbit_remaining``'s taper still divides by the same gradient.
    # **Say what the search was actually offered.**  Two knobs meant to move
    # this aim were flown and neither moved the landing by a metre:
    # ``DEORBIT_LONG_BIAS_M`` reaches only the burn's stop test (failure 33),
    # and ``DEORBIT_CENTRE_BIAS_M`` at both -2500 and -17000 left the
    # arrival within noise of unbiased.  A batch can only report that the
    # landing did not move; it cannot say whether the knob reached the
    # decision.  These five numbers can, in one flight.
    if window_note is not None:
        window_note(shortest, longest, needed, want,
                    float(cfg.DEORBIT_CENTRE_BIAS_M))
    return 2.0 * (want - needed + cfg.DEORBIT_CENTRE_BIAS_M)


def deorbit_aim(cfg, prediction):
    """How far past the gate this entry should be aimed, in metres.

    **One definition, shared by the search and by the burn's stop test**, and
    that sharing is not tidiness.  ``deorbit_solution`` picks the dv whose
    arrival sits on the aim and ``deorbit_remaining`` decides when the engine
    has delivered it; if they disagree the burn overruns or stops short of
    the trajectory that was actually chosen, by exactly the difference.  When
    the aim was a single constant they could not disagree, and making it a
    function of the entry is precisely the change that lets them -- so it is
    a function both of them call.

    A fraction of the arc flown *inside the atmosphere*, floored: see
    ``deorbit_solution`` for why the scale is the entry's own length and not
    the distance the runway currently happens to be.
    """
    # **A floor under a fraction, not a floor under the answer.**  With the
    # fraction retired (see ``DEORBIT_LONG_BIAS_FRACTION``) the constant *is*
    # the aim, and a ``max`` then quietly refuses to aim short: ``-3000``
    # and ``0`` are the same flight, which is exactly the "read the numeric
    # value, never the label" trap this project has already paid for once.
    fraction = float(cfg.DEORBIT_LONG_BIAS_FRACTION)
    if fraction == 0.0:
        return float(cfg.DEORBIT_LONG_BIAS_M)
    return max(float(cfg.DEORBIT_LONG_BIAS_M),
               fraction * prediction.entry_arc)


def deorbit_chosen_aim(env, r, v, mass, cfg, end, dv):
    """How far past the gate the *chosen* burn lands, at the nominal schedule.

    The burn's stop test measures the arrival against an offset from the gate
    (see ``deorbit_progress``), and with the authority window there is no
    fitted offset to measure against -- so it is read off the solution the
    search actually took.  One propagation, once, at commit time.

    This keeps the sharing discipline ``deorbit_aim`` was written for: the
    number the search chose is the number the stop test tracks.  What has
    changed is only where it comes from -- a corner-propagated window instead
    of ``DEORBIT_LONG_BIAS_FRACTION``.
    """
    speed = vec.norm(v)
    if speed <= 0.0:
        return None
    retro = vec.scale(v, -1.0 / speed)
    v2 = vec.add(v, vec.scale(retro, dv))
    gate = env.runway.gate(end)
    steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                  cfg=cfg, mass=mass)
    prediction = trajectory.predict(env, r, v2, mass, cfg, steer=steer,
                                    target_radius=vec.norm(gate))
    if not prediction.reached or prediction.skipped:
        return None
    needed = trajectory.forward_arc(env, r, v2, gate)
    carried = trajectory.forward_arc(env, r, v2, prediction.position)
    return carried - needed


def deorbit_progress(env, r, v, mass, cfg, end, aim=None, record=None):
    """Signed range error if the engine were shut down *now*.

    One propagation, which is what makes it usable as the burn's stop test:
    the full search is sixteen, and running that every tick of a burn is both
    slow and the wrong question.  The right question during a burn is the one
    boosterland asks of boostback -- "does the trajectory I am already on do
    the job?" -- because that needs no model of how much longer to thrust.

    The first version stopped when the *search* stopped finding a burn above
    ``DEORBIT_DV_MIN``, which cannot work: as the burn proceeds the dv still
    required falls towards zero, the search floor is 10 m/s, and the condition
    only trips once the requirement has fallen through the floor and the
    debounce has run -- about ten seconds of extra thrust at 13 m/s^2, or
    130 m/s of overburn on a 60 m/s burn.
    """
    gate = env.runway.gate(end)
    steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                  cfg=cfg, mass=mass)
    prediction = trajectory.predict(env, r, v, mass, cfg, steer=steer,
                                    gate=gate, end=end,
                                    target_radius=vec.norm(gate))
    # **Hand the whole prediction back when asked.**  The burn already pays
    # for this propagation every tick, and it passes through both the
    # atmosphere boundary and ``ENTRY_INTERFACE_M`` on its way to the gate,
    # so anything wanting the predicted handover state can read it here
    # rather than running a second one.  See ``Prediction.entry_position``.
    if record is not None:
        record["prediction"] = prediction
    if prediction.time_to_go > cfg.DEORBIT_MAX_TIME_TO_GO_S:
        # An arrival a revolution away is not this entry's arrival.
        return None
    if prediction.skipped:
        # The arc goes into the air and comes back out of it.  That is not a
        # deorbit, whatever range it eventually reports, so the burn is not
        # finished: ``None`` keeps it thrusting on the solved dv.
        return None
    if not prediction.reached:
        # **Not a number, None.**  The first version returned a large positive
        # sentinel for "the arc never got down to the gate", which is exactly
        # the state a deorbit burn is in before it has done anything -- and
        # the burn's stop test is ``progress >= bias``, which a sentinel
        # satisfies.  In game that ended the burn after 35 kg of propellant:
        # ``DEORBIT -> DRAIN range error +10000000 m``.  A missing answer must
        # not be allowed to look like a good one.
        return None
    needed = trajectory.forward_arc(env, r, v, gate)
    carried = trajectory.forward_arc(env, r, v, prediction.position)
    offset = deorbit_aim(cfg, prediction) if aim is None else aim
    return carried - needed - offset


def deorbit_remaining(env, r, v, mass, cfg, end, aim=None, record=None):
    """How much more retrograde dv the burn still owes, measured.

    Two propagations: where the current arc lands, and where it would land
    with ``DEORBIT_PROBE_DV`` more taken off.  The difference is the range
    sensitivity in metres per m/s, and the burn's remaining dv is the error
    divided by it -- boosterland's ``miss_gradient``, doing the same job.

    It exists because a debounced threshold cannot end this burn.  The
    sensitivity here is about 25 km of range per m/s, so the three-tick
    debounce that made boosterland's boostback exit repeatable is, at full
    throttle on a 2 s tick, seventy-odd m/s of overburn: in game that ended
    burns 85, 99, 128 km and once 1957 km *short* of the aim.  A burn whose
    last tick is worth 25 km has to be tapered, not debounced.

    Returns ``(error, dv)`` and either may be ``None`` -- the error when the
    arc does not reach the gate at all, the dv when the two propagations
    disagree too little to divide by.
    """
    error = deorbit_progress(env, r, v, mass, cfg, end, aim, record=record)
    if error is None:
        return None, None
    speed = vec.norm(v)
    if speed <= 0.0:
        return error, None
    probe = vec.add(v, vec.scale(v, -float(cfg.DEORBIT_PROBE_DV) / speed))
    probed = deorbit_progress(env, r, probe, mass, cfg, end, aim)
    if probed is None:
        return error, None
    span = float(cfg.DEORBIT_PROBE_DV)
    gradient = (probed - error) / span
    if gradient > -float(cfg.DEORBIT_MIN_GRADIENT):
        # More retrograde dv is not shortening the flight by enough to be
        # worth dividing by.  Fall back to the raw error rather than command
        # a burn length off a number the probes do not support.
        return error, None
    # ``error`` is already net of the aim (see ``deorbit_progress``), so what
    # the burn owes is the error itself divided by the sensitivity.
    owed = error / (-gradient)

    # **Re-probe at the step actually being contemplated.**  The first probe
    # is a tangent taken over a fixed 2 m/s, and the sensitivity here is not
    # a fixed thing: it is ~25 km of range per m/s on a shallow entry state
    # and ~290 km on a steep one, so on the steep ones that tangent is
    # measured across nearly 600 km of range and is nothing like local.  The
    # burn then ends owing half a m/s it does not know about, which at 290 km
    # per m/s is the -73 km and -183 km two flights exited at, against
    # -0.01 m/s and ~50 m on the two shallow ones in the same batch.
    #
    # Re-measuring with the probe set to ``owed`` turns the tangent into a
    # secant across exactly the step being taken, which is the right
    # linearisation to invert.  It costs one more propagation and only while
    # the burn is closing.
    if owed > 0.0:
        # Unconditionally, not just once ``owed`` is inside one probe width.
        # Gated on ``owed < span`` a badly-scaled tangent steps straight past
        # the window and the secant never runs at all: one flight exited
        # 436 km short still owing 2.12 m/s, on a state whose sensitivity is
        # ~200 km per m/s.  Probing at the contemplated step costs one
        # propagation and is always the right linearisation to invert.
        probe = vec.add(v, vec.scale(v, -min(owed, span * 50.0) / speed))
        probed = deorbit_progress(env, r, probe, mass, cfg, end, aim)
        if probed is not None:
            gradient = (probed - error) / min(owed, span * 50.0)
            if gradient <= -float(cfg.DEORBIT_MIN_GRADIENT):
                owed = error / (-gradient)
    return error, owed


def deorbit_commits(env, r, v2, mass, cfg, gate, steer):
    """Does this post-burn state fly an entry, or bounce back out?

    The predicate the shallow search bisects on.  Four ways to fail and they
    are all "not an entry": never gets to the gate's altitude, skips out of
    the air and comes back later, hits the ground first, or arrives a
    revolution away.
    """
    prediction = trajectory.predict(env, r, v2, mass, cfg, steer=steer,
                                    target_radius=vec.norm(gate))
    if not prediction.reached or prediction.skipped or prediction.grounded:
        return False, prediction
    if prediction.time_to_go > cfg.DEORBIT_MAX_TIME_TO_GO_S:
        return False, prediction
    return True, prediction


def drag_switch_for(env, r, v2, mass, cfg, end, gate, needed):
    """The airspeed at which to stop making drag and start making lift.

    **The entry's range control, and it costs nothing.**  ``ENTRY_MAX_DRAG``
    flies the vehicle broadside, which is five times the drag and *zero*
    lift -- so it cannot steer and it cannot stretch.  Giving the angle of
    attack back at a chosen speed restores both: switch early and the entry
    is long, switch late and it is short, and the answer is a single number
    that the propagator can solve for.

    That replaces the constant this started as.  A fixed Mach 3 is the shape
    this project has a rule against -- a number standing in for a quantity
    the program can compute -- and worse, it is the *wrong* number to be
    fixed, because it decides the entry's length and the entry's length is
    what decides where the deorbit burn goes.  Solved here, the burn and the
    entry agree with each other by construction.

    Monotone, so a bisection is honest: a higher switch speed means less time
    broadside, more lift and a longer arc.  Returns ``None`` when the gate is
    outside the span -- flying the whole entry for lift still falls short, or
    flying all of it for drag still overflies -- which is the signal to wait
    rather than to commit to an entry that cannot reach.

    A closed form was tried and does not survive contact with the numbers:
    the equilibrium-glide range equation reads 647 km where this airframe
    flies 2317, because the arc is flown near circular speed with centrifugal
    lift carrying it and the vehicle nowhere near equilibrium glide.
    """
    speed = vec.norm(v2)
    radius = vec.norm(gate)

    # **And the switch has a floor that is not about range at all.**  The
    # solve above will happily walk the switch to zero -- fly broadside all
    # the way to the gate -- because that is the shortest arc and the
    # shortest arc is what a long entry wants.  It is also a vehicle that
    # arrives over the gate with no lift, no steering and no speed, which is
    # not a landing.  So a candidate only counts if it reaches the gate
    # *flying*: at or above the arrival speed the whole landing chain is
    # sized on, which is the same ``GLIDE_ARRIVAL_FACTOR x stall`` the glide
    # already caps itself with.  A prediction that cannot be landed is not a
    # bound on anything.
    floor = cfg.GLIDE_ARRIVAL_FACTOR * airframe.stall(env, cfg)

    def arc_for(until):
        steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                      cfg=cfg, mass=mass, drag_until=until)
        prediction = trajectory.predict(env, r, v2, mass, cfg, steer=steer,
                                        target_radius=radius)
        if (not prediction.reached or prediction.skipped
                or prediction.grounded):
            return None
        if prediction.speed < floor:
            return None
        return trajectory.forward_arc(env, r, v2, prediction.position)

    # The two ends of the span: all lift (the longest entry this vehicle
    # flies) and all drag (the shortest).
    longest = arc_for(speed * 2.0)
    shortest = arc_for(0.0)
    if longest is None:
        return None
    if shortest is None:
        # All-drag does not arrive flyable, which is the usual case: the
        # bracket's short end is then the shortest arc that *does*, and the
        # bisection below finds it because ``arc_for`` refuses the rest.
        shortest = 0.0
    if not (shortest <= needed <= longest):
        return None
    lo, hi = 0.0, speed * 2.0
    for _ in range(int(cfg.DRAG_SWITCH_PASSES)):
        mid = 0.5 * (lo + hi)
        arc = arc_for(mid)
        if arc is None:
            # A candidate that does not fly is not a bound; walk away from it
            # toward the lifting end, which is the one that always does.
            lo = mid
            continue
        if arc < needed:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def deorbit_shallowest(env, r, v, mass, cfg, end):
    """The smallest burn that still commits, and whether to light it now.

    **A different question from the one the rest of this file asks, and a
    much better conditioned one.**  ``deorbit_solution``'s grid searches over
    *range*, which is not monotone in dv on this airframe -- it peaks around
    50-80 m/s -- so the search has to be a grid and its answer has to be
    checked against a window.  "Does this burn commit?" is monotone: more
    retrograde dv lowers the periapsis and nothing about that reverses.  So
    this is a bisection, it converges to a tenth of a m/s in a dozen
    propagations, and the answer it converges to is a property of the
    vehicle and the atmosphere rather than of a fitted constant.

    **What it is for.**  The deorbit burn is the only irreversible decision
    in the flight and the only one that spends propellant.  Taking the
    *shallowest* burn that still captures is the least propellant the
    trajectory can be bought with -- which is free on the test craft and is
    not free on the next one.  It pairs with ``ENTRY_MAX_DRAG``: a broadside
    entry makes ``ClA`` exactly zero, so there is nothing to skip on, and
    measured offline the threshold moves **16 -> 12 m/s** when the entry is
    flown for drag.  Less lift is what makes a shallower capture safe, and
    more drag is what makes it capture at all.

    **Then the range has to come from somewhere, and it comes from the
    clock.**  With the dv pinned at the threshold, where the vehicle lands is
    set by *when* the burn is lit: the entry's arc is a property of the
    trajectory and the arc still to run shrinks as the vehicle coasts, so
    the difference crosses zero exactly once per revolution.  That crossing
    is the ignition point.  Returns ``(dv, miss)`` at the crossing and
    ``(None, miss)`` everywhere else, which is the same contract
    ``deorbit_solution`` already has with a phase that knows how to wait.

    Note what this gives up: at 90 degrees of alpha there is no lift, so
    there is no bank steering either.  The entry cannot correct downrange and
    is not asked to -- which is the point of putting the targeting in the
    ignition time.
    """
    speed = vec.norm(v)
    if speed < 1.0:
        return None, 0.0
    retro = vec.scale(v, -1.0 / speed)
    gate = env.runway.gate(end)
    needed = trajectory.forward_arc(env, r, v, gate)
    steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                  cfg=cfg, mass=mass)

    def at(dv):
        return deorbit_commits(env, r, vec.add(v, vec.scale(retro, dv)),
                               mass, cfg, gate, steer)

    lo, hi = float(cfg.DEORBIT_DV_MIN), float(cfg.DEORBIT_DV_MAX)
    ok, prediction = at(hi)
    if not ok:
        # Nothing in range commits from here.  Not a solution, and signed the
        # way the phase reads "keep waiting".
        return None, 0.0
    if at(lo)[0]:
        # Already committed at the smallest burn considered: the threshold is
        # below the search, so the smallest burn *is* the answer.
        threshold = lo
    else:
        for _ in range(int(cfg.DEORBIT_SHALLOW_PASSES)):
            mid = 0.5 * (lo + hi)
            if at(mid)[0]:
                hi = mid
            else:
                lo = mid
        threshold = hi
    # **And then step off the threshold, because the threshold is a cliff.**
    # On the shallow side of it the vehicle skips and comes down a revolution
    # later somewhere else entirely, so the margin is not a tuning parameter
    # -- it is the burn's own execution error, and it belongs here until the
    # log's ``cutoff drift`` line has enough flights behind it to say what
    # that error actually is.  See ``DEORBIT_SKIP_MARGIN_MS``.
    margin = float(cfg.DEORBIT_SKIP_MARGIN_MS)
    dv = min(float(cfg.DEORBIT_DV_MAX), threshold + margin)
    ok, prediction = at(dv)
    while not ok and dv < float(cfg.DEORBIT_DV_MAX):
        # The bisection found *a* crossing of a predicate that is monotone in
        # principle and measured on a propagator: verify rather than assume.
        dv = min(float(cfg.DEORBIT_DV_MAX), dv + margin)
        ok, prediction = at(dv)
    if not ok:
        return None, 0.0
    carried = trajectory.forward_arc(env, r, v, prediction.position)
    miss = carried - needed
    # **And the gate has to be inside the glide's authority, which this
    # path skipped.**  ``deorbit_solution``'s centring branch checks
    # ``shortest <= want <= longest`` before it returns; this one returned
    # before reaching it, and the first flight duly committed to a burn whose
    # own window topped out at 1854 km with the gate at 1949 -- logged, in
    # capitals, by the very next line -- and arrived **484 km short**
    # (``logs/LOG2868``).  A search that reports a solution its own bound
    # refuses is failure 33's shape: the log faithfully prints the number
    # that was ignored.
    v2 = vec.add(v, vec.scale(retro, dv))
    window = deorbit_window(env, r, v2, mass, cfg, end, gate)
    if window is None:
        return None, miss
    shortest, longest, want = window
    if not (shortest <= want <= longest):
        return None, miss
    if getattr(cfg, "ENTRY_MAX_DRAG", False):
        # **Solve where the entry stops braking, here, before the burn.**
        # The broadside entry has no lift, so it has no range authority and
        # no steering; what it has instead is a *switch* -- the speed at
        # which the angle of attack goes back to the range solve -- and that
        # one number spans the whole reachable arc.  Solving it at this
        # moment is what makes the burn and the entry consistent: the arc the
        # search is judging is the arc the vehicle will actually fly.
        #
        # It also turns the ignition point from an instant into a *window*.
        # With the dv pinned and nothing else to trim with, the arrival lands
        # on the gate at one moment per revolution; with the switch solved,
        # any moment whose gate lies between the all-drag and all-lift arcs
        # will do, and the switch takes up the difference.
        switch = drag_switch_for(env, r, v2, mass, cfg, end, gate, needed)
        if switch is None:
            return None, miss
        return dv, miss
    if abs(miss) > max(500.0, float(cfg.DEORBIT_TOLERANCE_M)):
        # Right burn, wrong moment.  Wait: the arc still to run is shrinking
        # and this crosses zero once a revolution.
        return None, miss
    return dv, miss


def deorbit_solution(env, r, v, mass, cfg, end):
    """The retrograde dv that puts the entry's gate crossing on the gate.

    A scan and then a bisection, each step a full entry propagation flown at
    the entry schedule -- which is affordable because nothing in orbit happens
    fast and the loop there ticks in seconds, not tenths.

    The answer is deliberately biased *long*.  This is the one irreversible
    decision in the flight: everything after it can shorten the glide -- angle
    of attack brakes, bank sinks the vehicle into thicker air -- and nothing
    can lengthen it, because there is no thrust left once the tanks are
    drained.  So the burn aims past the gate by ``DEORBIT_LONG_BIAS_M`` and
    lets the glide spend the surplus.  Returns ``(dv, long_miss_at_dv)``, or
    ``(None, best_long)`` when no dv in range reaches the gate at all -- which
    is the signal to stay in orbit another pass rather than commit.
    """
    if getattr(cfg, "DEORBIT_SHALLOWEST", False):
        return deorbit_shallowest(env, r, v, mass, cfg, end)
    speed = vec.norm(v)
    if speed < 1.0:
        return None, 0.0
    retro = vec.scale(v, -1.0 / speed)
    # Predict the entry the glide will actually fly.  Bank cannot go below
    # ``SOLVE_BANK_MIN_DEG`` once the glide has the controls, so a deorbit
    # solved against a wings-level entry is solved against a trajectory the
    # vehicle is not allowed to fly -- and a bank floor *lengthens* the
    # reachable range on this airframe (1730 km at 0 degrees against 1872 at
    # 30), so the error is not even in the safe direction.
    steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                  cfg=cfg, mass=mass)
    gate = env.runway.gate(end)
    radius = vec.norm(gate)

    # How far round the vehicle has to go to reach the gate, the way it is
    # actually travelling.  Fixed for the whole search: every candidate burn
    # starts from the same state.
    needed = trajectory.forward_arc(env, r, v, gate)

    # **The aim is a fraction of the entry, not a distance -- and not a
    # fraction of how far away the runway currently is.**
    #
    # What the bias compensates is the propagator's over-prediction of the
    # glide's range, which is a *relative* error: a longer entry accumulates
    # more of it.  A fixed number is therefore right for one entry length and
    # wrong for every other -- measured, a fixed 300 km aim puts three states
    # with ~1100 km to run within 4-20 km while sending the elliptical save,
    # which has 728 km to run, 57-60 km long into the sea with alpha at its
    # stop and 60 degrees of bank on.  The glide had spent everything it had;
    # the aim was simply past its authority.
    #
    # The scale is ``Prediction.entry_arc`` -- the arc flown *inside the
    # atmosphere* -- and taking it from the candidate rather than from
    # ``needed`` is the whole point.  Scaled against ``needed`` the aim
    # shrinks as the vehicle coasts closer, so **waiting makes the target
    # easier**: a pass that cannot be solved becomes solvable by doing
    # nothing, and one flight duly sat until 410 km to run and then committed
    # on ``DEORBIT_DV_MAX`` exactly.  The entry's own length is a property of
    # the trajectory being judged and does not move while the phase waits.
    def aim_for(prediction):
        return deorbit_aim(cfg, prediction)

    def miss_for(dv):
        """Signed metres past this candidate's *own* aim.

        Returning the error against the aim, rather than the raw arrival,
        means every candidate is judged against the aim its own entry length
        earns -- so a steep short entry and a shallow long one are compared
        on the same footing.
        """
        v2 = vec.add(v, vec.scale(retro, dv))
        prediction = trajectory.predict(env, r, v2, mass, cfg, steer=steer,
                                        target_radius=radius)
        if not prediction.reached:
            # Never reached the gate altitude in the time allowed, or hit the
            # ground first.  Both are "not a solution"; signing them keeps a
            # search from wandering into them.
            return (1e7 if not prediction.grounded else -1e7)
        if prediction.time_to_go > cfg.DEORBIT_MAX_TIME_TO_GO_S:
            # Comes down, but on a later pass.  Signed like an undershoot for
            # the same reason as the skip below: more dv is what brings the
            # arrival forward.
            return -1e7
        if prediction.skipped:
            # **The entry has to commit.**  A burn that leaves the periapsis
            # high does not fail the range test -- the arc dips into the air,
            # maximum-lift alpha pushes it straight back out, and it comes
            # down somewhere on a later pass, which the propagation dutifully
            # reports as a range.  Signed like an undershoot so the search
            # walks away from it towards more dv, which is the direction that
            # actually lowers the periapsis.
            return -1e7
        # Signed range error along the ground track, positive when the arc
        # carries past the gate, less the aim this candidate earns.  Arc
        # lengths, not a projection onto the runway's local heading: see
        # ``trajectory.forward_arc``.
        carried = trajectory.forward_arc(env, r, v, prediction.position)
        return carried - needed - aim_for(prediction)

    def centring_for(dv):
        """The same question asked of the glide's own authority."""
        v2 = vec.add(v, vec.scale(retro, dv))
        answer = deorbit_centring(env, r, v2, mass, cfg, end, gate)
        # A state with no usable entry at either corner is not a solution,
        # and it is signed like an undershoot so the search walks toward more
        # dv -- which is the direction that brings an arc down at all.
        return -1e7 if answer is None else answer

    # A grid, not a bisection.  Range against dv is **not monotonic** on this
    # airframe -- offline it peaks around 50-80 m/s and falls away on both
    # sides, because a shallower entry decelerates higher and then glides
    # slowly, while a steeper one arrives with less time to glide at all.  A
    # bracket-and-bisect assumes a monotone function and would happily
    # converge on the wrong side of the hump.
    score = centring_for if cfg.DEORBIT_AUTHORITY_WINDOW else miss_for
    steps = max(8, int(cfg.DEORBIT_SEARCH_STEPS))
    coarse = max(6, steps // 2)
    lo, hi = cfg.DEORBIT_DV_MIN, cfg.DEORBIT_DV_MAX
    grid = []
    for i in range(coarse):
        dv = lo + (hi - lo) * i / (coarse - 1.0)
        grid.append((dv, score(dv)))

    def refine(around, width):
        out = []
        for i in range(5):
            dv = max(lo, min(hi, around - width + 2.0 * width * i / 4.0))
            out.append((dv, score(dv)))
        return out

    # **Refine until the step is smaller than the band, not once.**  One pass
    # leaves the grid at ~16 m/s, and the range sensitivity here runs 25 to
    # 290 km per m/s -- so a single refinement resolves the arrival to
    # hundreds of kilometres and then asks whether it is inside a 25 km
    # window.  It usually is not, and "no solution" on a perfectly reachable
    # pass sends the phase round another orbit.  Each pass quarters the
    # width, so five of them take ~32 m/s to well under one, for the cost of
    # five propagations apiece in a phase that ticks in seconds.
    # ``wanted`` is zero here: ``miss_for`` already reports the error against
    # each candidate's own aim, so "on target" is simply zero for all of them.
    wanted = 0.0
    width = (hi - lo) / (coarse - 1.0)
    for _ in range(int(cfg.DEORBIT_REFINE_PASSES)):
        best = min(grid, key=lambda x: abs(x[1] - wanted))
        grid += refine(best[0], width)
        width /= 4.0

    # Among the solutions that are good enough, take the *smallest* burn.  Not
    # to save propellant -- it is about to be thrown overboard anyway -- but
    # because a shallower entry is a longer one, and every extra minute in the
    # air is more time for the glide solve to spend its authority before the
    # gate.  Steeper entries also arrive hotter and with less margin.
    # **The band is one-sided, because the constraint is.**  Everything after
    # this burn can shorten the glide and nothing can lengthen it, so a
    # candidate that lands *short* of the aim is not an acceptable solution at
    # any tolerance -- it is the one outcome the whole design is built to
    # avoid.  The first version accepted ``abs(m - wanted) <= tolerance`` and
    # then took the smallest burn in the band, which is the shortest arrival
    # it was allowed to pick: the aim said "50 km long" and the rule quietly
    # took "up to 50 km short", turning a safety margin into its opposite.
    #
    # That is also why sweeping ``DEORBIT_LONG_BIAS_M`` looked like it did
    # nothing.  The tolerance was *derived from the bias*, so raising the aim
    # widened the band underneath it by exactly as much and the selection
    # stayed on the same shallow candidates -- across a 110 km sweep the
    # landings moved -53, -78, -65, -57 km, which reads as "the aim is not
    # the lever" and was really "the aim is not connected".
    if cfg.DEORBIT_AUTHORITY_WINDOW:
        # Centring is a single-valued target, so there is no band to accept
        # inside of and no "which of the acceptable ones" rule to get wrong.
        # What still has to be checked is that the gate is inside the window
        # at all: ``m`` near zero says the gate is near the *centre* of the
        # span, and a span the vehicle cannot fly is centred on nothing.
        best = min((row for row in grid if abs(row[1]) < 1e6),
                   key=lambda x: abs(x[1]), default=None)
        if best is None:
            return None, grid[0][1] if grid else 0.0
        v2 = vec.add(v, vec.scale(retro, best[0]))
        window = deorbit_window(env, r, v2, mass, cfg, end, gate)
        if window is None:
            return None, best[1]
        shortest, longest, want = window
        if not (shortest <= want <= longest):
            return None, best[1]
        return best

    tolerance = max(500.0, float(cfg.DEORBIT_TOLERANCE_M))
    good = [(dv, m) for dv, m in grid
            if abs(m) < 1e6 and wanted <= m <= wanted + tolerance]
    if good:
        # **Among arrivals that are all long enough, the steepest.**  The
        # original rule took the smallest burn, for time in the air and for
        # heating, and both were measured and neither survived.  Steeper is
        # better here for a reason that is about the *model* rather than the
        # vehicle: a shallow entry spends its energy high, in thin air, over
        # a long arc -- which is the regime the propagator is least reliable
        # in and the one where the skip lives (failure 8) -- while a steep one
        # commits, and its prediction is correspondingly shorter-range and
        # better tested.  The heating argument points the same way once
        # measured: the skin peaked at 0.73 of limit on a steep entry against
        # 0.78 on the shallowest of the same batch, because the shallow entry
        # is the *long* one and it is the time that heats it.
        #
        # Flown both ways on three entry states, with the old two-sided band:
        # smallest landed -75.0, -62.7, -57.4 km and largest -65.2, -63.0,
        # -57.7. With this one-sided band and the smallest rule the search
        # resolves the shallowest burn that just reaches the aim and it is
        # worse again (-71.5, -79.7). The lever is weak either way -- it is
        # worth about 10 km of a 60 km shortfall -- but its sign is now
        # measured three times.
        dv, m = max(good, key=lambda x: x[0])
        return dv, m
    best = min(grid, key=lambda x: abs(x[1] - wanted))
    return None, best[1]
