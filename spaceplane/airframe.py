"""What the swept aerodynamic table says about the aircraft it describes.

The constants that decide the landing -- the stall speed, the best glide
ratio, where the gate goes, how fast the approach is flown -- were all
transcribed by hand out of ``planeprobe``'s file, and that file is wrong by
about 1.8x subsonically (spaceplane failure 13).  Every one of them was
therefore set for an aircraft that does not exist, and nothing in the project
could contradict them, because a measurement whose only consumer is a
hand-copied constant cannot be contradicted.

``Autopilot`` already sweeps the real airframe's coefficients in STANDBY, and
that table agrees with what the vehicle goes on to do (measured in flight:
ClA 23.3 against 22.9 at 12 degrees, 26.2 against 26.8 at 16).  So the
numbers can be *derived*, once, from the aircraft that is actually about to
be flown -- which is what makes the same autopilot fly a different one.

**The one place the table is not trustworthy is the top of the lift curve.**
At 30 degrees it reads ClA 50 where the airframe delivers 34: the vehicle is
rarely steady there and the table is a hypothetical.  Maximum lift is exactly
what a stall speed is, so that error lands squarely on the number that
matters most, and it errs *optimistic* -- a stall speed 20% low, which is the
dangerous direction for an approach speed derived from it.  ``MARGIN`` is
that discount, and it is one stated assumption in place of four transcribed
constants.
"""
import math

SEA_LEVEL_MACH = 0.25           # where the subsonic polar is read
# The table over-reads the top of the lift curve; see the module docstring.
# 0.70 of the tabulated maximum is what this airframe actually delivers
# (34 against 50), and it is applied as a *lift* discount so the stall speed
# it implies is conservative.
MARGIN = 0.70


class Airframe:
    """Stall speed and best glide, read off the table in STANDBY.

    Every field is ``None`` when the table cannot answer -- not a plausible
    default.  A landing flown on an invented stall speed is the failure this
    module exists to end, and quietly inventing one here would be that
    failure with a different author.
    """

    def __init__(self, stall_speed=None, best_ld=None, best_alpha=None,
                 best_speed=None, max_cla=None, discount=MARGIN,
                 measured_discount=False, stall_alpha=None):
        self.stall_speed = stall_speed
        self.best_ld = best_ld
        self.best_alpha = best_alpha
        self.best_speed = best_speed
        self.max_cla = max_cla
        # The angle the table's lift peaks at -- where this
        # wing stalls.  ``ALPHA_MAX_DEG`` is a constant for
        # the same thing and says so: "past 30 the lift curve
        # turns over".  Past 30 on *this* wing; the other
        # aircraft on disk turns over at 25.
        self.stall_alpha = stall_alpha
        self.discount = discount
        # Whether ``discount`` came off the vehicle or out of ``MARGIN``.
        # A log that does not say which cannot be read later.
        self.measured_discount = measured_discount

    def __repr__(self):                                     # pragma: no cover
        return ("Airframe(stall=%s, best_ld=%s at %s deg, %s m/s)"
                % (self.stall_speed, self.best_ld, self.best_alpha,
                   self.best_speed))

    def describe(self):
        if self.stall_speed is None:
            return "airframe: the table could not be read"
        return ("airframe, from its own swept table: stall %.1f m/s "
                "(ClA max %.1f, discounted %.0f%% %s), best glide L/D %.2f "
                "at %.0f deg and %.1f m/s"
                % (self.stall_speed, self.max_cla,
                   100.0 * (1.0 - self.discount),
                   "measured off this vehicle" if self.measured_discount
                   else "from MARGIN, nothing measured yet",
                   self.best_ld, self.best_alpha, self.best_speed))


def measure(env, mass, gravity, altitude=0.0):
    """Read the subsonic polar out of ``env`` and reduce it to four numbers.

    ``mass`` is the mass the *landing* is flown at -- the drained one -- not
    whatever is aboard now: the stall speed of a vehicle carrying 2.8 t of
    propellant it is about to throw away is not the stall speed of the
    aircraft that lands.
    """
    if env is None or not getattr(env, "ready", lambda: False)():
        return Airframe()
    alphas = sorted(getattr(env, "_alphas", ()) or ())
    if not alphas:
        return Airframe()
    speed = SEA_LEVEL_MACH * max(1.0, env.mach(1.0, altitude) ** -1.0)
    rho = env.density(altitude)
    if rho <= 0.0:
        return Airframe()

    best = None
    peak = 0.0
    peak_alpha = None
    for alpha in alphas:
        cla, cda = env.coefficients(alpha, speed, altitude)
        if cla <= 0.0 or cda <= 0.0:
            continue
        if cla > peak:
            peak, peak_alpha = cla, alpha
        ratio = cla / cda
        if best is None or ratio > best[1]:
            best = (alpha, ratio, cla)
    if best is None or peak <= 0.0:
        return Airframe()

    alpha, ratio, cla = best
    weight = mass * gravity
    discount = MARGIN
    stall = math.sqrt(2.0 * weight / (rho * peak * discount))
    glide = math.sqrt(2.0 * weight / (rho * cla))
    return Airframe(stall_speed=stall, best_ld=ratio, best_alpha=alpha,
                    best_speed=glide, max_cla=peak, discount=discount,
                    measured_discount=False,
                    stall_alpha=peak_alpha)


def _derived(env, cfg, field, fallback):
    """The measured number when there is one, the configured one otherwise.

    ``AIRFRAME_DERIVED`` is the switch, and it is a switch rather than an
    unconditional change because the committed configuration lands and this
    moves four numbers under it at once -- ``pairfly.sh`` needs both arms to
    exist.  The fallback is the transcribed constant, which is also what the
    offline tests fly: ``boosterland/tests/fakeksp`` sweeps no table, so ``env`` carries
    nothing and every call here returns exactly what it returned before.

    The fallback is deliberately *not* a plausible invented value.  When the
    sweep fails in flight ``Autopilot.read_airframe`` says so in the log and
    the configured constant is flown -- a stated retreat to a known number,
    not a silent one to a made-up one.
    """
    if cfg is not None and not getattr(cfg, "AIRFRAME_DERIVED", False):
        return fallback
    measured = getattr(env, field, None)
    if measured is not None and measured > 0.0:
        return measured
    return fallback


def at_mass(env, speed):
    """``speed``, read off the table at ``env.stall_mass``, scaled to the
    vehicle's mass now (``env.vehicle_mass``): a speed at fixed lift
    coefficient goes as the square root of the weight.  Unscaled when either
    mass is unknown."""
    if speed is None:
        return None
    m0 = getattr(env, "stall_mass", None)
    m = getattr(env, "vehicle_mass", None)
    if m0 and m and m0 > 0.0 and m > 0.0:
        return speed * math.sqrt(m / m0)
    return speed


def stall(env, cfg):
    """The vehicle's stall speed (equivalent airspeed) at its mass now, off
    its own swept table -- or ``None`` before the table has been read.

    There is no configured fallback.  ``STALL_SPEED_M_S`` (48) was the old
    craft's, taken in flight as a *true* airspeed, and with
    ``STALL_CALIBRATION_M_S`` it rescaled every craft's table to the old
    craft's units; both are gone (the user, 2026-10-05).  The STANDBY
    reading is taken at the entry mass, 18% above the shuttle that flies
    the cone, hence ``at_mass``.
    """
    got = getattr(env, "stall_speed", None)
    if got is None or got <= 0.0:
        return None
    return at_mass(env, got)


def glide_ld(env, cfg):
    """Best glide ratio -- path per metre of height, flown at ``best_alpha``.

    This is the *straight* glide.  The cone flies a banked turn and faster
    than best glide, and both cost path per metre of height; ``turning_ld``
    is that number and it is derived from this one rather than configured
    beside it.  Failure 19's shape: one constant standing in for two things.
    """
    return _derived(env, cfg, "best_ld", cfg.APPROACH_BEST_LD)


# What ``turning_ld``'s arithmetic has to be divided by to become a number
# the cone can *plan* with.
#
# **Not a tracking overhead, and calling it one cost a batch.**  The story
# is worth keeping because it is the same mistake three times in one
# session.  ``Config.HAC_LD``'s comment decomposes the achieved ratio as
# "LD flown 1.86-1.90 over tracking 1.07-1.11", so 1.09 looked like the
# tracking term and the name followed.  Re-measuring with ``conesum.py``
# over 58 flights said tracking is really 1.27-1.39 on the configuration
# now flown, so the divisor was "corrected" to 1.36, giving cone_ld 1.49
# against the measured achieved ratio of 1.42.  That looked like a
# derivation finally landing on its measurement.
#
# Flown, it was a disaster: the cone left 3825 and 3903 m from the gate
# against the default arm's 1948 and 2075, and the vehicle stopped 5346 and
# 5208 m along a runway where the default stops at 2081.  Two flights an
# arm, no overlap, and ``HAC_LD``'s own comment had already written the
# result down -- "1.35 was the transcribed number and it was 25% low...
# the cone plans a shorter circle than the height can pay for, so it
# arrives over the gate high", ``logs/LOG1315``, overflew by 3.5 km into
# the sea.  1.49 is 20% low and overflew by 3.2 km.
#
# **The achieved ratio and the planning ratio are different quantities.**
# The cone must plan with a ratio *above* what it achieves, because the
# radius is re-solved every tick against remaining height: plan optimistic
# and the circle tightens as it goes, which converges; plan at what you
# actually achieve and any shortfall has nowhere to go but over the gate.
# The achieved ratio is 1.42 and the planning ratio is 1.86, and that gap
# is deliberate.
#
# So this is a calibration, in the same sense as
# ``Config.STALL_CALIBRATION_M_S``: its value is whatever makes
# ``turning_ld`` reproduce the committed planning ratio on the aircraft
# that ratio was fitted on (2.02 / 1.09 = 1.85 against ``HAC_LD`` 1.86).
# That is what makes the derivation inert on the reference craft and
# scaling on every other one, which is the only thing it is for.  It is
# *not* independently measurable, and ``conesum.py``'s "tracking" column
# is not it -- that column measures the achieved ratio and this does not.
PLANNING_BIAS = 1.09


def turning_ld(env, cfg, speed, altitude, mass, gravity, bank):
    """Path per metre of height in the cone's banked descent.

    This is what ``HAC_LD`` was standing in for, and ``config.py`` decomposed
    it years before anything computed it: "the achieved glide ratio
    discounted for tracking", achieved 1.86-1.90 and tracking 1.07-1.11.
    The second half is ``TRACKING`` above.  The first half is arithmetic:

        L cos(bank) = W cos(gamma),  D = W sin(gamma)
        => path / height = 1 / tan(gamma) = (L/D) * cos(bank)

    at the angle of attack that *holds* the turn -- ``alpha_for_load`` at
    ``1/cos(bank)`` -- rather than at best glide, because the cone is flown
    to a speed and not to an angle.  The two differ by a lot on an airframe
    whose best glide sits at a low speed: the vehicle flies fast, trims down,
    and gives away glide ratio in exact proportion.

    Checked against the number it replaces: on the vehicle ``HAC_LD`` was
    fitted to, this returns 1.85 where the fitted constant is 1.86.  That
    agreement is the reason to believe it, and it is also the warning -- a
    derivation that reproduces the fit tells you the fit was right *for that
    aircraft*, and says nothing yet about the next one.

    Returns ``None`` when the table cannot answer.  The caller falls back to
    ``cfg.HAC_LD``; there is no invented ratio here, because a cone planned
    on a made-up glide ratio is the failure that put this vehicle in the sea
    (``logs/LOG1315``, and ``logs/LOG2756`` from the other side).
    """
    from . import trajectory              # local: trajectory has no airframe
    cosb = math.cos(math.radians(min(abs(bank), 75.0)))
    load = 1.0 / max(0.2, cosb)
    alpha = trajectory.alpha_for_load(env, speed, altitude, mass, gravity,
                                      load)
    if alpha is None:
        return None
    try:
        cla, cda = env.coefficients(alpha, speed, altitude)
    except Exception:                                       # noqa: BLE001
        return None
    if cla <= 0.0 or cda <= 0.0:
        return None
    return (cla / cda) * cosb / PLANNING_BIAS


def turn_load(env, cfg, speed, altitude, mass, gravity):
    """The lateral load the wing can actually pay for, in g.

    **What the cone's radius model assumes, measured instead of assumed.**
    ``R = v^2 / (g tan(bank))`` is the level-turn formula, and it is level
    because it assumes the wing pulls ``1/cos(bank)`` -- 1.41 g at 45
    degrees, 2.00 at 60. Over 634 HAC ticks this airframe pulls a median
    **1.06** (p90 1.36, max 1.91), so the planned circle is about a third
    too tight at 45 degrees and nearly twice too tight at 60. The error
    grows with bank, which is precisely where a raised cap would put it.

    **But the measured 1.06 is the wrong number to substitute**, and that is
    the trap this project has a rule about. 1.06 is the load the vehicle *is*
    pulling in a descending spiral at a trimmed angle of attack; the radius
    question is what it *could* pull if it asked. Those are different
    quantities that happen to share a name, and swapping them would plan the
    cone on a vehicle flying lazily rather than on the airframe.

    So it is computed: the lift available at the angle of attack the cone is
    allowed to hold, over the weight. Every term comes from the swept table
    and the state, nothing is transcribed, and it answers ``None`` when the
    table cannot -- a missing load must not look like a generous one.
    """
    if mass <= 0.0 or gravity <= 0.0 or speed <= 0.0:
        return None
    try:
        rho = env.density(altitude)
        cla, _ = env.coefficients(cfg.HAC_ALPHA_MAX_DEG, speed, altitude)
    except Exception:                                       # noqa: BLE001
        return None
    if rho <= 0.0 or cla <= 0.0:
        return None
    return 0.5 * rho * speed * speed * cla / (mass * gravity)


def cone_ld(env, cfg, speed, altitude, mass, gravity):
    """``turning_ld`` at the cone's own bank limit, or ``HAC_LD``."""
    if not getattr(cfg, "AIRFRAME_DERIVED", False):
        return cfg.HAC_LD
    got = turning_ld(env, cfg, speed, altitude, mass, gravity,
                     cfg.HAC_BANK_MAX_DEG)
    return cfg.HAC_LD if got is None else got


def approach_ld(env, cfg, altitude, mass, gravity):
    """Ground per metre of height on final -- **not** best glide.

    ``APPROACH_BEST_LD`` is named after a quantity it is not.  Its own
    comment says what it is: ground from the cone's rollout to the first
    wheel contact over the height there, measured over 41 flights at **mean
    2.08, sd 0.27**.  Best glide on this airframe is 3.07, and the approach
    does not fly best glide -- it flies ``APPROACH_FACTOR`` times the stall,
    which is 2.25 and well up the back of the drag curve, where the ratio is
    2.33.  Two different numbers with one name is failure 19's shape, and
    routing this through ``glide_ld`` would have been that mistake made
    again by a different author.

    So it is derived at the speed the approach is actually flown at, wings
    level and at one g, and it lands on 2.33 against the 41 flights' 2.08 --
    inside one standard deviation of the thing it is meant to describe.

    **And the committed 4.2 is right, which this derivation is not.**
    Measured on the batch of 2026-09-21, ``landsum.py``'s rollout-to-wheels
    ratio -- the quantity ``APPROACH_BEST_LD`` is defined as -- reads
    **3.96 sd 0.41** over eight flights of the committed configuration and
    4.15 sd 0.82 over eight of the derived one.  The 4.2 is inside one
    standard deviation of what the aircraft does.

    So the "2.08 sd 0.27 over 41 flights" in ``Config.APPROACH_BEST_LD``'s
    comment is stale -- it describes a configuration that is no longer
    flown, which is failure 23's rule and the second time this session a
    number read out of a comment turned out to describe a different
    aeroplane (see ``TRACKING``).  **Re-run the tool, do not read the
    prose.**

    That leaves this function measuring the right idea and landing on the
    wrong number: 2.33 is the straight glide ratio at the approach speed,
    and the rollout-to-wheels span it has to describe also contains the
    flare, which is flat and buys 400-500 m of ground for 150 of height.
    The derivation is missing that term, which is most of the gap.  Off
    until it has one.
    """
    if not getattr(cfg, "APPROACH_LD_DERIVED", False):
        return cfg.APPROACH_BEST_LD
    vs = stall(env, cfg)
    if vs is None:
        return cfg.APPROACH_BEST_LD
    speed = cfg.APPROACH_FACTOR * vs
    got = turning_ld(env, cfg, speed, altitude, mass, gravity, 0.0)
    if got is None:
        return cfg.APPROACH_BEST_LD
    # ``turning_ld`` carries the cone's tracking overhead; a straight final
    # is not chasing a re-solved circle, so it does not pay it.
    return got * PLANNING_BIAS


def touchdown_aim(env, cfg):
    """Where the final's glide line meets the runway, metres past the
    threshold: ``TOUCHDOWN_AIM_M``, or under ``TOUCHDOWN_AIM_DERIVED`` the
    touchdown zone (``TOUCHDOWN_ZONE_FRACTION`` of the runway) less the
    flare's float.

    The float is the flare bleeding the door speed to the stall at the
    deceleration of best glide, ``(v_door^2 - v_stall^2) / (2 g / best_ld)``:
    the longest it can float, so the wheels land no later than the zone.
    2400 m is the far threshold -- an old-craft bias that made the shuttle
    cross the midpoint 840 m up and dive into the runway at 90 m/s
    (LOG4375).
    """
    if not getattr(cfg, "TOUCHDOWN_AIM_DERIVED", False):
        return cfg.TOUCHDOWN_AIM_M
    # The table's own numbers when it has been read, whatever
    # ``AIRFRAME_DERIVED`` says: this quantity has no fitted history to keep.
    v_stall = stall(env, cfg)
    if v_stall is None:
        return cfg.TOUCHDOWN_AIM_M
    v_door = cfg.APPROACH_FLARE_FACTOR * v_stall
    ratio = getattr(env, "best_ld", None) or glide_ld(env, cfg)
    decel = 9.81 / max(1.0, ratio)
    float_m = max(0.0, (v_door * v_door - v_stall * v_stall) / (2.0 * decel))
    zone = cfg.TOUCHDOWN_ZONE_FRACTION * cfg.RUNWAY_LENGTH_M
    return max(0.0, zone - float_m)


def alpha_ceiling(env, cfg):
    """The highest angle of attack worth commanding on this wing.

    ``ALPHA_MAX_DEG`` is 32 and its comment is "past 30 the lift curve turns
    over" -- an airframe property, measured once, on one airframe, and then
    written down.  The swept table answers it per aircraft: the wing in
    ``logs/LOG2747`` peaks at 30 degrees, and the one in ``logs/LOG2756``
    peaks at **25**, so the committed constant commands that second aircraft
    seven degrees onto the *back* of its own lift curve, where more angle
    buys less lift and a great deal more drag.

    **This is not the ceiling the vehicle already learns.**
    ``Holdable``/``alpha_ceiling`` watches for a command the vehicle cannot
    *achieve* -- a control-authority limit, which moves with dynamic
    pressure.  Stalling the wing is the opposite case: the vehicle achieves
    the angle perfectly well and simply makes less lift there, so nothing in
    the achieved-versus-commanded comparison can see it.  Two different
    limits; this one seeds and caps the other.

    Returns ``ALPHA_MAX_DEG`` when the table has not been swept, and never
    returns *more* than it: this is allowed to be a stricter bound on a
    wing that needs one, not a licence to command more than the
    configuration permits.
    """
    if not getattr(cfg, "AIRFRAME_DERIVED", False):
        return cfg.ALPHA_MAX_DEG
    measured = getattr(env, "stall_alpha", None)
    if measured is None or measured <= 0.0:
        return cfg.ALPHA_MAX_DEG
    return min(cfg.ALPHA_MAX_DEG, float(measured))
