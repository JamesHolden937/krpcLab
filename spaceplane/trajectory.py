"""Local propagator: gravity, drag, lift, and the rotating-frame terms.

Inherited wholesale from ``boosterland.trajectory``, including the two things
that file insists on and paid for:

* **RK4, and the step is load-bearing.**  Drag goes as ``v^2`` through a
  density that changes by a factor of e every 5 km, so a first-order step over
  a long descent *biases* the answer rather than adding noise to it -- and the
  whole of boosterland's systematic overshoot, and three sessions of aim-bias
  calibration fitted to it, turned out to be that bias (failure 16).  A glider
  has no engine with which to fix an arrival, so it can afford the error even
  less.
* **No remote calls inside the loop.**  The air comes from tables sampled once
  and the coefficients from a local grid, so a propagation is pure Python.

What is new is the control.  A booster's propagation takes a commanded angle of
attack in a fixed plane and applies a quadratic sideforce -- a cylinder in
crossflow, with lift as a small trim.  Here lift is the *primary* actuator,
there is a real lift curve that turns over at 30 degrees, and the vehicle can
roll it about the velocity.  So ``Steer`` carries an angle of attack **and a
bank angle**, and both coefficients are read from the measured table at the
angle actually being flown.  The drag is therefore a function of the control,
which is the thing that makes energy management possible at all: raising the
nose brakes.
"""
import math
from dataclasses import dataclass, field

from common import vec
from . import airframe


@dataclass
class Prediction:
    """Where the arc gets to the gate, and with how much left over.

    ``high`` is the quantity the glide is actually solved on, and choosing it
    took one wrong turn worth recording.  The obvious target is a *position*:
    propagate to the gate's altitude and measure how far from the gate the
    vehicle crosses it.  That is what boosterland does with the pad, and on a
    booster it is right, because the landing burn controls the arrival speed
    separately.

    A glider has no such second control, and the position target is
    consequently satisfiable in a way that is no use at all: the propagation
    happily flies through 1400 m a few hundred metres from the gate doing
    400 m/s, reports a miss of zero, and the guidance sees nothing to correct.
    Offline that produced a vehicle needing 73 m/s^2 of drag to arrive on
    energy, against an airframe that can make about 20, with the predicted
    miss reading +487 m the whole way down.

    So the arc is propagated to its closest approach to the gate and what is
    reported is the *height above the gate* there.  For an unpowered vehicle
    that single number is the energy error: arriving high over the gate is
    surplus and arriving low is a shortfall, and it is monotone in both
    controls -- more bank sinks it, more angle of attack holds it up.
    """
    position: tuple            # the closest approach to the gate
    velocity: tuple
    time_to_go: float
    speed: float
    altitude: float
    reached: bool              # True if it got to the gate's neighbourhood
    grounded: bool             # True if it hit the ground first
    steps: int
    high: float = 0.0          # altitude above the gate at closest approach
    long: float = 0.0          # along-track miss at the gate altitude
    cross: float = 0.0         # lateral offset from the centreline there
    closest: float = 0.0       # horizontal distance to the gate there
    profile: tuple = ()        # (speed, altitude) along the way down
    min_altitude: float = 0.0
    skipped: bool = False      # went into the air and came back out of it
    # **The arc flown inside the atmosphere**, from the first crossing of the
    # interface to wherever this arc ends.  It is the length of the *entry*,
    # as opposed to how far round the planet the vehicle happens to be from
    # the runway right now -- so it is a property of the trajectory and not
    # of how long the phase sat in orbit before committing to it.  The
    # deorbit's aim is a fraction of this; see ``deorbit_solution``.
    entry_arc: float = 0.0
    # **The two altitudes every entry propagation already passes through.**
    # ``entry_*`` is the atmosphere boundary (where ``entry_arc`` is
    # measured from) and ``interface_*`` is ``ENTRY_INTERFACE_M``, where the
    # glide takes the controls.  Recording them costs two comparisons per
    # step and saves a whole second propagation: the first version of the
    # interface instrument ran its own, and the stall that cost the control
    # loop moved the vehicle **9 km** at the interface and 1.5 km on the
    # ground.  A diagnostic must not change what it measures.
    entry_position: tuple = ()
    entry_velocity: tuple = ()
    interface_position: tuple = ()
    interface_velocity: tuple = ()
    interface_time: float = 0.0
    # True when this arc was chosen by ``guidance.max_range`` rather than by
    # nulling a miss -- i.e. the gate was out of reach and the vehicle is
    # flying for distance.  Carried on the prediction so the telemetry can
    # say which law flew the tick: a glide that quietly stops steering looks
    # exactly like one that is on profile, and for eighteen flights it did.
    max_range: bool = False


@dataclass
class Steer:
    """A commanded angle of attack and bank, held for one propagation.

    ``alpha`` is degrees off the relative wind, always positive: the lift
    curve is symmetric on this airframe (KSP's stock wings have no camber, and
    the probe reads the same magnitude upright and inverted), so there is
    nothing a negative angle buys that a 180 degree bank does not.

    ``bank`` is degrees of roll of the lift vector about the velocity, from
    "wings level, lift up".  Its *magnitude* is the energy and range control
    -- rolling the lift off the vertical lets the vehicle sink into thicker
    air, which is what actually brakes it -- and its *sign* is the only
    cross-track authority in the flight.  That split is not an implementation
    detail, it is the whole entry scheme: angle of attack is pinned near the
    top of the lift curve by the need to decelerate, so bank is what is left
    to steer with.
    """
    alpha: float = 0.0
    bank: float = 0.0
    cfg: object = None          # set to enable the arrival-speed cap
    mass: float = 0.0
    reversing: bool = True      # predict the mean of a reversing entry
    holdable: object = None     # the learned alpha ceiling, if any
    # **Where the hot entry stops making drag and starts making lift**, as an
    # airspeed.  ``ENTRY_MAX_DRAG`` flies the vehicle broadside while it is
    # faster than this and hands the angle of attack back to the range solve
    # below it, so this one number is the entry's whole energy budget -- and
    # it is *solved for*, not configured.  See ``guidance.drag_switch_for``.
    # ``None`` means *not solved*, which is not the same as zero: zero is
    # "stay broadside all the way down", a legitimate end of the bracket the
    # switch is solved over, and an entry flown that way has no lift and
    # cannot reach a runway.  A missing answer must not look like a good one,
    # so an unsolved switch turns the law off rather than flying its most
    # aggressive setting.
    drag_until: object = None

    def as_tuple(self):
        return (self.alpha, self.bank)


_LOAD_CACHE = {}
_LOAD_CACHE_KEY = None
# **``id()`` is not an identity, it is an address.**  CPython hands a freed
# object's id to the next one allocated, so a cache keyed on it can serve one
# environment's answers to another -- silently, and only when the first has
# been collected, which is why it shows up in a whole test run and never in
# the one test reproduced on its own.  Measured: the approach returned an
# angle of attack 2.2 degrees below the trim its own floor guarantees.  A
# serial is an identity: it is never reused.
_ENV_SERIAL = 0


def _env_identity(env):
    global _ENV_SERIAL
    serial = getattr(env, "_alpha_cache_serial", None)
    if serial is None:
        _ENV_SERIAL += 1
        serial = _ENV_SERIAL
        try:
            env._alpha_cache_serial = serial
        except Exception:                               # noqa: BLE001
            return id(env)                              # unsettable: fall back
    return serial


def alpha_for_load(env, speed, altitude, mass, gravity, load=1.0):
    """The angle of attack whose lift is ``load`` times the weight.

    Memoised on a coarse key, and that is not premature: this is called from
    inside ``acceleration``, so it runs four times per RK4 step and a couple
    of thousand steps per propagation.  Unmemoised it searched all fourteen
    angle bins every time -- eighteen table lookups per acceleration
    evaluation against the two the drag and lift need -- and it was most of
    the reason a single entry propagation took a third of a second.  The
    states an integration visits are close together, so the hit rate is high,
    and the key carries the table's generation so a re-probe drops the cache
    on its own.

    Searched on the *front* side of the lift curve only.  Past maximum lift
    the curve comes back down, so a given load is available at a high angle as
    well as a sensible one, and a naive search returns the high one -- which
    is a configuration falling out of the sky with 50 m^2 of drag.
    ``planeprobe.py`` printed an 81 m/s sink rate for exactly that mistake
    before it was fixed there too.
    """
    global _LOAD_CACHE_KEY
    generation = (getattr(env, "lift", None) is not None
                  and env.lift.generation)
    identity = (_env_identity(env), generation)
    if identity != _LOAD_CACHE_KEY:
        _LOAD_CACHE_KEY = identity
        _LOAD_CACHE.clear()
    key = (round(speed * 0.5), round(altitude * 0.005), round(mass * 0.01),
           round(load * 20.0))
    hit = _LOAD_CACHE.get(key)
    if hit is not None:
        return hit[0]

    rho = env.density(altitude)
    if rho <= 0.0 or speed <= 1.0 or mass <= 0.0:
        _LOAD_CACHE[key] = (None,)
        return None
    q = 0.5 * rho * speed * speed
    wanted = load * mass * gravity
    alphas = sorted(env._alphas)
    if not alphas:
        return None
    peak_alpha, peak_cl = alphas[0], -1.0
    for alpha in alphas:
        cla, _ = env.coefficients(alpha, speed, altitude)
        if cla > peak_cl:
            peak_alpha, peak_cl = alpha, cla
    if peak_cl * q < wanted:
        _LOAD_CACHE[key] = (peak_alpha,)
        return peak_alpha               # cannot make that much: ask for all
    previous = None
    for alpha in alphas:
        if alpha > peak_alpha:
            break
        cla, _ = env.coefficients(alpha, speed, altitude)
        lift = cla * q
        # **The fast end had no answer and returned the wrong one.**  The
        # crossing search needs two samples to bracket the load, so a vehicle
        # whose *lowest* tabulated angle already carries more than the load
        # asked for brackets nothing, falls out of the loop, and takes the
        # ``peak_alpha`` return at the bottom -- which is the angle of
        # **maximum lift**, the opposite end of the table from the answer.
        # Measured: at 140 m/s and 3 km this airframe holds 1 g at 0 degrees
        # with 40% to spare, and ``alpha_for_load`` returned 30.
        #
        # It is not a corner case.  ``alpha_limit_for_speed``,
        # ``guidance.approach`` and ``guidance.flare`` all take this as the
        # *trim*, and the approach builds its command as ``trim + gains``, so
        # a fast arrival -- exactly the arrival this vehicle keeps making --
        # was commanded near maximum lift and maximum drag on final.  A
        # saturated sentinel that a caller cannot tell from a measurement is
        # the failure this project has a rule about.
        if previous is None and lift >= wanted:
            _LOAD_CACHE[key] = (alpha,)
            return alpha
        if previous is not None:
            a0, l0 = previous
            if (l0 - wanted) * (lift - wanted) <= 0.0 and lift != l0:
                f = (wanted - l0) / (lift - l0)
                answer = a0 + f * (alpha - a0)
                _LOAD_CACHE[key] = (answer,)
                return answer
        previous = (alpha, lift)
    _LOAD_CACHE[key] = (peak_alpha,)
    return peak_alpha


def alpha_limit_for_speed(env, cfg, speed, altitude, mass, gravity):
    """Cap the angle of attack so the vehicle arrives able to *land*.

    This is the correction that a first offline propagation demanded, and the
    reason is worth keeping.  Holding the entry angle of attack all the way
    down is right while the vehicle is fast -- 30 degrees is maximum lift and
    ten times the drag of zero, which is the whole braking authority -- but
    held to the end it decelerates the vehicle to *that angle's* equilibrium
    glide speed, which the measured table puts at 37 m/s.  The first entry
    propagations arrived at the gate doing 37-44 m/s against a stall speed of
    37.1 and an approach speed of 67, i.e. they arrived unable to flare.

    So above a threshold the cap is wide open and the guidance may brake with
    everything it has; below it the cap becomes a speed hold on the approach
    speed, and the angle of attack is what holds it.  Bank is then the
    ranging control, which is the shuttle's division of labour and it is
    forced by exactly this: angle of attack is spoken for.

    **The propagator applies this too, not just the control loop**, which is
    the point of putting it here.  boosterland learned that one the expensive
    way: a prediction that does not fly the law the vehicle flies is a
    prediction of a trajectory nobody flies, and the fix there was to make
    the throttle law itself shared between the propagator and the loop.
    """
    if cfg is None:
        return cfg_default_max(cfg)
    # The *glide's* arrival speed, not the approach's touchdown speed: see
    # ``Config.GLIDE_ARRIVAL_FACTOR``.
    approach = cfg.GLIDE_ARRIVAL_FACTOR * airframe.stall(env, cfg)
    if speed >= cfg.SPEED_HOLD_FACTOR * approach:
        return cfg.ALPHA_MAX_DEG
    trim = alpha_for_load(env, speed, altitude, mass, gravity, 1.0)
    if trim is None:
        return cfg.ALPHA_MAX_DEG
    wanted = trim + cfg.SPEED_HOLD_KP * (speed - approach)
    return vec.clamp(wanted, cfg.ALPHA_MIN_DEG, cfg.ALPHA_MAX_DEG)


def alpha_floor_for_speed(env, cfg, speed, altitude, mass, gravity):
    """The *lowest* angle of attack that is honest about the speed.

    The other half of ``alpha_limit_for_speed``: the same law, used as a
    floor.  See ``Config.SPEED_FLOOR_ON`` for the measurement that asks for
    it -- a glide solved on the miss at the gate's altitude buys range by
    diving, arrives over the gate at 82 degrees nose down, and cannot land.

    Returns ``ALPHA_MIN_DEG`` -- i.e. no constraint -- when the feature is
    off, when the vehicle is already at or below the approach speed (there is
    nothing to bleed and best glide is the right answer), or supersonically,
    where ``SOLVE_ALPHA_MIN_DEG`` guards an interior optimum this would fight.

    **Shared by the propagator and the control loop**, like the cap above it
    and like boosterland's ``landing_command``: a prediction of a law nobody
    flies is a prediction of a trajectory nobody flies.
    """
    if cfg is None or not getattr(cfg, "SPEED_FLOOR_ON", False):
        return cfg_default_min(cfg)
    # Only while the glide has surplus to spend: see
    # ``Config.SPEED_FLOOR_WHEN_LONG``.  ``env.spending`` is set by the
    # control loop from its own prediction each tick, so the propagations of
    # that tick all see the same law -- which is the point of hanging it on
    # the environment.
    if (getattr(cfg, "SPEED_FLOOR_WHEN_LONG", False)
            and getattr(env, "spending", True) is False):
        return cfg.ALPHA_MIN_DEG
    approach = cfg.GLIDE_ARRIVAL_FACTOR * airframe.stall(env, cfg)
    if speed <= approach:
        return cfg.ALPHA_MIN_DEG
    try:
        mach = env.mach(speed, altitude)
    except Exception:                                   # noqa: BLE001
        return cfg.ALPHA_MIN_DEG
    if mach >= cfg.SPEED_FLOOR_MACH:
        return cfg.ALPHA_MIN_DEG
    trim = alpha_for_load(env, speed, altitude, mass, gravity, 1.0)
    if trim is None:
        # **Not a sentinel.**  ``alpha_for_load`` returns None when the wing
        # cannot make the load at all, and inventing a floor from that is
        # inventing a measurement.  No constraint is the honest answer.
        return cfg.ALPHA_MIN_DEG
    # Floored on the subsonic solve floor as well as on the trim, so the
    # constraint tightens with speed instead of relaxing.  The 1-g trim
    # *falls* as the vehicle goes faster -- that is what trim means -- so a
    # floor built on trim alone is loosest exactly where the dive is: at
    # 137 m/s it came out below the floor it gives at 87.
    wanted = (max(trim, cfg.SOLVE_ALPHA_MIN_SUB_DEG)
              + cfg.SPEED_FLOOR_KP * (speed - approach)
              - cfg.SPEED_FLOOR_SLACK_DEG)
    # **Under the cap, always.**  These are two halves of one speed hold, and
    # a floor above the ceiling leaves the angle of attack no legal value at
    # all -- whichever of the two is applied second silently wins.  It is not
    # hypothetical: with the approach speed at 115 m/s the ``max(trim, 8)``
    # term put the floor at 9.7 degrees where the hold wanted 7.4, because
    # the 8 is a *subsonic solve* floor and has nothing to say about a
    # vehicle that is simply fast.
    return vec.clamp(wanted, cfg.ALPHA_MIN_DEG,
                     alpha_limit_for_speed(env, cfg, speed, altitude,
                                           mass, gravity))


def cfg_default_min(cfg):
    return 0.0 if cfg is None else cfg.ALPHA_MIN_DEG


def cfg_default_max(cfg):
    return 90.0 if cfg is None else cfg.ALPHA_MAX_DEG


def gravity_at(env, r):
    d = vec.norm(r)
    return vec.scale(r, -env.mu / (d * d * d))


def lift_frame(r, v):
    """``(up_perp, side)``: the plane the lift vector is rolled in.

    ``up_perp`` is the local vertical with the along-track part removed, which
    is where the lift points with the wings level.  ``side`` completes the
    pair.  Which of left and right ``+side`` is depends on kRPC's handedness
    and is deliberately *not* reasoned about: the same function serves the
    propagator and the control command, so whichever way round it is, the two
    agree -- and the solve measures the sensitivity numerically, so the sign
    cancels there too.  This is the same trade ``Environment._measure_omega``
    makes.
    """
    speed = vec.norm(v)
    if speed < 1e-6:
        return None, None
    vhat = vec.scale(v, 1.0 / speed)
    up = vec.unit(r)
    up_perp = vec.project_out(up, vhat)
    if vec.norm(up_perp) < 1e-6:
        # Straight up or straight down: any perpendicular will do, and the
        # bank angle stops meaning anything anyway.
        for trial in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            up_perp = vec.project_out(trial, vhat)
            if vec.norm(up_perp) > 1e-6:
                break
    up_perp = vec.unit(up_perp)
    side = vec.unit(vec.cross(vhat, up_perp))
    return up_perp, side


def lift_direction(r, v, bank_deg):
    up_perp, side = lift_frame(r, v)
    if up_perp is None:
        return None
    bank = math.radians(bank_deg)
    return vec.unit(vec.add(vec.scale(up_perp, math.cos(bank)),
                            vec.scale(side, math.sin(bank))))


class Holdable:
    """The angle of attack this airframe can hold, learned while flying it.

    The plant is a **saturation**: ``achievable = min(command, holdable(q))``.
    That shape is not a guess -- it is what the telemetry says, and it is why
    ``ALPHA_TRACKING``'s proportional gain failed (the solve cancels a gain by
    asking for more; it cannot talk a ceiling round).  What no measurement in
    this project could supply was ``holdable`` itself, because
    ``simulate_aerodynamic_force_at`` returns a force and kRPC will not report
    a pitching moment at any price.

    So it is *observed* rather than tabulated.  Every tick the control loop
    already knows what it asked for, what the vehicle achieved, and the
    dynamic pressure it happened at.  When the vehicle is short of its command
    by more than ``HOLDABLE_SATURATED_DEG``, that tick is evidence about the
    ceiling and the achieved angle is a sample of it; when it is tracking, the
    tick says only that the ceiling is at least the command, which raises a
    bin that an earlier transient pushed too low.

    **Nothing in here is a number about this aircraft.**  A fitted table would
    fly the next vehicle on this one's trim, which is the same reason the aero
    table is swept in STANDBY rather than written down and the vessel's axes
    are measured rather than assumed.

    Two properties it has to have, both of them this project's rules:

    * **A bin with no evidence returns ``None``**, not a large angle that a
      ``min`` would quietly accept.  A missing answer must not look like a
      good one.
    * **It extrapolates downward, never upward.**  The propagator asks about
      dynamic pressures the vehicle has not reached yet -- that is the whole
      point of a prediction -- and the ceiling falls as the air thickens.  So
      a query above everything seen is answered with the lowest ceiling
      observed so far, which is conservative in the direction that matters:
      predicting less lift than the vehicle turns out to have makes the
      deorbit burn larger, and surplus energy is the recoverable side.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.bins = {}                  # bin index -> [ceiling, samples]
        self.generation = 0
        # The densest air the vehicle has been seen to saturate in, as
        # ``(alpha achieved, q, mach)``.  One such sample is enough to
        # predict the ceiling everywhere denser; see ``prior``.
        self.anchor = None

    def _bin(self, q):
        per = max(1, int(self.cfg.HOLDABLE_Q_DECADE_BINS))
        return int(math.floor(per * math.log10(max(1.0, q))))

    def observe(self, commanded, achieved, q, mach=None):
        """One tick of evidence.  Cheap enough to call every tick, and is."""
        if q < float(self.cfg.HOLDABLE_MIN_Q) or commanded <= 0.0:
            return
        if achieved > commanded + float(self.cfg.HOLDABLE_SATURATED_DEG):
            # Trimming nose-*high* of its command.  That says nothing about a
            # ceiling -- it is the transonic overshoot the ratchet's own
            # comment records -- and taking it as one would raise the limit
            # above anything the vehicle holds steadily.
            return
        index = self._bin(q)
        short = commanded - achieved
        current = self.bins.get(index)
        if short > float(self.cfg.HOLDABLE_SATURATED_DEG):
            # Saturated: the vehicle is telling us where its ceiling is.
            value = achieved
            if current is None:
                self.bins[index] = [value, 1]
            else:
                # The *highest* angle seen while saturated, because a single
                # low sample is a transient -- a bank reversal throws the
                # tracking error for a second or two -- and a ratchet that
                # believes one is the failure ``ALPHA_RECOVER_DEG_S`` exists
                # to undo.
                current[0] = max(current[0], value)
                current[1] += 1
            # **One saturated sample anchors the whole curve.**  Keep the one
            # in the densest air: predicting *denser* air than the anchor is
            # an extrapolation and the shorter it is, the better.
            if mach is not None and (self.anchor is None or q > self.anchor[1]):
                self.anchor = (achieved, q, mach)
        else:
            # Tracking: the ceiling is at least what was asked for.
            if current is None:
                return
            current[0] = max(current[0], commanded)
            current[1] += 1
        self.generation += 1

    def limit(self, q):
        """The ceiling at this ``q``, or ``None`` where there is no evidence."""
        if q < float(self.cfg.HOLDABLE_MIN_Q):
            return None
        trusted = [(i, v[0]) for i, v in self.bins.items()
                   if v[1] >= int(self.cfg.HOLDABLE_MIN_SAMPLES)]
        if not trusted:
            # Nothing measured yet this flight -- which is most of an entry,
            # and all of the part where a shortfall can still be fixed.
            return self.probe(q)
        index = self._bin(q)
        exact = dict(trusted)
        if index in exact:
            best = exact[index]
        else:
            below = [v for i, v in trusted if i < index]
            above = [v for i, v in trusted if i > index]
            if below and above:
                best = min(min(below), min(above))
            elif below:
                # Denser air than anything flown yet: the ceiling only falls,
                # so carry the lowest one seen rather than inventing a higher.
                best = min(below)
                fitted = self._extrapolate(index, trusted)
                if fitted is not None:
                    best = min(best, fitted)
                # **This is the branch that matters.**  Denser air than the
                # vehicle has flown is exactly where the propagator has been
                # optimistic, and ``min(below)`` is a thin-air bin -- which
                # over 98 flights carries 4.5 to 5.7 degrees of scatter
                # because it records the command rather than a ceiling.  The
                # probe measured this ``q`` directly.
                measured = self.probe(q)
                if measured is not None:
                    best = min(best, measured)
            else:
                # Thinner air than anything flown: nothing has refused a
                # command up here, so do not invent a limit.
                return self.probe(q)
        return best + float(self.cfg.HOLDABLE_MARGIN_DEG)

    def probe(self, q):
        """The ceiling this airframe was *measured* to hold at this ``q``.

        ``limit`` above answers from the flight in hand, and for most of an
        entry it has nothing to answer with: a bin is only evidence where the
        vehicle was commanded past its ceiling, which does not reliably
        happen until below about 28 km.  Until then it returned ``None`` --
        *no limit* -- and the propagator flew a vehicle that holds whatever
        it is asked in air it has never met.  That optimism is the whole of
        the entry's range deficit (spaceplane failures 47, 50 and 51), and it
        is optimism in the one direction nothing downstream can recover from.

        So fall back on a curve measured once by a flight whose job was to be
        refused -- ``Config.HOLDABLE_PROBE``, and see that entry for how it
        was taken and what would disagree with it.  Evidence from the flight
        in hand always wins where it exists: the probe describes the
        airframe, the flight describes today.

        Interpolated in log ``q``, because that is the axis the ceiling is
        straight in and the axis the bins are cut on.  Off the ends it holds
        the end value rather than extrapolating a curve that is only
        trustworthy between measured points.
        """
        if not getattr(self.cfg, "HOLDABLE_PROBE_ON", False):
            return None
        table = getattr(self.cfg, "HOLDABLE_PROBE", ())
        if not table or q <= 0.0:
            return None
        points = sorted(table)
        if q <= points[0][0]:
            return points[0][1]
        if q >= points[-1][0]:
            return points[-1][1]
        for (q0, a0), (q1, a1) in zip(points, points[1:]):
            if q0 <= q <= q1 and q1 > q0:
                share = (math.log(q) - math.log(q0)) / (math.log(q1)
                                                        - math.log(q0))
                return a0 + share * (a1 - a0)
        return None

    def prior(self, env, q, mach):
        """The ceiling in air the vehicle has not reached, from one sample.

        **The estimator this class had could not be pessimistic in time.**
        ``limit`` answers ``None`` -- *no limit* -- for any dynamic pressure
        with no trusted bin, so the propagator assumes this airframe holds
        whatever it is commanded in air it has never met, and the default is
        optimism in the one direction that cannot be recovered from.
        ``_extrapolate`` was written to fix that and measured inert: it needs
        ``HOLDABLE_MIN_SAMPLES`` samples in each of ``HOLDABLE_FIT_MIN_BINS``
        bins -- eight ticks of saturation -- before it has a trend, and the
        vehicle does not reliably produce one until below 32 km, by which
        time the range deficit is already being made.  Spaceplane failures
        47 and 50.

        **This needs one sample, and the unknowns cancel.**  The angle of
        attack is lost when the aerodynamic pitching moment overcomes the
        control torque holding it:

            q * Cn(alpha, M) * arm  =  T

        The moment arm and the available torque are both properties of the
        vehicle that kRPC will not report usefully -- it gives no pitching
        moment at any price, and ``available_rcs_torque`` is a peak figure a
        steady aerodynamic moment never sees.  But neither has to be known.
        Take one observed saturation ``(alpha_s, q_s, M_s)``: the same
        product holds there, so for any other dynamic pressure

            Cn(alpha, M)  =  Cn(alpha_s, M_s) * q_s / q

        and the ceiling is whatever angle satisfies it.  ``arm`` and ``T``
        divide out, leaving only the vehicle's own swept table -- so there is
        no constant about this airframe anywhere in it, and a different
        vehicle gets a different answer from its own sweep for free.

        Only ever used to make a ceiling *lower*, and only for air denser
        than the anchor: predicting thinner air would be extrapolating the
        wrong way down a curve whose shape is only trustworthy between
        measured points.
        """
        if not getattr(self.cfg, "HOLDABLE_PRIOR", False):
            return None
        if self.anchor is None or env is None or mach is None:
            return None
        alpha_s, q_s, mach_s = self.anchor
        if q <= q_s:
            return None
        try:
            target = normal_coefficient(env, alpha_s, mach_s) * q_s / q
        except Exception:                                   # noqa: BLE001
            return None
        low = float(self.cfg.ALPHA_CEILING_FLOOR_DEG)
        high = float(self.cfg.ALPHA_MAX_DEG)
        try:
            if normal_coefficient(env, low, mach) >= target:
                return low
            if normal_coefficient(env, high, mach) <= target:
                return None         # even the stop is holdable; say nothing
            for _ in range(24):
                mid = 0.5 * (low + high)
                if normal_coefficient(env, mid, mach) < target:
                    low = mid
                else:
                    high = mid
        except Exception:                                   # noqa: BLE001
            return None
        return 0.5 * (low + high)

    def _extrapolate(self, index, trusted):
        """Carry the *trend* into air the vehicle has not reached, not the
        last value.

        Holding the lowest ceiling seen is conservative against a flat plant
        and wildly optimistic against this one.  Measured on this airframe
        (``logs/LOG1015``): 2154 Pa: 31.9 deg, 3162: 27.0, 4642: 24.0,
        6813: 17.9, 10000: 14.9 -- about **26 degrees per decade of dynamic
        pressure**, and monotone.  Early in an entry nothing has saturated
        yet, so ``min(below)`` is whatever the thin air allowed, and the
        propagator predicts the vehicle holding it at ten times the ``q``.

        That is not a small error and it has a direction: the prediction is
        optimistic about range for the whole part of the entry where the
        solve still has the authority to fix a shortfall, and becomes honest
        only below 25 km where it has none.  Over 34 flights the predicted
        miss sits near zero down to 27 km and then falls off a cliff; a
        flight still positive at 27 km lands a median 1.7 km out and one
        already negative lands 21.5 km out.

        Fitted by least squares on the bins already trusted, and only ever
        used to make the ceiling *lower* -- a fit that slopes upward is
        discarded rather than believed, because "it gets easier in thicker
        air" is not a thing this plant does and a two-bin fit is noisy.

        Still learned in flight and still this vehicle: what is extrapolated
        is the trend in its own samples, not a table from a previous one.
        """
        if not getattr(self.cfg, "HOLDABLE_EXTRAPOLATE", False):
            return None
        if len(trusted) < max(2, int(self.cfg.HOLDABLE_FIT_MIN_BINS)):
            return None
        xs = [float(i) for i, _ in trusted]
        ys = [float(v) for _, v in trusted]
        n = float(len(xs))
        mx = sum(xs) / n
        my = sum(ys) / n
        den = sum((x - mx) ** 2 for x in xs)
        if den <= 0.0:
            return None
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
        if slope >= 0.0:
            return None
        value = my + slope * (float(index) - mx)
        return max(float(self.cfg.ALPHA_CEILING_FLOOR_DEG), value)

    def summary(self):
        """One short string for the log, so a flight says what it learned."""
        trusted = sorted((i, v) for i, v in self.bins.items()
                         if v[1] >= int(self.cfg.HOLDABLE_MIN_SAMPLES))
        per = max(1, int(self.cfg.HOLDABLE_Q_DECADE_BINS))
        return " ".join("%.0fPa:%.1f/%d" % (10.0 ** (i / float(per)), v[0], v[1])
                        for i, v in trusted)


def normal_coefficient(env, alpha_deg, mach):
    """``Cn*A`` at an angle and Mach, from the vehicle's own swept table.

    The force perpendicular to the body is what makes the pitching moment
    that takes the angle of attack away from the vehicle, so it is the
    quantity the alpha ceiling is really about -- not lift, which is
    perpendicular to the *airflow* and therefore rotates as the angle
    changes.
    """
    cla = env.lift.lookup(alpha_deg, mach)
    cda = env.drag.lookup(alpha_deg, mach)
    a = math.radians(alpha_deg)
    return cla * math.cos(a) + cda * math.sin(a)


def max_drag_alpha(cfg, q, mach, holdable=None, speed=None, until=None):
    """The hot entry's angle of attack: as much drag as the vehicle can hold.

    ``None`` when the law is off or the entry is no longer hot, which leaves
    the angle of attack to whoever owns it otherwise -- the range solve in
    GLIDE, ``ENTRY_ALPHA_DEG`` in COAST.

    **Why this is a different quantity from every alpha constant above it.**
    ``ENTRY_ALPHA_DEG`` and ``ALPHA_MAX_DEG`` are both set by lift arguments
    (a range plateau, and the lift curve turning over at 35).  Drag does not
    turn over: on this airframe's swept table ``CdA`` climbs monotonically to
    its peak at 90 degrees, where it is five times the value at 22 and the
    lift is exactly zero.  A phase whose job is to destroy energy should be
    bounded by the drag stop, and until now nothing has ever asked for it.

    **The guard is the plant, not a constant.**  The command returned here is
    the ceiling ``Holdable`` has measured -- live evidence first, the
    ``HOLDABLE_PROBE`` curve as the prior -- so the vehicle is asked for
    exactly what it has been seen to hold and never for more.  That matters
    twice over: an angle it cannot hold is not flown (so the prediction and
    the flight agree), and there is no standing shortfall for the attitude
    controller to spend monopropellant on, which is how the entire tank once
    went for three kilometres of altitude (LOG1616).

    Above ``HOLDABLE_MIN_Q`` there is no aerodynamic moment worth the name --
    at 100 Pa it is a twentieth of the 1 kPa moment the vehicle was measured
    to hold 48 degrees against -- so up there the stop is the only limit.
    With no evidence and no prior, this returns ``ENTRY_ALPHA_DEG``: today's
    entry, which is the conservative answer rather than a large angle a
    ``min`` would quietly accept.
    """
    if not getattr(cfg, "ENTRY_MAX_DRAG", False):
        return None
    # **The handover is a solved speed, not a constant.**  The first version
    # of this law switched at a fixed Mach 3, which is the shape this project
    # has a rule against: a constant standing in for a quantity the program
    # can compute.  What the switch actually has to satisfy is "stop spending
    # energy when you can no longer afford to", and that is a statement about
    # the range still to run -- so it is solved, against the propagator, at
    # the same moment the burn is (``guidance.drag_switch_for``).
    #
    # It also has to be known *before* the burn, because the entry's arc
    # depends on it and the arc is what decides where the burn goes.  Solving
    # it there is what makes the deorbit's answer self-consistent.
    #
    # A closed form will not do: the equilibrium-glide range equation gives
    # 647 km where this vehicle flies 2317, because most of the arc is flown
    # near circular speed with centrifugal lift carrying it and the vehicle
    # nowhere near equilibrium.
    if until is None:
        # Nobody solved it.  That happens when the flight is picked up past
        # the burn (``ENGAGE_INTO_LANDING``, an entry save) and it must not
        # read as "broadside for the whole entry".
        return None
    if speed is not None and speed <= float(until):
        return None
    if mach is not None and mach < float(getattr(cfg, "ENTRY_MAX_DRAG_MACH",
                                                 0.0)):
        return None
    ceiling = float(getattr(cfg, "ENTRY_ALPHA_CEILING_DEG", 90.0))
    if q < float(cfg.HOLDABLE_MIN_Q):
        # Nothing up here can refuse it.
        return ceiling
    limit = None
    if holdable is not None:
        limit = holdable.limit(q)
        if limit is None:
            limit = holdable.probe(q)
    if limit is None:
        return float(cfg.ENTRY_ALPHA_DEG)
    return max(float(cfg.ENTRY_ALPHA_DEG), min(ceiling, limit))


def holdable_alpha(cfg, alpha, q, holdable=None, env=None, mach=None):
    """Clamp a commanded angle to what the airframe has shown it can hold.

    ``holdable`` is a ``Holdable`` carried on the ``Steer``, so the propagator
    and the control loop consult the same learned plant -- the project's
    "predict the law you fly", in the one place where the plant limit is
    knowable only by watching.  With no estimator, or none with evidence at
    this ``q``, the command is returned untouched.
    """
    if not getattr(cfg, "HOLDABLE_ON", False) or holdable is None:
        return alpha
    limit = holdable.limit(q)
    if env is not None and mach is not None:
        # **The prior is used only to make a ceiling lower, never higher.**
        # Measured evidence at this dynamic pressure outranks a prediction
        # about it; the prior exists for the air the vehicle has not reached.
        predicted = holdable.prior(env, q, mach)
        if predicted is not None:
            limit = predicted if limit is None else min(limit, predicted)
    return alpha if limit is None else min(alpha, limit)


def tracked_alpha(cfg, alpha, q):
    """The angle of attack the vehicle will hold, given what it was asked.

    Measured, not assumed: see ``Config.ALPHA_TRACKING``.  In thin air this
    airframe holds its command; in dense air it drifts toward its own trim
    point and delivers about 85% of it, and transonically it can overshoot
    the other way.  A propagation that believes the command is a propagation
    of a trajectory the vehicle does not fly, which is the error this project
    has now made in five places.
    """
    if not getattr(cfg, "ALPHA_TRACKING_ON", False) \
            and not getattr(cfg, "_tracking_forced", False):
        return alpha
    table = getattr(cfg, "ALPHA_TRACKING", ())
    if not table or q <= 0.0:
        return alpha
    first_q, first_f = table[0]
    if q <= first_q:
        return alpha * first_f
    previous = table[0]
    for point in table[1:]:
        if q <= point[0]:
            q0, f0 = previous
            q1, f1 = point
            span = q1 - q0
            f = f0 if span <= 0.0 else f0 + (f1 - f0) * (q - q0) / span
            return alpha * f
        previous = point
    return alpha * table[-1][1]


def acceleration(env, r, v, mass, steer):
    """Total acceleration in the rotating body frame."""
    a = gravity_at(env, r)
    speed = vec.norm(v)
    if mass > 0.0 and speed > 1e-6:
        altitude = vec.norm(r) - env.equatorial_radius
        rho = env.density(altitude)
        if rho > 0.0:
            alpha = steer.alpha if steer is not None else 0.0
            bank = steer.bank if steer is not None else 0.0
            q0 = 0.5 * rho * speed * speed
            if steer is not None and steer.cfg is not None:
                # **The hot entry's own law, and the propagator flies it
                # because the vehicle does.**  A prediction of a law nobody
                # flies is a prediction of a trajectory nobody flies -- and
                # this one changes the drag by a factor of five, so a deorbit
                # solved without it is solved for a different aircraft.  That
                # is the point: the search sees the extra drag and asks for a
                # smaller burn.
                hot = max_drag_alpha(
                    steer.cfg, q0,
                    speed / env.speed_of_sound(altitude),
                    getattr(env, "holdable", None)
                    or getattr(steer, "holdable", None),
                    speed,
                    # Off the ``Steer`` first here, not the environment: the
                    # deorbit search propagates *candidate* switch speeds and
                    # each candidate has to fly its own.
                    steer.drag_until if steer.drag_until is not None
                    else getattr(env, "drag_until", None))
                if hot is not None:
                    alpha = hot
                gravity = env.mu / (vec.norm(r) ** 2)
                alpha = min(alpha, alpha_limit_for_speed(
                    env, steer.cfg, speed, altitude,
                    steer.mass or mass, gravity))
                alpha = max(alpha, alpha_floor_for_speed(
                    env, steer.cfg, speed, altitude,
                    steer.mass or mass, gravity))
            q = q0
            if steer is not None and steer.cfg is not None:
                # Off the *environment*, not the ``Steer``: every
                # propagation already carries ``env``, and hanging the
                # learned plant there means the sixteen ``Steer`` objects
                # the deorbit search builds cannot each forget to pass it.
                # A limit nobody passed on is a limit nobody flies.
                alpha = holdable_alpha(steer.cfg, alpha, q,
                                       getattr(env, "holdable", None)
                                       or getattr(steer, "holdable", None),
                                       env, speed / env.speed_of_sound(altitude))
                alpha = tracked_alpha(steer.cfg, alpha, q)
            cla, cda = env.coefficients(alpha, speed, altitude)
            vhat = vec.scale(v, 1.0 / speed)
            if cda > 0.0:
                a = vec.add(a, vec.scale(vhat, -cda * q / mass))
            if cla > 0.0:
                if steer is not None and steer.reversing:
                    # **The mean of a reversing entry, not one constant lean.**
                    # The vehicle flies bank reversals: it holds a magnitude
                    # and swaps the sign whenever the azimuth to the runway
                    # drifts outside a deadband.  Over the entry the vertical
                    # component of lift is steadily reduced by cos(bank) --
                    # which is the whole energy control -- while the lateral
                    # component averages to nothing.
                    #
                    # Propagating one constant lean instead predicts a vast
                    # curving arc: offline it reported cross-track misses of
                    # 80-98 km that alternated sign with every guidance cycle,
                    # and the sign logic thrashed because neither answer was
                    # ever good.  A predictor that cannot represent the
                    # manoeuvre the vehicle flies cannot be used to choose it
                    # -- boosterland's "the predictor flies the landing burn",
                    # in the one place where flying it literally is wrong.
                    # **And the magnitude is a duty cycle, not a lean.**
                    # The averaging above is over the *azimuth* -- the sign
                    # swaps and the lateral component cancels -- and the
                    # magnitude is still one constant.  It is not: a reversal
                    # is about seventeen seconds of slew at
                    # ``BANK_RATE_DEG_S``, and a glide that reverses eleven
                    # times spends 138 s of its 700 near wings level against
                    # 88 s for one that reverses seven times.  Near wings
                    # level the vehicle sinks less and flies further, so the
                    # reversal *count* is a range term -- and it is the one
                    # the arrival is bimodal in (failure 78).
                    #
                    # Both single-bank repairs have been flown and both are
                    # worse: the instantaneous command spikes ``long`` by
                    # +20 km mid-reversal, and ``BANK_PREDICT_INTENT``'s
                    # stop-lean lands it long because the vehicle really is
                    # near level for part of the time.  What is wanted is the
                    # mean of ``cos(bank)`` *including its duty cycle*, which
                    # is neither -- see the comment at ``BANK_RATE_DEG_S``,
                    # which says exactly this and stops there.
                    #
                    # ``env.bank_cos_duty`` is that mean, measured by the
                    # vehicle on itself (``Autoland.update_bank_duty``) as the
                    # ratio of the time-averaged ``cos`` of what it commanded
                    # to the time-averaged ``cos`` of the lean it meant to
                    # hold.  A flight that sits on its stops reports 1.0 and
                    # this is inert; a flight that reverses constantly reports
                    # the extra lift it is really getting.  One number, no new
                    # fitted constant, and it cannot spike -- the mean does
                    # not care where in a slew the tick happened to land.
                    up_perp, _ = lift_frame(r, v)
                    if up_perp is not None:
                        share = math.cos(math.radians(bank))
                        duty = getattr(env, "bank_cos_duty", None)
                        if (duty and steer.cfg is not None
                                and getattr(steer.cfg, "GLIDE_BANK_DUTY_ON",
                                            False)):
                            share = min(1.0, share * duty)
                        vertical = cla * q * share
                        a = vec.add(a, vec.scale(up_perp, vertical / mass))
                else:
                    side = lift_direction(r, v, bank)
                    if side is not None:
                        a = vec.add(a, vec.scale(side, cla * q / mass))
    w = env.omega
    a = vec.add(a, vec.scale(vec.cross(w, v), -2.0))            # Coriolis
    a = vec.add(a, vec.scale(vec.cross(w, vec.cross(w, r)), -1.0))  # centrifugal
    return a


def _rk4(env, r, v, mass, dt, steer):
    def deriv(rr, vv):
        return vv, acceleration(env, rr, vv, mass, steer)

    k1r, k1v = deriv(r, v)
    k2r, k2v = deriv(vec.add(r, vec.scale(k1r, dt / 2.0)),
                     vec.add(v, vec.scale(k1v, dt / 2.0)))
    k3r, k3v = deriv(vec.add(r, vec.scale(k2r, dt / 2.0)),
                     vec.add(v, vec.scale(k2v, dt / 2.0)))
    k4r, k4v = deriv(vec.add(r, vec.scale(k3r, dt)),
                     vec.add(v, vec.scale(k3v, dt)))

    def combine(a, b, c, d):
        return vec.scale(vec.add(vec.add(a, vec.scale(b, 2.0)),
                                 vec.add(vec.scale(c, 2.0), d)), dt / 6.0)

    return (vec.add(r, combine(k1r, k2r, k3r, k4r)),
            vec.add(v, combine(k1v, k2v, k3v, k4v)))


def _euler(env, r, v, mass, dt, steer):
    a = acceleration(env, r, v, mass, steer)
    v2 = vec.add(v, vec.scale(a, dt))
    return vec.add(r, vec.scale(v2, dt)), v2


def _step(env, r, v, mass, dt, cfg, steer):
    if cfg.PREDICT_RK4:
        return _rk4(env, r, v, mass, dt, steer)
    return _euler(env, r, v, mass, dt, steer)



def predict(env, r0, v0, mass, cfg, steer=None, gate=None, end=None,
            target_radius=None):
    """Propagate to the gate's altitude, or to the ground, and say where.

    The stop is the *gate's altitude* and the horizontal miss there is what
    the glide is solved on.  Stopping at the closest approach to the gate was
    tried instead, on the reasoning that the height left over there is the
    energy error: it converges that quantity nicely and lands the vehicle
    twenty kilometres away, because nulling the energy says nothing about
    *where* it is nulled.  The arrival speed, which was the worry that
    prompted it, is handled where it belongs -- by the angle-of-attack cap in
    ``alpha_limit_for_speed``, which holds 65 m/s at the gate across the whole
    usable range of deorbit burns.

    Returns a :class:`Prediction`.  ``reached`` says the arc got down to the
    gate's altitude; ``grounded`` says it ran out of sky first, which is a
    shortfall and not a miss, and the guidance needs to be able to tell them
    apart.
    """
    if steer is None:
        steer = Steer()
    r = tuple(r0)
    v = tuple(v0)
    t = 0.0
    steps = 0
    profile = []
    lowest = vec.norm(r) - env.equatorial_radius
    entered = False
    skipped = False
    ground = env.target_radius
    stop = (target_radius if target_radius is not None
            else env.target_radius + cfg.GATE_ALT_M)
    closest = None

    def answer(rr, vv, tt, ss, reached, grounded):
        radius = vec.norm(rr)
        long = cross = 0.0
        if gate is not None and end is not None:
            long, cross = miss_components(env, end, rr, gate)
        return Prediction(
            position=tuple(rr), velocity=tuple(vv), time_to_go=tt,
            speed=vec.norm(vv), altitude=radius - env.equatorial_radius,
            reached=reached, grounded=grounded, steps=ss,
            high=radius - (env.target_radius + cfg.GATE_ALT_M),
            long=long, cross=cross,
            closest=(closest if closest is not None else 0.0),
            profile=tuple(profile), min_altitude=lowest, skipped=skipped,
            entry_arc=(0.0 if entry_r is None
                       else forward_arc(env, entry_r, entry_v, rr)),
            entry_position=(() if air_r is None else tuple(air_r)),
            entry_velocity=(() if air_v is None else tuple(air_v)),
            interface_position=(() if iface_r is None else tuple(iface_r)),
            interface_velocity=(() if iface_v is None else tuple(iface_v)),
            interface_time=(0.0 if iface_t is None else iface_t))

    entry_r = entry_v = None
    iface_r = iface_v = iface_t = None
    air_r = air_v = None
    while t < cfg.PREDICT_MAX_TIME_S:
        radius = vec.norm(r)
        altitude = radius - env.equatorial_radius
        lowest = min(lowest, altitude)
        # A skip is an entry that does not commit.  This airframe holds
        # 30 deg -- *maximum lift* -- through the entry, and if the burn
        # leaves the periapsis high the lift wins before the drag has taken
        # the energy: the vehicle dips into the air, is pushed back out, and
        # leaves on another orbit.  It is not a range error and the range
        # solve cannot see it, because the arc does eventually come down and
        # the propagation dutifully reports where.
        if iface_r is None and altitude < float(cfg.ENTRY_INTERFACE_M):
            iface_r, iface_v, iface_t = r, v, t
        # **The true vacuum/aero boundary, not the skip threshold.**
        # ``entry_r`` is taken ``SKIP_ENTER_MARGIN_M`` *below* the
        # atmosphere top because it feeds skip detection, so using it to
        # split the vacuum leg from the aerodynamic one would put 5 km of
        # air on the wrong side of the line.
        if air_r is None and altitude < env.atmosphere_depth:
            air_r, air_v = r, v
        if altitude < env.atmosphere_depth - cfg.SKIP_ENTER_MARGIN_M:
            if not entered:
                entry_r, entry_v = r, v
            entered = True
        elif entered and altitude > env.atmosphere_depth:
            skipped = True
        speed = vec.norm(v)
        up = vec.scale(r, 1.0 / radius)
        descending = vec.dot(v, up) < 0.0

        if gate is not None:
            distance = surface_distance(env, r, gate)
            if closest is None or distance < closest:
                closest = distance

        if descending and radius <= stop:
            return answer(r, v, t, steps, True, False)
        if radius <= ground:
            return answer(r, v, t, steps, False, True)

        if altitude >= env.atmosphere_depth:
            dt = cfg.PREDICT_DT_VACUUM
        elif altitude >= cfg.PREDICT_UPPER_ALT_M:
            # The air up here decelerates this airframe at 0.04-1.1 m/s^2
            # (measured), so a coarse step costs nothing -- and the answer is
            # converged: an offline entry propagated at 4.0, 2.0, 0.5 and 0.25
            # seconds a step agrees on the range to within 100 m of 1703 km.
            # That is boosterland's "the step is fine enough to have stopped
            # mattering", and it is the property to hold, not the scheme.
            dt = cfg.PREDICT_DT_UPPER
        else:
            dt = cfg.PREDICT_DT_ATMO
        if speed > 1.0:
            # Never step further than a quarter of the way to the stop, so the
            # crossing is resolved rather than jumped over.
            margin = max(1.0, radius - stop)
            dt = min(dt, max(0.05, 0.25 * margin / speed))
        if descending and altitude < env.atmosphere_depth:
            profile.append((speed, altitude))

        r, v = _step(env, r, v, mass, dt, cfg, steer)
        t += dt
        steps += 1

    return answer(r, v, t, steps, False, False)


def miss_components(env, end, predicted, gate):
    """``(long, cross)`` metres from the gate, along and across the runway.

    Arc lengths through the tangent plane at the gate, not a chord projection.
    The chord version is fine for a near miss and catastrophic for a far one,
    and the failure is silent in the worst way: a predicted landing point on
    the *far side of the planet* has an offset from the gate that is almost
    entirely **radial**, so both tangential components come back near zero and
    the guidance reads a perfect hit.  The first in-game flight showed
    ``long=+154 cross=-1140`` while 1533 km from the runway and receding.

    Two signed numbers rather than a distance, for the reason boosterland's
    ``long=``/``cross=`` columns exist: a great-circle distance cannot tell an
    overshoot from an undershoot, and those want opposite corrections.
    """
    along = env.runway.horizontal(end, end["along"])
    up = vec.unit(gate)
    across = vec.unit(vec.cross(up, along))
    other = vec.unit(predicted)
    angle = math.acos(vec.clamp(vec.dot(up, other), -1.0, 1.0))
    if angle < 1e-9:
        return 0.0, 0.0
    direction = vec.project_out(other, up)
    if vec.norm(direction) < 1e-12:
        # Exactly overhead or exactly antipodal.  Overhead was handled above,
        # so this is the antipode, where every direction is equally "towards"
        # and the honest answer is "as wrong as it is possible to be" rather
        # than the zero the algebra would otherwise hand back.
        return math.pi * vec.norm(gate), 0.0
    direction = vec.unit(direction)
    arc = angle * vec.norm(gate)
    return arc * vec.dot(direction, along), arc * vec.dot(direction, across)


def forward_arc(env, r, v, target):
    """Ground-track distance from ``r`` to ``target`` *in the direction of travel*.

    ``surface_distance`` gives the short way round, which is the wrong number
    whenever the vehicle happens to be heading the other way -- and in orbit it
    is heading the other way half the time.  This measures the angle about the
    orbit normal instead, taken forwards, so it runs from 0 to a full
    circumference and is the distance the vehicle will actually cover.

    It exists because the deorbit search was deciding whether a burn reached
    the runway with a *chord projection*.  That is a good approximation within
    a few tens of kilometres and nonsense at a thousand: the first in-game
    flight solved a 335 m/s burn -- nearly the configured maximum -- on its
    very first tick, with the runway 1306 km away and receding, because a
    landing point most of a planet from the gate projected onto the runway's
    local heading came back as a plausible small number.
    """
    normal = vec.cross(r, v)
    if vec.norm(normal) < 1e-6:
        return surface_distance(env, r, target)
    normal = vec.unit(normal)
    a = vec.unit(vec.project_out(r, normal))
    b = vec.project_out(target, normal)
    if vec.norm(b) < 1e-6:
        return surface_distance(env, r, target)
    b = vec.unit(b)
    angle = math.atan2(vec.dot(vec.cross(a, b), normal), vec.dot(a, b))
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle * env.target_radius


def surface_distance(env, a, b):
    """Great-circle distance between two points at the body's surface."""
    ua, ub = vec.unit(a), vec.unit(b)
    dot = vec.clamp(vec.dot(ua, ub), -1.0, 1.0)
    return env.target_radius * math.acos(dot)
