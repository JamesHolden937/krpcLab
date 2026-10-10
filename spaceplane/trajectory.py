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
from dataclasses import dataclass

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
    # **The predicted handover**: ``(altitude, speed)`` where the arc first
    # meets the cone's entry test (Mach at most ``HAC_ENTRY_MACH`` and within
    # ``HAC_ENTRY_DIST_M`` of the gate, or down to the gate's altitude), so
    # the log can set the energy the prediction promises the cone against
    # the one it gets.  ``None`` when the arc never meets it.
    handover: object = None
    # Whether ``handover`` is where the arc met the entry test (and where:
    # ``handover_state`` = ``(r, v)``), or only the fallback at the target.
    handover_met: bool = False
    handover_state: object = None
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


def past_interface(env, cfg, altitude):
    """Whether ``altitude`` (above the body's datum) is inside the entry,
    where the glide flies rather than the coast.

    ``ENTRY_INTERFACE_M`` (58 km) is a Kerbin altitude with no recorded
    reason.
    """
    return altitude < float(cfg.ENTRY_INTERFACE_M)


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
    vs = airframe.stall(env, cfg)
    if vs is None:                  # no table yet: no speed hold either
        return glide_alpha_max(cfg, env, speed, altitude)
    approach = cfg.GLIDE_ARRIVAL_FACTOR * vs
    if speed >= cfg.SPEED_HOLD_FACTOR * approach:
        return glide_alpha_max(cfg, env, speed, altitude)
    trim = alpha_for_load(env, speed, altitude, mass, gravity, 1.0)
    if trim is None:
        return glide_alpha_max(cfg, env, speed, altitude)
    wanted = trim + cfg.SPEED_HOLD_KP * (speed - approach)
    return vec.clamp(wanted, cfg.ALPHA_MIN_DEG, glide_alpha_max(cfg, env, speed, altitude))


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
    if cfg is None:
        return cfg_default_min(cfg)
    # Only while the glide has surplus to spend: see
    # ``Config.SPEED_FLOOR_WHEN_LONG``.  ``env.spending`` is set by the
    # control loop from its own prediction each tick, so the propagations of
    # that tick all see the same law -- which is the point of hanging it on
    # the environment.
    if getattr(env, "spending", True) is False:
        return cfg.ALPHA_MIN_DEG
    vs = airframe.stall(env, cfg)
    if vs is None:                  # no table yet: no floor either
        return cfg.ALPHA_MIN_DEG
    approach = cfg.GLIDE_ARRIVAL_FACTOR * vs
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


def glide_alpha_max(cfg, env=None, speed=None, altitude=None):
    """The glide's alpha ceiling.

    With ``GLIDE_ALPHA_PLATEAU`` and a table to read: the far edge of this
    wing's lift plateau at this Mach -- the highest alpha whose ``ClA`` is
    still within that fraction of the row's peak (see the config entry).
    Else ``GLIDE_ALPHA_MAX_DEG``, else ``ALPHA_MAX_DEG``.
    """
    frac = float(getattr(cfg, "GLIDE_ALPHA_PLATEAU", 0.0) or 0.0)
    if frac > 0.0 and env is not None and speed and altitude is not None:
        got = plateau_edge(env, frac, speed, altitude)
        if got is not None:
            return got
    top = float(getattr(cfg, "GLIDE_ALPHA_MAX_DEG", 0.0) or 0.0)
    return top if top > 0.0 else cfg.ALPHA_MAX_DEG


def plateau_edge(env, frac, speed, altitude):
    """Highest alpha with ``ClA >= (1 - frac) * peak`` at this Mach, from the
    swept table, interpolated; ``None`` when the table cannot answer.
    Cached on ``env`` per 0.1 of Mach -- the propagator asks every step."""
    try:
        if not env.ready():
            return None
        mach = env.mach(speed, altitude)
    except Exception:                                       # noqa: BLE001
        return None
    key = round(mach, 1)
    cache = getattr(env, "_plateau_cache", None)
    if cache is None:
        cache = env._plateau_cache = {}
    if key in cache:
        return cache[key]
    alphas = sorted(getattr(env, "_alphas", ()) or ())
    out = None
    if alphas:
        try:
            lift = [env.coefficients(a, speed, altitude)[0] for a in alphas]
        except Exception:                                   # noqa: BLE001
            lift = None
        if lift and max(lift) > 0.0:
            i = max(range(len(lift)), key=lambda k: lift[k])
            floor = (1.0 - frac) * lift[i]
            out = alphas[i]
            for k in range(i + 1, len(alphas)):
                if lift[k] >= floor:
                    out = alphas[k]
                    continue
                span = lift[k - 1] - lift[k]
                share = (lift[k - 1] - floor) / span if span > 0.0 else 0.0
                out = alphas[k - 1] + share * (alphas[k] - alphas[k - 1])
                break
    cache[key] = out
    return out


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
    ``simulate_aerodynamic_force_at`` returns a force.  (kRPC 0.6 *does*
    report the moment -- ``simulate_aerodynamic_wrench_at`` -- and can hold a
    surface deflection; see the journal, "Session, 2026-09-23 (second)".)

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
        saturated = short > float(self.cfg.HOLDABLE_SATURATED_DEG)
        if saturated:
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

    def limit(self, q, mach=None):
        """The ceiling at this ``q``, or ``None`` where there is no evidence."""
        if q < float(self.cfg.HOLDABLE_MIN_Q):
            return None
        trusted = [(i, v[0]) for i, v in self.bins.items()
                   if v[1] >= int(self.cfg.HOLDABLE_MIN_SAMPLES)]
        if not trusted:
            return None
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
            else:
                # Thinner air than anything flown: nothing has refused a
                # command up here, so do not invent a limit.
                return None
        return best + float(self.cfg.HOLDABLE_MARGIN_DEG)

    def summary(self):
        """One short string for the log, so a flight says what it learned."""
        trusted = sorted((i, v) for i, v in self.bins.items()
                         if v[1] >= int(self.cfg.HOLDABLE_MIN_SAMPLES))
        per = max(1, int(self.cfg.HOLDABLE_Q_DECADE_BINS))
        return " ".join("%.0fPa:%.1f/%d" % (
            10.0 ** (i / float(per)), v[0], v[1]) for i, v in trusted)


def holdable_alpha(cfg, alpha, q, holdable=None, env=None, mach=None):
    """Clamp a commanded angle to what the airframe has shown it can hold.

    ``holdable`` is a ``Holdable`` carried on the ``Steer``, so the propagator
    and the control loop consult the same learned plant -- the project's
    "predict the law you fly", in the one place where the plant limit is
    knowable only by watching.  With no estimator, or none with evidence at
    this ``q``, the command is returned untouched.
    """
    if holdable is None:
        return alpha
    limit = holdable.limit(q, mach)
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
                    up_perp, _ = lift_frame(r, v)
                    if up_perp is not None:
                        share = math.cos(math.radians(bank))
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


def _step(env, r, v, mass, dt, cfg, steer):
    return _rk4(env, r, v, mass, dt, steer)



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
    handover = None
    handover_state = None
    entry_mach = float(getattr(cfg, "HAC_ENTRY_MACH", 0.0) or 0.0)
    entry_dist = float(getattr(cfg, "HAC_ENTRY_DIST_M", 0.0) or 0.0)

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
            handover=(handover if handover is not None
                      else ((radius - env.equatorial_radius, vec.norm(vv))
                            if reached else None)),
            handover_met=handover is not None,
            handover_state=handover_state,
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
        if iface_r is None and past_interface(env, cfg, altitude):
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
            if (handover is None and entry_dist > 0.0
                    and distance <= entry_dist and descending):
                try:
                    slow = speed <= entry_mach * env.speed_of_sound(altitude)
                except Exception:                       # noqa: BLE001
                    slow = False
                if slow:
                    handover = (altitude, speed)
                    handover_state = (tuple(r), tuple(v))

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
