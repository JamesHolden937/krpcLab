"""A speedbrake made of hardware the vehicle already carries.

**The Shuttle this cone is borrowed from uses a speedbrake continuously and
nothing in this autopilot has ever commanded one.**  Every brake constant in
``config.py`` is a wheel brake; the only dissipation controls in the air are
bank and, since ``GEAR_FOR_ENERGY``, the landing gear -- and the gear turns
out to be worth 3% of the glide ratio, not the 19% it was sized on
(``GEAR_DRAG_FRACTION``).  So the approach's one real range control is the
S-turn, and over batches 3-4 of 2026-09-21 the S-turn command sits at its
45-degree cap for **52% of approach ticks with +639 m of surplus still
unspent**.  That is the guidance saying, in its own telemetry, that it has
run out of authority.

The authority it has not been given is the vertical pair.  Two vertical
surfaces deployed in *opposing* directions are a split rudder: the side
forces act at equal and opposite moment arms, so yaw and roll both cancel and
what is left is drag.  On this craft it is free in the strongest sense --
**all six control surfaces have Pitch, Yaw and Roll disabled** and the
vehicle flies on 15 kN m of reaction wheel (docs/spaceplane/design.md, "This vehicle
has no aerodynamic control at all"), so splitting a pair costs no control
authority because they have none to cost.

**Correction, 2026-09-23: the premise of this paragraph is false.**  Every
surface on both craft has all three axes enabled -- the probe read the part
menu's ``Pitch=False``, which is KSP's *ignore* flag, and the torque on the
pad at q=0.  So a deployed surface is *not* free: its deploy angle and its
control deflection share one travel, and deploying it borrows authority from
whatever it is controlling.  Read the probe's ``aoa=cmd/actual`` with that in
mind.  docs/spaceplane/journal.md, "Session, 2026-09-23".

Two rules run this module, and both are about generality rather than about
this aircraft.

**The identification is geometric and refuses rather than guesses.**  The
span axis comes off the part's rotation, a surface is vertical when its span
runs along the vessel's dorsal axis rather than its lateral one, and the
brake arms only on a pair mirrored about the centreline.  No pair, two
candidate pairs, mismatched areas -- **no airbrake**, and the vehicle flies
exactly as it does now.  Horizontal surfaces are never candidates: cancelling
a canard's pitching moment against an elevon's needs a balance of areas and
moment arms that differs on every aircraft, and this autopilot's bar is any
craft a player could fly the entry in.  An unbalanced deployment on final is
failure 34's shape -- the vehicle lands crabbed and tears its gear off.

**The brake is commanded against a surplus, never scheduled.**  The gear is
the warning here: ``APPROACH_SPEED_PATH`` commands the descent angle that
holds the commanded speed, so drag that simply appears is absorbed by
re-trimming alpha and buys almost nothing.  A drag device is a range lever
only while the guidance is spending a surplus with it, which is why the
trigger is the S-turn's own saturation and the release is the surplus
running out.
"""
import math


class Surface(object):
    """One control surface, reduced to what the identification needs.

    ``position`` and ``span`` are in the vessel frame (x lateral, y forward,
    z dorsal/ventral), ``area`` in square metres, and ``key`` is whatever the
    caller needs to find the part again.  Keeping this a plain record is what
    lets the rule be tested offline against the measured geometry of
    ``qs_plane`` without a game running.
    """

    def __init__(self, key, position, span, area=0.0, title=""):
        self.key = key
        self.position = tuple(position)
        self.span = tuple(span)
        self.area = float(area)
        self.title = title or str(key)

    def __repr__(self):                                 # pragma: no cover
        return "Surface(%r, x=%+.2f, span_z=%+.2f)" % (
            self.title, self.position[0], self.span[2])


def span_axis(rotation):
    """The part's own x axis -- its span -- in the frame the rotation is in.

    ``part.rotation(vessel.reference_frame)`` is a quaternion ``(x, y, z,
    w)``; a control surface spans along its local x.  Written out rather than
    pulled from a library because ``common.vec`` is deliberately plain
    3-tuples and this is the only quaternion in either autopilot.
    """
    x, y, z, w = rotation
    vx, vy, vz = 1.0, 0.0, 0.0
    tx, ty, tz = (2.0 * (y * vz - z * vy),
                  2.0 * (z * vx - x * vz),
                  2.0 * (x * vy - y * vx))
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def is_vertical(surface):
    """Vertical when the span runs dorsally rather than laterally.

    A ratio, not a threshold on an angle, so a canted fin is judged by which
    axis it is *more* aligned with and a 45-degree one is rejected by the
    pair test below rather than by a fitted tolerance here.
    """
    return abs(surface.span[2]) > abs(surface.span[0])


def find_split_rudder(surfaces, cfg=None):
    """The one mirrored vertical pair, or ``None`` and the reason why.

    Returns ``(pair, reason)``: ``pair`` is a two-tuple of :class:`Surface`
    ordered left (x < 0) then right, or ``None`` when the brake must not arm.
    ``reason`` always says what was decided, because this runs once at
    STANDBY and the log is the only place anyone will ever see it.

    **A missing answer must not look like a good one** (CLAUDE.md): there is
    no "closest pair" fallback and no single-surface mode.  One surface
    deployed alone is a rudder deflection nobody commanded.
    """
    tol = 0.20 if cfg is None else float(
        getattr(cfg, "AIRBRAKE_PAIR_TOL_M", 0.20))
    min_offset = 0.20 if cfg is None else float(
        getattr(cfg, "AIRBRAKE_MIN_OFFSET_M", 0.20))
    area_tol = 0.25 if cfg is None else float(
        getattr(cfg, "AIRBRAKE_AREA_TOL", 0.25))

    verticals = [s for s in surfaces if is_vertical(s)]
    if len(verticals) < 2:
        return None, ("no airbrake: %d vertical surface(s) of %d"
                      % (len(verticals), len(surfaces)))

    pairs = []
    for i, a in enumerate(verticals):
        for b in verticals[i + 1:]:
            # Mirrored about the centreline: the x offsets cancel, and
            # neither sits on it -- a centreline fin has no moment arm to
            # cancel against and splitting it against a wingtip one yaws.
            if abs(a.position[0] + b.position[0]) > tol:
                continue
            if min(abs(a.position[0]), abs(b.position[0])) < min_offset:
                continue
            # Same station fore-and-aft, or the drag couple pitches.
            if abs(a.position[1] - b.position[1]) > tol:
                continue
            if a.area > 0.0 and b.area > 0.0:
                bigger = max(a.area, b.area)
                if abs(a.area - b.area) / bigger > area_tol:
                    continue
            pairs.append((a, b) if a.position[0] < b.position[0] else (b, a))

    if not pairs:
        return None, ("no airbrake: %d vertical surface(s), no mirrored pair"
                      % len(verticals))
    if len(pairs) > 1:
        # Refuse rather than pick.  Two pairs is a vehicle whose geometry
        # this rule does not understand, and the failure mode of guessing is
        # an uncommanded yaw on final.
        return None, ("no airbrake: %d candidate pairs, refusing to choose"
                      % len(pairs))
    left, right = pairs[0]
    return pairs[0], ("airbrake armed: %s / %s, x=%+.2f/%+.2f, %.1f m^2 each"
                      % (left.title, right.title, left.position[0],
                         right.position[0], left.area))


def is_horizontal(surface):
    """Horizontal when the span runs laterally rather than dorsally."""
    return not is_vertical(surface)


def find_opposed_flaps(surfaces, com_y=0.0, cfg=None):
    """Fore and aft horizontal groups whose pitching moments can cancel.

    **The user's mechanism, and it is a better one than the split rudder.**
    Canards sit ahead of the centre of mass and elevons behind it, so the
    *same* trailing-edge sense gives them opposite pitching moments: deflect
    both trailing-edge up and the canard's nose-down moment cancels the
    elevon's nose-up one, while both surfaces spoil lift and both make drag.

    That is the right currency, which the split rudder was not.  Drag alone
    spends *speed*, and a glider buys speed back by diving -- measured, 21
    m/s gone in eleven seconds and two vehicles into the ground at 79-82 m/s
    of sink.  Sideslip worked because it spoils lift at constant drag, which
    spends *height*.  ``tan(gamma) = D/L``: less lift and more drag together
    is a steeper path at the same speed, and height is the surplus the
    approach cannot get rid of.

    **This module used to refuse horizontal surfaces on principle**, and the
    stated reason was that balancing a canard against an elevon "needs a
    balance of areas and moment arms that differs on every aircraft".  That
    is not a disqualification in this project, it is the definition of
    something to measure: kRPC reports ``surface_area`` and the geometry is
    already read here, so the balance is *computed per vehicle* rather than
    fitted for one.  The same objection would have retired
    ``airframe.measure``, ``Holdable`` and ``turn_load``.

    Returns ``(forward, aft, ratio, reason)``.  ``ratio`` is how much less
    the forward group must deflect for the moments to cancel, so a caller
    deflecting the aft group by ``d`` deflects the forward one by
    ``d * ratio``.  ``None`` and a reason when the vehicle's geometry does
    not support it -- **there is no closest-fit fallback**, because an
    unbalanced deployment is an uncommanded pitch input and failure 34's
    shape one phase later.
    """
    tol = 0.20 if cfg is None else float(
        getattr(cfg, "AIRBRAKE_PAIR_TOL_M", 0.20))
    min_arm = 0.20 if cfg is None else float(
        getattr(cfg, "AIRBRAKE_MIN_OFFSET_M", 0.20))

    horizontals = [s for s in surfaces if is_horizontal(s)]
    if len(horizontals) < 2:
        return None, None, 0.0, ("no flap brake: %d horizontal surface(s) "
                                 "of %d" % (len(horizontals), len(surfaces)))

    forward = [s for s in horizontals if s.position[1] - com_y > min_arm]
    aft = [s for s in horizontals if com_y - s.position[1] > min_arm]
    if not forward or not aft:
        return None, None, 0.0, (
            "no flap brake: %d forward and %d aft of the centre of mass -- "
            "nothing to cancel against" % (len(forward), len(aft)))

    # **Each group has to be laterally balanced on its own**, or deploying it
    # rolls the vehicle.  Same test the split rudder uses, applied per group
    # rather than to one pair.
    for name, group in (("forward", forward), ("aft", aft)):
        offset = sum(s.position[0] for s in group)
        if abs(offset) > tol:
            return None, None, 0.0, (
                "no flap brake: the %s group is %.2f m off the centreline -- "
                "deploying it would roll" % (name, offset))

    def moment(group):
        return sum(max(0.0, s.area) * abs(s.position[1] - com_y)
                   for s in group)

    fwd_moment, aft_moment = moment(forward), moment(aft)
    if fwd_moment <= 0.0 or aft_moment <= 0.0:
        return None, None, 0.0, ("no flap brake: no usable area x arm "
                                 "(forward %.2f, aft %.2f)"
                                 % (fwd_moment, aft_moment))
    ratio = aft_moment / fwd_moment
    return forward, aft, ratio, (
        "flap brake armed: %d forward (%.1f m^2 x arm) against %d aft "
        "(%.1f m^2 x arm), forward deflects x%.2f"
        % (len(forward), fwd_moment, len(aft), aft_moment, ratio))


class Armed(object):
    """What of this vehicle can be deployed as a brake, and what cannot.

    Three independent groups, because they cancel on **different axes** and
    therefore compose without interfering:

    * the mirrored *vertical* pair, whose side forces cancel in yaw and roll
      by symmetry (the split rudder);
    * the *forward* horizontal group and the *aft* one, whose pitching
      moments cancel against each other at the ``ratio`` of their area times
      arm (the opposed flaps).

    Nothing about deploying both at once breaks either cancellation, and a
    vehicle that can do both has **all** of its control surfaces available as
    drag.  On the craft this autopilot grew up on that is the difference
    between 2 m^2, 4 m^2 and 6 m^2 against a whole-craft ``CdA`` of 5.5 --
    and the 2 m^2 version alone was worth 21 m/s in eleven seconds.

    **One group refusing must not veto the others.**  A craft may have
    canards and no mirrored fin, or a fin and no foreplane, and the brake it
    can make is the brake it should get.  ``reasons`` keeps every decision
    because a brake that silently did not arm reads, in the logs, exactly
    like one that armed and did nothing -- and those want opposite next
    steps.
    """

    def __init__(self, pair=None, forward=None, aft=None, ratio=0.0,
                 reasons=()):
        self.pair = pair
        self.forward = forward
        self.aft = aft
        self.ratio = ratio
        self.reasons = tuple(reasons)

    @property
    def any(self):
        return bool(self.pair) or bool(self.forward and self.aft)

    def area(self):
        """Total surface area this brake can deploy, in square metres."""
        total = 0.0
        for group in (self.pair or (), self.forward or (), self.aft or ()):
            total += sum(max(0.0, s.area) for s in group)
        return total

    def surfaces(self):
        """Every surface and the deflection multiplier it takes.

        The aft group and the vertical pair take the commanded angle; the
        forward group takes ``ratio`` times it, because it is usually small
        and on a long arm.  **Deploying both horizontal groups to the same
        angle is not a brake, it is a large uncommanded pitch input.**
        """
        out = []
        for surface in (self.pair or ()):
            out.append((surface, 1.0))
        for surface in (self.aft or ()):
            out.append((surface, 1.0))
        for surface in (self.forward or ()):
            out.append((surface, self.ratio))
        return out

    def describe(self):
        if not self.any:
            return "brake: nothing armed -- " + "; ".join(self.reasons)
        parts = []
        if self.pair:
            parts.append("split rudder (%d surfaces)" % len(self.pair))
        if self.forward and self.aft:
            parts.append("opposed flaps (%d forward x%.2f against %d aft)"
                         % (len(self.forward), self.ratio, len(self.aft)))
        return ("brake armed: %s, %.1f m^2 total | %s"
                % (" + ".join(parts), self.area(), "; ".join(self.reasons)))


def find_all_brakes(surfaces, com_y=0.0, cfg=None):
    """Everything this vehicle can deploy without disturbing its attitude.

    Runs both identifications and keeps whichever arm.  See :class:`Armed`
    for why they compose, and ``find_split_rudder`` /
    ``find_opposed_flaps`` for the individual rules -- both of which refuse
    rather than guess, and both of which keep refusing here.
    """
    reasons = []
    pair, why = find_split_rudder(surfaces, cfg)
    reasons.append(why)
    forward, aft, ratio, why = find_opposed_flaps(surfaces, com_y, cfg)
    reasons.append(why)
    return Armed(pair, forward, aft, ratio, reasons)


class Brake(object):
    """Extended or not, with the hysteresis that keeps it from chattering.

    The state is deliberately here rather than in the phase machine: the
    trigger is a *fraction of recent ticks saturated*, which needs memory,
    and a latch that lives in the autopilot is a latch that is reset by every
    phase change.

    ``update`` takes what ``guidance.approach`` already computed -- the
    S-turn command, its cap, the surplus height and the height -- and returns
    the wanted state.  Nothing in here reads the game.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.extended = False
        self.saturated = 0.0            # share of the phase spent at the cap
        self.capped_s = 0.0
        self.elapsed_s = 0.0
        self.changes = 0
        self.last_reason = ""

    def update(self, dt, scurve_deg, scurve_max_deg, excess, height,
               flare_trigger=0.0, sink=0.0, speed=None, target_speed=None,
               gravity=9.81):
        """Whether the brake should be out, this tick.

        **Every threshold here is derived from a constant some other phase
        already owns**, because a brake with three fitted numbers of its own
        is a brake that has to be re-fitted on the next aircraft -- and the
        bar is any craft a player could fly the entry in, not this one.

        * *Out* when the weave has held its cap for
          ``AIRBRAKE_SATURATED_FRAC`` of the window, with more surplus left
          than ``APPROACH_SCURVE_M`` -- the same surplus that started the
          weave.  One quantity, one constant: the brake arms exactly when the
          guidance is weaving and has run out of weave.
        * The share is **capped seconds over elapsed seconds of the phase**,
          believed once ``AIRBRAKE_MIN_CYCLES`` full weave cycles
          (``2 x APPROACH_SCURVE_PERIOD_S``) have been sampled.  It was an
          exponential average and that measured nothing -- see the comment
          in ``update`` and ``AIRBRAKE_MIN_CYCLES``.
        * *In* again at ``AIRBRAKE_RETRACT_FRAC`` of that same surplus, which
          is the hysteresis and nothing else, and always
          ``AIRBRAKE_STOW_LEAD_S`` seconds of sink above the flare door the
          approach itself computes.  **A time, not a height**: the door is
          ``FLARE_ALT_M + FLARE_LEAD_S * sink`` on whatever vehicle this is,
          and a brake still out at the flare turns a float into a drop.
        * *And never below the speed the approach is holding.*  **This is
          what the first flight of the brake got wrong, and it destroyed two
          vehicles.**  The brake took 21 m/s out of the approach
          (105.6 -> 84.1, LOG2825), the speed loop can only make speed by
          trading height for it, and the dive it commanded to recover
          arrived at the flare door at **250 m with 79-82 m/s of sink**
          against 29-36 unbraked.  Both flights came apart on contact.
          The surplus this brake exists to spend is *height*; the speed is
          the approach's margin and is not the brake's to take.
        """
        cfg = self.cfg
        at_cap = (scurve_max_deg > 0.0
                  and scurve_deg >= scurve_max_deg - float(
                      getattr(cfg, "AIRBRAKE_CAP_EPS_DEG", 0.5)))
        # **The share of the phase spent at the cap, not an exponential
        # average of it.**  Flown as an EMA this measured nothing: the
        # average starts cold at the phase boundary and the window that
        # spans a weave cycle is half the approach, so it was still charging
        # when the vehicle reached the stow height.  Of four flights, two
        # deployed for seven seconds at 702 m and two peaked at 0.44-0.48
        # and never armed at all (LOG2815-2818).  **The quantity the
        # mechanism was built on is a ratio of ticks** -- "the cap for 52% of
        # approach ticks" -- and a ratio is what this now keeps: capped
        # seconds over elapsed seconds, unbiased from the first tick, with a
        # minimum sample instead of a time constant.
        self.elapsed_s += max(0.0, dt)
        if at_cap:
            self.capped_s += max(0.0, dt)
        if self.elapsed_s > 0.0:
            self.saturated = self.capped_s / self.elapsed_s
        # A minimum sample, because three ticks of a weave at its cap is
        # where the weave *is*, not evidence that it is stuck there.  One
        # full cycle is the shortest span over which the share means
        # anything, and the cycle is the craft's own constant.
        sampled = self.elapsed_s >= (
            float(getattr(cfg, "AIRBRAKE_MIN_CYCLES", 1.0))
            * 2.0 * float(getattr(cfg, "APPROACH_SCURVE_PERIOD_S", 10.0)))

        arm_at = float(getattr(cfg, "APPROACH_SCURVE_M", 200.0))
        # The speed guard, above every other consideration: a brake cannot be
        # allowed to buy height with speed the flare needs.
        if (speed is not None and target_speed is not None
                and speed < target_speed * float(
                    getattr(cfg, "AIRBRAKE_SPEED_GUARD", 1.0))):
            return self._set(False, "speed %.0f below target %.0f"
                             % (speed, target_speed))
        # **And never with more sink than the flare can arrest from its
        # door.**  The flare flies ``sqrt(td^2 + 2 a h)`` with
        # ``a = (FLARE_TRACK_LOAD - 1) g`` (``guidance.flare_command``), so
        # the most sink it can take at the door is that at the door's height.
        # Measured with the opposed flaps on ``qs_plane``: the brake moved the
        # touchdown ~600 m earlier -- the first thing that ever has -- and
        # reached the door at 38-58 m/s of sink against 29-36 unbraked, past
        # the ~39 the committed flare can take; 3 of 6 broke up
        # (`pairfly-flapbrake.txt`).  Every number here is the flare's own.
        if getattr(cfg, "AIRBRAKE_SINK_GUARD", False) and flare_trigger > 0.0:
            rise = max(0.05, float(getattr(cfg, "FLARE_TRACK_LOAD", 1.5))
                       - 1.0)
            touchdown = float(getattr(cfg, "FLARE_TOUCHDOWN_SINK_M_S", 8.0))
            limit = math.sqrt(touchdown * touchdown
                              + 2.0 * rise * gravity * flare_trigger)
            if sink > limit:
                return self._set(False, "sink %.0f above the %.0f the flare "
                                        "can arrest from its door"
                                 % (sink, limit))
        stow = (flare_trigger
                + float(getattr(cfg, "AIRBRAKE_STOW_LEAD_S", 4.0))
                * max(0.0, sink))
        if height <= stow:
            return self._set(False, "%.0f m, the flare door plus %.0fs"
                             % (height,
                                float(getattr(cfg, "AIRBRAKE_STOW_LEAD_S",
                                              4.0))))
        if self.extended:
            if excess <= arm_at * float(getattr(cfg, "AIRBRAKE_RETRACT_FRAC",
                                                0.5)):
                return self._set(False, "surplus spent (%+.0f m)" % excess)
            return True
        if (sampled
                and self.saturated >= float(
                    getattr(cfg, "AIRBRAKE_SATURATED_FRAC", 0.5))
                and excess > arm_at):
            return self._set(True, "S-turn saturated %.0f%% with %+.0f m left"
                             % (100.0 * self.saturated, excess))
        return False

    def _set(self, wanted, reason):
        if wanted != self.extended:
            self.extended = wanted
            self.changes += 1
            self.last_reason = reason
        return wanted


class MeasuredBrake(Armed):
    """The opposed flaps chosen by measurement, not by geometry.

    **Positive ``Deploy Angle`` is not one direction.**  KSP's deploy sense is
    a per-part setting, and on the shuttle a positive angle takes one pair of
    main elevons trailing-edge up (lift -9.1 ClA) and the other pair and the
    forward pair trailing-edge *down* (lift +4.7, +5.5): the geometric brake
    deployed all of them at +20 and was half a flap.  The old craft's probe
    saw the same thing from outside -- its "brake" *raised* L/D
    hypersonically, 0.88 -> 1.27.

    So each surface is deployed both ways in vacuum and the game's own
    ``simulate_aerodynamic_wrench_at`` says what it does.  Each keeps the
    sense that spoils its lift; the surfaces are split by the **sign of the
    moment they then make** (not by station -- the shuttle's main elevons sit
    on its centre of mass and pitch by camber, not by arm), and the stronger
    side is scaled down until the moments cancel.  What is left is a lift
    spoiler, which is the currency sideslip proved and a drag brake is not.
    """

    def __init__(self, entries, lift, moment, reasons=(), parts=(),
                 gains=(1.0, 1.0)):
        super(MeasuredBrake, self).__init__(reasons=reasons)
        self.entries = list(entries)     # (record, signed multiplier)
        self.lift = lift                 # predicted dClA at full deployment
        self.moment = moment             # predicted residual dCmA
        # (record, sense, dClA, dCmA) at unit gain, and the (nose-up,
        # nose-down) side gains -- what ``rebalanced`` needs to correct a
        # measured residual without re-probing every surface.
        self.parts = list(parts)
        self.gains = tuple(gains)

    def slopes(self):
        """dCmA per unit gain of the nose-up side and the nose-down side."""
        up = sum(dm for _, _, _, dm in self.parts if dm > 0.0)
        down = sum(dm for _, _, _, dm in self.parts if dm <= 0.0)
        return up, down

    def with_gains(self, g_up, g_down):
        entries, lift, moment = [], 0.0, 0.0
        for record, sense, dl, dm in self.parts:
            g = g_up if dm > 0.0 else g_down
            entries.append((record, sense * g))
            lift += g * dl
            moment += g * dm
        out = MeasuredBrake(entries, lift, moment, self.reasons, self.parts,
                            (g_up, g_down))
        out.kind = getattr(self, "kind", "spoiler")
        return out

    def balance(self):
        """The two side gains as one number ``x`` in [0, 2]: 0 is the
        nose-down side alone, 1 both sides full, 2 the nose-up side alone.
        The moment rises monotonically along it, which is what lets a
        bracket be searched."""
        g_up, g_down = self.gains
        return g_up if g_down >= 1.0 - 1e-9 else 2.0 - g_down

    def at_balance(self, x):
        x = max(0.0, min(2.0, x))
        if x <= 1.0:
            return self.with_gains(x, 1.0)
        return self.with_gains(1.0, 2.0 - x)

    def rebalanced(self, residual, history=()):
        """Move the balance against a *measured* residual moment.

        Deflections do not add, and a correction taken from the single-
        surface slopes overshoots: the shuttle's flap set went 0.92 -> 0.20
        -> 0.72 -> 0.28 with the moment +78 -> -56 -> +48 -> -42 and never
        converged.  So once the measured ``history`` of ``(balance,
        moment)`` brackets zero, the next balance is the **regula falsi**
        point between the tightest bracket -- the game's wrench closes the
        balance and the prediction only seeds it.
        """
        x = self.balance()
        pts = list(history) + [(x, residual)]
        below = [p for p in pts if p[1] < 0.0]
        above = [p for p in pts if p[1] > 0.0]
        if below and above:
            lo = max(below, key=lambda p: p[0])    # most nose-up of the lows
            hi = min(above, key=lambda p: p[0])    # most nose-down of highs
            if hi[1] != lo[1]:
                x_new = lo[0] + (0.0 - lo[1]) * (hi[0] - lo[0]) / (
                    hi[1] - lo[1])
                return self.at_balance(x_new)
        s_up, s_down = self.slopes()
        # d(moment)/dx: along [0,1] the nose-up gain moves, along [1,2]
        # the nose-down gain falls
        slope = s_up if x < 1.0 else -s_down
        if slope <= 0.0:
            return self
        return self.at_balance(x - residual / slope)

    @property
    def any(self):
        return bool(self.entries)

    def area(self):
        return sum(max(0.0, r.area) for r, _ in self.entries)

    def surfaces(self):
        return list(self.entries)

    def describe(self):
        kind = getattr(self, "kind", "spoiler")
        if not self.entries:
            return "%s (measured): nothing armed -- " % kind + "; ".join(
                self.reasons)
        return ("%s (measured): %d surfaces, predicted dClA %+.1f, "
                "residual dCmA %+.1f | %s"
                % (kind, len(self.entries), self.lift, self.moment,
                   " ".join("%s x%+.2f" % (r.title, m)
                            for r, m in self.entries)))


def choose_measured_brake(samples):
    """The spoiler: see :func:`choose_measured_set`."""
    return choose_measured_set(samples, spoil=True)


def choose_measured_set(samples, spoil=True):
    """``samples``: ``(record, dL_plus, dM_plus, dL_minus, dM_minus)`` per
    surface, the change in ``ClA`` and pitching ``CmA`` for a deployment of
    ``+theta`` and ``-theta``.  Each surface takes the sense that spoils its
    lift (``spoil``) or adds to it; the two moment sides are balanced.
    Returns a :class:`MeasuredBrake`, empty with a reason when no
    moment-cancelling set exists.
    """
    word = "spoiler" if spoil else "flap"
    sign = 1.0 if spoil else -1.0        # spoil: most negative lift wins
    chosen = []
    for record, lp, mp, lm, mm in samples:
        if spoil and min(lp, lm) >= 0.0:
            continue                     # spoils nothing either way
        if not spoil and max(lp, lm) <= 0.0:
            continue                     # adds nothing either way
        if sign * lp <= sign * lm:
            chosen.append((record, 1.0, lp, mp))
        else:
            chosen.append((record, -1.0, lm, mm))
    pos = [c for c in chosen if c[3] > 0.0]
    neg = [c for c in chosen if c[3] <= 0.0]
    m_pos = sum(c[3] for c in pos)
    m_neg = -sum(c[3] for c in neg)
    if not pos or not neg or m_pos <= 0.0 or m_neg <= 0.0:
        return MeasuredBrake([], 0.0, 0.0, reasons=(
            "no measured %s: every deployment in that sense pitches the "
            "same way (%d nose-up, %d nose-down)" % (word, len(pos), len(neg)),))
    g_pos = min(1.0, m_neg / m_pos)
    g_neg = min(1.0, m_pos / m_neg)
    parts = [(record, sense, dl, dm) for record, sense, dl, dm in chosen]
    brake = MeasuredBrake([], 0.0, 0.0, reasons=(
        "measured: nose-up side x%.2f, nose-down side x%.2f"
        % (g_pos, g_neg),), parts=parts).with_gains(g_pos, g_neg)
    if sign * brake.lift >= 0.0:
        return MeasuredBrake([], 0.0, 0.0, reasons=(
            "no measured %s: the moment-cancelling set changes lift the "
            "wrong way (dClA %+.1f)" % (word, brake.lift),))
    brake.kind = word
    return brake


def drag_brake_fraction(cfg, speed, target, excess, height, was_out):
    """How far out the in-flight drag brake should be, in [0, 1].

    **For "too fast at the right height"** (the user, 2026-09-25) -- the
    one surplus the spoiler cannot spend, because a spoiler spends *height*.
    Out when the approach is ``AIR_DRAG_ON_M_S`` over its own
    ``target_speed`` and not below its height profile by more than
    ``AIR_DRAG_LOW_M`` (below it, the speed is the height it is short of
    and must not be braked away); in again under ``AIR_DRAG_OFF_M_S``
    (hysteresis) or under ``AIR_DRAG_MIN_H_M``.  Proportional between, in
    quarters, so the deploy field is written only when the step changes.
    """
    if speed is None or target is None:
        return 0.0
    over = speed - target
    if height < cfg.AIR_DRAG_MIN_H_M or excess < -cfg.AIR_DRAG_LOW_M:
        return 0.0
    if over < (cfg.AIR_DRAG_OFF_M_S if was_out else cfg.AIR_DRAG_ON_M_S):
        return 0.0
    frac = max(0.0, min(1.0, over / cfg.AIR_DRAG_FULL_M_S))
    return max(0.25, math.ceil(frac * 4.0 - 1e-9) / 4.0)

