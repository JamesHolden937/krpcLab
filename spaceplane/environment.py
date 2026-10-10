"""Measured world: the air, the frame, the runway, and the airframe's table.

Everything physical here is measured rather than assumed -- ``mu``, the body
radius, the whole density profile, the local speed of sound, the rotation
vector, and both aerodynamic coefficients.  That is inherited from
``boosterland.environment`` and for the same reasons; what is different is the
shape of the aerodynamic answer.

**Two dimensions, not one.**  A booster can hold ``Cd*A`` against Mach alone,
because a cylinder's drag barely cares what angle it is held at and it has no
lift worth the name.  This airframe's lift varies ninefold between subsonic
and Mach 5 -- ``Cl*A`` at 12 degrees measures 36 m^2 at Mach 0.15 and 5.5 m^2
at Mach 2.35 -- and *both* coefficients depend strongly on angle of attack,
which is the control.  So the table is ``(alpha, Mach) -> (Cl*A, Cd*A)``.

**The airflow is aimed, not the vessel.**  ``simulate_aerodynamic_force_at``
takes the velocity to evaluate at, so a bin can be probed at exactly the angle
of attack it tabulates by constructing a velocity that makes that angle with
the vehicle's own axes, leaving the attitude alone.  That removes at a stroke
the failure that cost ``boosterland`` two sessions (failures 13 and 14): every
bin there was written while the booster sat broadside and then used to fly a
nose-on descent, and the fix was to re-sweep the curve repeatedly and hope the
two converged.  Here the probe is *always* in the configuration it is
tabulating, because the configuration is an input.

The trap that replaces it is subtler and is the reason ``probe_altitudes``
exists: KSP scales drag by a pseudo-Reynolds term as well as Mach, so the same
Mach reads 15-20% lower down low than it does at 30 km.  A bin has to be
probed in the air it will be *used* in, which means knowing where the vehicle
will be when it is doing that speed -- which comes from a prediction made with
the table being replaced.  So the altitudes start from a nominal profile and
are refined from the prediction's own once there is one.
"""
import math

from common import krpcbatch, vec


class Table:
    """A coefficient against ``(alpha, Mach)``, probed sparsely, read densely.

    Two jobs, and they pull in opposite directions.  Filling it is sparse and
    irregular: rows arrive as the vehicle probes them, some never get measured
    at all, and a cell nobody has asked about must not be invented.  Reading it
    is the innermost operation in the whole program -- four acceleration
    evaluations per RK4 step, a couple of thousand steps a propagation, four
    propagations a guidance tick -- so it has to be a couple of microseconds.

    The first version did the sparse handling *inside* the read: a neighbour
    search per lookup to find the nearest measured cell.  That measured
    166 microseconds a step against boosterland's 2.4, and put a single entry
    propagation at a third of a second, which is three thousand seconds of CPU
    over an entry.  So the two jobs are separated: writes mark the table
    dirty, and ``resolve`` collapses it once into a dense grid that the read
    can interpolate with two bisections and no branches.
    """

    def __init__(self, alphas, machs):
        self.alphas = tuple(float(a) for a in alphas)
        self.machs = tuple(float(m) for m in machs)
        self.raw = [[None] * len(self.alphas) for _ in self.machs]
        self._dense = None
        self.measured = 0
        # Bumped on every write.  Anything that caches a value derived from
        # this table keys on it, so a re-probe invalidates the cache without
        # anyone having to remember to.
        self.generation = 0

    def set(self, row, column, value, weight=1.0):
        old = self.raw[row][column]
        self.raw[row][column] = (value if old is None
                                 else old + weight * (value - old))
        if old is None:
            self.measured += 1
        self._dense = None
        self.generation += 1

    def filled(self):
        return self.measured > 0

    def resolve(self):
        """Fill every cell from the nearest measured row in the same column.

        Nearest *in Mach*, holding rather than extrapolating: a coefficient
        nobody has probed is not improved by projecting a trend into it, which
        is the same choice boosterland's drag curve makes at the ends of its
        range.
        """
        rows, columns = len(self.machs), len(self.alphas)
        dense = [[0.0] * columns for _ in range(rows)]
        for column in range(columns):
            have = [r for r in range(rows) if self.raw[r][column] is not None]
            if not have:
                continue
            for r in range(rows):
                nearest = min(have, key=lambda h: abs(h - r))
                dense[r][column] = self.raw[r][column] \
                    if self.raw[r][column] is not None \
                    else self.raw[nearest][column]
        self._dense = dense
        return dense

    def lookup(self, alpha_deg, mach):
        dense = self._dense if self._dense is not None else self.resolve()
        a = abs(alpha_deg)
        alphas, machs = self.alphas, self.machs

        i = 0
        n = len(machs) - 1
        while i < n and machs[i + 1] < mach:
            i += 1
        if i < n and machs[i + 1] > machs[i]:
            mf = (mach - machs[i]) / (machs[i + 1] - machs[i])
            mf = 0.0 if mf < 0.0 else (1.0 if mf > 1.0 else mf)
        else:
            mf = 0.0
        j = i + 1 if i < n else i

        k = 0
        m = len(alphas) - 1
        while k < m and alphas[k + 1] < a:
            k += 1
        if k < m and alphas[k + 1] > alphas[k]:
            af = (a - alphas[k]) / (alphas[k + 1] - alphas[k])
            af = 0.0 if af < 0.0 else (1.0 if af > 1.0 else af)
        else:
            af = 0.0
        l = k + 1 if k < m else k

        r0, r1 = dense[i], dense[j]
        lo = r0[k] + af * (r0[l] - r0[k])
        hi = r1[k] + af * (r1[l] - r1[k])
        return lo + mf * (hi - lo)


class LiftTrim:
    """How much lift the vehicle really makes, against what the table says.

    The same shape as ``trajectory.Holdable`` and for the same reason: a
    quantity that describes *this* airframe as it is *being flown* cannot be
    written down in advance, and a constant transcribed from one vehicle is
    the failure this project keeps paying for.  So it is watched.

    Binned by Mach, because that is where the error lives -- lift reads 1.77x
    the table at Mach 6 and 0.57x at Mach 3.2, which is not a scale factor,
    it is a shape.

    Three properties it has to keep:

    - **A bin with no evidence returns ``None``**, never 1.0.  "No
      correction" and "no measurement" are different answers and only one of
      them should make the propagator confident.
    - **It is clamped.**  A ratio outside ``LIFT_TRIM_MIN``/``MAX`` is a bad
      sample -- a reversal mid-slew, a tick where the force read stale -- not
      a discovery about the airframe.
    - **It is smoothed, and it does not extrapolate.**  A bin nobody has
      flown is not improved by projecting a trend into it.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.bins = {}          # mach bin -> (factor, samples)

    def _key(self, mach):
        width = max(0.1, float(self.cfg.LIFT_TRIM_MACH_BIN))
        return int(max(0.0, mach) / width)

    def observe(self, mach, measured_cla, table_cla):
        """One tick's evidence: what was made against what was predicted."""
        if table_cla <= self.cfg.LIFT_TRIM_MIN_CLA or measured_cla <= 0.0:
            return
        ratio = measured_cla / table_cla
        if not (self.cfg.LIFT_TRIM_MIN <= ratio <= self.cfg.LIFT_TRIM_MAX):
            return
        key = self._key(mach)
        factor, samples = self.bins.get(key, (ratio, 0))
        weight = float(self.cfg.LIFT_TRIM_SMOOTHING)
        self.bins[key] = (factor + weight * (ratio - factor), samples + 1)

    def measured(self, mach):
        """What the bin says, whether or not anyone is applying it.

        **Measuring and applying are different decisions.**  ``factor``
        below is gated on ``LIFT_TRIM_ON`` because applying this to
        ``coefficients`` changes every propagation in the flight; but the
        number is taken on every flight regardless and printed at shutdown,
        and a reader who wants the measurement had no way to get it without
        also turning on the correction.  That is how ``airframe.MARGIN``
        came to be a hand-fitted 0.70 sitting next to a live measurement of
        the same quantity reading 0.74.
        """
        entry = self.bins.get(self._key(mach))
        if entry is None or entry[1] < int(self.cfg.LIFT_TRIM_MIN_SAMPLES):
            return None
        return entry[0]

    def report(self):
        """One line for the log: every bin that has an answer."""
        width = max(0.1, float(self.cfg.LIFT_TRIM_MACH_BIN))
        parts = []
        for key in sorted(self.bins):
            factor, samples = self.bins[key]
            if samples >= int(self.cfg.LIFT_TRIM_MIN_SAMPLES):
                parts.append("M%.1f:%.2f/%d" % (key * width, factor, samples))
        return " ".join(parts)


class FlownLift:
    """The lift this vehicle has made, binned by Mach and achieved alpha.

    ``HAC_BANK_FROM_LIFT``'s wing.  Measured in this flight, because what
    sets it -- mass, centre of gravity, the deflection it takes to trim --
    changes with every load, and a polar written down for one load is a
    different aircraft carrying another (the user, 2026-10-10).  The cone's
    flown lift peaks near 12-15 deg at about half the table's and falls
    past it (``tools/conepolar.py``), so the table cannot stand in.

    Binned by Mach as ``LiftTrim`` is, so the glide's last minute answers
    for the cone's entry speed: subsonic-only, nothing had been measured by
    cone entry on any flight of rot-smoke-bfl-1010 and the limit sat at its
    floor exactly where it mattered.  ``peak(mach)`` is the largest alpha
    bin of that Mach bin with ``LIFT_TRIM_MIN_SAMPLES`` behind it: a *lower
    bound* on the wing, which only understates what it can pull.  ``None``
    when that Mach has no such bin.  Samples are filtered as ``LiftTrim``'s
    are and smoothed with ``LIFT_TRIM_SMOOTHING``.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.bins = {}          # (mach bin, alpha bin) -> (cla, samples)

    def _mach_key(self, mach):
        return int(max(0.0, mach)
                   / max(0.1, float(self.cfg.LIFT_TRIM_MACH_BIN)))

    def observe(self, mach, alpha, measured_cla, table_cla):
        cfg = self.cfg
        if alpha < 0.0:
            return
        if table_cla <= cfg.LIFT_TRIM_MIN_CLA or measured_cla <= 0.0:
            return
        if not (cfg.LIFT_TRIM_MIN <= measured_cla / table_cla
                <= cfg.LIFT_TRIM_MAX):
            return
        key = (self._mach_key(mach),
               int(alpha / max(0.5, float(cfg.FLOWN_LIFT_ALPHA_BIN_DEG))))
        cla, samples = self.bins.get(key, (measured_cla, 0))
        weight = float(cfg.LIFT_TRIM_SMOOTHING)
        self.bins[key] = (cla + weight * (measured_cla - cla), samples + 1)

    def peak(self, mach):
        """``(cla, alpha)`` of the best-supported highest bin at this Mach,
        or ``None``."""
        width = max(0.5, float(self.cfg.FLOWN_LIFT_ALPHA_BIN_DEG))
        want = self._mach_key(mach)
        best = None
        for (mkey, akey), (cla, samples) in self.bins.items():
            if mkey != want:
                continue
            if samples < int(self.cfg.LIFT_TRIM_MIN_SAMPLES):
                continue
            if best is None or cla > best[0]:
                best = (cla, (akey + 0.5) * width)
        return best


class Runway:
    """Two thresholds, two headings, and the geometry of an approach to each.

    Both ends are carried because the useful one depends on where the vehicle
    arrives, and arriving from the east to land westward is not a manoeuvre a
    glider with a 3.4 glide ratio can undo.  ``choose`` picks on the heading
    the vehicle is already making good, which is the only thing it cannot
    change cheaply.
    """

    def __init__(self, body, cfg, frame):
        self.cfg = cfg
        self.frame = frame
        self.ends = {}
        for name, lat, lon, heading in (
                ("09", cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON, 90.0),
                ("27", cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON, 270.0)):
            threshold = tuple(body.surface_position(lat, lon, frame))
            self.ends[name] = {
                "name": name,
                "threshold": threshold,
                "heading": heading,
                "radius": vec.norm(threshold) + cfg.RUNWAY_ALT_OFFSET_M,
            }
        # The centreline, as a unit vector at each threshold pointing the way
        # a landing aircraft is travelling.  Taken from the *other* threshold
        # rather than from a compass bearing, so it is the real line between
        # the two points however the body is shaped.
        a = self.ends["09"]["threshold"]
        b = self.ends["27"]["threshold"]
        along = vec.unit(vec.sub(b, a))
        self.ends["09"]["along"] = along
        self.ends["27"]["along"] = vec.scale(along, -1.0)
        self.radius = 0.5 * (self.ends["09"]["radius"]
                             + self.ends["27"]["radius"])
        self.midpoint = vec.scale(vec.unit(vec.add(a, b)), self.radius)

    def horizontal(self, end, direction):
        """``direction`` with the local vertical at that threshold removed."""
        up = vec.unit(end["threshold"])
        return vec.unit(vec.project_out(direction, up))

    def gate_dist(self):
        """How far before the threshold the low gate sits: the derived
        distance when ``GATE_FROM_APPROACH`` has set one, else
        ``GATE_DIST_M``."""
        got = getattr(self, "gate_dist_m", None)
        return self.cfg.GATE_DIST_M if got is None else got

    def low_gate(self, end):
        """The aim point: on the extended centreline, before the threshold.

        The glide is solved to arrive here rather than at the threshold,
        because the last few kilometres are flown geometrically.  A predictor
        that ran all the way to the tarmac would be predicting the flare,
        which it has no model of -- and ``boosterland`` failure 5 is what
        happens when a predicted manoeuvre and the flown one disagree.
        """
        along = self.horizontal(end, end["along"])
        back = vec.scale(along, -self.gate_dist())
        out = vec.add(end["threshold"], back)
        return vec.scale(vec.unit(out), end["radius"] + self.cfg.GATE_ALT_M)

    def high_gate(self, end):
        """A point on the straight-in profile: what the *entry* aims at.

        **Not over the gate**, and that correction is the difference between
        a cone that works and one that cannot.  Aimed at the gate's own
        ground point, the vehicle arrives exactly at the cone's *rollout* --
        and a rollout is the one place on the circle where the turn still to
        fly wraps, so whether the guidance reads 2 degrees or 358 is decided
        by which side of it the vehicle happens to be.  Measured, it read
        342-345 four times in a row, committed to most of an orbit, and ran
        out of height five kilometres short with the runway behind it.

        The entry point is instead where a vehicle on the nominal straight-in
        profile would be at ``HAC_ALT_M``: further out along the extended
        centreline by exactly the height it has to lose, at the glide ratio
        the cone is planned with.  Arriving there, the turn is small and
        unambiguous, the path to the gate matches the height, and the circle
        and the weave are left doing what they are for -- absorbing the
        *errors* in alignment and energy rather than a geometry the entry was
        aimed into.

        It is derived rather than configured, so it cannot drift out of step
        with the gate, the cone's altitude, or the glide ratio it is planned
        at.
        

        The low gate is a point the entry has to arrive at with the right
        energy, and that is the requirement this project has never been able
        to meet -- 14.8 km of along-track scatter, almost none of it made in
        the glide.  The heading alignment cone removes the requirement rather
        than tightening it: arrive *over* the field with height to spare and
        spend the surplus circling, which is what a shuttle does and for the
        same reason.  So the entry is solved to a point two kilometres of
        ground and ten of altitude away from where the landing starts, and
        arriving there long or high is no longer an error to be eliminated
        but the state the next phase is designed to consume.
        """
        along = self.horizontal(end, end["along"])
        # **Not ``HAC_LD``, though it was for a while, and the two are not
        # the same quantity.**  ``HAC_LD`` is *path* per metre of height and
        # is what the cone's energy budget divides by; this is *ground* per
        # metre of height, which is less, because the cone spends much of
        # that path going round rather than along.  Sharing one constant
        # made the cone's budget a range knob: correcting ``HAC_LD`` from
        # 1.35 to its measured 1.70 moved this aim point 3.3 km further out
        # and every arrival with it.  Failure 19's shape -- one number
        # standing in for two things -- caught before it cost a session.
        # ``HAC_AIM_DERIVED``: the ratio the table says a straight-in at
        # the cone's speed flies (``guidance.straight_in_reach``), set once
        # by the autopilot when the table is ready; ``HAC_GATE_LD`` until.
        ratio = getattr(self, "aim_ld", None) or self.cfg.HAC_GATE_LD
        reach = (self.cfg.HAC_ALT_M - self.cfg.GATE_ALT_M) * ratio
        back = vec.scale(along, -(self.gate_dist() + max(0.0, reach)))
        out = vec.add(end["threshold"], back)
        return vec.scale(vec.unit(out), end["radius"] + self.cfg.HAC_ALT_M)

    def gate(self, end):
        """What the entry is solved to arrive at.

        The low gate normally; the high one when the cone is flying, because
        the cone can only be entered where the vehicle can *turn*.  Measured
        on the arrival this vehicle actually makes: it crosses the field at
        19 km still doing Mach 2.9, and a 45 degree turn at Mach 2.9 has a
        radius of 120 km -- there is no cone to fly at that speed.  It falls
        subsonic at about 12-13 km, where the same turn is 9 km across, so
        that is where the entry has to deliver it and that is where the high
        gate is.

        Raising the target also gives back about 19 km of required range
        (12 km of height at the turning glide ratio), which is most of the
        surplus the entry currently arrives with -- so the same change that
        makes the cone enterable also unsaturates the glide solve that feeds
        it.
        """
        return self.high_gate(end)

    def choose(self, position, velocity):
        """Which end to land on, given where the vehicle is and its heading.

        On the heading rather than on the distance: a glider arriving from the
        east cannot turn round to land westward and still reach the runway, so
        the cheap approach is the one already most nearly lined up.
        """
        if not self.cfg.RUNWAY_BOTH_ENDS:
            return self.ends["09"]
        # The *arrival* bearing, not the current heading.  Comparing the
        # vehicle's velocity with a runway direction as plain 3D vectors is
        # meaningless once they are far apart on a sphere: 165 degrees out,
        # local east at the vehicle is nearly opposite local east at the
        # runway, and the first offline entry picked the wrong end of the
        # runway for the whole flight because of it.  The chord from here to
        # the threshold, with the threshold's own vertical removed, is the
        # direction the vehicle will be travelling when it gets there.
        reference = self.ends["09"]["threshold"]
        chord = vec.sub(reference, position)
        track = vec.project_out(chord, vec.unit(reference))
        if vec.norm(track) < 1.0:
            track = vec.project_out(velocity, vec.unit(position))
        if vec.norm(track) < 1.0:
            return self.ends["09"]
        track = vec.unit(track)
        best, best_dot = None, None
        for end in self.ends.values():
            along = self.horizontal(end, end["along"])
            score = vec.dot(track, along)
            if best_dot is None or score > best_dot:
                best, best_dot = end, score
        return best


class Environment:
    def __init__(self, conn, vessel, body, cfg, logbook, ut):
        self.conn = conn
        self.vessel = vessel
        self.body = body
        self.cfg = cfg
        self.logbook = logbook
        self.frame = body.reference_frame
        self.flight = vessel.flight(self.frame)

        self.mu = body.gravitational_parameter
        self.equatorial_radius = body.equatorial_radius
        self.atmosphere_depth = (body.atmosphere_depth
                                 if body.has_atmosphere else 0.0)
        self.omega = self._measure_omega()
        self._alt_step, self._density_table = self._sample_density()
        self._sound_table = self._sample_speed_of_sound()

        self.runway = Runway(body, cfg, self.frame)
        self.target = self.runway.gate(self.runway.ends["09"])
        self.target_radius = self.runway.radius

        # (alpha, Mach) -> Cl*A, Cd*A.  ``None`` means never probed; the
        # interpolation holds the nearest measured value rather than
        # extrapolating a coefficient into a regime nobody has asked about.
        self._alphas = tuple(float(a) for a in cfg.ALPHA_BINS)
        self._machs = tuple(float(m) for m in cfg.MACH_BINS)
        self._probe_alt = list(float(a) for a in cfg.PROBE_ALTITUDES)
        self.lift = Table(self._alphas, self._machs)
        self.drag = Table(self._alphas, self._machs)
        self.lift_trim = LiftTrim(cfg)
        self.flown_lift = FlownLift(cfg)
        self._swept = False
        self._next_row = 0
        self._next_refresh_ut = None
        self.rows_measured = 0
        # The current values, for the log line only; the propagator reads the
        # table.
        self.cla_now = 0.0
        self.cda_now = 0.0

        logbook.event(ut, "env: body=%s mu=%.4g R=%.1f atmo=%.0f "
                          "runway_r=%.1f c0=%.0f omega=(%.3e,%.3e,%.3e)"
                      % (body.name, self.mu, self.equatorial_radius,
                         self.atmosphere_depth, self.target_radius,
                         self.speed_of_sound(0.0), *self.omega))
        for name, end in sorted(self.runway.ends.items()):
            logbook.event(ut, "env: runway %s hdg %.0f threshold=(%.1f,%.1f,"
                              "%.1f) r=%.1f gate=(%.1f,%.1f,%.1f)"
                          % ((name, end["heading"]) + tuple(end["threshold"])
                             + (end["radius"],)
                             + tuple(self.runway.gate(end))))

    # -- the atmosphere ----------------------------------------------------
    def _sample_density(self):
        """The whole density profile, once, into a table.

        A propagation is hundreds of steps and a kRPC call per step would be
        far too slow, so the profile is sampled at startup and interpolated
        locally.  Same reason as boosterland; same consequence, that the
        integration loop makes no remote calls at all.
        """
        if self.atmosphere_depth <= 0.0:
            return 1.0, [0.0]
        step = float(self.cfg.DENSITY_TABLE_STEP_M)
        count = int(self.atmosphere_depth / step) + 2
        return step, [self.body.density_at(min(i * step,
                                               self.atmosphere_depth))
                      for i in range(count)]

    def _sample_speed_of_sound(self):
        """``c = sqrt(1.4 P / rho)`` -- no gas constant, no temperature model.

        Both ``pressure_at`` and ``density_at`` are available, and their ratio
        is all the speed of sound needs.  The Mach axis the whole table is
        binned on comes from this, so a body that will not answer only makes
        the bins the wrong width rather than the physics wrong.
        """
        if self.atmosphere_depth <= 0.0:
            return [self.cfg.SOUND_SPEED_FALLBACK_M_S]
        out = []
        for i in range(len(self._density_table)):
            altitude = min(i * self._alt_step, self.atmosphere_depth)
            rho = self._density_table[i]
            try:
                pressure = self.body.pressure_at(altitude)
            except Exception:                           # noqa: BLE001
                pressure = 0.0
            if rho > 1e-12 and pressure > 0.0:
                out.append(math.sqrt(1.4 * pressure / rho))
            else:
                out.append(out[-1] if out
                           else self.cfg.SOUND_SPEED_FALLBACK_M_S)
        return out

    def _interpolate(self, table, altitude):
        if len(table) == 1:
            return table[0]
        if altitude <= 0.0:
            return table[0]
        index = altitude / self._alt_step
        low = int(index)
        if low >= len(table) - 1:
            return table[-1]
        fraction = index - low
        return table[low] + fraction * (table[low + 1] - table[low])

    def density(self, altitude):
        if altitude >= self.atmosphere_depth:
            return 0.0
        return max(0.0, self._interpolate(self._density_table, altitude))

    def speed_of_sound(self, altitude):
        return max(1.0, self._interpolate(self._sound_table, altitude))

    def mach(self, speed, altitude):
        return speed / self.speed_of_sound(altitude)

    # -- the table ---------------------------------------------------------
    def coefficients(self, alpha_deg, speed, altitude):
        """``(Cl*A, Cd*A)`` -- the pair the propagator applies.

        Returns ``(0.0, 0.0)`` before anything has been measured, which makes
        an unprobed vehicle fly a vacuum arc rather than a fictional one.  A
        table nobody has filled in is not a model.

        **Trimmed by what the vehicle is actually making.**  The swept table
        is a probe of the airframe as it sits; the vehicle in the air is
        holding an attitude with its control surfaces deflected, and
        ``simulate_aerodynamic_force_at`` does not see that deflection.
        Measured against flight (``aeroaudit.py`` on LOG1722): drag agrees to
        within 1%, and lift reads **1.77x** the table hypersonically and
        **0.57x** at Mach 3.2.  A model that wrong about lift cannot predict
        range, and the glide spent 7 km of its miss in exactly the band where
        the ratio is worst.  ``LiftTrim`` learns the ratio per Mach from the
        force the game reports and applies it here, at the single point the
        propagator and the control loop both read.
        """
        if not self.drag.filled():
            return 0.0, 0.0
        mach = speed / self.speed_of_sound(altitude)
        cla = self.lift.lookup(alpha_deg, mach)
        cda = self.drag.lookup(alpha_deg, mach)
        return cla, cda

    def ready(self):
        return self.drag.filled()

    # -- probing -----------------------------------------------------------
    def _axes(self):
        """The vehicle's nose and dorsal, in the body frame, right now.

        kRPC documents its vessel frame as x-right, y-forward,
        z-out-of-the-bottom, and that is *checked* against ``Vessel.direction``
        rather than trusted: the whole table is expressed relative to these
        axes, so getting them wrong would not produce a wrong number, it would
        produce a plausible table for a different aircraft.
        """
        try:
            nose = vec.unit(self.vessel.direction(self.frame))
            forward = vec.unit(self.conn.space_center.transform_direction(
                (0.0, 1.0, 0.0), self.vessel.reference_frame, self.frame))
            if vec.angle_between(forward, nose) > 5.0:
                return None, None
            dorsal = vec.scale(vec.unit(
                self.conn.space_center.transform_direction(
                    (0.0, 0.0, 1.0), self.vessel.reference_frame,
                    self.frame)), -1.0)
            return nose, dorsal
        except Exception:                               # noqa: BLE001
            return None, None

    def _probe_row(self, index, nose, dorsal, rotation):
        """Probe every angle of attack at one Mach, and fold it in.

        One position and one attitude answers the whole row: the probe takes
        the velocity to evaluate at, so the speed and the angle are both
        inputs and dividing by ``q`` removes the air.  That is what makes a
        two-dimensional table affordable at all -- a row is fourteen calls and
        the whole table is one second of them.
        """
        altitude = self._probe_alt[index]
        mach = self._machs[index]
        speed = max(self.cfg.PROBE_SPEED_FLOOR,
                    mach * self.speed_of_sound(altitude))
        rho = self.density(altitude)
        if rho <= 1e-12:
            return False
        q = 0.5 * rho * speed * speed
        if q <= 0.0:
            return False
        up = vec.unit(self.vessel.position(self.frame))
        position = vec.scale(up, self.equatorial_radius + altitude)
        weight = self.cfg.AERO_SMOOTHING
        got = False
        directions = []
        for alpha_deg in self._alphas:
            alpha = math.radians(alpha_deg)
            # Positive angle of attack: the wind arrives from ahead and below,
            # so the velocity is the nose rotated away from the dorsal.  Lift
            # then acts along the dorsal.
            directions.append(vec.unit(vec.sub(
                vec.scale(nose, math.cos(alpha)),
                vec.scale(dorsal, math.sin(alpha)))))
        # The whole row in one round trip (``Config.RPC_BATCH``).
        forces = krpcbatch.ask(self.conn, [
            (self.flight.simulate_aerodynamic_force_at,
             (self.body, tuple(position),
              tuple(vec.scale(direction, speed)), tuple(rotation)))
            for direction in directions],
            batched=getattr(self.cfg, "RPC_BATCH", False))
        for column, (direction, force) in enumerate(zip(directions, forces)):
            if force is None:
                continue
            flow = direction
            # Drag along the flow; lift is everything across it, as a
            # magnitude.  The direction is a control input -- the bank angle
            # rolls it wherever the guidance wants -- not a property of the
            # airframe, so what the table owes the propagator is how *much*
            # lift an angle of attack makes.
            cda = -vec.dot(force, flow) / q
            cla = vec.norm(vec.project_out(force, flow)) / q
            if not (0.0 <= cda < 1e4) or not (0.0 <= cla < 1e4):
                continue
            self.drag.set(index, column, cda, weight)
            self.lift.set(index, column, cla, weight)
            got = True
        if got:
            self.rows_measured += 1
        return got

    def dump_table(self, ut):
        """Write the swept table into the log, once, whole.

        **A measurement whose only consumer is the propagator cannot be
        argued with.**  The table has always carried every alpha bin out to
        90 degrees -- the high-alpha entry question has been measured on this
        airframe every flight of the project -- and nothing ever printed it,
        so the only cells anyone could see were the two the vehicle happened
        to be flying (``cla=``/``cda=`` on the telemetry line).  That is the
        shape of failure 13 again: a number nothing reads back is a number
        nothing can contradict.

        One line per Mach row, ``alpha:ClA/CdA``, plus the altitude the row
        was probed at, because the same Mach reads 15-20% lower down low and
        a row means nothing without it.  Thirteen lines a flight, in STANDBY,
        where nothing is happening.
        """
        self.logbook.event(ut, "aero table: alpha:ClA/CdA per Mach row, "
                               "probed at the altitude each Mach is flown at")
        for index, mach in enumerate(self._machs):
            cells = []
            for column, alpha in enumerate(self._alphas):
                cla = self.lift.raw[index][column]
                cda = self.drag.raw[index][column]
                if cla is None or cda is None:
                    cells.append("%g:-" % alpha)
                else:
                    cells.append("%g:%.1f/%.1f" % (alpha, cla, cda))
            self.logbook.event(ut, "aero M=%.1f alt=%.0f %s"
                               % (mach, self._probe_alt[index],
                                  " ".join(cells)))
        # And the summary the high-alpha question actually wants: where the
        # drag peaks, and what the broadside bin is worth against the angle
        # the entry flies today.  A reader should not have to do this by eye
        # across thirteen lines.
        for index, mach in enumerate(self._machs):
            row = self.drag.raw[index]
            have = [(c, v) for c, v in enumerate(row) if v is not None]
            if not have:
                continue
            peak_column, peak = max(have, key=lambda cv: cv[1])
            self.logbook.event(
                ut, "aero M=%.1f peak CdA %.1f at %g deg | %s"
                % (mach, peak, self._alphas[peak_column],
                   " ".join("%g deg CdA %.1f ClA %s"
                            % (self._alphas[c], row[c],
                               "-" if self.lift.raw[index][c] is None
                               else "%.1f" % self.lift.raw[index][c])
                            for c in range(len(self._alphas))
                            if self._alphas[c] in (30.0, 65.0, 90.0)
                            and row[c] is not None)))

    def dump_torque(self, ut):
        """What the vehicle has to hold an attitude *with*.

        The other half of the holdability question, and the half kRPC will
        actually answer: it reports no aerodynamic pitching moment at any
        price, but it does report the torque available from the reaction
        wheels and from RCS.  Neither is worth anything without the other --
        "four RCS blocks are not enough" is an assertion until the number is
        in the log next to the dynamic pressure the vehicle failed at.

        Defensive because these three properties are not in ``docs/krpc.md``
        and their shapes vary across server versions: whatever answers, gets
        logged; whatever does not, is skipped.
        """
        for name in ("available_torque", "available_reaction_wheel_torque",
                     "available_rcs_torque", "available_engine_torque"):
            try:
                value = getattr(self.vessel, name)
            except Exception:                           # noqa: BLE001
                continue
            try:
                positive = value[0]
                text = "(%.0f, %.0f, %.0f) N m" % tuple(positive)
            except Exception:                           # noqa: BLE001
                text = str(value)
            self.logbook.event(ut, "torque %s = %s" % (name, text))

    def sweep(self, ut, full=False):
        """Refresh the table: the current Mach's row, and one more in turn.

        The first sweep takes every row, because the first prediction needs a
        whole table and there is a quiet vacuum coast in which to pay for it.
        After that it is two rows a second, which keeps the table tracking the
        altitude each Mach is actually being flown at.
        """
        nose, dorsal = self._axes()
        if nose is None:
            return False
        try:
            rotation = tuple(self.vessel.rotation(self.frame))
        except Exception:                               # noqa: BLE001
            return False
        if full or not self._swept:
            any_row = False
            for index in range(len(self._machs)):
                any_row = self._probe_row(index, nose, dorsal,
                                          rotation) or any_row
            if any_row:
                self._swept = True
                self.logbook.event(ut, "aero: swept %d of %d Mach rows"
                                   % (self.rows_measured, len(self._machs)))
                if self.cfg.AERO_DUMP:
                    self.dump_table(ut)
                    self.dump_torque(ut)
            return any_row
        rows = max(1, int(self.cfg.AERO_ROWS_PER_REFRESH))
        for _ in range(rows):
            self._probe_row(self._next_row, nose, dorsal, rotation)
            self._next_row = (self._next_row + 1) % len(self._machs)
        return True

    def refresh(self, ut, speed=0.0, altitude=0.0):
        """Time-gated wrapper, and the log's current-value bookkeeping."""
        if (self._next_refresh_ut is not None
                and ut < self._next_refresh_ut):
            return
        self._next_refresh_ut = ut + self.cfg.AERO_REFRESH_UT
        self.sweep(ut)

    def set_profile(self, profile):
        """Re-aim each Mach row at the altitude the descent will use it at.

        ``profile`` is the ``(speed, altitude)`` trail of a prediction.  The
        circularity is real -- the altitudes come from a prediction made with
        the table they are about to change -- and the answer is the same one
        boosterland arrived at: take it repeatedly and let the two converge.
        """
        if not profile:
            return
        for index, mach in enumerate(self._machs):
            wanted = mach * self.speed_of_sound(self._probe_alt[index])
            best, best_error = None, None
            for speed, altitude in profile:
                error = abs(speed - wanted)
                if best_error is None or error < best_error:
                    best, best_error = altitude, error
            if best is not None and best_error is not None:
                # Move rather than jump: the profile is noisy early on and a
                # row that hops altitude every tick never settles.
                self._probe_alt[index] += 0.35 * (best - self._probe_alt[index])

    # -- the rotating frame ------------------------------------------------
    def _measure_omega(self):
        """Rotation vector of the body frame, measured rather than assumed.

        A point fixed in the rotating frame at radius ``r`` moves at ``w x r``
        in the inertial frame, so comparing the vessel's velocity in both
        frames pins magnitude and sign without reasoning about kRPC's
        left-handed axes.  Straight from ``boosterland``, where replacing it
        with a hard-coded ``(0, rotational_speed, 0)`` is explicitly warned
        against.
        """
        sc = self.conn.space_center
        inertial = self.body.non_rotating_reference_frame
        r = tuple(self.vessel.position(self.frame))
        v_rot = tuple(self.vessel.velocity(self.frame))
        v_inertial = sc.transform_direction(
            tuple(self.vessel.velocity(inertial)), inertial, self.frame)
        measured = vec.sub(v_inertial, v_rot)           # == w x r
        speed = self.body.rotational_speed
        best, best_err = (0.0, speed, 0.0), None
        for candidate in ((0.0, speed, 0.0), (0.0, -speed, 0.0)):
            err = vec.norm(vec.sub(vec.cross(candidate, r), measured))
            if best_err is None or err < best_err:
                best, best_err = candidate, err
        return best
