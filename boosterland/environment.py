"""Slowly-changing world data, sampled once so the predictor can run offline.

Trajectory propagation would be unusably slow if every integration step made a
remote kRPC call, so the expensive things are cached here:

* the atmospheric density profile, sampled once at startup into a table that
  :meth:`Environment.density` interpolates locally, and alongside it the local
  speed of sound, so the propagator can talk in Mach numbers;
* an effective drag area (Cd*A) **as a curve against Mach**, probed with
  ``Flight.simulate_aerodynamic_force_at`` and refreshed a bin at a time at
  most once per ``AERO_REFRESH_UT`` in-game seconds;
* the rotation vector of the body's (rotating) reference frame, measured from
  the game rather than assumed, so the propagator's Coriolis and centrifugal
  terms carry the right sign in kRPC's left-handed frames.
"""

import bisect
import math

from . import vec


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
        self.atmosphere_depth = body.atmosphere_depth if body.has_atmosphere else 0.0

        # Aim at the pad, plus whatever bias has been measured for this
        # vehicle.  The booster's miss is not random: over three flights from
        # one quicksave it landed 105, 106 and 102 m west and 16-17 m north of
        # the pad -- a spread of about 2 m around a systematic 105 m.  Nothing
        # in the guidance can null that (BOOSTBACK_TOLERANCE_M is quantised by
        # how fast the predicted miss falls: 175 and 190 stop on the *same*
        # tick, 205 stops 3.3 s earlier and lands 290 m the other side), but a
        # bias that repeatable can simply be aimed off.  Measure it, put it
        # here, and the systematic part cancels; what is left is the 2 m of
        # spread.  It is calibration for one vehicle on one profile, not a
        # better algorithm -- a different booster needs its own number.
        lat = cfg.PAD_LAT + math.degrees(cfg.AIM_BIAS_NORTH_M
                                         / self.equatorial_radius)
        lon = cfg.PAD_LON + math.degrees(
            cfg.AIM_BIAS_EAST_M
            / (self.equatorial_radius * math.cos(math.radians(cfg.PAD_LAT))))
        self.target = tuple(body.surface_position(lat, lon, self.frame))
        self.pad = tuple(body.surface_position(cfg.PAD_LAT, cfg.PAD_LON,
                                               self.frame))
        self.target_radius = vec.norm(self.target) + cfg.PAD_ALT_OFFSET_M

        self.omega = self._measure_omega()
        self._alt_step, self._density_table = self._sample_density()
        self._sound_table = self._sample_speed_of_sound()

        # Cd*A against Mach.  ``drag_area`` is only the value at the Mach the
        # booster is doing right now -- the log column and nothing else; the
        # propagator reads the curve.
        self.drag_area = 0.0
        self._bins = max(2, int(cfg.DRAG_CURVE_BINS))
        self._bin_width = cfg.DRAG_MACH_MAX / self._bins
        self._curve = [None] * self._bins
        self._clean = [False] * self._bins   # bin measured at a descent attitude
        self._samples = [0] * self._bins     # how far past warm-up each bin is
        self._rotation_warned = False
        self.diag_probe = None          # (mach, real, descent, aoa)
        self._curve_machs = []          # sorted, filled bins only
        self._curve_areas = []
        self._curve_swept = False
        self._reswept = False           # curve re-taken at a descent attitude
        self.reswept_ut = None          # ... and when, for the CORRECTION gate
        self._last_sweep_ut = None      # ... and when that last happened
        self._next_curve_bin = 0
        self._next_aero_ut = None
        self._descent_profile = ()      # (speed, altitude) down to the pad

        # Cl*A per radian squared against Mach: same bins, same smoothing, a
        # different component of the same probe.  It starts empty and stays
        # empty until something measures it -- there is no fallback slope,
        # because an unmeasured steering gain is a guess with an unknown sign.
        self._lift = [None] * self._bins
        self._lift_machs = []
        self._lift_areas = []
        self.lift_slope = None          # the value at the current Mach, logged

        logbook.event(ut, "env: body=%s mu=%.4g R=%.1f atmo=%.0f pad_r=%.1f "
                           "c0=%.0f omega=(%.3e,%.3e,%.3e)"
                      % (body.name, self.mu, self.equatorial_radius,
                         self.atmosphere_depth, self.target_radius,
                         self.speed_of_sound(0.0),
                         self.omega[0], self.omega[1], self.omega[2]))

        if cfg.DIAG_STATE:
            # Everything replay.py needs to rebuild this world offline.  The
            # tables are measured from the body once, so they belong in the
            # log next to the flight they describe rather than being
            # re-derived from a guess at which body it was.
            logbook.event(ut, "diag env target=%.1f,%.1f,%.1f "
                              "pad=%.1f,%.1f,%.1f target_radius=%.3f "
                              "mu=%.6e R=%.1f atmo=%.1f alt_step=%.4f "
                              "omega=%.6e,%.6e,%.6e"
                          % (self.target + self.pad
                             + (self.target_radius, self.mu,
                                self.equatorial_radius, self.atmosphere_depth,
                                self._alt_step) + tuple(self.omega)))
            logbook.event(ut, "diag rho=[%s]"
                          % " ".join("%.6g" % x for x in self._density_table))
            logbook.event(ut, "diag sound=[%s]"
                          % " ".join("%.4g" % x for x in self._sound_table))

    # -- atmosphere --------------------------------------------------------
    def _sample_density(self):
        if self.atmosphere_depth <= 0.0:
            return 1.0, [0.0]
        n = max(2, int(self.cfg.DENSITY_SAMPLE_COUNT))
        step = self.atmosphere_depth / (n - 1)
        table = [self.body.density_at(i * step) for i in range(n)]
        return step, table

    def _sample_speed_of_sound(self):
        """Local speed of sound, on the same altitude grid as the density.

        ``c = sqrt(gamma * P / rho)`` needs no gas constant and no temperature
        model, and both ``pressure_at`` and ``density_at`` are body-level calls
        that can be sampled once.  Older servers may not have ``pressure_at``;
        a constant is a poor atmosphere but a fine Mach axis, since all it has
        to do is keep the drag curve's bins lined up with the transonic hump.
        """
        fallback = self.cfg.SOUND_SPEED_FALLBACK_M_S
        table = []
        for i in range(len(self._density_table)):
            altitude = i * self._alt_step
            c = fallback
            try:
                pressure = self.body.pressure_at(altitude)
                rho = self._density_table[i]
                if pressure > 0.0 and rho > 0.0:
                    c = math.sqrt(1.4 * pressure / rho)
            except Exception:               # noqa: BLE001 -- old server
                pass
            if not math.isfinite(c) or c <= 1.0:
                c = fallback
            table.append(c)
        return table or [fallback]

    def _interpolate(self, table, altitude):
        if len(table) < 2:
            return table[0]
        if altitude <= 0.0:
            return table[0]
        x = altitude / self._alt_step
        i = int(x)
        if i >= len(table) - 1:
            return table[-1]
        f = x - i
        return table[i] * (1.0 - f) + table[i + 1] * f

    def density(self, altitude):
        """Atmospheric density (kg/m^3) at a mean altitude, interpolated."""
        if len(self._density_table) < 2 or altitude >= self.atmosphere_depth:
            return 0.0
        return self._interpolate(self._density_table, altitude)

    def speed_of_sound(self, altitude):
        """Local speed of sound (m/s); clamped to the top of the table."""
        return self._interpolate(self._sound_table, altitude)

    def mach(self, speed, altitude):
        c = self.speed_of_sound(altitude)
        return speed / c if c > 0.0 else 0.0

    # -- the drag curve ----------------------------------------------------
    def drag_area_at(self, speed, altitude):
        """Cd*A at this speed and altitude, interpolated over the Mach curve.

        A single scalar Cd*A is not good enough to aim a boostback burn with.
        Cd has a transonic hump -- LOG15 measured 15 m^2 subsonic, 50 m^2 near
        Mach 1.2 and 20 m^2 at Mach 2.5 on the same vehicle -- and the
        propagator runs the booster through all of it in one prediction.  Held
        at the value probed at the current Mach, the propagation is wrong in
        whichever direction the vehicle is about to move: boostback ended
        LOG15 sitting on the transonic peak (49 m^2) and predicted the whole
        supersonic descent with more than twice the drag it would really see,
        so the prediction landed short of the truth, and boostback stopped
        burning with the real touchdown point well beyond the pad.  The
        booster flew over the pad at 9 km.

        Outside the probed range the end values are held: extrapolating a Cd
        curve is guesswork, and the ends are where the samples are sparsest.
        """
        if not self._curve_machs:
            return self.drag_area
        return self._interp_curve(self._curve_machs, self._curve_areas,
                                  self.mach(speed, altitude))

    @staticmethod
    def _interp_curve(machs, values, mach):
        """Interpolate a filled-bin curve, holding the end values."""
        if not machs:
            return 0.0
        if mach <= machs[0]:
            return values[0]
        if mach >= machs[-1]:
            return values[-1]
        i = bisect.bisect_left(machs, mach)
        lo, hi = machs[i - 1], machs[i]
        if hi <= lo:
            return values[i]
        f = (mach - lo) / (hi - lo)
        return values[i - 1] * (1.0 - f) + values[i] * f

    def curve_snapshot(self):
        """The filled bins as ``[(mach, area), ...]`` -- diagnostics only.

        A prediction cannot be re-flown offline from a log that records only
        the Cd*A at the Mach the booster happened to be doing, because the
        propagator reads the whole curve.  ``DIAG_STATE`` logs this next to
        the state vector so ``replay.py`` can re-propagate a real tick with
        the air that tick actually saw.
        """
        return list(zip(self._curve_machs, self._curve_areas))

    def _bin_index(self, mach):
        return int(vec.clamp(mach / self._bin_width, 0, self._bins - 1))

    def _bin_mach(self, index):
        """Speed to probe this bin at: its centre."""
        return (index + 0.5) * self._bin_width

    def _record(self, index, sample, clean=True):
        """Blend a fresh sample into one bin and rebuild the lookup arrays.

        Single samples are noisy -- the sim is evaluated at whatever
        orientation the booster currently holds, which swings while it is
        flipping -- and a spike moves the predicted touchdown by kilometres.
        Clamp the outliers against what this bin already said, then low-pass
        what is left.  Each bin smooths independently, so a real change with
        Mach is not mistaken for noise the way it was when one scalar had to
        track the whole curve.

        ``clean`` says the probe was taken near the attitude the descent is
        actually flown at; see :meth:`sample_is_clean`.  A dirty sample is
        only ever *provisional*: it fills a bin that has nothing in it, so
        the first prediction still has a curve to read, and it is thrown away
        outright by the first clean sample rather than being averaged with
        it.  Averaging is what made the recovery slow enough to matter --
        a quarter-weight EMA probed round-robin every ~12 s takes the better
        part of a minute to walk a bin down by a third, which is most of the
        coast.
        """
        current = self._curve[index]
        if not clean and self._clean[index]:
            return                          # never pollute a measured bin
        if current is None or current <= 0.0 or (clean and not self._clean[index]):
            self._curve[index] = sample     # empty, or first real measurement
            self._samples[index] = 1
        else:
            # Smoothing that a bin has to *earn*.  The clamp and the EMA are
            # both defences against a single spike, and against a settled bin
            # they are right -- but they apply just as hard to a bin whose
            # only content is one junk reading, and then they are what stops
            # it recovering.  The opening sweep is taken wherever the booster
            # happens to be at startup, which is high, thin air at Mach 3,
            # and it can be out by an order of magnitude: LOG60's Mach 1.38
            # bin opened at 2.1 m^2 against a real 23, and crawled 2.1 -> 3.3
            # -> 4.2 -> 5.2 -> 6.5 -> 17.6 -> 23 over ninety seconds, which
            # is a minute past the boostback exit that needed it.  Averaging
            # the first few samples with equal weight gets a bin to the right
            # order of magnitude at once and hands over to the EMA after
            # DRAG_WARMUP_SAMPLES, so a spike into a settled bin is still
            # clamped and low-passed exactly as before.
            seen = self._samples[index]
            warmup = max(1, int(self.cfg.DRAG_WARMUP_SAMPLES))
            if seen < warmup:
                weight = 1.0 / (seen + 1.0)
            else:
                ratio = self.cfg.DRAG_OUTLIER_RATIO
                sample = vec.clamp(sample, current / ratio, current * ratio)
                weight = self.cfg.DRAG_SMOOTHING
            self._curve[index] = (1.0 - weight) * current + weight * sample
            self._samples[index] = seen + 1
        if clean:
            self._clean[index] = True
        self._curve_machs = [self._bin_mach(i)
                             for i, a in enumerate(self._curve) if a is not None]
        self._curve_areas = [a for a in self._curve if a is not None]

    def set_descent_profile(self, profile):
        """Remember where the predicted descent is when it is going how fast.

        Supplied by ``trajectory.predict_landing``; see :meth:`probe_altitude`
        for what it is for.  Stored rather than passed because the probe runs
        from ``refresh_drag``, which is deliberately not on the propagator's
        call path.
        """
        self._descent_profile = profile or ()

    def probe_altitude(self, speed, altitude):
        """The altitude to evaluate a probe of this speed at.

        Cd*A is **not** a function of Mach alone.  KSP multiplies drag by a
        pseudo-Reynolds term -- a curve in density times speed -- on top of
        the Mach curve, so the same bin reads differently at 30 km and at
        3 km.  Measured on LOG1, with the vehicle on retrograde throughout so
        attitude is not the variable: Mach 0.62 goes 15.0 -> 11.9 between
        29 km and the ground, Mach 0.88 20.3 -> 17.0, Mach 1.12 51.8 -> 42.5.
        That is 16-21%, all in the same direction, and it is exactly the
        residual walk left over once the descent-attitude re-sweep has taken
        the broadside error out: LOG1's predicted miss holds near 10 m at
        13 km and drifts to 194 m by 1.6 km with the vehicle doing nothing.

        A Mach-indexed table cannot represent that, and widening it to two
        dimensions would multiply the probing.  But the probe is free to ask
        about any position it likes, and the propagator has just worked out
        where the booster will be when it is doing each speed -- so ask about
        *that* air.  The bin is then measured in the conditions it is going to
        be used in, and the one number stored stays the right one.

        Falls back to the vehicle's current altitude when the profile does not
        cover this speed, which is the old behaviour and is what happens
        before the first prediction and above the top of the descent.
        """
        if not self.cfg.DRAG_PROBE_DESCENT_ALTITUDE:
            return altitude
        profile = self._descent_profile
        if not profile:
            return altitude
        # Speed is *not* monotone in altitude: the booster accelerates as it
        # falls and then decelerates again, so it passes most speeds twice --
        # once thin and once thick.  Drag goes as the density, so the deep
        # pass is where nearly all of the bin's work is done and the thin one
        # barely matters; take the lowest altitude among the close matches
        # rather than the arithmetically nearest sample.
        near = [sa for sa in profile
                if abs(sa[0] - speed) <= 0.1 * max(speed, 1.0)]
        if near:
            return min(near, key=lambda sa: sa[1])[1]
        best = min(profile, key=lambda sa: abs(sa[0] - speed))
        if abs(best[0] - speed) > 0.5 * max(speed, 1.0):
            return altitude             # nothing close: do not invent air
        return best[1]

    def probe_position(self, position, altitude):
        """``position`` moved radially to ``altitude`` -- same place, different air."""
        radius = vec.norm(position)
        if radius < 1e-6:
            return tuple(position)
        return vec.scale(position, (self.equatorial_radius + altitude) / radius)

    def probe_rotation(self, rotation, velocity, nose):
        """The attitude to evaluate the aerodynamic probe at.

        Returns ``(rotation, clean)``.

        ``sample_is_clean`` refuses samples taken at the wrong attitude, which
        keeps the curve honest but cannot help the decision that most needs
        it: boostback's exit is taken *during* boostback, before any bin has
        ever been probed at a descent attitude, so it still reads a curve
        written broadside.  In LOG53 that is the whole 620 m -- boostback
        stopped on a prediction of 773 m from the pad that was really 113 m,
        and only the aim bias made up the difference.

        So rather than wait for the vehicle to fly the attitude, turn it: the
        probe is asked about the *same vessel* rotated along the shortest arc
        that puts its nose on retrograde, which is where the nose sits all the
        way down.  ``simulate_aerodynamic_force_at`` takes the attitude to
        evaluate at, so this costs nothing extra -- it is the same call with a
        different quaternion, and the whole curve is still probed from one
        position.

        This is not ``DRAG_PROBE_AXIAL``, which measured worse in game and is
        still off by default.  That one kept the real attitude and moved the
        *airflow* onto the vessel's axis, which is a zero angle of attack in
        whichever direction the booster happened to be pointing -- nose-first
        during the coast, and this booster falls engines-first behind its grid
        fins.  It read 10.8-14.2 m^2 where the vehicle really had 17.4-18.8,
        and the flight landed 3.3 km short.  Here the airflow stays exactly
        where it really is and the *vessel* is turned to meet it the way the
        descent will.

        ``vec.rotation_onto`` returns ``None`` if it cannot verify its own
        answer -- kRPC's frames are left-handed and the quaternion convention
        is not something to assume -- and then this falls back to the real
        attitude and the sample is judged by ``sample_is_clean`` as before.
        """
        if not self.cfg.DRAG_PROBE_DESCENT or nose is None:
            return tuple(rotation), self.sample_is_clean(velocity, nose)
        if vec.norm(velocity) < 1e-6:
            return tuple(rotation), self.sample_is_clean(velocity, nose)
        retrograde = vec.scale(vec.unit(velocity), -1.0)
        turned = vec.rotation_onto(rotation, nose, retrograde)
        if turned is None:
            if not self._rotation_warned:
                self._rotation_warned = True
                self.logbook.event(
                    0.0, "descent-attitude probe unavailable: could not verify "
                         "a rotation onto retrograde; using the real attitude")
            return tuple(rotation), self.sample_is_clean(velocity, nose)
        return turned, True

    def in_descent_configuration(self, velocity, nose):
        """Is the vehicle sitting the way it will sit all the way down?

        The angle is measured from retrograde, because that is where the nose
        goes on the way down -- the booster falls engines-first and the nose
        tracks anti-velocity.  This is deliberately a separate question from
        :meth:`sample_is_clean`: that one decides whether to *keep* a sample
        and defaults to keeping all of them, while this one decides when the
        whole curve is worth taking again.
        """
        limit = self.cfg.DRAG_RESWEEP_AOA_DEG
        if limit <= 0.0 or nose is None:
            return False
        speed = vec.norm(velocity)
        axis = vec.norm(nose)
        if speed < 1e-6 or axis < 1e-6:
            return False
        cos = -vec.dot(nose, velocity) / (axis * speed)
        return cos >= math.cos(math.radians(limit))

    def sample_is_clean(self, velocity, nose):
        """Is the vehicle pointed near the attitude the descent is flown at?

        ``simulate_aerodynamic_force_at`` evaluates the vessel at the attitude
        it is given, so a probe taken during BOOSTBACK measures the booster
        *broadside* to a steep airstream -- and the propagation it feeds is of
        a descent flown engines-first, at no angle of attack at all.

        This is what LOG53 cost, measured against the path the booster
        actually flew.  Propagating its coast with the curve as it stood at
        the boostback exit misses the real position by 1297 m after 99 s;
        with the curve as it stood at touchdown, by 6 m.  The difference is
        three bins -- the transonic ones the descent falls through, Mach
        1.12/1.38/1.62 -- reading 1.36/1.32/1.35 times too high at the exit,
        every other bin being within 10%.  Watch one of them across the
        flight and the mechanism is plain: Mach 1.12 climbs 19.9 -> 55.8
        through boostback, peaking on the exit tick, then decays to 41.6 once
        the coast holds a real descent attitude.

        Over-stated drag lands the prediction short, and boostback stops
        burning when the prediction reaches the pad, so the vehicle flies
        past: LOG53's prediction said 773 m from the pad at the exit and the
        booster landed 113 m from it, having been aimed 628 m off to absorb
        exactly that.

        The angle is measured from *retrograde*, because that is where the
        nose sits on the way down -- the engines are what faces the airstream.
        ``DRAG_PROBE_MAX_AOA_DEG`` at 0 disables the gate and takes every
        sample, which is the old behaviour.
        """
        limit = self.cfg.DRAG_PROBE_MAX_AOA_DEG
        if limit <= 0.0 or nose is None:
            return True
        speed = vec.norm(velocity)
        axis = vec.norm(nose)
        if speed < 1e-6 or axis < 1e-6:
            return True
        cos = -vec.dot(nose, velocity) / (axis * speed)
        return cos >= math.cos(math.radians(limit))

    def _probe_force(self, position, velocity, rotation):
        """One aerodynamic simulation call, as the force vector it returns."""
        try:
            return self.flight.simulate_aerodynamic_force_at(
                self.body, tuple(position), tuple(velocity), tuple(rotation))
        except Exception:                   # noqa: BLE001 -- KSP-side failure
            return None

    def _probe(self, position, velocity_direction, speed, altitude, rotation):
        """One aerodynamic simulation call, reduced to a Cd*A."""
        rho = self.density(altitude)
        if rho < 1e-9 or speed < 20.0:
            return None
        velocity = vec.scale(velocity_direction, speed)
        force = self._probe_force(position, velocity, rotation)
        if force is None:
            return None
        drag = -vec.dot(force, velocity_direction)     # component opposing v
        if drag <= 0.0:
            return None
        return drag / (0.5 * rho * speed * speed)

    def probe_lift_slope(self, position, velocity, rotation, nose):
        """Measure how much sideforce an angle of attack is worth, per radian.

        The same call that measures Cd*A returns the *whole* force, and
        everything perpendicular to the airflow has been thrown away since
        this file was written.  That perpendicular part is the only actuator
        in the descent that can move the landing point in both directions
        without an engine relight: a booster held a few degrees off retrograde
        with its grid fins out is a wing, and it flies for a minute through
        air thick enough to decelerate it at more than a g.

        The probe turns the *vessel* to an attitude ``AERO_STEER_PROBE_AOA_DEG``
        off retrograde, in the plane the steering is flown in, and leaves the
        airflow exactly where it really is -- the same construction
        ``probe_rotation`` uses, and for the same reason (see
        ``DRAG_PROBE_AXIAL``, which did it the other way round and flew the
        booster into the sea).

        The answer is reduced to ``Cl*A per radian squared`` along
        ``+cross(n, v)``, the same basis ``trajectory.Steer`` applies it in,
        **with its sign as measured**.  Squared, because that is what the
        vehicle measured: probing this booster at 2, 5, 8, 12 and 20 degrees
        gives sideforces whose *linear* slope varies eightfold (-5.2 to
        -40.2) while ``force / (q a|a|)`` holds at -149, -147, -143, -136,
        -115.  A cylinder's crossflow force goes as sin^2 of the angle of
        attack, and a linear gain fitted to it is wrong at both ends of its
        own range.  Whether tilting the nose one way pushes the vehicle that
        way or the other is a question about the vehicle and the frame's
        handedness, and this does not guess at it: the sign comes back in the
        number, exactly as ``_measure_omega`` does it for the rotation vector.

        Returns ``None`` when the air, the speed or the geometry will not
        support a measurement -- there is no fallback value, because a lift
        slope nobody measured is a steering gain nobody measured.
        """
        speed = vec.norm(velocity)
        altitude = vec.norm(position) - self.equatorial_radius
        rho = self.density(altitude)
        if rho < 1e-9 or speed < 20.0 or nose is None:
            return None
        normal = vec.cross(position, velocity)
        if vec.norm(normal) < 1e-6:
            return None
        normal = vec.unit(normal)
        side = vec.unit(vec.cross(normal, velocity))
        retrograde = vec.scale(vec.unit(velocity), -1.0)
        aoa = math.radians(self.cfg.AERO_STEER_PROBE_AOA_DEG)
        if aoa <= 0.0:
            return None
        tilted = vec.unit(vec.add(vec.scale(retrograde, math.cos(aoa)),
                                  vec.scale(side, math.sin(aoa))))
        turned = vec.rotation_onto(rotation, nose, tilted)
        if turned is None:
            return None
        force = self._probe_force(position, velocity, turned)
        if force is None:
            return None
        q = 0.5 * rho * speed * speed
        return vec.dot(force, side) / (q * aoa * abs(aoa))

    def lift_slope_at(self, speed, altitude):
        """Cl*A per radian squared at this Mach, interpolated like the drag."""
        return self._interp_curve(self._lift_machs, self._lift_areas,
                                  self.mach(speed, altitude))

    def refresh_lift(self, position, velocity, rotation, nose):
        """Keep the lift curve current: one extra probe per refresh."""
        if not self.cfg.AERO_STEER:
            return
        sample = self.probe_lift_slope(position, velocity, rotation, nose)
        if sample is None:
            return
        speed = vec.norm(velocity)
        altitude = vec.norm(position) - self.equatorial_radius
        index = self._bin_index(self.mach(speed, altitude))
        previous = self._lift[index]
        if previous is None:
            self._lift[index] = sample
        else:
            w = self.cfg.DRAG_SMOOTHING
            self._lift[index] = previous * (1.0 - w) + sample * w
        self._lift_machs, self._lift_areas = [], []
        for i, value in enumerate(self._lift):
            if value is not None:
                self._lift_machs.append(self._bin_mach(i))
                self._lift_areas.append(value)
        self.lift_slope = self._lift[index]

    def _probe_bin(self, position, direction, index, altitude, c, rotation):
        """Probe one Mach bin, in the air that bin is going to be used in.

        ``c`` is the speed of sound where the booster is; the bin's speed is
        derived from it so the Mach axis stays the one the propagator reads.
        The *position* then moves to wherever the descent passes that speed,
        and the density used for the reduction moves with it -- both have to,
        or the probe divides a force measured in one atmosphere by the density
        of another.
        """
        speed = self._bin_mach(index) * c
        at = self.probe_altitude(speed, altitude)
        if at == altitude:
            return self._probe(position, direction, speed, altitude, rotation)
        return self._probe(self.probe_position(position, at), direction,
                           speed, at, rotation)

    def probe_direction(self, velocity, nose):
        """Which way the airflow should come from, for a probe.

        ``simulate_aerodynamic_force_at`` evaluates the vessel at the attitude
        it is given, so a probe taken with the real velocity measures Cd*A *at
        the angle of attack the booster currently happens to hold*.  That is
        not a property of the vehicle, and it is not the number the propagator
        wants: the descent is flown nose-retrograde, at no angle of attack at
        all.  Boostback holds the booster broadside to a steep airstream for a
        minute, and every bin probed during it -- the transonic bins included,
        because that is where boostback ends -- was being written with a
        side-on area and then used to fly a nose-on descent.

        Two real flights from the *same* quicksave measured the same curve
        shape 20% apart at every Mach (LOG4 against LOG6), which is worth
        about a kilometre at the pad: over-stated drag lands the prediction
        short, so boostback burns too long and the booster flies past.  LOG6
        crossed 41 m from the pad at 8.3 km and sailed on to 1220 m.

        Taking the airflow along the vessel's own axis instead -- a zero
        angle of attack probe, independent of what the booster is doing when
        the sample is taken -- is what this exists to do, and it **measured
        worse in game, so it defaults off**.

        LOG11 flew it: boostback ended clean at ``miss=145``, the coast then
        opened to 1272 m by 2926 and 3823 m by the landing burn, and the
        booster fell 3.3 km short into the sea.  Against a 137/135/145 m
        baseline over three flights from the same quicksave.  The axial probe
        read ``cda`` 10.8-14.2 where the same vehicle at the same Mach read
        17.4-18.8 with the ordinary probe: it under-states the drag, so the
        propagation flies the booster further than it goes, boostback stops
        short of the burn it needed, and the real vehicle lands short.

        The reasoning was half right and that is the useful part.  Probing at
        the current attitude *is* wrong during boostback, which holds the
        vehicle broadside for a minute.  But it is *right* during the coast,
        which is most of the flight and all of the part being predicted --
        the booster really is descending retrograde by then.  And zero angle
        of attack is not what it descends at either: the grid fins are out,
        the coast attitude carries a deliberate bias, and retrograde is only
        ever approximately held.  The honest fix for the boostback samples is
        to reject them, not to replace every sample with a fiction; that is
        untried and is the next thing to measure.

        Set ``DRAG_PROBE_AXIAL`` True to fly it anyway.
        """
        if nose is not None and self.cfg.DRAG_PROBE_AXIAL:
            axis = vec.unit(nose)
            if vec.norm(axis) > 0.5:            # a real direction, not (0,0,0)
                return vec.scale(axis, -1.0)    # flow comes at the nose
        return vec.unit(velocity)

    def refresh_drag(self, ut, position, velocity, rotation, nose=None):
        """Keep the Mach curve current, at a bounded cost in remote calls.

        The first refresh that finds usable air sweeps the whole curve, so the
        very first prediction already knows the shape rather than a point.
        After that each refresh probes two bins: the one the booster is in --
        which is the one the next few seconds of flight depend on -- and one
        more round-robin, so the rest of the curve keeps up with altitude and
        attitude for two kRPC calls a second.

        ``nose`` is the vessel's forward direction; see :meth:`probe_direction`
        for why the probe does not use the real velocity direction.
        """
        if self._next_aero_ut is not None and ut < self._next_aero_ut:
            return self.drag_area
        self._next_aero_ut = ut + self.cfg.AERO_REFRESH_UT

        speed = vec.norm(velocity)
        altitude = vec.norm(position) - self.equatorial_radius
        if speed < 20.0 or self.density(altitude) < 1e-6:
            return self.drag_area
        direction = self.probe_direction(velocity, nose)
        c = self.speed_of_sound(altitude)
        rotation, clean = self.probe_rotation(rotation, velocity, nose)

        descent = self.in_descent_configuration(velocity, nose)
        # Once is not enough, and the reason is circular.  The descent-altitude
        # probe asks where the booster will be when it is doing each speed,
        # and that comes from a prediction made with the curve as it stands --
        # so the first re-sweep is taken at altitudes derived from the very
        # curve it is replacing.  Re-taking it periodically lets the two
        # converge on each other instead of leaving the answer wherever the
        # first attempt landed.  It also beats waiting for the round-robin,
        # which reaches a given bin about every twelve seconds and then only
        # moves it by one EMA step.
        due = (self.cfg.DRAG_RESWEEP_INTERVAL_S > 0.0 and self._reswept
               and (self._last_sweep_ut is None
                    or ut - self._last_sweep_ut >= self.cfg.DRAG_RESWEEP_INTERVAL_S))
        repeat = descent and due
        resweeping = descent and not self._reswept and self._curve_swept
        if resweeping:
            # Probe first, replace second.  Throwing the curve away and then
            # finding the air will not answer would leave the propagator with
            # no curve at all, on a booster that is already committed -- and
            # the old curve, broadside or not, is a far better answer than a
            # fallback scalar.  So the replacement only happens if there is
            # something to replace it with.
            fresh = [self._probe_bin(position, direction, i, altitude, c,
                                     rotation)
                     for i in range(self._bins)]
            if not any(x is not None for x in fresh):
                resweeping = False
        if not self._curve_swept or resweeping or repeat:
            before = self.curve_snapshot()
            self._last_sweep_ut = ut
            if resweeping:
                # A *replacement*, not a blend.  Every bin in the curve was
                # written broadside during boostback, and the clamp and the
                # EMA that protect a settled bin from a spike are exactly what
                # would stop it recovering -- LOG53's three transonic bins read
                # 1.32-1.36 times high at the boostback exit and took most of
                # the coast to walk down under round-robin probing, which is a
                # minute after the decision that needed them.  So they are
                # thrown away outright the first tick the vehicle is actually
                # in the configuration the descent is flown in.
                self._curve = [None] * self._bins
                self._samples = [0] * self._bins
                self._curve_machs, self._curve_areas = [], []
            for index in range(self._bins):
                sample = (fresh[index] if resweeping
                          else self._probe_bin(position, direction, index,
                                               altitude, c, rotation))
                # `resweeping` replaced the curve outright because every bin
                # in it was known to be broadside.  A `repeat` sweep is a
                # refresh of bins that are already honest, so it goes through
                # the clamp and the EMA like any other sample.
                if sample is not None:
                    self._record(index, sample, clean)
            # A sweep taken broadside still counts as a sweep: the curve has
            # to hold *something* from the first prediction, and every bin of
            # it is provisional until a descent-attitude probe replaces it.
            self._curve_swept = bool(self._curve_machs)
            if resweeping and self._curve_swept:
                self._reswept = True
                self.reswept_ut = ut
                was = dict(before)
                self.logbook.event(
                    ut, "drag curve re-swept at descent attitude (aoa %.0f deg): "
                        "%s" % (vec.angle_between(vec.scale(nose, -1.0), velocity),
                                " ".join("M%.2f %.1f->%.1f" % (m, was[m], a)
                                         for m, a in self.curve_snapshot()
                                         if m in was)))
        else:
            here = self._bin_index(self.mach(speed, altitude))
            for index in (here, self._next_curve_bin):
                sample = self._probe_bin(position, direction, index, altitude,
                                         c, rotation)
                if sample is not None:
                    self._record(index, sample, clean)
            self._next_curve_bin = (self._next_curve_bin + 1) % self._bins

        if self.cfg.DIAG_STATE and nose is not None:
            # Probe the bin the booster is in at *both* attitudes.  During the
            # coast the vessel really is on retrograde, so the two must agree;
            # that is what says the synthesised quaternion is the attitude it
            # claims to be, rather than a plausible-looking wrong one.  During
            # boostback they should differ, and by how much is the size of the
            # error this exists to remove.
            here = self._bin_index(self.mach(speed, altitude))
            at = self._bin_mach(here) * c
            real = self._probe(position, direction, at, altitude, rotation)
            turned = vec.rotation_onto(rotation, nose,
                                       vec.scale(vec.unit(velocity), -1.0))
            other = (self._probe(position, direction, at, altitude, turned)
                     if turned is not None else None)
            self.diag_probe = (self._bin_mach(here), real, other,
                               vec.angle_between(vec.scale(nose, -1.0),
                                                 velocity))

        self.drag_area = self.drag_area_at(speed, altitude)
        return self.drag_area

    # -- rotating frame ----------------------------------------------------
    def _measure_omega(self):
        """Rotation vector of ``body.reference_frame``, measured in-game.

        A point fixed in the rotating frame at radius ``r`` moves at ``w x r``
        as seen from the inertial frame, so comparing the vessel's velocity in
        both frames pins down both magnitude and sign without having to reason
        about kRPC's left-handed axes.
        """
        sc = self.conn.space_center
        inertial = self.body.non_rotating_reference_frame
        r = tuple(self.vessel.position(self.frame))
        v_rot = tuple(self.vessel.velocity(self.frame))
        v_inertial = sc.transform_direction(
            tuple(self.vessel.velocity(inertial)), inertial, self.frame)
        measured = vec.sub(v_inertial, v_rot)          # == w x r

        speed = self.body.rotational_speed
        best, best_err = (0.0, speed, 0.0), None
        for candidate in ((0.0, speed, 0.0), (0.0, -speed, 0.0)):
            err = vec.norm(vec.sub(vec.cross(candidate, r), measured))
            if best_err is None or err < best_err:
                best, best_err = candidate, err
        # If neither sign fits (vessel on the axis, or a stationary body),
        # the +y guess is harmless: |w| is tiny compared to everything else.
        return best
