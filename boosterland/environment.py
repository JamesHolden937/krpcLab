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

from common import vec


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
        # in the guidance can null that (the boostback tolerance is quantised by
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
        self.diag_probe = None          # (mach, real, descent, aoa)
        self._curve_machs = []          # sorted, filled bins only
        self._curve_areas = []
        self._curve_swept = False
        self._reswept = False           # curve re-taken at a descent attitude
        self.reswept_ut = None          # ... and when, for the CORRECTION gate
        self._last_sweep_ut = None      # ... and when that last happened
        self._next_curve_bin = 0
        self._next_aero_ut = None
        # The curve at full airbrake, taken once (DEPLOY_LAW), and the
        # altitude below which the propagator flies it: the brakes are going
        # to open there, so every prediction has to know it.  None = no switch.
        self._full_machs, self._full_areas = [], []
        # The landing burn's curve (set_burn_curve): swept with the fins
        # held neutral, as the burn flies them.
        self._burn_machs, self._burn_areas = [], []
        # ... and its normal force: Cn*A per radian at the reference angle,
        # per Mach, and the shape in angle ((alpha, lift factor, drag
        # factor), set_burn_lift).
        self._lift_machs, self._lift_slopes = [], []
        self._lift_shape = []
        self._full_lift = None          # (machs, slopes, shape), brakes open
        self.late_switch_alt = None
        self._replace_next = False      # the next sweep replaces, not blends
        self._descent_profile = ()      # (speed, altitude) down to the pad

        # Cl*A per radian squared against Mach: same bins, same smoothing, a
        # different component of the same probe.  It starts empty and stays
        # empty until something measures it -- there is no fallback slope,
        # because an unmeasured steering gain is a guess with an unknown sign.
        self._lift = [None] * self._bins
        self._lift_machs = []
        self._lift_areas = []
        self.lift_slope = None          # the value at the current Mach, logged
        self.leg_clearance = 0.0        # COM to vessel bottom, m (telemetry)
        self.tip_angle_deg = None       # the legs' static tip-over angle
        self.lateral_tolerance = None   # m/s sideways the legs survive
        self.descent_aoa = None         # rad, the coast's last command
        self.pointing_band = None       # deg, the pointing error's noise
        self.touchdown_tolerance = None  # m/s the craft survives at contact
                                        # (Autoland: weakest touching part,
                                        # the legs' suspension when lower)
        self.unsteered = None           # the coast's zero-angle prediction
        self.miss_band = None           # m, the prediction's noise (Autoland)
        self.taper_rate = None          # 1/s, held from ignition
        self.engine_spool_s = None      # trigger to thrust (Autoland.
                                        # track_spool, the first ignition)
        self.control_interval = None    # game-s between control ticks
        self.attitude_rate = None       # 1/s the lateral loops may close at
                                        # (Autoland.null_rate), for the
                                        # burn's law and its propagation
        self.burn_isp = None            # s, the engines' Isp as last read:
                                        # the burn propagation's mass flow
        self.unwind_in = None           # s to the predicted ignition, and
        self.unwind_span = None         # the s the coast's angle fades over
                                        # before it (Autoland.set_unwind)
        self.burn_lit = False           # the landing burn's engines are
                                        # delivering (no reaction to wait)
        self.nose_lag = None            # s, the nose behind its command
        self.burn_dry_mass = None       # kg, the vessel with its engines'
                                        # propellant gone (Autoland.
                                        # read_dry_mass)
        self.nose = None                # where the nose points now
        # The fraction of a commanded angle of attack the vehicle actually
        # holds; see ``record_tracking``.  1 until measured.
        self.aoa_tracking = 1.0
        self.tracking_samples = 0
        # The restoring moment per unit of dynamic pressure at each probe
        # angle, per Mach bin; see ``refresh_holdable``.
        self._hold = [None] * self._bins
        # The angle of most lift, per Mach bin: the wing's ceiling.
        self._lift_peak = [None] * self._bins
        self._hold_fixed = 0.0          # N m: wheels + RCS, the most seen
        self._hold_rcs = 0.0
        self._hold_surface = 0.0        # N m per Pa: control surfaces
        self._next_hold_ut = None
        self.holdable_deg = None        # at the current state, logged

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
        if (self.late_switch_alt is not None and self._full_machs
                and altitude <= self.late_switch_alt):
            return self._interp_curve(self._full_machs, self._full_areas,
                                      self.mach(speed, altitude))
        if not self._curve_machs:
            return self.drag_area
        return self._interp_curve(self._curve_machs, self._curve_areas,
                                  self.mach(speed, altitude))

    def sweep_bins(self, position, velocity, rotation, nose=None):
        """Probe every Mach bin as the vessel is now, recording nothing.

        Returns one Cd*A per bin, None where the probe would not answer.
        """
        altitude = vec.norm(position) - self.equatorial_radius
        direction = self.probe_direction(velocity, nose)
        c = self.speed_of_sound(altitude)
        rotation, _ = self.probe_rotation(rotation, velocity, nose, position)
        return [self._probe_bin(position, direction, i, altitude, c, rotation)
                for i in range(self._bins)]

    def set_full_curve(self, bins):
        """The full-brake curve, from :meth:`sweep_bins`; False if empty."""
        pairs = [(self._bin_mach(i), a) for i, a in enumerate(bins)
                 if a is not None and a > 0.0]
        self._full_machs = [m for m, _ in pairs]
        self._full_areas = [a for _, a in pairs]
        return bool(pairs)

    def set_burn_curve(self, bins):
        """The landing burn's curve, from :meth:`sweep_bins` taken with the
        fins held neutral (``Autoland.sweep_burn_curve``); False if empty."""
        pairs = [(self._bin_mach(i), a) for i, a in enumerate(bins)
                 if a is not None and a > 0.0]
        if not pairs:
            return False
        self._burn_machs = [m for m, _ in pairs]
        self._burn_areas = [a for _, a in pairs]
        return True

    def has_burn_curve(self):
        return bool(self._burn_machs)

    def set_burn_lift(self, slopes, shape, full=False):
        """The burn's normal force: ``slopes`` one Cn*A per radian per Mach
        bin (None where unread), at the shape's first angle; ``shape``
        ``[(alpha, lift factor, drag factor), ...]`` from one ladder, the
        lift factor normalised to the first angle's slope and the drag
        factor to the drag on the airflow.  False if either is empty."""
        pairs = [(self._bin_mach(i), k) for i, k in enumerate(slopes)
                 if k is not None]
        if not pairs or not shape:
            return False
        table = ([m for m, _ in pairs], [k for _, k in pairs], sorted(shape))
        if full:
            self._full_lift = table
        else:
            self._lift_machs, self._lift_slopes, self._lift_shape = table
        return True

    def sweep_burn_lift(self, position, velocity, rotation, nose, angles):
        """Probe the normal force for :meth:`set_burn_lift`, as the vessel
        is now: ``(slopes, shape)``, or None if the probe would not answer.

        ``angles`` (radians, ascending) are the nose's tilts off retrograde
        in the vertical plane; the shape is read here at each, the per-Mach
        slope at the first in each bin's air (``probe_altitude``)."""
        speed = vec.norm(velocity)
        altitude = vec.norm(position) - self.equatorial_radius
        rho = self.density(altitude)
        if speed < 20.0 or rho <= 0.0 or not angles:
            return None
        retro = vec.scale(velocity, -1.0 / speed)
        side = vec.project_out(vec.unit(position), retro)
        if vec.norm(side) < 1e-6:
            return None
        side = vec.unit(side)

        def tilted(a):
            return vec.rotation_onto(rotation, nose, vec.add(
                vec.scale(retro, math.cos(a)), vec.scale(side, math.sin(a))))

        def coefficients(pos, vel, rot, q):
            f = self._probe_force(pos, vel, rot) if rot is not None else None
            if f is None:
                return None
            return vec.dot(f, retro) / q, -vec.dot(f, side) / q

        q = 0.5 * rho * speed * speed
        zero = coefficients(position, velocity, tilted(0.0), q)
        ladder = [coefficients(position, velocity, tilted(a), q)
                  for a in angles]
        if zero is None or zero[0] <= 0.0 or ladder[0] is None \
                or ladder[0][1] == 0.0:
            return None
        ref = ladder[0][1] / angles[0]
        shape = [(a, c[1] / (ref * a), c[0] / zero[0])
                 for a, c in zip(angles, ladder) if c is not None]
        rot = tilted(angles[0])
        c = self.speed_of_sound(altitude)
        slopes = []
        for i in range(self._bins):
            s = self._bin_mach(i) * c
            at = self.probe_altitude(s, altitude)
            rho_at = self.density(at)
            out = (coefficients(self.probe_position(position, at),
                                vec.scale(retro, -s), rot,
                                0.5 * rho_at * s * s)
                   if rho_at > 0.0 else None)
            slopes.append(out[1] / angles[0] if out is not None else None)
        return slopes, shape

    def burn_aero(self, speed, altitude, alpha):
        """``(Cn*A, drag factor)`` with the nose ``alpha`` radians off the
        airflow in the burn, or None before set_burn_lift.

        Cn*A acts opposite the nose's tilt: a booster falling engine-first
        with its nose held behind retrograde is pushed forward and up.  It
        is not small: at M2 with the fins neutral the fin booster makes
        1.1 m^2 of it per degree beside 3.1 m^2 of drag (lad-1117), so the
        0.3-0.7 deg the nose lags its command through the burn, and the few
        degrees it lights with, take a fifth to all of the drag's
        horizontal braking away.  Below the first angle the slope is held,
        and the drag factor runs linearly from 1 on the airflow; past the
        last, both are held."""
        machs, slopes, shape = (self._lift_machs, self._lift_slopes,
                                self._lift_shape)
        if (self.late_switch_alt is not None and self._full_machs
                and altitude <= self.late_switch_alt):
            if self._full_lift is None:
                return None
            machs, slopes, shape = self._full_lift
        if not machs or not shape:
            return None
        slope = self._interp_curve(machs, slopes, self.mach(speed, altitude))
        a0, n0, d0 = shape[0]
        if alpha <= a0:
            n_f = n0
            d_f = 1.0 + (d0 - 1.0) * (alpha / a0 if a0 > 0.0 else 1.0)
        else:
            n_f, d_f = shape[-1][1], shape[-1][2]
            for (a1, n1, d1), (a2, n2, d2) in zip(shape, shape[1:]):
                if alpha <= a2:
                    f = (alpha - a1) / (a2 - a1) if a2 > a1 else 1.0
                    n_f = n1 + (n2 - n1) * f
                    d_f = d1 + (d2 - d1) * f
                    break
        return slope * alpha * n_f, d_f

    def burn_drag_area_at(self, speed, altitude):
        """Cd*A in the landing burn: the burn's own curve where the brakes
        are as it was swept, else :meth:`drag_area_at`.

        The coast holds its angle of attack on the fins, pinned at full
        deflection most of the way down, and a probe evaluates the fins as
        they sit: the coast's curve reads 1.3-2.5 times the drag of the same
        booster with the fins neutral (drg-1117: M2.4 8.3 -> 3.1 m^2 within
        a second of holding them).  The burn steers on the gimbals with the
        fins near neutral, and flown on the coast's curve its prediction
        braked harder than the booster does."""
        if (self.late_switch_alt is not None and self._full_machs
                and altitude <= self.late_switch_alt):
            return self.drag_area_at(speed, altitude)
        if not self._burn_machs:
            return self.drag_area_at(speed, altitude)
        return self._interp_curve(self._burn_machs, self._burn_areas,
                                  self.mach(speed, altitude))

    def full_curve_ratio(self):
        """Median full-brake / closed Cd*A over the bins both hold: the
        burn's curve where there is one (both fins-neutral), else the
        current curve."""
        full = dict(zip(self._full_machs, self._full_areas))
        machs, areas = ((self._burn_machs, self._burn_areas)
                        if self._burn_machs
                        else (self._curve_machs, self._curve_areas))
        ratios = sorted(full[m] / a for m, a in zip(machs, areas)
                        if m in full and a > 0.0)
        return ratios[len(ratios) // 2] if ratios else None

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

    def probe_rotation(self, rotation, velocity, nose, position=None):
        """The attitude to evaluate the aerodynamic probe at: the real one.

        Returns ``(rotation, clean)``, ``clean`` from ``sample_is_clean``.
        Probing the vessel turned onto retrograde instead was refuted twice:
        pd10-1016 (3 of 4 heavies lost) and keep1/gate1-1111 (median ~200 v
        22 on the fin booster, even with the brake fins kept deployed).

        Away from the descent's attitude the vessel is evaluated at it
        instead: retrograde tilted by the angle the coast last commanded
        (``descent_aoa``, radians; 0 before the coast, which is the
        unsteered descent every coast solve starts from) in the trajectory
        plane.  The broadside curve read the boostback-exit trajectory ~1 km
        long of the descent re-sweep.
        """
        tilt = math.degrees(self.descent_aoa or 0.0)
        if (nose is None or vec.norm(velocity) < 1e-6
                or self.in_descent_configuration(velocity, nose)):
            return tuple(rotation), self.sample_is_clean(velocity, nose)
        retrograde = vec.scale(vec.unit(velocity), -1.0)
        up = (vec.project_out(vec.unit(position), retrograde)
              if position is not None else None)
        if up is None or vec.norm(up) < 1e-6:
            return tuple(rotation), self.sample_is_clean(velocity, nose)
        a = math.radians(tilt)
        target = vec.unit(vec.add(vec.scale(retrograde, math.cos(a)),
                                  vec.scale(vec.unit(up), math.sin(a))))
        turned = vec.rotation_onto(rotation, nose, target)
        if turned is None:
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
        # Within the commanded angle plus the pointing noise (measured by
        # the autopilot's valve; unmeasured, never).
        band = self.pointing_band
        if band is None or nose is None:
            return False
        limit = math.degrees(self.descent_aoa or 0.0) + band
        speed = vec.norm(velocity)
        axis = vec.norm(nose)
        if speed < 1e-6 or axis < 1e-6:
            return False
        cos = -vec.dot(nose, velocity) / (axis * speed)
        return cos >= math.cos(math.radians(limit)) - 1e-9   # float slack

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

        Every sample is taken: an angle gate on them was measured and lost.
        The probe attitude is chosen in ``probe_rotation`` instead.
        """
        return True

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

        The probe turns the *vessel* to the attitude the coast commands
        off retrograde, in the plane the steering is flown in, and leaves the
        airflow exactly where it really is (moving the airflow instead, see
        ``probe_direction``, flew the booster into the sea).

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
        # At the angle the coast is commanding, so the coefficient is the
        # one the solve flies (it falls ~20% from 2 to 20 deg); never below
        # the pointing noise, the least angle the nose holds distinctly.
        # Neither measured yet: no probe.
        if self.pointing_band is None:
            return None
        # A degree at least: the probe's sideforce goes as the angle
        # squared, and much under that it is lost in the float noise of the
        # drag it rides on (a perfect nose read 3.5 for 12, testFlightSim).
        aoa = max(self.descent_aoa or 0.0, math.radians(self.pointing_band),
                  math.radians(1.0))
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
        """Cl*A per radian squared of *commanded* angle, at this Mach.

        The probe measures the lift of the vehicle held at an angle; the
        propagator is asked about the lift of a *command*.  Those are the same
        only on a vehicle that holds what it is told, and a booster with six
        airbrakes at 90 degrees on its trailing end is a weathervane: told 10
        degrees it held 2-3 (LOG10236 live, LOG10237 farm), and since the
        force goes as the angle squared the prediction credited it with ten
        times the lift it made.  The miss walked 800 m short through the last
        6 km with the solve pinned at its limit, and nothing could see it
        coming.  So the curve is scaled by the measured tracking squared:
        lift of what the vehicle *will* hold when given this command.
        """
        slope = self._interp_curve(self._lift_machs, self._lift_areas,
                                   self.mach(speed, altitude))
        return slope * self.aoa_tracking * self.aoa_tracking

    def lift_slope_held(self, speed, altitude):
        """Cl*A per radian squared of angle *held*, at this Mach: the probed
        curve itself, for a caller that already has the real angle of attack
        (``lift_slope_at`` discounts a command by the measured tracking)."""
        return self._interp_curve(self._lift_machs, self._lift_areas,
                                  self.mach(speed, altitude))

    def record_tracking(self, commanded_deg, held_deg):
        """Fold one (commanded, held) pair into ``aoa_tracking``.

        The caller only passes pairs taken under a steady command bigger
        than the pointing noise -- small commands are dominated by the
        autopilot's own deadband, and a command that just changed has not had
        time to be held.  Clamped to ``[AERO_STEER_TRACK_FLOOR, 1]``: holding
        more than commanded is an oscillation, not authority.
        """
        cfg = self.cfg
        sample = vec.clamp(held_deg / commanded_deg,
                           cfg.AERO_STEER_TRACK_FLOOR, 1.0)
        if self.tracking_samples == 0:
            self.aoa_tracking = sample
        else:
            w = cfg.AERO_STEER_TRACK_SMOOTHING
            self.aoa_tracking = self.aoa_tracking * (1.0 - w) + sample * w
        self.tracking_samples += 1

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

    def _probe_torque(self, position, velocity, rotation):
        try:
            return self.flight.simulate_aerodynamic_torque_at(
                self.body, tuple(position), tuple(velocity), tuple(rotation),
                (0.0, 0.0, 0.0))
        except Exception:                   # noqa: BLE001 -- KSP-side failure
            return None

    def _control_torque(self, name):
        """The weaker tilt axis (pitch or yaw, either sign) of one source."""
        try:
            pos, neg = getattr(self.vessel, name)
        except Exception:                   # noqa: BLE001 -- KSP-side failure
            return None
        value = min(abs(pos[0]), abs(pos[2]), abs(neg[0]), abs(neg[2]))
        if name == "available_rcs_torque":
            # RCS reads 0 while the valve is shut -- and on the tick it
            # opens -- but it is still there to be used.  Kept from every
            # read (the flip's, every tick), not only the probe's: a valve
            # shut through a steady boostback and reopened at the coast
            # made the first probe's budget wheels-only, holdable 12-15 deg
            # against 42-46 (pace1-1113).
            self._hold_rcs = max(self._hold_rcs, value)
        return value

    def refresh_holdable(self, ut, position, velocity, rotation, nose):
        """Measure what the booster can hold off retrograde, before asking.

        A booster with its airbrakes deployed on the trailing end is a
        weathervane: the air turns the nose back onto retrograde with a moment
        that grows with the angle, and the attitude controller can only push
        back with what its wheels, RCS and control surfaces supply.  Where the
        two meet is the most it will hold -- 2-3 deg on the heavy booster
        against a 10 deg command (LOG10236) -- and ``tracking`` found that out
        only after flying it.  This asks the game first: the vessel as it is
        now (airbrakes, CoM, propellant) turned to a few angles off
        retrograde, the aerodynamic torque about its CoM at each, reduced to
        the restoring component per pascal of dynamic pressure.
        """
        cfg = self.cfg
        if not cfg.AERO_STEER or nose is None:
            return
        if self._next_hold_ut is not None and ut < self._next_hold_ut:
            return
        self._next_hold_ut = ut + cfg.AERO_STEER_HOLD_PROBE_S
        speed = vec.norm(velocity)
        altitude = vec.norm(position) - self.equatorial_radius
        rho = self.density(altitude)
        if rho < 1e-9 or speed < 20.0:
            return
        q = 0.5 * rho * speed * speed
        normal = vec.cross(position, velocity)
        if vec.norm(normal) < 1e-6:
            return
        side = vec.unit(vec.cross(vec.unit(normal), velocity))
        retrograde = vec.scale(vec.unit(velocity), -1.0)
        top = math.pi / 2.0
        n = max(1, int(cfg.AERO_STEER_HOLD_PROBES))
        curve = []
        lifts = []
        polar = []
        nose_first = vec.rotation_onto(rotation, nose, retrograde)
        if nose_first is None:
            return
        force = self._probe_force(position, velocity, nose_first)
        if force is None:
            return
        polar.append((0.0, 0.0, -vec.dot(force, vec.unit(velocity)) / q))
        for i in range(1, n + 1):
            aoa = top * i / n
            tilted = vec.unit(vec.add(vec.scale(retrograde, math.cos(aoa)),
                                      vec.scale(side, math.sin(aoa))))
            turned = vec.rotation_onto(rotation, nose, tilted)
            if turned is None:
                return
            torque = self._probe_torque(position, velocity, turned)
            if torque is None:
                return
            # tau along cross(t, retrograde) turns t toward retrograde: the
            # identity (t x F) x t ~ F holds in either handedness.
            axis = vec.cross(tilted, retrograde)
            if vec.norm(axis) < 1e-9:
                return
            curve.append((aoa, vec.dot(torque, vec.unit(axis)) / q))
            force = self._probe_force(position, velocity, turned)
            if force is None:
                return
            lifts.append((aoa, vec.dot(force, side) / q))
            polar.append((aoa, lifts[-1][1],
                          -vec.dot(force, vec.unit(velocity)) / q))
        wheels = self._control_torque("available_reaction_wheel_torque")
        rcs = self._control_torque("available_rcs_torque")
        surface = self._control_torque("available_control_surface_torque")
        if wheels is None or rcs is None or surface is None:
            return
        # RCS reads 0 while the valve is shut; it is still there to be used.
        self._hold_rcs = max(self._hold_rcs, rcs)
        self._hold_fixed = wheels + self._hold_rcs
        self._hold_surface = surface / q
        index = self._bin_index(self.mach(speed, altitude))
        self._hold[index] = curve
        first = self._lift_peak[index] is None
        self._lift_peak[index] = lift_peak(lifts)
        if first and self._lift_peak[index] is not None:
            # aoa:lift/drag, both per pascal (m^2), drag along -v.
            self.logbook.event(ut, "lift peak at M%.2f: %.1f deg (%s)"
                               % (self.mach(speed, altitude),
                                  math.degrees(self._lift_peak[index]),
                                  " ".join("%.0f:%.0f/%.0f"
                                           % (math.degrees(a), l, d)
                                           for a, l, d in polar)))
            limit = self.torque_limit_aoa(speed, altitude)
            self.logbook.event(ut, "torque limit at M%.2f: %s deg (moment/q "
                               "%s; budget/q %.0f)"
                               % (self.mach(speed, altitude),
                                  "%.1f" % math.degrees(limit)
                                  if limit is not None else "-",
                                  " ".join("%.0f:%.0f" % (math.degrees(a), m)
                                           for a, m in curve),
                                  cfg.AERO_STEER_HOLD_MARGIN
                                  * (self._hold_fixed / q
                                     + self._hold_surface)))
        held = self.holdable_aoa(speed, altitude)
        self.holdable_deg = math.degrees(held) if held is not None else None
        if cfg.DIAG_STATE:
            self.logbook.event(ut, "diag hold mach=%.2f q=%.0f fixed=%.0f "
                               "surf=%.0f curve=[%s] -> %s deg"
                               % (self.mach(speed, altitude), q,
                                  self._hold_fixed, surface,
                                  " ".join("%.1f:%.0f" % (math.degrees(a), m)
                                           for a, m in curve),
                                  "%.1f" % self.holdable_deg
                                  if held is not None else "-"))

    def holdable_aoa(self, speed, altitude):
        """Radians the booster can hold off retrograde here, or None.

        The nearest probed Mach bin's moment curve, scaled by this point's
        dynamic pressure, against ``AERO_STEER_HOLD_MARGIN`` of the control
        torque at the same q.  None until something has been probed -- an
        unmeasured limit must not read as a measured one.
        """
        filled = [i for i, c in enumerate(self._hold) if c]
        if not filled:
            return None
        here = self._bin_index(self.mach(speed, altitude))
        curve = self._hold[min(filled, key=lambda i: abs(i - here))]
        rho = self.density(altitude)
        q = 0.5 * rho * speed * speed
        peaks = [i for i, p in enumerate(self._lift_peak) if p is not None]
        if not peaks:
            return None
        top = self._lift_peak[min(peaks, key=lambda i: abs(i - here))]
        curve = [(a, m) for a, m in curve if a <= top]
        if q <= 0.0:
            return top
        budget = self.cfg.AERO_STEER_HOLD_MARGIN * (
            self._hold_fixed + self._hold_surface * q) / q
        last_a, last_m = 0.0, 0.0
        for aoa, moment in curve:
            if moment > budget:
                # Linear between the last angle that held and this one.
                if moment <= last_m:
                    return last_a
                return last_a + (aoa - last_a) * (budget - last_m) / (
                    moment - last_m)
            last_a, last_m = aoa, max(last_m, moment)
        return top

    def _nearest(self, table, speed, altitude):
        filled = [i for i, c in enumerate(table) if c]
        if not filled:
            return None
        here = self._bin_index(self.mach(speed, altitude))
        return table[min(filled, key=lambda i: abs(i - here))]

    def torque_limit_aoa(self, speed, altitude):
        """Radians the controls can hold off retrograde here, or None.

        ``holdable_aoa`` without its cap at the lift peak: the angle where
        the aerodynamic moment meets ``AERO_STEER_HOLD_MARGIN`` of the
        control torque, over the whole probed curve to 90 deg.  Logged with
        the polar (the ``torque limit`` event); nothing flies it.
        """
        curve = self._nearest(self._hold, speed, altitude)
        if not curve:
            return None
        rho = self.density(altitude)
        q = 0.5 * rho * speed * speed
        if q <= 0.0:
            return curve[-1][0]
        budget = self.cfg.AERO_STEER_HOLD_MARGIN * (
            self._hold_fixed + self._hold_surface * q) / q
        last_a, last_m = 0.0, 0.0
        for aoa, moment in curve:
            if moment > budget:
                if moment <= last_m:
                    return last_a
                return last_a + (aoa - last_a) * (budget - last_m) / (
                    moment - last_m)
            last_a, last_m = aoa, max(last_m, moment)
        return curve[-1][0]

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
        """Which way the airflow comes from, for a probe: the real velocity.

        Moving the airflow onto the vessel's axis instead (zero angle of
        attack in whatever direction it points) read 10.8-14.2 m^2 where the
        booster had 17.4-18.8, and the flight landed 3.3 km short.
        """
        return vec.unit(velocity)

    def curve_bins(self):
        """A copy of the Mach curve, one entry per bin (None if unfilled)."""
        return list(self._curve)

    def last_sweep_ut(self):
        return self._last_sweep_ut

    def request_resweep(self, replace=False):
        """Re-take the whole curve at the next descent-attitude refresh.

        The probe simulates the vessel as it is *now*, so after the airbrakes'
        deploy angle changes every bin is of a shape the booster no longer
        has until the next sweep.  An ordinary re-sweep goes through the
        clamp and the quarter-weight EMA, so it moves the curve only a
        quarter of the way to the new shape; ``replace`` throws the old
        curve away instead, since the change is known and not a spike.
        """
        self._last_sweep_ut = None
        self._next_aero_ut = None
        if replace:
            self._replace_next = True

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
        rotation, clean = self.probe_rotation(rotation, velocity, nose,
                                              position)

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
        if self._replace_next and self._curve_swept and not resweeping:
            fresh = [self._probe_bin(position, direction, i, altitude, c,
                                     rotation)
                     for i in range(self._bins)]
            if any(x is not None for x in fresh):
                self._replace_next = False
                self._last_sweep_ut = ut
                self._curve = [None] * self._bins
                self._samples = [0] * self._bins
                self._clean = [False] * self._bins
                self._curve_machs, self._curve_areas = [], []
                for index, sample in enumerate(fresh):
                    if sample is not None:
                        self._record(index, sample, clean)
                self.drag_area = self.drag_area_at(speed, altitude)
                return self.drag_area
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


def lift_peak(lifts):
    """The angle of the most lift in ``[(aoa, lift)]``, ascending in aoa.

    The sign is whatever the first probe reads (the probe's side vector is
    the vehicle's, so lift may come out negative); the peak is the largest
    lift of that sign, refined by the parabola through it and its two
    neighbours.  None if there is nothing to read.
    """
    if not lifts:
        return None
    sign = 1.0 if lifts[0][1] >= 0.0 else -1.0
    values = [sign * l for _, l in lifts]
    k = max(range(len(values)), key=lambda i: values[i])
    if 0 < k < len(values) - 1:
        (a0, y0), (a1, y1), (a2, y2) = ((lifts[j][0], values[j])
                                        for j in (k - 1, k, k + 1))
        den = y0 - 2.0 * y1 + y2
        if den < 0.0 and abs(a2 - a1 - (a1 - a0)) < 1e-9:
            return a1 + 0.5 * (a1 - a0) * (y0 - y2) / den
    return lifts[k][0]
