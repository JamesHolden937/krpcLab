"""Every per-tick value, read through kRPC streams.

Streams rather than calls: a tick reads a dozen quantities and a remote call
each would dominate the loop.  Same structure as ``boosterland.telemetry``,
including the part of it that was paid for four times over -- the height.
"""
import math

from dataclasses import dataclass

from common import vec


@dataclass
class Snapshot:
    ut: float
    position: tuple
    velocity: tuple
    mass: float
    surface_altitude: float
    mean_altitude: float
    height_above_runway: float
    wheel_clearance: float
    situation: object
    alpha_actual: float
    surface_sane: bool
    liquid_fuel: float
    monopropellant: float
    oxidizer: float
    available_thrust: float
    max_thrust: float
    # **What the engine is producing right now, not what it could.**  The
    # burn's stop test measures the state, and a state sampled while thrust
    # is still tailing off is a state the vehicle has already left: measured
    # off the logs, the vehicle loses a further 2-9 m/s between the burn's
    # commanded cutoff and the atmosphere, state-dependently, which is worth
    # 30-130 km of entry ground track.  Nothing here had ever looked.
    thrust: float = 0.0
    # Vacuum Isp, so the drain's reserve can say what dv it actually buys
    # instead of being a units count nobody can check.
    vacuum_isp: float = 0.0
    skin_fraction: float = 0.0     # hottest part, as a fraction of its limit
    skin_hottest: str = ""         # which part that is
    # **Parts that have stopped answering, which is what breaking up looks
    # like from here.**  Every flight of this vehicle so far ends with kRPC
    # reporting 0 parts and 0.00 t -- it is being destroyed -- and until this
    # existed the log said nothing about it at all.
    parts_lost: int = 0
    parts_lost_names: tuple = ()   # the first few, so a log says what broke
    parts_now: int = 0             # 0 when it has not been counted yet
    # The game's own angles, not this module's reconstruction of them.
    # ``alpha_actual`` above is nose-to-velocity and so carries the slip
    # inside it; these two separate it.
    krpc_aoa: float = 0.0
    sideslip: float = 0.0
    # The pitch input the vessel is actually getting: kRPC's
    # ``Control.pitch`` reads back the *sum* of the game's state, this
    # client's manual input and the attitude controller's output
    # (``PilotAddon.OnFlyByWire``), so +-1 is a saturated elevator.
    pitch_input: float = 0.0
    # The nose, in the body's rotating frame.  Already streamed for
    # ``alpha_actual``; carried whole because the pointing *error* -- nose
    # against the commanded nose -- is a two-axis quantity and the angle to
    # the velocity throws the bank half of it away.
    nose: tuple = (0.0, 0.0, 0.0)
    # The roof (the vessel's -z), so the bank the vehicle *flies* can be set
    # beside the one it was commanded: the log carried only the command, and
    # a shuttle turning at a fifth of its commanded bank looked like it was
    # banked 40 degrees for a minute.  Empty when kRPC could not answer.
    roof: tuple = ()
    # -- what the air actually did, read from the game rather than inferred.
    # ``aero_force`` is the whole aerodynamic force in the body's rotating
    # frame, so a ``Cl*A``/``Cd*A`` taken from it is the *measured* polar at
    # the attitude the vehicle was really in.  ``aeroaudit.py`` has to
    # finite-difference the velocity for the same thing and pays 10-30% of
    # Coriolis for it; this is the force itself.
    aero_force: tuple = (0.0, 0.0, 0.0)
    air_density: float = 0.0       # the game's, not the table's
    dynamic_pressure: float = 0.0

    @property
    def speed(self):
        return vec.norm(self.velocity)

    @property
    def wheels_altitude(self):
        return self.surface_altitude - self.wheel_clearance

    @property
    def landing_height(self):
        """Height of the wheels over the ground, clamped on both sides.

        The terrain sensor says where the ground actually is and the centre of
        mass is metres above the tyres, so this is the only height worth
        flying a flare on.  But a sensor can be poisoned -- in boosterland a
        bad one ended *five* flights, twice by reading far too low and once by
        driving the vehicle back into the sky -- so the answer is clamped
        against the runway radius, which is the one height here no sensor can
        break.

        Both bounds matter and both have been paid for in the sibling project:
        the ground cannot rise more than ``TERRAIN_MAX_M`` above the runway,
        and it cannot fall further below it than the runway's own height over
        the datum plus a margin.  Do **not** replace this with
        ``min(height_above_runway, wheels_altitude)``: that is only
        conservative when the ground is *higher* than the target, and an
        aircraft that misses is over lower ground every time.
        """
        cfg = self._cfg
        pad = self.height_above_runway
        if not self.surface_sane:
            return pad
        low = pad - cfg.TERRAIN_MAX_M
        high = pad + self.runway_above_datum + cfg.TERRAIN_BELOW_SEA_M
        return vec.clamp(self.wheels_altitude, low, high)

    @property
    def max_accel(self):
        """Thrust over mass, with the engines-off case handled.

        ``available_thrust`` is zero when the engines are not active, which
        would silently make the deorbit solve think it had no vehicle.
        """
        if self.mass <= 0.0:
            return 0.0
        thrust = self.available_thrust or self.max_thrust
        return thrust / self.mass


class Telemetry:
    def __init__(self, conn, vessel, frame, cfg, logbook=None):
        self.conn = conn
        self.vessel = vessel
        self.frame = frame
        self.cfg = cfg
        self.logbook = logbook
        self.streams = []
        self.wheel_clearance = float(cfg.WHEEL_CLEARANCE_M)
        # None until the bounding box answers; a missing geometry must not
        # look like a generous one.
        self.tail_angle_deg = None
        self._clearance_measured = False
        self._clearance_warned = False
        self._surface_warned = False
        self.runway_above_datum = 0.0

        sc = conn.space_center
        flight = vessel.flight(frame)

        def stream(*args):
            s = conn.add_stream(*args)
            self.streams.append(s)
            return s

        self.ut = stream(getattr, sc, "ut")
        self.position = stream(vessel.position, frame)
        self.velocity = stream(vessel.velocity, frame)
        self.direction = stream(vessel.direction, frame)
        try:
            self.roof = stream(sc.transform_direction, (0.0, 0.0, -1.0),
                               vessel.reference_frame, frame)
        except Exception:                                   # noqa: BLE001
            self.roof = None
        self.mass = stream(getattr, vessel, "mass")
        self.surface_altitude = stream(getattr, flight, "surface_altitude")
        self.mean_altitude = stream(getattr, flight, "mean_altitude")
        self.situation = stream(getattr, vessel, "situation")
        self.available_thrust = stream(getattr, vessel, "available_thrust")
        self.thrust = stream(getattr, vessel, "thrust")
        self.vacuum_isp = stream(getattr, vessel, "vacuum_specific_impulse")
        # **The aerodynamic force, measured.**  The whole of the spaceplane's
        # open failure is a propagator that over-predicts the glide's range,
        # and every attempt to localise it so far has gone through finite
        # differences of the velocity -- which carry the Coriolis term the
        # audit has to warn about.  kRPC reports the force directly, and the
        # density and dynamic pressure the game applied to make it, so the
        # model's ``Cl*A``/``Cd*A`` can be compared against the vehicle's own
        # on every tick at no cost beyond three more streams.
        # **Sideslip, which nothing here had ever measured.**  ``aim`` builds
        # a coordinated command by construction -- the nose is placed in the
        # plane spanned by the velocity and the banked lift, so the relative
        # wind lies in the vehicle's plane of symmetry -- but nothing checked
        # the vehicle *flies* it that way.  It cannot be checked from
        # ``alpha_actual`` either: that is the angle between the nose and the
        # velocity, which conflates the two, so 20 deg of alpha with 10 of
        # slip reads the same as 22.4 deg with none.
        #
        # It matters most exactly where this vehicle is least understood: a
        # bank reversal is thirteen seconds of continuous roll at the rate
        # limit, and an uncoordinated roll both costs drag and pushes the
        # vehicle sideways -- a shorter flight and a cross-track error, which
        # are the two open problems.  kRPC reports it directly; failure 10b
        # is the entry about reconstructing what the game will simply hand
        # you.
        self.krpc_aoa = stream(getattr, flight, "angle_of_attack")
        self.sideslip = stream(getattr, flight, "sideslip_angle")
        try:
            self.pitch_input = stream(getattr, vessel.control, "pitch")
        except Exception:                                   # noqa: BLE001
            self.pitch_input = None
        self.aero_force = stream(getattr, flight, "aerodynamic_force")
        self.air_density = stream(getattr, flight, "atmosphere_density")
        self.dynamic_pressure = stream(getattr, flight, "dynamic_pressure")
        self.max_thrust = stream(getattr, vessel, "max_thrust")
        self.resources = vessel.resources

        # **Thermal margin, because nothing here had ever measured it.**  A
        # Mach 7 entry held at maximum lift is the part of this flight most
        # likely to end with parts coming off, and the guidance had no idea
        # how close it was.  Streamed per part rather than polled: kRPC pushes
        # a stream, so twenty-odd of them cost one setup and nothing per tick,
        # where reading ``parts.all`` every tick would be twenty-odd remote
        # calls inside the control loop.  The *limits* are read once -- they
        # do not change -- and only parts with a real skin limit are watched.
        self.skin = []
        try:
            for part in vessel.parts.all:
                limit = float(part.max_skin_temperature)
                if limit > 0.0:
                    self.skin.append((stream(getattr, part,
                                             "skin_temperature"), limit,
                                      part.title))
        except Exception:                                   # noqa: BLE001
            self.skin = []
        self._parts_at = None
        self._parts_seen = 0
        self._vessel = vessel

    def _count_parts(self, ut):
        """How many parts the vessel still has, polled slowly.

        The silent-stream count is free but indirect -- a stream can fall
        quiet for reasons other than the part being gone -- so this is the
        unambiguous version, and it is polled rather than streamed because
        ``parts.all`` is a list transfer and this project does not put those
        in the control loop (see the skin streams above, which exist for
        exactly that reason).

        Slowly is enough: parts come off over seconds of entry, not between
        two ticks, and what the log needs is *that* it happened and roughly
        when, not the tick it happened on.
        """
        every = float(getattr(self.cfg, "PART_COUNT_INTERVAL_UT", 5.0))
        if self._parts_at is not None and ut - self._parts_at < every:
            return self._parts_seen
        self._parts_at = ut
        try:
            self._parts_seen = len(self._vessel.parts.all)
        except Exception:                                   # noqa: BLE001
            pass            # a count we could not take is not a count of zero
        return self._parts_seen

    def sample(self, env):
        ut = self.ut()
        position = tuple(self.position())
        velocity = tuple(self.velocity())
        self.runway_above_datum = max(
            0.0, env.target_radius - env.equatorial_radius)
        self._refresh_wheel_clearance(ut)

        radius = vec.norm(position)
        height_above_runway = radius - env.target_radius
        mean_altitude = self.mean_altitude()
        raw = self.surface_altitude()
        sane = surface_altitude_sane(raw, mean_altitude, self.cfg,
                                     self.runway_above_datum)
        if not sane and not self._surface_warned and self.logbook:
            self._surface_warned = True
            self.logbook.event(ut, "surface_altitude implausible (%.1f with "
                                   "mean %.1f) -- flying on the runway radius"
                               % (raw, mean_altitude))

        # The angle of attack the vehicle is *achieving*, which is the only
        # way to find out what it can hold: the aerodynamic pitching moment at
        # an attitude the vessel is not in is not available from kRPC at any
        # price, so the command is ratcheted against this instead.
        alpha_actual = 0.0
        nose = tuple(self.direction())
        if vec.norm(velocity) > 5.0:
            # Unsigned, which is all that is wanted: the command is always a
            # positive angle of attack (the lift curve is symmetric, so a
            # negative one buys nothing a 180 degree bank does not), and what
            # this is compared against is the magnitude of that command.
            alpha_actual = vec.angle_between(nose, velocity)
            # **"The command is always positive" is not "the vehicle is
            # always positive."**  An unsigned angle reads a nose pitched
            # five degrees *below* the airflow as five above, so on the
            # shuttle's dive (LOG3035: kRPC -1.5, -4.7, -5.5 deg while this
            # read +4.6..+7.4) a fifteen degree tracking error looked like
            # five -- inside ``ALPHA_TRACK_TOLERANCE_DEG`` -- and nothing
            # reacted.  kRPC's angle is signed and in the pitch plane.
            if getattr(self.cfg, "ALPHA_SIGNED", False):
                try:
                    alpha_actual = float(self.krpc_aoa())
                except Exception:                           # noqa: BLE001
                    pass

        # The hottest part as a fraction of what it can take.  One number,
        # because that is the one that decides whether the vehicle arrives in
        # one piece, and the part's name with it so a log says what to look at.
        #
        # **A part that has stopped answering is counted, not skipped.**  The
        # loop used to ``continue`` past a failed read, so when the hottest
        # part burned off, its stream fell silent, it was quietly dropped, and
        # ``skin=`` reported the hottest *survivor* -- which reads as the
        # vehicle cooling down at the exact moment it is coming apart.  The
        # one instrument meant to catch a breakup was hiding it.
        fraction, hottest, silent = 0.0, "", 0
        lost = []
        for read, limit, title in self.skin:
            try:
                share = read() / limit
            except Exception:                               # noqa: BLE001
                silent += 1
                # **And say which one.**  "10 skin sensors have gone silent"
                # is the same line whether the tail scraped or the nose gear
                # folded, and those want opposite fixes -- one wants a lower
                # touchdown attitude and the other a softer one.  The names
                # are already here; only the count was being kept.
                if len(lost) < 8:
                    lost.append(title)
                continue
            if share > fraction:
                fraction, hottest = share, title

        snap = Snapshot(
            ut=ut, position=position, velocity=velocity, mass=self.mass(),
            surface_altitude=raw, mean_altitude=mean_altitude,
            height_above_runway=height_above_runway,
            wheel_clearance=self.wheel_clearance,
            situation=self.situation(), alpha_actual=alpha_actual,
            nose=nose,
            roof=self._roof(),
            surface_sane=sane,
            liquid_fuel=self._amount("LiquidFuel"),
            # **What the attitude has cost so far.**  The tank is 150 units
            # and the entry is three minutes; whether RCS can hold an angle
            # of attack the air is fighting is a question about this number,
            # not about torque.
            monopropellant=self._amount("MonoPropellant"),
            oxidizer=self._amount("Oxidizer"),
            available_thrust=self.available_thrust(),
            max_thrust=self.max_thrust(),
            thrust=self._scalar(self.thrust),
            vacuum_isp=self._scalar(self.vacuum_isp),
            skin_fraction=fraction, skin_hottest=hottest,
            parts_lost=silent, parts_lost_names=tuple(lost),
            parts_now=self._count_parts(ut),
            krpc_aoa=self._scalar(self.krpc_aoa),
            sideslip=self._scalar(self.sideslip),
            pitch_input=(self._scalar(self.pitch_input)
                         if self.pitch_input is not None else 0.0),
            aero_force=self._vector(self.aero_force),
            air_density=self._scalar(self.air_density),
            dynamic_pressure=self._scalar(self.dynamic_pressure))
        snap._cfg = self.cfg
        snap.runway_above_datum = self.runway_above_datum
        snap.nose = nose
        return snap

    @staticmethod
    def _vector(read):
        try:
            return tuple(read())
        except Exception:                               # noqa: BLE001
            return (0.0, 0.0, 0.0)

    def _roof(self):
        if self.roof is None:
            return ()
        try:
            return tuple(self.roof())
        except Exception:                               # noqa: BLE001
            return ()

    @staticmethod
    def _scalar(read):
        try:
            return float(read())
        except Exception:                               # noqa: BLE001
            return 0.0

    def _amount(self, name):
        try:
            return self.resources.amount(name)
        except Exception:                               # noqa: BLE001
            return 0.0

    def _aft_extent(self):
        """How far behind the centre of mass the furthest-aft part sits.

        Measured from the parts rather than taken from the bounding box,
        because the box has been seen to answer with a usable z beside a
        y eight times the length of the aircraft.  ``None`` when the parts
        cannot be read, so that the caller can fall back rather than treat a
        failure as a measurement.

        -y is aft in kRPC's vessel frame, and the frame's origin is the
        centre of mass, which is what the rotation is taken about.
        """
        try:
            frame = self.vessel.reference_frame
            aft = max(-part.position(frame)[1] for part in self.vessel.parts.all)
        except Exception:                               # noqa: BLE001
            return None
        return aft if aft > 0.0 else None

    def measure_gear_geometry(self, ut, mains, apply):
        """The landing geometry with the gear *down*, from every part's box.

        ``_refresh_wheel_clearance`` reads the vessel's box on the first
        sample, and every save this vehicle flies from has the gear up -- so
        on the shuttle it measured the fuselage's belly (1.82 m) as the
        wheels, and the tail-strike angle (11.0 deg) about a belly the
        vehicle never rolls on.  Gear down, the LY-60s put the tyres 3.72 m
        under the CoM and the first thing aft of them to reach the runway
        is the engine bell, at ~25 deg.  The flare flew 1.9 m of phantom
        height (contact at ``h 2.2``, every shuttle log) and capped its
        pitch at 8.8 deg.

        ``mains`` are the braked wheels' parts.  Returns the per-part
        corner table (vessel frame) for ``Autopilot.ground_watch``, or None.
        Sets ``wheel_clearance`` and ``tail_angle_deg`` only when ``apply``
        (``GEAR_GEOMETRY_DEPLOYED``)."""
        frame = self.vessel.reference_frame
        try:
            wheel_parts = [w.part for w in self.vessel.parts.wheels]
            rows = []
            for part in self.vessel.parts.all:
                lo, hi = part.bounding_box(frame)
                corners = [(x, y, z) for x in (lo[0], hi[0])
                           for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
                rows.append((part.title, part, part in wheel_parts,
                             part in mains, tuple(part.position(frame)),
                             corners))
        except Exception:                               # noqa: BLE001
            return None
        main_rows = [r for r in rows if r[3]]
        if not main_rows:
            return None
        ground_z = sum(max(c[2] for c in r[5]) for r in main_rows) / len(
            main_rows)
        axle_y = sum(r[4][1] for r in main_rows) / len(main_rows)
        tail, tail_part = None, None
        for title, _, is_wheel, _, _, corners in rows:
            if is_wheel:
                continue
            for x, y, z in corners:
                if y < axle_y - 0.3:
                    angle = math.degrees(math.atan2(ground_z - z, axle_y - y))
                    if tail is None or angle < tail:
                        tail, tail_part = angle, title
        sane = 0.1 <= ground_z <= self.cfg.WHEEL_CLEARANCE_MAX_M
        if self.logbook:
            self.logbook.event(
                ut, "gear-down geometry: tyres %.2f m under the CoM (was "
                    "%.2f), mains %.2f m aft, tail strike %s (%s)%s"
                % (ground_z, self.wheel_clearance, -axle_y,
                   "?" if tail is None else "%.1f deg" % tail, tail_part,
                   " -- applied" if apply and sane else
                   (" -- NOT sane, ignored" if not sane else
                    " -- logged only")))
        if apply and sane:
            self.wheel_clearance = ground_z
            self._clearance_measured = True
            if tail is not None and 1.0 <= tail <= 45.0:
                self.tail_angle_deg = tail
        return rows

    def _refresh_wheel_clearance(self, ut):
        """Centre of mass down to the tyres, re-measured until it is sane.

        The bounding box is the number that broke five boosterland flights:
        KSP returned a lower corner of about -7e17 m, it was read *once* at
        startup and cached, and every height in the flight was wrong from the
        first tick.  So this keeps asking until it gets an answer a vehicle
        could have, and flies on the configured fallback in the meantime.
        """
        if self._clearance_measured:
            return
        try:
            box = self.vessel.bounding_box(self.vessel.reference_frame)
        except Exception:                               # noqa: BLE001
            return
        # kRPC's vessel frame is x-right, y-forward, **z out of the bottom**,
        # so the lowest point of an aircraft -- its wheels -- is at the box's
        # *maximum* z.  boosterland takes ``abs(min(lower))`` because a
        # booster stands on its tail along -y; copied here unchanged it would
        # have returned half the wingspan and called it the leg clearance.
        candidate = abs(box[1][2])
        if 0.1 <= candidate <= self.cfg.WHEEL_CLEARANCE_MAX_M:
            self.wheel_clearance = candidate
            self._clearance_measured = True
            if self.logbook:
                self.logbook.event(ut, "wheel clearance measured %.2f m"
                                   % candidate)
            # **And how far the nose may come up before the tail arrives.**
            # The same box says where the back of the aircraft is: -y is aft
            # in kRPC's vessel frame, so rotating nose-up about the wheels
            # swings that point down, and it reaches the ground at
            # ``atan(wheel clearance / aft extent)``.  Measured, because it
            # is pure geometry and every airframe has a different one -- a
            # constant here would be the ``planeprobe`` mistake again, this
            # time deciding whether the tail hits.
            # **The box's z was checked and its y was not, and the y is
            # wrong.**  ``WHEEL_CLEARANCE_MAX_M`` rejects an impossible
            # clearance, so a half-broken box -- a usable z beside a garbage
            # y -- passes as a good measurement.  Flown: this aircraft is
            # 5.65 m from its docking port to its engine bell and the box
            # reported **24.95 m** of tail behind the CoM, giving a tail
            # strike angle of 5.0 deg where the airframe's own measured
            # figure is 18.7.  ``TAIL_STRIKE_MARGIN`` then made it 4.0, and
            # ``aim_runway`` clamps the flare's *pitch* command to that -- so
            # the phase whose only tool is angle of attack was being held to
            # four degrees of it by a number nobody had range-checked.
            #
            # A too-large ``aft`` yields a *small* angle, which sails through
            # a lower bound of 1.0 deg looking conservative.  That is
            # CLAUDE.md's rule exactly: a missing answer must not be allowed
            # to look like a good one.  So the aft extent is measured from
            # the parts, the way ``_release_nose_brake`` finds the nose
            # wheel -- the furthest-aft part along the direction the vehicle
            # points -- and the box is used only for the clearance it can
            # actually be checked on.
            # The box is still the source; the parts are the *check* on it.
            # Believing the parts instead would take the limit from 4.0 deg
            # straight to 28.8 in one step, on the phase that owns the
            # touchdown, on a geometry nobody has confirmed in game -- the
            # aft-most part is measured from the centre of mass and the
            # rotation really happens about the main wheels, and those two
            # differ by more than the margin does.  So: reject a box that the
            # airframe contradicts, and fly the fallback until someone
            # measures it properly.  ``TAIL_ANGLE_FALLBACK_DEG`` is 12, which
            # is three times what the broken box was allowing and still
            # inside the 18.7 deg this airframe is documented to have.
            #
            # **How often this fires: once.**  Over every log on disk the box
            # reads 18.7 or 19.9 deg with 3.5-3.8 m of tail behind the CoM;
            # the 24.95 m appears in a single hand-flown flight.  So this is a
            # guard against a rare hazard, not an explanation of the rollout
            # breakups -- but when it does fire nothing else in the flight
            # says so, and the flare loses three quarters of its authority
            # for the whole landing.
            aft = abs(box[0][1])
            parts_aft = (self._aft_extent()
                         if getattr(self.cfg, "TAIL_EXTENT_CHECK", False)
                         else None)
            if parts_aft is not None and aft > self.cfg.TAIL_EXTENT_SLACK * parts_aft:
                # **And then use the parts, because they answered.**  This
                # branch used to return, and the fallback constant flew --
                # so a measurement good enough to *veto* the box was not
                # good enough to replace it, which is the same shape as a
                # probe whose only consumer is a hand-copied number.
                # Measured on ``logs/LOG2756``: box 11.26 m against parts
                # 2.46 on a 24-part aircraft, so the fallback's 12 deg
                # capped the touchdown attitude at 9.6 where the geometry
                # allows 33.
                #
                # Part *positions* are origins, so this runs a little short
                # on the aft extent and therefore a little generous on the
                # angle -- on the aircraft where the box does work it reads
                # 19.9 deg against the 18.7 measured by hand, 6% optimistic.
                # ``TAIL_STRIKE_MARGIN`` is 0.8 and covers that; the point
                # of noting it is that the error has a known sign.
                if self.logbook:
                    self.logbook.event(
                        ut, "bounding box says %.2f m of tail behind the CoM "
                            "and the parts say %.2f -- flying on the parts"
                        % (aft, parts_aft))
                aft = parts_aft
            ceiling = (self.cfg.TAIL_EXTENT_MAX_M
                       if getattr(self.cfg, "TAIL_EXTENT_CHECK", False)
                       else float("inf"))
            if 0.5 < aft <= ceiling:
                angle = math.degrees(math.atan2(candidate, aft))
                if 1.0 <= angle <= 45.0:
                    self.tail_angle_deg = angle
                    if self.logbook:
                        self.logbook.event(
                            ut, "tail strike angle %.1f deg (wheels %.2f m "
                                "under the CoM, %.2f m of tail behind them)"
                            % (angle, candidate, aft))
            elif self.logbook:
                self.logbook.event(
                    ut, "tail extent %.2f m is not a vehicle -- flying on the "
                        "%.0f deg fallback" % (aft,
                                               self.cfg.TAIL_ANGLE_FALLBACK_DEG))

        elif not self._clearance_warned:
            self._clearance_warned = True
            if self.logbook:
                self.logbook.event(ut, "bounding box not usable (%.3g) -- "
                                       "flying on %.2f m and re-measuring"
                                   % (candidate, self.wheel_clearance))

    def close(self):
        for s in self.streams:
            try:
                s.remove()
            except Exception:                           # noqa: BLE001
                pass
        self.streams = []


def surface_altitude_sane(value, mean_altitude, cfg, runway_above_datum):
    """Is the terrain sensor's answer one the body could produce?"""
    if value is None or not math.isfinite(value):
        return False
    if value < -cfg.TERRAIN_BELOW_SEA_M - runway_above_datum - 50.0:
        return False
    if mean_altitude is not None and math.isfinite(mean_altitude):
        if value > mean_altitude + cfg.TERRAIN_BELOW_SEA_M + 50.0:
            return False
        if value < mean_altitude - cfg.TERRAIN_MAX_M - 50.0:
            return False
    return True
