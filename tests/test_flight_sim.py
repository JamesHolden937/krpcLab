"""Closed-loop test: fly the real control loop against the fake game.

This is the end-to-end check that the phase machine sequences correctly and
that boostback + landing guidance actually converge on the pad.  It runs the
unmodified :class:`boosterland.autoland.Autoland` loop; only the game behind it
is fake.
"""

import math
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fakeksp                                              # noqa: E402
from boosterland import trajectory, vec                     # noqa: E402
from boosterland.config import Config                       # noqa: E402
from boosterland.logbook import Logbook                     # noqa: E402

SIM_DT = 0.2        # in-game seconds advanced per control tick


def _burn_trigger_height(log_path):
    """The ``h=`` the log records on the COAST -> LANDING_BURN transition."""
    for line in open(log_path):
        if "-> LANDING_BURN" in line:
            return float(line.split("h=")[1].split()[0])
    raise AssertionError("never lit the landing burn: %s" % log_path)


def fly(vessel, log_dir, max_ut=1200.0, cfg=None):
    conn = fakeksp.install(vessel)
    from boosterland import autoland          # imported after the fake krpc

    cfg = cfg or Config()
    cfg.LOG_DIR = log_dir
    cfg.PREDICT_DT_VACUUM = 2.0

    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    run = autoland.Autoland(conn, cfg, log)
    run.gui.start_button.clicked = True       # press START on the first tick

    def advance(_seconds):
        vessel.step(SIM_DT)
        conn.space_center.ut += SIM_DT

    with mock.patch("boosterland.autoland.time.sleep", advance):
        deadline = conn.space_center.ut + max_ut
        while not run.finished and conn.space_center.ut < deadline:
            run.tick()
            advance(cfg.LOOP_SLEEP_S)
    run.shutdown("test complete")
    log.close()
    return run, log.path


class TestClosedLoop(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    # A booster cannot rotate instantly, and pretending it can hides the
    # failure that mattered most: a steering command the vehicle never reaches
    # is a command it never obeys.  ~30 deg/s is what LOG7's booster managed
    # (a ~150 deg boostback flip inside 5 s).
    SLEW = 30.0

    def make_booster(self, atmosphere=True, slew=SLEW):
        """Post-separation state: 45 km up, 25 km downrange, still coasting up."""
        r = (fakeksp.RADIUS + 45000.0, 0.0, 25000.0)
        v = (250.0, 0.0, 1100.0)
        return fakeksp.Vessel(r, v, mass=60000.0, thrust=1.6e6, isp=290.0,
                              atmosphere=atmosphere, slew_deg_s=slew)

    def make_upper_stage(self, vessel, gap=25.0, push=3.0):
        """A stage that has fallen behind, so the boostback burn points at it.

        A stage *ahead* of the booster is receding and the burn pushes the
        other way, so it does not hold the flip at all -- see
        ``keep_out_distance`` and LOG16.  Behind is the geometry that waits.
        """
        behind = fakeksp._scale(fakeksp._unit(vessel.v), -1.0)
        return fakeksp.OtherCraft(
            fakeksp._add(vessel.r, fakeksp._scale(behind, gap)),
            fakeksp._add(vessel.v, fakeksp._scale(behind, push)))

    def make_log7_booster(self):
        """The real entry state from LOG7, which missed by 1154 m."""
        r = (fakeksp.RADIUS + 18478.0, 0.0, 16073.0)
        v = (377.5, 0.0, 582.9)
        return fakeksp.Vessel(r, v, mass=29660.0, thrust=520000.0, isp=300.0,
                              atmosphere=True, slew_deg_s=self.SLEW)

    def test_booster_lands_through_an_atmosphere(self):
        """The same flight with drag the guidance has to estimate as it goes.

        The fake's aerodynamic probe returns the true drag with a deliberate
        wobble, so this covers what the vacuum run cannot: a Cd*A that has to
        be smoothed, and a coast that keeps bending the trajectory after
        boostback has finished.
        """
        vessel = self.make_booster()
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)
        self.assertLess(vessel.impact_speed, 5.0,
                        "hit at %.1f m/s" % vessel.impact_speed)
        self.assertGreater(run.env.drag_area, 0.0, "never estimated drag")
        self.assertGreater(vessel.aero_probes, 0, "never probed the atmosphere")

    def test_booster_flies_the_whole_sequence_back_to_the_pad(self):
        # Vacuum and instant pointing: the fast, degenerate case, kept for
        # sequencing only.  It is not a Kerbin return -- with no drag the
        # booster arrives far too fast for its engines -- so it asserts that
        # every phase runs, not where it lands.
        vessel = self.make_booster(atmosphere=False, slew=None)
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN", "sequence did not complete")
        # The legs have to survive the arrival: cutting the engines on the
        # centre-of-mass altitude used to drop the booster its own half-length.
        self.assertIsNotNone(vessel.impact_speed)
        self.assertLess(vessel.impact_speed, 5.0,
                        "hit at %.1f m/s" % vessel.impact_speed)

        # Startup actions, gear and a populated log.
        #
        # **RCS is permitted, not held open.**  This used to assert the valve
        # itself, which was the same thing while START switched RCS on and
        # left it on for the flight.  It no longer is: the valve is a relay on
        # the pointing error, so a booster sitting on its legs pointed exactly
        # where it was told has its thrusters *shut*, and asserting otherwise
        # asserts the waste.  What START still owes is the permission.
        self.assertTrue(run.rcs_permitted)
        self.assertFalse(vessel.control.rcs,
                         "thrusters left open on a settled attitude")
        self.assertTrue(vessel.control.action_groups.get(2))
        self.assertTrue(vessel.control.gear)
        self.assertEqual(vessel.control.throttle, 0.0)   # engines cut at the end
        body = open(log_path).read()
        for phase in ("BOOSTBACK", "COAST", "LANDING_BURN", "TOUCHDOWN"):
            self.assertIn(phase, body)

    def test_it_lands_close_from_the_log7_entry_state(self):
        """The flight that missed by 1154 m, flown again.

        LOG7's booster passed 127 m from the pad at 8.5 km still carrying
        72 m/s of horizontal velocity, and coasted a kilometre past it.  The
        CORRECTION that should have caught that spent 36 s commanding an
        attitude 45-72 deg off retrograde -- unreachable in that airstream --
        with the throttle closed, and timed out.

        This asserts where the booster ends up, and deliberately not that
        CORRECTION ran.  It used to: with a single scalar Cd*A the coast
        reliably opened a few hundred metres of miss for the phase to trim,
        and now that the propagator interpolates a Mach curve this state flies
        the whole coast inside CORRECTION_ENTER_M and never needs it.  The
        phase itself is covered by the mid-coast-kick and stuck-turn tests
        below, which put a miss in front of it that no prediction can avoid.
        """
        vessel = self.make_log7_booster()
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 400.0,
                        "landed %.0f m from the pad (LOG7 managed 1154)"
                        % distance)
        self.assertLess(vessel.impact_speed, 5.0,
                        "hit at %.1f m/s" % vessel.impact_speed)

    def test_the_flip_starts_during_the_separation_coast(self):
        """Turning and drifting at once, not one then the other.

        The booster recedes from the pad at hundreds of m/s through the flip,
        so the seconds cost downrange the burn then has to undo.  Overlapping
        the turn with the separation coast is nearly free and, for a
        slow-turning vehicle, worth kilometres.  It is off by default because
        an unpowered flip at separation speed is not controllable on a real
        booster; this covers the option, not the default.
        """
        vessel = self.make_booster(slew=8.0)
        conn = fakeksp.install(vessel)
        # SEPARATION now lasts until the sky is clear rather than for a fixed
        # three seconds, so something the burn would run into has to actually
        # be there for there to be a coast to pre-turn during.
        fakeksp.add_craft(conn, vessel, self.make_upper_stage(vessel))
        from boosterland import autoland

        cfg = Config()
        cfg.SEPARATION_PRETURN = True
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True
        run.tick()                                   # STANDBY -> SEPARATION
        self.assertEqual(run.state, "SEPARATION")

        held = vessel.pointing
        for _ in range(10):          # 2 s, inside SEPARATION_COAST_S
            run.tick()
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT
        log.close()
        self.assertEqual(run.state, "SEPARATION", "left the phase too early")
        self.assertEqual(vessel.control.throttle, 0.0, "lit the engines early")
        self.assertGreater(fakeksp._angle_between(vessel.pointing, held), 0.05,
                           "sat still through the separation coast")

    def test_the_engines_light_before_the_flip_by_default(self):
        """Light, then flip -- the engines fly the turn, not the RCS.

        LOG9 flipped a 30 t booster at 19 km and 584 m/s on reaction wheels
        alone, because BOOSTBACK held the throttle shut until it was inside
        ``FLIP_ALIGN_DEG``.  With ``FLIP_UNDER_POWER`` the throttle comes up
        on the first BOOSTBACK tick however far off the aim the vehicle is,
        so the gimbal has authority for the whole turn.  The separation coast
        is still engines-off: the upper stage has to clear.
        """
        vessel = self.make_booster(slew=8.0)
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True
        run.tick()                                   # STANDBY -> SEPARATION
        self.assertEqual(run.state, "SEPARATION")

        held = vessel.pointing
        while run.state == "SEPARATION":
            run.tick()
            self.assertEqual(vessel.control.throttle, 0.0,
                             "lit the engines before the stage cleared")
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT
        self.assertAlmostEqual(
            fakeksp._angle_between(vessel.pointing, held), 0.0, places=3,
            msg="flipped unpowered through the separation coast")

        run.tick()                                   # first BOOSTBACK tick
        log.close()
        self.assertEqual(run.state, "BOOSTBACK")
        self.assertGreater(vessel.control.throttle, 0.0,
                           "waited for the flip before lighting the engines")
        aim = vessel.auto_pilot.target_direction     # radians, as the fake works
        self.assertGreater(fakeksp._angle_between(vessel.pointing, aim),
                           math.radians(cfg.FLIP_ALIGN_DEG),
                           "already aligned; the test proves nothing")

    def test_the_roll_is_commanded_and_held_all_flight(self):
        """Every attitude command carries a roll reference, and it is stable.

        Left unset, kRPC's autopilot only damps the roll rate, so the booster
        keeps whatever roll the flip left it in and drifts from there.  This
        checks the law over a real flight: a reference on every command,
        always perpendicular to the commanded nose, always in the trajectory
        plane, and never snapping to the other side between ticks -- the nose
        passes through both the vertical and a 180 deg flip, which is exactly
        where a naive reference inverts.
        """
        vessel = self.make_booster()
        run, _ = fly(vessel, self.tmp.name)
        history = vessel.auto_pilot.attitude_history

        self.assertEqual(run.state, "TOUCHDOWN")
        self.assertGreater(len(history), 100, "roll was never commanded")
        self.assertEqual(vessel.auto_pilot.target_roll, 0.0)

        previous = None
        for nose, up in history:
            self.assertAlmostEqual(fakeksp._norm(up), 1.0, places=6)
            self.assertAlmostEqual(fakeksp._dot(fakeksp._unit(nose), up), 0.0,
                                   places=6, msg="roof not perpendicular")
            if previous is not None:
                # The reference has to turn with the nose -- it stays
                # perpendicular to it -- so what matters is that it turns no
                # *further* than the nose did.  Anything beyond that is roll.
                moved = fakeksp._angle_between(up, previous[1])
                pitched = fakeksp._angle_between(nose, previous[0])
                self.assertLess(moved, pitched + math.radians(1.0),
                                "roll reference moved %.0f deg while the nose "
                                "moved %.0f deg" % (math.degrees(moved),
                                                    math.degrees(pitched)))
            previous = (nose, up)

    def test_roll_alignment_can_be_turned_off(self):
        """``ROLL_ALIGN=False`` leaves roll to the autopilot, as before."""
        vessel = self.make_booster()
        cfg = Config()
        cfg.ROLL_ALIGN = False
        run, _ = fly(vessel, self.tmp.name, cfg=cfg)

        self.assertEqual(run.state, "TOUCHDOWN")
        self.assertEqual(vessel.auto_pilot.attitude_history, [])
        self.assertTrue(math.isnan(vessel.auto_pilot.target_roll))

    def test_it_survives_ksp_refusing_to_engage_the_autopilot(self):
        """LOG10: the flight died on its first tick, throttle never touched.

        Engaging the autopilot makes kRPC total the vessel's available
        torque, and ModuleGimbal.GetPotentialTorque throws IndexOutOfRange
        while the gimbals are still settling after staging -- which is
        exactly when this script connects.  It is transient, so the sequence
        keeps going and keeps asking; what it must not do is end the flight.
        """
        vessel = self.make_booster()
        vessel.auto_pilot.engage_failures = 10      # ~2 s of refusals
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN")
        self.assertTrue(run.autopilot_engaged, "never got the autopilot back")
        self.assertGreater(run.autopilot_retries, 0, "test refused nothing")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)

        body = open(log_path).read()
        self.assertIn("autopilot refused engagement", body)
        self.assertIn("autopilot engaged after", body)

    def test_it_does_not_burn_boostback_with_nobody_steering(self):
        """A full-throttle burn along an unsteered attitude deepens the miss.

        FLIP_UNDER_POWER lights the engines without waiting for alignment, so
        the alignment check no longer stands between a dead autopilot and
        full thrust.  This is the check that replaces it.
        """
        vessel = self.make_booster()
        vessel.auto_pilot.engage_failures = 10**6   # never engages
        run, _ = fly(vessel, self.tmp.name, max_ut=60.0)

        self.assertFalse(run.autopilot_engaged)
        self.assertEqual(vessel.control.throttle, 0.0,
                         "burned with no attitude control")

    def test_a_diving_command_is_flown_over_the_top(self):
        """The routing is wired into every attitude command, not just tested.

        Asks for an attitude ~170 deg away and below the horizon -- the case
        where the shortest arc swings the nose down through the airstream --
        and checks what actually reached the autopilot.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        snap = run.telemetry.sample(run.env)
        log.close()

        # Both attitudes a little below the horizon, half a turn apart: the
        # shortest arc between them runs straight down through the nadir.
        zenith = fakeksp._unit(vessel.r)
        along = vec.unit(vec.cross(zenith, (0.0, 1.0, 0.0)))
        dip = fakeksp._scale(zenith, -0.3)
        vessel.pointing = fakeksp._unit(fakeksp._add(dip, along))
        below = fakeksp._unit(fakeksp._add(dip, fakeksp._scale(along, -1.0)))
        snap = run.telemetry.sample(run.env)
        self.assertLess(fakeksp._dot(snap.direction, zenith), 0.0)
        self.assertLess(fakeksp._dot(below, zenith), 0.0)

        run.aim(below, snap)
        commanded = vessel.auto_pilot.target_direction
        self.assertGreater(fakeksp._dot(fakeksp._unit(commanded), zenith), 0.0,
                           "commanded the nose down through the airstream")

        cfg.FLIP_VIA_VERTICAL = False
        run.aim(below, snap)
        self.assertLess(
            fakeksp._angle_between(vessel.auto_pilot.target_direction, below),
            1e-6, "routing could not be turned off")

    def test_the_prediction_holds_up_through_the_landing_burn(self):
        """Where it says it will land is where it lands.

        The propagator hands over to the landing burn and then flies it, so
        the prediction at the trigger has nothing left to guess.  The closed
        form this replaced assumed a full-throttle stop with no drag: on this
        entry it promised 163 m and the booster touched down 459 m out,
        having spent the whole burn drifting downrange exactly as predicted
        -- by a prediction nobody could act on any more.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        cfg.PREDICT_DT_VACUUM = 2.0
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True

        promised = None

        def advance(_seconds):
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT

        with mock.patch("boosterland.autoland.time.sleep", advance):
            deadline = conn.space_center.ut + 1200.0
            while not run.finished and conn.space_center.ut < deadline:
                was = run.state
                run.tick()
                if was == "COAST" and run.state == "LANDING_BURN":
                    promised = run.prediction.position
                advance(cfg.LOOP_SLEEP_S)
        run.shutdown("test complete")
        log.close()

        self.assertEqual(run.state, "TOUCHDOWN")
        self.assertIsNotNone(promised, "never lit the landing burn")
        drift = trajectory.surface_distance(run.env, promised, vessel.r)
        self.assertLess(drift, 150.0,
                        "landed %.0f m from where the trigger predicted" % drift)

    def test_it_lands_with_an_engine_that_cannot_throttle_deep(self):
        """Most real engines will not run below ~40%.

        The throttle law asks for whatever the profile needs, including 0.1;
        an engine with a floor either ignores that or snaps up to the floor,
        so the final approach is flown in bangs rather than smoothly.  It has
        to still arrive on its legs.
        """
        vessel = self.make_log7_booster()
        vessel.min_throttle = 0.4
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)
        self.assertLess(vessel.impact_speed, 6.0,
                        "hit at %.1f m/s" % vessel.impact_speed)

    def test_a_correction_that_deepens_the_miss_is_cut(self):
        """LOG2/LOG3: two burns that reported "helping" all the way out to 1.6 km.

        Both flights lit CORRECTION on a few hundred metres of coast drift and
        both burns walked the miss steadily the wrong way -- 335 -> 855 and
        927 -> 1596 -- while ``correction_gradient`` reported a healthy
        positive gradient every tick.  The gradient is one 5 m/s probe through
        a propagation, and low in the atmosphere it is linearised through drag
        that goes as v^2, so it held over the probe and not over the ~40 m/s
        the burn spent.  The phase watched the prediction and never the
        measurement.  This replays LOG3's own miss trace.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.state = "CORRECTION"
        run.phase_start_ut = 0.0
        run.correction_start_miss = 335.0

        # Inside the grace period nothing is judged: a burn that has to swing
        # the nose round first legitimately gives ground before it takes any.
        self.assertFalse(run.correction_making_it_worse(1.0, 583.0))
        # LOG3's first burn, at the tick the grace period expires.
        self.assertTrue(run.correction_making_it_worse(3.0, 583.0))
        # LOG3's second burn.
        run.correction_start_miss = 927.0
        self.assertTrue(run.correction_making_it_worse(6.0, 1254.0))
        # A burn that is working is never cut, however far it still has to go.
        self.assertFalse(run.correction_making_it_worse(6.0, 800.0))
        # Nor is one whose prediction has gone briefly noisy inside the ratio.
        self.assertFalse(run.correction_making_it_worse(6.0, 1100.0))
        # A prediction that is not a number cannot condemn a burn.
        self.assertFalse(run.correction_making_it_worse(6.0, float("nan")))
        log.close()

    def test_coast_drift_is_corrected_before_the_landing_burn(self):
        """A mid-coast shove must be flown out, not carried to the ground.

        This is the LOG3 failure in miniature: boostback finishes on target,
        the coast opens a kilometres-wide miss, and the landing burn is far too
        short to translate that far.  The correction phase exists to catch it.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True

        kicked = False

        def advance(_seconds):
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT

        with mock.patch("boosterland.autoland.time.sleep", advance):
            deadline = conn.space_center.ut + 1200.0
            while not run.finished and conn.space_center.ut < deadline:
                run.tick()
                if not kicked and run.state == "COAST":
                    # ~2 km of downrange error, the size LOG3 accumulated.
                    vessel.v = fakeksp._add(vessel.v, (0.0, 0.0, 25.0))
                    kicked = True
                advance(cfg.LOOP_SLEEP_S)
        run.shutdown("test complete")
        log.close()

        self.assertTrue(kicked)
        self.assertGreater(run.corrections, 0, "never tried to correct")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)
        self.assertIn("CORRECTION", open(log.path).read())

    def test_correction_burns_for_gimbal_when_it_cannot_point(self):
        """Pointed the wrong way is a reason to light the engines, not wait.

        A booster falling through thick air has very little authority on RCS
        and reaction wheels; the gimbal is where the authority is, and it
        only exists while something is coming out of the engines.  LOG7 held
        the throttle shut for 36 s waiting for an attitude it could not reach
        unpowered, timed the burn out having done nothing, and let the miss
        grow 341 -> 966 m.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True

        def advance(_seconds):
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT

        kicked = False
        with mock.patch("boosterland.autoland.time.sleep", advance):
            deadline = conn.space_center.ut + 1200.0
            while not run.finished and conn.space_center.ut < deadline:
                run.tick()
                if not kicked and run.state == "COAST":
                    vessel.v = fakeksp._add(vessel.v, (0.0, 0.0, 25.0))
                    kicked = True
                if run.state == "CORRECTION":
                    break
                advance(cfg.LOOP_SLEEP_S)
        self.assertEqual(run.state, "CORRECTION", "never entered the phase")

        # Pin it somewhere it certainly was not asked to point, and hold it
        # there: an attitude error that is not closing is the definition of
        # stuck, and it takes two ticks to measure that.
        with mock.patch.object(fakeksp.Vessel, "_slew", lambda *a: None):
            vessel.pointing = fakeksp._scale(
                fakeksp._unit(vessel.auto_pilot.target_direction), -1.0)
            for _ in range(2):
                run.tick()
                advance(cfg.LOOP_SLEEP_S)
            self.assertEqual(run.state, "CORRECTION")
            self.assertAlmostEqual(vessel.control.throttle,
                                   cfg.CORRECTION_GIMBAL_THROTTLE, places=6,
                                   msg="waited for an attitude it cannot reach")

            cfg.CORRECTION_GIMBAL_THROTTLE = 0.0
            run.tick()
            log.close()
            self.assertEqual(vessel.control.throttle, 0.0,
                             "the gimbal burn cannot be turned off")

    def test_it_steers_the_coast_with_the_air(self):
        """A booster with a wing flies to the pad on it.

        Everything else in this flight that can move the landing point costs
        propellant and one of three engine relights, and can only ever land
        the vehicle *shorter* (failure 16).  The descent itself is a minute
        of flight through air thick enough to brake at over a g, with the
        grid fins out, and holding a few degrees off retrograde turns that
        into a sideforce in either direction for free.

        ``lift_per_rad`` gives the fake a wing so the loop has something to
        act on; the real vehicle's slope is whatever
        ``Environment.probe_lift_slope`` measures in flight, which is the
        point -- no number here is fitted to this test.
        """
        cfg = Config()
        cfg.AERO_STEER = True
        # In game the wing is held off until AERO_STEER_MAX_ALT_M, because up
        # at the boostback exit the predicted miss is still failure 13's walk
        # rather than a miss, and an unsteered coast closes it by itself.  The
        # fake has no walk to wait for -- no transonic curve, no re-sweep, and
        # a Cd*A the propagator already agrees with -- so the gate here would
        # only be throwing away the altitude this test is about.
        cfg.AERO_STEER_MAX_ALT_M = 1e9
        vessel = fakeksp.Vessel((fakeksp.RADIUS + 45000.0, 0.0, 25000.0),
                                (250.0, 0.0, 1100.0), mass=60000.0,
                                thrust=1.6e6, isp=290.0, atmosphere=True,
                                slew_deg_s=self.SLEW, lift_per_rad=12.0)
        run, log_path = fly(vessel, self.tmp.name, cfg=cfg)

        self.assertEqual(run.state, "TOUCHDOWN")
        distance = trajectory.surface_distance(run.env, vessel.r,
                                               run.env.target)
        self.assertLess(distance, 60.0,
                        "steered flight landed %.0f m from the pad" % distance)
        self.assertLess(vessel.impact_speed, 5.0)

        # The slope is measured, sign included.  The fake pushes *opposite*
        # the nose offset, so a correct measurement comes back negative --
        # and a guidance law that assumed the sign would fly this backwards.
        self.assertIsNotNone(run.env.lift_slope)
        self.assertLess(run.env.lift_slope, 0.0,
                        "measured the sideforce with the wrong sign")
        self.assertAlmostEqual(abs(run.env.lift_slope), 12.0, delta=1.5)

        angles = [float(line.split("aoa=")[1].split()[0])
                  for line in open(log_path) if "aoa=" in line]
        self.assertTrue(any(abs(a) > 1.0 for a in angles),
                        "never actually commanded an angle of attack")
        self.assertTrue(all(abs(a) <= cfg.AERO_STEER_MAX_AOA_DEG + 1e-6
                            for a in angles), "flew past the attitude limit")

    def test_a_booster_with_no_wing_is_not_steered(self):
        """No measured lift, no commanded angle -- and no worse a landing.

        The slope has no fallback value on purpose: a steering gain nobody
        measured is a gain with an unknown sign, and this is the case where
        the honest answer is to hold retrograde and let the engines do the
        work.  ``fakeksp``'s default vessel has no wing at all, which is the
        strongest version of the test.
        """
        cfg = Config()
        cfg.AERO_STEER = True
        vessel = self.make_booster()
        run, log_path = fly(vessel, self.tmp.name, cfg=cfg)

        self.assertEqual(run.state, "TOUCHDOWN")
        angles = [float(line.split("aoa=")[1].split()[0])
                  for line in open(log_path) if "aoa=" in line]
        self.assertTrue(angles, "logged no steering column at all")
        self.assertTrue(all(abs(a) < 1e-6 for a in angles),
                        "steered a vehicle that has no measurable lift")
        self.assertLess(vessel.impact_speed, 5.0)

    def test_correction_does_not_burn_while_it_is_coming_round(self):
        """Mid-turn is not stuck, and burning through it costs accuracy.

        The gimbal burn is inefficient on purpose; it has to stay off a
        vehicle whose RCS is closing the error perfectly well, or every
        correction pays for authority it did not need.
        """
        fakeksp.install(self.make_booster())     # the fake krpc, not the real one
        from boosterland import autoland

        cfg = Config()
        run = autoland.Autoland.__new__(autoland.Autoland)
        run.cfg = cfg
        run.correction_align = None

        self.assertFalse(run.turning_too_slowly(None),
                         "burned before it had measured anything")
        brisk = cfg.CORRECTION_STUCK_RATE_DEG_S + 5.0
        self.assertFalse(run.turning_too_slowly(brisk),
                         "burned while the error was closing fast")
        self.assertTrue(run.turning_too_slowly(0.1), "did not notice it was stuck")
        self.assertTrue(run.turning_too_slowly(-4.0), "losing ground counts too")

        cfg.CORRECTION_GIMBAL_THROTTLE = 0.0
        self.assertFalse(run.turning_too_slowly(0.1),
                         "the gimbal burn cannot be turned off")

    def test_the_alignment_rate_needs_two_ticks(self):
        fakeksp.install(self.make_booster())
        from boosterland import autoland

        run = autoland.Autoland.__new__(autoland.Autoland)
        run.correction_align = None
        self.assertIsNone(run.alignment_rate(100.0, 60.0))
        self.assertAlmostEqual(run.alignment_rate(102.0, 40.0), 10.0)
        self.assertAlmostEqual(run.alignment_rate(103.0, 45.0), -5.0)
        self.assertIsNone(run.alignment_rate(103.0, 45.0), "no time passed")

    def test_a_broken_terrain_sensor_does_not_end_the_flight(self):
        """The LOG5 failure: ``surface_altitude`` stuck at about -7e17 m.

        That one bad number tripped the landing-burn trigger the instant
        boostback ended and then satisfied the touchdown backstop, so the
        engines were cut at 24 km and 386 m/s.  With the sensor rejected, the
        pad radius carries the flight and the booster still lands.
        """
        vessel = self.make_booster()
        vessel.broken_terrain_sensor = True
        run, log_path = fly(vessel, self.tmp.name)

        self.assertEqual(run.state, "TOUCHDOWN")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)
        self.assertIsNotNone(vessel.impact_speed)
        self.assertLess(vessel.impact_speed, 6.0)
        self.assertIn("surface_altitude REJECTED", open(log_path).read())

    def test_a_garbage_bounding_box_does_not_end_the_flight(self):
        """The LOG6 failure: ``bounding_box`` returned a -7e17 m lower corner.

        Measured once at startup, that offset became every landing height for
        the whole flight: the landing burn lit at 26 km and burned off 300 m/s
        of horizontal velocity sideways -- the flip that was seen in game --
        and then the hover backstop cut the engines still 26 km up.
        """
        vessel = self.make_booster()
        vessel.broken_bounding_box = True          # and it never recovers
        run, log_path = fly(vessel, self.tmp.name)

        # The touchdown speed is not asserted: with the vehicle's own length
        # unknown the flare cannot be placed exactly.  What must not happen is
        # LOG6 -- a landing burn kilometres up and engines cut in mid-air.
        self.assertEqual(run.state, "TOUCHDOWN")
        self.assertEqual(vessel.situation, fakeksp.Situation.landed,
                         "cut the engines in mid-air")
        distance = trajectory.surface_distance(run.env, vessel.r, run.env.target)
        self.assertLess(distance, 500.0,
                        "landed %.0f m from the pad" % distance)
        healthy = fly(self.make_booster(), self.tmp.name)[1]
        self.assertAlmostEqual(_burn_trigger_height(log_path),
                               _burn_trigger_height(healthy), delta=500.0,
                               msg="landing burn lit at the wrong height")

    def test_the_bounding_box_is_re_measured_until_it_is_believable(self):
        """A box that is garbage at connect time but fine a moment later."""
        vessel = self.make_booster()
        vessel.broken_bounding_box = True
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        self.assertIsNone(run.telemetry.leg_clearance, "believed the garbage")

        vessel.broken_bounding_box = False           # parts settle
        run.tick()
        log.close()
        self.assertAlmostEqual(run.telemetry.leg_clearance,
                               vessel.bottom + cfg.LEG_CLEARANCE_MARGIN_M)
        self.assertIn("leg clearance", open(log.path).read())

    def test_terminate_button_stops_the_script(self):
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True
        run.tick()                              # starts the sequence
        run.gui.stop_button.clicked = True
        run.tick()                              # terminate is handled first
        run.shutdown(run.exit_reason)
        log.close()

        self.assertTrue(run.finished)
        self.assertIn("terminated", run.exit_reason)
        self.assertEqual(vessel.control.throttle, 0.0)
        self.assertFalse(vessel.auto_pilot.engaged)
        self.assertTrue(vessel.control.sas)
        self.assertTrue(conn.closed)


class TestDragCurve(unittest.TestCase):
    """Cd*A is a curve against Mach, not a number.

    LOG15's boostback ended sitting on the transonic peak (49 m^2) and the
    propagator then flew the whole supersonic descent with that, against a
    real 20-30 m^2.  Over-stated drag lands the *prediction* short of the
    truth, so boostback stopped burning while the real touchdown point was
    still well beyond the pad, and the booster crossed the pad at 9 km.
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = Logbook(self.tmp.name, 1.0)
        self.addCleanup(self.log.close)

    def environment(self, vessel):
        from boosterland.environment import Environment
        conn = fakeksp.install(vessel)
        return Environment(conn, vessel, vessel.body, Config(), self.log, 0.0)

    def make_vessel(self):
        r = (fakeksp.RADIUS + 12000.0, 0.0, 0.0)
        return fakeksp.Vessel(r, (0.0, 0.0, 400.0), mass=30000.0,
                              thrust=520000.0, isp=300.0, atmosphere=True)

    def test_a_probe_off_retrograde_is_not_taken_as_a_measurement(self):
        """The angle is measured from retrograde: the engines face the flow.

        ``tests/fakeksp`` cannot show what this is worth -- its drag ignores
        attitude entirely, so a broadside probe and a nose-on one return the
        same number.  The evidence is LOG53, where the curve at the boostback
        exit propagates the coast 1297 m wrong and the curve at touchdown
        gets it to 6 m; see ``Environment.sample_is_clean``.  What is
        testable here is the bookkeeping that follows from it.
        """
        env = self.environment(self.make_vessel())
        env.cfg.DRAG_PROBE_MAX_AOA_DEG = 20.0   # ships off; see the config note
        velocity = (0.0, 0.0, 400.0)
        self.assertTrue(env.sample_is_clean(velocity, (0.0, 0.0, -1.0)))
        self.assertTrue(env.sample_is_clean(velocity, (0.0, 0.2, -1.0)))
        self.assertFalse(env.sample_is_clean(velocity, (0.0, 1.0, 0.0)))
        self.assertFalse(env.sample_is_clean(velocity, (0.0, 0.0, 1.0)))
        env.cfg.DRAG_PROBE_MAX_AOA_DEG = 0.0
        self.assertTrue(env.sample_is_clean(velocity, (0.0, 1.0, 0.0)),
                        "0 should disable the gate, not reject everything")

    def test_a_broadside_sample_is_provisional_until_a_clean_one_lands(self):
        """It fills an empty bin, and the first clean probe throws it out.

        Both halves matter.  The curve has to hold something from the first
        prediction, or boostback's early decisions are made with no drag at
        all -- but a bin written broadside is wrong by a third on this
        vehicle, and easing away from it at ``DRAG_SMOOTHING`` per probe
        takes most of the coast, which is exactly how long LOG53's predicted
        miss took to walk out.
        """
        env = self.environment(self.make_vessel())

        env._record(3, 60.0, clean=False)
        self.assertEqual(env._curve[3], 60.0, "an empty bin takes anything")

        env._record(3, 20.0, clean=True)
        self.assertEqual(env._curve[3], 20.0,
                         "the first real measurement replaces, not averages")

        env._record(3, 60.0, clean=False)
        self.assertEqual(env._curve[3], 20.0,
                         "a broadside probe must not move a measured bin")

        env._record(3, 24.0, clean=True)
        self.assertGreater(env._curve[3], 20.0)
        self.assertLess(env._curve[3], 24.0,
                        "clean samples still low-pass against each other")

    def test_the_curve_is_populated_even_if_the_booster_never_faces_the_flow(self):
        """A flight that only ever probes broadside still gets a curve."""
        vessel = self.make_vessel()
        env = self.environment(vessel)
        env.cfg.DRAG_PROBE_MAX_AOA_DEG = 20.0   # ships off; see the config note
        broadside = (0.0, 1.0, 0.0)          # 90 deg off, always rejected
        for tick in range(20):
            env.refresh_drag(tick * Config().AERO_REFRESH_UT, vessel.r,
                             vessel.v, (0.0, 0.0, 0.0, 1.0), broadside)
        self.assertTrue(env._curve_machs, "no curve at all to predict with")
        self.assertFalse(any(env._clean),
                         "broadside probes must not count as measurements")

    def test_a_bin_can_escape_a_junk_opening_sample(self):
        """The opening sweep is taken at startup, and startup is thin air.

        LOG60 opened its Mach 1.38 bin at 2.1 m^2 against a real 23 and took
        ninety seconds to get there -- clamped to 2x a step and then
        low-passed at a quarter weight, which is the right treatment for a
        spike hitting a settled bin and the wrong one for a bin that has only
        ever seen rubbish.  Boostback had exited a minute before it arrived.
        """
        env = self.environment(self.make_vessel())
        env.cfg.DRAG_WARMUP_SAMPLES = 4     # ships off; see the config note
        env._record(5, 2.0)
        for _ in range(3):
            env._record(5, 23.0)
        self.assertGreater(env._curve[5], 15.0,
                           "still at %.1f after three good samples"
                           % env._curve[5])

        # ... and once warmed up, a spike is clamped and low-passed as before.
        for _ in range(10):
            env._record(5, 23.0)
        settled = env._curve[5]
        env._record(5, 500.0)
        self.assertLess(env._curve[5], 2.0 * settled,
                        "a spike moved a settled bin by more than the clamp")

    def test_the_curve_tracks_the_transonic_hump(self):
        """The shape, not any one bin.

        Every probe carries the fake's deliberate +/-25% wobble -- the real
        one is evaluated at whatever attitude the booster holds -- so what has
        to be right is where the peak is and how much bigger it is than the
        supersonic tail.  That is what a boostback burn is aimed with; a bin
        being 20% out is what ``DRAG_SMOOTHING`` is for.
        """
        vessel = self.make_vessel()
        env = self.environment(vessel)
        for tick in range(40):
            env.refresh_drag(tick * Config().AERO_REFRESH_UT, vessel.r,
                             vessel.v, (0.0, 0.0, 0.0, 1.0))

        altitude = 12000.0
        curve = list(zip(env._curve_machs, env._curve_areas))
        peak_mach = max(curve, key=lambda pair: pair[1])[0]
        self.assertTrue(0.7 <= peak_mach <= 1.5,
                        "peak Cd*A at Mach %.2f, not transonic" % peak_mach)

        hump = env.drag_area_at(320.0, altitude)
        tail = [area for mach, area in curve if mach > 2.0]
        self.assertGreater(hump, 1.25 * (sum(tail) / len(tail)),
                           "flat across Mach: one scalar would do")
        self.assertAlmostEqual(hump / vessel.drag_area(320.0), 1.0, delta=0.4,
                               msg="Cd*A at the hump is %.1f, truth %.1f"
                                   % (hump, vessel.drag_area(320.0)))

    def test_one_probe_costs_a_bounded_number_of_calls(self):
        """The sweep happens once; after that it is two probes a refresh.

        A propagation is hundreds of steps and there are several a tick, so
        the curve has to be a local table.  Refreshing it must not turn into
        a per-step remote call.
        """
        vessel = self.make_vessel()
        env = self.environment(vessel)
        env.refresh_drag(0.0, vessel.r, vessel.v, (0.0, 0.0, 0.0, 1.0))
        swept = vessel.aero_probes
        self.assertLessEqual(swept, Config().DRAG_CURVE_BINS)
        for tick in range(1, 6):
            env.refresh_drag(tick * Config().AERO_REFRESH_UT, vessel.r,
                             vessel.v, (0.0, 0.0, 0.0, 1.0))
        self.assertLessEqual(vessel.aero_probes - swept, 10)

    def test_the_prediction_uses_the_curve_not_the_current_value(self):
        """Which is the whole point: a booster changes Mach as it falls."""
        vessel = self.make_vessel()
        env = self.environment(vessel)
        env.refresh_drag(0.0, vessel.r, vessel.v, (0.0, 0.0, 0.0, 1.0))
        cfg = Config()

        curved = trajectory.predict_landing(env, vessel.r, vessel.v, 30000.0,
                                            0.0, cfg)
        pinned = trajectory.predict_landing(env, vessel.r, vessel.v, 30000.0,
                                            0.0, cfg, drag_area=env.drag_area)
        self.assertGreater(vec.norm(vec.sub(curved.position, pinned.position)),
                           100.0,
                           "holding Cd*A at the current Mach changed nothing; "
                           "the propagator is not reading the curve")


class TestThrottleWatchdog(unittest.TestCase):
    """Say so when the vessel is not at the throttle it was told to hold.

    LOG16 spent the whole of SEPARATION at ``thr=0.98`` while the phase set
    the throttle to zero every tick, gaining 70 m/s and 9 km of downrange
    before it was terminated.  kRPC's throttle is the same value the player's
    throttle axis writes and KSP reasserts that axis every frame, so a
    physical throttle left up from the ascent wins; the script cannot stop it,
    but it can stop it being silent.
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_a_throttle_the_game_is_holding_up_is_reported(self):
        vessel = fakeksp.Vessel(
            (fakeksp.RADIUS + 18478.0, 0.0, 16073.0), (377.5, 0.0, 582.9),
            mass=29660.0, thrust=520000.0, isp=300.0, atmosphere=True)
        conn = fakeksp.install(vessel)
        from boosterland import autoland

        cfg = Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True
        run.tick()                                   # STANDBY -> SEPARATION
        for _ in range(6):
            # The game puts the throttle back up after every command, the way
            # the player's axis does.
            run.tick()
            vessel.control.throttle = 0.98
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT
        log.close()

        body = open(log.path).read()
        self.assertIn("throttle DIVERGED", body)
        self.assertIn("0.98", body)
        self.assertEqual(body.count("throttle DIVERGED"), 1,
                         "reported every tick instead of once per episode")


class TestSeparationClearance(unittest.TestCase):
    """SEPARATION ends when the sky is clear, not when a timer runs out."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def make_booster(self):
        r = (fakeksp.RADIUS + 18478.0, 0.0, 16073.0)
        return fakeksp.Vessel(r, (377.5, 0.0, 582.9), mass=29660.0,
                              thrust=520000.0, isp=300.0, atmosphere=True,
                              slew_deg_s=30.0)

    def start(self, vessel, conn, cfg=None):
        from boosterland import autoland
        cfg = cfg or Config()
        cfg.LOG_DIR = self.tmp.name
        log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
        self.addCleanup(log.close)
        run = autoland.Autoland(conn, cfg, log)
        run.gui.start_button.clicked = True
        run.tick()                                  # STANDBY -> SEPARATION
        return run, log

    def advance(self, run, conn, vessel, seconds):
        """Tick until the phase changes or the time runs out."""
        end = conn.space_center.ut + seconds
        while run.state == "SEPARATION" and conn.space_center.ut < end:
            run.tick()
            vessel.step(SIM_DT)
            conn.space_center.ut += SIM_DT
        return conn.space_center.ut

    def test_an_empty_sky_flips_at_once(self):
        """No stage to clear, so no reason to spend downrange seconds waiting.

        The three-second coast this replaces was a guess, and at separation
        the booster is receding from the pad at hundreds of m/s -- every
        second of it is work the boostback burn then has to undo.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        run, log = self.start(vessel, conn)
        self.assertEqual(run.state, "SEPARATION")
        start = conn.space_center.ut
        end = self.advance(run, conn, vessel, 10.0)
        self.assertEqual(run.state, "BOOSTBACK")
        self.assertLess(end - start, 1.0,
                        "waited %.1f s with nothing in the way" % (end - start))

    def test_a_stage_the_burn_would_hit_holds_the_flip_until_it_clears(self):
        """The boostback burn points back the way the booster came.

        So a stage that has fallen *behind* -- one that separated slowly, or
        the booster overtook -- is the one the burn drives into, and the one
        worth waiting out.  A stage ahead is not: it is receding, the burn
        pushes the other way, and holding for it is what cost LOG16 an
        airbrake.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        behind = fakeksp._scale(fakeksp._unit(vessel.v), -1.0)
        fakeksp.add_craft(conn, vessel, fakeksp.OtherCraft(
            fakeksp._add(vessel.r, fakeksp._scale(behind, 25.0)),
            fakeksp._add(vessel.v, fakeksp._scale(behind, 3.0))))
        run, log = self.start(vessel, conn)
        start = conn.space_center.ut
        end = self.advance(run, conn, vessel, 30.0)

        self.assertEqual(run.state, "BOOSTBACK", "never flipped")
        held = end - start
        self.assertGreater(held, 0.5, "flipped straight into the upper stage")
        self.assertLess(held, Config().SEPARATION_MAX_COAST_S,
                        "cleared on the guard, not on the geometry")
        self.assertEqual(vessel.control.throttle, 0.0,
                         "lit the engines before the stage cleared")
        self.assertIn("clearance scan", open(log.path).read())

    def test_a_stage_ahead_and_receding_does_not_hold_it(self):
        """LOG16, which is the one that has actually happened.

        The stage 19 m ahead reported a 44.5 m bounding radius, the booster's
        own box was unreadable so it flew on a 15 m fallback, and a 64.5 m
        keep-out against a 19 m gap could never clear.  The booster sat at
        Mach 2.4 with its grid fins deployed until it was terminated, and it
        lost an airbrake to the airstream it was sitting in.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        ahead = fakeksp._unit(vessel.v)
        stage = fakeksp.OtherCraft(
            fakeksp._add(vessel.r, fakeksp._scale(ahead, 19.0)),
            fakeksp._add(vessel.v, fakeksp._scale(ahead, 2.0)),
            size=44.0)
        fakeksp.add_craft(conn, vessel, stage)
        run, log = self.start(vessel, conn)
        start = conn.space_center.ut
        end = self.advance(run, conn, vessel, 30.0)

        self.assertEqual(run.state, "BOOSTBACK")
        self.assertLess(end - start, 1.0,
                        "held %.1f s for a stage that was already leaving"
                        % (end - start))

    def test_a_stage_that_never_clears_still_lets_the_booster_go(self):
        """The guard.

        A stage matching the booster's velocity exactly never separates, and
        sitting right where the boostback burn is about to go, so nothing the
        scan can see ever changes.  A booster that never flips is lost for
        certain -- one that flips next to what it dropped only probably is,
        and one that sits still at Mach 2.4 loses parts (LOG16).  So the wait
        is bounded, and the log says why it ended.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        behind = fakeksp._scale(fakeksp._unit(vessel.v), -12.0)
        fakeksp.add_craft(conn, vessel, fakeksp.OtherCraft(
            fakeksp._add(vessel.r, behind), vessel.v,
            follow=vessel, offset=behind))
        cfg = Config()
        cfg.SEPARATION_MAX_COAST_S = 5.0
        run, log = self.start(vessel, conn, cfg)
        start = conn.space_center.ut
        end = self.advance(run, conn, vessel, 30.0)

        self.assertEqual(run.state, "BOOSTBACK")
        self.assertGreaterEqual(end - start, cfg.SEPARATION_MAX_COAST_S)
        self.assertIn("TIMEOUT", open(log.path).read())

    def test_it_turns_while_it_waits(self):
        """Holding is not the same as sitting still.

        With nothing in the way the phase ends immediately and BOOSTBACK
        flies the whole flip under power, which is the better order (LOG9).
        But once the scan is holding, the choice is an unpowered flip against
        no flip at all, and LOG16 is what no flip costs: the booster held its
        separation attitude at Mach 2.4 with the grid fins deployed and lost
        an airbrake to the airstream.  Turning while waiting is free.
        """
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        behind = fakeksp._scale(fakeksp._unit(vessel.v), -1.0)
        fakeksp.add_craft(conn, vessel, fakeksp.OtherCraft(
            fakeksp._add(vessel.r, fakeksp._scale(behind, 25.0)),
            fakeksp._add(vessel.v, fakeksp._scale(behind, 3.0))))
        cfg = Config()
        self.assertFalse(cfg.SEPARATION_PRETURN, "this covers the default")
        run, log = self.start(vessel, conn, cfg)
        held = vessel.pointing
        end = self.advance(run, conn, vessel, 30.0)

        self.assertEqual(run.state, "BOOSTBACK")
        self.assertGreater(end - conn.space_center.ut, -1e-9)
        self.assertGreater(fakeksp._angle_between(vessel.pointing, held), 0.05,
                           "sat still through a hold it could have turned in")
        self.assertEqual(vessel.control.throttle, 0.0,
                         "the hold is engines-off whatever else it is")

    def test_the_old_timer_is_still_available(self):
        vessel = self.make_booster()
        conn = fakeksp.install(vessel)
        cfg = Config()
        cfg.SEPARATION_CLEARANCE_SCAN = False
        run, log = self.start(vessel, conn, cfg)
        start = conn.space_center.ut
        end = self.advance(run, conn, vessel, 30.0)
        self.assertEqual(run.state, "BOOSTBACK")
        self.assertGreaterEqual(end - start, cfg.SEPARATION_COAST_S - SIM_DT)


if __name__ == "__main__":
    unittest.main()
