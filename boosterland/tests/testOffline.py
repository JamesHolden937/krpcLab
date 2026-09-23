"""Offline checks for the parts that do not need a running game.

The propagator, the guidance laws, the log gating and the config overrides all
run against a stand-in for :class:`boosterland.environment.Environment`, so
``python3 -m unittest`` is a real regression test even with KSP
closed.
"""

import copy
import math
import os
import random
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from boosterland import environment, guidance, proximity, trajectory  # noqa: E402
from common import rcs, vec                                 # noqa: E402
from boosterland.telemetry import Snapshot                  # noqa: E402
from boosterland.config import Config, apply_overrides     # noqa: E402
from common.logbook import Logbook                    # noqa: E402
sys.modules.setdefault('krpc', __import__('types').ModuleType('krpc'))
from boosterland import autoland as autoland_module        # noqa: E402

KERBIN_MU = 3.5316000e12
KERBIN_R = 600000.0


class FakeEnv:
    """Same surface as Environment, with a analytic isothermal atmosphere."""

    def __init__(self, rotating=False, atmosphere=False):
        self.mu = KERBIN_MU
        self.equatorial_radius = KERBIN_R
        self.atmosphere_depth = 70000.0 if atmosphere else 0.0
        self.target = (KERBIN_R, 0.0, 0.0)
        self.target_radius = KERBIN_R
        self.omega = (0.0, 2.9089e-4, 0.0) if rotating else (0.0, 0.0, 0.0)
        self.drag_area = 0.0
        self.lift_slope = 0.0

    def density(self, altitude):
        if altitude >= self.atmosphere_depth or self.atmosphere_depth == 0.0:
            return 0.0
        return 1.225 * math.exp(-max(0.0, altitude) / 5600.0)

    def speed_of_sound(self, altitude):
        return 340.0

    def mach(self, speed, altitude):
        return speed / self.speed_of_sound(altitude)

    def drag_area_at(self, speed, altitude):
        """The real Environment interpolates a Mach curve here; a flat Cd*A
        is enough for the propagator tests, which only need the two sides of
        a comparison to see the same air."""
        return self.drag_area

    def lift_slope_at(self, speed, altitude):
        """Cl*A per radian squared, the way probe_lift_slope reports it.

        The real one is measured in flight; a constant is enough here,
        because what the solver's tests are about is whether it inverts the
        sensitivity it measures, not what the sensitivity is."""
        return self.lift_slope


def state_above_pad(height, velocity=(0.0, 0.0, 0.0)):
    return ((KERBIN_R + height, 0.0, 0.0), velocity)


class TestPropagation(unittest.TestCase):
    def test_vertical_drop_matches_free_fall(self):
        env = FakeEnv()
        r, v = state_above_pad(10000.0)
        prediction = trajectory.predict_landing(env, r, v, 1000.0, 0.0, Config())
        g = env.mu / (KERBIN_R ** 2)
        expected = math.sqrt(2 * 10000.0 / g)
        self.assertAlmostEqual(prediction.time_to_land, expected, delta=0.05 * expected)
        self.assertFalse(prediction.powered)

    def test_downrange_grows_with_horizontal_speed(self):
        env = FakeEnv()
        cfg = Config()
        r, _ = state_above_pad(20000.0)
        near = trajectory.predict_landing(env, r, (0.0, 0.0, 100.0), 1e4, 0.0, cfg)
        far = trajectory.predict_landing(env, r, (0.0, 0.0, 400.0), 1e4, 0.0, cfg)
        self.assertGreater(
            trajectory.surface_distance(env, far.position, env.target),
            trajectory.surface_distance(env, near.position, env.target))

    def test_drag_shortens_the_flight_path(self):
        cfg = Config()
        r, v = state_above_pad(30000.0, (0.0, 0.0, 500.0))
        dry = FakeEnv(atmosphere=True)
        wet = FakeEnv(atmosphere=True)
        wet.drag_area = 30.0
        clean = trajectory.predict_landing(dry, r, v, 40000.0, 0.0, cfg)
        dragged = trajectory.predict_landing(wet, r, v, 40000.0, 0.0, cfg)
        self.assertLess(
            trajectory.surface_distance(wet, dragged.position, wet.target),
            trajectory.surface_distance(dry, clean.position, dry.target))

    def test_landing_burn_shortens_the_touchdown_point(self):
        """The powered touchdown is short of where a rock would hit.

        The landing burn sheds horizontal velocity, so aiming boostback at the
        ballistic impact point would overshoot the pad; how much short it lands
        depends on how hard the vehicle can brake.
        """
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(8000.0, (0.0, 0.0, 300.0))
        ballistic = trajectory.predict_landing(env, r, v, 40000.0, 0.0, cfg)
        weak = trajectory.predict_landing(env, r, v, 40000.0, 20.0, cfg)
        strong = trajectory.predict_landing(env, r, v, 40000.0, 200.0, cfg)

        self.assertTrue(weak.powered and strong.powered)
        self.assertGreater(weak.burn_time, strong.burn_time)
        distances = [trajectory.surface_distance(env, p.position, env.target)
                     for p in (weak, strong, ballistic)]
        # Weaker braking starts higher and stops further short of the impact
        # point; infinite braking would land exactly on it.
        self.assertLess(distances[0], distances[1])
        self.assertLess(distances[1], distances[2])

    def test_rotating_frame_terms_deflect_the_impact(self):
        cfg = Config()
        r, v = state_above_pad(40000.0, (0.0, 0.0, 600.0))
        still = trajectory.predict_landing(FakeEnv(), r, v, 4e4, 0.0, cfg)
        spun = trajectory.predict_landing(FakeEnv(rotating=True), r, v, 4e4,
                                          0.0, cfg)
        self.assertGreater(vec.norm(vec.sub(still.position, spun.position)), 1.0)

    def _converged(self, env, r, v, cfg):
        """The same propagation with the step refined until it stops moving."""
        fine = copy.deepcopy(cfg)
        fine.PREDICT_DT_ATMO = 0.02
        fine.PREDICT_DT_VACUUM = 0.05
        return trajectory.predict_landing(env, r, v, 4e4, 0.0, fine)

    def test_the_step_is_fine_enough_to_have_stopped_mattering(self):
        """A propagation whose answer moves with the step size is a guess.

        Drag goes as v^2 through a density that changes by a factor of e
        every 5 km, so a first-order step does not add noise to a long
        atmospheric descent -- it biases it, always the same way, and
        boostback then stops where the biased answer says the pad is.  On
        LOG243, which landed 109 m from the pad, the prediction from 20 km
        read 48 m under semi-implicit Euler at the committed step and 106 m
        at a twentieth of it, with the vehicle doing nothing in between; the
        walk the coast was famous for was the integrator.  See CLAUDE.md
        failure 16.

        So the property to hold is not "RK4" but "converged": at the step the
        control loop can afford, the answer must be the one a much finer step
        gives.
        """
        env = FakeEnv(atmosphere=True)
        env.drag_area = 20.0
        cfg = Config()
        r, v = state_above_pad(30000.0, (-120.0, 0.0, 700.0))

        truth = self._converged(env, r, v, cfg)
        committed = trajectory.predict_landing(env, r, v, 4e4, 0.0, cfg)
        drift = trajectory.surface_distance(env, committed.position,
                                            truth.position)
        self.assertLess(drift, 15.0,
                        "the committed step lands %.0f m from the converged "
                        "answer" % drift)

        euler = copy.deepcopy(cfg)
        euler.PREDICT_RK4 = False
        old = trajectory.predict_landing(env, r, v, 4e4, 0.0, euler)
        self.assertGreater(
            trajectory.surface_distance(env, old.position, truth.position),
            drift,
            "Euler at this step used to be hundreds of metres out -- if it is "
            "not, this test no longer measures anything")


class TestMissComponents(unittest.TestCase):
    """The signed miss: which way, not just how far.

    ``surface_distance`` cannot tell an overshoot from an undershoot, and the
    two want opposite corrections -- see ``trajectory.miss_components``.
    """

    def setUp(self):
        self.env = FakeEnv()
        self.target = self.env.target                       # (R, 0, 0)
        # The booster approaches from +z, so "along" points from +z toward
        # the target, i.e. in -z; a touchdown at -z of the target is beyond it.
        self.booster = (KERBIN_R, 0.0, 20000.0)

    def offset(self, north, east):
        """A point ``east`` metres in -z and ``north`` metres in +y of the pad."""
        return (KERBIN_R, north, -east)

    def test_flying_past_the_target_reads_positive(self):
        beyond = self.offset(0.0, 500.0)
        along, cross = trajectory.miss_components(
            self.env, beyond, self.target, self.booster)
        self.assertAlmostEqual(along, 500.0, delta=1.0)
        self.assertAlmostEqual(cross, 0.0, delta=1.0)

    def test_stopping_short_reads_negative(self):
        short = self.offset(0.0, -500.0)
        along, _ = trajectory.miss_components(
            self.env, short, self.target, self.booster)
        self.assertAlmostEqual(along, -500.0, delta=1.0)

    def test_a_sideways_miss_is_all_crossrange(self):
        beside = self.offset(500.0, 0.0)
        along, cross = trajectory.miss_components(
            self.env, beside, self.target, self.booster)
        self.assertAlmostEqual(along, 0.0, delta=1.0)
        self.assertAlmostEqual(abs(cross), 500.0, delta=1.0)

    def test_the_magnitude_agrees_with_the_distance(self):
        point = self.offset(300.0, 400.0)
        along, cross = trajectory.miss_components(
            self.env, point, self.target, self.booster)
        self.assertAlmostEqual(math.hypot(along, cross),
                               trajectory.surface_distance(
                                   self.env, point, self.target),
                               delta=2.0)


class TestRollReference(unittest.TestCase):
    """The roof is rolled into the plane the guidance steers in.

    Every command this script issues -- boostback aim, coast bias, correction
    tilt -- lies in the trajectory plane, so that is where the pitch axis
    belongs.  The autopilot's own zenith reference cannot express it through
    the vertical; this one can.
    """

    def setUp(self):
        self.r = (6.0e5 + 30000.0, 0.0, 0.0)
        self.v = (400.0, 0.0, 900.0)

    def test_the_reference_is_perpendicular_to_the_nose_and_in_plane(self):
        normal = vec.cross(self.r, self.v)
        for aim in [(-0.1, 0.0, -1.0), (0.7, 0.0, -0.7), (0.2, 0.0, 0.98)]:
            up = guidance.roll_reference(self.r, self.v, aim)
            self.assertAlmostEqual(vec.norm(up), 1.0, places=9)
            self.assertAlmostEqual(vec.dot(up, vec.unit(aim)), 0.0, places=9,
                                   msg="roof not perpendicular to the nose")
            self.assertAlmostEqual(vec.dot(up, vec.unit(normal)), 0.0, places=9,
                                   msg="roof left the trajectory plane")

    def test_it_picks_the_side_away_from_the_body(self):
        up = guidance.roll_reference(self.r, self.v, (0.0, 0.0, -1.0))
        self.assertGreater(vec.dot(up, vec.unit(self.r)), 0.0)

    def test_it_does_not_flip_through_the_vertical(self):
        """Straight up is where the autopilot's own reference gives out.

        The landing burn points the nose almost exactly at the zenith, so the
        "which side is up" test is a coin toss there.  Last tick's answer
        breaks the tie; without it the reference snaps 180 deg and the
        booster is commanded to barrel-roll on final.
        """
        previous = None
        worst = 0.0
        for i in range(41):                      # nose sweeping through +z up
            tilt = math.radians(20.0 - i)
            aim = (math.cos(tilt), 0.0, math.sin(tilt))
            up = guidance.roll_reference(self.r, self.v, aim, previous)
            if previous is not None:
                worst = max(worst, vec.angle_between(up, previous))
            previous = up
        self.assertLess(worst, 5.0,
                        "reference jumped %.0f deg between ticks" % worst)

    def test_degenerate_geometry_keeps_the_last_reference(self):
        up = guidance.roll_reference(self.r, self.v, (0.0, 1.0, 0.0),
                                     (0.0, 0.0, 1.0))
        self.assertEqual(up, (0.0, 0.0, 1.0))


class TestFlipRouting(unittest.TestCase):
    """A big turn is taken over the top, not nose-down under it."""

    ZENITH = (1.0, 0.0, 0.0)

    def route(self, current, target, min_turn=90.0):
        return guidance.flip_waypoint(current, target, self.ZENITH, min_turn)

    def test_a_small_turn_is_commanded_directly(self):
        target = (0.5, 0.0, 0.87)
        self.assertEqual(self.route((0.7, 0.0, 0.71), target), vec.unit(target))

    def test_a_big_turn_that_already_climbs_is_commanded_directly(self):
        # Nose well above the horizon, target on it: the direct arc stays up.
        target = (0.0, 0.0, -1.0)
        self.assertEqual(self.route((0.6, 0.0, 0.8), target), vec.unit(target))

    def test_a_big_turn_that_would_dive_is_routed_over_the_top(self):
        """Both endpoints below the horizon; the short arc goes under."""
        waypoint = self.route((-0.3, 0.0, 0.95), (-0.3, 0.0, -0.95))
        self.assertGreater(vec.dot(waypoint, self.ZENITH), 0.0,
                           "commanded the nose down through the airstream")

    def test_an_exactly_opposite_turn_climbs(self):
        waypoint = self.route((-0.2, 0.0, 0.98), (0.2, 0.0, -0.98))
        self.assertAlmostEqual(vec.norm(waypoint), 1.0, places=9)
        self.assertGreater(vec.dot(waypoint, self.ZENITH), 0.9,
                           "did not head for the vertical")

    def test_the_waypoint_is_on_the_same_arc_as_the_two_vectors(self):
        """Routing changes which way round, never which plane.

        Going over the top must not become a detour out of the trajectory
        plane -- the extra travel is 20 deg on a 170 deg flip only because
        the waypoint stays on the great circle through both vectors.
        """
        nose, target = (-0.3, 0.0, 0.95), (-0.3, 0.0, -0.95)
        normal = vec.unit(vec.cross(nose, target))
        self.assertAlmostEqual(vec.dot(self.route(nose, target), normal), 0.0,
                               places=9)

    def test_it_releases_itself_as_the_nose_comes_round(self):
        """Fed its own waypoint, it eventually commands the target.

        The routing has no state: it re-decides from the vehicle's real
        attitude every tick, so it has to stop steering at the waypoint on
        its own once the rest of the turn is unambiguous.
        """
        target = (-0.3, 0.0, -0.95)
        nose = (-0.3, 0.0, 0.95)
        for _ in range(20):
            commanded = self.route(nose, target)
            nose = commanded          # a vehicle that turns instantly
        self.assertEqual(commanded, vec.unit(target))


class TestRotationOnto(unittest.TestCase):
    """Turning a vessel onto a direction without assuming a convention.

    kRPC's frames are left-handed and its quaternions compose in an order
    this code does not get to assume, so ``vec.rotation_onto`` checks its own
    answer and returns ``None`` when it cannot -- the aerodynamic probe then
    falls back to the real attitude rather than measuring a fiction.
    """

    def cases(self):
        random.seed(20250908)
        for _ in range(400):
            q = [random.gauss(0.0, 1.0) for _ in range(4)]
            n = math.sqrt(sum(x * x for x in q))
            yield (tuple(x / n for x in q),
                   vec.unit(tuple(random.gauss(0.0, 1.0) for _ in range(3))),
                   vec.unit(tuple(random.gauss(0.0, 1.0) for _ in range(3))))

    def test_it_puts_the_source_direction_on_the_target(self):
        for rotation, source, target in self.cases():
            turned = vec.rotation_onto(rotation, source, target)
            self.assertIsNotNone(turned, "gave up on an ordinary rotation")
            # The body-frame direction that pointed along source has to end
            # up along target; that is the whole contract.
            local = vec.quat_rotate(vec.quat_conjugate(rotation), source)
            landed = vec.quat_rotate(turned, local)
            self.assertLess(vec.norm(vec.sub(landed, target)), 0.02,
                            "landed on %s, wanted %s" % (landed, target))

    def test_it_takes_the_short_way_round(self):
        """The probe must not fly the vessel through a different attitude."""
        for rotation, source, target in self.cases():
            turned = vec.rotation_onto(rotation, source, target)
            swept = 2.0 * math.degrees(
                math.acos(min(1.0, abs(vec.dot(turned[:3], rotation[:3])
                                       + turned[3] * rotation[3]))))
            self.assertLessEqual(
                round(swept, 3),
                round(vec.angle_between(source, target), 3) + 1e-3,
                "turned %.1f deg to close a %.1f deg gap"
                % (swept, vec.angle_between(source, target)))

    def test_a_vessel_already_pointing_there_is_left_alone(self):
        rotation = (0.0, 0.0, 0.0, 1.0)
        source = (0.0, 0.0, 1.0)
        self.assertEqual(vec.rotation_onto(rotation, source, source),
                         rotation)

    def test_it_handles_an_exactly_opposed_target(self):
        """Retrograde is 180 deg from prograde, and the axis is degenerate."""
        rotation = (0.0, 0.0, 0.0, 1.0)
        source = (0.0, 0.0, 1.0)
        turned = vec.rotation_onto(rotation, source, (0.0, 0.0, -1.0))
        self.assertIsNotNone(turned)
        local = vec.quat_rotate(vec.quat_conjugate(rotation), source)
        landed = vec.quat_rotate(turned, local)
        self.assertLess(vec.norm(vec.sub(landed, (0.0, 0.0, -1.0))), 0.02)


class TestGuidance(unittest.TestCase):
    def test_boostback_burns_against_the_miss(self):
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(30000.0, (0.0, 0.0, 500.0))
        prediction = trajectory.predict_landing(env, r, v, 4e4, 0.0, cfg)
        solution = guidance.boostback_solution(env, r, v, 4e4, 20.0, 280.0,
                                               prediction, cfg)
        miss_direction = vec.unit(vec.project_out(solution.miss, vec.unit(r)))
        self.assertLess(vec.dot(solution.aim, miss_direction), -0.99)
        self.assertGreater(solution.dv, 0.0)
        self.assertGreater(solution.burn_time, 0.0)
        self.assertEqual(solution.throttle, 1.0)   # far away: full throttle

    def test_boostback_tapers_when_nearly_solved(self):
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(30000.0, (0.0, 0.0, 500.0))
        prediction = trajectory.predict_landing(env, r, v, 4e4, 0.0, cfg)
        prediction.time_to_land = 1e5          # so dv/tick is tiny
        solution = guidance.boostback_solution(env, r, v, 4e4, 20.0, 280.0,
                                               prediction, cfg)
        self.assertLess(solution.throttle, 1.0)
        self.assertGreaterEqual(solution.throttle, cfg.BOOSTBACK_MIN_THROTTLE)

    def test_landing_attitude_is_mostly_retrograde(self):
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(2000.0, (0.0, 0.0, -50.0))
        v = (-200.0, 0.0, -50.0)
        prediction = trajectory.predict_landing(env, r, v, 4e4, 25.0, cfg)
        aim = guidance.landing_attitude(env, r, v, prediction, cfg)
        retrograde = vec.unit(vec.scale(v, -1.0))
        self.assertLessEqual(vec.angle_between(aim, retrograde),
                             cfg.LANDING_MAX_TILT_DEG + 1e-6)

    def test_the_propagator_and_the_control_loop_share_one_throttle_law(self):
        """``guidance.landing_throttle`` is a wrapper, not a second opinion.

        The propagator integrates the landing burn against
        ``trajectory.landing_command``; if the control loop ever flew a
        different profile, every prediction would be of a burn that never
        happens.
        """
        env = FakeEnv()
        cfg = Config()
        r, _ = state_above_pad(800.0, (0.0, 0.0, 0.0))
        v = (-120.0, 0.0, 15.0)
        command = guidance.landing_throttle(env, r, v, 25.0, cfg)
        throttle, target = trajectory.landing_command(
            env, vec.norm(r), vec.norm(v), vec.norm(r) - env.target_radius,
            25.0, cfg)
        self.assertEqual((command.throttle, command.target_speed),
                         (throttle, target))

    def test_the_predicted_burn_drifts_further_than_a_full_throttle_stop(self):
        """The burn starts early, so it is gentler, so it drifts further.

        The closed form this replaced took ``speed / net`` -- the time a
        full-throttle stop would need -- and under-stated the downrange creep
        badly on a fast entry.  Anything predicting less drift than the
        vertical-only bound is predicting a burn the vehicle will not fly.
        """
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(20000.0, (0.0, 0.0, 300.0))
        v = (-700.0, 0.0, 300.0)                 # falling fast, still moving
        prediction = trajectory.predict_landing(env, r, v, 4e4, 25.0, cfg)

        self.assertTrue(prediction.powered, "never reached the landing burn")
        ballistic = trajectory.predict_landing(env, r, v, 4e4, 0.0, cfg)
        powered_miss = trajectory.surface_distance(
            env, prediction.position, env.target)
        ballistic_miss = trajectory.surface_distance(
            env, ballistic.position, env.target)
        self.assertLess(powered_miss, ballistic_miss,
                        "braking has to land the booster short of ballistic")
        self.assertGreater(prediction.burn_time, 0.0)

    def test_landing_throttle_tracks_the_profile(self):
        env = FakeEnv()
        cfg = Config()
        r, _ = state_above_pad(1000.0)
        slow = guidance.landing_throttle(env, r, (-5.0, 0.0, 0.0), 25.0, cfg)
        fast = guidance.landing_throttle(env, r, (-400.0, 0.0, 0.0), 25.0, cfg)
        self.assertEqual(slow.throttle, 0.0)                  # under profile
        self.assertEqual(fast.throttle, cfg.LANDING_THROTTLE_CAP)

        # The profile speed depends on height alone, and collapses to the
        # touchdown speed at the cutoff altitude.
        low, _ = state_above_pad(cfg.TOUCHDOWN_ALT_M)
        near_pad = guidance.landing_throttle(env, low, (-5.0, 0.0, 0.0), 25.0, cfg)
        self.assertGreater(fast.target_speed, near_pad.target_speed)
        self.assertAlmostEqual(near_pad.target_speed, cfg.TOUCHDOWN_SPEED)

    def test_burn_trigger_starts_earlier_when_the_miss_is_large(self):
        """A late, saturated burn has no time to translate sideways.

        The trigger buys that time back by lighting the engines higher in
        proportion to how far the prediction is from the pad.  The lead is off
        by default -- the landing burn flies straight retrograde and has
        nowhere to translate to -- so this sets it to exercise the mechanism.
        """
        env = FakeEnv()
        cfg = Config()
        cfg.LANDING_DIVERT_LEAD = 1.0
        r, v = state_above_pad(5000.0), (-250.0, 0.0, 0.0)
        _, on_target, _ = trajectory.landing_burn_state(
            env, r[0], v, 25.0, cfg, miss=0.0)
        _, off_target, _ = trajectory.landing_burn_state(
            env, r[0], v, 25.0, cfg, miss=1500.0)
        self.assertGreater(off_target, on_target)
        self.assertAlmostEqual(off_target - on_target,
                               cfg.LANDING_MAX_EARLY_M, places=6)

    def test_burn_trigger_has_no_divert_lead_by_default(self):
        """With no divert to make time for, the phase starts when it burns."""
        env = FakeEnv()
        cfg = Config()
        r, v = state_above_pad(5000.0), (-250.0, 0.0, 0.0)
        _, on_target, _ = trajectory.landing_burn_state(
            env, r[0], v, 25.0, cfg, miss=0.0)
        _, off_target, _ = trajectory.landing_burn_state(
            env, r[0], v, 25.0, cfg, miss=1500.0)
        self.assertAlmostEqual(off_target, on_target)

    def test_the_landing_burn_kills_lateral_speed_it_cannot_null_late(self):
        """The half LOG7's divert was missing.

        LOG7 tilted on the miss alone, closed 36 m of 1190, and arrived doing
        10 m/s sideways because nothing in the law was ever going to take that
        velocity back out (failure 5).  With the offset already zero the only
        thing left to correct is the lateral speed, so the command must lean
        *against* it -- a position-only law would sit exactly on retrograde
        here and let the booster drift off the pad.
        """
        env = FakeEnv()
        cfg = Config()
        r = state_above_pad(500.0)[0]
        drift = 25.0
        v = (-200.0, 0.0, drift)
        aim = guidance.terminal_attitude(env, r, v, 500.0, 8.0, 25.0, cfg)
        up = vec.unit(r)
        # The sideways part of the command opposes the sideways motion.
        sideways = vec.project_out(aim, up)
        self.assertLess(vec.dot(sideways, (0.0, 0.0, drift)), 0.0)
        self.assertLessEqual(
            vec.angle_between(aim, vec.unit(vec.scale(v, -1.0))),
            cfg.LANDING_MAX_TILT_DEG + 1e-6)

    def test_the_landing_burn_stops_steering_before_it_lands(self):
        """Both gains diverge at touchdown; neither may reach the vehicle.

        An unbounded lateral command in the last second is LOG7's tipped
        booster by another route, so the law stops steering below
        LANDING_DIVERT_MIN_ALT_M and the floor under t_go holds everywhere
        above it.
        """
        env = FakeEnv()
        cfg = Config()
        r = state_above_pad(cfg.LANDING_DIVERT_MIN_ALT_M - 1.0)[0]
        v = (-20.0, 0.0, 5.0)
        aim = guidance.terminal_attitude(
            env, r, v, cfg.LANDING_DIVERT_MIN_ALT_M - 1.0, 0.0, 25.0, cfg)
        self.assertLess(
            vec.angle_between(aim, vec.unit(vec.scale(v, -1.0))), 1e-6)
        # And well above it, a zero time-to-go must not produce a wild command.
        high = guidance.terminal_attitude(env, state_above_pad(2000.0)[0],
                                          v, 2000.0, 0.0, 25.0, cfg)
        self.assertLessEqual(
            vec.angle_between(high, vec.unit(vec.scale(v, -1.0))),
            cfg.LANDING_MAX_TILT_DEG + 1e-6)

    def test_terminal_guidance_can_be_switched_off(self):
        """LANDING_TERMINAL_GUIDANCE=False is the pure-retrograde burn."""
        env = FakeEnv()
        cfg = Config()
        cfg.LANDING_TERMINAL_GUIDANCE = False
        r = state_above_pad(3000.0)[0]
        v = (-200.0, 0.0, 30.0)
        aim = guidance.terminal_attitude(env, r, v, 3000.0, 10.0, 25.0, cfg)
        self.assertLess(
            vec.angle_between(aim, vec.unit(vec.scale(v, -1.0))), 1e-6)

    def test_burn_trigger_uses_the_height_it_is_given(self):
        """Terrain under the booster, not the pad's radius, is what it hits."""
        env = FakeEnv()
        cfg = Config()
        r, _ = state_above_pad(5000.0)
        default_h, _, _ = trajectory.landing_burn_state(
            env, r, (-100.0, 0.0, 0.0), 25.0, cfg)
        terrain_h, _, _ = trajectory.landing_burn_state(
            env, r, (-100.0, 0.0, 0.0), 25.0, cfg, height=1200.0)
        self.assertAlmostEqual(default_h, 5000.0, delta=1.0)
        self.assertEqual(terrain_h, 1200.0)

    def test_burn_trigger_height_grows_with_speed(self):
        env = FakeEnv()
        cfg = Config()
        r, _ = state_above_pad(5000.0)
        _, slow_needed, _ = trajectory.landing_burn_state(
            env, r, (-100.0, 0.0, 0.0), 25.0, cfg)
        _, fast_needed, _ = trajectory.landing_burn_state(
            env, r, (-300.0, 0.0, 0.0), 25.0, cfg)
        self.assertGreater(fast_needed, slow_needed)


class TestLogbook(unittest.TestCase):
    def test_concurrent_logbooks_never_share_a_file(self):
        """Four flights into one logs/ directory must get four files.

        The parallel rig runs every instance's flight into the same
        ``logs/``, and picking ``highest + 1`` and *then* opening it is a
        race: two that start together choose the same name and the second
        truncates the first, leaving one file holding two interleaved
        flights.  It is silent -- the file still looks like a log -- and it
        surfaced only as a sweep reporting one LOG number for two different
        configurations.
        """
        import threading
        with tempfile.TemporaryDirectory() as tmp:
            paths, errors = [], []
            lock = threading.Lock()
            start = threading.Barrier(8)

            def claim():
                try:
                    start.wait()
                    log = Logbook(tmp, 1.0)
                    log.event(0.0, "hello")
                    log.close()
                    with lock:
                        paths.append(log.path)
                except Exception as exc:            # pragma: no cover
                    with lock:
                        errors.append(exc)

            threads = [threading.Thread(target=claim) for _ in range(8)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

            self.assertEqual(errors, [])
            self.assertEqual(len(paths), 8)
            self.assertEqual(len(set(paths)), 8, "two logbooks shared a file")
            for path in paths:
                with open(path) as fh:
                    self.assertEqual(fh.read().count("hello"), 1)

    def test_interval_is_gated_on_game_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            with Logbook(tmp, 2.0) as log:
                self.assertTrue(log.telemetry(100.0, "a"))
                self.assertFalse(log.telemetry(101.0, "b"))   # paused/too soon
                self.assertTrue(log.telemetry(102.0, "c"))
                self.assertTrue(log.telemetry(500.0, "d"))    # after a warp
                log.event(500.0, "always")
                path = log.path
            body = open(path).read()
            self.assertNotIn(" b\n", body)
            self.assertIn("always", body)

    def test_log_files_are_numbered_in_sequence(self):
        with tempfile.TemporaryDirectory() as tmp:
            first, second = Logbook(tmp, 1.0), Logbook(tmp, 1.0)
            first.close()
            second.close()
            self.assertTrue(first.path.endswith("LOG1"))
            self.assertTrue(second.path.endswith("LOG2"))


class TestClearance(unittest.TestCase):
    """The scan that decides when the flip is safe.

    Pure geometry: two bounding spheres, a relative velocity, and the
    acceleration the boostback burn is about to apply.  Everything that needs
    the game lives in ``ProximityScan``.
    """

    def setUp(self):
        self.cfg = Config()
        self.here = (KERBIN_R + 20000.0, 0.0, 0.0)
        self.velocity = (300.0, 0.0, 600.0)

    def stage(self, offset, drift=(0.0, 0.0, 0.0), radius=10.0):
        return proximity.Neighbour(
            name="Upper Stage",
            position=vec.add(self.here, offset),
            velocity=vec.add(self.velocity, drift),
            radius=radius)

    def check(self, neighbour, accel=(0.0, 0.0, 0.0), radius=11.0,
              horizon=None):
        return proximity.conflict(
            self.here, self.velocity, accel, radius, [neighbour], self.cfg,
            horizon if horizon is not None else self.cfg.CLEARANCE_HORIZON_S)

    def test_a_stage_it_is_sitting_next_to_intact_does_not_block_the_flip(self):
        """The LOG16 clamp, which is the whole model.

        Two craft that were one craft a second ago always have their centres
        closer together than the sum of their bounding spheres, so the sphere
        model is violated at t=0 by construction and stays violated however
        fast they separate.  LOG16 read a 44.5 m radius off a stage 19 m away
        and could never clear it: SEPARATION held the attitude at Mach 2.4
        with the grid fins out until the flight was terminated, and the
        vehicle lost an airbrake to the airstream.  If the craft are intact
        and not touching, that distance is survivable whatever the boxes say.
        """
        stage = self.stage((0.0, 0.0, 12.0), radius=40.0)
        self.assertGreater(11.0 + stage.radius, 12.0,
                           "the spheres have to overlap for this to prove "
                           "anything")
        self.assertIsNone(self.check(stage))

    def test_but_getting_closer_than_that_still_does(self):
        found = self.check(self.stage((0.0, 0.0, 12.0), drift=(0.0, 0.0, -2.0),
                                      radius=40.0))
        self.assertIsNotNone(found, "closing on a stage 12 m away is fine?")
        self.assertLess(found.separation, 0.0)

    def test_a_stage_that_has_already_gone_does_not(self):
        self.assertIsNone(self.check(self.stage((0.0, 0.0, 200.0))))

    def test_a_stage_it_is_drifting_onto_blocks_it_early(self):
        """Clear right now, but not for the next ten seconds."""
        found = self.check(self.stage((0.0, 0.0, 60.0), drift=(0.0, 0.0, -10.0)))
        self.assertIsNotNone(found)
        self.assertGreater(found.time, 0.0, "should be a future conflict")

    def test_the_burn_is_what_makes_the_difference(self):
        """The acceleration is not decoration.

        A stage 60 m off to the side is a clean miss for a coasting booster
        and a collision for one that lights its engines pointed at it.  The
        flip and the burn are the same manoeuvre, so the scan has to see the
        burn -- at 20 m/s^2 the booster covers that 60 m in under three
        seconds.
        """
        stage = self.stage((0.0, 0.0, 60.0))
        burn = self.cfg.CLEARANCE_BURN_HORIZON_S
        self.assertIsNone(self.check(stage, horizon=burn),
                          "coasting should miss it")
        toward = (0.0, 0.0, 20.0)
        self.assertIsNotNone(self.check(stage, accel=toward, horizon=burn),
                             "burning straight at it should not")
        away = (0.0, 0.0, -20.0)
        self.assertIsNone(self.check(stage, accel=away, horizon=burn))

    def test_a_broken_bounding_box_is_not_believed(self):
        """The LOG4 corner, on someone else's vessel this time.

        A radius of 7e17 m would make every craft in the game a conflict and
        the booster would never flip, so an unusable box has to read as
        "unknown" and fall back, exactly as the leg clearance does.
        """
        class Broken:
            reference_frame = None

            def bounding_box(self, frame):
                return ((-1.8, -6.998429e17, -1.8), (1.8, 11.0, 1.8))

        class Fine:
            reference_frame = None

            def bounding_box(self, frame):
                return ((-1.8, -11.0, -1.8), (1.8, 11.0, 1.8))

        self.assertIsNone(proximity.bounding_radius(Broken(), self.cfg))
        self.assertAlmostEqual(proximity.bounding_radius(Fine(), self.cfg),
                               vec.norm((1.8, 11.0, 1.8)), places=6)


class TestLandingHeight(unittest.TestCase):
    """The height the engines are cut on, which has now cost four flights."""

    def snap(self, height_above_pad, surface_altitude, clearance=10.0,
             pad_above_datum=72.5):
        return Snapshot(
            ut=0.0, position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0),
            direction=(1.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0),
            mass=1000.0, available_thrust=0.0, max_thrust=0.0, isp=300.0,
            surface_altitude=surface_altitude, vertical_speed=0.0,
            horizontal_speed=0.0, throttle=0.0, situation=None,
            height_above_pad=height_above_pad, mean_altitude=height_above_pad,
            leg_clearance=clearance, pad_above_datum=pad_above_datum)

    def test_ground_below_the_pad_radius_is_believed(self):
        """LOG2/LOG3: cut the engines 48 m up, over terrain lower than the pad.

        Both flights came down ~1.6 km out where the ground sits 58 m below
        the pad's radius.  The pad-relative height reached zero with the legs
        still 48 m in the air, the old ``min()`` took it, and the booster was
        dropped from there after a burn that had flown it to vs=-2.0.
        """
        snap = self.snap(height_above_pad=1.0, surface_altitude=58.6)
        self.assertAlmostEqual(snap.legs_altitude, 48.6)
        self.assertAlmostEqual(snap.landing_height, 48.6)

    def test_ground_above_the_pad_radius_is_believed_too(self):
        """The other side, which the old min() was right about: a hill."""
        snap = self.snap(height_above_pad=500.0, surface_altitude=210.0)
        self.assertAlmostEqual(snap.landing_height, 200.0)

    def test_a_broken_box_cannot_drive_the_height_negative(self):
        """LOG4/5/6: bounding_box returned -7e17 and every height went with it."""
        snap = self.snap(height_above_pad=20000.0, surface_altitude=-7e17)
        self.assertAlmostEqual(snap.landing_height, 20000.0 - 7000.0)

    def test_a_broken_sensor_cannot_hold_the_height_up(self):
        """The ceiling: no terrain is 1e17 m below the pad, so do not believe it."""
        snap = self.snap(height_above_pad=100.0, surface_altitude=7e17)
        self.assertAlmostEqual(snap.landing_height, 100.0 + 72.5 + 100.0)


class TestDescentAwareDragProbe(unittest.TestCase):
    """The two questions a Cd*A probe has to get right: what attitude, and
    what air.  Both are properties of where the booster is *going*, not of
    where it is, and both cost hundreds of metres when they are wrong -- see
    CLAUDE.md failure 13."""

    def make_env(self):
        env = environment.Environment.__new__(environment.Environment)
        env.cfg = Config()
        env.equatorial_radius = 600000.0
        env._descent_profile = ()
        return env

    def test_descent_configuration_is_measured_from_retrograde(self):
        env = self.make_env()
        env.cfg.DRAG_RESWEEP_AOA_DEG = 25.0
        v = (0.0, -100.0, 0.0)                  # falling
        nose_up = (0.0, 1.0, 0.0)               # engines into the flow
        nose_down = (0.0, -1.0, 0.0)
        self.assertTrue(env.in_descent_configuration(v, nose_up))
        self.assertFalse(env.in_descent_configuration(v, nose_down))
        # Broadside is the boostback attitude, and the whole point is that it
        # does not count.
        self.assertFalse(env.in_descent_configuration(v, (1.0, 0.0, 0.0)))

    def test_the_resweep_can_be_switched_off(self):
        env = self.make_env()
        env.cfg.DRAG_RESWEEP_AOA_DEG = 0.0
        self.assertFalse(env.in_descent_configuration((0.0, -100.0, 0.0),
                                                      (0.0, 1.0, 0.0)))

    def test_a_bin_is_probed_where_the_descent_passes_that_speed(self):
        env = self.make_env()
        env.cfg.DRAG_PROBE_DESCENT_ALTITUDE = True
        # Thin high, thick low -- and the descent passes 300 m/s twice.
        env.set_descent_profile(((300.0, 28000.0), (420.0, 20000.0),
                                 (300.0, 4000.0), (120.0, 900.0)))
        # Drag goes as the density, so the deep pass is where the bin does
        # essentially all of its work; taking the arithmetically nearest
        # sample would pick 28 km and measure air the booster barely touches.
        self.assertEqual(env.probe_altitude(300.0, 31000.0), 4000.0)
        self.assertEqual(env.probe_altitude(120.0, 31000.0), 900.0)

    def test_a_speed_the_descent_never_reaches_falls_back(self):
        env = self.make_env()
        env.cfg.DRAG_PROBE_DESCENT_ALTITUDE = True
        env.set_descent_profile(((300.0, 4000.0),))
        # Mach 3 on the way up is not part of the descent at all: better the
        # air the booster is in than air invented for it.
        self.assertEqual(env.probe_altitude(1200.0, 31000.0), 31000.0)

    def test_no_profile_and_the_flag_off_both_use_the_current_altitude(self):
        env = self.make_env()
        env.cfg.DRAG_PROBE_DESCENT_ALTITUDE = True
        self.assertEqual(env.probe_altitude(300.0, 31000.0), 31000.0)
        env.cfg.DRAG_PROBE_DESCENT_ALTITUDE = False
        env.set_descent_profile(((300.0, 4000.0),))
        self.assertEqual(env.probe_altitude(300.0, 31000.0), 31000.0)

    def test_the_probe_position_keeps_the_place_and_changes_the_air(self):
        env = self.make_env()
        r = (600000.0 + 30000.0, 0.0, 0.0)
        moved = env.probe_position(r, 4000.0)
        self.assertAlmostEqual(vec.norm(moved), 604000.0, places=3)
        # Same direction from the centre: the probe asks about this longitude
        # and latitude, lower down, not about somewhere else entirely.
        self.assertAlmostEqual(vec.dot(vec.unit(moved), vec.unit(r)), 1.0,
                               places=9)

    def test_the_prediction_reports_the_descent_it_flew(self):
        env = FakeEnv(atmosphere=True)
        env.drag_area = 15.0
        cfg = Config()
        r, v = state_above_pad(30000.0, (0.0, 0.0, 400.0))
        pred = trajectory.predict_landing(env, r, v, 20000.0, 20.0, cfg)
        self.assertTrue(pred.profile, "no descent profile recorded")
        # Only the descent: a profile that included the climb would offer the
        # probe two altitudes for the same speed with no way to tell which.
        alts = [a for _, a in pred.profile]
        self.assertEqual(alts, sorted(alts, reverse=True))


class TestGradientProbe(unittest.TestCase):
    """The gradient decides whether a correction burn happens at all, and it
    was measured lying at low altitude -- see trajectory.miss_gradient."""

    def test_the_probe_can_fly_the_landing_burn(self):
        env = FakeEnv(atmosphere=True)
        env.drag_area = 15.0
        cfg = Config()
        r, v = state_above_pad(9000.0, (0.0, 0.0, 300.0))
        aim = vec.unit(vec.scale(v, -1.0))
        ballistic = trajectory.miss_gradient(env, r, v, 20000.0, aim, cfg,
                                             cfg.CORRECTION_PROBE_DV)
        powered = trajectory.miss_gradient(env, r, v, 20000.0, aim, cfg,
                                           cfg.CORRECTION_PROBE_DV,
                                           max_accel=20.0)
        # Low down, nearly all of the remaining flight is the burn, so the two
        # answers are not the same question.  Which is the point: the
        # ballistic one measures a trajectory the vehicle will never fly.
        self.assertNotAlmostEqual(ballistic, powered, places=2)

    def test_the_default_is_still_ballistic(self):
        env = FakeEnv(atmosphere=True)
        env.drag_area = 15.0
        cfg = Config()
        r, v = state_above_pad(9000.0, (0.0, 0.0, 300.0))
        aim = vec.unit(vec.scale(v, -1.0))
        self.assertAlmostEqual(
            trajectory.miss_gradient(env, r, v, 20000.0, aim, cfg, 5.0),
            trajectory.miss_gradient(env, r, v, 20000.0, aim, cfg, 5.0,
                                     max_accel=0.0), places=9)


class TestTerminalOnPrediction(unittest.TestCase):
    """Steering the landing burn on the propagator's answer rather than on a
    straight line drawn through the current state."""

    def setUp(self):
        self.env = FakeEnv(atmosphere=True)
        self.cfg = Config()
        self.cfg.LANDING_TERMINAL_GUIDANCE = True

    def test_it_tilts_toward_the_pad_when_the_prediction_overshoots(self):
        # Falling fast with a little downrange speed left, which is what the
        # landing burn actually looks like.  A purely horizontal velocity has
        # no tilt available in the plane of the miss and is the wrong state to
        # ask this question in.
        r, v = state_above_pad(1000.0, (-200.0, 0.0, 10.0))
        # Predicted to land 200 m along +z of the pad: the command must lean
        # back the other way.
        zem = (0.0, 0.0, 200.0)
        cmd = guidance.terminal_command(self.env, r, v, 1000.0, 12.0, 15.0,
                                        self.cfg, zem)
        retrograde = vec.unit(vec.scale(v, -1.0))
        lean = vec.sub(cmd.direction, vec.scale(retrograde,
                                                vec.dot(cmd.direction, retrograde)))
        self.assertLess(lean[2], 0.0, "leaned toward the overshoot, not away")
        self.assertGreater(cmd.demand, 0.0)

    def test_a_prediction_on_the_pad_asks_for_no_tilt(self):
        r, v = state_above_pad(1000.0, (-200.0, 0.0, 10.0))
        cmd = guidance.terminal_command(self.env, r, v, 1000.0, 12.0, 15.0,
                                        self.cfg, (0.0, 0.0, 0.0))
        retrograde = vec.unit(vec.scale(v, -1.0))
        self.assertAlmostEqual(vec.angle_between(cmd.direction, retrograde),
                               0.0, places=6)

    def test_the_throttle_budget_closes_the_tilt_at_the_cap(self):
        r, v = state_above_pad(1000.0, (-200.0, 0.0, 10.0))
        retrograde = vec.unit(vec.scale(v, -1.0))
        # A big miss and plenty of thrust: unbounded, this leans hard.
        loose = guidance.terminal_command(self.env, r, v, 1000.0, 12.0, 15.0,
                                          self.cfg, (0.0, 0.0, 400.0), 20.0)
        self.assertGreater(vec.angle_between(loose.direction, retrograde), 1.0)
        # Same demand with no throttle to spare: the braking wins.
        tight = guidance.terminal_command(self.env, r, v, 1000.0, 12.0, 15.0,
                                          self.cfg, (0.0, 0.0, 400.0), 0.0)
        self.assertAlmostEqual(vec.angle_between(tight.direction, retrograde),
                               0.0, places=6)

    def test_the_tilt_is_bounded(self):
        r, v = state_above_pad(1000.0, (-200.0, 0.0, 10.0))
        self.cfg.LANDING_MAX_TILT_DEG = 15.0
        cmd = guidance.terminal_command(self.env, r, v, 1000.0, 3.0, 5.0,
                                        self.cfg, (0.0, 0.0, 9000.0))
        retrograde = vec.unit(vec.scale(v, -1.0))
        self.assertLessEqual(vec.angle_between(cmd.direction, retrograde),
                             15.0 + 1e-6)


class TestNoGoAround(unittest.TestCase):
    """The landing burn must never fly the booster back into the sky.

    LOG429: a clean final approach at 11 m and 0.22 throttle went to 0.95 and
    left the ground at 101 m/s, coasted to 951 m, fell back, burned again and
    landed 1106 m out having spent 3.8 t -- from a burn that was 132 m out
    when it started.  ``span`` is floored at a metre, so a height reading near
    zero with several m/s still on the clock asks for ``(speed^2 - 4) / 2`` of
    deceleration, and a nearly empty booster has 3.5 g with which to deliver
    it.
    """

    def setUp(self):
        self.env = FakeEnv()
        self.cfg = Config()

    def command(self, height, speed, vertical_speed):
        return trajectory.landing_command(
            self.env, KERBIN_R + height, speed, height, 34.0, self.cfg,
            vertical_speed)[0]

    def test_a_bad_height_cannot_light_the_engine_under_an_ascent(self):
        # The LOG429 tick: a height that reads as good as on the ground while
        # the vehicle is actually moving, and already going up.
        self.assertEqual(self.command(0.3, 7.4, +1.0), 0.0)

    def test_it_cuts_the_moment_the_descent_is_arrested(self):
        low = self.cfg.LANDING_GOAROUND_ALT_M - 10.0
        self.assertEqual(self.command(low, 5.0, 0.0), 0.0)
        self.assertEqual(self.command(low, 35.0, +2.0), 0.0)

    def test_an_ordinary_descent_is_untouched(self):
        """The guard must not clip a real flare -- only an ascent."""
        for height, speed in ((40.0, 20.0), (10.0, 7.0), (3.0, 3.0)):
            self.assertGreater(self.command(height, speed, -speed), 0.0)

    def test_it_does_not_reach_up_where_a_burn_is_still_being_flown(self):
        """High up, an ascending vehicle is a different situation entirely."""
        # Fast enough to be *above* the profile, so the throttle is
        # genuinely open and the guard is the only thing that could shut it.
        high = self.cfg.LANDING_GOAROUND_ALT_M + 500.0
        self.assertGreater(self.command(high, 220.0, +5.0), 0.0)

    def test_a_slow_arrival_a_metre_up_gets_a_hover_and_no_more(self):
        """LOG446's tick: 1.6 m, descending 3.5 m/s, and it asked for 0.95.

        The vehicle had been flying the whole approach at 0.30 -- which is
        hover for it -- so the demand was not a late correction, it was the
        span floor.  A 3.5 g booster does not land under full throttle; it
        tips, and the lit engine takes it sideways at 30 m/s.
        """
        hover = (self.env.mu / (KERBIN_R ** 2)) / 34.0
        throttle = self.command(1.6, 5.3, -3.5)
        self.assertLessEqual(throttle, hover + self.cfg.LANDING_FLARE_MARGIN + 1e-9)
        self.assertGreater(throttle, 0.0, "it still has to hold itself up")

    def test_a_fast_arrival_low_down_still_gets_everything(self):
        """The cap is for a vehicle that cannot want the thrust, not for one
        that is late -- that one needs all of it and the cap is not what is
        wrong with it."""
        fast = self.cfg.LANDING_FLARE_SPEED + 30.0
        self.assertGreater(self.command(10.0, fast, -fast), 0.5)

    def test_the_propagator_and_the_control_loop_share_the_guard(self):
        """Both callers pass the vertical speed, so neither flies a law the
        other does not -- the same rule as landing_burn_state."""
        r = (KERBIN_R + 20.0, 0.0, 0.0)
        v = (3.0, 0.0, 1.0)               # ascending
        cmd = guidance.landing_throttle(self.env, r, v, 34.0, self.cfg,
                                        height=20.0)
        self.assertEqual(cmd.throttle, 0.0)


class TestCorrectionAltitudeWindow(unittest.TestCase):
    """CORRECTION waits for the same settled prediction the wing waits for.

    Up at the boostback exit the predicted miss is the drag curve's walk and
    not a miss (see failure 13 and the wing's AERO_STEER_MAX_ALT_M).  LOG542
    entered CORRECTION at 238 m up there and came out at 795 -- the abort
    guard caught the burn, but 795 m is more than the wing can close in the
    descent that is left, and it landed 112 m out where its sibling flights
    on the same config landed at 3 m.
    """

    def run_with(self, height):
        run = object.__new__(autoland_module.Autoland)
        run.cfg = Config()
        snap = SimpleNamespace(max_accel=18.0, landing_height=height)
        prediction = SimpleNamespace(time_to_land=120.0)
        return run, snap, prediction

    def test_it_will_not_correct_above_the_window_when_one_is_set(self):
        run, snap, prediction = self.run_with(30000.0)
        run.cfg.CORRECTION_MAX_ALT_M = 25000.0
        self.assertFalse(run.correction_worthwhile(snap, prediction, 500.0))

    def test_it_will_not_correct_below_the_window(self):
        run, snap, prediction = self.run_with(
            Config().CORRECTION_MIN_ALT_M - 1000.0)
        self.assertFalse(run.correction_worthwhile(snap, prediction, 500.0))

    def test_the_ceiling_is_off_by_default(self):
        """Because closing it was measured and is worse.

        The reasoning was symmetry: CORRECTION should wait for a settled
        prediction the way the wing does.  In game it is a clear regression --
        `qs_north` 4/5 m became 118/141/107 and `qs_cold` 3/3 became 42/58/20.
        CORRECTION's early burns take the first bite out of the ~180 m
        boostback lead even acting on a prediction that has not settled, and
        the wing cannot close that much alone: the regressed flights read
        coast entry -182 -> burn entry -120 where the good ones read -174 ->
        -16.  One backfire in ten flights is cheaper than losing the phase.
        """
        self.assertGreater(Config().CORRECTION_MAX_ALT_M, 1e6)


class TestTouchdownBackstop(unittest.TestCase):
    """A booster lying on the pad is down, however fast it is sliding.

    The backstop that stops a bad height reading cutting the engines at
    altitude (LOG5: 24 km, 386 m/s) used to test *total* speed, which cannot
    tell a booster falling out of the sky from one on the ground skidding
    along it.  LOG480 was the second: ``legs=-0.5``, ``vs=+0.5``, ``hs=38.2``,
    throttle 0.95, and the engine drove it across the pad.
    """

    def make(self, **over):
        run = object.__new__(autoland_module.Autoland)
        run.cfg = Config()
        for k, v in over.items():
            setattr(run.cfg, k, v)

        class Situations:
            landed, splashed = "landed", "splashed"

        class Conn:
            class space_center:
                VesselSituation = Situations
        run.conn = Conn
        return run

    def snap(self, speed, vertical_speed, situation="flying"):
        return SimpleNamespace(speed=speed, vertical_speed=vertical_speed,
                               situation=situation)

    def command(self, height):
        return SimpleNamespace(height=height)

    def test_a_skidding_booster_is_on_the_ground(self):
        run = self.make()
        # LOG480's tick: below the pad radius, barely moving vertically, and
        # 38 m/s of horizontal slide.
        self.assertTrue(run.touched_down(self.snap(38.2, 0.5),
                                         self.command(-0.5)))

    def test_a_booster_still_falling_fast_is_not(self):
        """LOG5: a bad terrain reading at 24 km and 386 m/s."""
        run = self.make()
        self.assertFalse(run.touched_down(self.snap(386.0, -386.0),
                                          self.command(0.0)))

    def test_an_ordinary_arrival_still_cuts(self):
        run = self.make()
        self.assertTrue(run.touched_down(self.snap(2.0, -2.0),
                                         self.command(0.1)))

    def test_the_game_saying_landed_always_wins(self):
        run = self.make()
        self.assertTrue(run.touched_down(self.snap(400.0, -400.0, "landed"),
                                         self.command(9999.0)))


class TestTwoAxisAeroSteer(unittest.TestCase):
    """The wing is the only cross-track actuator in the flight.

    `solve_steer` used to solve the downrange miss alone, because the lift
    plane was the trajectory plane.  Cross-track then had no actuator
    anywhere: CORRECTION tilts off retrograde, which is a downrange lever,
    and the landing burn has ten metres of authority.  Measured over 20
    flights of the five saves, the cross-track miss sat at 15-28 m and did
    not move from 28 km down.
    """

    def setUp(self):
        self.env = FakeEnv(atmosphere=True)
        self.env.drag_area = 15.0
        self.env.lift_slope = 140.0     # what liftprobe.py measures on the
                                        # real booster, Cl*A per rad^2
        self.cfg = Config()
        self.cfg.AERO_STEER = True
        # These tests are about the solver, not about when it is allowed to
        # run; the altitude window has its own test below.
        self.cfg.AERO_STEER_MAX_ALT_M = 1e9

    def entry(self):
        """High and fast enough that a minute of air is still to come."""
        r = (KERBIN_R + 30000.0, 0.0, 12000.0)
        v = (-250.0, 0.0, -350.0)
        return r, v

    def miss_of(self, steer, r, v):
        prediction = trajectory.predict_landing(self.env, r, v, 20000.0,
                                                18.0, self.cfg, steer=steer)
        return trajectory.miss_components(self.env, prediction.position,
                                          self.env.target, r)

    def test_it_nulls_the_cross_track_miss_as_well_as_the_downrange(self):
        r, v = self.entry()
        # Put a real cross-track error in front of it by moving the target
        # off the trajectory plane.
        self.env.target = (KERBIN_R, 900.0, 0.0)
        self.cfg.AERO_STEER_CROSS = True
        steer, prediction = guidance.solve_steer(self.env, r, v, 20000.0,
                                                 18.0, self.cfg)
        self.assertIsNotNone(steer)
        long0, cross0 = self.miss_of(None, r, v)
        long1, cross1 = self.miss_of(steer, r, v)
        self.assertLess(abs(cross1), abs(cross0),
                        "the cross-track miss was not closed at all")
        self.assertLess(abs(long1), abs(long0) + 1.0)

    def test_one_axis_leaves_the_cross_track_miss_untouched(self):
        """The old behaviour, and the control for the test above."""
        r, v = self.entry()
        self.env.target = (KERBIN_R, 900.0, 0.0)
        self.cfg.AERO_STEER_CROSS = False
        steer, _ = guidance.solve_steer(self.env, r, v, 20000.0, 18.0,
                                        self.cfg)
        _, cross0 = self.miss_of(None, r, v)
        _, cross1 = self.miss_of(steer, r, v)
        self.assertAlmostEqual(cross1, cross0, delta=max(1.0, abs(cross0) * 0.1))

    def test_the_command_stays_inside_the_angle_the_vehicle_can_hold(self):
        r, v = self.entry()
        self.env.target = (KERBIN_R, 90000.0, 40000.0)   # hopeless, on purpose
        self.cfg.AERO_STEER_CROSS = True
        steer, _ = guidance.solve_steer(self.env, r, v, 20000.0, 18.0,
                                        self.cfg)
        self.assertLessEqual(abs(math.degrees(steer.aoa)),
                             self.cfg.AERO_STEER_MAX_AOA_DEG + 1e-6)

    def test_it_stops_steering_before_the_air_runs_out(self):
        """The solve divides by a sensitivity that collapses near the ground.

        LOG388 held 3.9 deg for the whole coast with the prediction pinned at
        0-5 m, then commanded 8.7 deg on the last coast tick -- a zero miss
        divided by nothing -- and handed the landing burn 16 m it had not had
        a moment before.  An angle of attack held into the burn is lateral
        speed the burn then has to cancel, which is failure 5's shape again.
        """
        self.cfg.AERO_STEER_CROSS = True
        self.cfg.AERO_STEER_MIN_ALT_M = 2000.0      # off by default; see config
        self.env.target = (KERBIN_R, 900.0, 0.0)
        low = self.cfg.AERO_STEER_MIN_ALT_M - 100.0
        r = (KERBIN_R + low, 0.0, 3000.0)
        v = (-200.0, 0.0, -120.0)
        steer, _ = guidance.solve_steer(self.env, r, v, 20000.0, 18.0,
                                        self.cfg)
        self.assertEqual(steer.aoa, 0.0)

    def test_it_does_not_steer_against_a_miss_already_closed(self):
        r, v = self.entry()
        self.cfg.AERO_STEER_CROSS = True
        # Aim at wherever it is already going, to within the deadband.
        flat = trajectory.predict_landing(self.env, r, v, 20000.0, 18.0,
                                          self.cfg)
        self.env.target = flat.position
        steer, _ = guidance.solve_steer(self.env, r, v, 20000.0, 18.0,
                                        self.cfg)
        self.assertEqual(steer.aoa, 0.0)

    def test_a_command_that_makes_it_worse_is_refused(self):
        """The solve checks its own answer against doing nothing.

        Near the ground the measured sensitivity collapses and the inversion
        answers an already-closed miss with the maximum angle -- LOG399 went
        from a predicted 0 m to -19 m in one tick that way, and LOG402 from
        -62 to -91.  The prediction at the commanded angle is computed anyway
        to hand back to the caller, so comparing it against the do-nothing
        prediction is free.
        """
        r, v = self.entry()
        self.cfg.AERO_STEER_CROSS = True
        # A target the wing cannot reach: the solve saturates, and the
        # saturated command must still not be worse than flying straight.
        self.env.target = (KERBIN_R, 0.0, -60000.0)
        steer, prediction = guidance.solve_steer(self.env, r, v, 20000.0,
                                                 18.0, self.cfg)
        long0, cross0 = self.miss_of(None, r, v)
        long1, cross1 = self.miss_of(steer, r, v)
        self.assertLessEqual(math.hypot(long1, cross1),
                             math.hypot(long0, cross0) + 1e-6)

    def test_the_returned_prediction_is_of_the_commanded_trajectory(self):
        """The caller stores it, so it must be the arc actually being flown."""
        r, v = self.entry()
        self.env.target = (KERBIN_R, 900.0, 0.0)
        self.cfg.AERO_STEER_CROSS = True
        steer, prediction = guidance.solve_steer(self.env, r, v, 20000.0,
                                                 18.0, self.cfg)
        direct = trajectory.predict_landing(self.env, r, v, 20000.0, 18.0,
                                            self.cfg, steer=steer)
        for a, b in zip(prediction.position, direct.position):
            self.assertAlmostEqual(a, b, delta=1.0)

    def test_it_waits_until_the_prediction_has_settled(self):
        """High up the miss is failure 13's walk, not a miss.

        At the boostback exit the prediction reads about -175 m on this
        booster and an *unsteered* coast walks it to zero by itself, because
        the drag curve is still being re-swept.  Acting on it there steers
        out an error that was going to correct itself -- worth -80 to -100 m
        on the two saves that were already landing at 8 m.
        """
        self.cfg.AERO_STEER_MAX_ALT_M = 25000.0
        self.env.target = (KERBIN_R, 900.0, 0.0)
        r = (KERBIN_R + 33000.0, 0.0, 12000.0)      # a boostback exit
        v = (-250.0, 0.0, -350.0)
        steer, _ = guidance.solve_steer(self.env, r, v, 20000.0, 18.0,
                                        self.cfg)
        self.assertEqual(steer.aoa, 0.0)
        # ... and once it is down in the window, it steers.
        low = (KERBIN_R + 20000.0, 0.0, 8000.0)
        steer, _ = guidance.solve_steer(self.env, low, v, 20000.0, 18.0,
                                        self.cfg)
        self.assertGreater(abs(steer.aoa), 0.0)

    def test_a_booster_with_no_wing_is_not_steered(self):
        r, v = self.entry()
        self.env.lift_slope = 0.0
        steer, prediction = guidance.solve_steer(self.env, r, v, 20000.0,
                                                 18.0, self.cfg)
        self.assertIsNone(steer)

    def test_the_lift_stays_perpendicular_to_the_velocity_when_rolled(self):
        """Why the roll is stored as a rotated normal, not a direction.

        ``Steer`` holds its normal fixed and takes ``cross(normal, v)`` every
        step, so the lift tracks a turning velocity.  A stored lift vector
        would not, and the propagation would quietly develop a component
        along the flight path.
        """
        r, v = self.entry()
        normal = guidance.steering_plane(r, v)
        steer = guidance.steer_for(normal, v, (0.03, 0.04))
        self.assertAlmostEqual(math.hypot(0.03, 0.04), steer.aoa, places=9)
        for turned in (v, (-350.0, 0.0, 250.0), (0.0, -10.0, -400.0)):
            side = trajectory.lift_direction(steer, turned)
            self.assertAlmostEqual(vec.dot(side, vec.unit(turned)), 0.0,
                                   places=9)

    def test_rolling_the_command_rolls_the_lift(self):
        r, v = self.entry()
        normal = guidance.steering_plane(r, v)
        in_plane = trajectory.lift_direction(
            guidance.steer_for(normal, v, (0.05, 0.0)), v)
        out = trajectory.lift_direction(
            guidance.steer_for(normal, v, (0.0, 0.05)), v)
        # A quarter turn about the velocity: perpendicular to each other, and
        # both still perpendicular to the velocity.
        self.assertAlmostEqual(vec.dot(in_plane, out), 0.0, places=6)
        self.assertAlmostEqual(vec.dot(out, vec.unit(v)), 0.0, places=9)


class TestTerminalVelocityNulling(unittest.TestCase):
    """The last stretch of the burn stops aiming and starts stopping.

    A divert that chases the offset all the way down hands the legs whatever
    sideways speed the chase built up, and that is what the in-game divert
    died of -- 3.4 m/s at 16 m up, a toppled booster, and a lit engine
    throwing it back to 950 m.  Below LANDING_NULL_ALT_M the position term is
    gone and only the velocity term remains.
    """

    def setUp(self):
        self.env = FakeEnv(atmosphere=True)
        self.cfg = Config()

    def low(self, height, lateral):
        v = (-20.0,) + lateral
        r = state_above_pad(height)[0]
        return r, v

    def test_low_down_it_leans_against_the_lateral_speed(self):
        r, v = self.low(self.cfg.LANDING_NULL_ALT_M - 50.0, (0.0, 4.0))
        cmd = guidance.terminal_command(self.env, r, v, 
                                        self.cfg.LANDING_NULL_ALT_M - 50.0,
                                        6.0, 15.0, self.cfg)
        retrograde = vec.unit(vec.scale(v, -1.0))
        lean = vec.project_out(cmd.direction, retrograde)
        # +z is the way it is drifting, so the tilt has to go -z.
        self.assertLess(lean[2], 0.0)
        self.assertGreater(cmd.demand, 0.0)

    def test_low_down_it_ignores_the_offset_entirely(self):
        """Same state, wildly different aiming signals, identical command."""
        height = self.cfg.LANDING_NULL_ALT_M - 50.0
        r, v = self.low(height, (0.0, 4.0))
        plain = guidance.terminal_command(self.env, r, v, height, 6.0, 15.0,
                                          self.cfg)
        # A 400 m predicted miss the other way must not pull it off the null.
        with_zem = guidance.terminal_command(self.env, r, v, height, 6.0, 15.0,
                                             self.cfg, (0.0, 0.0, -400.0))
        self.assertAlmostEqual(
            vec.angle_between(plain.direction, with_zem.direction), 0.0,
            places=6)

    def test_a_stopped_booster_is_left_on_retrograde(self):
        height = self.cfg.LANDING_NULL_ALT_M - 50.0
        r, v = self.low(height, (0.0, self.cfg.LANDING_HVEL_TOL / 2.0))
        cmd = guidance.terminal_command(self.env, r, v, height, 6.0, 15.0,
                                        self.cfg, (0.0, 0.0, 400.0))
        retrograde = vec.unit(vec.scale(v, -1.0))
        self.assertAlmostEqual(vec.angle_between(cmd.direction, retrograde),
                               0.0, places=6)
        self.assertEqual(cmd.demand, 0.0)

    def test_high_up_it_still_aims(self):
        """The nulling is a floor on the law, not a replacement for it."""
        height = self.cfg.LANDING_NULL_ALT_M + 500.0
        r, v = self.low(height, (0.0, 0.0))
        cmd = guidance.terminal_command(self.env, r, v, height, 12.0, 15.0,
                                        self.cfg, (0.0, 0.0, 400.0))
        lean = vec.project_out(cmd.direction, vec.unit(vec.scale(v, -1.0)))
        self.assertLess(lean[2], 0.0, "leaned away from the overshoot")


class TestCorrectionRetirement(unittest.TestCase):
    """A backfire is evidence about the phase, not just about the burn."""

    def make_run(self, retire):
        run = object.__new__(autoland_module.Autoland)
        run.cfg = Config()
        run.cfg.CORRECTION_RETIRE_ON_BACKFIRE = retire
        run.cfg.CORRECTION_ABORT_GRACE_S = 3.0
        run.corrections = 1
        run.correction_retired = False
        run.correction_ready_ut = 0.0
        run.correction_start_miss = 200.0
        run.phase_start_ut = 0.0
        return run

    def test_a_backfire_is_recognised_after_the_grace_period(self):
        run = self.make_run(True)
        # Inside the grace period a burn is allowed to give ground: it has to
        # swing the nose round before it takes any.
        self.assertFalse(run.correction_making_it_worse(1.0, 900.0))
        self.assertTrue(run.correction_making_it_worse(10.0, 900.0))
        self.assertFalse(run.correction_making_it_worse(10.0, 210.0))

    def test_retirement_blocks_further_corrections(self):
        run = self.make_run(True)
        run.correction_retired = True
        snap = _snapshot_for_correction()
        self.assertFalse(run.correction_worthwhile(snap, _prediction(), 5000.0),
                         "corrected again after retiring")

    def test_without_the_flag_the_phase_may_re_enter(self):
        run = self.make_run(False)
        run.correction_retired = False
        snap = _snapshot_for_correction()
        run.correction_gradient = lambda *a: 99.0
        self.assertTrue(run.correction_worthwhile(snap, _prediction(), 5000.0))


def _prediction():
    return trajectory.Prediction(position=(KERBIN_R, 0.0, 0.0),
                                 time_to_land=120.0, burn_time=10.0,
                                 burn_altitude=800.0, entry_speed=200.0,
                                 powered=True, steps=10)


def _snapshot_for_correction():
    r, v = state_above_pad(20000.0, (0.0, 0.0, 300.0))
    return Snapshot(ut=100.0, position=r, velocity=v, direction=(1.0, 0.0, 0.0),
                    rotation=(0.0, 0.0, 0.0, 1.0), mass=20000.0,
                    available_thrust=4.0e5, max_thrust=4.0e5, isp=300.0,
                    surface_altitude=20000.0, vertical_speed=-100.0,
                    horizontal_speed=300.0, throttle=0.0, situation=None,
                    height_above_pad=20000.0, mean_altitude=20000.0,
                    leg_clearance=7.0)


class TestConfig(unittest.TestCase):
    def test_overrides_are_type_coerced(self):
        cfg = apply_overrides(Config(), ["BOOSTBACK_TOLERANCE_M=250",
                                         "STARTUP_ACTION_GROUP=3",
                                         "ENABLE_RCS=false"])
        self.assertEqual(cfg.BOOSTBACK_TOLERANCE_M, 250.0)
        self.assertEqual(cfg.STARTUP_ACTION_GROUP, 3)
        self.assertIs(cfg.ENABLE_RCS, False)

    def test_unknown_field_is_rejected(self):
        with self.assertRaises(KeyError):
            apply_overrides(Config(), ["NOPE=1"])


class TestTheRcsValve(unittest.TestCase):
    """Monopropellant is spent on turns, not on holding still.

    RCS used to be switched on at START and left on for the whole flight in
    the booster, and permitted for a whole phase in the spaceplane.  Measured
    on the spaceplane holding prograde in orbit and *doing nothing*: 55 of 150
    units in two minutes, because kRPC's autopilot hunts and the thrusters
    were open through the hunt.  These pin the relay that replaced it --
    especially the parts that are not a simple threshold, which is where a
    later simplification would put the waste back.

    ``boosterland/tests/fakeksp`` cannot check any of this from a flight: its
    ``control.rcs`` is a bare bool and its slew does not depend on it, so the
    closed-loop test can only show that nothing else broke.  In game, the
    thing to watch is the ``rcs on``/``rcs off`` events against the flip.
    """

    def setUp(self):
        self.cfg = Config()
        self.applied = []

    def valve(self):
        v = rcs.Valve(self.cfg)
        return v

    def drive(self, v, ut, error, permitted=True, q=None):
        return v.update(ut, permitted, error, q, apply=self.applied.append)

    def test_a_big_error_opens_it(self):
        v = self.valve()
        self.assertTrue(self.drive(v, 0.0, 40.0))

    def test_it_does_not_chatter_inside_the_deadband(self):
        """The whole reason there are two thresholds.  An error wandering
        between ON and OFF -- which is what the tail of every slew looks like
        -- must not switch the thrusters at the frequency of the oscillation
        they are damping."""
        v = self.valve()
        self.drive(v, 0.0, 40.0)
        switches = len(self.applied)
        for i, error in enumerate([4.9, 2.0, 4.0, 1.6, 3.0, 2.2, 4.8]):
            self.drive(v, 1.0 + i, error)
        self.assertTrue(v.on, "the valve shut inside its own deadband")
        self.assertEqual(len(self.applied), switches,
                         "the valve switched %d times crossing no threshold"
                         % (len(self.applied) - switches))

    def test_it_shuts_only_after_the_error_stays_small(self):
        """A swing through zero is not an arrival.  Without the settle time
        the valve drops out mid-oscillation and is straight back on."""
        v = self.valve()
        self.drive(v, 0.0, 40.0)
        self.drive(v, 1.0, 0.2)
        self.assertTrue(v.on, "shut on the first tick inside OFF")
        self.drive(v, 1.0 + 0.5 * self.cfg.RCS_SETTLE_S, 0.2)
        self.assertTrue(v.on, "shut before the settle time was up")
        self.drive(v, 1.0 + 1.5 * self.cfg.RCS_SETTLE_S, 0.2)
        self.assertFalse(v.on, "never shut at all")

    def test_a_new_turn_reopens_it_at_once(self):
        v = self.valve()
        self.drive(v, 0.0, 40.0)
        self.drive(v, 10.0, 0.2)
        self.drive(v, 10.0 + 2.0 * self.cfg.RCS_SETTLE_S, 0.2)
        self.assertFalse(v.on)
        self.assertTrue(self.drive(v, 20.0, 30.0),
                        "a fresh command did not get its thrusters back")

    def test_an_unknown_error_holds_rather_than_deciding(self):
        """No command yet, or a nose that cannot be read.  A missing answer
        must not look like a good one: reporting 0 here would shut the
        thrusters in the middle of a flip."""
        for unknown in (None, -1.0):
            v = self.valve()
            self.drive(v, 0.0, 40.0)
            self.drive(v, 1.0, unknown)
            self.assertTrue(v.on, "%r was read as settled" % (unknown,))
            self.drive(v, 2.0, unknown)
            self.assertTrue(v.on)

    def test_refusing_permission_is_immediate_and_absolute(self):
        """A phase that says no is not overruled by a large error: that is
        what keeps the thrusters off the slow turns entirely."""
        v = self.valve()
        self.drive(v, 0.0, 40.0)
        self.assertFalse(self.drive(v, 1.0, 90.0, permitted=False))

    def test_disabling_rcs_disables_all_of_it(self):
        self.cfg = apply_overrides(Config(), ["ENABLE_RCS=False"])
        v = self.valve()
        self.assertFalse(self.drive(v, 0.0, 180.0))
        self.assertEqual(self.applied, [False])

    def test_the_booster_has_no_dynamic_pressure_ceiling(self):
        """The spaceplane shuts its thrusters in thick air because its
        control surfaces own the attitude there.  A booster's do not -- the
        descent is flown on RCS, wheels and the gimbal -- so the ceiling is
        off, and off has to mean off rather than 0 Pa."""
        self.assertEqual(Config().RCS_Q_MAX_PA, 0.0)
        v = self.valve()
        self.assertTrue(self.drive(v, 0.0, 40.0, q=40000.0))

    def test_a_configured_ceiling_shuts_it(self):
        self.cfg = apply_overrides(Config(), ["RCS_Q_MAX_PA=500"])
        v = self.valve()
        self.drive(v, 0.0, 40.0, q=100.0)
        self.assertTrue(v.on)
        self.assertFalse(self.drive(v, 1.0, 40.0, q=900.0))

    def test_it_writes_the_hardware_only_when_it_changes(self):
        """The valve is called every tick of every phase.  A kRPC write per
        tick is a round trip per tick for a bool that has not moved."""
        v = self.valve()
        for i in range(10):
            self.drive(v, float(i), 40.0)
        self.assertEqual(self.applied, [True])


class TestTheBoosterMeasuresTheTurnItIsFlying(unittest.TestCase):
    """``aim`` may route a big turn over the top (``FLIP_VIA_VERTICAL``), and
    the pointing error has to be taken against the *waypoint* it commanded,
    not the final aim.  Against the final aim a flip going exactly to plan
    reads as a hundred degrees of failure -- which is harmless while it only
    opens thrusters, and is not harmless the moment anything else reads it.
    """

    def test_the_error_is_measured_against_the_last_command(self):
        run = object.__new__(autoland_module.Autoland)
        run.commanded_direction = (1.0, 0.0, 0.0)
        snap = SimpleNamespace(direction=(0.0, 1.0, 0.0))
        self.assertAlmostEqual(run.pointing_error(snap), 90.0, places=6)

    def test_no_command_yet_is_not_zero_error(self):
        run = object.__new__(autoland_module.Autoland)
        run.commanded_direction = None
        snap = SimpleNamespace(direction=(0.0, 1.0, 0.0))
        self.assertLess(run.pointing_error(snap), 0.0)

    def test_an_unreadable_nose_is_not_zero_error(self):
        run = object.__new__(autoland_module.Autoland)
        run.commanded_direction = (1.0, 0.0, 0.0)
        snap = SimpleNamespace(direction=(0.0, 0.0, 0.0))
        self.assertLess(run.pointing_error(snap), 0.0)


if __name__ == "__main__":
    unittest.main()
