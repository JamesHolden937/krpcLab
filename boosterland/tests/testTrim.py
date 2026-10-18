"""The boostback heading hold and the coast's RCS cross-track trim."""
import math
import unittest
from types import SimpleNamespace

from common import vec
from boosterland import guidance, trajectory
from boosterland.autoland import body_vector
from boosterland.config import Config

R = 600000.0


class FakeEnv:
    def __init__(self, target):
        self.target = target


def prediction_at(point, ttl=140.0):
    return SimpleNamespace(position=point, time_to_land=ttl)


class TestHeldBoostback(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.target = (R, 0.0, 0.0)
        self.env = FakeEnv(self.target)
        self.r = (R + 30000.0, 0.0, 20000.0)
        # The held heading: toward +z (the miss was toward -z, short).
        self.heading = (0.0, 0.0, 1.0)

    def test_burns_only_the_miss_along_the_heading(self):
        # 2 km short along the heading, 150 m off to the side.
        pred = prediction_at((R, 150.0, -2000.0))
        throttle, remaining = guidance.held_boostback(
            self.env, self.r, pred, self.heading, self.cfg)
        self.assertAlmostEqual(remaining, 2000.0, delta=5.0)
        self.assertAlmostEqual(throttle, 2000.0 / 140.0 / 60.0, places=2)

    def test_cross_track_alone_does_not_burn(self):
        pred = prediction_at((R, 150.0, 0.0))
        throttle, remaining = guidance.held_boostback(
            self.env, self.r, pred, self.heading, self.cfg)
        self.assertAlmostEqual(remaining, 0.0, delta=1.0)
        self.assertEqual(throttle, 0.0)

    def test_past_the_pad_stops(self):
        pred = prediction_at((R, 0.0, 300.0))
        throttle, remaining = guidance.held_boostback(
            self.env, self.r, pred, self.heading, self.cfg)
        self.assertLess(remaining, 0.0)
        self.assertEqual(throttle, 0.0)


class TestRcsTrim(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_pushes_against_the_cross_track(self):
        target = (R, 0.0, 0.0)
        here = (R + 20000.0, 0.0, 20000.0)
        along, left = trajectory.miss_axes(target, here)
        # A miss to the left: the command pushes right, proportionally.
        point = vec.add(target, vec.scale(left, 15.0))
        _, cross = trajectory.miss_components(None, point, target, here)
        self.assertAlmostEqual(cross, 15.0, delta=0.1)
        cmd = guidance.rcs_trim_command(cross, left, self.cfg)
        self.assertAlmostEqual(vec.dot(cmd, left), -0.5, places=3)
        self.assertAlmostEqual(vec.dot(cmd, along), 0.0, places=6)

    def test_saturates_and_deadbands(self):
        left = (0.0, 1.0, 0.0)
        cmd = guidance.rcs_trim_command(-90.0, left, self.cfg)
        self.assertAlmostEqual(vec.norm(cmd), 1.0)
        self.assertGreater(cmd[1], 0.0)
        self.assertIsNone(guidance.rcs_trim_command(2.0, left, self.cfg))
        self.assertIsNone(guidance.rcs_trim_command(float("nan"), left,
                                                    self.cfg))


    def test_with_long_pushes_against_the_whole_miss(self):
        left, along = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
        cmd = guidance.rcs_trim_command(9.0, left, self.cfg, 12.0, along)
        self.assertAlmostEqual(vec.norm(cmd), 0.5, places=6)
        self.assertAlmostEqual(cmd[1] / cmd[2], 9.0 / 12.0, places=6)
        self.assertLess(cmd[2], 0.0)
        # Inside the deadband on both: nothing.
        self.assertIsNone(guidance.rcs_trim_command(1.0, left, self.cfg,
                                                    2.0, along))


class TestLongIsFree(unittest.TestCase):
    """Run the gate on a bare Autoland: it must read only real fields."""
    def run_for(self, late_state, **sets):
        from boosterland.autoland import Autoland
        run = object.__new__(Autoland)
        run.cfg = Config()
        for k, v in sets.items():
            setattr(run.cfg, k, v)
        run.late_state = late_state
        return run.long_is_free(SimpleNamespace(position=(R + 5000.0, 0, 0)))

    def test_owned_by_the_late_solve_until_handed(self):
        self.assertFalse(self.run_for("armed"))
        self.assertFalse(self.run_for(None))
        self.assertTrue(self.run_for("handed"))
        self.assertTrue(self.run_for("off"))
        self.assertTrue(self.run_for(None, DEPLOY_LATE=False))


class TestBodyVector(unittest.TestCase):
    def test_round_trip_through_the_attitude(self):
        q = vec.quat_axis_angle(vec.unit((1.0, 2.0, 0.5)), 1.1)
        nose = vec.quat_rotate(q, (0.0, 1.0, 0.0))
        snap = SimpleNamespace(rotation=q, direction=nose)
        world = (0.3, -0.2, 0.9)
        body = body_vector(snap, world)
        back = vec.quat_rotate(q, body)
        for a, b in zip(back, world):
            self.assertAlmostEqual(a, b, places=9)
        # The nose itself is body +y.
        b = body_vector(snap, nose)
        self.assertAlmostEqual(b[1], 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
