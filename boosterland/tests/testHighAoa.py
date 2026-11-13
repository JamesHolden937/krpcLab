"""HIGH_AOA_DESCENT: the polar, the back-branch lift map and its solve."""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from boosterland import guidance, trajectory          # noqa: E402
from boosterland.config import Config                 # noqa: E402
from boosterland.tests.testOffline import FakeEnv, KERBIN_R  # noqa: E402


def fin_polar(speed=None, altitude=None):
    """Shaped like the fin boosters' probes: lift peaks at 45 deg with the
    probe's sign (negative), ~0 at 90; drag rises toward broadside."""
    out = []
    for i in range(9):
        a = math.radians(90.0 * i / 8)
        out.append((a, -150.0 * math.sin(2.0 * a), 20.0 + 130.0 * math.sin(a) ** 2))
    return out


class PolarEnv(FakeEnv):
    def __init__(self):
        super().__init__(atmosphere=True)
        self.drag_area = 20.0
        self.reswept_ut = 0.0

    def polar_at(self, speed, altitude):
        return fin_polar()

    def torque_limit_aoa(self, speed, altitude):
        return math.radians(90.0)


class TestPolar(unittest.TestCase):
    def test_point(self):
        lift, ratio = trajectory.polar_point(fin_polar(), math.radians(45.0))
        self.assertAlmostEqual(lift, -150.0, delta=1.0)
        self.assertAlmostEqual(ratio, 85.0 / 20.0, delta=0.01)
        lift, ratio = trajectory.polar_point(fin_polar(), math.radians(90.0))
        self.assertAlmostEqual(lift, 0.0, delta=1e-6)
        self.assertAlmostEqual(ratio, 150.0 / 20.0, delta=1e-6)

    def test_lift_command_makes_the_lift_asked(self):
        branch = guidance.back_branch(fin_polar(), math.radians(90.0))
        self.assertIsNotNone(branch)
        for ell in ((50.0, 0.0), (0.0, -80.0), (30.0, 40.0)):
            u, e = guidance.lift_command(branch, ell, (1.0, 0.0))
            aoa = math.hypot(*u)
            self.assertGreaterEqual(aoa, math.radians(45.0) - 1e-6,
                                    "flown on the draggy side of the peak")
            lift, _ = trajectory.polar_point(fin_polar(), aoa)
            # The lift acts along the tilt direction with the polar's sign.
            made = (lift * u[0] / aoa, lift * u[1] / aoa)
            self.assertAlmostEqual(made[0], ell[0], delta=1.0)
            self.assertAlmostEqual(made[1], ell[1], delta=1.0)
        u, e = guidance.lift_command(branch, (0.0, 0.0), (0.0, 1.0))
        self.assertAlmostEqual(math.hypot(*u), math.radians(90.0), places=6)
        self.assertEqual(e, (0.0, 1.0))

    def test_no_far_side_below_the_peak(self):
        self.assertIsNone(guidance.back_branch(fin_polar(), math.radians(40.0)))


class TestHighAoaPropagation(unittest.TestCase):
    def setUp(self):
        self.env = PolarEnv()
        self.cfg = Config()
        # 20 km up, 3 km out, falling, a little horizontal speed toward the pad.
        self.r = (KERBIN_R + 20000.0, 0.0, 3000.0)
        self.v = (-600.0, 0.0, -60.0)

    def test_broadside_reaches_the_burn_slower(self):
        normal = guidance.steering_plane(self.r, self.v)
        plain = trajectory.predict_landing(self.env, self.r, self.v, 2e4,
                                           40.0, self.cfg)
        flat = trajectory.Steer(math.radians(90.0), normal,
                                polar=self.env.polar_at, flip_s=5.0)
        slow = trajectory.predict_landing(self.env, self.r, self.v, 2e4,
                                          40.0, self.cfg, steer=flat)
        self.assertTrue(plain.powered and slow.powered)
        self.assertLess(slow.entry_speed, 0.8 * plain.entry_speed)

    def test_solve_closes_the_miss(self):
        env, cfg = self.env, self.cfg
        cfg.HIGH_AOA_MAX_MISS_M = 1e9
        normal = guidance.steering_plane(self.r, self.v)
        rest = trajectory.Steer(math.radians(90.0), normal,
                                polar=env.polar_at, flip_s=5.0)
        before = trajectory.predict_landing(env, self.r, self.v, 2e4, 40.0,
                                            cfg, steer=rest)
        m0 = math.hypot(*trajectory.miss_components(
            env, before.position, env.target, self.r))
        steer, prediction, e, ell = guidance.solve_high_aoa(
            env, self.r, self.v, 2e4, 40.0, cfg, None, 5.0)
        self.assertIsNotNone(steer)
        m1 = math.hypot(*trajectory.miss_components(
            env, prediction.position, env.target, self.r))
        self.assertLess(m1, 0.5 * m0, "miss %.0f -> %.0f" % (m0, m1))
        self.assertGreaterEqual(abs(steer.aoa), math.radians(45.0) - 1e-6)

    def test_unavailable_without_a_flip_time(self):
        self.assertEqual(guidance.solve_high_aoa(
            self.env, self.r, self.v, 2e4, 40.0, self.cfg, None, None)[0],
            None)


if __name__ == "__main__":
    unittest.main()
