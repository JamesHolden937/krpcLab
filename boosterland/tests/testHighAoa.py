"""HIGH_AOA_DESCENT: the polar, the scheduled angle, the cone and its solve."""

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
    """Holds 40 deg in thin air, falling as 1/sqrt(q) above 500 Pa."""

    def __init__(self, lift=True):
        super().__init__(atmosphere=True)
        self.drag_area = 20.0
        self.reswept_ut = 0.0
        self.lift = lift

    def polar_at(self, speed, altitude):
        if self.lift:
            return fin_polar()
        return [(a, 0.0, d) for a, _, d in fin_polar()]

    def holdable_aoa(self, speed, altitude):
        q = 0.5 * self.density(altitude) * speed * speed
        return math.radians(40.0) * min(1.0, math.sqrt(500.0 / max(q, 1.0)))


class TestPolar(unittest.TestCase):
    def test_point(self):
        lift, ratio = trajectory.polar_point(fin_polar(), math.radians(45.0))
        self.assertAlmostEqual(lift, -150.0, delta=1.0)
        self.assertAlmostEqual(ratio, 85.0 / 20.0, delta=0.01)
        lift, ratio = trajectory.polar_point(fin_polar(), math.radians(90.0))
        self.assertAlmostEqual(lift, 0.0, delta=1e-6)
        self.assertAlmostEqual(ratio, 150.0 / 20.0, delta=1e-6)

    def test_schedule_is_the_hold_capped(self):
        env, cfg = PolarEnv(), Config()
        cfg.HIGH_AOA_MAX_DEG = 30.0
        schedule = guidance.aoa_schedule(env, cfg)
        self.assertAlmostEqual(schedule(100.0, 30000.0), math.radians(30.0))
        thick = schedule(600.0, 5000.0)
        self.assertLess(thick, math.radians(30.0))
        self.assertAlmostEqual(thick, env.holdable_aoa(600.0, 5000.0))

    def test_cone_limit_follows_the_flip_rate(self):
        cfg = Config()
        cfg.HIGH_AOA_CONE_MAX = 10.0
        w = guidance.cone_limit(cfg, 20.0, math.radians(30.0))
        self.assertAlmostEqual(w * math.sin(math.radians(30.0)),
                               cfg.HIGH_AOA_CONE_FRACTION * math.radians(20.0))


class TestHighAoaPropagation(unittest.TestCase):
    def setUp(self):
        self.env = PolarEnv()
        self.cfg = Config()
        # 20 km up, 3 km out, falling, a little horizontal speed toward the pad.
        self.r = (KERBIN_R + 20000.0, 0.0, 3000.0)
        self.v = (-600.0, 0.0, -60.0)
        self.normal = guidance.steering_plane(self.r, self.v)

    def predict(self, env, steer=None):
        return trajectory.predict_landing(env, self.r, self.v, 2e4, 40.0,
                                          self.cfg, steer=steer)

    def miss(self, env, prediction):
        return trajectory.miss_components(env, prediction.position,
                                          env.target, self.r)

    def scheduled(self, env, rate):
        return trajectory.Steer(0.3, self.normal, polar=env.polar_at,
                                flip_s=5.0,
                                schedule=guidance.aoa_schedule(env, self.cfg),
                                cone_rate=rate)

    def test_scheduled_angle_reaches_the_burn_slower(self):
        plain = self.predict(self.env)
        slow = self.predict(self.env, self.scheduled(self.env, 0.5))
        self.assertTrue(plain.powered and slow.powered)
        self.assertLess(slow.entry_speed, 0.9 * plain.entry_speed)

    def test_coning_averages_the_lift_out(self):
        flat = PolarEnv(lift=False)
        drag_only = self.miss(flat, self.predict(flat,
                                                 self.scheduled(flat, 0.5)))
        held = self.miss(self.env, self.predict(self.env,
                                                self.scheduled(self.env, 0.0)))
        coned = self.miss(self.env, self.predict(self.env,
                                                 self.scheduled(self.env, 1.0)))
        off_held = math.hypot(held[0] - drag_only[0], held[1] - drag_only[1])
        off_coned = math.hypot(coned[0] - drag_only[0],
                               coned[1] - drag_only[1])
        self.assertLess(off_coned, 0.2 * off_held,
                        "held %.0f coned %.0f" % (off_held, off_coned))

    def test_solve_closes_the_miss(self):
        env, cfg = self.env, self.cfg
        cfg.HIGH_AOA_MAX_MISS_M = 1e9
        cfg.HIGH_AOA_SOLVE_ITERS = 6
        before = self.miss(env, self.predict(env, self.scheduled(env, 0.5)))
        m0 = math.hypot(*before)
        steer, prediction, got, flip_s = guidance.solve_high_aoa(
            env, self.r, self.v, 2e4, 40.0, cfg, None, 20.0)
        self.assertIsNotNone(steer)
        m1 = math.hypot(*self.miss(env, prediction))
        self.assertLess(m1, max(0.3 * m0, 50.0), "miss %.0f -> %.0f" % (m0, m1))
        self.assertGreater(flip_s, 0.0)
        self.assertGreaterEqual(got[1], cfg.HIGH_AOA_CONE_MIN)

    def test_unavailable_without_a_flip_rate(self):
        self.assertIsNone(guidance.solve_high_aoa(
            self.env, self.r, self.v, 2e4, 40.0, self.cfg, None, None)[0])


if __name__ == "__main__":
    unittest.main()
