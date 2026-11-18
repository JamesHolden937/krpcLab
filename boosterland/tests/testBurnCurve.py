"""The landing burn's own Cd*A curve (fins neutral) is what its
propagation flies, and only its propagation."""

import math
import unittest
from types import SimpleNamespace

from common import vec

from boosterland import trajectory
from boosterland.config import Config
from boosterland.tests import fakeksp


class Log(object):
    def event(self, *a, **k):
        pass

    def telemetry(self, *a, **k):
        return False


class TestBurnCurve(unittest.TestCase):

    def environment(self):
        from boosterland.environment import Environment
        vessel = fakeksp.Vessel((fakeksp.RADIUS + 12000.0, 0.0, 0.0),
                                (0.0, 0.0, 400.0), mass=30000.0,
                                thrust=520000.0, isp=300.0, atmosphere=True)
        conn = fakeksp.install(vessel)
        return Environment(conn, vessel, vessel.body, Config(), Log(), 0.0)

    def test_burn_flies_its_own_curve_and_the_coast_does_not(self):
        env = self.environment()
        env._curve_machs, env._curve_areas = [0.5, 3.0], [8.0, 8.0]
        self.assertEqual(env.burn_drag_area_at(500.0, 5000.0), 8.0)
        self.assertTrue(env.set_burn_curve([3.0] * env._bins))
        self.assertAlmostEqual(env.burn_drag_area_at(500.0, 5000.0), 3.0)
        self.assertEqual(env.drag_area_at(500.0, 5000.0), 8.0)
        self.assertAlmostEqual(
            trajectory.drag_lookup(env, burn=True)(500.0, 5000.0), 3.0)
        self.assertEqual(trajectory.drag_lookup(env)(500.0, 5000.0), 8.0)
        self.assertAlmostEqual(
            trajectory.drag_lookup(env, scale=2.0, burn=True)(500.0, 5000.0),
            6.0)

    def test_an_empty_sweep_leaves_the_burn_on_the_coast_curve(self):
        env = self.environment()
        env._curve_machs, env._curve_areas = [0.5, 3.0], [8.0, 8.0]
        self.assertFalse(env.set_burn_curve([None] * env._bins))
        self.assertEqual(env.burn_drag_area_at(500.0, 5000.0), 8.0)

    def test_open_brakes_fly_the_full_curve(self):
        env = self.environment()
        env._curve_machs, env._curve_areas = [0.5, 3.0], [8.0, 8.0]
        env.set_burn_curve([3.0] * env._bins)
        env.set_full_curve([12.0] * env._bins)
        env.late_switch_alt = float("inf")
        self.assertEqual(env.burn_drag_area_at(500.0, 5000.0), 12.0)


class TestBurnLift(unittest.TestCase):

    def environment(self):
        return TestBurnCurve.environment(self)

    def test_slope_and_shape_interpolate(self):
        env = self.environment()
        a1, a2 = math.radians(2.0), math.radians(8.0)
        self.assertIsNone(env.burn_aero(500.0, 5000.0, a1))
        self.assertTrue(env.set_burn_lift([60.0] * env._bins,
                                          [(a1, 1.0, 1.1), (a2, 1.3, 1.8)]))
        cn, d = env.burn_aero(500.0, 5000.0, a1 / 2.0)
        self.assertAlmostEqual(cn, 60.0 * a1 / 2.0)
        self.assertAlmostEqual(d, 1.05)
        cn, d = env.burn_aero(500.0, 5000.0, (a1 + a2) / 2.0)
        self.assertAlmostEqual(cn, 60.0 * (a1 + a2) / 2.0 * 1.15)
        self.assertAlmostEqual(d, 1.45)
        cn, d = env.burn_aero(500.0, 5000.0, 2 * a2)
        self.assertAlmostEqual(d, 1.8)

    def test_the_force_acts_opposite_the_nose_tilt(self):
        env = SimpleNamespace(equatorial_radius=600000.0,
                              density=lambda alt: 1.0)
        up = (1.0, 0.0, 0.0)
        r = (605000.0, 0.0, 0.0)
        v = (-100.0, 0.0, 100.0)            # falling, moving +z
        retro = vec.unit(vec.scale(v, -1.0))
        a = math.radians(2.0)
        side = vec.unit(vec.project_out(up, retro))
        nose = vec.add(vec.scale(retro, math.cos(a)), vec.scale(side, math.sin(a)))
        aero = lambda s, h, alpha: (10.0 * alpha, 1.0)
        total = trajectory._with_body_aero(
            env, 1.0, lambda s, h: 0.0, aero, lambda rr, vv: (0.0, 0.0, 0.0),
            nose, None)
        f = total(r, v)
        q = 0.5 * vec.norm(v) ** 2
        self.assertAlmostEqual(vec.dot(f, side), -q * 10.0 * a, places=3)
        self.assertAlmostEqual(vec.dot(f, retro), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
