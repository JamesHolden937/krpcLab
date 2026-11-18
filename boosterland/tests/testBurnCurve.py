"""The landing burn's own Cd*A curve (fins neutral) is what its
propagation flies, and only its propagation."""

import unittest

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


if __name__ == "__main__":
    unittest.main()
