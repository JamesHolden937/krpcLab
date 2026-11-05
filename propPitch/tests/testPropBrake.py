"""The Brakes group brakes with the props."""

import types
import unittest

from propPitch.autopilot import PropPilot
from propPitch.bladeModel import (MOST_FORWARD, MOST_REVERSE,
                                  PropModel)
from propPitch.config import Config
from propPitch.tests.testBladeModel import rotor


class TestBrakeObjective(unittest.TestCase):
    def test_against_the_motion(self):
        p = types.SimpleNamespace(cfg=Config(), held=False)
        self.assertEqual(PropPilot.brake_objective(p, 20.0), MOST_REVERSE)
        self.assertEqual(PropPilot.brake_objective(p, -3.0), MOST_FORWARD)
        self.assertFalse(p.held)
        self.assertIsNone(PropPilot.brake_objective(p, 0.2))
        self.assertTrue(p.held)

    def test_stopped_until_it_moves(self):
        """Still, the props stay stopped through a creep below
        BRAKE_RESUME_SPEED and spin up past it."""
        p = types.SimpleNamespace(cfg=Config(), held=False)
        self.assertIsNone(PropPilot.brake_objective(p, -0.2))
        self.assertIsNone(PropPilot.brake_objective(p, 1.5))
        self.assertIsNone(PropPilot.brake_objective(p, -1.9))
        self.assertEqual(PropPilot.brake_objective(p, 2.5), MOST_REVERSE)
        self.assertFalse(p.held)


class TestBrakeModel(unittest.TestCase):
    def setUp(self):
        self.m = PropModel([rotor()])

    def test_limits_from_the_blade(self):
        self.assertEqual(self.m.deploy_limit, 90.0)       # R-25: 1.5 x 60

    def test_reverse_thrust(self):
        lim = self.m.deploy_limit
        for v in (0.0, 30.0, 100.0):
            o = self.m.optimum([460.0], (0.0, v, 0.0), 1.2, 340.0, -lim, lim,
                               objective=MOST_REVERSE)
            self.assertLess(o.thrust, -50.0, v)
            for d in range(-90, 91, 5):
                self.assertGreaterEqual(
                    self.m.evaluate(d, [460.0], (0.0, v, 0.0), 1.2,
                                    340.0).thrust, o.thrust - 1e-6)

    def test_floor_holds_the_thrust(self):
        """A floor turns the most reverse thrust into the most that does not
        pass it; a ceiling the same for forward thrust."""
        free = self.m.optimum([460.0], (0.0, 30.0, 0.0), 1.2, 340.0, -90, 90,
                              objective=MOST_REVERSE)
        o = self.m.optimum([460.0], (0.0, 30.0, 0.0), 1.2, 340.0, -90, 90,
                           objective=MOST_REVERSE, floor=free.thrust / 3)
        self.assertAlmostEqual(o.thrust, free.thrust / 3, delta=0.5)
        fwd = self.m.optimum([460.0], (0.0, -5.0, 0.0), 1.2, 340.0, -90, 90,
                             objective=MOST_FORWARD, ceiling=20.0)
        self.assertAlmostEqual(fwd.thrust, 20.0, delta=0.5)

    def test_feather_is_edge_on(self):
        """At 0 rpm the most thrust is the least drag: Deploy 0, the blade
        edge-on to an axial flow, ~25x less drag than at 85 deg."""
        o = self.m.optimum([0.0], (0.0, 60.0, 0.0), 1.2, 340.0, -90, 90)
        self.assertAlmostEqual(o.deploy, 0.0, delta=1.0)
        e = self.m.evaluate(85.0, [0.0], (0.0, 60.0, 0.0), 1.2, 340.0)
        self.assertGreater(o.thrust, 10.0 * e.thrust)        # both < 0

    def test_windmill_overspeeds(self):
        """Air driving the rotor harder than the motor can hold it back
        settles above the limit, with the driving torque at the cap."""
        r = rotor()
        r.max_torque = 5.0
        t, q, w = r.settled(76.6, 460.0, (0.0, 100.0, 0.0), 1.2, 340.0,
                            (0.0, 1.0, 0.0))
        t0, q0 = r.thrust_torque(76.6, 460.0, (0.0, 100.0, 0.0), 1.2, 340.0)
        self.assertLess(q0, -5.0)
        self.assertGreater(w, 460.0)
        self.assertAlmostEqual(q, -5.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
