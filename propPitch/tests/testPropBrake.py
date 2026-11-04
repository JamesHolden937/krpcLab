"""The Brakes group brakes with the props."""

import types
import unittest

from propPitch.autopilot import PropPilot
from propPitch.bladeModel import (MOST_FORWARD, MOST_REVERSE, ZERO_THRUST,
                                  PropModel)
from propPitch.config import Config
from propPitch.tests.testBladeModel import rotor


class TestBrakeObjective(unittest.TestCase):
    def test_against_the_motion(self):
        p = types.SimpleNamespace(cfg=Config())
        self.assertEqual(PropPilot.brake_objective(p, 20.0), MOST_REVERSE)
        self.assertEqual(PropPilot.brake_objective(p, -3.0), MOST_FORWARD)
        self.assertEqual(PropPilot.brake_objective(p, 0.2), ZERO_THRUST)
        self.assertEqual(PropPilot.brake_objective(p, -0.2), ZERO_THRUST)


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

    def test_zero_thrust(self):
        o = self.m.optimum([460.0], (0.0, 0.3, 0.0), 1.2, 340.0, -90, 90,
                           objective=ZERO_THRUST)
        self.assertLess(abs(o.thrust), 0.5)

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
