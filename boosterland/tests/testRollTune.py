"""boosterland/rolltune.py: kRPC's auto-tune reproduced, the rate cap."""

import math
import unittest

from boosterland import rolltune


class TestRollTune(unittest.TestCase):
    def test_krpc_defaults(self):
        zeta, omega = rolltune.damping_and_frequency(1.0, 0.01)
        self.assertAlmostEqual(2 * zeta * omega, 9.21, delta=0.01)
        self.assertAlmostEqual(omega * omega, 31.1, delta=0.1)

    def test_reproduces_the_logged_roll_gains(self):
        # LOG16184 burn start, auto-tune: roll (0.0269, 0.0909, 0)
        zeta, omega = rolltune.damping_and_frequency(1.0, 0.01)
        accel = 2 * zeta * omega / 0.0269
        kp, ki, kd = rolltune.krpc_gains(accel, zeta, omega)
        self.assertAlmostEqual(kp, 0.0269, places=4)
        self.assertAlmostEqual(ki, 0.0909, delta=0.0005)
        self.assertEqual(kd, 0.0)

    def test_no_authority_no_gains(self):
        self.assertIsNone(rolltune.krpc_gains(None, 0.8, 5.0))
        self.assertIsNone(rolltune.krpc_gains(0.0, 0.8, 5.0))

    def test_rate_cap_stops_inside_the_brake_angle(self):
        cap = rolltune.rate_cap(1.5, 5.0, 1.0)
        self.assertAlmostEqual(cap * cap / (2 * 1.5), math.radians(5.0))
        self.assertEqual(rolltune.rate_cap(1e6, 5.0, 1.0), 1.0)
        self.assertEqual(rolltune.rate_cap(None, 5.0, 1.0), 1.0)


if __name__ == "__main__":
    unittest.main()
