"""guidance.flip_throttle: the flip's throttle, from authority and drift."""

import math
import unittest

from boosterland import guidance


class TestFlipLaw(unittest.TestCase):
    def test_turn_time_from_rest(self):
        self.assertAlmostEqual(guidance.turn_time(math.pi, 0.0, 1.0),
                               2.0 * math.sqrt(math.pi))

    def test_no_thrust_when_rcs_can_turn_it(self):
        u, t, _ = guidance.flip_throttle(math.radians(150), 0.0, 0.5, 0.05,
                                         54.0, 5.0)
        self.assertEqual(u, 0.0)
        self.assertTrue(math.isfinite(t))

    def test_some_thrust_when_nothing_else_turns_it(self):
        u, _, _ = guidance.flip_throttle(math.radians(150), 0.0, 0.5, 0.0,
                                         54.0, 5.0)
        self.assertGreater(u, 0.0)
        self.assertLess(u, 1.0)

    def test_full_thrust_when_nearly_aligned(self):
        u, _, _ = guidance.flip_throttle(math.radians(3), 0.0, 0.5, 0.05,
                                         54.0, 5.0)
        self.assertEqual(u, 1.0)

    def test_more_drift_buys_more_thrust(self):
        lo, _, _ = guidance.flip_throttle(math.radians(150), 0.0, 0.5, 0.0,
                                          54.0, 2.0)
        hi, _, _ = guidance.flip_throttle(math.radians(150), 0.0, 0.5, 0.0,
                                          54.0, 40.0)
        self.assertGreaterEqual(hi, lo)

    def test_drift_rate(self):
        self.assertAlmostEqual(guidance.drift_rate(10000.0, 500.0, 100.0), 6.0)


if __name__ == "__main__":
    unittest.main()
