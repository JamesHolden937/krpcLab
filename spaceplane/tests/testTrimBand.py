"""TrimBand and holdable_span (Config.TRIM_BAND)."""
import unittest

from spaceplane import trajectory
from spaceplane.config import Config


def cfg(on=True):
    c = Config()
    c.TRIM_BAND = on
    return c


A = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0]


class TestHoldableSpan(unittest.TestCase):
    def test_unstable_gives_a_ceiling(self):
        # nose-up moment growing with alpha, control 100: share 0.5 holds 50
        lo, hi = trajectory.holdable_span(A, [0, 20, 40, 60, 80, 100],
                                          [100] * 6, 0.5)
        self.assertIsNone(lo)
        self.assertAlmostEqual(hi, 17.5)

    def test_stable_and_wide_gives_no_limit(self):
        self.assertEqual(trajectory.holdable_span(
            A, [-10, -15, -20, -25, -30, -35], [100] * 6, 0.5), (None, None))

    def test_nothing_holdable(self):
        self.assertEqual(trajectory.holdable_span(
            A, [90] * 6, [100] * 6, 0.5), (None, None))

    def test_floor_and_ceiling(self):
        lo, hi = trajectory.holdable_span(A, [-80, -40, 0, 40, 80, 120],
                                          [100] * 6, 0.5)
        self.assertAlmostEqual(lo, 8.75)
        self.assertAlmostEqual(hi, 21.25)


class TestTrimBand(unittest.TestCase):
    def test_clamp_interpolates(self):
        b = trajectory.TrimBand(cfg(), [(7.0, None, 30.0), (5.0, None, 20.0)])
        self.assertAlmostEqual(b.clamp(40.0, 6.0), 25.0)
        self.assertAlmostEqual(b.clamp(10.0, 6.0), 10.0)
        self.assertAlmostEqual(b.clamp(40.0, 9.0), 30.0)

    def test_off(self):
        b = trajectory.TrimBand(cfg(False), [(5.0, None, 20.0)])
        self.assertEqual(b.clamp(40.0, 5.0), 40.0)


if __name__ == "__main__":
    unittest.main()
