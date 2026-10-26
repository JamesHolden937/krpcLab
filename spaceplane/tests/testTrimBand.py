"""TrimBand and trim_crossing (Config.TRIM_BAND)."""
import unittest

from spaceplane import trajectory
from spaceplane.config import Config


def cfg(on=True):
    c = Config()
    c.TRIM_BAND = on
    return c


class TestTrimBand(unittest.TestCase):
    def test_crossing(self):
        self.assertAlmostEqual(trajectory.trim_crossing(
            [10, 20, 30, 40], [+30, +10, -10, -30]), 25.0)

    def test_no_crossing_is_none(self):
        self.assertIsNone(trajectory.trim_crossing([10, 20], [-5, -10]))
        self.assertIsNone(trajectory.trim_crossing([10, 20], [5, 10]))

    def test_floor_interpolates_and_subtracts(self):
        b = trajectory.TrimBand(cfg(), [(7.0, 36.0), (5.0, 32.0), (3.0, None)])
        below = cfg().TRIM_BAND_BELOW_DEG
        self.assertAlmostEqual(b.floor(6.0), 34.0 - below)
        self.assertAlmostEqual(b.floor(9.0), 36.0 - below)
        self.assertAlmostEqual(b.floor(2.0), 32.0 - below)

    def test_off_or_empty(self):
        self.assertIsNone(trajectory.TrimBand(cfg(False), [(5.0, 30.0)]).floor(5))
        self.assertIsNone(trajectory.TrimBand(cfg(), [(5.0, None)]).floor(5))


if __name__ == "__main__":
    unittest.main()
