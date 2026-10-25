"""TrimFloor: the learned lowest holdable alpha (Config.TRIM_FLOOR)."""
import unittest

from spaceplane import trajectory
from spaceplane.config import Config


def cfg(on=True):
    c = Config()
    c.TRIM_FLOOR = on
    return c


class TestTrimFloor(unittest.TestCase):
    def test_no_evidence_is_none(self):
        self.assertIsNone(trajectory.TrimFloor(cfg()).limit(5.0))

    def test_saturated_nose_high_sets_the_lowest_sample(self):
        f = trajectory.TrimFloor(cfg())
        for a in (38.0, 34.0, 36.0, 35.0):
            f.observe(25.0, a, -0.95, 5.1)
        self.assertAlmostEqual(f.limit(5.2), 34.0 + cfg().TRIM_FLOOR_MARGIN_DEG)

    def test_off_returns_none(self):
        f = trajectory.TrimFloor(cfg(False))
        for a in (38.0, 34.0, 36.0, 35.0):
            f.observe(25.0, a, -0.95, 5.1)
        self.assertIsNone(f.limit(5.1))

    def test_unsaturated_or_tracking_learns_nothing_new(self):
        f = trajectory.TrimFloor(cfg())
        for _ in range(6):
            f.observe(25.0, 34.0, -0.3, 5.1)     # nose high, input clear
            f.observe(25.0, 25.5, -0.3, 5.1)     # tracking, no bin yet
        self.assertIsNone(f.limit(5.1))

    def test_tracking_lowers_a_bin(self):
        f = trajectory.TrimFloor(cfg())
        for _ in range(4):
            f.observe(25.0, 34.0, -0.9, 5.1)
        f.observe(28.0, 28.5, -0.5, 5.1)
        self.assertAlmostEqual(f.limit(5.1), 28.0 + cfg().TRIM_FLOOR_MARGIN_DEG)

    def test_reach(self):
        f = trajectory.TrimFloor(cfg())
        for _ in range(4):
            f.observe(25.0, 34.0, -0.9, 5.1)
        self.assertIsNotNone(f.limit(4.0))
        self.assertIsNone(f.limit(2.0))


if __name__ == "__main__":
    unittest.main()
