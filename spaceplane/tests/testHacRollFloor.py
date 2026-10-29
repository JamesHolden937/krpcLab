"""``hac_roll_floor``: the cone's roll floor follows the yaw share of a bank."""

import unittest

from spaceplane.autopilot import hac_roll_floor
from spaceplane.config import Config


class TestHacRollFloor(unittest.TestCase):
    def test_full_share_at_reference_alpha(self):
        cfg = Config()
        self.assertAlmostEqual(hac_roll_floor(cfg, 10.0, 30.0), 10.0, places=6)
        self.assertAlmostEqual(hac_roll_floor(cfg, 10.0, 45.0), 10.0, places=6)

    def test_low_alpha_barely_floors(self):
        cfg = Config()
        self.assertLess(hac_roll_floor(cfg, 10.0, 5.0), 2.8)
        self.assertLess(hac_roll_floor(cfg, 10.0, -7.0), 2.8)
        self.assertEqual(hac_roll_floor(cfg, 10.0, None), 0.0)

    def test_departure_alpha_floored(self):
        # LOG14776 departed above ~16 deg of commanded alpha with roll at 2.8.
        self.assertGreater(hac_roll_floor(Config(), 10.0, 16.0), 5.0)


if __name__ == "__main__":
    unittest.main()
