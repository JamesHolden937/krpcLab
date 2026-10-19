import unittest

from ascent.attitude import pad_roll

# (up, north, east)
NORTH = (0.0, 1.0, 0.0)
SOUTH = (0.0, -1.0, 0.0)
EAST = (0.0, 0.0, 1.0)
WEST = (0.0, 0.0, -1.0)


class TestPadRoll(unittest.TestCase):
    def test_wings_level_for_the_heading(self):
        # Flying east, the right wing is south: no roll.
        self.assertAlmostEqual(pad_roll(SOUTH, 90.0), 0.0)
        self.assertAlmostEqual(pad_roll(EAST, 0.0), 0.0)

    def test_right_wing_down_is_positive(self):
        # Pitched over towards the east, the belly (down) is east-facing on
        # the pad: a right wing pointing east is rolled 90 right.
        self.assertAlmostEqual(pad_roll(EAST, 90.0), 90.0)
        self.assertAlmostEqual(pad_roll(WEST, 90.0), -90.0)
        self.assertAlmostEqual(abs(pad_roll(NORTH, 90.0)), 180.0)

    def test_a_north_launch_differs_by_the_heading_change(self):
        # The same vehicle against a north pitch-over: 90 deg apart.
        self.assertAlmostEqual(abs(pad_roll(SOUTH, 90.0) - pad_roll(SOUTH, 0.0)),
                               90.0)


if __name__ == "__main__":
    unittest.main()
