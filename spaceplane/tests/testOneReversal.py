import math
import unittest

from spaceplane import trajectory
from spaceplane.config import Config


class ReversalBand(unittest.TestCase):
    def test_turn_left_is_ld_sin_bank_log_speed(self):
        cfg = Config()
        cfg.ONE_REVERSAL_MAX_DEG = 90.0
        got = trajectory.reversal_band(cfg, (1.0, 100.0, 0.0), 100.0 * math.e,
                                       30.0)
        self.assertAlmostEqual(got, math.degrees(0.5), places=6)

    def test_capped_and_floored(self):
        cfg = Config()
        self.assertEqual(trajectory.reversal_band(cfg, (2.0, 100.0, 5.0),
                                                  2000.0, 45.0),
                         cfg.ONE_REVERSAL_MAX_DEG)
        # Speed already at the arrival: no turn left, the floor holds.
        self.assertEqual(trajectory.reversal_band(cfg, (2.0, 100.0, 5.0),
                                                  90.0, 45.0), 5.0)


class LeanSign(unittest.TestCase):
    R = 600000.0
    r = (R, 0.0, 0.0)
    v = (0.0, 0.0, 2000.0)                      # north along +z

    def gate_at(self, deg):
        """A gate 100 km away, ``deg`` off the track toward +y."""
        a = math.radians(deg)
        d = 100000.0
        return (self.R, d * math.sin(a), d * math.cos(a))

    def test_holds_inside_the_band(self):
        self.assertEqual(trajectory.lean_sign(self.r, self.v,
                                              self.gate_at(10.0), -1.0, 15.0),
                         -1.0)

    def test_past_the_band_leans_toward_the_gate_either_side(self):
        a = trajectory.lean_sign(self.r, self.v, self.gate_at(20.0), 1.0, 15.0)
        b = trajectory.lean_sign(self.r, self.v, self.gate_at(-20.0), 1.0,
                                 15.0)
        self.assertEqual(a, -b)
        # The sign chosen for a gate is the one that holds once it is chosen.
        self.assertEqual(trajectory.lean_sign(self.r, self.v,
                                              self.gate_at(20.0), a, 15.0), a)


if __name__ == "__main__":
    unittest.main()
