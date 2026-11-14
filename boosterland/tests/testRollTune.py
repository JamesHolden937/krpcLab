"""boosterland/rolltune.py: kRPC's auto-tune reproduced, the rate cap."""

import math
import unittest

from boosterland import rolltune
from boosterland.autoland import nearest_slot, symmetry_order


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

    def test_time_to_peak_for_a_decay_time(self):
        zeta, omega = rolltune.damping_and_frequency(
            rolltune.time_to_peak_for(0.375, 0.01), 0.01)
        self.assertAlmostEqual(1.0 / (zeta * omega), 0.375, places=6)
        # kRPC's own 1 s decays in 0.217 s
        self.assertAlmostEqual(rolltune.time_to_peak_for(0.2172, 0.01), 1.0,
                               delta=0.01)

    def test_no_authority_no_gains(self):
        self.assertIsNone(rolltune.krpc_gains(None, 0.8, 5.0))
        self.assertIsNone(rolltune.krpc_gains(0.0, 0.8, 5.0))

    def test_rate_cap_stops_inside_the_brake_angle(self):
        cap = rolltune.rate_cap(1.5, 5.0, 1.0)
        self.assertAlmostEqual(cap * cap / (2 * 1.5), math.radians(5.0))
        self.assertEqual(rolltune.rate_cap(1e6, 5.0, 1.0), 1.0)
        self.assertEqual(rolltune.rate_cap(None, 5.0, 1.0), 1.0)


class TestRollSymmetry(unittest.TestCase):
    @staticmethod
    def ring(n, key="fin", radius=4.0, phase=0.3):
        return [(key, phase + 2 * math.pi * i / n, radius) for i in range(n)]

    def test_six_fins(self):
        self.assertEqual(symmetry_order(self.ring(6)), 6)

    def test_rings_combine_by_gcd(self):
        parts = self.ring(6) + self.ring(4, key="brake", radius=2.0)
        self.assertEqual(symmetry_order(parts), 2)

    def test_a_lone_fin_breaks_it(self):
        parts = self.ring(6) + [("tail", 1.0, 3.0)]
        self.assertEqual(symmetry_order(parts), 1)

    def test_uneven_ring_breaks_it(self):
        parts = self.ring(4)
        parts[1] = ("fin", parts[1][1] + 0.5, 4.0)
        self.assertEqual(symmetry_order(parts), 1)

    def test_on_axis_parts_do_not_count(self):
        self.assertEqual(symmetry_order(self.ring(3) + [("x", 0.0, 0.0)]), 3)
        self.assertEqual(symmetry_order([]), 1)

    def test_slot_hysteresis(self):
        slot = math.radians(60)
        self.assertEqual(nearest_slot(math.radians(20), slot, None, 0.15), 0)
        # past the midpoint but inside the hysteresis: kept
        self.assertEqual(nearest_slot(math.radians(33), slot, 0, 0.15), 0)
        # well past it: the next slot
        self.assertEqual(nearest_slot(math.radians(40), slot, 0, 0.15), 1)
        self.assertEqual(nearest_slot(math.radians(-100), slot, 0, 0.15), -2)


if __name__ == "__main__":
    unittest.main()
