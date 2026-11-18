import math
import unittest

from recon import guidance as gd

R = 600000.0


class TestSphere(unittest.TestCase):
    def test_distance_and_bearing_east_on_equator(self):
        d = gd.distance(0.0, 0.0, 0.0, 1.0, R)
        self.assertAlmostEqual(d, R * math.radians(1.0), places=3)
        self.assertAlmostEqual(gd.bearing(0.0, 0.0, 0.0, 1.0), math.pi / 2)

    def test_offset_round_trip(self):
        lat, lon = gd.offset(-0.05, -74.7, math.radians(270.0), 12000.0, R)
        self.assertAlmostEqual(gd.distance(-0.05, -74.7, lat, lon, R), 12000.0,
                               places=1)
        self.assertAlmostEqual(math.degrees(gd.bearing(-0.05, -74.7, lat, lon)),
                               270.0, places=2)

    def test_along_cross_sign(self):
        # A point north of an eastbound line is left of it (cross < 0).
        along, cross = gd.along_cross(0.01, 0.05, 0.0, 0.0, math.pi / 2, R)
        self.assertGreater(along, 0.0)
        self.assertLess(cross, 0.0)


class TestLoad(unittest.TestCase):
    def test_budget_leaves_room_for_thrust(self):
        self.assertAlmostEqual(gd.normal_g_budget(15.0, 9.0), 12.0)
        self.assertEqual(gd.normal_g_budget(15.0, 16.0), 0.0)

    def test_bank_capped_by_load(self):
        b = gd.bank_for_turn(1.0, 1500.0, 13.0)
        self.assertAlmostEqual(math.tan(b), math.sqrt(13.0 ** 2 - 1.0))
        self.assertLess(abs(gd.bank_for_turn(-0.01, 100.0, 13.0)), 0.2)


class TestHeat(unittest.TestCase):
    def test_throttle_shuts_on_the_prediction(self):
        self.assertEqual(gd.heat_throttle(0.5, 0.0, 0.85, 6.0, 0.08), 1.0)
        # 0.80 rising 0.02/s is 0.92 in 6 s: shut.
        self.assertEqual(gd.heat_throttle(0.80, 0.02, 0.85, 6.0, 0.08), 0.0)
        mid = gd.heat_throttle(0.81, 0.0, 0.85, 6.0, 0.08)
        self.assertTrue(0.0 < mid < 1.0)


class TestEnergy(unittest.TestCase):
    def test_coast_speed(self):
        self.assertAlmostEqual(gd.coast_speed(100.0, 0.0, 1e-4), 100.0)
        self.assertAlmostEqual(gd.coast_speed(100.0, 1e4, 1e-4),
                               100.0 * math.e)

    def test_flight_path_command_clamped(self):
        g = gd.flight_path_command(20000.0, 0.0, 300.0, 10.0,
                                   math.radians(30.0), math.radians(-15.0))
        self.assertAlmostEqual(g, math.radians(30.0))

    def test_localizer_turns_toward_the_line(self):
        # Right of an eastbound centreline: steer left of east.
        c = gd.localizer_course(math.pi / 2, 300.0, 100.0, 12.0,
                                math.radians(30.0))
        self.assertLess(c, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
