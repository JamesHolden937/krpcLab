import math
import unittest

from ascent import budget, turnshape as ts

MU, R = 3.5316e12, 600070.0


def kerbin():
    pres = [math.exp(-k * 1000.0 / 5600.0) for k in range(71)] + [0.0]
    return ts.World(budget.Body(MU, R, 1.225), 2 * math.pi / 21549.425,
                    pres, [1.225 * p for p in pres])


def stack():
    b = budget.Booster(mass=264e3, dry_mass=40e3, thrust_vac=5e6,
                       thrust_sl=4.68e6, isp_vac=315, isp_sl=295, radius=2.5)
    return ts.Craft(b, ts.Engines(5e6, 315, 295), upper_mass=101e3,
                    upper_propellant=60e3,
                    upper_engines=ts.Engines(1.2e6, 330, 250),
                    upper_lit=False, heights=[0.0],
                    speeds=[0, 300, 340, 400, 1000, 3000],
                    cda=[[4, 4, 9, 8, 5, 4]])


class TestTurnShape(unittest.TestCase):
    def test_pitch_is_mechjebs_classic_path(self):
        path = ts.Path(start=1000.0, end=61000.0)
        self.assertEqual(ts.pitch_deg(500.0, path, 0.4), 90.0)
        self.assertEqual(ts.pitch_deg(70000.0, path, 0.4), 0.0)
        self.assertAlmostEqual(ts.pitch_deg(31000.0, path, 0.4),
                               90.0 - 0.5 ** 0.4 * 90.0)

    def test_turn_measured_from_where_the_speed_started_it(self):
        # MechJeb leaves the vertical at its start speed below the start
        # height and runs the curve from there (LOG13036: from ~1250 m)
        path = ts.Path(start=3500.0, end=59500.0, start_speed=99.0)
        self.assertAlmostEqual(ts.pitch_deg(1944.0, path, 0.46, 1250.0),
                               90.0 - (694.0 / 58250.0) ** 0.46 * 90.0)
        self.assertEqual(ts.pitch_deg(1944.0, path, 0.46), 90.0)

    def test_circular_orbit_apoapsis(self):
        w = kerbin()
        r = R + 80000.0
        ra, va = ts.apsides(w, r, 0.0, math.sqrt(MU / r))
        self.assertAlmostEqual(ra, r, delta=1.0)
        self.assertIsNone(ts.apsides(w, r, 0.0, 2 * math.sqrt(MU / r))[0])

    def test_optimum_is_inside_and_extremes_lose(self):
        best, flown = ts.solve(stack(), kerbin(), ts.Path(3500.0, 60000.0))
        self.assertTrue(0.15 < best.shape < 0.9, best.shape)
        steep = min(flown, key=lambda r: r.shape)
        flat = max(flown, key=lambda r: r.shape)
        self.assertGreater(best.mass, steep.mass)
        self.assertGreater(best.mass, flat.mass)

    def test_a_flatter_turn_leaves_the_booster_lower_and_further_out(self):
        w, c, p = kerbin(), stack(), ts.Path(3500.0, 60000.0)
        flat, steep = ts.fly(c, w, p, 0.3), ts.fly(c, w, p, 0.8)
        self.assertLess(flat.sep_height, steep.sep_height)
        self.assertGreater(flat.sep_downrange, steep.sep_downrange)


if __name__ == "__main__":
    unittest.main()
