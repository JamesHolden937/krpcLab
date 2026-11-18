import unittest

from recon import thermal as th


def cockpit(t_s=500.0, t_i=400.0):
    p = th.PartHeat("cockpit", 1100.0, 2000.0)
    p.a_c, p.e_a, p.g, p.c_s, p.c_i = 1.0, 2.0, 0.05, 20.0, 800.0
    p.t_s, p.t_i = t_s, t_i
    return p


class TestFormulas(unittest.TestCase):
    def test_lerp_ends(self):
        self.assertEqual(th.mach_lerp(1.0), 0.0)
        self.assertEqual(th.mach_lerp(6.0), 1.0)

    def test_shock_temperature_high_mach(self):
        self.assertAlmostEqual(th.shock_temperature(1500.0, 5.0),
                               21.0 * 1500.0 ** 0.75)


class TestPrediction(unittest.TestCase):
    def test_skin_settles_and_interior_lags(self):
        p = cockpit()
        p.s_max = p.t_max = 1e9
        f60, ts60, ti60 = p.peak(0.05, 1500.0, 5.0, 220.0, 60.0)
        f600, ts600, ti600 = p.peak(0.05, 1500.0, 5.0, 220.0, 600.0)
        self.assertAlmostEqual(ts60, ts600, delta=20.0)   # skin at equilibrium
        self.assertGreater(ti600, ti60 + 50.0)            # interior still rising
        self.assertGreaterEqual(f600, f60)

    def test_longer_flight_is_slower(self):
        # The interior binds (a skin that takes anything): a long flight
        # soaks it through.
        def part():
            p = cockpit()
            p.s_max = 1e9
            p.g, p.c_i = 0.5, 200.0
            return p
        short = th.speed_limit([part()], 0.05, 300.0, 220.0, 100e3, 0.85)
        long_ = th.speed_limit([part()], 0.05, 300.0, 220.0, 3000e3, 0.85)
        self.assertGreater(short, long_)

    def test_thinner_air_is_faster(self):
        thick = th.speed_limit([cockpit()], 0.2, 300.0, 220.0, 500e3, 0.85)
        thin = th.speed_limit([cockpit()], 0.02, 300.0, 220.0, 500e3, 0.85)
        self.assertGreater(thin, thick)

    def test_unmeasured_says_nothing(self):
        self.assertIsNone(th.speed_limit([th.PartHeat("x", 1, 1)], 0.1, 300.0,
                                         220.0, 1e5, 0.85))


if __name__ == "__main__":
    unittest.main()
