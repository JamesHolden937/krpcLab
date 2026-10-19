"""The separation budget against the one booster boostback measured in game.

LOG12758 (qs_heavy, defaults bb3fa2e6): separated at 22583 m, 25464 m from
the pad, 790.8 m/s away and 394.2 up, 93.10 t; five Vectors, 5000 kN vacuum,
Isp 315/295.  The boostback took it to 62.08 t -- 1251 m/s at Isp 315 -- and
the landing burn from 60.58 to 54.28 t, ~330 m/s, starting at ~242 m/s.
"""

import math
import unittest

from ascent import budget

KERBIN = budget.Body(mu=3.5316e12, radius=600072.0, density_pad=1.18)
HEAVY = budget.Booster(mass=93100.0, dry_mass=40000.0, thrust_vac=5.0e6,
                       thrust_sl=4.687e6, isp_vac=315.0, isp_sl=295.0,
                       radius=1.875)
SEP = budget.State(height=22583.0, downrange=25464.0, v_away=790.8,
                   v_cross=0.0, v_up=394.2)


class TestBudget(unittest.TestCase):
    def test_boostback_matches_the_measured_burn(self):
        rec = budget.recovery(KERBIN, HEAVY, SEP)
        self.assertAlmostEqual(rec.boostback_dv, 1251.0, delta=0.05 * 1251.0)

    def test_landing_matches_the_measured_burn(self):
        rec = budget.recovery(KERBIN, HEAVY, SEP)
        self.assertAlmostEqual(rec.landing_dv, 330.0, delta=0.15 * 330.0)
        self.assertGreater(rec.terminal, 242.0)     # errs high: keeps fuel

    def test_further_and_faster_needs_more(self):
        base = budget.recovery(KERBIN, HEAVY, SEP)
        later = budget.recovery(KERBIN, HEAVY, budget.State(
            30000.0, 40000.0, 1000.0, 0.0, 450.0))
        self.assertGreater(later.boostback_dv, base.boostback_dv)

    def test_a_slower_flip_costs_more(self):
        fast = budget.recovery(KERBIN, HEAVY, SEP, flip_rate_deg=60.0)
        slow = budget.recovery(KERBIN, HEAVY, SEP, flip_rate_deg=15.0)
        self.assertGreater(slow.boostback_dv, fast.boostback_dv)

    def test_cross_track_is_charged(self):
        side = budget.recovery(KERBIN, HEAVY, budget.State(
            SEP.height, SEP.downrange, SEP.v_away, 200.0, SEP.v_up))
        base = budget.recovery(KERBIN, HEAVY, SEP)
        self.assertGreater(side.boostback_dv, base.boostback_dv)

    def test_an_empty_booster_cannot_return(self):
        dry = budget.Booster(**{**HEAVY.__dict__, "mass": 41000.0})
        self.assertLess(budget.recovery(KERBIN, dry, SEP).propellant_left, 0.0)

    def test_no_air_lands_from_the_vacuum_fall(self):
        moon = budget.Body(mu=6.5138e10, radius=200000.0, density_pad=0.0)
        rec = budget.recovery(moon, HEAVY, budget.State(5000.0, 2000.0, 100.0,
                                                        0.0, 50.0))
        self.assertTrue(math.isfinite(rec.landing_dv))

    def test_separation_target(self):
        target, frac = budget.separation_target(20.0, 100000.0, 4.0e6, 0.75)
        self.assertAlmostEqual(target, 1.5e6)
        self.assertAlmostEqual(frac, 0.375)


if __name__ == "__main__":
    unittest.main()
