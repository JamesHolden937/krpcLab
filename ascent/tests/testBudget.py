"""The separation budget against the one booster boostback measured in game.

LOG12758 (qs_heavy, defaults bb3fa2e6): separated at 22583 m, 25464 m from
the pad, 790.8 m/s away and 394.2 up, 93.10 t; five Vectors, 5000 kN vacuum,
Isp 315/295.  The boostback took it to 62.08 t -- 1251 m/s at Isp 315 -- and
the landing burn from 60.58 to 54.28 t, ~330 m/s, starting at ~242 m/s
with its airbrakes open (which the budget does not model: it errs high).

LOG16183/16184 (the Heavy Shuttle booster, live, 13 Vectors, 230.6 t, dry
122.3 t): separated at 20971 m, 20810 m out, 783 m/s away and 462 up; the
fall's apex 34.2 km; the landing burn lit at 5.2 km doing 680 m/s (623 down,
288 across) and flamed out at 29 m after ~737 m/s, still 18 m/s down: it
needed ~850 m/s.  The stack's zero-alpha Cd*A at 5 km is the ascent log's
"cda" row.
"""

import math
import unittest

from ascent import budget

# Kerbin's air every 2 km from the pad (kspSim.atmosphere, kg/m^3)
AIR = (1.12996, 0.88351, 0.68594, 0.52595, 0.39552, 0.27809, 0.18987,
       0.12929, 0.08763, 0.05865, 0.03947, 0.0267, 0.01807, 0.01233, 0.00832,
       0.00564, 0.00389, 0.00272, 0.0019, 0.00134, 0.00099, 0.00074, 0.00055,
       0.00041, 0.0003, 0.00022, 0.00016, 0.00011, 8e-05, 6e-05, 4e-05,
       2e-05, 2e-05, 1e-05, 0.0, 0.0)
KERBIN = budget.Body(mu=3.5316e12, radius=600072.0, density_pad=1.13,
                     density=AIR, step=2000.0)
HEAVY = budget.Booster(mass=93100.0, dry_mass=40000.0, thrust_vac=5.0e6,
                       thrust_sl=4.687e6, isp_vac=315.0, isp_sl=295.0,
                       radius=1.875)
SEP = budget.State(height=22583.0, downrange=25464.0, v_away=790.8,
                   v_cross=0.0, v_up=394.2)
SHUTTLE = budget.Booster(mass=230620.0, dry_mass=122300.0, thrust_vac=13.0e6,
                         thrust_sl=12.185e6, isp_vac=315.0, isp_sl=295.0,
                         radius=2.61, gimbal_accel=1.139, fixed_accel=0.056)
SHUTTLE_SEP = budget.State(height=20971.0, downrange=20810.0, v_away=783.3,
                           v_cross=0.0, v_up=461.7)
_SPEEDS = (100, 200, 300, 350, 400, 500, 700, 1000, 1500, 2200)
_CDA = (27.8, 24.8, 20.7, 24.2, 22.9, 17.7, 13.0, 12.2, 12.0, 13.0)


def shuttle_cda(h, v):
    if v <= _SPEEDS[0]:
        return _CDA[0]
    for i in range(len(_SPEEDS) - 1):
        if v <= _SPEEDS[i + 1]:
            f = (v - _SPEEDS[i]) / (_SPEEDS[i + 1] - _SPEEDS[i])
            return _CDA[i] * (1 - f) + _CDA[i + 1] * f
    return _CDA[-1]


class TestBudget(unittest.TestCase):
    def test_boostback_matches_the_measured_burn(self):
        rec = budget.recovery(KERBIN, HEAVY, SEP)
        self.assertAlmostEqual(rec.boostback_dv, 1251.0, delta=0.05 * 1251.0)

    def test_landing_errs_high_without_the_airbrakes(self):
        rec = budget.recovery(KERBIN, HEAVY, SEP)
        self.assertGreater(rec.landing_dv, 330.0)
        self.assertGreater(rec.terminal, 242.0)

    def test_landing_matches_the_shuttle_boosters_burn(self):
        rec = budget.recovery(KERBIN, SHUTTLE, SHUTTLE_SEP, cda=shuttle_cda)
        self.assertAlmostEqual(rec.apex, 34200.0, delta=0.05 * 34200.0)
        self.assertAlmostEqual(rec.terminal, 680.0, delta=0.05 * 680.0)
        self.assertAlmostEqual(rec.landing_dv, 850.0, delta=0.05 * 850.0)

    def test_the_shuttle_booster_could_not_have_separated_there(self):
        # it flamed out short of the ground: the budget has to say so
        rec = budget.recovery(KERBIN, SHUTTLE, SHUTTLE_SEP, cda=shuttle_cda)
        need = rec.propellant - rec.propellant_left
        self.assertLess(rec.propellant_left, 0.15 * need)

    def test_air_without_a_profile_is_refused(self):
        bare = budget.Body(mu=3.5316e12, radius=600072.0, density_pad=1.13)
        with self.assertRaises(ValueError):
            budget.recovery(bare, HEAVY, SEP)

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
