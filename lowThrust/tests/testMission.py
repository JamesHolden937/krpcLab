"""Offline checks of the transfers between bodies in the stand-in solar
system: the window, the parking-ellipse geometry, and whole missions to a
moon, a planet and a planet's moon, strong and weak engines."""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import vec                                          # noqa: E402
from lowThrust import (bodies, config, guidance, mission, orbit,   # noqa: E402
                       sim, transfer)

SYS = bodies.stock()
CFG = config.Config()
DAY = 21600.0


def fly(dest, alt_km, thrust=2000.0, inc=None):
    k = SYS["Kerbin"]
    d = SYS[dest]
    r, v = orbit.state_from_elements(k.mu, k.radius + 100e3, 0, 0, 0, 0, 0)
    ves = sim.Vessel(5000.0, thrust, 4200.0)
    tgt = guidance.Target(d.radius + alt_km * 1e3, d.radius + alt_km * 1e3,
                          None if inc is None else math.radians(inc))
    return mission.fly(SYS, ves, k, r, v, 0.0, d, tgt, CFG), d


class TestGeometry(unittest.TestCase):
    def test_soi_and_periods_are_stock(self):
        self.assertAlmostEqual(SYS["Kerbin"].soi / 1e3, 84159, delta=5)
        self.assertAlmostEqual(SYS["Mun"].soi / 1e3, 2430, delta=2)
        self.assertAlmostEqual(SYS["Duna"].period / DAY, 801.6, delta=0.5)

    def test_first_duna_window(self):
        k, d, sun = SYS["Kerbin"], SYS["Duna"], SYS["Sun"]
        w = transfer.window(k, d, 0.0, sun.mu, 300 * DAY)
        self.assertAlmostEqual(w.t_dep / DAY, 230, delta=10)
        self.assertAlmostEqual(vec.norm(w.vinf_dep), 860, delta=60)

    def test_parking_ellipse_leaves_on_the_asymptote(self):
        k = SYS["Kerbin"]
        vinf = (600.0, -500.0, 80.0)
        rp, ra = k.radius + 100e3, k.radius + 7000e3
        n, P, _ = transfer.parking_orientation(k.mu, rp, ra, vinf,
                                               (0.0, 0.0, 1.0))
        e, nu = transfer.departure_hyperbola(k.mu, rp, vec.norm(vinf))
        Q = vec.cross(n, P)
        out = vec.add(vec.scale(P, math.cos(nu)), vec.scale(Q, math.sin(nu)))
        self.assertGreater(vec.dot(out, vec.unit(vinf)), 0.9999)
        self.assertGreater(n[2], 0)

    def test_routes(self):
        k = SYS["Kerbin"]
        self.assertEqual([b.name for b in mission.route(k, SYS["Ike"])],
                         ["Duna", "Ike"])
        self.assertEqual([b.name for b in mission.route(k, SYS["Mun"])],
                         ["Mun"])
        with self.assertRaises(ValueError):
            mission.route(SYS["Mun"], SYS["Ike"])


class TestLongCapture(unittest.TestCase):
    """A capture too long for one pass round periapsis starts when it must,
    outside the SOI if need be (Moho: v_inf 3.5 km/s, SOI 9.6 Mm)."""
    MU, R, SOI = 1.686e11, 250e3, 9646663.0

    def cap(self):
        return transfer.Capture(self.MU, 0.3 * self.SOI, 0.5 * self.SOI,
                                self.R + 30e3, 1.5 * (self.R + 50e3), self.R,
                                math.radians(60), 0.8, 0.8)

    def test_every_engine_captures(self):
        rc = self.R + 50e3
        impulsive = transfer.capture_dv(self.MU, rc, 0.3 * self.SOI, 3497.0)
        starts = []
        for thrust in (4000.0, 2000.0, 500.0, 100.0):
            got = transfer.capture_start(self.cap(), rc, 3497.0,
                                         thrust / 5000.0)
            self.assertIsNotNone(got, thrust)
            tau, dv, out = got
            self.assertGreater(dv, impulsive)
            self.assertLess(dv, 3497.0)
            starts.append(out)
        self.assertGreater(starts[1], self.SOI)        # outside the SOI
        self.assertEqual(starts, sorted(starts))       # weaker: earlier

    def test_coasts_until_needed_then_throttles_steadily(self):
        cap = self.cap()
        rc = self.R + 50e3
        vp = math.sqrt(3497.0 ** 2 + 2 * self.MU / rc)
        far = orbit.propagate((rc, 0.0, 0.0), (0.0, vp, 0.0), self.MU,
                              -30000.0)
        c = orbit.conic(*far, self.MU)
        rho, wait = transfer.capture_step(*far, c, cap, 0.4, started=False)
        self.assertEqual(rho, 0.0)
        self.assertGreater(wait, 1000.0)
        near = orbit.propagate((rc, 0.0, 0.0), (0.0, vp, 0.0), self.MU,
                               -3000.0)
        c = orbit.conic(*near, self.MU)
        rho, _ = transfer.capture_step(*near, c, cap, 0.4, started=True)
        self.assertGreater(rho, 0.5)


class TestMissions(unittest.TestCase):
    def check(self, res, dest, alt_km, dv_max):
        self.assertTrue(res.done, res.reason)
        self.assertIs(res.body, dest)
        c = res.final
        self.assertLess(abs(c.ra - dest.radius - alt_km * 1e3),
                        CFG.TOLERANCE_M)
        self.assertLess(abs(c.rp - dest.radius - alt_km * 1e3),
                        CFG.TOLERANCE_M)
        self.assertLess(res.dv, dv_max)

    def test_to_the_mun(self):
        res, d = fly("Mun", 50)
        self.check(res, d, 50, 1250)

    def test_to_minmus_out_of_plane(self):
        res, d = fly("Minmus", 30)
        self.check(res, d, 30, 1300)

    def test_to_duna_with_a_plane_change(self):
        res, d = fly("Duna", 200, inc=10)
        self.check(res, d, 200, 2000)
        inc, _ = orbit.inclination_lan(res.final.rvec, res.final.vvec)
        self.assertAlmostEqual(math.degrees(inc), 10, delta=0.05)

    def test_to_ike_through_duna(self):
        res, d = fly("Ike", 50)
        self.check(res, d, 50, 2100)

    def test_to_moho_matching_in_the_cruise(self):
        """v_inf 3.5 km/s: part shed about the Sun (MATCH), the rest in a
        long capture inside the SOI."""
        res, d = fly("Moho", 50)
        self.check(res, d, 50, 6700)
        self.assertTrue(any(e.text.startswith("MATCH") for e in res.events))

    def test_weak_engine_to_moho(self):
        """TWR 0.004: nearly all of v_inf shed about the Sun."""
        res, d = fly("Moho", 50, thrust=200.0)
        self.check(res, d, 50, 7500)

    def test_weak_engine_to_the_mun(self):
        """TWR 0.004: captures high and loose, then spirals down."""
        res, d = fly("Mun", 50, thrust=200.0)
        self.check(res, d, 50, 1700)


if __name__ == "__main__":
    unittest.main()
