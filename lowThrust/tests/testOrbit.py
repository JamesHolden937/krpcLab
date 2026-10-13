"""Offline checks of the two-body maths the guidance and the stand-in share."""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import vec                                  # noqa: E402
from lowThrust import orbit, sim                        # noqa: E402

MU = sim.KERBIN.mu
R0 = sim.KERBIN.radius


class TestOrbit(unittest.TestCase):
    def test_elements_of_an_ellipse(self):
        r, v = sim.state_at_periapsis(sim.KERBIN, 2000e3, 100e3)
        el = orbit.elements(r, v, MU)
        self.assertAlmostEqual(el.ra, R0 + 2000e3, delta=1e-3)
        self.assertAlmostEqual(el.rp, R0 + 100e3, delta=1e-3)
        self.assertAlmostEqual(el.nu, 0.0, places=9)

    def test_unbound_is_none(self):
        r = (R0 + 100e3, 0.0, 0.0)
        v = (0.0, 1.01 * math.sqrt(2 * MU / r[0]), 0.0)
        self.assertIsNone(orbit.elements(r, v, MU))

    def test_coast_is_exact(self):
        """A full period comes back to the start; half reaches apoapsis."""
        r, v = sim.state_at_periapsis(sim.KERBIN, 2000e3, 100e3)
        el = orbit.elements(r, v, MU)
        r1, v1 = orbit.coast(r, v, MU, el.period)
        self.assertLess(vec.norm(vec.sub(r1, r)), 1e-3)
        r2, _ = orbit.coast(r, v, MU, el.period / 2)
        self.assertAlmostEqual(vec.norm(r2), el.ra, delta=1e-3)

    def test_time_to(self):
        r, v = sim.state_at_periapsis(sim.KERBIN, 2000e3, 100e3)
        el = orbit.elements(r, v, MU)
        self.assertAlmostEqual(orbit.time_to(el, math.pi, MU),
                               el.period / 2, delta=1e-6)
        r2, v2 = orbit.coast(r, v, MU, 1000.0)
        el2 = orbit.elements(r2, v2, MU)
        self.assertAlmostEqual(orbit.time_to(el2, 0.0, MU),
                               el.period - 1000.0, delta=1e-3)

    def test_radius_toward_the_apsides(self):
        r, v = sim.state_at_periapsis(sim.KERBIN, 2000e3, 100e3)
        el = orbit.elements(r, v, MU)
        pe_dir = vec.unit(el.evec)
        self.assertAlmostEqual(orbit.radius_toward(el.p, el.evec, pe_dir),
                               el.rp, delta=1e-3)
        self.assertAlmostEqual(
            orbit.radius_toward(el.p, el.evec, vec.scale(pe_dir, -1.0)),
            el.ra, delta=1e-3)

    def test_impulsive_dv_is_hohmann(self):
        r1, r2 = R0 + 100e3, R0 + 500e3
        at = (r1 + r2) / 2
        hohmann = (math.sqrt(MU / r1) * (math.sqrt(r2 / at) - 1)
                   + math.sqrt(MU / r2) * (1 - math.sqrt(r1 / at)))
        self.assertAlmostEqual(orbit.impulsive_dv(MU, r1, r1, r2, r2),
                               hohmann, delta=1e-6)
        self.assertAlmostEqual(orbit.impulsive_dv(MU, r2, r2, r1, r1),
                               hohmann, delta=1e-6)


if __name__ == "__main__":
    unittest.main()


class TestConics(unittest.TestCase):
    def test_propagate_matches_the_elliptic_coast(self):
        r, v = sim.state_at_periapsis(sim.KERBIN, 2000e3, 100e3, 0.3, 1.0, 0.5)
        for dt in (100.0, 3000.0, 20000.0):
            a, b = orbit.coast(r, v, MU, dt), orbit.propagate(r, v, MU, dt)
            self.assertLess(vec.norm(vec.sub(a[0], b[0])), 1e-3)

    def test_hyperbola_round_trip_and_periapsis_time(self):
        r, v = (700e3, 0.0, 0.0), (0.0, 3500.0, 0.0)
        r2, v2 = orbit.propagate(r, v, MU, 50000.0)
        r3, _ = orbit.propagate(r2, v2, MU, -50000.0)
        self.assertLess(vec.norm(vec.sub(r3, r)), 1e-2)
        c = orbit.conic(r2, v2, MU)
        self.assertGreater(c.e, 1)
        self.assertAlmostEqual(orbit.time_to_periapsis(c, MU), -50000.0,
                               delta=1e-3)
        r4, v4 = orbit.propagate(r, v, MU, -20000.0)
        self.assertAlmostEqual(
            orbit.time_to_periapsis(orbit.conic(r4, v4, MU), MU), 20000.0,
            delta=1e-3)

    def test_lambert(self):
        r1 = (7e5, 0.0, 0.0)
        for r2, tof in (((9e5 * math.cos(2.1), 9e5 * math.sin(2.1), 0.0), 2500),
                        ((9e5 * math.cos(4.4), 9e5 * math.sin(4.4), 0.0), 4000),
                        ((0.0, 9e5, 3e5), 3000), ((0.0, 5e7, 0.0), 20000)):
            v1, v2 = orbit.lambert(r1, r2, tof, MU)
            rr, vv = orbit.propagate(r1, v1, MU, tof)
            self.assertLess(vec.norm(vec.sub(rr, r2)), 1e-2)
            self.assertLess(vec.norm(vec.sub(vv, v2)), 1e-6)
