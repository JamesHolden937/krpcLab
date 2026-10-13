"""Offline checks that the guidance flies apsides changes in the stand-in:
it arrives, within a few percent of the impulsive Hohmann, and the planner
meets a deadline or says it cannot."""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import vec                                     # noqa: E402
from lowThrust import config, guidance, orbit, plan, sim   # noqa: E402

BODY = sim.KERBIN
CFG = config.Config()


def transfer(start, target, vessel=sim.ION_PROBE, eta=None):
    """Fly ``start`` (ap, pe km[, inc, lan, argpe deg]) to ``target`` (ap,
    pe km[, inc, lan deg]; None free).  Returns the flight, the target, the
    impulsive reference and the target plane's normal."""
    ang = [math.radians(x) for x in start[2:]]
    r, v = sim.state_at_periapsis(BODY, start[0] * 1e3, start[1] * 1e3, *ang)
    radii = [None if x is None else BODY.radius + x * 1e3 for x in target[:2]]
    angles = [None if x is None else math.radians(x) for x in target[2:]]
    tgt = guidance.Target(*(radii + angles))
    res = sim.fly(BODY, vessel, r, v, tgt,
                  CFG.ETA_DEFAULT if eta is None else eta, CFG)
    el0 = orbit.elements(r, v, BODY.mu)
    ra1 = tgt.ra if tgt.ra is not None else res.el.ra
    rp1 = tgt.rp if tgt.rp is not None else res.el.rp
    di = guidance.plane_error(el0, guidance.lock(tgt, el0, CFG))
    if tgt.ra is None and tgt.rp is None and tgt.inc is not None:
        locked = guidance.lock(tgt, el0, CFG)
        ref = orbit.plane_change_dv(el0, vec.unit(vec.cross(r, v)),
                                    locked.normal)
    else:
        ref = orbit.impulsive_dv(BODY.mu, el0.ra, el0.rp, ra1, rp1, di)
    return res, tgt, ref


class Checks:
    def check(self, start, target, worst, **kw):
        res, tgt, ref = transfer(start, target, **kw)
        self.assertTrue(res.done, res.reason)
        if tgt.ra is not None:
            self.assertLess(abs(res.el.ra - tgt.ra), CFG.TOLERANCE_M)
        if tgt.rp is not None:
            self.assertLess(abs(res.el.rp - tgt.rp), CFG.TOLERANCE_M)
        if tgt.inc is not None:
            inc, lan = orbit.inclination_lan(res.el.rvec, res.el.vvec)
            self.assertLess(abs(math.degrees(inc - tgt.inc)),
                            2 * CFG.PLANE_TOLERANCE_DEG)
            if tgt.lan is not None:
                self.assertLess(abs(math.degrees(lan - tgt.lan)), 0.05)
        self.assertLess(res.dv / ref - 1, worst)
        return res


class TestTransfers(Checks, unittest.TestCase):

    def test_raise_circular(self):
        self.check((100, 100), (500, 500), 0.02)

    def test_lower_circular(self):
        self.check((500, 500), (100, 100), 0.02)

    def test_circularise_at_apoapsis(self):
        self.check((2000, 100), (2000, 2000), 0.03)

    def test_one_apsis_free(self):
        res = self.check((1000, 80), (None, 300), 0.02)
        self.assertLess(abs(res.el.ra - (BODY.radius + 1000e3)), 5e3)

    def test_cross_over(self):
        self.check((800, 300), (600, 200), 0.02)

    def test_weak_engine(self):
        weak = sim.Vessel(4000.0, 1000.0, 200.0, 4200.0)
        self.check((100, 100), (300, 300), 0.02, vessel=weak)

    def test_apsides_burns_are_prograde_or_retrograde_only(self):
        el = orbit.elements(*sim.state_at_periapsis(BODY, 100e3, 100e3),
                            BODY.mu)
        tgt = guidance.lock(
            guidance.Target(BODY.radius + 500e3, BODY.radius + 500e3), el,
            CFG)
        cmd = guidance.command(el, tgt, 0.4, 10.0, CFG.ETA_DEFAULT, CFG,
                               BODY.mu)
        self.assertEqual((cmd.prograde, cmd.normal), (1.0, 0.0))
        self.assertGreater(cmd.throttle, 0)


class TestPlaneChange(Checks, unittest.TestCase):
    def test_inclination_on_a_circular_orbit(self):
        self.check((500, 500), (None, None, 10), 0.02)

    def test_plane_change_at_apoapsis(self):
        """An eccentric orbit's plane is changed at the apoapsis node."""
        self.check((2000, 100), (None, None, 5), 0.01)

    def test_node_longitude(self):
        self.check((500, 500, 30, 0), (None, None, 30, 40), 0.02)

    def test_raise_with_plane_change(self):
        self.check((100, 100), (500, 500, 5), 0.03)

    def test_lower_with_plane_change(self):
        """The plane changes on the big orbit, before the apoapsis drops."""
        self.check((3000, 3000), (100, 100, 20), 0.10)


class TestPlanner(unittest.TestCase):
    def setUp(self):
        self.r, self.v = sim.state_at_periapsis(BODY, 100e3, 100e3)
        self.tgt = guidance.Target(BODY.radius + 300e3, BODY.radius + 300e3)
        self.weak = sim.Vessel(4000.0, 1000.0, 200.0, 4200.0)

    def test_meets_a_deadline_and_trades_dv_for_it(self):
        free = plan.plan(BODY, self.weak, self.r, self.v, self.tgt, CFG)
        budget = free.result.time / 3
        p = plan.plan(BODY, self.weak, self.r, self.v, self.tgt, CFG, budget)
        self.assertTrue(p.feasible, p.reason)
        self.assertLessEqual(p.result.time, budget)
        self.assertGreater(p.result.dv, free.result.dv)

    def test_an_impossible_deadline_says_so(self):
        p = plan.plan(BODY, self.weak, self.r, self.v, self.tgt, CFG, 600.0)
        self.assertFalse(p.feasible)
        self.assertTrue(p.result.done)       # it still says what is possible
        self.assertGreater(p.result.time, 600.0)


if __name__ == "__main__":
    unittest.main()
