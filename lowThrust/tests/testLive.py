"""Offline checks of live.py's conversions (no game): elements from a state
vector round-trip, and the frame swap is its own inverse."""

import math
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from lowThrust import bodies, live, orbit                    # noqa: E402


class TestLive(unittest.TestCase):
    def test_angles_round_trip(self):
        rng = random.Random(1)
        mu = 3.5316e12
        for _ in range(500):
            a = rng.uniform(7e5, 5e7)
            e = rng.choice([0.0, rng.uniform(0, 0.9)])
            inc = rng.choice([0.0, rng.uniform(0, math.pi)])
            el = (a, e, inc, rng.uniform(0, 2 * math.pi),
                  rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi))
            r, v = orbit.state_from_elements(mu, *el)
            r2, v2 = orbit.state_from_elements(mu, *live.angles(r, v, mu))
            for x, y in zip(r + v, r2 + v2):
                self.assertAlmostEqual(x, y, delta=1e-6 * a)

    def test_body_rails_from_one_state(self):
        # a Body built from its own state at some UT moves as the original
        sys_ = bodies.stock()
        duna = sys_["Duna"]
        t = 4.0e6
        r, v = duna.state(t)
        a, e, inc, lan, argpe, nu = live.angles(r, v, duna.parent.mu)
        n = math.sqrt(duna.parent.mu / a ** 3)
        m0 = (live.mean_from_true(nu, e) - n * t) % (2 * math.pi)
        copy = bodies.Body("x", duna.mu, duna.radius, 0, duna.parent, a, e,
                           inc, lan, argpe, m0)
        for dt in (0, 1e5, 3e7):
            p, q = duna.position(t + dt), copy.position(t + dt)
            self.assertLess(math.dist(p, q), 1.0)

    def test_swap_is_involution(self):
        x = (1.0, 2.0, 3.0)
        self.assertEqual(live.to_krpc(live.to_ours(x)), x)
        self.assertEqual(live.to_ours(x), (1.0, 3.0, 2.0))


if __name__ == "__main__":
    unittest.main()
