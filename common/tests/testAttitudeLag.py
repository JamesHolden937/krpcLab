import math
import random
import unittest

from common import vec
from common.attitudelag import LagFit, error_vector


def rotate_toward(n, c, angle):
    """``n`` turned ``angle`` radians toward ``c``."""
    perp = vec.project_out(c, n)
    if vec.norm(perp) < 1e-12:
        return n
    p = vec.unit(perp)
    return vec.unit(vec.add(vec.scale(n, math.cos(angle)),
                            vec.scale(p, math.sin(angle))))


def fly(lam, dt=0.1, seconds=40.0, slew=None, noise=0.0, seed=1):
    """A nose following a command that wanders: first order at ``lam``,
    optionally rate-limited to ``slew`` rad/s, with ``noise`` rad of jitter."""
    rnd = random.Random(seed)
    fit = LagFit()
    nose = (0.0, 1.0, 0.0)
    t = 0.0
    while t < seconds:
        a = 0.3 * math.sin(0.7 * t) + 0.2 * math.sin(1.9 * t + 1.0)
        b = 0.25 * math.cos(1.3 * t)
        cmd = vec.unit((math.sin(a), math.cos(a) * math.cos(b), math.sin(b)))
        seen = nose
        if noise:
            seen = vec.unit(tuple(x + rnd.gauss(0.0, noise) for x in nose))
        fit.update(t, seen, cmd)
        e = vec.norm(error_vector(nose, cmd))
        step = e * (1.0 - math.exp(-lam * dt))
        if slew is not None:
            step = min(step, slew * dt)
        nose = rotate_toward(nose, cmd, step)
        t += dt
    return fit


class TestLagFit(unittest.TestCase):
    def test_first_order(self):
        for lam in (0.5, 1.0, 3.0):
            fit = fly(lam)
            # The discrete step 1-exp(-lam dt) over dt reads a little under lam.
            expected = (1.0 - math.exp(-lam * 0.1)) / 0.1
            self.assertIsNotNone(fit.rate)
            self.assertAlmostEqual(fit.rate, expected, delta=0.02 * expected)

    def test_noisy(self):
        fit = fly(1.0, noise=0.002)
        self.assertIsNotNone(fit.rate)
        self.assertAlmostEqual(fit.rate, 0.95, delta=0.15)

    def test_rate_limited_reads_slower(self):
        fast = fly(2.0)
        limited = fly(2.0, slew=0.2)
        self.assertLess(limited.fit, fast.fit)

    def test_pinned_ticks_ignored(self):
        fit = LagFit()
        fit.update(0.0, (0, 1, 0), (1, 0, 0), saturation=1.0)
        fit.update(0.1, (0.1, 1, 0), (1, 0, 0), saturation=1.0)
        self.assertEqual(fit.samples, 0)
        self.assertIsNone(fit.rate)

    def test_unknown_until_measured(self):
        fit = LagFit()
        fit.update(0.0, (0, 1, 0), (0, 1, 0))
        fit.update(0.1, (0, 1, 0), (0, 1, 0))
        self.assertIsNone(fit.rate)


if __name__ == "__main__":
    unittest.main()
