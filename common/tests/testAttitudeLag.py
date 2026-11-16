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
    last = None             # the command in force since the last update
    while t < seconds:
        a = 0.3 * math.sin(0.7 * t) + 0.2 * math.sin(1.9 * t + 1.0)
        b = 0.25 * math.cos(1.3 * t)
        cmd = vec.unit((math.sin(a), math.cos(a) * math.cos(b), math.sin(b)))
        seen = nose
        if noise:
            seen = vec.unit(tuple(x + rnd.gauss(0.0, noise) for x in nose))
        fit.update(t, seen, last)
        last = cmd
        e = vec.norm(error_vector(nose, cmd))
        step = e * (1.0 - math.exp(-lam * dt))
        if slew is not None:
            step = min(step, slew * dt)
        nose = rotate_toward(nose, cmd, step)
        t += dt
    return fit


def delayed(delay, dt=0.04, seconds=20.0):
    """A nose that is exactly the command ``delay`` seconds ago (a loop that
    tracks a moving command with no standing error, as kRPC's PI does)."""
    fit = LagFit()
    cmds = []
    t = 0.0
    last = None
    while t < seconds:
        a = 0.3 * math.sin(0.7 * t) + 0.2 * math.sin(1.9 * t + 1.0)
        b = 0.25 * math.cos(1.3 * t)
        cmds.append((t, vec.unit((math.sin(a), math.cos(a) * math.cos(b),
                                  math.sin(b)))))
        old = [c for tc, c in cmds if tc <= t - delay]
        nose = old[-1] if old else cmds[0][1]
        fit.update(t, nose, last)
        last = cmds[-1][1]
        t += dt
    return fit


class TestLagFit(unittest.TestCase):
    def test_a_pure_delay_is_read_as_its_delay(self):
        for delay in (0.12, 0.3, 0.6):
            fit = delayed(delay)
            self.assertIsNotNone(fit.lag)
            self.assertAlmostEqual(fit.lag, delay, delta=0.05)

    def test_first_order_reads_about_its_time_constant(self):
        for lam in (0.7, 1.5, 3.0):
            fit = fly(lam, dt=0.04)
            self.assertIsNotNone(fit.rate)
            # A first-order lag delays a wandering command by less than its
            # time constant (phase delay atan(w/lam)/w < 1/lam).
            self.assertGreater(fit.lag, 0.4 / lam)
            self.assertLess(fit.lag, 1.2 / lam)

    def test_noisy(self):
        fit = fly(1.0, dt=0.04, noise=0.002)
        self.assertIsNotNone(fit.rate)
        self.assertAlmostEqual(fit.lag, 1.0, delta=0.4)

    def test_rate_limited_reads_slower(self):
        fast = fly(3.0, dt=0.04)
        limited = fly(3.0, dt=0.04, slew=0.1)
        self.assertGreater(limited.lag, fast.lag)

    def test_pinned_ticks_ignored(self):
        fit = LagFit()
        fit.update(0.0, (0, 1, 0), (1, 0, 0), saturation=1.0)
        fit.update(0.1, (0.1, 1, 0), (1, 0, 0), saturation=1.0)
        self.assertEqual(fit.samples, 0)
        self.assertIsNone(fit.rate)

    def test_unknown_until_measured(self):
        fit = LagFit()
        for i in range(30):
            fit.update(0.04 * i, (0, 1, 0), (0, 1, 0))
        self.assertIsNone(fit.rate)


if __name__ == "__main__":
    unittest.main()
