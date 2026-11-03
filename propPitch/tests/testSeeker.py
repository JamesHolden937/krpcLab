import math
import random
import unittest

from propPitch.config import Config
from propPitch.seeker import PitchSeeker, solve


def plant(pitch, speed):
    """A thrust curve whose best pitch rises with airspeed (the inflow)."""
    best = 20.0 + 0.4 * speed
    return 2.0 - 0.004 * (pitch - best) ** 2 - 0.0002 * speed * speed


class TestSeeker(unittest.TestCase):
    def test_solve(self):
        x = solve([[2.0, 1.0], [1.0, 3.0]], [3.0, 5.0])
        self.assertAlmostEqual(x[0], 0.8)
        self.assertAlmostEqual(x[1], 1.4)

    def run_plant(self, start, seconds=200.0, speed_fn=lambda t: 50.0,
                  noise=0.0, lag_s=0.0):
        cfg = Config()
        s = PitchSeeker(cfg, start)
        rng = random.Random(1)
        t, dt = 0.0, 0.1
        cmd = s.command(0.0)
        hist = []
        while t < seconds:
            hist.append(cmd)
            delayed = hist[max(0, len(hist) - 1 - int(lag_s / dt))]
            j = plant(delayed, speed_fn(t)) + rng.gauss(0.0, noise)
            t += dt
            cmd = s.step(t, j)
        return s

    def test_climbs_from_either_side(self):
        for start in (5.0, 70.0):
            s = self.run_plant(start)
            self.assertAlmostEqual(s.centre, 40.0, delta=2.0)

    def test_noise_and_lag(self):
        s = self.run_plant(10.0, noise=0.05, lag_s=0.3)
        self.assertAlmostEqual(s.centre, 40.0, delta=3.0)

    def test_follows_speed(self):
        speed = lambda t: 30.0 + 0.3 * t                # noqa: E731
        s = self.run_plant(30.0, seconds=200.0, speed_fn=speed)
        self.assertAlmostEqual(s.centre, 20.0 + 0.4 * speed(200.0), delta=4.0)

    def test_clamped(self):
        cfg = Config()
        s = PitchSeeker(cfg, 200.0)
        self.assertEqual(s.centre, cfg.PITCH_MAX_DEG)
        self.assertTrue(math.isfinite(s.step(0.0, None)))


if __name__ == "__main__":
    unittest.main()
