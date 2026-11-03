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
        t, dt = 0.0, 0.05
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
        s = self.run_plant(10.0, noise=0.05, lag_s=0.1)
        self.assertAlmostEqual(s.centre, 40.0, delta=3.0)

    def test_follows_speed(self):
        speed = lambda t: 30.0 + 0.3 * t                # noqa: E731
        s = self.run_plant(30.0, seconds=200.0, speed_fn=speed)
        self.assertAlmostEqual(s.centre, 20.0 + 0.4 * speed(200.0), delta=4.0)

    def test_takeoff_roll(self):
        """0 -> 120 m/s in 30 s: the optimum moves 1.6 deg/s."""
        speed = lambda t: min(120.0, 4.0 * t)          # noqa: E731
        s = self.run_plant(5.0, seconds=40.0, speed_fn=speed, noise=0.01)
        self.assertAlmostEqual(s.centre, 20.0 + 0.4 * 120.0, delta=2.0)

    def test_no_hunting(self):
        cfg = Config()
        s = PitchSeeker(cfg, 70.0)
        rng = random.Random(3)
        t, cmd, seen = 0.0, s.command(0.0), []
        while t < 120.0:
            t += 0.05
            cmd = s.step(t, plant(cmd, 50.0) + rng.gauss(0.0, 0.01))
            if t > 60.0:
                seen.append(s.centre)
        self.assertLess(max(seen) - min(seen), 2.0)
        self.assertAlmostEqual(sum(seen) / len(seen), 40.0, delta=1.0)

    def test_clamped(self):
        cfg = Config()
        s = PitchSeeker(cfg, 200.0)
        self.assertEqual(s.centre, cfg.PITCH_MAX_DEG)
        self.assertTrue(math.isfinite(s.step(0.0, None)))


if __name__ == "__main__":
    unittest.main()
