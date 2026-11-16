import random
import unittest

from common.jitter import SAMPLES, Jitter


class TestJitter(unittest.TestCase):
    def test_reads_the_noise_of_a_still_quantity(self):
        rnd = random.Random(3)
        j = Jitter()
        for _ in range(400):
            j.update((100.0 + rnd.gauss(0, 5.0), rnd.gauss(0, 5.0), 0.0))
        # Two components of sd 5: a second difference is Rayleigh with
        # sigma 5 sqrt(6), median 1.1774 of that; over sqrt(6): 5.9.
        self.assertAlmostEqual(j.sd, 5.0 * 1.1774, delta=1.5)

    def test_a_steady_drift_is_not_noise(self):
        j = Jitter()
        for i in range(SAMPLES * 2):
            j.update((750.0 * i, 0.0, 0.0))
        self.assertLess(j.sd, 1e-6)

    def test_a_real_jump_is_not_noise(self):
        j = Jitter()
        for i in range(SAMPLES * 2):
            j.update((1000.0 if i == SAMPLES else 0.0, 0.0, 0.0))
        self.assertLess(j.sd, 1.0)

    def test_unknown_until_enough_samples(self):
        j = Jitter()
        for i in range(SAMPLES + 1):
            j.update((float(i), 0.0, 0.0))
        self.assertIsNone(j.sd)
        self.assertIsNone(j.band(2.0))
        j.update((99.0, 0.0, 0.0))
        self.assertIsNotNone(j.sd)


if __name__ == "__main__":
    unittest.main()
