import random
import unittest

from common.jitter import SAMPLES, Jitter


class TestJitter(unittest.TestCase):
    def test_reads_the_noise_of_a_still_quantity(self):
        rnd = random.Random(3)
        j = Jitter()
        for _ in range(400):
            j.update((100.0 + rnd.gauss(0, 5.0), rnd.gauss(0, 5.0), 0.0))
        # Two components of sd 5: the vector's sd is 5*sqrt(2).
        self.assertAlmostEqual(j.sd, 5.0 * 2 ** 0.5, delta=2.0)

    def test_unknown_until_enough_samples(self):
        j = Jitter()
        for i in range(SAMPLES):
            j.update((float(i), 0.0, 0.0))
        self.assertIsNone(j.sd)
        self.assertIsNone(j.band(2.0))
        j.update((99.0, 0.0, 0.0))
        self.assertIsNotNone(j.sd)


if __name__ == "__main__":
    unittest.main()
