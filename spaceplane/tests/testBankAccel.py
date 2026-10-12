"""``guidance.bank_accel`` and ``guidance.ease_bank``: the bank command is
slewed as the vehicle's torque can stop it."""

import unittest

from spaceplane import guidance

SHUTTLE_I = (1.63e6, 7.4e4, 1.66e6)          # pitch, roll, yaw kg m^2
SURF = (2.0e6, 2.2e6, 2.2e5)                 # N m, q ~2 kPa


class TestBankAccel(unittest.TestCase):
    def test_yaw_limits_at_high_alpha(self):
        high = guidance.bank_accel(SURF, SHUTTLE_I, 40.0)
        low = guidance.bank_accel(SURF, SHUTTLE_I, 5.0)
        self.assertLess(high, low)
        yaw_only = 57.2958 * SURF[2] / SHUTTLE_I[2] / 0.6428
        self.assertAlmostEqual(high, yaw_only, delta=0.5)

    def test_zero_alpha_is_body_roll(self):
        got = guidance.bank_accel(SURF, SHUTTLE_I, 0.0)
        self.assertAlmostEqual(got, 57.2958 * SURF[1] / SHUTTLE_I[1],
                               delta=1.0)

    def test_no_reading(self):
        self.assertIsNone(guidance.bank_accel(None, SHUTTLE_I, 10.0))


class TestEaseBank(unittest.TestCase):
    def fly(self, accel, dt=0.1):
        bank, rate, peak = -40.0, 0.0, -40.0
        for _ in range(400):
            bank, rate = guidance.ease_bank(bank, 40.0, rate, 13.5, dt, accel)
            peak = max(peak, bank)
        return bank, rate, peak

    def test_arrives_without_overshoot_or_rate(self):
        bank, rate, peak = self.fly(5.0)
        self.assertAlmostEqual(bank, 40.0, places=6)
        self.assertEqual(rate, 0.0)
        self.assertLessEqual(peak, 40.0)

    def test_respects_rate_limit(self):
        bank, rate = guidance.ease_bank(0.0, 90.0, 13.5, 13.5, 0.1, 1e6)
        self.assertAlmostEqual(rate, 13.5, places=6)

    def test_accelerates_from_rest(self):
        bank, rate = guidance.ease_bank(0.0, 90.0, 0.0, 13.5, 0.1, 5.0)
        self.assertAlmostEqual(rate, 0.5, places=6)

    def test_no_accel_is_the_old_clamp(self):
        bank, rate = guidance.ease_bank(0.0, 90.0, 0.0, 13.5, 0.1, None)
        self.assertAlmostEqual(bank, 1.35, places=6)


if __name__ == "__main__":
    unittest.main()
