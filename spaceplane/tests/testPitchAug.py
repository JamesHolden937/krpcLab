"""``PITCH_STIFF_AUG``: the moment table's slope and the law that tops the
airframe's pitch stiffness up to a target."""

import math
import unittest
from types import SimpleNamespace

from spaceplane import environment
from spaceplane.autopilot import stiff_aug
from spaceplane.config import Config

CARGO_I = 1.424e6                    # kg m^2, qs_cargo_rigoff (stiffprobe)
SURF = 2.0e5                         # N m per unit input


def cfg(**kw):
    c = Config()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class TestStiffAug(unittest.TestCase):
    def test_stiffer_airframe_gets_nothing(self):
        # The twin-fin shuttle at Mach 5: -5 m^3/deg at q 1199.
        m_alpha = -5.0 * 180.0 / math.pi * 1199.0
        self.assertEqual(stiff_aug(cfg(), m_alpha, 2.086e6, SURF, 40.0, 30.0),
                         (0.0, 0.0))

    def test_divergent_band_is_augmented_nose_down_when_high(self):
        # The cargo shuttle between 10 and 25 deg at Mach 5: +1 m^3/deg.
        m_alpha = 1.0 * 180.0 / math.pi * 1199.0
        gain, u = stiff_aug(cfg(), m_alpha, CARGO_I, SURF, 30.0, 20.0)
        want = (m_alpha + CARGO_I * 0.2 ** 2) / SURF
        self.assertAlmostEqual(gain, want, places=6)
        self.assertLess(u, 0.0)                       # alpha high: nose down
        _, below = stiff_aug(cfg(), m_alpha, CARGO_I, SURF, 15.0, 20.0)
        self.assertGreater(below, 0.0)

    def test_neutral_airframe_gets_the_target(self):
        gain, _ = stiff_aug(cfg(), 0.0, CARGO_I, SURF, 20.0, 20.0)
        self.assertAlmostEqual(gain * SURF, CARGO_I * 0.2 ** 2, places=3)

    def test_caps(self):
        c = cfg(PITCH_AUG_MAX_GAIN=1.0, PITCH_AUG_MAX_INPUT=0.3)
        gain, u = stiff_aug(c, 1e7, CARGO_I, SURF, 60.0, 20.0)
        self.assertEqual(gain, 1.0)
        self.assertEqual(u, -0.3)

    def test_no_authority_reading_is_no_augmentation(self):
        self.assertEqual(stiff_aug(cfg(), 1e5, CARGO_I, 0.0, 30.0, 20.0),
                         (0.0, 0.0))


class TestPitchStiffness(unittest.TestCase):
    def env(self, moments):
        alphas = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0)
        table = environment.Table(alphas, (0.0, 5.0))
        for row in range(2):
            for col, m in enumerate(moments):
                table.set(row, col, m)
        return SimpleNamespace(moment=table, cfg=Config())

    def test_slope_sign_follows_the_curve(self):
        # The cargo shuttle's Mach 5 row, nose-up positive (LOG13675).
        e = self.env([-15.1, -20.9, -20.8, -16.3, -10.5, -7.5, -22.7, -51.9])
        stiff = environment.Environment.pitch_stiffness
        self.assertGreater(stiff(e, 17.5, 5.0), 0.0)     # divergent band
        self.assertLess(stiff(e, 32.5, 5.0), 0.0)        # stable above it
        self.assertAlmostEqual(stiff(e, 17.5, 5.0),
                               (-10.5 + 16.3) / 5.0 * 180.0 / math.pi,
                               places=6)

    def test_no_table_is_none(self):
        e = SimpleNamespace(moment=environment.Table((0.0, 5.0), (0.0,)),
                            cfg=Config())
        self.assertIsNone(environment.Environment.pitch_stiffness(e, 5, 1))


if __name__ == "__main__":
    unittest.main()
