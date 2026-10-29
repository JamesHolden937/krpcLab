"""``hac_roll_floor``: the cone's roll floor follows the yaw share of a bank."""

import unittest

from spaceplane.autopilot import hac_roll_floor
from spaceplane.config import Config


class TestHacRollFloor(unittest.TestCase):
    def test_full_share_at_reference_alpha(self):
        cfg = Config()
        self.assertAlmostEqual(hac_roll_floor(cfg, 10.0, 30.0), 10.0, places=6)
        self.assertAlmostEqual(hac_roll_floor(cfg, 10.0, 45.0), 10.0, places=6)

    def test_low_alpha_barely_floors(self):
        cfg = Config()
        self.assertLess(hac_roll_floor(cfg, 10.0, 5.0), 2.8)
        self.assertLess(hac_roll_floor(cfg, 10.0, -7.0), 2.8)
        self.assertEqual(hac_roll_floor(cfg, 10.0, None), 0.0)

    def test_departure_alpha_floored(self):
        # LOG14776 departed above ~16 deg of commanded alpha with roll at 2.8.
        self.assertGreater(hac_roll_floor(Config(), 10.0, 16.0), 5.0)


class TestCoordRollFloor(unittest.TestCase):
    """``ROLL_COORD_FLOOR``: roll held back only as far as the rudder needs."""

    def floor(self, bank, alpha, yaw_torque, inertia=1.49e6):
        from spaceplane.autopilot import Autopilot
        ap = Autopilot.__new__(Autopilot)
        ap.cfg = Config()
        ap._bank_torque = (0.0, 0.0, yaw_torque)
        ap._bank_inertia = (0.0, 0.0, inertia)
        return ap.coord_roll_floor(bank, alpha)

    def test_cone_rudder_frees_roll(self):
        # LOG14863's cone: bank 45, alpha ~5, wheels 15 + rudder 333 kN m.
        self.assertLess(self.floor(45.0, 5.0, 348e3), 1.0)

    def test_glide_wheels_only_holds_roll(self):
        # Mach 6 at q ~900 Pa: bank 27, alpha 31, ~36 kN m of yaw.
        self.assertGreater(self.floor(27.4, 31.0, 36e3), 4.0)

    def test_no_reading_is_none(self):
        self.assertIsNone(self.floor(30.0, 30.0, 0.0))


class TestYawAirPeak(unittest.TestCase):
    """``YAW_AIR_TUNE``: yaw from wheels + rudder, bounded by static and floor."""

    def peak(self, torque, on=True, state=None):
        from spaceplane import autopilot as ap_mod
        ap = ap_mod.Autopilot.__new__(ap_mod.Autopilot)
        ap.cfg = Config()
        ap.cfg.YAW_AIR_TUNE = on
        ap.state = state or ap_mod.GLIDE
        ap._bank_torque = (0.0, 0.0, torque)
        ap._bank_inertia = (0.0, 0.0, 1.49e6)
        return ap.yaw_air_peak(10.0)

    def test_off_is_static(self):
        self.assertEqual(self.peak(36e3, on=False), 10.0)

    def test_glide_rudder_speeds_yaw(self):
        self.assertAlmostEqual(self.peak(36e3), (1.49e6 / 36e3) ** 0.5, places=3)

    def test_floor_and_static_bound(self):
        self.assertEqual(self.peak(348e3), Config().ATTITUDE_TIME_TO_PEAK_S)
        self.assertEqual(self.peak(5e3), 10.0)


if __name__ == "__main__":
    unittest.main()
