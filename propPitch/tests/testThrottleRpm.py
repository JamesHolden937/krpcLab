"""The throttle sets the rotors' RPM limit."""

import types
import unittest

from propPitch.autopilot import PropPilot
from propPitch.config import Config


def pilot(**over):
    cfg = Config()
    for k, v in over.items():
        setattr(cfg, k, v)
    p = types.SimpleNamespace(cfg=cfg, rpm_cmd=cfg.ROTOR_RPM,
                              rotors=[types.SimpleNamespace(target_rpm=0.0)])
    return p


class TestThrottleRpm(unittest.TestCase):
    def test_scales_with_throttle(self):
        p = pilot()
        self.assertEqual(PropPilot.throttle_rpm(p, 1.0), 460.0)
        self.assertEqual(PropPilot.throttle_rpm(p, 0.5), 230.0)
        self.assertEqual(PropPilot.throttle_rpm(p, 0.0), 0.0)
        self.assertEqual(PropPilot.throttle_rpm(p, 1.7), 460.0)

    def test_feather_band(self):
        p = pilot()
        self.assertEqual(PropPilot.throttle_rpm(p, 0.02), 0.0)
        self.assertGreater(PropPilot.throttle_rpm(p, 0.03), 13.0)
        p.rpm_cmd = 0.0
        self.assertTrue(PropPilot.feathered(p, False))
        self.assertFalse(PropPilot.feathered(p, True))       # braking

    def test_floor_and_off(self):
        self.assertEqual(PropPilot.throttle_rpm(
            pilot(THROTTLE_RPM_MIN=100.0), 0.1), 100.0)
        self.assertEqual(PropPilot.throttle_rpm(
            pilot(THROTTLE_RPM=False), 0.1), 460.0)
        self.assertEqual(PropPilot.throttle_rpm(pilot(), None), 460.0)

    def test_writes_only_on_a_step_or_an_end(self):
        p = pilot()
        PropPilot.set_rpm(p, 0.0, 459.0)
        self.assertEqual(p.rotors[0].target_rpm, 0.0)       # under the step
        PropPilot.set_rpm(p, 0.0, 300.0)
        self.assertEqual(p.rotors[0].target_rpm, 300.0)
        PropPilot.set_rpm(p, 0.0, 301.0)
        self.assertEqual(p.rpm_cmd, 300.0)
        PropPilot.set_rpm(p, 0.0, 1.0)
        PropPilot.set_rpm(p, 0.0, 0.0)                      # cut: exact 0
        self.assertEqual(p.rotors[0].target_rpm, 0.0)
        self.assertEqual(p.rpm_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
