"""``PITCH_OWN``: the canards fly pitch by incremental dynamic inversion,
and kRPC is left the yaw error only."""

import math
import unittest
from types import SimpleNamespace

from common import vec
from spaceplane import autopilot as ap
from spaceplane.config import Config

# Body frame for the test: nose +y, roof +z, right +x (right-handed here).
# The game's raw torque about +x is nose-down; (w x nose).roof is the
# nose-up rate.
NOSE, ROOF, RIGHT = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
POWER = 40.0                             # m^3 per unit deflection


class Stub(SimpleNamespace):
    pass


def pilot(torque_x=0.0, state=ap.GLIDE, alpha=10.0):
    cfg = Config()
    cfg.PITCH_OWN = True
    canards = [SimpleNamespace(deflection=0.0, deflection_override=False)
               for _ in range(2)]
    flight = SimpleNamespace(
        simulate_aerodynamic_wrench_at=lambda *a: ((0, 0, 0),
                                                   (torque_x, 0.0, 0.0)))
    env = SimpleNamespace(flight=flight, body=None, equatorial_radius=0.0,
                          mach=lambda v, h: 5.0)
    s = Stub(cfg=cfg, state=state, env=env,
             vessel=SimpleNamespace(moment_of_inertia=(1.4e6, 1e5, 1.4e6)),
             logbook=SimpleNamespace(event=lambda *a: None),
             autopilot=SimpleNamespace(time_to_peak=(3, 3, 3)),
             _tuned_peak=(3.0, 3.0, 3.0), _canards=canards,
             _canard_power=[(0.0, POWER), (10.0, POWER)])
    for name in ("pitch_owned", "_owned_ttp", "_own_ttp", "own_pitch",
                 "krpc_nose", "canard_power"):
        setattr(s, name, getattr(ap.Autopilot, name).__get__(s))
    snap = SimpleNamespace(ut=10.0, position=(0, 1, 0), velocity=(0, 1, 0),
                           right=RIGHT, rotation=(0, 0, 0, 1),
                           angular_velocity=(0.0, 0.0, 0.0), roof=ROOF,
                           nose=NOSE, dynamic_pressure=1000.0,
                           krpc_aoa=alpha)
    return s, snap, canards


class TestOwnPitch(unittest.TestCase):
    def test_nose_low_error_raises_the_canards(self):
        s, snap, canards = pilot()
        s.own_pitch(snap, 5.0)
        self.assertGreater(canards[0].deflection, 0.0)
        self.assertTrue(all(c.deflection_override for c in canards))

    def test_a_nose_up_moment_is_cancelled(self):
        # Raw torque about +x negative = a nose-up moment; no error, no rate.
        s, snap, canards = pilot(torque_x=-1e4)
        s.own_pitch(snap, 0.0)
        self.assertAlmostEqual(canards[0].deflection, -1e4 / (POWER * 1000.0),
                               places=6)

    def test_nose_up_rate_is_damped(self):
        s, snap, canards = pilot()
        snap.angular_velocity = (0.1, 0.0, 0.0)     # nose-up in this frame
        self.assertGreater(vec.dot(vec.cross(snap.angular_velocity, NOSE),
                                   ROOF), 0.0)
        s.own_pitch(snap, 0.0)
        self.assertLess(canards[0].deflection, 0.0)

    def test_increments_from_the_last_deflection(self):
        s, snap, canards = pilot(torque_x=-1e4)
        s.own_pitch(snap, 0.0)
        first = s._own_defl
        s.own_pitch(snap, 0.0)                      # moment unchanged
        self.assertAlmostEqual(s._own_defl, 2 * first, places=9)

    def test_stall_caps_nose_up_deflection(self):
        s, snap, canards = pilot(alpha=27.0)        # local cap (28-27)/37.5
        s.own_pitch(snap, 30.0)
        self.assertLessEqual(canards[0].deflection, 1.0 / 37.5 + 1e-9)

    def test_kRPC_pitch_tune_is_slowed_while_owned(self):
        s, snap, _ = pilot()
        s.own_pitch(snap, 0.0)
        self.assertEqual(s.autopilot.time_to_peak[0],
                         s.cfg.PITCH_OWN_KRPC_TTP_S)

    def test_not_owned_without_measured_canards_or_outside_the_air(self):
        s, _, _ = pilot(state=ap.COAST)
        self.assertFalse(s.pitch_owned())
        s, _, _ = pilot()
        s._canard_power = []
        self.assertFalse(s.pitch_owned())


class TestKrpcNose(unittest.TestCase):
    def test_pitch_component_is_removed_yaw_kept(self):
        s, snap, _ = pilot()
        a, b = math.radians(10.0), math.radians(4.0)
        cmd = vec.unit((math.sin(b), math.cos(a), math.sin(a)))
        got = s.krpc_nose(cmd, snap)
        self.assertAlmostEqual(vec.dot(got, ROOF), 0.0, places=9)
        self.assertGreater(vec.dot(got, RIGHT), 0.0)

    def test_untouched_when_not_owned(self):
        s, snap, _ = pilot(state=ap.COAST)
        cmd = vec.unit((0.0, 1.0, 0.3))
        self.assertEqual(s.krpc_nose(cmd, snap), cmd)


if __name__ == "__main__":
    unittest.main()
