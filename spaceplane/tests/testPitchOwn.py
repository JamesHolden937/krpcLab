"""``PITCH_OWN``: the client's pitch law by incremental dynamic inversion,
and the nose kRPC is left to fly."""

import math
import unittest
from types import SimpleNamespace

from common import vec
from spaceplane import autopilot as ap
from spaceplane.config import Config

# Body frame for the test: nose +y, roof +z, right +x.  The game's raw
# torque about +x is nose-down; (w x nose).roof is the nose-up rate.
NOSE, ROOF, RIGHT = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)


class Stub(SimpleNamespace):
    pass


def pilot(torque_x=0.0, state=ap.GLIDE, valve=False, total=0.0):
    cfg = Config()
    cfg.PITCH_OWN = True
    sent = []
    flight = SimpleNamespace(
        simulate_aerodynamic_wrench_at=lambda *a: ((0, 0, 0),
                                                   (torque_x, 0.0, 0.0)))
    s = Stub(cfg=cfg, state=state, env=SimpleNamespace(flight=flight,
                                                       body=None),
             vessel=SimpleNamespace(moment_of_inertia=(1.4e6, 1e5, 1.4e6),
                                    available_control_surface_torque=(
                                        (2e5, 2e5, 2e4), (-2e5, -2e5, -2e4)),
                                    available_rcs_torque=((2.8e5, 4e4, 2.8e5),
                                                          (0, 0, 0))),
             rcs=SimpleNamespace(on=valve),
             logbook=SimpleNamespace(event=lambda *a: None),
             autopilot=SimpleNamespace(time_to_peak=(3, 3, 3)),
             _tuned_peak=(3.0, 3.0, 3.0), _pitch_assist_sent=0.0)
    s.control = SimpleNamespace()

    class Ctl:
        def __setattr__(self, k, v):
            sent.append(v)
    s.control = Ctl()
    for name in ("pitch_owned", "_owned_ttp", "_own_ttp", "_send_pitch",
                 "own_pitch", "krpc_nose"):
        setattr(s, name, getattr(ap.Autopilot, name).__get__(s))
    snap = SimpleNamespace(ut=10.0, position=(0, 0, 0), velocity=(0, 1, 0),
                           right=RIGHT, rotation=(0, 0, 0, 1),
                           angular_velocity=(0.0, 0.0, 0.0), roof=ROOF,
                           nose=NOSE, pitch_input=total)
    return s, snap, sent


class TestOwnPitch(unittest.TestCase):
    def test_nose_low_error_pitches_up(self):
        s, snap, sent = pilot()
        s.own_pitch(snap, 5.0)
        self.assertGreater(sent[-1], 0.0)

    def test_a_nose_up_moment_is_cancelled(self):
        # Raw torque about +x negative = nose-up moment; no error, no rate.
        s, snap, sent = pilot(torque_x=-5e4)
        s.own_pitch(snap, 0.0)
        self.assertAlmostEqual(sent[-1], -5e4 / 2e5, places=6)

    def test_nose_up_rate_is_damped(self):
        s, snap, sent = pilot()
        # Here (right-handed) w about +x turns the nose toward the roof.
        snap.angular_velocity = (0.1, 0.0, 0.0)
        self.assertGreater(vec.dot(vec.cross(snap.angular_velocity, NOSE),
                                   ROOF), 0.0)
        s.own_pitch(snap, 0.0)
        self.assertLess(sent[-1], 0.0)

    def test_krpc_residual_is_subtracted(self):
        # The read-back is 0.3 of which the client sent 0: kRPC's 0.3.
        s, snap, sent = pilot(torque_x=0.0, total=0.3)
        s.own_pitch(snap, 0.0)
        self.assertEqual(sent, [])                    # 0 wanted, 0 sent
        self.assertEqual(s._pitch_assist_sent, 0.0)

    def test_kRPC_pitch_tune_is_slowed_while_owned(self):
        s, snap, _ = pilot()
        s.own_pitch(snap, 0.0)
        self.assertEqual(s.autopilot.time_to_peak[0],
                         s.cfg.PITCH_OWN_KRPC_TTP_S)

    def test_not_owned_outside_the_air(self):
        s, _, _ = pilot(state=ap.COAST)
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
