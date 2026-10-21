"""``SURFACE_OWN``: the client flies the attitude on every measured
surface, by incremental dynamic inversion and allocation."""

import math
import unittest
from types import SimpleNamespace

from common import vec
from spaceplane import allocate
from spaceplane import autopilot as ap
from spaceplane.config import Config

# Body frame: right +x, nose +y, belly +z; the roof is -z.  Raw torque and
# rate about +x are nose-down.
RIGHT, NOSE, ROOF = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)

CANARD = (-40.0, 0.0, 0.0)         # m^3 per unit: +deflection nose-up
ELEV_L = (-10.0, 8.0, 0.0)         # nose-up and roll one way
ELEV_R = (-10.0, -8.0, 0.0)        # nose-up and roll the other


def table(col):
    return [[col, col], [col, col]]


def pilot(layout, aero=(0.0, 0.0, 0.0)):
    cfg = Config()
    cfg.SURFACE_OWN = True
    cs = [SimpleNamespace(deflection=0.0, deflection_override=False)
          for _ in layout]
    flight = SimpleNamespace(
        simulate_aerodynamic_wrench_at=lambda *a: ((0, 0, 0), aero))
    env = SimpleNamespace(flight=flight, body=None, equatorial_radius=0.0,
                          mach=lambda v, h: 3.0)
    s = SimpleNamespace(
        cfg=cfg, state=ap.GLIDE, env=env,
        vessel=SimpleNamespace(moment_of_inertia=(1.4e6, 1e5, 1.4e6),
                               available_rcs_torque=((0.0, 0.0, 0.0),
                                                     (0.0, 0.0, 0.0))),
        rcs=SimpleNamespace(on=False),
        logbook=SimpleNamespace(event=lambda *a: None),
        autopilot=SimpleNamespace(engaged=True), autopilot_engaged=True,
        control=SimpleNamespace(pitch=0.0, roll=0.0, yaw=0.0),
        _surfaces=[(c, table(col)) for c, col in zip(cs, layout)],
        _surface_machs=(0.0, 10.0), _surface_alphas=(0.0, 40.0),
        commanded_nose=NOSE, commanded_up=ROOF)
    for name in ("surfaces_owned", "surface_columns", "own_attitude",
                 "release_surfaces"):
        setattr(s, name, getattr(ap.Autopilot, name).__get__(s))
    snap = SimpleNamespace(ut=10.0, position=(0, 1, 0), velocity=(0, 1, 0),
                           right=RIGHT, rotation=(0, 0, 0, 1),
                           angular_velocity=(0.0, 0.0, 0.0), roof=ROOF,
                           nose=NOSE, dynamic_pressure=1000.0, krpc_aoa=20.0)
    return s, snap, cs


def tilt(axis, deg):
    q = vec.quat_axis_angle(axis, math.radians(deg))
    return vec.quat_rotate(q, NOSE), vec.quat_rotate(q, ROOF)


def made(s, cs, layout):
    return tuple(sum(c.deflection * col[k] for c, col in zip(cs, layout))
                 for k in range(3))


class TestOwnAttitude(unittest.TestCase):
    def test_takes_the_surfaces_and_disengages_krpc(self):
        layout = [CANARD, CANARD]
        s, snap, cs = pilot(layout)
        s.own_attitude(snap)
        self.assertFalse(s.autopilot.engaged)
        self.assertTrue(all(c.deflection_override for c in cs))

    def test_nose_low_pitches_up_on_canards_and_elevons(self):
        layout = [CANARD, CANARD, ELEV_L, ELEV_R]
        s, snap, cs = pilot(layout)
        # The command is 5 deg nose-up of the nose: about -x (raw sense).
        s.commanded_nose, s.commanded_up = tilt(RIGHT, -5.0)
        s.own_attitude(snap)
        m = made(s, cs, layout)
        self.assertLess(m[0], 0.0)                    # nose-up torque
        self.assertAlmostEqual(m[1], 0.0, places=6)   # no roll
        self.assertAlmostEqual(cs[2].deflection, cs[3].deflection, places=6)

    def test_one_set_of_elevons_does_pitch_and_roll(self):
        layout = [ELEV_L, ELEV_R]
        s, snap, cs = pilot(layout)
        s.commanded_nose, s.commanded_up = tilt(NOSE, 4.0)   # a roll
        s.own_attitude(snap)
        m = made(s, cs, layout)
        self.assertGreater(m[1], 0.0)
        self.assertAlmostEqual(m[0], 0.0, places=6)
        self.assertGreater(cs[0].deflection, cs[1].deflection)

    def test_the_airframes_own_moment_is_cancelled(self):
        layout = [CANARD, CANARD]
        aero = (2e4, 0.0, 0.0)                        # nose-down 20 kN m
        s, snap, cs = pilot(layout, aero=aero)
        s.own_attitude(snap)
        self.assertAlmostEqual(made(s, cs, layout)[0] * 1000.0, -2e4,
                               delta=50.0)

    def test_hands_back(self):
        layout = [CANARD]
        s, snap, cs = pilot(layout)
        s.own_attitude(snap)
        s.release_surfaces(snap)
        self.assertFalse(cs[0].deflection_override)
        self.assertFalse(s._surfaces_on)

    def test_not_owned_without_measurement_or_outside_the_air(self):
        s, _, _ = pilot([CANARD])
        s.state = ap.COAST
        self.assertFalse(s.surfaces_owned())
        s, _, _ = pilot([CANARD])
        s._surfaces = []
        self.assertFalse(s.surfaces_owned())


if __name__ == "__main__":
    unittest.main()


class TestTrimmedLift(unittest.TestCase):
    def run_trim(self, base, cols):
        cfg = Config()
        seen = {}
        s = SimpleNamespace(cfg=cfg, logbook=SimpleNamespace(
            event=lambda ut, text: seen.setdefault("line", text)))
        ap.Autopilot.trimmed_lift(s, None if False else
                                  SimpleNamespace(ut=0.0), [10.0, 20.0, 30.0],
                                  base, cols)
        return s._trimmed_cla, seen["line"]

    def test_trim_costs_lift_and_an_untrimmable_alpha_does_not_count(self):
        # Nose-down moment growing with alpha (raw +x); one aft elevon whose
        # +1 is nose-up (-x, 10 m^3) and spoils 5 m^2 of lift.
        base = [((4.0, 0.0, 0.0), 100.0), ((8.0, 0.0, 0.0), 150.0),
                ((20.0, 0.0, 0.0), 190.0)]
        cols = [[((-10.0, 0.0, 0.0), -5.0)] * 3]
        cla, line = self.run_trim(base, cols)
        # 20 deg: 0.8 of travel, 150 - 4 = 146; 30 deg needs 2.0: refused.
        self.assertAlmostEqual(cla, 146.0, places=3)
        self.assertIn("30:", line)
        self.assertIn("x", line.split("30:")[1].split()[0])

    def test_a_neutral_craft_keeps_its_table_lift(self):
        base = [((0.0, 0.0, 0.0), 100.0), ((0.0, 0.0, 0.0), 150.0),
                ((0.0, 0.0, 0.0), 190.0)]
        cols = [[((-10.0, 0.0, 0.0), -5.0)] * 3]
        cla, _ = self.run_trim(base, cols)
        self.assertAlmostEqual(cla, 190.0, places=6)
