import math
import os
import unittest

from propPitch import bladeModel, bladeTable
from propPitch.bladeModel import Blade, Curve, PropModel, Rotor, choose_signs

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# qs_prop_noon's front-left rotor, read off the farm (vessel frame, relative
# to the rotor's origin): four R-25s, origin 0.21 m out, x axis pointing in.
GEOM = [((-0.212, 0.313, -0.016), (0.997, 0.0, 0.073), (-0.073, 0.0, 0.997)),
        ((-0.016, 0.313, 0.212), (0.073, 0.0, -0.997), (0.997, 0.0, 0.073)),
        ((0.212, 0.313, 0.015), (-0.997, 0.0, -0.073), (0.073, 0.0, -0.997)),
        ((0.015, 0.313, -0.212), (-0.073, 0.0, 0.997), (-0.997, 0.0, -0.073))]


def rotor():
    kind = bladeModel.blade_type("largeFanBlade")
    r = Rotor((0.0, 1.0, 0.0), [Blade(kind, o, x, z, up=(0.0, 1.0, 0.0))
                                for o, x, z in GEOM], max_torque=70.0)
    choose_signs([r], 460.0)
    return r


class TestCurve(unittest.TestCase):
    def test_keys_and_clamp(self):
        c = Curve([(0, 0, 12, 12), (0.04, 0.6, 10, 10), (0.08, 1, 0.5, 0.5)])
        self.assertAlmostEqual(c(0.04), 0.6)
        self.assertAlmostEqual(c(-1), 0.0)
        self.assertAlmostEqual(c(5), 1.0)
        self.assertTrue(0.0 < c(0.02) < 0.6)


class TestModel(unittest.TestCase):
    def test_signs_consistent(self):
        r = rotor()
        self.assertEqual(len({b.sign for b in r.blades}), 1)

    def test_radius(self):
        r = rotor()
        for b in r.blades:
            self.assertAlmostEqual(math.hypot(b.point[0], b.point[2]), 6.01,
                                   delta=0.02)

    def test_optimum_tracks_inflow(self):
        """At rest the best angle leaves the blade near its best incidence
        (90 - 4.6 deg); the inflow angle atan(V / omega r) takes it down."""
        m = PropModel([rotor()])
        rest = m.optimum([460.0], (0.0, 0.0, 0.0), 1.2, 340.0)
        fast = m.optimum([460.0], (0.0, 90.0, 0.0), 1.2, 340.0)
        self.assertGreater(rest.thrust, 0.0)
        self.assertAlmostEqual(rest.deploy, 85.4, delta=1.5)
        phi = math.degrees(math.atan2(90.0, 460 * math.pi / 30 * 6.01))
        self.assertAlmostEqual(fast.deploy, 85.4 - phi, delta=2.5)
        self.assertLess(fast.deploy, rest.deploy)

    def test_optimum_is_the_max(self):
        m = PropModel([rotor()])
        best = m.optimum([460.0], (0.0, 40.0, 0.0), 1.2, 340.0)
        for d in range(0, 91, 3):
            self.assertLessEqual(
                m.evaluate(d, [460.0], (0.0, 40.0, 0.0), 1.2, 340.0).thrust,
                best.thrust + 1e-9)


class TestTable(unittest.TestCase):
    @unittest.skipUnless(os.path.isdir(os.path.join(ROOT, "testInstances",
                                                    "base", "GameData")),
                         "no farm base copy")
    def test_table_matches_game(self):
        """The committed table is what the game's configs say."""
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "bladetable", os.path.join(ROOT, "propPitch", "tools",
                                       "bladetable.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        d = os.path.join(ROOT, "testInstances", "base", "GameData",
                         "SquadExpansion", "Serenity", "Parts", "Propellers")
        for f in os.listdir(d):
            if f.endswith(".cfg"):
                name, entry = mod.parse_part(os.path.join(d, f))
                self.assertEqual(bladeTable.BLADES[name], entry, name)


if __name__ == "__main__":
    unittest.main()


class TestMotor(unittest.TestCase):
    def test_starved_motor_is_learned_and_feathers(self):
        """A rotor that sags under load teaches the model its real torque;
        with none left the optimum stops loading it."""
        m = PropModel([rotor()])
        r = m.rotors[0]
        full = m.optimum([460.0], (0.0, 60.0, 0.0), 1.2, 340.0)
        for _ in range(100):      # sagging to 300 rpm at the full-cap angle
            m.observe([300.0], [460.0], full.deploy, (0.0, 60.0, 0.0), 1.2,
                      340.0, 0.05)
        self.assertLess(r.max_torque, r.cap)
        weak = m.optimum([460.0], (0.0, 60.0, 0.0), 1.2, 340.0)
        self.assertLess(weak.rpms[0], 460.0)
        r.max_torque = 0.0
        dead = m.optimum([460.0], (0.0, 60.0, 0.0), 1.2, 340.0)
        self.assertGreater(dead.thrust, m.evaluate(full.deploy, [460.0],
                                                   (0.0, 60.0, 0.0), 1.2,
                                                   340.0).thrust)

    def test_holding_target_recovers(self):
        m = PropModel([rotor()])
        r = m.rotors[0]
        r.max_torque = 5.0
        for _ in range(400):
            m.observe([460.0], [460.0], 80.0, (0.0, 30.0, 0.0), 1.2, 340.0,
                      0.05)
        self.assertGreater(r.max_torque, 0.9 * r.cap)
