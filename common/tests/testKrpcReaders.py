"""The readers of kRPC's leg, joint and launch-site properties, on fakes
(``krpcbatch.ask`` falls back to one call at a time without a wire)."""

import unittest

from common import impact, launchsite, touchdown


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def field(name, value):
    return Obj(name=name, float_value=value)


def leg(x, z, y=-5.0):
    susp = Obj(name="ModuleWheelSuspension", field_list=[
        field("springRatio", 50.0), field("damperRatio", 1.0),
        field("suspensionDistance", 0.95)])
    base = Obj(name="ModuleWheelBase", field_list=[
        field("radius", 0.2), field("frictionPeak", 1.0)])
    part = Obj(title="LT-2 Landing Strut", impact_tolerance=12.0,
               modules=[Obj(name="ModuleWheelDeployment"), susp, base])
    return Obj(part=part, impact_tolerance=600.0, explode_multiplier=1.5,
               contact_position=lambda frame: (x, y, z),
               suspension_travel=0.95, suspension_spring=None,
               suspension_damper=None, deployed=True)


class Vessel:
    reference_frame = "vessel"

    def __init__(self, legs=(), parts=()):
        self.parts = Obj(legs=list(legs), all=list(parts))


class TestLegs(unittest.TestCase):
    def test_constants_and_live_feet(self):
        v = Vessel([leg(2, 0), leg(-1, 1.7), leg(-1, -1.7)])
        const = touchdown.read_leg_constants(None, v)
        self.assertEqual(len(const), 3)
        self.assertEqual(const[0]["spring_ratio"], 50.0)
        self.assertEqual(const[0]["friction"], 1.0)
        legs = touchdown.read_legs(None, v, const)
        self.assertEqual(len(legs), 3)
        self.assertEqual(legs[0]["spring"], 0.0)       # None in the air
        self.assertEqual(legs[0]["y"], -5.0)
        lim = touchdown.landing_limits(10e3, 5e4, 9.81, legs)
        self.assertAlmostEqual(lim["h"], 5.0)           # the foot's bottom
        self.assertGreater(lim["vertical"], 0.0)

    def test_a_missing_constant_is_no_leg(self):
        bad = leg(0, 0)
        bad.part.modules = []
        v = Vessel([bad])
        legs = touchdown.read_legs(None, v, touchdown.read_leg_constants(None, v))
        self.assertIsNone(legs)

    def test_no_api(self):
        self.assertIsNone(touchdown.read_leg_constants(None, Obj()))


class TestJoints(unittest.TestCase):
    def test_chain(self):
        def part(title, parent, y, bf):
            return Obj(title=title, mass=1000.0, impact_tolerance=10.0,
                       parent=parent, joint_break_force=bf,
                       joint_break_torque=bf, joint_spring=1e5,
                       joint_damper=0.0,
                       center_of_mass=lambda f, y=y: (0.0, y, 0.0),
                       bounding_box=lambda f, y=y: ((0, y - 1, 0), (0, y + 1, 0)))
        root = part("tank", None, 2.0, None)
        eng = part("engine", root, -2.0, 3.4e38)
        parts = impact.read(None, Vessel(parts=[root, eng]))
        self.assertEqual([p["parent"] for p in parts], [-1, 0])
        self.assertEqual(parts[1]["break_force"], impact.INF)
        self.assertEqual(parts[0]["break_force"], 0.0)
        self.assertEqual(impact.touching(parts), [1])
        self.assertEqual(parts[0]["mass"], 1.0)


class TestLaunchSite(unittest.TestCase):
    def test_site(self):
        site = Obj(latitude=-0.0972, longitude=-74.5577, altitude=70.0,
                   name="LaunchPad", body=Obj(name="Kerbin"))
        got = launchsite.launch_site(None, Obj(launch_site=site))
        self.assertEqual(got["site"], "LaunchPad")
        self.assertEqual(got["body"], "Kerbin")

    def test_none(self):
        self.assertIsNone(launchsite.launch_site(None, Obj(launch_site=None)))
        self.assertIsNone(launchsite.launch_site(None, Obj()))
        nan = float("nan")
        site = Obj(latitude=nan, longitude=nan, altitude=nan, name="x",
                   body=Obj(name="Kerbin"))
        self.assertIsNone(launchsite.launch_site(None, Obj(launch_site=site)))


if __name__ == "__main__":
    unittest.main()
