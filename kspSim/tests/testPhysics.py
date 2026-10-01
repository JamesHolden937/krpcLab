"""Offline tests for the simulator's physics: each law against the number the
game gave when it was measured (kspSim/CLAUDE.md, docs/kspSim/journal.md).

A synthetic vessel is built from a minimal model -- no probe, no game.
"""
import math
import os
import struct
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from kspSim import partcfg, vec  # noqa: E402
from kspSim import world as W  # noqa: E402
from kspSim.aero import Aero  # noqa: E402
from kspSim.curves import FloatCurve  # noqa: E402
from kspSim.thermal import Physics  # noqa: E402


def part(name, pos, mass, com=None, parent=None, rot=(0.0, 0.0, 0.0, 1.0)):
    return {"name": name, "title": name, "position": list(pos), "com": list(com or pos),
            "rotation": list(rot), "direction": [0, 1, 0], "mass": mass, "resources": [],
            "modules": [], "parent": parent, "bounding_box": [[-1, -1, -1], [1, 1, 1]]}


def model(parts, inertia=(100.0, 0, 0, 0, 50.0, 0, 0, 0, 100.0), **extra):
    m = {"ut": 1000.0,
         "body": {"name": "Kerbin", "equatorial_radius": 600000.0,
                  "gravitational_parameter": 3.5316e12, "rotational_period": 21549.425,
                  "rotation_angle": 0.0, "atmosphere_depth": 70000.0, "surface_gravity": 9.81},
         "vessel": {"name": "t", "inertia_tensor": list(inertia), "root": 0,
                    "bounding_box": [[-1, -1, -1], [1, 1, 1]], "situation": "orbiting"},
         "state": {"position_nonrot": [700000.0, 0.0, 0.0], "velocity_nonrot": [0.0, 0.0, 2300.0],
                   "rotation_nonrot": [0.0, 0.0, 0.0, 1.0], "angular_velocity": [0.0, 0.0, 0.0]},
         "parts": parts, "engines": [], "reaction_wheels": [], "control_surfaces": [],
         "rcs": [], "wheels": []}
    m.update(extra)
    return m


def vessel(m):
    w = W.World()
    w.t = m["ut"]
    w.body = W.Body(m)
    w.vessel = W.Vessel(m, w)
    w.loaded = True
    return w.vessel


class TestInertia(unittest.TestCase):
    def test_mass_moves_from_transform_to_centre_of_mass(self):
        # A wing whose root is on the axis and whose mass is 2 m out: kRPC
        # counts it at the root, the physics at 2 m.
        wing = part("wing", (0.0, 0.0, 0.0), 500.0, com=(2.0, 0.0, 0.0))
        v = vessel(model([part("body", (0, 0, 0), 1000.0), wing]))
        # Roll is about y: the wing adds m x^2 = 500 * 4 about y and about z.
        self.assertAlmostEqual(v.inertia0[1][1], 50.0 + 2000.0, places=6)
        self.assertAlmostEqual(v.inertia0[2][2], 100.0 + 2000.0, places=6)
        self.assertAlmostEqual(v.inertia0[0][0], 100.0, places=6)


class TestWheels(unittest.TestCase):
    def test_lerp_at_the_response_speed(self):
        # qs_plane's wheel step read 0.60, 0.84, 0.94 of full on successive
        # frames: Lerp(input, command, 30 * 0.02).
        m = model([part("pod", (0, 0, 0), 1000.0)])
        m["reaction_wheels"] = [{"part": 0, "active": True,
                                 "max_torque": [[15000.0] * 3, [-15000.0] * 3],
                                 "authority_limiter": 100.0}]
        v = vessel(m)
        got = [v.wheels_torque((1.0, 0.0, 0.0), W.DT)[0] / -15000.0 for _ in range(3)]
        for g, want in zip(got, (0.6, 0.84, 0.936)):
            self.assertAlmostEqual(g, want, places=3)


class TestRCS(unittest.TestCase):
    def rcs_model(self, thrusters, live=None):
        m = model([part("pod", (0, 0, 0), 1000.0), part("block", (0, -2, 1), 10.0)])
        m["rcs"] = [{"part": 1, "enabled": True, "max_vacuum_thrust": 1000.0,
                     "vacuum_isp": 240.0, "propellants": ["MonoPropellant"],
                     "thrusters": thrusters, "live": live or [True] * len(thrusters),
                     "pitch": True, "roll": True, "yaw": True}]
        m["parts"][0]["resources"] = [{"name": "MonoPropellant", "amount": 100.0,
                                       "max": 100.0, "density": 4.0}]
        return vessel(m)

    def test_translation_fires_at_the_input_projection(self):
        # A nose nozzle tilted 10 deg: forward 1.0 gives cos^2 of full
        # (qs_shuttle: 7875 N = 4000 + 4 x 0.9845^2 x 1000).
        c, s = math.cos(math.radians(10)), math.sin(math.radians(10))
        v = self.rcs_model([{"position": [0.0, 5.0, 0.0], "direction": [s, c, 0.0]}])
        f, _ = v.rcs_wrench((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.0, W.DT)
        self.assertAlmostEqual(f[1], 1000.0 * c * c, places=3)

    def test_rotation_lever_is_a_direction_only(self):
        # A nozzle far along the roll axis still fires at the full input
        # (the |p| normalisation made the shuttle's roll 5.6x weak).
        v = self.rcs_model([{"position": [0.0, -8.0, 1.0], "direction": [-1.0, 0.0, 0.0]}])
        _, t = v.rcs_wrench((0.0, 1.0, 0.0), (0.0, 0.0, 0.0), 0.0, W.DT)
        self.assertLess(t[1], 0.0)                      # opposes the input
        # Full thrust, whatever the lever: 1000 N at z = 1 - com.
        self.assertAlmostEqual(t[1], -1000.0 * (1.0 - v.com[2]), places=3)

    def test_dead_variant_nozzles_do_not_fire(self):
        v = self.rcs_model([{"position": [0.0, 5.0, 0.0], "direction": [0.0, 1.0, 0.0]}],
                           live=[False])
        f, _ = v.rcs_wrench((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.0, W.DT)
        self.assertEqual(f, (0.0, 0.0, 0.0))


class TestGimbal(unittest.TestCase):
    def test_torque_lever_is_the_pivot(self):
        # qs_plane: nozzle -3.50, pivot -1.78; the game's gimbal torque over
        # its lateral force read 1.78.
        m = model([part("pod", (0, 0, 0), 1000.0), part("engine", (0, -2.4, 0), 100.0)])
        m["engines"] = [{"part": 1, "active": True, "thrust_limit": 1.0,
                         "isp_at": [[0, 300], [1, 250]], "thrust_at": [[0, 125000], [1, 100000]],
                         "max_vacuum_thrust": 125000.0, "propellants": [],
                         "gimbal_range": 4.0,
                         "gimbal": {"range": 4.0, "response_speed": 20.0, "use_response": False,
                                    "limit": 1.0},
                         "thrusters": [{"position": [0.0, -3.5, 0.0], "direction": [0.0, 1.0, 0.0],
                                        "gimbal_position": [0.0, -1.78, 0.0]}]}]
        v = vessel(m)
        v.com = (0.0, 0.0, 0.0)
        e = v.engines[0]
        v._gimbal_step(e, (1.0, 0.0, 0.0), W.DT)
        d, at = v._gimballed(e, e.thrusters[0])
        f = vec.scale(d, 125000.0)
        t = vec.cross(at, f)
        self.assertAlmostEqual(f[2], 125000.0 * math.sin(math.radians(4.0)), delta=5.0)
        self.assertAlmostEqual(-t[0] / f[2], 1.78, places=2)


class TestIntegration(unittest.TestCase):
    def test_position_takes_the_mean_velocity(self):
        # Against the game in orbit: 3 mm in 5 s, where PhysX's order sank 0.37 m.
        v = vessel(model([part("pod", (0, 0, 0), 1000.0)]))
        r0, v0 = v.r, v.v
        g = vec.scale(r0, -v.world.body.mu / vec.norm(r0) ** 3)
        v.env = v.environment()
        v.integrate((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        want = vec.add(r0, vec.add(v0, vec.scale(g, 0.5)))
        self.assertLess(vec.norm(vec.sub(v.r, want)), 1e-6)


class TestDrainValves(unittest.TestCase):
    def test_each_valve_pushes_along_its_own_forward(self):
        # ModuleResourceDrain: part.AddForce(part.transform.forward * thrust)
        # per valve.  The shuttle's mirrored pair cancels; one lumped valve
        # pushing +z put 2 m/s into its circular orbit.
        s = math.sqrt(0.5)
        tank = part("tank", (0, 0, 0), 1000.0)
        tank["resources"] = [{"name": "LiquidFuel", "amount": 500.0, "max": 1000.0,
                              "density": 5.0, "enabled": True}]
        left = part("valve", (-1.6, 0, 0), 10.0, parent=0, rot=(0.0, s, 0.0, s))
        right = part("valve", (1.6, 0, 0), 10.0, parent=0, rot=(0.0, -s, 0.0, s))
        v = vessel(model([tank, left, right], drains=[{"part": 1}, {"part": 2}]))
        v.drain_active = True
        v.control.drain_rate = 20.0
        force, torque = v.wrench(W.DT)
        self.assertLess(vec.norm(force), 1e-6)
        # Each valve takes its rate of the vessel's capacity: two drain twice.
        self.assertAlmostEqual(500.0 - v.resource_amount("LiquidFuel"),
                               2 * 1000.0 * 0.2 * W.DT, places=6)


class TestWarp(unittest.TestCase):
    def test_rails_rate_ramps_over_one_real_second(self):
        # TimeWarp.SetRate(i, instant=false): Lerp(last, target, Time.time -
        # start), Time.timeScale held at 1 on rails.
        v = vessel(model([part("pod", (0, 0, 0), 1000.0)]))
        w = v.world
        w.body.warp_limits = [0.0] * len(W.RAILS_RATES)
        w.set_warp(2)
        t0 = w.t
        for _ in range(40):                     # 2 s of real time
            w.advance(0.05 * w.warp_rate())
        self.assertAlmostEqual(w.t - t0, 5.5 + 10.0, delta=0.6)
        w.set_warp(0)
        t1 = w.t
        for _ in range(40):
            w.advance(0.05 * (w.warp_rate() if w.on_rails else 1.0))
        self.assertFalse(w.on_rails)
        self.assertAlmostEqual(w.t - t1, 5.5 + 1.0, delta=0.6)


class TestDensityAt(unittest.TestCase):
    def test_equator_day_average(self):
        # kRPC's CelestialBody.DensityAt: the game's c0 read 353 m/s.
        from kspSim import api
        from kspSim.atmosphere import Atmosphere

        class B:
            atm = Atmosphere()

        class Wd:
            body = B()
        rho = api.body_density_at(Wd(), None, 0.0)
        p = B.atm.pressure(0.0)
        self.assertAlmostEqual(B.atm.sound_speed(p, rho), 352.8, delta=0.3)


class TestReynolds(unittest.TestCase):
    def test_cube_drag_follows_the_multiplier(self):
        curve = [[0, 4, 0, 0], [1, 1, 0, 0], [100, 1, 0, 0], [200, 0.82, 0, 0], [10000, 0.95, 0, 0]]
        rest = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
        cube = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0]
        m1, m2 = 1.0, 0.82

        def table(m):
            cell = [rest[k] + m * cube[k] for k in range(6)]
            return [[[cell for _ in range(2)] for _ in range(2)] for _ in range(2)]
        tables = {"mach": [0.5, 1.0], "alpha": [0.0, 10.0], "beta": [-5.0, 5.0],
                  "base": table(m1), "base2": table(m2), "re": [50.0, 50.0], "re2": [200.0, 200.0]}
        a = Aero(tables, {"curves": {"DRAG_PSEUDOREYNOLDS": curve}})
        c = a.coefficients(0.7, 5.0, 0.0, reynolds=200.0)
        self.assertAlmostEqual(c[1], -2.0 * 0.82, places=6)
        c = a.coefficients(0.7, 5.0, 0.0, reynolds=60.0)
        self.assertAlmostEqual(c[1], -2.0, places=6)
        self.assertAlmostEqual(c[2], -1.0, places=6)


class TestPartConfigs(unittest.TestCase):
    CFG = """UrlConfig
{
\tparentUrl = Mod/Parts/block.cfg
\tPART
\t{
\t\tname = Block_v2
\t\tmaxTemp = 1500
\t\tMODULE
\t\t{
\t\t\tname = ModuleRCSFX
\t\t\tthrusterTransformName = jet
\t\t}
\t\tMODEL
\t\t{
\t\t\tmodel = Mod/Assets/block
\t\t}
\t\tMODULE
\t\t{
\t\t\tname = ModulePartVariants
\t\t\tbaseVariant = Two
\t\t\tVARIANT
\t\t\t{
\t\t\t\tname = One
\t\t\t\tGAMEOBJECTS
\t\t\t\t{
\t\t\t\t\tV1 = true
\t\t\t\t\tV2 = false
\t\t\t\t}
\t\t\t}
\t\t\tVARIANT
\t\t\t{
\t\t\t\tname = Two
\t\t\t\tGAMEOBJECTS
\t\t\t\t{
\t\t\t\t\tV1 = false
\t\t\t\t\tV2 = true
\t\t\t\t}
\t\t\t}
\t\t}
\t}
}
"""

    def test_live_nozzles_come_from_the_variant(self):
        with tempfile.TemporaryDirectory() as d:
            cache = os.path.join(d, "cache")
            with open(cache, "w") as fh:
                fh.write(self.CFG)
            os.makedirs(os.path.join(d, "Mod", "Assets"))

            def s(x):
                return struct.pack("B", len(x)) + x.encode()
            # Hierarchy order: V1 owns one nozzle, V2 owns two.
            mu = b"\x00" + s("V1") + b"\x01" + s("jet") + b"\x02" + s("V2") + s("jet") + s("jet")
            with open(os.path.join(d, "Mod", "Assets", "block.mu"), "wb") as fh:
                fh.write(mu)
            pc = partcfg.PartConfigs(cache)
            self.assertEqual(pc.url("Block.v2"), "Mod/Parts/block/Block_v2")
            va = partcfg.Variants(pc, d)
            self.assertEqual(va.live_mask("Block.v2", None, "jet", 3), [False, True, True])
            self.assertEqual(va.live_mask("Block.v2", "One", "jet", 3), [True, False, False])


class TestThermal(unittest.TestCase):
    def test_shock_temperature_lerps_to_the_mach_law(self):
        p = Physics({}, heat_scale=1.0)
        self.assertAlmostEqual(p.external_temperature(500.0, 1.5, 200.0), 500.0)
        self.assertAlmostEqual(p.external_temperature(2000.0, 6.0, 200.0),
                               21.0 * 2000.0 ** 0.75, places=6)

    def test_burn_up_takes_the_children(self):
        m = model([part("root", (0, 0, 0), 1000.0), part("wing", (1, 0, 0), 100.0, parent=0),
                   part("tip", (2, 0, 0), 10.0, parent=1)])
        v = vessel(m)
        v.burn_up(1)
        self.assertEqual([p.alive for p in v.parts], [True, False, False])
        self.assertAlmostEqual(v.mass, 1000.0)
        self.assertFalse(v.destroyed)
        v.burn_up(0)
        self.assertTrue(v.destroyed)


class TestThermalGeometry(unittest.TestCase):
    """The FlightIntegrator port's geometry (thermal.py)."""

    def test_stacked_neighbour_covers_its_facing_area(self):
        # DragCubeList.SetPartOcclusion: a 1 m^2-faced part stacked on a
        # 2 m^2-faced one takes 1 m^2 off the lower part's top face, and
        # that is the contact area the two conduct over.
        from kspSim import thermal
        big = {"faces": [[2.0, 1.0, 1.0]] * 6, "center": [0, 0, 0], "size": [1.4, 1.4, 1.4]}
        small = {"faces": [[1.0, 1.0, 1.0]] * 6, "center": [0, 0, 0], "size": [1, 1, 1]}
        low = part("low", (0, 0, 0), 1000.0)
        top = part("top", (0, 1.2, 0), 500.0, parent=0)
        m = model([low, top], dragcubes={"low": big, "top": small},
                  attach=[{"att": [["top", 1]], "srf": None}, {"att": [["bottom", 0]], "srf": None}],
                  nodes={"low": {"top": [0, 0.7, 0, 0, 1, 0]},
                         "top": {"bottom": [0, -0.5, 0, 0, -1, 0]}})
        v = vessel(m)
        th = thermal.Thermal(v, m)
        lo, hi = th.parts
        self.assertAlmostEqual(lo.cube.occ[2], 1.0)          # +y face: 2 - 1
        self.assertAlmostEqual(hi.cube.occ[3], 0.0)          # -y face: 1 - 2, floored
        self.assertAlmostEqual(lo.rad_area, 11.0)
        # At load the root part's nodes have no contact area yet: a link that
        # reads one (the upper part's, which takes the remote's node) gets
        # the 0.01 m^2 floor -- measured, the game's conduction flux.
        self.assertAlmostEqual(hi.links[0][1], 0.01)
        self.assertAlmostEqual(lo.links[0][1], 2.0)
        # Rebuilt after a part is lost, every link takes the remote part's
        # node: the facing area of the part it holds -- so the two links of
        # one joint differ (2 m^2 from below, 1 m^2 from above), as in KSP.
        th.parts_changed()
        self.assertAlmostEqual(lo.links[0][1], 2.0)
        self.assertAlmostEqual(hi.links[0][1], 1.0)

    def test_exposed_area_weights_faces_by_the_surface_curves(self):
        # AddSurfaceDragDirection at Mach 0 with tip 1, surface 0.02, tail 1
        # and faces of drag 1: the front and back faces count whole, the
        # four side faces 0.02 each.
        from kspSim.thermal import Cube
        c = Cube({"faces": [[1.0, 1.0, 1.0]] * 6})
        c.finish(None)
        exposed, cross, taper, depth, cd = c.set_drag((0.0, 1.0, 0.0), 1.0, 0.02, 1.0, 1.0, 1.0)
        self.assertAlmostEqual(exposed, 2.0 + 4 * 0.02)
        self.assertAlmostEqual(cross, 1.0)
        self.assertAlmostEqual(taper, 1.0)

    def test_circle_overlap(self):
        from kspSim.thermal import _circle_overlap
        self.assertAlmostEqual(_circle_overlap(1.0, 1.0, 0.0), 1.0)
        self.assertAlmostEqual(_circle_overlap(1.0, 1.0, 4.1), 0.0)
        self.assertAlmostEqual(_circle_overlap(1.0, 2.0, 0.0), 0.25)
        half = _circle_overlap(100.0, 1.0, 100.0 ** 2)    # a huge circle's edge
        self.assertAlmostEqual(half, 0.5, places=2)


class TestCurves(unittest.TestCase):
    def test_float_curve_hits_its_keys(self):
        c = FloatCurve([[0, 4, 0, -2975.412], [1, 1, 0, 0], [100, 1, 0, 0], [200, 0.82, 0, 0]])
        self.assertAlmostEqual(c(1.0), 1.0)
        self.assertAlmostEqual(c(50.0), 1.0)
        self.assertAlmostEqual(c(200.0), 0.82)


class TestWireCodec(unittest.TestCase):
    """``fastpb`` and the compiled codecs give protobuf's bytes exactly."""

    def setUp(self):
        try:
            import krpc.schema.KRPC_pb2  # noqa: F401
        except ImportError:
            self.skipTest("krpc not installed")

    def test_messages(self):
        import krpc.schema.KRPC_pb2 as KRPC
        from kspSim import fastpb
        req = KRPC.Request()
        c = req.calls.add(service="SpaceCenter", procedure="Flight_SimulateAerodynamicForceAt")
        c.arguments.add(position=0, value=b"\x05")
        c.arguments.add(position=3, value=b"")
        c.arguments.add(position=2, value=bytes(range(200)))
        req.calls.add(service="KRPC", procedure="GetStatus", service_id=3, procedure_id=300)
        raw = req.SerializeToString()
        calls = fastpb.parse_request(raw)
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0].arguments, [(0, b"\x05"), (3, b""), (2, bytes(range(200)))])
        self.assertEqual(b"".join(fastpb._ld(1, fastpb.serialize_call(x)) for x in calls), raw)
        for err, val in ((None, b""), (None, b"\x00" * 300), (("KRPC", "", "no", ""), b""),
                         (("", "", "Boom: x", "trace\nline"), b"")):
            r = KRPC.ProcedureResult()
            if val:
                r.value = val
            if err:
                r.error.service, r.error.name, r.error.description, r.error.stack_trace = err
            fast = fastpb.serialize_result(fastpb.Result(val, err))
            self.assertEqual(fast, r.SerializeToString())
            resp = KRPC.Response()
            resp.results.append(r)
            resp.results.append(KRPC.ProcedureResult())
            self.assertEqual(fastpb.serialize_response([fast, b""]), resp.SerializeToString())
            up = KRPC.StreamUpdate()
            up.results.add(id=0).result.CopyFrom(r)
            up.results.add(id=2 ** 40).result.CopyFrom(r)
            self.assertEqual(fastpb.serialize_stream_update([(0, fast), (2 ** 40, fast)]),
                             up.SerializeToString())

    def test_every_signature(self):
        """Each procedure's compiled codecs against the protobuf reference,
        on sample values of every parameter and return type."""
        from kspSim import protocol
        from krpc.types import (ClassType, EnumerationType, ValueType, TupleType,
                                ListType, SetType, DictionaryType, MessageType)
        import krpc.schema.KRPC_pb2 as KRPC

        class Obj:
            __sim_object__ = True
        reg = protocol.Registry()
        objs = [Obj() for _ in range(3)]
        for o in objs:
            reg.id_of(o)

        def sample(t, k):
            if isinstance(t, ClassType):
                return objs[k % 3] if k % 4 else None
            if isinstance(t, EnumerationType):
                return (-2, 0, 1, 7)[k % 4]
            if isinstance(t, ValueType):
                code = t.protobuf_type.code
                if code == KRPC.Type.DOUBLE:
                    return (-1.25e-7, 0.0, 3.5, 6.02e23)[k % 4]
                if code == KRPC.Type.FLOAT:
                    return (-1.5, 0.0, 0.25, 1e10)[k % 4]
                if code in (KRPC.Type.SINT32, KRPC.Type.SINT64):
                    return (-300, 0, 1, 2 ** 30)[k % 4]
                if code in (KRPC.Type.UINT32, KRPC.Type.UINT64):
                    return (0, 1, 300, 2 ** 40)[k % 4]
                if code == KRPC.Type.BOOL:
                    return bool(k % 2)
                if code == KRPC.Type.STRING:
                    return ("", "a", "Mk3 \u00e9 %d" % k, "x" * 200)[k % 4]
                return (b"", b"\x00\x01", bytes(300))[k % 3]
            if isinstance(t, TupleType):
                return tuple(sample(x, k + i) for i, x in enumerate(t.value_types))
            if isinstance(t, (ListType, SetType)):
                return [sample(t.value_type, k + i) for i in range(k % 3)]
            if isinstance(t, DictionaryType):
                return {sample(t.key_type, i): sample(t.value_type, i + 1) for i in range(1, 3)}
            return None
        sig = protocol.Signatures()
        n = 0
        for key, (params, _, ret) in sig.procs.items():
            decs, _, enc = sig.codecs[key]
            for k in range(4):
                for t, dec in zip(params, decs):
                    if isinstance(t, MessageType):
                        continue
                    v = sample(t, k)
                    ref = protocol.encode(v, t)
                    self.assertEqual(protocol.encoder(t)(v), ref, (key, t, v))
                    self.assertEqual(dec(ref, reg), protocol.decode(ref, t, reg), (key, t, v))
                    n += 1
                if ret is not None and not isinstance(ret, MessageType):
                    v = sample(ret, k)
                    self.assertEqual(enc(v), protocol.encode(v, ret), (key, ret, v))
        self.assertGreater(n, 1000)


class TestFastClient(unittest.TestCase):
    """``fastclient``'s encoders and decoders against krpc's own, on every
    parameter and return type the game declares."""

    def test_every_signature(self):
        try:
            import krpc.schema.KRPC_pb2 as KRPC
            from krpc.decoder import Decoder
            from krpc.encoder import Encoder
            from krpc.types import (Types, ClassType, EnumerationType, ValueType,
                                    TupleType, ListType, SetType, DictionaryType,
                                    MessageType)
        except ImportError:
            self.skipTest("krpc not installed")
        from kspSim import fastclient, protocol
        enc0 = Encoder.encode.__func__
        dec0 = Decoder.decode.__func__
        if fastclient._ORIGINAL:
            enc0, dec0 = fastclient._ORIGINAL["encode"], fastclient._ORIGINAL["decode"]

        def oenc(v, t):
            return enc0(Encoder, v, t)

        def odec(c, d, t):
            return dec0(Decoder, c, d, t)
        types = Types()
        raw = open(os.path.join(ROOT, "kspSim", "data", "services.bin"), "rb").read()
        services = KRPC.Services()
        services.ParseFromString(raw)
        seen = []
        for svc in services.services:
            for proc in svc.procedures:
                for pb in [p.type for p in proc.parameters] + [proc.return_type]:
                    if pb.code == KRPC.Type.NONE:
                        continue
                    try:
                        seen.append(types.as_type(pb))
                    except Exception:           # noqa: BLE001
                        continue

        def sample(t, k):
            if isinstance(t, ClassType):
                return None if k % 4 == 0 else t.python_type(None, k)
            if isinstance(t, EnumerationType):
                members = list(t.python_type)
                return members[k % len(members)] if members else None
            if isinstance(t, ValueType):
                code = t.protobuf_type.code
                return {KRPC.Type.DOUBLE: (-1.25e-7, 0.0, 3.5, 6.02e23)[k % 4],
                        KRPC.Type.FLOAT: (-1.5, 0.0, 0.25, 1e10)[k % 4],
                        KRPC.Type.SINT32: (-300, 0, 1, 2 ** 30)[k % 4],
                        KRPC.Type.SINT64: (-300, 0, 1, 2 ** 40)[k % 4],
                        KRPC.Type.UINT32: (0, 1, 300, 2 ** 31)[k % 4],
                        KRPC.Type.UINT64: (0, 1, 300, 2 ** 40)[k % 4],
                        KRPC.Type.BOOL: bool(k % 2),
                        KRPC.Type.STRING: ("", "a", "\u00e9 %d" % k, "x" * 200)[k % 4],
                        KRPC.Type.BYTES: (b"", b"\x01", bytes(200))[k % 3]}[code]
            if isinstance(t, TupleType):
                return tuple(sample(x, k + i) for i, x in enumerate(t.value_types))
            if isinstance(t, ListType):
                return [sample(t.value_type, k + i) for i in range(k % 3)]
            if isinstance(t, SetType):
                return {sample(t.value_type, k + i) for i in range(k % 3)}
            return None
        enums = {(svc.name, e.name): {v.name: {"value": v.value, "doc": ""} for v in e.values}
                 for svc in services.services for e in svc.enumerations}

        def with_values(t):
            if isinstance(t, EnumerationType) and t.python_type is None:
                t.set_values(enums[(t._service_name, t._enum_name)])
            for sub in getattr(t, "value_types", None) or [getattr(t, "value_type", None)]:
                if sub is not None:
                    with_values(sub)
        n = 0
        for t in seen:
            with_values(t)
            if isinstance(t, (MessageType, DictionaryType)):
                continue
            try:
                for k in range(4):
                    v = sample(t, k)
                    if isinstance(t, EnumerationType) and v is None:
                        continue
                    ref = oenc(v, t)
                    self.assertEqual(fastclient.encoder(t, oenc)(v), ref, (t, v))
                    fastclient._DEC.pop(t, None)
                    self.assertEqual(fastclient.decoder(t, odec)(None, ref),
                                     odec(None, ref, t), (t, v))
                    fastclient._DEC.pop(t, None)
                    n += 1
            finally:
                fastclient._ENC.pop(t, None)
                fastclient._DEC.pop(t, None)
        self.assertGreater(n, 100)
        # A list or tuple sent as a single zero byte is None to krpc.
        lt = types.list_type(types.double_type)
        self.assertIsNone(fastclient.decoder(lt, odec)(None, b"\x00"))


if __name__ == "__main__":
    unittest.main()
