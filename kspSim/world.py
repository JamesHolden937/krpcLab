"""The simulated world: Kerbin, one vessel, and time.

State is kept in the body's **non-rotating** frame (kRPC's
``CelestialBody.non_rotating_reference_frame``): position ``r``, velocity
``v``, rotation ``q`` (vessel axes -> that frame) and angular velocity ``w``.
The rotating frame is that frame turned about +y by the body's rotation
angle, measured from the game (``tools/probe.py``):

    nr = M(phi) bf,   M = [[c, 0, -s], [0, 1, 0], [s, 0, c]],   spin (0, -omega, 0)

Physics runs at KSP's fixed step, 0.02 s, integrated the way PhysX does it
(velocity first, then position).  On rails -- time warp above the
atmosphere -- the orbit is propagated analytically and attitude is frozen,
also as KSP does: a warped interval costs one Kepler solve however long it
is.
"""
import copy
import json
import math
import os

from kspSim import quat, vec
from kspSim.aero import Aero
from kspSim.atmosphere import Atmosphere
from kspSim.attitude import AttitudeController
from kspSim.thermal import Thermal

DT = 0.02
THERMAL_EVERY = 5
SYNC_SURFACES = True
G0 = 9.80665
RAILS_RATES = [1, 5, 10, 50, 100, 1000, 10000, 100000]
MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# Drain valves: KSP's RESOURCE_DRAIN_DEFINITION drainForceISP for the
# drainable resources (ResourcesGeneric.cfg).  Measured on qs_plane: 800 kg/s
# vented gave 39 kN.
DRAIN_ISP = 5.0
DRAINABLE = ("LiquidFuel", "Oxidizer")


# -- small helpers ----------------------------------------------------------

def kepler(r, v, mu, dt):
    """Two-body propagation by universal variables."""
    r0 = vec.norm(r)
    v0 = vec.norm(v)
    vr0 = vec.dot(r, v) / r0
    alpha = 2.0 / r0 - v0 * v0 / mu
    sq = math.sqrt(mu)
    x = sq * abs(alpha) * dt if alpha > 1e-12 else sq * dt / r0
    for _ in range(60):
        z = alpha * x * x
        c, s = _stumpff(z)
        f = (r0 * vr0 / sq * x * x * c + (1 - alpha * r0) * x ** 3 * s + r0 * x - sq * dt)
        df = (r0 * vr0 / sq * x * (1 - z * s) + (1 - alpha * r0) * x * x * c + r0)
        step = f / df
        x -= step
        if abs(step) < 1e-9:
            break
    z = alpha * x * x
    c, s = _stumpff(z)
    f = 1 - x * x / r0 * c
    g = dt - x ** 3 / sq * s
    rn = vec.add(vec.scale(r, f), vec.scale(v, g))
    rnn = vec.norm(rn)
    fd = sq / (rnn * r0) * (alpha * x ** 3 * s - x)
    gd = 1 - x * x / rnn * c
    vn = vec.add(vec.scale(r, fd), vec.scale(v, gd))
    return rn, vn


def _stumpff(z):
    if z > 1e-6:
        s = math.sqrt(z)
        return (1 - math.cos(s)) / z, (s - math.sin(s)) / (s ** 3)
    if z < -1e-6:
        s = math.sqrt(-z)
        return (math.cosh(s) - 1) / (-z), (math.sinh(s) - s) / (s ** 3)
    return 0.5 - z / 24.0, 1.0 / 6.0 - z / 120.0


def interp(table, x):
    """Piecewise-linear over [[x, y], ...], clamped."""
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


def inertia_at_part_coms(tensor, parts):
    """kRPC's ``Vessel.inertia_tensor`` with each part's mass moved from its
    transform to its centre of mass.

    kRPC puts every part's mass at ``rb.position`` -- the part's transform,
    which for a wing is its root -- where the physics has it at the part's
    centre of mass.  Measured with the wheels alone in orbit, the true roll
    inertia is 1.1433x kRPC's on the Mk3 shuttle (this correction: 1.1435)
    and 1.025x on the old plane (1.023); pitch, yaw and the products agree
    as well.  Point masses about the origin, which is the probe-time centre
    of mass."""
    io = [list(row) for row in tensor]
    for p in parts:
        m = p.rec["mass"]
        for d, sign in ((p.com, 1.0), (p.position, -1.0)):
            dd = vec.dot(d, d)
            for i in range(3):
                for j in range(3):
                    io[i][j] += sign * m * ((dd if i == j else 0.0) - d[i] * d[j])
    return tuple(tuple(row) for row in io)


# -- the body ---------------------------------------------------------------

class Body:
    def __init__(self, model):
        b = model["body"]
        cfg = json.load(open(os.path.join(DATA, "kerbin.json")))
        self.name = b["name"]
        self.radius = b["equatorial_radius"]
        self.mu = b["gravitational_parameter"]
        self.period = b["rotational_period"]
        self.omega = 2.0 * math.pi / self.period
        self.t0 = model["ut"]
        self.phi0 = b["rotation_angle"]
        self.atmosphere_depth = b["atmosphere_depth"]
        self.surface_gravity = b["surface_gravity"]
        self.soi = b.get("sphere_of_influence", 8.4e7)
        self.atm = Atmosphere(cfg)
        self.warp_limits = cfg["timewarpAltitudeLimits"]
        sun = b.get("sun_position_nonrot", (1.0, 0.0, 0.0))
        self.sun_nr = vec.unit(sun)
        self.spin = (0.0, -self.omega, 0.0)
        self.terrain = Terrain(model.get("terrain"))

    def phi(self, t):
        return self.phi0 + self.omega * (t - self.t0)

    def to_bf(self, v, t):
        p = self.phi(t)
        c, s = math.cos(p), math.sin(p)
        return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])

    def from_bf(self, v, t):
        p = self.phi(t)
        c, s = math.cos(p), math.sin(p)
        return (c * v[0] - s * v[2], v[1], s * v[0] + c * v[2])

    def q_bf(self, t):
        """Rotation taking body-frame vectors into the non-rotating frame."""
        return quat.from_axis_angle((0.0, 1.0, 0.0), -self.phi(t))

    def lat_lon(self, r_bf):
        n = vec.norm(r_bf)
        lat = math.degrees(math.asin(max(-1.0, min(1.0, r_bf[1] / n))))
        lon = math.degrees(math.atan2(r_bf[2], r_bf[0]))
        return lat, lon

    def surface_point(self, lat, lon, alt):
        la, lo = math.radians(lat), math.radians(lon)
        r = self.radius + alt
        return (r * math.cos(la) * math.cos(lo), r * math.sin(la), r * math.cos(la) * math.sin(lo))

    def sun_bf(self, t):
        return self.to_bf(self.sun_nr, t)

    def air(self, r_bf, t):
        """(altitude, pressure kPa, temperature K, density, sound speed)."""
        alt = vec.norm(r_bf) - self.radius
        if alt >= self.atmosphere_depth:
            return alt, 0.0, 0.0, 0.0, 0.0
        lat, _ = self.lat_lon(r_bf)
        p = self.atm.pressure(alt)
        temp = self.atm.temperature(alt, lat, self.atm.sun_dot(r_bf, self.sun_bf(t)))
        rho = self.atm.density(p, temp)
        return alt, p, temp, rho, self.atm.sound_speed(p, rho)


class Terrain:
    """Ground height.  A grid sampled from the game where there is one
    (``probe.py --terrain``), the KSC plateau at the runway's height, and
    sea level elsewhere."""

    KSC_BOX = (-0.25, 0.15, -74.95, -74.35)   # lat lo, lat hi, lon lo, lon hi
    KSC_HEIGHT = 69.2                          # CelestialBody.SurfacePosition at the runway

    _fine = None

    def __init__(self, grid=None):
        self.grid = grid
        if Terrain._fine is None:
            # The KSC's pad and runway, finely (``tools/kscterrain.py``): the
            # pad is a 5 m mound the coarse grid smooths away.
            path = os.path.join(DATA, "ksc_terrain.json")
            try:
                Terrain._fine = json.load(open(path))
            except (OSError, ValueError):
                Terrain._fine = []

    @staticmethod
    def _bilinear(g, lat, lon):
        i = (lat - g["lat0"]) / g["dlat"]
        j = (lon - g["lon0"]) / g["dlon"]
        if 0 <= i < g["nlat"] - 1 and 0 <= j < g["nlon"] - 1:
            i0, j0 = int(i), int(j)
            fi, fj = i - i0, j - j0
            h = g["heights"]
            a = h[i0][j0] * (1 - fj) + h[i0][j0 + 1] * fj
            b = h[i0 + 1][j0] * (1 - fj) + h[i0 + 1][j0 + 1] * fj
            return a * (1 - fi) + b * fi
        return None

    def height(self, lat, lon):
        for g in Terrain._fine:
            h = self._bilinear(g, lat, lon)
            if h is not None:
                return h
        g = self.grid
        if g:
            la0, dla, nla = g["lat0"], g["dlat"], g["nlat"]
            lo0, dlo, nlo = g["lon0"], g["dlon"], g["nlon"]
            i = (lat - la0) / dla
            j = (lon - lo0) / dlo
            if 0 <= i < nla - 1 and 0 <= j < nlo - 1:
                i0, j0 = int(i), int(j)
                fi, fj = i - i0, j - j0
                h = g["heights"]
                a = h[i0][j0] * (1 - fj) + h[i0][j0 + 1] * fj
                b = h[i0 + 1][j0] * (1 - fj) + h[i0 + 1][j0 + 1] * fj
                return a * (1 - fi) + b * fi
        b = self.KSC_BOX
        if b[0] <= lat <= b[1] and b[2] <= lon <= b[3]:
            return self.KSC_HEIGHT
        return 0.0


# -- the vessel -------------------------------------------------------------

class Resource:
    def __init__(self, rec, part):
        self.name = rec["name"]
        self.amount = rec["amount"]
        self.max = rec["max"]
        self.density = rec["density"]
        self.enabled = rec.get("enabled", True)
        self.part = part


class Part:
    __sim_object__ = True

    def __init__(self, index, rec, vessel):
        self.index = index
        self.rec = rec
        self.vessel = vessel
        self.name = rec["name"]
        self.title = rec["title"]
        self.position = tuple(rec["position"])
        self.com = tuple(rec.get("com", rec["position"]))
        self.rotation = tuple(rec.get("rotation", (0.0, 0.0, 0.0, 1.0)))
        self.direction = tuple(rec.get("direction", (0.0, 1.0, 0.0)))
        self.resources = [Resource(r, self) for r in rec["resources"]]
        self.resource_mass0 = sum(r.amount * r.density for r in self.resources)
        self.dry_mass = rec["mass"] - self.resource_mass0
        self.dry_mass0 = self.dry_mass
        self.modules = [Module(m, self) for m in rec["modules"]]
        self.skin_temperature = rec.get("skin_temperature", 300.0)
        self.temperature = rec.get("temperature", 300.0)
        self.alive = True

    def mass(self):
        return self.dry_mass + sum(r.amount * r.density for r in self.resources)


class Module:
    __sim_object__ = True

    def __init__(self, rec, part):
        self.name = rec["name"]
        self.fields = dict(rec.get("fields", {}))
        self.actions = list(rec.get("actions", []))
        self.events = list(rec.get("events", []))
        self.part = part


class Engine:
    __sim_object__ = True

    def __init__(self, rec, vessel):
        self.rec = rec
        self.part = vessel.parts[rec["part"]]
        self.vessel = vessel
        self.active = rec["active"]
        self.thrust_limit = rec["thrust_limit"]
        self.isp_at = rec["isp_at"]
        self.thrust_at = rec["thrust_at"]
        self.max_vacuum_thrust = rec["max_vacuum_thrust"]
        self.propellants = rec["propellants"]
        self.thrusters = rec["thrusters"]
        self.gimbal_range = rec.get("gimbal_range", 0.0)
        self.gimbal_locked = False
        self.thrust = 0.0
        self.flameout = False

    def max_thrust(self, p_atm):
        return interp(self.thrust_at, p_atm)

    def isp(self, p_atm):
        return interp(self.isp_at, p_atm)


class Vessel:
    """Everything that moves, and every actuator that moves it."""

    def __init__(self, model, world):
        self.world = world
        self.model = model
        v = model["vessel"]
        s = model["state"]
        self.name = v["name"]
        self.parts = [Part(i, p, self) for i, p in enumerate(model["parts"])]
        self.engines = [Engine(e, self) for e in model["engines"]]
        self.reaction_wheels = model["reaction_wheels"]
        self.control_surfaces = model["control_surfaces"]
        self.rcs = model["rcs"]
        self.wheels = model["wheels"]
        self.drains = model.get("drains", [])
        self.aero = Aero(model["aero"], model.get("physics")) if model.get("aero") else None
        # Where the tables' reference centre of mass sits in this save's
        # vessel frame (another save of the same craft carries less fuel).
        self.aero_origin = tuple(model.get("aero_origin", (0.0, 0.0, 0.0)))
        self.residual = model.get("aero_residual")
        body = world.body
        t = model["ut"]
        if "position_nonrot" in s:
            self.r = tuple(s["position_nonrot"])
            self.v = tuple(s["velocity_nonrot"])
            self.q = quat.norm(tuple(s["rotation_nonrot"]))
        else:
            rb = tuple(s["position"])
            self.r = body.from_bf(rb, t)
            self.v = vec.add(body.from_bf(tuple(s["velocity"]), t),
                             vec.cross(body.spin, self.r))
            self.q = quat.norm(quat.mul(body.q_bf(t), tuple(s["rotation"])))
        wb = tuple(s["angular_velocity"])            # relative to the body frame
        self.w = vec.add(body.from_bf(wb, t), body.spin)

        # Mass properties: the probe's inertia is about the probe-time centre
        # of mass, which is the origin of these part positions.
        it = v["inertia_tensor"]
        self.inertia0 = inertia_at_part_coms(
            ((it[0], it[1], it[2]), (it[3], it[4], it[5]), (it[6], it[7], it[8])), self.parts)
        self.mass0 = sum(p.mass() for p in self.parts)
        self._mass_props()
        self.cfg = model.get("partcfg") or {}

        self.control = Controls(s)
        self.control.surface_rate = self._surface_rate()
        self._init_airbrakes(model)
        self.sync_surfaces = SYNC_SURFACES and any(
            m["name"] == "SyncModuleControlSurface"
            for cs in self.control_surfaces for m in self.parts[cs["part"]].rec.get("modules", []))
        # Temperatures, every THERMAL_EVERY physics steps (they move on the
        # scale of seconds).
        self.thermal = Thermal(self, model) if model.get("dragcubes") else None
        self._thermal_n = 0
        self.autopilot = AttitudeController()
        self.autopilot_frame = None
        names = {"prelaunch": "PreLaunch", "orbiting": "Orbiting", "sub_orbital": "SubOrbital",
                 "escaping": "Escaping", "flying": "Flying", "landed": "Landed",
                 "splashed": "Splashed", "docked": "Docked"}
        sit = v.get("situation", "orbiting")
        self.situation = names.get(sit.lower(), sit)
        # Each surface's mixed input as actually reached (actuator lag).
        self.deflections = [0.0] * (len(self.aero.surfaces) if self.aero else 0)
        self.eff = [0.0, 0.0, 0.0]          # axis inputs, for per-axis tables
        self.gear_deploy = 1.0 if s.get("gear") else 0.0
        self.drain_active = False
        # Reaction wheels answer through a lag (ModuleReactionWheel:
        # Lerp(input, command, torqueResponseSpeed * dt), default 30 -- the
        # torque of a full step reads 60%, 84%, 94% of full on successive
        # frames of qs_plane's inertia test).
        self.wheel_rate = [self._module_value(w["part"], "ModuleReactionWheel",
                                              "torqueResponseSpeed", 30.0)
                           for w in self.reaction_wheels]
        self.wheel_input = [[0.0, 0.0, 0.0] for _ in self.reaction_wheels]
        # Electric charge: what the wheels draw per second at unit input sum,
        # what command modules draw always, what generators and (with the
        # engine running) alternators make.
        self.wheel_ec = [self._module_rate(w["part"], "ModuleReactionWheel", "inputs")
                         for w in self.reaction_wheels]
        self.ec_draw = sum(self._module_rate(i, "ModuleCommand", "inputs")
                           for i in range(len(self.parts)))
        self.ec_make = sum(self._module_rate(i, "ModuleGenerator", "outputs")
                           for i in range(len(self.parts)))
        for e in self.engines:
            e.actuation = [0.0, 0.0, 0.0]
        self.rcs_force = (0.0, 0.0, 0.0)
        self.last_force = (0.0, 0.0, 0.0)   # aerodynamic, body frame (kRPC's bf)
        self.last_aero_body = (0.0, 0.0, 0.0)
        self.last_torque = (0.0, 0.0, 0.0)
        self.env = None
        self.contact = False
        self._touching_parts = set()
        self.destroyed = False
        self.surface_torque = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    # -- part configs -------------------------------------------------------

    def _modules(self, part_index):
        rec = self.cfg.get(self.parts[part_index].name) or {}
        return rec.get("modules", [])

    def _module_value(self, part_index, module, field, default):
        for m in self._modules(part_index):
            if m["name"] == module and field in m:
                try:
                    return float(m[field])
                except ValueError:
                    return default
        return default

    # -- airbrakes and action groups ------------------------------------------

    def _init_airbrakes(self, model):
        """ModuleAeroSurface state, per control surface that is one."""
        self.airbrakes = {}
        if not self.aero:
            return
        for k, cs in enumerate(self.control_surfaces):
            if k not in self.aero.brakes:
                continue
            p = self.parts[cs["part"]]
            fields = next((m.get("fields", {}) for m in p.rec.get("modules", [])
                           if m["name"] == "ModuleAeroSurface"), {})

            def num(name, default):
                try:
                    return float(fields.get(name, default))
                except (TypeError, ValueError):
                    return default
            self.airbrakes[k] = {
                "deploy": bool(cs.get("deployed")),
                "ignore": [not cs.get("pitch", False), not cs.get("roll", False),
                           not cs.get("yaw", False)],
                "deploy_angle": num("Deploy Angle", 70.0),
                "authority": num("Authority Limiter", 100.0),
                "range": self._module_value(cs["part"], "ModuleAeroSurface", "ctrlSurfaceRange", 70.0),
                "speed": self._module_value(cs["part"], "ModuleAeroSurface", "actuatorSpeed", 20.0),
                "angle": 0.0,
            }
        self.action_groups = [False] * 10
        self.actions = model.get("actions") or []

    def fire_group(self, group, state):
        """An action group changed: its bound actions fire (KSP
        ActionGroupList.SetGroup; kRPC's group N is CustomN, 0 is Custom10)."""
        name = "Custom%02d" % (group if group else 10)
        for a in self.actions:
            if name in a["groups"]:
                self._do_action(a, state, name)

    def set_brakes(self, state):
        for a in self.actions:
            if "Brakes" in a["groups"]:
                self._do_action(a, state, "Brakes")

    def _do_action(self, a, state, group):
        k = next((k for k, cs in enumerate(self.control_surfaces) if cs["part"] == a["part"]), None)
        b = self.airbrakes.get(k)
        if b is None:
            return
        act = a["action"]
        if act == "ActionToggleBrakes":
            b["deploy"] = bool(state) if group == "Brakes" else not b["deploy"]
        elif act == "ActionToggle":
            b["deploy"] = not b["deploy"]
        elif act == "ActionExtend":
            b["deploy"] = True
        elif act == "ActionRetract":
            b["deploy"] = False
        elif act in ("TogglePitch", "ToggleRoll", "ToggleYaw"):
            i = {"TogglePitch": 0, "ToggleRoll": 1, "ToggleYaw": 2}[act]
            b["ignore"][i] = not b["ignore"][i]

    def _airbrake_step(self, cmd, dt):
        """ModuleAeroSurface.CtrlSurfaceUpdate: open clamp01(input . lever
        direction) x range x authority, one-sided, plus the deploy angle when
        deployed; clamp to 1.5 x range; MoveTowards at actuatorSpeed."""
        up = (0.0, 1.0, 0.0)
        for k, b in self.airbrakes.items():
            cs = self.control_surfaces[k]
            p = self.parts[cs["part"]]
            x = 0.0 if b["ignore"][0] else cmd[0]
            z = 0.0 if b["ignore"][2] else cmd[2]
            base_up = quat.rotate(p.rotation, (0.0, 1.0, 0.0))
            lever = vec.sub(self.com, p.position)
            lever_h = vec.sub(lever, vec.scale(up, vec.dot(lever, up)))
            proj = vec.scale(up, vec.dot(base_up, up))      # Project(base.up, -up)
            rhs = vec.cross(proj, lever_h)
            action = max(0.0, min(1.0, x * rhs[0] + z * rhs[2])) * b["range"] * b["authority"] * 0.01
            if b["deploy"]:
                action += b["deploy_angle"]
            cap = 1.5 * b["range"]
            action = max(-cap, min(cap, action))
            step = b["speed"] * dt
            b["angle"] += max(-step, min(step, action - b["angle"]))

    def _module_rate(self, part_index, module, key, resource="ElectricCharge"):
        total = 0.0
        for m in self._modules(part_index):
            if m["name"] == module:
                for r in m.get(key, []):
                    if r.get("name") == resource:
                        total += float(r.get("rate", 0.0))
        return total

    def add_resource(self, name, amount):
        """Put up to ``amount`` units in, filling tanks evenly; returns put."""
        pool = [r for p in self.parts if p.alive for r in p.resources
                if r.name == name and r.amount < r.max]
        room = sum(r.max - r.amount for r in pool)
        if room <= 0 or amount <= 0:
            return 0.0
        put = min(room, amount)
        for r in pool:
            r.amount += put * (r.max - r.amount) / room
        return put

    def electric(self, dt):
        """Generators, alternators and command modules, per step (wheels
        draw their own in ``wheels_torque``)."""
        make = self.ec_make
        for e in self.engines:
            if e.thrust > 0 and e.max_vacuum_thrust > 0:
                rate = self._module_rate(e.part.index, "ModuleAlternator", "inputs")
                make += rate * min(1.0, e.thrust / e.max_vacuum_thrust)
        if make:
            self.add_resource("ElectricCharge", make * dt)
        if self.ec_draw:
            self.draw("ElectricCharge", self.ec_draw * dt)

    def _surface_rate(self):
        """Control-surface travel, fraction of full deflection per second.

        AtmosphereAutopilot's ``SyncModuleControlSurface`` (installed on the
        farm) replaces stock surfaces and moves every one at the same rate,
        2.0 per second: the one-step pitch torque of qs_cone, qs_shuttle_cone
        and qs_shuttle_final is best at 2.0 of 1.2-4.0.  Stock surfaces move
        ``actuatorSpeed`` degrees per second over ``ctrlSurfaceRange``."""
        rates = []
        for cs in self.control_surfaces:
            names = [m["name"] for m in cs.get("modules", [])] or \
                [m["name"] for m in self.parts[cs["part"]].rec.get("modules", [])]
            if "SyncModuleControlSurface" in names:
                return 2.0
            speed = self._module_value(cs["part"], "ModuleControlSurface", "actuatorSpeed", None)
            rng = self._module_value(cs["part"], "ModuleControlSurface", "ctrlSurfaceRange", None)
            if speed and rng:
                rates.append(speed / rng)
        return min(rates) if rates else 2.0

    # -- mass -----------------------------------------------------------

    def _mass_props(self):
        self._mass_dirty = False
        m = 0.0
        c = [0.0, 0.0, 0.0]
        for p in self.parts:
            pm = p.mass()
            m += pm
            for k in range(3):
                c[k] += pm * p.com[k]
        self.mass = m
        self.com = tuple(x / m for x in c) if m > 0 else (0.0, 0.0, 0.0)
        # Inertia about the probe origin, adjusted by each part's resource
        # change as a point mass, then moved to the new centre of mass.
        io = [list(row) for row in self.inertia0]
        for p in self.parts:
            dm = p.mass() - (p.dry_mass0 + p.resource_mass0)
            if dm == 0.0:
                continue
            d = p.com
            dd = vec.dot(d, d)
            for i in range(3):
                for j in range(3):
                    io[i][j] += dm * ((dd if i == j else 0.0) - d[i] * d[j])
        cc = vec.dot(self.com, self.com)
        for i in range(3):
            for j in range(3):
                io[i][j] -= m * ((cc if i == j else 0.0) - self.com[i] * self.com[j])
        self.inertia = tuple(tuple(row) for row in io)
        self.inertia_inv = vec.mat_inv(self.inertia)

    def moi(self):
        return (self.inertia[0][0], self.inertia[1][1], self.inertia[2][2])

    def resource_amount(self, name):
        return sum(r.amount for p in self.parts for r in p.resources if r.name == name)

    def resource_max(self, name):
        return sum(r.max for p in self.parts for r in p.resources if r.name == name)

    def resource_names(self):
        names = []
        for p in self.parts:
            for r in p.resources:
                if r.name not in names:
                    names.append(r.name)
        return names

    def draw(self, name, amount):
        """Take up to ``amount`` units of a resource, evenly; returns taken."""
        pool = [r for p in self.parts for r in p.resources if r.name == name and r.amount > 0]
        have = sum(r.amount for r in pool)
        if have <= 0 or amount <= 0:
            return 0.0
        take = min(have, amount)
        for r in pool:
            r.amount -= take * r.amount / have
        if pool[0].density > 0:
            self._mass_dirty = True
        return take

    # -- geometry ---------------------------------------------------------

    def q_bf(self):
        """Vessel rotation in the rotating body frame."""
        return quat.mul(quat.conj(self.world.body.q_bf(self.world.t)), self.q)

    def r_bf(self):
        return self.world.body.to_bf(self.r, self.world.t)

    def v_bf(self):
        body = self.world.body
        rb = body.to_bf(self.r, self.world.t)
        return vec.sub(body.to_bf(self.v, self.world.t), vec.cross(body.spin, rb))

    # -- forces -----------------------------------------------------------

    def environment(self):
        w = self.world
        body = w.body
        rb = body.to_bf(self.r, w.t)
        alt, p, temp, rho, sound = body.air(rb, w.t)
        vb = vec.sub(body.to_bf(self.v, w.t), vec.cross(body.spin, rb))
        qbf = quat.mul(quat.conj(body.q_bf(w.t)), self.q)
        v_body = quat.rotate(quat.conj(qbf), vb)
        alpha, beta, speed = Aero.angles(v_body)
        lat, lon = body.lat_lon(rb)
        terrain = body.terrain.height(lat, lon)
        env = {
            "r_bf": rb, "v_bf": vb, "q_bf": qbf, "alt": alt, "pressure": p,
            "temperature": temp, "density": rho, "sound": sound, "speed": speed,
            "mach": speed / sound if sound > 0 else 0.0,
            "q": 0.5 * rho * speed * speed, "alpha": alpha, "beta": beta,
            "lat": lat, "lon": lon, "terrain": terrain,
            "v_body": v_body,
        }
        self.env = env
        return env

    def aero_wrench(self, env, controls=None, omega_rel_body=None, gear=None,
                    deflections=None):
        """(force, torque) in body axes about the current centre of mass."""
        if self.aero is None or env["q"] <= 0.0:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if controls is None and not self.aero.surfaces:
            controls = {"pitch": self.eff[0], "roll": self.eff[1], "yaw": self.eff[2]}
        if deflections is None:
            deflections = self.deflections
        c = self.aero.coefficients(env["mach"], env["alpha"], env["beta"], controls,
                                   omega_rel_body, self.gear_deploy if gear is None else gear,
                                   speed=env.get("speed"), deflections=deflections,
                                   reynolds=env.get("density", 0.0) * env.get("speed", 0.0),
                                   brakes=[(k, b["angle"]) for k, b in self.airbrakes.items()])
        q = env["q"]
        f = (c[0] * q, c[1] * q, c[2] * q)
        t = (c[3] * q, c[4] * q, c[5] * q)
        # Tables are about the probe-time centre of mass (``aero_origin``).
        t = vec.add(t, vec.cross(vec.sub(self.aero_origin, self.com), f))
        if self.residual is not None:
            # Fitted about the centre of mass the vessel had in flight.
            r = self.residual_coefficients(env, omega_rel_body, controls)
            f = (f[0] + r[0] * q, f[1] + r[1] * q, f[2] + r[2] * q)
            t = (t[0] + r[3] * q, t[1] + r[4] * q, t[2] + r[5] * q)
        return f, t

    def residual_coefficients(self, env, omega, controls=None):
        """``tools/calibrate.py``'s fit of what flight tests measured and the
        tables did not, over dynamic pressure, faded out over 0.3 Mach beyond
        the range it was fitted in."""
        res = self.residual
        mach = env["mach"]
        lo, hi = res["mach"]
        out = max(lo - mach, mach - hi, 0.0)
        w = max(0.0, 1.0 - out / 0.3)
        if w <= 0.0:
            return (0.0,) * 6
        m = min(max(mach, lo), hi)
        a = math.radians(env["alpha"])
        b = math.radians(env["beta"])
        k = res["ref_length"] / max(env.get("speed") or 1.0, 1.0)
        om = omega or (0.0, 0.0, 0.0)
        cmd = getattr(self, "command", (0.0, 0.0, 0.0))
        base = [1.0, a, b, a * a, a * b, b * b, om[0] * k, om[1] * k, om[2] * k,
                cmd[0], cmd[1], cmd[2]]
        x = base + [v * m for v in base]
        coef = res["coef"]
        return tuple(w * sum(x[i] * coef[i][j] for i in range(len(x))) for j in range(6))

    def surface_authority(self, env):
        """Available control-surface torque (+, -) per axis **as kRPC reports
        it**, which is what its attitude controller tunes on.

        KSP's ModuleControlSurface.GetPotentialTorque (decompiled): the
        surface's extra lift at full deflection each way, L, at its lever r
        from the centre of mass, gives Scale(r x L, r) -- the torque
        multiplied component by component by the lever arm again -- with the
        roll component negated; kRPC takes the magnitude of each component
        and sums the surfaces.  On qs_cone the game reported (77, 78, 1)
        kN m where "torque at full deflection" gives (60, 34, 21)."""
        aero = self.aero
        if aero is None or env["q"] <= 0.0 or not (aero.controls or aero.surfaces):
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if aero.surfaces and len(aero.surfaces) == len(self.control_surfaces):
            pos = [0.0, 0.0, 0.0]
            neg = [0.0, 0.0, 0.0]
            q = env["q"]
            a = math.radians(env["alpha"])
            flow = (0.0, math.cos(a), math.sin(a))      # the tables' beta-0 airflow
            for k, cs in enumerate(self.control_surfaces):
                r = vec.sub(self.parts[cs["part"]].com, self.com)
                on = (cs.get("pitch", True), cs.get("roll", True), cs.get("yaw", True))
                # KSP's positive deflection is a direction on the part, and a
                # surface's mirrored twin deflects the other way for the same
                # input, so a symmetric pair puts one of each direction into
                # pos and the other into neg: the game's two came out equal at
                # every alpha (pairing +input with pos everywhere: 95/71).  Each
                # surface counts the mean of its two directions on each side.
                # The potential is against the *undeflected* surface
                # (baseLiftForce), whatever the deflection now.
                # GetPotentialLift is lift only; the probed delta carries the
                # deflection's drag too (large at high alpha), so the part
                # along the airflow comes off.
                both = [0.0, 0.0, 0.0]
                for level in (1.0, -1.0):
                    d = aero.surface_delta(k, env["mach"], env["alpha"], level)
                    f = (d[0], d[1], d[2])
                    f = vec.sub(f, vec.scale(flow, vec.dot(f, flow)))
                    t = vec.cross(r, (f[0] * q, f[1] * q, f[2] * q))
                    for i in range(3):
                        if on[i]:
                            both[i] += 0.5 * abs(t[i] * r[i])
                for i in range(3):
                    pos[i] += both[i]
                    neg[i] += both[i]
            return tuple(pos), tuple(neg)
        base = aero.coefficients(env["mach"], env["alpha"], env["beta"], None, None,
                                 self.gear_deploy)
        pos = [0.0, 0.0, 0.0]
        neg = [0.0, 0.0, 0.0]
        q = env["q"]
        for i, axis in enumerate(("pitch", "roll", "yaw")):
            for level, out in ((1.0, pos), (-1.0, neg)):
                if aero.surfaces:
                    u = [0.0, 0.0, 0.0]
                    u[i] = level
                    c = aero.coefficients(env["mach"], env["alpha"], env["beta"], None, None,
                                          self.gear_deploy,
                                          deflections=aero.surface_targets(u[0], u[1], u[2]))
                elif axis in aero.controls:
                    c = aero.coefficients(env["mach"], env["alpha"], env["beta"],
                                          {axis: level}, None, self.gear_deploy)
                else:
                    continue
                out[i] = max(out[i], abs(c[3 + i] - base[3 + i]) * q)
        return tuple(pos), tuple(neg)

    def reaction_wheel_torque(self):
        """What kRPC *reports* (``available_reaction_wheel_torque``): it does
        not look at electric charge -- qs_cone's pod has none, its wheels
        push nothing in the game, and kRPC still says 15 kN m, which is what
        its attitude controller tunes on.  The physics is ``wheels_torque``."""
        pos = [0.0, 0.0, 0.0]
        for w in self.reaction_wheels:
            if not w.get("active", True):
                continue
            lim = w.get("authority_limiter", 100.0)
            lim = lim / 100.0 if lim > 1.0 else lim
            for i in range(3):
                pos[i] += abs(w["max_torque"][0][i]) * lim
        return tuple(pos)

    def rcs_torque(self):
        if not self.control.rcs:
            return (0.0, 0.0, 0.0)
        t = self.model["vessel"].get("available_rcs_torque")
        if not t or self.resource_amount("MonoPropellant") <= 0:
            return (0.0, 0.0, 0.0)
        return tuple(max(abs(t[0][i]), abs(t[1][i])) for i in range(3))

    def engine_gimbal_torque(self, p_atm):
        """ModuleGimbal.GetPotentialTorque, as kRPC reports it: per engine,
        |pivot - CoM| x thrust x (sin|x| + sin|z|) of the full-input gimbal
        angles on pitch and yaw, and the pivot's distance off the roll axis
        for roll."""
        pos = [0.0, 0.0, 0.0]
        for e in self.engines:
            if not e.active or e.thrust <= 0 or not e.thrusters:
                continue
            th = e.thrusters[0]
            pivot = tuple(th.get("gimbal_position") or th["position"])
            r = vec.sub(pivot, self.com)
            mag = vec.norm(r)
            perp = math.hypot(r[0], r[2])
            for i, u in ((0, (1.0, 0.0, 0.0)), (2, (0.0, 0.0, 1.0)), (1, (0.0, 1.0, 0.0))):
                if i == 1 and perp <= self.GIMBAL_MIN_ROLL_OFFSET:
                    continue
                tx, tz = self._gimbal_target(e, u, pivot)
                arm = perp if i == 1 else mag
                pos[i] += (math.sin(math.radians(abs(tx))) + math.sin(math.radians(abs(tz)))) \
                    * arm * e.thrust
        return tuple(pos)

    def available_torque(self, env):
        rw = self.reaction_wheel_torque()
        cs = self.surface_torque[0]
        rc = self.rcs_torque()
        en = self.engine_gimbal_torque(env["pressure"] / 101.325 if env else 0.0)
        return tuple(rw[i] + cs[i] + rc[i] + en[i] for i in range(3))

    # -- actuators ------------------------------------------------------------

    def wheels_torque(self, cmd, dt):
        """Body torque of the reaction wheels (ModuleReactionWheel, decompiled):
        the torque lerps toward the command, and is applied only while the
        wheel is operational -- input nonzero *and* 90% of the charge it
        asked for (rate x (|pitch| + |roll| + |yaw|) per second) delivered.
        So a wheel stops the frame its input goes to zero, and an empty pod
        has none: qs_entry's pod ran dry holding the nose up and the game's
        wheels gave nothing from Mach 3 down, where the simulator's gave
        15 kN m (and landed the flight 7 km longer)."""
        out = [0.0, 0.0, 0.0]
        for k, w in enumerate(self.reaction_wheels):
            if not w.get("active", True):
                continue
            lim = w.get("authority_limiter", 100.0)
            lim = lim / 100.0 if lim > 1.0 else lim
            state = self.wheel_input[k]
            a = min(1.0, self.wheel_rate[k] * dt)
            for i in range(3):
                state[i] += (cmd[i] * lim - state[i]) * a
            need_sum = (abs(cmd[0]) + abs(cmd[1]) + abs(cmd[2])) * lim
            if need_sum <= 0.0:
                continue
            rate = self.wheel_ec[k] if k < len(self.wheel_ec) else 0.0
            if rate > 0.0:
                need = rate * need_sum * dt
                if self.draw("ElectricCharge", need) < 0.9 * need:
                    continue
            for i in range(3):
                out[i] -= state[i] * abs(w["max_torque"][0][i])
        return tuple(out)

    def rcs_wrench(self, rot, lin, p_atm, dt):
        """RCS force and torque, body axes, one nozzle at a time.

        ModuleRCS: a nozzle whose exhaust points along ``e``, at ``p`` from
        the centre of mass, fires at max(e . unit(w x p) |w|, 0) +
        max(e . l, 0) of its power, clamped to 1, where ``w`` is the
        rotation input (pitch, roll, yaw) and ``l`` the translation input
        (-right, -forward, up); the force is along -e.  Measured on
        qs_shuttle: forward 1.0 gave 7875 N from four aft nozzles and four
        nose nozzles tilted 10 degrees (4000 + 4 x 0.9845^2 x 1000).  The
        lever is a direction only: with w x p/|p| instead, roll -- where the
        nozzles sit far along the axis -- came out 1/|w^ x p^| weak, 5.6x on
        the shuttle and 1.5x on the old plane.  Thrust follows the Isp curve
        at a fixed fuel flow."""
        force = [0.0, 0.0, 0.0]
        torque = [0.0, 0.0, 0.0]
        right, forward, up = lin
        flow = 0.0
        for r in self.rcs:
            if not r.get("enabled", True) or not r["thrusters"]:
                continue
            w = (rot[0] if r.get("pitch", True) else 0.0,
                 rot[1] if r.get("roll", True) else 0.0,
                 rot[2] if r.get("yaw", True) else 0.0)
            l_in = (-right if r.get("right", True) else 0.0,
                    -forward if r.get("forward", True) else 0.0,
                    up if r.get("up", True) else 0.0)
            if not any(w) and not any(l_in):
                continue
            lim = r.get("thrust_limit", 1.0)
            lim = lim / 100.0 if lim > 1.0 else lim
            power = r["max_vacuum_thrust"] * lim
            isp_vac = r.get("vacuum_isp") or 240.0
            isp = isp_vac
            curve = r.get("isp_curve")
            if curve and p_atm > 0:
                isp = interp([(k[0], k[1]) for k in curve], p_atm)
            scale = isp / isp_vac
            nozzles = r.get("_nozzles")
            if nozzles is None:
                live = r.get("live") or [True] * len(r["thrusters"])
                nozzles = r["_nozzles"] = [
                    (tuple(th["position"]), vec.scale(tuple(th["direction"]), -1.0))
                    for th, ok in zip(r["thrusters"], live) if ok]
            fmin = r.get("full_thrust_min", 0.2)
            for p_th, e in nozzles:
                pos = vec.sub(p_th, self.com)
                level = 0.0
                if any(w):
                    tv = vec.cross(w, pos)
                    n = vec.norm(tv)
                    if n > 1e-9:
                        level += max(vec.dot(e, tv) / n * vec.norm(w), 0.0)
                level += max(vec.dot(e, l_in), 0.0)
                if level <= 1e-4:
                    continue
                if r.get("full_thrust") and level >= fmin:
                    level = 1.0
                level = min(level, 1.0)
                f = vec.scale(e, -level * power * scale)
                for i in range(3):
                    force[i] += f[i]
                t = vec.cross(pos, f)
                for i in range(3):
                    torque[i] += t[i]
                flow += level * power / (isp_vac * G0)
        if flow > 0:
            prop = self.rcs[0].get("propellants") or ["MonoPropellant"]
            got = 0.0
            for name in prop:
                got += self.draw(name, flow * dt / len(prop) / self._density(name))
            if got <= 0.0:
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        return tuple(force), tuple(torque)

    GIMBAL_MIN_ROLL_OFFSET = 0.1      # ModuleGimbal.minRollOffset

    def _gimbal_frame(self):
        """The vessel's control point: ModuleGimbal decides roll from the
        engine's offset from it, not from the centre of mass."""
        ctl = self.model["vessel"].get("controlling")
        if ctl is None:
            ctl = self.model["vessel"].get("root") or 0
        return self.parts[ctl].position

    def _gimbal_target(self, e, cmd, pivot):
        """ModuleGimbal.GimbalRotation for the input (pitch, roll, yaw): the
        gimbal turns about the vessel's pitch axis for pitch and its yaw axis
        for yaw, both reversed for an engine ahead of the centre of mass; roll
        goes to the axis an off-centre engine can roll with, at full input;
        each clamped to 1 and scaled by the range (degrees)."""
        g = e.rec.get("gimbal") or {}
        rng = g.get("range", e.gimbal_range) * g.get("limit", 1.0)
        if rng <= 0 or e.gimbal_locked:
            return 0.0, 0.0
        x, roll, z = cmd
        if self.com[1] < pivot[1]:
            x, z = -x, -z
        if roll:
            ref = self._gimbal_frame()
            off_x, off_z = pivot[0] - ref[0], pivot[2] - ref[2]
            m = self.GIMBAL_MIN_ROLL_OFFSET
            if off_x > m:
                x += roll
            elif off_x < -m:
                x -= roll
            if off_z > m:
                z += roll
            elif off_z < -m:
                z -= roll
        return max(-1.0, min(1.0, x)) * rng, max(-1.0, min(1.0, z)) * rng

    def _gimbal_step(self, e, cmd, dt):
        """Move an engine's gimbal toward the input, lerping the angles at
        ``gimbalResponseSpeed`` when ``useGimbalResponseSpeed`` (the farm's
        AtmosphereAutopilot patch sets it on every gimbal)."""
        th = e.thrusters[0] if e.thrusters else {}
        pivot = tuple(th.get("gimbal_position") or th.get("position") or (0.0, 0.0, 0.0))
        tx, tz = self._gimbal_target(e, cmd, pivot)
        g = e.rec.get("gimbal") or {}
        a = min(1.0, g.get("response_speed", 10.0) * dt) if g.get("use_response") else 1.0
        e.actuation[0] += (tx - e.actuation[0]) * a
        e.actuation[2] += (tz - e.actuation[2]) * a

    def _gimballed(self, e, th):
        """(thrust direction, point of application) of a nozzle with the
        gimbal's current angles: turned about the vessel's pitch axis, then
        its yaw axis, and swinging about the gimbal's pivot -- so the
        torque's lever is the pivot's (qs_plane: -1.78 m, the nozzle -3.50)."""
        d = tuple(th["direction"])
        p = tuple(th["position"])
        ax, az = e.actuation[0], e.actuation[2]
        if not ax and not az:
            return d, p
        rot = quat.mul(quat.from_axis_angle((0.0, 0.0, 1.0), math.radians(az)),
                       quat.from_axis_angle((1.0, 0.0, 0.0), math.radians(ax)))
        pivot = tuple(th.get("gimbal_position") or p)
        d2 = quat.rotate(rot, d)
        at = vec.add(pivot, quat.rotate(rot, vec.sub(p, pivot)))
        return d2, at

    # -- one physics step ---------------------------------------------------

    def step(self, dt):
        if self.mass <= 0.0:
            # Nothing left to move (every part destroyed).
            self.destroyed = True
            return
        force, torque = self.wrench(dt)
        if self.mass <= 0.0:                 # destroyed on this step
            self.destroyed = True
            return
        self.integrate(force, torque, dt)

    def wrench(self, dt):
        """Everything but gravity that acts over the next step, (force, torque)
        in body axes about the centre of mass.  Moves the actuators and draws
        the resources the step uses; does not move the vessel."""
        w = self.world
        body = w.body
        env = self.environment()
        ctrl = self.control
        p_atm = env["pressure"] / 101.325

        # Control-surface authority for the auto-pilot: KSP recomputes it
        # every frame; at 10 Hz it is indistinguishable and half the cost.
        self._authority_age = getattr(self, "_authority_age", 99) + 1
        if self._authority_age >= 5:
            self.surface_torque = self.surface_authority(env)
            self._authority_age = 0

        # Rotation commands: the auto-pilot if engaged, else the client's.
        if self.autopilot.engaged:
            frame = self.autopilot_frame
            q_ap = frame.rotation_of(self.q, w) if frame is not None else env["q_bf"]
            w_rel = frame.angular_velocity_of(self.w, w) if frame is not None else \
                body.to_bf(vec.sub(self.w, body.spin), w.t)
            torque = self.available_torque(env)
            p, r, y = self.autopilot.update(w.t, dt, q_ap, w_rel, torque, self.moi())
            # kRPC adds a client's manual inputs to the attitude
            # controller's output (PilotAddon.OnFlyByWire), it does not
            # replace them.
            cmd = [p + ctrl.pitch, r + ctrl.roll, y + ctrl.yaw]
        else:
            cmd = [ctrl.pitch, ctrl.roll, ctrl.yaw]
        for i in range(3):
            cmd[i] = max(-1.0, min(1.0, cmd[i]))
        self.command = tuple(cmd)

        # Surfaces move at their actuator speed (ModuleControlSurface:
        # actuatorSpeed 30 deg/s over a 20 deg range on the stock elevons).
        rate = ctrl.surface_rate * dt
        for i in range(3):
            d = cmd[i] - self.eff[i]
            self.eff[i] += max(-rate, min(rate, d))
        self._airbrake_step(cmd, dt)
        if self.deflections:
            if self.sync_surfaces:
                # AtmosphereAutopilot moves "all control surfaces in one
                # phase": the axis inputs lag, then every surface mixes them.
                self.deflections = self.aero.surface_targets(*self.eff)
                for k in self.airbrakes:
                    self.deflections[k] = 0.0
            else:
                for i, t in enumerate(self.aero.surface_targets(cmd[0], cmd[1], cmd[2])):
                    if i in self.airbrakes:
                        t = 0.0
                    d = t - self.deflections[i]
                    self.deflections[i] += max(-rate, min(rate, d))
        # Gear deploys in a few seconds.
        target = 1.0 if ctrl.gear else 0.0
        self.gear_deploy += max(-dt / 4.0, min(dt / 4.0, target - self.gear_deploy))

        # Air-relative body rates, for damping.
        w_rel_body = quat.rotate(quat.conj(self.q), vec.sub(self.w, body.spin))
        f_aero, t_aero = self.aero_wrench(env, omega_rel_body=w_rel_body)
        self.last_aero_body = f_aero
        force = f_aero
        torque = t_aero

        # Electric charge made and drawn, then the wheels (a positive command
        # drives the body rate negative, kRPC's controller convention).
        self.electric(dt)
        torque = vec.add(torque, self.wheels_torque(cmd, dt))

        # RCS, nozzle by nozzle.
        if ctrl.rcs and self.rcs:
            lin = (ctrl.right, ctrl.forward, ctrl.up)
            f_rcs, t_rcs = self.rcs_wrench(cmd, lin, p_atm, dt)
            force = vec.add(force, f_rcs)
            torque = vec.add(torque, t_rcs)
            self.rcs_force = f_rcs
        else:
            self.rcs_force = (0.0, 0.0, 0.0)

        # Engines.
        throttle = ctrl.throttle
        for e in self.engines:
            e.thrust = 0.0
            self._gimbal_step(e, cmd, dt)
            if not e.active or throttle <= 0.0:
                continue
            thrust = throttle * (e.thrust_limit if e.thrust_limit <= 1.0 else e.thrust_limit / 100.0) \
                * e.max_thrust(p_atm)
            isp = e.isp(p_atm)
            if thrust <= 0 or isp <= 0:
                continue
            mdot = thrust / (isp * G0)
            dens = sum(pp["ratio"] * self._density(pp["name"]) for pp in e.propellants)
            ok = True
            for pp in e.propellants:
                need = mdot * pp["ratio"] / dens * dt if dens > 0 else 0.0
                if self.resource_amount(pp["name"]) < need:
                    ok = False
            if not ok:
                e.flameout = True
                continue
            for pp in e.propellants:
                self.draw(pp["name"], mdot * pp["ratio"] / dens * dt)
            e.thrust = thrust
            n = len(e.thrusters)
            for th in e.thrusters:
                d, at = self._gimballed(e, th)
                fb = vec.scale(d, thrust / n)
                force = vec.add(force, fb)
                torque = vec.add(torque, vec.cross(vec.sub(at, self.com), fb))

        # Drain valves (KSP's ModuleResourceDrain): every open valve takes
        # its drain rate of the vessel's whole capacity per second, and
        # vents it as thrust along its own part's transform.forward -- so
        # a mirrored pair cancels.  One lumped valve pushing +z put 2 m/s
        # and a slow yaw into the shuttle's circular orbit.
        if self.drain_active and self.drains:
            vented = 0.0
            for dr in self.drains:
                out = 0.0
                for name in DRAINABLE:
                    mx = self.resource_max(name)
                    take = self.draw(name, mx * ctrl.drain_rate / 100.0 * dt)
                    out += take * self._density(name)
                if out > 0:
                    part = self.parts[dr["part"]]
                    f = out / dt * DRAIN_ISP * G0
                    direction = ctrl.drain_direction or quat.rotate(tuple(part.rotation), (0.0, 0.0, 1.0))
                    fb = vec.scale(direction, f)
                    force = vec.add(force, fb)
                    torque = vec.add(torque, vec.cross(vec.sub(part.position, self.com), fb))
                vented += out
            if vented <= 0:
                self.drain_active = False

        # Mass properties only move when propellant does.
        if getattr(self, "_mass_dirty", True):
            self._mass_props()

        # Ground.
        f_ground, t_ground = self.ground(env, dt)
        force = vec.add(force, f_ground)
        torque = vec.add(torque, t_ground)

        self.last_torque = torque
        return force, torque

    def integrate(self, force, torque, dt):
        """Velocity by Euler, position by the mean of the old and new
        velocity.  Against the game's own frames in orbit, PhysX's symplectic
        order (position from the new velocity) sank 0.37 m in 5 s -- a*dt*t/2
        -- and explicit Euler rose as much; the mean matches to 3 mm."""
        body = self.world.body
        env = self.env
        g = vec.scale(self.r, -body.mu / vec.norm(self.r) ** 3)
        a = vec.add(g, vec.scale(quat.rotate(self.q, force), 1.0 / self.mass))
        v0 = self.v
        self.v = vec.add(self.v, vec.scale(a, dt))
        self.r = vec.add(self.r, vec.scale(vec.add(v0, self.v), 0.5 * dt))
        wb = quat.rotate(quat.conj(self.q), self.w)
        iw = vec.mat_vec(self.inertia, wb)
        dw = vec.mat_vec(self.inertia_inv, vec.sub(torque, vec.cross(wb, iw)))
        wb = vec.add(wb, vec.scale(dw, dt))
        # PhysX caps a rigidbody's spin; KSP leaves the cap at 50 rad/s.
        n = vec.norm(wb)
        if n > 50.0:
            wb = vec.scale(wb, 50.0 / n)
        self.q = quat.norm(quat.mul(self.q, quat.from_rotvec(vec.scale(wb, dt))))
        self.w = quat.rotate(self.q, wb)
        self.update_situation(env)
        if self.thermal is not None:
            self._thermal_n += 1
            if self._thermal_n >= THERMAL_EVERY:
                self._thermal_n = 0
                for k in self.thermal.step(THERMAL_EVERY * dt, env):
                    self.burn_up(k)

    def burn_up(self, index):
        self.destroy_part(index, "overheated (skin %.0f K, interior %.0f K)"
                          % (self.parts[index].skin_temperature, self.parts[index].temperature))

    def destroy_part(self, index, why):
        """A part is destroyed, and with it every part attached through it;
        the root's loss is the vessel's.  The aerodynamic tables stay those
        of the whole craft (a limit of this model, logged)."""
        part = self.parts[index]
        if not part.alive:
            return
        root = self.model["vessel"].get("root") or 0
        gone = [index]
        k = 0
        while k < len(gone):
            gone += [p.index for p in self.parts
                     if p.rec.get("parent") == gone[k] and p.index not in gone]
            k += 1
        for i in gone:
            p = self.parts[i]
            p.alive = False
            for r in p.resources:
                r.amount = 0.0
            p.dry_mass = 0.0
        self._mass_props()
        if getattr(self, "thermal", None) is not None:
            self.thermal.parts_changed()
        self.world.events.append("%s %s%s" % (part.title, why,
                                              " with %d attached" % (len(gone) - 1)
                                              if len(gone) > 1 else ""))
        if index == root:
            self.destroyed = True
            self.world.events.append("vessel destroyed: the root part " + why)

    def _density(self, name):
        for p in self.parts:
            for r in p.resources:
                if r.name == name:
                    return r.density
        return 5.0

    # -- the ground ---------------------------------------------------------

    CONTACT_HZ = 2.0            # natural frequency of a contact, per point
    CONTACT_ZETA = 0.8          # damping ratio
    HULL_MU = 0.6               # sliding friction of a part on the ground

    def contact_points(self):
        """(kind, body-frame point, part index, wheel record): each deployed
        wheel's lowest point, and the corners of every live part's box -- so
        the vessel touches with its own shape, and the part that struck is
        known (KSP breaks it past its crashTolerance)."""
        pts = []
        if self.gear_deploy > 0.5:
            for wh in self.wheels:
                lo, hi = wh.get("deployed_bounding_box") or wh["bounding_box"]
                x = 0.5 * (lo[0] + hi[0])
                y = 0.5 * (lo[1] + hi[1])
                pts.append(("wheel", (x, y, hi[2]), wh["part"], wh))
        # A deployed wheel touches with its wheel collider, not its box.
        wheel_parts = {wh["part"] for wh in self.wheels} if self.gear_deploy > 0.5 else set()
        for p in self.parts:
            if not p.alive or p.index in wheel_parts:
                continue
            box = p.rec.get("bounding_box")
            if not box:
                continue
            lo, hi = box
            for x in (lo[0], hi[0]):
                for y in (lo[1], hi[1]):
                    for z in (lo[2], hi[2]):
                        pts.append(("hull", (x, y, z), p.index, None))
        return pts

    def _effective_mass(self, rel, n):
        """Mass the ground feels at ``rel`` pushing along ``n`` (body axes):
        1 / (1/m + (r x n) . I^-1 (r x n)).  A point far out on a lever is
        light, and a spring sized for the whole mass there is too stiff for a
        0.02 s step -- the first contact model launched a booster dropped at
        5 m/s 120 m back into the air."""
        rn = vec.cross(rel, n)
        return 1.0 / (1.0 / self.mass + vec.dot(rn, vec.mat_vec(self.inertia_inv, rn)))

    def ground(self, env, dt):
        w = self.world
        body = w.body
        if env["alt"] - env["terrain"] > 60.0:
            self.contact = False
            self._touching_parts = set()
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        up_bf = vec.unit(env["r_bf"])
        up_body = quat.rotate(quat.conj(env["q_bf"]), up_bf)
        v_body = quat.rotate(quat.conj(env["q_bf"]), env["v_bf"])
        w_rel_body = quat.rotate(quat.conj(self.q), vec.sub(self.w, body.spin))
        ground_h = max(env["terrain"], 0.0)
        centre_h = env["alt"] - ground_h
        touching = []
        for kind, p, part, wh in self.contact_points():
            rel = vec.sub(p, self.com)
            h = centre_h + vec.dot(rel, up_body)
            if h < 0.0:
                touching.append((kind, rel, h, part, wh))
        self.contact = bool(touching)
        was = self._touching_parts
        self._touching_parts = {t[3] for t in touching if t[0] == "hull"}
        if not touching:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        # Points touching together share the load: each gets its share of
        # the stiffness a single point would have.
        share = 1.0 / len(touching)
        om = 2.0 * math.pi * self.CONTACT_HZ
        brakes = self.control.brakes
        f_tot = (0.0, 0.0, 0.0)
        t_tot = (0.0, 0.0, 0.0)
        broken = set()
        for kind, rel, h, part, wh in touching:
            vp = vec.add(v_body, vec.cross(w_rel_body, rel))
            vn = vec.dot(vp, up_body)
            # Part.OnCollisionEnter -> HandleCollision: a part breaks when
            # its collider first meets the ground at a relative speed (the
            # whole of it, sliding included) past its crashTolerance.  Wheel
            # colliders are exempt (Wheel_Piston_Collider).  The first
            # version tested the normal speed on every step, so a 60 m/s
            # touchdown at a shallow sink kept its wings and its pod where
            # the game lost the vessel.
            if kind == "hull" and part not in was:
                tol = self.parts[part].rec.get("impact_tolerance") or 9.0
                if vec.norm(vp) > tol:
                    broken.add(part)
            m_eff = self._effective_mass(rel, up_body) * share
            fn = max(0.0, -m_eff * om * om * h - 2.0 * self.CONTACT_ZETA * m_eff * om * vn)
            f = vec.scale(up_body, fn)
            vt = vec.sub(vp, vec.scale(up_body, vn))
            sp = vec.norm(vt)
            if sp > 1e-6:
                # Friction that cannot reverse the slip within one step.
                stop = m_eff * sp / dt
                if kind == "wheel":
                    nose = vec.unit(vec.sub((0.0, 1.0, 0.0), vec.scale(up_body, up_body[1])))
                    side = vec.unit(vec.cross(up_body, nose))
                    mu_long = 0.02 + (0.6 * brakes if wh.get("has_brakes") else 0.0)
                    v_along, v_side = vec.dot(vt, nose), vec.dot(vt, side)
                    f_long = -math.copysign(min(mu_long * fn, m_eff * abs(v_along) / dt), v_along)
                    f_side = -math.copysign(min(0.8 * fn, m_eff * abs(v_side) / dt), v_side)
                    f = vec.add(f, vec.add(vec.scale(nose, f_long), vec.scale(side, f_side)))
                else:
                    f = vec.add(f, vec.scale(vt, -min(self.HULL_MU * fn, stop) / sp))
            f_tot = vec.add(f_tot, f)
            t_tot = vec.add(t_tot, vec.cross(rel, f))
        for part in broken:
            self.destroy_part(part, "struck the ground")
        return f_tot, t_tot

    def update_situation(self, env):
        body = self.world.body
        alt = env["alt"]
        speed = vec.norm(env["v_bf"])
        if self.contact and speed < 1.0:
            self.situation = "Splashed" if env["terrain"] <= 0.0 and alt < 5 else "Landed"
            return
        if self.contact:
            self.situation = "Landed" if env["terrain"] > 0 else "Splashed"
            return
        if alt < body.atmosphere_depth:
            self.situation = "Flying"
            return
        el = self.elements()
        if el["e"] >= 1.0:
            self.situation = "Escaping"
        elif el["periapsis"] - body.radius < body.atmosphere_depth:
            self.situation = "SubOrbital"
        else:
            self.situation = "Orbiting"

    def elements(self):
        mu = self.world.body.mu
        r, v = self.r, self.v
        rn = vec.norm(r)
        energy = vec.dot(v, v) / 2.0 - mu / rn
        h = vec.cross(r, v)
        evec = vec.sub(vec.scale(vec.cross(v, h), 1.0 / mu), vec.scale(r, 1.0 / rn))
        e = vec.norm(evec)
        a = -mu / (2.0 * energy) if energy != 0 else float("inf")
        p = vec.dot(h, h) / mu
        peri = p / (1.0 + e)
        apo = a * (1.0 + e) if e < 1.0 else float("inf")
        return {"a": a, "e": e, "periapsis": peri, "apoapsis": apo, "h": h, "evec": evec}


class Controls:
    __sim_object__ = True

    def __init__(self, s):
        self.throttle = s.get("throttle", 0.0)
        self.pitch = self.yaw = self.roll = 0.0
        self.forward = self.up = self.right = 0.0
        self.wheel_throttle = self.wheel_steering = 0.0
        self.sas = s.get("sas", False)
        self.rcs = s.get("rcs", False)
        self.gear = s.get("gear", False)
        self.brakes = 1.0 if s.get("brakes", False) else 0.0
        self.lights = s.get("lights", False)
        self.abort = False
        self.action_groups = list(s.get("action_groups", [False] * 10))
        # Fraction of full deflection per second: a pitch step on qs_cone
        # swung the surfaces end to end in 0.46 s of game time.
        self.surface_rate = 2.2
        self.drain_rate = 10.0         # percent of capacity per second
        self.drain_direction = None


# -- the world --------------------------------------------------------------

_MODELS = {}
MODEL_CACHE = 2


def load_model(path):
    """The model at ``path``, parsed once per server process: a model is
    80-90 MB of JSON, 0.8 s to parse under PyPy, and every flight's Load
    paid it.  ``aero`` (all but 1 MB of it) is only ever read and is shared;
    the rest is copied, because the API's setters write into the part and
    module records a vessel holds."""
    mtime = os.path.getmtime(path)
    hit = _MODELS.get(path)
    if hit is None or hit[0] != mtime:
        with open(path) as fh:
            hit = (mtime, json.load(fh))
        _MODELS.pop(path, None)
        while len(_MODELS) >= MODEL_CACHE:
            _MODELS.pop(next(iter(_MODELS)))   # a parsed model is a few hundred MB
        _MODELS[path] = hit
    return {k: (v if k == "aero" else copy.deepcopy(v)) for k, v in hit[1].items()}


class World:
    def __init__(self):
        self.loaded = False
        self.paused = False
        self.t = 0.0
        self.body = None
        self.vessel = None
        self.warp_factor = 0
        self._warp_from = self._warp_to = 1.0
        self._warp_ramp = 1.0
        self.save = None
        self.events = []
        self._acc = 0.0
        self.steps = 0

    def load(self, save):
        path = os.path.join(MODELS, save + ".json")
        if not os.path.exists(path):
            raise ValueError("no model for save %r (run kspSim/tools/probe.py)" % save)
        model = load_model(path)
        self.t = model["ut"]
        self.body = Body(model)
        self.vessel = Vessel(model, self)
        self.save = save
        self.warp_factor = 0
        self._warp_from = self._warp_to = 1.0
        self._warp_ramp = 1.0
        self._acc = 0.0
        self.events = []
        self.loaded = True
        self.paused = False

    def client_gone(self, client):
        pass

    def warp_rate(self):
        """The rate the clock runs at now.  KSP's TimeWarp (asked through
        kRPC's non-instant SetRate) lerps it from the old rate to the new one
        over one second of Time.time -- real time, since rails warp holds
        Time.timeScale at 1 -- so dropping from 10x spends ~5 game seconds
        on the way down."""
        a = min(1.0, self._warp_ramp)
        return self._warp_from + (self._warp_to - self._warp_from) * a

    @property
    def on_rails(self):
        return bool(self.warp_factor) or self._warp_ramp < 1.0

    def _set_rate_index(self, factor, instant=False):
        target = float(RAILS_RATES[factor]) if factor else 1.0
        if instant:
            self._warp_from = target
        else:
            self._warp_from = self.warp_rate()
        self._warp_to = target
        self._warp_ramp = 0.0 if self._warp_from != target else 1.0
        self.warp_factor = factor

    def allowed_warp(self, factor):
        """KSP drops rails warp to what the altitude allows, and allows none
        in the atmosphere."""
        if not self.vessel:
            return 0
        alt = vec.norm(self.vessel.r) - self.body.radius
        factor = max(0, min(factor, len(RAILS_RATES) - 1))
        if alt < self.body.atmosphere_depth:
            return 0
        while factor > 0 and alt < self.body.warp_limits[factor]:
            factor -= 1
        return factor

    def set_warp(self, factor):
        factor = self.allowed_warp(factor)
        if factor and not self.warp_factor:
            # Going on rails freezes rotation.
            self.vessel.w = (0.0, 0.0, 0.0)
        if factor != self.warp_factor:
            self._set_rate_index(factor)

    def advance(self, dt):
        """Run ``dt`` game seconds (the real-time flow)."""
        if not self.loaded or dt <= 0:
            return
        if self.on_rails:
            dt = self._rails(dt)
            if dt <= 0:
                return
        self._acc += dt
        n = int(self._acc / DT)
        self._acc -= n * DT
        for _ in range(n):
            self.step()

    def advance_to(self, ut):
        if not self.loaded:
            return
        while self.t < ut - 1e-9:
            if self.on_rails:
                self._rails(ut - self.t)
            else:
                self.step()

    def _rails(self, dt):
        """Up to ``dt`` game seconds on rails, the rate ramping as it goes;
        returns what is left once the ramp has brought it off rails."""
        v = self.vessel
        while dt > 1e-9 and self.on_rails:
            rate = self.warp_rate()
            piece = dt if self._warp_ramp >= 1.0 else min(dt, rate * 0.02)
            v.r, v.v = kepler(v.r, v.v, self.body.mu, piece)
            self.t += piece
            dt -= piece
            if self._warp_ramp < 1.0:
                self._warp_ramp = min(1.0, self._warp_ramp + piece / rate)
            # Drop out (at once, as KSP does) if the orbit has taken it
            # below the warp limit.
            allowed = self.allowed_warp(self.warp_factor)
            if allowed != self.warp_factor:
                self._set_rate_index(allowed, instant=True)
        v.environment()
        v.update_situation(v.env)
        return dt

    def step(self):
        self.vessel.step(DT)
        self.t += DT
        self.steps += 1
