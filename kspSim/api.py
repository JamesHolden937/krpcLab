"""kRPC procedures, answered from the simulated world.

``HANDLERS[(service, procedure)] = fn(world, *args)``, with ``args[0]`` the
instance for class members.  The set grew from recorded traces
(``tools/krpcproxy.py``): whatever a flight calls that is not here is logged
to ``logs/kspsim-missing.txt`` and returned as an error.

Every object handed to a client is a stable Python object (cached on the
world), so the same part or frame always crosses the wire with the same id.
"""
import math

from kspSim import quat, vec
from kspSim.world import RAILS_RATES

HANDLERS = {}
SITUATIONS = {"PreLaunch": 0, "Orbiting": 1, "SubOrbital": 2, "Escaping": 3,
              "Flying": 4, "Landed": 5, "Splashed": 6, "Docked": 7}


def rpc(name, service="SpaceCenter"):
    def deco(fn):
        HANDLERS[(service, name)] = fn
        return fn
    return deco


class Obj:
    """A handle a client may hold."""
    __sim_object__ = True

    def __init__(self, **kw):
        self.__dict__.update(kw)


def objects(world):
    """The per-load handle cache."""
    cache = getattr(world, "_api", None)
    if cache is None or cache.get("vessel_model") is not world.vessel:
        v = world.vessel
        cache = {"vessel_model": v}
        cache["body"] = Obj(kind="body")
        cache["sun"] = Obj(kind="sun")
        cache["frames"] = {}
        cache["vessel"] = Obj(kind="vessel", v=v)
        cache["parts"] = Obj(kind="parts")
        cache["control"] = v.control
        cache["autopilot"] = Obj(kind="autopilot")
        cache["orbit"] = Obj(kind="orbit")
        cache["resources"] = Obj(kind="resources", part=None)
        cache["flights"] = {}
        cache["engines"] = list(v.engines)
        cache["wheels"] = [Obj(kind="wheel", rec=w, part=v.parts[w["part"]]) for w in v.wheels]
        cache["reaction_wheels"] = [Obj(kind="rw", rec=w, part=v.parts[w["part"]])
                                    for w in v.reaction_wheels]
        cache["control_surfaces"] = [Obj(kind="cs", rec=c, part=v.parts[c["part"]])
                                     for c in v.control_surfaces]
        cache["rcs"] = [Obj(kind="rcs", rec=r, part=v.parts[r["part"]]) for r in v.rcs]
        cache["part_resources"] = {}
        world._api = cache
    return cache


# -- reference frames --------------------------------------------------------

class Frame:
    """A kRPC reference frame: origin, orientation and motion in the
    body's non-rotating frame, as functions of the world state."""
    __sim_object__ = True

    def __init__(self, kind, part=None):
        self.kind = kind
        self.part = part

    def pose(self, w):
        """(origin, rotation frame->nr, origin velocity, angular velocity)."""
        b = w.body
        v = w.vessel
        if self.kind == "body":
            return (0.0, 0.0, 0.0), b.q_bf(w.t), (0.0, 0.0, 0.0), b.spin
        if self.kind == "nonrot":
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if self.kind == "vessel":
            return v.r, v.q, v.v, v.w
        if self.kind == "part":
            off = quat.rotate(v.q, vec.sub(self.part.position, v.com))
            q = quat.mul(v.q, self.part.rotation)
            return vec.add(v.r, off), q, vec.add(v.v, vec.cross(v.w, off)), v.w
        if self.kind == "surface":
            up = vec.unit(v.r)
            north = vec.unit(vec.sub((0.0, 1.0, 0.0), vec.scale(up, up[1])))
            east = vec.cross(up, north)
            # x up, y north, z east -- as columns
            m = ((up[0], north[0], east[0]), (up[1], north[1], east[1]), (up[2], north[2], east[2]))
            return v.r, quat.from_matrix(m), v.v, b.spin
        if self.kind == "orbital":
            prograde = vec.unit(v.v)
            normal = vec.unit(vec.cross(v.r, v.v))
            anti_radial = vec.cross(prograde, normal)
            m = ((anti_radial[0], prograde[0], normal[0]),
                 (anti_radial[1], prograde[1], normal[1]),
                 (anti_radial[2], prograde[2], normal[2]))
            return v.r, quat.from_matrix(m), v.v, (0.0, 0.0, 0.0)
        raise ValueError(self.kind)

    def position(self, p_nr, w):
        o, q, _, _ = self.pose(w)
        return quat.rotate(quat.conj(q), vec.sub(p_nr, o))

    def position_to_nr(self, p, w):
        o, q, _, _ = self.pose(w)
        return vec.add(o, quat.rotate(q, p))

    def direction(self, d_nr, w):
        return quat.rotate(quat.conj(self.pose(w)[1]), d_nr)

    def direction_to_nr(self, d, w):
        return quat.rotate(self.pose(w)[1], d)

    def velocity(self, p_nr, v_nr, w):
        o, q, vo, wo = self.pose(w)
        rel = vec.sub(vec.sub(v_nr, vo), vec.cross(wo, vec.sub(p_nr, o)))
        return quat.rotate(quat.conj(q), rel)

    def velocity_to_nr(self, p, vel, w):
        o, q, vo, wo = self.pose(w)
        p_nr = vec.add(o, quat.rotate(q, p))
        return vec.add(vec.add(vo, quat.rotate(q, vel)), vec.cross(wo, vec.sub(p_nr, o)))

    def rotation_of(self, q_nr, w):
        return quat.mul(quat.conj(self.pose(w)[1]), q_nr)

    def rotation_to_nr(self, q, w):
        return quat.mul(self.pose(w)[1], q)

    def angular_velocity_of(self, w_nr, w):
        _, q, _, wo = self.pose(w)
        return quat.rotate(quat.conj(q), vec.sub(w_nr, wo))


def frame(world, kind, part=None):
    frames = objects(world)["frames"]
    key = (kind, id(part))
    if key not in frames:
        frames[key] = Frame(kind, part)
    return frames[key]


# -- SpaceCenter ---------------------------------------------------------------

@rpc("get_UT")
def ut(w):
    return w.t


@rpc("get_ActiveVessel")
def active_vessel(w):
    return objects(w)["vessel"] if w.loaded else None


@rpc("get_Vessels")
def vessels(w):
    return [objects(w)["vessel"]] if w.loaded else []


@rpc("get_Bodies")
def bodies(w):
    o = objects(w)
    return {"Kerbin": o["body"], "Sun": o["sun"]}


@rpc("Load")
def load(w, name):
    w.load(name)
    return None


@rpc("Quickload")
def quickload(w):
    w.load(w.save)


@rpc("get_RailsWarpFactor")
def rails_warp(w):
    return w.warp_factor


@rpc("set_RailsWarpFactor")
def set_rails_warp(w, factor):
    w.set_warp(int(factor))


@rpc("get_PhysicsWarpFactor")
def physics_warp(w):
    return 0


@rpc("set_PhysicsWarpFactor")
def set_physics_warp(w, factor):
    return None


@rpc("get_WarpRate")
def warp_rate(w):
    return float(w.warp_rate())


@rpc("get_WarpFactor")
def warp_factor(w):
    return float(w.warp_factor)


@rpc("get_MaximumRailsWarpFactor")
def max_rails(w):
    return w.allowed_warp(7)


@rpc("WarpTo")
def warp_to(w, ut, max_rails=100000.0, max_physics=2.0):
    w.advance_to(ut)


@rpc("CanRailsWarpAt")
def can_rails(w, factor=1):
    return w.allowed_warp(factor) == factor


@rpc("get_G")
def grav_const(w):
    return 6.67430e-11


@rpc("TransformPosition")
def transform_position(w, position, frm, to):
    return to.position(frm.position_to_nr(position, w), w)


@rpc("TransformDirection")
def transform_direction(w, direction, frm, to):
    return to.direction(frm.direction_to_nr(direction, w), w)


@rpc("TransformRotation")
def transform_rotation(w, rotation, frm, to):
    return to.rotation_of(frm.rotation_to_nr(tuple(rotation), w), w)


@rpc("TransformVelocity")
def transform_velocity(w, position, velocity, frm, to):
    p_nr = frm.position_to_nr(position, w)
    return to.velocity(p_nr, frm.velocity_to_nr(position, velocity, w), w)


# -- CelestialBody -------------------------------------------------------------

def _is_sun(b):
    return getattr(b, "kind", "") == "sun"


@rpc("CelestialBody_get_Name")
def body_name(w, b):
    return "Sun" if _is_sun(b) else w.body.name


@rpc("CelestialBody_get_ReferenceFrame")
def body_frame(w, b):
    return frame(w, "body")


@rpc("CelestialBody_get_NonRotatingReferenceFrame")
def body_nonrot(w, b):
    return frame(w, "nonrot")


@rpc("CelestialBody_get_EquatorialRadius")
def body_radius(w, b):
    return w.body.radius


@rpc("CelestialBody_get_GravitationalParameter")
def body_mu(w, b):
    return w.body.mu


@rpc("CelestialBody_get_Mass")
def body_mass(w, b):
    return w.body.mu / 6.67430e-11


@rpc("CelestialBody_get_SurfaceGravity")
def body_g(w, b):
    return w.body.surface_gravity


@rpc("CelestialBody_get_RotationalSpeed")
def body_rot_speed(w, b):
    return w.body.omega


@rpc("CelestialBody_get_RotationalPeriod")
def body_rot_period(w, b):
    return w.body.period


@rpc("CelestialBody_get_RotationAngle")
def body_rot_angle(w, b):
    return w.body.phi(w.t) % (2 * math.pi)


@rpc("CelestialBody_get_HasAtmosphere")
def body_has_atm(w, b):
    return True


@rpc("CelestialBody_get_HasAtmosphericOxygen")
def body_has_o2(w, b):
    return True


@rpc("CelestialBody_get_AtmosphereDepth")
def body_atm_depth(w, b):
    return w.body.atmosphere_depth


@rpc("CelestialBody_get_SphereOfInfluence")
def body_soi(w, b):
    return w.body.soi


@rpc("CelestialBody_get_HasSolidSurface")
def body_solid(w, b):
    return True


@rpc("CelestialBody_DensityAt")
def body_density_at(w, b, alt):
    # kRPC's StockAerodynamics.GetDensity: the equator's day-average air
    # (latitude bias at 0, half the sun term, the axial curve at 0) -- not
    # the bare curve, which is 22 K colder at Kerbin's sea level (c0 340
    # against the game's 353).
    atm = w.body.atm
    p = atm.pressure(alt)
    if p <= 0.0:
        return 0.0
    alt_c = min(max(alt, 0.0), atm.depth)
    offset = atm.lat_bias(0.0) + atm.lat_sun_mult(0.0) * 0.5 + atm.axial_mult(0.0)
    return atm.density(p, atm.temperature_curve(alt_c) + atm.sun_mult(alt_c) * offset)


@rpc("CelestialBody_PressureAt")
def body_pressure_at(w, b, alt):
    return w.body.atm.pressure(alt) * 1000.0


@rpc("CelestialBody_TemperatureAt")
def body_temperature_at(w, b, position, ref):
    rb = frame(w, "body").position(ref.position_to_nr(position, w), w)
    return w.body.air(rb, w.t)[2]


@rpc("CelestialBody_AtmosphericDensityAtPosition")
def body_density_at_position(w, b, position, ref):
    rb = frame(w, "body").position(ref.position_to_nr(position, w), w)
    return w.body.air(rb, w.t)[3]


@rpc("CelestialBody_SurfaceHeight")
def body_surface_height(w, b, lat, lon):
    return w.body.terrain.height(lat, lon)


@rpc("CelestialBody_BedrockHeight")
def body_bedrock_height(w, b, lat, lon):
    return w.body.terrain.height(lat, lon)


def _surface_pos(w, lat, lon, alt, ref):
    p_bf = w.body.surface_point(lat, lon, alt)
    return ref.position(frame(w, "body").position_to_nr(p_bf, w), w)


@rpc("CelestialBody_SurfacePosition")
def body_surface_position(w, b, lat, lon, ref):
    return _surface_pos(w, lat, lon, max(0.0, w.body.terrain.height(lat, lon)), ref)


@rpc("CelestialBody_BedrockPosition")
def body_bedrock_position(w, b, lat, lon, ref):
    return _surface_pos(w, lat, lon, w.body.terrain.height(lat, lon), ref)


@rpc("CelestialBody_MSLPosition")
def body_msl_position(w, b, lat, lon, ref):
    return _surface_pos(w, lat, lon, 0.0, ref)


@rpc("CelestialBody_PositionAtAltitude")
def body_position_at_altitude(w, b, lat, lon, alt, ref):
    return _surface_pos(w, lat, lon, alt, ref)


def _bf(w, position, ref):
    return frame(w, "body").position(ref.position_to_nr(position, w), w)


@rpc("CelestialBody_LatitudeAtPosition")
def body_lat_at(w, b, position, ref):
    return w.body.lat_lon(_bf(w, position, ref))[0]


@rpc("CelestialBody_LongitudeAtPosition")
def body_lon_at(w, b, position, ref):
    return w.body.lat_lon(_bf(w, position, ref))[1]


@rpc("CelestialBody_AltitudeAtPosition")
def body_alt_at(w, b, position, ref):
    return vec.norm(_bf(w, position, ref)) - w.body.radius


@rpc("CelestialBody_Position")
def body_position(w, b, ref):
    if _is_sun(b):
        return ref.position(vec.scale(w.body.sun_nr, 13599840256.0), w)
    return ref.position((0.0, 0.0, 0.0), w)


@rpc("CelestialBody_Velocity")
def body_velocity(w, b, ref):
    return ref.velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), w)


@rpc("CelestialBody_Rotation")
def body_rotation(w, b, ref):
    return ref.rotation_of(w.body.q_bf(w.t), w)


@rpc("CelestialBody_Direction")
def body_direction(w, b, ref):
    return ref.direction(quat.rotate(w.body.q_bf(w.t), (0.0, 1.0, 0.0)), w)


@rpc("CelestialBody_AngularVelocity")
def body_angular_velocity(w, b, ref):
    return ref.direction(w.body.spin, w)


# -- Vessel ------------------------------------------------------------------

@rpc("Vessel_get_Name")
def vessel_name(w, v):
    return w.vessel.name


@rpc("Vessel_get_Situation")
def vessel_situation(w, v):
    return SITUATIONS[w.vessel.situation]


@rpc("Vessel_get_Orbit")
def vessel_orbit(w, v):
    return objects(w)["orbit"]


@rpc("Vessel_get_Control")
def vessel_control(w, v):
    return objects(w)["control"]


@rpc("Vessel_get_AutoPilot")
def vessel_autopilot(w, v):
    return objects(w)["autopilot"]


@rpc("Vessel_get_Parts")
def vessel_parts(w, v):
    return objects(w)["parts"]


@rpc("Vessel_get_Resources")
def vessel_resources(w, v):
    return objects(w)["resources"]


@rpc("Vessel_get_ReferenceFrame")
def vessel_frame(w, v):
    return frame(w, "vessel")


@rpc("Vessel_get_OrbitalReferenceFrame")
def vessel_orbital_frame(w, v):
    return frame(w, "orbital")


@rpc("Vessel_get_SurfaceReferenceFrame")
def vessel_surface_frame(w, v):
    return frame(w, "surface")


@rpc("Vessel_get_Mass")
def vessel_mass(w, v):
    return w.vessel.mass


@rpc("Vessel_get_DryMass")
def vessel_dry_mass(w, v):
    return sum(p.dry_mass for p in w.vessel.parts)


@rpc("Vessel_get_MET")
def vessel_met(w, v):
    return w.t


@rpc("Vessel_Flight")
def vessel_flight(w, v, ref=None):
    ref = ref or frame(w, "surface")
    flights = objects(w)["flights"]
    if id(ref) not in flights:
        flights[id(ref)] = Obj(kind="flight", ref=ref)
    return flights[id(ref)]


@rpc("Vessel_Position")
def vessel_position(w, v, ref):
    return ref.position(w.vessel.r, w)


@rpc("Vessel_Velocity")
def vessel_velocity(w, v, ref):
    return ref.velocity(w.vessel.r, w.vessel.v, w)


@rpc("Vessel_Rotation")
def vessel_rotation(w, v, ref):
    return ref.rotation_of(w.vessel.q, w)


@rpc("Vessel_Direction")
def vessel_direction(w, v, ref):
    return ref.direction(quat.rotate(w.vessel.q, (0.0, 1.0, 0.0)), w)


@rpc("Vessel_AngularVelocity")
def vessel_angular_velocity(w, v, ref):
    return ref.angular_velocity_of(w.vessel.w, w)


@rpc("Vessel_BoundingBox")
def vessel_bbox(w, v, ref):
    lo, hi = w.vessel.model["vessel"]["bounding_box"]
    if ref.kind == "vessel":
        return (tuple(vec.sub(lo, w.vessel.com)), tuple(vec.sub(hi, w.vessel.com)))
    pts = [ref.position(frame(w, "vessel").position_to_nr(vec.sub((x, y, z), w.vessel.com), w), w)
           for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    return (tuple(min(p[i] for p in pts) for i in range(3)),
            tuple(max(p[i] for p in pts) for i in range(3)))


@rpc("Vessel_get_MomentOfInertia")
def vessel_moi(w, v):
    return w.vessel.moi()


@rpc("Vessel_get_InertiaTensor")
def vessel_inertia(w, v):
    return [x for row in w.vessel.inertia for x in row]


def _pair(t):
    return (tuple(t), tuple(-x for x in t))


@rpc("Vessel_get_AvailableReactionWheelTorque")
def vessel_rw_torque(w, v):
    return _pair(w.vessel.reaction_wheel_torque())


@rpc("Vessel_get_AvailableRCSTorque")
def vessel_rcs_torque(w, v):
    return _pair(w.vessel.rcs_torque())


@rpc("Vessel_get_AvailableControlSurfaceTorque")
def vessel_cs_torque(w, v):
    pos, neg = w.vessel.surface_torque
    return (tuple(pos), tuple(-x for x in neg))


@rpc("Vessel_get_AvailableEngineTorque")
def vessel_engine_torque(w, v):
    return _pair(w.vessel.engine_gimbal_torque(0.0))


@rpc("Vessel_get_AvailableOtherTorque")
def vessel_other_torque(w, v):
    return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@rpc("Vessel_get_AvailableTorque")
def vessel_available_torque(w, v):
    env = w.vessel.env or w.vessel.environment()
    return _pair(w.vessel.available_torque(env))


def _p_atm(w):
    env = w.vessel.env or w.vessel.environment()
    return env["pressure"] / 101.325


def _limit(e):
    return e.thrust_limit if e.thrust_limit <= 1.0 else e.thrust_limit / 100.0


@rpc("Vessel_get_Thrust")
def vessel_thrust(w, v):
    return sum(e.thrust for e in w.vessel.engines)


@rpc("Vessel_get_AvailableThrust")
def vessel_available_thrust(w, v):
    p = _p_atm(w)
    return sum(_limit(e) * e.max_thrust(p) for e in w.vessel.engines if e.active)


@rpc("Vessel_get_MaxThrust")
def vessel_max_thrust(w, v):
    p = _p_atm(w)
    return sum(e.max_thrust(p) for e in w.vessel.engines if e.active)


@rpc("Vessel_get_MaxVacuumThrust")
def vessel_max_vac_thrust(w, v):
    return sum(e.max_vacuum_thrust for e in w.vessel.engines if e.active)


def _isp(w, p):
    act = [e for e in w.vessel.engines if e.active]
    thrust = sum(e.max_thrust(p) for e in act)
    flow = sum(e.max_thrust(p) / e.isp(p) for e in act if e.isp(p) > 0)
    return thrust / flow if flow > 0 else 0.0


@rpc("Vessel_get_VacuumSpecificImpulse")
def vessel_vac_isp(w, v):
    return _isp(w, 0.0)


@rpc("Vessel_get_SpecificImpulse")
def vessel_isp(w, v):
    return _isp(w, _p_atm(w))


@rpc("Vessel_get_KerbinSeaLevelSpecificImpulse")
def vessel_sl_isp(w, v):
    return _isp(w, 1.0)


# -- Flight --------------------------------------------------------------------

def _env(w):
    return w.vessel.env or w.vessel.environment()


@rpc("Flight_get_MeanAltitude")
def flight_mean_alt(w, f):
    return vec.norm(w.vessel.r) - w.body.radius


@rpc("Flight_get_SurfaceAltitude")
def flight_surface_alt(w, f):
    env = _env(w)
    mean = vec.norm(w.vessel.r) - w.body.radius
    return min(mean - max(env["terrain"], 0.0), mean)


@rpc("Flight_get_BedrockAltitude")
def flight_bedrock_alt(w, f):
    env = _env(w)
    return vec.norm(w.vessel.r) - w.body.radius - env["terrain"]


@rpc("Flight_get_Elevation")
def flight_elevation(w, f):
    return _env(w)["terrain"]


@rpc("Flight_get_Latitude")
def flight_lat(w, f):
    return _env(w)["lat"]


@rpc("Flight_get_Longitude")
def flight_lon(w, f):
    return _env(w)["lon"]


@rpc("Flight_get_Velocity")
def flight_velocity(w, f):
    return f.ref.velocity(w.vessel.r, w.vessel.v, w)


@rpc("Flight_get_Speed")
def flight_speed(w, f):
    return vec.norm(f.ref.velocity(w.vessel.r, w.vessel.v, w))


@rpc("Flight_get_HorizontalSpeed")
def flight_hspeed(w, f):
    vel = f.ref.velocity(w.vessel.r, w.vessel.v, w)
    up = f.ref.direction(vec.unit(w.vessel.r), w)
    vv = vec.dot(vel, up)
    return math.sqrt(max(0.0, vec.dot(vel, vel) - vv * vv))


@rpc("Flight_get_VerticalSpeed")
def flight_vspeed(w, f):
    vel = f.ref.velocity(w.vessel.r, w.vessel.v, w)
    up = f.ref.direction(vec.unit(w.vessel.r), w)
    return vec.dot(vel, up)


@rpc("Flight_get_CenterOfMass")
def flight_com(w, f):
    return f.ref.position(w.vessel.r, w)


@rpc("Flight_get_Rotation")
def flight_rotation(w, f):
    return f.ref.rotation_of(w.vessel.q, w)


@rpc("Flight_get_Direction")
def flight_direction(w, f):
    return f.ref.direction(quat.rotate(w.vessel.q, (0.0, 1.0, 0.0)), w)


def _surface_angles(w):
    """Pitch, heading, roll as KSP's navball shows them."""
    v = w.vessel
    up = vec.unit(v.r)
    north = vec.unit(vec.sub((0.0, 1.0, 0.0), vec.scale(up, up[1])))
    east = vec.cross(up, north)
    nose = quat.rotate(v.q, (0.0, 1.0, 0.0))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, vec.dot(nose, up)))))
    heading = math.degrees(math.atan2(vec.dot(nose, east), vec.dot(nose, north))) % 360.0
    return pitch, heading, up, north, east, nose


@rpc("Flight_get_Pitch")
def flight_pitch(w, f):
    return _surface_angles(w)[0]


@rpc("Flight_get_Heading")
def flight_heading(w, f):
    return _surface_angles(w)[1]


@rpc("Flight_get_Roll")
def flight_roll(w, f):
    pitch, heading, up, north, east, nose = _surface_angles(w)
    roof = quat.rotate(w.vessel.q, (0.0, 0.0, -1.0))
    horiz = vec.unit(vec.cross(nose, up))
    if vec.norm(horiz) == 0:
        return 0.0
    level_roof = vec.unit(vec.cross(horiz, nose))
    ang = math.degrees(math.atan2(vec.dot(vec.cross(level_roof, roof), nose),
                                  vec.dot(level_roof, roof)))
    return ang


@rpc("Flight_get_AtmosphereDensity")
def flight_density(w, f):
    return _env(w)["density"]


@rpc("Flight_get_DynamicPressure")
def flight_q(w, f):
    return _env(w)["q"]


@rpc("Flight_get_StaticPressure")
def flight_static_pressure(w, f):
    return _env(w)["pressure"] * 1000.0


@rpc("Flight_get_StaticPressureAtMSL")
def flight_msl_pressure(w, f):
    return 101325.0


@rpc("Flight_get_SpeedOfSound")
def flight_sound(w, f):
    return _env(w)["sound"]


@rpc("Flight_get_Mach")
def flight_mach(w, f):
    return _env(w)["mach"]


@rpc("Flight_get_TrueAirSpeed")
def flight_tas(w, f):
    return _env(w)["speed"]


@rpc("Flight_get_EquivalentAirSpeed")
def flight_eas(w, f):
    env = _env(w)
    return env["speed"] * math.sqrt(env["density"] / 1.2250) if env["density"] > 0 else 0.0


@rpc("Flight_get_StaticAirTemperature")
def flight_sat(w, f):
    return _env(w)["temperature"]


@rpc("Flight_get_TotalAirTemperature")
def flight_tat(w, f):
    env = _env(w)
    return env["temperature"] * (1 + 0.2 * env["mach"] ** 2)


@rpc("Flight_get_AngleOfAttack")
def flight_aoa(w, f):
    env = _env(w)
    vb = env["v_body"]
    s = vec.norm(vb)
    return math.degrees(math.asin(max(-1.0, min(1.0, vb[2] / s)))) if s > 0 else 0.0


@rpc("Flight_get_SideslipAngle")
def flight_sideslip(w, f):
    env = _env(w)
    vb = env["v_body"]
    s = vec.norm(vb)
    return math.degrees(math.asin(max(-1.0, min(1.0, vb[0] / s)))) if s > 0 else 0.0


def _aero_nr(w):
    return quat.rotate(w.vessel.q, w.vessel.last_aero_body)


@rpc("Flight_get_AerodynamicForce")
def flight_aero_force(w, f):
    return f.ref.direction(_aero_nr(w), w)


def _lift_drag(w):
    fa = _aero_nr(w)
    vel = vec.sub(w.vessel.v, vec.cross(w.body.spin, w.vessel.r))
    if vec.norm(vel) == 0:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    u = vec.unit(vel)
    drag = vec.scale(u, vec.dot(fa, u))
    return vec.sub(fa, drag), drag


@rpc("Flight_get_Lift")
def flight_lift(w, f):
    return f.ref.direction(_lift_drag(w)[0], w)


@rpc("Flight_get_Drag")
def flight_drag(w, f):
    return f.ref.direction(_lift_drag(w)[1], w)


@rpc("Flight_get_GForce")
def flight_gforce(w, f):
    return vec.norm(w.vessel.last_aero_body) / w.vessel.mass / 9.81


def _simulate(w, f, body, position, velocity, rotation=None, angular_velocity=None):
    """The game's aero oracle, answered by the same tables that fly."""
    v = w.vessel
    ref = f.ref
    p_nr = ref.position_to_nr(position, w)
    bfr = frame(w, "body")
    rb = bfr.position(p_nr, w)
    # The velocity argument is relative to the frame it is given in; the
    # airflow is relative to the rotating body.
    v_nr = ref.velocity_to_nr(position, velocity, w)
    vb_air = bfr.velocity(p_nr, v_nr, w)
    q_nr = ref.rotation_to_nr(tuple(rotation), w) if rotation is not None else v.q
    q_bf = bfr.rotation_of(q_nr, w)
    alt, p, temp, rho, sound = w.body.air(rb, w.t)
    vbody = quat.rotate(quat.conj(q_bf), vb_air)
    from kspSim.aero import Aero
    alpha, beta, speed = Aero.angles(vbody)
    env = {"q": 0.5 * rho * speed * speed, "mach": speed / sound if sound > 0 else 0.0,
           "alpha": alpha, "beta": beta, "density": rho, "speed": speed}
    om = None
    if angular_velocity is not None:
        w_nr = ref.direction_to_nr(angular_velocity, w)
        om = quat.rotate(quat.conj(q_nr), w_nr)
    fb, tb = v.aero_wrench(env, omega_rel_body=om)
    return (ref.direction(quat.rotate(q_nr, fb), w), ref.direction(quat.rotate(q_nr, tb), w))


@rpc("Flight_SimulateAerodynamicForceAt")
def flight_simulate_force(w, f, body, position, velocity, rotation=None):
    return _simulate(w, f, body, position, velocity, rotation)[0]


@rpc("Flight_SimulateAerodynamicWrenchAt")
def flight_simulate_wrench(w, f, body, position, velocity, rotation=None,
                           angular_velocity=None, ut=None):
    return _simulate(w, f, body, position, velocity, rotation, angular_velocity)


# -- Orbit ---------------------------------------------------------------------

@rpc("Orbit_get_Body")
def orbit_body(w, o):
    return objects(w)["body"]


def _el(w):
    return w.vessel.elements()


@rpc("Orbit_get_Apoapsis")
def orbit_apoapsis(w, o):
    return _el(w)["apoapsis"]


@rpc("Orbit_get_Periapsis")
def orbit_periapsis(w, o):
    return _el(w)["periapsis"]


@rpc("Orbit_get_ApoapsisAltitude")
def orbit_apo_alt(w, o):
    return _el(w)["apoapsis"] - w.body.radius


@rpc("Orbit_get_PeriapsisAltitude")
def orbit_peri_alt(w, o):
    return _el(w)["periapsis"] - w.body.radius


@rpc("Orbit_get_SemiMajorAxis")
def orbit_sma(w, o):
    return _el(w)["a"]


@rpc("Orbit_get_Eccentricity")
def orbit_ecc(w, o):
    return _el(w)["e"]


@rpc("Orbit_get_Inclination")
def orbit_inc(w, o):
    h = _el(w)["h"]
    return math.acos(max(-1.0, min(1.0, -h[1] / vec.norm(h))))


@rpc("Orbit_get_Speed")
def orbit_speed(w, o):
    return vec.norm(w.vessel.v)


@rpc("Orbit_get_OrbitalSpeed")
def orbit_orbital_speed(w, o):
    return vec.norm(w.vessel.v)


@rpc("Orbit_get_Radius")
def orbit_radius(w, o):
    return vec.norm(w.vessel.r)


@rpc("Orbit_get_Period")
def orbit_period(w, o):
    a = _el(w)["a"]
    return 2 * math.pi * math.sqrt(a ** 3 / w.body.mu) if a > 0 else float("inf")


def _time_to(w, target_anomaly):
    el = _el(w)
    mu = w.body.mu
    a, e = el["a"], el["e"]
    if e >= 1 or a <= 0:
        return float("inf")
    r = w.vessel.r
    ev = el["evec"]
    rn = vec.norm(r)
    cos_nu = vec.dot(ev, r) / (e * rn) if e > 1e-9 else 1.0
    nu = math.acos(max(-1.0, min(1.0, cos_nu)))
    if vec.dot(r, w.vessel.v) < 0:
        nu = 2 * math.pi - nu
    E = 2 * math.atan2(math.sqrt(1 - e) * math.sin(nu / 2), math.sqrt(1 + e) * math.cos(nu / 2))
    M = E - e * math.sin(E)
    n = math.sqrt(mu / a ** 3)
    Mt = target_anomaly
    return ((Mt - M) % (2 * math.pi)) / n


@rpc("Orbit_get_TimeToApoapsis")
def orbit_tta(w, o):
    return _time_to(w, math.pi)


@rpc("Orbit_get_TimeToPeriapsis")
def orbit_ttp(w, o):
    return _time_to(w, 0.0)


# -- Control -------------------------------------------------------------------

def _ctl_prop(name, cast=float):
    def getter(w, c):
        return getattr(c, name)

    def setter(w, c, value):
        setattr(c, name, cast(value))
    return getter, setter


for _name, _attr, _cast in (("Throttle", "throttle", float), ("Pitch", "pitch", float),
                            ("Yaw", "yaw", float), ("Roll", "roll", float),
                            ("Forward", "forward", float), ("Up", "up", float),
                            ("Right", "right", float), ("WheelThrottle", "wheel_throttle", float),
                            ("WheelSteering", "wheel_steering", float), ("SAS", "sas", bool),
                            ("RCS", "rcs", bool), ("Gear", "gear", bool), ("Lights", "lights", bool),
                            ("Abort", "abort", bool)):
    g, s = _ctl_prop(_attr, _cast)
    HANDLERS[("SpaceCenter", "Control_get_" + _name)] = g
    HANDLERS[("SpaceCenter", "Control_set_" + _name)] = s


@rpc("Control_get_Brakes")
def control_brakes(w, c):
    return c.brakes > 0


@rpc("Control_set_Brakes")
def control_set_brakes(w, c, value):
    c.brakes = 1.0 if value else 0.0
    w.vessel.set_brakes(bool(value))


@rpc("Control_GetActionGroup")
def control_get_ag(w, c, group):
    return c.action_groups[group % 10]


@rpc("Control_SetActionGroup")
def control_set_ag(w, c, group, state):
    c.action_groups[group % 10] = bool(state)
    w.vessel.fire_group(group % 10, bool(state))


@rpc("Control_ToggleActionGroup")
def control_toggle_ag(w, c, group):
    c.action_groups[group % 10] = not c.action_groups[group % 10]
    w.vessel.fire_group(group % 10, c.action_groups[group % 10])


@rpc("Control_get_SASMode")
def control_sas_mode(w, c):
    return 0


@rpc("Control_set_SASMode")
def control_set_sas_mode(w, c, mode):
    return None


@rpc("Control_get_CurrentStage")
def control_stage(w, c):
    return 0


# -- AutoPilot ---------------------------------------------------------------

def _ap(w):
    return w.vessel.autopilot


@rpc("AutoPilot_Engage")
def ap_engage(w, a):
    v = w.vessel
    _ap(w).engage(w.t, v.available_torque(_env(w)), v.moi())


@rpc("AutoPilot_Disengage")
def ap_disengage(w, a):
    _ap(w).disengage()


@rpc("AutoPilot_get_Engaged")
def ap_engaged(w, a):
    return _ap(w).engaged


@rpc("AutoPilot_set_Engaged")
def ap_set_engaged(w, a, value):
    if value:
        ap_engage(w, a)
    else:
        ap_disengage(w, a)


@rpc("AutoPilot_Wait")
def ap_wait(w, a):
    return None


@rpc("AutoPilot_get_ReferenceFrame")
def ap_frame(w, a):
    return w.vessel.autopilot_frame or frame(w, "surface")


@rpc("AutoPilot_set_ReferenceFrame")
def ap_set_frame(w, a, ref):
    w.vessel.autopilot_frame = ref


@rpc("AutoPilot_get_TargetDirection")
def ap_target_direction(w, a):
    return _ap(w).target_direction()


@rpc("AutoPilot_set_TargetDirection")
def ap_set_target_direction(w, a, d):
    _ap(w).set_target_direction(tuple(d))


@rpc("AutoPilot_SetDirectionAndUp")
def ap_direction_and_up(w, a, direction, up, roll=0.0):
    _ap(w).set_direction_and_up(tuple(direction), tuple(up), roll)


@rpc("AutoPilot_get_TargetRoll")
def ap_target_roll(w, a):
    return _ap(w).target_roll()


@rpc("AutoPilot_set_TargetRoll")
def ap_set_target_roll(w, a, value):
    _ap(w).set_target_roll(value)


@rpc("AutoPilot_get_TargetRotation")
def ap_target_rotation(w, a):
    return _ap(w).target_rotation


@rpc("AutoPilot_set_TargetRotation")
def ap_set_target_rotation(w, a, value):
    _ap(w).set_target_rotation(tuple(value))


def _ap_vec(attr, setter=None):
    def g(w, a):
        return tuple(getattr(_ap(w), attr))

    def s(w, a, value):
        if setter:
            getattr(_ap(w), setter)(tuple(value))
        else:
            setattr(_ap(w), attr, tuple(value))
    return g, s


for _name, _attr, _setter in (("TimeToPeak", "time_to_peak", "set_time_to_peak"),
                              ("Overshoot", "overshoot", "set_overshoot"),
                              ("MaxAngularVelocity", "max_angular_velocity", None)):
    g, s = _ap_vec(_attr, _setter)
    HANDLERS[("SpaceCenter", "AutoPilot_get_" + _name)] = g
    HANDLERS[("SpaceCenter", "AutoPilot_set_" + _name)] = s


@rpc("AutoPilot_get_Error")
def ap_error(w, a):
    ap = _ap(w)
    frm = w.vessel.autopilot_frame or frame(w, "body")
    nose = quat.rotate(frm.rotation_of(w.vessel.q, w), (0.0, 1.0, 0.0))
    return vec.angle_deg(nose, ap.target_direction())


@rpc("AutoPilot_get_AutoTune")
def ap_autotune(w, a):
    return _ap(w).auto_tune


@rpc("AutoPilot_set_AutoTune")
def ap_set_autotune(w, a, value):
    _ap(w).auto_tune = bool(value)


@rpc("AutoPilot_get_TargetSmoothingTime")
def ap_smoothing(w, a):
    return _ap(w).target_smoothing


@rpc("AutoPilot_set_TargetSmoothingTime")
def ap_set_smoothing(w, a, value):
    _ap(w).target_smoothing = max(0.0, value)


@rpc("AutoPilot_get_RollThreshold")
def ap_roll_threshold(w, a):
    return _ap(w).roll_start


@rpc("AutoPilot_set_RollThreshold")
def ap_set_roll_threshold(w, a, value):
    _ap(w).roll_start = value


@rpc("AutoPilot_get_RollStartAngle")
def ap_roll_start_angle(w, a):
    return _ap(w).roll_start


@rpc("AutoPilot_set_RollStartAngle")
def ap_set_roll_start_angle(w, a, value):
    _ap(w).roll_start = value


@rpc("AutoPilot_get_RollEngageAngle")
def ap_roll_engage_angle(w, a):
    return _ap(w).roll_engage


@rpc("AutoPilot_set_RollEngageAngle")
def ap_set_roll_engage_angle(w, a, value):
    _ap(w).roll_engage = value


# -- Parts ---------------------------------------------------------------------

@rpc("Parts_get_All")
def parts_all(w, p):
    return [x for x in w.vessel.parts if x.alive]


@rpc("Parts_get_Root")
def parts_root(w, p):
    return w.vessel.parts[w.vessel.model["vessel"].get("root") or 0]


@rpc("Parts_get_Controlling")
def parts_controlling(w, p):
    return w.vessel.parts[w.vessel.model["vessel"].get("controlling") or 0]


@rpc("Parts_get_Engines")
def parts_engines(w, p):
    return objects(w)["engines"]


@rpc("Parts_get_Wheels")
def parts_wheels(w, p):
    return objects(w)["wheels"]


@rpc("Parts_get_ReactionWheels")
def parts_rws(w, p):
    return objects(w)["reaction_wheels"]


@rpc("Parts_get_ControlSurfaces")
def parts_cs(w, p):
    return objects(w)["control_surfaces"]


@rpc("Parts_get_RCS")
def parts_rcs(w, p):
    return objects(w)["rcs"]


@rpc("Parts_WithTitle")
def parts_with_title(w, p, title):
    return [x for x in w.vessel.parts if x.title == title]


@rpc("Parts_WithName")
def parts_with_name(w, p, name):
    return [x for x in w.vessel.parts if x.name == name]


@rpc("Parts_WithModule")
def parts_with_module(w, p, name):
    return [x for x in w.vessel.parts if any(m.name == name for m in x.modules)]


@rpc("Parts_ModulesWithName")
def parts_modules_with_name(w, p, name):
    return [m for x in w.vessel.parts for m in x.modules if m.name == name]


@rpc("Part_get_Name")
def part_name(w, p):
    return p.name


@rpc("Part_get_Title")
def part_title(w, p):
    return p.title


@rpc("Part_get_Tag")
def part_tag(w, p):
    return ""


@rpc("Part_get_Mass")
def part_mass(w, p):
    return p.mass()


@rpc("Part_get_DryMass")
def part_dry_mass(w, p):
    return p.dry_mass


@rpc("Part_get_Modules")
def part_modules(w, p):
    return p.modules


@rpc("Part_get_MaxSkinTemperature")
def part_max_skin(w, p):
    return p.rec["max_skin_temperature"]


@rpc("Part_get_MaxTemperature")
def part_max_temp(w, p):
    return p.rec["max_temperature"]


@rpc("Part_get_SkinTemperature")
def part_skin(w, p):
    return p.skin_temperature


@rpc("Part_get_Temperature")
def part_temp(w, p):
    return p.temperature


@rpc("Part_get_ImpactTolerance")
def part_impact(w, p):
    return p.rec.get("impact_tolerance", 0.0)


@rpc("Part_get_Shielded")
def part_shielded(w, p):
    return p.rec.get("shielded", False)


@rpc("Part_get_Parent")
def part_parent(w, p):
    i = p.rec.get("parent")
    return w.vessel.parts[i] if i is not None else None


@rpc("Part_get_Children")
def part_children(w, p):
    return [x for x in w.vessel.parts if x.rec.get("parent") == p.index]


@rpc("Part_get_Stage")
def part_stage(w, p):
    return p.rec.get("stage", -1)


@rpc("Part_get_DecoupleStage")
def part_decouple_stage(w, p):
    return p.rec.get("decouple_stage", -1)


@rpc("Part_get_Vessel")
def part_vessel(w, p):
    return objects(w)["vessel"]


@rpc("Part_get_ReferenceFrame")
def part_frame(w, p):
    return frame(w, "part", p)


@rpc("Part_Position")
def part_position(w, p, ref):
    off = quat.rotate(w.vessel.q, vec.sub(p.position, w.vessel.com))
    return ref.position(vec.add(w.vessel.r, off), w)


@rpc("Part_CenterOfMass")
def part_com(w, p, ref):
    off = quat.rotate(w.vessel.q, vec.sub(p.com, w.vessel.com))
    return ref.position(vec.add(w.vessel.r, off), w)


@rpc("Part_Direction")
def part_direction(w, p, ref):
    return ref.direction(quat.rotate(w.vessel.q, p.direction), w)


@rpc("Part_Rotation")
def part_rotation(w, p, ref):
    return ref.rotation_of(quat.mul(w.vessel.q, p.rotation), w)


@rpc("Part_BoundingBox")
def part_bbox(w, p, ref):
    lo, hi = p.rec["bounding_box"]
    if ref.kind == "vessel":
        return (tuple(vec.sub(lo, w.vessel.com)), tuple(vec.sub(hi, w.vessel.com)))
    pts = [ref.position(frame(w, "vessel").position_to_nr(vec.sub((x, y, z), w.vessel.com), w), w)
           for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    return (tuple(min(q[i] for q in pts) for i in range(3)),
            tuple(max(q[i] for q in pts) for i in range(3)))


@rpc("Part_get_Resources")
def part_resources(w, p):
    cache = objects(w)["part_resources"]
    if p.index not in cache:
        cache[p.index] = Obj(kind="resources", part=p)
    return cache[p.index]


def _part_module(p, cls, kind):
    for o in objects(p.vessel.world)[kind]:
        if o.part is p:
            return o
    return None


@rpc("Part_get_Engine")
def part_engine(w, p):
    return next((e for e in w.vessel.engines if e.part is p), None)


@rpc("Part_get_Wheel")
def part_wheel(w, p):
    return _part_module(p, None, "wheels")


@rpc("Part_get_ControlSurface")
def part_cs(w, p):
    return _part_module(p, None, "control_surfaces")


@rpc("Part_get_ReactionWheel")
def part_rw(w, p):
    return _part_module(p, None, "reaction_wheels")


@rpc("Part_get_RCS")
def part_rcs(w, p):
    return _part_module(p, None, "rcs")


# -- Modules (fields and actions) ----------------------------------------------

@rpc("Module_get_Name")
def module_name(w, m):
    return m.name


@rpc("Module_get_Part")
def module_part(w, m):
    return m.part


@rpc("Module_get_Fields")
def module_fields(w, m):
    return dict(m.fields)


@rpc("Module_get_Actions")
def module_actions(w, m):
    return list(m.actions)


@rpc("Module_get_Events")
def module_events(w, m):
    return list(m.events)


@rpc("Module_HasField")
def module_has_field(w, m, name):
    return name in m.fields


@rpc("Module_GetField")
def module_get_field(w, m, name):
    if name not in m.fields:
        raise ValueError("no field %r" % name)
    return str(m.fields[name])


@rpc("Module_HasAction")
def module_has_action(w, m, name):
    return name in m.actions


@rpc("Module_HasEvent")
def module_has_event(w, m, name):
    return name in m.events


def _set_field(w, m, name, value):
    m.fields[name] = value
    if m.name == "ModuleResourceDrain" and name == "Drain rate":
        w.vessel.control.drain_rate = float(value)


for _kind, _cast in (("Int", int), ("Float", float), ("String", str), ("Bool", bool)):
    def _mk(cast):
        def fn(w, m, name, value):
            _set_field(w, m, name, str(cast(value)))
        return fn
    HANDLERS[("SpaceCenter", "Module_SetField" + _kind)] = _mk(_cast)


@rpc("Module_SetAction")
def module_set_action(w, m, name, value=True):
    """Only the actions a flight has been seen to use do anything."""
    v = w.vessel
    if m.name == "ModuleResourceDrain":
        if name in ("Drain",) and value:
            v.drain_active = True
            m.fields["Drain"] = "True"
        elif name == "Stop Draining":
            v.drain_active = False
            m.fields["Drain"] = "False"
        elif name == "Toggle Draining":
            v.drain_active = not v.drain_active
    elif name == "Lock Gimbal":
        for e in v.engines:
            if e.part is m.part:
                e.gimbal_locked = True
    elif name == "Free Gimbal":
        for e in v.engines:
            if e.part is m.part:
                e.gimbal_locked = False


@rpc("Module_TriggerEvent")
def module_trigger_event(w, m, name):
    module_set_action(w, m, name, True)


# -- Resources -----------------------------------------------------------------

def _res_scope(w, r):
    parts = [r.part] if r.part is not None else w.vessel.parts
    return [x for p in parts for x in p.resources]


@rpc("Resources_Amount")
def resources_amount(w, r, name):
    return sum(x.amount for x in _res_scope(w, r) if x.name == name)


@rpc("Resources_Max")
def resources_max(w, r, name):
    return sum(x.max for x in _res_scope(w, r) if x.name == name)


@rpc("Resources_HasResource")
def resources_has(w, r, name):
    return any(x.name == name for x in _res_scope(w, r))


@rpc("Resources_get_Names")
def resources_names(w, r):
    out = []
    for x in _res_scope(w, r):
        if x.name not in out:
            out.append(x.name)
    return out


@rpc("Resources_static_Density", )
def resources_density(w, name):
    return w.vessel._density(name)


@rpc("Resources_get_Enabled")
def resources_enabled(w, r):
    return True


# -- Engines ---------------------------------------------------------------------

@rpc("Engine_get_Part")
def engine_part(w, e):
    return e.part


@rpc("Engine_get_Active")
def engine_active(w, e):
    return e.active


@rpc("Engine_set_Active")
def engine_set_active(w, e, value):
    e.active = bool(value)


@rpc("Engine_get_ThrustLimit")
def engine_thrust_limit(w, e):
    return e.thrust_limit


@rpc("Engine_set_ThrustLimit")
def engine_set_thrust_limit(w, e, value):
    e.thrust_limit = max(0.0, min(1.0, value))


@rpc("Engine_get_Thrust")
def engine_thrust(w, e):
    return e.thrust


@rpc("Engine_get_AvailableThrust")
def engine_available_thrust(w, e):
    return _limit(e) * e.max_thrust(_p_atm(w)) if e.active else 0.0


@rpc("Engine_get_MaxThrust")
def engine_max_thrust(w, e):
    return e.max_thrust(_p_atm(w))


@rpc("Engine_get_MaxVacuumThrust")
def engine_max_vac_thrust(w, e):
    return e.max_vacuum_thrust


@rpc("Engine_get_SpecificImpulse")
def engine_isp(w, e):
    return e.isp(_p_atm(w))


@rpc("Engine_get_VacuumSpecificImpulse")
def engine_vac_isp(w, e):
    return e.isp(0.0)


@rpc("Engine_get_KerbinSeaLevelSpecificImpulse")
def engine_sl_isp(w, e):
    return e.isp(1.0)


@rpc("Engine_SpecificImpulseAt")
def engine_isp_at(w, e, pressure):
    return e.isp(pressure)


@rpc("Engine_MaxThrustAt")
def engine_max_thrust_at(w, e, pressure):
    return e.max_thrust(pressure)


@rpc("Engine_AvailableThrustAt")
def engine_available_thrust_at(w, e, pressure):
    return _limit(e) * e.max_thrust(pressure) if e.active else 0.0


@rpc("Engine_get_PropellantNames")
def engine_propellant_names(w, e):
    return [p["name"] for p in e.propellants]


@rpc("Engine_get_PropellantRatios")
def engine_propellant_ratios(w, e):
    return {p["name"]: p["ratio"] for p in e.propellants}


@rpc("Engine_get_HasFuel")
def engine_has_fuel(w, e):
    return all(w.vessel.resource_amount(p["name"]) > 0 for p in e.propellants)


@rpc("Engine_get_Flameout")
def engine_flameout(w, e):
    return e.flameout


@rpc("Engine_get_Throttle")
def engine_throttle(w, e):
    return w.vessel.control.throttle


@rpc("Engine_get_ThrottleLocked")
def engine_throttle_locked(w, e):
    return e.rec.get("throttle_locked", False)


@rpc("Engine_get_Gimballed")
def engine_gimballed(w, e):
    return e.rec.get("gimballed", False)


@rpc("Engine_get_GimbalRange")
def engine_gimbal_range(w, e):
    return e.gimbal_range


@rpc("Engine_get_GimbalLocked")
def engine_gimbal_locked(w, e):
    return e.gimbal_locked


@rpc("Engine_set_GimbalLocked")
def engine_set_gimbal_locked(w, e, value):
    e.gimbal_locked = bool(value)


# -- Wheels, reaction wheels, control surfaces, RCS -----------------------------

@rpc("Wheel_get_Part")
def wheel_part(w, x):
    return x.part


@rpc("Wheel_get_HasBrakes")
def wheel_has_brakes(w, x):
    return x.rec["has_brakes"]


@rpc("Wheel_get_Brakes")
def wheel_brakes(w, x):
    return 100.0 * w.vessel.control.brakes


@rpc("Wheel_set_Brakes")
def wheel_set_brakes(w, x, value):
    # Brake torque as a percentage; the ground model reads the vessel's
    # brake fraction, which every autopilot here sets on all wheels at once.
    w.vessel.control.brakes = max(0.0, min(1.0, value / 100.0)) if value > 1.0 else \
        max(0.0, min(1.0, value))


@rpc("Wheel_get_Steerable")
def wheel_steerable(w, x):
    return x.rec["steerable"]


@rpc("Wheel_get_SteeringEnabled")
def wheel_steering_enabled(w, x):
    return x.rec["steerable"]


@rpc("Wheel_get_Deployable")
def wheel_deployable(w, x):
    return x.rec["deployable"]


@rpc("Wheel_get_Deployed")
def wheel_deployed(w, x):
    return w.vessel.gear_deploy > 0.99


@rpc("Wheel_set_Deployed")
def wheel_set_deployed(w, x, value):
    w.vessel.control.gear = bool(value)


@rpc("Wheel_get_Radius")
def wheel_radius(w, x):
    return x.rec["radius"]


@rpc("Wheel_get_Grounded")
def wheel_grounded(w, x):
    return w.vessel.contact


@rpc("Wheel_get_Broken")
def wheel_broken(w, x):
    return False


@rpc("ReactionWheel_get_Part")
def rw_part(w, x):
    return x.part


@rpc("ReactionWheel_get_Active")
def rw_active(w, x):
    return x.rec.get("active", True)


@rpc("ReactionWheel_set_Active")
def rw_set_active(w, x, value):
    x.rec["active"] = bool(value)


@rpc("ReactionWheel_get_Broken")
def rw_broken(w, x):
    return False


@rpc("ReactionWheel_get_MaxTorque")
def rw_max_torque(w, x):
    return tuple(tuple(v) for v in x.rec["max_torque"])


@rpc("ReactionWheel_get_AvailableTorque")
def rw_available_torque(w, x):
    return tuple(tuple(v) for v in x.rec["max_torque"])


@rpc("ReactionWheel_get_AuthorityLimiter")
def rw_authority(w, x):
    return x.rec.get("authority_limiter", 100.0)


for _name, _key in (("PitchEnabled", "pitch"), ("YawEnabled", "yaw"), ("RollEnabled", "roll"),
                    ("AuthorityLimiter", "authority_limiter"), ("Inverted", "inverted"),
                    ("Deployed", "deployed"), ("SurfaceArea", "surface_area")):
    def _cs_get(key):
        return lambda w, x: x.rec[key]
    HANDLERS[("SpaceCenter", "ControlSurface_get_" + _name)] = _cs_get(_key)


@rpc("ControlSurface_get_Part")
def cs_part(w, x):
    return x.part


@rpc("ControlSurface_get_AvailableTorque")
def cs_available_torque(w, x):
    n = max(1, len(w.vessel.control_surfaces))
    pos, neg = w.vessel.surface_torque
    return (tuple(p / n for p in pos), tuple(-q / n for q in neg))


@rpc("RCS_get_Part")
def rcs_part(w, x):
    return x.part


@rpc("RCS_get_Enabled")
def rcs_enabled(w, x):
    return x.rec.get("enabled", True)


@rpc("RCS_set_Enabled")
def rcs_set_enabled(w, x, value):
    x.rec["enabled"] = bool(value)


@rpc("RCS_get_Active")
def rcs_active(w, x):
    return w.vessel.control.rcs and x.rec.get("enabled", True)


# -- UI: accepted and ignored ------------------------------------------------------

class UIObj:
    __sim_object__ = True

    def __init__(self, kind):
        self.kind = kind
        self.props = {}


_CANVAS = UIObj("Canvas")


def _ui_default(typ):
    from krpc.types import ClassType, TupleType, ListType, ValueType, EnumerationType
    import krpc.schema.KRPC_pb2 as KRPC
    if typ is None:
        return None
    if isinstance(typ, ClassType):
        return UIObj(typ.protobuf_type.name)
    if isinstance(typ, TupleType):
        return tuple(_ui_default(t) for t in typ.value_types)
    if isinstance(typ, ListType):
        return []
    if isinstance(typ, EnumerationType):
        return 0
    code = typ.protobuf_type.code
    if code == KRPC.Type.BOOL:
        return False
    if code == KRPC.Type.STRING:
        return ""
    if code in (KRPC.Type.DOUBLE, KRPC.Type.FLOAT):
        return 0.0
    return 0


def install_ui(signatures):
    """The in-game panels: accepted and remembered, never drawn.  Getters
    return what was set, else a default of the declared type; ``Add*``
    makes a new element; buttons are never clicked."""
    for (svc, proc), (params, _, ret) in signatures.procs.items():
        if svc != "UI" or (svc, proc) in HANDLERS:
            continue
        if proc in ("get_StockCanvas", "AddCanvas"):
            HANDLERS[(svc, proc)] = lambda w, *a: _CANVAS
            continue
        cls, _, member = proc.partition("_")
        if cls in ("get", "set") or not member:
            HANDLERS[(svc, proc)] = (lambda r: lambda w, *a: _ui_default(r))(ret)
            continue

        def make(member, ret):
            def fn(w, this=None, *args):
                if member.startswith("set_"):
                    if this is not None and args:
                        this.props[member[4:]] = args[0]
                    return None
                if member.startswith("get_") and this is not None and member[4:] in this.props:
                    return this.props[member[4:]]
                if member == "get_Size":
                    return (1920.0, 1080.0)
                return _ui_default(ret)
            return fn
        HANDLERS[(svc, proc)] = make(member, ret)
