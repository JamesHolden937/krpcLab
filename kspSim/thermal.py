"""Part temperatures and burning up: KSP's FlightIntegrator thermal model.

A port of the game's own model, read from the decompiled classes (cited by
name below; nothing copied), so a craft needs no thermal fitting:

- **Areas from the drag cube** (``DragCubeList``).  Each face's area is cut
  by the neighbours stacked on its attach nodes (``SetPartOcclusion``); the
  cut area radiates (``PostOcclusionArea``); the area the flow convects onto
  is the cube's ``ExposedArea`` for the current flow direction and Mach
  (``AddSurfaceDragDirection``), lerped from the whole radiative area below
  Mach 0.8 to it by Mach 1.5 (``SetSkinProperties``).
- **Supersonic occlusion** (``UpdateOcclusionConvection``, ``OcclusionData``,
  ``OcclusionCone``): parts sorted along the flow, each casting a shock cone
  that cuts the convective area, the post-shock temperature and the
  coefficient of the parts behind it.  The Sun and the planet are occluded
  the same way with cylinders.
- **Two skins per part**: the exposed fraction (the one kRPC reports as
  ``skin_temperature``) and the rest, each with its share of the skin's
  thermal mass, exchanging heat with each other and with the interior.
- **Conduction** between linked parts (parent and children), internal to
  internal and skin to skin, over the attach nodes' contact areas, each
  exchange limited so a pair cannot overshoot its common temperature
  (``UpdateConduction``).
- **Convection** at KSP's coefficient and shock temperature, turbulence from
  the pseudo-Reynolds number times the part's drag coefficient, the
  coefficient capped at the exposed skin's thermal mass per second
  (``PrecalcConvection``); **radiation** to a background that lerps from the
  air to space with density, sunlight and the planet's own and reflected
  flux absorbed (``PrecalcRadiation``, ``CelestialBody.GetAtmoThermalStats``).
- One explicit pass per step with KSP's convergence factor
  (``ThermalIntegrationPass``).

A part whose exposed skin passes ``skinMaxTemp`` or whose interior passes
``maxTemp`` fails; the root part's loss is the vessel's.
"""
import math

from kspSim import quat, vec
from kspSim.curves import FloatCurve

SIGMA = 5.670374419e-8
FACES = ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
         (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0))
# CelestialBody's defaults, which stock Kerbin keeps.
BODY_ALBEDO = 0.35
BODY_EMISSIVITY = 0.65
# OcclusionCone's statics.
DETACHED_HEAT = 0.5
DETACHED_COEFF = 1.0
BEHIND_DETACHED_HEAT = 0.4
BEHIND_DETACHED_COEFF = 1.0
DETACHED_MACH_ANGLE = 0.05
DETACHED_START = math.pi * 49.0 / 100.0
DETACHED_END = math.pi * 9.0 / 25.0
OBLIQUE_ANGLE = 0.8
OBLIQUE_PART_ANGLE = 0.25
OBLIQUE_MIN_ANGLE = 1.05
OBLIQUE_CONE_HEAT = 0.75
OBLIQUE_CONE_COEFF = 1.0
OBLIQUE_CYL_HEAT = 0.55
OBLIQUE_CYL_COEFF = 1.0
SUN_EVERY = 10          # thermal steps between Sun and planet occlusion updates
# Which contact areas a freshly loaded vessel's thermal links carry (see
# Thermal._contact): "fresh" every link 0.01 m^2, "real" the nodes' areas,
# "root" the nodes' areas except the root part's own nodes and parts
# surface-attached to it, which read 0.01.
LINK_AREAS = "root"
# KSP holds the planet still in the world below this height (inverse
# rotation) -- Kerbin's inverseRotThresholdAltitude.
INVERSE_ROTATION_ALT = 100000.0


def _f(d, key, default):
    try:
        return float(d.get(key, default))
    except (TypeError, ValueError):
        return default


def _clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def _lerp(a, b, t):
    return a + (b - a) * _clamp01(t)


class Physics:
    """Physics.cfg's constants and drag curves."""

    def __init__(self, physics, heat_scale=1.2):
        s = (physics or {}).get("scalars") or {}
        g = lambda k, d: _f(s, k, d)  # noqa: E731
        self.cp = g("standardSpecificHeatCapacity", 800.0)
        self.space_t = g("spaceTemperature", 4.0)
        self.newton_base = g("newtonianConvectionFactorBase", 8.14)
        self.newton_total = g("newtonianConvectionFactorTotal", 4.0)
        self.newton_rho_exp = g("newtonianDensityExponent", 0.5)
        self.newton_v_exp = g("newtonianVelocityExponent", 1.0)
        self.lerp_start = g("newtonianMachTempLerpStartMach", 2.0)
        self.lerp_end = g("newtonianMachTempLerpEndMach", 5.0)
        self.lerp_exp = g("newtonianMachTempLerpExponent", 3.0)
        self.mach_factor = g("machConvectionFactor", 7.0)
        self.mach_rho_exp = g("machConvectionDensityExponent", 0.5)
        self.mach_v_exp = g("machConvectionVelocityExponent", 3.0)
        self.mach_t_scalar = g("machTemperatureScalar", 21.0)
        self.mach_t_exp = g("machTemperatureVelocityExponent", 0.75)
        self.newton_t_factor = g("newtonianTemperatureFactor", 1.0)
        self.turb_start = g("turbulentConvectionStart", 100.0)
        self.turb_end = g("turbulentConvectionEnd", 200.0)
        self.turb_mult = g("turbulentConvectionMult", 50.0)
        self.full_area_lo = g("fullToCrossSectionLerpStart", 0.8)
        self.full_area_hi = g("fullToCrossSectionLerpEnd", 1.5)
        self.full_area_min = g("fullConvectionAreaMin", 0.2)
        self.skin_internal = g("skinInternalConductionFactor", 0.005)
        self.skin_skin = g("skinSkinConductionFactor", 0.003)
        self.shielded_conduction = g("shieldedConductionFactor", 0.01)
        self.conduction = g("conductionFactor", 120.0)
        self.convergence = g("thermalConvergenceFactor", 0.63)
        self.radiation = g("radiationFactor", 1.0)
        self.solar_home = g("solarLuminosityAtHome", 1360.0)
        self.insolation = g("solarInsolationAtHome", 0.15)
        self.heat_scale = heat_scale
        c = (physics or {}).get("curves") or {}

        def curve(name, default):
            return FloatCurve(c[name]) if name in c else FloatCurve([(0.0, default)])
        self.tip = curve("DRAG_TIP", 1.0)
        self.surf = curve("DRAG_SURFACE", 0.02)
        self.tail = curve("DRAG_TAIL", 1.0)
        self.mult = curve("DRAG_MULTIPLIER", 1.0)
        self.cd = curve("DRAG_CD", 1.0)
        self.cd_power = curve("DRAG_CD_POWER", 1.0)

    def mach_lerp(self, mach):
        t = (mach - self.lerp_start) / (self.lerp_end - self.lerp_start)
        return _clamp01(t) ** self.lerp_exp

    def external_temperature(self, speed, mach, ambient):
        """FlightIntegrator.CalculateShockTemperature, floored at the air's:
        ReentryHeatScale multiplies the temperature, not the coefficient."""
        t = self.mach_lerp(mach)
        shock = speed * self.newton_t_factor
        if t > 0.0:
            shock += (self.mach_t_scalar * speed ** self.mach_t_exp - shock) * t
        return max(ambient, shock * self.heat_scale)

    def convective_coefficient(self, density, speed, mach):
        """CalculateConvectiveCoefficient (W/m^2/K; the part's final
        coefficient takes its area and 0.001)."""
        if density <= 0.0 or speed <= 0.0:
            return 0.0
        t = self.mach_lerp(mach)
        rn = density ** self.newton_rho_exp if density <= 1.0 else density
        newton = rn * (self.newton_base + speed ** self.newton_v_exp) * self.newton_total
        if t == 0.0:
            return newton
        rm = density ** self.mach_rho_exp if density <= 1.0 else density
        mach_c = 1e-7 * self.mach_factor * rm * speed ** self.mach_v_exp
        return newton + (mach_c - newton) * t

    def density_thermal_lerp(self, density, mach, gamma=1.4):
        """CalculateDensityThermalLerp: 1 in vacuum, falling with the
        (post-shock above Mach 1) density."""
        d = density
        if mach > 1.0:
            m2 = mach * mach
            d = (gamma + 1.0) * m2 / (2.0 + (gamma - 1.0) * m2) * d
        if d < 0.0625:
            return 1.0 - math.sqrt(math.sqrt(d))
        if d < 0.25:
            return 0.75 - math.sqrt(d)
        return 0.0625 / d


class Cube:
    """One part's drag cube (``DragCubeList``): per face +x -x +y -y +z -z
    the area, drag coefficient and depth, after its stacked neighbours'
    occlusion."""

    def __init__(self, rec):
        faces = rec["faces"]
        self.area = [f[0] for f in faces]
        self.drag0 = [f[1] for f in faces]
        self.depth = [f[2] for f in faces]
        self.center = tuple(rec.get("center", (0.0, 0.0, 0.0)))
        self.size = tuple(rec.get("size", (1.0, 1.0, 1.0)))
        self.occ = list(self.area)
        self.drag = list(self.drag0)
        self.finish(None)

    @classmethod
    def from_box(cls, box):
        """No cube in the database: a box of the part's bounds, drag 1."""
        lo, hi = box
        s = [max(hi[i] - lo[i], 0.05) for i in range(3)]
        a = (s[1] * s[2], s[1] * s[2], s[0] * s[2], s[0] * s[2], s[0] * s[1], s[0] * s[1])
        d = (s[0], s[0], s[1], s[1], s[2], s[2])
        return cls({"faces": [[a[i], 1.0, d[i]] for i in range(6)],
                    "center": [0.5 * (lo[i] + hi[i]) for i in range(3)], "size": s})

    @staticmethod
    def facing(d, arr):
        return sum(a * _clamp01(vec.dot(d, f)) for a, f in zip(arr, FACES))

    def occlude(self, d, area):
        """AreaToCubeOperation(max(0, occ - area * dot)) along ``d``."""
        for i, f in enumerate(FACES):
            k = _clamp01(vec.dot(d, f))
            if k <= 0.0:
                continue
            self.occ[i] = max(0.0, self.occ[i] - area * k)
            gone = self.area[i] - self.occ[i]
            self.drag[i] = (max(0.0, (self.drag0[i] * self.area[i] - gone) / self.occ[i])
                            if self.occ[i] > 0.0 else 1e-5)

    def finish(self, phys):
        self.post_area = sum(self.occ)
        # The inverse drag coefficient AddSurfaceDragDirection weights the
        # exposed area and the taper with.
        self.inv = [1.0 / d if 0.01 < d < 1.0 else 1.0 for d in self.drag]
        self.cd_val = [phys.cd(d) if (phys is not None and d < 1.0) else d for d in self.drag]

    def area_dir(self, d):
        return sum(a * max(0.0, vec.dot(d, f)) for a, f in zip(self.occ, FACES))

    def set_drag(self, d, tip, surf, tail, mult, cd_pow):
        """AddSurfaceDragDirection for part-space flow direction ``d`` (the
        direction of motion): (exposed area, cross-section, taper, depth,
        drag coefficient)."""
        area = area_drag = cross = exposed = depth = taper = wsum = 0.0
        for i in range(6):
            f = FACES[i]
            k = d[0] * f[0] + d[1] * f[1] + d[2] * f[2]
            dn = (k + 1.0) * 0.5
            if dn <= 0.5:
                cv = (tail + (surf - tail) * dn * 2.0) * mult
            else:
                cv = (surf + (tip - surf) * (dn - 0.5) * 2.0) * mult
            a5 = self.occ[i] * cv
            area += a5
            c7 = self.drag[i] if self.drag[i] >= 1.0 else self.cd_val[i] ** cd_pow
            area_drag += a5 * c7
            if k > 0.0:
                cross += self.occ[i] * min(k, 1.0)
                wsum += k
                depth += k * self.depth[i]
                taper += k * self.inv[i]
            exposed += a5 / mult * self.inv[i] if mult else 0.0
        if wsum > 0.0:
            depth /= wsum
            taper /= wsum
        return exposed, cross, taper, depth, (area_drag / area if area > 0.0 else 0.0)


# -- occlusion (OcclusionData, OcclusionCone, OcclusionCylinder) -------------

def _rect_rect(ex, ey, min_x, max_x, min_y, max_y):
    a = (max_x - min_x) * (max_y - min_y)
    if max_x < -ex or min_x > ex or max_y < -ey or min_y > ey or a == 0.0:
        return 0.0
    return (max(0.0, min(ex, max_x) - max(-ex, min_x))
            * max(0.0, min(ey, max_y) - max(-ey, min_y)) / a)


def _circle_overlap(r_exist, r_new, d2):
    """AreaOfIntersection: the share of the new circle inside the other."""
    s = r_exist + r_new
    if d2 >= s * s:
        return 0.0
    if r_new == 0.0:
        return 1.0
    if r_exist == 0.0:
        return 0.0
    d = math.sqrt(d2)
    if r_exist >= d + r_new:
        return 1.0
    rn2 = r_new * r_new
    if r_new >= d + r_exist:
        return _clamp01(r_exist * r_exist / rn2)
    a, b = (r_new, r_exist) if r_new < r_exist else (r_exist, r_new)
    a2, b2 = a * a, b * b
    try:
        p1 = a2 * math.acos((d2 + a2 - b2) / (2.0 * d * a))
        p2 = b2 * math.acos((d2 + b2 - a2) / (2.0 * d * b))
        p3 = 0.5 * math.sqrt((-d + s) * (d + a - b) * (d - a + b) * (d + s))
    except ValueError:
        return 0.0
    return (p1 + p2 - p3) / (math.pi * rn2)


class Occ:
    """One part seen along a direction (OcclusionData.Update)."""
    __slots__ = ("pt", "max_dot", "centroid", "pcenter", "center", "mn", "mx",
                 "ext", "radius", "inv_fine", "depth", "cone")

    def __init__(self, pt):
        self.pt = pt

    def update(self, n, frame, cross, taper, depth):
        c = self.pt.box_center
        cd = vec.dot(c, n)
        self.centroid = cd
        self.pcenter = vec.sub(c, vec.scale(n, cd))
        mx = -1e30
        x0 = y0 = 1e30
        x1 = y1 = -1e30
        e1, e2 = frame
        for p in self.pt.box_corners:
            k = vec.dot(p, n)
            if k > mx:
                mx = k
            x = vec.dot(p, e1)
            y = vec.dot(p, e2)
            x0, x1 = min(x0, x), max(x1, x)
            y0, y1 = min(y0, y), max(y1, y)
        self.max_dot = mx
        self.mn = (x0, y0)
        self.mx = (x1, y1)
        self.ext = ((x1 - x0) * 0.5, (y1 - y0) * 0.5)
        self.center = (x0 + self.ext[0], y0 + self.ext[1])
        self.radius = math.sqrt(max(cross, 0.0) / math.pi)
        self.inv_fine = taper
        self.depth = depth

    def rect_share(self, other):
        ox, oy = -other.center[0], -other.center[1]
        return _rect_rect(other.ext[0], other.ext[1], ox + self.mn[0], ox + self.mx[0],
                          oy + self.mn[1], oy + self.mx[1])


class Cone:
    """OcclusionCone.Setup: the shock a part casts."""
    __slots__ = ("occ", "nose_dot", "angle", "shock_t", "shock_c", "occ_t", "occ_c", "occ_a")

    def __init__(self, o, sqrt_mach, sqrt_mach_angle, detach_angle):
        pt = o.pt
        t, c = pt.conv_t, pt.conv_c
        self.occ = o
        fine = math.asin(min(1.0, o.inv_fine)) if o.inv_fine < 1.0 else None
        if fine is not None and fine <= detach_angle:
            self.nose_dot = o.max_dot
            self.angle = max(fine * OBLIQUE_MIN_ANGLE,
                             sqrt_mach_angle * OBLIQUE_ANGLE + fine * OBLIQUE_PART_ANGLE)
            self.occ_a = 1.0
            if t >= DETACHED_HEAT - 0.05 and c >= DETACHED_COEFF - 0.05:
                self.occ_t, self.shock_t = OBLIQUE_CYL_HEAT, OBLIQUE_CONE_HEAT
                self.occ_c, self.shock_c = OBLIQUE_CYL_COEFF, OBLIQUE_CONE_COEFF
            else:
                v = max(t, BEHIND_DETACHED_HEAT)
                self.occ_t, self.shock_t = min(v, OBLIQUE_CYL_HEAT), min(v, OBLIQUE_CONE_HEAT)
                v = max(c, BEHIND_DETACHED_COEFF)
                self.occ_c, self.shock_c = min(v, OBLIQUE_CYL_COEFF), min(v, OBLIQUE_CONE_COEFF)
        else:
            self.nose_dot = o.max_dot + o.radius * o.inv_fine
            self.angle = DETACHED_END + (DETACHED_START - DETACHED_END) * _clamp01(
                sqrt_mach * DETACHED_MACH_ANGLE)
            self.occ_t = 0.0
            self.occ_a = 0.0
            self.occ_c = BEHIND_DETACHED_COEFF   # never read: occ_a is 0
            if t >= DETACHED_HEAT - 0.05 and c >= DETACHED_COEFF - 0.05:
                t, c = DETACHED_HEAT, DETACHED_COEFF
            else:
                t, c = BEHIND_DETACHED_HEAT, BEHIND_DETACHED_COEFF
            self.shock_t, self.shock_c = BEHIND_DETACHED_HEAT, BEHIND_DETACHED_COEFF
        pt.conv_t, pt.conv_c = t, c

    def radius_at(self, dot):
        return self.occ.radius + (self.nose_dot - dot) * math.tan(self.angle)

    def stats(self, o):
        """OcclusionData.GetShockStats: blends ``o``'s multipliers; returns
        its area factor."""
        pt = o.pt
        inside = o.rect_share(self.occ)
        shocked = 1.0
        if inside < 0.99:
            d2 = vec.dot(vec.sub(o.pcenter, self.occ.pcenter), vec.sub(o.pcenter, self.occ.pcenter))
            shocked = _circle_overlap(self.radius_at(o.centroid), o.radius, d2)
        else:
            inside = 1.0
        free = 1.0 - shocked
        band = shocked - inside
        pt.conv_t = free * pt.conv_t + band * self.shock_t + inside * self.occ_t
        pt.conv_c = free * pt.conv_c + band * self.shock_c + inside * self.occ_c
        return 1.0 - inside + inside * self.occ_a


def _frame(n, q_bf, world_q):
    """The two axes across ``n`` (vessel frame) that KSP measures a part's
    extents along: OcclusionData takes Quaternion.FromToRotation(velocity,
    Vector3.up) of *world* positions, so the rectangles are square to the
    world's axes, not the vessel's.  Below the inverse-rotation height the
    world holds the planet still as it stood when that began: its up is the
    spin axis and its other axes are the body frame's turned by the
    planet's rotation angle then (``world_q``).  Measured on two saves: the
    skin error is least at exactly that angle (36.5 and 14.0 degrees), and
    rectangles square to the vessel instead covered 76% of the pod with the
    docking port's box."""
    q_w = quat.mul(world_q, q_bf)
    n_w = quat.rotate(q_w, n)
    qi = quat.conj(quat.mul(quat.from_to(n_w, (0.0, 1.0, 0.0)), q_w))
    return quat.rotate(qi, (1.0, 0.0, 0.0)), quat.rotate(qi, (0.0, 0.0, 1.0))


# -- parts ---------------------------------------------------------------------

class PartThermal:
    """One part's thermal state (Part's thermal fields and PartThermalData)."""

    def __init__(self, part, cfg, cube, hsp):
        f = (cfg or {}).get("part", {})
        self.part = part
        self.index = part.index
        self.max_internal = part.rec.get("max_temperature") or _f(f, "maxTemp", 2000.0)
        skin_max = part.rec.get("max_skin_temperature") or _f(f, "skinMaxTemp", -1.0)
        self.max_skin = skin_max if skin_max > 0 else self.max_internal
        self.emissive = _f(f, "emissiveConstant", 0.4)
        absorb = _f(f, "absorptiveConstant", -1.0)
        self.absorptive = absorb if absorb >= 0.0 else self.emissive
        self.mass_mod = _f(f, "thermalMassModifier", 1.0)
        self.skin_mass_per_area = _f(f, "skinMassPerArea", 1.0)
        self.skin_mass_mod = _f(f, "skinThermalMassModifier", 1.0)
        self.skin_int_mult = _f(f, "skinInternalConductionMult", 1.0)
        self.skin_skin_mult = _f(f, "skinSkinConductionMult", 1.0)
        self.conductivity = _f(f, "heatConductivity", 0.12)
        self.convective_const = _f(f, "heatConvectiveConstant", 1.0)
        self.shielded = bool(part.rec.get("shielded"))
        self.cfg_mass = _f(f, "mass", 0.0)
        self.cube = cube
        self.hsp = hsp
        self.rot = tuple(part.rotation)
        self.pos = tuple(part.position)
        self.inv_rot = quat.conj(self.rot)
        # The cube's box in vessel space (for occlusion).
        c, s = cube.center, cube.size
        self.box_center = vec.add(self.pos, quat.rotate(self.rot, c))
        self.box_corners = [vec.add(self.pos, quat.rotate(self.rot, (c[0] + sx * s[0] * 0.5,
                                                                      c[1] + sy * s[1] * 0.5,
                                                                      c[2] + sz * s[2] * 0.5)))
                            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        self.skin = part.skin_temperature
        self.skin_unexp = part.skin_temperature
        self.internal = part.temperature
        self.exposed = False
        self.frac = 1.0
        self.exp_mass_mult = 1.0
        self.unexp_mass_mult = 0.0
        self.skin_exposed_area = 0.0
        self.conv_area = 0.0
        self.conv_t = self.conv_c = self.conv_a = 1.0
        self.sun_mult = self.body_mult = 1.0
        self.links = []
        self.node_contact = {}
        self.srf_contact = 0.0

    @property
    def rad_area(self):
        return self.cube.post_area

    def masses(self, cp):
        """FlightIntegrator.UpdateMassStats and SetSkinThermalMass (kJ/K).
        The world keeps kRPC's units, kg and kg a unit; KSP's are tonnes."""
        # A physicsless part (RCS blocks, valves) reads 0 kg through kRPC;
        # its thermal mass is still its config mass.
        dry = (self.part.dry_mass * 1e-3) or self.cfg_mass
        res = sum(r.amount * r.density * self.hsp.get(r.name, 0.0)
                  for r in self.part.resources) * 1e-3
        base = cp * self.mass_mod
        total = dry * base + res
        skin = max(0.1, min(0.001 * self.skin_mass_per_area * self.skin_mass_mod
                            * self.rad_area * base, dry * base * 0.5))
        self.skin_mass = skin
        self.int_mass = max(total - skin, 0.1)

    def unified(self):
        if self.exposed:
            return self.frac * self.skin + (1.0 - self.frac) * self.skin_unexp
        return self.skin


class Thermal:
    def __init__(self, vessel, model):
        self.phys = Physics(model.get("physics"), model.get("reentry_heat_scale", 1.2))
        cfg = model.get("partcfg") or {}
        cubes = model.get("dragcubes") or {}
        hsp = {k: v.get("hsp", 0.0) for k, v in (model.get("resource_defs") or {}).items()}
        self.vessel = vessel
        self.body = vessel.world.body
        self.parts = []
        for p in vessel.parts:
            rec = cubes.get(p.name)
            cube = Cube(rec) if rec else Cube.from_box(p.rec.get("bounding_box")
                                                       or [[-0.5] * 3, [0.5] * 3])
            self.parts.append(PartThermal(p, cfg.get(p.name), cube, hsp))
        self.root = model["vessel"].get("root") or 0
        self._stack(model)
        for pt in self.parts:
            pt.cube.finish(self.phys)
        self._links()
        self._n = 0
        self.atm_density_asl = self._density_asl()
        self.world_q = None
        self._world_axes(vessel.world.t, vessel.r)

    # -- setup ---------------------------------------------------------------

    def _stack(self, model):
        """DragCubeList.SetPartOcclusion: each stack node's neighbour covers
        its facing area of this part's faces; the node's contact area is that
        area (the surface-attach node's is this part's own facing area)."""
        attach = model.get("attach")
        nodes = model.get("nodes") or {}
        if not attach or len(attach) != len(self.parts):
            return
        for pt, a in zip(self.parts, attach):
            mine = nodes.get(pt.part.name) or {}
            for nid, target in a.get("att") or []:
                if nid not in mine or not 0 <= target < len(self.parts):
                    continue
                other = self.parts[target]
                n = vec.unit(tuple(mine[nid][3:6]))
                v = quat.rotate(other.inv_rot, quat.rotate(pt.rot, n))
                area = Cube.facing(vec.scale(v, -1.0), other.cube.area)
                pt.cube.occlude(n, area)
                pt.node_contact[target] = area
            srf = a.get("srf")
            if srf and "srfAttach" in mine and 0 <= srf[1] < len(self.parts):
                n = vec.unit(tuple(mine["srfAttach"][3:6]))
                pt.srf_contact = Cube.facing(n, pt.cube.area)
                pt.srf_target = srf[1]

    def _contact(self, a, b, fresh):
        """ThermalLink: the remote part's node to this one, else this part's
        node to it; at least 0.01 m^2.

        **On a fresh vessel every link is 0.01 m^2.**  KSP builds the thermal
        graph at load (UpdateThermalGraph) *before* SetPartOcclusion has set
        any node's ``contactArea``, and a link reads it once, when made.  The
        game's own conduction flux says so: pod to service bay, 52 kW where
        the nodes' 4.8 m^2 would have moved 9 MW.  The graph is rebuilt when
        a part is lost, and the links made then carry the real areas."""
        rule = LINK_AREAS if fresh else "real"
        if rule == "fresh":
            return 0.01
        unset = set()
        if rule == "root":
            unset = {pt.index for pt in self.parts
                     if pt.index == self.root or getattr(pt, "srf_target", None) == self.root}
        area = b.node_contact.get(a.index, 0.0) if b.index not in unset else 0.0
        if area <= 0.0 and getattr(b, "srf_target", None) == a.index and b.index not in unset:
            area = b.srf_contact
        if area <= 0.0 and a.index not in b.node_contact:
            if a.index not in unset:
                area = a.node_contact.get(b.index, 0.0)
                if area <= 0.0 and getattr(a, "srf_target", None) == b.index:
                    area = a.srf_contact
        return max(area, 0.01)

    def _links(self, fresh=True):
        by_index = {pt.index: pt for pt in self.parts if pt.part.alive}
        for pt in self.parts:
            pt.links = []
        for pt in by_index.values():
            parent = pt.part.rec.get("parent")
            if parent is not None and parent >= 0 and parent != pt.index and parent in by_index:
                other = by_index[parent]
                pt.links.append([other, self._contact(pt, other, fresh)])
                other.links.append([pt, self._contact(other, pt, fresh)])

    def parts_changed(self):
        """A part was lost: KSP rebuilds the thermal graph, now with the
        nodes' contact areas."""
        self._links(fresh=False)

    def _density_asl(self):
        atm = self.body.atm
        p = atm.pressure(0.0)
        return atm.density(p, atm.temperature_curve(0.0)) or 1.225

    def _world_axes(self, t, r):
        """The world's axes: fixed when the vessel is first below the
        inverse-rotation height (at load, for a save in the air)."""
        if self.world_q is None and vec.norm(r) - self.body.radius < INVERSE_ROTATION_ALT:
            self.world_q = quat.from_axis_angle((0.0, 1.0, 0.0), self.body.phi(t))

    # -- the environment -----------------------------------------------------

    def _radiant(self, env, up_bf, sun_bf, density):
        """(solar W/m^2 at the vessel, planet W/m^2): CalculateSunBodyFlux,
        GetSolarAtmosphericEffects and GetAtmoThermalStats."""
        body = self.body
        phys = self.phys
        r = env["r_bf"]
        rs = vec.dot(r, sun_bf)
        lit = not (rs < 0.0 and vec.norm(vec.sub(r, vec.scale(sun_bf, rs))) < body.radius)
        sun_dot = vec.dot(sun_bf, up_bf)
        solar = phys.solar_home if lit else 0.0
        if solar and env["pressure"] > 0.0:
            raf = body.radius / body.atmosphere_depth * -math.log(1e-6)
            k = raf * sun_dot
            air_mass = (math.sqrt(2.0 * raf + 1.0) if k < 0.0
                        else math.sqrt(k * k + 2.0 * raf + 1.0) - k)
            n = (1.0 - phys.insolation) * self.atm_density_asl
            solar *= n / (n + density * air_mass * phys.insolation)
        # The planet: its own emission at a temperature that follows the
        # latitude and the Sun, and the sunlight it reflects.
        atm = body.atm
        alt = env["alt"]
        spin = (0.0, 1.0, 0.0)
        c_up = max(-1.0, min(1.0, vec.dot(spin, up_bf)))
        c_sun = max(-1.0, min(1.0, vec.dot(spin, sun_bf)))
        colat, sun_colat = math.acos(c_up), math.acos(c_sun)
        near = (1.0 + math.cos(sun_colat - colat)) * 0.5
        far = (1.0 + math.cos(sun_colat + colat)) * 0.5
        lead = atm.sun_dot(r, sun_bf)
        span = near - far
        day = (lead - far) / span if span > 0.001 else far + span * 0.5
        lat = 90.0 - math.degrees(colat if colat <= math.pi / 2 else math.pi - colat)
        t0 = atm.temperature_curve(0.0)
        body_t = t0 + atm.lat_bias(lat) + atm.lat_sun_mult(lat) * day + atm.axial_mult(lat) * atm.axial_bias(0.0)
        cold = t0 + atm.lat_bias(90.0) + atm.axial_mult(0.0)
        hot = t0 + atm.lat_bias(0.0) + atm.lat_sun_mult(0.0) + atm.axial_mult(0.0)
        k18 = _clamp01(1.0 - math.sqrt(hot) * 0.0016)

        def lu(a, b, t):
            return a + (b - a) * t
        b = lu(lu(lu(cold, hot, lu(0.782048841, 0.87513007, k18)),
                  lu(cold, hot, lu(0.093081228, 0.87513007, k18)), day),
               lu(cold, hot, lu(0.398806364, 0.797612728, k18)), near)
        t = _lerp(body_t, b, 0.2 + alt / body.radius * 0.5)
        dil = min(1.0, body.radius ** 2 / (body.radius + alt) ** 2)
        emit = SIGMA * BODY_EMISSIVITY * t ** 4 * dil
        albedo = phys.solar_home * 0.5 * (sun_dot + 1.0) * BODY_ALBEDO * dil
        return solar, emit + albedo

    def _occlude_convection(self, n, mach, flows, q_bf):
        live = [pt for pt in self.parts if pt.part.alive]
        for pt in live:
            pt.conv_t = pt.conv_c = pt.conv_a = 1.0
        if mach <= 1.0 or not live:
            return
        frame = _frame(n, q_bf, self._q_world)
        occs = []
        for pt in live:
            o = Occ(pt)
            _, cross, taper, depth, _ = flows[pt.index]
            o.update(n, frame, cross, taper, depth)
            occs.append(o)
        occs.sort(key=lambda o: o.max_dot)
        sm = math.sqrt(mach)
        sma = math.asin(1.0 / sm)
        det = 0.7957 * (1.0 - 1.0 / (mach * sm))
        cones = [Cone(occs[-1], sm, sma, det)]
        for o in reversed(occs[:-1]):
            pt = o.pt
            a = 1.0
            for cone in cones:
                a *= cone.stats(o)
                if a <= 0.001:
                    a = 0.0
            pt.conv_a = a
            if a > 0.0:
                cones.append(Cone(o, sm, sma, det))

    def _occlude_cylinders(self, n, attr, q_bf):
        live = [pt for pt in self.parts if pt.part.alive]
        if not live:
            return
        frame = _frame(n, q_bf, self._q_world)
        occs = []
        for pt in live:
            o = Occ(pt)
            # OcclusionData.Update keeps the flow's cube numbers even here;
            # a cylinder reads only the box.
            o.update(n, frame, pt.last_cross, 1.0, 0.0)
            occs.append(o)
        occs.sort(key=lambda o: o.max_dot)
        setattr(occs[-1].pt, attr, 1.0)
        shadows = [occs[-1]]
        for o in reversed(occs[:-1]):
            m = 1.0
            for s in shadows:
                m -= o.rect_share(s)
                if m <= 0.001:
                    m = 0.0
            setattr(o.pt, attr, m)
            if m > 0.0:
                shadows.append(o)

    # -- the step ------------------------------------------------------------

    def step(self, dt, env):
        """Advance every part's temperatures by ``dt`` in ``env`` (the
        vessel's environment dict); returns the indices of parts that failed."""
        phys = self.phys
        world = self.vessel.world
        density = env["density"]
        in_air = env["pressure"] > 0.0 and density > 0.0
        speed = env["speed"]
        mach = env["mach"] if in_air else 0.0
        if in_air:
            air_t = env["temperature"]
            ext_t = phys.external_temperature(speed, mach, air_t)
            h = phys.convective_coefficient(density, speed, mach)
            pseudo_re = density * speed
        else:
            air_t = ext_t = phys.space_t
            h = 0.0
            pseudo_re = 0.0
        dtl = phys.density_thermal_lerp(density if in_air else 0.0, mach)
        brt = _lerp(air_t, phys.space_t, dtl)
        brt_exp = _lerp(ext_t, phys.space_t, dtl)

        q_bf = env["q_bf"]
        if self.world_q is None:
            self._world_axes(world.t, env["r_bf"])
        self._q_world = self.world_q or (0.0, 0.0, 0.0, 1.0)
        to_vessel = quat.conj(q_bf)
        up_bf = vec.unit(env["r_bf"])
        sun_bf = self.body.sun_bf(world.t)
        up_v = quat.rotate(to_vessel, up_bf)
        sun_v = quat.rotate(to_vessel, sun_bf)
        solar, planet = self._radiant(env, up_bf, sun_bf, density if in_air else 0.0)
        n = vec.scale(env["v_body"], 1.0 / speed) if speed > 1e-3 else (0.0, 1.0, 0.0)

        tip, surf, tail = phys.tip(mach), phys.surf(mach), phys.tail(mach)
        mult, cd_pow = phys.mult(mach), phys.cd_power(mach)
        live = [pt for pt in self.parts if pt.part.alive]
        flows = {}
        for pt in live:
            flows[pt.index] = pt.cube.set_drag(quat.rotate(pt.inv_rot, n), tip, surf, tail,
                                               mult, cd_pow)
            pt.last_cross = flows[pt.index][1]
        self._occlude_convection(n, mach, flows, q_bf)
        if self._n % SUN_EVERY == 0:
            self._occlude_cylinders(sun_v, "sun_mult", q_bf)
            self._occlude_cylinders(vec.scale(up_v, -1.0), "body_mult", q_bf)
        self._n += 1

        cp = phys.cp
        rad_k = phys.radiation * 1e-3
        for pt in live:
            pt.masses(cp)
            self._skin_properties(pt, flows[pt.index][0], mach, in_air)
            # PrecalcConduction
            pt.cond_mult = phys.conduction * 10.0 * pt.conductivity
            pt.skin_skin = (pt.skin_skin_mult * phys.skin_skin * min(pt.frac, 1.0 - pt.frac)
                            * 2.0 * math.sqrt(pt.rad_area))
            # PrecalcConvection
            pt.post_shock_t = air_t + (ext_t - air_t) * pt.conv_t
            coeff = 0.0
            if pt.conv_area > 0.0 and h > 0.0:
                coeff = h * pt.conv_area * 0.001 * pt.convective_const * pt.conv_c
                re = pseudo_re * flows[pt.index][4]
                if re > phys.turb_start:
                    if re > phys.turb_end:
                        coeff *= phys.turb_mult
                    else:
                        coeff *= 1.0 + (phys.turb_mult - 1.0) * (re - phys.turb_start) / (
                            phys.turb_end - phys.turb_start)
                coeff = min(coeff, pt.skin_mass * pt.frac)
            pt.coeff = coeff
            # PrecalcRadiation
            pt.emiss = pt.emissive * rad_k
            absorb = pt.absorptive * rad_k
            pt.brt_exp = brt + (brt_exp - brt) * pt.conv_t
            pt.exp_flux = pt.unexp_flux = 0.0
            if pt.shielded:
                continue
            unexp_area = pt.rad_area * (1.0 - pt.frac)
            if solar > 0.0:
                sun_area = pt.cube.area_dir(quat.rotate(pt.inv_rot, sun_v)) * pt.sun_mult
                if sun_area > 0.0:
                    f = absorb * solar
                    if pt.exposed:
                        k = (vec.dot(sun_v, n) + 1.0) * 0.5
                        a_exp = min(sun_area, pt.skin_exposed_area * k)
                        a_un = min(sun_area - a_exp, unexp_area * (1.0 - k))
                        pt.exp_flux += f * a_exp
                        pt.unexp_flux += f * a_un
                    else:
                        pt.exp_flux += f * sun_area
            if planet > 0.0:
                down = vec.scale(up_v, -1.0)
                body_area = pt.cube.area_dir(quat.rotate(pt.inv_rot, down)) * pt.body_mult
                if body_area > 0.0:
                    f = planet * dtl * absorb
                    if pt.exposed:
                        k = (vec.dot(down, n) + 1.0) * 0.5
                        a_exp = min(body_area, pt.skin_exposed_area * k)
                        a_un = min(body_area - a_exp, unexp_area * (1.0 - k))
                        pt.exp_flux += f * a_exp
                        pt.unexp_flux += f * a_un
                    else:
                        pt.exp_flux += f * body_area

        self._conduction(live)
        conv = phys.convergence
        space = phys.space_t
        failed = []
        for pt in live:
            k_exp = dt / pt.skin_mass * pt.exp_mass_mult
            k_un = dt / pt.skin_mass * pt.unexp_mass_mult if pt.frac > 0.0 else 0.0
            k_int = dt / pt.int_mass
            # convection and radiation
            conv_flux = (pt.post_shock_t - pt.skin) * pt.coeff if pt.conv_area > 0.0 else 0.0
            rad_exp = pt.exp_flux
            rad_un = pt.unexp_flux
            if not pt.shielded:
                a_exp = pt.rad_area * pt.frac
                a_un = pt.rad_area * (1.0 - pt.frac)
                if pt.skin_unexp > 0.0 and a_un > 0.0:
                    rad_un -= (pt.skin_unexp ** 4 - brt ** 4) * SIGMA * pt.emiss * a_un
                if pt.skin > 0.0 and a_exp > 0.0:
                    rad_exp -= (pt.skin ** 4 - pt.brt_exp ** 4) * SIGMA * pt.emiss * a_exp
            # The flows as the game reports them (Part.thermal*Flux, kW).
            pt.q_conv = conv_flux * conv
            pt.q_rad = rad_exp + rad_un
            pt.q_cond = pt.int_cond * conv
            pt.q_skin_int = (pt.skin_int + pt.unexp_int) * conv
            pt.internal += (pt.int_cond + pt.skin_int + pt.unexp_int) * conv * k_int
            pt.skin += (pt.skin_cond * pt.frac - pt.skin_skin_flux - pt.skin_int) * k_exp * conv
            pt.skin_unexp += (pt.skin_cond * (1.0 - pt.frac) + pt.skin_skin_flux
                              - pt.unexp_int) * k_un * conv
            pt.skin += conv_flux * conv * k_exp
            pt.skin += rad_exp * k_exp
            pt.skin_unexp += rad_un * k_un
            pt.skin = max(pt.skin, space)
            pt.skin_unexp = max(pt.skin_unexp, space)
            pt.internal = max(pt.internal, space)
            pt.part.skin_temperature = pt.skin
            pt.part.temperature = pt.internal
            if pt.internal > pt.max_internal or pt.skin > pt.max_skin:
                failed.append(pt.index)
        return failed

    def _skin_properties(self, pt, exposed_area, mach, in_air):
        """FlightIntegrator.SetSkinProperties: the exposed fraction of the
        skin, and moving heat between the two skins when it changes."""
        phys = self.phys
        rad = max(pt.rad_area, 0.001)
        frac = None
        if not pt.shielded and in_air:
            t = -phys.full_area_min + (mach - phys.full_area_lo) / (phys.full_area_hi - phys.full_area_lo)
            pt.conv_area = _lerp(rad, exposed_area, t) * pt.conv_a
            r = pt.conv_area / rad
            # A share at or under 0.001 is no exposure at all.
            frac = 1.0 if r >= 0.999 else r if r > 0.001 else None
        if frac is not None:
            if not (pt.exposed and frac != 1.0):
                pt.skin_unexp = pt.skin
                pt.exposed = True
            pt.exp_mass_mult = 1.0 / frac if frac > 0.0 else 1.0
            pt.unexp_mass_mult = 1.0 / (1.0 - frac) if frac < 1.0 else 0.0
            old = pt.frac
            if (old != frac and pt.skin_unexp != pt.skin and 0.0 < old < 1.0 and frac < 1.0):
                tm = pt.skin_mass
                e_exp = pt.skin * old * tm
                e_un = pt.skin_unexp * (1.0 - old) * tm
                d = frac - old
                move = d / (1.0 - old) * e_un if d > 0.0 else d / old * e_exp
                if frac > 0.0:
                    pt.skin = (e_exp + move) * pt.exp_mass_mult / tm
                pt.skin_unexp = (e_un - move) * pt.unexp_mass_mult / tm
            pt.frac = frac
            pt.skin_exposed_area = frac * rad
            return
        if pt.exposed:
            pt.skin = pt.unified()
        pt.exposed = False
        pt.conv_area = 0.0
        pt.skin_exposed_area = 0.0
        pt.unexp_mass_mult = 0.0
        pt.frac = 1.0
        pt.exp_mass_mult = 1.0

    def _conduction(self, live):
        """FlightIntegrator.UpdateConduction."""
        phys = self.phys
        alive = {pt.index for pt in live}
        for pt in live:
            pt.int_cond = pt.local_int = 0.0
            pt.skin_cond = pt.local_skin = 0.0
            pt.skin_skin_flux = pt.skin_int = pt.unexp_int = 0.0
            pt.u = pt.unified()
        # Interior to interior, coldest first, each part drawing from its
        # hotter links, limited to their common temperature.
        order = sorted(live, key=lambda p: p.internal)
        for pt in reversed(order[:-1]):
            e = pt.int_mass * pt.internal
            m = pt.int_mass
            for other, area in pt.links:
                if other.index not in alive:
                    continue
                d = other.internal - pt.internal
                if d <= 0.0:
                    continue
                e += other.int_mass * other.internal
                m += other.int_mass
                k = pt.cond_mult * other.conductivity
                if pt.shielded != other.shielded:
                    k *= phys.shielded_conduction
                q = area * k * d
                other.local_int -= q
                pt.local_int += q
            if pt.local_int != 0.0:
                eq = e / m
                scale = 1.0
                step = pt.local_int / pt.int_mass
                over = pt.internal + step - eq
                if over > step:
                    scale = 0.0
                elif over > 0.0 and step > 0.0:
                    scale = (step - over) / step
                    pt.local_int *= scale
                for other, _ in pt.links:
                    if other.index not in alive:
                        continue
                    other.local_int *= scale
                    if other.local_int < 0.0:
                        step = other.local_int / other.int_mass
                        over = other.internal + step - eq
                        if over < 0.0 and step < 0.0:
                            cap = over * other.int_mass
                            if other.local_int < cap:
                                pt.local_int += cap
                                other.local_int -= cap
                            else:
                                pt.local_int += other.local_int
                                other.local_int = 0.0
                    other.int_cond += other.local_int
                    other.local_int = 0.0
                if pt.local_int > 0.0:
                    pt.int_cond += pt.local_int
                pt.local_int = 0.0
        # Skins: exposed <-> unexposed, each skin <-> the interior, and skin
        # to skin between parts, again coldest first.
        order = sorted(live, key=lambda p: p.u)
        count = len(order)
        for idx in range(count - 1, -1, -1):
            pt = order[idx]
            if not pt.rad_area > 0.0:
                continue
            half = pt.cond_mult * 0.5
            t_int_e = pt.internal * pt.int_mass
            if pt.exposed:
                if pt.frac < 1.0:
                    d = pt.skin - pt.skin_unexp
                    if abs(d) > 0.001:
                        k = half * pt.skin_skin
                        s_e = -d * k / pt.skin_mass * pt.exp_mass_mult
                        s_u = d * k / pt.skin_mass * pt.unexp_mass_mult
                        o_e = pt.skin + s_e - pt.u
                        o_u = pt.skin_unexp + s_u - pt.u
                        v1 = v2 = 1.0
                        if d > 0.0:
                            if o_e < 0.0:
                                v1 = (s_e - o_e) / s_e
                            if o_u > 0.0:
                                v2 = (s_u - o_u) / s_u
                        else:
                            if o_e > 0.0:
                                v1 = (s_e - o_e) / s_e
                            if o_u < 0.0:
                                v2 = (s_u - o_u) / s_u
                        pt.skin_skin_flux = k * max(0.0, min(v1, v2)) * d
                    # unexposed skin <-> interior
                    d = pt.skin_unexp - pt.internal
                    if abs(d) > 0.001:
                        pt.unexp_int = self._skin_int(pt, d, pt.rad_area * (1.0 - pt.frac) * half,
                                                      pt.skin_mass * (1.0 - pt.frac),
                                                      pt.skin_unexp, pt.unexp_mass_mult, t_int_e)
            # exposed skin <-> interior
            d = pt.skin - pt.internal
            if abs(d) > 0.001:
                pt.skin_int = self._skin_int(pt, d, pt.frac * pt.rad_area * half,
                                             pt.skin_mass * pt.frac, pt.skin,
                                             pt.exp_mass_mult, t_int_e)
            # skin to skin between parts
            if idx >= count - 1:
                continue
            e = pt.skin_mass * pt.u
            m = pt.skin_mass
            for other, area in pt.links:
                if other.index not in alive or pt.shielded != other.shielded:
                    continue
                d = other.u - pt.u
                if d <= 0.0:
                    continue
                if pt.skin >= max(other.skin, other.skin_unexp):
                    continue
                e += other.skin_mass * other.u
                m += other.skin_mass
                q = (half * other.conductivity * pt.skin_skin_mult * other.skin_skin_mult
                     * phys.skin_skin * math.sqrt(area) * 200.0 * d)
                other.local_skin = -q
                pt.local_skin += q
            if pt.local_skin == 0.0:
                continue
            eq = e / m
            scale = 1.0
            step = pt.local_skin / pt.skin_mass
            over = pt.u + step - eq
            if over > step:
                scale = 0.0
            elif over > 0.0 and step > 0.0:
                scale = (step - over) / step
                pt.local_skin *= scale
            for other, _ in pt.links:
                if other.index not in alive:
                    continue
                other.local_skin *= scale
                if other.local_skin < 0.0:
                    step = other.local_skin / other.skin_mass
                    over = other.u + step - eq
                    if over < 0.0 and step < 0.0:
                        cap = over * other.skin_mass
                        if other.local_skin < cap:
                            pt.local_skin += cap
                            other.local_skin -= cap
                        else:
                            pt.local_skin += other.local_skin
                            other.local_skin = 0.0
                other.skin_cond += other.local_skin
                other.local_skin = 0.0
            if pt.local_skin > 0.0:
                pt.skin_cond += pt.local_skin
            pt.local_skin = 0.0

    def _skin_int(self, pt, d, base, skin_share, skin_t, mass_mult, t_int_e):
        """One skin's conduction to the interior, limited to their common
        temperature (UpdateConduction's two skin-internal blocks)."""
        k = base * self.phys.skin_internal * pt.skin_int_mult
        eq = (t_int_e + skin_t * skin_share) / (pt.int_mass + skin_share)
        s_s = -d * k / pt.skin_mass * mass_mult
        s_i = d * k / pt.int_mass
        o_s = skin_t + s_s - eq
        o_i = pt.internal + s_i - eq
        v_i = v_s = 1.0
        if d > 0.0:
            if o_s < 0.0:
                v_s = (s_s - o_s) / s_s
            if o_i > 0.0:
                v_i = (s_i - o_i) / s_i
        else:
            if o_s > 0.0:
                v_s = (s_s - o_s) / s_s
            if o_i < 0.0:
                v_i = (s_i - o_i) / s_i
        return k * max(0.0, min(v_i, v_s)) * d
