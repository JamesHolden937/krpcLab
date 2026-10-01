#!/usr/bin/env python3
"""Read a save's vehicle, state and aerodynamics out of a real game.

    ./kspSim/tools/probe.py --instance 1 --save qs_plane

Writes ``kspSim/models/<save>.json``: everything the simulator needs to fly
that save without the game -- the initial state, the body, the parts and
their modules and resources, the engines, wheels and reaction wheels, and
aerodynamic tables taken from the game's own
``Flight.simulate_aerodynamic_wrench_at``.

The aerodynamics are not re-derived from part configs.  Stock aero has drag
cubes, occlusion and body lift computed at runtime, and the install carries
mods that replace control-surface modules; the game's own answer is the only
one that cannot drift from the game.  Tables are force and torque over
dynamic pressure, in body axes (x right, y nose, z belly), over Mach x alpha
x sideslip, each Mach at an altitude it is flown at (pseudo-Reynolds drag
depends on density as well as Mach).  Control inputs, gear and angular
velocity are delta tables at zero sideslip.
"""
import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc  # noqa: E402

from kspSim import quat  # noqa: E402

MODELS = os.path.join(ROOT, "kspSim", "models")

MACH = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2,
        1.35, 1.5, 1.75, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0, 9.5]
# The whole circle: a tumbling booster, or one flown engine-first, lives far
# from the +-35 deg a glider does (the first grid stepped 30 deg out there and
# the booster's one-step error was 100% of its axial force).
ALPHA = ([-180.0 + 5.0 * i for i in range(30)] + [-30.0 + 2.5 * i for i in range(28)]
         + [40.0 + 5.0 * i for i in range(28)])
# Out to the poles: (alpha, beta) covers the sphere only if beta reaches
# +-90; the first grid stopped at 30 and clamped beyond it.
BETA = [-90, -60, -45, -30, -15, -8, -4, -2, 0, 2, 4, 8, 15, 30, 45, 60, 90]
# Control levels per axis; the base (0) is the base table.
LEVELS = [-1.0, -0.5, 0.5, 1.0]
OMEGA = 0.1   # rad/s per axis for the damping table


def altitude_for_mach(mach):
    """Where this Mach number is flown on an entry, so the density is right."""
    if mach >= 5.0:
        return 45000.0
    if mach >= 3.0:
        return 35000.0
    if mach >= 1.5:
        return 25000.0
    if mach >= 0.9:
        return 14000.0
    return 4000.0


def body_velocity(alpha_deg, beta_deg, speed):
    a = math.radians(alpha_deg)
    b = math.radians(beta_deg)
    return (speed * math.sin(b), speed * math.cos(a) * math.cos(b),
            speed * math.sin(a) * math.cos(b))


def connect(instance):
    base = os.path.join(ROOT, "testInstances", "ksp%s" % instance)
    rpc = int(open(os.path.join(base, ".rpc_port")).read())
    stream = int(open(os.path.join(base, ".stream_port")).read())
    return krpc.connect(name="kspsim-probe", rpc_port=rpc, stream_port=stream)


def safe(fn, default=None):
    try:
        return fn()
    except Exception:                                   # noqa: BLE001
        return default


def part_record(part, index_of, vframe):
    rec = {
        "name": part.name, "title": part.title,
        "position": list(part.position(vframe)),
        "rotation": list(part.rotation(vframe)),
        "direction": list(part.direction(vframe)),
        "com": list(safe(lambda: part.center_of_mass(vframe), part.position(vframe))),
        "mass": part.mass, "dry_mass": part.dry_mass,
        "max_skin_temperature": part.max_skin_temperature,
        "max_temperature": part.max_temperature,
        "skin_temperature": part.skin_temperature,
        "temperature": part.temperature,
        "impact_tolerance": safe(lambda: part.impact_tolerance, 0.0),
        "parent": index_of.get(safe(lambda: part.parent._object_id)),
        "stage": part.stage, "decouple_stage": part.decouple_stage,
        "shielded": safe(lambda: part.shielded, False),
        "bounding_box": [list(v) for v in part.bounding_box(vframe)],
        "resources": [], "modules": [],
    }
    for r in part.resources.all:
        rec["resources"].append({"name": r.name, "amount": r.amount, "max": r.max,
                                 "density": r.density, "enabled": r.enabled})
    for m in part.modules:
        mod = {"name": m.name, "fields": safe(lambda: dict(m.fields), {}),
               "actions": safe(lambda: list(m.actions), []),
               "events": safe(lambda: list(m.events), [])}
        rec["modules"].append(mod)
    return rec


def probe_static(conn, vessel):
    sc = conn.space_center
    body = vessel.orbit.body
    bframe = body.reference_frame
    vframe = vessel.reference_frame
    parts = vessel.parts.all
    index_of = {p._object_id: i for i, p in enumerate(parts)}
    ctrl = vessel.control
    model = {
        "ut": sc.ut,
        "vessel": {
            "name": vessel.name, "mass": vessel.mass, "dry_mass": vessel.dry_mass,
            "situation": str(vessel.situation).split(".")[-1],
            "moment_of_inertia": list(vessel.moment_of_inertia),
            "inertia_tensor": list(vessel.inertia_tensor),
            "bounding_box": [list(v) for v in vessel.bounding_box(vframe)],
            "available_torque": [list(v) for v in vessel.available_torque],
            "available_reaction_wheel_torque": [list(v) for v in vessel.available_reaction_wheel_torque],
            "available_rcs_torque": [list(v) for v in vessel.available_rcs_torque],
            "available_engine_torque": [list(v) for v in vessel.available_engine_torque],
            "available_control_surface_torque": [list(v) for v in vessel.available_control_surface_torque],
            "root": index_of.get(vessel.parts.root._object_id),
            "controlling": index_of.get(safe(lambda: vessel.parts.controlling._object_id)),
        },
        "state": {
            "position": list(vessel.position(bframe)),
            "velocity": list(vessel.velocity(bframe)),
            "rotation": list(vessel.rotation(bframe)),
            "position_nonrot": list(vessel.position(body.non_rotating_reference_frame)),
            "velocity_nonrot": list(vessel.velocity(body.non_rotating_reference_frame)),
            "rotation_nonrot": list(vessel.rotation(body.non_rotating_reference_frame)),
            "angular_velocity": list(vessel.angular_velocity(bframe)),
            "throttle": ctrl.throttle, "sas": ctrl.sas, "rcs": ctrl.rcs,
            "gear": ctrl.gear, "brakes": ctrl.brakes, "lights": ctrl.lights,
            "abort": ctrl.abort,
            "action_groups": [ctrl.get_action_group(i) for i in range(10)],
            "rails_warp_factor": sc.rails_warp_factor,
        },
        "body": {
            "name": body.name, "equatorial_radius": body.equatorial_radius,
            "gravitational_parameter": body.gravitational_parameter,
            "rotational_period": body.rotational_period,
            "rotational_speed": body.rotational_speed,
            "rotation_angle": body.rotation_angle,
            "initial_rotation": body.initial_rotation,
            "atmosphere_depth": body.atmosphere_depth,
            "surface_gravity": body.surface_gravity,
            "has_atmosphere": body.has_atmosphere,
            "sphere_of_influence": body.sphere_of_influence,
        },
        "parts": [part_record(p, index_of, vframe) for p in parts],
    }
    nonrot = body.non_rotating_reference_frame
    # The rotating frame's axes in the non-rotating frame, now: fixes the
    # relation between the two exactly rather than trusting initialRotation.
    model["body"]["frame_axes_nonrot"] = [
        list(sc.transform_direction(ax, bframe, nonrot))
        for ax in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    sun = sc.bodies.get("Sun")
    if sun is not None:
        model["body"]["sun_position_nonrot"] = list(sun.position(nonrot))
        model["body"]["sun_position_body"] = list(sun.position(bframe))

    engines = []
    for e in vessel.parts.engines:
        rec = {"part": index_of[e.part._object_id], "active": e.active,
               "max_thrust": e.max_thrust, "max_vacuum_thrust": e.max_vacuum_thrust,
               "thrust_limit": e.thrust_limit,
               "vacuum_isp": e.vacuum_specific_impulse,
               "sea_level_isp": e.kerbin_sea_level_specific_impulse,
               "isp_at": [[p, e.specific_impulse_at(p)] for p in (0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)],
               "thrust_at": [[p, e.max_thrust_at(p)] for p in (0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)],
               "propellants": [{"name": p.name, "ratio": p.ratio,
                                "ignore_for_isp": p.ignore_for_isp}
                               for p in e.propellants],
               "throttle_locked": e.throttle_locked,
               "gimballed": e.gimballed,
               "gimbal_range": safe(lambda: e.gimbal_range, 0.0),
               "thrusters": []}
        for t in e.thrusters:
            # thrust_position intermittently throws NullReference right after
            # a load; the part's own position and axis are the fallback.
            rec["thrusters"].append({
                "position": list(safe(lambda: t.thrust_position(vframe),
                                      e.part.position(vframe))),
                "direction": list(safe(lambda: t.thrust_direction(vframe),
                                       e.part.direction(vframe)))})
        engines.append(rec)
    model["engines"] = engines

    model["reaction_wheels"] = [
        {"part": index_of[w.part._object_id], "active": w.active,
         "max_torque": [list(v) for v in w.max_torque],
         "authority_limiter": w.authority_limiter}
        for w in vessel.parts.reaction_wheels]
    model["control_surfaces"] = [
        {"part": index_of[c.part._object_id], "pitch": c.pitch_enabled,
         "yaw": c.yaw_enabled, "roll": c.roll_enabled,
         "authority_limiter": c.authority_limiter, "inverted": c.inverted,
         "deployed": c.deployed, "surface_area": c.surface_area}
        for c in vessel.parts.control_surfaces]
    model["rcs"] = [
        {"part": index_of[r.part._object_id], "enabled": r.enabled,
         "max_thrust": r.max_thrust, "max_vacuum_thrust": r.max_vacuum_thrust,
         "vacuum_isp": r.vacuum_specific_impulse,
         "propellants": list(r.propellants),
         "thrusters": [{"position": list(safe(lambda: t.thrust_position(vframe),
                                                  r.part.position(vframe))),
                        "direction": list(safe(lambda: t.thrust_direction(vframe),
                                               r.part.direction(vframe)))}
                       for t in r.thrusters],
         "pitch": r.pitch_enabled, "yaw": r.yaw_enabled, "roll": r.roll_enabled}
        for r in vessel.parts.rcs]
    model["wheels"] = [
        {"part": index_of[w.part._object_id], "radius": w.radius,
         "has_brakes": w.has_brakes, "brakes": safe(lambda: w.brakes, 0.0),
         "steerable": w.steerable, "deployable": w.deployable,
         "deployed": safe(lambda: w.deployed, True),
         "has_suspension": w.has_suspension,
         "spring": safe(lambda: w.suspension_spring_strength, 0.0),
         "damper": safe(lambda: w.suspension_damper_strength, 0.0),
         "stress_tolerance": safe(lambda: w.stress_tolerance, 0.0),
         "bounding_box": [list(v) for v in w.part.bounding_box(vframe)]}
        for w in vessel.parts.wheels]
    model["drains"] = [
        {"part": index_of[d.part._object_id]}
        for d in safe(lambda: vessel.parts.resource_drains, [])]
    return model


def probe_atmosphere(conn, body, ut):
    """Density, temperature and pressure the game computes, for validation."""
    bframe = body.reference_frame
    rows = []
    for lat in (-60, -30, 0, 30, 60):
        for lon in (-150, -90, -30, 30, 90, 150):
            for alt in (0, 5000, 15000, 30000, 45000, 60000):
                pos = body.position_at_altitude(lat, lon, alt, bframe)
                rows.append([lat, lon, alt,
                             body.atmospheric_density_at_position(pos, bframe),
                             body.temperature_at(pos, bframe)])
    rows_std = [[alt, body.pressure_at(alt), body.density_at(alt)]
                for alt in range(0, 70001, 2500)]
    return {"ut": ut, "at_position": rows, "standard": rows_std}


class WrenchProbe:
    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel
        self.body = vessel.orbit.body
        self.bframe = self.body.reference_frame
        self.flight = vessel.flight(self.bframe)
        self.rot = tuple(vessel.rotation(self.bframe))
        self.ut = conn.space_center.ut
        self.cache = {}

    def site(self, alt):
        if alt not in self.cache:
            # Under the vessel's own ground track: the temperature model
            # varies with latitude and sun angle, and the density at the
            # probe point is read back rather than assumed.
            pos = self.body.position_at_altitude(0.0, 0.0, alt, self.bframe)
            rho = self.body.atmospheric_density_at_position(pos, self.bframe)
            temp = self.body.temperature_at(pos, self.bframe)
            # KSP: speed of sound from the gas law at that temperature.
            c = math.sqrt(1.39999997615814 * 8.31446261815324 * temp / 0.0289644002914429)
            self.cache[alt] = (pos, rho, temp, c)
        return self.cache[alt]

    def wrench(self, mach, alpha, beta, omega_body=(0.0, 0.0, 0.0), alt=None):
        alt = altitude_for_mach(mach) if alt is None else alt
        pos, rho, temp, c = self.site(alt)
        speed = mach * c
        vb = body_velocity(alpha, beta, speed)
        vf = quat.rotate(self.rot, vb)
        wf = quat.rotate(self.rot, omega_body)
        force, torque = self.flight.simulate_aerodynamic_wrench_at(
            self.body, pos, vf, self.rot, wf, self.ut)
        q = 0.5 * rho * speed * speed
        inv = quat.conj(self.rot)
        fb = quat.rotate(inv, force)
        tb = quat.rotate(inv, torque)
        return [fb[0] / q, fb[1] / q, fb[2] / q, tb[0] / q, tb[1] / q, tb[2] / q]


def settle(conn, seconds):
    ut0 = conn.space_center.ut
    deadline = time.time() + 20
    while conn.space_center.ut - ut0 < seconds and time.time() < deadline:
        time.sleep(0.02)


def load(args, save):
    """Load and freeze at once: a save taken in the air falls for every
    wall-second it is left running, and the state is what the simulator
    starts from (the first version slept 3 s here and caught the booster
    2.5 km above its save)."""
    conn = connect(args.instance)
    try:
        conn.space_center.load(save)
    except Exception:                                   # noqa: BLE001
        pass
    for _ in range(50):
        try:
            conn.krpc.paused = True
            break
        except Exception:                               # noqa: BLE001
            try:
                conn.close()
            except Exception:                           # noqa: BLE001
                pass
            time.sleep(0.2)
            conn = connect(args.instance)
    last = None
    for attempt in range(40):
        try:
            vessel = conn.space_center.active_vessel
            model = probe_static(conn, vessel)
            return conn, vessel, model
        except Exception as exc:                        # noqa: BLE001
            # The scene is still settling; wait paused.
            last = exc
            time.sleep(0.5)
    raise last


SURFACE_LEVELS = [-1.0, -0.5, 0.5, 1.0]


def probe_surfaces(conn, vessel, probe, tables):
    """Each control surface's own wrench delta at fixed deflections.

    Axis inputs do not superpose (pitch +1 with roll +1 measured 45% of the
    sum of the two: each surface saturates or cancels on its own), so the
    simulator mixes inputs per surface the way KSP does and needs each
    surface's effect.  ``deflection_override`` holds a surface at a given
    fraction of its range; it needs game frames to move, so the game runs
    between settings and is paused for each sweep.
    """
    mach, alpha = tables["mach"], tables["alpha"]
    out = []
    for i, cs in enumerate(vessel.parts.control_surfaces):
        per = {}
        for d in SURFACE_LEVELS:
            conn.krpc.paused = False
            cs.deflection_override = True
            cs.deflection = d
            settle(conn, 0.8)
            conn.krpc.paused = True
            time.sleep(0.05)
            per[str(d)] = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
        conn.krpc.paused = False
        cs.deflection = 0.0
        cs.deflection_override = False
        settle(conn, 0.8)
        conn.krpc.paused = True
        time.sleep(0.05)
        out.append(per)
    zero = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
    return out, zero


SOLO_INPUTS = [("pitch", 1.0), ("pitch", -1.0), ("pitch", 0.5), ("pitch", -0.5),
               ("roll", 1.0), ("roll", -1.0), ("roll", 0.5), ("roll", -0.5),
               ("yaw", 1.0), ("yaw", -1.0), ("yaw", 0.5), ("yaw", -0.5)]


def probe_solo_surfaces(args, tables, main_model):
    """Each surface's response to real control input, alone.

    Every other surface has its pitch/yaw/roll axes switched off, so the
    measured delta is this surface's deflection under KSP's own mixing,
    clamping and authority limiter -- the quantity the simulator needs, which
    ``deflection_override`` (a deploy angle, on a different scale) is not.
    A fresh load per surface keeps the vessel high: it flies while each
    input settles.
    """
    mach, alpha = tables["mach"], tables["alpha"]
    out = []
    n = len(main_model["control_surfaces"])
    rm = root_position(main_model)
    for s in range(n):
        conn, vessel, model = load(args, args.controls_save)
        rc = root_position(model)
        offset = [rm[i] - rc[i] for i in range(3)]
        ctrl = vessel.control
        ctrl.sas = False
        surfaces = vessel.parts.control_surfaces
        for k, cs in enumerate(surfaces):
            on = (k == s)
            cs.pitch_enabled = on
            cs.yaw_enabled = on
            cs.roll_enabled = on
        probe = WrenchProbe(conn, vessel)

        def hold(axis, level):
            conn.krpc.paused = False
            ctrl.pitch = ctrl.yaw = ctrl.roll = 0.0
            setattr(ctrl, axis, level)
            settle(conn, 1.0)
            conn.krpc.paused = True
            time.sleep(0.05)

        hold("pitch", 0.0)
        zero = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
        resp = {}
        for axis, level in SOLO_INPUTS:
            hold(axis, level)
            resp["%s %s" % (axis, level)] = [[probe.wrench(m, a, 0.0) for a in alpha]
                                             for m in mach]
        conn.krpc.paused = True
        conn.close()
        out.append({"zero": zero, "responses": resp, "com_offset": offset})
        print("  surface %d of %d" % (s + 1, n), flush=True)
    return out


# Sideslips at which each surface's solo tables are taken again.  A deflected
# surface's moment depends on sideslip as well as on alpha: the shuttle's
# elevons at full nose-up input take its yaw stiffness at Mach 5, alpha 30
# from -2.92 to -0.58 N m/Pa/deg (the game's oracle), and tables taken at
# beta 0 alone left the simulator 4x too stiff in yaw through the whole
# hypersonic entry -- it lost attitude there and the game did not.
SOLO_BETAS = [-15.0, -4.0, 4.0, 15.0]


def dominant_axis(solo):
    """The axis a surface answers most on, from its beta-0 solo tables (the
    rule ``aero.Surface`` uses)."""
    def size(key):
        t, z = solo["responses"].get(key), solo["zero"]
        if t is None:
            return 0.0
        return sum((x - y) ** 2 for r1, r2 in zip(t, z) for c1, c2 in zip(r1, r2)
                   for x, y in zip(c1, c2))
    return max(("pitch", "roll", "yaw"), key=lambda ax: size(ax + " 1.0"))


def probe_solo_beta(args, tables, main_model):
    """Each surface's solo tables at ``SOLO_BETAS``: its zero and its
    dominant axis's levels only (the other axes only select a weight, which
    beta 0 settles).  Same procedure as ``probe_solo_surfaces``."""
    mach, alpha = tables["mach"], tables["alpha"]
    n = len(main_model["control_surfaces"])
    for s in range(n):
        solo = tables["solo"][s]
        dom = dominant_axis(solo)
        conn, vessel, model = load(args, args.controls_save)
        ctrl = vessel.control
        ctrl.sas = False
        for k, cs in enumerate(vessel.parts.control_surfaces):
            on = (k == s)
            cs.pitch_enabled = on
            cs.yaw_enabled = on
            cs.roll_enabled = on
        probe = WrenchProbe(conn, vessel)

        def hold(axis, level):
            conn.krpc.paused = False
            ctrl.pitch = ctrl.yaw = ctrl.roll = 0.0
            setattr(ctrl, axis, level)
            settle(conn, 1.0)
            conn.krpc.paused = True
            time.sleep(0.05)

        per = {str(b): {"responses": {}} for b in SOLO_BETAS}
        hold("pitch", 0.0)
        for b in SOLO_BETAS:
            per[str(b)]["zero"] = [[probe.wrench(m, a, b) for a in alpha] for m in mach]
        for axis, level in SOLO_INPUTS:
            if axis != dom:
                continue
            hold(axis, level)
            for b in SOLO_BETAS:
                per[str(b)]["responses"]["%s %s" % (axis, level)] = [
                    [probe.wrench(m, a, b) for a in alpha] for m in mach]
        conn.krpc.paused = True
        conn.close()
        solo["beta"] = per
        print("  surface %d of %d (%s) at beta %s" % (s + 1, n, dom, SOLO_BETAS), flush=True)


# Levels of each surface's dominant axis taken again, between the solo
# inputs' +-0.5 and +-1.  A deflected surface's drag grows as the square of
# its deflection and its lift saturates past half travel, so the chord from
# 0 to 0.5 is a poor stand-in for either: the shuttle trims its elevons at
# 0.17 on final, where that chord gave 4-5 m^2 of drag against a whole
# vehicle's 7-8 -- the simulated approach flew 30 m/s slow into the flare
# (a_qs_shuttle_final_1, 2026-09-24).
SOLO_EXTRA_LEVELS = [-0.75, -0.3, -0.15, -0.05, 0.05, 0.15, 0.3, 0.75]


def probe_solo_levels(args, tables, main_model):
    """Each surface's dominant axis at ``SOLO_EXTRA_LEVELS``, at beta 0 and at
    every sideslip its solo tables already have.  Same procedure as
    ``probe_solo_surfaces``; the deltas are against the zero tables already
    in the model (the same save, loaded afresh, is the same vessel)."""
    mach, alpha = tables["mach"], tables["alpha"]
    n = len(main_model["control_surfaces"])
    for s in range(n):
        solo = tables["solo"][s]
        dom = dominant_axis(solo)
        betas = sorted((solo.get("beta") or {}).keys(), key=float)
        conn, vessel, model = load(args, args.controls_save)
        ctrl = vessel.control
        ctrl.sas = False
        for k, cs in enumerate(vessel.parts.control_surfaces):
            on = (k == s)
            cs.pitch_enabled = on
            cs.yaw_enabled = on
            cs.roll_enabled = on
        probe = WrenchProbe(conn, vessel)
        for level in SOLO_EXTRA_LEVELS:
            conn.krpc.paused = False
            ctrl.pitch = ctrl.yaw = ctrl.roll = 0.0
            setattr(ctrl, dom, level)
            settle(conn, 1.0)
            conn.krpc.paused = True
            time.sleep(0.05)
            key = "%s %s" % (dom, level)
            solo["responses"][key] = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
            for b in betas:
                solo["beta"][b]["responses"][key] = [
                    [probe.wrench(m, a, float(b)) for a in alpha] for m in mach]
        conn.krpc.paused = True
        conn.close()
        print("  surface %d of %d (%s) at levels %s, betas 0 %s"
              % (s + 1, n, dom, SOLO_EXTRA_LEVELS, betas), flush=True)


AERO_SURFACE_ANGLES = [15.0, 35.0, 55.0, 70.0, 90.0, 105.0]


def probe_aero_surfaces(args, tables, main_model):
    """Each airbrake's (ModuleAeroSurface) wrench delta against its opening
    angle, measured alone.

    An airbrake is not mixed like a control surface: it opens to its deploy
    angle, and further -- one-sided, up to 1.5 x its range -- when opening
    helps a pitch or yaw input (ModuleAeroSurface.CtrlSurfaceUpdate,
    decompiled).  ``deflection_override`` cannot place it (it drives
    ``deployAngle``, which the airbrake ignores for its own
    ``aeroDeployAngle``), so each angle is set through the module's "Deploy
    Angle" field and the brake deployed on its own, on a fresh load of the
    in-air save; the zero table is the same load before anything moved.
    Returns [None or {"angles": [...], "tables": [...], "zero": ...,
    "com_offset": ...}] per control surface."""
    mach, alpha = tables["mach"], tables["alpha"]
    out = []
    rm = root_position(main_model)
    for s, rec in enumerate(main_model["control_surfaces"]):
        mods = [m["name"] for m in main_model["parts"][rec["part"]]["modules"]]
        if "ModuleAeroSurface" not in mods:
            out.append(None)
            continue
        conn, vessel, model = load(args, args.controls_save)
        rc = root_position(model)
        offset = [rm[i] - rc[i] for i in range(3)]
        ctrl = vessel.control
        ctrl.sas = False
        ctrl.brakes = False
        cs = vessel.parts.control_surfaces[s]
        module = next(m for m in cs.part.modules if m.name == "ModuleAeroSurface")
        probe = WrenchProbe(conn, vessel)
        zero = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
        rec_out = {"angles": [], "tables": [], "zero": zero, "com_offset": offset}
        for ang in AERO_SURFACE_ANGLES:
            try:
                module.set_field_float("Deploy Angle", ang)
            except Exception:                           # noqa: BLE001
                try:
                    module.set_field_string("Deploy Angle", str(ang))
                except Exception:                       # noqa: BLE001
                    continue
            cs.deployed = True
            conn.krpc.paused = False
            # MoveTowards at actuatorSpeed (20 deg/s on the stock airbrake).
            settle(conn, ang / 20.0 + 0.8)
            conn.krpc.paused = True
            time.sleep(0.05)
            rec_out["angles"].append(ang)
            rec_out["tables"].append([[probe.wrench(m, a, 0.0) for a in alpha] for m in mach])
        cs.deployed = False
        conn.close()
        out.append(rec_out)
        print("  airbrake %d: angles %s" % (s, rec_out["angles"]), flush=True)
    return out


def root_position(model):
    return model["parts"][model["vessel"]["root"] or 0]["position"]


def probe_terrain(conn, body, lat0=-1.5, lat1=1.0, lon0=-77.5, lon1=-72.0, step=0.025):
    nlat = int(round((lat1 - lat0) / step)) + 1
    nlon = int(round((lon1 - lon0) / step)) + 1
    heights = [[body.surface_height(lat0 + i * step, lon0 + j * step) for j in range(nlon)]
               for i in range(nlat)]
    return {"lat0": lat0, "dlat": step, "nlat": nlat, "lon0": lon0, "dlon": step,
            "nlon": nlon, "heights": heights}


def probe_controls(args, tables, main_model):
    """Control-surface deltas, measured on a save that is *in the air*.

    Surfaces do not answer pitch/yaw/roll input in orbit (measured: the
    orbital probe's control tables were identically zero), so the vessel
    has to be flying.  Each input is held while the game runs until the
    surfaces reach it, then the game is paused and the table swept at that
    deflection -- the oracle's arguments do not depend on where the vessel
    actually is.
    """
    conn, vessel, model = load(args, args.controls_save)
    ctrl = vessel.control
    ctrl.sas = False
    probe = WrenchProbe(conn, vessel)
    mach, alpha = tables["mach"], tables["alpha"]
    # Centre of mass of this session relative to the main model's origin.
    rm, rc = root_position(main_model), root_position(model)
    com_offset = [rm[i] - rc[i] for i in range(3)]

    def hold(**inputs):
        conn.krpc.paused = False
        for k, v in inputs.items():
            setattr(ctrl, k, v)
        settle(conn, 1.2)
        conn.krpc.paused = True
        time.sleep(0.1)

    hold(pitch=0.0, yaw=0.0, roll=0.0)
    zero = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
    controls = {}
    for axis in ("pitch", "yaw", "roll"):
        controls[axis] = {}
        for level in LEVELS:
            hold(**{axis: level})
            controls[axis][str(level)] = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
        hold(**{axis: 0.0})
    # A combined input, to test whether per-axis deltas add.
    hold(pitch=1.0, roll=1.0)
    combined = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
    hold(pitch=0.0, roll=0.0)

    # Actuator dynamics: a pitch step, the oracle sampled as the game runs.
    step = []
    conn.krpc.paused = False
    settle(conn, 0.5)
    ut0 = conn.space_center.ut
    ctrl.pitch = 1.0
    while conn.space_center.ut - ut0 < 2.0:
        step.append([conn.space_center.ut - ut0, probe.wrench(0.7, 5.0, 0.0)[3]])
    ctrl.pitch = 0.0
    settle(conn, 1.0)

    conn.close()

    # Gear down, on a fresh load: by now the vessel has flown twenty-odd
    # seconds of stepped inputs, and qs_shuttle_final (saved near the stall
    # at 3 km) was no longer the vessel it had been -- its gear table took
    # 270 kN of lift away.  The gear table is a delta against a zero table
    # taken in the same load.
    conn, vessel, _ = load(args, args.controls_save)
    ctrl = vessel.control
    ctrl.sas = False
    probe = WrenchProbe(conn, vessel)
    gear_zero = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
    conn.krpc.paused = False
    ctrl.gear = True
    settle(conn, 4.0)
    conn.krpc.paused = True
    time.sleep(0.1)
    vframe = vessel.reference_frame
    wheels = [[list(v) for v in w.part.bounding_box(vframe)] for w in vessel.parts.wheels]
    gear_down = [[probe.wrench(m, a, 0.0) for a in alpha] for m in mach]
    conn.close()
    return {"zero": zero, "controls": controls, "combined_pitch1_roll1": combined,
            "com_offset": com_offset, "pitch_step": step,
            "deployed_wheel_boxes": wheels, "gear_down": gear_down, "gear_zero": gear_zero,
            "save": args.controls_save}


def physics_curves():
    """The game's Physics.cfg as ModuleManager left it: its curves and
    scalars (the drag's pseudo-Reynolds multiplier, the thermal constants)."""
    from kspSim.curves import parse_cfg_curves
    for n in range(10):
        path = os.path.join(ROOT, "testInstances", "ksp%d" % n, "GameData", "ModuleManager.Physics")
        if os.path.exists(path):
            break
    else:
        path = os.path.join(ROOT, "testInstances", "base", "Physics.cfg")
    text = open(path, encoding="utf-8", errors="replace").read()
    scalars = {}
    for raw in text.splitlines():
        line = raw.split("//")[0].strip()
        if "=" in line and not line.startswith("key"):
            k, v = line.split("=", 1)
            scalars[k.strip()] = v.strip()
    return {"curves": parse_cfg_curves(text), "scalars": scalars}


RE_ALTITUDES = [a * 1000.0 for a in range(0, 66, 2)]


def probe_reynolds(probe, mach, alpha, beta, physics):
    """The base table again, each Mach at a second density.

    KSP multiplies drag-cube drag (and nothing else) by
    DRAG_PSEUDOREYNOLDS(density x speed): 1.0 between 1 and 100, 0.82 at
    200, above 1 in thin air.  One table at one altitude per Mach bakes in
    that altitude's multiplier -- qs_shuttle_cone flies Mach 0.75 at 10 km
    (x1.00) on a table taken at 4 km (x0.86), and the simulator's axial
    force was 15% light.  Two tables at different multipliers separate the
    two parts exactly: cube = (b1 - b2) / (m1 - m2)."""
    from kspSim.curves import FloatCurve
    curve = FloatCurve(physics["curves"]["DRAG_PSEUDOREYNOLDS"])
    re1, re2, alt2 = [], [], []
    for m in mach:
        _, rho, _, c = probe.site(altitude_for_mach(m))
        r1 = rho * m * c
        best = None
        for alt in RE_ALTITUDES:
            _, rho2, _, c2 = probe.site(alt)
            r = rho2 * m * c2
            gap = abs(curve(r) - curve(r1))
            if best is None or gap > best[0] + 1e-6:
                best = (gap, alt, r)
        re1.append(r1)
        re2.append(best[2])
        alt2.append(best[1])
    base2 = [[[probe.wrench(m, a, b, alt=alt2[i]) for b in beta] for a in alpha]
             for i, m in enumerate(mach)]
    return {"re": re1, "re2": re2, "altitude2": alt2, "base2": base2}


def craft_layout(model):
    """[(name, position relative to the root part)] -- the craft, whatever the
    save's fuel and centre of mass."""
    root = model["parts"][model["vessel"]["root"] or 0]["position"]
    return [(p["name"], [p["position"][k] - root[k] for k in range(3)]) for p in model["parts"]]


def same_craft(a, b, tol=0.1):
    """None if the two models are the same craft (every part within ``tol``
    metres of a same-named part), else the parts that are not.  Two variants
    of the old spaceplane share a name and a part list and differ by 0.73 m
    of wing height (qs_plane vs qs_cone), and the first probe gave one of
    them the other's tables."""
    import math
    lb = craft_layout(b)
    used = set()
    bad = []
    for name, pa in craft_layout(a):
        best = None
        for j, (nb, pb) in enumerate(lb):
            if nb == name and j not in used:
                d = math.dist(pa, pb)
                if best is None or d < best[0]:
                    best = (d, j)
        if best is None or best[0] > tol:
            bad.append((name, None if best is None else round(best[0], 2)))
        if best is not None:
            used.add(best[1])
    return bad or None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", default="1")
    ap.add_argument("--save", required=True)
    ap.add_argument("--controls-save", default=None,
                    help="a save of the same craft in the air, for control deltas")
    ap.add_argument("--quick", action="store_true", help="coarse tables, for testing")
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--force", action="store_true", help="--aero-from even across craft")
    ap.add_argument("--airbrakes-only", action="store_true",
                    help="add only the airbrake angle tables to the existing model")
    ap.add_argument("--solo-beta", action="store_true",
                    help="add only the per-surface sideslip tables to the existing "
                         "model (needs --controls-save) and to its --aero-from siblings")
    ap.add_argument("--solo-levels", action="store_true",
                    help="add only the per-surface tables at SOLO_EXTRA_LEVELS to the "
                         "existing model (needs --controls-save) and to its --aero-from siblings")
    ap.add_argument("--no-augment", action="store_true")
    ap.add_argument("--aero-from", default=None,
                    help="reuse the tables of this model (same craft): probe only the state")
    args = ap.parse_args()
    os.makedirs(MODELS, exist_ok=True)
    t0 = time.time()
    if args.airbrakes_only:
        path = os.path.join(MODELS, args.save + ".json")
        model = json.load(open(path))
        model["aero"]["aero_surfaces"] = probe_aero_surfaces(args, model["aero"], model)
        with open(path, "w") as fh:
            json.dump(model, fh)
        print("wrote airbrake tables to %s  %.1fs" % (path, time.time() - t0))
        return
    if args.solo_beta or args.solo_levels:
        if not args.controls_save:
            raise SystemExit("--solo-beta/--solo-levels need --controls-save (an air save "
                             "of the craft)")
        path = os.path.join(MODELS, args.save + ".json")
        model = json.load(open(path))
        if args.solo_beta:
            probe_solo_beta(args, model["aero"], model)
        if args.solo_levels:
            probe_solo_levels(args, model["aero"], model)
        with open(path, "w") as fh:
            json.dump(model, fh)
        print("wrote solo tables to %s  %.1fs" % (path, time.time() - t0))
        for name in sorted(os.listdir(MODELS)):
            if not name.endswith(".json") or name == args.save + ".json":
                continue
            sp = os.path.join(MODELS, name)
            sib = json.load(open(sp))
            if sib.get("aero_from") == args.save:
                sib["aero"] = model["aero"]
                with open(sp, "w") as fh:
                    json.dump(sib, fh)
                print("  and to %s" % name)
        return
    conn, vessel, model = load(args, args.save)
    model["save"] = args.save
    model["physics"] = physics_curves()
    model["atmosphere"] = probe_atmosphere(conn, vessel.orbit.body, model["ut"])
    print("static %.1fs" % (time.time() - t0), flush=True)
    if args.aero_from:
        # Same craft, another state: the tables are about the source save's
        # centre of mass, which sits at ``aero_origin`` in this save's frame
        # (the root part is the fixed point both frames can see).
        src = json.load(open(os.path.join(MODELS, args.aero_from + ".json")))
        bad = same_craft(model, src)
        if bad and not args.force:
            conn.close()
            raise SystemExit("%s is not the craft of %s (%s): probe it in full"
                             % (args.save, args.aero_from, bad[:4]))
        rs, rn = root_position(src), root_position(model)
        origin = [rn[i] - rs[i] for i in range(3)]
        model["aero"] = src["aero"]
        model["aero_origin"] = origin
        model["aero_from"] = args.aero_from
        model["terrain"] = src.get("terrain")
        for w, ws in zip(model["wheels"], src["wheels"]):
            if "deployed_bounding_box" in ws:
                w["deployed_bounding_box"] = [[ws["deployed_bounding_box"][k][i] + origin[i]
                                               for i in range(3)] for k in range(2)]
        conn.close()
        path = os.path.join(MODELS, args.save + ".json")
        with open(path, "w") as fh:
            json.dump(model, fh)
        print("wrote %s (tables from %s)  %.1fs" % (path, args.aero_from, time.time() - t0))
        finish(args)
        return
    if not args.no_terrain:
        model["terrain"] = probe_terrain(conn, vessel.orbit.body)
        print("terrain %.1fs" % (time.time() - t0), flush=True)

    mach = MACH[::3] if args.quick else MACH
    alpha = ALPHA[::3] if args.quick else ALPHA
    beta = BETA[::2] if args.quick else BETA
    probe = WrenchProbe(conn, vessel)
    tables = {"mach": mach, "alpha": alpha, "beta": beta,
              "altitude": [altitude_for_mach(m) for m in mach],
              "sites": {}}
    tables["base"] = [[[probe.wrench(m, a, b) for b in beta] for a in alpha] for m in mach]
    print("base %.1fs" % (time.time() - t0), flush=True)
    for alt, (pos, rho, temp, c) in probe.cache.items():
        tables["sites"][str(int(alt))] = {"density": rho, "temperature": temp, "sound": c}
    tables["speed"] = [m * probe.site(altitude_for_mach(m))[3] for m in mach]
    tables.update(probe_reynolds(probe, mach, alpha, beta, model["physics"]))
    print("second Reynolds number %.1fs" % (time.time() - t0), flush=True)
    damping = {}
    for i, axis in enumerate(("x", "y", "z")):
        w = [0.0, 0.0, 0.0]
        w[i] = OMEGA
        damping[axis] = [[probe.wrench(m, a, 0.0, tuple(w)) for a in alpha] for m in mach]
    tables["damping"] = damping
    tables["omega"] = OMEGA
    print("damping %.1fs" % (time.time() - t0), flush=True)
    if vessel.parts.control_surfaces:
        tables["surfaces"], tables["surfaces_zero"] = probe_surfaces(conn, vessel, probe, tables)
        tables["surface_levels"] = SURFACE_LEVELS
        print("surfaces %.1fs" % (time.time() - t0), flush=True)
    conn.close()

    if args.controls_save:
        c = probe_controls(args, tables, model)
        tables["controls"] = c.pop("controls")
        tables["controls_zero"] = c.pop("zero")
        tables["gear_down"] = c.pop("gear_down")
        tables["gear_zero"] = c.pop("gear_zero")
        tables["controls_meta"] = c
        for w, box in zip(model["wheels"], c["deployed_wheel_boxes"]):
            w["deployed_bounding_box"] = [[box[k][i] + c["com_offset"][i] for i in range(3)]
                                          for k in range(2)]
        print("controls %.1fs" % (time.time() - t0), flush=True)
        tables["solo"] = probe_solo_surfaces(args, tables, model)
        print("solo surfaces %.1fs" % (time.time() - t0), flush=True)
        probe_solo_beta(args, tables, model)
        print("solo surfaces at sideslip %.1fs" % (time.time() - t0), flush=True)
        brakes = probe_aero_surfaces(args, tables, model)
        if any(b is not None for b in brakes):
            tables["aero_surfaces"] = brakes
            print("airbrakes %.1fs" % (time.time() - t0), flush=True)
    model["aero"] = tables
    path = os.path.join(MODELS, args.save + ".json")
    with open(path, "w") as fh:
        json.dump(model, fh)
    print("wrote %s  %.1fs" % (path, time.time() - t0))
    finish(args)


def finish(args):
    """Thruster geometry, live nozzles and part configs (``augment.py``)."""
    if args.no_augment:
        return
    from kspSim import partcfg
    from kspSim.tools import augment
    pc = partcfg.PartConfigs(partcfg.default_cache(ROOT))
    augment.augment(args.instance, args.save, pc, partcfg.Variants(pc, augment.GAMEDATA))
    path = os.path.join(MODELS, args.save + ".json")
    model = json.load(open(path))
    augment.offline(model, pc, args.save)
    with open(path, "w") as fh:
        json.dump(model, fh)


if __name__ == "__main__":
    main()
