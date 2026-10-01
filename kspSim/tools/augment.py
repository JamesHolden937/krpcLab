#!/usr/bin/env python3
"""Add to an existing model what the first probe did not record.

    ./kspSim/tools/augment.py --instance 0 qs_shuttle [qs_plane ...]

- **Thruster geometry** of every RCS module and engine, read after the scene
  has run for a moment: right after a load ``Thruster.thrust_direction``
  throws, and the first probe fell back to the *part's* axis for every
  nozzle (every RCS nozzle of qs_shuttle pointed along +y).
- **Which nozzles are live.**  A part with variants (ReStock's RV-105 block
  lists all eighteen nozzles of its five variants) fires only the ones under
  the selected variant's GameObjects: the save names the variant, the
  config names the objects, the ``.mu`` model says which nozzle sits under
  which (``partcfg.Variants``).
- **Part configs** (``partcfg``), as the game loaded them: actuator and
  response speeds, gimbal ranges, RCS power and Isp curve, and each part's
  thermal fields.

Models made with ``--aero-from`` share the craft but not the state, and are
augmented on their own save.  Parts are matched by index; the part names of
the model and the live vessel must agree, or nothing is written.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

from kspSim import partcfg  # noqa: E402
from kspSim.tools import flighttest  # noqa: E402

MODELS = os.path.join(ROOT, "kspSim", "models")
GAMEDATA = os.path.join(ROOT, "testInstances", "base", "GameData")

# Module fields the simulator reads, per module name.
KEEP = {
    "ModuleControlSurface": ("actuatorSpeed", "ctrlSurfaceRange", "deflectionLiftCoeff",
                             "ignorePitch", "ignoreYaw", "ignoreRoll", "deployAngle"),
    "SyncModuleControlSurface": ("actuatorSpeed", "ctrlSurfaceRange", "deflectionLiftCoeff"),
    "ModuleAeroSurface": ("actuatorSpeed", "ctrlSurfaceRange", "deflectionLiftCoeff",
                          "ignorePitch", "ignoreYaw", "ignoreRoll"),
    "ModuleReactionWheel": ("PitchTorque", "YawTorque", "RollTorque", "torqueResponseSpeed"),
    "ModuleGimbal": ("gimbalRange", "gimbalRangeXP", "gimbalRangeXN", "gimbalRangeYP",
                     "gimbalRangeYN", "gimbalResponseSpeed", "useGimbalResponseSpeed",
                     "enableRoll", "gimbalTransformName"),
    "ModuleRCSFX": ("thrusterPower", "fullThrust", "fullThrustMin", "useZaxis",
                    "thrusterTransformName", "enableX", "enableY", "enableZ",
                    "enablePitch", "enableYaw", "enableRoll", "useThrottle"),
    "ModuleRCS": ("thrusterPower", "fullThrust", "fullThrustMin", "useZaxis",
                  "thrusterTransformName"),
    "ModuleCommand": ("minimumCrew",),
    "ModuleGenerator": ("isAlwaysActive",),
    "ModuleAlternator": ("preferMultiMode",),
    "ModuleEngines": ("maxThrust", "minThrust", "engineAccelerationSpeed",
                      "engineDecelerationSpeed", "useEngineResponseTime", "thrustVectorTransformName"),
    "ModuleEnginesFX": ("maxThrust", "minThrust", "engineAccelerationSpeed",
                        "engineDecelerationSpeed", "useEngineResponseTime",
                        "thrustVectorTransformName"),
}
CURVES = ("atmosphereCurve",)
PART_FIELDS = ("mass", "maxTemp", "skinMaxTemp", "emissiveConstant", "thermalMassModifier",
               "skinMassPerArea", "skinThermalMassModifier", "skinInternalConductionMult",
               "heatConductivity", "radiatorHeadroom", "absorptiveConstant",
               "heatConvectiveConstant", "skinSkinConductionMult", "crashTolerance",
               "PhysicsSignificance", "dragModelType", "maximum_drag", "minimum_drag",
               "angularDrag", "CoMOffset", "CoLOffset", "CoPOffset")


def cfg_summary(pc, name):
    mods = pc.modules(name)
    if mods is None:
        return None
    out = {"part": {k: v for k, v in (pc.fields(name) or {}).items() if k in PART_FIELDS},
           "modules": []}
    for m in mods:
        keep = KEEP.get(m["name"])
        if keep is None:
            continue
        rec = {"name": m["name"]}
        rec.update({k: m[k] for k in keep if k in m})
        # What the module draws and makes, per second (RESOURCE /
        # OUTPUT_RESOURCE): a reaction wheel's charge at full input, a
        # probe core's, an RTG's output, an alternator's at full thrust.
        for node, key in (("RESOURCE", "inputs"), ("OUTPUT_RESOURCE", "outputs")):
            got = [{"name": r.get("name"), "rate": float(r.get("rate", 0.0))}
                   for r in m.get("_nodes", {}).get(node, []) if r.get("name")]
            if got:
                rec[key] = got
        for curve in CURVES:
            keys = curve_keys(pc, name, m["name"], curve)
            if keys:
                rec[curve] = keys
        out["modules"].append(rec)
    return out


def curve_keys(pc, name, module, curve):
    """All ``key`` lines of a module's curve node (values() keeps only the first)."""
    node = pc.part(name)
    for m in node["nodes"] if node else []:
        if m["name"] != "MODULE" or partcfg.values(m).get("name") != module:
            continue
        for c in m["nodes"]:
            if c["name"] == curve:
                return [[float(x) for x in v.split()] for k, v in c["values"] if k == "key"]
    return None


def thrusters(obj, vf, tries=20):
    """[(position, direction)] of a module's thrusters, retrying while the
    scene is still building them."""
    last = None
    for _ in range(tries):
        try:
            return [(list(t.thrust_position(vf)), list(t.thrust_direction(vf)))
                    for t in obj.thrusters]
        except Exception as exc:                        # noqa: BLE001
            last = exc
            time.sleep(0.25)
    raise RuntimeError("thrusters not readable: %s" % last)


def sibling_geometry(model, save):
    """Thruster geometry of the same craft from another augmented model, moved
    to this model's part positions: {(kind, part index): [(pos, dir)]}.

    Some modules' thruster transforms are stale in kRPC after a load (every
    one of qs_shuttle's pod and first two RCS blocks throws in
    ``Transform.get_position``, while qs_shuttle_final's same parts read
    fine); a nozzle's offset from its own part does not depend on the save."""
    names = [p["name"] for p in model["parts"]]
    for fn in sorted(os.listdir(MODELS)):
        if not fn.endswith(".json") or fn[:-5] == save or fn.startswith("_"):
            continue
        try:
            other = json.load(open(os.path.join(MODELS, fn)))
        except (OSError, ValueError):
            continue
        if not other.get("augmented") or [p["name"] for p in other["parts"]] != names:
            continue
        out = {}
        for kind in ("rcs", "engines"):
            for rec in other[kind]:
                i = rec["part"]
                src = other["parts"][i]["position"]
                dst = model["parts"][i]["position"]
                out[(kind, i)] = [([t["position"][k] - src[k] + dst[k] for k in range(3)],
                                   t["direction"]) for t in rec["thrusters"]]
        return out, fn[:-5]
    return None, None


def augment(instance, save, pc, variants):
    path = os.path.join(MODELS, save + ".json")
    model = json.load(open(path))
    sibling, sibling_name = sibling_geometry(model, save)
    conn = flighttest.load_paused(instance, save)
    try:
        # Let the scene finish building (thrusters throw until it has).
        conn.krpc.paused = False
        time.sleep(1.5)
        conn.krpc.paused = True
        v = conn.space_center.active_vessel
        vf = v.reference_frame
        parts = v.parts.all
        names = [p.name for p in parts]
        if names != [p["name"] for p in model["parts"]]:
            raise SystemExit("%s: part list differs from the model's" % save)
        index = {p._object_id: i for i, p in enumerate(parts)}
        sfs = partcfg.sfs_active_parts(os.path.join(ROOT, "saves", save + ".sfs"))
        if sfs is not None and [n for n, _ in sfs] != names:
            print("%s: save part order differs from the game's; variants from the base" % save)
            sfs = None

        def variant_of(i):
            if sfs is None:
                return None
            return sfs[i][1].get("ModulePartVariants", {}).get("selectedVariant")

        rcs = []
        for r in v.parts.rcs:
            i = index[r.part._object_id]
            try:
                geo = thrusters(r, vf, tries=4)
            except RuntimeError:
                if not sibling or ("rcs", i) not in sibling:
                    raise
                print("%s: part %d's nozzles from %s" % (save, i, sibling_name))
                geo = sibling[("rcs", i)]
            mods = {m["name"]: m for m in pc.modules(names[i]) or []}
            m = mods.get("ModuleRCSFX") or mods.get("ModuleRCS") or {}
            mask = variants.live_mask(names[i], variant_of(i),
                                      m.get("thrusterTransformName", "RCSthruster"), len(geo))
            rec = {"part": i, "enabled": r.enabled,
                   "max_vacuum_thrust": r.max_vacuum_thrust,
                   "vacuum_isp": r.vacuum_specific_impulse,
                   "sea_level_isp": r.kerbin_sea_level_specific_impulse,
                   "propellants": list(r.propellants),
                   "pitch": r.pitch_enabled, "yaw": r.yaw_enabled, "roll": r.roll_enabled,
                   "forward": r.forward_enabled, "up": r.up_enabled, "right": r.right_enabled,
                   "thrust_limit": r.thrust_limit,
                   "thrusters": [{"position": p, "direction": d} for p, d in geo],
                   "live": mask if mask is not None else [True] * len(geo),
                   "variant": variant_of(i),
                   "full_thrust": str(m.get("fullThrust", "false")).lower() == "true",
                   "full_thrust_min": float(m.get("fullThrustMin", 0.2)),
                   "isp_curve": curve_keys(pc, names[i], m.get("name", "ModuleRCSFX"),
                                           "atmosphereCurve")}
            rcs.append(rec)
        model["rcs"] = rcs

        for e, rec in zip(v.parts.engines, model["engines"]):
            i = index[e.part._object_id]
            if rec["part"] != i:
                raise SystemExit("%s: engine order differs" % save)
            try:
                geo = thrusters(e, vf, tries=4)
            except RuntimeError:
                if not sibling or ("engines", i) not in sibling:
                    raise
                geo = sibling[("engines", i)]
            rec["thrusters"] = [{"position": p, "direction": d} for p, d in geo]
            # The gimbal turns the nozzle about this point, so the thrust's
            # lever for gimbal torque is the pivot's, not the nozzle's
            # (qs_plane: pivot -1.78 m, nozzle -3.50; the game's gimbal
            # torque over its lateral force measured 1.78).
            for th, t in zip(rec["thrusters"], e.thrusters):
                try:
                    th["gimbal_position"] = list(t.gimbal_position(vf))
                except Exception:                       # noqa: BLE001
                    pass
            mods = {m["name"]: m for m in pc.modules(names[i]) or []}
            g = mods.get("ModuleGimbal")
            if g:
                rec["gimbal"] = {"range": float(g.get("gimbalRange", rec.get("gimbal_range", 0.0))),
                                 "response_speed": float(g.get("gimbalResponseSpeed", 10.0)),
                                 "use_response": str(g.get("useGimbalResponseSpeed",
                                                           "false")).lower() == "true",
                                 "limit": 1.0}
                try:
                    rec["gimbal"]["limit"] = e.gimbal_limit
                    rec["gimbal_locked"] = e.gimbal_locked
                except Exception:                       # noqa: BLE001
                    pass
            eng = mods.get("ModuleEnginesFX") or mods.get("ModuleEngines") or {}
            rec["response"] = {
                "use": str(eng.get("useEngineResponseTime", "false")).lower() == "true",
                "up": float(eng.get("engineAccelerationSpeed", 10.0)),
                "down": float(eng.get("engineDecelerationSpeed", 10.0))}

        model["partcfg"] = {n: cfg_summary(pc, n) for n in sorted(set(names))}
        model["augmented"] = time.time()
    finally:
        conn.close()
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(model, fh)
    os.replace(tmp, path)
    live = sum(sum(r["live"]) for r in model["rcs"])
    total = sum(len(r["live"]) for r in model["rcs"])
    print("%s: %d RCS modules, %d of %d nozzles live; %d engines" % (
        save, len(model["rcs"]), live, total, len(model["engines"])))


def pick_cube(found, mods, cfg_modules):
    """The cube KSP weights fully for the part's state in the save: a
    variant's cube is named by its index in the VARIANT list (``0``, ``1``,
    ...), gear by ``Retracted``/``Deployed``, an animation's ends ``B``
    (time 0) and ``A`` (time 1), control surfaces ``neutral``.  The first version
    took the first cube: a retracted gear bay flew with its deployed cube,
    and an engine with another variant's shroud."""
    if len(found) == 1:
        return next(iter(found.values()))
    variant = (mods.get("ModulePartVariants") or {}).get("selectedVariant")
    listed = [v.get("name") for m in cfg_modules if m.get("name") == "ModulePartVariants"
              for v in (m.get("_nodes") or {}).get("VARIANT", [])]
    if listed:
        k = listed.index(variant) if variant in listed else 0
        if str(k) in found:
            return found[str(k)]
    if variant in found:
        return found[variant]
    if "Retracted" in found and "Deployed" in found:
        dep = mods.get("ModuleWheelDeployment") or {}
        try:
            out = float(dep.get("position", 0.0)) > 0.5
        except ValueError:
            out = False
        return found["Deployed" if out or dep.get("stateString") == "Deployed" else "Retracted"]
    if "A" in found and "B" in found:
        try:
            t = float((mods.get("ModuleAnimateGeneric") or {}).get("animTime", 0.0))
        except ValueError:
            t = 0.0
        # ModuleAnimateGeneric.SetDragState(animTime): A weighs animTime, B
        # the rest -- a closed bay or shielded port (time 0) flies B.
        return found["A" if t >= 0.5 else "B"]
    for key in ("neutral", "Default"):
        if key in found:
            return found[key]
    return next(iter(found.values()))


def offline(model, pc, save):
    """What needs no game: part configs and drag cubes (``PartDatabase.cfg``,
    the selected variant's cube where a part has several)."""
    names = [p["name"] for p in model["parts"]]
    model["partcfg"] = {n: cfg_summary(pc, n) for n in sorted(set(names))}
    db = None
    for n in range(10):
        path = os.path.join(ROOT, "testInstances", "ksp%d" % n, "PartDatabase.cfg")
        if os.path.exists(path):
            db = partcfg.drag_cubes(path)
            break
    if db is None:
        return
    sfs = partcfg.sfs_active_parts(os.path.join(ROOT, "saves", save + ".sfs"))
    cubes = {}
    for i, name in enumerate(names):
        found = db.get(pc.url(name) or "", {})
        if not found:
            continue
        mods = sfs[i][1] if sfs is not None and i < len(sfs) and sfs[i][0] == name else {}
        cubes.setdefault(name, pick_cube(found, mods, pc.modules(name) or []))
    model["dragcubes"] = cubes
    # Action-group bindings from the save (the booster's grid fins deploy
    # and become pitch/yaw surfaces on Custom02).
    actions = []
    if sfs is not None and [n for n, _ in sfs] == names:
        for i, (_, mods) in enumerate(sfs):
            for mod_name, m in mods.items():
                for act, groups in (m.get("_actions") or {}).items():
                    if groups:
                        actions.append({"part": i, "module": mod_name, "action": act,
                                        "groups": groups})
    model["actions"] = actions
    # The thermal graph (``thermal.py``): which node of which part touches
    # which part, the nodes' orientations, and the resources' specific heats.
    att = partcfg.sfs_attachments(os.path.join(ROOT, "saves", save + ".sfs"))
    if att is not None and [a["name"] for a in att] == names:
        model["attach"] = [{"att": a["att"], "srf": a["srf"]} for a in att]
    model["nodes"] = {n: pc.nodes(n) for n in sorted(set(names))}
    model["resource_defs"] = {k: {kk: float(vv) for kk, vv in v.items()}
                              for k, v in pc.resources.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", default=None)
    ap.add_argument("--offline", action="store_true",
                    help="only part configs and drag cubes; no game needed")
    ap.add_argument("saves", nargs="+")
    args = ap.parse_args()
    cache = partcfg.default_cache(ROOT)
    pc = partcfg.PartConfigs(cache)
    variants = partcfg.Variants(pc, GAMEDATA)
    for save in args.saves:
        if not args.offline:
            augment(args.instance, save, pc, variants)
        path = os.path.join(MODELS, save + ".json")
        model = json.load(open(path))
        offline(model, pc, save)
        with open(path + ".tmp", "w") as fh:
            json.dump(model, fh)
        os.replace(path + ".tmp", path)
        print("%s: %d drag cubes" % (save, len(model.get("dragcubes", {}))))


if __name__ == "__main__":
    main()
