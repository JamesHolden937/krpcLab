#!/usr/bin/env python3
"""The thermal model against a game flight's temperatures.

    ./kspSim/tools/thermcheck.py logs/kspsim/obs_entry1.jsonl --model qs_entry [--segment N]

Reads an ``observe.py`` recording (body-frame position, velocity, rotation
and every part's skin and internal temperature, about once a second) and
replays the recorded trajectory through ``kspSim.thermal`` -- the whole
vessel at once, since its parts conduct to each other and shade each other
from the flow -- open loop in the temperatures, the flight itself taken
from the game.  Nothing is fitted: this is the port of KSP's
FlightIntegrator scored against KSP.  Per part: the rms and peak error of
the exposed skin and of the interior, and the peak skin each reached.

The recording does not carry the propellant: parts start with the model's
resources, so a tank drained before the recording starts reads a thermal
mass too high in the interior (its skin is barely affected).
"""
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from kspSim import quat, vec  # noqa: E402
from kspSim import world as W  # noqa: E402
from kspSim.aero import Aero  # noqa: E402
from kspSim.thermal import Thermal  # noqa: E402

STEP = 0.1        # s: the simulator's thermal step (world.THERMAL_EVERY x 0.02)


def segments(path):
    segs, cur = [], None
    for line in open(path):
        r = json.loads(line)
        if "new_vessel" in r:
            cur = {"parts": r["new_vessel"], "rows": []}
            segs.append(cur)
        elif cur is not None and "skin" in r:
            cur["rows"].append(r)
    return segs


def environment(body, r):
    rb = tuple(r["pos"])
    vb = tuple(r["vel"])
    alt, p, temp, rho, sound = body.air(rb, r["ut"])
    qbf = quat.norm(tuple(r["rot"]))
    v_body = quat.rotate(quat.conj(qbf), vb)
    _, _, speed = Aero.angles(v_body)
    return {"r_bf": rb, "q_bf": qbf, "alt": alt, "pressure": p, "temperature": temp,
            "density": rho, "speed": speed, "mach": speed / sound if sound > 0 else 0.0,
            "v_body": v_body}


def replay(model, rows):
    """Every part's (skin, internal) at every row."""
    world = W.World()
    world.t = rows[0]["ut"]
    world.body = W.Body(model)
    world.vessel = W.Vessel(model, world)
    th = Thermal(world.vessel, model)
    for k, pt in enumerate(th.parts):
        pt.skin = pt.skin_unexp = rows[0]["skin"][k]
        pt.internal = rows[0]["temp"][k]
    out = [[(pt.skin, pt.internal) for pt in th.parts]]
    for i in range(len(rows) - 1):
        dt = rows[i + 1]["ut"] - rows[i]["ut"]
        if dt > 0:
            env = environment(world.body, rows[i])
            n = max(1, int(math.ceil(dt / STEP)))
            for _ in range(n):
                world.t += dt / n
                th.step(dt / n, env)
        out.append([(pt.skin, pt.internal) for pt in th.parts])
    return th, out


FLUXES = (("thermal_convection_flux", "q_conv"), ("thermal_radiation_flux", "q_rad"),
          ("thermal_conduction_flux", "q_cond"), ("thermal_skin_to_internal_flux", "q_skin_int"))


def flux_check(model, rows):
    """Term by term at the game's own temperatures: each recorded row, the
    model's temperatures are set to the game's (both skins to the reported
    one), one step of zero length is taken, and its flows are compared with
    the game's.  Also the thermal masses."""
    rows = [r for r in rows if "thermal_convection_flux" in r]
    if not rows:
        print("no fluxes in this recording (observe.py --fluxes)")
        return
    world = W.World()
    world.t = rows[0]["ut"]
    world.body = W.Body(model)
    world.vessel = W.Vessel(model, world)
    th = Thermal(world.vessel, model)
    acc = [{} for _ in th.parts]
    for r in rows:
        world.t = r["ut"]
        env = environment(world.body, r)
        for k, pt in enumerate(th.parts):
            pt.skin = pt.skin_unexp = r["skin"][k]
            pt.internal = r["temp"][k]
        th.step(1e-9, env)
        for k, pt in enumerate(th.parts):
            a = acc[k]
            for key, attr in FLUXES:
                g, s_ = r[key][k] * 1e3, getattr(pt, attr, 0.0)    # kRPC: MW
                a.setdefault(attr, []).append((g, s_))
            # kRPC reports the masses in MJ/K (KSP keeps kJ/K).
            a.setdefault("mass", []).append((r["thermal_mass"][k] * 1e3, pt.int_mass,
                                             r["thermal_skin_mass"][k] * 1e3, pt.skin_mass))
    print("%-3s %-22s %-19s %-19s %-19s %-19s %s" % ("", "part", "convection kW", "radiation kW",
                                                     "conduction kW", "skin->int kW",
                                                     "mass int / skin (game, sim)"))
    for k, pt in enumerate(th.parts):
        a = acc[k]
        cells = []
        for _, attr in FLUXES:
            pairs = a[attr]
            peak = max(pairs, key=lambda p: abs(p[0]))
            # The integral over the flight, and the value at the game's peak.
            gi, si = sum(p[0] for p in pairs), sum(p[1] for p in pairs)
            cells.append("%7.2f/%7.2f %3.0f%%" % (peak[0], peak[1],
                                                  100.0 * si / gi if abs(gi) > 1e-9 else 0.0))
        m = a["mass"][len(a["mass"]) // 2]
        print("%-3d %-22s %s  %6.0f/%6.0f %5.1f/%5.1f" % (k, pt.part.name[:22], " ".join(cells),
                                                         m[0], m[1], m[2], m[3]))
    print("cells: game/sim at the game's peak, and the sim's integral as a share of the game's")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--model", required=True)
    ap.add_argument("--fluxes", action="store_true",
                    help="compare each heat flow and the thermal masses, term by term")
    ap.add_argument("--segment", type=int, nargs="*", default=None,
                    help="which vessels (flights) of the recording; default all")
    args = ap.parse_args()
    model = json.load(open(os.path.join(W.MODELS, args.model + ".json")))
    segs = segments(args.recording)
    names = [p["name"] for p in model["parts"]]
    picked = args.segment if args.segment is not None else range(len(segs))
    for s in picked:
        seg = segs[s]
        if seg["parts"] != names or len(seg["rows"]) < 2:
            print("segment %d: not this model's parts (or empty); skipped" % s)
            continue
        rows = seg["rows"]
        if args.fluxes:
            print("segment %d:" % s)
            flux_check(model, rows)
            continue
        th, out = replay(model, rows)
        print("segment %d: %d rows, %.0f s" % (s, len(rows), rows[-1]["ut"] - rows[0]["ut"]))
        print("%-3s %-26s %9s %9s  %11s %11s  %6s" % ("", "part", "skin rms", "int rms",
                                                      "peak sim/game", "int sim/game", "limit"))
        tot = []
        for k, pt in enumerate(th.parts):
            es = [o[k][0] - r["skin"][k] for o, r in zip(out, rows)]
            ei = [o[k][1] - r["temp"][k] for o, r in zip(out, rows)]
            rms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))  # noqa: E731
            tot.append(rms(es))
            print("%-3d %-26s %7.0f K %7.0f K  %5.0f/%5.0f  %5.0f/%5.0f  %6.0f"
                  % (k, pt.part.name[:26], rms(es), rms(ei), max(o[k][0] for o in out),
                     max(r["skin"][k] for r in rows), max(o[k][1] for o in out),
                     max(r["temp"][k] for r in rows), pt.max_skin))
        tot.sort()
        print("skin rms over parts: median %.0f K, worst %.0f K" % (tot[len(tot) // 2], tot[-1]))


if __name__ == "__main__":
    main()
