#!/usr/bin/env python3
"""Record a game flight from the side: state and every part's temperatures.

    ./kspSim/tools/observe.py --instance 2 -o obs.jsonl [--seconds 3600]

A passive kRPC client: it commands nothing.  Every 0.2 wall seconds it
writes one line -- UT, position and velocity in the body frame, rotation,
density, and each part's skin and internal temperature by part index
(``parts.all`` order, the probe's order).  Whatever autopilot is flying, its
flight becomes data the simulator's thermal model is fitted and checked
against.  Survives scene reloads between flights of a batch.

``--fluxes`` adds each part's heat flows as the game computed them (kW:
convection, radiation, conduction, skin to interior) and its thermal and
skin masses (kJ/K) -- what ``thermcheck.py --fluxes`` compares term by term.
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

import krpc  # noqa: E402


FLUXES = ("thermal_convection_flux", "thermal_radiation_flux", "thermal_conduction_flux",
          "thermal_skin_to_internal_flux", "thermal_mass", "thermal_skin_mass")


def attach(instance, fluxes=False):
    base = os.path.join(ROOT, "testInstances", "ksp%s" % instance)
    rpc = int(open(os.path.join(base, ".rpc_port")).read())
    stream = int(open(os.path.join(base, ".stream_port")).read())
    conn = krpc.connect(name="kspsim-observer", rpc_port=rpc, stream_port=stream)
    sc = conn.space_center
    v = sc.active_vessel
    bf = v.orbit.body.reference_frame
    fl = v.flight(bf)
    s = {
        "ut": conn.add_stream(getattr, sc, "ut"),
        "pos": conn.add_stream(v.position, bf),
        "vel": conn.add_stream(v.velocity, bf),
        "rot": conn.add_stream(v.rotation, bf),
        "rho": conn.add_stream(getattr, fl, "atmosphere_density"),
        "situation": conn.add_stream(getattr, v, "situation"),
    }
    parts = v.parts.all
    skins = [conn.add_stream(getattr, p, "skin_temperature") for p in parts]
    temps = [conn.add_stream(getattr, p, "temperature") for p in parts]
    names = [p.name for p in parts]
    extra = {}
    if fluxes:
        for f in FLUXES:
            extra[f] = [conn.add_stream(getattr, p, f) for p in parts]
    return conn, v, s, skins, temps, names, extra


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--seconds", type=float, default=7200)
    ap.add_argument("--fluxes", action="store_true", help="also each part's heat flows and masses")
    args = ap.parse_args()
    out = open(args.out, "a", buffering=1)
    end = time.time() + args.seconds
    conn = None
    vessel_id = None
    while time.time() < end:
        try:
            if conn is None:
                conn, v, s, skins, temps, names, extra = attach(args.instance, args.fluxes)
                vessel_id = v._object_id
                out.write(json.dumps({"new_vessel": names, "wall": time.time()}) + "\n")
            if conn.space_center.active_vessel._object_id != vessel_id:
                raise RuntimeError("vessel changed")
            row = {k: (str(f()) if k == "situation" else f()) for k, f in s.items()}
            row["skin"] = [round(f(), 2) for f in skins]
            row["temp"] = [round(f(), 2) for f in temps]
            for k, fs in extra.items():
                row[k] = [round(f(), 4) for f in fs]
            out.write(json.dumps(row) + "\n")
            time.sleep(0.2)
        except Exception:                               # noqa: BLE001
            # A reload between flights, or a part that burnt off: reattach.
            try:
                conn.close()
            except Exception:                           # noqa: BLE001
                pass
            conn = None
            time.sleep(2.0)


if __name__ == "__main__":
    main()
