#!/usr/bin/env python3
"""Fit the part of a craft's aerodynamics the probed tables miss.

    ./kspSim/tools/calibrate.py --model qs_shuttle logs/kspsim/ft/g_qs_shuttle_cone_*.jsonl \\
        logs/kspsim/ft/g_qs_shuttle_final_*.jsonl [--write]

The tables are the game's own wrench oracle, asked about a paused vessel.
The flying vessel is not quite that vessel: on the Mk3 shuttle the game's
roll and yaw due to sideslip exceed the oracle's by up to 60 kN m and grow
with angle of attack (wings bending under 350 kN of lift is the likely
reason), while on the old plane the two agree to 20 N m.  Whatever the
cause, flight tests measure it.

Per frame of every recording, the residual (game - simulator) of the
**total** force and torque in body axes, over dynamic pressure, is regressed
on smooth terms of the state -- 1, alpha, beta, alpha^2, alpha beta, beta^2,
the three body rates made dimensionless by airspeed, and the three control
inputs -- by ridge regression.  The fit is reported in sample and
leave-one-recording-out, and ``--write`` stores it in every model of the
craft (``aero_residual``) with the Mach range it was fitted over; the
simulator applies it inside that range and fades it out over 0.3 Mach
beyond.
"""
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import numpy as np  # noqa: E402

from kspSim import quat, vec  # noqa: E402
from kspSim import world as W  # noqa: E402
from kspSim.tools import fidelity as F  # noqa: E402

BASE_TERMS = ("1", "a", "b", "a2", "ab", "b2", "p", "q", "r", "cp", "cr", "cy")
TERMS = BASE_TERMS + tuple(t + "*M" for t in BASE_TERMS)
REF_LENGTH = 10.0      # metres: rates are made dimensionless as w L / V


def features(alpha_deg, beta_deg, rates, speed, cmd, mach):
    """The regressors: smooth terms of the state, and each again times Mach
    (a residual fitted at Mach 0.2-0.4 made Mach 0.75 worse without them)."""
    a = math.radians(alpha_deg)
    b = math.radians(beta_deg)
    k = REF_LENGTH / max(speed, 1.0)
    base = [1.0, a, b, a * a, a * b, b * b, rates[0] * k, rates[1] * k, rates[2] * k,
            cmd[0], cmd[1], cmd[2]]
    return base + [x * mach for x in base]


def frames(path, model_name=None):
    h, rows = F.read(path)
    rp = F.Replay(model_name or h["save"], h, rows)
    # A fresh fit is against the tables alone, not the residual already in
    # the model -- or each refit would fit what the last one left.
    rp.v.residual = None
    X, Y, M = [], [], []
    body = rp.world.body
    for f in rp.forces():
        if f["q"] < 50.0:
            continue
        rp.set_state(f["i"])
        v = rp.v
        env = v.environment()
        wb = quat.rotate(quat.conj(v.q), vec.sub(v.w, body.spin))
        c = f["c"]
        X.append(features(env["alpha"], env["beta"], wb, env["speed"], (c[0], c[2], c[1]),
                          env["mach"]))
        res = [(g - s) / f["q"] for g, s in zip(list(f["f_game"]) + list(f["t_game"]),
                                                list(f["f_sim"]) + list(f["t_sim"]))]
        Y.append(res)
        M.append(env["mach"])
    return np.array(X), np.array(Y), np.array(M), h["save"]


def fit(X, Y, ridge):
    n = X.shape[1]
    scale = np.sqrt((X ** 2).mean(0)) + 1e-12
    Xs = X / scale
    A = Xs.T @ Xs + ridge * len(X) * np.eye(n)
    A[0, 0] -= ridge * len(X)            # the constant is not shrunk
    coef = np.linalg.solve(A, Xs.T @ Y)
    return coef / scale[:, None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--model", required=True, help="the craft's full model (where tables live)")
    ap.add_argument("--ridge", type=float, default=1e-3)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    sets = [frames(p) for p in args.recordings]
    names = ["f" + x for x in "xyz"] + ["t" + x for x in "xyz"]

    def rms(Y):
        return np.sqrt((Y ** 2).mean(0))
    X = np.vstack([s[0] for s in sets])
    Y = np.vstack([s[1] for s in sets])
    M = np.concatenate([s[2] for s in sets])
    coef = fit(X, Y, args.ridge)
    print("frames %d, Mach %.2f-%.2f" % (len(X), M.min(), M.max()))
    print("residual/q rms, before: " + " ".join("%s %.3g" % (n, x) for n, x in zip(names, rms(Y))))
    print("in sample, after:       " + " ".join("%s %.3g" % (n, x)
                                                  for n, x in zip(names, rms(Y - X @ coef))))
    # Leave one recording out.
    before, after = [], []
    for i, (Xi, Yi, _, _) in enumerate(sets):
        rest = [s for j, s in enumerate(sets) if j != i]
        if not rest:
            break
        c = fit(np.vstack([s[0] for s in rest]), np.vstack([s[1] for s in rest]), args.ridge)
        before.append(Yi)
        after.append(Yi - Xi @ c)
        print("  held out %-34s " % os.path.basename(args.recordings[i])[2:-6]
              + " ".join("%s %.3g>%.3g" % (n, x, y) for n, x, y in
                         zip(names, rms(Yi), rms(Yi - Xi @ c)) if n[0] == "t"))
    if before:
        print("held out, before:       " + " ".join("%s %.3g" % (n, x) for n, x in
                                                     zip(names, rms(np.vstack(before)))))
        print("held out, after:        " + " ".join("%s %.3g" % (n, x) for n, x in
                                                     zip(names, rms(np.vstack(after)))))
    # Keep only the components the held-out recordings say are improved
    # (by 10%): on the shuttle, pitch and axial force were not.
    keep = [True] * 6
    if before:
        b_rms, a_rms = rms(np.vstack(before)), rms(np.vstack(after))
        keep = [bool(a < 0.9 * b) for a, b in zip(a_rms, b_rms)]
        coef[:, [k for k in range(6) if not keep[k]]] = 0.0
        print("kept: " + " ".join(n for n, k in zip(names, keep) if k))
    if args.write:
        rec = {"terms": list(TERMS), "ref_length": REF_LENGTH, "kept": keep,
               "coef": coef.tolist(), "mach": [float(M.min()), float(M.max())],
               "recordings": [os.path.basename(p) for p in args.recordings]}
        src = json.load(open(os.path.join(W.MODELS, args.model + ".json")))
        from kspSim.tools.probe import same_craft
        for fn in sorted(os.listdir(W.MODELS)):
            if not fn.endswith(".json") or fn.startswith("_"):
                continue
            path = os.path.join(W.MODELS, fn)
            m = json.load(open(path))
            if same_craft(m, src):
                continue
            m["aero_residual"] = rec
            with open(path + ".tmp", "w") as fh:
                json.dump(m, fh)
            os.replace(path + ".tmp", path)
            print("wrote aero_residual to", fn)


if __name__ == "__main__":
    main()
