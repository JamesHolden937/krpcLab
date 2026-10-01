#!/usr/bin/env python3
"""Ask the game's own wrench oracle about a recording's states.

    ./kspSim/tools/oracle.py --instance 1 logs/kspsim/ft/g_qs_cone_yaw.jsonl -o oracle.json [--every 5]

Loads the recording's save paused and calls
``Flight.simulate_aerodynamic_wrench_at`` at the recorded position, velocity,
rotation and angular velocity of every Nth frame (body frame, converted with
the model's rotation).  The simulator's tables are made of this oracle, so
where the oracle agrees with the recorded dynamics and the simulator does
not, the tables are wrong; where the oracle disagrees with the recorded
dynamics, the oracle is not the physics the game flies.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

from kspSim import quat, vec  # noqa: E402
from kspSim import world as W  # noqa: E402
from kspSim.tools import flighttest  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--instance", required=True)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--no-omega", action="store_true", help="ask at zero angular velocity")
    ap.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"),
                    help="only frames with recording time t in [T0, T1]")
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()
    rows = [json.loads(line) for line in open(args.recording)]
    header, rows = rows[0], rows[1:]
    model = json.load(open(os.path.join(W.MODELS, header["save"] + ".json")))
    body = W.Body(model)
    conn = flighttest.load_paused(args.instance, header["save"])
    sc = conn.space_center
    v = sc.active_vessel
    kb = v.orbit.body
    bf = kb.reference_frame
    fl = v.flight(bf)
    out = []
    for i in range(0, len(rows), args.every):
        r = rows[i]
        if args.window and not args.window[0] <= r["t"] <= args.window[1]:
            continue
        t = r["ut"]
        pos = body.to_bf(tuple(r["r"]), t)
        # Air-relative velocity in the rotating frame.
        vel = vec.sub(body.to_bf(tuple(r["v"]), t), vec.cross(body.spin, pos))
        q_bf = quat.mul(quat.conj(body.q_bf(t)), quat.norm(tuple(r["q"])))
        w_bf = body.to_bf(vec.sub(tuple(r["w"]), body.spin), t)
        if args.no_omega:
            w_bf = (0.0, 0.0, 0.0)
        f, tq = fl.simulate_aerodynamic_wrench_at(kb, pos, vel, q_bf, w_bf, t)
        inv = quat.conj(q_bf)
        out.append({"i": i, "t": r["t"], "f": quat.rotate(inv, f), "tq": quat.rotate(inv, tq)})
    conn.close()
    json.dump({"recording": args.recording, "rows": out}, open(args.out, "w"))
    print("%d oracle calls" % len(out))


if __name__ == "__main__":
    main()
