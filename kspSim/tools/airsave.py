#!/usr/bin/env python3
"""Make an in-air save of a craft that only has orbital saves.

    ./kspSim/tools/airsave.py --instance 0 --save qs_plane --out qs_plane_air [--alt 25000]

``probe.py --controls-save`` needs the craft flying: control surfaces do not
answer input in vacuum.  The low-wing variant of the old spaceplane had no
save in the air, and the first probe took its control tables from the
mid-wing variant's.  This burns retrograde until the periapsis is at
``--periapsis``, falls to ``--alt`` holding the nose ``--pitch`` degrees
above the airflow with kRPC's autopilot, and saves there.  The save lands in
the instance's saves directory; ``testInstances/syncSaves.sh pull N <out>``
adopts it.
"""
import argparse
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

from common import timescale as ts  # noqa: E402
from kspSim.tools import flighttest  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("--save", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--alt", type=float, default=25000.0)
    ap.add_argument("--periapsis", type=float, default=30000.0)
    ap.add_argument("--pitch", type=float, default=15.0)
    args = ap.parse_args()
    conn = flighttest.load_paused(args.instance, args.save)
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    ctrl = v.control
    ap_ = v.auto_pilot
    ctrl.sas = False
    conn.krpc.paused = False
    ts.write(ts.instance_dir(args.instance), "4")
    ap_.reference_frame = v.orbital_reference_frame
    ap_.target_direction = (0.0, -1.0, 0.0)
    ap_.engaged = True
    t0 = time.time()
    while ap_.error > 3.0 and time.time() - t0 < 120:
        time.sleep(0.2)
    ctrl.throttle = 1.0
    while v.orbit.periapsis_altitude > args.periapsis and time.time() - t0 < 600:
        time.sleep(0.05)
    ctrl.throttle = 0.0
    print("periapsis %.0f m" % v.orbit.periapsis_altitude, flush=True)
    # Coast to the atmosphere on rails, then fly the nose above the airflow.
    ap_.engaged = False
    t_atm = None
    try:
        t_atm = sc.ut + v.orbit.time_to_periapsis * 0.5
        while body.altitude_at_position(v.position(body.reference_frame),
                                        body.reference_frame) > 72000 and time.time() - t0 < 900:
            sc.rails_warp_factor = 3
            time.sleep(0.5)
    finally:
        sc.rails_warp_factor = 0
    del t_atm
    # The nose ``pitch`` above the air-relative velocity, wings level: in the
    # surface frame (x up, y north, z east), recomputed as the flight turns.
    sf = v.surface_reference_frame
    ap_.reference_frame = sf
    ap_.target_roll = 0.0
    ap_.engaged = True
    fl = v.flight(body.reference_frame)
    p = math.radians(args.pitch)
    while fl.mean_altitude > args.alt and time.time() - t0 < 1800:
        vel = sc.transform_direction(v.velocity(body.reference_frame), body.reference_frame, sf)
        n = math.sqrt(sum(x * x for x in vel)) or 1.0
        u = [x / n for x in vel]
        perp = [1.0 - u[0] * u[0], -u[0] * u[1], -u[0] * u[2]]
        m = math.sqrt(sum(x * x for x in perp)) or 1.0
        ap_.target_direction = tuple(math.cos(p) * u[i] + math.sin(p) * perp[i] / m
                                     for i in range(3))
        time.sleep(0.1)
    ts.hold(ts.instance_dir(args.instance))
    conn.krpc.paused = True
    ap_.engaged = False
    sc.save(args.out)
    print("saved %s at %.0f m, %.0f m/s" % (args.out, fl.mean_altitude, fl.speed))
    conn.close()


if __name__ == "__main__":
    main()
