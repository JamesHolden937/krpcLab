#!/usr/bin/env python3
"""Fly the recon pilot on a farm instance: the QUDRAC on runway 09, a custom
waypoint placed where ``--lat``/``--lon`` say, START, and the flight's
events (target, pass, gate, touchdown, stop) and its peaks printed.

    ./recon/tools/reconfly.py --instance 0 --lat 5 --lon -60
    ./recon/tools/reconfly.py --instance 1 --lat -1.5 --lon -71.9 --set HEAT_MAX=0.8

The craft is ``saves/craft/SPH/QUDRAC.craft`` (the live one with its tank
filled: the live craft file has 0 of 400 LiquidFuel).
"""
import argparse
import os
import re
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "airplane", "tools"))
import krpc                                             # noqa: E402
import farmflight                                       # noqa: E402
import airfly                                           # noqa: E402
from common import timescale as ts                       # noqa: E402

EVENTS = ("target ", "phase ", "home:", "closest", "shutdown", "FATAL",
          "authority")


def unpause(rpc, instance, scale, lead_s):
    time.sleep(lead_s)
    conn = krpc.connect(name="reconfly-unpause", port=rpc)
    ts.write(ts.instance_dir(instance), scale)
    conn.krpc.paused = False
    conn.close()


def summary(path):
    out = []
    peak = {"spd": 0.0, "g": 0.0, "heat": 0.0, "alt": 0.0}
    with open(path) as fh:
        for line in fh:
            body = line.split("] ", 1)[-1]
            if body.startswith(EVENTS):
                out.append(line.rstrip())
                continue
            for key, pat in (("spd", r"spd=\s*([\d.]+)"), ("g", r" g=\s*([\d.]+)"),
                             ("heat", r"heat=([\d.]+)"), ("alt", r"alt=\s*([\d.]+)")):
                m = re.search(pat, body)
                if m:
                    peak[key] = max(peak[key], float(m.group(1)))
    out.append("peaks: speed %.0f m/s, g %.1f, heat %.3f, alt %.0f m"
               % (peak["spd"], peak["g"], peak["heat"], peak["alt"]))
    return "\n".join(out)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", default="0")
    p.add_argument("--lat", type=float, default=5.0)
    p.add_argument("--lon", type=float, default=-60.0)
    p.add_argument("--timescale", default="2")
    p.add_argument("--seconds", type=float, default=3600.0,
                   help="game seconds before the pilot stops")
    p.add_argument("--set", action="append", default=[])
    farmflight.add_arguments(p)
    args = p.parse_args()
    rpc, stream = paths.ports(args.instance)
    conn = airfly.launch_on_runway(args.instance, "QUDRAC", rpc, stream)
    sc = conn.space_center
    body = sc.active_vessel.orbit.body
    for w in list(sc.waypoint_manager.waypoints):
        try:
            if not w.has_contract:
                w.remove()
        except Exception:                               # noqa: BLE001
            pass
    sc.waypoint_manager.add_waypoint(args.lat, args.lon, body, "recon")
    conn.close()
    farmflight.install_reaper()
    th = threading.Thread(target=unpause, args=(rpc, args.instance,
                                                args.timescale, 6.0),
                          daemon=True)
    th.start()
    overrides = ["RUN_TIME_S=%g" % args.seconds] + list(args.set)
    try:
        path, timed_out = farmflight.fly(args, "recon", "recon.autopilot",
                                         overrides, args.seconds + 300,
                                         ports=(rpc, stream))
    finally:
        try:
            ts.hold(ts.instance_dir(args.instance))
        except Exception:                               # noqa: BLE001
            pass
    print("ksp%s %s target %.3f,%.3f %s%s" % (
        args.instance, path, args.lat, args.lon,
        " ".join(args.set) or "(defaults)", " TIMEOUT" if timed_out else ""))
    if path and os.path.exists(path):
        print(summary(path))


if __name__ == "__main__":
    main()
