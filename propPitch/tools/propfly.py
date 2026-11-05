#!/usr/bin/env python3
"""Fly the prop pilot on a farm instance: load a save of a plane on the
runway, take off and hold level flight with kRPC's autopilot (the harness's
own connection), and let the prop pilot own the blades.  Prints one line per
flight: the airspeed reached, from the log.

    ./propPitch/tools/propfly.py --instance 0
    ./propPitch/tools/propfly.py --instance 1 --probe --takeoff-speed 9999 \
        --seconds 40     # the model checked against the game, on the roll

The plane is flown by the harness, not the pilot: the pilot is the
propellers only.  Heading HEADING, wheels until TAKEOFF_SPEED, a climb at
CLIMB_PITCH to ALT, then a height hold.
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
import krpc                                             # noqa: E402
import farmflight                                       # noqa: E402
from common.krpcpilot import set_autopilot_engaged      # noqa: E402


def fly_plane(rpc, stream, args, stop):
    """Take off and hold ``args.alt`` on ``args.heading`` until ``stop``."""
    conn = krpc.connect(name="propfly-plane", rpc_port=rpc, stream_port=stream)
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    surf = v.flight(body.reference_frame)
    ctl = v.control
    ctl.sas = False
    ctl.brakes = False
    ctl.throttle = args.throttle       # the throttle sets the rotors' rpm
    ap = v.auto_pilot
    ap.reference_frame = v.surface_reference_frame
    ap.target_heading = args.heading
    ap.target_pitch = 0.0
    ap.target_roll = 0.0
    set_autopilot_engaged(ap, True)
    ground = surf.mean_altitude
    airborne = False
    cut = False
    try:
        while not stop.is_set():
            spd, alt, vs = surf.speed, surf.mean_altitude, surf.vertical_speed
            if args.cut_at and spd > args.cut_at and not cut:
                cut = True
                ctl.throttle = 0.0         # the props feather (THROTTLE_FEATHER)
            if args.brake_at and spd > args.brake_at and not ctl.brakes:
                if args.wheel_brakes is not None:
                    for w in v.parts.wheels:
                        if w.has_brakes:
                            w.brakes = args.wheel_brakes
                ctl.brakes = True          # the props brake (PROP_BRAKE)
            if not airborne:
                pitch = args.climb_pitch if spd > args.takeoff_speed else 0.0
                if alt - ground > 20.0:
                    airborne = True
                    ctl.gear = False
            else:
                err = args.alt - alt
                pitch = max(-3.0, min(5.0, 0.01 * err - 0.15 * vs))
            ap.target_pitch = pitch
            time.sleep(0.1)
    finally:
        try:
            set_autopilot_engaged(ap, False)
        except Exception:                               # noqa: BLE001
            pass
        conn.close()


def summary(path):
    """Speed at the end and best 10-s mean, pitch centre at the end."""
    speeds, ctrs = [], []
    for line in open(path):
        m = re.search(r"TRACK spd=\s*([\d.]+) alt=\s*(-?\d+).*?pitch=\s*([\d.]+)",
                      line)
        if m:
            speeds.append(float(m.group(1)))
            ctrs.append(float(m.group(3)))
    if not speeds:
        return "no TRACK lines"
    best = max(sum(speeds[i:i + 10]) / 10 for i in range(max(1, len(speeds) - 9)))
    return ("end %.1f m/s, best 10 s %.1f, last 30 s %.1f, pitch %.1f"
            % (speeds[-1], best, sum(speeds[-30:]) / len(speeds[-30:]),
               ctrs[-1]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--instance", default="0")
    p.add_argument("--save", default="qs_prop_noon")
    p.add_argument("--seconds", type=float, default=300.0,
                   help="game seconds the pilot runs")
    p.add_argument("--heading", type=float, default=90.0)
    p.add_argument("--alt", type=float, default=300.0)
    p.add_argument("--takeoff-speed", type=float, default=40.0)
    p.add_argument("--climb-pitch", type=float, default=5.0)
    p.add_argument("--cut-at", type=float, default=0.0,
                   help="cut the throttle once past this speed (m/s; 0 = never)")
    p.add_argument("--brake-at", type=float, default=0.0,
                   help="press Brakes once past this speed (m/s; 0 = never)")
    p.add_argument("--wheel-brakes", type=float, default=None,
                   help="set every wheel's brake force (percent) as Brakes "
                        "goes on (default: the craft's)")
    p.add_argument("--throttle", type=float, default=1.0,
                   help="throttle held (sets the rotors' rpm limit)")
    p.add_argument("--set", action="append", default=[])
    p.add_argument("--probe", action="store_true",
                   help="fly tools/bladeprobe.py's angle sweep (implies --tree)")
    farmflight.add_arguments(p)
    args = p.parse_args()
    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect(name="propfly-loader", rpc_port=rpc, stream_port=stream)
    conn.space_center.load(args.save)
    conn.close()
    time.sleep(5.0)
    farmflight.install_reaper()
    stop = threading.Event()
    plane = threading.Thread(target=fly_plane, args=(rpc, stream, args, stop),
                             daemon=True)
    plane.start()
    overrides = list(args.set) + ["RUN_TIME_S=%g" % args.seconds]
    try:
        if args.probe:
            args.tree = True
        path, timed_out = farmflight.fly(args, "prop", "propPitch.tools.bladeprobe"
                                         if args.probe else "propPitch.autopilot",
                                         overrides, args.seconds * 3 + 120,
                                         ports=(rpc, stream))
    finally:
        stop.set()
        plane.join(10.0)
    print("ksp%s %s %s: %s%s" % (args.instance, path, " ".join(args.set)
                                 or "(defaults)", summary(path) if path and
                                 os.path.exists(path) else "no log",
                                 " TIMEOUT" if timed_out else ""))


if __name__ == "__main__":
    main()
