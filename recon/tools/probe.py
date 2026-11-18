#!/usr/bin/env python3
"""Characterize the QUDRAC on a farm instance: what it is, and how hot and
how hard it gets at full throttle at a held altitude.

    ./recon/tools/probe.py --instance 0 --alt 10000 --seconds 240

TEST HARNESS, not the pilot: kRPC's own autopilot flies the attitude.  The
craft (``saves/craft/SPH/QUDRAC.craft``) is launched on runway 09, the
RAPIER is locked to air-breathing, full throttle, it rotates at
``--rotate`` m/s and climbs to ``--alt`` (vertical speed asked
``(alt - h)/20`` s, capped at ``--climb``), then holds it.  Every second a
line: speed, Mach, altitude, q, thrust, g, alpha, and the hottest part as a
fraction of its limit (internal and skin).  Past ``--abort-frac`` of any
limit the throttle is cut (the probe must not burn the craft before it
says where the limit is).  Writes ``logs/probe-<instance>-<alt>.txt``.
"""
import argparse
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
sys.path.insert(0, os.path.join(ROOT, "airplane", "tools"))
from common import timescale as ts                       # noqa: E402
import airfly                                           # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", default="0")
    p.add_argument("--alt", type=float, default=10000.0)
    p.add_argument("--climb", type=float, default=150.0)
    p.add_argument("--rotate", type=float, default=80.0)
    p.add_argument("--seconds", type=float, default=240.0)
    p.add_argument("--throttle", type=float, default=1.0)
    p.add_argument("--abort-frac", type=float, default=0.9)
    p.add_argument("--lookahead", type=float, default=4.0)
    p.add_argument("--climb-speed", type=float, default=350.0)
    p.add_argument("--cap-alt", type=float, default=8000.0,
                   help="the climb speed is held only below this")
    p.add_argument("--timescale", default="3")
    args = p.parse_args()
    rpc, stream = paths.ports(args.instance)
    conn = airfly.launch_on_runway(args.instance, "QUDRAC", rpc, stream)
    sc = conn.space_center
    v = sc.active_vessel
    out = open(os.path.join(ROOT, "logs", "probe-%s-%d.txt"
                            % (args.instance, args.alt)), "w")

    def say(s):
        out.write(s + "\n")
        out.flush()
        print(s, flush=True)

    parts = v.parts.all
    say("vessel %s mass %.0f dry %.0f parts %d" % (v.name, v.mass, v.dry_mass,
                                                   len(parts)))
    for pt in parts:
        say("part %-22s m=%.3f maxT=%.0f maxSkin=%.0f T=%.0f" % (
            pt.name, pt.mass, pt.max_temperature, pt.max_skin_temperature,
            pt.temperature))
    eng = v.parts.engines[0]
    say("engine modes %s mode %s auto %s" % (
        list(eng.modes.keys()) if hasattr(eng, "modes") else "?",
        eng.mode, eng.auto_mode_switch))
    say("torque %s inertia %s" % (v.available_torque, v.moment_of_inertia))
    ref = v.orbit.body.reference_frame
    fl = v.flight(ref)
    c = v.control
    eng.auto_mode_switch = False
    if "Airbreathing" not in str(eng.mode) and "AirBreathing" not in str(eng.mode):
        eng.toggle_mode()
    eng.active = True
    c.sas = False
    ap = v.auto_pilot
    ap.reference_frame = v.surface_reference_frame
    ap.target_pitch_and_heading(0.0, 90.0)
    ap.target_roll = 0.0
    ap.engaged = True
    c.brakes = False
    c.throttle = args.throttle
    ts.write(ts.instance_dir(args.instance), args.timescale)
    conn.krpc.paused = False
    t0 = sc.ut
    last = -1.0
    airborne = False
    cut = False
    hottest = (0.0, "")
    prev = None
    while sc.ut - t0 < args.seconds:
        try:
            ut = sc.ut
            V = fl.speed
            h = fl.mean_altitude
        except Exception as exc:                        # noqa: BLE001
            say("ended: %r" % exc)
            break
        # Below the level altitude the speed is held to --climb-speed.
        if not cut:
            if h < min(args.alt - 1500, args.cap_alt):
                c.throttle = max(0.0, min(args.throttle,
                                          (args.climb_speed - V) / 50.0))
            else:
                c.throttle = args.throttle
        if not airborne and V > args.rotate:
            airborne = True
        if airborne:
            vs = max(-100.0, min(args.climb, (args.alt - h) / 20.0))
            gamma = math.degrees(math.asin(max(-1, min(1, vs / max(V, 50)))))
            pitch = gamma + (fl.angle_of_attack or 0.0)
            ap.target_pitch = max(-20.0, min(40.0, pitch))
            if h > 200:
                c.gear = False
        frac, name = 0.0, ""
        try:
            for pt in parts:
                f = max(pt.temperature / pt.max_temperature,
                        pt.skin_temperature / pt.max_skin_temperature)
                if f > frac:
                    frac, name = f, pt.name
        except Exception as exc:                        # noqa: BLE001
            say("ended: %r" % exc)
            break
        rate = 0.0
        if prev is not None and ut > prev[0]:
            rate = (frac - prev[1]) / (ut - prev[0])
        prev = (ut, frac)
        if frac > hottest[0]:
            hottest = (frac, name)
        # Predictive: where the fraction is going in --lookahead s.
        # A bang-bang heat governor: shut past --abort-frac predicted,
        # reopened once the prediction is 0.05 under it.
        ahead = frac + max(0.0, rate) * args.lookahead
        if ahead > args.abort_frac and not cut:
            c.throttle = 0.0
            cut = True
            say("CUT: %s at %.3f of its limit, rising %.4f/s, V=%.0f h=%.0f"
                % (name, frac, rate, V, h))
        elif cut and ahead < args.abort_frac - 0.05:
            cut = False
            say("REOPEN: %s at %.3f, V=%.0f h=%.0f" % (name, frac, V, h))
        if ut - last >= 1.0:
            last = ut
            cp = parts[0]
            say("t=%6.1f V=%6.1f M=%4.2f h=%6.0f q=%6.0f thr=%6.1f g=%5.2f "
                "aoa=%+5.1f hot=%.3f %+.4f/s %s cockpit=%.0f/%.0f LF=%.0f IA=%.2f thr_set=%.2f"
                % (ut - t0, V, fl.mach, h, fl.dynamic_pressure,
                   v.thrust / 1000.0, fl.g_force, fl.angle_of_attack,
                   frac, rate, name, cp.temperature, cp.skin_temperature,
                   v.resources.amount("LiquidFuel"),
                   v.resources.amount("IntakeAir"), c.throttle))
        if v.situation.name in ("splashed",) or (airborne and h < 5 and V < 5):
            say("ended: %s" % v.situation)
            break
        time.sleep(0.05)
    say("hottest %.2f %s" % hottest)
    try:
        conn.krpc.paused = True
        ap.engaged = False
    except Exception:                                   # noqa: BLE001
        pass
    ts.hold(ts.instance_dir(args.instance))
    out.close()


if __name__ == "__main__":
    main()
