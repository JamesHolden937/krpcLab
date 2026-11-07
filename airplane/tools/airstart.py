#!/usr/bin/env python3
"""Make a save of a plane already flying: level at ``--alt`` m above sea
level, heading 090 out of KSC, at ``--speed`` m/s and full throttle.

    ./airplane/tools/airstart.py --instance 0      # the Electric Warbird, 100 m, 200 m/s

The craft (``saves/craft/SPH/<craft>.craft``) is launched on runway 09 of
farm instance N and flown there by kRPC's autopilot (the harness's, not
ours) while ``propPitch`` flies the blades (a child process, as
``propPitch/tools/propfly.py`` runs it): full throttle, take off at ``--rotate`` m/s, gear up, an altitude
hold to ``--alt``; the game is paused and saved the moment it is level
there at ``--speed``.  The save is flown, not edited, on purpose: a save
edited into the air carries a propeller at rest, and the Warbird spawned at
150 m/s with stopped blades never spun them up -- it slowed to 23 m/s in
18 s and fell in (2026-11-05).  The save lands in the instance's saves
directory; ``testInstances/syncSaves.sh pull N <out>`` adopts it.
"""
import argparse
import os
import shutil
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
sys.path.insert(0, os.path.join(ROOT, "tools"))
import krpc                                             # noqa: E402
import farmflight                                       # noqa: E402
from common.krpcpilot import set_autopilot_engaged      # noqa: E402
from kspSim.tools import flighttest                     # noqa: E402


def state(conn):
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    f = v.flight(v.surface_reference_frame)
    fb = v.flight(body.reference_frame)
    return (sc.ut, fb.mean_altitude, fb.surface_altitude, fb.speed, f.pitch,
            f.heading, f.roll, str(v.situation).split(".")[-1])


def fly_to(conn, args):
    """Take off and fly level at ``args.alt`` until it is there at
    ``args.speed``; pause.  True when it got there."""
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    fb = v.flight(body.reference_frame)
    fs = v.flight(v.surface_reference_frame)
    c = v.control
    c.sas = False
    c.brakes = False
    c.throttle = 1.0
    ap = v.auto_pilot
    ap.reference_frame = v.surface_reference_frame
    ap.target_heading = 90.0
    ap.target_roll = 0.0
    ap.target_pitch = fs.pitch            # the tail on the ground
    set_autopilot_engaged(ap, True)
    t0 = sc.ut
    last = t0
    ground = fb.mean_altitude
    airborne = False
    try:
        while sc.ut - t0 < args.timeout:
            alt, vs, spd = fb.mean_altitude, fb.vertical_speed, fb.speed
            ut = sc.ut
            if not airborne:
                if spd > args.rotate:
                    ap.target_pitch = 8.0
                if alt - ground > 15.0:
                    airborne = True
                    c.gear = False
            else:
                ap.target_pitch = max(-5.0, min(10.0, 0.05 * (args.alt - alt)
                                                - 0.3 * vs))
                # Held to the save's speed while it levels; full throttle is
                # set at the save point.
                c.throttle = max(0.05, min(1.0, 0.5 + 0.05 * (args.speed - spd)))
            if ut - last >= 2.0:
                last = ut
                print("  t+%5.1f alt %6.1f vs %+5.1f spd %5.1f pitch %+5.1f "
                      "roll %+5.1f" % (ut - t0, alt, vs, spd, fs.pitch,
                                       fs.roll), flush=True)
            # Within 8 m: the altitude law above settles ~6 m off (MiniSystem
            # farm, 2026-11-06), and kRPC's roll swings through level.
            if (airborne and abs(alt - args.alt) < 8.0 and abs(vs) < 1.5
                    and abs(fs.roll) < 2.0 and abs(spd - args.speed) < 2.0
                    and ut - t0 > 30.0):
                conn.krpc.paused = True
                c.throttle = 1.0
                return True
            time.sleep(0.02)
    finally:
        set_autopilot_engaged(ap, False)
    conn.krpc.paused = True
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", default="0")
    p.add_argument("--craft", default="Electric Warbird")
    p.add_argument("--out", default="qs_warbird")
    p.add_argument("--alt", type=float, default=100.0,
                   help="the save's altitude, m above sea level")
    p.add_argument("--speed", type=float, default=200.0,
                   help="the save's airspeed, m/s")
    p.add_argument("--rotate", type=float, default=50.0,
                   help="nose up for take-off past this speed, m/s")
    p.add_argument("--timeout", type=float, default=300.0,
                   help="game seconds")
    farmflight.add_arguments(p)
    args = p.parse_args()

    inst = paths.instance_dir(args.instance)
    sdir = os.path.join(inst, "saves", "default")
    src = os.path.join(ROOT, "saves", "craft", "SPH", args.craft + ".craft")
    shutil.copy(src, os.path.join(sdir, "Ships", "SPH", args.craft + ".craft"))
    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect(name="airstart", rpc_port=rpc, stream_port=stream)
    try:
        conn.krpc.paused = False          # a run killed while paused
    except Exception:                                   # noqa: BLE001
        pass                              # not in flight: nothing to unpause
    conn.space_center.launch_vessel("SPH", args.craft, "Runway", recover=True)
    time.sleep(8.0)
    print("on the runway: ut %.1f alt %.1f (surface %.1f) spd %.1f pitch %.1f "
          "hdg %.1f roll %.1f %s" % state(conn), flush=True)
    # propPitch flies the blades for the whole flight to the save; it is
    # reaped once the game is paused at the save point.
    farmflight.install_reaper()
    prop = farmflight.launch(farmflight.command(
        args, "prop", "propPitch.autopilot",
        ["RUN_TIME_S=%g" % (args.timeout + 60.0)])
        + ["--rpc-port", str(rpc), "--stream-port", str(stream)])
    try:
        ok = fly_to(conn, args)
    finally:
        farmflight.reap(prop)
    if not ok:
        raise SystemExit("did not reach %.0f m level at %.0f m/s in %.0f s"
                         % (args.alt, args.speed, args.timeout))
    print("saving %s: ut %.2f alt %.1f (surface %.1f) spd %.1f pitch %.1f "
          "hdg %.1f roll %.1f %s" % ((args.out,) + state(conn)), flush=True)
    conn.space_center.save(args.out)
    conn.close()
    time.sleep(2.0)
    conn = flighttest.load_paused(args.instance, args.out)
    print("reloaded (paused): ut %.2f alt %.1f (surface %.1f) spd %.1f "
          "pitch %.1f hdg %.1f roll %.1f %s" % state(conn), flush=True)
    conn.krpc.paused = False
    for _ in range(4):
        time.sleep(1.0)
        print("free: ut %.2f alt %.1f (surface %.1f) spd %.1f pitch %.1f "
              "hdg %.1f roll %.1f %s" % state(conn), flush=True)
    conn.krpc.paused = True
    conn.close()


if __name__ == "__main__":
    main()
