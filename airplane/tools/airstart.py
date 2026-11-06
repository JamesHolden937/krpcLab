#!/usr/bin/env python3
"""Make a save of a plane already flying: level, wings level, heading east
over runway 09 at ``--alt`` m above it and ``--speed`` m/s through the air.

    ./airplane/tools/airstart.py --instance 0 --craft "Electric Warbird" \
        --out qs_warbird            # 100 m, 200 m/s (the defaults)

The craft (``saves/craft/SPH/<craft>.craft``) is launched on the runway of
farm instance N and saved; then the save is edited, not flown: the active
vessel's ``ORBIT`` is rewritten from a state vector raised ``--alt`` m
straight up from where it stood (the surface's rotation scaled to the new
radius) with ``--speed`` m/s added along that rotation -- east, down the
runway -- (``tools/savegen.py``'s element conversion, whose frame cancels);
its ``sit`` becomes FLYING, and its ``rot`` is turned about the vessel's
pitch axis by the pitch it sat at, so the nose is on the horizon.  Then the
save is loaded and the state it actually flies from is printed: a vessel on
rails while the scene loads falls a little before physics takes it.

``--pitch-sign`` exists because Unity's quaternions in the save are
left-handed: the tool checks the loaded pitch and says if it went the wrong
way.  The save lands in the instance's saves directory;
``testInstances/syncSaves.sh pull N <out>`` adopts it.
"""
import argparse
import math
import os
import re
import shutil
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
sys.path.insert(0, os.path.join(ROOT, "tools"))
import krpc                                             # noqa: E402
import savegen                                          # noqa: E402
from kspSim.tools import flighttest                     # noqa: E402


def vessel_block(lines):
    """(first line, line after) of the active vessel's VESSEL block."""
    active = int(next(re.match(r"\s*activeVessel = (\d+)", x).group(1)
                      for x in lines if re.match(r"\s*activeVessel = ", x)))
    vessels = savegen.top_level_vessels(lines)
    start = [v for v in vessels if v[0] == active][0][2]
    depth = 0
    for j in range(start + 1, len(lines)):
        s = lines[j].strip()
        if s == "{":
            depth += 1
        elif s == "}":
            depth -= 1
            if depth == 0:
                return start, j + 1
    raise SystemExit("unterminated VESSEL")


def set_field(lines, lo, hi, key, value):
    """Set a field that sits directly in the block (three tabs deep)."""
    for i in range(lo, hi):
        if lines[i].startswith("\t\t\t%s = " % key):
            lines[i] = "\t\t\t%s = %s\n" % (key, value)
            return
    raise SystemExit("no %s in the vessel" % key)


def get_field(lines, lo, hi, key):
    for i in range(lo, hi):
        if lines[i].startswith("\t\t\t%s = " % key):
            return lines[i].split(" = ", 1)[1].strip()
    raise SystemExit("no %s in the vessel" % key)


def qmul(a, b):
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def edit(lines, alt, speed, pitch_deg, mu):
    """Raise the active vessel ``alt`` m, add ``speed`` m/s east, make it
    FLYING at full throttle, nose down by ``pitch_deg``; returns the
    altitude it stood at."""
    lo, hi = vessel_block(lines)
    olo, ohi, _ = savegen.find_orbit_block(lines)
    el = savegen.read_elements(lines, olo, ohi)
    r, v = savegen.elements_to_state(el, mu)
    rn, vn = savegen._norm(r), savegen._norm(v)
    r2 = savegen._scale(r, (rn + alt) / rn)
    v2 = savegen._scale(savegen._unit(v), vn * (rn + alt) / rn + speed)
    savegen.write_elements(lines, olo, ohi,
                           savegen.state_to_elements(r2, v2, mu, el["EPH"]))
    a0 = float(get_field(lines, lo, hi, "alt"))
    # Full throttle from the first frame: at idle the stopped blades brake
    # it at ~50 m/s^2 while the scene loads.
    for i in range(lo, hi):
        if lines[i].strip().startswith("mainThrottle = "):
            lines[i] = lines[i].split("=")[0] + "= 1\n"
            break
    else:
        raise SystemExit("no mainThrottle in the vessel")
    set_field(lines, lo, hi, "sit", "FLYING")
    set_field(lines, lo, hi, "landed", "False")
    set_field(lines, lo, hi, "landedAt", "")
    set_field(lines, lo, hi, "displaylandedAt", "")
    set_field(lines, lo, hi, "alt", repr(a0 + alt))
    set_field(lines, lo, hi, "hgt", repr(alt))
    # Nose down by the pitch it sat at, about the vessel's own x (pitch)
    # axis: rot' = rot * q(x, -pitch).
    rot = tuple(float(x) for x in get_field(lines, lo, hi, "rot").split(","))
    h = math.radians(-pitch_deg) / 2.0
    new = qmul(rot, (math.sin(h), 0.0, 0.0, math.cos(h)))
    set_field(lines, lo, hi, "rot", ",".join("%.9g" % x for x in new))
    return a0


def state(conn):
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    f = v.flight(v.surface_reference_frame)
    fb = v.flight(body.reference_frame)
    return (sc.ut, fb.mean_altitude, fb.surface_altitude, fb.speed, f.pitch,
            f.heading, f.roll, str(v.situation).split(".")[-1])


def fly_to(conn, args):
    """Fly the spawned plane level at full throttle (kRPC's autopilot,
    heading 090, an altitude hold to ``--alt`` m above sea level) until it
    is level there at ``--speed``, and pause.  True when it got there."""
    from common.krpcpilot import set_autopilot_engaged
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    fb = v.flight(body.reference_frame)
    fs = v.flight(v.surface_reference_frame)
    c = v.control
    c.sas = False
    c.throttle = 1.0
    c.gear = False
    ap = v.auto_pilot
    ap.reference_frame = v.surface_reference_frame
    ap.target_heading = 90.0
    ap.target_roll = 0.0
    ap.target_pitch = 0.0
    set_autopilot_engaged(ap, True)
    conn.krpc.paused = False
    t0 = sc.ut
    last = t0
    try:
        while sc.ut - t0 < args.timeout:
            alt, vs, spd = fb.mean_altitude, fb.vertical_speed, fb.speed
            ap.target_pitch = max(-8.0, min(8.0, 0.03 * (args.alt - alt)
                                            - 0.25 * vs))
            ut = sc.ut
            if ut - last >= 2.0:
                last = ut
                print("  t+%5.1f alt %6.1f vs %+5.1f spd %5.1f pitch %+5.1f "
                      "roll %+5.1f" % (ut - t0, alt, vs, spd, fs.pitch,
                                       fs.roll))
            if (ut - t0 > args.settle and abs(alt - args.alt) < 3.0
                    and abs(vs) < 1.5 and abs(fs.roll) < 3.0
                    and abs(spd - args.speed) < 2.0):
                conn.krpc.paused = True
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
    p.add_argument("--spawn-alt", type=float, default=300.0,
                   help="spawned this far above the runway")
    p.add_argument("--spawn-speed", type=float, default=150.0,
                   help="spawned at this airspeed, east")
    p.add_argument("--settle", type=float, default=20.0,
                   help="game seconds flown before the save may be taken "
                        "(the rotor spinning up)")
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--pitch", type=float, default=None,
                   help="nose-down rotation (deg); default: the pitch it "
                        "sat at on the runway")
    args = p.parse_args()

    inst = paths.instance_dir(args.instance)
    sdir = os.path.join(inst, "saves", "default")
    src = os.path.join(ROOT, "saves", "craft", "SPH", args.craft + ".craft")
    shutil.copy(src, os.path.join(sdir, "Ships", "SPH", args.craft + ".craft"))
    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect(name="airstart", rpc_port=rpc, stream_port=stream)
    sc = conn.space_center
    sc.launch_vessel("SPH", args.craft, "Runway", recover=True)
    time.sleep(8.0)
    st = state(conn)
    print("on the runway: ut %.1f alt %.1f (surface %.1f) pitch %.1f hdg %.1f "
          "roll %.1f %s" % (st[0], st[1], st[2], st[4], st[5], st[6], st[7]))
    rwy = args.out + "_rwy"
    spawn = args.out + "_spawn"
    sc.save(rwy)
    time.sleep(2.0)
    with open(os.path.join(sdir, rwy + ".sfs")) as fh:
        lines = fh.readlines()
    pitch = st[4] if args.pitch is None else args.pitch
    a0 = edit(lines, args.spawn_alt, args.spawn_speed, pitch,
              savegen.MU_KERBIN)
    with open(os.path.join(sdir, spawn + ".sfs"), "w") as fh:
        fh.writelines(lines)
    os.remove(os.path.join(sdir, rwy + ".sfs"))
    print("spawn %s: alt %.1f -> %.1f, +%.0f m/s east, nose down %.1f"
          % (spawn, a0, a0 + args.spawn_alt, args.spawn_speed, pitch))
    conn.close()
    conn = flighttest.load_paused(args.instance, spawn)
    if not fly_to(conn, args):
        raise SystemExit("did not reach %.0f m level at %.0f m/s in %.0f s"
                         % (args.alt, args.speed, args.timeout))
    st = state(conn)
    print("saving %s: ut %.2f alt %.1f (surface %.1f) spd %.1f pitch %.1f "
          "hdg %.1f roll %.1f %s" % ((args.out,) + st))
    conn.space_center.save(args.out)
    conn.close()
    time.sleep(2.0)
    os.remove(os.path.join(sdir, spawn + ".sfs"))
    conn = flighttest.load_paused(args.instance, args.out)
    st = state(conn)
    print("reloaded (paused): ut %.2f alt %.1f (surface %.1f) spd %.1f "
          "pitch %.1f hdg %.1f roll %.1f %s" % st)
    conn.close()


if __name__ == "__main__":
    main()
