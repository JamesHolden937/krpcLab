#!/usr/bin/env python3
"""spoilerprobe.py <instance> [save] -- what each control surface is worth as
a spoiler at the glide's (or the cone's) states.

TEMPORARY TEST HARNESS: prints to stdout deliberately.

Loads ``save`` (default ``qs_shuttle2_rigoff``, in vacuum, where deflecting a
surface costs nothing) and, one surface at a time, sets its ``Deploy Angle``
to +/-``--angle`` (read back: the part clamps it), deploys it, and probes
``simulate_aerodynamic_wrench_at`` at each state.  Prints per surface and
state the change in drag, lift and the pitching, yawing and rolling moments
(all per unit q: m^2 and m^3), then the best pitch-neutral lift dump the
measured surfaces can sum to -- mirror pairs together so roll and yaw cancel,
forward and aft groups weighted so pitch cancels -- against the airframe's
own lift at that state.  That last line is the question: how much lift a
spoiler can take away without a pitching moment the glide would have to
trim, and so without costing it the alpha it is already short of.

Every surface goes back to stowed before it exits.
"""
import argparse
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import paths  # noqa: E402

paths.use_venv()

import krpc  # noqa: E402

from common import vec  # noqa: E402

GLIDE = [("M2", 22000.0, 600.0, 20.0), ("M4", 32000.0, 1200.0, 30.0),
         ("M6", 40000.0, 1800.0, 35.0), ("M7.5", 50000.0, 2200.0, 40.0)]
CONE = [("cone hi", 10000.0, 200.0, 15.0), ("cone lo", 5000.0, 120.0, 8.0),
        ("approach", 1500.0, 110.0, 4.0)]


def surfaces(vessel):
    out = []
    for p in vessel.parts.all:
        for m in p.modules:
            if "ControlSurface" in m.name and m.has_field("Deploy Angle"):
                out.append((p, m))
                break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("save", nargs="?", default="qs_shuttle2_rigoff")
    ap.add_argument("--angle", type=float, default=15.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--cone", action="store_true",
                    help="the cone's and the approach's states instead")
    args = ap.parse_args()
    states = CONE if args.cone else GLIDE
    base = os.path.join(ROOT, "testInstances", "ksp%s" % args.instance)
    conn = krpc.connect(name="spoilerprobe", address="127.0.0.1",
                        rpc_port=int(open(os.path.join(base, ".rpc_port")).read()),
                        stream_port=int(open(os.path.join(base, ".stream_port")).read()))
    sc = conn.space_center
    sc.load(args.save)
    time.sleep(2.0)
    try:
        conn.krpc.paused = False
    except Exception:                                       # noqa: BLE001
        pass
    v = sc.active_vessel
    body = v.orbit.body
    frame = body.reference_frame
    rot = tuple(v.rotation(frame))
    tf = sc.transform_direction
    nose = vec.unit(tf((0.0, 1.0, 0.0), v.reference_frame, frame))
    dorsal = vec.unit(tf((0.0, 0.0, -1.0), v.reference_frame, frame))
    right = vec.unit(tf((1.0, 0.0, 0.0), v.reference_frame, frame))
    up = vec.unit(v.position(frame))
    flight = v.flight(frame)

    def probe(alt, speed, alpha):
        a = math.radians(alpha)
        d = vec.unit(vec.sub(vec.scale(nose, math.cos(a)),
                             vec.scale(dorsal, math.sin(a))))
        pos = tuple(vec.scale(up, body.equatorial_radius + alt))
        force, torque = flight.simulate_aerodynamic_wrench_at(
            body, pos, tuple(vec.scale(d, speed)), rot, (0.0, 0.0, 0.0), sc.ut)
        rho = body.atmospheric_density_at_position(pos, frame)
        q = 0.5 * rho * speed * speed
        lift_dir = vec.unit(vec.project_out(dorsal, d))
        return (-vec.dot(force, d) / q, vec.dot(force, lift_dir) / q,
                vec.dot(torque, right) / q, vec.dot(torque, dorsal) / q,
                vec.dot(torque, nose) / q)

    surf = surfaces(v)
    rows = []
    try:
        for p, m in surf:
            m.set_field_float("Deploy Angle", 0.0)
            p.control_surface.deployed = False
        time.sleep(args.settle)
        ref = {s[0]: probe(*s[1:]) for s in states}
        for label, alt, spd, al in states:
            cd, cl = ref[label][0], ref[label][1]
            print("%-8s h %5.0f v %4.0f a %4.1f: CdA %6.1f ClA %6.1f L/D %.2f"
                  % (label, alt, spd, al, cd, cl, cl / cd))
        for i, (p, m) in enumerate(surf):
            pos = tuple(round(x, 2) for x in p.position(v.reference_frame))
            for sign in (1.0, -1.0):
                m.set_field_float("Deploy Angle", sign * args.angle)
                p.control_surface.deployed = True
                time.sleep(args.settle)
                got = m.get_field("Deploy Angle")
                for label, alt, spd, al in states:
                    r = ref[label]
                    d = [a - b for a, b in zip(probe(alt, spd, al), r)]
                    rows.append((i, p.title, pos, sign, label, d))
                    print("#%d %-22s x%+5.2f y%+6.2f %+3.0f (read %s) %-8s dCdA "
                          "%+6.2f dClA %+6.2f pitch %+7.1f yaw %+7.1f roll %+7.1f"
                          % (i, p.title, pos[0], pos[1], sign * args.angle, got,
                             label, d[0], d[1], d[2], d[3], d[4]))
                m.set_field_float("Deploy Angle", 0.0)
                p.control_surface.deployed = False
                time.sleep(args.settle)
    finally:
        for p, m in surfaces(v):
            try:
                m.set_field_float("Deploy Angle", 0.0)
                p.control_surface.deployed = False
            except Exception:                               # noqa: BLE001
                pass

    # The best pitch-neutral lift dump: per state, mirror pairs (same |x|,
    # same y, same title) deployed together in the sign that dumps lift;
    # then the forward and aft pairs combined so the pitching moments cancel.
    print("\n-- pitch-neutral lift dump at %.0f deg (pairs by mirror, fore/aft "
          "weighted to cancel pitch)" % args.angle)
    for label, alt, spd, al in states:
        pairs = {}
        for i, title, pos, sign, lab, d in rows:
            if lab != label:
                continue
            key = (title, round(abs(pos[0]), 1), round(pos[1], 1))
            pairs.setdefault((key, sign), []).append(d)
        options = []
        for (key, sign), ds in pairs.items():
            if len(ds) < 2:
                continue
            s = [sum(x[k] for x in ds) for k in range(5)]
            if s[1] < 0.0:
                options.append((key, sign, s))
        best = None
        for a in options:
            for b in options:
                if a[0] == b[0] or a[2][2] * b[2][2] >= 0.0:
                    continue
                w = -a[2][2] / b[2][2]
                if not 0.0 < w <= 1.0:
                    continue
                tot = [a[2][k] + w * b[2][k] for k in range(5)]
                if best is None or tot[1] < best[2][1]:
                    best = (a, (b, w), tot)
        lone = min(options, key=lambda o: o[2][1], default=None)
        r = ref[label]
        if best:
            tot = best[2]
            print("%-8s pair %s + %.2f x %s: dClA %+6.1f (%+.0f%% of %.1f) "
                  "dCdA %+5.1f pitch %+5.1f roll %+5.1f"
                  % (label, best[0][0][0], best[1][1], best[1][0][0][0],
                     tot[1], 100.0 * tot[1] / r[1], r[1], tot[0], tot[2],
                     tot[4]))
        elif lone:
            print("%-8s no cancelling pair; best single pair %s: dClA %+6.1f "
                  "pitch %+7.1f" % (label, lone[0][0], lone[2][1], lone[2][2]))
        else:
            print("%-8s no surface pair dumps lift" % label)


if __name__ == "__main__":
    main()
