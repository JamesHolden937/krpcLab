#!/usr/bin/env python3
"""splitprobe.py <instance> [save] -- what the split rudder is worth as a brake.

TEMPORARY TEST HARNESS: prints to stdout deliberately.

Loads ``save`` (default ``qs_shuttle2_rigoff``, in vacuum, where deflecting a
surface costs nothing), deploys the two Big-S tail fins together at each
``Deploy Angle`` in ``--angles`` (read back after setting: the part clamps
it), and at each setting probes ``simulate_aerodynamic_wrench_at`` at the
cone's and the approach's states.  Reports, against the stowed fins:
added drag (dCdA), lift (dClA), and the yawing, pitching and rolling moments
(dCnA, dCmA, dClA_roll) per unit q -- so whether the pair really splits
(yaw near zero) and how much brake it is, as a fraction of the airframe's
own drag at the same state.  ``--one-side`` deploys only the left fin, the
check that the two halves are mirror images and not one rudder.

The fins go back to stowed at angle 0 before it exits.
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


def fins(vessel):
    out = []
    for p in vessel.parts.all:
        if "Tail Fin" in p.title:
            for m in p.modules:
                if "ControlSurface" in m.name and m.has_field("Deploy Angle"):
                    out.append((p, m))
                    break
    return out


def set_fins(conn, fin_list, angle, settle):
    for p, m in fin_list:
        m.set_field_float("Deploy Angle", float(angle))
        try:
            p.control_surface.deployed = angle != 0.0
        except Exception:                                   # noqa: BLE001
            for e in ("Deploy", "Retract"):
                if m.has_event(e) and (angle != 0.0) == (e == "Deploy"):
                    m.trigger_event(e)
    time.sleep(settle)
    return [m.get_field("Deploy Angle") for _, m in fin_list]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("save", nargs="?", default="qs_shuttle2_rigoff")
    ap.add_argument("--angles", default="0,10,20,30,38,45")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--one-side", action="store_true")
    # Sideslip as the brake instead (fins stowed): the airflow yawed by beta.
    ap.add_argument("--slip", default="")
    args = ap.parse_args()
    base = os.path.join(ROOT, "testInstances", "ksp%s" % args.instance)
    conn = krpc.connect(name="splitprobe", address="127.0.0.1",
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
    fin_list = fins(v)
    print("vessel %s, %d tail-fin surfaces: %s" % (
        v.name, len(fin_list), ", ".join("%s @ %s" % (
            p.title, tuple(round(x, 2) for x in p.position(v.reference_frame)))
            for p, _ in fin_list)))
    if args.one_side:
        fin_list = fin_list[:1]
    rot = tuple(v.rotation(frame))
    tf = sc.transform_direction
    nose = vec.unit(tf((0.0, 1.0, 0.0), v.reference_frame, frame))
    dorsal = vec.unit(tf((0.0, 0.0, -1.0), v.reference_frame, frame))
    right = vec.unit(tf((1.0, 0.0, 0.0), v.reference_frame, frame))
    up = vec.unit(v.position(frame))
    flight = v.flight(frame)

    def rho(alt):
        return body.atmospheric_density_at_position(
            tuple(vec.scale(up, body.equatorial_radius + alt)), frame)

    # (label, altitude m, true airspeed m/s, alpha deg)
    states = [("cone hi", 10000.0, 200.0, 15.0), ("cone lo", 5000.0, 120.0, 8.0),
              ("approach", 1500.0, 110.0, 4.0), ("flare", 200.0, 80.0, 8.0)]

    def probe(alt, speed, alpha, beta=0.0):
        a = math.radians(alpha)
        d = vec.unit(vec.sub(vec.scale(nose, math.cos(a)),
                             vec.scale(dorsal, math.sin(a))))
        if beta:
            b = math.radians(beta)
            d = vec.unit(vec.add(vec.scale(d, math.cos(b)),
                                 vec.scale(right, math.sin(b))))
        pos = tuple(vec.scale(up, body.equatorial_radius + alt))
        force, torque = flight.simulate_aerodynamic_wrench_at(
            body, pos, tuple(vec.scale(d, speed)), rot, (0.0, 0.0, 0.0), sc.ut)
        q = 0.5 * rho(alt) * speed * speed
        # Lift is the force across the flow in the plane of symmetry's
        # normal; under sideslip the side force is reported apart.
        lift_dir = vec.unit(vec.project_out(dorsal, d))
        return (-vec.dot(force, d) / q, vec.dot(force, lift_dir) / q,
                vec.dot(torque, dorsal) / q, vec.dot(torque, right) / q,
                vec.dot(torque, nose) / q)

    angles = [float(x) for x in args.angles.split(",")]
    try:
        set_fins(conn, fin_list, 0.0, args.settle)
        ref = {s[0]: probe(*s[1:]) for s in states}
        for label, alt, spd, al in states:
            cd, cl = ref[label][0], ref[label][1]
            print("%-9s h %5.0f v %3.0f a %4.1f: stowed CdA %6.1f ClA %6.1f L/D %.2f"
                  % (label, alt, spd, al, cd, cl, cl / cd))
        for beta in [float(x) for x in args.slip.split(",") if x]:
            print("-- sideslip %.0f deg, fins stowed" % beta)
            for label, alt, spd, al in states:
                cd, cl, cn, cm, cr = probe(alt, spd, al, beta)
                r = ref[label]
                print("   %-9s dCdA %+6.1f (%+4.0f%%)  dClA %+6.1f  L/D %.2f->%.2f"
                      "  yaw %+7.1f pitch %+7.1f roll %+7.1f (moments /q, m^3)"
                      % (label, cd - r[0], 100.0 * (cd - r[0]) / r[0], cl - r[1],
                         r[1] / r[0], cl / cd, cn - r[2], cm - r[3], cr - r[4]))
        for angle in (angles[1:] if not args.slip else []):
            got = set_fins(conn, fin_list, angle, args.settle)
            print("-- asked %.0f, read back %s" % (angle, got))
            for label, alt, spd, al in states:
                cd, cl, cn, cm, cr = probe(alt, spd, al)
                r = ref[label]
                print("   %-9s dCdA %+6.1f (%+4.0f%%)  dClA %+6.1f  L/D %.2f->%.2f"
                      "  yaw %+7.1f pitch %+7.1f roll %+7.1f (moments /q, m^3)"
                      % (label, cd - r[0], 100.0 * (cd - r[0]) / r[0], cl - r[1],
                         r[1] / r[0], cl / cd, cn - r[2], cm - r[3], cr - r[4]))
    finally:
        set_fins(conn, fins(v), 0.0, 0.2)


if __name__ == "__main__":
    main()
