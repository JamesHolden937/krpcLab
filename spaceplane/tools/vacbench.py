#!/usr/bin/env python3
"""kRPC's attitude steps in vacuum, on the RCS alone, with its auto-tuned
gains scaled.
TEMPORARY TEST HARNESS -- not part of the flight software.

kRPC sizes its gains on ``available_torque``; on the cargo shuttle that
reads ~4x the RCS moment the nozzles can actually make about any one axis
(``rcsGeometry.py``: pitch/yaw 17.8 kN m pure against kRPC's 66), so its
loop is ~4x too soft and the coast never reached the entry attitude
(LOG14023, LOG14058).  This loads an orbital save, engages kRPC with the
pilot's own time_to_peak, lets it auto-tune, then (``--gain K``) freezes
the tune and multiplies every PID gain by K, and steps the nose 90 deg in
pitch and in yaw and the roof 90 deg in roll, logging the time to come
within 5 deg and the overshoot:

    ./spaceplane/tools/vacbench.py 0 --save qs_cargo_rigoff --gain 1
    ./spaceplane/tools/vacbench.py 0 --save qs_cargo_rigoff --gain 4
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
import krpc  # noqa: E402


def angle(a, b):
    d = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    return math.degrees(math.acos(max(-1.0, min(1.0, d / (na * nb)))))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", type=int)
    p.add_argument("--save", default="qs_cargo_rigoff")
    p.add_argument("--gain", type=float, default=1.0)
    p.add_argument("--peak", default="4.2,3.0,4.3",
                   help="time_to_peak pitch,roll,yaw (the pilot's static tune)")
    p.add_argument("--hold", type=float, default=40.0,
                   help="game-seconds per step")
    args = p.parse_args(argv)
    port = 50100 + 2 * args.instance
    conn = krpc.connect(name="vacbench", address="127.0.0.1",
                        rpc_port=port, stream_port=port + 1)
    sc = conn.space_center
    sc.load(args.save)
    time.sleep(3.0)
    v = sc.active_vessel
    v.control.rcs = True
    v.control.sas = False
    ap = v.auto_pilot
    frame = v.surface_velocity_reference_frame     # y along the velocity
    ap.reference_frame = frame
    vref = v.reference_frame
    for name in ("oscillation_bandwidth_floor_mode",
                 "oscillation_feedforward_mode",
                 "oscillation_output_filter_mode",
                 "pitch_yaw_rate_filter_mode", "roll_rate_filter_mode"):
        try:
            setattr(ap, name, type(getattr(ap, name)).off)
        except Exception:                               # noqa: BLE001
            pass
    ap.roll_start_angle = 180.0
    ap.roll_engage_angle = 175.0
    ap.time_to_peak = tuple(float(x) for x in args.peak.split(","))
    # Start where the vehicle points, roof where it is.
    nose0 = sc.transform_direction((0, 1, 0), vref, frame)
    roof0 = sc.transform_direction((0, 0, -1), vref, frame)
    ap.set_direction_and_up(nose0, roof0, 0.0)
    ap.engage()
    t0 = sc.ut
    while sc.ut - t0 < 3.0:
        time.sleep(0.05)
    print("torque kN m available (+): %s" % (tuple(
        round(x / 1000.0, 1) for x in v.available_torque[0]),))
    print("auto gains pitch %s roll %s yaw %s" % (
        ap.pitch_pid_gains, ap.roll_pid_gains, ap.yaw_pid_gains))
    if args.gain != 1.0:
        gp, gr, gy = ap.pitch_pid_gains, ap.roll_pid_gains, ap.yaw_pid_gains
        ap.auto_tune = False
        ap.pitch_pid_gains = tuple(g * args.gain for g in gp)
        ap.roll_pid_gains = tuple(g * args.gain for g in gr)
        ap.yaw_pid_gains = tuple(g * args.gain for g in gy)
        print("frozen x%.1f: pitch %s roll %s yaw %s" % (
            args.gain, ap.pitch_pid_gains, ap.roll_pid_gains,
            ap.yaw_pid_gains))

    def rot(vec_, axis, deg):
        a = math.radians(deg)
        k = axis
        c, s = math.cos(a), math.sin(a)
        dot = sum(x * y for x, y in zip(k, vec_))
        cr = (k[1] * vec_[2] - k[2] * vec_[1], k[2] * vec_[0] - k[0] * vec_[2],
              k[0] * vec_[1] - k[1] * vec_[0])
        return tuple(vec_[i] * c + cr[i] * s + k[i] * dot * (1 - c)
                     for i in range(3))

    for label in ("pitch", "yaw", "roll"):
        nose = sc.transform_direction((0, 1, 0), vref, frame)
        roof = sc.transform_direction((0, 0, -1), vref, frame)
        right = sc.transform_direction((1, 0, 0), vref, frame)
        if label == "pitch":
            want_nose, want_roof = rot(nose, right, 90), rot(roof, right, 90)
        elif label == "yaw":
            want_nose, want_roof = rot(nose, roof, 90), roof
        else:
            want_nose, want_roof = nose, rot(roof, nose, 90)
        ap.set_direction_and_up(want_nose, want_roof, 0.0)
        t0 = sc.ut
        within, worst_after = None, 0.0
        trace = []
        while sc.ut - t0 < args.hold:
            n = sc.transform_direction((0, 1, 0), vref, frame)
            r = sc.transform_direction((0, 0, -1), vref, frame)
            err = max(angle(n, want_nose), angle(r, want_roof))
            t = sc.ut - t0
            if within is None and err < 5.0:
                within = t
            if within is not None:
                worst_after = max(worst_after, err)
            if not trace or t - trace[-1][0] >= 2.0:
                c = v.control
                trace.append((t, err, c.pitch, c.roll, c.yaw))
            time.sleep(0.05)
        print("%-5s step 90: within 5 deg at %s s, worst after %.1f deg, "
              "final %.1f" % (label, "%.1f" % within if within else "never",
                              worst_after, err))
        print("      " + "  ".join("%.0fs:%.0f(%+.1f/%+.1f/%+.1f)" % x
                                   for x in trace))
    ap.disengage()
    conn.close()


if __name__ == "__main__":
    main()
