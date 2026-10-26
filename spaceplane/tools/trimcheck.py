#!/usr/bin/env python3
"""trimcheck.py <instance> <save> -- is the probe's pitching moment the flown one?

Writes a ``logs/LOG<n>``; nothing prints.

Loads an in-air save (``entrysave.py``), and at the vessel's own state reads
``simulate_aerodynamic_wrench_at`` (surfaces at neutral: nothing is flying it)
over an alpha sweep about the flown one, in the stiffprobe's convention
(nose-up positive, per unit q).  Then lets the airframe go, controls at zero
and SAS off, and logs alpha and pitch rate for ``--free`` seconds: the sign
of the free alpha change at the flown alpha is the ground truth the probe's
sign must match.  Run it with nothing else flying the instance.
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
from common.logbook import Logbook  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("save")
    ap.add_argument("--free", type=float, default=3.0)
    args = ap.parse_args()
    base = os.path.join(ROOT, "testInstances", "ksp%s" % args.instance)
    conn = krpc.connect(name="trimcheck", address="127.0.0.1",
                        rpc_port=int(open(os.path.join(base, ".rpc_port")).read()),
                        stream_port=int(open(os.path.join(base, ".stream_port")).read()))
    sc = conn.space_center
    book = Logbook(os.path.join(ROOT, "logs"), 0.0)

    def log(text):
        book.event(sc.ut, text)
    sc.load(args.save)
    time.sleep(2.0)
    conn.krpc.paused = True
    v = sc.active_vessel
    body = v.orbit.body
    frame = body.reference_frame
    tf = sc.transform_direction
    nose = vec.unit(tf((0.0, 1.0, 0.0), v.reference_frame, frame))
    dorsal = vec.unit(tf((0.0, 0.0, -1.0), v.reference_frame, frame))
    right = vec.unit(tf((1.0, 0.0, 0.0), v.reference_frame, frame))
    pos = tuple(v.position(frame))
    vel = tuple(v.velocity(frame))
    rot = tuple(v.rotation(frame))
    flight = v.flight(frame)
    speed = vec.norm(vel)
    vhat = vec.unit(vel)
    # Alpha in the pitch plane: the velocity's angle below the nose.
    alpha0 = math.degrees(math.atan2(-vec.dot(vhat, dorsal), vec.dot(vhat, nose)))
    beta0 = math.degrees(math.asin(vec.clamp(vec.dot(vhat, right), -1, 1)))
    q = flight.dynamic_pressure
    log("trimcheck %s: alt %.0f m, %.0f m/s, Mach %.2f, q %.0f Pa, alpha %.1f "
        "(flight says %.1f), beta %.1f, pitch input in save %+.2f, %.2f t"
        % (args.save, flight.mean_altitude, speed, flight.mach, q, alpha0,
           flight.angle_of_attack, beta0, v.control.pitch, v.mass / 1e3))
    force, torque = flight.simulate_aerodynamic_wrench_at(
        body, pos, vel, rot, tuple(v.angular_velocity(frame)), sc.ut)
    log("  at the flown state: CmA(nose-up+) %+.1f m^3, lift %.0f kN"
        % (-vec.dot(torque, right) / q, vec.dot(force, vec.unit(
            vec.project_out(dorsal, vhat))) / 1e3))
    for da in (-10, -5, 0, 5, 10, 15, 20):
        a = math.radians(alpha0 + da)
        d = vec.unit(vec.sub(vec.scale(nose, math.cos(a)),
                             vec.scale(dorsal, math.sin(a))))
        f, t = flight.simulate_aerodynamic_wrench_at(
            body, pos, tuple(vec.scale(d, speed)), rot, (0.0, 0.0, 0.0), sc.ut)
        log("  alpha %5.1f: CmA(nose-up+) %+7.1f" % (alpha0 + da,
                                                     -vec.dot(t, right) / q))
    v.auto_pilot.engaged = False
    v.control.sas = False
    v.control.pitch = v.control.yaw = v.control.roll = 0.0
    conn.krpc.paused = False
    t0 = time.time()
    while time.time() - t0 < args.free:
        log("  free: alpha %.2f pitch rate %+.3f rad/s (body x) input %+.2f"
            % (v.flight(frame).angle_of_attack,
               v.angular_velocity(v.reference_frame)[0]
               if False else vec.dot(v.angular_velocity(frame), right),
               v.control.pitch))
        time.sleep(0.25)
    conn.krpc.paused = True
    book.close()
    conn.close()


if __name__ == "__main__":
    main()
