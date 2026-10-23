#!/usr/bin/env python3
"""Every RCS nozzle's moment about the CoM, and the best pure torque any
firing mix can make about each body axis.
TEMPORARY TEST HARNESS -- not part of the flight software.

KSP groups the nozzles per control input; a craft whose roll input makes
pitch (the cargo shuttle, LOG13790/13817) may still have a roll in its
nozzles that the grouping never fires.  This answers whether it does,
from the geometry alone, before anything is built to fire it:

    ./spaceplane/tools/rcsGeometry.py 0 --save qs_cargo_rigoff

For each body axis (pitch = right, roll = nose, yaw = belly) it finds the
largest moment about that axis with the other two held at zero, over every
nozzle at a throttle in [0, 1], and lists the nozzles that make it.
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

AXES = ("pitch", "roll", "yaw")       # body x (right), y (nose), z (belly)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def best_pure(cols, axis, iters=4000):
    """Max moment about ``axis`` with the other two at zero, u in [0, 1]:
    bisect on the target, projected gradient on the box for each."""
    def residual(target):
        u = [0.5] * len(cols)
        norm = sum(c[0] ** 2 + c[1] ** 2 + c[2] ** 2 for c in cols) or 1.0
        step = 1.0 / norm
        for _ in range(iters):
            m = [sum(u[i] * cols[i][k] for i in range(len(cols)))
                 for k in range(3)]
            err = [m[k] - (target if k == axis else 0.0) for k in range(3)]
            for i, c in enumerate(cols):
                g = c[0] * err[0] + c[1] * err[1] + c[2] * err[2]
                u[i] = min(1.0, max(0.0, u[i] - step * g))
        m = [sum(u[i] * cols[i][k] for i in range(len(cols))) for k in range(3)]
        return math.sqrt(sum((m[k] - (target if k == axis else 0.0)) ** 2
                             for k in range(3))), u, m
    best = {}
    for sign in (1.0, -1.0):
        hi = sign * sum(abs(c[axis]) for c in cols)
        lo = 0.0
        found = (0.0, [0.0] * len(cols), [0.0, 0.0, 0.0])
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            res, u, m = residual(mid)
            if res <= 0.02 * abs(mid) + 1.0:
                lo, found = mid, (mid, u, m)
            else:
                hi = mid
        best[sign] = found
    return best


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", type=int)
    p.add_argument("--save", default=None, help="load this save first")
    args = p.parse_args(argv)
    port = 50100 + 2 * args.instance
    conn = krpc.connect(name="rcsGeometry", address="127.0.0.1",
                        rpc_port=port, stream_port=port + 1)
    sc = conn.space_center
    if args.save:
        sc.load(args.save)
        time.sleep(3.0)
    vessel = sc.active_vessel
    frame = vessel.reference_frame          # origin at the CoM
    print("vessel %s, %d parts, %.2f t" % (vessel.name, len(vessel.parts.all),
                                            vessel.mass / 1000.0))
    cols, names = [], []
    for rcs in vessel.parts.rcs:
        part = rcs.part
        thrust = rcs.max_vacuum_thrust
        nozzles = rcs.thrusters
        each = thrust / max(1, len(nozzles))
        for j, t in enumerate(nozzles):
            pos = t.thrust_position(frame)
            d = t.thrust_direction(frame)
            force = tuple(each * x for x in d)
            m = cross(pos, force)
            cols.append(tuple(x / 1000.0 for x in m))        # kN m
            names.append("%s#%d@(%+.1f,%+.1f,%+.1f) dir(%+.2f,%+.2f,%+.2f)"
                         % (part.title[:18], j, pos[0], pos[1], pos[2],
                            d[0], d[1], d[2]))
    print("%d nozzles" % len(cols))
    for n, c in zip(names, cols):
        print("  %-70s  M kN m (pitch %+6.1f roll %+6.1f yaw %+6.1f)"
              % (n, c[0], c[1], c[2]))
    for axis in range(3):
        best = best_pure(cols, axis)
        for sign, (val, u, m) in sorted(best.items()):
            used = [names[i].split("@")[0] for i in range(len(u)) if u[i] > 0.05]
            print("pure %s %s: %+.1f kN m (moment %+.1f/%+.1f/%+.1f) from %d "
                  "nozzle(s)" % (AXES[axis], "+" if sign > 0 else "-", val,
                                 m[0], m[1], m[2], len(used)))
    conn.close()


if __name__ == "__main__":
    main()
