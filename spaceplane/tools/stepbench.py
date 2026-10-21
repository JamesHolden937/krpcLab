#!/usr/bin/env python3
"""stepbench.py <instance> -- ``SURFACE_OWN``'s step response, in a minute.

A landing distance is the wrong instrument for an attitude loop: twenty
minutes of deorbit and glide to learn that something oscillated somewhere.
This drives the pilot's own code on the bench instead:

1. loads ``--orbit`` (the craft in vacuum) and runs ``measure_surfaces``,
   the same vacuum sweep a flight does;
2. loads ``--air`` (the same craft flying), maps the measured tables onto its
   surfaces by position, and hands the attitude to ``own_attitude``;
3. holds the attitude it was loaded in, then steps the target ``--deg`` about
   each body axis in turn (pitch nose-up, roll, yaw) and back, ``--hold``
   game-seconds each, logging every tick and, per step, the peak, overshoot,
   time to 10% and the mean error over the hold's last second.

Writes ``logs/LOG<n>``; nothing prints.  ``--set FIELD=VALUE`` as a flight.
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
from spaceplane import allocate  # noqa: E402
from spaceplane import autopilot as ap  # noqa: E402
from spaceplane.config import Config, apply_overrides  # noqa: E402


def rotate_axes(axes, axis_index, deg):
    """The target ``(right, nose, belly)`` turned ``deg`` about one of its
    own axes, in the sense the controller's error uses
    (``allocate.attitude_error``: d(axis)/dt = w x axis)."""
    q = vec.quat_axis_angle(axes[axis_index], math.radians(deg))
    return tuple(vec.quat_rotate(q, a) for a in axes)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("instance")
    p.add_argument("--orbit", default="qs_shuttle2_rigoff")
    p.add_argument("--air", default="qs_shuttle2_gate")
    p.add_argument("--deg", type=float, default=5.0)
    p.add_argument("--hold", type=float, default=5.0)
    p.add_argument("--tick", type=float, default=0.1)
    p.add_argument("--set", action="append", default=[])
    args = p.parse_args()
    cfg = apply_overrides(Config(), ["SURFACE_OWN=1"] + args.set)
    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect(name="stepbench", rpc_port=rpc, stream_port=stream)
    sc = conn.space_center
    book = Logbook(os.path.join(ROOT, "logs"), 0.0)

    def say(text):
        book.event(sc.ut, text)

    say("stepbench: orbit %s, air %s, steps %.1f deg x %.1f s, tick %.2f, "
        "set %s" % (args.orbit, args.air, args.deg, args.hold, args.tick,
                    " ".join(args.set) or "-"))
    # 1. the vacuum sweep, on the pilot's own code
    sc.load(args.orbit)
    time.sleep(2.0)
    try:
        conn.krpc.paused = False
    except Exception:                                   # noqa: BLE001
        pass
    run = ap.Autopilot(conn, cfg, book)
    snap = run.telemetry.sample(run.env)
    run.env.sweep(snap.ut, full=True)
    snap = run.telemetry.sample(run.env)
    run.measure_surfaces(snap)
    if not run._surfaces:
        say("no surfaces measured -- stop")
        book.close()
        return
    # By position relative to the root part: the vessel frame's origin is
    # the CoM, which the two saves' propellant puts in different places.
    frame = run.vessel.parts.root.reference_frame
    tables = [(tuple(round(c, 1) for c in cs.part.position(frame)), table)
              for cs, table in run._surfaces]
    machs, alphas = run._surface_machs, run._surface_alphas

    # 2. the same craft in the air
    sc.load(args.air)
    time.sleep(1.0)
    run = ap.Autopilot(conn, cfg, book)
    frame = run.vessel.parts.root.reference_frame
    mapped = []
    for cs in run.vessel.parts.control_surfaces:
        pos = tuple(round(c, 1) for c in cs.part.position(frame))
        best = min(tables, key=lambda t: vec.norm(vec.sub(t[0], pos)))
        if vec.norm(vec.sub(best[0], pos)) < 0.5:
            mapped.append((cs, best[1]))
    run._surfaces = mapped
    run._surface_machs, run._surface_alphas = machs, alphas
    run.state = ap.HAC
    try:
        conn.krpc.paused = False
    except Exception:                                   # noqa: BLE001
        pass
    snap = run.telemetry.sample(run.env)
    base = allocate.body_axes(snap.right, snap.nose, snap.roof)
    say("air: %d of %d surfaces mapped, q %.0f Pa, alpha %.1f"
        % (len(mapped), len(tables), snap.dynamic_pressure, snap.krpc_aoa))

    # 3. hold, then step each axis and back
    names = ("pitch", "roll", "yaw")
    # Nose-up is a rotation about -right; roll and yaw about +nose/+belly.
    senses = (-1.0, 1.0, 1.0)
    plan = [("hold", base)]
    for k in range(3):
        plan.append(("%s %+g" % (names[k], args.deg),
                     rotate_axes(base, k, senses[k] * args.deg)))
        plan.append(("back", base))
    summary = []
    for label, target in plan:
        right, nose, belly = target
        run.commanded_nose = nose
        run.commanded_up = vec.scale(belly, -1.0)
        start = sc.ut
        rows = []
        while sc.ut - start < args.hold:
            snap = run.telemetry.sample(run.env)
            run.own_attitude(snap)
            axes = allocate.body_axes(snap.right, snap.nose, snap.roof)
            err = tuple(math.degrees(e) for e in allocate.body(
                allocate.attitude_error(axes, target), axes))
            rate = tuple(math.degrees(r) for r in allocate.body(
                snap.angular_velocity, axes))
            rows.append((snap.ut - start, err))
            say("%-10s t=%5.2f err=%+6.2f/%+6.2f/%+6.2f rate=%+6.1f/%+6.1f/"
                "%+6.1f d=%s" % ((label, snap.ut - start) + err + rate
                                 + (" ".join("%+.2f" % d for d in
                                             run._surface_defl),)))
            wake = snap.ut + args.tick
            while sc.ut < wake:
                time.sleep(0.005)
        if not rows:
            continue
        line = []
        for k in range(3):
            e = [r[1][k] for r in rows]
            first = abs(e[0])
            peak = max(abs(x) for x in e)
            over = 0.0
            if first > 0.5:
                sign = 1.0 if e[0] > 0 else -1.0
                over = max(0.0, -min(sign * x for x in e)) / first
            t10 = next((r[0] for r in rows
                        if abs(r[1][k]) <= 0.1 * max(first, 0.5)), None)
            tail = [r[1][k] for r in rows if r[0] >= args.hold - 1.0]
            line.append("%s %.1f->%.1f peak %.1f over %.0f%% t10 %s"
                        % (names[k], e[0], sum(tail) / max(1, len(tail)),
                           peak, 100 * over,
                           "%.1f" % t10 if t10 is not None else "-"))
        summary.append("%-10s %s" % (label, " | ".join(line)))
    for s in summary:
        say("step " + s)
    run.release_surfaces(snap)
    book.close()


if __name__ == "__main__":
    main()
