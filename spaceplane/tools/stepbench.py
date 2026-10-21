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


def identify(sc, run, say, args):
    """Open loop, kRPC off, every surface at neutral: each surface in turn
    to ``--pulse`` for 0.4 game-s and back, logging every tick's measured
    moment (I x the differenced body rate) against what the surface's
    column says the pulse makes -- whether B is the craft's B, and how
    long a deflection takes to arrive."""
    ap.set_autopilot_engaged(run.autopilot, False)
    for name in ("pitch", "roll", "yaw"):
        setattr(run.control, name, 0.0)
    surfaces = run._surfaces
    for cs, _ in surfaces:
        cs.deflection_override = False
    inertia = tuple(float(i) for i in run.vessel.moment_of_inertia)
    prev = None
    base = None

    def tick(label, i, d):
        nonlocal prev
        snap = run.telemetry.sample(run.env)
        axes = allocate.body_axes(snap.right, snap.nose, snap.roof)
        rate = allocate.body(snap.angular_velocity, axes)
        m = None
        if prev is not None and snap.ut > prev[0]:
            dt = snap.ut - prev[0]
            m = tuple(I * (r - p) / dt
                      for I, r, p in zip(inertia, rate, prev[1]))
        prev = (snap.ut, rate)
        mach = run.env.mach(vec.norm(snap.velocity), vec.norm(snap.position)
                            - run.env.equatorial_radius)
        # The slope at half the pulse times the pulse is a p + b p^2,
        # the whole moment the pulse adds on the quadratic model.
        col = run.surface_columns(mach, float(snap.krpc_aoa),
                                  snap.dynamic_pressure,
                                  [0.5 * args.pulse] * len(surfaces))[i]
        return snap, m, col

    for i, (cs, _) in enumerate(surfaces):
        if args.only and i not in args.only:
            continue
        rows = []
        for phase, d, n in (("pre", None, 4),
                            ("on", args.pulse, args.on_ticks),
                            ("off", None, args.on_ticks)):
            # Off is the input path with no input: an override of ~0
            # would leave the surface where it was.
            cs.deflection_override = d is not None
            if d is not None:
                cs.deflection = d
            for _ in range(n):
                snap, m, col = tick(phase, i, d)
                if m is not None:
                    rows.append((phase, m))
                    say("id d[%d] %-3s t=%.2f q=%.0f a=%+.1f M=%+.0f/%+.0f/"
                        "%+.0f kNm col*pulse=%+.0f/%+.0f/%+.0f"
                        % ((i, phase, snap.ut, snap.dynamic_pressure,
                            snap.krpc_aoa) + tuple(x / 1e3 for x in m)
                           + tuple(c * args.pulse / 1e3 for c in col)))
                wake = snap.ut + args.tick
                while sc.ut < wake:
                    time.sleep(0.005)
        pre = [m for ph, m in rows if ph == "pre"]
        on = [m for ph, m in rows if ph == "on"][2:]
        if pre and on:
            d_m = tuple(sum(m[k] for m in on) / len(on)
                        - sum(m[k] for m in pre) / len(pre) for k in range(3))
            say("id d[%d] %s: measured %+.0f/%+.0f/%+.0f kNm, column "
                "%+.0f/%+.0f/%+.0f" % ((i, cs.part.title)
                                       + tuple(x / 1e3 for x in d_m)
                                       + tuple(c * args.pulse / 1e3
                                               for c in col)))
    for cs, _ in surfaces:
        cs.deflection_override = False


def main():
    p = argparse.ArgumentParser()
    p.add_argument("instance")
    p.add_argument("--orbit", default="qs_shuttle2_rigoff")
    p.add_argument("--air", default="qs_shuttle2_gate")
    p.add_argument("--deg", type=float, default=5.0)
    p.add_argument("--hold", type=float, default=5.0)
    p.add_argument("--tick", type=float, default=0.1)
    p.add_argument("--set", action="append", default=[])
    p.add_argument("--identify", action="store_true",
                   help="open loop: pulse each surface alone and compare "
                        "the moment it makes with its measured column")
    p.add_argument("--pulse", type=float, default=0.5)
    p.add_argument("--on-ticks", type=int, default=6)
    p.add_argument("--only", type=int, action="append", default=[],
                   help="identify only these surface indices")
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
    tables = [(tuple(round(c, 1) for c in cs.part.position(frame)), table,
               law) for (cs, table), law in zip(run._surfaces,
                                                 run._surface_rate)]
    machs, alphas = run._surface_machs, run._surface_alphas

    # 2. the same craft in the air
    sc.load(args.air)
    time.sleep(1.0)
    run = ap.Autopilot(conn, cfg, book)
    frame = run.vessel.parts.root.reference_frame
    mapped, laws = [], []
    for cs in run.vessel.parts.control_surfaces:
        pos = tuple(round(c, 1) for c in cs.part.position(frame))
        best = min(tables, key=lambda t: vec.norm(vec.sub(t[0], pos)))
        if vec.norm(vec.sub(best[0], pos)) < 0.5:
            mapped.append((cs, best[1]))
            laws.append(best[2])
    run._surfaces = mapped
    run._surface_rate = laws
    run._surface_machs, run._surface_alphas = machs, alphas
    run.state = ap.HAC
    try:
        conn.krpc.paused = False
    except Exception:                                   # noqa: BLE001
        pass
    snap = run.telemetry.sample(run.env)
    base = allocate.body_axes(snap.right, snap.nose, snap.roof)
    for i, (cs, table) in enumerate(mapped):
        say("d[%d] %s at %s, travel %s" % (i, cs.part.title, tuple(
            round(c, 1) for c in cs.part.position(frame)), laws[i]))
    say("air: %d of %d surfaces mapped, q %.0f Pa, alpha %.1f, inertia "
        "%s kg m^2" % (len(mapped), len(tables), snap.dynamic_pressure,
                       snap.krpc_aoa, "/".join(
                           "%.3g" % i for i in run.vessel.moment_of_inertia)))

    if args.identify:
        identify(sc, run, say, args)
        book.close()
        return

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
            dbg = getattr(run, "_own_dbg", None)
            # want / in hand / allocated step, kN m per body axis
            # (right, nose, belly), the arithmetic ``own_attitude`` did.
            moments = "" if dbg is None else " M=%s" % " ".join(
                "%+.0f/%+.0f/%+.0f" % tuple(v[k] / 1e3 for v in dbg[:3])
                for k in range(3))
            say("%-10s t=%5.2f err=%+6.2f/%+6.2f/%+6.2f rate=%+6.1f/%+6.1f/"
                "%+6.1f q=%.0f a=%.1f d=%s%s"
                % ((label, snap.ut - start) + err + rate
                   + (snap.dynamic_pressure, snap.krpc_aoa,
                      " ".join("%+.2f" % d for d in
                               getattr(run, "_surface_act", None)
                               or run._surface_defl),
                      moments)))
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
