#!/usr/bin/env python3
"""stiffprobe.py <instance> [save ...] -- each craft's pitch stiffness, from the game.

Writes a ``logs/LOG<n>`` like every flight; nothing prints.

Loads each save (in vacuum, where deflecting a surface costs nothing), pumps
its propellant into the forward-most tank of each kind as ``FUEL_TO_NOSE``
does on the first COAST tick (``--no-pump`` leaves it), and probes
``simulate_aerodynamic_wrench_at`` over an alpha sweep at the glide's and the
cone's states.  Per state it reports the pitching moment per unit q
(``CmA``, m^3) against alpha, then:

- **the sign of nose-up**: the change in ``CmA`` with the pitch input held
  at +1 (positive pitch input is nose-up; the surfaces deflect in vacuum);
- **trim**: the alpha where the moment crosses zero at zero input;
- **stiffness** ``dCmA/dalpha`` there, signed so that negative is stable
  (restoring), and the short-period frequency it gives at that state,
  ``w^2 = -stiff * q / I_pitch`` (negative: divergent, ``1/sqrt(-w^2)`` is
  the e-folding time);
- **control**: ``dCmA`` per unit input, as the angle of attack it is worth
  against the stiffness (``ctrl / |stiff|`` deg per unit input).
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

STATES = [("M7", 55000.0, 2050.0), ("M5", 40000.0, 1550.0),
          ("M3", 30000.0, 950.0), ("M1.5", 19000.0, 445.0),
          ("M0.8", 12000.0, 240.0), ("cone", 6000.0, 150.0),
          ("final", 1000.0, 110.0)]

# The alpha each state is flown at, for the sideslip sweep.
YAW_ALPHA = {"M7": 35.0, "M5": 35.0, "M3": 25.0, "M1.5": 15.0,
             "M0.8": 10.0, "cone": 8.0, "final": 8.0}


def pump_to_nose(sc, vessel):
    """Every propellant's tanks into its forward-most one, as far as it holds."""
    frame = vessel.reference_frame
    moved = []
    for name in ("LiquidFuel", "Oxidizer"):
        tanks = []
        for part in vessel.parts.all:
            res = part.resources
            if res.has_resource(name):
                tanks.append((part.position(frame)[1], part,
                              float(res.max(name))))
        if len(tanks) < 2:
            continue
        tanks.sort(key=lambda t: t[0])
        _, nose, cap = tanks[-1]
        room = cap - float(nose.resources.amount(name))
        for _, part, _ in tanks[:-1]:
            amount = min(float(part.resources.amount(name)), room)
            if amount > 0.5:
                sc.ResourceTransfer.start(part, nose, name, amount)
                room -= amount
                moved.append("%.0f %s" % (amount, name))
    time.sleep(3.0)
    return moved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("saves", nargs="*", default=["qs_shuttle2_rigoff"])
    ap.add_argument("--alphas", default="-5,0,5,10,15,20,25,30,35,40,45,50")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--no-pump", action="store_true")
    ap.add_argument("--yaw", action="store_true",
                    help="also sweep sideslip at each state's alpha: the "
                    "yaw and roll moments per unit q against beta")
    ap.add_argument("--betas", default="-8,-4,0,4,8")
    ap.add_argument("--yaw-alphas", default="",
                    help="comma list: sweep sideslip at each of these "
                    "alphas at every state instead of the state's own")
    args = ap.parse_args()
    base = os.path.join(ROOT, "testInstances", "ksp%s" % args.instance)
    conn = krpc.connect(name="stiffprobe", address="127.0.0.1",
                        rpc_port=int(open(os.path.join(base, ".rpc_port")).read()),
                        stream_port=int(open(os.path.join(base, ".stream_port")).read()))
    sc = conn.space_center
    alphas = [float(a) for a in args.alphas.split(",")]
    book = Logbook(os.path.join(ROOT, "logs"), 0.0)
    book.event(sc.ut, "stiffprobe: instance %s, saves %s, pump %s" % (
        args.instance, " ".join(args.saves), not args.no_pump))

    def log(text):
        book.event(sc.ut, text)
    for save in args.saves:
        sc.load(save)
        time.sleep(2.0)
        try:
            conn.krpc.paused = False
        except Exception:                                   # noqa: BLE001
            pass
        v = sc.active_vessel
        moved = [] if args.no_pump else pump_to_nose(sc, v)
        body = v.orbit.body
        frame = body.reference_frame
        tf = sc.transform_direction
        nose = vec.unit(tf((0.0, 1.0, 0.0), v.reference_frame, frame))
        dorsal = vec.unit(tf((0.0, 0.0, -1.0), v.reference_frame, frame))
        right = vec.unit(tf((1.0, 0.0, 0.0), v.reference_frame, frame))
        up = vec.unit(v.position(frame))
        rot = tuple(v.rotation(frame))
        flight = v.flight(frame)
        inertia = v.moment_of_inertia[0]
        log("== %s: %s, %d parts, %.2f t, I_pitch %.0f t m^2, pumped: %s"
              % (save, v.name, len(v.parts.all), v.mass / 1e3,
                 inertia / 1e3, ", ".join(moved) or "nothing"))

        def rho(alt):
            return body.atmospheric_density_at_position(
                tuple(vec.scale(up, body.equatorial_radius + alt)), frame)

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
                body, pos, tuple(vec.scale(d, speed)), rot,
                (0.0, 0.0, 0.0), sc.ut)
            q = 0.5 * rho(alt) * speed * speed
            lift = vec.unit(vec.project_out(dorsal, d))
            if beta:
                return (vec.dot(torque, dorsal) / q,
                        vec.dot(torque, nose) / q, q)
            return vec.dot(force, lift) / q, vec.dot(torque, right) / q, q

        rows = {}
        for label, alt, speed in STATES:
            rows[label] = [probe(alt, speed, a) for a in alphas]
        v.control.pitch = 1.0
        time.sleep(args.settle)
        ctrl = {}
        for label, alt, speed in STATES:
            ctrl[label] = [probe(alt, speed, a)[1] for a in alphas]
        v.control.pitch = 0.0
        time.sleep(args.settle)
        for label, alt, speed in STATES:
            r = rows[label]
            q = r[0][2]
            dctrl = [c - x[1] for c, x in zip(ctrl[label], r)]
            mid = dctrl[len(dctrl) // 2]
            # Raw torque about the vessel's right axis is positive
            # *nose-down* (Unity's rotation sense: +x turns the nose, +y,
            # toward the belly, +z).  Settled by comparing airframes, not by
            # the +1 input, whose read-back sign is logged beside it: moving
            # the CoM aft lowers the raw moment and flattens its slope.
            up_sign = -1.0
            log("        +1 pitch input: dCmA(raw) %s"
                % " ".join("%g:%+.1f" % (a, d) for a, d in zip(alphas, dctrl)))
            m = [up_sign * x[1] for x in r]         # nose-up positive
            log("  %-5s q=%6.0f Pa  nose-up is %s CmA  ClA/CmA(nose-up+): %s"
                  % (label, q, "+" if up_sign > 0 else "-", " ".join(
                      "%g:%.0f/%+.1f" % (a, x[0], mm)
                      for a, x, mm in zip(alphas, r, m))))
            trims = []
            for k in range(len(alphas) - 1):
                if m[k] * m[k + 1] <= 0.0 and m[k] != m[k + 1]:
                    f = m[k] / (m[k] - m[k + 1])
                    a_t = alphas[k] + f * (alphas[k + 1] - alphas[k])
                    stiff = (m[k + 1] - m[k]) / (alphas[k + 1] - alphas[k])
                    trims.append((a_t, stiff))
            for a_t, stiff in trims:
                w2 = -stiff * 180.0 / math.pi * q / inertia
                kind = ("w %.2f rad/s (period %.1f s)" % (
                    math.sqrt(w2), 2 * math.pi / math.sqrt(w2)) if w2 > 0
                    else "DIVERGENT, e-fold %.1f s" % (1.0 / math.sqrt(-w2))
                    if w2 < 0 else "neutral")
                ctl = abs(up_sign * mid)
                log("        trim %5.1f deg  stiffness %+.2f m^3/deg  %s  "
                      "control %.1f m^3/unit = %.1f deg of alpha per unit "
                      "input" % (a_t, stiff, kind, ctl,
                                 ctl / abs(stiff) if stiff else float("inf")))
            if not trims:
                log("        no trim in the sweep (moment one sign: %s)"
                      % ("nose-up" if m[0] > 0 else "nose-down"))
        if args.yaw:
            betas = [float(b) for b in args.betas.split(",")]
            # Raw torques, signs as the game reports them: compare
            # airframes (fins forward must flatten the yaw slope) before
            # reading a sign as stable or unstable.
            yaw_alphas = [float(a) for a in args.yaw_alphas.split(",") if a]
            for label, alt, speed, a in [
                    (lb, al, sp, a) for lb, al, sp in STATES
                    for a in (yaw_alphas or [YAW_ALPHA[lb]])]:
                r = [probe(alt, speed, a, b) for b in betas]
                n = [x[0] for x in r]
                l = [x[1] for x in r]
                span = betas[-1] - betas[0]
                log("  %-5s alpha %2.0f  CnA(raw) %s  slope %+.2f m^3/deg"
                    "  ClA_roll(raw) slope %+.2f m^3/deg"
                    % (label, a, " ".join("%g:%+.1f" % (b, x)
                                          for b, x in zip(betas, n)),
                       (n[-1] - n[0]) / span, (l[-1] - l[0]) / span))
    book.close()
    conn.close()


if __name__ == "__main__":
    main()
