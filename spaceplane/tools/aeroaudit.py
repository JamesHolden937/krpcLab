#!/usr/bin/env python3
"""Audit the aerodynamic model against the flight it flew.

TEMPORARY TEST HARNESS -- not part of the flight software, and it prints to
stdout deliberately: it is a diagnostic, run on a log after the fact.

    ./aeroaudit.py logs/LOG620
    ./aeroaudit.py logs/LOG620 --rows 30

The spaceplane had no equivalent of ``replay.py``, and that gap cost a
session.  Five batches of in-game flights showed the vehicle landing 40-70 km
short with the deorbit exiting 200 m from its aim, the predicted miss nulled
to tens of metres at 54 km, cross-track inside a kilometre, and the commanded
angle of attack tracked to under a degree.  Every *guidance* number was good
and the vehicle still missed, which is the signature of a **model** error
rather than a control one -- and a model error can be audited against any log
already on disk, with no flight at all.

What it does is recover what the vehicle actually did from the telemetry and
compare it against what the table said it would do.  The log already carries
everything needed:

* ``v``, ``vs`` and the timestamp give the along-track and normal
  accelerations by finite difference.
* ``alt`` gives the local gravity and the sphere's own ``v^2/R``.
* ``dec`` is the model's drag deceleration, ``cda``/``cla`` the coefficients
  the propagator was reading, and ``bank`` how much of the lift was vertical.

So ``q/m`` comes out of ``dec / cda`` without needing the density, and the
model's lift acceleration is ``cla * (dec/cda) * cos(bank)``.  Both halves are
then directly comparable with the finite-differenced truth.

**Read the lift column with care, and never high in the entry.**
``lift_actual`` is a residual: a finite-differenced flight-path angle rate
minus ``g cos(gamma)`` minus ``v^2/R``, two terms of about 9 m/s^2 each that
very nearly cancel.  Above 40 km the lift being measured is a few tenths of
a m/s^2, so a 1% error in either subtracted term is tens of percent of the
answer.  On LOG1722 this column read **1.77x** median and 5.13x at Mach 6.9,
and a batch was spent building a correction for it -- while the direct
in-flight comparison (``environment.LiftTrim``: the game's own reported
force, resolved the way the table is, at the angle the vehicle is
*achieving*) read **0.98-1.00 in every Mach bin over thousands of samples**.
The table was right.

The drag column does not have this problem -- it is compared against the
measured deceleration directly -- and it read 0.99, which was the clue.

Run on LOG620 this printed, in one pass:

    alt    Mach   L/D model  L/D actual
    52357  6.6        1.07       4.62
    33772  5.8        0.89       1.19
    23926  3.0        0.98       0.96
    19018  2.0        0.94       0.80
    12921  0.9        1.06       0.41     <- falling, not gliding
     2036  0.2        1.63       1.56

The airframe's measured subsonic best glide is **3.39**, so a vehicle doing
0.41 at Mach 0.9 is not gliding at all, and that is where the whole shortfall
was being spent.  The cause was ``SOLVE_ALPHA_MIN_DEG`` -- a 20 degree floor
that is right at Mach 5-7, where the range curve has an interior minimum
there, and badly wrong subsonically, where best glide is at 10 degrees and
extra angle of attack is only drag.

**Read the ratio columns, not the absolute numbers.**  The finite difference
is noisy tick to tick (the default window smooths over several), the rotating
frame's Coriolis term is neglected here, and near the ground the flight-path
angle rate is dominated by the flare.  What survives all of that is whether
the model and the vehicle agree to tens of percent, and in which direction.
"""
import argparse
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

MU = 3.532e12
BODY_RADIUS = 600000.0

FIELD = re.compile(r"(\w+)=\s*([-+0-9.]+)")
STAMP = re.compile(r"\[\s*([0-9.]+)\]")


def read(path, phase):
    """Every telemetry line of one phase, as a dict plus its timestamp."""
    out = []
    for line in open(path):
        if (" %s " % phase) not in line:
            continue
        stamp = STAMP.match(line)
        if not stamp:
            continue
        fields = dict(FIELD.findall(line))
        try:
            out.append((float(stamp.group(1)),
                        {k: float(v) for k, v in fields.items()}))
        except ValueError:
            continue
    return out


def audit(rows, window, wanted):
    """Model against truth, one row per sample."""
    out = []
    step = max(1, (len(rows) - 2 * window) // max(1, wanted))
    for i in range(window, len(rows) - window, step):
        t0, a = rows[i - window]
        t1, b = rows[i + window]
        dt = t1 - t0
        if dt <= 0:
            continue
        try:
            v0, v1 = a["v"], b["v"]
            vs0, vs1 = a["vs"], b["vs"]
            altitude, dec = b["alt"], b["dec"]
            cla, cda, bank, mach = b["cla"], b["cda"], b["bank"], b["M"]
        except KeyError:
            continue
        if cda <= 0.0 or v1 <= 1.0 or dec <= 0.0:
            continue
        radius = BODY_RADIUS + altitude
        gravity = MU / (radius * radius)

        # Along the velocity: gravity helps while descending, drag opposes.
        drag_actual = -(v1 - v0) / dt - gravity * (vs1 / v1)

        # Across it: the flight path angle's rate, less what the sphere and
        # gravity account for on their own.
        gamma0 = math.asin(max(-1.0, min(1.0, vs0 / max(1.0, v0))))
        gamma1 = math.asin(max(-1.0, min(1.0, vs1 / max(1.0, v1))))
        gamma = 0.5 * (gamma0 + gamma1)
        speed = 0.5 * (v0 + v1)
        lift_actual = (speed * (gamma1 - gamma0) / dt
                       + gravity * math.cos(gamma)
                       - speed * speed / radius)

        # The model, with the air divided out: ``dec = cda * q / m``.
        q_over_m = dec / cda
        lift_model = cla * q_over_m * math.cos(math.radians(bank))
        if lift_model <= 0.0 or drag_actual <= 0.0:
            continue
        # A tick can carry a nonsense ``cla``/``cda`` -- the table is being
        # re-probed in flight and a row mid-refresh reads absurdly -- and one
        # such row drowns the column it lands in.  Nothing this airframe does
        # exceeds a few g of either.
        if dec > 200.0 or lift_model > 200.0:
            continue
        out.append((altitude, mach, b.get("aoa_c"), dec, drag_actual,
                    lift_model, lift_actual,
                    cla / cda, lift_actual / max(1e-6, drag_actual)))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("log")
    p.add_argument("--phase", default="GLIDE",
                   help="which phase's telemetry to audit (default GLIDE)")
    p.add_argument("--window", type=int, default=5,
                   help="half-width in ticks of the finite difference")
    p.add_argument("--rows", type=int, default=16)
    args = p.parse_args()

    rows = read(args.log, args.phase)
    if len(rows) < 2 * args.window + 2:
        print("%s: only %d %s lines" % (args.log, len(rows), args.phase))
        return 1

    table = audit(rows, args.window, args.rows)
    if not table:
        print("%s: nothing auditable" % args.log)
        return 1

    print("%s -- %d %s lines, window +/-%d ticks"
          % (args.log, len(rows), args.phase, args.window))
    print("    alt   Mach |   drag: model  actual  ratio "
          "|   lift: model  actual  ratio |   L/D: model  actual")
    for (alt, mach, _, dm, da, lm, la, ldm, lda) in table:
        print("%7.0f  %5.2f | %12.2f %7.2f %6.2f | %12.2f %7.2f %6.2f "
              "| %10.2f %7.2f"
              % (alt, mach, dm, da, da / dm if dm else 0.0,
                 lm, la, la / lm if lm else 0.0, ldm, lda))

    # One summary line, because a sweep of logs wants comparing.
    drag = [da / dm for (_, _, _, dm, da, _, _, _, _) in table if dm]
    lift = [la / lm for (_, _, _, _, _, lm, la, _, _) in table if lm]
    ratio = [lda / ldm for (_, _, _, _, _, _, _, ldm, lda) in table if ldm]

    def middle(xs):
        xs = sorted(xs)
        return xs[len(xs) // 2] if xs else float("nan")

    print("\nmedian actual/model -- drag %.2f  lift %.2f  L/D %.2f"
          % (middle(drag), middle(lift), middle(ratio)))
    print("A model the vehicle agrees with reads 1.00 in all three.  The L/D "
          "column is the one\nthat sets the range, and it is the one to read "
          "against the airframe's measured\npolar: 1.28-1.50 hypersonic, "
          "3.39 subsonic at best glide.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
