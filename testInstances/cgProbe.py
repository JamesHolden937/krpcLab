#!/usr/bin/env python3
"""cgProbe.py <instance> [save] [--units N] -- where the CG is, and what it costs.

Answers "would moving the propellant help?" without flying: every part's mass
and centre of mass, the tanks that could hold fuel, and -- for several
placements of ``--units`` of propellant (nose tank as flown, aft tanks, wing
tanks, drained) -- the vehicle's CG along the body axis and the aerodynamic
pitching moment against alpha at neutral surfaces, as a fraction of the
pitch torque the control surfaces have.  That fraction is the part of the
elevons' travel spent on trim and not available to manoeuvre: a
placement that needs >1 cannot be trimmed at that alpha at all.

Method: ``Flight.simulate_aerodynamic_force_at`` / ``_torque_at`` at the
vehicle's own position and rotation, with the *airflow* rotated in the
symmetry plane to set alpha (so no quaternion is built).  The torque comes
back about the vehicle's current CoM; another CG's moment is
``M - (r_cg' - r_cg) x F``.  Both it and ``available_control_surface_torque``
scale with q, so the ratio holds at any speed of the same Mach.

Read-only: it loads the save (if given) and reports; it commands nothing.
"""
import math
import os
import sys

import krpc

here = os.path.dirname(os.path.abspath(__file__))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
units = 376.0
if "--units" in sys.argv:
    units = float(sys.argv[sys.argv.index("--units") + 1])
n = args[0]
base = os.path.join(here, "ksp%s" % n)
rpc = int(open(os.path.join(base, ".rpc_port")).read())
stream = int(open(os.path.join(base, ".stream_port")).read())
conn = krpc.connect(name="cgProbe", address="127.0.0.1",
                    rpc_port=rpc, stream_port=stream)
sc = conn.space_center
if len(args) > 1:
    sc.load(args[1])
v = sc.active_vessel
# **Paused, and in the vessel's own frame.**  A flying save moves 63 m/s
# while thirty parts are read one RPC at a time; in the body frame the late
# reads land metres forward of the early ones (the first version's CG check
# missed by 1.9 m).
try:
    conn.krpc.paused = True
except Exception:                                  # noqa: BLE001
    pass
vf = v.reference_frame
body = v.orbit.body
bf = body.reference_frame
KG_PER_UNIT = 5.0          # LiquidFuel and Oxidizer are both 5 kg/unit


def add(a, b): return tuple(x + y for x, y in zip(a, b))
def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def mul(a, k): return tuple(x * k for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])
def unit(a):
    m = math.sqrt(dot(a, a))
    return mul(a, 1.0 / m)


right = sc.transform_direction((1, 0, 0), v.reference_frame, bf)
fwd = sc.transform_direction((0, 1, 0), v.reference_frame, bf)
belly = sc.transform_direction((0, 0, 1), v.reference_frame, bf)
pos = v.position(bf)
com = (0.0, 0.0, 0.0)                     # vessel frame: the CoM
rot = v.rotation(bf)
print("vessel %s  %d parts  %.3f t (dry %.3f)"
      % (v.name, len(v.parts.all), v.mass / 1e3, v.dry_mass / 1e3))

# -- the parts, along the body axis -----------------------------------------
dry_moment = (0.0, 0.0, 0.0)
dry_mass = 0.0
tanks = []
for p in v.parts.all:
    # Vessel frame: the origin is the CoM and y is the body axis, so the
    # station is simply y (a body-frame round trip missed by 1.9 m).
    c = p.center_of_mass(vf)
    fuel = 0.0
    cap = 0.0
    for r in p.resources.all:
        if r.name in ("LiquidFuel", "Oxidizer"):
            fuel += r.amount
            cap += r.max
    # Everything but the propellant that could move: the part's whole mass
    # (monopropellant, crew, charge) less its LiquidFuel and Oxidizer.
    dm = (p.mass - fuel * KG_PER_UNIT) if not p.massless else 0.0
    dry_moment = add(dry_moment, mul(c, dm))
    dry_mass += dm
    station = c[1]
    if cap > 0:
        tanks.append((p, c, station, fuel, cap))
    if dm > 200 or cap > 0:
        print("  %-26s station %+7.2f m  dry %6.0f kg  fuel %6.0f/%-6.0f u"
              % (p.name, station, dm, fuel, cap))
dry_cg = mul(dry_moment, 1.0 / dry_mass)
now_fuel = [(c, f) for _, c, _, f, _ in tanks]
check = mul(add(dry_moment, tuple(sum(c[i] * f * KG_PER_UNIT
                                      for c, f in now_fuel) for i in range(3))),
            1.0 / (dry_mass + sum(f for _, f in now_fuel) * KG_PER_UNIT))
print("as loaded: summed CG station %+.2f m against kRPC's 0 "
      "(the check that the sum is right), summed mass %.3f t"
      % (check[1],
         (dry_mass + sum(f for _, f in now_fuel) * KG_PER_UNIT) / 1e3))
print("CG with no LF/Ox station %+.2f m (current CoM is 0 by construction)"
      % dry_cg[1])


def cg_for(placement):
    """placement: list of (centre, units)."""
    m = dry_mass
    mom = mul(dry_cg, dry_mass)
    for c, u in placement:
        m += u * KG_PER_UNIT
        mom = add(mom, mul(c, u * KG_PER_UNIT))
    return mul(mom, 1.0 / m), m


tanks.sort(key=lambda t: -t[2])
nose = tanks[0]
aft = [t for t in tanks if t[2] < nose[2] - 5]
placements = [("drained", [])]
placements.append(("nose tank", [(nose[1], units)]))
for t in aft:
    if t[4] >= units:
        placements.append(("%s @%+.1f" % (t[0].name, t[2]), [(t[1], units)]))
wings = [t for t in aft if "wing" in t[0].name.lower()]
if wings:
    per = units / len(wings)
    placements.append(("wings, split", [(t[1], per) for t in wings]))

# -- the pitch authority ----------------------------------------------------
flight = v.flight(bf)
q_now = flight.dynamic_pressure
surf = v.available_control_surface_torque   # ((+x,+y,+z),(-x,-y,-z)) N m
pitch_auth = min(abs(surf[0][0]), abs(surf[1][0]))
print("q now %.0f Pa, surface pitch torque %.0f kN m (%.1f N m per Pa)"
      % (q_now, pitch_auth / 1e3, pitch_auth / max(q_now, 1.0)))


def moments(speed, alt=None):
    """Nose-up moment about each placement's CG, per unit of pitch
    authority, for alpha 0..40."""
    p = pos
    if alt is not None:
        up = unit(pos)
        p = mul(up, body.equatorial_radius + alt)
    q = None
    rows = []
    for a in range(-4, 44, 4):
        ar = math.radians(a)
        vel = mul(add(mul(fwd, math.cos(ar)), mul(belly, math.sin(ar))), speed)
        F = flight.simulate_aerodynamic_force_at(body, p, vel, rot)
        M = flight.simulate_aerodynamic_torque_at(body, p, vel, rot,
                                                     (0.0, 0.0, 0.0))
        if q is None:
            rho = body.atmospheric_density_at_position(p, bf) \
                if hasattr(body, "atmospheric_density_at_position") else None
            q = 0.5 * rho * speed * speed if rho else None
        row = [a]
        for _, placement in placements:
            cg, _ = cg_for(placement)
            d = add(add(mul(right, cg[0]), mul(fwd, cg[1])), mul(belly, cg[2]))
            Mc = sub(M, cross(d, F))
            nose_up = -dot(Mc, right)    # +x rotation pitches the nose down
            row.append(nose_up)
        lift = -dot(F, belly)
        row.append(lift)
        rows.append(row)
    scale = pitch_auth / max(q_now, 1.0) * (q if q else q_now)
    return rows, q, scale


print("\nplacements of %.0f units (%.0f kg):" % (units, units * KG_PER_UNIT))
for name, placement in placements:
    cg, m = cg_for(placement)
    print("  %-34s CG station %+6.2f m  mass %.2f t"
          % (name, cg[1], m / 1e3))

for label, speed, alt in (("final, 100 m/s, here", 100.0, None),
                          ("final, 70 m/s, here", 70.0, None),
                          ("glide, Mach ~2, 25 km", 600.0, 25000.0),
                          ("entry, Mach ~6, 45 km", 1800.0, 45000.0)):
    try:
        rows, q, scale = moments(speed, alt)
    except Exception as exc:                       # noqa: BLE001
        print("\n%s: failed (%r)" % (label, exc))
        continue
    print("\n%s: nose-up moment / surface pitch authority (q %s)"
          % (label, "%.0f Pa" % q if q else "?"))
    print("  alpha " + " ".join("%14s" % nm[:14] for nm, _ in placements)
          + "   lift kN")
    for r in rows:
        print("  %5d " % r[0] + " ".join("%+14.2f" % (x / scale)
                                        for x in r[1:-1])
              + "   %7.0f" % (r[-1] / 1e3))
