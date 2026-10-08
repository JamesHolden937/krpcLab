#!/usr/bin/env python3
"""Upright vs inverted vs knife-edge: drag at 1 g of lift, from the game's own
aero (simulate_aerodynamic_force_at / _torque_at).  Scratch diagnostic.
Surfaces neutral; the vessel is paused right after the load."""
import math, sys, time
sys.path.insert(0, "/home/holden/krpcLab")
import krpc
from common import vec

inst = sys.argv[1] if len(sys.argv) > 1 else "0"
save = sys.argv[2] if len(sys.argv) > 2 else "qs_s2_hac0"
d = "/home/holden/krpcLab/testInstances/ksp" + inst
rp, sp = int(open(d + "/.rpc_port").read()), int(open(d + "/.stream_port").read())
c = krpc.connect(name="inv-loader", rpc_port=rp, stream_port=sp)
c.space_center.load(save)
c.close()
conn = None
for _ in range(60):
    time.sleep(2)
    try:
        conn = krpc.connect(name="invprobe", rpc_port=rp, stream_port=sp)
        v = conn.space_center.active_vessel
        if v.flight(v.orbit.body.reference_frame).speed > 1 and len(v.parts.control_surfaces) > 0:
            break
        conn.close(); conn = None
    except Exception:
        conn = None
sc = conn.space_center
sc.paused = True
v = sc.active_vessel
body = v.orbit.body
F = body.reference_frame
fl = v.flight(F)
rot = v.rotation(F)
pos0 = v.position(F)
pos = vec.scale(vec.unit(pos0), body.equatorial_radius + 2000.0)
def bdir(x):
    return sc.transform_direction(x, v.reference_frame, F)
nose, belly, right = bdir((0, 1, 0)), bdir((0, 0, 1)), bdir((1, 0, 0))
roof = vec.scale(belly, -1)
W = v.mass * 9.81
print("mass %.2f t  W %.0f kN  at 2000 m" % (v.mass / 1e3, W / 1e3))

def sample(spd, alpha, beta=0.0):
    a, b = math.radians(alpha), math.radians(beta)
    # flow direction in body terms: along nose, from below (alpha>0), from the right (beta>0)
    flow = vec.add(vec.add(vec.scale(nose, math.cos(a) * math.cos(b)),
                           vec.scale(roof, -math.sin(a) * math.cos(b))),
                   vec.scale(right, -math.sin(b)))
    vel = vec.scale(vec.unit(flow), spd)
    f = fl.simulate_aerodynamic_force_at(body, pos, vel, rot)
    t = fl.simulate_aerodynamic_torque_at(body, pos, vel, rot, (0.0, 0.0, 0.0))
    u = vec.unit(vel)
    drag = -vec.dot(f, u)
    # lift toward the roof, perpendicular to the flow; side force toward the right
    lroof = vec.unit(vec.sub(roof, vec.scale(u, vec.dot(roof, u))))
    lright = vec.unit(vec.sub(right, vec.scale(u, vec.dot(right, u))))
    return vec.dot(f, lroof), drag, vec.dot(f, lright), vec.dot(t, right)

def at_lift(rows, target):
    """Interpolate the first crossing of lift == target in a list of (x, L, D, S, M)."""
    for (x0, l0, d0, s0, m0), (x1, l1, d1, s1, m1) in zip(rows, rows[1:]):
        if (l0 - target) * (l1 - target) <= 0 and l1 != l0:
            k = (target - l0) / (l1 - l0)
            return (x0 + k * (x1 - x0), d0 + k * (d1 - d0), m0 + k * (m1 - m0))
    return None

for spd in (80.0, 100.0, 120.0):
    rows = []
    for alpha in range(-45, 46):
        L, D, S, M = sample(spd, float(alpha))
        rows.append((alpha, L, D, S, M))
    up = at_lift([r for r in rows if r[0] >= -10], W)
    inv = at_lift(list(reversed([r for r in rows if r[0] <= 10])), -W)
    zero = [r for r in rows if r[0] == 0][0]
    maxL = max(r[1] for r in rows); minL = min(r[1] for r in rows)
    srows = []
    for beta in range(0, 61, 2):
        L, D, S, M = sample(spd, 0.0, float(beta))
        srows.append((beta, -S, D, L, M))   # side force toward the left (beta>0 flow from right)
    kn = at_lift(srows, W)
    print("\n%3.0f m/s: lift at alpha 0 = %+.0f kN (camber/incidence), max %+.0f, min %+.0f kN" % (
        spd, zero[1] / 1e3, maxL / 1e3, minL / 1e3))
    for name, res in (("upright  ", up), ("inverted ", inv)):
        if res:
            print("  %s 1 g at body alpha %+5.1f deg: drag %6.1f kN  L/D %4.2f  pitch moment %+7.1f kN m (+ nose-down)" % (
                name, res[0], res[1] / 1e3, W / res[1], res[2] / 1e3))
        else:
            print("  %s cannot make 1 g in -45..45 deg" % name)
    if kn:
        print("  knife-edge 1 g at sideslip %4.1f deg: drag %6.1f kN  L/D %4.2f" % (kn[0], kn[1] / 1e3, W / kn[1]))
    else:
        print("  knife-edge cannot make 1 g of side force by 60 deg of sideslip (max %.0f kN)" % (
            max(r[1] for r in srows) / 1e3))
    print("  upright, for reference: drag at alpha 10 / 15 / 20 = %s kN, lift %s kN" % (
        " / ".join("%.0f" % (r[2] / 1e3) for r in rows if r[0] in (10, 15, 20)),
        " / ".join("%.0f" % (r[1] / 1e3) for r in rows if r[0] in (10, 15, 20))))
conn.close()
