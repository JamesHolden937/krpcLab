#!/usr/bin/env python3
"""Cargo bay doors as a speedbrake: aero force (and torque) closed vs open,
at the vessel's own position, over alpha and speed.  Scratch diagnostic."""
import math, os, sys, time
ROOT = "/home/holden/krpcLab"
sys.path.insert(0, ROOT)
import krpc
from common import vec

inst = sys.argv[1] if len(sys.argv) > 1 else "0"
save = sys.argv[2] if len(sys.argv) > 2 else "qs_s2_hac0"
d = os.path.join(ROOT, "testInstances", "ksp" + inst)
rp, sp = int(open(d + "/.rpc_port").read()), int(open(d + "/.stream_port").read())
c = krpc.connect(name="bay-loader", rpc_port=rp, stream_port=sp)
c.space_center.load(save)
c.close()
conn = None
for _ in range(60):
    time.sleep(2)
    try:
        conn = krpc.connect(name="bayprobe", rpc_port=rp, stream_port=sp)
        v = conn.space_center.active_vessel
        if v.flight(v.orbit.body.reference_frame).speed > 1:
            break
    except Exception:
        conn = None
sc = conn.space_center
v = sc.active_vessel
body = v.orbit.body
F = body.reference_frame
fl = v.flight(F)
sc.paused = True
pos = v.position(F)
rot = v.rotation(F)
alt0 = fl.mean_altitude
def bdir(x):
    return sc.transform_direction(x, v.reference_frame, F)
nose, belly, right = bdir((0, 1, 0)), bdir((0, 0, 1)), bdir((1, 0, 0))
roof = vec.scale(belly, -1)
has_torque = hasattr(fl, "simulate_aerodynamic_torque_at")
print("vessel", v.name, "mass %.2f t" % (v.mass / 1000), "alt %.0f" % alt0,
      "bays", len(v.parts.cargo_bays), "torque_api", has_torque)
for b in v.parts.cargo_bays:
    print(" bay", b.part.title, "open", b.open, "state", b.state)

def table(tag):
    rows = {}
    for spd in (90.0, 150.0, 220.0):
        for a in (0.0, 5.0, 10.0, 15.0, 20.0):
            ar = math.radians(a)
            flow = vec.add(vec.scale(nose, math.cos(ar)), vec.scale(roof, -math.sin(ar)))
            vel = vec.scale(flow, spd)
            f = fl.simulate_aerodynamic_force_at(body, pos, vel, rot)
            drag = -vec.dot(f, vec.unit(vel))
            lift = vec.dot(f, vec.unit(vec.cross(vel, right)))  # perpendicular, in symmetry plane
            t = None
            if has_torque:
                try:
                    tq = fl.simulate_aerodynamic_torque_at(body, pos, vel, rot)
                    t = vec.dot(tq, right)
                except Exception as e:
                    t = None
            rows[(spd, a)] = (lift, drag, t)
    return rows

closed = table("closed")
sc.paused = False
for b in v.parts.cargo_bays:
    b.open = True
time.sleep(6)
sc.paused = True
for b in v.parts.cargo_bays:
    print(" bay after", b.open, b.state)
opened = table("open")
print("spd  alpha |  L_c kN   D_c kN  L/D_c |  L_o kN   D_o kN  L/D_o | dD %%  dL %%  dM kNm")
for k in sorted(closed):
    lc, dc, tc = closed[k]
    lo, do, to = opened[k]
    dm = (to - tc) / 1000 if (tc is not None and to is not None) else float("nan")
    print("%4.0f %5.1f | %7.1f %8.1f %6.2f | %7.1f %8.1f %6.2f | %+5.0f %+5.0f %+7.1f" % (
        k[0], k[1], lc / 1e3, dc / 1e3, lc / dc if dc else 0, lo / 1e3, do / 1e3,
        lo / do if do else 0, 100 * (do - dc) / dc, 100 * (lo - lc) / (abs(lc) + 1e-9), dm))
for b in v.parts.cargo_bays:
    b.open = False
conn.close()
