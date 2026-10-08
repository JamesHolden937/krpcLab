#!/usr/bin/env python3
"""Deployed surface pairs as trim tabs: pitch moment, lift, drag per config.
Scratch diagnostic.  Loads a save on an instance, holds a synthetic state."""
import math, os, sys, time
sys.path.insert(0, "/home/holden/krpcLab")
import krpc
from common import vec

inst = sys.argv[1] if len(sys.argv) > 1 else "0"
save = sys.argv[2] if len(sys.argv) > 2 else "qs_s2_hac0"
d = "/home/holden/krpcLab/testInstances/ksp" + inst
rp, sp = int(open(d + "/.rpc_port").read()), int(open(d + "/.stream_port").read())
c = krpc.connect(name="tab-loader", rpc_port=rp, stream_port=sp)
c.space_center.load(save)
c.close()
conn = None
for _ in range(60):
    time.sleep(2)
    try:
        conn = krpc.connect(name="tabprobe", rpc_port=rp, stream_port=sp)
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
pos, rot = v.position(F), v.rotation(F)
def bdir(x):
    return sc.transform_direction(x, v.reference_frame, F)
nose, belly, right = bdir((0, 1, 0)), bdir((0, 0, 1)), bdir((1, 0, 0))
roof = vec.scale(belly, -1)
print("alt %.0f mass %.2f t" % (fl.mean_altitude, v.mass / 1000))

groups = {"canard": [], "elevon1": [], "elevon2": []}
ONLY = "canard"
for cs in v.parts.control_surfaces:
    y = cs.part.position(v.reference_frame)[1]
    t = cs.part.title
    m = [x for x in cs.part.modules if "ControlSurface" in x.name][0]
    if y > 5:
        groups["canard"].append((cs, m))
    elif "Elevon 1" in t:
        groups["elevon1"].append((cs, m))
    elif "Elevon 2" in t:
        groups["elevon2"].append((cs, m))
    cs.deployed = False

def measure():
    rows = []
    for spd in (120.0,):
        for a in (5.0, 12.0, 18.0, 24.0, 30.0, 38.0):
            ar = math.radians(a)
            vel = vec.scale(vec.add(vec.scale(nose, math.cos(ar)), vec.scale(roof, -math.sin(ar))), spd)
            f = fl.simulate_aerodynamic_force_at(body, pos, vel, rot)
            tq = fl.simulate_aerodynamic_torque_at(body, pos, vel, rot, (0.0, 0.0, 0.0))
            u = vec.unit(vel)
            drag = -vec.dot(f, u)
            lift = vec.dot(f, vec.unit(vec.cross(vel, right)))
            rows.append((spd, a, lift / 1e3, drag / 1e3, vec.dot(tq, right) / 1e3))
    return rows

def settle():
    sc.paused = False
    time.sleep(1.2)
    sc.paused = True
    print('   [alt %.0f spd %.0f %s]' % (fl.mean_altitude, fl.speed, v.situation))

settle()
base = measure()
print("baseline (all stowed):")
for r in base:
    print("  v=%3.0f a=%4.1f L=%7.1f kN D=%6.1f kN M(right)=%+8.1f kNm" % r)
for name, members in [(k, g) for k, g in groups.items() if k == ONLY]:
    for ang in (15.0, -15.0):
        for cs, m in members:
            m.set_field_float("Deploy Angle", ang)
            cs.deployed = True
        settle()
        rows = measure()
        print("%s deployed %+.0f (n=%d):" % (name, ang, len(members)))
        for b, r in zip(base, rows):
            print("  v=%3.0f a=%4.1f dL=%+7.1f dD=%+6.1f dM=%+8.1f kNm  (L/D %.2f -> %.2f)" % (
                r[0], r[1], r[2] - b[2], r[3] - b[3], r[4] - b[4], b[2] / max(b[3],1e-6), r[2] / max(r[3],1e-6)))
        for cs, m in members:
            cs.deployed = False
        settle()
# reference: full nose-up pitch input, all surfaces stowed
v.control.pitch = 1.0
settle()
rows = measure()
print("pitch input +1.0 (all surfaces):")
for b, r in zip(base, rows):
    print("  v=%3.0f a=%4.1f dL=%+7.1f dD=%+6.1f dM=%+8.1f kNm" % (r[0], r[1], r[2] - b[2], r[3] - b[3], r[4] - b[4]))
v.control.pitch = 0.0
conn.close()
