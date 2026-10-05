#!/usr/bin/env python3
"""Which part reaches the ground before the wheels, asked of the game.

Loads a save, lowers the gear, and reads every part's bounding box in the
vessel frame (x right, y forward, z out of the bottom).  Then, for a range of
body pitch and bank about the main wheels' contact line, prints which non-wheel
part comes lowest and how far it stays above the plane the main tyres touch.
At ~45 m/s any part that reaches the runway exceeds its crash tolerance.

    testInstances/gearProbe.py 0 qs_s2_hac0
"""
import os, sys, math, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import krpc

inst = sys.argv[1] if len(sys.argv) > 1 else "0"
save = sys.argv[2] if len(sys.argv) > 2 else "qs_s2_hac0"
base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ksp" + inst)
rpc = int(open(os.path.join(base, ".rpc_port")).read())
stream = int(open(os.path.join(base, ".stream_port")).read())
conn = krpc.connect(name="gearprobe", rpc_port=rpc, stream_port=stream)
sc = conn.space_center
sc.load(save)
time.sleep(3)
v = sc.active_vessel
v.control.gear = True
t0 = sc.ut
while sc.ut - t0 < 6:
    time.sleep(0.2)
frame = v.reference_frame
rows = []
for p in v.parts.all:
    try:
        lo, hi = p.bounding_box(frame)
    except Exception:
        continue
    corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
               for z in (lo[2], hi[2])]
    rows.append((p.title, p.wheel is not None, p.position(frame), corners))
sc.quickload() if False else None
wheels = [r for r in rows if r[1]]
for t, _, pos, c in wheels:
    print("wheel %-22s pos (%+.2f %+.2f %+.2f) bottom z %.2f" %
          (t[:22], pos[0], pos[1], pos[2], max(k[2] for k in c)))
mains = [r for r in wheels if r[2][1] < 0]
ground_z = max(max(k[2] for k in r[3]) for r in mains)
axle_y = sum(r[2][1] for r in mains) / len(mains)
print("main contact plane z %.2f at y %.2f" % (ground_z, axle_y))
print()
print("lowest non-wheel parts, gear level (z below the contact plane is +):")
others = [r for r in rows if not r[1]]
def clearance(corners, pitch, bank):
    # rotate about the main-wheel contact line; nose-up pitch lowers the aft
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cb, sb = math.cos(math.radians(bank)), math.sin(math.radians(bank))
    best = 1e9
    for x, y, z in corners:
        dy, dz = y - axle_y, z - ground_z
        # pitch: nose up raises +y (z decreases) and lowers the aft
        z1 = dz * cp - dy * sp
        # bank right wing down: +x goes down (z increases)
        z2 = z1 * cb - x * sb
        best = min(best, -z2)
    return best
for t, _, pos, c in sorted(others, key=lambda r: clearance(r[3], 0, 0)):
    print("  %-30s pos (%+.2f %+.2f %+.2f) x %+.2f..%+.2f y %+.2f..%+.2f clearance %.2f m, at pitch 8: %.2f" %
          (t[:30], pos[0], pos[1], pos[2], min(k[0] for k in c), max(k[0] for k in c), min(k[1] for k in c), max(k[1] for k in c), clearance(c, 0, 0), clearance(c, 8, 0)))
print()
print("pitch/bank -> first part to touch, its clearance (m)")
for bank in (0, 4, -4, 8):
    for pitch in (0, 3, 5, 7, 9, 11, 13):
        worst = min(others, key=lambda r: clearance(r[3], pitch, bank))
        print("  pitch %+3d bank %+2d  %-30s %+.2f" %
              (pitch, bank, worst[0][:30], clearance(worst[3], pitch, bank)))
conn.close()
