#!/usr/bin/env python3
"""Where the tarmac actually is, asked of the game rather than assumed.

The runway's two thresholds are configured coordinates and its centreline is
the line between them.  If either is wrong -- and the east one sits exactly
``RUNWAY_LENGTH_M`` from the west, which is what a *computed* coordinate
looks like -- then every cross-track number this project reports is measured
from the wrong line.

KSC's runway is a raised, flat strip: the grass beside it reads 2-3 m lower
in the flight logs.  So sample the surface height across the assumed
centreline and look for the plateau.
"""
import os, sys, math
sys.path.insert(0, "/home/holden/boosterlandKSP")
import krpc
from spaceplane.config import Config

inst = sys.argv[1] if len(sys.argv) > 1 else "0"
base = "/home/holden/boosterlandKSP/test_instances/ksp" + inst
rpc = int(open(os.path.join(base, ".rpc_port")).read())
stream = int(open(os.path.join(base, ".stream_port")).read())
conn = krpc.connect(name="runwayprobe", rpc_port=rpc, stream_port=stream)
body = conn.space_center.bodies["Kerbin"]
cfg = Config()

R = body.equatorial_radius
per_deg = math.pi / 180.0 * R

def height(lat, lon):
    return body.surface_height(lat, lon)

print("surface height across the assumed centreline (lat %.6f)" % cfg.RUNWAY_09_LAT)
mid_lon = 0.5 * (cfg.RUNWAY_09_LON + cfg.RUNWAY_27_LON)
for where, lon in (("west end ", cfg.RUNWAY_09_LON),
                   ("midpoint ", mid_lon),
                   ("east end ", cfg.RUNWAY_27_LON)):
    row = []
    for offset in range(-120, 121, 20):
        lat = cfg.RUNWAY_09_LAT + offset / per_deg
        row.append("%6.1f" % height(lat, lon))
    print("  %s %s" % (where, " ".join(row)))
print("  %s %s" % ("offset m ",
                   " ".join("%6d" % o for o in range(-120, 121, 20))))

print()
print("along the assumed centreline, every 300 m:")
row = []
for step in range(-1500, 1501, 300):
    lon = mid_lon + step / per_deg
    row.append("%6.1f" % height(cfg.RUNWAY_09_LAT, lon))
print("  height   %s" % " ".join(row))
print("  from mid %s" % " ".join("%6d" % s for s in range(-1500, 1501, 300)))
conn.close()
