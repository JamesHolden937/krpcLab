#!/usr/bin/env python3
"""Sample the ground around the KSC finely, once, for every model.

    ./kspSim/tools/kscterrain.py --instance 0

The models' terrain grids step 0.025 deg (~260 m) and smooth away what an
autopilot lands on: the launch pad is a 5 m mound (72.4 m against 67.6 m
thirty metres off), and a booster that expected the pad hovered over the
sim's lower ground until it tipped over.  This writes
``kspSim/data/ksc_terrain.json``: the pad at 0.0002 deg (2 m), the runway
strip at 0.0005 deg (5 m), from the game's ``CelestialBody.surface_height``.
``world.Terrain`` looks here first.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

from kspSim.tools import flighttest  # noqa: E402

BOXES = [("pad", -0.1072, -0.0872, -74.5677, -74.5477, 0.0002),
         ("runway", -0.0560, -0.0420, -74.7300, -74.4900, 0.0005)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", default="0")
    args = ap.parse_args()
    conn = flighttest.connect(args.instance, "kscterrain")
    body = conn.space_center.bodies["Kerbin"]
    grids = []
    for name, la0, la1, lo0, lo1, step in BOXES:
        nla = int(round((la1 - la0) / step)) + 1
        nlo = int(round((lo1 - lo0) / step)) + 1
        heights = [[body.surface_height(la0 + i * step, lo0 + j * step) for j in range(nlo)]
                   for i in range(nla)]
        grids.append({"name": name, "lat0": la0, "dlat": step, "nlat": nla,
                      "lon0": lo0, "dlon": step, "nlon": nlo, "heights": heights})
        print("%s: %d x %d" % (name, nla, nlo), flush=True)
    conn.close()
    path = os.path.join(ROOT, "kspSim", "data", "ksc_terrain.json")
    with open(path, "w") as fh:
        json.dump(grids, fh)
    print("wrote", path)


if __name__ == "__main__":
    main()
