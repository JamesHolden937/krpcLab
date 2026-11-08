#!/usr/bin/env python3
"""Write stock jet craft files for testing the airplane pilot on more than
one airframe.

    ./airplane/tools/craftgen.py            # every design -> saves/craft/SPH/
    ./airplane/tools/craftgen.py trainer    # one

A design is a list of parts placed in the SPH frame (x right, y up, z
forward), each attached to a parent by a stack node or by its surface.
Every part's module and resource data is copied from the first instance
of that part in an existing craft file (``saves/craft/SPH``), so only stock
parts that some craft there already uses can be placed.

Orientations (measured from the craft files, the Electric Warbird's main
wing and tail):

* stack parts (cockpit, fuselage, intakes, engine): ``STACK`` -- the part's
  +y (its top node) forward;
* wing-type parts (wing connectors, swept wings, elevons, the airliner
  surface): span along the part's -x, chord along +y (leading edge
  forward), attach node at the origin.  ``LEFT`` puts the span to -x,
  ``RIGHT`` to +x, ``FIN`` up;
* gear: from the Warbird's own gear instances.

Check a new design by looking at it (``testInstances/shoot.py``) before
flying it.
"""
import argparse
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CRAFT = os.path.join(ROOT, "saves", "craft", "SPH")
TEMPLATES = ("Sport plane prototype 2", "Electric Warbird", "Arrow-E",
             "Seahawk", "Suborbital hopper")

S2 = math.sqrt(0.5)


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def qaxis(axis, deg):
    h = math.radians(deg) / 2.0
    s = math.sin(h)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h))


STACK = (S2, 0.0, 0.0, S2)                      # part +y forward
LEFT = STACK                                    # span to -x, flat
RIGHT = qmul(qaxis((0, 0, 1), 180.0), LEFT)     # span to +x
FIN = qmul(qaxis((0, 0, 1), -90.0), LEFT)       # span up


def dihedral(rot, deg, side):
    """A wing rotation tipped up ``deg`` (side -1 left, +1 right)."""
    return qmul(qaxis((0, 0, 1), side * deg), rot)


def parse_parts(path):
    text = open(path).read()
    blocks = re.split(r"\nPART\n", text)
    out = []
    for b in blocks[1:]:
        b = "PART\n" + b
        name = re.search(r"\n\tpart = (\S+)", b).group(1)
        out.append((name.rsplit("_", 1)[0], b))
    return text[:text.index("\nPART\n")], out


def templates():
    lib = {}
    for t in TEMPLATES:
        _, parts = parse_parts(os.path.join(CRAFT, t + ".craft"))
        for name, block in parts:
            lib.setdefault(name, block)
    # Gear rotations as the Warbird mounts them (wheel down).
    return lib


def block_end(block):
    """The block cut at its closing brace (the file's own trailing text,
    for the last part, dropped)."""
    depth = 0
    for i, ch in enumerate(block):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return block[:i + 1] + "\n"
    return block


def fmt(v):
    return ",".join("%.7g" % x for x in v)


class Design:
    def __init__(self, name):
        self.name = name
        self.parts = []          # dicts
        self.next_id = 4294000000

    def add(self, part, pos, rot, parent=None, stack=None, extra=None):
        """``stack``: (my node, parent node, my node y, parent node y) for a
        stack attachment; None for a surface one."""
        self.next_id -= 1000
        p = {"part": part, "id": "%s_%d" % (part, self.next_id),
             "pos": pos, "rot": rot, "parent": parent, "stack": stack,
             "children": [], "attN": [], "extra": extra or {}}
        if parent is not None:
            parent["children"].append(p)
            if stack is not None:
                mine, theirs, my_y, their_y = stack
                p["attN"].append((mine, parent["id"], my_y,
                                  1.0 if my_y >= 0 else -1.0))
                parent["attN"].append((theirs, p["id"], their_y,
                                       1.0 if their_y >= 0 else -1.0))
        self.parts.append(p)
        return p

    def pair(self, part, pos, rot_left, parent, rot_right=None):
        """A mirrored pair: ``pos`` is the left one's (x < 0)."""
        l = self.add(part, pos, rot_left, parent)
        r = self.add(part, (-pos[0], pos[1], pos[2]),
                     rot_right or qmul(qaxis((0, 0, 1), 180.0), rot_left)
                     if rot_right is None else rot_right,
                     parent if not isinstance(parent, tuple) else None)
        return l, r

    def write(self, lib, path):
        header, _ = parse_parts(os.path.join(CRAFT, TEMPLATES[0] + ".craft"))
        header = re.sub(r"ship = .*", "ship = " + self.name, header)
        header = re.sub(r"vesselType = .*", "vesselType = Plane", header)
        header = re.sub(r"persistentId = .*", "persistentId = %d"
                        % (abs(hash(self.name)) % 10 ** 9), header)
        out = [header.rstrip("\n") + "\n"]
        for n, p in enumerate(self.parts):
            b = block_end(lib[p["part"]])
            b = re.sub(r"\n\tpart = \S+", "\n\tpart = " + p["id"], b, count=1)
            b = re.sub(r"\n\tpersistentId = \d+", "\n\tpersistentId = %d"
                       % (1000000000 + 7919 * (n + 1)), b, count=1)
            b = re.sub(r"\n\tpos = .*", "\n\tpos = " + fmt(p["pos"]), b, count=1)
            b = re.sub(r"\n\tattPos = .*", "\n\tattPos = 0,0,0", b, count=1)
            b = re.sub(r"\n\tattPos0 = .*", "\n\tattPos0 = " + fmt(p["pos"]),
                       b, count=1)
            b = re.sub(r"\n\trot = .*", "\n\trot = " + fmt(p["rot"]), b, count=1)
            b = re.sub(r"\n\tattRot = .*", "\n\tattRot = 0,0,0,1", b, count=1)
            b = re.sub(r"\n\tattRot0 = .*", "\n\tattRot0 = " + fmt(p["rot"]),
                       b, count=1)
            b = re.sub(r"\n\tmir = .*", "\n\tmir = 1,1,1", b, count=1)
            for k, v in p["extra"].items():
                b = re.sub(r"\n\t%s = .*" % k, "\n\t%s = %s" % (k, v), b, count=1)
            b = re.sub(r"\n\t(link|attN|srfN|sym) = .*", "", b)
            lines = ["\tlink = " + c["id"] for c in p["children"]]
            for node, other, y, d in p["attN"]:
                g = "0|%g|0_0|%g|0" % (y, d)
                lines.append("\tattN = %s,%s_%s_%s" % (node, other, g, g))
            if p["parent"] is not None and p["stack"] is None:
                lines.append("\tsrfN = srfAttach,%s,COL,0|0|0,1|0|0,0|0|0"
                             % p["parent"]["id"])
            if lines:
                b = re.sub(r"(\n\tsameVesselCollision = .*|\n\tattm = .*)",
                           lambda m: m.group(1), b, count=1)
                i = b.index("\n\tEVENTS")
                b = b[:i] + "\n" + "\n".join(lines) + b[i:]
            out.append(b)
        with open(path, "w") as f:
            f.write("".join(out))


def fuselage(d, nose, sections, engine):
    """A Mk1 stack, nose forward from z=0: ``nose`` (a part with a bottom
    node), the inline cockpit, ``sections`` fuselage parts (each 1.875 m),
    ``engine`` (top node at its origin).  Returns (cockpit, [sections],
    engine part)."""
    cockpit = d.add("Mark2Cockpit", (0, 10, 0), STACK)
    if nose:
        d.add(nose, (0, 10, 0.9375 + 0.625), STACK, cockpit,
              ("bottom", "top", -0.625, 0.9375))
    prev, z, secs = cockpit, 0.0, []
    for s in sections:
        z -= 1.875
        prev = d.add(s, (0, 10, z), STACK, prev,
                     ("top", "bottom", 0.9375, -0.9375))
        secs.append(prev)
    eng = d.add(engine, (0, 10, z - 0.9375), STACK, prev,
                ("top", "bottom", 0.0, -0.9375))
    return cockpit, secs, eng


GEAR_DOWN = (0.0, 0.0, 0.0, 1.0)


DESIGNS = {}


def design(f):
    DESIGNS[f.__name__] = f
    return f


def wingpair(d, part, x, y, z, parent, dih=0.0):
    """A mirrored pair of a wing-type part, the left root at (x, y, z)."""
    l = d.add(part, (x, y, z), dihedral(LEFT, dih, -1), parent)
    r = d.add(part, (-x, y, z), dihedral(RIGHT, dih, 1), parent)
    return l, r

GEAR = (0.0, 1.0, 0.0, 0.0)             # LY-10 bay, wheel down (the Warbird's)
SURFACE_TE = 1.13                       # wingConnector3: chord +/-1.13 m
STAB_TE = 0.81                          # structuralWing4: chord -0.81..1.44


def swept(rot, deg, side):
    """A wing rotation swept back ``deg`` (side -1 left, +1 right)."""
    return qmul(qaxis((0, 1, 0), side * deg), rot)


def qrot(q, v):
    """Vector ``v`` rotated by ``q``."""
    x, y, z, w = qmul(qmul(q, (v[0], v[1], v[2], 0.0)),
                      (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def along(p0, q, v):
    d = qrot(q, v)
    return (p0[0] + d[0], p0[1] + d[1], p0[2] + d[2])


def tail(d, parent, z, elevators=True):
    """structuralWing4 stabiliser pair and fin on top, elevon3 elevators
    and rudder at their trailing edges, roots at fuselage station z."""
    if elevators:
        sl, sr = wingpair(d, "structuralWing4", -0.62, 10.0, z, parent)
        for s, x in ((sl, -1.12), (sr, 1.12)):
            d.add("elevon3", (x, 10.0, z - STAB_TE),
                  LEFT if x < 0 else RIGHT, s)
    fin = d.add("structuralWing4", (0.0, 10.62, z), FIN, parent)
    d.add("elevon3", (0.0, 10.62 + 0.5, z - STAB_TE), FIN, fin)


def wing(d, parent, z, y=9.5, root=0.375, panels=2, sweep=0.0, dih=0.0,
         aileron_panel=None):
    """wingConnector3 panels (2 m of span each, in the part's -x) out from
    the fuselage side at (root, y, z), swept and tipped up as asked, an
    elevon3 hinged on the trailing edge (1.13 m aft of the panel's origin)
    of the outermost (or ``aileron_panel``) one.  Returns the left and
    right panels."""
    out = ([], [])
    for side, k in ((-1, 0), (1, 1)):
        rot = swept(dihedral(LEFT if side < 0 else RIGHT, dih, side),
                    sweep, side)
        prev, pos = parent, (side * root, y, z)
        for i in range(panels):
            prev = d.add("wingConnector3", pos, rot, prev)
            out[k].append(prev)
            pos = along(pos, rot, (-2.0, 0.0, 0.0))
        sel = panels - 1 if aileron_panel is None else aileron_panel
        pnl = out[k][sel]
        d.add("elevon3", along(pnl["pos"], rot, (-1.0, -SURFACE_TE, 0.0)),
              rot, pnl)
    return out


@design
def trainer():
    """Conventional low-wing jet: Panther, two Mk1 intake fuselages, a
    straight wing of four wingConnector3 panels, a tail."""
    d = Design("KL Jet Trainer")
    cockpit, secs, eng = fuselage(d, "shockConeIntake",
                                  ["MK1IntakeFuselage", "MK1IntakeFuselage"],
                                  "turboJet")
    left, right = wing(d, secs[0], -3.3)
    tail(d, secs[1], -4.4)
    d.add("SmallGearBay", (0.0, 9.375, 0.3), GEAR, cockpit)
    d.add("SmallGearBay", (-1.4, 9.41, -3.25), GEAR, left[0])
    d.add("SmallGearBay", (1.4, 9.41, -3.25), GEAR, right[0])
    return d


@design
def delta():
    """Tailless: a wing swept 35 deg with elevons at its trailing edge (pitch
    and roll on the same surfaces), a fin and rudder, no stabiliser."""
    d = Design("KL Jet Delta")
    cockpit, secs, eng = fuselage(d, "shockConeIntake",
                                  ["MK1IntakeFuselage", "MK1IntakeFuselage"],
                                  "turboJet")
    left, right = wing(d, secs[0], -2.8, sweep=35.0, aileron_panel=1)
    tail(d, secs[1], -4.4, elevators=False)
    d.add("SmallGearBay", (0.0, 9.375, 0.3), GEAR, cockpit)
    d.add("SmallGearBay", (-1.4, 9.41, -3.25), GEAR, left[0])
    d.add("SmallGearBay", (1.4, 9.41, -3.25), GEAR, right[0])
    return d


@design
def canard():
    """The Sport plane's layout (canards on the cockpit, a short main wing
    aft) with a fin and gear: a craft whose pitch surface is ahead of the
    centre of mass."""
    d = Design("KL Jet Canard")
    cockpit, secs, eng = fuselage(d, "shockConeIntake",
                                  ["MK1IntakeFuselage", "MK1IntakeFuselage"],
                                  "turboJet")
    # Canards: elevon3 straight on the cockpit's sides.
    d.add("smallCtrlSrf", (-1.05, 10.0, 0.6), LEFT, cockpit)
    d.add("smallCtrlSrf", (1.05, 10.0, 0.6), RIGHT, cockpit)
    left, right = wing(d, secs[1], -3.9, y=9.9, root=0.62, sweep=20.0,
                       aileron_panel=0)
    fin = d.add("structuralWing4", (0.0, 10.62, -4.6), FIN, secs[1])
    d.add("elevon3", (0.0, 11.12, -4.6 - STAB_TE), FIN, fin)
    d.add("SmallGearBay", (0.0, 9.375, 0.3), GEAR, cockpit)
    d.add("SmallGearBay", (-1.6, 9.81, -4.2), GEAR, left[0])
    d.add("SmallGearBay", (1.6, 9.81, -4.2), GEAR, right[0])
    return d


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("names", nargs="*")
    p.add_argument("--check", metavar="INSTANCE", default=None,
                   help="also launch each on the runway of this farm "
                   "instance: parts loaded, mass, where the gear and the "
                   "control surfaces sit from the centre of mass, and "
                   "screenshots (logs/shots/craftgen/) from above, the side "
                   "and the front -- look at them before flying it")
    args = p.parse_args()
    lib = templates()
    for name in args.names or sorted(DESIGNS):
        d = DESIGNS[name]()
        path = os.path.join(CRAFT, d.name + ".craft")
        d.write(lib, path)
        print("%s: %d parts -> %s" % (name, len(d.parts), path))
        if args.check is not None:
            check(args.check, d)


def check(instance, d):
    """Launch ``d`` on the runway, say what loaded, photograph it."""
    import shutil
    import time
    sys.path.insert(0, ROOT)
    from common import paths
    paths.use_venv()
    import krpc
    sys.path.insert(0, os.path.join(ROOT, "testInstances"))
    import shoot
    rpc, stream = paths.ports(instance)
    sdir = os.path.join(paths.instance_dir(instance), "saves", "default",
                        "Ships", "SPH")
    shutil.copy(os.path.join(CRAFT, d.name + ".craft"), sdir)
    conn = krpc.connect(name="craftgen", rpc_port=rpc, stream_port=stream)
    sc = conn.space_center
    try:
        conn.krpc.paused = False
    except Exception:                                   # noqa: BLE001
        pass
    # From the flight scene (a launch from the space centre has hung on
    # the farm); crewed by name, or the pre-flight check can find the
    # cockpit empty ("NoControlSources") and wait on a dialog.
    from kspSim.tools import flighttest
    conn.close()
    conn = flighttest.load_paused(instance, "qs_warbird")
    sc = conn.space_center
    conn.krpc.paused = False
    sc.launch_vessel("SPH", d.name, "Runway", recover=True,
                     crew=["Jebediah Kerman"])
    time.sleep(5.0)
    v = sc.active_vessel
    vf = v.reference_frame            # x right, y nose, z down; CoM origin
    print("  loaded %d of %d parts, %.2f t, situation %s" % (
        len(v.parts.all), len(d.parts), v.mass / 1000.0,
        str(v.situation).split(".")[-1]))
    for part in v.parts.all:
        if part.name in ("SmallGearBay", "elevon3", "smallCtrlSrf",
                         "turboJet", "wingConnector3", "structuralWing4"):
            x, y, z = part.position(vf)
            print("    %-16s right %+5.2f fwd %+5.2f down %+5.2f" % (
                part.name, x, y, z))
    out = os.path.join(ROOT, "logs", "shots", "craftgen")
    os.makedirs(out, exist_ok=True)
    cam = sc.camera
    cam.mode = type(cam.mode).free           # heading is the world's here
    # The game writes a screenshot a frame or more late: the first one
    # here would show the camera as it was.
    sc.screenshot(shoot.wine_path(os.path.join(out, "_discard.png")), 1)
    for tag, heading, pitch in (("top", 0.0, 85.0), ("left", 0.0, 15.0),
                                ("front", 90.0, 15.0), ("rear", 270.0, 20.0),
                                ("right34", 135.0, 35.0)):
        cam.distance = 14.0
        cam.heading, cam.pitch = heading, pitch
        time.sleep(2.5)
        f = os.path.join(out, "%s_%s.png" % (d.name.replace(" ", ""), tag))
        sc.screenshot(shoot.wine_path(f), 1)
        print("    " + f)
    conn.close()


if __name__ == "__main__":
    sys.exit(main())
