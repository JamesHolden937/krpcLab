#!/usr/bin/env python3
"""Put a different booster into a separation save: a new airframe, same state.

TEMPORARY TEST HARNESS -- not part of the flight software.

A booster is tested on the state it separates in, and a new airframe is only
ever built in the VAB, on the pad, stacked under something.  This takes the
stack as the game saved it (any save holding it -- FMRS's ``before_launch``
is the one the live game writes on every launch), cuts out the subtree below
one decoupler, and puts it in place of the active vessel of a separation
save, keeping that save's position, velocity and attitude.

The cut is clean because KSP stores a vessel as a tree: the part directly
under the decoupler becomes the new root, every index is shifted down by its
old one, references that leave the subtree become ``-1``, and positions are
re-expressed relative to the new root.  The new root must have an identity
rotation in the stack (a stacked tank does), which this checks.

The propellant is set to a target *mass*, because that is what a flight log
records at separation (``m=`` on the SEPARATION line): every tank holding
LiquidFuel/Oxidizer is filled to the same fraction.

    ./boosterland/tools/splicebooster.py \\
        --stack "~/Kerbal Space Program/saves/default/FMRS/before_launch.sfs" \\
        --decoupler Decoupler.3 --mass 99.36 --name "heavy booster" \\
        --state saves/quicksave.sfs -o saves/qs_heavy.sfs
"""

import argparse
import os
import re
import sys

RESOURCE_DENSITY_T = {"LiquidFuel": 0.005, "Oxidizer": 0.005,
                      "MonoPropellant": 0.004, "ElectricCharge": 0.0,
                      "Ore": 0.010}


def vessels(lines):
    """``[(start, end)]`` of the FLIGHTSTATE's top-level VESSEL blocks."""
    out = []
    i = 0
    while i < len(lines):
        if lines[i] == "\t\tVESSEL":
            j = i
            while lines[j] != "\t\t}":
                j += 1
            out.append((i, j))
            i = j
        i += 1
    return out


def parts_of(lines, start, end):
    """``[(start, end)]`` of the PART blocks directly in a vessel."""
    out = []
    i = start
    while i < end:
        if lines[i] == "\t\t\tPART":
            j = i
            while lines[j] != "\t\t\t}":
                j += 1
            out.append((i, j))
            i = j
        i += 1
    return out


def field(block, name):
    for line in block:
        if line.startswith("\t\t\t\t%s = " % name):
            return line.split(" = ", 1)[1]
    return None


def stack_vessel(lines, name):
    for s, e in vessels(lines):
        for k in range(s, e):
            if lines[k] == "\t\t\tname = %s" % name:
                return s, e
            if lines[k] == "\t\t\tPART":
                break
    return None


def subtree(blocks, root):
    """Indices of ``root`` and everything below it, by parent links."""
    parent = [int(field(b, "parent")) for b in blocks]
    keep = {root}
    changed = True
    while changed:
        changed = False
        for i, p in enumerate(parent):
            if i not in keep and p in keep and i != p:
                keep.add(i)
                changed = True
    return sorted(keep)


def remap_part(block, index_map, root_new, origin):
    """One PART block with its references re-indexed and its position moved."""
    out = []
    for line in block:
        m = re.match(r"^(\t\t\t\t)(parent|sym) = (-?\d+)$", line)
        if m:
            old = int(m.group(3))
            new = index_map.get(old, -1)
            if m.group(2) == "parent" and new == -1:
                new = root_new
            out.append("%s%s = %d" % (m.group(1), m.group(2), new))
            continue
        m = re.match(r"^(\t\t\t\t)(attN|srfN) = ([^,]*), (-?\d+)(.*)$", line)
        if m:
            old = int(m.group(4))
            new = index_map.get(old, -1) if old >= 0 else -1
            out.append("%s%s = %s, %d%s" % (m.group(1), m.group(2),
                                            m.group(3), new, m.group(5)))
            continue
        m = re.match(r"^(\t\t\t\tposition = )(.*)$", line)
        if m:
            p = [float(x) for x in m.group(2).split(",")]
            out.append("%s%r,%r,%r" % (m.group(1), p[0] - origin[0],
                                       p[1] - origin[1], p[2] - origin[2]))
            continue
        out.append(line)
    return out


def set_propellant(block, fraction):
    """Fill LiquidFuel/Oxidizer to ``fraction`` of capacity; return the mass."""
    out, mass, name = [], 0.0, None
    for line in block:
        m = re.match(r"^\t\t\t\t\tname = (\w+)$", line)
        if m:
            name = m.group(1)
        m = re.match(r"^(\t\t\t\t\tmaxAmount = )(.*)$", line)
        if m and name in ("LiquidFuel", "Oxidizer"):
            mass += float(m.group(2)) * fraction * RESOURCE_DENSITY_T[name]
        out.append(line)
    # Second pass: rewrite amount from the maxAmount that follows it.
    result = []
    for i, line in enumerate(out):
        m = re.match(r"^(\t\t\t\t\tamount = )(.*)$", line)
        if m:
            res = None
            for k in range(i, -1, -1):
                n = re.match(r"^\t\t\t\t\tname = (\w+)$", out[k])
                if n:
                    res = n.group(1)
                    break
            if res in ("LiquidFuel", "Oxidizer"):
                cap = next(float(out[k].split(" = ")[1])
                           for k in range(i, len(out))
                           if out[k].startswith("\t\t\t\t\tmaxAmount = "))
                line = "%s%r" % (m.group(1), cap * fraction)
        result.append(line)
    return result, mass


def other_mass(block):
    """Dry mass plus every resource that is not LiquidFuel/Oxidizer."""
    mass = float(field(block, "mass") or 0.0)
    name = None
    for line in block:
        m = re.match(r"^\t\t\t\t\tname = (\w+)$", line)
        if m:
            name = m.group(1)
        m = re.match(r"^\t\t\t\t\tamount = (.*)$", line)
        if m and name not in ("LiquidFuel", "Oxidizer"):
            mass += float(m.group(1)) * RESOURCE_DENSITY_T.get(name, 0.0)
    return mass


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stack", required=True, help="save holding the stack")
    p.add_argument("--vessel", default=None,
                   help="the stack's vessel name (default: the active one)")
    p.add_argument("--decoupler", required=True,
                   help="part name of the decoupler the booster hangs from")
    p.add_argument("--state", required=True,
                   help="separation save whose active vessel is replaced")
    p.add_argument("--mass", type=float, required=True,
                   help="booster mass at separation, tonnes")
    p.add_argument("--name", required=True, help="the new vessel's name")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args(argv)

    src = open(os.path.expanduser(args.stack)).read().split("\n")
    if args.vessel is None:
        active = int(next(l for l in src if l.strip().startswith(
            "activeVessel = ")).split("=")[1])
        vs, ve = vessels(src)[active]
    else:
        found = stack_vessel(src, args.vessel)
        if found is None:
            raise SystemExit("no vessel %r in the stack save" % args.vessel)
        vs, ve = found
    pblocks = [src[s:e + 1] for s, e in parts_of(src, vs, ve)]
    names = [field(b, "name") for b in pblocks]
    decouplers = [i for i, n in enumerate(names) if n == args.decoupler]
    if len(decouplers) != 1:
        raise SystemExit("%d parts named %s" % (len(decouplers), args.decoupler))
    below = [i for i, b in enumerate(pblocks)
             if int(field(b, "parent")) == decouplers[0]]
    if len(below) != 1:
        raise SystemExit("%d parts hang from the decoupler" % len(below))
    root = below[0]
    rot = [float(x) for x in field(pblocks[root], "rotation").split(",")]
    if max(abs(rot[0]), abs(rot[1]), abs(rot[2])) > 1e-4:
        raise SystemExit("new root %s is rotated in the stack (%s)"
                         % (names[root], rot))
    origin = [float(x) for x in field(pblocks[root], "position").split(",")]
    keep = subtree(pblocks, root)
    index_map = {old: new for new, old in enumerate(keep)}
    booster = [remap_part(pblocks[i], index_map, 0, origin) for i in keep]

    fixed = sum(other_mass(b) for b in booster)
    capacity = sum(set_propellant(b, 1.0)[1] for b in booster)
    fraction = (args.mass - fixed) / capacity
    if not 0.0 <= fraction <= 1.0:
        raise SystemExit("mass %.2f t needs %.1f%% propellant (dry %.2f, "
                         "capacity %.2f)" % (args.mass, 100 * fraction,
                                             fixed, capacity))
    booster = [set_propellant(b, fraction)[0] for b in booster]

    # The control point: the first command part in the subtree.
    ref = None
    for b in booster:
        if any(l.strip() == "name = ModuleCommand" for l in b):
            ref = field(b, "uid")
            break

    dst = open(args.state).read().split("\n")
    active = int(next(l for l in dst if l.strip().startswith(
        "activeVessel = ")).split("=")[1])
    ds, de = vessels(dst)[active]
    old_parts = parts_of(dst, ds, de)
    first, last = old_parts[0][0], old_parts[-1][1]
    header = dst[ds:first]
    header = [("\t\t\tname = %s" % args.name) if l.startswith("\t\t\tname = ")
              else ("\t\t\tref = %s" % ref) if (ref and l.startswith("\t\t\tref = "))
              else l for l in header]
    new_block = header + [l for b in booster for l in b] + dst[last + 1:de + 1]
    out = dst[:ds] + new_block + dst[de + 1:]
    with open(args.out, "w") as f:
        f.write("\n".join(out))
    print("%s: %d parts from %s, root %s, propellant %.1f%% "
          "(dry+other %.2f t, propellant %.2f t), control %s"
          % (args.out, len(booster), names[decouplers[0]], names[root],
             100 * fraction, fixed, capacity * fraction, ref))


if __name__ == "__main__":
    sys.exit(main())
