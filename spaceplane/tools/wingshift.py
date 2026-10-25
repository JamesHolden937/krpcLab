#!/usr/bin/env python3
"""wingshift.py SOURCE OUT SHIFT -- move a shuttle's delta wings along its axis.

Every ``wingShuttleDelta`` in every vessel of the save, and every part
attached below it (elevons, fins, main gear), has its ``position`` y raised
by SHIFT metres: positive is forward.  The same parts, so the same mass and
lift; only the stability margin in pitch and yaw moves.  This is how
``qs_shuttle2_fwdwing_rigoff`` (0.6) and ``_fwdwing09_rigoff`` (0.9) were
made.

    ./spaceplane/tools/wingshift.py saves/qs_shuttle2_rigoff.sfs \\
        saves/qs_shuttle2_fwdwing12_rigoff.sfs 1.2
"""
import argparse
import re

WING = "wingShuttleDelta"


def shift_lines(lines, shift):
    """The save's lines with every wing subtree moved; the count moved."""
    out = list(lines)
    moved = 0
    vessel = None           # (parts, start) of the vessel being read
    parts = []              # per part: [name, parent, position line index]
    depth = 0
    in_vessel_depth = None
    part_depth = None

    def finish(parts):
        nonlocal moved
        children = {}
        for i, p in enumerate(parts):
            if p[1] is not None and p[1] != i:
                children.setdefault(p[1], []).append(i)
        todo = [i for i, p in enumerate(parts) if p[0] == WING]
        seen = set()
        while todo:
            i = todo.pop()
            if i in seen:
                continue
            seen.add(i)
            todo.extend(children.get(i, []))
        for i in sorted(seen):
            idx = parts[i][2]
            m = re.match(r"(\s*position = )([^,]+),([^,]+),(.+)$",
                         out[idx].rstrip("\n"))
            y = float(m.group(3)) + shift
            out[idx] = "%s%s,%r,%s\n" % (m.group(1), m.group(2), y,
                                         m.group(4))
            moved += 1

    for n, line in enumerate(lines):
        s = line.strip()
        if s == "{":
            depth += 1
            continue
        if s == "}":
            if part_depth is not None and depth == part_depth:
                part_depth = None
            if in_vessel_depth is not None and depth == in_vessel_depth:
                finish(parts)
                in_vessel_depth = None
            depth -= 1
            continue
        if s == "VESSEL":
            in_vessel_depth = depth + 1
            parts = []
        elif s == "PART" and in_vessel_depth is not None \
                and depth == in_vessel_depth:
            part_depth = depth + 1
            parts.append([None, None, None])
        elif part_depth is not None and depth == part_depth:
            key, _, val = s.partition(" = ")
            if key == "name" and parts[-1][0] is None:
                parts[-1][0] = val
            elif key == "parent":
                parts[-1][1] = int(val)
            elif key == "position":
                parts[-1][2] = n
    return out, moved


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("source")
    ap.add_argument("out")
    ap.add_argument("shift", type=float, help="metres forward")
    args = ap.parse_args()
    with open(args.source) as f:
        lines = f.readlines()
    out, moved = shift_lines(lines, args.shift)
    if not moved:
        raise SystemExit("no %s found" % WING)
    with open(args.out, "w") as f:
        f.writelines(out)
    print("%d parts moved %+.2f m" % (moved, args.shift))


if __name__ == "__main__":
    main()
