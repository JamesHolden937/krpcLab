#!/usr/bin/env python3
"""Write a craft file with one more stack tank: a second vehicle to test on.

    ./ascent/tools/stretchcraft.py "saves/craft/VAB/Untitled Space Craft.craft" \\
        Rockomax32.BW_4293107636 "saves/craft/VAB/stack small stretched.craft"

A copy of the named tank part goes in directly above it -- the copy takes
the original's place under its parent, and the original and everything
under it move down by the tank's node spacing.  The copy carries the
tank's resources and modules but none of its surface children.  KSP places
a launched craft by its own bounds, so lengthening the bottom is safe.
"""

import random
import re
import sys


def blocks(text):
    """(start, end, part id) of every top-level PART block."""
    out = []
    for m in re.finditer(r"\nPART\n\{\n", text):
        start = m.start() + 1
        depth, i = 0, m.end() - 2
        while True:
            c = text[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        body = text[start:i + 1]
        pid = re.search(r"\tpart = (\S+)", body).group(1)
        out.append((start, i + 1, pid))
    return out


def field(body, key):
    return re.search(r"\n\t%s = (.*)" % key, body).group(1)


def main(src, tank, dst):
    raw = open(src, "rb").read()
    newline = "\r\n" if b"\r\n" in raw else "\n"
    text = raw.decode("utf-8", "surrogateescape").replace("\r\n", "\n")
    parts = {pid: text[s:e] for s, e, pid in blocks(text)}
    if tank not in parts:
        sys.exit("no part %s" % tank)
    body = parts[tank]
    top = [a for a in re.findall(r"\n\tattN = (.*)", body)
           if a.startswith("top,")]
    bottom = [a for a in re.findall(r"\n\tattN = (.*)", body)
              if a.startswith("bottom,")]
    if not top or not bottom:
        sys.exit("%s has no top and bottom stack nodes" % tank)
    parent = re.match(r"(.+?_\d+)_", top[0].split(",", 1)[1]).group(1)
    # node offsets "x|y|z" are the second _-field of the attN value
    y_top = float(top[0].split(",", 1)[1].split("_")[-4].split("|")[1])
    y_bot = float(bottom[0].split(",", 1)[1].split("_")[-4].split("|")[1])
    length = y_top - y_bot

    # Everything at and under the tank moves down.
    children = {pid: re.findall(r"\n\tlink = (\S+)", b)
                for pid, b in parts.items()}
    below, todo = set(), [tank]
    while todo:
        p = todo.pop()
        below.add(p)
        todo.extend(children.get(p, []))

    def shift(b):
        x, y, z = (float(v) for v in field(b, "pos").split(","))
        return b.replace("\tpos = %s" % field(b, "pos"),
                         "\tpos = %r,%r,%r" % (x, y - length, z), 1)

    name, num = tank.rsplit("_", 1)
    new_id = "%s_%d" % (name, int(num) - 1)
    while new_id in parts:
        new_id = "%s_%d" % (name, int(new_id.rsplit("_", 1)[1]) - 1)

    copy = body.replace(tank, new_id, 1)
    copy = re.sub(r"\n\tlink = \S+", "", copy)
    copy = copy.replace("\n\tattN = ", "\n\tlink = %s\n\tattN = " % tank, 1)
    copy = re.sub(r"(\n\tattN = bottom,)\S+?(_0\|)", r"\g<1>%s\g<2>" % tank,
                  copy, count=1)
    copy = re.sub(r"\n\tpersistentId = \d+",
                  "\n\tpersistentId = %d" % random.randint(10**9, 4 * 10**9),
                  copy, count=1)

    moved = {}
    for pid in below:
        b = shift(parts[pid])
        if pid == tank:
            b = re.sub(r"(\n\tattN = top,)\S+?(_0\|)",
                       r"\g<1>%s\g<2>" % new_id, b, count=1)
            b = re.sub(r"\n\tattPos0 = (.*)",
                       "\n\tattPos0 = 0,%r,0" % (-length), b, count=1)
        moved[pid] = b
    pb = parts[parent]
    pb = pb.replace("\tlink = %s\n" % tank, "\tlink = %s\n" % new_id, 1)
    pb = re.sub(r"(\n\tattN = \w+,)%s(_0\|)" % re.escape(tank),
                r"\g<1>%s\g<2>" % new_id, pb, count=1)
    moved[parent] = pb

    out = text
    for s, e, pid in reversed(blocks(text)):
        if pid in moved:
            out = out[:s] + moved[pid] + out[e:]
        if pid == tank:
            out = out[:s] + copy + "\n" + out[s:]
    out = re.sub(r"(?m)^ship = .*", "ship = %s" % dst.rsplit("/", 1)[-1]
                 .rsplit(".", 1)[0], out, count=1)
    open(dst, "wb").write(out.replace("\n", newline)
                          .encode("utf-8", "surrogateescape"))
    print("%s: +%s (%.2f m) under %s; %d parts moved down"
          % (dst, new_id, length, parent, len(below)))


if __name__ == "__main__":
    main(*sys.argv[1:4])
