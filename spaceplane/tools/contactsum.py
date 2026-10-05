#!/usr/bin/env python3
"""One line per touchdown: the contact state and what the airframe kept.

Reads the ``contact:`` line and, with ``GROUND_WATCH_S`` on, the ``ground``
lines: the part count 0.1 / 0.5 / 3 s after contact and the rigid
clearance of the lowest non-wheel part at the moment the count first fell
(positive = nothing but the wheels was touching -- the parts left at the
joints, not on the runway).

    ./spaceplane/tools/contactsum.py logs/LOG57[5-9]* [--by FIELD]
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

CONTACT = re.compile(r"at \(sink ([-\d.]+) m/s at ([\d.]+) m/s, h ([-\d.]+), "
                     r"bank ([-+\d.]+), pitch ([-+\d.na]+)")
GROUND = re.compile(r"\] ground t([-+\d.]+) n=(\d+) mains (\S+) \| low \S+.*?"
                    r"([-+]\d+\.\d+)")


def summarise(path, by):
    save, arm, contact, first = "?", [], None, None
    counts, n0 = {}, None
    with open(path, errors="replace") as fh:
        for line in fh:
            if "config:" in line and save == "?":
                m = re.search(r"SAVE_NAME=(\w+)", line)
                save = m.group(1) if m else "?"
                for field in by:
                    m = re.search(r"%s=([^,\[]+)" % field, line)
                    arm.append("%s=%s" % (field, m.group(1).strip()
                                          if m else "dflt"))
            elif "vessel:" in line and n0 is None:
                m = re.search(r"(\d+) parts", line)
                n0 = int(m.group(1)) if m else None
            elif "] contact:" in line and contact is None:
                m = CONTACT.search(line)
                if m:
                    contact = tuple(float(x) for x in m.groups())
            elif "] ground t" in line:
                m = GROUND.search(line)
                if not m:
                    continue
                t, n, low = float(m.group(1)), int(m.group(2)), float(
                    m.group(4))
                for mark in (0.1, 0.5, 3.0):
                    if t <= mark + 0.05:
                        counts[mark] = n
                if n0 is not None and n < n0 and first is None:
                    first = (t, low)
    return save, " ".join(arm), contact, counts, first, n0


def main():
    args = sys.argv[1:]
    by = []
    while "--by" in args:
        i = args.index("--by")
        by.append(args[i + 1])
        del args[i:i + 2]
    print("%-8s %-11s %-28s %5s %5s %5s %6s  %s" % (
        "log", "save", "arm", "sink", "v", "pitch", "n.1/.5/3", "first loss"))
    for path in args:
        save, arm, c, counts, first, n0 = summarise(path, by)
        if c is None:
            print("%-8s %-11s %-28s  no contact" % (
                os.path.basename(path), save[-6:], arm))
            continue
        print("%-8s %-11s %-28s %5.1f %5.1f %+5.1f %3s/%s/%s  %s" % (
            os.path.basename(path), save[-6:], arm, c[0], c[1], c[4],
            counts.get(0.1, "?"), counts.get(0.5, "?"), counts.get(3.0, "?"),
            "-" if first is None else "t+%.2f low %+.2f m" % first))


if __name__ == "__main__":
    main()
