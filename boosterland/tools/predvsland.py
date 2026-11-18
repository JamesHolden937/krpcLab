#!/usr/bin/env python3
"""TEMPORARY DIAGNOSTIC (not flight software). Predicted landing at ignition (last COAST line) against the flown one.

Approach is westbound, so long ~ -E, cross ~ +N (checked on LOG17543: pred
long -30 cross -38 -> landed N -40 E +29).  Usage: predvsland.py ROTFILE
"""
import re
import statistics as st
import sys

rows = []
for line in open(sys.argv[1]):
    m = re.search(r"N +([-+]?\d+) E +([-+]?\d+) +LOG(\d+)", line)
    if not m:
        continue
    n_land, e_land, log = int(m.group(1)), int(m.group(2)), m.group(3)
    coast = [l for l in open("logs/LOG" + log) if re.search(r"\] COAST +alt=", l)]
    p = re.search(r"long= *([-+]?\d+) cross= *([-+]?\d+)", coast[-1])
    lo, cr = int(p.group(1)), int(p.group(2))
    n_pred, e_pred = cr, -lo
    dn, de = n_land - n_pred, e_land - e_pred
    rows.append((log, n_pred, e_pred, n_land, e_land, dn, de))
print("log      pred N/E      landed N/E    landed-pred N/E   |err|")
for r in sorted(rows):
    print("LOG%s  %+5d %+5d   %+5d %+5d    %+5d %+5d   %5.0f"
          % (r + ((r[5] ** 2 + r[6] ** 2) ** 0.5,)))
for name, i in (("pred |miss|", None), ("err N", 5), ("err E", 6)):
    if i is None:
        v = [(r[1] ** 2 + r[2] ** 2) ** 0.5 for r in rows]
    else:
        v = [r[i] for r in rows]
    print("%-12s median %+6.0f sd %5.0f" % (name, st.median(v), st.stdev(v)))
land = [(r[3] ** 2 + r[4] ** 2) ** 0.5 for r in rows]
print("landed |miss| median %.0f, sorted %s" % (st.median(land),
                                               sorted(int(x) for x in land)))
