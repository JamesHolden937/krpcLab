#!/usr/bin/env python3
"""flaresum.py OUTFILE -- per flight of a rotfly result: the flare, tick by tick summarised."""
import re, sys, os
out = sys.argv[1]
arms = dict(re.findall(r"^(arm\d+)='([^']*)'", open(out).read(), re.M))
rows = re.findall(r"^(arm\d+) (ksp\d).*?(LOG\d+)", open(out).read(), re.M)
T = re.compile(r"^\[\s*([0-9.]+)\] (FLARE|APPROACH)\s+alt=\s*-?\d+ h=\s*(-?[0-9.]+) v=\s*([0-9.]+) M=\s*[0-9.]+ vs=\s*([+-]?[0-9.]+) aoa=\s*([0-9.]+)/\s*([0-9.]+) aoak=\s*([+-]?[0-9.]+) dal=\s*([+-]?[0-9.]+).*?bank=\s*([+-]?[0-9.]+) bnk=\s*([+-]?[0-9.]+)")
by = {}
for arm, ksp, log in rows:
    txt = open(os.path.join("/home/holden/krpcLab/logs", log), errors="replace").read()
    fl = [m for m in (T.match(l) for l in txt.splitlines()) if m and m.group(2) == "FLARE"]
    if not fl: print(arm, log, "no flare"); continue
    f = lambda m, i: float(m.group(i))
    door = fl[0]; last = fl[-1]
    arrest = next((f(m, 3) for m in fl if -f(m, 5) < 5.0), None)
    vmin = min(f(m, 4) for m in fl)
    amax = max(f(m, 8) for m in fl)
    dn = re.search(r"DOWN: .*?along ([+-]?\d+), across ([+-]?\d+).*?touchdown ([0-9.]+) m/s, (\d+) of", txt)
    ct = re.search(r"contact: before \(sink ([0-9.-]+) m/s at ([0-9.]+) m/s, h ([0-9.-]+), bank ([+-][0-9.]+), pitch ([+-]?[0-9.na]+)", txt)
    ctx = "CT s%s v%s h%s b%s p%s" % ct.groups() if ct else "CT -"
    first = re.search(r"skin sensor\(s\) have gone silent[^:]*: ([^\n]*)", txt)
    line = "%s %s door h%4.0f v%3.0f s%3.0f | arrest@%s vmin %3.0f amax %4.1f dal %+4.1f | last h%4.1f v%3.0f vs%+5.1f bnk%+5.1f | %s | %s | %s" % (
        arm, log, f(door, 3), f(door, 4), -f(door, 5), "%.0f" % arrest if arrest is not None else "-", vmin, amax, f(last, 9),
        f(last, 3), f(last, 4), f(last, 5), f(last, 11), ctx, "parts %s along %s" % (dn.group(4), dn.group(1)) if dn else "no DOWN",
        first.group(1)[:50] if first else "-")
    by.setdefault(arm, []).append(line)
for a in sorted(by):
    print("==", a, arms.get(a, ""))
    for l in by[a]: print("  ", l)
