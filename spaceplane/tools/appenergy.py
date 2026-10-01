#!/usr/bin/env python3
"""appenergy.py LOG... -- the approach's energy budget, gate to door.
effective L/D = horizontal path / (drop in energy height h + v^2/2g)."""
import re, sys, math
G = 9.81
T = re.compile(r"^\[\s*([0-9.]+)\] (APPROACH|FLARE)\s+alt=\s*-?\d+ h=\s*(-?[0-9.]+) v=\s*([0-9.]+) M=\s*[0-9.]+ vs=\s*([+-]?[0-9.]+)")
for p in sys.argv[1:]:
    txt = open(p, errors="replace").read()
    ex = re.search(r"HAC -> APPROACH.*?h=(\d+) \(needed (\d+)\)", txt)
    if not ex: continue
    cfg = re.search(r"config: (.*)", txt).group(1)
    sets = ";".join(s for s in cfg.split(", ") if not s.startswith(("SAVE_NAME", "LOOP", "TIMESC")))[:40]
    rows = [(float(a), b, float(c), float(d), float(e)) for a, b, c, d, e in T.findall(txt, re.M)] if False else []
    for line in txt.splitlines():
        m = T.match(line)
        if m: rows.append((float(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4)), float(m.group(5))))
    app = [r for r in rows if r[1] == "APPROACH"]
    if len(app) < 3: continue
    path = 0.0
    for a, b in zip(app, app[1:]):
        vh = math.sqrt(max(0.0, a[3] ** 2 - a[4] ** 2)); path += vh * (b[0] - a[0])
    e0 = app[0][2] + app[0][3] ** 2 / (2 * G); e1 = app[-1][2] + app[-1][3] ** 2 / (2 * G)
    ld = path / max(1.0, e0 - e1)
    fl = [r for r in rows if r[1] == "FLARE"]
    print("%s exit h %5.0f need %5.0f v0 %5.1f | door h %5.0f v %5.1f sink %5.1f | path %5.0f dE %5.0f ld %4.2f dur %4.0f | %s" % (
        p.split("/")[-1], app[0][2], float(ex.group(2)), app[0][3], app[-1][2], app[-1][3], -app[-1][4], path, e0 - e1, ld, app[-1][0] - app[0][0], sets))
