#!/usr/bin/env python3
"""flightsum.py LOG... -- one line per flight: arm, slip, handover, touchdown, first loss."""
import re, sys
TEL = re.compile(r"\]\s+(GLIDE|HAC|APPROACH|FLARE|ROLLOUT)\s+alt=\s*(-?\d+).*?\bM=\s*([0-9.]+).*?slip=\s*([+-]?[0-9.]+).*?\bn=\s*(\d+)")
for path in sys.argv[1:]:
    txt = open(path, errors="replace").read()
    cfg = re.search(r"config: (.*)", txt)
    cfg = cfg.group(1) if cfg else ""
    save = re.search(r"SAVE_NAME=(\w+)", cfg); save = save.group(1) if save else "?"
    fp = re.search(r"\[defaults (\w+)\]", cfg); fp = fp.group(1) if fp else "?"
    sets = [s for s in cfg.split(", ") if not s.startswith(("SAVE_NAME", "LOOP_PACING", "TIMESCALE_GOVERNOR"))]
    sets = ";".join(s.split(" [defaults")[0] for s in sets)
    gh = gl = hac = 0.0; m15 = None; last = "?"; n0 = None
    for m in TEL.finditer(txt):
        ph, M, slip = m.group(1), float(m.group(3)), abs(float(m.group(4)))
        last = ph
        if ph == "GLIDE":
            if M >= 2: gh = max(gh, slip)
            else: gl = max(gl, slip)
        elif ph == "HAC":
            hac = max(hac, slip)
        if ph in ("GLIDE", "HAC") and slip > 15 and m15 is None: m15 = M
    ha = re.search(r"HAC -> APPROACH.*?h=(\d+) \(needed (\d+)\)", txt)
    fl = re.search(r"APPROACH -> FLARE h=([0-9.]+) v=([0-9.]+) sink=([0-9.-]+)", txt)
    td = re.search(r"FLARE -> ROLLOUT sink=([0-9.-]+) speed=([0-9.]+)", txt)
    down = re.search(r"DOWN: (.*)", txt)
    dn = ""
    if down:
        d = down.group(1)
        a = re.search(r"along ([+-]?\d+), across ([+-]?\d+)", d)
        p = re.search(r"(\d+) of (\d+) parts", d)
        dn = "along %s across %s parts %s" % (a.group(1) if a else "?", a.group(2) if a else "?", p.group(1) if p else "?")
    lastfl = None
    for line in txt.splitlines():
        if "] FLARE " in line and " alt=" in line:
            lastfl = line
    tdk = "-"
    if lastfl:
        mm = re.search(r"h=\s*([0-9.-]+) v=\s*([0-9.]+).*?vs=\s*([+-]?[0-9.]+).*?bnk=\s*([+-]?[0-9.]+)", lastfl)
        if mm: tdk = "h%s v%s vs%s bnk%s" % mm.groups()
    first = re.search(r"skin sensor\(s\) have gone silent[^:]*: ([^\n]*)", txt)
    first = first.group(1)[:60] if first else "-"
    print("%s %-18s %s %-28s slipG>2 %4.0f <2 %4.0f HAC %4.0f m15 %4s | hac h %s/%s | flare %s | td %s [%s] | %s | last %s | %s" % (
        path.split("/")[-1], save, fp, sets[:28], gh, gl, hac,
        "-" if m15 is None else "%.1f" % m15,
        ha.group(1) if ha else "-", ha.group(2) if ha else "-",
        "%s@%s/%s" % fl.groups() if fl else "-",
        "%s/%s" % td.groups() if td else "-", tdk, dn or "no DOWN", last, first))
