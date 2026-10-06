#!/usr/bin/env python3
"""The angle of attack the glide actually holds while saturated, by dynamic
pressure and |bank| (GLIDE ticks above Mach 1.5 whose command exceeds the
achieved angle by 2.5 deg).  Reads logs only.

    ./spaceplane/tools/holdmap.py logs/LOG71*
"""
import re, sys, collections
cells=collections.defaultdict(list)
for f in sys.argv[1:]:
    txt=open(f,errors='replace')
    first=True
    for l in txt:
        if first:
            first=False
        if 'GLIDE    alt' not in l: continue
        m=re.search(r'aoa= *([-0-9.]+)/ *([-0-9.]+)',l); q=re.search(r'\bq= *([0-9.]+)',l); b=re.search(r'\bbank= *([-+0-9.]+)',l); M=re.search(r'\bM= *([0-9.]+)',l)
        if not (m and q and b and M): continue
        cmd,ach=float(m.group(1)),float(m.group(2)); q=float(q.group(1)); bank=abs(float(b.group(1))); mach=float(M.group(1))
        if q<800 or mach<1.5: continue
        if cmd-ach<2.5: continue  # only saturated ticks
        qb=int(q//1000)*1000; bb=int(bank//15)*15
        cells[(qb,bb)].append(ach)
print("saturated achieved alpha, median (n), by q (rows) and |bank| (cols)")
banks=sorted({k[1] for k in cells}); qs=sorted({k[0] for k in cells})
print("q\\bank "+" ".join("%9d"%b for b in banks))
for qb in qs:
    row=[]
    for bb in banks:
        v=sorted(cells.get((qb,bb),[]))
        row.append("%5.1f(%3d)"%(v[len(v)//2],len(v)) if len(v)>=5 else "    -    ")
    print("%5d "%qb+" ".join(row))
