#!/usr/bin/env python3
"""lateralsum.py LOG... -- subsonic lateral health and the landing, one line per flight.
bad60: subsonic ticks with flown bank >60 deg off command (includes reversal lag).
slip10: subsonic ticks with |sideslip| >10 deg (the departure signature).
bnkF / bnkR: flown bank at the last FLARE tick and the first ROLLOUT tick;
cbank: bank at the `contact:` line's "at" (first main-wheel contact).
"""
import re, sys
for p in sys.argv[1:]:
    n=bad=s10=0; smax=0; td=''; down=''; hac=''; bF=bR=cb=float('nan')
    for l in open(p, errors='replace'):
        m=re.search(r' M=\s*([\d.]+).*?slip=\s*([-+\d.]+) bank=\s*([-+\d.]+) bnk=\s*([-+\d.]+)', l)
        if m and float(m.group(1))<1.0 and ' alt=' in l:
            d=abs((float(m.group(4))-float(m.group(3))+180)%360-180); s=abs(float(m.group(2)))
            n+=1; bad+=d>60; s10+=s>10; smax=max(smax,s)
        b=re.search(r'^\[[ \d.]+\] (FLARE|ROLLOUT)\s.*bnk=\s*([-+\d.]+)', l)
        if b and b.group(1)=='FLARE': bF=float(b.group(2))
        if b and b.group(1)=='ROLLOUT' and bR!=bR: bR=float(b.group(2))
        c=re.search(r'contact: before .*?, at \(.*?bank ([-+\d.]+)', l)
        if c and cb!=cb: cb=float(c.group(1))
        if 'FLARE -> ROLLOUT' in l: td=re.sub(r'.*ROLLOUT ','',l.strip())
        if 'HAC -> APPROACH' in l: h=re.search(r'h=(\d+) \(needed (\d+)\)',l); hac='exit %+d'%(int(h.group(1))-int(h.group(2))) if h else ''
        if 'DOWN:' in l: d=re.search(r'along ([-+\d]+), across ([-+\d]+)\).*?(\d+ of \d+ parts)',l); down='along %s across %s %s'%d.groups() if d else l[-60:]
    print("%-8s slip10 %3d  slipmax %5.1f  %-11s bnkF %+6.1f bnkR %+6.1f cbank %+6.1f  td %-24s %s" % (p.split('/')[-1], s10, smax, hac, bF, bR, cb, td, down))
