#!/usr/bin/env python3
import re, sys, glob, os
pat = re.compile(r"aoa=\s*([-0-9.]+)/\s*([-0-9.]+).*?pin=([-+0-9.]+)/([-+0-9.]+)/([-+0-9.]+)")
res = []
for rot in sys.argv[1:]:
    arms = {}
    for line in open(rot):
        m = re.match(r"(arm\d+)='([^|]*)\|(.*)'", line)
        if m: arms[m.group(1)] = (m.group(2), m.group(3)); continue
        m = re.match(r"(arm\d+) ksp\d+\s+\d+\s+(\S+).*along\s+([-+0-9]+)\s+across\s+([-+0-9]+).*?(\d+) parts.*?(LOG\d+)", line)
        if not m: continue
        arm, save, along, across, parts, log = m.groups()
        f = "logs/" + log
        if not os.path.exists(f): continue
        n = stuck = 0
        for l in open(f, errors="replace"):
            if "] HAC " not in l and "] APPROACH " not in l: continue
            mm = pat.search(l)
            if not mm: continue
            cmd, act, pin, _, err = map(float, mm.groups())
            n += 1
            if abs(pin) < 0.98 and err < -3 and pin > 0.1: stuck += 1
        if n == 0: continue
        intact = int(parts) >= 18
        res.append((os.path.basename(rot), arm, save, int(along), int(across), intact, 100.0*stuck/n, log, arms.get(arm, ("", ""))[1]))
# bucket
def bucket(r):
    if not r[5]: return "lost"
    if abs(r[3]) <= 1200 and abs(r[4]) <= 35: return "runway"
    return "short" if r[3] < -1200 else "long"
import collections
b = collections.defaultdict(list)
for r in res:
    b[bucket(r)].append(r[6])
for k, v in b.items():
    v.sort()
    print("%-7s n=%3d stuck%% median %5.1f  [%s]" % (k, len(v), v[len(v)//2], " ".join("%.0f" % x for x in v)))
# threshold table
for th in (5, 10, 20):
    lo = [r for r in res if r[6] < th]; hi = [r for r in res if r[6] >= th]
    def rate(rs): 
        return "%d/%d runway, %d short" % (sum(bucket(r)=="runway" for r in rs), len(rs), sum(bucket(r)=="short" for r in rs))
    print("stuck <%d%%: %s   >=%d%%: %s" % (th, rate(lo), th, rate(hi)))
