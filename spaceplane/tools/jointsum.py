#!/usr/bin/env python3
"""Per touchdown, the wing-root load the game measured (CollisionSpy).

The CollisionSpy plugin (testInstances/collisionSpySrc) logs, every physics
step while the vessel is low, the largest attach-joint force and torque on
the ``wingShuttleDelta`` parts, and every joint break.  This splits an
instance's KSP.log into flights (one per save load), finds the touchdown --
the first step the wing-root force leaves its in-flight level -- and reports
the peak force and torque over the following ``--window`` seconds up to the
break, and whether the wing joints broke.

    ./spaceplane/tools/jointsum.py testInstances/ksp0/KSP.log [...]
    ./spaceplane/tools/jointsum.py --batch logs/sav-susp-1005.txt

``--batch`` reads every instance a savefly batch used and labels each
touchdown with the arm and LOG the batch file gives it, in flight order.
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
JOINT = re.compile(r"ut=([\d.]+) JOINT agl=([-\d.]+) .*wingF=(\d+)/(\d+) "
                   r"wingT=(\d+)/(\d+) wingJoints=(\d+) n=(\d+)")
BREAK = re.compile(r"ut=([\d.]+) JOINTBREAK part=wingShuttleDelta .*force=(\d+)")


def flights(path):
    """Yield the CollisionSpy lines of each flight (split at FLIGHT loads)."""
    cur = []
    for line in open(path, errors="replace"):
        if "Scene Change" in line and "to FLIGHT" in line:
            if cur:
                yield cur
            cur = []
        elif "[CollisionSpy]" in line:
            cur.append(line)
    if cur:
        yield cur


def touchdown(lines, window):
    rows = []
    for line in lines:
        m = JOINT.search(line)
        if m:
            rows.append(tuple(float(x) for x in m.groups()))
            continue
        b = BREAK.search(line)
        if b:
            rows.append(("break", float(b.group(1)), float(b.group(2))))
    base = None
    for i, r in enumerate(rows):
        if r[0] == "break":
            continue
        ut, agl, f, fb, t, tb, nj, n = r
        if nj == 0:
            continue
        if base is None:
            base = f
        if f > max(40.0, 2.0 * base):
            peakf, peakt, broke, bforce = f, t, False, None
            for r2 in rows[i:]:
                if r2[0] == "break":
                    if r2[1] - ut <= window:
                        broke, bforce = True, max(bforce or 0, r2[2])
                    continue
                if r2[0] - ut > window or r2[6] == 0:
                    break
                peakf, peakt = max(peakf, r2[2]), max(peakt, r2[4])
            return dict(ut=ut, peakf=peakf, peakt=peakt, fb=fb, tb=tb,
                        broke=broke, bforce=bforce)
        base = 0.8 * base + 0.2 * f
    return None


def batch_labels(path):
    """{instance: [(arm, LOG), ...]} in flight order, from a savefly file."""
    out = {}
    for line in open(path):
        m = re.match(r"(arm\d+) ksp(\d+) .*LOG(\d+)", line)
        if m:
            out.setdefault(m.group(2), []).append((m.group(1), m.group(3)))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("logs", nargs="*")
    p.add_argument("--batch")
    p.add_argument("--window", type=float, default=0.3)
    a = p.parse_args()
    jobs = []
    if a.batch:
        # savefly appends a line when a flight *ends*, so per instance the
        # file order is flight order.
        for inst, labels in sorted(batch_labels(a.batch).items()):
            jobs.append((os.path.join(ROOT, "testInstances", "ksp" + inst,
                                      "KSP.log"), labels))
    jobs += [(path, None) for path in a.logs]
    print("%-6s %-6s %-8s %10s %8s %8s  %s" % ("inst", "arm", "log", "ut",
                                               "peakF", "peakT", "result"))
    for path, labels in jobs:
        inst = re.sub(r".*ksp(\d+).*", r"\1", path)
        found = [td for td in (touchdown(f, a.window) for f in flights(path))
                 if td]
        if labels and len(found) != len(labels):
            print("ksp%s: %d touchdowns in KSP.log, %d flights in the batch "
                  "file -- labels may be off" % (inst, len(found), len(labels)))
        for k, td in enumerate(found):
            arm, log = (labels[k] if labels and k < len(labels) else ("?", "?"))
            print("ksp%-3s %-6s LOG%-5s %10.3f %8.0f %8.0f  %s" % (
                inst, arm, log, td["ut"], td["peakf"], td["peakt"],
                "BROKE (force %.0f)" % td["bforce"] if td["broke"]
                else "held"))


if __name__ == "__main__":
    main()
