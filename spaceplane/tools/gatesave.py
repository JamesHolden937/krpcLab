#!/usr/bin/env python3
"""Quicksave a farm flight the moment its cone hands over on profile.

TEMPORARY TEST HARNESS -- not part of the flight software.

The landing chain (approach, flare, rollout) cannot be measured from orbit:
the cone's exit scatters by kilometres and every landing inherits it.  A save
taken at ``HAC -> APPROACH`` with the height the approach needed is the
repeatable arrival that work wants, and it costs a minute of game time.

    ./spaceplane/tools/gatesave.py --name qs_shuttle2_gate --tol 300

Watches every log newer than the newest one at start, finds which farm
instance each belongs to (its ``timescale.txt`` path), and on the first
``HAC -> APPROACH rolled out ... h=H (needed N)`` with ``|H - N| <= tol``
saves that instance's game under ``--name``.  It commands nothing.
Then ``testInstances/syncSaves.sh pull N NAME``.
"""
import argparse
import glob
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402

paths.use_venv()
import krpc  # noqa: E402

HANDOVER = re.compile(r"HAC -> APPROACH rolled out .*? h=(\d+) \(needed (\d+)\)")
INSTANCE = re.compile(r"testInstances/ksp(\d+)/timescale")


def lognum(path):
    return int(re.sub(r"\D", "", os.path.basename(path)) or 0)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--name", default="qs_shuttle2_gate")
    p.add_argument("--tol", type=float, default=300.0)
    p.add_argument("--timeout", type=float, default=7200.0)
    p.add_argument("--after", type=int, default=None,
                   help="watch logs numbered above this (default: the newest "
                        "at start -- a flight already flying has its log)")
    args = p.parse_args(argv)
    logs = os.path.join(ROOT, "logs")
    start = args.after if args.after is not None else max(
        (lognum(f) for f in glob.glob(os.path.join(logs, "LOG*"))), default=0)
    deadline = time.time() + args.timeout
    offsets = {}
    while time.time() < deadline:
        for path in glob.glob(os.path.join(logs, "LOG*")):
            if lognum(path) <= start:
                continue
            try:
                with open(path, errors="ignore") as fh:
                    fh.seek(offsets.get(path, 0))
                    text = fh.read()
                    offsets[path] = fh.tell()
            except OSError:
                continue
            m = HANDOVER.search(text)
            if not m:
                continue
            h, need = float(m.group(1)), float(m.group(2))
            with open(path, errors="ignore") as fh:
                inst = INSTANCE.search(fh.read())
            print("%s: handover h=%.0f needed %.0f (%+.0f)%s"
                  % (os.path.basename(path), h, need, h - need,
                     "" if inst else " -- no instance"), flush=True)
            if inst is None or abs(h - need) > args.tol:
                continue
            port = 50100 + 2 * int(inst.group(1))
            conn = krpc.connect(name="gatesave", address="127.0.0.1",
                                rpc_port=port, stream_port=port + 1)
            conn.space_center.save(args.name)
            print("saved %r on ksp%s from %s" % (args.name, inst.group(1),
                                                 os.path.basename(path)))
            return 0
        time.sleep(0.3)
    print("timed out", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
