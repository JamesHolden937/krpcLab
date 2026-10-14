#!/usr/bin/env python3
"""Fly the lowThrust mod on a farm instance from a save, and report.

    ./lowThrust/tools/ltfly.py --instance 0 --save qs_lowthrust --target "kerbin: ap=150 pe=150"
    ./lowThrust/tools/ltfly.py --instance 0 --target "mun: ap=50 pe=50" --timeout 7200 --set ALIGN_DEG=1

Loads the save, starts a flight through the mod's farm bridge (the kRPC
service LowThrustBridge, LowThrustKRPC.dll, installed on the farm by
lowThrust/mod/build.sh), waits for it to end or the timeout (then
terminates it), copies the mod's flight log to logs/LOG<n> and prints its
name and last lines.  The flight itself runs inside the game; nothing in it
goes over kRPC.
"""
import argparse
import ntpath
import os
import shutil
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import paths                                # noqa: E402
from common.logbook import _open_next_log               # noqa: E402

paths.use_venv()
import krpc                                             # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", default="0")
    ap.add_argument("--save", default="qs_lowthrust")
    ap.add_argument("--target", required=True)
    ap.add_argument("--timeout", type=float, default=3600.0, help="wall s")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--max-dv", type=float, help="the dv budget, m/s")
    ap.add_argument("--max-days", type=float, help="the time budget, days")
    args = ap.parse_args()
    if args.max_dv:
        args.set.append("MAX_DV=%g" % args.max_dv)
    if args.max_days:
        args.set.append("MAX_DAYS=%g" % args.max_days)

    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect("ltfly", rpc_port=rpc, stream_port=stream)
    conn.space_center.load(args.save)
    time.sleep(2.0)
    t0 = conn.space_center.ut
    time.sleep(1.0)
    if conn.space_center.ut <= t0:
        raise SystemExit("ksp%s: the save loaded but game time is not "
                         "advancing" % args.instance)
    br = getattr(conn, "low_thrust_bridge", None)
    if br is None:
        raise SystemExit("ksp%s has no LowThrustBridge service: run "
                         "lowThrust/mod/build.sh and restart it" % args.instance)
    before = int(br.status()[3])
    started = time.time()
    br.start(args.target, args.set)
    how = None
    while how is None:
        time.sleep(2.0)
        st = br.status()
        if int(st[3]) > before:
            how = "ended"
        elif time.time() - started > args.timeout:
            br.terminate()
            time.sleep(2.0)
            st = br.status()
            how = "TIMEOUT after %.0f s" % args.timeout
    wall = time.time() - started
    src = os.path.join(paths.instance_dir(args.instance), "GameData",
                       "LowThrust", "PluginData", "logs",
                       ntpath.basename(st[2].replace("/", "\\")))
    name = "(no log)"
    if os.path.exists(src):
        dest, f = _open_next_log(paths.LOGS)
        f.close()
        shutil.copyfile(src, dest)
        name = os.path.basename(dest)
    print("ksp%s %s %s: %s, %.0f s wall: %s" % (args.instance, args.save,
                                               args.target, how, wall,
                                               st[1]))
    print(name)
    if name != "(no log)":
        lines = open(os.path.join(paths.LOGS, name)).read().splitlines()
        for line in [x for x in lines if "ARRIVED" in x or "FAILED" in x
                     or "FATAL" in x or "shutdown" in x
                     or "per phase" in x or "per physics" in x
                     or "thrust commands" in x or "plan:" in x
                     or "budget" in x][-10:]:
            print("  " + line[:400])
    conn.close()


if __name__ == "__main__":
    main()
