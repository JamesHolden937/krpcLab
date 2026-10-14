#!/usr/bin/env python3
"""Fly the low-thrust pilot on a farm instance from a save, and report.

    ./lowThrust/tools/ltfly.py --instance 0 --save qs_lowthrust --target "kerbin: ap=150 pe=150"
    ./lowThrust/tools/ltfly.py --instance 0 --target "mun: ap=50 pe=50" --timeout 7200 --set ALIGN_DEG=1

Loads the save, runs ``lowThrust.pilot`` as a child (its own process group,
reaped on any exit: an orphaned pilot keeps flying the vessel), waits for it
to end or the timeout, and prints the log's name and its last lines.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from common import paths                                # noqa: E402

paths.use_venv()
import krpc                                             # noqa: E402

_LIVE = []


def _reap(p):
    if p.poll() is not None:
        return
    for sig, wait in ((signal.SIGTERM, 20.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(p.pid), sig)
        except Exception:                               # noqa: BLE001
            pass
        try:
            p.wait(timeout=wait)
            return
        except Exception:                               # noqa: BLE001
            continue


def _handler(signum, frame):                            # noqa: ARG001
    for p in list(_LIVE):
        _reap(p)
    raise SystemExit(128 + signum)


def logs():
    return {int(m.group(1)) for n in os.listdir(paths.LOGS)
            for m in [re.fullmatch(r"LOG(\d+)", n)] if m}


def pilot_log(pid):
    """The LOG<n> the process ``pid`` has open, or None."""
    try:
        for fd in os.listdir("/proc/%d/fd" % pid):
            target = os.readlink("/proc/%d/fd/%s" % (pid, fd))
            m = re.fullmatch(r"LOG(\d+)", os.path.basename(target))
            if m and os.path.dirname(target) == os.path.realpath(paths.LOGS):
                return m.group(0)
    except OSError:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", default="0")
    ap.add_argument("--save", default="qs_lowthrust")
    ap.add_argument("--target", required=True)
    ap.add_argument("--timeout", type=float, default=3600.0, help="wall s")
    ap.add_argument("--set", action="append", default=[])
    args = ap.parse_args()
    for s in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(s, _handler)

    rpc, stream = paths.ports(args.instance)
    conn = krpc.connect("ltfly-loader", rpc_port=rpc, stream_port=stream)
    conn.space_center.load(args.save)
    time.sleep(2.0)
    t0 = conn.space_center.ut
    time.sleep(1.0)
    if conn.space_center.ut <= t0:
        raise SystemExit("ksp%s: the save loaded but game time is not "
                         "advancing" % args.instance)
    conn.close()

    before = logs()
    cmd = [paths.VENV_PYTHON, "-m", "lowThrust.pilot", "--rpc-port", str(rpc),
           "--stream-port", str(stream), "--autostart", "--target",
           args.target]
    for s in args.set:
        cmd += ["--set", s]
    started = time.time()
    p = subprocess.Popen(cmd, cwd=ROOT, start_new_session=True,
                         stdout=subprocess.DEVNULL,
                         stderr=open(os.path.join(paths.LOGS, "stderr.log"),
                                     "a"))
    _LIVE.append(p)
    name = None
    try:
        # the log is the one the pilot has open: two flights started together
        # on two instances both see each other's as new
        while name is None and p.poll() is None \
                and time.time() - started < 120:
            name = pilot_log(p.pid)
            time.sleep(0.5)
        p.wait(timeout=max(1.0, args.timeout - (time.time() - started)))
        how = "exit %d" % p.returncode
    except subprocess.TimeoutExpired:
        how = "TIMEOUT after %.0f s" % args.timeout
    finally:
        _reap(p)
    if name is None:
        new = sorted(logs() - before)
        name = "LOG%d" % new[0] if len(new) == 1 else None
    name = name or "(no log)"
    print("ksp%s %s %s: %s, %.0f s wall" % (args.instance, args.save,
                                           args.target, how,
                                           time.time() - started))
    if name != "(no log)":
        lines = open(os.path.join(paths.LOGS, name)).read().splitlines()
        print(name)
        for line in [x for x in lines if "ARRIVED" in x or "FAILED" in x
                     or "FATAL" in x or "shutdown" in x][-6:]:
            print("  " + line[:300])


if __name__ == "__main__":
    main()
