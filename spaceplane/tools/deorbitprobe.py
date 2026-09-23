#!/usr/bin/env python3
"""What state the deorbit hands the glide, without flying the glide.

TEMPORARY TEST HARNESS -- not part of the flight software.

Measured over 15 flights from fixed entry states, the glide repeats to
sd 2.3 km while the whole flight scatters at 14.8 -- so 97.6% of the
variance, and the 24 km bias with it, is made before the vehicle reaches the
air.  The deorbit is the thing to work on, and its output is not where the
vehicle lands: it is the **predicted along-track miss at the first glide
solve**, which arrives about ninety seconds into a warped flight.

    ./spaceplane/tools/deorbitprobe.py 0 -n 6
    ./spaceplane/tools/deorbitprobe.py 0 -n 6 --set DEORBIT_WINDOW_BIAS=0.5

So this flies the deorbit and the coast, reads the first settled prediction
out of the flight's own log, and stops there.  A deorbit arm costs a fifth
of a landing, and the metric is upstream of every glide knob rather than
convolved with all of them.

**It reports the median of the first few solves, not the first one.**  The
first tick after the interface often reports an arc that never reaches the
gate at all, and the ticks around a bank reversal read tens of kilometres
long (failure 16).  Three settled samples is enough to tell a -30 km handover
from a -5 km one, which is the size of the thing being measured.
"""
import argparse
import os
import re
import signal
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LONG = re.compile(r"\sGLIDE\s.*\slong=\s*([-+0-9]+)")


def newest_log(before):
    logs = {name for name in os.listdir(os.path.join(ROOT, "logs"))
            if name.startswith("LOG")}
    fresh = logs - before
    if not fresh:
        return None
    return max(fresh, key=lambda n: int(n[3:]) if n[3:].isdigit() else -1)


def one(instance, overrides, samples, timeout):
    port = 50100 + 2 * instance
    before = {n for n in os.listdir(os.path.join(ROOT, "logs"))
              if n.startswith("LOG")}
    cmd = [os.path.join(ROOT, ".venv/bin/python"), "-m", "spaceplane.autopilot",
           "--autostart", "--rpc-port", str(port), "--stream-port", str(port + 1),
           "--set", "LOOP_PACING_GAME_TIME=True"]
    for item in overrides:
        cmd += ["--set", item]
    loader = [os.path.join(ROOT, ".venv/bin/python"), "-c",
              "import krpc;c=krpc.connect(name='load',address='127.0.0.1',"
              "rpc_port=%d,stream_port=%d);c.space_center.load('qs_plane')"
              % (port, port + 1)]
    subprocess.run(loader, cwd=ROOT, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6.0)
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    deadline = time.time() + timeout
    log = None
    seen = []
    try:
        while time.time() < deadline:
            if log is None:
                log = newest_log(before)
                time.sleep(1.0)
                continue
            path = os.path.join(ROOT, "logs", log)
            seen = [int(m) for m in LONG.findall(open(path, errors="replace").read())]
            if len(seen) >= samples:
                break
            if proc.poll() is not None:
                break
            time.sleep(1.5)
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    if not seen:
        return log, None
    return log, statistics.median(seen[:samples]) / 1000.0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", type=int)
    p.add_argument("-n", type=int, default=3)
    p.add_argument("--samples", type=int, default=3,
                   help="glide predictions to median over")
    p.add_argument("--timeout", type=float, default=900.0)
    p.add_argument("--set", action="append", default=[])
    args = p.parse_args(argv)

    got = []
    for i in range(1, args.n + 1):
        log, miss = one(args.instance, args.set, args.samples, args.timeout)
        print("%2d  %-9s handover %s" % (i, log or "?",
              "%+8.1f km" % miss if miss is not None else "no prediction"),
              flush=True)
        if miss is not None:
            got.append(miss)
    if len(got) > 1:
        sd = statistics.stdev(got)
        print("    mean %+.1f km  sd %.1f  se %.1f  over %d"
              % (statistics.mean(got), sd, sd / len(got) ** 0.5, len(got)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
