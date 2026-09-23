#!/usr/bin/env python3
"""Fly a grid of (save, config) jobs across the testInstances in parallel.

TEMPORARY TEST HARNESS -- not part of the flight software.

`quickfly.py` flies one instance at a time; this hands it a work queue and a
pool of instances.  A flight is about three minutes, and the question that
matters -- does a change help on *every* entry state, or only the one it was
tuned on -- is a grid, so serially it is an hour a question.

    ./boosterland/tools/sweep.py -n 2 --saves quicksave,qs_hot,qs_cold
    ./boosterland/tools/sweep.py -n 2 --compare CORRECTION_ENTER_M=300,1200
    ./boosterland/tools/sweep.py -n 3 --saves quicksave --set AIM_BIAS_EAST_M=0

Every job in one sweep runs under the same instance count, because that is the
comparison the harness supports: a loaded machine slows the game below real
time and the control loop sleeps on wall time, so the number of instances is
part of the configuration (see testInstances/HANDOFF.md).
"""

import argparse
import json
import os
import queue
import re
import statistics
import subprocess
import sys
import threading
import time

INSTANCES = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "testInstances")
LINE = re.compile(r"^\s+\d+/\d+\s+(\S+)\s+(-?[\d.]+) m\s+([\d.]+) m/s\s+c(-?\d+)"
                  r"\s+N\s*([-+][\d.]+)\s+E\s*([-+][\d.]+)\s+(\S+)\s*$")


def live_instances():
    out = []
    for name in sorted(os.listdir(INSTANCES)):
        if not re.fullmatch(r"ksp\d+", name):
            continue
        port_file = os.path.join(INSTANCES, name, ".rpc_port")
        if not os.path.exists(port_file):
            continue
        port = open(port_file).read().strip()
        listening = subprocess.run(["ss", "-ltn"], capture_output=True,
                                   text=True).stdout
        if (":" + port) in listening:
            out.append(name[3:])
    return out


def run_job(inst, job, timeout):
    cmd = [os.path.join(INSTANCES, "fly.sh"), inst,
           "--save", job["save"], "-n", str(job["runs"])]
    for pair in job["overrides"]:
        cmd += ["--set", pair]
    started = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    flights = []
    for line in proc.stdout.splitlines():
        m = LINE.match(line)
        if m:
            flights.append({"situation": m.group(1), "distance": float(m.group(2)),
                            "speed": float(m.group(3)), "corrections": int(m.group(4)),
                            "north": float(m.group(5)), "east": float(m.group(6)),
                            "log": m.group(7)})
    return {"job": job, "instance": inst, "flights": flights,
            "seconds": time.time() - started, "rc": proc.returncode,
            "stderr": proc.stderr[-2000:] if proc.returncode else ""}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-n", "--runs", type=int, default=2,
                   help="flights per (save, config) cell")
    p.add_argument("--saves", default="quicksave",
                   help="comma-separated save names")
    p.add_argument("--set", action="append", default=[], metavar="K=V",
                   help="override applied to every job")
    p.add_argument("--compare", metavar="FIELD=A,B",
                   help="one config column per value of FIELD")
    p.add_argument("--configs", metavar="LABEL:K=V,K=V;LABEL:...",
                   help="arbitrary config columns, semicolon separated -- for "
                        "comparing combinations rather than one field's values")
    p.add_argument("--label", default="sweep")
    p.add_argument("--timeout", type=float, default=0.0,
                   help="wall-clock seconds to allow one cell; 0 derives it "
                        "from --runs.  A cell that outlives this is abandoned "
                        "rather than left to hang the sweep -- a wedged game "
                        "blocks kRPC forever and quickfly's own deadline is "
                        "never reached, so four cells once sat for 25 minutes.")
    p.add_argument("--out", help="write the raw results here as JSON")
    args = p.parse_args(sys.argv[1:] if argv is None else argv)

    if not args.timeout:
        args.timeout = args.runs * 600.0 + 180.0
    insts = live_instances()
    if not insts:
        raise SystemExit("no instance is listening -- start one with "
                         "testInstances/kwinRun.sh N")
    print("instances: %s" % ", ".join("ksp" + i for i in insts), flush=True)

    configs = [("base", list(args.set))]
    if args.configs:
        configs = []
        for column in args.configs.split(";"):
            label, _, pairs = column.partition(":")
            configs.append((label, args.set
                            + [p for p in pairs.split(",") if p]))
    if args.compare:
        field, _, values = args.compare.partition("=")
        configs = [("%s=%s" % (field, v), args.set + ["%s=%s" % (field, v)])
                   for v in values.split(",")]

    jobs = [{"save": s, "config": name, "overrides": ov, "runs": args.runs}
            for s in args.saves.split(",") for name, ov in configs]

    work = queue.Queue()
    for j in jobs:
        work.put(j)
    results, lock = [], threading.Lock()

    def worker(inst):
        while True:
            try:
                job = work.get_nowait()
            except queue.Empty:
                return
            try:
                res = run_job(inst, job, args.timeout)
            except Exception as exc:            # noqa: BLE001
                res = {"job": job, "instance": inst, "flights": [],
                       "rc": -1, "seconds": 0.0, "stderr": repr(exc)}
            with lock:
                results.append(res)
                print("  ksp%s  %-14s %-28s %s (%.0fs)"
                      % (inst, job["save"], job["config"],
                         " ".join("%.0f" % f["distance"] for f in res["flights"])
                         or "FAILED rc=%s %s" % (res["rc"], res["stderr"][-200:]),
                         res["seconds"]), flush=True)

    threads = [threading.Thread(target=worker, args=(i,)) for i in insts]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("\n== %s ==" % args.label)
    print("%-14s %-28s %8s %8s %8s   %s"
          % ("save", "config", "median", "mean", "worst", "flights"))
    cells = {}
    for res in results:
        cells[(res["job"]["save"], res["job"]["config"])] = res["flights"]
    for save in args.saves.split(","):
        for name, _ in configs:
            fl = cells.get((save, name), [])
            d = [f["distance"] for f in fl]
            if not d:
                print("%-14s %-28s %s" % (save, name, "  (no flights)"))
                continue
            print("%-14s %-28s %8.0f %8.0f %8.0f   %s"
                  % (save, name, statistics.median(d), statistics.mean(d),
                     max(d),
                     "  ".join("%.0f(N%+.0f E%+.0f %.1fm/s)"
                               % (f["distance"], f["north"], f["east"],
                                  f["speed"]) for f in fl)))
    for name, _ in configs:
        d = [f["distance"] for res in results
             if res["job"]["config"] == name for f in res["flights"]]
        if d:
            print("  ALL SAVES %-24s median %6.0f  mean %6.0f  worst %6.0f  (n=%d)"
                  % (name, statistics.median(d), statistics.mean(d), max(d), len(d)))
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(results, fh, indent=1)
        print("raw -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
