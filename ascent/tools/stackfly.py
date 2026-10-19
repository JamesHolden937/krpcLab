#!/usr/bin/env python3
"""Fly the ascent from a pad save on a farm instance, unattended.

    ./ascent/tools/stackfly.py --instance 0 -n 1
    ./ascent/tools/stackfly.py --instance 2 --save qs_stack_half --set RESERVE_FRACTION=0.1

Loads ``--save`` (default ``qs_stack``, the stack on the pad), runs
``ascent.autopilot`` in this process with autostart, and prints one line per
flight: where it separated, what the model reserved for the booster, and
what the booster landed with (boosterland's log, named in the ascent's).

This LOADS A SAVE, which discards whatever is currently in the game.
"""

import argparse
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc  # noqa: E402

from ascent import autopilot  # noqa: E402
from ascent.config import Config, apply_overrides  # noqa: E402
from common import timescale as ts  # noqa: E402
from common.logbook import Logbook  # noqa: E402

SETTLE_TIMEOUT_S = 120.0


def instance_ports(n):
    d = os.path.join(ROOT, "testInstances", "ksp%s" % n)
    rpc = int(open(os.path.join(d, ".rpc_port")).read())
    stream = int(open(os.path.join(d, ".stream_port")).read())
    return rpc, stream


def connect(args, name):
    return krpc.connect(name=name, address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)


def load_save(args):
    conn = connect(args, "stackfly-loader")
    try:
        conn.space_center.load(args.save)
    except Exception:                   # noqa: BLE001 -- the load drops us
        pass
    finally:
        try:
            conn.close()
        except Exception:               # noqa: BLE001
            pass
    deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < deadline:
        time.sleep(1.0)
        try:
            conn = connect(args, "stackfly")
            v = conn.space_center.active_vessel
            ut0 = conn.space_center.ut
            time.sleep(1.0)
            if conn.space_center.ut > ut0 and v.parts.engines:
                time.sleep(2.0)         # let the vessel settle on the pad
                return conn
            conn.close()
        except Exception:               # noqa: BLE001
            try:
                conn.close()
            except Exception:           # noqa: BLE001
                pass
    raise RuntimeError("save %r never settled" % args.save)


SEP = re.compile(r"separating: alt=(\S+) downrange=(\S+) v_away=(\S+) "
                 r"v_up=(\S+) .* booster=(\S+)t prop=(\S+)t .* left=(\S+)t")
REPORT = re.compile(r"booster propellant: (\S+) t at separation, model needed "
                    r"(\S+) t \(reserve (\S+) t\); (\S+) t left")
BLOG = re.compile(r"booster handed to boosterland: (\S+)")
TOUCH = re.compile(r"TOUCHDOWN .* pad=\s*([\d.]+)")
SHUT = re.compile(r"shutdown: (.*)")
DOWN = re.compile(r"phase THROTTLE_DOWN -> SEPARATE (.*)")


def summarise(path):
    body = open(path).read()
    out = {"log": os.path.basename(path)}
    m = SEP.search(body)
    if m:
        out.update(alt=float(m.group(1)), dr=float(m.group(2)),
                   va=float(m.group(3)), vu=float(m.group(4)))
    m = REPORT.search(body)
    if m:
        out.update(prop_sep=float(m.group(1)), need=float(m.group(2)),
                   reserve=float(m.group(3)), left=float(m.group(4)))
    m = DOWN.search(body)
    out["sep"] = m.group(1) if m else "-"
    m = BLOG.search(body)
    if m:
        out["blog"] = os.path.basename(m.group(1))
        try:
            b = open(m.group(1)).read()
            t = TOUCH.search(b)
            out["pad"] = float(t.group(1)) if t else float("nan")
            s = SHUT.findall(b)
            out["bend"] = s[-1] if s else "-"
        except OSError:
            pass
    s = SHUT.findall(body)
    out["end"] = s[-1] if s else "-"
    return out


def fly_once(args):
    tsdir = ts.instance_dir(args.instance, args.rpc_port)
    if args.timescale is not None:
        ts.write(tsdir, args.timescale)
    conn = load_save(args)
    cfg = apply_overrides(Config(), args.set)
    if args.timescale not in (None, "off"):
        cfg.LOOP_PACING_GAME_TIME = True
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    try:
        autopilot.run(conn, cfg, log, autostart=True,
                      booster_set=args.booster_set)
    finally:
        log.close()
        try:
            conn.close()
        except Exception:               # noqa: BLE001
            pass
    return summarise(log.path)


def fmt(r):
    def g(k, f="%.0f"):
        v = r.get(k)
        return "-" if v is None else f % v
    return ("%s sep alt=%s dr=%s va=%s vu=%s | prop %s t need %s reserve %s "
            "-> landed with %s t | %s pad=%s (%s) | %s"
            % (r["log"], g("alt"), g("dr"), g("va"), g("vu"),
               g("prop_sep", "%.2f"), g("need", "%.2f"), g("reserve", "%.2f"),
               g("left", "%.2f"), r.get("blog", "-"), g("pad"),
               r.get("bend", "-"), r["end"]))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--instance", default="0")
    p.add_argument("--address", default="127.0.0.1")
    p.add_argument("--save", default="qs_stack")
    p.add_argument("-n", type=int, default=1)
    p.add_argument("--set", action="append", default=[])
    p.add_argument("--booster-set", action="append", default=[])
    p.add_argument("--timescale", default=None)
    args = p.parse_args(argv)
    args.rpc_port, args.stream_port = instance_ports(args.instance)
    for i in range(args.n):
        r = fly_once(args)
        print("%d/%d %s" % (i + 1, args.n, fmt(r)), flush=True)


if __name__ == "__main__":
    main()
