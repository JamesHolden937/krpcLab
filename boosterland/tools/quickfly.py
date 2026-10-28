#!/usr/bin/env python3
"""Fly the autoland repeatedly from a quicksave, unattended.

TEMPORARY TEST HARNESS -- not part of the flight software.

`boosterland/tests/fakeksp.py` is a point mass: it has no aerodynamic torque, its drag
ignores attitude entirely, and its atmosphere is isothermal, so the estimator
noise and the transonic behaviour that actually limit this booster's accuracy
are invisible to it.  This runs the *real* `Autoland` against the *real* game
instead, from the same entry state every time, so a parameter can be measured
rather than guessed.

    ./boosterland/tools/quickfly.py -n 5
    ./boosterland/tools/quickfly.py -n 3 --set CORRECTION_MAX_BURNS=0
    ./boosterland/tools/quickfly.py --compare CORRECTION_ENTER_M=300,1200

Each run loads `--save` (default "quicksave", the post-separation state at
UT 2843.5), pauses it, and flies the booster from the single-file bundle
(``--tree``: the modules) with ``--autostart`` in a child process, which
engages on the paused state and unpauses.  It flies to touchdown or
`--timeout`, and the harness records where the booster ended up.  Logs land
in `logs/` as usual.

This LOADS A SAVE, which discards whatever is currently in the game.
"""

import os
import argparse
import re
import statistics
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc

from boosterland.config import Config

from common import timescale as ts
sys.path.insert(0, os.path.join(ROOT, "tools"))
import farmflight  # noqa: E402

SETTLE_TIMEOUT_S = 90.0
SETTLE_POLL_S = 0.5      # game time keeps running while we wait, so poll fast
SIM_START_LAG_S = 5.8    # game time from the save to START in the game (LOG3094-3097)
DONE = re.compile(r"shutdown: (\S+) ([\d.]+) m from pad, alt=(-?[\d.]+) spd=([\d.]+)")
# Fallback, and the number that is always right: the last telemetry line's own
# great-circle distance from the pad.
LAST_PAD = re.compile(r"^\[.*\bpad=\s*([\d.]+).*$", re.M)
SHUTDOWN = re.compile(r"shutdown: (.*)")
CORRECTION = re.compile(r"-> CORRECTION\b")


def connect(args, name):
    return krpc.connect(name=name, address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)


def load_save(args):
    """Load the save and wait for the game to hand back a flyable vessel.

    `load` invalidates every object reference the connection holds -- and the
    connection itself, while the scene is being rebuilt -- so this reconnects
    rather than trying to reuse anything across the load.
    """
    conn = connect(args, "quickfly-loader")
    sim = str(args.instance or "").startswith("sim")
    try:
        conn.space_center.load(args.save)
        if sim:
            # A simulated instance loads at once and would otherwise run on
            # while this harness polls in wall time: freeze it on the save.
            conn.krpc.paused = True
            save_ut = conn.space_center.ut
    finally:
        try:
            conn.close()
        except Exception:
            pass
    if sim:
        # The game reaches START 5.68-5.88 s of game time after the save's
        # UT (LOG3094-3097: load, settle, reconnect); fly the sim from the
        # same place on the trajectory, or its start is kilometres off.
        conn = connect(args, "quickfly")
        conn.krpc.paused = False       # the sim does not advance while paused
        conn.sim.advance_to(save_ut + SIM_START_LAG_S)
        vessel = conn.space_center.active_vessel
        return conn, conn.space_center.ut, \
            vessel.flight(vessel.orbit.body.reference_frame).speed

    deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < deadline:
        time.sleep(SETTLE_POLL_S)
        try:
            conn = connect(args, "quickfly")
            vessel = conn.space_center.active_vessel
            ut = conn.space_center.ut
            flight = vessel.flight(vessel.orbit.body.reference_frame)
            speed = flight.speed
            if speed > 1.0:               # physics running, not still loading
                return conn, ut, speed
            conn.close()
        except Exception:
            try:
                conn.close()
            except Exception:
                pass
    raise RuntimeError("save %r never settled into a flyable vessel" % args.save)


def hold_for(conn, seconds):
    """Let the booster coast `seconds` of *game* time before starting.

    The quicksave is one separation state, and a bias measured against one
    state proves nothing about the next.  Holding before START moves the
    entry state along the trajectory the booster is already on -- higher,
    further downrange, different speed and attitude -- which is exactly how a
    separation a second early or late would present.  It is a real
    perturbation rather than a simulated one, and it needs no way to write
    velocities the game does not offer.
    """
    if seconds <= 0.0:
        return
    target = conn.space_center.ut + seconds
    while conn.space_center.ut < target:
        time.sleep(0.1)


def fly_once(args, overrides):
    tsdir = ts.instance_dir(args.instance, args.rpc_port)
    if args.timescale is not None:
        ts.write(tsdir, args.timescale)
    conn, ut0, speed0 = load_save(args)
    try:
        hold_for(conn, args.delay)
        # **Paused here, where START used to be pressed**: the flight is a
        # child process now (the bundle flies, as everywhere), and the
        # booster would fall for as long as it takes to start.  Its first
        # tick engages on this state and unpauses (``autoland.main``).
        # **At real time**: a pause lands a few frames after it is asked
        # for, and at 4x that was 0.2-0.8 game-seconds, a different start
        # each flight.  The scale asked for applies once the child unpauses.
        if args.timescale not in (None, "off"):
            ts.hold(tsdir)
        conn.krpc.paused = True
        if args.timescale not in (None, "off"):
            ts.write(tsdir, args.timescale)
    finally:
        try:
            conn.close()
        except Exception:
            pass
    overrides = list(overrides)
    # Off 1x, the loop has to be paced on the game clock or the control rate
    # the vehicle sees falls by exactly the speedup factor.  An explicit
    # --set LOOP_PACING_GAME_TIME=... still wins: this is a default, not a
    # policy, and the wall-clock-paced version is the thing worth A/B-ing
    # against.
    if (args.timescale not in (None, "off")
            and not any(o.startswith("LOOP_PACING_GAME_TIME") for o in overrides)):
        overrides.append("LOOP_PACING_GAME_TIME=True")
    wall0 = time.time()
    path, timed_out = farmflight.fly(args, "boosterland",
                                     "boosterland.autoland", overrides,
                                     args.timeout,
                                     ports=(args.rpc_port, args.stream_port))
    where = None
    try:
        conn = connect(args, "quickfly-report")
        try:
            vessel = conn.space_center.active_vessel
            fl = vessel.flight(vessel.orbit.body.reference_frame)
            where = (fl.latitude, fl.longitude)
        finally:
            conn.close()
    except Exception:
        where = None

    body = ""
    if path and not path.startswith("?"):
        with open(path, errors="replace") as fh:
            body = fh.read()
    shut = SHUTDOWN.findall(body)
    if timed_out:
        reason = "TIMEOUT after %.0fs" % args.timeout
    else:
        reason = shut[-1] if shut else "(no shutdown line)"
    hit = DONE.search(body)
    result = {"log": path or "-", "ut0": ut0, "speed0": speed0,
              "reason": reason,
              "corrections": len(CORRECTION.findall(body)),
              "offset": _offset(where, Config()),
              "wall": time.time() - wall0,
              # What the game ACHIEVED, not what it was told.  Asking for 4x
              # on a main thread that can sustain 1.6x gets 1.6x, and the two
              # look identical from here unless the plugin reports back.
              "ts": ts.status(tsdir)}
    if hit:
        result.update(situation=hit.group(1), distance=float(hit.group(2)),
                      altitude=float(hit.group(3)), speed=float(hit.group(4)))
    else:
        # `finish` only writes that line after two seconds in TOUCHDOWN; a run
        # that ended any other way still has every telemetry line it flew.
        pads = LAST_PAD.findall(body)
        result.update(situation="(no exit line)", altitude=float("nan"),
                      speed=float("nan"),
                      distance=float(pads[-1]) if pads else float("nan"))
    return result


def _offset(where, cfg):
    """North/east metres from the pad -- which *way* it missed, not just how far.

    A repeatable miss has a direction, and the direction is what says whether
    the booster is stopping short or flying past; the great-circle distance in
    the log cannot tell those apart.
    """
    if where is None:
        return None
    import math
    r = 600000.0
    dn = math.radians(where[0] - cfg.PAD_LAT) * r
    de = (math.radians(where[1] - cfg.PAD_LON) * r
          * math.cos(math.radians(cfg.PAD_LAT)))
    return (dn, de)


def report(label, results):
    good = [r["distance"] for r in results if r["distance"] == r["distance"]]
    print("\n== %s ==" % label)
    for i, r in enumerate(results, 1):
        off = r.get("offset")
        way = ("  N%+6.0f E%+6.0f" % off) if off else ""
        st = r.get("ts") or {}
        rate = ("  %.2fx" % st["achieved"]) if "achieved" in st else ""
        print("  %d/%d  %-22s %8.0f m  %5.1f m/s  c%-2d%s  %s  %.0fs%s"
              % (i, len(results), r["situation"], r["distance"], r["speed"],
                 r["corrections"], way, r["log"].split("/")[-1],
                 r.get("wall", float("nan")), rate))
    if good:
        print("  -> median %.0f m   mean %.0f m   worst %.0f m   (n=%d)"
              % (statistics.median(good), statistics.mean(good), max(good),
                 len(good)))
    return good


def main(argv=None):
    p = argparse.ArgumentParser(prog="quickfly", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default="127.0.0.1")
    p.add_argument("--rpc-port", type=int, default=50000)
    p.add_argument("--stream-port", type=int, default=50001)
    p.add_argument("--save", default="quicksave")
    p.add_argument("-n", "--runs", type=int, default=1)
    p.add_argument("--delay", type=float, default=0.0,
                   help="coast this many game-seconds before pressing START, "
                        "to perturb the entry state")
    p.add_argument("--delays", metavar="A,B,C",
                   help="fly --runs flights at each of these delays and print "
                        "the groups side by side")
    p.add_argument("--timescale", default=None, metavar="SPEC",
                   help="run the game faster than real time without "
                        "coarsening the physics step: off | max | 2.0")
    p.add_argument("--instance", default=None,
                   help="instance number, for locating the timescale file")
    p.add_argument("--timeout", type=float, default=420.0,
                   help="wall-clock seconds before a flight is abandoned")
    p.add_argument("--set", action="append", default=[], metavar="K=V",
                   help="override a Config field, repeatable")
    p.add_argument("--compare", metavar="FIELD=A,B[,C]",
                   help="fly --runs flights at each value of FIELD and "
                        "print the groups side by side")
    farmflight.add_arguments(p)
    args = p.parse_args(sys.argv[1:] if argv is None else argv)
    farmflight.install_reaper()

    groups = [("default", args.set)]
    if args.delays:
        groups = [("delay=%ss" % d, args.set) for d in args.delays.split(",")]
    if args.compare:
        field, _, values = args.compare.partition("=")
        groups = [("%s=%s" % (field, v), args.set + ["%s=%s" % (field, v)])
                  for v in values.split(",")]

    summary = {}
    delays = ([float(d) for d in args.delays.split(",")] if args.delays
              else [args.delay] * len(groups))
    for (label, overrides), delay in zip(groups, delays):
        args.delay = delay
        results = []
        for i in range(args.runs):
            print("flying %s  (%d/%d) ..." % (label, i + 1, args.runs), flush=True)
            results.append(fly_once(args, overrides))
            print("   %s %.0f m  %s" % (results[-1]["situation"],
                                        results[-1]["distance"],
                                        results[-1]["reason"]), flush=True)
        summary[label] = report(label, results)

    if len(summary) > 1:
        print("\n== comparison ==")
        for label, good in summary.items():
            if good:
                print("  %-28s median %6.0f   mean %6.0f   worst %6.0f"
                      % (label, statistics.median(good), statistics.mean(good),
                         max(good)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
