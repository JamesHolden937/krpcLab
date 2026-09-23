#!/usr/bin/env python3
"""Fly the spaceplane from a quicksave and report where it stopped.

TEMPORARY TEST HARNESS -- not part of the flight software, and the same idea
as ``quickfly.py``: load a save, press START on the panel itself, wait for the
autopilot to finish, and print one line saying what happened.  In-game
measurement is the bottleneck, so the loop has to be pressable by a script.

    ./quickglide.py -n 3
    ./quickglide.py -n 1 --instance 0 --set AERO_REFRESH_UT=0.5
    ./quickglide.py -n 3 --instance 0 --timescale 4.0

What it reports is the *signed* offset from the runway midpoint, along and
across the centreline, not a great-circle distance.  A distance cannot tell an
overshoot from an undershoot and those want opposite corrections; that
ambiguity cost the sibling project two sessions, and the runway makes it worse
-- landing 2 km long and 2 km short are both "2 km" and only one of them is
survivable.
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc                                             # noqa: E402
from common import timescale as ts                                  # noqa: E402
from common import vec                             # noqa: E402
from spaceplane.config import Config                    # noqa: E402


def ports(instance):
    if instance is None:
        return None, None
    base = os.path.join(ROOT, "testInstances", "ksp" + str(instance))
    return (int(open(os.path.join(base, ".rpc_port")).read()),
            int(open(os.path.join(base, ".stream_port")).read()))


_LIVE = []


def _install_reaper():
    """Reap the flight on SIGTERM and SIGINT, not just on the normal path.

    A ``finally`` is not enough: Python's default SIGTERM handling exits
    without unwinding, so ``pkill -f quickglide.py`` left the autopilot --
    a *child* process -- flying.  Measured, two abandoned batches left eight
    of them alive on four instances.  See the note at ``Popen``.
    """
    def handler(signum, frame):                         # noqa: ARG001
        for process in list(_LIVE):
            _reap(process)
        raise SystemExit(128 + signum)
    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        try:
            signal.signal(signum, handler)
        except Exception:                               # noqa: BLE001
            pass


def _reap(process):
    """Make sure no autopilot outlives the harness that started it.

    Signals the whole process group, because the flight is a child and an
    orphaned child keeps flying the vessel -- see the note at ``Popen``.
    Idempotent and silent: called on the normal path as well as on the
    timeout, and on the way out of an interrupt.
    """
    if process.poll() is not None:
        return
    for signaller, wait in ((signal.SIGTERM, 20.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(process.pid), signaller)
        except Exception:                               # noqa: BLE001
            try:
                process.kill()
            except Exception:                           # noqa: BLE001
                pass
        try:
            process.wait(timeout=wait)
            return
        except Exception:                               # noqa: BLE001
            continue


def connect(name, rpc, stream, tries=40):
    last = None
    for _ in range(tries):
        try:
            kwargs = {"name": name}
            if rpc:
                kwargs["rpc_port"] = rpc
                kwargs["stream_port"] = stream
            return krpc.connect(**kwargs)
        except Exception as exc:                        # noqa: BLE001
            last = exc
            time.sleep(3.0)
    raise SystemExit("could not connect: %s" % last)


def offsets(conn, cfg):
    """Signed metres from the runway midpoint, along and across it."""
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    frame = body.reference_frame
    a = tuple(body.surface_position(cfg.RUNWAY_09_LAT, cfg.RUNWAY_09_LON,
                                    frame))
    b = tuple(body.surface_position(cfg.RUNWAY_27_LAT, cfg.RUNWAY_27_LON,
                                    frame))
    middle = vec.scale(vec.add(a, b), 0.5)
    along = vec.unit(vec.sub(b, a))
    up = vec.unit(middle)
    across = vec.unit(vec.cross(up, along))
    here = vessel.position(frame)
    radius = vec.norm(middle)
    offset = vec.sub(vec.scale(vec.unit(here), radius), middle)
    return vec.dot(offset, along), vec.dot(offset, across), vessel


def clock_is_running(rpc, stream, seconds=3.0):
    """Is game time actually advancing on this instance?

    **A loaded save is not a flying game.**  An instance can come up with the
    vessel in the scene, the situation reading ``orbiting`` and kRPC happily
    answering every query, while the clock does not move at all -- rendering
    240 fps with `ut` frozen to the millisecond over three seconds, and zero
    parts on a vessel that has twenty-three.  Nothing in the harness could
    tell that from a flight: the autopilot would launch, log nothing, and the
    slot would burn the full timeout before reporting a position as though it
    meant something.

    kRPC exposes no pause flag in this version, so the clock is the test.
    """
    try:
        conn = krpc.connect(name="quickglide-clock", rpc_port=rpc,
                            stream_port=stream) if rpc else \
            krpc.connect(name="quickglide-clock")
    except Exception:                                   # noqa: BLE001
        return None
    try:
        first = conn.space_center.ut
        time.sleep(seconds)
        return conn.space_center.ut > first
    except Exception:                                   # noqa: BLE001
        return None
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass


def fly(args, index):
    rpc, stream = ports(args.instance)
    before = log_names()
    tsdir = ts.instance_dir(args.instance, rpc)
    ceiling = None
    if args.timescale is not None:
        ceiling = ts.write(tsdir, args.timescale)
    conn = connect("quickglide-loader", rpc, stream)
    try:
        conn.space_center.load(args.save)
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass
    time.sleep(6.0)
    # **Refuse to fly rather than fly slowly and say nothing.**  KSP loads
    # plugins at startup, so an instance booted before
    # ``BoosterlandTimeScale.dll`` was installed never reads the file
    # ``ts.write`` just left for it -- and nothing anywhere reports an error.
    # Asked for 4x, two instances of this farm flew at 1.2x, and the only way
    # to tell was to time the log's own clock against the wall.
    #
    # The check has to be *here* and not before the load: the plugin is a
    # ``KSPAddon.Startup.Flight`` addon, so it writes nothing at all until a
    # vessel is in the scene.  Checking it at the top of the run reports the
    # plugin missing on every freshly booted instance, which is the other
    # half of the same lie.
    running = clock_is_running(rpc, stream)
    if running is False:
        raise SystemExit(
            "ksp%s: the save loaded but game time is not advancing -- the "
            "instance is\n  wedged or paused.  Flying it would burn the whole "
            "timeout and report a\n  position as though it were a landing.  "
            "Restart the instance."
            % (args.instance if args.instance is not None else "?"))
    if args.timescale not in (None, "off") and ts.status(tsdir) is None:
        raise SystemExit(
            "%s: asked for %sx but the timescale plugin is not answering with "
            "a\n  vessel loaded.  It is loaded at KSP startup, so an instance "
            "booted\n  before the plugin was installed ignores it silently.  "
            "Restart the\n  instance, or pass --timescale off."
            % (tsdir, args.timescale))

    command = [os.path.join(ROOT, ".venv", "bin", "python"), "-m",
               "spaceplane.autopilot", "--autostart"]
    if rpc:
        command += ["--rpc-port", str(rpc), "--stream-port", str(stream)]
    overrides = list(args.set)
    # The log's config line is the only record of what flew, and the save is
    # half of that.  An explicit --set still wins.
    if not any(o.startswith("SAVE_NAME") for o in overrides):
        overrides.append("SAVE_NAME=%s" % args.save)
    # Off 1x the loop has to be paced on the game clock, or the control rate
    # the vehicle sees falls by exactly the speedup factor -- and on this
    # vehicle the control rate *is* the bank rate limiter, so a wall-paced
    # loop at 4x flies a different vehicle rather than the same one sooner.
    # An explicit --set still wins: a default, not a policy.
    if (args.timescale not in (None, "off")
            and not any(o.startswith("LOOP_PACING_GAME_TIME")
                        for o in overrides)):
        overrides.append("LOOP_PACING_GAME_TIME=True")
    # **And the scale itself is the autopilot's to set.**  A fixed multiplier
    # is a promise about the machine, not about the flight: the same 6x that
    # leaves the glide's one-second tick untouched cuts the approach's 0.1 s
    # tick to whatever kRPC can deliver, which is 2 s -- and the landing chain
    # was fitted to that (failure 63).  With the governor on, ``--timescale``
    # is a *ceiling* and each phase runs as fast as its own control interval
    # allows.  ``--no-govern`` restores the old meaning, for measuring against.
    if (args.timescale not in (None, "off") and not args.no_govern
            and not any(o.startswith("TIMESCALE_GOVERNOR") for o in overrides)):
        overrides.append("TIMESCALE_GOVERNOR=%s"
                         % os.path.join(tsdir, "timescale.txt"))
        overrides.append("TIMESCALE_GOVERNOR_MAX=%.4f" % (ceiling or 8.0))
    for item in overrides:
        command += ["--set", item]
    started = time.time()
    # **Not DEVNULL.**  Nothing prints in normal operation (CLAUDE.md), so the
    # only thing that ever reaches these two streams is a crash before the
    # logbook opened or a traceback out of the control loop -- and discarding
    # them turns "the autopilot died at 2.5 km" into a log that simply stops
    # mid-glide with no shutdown line and a vessel the game reports as 0.00 t.
    # ``run-glide.sh`` appends to the same file; so does this.
    launched = time.time()
    errors = open(os.path.join(ROOT, "logs", "stderr.log"), "a")
    try:
        # **Its own process group, so killing this harness kills the flight.**
        # The autopilot is a *child*, and ``pkill -f quickglide.py`` matches
        # only the parent: the child goes on flying, still connected, still
        # commanding the vessel. Measured on 2026-09-16, two killed batches
        # left **eight** autopilots alive on four instances -- three of them
        # on one port -- fighting each other over one aircraft. What that
        # produces is not a crash, it is plausible telemetry: a batch started
        # afterwards read +30, +42 and +61 km against a +2.7 km baseline and
        # looked exactly like a regression in the change under test, and
        # another logged ``APPROACH`` at 80 km and Mach 7.
        process = subprocess.Popen(command, cwd=ROOT,
                                   stdout=errors, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        _LIVE.append(process)
    finally:
        errors.close()
    try:
        while process.poll() is None and time.time() - started < args.timeout:
            time.sleep(2.0)
    finally:
        _reap(process)
        if process in _LIVE:
            _LIVE.remove(process)
    timed_out = process.poll() is None
    if timed_out:
        _reap(process)

    conn = connect("quickglide-report", rpc, stream)
    cfg = Config()
    try:
        along, across, vessel = offsets(conn, cfg)
        speed = vessel.flight(vessel.orbit.body.reference_frame).speed
        situation = str(vessel.situation)
        parts = len(vessel.parts.all)
        mass = vessel.mass / 1000.0
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass

    log = claim_log(before, launched)
    distance = math.hypot(along, across)
    # **Commanded is not achieved.**  KSP's physics is single-threaded, so a
    # machine that will only sustain 1.6x answers a request for 4x with 1.6x,
    # and from inside the game the two are indistinguishable.  Print what the
    # plugin measured, so a flight that quietly ran at 1x says so.
    achieved = ""
    if args.timescale not in (None, "off"):
        st = ts.status(tsdir)
        achieved = ("  %.1fx" % st["achieved"]) if st else "  ?x"
    print("%2d  %-10s  %7.0f m from the midpoint  along %+8.0f  across %+8.0f"
          "  %6.1f m/s  %-22s %2d parts %5.2f t  %s%s%s"
          % (index, args.save, distance, along, across, speed, situation,
             parts, mass,
             log if (log or "").startswith("?") else os.path.basename(log or "-"),
             achieved,
             "  TIMEOUT" if timed_out else ""))
    return {"along": along, "across": across, "distance": distance,
            "speed": speed, "situation": situation, "parts": parts,
            "log": log, "timeout": timed_out}


def log_names():
    directory = os.path.join(ROOT, "logs")
    return set(n for n in os.listdir(directory)
               if n.startswith("LOG") and n[3:].isdigit())


def claim_log(before, launched):
    """Which log this flight wrote, given the ones that existed before it.

    **"The highest-numbered log" is wrong the moment two instances fly at
    once**, which is the only way this harness is worth running: it returns
    whichever instance happened to open a log last, so a sweep attributes one
    instance's trace to another instance's result.  ``Logbook`` itself claims
    its number atomically (``O_CREAT | O_EXCL``) and is not the problem --
    the number is claimed correctly and then read back by the wrong rule.

    So diff against a snapshot taken before the launch, and where several
    flights started together, take the one whose file appeared soonest after
    *this* subprocess did.  If that still cannot separate them, say so with
    the candidates rather than name one: a sweep that quietly cites the wrong
    log is worse than one that admits it does not know which.
    """
    directory = os.path.join(ROOT, "logs")
    fresh = sorted(log_names() - before, key=lambda n: int(n[3:]))
    if not fresh:
        return None
    if len(fresh) == 1:
        return os.path.join(directory, fresh[0])
    timed = sorted((abs(opened_at(os.path.join(directory, n)) - launched), n)
                   for n in fresh)
    if len(timed) > 1 and timed[1][0] - timed[0][0] < 1.0:
        return "?" + "|".join(n for _, n in timed[:3])
    return os.path.join(directory, timed[0][1])


def opened_at(path):
    """When the log was opened, from its own first line.

    **Not the mtime**, which is what this used first: a log still being
    written has an mtime of *now*, so every candidate looked equally recent
    and the tiebreak reported an ambiguity that was not there.  The header
    line is written once and never moves.
    """
    try:
        with open(path, errors="replace") as fh:
            m = re.search(r"opened (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)",
                          fh.readline())
        if m:
            return time.mktime(time.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
    except OSError:
        pass
    return float("inf")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("-n", type=int, default=1)
    p.add_argument("--save", default="qs_plane")
    p.add_argument("--instance", default=None)
    p.add_argument("--timeout", type=float, default=3600.0)
    p.add_argument("--set", action="append", default=[])
    p.add_argument("--timescale", default=None, metavar="SPEC",
                   help="off | max | a multiplier such as 4.0")
    p.add_argument("--no-govern", action="store_true",
                   help="do not let the autopilot hold the time scale down to "
                        "what its control loop can serve (see "
                        "Config.TIMESCALE_GOVERNOR); --timescale then means "
                        "a fixed scale for the whole flight, which is what "
                        "fitted the landing chain to the farm's loop rate")
    args = p.parse_args()
    _install_reaper()

    results = []
    for i in range(1, args.n + 1):
        results.append(fly(args, i))
    if len(results) > 1:
        good = [r for r in results if not r["timeout"]]
        if good:
            distances = sorted(r["distance"] for r in good)
            print("   median %.0f m, mean %.0f m, worst %.0f m over %d flights"
                  % (distances[len(distances) // 2],
                     sum(distances) / len(distances), distances[-1],
                     len(distances)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
