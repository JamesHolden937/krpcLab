#!/usr/bin/env python3
"""Fly the spaceplane from a quicksave and report where it stopped.

TEMPORARY TEST HARNESS -- not part of the flight software, and the same idea
as ``quickfly.py``: load a save, press START on the panel itself, wait for the
autopilot to finish, and print one line saying what happened.  In-game
measurement is the bottleneck, so the loop has to be pressable by a script.

    ./spaceplane/tools/quickglide.py -n 3
    ./spaceplane/tools/quickglide.py -n 1 --instance 0 --set AERO_REFRESH_UT=0.5
    ./spaceplane/tools/quickglide.py -n 3 --instance 0 --timescale 4.0

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
sys.path.insert(0, os.path.join(ROOT, "tools"))
import farmflight                                       # noqa: E402
from farmflight import claim_log, log_names            # noqa: E402




# Game time from a save's UT to the autopilot's first tick in the game's
# governed load (load, 1 s clock check, pause): 1.14-1.16 s, LOG3380/3381.
SIM_START_LAG_S = 1.15

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


def set_paused(rpc, stream, paused, settle=0.5):
    """Pause or unpause, and say whether the game clock obeyed.

    ``KRPC.paused`` reads back whatever was written even when the game runs
    on regardless -- the time-scale plugin used to rewrite the time scale
    every frame -- so the answer is the clock, not the flag.
    """
    try:
        conn = krpc.connect(name="quickglide-pause", rpc_port=rpc,
                            stream_port=stream) if rpc else \
            krpc.connect(name="quickglide-pause")
    except Exception:                                   # noqa: BLE001
        return None
    try:
        conn.krpc.paused = paused
        first = conn.space_center.ut
        time.sleep(settle)
        return (conn.space_center.ut == first) == paused
    except Exception:                                   # noqa: BLE001
        return None
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass


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

    ``KRPC.paused`` reads back whatever was written whether or not the clock
    obeys it (see ``set_paused``), so the clock is the test.
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
    rpc, stream = paths.ports(args.instance)
    before = log_names()
    tsdir = ts.instance_dir(args.instance, rpc)
    ceiling = None
    if args.timescale is not None:
        ceiling = ts.write(tsdir, args.timescale)
        # **Load at real time when the autopilot will govern the scale.**
        # The six wall-seconds below were being run at the ceiling, which is
        # nothing for an orbital save and 56 game-seconds of uncontrolled
        # fall for one taken in the air: ``qs_shuttle_cone``, saved at 12 km
        # and 221 m/s, engaged at 2.2 km in a 127 m/s dive and all three
        # flights of a batch broke up (LOG3031-3033).  The governor raises
        # the scale itself from the first tick it can serve.
        if args.timescale != "off" and not args.no_govern:
            ts.hold(tsdir)
    sim = str(args.instance).startswith("sim")
    conn = connect("quickglide-loader", rpc, stream)
    try:
        conn.space_center.load(args.save)
        save_ut = conn.space_center.ut if sim else None
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass
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
    #
    # Checked over one real-time second (the load runs at 1x, above), then
    # **paused**: the autopilot unpauses once it has engaged on the state the
    # save recorded.  The settle wait used to run with the game live, and a
    # save taken in the air fell for all of it (LOG3031-3033).
    # A simulated instance (kspSim) needs none of the game's settling: its
    # clock and pause answer at once, and a settle wait there is pure wall
    # time (six of a 15 s flight).
    running = clock_is_running(rpc, stream, seconds=0.1 if sim else 1.0)
    if running and args.timescale != "off" and not args.no_govern:
        held = set_paused(rpc, stream, True, settle=0.05 if sim else 0.5)
        time.sleep(0.0 if sim else 4.0)
        if held is not True:
            print("ksp%s: could not pause after the load -- an instance "
                  "running the old time-scale plugin ignores the pause, and "
                  "the save runs on while the autopilot starts"
                  % (args.instance if args.instance is not None else "?"),
                  file=sys.stderr)
    if sim and running and args.timescale != "off" and not args.no_govern:
        # Engage where the game would have: its load check and pause above
        # cost it SIM_START_LAG_S of game time after the save's UT, which a
        # simulated instance spends in a tenth of that.  Measured on
        # qs_shuttle_final, the gap was 42 m of height at the handover.
        conn = connect("quickglide-lag", rpc, stream)
        try:
            conn.krpc.paused = False
            conn.sim.advance_to(save_ut + SIM_START_LAG_S)
            conn.krpc.paused = True
        finally:
            conn.close()
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
    argv = farmflight.command(args, "spaceplane", "spaceplane.autopilot",
                              overrides)
    if rpc:
        argv += ["--rpc-port", str(rpc), "--stream-port", str(stream)]
    launched = time.time()
    process = farmflight.launch(argv)
    timed_out = farmflight.wait(process, args.timeout, 0.1 if sim else 2.0)

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

    log = claim_log(before, launched, process.pid)
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


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("-n", type=int, default=1)
    p.add_argument("--save", default="qs_shuttle2_rigoff")
    p.add_argument("--instance", default=None)
    p.add_argument("--timeout", type=float, default=3600.0)
    p.add_argument("--set", action="append", default=[])
    farmflight.add_arguments(p)
    p.add_argument("--timescale", default=None, metavar="SPEC",
                   help="off | max | a multiplier such as 4.0; on a farm "
                        "instance the default is 20, the governor's ceiling")
    p.add_argument("--no-govern", action="store_true",
                   help="do not let the autopilot hold the time scale down to "
                        "what its control loop can serve (see "
                        "Config.TIMESCALE_GOVERNOR); --timescale then means "
                        "a fixed scale for the whole flight, which is what "
                        "fitted the landing chain to the farm's loop rate")
    args = p.parse_args()
    # **A farm instance flies governed at a ceiling of 20 unless told
    # otherwise** (2026-10-02): the governor holds every phase to its own
    # control interval, so the ceiling only says how fast the cheap phases
    # may go.  A flight against the player's own game (no --instance) is
    # left at 1x.
    if args.timescale is None and args.instance is not None:
        args.timescale = "20"
    farmflight.install_reaper()

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
