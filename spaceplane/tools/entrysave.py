#!/usr/bin/env python3
"""Quicksave a flight at the entry interface, to fly the glide from there.

TEMPORARY TEST HARNESS -- not part of the flight software.

Half of a measurement flight is a deorbit wait and a ballistic coast that
most changes do not touch: 275 s and 470 s of game time against the glide's
745 s.  Worse, the burn is a *noise source* -- the along-track scatter of one
unchanged configuration is about 10 km (spaceplane failure 27) and nothing
says how much of that is made before the vehicle ever reaches the air.

A save taken at the interface removes both.  Every flight from it starts from
byte-identical state, so the glide's own repeatability can be measured
without the deorbit underneath it, and each flight costs half as much.

    ./entrysave.py 0                       # watch instance 0, save as qs_entry
    ./entrysave.py 0 --name qs_entry_hot --alt 58000

Run it alongside a normal ``quickglide.py`` flight on the same instance: it
opens its own kRPC connection, watches the altitude, and saves once on the
way down.  It commands nothing.

The autopilot knows what to do with the result -- engaged below
``ENTRY_INTERFACE_M`` it enters GLIDE directly rather than looking for a
deorbit burn it has already flown.
"""
import os
import argparse
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", type=int)
    p.add_argument("--name", default="qs_entry")
    p.add_argument("--alt", type=float, default=58000.0,
                   help="save the first time the vehicle is below this, falling")
    p.add_argument("--low", type=float, default=52000.0,
                   help="the bottom of the window; below this is not an entry")
    p.add_argument("--sink", type=float, default=10.0,
                   help="minimum sink rate, to reject the climb out of a skip")
    p.add_argument("--timeout", type=float, default=2400.0)
    args = p.parse_args(argv)

    port = 50100 + 2 * args.instance
    conn = krpc.connect(name="entrysave", address="127.0.0.1",
                        rpc_port=port, stream_port=port + 1)
    sc = conn.space_center
    deadline = time.time() + args.timeout
    seen = None
    while time.time() < deadline:
        # **Re-fetch the vessel every poll.**  ``quickglide`` loads a save
        # *after* this connects, which replaces the scene: a handle taken
        # once points at the previous flight, and the first version of this
        # watched a wreck on the runway while the flight it meant to catch
        # went past.  Three saves came out at 65-768 m on final approach.
        try:
            vessel = sc.active_vessel
            flight = vessel.flight(vessel.orbit.body.reference_frame)
            altitude = flight.mean_altitude
            vertical = flight.vertical_speed
            speed = flight.speed
        except Exception:                                   # noqa: BLE001
            time.sleep(0.5)                 # mid-scene-change; try again
            continue
        # A *window*, not a ceiling, and falling fast.  "Below 58 km" is also
        # true on the runway, which is how the first version failed; and the
        # climbing half of a skip crosses the same altitude going up, where a
        # save is not an entry state.
        if args.low <= altitude <= args.alt and vertical < -args.sink:
            sc.save(args.name)
            print("saved %r at %.0f m, %.0f m/s down, %.1f m/s"
                  % (args.name, altitude, -vertical, speed))
            return 0
        if seen is None or abs(altitude - seen) > 5000.0:
            seen = altitude
            print("  ... %.0f m, %+.0f m/s vertical" % (altitude, vertical),
                  flush=True)
        time.sleep(0.5)
    print("timed out without a sample in [%.0f, %.0f] falling"
          % (args.low, args.alt), file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
