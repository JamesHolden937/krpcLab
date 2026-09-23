#!/usr/bin/env python3
"""Measure the launchpad's coordinates from the game.

TEMPORARY TEST HARNESS -- not part of the flight software.

`Config.PAD_LAT`/`PAD_LON` are six decimal places, which quantises the target
to about a centimetre on Kerbin (1e-6 deg * 600 km = 1.05 cm).  That says
nothing about whether the numbers are *right*: nothing in the tree records
where they came from.  This asks the game.

kRPC will not answer directly.  `SpaceCenter.LaunchSite` exposes `name`,
`body` and `editor_facility` and no position at all, so the pad is only
observable through something standing on it.  So:

    put a craft on the pad, revert to launch, and run this.

It reads the position of the vessel's **root part** -- not the vessel, whose
`position` is the centre of mass and therefore moves with asymmetry and fuel
state.  KSP places a launching craft by snapping the root transform to the
pad's spawn point, so the root is the pad and the CoM is the rocket.

Sampling is repeated and medianed because a landed craft jitters on its part
joints and suspension; the spread is reported so the jitter is visible rather
than assumed.  Everything is done in `body.reference_frame`, the rotating
frame the rest of this code works in.

    ./padfix.py                     # measure, compare against Config
    ./padfix.py -n 1000 --interval 0.02
    ./padfix.py --raycast           # also find the deck's geometric centre

The spawn point and the centre of the concrete are not the same thing, and
which one you want is a choice of a few metres rather than a measurement.
`--raycast` finds the second: it casts rays straight down along two lines
through the spawn point and looks for the step up from the surrounding
terrain onto the pad deck.  **Run that with the pad empty** -- a craft
standing on it is also a step up from the terrain, and the scan cannot tell
the two apart.
"""

import os
import argparse
import math
import statistics
import time
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc

from boosterland.config import Config


def connect(args, name):
    return krpc.connect(name=name, address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)


def _north_east(lat, lon, ref_lat, ref_lon, radius):
    """Metres north and east from (`ref_lat`, `ref_lon`).

    The same small-angle conversion `quickfly._offset` uses, so the two
    harnesses report offsets the same way.
    """
    dn = math.radians(lat - ref_lat) * radius
    de = (math.radians(lon - ref_lon) * radius
          * math.cos(math.radians(ref_lat)))
    return dn, de


def sample_spawn_point(conn, args):
    """Median position of the root part, plus how much it moved while sampling.

    Returns (latitude, longitude, altitude, spread_m, samples).
    """
    sc = conn.space_center
    vessel = sc.active_vessel
    body = vessel.orbit.body
    frame = body.reference_frame
    root = vessel.parts.root

    situation = str(vessel.situation)
    if not any(s in situation for s in ("pre_launch", "landed", "splashed")):
        print("warning: vessel situation is %s -- this wants a craft sitting "
              "still on the pad" % situation, file=sys.stderr)

    xs, ys, zs = [], [], []
    for i in range(args.samples):
        x, y, z = root.position(frame)
        xs.append(x)
        ys.append(y)
        zs.append(z)
        if args.interval > 0.0 and i + 1 < args.samples:
            # Wait in game time: a paused game must not silently return
            # `samples` copies of one physics frame and call it a spread.  The
            # wall-clock deadline is what stops a paused game hanging this
            # instead -- it gives up on the wait, not on the run.
            target = sc.ut + args.interval
            give_up = time.time() + max(1.0, args.interval * 10.0)
            while sc.ut < target and time.time() < give_up:
                time.sleep(0.002)

    mid = (statistics.median(xs), statistics.median(ys), statistics.median(zs))
    spread = max(math.dist(p, mid) for p in zip(xs, ys, zs))

    lat = body.latitude_at_position(mid, frame)
    lon = body.longitude_at_position(mid, frame)
    alt = body.altitude_at_position(mid, frame)
    return lat, lon, alt, spread, len(xs)


def _cast_down(sc, body, frame, lat, lon, start_alt):
    """Altitude of whatever is under (`lat`, `lon`), or None if nothing is.

    Casts from `start_alt` straight down along the local vertical.  The hit
    point is reconstructed rather than differenced against the start altitude,
    because "down" is only radial at the point it was taken from.
    """
    start = body.position_at_altitude(lat, lon, start_alt, frame)
    surface = body.position_at_altitude(lat, lon, 0.0, frame)
    down = tuple(s - t for s, t in zip(surface, start))
    length = math.sqrt(sum(c * c for c in down))
    down = tuple(c / length for c in down)

    distance = sc.raycast_distance(start, down, frame)
    if not math.isfinite(distance):
        return None
    hit = tuple(s + d * distance for s, d in zip(start, down))
    return body.altitude_at_position(hit, frame)


def _scan_axis(sc, body, frame, lat, lon, radius, axis, args):
    """Altitudes along a line through (`lat`, `lon`), as (offset_m, altitude)."""
    steps = int(args.raycast_span / args.raycast_step)
    out = []
    for i in range(-steps, steps + 1):
        d = i * args.raycast_step
        if axis == "north":
            here = (lat + math.degrees(d / radius), lon)
        else:
            here = (lat, lon + math.degrees(d / (radius * math.cos(
                math.radians(lat)))))
        alt = _cast_down(sc, body, frame, here[0], here[1], args.raycast_from)
        if alt is not None:
            out.append((d, alt))
    return out


def _deck_span(profile, args):
    """The contiguous run of raised samples straddling offset 0.

    The deck is a step up from the terrain around it, so the baseline is the
    median of the outer edges of the scan -- deliberately not the whole
    profile, which the deck itself would drag upwards.
    """
    if not profile:
        return None
    edge = max(1, len(profile) // 5)
    baseline = statistics.median(
        [a for _, a in profile[:edge]] + [a for _, a in profile[-edge:]])
    raised = [(d, a) for d, a in profile if a - baseline >= args.raycast_rise]
    if not raised:
        return None

    # Walk outwards from the sample nearest offset 0 so a raised patch
    # somewhere else in the scan cannot join the answer.
    centre = min(range(len(profile)), key=lambda i: abs(profile[i][0]))
    if profile[centre][1] - baseline < args.raycast_rise:
        return None
    lo = hi = centre
    while lo > 0 and profile[lo - 1][1] - baseline >= args.raycast_rise:
        lo -= 1
    while hi + 1 < len(profile) and profile[hi + 1][1] - baseline >= args.raycast_rise:
        hi += 1
    return profile[lo][0], profile[hi][0], baseline, statistics.median(
        [a for _, a in profile[lo:hi + 1]])


def main(argv=None):
    p = argparse.ArgumentParser(
        prog="padfix", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default="127.0.0.1")
    p.add_argument("--rpc-port", type=int, default=50000)
    p.add_argument("--stream-port", type=int, default=50001)
    p.add_argument("-n", "--samples", type=int, default=400)
    p.add_argument("--interval", type=float, default=0.01,
                   help="game-seconds between samples (0 = as fast as kRPC "
                        "will answer, which may re-read one physics frame)")
    p.add_argument("--raycast", action="store_true",
                   help="also scan for the deck's geometric centre; run this "
                        "with the pad EMPTY")
    p.add_argument("--raycast-span", type=float, default=40.0,
                   help="metres either side of the spawn point to scan")
    p.add_argument("--raycast-step", type=float, default=0.5)
    p.add_argument("--raycast-from", type=float, default=400.0,
                   help="altitude to cast down from")
    p.add_argument("--raycast-rise", type=float, default=1.5,
                   help="metres above the surrounding terrain that counts as "
                        "being on the deck")
    args = p.parse_args(argv)

    cfg = Config()
    conn = connect(args, "padfix")
    try:
        sc = conn.space_center
        body = sc.active_vessel.orbit.body
        frame = body.reference_frame
        radius = body.equatorial_radius

        lat, lon, alt, spread, n = sample_spawn_point(conn, args)
        dn, de = _north_east(lat, lon, cfg.PAD_LAT, cfg.PAD_LON, radius)

        print("body            %s   equatorial radius %.0f m" % (body.name, radius))
        print("vessel          %s (%s)"
              % (sc.active_vessel.name, sc.active_vessel.situation))
        print()
        print("spawn point     lat %+.7f  lon %+.7f  alt %.2f m" % (lat, lon, alt))
        print("  sampled       n=%d, root part, spread %.4f m" % (n, spread))
        print("  vs Config     N%+.2f m  E%+.2f m   (%.2f m)"
              % (dn, de, math.hypot(dn, de)))
        print()
        print("  PAD_LAT: float = %.7f" % lat)
        print("  PAD_LON: float = %.7f" % lon)

        if args.raycast:
            print()
            print("raycast scan    +/-%.0f m at %.2f m, from %.0f m"
                  % (args.raycast_span, args.raycast_step, args.raycast_from))
            centres = {}
            for axis in ("north", "east"):
                profile = _scan_axis(sc, body, frame, lat, lon, radius, axis, args)
                span = _deck_span(profile, args)
                if span is None:
                    print("  %-6s        no deck found (terrain only)" % axis)
                    continue
                lo, hi, baseline, deck = span
                centres[axis] = (lo + hi) / 2.0
                print("  %-6s        edges %+.2f .. %+.2f m  width %.2f m  "
                      "centre %+.2f m   deck %.1f m over terrain %.1f m"
                      % (axis, lo, hi, hi - lo, centres[axis], deck, baseline))
            if len(centres) == 2:
                clat = lat + math.degrees(centres["north"] / radius)
                clon = lon + math.degrees(centres["east"] / (
                    radius * math.cos(math.radians(lat))))
                cdn, cde = _north_east(clat, clon, cfg.PAD_LAT, cfg.PAD_LON, radius)
                print()
                print("deck centre     lat %+.7f  lon %+.7f" % (clat, clon))
                print("  vs spawn      N%+.2f m  E%+.2f m"
                      % (centres["north"], centres["east"]))
                print("  vs Config     N%+.2f m  E%+.2f m   (%.2f m)"
                      % (cdn, cde, math.hypot(cdn, cde)))
                print()
                print("  PAD_LAT: float = %.7f" % clat)
                print("  PAD_LON: float = %.7f" % clon)

        print()
        print("note: changing PAD_LAT/PAD_LON does not move where the booster")
        print("      lands -- AIM_BIAS_NORTH_M/EAST_M were measured as an")
        print("      offset from the current values, so shifting the datum")
        print("      shifts the bias by the same amount.  Re-measure the bias.")
    finally:
        try:
            conn.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
