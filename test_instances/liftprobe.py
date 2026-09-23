#!/usr/bin/env python3
"""Does this booster have a wing?  Ask the game, without flying.

TEMPORARY DIAGNOSTIC HARNESS -- not part of the flight software.

``AERO_STEER`` rests on one empirical claim: that holding a few degrees off
retrograde produces a force perpendicular to the airflow big enough to move
the landing point.  In KSP that force has two sources -- the drag cube of a
tilted body (whose per-face resultant is not aligned with the airstream) and
the explicit body-lift term -- plus the grid fins, which are lifting surfaces
proper and are deployed by ``STARTUP_ACTION_GROUP``.

None of that is worth arguing about, because ``simulate_aerodynamic_force_at``
takes the attitude to evaluate at, so the question can simply be asked: probe
the real craft at 0 and at a few angles of attack, and report the component
across the airflow.  No flight, no quicksave reload, a couple of seconds.

    ./liftprobe.py 0                 # instance ksp0
    ./liftprobe.py 0 --action-group 2    # ... with the grid fins out first

What the numbers mean: ``ClA/rad`` is the slope ``guidance.solve_steer``
steers on, in m^2 per radian.  ``lat accel`` is what that is worth to *this*
vehicle at 5 degrees, in m/s^2 -- the honest figure of merit, since a minute
of it is what moves the touchdown point.  A tenth of a m/s^2 held for 60 s is
a couple of hundred metres.
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import krpc                                             # noqa: E402
from boosterland import vec                             # noqa: E402


def probe(flight, body, position, velocity, rotation):
    return flight.simulate_aerodynamic_force_at(
        body, tuple(position), tuple(velocity), tuple(rotation))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance")
    p.add_argument("--action-group", type=int, default=None,
                   help="fire this before probing (2 = grid fins)")
    p.add_argument("--save", default="quicksave",
                   help="load this first; an instance parked at the space "
                        "centre has no vessel in an airstream to ask about")
    p.add_argument("--settle", type=float, default=4.0,
                   help="seconds to let the action group's animation finish")
    p.add_argument("--angles", default="2,5,8,12,20")
    p.add_argument("--speeds", default="200,300,450,700")
    p.add_argument("--altitudes", default="3000,8000,15000,25000")
    args = p.parse_args()

    port = open(os.path.join(HERE, "ksp" + args.instance, ".rpc_port")).read()
    stream = open(os.path.join(HERE, "ksp" + args.instance,
                               ".stream_port")).read()
    def connect(name):
        return krpc.connect(name=name, rpc_port=int(port),
                            stream_port=int(stream))

    # ``load`` invalidates the connection along with every object reference on
    # it, so reconnect rather than reuse -- same reason quickfly.py does.
    if args.save:
        conn = connect("liftprobe-loader")
        try:
            conn.space_center.load(args.save)
        finally:
            try:
                conn.close()
            except Exception:               # noqa: BLE001
                pass
        deadline = time.time() + 120.0
        conn = None
        while time.time() < deadline:
            time.sleep(2.0)
            try:
                conn = connect("liftprobe")
                v = conn.space_center.active_vessel
                if v.flight(v.orbit.body.reference_frame).speed > 1.0:
                    break
                conn.close()
                conn = None
            except Exception:               # noqa: BLE001
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:       # noqa: BLE001
                        pass
                conn = None
        if conn is None:
            print("save %r never settled into a flyable vessel" % args.save)
            return 2
    else:
        conn = connect("liftprobe")
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    frame = body.reference_frame
    flight = vessel.flight(frame)
    if args.action_group is not None and args.action_group >= 0:
        vessel.control.set_action_group(args.action_group, True)
        # Grid fins and airbrakes are *animated*.  Probing the instant the
        # action group fires measures the vehicle mid-deployment, which is a
        # configuration it flies in for about a second and never lands in.
        time.sleep(args.settle)

    r = vessel.position(frame)
    rotation = vessel.rotation(frame)
    nose = vessel.direction(frame)
    up = vec.unit(r)
    mass = vessel.mass
    equatorial = body.equatorial_radius
    print("# vessel=%s mass=%.1f t  parts=%d" % (vessel.name, mass / 1000.0,
                                                 len(vessel.parts.all)))
    print("#  alt   speed   Mach    Cd*A   ClA/rad    L/D   lat accel @5deg")

    # A descent: falling steeply, so the airflow comes from below and a little
    # downrange.  The exact geometry barely matters -- what is being measured
    # is the vehicle's response to angle of attack, not this trajectory.
    east = vec.unit(vec.cross((0.0, 1.0, 0.0), up))
    for altitude in [float(x) for x in args.altitudes.split(",")]:
        position = vec.scale(up, equatorial + altitude)
        rho = body.density_at(altitude)
        try:
            c = math.sqrt(1.4 * body.pressure_at(altitude) / rho)
        except Exception:                   # noqa: BLE001
            c = 340.0
        for speed in [float(x) for x in args.speeds.split(",")]:
            flow = vec.unit(vec.add(vec.scale(up, -1.0), vec.scale(east, 0.25)))
            velocity = vec.scale(flow, speed)
            retrograde = vec.scale(flow, -1.0)
            normal = vec.unit(vec.cross(position, velocity))
            side = vec.unit(vec.cross(normal, velocity))
            q = 0.5 * rho * speed * speed
            if q <= 0.0:
                continue

            straight = vec.rotation_onto(rotation, nose, retrograde)
            if straight is None:
                print("  rotation_onto could not verify itself -- no answer")
                return 2
            base = probe(flight, body, position, velocity, straight)
            cda = -vec.dot(base, flow) / q
            base_side = vec.dot(base, side)

            row = []
            for degrees in [float(x) for x in args.angles.split(",")]:
                aoa = math.radians(degrees)
                tilted = vec.unit(vec.add(
                    vec.scale(retrograde, math.cos(aoa)),
                    vec.scale(side, math.sin(aoa))))
                turned = vec.rotation_onto(rotation, nose, tilted)
                force = probe(flight, body, position, velocity, turned)
                lift = vec.dot(force, side) - base_side
                slope = lift / (q * aoa)
                row.append((degrees, slope, lift / mass))
            # Report the middle angle as the headline and the sweep after it,
            # because a slope that changes with angle is the thing that would
            # break a linear solve.
            headline = min(row, key=lambda x: abs(x[0] - 5.0))
            drag_accel = cda * q / mass
            print("%7.0f %6.0f  %5.2f  %6.2f  %8.2f  %5.3f  %7.3f m/s^2"
                  % (altitude, speed, speed / c, cda, headline[1],
                     abs(headline[1] * math.radians(5.0) / cda) if cda else 0.0,
                     abs(headline[2] * math.radians(5.0) / math.radians(headline[0]))))
            print("        sweep: " + "  ".join(
                "%.0fdeg %.2f" % (d, s) for d, s, _ in row)
                + "   (drag %.1f m/s^2)" % drag_accel)
    return 0


if __name__ == "__main__":
    sys.exit(main())
