#!/usr/bin/env python3
"""What kind of glider is this?  Ask the game, without flying.

TEMPORARY DIAGNOSTIC HARNESS -- not part of the flight software.

``liftprobe.py`` asks a booster whether it has *any* wing: one small angle,
one slope, is it worth steering on.  A spaceplane needs the whole curve, and
for three different decisions:

  entry      the drag it can hold high up, against angle of attack, at
             hypersonic speed -- which sets the deceleration and the heat
  glide      max Cl/Cd and the angle it happens at -- which sets the range
             from entry interface, and therefore where to deorbit
  approach   Cl*A at its maximum -- which sets the stall speed, and with it
             the approach speed, the flare, and whether 2.4 km is enough

So this sweeps angle of attack from 0 to 90 degrees and keeps *both*
components of the force at every point: ``Cd*A`` along the airflow and
``Cl*A`` across it.  A booster could get away with one number against Mach
because a cylinder's drag barely cares what angle it is held at.  A wing's
does, and its lift stops being linear well before the angles an entry is
flown at -- so what comes out is a table in (alpha, Mach), not a curve.

**The pitch plane is measured, not assumed.**  A booster is axisymmetric, so
``liftprobe`` could tilt it in any plane and get the same answer.  This
vehicle has wings, and tilting it nose-up is a completely different thing
from tilting it sideways -- so which plane is which has to be established
before any of the rest means anything.  Rather than decode KSP's body-axis
convention (the nose is body +y, but the dorsal axis and the quaternion's
handedness are exactly the sort of thing this project has twice paid for
assuming), the tilt direction is *swept around the airflow* and the plane
that produces the most force across it is the pitch plane.  Same tactic as
``Environment._measure_omega`` and ``vec.rotation_onto``: ask the game which
way round it is.

That sweep has a second use.  The ratio of the largest perpendicular force
to the smallest, around one cone, is a direct measure of how wing-like the
vehicle is: 1.0 is a cylinder and this booster measured about that; a plane
should be several.  If it comes back near 1.0, the thing is a lifting body
with delusions and the entry plan needs rethinking before any code is
written.

    ./planeprobe.py 0                        # entry + approach grids
    ./planeprobe.py 0 --gear                 # ... with the gear down
    ./planeprobe.py 0 --mass 6.715           # at the post-drain mass
    ./planeprobe.py 0 --azimuth-only         # just find the pitch plane

What it *cannot* answer, and this matters: whether the vehicle can **hold**
any of these angles.  ``simulate_aerodynamic_force_at`` returns a force, not
a moment, so the aerodynamic pitching moment at an attitude the vessel is not
in is not available at any price.  The highest holdable angle of attack has
to be discovered in flight by commanding it and watching whether the vehicle
tracks -- the same closed loop ``Autoland.alignment_rate`` runs, and for the
same reason (failure 6: a commanded attitude the vehicle could not reach).
Everything printed here is what the air would do *if* the attitude were held.
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

G0 = 9.80665


def parse_floats(text):
    return [float(x) for x in text.split(",") if x.strip()]


class Probe:
    """One vessel, held still, asked what the air does from every direction.

    **The vessel is not rotated; the airflow is aimed.**  ``liftprobe.py``
    turns the vessel onto an attitude with ``vec.rotation_onto``, which is the
    right tool for an axisymmetric booster -- any roll about the airflow gives
    the same answer, so the shortest arc is as good as any other.  A winged
    aircraft is not axisymmetric, and the shortest arc then leaves the
    airframe's roll relative to the tilt plane as an accident of where the
    nose happened to start.  Measured that way the first version of this file
    read 4.44 at one azimuth and 3.18 at the azimuth 180 degrees from it, on a
    sweep that is symmetric by construction, with the discrepancy showing up
    in the ``skew`` column.

    So the attitude stays exactly as it is and the *velocity* is constructed
    from the vehicle's own axes.  ``simulate_aerodynamic_force_at`` takes the
    velocity to evaluate at, so this costs nothing, and what it buys is that
    the angle of attack and the roll of the airflow are both exactly what was
    asked for rather than whatever a quaternion convention left behind.  It is
    the opposite trade from ``DRAG_PROBE_AXIAL``, which also moved the
    airflow but did so to a direction with no physical meaning -- zero angle
    of attack along whatever way the booster happened to point.  Here every
    direction asked about is a real attitude the aircraft can be in.

    The axes are *measured*, not assumed: kRPC documents its vessel frame as
    x-right, y-forward, z-out-of-the-bottom, and
    ``transform_direction((0,1,0))`` is checked against
    ``Vessel.direction`` before anything is believed.
    """

    def __init__(self, conn, vessel, mass=None):
        self.conn = conn
        self.vessel = vessel
        self.body = vessel.orbit.body
        self.frame = self.body.reference_frame
        self.flight = vessel.flight(self.frame)
        self.equatorial = self.body.equatorial_radius
        self.mass = mass if mass else vessel.mass
        self.rotation = vessel.rotation(self.frame)
        self.position0 = vessel.position(self.frame)
        self.up = vec.unit(self.position0)

        def body_axis(axis):
            return vec.unit(conn.space_center.transform_direction(
                axis, vessel.reference_frame, self.frame))

        self.nose = vec.unit(vessel.direction(self.frame))
        forward = body_axis((0.0, 1.0, 0.0))
        if vec.angle_between(forward, self.nose) > 1.0:
            raise SystemExit(
                "vessel frame is not the documented one: body +y is %.1f deg "
                "from the nose, not 0 -- nothing below would mean anything"
                % vec.angle_between(forward, self.nose))
        self.right = body_axis((1.0, 0.0, 0.0))
        self.dorsal = vec.scale(body_axis((0.0, 0.0, 1.0)), -1.0)

    # -- geometry ---------------------------------------------------------

    def air(self, altitude):
        rho = self.body.density_at(altitude)
        try:
            c = math.sqrt(1.4 * self.body.pressure_at(altitude) / rho)
        except Exception:                               # noqa: BLE001
            c = 340.0
        return rho, c

    def position_at(self, altitude):
        """Somewhere at this altitude.  The probe divides out the air, so the
        only thing this has to get right is the density and the speed of
        sound -- which is why the whole table can be asked from orbit."""
        return vec.scale(self.up, self.equatorial + altitude)

    def tilt_axis(self, azimuth):
        """The direction the nose is raised toward, rolled about the nose.

        ``azimuth`` 0 is the dorsal -- nose up, ordinary angle of attack, lift
        toward the roof.  90 is the starboard wing, i.e. sideslip.  A wing
        should make far more force at 0 and 180 than at 90 and 270, and
        exactly as much at 0 as at 180 with the sign reversed.
        """
        t = math.radians(azimuth)
        return vec.unit(vec.add(vec.scale(self.dorsal, math.cos(t)),
                                vec.scale(self.right, math.sin(t))))

    def velocity_for(self, speed, aoa_deg, azimuth):
        """The airflow that puts the nose ``aoa_deg`` above the relative wind.

        Positive angle of attack means the wind arrives from ahead and below,
        so the velocity is the nose rotated *away* from the tilt axis.  Lift
        then acts along the tilt axis.
        """
        aoa = math.radians(aoa_deg)
        w = self.tilt_axis(azimuth)
        direction = vec.unit(vec.sub(vec.scale(self.nose, math.cos(aoa)),
                                     vec.scale(w, math.sin(aoa))))
        return vec.scale(direction, speed), w

    def force(self, position, velocity):
        return self.flight.simulate_aerodynamic_force_at(
            self.body, tuple(position), tuple(velocity), tuple(self.rotation))

    # -- measurements -----------------------------------------------------

    def resolve(self, position, velocity, w, q):
        """``(ClA, CdA, signed lift along w, skew in degrees)``.

        Drag is along the airflow and lift is everything across it -- a
        magnitude, not a projection, because the guidance banks the lift
        vector wherever it wants it.  What the airframe owes the propagator is
        *how much* lift an angle of attack makes; the direction is a control
        input, not a property.  ``skew`` is the angle between the force across
        the flow and the axis the nose was raised toward, so a clean wing
        reads near zero and anything large means the force is not where the
        geometry says it should be.
        """
        f = self.force(position, velocity)
        if f is None:
            return None
        flow = vec.unit(velocity)
        across = vec.project_out(f, flow)
        cda = -vec.dot(f, flow) / q
        cla = vec.norm(across) / q
        signed = vec.dot(across, w) / q
        skew = vec.angle_between(across, w) if vec.norm(across) > 1e-9 else 0.0
        return cla, cda, signed, skew

    def find_pitch_plane(self, altitude, speed, aoa_deg, step=15.0):
        """Which way is up, and is this a wing at all?

        Returns ``(azimuth, rows, anisotropy)``.  ``anisotropy`` is the
        largest cross-flow force around the cone over the smallest: 1.0 is a
        cylinder and this project's booster measured about that; a wing is
        several.  If it comes back near 1.0 the vehicle is a lifting body with
        delusions and the entry plan wants rethinking before any code.
        """
        position = self.position_at(altitude)
        rho, _ = self.air(altitude)
        q = 0.5 * rho * speed * speed
        if q <= 0.0:
            return None, [], 0.0
        rows = []
        azimuth = 0.0
        while azimuth < 360.0:
            velocity, w = self.velocity_for(speed, aoa_deg, azimuth)
            zero, _ = self.velocity_for(speed, 0.0, azimuth)
            hot = self.resolve(position, velocity, w, q)
            cold = self.resolve(position, zero, w, q)
            if hot is None or cold is None:
                azimuth += step
                continue
            # The *incremental* response to angle of attack: a capsule with
            # wings makes a force at zero alpha too.
            rows.append((azimuth, hot[2] - cold[2], hot[3]))
            azimuth += step
        if not rows:
            return None, [], 0.0
        # The airframe's lift response is symmetric top to bottom -- KSP's
        # stock wings have no camber -- so azimuth 0 and 180 tie for the most
        # lift, and a bare argmax is as likely to pick 180 as 0.  180 is the
        # same aircraft flying *inverted*: fine for the anisotropy test, wrong
        # for everything after it.  So among the azimuths within a few percent
        # of the best, take the one nearest upright.
        peak = max(r[1] for r in rows)
        contenders = [r for r in rows if r[1] >= 0.97 * peak]
        def from_upright(azimuth):
            return abs((azimuth + 180.0) % 360.0 - 180.0)
        best = min(contenders, key=lambda r: from_upright(r[0]))
        magnitudes = [abs(r[1]) for r in rows]
        top, bottom = max(magnitudes), min(magnitudes)
        return best[0], rows, (top / bottom if bottom > 1e-9 else float("inf"))

    def sweep(self, altitude, speed, azimuth, angles):
        """``[(alpha, ClA, CdA)]`` at one point in the air."""
        position = self.position_at(altitude)
        rho, c = self.air(altitude)
        q = 0.5 * rho * speed * speed
        if q <= 0.0:
            return [], rho, 0.0
        out = []
        for degrees in angles:
            velocity, w = self.velocity_for(speed, degrees, azimuth)
            answer = self.resolve(position, velocity, w, q)
            if answer is None:
                continue
            out.append((degrees, answer[0], answer[1]))
        return out, rho, speed / c


def report_sweep(rows, rho, mach, altitude, speed, mass):
    """One point in the air: the curve, and what it is worth."""
    if not rows:
        print("  %6.0f m %5.0f m/s  -- no answer" % (altitude, speed))
        return None
    q = 0.5 * rho * speed * speed
    best_ld = max(rows, key=lambda r: (r[1] / r[2]) if r[2] > 1e-9 else -1e9)
    best_cl = max(rows, key=lambda r: r[1])
    print("  %6.0f m %5.0f m/s  M%5.2f  rho=%.3g   "
          "max L/D %.2f @%.0fdeg   ClA max %.2f @%.0fdeg"
          % (altitude, speed, mach, rho,
             best_ld[1] / best_ld[2] if best_ld[2] else 0.0, best_ld[0],
             best_cl[1], best_cl[0]))
    print("      aoa   " + " ".join("%7.0f" % r[0] for r in rows))
    print("      ClA   " + " ".join("%7.2f" % r[1] for r in rows))
    print("      CdA   " + " ".join("%7.2f" % r[2] for r in rows))
    print("      decel " + " ".join("%7.2f" % (r[2] * q / mass) for r in rows)
          + "   m/s^2 along the flow")
    return best_ld, best_cl


def glide_numbers(probe, altitude, speeds, azimuth, angles, mass, gravity):
    """Stall, best glide and minimum sink, from lift against weight.

    For each angle of attack, find the speed at which the lift the air makes
    equals the weight -- interpolating across the probed speeds rather than
    assuming a Mach-independent coefficient -- and then read the sink rate off
    the drag at that speed.  Best glide maximises ``Cl/Cd``; minimum sink
    maximises ``Cl^1.5/Cd``, which is a slower speed and a steeper angle, and
    the two are usually 15-20% apart.  The flare lives between them.
    """
    weight = mass * gravity
    rho, c = probe.air(altitude)
    table = {}
    for speed in speeds:
        rows, _, _ = probe.sweep(altitude, speed, azimuth, angles)
        for degrees, cla, cda in rows:
            table.setdefault(degrees, []).append((speed, cla, cda))
    for samples in table.values():
        samples.sort()

    print()
    print("  glide at %.0f m, mass %.3f t, g %.2f m/s^2" %
          (altitude, mass / 1000.0, gravity))
    print("      aoa   speed   sink   L/D    ClA    CdA")
    results = []
    for degrees in sorted(table):
        samples = table[degrees]
        # lift(v) = ClA(v) * 0.5 rho v^2 ; find where that crosses the weight
        crossing = None
        for (v0, cl0, cd0), (v1, cl1, cd1) in zip(samples, samples[1:]):
            l0 = cl0 * 0.5 * rho * v0 * v0
            l1 = cl1 * 0.5 * rho * v1 * v1
            if (l0 - weight) * (l1 - weight) <= 0.0 and l1 != l0:
                f = (weight - l0) / (l1 - l0)
                crossing = (v0 + f * (v1 - v0), cl0 + f * (cl1 - cl0),
                            cd0 + f * (cd1 - cd0))
                break
        if crossing is None:
            continue
        v, cla, cda = crossing
        if cla <= 0.0 or cda <= 0.0:
            continue
        ld = cla / cda
        sink = v / ld
        results.append((degrees, v, sink, ld, cla, cda))
        print("      %4.0f  %6.1f  %5.2f  %5.2f  %5.2f  %5.2f"
              % (degrees, v, sink, ld, cla, cda))
    if not results:
        print("      nothing carried its own weight in the probed speed range")
        return None
    # Stall is the slowest speed the vehicle can *fly*, which is on the near
    # side of maximum lift.  Past that the lift falls away while the drag goes
    # on climbing, so those angles also carry the weight at a low speed --
    # this airframe does it at 50 degrees with 48 m^2 of drag and a 64 m/s
    # sink rate, and calling that the stall speed would size the whole
    # approach off a configuration that is falling out of the sky.
    peak_cl = max(results, key=lambda r: r[4])[0]
    front = [r for r in results if r[0] <= peak_cl] or results
    stall = min(front, key=lambda r: r[1])
    glide = max(front, key=lambda r: r[3])
    sink = min(front, key=lambda r: r[2])
    print()
    print("      stall      %6.1f m/s at %.0f deg   -> approach 1.3x = %.0f m/s"
          % (stall[1], stall[0], 1.3 * stall[1]))
    print("      best glide %6.1f m/s at %.0f deg   L/D %.2f, sink %.1f m/s"
          % (glide[1], glide[0], glide[3], glide[2]))
    print("      min sink   %6.1f m/s at %.0f deg   L/D %.2f, sink %.1f m/s"
          % (sink[1], sink[0], sink[3], sink[2]))
    return table, stall, glide, sink


def interpolate(table, degrees, speed):
    """``(ClA, CdA)`` at one angle of attack and speed, from the probed grid."""
    samples = table.get(degrees)
    if not samples:
        return None
    if speed <= samples[0][0]:
        return samples[0][1], samples[0][2]
    if speed >= samples[-1][0]:
        return samples[-1][1], samples[-1][2]
    for (v0, cl0, cd0), (v1, cl1, cd1) in zip(samples, samples[1:]):
        if v0 <= speed <= v1 and v1 != v0:
            f = (speed - v0) / (v1 - v0)
            return cl0 + f * (cl1 - cl0), cd0 + f * (cd1 - cd0)
    return samples[-1][1], samples[-1][2]


def fly_flare(table, alpha, rho, mass, gravity, speed, gamma, stall_speed,
              dt=0.02, limit=30.0):
    """Integrate a flare: hold ``alpha`` and see whether the descent arrests.

    The reason this is integrated rather than estimated is that the two things
    the flare needs are in direct competition.  Arresting the sink needs a
    load factor above one, which needs a high angle of attack; a high angle of
    attack on this airframe drags at 50 m^2, which at 48 m/s is 10 m/s^2 of
    deceleration -- so the manoeuvre eats the very airspeed the lift is made
    from.  Whether it converges before the wing runs out of speed is not
    something a closed form answers honestly, and it is the whole question of
    whether the vehicle can land at all.

    Returns ``(outcome, height used, speed at the end, seconds)`` where
    outcome is "arrested", "stalled" or "ran out".
    """
    t = 0.0
    height = 0.0
    while t < limit:
        got = interpolate(table, alpha, speed)
        if got is None:
            return "no data", height, speed, t
        cla, cda = got
        q = 0.5 * rho * speed * speed
        lift = cla * q
        drag = cda * q
        # Flight-path form: gravity adds speed in a descent and the lift
        # surplus over the weight component is what turns the path upward.
        dv = -drag / mass - gravity * math.sin(gamma)
        dgamma = (lift - mass * gravity * math.cos(gamma)) / (mass * speed)
        height += speed * math.sin(gamma) * dt
        speed += dv * dt
        gamma += dgamma * dt
        t += dt
        if speed <= stall_speed:
            return "stalled", -height, speed, t
        if gamma >= 0.0:
            return "arrested", -height, speed, t
    return "ran out", -height, speed, t


def flare_study(table, rho, mass, gravity, stall, angles):
    """Which approach speeds can actually be flared, and to what.

    The answer this airframe gives is not the textbook one.  1.3 times the
    stall speed is the conventional approach, and here it is *too slow to land
    on*: the flare has to bleed more speed than the margin contains, so the
    vehicle stalls in the manoeuvre.  What works is a much faster, shuttle-ish
    approach with a long rollout -- which is a runway-length question, not an
    aerodynamic one, and that is why this prints the touchdown speed rather
    than a verdict.
    """
    peak_cl = max(angles, key=lambda a: (interpolate(table, a, stall[1]) or
                                         (0.0, 1.0))[0])
    print()
    print("  flare, holding %.0f deg (maximum lift), stall %.1f m/s"
          % (peak_cl, stall[1]))
    print("      approach   x Vs   sink   outcome      height   touchdown")
    for multiple in (1.3, 1.5, 1.7, 1.9, 2.1, 2.4, 2.7, 3.0):
        speed = multiple * stall[1]
        got = interpolate(table, peak_cl, speed)
        if got is None:
            continue
        # Arrive on the angle that carries the weight at this speed -- and
        # search only the *front* side of the lift curve for it.  Past maximum
        # lift the curve comes back down, so a fast approach can be matched to
        # the weight at a post-stall angle just as well as at a sensible one,
        # and the picker will happily choose it: with 50 and 65 degrees in the
        # angle list this printed an 81 m/s sink rate and a 144 m flare for
        # the 2.4 Vs row while its neighbours were at 22 m/s and 11 m.
        front = [a for a in angles if a <= peak_cl] or list(angles)
        approach_alpha = min(
            front,
            key=lambda a: abs((interpolate(table, a, speed) or (0.0, 1.0))[0]
                              * 0.5 * rho * speed * speed - mass * gravity))
        cla, cda = interpolate(table, approach_alpha, speed)
        gamma = -math.atan2(cda, cla) if cla > 0.0 else -math.pi / 4.0
        outcome, height, final, _ = fly_flare(
            table, peak_cl, rho, mass, gravity, speed, gamma, stall[1])
        print("      %6.1f    %5.2f  %5.1f   %-11s  %5.1f m   %6.1f m/s"
              % (speed, multiple, abs(speed * math.sin(gamma)), outcome,
                 height, final))
    print("      (sink is what the approach arrives at; height is what the")
    print("       flare consumes; touchdown is the speed the wheels see)")


def connect_instance(instance, name, save=None):
    port = open(os.path.join(HERE, "ksp" + instance, ".rpc_port")).read()
    stream = open(os.path.join(HERE, "ksp" + instance,
                               ".stream_port")).read()

    def dial(tag):
        return krpc.connect(name=tag, rpc_port=int(port),
                            stream_port=int(stream))

    if not save:
        return dial(name)
    # ``load`` invalidates the connection and every object reference on it,
    # so reconnect rather than reuse -- same reason quickfly.py does.
    conn = dial(name + "-loader")
    try:
        conn.space_center.load(save)
    finally:
        try:
            conn.close()
        except Exception:                               # noqa: BLE001
            pass
    deadline = time.time() + 120.0
    while time.time() < deadline:
        time.sleep(2.0)
        conn = None
        try:
            conn = dial(name)
            vessel = conn.space_center.active_vessel
            if vessel.orbit.body.name and len(vessel.parts.all) > 1:
                return conn
        except Exception:                               # noqa: BLE001
            pass
        if conn is not None:
            try:
                conn.close()
            except Exception:                           # noqa: BLE001
                pass
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance")
    p.add_argument("--save", default="qs_plane")
    p.add_argument("--mass", type=float, default=None,
                   help="tonnes to compute the derived speeds at "
                        "(default: what the vessel weighs now)")
    p.add_argument("--gear", action="store_true",
                   help="drop the gear before probing (it is on Gear, and "
                        "SmallGearBay carries ModuleDragModifier)")
    p.add_argument("--settle", type=float, default=4.0,
                   help="seconds for the gear animation to finish; probing "
                        "mid-deployment measures a configuration nothing flies")
    p.add_argument("--angles", default="0,2,5,8,12,16,20,25,30,40,50,65,90")
    p.add_argument("--entry", default="30000:2200,30000:1600,"
                                      "20000:1400,20000:900,"
                                      "10000:700,10000:400",
                   help="alt:speed pairs for the hypersonic/entry table")
    p.add_argument("--approach-alt", type=float, default=100.0)
    p.add_argument("--approach-speeds", default="50,70,90,110,130,160,200,250")
    p.add_argument("--azimuth", type=float, default=None,
                   help="skip the search and tilt in this plane")
    p.add_argument("--azimuth-only", action="store_true")
    p.add_argument("--azimuth-aoa", type=float, default=10.0)
    args = p.parse_args()

    conn = connect_instance(args.instance, "planeprobe", args.save)
    if conn is None:
        print("save %r never settled into a vessel" % args.save)
        return 2
    vessel = conn.space_center.active_vessel
    if args.gear:
        vessel.control.gear = True
        time.sleep(args.settle)

    mass = args.mass * 1000.0 if args.mass else None
    probe = Probe(conn, vessel, mass=mass)
    body = probe.body
    gravity = body.surface_gravity

    print("# vessel=%s  parts=%d  mass now=%.3f t  probing at %.3f t"
          % (vessel.name, len(vessel.parts.all), vessel.mass / 1000.0,
             probe.mass / 1000.0))
    print("# body=%s  R=%.0f m  g=%.2f m/s^2  atmosphere=%.0f m  gear=%s"
          % (body.name, body.equatorial_radius, gravity,
             body.atmosphere_depth, "down" if args.gear else "up"))

    angles = parse_floats(args.angles)

    # -- which plane is the pitch plane -----------------------------------
    if args.azimuth is None:
        azimuth, rows, anisotropy = probe.find_pitch_plane(
            10000.0, 700.0, args.azimuth_aoa)
        if azimuth is None:
            print("could not establish a pitch plane -- no usable geometry")
            return 2
        print()
        print("# pitch plane: tilt azimuth %.0f deg gives the most lift; "
              "anisotropy %.2f (1.0 = a cylinder)" % (azimuth, anisotropy))
        print("#   azimuth:lift(skew)  " + "  ".join(
            "%.0f:%.2f(%.0f)" % (a, l, k) for a, l, k in rows))
        if anisotropy < 1.5:
            print("#   WARNING: this vehicle is nearly axisymmetric.  Either "
                  "the wings are doing very little, or the probe is not "
                  "seeing them.")
    else:
        azimuth = args.azimuth
        print()
        print("# pitch plane: azimuth %.0f deg, as given" % azimuth)
    if args.azimuth_only:
        return 0

    # -- entry / hypersonic -----------------------------------------------
    print()
    print("=== entry: what the air will do at angle of attack ===")
    for pair in [x for x in args.entry.split(",") if x.strip()]:
        altitude, _, speed = pair.partition(":")
        rows, rho, mach = probe.sweep(float(altitude), float(speed), azimuth,
                                      angles)
        report_sweep(rows, rho, mach, float(altitude), float(speed),
                     probe.mass)

    # -- approach ----------------------------------------------------------
    print()
    print("=== approach: stall, glide and sink ===")
    answer = glide_numbers(probe, args.approach_alt,
                           parse_floats(args.approach_speeds), azimuth,
                           angles, probe.mass, gravity)
    if answer:
        table, stall, _, _ = answer
        rho, _ = probe.air(args.approach_alt)
        flare_study(table, rho, probe.mass, gravity, stall, angles)

    return 0


if __name__ == "__main__":
    sys.exit(main())
