#!/usr/bin/env python3
"""actuators.py <instance> [save] [--probe] -- every writable thing on the vessel.

The standing audit CLAUDE.md asks for ("Every few batches, stop and audit the
whole vehicle"): list each part's modules with their fields, events and
actions, so what the craft *can* do can be diffed by eye against what the
autopilot commands.  The shuttle found five things in one afternoon that
~2800 flights on one craft never exposed, and every one of them was a module
setting left at whatever the craft file said.

Also prints the vessel-level authorities kRPC reports (torque by source,
inertia) and the kRPC auto-pilot's tuning, which the autopilot inherits.

Read-only: it loads the save (if given) and reports; it commands nothing --
except with ``--probe``, which flies: +-1 steps on each axis (``PROBE_STEP_S``
game-s each), first on RCS alone (engines off), then on the engine gimbals
alone (every liquid engine lit at full throttle, RCS off), and prints the
angular acceleration kRPC credits (that source's available torque over the
moment of inertia) against the one measured (the step's change in rate,
+ and - differenced so a steady aerodynamic moment cancels).  Reload the
save after it: it burns propellant.
"""
import os
import sys
from collections import defaultdict

import krpc

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(here))
from common import partmodule  # noqa: E402
n = sys.argv[1]
base = os.path.join(here, "ksp%s" % n)
rpc = int(open(os.path.join(base, ".rpc_port")).read())
stream = int(open(os.path.join(base, ".stream_port")).read())
conn = krpc.connect(name="actuators", address="127.0.0.1",
                    port=rpc)
sc = conn.space_center
args = [a for a in sys.argv[2:] if not a.startswith("--")]
PROBE = "--probe" in sys.argv
if args:
    sc.load(args[0])
v = sc.active_vessel
print("vessel %s  %d parts  %.3f t" % (v.name, len(v.parts.all), v.mass / 1e3))

if "--joints" in sys.argv:
    # The touchdown impact model's inputs and answer (common/impact.py).
    from common import impact  # noqa: E402
    parts = impact.read(conn)
    ground = set(impact.touching(parts))
    print("%3s %3s %-36s %8s %6s %7s %7s %10s %10s %10s %10s" % (
        "i", "par", "part", "mass t", "crash", "y", "low", "breakF kN", "breakT kNm",
        "spring", "damper"))
    for p in parts:
        print("%3d %3d %-36s %8.3f %6.1f %7.2f %7.2f %10s %10s %10.0f %10.0f%s" % (
            p["index"], p["parent"], p["title"][:36], p["mass"], p["crash"],
            p["y"], p["low"], "%.0f" % p["break_force"], "%.0f" % p["break_torque"],
            p["spring"], p["damper"], "  <- touches" if p["index"] in ground else ""))
    for speed in (1, 2, 4, 8):
        print("contact %d m/s: worst ratio %.2f (%s)" % (
            (speed,) + impact.peak_load(parts, speed)))
    print("survives %s" % (impact.survivable_speed(parts),))
    sys.exit(0)

PROBE_STEP_S = 0.8      # game seconds per step: several gimbal lags (0.125 s)
PROBE_REST_S = 0.8


def wait_game(dt):
    import time
    end = sc.ut + dt
    while sc.ut < end:
        time.sleep(0.01)


def body_rate():
    """Angular velocity in the vessel frame: (pitch, roll, yaw) rad/s."""
    frame = v.orbit.body.non_rotating_reference_frame
    w = v.angular_velocity(frame)
    x, y, z = sc.transform_direction(w, frame, v.reference_frame)
    return (x, y, z)


def probe(label, source):
    c = v.control
    moi = v.moment_of_inertia
    axes = ("pitch", "roll", "yaw")
    print("\n-- probe: %s" % label)
    for i, axis in enumerate(axes):
        # +1 for a step then -1 for a step: the rate comes back to where it
        # started, and a steady disturbance d cancels from
        # (a + d) - (-a + d) = 2a.
        rates, credit = [body_rate()[i]], []
        for sign in (1.0, -1.0):
            setattr(c, axis, sign)
            wait_game(PROBE_STEP_S * 0.5)
            tq = getattr(v, source)
            credit.append(abs((tq[0] if sign > 0 else tq[1])[i]) / moi[i])
            wait_game(PROBE_STEP_S * 0.5)
            rates.append(body_rate()[i])
        setattr(c, axis, 0.0)
        up = (rates[1] - rates[0]) / PROBE_STEP_S
        down = (rates[2] - rates[1]) / PROBE_STEP_S
        measured = abs(up - down) / 2.0
        cred = sum(credit) / len(credit)
        print("  %-5s credited %8.3f rad/s2  measured %8.3f  (+ %+.3f, - %+.3f)"
              "  measured/credited %.2f" % (axis, cred, measured, up, down,
                                            measured / cred if cred > 0
                                            else float("nan")))
        wait_game(PROBE_REST_S)


SURFACE_SLEW_S = 0.8     # game s for the fins to reach a step's deflection
SURFACE_MEASURE_S = 0.6  # ... then the rate's slope over this long
SURFACE_CYCLES = 4       # 0, +1, 0, -1 per cycle


def _arg(name, default, kind=float):
    for a in sys.argv:
        if a.startswith(name + "="):
            return kind(a.split("=", 1)[1])
    return default


# --probe-axes=roll,pitch  --probe-cycles=N  --surfaces-only  --deployed
AXES = _arg("--probe-axes", "pitch,roll,yaw", str).split(",")
SURFACE_CYCLES = int(_arg("--probe-cycles", SURFACE_CYCLES))


def surface_step(axis, i, value, surfaces, moi):
    """Hold ``axis`` at ``value``; after SURFACE_SLEW_S return (measured
    acceleration about axis i over SURFACE_MEASURE_S, credited rad/s2 for
    the sign of value, deflection min..max, q)."""
    setattr(v.control, axis, value)
    wait_game(SURFACE_SLEW_S)
    tq = v.available_control_surface_torque
    cred = abs((tq[0] if value >= 0 else tq[1])[i]) / moi[i]
    angles = [s.deflection_angle for s in surfaces]
    q = v.flight().dynamic_pressure
    t0, r0 = sc.ut, body_rate()[i]
    wait_game(SURFACE_MEASURE_S)
    return ((body_rate()[i] - r0) / (sc.ut - t0), cred,
            (min(angles), max(angles)), q)


def probe_surfaces(label):
    """+-1 steps on each axis on the control surfaces alone (engines off,
    RCS off), SURFACE_CYCLES cycles of 0, +1, 0, -1.  The surfaces slew
    (actuatorSpeed), so each acceleration is the rate's slope after
    SURFACE_SLEW_S, once the deflection has settled.  The effect of a step
    is its acceleration less the mean of the zero-input steps either side
    (a slowly changing aerodynamic moment cancels); credited is kRPC's
    ``available_control_surface_torque`` over the inertia, read in the same
    step.  Each cycle gives a ratio: the median and range are printed."""
    moi = v.moment_of_inertia
    surfaces = v.parts.control_surfaces
    print("\n-- probe: %s (%d surfaces)" % (label, len(surfaces)))
    for i, axis in enumerate(("pitch", "roll", "yaw")):
        if axis not in AXES:
            continue
        seq = []
        for _ in range(SURFACE_CYCLES):
            for value in (0.0, 1.0, 0.0, -1.0):
                seq.append((value, surface_step(axis, i, value, surfaces,
                                                moi)))
        seq.append((0.0, surface_step(axis, i, 0.0, surfaces, moi)))
        ratios, effs, creds, qs, defl = [], [], [], [], []
        for k in range(1, len(seq) - 1, 2):
            value, (acc, cred, d, q) = seq[k]
            base = 0.5 * (seq[k - 1][1][0] + seq[k + 1][1][0])
            # Signed: > 0 = the way RCS and the gimbals answer the same
            # input.  A positive kRPC input turns the vessel the negative
            # way about its reference_frame axis on every axis (the RCS and
            # gimbal probes: + step -> -rate), hence the minus.
            eff = -(acc - base) * value
            effs.append(eff)
            creds.append(cred)
            qs.append(q)
            defl.append(d)
            if cred > 0:
                ratios.append(eff / cred)
        ratios.sort()
        med = ratios[len(ratios) // 2] if ratios else float("nan")
        print("  %-5s credited %.4f-%.4f rad/s2  measured %+.4f..%+.4f  "
              "measured/credited median %.2f (%.2f..%.2f, n=%d)  "
              "deflection %+.1f..%+.1f deg  q %.0f-%.0f Pa"
              % (axis, min(creds), max(creds), min(effs), max(effs), med,
                 ratios[0] if ratios else float("nan"),
                 ratios[-1] if ratios else float("nan"), len(ratios),
                 min(d[0] for d in defl), max(d[1] for d in defl),
                 min(qs), max(qs)))
    v.control.pitch = v.control.roll = v.control.yaw = 0.0


def hold_until_below(alt):
    """--probe-below=ALT: kRPC's autopilot (RCS on) holds surface
    retrograde until the vessel falls below ALT m, then lets go, so the
    surfaces are probed in a steady, high-q, tail-first descent rather
    than at separation."""
    ap = v.auto_pilot
    c = v.control
    c.rcs = True
    ap.reference_frame = v.surface_velocity_reference_frame
    ap.target_direction = (0.0, -1.0, 0.0)
    ap.engaged = True
    print("holding retrograde until below %.0f m" % alt)
    flight = v.flight(v.orbit.body.reference_frame)
    while not (flight.mean_altitude < alt and flight.vertical_speed < 0):
        wait_game(1.0)
    ap.engaged = False
    c.rcs = False
    wait_game(2.0)
    print("released at %.0f m, %.0f m/s, q %.0f Pa"
          % (flight.mean_altitude, flight.speed,
             v.flight().dynamic_pressure))


if PROBE:
    c = v.control
    try:
        v.auto_pilot.engaged = False
    except Exception:           # noqa: BLE001
        pass
    c.sas = False
    c.throttle = 0.0
    c.rcs = False
    wait_game(0.5)
    below = _arg("--probe-below", None)
    if below is not None:
        hold_until_below(below)
    if v.parts.control_surfaces:
        if "--deployed" not in sys.argv:
            probe_surfaces("control surfaces only, as saved (engines off, "
                           "RCS off)")
        from boosterland.autoland import deploy_brake_fins
        from boosterland.config import Config
        print("  " + deploy_brake_fins(v, Config()))
        wait_game(2.0)
        probe_surfaces("control surfaces only, brake fins deployed")
    if "--surfaces-only" in sys.argv:
        print("\n(probe done: reload the save)")
        sys.exit(0)
    c.rcs = True
    wait_game(0.5)
    probe("RCS only (engines off, valve open)", "available_rcs_torque")
    c.rcs = False
    for e in v.parts.engines:
        try:
            if "LiquidFuel" in e.propellant_names:
                e.active = True
                e.thrust_limit = 1.0    # saves can carry 0 (EngineCluster)
                if e.gimballed:
                    e.gimbal_locked = False
        except Exception:       # noqa: BLE001
            pass
    c.throttle = 1.0
    wait_game(1.5)
    probe("gimbals only (full throttle, RCS off)", "available_engine_torque")
    c.throttle = 0.0
    print("\n(probe done: reload the save)")

def safe(f, default="?"):
    try:
        return f()
    except Exception as e:      # noqa: BLE001 -- a report, not a controller
        return "%s(%s)" % (default, type(e).__name__)

print("MoI (pitch, roll, yaw)", safe(lambda: v.moment_of_inertia))
for src in ("available_torque", "available_reaction_wheel_torque",
            "available_rcs_torque", "available_engine_torque",
            "available_control_surface_torque", "available_other_torque"):
    print("%-34s %s" % (src, safe(lambda: getattr(v, src))))
ap = v.auto_pilot
for f in ("stopping_time", "deceleration_time", "attenuation_angle",
          "time_to_peak", "overshoot", "auto_tune", "roll_threshold",
          "pitch_pid_gains", "roll_pid_gains", "yaw_pid_gains"):
    print("auto_pilot.%-18s %s" % (f, safe(lambda: getattr(ap, f))))
c = v.control
for f in ("sas", "rcs", "gear", "brakes", "lights", "abort",
          "throttle", "wheel_throttle", "wheel_steering", "input_mode"):
    print("control.%-16s %s" % (f, safe(lambda: getattr(c, f))))

# Group identical parts so a 30-part craft reads in one screen.
seen = defaultdict(list)
for p in v.parts.all:
    seen[p.name].append(p)
for name, parts in sorted(seen.items()):
    p = parts[0]
    print("\n== %s x%d  (%s)" % (name, len(parts), p.title))
    for m in p.modules:
        fields, events, actions = {}, [], []
        try:
            fields = dict(partmodule.fields(m))
            events = list(partmodule.events(m))
            actions = list(partmodule.actions(m))
        except Exception as e:  # noqa: BLE001
            print("  [%s] unreadable: %s" % (m.name, type(e).__name__))
            continue
        print("  [%s]" % m.name)
        if fields:
            print("    fields : " + "; ".join(
                "%s=%s" % kv for kv in sorted(fields.items())))
        if events:
            print("    events : " + ", ".join(events))
        if actions:
            print("    actions: " + ", ".join(actions))
