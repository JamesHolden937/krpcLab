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
                    rpc_port=rpc, stream_port=stream)
sc = conn.space_center
args = [a for a in sys.argv[2:] if not a.startswith("--")]
PROBE = "--probe" in sys.argv
if args:
    sc.load(args[0])
v = sc.active_vessel
print("vessel %s  %d parts  %.3f t" % (v.name, len(v.parts.all), v.mass / 1e3))

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


if PROBE:
    c = v.control
    try:
        v.auto_pilot.disengage()
    except Exception:           # noqa: BLE001
        pass
    c.sas = False
    c.throttle = 0.0
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
