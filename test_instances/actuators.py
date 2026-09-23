#!/usr/bin/env python3
"""actuators.py <instance> [save] -- every writable thing on the vessel.

The standing audit CLAUDE.md asks for ("Every few batches, stop and audit the
whole vehicle"): list each part's modules with their fields, events and
actions, so what the craft *can* do can be diffed by eye against what the
autopilot commands.  The shuttle found five things in one afternoon that
~2800 flights on one craft never exposed, and every one of them was a module
setting left at whatever the craft file said.

Also prints the vessel-level authorities kRPC reports (torque by source,
inertia) and the kRPC auto-pilot's tuning, which the autopilot inherits.

Read-only: it loads the save (if given) and reports; it commands nothing.
"""
import os
import sys
from collections import defaultdict

import krpc

here = os.path.dirname(os.path.abspath(__file__))
n = sys.argv[1]
base = os.path.join(here, "ksp%s" % n)
rpc = int(open(os.path.join(base, ".rpc_port")).read())
stream = int(open(os.path.join(base, ".stream_port")).read())
conn = krpc.connect(name="actuators", address="127.0.0.1",
                    rpc_port=rpc, stream_port=stream)
sc = conn.space_center
if len(sys.argv) > 2:
    sc.load(sys.argv[2])
v = sc.active_vessel
print("vessel %s  %d parts  %.3f t" % (v.name, len(v.parts.all), v.mass / 1e3))

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
            fields = dict(m.fields)
            events = list(m.events)
            actions = list(m.actions)
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
