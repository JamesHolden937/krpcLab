#!/usr/bin/env python3
"""Flight test: fly scripted control inputs on a save and record every frame.

    ./kspSim/tools/flighttest.py --instance 0 --save qs_cone --script pitch -o rec.jsonl

The game (``--instance N``) or the simulator (``--instance simN``) loads the
save at 1x, pauses on the saved state, then flies a fixed schedule of raw
control inputs -- no autopilot, SAS off -- while a stream records the state
at every server tick: UT, position, velocity, rotation and angular velocity
in the body's non-rotating frame, the control inputs the vessel actually
received, mass, and the aerodynamic force in body axes.

The recording is the measurement ``fidelity.py`` replays through the
simulator.  An open-loop script is the point: whatever an autopilot would do
with the vehicle, the physics of the vehicle is what these frames record.

The first line is a header (save, script, instance); every other line is one
frame.  Scripts are in ``SCRIPTS``: (seconds from start, {control: value}).
"""
import argparse
import faulthandler
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc  # noqa: E402
from common import timescale as ts  # noqa: E402


def doublet(axis, amp=0.5, t0=2.0, width=1.0):
    return [(t0, {axis: amp}), (t0 + width, {axis: -amp}), (t0 + 2 * width, {axis: 0.0})]


def multistep(axis, amp=0.5, t0=2.0):
    """3-2-1-1: the flight-test input that excites a band of frequencies."""
    return [(t0, {axis: amp}), (t0 + 1.5, {axis: -amp}), (t0 + 2.5, {axis: amp}),
            (t0 + 3.0, {axis: -amp}), (t0 + 3.5, {axis: 0.0})]


SCRIPTS = {
    # Hands off: static stability, drag, lift at trim.
    "free": ([], 20.0),
    "pitch": (doublet("pitch", 0.5) + multistep("pitch", 0.3, 8.0), 16.0),
    "roll": (doublet("roll", 0.5) + multistep("roll", 0.3, 8.0), 16.0),
    "yaw": (doublet("yaw", 0.5) + multistep("yaw", 0.3, 8.0), 16.0),
    # Axes together: surfaces clamp their mixed input, so this is the
    # combination the per-surface model exists for.
    "mix": ([(2.0, {"pitch": 0.6, "roll": 0.6}), (3.0, {"pitch": -0.4, "roll": -0.8, "yaw": 0.4}),
             (4.0, {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}),
             (7.0, {"pitch": 1.0, "yaw": -1.0}), (7.6, {"pitch": 0.0, "yaw": 0.0})], 14.0),
    "gear": ([(2.0, {"gear": True}), (10.0, {"brakes": True}), (14.0, {"brakes": False})], 18.0),
    "brakes": ([(2.0, {"brakes": True}), (8.0, {"brakes": False})], 12.0),
    # Engines: throttle steps, with a pitch and a yaw input under thrust
    # (gimbal torque).
    "throttle": ([(1.0, {"throttle": 1.0}), (4.0, {"pitch": 0.5}), (5.0, {"pitch": 0.0}),
                  (6.0, {"yaw": 0.5}), (7.0, {"yaw": 0.0}), (8.0, {"throttle": 0.4}),
                  (11.0, {"throttle": 0.0})], 13.0),
    # Held pitch input: the angle of attack each level can trim to (where
    # the surfaces' authority meets the airframe's stability).
    "trim": ([(1.0, {"pitch": 0.5}), (4.0, {"pitch": 1.0}), (8.0, {"pitch": 0.0}),
              (10.0, {"pitch": -0.5}), (12.0, {"pitch": 0.0})], 14.0),
    # Long hands-off: for thermal and slow drift.
    "long": ([], 90.0),
    # Wheels alone, full steps on each axis and all three: in vacuum the
    # torque is known exactly, so the angular acceleration measures the
    # inertia tensor.
    "inertia": ([(0.0, {"rcs": False}), (1.0, {"pitch": 1.0}), (2.0, {"pitch": -1.0}),
                 (3.0, {"pitch": 0.0}), (4.0, {"roll": 1.0}), (5.0, {"roll": -1.0}),
                 (6.0, {"roll": 0.0}), (7.0, {"yaw": 1.0}), (8.0, {"yaw": -1.0}),
                 (9.0, {"yaw": 0.0}), (10.0, {"pitch": 1.0, "roll": -1.0, "yaw": 1.0}),
                 (11.0, {"pitch": 0.0, "roll": 0.0, "yaw": 0.0})], 12.0),
    # RCS on: rotation at two levels per axis, then each translation axis.
    "rcs": ([(0.0, {"rcs": True}), (1.0, {"pitch": 0.5}), (2.0, {"pitch": -1.0}),
             (3.0, {"pitch": 0.0}), (4.0, {"roll": 0.5}), (5.0, {"roll": -1.0}),
             (6.0, {"roll": 0.0}), (7.0, {"yaw": 0.5}), (8.0, {"yaw": -1.0}),
             (9.0, {"yaw": 0.0}), (10.0, {"forward": 1.0}), (11.0, {"forward": -0.5}),
             (12.0, {"forward": 0.0, "right": 1.0}), (13.0, {"right": -0.5}),
             (14.0, {"right": 0.0, "up": 1.0}), (15.0, {"up": -0.5}),
             (16.0, {"up": 0.0, "pitch": 0.3, "roll": 0.3, "forward": 0.5}),
             (17.0, {"pitch": 0.0, "roll": 0.0, "forward": 0.0})], 18.0),
}

AXES = ("pitch", "yaw", "roll", "throttle", "forward", "up", "right")


def ports(instance):
    base = os.path.join(ROOT, "testInstances", "ksp" + str(instance))
    return (int(open(os.path.join(base, ".rpc_port")).read()),
            int(open(os.path.join(base, ".stream_port")).read()))


def connect(instance, name="flighttest"):
    rpc, stream = ports(instance)
    last = None
    for _ in range(30):
        try:
            return krpc.connect(name=name, rpc_port=rpc, stream_port=stream)
        except Exception as exc:                        # noqa: BLE001
            last = exc
            time.sleep(1.0)
    raise SystemExit("could not connect to %s: %s" % (instance, last))


def load_paused(instance, save):
    """Load and freeze at once: a save in the air falls while it is left
    running, and the recorded state is where the replay starts."""
    if not str(instance).startswith("sim"):
        ts.hold(ts.instance_dir(instance))
    conn = connect(instance)
    try:
        conn.space_center.load(save)
    except Exception:                                   # noqa: BLE001
        pass
    for _ in range(60):
        try:
            conn.krpc.paused = True
            v = conn.space_center.active_vessel
            v.parts.all
            return conn
        except Exception:                               # noqa: BLE001
            try:
                conn.close()
            except Exception:                           # noqa: BLE001
                pass
            time.sleep(0.5)
            conn = connect(instance)
    raise SystemExit("load of %s did not settle" % save)


def record(args):
    if args.attach:
        # Passive: record whatever is flying (an autopilot's flight), frame by
        # frame, without loading, pausing or touching a control.
        conn = connect(args.instance, name="flighttest-attach")
    else:
        conn = load_paused(args.instance, args.save)
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    nr = body.non_rotating_reference_frame
    ctrl = v.control
    fl = v.flight(v.reference_frame)
    bf = body.reference_frame
    flb = v.flight(bf)
    parts = v.parts.all
    streams = {
        "ut": conn.add_stream(getattr, sc, "ut"),
        "r": conn.add_stream(v.position, nr),
        "v": conn.add_stream(v.velocity, nr),
        "q": conn.add_stream(v.rotation, nr),
        "w": conn.add_stream(v.angular_velocity, nr),
        "m": conn.add_stream(getattr, v, "mass"),
        "fa": conn.add_stream(getattr, fl, "aerodynamic_force"),
        "sit": conn.add_stream(getattr, v, "situation"),
        # Massless, so the recorded mass cannot restore it; a pod that runs
        # dry loses its wheels.
        "ec": conn.add_stream(v.resources.amount, "ElectricCharge"),
        # The vessel frame's origin is the centre of mass, so the root part's
        # position in it places the CoM: fuel drains tank by tank in the
        # game, and the recorded mass alone cannot say from which.
        "root": conn.add_stream(v.parts.root.position, v.reference_frame),
        # Switches: action groups deploy and enable surfaces (the booster's
        # grid fins), brakes deploy airbrakes; a replay has to fire them.
        "brakes_on": conn.add_stream(getattr, ctrl, "brakes"),
        "rcs_on": conn.add_stream(getattr, ctrl, "rcs"),
        "sas_on": conn.add_stream(getattr, ctrl, "sas"),
        "gear_on": conn.add_stream(getattr, ctrl, "gear"),
    }
    for g in range(10):
        streams["ag%d" % g] = conn.add_stream(ctrl.get_action_group, g)
    for ax in AXES:
        streams[ax] = conn.add_stream(getattr, ctrl, ax)
    if args.torques:
        # What kRPC reports as available: its attitude controller tunes on
        # these, so the simulator has to report them as the game does.
        for key, attr in (("avail", "available_torque"),
                          ("avail_cs", "available_control_surface_torque"),
                          ("avail_rw", "available_reaction_wheel_torque"),
                          ("avail_rcs", "available_rcs_torque"),
                          ("avail_eng", "available_engine_torque")):
            streams[key] = conn.add_stream(getattr, v, attr)
    skins = []
    if args.thermal:
        skins = [(conn.add_stream(getattr, p, "skin_temperature"),
                  conn.add_stream(getattr, p, "temperature")) for p in parts]
    if not args.attach:
        ctrl.sas = False
        for ax in ("pitch", "yaw", "roll", "forward", "up", "right"):
            setattr(ctrl, ax, 0.0)
    events, duration = SCRIPTS[args.script] if not args.attach else ([], 0.0)
    if args.seconds:
        duration = args.seconds
    header = {"save": args.save, "script": args.script, "instance": str(args.instance),
              "axes": list(AXES), "rcs": ctrl.rcs, "gear": ctrl.gear, "brakes": ctrl.brakes,
              "events": events, "duration": duration, "wall": time.time(),
              "parts": [p.name for p in parts]}
    out = open(args.out, "w")
    out.write(json.dumps(header) + "\n")
    # Every stream's first value, outside the update condition: a stream
    # read before its first update waits for it, and the update thread
    # needs the condition to deliver it.
    for f in streams.values():
        f()
    for a, b in skins:
        a()
        b()
    ut0 = streams["ut"]()
    pending = list(events)
    if not args.attach:
        conn.krpc.paused = False
    vessel_id = v._object_id
    last_ut = None
    cond = conn.stream_update_condition
    rows = 0
    try:
        while True:
            with cond:
                conn.wait_for_stream_update(timeout=2.0)
                snap = {k: f() for k, f in streams.items()}
                temps = [(round(a(), 2), round(b(), 2)) for a, b in skins]
            ut = snap["ut"]
            t = ut - ut0
            while pending and pending[0][0] <= t:
                for k, val in pending.pop(0)[1].items():
                    setattr(ctrl, k, val)
            if ut == last_ut:
                continue
            last_ut = ut
            row = {"t": t, "ut": ut, "r": snap["r"], "v": snap["v"], "q": snap["q"],
                   "w": snap["w"], "m": snap["m"], "fa": snap["fa"],
                   "sit": str(snap["sit"]).split(".")[-1], "ec": snap["ec"],
                   "root": snap["root"], "brakes": snap["brakes_on"],
                   "rcs": snap["rcs_on"], "sas": snap["sas_on"], "gear": snap["gear_on"],
                   "ag": [snap["ag%d" % g] for g in range(10)],
                   "c": [snap[ax] for ax in AXES]}
            if args.torques:
                for key in ("avail", "avail_cs", "avail_rw", "avail_rcs", "avail_eng"):
                    row[key] = [list(snap[key][0]), list(snap[key][1])]
            if temps:
                row["skin"] = [a for a, _ in temps]
                row["temp"] = [b for _, b in temps]
            if args.oracle_every and rows % args.oracle_every == 0:
                # The oracle at the vessel's own state, while it flies: parts
                # as they are now (flexed under load), not as they load.
                pos = v.position(bf)
                f, tq = flb.simulate_aerodynamic_wrench_at(
                    body, pos, v.velocity(bf), v.rotation(bf), v.angular_velocity(bf),
                    sc.ut)
                row["oracle_bf"] = [list(f), list(tq), list(v.rotation(bf))]
            out.write(json.dumps(row) + "\n")
            rows += 1
            if t >= duration:
                break
            if args.attach and rows % 500 == 0:
                # A flight ends (or the harness reloads): stop with it.
                try:
                    if sc.active_vessel._object_id != vessel_id or \
                            str(snap["sit"]).endswith(("landed", "splashed")):
                        break
                except Exception:                       # noqa: BLE001
                    break
    finally:
        if not args.attach:
            try:
                conn.krpc.paused = True
            except Exception:                           # noqa: BLE001
                pass
        out.close()
        conn.close()
    print("%s %s %s: %d frames over %.1f s" % (args.instance, args.save, args.script, rows,
                                               duration))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True, help="N for the game, simN for the simulator")
    ap.add_argument("--save", required=True)
    ap.add_argument("--script", default="free", choices=sorted(SCRIPTS))
    ap.add_argument("--attach", action="store_true",
                    help="record the flight already flying (no load, no inputs) for --seconds")
    ap.add_argument("--seconds", type=float, default=None, help="override the script's length")
    ap.add_argument("--thermal", action="store_true", help="also record every part's temperatures")
    ap.add_argument("--torques", action="store_true",
                    help="also record kRPC's available torques (as reported)")
    ap.add_argument("--oracle-every", type=int, default=0,
                    help="call simulate_aerodynamic_wrench_at at the live state every N frames")
    ap.add_argument("-o", "--out", required=True)
    faulthandler.dump_traceback_later(float(os.environ.get("FT_DUMP", "1e9")))
    record(ap.parse_args())


if __name__ == "__main__":
    main()
