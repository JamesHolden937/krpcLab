#!/usr/bin/env python3
"""Fly the airplane pilot on a farm instance from a save of a plane in the
air, disturb it, and say how well it held.

    ./airplane/tools/airfly.py --instance 0                  # CRUISE, qs_warbird
    ./airplane/tools/airfly.py --instance 1 --mode 1 --set RATE_PHASE_MARGIN_DEG=50

The save (``airstart.py``: the Electric Warbird level at 100 m, 200 m/s,
full throttle) is loaded paused, the pilot is started with ``--autostart``
and ``PROP_PLANE`` (propPitch flies the blades, in-process, as live)
and takes the attitude it finds; then the game runs and the harness puts
**pulses** (``--pulses``: ``axis:value:start:length`` in game seconds after
the unpause).  ``pitch``, ``roll``, ``yaw`` are **the player's stick**: the
window mod's ``TestStick`` adds them where KSP puts the player's input,
before kRPC adds the pilot's, so the pilot reads them as the player flying
(a second kRPC client cannot: kRPC keeps one set of inputs per vessel,
``PilotAddon.manualInputs``).  ``pitch_trim`` etc. are a disturbance the
hold cannot see -- KSP adds the trim to the stick and the pilot takes it
out of what it reads as the player's -- a gust or a change of trim.  After
a stick pulse the target is wherever the stick left the attitude, so the
error is the hold settling there.

The summary per axis, from the LOG's FLY lines: the attitude error's rms
and peak, the body rate's rms, and its **reversals** -- swings of the rate
of more than ``REVERSAL_DEG_S`` that change sign, per second -- in the
quiet windows (``QUIET_S`` after each pulse ends to the next pulse) and
overall.  A loop that rings shows as reversals; a sluggish one as a large
error with few.
"""
import argparse
import math
import os
import re
import shutil
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
sys.path.insert(0, os.path.join(ROOT, "tools"))
import krpc                                             # noqa: E402
import farmflight                                       # noqa: E402
from kspSim.tools import flighttest                     # noqa: E402

# A player's flying: bank right and let go, reverse to the left, roll back
# level, pitch up, pitch down, a rudder kick; then an out-of-trim roll.
# The Warbird rolls ~180 deg/s at 0.15 of stick: 0.05 for 0.5 s banks it
# ~30 deg.  (0.15 for 0.6 s banked it 85 and CRUISE, holding that bank in a
# 4 g turn, spiralled into the sea from 100 m.)
DEFAULT_PULSES = ("roll:0.05:6:0.5,roll:-0.05:14:1.0,roll:0.05:22:0.5,"
                  "pitch:0.1:30:0.5,pitch:-0.1:38:0.5,yaw:0.2:46:0.5,"
                  "roll_trim:0.05:52:3")
QUIET_S = 1.0
ROTATE_STICK = 1.0
ROTATE_PITCH_DEG = 10.0
ROTATE_MAX_S = 4.0
REVERSAL_DEG_S = 2.0


def parse_pulses(text):
    out = []
    for item in text.split(","):
        if not item.strip():
            continue
        axis, value, start, length = item.split(":")
        out.append((axis, float(value), float(start), float(length)))
    return out


def disturb(rpc, stream, pulses, started, stop, t0_box, lead_s,
            runway=False, rotate=None):
    """Unpause once the pilot has started (on the runway: full throttle,
    brakes off), then fly the pulses: ``pitch``, ``roll`` and ``yaw`` as
    the player's stick (the window mod's ``TestStick``), ``*_trim`` as the
    trims."""
    conn = krpc.connect(name="airfly-disturb", rpc_port=rpc,
                        stream_port=stream)
    svc = conn.krpc_lab_launcher
    stick = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    c = conn.space_center.active_vessel.control
    try:
        ut = conn.add_stream(getattr, conn.space_center, "ut")
        started.wait(60.0)
        time.sleep(lead_s)
        conn.krpc.paused = False
        if runway:
            c.brakes = False
            c.throttle = 1.0
            # A jet's engines (props are flown by propPitch).
            for e in conn.space_center.active_vessel.parts.engines:
                try:
                    e.active = True
                except Exception:                       # noqa: BLE001
                    pass
        t0 = ut()
        t0_box.append(t0)
        if runway and rotate:
            # A player's rotation: stick back from ``rotate`` m/s until the
            # nose is ROTATE_PITCH_DEG up (or ROTATE_MAX_S), then let go --
            # the hold keeps the pitch it is left at.
            v = conn.space_center.active_vessel
            fl = v.flight(v.orbit.body.reference_frame)
            speed = conn.add_stream(getattr, fl, "speed")
            # (pitch from the surface frame's flight: in the body's frame
            # kRPC reads 75 deg on the runway)
            pitch = conn.add_stream(getattr, v.flight(), "pitch")
            while not stop.is_set() and speed() < rotate:
                time.sleep(0.02)
            svc.test_stick(ROTATE_STICK, 0.0, 0.0)
            t_rot = ut()
            while (not stop.is_set() and pitch() < ROTATE_PITCH_DEG
                   and ut() - t_rot < ROTATE_MAX_S):
                time.sleep(0.02)
            svc.test_stick(0.0, 0.0, 0.0)
        events = []
        for axis, value, start, length in pulses:
            events.append((start, axis, value))
            events.append((start + length, axis, 0.0))
        events.sort()
        for at, axis, value in events:
            while not stop.is_set() and ut() - t0 < at:
                time.sleep(0.005)
            if stop.is_set():
                break
            if axis in stick:
                stick[axis] = value
                svc.test_stick(stick["pitch"], stick["roll"], stick["yaw"])
            else:
                setattr(c, axis, value)
        while not stop.is_set():
            time.sleep(0.1)
    finally:
        try:
            svc.test_stick(0.0, 0.0, 0.0)
            for axis in ("pitch", "roll", "yaw"):
                setattr(c, axis + "_trim", 0.0)
        except Exception:                               # noqa: BLE001
            pass
        conn.close()


FLY_RE = re.compile(
    r"\[\s*([\d.]+)\] FLY (\w+) spd=\s*([\d.]+) alt=\s*(-?[\d.]+).*?"
    r"att=\s*([-+\d.]+),\s*([-+\d.]+),\s*([\d.]+) "
    r"tgt=\s*([-+\d.]+),\s*([-+\d.]+),\s*([\d.]+) slip=\s*([-+\d.na]+) "
    r"rate=\s*([-+\d.]+),\s*([-+\d.]+),\s*([-+\d.]+) ")


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def takeoff(path, t0):
    """On the runway: the ground roll's peak bank, heading drift and pitch
    rate, and the lift-off (the first tick 2 m up that stays up 2 s; the
    FLY line's ``gnd=`` where it has one)."""
    rows = []
    for line in open(path):
        m = FLY_RE.search(line)
        if not m or float(m.group(1)) < t0:
            continue
        g = re.search(r" gnd=(\d)", line)
        rows.append((float(m.group(1)), float(m.group(3)), float(m.group(4)),
                     float(m.group(5)), float(m.group(6)), float(m.group(7)),
                     float(m.group(12)), None if g is None else g.group(1) == "1"))
    if len(rows) < 5:
        return "no FLY lines"
    alt0, psi0 = rows[0][2], rows[0][5]
    gnd = [r[7] if r[7] is not None else r[2] < alt0 + 2.0 for r in rows]
    lift = next((i for i in range(len(rows)) if not any(
        gnd[j] for j in range(i, len(rows)) if rows[j][0] - rows[i][0] <= 2.0)),
        None)
    ground = rows[:lift] if lift else rows
    out = "ground: bank max %.1f hdg drift max %.1f pitch-rate max %.1f" % (
        max(abs(r[3]) for r in ground),
        max(abs(wrap180(r[5] - psi0)) for r in ground),
        max(abs(r[6]) for r in ground))
    if lift is None:
        return out + "  NO LIFT-OFF (end spd %.0f)" % rows[-1][1]
    return out + "  lift-off t+%.1f %.0f m/s pitch %+.1f  " % (
        rows[lift][0] - t0, rows[lift][1], rows[lift][4])


def summary(path, t0, pulses):
    rows = []
    for line in open(path):
        m = FLY_RE.search(line)
        if m:
            rows.append([m.group(2)] + [float(x) if x not in ("nan",) else
                                        float("nan") for x in
                                        m.groups()[2:]] + [float(m.group(1))])
    if not rows or t0 is None:
        return "no FLY lines"
    busy = [(s - 0.0, s + ln + QUIET_S) for _, _, s, ln in pulses]

    # From 10 s before the first pulse (the take-off and climb-out, on the
    # runway, are not a hold being disturbed).
    start = max(2.0, min((s for _, _, s, _ in pulses), default=0.0) - 10.0)

    def quiet(t):
        return not any(a <= t - t0 < b for a, b in busy) and t - t0 > start

    out = []
    # axis: (name, attitude index, target index, rate index)
    # row: mode, spd, alt, phi, theta, psi, tphi, ttheta, tpsi, slip, q, p, r, ut
    for name, ai, ti, ri, heading in (("roll", 3, 6, 11, False),
                                      ("pitch", 4, 7, 10, False),
                                      ("yaw", 5, 8, 12, True)):
        stats = []
        for sel in (quiet, lambda t: t - t0 > start):
            rs = [r for r in rows if sel(r[-1])]
            if len(rs) < 3:
                stats.append("-")
                continue
            if name == "yaw" and rs[0][0] == "CRUISE":
                errs = [r[9] for r in rs if not math.isnan(r[9])]   # slip
            else:
                errs = [wrap180(r[ai] - r[ti]) if heading else r[ai] - r[ti]
                        for r in rs]
            rates = [r[ri] for r in rs]
            # Reversals: the rate's swings past REVERSAL_DEG_S each way.
            rev, ext, direction = 0, rates[0], 0
            for x in rates[1:]:
                if direction >= 0 and x < ext - REVERSAL_DEG_S:
                    rev += direction > 0
                    direction, ext = -1, x
                elif direction <= 0 and x > ext + REVERSAL_DEG_S:
                    rev += direction < 0
                    direction, ext = 1, x
                elif (direction > 0 and x > ext) or (direction < 0
                                                     and x < ext):
                    ext = x
                elif direction == 0:
                    ext = x
            span = max(1e-6, rs[-1][-1] - rs[0][-1])
            rms_e = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else float("nan")
            stats.append("err %.1f/%.1f rate %.1f rev %.2f/s" % (
                rms_e, max(abs(e) for e in errs) if errs else float("nan"),
                math.sqrt(sum(x * x for x in rates) / len(rates)), rev / span))
        out.append("%s[%s | all %s]" % (name, stats[0], stats[1]))
    last = rows[-1]
    out.append("end spd %.0f alt %.0f" % (last[1], last[2]))
    return "  ".join(out)


def launch_on_runway(instance, craft, rpc, stream):
    """The craft (``saves/craft/SPH``) on runway 09, settled, throttle shut,
    brakes on, paused."""
    sdir = os.path.join(paths.instance_dir(instance), "saves", "default")
    shutil.copy(os.path.join(ROOT, "saves", "craft", "SPH", craft + ".craft"),
                os.path.join(sdir, "Ships", "SPH", craft + ".craft"))
    conn = krpc.connect(name="airfly", rpc_port=rpc, stream_port=stream)
    try:
        conn.krpc.paused = False          # a run killed while paused
    except Exception:                                   # noqa: BLE001
        pass
    if conn.space_center.active_vessel is None:
        # In the space centre (a fresh instance) kRPC's launch has hung on
        # the farm: load a flight first.
        conn.close()
        conn = flighttest.load_paused(instance, "qs_warbird")
        conn.krpc.paused = False
    conn.space_center.launch_vessel("SPH", craft, "Runway", recover=True)
    time.sleep(8.0)
    c = conn.space_center.active_vessel.control
    c.throttle = 0.0
    c.brakes = True
    time.sleep(2.0)
    conn.krpc.paused = True
    return conn


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", default="0")
    p.add_argument("--save", default="qs_warbird")
    p.add_argument("--runway", action="store_true",
                   help="start on runway 09 instead of from --save: the "
                   "craft (--craft) is launched, settles, and on the unpause "
                   "goes to full throttle, brakes off; no pulses unless "
                   "--pulses is given; the summary adds the take-off")
    p.add_argument("--craft", default="Electric Warbird")
    p.add_argument("--rotate", type=float, default=None, metavar="M_S",
                   help="--runway: pull the stick back (as the player) "
                   "from this speed until the nose is %g deg up, then let go "
                   "(a nose-wheel plane needs it; the hold keeps the pitch "
                   "it is left at)" % ROTATE_PITCH_DEG)
    p.add_argument("--mode", type=int, default=0, help="0 CRUISE, 1 HOLD")
    p.add_argument("--seconds", type=float, default=60.0,
                   help="game seconds the pilot flies")
    p.add_argument("--pulses", default=DEFAULT_PULSES,
                   help="axis:trim:start:length,... (game s after unpause)")
    p.add_argument("--set", action="append", default=[])
    farmflight.add_arguments(p)
    args = p.parse_args()
    if args.runway and args.pulses == DEFAULT_PULSES:
        args.pulses = ""
    pulses = parse_pulses(args.pulses)
    rpc, stream = paths.ports(args.instance)
    if args.runway:
        conn = launch_on_runway(args.instance, args.craft, rpc, stream)
    else:
        conn = flighttest.load_paused(args.instance, args.save)
        conn.space_center.active_vessel.control.gear = False
    conn.close()
    farmflight.install_reaper()
    stop, started = threading.Event(), threading.Event()
    t0_box = []
    th = threading.Thread(target=disturb, args=(rpc, stream, pulses, started,
                                                stop, t0_box, 3.0,
                                                args.runway, args.rotate),
                          daemon=True)
    th.start()
    # The blades are flown by propPitch inside the pilot, as live.
    overrides = (["MODE=%d" % args.mode, "LOG_INTERVAL_UT=0.1",
                  "PROP_PLANE=True"]
                 + list(args.set))
    # The pilot's RUN_TIME_S counts from its takeover, made while paused.
    overrides.append("RUN_TIME_S=%g" % args.seconds)
    started.set()
    try:
        path, timed_out = farmflight.fly(args, "airplane",
                                         "airplane.autopilot", overrides,
                                         args.seconds * 4 + 120,
                                         ports=(rpc, stream))
    finally:
        stop.set()
        th.join(10.0)
    print("ksp%s %s %s %s: %s%s" % (
        args.instance, path, ("CRUISE", "HOLD")[args.mode],
        " ".join(args.set) or "(defaults)",
        (takeoff(path, t0_box[0]) if args.runway and t0_box else "")
        + summary(path, t0_box[0] if t0_box else None, pulses)
        if path and os.path.exists(path) else "no log",
        " TIMEOUT" if timed_out else ""))


if __name__ == "__main__":
    main()
