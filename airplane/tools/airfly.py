#!/usr/bin/env python3
"""Fly the airplane pilot on a farm instance from a save of a plane in the
air, disturb it, and say how well it held.

    ./airplane/tools/airfly.py --instance 0                  # CRUISE, qs_warbird
    ./airplane/tools/airfly.py --instance 1 --mode 1 --set RATE_TAU_TICKS=6

The save (``airstart.py``: the Electric Warbird level at 100 m, 200 m/s,
full throttle) is loaded paused, the pilot is started with ``--autostart``
and ``PROP_PLANE`` (propPitch flies the blades, in-process, as live)
and takes the attitude it finds; then the game runs and the harness puts
**trim pulses** on each axis in turn (``--pulses``: ``axis:value:start:
length`` in game seconds after the unpause).  KSP adds the trim to the
stick and the pilot takes it out of what it reads as the player's input
(``control.stick``), so a trim is a disturbance the hold cannot see and
must reject -- a gust or a change of trim -- and its release is a step back
to the held attitude.  (A second kRPC client cannot play the stick: kRPC
keeps one set of inputs per vessel, ``PilotAddon.manualInputs``, so its
writes would replace the pilot's.)

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

DEFAULT_PULSES = ("roll:0.3:8:3,pitch:0.15:18:3,yaw:0.3:28:3,"
                  "roll:-0.5:38:1.5,pitch:-0.15:46:1.5")
QUIET_S = 1.0
REVERSAL_DEG_S = 2.0


def parse_pulses(text):
    out = []
    for item in text.split(","):
        if not item.strip():
            continue
        axis, value, start, length = item.split(":")
        out.append((axis, float(value), float(start), float(length)))
    return out


def disturb(rpc, stream, pulses, started, stop, t0_box, lead_s):
    """Unpause once the pilot has started, then put the trim pulses on."""
    conn = krpc.connect(name="airfly-disturb", rpc_port=rpc,
                        stream_port=stream)
    try:
        c = conn.space_center.active_vessel.control
        ut = conn.add_stream(getattr, conn.space_center, "ut")
        started.wait(60.0)
        time.sleep(lead_s)
        conn.krpc.paused = False
        t0 = ut()
        t0_box.append(t0)
        events = []
        for axis, value, start, length in pulses:
            events.append((start, axis, value))
            events.append((start + length, axis, 0.0))
        events.sort()
        for at, axis, value in events:
            while not stop.is_set() and ut() - t0 < at:
                time.sleep(0.01)
            if stop.is_set():
                break
            setattr(c, axis + "_trim", value)
        while not stop.is_set():
            time.sleep(0.1)
        for axis in ("pitch", "roll", "yaw"):
            setattr(c, axis + "_trim", 0.0)
    finally:
        conn.close()


FLY_RE = re.compile(
    r"\[\s*([\d.]+)\] FLY (\w+) spd=\s*([\d.]+) alt=\s*(-?[\d.]+).*?"
    r"att=\s*([-+\d.]+),\s*([-+\d.]+),\s*([\d.]+) "
    r"tgt=\s*([-+\d.]+),\s*([-+\d.]+),\s*([\d.]+) slip=\s*([-+\d.na]+) "
    r"rate=\s*([-+\d.]+),\s*([-+\d.]+),\s*([-+\d.]+) ")


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


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

    def quiet(t):
        return not any(a <= t - t0 < b for a, b in busy) and t - t0 > 2.0

    out = []
    # axis: (name, attitude index, target index, rate index)
    # row: mode, spd, alt, phi, theta, psi, tphi, ttheta, tpsi, slip, q, p, r, ut
    for name, ai, ti, ri, heading in (("roll", 3, 6, 11, False),
                                      ("pitch", 4, 7, 10, False),
                                      ("yaw", 5, 8, 12, True)):
        stats = []
        for sel in (quiet, lambda t: t - t0 > 2.0):
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


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", default="0")
    p.add_argument("--save", default="qs_warbird")
    p.add_argument("--mode", type=int, default=0, help="0 CRUISE, 1 HOLD")
    p.add_argument("--seconds", type=float, default=56.0,
                   help="game seconds the pilot flies")
    p.add_argument("--pulses", default=DEFAULT_PULSES,
                   help="axis:trim:start:length,... (game s after unpause)")
    p.add_argument("--set", action="append", default=[])
    farmflight.add_arguments(p)
    args = p.parse_args()
    pulses = parse_pulses(args.pulses)
    rpc, stream = paths.ports(args.instance)
    conn = flighttest.load_paused(args.instance, args.save)
    conn.space_center.active_vessel.control.gear = False
    conn.close()
    farmflight.install_reaper()
    stop, started = threading.Event(), threading.Event()
    t0_box = []
    th = threading.Thread(target=disturb, args=(rpc, stream, pulses, started,
                                                stop, t0_box, 3.0),
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
        summary(path, t0_box[0] if t0_box else None, pulses)
        if path and os.path.exists(path) else "no log",
        " TIMEOUT" if timed_out else ""))


if __name__ == "__main__":
    main()
