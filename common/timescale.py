#!/usr/bin/env python3
"""Set and read back the in-game time scale of a measurement instance.

TEMPORARY TEST HARNESS -- not part of the flight software.

The plugin in ``testInstances/timescaleSrc`` runs the flight faster than
real time without touching ``Time.fixedDeltaTime``, so every physics step is
the step the vehicle flies at 1x.  It is driven by a file at the instance
root, and this is the other end of that file.

    ./timescale.py 0 2.0        # instance ksp0 at 2x
    ./timescale.py 0 max        # as fast as its main thread will go
    ./timescale.py 0 off        # back to 1x
    ./timescale.py 0            # what is it actually achieving?

The distinction that matters is **commanded** versus **achieved**.  KSP's
physics is single-threaded, so asking for 4x on a machine that can sustain 1.6x
gets you 1.6x -- and from inside the game the two are indistinguishable, which
is why the plugin measures game seconds against real seconds and writes the
answer back out.
"""
import argparse
import os
import time
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The largest control quantum, in game-seconds, that a command may land on.
# Unity applies control input once a frame, so the autopilot's commands sit on
# a grid of `timeScale / fps` -- push the time scale up without pushing the
# frame rate up and the physics stays faithful while the controller flying it
# does not.  0.05 s is the booster's own command interval; letting the grid
# grow past it means commands start being merged.
DEFAULT_QUANT_S = 0.05


def instance_dir(instance=None, rpc_port=None):
    """Where the instance's control file lives.

    Ports are `50100 + 2N`, so the instance number can be recovered from the
    port when nobody passed one -- fly.sh passes ports, not instance numbers.
    """
    if instance is not None:
        return os.path.join(ROOT, "testInstances", "ksp" + str(instance))
    if rpc_port is not None and rpc_port >= 50100:
        return os.path.join(ROOT, "testInstances",
                            "ksp" + str((rpc_port - 50100) // 2))
    return os.path.expanduser("~/Kerbal Space Program")


def write(dirpath, spec, quant_s=DEFAULT_QUANT_S, max_scale=8.0):
    """Point the instance at ``spec``: "off", "max", or a multiplier.

    Returns the commanded scale as a float, 1.0 for off.
    """
    spec = str(spec).strip().lower()
    if spec in ("", "off", "1", "1.0", "none"):
        mode, scale = "off", 1.0
    elif spec in ("max", "adaptive", "auto"):
        mode, scale = "adaptive", 1.0
    else:
        mode, scale = "fixed", float(spec)
    path = os.path.join(dirpath, "timescale.txt")
    with open(path, "w") as fh:
        fh.write("mode = %s\nscale = %.4f\nmax_scale = %.4f\nquant_s = %.4f\n"
                 % (mode, scale, max_scale, quant_s))
    return scale if mode == "fixed" else (max_scale if mode == "adaptive" else 1.0)


# How old a status file may be and still describe the present.  The plugin
# rewrites it about once a second while a vessel is in the scene, so anything
# older than this is from a previous life.
STALE_S = 10.0


def status(dirpath, stale_s=STALE_S):
    """What the instance is actually doing, or ``None`` if it has not said.

    ``None`` rather than a default: a flight that silently ran at 1x when it
    was asked for 3x, and reported 3x because that is what was written, is a
    measurement that lies in the direction you were hoping for.

    **And the file outlives the process that wrote it.**  It is an ordinary
    file in the instance directory, so a stopped, restarted or still-loading
    instance keeps answering with whatever it last said -- in a concurrent
    benchmark one instance reported 9.80x while it was in fact still on the
    loading screen, and the number was three minutes old and from a different
    process.  Freshness is the only thing separating "this is what it is
    doing" from "this is what it once did", so a stale file is no answer at
    all.
    """
    path = os.path.join(dirpath, "timescale-status.txt")
    if not os.path.exists(path):
        return None
    try:
        if time.time() - os.path.getmtime(path) > stale_s:
            return None
    except OSError:
        return None
    out = {}
    try:
        for line in open(path):
            if "=" in line:
                k, v = line.split("=", 1)
                k = k.strip()
                v = v.strip()
                out[k] = v if k == "mode" else float(v)
    except (ValueError, OSError):
        return None
    return out or None


def describe(st):
    if not st:
        return "timescale: no status (plugin not loaded, or never flew)"
    return ("timescale: commanded %.2fx achieved %.2fx  fps %.0f  "
            "quantum %.3fs  fixed_dt %.4f"
            % (st.get("commanded", 0.0), st.get("achieved", 0.0),
               st.get("fps", 0.0), st.get("quant_s", 0.0),
               st.get("fixed_dt", 0.0)))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("instance", help="instance number, or a path")
    p.add_argument("spec", nargs="?", default=None,
                   help="off | max | a multiplier such as 2.0")
    p.add_argument("--quant-s", type=float, default=DEFAULT_QUANT_S)
    p.add_argument("--max-scale", type=float, default=8.0)
    a = p.parse_args(argv)

    d = a.instance if os.path.isdir(a.instance) else instance_dir(a.instance)
    if not os.path.isdir(d):
        sys.stderr.write("no such instance: %s\n" % d)
        return 1
    if a.spec is not None:
        write(d, a.spec, a.quant_s, a.max_scale)
    sys.stdout.write(describe(status(d)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
