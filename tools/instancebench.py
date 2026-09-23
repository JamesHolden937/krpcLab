#!/usr/bin/env python3
"""Find how fast a test instance will actually fly, and what is stopping it.

TEMPORARY TEST HARNESS -- not part of the flight software.

In-game measurement is this project's bottleneck, so the speed of an instance
is worth measuring properly once rather than guessing at repeatedly.  The
timescale plugin holds

    timeScale = min(what was asked for, quant_s * fps)

so there are two different ceilings and they want opposite fixes:

* **Quantization.**  Unity applies control input once a frame, so
  ``timeScale / fps`` is the grid every autopilot command lands on.  If the
  achieved multiplier sits at ``quant_s * fps`` the instance is frame-starved
  and the fix is more frames (or a coarser quantum, if the vehicle's control
  interval can take one).
* **CPU.**  Physics is single-threaded and runs ``timeScale / fixed_dt`` steps
  a second.  If the achieved multiplier sits below *both* the request and
  ``quant_s * fps``, the main thread is the limit and more frames will make it
  worse, because frames and physics share the core.

Telling those apart from inside the game is impossible -- both look like "it
is going at 4x" -- so this reports which one is binding, per setting.

    ./tools/instancebench.py 5                          # what is it doing now
    ./tools/instancebench.py 5 --quant 0.05,0.1,0.2,0.4
    ./tools/instancebench.py 5 --quant 0.2 --adaptive --max 32
    ./tools/instancebench.py 5 --save qs_plane --dwell 25

It needs a vessel in the flight scene, because the plugin is a
``KSPAddon.Startup.Flight`` addon and writes nothing without one.  Pass
``--save`` to load one; without it the instance is assumed to be in flight
already.
"""
import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

from common import timescale as ts                                  # noqa: E402


def ports(instance):
    base = os.path.join(ROOT, "testInstances", "ksp" + str(instance))
    return (int(open(os.path.join(base, ".rpc_port")).read()),
            int(open(os.path.join(base, ".stream_port")).read()))


def load(instance, save):
    import krpc
    rpc, stream = ports(instance)
    for _ in range(40):
        try:
            conn = krpc.connect(name="instancebench", rpc_port=rpc,
                                stream_port=stream)
            break
        except Exception:                               # noqa: BLE001
            time.sleep(3.0)
    else:
        raise SystemExit("could not connect to ksp%s" % instance)
    try:
        conn.space_center.load(save)
    finally:
        conn.close()
    time.sleep(8.0)


def sample(tsdir, dwell):
    """Let it settle, then take the plugin's own windowed report.

    ``dwell`` matters: adaptive mode walks the multiplier by 15% a second and
    the frame rate takes a moment to follow the physics load, so a reading
    taken immediately is a reading of the transient.
    """
    time.sleep(dwell)
    best = None
    for _ in range(5):                  # a few windows, take the median-ish
        st = ts.status(tsdir)
        if st:
            best = st if best is None else (
                st if st["achieved"] > best["achieved"] else best)
        time.sleep(1.2)
    return best


def verdict(st, quant, ceiling=None):
    """Which ceiling is binding.  The whole point of the tool.

    The discriminator is **physics steps per real second**, not the
    multiplier: that is the quantity the single-threaded main thread is
    actually spending, and it is what plateaus when the CPU is the limit.
    Comparing ``achieved`` against ``quant_s * fps`` cannot tell them apart,
    because past the CPU ceiling the frame rate collapses and drags the
    apparent allowance down with it -- so a CPU-bound instance reports
    ``achieved`` far *above* its own computed allowance and reads as
    quantization-bound.  That is what this function said first, on every row
    that mattered.
    """
    if not st:
        return "no status -- plugin not loaded?"
    steps = st["achieved"] / max(st["fixed_dt"], 1e-6)
    grid = st["quant_s"]
    # The grid the vehicle is actually flown on, which is not necessarily the
    # one that was asked for: past the CPU ceiling the game misses the
    # maximumDeltaTime clamp and the effective quantum runs away.
    slipped = "  [grid %.3fs, %.1fx asked]" % (grid, grid / quant) \
        if grid > quant * 1.25 else ""
    if ceiling and steps >= ceiling * 0.93:
        return "CPU (%.0f steps/s) -- more frames would hurt%s" % (steps, slipped)
    if st["achieved"] >= quant * st["fps"] * 0.92:
        return "QUANTIZATION (%.1f = %.3f x %.0f fps) -- more frames would help%s" \
            % (quant * st["fps"], quant, st["fps"], slipped)
    return "REQUEST -- neither ceiling is binding; ask for more%s" % slipped


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance")
    p.add_argument("--quant", default=None,
                   help="comma-separated control quanta to try, seconds")
    p.add_argument("--adaptive", action="store_true",
                   help="let the plugin walk the multiplier up itself")
    p.add_argument("--fixed", type=float, default=None,
                   help="hold this multiplier instead of walking")
    p.add_argument("--max", type=float, default=32.0)
    p.add_argument("--dwell", type=float, default=25.0)
    p.add_argument("--save", default=None)
    p.add_argument("--restore", action="store_true",
                   help="put the instance back to 1x when finished")
    a = p.parse_args()

    tsdir = ts.instance_dir(a.instance)
    if not os.path.isdir(tsdir):
        raise SystemExit("no such instance: %s" % tsdir)
    if a.save:
        load(a.instance, a.save)

    if a.quant is None:
        st = ts.status(tsdir)
        print(ts.describe(st))
        print("  limit: %s" % verdict(st, st["quant_s"] if st else 0.05))
        return 0

    spec = ("%.4f" % a.fixed) if a.fixed else ("max" if a.adaptive else "max")
    # Two passes: the first finds the machine's own physics ceiling (the
    # steps/s that stops rising however coarse the quantum gets), the second
    # reports each row against it.  Without the ceiling there is nothing to
    # call "CPU-bound" relative to.
    rows = []
    for q in [float(x) for x in a.quant.split(",")]:
        ts.write(tsdir, spec, quant_s=q, max_scale=a.max)
        st = sample(tsdir, a.dwell)
        rows.append((q, st))
    ceiling = max((st["achieved"] / max(st["fixed_dt"], 1e-6)
                   for _, st in rows if st), default=None)

    print("%-8s %9s %9s %7s %9s %7s  %s"
          % ("quant_s", "commanded", "achieved", "fps", "steps/s", "grid",
             "binding on"))
    for q, st in rows:
        if not st:
            print("%-8.3f  no status" % q)
            continue
        steps = st["achieved"] / max(st["fixed_dt"], 1e-6)
        print("%-8.3f %9.2f %9.2f %7.0f %9.0f %7.3f  %s"
              % (q, st["commanded"], st["achieved"], st["fps"], steps,
                 st["quant_s"], verdict(st, q, ceiling)))
    if a.restore:
        ts.write(tsdir, "off")
        print("restored to 1x")
    return 0


if __name__ == "__main__":
    sys.exit(main())
