#!/usr/bin/env python3
"""Fly several entry saves many times each, across the farm, and tabulate.

TEMPORARY TEST HARNESS -- not part of the flight software.

``quickglide.py`` flies one save on one instance.  A question of the shape
"how much surplus energy can the glide actually absorb?" is a *ladder*: six
or seven entry states that differ by one number, each flown often enough to
beat the glide's own 2.3 km repeatability, with the arms rotated across
instances so an instance effect cannot masquerade as a dose-response.  Doing
that by hand is what this replaces.

    ./spaceplane/tools/ladder.py --arms qs_l0,qs_l8,qs_l16 -n 4
    ./spaceplane/tools/ladder.py --arms qs_l0,qs_l32 -n 3 --instances 0,4 --timescale 6

Three properties it has to keep, each of which is a rule this project has
already paid for:

- **The arms rotate.**  Job ``k`` goes to instance ``k % len(instances)``
  and the jobs are ordered arm-major, so with a number of arms coprime to the
  number of instances every arm visits every instance.  Nine flights once
  came back with an instance effect suspected and nothing able to rule it
  out; CLAUDE.md's farm rule is that instances are only comparable if they
  are the same game, and the cheap insurance is not to need them to be.
- **It prints the table after every round, not at the end.**  A batch watched
  only at its end is a batch whose first bad flight is discovered an hour
  late, and the standard error is what says whether to keep flying.
- **Launches are staggered.**  ``quickglide.claim_log`` separates concurrent
  flights by when their logs were opened and reports ``?LOG1|LOG2`` when it
  cannot; a few seconds between launches keeps the attribution unambiguous.

Along-track is the number: a great-circle distance cannot tell an overshoot
from an undershoot and those want opposite corrections.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# quickglide.py's result line, of which only these fields matter here.
LINE = re.compile(r"along\s+([-+]\d+)\s+across\s+([-+]\d+)\s+"
                  r"([\d.]+) m/s\s+(\S+)\s+(\d+) parts")


class Result:
    def __init__(self, arm, instance, along, across, speed, situation, parts,
                 log):
        self.arm = arm
        self.instance = instance
        self.along = along
        self.across = across
        self.speed = speed
        self.situation = situation
        self.parts = parts
        self.log = log


def parse_arm(spec, default_save):
    """``name`` (a save) or ``label:K=V,K=V`` (a configuration).

    The rotation, the per-round table and the standard error are properties
    of the *harness*, not of what is being varied, and every one of them is a
    rule this project has already paid for.  A comparison between two
    configurations wants all three just as much as a comparison between two
    entry states does -- and doing it with ``armfly.sh`` instead pins each
    arm to one instance for the whole batch, which is the instance-by-arm
    confound the rotation exists to prevent.

    Returns ``(label, save, [settings])``.
    """
    if ":" not in spec:
        return spec, spec, []
    label, _, rest = spec.partition(":")
    settings = []
    save = default_save
    for item in rest.split(","):
        if not item:
            continue
        # ``save=NAME`` inside an arm picks that arm's entry state, so one
        # batch can cross a configuration with a save -- which is the shape
        # of "does this constant transfer?", and the question a batch that
        # can only vary one of them cannot ask.
        if item.startswith("save="):
            save = item.split("=", 1)[1]
            continue
        settings.append(item)
    return label, save, settings


def fly(arm, instance, args, lock):
    """One flight, as a subprocess, returning the parsed result or None."""
    label, save, settings = arm
    cmd = [os.path.join(ROOT, ".venv/bin/python"),
           os.path.join(ROOT, "spaceplane", "tools", "quickglide.py"),
           "--save", save, "--instance", str(instance), "-n", "1"]
    if args.timescale:
        cmd += ["--timescale", args.timescale]
    for setting in settings:
        cmd += ["--set", setting]
    for setting in args.set:
        cmd += ["--set", setting]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=args.timeout).stdout
    except subprocess.TimeoutExpired:
        out = ""
    match = LINE.search(out)
    log = ""
    m = re.search(r"(LOG\d+|\?LOG\S+)", out)
    if m:
        log = m.group(1)
    if not match:
        with lock:
            print("  %-12s ksp%s  NO RESULT: %s"
                  % (label, instance, out.strip().replace("\n", " ")[:120]),
                  flush=True)
        return None
    return Result(label, instance, float(match.group(1)), float(match.group(2)),
                  float(match.group(3)), match.group(4), int(match.group(5)),
                  log)


def stats(values):
    n = len(values)
    if n == 0:
        return None, None, None
    mean = sum(values) / n
    if n < 2:
        return mean, None, None
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))
    return mean, sd, sd / math.sqrt(n)


def table(arms, results):
    """One line per arm, in the order the arms were given."""
    lines = ["  %-12s %3s  %8s %7s %7s   %s"
             % ("arm", "n", "along", "sd", "se", "flights")]
    for arm in arms:
        label = arm[0] if isinstance(arm, tuple) else arm
        got = [r for r in results if r.arm == label]
        mean, sd, se = stats([r.along / 1000.0 for r in got])
        if mean is None:
            lines.append("  %-12s %3d" % (label, 0))
            continue
        # A vehicle that came apart still measured its along-track, but say
        # so: the arrival and the survival are different questions and this
        # harness only answers the first.
        broke = sum(1 for r in got if r.parts == 0)
        lines.append("  %-12s %3d  %+8.1f %7s %7s   %s%s"
                     % (label, len(got), mean,
                        "%.1f" % sd if sd is not None else "-",
                        "%.1f" % se if se is not None else "-",
                        " ".join("%+.1f" % (r.along / 1000.0) for r in got),
                        "   (%d broke up)" % broke if broke else ""))
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--arms", required=True,
                   help="comma-separated arms: a save name, or "
                        "'label:FIELD=V,FIELD=V' for a configuration arm "
                        "flown on --save.  **If any arm has settings, "
                        "separate the arms with ';'** -- a comma inside "
                        "'af:save=X,FIELD=V' otherwise splits it into two "
                        "arms, the second a save that does not exist")
    p.add_argument("--save", default="qs_plane",
                   help="the save configuration arms are flown from")
    p.add_argument("--instances", default="0,1,4,5")
    p.add_argument("-n", type=int, default=3, help="flights per arm")
    p.add_argument("--timescale", default="8.0")
    p.add_argument("--timeout", type=float, default=1800.0)
    p.add_argument("--set", action="append", default=[])
    p.add_argument("--stagger", type=float, default=8.0)
    args = p.parse_args()

    # Configuration arms carry commas inside them, so split on ';' when any
    # arm is one.  A bare list of saves keeps the original comma form.
    raw = args.arms.split(";") if ";" in args.arms else args.arms.split(",")
    arms = [parse_arm(a.strip(), args.save) for a in raw if a.strip()]
    instances = [int(i) for i in args.instances.split(",") if i != ""]

    # Arm-major so that with coprime counts every arm visits every instance.
    jobs = [(arm, rep) for rep in range(args.n) for arm in arms]
    results = []
    lock = threading.Lock()
    started = time.time()

    print("%d arms x %d flights on %d instances, %d flights total"
          % (len(arms), args.n, len(instances), len(jobs)), flush=True)
    for label, save, settings in arms:
        print("  arm %-12s save=%-10s %s"
              % (label, save, " ".join(settings) or "(as given)"), flush=True)

    # **The assignment is fixed up front, not raced for.**  A pool that hands
    # out the next job to whichever instance finishes first would let a slow
    # instance fly fewer flights of every arm -- which is exactly the
    # instance-by-arm confound the rotation exists to prevent.  Job ``k``
    # belongs to instance ``k % len(instances)`` and to no other.
    mine = {slot: [job for k, job in enumerate(jobs)
                   if k % len(instances) == slot]
            for slot in range(len(instances))}

    def worker(slot, instance):
        for arm, rep in mine[slot]:
            result = fly(arm, instance, args, lock)
            with lock:
                if result is not None:
                    results.append(result)
                    print("  %-12s ksp%-2s rep%d  along %+8.0f  across %+7.0f"
                          "  %5.1f m/s  %-24s %2d parts  %s"
                          % (arm[0], instance, rep + 1, result.along,
                             result.across, result.speed, result.situation,
                             result.parts, result.log), flush=True)
                done = len(results)
                if done and done % len(instances) == 0:
                    print("\n  -- after %d flights, %.0f min --"
                          % (done, (time.time() - started) / 60.0))
                    print(table(arms, results), flush=True)
                    print("", flush=True)

    threads = [threading.Thread(target=worker, args=(slot, instance))
               for slot, instance in enumerate(instances)]
    for t in threads:
        t.start()
        time.sleep(args.stagger)
    for t in threads:
        t.join()

    print("\n== %d flights, %.0f min ==" % (len(results),
                                            (time.time() - started) / 60.0))
    print(table(arms, results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
