"""Run one autopilot flight as a child process, the way the farm harnesses do.

Shared by ``spaceplane/tools/quickglide.py``, ``boosterland/tools/quickfly.py``
and ``ascent/tools/stackfly.py``: the command (the single-file bundle unless
``tree``), the launch in its own process group, the reaper that makes sure no
flight outlives its harness, and which ``logs/LOG<n>`` the flight wrote.
"""
import os
import re
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, "logs")

_LIVE = []


def add_arguments(parser):
    """``--bundle``, ``--tree``, ``--pypy``, ``--cpython``."""
    parser.add_argument("--bundle", default=None, metavar="FILE",
                        help="fly this bundle (default: dist/krpcLab.py, "
                             "rebuilt if stale)")
    parser.add_argument("--tree", action="store_true",
                        help="fly the modules in the tree, not the bundle")
    parser.add_argument("--pypy", action="store_true",
                        help="run the autopilot under PyPy (.venv-pypy; the "
                             "default)")
    parser.add_argument("--cpython", action="store_true",
                        help="run the autopilot under CPython (.venv) instead")


def command(args, pilot, module, overrides):
    """The argv that flies ``pilot`` (``module`` under ``--tree``), with
    ``--autostart`` and ``overrides`` as ``--set``s.

    **PyPy is the default** wherever ``.venv-pypy`` exists (2026-10-02): the
    farm's numbers are measured with it; ``--cpython`` opts out.  **The
    single-file bundle flies, as everywhere** (``tools/bundle.py``, rebuilt
    first if a source changed); ``--tree`` flies the modules."""
    pypy = (getattr(args, "pypy", False)
            or not getattr(args, "cpython", False)) and os.path.exists(
        os.path.join(ROOT, ".venv-pypy", "bin", "python"))
    python = os.path.join(ROOT, ".venv-pypy" if pypy else ".venv", "bin",
                          "python")
    overrides = list(overrides)
    if getattr(args, "tree", False):
        argv = [python, "-m", module]
    else:
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import bundle
        path = (os.path.abspath(args.bundle) if getattr(args, "bundle", None)
                else bundle.ensure())
        argv = [python, path, "--pilot", pilot]
        if not any(o.startswith("LOG_DIR") for o in overrides):
            overrides.append("LOG_DIR=%s" % LOGS)
    argv.append("--autostart")
    for item in overrides:
        argv += ["--set", item]
    return argv


def install_reaper():
    """Reap the flight on SIGTERM and SIGINT, not just on the normal path.

    A ``finally`` is not enough: Python's default SIGTERM handling exits
    without unwinding, so ``pkill -f quickglide.py`` left the autopilot --
    a *child* process -- flying.  Measured, two abandoned batches left eight
    of them alive on four instances.  See the note at ``launch``.
    """
    def handler(signum, frame):                         # noqa: ARG001
        for process in list(_LIVE):
            reap(process)
        raise SystemExit(128 + signum)
    for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        try:
            signal.signal(signum, handler)
        except Exception:                               # noqa: BLE001
            pass


def reap(process):
    """Make sure no autopilot outlives the harness that started it.

    Signals the whole process group, because the flight is a child and an
    orphaned child keeps flying the vessel -- see the note at ``launch``.
    Idempotent and silent: called on the normal path as well as on the
    timeout, and on the way out of an interrupt.
    """
    if process.poll() is not None:
        return
    for signaller, wait in ((signal.SIGTERM, 20.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(process.pid), signaller)
        except Exception:                               # noqa: BLE001
            try:
                process.kill()
            except Exception:                           # noqa: BLE001
                pass
        try:
            process.wait(timeout=wait)
            return
        except Exception:                               # noqa: BLE001
            continue


def launch(argv):
    """Start the flight; returns the process.

    **Not DEVNULL.**  Nothing prints in normal operation (CLAUDE.md), so the
    only thing that ever reaches stdout and stderr is a crash before the
    logbook opened or a traceback out of the control loop -- and discarding
    them turns "the autopilot died at 2.5 km" into a log that simply stops
    with no shutdown line.  ``run.sh`` appends to the same file.

    **Its own process group, so killing the harness kills the flight.**  The
    autopilot is a *child*, and ``pkill -f quickglide.py`` matches only the
    parent: the child goes on flying, still connected, still commanding the
    vessel.  Measured on 2026-09-16, two killed batches left **eight**
    autopilots alive on four instances -- three of them on one port --
    fighting each other over one aircraft.  What that produces is not a
    crash, it is plausible telemetry: a batch started afterwards read +30,
    +42 and +61 km against a +2.7 km baseline and looked exactly like a
    regression in the change under test.
    """
    errors = open(os.path.join(LOGS, "stderr.log"), "a")
    try:
        process = subprocess.Popen(argv, cwd=ROOT, stdout=errors,
                                   stderr=subprocess.STDOUT,
                                   start_new_session=True)
        _LIVE.append(process)
    finally:
        errors.close()
    return process


def wait(process, timeout, poll=2.0):
    """Wait for the flight to end or ``timeout`` wall seconds; reap it
    either way.  True if it timed out."""
    started = time.time()
    try:
        while process.poll() is None and time.time() - started < timeout:
            time.sleep(poll)
    finally:
        timed_out = process.poll() is None
        reap(process)
        if process in _LIVE:
            _LIVE.remove(process)
    return timed_out


def log_names():
    return set(n for n in os.listdir(LOGS)
               if n.startswith("LOG") and n[3:].isdigit())


def claim_log(before, launched):
    """Which log this flight wrote, given the ones that existed before it.

    **"The highest-numbered log" is wrong the moment two instances fly at
    once**, which is the only way a farm harness is worth running: it returns
    whichever instance happened to open a log last, so a sweep attributes one
    instance's trace to another instance's result.  ``Logbook`` itself claims
    its number atomically (``O_CREAT | O_EXCL``) and is not the problem --
    the number is claimed correctly and then read back by the wrong rule.

    So diff against a snapshot taken before the launch, and where several
    flights started together, take the one whose file appeared soonest after
    *this* subprocess did.  If that still cannot separate them, say so with
    the candidates rather than name one: a sweep that quietly cites the wrong
    log is worse than one that admits it does not know which.
    """
    fresh = sorted(log_names() - before, key=lambda n: int(n[3:]))
    if not fresh:
        return None
    if len(fresh) == 1:
        return os.path.join(LOGS, fresh[0])
    timed = sorted((abs(opened_at(os.path.join(LOGS, n)) - launched), n)
                   for n in fresh)
    if len(timed) > 1 and timed[1][0] - timed[0][0] < 1.0:
        return "?" + "|".join(n for _, n in timed[:3])
    return os.path.join(LOGS, timed[0][1])


def opened_at(path):
    """When the log was opened, from its own first line.

    **Not the mtime**: a log still being written has an mtime of *now*, so
    every candidate looked equally recent and the tiebreak reported an
    ambiguity that was not there.  The header line is written once and never
    moves.
    """
    try:
        with open(path, errors="replace") as fh:
            m = re.search(r"opened (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)",
                          fh.readline())
        if m:
            return time.mktime(time.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
    except OSError:
        pass
    return float("inf")


def fly(args, pilot, module, overrides, timeout, poll=2.0, ports=None,
        extra=()):
    """Launch, wait, reap: ``(log path or None, timed out)``.  ``ports`` is
    ``(rpc, stream)`` or None for the default game; ``extra``, more of the
    pilot's own arguments."""
    argv = command(args, pilot, module, overrides) + list(extra)
    if ports and ports[0]:
        argv += ["--rpc-port", str(ports[0]), "--stream-port", str(ports[1])]
    address = getattr(args, "address", None)
    if address:
        argv += ["--address", address]
    before = log_names()
    launched = time.time()
    timed_out = wait(launch(argv), timeout, poll)
    return claim_log(before, launched), timed_out
