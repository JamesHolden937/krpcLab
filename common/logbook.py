"""File logging, paced by in-game time.

Nothing in this project writes to stdout/stderr on purpose: the only output is
``logs/LOG<n>``, a fresh numbered file per run.  Periodic telemetry lines are
gated on universal time (``SpaceCenter.ut``), so a paused game logs nothing and
a time-warped game still logs every ``LOG_INTERVAL_UT`` *in-game* seconds.
"""

import os
import re
import time


class Logbook:
    def __init__(self, directory, interval_ut):
        self.interval_ut = float(interval_ut)
        self.path, self._fh = _open_next_log(directory)
        self._next_ut = None
        self._fh.write("# boosterland log, opened %s\n"
                       % time.strftime("%Y-%m-%d %H:%M:%S"))

    # -- writing -----------------------------------------------------------
    def event(self, ut, text):
        """Always written, regardless of the interval gate."""
        self._fh.write("[%10.2f] %s\n" % (ut, text))

    def telemetry(self, ut, text):
        """Written only once per ``interval_ut`` in-game seconds."""
        if self._next_ut is None or ut >= self._next_ut:
            self._fh.write("[%10.2f] %s\n" % (ut, text))
            # Re-anchor rather than accumulate, so a time warp that skips a
            # whole interval produces one line instead of a burst.
            self._next_ut = ut + self.interval_ut
            return True
        return False

    def close(self):
        if not self._fh.closed:
            self._fh.write("# closed\n")
            self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _next_log_path(directory):
    """The next free ``LOG<n>`` name.  Racy on its own -- see `_open_next_log`."""
    os.makedirs(directory, exist_ok=True)
    highest = 0
    for name in os.listdir(directory):
        m = re.fullmatch(r"LOG(\d+)", name)
        if m:
            highest = max(highest, int(m.group(1)))
    return os.path.join(directory, "LOG%d" % (highest + 1))


def _open_next_log(directory):
    """Claim the next ``LOG<n>`` atomically, and return it open.

    Scanning for ``highest + 1`` and then opening it is a race, and the
    parallel test rig runs four flights into *one* ``logs/`` directory: two
    that start together pick the same number, the second truncates the
    first, and what is left is one file holding an interleaving of two
    flights.  It is a silent corruption -- the file looks like a log -- and
    it was found only because a sweep reported the same LOG number for two
    different configurations.

    ``O_CREAT | O_EXCL`` makes the claim the same operation as the choice, so
    the loser of the race sees EEXIST and takes the next number instead.
    """
    for _ in range(1000):
        path = _next_log_path(directory)
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            continue            # somebody else claimed it; take the next one
        return path, os.fdopen(fd, "w", buffering=1)
    raise RuntimeError("could not claim a log name in %s" % directory)
