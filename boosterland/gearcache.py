"""How long each kind of landing leg takes to deploy, kept across flights.

Nothing in kRPC says how long a part's deploy animation runs (the length is
in the part's model, not its config), so the autopilot times the legs once
in flight and writes the answer here, keyed by part name.  A craft whose
legs are all in the file never has to cycle them again.

The file is regenerable -- delete it and the next flight re-times its legs
-- so it is git-ignored.  Farm instances share it; writes merge under a lock
and replace the file atomically.
"""

import fcntl
import json
import os
import sys

# Beside the package, or beside the script when run as a ``tools/bundle.py``
# single file, which has no ``__file__``.
PATH = os.path.join(
    os.path.dirname(os.path.abspath(globals().get("__file__") or sys.argv[0])),
    "data", "gearTimes.json")


def load(path=PATH):
    """{part name: {"deploy": s, "retract": s or None}}; empty if none."""
    try:
        with open(path) as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def lookup(names, path=PATH):
    """Each name's deploy time, or None if any of them is unknown."""
    known = load(path)
    times = {}
    for name in names:
        entry = known.get(name)
        deploy = entry.get("deploy") if isinstance(entry, dict) else None
        if not isinstance(deploy, (int, float)) or deploy <= 0.0:
            return None
        times[name] = float(deploy)
    return times


def record(entries, path=PATH):
    """Merge ``{name: {"deploy": s, "retract": s}}`` into the file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = load(path)
        data.update(entries)
        tmp = "%s.%d.tmp" % (path, os.getpid())
        with open(tmp, "w") as fh:
            json.dump(data, fh, indent=1, sort_keys=True)
        os.replace(tmp, path)
