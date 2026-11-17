"""Each booster's own drag as boosterland measured it, kept across flights.

The ascent has to know what the booster's return will cost before the
booster has flown, and the only drag it can read then is the whole stack's
at zero angle of attack (``Flight.simulate_aerodynamic_force_at`` takes a
vessel, not a part): orbiter included, nose first, 2-3 times the booster's
own Cd*A falling engine first (fin booster, bp1-1116: 3.3 at M2.4 against
the stack's 12-13).  Fed that, the ascent's budget fell too slowly, lit the
landing burn too slow and charged it too little.  So boosterland writes the
Cd*A-against-Mach curve it measured in flight here, keyed by the booster's
engines and tanks, and the ascent of a stack carrying the same booster
reads it back.

Regenerable (delete it and the next booster flight writes it again), so
git-ignored.  Farm instances share it; writes merge under a lock and replace
the file atomically, as ``gearcache``.
"""

import fcntl
import hashlib
import json
import os

from common import paths

PATH = os.path.join(paths.ROOT, "boosterland", "data", "dragCurves.json")
PROPELLANTS = ("LiquidFuel", "Oxidizer")


def booster_key(parts):
    """A booster's identity from its kRPC parts (or a function returning
    them): the names of the parts that
    carry an engine or main propellant, as a multiset.  The decoupler and
    whatever stays on the other side of it do not count, so the ascent (the
    parts its staging drops) and boosterland (the vessel after separation)
    agree.  None when nothing qualifies or the parts cannot be read."""
    names = []
    try:
        if callable(parts):
            parts = parts()
        for p in parts:
            if p.engine is not None or any(
                    r.name in PROPELLANTS for r in p.resources.all):
                names.append(p.name)
    except Exception:                                   # noqa: BLE001
        return None
    if not names:
        return None
    return hashlib.sha1("|".join(sorted(names)).encode()).hexdigest()[:16]


def load(path=PATH):
    try:
        with open(path) as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def lookup(key, path=PATH):
    """``(machs, areas)`` for the booster, or None if it has not flown."""
    entry = load(path).get(key) if key else None
    if not isinstance(entry, dict):
        return None
    machs, areas = entry.get("machs"), entry.get("areas")
    if (not isinstance(machs, list) or not isinstance(areas, list)
            or len(machs) != len(areas) or len(machs) < 2):
        return None
    return [float(m) for m in machs], [float(a) for a in areas]


def record(key, machs, areas, note="", path=PATH):
    """Replace the booster's curve with this flight's."""
    if not key or len(machs) < 2:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = load(path)
        data[key] = {"machs": [round(m, 4) for m in machs],
                     "areas": [round(a, 4) for a in areas], "note": note}
        tmp = "%s.%d.tmp" % (path, os.getpid())
        with open(tmp, "w") as fh:
            json.dump(data, fh, indent=1, sort_keys=True)
        os.replace(tmp, path)


def interp(machs, areas, mach):
    """The curve at ``mach``, its end values held (as the Environment's)."""
    if mach <= machs[0]:
        return areas[0]
    if mach >= machs[-1]:
        return areas[-1]
    for i in range(1, len(machs)):
        if mach <= machs[i]:
            lo, hi = machs[i - 1], machs[i]
            f = (mach - lo) / (hi - lo) if hi > lo else 1.0
            return areas[i - 1] * (1.0 - f) + areas[i] * f
    return areas[-1]
