"""The highest touchdown speed a craft survives, from its own joints.

A part's impact tolerance is an upper bound only: the stack above the parts
that touch has to be stopped through the joints between them, and a joint
breaks when PhysX's constraint force passes its ``breakForce`` -- often well
before any part reaches its crash tolerance.  ``launcherMod``'s
``KrpcLabJoints.Parts`` reads the live values (masses, crash tolerances,
each part's attach joint: break force, spring, damper); this models the
contact as a chain along the craft's axis -- every part a mass, every joint a
spring and damper to its parent, the touching parts stopped dead by the
ground -- integrates it, and bisects the contact speed for the highest one at
which no joint and no touching part fails.

PhysX does not solve joints as ideal springs (iteration count, projection),
so this is an estimate; every landing's outcome is the calibration.  Units
are KSP's physics units: t, m, s, kN.
"""

import math

INF = float("inf")


def parse_parts(rows):
    """``KrpcLabJoints.Parts`` lines -> list of dicts."""
    out = []
    for row in rows:
        f = row.split("|")
        if len(f) < 14:
            continue
        num = lambda s: INF if s == "inf" else float(s)        # noqa: E731
        out.append({"index": int(f[0]), "parent": int(f[1]), "title": f[2],
                    "mass": float(f[3]), "crash": float(f[4]),
                    "y": float(f[5]), "low": float(f[6]), "joints": int(f[7]),
                    "break_force": num(f[8]), "break_torque": num(f[9]),
                    "spring": float(f[10]), "damper": float(f[11])})
    return out


def touching(parts, band=1.0):
    """Indices of the parts within ``band`` m of the craft's lowest point."""
    if not parts:
        return []
    bottom = min(p["low"] for p in parts)
    return [p["index"] for p in parts if p["low"] <= bottom + band]


def peak_load(parts, speed, duration=0.4, dt=2e-4, band=1.0):
    """``(worst ratio, title)``: the largest joint force over its break force
    (or touching part's speed over its crash tolerance) when the craft
    meets rigid ground at ``speed`` m/s straight down."""
    by = {p["index"]: p for p in parts}
    ground = set(touching(parts, band))
    worst, who = 0.0, None
    for i in ground:
        c = by[i]["crash"]
        if c > 0.0 and speed / c > worst:
            worst, who = speed / c, by[i]["title"]
    # Along the axis, every part moving down at ``speed``; the ground stops
    # the touching ones.  x is each part's displacement from its rest place.
    x = {i: 0.0 for i in by}
    v = {i: (0.0 if i in ground else -speed) for i in by}
    links = [(i, p["parent"]) for i, p in by.items()
             if p["parent"] in by and p["spring"] > 0.0]
    steps = int(duration / dt)
    for _ in range(steps):
        force = {i: 0.0 for i in by}
        for child, parent in links:
            p = by[child]
            # Tension positive: the joint's force on the child toward the parent.
            f = (p["spring"] * (x[parent] - x[child])
                 + p["damper"] * (v[parent] - v[child]))
            force[child] += f
            force[parent] -= f
            bf = p["break_force"]
            if bf > 0.0 and bf != INF and abs(f) / bf > worst:
                worst, who = abs(f) / bf, "%s joint" % p["title"]
        for i in by:
            if i in ground:
                continue
            m = by[i]["mass"]
            if m > 0.0:
                v[i] += force[i] / m * dt
            x[i] += v[i] * dt
    return worst, who


def survivable_speed(parts, hi=None, tol=0.05, band=1.0):
    """``(speed, title)``: the highest straight-down contact speed with every
    ratio of :func:`peak_load` below 1, and what fails first above it; or
    None when nothing was read."""
    if not parts:
        return None
    ground = touching(parts, band)
    if hi is None:
        crash = [p["crash"] for p in parts if p["index"] in ground
                 and p["crash"] > 0.0]
        hi = min(crash) if crash else 50.0
    lo, who = 0.0, None
    ratio, name = peak_load(parts, hi, band=band)
    if ratio < 1.0:
        return hi, name
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        ratio, name = peak_load(parts, mid, band=band)
        if ratio < 1.0:
            lo = mid
        else:
            hi, who = mid, name
    return lo, who


def read(conn):
    """The active vessel's parts via the launcher mod, or [] without it."""
    try:
        return parse_parts(conn.krpc_lab_joints.parts())
    except Exception:                                       # noqa: BLE001
        return []
