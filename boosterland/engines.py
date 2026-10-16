"""Throttle allocation across a cluster: center engine first, then pairs.

A booster with several engines can make a given thrust two ways: every
engine at the same part throttle, or as few engines as possible at full.
The second is how a real booster lands -- a 9 g, five-engine stage cannot
hover on all five, and a small throttle change on five engines is a large
force.  So the throttle the guidance asks for (a fraction of the cluster's
total thrust) is filled group by group: the engine on the thrust axis
first, then symmetric pairs outward, each to full before the next lights.

A *group* is a set of engines whose combined thrust passes through the
cluster's axis, so lighting it adds no torque: one engine on the axis, two
opposite each other, or -- for a ring with no opposite partners, such as
three at 120 degrees -- the whole ring together.

Pure functions: positions in, groups and per-engine limits out.  The kRPC
side (finding the engines, writing ``thrust_limit``) is in ``autoland``.
"""

import math


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _norm(a):
    return math.hypot(a[0], a[1])


def group_engines(engines, axis_tol=0.3, ring_tol=0.3, pair_tol=0.25):
    """Groups of engine keys, in lighting order.

    ``engines`` is ``[(key, (x, y), thrust), ...]`` with ``(x, y)`` the
    engine's offset in the plane perpendicular to the thrust axis, metres.
    Offsets are taken about the thrust-weighted centroid, so an engine
    plate that is not centred on the vessel's own axis still groups right.

    * within ``axis_tol`` of the centroid: the center group, lit first;
    * the rest in rings (radii within ``ring_tol``), innermost first;
    * in a ring, an engine is paired with the one whose offset most nearly
      cancels its own (``|a + b| <= pair_tol * radius``) and whose thrust
      matches within ``pair_tol``; what cannot be paired stays together as
      one group, which for a symmetric ring is balanced.
    """
    if not engines:
        return []
    total = sum(t for _, _, t in engines) or 1.0
    cx = sum(p[0] * t for _, p, t in engines) / total
    cy = sum(p[1] * t for _, p, t in engines) / total
    rel = [(k, _sub(p, (cx, cy)), t) for k, p, t in engines]

    center = [k for k, p, _ in rel if _norm(p) <= axis_tol]
    rest = sorted((e for e in rel if _norm(e[1]) > axis_tol),
                  key=lambda e: _norm(e[1]))
    rings = []
    for e in rest:
        if rings and _norm(e[1]) - _norm(rings[-1][0][1]) <= ring_tol:
            rings[-1].append(e)
        else:
            rings.append([e])

    groups = [center] if center else []
    for ring in rings:
        free = list(ring)
        while free:
            k, p, t = free.pop(0)
            r = _norm(p)
            best, best_err = None, None
            for j, (k2, p2, t2) in enumerate(free):
                err = _norm((p[0] + p2[0], p[1] + p2[1]))
                if (err <= pair_tol * r
                        and abs(t - t2) <= pair_tol * max(t, t2)
                        and (best_err is None or err < best_err)):
                    best, best_err = j, err
            if best is None:
                groups.append(("unpaired", k))
            else:
                groups.append([k, free.pop(best)[0]])
        # Whatever found no partner in this ring lights together.
        lone = [g[1] for g in groups if isinstance(g, tuple)]
        groups = [g for g in groups if not isinstance(g, tuple)]
        if lone:
            groups.append(lone)
    return groups


def allocate(demand, groups, thrust):
    """Per-engine thrust limits that make ``demand`` of the cluster's thrust.

    ``demand`` is a fraction of the total (0-1); ``thrust`` maps each key to
    its maximum thrust.  Groups fill in order, each to 1.0 before the next
    starts; the group in progress gets the fraction that completes the
    demand.  Every engine in a group gets the same limit, so it stays
    balanced.
    """
    total = sum(thrust[k] for g in groups for k in g)
    limits = {}
    if total <= 0.0:
        return limits
    left = max(0.0, min(1.0, demand)) * total
    for g in groups:
        cap = sum(thrust[k] for k in g)
        x = 0.0 if cap <= 0.0 else max(0.0, min(1.0, left / cap))
        for k in g:
            limits[k] = x
        left -= x * cap
    return limits
