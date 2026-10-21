"""Attitude control by incremental dynamic inversion and control allocation.

``SURFACE_OWN``'s arithmetic, free of kRPC so it can be tested offline.
Vectors are body components ``(x, y, z)`` = (right, nose, belly) in the
game's raw sense: torque and angular velocity about +x are nose-down, and
``vec.cross`` gives ``d(axis)/dt = w x axis`` (both measured in game,
``tools/stiffprobe.py`` and the rate-sign check of 2026-10-21).
"""

from common import vec


def attitude_error(current, target):
    """The rotation vector that takes the ``current`` body axes onto the
    ``target`` ones, in the same frame as the axes: half the sum of
    ``a x b`` over the axis pairs.  Exact in direction, ``sin`` in size,
    which is what a controller wants -- it saturates instead of wrapping.
    ``current`` and ``target``: ``(right, nose, belly)`` unit vectors."""
    e = (0.0, 0.0, 0.0)
    for a, b in zip(current, target):
        e = vec.add(e, vec.cross(a, b))
    return vec.scale(e, 0.5)


def body(v, axes):
    """``v`` (a world vector) in body components along ``axes``."""
    return tuple(vec.dot(v, a) for a in axes)


def target_axes(nose, up):
    """``(right, nose, belly)`` of the attitude whose nose is ``nose`` and
    whose roof points as near ``up`` as the nose allows; ``None`` when the
    two are parallel.  ``right = nose x belly`` matches the game's frame
    (checked against the vessel's own axes in ``body_axes``)."""
    n = vec.unit(nose)
    roof = vec.sub(up, vec.scale(n, vec.dot(up, n)))
    if vec.norm(roof) < 1e-6:
        return None
    belly = vec.scale(vec.unit(roof), -1.0)
    return (vec.unit(vec.cross(n, belly)), n, belly)


def body_axes(right, nose, roof):
    """The vessel's own ``(right, nose, belly)`` from its streamed axes."""
    return (vec.unit(right), vec.unit(nose), vec.scale(vec.unit(roof), -1.0))


def wanted_torque(inertia, error, rate, w, zeta):
    """Per body axis, ``I (w^2 e - 2 zeta w rate)``: the torque that makes
    the error decay as a second-order system of frequency ``w`` and damping
    ``zeta``.  ``w`` may be one number or one per axis."""
    ws = tuple(w) if isinstance(w, (tuple, list)) else (w, w, w)
    return tuple(i * (wi * wi * e - 2.0 * zeta * wi * r)
                 for i, wi, e, r in zip(inertia, ws, error, rate))


def solve(columns, want, lo, hi, reg=1e-3, passes=4):
    """Bounded least squares: steps ``d`` (one per actuator) with
    ``sum d_i columns[i] ~= want`` and ``lo_i <= d_i <= hi_i``.

    A damped pseudo-inverse over the free actuators, re-solved with every
    actuator that hit a bound fixed there -- the redistribution that lets a
    saturated surface's share go to the others.  ``reg`` is relative to the
    largest column, so it is scale-free.  Returns the steps and the torque
    they make."""
    n = len(columns)
    d = [0.0] * n
    free = [True] * n
    scale = max((vec.dot(c, c) for c in columns), default=0.0)
    if scale <= 0.0:
        return d, (0.0, 0.0, 0.0)
    lam = reg * scale
    for _ in range(passes):
        made = (0.0, 0.0, 0.0)
        for i in range(n):
            if not free[i]:
                made = vec.add(made, vec.scale(columns[i], d[i]))
        rest = vec.sub(want, made)
        # (B B^T + lam I) y = rest; d_free = B^T y
        m = [[lam if r == c else 0.0 for c in range(3)] for r in range(3)]
        for i in range(n):
            if free[i]:
                col = columns[i]
                for r in range(3):
                    for c in range(3):
                        m[r][c] += col[r] * col[c]
        y = _solve3(m, rest)
        if y is None:
            break
        changed = False
        for i in range(n):
            if not free[i]:
                continue
            d[i] = vec.dot(columns[i], y)
            if d[i] > hi[i]:
                d[i], free[i], changed = hi[i], False, True
            elif d[i] < lo[i]:
                d[i], free[i], changed = lo[i], False, True
        if not changed:
            break
    made = (0.0, 0.0, 0.0)
    for i in range(n):
        made = vec.add(made, vec.scale(columns[i], d[i]))
    return d, made


def _solve3(m, b):
    """3x3 linear solve by Cramer's rule; ``None`` when singular."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    D = det(m)
    if abs(D) < 1e-300:
        return None
    out = []
    for k in range(3):
        a = [row[:] for row in m]
        for r in range(3):
            a[r][k] = b[r]
        out.append(det(a) / D)
    return tuple(out)
