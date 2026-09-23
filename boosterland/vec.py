"""Minimal 3-vector helpers.

Vectors are plain ``(x, y, z)`` tuples so they can be passed straight to and
from kRPC calls without conversion.

Handedness note: kRPC reference frames are left-handed, so ``cross()`` here is
the ordinary component formula applied to those coordinates -- it may be the
mirror of the physically "right-handed" cross product.  Everything that needs
a cross product (the rotating-frame terms in :mod:`boosterland.trajectory`)
uses the sign that :func:`boosterland.environment.estimate_frame_omega`
measured in-game, so the convention cancels out.
"""

import math


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def unit(a):
    n = norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def project_out(a, axis):
    """Component of ``a`` perpendicular to the unit vector ``axis``."""
    return sub(a, scale(axis, dot(a, axis)))


def angle_between(a, b):
    """Angle between two vectors, in degrees."""
    na, nb = norm(a), norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    c = max(-1.0, min(1.0, dot(a, b) / (na * nb)))
    return math.degrees(math.acos(c))


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# -- quaternions -----------------------------------------------------------
#
# kRPC hands out attitudes as (x, y, z, w) quaternions in the frame the object
# was asked for.  These are only needed to answer one question -- "what would
# the vessel measure if it were pointed the way the descent flies?" -- and the
# frames are left-handed, so nothing here assumes a handedness or a
# composition order: ``rotation_onto`` tries both and keeps whichever one
# actually lands the vector where it was asked to, the same way
# ``Environment._measure_omega`` pins down the rotation vector.

def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    """Apply a quaternion to a vector."""
    x, y, z, w = q
    t = cross((x, y, z), v)
    t = scale(t, 2.0)
    return add(add(v, scale(t, w)), cross((x, y, z), t))


def quat_axis_angle(axis, radians):
    n = norm(axis)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(radians / 2.0) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(radians / 2.0))


def rotation_onto(rotation, source, target, tolerance=0.02):
    """``rotation`` re-aimed so that ``source`` ends up along ``target``.

    ``source`` is a direction the object currently has (its nose, say) and
    ``rotation`` is its current attitude.  The result is an attitude of the
    same object turned along the shortest arc that puts that direction on
    ``target``, which is what an aerodynamic probe needs in order to ask about
    an attitude the vessel is not currently holding.

    Returns ``None`` if the answer cannot be verified -- the caller then has
    nothing to guess with and should fall back to the real attitude.
    """
    s, t = unit(source), unit(target)
    if norm(s) < 0.5 or norm(t) < 0.5:
        return None
    c = clamp(dot(s, t), -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-4:
        return tuple(rotation)              # already there

    axis = cross(s, t)
    if norm(axis) < 1e-9:                   # exactly opposed: any perpendicular
        for trial in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            axis = cross(s, trial)
            if norm(axis) > 1e-6:
                break

    # Check the answer rather than trusting a convention.  ``local`` is the
    # body-frame direction that currently points along ``source``; a correct
    # attitude must carry that same body direction onto ``target``.  The two
    # pairings below are the two conventions a quaternion can be under -- one
    # where it rotates body into world, one where it is the inverse -- and
    # each is verified with the ``local`` that belongs to it, so a pass under
    # one cannot be mistaken for a pass under the other.
    pairings = (
        (lambda d: quat_mul(d, rotation), quat_conjugate(rotation)),
        (lambda d: quat_mul(rotation, d), rotation),
    )
    for delta in (quat_axis_angle(axis, angle), quat_axis_angle(axis, -angle)):
        if norm(sub(quat_rotate(delta, s), t)) > tolerance:
            continue                        # this arc turns the wrong way
        for compose, inverse in pairings:
            local = quat_rotate(inverse, s)
            candidate = compose(delta)
            if norm(sub(quat_rotate(candidate, local), t)) <= tolerance:
                return candidate
    return None
