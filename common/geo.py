"""Body geometry the propagators share: point-mass gravity and great-circle
distance.  ``env`` is any object with ``mu`` and ``target_radius``."""

import math

from common import vec


def gravity_at(env, r):
    """Point-mass gravitational acceleration at ``r`` (body-centred)."""
    d = vec.norm(r)
    return vec.scale(r, -env.mu / (d * d * d))


def surface_distance(env, a, b):
    """Great-circle distance between the directions of ``a`` and ``b``, at
    ``env.target_radius``."""
    ua, ub = vec.unit(a), vec.unit(b)
    dot = vec.clamp(vec.dot(ua, ub), -1.0, 1.0)
    return env.target_radius * math.acos(dot)
