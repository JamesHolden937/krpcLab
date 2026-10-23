"""``VACUUM_OWN``: the coast's attitude flown by the client on a measured
input matrix, for a craft kRPC cannot point in vacuum.

kRPC assumes each control input turns its own axis.  The cargo shuttle's
RCS does not: in body rates a yaw input makes more pitch than yaw and a
roll input mostly pitch and yaw (2026-10-22, 1 s pulses in orbit), so
kRPC's corrections land on the wrong axes and a 90 deg step never closes
(``tools/vacbench.py``: the error grows).  The game applies raw inputs
fine, so the coast measures B -- body angular acceleration per unit of
each input -- and inverts it: the inputs that make the rotation wanted.

Body components are ``(right, nose, belly)`` as in ``allocate``; inputs
are ``(pitch, yaw, roll)``.  Free of kRPC so it can be tested offline.
"""

import math

from common import vec
from spaceplane import allocate

INPUTS = ("pitch", "yaw", "roll")


def matrix(plus, minus, seconds):
    """B as three columns, one per input: the body angular-velocity change
    a +1 pulse made minus the one a -1 pulse made, over twice the pulse.
    The difference cancels whatever the craft was already doing."""
    if seconds <= 0.0:
        return None
    return [vec.scale(vec.sub(p, m), 0.5 / seconds)
            for p, m in zip(plus, minus)]


def _rows(columns):
    return [[columns[c][r] for c in range(3)] for r in range(3)]


def inputs_for(columns, accel):
    """The inputs whose B-weighted sum is ``accel``; ``None`` if B is
    singular."""
    return allocate._solve3(_rows(columns), list(accel))


def axis_authority(columns):
    """Per body axis, the largest pure angular acceleration the inputs can
    make with every input inside +-1 (rad/s^2); 0 for an axis B cannot
    reach."""
    out = []
    for k in range(3):
        unit = [1.0 if i == k else 0.0 for i in range(3)]
        u = inputs_for(columns, unit)
        biggest = max(abs(x) for x in u) if u else 0.0
        out.append(1.0 / biggest if biggest > 0.0 else 0.0)
    return tuple(out)


def command(columns, authority, error, rate, tau, share, w_max):
    """The inputs for one tick.

    Per body axis, the rate to be turning at is the bang-bang approach to
    ``error`` (rad) on ``share`` of that axis's authority, capped at
    ``w_max``; the acceleration asked is the rate error over ``tau``; the
    inputs are B's inverse of it, scaled together until every one fits in
    +-1 so the direction of the correction survives saturation."""
    accel = []
    for e, r, a in zip(error, rate, authority):
        a = max(0.0, a) * share
        want = math.copysign(min(w_max, math.sqrt(2.0 * a * abs(e))), e) \
            if a > 0.0 else 0.0
        accel.append((want - r) / max(1e-3, tau))
    u = inputs_for(columns, accel)
    if u is None:
        return (0.0, 0.0, 0.0)
    biggest = max(abs(x) for x in u)
    if biggest > 1.0:
        u = tuple(x / biggest for x in u)
    return tuple(u)


def error_and_rate(right, nose, roof, angular_velocity, want_nose, want_up):
    """``(error, rate)`` in body components: the rotation taking the
    vessel's axes onto the wanted ones, and its angular velocity; ``None``
    when the target is degenerate."""
    axes = allocate.body_axes(right, nose, roof)
    target = allocate.target_axes(want_nose, want_up)
    if target is None:
        return None
    error = allocate.body(allocate.attitude_error(axes, target), axes)
    return error, allocate.body(angular_velocity, axes)
