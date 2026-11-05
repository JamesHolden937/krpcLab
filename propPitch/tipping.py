"""How much prop thrust a plane on its wheels can take before it tips over
a wheel, and what the props' turning does to the airframe.  No kRPC here.

**Tipping** (``thrust_bounds``).  The plane rests on its wheels; it starts
to go over when the moment about the foremost (nose-over) or rearmost
(tail-over) wheel contact lifts the wheels at the other end.  Every force
but the ground's is known or measured, in the vessel's axes about its CoM:

* gravity ``-m g up`` at the CoM;
* the inertia of the measured acceleration, ``-m a``, at the CoM (this is
  where the wheel brakes' share enters: their friction acts in the ground
  plane, so it has no moment about a contact line, but the deceleration it
  causes does);
* the rest of the aero force (measured less the props' thrust), taken at
  the CoM;
* the props' thrust ``T`` along the nose, at the hubs.

The ground's forces act on the contact line and have no moment about it,
so the moment ``M`` about a pivot ``P`` is these forces' alone.  Changing
the thrust by ``dT`` changes the acceleration by ``dT / m`` too, and the two
together move the moment by ``dT (hub x forward)``: the thrust line's
moment about the CoM, whichever pivot.  ``M`` is linear in ``T``, so the
thrust at which the far wheels stay loaded with ``margin`` of gravity's own
restoring moment is solved for directly.

Every moment and rotation here uses one cross-product formula, so the
answer holds in kRPC's left-handed frames as well as in a right-handed one.
"""

import math


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return _scale(a, 1.0 / n) if n > 0 else None


def contact_point(box_min, box_max, up):
    """The point of a wheel's bounding box (vessel frame) that touches the
    ground: below the box's centre, at its lowest extent along ``up``."""
    c = _scale(tuple(a + b for a, b in zip(box_min, box_max)), 0.5)
    half = _scale(_sub(box_max, box_min), 0.5)
    drop = sum(abs(h * u) for h, u in zip(half, up))
    return _sub(c, _scale(up, drop))


def thrust_bounds(contacts, up, g, mass, accel, aero, hub, thrust_now,
                  margin, forward=(0.0, 1.0, 0.0), min_span=0.3):
    """(floor, ceiling) on the props' thrust along ``forward`` (N) that keeps
    every wheel on the ground with ``margin`` (0..1) of gravity's restoring
    moment left, about both the foremost and the rearmost contact; each is
    None where that side sets no bound.  None if the contacts do not span
    ``min_span`` m along the ground (one wheel, or a row of them across).

    ``contacts`` (vessel frame, m, relative to the CoM), ``up`` (unit),
    ``g`` (m/s^2), ``mass`` (kg), ``accel`` (the vessel's measured
    acceleration, m/s^2), ``aero`` (the whole measured aero force, N),
    ``hub`` (where the thrust acts), ``thrust_now`` (the props' share of
    ``aero`` along ``forward``, N)."""
    if len(contacts) < 2:
        return None
    fh = _unit(_sub(forward, _scale(up, _dot(forward, up))))
    if fh is None:
        return None
    axis = _unit(_cross(up, fh))
    along = [_dot(c, fh) for c in contacts]
    rear = contacts[along.index(min(along))]
    front = contacts[along.index(max(along))]
    if max(along) - min(along) < min_span:
        return None
    grav = _scale(up, -mass * g)
    rest = _sub(aero, _scale(forward, thrust_now))
    other = tuple(gr - mass * a + r for gr, a, r in zip(grav, accel, rest))
    prop = _scale(forward, thrust_now)
    dmdt = _dot(_cross(hub, forward), axis)
    floor = ceiling = None
    for pivot, far in ((rear, front), (front, rear)):
        lever = _sub(far, pivot)
        # How far ``far`` rises per unit moment about the pivot's axis.
        rise = _dot(_cross(axis, lever), up)

        def tip(m_axis):
            return m_axis * rise
        m_now = (_dot(_cross(_scale(pivot, -1.0), other), axis)
                 + _dot(_cross(_sub(hub, pivot), prop), axis))
        m_grav = _dot(_cross(_scale(pivot, -1.0), grav), axis)
        restoring = tip(m_grav)
        if restoring >= 0.0:
            continue                    # gravity alone would lift it
        slope = tip(dmdt)
        if abs(slope) < 1e-12:
            continue
        limit = thrust_now + (margin * restoring - tip(m_now)) / slope
        if slope > 0.0:
            ceiling = limit if ceiling is None else min(ceiling, limit)
        else:
            floor = limit if floor is None else max(floor, limit)
    return floor, ceiling


def reaction_torque(rotors, torques, senses):
    """The props' torque on the airframe (vessel frame, same units as
    ``torques``): each rotor's blade load ``q`` (against its spin, as the
    model scores it) turns the airframe by ``-q`` about its real spin
    direction, ``sense`` x the model's (``None`` = not yet measured: 0)."""
    out = [0.0, 0.0, 0.0]
    for r, q, s in zip(rotors, torques, senses):
        if s is None or q is None:
            continue
        k = -q * s * r.spin
        for i in range(3):
            out[i] += k * r.axis[i]
    return tuple(out)


def counter_trim(torque, available):
    """The (pitch, roll, yaw) input that cancels ``torque`` (N m, vessel
    frame): KSP turns an input ``u`` into ``-u x`` the available torque on
    each axis (``ModuleReactionWheel.ActiveUpdate``: ReferenceTransform
    rotation x -inputVector; control surfaces follow the same sense).
    ``available`` is kRPC's ((+x, +y, +z), (-x, -y, -z)); an axis with no
    authority gets 0.  Clamped to +/-1."""
    out = []
    pos, neg = available
    for i in range(3):
        need = -torque[i]                       # the torque to apply
        cap = abs(pos[i]) if need >= 0.0 else abs(neg[i])
        u = 0.0 if cap <= 0.0 else torque[i] / cap
        out.append(max(-1.0, min(1.0, u)))
    return tuple(out)
