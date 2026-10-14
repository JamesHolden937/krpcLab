"""The live game as the planner sees it: the solar system and the vessel's
state, read through kRPC.

The planner (``mission``) works in one right-handed inertial frame per body,
x-y the equator, z the north pole.  kRPC's ``non_rotating_reference_frame``
of a body has y toward its north pole and x-z in the equator, and is
left-handed; swapping y and z turns one into the other (and is its own
inverse).  Every body's non-rotating frame has the same axes in KSP, so
positions relative to a parent add up as the planner's do.

The bodies are on Keplerian rails, so elements taken from one state vector
give their whole motion exactly; ``check_system`` compares them against the
game's own predictions.
"""

import math

from common import vec
from lowThrust import bodies, orbit


def to_ours(t):
    """A kRPC non-rotating-frame vector in the planner's frame."""
    return (t[0], t[2], t[1])


to_krpc = to_ours        # the swap is its own inverse


def angles(r, v, mu):
    """(a, e, inc, lan, argpe, nu) of the orbit through (r, v): the inverse
    of ``orbit.state_from_elements``.  Circular: argpe 0, nu from the node;
    equatorial: lan 0, longitudes from +x."""
    h = vec.cross(r, v)
    hn = vec.norm(h)
    rn = vec.norm(r)
    inc = math.acos(max(-1.0, min(1.0, h[2] / hn)))
    node = (-h[1], h[0], 0.0)
    nn = vec.norm(node)
    ev = vec.sub(vec.scale(vec.cross(v, h), 1 / mu), vec.scale(r, 1 / rn))
    e = vec.norm(ev)
    a = 1 / (2 / rn - vec.dot(v, v) / mu)
    if nn < 1e-9 * hn:
        lan = 0.0
        ref = (1.0, 0.0, 0.0)
    else:
        lan = math.atan2(node[1], node[0]) % (2 * math.pi)
        ref = vec.scale(node, 1 / nn)
    # angles in the orbit plane, from ``ref``, positive with the motion
    w = vec.cross(vec.scale(h, 1 / hn), ref)

    def angle(x):
        return math.atan2(vec.dot(x, w), vec.dot(x, ref)) % (2 * math.pi)
    if e < 1e-9:
        return a, 0.0, inc, lan, 0.0, angle(r)
    argpe = angle(ev)
    nu = (angle(r) - argpe) % (2 * math.pi)
    return a, e, inc, lan, argpe, nu


def mean_from_true(nu, e):
    E = 2 * math.atan2(math.sqrt(1 - e) * math.sin(nu / 2),
                       math.sqrt(1 + e) * math.cos(nu / 2))
    return (E - e * math.sin(E)) % (2 * math.pi)


def system(conn):
    """{name: bodies.Body} for every body in the game, on the rails the game
    has them on now (their elements at the current UT, mean anomaly carried
    back to UT 0 as ``bodies.Body`` keeps it)."""
    sc = conn.space_center
    ut = sc.ut
    game = sc.bodies
    out = {}

    def make(name):
        if name in out:
            return out[name]
        cb = game[name]
        atm = cb.atmosphere_depth if cb.has_atmosphere else 0.0
        o = cb.orbit
        if o is None:
            b = bodies.Body(name, cb.gravitational_parameter,
                            cb.equatorial_radius, atm)
        else:
            parent = make(o.body.name)
            frame = game[parent.name].non_rotating_reference_frame
            r = to_ours(cb.position(frame))
            v = to_ours(cb.velocity(frame))
            a, e, inc, lan, argpe, nu = angles(r, v, parent.mu)
            n = math.sqrt(parent.mu / a ** 3)
            m0 = (mean_from_true(nu, e) - n * ut) % (2 * math.pi)
            b = bodies.Body(name, cb.gravitational_parameter,
                            cb.equatorial_radius, atm, parent, a, e, inc,
                            lan, argpe, m0)
            parent.children.append(b)
        b.game_soi = cb.sphere_of_influence if o is not None else math.inf
        out[name] = b
        return b
    for name in sorted(game):
        make(name)
    return out


def check_system(conn, system_, horizon_s=1e6):
    """The largest error, per body, between the rails built here and the
    game's ``Orbit.position_at`` ``horizon_s`` ahead, and in the SOI radius
    (metres): [(name, position error, SOI error)]."""
    sc = conn.space_center
    game = sc.bodies
    t = sc.ut + horizon_s
    rows = []
    for name, b in system_.items():
        if b.parent is None:
            continue
        frame = game[b.parent.name].non_rotating_reference_frame
        got = to_ours(game[name].orbit.position_at(t, frame))
        rows.append((name, vec.norm(vec.sub(got, b.position(t))),
                     abs(b.soi - b.game_soi)))
    return rows


def vessel_state(vessel, body_frame):
    """(r, v) of the vessel in the planner's frame about the body whose
    non-rotating frame is ``body_frame``."""
    return (to_ours(vessel.position(body_frame)),
            to_ours(vessel.velocity(body_frame)))
