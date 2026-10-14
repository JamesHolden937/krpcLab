"""The solver: the Hohmann transfer split into many small burns.  Pure
functions, no kRPC.

Every tick, from the current orbit alone, it answers: thrust or coast,
prograde or retrograde, how hard.  There is no stored schedule -- a lost
tick, a weak engine or a shadow pass is corrected on the next pass by the
same law.

**Decoupled.**  A tangential burn at periapsis moves the apoapsis and, to
first order, not the periapsis; one at apoapsis moves only the periapsis.
So each apsis arc fixes one element: at the periapsis arc the apoapsis is
driven to its target, at the apoapsis arc the periapsis.  The last pass
nulls its element exactly, so the end is deadbeat.

**Fixed apse line.**  The two arcs sit on an apse line held fixed in
inertial space (``Target.apse``) through each burn, and the law controls
the orbit's radius *toward* those two directions, r(phi) = p / (1 + e.u),
which is smooth through circular.  Between burns the line is re-locked to
the real periapsis (``lock_apse``).

**Arcs, and the time budget.**  The arcs are centred on the two directions
with half-width acos(eta_cut) (a tangential burn's effectivity off-apsis is
about the cosine of the angle).  A burn starts half its duration before
the apsis and runs at full throttle until it is done or the arc ends, so a
burn is centred and no single pass is longer than the arc.  Narrow arcs are
many small near-impulsive burns, within a few percent of the impulsive
Hohmann; wide arcs deliver more per pass and finish sooner for more dv.
The planner (``plan.py``) picks the narrowest arcs that meet the deadline.

**Order.**  Outward first and inward first, as Hohmann: the periapsis is
raised only once the apoapsis is up (a periapsis raise is cheapest at the
final apoapsis), the apoapsis lowered only once the periapsis is down.

**Plane change.**  Normal or anti-normal arcs at the two nodes between the
current plane and the target plane (held fixed through the transfer),
with the same half-width and the same half-duration-early start.  A
plane change costs the velocity it rotates, so it is cheapest where the
vessel is slowest: a node whose radius is below eta_cut of the other
node's is skipped, the plane waits until the apoapsis is raised, and the
apoapsis is not lowered until the plane is done -- the plane is changed on
the biggest orbit of the transfer.  Where a node arc and an apsis arc
overlap, the two burns are one vector, which is cheaper than both.

Pure apsis or pure plane burns are prograde/retrograde or normal/anti-
normal -- plain SAS modes.  A combined burn points between them.
"""

import math
from collections import namedtuple

from common import vec
from lowThrust import orbit

# ra, rp: radii, m; inc, lan: rad (None = free).  ``apse`` is the in-plane
# inertial unit vector the periapsis arc is centred on and ``normal`` the
# target plane's unit normal; ``lock`` sets both.
# ``hold_apse``: the periapsis must lie along ``apse`` (a departure's
# parking orbit): never re-locked, and arcs at +-90 deg from it correct the
# eccentricity across the line once the apsides are nearly there.
# ``argpe`` (rad): the periapsis this far past the ascending node of the
# target plane (from ``lan`` on an equatorial one); ``lock`` turns it into a
# held apse line once the plane is known.
Target = namedtuple("Target", "ra rp inc lan apse normal hold_apse argpe",
                    defaults=(None, None, None, None, False, None))
# ``prograde`` and ``normal`` are the components of the unit thrust
# direction along the velocity and the orbit normal; ``eta`` this point's
# effectivity, ``err`` the error (m) of the radius an apsis arc is driving.
Command = namedtuple("Command", "done throttle prograde normal eta err need",
                     defaults=(0.0,))
COAST = None


def lock_apse(target, el, coasting=True):
    """The target with its apse line set.

    Re-locked to the osculating periapsis whenever the vessel is coasting,
    held through a burn.  Within a burn the line must not move: near-
    circular, the osculating apse turns as fast as a vessel thrusting short
    of apoapsis moves, and a law chasing it rode its arc's edge for weeks.
    Between burns it must follow the real orbit: held for a whole transfer,
    the burns cannot correct the eccentricity across the line, and a 500 km
    circular target ended 628 x 396.  On a circular orbit with no line yet,
    the vessel's position.
    """
    if target.apse is not None and (target.hold_apse or not coasting):
        return target
    if el.e > 1e-6:
        return target._replace(apse=vec.unit(el.evec))
    if target.apse is None:
        return target._replace(apse=vec.unit(el.rvec))
    return target


def lock(target, el, cfg, coasting=True):
    """``lock_apse``, and the target plane fixed once.

    With the node longitude free, the plane keeps the current one's (the
    smallest rotation that gives the inclination).  An equatorial orbit has
    none, so the choice is free: it is made when the plane change may begin
    (``_plane_may_start``) and puts the nodes on the apse line of that
    moment, so the plane burns share the apsis arcs.  (Fixed at the start,
    15 orbits of apoapsis raising had turned the apse line off the nodes.)
    """
    target = lock_apse(target, el, coasting)
    if target.inc is not None and target.normal is None:
        lan = target.lan
        if lan is None:
            inc0, lan = orbit.inclination_lan(el.rvec, el.vvec)
            if math.sin(inc0) < 1e-6:
                ref = target.apse
                if target.argpe is not None:
                    # an argument of periapsis: the node where the apse
                    # line, held from now, lies that far past it
                    w = target.argpe
                    lan = math.atan2(ref[1], ref[0]) - math.atan2(
                        math.cos(target.inc) * math.sin(w), math.cos(w))
                elif not (coasting and _plane_may_start(el, target, cfg)):
                    return target
                else:
                    lan = math.atan2(ref[1], ref[0])
        target = target._replace(normal=orbit.plane_normal(target.inc, lan))
    if target.argpe is not None and not target.hold_apse:
        target = _lock_argpe(target, el)
    return target


def _lock_argpe(target, el):
    """The held apse line of a target argument of periapsis: ``argpe`` past
    the ascending node, in the target plane (the orbit's own when the
    inclination is free); on an equatorial plane the node is the ``lan``
    direction (0 if free).  Not yet if the plane is still to be chosen."""
    if target.normal is not None:
        n = target.normal
    elif target.inc is None:
        n = vec.unit(vec.cross(el.rvec, el.vvec))
    else:
        return target
    node = vec.cross((0.0, 0.0, 1.0), n)
    if vec.norm(node) < 1e-6:
        lan = target.lan or 0.0
        node = (math.cos(lan), math.sin(lan), 0.0)
    node = vec.unit(node)
    w = target.argpe
    apse = vec.add(vec.scale(node, math.cos(w)),
                   vec.scale(vec.cross(n, node), math.sin(w)))
    return target._replace(apse=vec.unit(apse), hold_apse=True)


def _plane_may_start(el, target, cfg):
    """The plane is changed on the biggest orbit: not while the apoapsis is
    still being raised."""
    return target.ra is None or el.ra >= target.ra - cfg.TOLERANCE_M


def plane_error(el, target):
    """Angle (rad) between the orbit plane and the target plane; 0 if free.
    Before the target plane is fixed, the inclination still to change."""
    if target.normal is None:
        if target.inc is None:
            return 0.0
        return abs(orbit.inclination_lan(el.rvec, el.vvec)[0] - target.inc)
    n = vec.unit(vec.cross(el.rvec, el.vvec))
    return math.acos(max(-1.0, min(1.0, vec.dot(n, target.normal))))


def _in_plane(el, direction):
    """``direction`` projected into the orbit plane, unit."""
    n = vec.unit(vec.cross(el.rvec, el.vvec))
    return vec.unit(vec.sub(direction, vec.scale(n, vec.dot(direction, n))))


def apse_error(el, target):
    """Angle (rad) between the periapsis and a held apse line; 0 if not held
    or the orbit is near circular."""
    if not target.hold_apse or el.e < 0.01:
        return 0.0
    return math.acos(max(-1.0, min(1.0, vec.dot(
        vec.unit(el.evec), _in_plane(el, target.apse)))))


def arrived(el, target, cfg):
    """Every targeted apsis, the plane and a held apse within tolerance."""
    return ((target.ra is None or abs(el.ra - target.ra) < cfg.TOLERANCE_M)
            and (target.rp is None
                 or abs(el.rp - target.rp) < cfg.TOLERANCE_M)
            and plane_error(el, target)
            < math.radians(cfg.PLANE_TOLERANCE_DEG)
            and apse_error(el, target) < math.radians(cfg.APSE_TOLERANCE_DEG))


def _apsis(el, target, accel, tick_s, half, cfg, mu, plane_pending):
    """The apsis arc's burn: (seconds needed at full thrust, sign, eta,
    err, seconds until the apsis), or COAST."""
    apse = _in_plane(el, target.apse) if target.hold_apse else target.apse
    theta = orbit.plane_angle(el.rvec, el.vvec, apse)

    def wrap(x):
        return math.atan2(math.sin(x), math.cos(x))
    d_ap = wrap(theta - math.pi)
    at_pe = abs(theta) <= abs(d_ap)          # this arc moves the apoapsis
    if at_pe:
        off, goal, far = theta, target.ra, vec.scale(apse, -1.0)
    else:
        off, goal, far = d_ap, target.rp, apse
    if target.hold_apse and abs(off) > half and None not in (target.ra,
                                                             target.rp):
        # Across the held line: at +-90 deg from it, drive the radius toward
        # the opposite side to the target's semi-latus rectum -- only once
        # the apsides are within ``APSE_ARC_GATE`` of target, as an end-game
        # correction of the eccentricity the apsis arcs cannot reach.
        side = vec.cross(vec.unit(vec.cross(el.rvec, el.vvec)), apse)
        r_pe = orbit.radius_toward(el.p, el.evec, apse)
        r_ap = orbit.radius_toward(el.p, el.evec, vec.scale(apse, -1.0))
        gate = cfg.APSE_ARC_GATE
        if (abs(r_pe - target.rp) < gate * target.rp
                and abs(r_ap - target.ra) < gate * target.ra):
            d_q = wrap(theta - math.pi / 2)
            d_mq = wrap(theta + math.pi / 2)
            if abs(d_q) <= half:
                off, far = d_q, vec.scale(side, -1.0)
            elif abs(d_mq) <= half:
                off, far = d_mq, side
            else:
                return COAST
            goal = 2 * target.ra * target.rp / (target.ra + target.rp)
            at_pe = None                     # no Hohmann ordering here
    if abs(off) > half or goal is None:
        return COAST

    # Hohmann's order (an explicit flag, not ``goal is target.rp``: under
    # PyPy ``is`` compares floats by value, and a circular target deadlocked)
    # -- and the apoapsis is not lowered while the plane still has to change.
    tol = cfg.TOLERANCE_M
    if at_pe is False and target.ra is not None:
        if el.rp < goal and el.ra < target.ra - tol:
            return COAST
    if at_pe is True and el.ra > goal:
        if target.rp is not None and el.rp > target.rp + tol:
            return COAST
        if plane_pending:
            return COAST

    # How the radius toward ``far`` moves per m/s of prograde, numerically
    # (exact for the orbit, and no singularity at e = 0).
    _, T, _ = orbit.rtn(el.rvec, el.vvec)
    radius = orbit.radius_toward(el.p, el.evec, far)
    err = radius - goal
    bump = orbit.elements(el.rvec, vec.add(el.vvec, vec.scale(T, cfg.RATE_DV)),
                          mu)
    rate = (orbit.radius_toward(bump.p, bump.evec, far) - radius) / cfg.RATE_DV
    if rate == 0 or abs(err) < tol:
        return COAST
    sign = -1 if err * rate > 0 else 1
    need = abs(err) / (abs(rate) * accel)          # seconds at full thrust

    wait = orbit.time_to(el, el.nu - off, mu) if off < 0 else 0.0
    return need, sign, math.cos(off), err, wait


def _plane(el, target, accel, tick_s, half, eta_cut, cfg, mu):
    """The node arc's burn: (seconds needed at full thrust, sign, eta, 0,
    seconds until the node), or COAST."""
    delta = plane_error(el, target)
    if delta < math.radians(cfg.PLANE_TOLERANCE_DEG) or target.normal is None:
        return COAST
    if not _plane_may_start(el, target, cfg):
        return COAST                       # change the plane on the big orbit
    n = vec.unit(vec.cross(el.rvec, el.vvec))
    nodes = vec.unit(vec.cross(n, target.normal))
    phi = orbit.plane_angle(el.rvec, el.vvec, nodes)
    if abs(phi) <= math.pi / 2:
        off, node = phi, nodes
    else:
        off = math.atan2(math.sin(phi - math.pi), math.cos(phi - math.pi))
        node = vec.scale(nodes, -1.0)
    r_node = orbit.radius_toward(el.p, el.evec, node)
    r_other = orbit.radius_toward(el.p, el.evec, vec.scale(node, -1.0))
    weight = r_node / max(r_node, r_other)
    if abs(off) > half or weight < eta_cut:
        return COAST
    # While the periapsis still has to change, only the node on the
    # apoapsis side, where the plane burn shares the periapsis burn: on a
    # circular orbit both nodes are equally good, and half the change went
    # uncombined at the other (3000 -> 100 km with 20 deg: +15%).
    # The periapsis measured as the apsis arc measures it (toward a held
    # apse line, not the osculating one), and a node within an arc of the
    # quadrature allowed: with the two gates measuring differently and the
    # node at 90 deg from the line, a weak engine's parking orbit waited
    # 4000 days for a burn neither gate would let start.
    if target.rp is not None:
        if target.hold_apse:
            rp_now = orbit.radius_toward(el.p, el.evec,
                                         _in_plane(el, target.apse))
        else:
            rp_now = el.rp
        if abs(rp_now - target.rp) > cfg.TOLERANCE_M:
            theta = orbit.plane_angle(el.rvec, el.vvec, target.apse)
            if abs(theta) < math.pi / 2 - half:
                return COAST
    # Normal thrust f tilts the orbit normal along -T * sign(f) (dh = r x f);
    # tilt it toward the target plane.
    tilt = vec.unit(vec.sub(target.normal, vec.scale(n, math.cos(delta))))
    _, T, _ = orbit.rtn(el.rvec, el.vvec)
    sign = 1 if vec.dot(T, tilt) < 0 else -1
    need = delta * el.h / (r_node * accel)      # rotate v_t by delta, at node
    wait = orbit.time_to(el, el.nu - off, mu) if off < 0 else 0.0
    return need, sign, math.cos(off) * weight, 0.0, wait


def command(el, target, accel, tick_s, eta_cut, cfg, mu):
    """This tick's thrust command.

    ``el`` the current elements (``orbit.elements``), ``target`` locked
    (``lock``), ``accel`` the acceleration the engine gives at full throttle
    now (m/s^2, measured, never the rated figure), ``tick_s`` how long the
    command will be held, ``eta_cut`` the arc cutoff.
    """
    if arrived(el, target, cfg):
        return Command(True, 0.0, 0.0, 0.0, 1.0, 0.0)
    half = math.acos(max(-1.0, min(1.0, eta_cut)))
    pending = plane_error(el, target) >= math.radians(cfg.PLANE_TOLERANCE_DEG)
    a = _apsis(el, target, accel, tick_s, half, cfg, mu, pending)
    p = _plane(el, target, accel, tick_s, half, eta_cut, cfg, mu)
    err = a[3] if a else 0.0
    # Start half the burn's duration before its apsis or node, so it is
    # centred -- the duration of the *combined* burn when both are due, or
    # the larger starts alone.  Consistent while burning: a second of burn
    # takes a second off the wait and half a second off the half-duration.
    # (Spreading a burn over the arc front-loaded it and overshot the far
    # apsis.)
    total = math.hypot(a[0] if a else 0.0, p[0] if p else 0.0)
    a = a if a and a[4] <= total / 2 else COAST
    p = p if p and p[4] <= total / 2 else COAST
    # One vector, in proportion to what each still needs, so both finish
    # together -- the impulsive combined burn.  (Each clipped to full
    # throttle first, an overlap always pointed at 45 deg and a 100 -> 500 km
    # raise with 5 deg of plane cost +11.8% on the impulsive.)
    along = a[0] * a[1] if a else 0.0
    across = p[0] * p[1] if p else 0.0
    need = math.hypot(along, across)
    if need == 0:
        return Command(False, 0.0, 0.0, 0.0, 0.0, err)
    eta = max(a[2] if a else 0.0, p[2] if p else 0.0)
    return Command(False, min(1.0, need / tick_s), along / need,
                   across / need, eta, err, need)
