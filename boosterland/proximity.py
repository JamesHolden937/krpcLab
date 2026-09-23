"""Is it safe to flip yet?

Separation used to be a stopwatch: drift for ``SEPARATION_COAST_S`` in-game
seconds, then turn.  Three seconds is a guess, and it is the wrong guess in
both directions -- too long behind a stage that is already 40 m away and
falling behind, too short behind one that is still alongside because the
separation motors barely fired.  The booster then flips a 30 m vehicle and
lights its engines in whatever is next to it.

So instead of a timer this asks the question directly: propagate the booster's
own motion forward *under the boostback burn's acceleration*, propagate every
nearby craft on its current velocity, and see whether the two bounding spheres
ever touch inside ``CLEARANCE_HORIZON_S``.  The flip goes when the answer is
no.  Vehicles are reduced to spheres about their own bounding boxes because
the booster is about to rotate through 150 deg: a sphere is the only shape
that is still correct once it has.
"""

import math
from dataclasses import dataclass

from . import vec


@dataclass
class Neighbour:
    name: str
    position: tuple          # body frame
    velocity: tuple
    radius: float


@dataclass
class Conflict:
    """The nearest miss the scan found, and when it happens."""
    neighbour: Neighbour
    separation: float        # gap between the two spheres, metres (negative)
    time: float              # seconds from now


def bounding_radius(vessel, cfg):
    """Radius of the sphere about the vessel's origin that contains it.

    ``bounding_box`` is not always answerable -- through staging KSP has
    returned corners around -7e17 m, which is what made every landing height
    in LOG4/5/6 wrong -- so the answer is range-checked before it is believed,
    exactly like ``Telemetry._refresh_leg_clearance``, and the caller falls
    back rather than trusting it.
    """
    try:
        lower, upper = vessel.bounding_box(vessel.reference_frame)
    except Exception:                   # noqa: BLE001 -- unloaded, or KSP-side
        return None
    extent = tuple(max(abs(lower[i]), abs(upper[i])) for i in range(3))
    radius = vec.norm(extent)
    if not math.isfinite(radius) or radius <= 0.0 \
            or radius > cfg.CLEARANCE_RADIUS_MAX_M:
        return None
    return radius


def keep_out_distance(gap_now, radius, other_radius, cfg):
    """How close the two craft may come, given how close they already are.

    The sum of the two bounding spheres is the obvious answer and it is
    unusable for the one craft this exists to avoid.  A second ago the booster
    and its upper stage were *one vehicle*, so their centres are always closer
    together than the sum of their half-lengths -- the sphere model is
    violated at t=0 by construction, and it says so forever no matter how fast
    they separate.  LOG16 is what that costs: the stage 19 m away reported a
    44.5 m radius, the booster's own box was unreadable so it flew on a 15 m
    fallback, and a keep-out of 64.5 m against a 19 m gap could never clear.
    SEPARATION held the attitude at Mach 2.4 with the grid fins already
    deployed until the flight was terminated, and the vehicle lost an airbrake
    to the airstream it was sitting broadside in.

    So the keep-out is capped at the gap the two craft are demonstrably
    sitting at right now.  They are intact and they are not touching, so this
    distance is survivable whatever the boxes claim; what is left to avoid is
    getting *closer* than this.  That keeps every tooth that matters -- a
    burn aimed back at the stage still closes the gap and still reads as a
    conflict -- and drops the one assertion the geometry cannot support.
    """
    return min(radius + other_radius + cfg.CLEARANCE_MARGIN_M, gap_now)


def conflict(position, velocity, accel, radius, neighbours, cfg, horizon):
    """The first neighbour a flip from here would run into, or None.

    Relative motion only: the booster accelerates under ``accel`` (the
    boostback burn, at full throttle along the aim it is about to hold) while
    the other craft coasts, and gravity is common to both so it drops out.
    Marching the separation rather than solving it keeps the quadratic honest
    -- the closest approach of two spheres under constant relative
    acceleration has no shortage of edge cases, and the horizon is seconds at
    a quarter-second step.
    """
    steps = max(1, int(horizon / max(1e-3, cfg.CLEARANCE_STEP_S)))
    worst = None
    for other in neighbours:
        p0 = vec.sub(position, other.position)
        dv = vec.sub(velocity, other.velocity)
        keep_out = keep_out_distance(vec.norm(p0), radius, other.radius, cfg)
        for step in range(steps + 1):
            t = step * cfg.CLEARANCE_STEP_S
            p = vec.add(p0, vec.add(vec.scale(dv, t),
                                    vec.scale(accel, 0.5 * t * t)))
            separation = vec.norm(p) - keep_out
            if separation < 0.0 and (worst is None or separation < worst.separation):
                worst = Conflict(other, separation, t)
    return worst


class ProximityScan:
    """The nearby-craft half of the question, which needs the game.

    Enumerating vessels is a remote call per vessel, so the list of candidates
    is re-read at most once per ``CLEARANCE_RESCAN_UT``; only SEPARATION asks,
    and only until it is clear, so the whole thing costs a handful of calls
    for a handful of seconds.
    """

    def __init__(self, conn, vessel, frame, cfg, logbook=None):
        self.conn = conn
        self.vessel = vessel
        self.frame = frame
        self.cfg = cfg
        self.log = logbook
        self.radius = None              # our own, once the box is believable
        self._candidates = []           # (vessel, radius), re-enumerated rarely
        self._next_scan_ut = None
        self._reported = False

    def own_radius(self, ut):
        """Our bounding sphere, re-measured until the game gives a real one."""
        if self.radius is None:
            self.radius = bounding_radius(self.vessel, self.cfg)
            if self.radius is not None and self.log is not None:
                self.log.event(ut, "clearance radius %.1f m" % self.radius)
        if self.radius is None:
            return self.cfg.CLEARANCE_RADIUS_FALLBACK_M
        return self.radius

    def _rescan(self, ut):
        """Re-enumerate what is in range.  Expensive, so done rarely.

        Debris counts.  So does the upper stage, which is the whole point --
        it is the one thing guaranteed to be there, and guaranteed to be
        close.  A vessel's extent never changes, so the radius is taken here;
        only where it *is* has to be read every tick.
        """
        if self._next_scan_ut is not None and ut < self._next_scan_ut:
            return
        self._next_scan_ut = ut + self.cfg.CLEARANCE_RESCAN_UT
        try:
            vessels = list(self.conn.space_center.vessels)
        except Exception:               # noqa: BLE001 -- KSP-side failure
            return
        here = tuple(self.vessel.position(self.frame))
        found = []
        for other in vessels:
            try:
                if other == self.vessel:
                    continue
                position = tuple(other.position(self.frame))
                if vec.norm(vec.sub(position, here)) > self.cfg.CLEARANCE_RANGE_M:
                    continue
                radius = bounding_radius(other, self.cfg)
                found.append((other, radius if radius is not None
                              else self.cfg.CLEARANCE_RADIUS_FALLBACK_M))
            except Exception:           # noqa: BLE001 -- gone, or unloaded
                continue
        self._candidates = found

    def neighbours(self, ut):
        """Where every craft in range is *now*.

        The list of candidates is cached; their positions are not, and must
        not be.  The booster is doing better than a kilometre a second at
        separation, so a position two seconds old is two kilometres of
        clearance the booster does not have -- cached state here would report
        every stage as cleanly missed on the tick after it was found.
        """
        self._rescan(ut)
        found = []
        for other, radius in self._candidates:
            try:
                found.append(Neighbour(
                    name=other.name,
                    position=tuple(other.position(self.frame)),
                    velocity=tuple(other.velocity(self.frame)),
                    radius=radius))
            except Exception:           # noqa: BLE001 -- gone, or unloaded
                continue
        if found and not self._reported and self.log is not None:
            self._reported = True
            here = tuple(self.vessel.position(self.frame))
            self.log.event(ut, "clearance scan: %s"
                           % ", ".join("%s r=%.1f d=%.0fm"
                                       % (n.name, n.radius,
                                          vec.norm(vec.sub(n.position, here)))
                                       for n in found))
        return found

    def check(self, ut, position, velocity, accel):
        """``Conflict`` if flipping and burning now would hit something.

        Two futures are checked, and the worse one decides: coasting, and
        accelerating along ``accel``.  The flip is not instantaneous, so for
        the first seconds after the phase ends the booster is still drifting
        on separation velocity with its thrust pointed somewhere between the
        old attitude and the new one -- the coasting case is what covers that
        stretch.  The thrusting case covers the rest, and is the one that
        matters when the burn drives the booster *toward* what it just
        dropped rather than away from it.

        They get different horizons, and the reason is the attitude.  The
        drifting case assumes nothing about where the booster points, so it
        can be trusted ten seconds out.  The thrusting case assumes the
        vehicle is *already* pointed at the boostback aim, which it is not --
        it has a 150 deg turn to make first -- so extrapolating that thrust
        much past ``CLEARANCE_BURN_HORIZON_S`` is fantasy, and expensive
        fantasy: at 17 m/s^2 a ten-second powered lookahead condemns anything
        within 800 m behind the booster and the phase can only ever time out.
        """
        radius = self.own_radius(ut)
        neighbours = self.neighbours(ut)
        worst = None
        for candidate, horizon in (((0.0, 0.0, 0.0), self.cfg.CLEARANCE_HORIZON_S),
                                   (tuple(accel), self.cfg.CLEARANCE_BURN_HORIZON_S)):
            found = conflict(position, velocity, candidate, radius,
                             neighbours, self.cfg, horizon)
            if found is not None and (worst is None
                                      or found.separation < worst.separation):
                worst = found
        return worst
