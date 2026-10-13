"""A whole transfer, flown in the stand-in solar system: within one SOI, to
a moon, or to another planet.  The phase machine here is the one the pilot
will run against kRPC; only the propagation is the stand-in's.

Phases (each re-decided every tick from the current state):

    within SOI:     FINAL
    to a moon:      PUMP -> WAIT -> PUSH -> COAST -> APPROACH -> CAPTURE -> FINAL
    to a planet:    PARK -> WAIT -> DEPART -> ESCAPE -> CRUISE -> APPROACH
                    -> CAPTURE -> FINAL

- PUMP / PARK / FINAL fly ``guidance`` (Hohmann in small burns, plane
  change at the nodes) to an orbit about the current body.  PUMP's waiting
  orbit is coplanar with the moon, its apoapsis just short of the moon's
  SOI, its periapsis raised as far as a weak engine needs to capture in one
  pass.  PARK's ellipse is oriented so its periapsis push leaves on the
  window's departure asymptote (``transfer.parking_orientation``).
- WAIT coasts.  To a moon: at each periapsis pass it asks whether a
  prograde push there can put the encounter on the aim point
  (``transfer.push_dv``) and starts the push half its duration early.  To a
  planet: it waits for the periapsis pass nearest the window's departure.
- PUSH re-solves the push every tick and burns prograde until nothing is
  left to add.  DEPART burns prograde at periapsis until the hyperbolic
  excess is the window's.
- COAST / CRUISE correct the B-plane miss at the target
  (``transfer.correction``) whenever it needs more than ``CORRECT_DV``.
- APPROACH, inside the target's SOI, nudges the hyperbola's periapsis onto
  the capture altitude.  CAPTURE only has to make the orbit bound (below
  ``CAPTURE_AP_FRACTION`` of the SOI, or the moon-clear ceiling once past
  periapsis); FINAL shapes it.  Its law (``transfer.capture_step``) burns a
  short capture centred on periapsis and starts a long one when it must --
  from COAST/CRUISE, outside the SOI if need be -- once the energy to shed
  reaches ``CAPTURE_PATH_SHARE`` of what full thrust removes on the path
  left to periapsis, then throttles to finish there.
"""

import math
from collections import namedtuple
from dataclasses import dataclass

from common import vec
from lowThrust import guidance, orbit, transfer
from lowThrust.sim import _rk4, tick_for, xenon_kg

Maneuver = namedtuple("Maneuver", "phase body t0 t1 dv prograde normal radial")
Event = namedtuple("Event", "t text")
MissionResult = namedtuple(
    "MissionResult", "done reason t0 t dv prop_used maneuvers events body "
                     "final")


@dataclass
class Craft:
    body: object
    r: tuple
    v: tuple
    m: float
    t: float


def _child_hit(craft, child, t, r):
    return vec.norm(vec.sub(r, child.position(t))) < child.soi


def _crossing(craft, r, t):
    """The SOI boundary (r, t) has crossed: ('exit', None), ('enter',
    child), or None."""
    if vec.norm(r) > craft.body.soi:
        return "exit", None
    for c in craft.body.children:
        if _child_hit(craft, c, t, r):
            return "enter", c
    return None


def _switch(craft, kind, child):
    if kind == "exit":
        b = craft.body
        rb, vb = b.state(craft.t)
        craft.r, craft.v = vec.add(craft.r, rb), vec.add(craft.v, vb)
        craft.body = b.parent
    else:
        rb, vb = child.state(craft.t)
        craft.r, craft.v = vec.sub(craft.r, rb), vec.sub(craft.v, vb)
        craft.body = child


def coast(craft, dt):
    """Coast ``dt`` exactly, stopping at an SOI boundary (bisected).
    Returns the crossing kind and body, or None."""
    mu = craft.body.mu
    r1, v1 = orbit.propagate(craft.r, craft.v, mu, dt)
    hit = _crossing(craft, r1, craft.t + dt)
    if hit is None:
        craft.r, craft.v, craft.t = r1, v1, craft.t + dt
        return None
    lo, hi = 0.0, dt
    for _ in range(50):
        mid = (lo + hi) / 2
        rm, _ = orbit.propagate(craft.r, craft.v, mu, mid)
        if _crossing(craft, rm, craft.t + mid) is None:
            lo = mid
        else:
            hi = mid
    craft.r, craft.v = orbit.propagate(craft.r, craft.v, mu, hi)
    craft.t += hi
    kind, child = _crossing(craft, craft.r, craft.t)
    _switch(craft, kind, child)
    return kind, child or craft.body


def _boundary_tick(craft):
    """A tick short enough not to jump an SOI boundary."""
    r = vec.norm(craft.r)
    spd = vec.norm(craft.v)
    lim = math.inf
    if craft.body.soi < math.inf:
        lim = max(10.0, 0.5 * (craft.body.soi - r) / max(spd, 1e-3))
    for c in craft.body.children:
        rc, vc = c.state(craft.t)
        gap = vec.norm(vec.sub(craft.r, rc)) - c.soi
        rel = vec.norm(vec.sub(craft.v, vc))
        lim = min(lim, max(10.0, 0.5 * gap / max(rel, 1e-3)))
    return lim


def _safe_until(craft, c):
    """True when the current bound orbit cannot reach any SOI boundary, so
    a long coast can be taken in one step."""
    if c.e >= 1 or c.ra >= craft.body.soi:
        return False
    for ch in craft.body.children:
        lo = ch.sma * (1 - ch.ecc) - ch.soi
        hi = ch.sma * (1 + ch.ecc) + ch.soi
        if not (c.ra < lo or c.rp > hi):
            return False
    return True


class Mission:
    def __init__(self, system, vessel, body, r, v, t0, dest, target, cfg,
                 record=True):
        self.cfg = cfg
        self.sys = system
        self.vessel = vessel
        self.craft = Craft(body, r, v, vessel.mass_kg, t0)
        self.t0 = t0
        self.dest = dest
        self._home = body
        self.final_target = target
        self.record = record
        self.dv = 0.0
        self.maneuvers = []
        self.events = []
        self._burn = None
        self._last_dv = 0.0
        self.phase = None
        self.target = None          # the in-SOI target of PUMP/PARK/FINAL
        self.plan = {}
        self.abort = None
        self._route()

    # --- bookkeeping ------------------------------------------------------
    def event(self, text):
        self.events.append(Event(self.craft.t, text))

    def set_phase(self, phase, text=""):
        self._close_burn()
        self._last_dv = 0.0         # a ledger starts clean (a stale value
        self.phase = phase          # once ended trims before they burned)
        self.event(f"{phase}" + (f": {text}" if text else ""))

    def _close_burn(self):
        b = self._burn
        if b is not None and self.record:
            n = max(b["dv"], 1e-12)
            self.maneuvers.append(Maneuver(
                b["phase"], b["body"], b["t0"], self.craft.t, b["dv"],
                b["pro"] / n, b["nor"] / n, b["rad"] / n))
        self._burn = None

    # --- routing ----------------------------------------------------------
    def _route(self):
        here, dest, cfg = self.craft.body, self.dest, self.cfg
        if dest is here:
            self.kind = "soi"
            self._final()
        elif dest.parent is here:
            self.kind = "moon"
            self._plan_moon()
        elif dest.parent is here.parent and here.parent is not None:
            self.kind = "planet"
            self._plan_planet()
        else:
            raise ValueError(f"no route from {here.name} to {dest.name} "
                             "(within an SOI, to a moon, or to a sibling)")

    def _capture_rp(self):
        d, tg = self.dest, self.final_target
        alt = tg.rp if tg.rp is not None else tg.ra
        return max(alt if alt is not None else d.radius + d.atmosphere
                   + self.cfg.SAFE_ALT_M,
                   d.radius + d.atmosphere + self.cfg.SAFE_ALT_M)

    def _capture_ra(self):
        """The capture ellipse's apoapsis: high (a cheap capture and a cheap
        plane change up there), but clear of the destination's moons --
        captured at 14000 km round Duna, lowering it fell into Ike's SOI.
        A moon has no moons: up to CAPTURE_AP_FRACTION of its SOI."""
        d, tg = self.dest, self.final_target
        ra = min(self.cfg.CAPTURE_AP_FRACTION * d.soi, self._ceiling(d))
        if tg.ra is not None:
            ra = max(ra, tg.ra)
        return max(ra, self._capture_rp())

    def _accel(self):
        return self.vessel.thrust_n / self.craft.m

    def _plan_moon(self):
        cfg, here, moon = self.cfg, self.craft.body, self.dest
        c = orbit.conic(self.craft.r, self.craft.v, here.mu)
        rc, ra_cap = self._capture_rp(), self._capture_ra()
        ra_wait, rp_wait, rc, ra_cap, ok = waiting_orbit(
            here, moon, c.rp, rc, ra_cap, self._accel(), cfg)
        if not ok:
            self.event("warning: no waiting orbit lets this engine capture "
                       "in one pass")
        elif rc > self._capture_rp() * 1.001:
            self.event(f"capture high and loose for this engine: "
                       f"{self._km(ra_cap, moon)} x {self._km(rc, moon)}")
        self.plan.update(rc=rc, ra_cap=ra_cap, rp_wait=rp_wait,
                         ra_wait=ra_wait)
        loose = cfg.CAPTURE_LOOSE_FRACTION * moon.soi
        self.plan["vcap"] = max(
            transfer.capturable_vinf(moon, self._accel(), rc, ra_cap,
                                     math.radians(cfg.CAPTURE_ARC_DEG),
                                     cfg.CAPTURE_FILL, loose),
            self._long_capturable())
        self.target = guidance.Target(ra_wait, rp_wait, moon.inc, moon.lan)
        self.set_phase("PUMP", f"waiting orbit {self._km(ra_wait, here)} x "
                               f"{self._km(rp_wait, here)}, in {moon.name}'s "
                               "plane")

    def _plan_planet(self):
        cfg, here, dest = self.cfg, self.craft.body, self.dest
        star = here.parent
        rp = max(orbit.conic(self.craft.r, self.craft.v, here.mu).rp,
                 here.radius + here.atmosphere + cfg.SAFE_ALT_M)
        ra = cfg.PARK_AP_FRACTION * self._park_ceiling()
        # how long the pump takes, to pick a window after it
        from lowThrust import sim as _sim
        est = _sim.fly(_SimBody(here), self.vessel, self.craft.r, self.craft.v,
                       guidance.Target(ra, rp), cfg.ETA_DEFAULT, cfg,
                       record=False)
        ready = self.craft.t + 1.3 * est.time + cfg.WINDOW_LEAD_S
        syn = 1 / abs(1 / here.period - 1 / dest.period)
        w = transfer.window(here, dest, ready, star.mu, syn)
        if w is None:
            raise ValueError("no Lambert window found")
        n_now = vec.unit(vec.cross(self.craft.r, self.craft.v))
        normal, apse, plane_dv = transfer.parking_orientation(
            here.mu, rp, ra, w.vinf_dep, n_now)
        rc, ra_cap = self._capture_rp(), self._capture_ra()
        cap = transfer.capture_orbit(dest, vec.norm(w.vinf_arr),
                                     self._accel(), rc, ra_cap,
                                     math.radians(cfg.CAPTURE_ARC_DEG),
                                     cfg.CAPTURE_FILL,
                                     max(ra_cap, self._ceiling(dest)))
        if cap is None:
            # Too long for one pass: a capture that starts inside the SOI
            # (where the destination alone pulls), or failing that MATCH,
            # which sheds the rest about the star in the cruise.  (Started
            # outside the SOI on a prediction about the planet alone, a
            # 0.04 m/s^2 capture 11 SOI from Moho was wrecked by the star's
            # tide.)
            self.plan.update(rc=rc, ra_cap=ra_cap)
            v_arr = vec.norm(w.vinf_arr)
            vmax = self._long_capturable()
            if v_arr <= vmax:
                tau, dv_cap, out = transfer.capture_start(
                    self._cap_spec(), rc, v_arr, self._accel())
                self.event(f"long capture: ~{dv_cap:.0f} m/s, starting "
                           f"{tau / 3600:.1f} h before periapsis, "
                           f"{out / 1e6:.1f} Mm out ({out / dest.soi:.1f} SOI)")
            else:
                arc = math.radians(cfg.CAPTURE_ARC_DEG)
                loose = max(ra_cap, 0.5 * dest.soi)
                v1 = transfer.capturable_vinf(dest, self._accel(), rc, ra_cap,
                                              arc, cfg.CAPTURE_FILL, loose)
                if v1 > vmax:
                    # a weak engine at a small planet: one pass, high and
                    # loose, beats a long capture into the tight ellipse
                    rc, ra_cap = transfer.capture_orbit(
                        dest, cfg.ARRIVE_MARGIN * v1, self._accel(), rc,
                        ra_cap, arc, cfg.CAPTURE_FILL, loose)
                vs = cfg.ARRIVE_MARGIN * max(v1, vmax)
                if vs <= 0:
                    self.abort = (f"this engine cannot capture at {dest.name}"
                                  " even arriving at rest")
                    return
                self.plan["match_vs"] = vs
                self.event(f"match {dest.name} in the cruise: shed "
                           f"{v_arr - vs:.0f} m/s about the star to arrive at "
                           f"its SOI at {vs:.0f} m/s, then capture into "
                           f"{self._km(ra_cap, dest)} x {self._km(rc, dest)}")
        elif cap[0] > rc * 1.001:
            rc, ra_cap = cap
            self.event(f"capture high and loose for this engine: "
                       f"{self._km(ra_cap, dest)} x {self._km(rc, dest)}")
        self.plan.update(window=w, rp_park=rp, ra_park=ra,
                         vinf=vec.norm(w.vinf_dep), rc=rc, ra_cap=ra_cap)
        if self.abort:
            return
        self.target = guidance.Target(ra, rp, apse=apse, normal=normal,
                                      inc=math.acos(normal[2]),
                                      hold_apse=True)
        self.event(f"window: depart {fmt_ut(w.t_dep)}, {w.tof / 21600:.0f} d "
                   f"flight, v_inf out {vec.norm(w.vinf_dep):.0f} m/s, in "
                   f"{vec.norm(w.vinf_arr):.0f} m/s")
        self.set_phase("PARK", f"ellipse {self._km(ra, here)} x "
                               f"{self._km(rp, here)} on the departure "
                               f"asymptote (plane change ~{plane_dv:.0f} m/s)")

    def _ceiling(self, body):
        """Highest apoapsis about ``body`` that stays clear of its moons and
        well inside its SOI."""
        top = 0.5 * body.soi
        for ch in body.children:
            top = min(top, ch.sma * (1 - ch.ecc)
                      - self.cfg.WAIT_SOI_MARGIN * ch.soi)
        return top

    def _park_ceiling(self):
        return self._ceiling(self.craft.body)

    @staticmethod
    def _km(r, body):
        return f"{(r - body.radius) / 1e3:.0f} km"

    # --- the loop -----------------------------------------------------------
    def run(self):
        cfg = self.cfg
        if self.abort:
            return self._end(False, self.abort)
        limit = self.t0 + cfg.MISSION_MAX_DAYS * 21600
        while True:
            cr = self.craft
            c = orbit.conic(cr.r, cr.v, cr.body.mu)
            floor = cr.body.radius + cr.body.atmosphere
            if c.r < floor:
                return self._end(False, f"hit {cr.body.name}'s "
                                        "atmosphere/surface")
            if (c.e < 1 and c.rp < floor
                    and self.phase in ("WAIT", "COAST", "ESCAPE")
                    and cr.body is not self.dest):
                self._repump("periapsis inside the atmosphere")
            if cr.t > limit:
                return self._end(False, "time limit")

            step = self._evade(c) or getattr(self, "_" + self.phase.lower())(c)
            if step == "done":
                if cr.body is not self.dest:
                    return self._end(False, f"finished about {cr.body.name}, "
                                            f"not {self.dest.name}")
                return self._end(True, "arrived")
            if isinstance(step, str):
                return self._end(False, step)
            throttle, direction, tick = step
            if throttle > 0:
                self._thrust(direction, throttle, tick)
            else:
                self._close_burn()
                hit = coast(cr, tick)
                if hit:
                    self._soi_change(*hit)

    def _end(self, done, reason):
        self._close_burn()
        cr = self.craft
        return MissionResult(done, reason, self.t0, cr.t, self.dv,
                             xenon_kg(self.vessel, self.dv),
                             self.maneuvers, self.events, cr.body,
                             orbit.conic(cr.r, cr.v, cr.body.mu))

    def _thrust(self, direction, throttle, tick):
        cr, ves, cfg = self.craft, self.vessel, self.cfg
        force = vec.scale(direction, ves.thrust_n * throttle)

        R, T, N = orbit.rtn(cr.r, cr.v)
        h = tick / cfg.THRUST_SUBSTEPS
        for _ in range(cfg.THRUST_SUBSTEPS):
            cr.r, cr.v, _ = _rk4(cr.r, cr.v, cr.m, cr.body.mu, force, 0.0, h)
        cr.t += tick
        dv = ves.thrust_n * throttle / cr.m * tick
        self.dv += dv
        self._last_dv = dv
        if self._burn is None or self._burn["phase"] != self.phase:
            self._close_burn()
            self._burn = dict(phase=self.phase, body=cr.body.name, t0=cr.t - tick,
                              dv=0.0, pro=0.0, nor=0.0, rad=0.0)
        b = self._burn
        b["dv"] += dv
        b["pro"] += dv * vec.dot(direction, T)
        b["nor"] += dv * vec.dot(direction, N)
        b["rad"] += dv * vec.dot(direction, R)
        hit = _crossing(cr, cr.r, cr.t)
        if hit:
            _switch(cr, *hit)
            self._soi_change(hit[0], hit[1] or cr.body)

    def _repump(self, why):
        """Back to the orbit-shaping phase after an unplanned flyby or a
        periapsis that dropped too low: the in-SOI law repairs the orbit."""
        self.plan.pop("pass", None)
        self.plan.pop("horizon", None)
        self.plan.pop("next_fix", None)
        if self.kind == "moon":
            self.target = guidance.Target(self.plan["ra_wait"],
                                          self.plan["rp_wait"], self.dest.inc,
                                          self.dest.lan)
            self.set_phase("PUMP", why + ": re-pumping")
        elif self.kind == "planet":
            self.target = self.target._replace()
            self.set_phase("PARK", why + ": re-parking")

    def _soi_change(self, kind, body):
        cr = self.craft
        self.event(f"SOI {'exit' if kind == 'exit' else 'entry'}: now "
                   f"orbiting {cr.body.name}")
        if (kind == "exit" and self.phase in ("COAST", "WAIT", "PUSH")
                and cr.body is not self.dest and cr.body is self._home):
            self._repump(f"after a {body.name if body else ''} flyby")
            return
        if cr.body is self.dest and self.phase in ("COAST", "CRUISE", "MATCH",
                                                   "ESCAPE", "PUSH", "WAIT",
                                                   "PUMP", "TRIM", "PARK"):
            self.set_phase("APPROACH", f"periapsis target "
                                       f"{self._km(self.plan['rc'], cr.body)}")
        elif (kind == "exit" and self.kind == "planet"
              and cr.body is self.dest.parent
              and self.phase in ("ESCAPE", "WAIT", "DEPART", "PARK", "TRIM")):
            # A weak engine's last departure pass can end bound with its
            # apoapsis past the SOI; it leaves anyway, and the cruise's
            # Lambert re-target flies the rest about the star.  (Left in
            # WAIT, it went on to "depart" from the Sun's periapsis.)
            self.set_phase("CRUISE", f"to {self.dest.name}")

    # --- tick sizes -----------------------------------------------------------
    def _tick(self, c):
        cfg, cr = self.cfg, self.craft
        if c.e < 1:
            el = orbit.elements(cr.r, cr.v, cr.body.mu)
            t = tick_for(el, cfg)
        else:
            t = c.r / max(vec.norm(cr.v), 1e-3) * 2 * math.pi / cfg.TICKS_PER_ORBIT
        return max(1.0, min(t, _boundary_tick(cr)))

    # --- phases -----------------------------------------------------------------
    def _soi_law(self, c, next_phase):
        cr, cfg = self.craft, self.cfg
        home = self.dest if self.phase == "FINAL" else self._home
        if cr.body is not home:
            if cr.body is home.parent:
                return f"thrown out of {home.name}'s SOI by a flyby"
            return 0.0, None, self._tick(c)   # an unplanned flyby: coast it
        if self._burn is None and c.e < 1:
            trim = self._trim_check(c)
            if trim is not None:
                self.plan["trim_left"] = abs(trim)
                self.plan["trim_sign"] = 1.0 if trim > 0 else -1.0
                self.plan["resume"] = self.phase
                self.set_phase("TRIM", f"{trim:+.1f} m/s at periapsis to miss "
                                       "a moon")
                return self._trim(c)
        el = orbit.elements(cr.r, cr.v, cr.body.mu)
        if el is None or el.ra > cfg.REBIND_SOI_FRACTION * cr.body.soi:
            # a flyby flung us out: brake until bound well inside the SOI
            # (bound alone is not enough -- an apoapsis past the SOI leaves
            # it all the same), then carry on
            if self.plan.get("rebinding") is not True:
                self.plan["rebinding"] = True
                self.event(f"unbound from {cr.body.name} after a flyby: "
                           "braking")
            tick = self._tick(c)
            return 1.0, vec.scale(vec.unit(cr.v), -1.0), tick
        self.plan["rebinding"] = False
        coasting = self._burn is None
        self.target = guidance.lock(self.target, el, cfg, coasting)
        tick = self._tick(c)
        cmd = guidance.command(el, self.target, self._accel(), tick,
                               cfg.ETA_DEFAULT, cfg, cr.body.mu)
        if cmd.done:
            return next_phase()
        if cmd.throttle <= 0 or cr.t < self.plan.get("blocked_until", -1):
            return 0.0, None, tick
        _, T, N = orbit.rtn(cr.r, cr.v)
        d = vec.unit(vec.add(vec.scale(T, cmd.prograde),
                             vec.scale(N, cmd.normal)))
        if self._burn is None:
            self._guard_pass(c, d, el, cmd.need)
        cap = self.plan.get("cap")
        if cap is not None and self._burn is not None \
                and self._burn["dv"] >= cap:
            self.plan["blocked_until"] = self.plan["arc_end"]
            self.plan["cap"] = None
            return 0.0, None, tick
        return cmd.throttle, d, tick

    def _others(self):
        """The bodies a coast about the current body must stay clear of."""
        return [b for b in self.craft.body.children if b is not self.dest
                and self.phase != "FINAL" or b is not self.dest
                and self.phase == "FINAL"]

    def _guard_pass(self, c, d, el, need):
        """First tick of an arc burn: cap this pass's dv so the next
        revolution stays out of every other moon's SOI.  A pump out past a
        moon crosses its orbit twice a revolution for dozens of revolutions,
        and an encounter was all but certain without this."""
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        others = self._others()
        self.plan["cap"] = None
        arc_s = 2 * math.acos(cfg.ETA_DEFAULT) * el.r * el.r / el.h
        self.plan["arc_end"] = cr.t + arc_s
        if not others:
            return
        full = self._accel() * min(arc_s, need)
        for f in (1.0, 0.75, 0.5, 0.25, 0.1, 0.0, 1.5, 2.0):
            v1 = vec.add(cr.v, vec.scale(d, f * full))
            c1 = orbit.conic(cr.r, v1, mu)
            if c1.e >= 1:
                continue
            if transfer.first_hazard(cr.r, v1, cr.t, mu, others,
                                     cr.t + 1.05 * c1.period) is None:
                if f != 1.0:
                    self.plan["cap"] = f * full
                    self.event(f"burn capped at {f * full:.1f} m/s: the "
                               "full pass would meet a moon next revolution")
                return
        self.event("warning: no burn this pass avoids a moon next revolution")

    def _trim_check(self, c):
        """Once a revolution while waiting: if the coming revolution meets
        another moon, a small phasing burn at periapsis (the smallest of
        +-0.5..8 m/s that clears the next two revolutions)."""
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        others = self._others()
        if not others:
            return None
        t_pe = orbit.time_to_periapsis(c, mu)
        key = round(cr.t + t_pe)
        if self.plan.get("trim_checked") == key:
            return None
        self.plan["trim_checked"] = key
        rp_, vp_ = orbit.propagate(cr.r, cr.v, mu, t_pe)
        t1 = cr.t + t_pe
        if transfer.first_hazard(rp_, vp_, t1, mu, others,
                                 t1 + 1.05 * c.period) is None:
            return None
        u = vec.unit(vp_)
        for dv in (0.5, -0.5, 1.0, -1.0, 2.0, -2.0, 4.0, -4.0, 8.0, -8.0):
            v1 = vec.add(vp_, vec.scale(u, dv))
            c1 = orbit.conic(rp_, v1, mu)
            if c1.e >= 1:
                continue
            if transfer.first_hazard(rp_, v1, t1, mu, others,
                                     t1 + 2.05 * c1.period) is None:
                return dv
        self.event("warning: no phasing trim clears the moons")
        return None

    def _final(self, c=None):
        if c is None:
            self.target = self.final_target
            self.set_phase("FINAL", "to the target orbit")
            return None
        return self._soi_law(c, lambda: "done")

    def _pump(self, c):
        def nxt():
            self.set_phase("WAIT", "for the pass whose push meets "
                                   f"{self.dest.name}")
            return 0.0, None, self._tick(c)
        return self._soi_law(c, nxt)

    def _park(self, c):
        def nxt():
            self.set_phase("WAIT", f"for the window, "
                                   f"{fmt_ut(self.plan['window'].t_dep)}")
            return 0.0, None, self._tick(c)
        return self._soi_law(c, nxt)

    def _wait(self, c):
        trim = self._trim_check(c) if c.e < 1 else None
        if trim is not None:
            self.plan["trim_left"] = abs(trim)
            self.plan["trim_sign"] = 1.0 if trim > 0 else -1.0
            self.plan["resume"] = "WAIT"
            self.set_phase("TRIM", f"{trim:+.1f} m/s at periapsis to miss a "
                                   "moon")
            return self._trim(c)
        return self._wait_moon(c) if self.kind == "moon" else self._wait_planet(c)

    def _trim(self, c):
        """A small prograde/retrograde burn centred on the next periapsis."""
        cr, cfg = self.craft, self.cfg
        tick = self._tick(c)
        left = self.plan["trim_left"] - self._last_dv
        self._last_dv = 0.0
        self.plan["trim_left"] = left
        if left <= 0.01:
            self.set_phase(self.plan["resume"], "trim done")
            return 0.0, None, tick
        t_pe = orbit.time_to_periapsis(c, cr.body.mu)
        if self._burn is None and t_pe > left / self._accel() / 2 \
                and t_pe < 0.9 * c.period:
            return self._hold(c, t_pe - left / self._accel() / 2)
        return (min(1.0, left / (self._accel() * tick)),
                vec.scale(vec.unit(cr.v), self.plan["trim_sign"]), tick)

    def _horizon(self, r, v):
        """How far ahead a moon encounter is looked for: within the first
        revolution only -- a direct transfer.  Over a whole period the push
        found second-apoapsis intercepts, hypersensitive, and its re-solves
        jumped between roots (68 -> 96 -> 53 m/s) and missed."""
        c = orbit.conic(r, v, self.craft.body.mu)
        return min(self.cfg.PUSH_REVOLUTION * c.period if c.e < 1 else 1e7,
                   self.cfg.PUSH_HORIZON_S)

    def _hold(self, c, dt):
        """Coast ``dt``: in one step when the orbit cannot reach an SOI
        boundary, else no further than a tick."""
        if not _safe_until(self.craft, c):
            dt = min(dt, self._tick(c))
        return 0.0, None, max(1.0, dt)

    def _wait_moon(self, c):
        cr, cfg, moon = self.craft, self.cfg, self.dest
        mu = cr.body.mu
        t_pe = orbit.time_to_periapsis(c, mu)
        accel = self._accel()
        # one decision per pass, made a little ahead of it
        lead = min(0.25 * c.period, cfg.PUSH_DV_MAX / accel)
        if t_pe > lead:
            return self._hold(c, t_pe - lead)
        key = round(cr.t + t_pe)
        if self.plan.get("pass") != key:
            self.plan["pass"] = key
            rp_, vp_ = orbit.propagate(cr.r, cr.v, mu, t_pe)
            vinf = transfer.moon_arrival_vinf(mu, c.rp, moon.sma,
                                              math.sqrt(mu / moon.sma))
            self.plan["b"] = transfer.impact_parameter(
                self.plan["rc"], max(vinf, 1.0), moon.mu)
            dv = transfer.push_dv(
                rp_, vp_, cr.t + t_pe, mu, moon, self.plan["b"], moon.normal,
                cfg.PUSH_DV_MAX, self._horizon)
            if dv is not None:
                # reject a pass whose coast meets another moon first
                v1 = vec.add(vp_, vec.scale(vec.unit(vp_), dv))
                ca = transfer.closest_approach(rp_, v1, cr.t + t_pe, mu, moon,
                                               self._horizon(rp_, v1))
                others = [b for b in cr.body.children if b is not moon]
                bad = transfer.first_hazard(rp_, v1, cr.t + t_pe, mu, others,
                                            ca.t)
                if bad is not None:
                    self.event(f"pass skipped: its coast would meet {bad.name}")
                    dv = None
                elif vec.norm(ca.vrel) > self.plan["vcap"] * (
                        1.25 ** self.plan.get("fast_skips", 0)):
                    # a fast intercept (354 m/s to Minmus where ~100 would
                    # do) arrived at 787 m/s, past what the capture sheds
                    # the bar rises 25% a skip, so the wait is bounded --
                    # the predictor undersells a small moon's own pull, and
                    # held at 80 m/s a 0.02 m/s^2 Mun transfer skipped
                    # 300-500 m/s passes for ten years
                    self.event(f"pass skipped: it arrives at "
                               f"{vec.norm(ca.vrel):.0f} m/s, faster than "
                               f"the {self.plan['vcap']:.0f} this engine "
                               "captures")
                    self.plan["fast_skips"] = \
                        self.plan.get("fast_skips", 0) + 1
                    dv = None
            self.plan["push"] = dv
        dv = self.plan.get("push")
        if dv is None:                       # not this pass
            return self._hold(c, t_pe + 0.05 * c.period)
        if t_pe <= dv / accel / 2:
            self.set_phase("PUSH", f"{dv:.1f} m/s prograde at periapsis")
            self.plan["push_left"] = dv
            return self._push(c)
        return self._hold(c, t_pe - dv / accel / 2)

    def _push(self, c):
        """Burn prograde until the push is spent.  The ledger is the planned
        dv less what has been delivered; a re-solve from the current state,
        searched round the ledger so it stays on the same root, refines it
        when it finds the encounter and is ignored when it does not.  The
        fine aim is COAST's.  (Steering on the B-plane correction inside the
        push flickered with the SOI test and oscillated the apoapsis 350 km
        a tick; taking a failed re-solve as "done" ended a Minmus push before
        it had burned.)"""
        cr, cfg, moon = self.craft, self.cfg, self.dest
        mu = cr.body.mu
        tick = self._tick(c)
        left = self.plan["push_left"] - self._last_dv
        # stay on the same root: search round the ledger, not from zero
        dv = transfer.push_dv(cr.r, cr.v, cr.t, mu, moon, self.plan["b"],
                              moon.normal, min(cfg.PUSH_DV_MAX,
                                               1.5 * left + 2),
                              self._horizon, n=12, dv_min=max(0.0, 0.5 * left
                                                              - 2))
        if dv is not None:
            left = dv
        self.plan["push_left"] = left
        self._last_dv = 0.0
        if left < cfg.CORRECT_DV:
            self.set_phase("COAST", f"to {moon.name}")
            return 0.0, None, tick
        return min(1.0, left / (self._accel() * tick)), vec.unit(cr.v), tick

    def _wait_planet(self, c):
        cr = self.craft
        mu = cr.body.mu
        w = self.plan["window"]
        t_pe = orbit.time_to_periapsis(c, mu)
        vinf = self.plan["vinf"]
        need = (math.sqrt(vinf ** 2 + 2 * mu / c.rp)
                - math.sqrt(mu * (2 / c.rp - 1 / c.a))) / self._accel()
        t_go = self.plan.get("t_go", w.t_dep)
        if cr.t + t_pe < t_go - c.period / 2:
            # not yet the pass nearest the window: skip ahead
            return self._hold(c, max(t_pe + 1.0, t_go - 1.5 * c.period - cr.t))
        key = round(cr.t + t_pe)
        if self.plan.get("checked") != key:
            self.plan["checked"] = key
            # the pass as DEPART will fly it, not an impulse at periapsis:
            # checked at the window's v_inf, a weak escape that reached only
            # 229 m/s met the Mun and fell back into Kerbin
            r1, v1, t1 = self._predict_depart(max(0.0, t_pe - need / 2))
            bad = transfer.first_hazard(r1, v1, t1, mu, cr.body.children,
                                        t1 + self.cfg.ESCAPE_CHECK_S)
            if bad is not None:
                self.event(f"departure pass skipped: its escape would meet "
                           f"{bad.name}")
                self.plan["t_go"] = cr.t + t_pe + c.period
                return self._hold(c, t_pe + 1.0)
        if t_pe <= need / 2:
            self.set_phase("DEPART", f"{need * self._accel():.0f} m/s at "
                                     "periapsis")
            return self._depart(c)
        return self._hold(c, t_pe - need / 2)

    def _predict_depart(self, wait):
        """(r, v, t) where a DEPART burn starting ``wait`` s from now ends
        -- the same law, flown on RK4 about the current body."""
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        r, v = orbit.propagate(cr.r, cr.v, mu, wait)
        t = cr.t + wait
        accel = self._accel()
        arc = math.radians(cfg.DEPART_ARC_DEG)
        for _ in range(100000):
            c = orbit.conic(r, v, mu)
            rem = math.sqrt(self.plan["vinf"] ** 2 + 2 * mu / c.r) - vec.norm(v)
            nu = c.nu if c.nu <= math.pi else c.nu - 2 * math.pi
            if rem <= cfg.CORRECT_DV or nu > arc:
                break
            h = max(1.0, min(rem / accel,
                             0.005 * c.r / max(vec.norm(v), 1.0)))
            r, v, _ = _rk4(r, v, 1.0, mu, vec.scale(vec.unit(v), accel), 0.0, h)
            t += h
        return r, v, t

    def _depart(self, c):
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        tick = self._tick(c)
        rem = math.sqrt(self.plan["vinf"] ** 2 + 2 * mu / c.r) - vec.norm(cr.v)
        # signed: started long before periapsis (a weak push to Jool, 61000
        # s), an unsigned anomaly read ~300 deg and ended it every minute
        nu = c.nu if c.nu <= math.pi else c.nu - 2 * math.pi
        if rem <= cfg.CORRECT_DV or nu > math.radians(cfg.DEPART_ARC_DEG):
            if c.e < 1:
                # A weak engine cannot escape in one pass: every pass raises
                # the energy until one escapes; the cruise corrections then
                # build what v_inf is missing.  (Ending the push bound once
                # left a 0.04 m/s^2 craft coasting into the Mun.)
                self.plan["t_go"] = cr.t
                self.plan.pop("checked", None)
                self.set_phase("WAIT", f"still bound after this pass "
                                       f"(apoapsis {self._km(c.ra, cr.body)}):"
                                       " another")
                return 0.0, None, tick
            self.set_phase("ESCAPE", f"v_inf {c.vinf:.0f} m/s "
                                     f"(window {self.plan['vinf']:.0f})")
            return 0.0, None, tick
        return min(1.0, rem / (self._accel() * tick)), vec.unit(cr.v), tick

    def _escape(self, c):
        return 0.0, None, self._tick(c)

    def _correct(self, c, frame):
        """B-plane correction toward the destination (COAST and CRUISE)."""
        cr, cfg, dest = self.craft, self.cfg, self.dest
        tick = self._tick(c)
        due, dt = self._capture_due(tick)
        if due:
            return self._start_capture(c)
        tick = min(tick, dt)
        if "match_vs" in self.plan and cr.body is frame:
            go, dt = self._match_due(tick)
            if go:
                return self._match(c)
            tick = min(tick, dt)
        if cr.body is not frame:
            return 0.0, None, tick           # an unplanned flyby: just coast
        due = self.plan.get("next_fix", -math.inf)
        if self._burn is None and cr.t < due:
            return 0.0, None, min(tick, due - cr.t)
        if self.kind == "planet":
            big = self._retarget(c, tick)
            if big is not None:
                return big
        vinf_guess = vec.norm(self.plan.get("vrel", (1e3, 0, 0)))
        b = transfer.impact_parameter(self.plan["rc"], max(vinf_guess, 1.0),
                                      dest.mu)
        horizon = self.plan.get("horizon")
        if horizon is None:
            if self.kind == "planet":
                w = self.plan["window"]
                horizon = 1.5 * max(self.plan.get("arrive_t", w.t_dep + w.tof)
                                    - cr.t, 86400.0)
            else:
                horizon = self._horizon(cr.r, cr.v)
        # round the last encounter or the re-target's arrival: searched
        # over the whole multi-year horizon (a sample a week), an Eeloo
        # cruise found the wrong minimum, read a 2 Gm miss, re-targeted,
        # and slid its arrival 1500 m/s faster before the capture failed
        guess = self.plan.get("arrive_t") if self.kind == "planet" else None
        dv, ca = transfer.correction(cr.r, cr.v, cr.t, cr.body.mu, dest,
                                     horizon, b, dest.normal, guess=guess)
        if guess is not None:
            self.plan["arrive_t"] = ca.t
        self.plan["vrel"], self.plan["ca_t"] = ca.vrel, ca.t
        self.plan["horizon"] = max(ca.t - cr.t, 0) * 1.5 + 3600
        mag = vec.norm(dv)
        to_ca = ca.t - cr.t
        if self.kind == "planet" and mag > cfg.CORRECT_MAX_DV:
            big = self._retarget(c, tick, force=True)
            if big is not None:
                return big
        if self.kind == "moon" and (vec.norm(ca.d) > cfg.MISS_REPLAN * dest.soi
                                    or mag > cfg.CORRECT_MAX_DV):
            # no real encounter ahead: a linear correction means nothing
            # this far out.  Plan a new push at the next periapsis.
            if orbit.conic(cr.r, cr.v, cr.body.mu).e < 1:
                self.plan.pop("pass", None)
                self.plan.pop("horizon", None)
                self.set_phase("WAIT", f"no encounter ({vec.norm(ca.d) / 1e3:.0f}"
                                       f" km from {dest.name}): a new push")
                return 0.0, None, tick
        if mag < cfg.CORRECT_DV or to_ca < cfg.CORRECT_MIN_LEAD_S:
            self.plan["next_fix"] = cr.t + max(to_ca * cfg.CORRECT_EVERY, 600)
            return 0.0, None, tick
        return min(1.0, mag / (self._accel() * tick)), vec.unit(dv), tick

    def _retarget(self, c, tick, force=False):
        """A miss far too big for a linear correction: fly a fresh Lambert
        transfer instead, its arrival time chosen once (the cheapest over
        0.6-1.6 of the planned time left) and held while burning.  "Too big"
        is RETARGET_MISS of the target's SOI or RETARGET_FRACTION of its
        distance from the star, whichever is less -- Jool's SOI is so large
        that 20 of them let a 40 Gm miss through to the linear correction,
        which then asked thousands of m/s."""
        cr, cfg, dest = self.craft, self.cfg, self.dest
        mu = cr.body.mu
        n = vec.unit(vec.cross(cr.r, cr.v))
        ta = self.plan.get("lambert_t")
        if ta is None:
            w = self.plan["window"]
            # from the arrival now planned, not the window's: an Eeloo
            # cruise re-targeted to arrive 170 days after its window read a
            # phantom 2 Gm miss off a horizon that had shrunk to 16 days
            left = max(self.plan.get("arrive_t", w.t_dep + w.tof) - cr.t,
                       10 * 86400.0)
            ca = transfer.closest_approach(cr.r, cr.v, cr.t, mu, dest,
                                           1.6 * left)
            limit = min(cfg.RETARGET_MISS * dest.soi,
                        cfg.RETARGET_FRACTION * dest.sma)
            if not force and vec.norm(ca.d) < limit:
                return None
            got = transfer.retarget(cr.r, cr.v, cr.t, mu, dest,
                                    cr.t + 0.6 * left, cr.t + 1.6 * left, n,
                                    rc=self.plan["rc"])
            if got is None:
                return None
            self.plan["lambert_t"] = self.plan["arrive_t"] = got[1]
            self.plan.pop("horizon", None)
            self.event(f"re-targeted: {vec.norm(got[0]):.0f} m/s onto a "
                       f"Lambert arc arriving {fmt_ut(got[1])} (the miss was "
                       f"{vec.norm(ca.d) / 1e9:.1f} Gm)")
            ta = got[1]
        dv = transfer.lambert_dv(cr.r, cr.v, cr.t, mu, dest, ta, n,
                                 rc=self.plan["rc"])
        if dv is None or vec.norm(dv) < cfg.CORRECT_DV:
            self.plan.pop("lambert_t", None)
            self.plan["next_fix"] = cr.t + 600
            return None
        return min(1.0, vec.norm(dv) / (self._accel() * tick)), vec.unit(dv), tick

    def _coast(self, c):
        return self._correct(c, self.dest.parent)

    def _cruise(self, c):
        return self._correct(c, self.dest.parent)

    def _approach(self, c):
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        tick = self._tick(c)
        if cr.body is not self.dest:
            return 0.0, None, tick           # an unplanned flyby: coast it
        t_pe = orbit.time_to_periapsis(c, mu)
        due, dt = self._capture_due(tick)
        if due or t_pe < 0:
            return self._start_capture(c)
        tick = min(tick, dt)
        need = transfer.capture_need(c, mu, self.plan["ra_cap"])
        err = c.rp - self.plan["rc"]
        if abs(err) < cfg.APPROACH_TOL * self.plan["rc"] or \
                t_pe < cfg.APPROACH_MIN_LEAD_S:
            return 0.0, None, min(tick, max(1.0, t_pe - need / self._accel() / 2))
        g = self._rp_gradient(c)
        gg = sum(x * x for x in g)
        if gg == 0:
            return 0.0, None, tick
        dv = tuple(-err * x / gg for x in g)
        mag = vec.norm(dv)
        if mag < cfg.CORRECT_DV:
            return 0.0, None, tick
        return min(1.0, mag / (self._accel() * tick)), vec.unit(dv), tick

    def _evade(self, c):
        """Inside an unplanned body's SOI with the periapsis below a safe
        height: burn along the periapsis gradient until it is clear.  (A
        weak pump that could not dodge the Mun by timing hit it.)"""
        cr, cfg = self.craft, self.cfg
        b = cr.body
        if b is self.dest or b is self._home or b is self.dest.parent:
            return None
        floor = b.radius + b.atmosphere + cfg.SAFE_ALT_M
        if c.rp >= floor or orbit.time_to_periapsis(c, b.mu) < 0:
            return None
        g = self._rp_gradient(c)
        gg = vec.dot(g, g)
        if gg == 0:
            return None
        if self.plan.get("evading") is not b.name:
            self.plan["evading"] = b.name
            self.event(f"evasive burn: periapsis {self._km(c.rp, b)} over "
                       f"{b.name}")
        tick = self._tick(c)
        want = vec.scale(g, (floor * 1.3 - c.rp) / gg)
        return (min(1.0, vec.norm(want) / (self._accel() * tick)),
                vec.unit(want), tick)

    # --- match: shed the arrival speed about the star ----------------------
    def _match_aim(self, u):
        """MATCH's end state relative to the destination, for an arrival
        along ``u``: just outside its SOI, moving ``match_vs`` toward it with
        the impact parameter of the capture periapsis, on the B-plane side
        the cruise aims for."""
        d, vs = self.dest, self.plan["match_vs"]
        w = vec.unit(vec.cross(u, d.normal))
        b = min(transfer.impact_parameter(self.plan["rc"], max(vs, 1.0), d.mu),
                0.8 * d.soi)
        back = math.sqrt(max((self.cfg.MATCH_STANDOFF * d.soi) ** 2 - b * b,
                             0.0))
        return (vec.add(vec.scale(u, -back), vec.scale(w, b)),
                vec.scale(u, vs), back)

    def _match_encounter(self):
        """The coast's closest approach to the destination, searched round
        the last one."""
        cr = self.craft
        ca = transfer.closest_approach(
            cr.r, cr.v, cr.t, cr.body.mu, self.dest,
            self.plan.get("horizon", 1.5 * max(self.plan["ca_t"] - cr.t,
                                               86400.0)),
            guess=self.plan["ca_t"])
        self.plan["ca_t"], self.plan["vrel"] = ca.t, ca.vrel
        return ca

    def _match_lead(self, v):
        """Seconds before the coast's closest approach that braking from
        arrival speed ``v`` to ``match_vs`` at MATCH_SHARE of the engine
        must start, to end at the aim point a standoff short of it."""
        vs = self.plan["match_vs"]
        a = self.cfg.MATCH_SHARE * self._accel()
        back = self.cfg.MATCH_STANDOFF * self.dest.soi
        return ((v * v - vs * vs) / (2 * a) + back) / v

    def _match_due(self, tick):
        """(whether MATCH must start, the longest tick that cannot pass its
        start), from the predicted encounter -- far out the instantaneous
        relative velocity means nothing (read as 20 km/s, it started MATCH
        19000 Mm from Moho)."""
        cr = self.craft
        if self.plan.get("match_done") or "ca_t" not in self.plan:
            return False, tick
        v = vec.norm(self.plan["vrel"])
        if v <= self.plan["match_vs"]:
            return False, tick
        lead = self._match_lead(v)
        left = self.plan["ca_t"] - cr.t
        if left > 1.5 * lead:
            return False, min(tick, max(60.0, 0.5 * (left - 1.5 * lead)))
        ca = self._match_encounter()
        v = vec.norm(ca.vrel)
        lead, left = self._match_lead(v), ca.t - cr.t
        if left <= lead:
            return True, tick
        return False, min(tick, max(60.0, 0.5 * (left - lead)))

    def _match(self, c):
        """Zero-effort-miss / zero-effort-velocity guidance about the star
        onto the aim point: a = 6 ZEM/t^2 - 2 ZEV/t, both predicted by
        coasting the vessel and the destination ``t`` ahead, with ``t`` the
        time a constant deceleration from the predicted arrival speed takes
        to reach the aim point.  A weak engine cannot brake near the planet
        (0.04 m/s^2 would have to start 270 Mm from Moho, where the star's
        tide breaks a prediction about the planet alone); about the star
        the planet's motion is modelled exactly."""
        cr, cfg, d = self.craft, self.cfg, self.dest
        tick = self._tick(c)
        if cr.body is not d.parent:
            return 0.0, None, tick
        ca = self._match_encounter()
        v = vec.norm(ca.vrel)
        if self.phase != "MATCH":
            self.set_phase("MATCH", f"{v:.0f} m/s to shed about "
                                    f"{d.parent.name}, "
                                    f"{(ca.t - cr.t) / 3600:.0f} h out")
        u = vec.unit(ca.vrel)
        P, V, back = self._match_aim(u)
        vs = self.plan["match_vs"]
        t_go = 2 * max(v * (ca.t - cr.t) - back, 0.0) / (v + vs)
        if v <= 1.05 * vs or t_go < cfg.MATCH_END_S:
            r, vr = self._dest_rel()
            self.plan["match_done"] = True
            self.plan.pop("next_fix", None)
            self.set_phase("CRUISE", f"matched: arriving at {v:.0f} m/s, "
                                     f"{vec.norm(r) / 1e6:.1f} Mm from "
                                     f"{d.name}")
            return 0.0, None, tick
        tick = min(tick, max(1.0, 0.02 * t_go))
        mu = cr.body.mu
        rc_, vc_ = orbit.propagate(cr.r, cr.v, mu, t_go)
        rd, vd = d.state(cr.t + t_go)
        zem = vec.sub(vec.add(rd, P), rc_)
        zev = vec.sub(vec.add(vd, V), vc_)
        acc = vec.sub(vec.scale(zem, 6 / t_go ** 2), vec.scale(zev, 2 / t_go))
        return min(1.0, vec.norm(acc) / self._accel()), vec.unit(acc), tick

    # --- capture ----------------------------------------------------------
    def _dest_rel(self):
        """(r, v) relative to the destination, from its SOI or its
        parent's; None from anywhere else."""
        cr, d = self.craft, self.dest
        if cr.body is d:
            return cr.r, cr.v
        if cr.body is d.parent:
            rb, vb = d.state(cr.t)
            return vec.sub(cr.r, rb), vec.sub(cr.v, vb)
        return None

    def _cap_spec(self):
        cfg, d = self.cfg, self.dest
        ra_cap = self.plan["ra_cap"]
        floor = d.radius + d.atmosphere + cfg.SAFE_ALT_M
        return transfer.Capture(
            d.mu, ra_cap, max(ra_cap, self._ceiling(d)), floor,
            max(cfg.CAPTURE_RP_BAND * self.plan["rc"], 1.2 * floor),
            d.radius + d.atmosphere, math.radians(cfg.CAPTURE_ARC_DEG),
            cfg.CAPTURE_FILL, cfg.CAPTURE_PATH_SHARE)

    def _long_capturable(self):
        """The fastest arrival a capture starting inside the SOI handles,
        with ARRIVE_MARGIN to spare in its start distance."""
        d = self.dest
        cap = self._cap_spec()

        def ok(vinf):
            got = transfer.capture_start(cap, self.plan["rc"], vinf,
                                         self._accel())
            return got is not None and got[2] <= self.cfg.ARRIVE_MARGIN * d.soi
        lo, hi = 0.0, 8000.0
        if ok(hi):
            return hi
        for _ in range(16):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if ok(mid) else (lo, mid)
        return lo

    def _capture_decide(self, tick, started):
        """The capture law's decision now (``transfer.capture_step``):
        ('burn', tick) or ('coast', dt).  Before the capture has started it
        is looked for only on an inbound pass within reach that aims below
        the loosest capture apoapsis."""
        cr = self.craft
        rel = self._dest_rel()
        if rel is None:
            return "coast", tick
        r, v = rel
        cap = self._cap_spec()
        cc = orbit.conic(r, v, cap.mu)
        if not started and cr.body is not self.dest:
            return "coast", tick
        rho, wait = transfer.capture_step(r, v, cc, cap, self._accel(),
                                          started)
        if rho <= 0:
            return "coast", min(tick, max(1.0, wait))
        return "burn", rho

    def _capture_due(self, tick):
        """(whether the capture must start now, the longest tick that cannot
        pass its start)."""
        what, x = self._capture_decide(tick, started=False)
        return (True, tick) if what == "burn" else (False, x)

    def _start_capture(self, c):
        rel = self._dest_rel()
        cap = self._cap_spec()
        cc = orbit.conic(*rel, cap.mu)
        out = vec.norm(rel[0])
        where = "" if self.craft.body is self.dest else \
            f", {out / 1e6:.1f} Mm out ({out / self.dest.soi:.1f} SOI)"
        self.set_phase("CAPTURE", f"{transfer.capture_need(cc, cap.mu, cap.ra_cap):.0f}"
                                  f" m/s at periapsis, starting "
                                  f"{orbit.time_to_periapsis(cc, cap.mu) / 3600:.1f}"
                                  f" h before it{where}")
        return self._capture(c)

    def _capture(self, c):
        cr = self.craft
        rel = self._dest_rel()
        if rel is None:
            # inside another body's SOI (a moon crossed mid-capture): coast
            # it -- the evasive burn guards the periapsis -- and re-decide
            # on the way out
            return 0.0, None, self._tick(c)
        r, v = rel
        cap = self._cap_spec()
        cc = orbit.conic(r, v, cap.mu)
        tick = min(self._tick(c), max(1.0, cc.r / max(vec.norm(v), 1e-3) * 2
                                      * math.pi / self.cfg.TICKS_PER_ORBIT),
                   transfer.capture_step_s(v, self._accel()))
        if cr.body is self.dest and transfer.captured(r, v, cc, cap):
            self.target = self.final_target
            self.set_phase("FINAL", f"captured, {self._km(cc.ra, cr.body)} x "
                                    f"{self._km(cc.rp, cr.body)}")
            return 0.0, None, tick
        if transfer.capture_lost(r, v, cc, cap):
            return "capture failed: the engine cannot shed the excess speed " \
                   "in one periapsis pass"
        what, x = self._capture_decide(tick, started=True)
        if what == "coast":
            return 0.0, None, x
        full = self._accel() * tick
        want = transfer.capture_command(r, v, cc, cap, x * full)
        return min(1.0, vec.norm(want) / full), vec.unit(want), tick

    def _rp_gradient(self, c):
        """d(periapsis radius)/d(impulse), numerically, inertial vector."""
        cr = self.craft
        return transfer.rp_gradient(cr.r, cr.v, cr.body.mu)[1]


class _SimBody:
    """Adapter: a ``bodies.Body`` as ``sim.fly``'s body."""
    def __init__(self, b):
        self.name, self.mu, self.radius = b.name, b.mu, b.radius
        self.atmosphere = b.atmosphere


def fmt_ut(t):
    """UT as 'y1 d230 03:12' in Kerbin years (426 d) and 6 h days."""
    day = 21600
    year = 426 * day
    y, rem = divmod(t, year)
    d, rem = divmod(rem, day)
    h, rem = divmod(rem, 3600)
    return f"y{int(y) + 1} d{int(d) + 1} {int(h):02d}:{int(rem // 60):02d}"


def waiting_orbit(here, moon, rp_now, rc_min, ra_cap, accel, cfg):
    """A moon transfer's waiting orbit about ``here`` and the capture it
    allows: (apoapsis, periapsis, capture periapsis, capture apoapsis,
    whether one-pass capture is possible).

    The apoapsis well short of the moon: the push (at periapsis, so no dearer
    than pumping further) then reaches it over a wide range of phases, and the
    shorter period offers more passes -- just short of it, a Minmus transfer
    waited 224 days for a pass that lined up.  The periapsis as low as the
    current one allows (cheapest), raised only as far as the engine needs to
    capture at the moon in one pass -- a higher waiting periapsis arrives
    slower relative to the moon -- and never inside another moon's band (a
    weak engine's 11988 km periapsis sat on the Mun's orbit).  The capture
    periapsis is the lowest that pass allows, at least ``rc_min``.

    With another moon orbiting inside the target's, the whole waiting orbit
    stays below that moon's band, its periapsis raised until the push across
    the band fits in one pass: a weak pump out past the Mun crossed its
    orbit twice a revolution for dozens of revolutions, met it, and was
    flung into Kerbin.  Where no such orbit exists, the band is crossed.
    (Applied only when the default orbit's push did not fit one pass, it
    saved a strong engine ~40 m/s to Minmus but lost 3 of 12 dates over
    0.04-0.4 m/s^2, against 8 of 8 applied always.)"""
    rp_min = max(rp_now, here.radius + here.atmosphere + cfg.SAFE_ALT_M)
    peri = moon.sma * (1 - moon.ecc)
    top = peri - cfg.WAIT_SOI_MARGIN * moon.soi
    others = [b for b in here.children if b is not moon]
    inner = [b for b in others if b.sma < moon.sma]
    arc = math.radians(cfg.CAPTURE_ARC_DEG)
    v_m = math.sqrt(here.mu / moon.sma)

    def search(ceiling, one_pass_push):
        rp = rp_min
        while rp <= ceiling:
            if transfer.clear_of(rp, others, cfg.WAIT_SOI_MARGIN):
                vinf = transfer.moon_arrival_vinf(here.mu, rp, moon.sma, v_m)
                cap = transfer.capture_orbit(
                    moon, vinf, accel, rc_min, ra_cap, arc, cfg.CAPTURE_FILL,
                    cfg.CAPTURE_LOOSE_FRACTION * moon.soi)
                ra = min(max(cfg.WAIT_AP_FRACTION * peri, 1.05 * rp), ceiling)
                if cap is not None and (not one_pass_push or transfer.push_fits(
                        here.mu, rp, ra, moon.sma, accel, arc,
                        cfg.CAPTURE_FILL)):
                    return rp, ra, cap
            rp += max(1e3, 0.01 * (ceiling - rp_min))
        return None

    found = None
    if inner:
        wall = min(b.sma * (1 - b.ecc) - cfg.WAIT_SOI_MARGIN * b.soi
                   for b in inner)
        if wall > rp_min:
            found = search(min(top, wall), True)
    if found is None:
        found = search(top, False)
    ok = found is not None
    if ok:
        rp_wait, ra_wait, (rc, ra_c) = found
    else:
        rp_wait, rc, ra_c = rp_min, rc_min, ra_cap
        ra_wait = min(max(cfg.WAIT_AP_FRACTION * peri, 1.05 * rp_wait), top)
    return ra_wait, min(rp_wait, ra_wait), rc, ra_c, ok


def route(here, dest):
    """The legs from ``here`` to ``dest``: a list of destination bodies,
    each leg within an SOI, to a moon, or to a sibling planet."""
    if dest is here or dest.parent is here or (
            dest.parent is here.parent and here.parent is not None):
        return [dest]
    if dest.parent is not None and dest.parent.parent is here.parent:
        return [dest.parent, dest]           # a sibling's moon
    raise ValueError(f"no route from {here.name} to {dest.name}")


def fly(system, vessel, body, r, v, t0, dest, target, cfg, record=True):
    """Fly from (body, r, v, t0) to ``target`` about ``dest``, in as many
    legs as ``route`` needs.  A leg to a planet that a moon leg follows ends
    in that moon leg's waiting orbit, captured straight into it."""
    legs = route(body, dest)
    t, results = t0, []
    for i, leg in enumerate(legs):
        tgt = target
        if i + 1 < len(legs):
            moon = legs[i + 1]
            accel = vessel.thrust_n / vessel.mass_kg
            rc = max(target.rp or target.ra or 0.0,
                     moon.radius + moon.atmosphere + cfg.SAFE_ALT_M)
            ra_w, rp_w, _, _, _ = waiting_orbit(
                leg, moon, leg.radius + leg.atmosphere + cfg.SAFE_ALT_M, rc,
                cfg.CAPTURE_AP_FRACTION * moon.soi, accel, cfg)
            tgt = guidance.Target(ra_w, rp_w, moon.inc, moon.lan)
        res = Mission(system, vessel, body, r, v, t, leg, tgt, cfg,
                      record).run()
        results.append(res)
        if not res.done:
            break
        body, t = res.body, res.t
        r, v = res.final.rvec, res.final.vvec
    last = results[-1]
    return MissionResult(
        last.done, last.reason, t0, last.t, sum(x.dv for x in results),
        sum(x.prop_used for x in results),
        [m for x in results for m in x.maneuvers],
        [e for x in results for e in x.events], last.body, last.final)
