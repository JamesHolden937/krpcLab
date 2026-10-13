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
  the capture altitude; CAPTURE burns retrograde centred on periapsis until
  the orbit is bound below ``CAPTURE_AP_FRACTION`` of the SOI.
"""

import math
from collections import namedtuple
from dataclasses import dataclass

from common import vec
from lowThrust import guidance, orbit, transfer
from lowThrust.sim import G0, _rk4, tick_for

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
                 record=True, m0=None):
        self.cfg = cfg
        self.sys = system
        self.vessel = vessel
        self.craft = Craft(body, r, v, vessel.dry_kg + vessel.prop_kg
                           if m0 is None else m0, t0)
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
        self._route()

    # --- bookkeeping ------------------------------------------------------
    def event(self, text):
        self.events.append(Event(self.craft.t, text))

    def set_phase(self, phase, text=""):
        self._close_burn()
        self.phase = phase
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
            # arrive slower: the cruise spends dv to match the planet
            # loose, out past the planet's moons if need be (the pass
            # guards and evasive burns then keep clear of them)
            arc = math.radians(cfg.CAPTURE_ARC_DEG)
            loose = max(ra_cap, 0.5 * dest.soi)
            vmax = cfg.ARRIVE_MARGIN * transfer.capturable_vinf(
                dest, self._accel(), rc, ra_cap, arc, cfg.CAPTURE_FILL, loose)
            cap = transfer.capture_orbit(dest, vmax, self._accel(), rc,
                                         ra_cap, arc, cfg.CAPTURE_FILL, loose)
            if vmax <= 0 or cap is None:
                self.event(f"warning: this engine cannot capture at "
                           f"{dest.name} in one pass at any arrival speed")
            else:
                if vmax < vec.norm(w.vinf_arr):
                    self.plan["vinf_max"] = vmax
                    self.event(f"arrive slow: the cruise will cut v_inf from "
                               f"{vec.norm(w.vinf_arr):.0f} to {vmax:.0f} "
                               "m/s so this engine can capture in one pass")
                else:
                    cap = transfer.capture_orbit(
                        dest, vec.norm(w.vinf_arr), self._accel(), rc,
                        ra_cap, arc, cfg.CAPTURE_FILL, loose)
                rc, ra_cap = cap
                self.event(f"capture high and loose for this engine: "
                           f"{self._km(ra_cap, dest)} x {self._km(rc, dest)}")
        elif cap[0] > rc * 1.001:
            rc, ra_cap = cap
            self.event(f"capture high and loose for this engine: "
                       f"{self._km(ra_cap, dest)} x {self._km(rc, dest)}")
        self.plan.update(window=w, rp_park=rp, ra_park=ra,
                         vinf=vec.norm(w.vinf_dep), rc=rc, ra_cap=ra_cap)
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
        limit = self.t0 + cfg.MISSION_MAX_DAYS * 21600
        while True:
            cr = self.craft
            c = orbit.conic(cr.r, cr.v, cr.body.mu)
            floor = cr.body.radius + cr.body.atmosphere
            if c.r < floor:
                return self._end(False, f"hit {cr.body.name}'s "
                                        "atmosphere/surface")
            if (c.e < 1 and c.rp < floor and self.phase in ("WAIT", "COAST")
                    and cr.body is not self.dest):
                self._repump("periapsis inside the atmosphere")
            if cr.t > limit:
                return self._end(False, "time limit")
            if cr.m <= self.vessel.dry_kg + 1e-9:
                return self._end(False, "out of propellant")

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
                             self.vessel.dry_kg + self.vessel.prop_kg - cr.m,
                             self.maneuvers, self.events, cr.body,
                             orbit.conic(cr.r, cr.v, cr.body.mu))

    def _thrust(self, direction, throttle, tick):
        cr, ves, cfg = self.craft, self.vessel, self.cfg
        force = vec.scale(direction, ves.thrust_n * throttle)
        mdot = ves.thrust_n * throttle / (ves.isp_s * G0)
        R, T, N = orbit.rtn(cr.r, cr.v)
        m0 = cr.m
        h = tick / cfg.THRUST_SUBSTEPS
        for _ in range(cfg.THRUST_SUBSTEPS):
            cr.r, cr.v, cr.m = _rk4(cr.r, cr.v, cr.m, cr.body.mu, force, mdot, h)
        cr.m = max(cr.m, ves.dry_kg)
        cr.t += tick
        dv = ves.isp_s * G0 * math.log(m0 / cr.m)
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
        if cr.body is self.dest and self.phase in ("COAST", "CRUISE",
                                                   "ESCAPE", "PUSH", "WAIT",
                                                   "PUMP", "TRIM", "PARK"):
            self.set_phase("APPROACH", f"periapsis target "
                                       f"{self._km(self.plan['rc'], cr.body)}")
        elif self.phase == "ESCAPE" and cr.body is self.dest.parent:
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
        if el is None:
            # a flyby flung us out: brake until bound again, then carry on
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
            rp_, vp_ = orbit.propagate(cr.r, cr.v, mu, t_pe)
            v1 = vec.add(vp_, vec.scale(vec.unit(vp_), need * self._accel()))
            bad = transfer.first_hazard(rp_, v1, cr.t + t_pe, mu,
                                        cr.body.children,
                                        cr.t + t_pe + self.cfg.ESCAPE_CHECK_S)
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

    def _depart(self, c):
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        tick = self._tick(c)
        rem = math.sqrt(self.plan["vinf"] ** 2 + 2 * mu / c.r) - vec.norm(cr.v)
        if rem <= cfg.CORRECT_DV or c.nu > math.radians(cfg.DEPART_ARC_DEG):
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
        if cr.body is not frame:
            return 0.0, None, tick           # an unplanned flyby: just coast
        due = self.plan.get("next_fix", -math.inf)
        if self._burn is None and cr.t < due:
            return 0.0, None, min(tick, due - cr.t)
        vinf_guess = vec.norm(self.plan.get("vrel", (1e3, 0, 0)))
        b = transfer.impact_parameter(self.plan["rc"], max(vinf_guess, 1.0),
                                      dest.mu)
        horizon = self.plan.get("horizon")
        if horizon is None:
            if self.kind == "planet":
                w = self.plan["window"]
                horizon = 1.5 * max(w.t_dep + w.tof - cr.t, 86400.0)
            else:
                horizon = self._horizon(cr.r, cr.v)
        dv, ca = transfer.correction(cr.r, cr.v, cr.t, cr.body.mu, dest,
                                     horizon, b, dest.normal,
                                     vinf_max=self.plan.get("vinf_max"))
        self.plan["vrel"] = ca.vrel
        self.plan["horizon"] = max(ca.t - cr.t, 0) * 1.5 + 3600
        mag = vec.norm(dv)
        to_ca = ca.t - cr.t
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

    def _coast(self, c):
        return self._correct(c, self.dest.parent)

    def _cruise(self, c):
        return self._correct(c, self.dest.parent)

    def _approach(self, c):
        cr, cfg = self.craft, self.cfg
        mu = cr.body.mu
        tick = self._tick(c)
        t_pe = orbit.time_to_periapsis(c, mu)
        need = self._capture_need(c)
        if t_pe <= need / self._accel() / 2 or t_pe < 0:
            self.set_phase("CAPTURE", f"{need:.0f} m/s retrograde at "
                                      "periapsis")
            return self._capture(c)
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

    def _capture_need(self, c):
        mu = self.craft.body.mu
        rp, ra = c.rp, self.plan["ra_cap"]
        v_pe = math.sqrt(max(0.0, 2 * c.energy + 2 * mu / rp))
        return max(0.0, v_pe - math.sqrt(mu * (2 / rp - 2 / (rp + ra))))

    def _capture(self, c):
        cr, cfg = self.craft, self.cfg
        tick = self._tick(c)
        if c.e < 1 and c.ra <= self.plan["ra_cap"] * 1.01:
            self.target = self.final_target
            self.set_phase("FINAL", f"captured, {self._km(c.ra, cr.body)} x "
                                    f"{self._km(c.rp, cr.body)}")
            return 0.0, None, tick
        if c.nu > math.radians(cfg.CAPTURE_ARC_DEG) and c.e >= 1:
            return "capture failed: the engine cannot shed the excess speed " \
                   "in one periapsis pass"
        # Retrograde, plus whatever keeps the periapsis above the safe floor:
        # a long retrograde burn started before periapsis drags it down
        # (Duna: aimed at 80 km, captured at 38, inside the atmosphere).
        # Only the floor -- holding it on the aim exactly cost Duna's capture
        # 100 m/s of radial thrust, and FINAL raises a low one cheaply.
        need = self._capture_need(c)
        want = vec.scale(vec.unit(cr.v), -max(need, 0.1))
        floor = (cr.body.radius + cr.body.atmosphere + cfg.SAFE_ALT_M)
        if c.rp < floor:
            g = self._rp_gradient(c)
            gg = vec.dot(g, g)
            if gg > 0:
                want = vec.add(want, vec.scale(g, (1.05 * floor - c.rp) / gg))
        mag = vec.norm(want)
        return min(1.0, mag / (self._accel() * tick)), vec.unit(want), tick

    def _rp_gradient(self, c):
        """d(periapsis radius)/d(impulse), numerically, inertial vector."""
        cr = self.craft
        g = []
        for k in range(3):
            dv = [0.0, 0.0, 0.0]
            dv[k] = 0.01
            ck = orbit.conic(cr.r, vec.add(cr.v, tuple(dv)), cr.body.mu)
            g.append((ck.rp - c.rp) / 0.01)
        return tuple(g)


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
    periapsis is the lowest that pass allows, at least ``rc_min``."""
    rp_min = max(rp_now, here.radius + here.atmosphere + cfg.SAFE_ALT_M)
    peri = moon.sma * (1 - moon.ecc)
    top = peri - cfg.WAIT_SOI_MARGIN * moon.soi
    others = [b for b in here.children if b is not moon]
    arc = math.radians(cfg.CAPTURE_ARC_DEG)
    v_m = math.sqrt(here.mu / moon.sma)
    rp = rp_min
    found = None
    while rp <= top:
        if transfer.clear_of(rp, others, cfg.WAIT_SOI_MARGIN):
            vinf = transfer.moon_arrival_vinf(here.mu, rp, moon.sma, v_m)
            cap = transfer.capture_orbit(moon, vinf, accel, rc_min, ra_cap,
                                         arc, cfg.CAPTURE_FILL,
                                         cfg.CAPTURE_LOOSE_FRACTION * moon.soi)
            if cap is not None:
                found = (rp, cap)
                break
        rp += max(1e3, 0.01 * (top - rp_min))
    ok = found is not None
    rp_wait, (rc, ra_c) = found if ok else (rp_min, (rc_min, ra_cap))
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
    m0, t, results = None, t0, []
    for i, leg in enumerate(legs):
        tgt = target
        if i + 1 < len(legs):
            moon = legs[i + 1]
            accel = vessel.thrust_n / (m0 or vessel.dry_kg + vessel.prop_kg)
            rc = max(target.rp or target.ra or 0.0,
                     moon.radius + moon.atmosphere + cfg.SAFE_ALT_M)
            ra_w, rp_w, _, _, _ = waiting_orbit(
                leg, moon, leg.radius + leg.atmosphere + cfg.SAFE_ALT_M, rc,
                cfg.CAPTURE_AP_FRACTION * moon.soi, accel, cfg)
            tgt = guidance.Target(ra_w, rp_w, moon.inc, moon.lan)
        res = Mission(system, vessel, body, r, v, t, leg, tgt, cfg, record,
                      m0).run()
        results.append(res)
        if not res.done:
            break
        body, t = res.body, res.t
        r, v = res.final.rvec, res.final.vvec
        m0 = vessel.dry_kg + vessel.prop_kg - sum(x.prop_used for x in results)
    last = results[-1]
    return MissionResult(
        last.done, last.reason, t0, last.t, sum(x.dv for x in results),
        sum(x.prop_used for x in results),
        [m for x in results for m in x.maneuvers],
        [e for x in results for e in x.events], last.body, last.final)
