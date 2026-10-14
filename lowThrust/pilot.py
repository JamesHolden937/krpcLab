"""The low-thrust autopilot in the game: flies ``mission.Mission``'s phase
machine against kRPC.

Each tick the vessel's state is read from the game (body, position,
velocity, mass, thrust), the mission decides a command (throttle,
direction, how long to hold it), and the game flies it under rails warp:
the LowThrustWarp plugin turns the vessel at its reaction wheels' rate and
sets Persistent Thrust's throttle; PT thrusts.  Nothing is propagated here
-- the next tick reads where the game put the vessel.  The solar system is
the game's own (``live.system``), and the plan preview (PLAN) is the
stand-in flown from the live state.

    ./run.sh --pilot lowthrust                        # panel: type a target, PLAN, START
    ./run.sh --pilot lowthrust --target "mun: ap=50 pe=50" --autostart

Needs Persistent Thrust 1.7.5 and ``GameData/LowThrustWarp/LowThrustWarp.dll``
(``lowThrust/plugin/build.sh``).  Rails warp only: the pilot stops with a
message where the game will not warp on rails (in an atmosphere, too low).
"""

import argparse
import math
import sys
import time

import krpc

from common import panel as common_panel
from common import paths, rpccount, vec
from common.logbook import Logbook
from lowThrust import config, live, mission, orbit, sim



# The window's target fields, in its order.
FIELDS = ("pe", "ap", "i", "lan", "argpe")


class Panel:
    """The plugin's window (a stock KSP window, opened from the toolbar):
    the target -- body from a dropdown, periapsis, apoapsis, inclination,
    LAN, argument of periapsis -- PLAN, START, TERMINATE, and the status
    lines written here.  The window keeps its fields when the pilot exits."""

    def __init__(self, conn, lt, target_text):
        self.lt = lt
        if target_text:
            name, _, rest = target_text.partition(":")
            f = dict(item.partition("=")[::2]
                     for item in rest.replace(",", " ").split())
            lt.panel_set_target([name.strip().capitalize()]
                                + [f.get(k, "") for k in FIELDS])
        lt.panel_flying(False)
        lt.panel_open(True)
        self._clicks = conn.add_stream(lt.panel_clicks)
        self._seen = list(self._clicks())

    def _pressed(self, i):
        c = self._clicks()
        if c[i] != self._seen[i]:
            self._seen[i] = c[i]
            return True
        return False

    def plan_pressed(self):
        return self._pressed(0)

    def start_pressed(self):
        return self._pressed(1)

    def terminate_pressed(self):
        return self._pressed(2)

    def target_text(self):
        """The target as ``body: pe= ap= [i= lan= argpe=]``: a blank field
        is free, but one apsis alone is a circular orbit there."""
        got = self.lt.panel_target()
        f = dict(zip(FIELDS, got[1:]))
        if f["pe"] and not f["ap"]:
            f["ap"] = f["pe"]
        elif f["ap"] and not f["pe"]:
            f["pe"] = f["ap"]
        return "%s: %s" % (got[0].lower(), " ".join(
            "%s=%s" % (k, f[k]) for k in FIELDS if f[k]))

    def flying(self):
        self.lt.panel_flying(True)

    def show(self, lines):
        """Status lines; a tab splits one into a label and a value."""
        self.lt.panel_status(lines)

    def close(self):
        try:
            self.lt.panel_flying(False)
            self._clicks.remove()
        except Exception:                                   # noqa: BLE001
            pass


class Cmd:
    """A command queued with the plugin, and what became of it."""

    def __init__(self, thr, d, tick, coast):
        self.thr, self.d, self.tick, self.coast = thr, d, tick, coast
        self.rtn = None             # thrust: d along (radial, along, normal)
        self.seq = 0
        self.r0 = self.v0 = None    # the state when the game started it
        self.m0 = 0.0
        self.fired = False
        self.pred = 0.0             # dv booked before it flew


class Terminated(Exception):
    pass


class Pilot:
    def __init__(self, conn, cfg, log, target_text, autostart):
        self.conn, self.cfg, self.log = conn, cfg, log
        self.orf = None
        self.sc = conn.space_center
        self.v = self.sc.active_vessel
        self.lt = getattr(conn, "low_thrust", None)
        if self.lt is None or not hasattr(self.lt, "queue"):
            raise RuntimeError(
                "this kRPC server has no LowThrust service, or an old one: "
                "install GameData/LowThrustWarp/LowThrustWarp.dll (from "
                "testInstances/base/GameData/LowThrustWarp/, built by "
                "lowThrust/plugin/build.sh) and restart KSP")
        self.counter = rpccount.RpcCounter().install(conn)
        self.gui = Panel(conn, self.lt, target_text)
        self.autostart = autostart
        self.ut = conn.add_stream(getattr, self.sc, "ut")
        self.body_s = conn.add_stream(getattr, self.v.orbit, "body")
        # KSP does not recompute a vessel's mass on rails (it read 29875 kg
        # all through a burn that used 22 kg of xenon): the mass is the
        # mass at load less the propellant (every resource with a density)
        # used since, which the plugin counts
        self._m0 = self.v.mass
        self._p0 = self.lt.propellant_mass()
        self.thrust_s = conn.add_stream(getattr, self.v, "available_thrust")
        self.rate_s = conn.add_stream(getattr, self.sc, "warp_rate")
        self.rec_s = conn.add_stream(self.lt.record)
        self.isp = 4200.0
        self.system = live.system(conn)
        self.game_bodies = self.sc.bodies
        self.body_names = list(self.lt.body_names())
        # this game's rails rates (BetterTimeWarp changes them; learned
        # from the eased warp_rate they came out wrong)
        self.rates = dict(enumerate(self.lt.rails_warp_rates()))
        self.frame_body = None
        self.coast_rtn = (0.0, 1.0, 0.0)       # prograde until a burn says
        self._shown = 0.0
        log.event(self.ut(), "config: %s defaults_fingerprint=%s" % (
            " ".join(config.differences(cfg)) or "(defaults)",
            config.defaults_fingerprint()))
        worst = max(live.check_system(conn, self.system),
                    key=lambda row: row[1])
        log.event(self.ut(), "rails check: worst body position error %.0f m "
                             "(%s, 1e6 s ahead), SOI %.0f m" % (worst[1],
                                                                 worst[0],
                                                                 worst[2]))

    # --- reading the game ---------------------------------------------------
    def mass_of(self, prop):
        """kg: the load mass less the propellant used since (``prop``: the
        propellant mass now, kg)."""
        return self._m0 - (self._p0 - prop)

    def mass(self):
        return self.mass_of(self.lt.propellant_mass())

    def body(self):
        return self.system[self.body_s().name]

    def frame(self, body):
        if self.frame_body is not body:
            self.frame_body = body
            self.nrf = self.game_bodies[body.name].non_rotating_reference_frame
            self.pos = self.conn.add_stream(self.v.position, self.nrf)
            self.vel = self.conn.add_stream(self.v.velocity, self.nrf)
        return self.nrf

    def state(self):
        """(body, r, v, t, mass) now, in the planner's frame (the game held,
        or not yet flying)."""
        b = self.body()
        self.frame(b)
        return b, live.to_ours(self.pos()), live.to_ours(self.vel()), \
            self.ut(), self.mass()

    def record(self):
        """The plugin's record of the last command switch (``Record``)."""
        x = self.rec_s()
        return dict(seq=int(x[0]), ut=x[1], prop=x[2], fired=x[3] > 0.5,
                    err=x[4], warp=int(x[5]),
                    body=self.body_names[int(x[6])] if x[0] > 0 else None,
                    r=live.to_ours(x[7:10]), v=live.to_ours(x[10:13]),
                    held=x[13] > 0.5, prop_now=x[14], queued=int(x[15]),
                    ut_now=x[16])

    def wait_update(self):
        """Until the record stream next updates (or 50 ms)."""
        with self.rec_s.condition:
            self.rec_s.wait(0.05)
        if self.gui.terminate_pressed():
            raise Terminated()

    def vessel_model(self):
        """The planner's vessel: the live mass and thrust (Isp for the
        report only)."""
        thrust = self.thrust_s()
        if thrust <= 0:
            # engines not yet on (a PLAN before START): their full thrust
            thrust = sum(e.max_vacuum_thrust * e.thrust_limit
                         for e in self.v.parts.engines)
        return sim.Vessel(self.mass(), thrust, self.isp)

    # --- commanding it ------------------------------------------------------
    def warp_index(self, tick):
        """The fastest rails rate whose frames still fit ``tick``
        (WARP_PER_TICK); the plugin caps it where the altitude does."""
        want = self.cfg.WARP_PER_TICK * tick
        idx = 1
        for i, rate in self.rates.items():
            if i > idx and rate <= want:
                idx = i
        return idx

    def issue(self, cmd, direction, frame):
        """Queue ``cmd`` with the plugin -- it starts when the command in
        flight ends, the game running through the switch -- and wait until
        it has.  Returns the plugin's record of the switch."""
        floor = self.warp_index(self.cfg.WARP_FLOOR_S)
        align = self.cfg.ALIGN_DEG if cmd.thr > 0 else 180.0

        def send():
            return self.lt.queue(cmd.thr, direction, frame, align,
                                 self.warp_index(cmd.tick), floor, cmd.tick,
                                 self.nrf)
        cmd.seq = send()
        lost = None
        while True:
            rec = self.record()
            if rec["seq"] >= cmd.seq:
                if rec["warp"] < 1:
                    raise RuntimeError("the game will not rails-warp here (an "
                                       "atmosphere, or too low): this pilot "
                                       "flies on rails only")
                return rec
            if rec["queued"] != cmd.seq:
                # dropped (the plugin took the pilot for dead and released
                # its hold): queue it again
                if lost is None:
                    lost = time.time()
                elif time.time() - lost > self.cfg.HOLD_LOST_S:
                    self.log.event(rec["ut_now"], "command %d lost: re-queued"
                                   % cmd.seq)
                    cmd.seq = send()
                    lost = None
            else:
                lost = None
            self.wait_update()

    def finish(self, cmd):
        """Wait for ``cmd`` to end with nothing after it -- the game holds
        there -- and return the record (the propellant now is its end's)."""
        end = None
        while True:
            rec = self.record()
            if rec["seq"] == cmd.seq and rec["held"]:
                return rec
            if rec["ut_now"] >= rec["ut"] + cmd.tick:
                # past its end and not held: the hold was dropped (a pause,
                # the plugin's orphan release) -- hold again, at once
                if end is None:
                    end = time.time()
                elif time.time() - end > self.cfg.HOLD_LOST_S:
                    self.log.event(rec["ut_now"], "hold at %.1f lost: re-issued"
                                   % (rec["ut"] + cmd.tick))
                    self.lt.run_until(rec["ut"] + cmd.tick,
                                      self.warp_index(self.cfg.WARP_FLOOR_S))
                    end = None
            self.wait_update()

    def book(self, mis, cmd, prop_end):
        """Correct ``cmd``'s booked dv to what its propellant says it
        delivered (counted as throttle x time, a raise to 150 km read 93.4
        m/s where its xenon says 99.2 -- under the impulsive 96.4)."""
        m1 = self.mass_of(prop_end)
        dv = self.isp * sim.G0 * math.log(cmd.m0 / m1) \
            if cmd.fired and m1 < cmd.m0 else 0.0
        mis.adjust(cmd.d, dv - cmd.pred, cmd.r0, cmd.v0)

    def predict(self, mis, cmd):
        """Advance the mission's craft from the switch to ``cmd``'s end: a
        thrust tick by RK4, the direction held against the orbit as the
        plugin holds it, at the stand-in's thrust; a coast exactly.  False
        if an SOI boundary falls inside it (then the game is let reach the
        end and the state is read there)."""
        cr, cfg = mis.craft, self.cfg
        if cmd.coast:
            return mission.coast(cr, cmd.tick) is None
        a = mis.vessel.thrust_n * cmd.thr / cr.m if cmd.fired else 0.0
        cR, cT, cN = cmd.rtn
        h = cmd.tick / cfg.THRUST_SUBSTEPS
        for _ in range(cfg.THRUST_SUBSTEPS):
            R, T, N = orbit.rtn(cr.r, cr.v)
            d = vec.add(vec.add(vec.scale(R, cR), vec.scale(T, cT)),
                        vec.scale(N, cN))
            cr.r, cr.v, _ = sim._rk4(cr.r, cr.v, 1.0, cr.body.mu,
                                     vec.scale(d, a), 0.0, h)
        cr.t += cmd.tick
        return mission._crossing(cr, cr.r, cr.t) is None

    # --- the flight -----------------------------------------------------------
    def wait_for_start(self):
        while True:
            if self.gui.terminate_pressed():
                raise Terminated()
            if self.autostart or self.gui.start_pressed():
                return self.parse()
            if self.gui.plan_pressed():
                try:
                    self.preview(*self.parse())
                except Exception as exc:                    # noqa: BLE001
                    self.gui.show(["PLAN failed:", str(exc)])
            time.sleep(0.2)

    def parse(self):
        """The target from the window.  LAN and argument of periapsis are
        typed as KSP shows them; the planner's longitudes start elsewhere
        (``live.lan_offset``)."""
        text = self.gui.target_text()
        dest, f = mission.parse_orbit(text, self.system)
        if "lan" in f or "argpe" in f:
            off, spread = live.lan_offset(self.conn, self.system)
            self.log.event(self.ut(), "LAN offset %.3f deg (bodies agree to "
                                      "%.3f)" % (math.degrees(off),
                                                 math.degrees(spread)))
            if "lan" in f:
                f["lan"] = (f["lan"] + math.degrees(off)) % 360.0
            elif abs(f.get("i", 0.0)) < 1e-6:
                # equatorial: the periapsis is measured from KSP's
                # reference direction (its LAN 0)
                f["lan"] = math.degrees(off) % 360.0
        return dest, mission.target_from(dest, f)

    def preview(self, dest, target):
        """The stand-in's flight from here (constant mass)."""
        self.gui.show(["planning..."])
        b, r, v, t, _ = self.state()
        res = mission.fly(self.system, self.vessel_model(), b, r, v, t,
                          dest, target, self.cfg)
        lines = ["PLAN %s: %s" % ("arrives" if res.done else "FAILS",
                                  res.reason),
                 "dv\t%.0f m/s, %.0f kg xenon" % (res.dv, res.prop_used),
                 "time\t%.1f days, to %s" % ((res.t - res.t0) / 21600,
                                             mission.fmt_ut(res.t))]
        by = {}
        for m in res.maneuvers:
            by[(m.body, m.phase)] = by.get((m.body, m.phase), 0.0) + m.dv
        for (body, phase), dv in list(by.items())[:12]:
            lines.append("  %s %s\t%.0f m/s" % (body, phase, dv))
        self.gui.show(lines)
        self.log.event(t, "plan: " + " | ".join(
            x.replace("\t", " ").strip() for x in lines))

    def engine_isp(self):
        """Vacuum Isp of the active engines (thrust-weighted), s."""
        f = w = 0.0
        for e in self.v.parts.engines:
            if e.active and e.vacuum_specific_impulse > 0:
                f += e.max_vacuum_thrust
                w += e.max_vacuum_thrust / e.vacuum_specific_impulse
        return f / w if w > 0 else 4200.0

    def prepare(self):
        """Engines on, SAS off (the plugin steers on rails), throttle shut,
        rails warp at RAILS_PACE."""
        self.lt.set_pace(self.cfg.RAILS_PACE)
        self.lt.set_frames_per_command(self.cfg.RAILS_FRAMES_PER_COMMAND)
        for e in self.v.parts.engines:
            if not e.active:
                e.active = True
        self.v.control.sas = False
        self.lt.set_throttle(0.0)
        self.isp = self.engine_isp()
        if self.lt.engine_count() == 0:
            raise RuntimeError("no Persistent Thrust engine on this vessel "
                               "(PT not installed, or a craft from before it "
                               "was)")
        self.stats0 = list(self.lt.pipeline_stats())

    def fly(self, dest, target):
        self.gui.flying()
        self.prepare()
        b, r, v, t, m = self.state()
        self.log.event(t, "target: %s -> %s %s" % (b.name, dest.name,
                                                   self.gui.target_text()))
        legs = mission.route(b, dest)
        result = None
        for i, leg in enumerate(legs):
            tgt = target
            if i + 1 < len(legs):
                moon = legs[i + 1]
                ves = self.vessel_model()
                rc = max(target.rp or target.ra or 0.0, moon.radius
                         + moon.atmosphere + self.cfg.SAFE_ALT_M)
                ra_w, rp_w, _, _, _ = mission.waiting_orbit(
                    leg, moon, leg.radius + leg.atmosphere
                    + self.cfg.SAFE_ALT_M, rc,
                    self.cfg.CAPTURE_AP_FRACTION * moon.soi,
                    ves.thrust_n / ves.mass_kg, self.cfg, from_above=True)
                tgt = mission.guidance.Target(ra_w, rp_w, moon.inc, moon.lan)
            result = self.fly_leg(leg, tgt)
            if not result.done:
                break
        return result

    def fly_leg(self, leg, target):
        """The mission flown against the game, one command ahead: while the
        game flies a command, the state at its end is predicted (from the
        state the plugin recorded when it started), the mission decides the
        next one from that, and it is queued to start the moment this one
        ends -- the game runs on, it is never stopped to wait for the pilot
        unless the pilot is late.  Each command's dv is booked as predicted
        and corrected from the propellant it took.  Where the prediction
        cannot be trusted (an SOI boundary inside the command, the body
        changed under it) the game is let reach the end and held, and the
        state is read there."""
        b, r, v, t, m = self.state()
        mis = mission.Mission(self.system, self.vessel_model(), b, r, v, t,
                              leg, target, self.cfg)
        self.seen = 0
        pending = None          # a command decided on a predicted coast
        cur = None              # the command the game is flying
        sync = True             # read the state now: the game is held
        while True:
            self.drain(mis)
            cr = mis.craft
            if sync:
                if cur is not None:
                    self.book(mis, cur, self.finish(cur)["prop_now"])
                    cur = None
                b, r, v, t, m = self.state()
                if b is not cr.body:
                    kind = "exit" if b is cr.body.parent else "enter"
                    cr.body, cr.r, cr.v, cr.t, cr.m = b, r, v, t, m
                    mis._soi_change(kind, b)
                    pending = None
                    continue
                cr.r, cr.v, cr.t, cr.m = r, v, t, m
                sync = False
            mis.vessel.mass_kg = cr.m
            mis.vessel.thrust_n = self.thrust_s()
            self.counter.tick(mis.phase, cr.t)
            s = pending if pending is not None else mis.step()
            pending = None
            if isinstance(s, mission.MissionResult):
                if cur is not None:
                    self.book(mis, cur, self.finish(cur)["prop_now"])
                self.drain(mis)
                return s
            thr, d, tick = s
            R, T, N = orbit.rtn(cr.r, cr.v)
            self.frame(cr.body)
            if thr > 0:
                cmd = Cmd(thr, d, tick, coast=False)
                cmd.rtn = (vec.dot(d, R), vec.dot(d, T), vec.dot(d, N))
                self.coast_rtn = cmd.rtn
                if self.orf is None:
                    self.orf = self.v.orbital_reference_frame
                # held fixed against the orbit (kRPC's orbital frame is
                # (-radial, along-track, normal)), so the plugin tracks it
                # through the tick: an inertial command held through a tick
                # is several degrees behind prograde by the next one near
                # the periapsis of a high orbit (6 deg in 26 s), and a tick
                # that starts beyond ALIGN_DEG is not fired
                direction = (-cmd.rtn[0], cmd.rtn[1], cmd.rtn[2])
                frame = self.orf
            else:
                # Coasting: the coast is exact two-body motion, so the
                # mission is run ahead on it until it next wants thrust, and
                # the game warps there in one command (ticked at 11 s, a low
                # orbit's coast between arcs ran at 23x of a 50x warp)
                mis.account(None, 0.0, tick, cr.r, cr.v)
                r0, v0, t0, b0 = cr.r, cr.v, cr.t, cr.body
                span, pending, aim = self.coast_ahead(mis, tick)
                cr.body, cr.r, cr.v, cr.t = b0, r0, v0, t0
                if aim is None:
                    cR, cT, cN = self.coast_rtn
                    aim = vec.add(vec.add(vec.scale(R, cR), vec.scale(T, cT)),
                                  vec.scale(N, cN))
                cmd = Cmd(0.0, vec.unit(aim), span, coast=True)
                direction, frame = live.to_krpc(vec.unit(aim)), self.nrf
            rec = self.issue(cmd, direction, frame)
            if cur is not None:
                self.book(mis, cur, rec["prop"])
            cur = cmd
            cmd.r0, cmd.v0, cmd.m0 = rec["r"], rec["v"], self.mass_of(rec["prop"])
            cmd.fired = rec["fired"]
            if rec["body"] != cr.body.name:
                # the vessel changed SOI before this command started: let it
                # run out (its dv booked from its propellant alone), then
                # start again from the state there
                sync = True
                continue
            cr.r, cr.v, cr.t, cr.m = rec["r"], rec["v"], rec["ut"], cmd.m0
            if not cmd.coast:
                R, T, N = orbit.rtn(cr.r, cr.v)
                cmd.d = vec.add(vec.add(vec.scale(R, cmd.rtn[0]),
                                        vec.scale(T, cmd.rtn[1])),
                                vec.scale(N, cmd.rtn[2]))
                cmd.pred = (mis.vessel.thrust_n * thr / cr.m * tick
                            if cmd.fired else 0.0)
            self.telemetry(mis, cr.body, cr.r, cr.v, cr.t, cmd.thr,
                           thr if cmd.fired else 0.0, rec["err"], cmd.tick,
                           rec["warp"])
            if not self.predict(mis, cmd):
                sync = True
            if not cmd.coast:
                mis.account(cmd.d, cmd.pred, cmd.tick, cmd.r0, cmd.v0)

    def coast_ahead(self, mis, tick):
        """Run the mission ahead along the predicted coast (``mission.coast``,
        exact) from the tick it just decided to coast, until it decides to
        thrust or ends, an SOI boundary is near, or COAST_AHEAD of a
        revolution has gone.  Returns (seconds to coast in the game, the
        command decided at its end or None, the direction to turn to)."""
        cr = mis.craft
        c = orbit.conic(cr.r, cr.v, cr.body.mu)
        limit = self.cfg.COAST_AHEAD * (c.period if c.e < 1 else 100 * tick)
        span = 0.0
        while True:
            saved = (cr.body, cr.r, cr.v, cr.t)
            if mission.coast(cr, tick) is not None:
                # stop short of the boundary: the game crosses it
                cr.body, cr.r, cr.v, cr.t = saved
                return max(span, 1.0), None, None
            span += tick
            if span >= limit:
                return span, None, None
            s = mis.step()
            if isinstance(s, mission.MissionResult) or s[0] > 0:
                return span, s, (None if isinstance(s, mission.MissionResult)
                                 else s[1])
            tick = s[2]

    def drain(self, mis):
        for e in mis.events[self.seen:]:
            self.log.event(e.t, e.text)
        self.seen = len(mis.events)

    def telemetry(self, mis, b, r, v, ut, thr, fire, err, tick, warp):
        c = orbit.conic(r, v, b.mu)
        alt = lambda x: (x - b.radius) / 1e3
        ap = "%.0f" % alt(c.ra) if c.e < 1 else "hyp"
        inc = math.degrees(orbit.inclination_lan(r, v)[0])
        # where the vessel is: from the osculating periapsis, and from the
        # apse line the in-SOI law holds (the arcs are centred on it)
        apse = getattr(mis.target, "apse", None) if mis.target else None
        from_apse = (math.degrees(math.atan2(
            vec.dot(vec.cross(apse, vec.unit(r)), vec.unit(vec.cross(r, v))),
            vec.dot(apse, vec.unit(r)))) if apse is not None else float("nan"))
        rate = self.rates.get(warp, 1)
        line = ("%-8s %-7s ap=%sk pe=%.0fk i=%.2f nu=%.0f apse=%+.0f "
                "thr=%.2f/%.2f err=%.1f tick=%.0fs warp=%.0fx m=%.0fkg dv=%.1f"
                % (mis.phase, b.name, ap, alt(c.rp), inc,
                   math.degrees(c.nu), from_apse, fire, thr, err,
                   tick, rate, mis.craft.m, mis.dv))
        self.log.telemetry(ut, line)
        if time.monotonic() - self._shown < 1.0:
            return
        self._shown = time.monotonic()
        self.gui.show(["phase\t%s" % mis.phase,
                       "body\t%s" % b.name,
                       "ap / pe\t%s / %.0f km" % (ap, alt(c.rp)),
                       "inclination\t%.2f deg" % inc,
                       "throttle\t%.2f (wants %.2f)" % (fire, thr),
                       "pointing\t%.1f deg off" % err,
                       "warp\t%.0fx, pace x%.0f" % (rate, self.cfg.RAILS_PACE),
                       "dv used\t%.0f m/s" % mis.dv,
                       "UT\t%s" % mission.fmt_ut(ut)])

    def shutdown(self, reason, lines=None):
        ut = 0.0
        try:
            ut = self.ut()
            self.lt.release_hold()
            self.lt.set_throttle(0.0)
            self.lt.clear_attitude()
            self.lt.set_pace(1.0)
            self.lt.set_rails_warp(0)
            self.v.control.sas = True
            st = list(self.lt.pipeline_stats())
            s0 = getattr(self, "stats0", [0.0, 0.0, 0.0])
            n, stalls, held = (st[i] - s0[i] for i in range(3))
            self.log.event(ut, "pipeline: %d commands, %d found nothing queued "
                               "(the game held for the pilot, %.1f s wall)"
                           % (n, stalls, held))
        except Exception as exc:                            # noqa: BLE001
            self.log.event(ut, "shutdown control error: %r" % exc)
        self.log.event(ut, self.counter.report())
        self.log.event(ut, self.counter.wall_report())
        self.log.event(ut, "shutdown: %s" % reason)
        try:
            self.gui.show(lines or [reason])
        except Exception:                                   # noqa: BLE001
            pass
        self.gui.close()


def parse_args(argv):
    p = argparse.ArgumentParser(prog="lowthrust", description=__doc__.splitlines()[0])
    p.add_argument("--address", default="127.0.0.1")
    p.add_argument("--rpc-port", type=int, default=50000)
    p.add_argument("--stream-port", type=int, default=50001)
    p.add_argument("--set", action="append", default=[], metavar="K=V")
    p.add_argument("--target", default=None,
                   help='e.g. "mun: ap=50 pe=50" (the panel field otherwise)')
    p.add_argument("--autostart", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = config.apply_overrides(config.Config(), args.set)
    log = Logbook(paths.LOGS, cfg.LOG_INTERVAL_UT)
    pilot, reason, status = None, "shutdown", 0
    try:
        conn = krpc.connect(name="lowthrust", address=args.address,
                            rpc_port=args.rpc_port,
                            stream_port=args.stream_port)
        pilot = Pilot(conn, cfg, log, args.target,
                      args.autostart and args.target is not None)
        dest, target = pilot.wait_for_start()
        res = pilot.fly(dest, target)
        reason = ("ARRIVED: " if res.done else "FAILED: ") + res.reason
        c = res.final
        log.event(res.t, "%s; %s ap %.1f km pe %.1f km i %.2f deg; %.1f m/s, "
                         "%.1f days" % (
                             reason, res.body.name,
                             (c.ra - res.body.radius) / 1e3,
                             (c.rp - res.body.radius) / 1e3,
                             math.degrees(orbit.inclination_lan(
                                 c.rvec, c.vvec)[0]),
                             res.dv, (res.t - res.t0) / 21600))
    except Terminated:
        reason = "terminated from the panel"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
    except Exception as exc:                                # noqa: BLE001
        import traceback
        log.event(0.0, "FATAL %r\n%s" % (exc, traceback.format_exc()))
        reason = "error: %r" % exc
        status = 1
    finally:
        if pilot is not None:
            pilot.shutdown(reason)
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
