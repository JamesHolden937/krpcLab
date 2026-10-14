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

# Stock rails warp rates by index; replaced by what the game reports once a
# rate has settled (BetterTimeWarp and friends change them).
STOCK_RATES = [1, 5, 10, 50, 100, 1000, 10000, 100000]


class Panel:
    """Target field, PLAN, START, TERMINATE, and a telemetry block."""
    W, H = common_panel.WIDTH, common_panel.HEIGHT

    def __init__(self, conn, target_text):
        ui = conn.ui
        self.conn = conn
        self.panel = common_panel.window(conn)

        def text(content, y, h, size=13, color=common_panel.TEXT,
                 anchor=ui.TextAnchor.upper_left):
            t = self.panel.add_text(content)
            t.rect_transform.position = (0, y)
            t.rect_transform.size = (self.W - 24, h)
            t.size = size
            t.color = color
            t.alignment = anchor
            return t
        text("LOW THRUST", self.H / 2 - 25, 24, 16, (0.3, 0.8, 1.0),
             ui.TextAnchor.middle_center)
        self.field = self.panel.add_input_field()
        self.field.rect_transform.position = (0, self.H / 2 - 58)
        self.field.rect_transform.size = (self.W - 40, 26)
        self.field.value = target_text or "mun: ap=50 pe=50"
        self.body = text("type a target orbit, then PLAN or START",
                         -10, self.H - 190)

        def button(label, y):
            b = self.panel.add_button(label)
            b.rect_transform.position = (0, y)
            b.rect_transform.size = (self.W - 40, 28)
            return b, conn.add_stream(getattr, b, "clicked")
        self.plan_b, self._plan = button("PLAN", -self.H / 2 + 90)
        self.start_b, self._start = button("START", -self.H / 2 + 57)
        self.stop_b, self._stop = button("TERMINATE", -self.H / 2 + 24)

    def _pressed(self, stream, b):
        if stream():
            b.clicked = False
            return True
        return False

    def plan_pressed(self):
        return self._pressed(self._plan, self.plan_b)

    def start_pressed(self):
        return self._pressed(self._start, self.start_b)

    def terminate_pressed(self):
        return self._pressed(self._stop, self.stop_b)

    def target_text(self):
        return self.field.value

    def flying(self):
        self.plan_b.visible = False
        self.start_b.visible = False

    def show(self, lines):
        self.body.content = "\n".join(lines)

    def close(self):
        for s in (self._plan, self._start, self._stop):
            try:
                s.remove()
            except Exception:                               # noqa: BLE001
                pass
        try:
            self.panel.remove()
        except Exception:                                   # noqa: BLE001
            pass


class Terminated(Exception):
    pass


class Pilot:
    def __init__(self, conn, cfg, log, target_text, autostart):
        self.conn, self.cfg, self.log = conn, cfg, log
        self.sc = conn.space_center
        self.v = self.sc.active_vessel
        self.lt = conn.low_thrust
        self.counter = rpccount.RpcCounter().install(conn)
        self.gui = Panel(conn, target_text)
        self.autostart = autostart
        self.ut = conn.add_stream(getattr, self.sc, "ut")
        self.body_s = conn.add_stream(getattr, self.v.orbit, "body")
        self.mass = conn.add_stream(getattr, self.v, "mass")
        self.thrust_s = conn.add_stream(getattr, self.v, "available_thrust")
        self.system = live.system(conn)
        self.game_bodies = self.sc.bodies
        self.rates = dict(enumerate(STOCK_RATES))
        self.warp_index = None
        self.throttle = None
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
        """(body, r, v, t, mass) now, in the planner's frame."""
        b = self.body()
        self.frame(b)
        return b, live.to_ours(self.pos()), live.to_ours(self.vel()), \
            self.ut(), self.mass()

    def vessel_model(self):
        """The planner's vessel: the live mass and thrust (Isp for the
        report only)."""
        thrust = self.thrust_s()
        isp = 4200.0
        for e in self.v.parts.engines:
            if e.active and e.vacuum_specific_impulse > 0:
                isp = e.vacuum_specific_impulse
        return sim.Vessel(self.mass(), thrust, isp)

    # --- commanding it ------------------------------------------------------
    def set_throttle(self, x):
        if self.throttle is None or abs(x - self.throttle) > 1e-4:
            self.lt.set_throttle(x)
            self.throttle = x

    def point(self, d):
        self.lt.set_attitude(live.to_krpc(d), self.nrf)

    def warp(self, tick):
        """The fastest rails rate whose frames and loop latency still fit
        ``tick`` (WARP_PER_TICK), within what the game allows here."""
        top = self.sc.maximum_rails_warp_factor
        if top < 1:
            raise RuntimeError("the game will not rails-warp here (an "
                               "atmosphere, or too low): this pilot flies "
                               "on rails only")
        want = self.cfg.WARP_PER_TICK * tick
        idx = 1
        for i in range(1, top + 1):
            if self.rates.get(i, STOCK_RATES[-1]) <= want:
                idx = i
        if idx != self.warp_index:
            self.sc.rails_warp_factor = idx
            self.warp_index = idx
        return self.rates.get(idx, 1)

    def hold(self, until):
        """Let the game run to UT ``until`` (rails warp, already set)."""
        while True:
            now = self.ut()
            left = until - now
            if left <= 0:
                return now
            rate = max(self.sc.warp_rate, 1.0)
            if self.warp_index is not None and rate > 1.0:
                self.rates[self.warp_index] = rate
            time.sleep(min(0.5, max(0.005, 0.7 * left / rate)))
            if self.gui.terminate_pressed():
                raise Terminated()

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
        dest, f = mission.parse_orbit(self.gui.target_text(), self.system)
        return dest, mission.target_from(dest, f)

    def preview(self, dest, target):
        """The stand-in's flight from here (constant mass)."""
        self.gui.show(["planning..."])
        b, r, v, t, _ = self.state()
        res = mission.fly(self.system, self.vessel_model(), b, r, v, t,
                          dest, target, self.cfg)
        lines = ["PLAN %s: %s" % ("arrives" if res.done else "FAILS",
                                  res.reason),
                 "  %.0f m/s, %.0f kg xenon" % (res.dv, res.prop_used),
                 "  %.1f days, to %s" % ((res.t - res.t0) / 21600,
                                         mission.fmt_ut(res.t))]
        by = {}
        for m in res.maneuvers:
            by[(m.body, m.phase)] = by.get((m.body, m.phase), 0.0) + m.dv
        for (body, phase), dv in list(by.items())[:12]:
            lines.append("  %-7s %-8s %6.0f m/s" % (body, phase, dv))
        self.gui.show(lines)
        self.log.event(t, "plan: " + " | ".join(x.strip() for x in lines))

    def prepare(self):
        """Engines on, SAS off (the plugin steers on rails), throttle shut."""
        for e in self.v.parts.engines:
            if not e.active:
                e.active = True
        self.v.control.sas = False
        self.set_throttle(0.0)
        if self.lt.engine_count() == 0:
            raise RuntimeError("no Persistent Thrust engine on this vessel "
                               "(PT not installed, or a craft from before it "
                               "was)")

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
        b, r, v, t, m = self.state()
        mis = mission.Mission(self.system, self.vessel_model(), b, r, v, t,
                              leg, target, self.cfg)
        seen = 0
        while True:
            for e in mis.events[seen:]:
                self.log.event(e.t, e.text)
            seen = len(mis.events)
            b, r, v, t, m = self.state()
            cr = mis.craft
            if b is not cr.body:
                kind = "exit" if b is cr.body.parent else "enter"
                cr.body, cr.r, cr.v, cr.t, cr.m = b, r, v, t, m
                mis._soi_change(kind, b)
                continue
            cr.r, cr.v, cr.t, cr.m = r, v, t, m
            mis.vessel.mass_kg = m
            mis.vessel.thrust_n = self.thrust_s()
            self.counter.tick(mis.phase, t)
            s = mis.step()
            if isinstance(s, mission.MissionResult):
                for e in mis.events[seen:]:
                    self.log.event(e.t, e.text)
                return s
            thr, d, tick = s
            if thr > 0:
                R, T, N = orbit.rtn(r, v)
                self.coast_rtn = (vec.dot(d, R), vec.dot(d, T), vec.dot(d, N))
            else:
                R, T, N = orbit.rtn(r, v)
                cR, cT, cN = self.coast_rtn
                d = vec.unit(vec.add(vec.add(vec.scale(R, cR),
                                             vec.scale(T, cT)),
                                     vec.scale(N, cN)))
            self.point(d)
            err = self.lt.attitude_error()
            fire = thr if thr > 0 and 0 <= err <= self.cfg.ALIGN_DEG else 0.0
            self.set_throttle(fire)
            self.warp(tick)
            now = self.hold(t + tick)
            dt = now - t
            dv = mis.vessel.thrust_n * fire / m * dt if fire > 0 else 0.0
            mis.account(d, dv, dt, r, v)
            self.telemetry(mis, b, r, v, now, thr, fire, err, tick)

    def telemetry(self, mis, b, r, v, ut, thr, fire, err, tick):
        c = orbit.conic(r, v, b.mu)
        alt = lambda x: (x - b.radius) / 1e3
        ap = "%.0f" % alt(c.ra) if c.e < 1 else "hyp"
        inc = math.degrees(orbit.inclination_lan(r, v)[0])
        line = ("%-8s %-7s ap=%sk pe=%.0fk i=%.2f thr=%.2f/%.2f err=%.1f "
                "tick=%.0fs warp=%sx m=%.0fkg dv=%.0f"
                % (mis.phase, b.name, ap, alt(c.rp), inc, fire, thr, err,
                   tick, self.rates.get(self.warp_index, "?"), self.mass(),
                   mis.dv))
        self.log.telemetry(ut, line)
        if time.monotonic() - self._shown < 1.0:
            return
        self._shown = time.monotonic()
        self.gui.show(["phase   : %s" % mis.phase,
                       "body    : %s" % b.name,
                       "ap/pe   : %s / %.0f km" % (ap, alt(c.rp)),
                       "incl    : %.2f deg" % inc,
                       "throttle: %.2f (wants %.2f)" % (fire, thr),
                       "pointing: %.1f deg off" % err,
                       "warp    : %sx" % self.rates.get(self.warp_index, "?"),
                       "dv used : %.0f m/s" % mis.dv,
                       "UT      : %s" % mission.fmt_ut(ut)])

    def shutdown(self, reason):
        ut = 0.0
        try:
            ut = self.ut()
            self.lt.set_throttle(0.0)
            self.lt.clear_attitude()
            self.sc.rails_warp_factor = 0
            self.v.control.sas = True
        except Exception as exc:                            # noqa: BLE001
            self.log.event(ut, "shutdown control error: %r" % exc)
        self.log.event(ut, self.counter.report())
        self.log.event(ut, self.counter.wall_report())
        self.log.event(ut, "shutdown: %s" % reason)
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
