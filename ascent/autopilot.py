"""MechJeb ascent, with the booster dropped when it can just get home.

Phase machine::

    STANDBY -> LAUNCH -> ASCENT -> THROTTLE_DOWN -> SEPARATE -> HANDOFF

MechJeb's ascent autopilot (over KRPC.MechJeb) flies the whole climb: the
pitch program, PVG and the throttle.  This script does three things MechJeb
does not:

1. **It decides when the booster goes.**  Every tick it asks
   ``budget.recovery`` how much propellant the booster would land with if it
   separated now -- boostback and landing burn, from this state, on this
   booster's own masses and engines -- and starts the separation when that
   (projected ``SEP_LEAD_S`` ahead) reaches ``RESERVE_FRACTION`` of what the
   two burns need.  Everything above that reserve has gone into the climb.
2. **It throttles the booster down before separating**, so that once free
   its acceleration is ``SEP_ACCEL_RATIO`` of the upper stage's and the
   upper stage pulls away from it.  The thrust comes off by engine, outer
   pairs first and the centre last (``boosterland.engines``: the order its
   landing burn lights them, reversed), through each engine's thrust limit
   -- MechJeb keeps the main throttle.  It stages only once the booster's
   *delivered* thrust is under the target.
3. **It hands over.**  ``HANDOFF=booster`` switches to the booster and flies
   ``boosterland.autoland`` in this process (its own ``LOG<n>``, named in
   this one); ``HANDOFF=upper`` stays with MechJeb to orbit.

The booster is found, not configured: the parts the next decoupling drops
that carry lit engines.  The upper stage's engines are the ones left, lit
now or lit by that same staging.
"""

import argparse
import math
import sys
import time
from dataclasses import fields

import krpc

from common import vec
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from common.rpccount import RpcCounter
from boosterland.engines import allocate, group_engines
from boosterland.gui import ControlPanel
from . import budget
from .config import Config, apply_overrides, defaults_fingerprint

STANDBY = "STANDBY"
LAUNCH = "LAUNCH"
ASCENT = "ASCENT"
THROTTLE_DOWN = "THROTTLE_DOWN"
SEPARATE = "SEPARATE"
HANDOFF = "HANDOFF"
DONE = "DONE"

PROPELLANTS = ("LiquidFuel", "Oxidizer")
ATM = 101325.0


class Stack:
    """The booster and the upper stage, read off the vessel's staging."""

    def __init__(self, conn, vessel, logbook, ut):
        sc = conn.space_center
        self.vessel = vessel
        engines = list(vessel.parts.engines)
        lit = [e for e in engines if e.active]
        if not lit:
            raise RuntimeError("no lit engine to find the booster by")
        stage = max(e.part.decouple_stage for e in lit)
        if stage < 0:
            raise RuntimeError("the lit engines are never decoupled: no booster")
        self.stage = stage
        self.parts = list(vessel.parts.in_decouple_stage(stage))
        ids = {p for p in self.parts}
        self.engines = [e for e in engines if e.part in ids
                        and any(n in e.propellant_names for n in PROPELLANTS)]
        self.upper = [e for e in engines if e.part not in ids
                      and (e.active or e.part.stage == stage)]
        self.density = {n: sc.Resources.density(n) for n in PROPELLANTS}
        self.resources = vessel.resources_in_decouple_stage(stage, False)
        prop = self.propellant()
        self.mass0 = sum(p.mass for p in self.parts)
        self.prop0 = prop
        self.dry = self.mass0 - prop
        self.radius = self._radius(logbook, ut)
        self.thrust = {i: e.max_vacuum_thrust or e.max_thrust
                       for i, e in enumerate(self.engines)}
        logbook.event(ut, "booster: decouple stage %d, %d parts, %.1f t "
                      "(%.1f t propellant), %d engines, radius %.2f m; "
                      "upper stage: %d engines"
                      % (stage, len(self.parts), self.mass0 / 1e3, prop / 1e3,
                         len(self.engines), self.radius, len(self.upper)))

    def propellant(self):
        return sum(self.resources.amount(n) * self.density[n]
                   for n in PROPELLANTS)

    def mass(self):
        """The booster's mass now: only its main propellant changes."""
        return self.dry + self.propellant()

    def _radius(self, logbook, ut):
        """Body radius for the frontal area: the widest tank.  The tanks set
        the diameter; fins, legs and airbrakes would only overstate the
        area, and an overstated area understates the landing burn."""
        best = 0.0
        for p in self.parts:
            try:
                if not any(r.name in PROPELLANTS for r in p.resources.all):
                    continue
                lo, hi = p.bounding_box(p.reference_frame)
            except Exception:           # noqa: BLE001 -- KSP-side failure
                continue
            r = 0.5 * max(hi[0] - lo[0], hi[2] - lo[2])
            if 0.05 < r < 20.0:
                best = max(best, r)
        if best <= 0.0:
            best = 1.25
            logbook.event(ut, "booster radius: no usable tank bounding box, "
                          "assuming %.2f m" % best)
        return best

    def build_groups(self, cfg):
        """Group the booster's engines centre first, then pairs, in the plane
        perpendicular to their own thrust axis -- the vessel's frame is the
        root part's, which need not be the booster's axis."""
        frame = self.vessel.reference_frame
        rows = []
        axis = (0.0, 0.0, 0.0)
        for i, e in enumerate(self.engines):
            d = e.thrusters[0].thrust_direction(frame) if e.thrusters \
                else e.part.direction(frame)
            axis = vec.add(axis, vec.scale(d, self.thrust[i]))
        axis = vec.unit(axis)
        a = vec.unit(vec.project_out((1.0, 0.0, 0.0), axis))
        if vec.norm(vec.project_out((1.0, 0.0, 0.0), axis)) < 0.3:
            a = vec.unit(vec.project_out((0.0, 0.0, 1.0), axis))
        b = vec.cross(axis, a)
        for i, e in enumerate(self.engines):
            p = e.part.position(frame)
            rows.append((i, (vec.dot(p, a), vec.dot(p, b)), self.thrust[i]))
        self.groups = group_engines(rows, cfg.ENGINE_AXIS_TOL_M,
                                    cfg.ENGINE_RING_TOL_M, cfg.ENGINE_PAIR_TOL)
        return self.groups


class Ascent:
    def __init__(self, conn, cfg, logbook):
        self.conn = conn
        self.cfg = cfg
        self.log = logbook
        sc = conn.space_center
        self.sc = sc
        self.vessel = sc.active_vessel
        self.body = self.vessel.orbit.body
        self.frame = self.body.reference_frame
        self.control = self.vessel.control
        self.mj = conn.mech_jeb
        ut = sc.ut
        logbook.event(ut, "vessel=%s" % self.vessel.name)
        changed = ["%s=%s" % (f.name, getattr(cfg, f.name))
                   for f in fields(cfg) if getattr(cfg, f.name) != f.default]
        logbook.event(ut, "config: %s defaults_fingerprint=%s"
                      % (" ".join(changed) or "(defaults)",
                         defaults_fingerprint()))
        from boosterland.config import Config as BoosterConfig
        bcfg = BoosterConfig()
        self.pad = self.body.surface_position(bcfg.PAD_LAT, bcfg.PAD_LON,
                                              self.frame)
        pad_alt = self.body.surface_height(bcfg.PAD_LAT, bcfg.PAD_LON)
        self.pad_alt = pad_alt
        try:
            rho = self.body.density_at(pad_alt)
        except Exception:               # noqa: BLE001 -- older kRPC
            rho = 1.2 if self.body.has_atmosphere else 0.0
        self.pad_pressure = (self.body.pressure_at(pad_alt) / ATM
                             if self.body.has_atmosphere else 0.0)
        self.model_body = budget.Body(self.body.gravitational_parameter,
                                      vec.norm(self.pad), rho)
        logbook.event(ut, "pad: lat %.6f lon %.6f r=%.1f rho=%.3f p=%.3f atm"
                      % (bcfg.PAD_LAT, bcfg.PAD_LON, vec.norm(self.pad), rho,
                         self.pad_pressure))
        self.gui = ControlPanel(conn, title="ASCENT", start_label="START ASCENT")
        self.loop_rate = LoopRate()
        self.rpc = None
        self.state = STANDBY
        self.phase_ut = ut
        self.autostart = False
        self.stack = None
        self.limits = {}
        self.reserve_hist = []          # (ut, reserve kg)
        self.down_ticks = 0
        self.target_thrust = None
        self.sep_record = None
        self.booster_vessel = None
        self.extra = ["press START ASCENT"]
        self.finished = False
        self.exit_reason = None
        self.landed_prop = float("nan")  # booster propellant after landing
        self.landed_ut = 0.0
        self.launch_tries = 0
        self.lit_ut = 0.0               # phase-relative time of the last stage
        self.last = None

    # -- helpers -----------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s %s" % (self.state, state, note))
        self.state = state
        self.phase_ut = ut

    def finish(self, reason):
        self.finished = True
        self.exit_reason = reason

    def geometry(self, r, v):
        """Height above the pad, ground distance from it, and the velocity
        split into away-from-pad, across and up."""
        up = vec.unit(r)
        p_hat = vec.unit(self.pad)
        angle = vec.angle_between(up, p_hat)
        downrange = math.radians(angle) * vec.norm(self.pad)
        towards = vec.project_out(p_hat, up)
        if vec.norm(towards) < 1e-9:
            away = vec.unit(vec.project_out(v, up)) if vec.norm(
                vec.project_out(v, up)) > 1e-6 else (0.0, 0.0, 0.0)
        else:
            away = vec.scale(vec.unit(towards), -1.0)
        v_up = vec.dot(v, up)
        horiz = vec.project_out(v, up)
        v_away = vec.dot(horiz, away)
        v_cross = vec.norm(vec.project_out(horiz, away)) if vec.norm(
            away) > 0 else 0.0
        height = vec.norm(r) - vec.norm(self.pad)
        return budget.State(height, downrange, v_away, v_cross, v_up)

    def booster_model(self, stack):
        thrust_vac = sum(stack.thrust.values())
        thrust_sl, isp_v, isp_s = 0.0, 0.0, 0.0
        for e in stack.engines:
            t = stack.thrust[stack.engines.index(e)]
            isp_v += t / max(1.0, e.vacuum_specific_impulse)
            sl_t = e.max_thrust_at(self.pad_pressure)
            thrust_sl += sl_t
            isp_s += sl_t / max(1.0, e.specific_impulse_at(self.pad_pressure))
        self._isp = (thrust_vac / isp_v if isp_v else 1.0,
                     thrust_sl / isp_s if isp_s else 1.0)
        self._thrust_sl = thrust_sl

    def recovery(self, r, v):
        stack = self.stack
        b = budget.Booster(mass=stack.mass(), dry_mass=stack.dry,
                           thrust_vac=sum(stack.thrust.values()),
                           thrust_sl=self._thrust_sl,
                           isp_vac=self._isp[0], isp_sl=self._isp[1],
                           radius=stack.radius)
        state = self.geometry(r, v)
        rec = budget.recovery(self.model_body, b, state,
                              cd=self.cfg.LANDING_CD,
                              flip_rate_deg=self.cfg.FLIP_RATE_DEG_S)
        return rec, state, b

    def booster_thrust(self):
        return sum(e.thrust for e in self.stack.engines)

    def upper_accel(self, mass_total, booster_mass, throttle):
        f = 0.0
        for e in self.stack.upper:
            try:
                f += e.max_thrust * e.thrust_limit
            except Exception:           # noqa: BLE001
                pass
        upper_mass = max(1.0, mass_total - booster_mass)
        return f * max(throttle, 1e-3) / upper_mass, f, upper_mass

    def set_limits(self, demand):
        for k, x in allocate(demand, self.stack.groups,
                             self.stack.thrust).items():
            if abs(self.limits.get(k, -1.0) - x) > 0.005:
                try:
                    self.stack.engines[k].thrust_limit = x
                    self.limits[k] = x
                except Exception:       # noqa: BLE001
                    pass

    def release_limits(self):
        if self.stack is None:
            return
        for e in self.stack.engines:
            try:
                e.thrust_limit = 1.0
            except Exception:           # noqa: BLE001
                pass
        self.limits = {}

    # -- phases ------------------------------------------------------------
    def run_standby(self, ut):
        if self.autostart or self.gui.start_pressed():
            a = self.mj.ascent_autopilot
            a.ascent_type = int(self.cfg.ASCENT_TYPE)
            a.desired_orbit_altitude = float(self.cfg.ORBIT_ALT_M)
            a.desired_inclination = float(self.cfg.INCLINATION_DEG)
            a.autostage = False
            try:
                self.mj.staging_controller.enabled = False
            except Exception:           # noqa: BLE001
                pass
            a.enabled = True
            self.log.event(ut, "mechjeb ascent: type %d, %.0f m, inc %.1f, "
                           "autostage off; status %s"
                           % (a.ascent_type, a.desired_orbit_altitude,
                              a.desired_inclination, a.status))
            self.gui.set_start_label("RUNNING")
            self.enter(LAUNCH, ut)

    def run_launch(self, ut):
        """Stage until an engine is lit and the vehicle is off the pad --
        engines first, then any clamps -- and never a stage that decouples
        anything: that one is the booster's.

        A lit engine is ``active``, not ``thrust > 0``: MechJeb holds the
        throttle for its countdown, and a check on thrust staged straight
        into the booster's decoupler (7 flights of def1-1018)."""
        sit = self.vessel.situation
        lit = any(e.active for e in self.vessel.parts.engines)
        if lit and sit not in (self.sc.VesselSituation.pre_launch,
                               self.sc.VesselSituation.landed):
            self.stack = Stack(self.conn, self.vessel, self.log, ut)
            self.stack.build_groups(self.cfg)
            self.booster_model(self.stack)
            self.log.event(ut, "booster model: thrust vac %.0f kN sl %.0f kN, "
                           "isp vac %.0f sl %.0f, dry %.1f t; engine groups %s"
                           % (sum(self.stack.thrust.values()) / 1e3,
                              self._thrust_sl / 1e3, self._isp[0],
                              self._isp[1], self.stack.dry / 1e3,
                              [len(g) for g in self.stack.groups]))
            self.enter(ASCENT, ut, "liftoff")
            return
        waited = ut - self.phase_ut
        if lit:
            # Burning and still on the pad: clamps, or thrust still coming
            # up.  Release the next stage only if it decouples nothing.
            if waited - self.lit_ut < self.cfg.LAUNCH_CLAMP_WAIT_S:
                return
            nxt = self.control.current_stage - 1
            if nxt < 0 or any(p.decoupler is not None
                              for p in self.vessel.parts.in_stage(nxt)):
                if waited > self.cfg.LAUNCH_TIMEOUT_S:
                    raise RuntimeError("lit for %.0f s and still on the pad"
                                       % waited)
                return
        elif self.launch_tries and waited < 1.0 * self.launch_tries:
            return
        if self.launch_tries >= 4:
            raise RuntimeError("could not launch: stage %d, situation %s"
                               % (self.control.current_stage, sit))
        nxt = self.control.current_stage - 1
        if not lit and (nxt < 0 or any(
                p.decoupler is not None
                for p in self.vessel.parts.in_stage(nxt))):
            raise RuntimeError("the next stage decouples before any engine "
                               "is lit")
        self.control.throttle = 1.0
        self.control.activate_next_stage()
        self.launch_tries += 1
        self.lit_ut = waited
        self.log.event(ut, "launch: staged to %d" % self.control.current_stage)

    def run_ascent(self, ut, r, v):
        cfg = self.cfg
        rec, state, b = self.recovery(r, v)
        need = rec.propellant - rec.propellant_left
        reserve = rec.propellant_left - cfg.RESERVE_FRACTION * need
        self.reserve_hist.append((ut, reserve))
        self.reserve_hist = [x for x in self.reserve_hist if ut - x[0] <= 3.0]
        rate = 0.0
        if len(self.reserve_hist) >= 2:
            (t0, r0), (t1, r1) = self.reserve_hist[0], self.reserve_hist[-1]
            if t1 > t0:
                rate = (r1 - r0) / (t1 - t0)
        ahead = reserve + min(0.0, rate) * cfg.SEP_LEAD_S
        self.last = (rec, state, b, reserve)
        self.extra = [
            "booster : %7.1f t, prop %6.1f t" % (b.mass / 1e3,
                                                 rec.propellant / 1e3),
            "needs   : %6.1f t (bb %4.0f + land %3.0f m/s)"
            % (need / 1e3, rec.boostback_dv, rec.landing_dv),
            "reserve : %+6.1f t (ahead %+6.1f)" % (reserve / 1e3, ahead / 1e3),
            "mj      : %s" % self.mj.ascent_autopilot.status,
        ]
        if state.height < cfg.MIN_ALT_M:
            return
        if ahead <= 0.0:
            self.log.event(ut, "separation point: %s" % self.budget_text(
                rec, state, b, reserve))
            self.enter(THROTTLE_DOWN, ut)
            self.run_throttle_down(ut)
        elif rec.propellant <= 1.0:
            self.enter(THROTTLE_DOWN, ut, "booster out of propellant")

    def budget_text(self, rec, state, b, reserve):
        return ("alt=%.0f downrange=%.0f v_away=%.1f v_up=%.1f v_cross=%.1f "
                "booster=%.2ft prop=%.2ft bb_dv=%.0f burn=%.1fs apex=%.0f "
                "land_dv=%.0f v_t=%.0f left=%.2ft reserve=%+.2ft"
                % (state.height, state.downrange, state.v_away, state.v_up,
                   state.v_cross, b.mass / 1e3, rec.propellant / 1e3,
                   rec.boostback_dv, rec.burn_s, rec.apex, rec.landing_dv,
                   rec.terminal, rec.propellant_left / 1e3, reserve / 1e3))

    def run_throttle_down(self, ut):
        cfg = self.cfg
        throttle = self.control.throttle
        m_b = self.stack.mass()
        a_u, f_u, m_u = self.upper_accel(self.vessel.mass, m_b, throttle)
        target, _ = budget.separation_target(a_u, m_b, 0.0, cfg.SEP_ACCEL_RATIO)
        full = sum(e.max_thrust for e in self.stack.engines) * max(throttle,
                                                                   1e-3)
        demand = min(1.0, target / full) if full > 0.0 else 0.0
        self.set_limits(demand)
        got = self.booster_thrust()
        self.target_thrust = target
        self.extra = [
            "upper   : %6.0f kN / %6.1f t = %5.1f m/s2" % (f_u / 1e3, m_u / 1e3,
                                                         a_u),
            "booster : %6.0f kN, target %6.0f kN" % (got / 1e3, target / 1e3),
            "limits  : %s" % " ".join("%.2f" % self.limits.get(k, 1.0)
                                      for k in sorted(self.stack.thrust)),
        ]
        if got <= target * (1.0 + cfg.SEP_THRUST_TOL) + 1.0:
            self.down_ticks += 1
        else:
            self.down_ticks = 0
        waited = ut - self.phase_ut
        if (self.down_ticks >= cfg.SEP_CONFIRM_TICKS
                or waited >= cfg.SEP_THROTTLE_TIMEOUT_S):
            note = ("booster %.0f kN <= target %.0f kN (%.2f g free v upper "
                    "%.2f g) after %.2fs"
                    % (got / 1e3, target / 1e3, got / m_b / 9.80665,
                       a_u / 9.80665, waited))
            if self.down_ticks < cfg.SEP_CONFIRM_TICKS:
                note = "TIMEOUT: " + note
            self.enter(SEPARATE, ut, note)
            self.separate(ut)

    def separate(self, ut):
        stage = self.stack.stage
        while self.control.current_stage - 1 > stage:
            self.log.event(ut, "staging past %d to reach the booster's "
                           "decoupler" % (self.control.current_stage - 1))
            self.control.activate_next_stage()
        r = self.vessel.position(self.frame)
        v = self.vessel.velocity(self.frame)
        rec, state, b = self.recovery(r, v)
        self.sep_record = (ut, rec, state, b)
        self.log.event(ut, "separating: %s" % self.budget_text(
            rec, state, b, rec.propellant_left
            - self.cfg.RESERVE_FRACTION * (rec.propellant
                                           - rec.propellant_left)))
        self.control.activate_next_stage()
        booster = None
        for e in self.stack.engines:
            try:
                bv = e.part.vessel
                if bv != self.vessel:
                    booster = bv
                    break
            except Exception:           # noqa: BLE001
                continue
        if booster is None:
            raise RuntimeError("staged, but the booster is still attached")
        self.booster_vessel = booster
        try:
            booster.control.throttle = 0.0
        except Exception:               # noqa: BLE001
            pass
        # The limits stay on: a booster freed with its throttle up and its
        # limits released runs at full thrust into the upper stage until the
        # throttle write lands (2140 kN read here against a 1280 kN target,
        # LOG12760).  Whoever flies the booster next owns them.
        self.log.event(ut, "separated: booster is '%s', upper '%s', booster "
                       "thrust after %.0f kN"
                       % (booster.name, self.vessel.name,
                          self.booster_thrust() / 1e3))
        self.enter(HANDOFF, ut, self.cfg.HANDOFF)

    def run_handoff_upper(self, ut):
        a = self.mj.ascent_autopilot
        if self.cfg.MJ_AUTOSTAGE_AFTER_SEP and not a.autostage:
            a.autostage = True
            try:
                self.mj.staging_controller.enabled = True
            except Exception:           # noqa: BLE001
                pass
        orbit = self.vessel.orbit
        self.extra = ["apo %.0f peri %.0f" % (orbit.apoapsis_altitude,
                                              orbit.periapsis_altitude),
                      "mj      : %s" % a.status]
        if not a.enabled:
            self.log.event(ut, "orbit: apo %.0f peri %.0f inc %.2f; "
                           "mechjeb finished"
                           % (orbit.apoapsis_altitude,
                              orbit.periapsis_altitude,
                              math.degrees(orbit.inclination)))
            self.finish("orbit")
        elif ut - self.phase_ut > self.cfg.ORBIT_TIMEOUT_S:
            self.finish("orbit TIMEOUT")

    # -- loop --------------------------------------------------------------
    def tick(self):
        ut = self.sc.ut
        if self.rpc is not None:
            self.rpc.tick(self.state, ut)
        started = time.monotonic()
        if self.gui.terminate_pressed():
            self.finish("terminated")
            return ut
        if self.state == STANDBY:
            self.run_standby(ut)
        elif self.state == LAUNCH:
            self.run_launch(ut)
        elif self.state == ASCENT:
            r = self.vessel.position(self.frame)
            v = self.vessel.velocity(self.frame)
            self.run_ascent(ut, r, v)
        elif self.state == THROTTLE_DOWN:
            self.run_throttle_down(ut)
        elif self.state == HANDOFF and self.cfg.HANDOFF == "upper":
            self.run_handoff_upper(ut)
        self.loop_rate.sample(self.state, ut, time.monotonic() - started)
        if self.state in (ASCENT, THROTTLE_DOWN) and self.last is not None:
            rec, state, b, reserve = self.last
            if self.log.telemetry(ut, "%-13s alt=%7.0f dr=%7.0f va=%7.1f "
                                  "vu=%6.1f mb=%6.2ft prop=%6.2ft bb=%5.0f "
                                  "land=%4.0f left=%6.2ft res=%+6.2ft thr=%.2f"
                                  % (self.state, state.height, state.downrange,
                                     state.v_away, state.v_up, b.mass / 1e3,
                                     rec.propellant / 1e3, rec.boostback_dv,
                                     rec.landing_dv,
                                     rec.propellant_left / 1e3,
                                     reserve / 1e3, self.control.throttle)):
                pass
        try:
            self.gui.update(["phase   : %s" % self.state] + self.extra)
        except Exception:               # noqa: BLE001
            pass
        return ut

    def shutdown(self, reason):
        ut = 0.0
        try:
            ut = self.sc.ut
        except Exception:               # noqa: BLE001
            pass
        if self.state not in (HANDOFF, DONE):
            # Hand the vessel back: no thrust limits left, MechJeb off.
            # (After separation the limits are the booster's pilot's.)
            self.release_limits()
            try:
                self.mj.ascent_autopilot.enabled = False
            except Exception:           # noqa: BLE001
                pass
        self.log.event(ut, "loop rate, game-s per tick / wall-s of work: %s"
                       % self.loop_rate.report())
        if self.rpc is not None:
            self.log.event(ut, self.rpc.report())
        self.log.event(ut, "shutdown: %s" % reason)
        try:
            self.gui.close()
        except Exception:               # noqa: BLE001
            pass


def fly_booster(conn, ascent, args_set, pacing_game_time=False):
    """Switch to the dropped booster and fly boosterland on it.  Returns the
    boosterland log path and its exit reason."""
    from boosterland.autoland import Autoland
    from boosterland.config import Config as BoosterConfig
    from boosterland.config import apply_overrides as booster_overrides
    sc = conn.space_center
    booster = ascent.booster_vessel
    ut = sc.ut
    sc.active_vessel = booster
    deadline = time.time() + ascent.cfg.SWITCH_TIMEOUT_S
    while time.time() < deadline:
        try:
            if sc.active_vessel == booster:
                break
        except Exception:               # noqa: BLE001
            pass
        time.sleep(0.05)
    else:
        raise RuntimeError("switch to the booster timed out")
    bcfg = booster_overrides(BoosterConfig(), args_set)
    if pacing_game_time:
        bcfg.LOOP_PACING_GAME_TIME = True
    blog = Logbook(bcfg.LOG_DIR, bcfg.LOG_INTERVAL_UT)
    ascent.log.event(ut, "booster handed to boosterland: %s (switch took "
                     "%.2f s game)" % (blog.path, sc.ut - ut))
    run = None
    reason = "complete"
    try:
        run = Autoland(conn, bcfg, blog)
        run.autostart = True
        run.gui.hide_start()
        wait = sleeper(bcfg, lambda: sc.ut, conn)
        first = True
        while not run.finished:
            snap = run.tick()
            if first and (run.cluster is None
                          or len(run.cluster.groups) <= 1):
                # boosterland's EngineCluster writes every limit itself, but
                # only with more than one group; any other booster gets them
                # back here, after its first tick has cut the throttle.
                ascent.release_limits()
            first = False
            wait(bcfg.LOOP_SLEEP_S, snap.ut if snap is not None else sc.ut)
        reason = run.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted"
        raise
    except Exception as exc:            # noqa: BLE001
        blog.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
    finally:
        # Read what the booster landed with before boosterland's shutdown,
        # which closes the connection.
        try:
            # The booster's own resources: the stack's per-decouple-stage
            # view belonged to the vessel it has left.
            res = booster.resources
            ascent.landed_prop = sum(res.amount(n) * ascent.stack.density[n]
                                     for n in PROPELLANTS)
            ascent.landed_ut = sc.ut
        except Exception:               # noqa: BLE001
            pass
        if run is not None:
            run.shutdown(reason)
        blog.close()
    return blog.path, reason


def booster_landing_report(ascent):
    """What the booster landed with, against what was reserved for it."""
    prop = ascent.landed_prop
    ut, rec, state, b = ascent.sep_record
    need = rec.propellant - rec.propellant_left
    return ("booster propellant: %.2f t at separation, model needed %.2f t "
            "(reserve %.2f t); %.2f t left after landing"
            % (rec.propellant / 1e3, need / 1e3,
               ascent.cfg.RESERVE_FRACTION * need / 1e3, prop / 1e3))


def run(conn, cfg, log, autostart=False, booster_set=(), wait=None):
    ascent = Ascent(conn, cfg, log)
    ascent.rpc = RpcCounter()
    ascent.rpc.install(conn)
    ascent.autostart = autostart
    if autostart:
        ascent.gui.hide_start()
    if wait is None:
        wait = sleeper(cfg, lambda: conn.space_center.ut, conn)
    reason = "complete"
    try:
        while not ascent.finished:
            ut = ascent.tick()
            if ascent.state == HANDOFF and cfg.HANDOFF == "booster":
                break
            wait(cfg.LOOP_SLEEP_S, ut)
        if ascent.state == HANDOFF and cfg.HANDOFF == "booster":
            ascent.gui.close()
            path, breason = fly_booster(conn, ascent, booster_set,
                                        cfg.LOOP_PACING_GAME_TIME)
            # boosterland closed the connection: nothing below may use it.
            ascent.log.event(ascent.landed_ut,
                             "boosterland finished: %s (%s)" % (breason, path))
            ascent.log.event(ascent.landed_ut, booster_landing_report(ascent))
            ascent.finish("booster " + breason)
        reason = ascent.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
        raise
    except Exception as exc:            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
    finally:
        ascent.shutdown(reason)
    return ascent, reason


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=50000)
    parser.add_argument("--stream-port", type=int, default=50001)
    parser.add_argument("--name", default="ascent")
    parser.add_argument("--set", action="append", default=[],
                        metavar="FIELD=VALUE")
    parser.add_argument("--booster-set", action="append", default=[],
                        metavar="FIELD=VALUE",
                        help="a boosterland Config override for the landing")
    parser.add_argument("--autostart", action="store_true",
                        help="do not wait for the panel's START button")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    status = 0
    try:
        conn = krpc.connect(name=args.name, address=args.address,
                            rpc_port=args.rpc_port,
                            stream_port=args.stream_port)
        _, reason = run(conn, cfg, log, args.autostart, args.booster_set)
        if reason.startswith("error"):
            status = 1
    except KeyboardInterrupt:
        pass
    except Exception as exc:            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        status = 1
    finally:
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
