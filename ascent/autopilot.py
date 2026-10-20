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
from . import attitude, budget, turnshape
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
        self.sizes = {}
        self.radius = self._radius(logbook, ut)
        self.thrust = {i: e.max_vacuum_thrust or e.max_thrust
                       for i, e in enumerate(self.engines)}
        self.control_part = self._control_part(vessel, ids)
        if self.control_part is None:
            logbook.event(ut, "WARNING upper stage has no control source: "
                          "no crewed command part and no probe core above "
                          "the booster -- once separated nothing can "
                          "throttle or steer it, MechJeb included")
        logbook.event(ut, "booster: decouple stage %d, %d parts, %.1f t "
                      "(%.1f t propellant), %d engines, radius %.2f m; "
                      "upper stage: %d engines, control %s"
                      % (stage, len(self.parts), self.mass0 / 1e3, prop / 1e3,
                         len(self.engines), self.radius, len(self.upper),
                         self.control_part))

    @staticmethod
    def _control_part(vessel, booster_parts):
        """The upper stage's control source: a crewed command part or a
        probe core outside the booster; None when there is neither.  An
        uncrewed pod controls nothing -- qs_stack_small, launched empty,
        flew its upper stage at a throttle MechJeb could not move
        (LOG12992-13025, LOG13028)."""
        try:
            parts = vessel.parts.with_module("ModuleCommand")
        except Exception:               # noqa: BLE001
            return "unknown"
        for p in parts:
            if p in booster_parts:
                continue
            if p.crew_capacity == 0 or len(p.crew) > 0:
                return p.title
        return None

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
                self.sizes[p] = (hi[1] - lo[1], r)
        if best <= 0.0:
            best = 1.25
            logbook.event(ut, "booster radius: no usable tank bounding box, "
                          "assuming %.2f m" % best)
        return best

    def authority(self, gimbal_share):
        """The booster's turning authority once free, from its own parts:
        (gimbal rad/s^2 per unit throttle, RCS + wheels rad/s^2, pitch
        inertia kg m^2).  Its centre of mass and inertia are summed over the
        parts -- each a point mass plus, for a tank, a solid cylinder about
        its own centre -- in the booster's axis frame, because the vessel's
        own values are the whole stack's."""
        frame = self.vessel.reference_frame
        axis, a, b = self.axes
        rows = []
        for p in self.parts:
            try:
                m = p.mass
                pos = p.position(frame)
            except Exception:           # noqa: BLE001
                continue
            rows.append((p, m, vec.dot(pos, axis), vec.dot(pos, a),
                         vec.dot(pos, b)))
        total = sum(r[1] for r in rows) or 1.0
        za = sum(r[1] * r[2] for r in rows) / total
        xa = sum(r[1] * r[3] for r in rows) / total
        ya = sum(r[1] * r[4] for r in rows) / total
        inertia = 0.0
        for p, m, z, x, y in rows:
            radial2 = (x - xa) ** 2 + (y - ya) ** 2
            inertia += m * ((z - za) ** 2 + 0.5 * radial2)
            size = self.sizes.get(p)
            if size is not None:
                length, radius = size
                inertia += m * (length * length / 12.0 + radius * radius / 4.0)
        inertia = max(1.0, inertia)
        gimbal = 0.0
        for i, e in enumerate(self.engines):
            try:
                if not e.gimballed:
                    continue
                travel = math.radians(e.gimbal_range * e.gimbal_limit)
                z = vec.dot(e.part.position(frame), axis)
            except Exception:           # noqa: BLE001
                continue
            gimbal += self.thrust[i] * math.sin(travel) * abs(z - za)
        fixed = 0.0
        for p, m, z, x, y in rows:
            try:
                rcs = p.rcs
                wheel = p.reaction_wheel
            except Exception:           # noqa: BLE001
                continue
            if rcs is not None:
                try:
                    f = rcs.max_vacuum_thrust
                    # A radial nozzle pushes along the pitch axis it is
                    # wanted on for ~1/pi of directions, about its lever.
                    fixed += f * abs(z - za) / math.pi
                except Exception:       # noqa: BLE001
                    pass
            if wheel is not None:
                try:
                    tq = wheel.max_torque
                    fixed += min(tq[0], tq[2])
                except Exception:       # noqa: BLE001
                    pass
        return gimbal_share * gimbal / inertia, fixed / inertia, inertia

    def build_groups(self, cfg):
        """Group the booster's engines centre first, then pairs, in the plane
        perpendicular to their own thrust axis -- the vessel's frame is the
        root part's, which need not be the booster's axis."""
        frame = self.vessel.reference_frame
        rows = []
        axis = (0.0, 0.0, 0.0)
        for i, e in enumerate(self.engines):
            try:
                d = e.thrusters[0].thrust_direction(frame)
            except Exception:           # noqa: BLE001
                # A thruster's transform is null for a moment after the
                # staging that lit it (all three HANDOFF=upper flights of
                # 2026-10-18 died here); the part's axis is the same line.
                d = e.part.direction(frame)
            axis = vec.add(axis, vec.scale(d, self.thrust[i]))
        axis = vec.unit(axis)
        a = vec.unit(vec.project_out((1.0, 0.0, 0.0), axis))
        if vec.norm(vec.project_out((1.0, 0.0, 0.0), axis)) < 0.3:
            a = vec.unit(vec.project_out((0.0, 0.0, 1.0), axis))
        b = vec.cross(axis, a)
        self.axes = (axis, a, b)
        for i, e in enumerate(self.engines):
            p = e.part.position(frame)
            rows.append((i, (vec.dot(p, a), vec.dot(p, b)), self.thrust[i]))
        self.groups = group_engines(rows, cfg.ENGINE_AXIS_TOL_M,
                                    cfg.ENGINE_RING_TOL_M, cfg.ENGINE_PAIR_TOL)
        return self.groups


class Ascent:
    def __init__(self, conn, cfg, logbook, booster_set=()):
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
        from boosterland.config import apply_overrides as booster_overrides
        # The configuration boosterland will fly the booster with: the
        # budget has to model the flip it will actually fly.
        bcfg = booster_overrides(BoosterConfig(), list(booster_set))
        self.bcfg = bcfg
        self.flip_auth = None           # (gimbal, fixed, inertia), law only
        self.flip_auth_ut = -1e9
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
        self.throttle_seen = None       # main throttle at THROTTLE_DOWN
        self.lit_ut = 0.0               # phase-relative time of the last stage
        self.last = None
        self.att = None                 # pitch/heading/roll streams
        self.mj_settings = {}           # ascent_settings as applied
        self.settings_checked = False
        self.ignite_ut = None           # upper engines lit (upper_lit)

    # -- helpers -----------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s %s" % (self.state, state, note))
        self.state = state
        self.phase_ut = ut

    def finish(self, reason):
        self.finished = True
        self.exit_reason = reason

    def pad_roll(self):
        """The roll the vehicle sits at, against a pitch-over towards
        ``ROLL_REF_HEADING_DEG`` (``attitude.pad_roll``)."""
        right = self.sc.transform_direction(
            (1.0, 0.0, 0.0), self.vessel.reference_frame,
            self.vessel.surface_reference_frame)
        return attitude.pad_roll(right, self.cfg.ROLL_REF_HEADING_DEG)

    def attitude_text(self):
        """Pitch, heading and roll for the telemetry line, from streams."""
        if self.att is None:
            flight = self.vessel.flight(self.vessel.surface_reference_frame)
            add = self.conn.add_stream
            self.att = tuple(add(getattr, flight, k)
                             for k in ("pitch", "heading", "roll"))
        p, h, r = (s() for s in self.att)
        return " pit=%5.1f hdg=%5.1f roll=%+6.1f" % (p, h, r)

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

    def recovery(self, r, v, ut=None):
        stack = self.stack
        gimbal, fixed = None, 0.0
        if self.bcfg.FLIP_THROTTLE_LAW:
            if ut is not None and (self.flip_auth is None or ut
                                   - self.flip_auth_ut
                                   >= self.cfg.AUTHORITY_REFRESH_S):
                self.flip_auth = stack.authority(self.bcfg.FLIP_GIMBAL_SHARE)
                self.flip_auth_ut = ut
            if self.flip_auth is not None:
                gimbal, fixed = self.flip_auth[0], self.flip_auth[1]
        b = budget.Booster(mass=stack.mass(), dry_mass=stack.dry,
                           thrust_vac=sum(stack.thrust.values()),
                           thrust_sl=self._thrust_sl,
                           isp_vac=self._isp[0], isp_sl=self._isp[1],
                           radius=stack.radius, gimbal_accel=gimbal,
                           fixed_accel=fixed)
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
            self.mj_settings = self.ascent_settings(self.pad_roll())
            self.apply_settings(a, self.mj_settings)
            a.autostage = False
            try:
                self.mj.staging_controller.enabled = False
            except Exception:           # noqa: BLE001
                pass
            a.enabled = True
            self.log.event(ut, "mechjeb ascent: type %d, %.0f m (apoapsis "
                           "%.0f), inc %.1f, pitch from %.0f m/s at %.2f "
                           "deg/s, turn shape %.2f, roll %.1f (pad, vs "
                           "heading %.0f), "
                           "autostage off; status %s"
                           % (a.ascent_type, a.desired_orbit_altitude,
                              a.desired_apoapsis, a.desired_inclination,
                              a.pitch_start_velocity, a.pitch_rate,
                              a.turn_shape_exponent,
                              a.vertical_roll, self.cfg.ROLL_REF_HEADING_DEG,
                              a.status))
            self.gui.set_start_label("RUNNING")
            self.enter(LAUNCH, ut)

    def ascent_settings(self, roll):
        """What this script sets on MechJeb's ascent, by attribute."""
        cfg = self.cfg
        return {
            "ascent_type": int(cfg.ASCENT_TYPE),
            "desired_orbit_altitude": float(cfg.ORBIT_ALT_M),
            # PVG (MechJeb 2.15) targets periapsis *and* apoapsis; left at
            # its default, it burned every upper stage to the end -- the
            # small stacks onto escape, the shuttle stack to a 460 km
            # apoapsis on a suborbital path (LOG12916-12918).
            "desired_apoapsis": float(cfg.ORBIT_ALT_M),
            "desired_inclination": float(cfg.INCLINATION_DEG),
            "pitch_start_velocity": float(cfg.PITCH_START_VELOCITY),
            "pitch_rate": float(cfg.PITCH_RATE_DEG_S),
            "turn_shape_exponent": float(cfg.TURN_SHAPE),
            "force_roll": True,
            "vertical_roll": float(roll),
            "turn_roll": float(roll),
        }

    @staticmethod
    def apply_settings(a, settings):
        for k, v in settings.items():
            setattr(a, k, v)

    def check_settings(self, ut, why):
        """Re-read MechJeb's ascent settings and put back any that differ.
        MechJeb runs on one core per vessel: when the separation leaves the
        upper stage under a core that is not the one configured at launch
        (the small stack's booster carries the probe core), that core flies
        its own defaults -- a 100 km target, LOG12992-12996."""
        a = self.mj.ascent_autopilot
        off = []
        for k, v in self.mj_settings.items():
            try:
                now = getattr(a, k)
            except Exception:           # noqa: BLE001
                continue
            if (isinstance(v, float) and abs(now - v) > 1e-6 * max(1.0, abs(v))) \
                    or (not isinstance(v, float) and now != v):
                off.append("%s %s->%s" % (k, now, v))
                setattr(a, k, v)
        self.log.event(ut, "mechjeb settings %s: %s; enabled %s, status %s"
                       % (why, ", ".join(off) or "as set", a.enabled,
                          a.status))
        if off and not a.enabled:
            a.enabled = True

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
            if self.cfg.ASCENT_TYPE == 0:
                self.solve_turn_shape(ut)
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
        # No throttle write: kRPC re-applies a throttle a client has set on
        # every frame, so 1.0 written here pinned the throttle for the whole
        # flight and MechJeb could never cut the upper stage off (every
        # upper stage burned to depletion, LOG12919-12921).  MechJeb's
        # ascent throttles up itself.
        self.control.activate_next_stage()
        self.launch_tries += 1
        self.lit_ut = waited
        self.log.event(ut, "launch: staged to %d" % self.control.current_stage)

    def solve_turn_shape(self, ut):
        """Set MechJeb's classic turn shape to the one ``turnshape.solve``
        finds for this craft (see ``Config.TURN_SHAPE``)."""
        t0 = time.monotonic()
        stack, cfg = self.stack, self.cfg
        body, frame = self.body, self.frame
        a = self.mj.ascent_autopilot
        pad_alt = self.pad_alt
        b_thrust_vac = sum(stack.thrust.values())
        gimbal, fixed = None, 0.0
        if self.bcfg.FLIP_THROTTLE_LAW:
            gimbal, fixed, _ = stack.authority(self.bcfg.FLIP_GIMBAL_SHARE)
        booster = budget.Booster(
            mass=stack.mass(), dry_mass=stack.dry, thrust_vac=b_thrust_vac,
            thrust_sl=self._thrust_sl, isp_vac=self._isp[0],
            isp_sl=self._isp[1], radius=stack.radius, gimbal_accel=gimbal,
            fixed_accel=fixed)
        b_engines = turnshape.Engines(
            b_thrust_vac, self._isp[0],
            self._isp[0] + (self._isp[1] - self._isp[0])
            / max(1e-3, self.pad_pressure))
        f_u, inv_v, inv_s = 0.0, 0.0, 0.0
        for e in stack.upper:
            f = e.max_vacuum_thrust or e.max_thrust
            f_u += f
            inv_v += f / max(1.0, e.vacuum_specific_impulse)
            inv_s += f / max(1.0, e.specific_impulse_at(1.0))
        u_engines = turnshape.Engines(f_u, f_u / inv_v if inv_v else 1.0,
                                      f_u / inv_s if inv_s else 1.0)
        res = self.vessel.resources
        total_prop = sum(res.amount(n) * stack.density[n]
                         for n in PROPELLANTS)
        upper_mass = self.vessel.mass - booster.mass
        upper_prop = max(0.0, total_prop - stack.propellant())
        # drag at zero angle of attack, the vessel as it stands
        flight = self.vessel.flight(frame)
        up = vec.unit(self.vessel.position(frame))
        rot = self.vessel.rotation(frame)
        heights = [float(x) for x in cfg.DRAG_HEIGHTS_M.split(",")]
        speeds = [float(x) for x in cfg.DRAG_SPEEDS_MS.split(",")]
        r_pad = vec.norm(self.pad)
        table = []
        for h in heights:
            pos = vec.scale(up, r_pad + h)
            rho = body.density_at(pad_alt + h)
            row = []
            for v in speeds:
                vel = vec.scale(up, v)
                cda = None
                if rho > 1e-6:
                    try:
                        force = flight.simulate_aerodynamic_force_at(
                            body, pos, vel, rot)
                        d = -vec.dot(force, up)
                        if d > 0.0:
                            cda = d / (0.5 * rho * v * v)
                    except Exception:   # noqa: BLE001
                        pass
                row.append(cda)
            table.append(row)
        for i, row in enumerate(table):     # fill gaps from the row below
            for j, c in enumerate(row):
                if c is None:
                    row[j] = table[i - 1][j] if i else 0.0
        depth = body.atmosphere_depth if body.has_atmosphere else 0.0
        step = 1000.0
        n = int(max(0.0, depth - pad_alt) / step) + 2
        pres = [body.pressure_at(pad_alt + k * step) / ATM
                if body.has_atmosphere else 0.0 for k in range(n)]
        dens = [body.density_at(pad_alt + k * step)
                if body.has_atmosphere else 0.0 for k in range(n)]
        world = turnshape.World(self.model_body, body.rotational_speed,
                                pres, dens, step)
        if a.auto_path:
            start, end = a.auto_turn_start_altitude, a.auto_turn_end_altitude
        else:
            start, end = a.turn_start_altitude, a.turn_end_altitude
        path = turnshape.Path(start=start - pad_alt, end=end - pad_alt,
                              end_angle=a.turn_end_angle,
                              orbit=cfg.ORBIT_ALT_M - pad_alt,
                              reserve=cfg.RESERVE_FRACTION,
                              landing_cd=cfg.LANDING_CD,
                              flip_rate_deg=cfg.FLIP_RATE_DEG_S)
        craft = turnshape.Craft(
            booster, b_engines, upper_mass, upper_prop, u_engines,
            any(e.active for e in stack.upper), heights, speeds, table)
        best, flown = turnshape.solve(craft, world, path)
        curve = " ".join("%.2f:%.1f" % (r.shape, r.mass / 1e3)
                         for r in sorted(flown, key=lambda r: r.shape)[::2])
        if math.isfinite(best.mass):
            a.turn_shape_exponent = float(best.shape)
            self.mj_settings["turn_shape_exponent"] = float(best.shape)
        self.log.event(ut, "turn shape: %s %.3f (orbit %.2f t, sep %.0f m "
                       "%.0f m out at %.0f s, circ %.0f m/s) path %.0f-%.0f m"
                       " end %.0f deg; upper %.1f t (%.1f prop) %.0f kN "
                       "isp %.0f/%.0f lit %s; cda %s; %.2f s wall; curve %s"
                       % ("solved" if math.isfinite(best.mass)
                          else "NO ORBIT, kept", a.turn_shape_exponent,
                          best.mass / 1e3, best.sep_height,
                          best.sep_downrange, best.sep_time, best.circ_dv,
                          start, end, a.turn_end_angle, upper_mass / 1e3,
                          upper_prop / 1e3, f_u / 1e3, u_engines.isp_vac,
                          u_engines.isp_sl, craft.upper_lit,
                          "/".join("%.1f" % c for c in table[1]),
                          time.monotonic() - t0, curve))
        if math.isfinite(best.mass):
            # the predicted flight, for shapetrack.py against the telemetry
            tr = turnshape.fly(craft, world, path, best.shape,
                               track_every=cfg.SHAPE_TRACK_S).track
            self.log.event(ut, "turn shape track: " + " ".join(
                "%.0f:%.0f/%.0f/%.0f" % p for p in tr))

    def run_ascent(self, ut, r, v):
        cfg = self.cfg
        rec, state, b = self.recovery(r, v, ut)
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

    def main_throttle(self):
        """The main throttle MechJeb is flying, from what the booster's
        engines deliver against what their limits allow.  kRPC's
        ``control.throttle`` does not read MechJeb's throttle (0.00 through
        a full-thrust burn, LOG12922)."""
        if self.throttle_seen is not None:
            return self.throttle_seen
        got, cap = 0.0, 0.0
        for i, e in enumerate(self.stack.engines):
            try:
                lim = e.thrust_limit
                if lim < 0.05:
                    continue
                got += e.thrust
                cap += e.max_thrust * lim
            except Exception:           # noqa: BLE001
                continue
        return min(1.0, got / cap) if cap > 0.0 else 1.0

    def run_throttle_down(self, ut):
        cfg = self.cfg
        # Read once, before the limits move: once they do, the engines'
        # thrust lags the new limit and the ratio is not the throttle.
        throttle = self.main_throttle()
        self.throttle_seen = throttle
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
            if not self.upper_lit(ut):
                return
            note = ("booster %.0f kN <= target %.0f kN (%.2f g free v upper "
                    "%.2f g) after %.2fs"
                    % (got / 1e3, target / 1e3, got / m_b / 9.80665,
                       a_u / 9.80665, waited))
            if self.down_ticks < cfg.SEP_CONFIRM_TICKS:
                note = "TIMEOUT: " + note
            self.enter(SEPARATE, ut, note)
            self.separate(ut)

    def upper_lit(self, ut):
        """Light the upper stage's engines before the decoupler fires, and
        report whether they are delivering their thrust yet.

        Staged together, the decoupler lets go in the frame before the
        upper engines make thrust, and the booster -- still at
        SEP_ACCEL_RATIO of an upper-stage acceleration that is not there
        yet -- rams the upper stage: the shuttle stack's delta wings broke
        off on 10 of 17 flights (CollisionSpy JOINTBREAK in the decoupling
        frame, def2-1019, sepcheck-1019).  Lit first, each stage is already
        under its own thrust when the joint goes."""
        cfg = self.cfg
        cold = [e for e in self.stack.upper if not e.active]
        if self.ignite_ut is None:
            self.ignite_ut = ut
            for e in cold:
                try:
                    e.active = True
                except Exception:       # noqa: BLE001
                    pass
            if cold:
                self.log.event(ut, "upper stage: lighting %d engines before "
                               "the decoupler" % len(cold))
        want = sum(e.max_thrust for e in self.stack.upper) \
            * max(self.throttle_seen or 1.0, 1e-3)
        got = sum(e.thrust for e in self.stack.upper)
        waited = ut - self.ignite_ut
        if got >= cfg.SEP_UPPER_THRUST_FRACTION * want:
            self.log.event(ut, "upper stage lit: %.0f of %.0f kN after %.2f s"
                           % (got / 1e3, want / 1e3, waited))
            return True
        if waited >= cfg.SEP_IGNITE_TIMEOUT_S:
            self.log.event(ut, "upper stage lit TIMEOUT: %.0f of %.0f kN "
                           "after %.2f s; separating anyway"
                           % (got / 1e3, want / 1e3, waited))
            return True
        return False

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
        if self.flip_auth is not None:
            self.log.event(ut, "booster flip model (FLIP_THROTTLE_LAW): "
                           "gimbal %.3f rad/s2 per throttle, rcs+wheels "
                           "%.3f, pitch inertia %.0f t m2"
                           % (self.flip_auth[0], self.flip_auth[1],
                              self.flip_auth[2] / 1e3))
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
        if not self.settings_checked:
            self.settings_checked = True
            self.check_settings(ut, "after separation")
        a = self.mj.ascent_autopilot
        if self.cfg.MJ_AUTOSTAGE_AFTER_SEP and not a.autostage:
            a.autostage = True
            try:
                self.mj.staging_controller.enabled = True
            except Exception:           # noqa: BLE001
                pass
        orbit = self.vessel.orbit
        apo, peri = orbit.apoapsis_altitude, orbit.periapsis_altitude
        status = a.status
        self.extra = ["apo %.0f peri %.0f" % (apo, peri),
                      "mj      : %s" % status]
        thrust = sum(e.thrust for e in self.stack.upper)
        mass = self.vessel.mass
        self.log.telemetry(ut, "UPPER         apo=%9.0f peri=%9.0f "
                           "thrust=%5.0fkN m=%6.2ft mj=%s"
                           % (apo, peri, thrust / 1e3, mass / 1e3, status))
        sit = self.vessel.situation
        if mass <= 0.0 or sit in (self.sc.VesselSituation.landed,
                                  self.sc.VesselSituation.splashed):
            # a stage that hit the ground reads mass 0 and MechJeb's status
            # never changes: without this the loop flew the wreck to the
            # timeout (shape1-1019, TURN_SHAPE 0.3)
            self.log.event(ut, "upper stage lost: %s, mass %.2f t"
                           % (sit, mass / 1e3))
            self.finish("upper stage lost")
        elif not a.enabled:
            self.log.event(ut, "orbit: apo %.0f peri %.0f inc %.2f mass "
                           "%.3f t; mechjeb finished"
                           % (orbit.apoapsis_altitude,
                              orbit.periapsis_altitude,
                              math.degrees(orbit.inclination),
                              self.vessel.mass / 1e3))
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
                                     reserve / 1e3, self.main_throttle()
                                     if self.stack else 1.0)
                                  + self.attitude_text()):
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
    ascent = Ascent(conn, cfg, log, booster_set)
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
