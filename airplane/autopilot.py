"""Airplane: an attitude autopilot for winged craft, with the player's stick
on top of it.

Phase machine::

    STANDBY -> FLY -> DONE

Two modes, picked on the panel and switchable in flight:

* **CRUISE** -- sideslip nulled on the rudder, bank held (never past
  ``BANK_LIMIT_DEG``, 89), pitch *attitude* held, every turn coordinated:
  banked, it asks for the turn rate ``g tan(phi) / V`` and so pulls the
  stick back by exactly what keeps the nose where it was (at 30 deg up and
  45 of bank it pulls until 30 is held).  Heading is free.
* **HOLD** -- pitch, heading and bank all held where they were taken.

Each mode turns its attitude errors into Euler rates, those into body rates
(``control.euler_to_body``), and each body rate is flown by a PI loop on
the stick (``control.RateLoop``).  **No gain is fitted to a craft by
hand**: each axis's response is measured in flight (``control.Plant``,
starting from kRPC's available torque over the moment of inertia) and its
PI gains place both poles of a loop that closes in a few of its own
measured ticks (``control.pi_gains``); an axis that rings has its gain
halved (``control.Ringing``).

**The player's stick adds to it.**  kRPC adds a script's input to the
player's, so the player's own is read back as what the game flew less ours
and the trim (``control.stick``).  Off centre on an axis, the hold on that
axis lets go (its trim stays) and, once the stick is back, holds the
attitude it was left at.  With **constant rates** checked the stick
commands a body rate instead -- full deflection = the panel's limit for
that axis, in rev/min -- and the pilot flies it, so at high speed a full
stick turns no faster than the limit; the holds' own rates are capped by
the same limits.  **Prop plane** runs ``propPitch`` beside it (its own
log), so the blades are flown too.

TERMINATE (or ``RUN_TIME_S``) hands the stick back at zero and SAS as it was.
"""

import math
import sys
import time

from common import cli
from common.krpcbatch import ask
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from common.panel import (ACCENT_AIRPLANE, ControlPanel, choice, field,
                          service, toggle)
from common.rpccount import RpcCounter
from . import control as law
from .config import Config, apply_overrides, defaults_fingerprint, differences

STANDBY = "STANDBY"
FLY = "FLY"
DONE = "DONE"

CRUISE = 0
HOLD = 1
MODES = ("CRUISE", "HOLD")
AXES = ("pitch", "roll", "yaw")
WRITE_STEP = 0.001


class AirplanePilot:
    def __init__(self, conn, cfg, log):
        self.conn = conn
        self.cfg = cfg
        self.log = log
        self.sc = conn.space_center
        self.vessel = self.sc.active_vessel
        self.state = STANDBY
        self.finished = False
        self.exit_reason = None
        self.autostart = False
        self.rpc = None
        self.loop_rate = LoopRate()
        self.mode = None
        self.target = None          # (phi, theta, psi), rad
        self.manual = [False, False, False]     # pitch, roll, yaw
        self.gates = None
        self.ground = None          # on the wheels/floats (None: not read yet)
        self.written = [0.0, 0.0, 0.0]
        self.loops = None
        self.plants = None
        self.flown = None           # (accel, input, rate) of the last tick
        self.ringing = None
        self.accel = None
        self.acts = ((None, 0.0),) * 3
        svc = service(conn)
        self.actuator_call = (getattr(svc, "surface_actuators")
                              if svc is not None else None)
        self.next_authority = 0.0
        self.interval = None
        self.slip_i = 0.0
        self.last_ut = None
        self.sas_was = None
        self.prop = None
        self.prop_error = None
        self.prop_lines = []
        self.lines = []
        self.start_ut = None
        ut = self.sc.ut
        log.event(ut, "vessel=%s parts=%d" % (self.vessel.name,
                                               len(self.vessel.parts.all)))
        log.event(ut, "config: %s defaults_fingerprint=%s"
                  % (" ".join(differences(cfg)) or "(defaults)",
                     defaults_fingerprint()))
        body = self.vessel.orbit.body
        self.vf = self.vessel.reference_frame
        self.sf = self.vessel.surface_reference_frame
        self.surface_flight = self.vessel.flight(body.reference_frame)
        # Velocity through the air (the body's rotating frame) in the
        # vessel's own axes.
        self.air_flight = self.vessel.flight(
            self.sc.ReferenceFrame.create_hybrid(
                position=body.reference_frame, rotation=self.vf))
        self.gui = ControlPanel(conn, "AIRPLANE", "START", ACCENT_AIRPLANE, [
            choice("mode", "", MODES, cfg.MODE),
            toggle("prop", "prop plane", cfg.PROP_PLANE),
            toggle("rates", "constant rates", cfg.CONSTANT_RATES),
            field("rpm_pitch", "pitch rate limit (RPM)", cfg.PITCH_RATE_RPM),
            field("rpm_roll", "roll rate limit (RPM)", cfg.ROLL_RATE_RPM),
            field("rpm_yaw", "yaw rate limit (RPM)", cfg.YAW_RATE_RPM),
        ])

    # -- takeover -----------------------------------------------------------
    def take_over(self, ut):
        cfg = self.cfg
        self.loops = [law.RateLoop(cfg.INTEGRAL_MAX) for _ in AXES]
        self.plants = [law.Plant(cfg.PLANT_FORGET_S, cfg.PLANT_P_MAX,
                                 cfg.PLANT_K_MIN, cfg.PLANT_K_MAX,
                                 cfg.PLANT_D_MAX, cfg.PLANT_EXCITE,
                                 cfg.PLANT_WINDOW_S)
                       for _ in AXES]
        self.flown = None
        self.ringing = [law.Ringing(cfg.RING_REVERSALS, cfg.RING_SWING,
                                    cfg.RING_WINDOW_S, cfg.RING_RECOVER_PER_S,
                                    cfg.RING_FLOOR) for _ in AXES]
        self.gates = [law.StickGate(cfg.STICK_DEADBAND, cfg.STICK_FIRM,
                                    cfg.STICK_CONFIRM_S) for _ in AXES]
        self.refresh_authority(ut)
        c = self.vessel.control
        if cfg.SAS_OFF:
            self.sas_was = c.sas
            c.sas = False
        self.written = [0.0, 0.0, 0.0]
        c.pitch, c.roll, c.yaw = self.written
        self.start_ut = ut

    def refresh_authority(self, ut):
        """Angular acceleration per unit input on each axis, from kRPC's
        available torque (the surfaces at the present q and Mach) over the
        moment of inertia; logged when it is first read."""
        torque, inertia = ask(self.conn, [
            (getattr, (self.vessel, "available_torque")),
            (getattr, (self.vessel, "moment_of_inertia"))])
        first = self.accel is None
        self.accel = law.authority(torque, inertia)
        self.next_authority = ut + self.cfg.AUTHORITY_REFRESH_S
        # The surfaces' actuators, from the game (the window mod); without
        # it, every axis is taken as instant.
        acts = None
        if self.actuator_call is not None:
            try:
                acts = law.actuators(list(self.actuator_call()))
            except Exception:                           # noqa: BLE001
                acts = None
        self.acts = acts or ((None, 0.0),) * 3
        if first:
            self.log.event(ut, "authority (pitch, roll, yaw) deg/s^2 per unit "
                           "input: %s; inertia %s kg m^2" % (
                               ",".join("%.1f" % math.degrees(a)
                                        if a is not None else "?"
                                        for a in self.accel),
                               ",".join("%.0f" % x for x in inertia or ())))
            self.log.event(ut, "surface actuators (pitch, roll, yaw): slew %s "
                           "full travel/s, lag %s s%s" % (
                               ",".join("%.2f" % sl if sl else "instant"
                                        for sl, _ in self.acts),
                               ",".join("%.2f" % lg for _, lg in self.acts),
                               "" if acts else " (not read: no window mod)"))

    def taus(self, caps):
        """Each axis's (pitch, roll, yaw) rate-loop closing time: at least
        RATE_TAU_TICKS of the loop's own measured interval (and
        RATE_TAU_MIN_S), and no faster than its surfaces can move
        (``control.closing_time``, a step of the axis's rate cap)."""
        cfg = self.cfg
        tick = self.interval if self.interval is not None else cfg.LOOP_SLEEP_S
        base = max(cfg.RATE_TAU_MIN_S, cfg.RATE_TAU_TICKS * tick)
        # On kRPC's figure, not the measured effect: the measured effect
        # (roll ~6x kRPC's on the Warbird) shortened roll's tau 0.37 ->
        # 0.26 s and its reversals rose 0.36 -> 0.91/s (CRUISE), 0.70 ->
        # 1.33/s (HOLD), LOG15387-15396 against LOG15363-15382.  This floor
        # is holding roll at a closing time the per-axis model cannot
        # explain (roll-yaw coupling, unmodelled, is the suspect).
        return [law.closing_time(base, self.accel[i], self.acts[i][0],
                                 self.acts[i][1], caps[i], cfg.SLEW_FACTOR,
                                 cfg.LAG_FACTOR) for i in range(3)]

    def effect(self, i):
        """Axis i's angular acceleration per unit input (rad/s^2): kRPC's
        figure (1 where it has none) times what the fit makes of it."""
        acc = self.accel[i] if self.accel[i] else 1.0
        return self.plants[i].k * acc if self.plants else acc

    def hand_back(self, ut):
        c = self.vessel.control
        try:
            c.pitch, c.roll, c.yaw = 0.0, 0.0, 0.0
        except Exception:                               # noqa: BLE001
            pass
        if self.sas_was is not None:
            try:
                c.sas = self.sas_was
            except Exception:                           # noqa: BLE001
                pass

    # -- the panel ----------------------------------------------------------
    def limits(self):
        """The panel's rate limits, rad/s (pitch, roll, yaw); a field that
        does not parse (or is <= 0) keeps the config's."""
        cfg = self.cfg
        out = []
        for cid, default in (("rpm_pitch", cfg.PITCH_RATE_RPM),
                             ("rpm_roll", cfg.ROLL_RATE_RPM),
                             ("rpm_yaw", cfg.YAW_RATE_RPM)):
            v = self.gui.number(cid, default)
            out.append((v if v > 0.0 else default) * law.RPM)
        return out

    def run_prop(self, ut, on):
        """propPitch beside the airplane, started and stopped by the box."""
        if not on:
            if self.prop is not None:
                self.stop_prop("prop plane off")
            self.prop_error = None
            return
        if self.prop_error is not None:
            return
        if self.prop is None:
            from propPitch.autopilot import PropPilot
            from propPitch.config import Config as PropConfig
            pcfg = PropConfig()
            plog = Logbook(pcfg.LOG_DIR, pcfg.LOG_INTERVAL_UT)
            try:
                self.prop = PropPilot(self.conn, pcfg, plog, gui=ControlPanel(
                    self.conn, "PROP PITCH", "START", ACCENT_AIRPLANE,
                    draw=False))
            except Exception as exc:                    # noqa: BLE001
                plog.event(ut, "FATAL %r" % exc)
                plog.close()
                self.prop_error = repr(exc)
                self.log.event(ut, "prop plane: not started: %r" % exc)
                return
            self.prop.autostart = True
            self.prop.plog = plog
            self.log.event(ut, "prop plane: propPitch on, %s" % plog.path)
        try:
            self.prop.tick()
            # Its panel lines, "name    : value": the blades' angle, the
            # rpm and what they are doing.
            got = dict((x.split(":", 1)[0].strip(), x.split(":", 1)[1].strip())
                       for x in self.prop.lines if ":" in x)
            self.prop_lines = ["prop    : %s" % " / ".join(
                got[k] for k in ("pitch", "rpm", "mode") if k in got)]
        except Exception as exc:                        # noqa: BLE001
            self.prop.plog.event(ut, "FATAL %r" % exc)
            self.prop_error = repr(exc)
            self.stop_prop("error: %r" % exc)

    def stop_prop(self, reason):
        prop, self.prop = self.prop, None
        self.prop_lines = []
        if prop is None:
            return
        try:
            prop.shutdown(reason)
        finally:
            prop.plog.close()
        self.log.event(0.0, "prop plane: propPitch off (%s)" % reason)

    # -- loop ---------------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s%s" % (self.state, state,
                                                 (": " + note) if note else ""))
        self.state = state

    def finish(self, reason):
        self.exit_reason = reason
        self.finished = True

    def read(self):
        vf, sf, c = self.vf, self.sf, self.vessel.control
        calls = [(self.sc.transform_direction, ((0.0, 1.0, 0.0), vf, sf)),
                 (self.sc.transform_direction, ((1.0, 0.0, 0.0), vf, sf)),
                 (self.sc.transform_direction, ((0.0, 0.0, 1.0), vf, sf)),
                 (self.vessel.angular_velocity, (sf,)),
                 (getattr, (self.air_flight, "velocity")),
                 (getattr, (self.surface_flight, "speed")),
                 (getattr, (self.surface_flight, "dynamic_pressure")),
                 (getattr, (self.surface_flight, "mean_altitude"))]
        calls += [(getattr, (c, a)) for a in AXES]
        calls += [(getattr, (c, a + "_trim")) for a in AXES]
        calls.append((getattr, (self.vessel, "situation")))
        return ask(self.conn, calls)

    def run_fly(self, ut):
        cfg = self.cfg
        got = self.read()
        nose, right, belly, omega, v_air, speed, q_dyn, alt = got[:8]
        reads = list(got[8:11])
        trims = list(got[11:14])
        ground = law.on_ground(got[14])
        dt = 0.0 if self.last_ut is None else max(0.0, ut - self.last_ut)
        self.last_ut = ut
        if dt > 0.0:
            self.interval = (dt if self.interval is None
                             else 0.9 * self.interval + 0.1 * dt)
        if ut >= self.next_authority:
            self.refresh_authority(ut)
        if None in (nose, right, belly, omega):
            return
        roof = tuple(-x for x in belly)
        phi, theta, psi = law.attitude(nose, right, roof)
        p, q, r = law.body_rates(omega, nose, right, roof)
        beta = law.sideslip(v_air)
        speed = speed or 0.0

        mode = self.gui.value("mode")
        mode = mode if mode in (CRUISE, HOLD) else CRUISE
        constant = bool(self.gui.value("rates"))
        lim = self.limits()
        if mode != self.mode:
            self.log.event(ut, "mode %s" % MODES[mode])
            self.mode = mode
            self.target = None
        if ground != self.ground:
            if self.ground is not None:
                self.log.event(ut, "lift-off" if not ground else "touchdown")
            self.ground = ground
            # Off the ground the hold takes the attitude it finds: the
            # pitch the take-off left it at, the heading it is on; the
            # slip loop starts afresh.
            if self.target is not None and not ground:
                self.target[1], self.target[2] = theta, psi
            self.slip_i = 0.0
        vertical = abs(theta) > math.radians(cfg.VERTICAL_DEG)
        bank_cap = math.radians(cfg.BANK_LIMIT_DEG)

        # The player's stick, per axis (pitch, roll, yaw).
        sticks = [law.stick(rd, w, t)
                  for rd, w, t in zip(reads, self.written, trims)]
        active = [g.update(s, dt) for g, s in zip(self.gates, sticks)]
        # Where the attitude is now is the target on every axis the stick
        # moves (and was moving): it is held once the stick lets go.
        if self.target is None:
            self.target = [phi, theta, psi]
        tphi, ttheta, tpsi = self.target
        # On the ground the gear holds pitch: it is not flown (the tail
        # wheel would wind its trim against the nose all down the runway
        # and pitch the plane up at lift-off), and the wings are held
        # level.
        if ground:
            tphi = 0.0
        if active[1] or self.manual[1]:
            tphi = phi
        if active[0] or self.manual[0] or ground:
            ttheta = theta
        if active[2] or self.manual[2] or vertical:
            tpsi = psi
        if mode == CRUISE:
            tphi = law.clamp(tphi, -bank_cap, bank_cap)
        # The rates each axis may be asked for (pitch, roll, yaw), and from
        # them and the surfaces each axis's closing time.
        caps = [math.radians(cfg.MAX_PITCH_RATE_DEG_S),
                math.radians(cfg.MAX_ROLL_RATE_DEG_S),
                math.radians(cfg.MAX_YAW_RATE_DEG_S)]
        if constant:
            caps = [min(c, x) for c, x in zip(caps, lim)]
        taus = self.taus(caps)
        k_att = [1.0 / (cfg.ATTITUDE_TAU_FACTOR * t) for t in taus]
        for i, a in enumerate(active):
            if a != self.manual[i]:
                self.log.event(ut, "stick %s %s" % (
                    AXES[i], "flying" if a else "released"))
        self.manual = active
        self.target = [tphi, ttheta, tpsi]

        # Outer loops: attitude error -> Euler rates.
        dphi = k_att[1] * law.wrap(tphi - phi)
        dtheta = k_att[0] * (ttheta - theta)
        if mode == CRUISE and not ground:
            # The turn the bank makes, coordinated (and the bank cap's
            # bank rather than the vessel's own, past it).
            dpsi = 0.0 if vertical else law.turn_rate(
                law.clamp(phi, -bank_cap, bank_cap), speed,
                cfg.TURN_MAX_G, cfg.TURN_MIN_SPEED_M_S)
        else:
            # HOLD's heading, and on the ground either mode's: the rudder
            # and the steering keep it on the runway.
            dpsi = k_att[2] * law.wrap(tpsi - psi)
        want = list(law.euler_to_body(phi, theta, dphi, dtheta, dpsi))
        if (mode == CRUISE and beta is not None and not active[2]
                and not ground):
            # The rudder takes the slip out.
            if dt > 0.0:
                cap = math.radians(cfg.SLIP_I_MAX_DEG_S)
                ki = k_att[2] / (cfg.INTEGRAL_TAU_FACTOR
                                 * cfg.ATTITUDE_TAU_FACTOR * taus[2])
                self.slip_i = law.clamp(self.slip_i + ki * beta * dt,
                                        -cap, cap)
            want[2] += k_att[2] * beta + self.slip_i
        # want/rate in the stick's order: pitch (q), roll (p), yaw (r).
        want = [want[1], want[0], want[2]]
        want = [law.clamp(w, -c, c) for w, c in zip(want, caps)]
        rate = [q, p, r]
        # What the last tick's input did: each axis's plant, measured --
        # in the air only: on the ground the gear answers the input too.
        if self.flown is not None and not ground:
            for i in range(3):
                acc_i, u_i, w_i = self.flown[i]
                self.plants[i].update(acc_i, u_i, w_i, rate[i], dt)

        out = []
        kps = []
        tick = self.interval if self.interval is not None else cfg.LOOP_SLEEP_S
        for i in range(3):
            s = sticks[i] or 0.0
            # Closed on this tick, when it is longer than the mean one (the
            # Warbird's roll rate flipped sign tick to tick on 0.15-0.2 s
            # ticks, gains sized on 0.10, LOG15423).
            taus[i] = law.tick_tau(taus[i], max(dt, tick), cfg.RATE_TAU_TICKS)
            kp, ki = law.pi_gains(self.effect(i), self.plants[i].d, taus[i],
                                  cfg.KP_MAX)
            slew = self.acts[i][0]
            ring = self.ringing[i]
            kp, ki = kp * ring.mult, ki * ring.mult
            kps.append(kp)
            if (active[i] and not constant) or (i == 0 and ground
                                                and not active[i]):
                # The player flies the axis (or, on the ground, the gear
                # holds the pitch); the hold keeps only its trim.
                out.append(self.loops[i].step(want[i], rate[i], kp, ki, dt,
                                              integrate=False,
                                              proportional=False))
                continue
            if active[i]:
                # The stick is a rate command; the game adds the player's
                # input to ours, so ours is the loop's less the stick.
                want[i] = s * lim[i]
            u = self.loops[i].step(want[i], rate[i], kp, ki, dt, slew=slew)
            if ring.update(ut, u, dt):
                self.log.event(ut, "%s ringing: gain x%.3f" % (AXES[i],
                                                               ring.mult))
            out.append(law.clamp(u - s, -1.0, 1.0) if active[i] else u)
        self.command(out)
        # The input this tick flies until the next: ours, the player's stick
        # and the trim (KSP adds all three).
        self.flown = [(self.accel[i], out[i] + (sticks[i] or 0.0)
                       + (trims[i] or 0.0), rate[i]) for i in range(3)]

        if cfg.RUN_TIME_S > 0 and ut - self.start_ut >= cfg.RUN_TIME_S:
            self.finish("run time")
        deg = math.degrees
        self.log.telemetry(ut, (
            "FLY %s spd=%6.1f alt=%6.0f q=%6.0f att=%+6.1f,%+6.1f,%5.1f "
            "tgt=%+6.1f,%+6.1f,%5.1f slip=%+5.1f rate=%+6.1f,%+6.1f,%+6.1f "
            "want=%+6.1f,%+6.1f,%+6.1f out=%+.3f,%+.3f,%+.3f "
            "i=%+.3f,%+.3f,%+.3f stick=%s kp=%.2f,%.2f,%.2f "
            "acc=%s k=%.2f,%.2f,%.2f d=%.1f,%.1f,%.1f fitq=%.2f,%.2f,%.2f "
            "ring=%.2f,%.2f,%.2f tau=%.2f,%.2f,%.2f gnd=%d%s"
            % (MODES[mode], speed, alt or 0.0, q_dyn or 0.0,
               deg(phi), deg(theta), deg(psi) % 360.0,
               deg(tphi), deg(ttheta), deg(tpsi) % 360.0,
               deg(beta) if beta is not None else float("nan"),
               *[deg(x) for x in rate], *[deg(x) for x in want], *out,
               *[lp.i for lp in self.loops],
               ",".join("%+.2f" % s if s is not None else "?" for s in sticks),
               *kps, ",".join("%.1f" % math.degrees(a) if a is not None
                              else "?" for a in self.accel),
               *[pl.k for pl in self.plants], *[pl.d for pl in self.plants],
               *[pl.quality for pl in self.plants],
               *[rg.mult for rg in self.ringing], *taus, int(ground),
               " const" if constant else "")))
        self.lines = [
            "mode    : %s%s%s" % (MODES[mode], ", constant rates" if constant
                                  else "", ", on the ground" if ground
                                  else ""),
            "speed   : %.1f m/s  alt %.0f m" % (speed, alt or 0.0),
            "bank    : %+.1f (hold %+.1f)" % (deg(phi), deg(tphi)),
            "pitch   : %+.1f (hold %+.1f)" % (deg(theta), deg(ttheta)),
            ("heading : %.1f (hold %.1f)" % (deg(psi) % 360, deg(tpsi) % 360)
             if mode == HOLD else "slip    : %+.1f deg" % (
                 deg(beta) if beta is not None else 0.0)),
            "stick   : %s" % " ".join(
                "%s%s" % (a[0], "*" if m else "") for a, m in
                zip(AXES, self.manual)),
        ]

    def command(self, out):
        c = self.vessel.control
        for i, (a, u) in enumerate(zip(AXES, out)):
            if abs(u - self.written[i]) >= WRITE_STEP:
                setattr(c, a, u)
                self.written[i] = u

    def tick(self):
        ut = self.sc.ut
        if self.rpc is not None:
            self.rpc.tick(self.state, ut)
        started = time.monotonic()
        if self.gui.terminate_pressed():
            self.finish("terminated")
            return ut
        if self.state == STANDBY:
            if self.autostart or self.gui.start_pressed():
                self.take_over(ut)
                self.gui.hide_start()
                self.enter(FLY, ut)
            else:
                self.lines = ["phase   : STANDBY", "press START"]
        elif self.state == FLY:
            self.run_fly(ut)
            self.run_prop(ut, bool(self.gui.value("prop")))
        self.loop_rate.sample(self.state, ut, time.monotonic() - started)
        try:
            self.gui.update(self.lines + self.prop_lines + (
                ["prop    : %s" % self.prop_error] if self.prop_error else []))
        except Exception:                               # noqa: BLE001
            pass
        return ut

    def shutdown(self, reason):
        ut = 0.0
        try:
            ut = self.sc.ut
        except Exception:                               # noqa: BLE001
            pass
        try:
            self.stop_prop(reason)
        except Exception:                               # noqa: BLE001
            pass
        if self.state == FLY:
            self.hand_back(ut)
        self.log.event(ut, "loop rate, game-s per tick / wall-s of work: %s"
                       % self.loop_rate.report())
        if self.rpc is not None:
            self.log.event(ut, self.rpc.report())
        self.log.event(ut, "shutdown: %s" % reason)
        try:
            self.gui.close()
        except Exception:                               # noqa: BLE001
            pass


def run(conn, cfg, log, autostart=False):
    pilot = AirplanePilot(conn, cfg, log)
    pilot.rpc = RpcCounter()
    pilot.rpc.install(conn)
    pilot.autostart = autostart
    if autostart:
        pilot.gui.hide_start()
    wait = sleeper(cfg, lambda: conn.space_center.ut, conn)
    reason = "complete"
    try:
        while not pilot.finished:
            ut = pilot.tick()
            wait(cfg.LOOP_SLEEP_S, ut)
        reason = pilot.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
        raise
    except Exception as exc:                            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
    finally:
        pilot.shutdown(reason)
    return pilot, reason


def main(argv=None):
    args = cli.parser("airplane", "Airplane attitude autopilot: cruise "
                      "and hold.").parse_args(
        sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    status = 0
    try:
        conn = cli.connect(args)
        _, reason = run(conn, cfg, log, args.autostart)
        if reason.startswith("error"):
            status = 1
    except KeyboardInterrupt:
        pass
    except Exception as exc:                            # noqa: BLE001
        log.event(0.0, "FATAL %r" % exc)
        status = 1
    finally:
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
