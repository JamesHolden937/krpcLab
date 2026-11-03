"""Prop pitch for top speed: the collective blade pitch that makes the most
forward force, found and re-found continuously in flight.

Phase machine::

    STANDBY -> SEEK -> DONE

The pilot flies nothing but the propellers.  At START it finds every
robotic rotor and the control-surface blades mounted on it (any part with a
``Deploy Angle`` field below a rotor), disables the craft's KAL controllers
(which otherwise write the blades' angle every frame), runs every rotor at
``ROTOR_RPM`` and ``ROTOR_TORQUE_PCT`` with its motor engaged, and from then
on owns the blades' ``Deploy Angle``.

**What it maximises**: the aerodynamic force along the vessel's forward
axis per unit mass (``Flight.aerodynamic_force`` in the vessel frame, which
sums every part's lift and drag, the blades' included): the propellers'
thrust less the airframe's drag, i.e. the acceleration the air gives the
plane.  At any airspeed the pitch that maximises it is the pitch that
accelerates fastest, so following it to the top of the curve is following
it to the highest speed the plane can hold.

**How** (``seeker.PitchSeeker``): the pitch is dithered by a sine, the
force's response fitted over the last periods, and the dither's centre
moved up the measured slope.  It needs no model of the blades, the rotor or
the craft, and it follows the optimum as the airspeed (and with it the
blades' inflow angle) changes.

TERMINATE (or ``RUN_TIME_S``) hands everything back: the KALs re-enabled,
the rotors' RPM and torque limits as they were.
"""

import math
import sys
import time

from common import cli, partmodule
from common.krpcbatch import ask
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from common.panel import ControlPanel
from common.rpccount import RpcCounter
from .config import Config, apply_overrides, defaults_fingerprint, differences
from .seeker import PitchSeeker

STANDBY = "STANDBY"
SEEK = "SEEK"
DONE = "DONE"

ACCENT_PROP = (0.5, 1.0, 0.5)
DEPLOY_FIELD = "Deploy Angle"
KAL_MODULE = "ModuleRoboticController"


def descendants(part):
    out = []
    stack = list(part.children)
    while stack:
        p = stack.pop()
        out.append(p)
        stack.extend(p.children)
    return out


def deploy_module(part):
    """The module of ``part`` with a ``Deploy Angle`` field, or None."""
    for m in part.modules:
        # Only control-surface modules have it (stock ModuleControlSurface,
        # or a mod's replacement); reading every module's field list cost
        # 2530 calls and ~3.5 s at START (LOG15158).
        if "ControlSurface" not in m.name:
            continue
        try:
            if partmodule.has_field(m, DEPLOY_FIELD):
                return m
        except Exception:                               # noqa: BLE001
            continue
    return None


class Blade:
    def __init__(self, part, module, surface):
        self.part = part
        self.module = module
        self.surface = surface
        self.field = None
        if hasattr(module, "field_list"):
            self.field = partmodule._table(module, "field")[DEPLOY_FIELD]

    def angle(self):
        return float(partmodule.get_field(self.module, DEPLOY_FIELD))

    def set(self, value):
        if self.field is not None:
            self.field.float_value = float(value)
        else:
            partmodule.set_field_float(self.module, DEPLOY_FIELD, value)


class PropPilot:
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
        self.rotors = []
        self.blades = []
        self.kals = []
        self.saved_rotors = []
        self.seeker = None
        self.aoa_field = None
        self.start_ut = None
        self.next_reassert = 0.0
        self.last_cmd = None
        self.lines = []
        ut = self.sc.ut
        log.event(ut, "vessel=%s parts=%d" % (self.vessel.name,
                                               len(self.vessel.parts.all)))
        log.event(ut, "config: %s defaults_fingerprint=%s"
                  % (" ".join(differences(cfg)) or "(defaults)",
                     defaults_fingerprint()))
        self.mu = self.vessel.orbit.body.gravitational_parameter
        self.frame = self.vessel.reference_frame
        self.flight = self.vessel.flight(self.frame)
        self.surface_flight = self.vessel.flight(
            self.vessel.orbit.body.reference_frame)
        self.gui = ControlPanel(conn, "PROP PITCH", "START", ACCENT_PROP)

    # -- takeover -----------------------------------------------------------
    def find(self, ut):
        self.rotors = list(self.vessel.parts.robotic_rotors)
        seen = set()
        for rotor in self.rotors:
            for p in descendants(rotor.part):
                if p in seen:
                    continue
                seen.add(p)
                cs = p.control_surface
                if cs is None:
                    continue
                m = deploy_module(p)
                if m is not None:
                    self.blades.append(Blade(p, m, cs))
        for p in self.vessel.parts.with_module(KAL_MODULE):
            for m in p.modules:
                if m.name == KAL_MODULE:
                    self.kals.append(m)
        self.log.event(ut, "found %d rotors, %d blades, %d KAL controllers"
                       % (len(self.rotors), len(self.blades), len(self.kals)))
        if not self.rotors or not self.blades:
            raise RuntimeError("no rotor with blades on this vessel")

    def take_over(self, ut):
        self.find(ut)
        if self.cfg.DISABLE_KAL:
            for m in self.kals:
                try:
                    partmodule.set_field_bool_by_id(m, "controllerEnabled",
                                                    False)
                except Exception as exc:                # noqa: BLE001
                    self.log.event(ut, "KAL not disabled: %r" % exc)
        for r in self.rotors:
            self.saved_rotors.append((r, r.target_rpm, r.torque_limit,
                                      r.motor_engaged))
            r.motor_engaged = True
            r.locked = False
            r.torque_limit = self.cfg.ROTOR_TORQUE_PCT
            r.target_rpm = self.cfg.ROTOR_RPM
        for b in self.blades:
            try:
                b.surface.deployed = True
            except Exception:                           # noqa: BLE001
                pass
        angles = [b.angle() for b in self.blades]
        start = (self.cfg.INITIAL_PITCH_DEG if self.cfg.INITIAL_PITCH_DEG >= 0
                 else sum(angles) / len(angles))
        self.log.event(ut, "blade angles at start %s; rotor rpm %s; "
                       "seeking from %.1f deg"
                       % (",".join("%.1f" % a for a in sorted(set(angles))),
                          ",".join("%.0f/%.0f" % (s[1], s[2])
                                   for s in self.saved_rotors), start))
        # The blade's own "Angle of Attack" (the part menu's): logged, so
        # the optimum the seeker finds can be read as a blade incidence.
        self.aoa_field = None
        b = self.blades[0]
        if hasattr(b.module, "field_list"):
            self.aoa_field = partmodule._table(b.module, "field").get(
                "Angle of Attack")
        self.seeker = PitchSeeker(self.cfg, start)
        self.start_ut = ut

    def reassert(self, ut):
        """Read back the KALs' Enabled flag and the rotors' limits; re-set
        whatever something else changed, and say so."""
        changed = []
        if self.cfg.DISABLE_KAL:
            for m in self.kals:
                try:
                    if partmodule.get_field(m, "Enabled") == "True":
                        partmodule.set_field_bool_by_id(
                            m, "controllerEnabled", False)
                        changed.append("KAL")
                except Exception:                       # noqa: BLE001
                    pass
        for r in self.rotors:
            try:
                if (abs(r.target_rpm - self.cfg.ROTOR_RPM) > 0.5
                        or abs(r.torque_limit - self.cfg.ROTOR_TORQUE_PCT) > 0.5
                        or not r.motor_engaged or r.locked):
                    changed.append("rotor %.0f/%.0f%s%s" % (
                        r.target_rpm, r.torque_limit,
                        "" if r.motor_engaged else " off",
                        " locked" if r.locked else ""))
                    r.motor_engaged = True
                    r.locked = False
                    r.torque_limit = self.cfg.ROTOR_TORQUE_PCT
                    r.target_rpm = self.cfg.ROTOR_RPM
            except Exception:                           # noqa: BLE001
                pass
        if changed:
            self.log.event(ut, "re-set: %s" % ", ".join(changed))

    def hand_back(self, ut):
        for r, rpm, torque, engaged in self.saved_rotors:
            try:
                r.target_rpm = rpm
                r.torque_limit = torque
                r.motor_engaged = engaged
            except Exception:                           # noqa: BLE001
                pass
        if self.cfg.DISABLE_KAL:
            for m in self.kals:
                try:
                    partmodule.set_field_bool_by_id(m, "controllerEnabled",
                                                    True)
                except Exception as exc:                # noqa: BLE001
                    self.log.event(ut, "KAL not re-enabled: %r" % exc)

    # -- loop ---------------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s%s" % (self.state, state,
                                                 (": " + note) if note else ""))
        self.state = state

    def finish(self, reason):
        self.exit_reason = reason
        self.finished = True

    def read(self):
        """One batch: forward aero force, mass, airspeed, height, rpm."""
        calls = [(getattr, (self.flight, "aerodynamic_force")),
                 (getattr, (self.vessel, "mass")),
                 (getattr, (self.surface_flight, "speed")),
                 (getattr, (self.surface_flight, "mean_altitude")),
                 (getattr, (self.surface_flight, "vertical_speed"))]
        bf = self.vessel.orbit.body.reference_frame
        calls += [(getattr, (self.surface_flight, "acceleration")),
                  (self.vessel.direction, (bf,)),
                  (self.vessel.position, (bf,)),
                  (self.vessel.resources.amount, ("ElectricCharge",))]
        calls.append((getattr, (self.aoa_field, "value"))
                     if self.aoa_field is not None else
                     (getattr, (self.surface_flight, "speed")))
        calls += [(getattr, (r, "current_rpm")) for r in self.rotors]
        return ask(self.conn, calls)

    def run_seek(self, ut):
        got = self.read()
        force, mass, speed, alt, vs, acc, fwd, pos, ec = got[:9]
        aoa = None
        if self.aoa_field is not None:
            try:
                aoa = float(got[9])
            except (TypeError, ValueError):
                pass
        rpms = [x for x in got[10:] if x is not None]
        # The measured specific force along the nose: the acceleration
        # less gravity (kRPC's includes it), to check J against.
        meas = None
        if acc is not None and fwd is not None and pos is not None:
            r = math.sqrt(sum(x * x for x in pos))
            g = self.mu / (r * r)
            meas = sum(a * d for a, d in zip(acc, fwd)) + g * sum(
                p * d for p, d in zip(pos, fwd)) / r
        j = None
        if force is not None and mass:
            j = force[1] / mass
        if ut >= self.next_reassert:
            self.reassert(ut)
            self.next_reassert = ut + self.cfg.REASSERT_S
        cmd = self.seeker.step(ut, j)
        if self.last_cmd is None or abs(cmd - self.last_cmd) >= 0.01:
            for b in self.blades:
                b.set(cmd)
            self.last_cmd = cmd
        fit = self.seeker.fit
        rpm = sum(rpms) / len(rpms) if rpms else float("nan")
        text = ("SEEK spd=%6.1f alt=%6.0f vs=%+5.1f pitch=%5.1f ctr=%5.1f "
                "J=%+7.3f acc=%+7.3f rpm=%5.0f aoa=%5.1f ec=%4.0f"
                % (speed or 0, alt or 0, vs or 0, cmd, self.seeker.centre,
                   j if j is not None else float("nan"),
                   meas if meas is not None else float("nan"), rpm,
                   aoa if aoa is not None else float("nan"), ec or 0))
        if fit is not None:
            text += (" slope=%+7.4f t=%+6.1f curv=%+8.5f/%+5.1f lag=%+5.0f "
                     "trend=%+6.3f rate=%+5.2f%s"
                     % (fit.slope, fit.slope_t, fit.curvature,
                        fit.curvature_t, fit.lag_deg, fit.trend,
                        self.seeker.rate,
                        " newton" if self.seeker.newton else ""))
        self.log.telemetry(ut, text)
        self.lines = ["phase   : SEEK",
                      "speed   : %.1f m/s" % (speed or 0),
                      "pitch   : %.1f deg (centre %.1f)"
                      % (cmd, self.seeker.centre),
                      "rpm     : %.0f" % rpm,
                      "accel   : %+.3f m/s^2" % (j or 0)]
        if fit is not None:
            self.lines.append("slope   : %+.4f per deg" % fit.slope)
        if self.cfg.RUN_TIME_S > 0 and ut - self.start_ut >= self.cfg.RUN_TIME_S:
            self.finish("run time")

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
                self.enter(SEEK, ut)
            else:
                self.lines = ["phase   : STANDBY", "press START"]
        elif self.state == SEEK:
            self.run_seek(ut)
        self.loop_rate.sample(self.state, ut, time.monotonic() - started)
        try:
            self.gui.update(self.lines)
        except Exception:                               # noqa: BLE001
            pass
        return ut

    def shutdown(self, reason):
        ut = 0.0
        try:
            ut = self.sc.ut
        except Exception:                               # noqa: BLE001
            pass
        if self.state == SEEK:
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
    pilot = PropPilot(conn, cfg, log)
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
    args = cli.parser("prop", "Prop pitch for top speed.").parse_args(
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
