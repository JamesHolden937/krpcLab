"""Prop pitch for top speed: the collective blade pitch that makes the most
forward thrust, computed every tick from the game's own blade aerodynamics.

Phase machine::

    STANDBY -> TRACK -> DONE

The pilot flies nothing but the propellers.  At START it finds every
robotic rotor and the control-surface blades mounted on it (any part with a
``Deploy Angle`` field below a rotor), disables the craft's KAL controllers
(which otherwise write the blades' angle every frame), runs every rotor at
``ROTOR_RPM`` and ``ROTOR_TORQUE_PCT`` with its motor engaged, reads each
blade's position and axes once, and from then on owns the blades'
``Deploy Angle``.

**How** (``bladeModel.PropModel``): every tick it reads the vessel's
velocity through the air (in its own axes), the air's density and speed of
sound, and evaluates KSP's lift and drag law for every blade at every
Deploy Angle; it commands the angle with the most thrust along the nose,
each rotor scored at the speed its motor can hold.  Nothing is searched
in flight, so a change of airspeed or attitude is answered on the next
tick.  The measured forward aero force per unit mass (``J``, which also
counts the airframe's drag) is logged against the model's thrust as the
check.

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
from .bladeModel import Blade, PropModel, Rotor, blade_type, choose_signs

STANDBY = "STANDBY"
TRACK = "TRACK"
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


def qrot(q, p):
    """Vector ``p`` turned by kRPC quaternion ``q`` (x, y, z, w)."""
    x, y, z, w = q
    tx = 2 * (y * p[2] - z * p[1])
    ty = 2 * (z * p[0] - x * p[2])
    tz = 2 * (x * p[1] - y * p[0])
    return (p[0] + w * tx + (y * tz - z * ty),
            p[1] + w * ty + (z * tx - x * tz),
            p[2] + w * tz + (x * ty - y * tx))


class BladePart:
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
        self.model = None
        self.opt = None
        self.start_ut = None
        self.next_reassert = 0.0
        self.last_cmd = None
        self.last_ut = None
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
        # Velocity through the air (the body's rotating frame) in the
        # vessel's own axes.
        self.air_flight = self.vessel.flight(
            self.sc.ReferenceFrame.create_hybrid(
                position=self.vessel.orbit.body.reference_frame,
                rotation=self.frame))
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
                    self.blades.append(BladePart(p, m, cs))
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
        self.build_model(ut)
        self.start_ut = ut

    def build_model(self, ut):
        """Every blade's geometry relative to its rotor's axis, read once
        (the blades turn with the rotor, but each rotor's evenly spaced
        blades are an average around the disc at any moment)."""
        vf = self.frame
        calls = []
        for r in self.rotors:
            calls += [(r.part.position, (vf,)), (r.part.direction, (vf,)),
                      (getattr, (r, "max_torque"))]
        owner = []
        for b in self.blades:
            calls += [(b.part.position, (vf,)), (b.part.rotation, (vf,)),
                      (getattr, (b.part, "name")), (getattr, (b.part, "parent"))]
        got = ask(self.conn, calls)
        rotor_parts = [r.part for r in self.rotors]
        rotors = []
        for i, r in enumerate(self.rotors):
            pos, axis, tmax = got[3 * i:3 * i + 3]
            rotors.append((pos, Rotor(axis, [], None if tmax is None else
                                      tmax * self.cfg.ROTOR_TORQUE_PCT / 100.0)))
        k = 3 * len(self.rotors)
        unknown = set()
        for j, b in enumerate(self.blades):
            pos, rot, name, parent = got[k + 4 * j:k + 4 * j + 4]
            kind = blade_type(name or "")
            if kind is None:
                unknown.add(name)
                continue
            # The rotor this blade hangs from: its nearest ancestor rotor.
            ri = None
            p = parent
            while p is not None and ri is None:
                if p in rotor_parts:
                    ri = rotor_parts.index(p)
                else:
                    p = p.parent
            if ri is None:
                ri = 0
            hub = rotors[ri][0]
            rel = tuple(x - h for x, h in zip(pos, hub))
            rotors[ri][1].blades.append(Blade(kind, rel, qrot(rot, (1, 0, 0)),
                                              qrot(rot, (0, 0, 1)),
                                              up=qrot(rot, (0, 1, 0))))
        if unknown:
            raise RuntimeError("no aero data for blade part(s) %s "
                               "(propPitch/bladeTable.py)" % sorted(unknown))
        self.model_rotors = [i for i, (_, r) in enumerate(rotors) if r.blades]
        rotors = [r for _, r in rotors if r.blades]
        signs = choose_signs(rotors, self.cfg.ROTOR_RPM)
        self.model = PropModel(rotors)
        for r in rotors:
            self.log.event(ut, "rotor axis %s max torque %s kN m, %d blades "
                           "at radius %s m (velocity point)"
                           % (",".join("%+.2f" % x for x in r.axis),
                              "%.1f" % r.max_torque if r.max_torque else "?",
                              len(r.blades),
                              ",".join("%.2f" % math.hypot(*[
                                  c - r.axis[i] * sum(
                                      bp * a for bp, a in zip(bl.point, r.axis))
                                  for i, c in enumerate(bl.point)])
                                       for bl in r.blades)))
        self.log.event(ut, "deflection senses %s (thrust at rest, +/-: %s)"
                       % ("".join("+" if s > 0 else "-" for s, _, _ in signs),
                          " ".join("%.0f/%.0f" % (a, b) for _, a, b in signs)))

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
        """One batch: forward aero force, mass, the airspeed vector in the
        vessel's axes, atmosphere, height, rpm."""
        f = self.surface_flight
        calls = [(getattr, (self.flight, "aerodynamic_force")),
                 (getattr, (self.vessel, "mass")),
                 (getattr, (self.air_flight, "velocity")),
                 (getattr, (f, "atmosphere_density")),
                 (getattr, (f, "speed_of_sound")),
                 (getattr, (f, "speed")),
                 (getattr, (f, "mean_altitude")),
                 (getattr, (f, "vertical_speed")),
                 (self.vessel.resources.amount, ("ElectricCharge",))]
        calls += [(getattr, (r, "current_rpm")) for r in self.rotors]
        return ask(self.conn, calls)

    def run_track(self, ut):
        got = self.read()
        force, mass, v_air, rho, sound, speed, alt, vs, ec = got[:9]
        rpm_list = list(got[9:])
        rpms = [x for x in rpm_list if x is not None]
        j = force[1] / mass if force is not None and mass else None
        if ut >= self.next_reassert:
            self.reassert(ut)
            self.next_reassert = ut + self.cfg.REASSERT_S
        cfg = self.cfg
        targets = [cfg.ROTOR_RPM] * len(self.model.rotors)
        opt = None
        dt = 0.0 if self.last_ut is None else ut - self.last_ut
        self.last_ut = ut
        if (v_air is not None and rho and sound and self.last_cmd is not None
                and None not in rpm_list):
            # The rotors' rpm now answers the angle flown since last tick.
            self.model.observe([rpm_list[i] for i in self.model_rotors],
                               targets, self.last_cmd, v_air, rho,
                               sound, dt)
        if v_air is not None and rho and sound:
            opt = self.model.optimum(targets, v_air, rho, sound,
                                     cfg.PITCH_MIN_DEG, cfg.PITCH_MAX_DEG,
                                     cfg.SEARCH_STEP_DEG, cfg.SEARCH_TOL_DEG)
        cmd = self.command(ut, opt, v_air, rho, sound)
        if cmd is not None and (self.last_cmd is None
                                or abs(cmd - self.last_cmd) >= 0.01):
            for b in self.blades:
                b.set(cmd)
            self.last_cmd = cmd
        self.opt = opt
        flown = opt
        if (opt is not None and cmd is not None
                and abs(cmd - opt.deploy) > 1e-6):
            flown = self.model.evaluate(cmd, targets, v_air, rho, sound)
        rpm = sum(rpms) / len(rpms) if rpms else float("nan")
        # The model's thrust at the angle flown, per unit mass: J less this
        # is the airframe's drag, whatever the angle.
        model_acc = (1000.0 * flown.thrust / mass
                     if flown is not None and mass else float("nan"))
        qfrac = (max(q / r.cap for q, r in zip(flown.torques,
                                                       self.model.rotors)
                     if r.cap) if flown is not None and any(
                         r.cap for r in self.model.rotors)
                 else float("nan"))
        text = ("TRACK spd=%6.1f alt=%6.0f vs=%+5.1f pitch=%5.1f opt=%5.1f "
                "J=%+7.3f model=%+7.3f rpm=%5.0f/%5.0f torque=%4.2f motor=%s "
                "va=%+6.1f,%+6.1f,%+6.1f ec=%4.0f"
                % (speed or 0, alt or 0, vs or 0,
                   cmd if cmd is not None else float("nan"),
                   opt.deploy if opt is not None else float("nan"),
                   j if j is not None else float("nan"), model_acc, rpm,
                   min(flown.rpms) if flown is not None else float("nan"), qfrac,
                   ",".join("%.0f" % (r.max_torque or 0)
                            for r in self.model.rotors),
                   *(v_air or (float("nan"),) * 3), ec or 0))
        self.log.telemetry(ut, text)
        self.lines = ["phase   : TRACK",
                      "speed   : %.1f m/s" % (speed or 0),
                      "pitch   : %.1f deg" % (cmd or 0),
                      "rpm     : %.0f" % rpm,
                      "accel   : %+.3f m/s^2 (model thrust %+.3f)"
                      % (j or 0, model_acc)]
        if cfg.RUN_TIME_S > 0 and ut - self.start_ut >= cfg.RUN_TIME_S:
            self.finish("run time")

    def command(self, ut, opt, v_air, rho, sound):
        """The Deploy Angle to fly: the optimum (a probe overrides this)."""
        return opt.deploy if opt is not None else None

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
                self.enter(TRACK, ut)
            else:
                self.lines = ["phase   : STANDBY", "press START"]
        elif self.state == TRACK:
            self.run_track(ut)
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
        if self.state == TRACK:
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


def run(conn, cfg, log, autostart=False, pilot_class=None):
    pilot = (pilot_class or PropPilot)(conn, cfg, log)
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


def main(argv=None, pilot_class=None):
    args = cli.parser("prop", "Prop pitch for top speed.").parse_args(
        sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    status = 0
    try:
        conn = cli.connect(args)
        _, reason = run(conn, cfg, log, args.autostart, pilot_class)
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
