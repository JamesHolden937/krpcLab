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
from . import tipping
from common.krpcbatch import ask
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from common.panel import ControlPanel
from common.rpccount import RpcCounter
from .config import Config, apply_overrides, defaults_fingerprint, differences
from .bladeModel import (MOST_FORWARD, MOST_REVERSE, Blade, PropModel, Rotor,
                         blade_type, choose_signs)

STANDBY = "STANDBY"
TRACK = "TRACK"
DONE = "DONE"

ACCENT_PROP = (0.5, 1.0, 0.5)
DEPLOY_FIELD = "Deploy Angle"
KAL_MODULE = "ModuleRoboticController"
ROTOR_MODULE = "ModuleRoboticServoRotor"
# The rotor's brake: a latched mode (``brakingMode``) the Brakes action
# group sets and clears, and the rotor also latches on losing power.  No
# kRPC property reads or clears it; only this action does (Deactivate
# releases it, Assembly-CSharp ``ModuleRoboticServoRotor.BrakeAction``).
BRAKE_ACTION = "BrakeAction"
# The log's mode= field while braking: which way the props push (None:
# the plane is still and the props are stopped).
BRAKE_TAG = {MOST_REVERSE: "brake-rev", MOST_FORWARD: "brake-fwd",
             None: "brake-stop"}


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


def brake_action(part):
    """The rotor part's ``BrakeAction`` (a kRPC PartAction), or None."""
    for m in part.modules:
        if m.name != ROTOR_MODULE or not hasattr(m, "action_list"):
            continue
        for a in m.action_list:
            if a.name == BRAKE_ACTION:
                return a
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
        self.brakes = []
        self.model = None
        self.opt = None
        self.start_ut = None
        self.next_reassert = 0.0
        self.last_cmd = None
        self.last_ut = None
        self.rpm_cmd = cfg.ROTOR_RPM
        self.braking = False
        # The prop brake has stopped the props (the plane is still).
        self.held = False
        # Tipping: the wheels, their bounding boxes (vessel frame, read at
        # START), the filtered acceleration and the hill climb's fraction.
        self.wheels = []
        self.wheel_boxes = []
        self.accel = (0.0, 0.0, 0.0)
        self.last_v = None
        self.last_grounded = None
        self.last_pitch = None
        self.tip_k = 1.0
        self.tip_climb = 0.0
        self.tip_base = None
        self.tip_computed = False
        self.tip_contacts = set()
        self.tip_backoff_ut = -1e9
        self.tip_note = "-"
        self.last_flown = None
        # Reaction torque: each model rotor's (rotor part, one blade part),
        # its measured spin sense, and the trims (the pilot's own, ours).
        self.spin_parts = []
        self.spin_sense = []
        self.spin_seen = []
        self.spin_votes = []
        self.hubs = []
        self.user_trim = None
        self.trim_written = None
        self.trim_prev = None
        self.counter = (0.0, 0.0, 0.0)
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

    def throttle_rpm(self, throttle):
        """The RPM limit the throttle asks for (ROTOR_RPM when the throttle
        does not set it, or cannot be read)."""
        cfg = self.cfg
        if not cfg.THROTTLE_RPM or throttle is None:
            return cfg.ROTOR_RPM
        t = max(0.0, min(1.0, float(throttle)))
        if t <= cfg.THROTTLE_FEATHER:
            return 0.0
        return max(cfg.THROTTLE_RPM_MIN, t * cfg.ROTOR_RPM)

    def feathered(self, braking):
        """Throttle cut: the rotors held stopped, the blades feathered."""
        return (self.cfg.THROTTLE_RPM and not braking
                and self.rpm_cmd <= 0.0)

    def set_rpm(self, ut, rpm):
        """Write the rotors' RPM limit when it moved by RPM_WRITE_STEP (or
        reached an end of the range)."""
        if (abs(rpm - self.rpm_cmd) < self.cfg.RPM_WRITE_STEP
                and not (rpm != self.rpm_cmd and (rpm <= self.cfg.THROTTLE_RPM_MIN
                                                   or rpm >= self.cfg.ROTOR_RPM))):
            return
        self.rpm_cmd = rpm
        for r in self.rotors:
            try:
                r.target_rpm = rpm
            except Exception:                           # noqa: BLE001
                pass

    def take_over(self, ut):
        self.find(ut)
        try:
            self.rpm_cmd = self.throttle_rpm(self.vessel.control.throttle)
        except Exception:                               # noqa: BLE001
            self.rpm_cmd = self.cfg.ROTOR_RPM
        self.log.event(ut, "rotor rpm %s: %.0f" % (
            "from the throttle" if self.cfg.THROTTLE_RPM else "fixed",
            self.rpm_cmd))
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
            r.target_rpm = self.rpm_cmd
            try:
                self.brakes.append(brake_action(r.part))
            except Exception:                           # noqa: BLE001
                self.brakes.append(None)
        # Started in flight, the rotors may be braked (Brakes pressed on the
        # ground and never released, or a power loss): a braked rotor holds
        # 0 rpm whatever its motor is told (LOG15188).  The vessel's wheel
        # brakes are left as they are.
        released = self.release_brakes(range(len(self.rotors)))
        self.log.event(ut, "rotor brakes released: %d of %d (Brakes group %s)"
                       % (released, len(self.rotors),
                          "on" if self.vessel.control.brakes else "off"))
        for b in self.blades:
            try:
                b.surface.deployed = True
            except Exception:                           # noqa: BLE001
                pass
        self.build_model(ut)
        self.find_wheels(ut)
        if self.cfg.TORQUE_COUNTER:
            c = self.vessel.control
            self.user_trim = (c.pitch_trim, c.roll_trim, c.yaw_trim)
        self.start_ut = ut

    def find_wheels(self, ut):
        """Every wheel and its bounding box in the vessel frame (fixed to
        the airframe), for the tipping limit."""
        body = self.vessel.orbit.body
        self.g = body.surface_gravity
        self.srf_frame = self.vessel.surface_reference_frame
        try:
            self.wheels = list(self.vessel.parts.wheels)
        except Exception:                               # noqa: BLE001
            self.wheels = []
        boxes = ask(self.conn, [(w.part.bounding_box, (self.frame,))
                                for w in self.wheels])
        keep = [(w, b) for w, b in zip(self.wheels, boxes) if b is not None]
        self.wheels = [w for w, _ in keep]
        self.wheel_boxes = [b for _, b in keep]
        self.log.event(ut, "wheels %d: %s" % (len(self.wheels), " ".join(
            "%+.1f,%+.1f,%+.1f" % tuple((a + b) / 2 for a, b in zip(*bx))
            for bx in self.wheel_boxes)))

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
        first_blade = {}
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
            first_blade.setdefault(ri, b.part)
            rel = tuple(x - h for x, h in zip(pos, hub))
            rotors[ri][1].blades.append(Blade(kind, rel, qrot(rot, (1, 0, 0)),
                                              qrot(rot, (0, 0, 1)),
                                              up=qrot(rot, (0, 1, 0))))
        if unknown:
            raise RuntimeError("no aero data for blade part(s) %s "
                               "(propPitch/bladeTable.py)" % sorted(unknown))
        self.model_rotors = [i for i, (_, r) in enumerate(rotors) if r.blades]
        self.hubs = [rotors[i][0] for i in self.model_rotors]
        self.spin_parts = [(self.rotors[i].part, first_blade[i])
                           for i in self.model_rotors]
        self.spin_sense = [None] * len(self.model_rotors)
        self.spin_seen = [None] * len(self.model_rotors)
        self.spin_votes = [0] * len(self.model_rotors)
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

    def release_brakes(self, which):
        """Release the brakes of rotors ``which``; how many were released."""
        n = 0
        for i in which:
            a = self.brakes[i] if i < len(self.brakes) else None
            if a is None:
                continue
            try:
                a.activated = False
                n += 1
            except Exception:                           # noqa: BLE001
                pass
        return n

    def engage_brakes(self):
        """Set every rotor's brake latch (the props held stopped)."""
        for a in self.brakes:
            if a is None:
                continue
            try:
                a.activated = True
            except Exception:                           # noqa: BLE001
                pass

    def reassert(self, ut, rpm_list=None, brakes=False):
        """Read back the KALs' Enabled flag and the rotors' limits; re-set
        whatever something else changed, and say so.  A rotor stopped while
        the Brakes group is off is a latched brake (a power loss latches
        one): released.  While the props brake, every rotor's brake is
        released."""
        changed = []
        if self.braking:
            if not self.held:
                self.release_brakes(range(len(self.rotors)))
        elif rpm_list and len(rpm_list) == len(self.rotors):
            stopped = [i for i, w in enumerate(rpm_list)
                       if w is not None and abs(w) < self.cfg.BRAKE_RPM]
            # A throttle cut stops the rotors on purpose: not a brake.
            if (stopped and self.rpm_cmd > self.cfg.BRAKE_RPM
                    and not brakes):
                if self.release_brakes(stopped):
                    changed.append("brake on %d stopped rotor(s)"
                                   % len(stopped))
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
                if (abs(r.target_rpm - self.rpm_cmd) > 0.5
                        or abs(r.torque_limit - self.cfg.ROTOR_TORQUE_PCT) > 0.5
                        or not r.motor_engaged or r.locked):
                    changed.append("rotor %.0f/%.0f%s%s" % (
                        r.target_rpm, r.torque_limit,
                        "" if r.motor_engaged else " off",
                        " locked" if r.locked else ""))
                    r.motor_engaged = True
                    r.locked = False
                    r.torque_limit = self.cfg.ROTOR_TORQUE_PCT
                    r.target_rpm = self.rpm_cmd
            except Exception:                           # noqa: BLE001
                pass
        if changed:
            self.log.event(ut, "re-set: %s" % ", ".join(changed))

    def hand_back(self, ut):
        if self.user_trim is not None and self.trim_written is not None:
            try:
                c = self.vessel.control
                c.pitch_trim, c.roll_trim, c.yaw_trim = self.user_trim
            except Exception:                           # noqa: BLE001
                pass
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
                 (self.vessel.resources.amount, ("ElectricCharge",)),
                 (getattr, (self.vessel.control, "throttle")),
                 (getattr, (self.vessel.control, "brakes"))]
        calls += [(getattr, (r, "current_rpm")) for r in self.rotors]
        cfg = self.cfg
        if cfg.PROP_BRAKE and cfg.BRAKE_TIP_LIMIT:
            calls += [(self.sc.transform_direction,
                       ((1.0, 0.0, 0.0), self.srf_frame, self.frame)),
                      (getattr, (f, "pitch"))]
            calls += [(getattr, (w, "grounded")) for w in self.wheels]
        if cfg.TORQUE_COUNTER:
            c = self.vessel.control
            calls += [(getattr, (self.vessel, "available_torque")),
                      (getattr, (c, "pitch_trim")), (getattr, (c, "roll_trim")),
                      (getattr, (c, "yaw_trim"))]
        return ask(self.conn, calls)

    def run_track(self, ut):
        got = self.read()
        force, mass, v_air, rho, sound, speed, alt, vs, ec = got[:9]
        throttle, brakes = got[9:11]
        n = len(self.rotors)
        rpm_list = list(got[11:11 + n])
        rest = list(got[11 + n:])
        cfg = self.cfg
        up = pitch = None
        grounded = []
        if cfg.PROP_BRAKE and cfg.BRAKE_TIP_LIMIT:
            up, pitch = rest[:2]
            grounded = rest[2:2 + len(self.wheels)]
            rest = rest[2 + len(self.wheels):]
        torque_avail = trims = None
        if cfg.TORQUE_COUNTER:
            torque_avail, trims = rest[0], tuple(rest[1:4])
        rpms = [x for x in rpm_list if x is not None]
        j = force[1] / mass if force is not None and mass else None
        dt = 0.0 if self.last_ut is None else ut - self.last_ut
        self.last_ut = ut
        self.track_accel(v_air, dt)
        braking = bool(cfg.PROP_BRAKE and brakes)
        pressed = braking and not self.braking
        if braking != self.braking:
            self.braking = braking
            self.held = False
            self.tip_k = 1.0
            self.tip_climb = 0.0
            self.tip_base = None
            self.tip_computed = False
            self.tip_contacts = set()
            self.log.event(ut, "prop brake %s" % ("on" if braking else "off"))
        objective = MOST_FORWARD
        if braking and v_air is not None:
            was = self.held
            # Against the motion: which way the air comes along the nose
            # (y in the vessel's axes) says which thrust stops it.
            objective = self.brake_objective(v_air[1])
            if self.held and (pressed or not was):
                # Still: the props stop and stay stopped.
                self.engage_brakes()
                self.log.event(ut, "prop brake: still at %+.2f m/s, props "
                               "stopped" % v_air[1])
            elif not self.held and (pressed or was):
                # The Brakes group has just latched the rotors' own brakes
                # (or the plane moves again): released, so the props spin.
                self.release_brakes(range(n))
                if was:
                    self.log.event(ut, "prop brake: moving at %+.2f m/s, "
                                   "props spun up" % v_air[1])
        elif pressed:
            self.release_brakes(range(n))
        if ut >= self.next_reassert:
            self.reassert(ut, rpm_list, bool(brakes))
            self.next_reassert = ut + self.cfg.REASSERT_S
        if braking:
            self.set_rpm(ut, 0.0 if self.held else cfg.ROTOR_RPM)
        else:
            self.set_rpm(ut, self.throttle_rpm(throttle))
        targets = [self.rpm_cmd] * len(self.model.rotors)
        if None not in rpm_list:
            self.probe_spin(ut, rpm_list)
        opt = None
        if (v_air is not None and rho and sound and self.last_cmd is not None
                and None not in rpm_list):
            # The rotors' rpm now answers the angle flown since last tick.
            self.model.observe([rpm_list[i] for i in self.model_rotors],
                               targets, self.last_cmd, v_air, rho,
                               sound, dt)
        feather = self.feathered(braking)
        mode = ("feather" if feather else "thrust") if not braking else "brake"
        self.tip_note = "-"
        if braking:
            mode = BRAKE_TAG[objective if v_air is not None else MOST_REVERSE]
        if v_air is not None and rho and sound and not (braking and self.held):
            lo, hi = cfg.PITCH_MIN_DEG, cfg.PITCH_MAX_DEG
            if feather or braking:
                # Feathered, the most thrust at 0 rpm is the least drag (each
                # blade edge-on to the flow, wherever it comes from); braking,
                # negative angles give reverse thrust.
                lo, hi = -self.model.deploy_limit, self.model.deploy_limit
            opt = self.model.optimum(targets, v_air, rho, sound, lo, hi,
                                     cfg.SEARCH_STEP_DEG, cfg.SEARCH_TOL_DEG,
                                     objective)
            # Once this brake has touched the ground, a bounce (every wheel
            # up for a moment) keeps the limit.
            if (braking and cfg.BRAKE_TIP_LIMIT and grounded
                    and (any(grounded) or self.tip_contacts)):
                opt = self.tip_limited(ut, opt, objective, targets, v_air, rho,
                                       sound, lo, hi, up, pitch, grounded,
                                       mass, force, dt)
            else:
                self.last_grounded = self.last_pitch = None
        if braking and self.held:
            cmd = self.last_cmd             # stopped: the blades stay put
        else:
            cmd = self.command(ut, opt, v_air, rho, sound)
        if cmd is not None and (self.last_cmd is None
                                or abs(cmd - self.last_cmd) >= 0.01):
            for b in self.blades:
                b.set(cmd)
            self.last_cmd = cmd
        self.opt = opt
        flown = opt
        if (cmd is not None and v_air is not None and rho and sound
                and (opt is None or abs(cmd - opt.deploy) > 1e-6)):
            flown = self.model.evaluate(cmd, targets, v_air, rho, sound)
        self.last_flown = flown
        self.counter_torque(flown if not (braking and self.held) else None,
                            torque_avail, trims)
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
                "va=%+6.1f,%+6.1f,%+6.1f ec=%4.0f thr=%4.2f mode=%s tip=%s "
                "trim=%+.2f,%+.2f,%+.2f"
                % (speed or 0, alt or 0, vs or 0,
                   cmd if cmd is not None else float("nan"),
                   opt.deploy if opt is not None else float("nan"),
                   j if j is not None else float("nan"), model_acc, rpm,
                   min(flown.rpms) if flown is not None else float("nan"), qfrac,
                   ",".join("%.0f" % (r.max_torque or 0)
                            for r in self.model.rotors),
                   *(v_air or (float("nan"),) * 3), ec or 0,
                   throttle if throttle is not None else float("nan"),
                   mode, self.tip_note, *self.counter))
        self.log.telemetry(ut, text)
        self.lines = ["phase   : TRACK",
                      "speed   : %.1f m/s" % (speed or 0),
                      "pitch   : %.1f deg" % (cmd or 0),
                      "rpm     : %.0f (limit %.0f%s)" % (
                          rpm, self.rpm_cmd,
                          ", throttle" if cfg.THROTTLE_RPM and not braking
                          else ""),
                      "accel   : %+.3f m/s^2 (model thrust %+.3f)"
                      % (j or 0, model_acc),
                      "mode    : %s" % mode]
        if self.tip_note != "-":
            self.lines.append("tip lim : %s" % self.tip_note)
        if cfg.RUN_TIME_S > 0 and ut - self.start_ut >= cfg.RUN_TIME_S:
            self.finish("run time")

    def track_accel(self, v_air, dt):
        """The vessel's acceleration (vessel axes), the airspeed's
        derivative low-passed over BRAKE_ACCEL_TAU_S."""
        if v_air is None:
            return
        if self.last_v is not None and dt > 0.0:
            a = min(1.0, dt / self.cfg.BRAKE_ACCEL_TAU_S)
            self.accel = tuple(f + a * ((v - lv) / dt - f) for f, v, lv
                               in zip(self.accel, v_air, self.last_v))
        self.last_v = v_air

    def tip_limited(self, ut, opt, objective, targets, v_air, rho, sound,
                    lo, hi, up, pitch, grounded, mass, force, dt):
        """The braking optimum held to the thrust that keeps the wheels on
        the ground (``tipping.thrust_bounds``), times the hill climb's
        fraction, which halves on a wheel lifting or a pitch rate."""
        cfg = self.cfg
        # The hill climb: a tipping symptom backs off, quiet raises it.
        lifted = (self.last_grounded is not None
                  and sum(grounded) < sum(self.last_grounded))
        rate = (abs(pitch - self.last_pitch) / dt
                if pitch is not None and self.last_pitch is not None and dt > 0
                else 0.0)
        self.last_grounded = list(grounded)
        self.last_pitch = pitch
        symptom = lifted or rate > cfg.BRAKE_TIP_RATE_DEG_S
        if symptom and ut - self.tip_backoff_ut >= cfg.BRAKE_TIP_BACKOFF_S:
            self.tip_k *= 0.5
            self.tip_climb *= 0.5
            self.tip_backoff_ut = ut
            self.log.event(ut, "tip: %s, braking x%.2f" % (
                "a wheel left the ground" if lifted
                else "pitch %.1f deg/s" % rate, self.tip_k))
        elif not symptom and dt > 0.0:
            self.tip_k = min(1.0, self.tip_k + cfg.BRAKE_TIP_RISE_PER_S * dt)
            self.tip_climb += cfg.BRAKE_TIP_CLIMB_G_PER_S * dt
        # Every wheel on the ground since Brakes went on.
        self.tip_contacts.update(i for i, gr in enumerate(grounded) if gr)
        reverse = objective == MOST_REVERSE
        if up is not None and mass and force is not None and self.wheel_boxes:
            contacts = [tipping.contact_point(bx[0], bx[1], up)
                        for i, bx in enumerate(self.wheel_boxes)
                        if i in self.tip_contacts]
            last = self.last_flown
            w = ([abs(t) for t in last.thrusts] if last is not None
                 and last.thrusts else [1.0] * len(self.hubs))
            if sum(w) <= 0.0:
                w = [1.0] * len(self.hubs)
            hub = tuple(sum(wi * h[i] for wi, h in zip(w, self.hubs)) / sum(w)
                        for i in range(3))
            t_now = 1000.0 * last.thrust if last is not None else 0.0
            # Twice: with the measured deceleration (the wheel brakes' share
            # in it, but filtered, so it lags a thrust that just swung), and
            # with the aero force's alone (no lag, no wheel friction); the
            # stricter of the two.
            aero_h = tipping._sub(force, tipping._scale(
                up, tipping._dot(force, up)))
            bounds = tipping.stricter(
                tipping.thrust_bounds(contacts, up, self.g, mass, self.accel,
                                      force, hub, t_now, cfg.BRAKE_TIP_MARGIN),
                tipping.thrust_bounds(contacts, up, self.g, mass,
                                      tipping._scale(aero_h, 1.0 / mass),
                                      force, hub, t_now, cfg.BRAKE_TIP_MARGIN))
            if bounds is not None:
                # No bound on this side: the props cannot tip it this way.
                b = bounds[0] if reverse else bounds[1]
                self.tip_base = (None if b is None else
                                 min(0.0, b / 1000.0) if reverse
                                 else max(0.0, b / 1000.0))
                self.tip_computed = True
        if self.tip_computed and self.tip_base is None:
            return opt
        if self.tip_base is not None:
            limit = self.tip_k * self.tip_base
            how = "x%.2f" % self.tip_k
        else:
            # Never computed: climb from nothing in m g per second.
            limit = (self.tip_climb * (mass or 0.0) * self.g / 1000.0
                     * (-1.0 if reverse else 1.0))
            how = "(climb %.2f g)" % self.tip_climb
        self.tip_note = "%+.1fkN %s" % (limit, how)
        if (opt.thrust >= limit) if reverse else (opt.thrust <= limit):
            return opt
        if reverse:
            return self.model.optimum(targets, v_air, rho, sound, lo, hi,
                                      cfg.SEARCH_STEP_DEG, cfg.SEARCH_TOL_DEG,
                                      objective, floor=limit)
        return self.model.optimum(targets, v_air, rho, sound, lo, hi,
                                  cfg.SEARCH_STEP_DEG, cfg.SEARCH_TOL_DEG,
                                  objective, ceiling=limit)

    def probe_spin(self, ut, rpm_list):
        """Each rotor's real spin sense about the model's axis: a blade's
        angle about the hub between two ticks, against what +rpm and -rpm
        would have turned it by in that time (the readings alias at 460
        rpm, so a tick only votes when the two predictions differ)."""
        cfg = self.cfg
        todo = [k for k, s in enumerate(self.spin_sense) if s is None
                and abs(rpm_list[self.model_rotors[k]]) > cfg.SPIN_PROBE_RPM]
        if not todo or not cfg.TORQUE_COUNTER:
            return
        got = ask(self.conn, [(self.spin_parts[k][1].position, (self.frame,))
                              for k in todo])
        for k, pos in zip(todo, got):
            if pos is None:
                continue
            r = self.model.rotors[k]
            ax = tuple(a * r.spin for a in r.axis)
            rel = tuple(p - h for p, h in zip(pos, self.hubs[k]))
            e1 = tipping._unit(tipping._cross(ax, (1.0, 0.0, 0.0))
                               if abs(ax[0]) < 0.9 else
                               tipping._cross(ax, (0.0, 1.0, 0.0)))
            e2 = tipping._cross(ax, e1)
            theta = math.atan2(tipping._dot(rel, e2), tipping._dot(rel, e1))
            seen = self.spin_seen[k]
            self.spin_seen[k] = (ut, theta)
            if seen is None or ut <= seen[0]:
                continue
            turn = abs(rpm_list[self.model_rotors[k]]) * math.pi / 30.0 * (
                ut - seen[0])
            d = theta - seen[1]

            def miss(x):
                return abs((x + math.pi) % (2.0 * math.pi) - math.pi)
            plus, minus = miss(d - turn), miss(d + turn)
            if abs(plus - minus) < math.radians(40.0):
                continue
            self.spin_votes[k] += 1 if plus < minus else -1
            if abs(self.spin_votes[k]) >= cfg.SPIN_VOTES:
                self.spin_sense[k] = 1.0 if self.spin_votes[k] > 0 else -1.0
                if all(x is not None for x in self.spin_sense):
                    self.log.event(ut, "rotor spin senses (x model): %s"
                                   % "".join("+" if x > 0 else "-"
                                             for x in self.spin_sense))

    def counter_torque(self, flown, avail, trims):
        """Set the trims to cancel the props' reaction torque, on top of
        whatever trim the pilot has set (a trim that moved since we wrote
        it is the pilot's)."""
        cfg = self.cfg
        if not cfg.TORQUE_COUNTER or trims is None or None in trims:
            return
        if self.trim_written is None:
            self.user_trim = trims          # nothing of ours in it yet
        elif all(any(abs(t - w) > 0.005 for t, w in zip(trims, ref))
                 for ref in (self.trim_written,
                             self.trim_prev or self.trim_written)):
            # Neither of our last two writes: the pilot moved it.
            self.user_trim = tuple(t - c for t, c in zip(trims, self.counter))
        torque = (0.0, 0.0, 0.0)
        if flown is not None and None not in self.spin_sense:
            torque = tipping.reaction_torque(
                self.model.rotors, [1000.0 * q for q in flown.torques],
                self.spin_sense)
        u = (tipping.counter_trim(torque, avail) if avail is not None
             else (0.0, 0.0, 0.0))
        want = tuple(max(-1.0, min(1.0, a + b))
                     for a, b in zip(self.user_trim, u))
        ref = self.trim_written if self.trim_written is not None else trims
        if all(abs(a - b) < cfg.TRIM_WRITE_STEP for a, b in zip(want, ref)):
            return
        c = self.vessel.control
        try:
            c.pitch_trim, c.roll_trim, c.yaw_trim = want
        except Exception:                               # noqa: BLE001
            return
        self.trim_prev = self.trim_written
        self.trim_written = want
        self.counter = tuple(w - a for w, a in zip(want, self.user_trim))

    def brake_objective(self, v_nose):
        """Reverse thrust while moving forward along the nose, forward thrust
        while moving backward; None (props stopped) once nearly still, until
        the plane moves past BRAKE_RESUME_SPEED again."""
        if self.held and abs(v_nose) <= self.cfg.BRAKE_RESUME_SPEED:
            return None
        self.held = abs(v_nose) < self.cfg.BRAKE_HOLD_SPEED
        if self.held:
            return None
        return MOST_REVERSE if v_nose > 0.0 else MOST_FORWARD

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
