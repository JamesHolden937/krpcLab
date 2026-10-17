"""Boostback, coast and propulsive landing on the KSC pad.

Phase machine::

    STANDBY -> SEPARATION -> BOOSTBACK -> COAST -> LANDING_BURN -> TOUCHDOWN
                                            ^        |
                                            +- CORRECTION

Each tick re-predicts the touchdown point from the current state (see
:mod:`boosterland.trajectory`); every phase transition and every steering
command is derived from that single prediction, so there is no stored plan to
go stale.  The script never prints: telemetry goes to ``logs/LOG<n>`` and to
the in-game panel.
"""

import argparse
import math
import sys
import time
from dataclasses import fields

import krpc

from common import vec
from . import guidance, trajectory
from common import rcs as rcs_valve
from .config import Config, apply_overrides, defaults_fingerprint
from .engines import allocate, group_engines
from .environment import Environment
from .gui import ControlPanel, telemetry_lines
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from .proximity import ProximityScan
from .telemetry import Telemetry

STANDBY = "STANDBY"
SEPARATION = "SEPARATION"
BOOSTBACK = "BOOSTBACK"
COAST = "COAST"
CORRECTION = "CORRECTION"
LANDING_BURN = "LANDING_BURN"
TOUCHDOWN = "TOUCHDOWN"


def set_autopilot_attitude(autopilot, direction, up=None, roll=0.0):
    """Command a nose direction and, when asked, the roll about it.

    ``set_direction_and_up`` is the singularity-free form and the one to use;
    older kRPC servers have only ``up_reference``/``target_roll``, and older
    ones than that no roll control at all.  Fall back rather than fail: a
    booster that holds its nose and drifts in roll still lands.
    """
    autopilot.target_direction = tuple(direction)
    if up is None:
        try:
            autopilot.target_roll = float("nan")     # damp the rate only
        except AttributeError:
            pass
        return
    try:
        autopilot.set_direction_and_up(tuple(direction), tuple(up), roll)
        return
    except AttributeError:
        pass
    try:
        autopilot.up_reference = tuple(up)
        autopilot.target_roll = roll
    except AttributeError:
        pass


def set_autopilot_engaged(autopilot, engaged):
    """Engage or disengage, returning the exception instead of raising it.

    kRPC has used both an ``engaged`` property and engage/disengage(), which
    is the AttributeError below.  The outer catch is for KSP itself: engaging
    the autopilot makes kRPC total up the vessel's available torque, and
    ``ModuleGimbal.GetPotentialTorque`` throws ``IndexOutOfRange`` on an
    engine whose gimbal is not initialised yet -- which is exactly when this
    script connects, moments after staging.  LOG10 died there on the first
    tick after START, before the throttle had ever been touched.

    So it is reported, not raised.  Losing the attitude command is bad;
    losing the flight because the game was not ready to answer a question is
    worse, and the answer changes a second later.  The caller retries.
    """
    try:
        try:
            autopilot.engaged = engaged
        except AttributeError:
            (autopilot.engage if engaged else autopilot.disengage)()
        return None
    except Exception as exc:            # noqa: BLE001 -- KSP-side failure
        return exc


AIRBRAKE_MODULE = "ModuleAeroSurface"   # the stock airbrake's part module


def set_airbrake_angle(vessel, angle):
    """Set every airbrake's deploy angle; return the angles read back."""
    angles = []
    for surface in vessel.parts.control_surfaces:
        for m in surface.part.modules:
            if m.name == AIRBRAKE_MODULE and m.has_field("Deploy Angle"):
                try:
                    m.set_field_float("Deploy Angle", float(angle))
                    angles.append(m.get_field("Deploy Angle"))
                except Exception:       # noqa: BLE001 -- KSP-side failure
                    angles.append("refused")
    return angles


def prepare_vehicle(vessel, cfg):
    """Deploy every airbrake and light every liquid engine, at START.

    This replaces firing an action group, which only worked on a craft whose
    builder had put the right parts on AG2.  Airbrakes are found by their
    part module, set to ``AIRBRAKE_DEPLOY_ANGLE_DEG`` and deployed; their
    control axes are left as the craft has them.  Liquid engines
    are those burning ``LiquidFuel``; activating one leaves the throttle where
    it is, which is zero in STANDBY.  Returns the lines to log: the deploy
    angle is read back, because the game may clamp it to the part's range.
    Each part is tried on its own, so one that refuses does not stop the rest.
    """
    lines = []
    brakes, angles = 0, []
    for surface in vessel.parts.control_surfaces:
        part = surface.part
        try:
            module = next((m for m in part.modules
                           if m.name == AIRBRAKE_MODULE), None)
            if module is None:
                continue
            if module.has_field("Deploy Angle"):
                module.set_field_float("Deploy Angle",
                                       float(cfg.AIRBRAKE_DEPLOY_ANGLE_DEG))
                angles.append(module.get_field("Deploy Angle"))
            # Deployed, but *not* made pitch/yaw control surfaces: an
            # airbrake that steers modulates its own deployment, so the drag
            # curve probed during the flip is not the one the coast flies.
            # Measured at 1x on quicksave: with pitch/yaw on, 93/141/262/300 m
            # (rt6-1016); the action-group deploy without it, 16-51 m on six
            # of eight (ab7-1016).  Re-flown with corrections gated at
            # 1000 m (abs1-1016, 6 a cell): heavy 2 lost + 1 splashed, mean
            # ~1020 v 578; quicksave mean 242 v 160.
            surface.deployed = True
            brakes += 1
        except Exception as exc:        # noqa: BLE001 -- KSP-side failure
            lines.append("airbrake %s refused: %s"
                         % (part.title, str(exc).splitlines()[0]))
    lines.append("airbrakes deployed: %d, deploy angle %s (asked %.0f)"
                 % (brakes, "/".join(sorted(set(angles))) or "n/a",
                    cfg.AIRBRAKE_DEPLOY_ANGLE_DEG))
    lit = 0
    for engine in vessel.parts.engines:
        try:
            if "LiquidFuel" not in engine.propellant_names:
                continue
            engine.active = True
            lit += 1
        except Exception as exc:        # noqa: BLE001 -- KSP-side failure
            lines.append("engine %s refused: %s"
                         % (engine.part.title, str(exc).splitlines()[0]))
    lines.append("liquid engines activated: %d" % lit)
    if cfg.GIMBAL_ROLL_OFF:
        lines.append(set_gimbal_roll(vessel, False))
    return lines


def set_gimbal_roll(vessel, enabled):
    """Let the engine gimbals answer the roll input, or not.

    kRPC credits an engine cluster's gimbals with almost no roll torque
    (0.006-0.16 kN m on the heavy, against 300-1600 in pitch and yaw), so
    auto-tune sets roll gains of 1e3-1e5.  The off-axis engines' gimbals
    *do* roll the vehicle, hard: lit with the roll input pinned, the heavy
    spun up to 200 deg/s in 1.3 s (diag2-1017, LOG11353) and coned through
    the whole burn.  Off, roll is left to RCS and wheels -- the torque kRPC
    actually credits.  ``ModuleGimbal.enableRoll``, the editor's hidden
    per-axis toggle; returns the line to log."""
    done, refused = 0, 0
    for engine in vessel.parts.engines:
        try:
            for module in engine.part.modules:
                if module.name == "ModuleGimbal":
                    module.set_field_bool_by_id("enableRoll", bool(enabled))
                    done += 1
        except Exception:               # noqa: BLE001 -- KSP-side failure
            refused += 1
    return ("gimbal roll %s on %d engines%s"
            % ("enabled" if enabled else "disabled", done,
               " (%d refused)" % refused if refused else ""))


class EngineCluster:
    """The booster's liquid engines, lit center first and then in pairs.

    Built once at START from each engine's position in the vessel frame
    (whose y axis is the vessel's long axis, the thrust axis), grouped by
    ``engines.group_engines``.  ``set`` turns a demanded fraction of the
    cluster's thrust into per-engine ``thrust_limit`` values with the main
    throttle at 1, writing only the limits that moved.  Guidance's maximum
    acceleration reads ``max_thrust``, which ignores the limits, so the
    planner keeps seeing the whole cluster.
    """

    def __init__(self, vessel, cfg):
        frame = vessel.reference_frame
        self.engines = {}
        thrust, rows = {}, []
        for i, engine in enumerate(vessel.parts.engines):
            try:
                if ("LiquidFuel" not in engine.propellant_names
                        or engine.throttle_locked):
                    continue
                x, _, z = engine.part.position(frame)
                t = engine.max_vacuum_thrust or engine.max_thrust
            except Exception:           # noqa: BLE001 -- KSP-side failure
                continue
            self.engines[i] = engine
            thrust[i] = t
            rows.append((i, (x, z), t))
        self.thrust = thrust
        self.groups = group_engines(rows, cfg.ENGINE_AXIS_TOL_M,
                                    cfg.ENGINE_RING_TOL_M,
                                    cfg.ENGINE_PAIR_TOL)
        self.radius = {i: math.hypot(p[0], p[1]) for i, p, _ in rows}
        self.limits = {}

    def describe(self):
        return "engine groups (lighting order): " + " ".join(
            "[%d @ %.1f m]" % (len(g), max(self.radius[k] for k in g))
            for g in self.groups)

    def set(self, demand):
        for k, x in allocate(demand, self.groups, self.thrust).items():
            if abs(self.limits.get(k, -1.0) - x) > 0.005:
                try:
                    self.engines[k].thrust_limit = x
                    self.limits[k] = x
                except Exception:       # noqa: BLE001 -- KSP-side failure
                    pass

    def delivered(self):
        """Each group's actual thrust as a fraction of its maximum, e.g.
        ``1.00/0.43/0.00``: what the game made of the limits, so the log
        shows a group staying dark until the one before it is full."""
        out = []
        for g in self.groups:
            cap = sum(self.thrust[k] for k in g)
            try:
                got = sum(self.engines[k].thrust for k in g)
            except Exception:           # noqa: BLE001 -- KSP-side failure
                return None
            out.append(got / cap if cap > 0.0 else 0.0)
        return "/".join("%.2f" % x for x in out)

    def release(self):
        """Every limit back to 100%, for the player."""
        for k, engine in self.engines.items():
            try:
                engine.thrust_limit = 1.0
            except Exception:           # noqa: BLE001
                pass


class Autoland:
    def __init__(self, conn, cfg, logbook):
        self.conn = conn
        self.cfg = cfg
        self.log = logbook
        self.vessel = conn.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.frame = self.body.reference_frame
        self.control = self.vessel.control
        self.autopilot = self.vessel.auto_pilot

        ut = conn.space_center.ut
        logbook.event(ut, "vessel=%s" % self.vessel.name)
        # Which configuration flew this?  A sweep is twenty logs deep and the
        # only difference between them is a --set, so a log that does not say
        # cannot be read afterwards -- twenty flights of one session were
        # unattributable for exactly this reason.  Only the fields that differ
        # from the defaults, so the line stays one line.
        # The aim bias is listed even when it is the default, because it is a
        # calibration: the default itself changes between sessions, so
        # "(defaults)" alone does not say where this flight was aiming.
        always = ("AIM_BIAS_NORTH_M", "AIM_BIAS_EAST_M")
        changed = ["%s=%s" % (f.name, getattr(cfg, f.name))
                   for f in fields(cfg)
                   if getattr(cfg, f.name) != f.default or f.name in always]
        logbook.event(ut, "config: %s defaults_fingerprint=%s"
                      % (" ".join(changed) or "(defaults)",
                         defaults_fingerprint()))
        self.loop_rate = LoopRate()
        self.env = Environment(conn, self.vessel, self.body, cfg, logbook, ut)
        self.telemetry = Telemetry(conn, self.vessel, self.frame, cfg,
                                   logbook)
        self.proximity = ProximityScan(conn, self.vessel, self.frame, cfg,
                                       logbook)
        self.gui = ControlPanel(conn)

        self.state = STANDBY
        self.autostart = False   # --autostart: STANDBY acts as if START was pressed
        self.phase_start_ut = ut
        self.prediction = None
        self.roll_up = None      # last roll reference, for continuity
        self.rcs = rcs_valve.Valve(cfg, logbook)
        self.rcs_permitted = False      # STANDBY gets none; START opens it
        self.commanded_direction = None  # what ``aim`` last asked for
        self.tune_logged = 0            # log_attitude_tune's progress
        self.rcs_held = False           # LANDING_RCS_HELD has opened it
        self.nose_block_logged = False  # nose_blocked has said so
        self.autopilot_engaged = False
        self.correction_align = None    # (ut, align) from the last tick
        self.autopilot_retries = 0
        self.extra_lines = ["press START AUTOLAND"]
        self.gear_deployed = False
        self.steer = None               # the angle of attack the coast flies
        self.steer_history = []         # (ut, commanded aoa deg), for tracking
        self.corrections = 0
        self.deploy_angle = None        # DEPLOY_LAW's command, deg
        self.deploy_next_ut = 0.0
        self.deploy_record = None       # (angle, curve, ut) at the last step
        self.deploy_k = None            # d ln(Cd*A) / d angle, per degree
        self.deploy_resweep_ut = None   # re-sweep once the brakes have moved
        self.deploy_none = False        # no airbrake answered: law off
        self.correction_ready_ut = 0.0
        self.correction_start_miss = None   # the miss the running burn began at
        self.correction_best_miss = None    # ... and the least it has reached
        self.correction_retired = False     # a burn backfired; stop trying
        self.boostback_under = 0        # consecutive ticks inside the tolerance
        self.boostback_under_since = None   # ... and when that run started
        self.commanded_throttle = None  # what set_throttle last asked for
        self.demand = 0.0               # fraction of cluster thrust wanted
        self.cluster = None             # EngineCluster, built at START
        self.prev_nose = None           # (ut, direction) of the last tick
        self.nose_rate = None           # deg/s, see track_nose_rate
        self.roll_rate = None           # deg/s about the nose, signed
        self.throttle_diverged = False
        self.last_sample = None         # (position, mass) of the last tick
        self.frozen_since = None        # ut the position stopped changing
        self.finished = False
        self.exit_reason = None

    # -- helpers -----------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s %s" % (self.state, state, note))
        self.state = state
        self.phase_start_ut = ut
        self.correction_align = None    # each burn judges its own turn
        # A steering command carried through another phase is not a steady
        # one: the first COAST tick after a correction would otherwise read
        # the correction's 50-degree tilt as "held" (LOG10244: 87.9).
        self.steer_history = []

    def predict(self, snap, powered, steer=None):
        max_accel = snap.max_accel if powered else 0.0
        self.prediction = trajectory.predict_landing(
            self.env, snap.position, snap.velocity, snap.mass, max_accel,
            self.cfg, steer=steer)
        # Hand the descent back to the estimator: the next aerodynamic probe
        # wants to know where the booster will be when it is doing each speed,
        # so that a Mach bin is measured in the air it is going to be used in.
        # See Environment.probe_altitude.
        self.env.set_descent_profile(self.prediction.profile)
        return self.prediction

    def aim(self, direction, snap=None):
        """Point the nose, roll the roof into the trajectory plane, and route
        a big turn over the top.

        Without a roll command the autopilot only damps the roll rate, so the
        booster flies whatever roll the flip left it in.  ``snap`` is what
        makes both the roll reference and the flip routing computable -- the
        latter needs the vehicle's *real* attitude, not the last command --
        so a caller with no state to hand gets nose-only steering.
        """
        self.autopilot.reference_frame = self.frame
        up = None
        if snap is not None:
            if self.cfg.FLIP_THROUGH_VERTICAL:
                # Big turns are two pitches through the zenith.
                direction = guidance.flip_through_vertical(
                    snap.direction, direction, snap.position,
                    self.cfg.FLIP_VIA_VERTICAL_DEG,
                    self.cfg.FLIP_VERTICAL_PAST_DEG)
            elif self.cfg.FLIP_VIA_VERTICAL:
                # Big turns go over the top rather than nose-down through the
                # airstream; releases itself as the vehicle comes round.
                direction = guidance.flip_waypoint(
                    snap.direction, direction, snap.position,
                    self.cfg.FLIP_VIA_VERTICAL_DEG)
            if self.cfg.ROLL_ALIGN:
                up = guidance.roll_reference(snap.position, snap.velocity,
                                             direction, self.roll_up,
                                             self.cfg.ROLL_PLANE_MIN_HS_MPS)
                self.roll_up = up
        set_autopilot_attitude(self.autopilot, direction, up,
                               self.cfg.ROLL_OFFSET_DEG)
        # The *routed* direction, not the one asked for: while the flip is
        # going over the top the vehicle is meant to be pointed at the
        # waypoint, and measuring the error against the final aim would call
        # a turn that is going exactly to plan a 100-degree failure.
        self.commanded_direction = tuple(direction)

    def pointing_error(self, snap):
        """Degrees between the commanded nose and the real one, or -1.

        -1 is "not known", which the valve treats as no evidence either way.
        A missing answer must not look like a good one -- here that would be
        reporting 0 and shutting the thrusters off mid-flip.
        """
        if snap is None or self.commanded_direction is None:
            return -1.0
        if vec.norm(snap.direction) < 0.5:
            return -1.0
        return vec.angle_between(snap.direction, self.commanded_direction)

    def update_rcs(self, snap):
        """Open the valve for the turns, shut it for the holds.

        RCS used to be switched on at START and left on for the whole flight.
        Every phase here does need thrusters at some point -- the flip has no
        gimbal, and the descent has very little authority on anything else --
        but *needing them in the phase* is not needing them in the tick, and
        the ticks where the booster is already pointed where it was told
        outnumber the ones where it is turning.  In COAST -- the booster's
        aerodynamic phase, the shuttle's glide -- a turn is not a need either:
        the valve opens only when a control axis is pinned and the error is
        not closing on the wheels and airbrakes alone (``RCS_SAT_*``, the
        shuttle's rule).  Everywhere else it is the error relay, as on the
        shuttle: the flip, the burns and the landing are slews that diverge
        in seconds, and on saturation alone the valve stayed shut while the
        landing burn tipped 41 deg at full throttle (LOG10251, 47.7 m/s).
        See ``common.rcs``.
        """
        if (self.cfg.LANDING_RCS_HELD and self.state == LANDING_BURN
                and self.rcs_permitted):
            # The relay flips the valve on the pointing error, and kRPC's
            # auto-tune re-derives its gains from the torque with the valve
            # as it is: 112 kN m open, 1.5 shut, Kp 7.6 v 209 (LOG11023,
            # LOG11037) -- a 30x gain swing every time it flips, through the
            # one phase that cannot afford it.  Held open, the gains are
            # steady and the thrusters help hold the burn upright.
            if not self.rcs_held:
                self.rcs_held = True
                self._apply_rcs(True)
                self.log.event(snap.ut, "rcs held open for the landing burn")
            return
        if self.rcs_held:
            # Out of the burn: shut it and give the valve back to the relay,
            # which never saw it open and so would never close it.
            self.rcs_held = False
            self._apply_rcs(False)
            self.log.event(snap.ut, "rcs released after the landing burn")
        self.rcs.update(snap.ut, self.rcs_permitted,
                        self.pointing_error(snap), None,
                        apply=self._apply_rcs,
                        saturation=(snap.input_saturation
                                    if self.state == COAST else None))

    def _apply_rcs(self, wanted):
        try:
            self.control.rcs = wanted
        except Exception:                            # noqa: BLE001
            pass

    def set_throttle(self, throttle):
        """Command the throttle, and remember what was commanded.

        Every phase goes through here so ``check_throttle`` has something to
        compare the vessel's actual throttle against.
        """
        self.demand = throttle
        cluster = self.cluster
        if cluster is not None and len(cluster.groups) > 1:
            # Center engine first, then pairs, each to full: see
            # EngineCluster.  The main throttle only gates the cluster.
            cluster.set(throttle)
            throttle = 1.0 if throttle > 0.0 else 0.0
        self.commanded_throttle = throttle
        self.control.throttle = throttle

    def check_throttle(self, snap):
        """Say so in the log when the vessel is not at the commanded throttle.

        LOG16 spent the whole of SEPARATION at ``thr=0.98`` while the phase
        was setting the throttle to zero on every tick, gaining 70 m/s and
        9 km of downrange before it was terminated.  kRPC's throttle is the
        same value the player's throttle axis writes, and KSP reasserts that
        axis every frame, so a physical throttle left up from the ascent wins
        -- there is nothing this script can do about it from here.  What it
        can do is stop the surprise being silent: the log said ``thr=0.98``
        in a phase documented as engines-off and nothing pointed at it.

        Reported once per episode; the reading is a tick old, so a throttle
        that is merely changing does not trip it.
        """
        commanded = self.commanded_throttle
        if commanded is None:
            return
        off = abs(snap.throttle - commanded) > self.cfg.THROTTLE_DIVERGENCE
        if off == self.throttle_diverged:
            return
        self.throttle_diverged = off
        self.log.event(snap.ut,
                       "throttle %s: commanded %.2f, vessel reports %.2f%s"
                       % ("DIVERGED" if off else "back under control",
                          commanded, snap.throttle,
                          " -- is the game's own throttle up?" if off else ""))

    def miss_distance(self):
        """How far the *predicted* touchdown is from the pad."""
        if self.prediction is None:
            return float("nan")
        return trajectory.surface_distance(
            self.env, self.prediction.position, self.env.target)

    def miss_components(self, snap):
        """The predicted miss, signed: ``(long, cross)`` metres.

        ``miss=`` in the log is a distance, so it cannot say whether the
        booster is about to stop short or fly past -- and those want opposite
        corrections.  See ``trajectory.miss_components``.
        """
        if self.prediction is None:
            return float("nan"), float("nan")
        return trajectory.miss_components(
            self.env, self.prediction.position, self.env.target, snap.position)

    def pad_distance(self, snap):
        """How far the booster is from the pad *right now*.

        The predicted miss says where guidance thinks it is going; this says
        where it actually is.  LOG5 has only the first, so when the flight
        ended 135 s early there was no way to tell from the log whether the
        prediction had been any good.

        Measured from the **real pad**, not from ``env.target``, which carries
        whatever ``AIM_BIAS_*`` has been dialled in.  Guidance flies to the
        aim point and ``miss=`` is its error against it; ``pad=`` has to stay
        the honest answer to "how far from the pad did it land", or a bias
        would flatter itself by exactly its own size.
        """
        pad = getattr(self.env, "pad", self.env.target)
        return trajectory.surface_distance(self.env, snap.position, pad)

    def engage_autopilot(self, ut):
        """Engage, and keep trying on later ticks if the game refuses.

        KSP can throw out of the engage call itself while the gimbals are
        still settling after staging (see ``set_autopilot_engaged``).  That
        is a transient, so it is retried every tick rather than being allowed
        to end the flight -- the same lesson as the bounding box in LOG4:
        startup is the worst moment to ask, so ask again.
        """
        if self.autopilot_engaged:
            return True
        error = set_autopilot_engaged(self.autopilot, True)
        if error is None:
            self.autopilot_engaged = True
            if self.autopilot_retries:
                self.log.event(ut, "autopilot engaged after %d retries"
                               % self.autopilot_retries)
            return True
        if not self.autopilot_retries:
            self.log.event(ut, "autopilot refused engagement, retrying: %s"
                           % str(error).splitlines()[0])
        self.autopilot_retries += 1
        return False

    # -- phases ------------------------------------------------------------
    def run_standby(self, snap):
        self.set_throttle(0.0)
        if self.autostart or self.gui.start_pressed():
            self.autostart = False
            self.rcs_permitted = True   # allowed from here; see ``update_rcs``
            for line in prepare_vehicle(self.vessel, self.cfg):
                self.log.event(snap.ut, line)
            if self.cfg.ENGINE_ALLOCATION:
                self.cluster = EngineCluster(self.vessel, self.cfg)
                self.log.event(snap.ut, self.cluster.describe())
            self.control.sas = False
            self.autopilot.reference_frame = self.frame
            try:
                # kRPC controls roll only while the pointing error is under
                # this; above it the roll input is zero and roll drifts.
                self.autopilot.roll_threshold = float(
                    self.cfg.ROLL_THRESHOLD_DEG)
            except Exception:           # noqa: BLE001 -- older kRPC
                pass
            self.aim(snap.direction, snap)   # hold attitude, but stop the roll
            self.engage_autopilot(snap.ut)
            self.gui.set_start_label("RUNNING")
            self.log.event(snap.ut, "start: rcs=%s" % self.cfg.ENABLE_RCS)
            self.enter(SEPARATION, snap.ut)
        self.extra_lines = ["press START AUTOLAND"]

    def run_separation(self, snap):
        """Drift until the upper stage is out of the way, then flip.

        The engines are off here whatever else happens -- that is the whole
        point of the coast -- so this is the one part of the flip that can
        only be flown on RCS and reaction wheels.  With
        ``SEPARATION_PRETURN`` the attitude command is already the boostback
        aim, which overlaps the turn with a coast that was happening anyway:
        the vehicle recedes from the pad at several hundred m/s throughout,
        so every second spent pointed the wrong way is downrange the burn
        then has to undo (at 900 m/s an 11 s flip buys 10 km of extra work,
        which is what stretches the burn, ends it lower, and leaves no coast
        for CORRECTION).

        It defaults off all the same.  An unpowered flip at separation speed
        is barely controllable on a real booster -- no gimbal, and aero
        torque fighting back -- and it is what LOG9 looked like from the
        cockpit.  Holding the separation attitude here keeps the whole turn
        inside BOOSTBACK, where the engines are lit and the gimbal is flying
        it.  See ``FLIP_UNDER_POWER``.

        **How long to hold is measured, not assumed.**  It used to be a flat
        ``SEPARATION_COAST_S``, and since every downrange second is work the
        burn has to undo, a timer is expensive when it is too long and
        dangerous when it is too short.  ``ProximityScan`` propagates the
        booster forward under the acceleration the boostback burn is about to
        apply, against every craft within ``CLEARANCE_RANGE_M`` coasting on
        its own velocity, and the phase ends the first tick nothing is in the
        way.  ``SEPARATION_MAX_COAST_S`` goes anyway if something never
        clears -- a booster that never flips is lost for certain, one that
        flips next to the stage it dropped only probably is.
        """
        self.set_throttle(0.0)
        prediction = self.predict(snap, powered=True)
        solution = guidance.boostback_solution(
            self.env, snap.position, snap.velocity, snap.mass, snap.max_accel,
            snap.isp, prediction, self.cfg)
        elapsed = snap.ut - self.phase_start_ut
        cfg = self.cfg
        blocker = None
        if cfg.SEPARATION_CLEARANCE_SCAN:
            # The flip and the burn are one manoeuvre, so the scan has to see
            # the burn: at full throttle along the aim the booster covers tens
            # of metres in the first few seconds, and which way it moves
            # relative to the stage above it is the entire question.
            accel = vec.scale(vec.unit(solution.aim), snap.max_accel)
            blocker = self.proximity.check(snap.ut, snap.position,
                                           snap.velocity, accel)
            clear = blocker is None and elapsed >= cfg.SEPARATION_MIN_COAST_S
            expired = elapsed >= cfg.SEPARATION_MAX_COAST_S
        else:
            clear = elapsed >= cfg.SEPARATION_COAST_S
            expired = False

        # Turn if -- and only if -- the flip is being held anyway.  With
        # nothing in the way the phase ends this tick and BOOSTBACK flies the
        # whole flip under power, which is what LOG9 argued for and the
        # better answer.  But once the scan *is* holding, the choice is not
        # powered flip against unpowered flip, it is unpowered flip against
        # no flip, and LOG16 shows what no flip costs: the booster sat
        # broadside at Mach 2.4 with the grid fins deployed and lost an
        # airbrake to the airstream.  Turning while waiting is free.
        if cfg.SEPARATION_PRETURN or blocker is not None:
            self.aim(solution.aim, snap)

        self.extra_lines = [
            "sep coast : %6.1f s" % elapsed,
            "clearance : %s"
            % ("clear" if blocker is None
               else "%s %+.0fm in %.1fs" % (blocker.neighbour.name,
                                            blocker.separation, blocker.time)),
            "align   : %8.1f deg"
            % vec.angle_between(snap.direction, solution.aim),
        ]
        if clear:
            self.enter(BOOSTBACK, snap.ut, "clear after %.1fs" % elapsed)
        elif expired:
            note = "TIMEOUT after %.1fs" % elapsed
            if blocker is not None:
                note += ", %s still %.0fm inside" % (blocker.neighbour.name,
                                                     -blocker.separation)
            self.enter(BOOSTBACK, snap.ut, note)

    def run_boostback(self, snap):
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        prediction = self.predict(snap, powered=True)
        solution = guidance.boostback_solution(
            self.env, snap.position, snap.velocity, snap.mass, snap.max_accel,
            snap.isp, prediction, self.cfg)
        self.aim(solution.aim, snap)

        align = vec.angle_between(snap.direction, solution.aim)
        distance = self.miss_distance()
        long, cross = self.miss_components(snap)
        elapsed = snap.ut - self.phase_start_ut

        # Debounce the exit.  The predicted miss is noisy by a couple of
        # hundred metres near the end of boostback -- it is a fresh
        # propagation every tick off a drag estimate that is still settling --
        # and a bare threshold turns that noise into *when the burn stops*.
        # LOG29 and LOG30 flew the same config from the same start state and
        # ended boostback 3 s apart at the same miss (186 and 184); those 3 s
        # of burn are 370 m at the pad, and the two landed 4 m and 362 m out.
        # The phase was not imprecise, it was bimodal.  Requiring the miss to
        # stay inside the tolerance for BOOSTBACK_EXIT_TICKS ticks makes a
        # single dip unable to end the burn.
        #
        # The test is on the *signed* longitudinal miss, and that half is not
        # cosmetic.  Every actuator after this phase can only make the booster
        # land *shorter*: CORRECTION steers within
        # CORRECTION_MAX_TILT_DEG of retrograde and is gated on a gradient
        # that refuses a burn which lengthens, and the landing burn thrusts
        # straight anti-velocity by design (failure 5).  So an overshoot at
        # the boostback exit is a miss the rest of the flight can trim, and an
        # undershoot is a miss nothing can touch -- boostback is the last
        # place the vehicle can still add range.  Stopping on |miss| treats
        # the two as the same number.  It was survivable only while the
        # propagator understated the overshoot (failure 16); with an honest
        # prediction boostback burns those extra seconds, and on ``qs_steep``
        # it burned straight through the pad -- exiting at long=-130 m and
        # landing 142 and 162 m *short*, against 38 and 41 m long before.
        #
        # So the burn ends only inside the band [0, tolerance]: past the pad,
        # but not by more than the phase after it can close.
        under = (distance <= self.cfg.BOOSTBACK_TOLERANCE_M
                 and long >= -self.cfg.BOOSTBACK_UNDERSHOOT_M)
        if under:
            self.boostback_under += 1
            if self.boostback_under_since is None:
                self.boostback_under_since = snap.ut
        else:
            self.boostback_under = 0
            self.boostback_under_since = None
        # A tick is wall-clock paced (LOOP_SLEEP_S plus however long the kRPC
        # calls take), so a tick count is a different amount of *burn* on a
        # loaded machine than on an idle one -- which is the one thing a
        # comparison between two flights must not depend on.  Requiring game
        # seconds as well pins the debounce to the vehicle rather than to the
        # host; BOOSTBACK_EXIT_S at 0 leaves the tick count alone in charge.
        held = (snap.ut - self.boostback_under_since
                if self.boostback_under_since is not None else 0.0)
        if (self.boostback_under >= self.cfg.BOOSTBACK_EXIT_TICKS
                and held >= self.cfg.BOOSTBACK_EXIT_S):
            self.set_throttle(0.0)
            # `held` and the tick count go in the event because the debounce
            # is the one thing here that is paced by the *host* rather than
            # the vehicle: a tick is wall-clock work, so four of them are a
            # different amount of burn on a loaded machine.  Solo and on four
            # instances the same save exited 2.7 s apart -- 150 m at the pad
            # -- and nothing in the log said so.  Now it does.
            self.enter(COAST, snap.ut,
                       "miss=%.0fm long=%+.0f held=%.2fs ticks=%d"
                       % (distance, long, held, self.boostback_under))
        elif elapsed > self.cfg.BOOSTBACK_MAX_BURN_S:
            self.set_throttle(0.0)
            self.enter(COAST, snap.ut, "TIMEOUT miss=%.0fm" % distance)
        elif not self.autopilot_engaged:
            # Nothing is steering: a full-throttle burn along whatever
            # attitude the separation left would only deepen the miss.
            # FLIP_UNDER_POWER lights the engines without waiting for
            # alignment, so this is the check that replaces that wait.
            self.set_throttle(0.0)
        elif align > self.cfg.FLIP_ALIGN_DEG and not self.cfg.FLIP_UNDER_POWER:
            # Still flipping: hold thrust so the flip does not add error.
            self.set_throttle(0.0)
        elif (align > self.cfg.FLIP_ALIGN_DEG and self.cfg.BOOSTBACK_CLEAR_NOSE
              and self.nose_blocked(snap)):
            # Lit mid-flip, the thrust is along the nose, not the aim -- and
            # at the start of the flip the nose points at the stage it just
            # dropped.  SEPARATION's scan cleared the *aim*; three qs_hot
            # boosters then drove into the upper stage at 40 m/s and lost
            # their tanks (LOG10720, LOG10774, LOG11177; KSP.log: airbrake
            # into the ServiceBay, core into the verniers).
            self.set_throttle(0.0)
        else:
            self.set_throttle(solution.throttle)

        self.extra_lines = [
            "miss    : %8.0f m" % distance,
            "long    : %8.0f m" % long,
            "dv left : %8.1f m/s" % solution.dv,
            "burn    : %8.1f s" % solution.burn_time,
            "align   : %8.1f deg" % align,
            "ttl     : %8.0f s" % prediction.time_to_land,
        ]

    def nose_blocked(self, snap):
        """Would full thrust along the nose, as it points now, close on a
        neighbour?  Logged once per flip."""
        accel = vec.scale(vec.unit(snap.direction), snap.max_accel)
        blocker = self.proximity.check(snap.ut, snap.position, snap.velocity,
                                       accel)
        if blocker is not None and not self.nose_block_logged:
            self.nose_block_logged = True
            self.log.event(snap.ut, "boostback thrust held: nose toward %s, "
                           "%.0f m short of keep-out in %.1f s"
                           % (blocker.neighbour.name, -blocker.separation,
                              blocker.time))
        return blocker is not None

    def run_coast(self, snap):
        self.set_throttle(0.0)
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        self.env.refresh_lift(snap.position, snap.velocity, snap.rotation,
                              snap.direction)
        first = self.env.holdable_deg is None
        self.env.refresh_holdable(snap.ut, snap.position, snap.velocity,
                                  snap.rotation, snap.direction)
        if first and self.env.holdable_deg is not None:
            self.log.event(snap.ut, "holdable aoa %.1f deg (computed)"
                           % self.env.holdable_deg)
        self.measure_tracking(snap)
        # Fly the air first, and only then think about the engines.  A booster
        # falling through 30 km of thickening atmosphere with its grid fins out
        # is a wing, and a wing costs no propellant, no relight and none of the
        # three burns CORRECTION is rationed to -- see guidance.solve_steer.
        # The angle is *solved* against the propagator and then handed back to
        # it, so the stored prediction is of the trajectory the booster is
        # about to fly rather than of a ballistic one it is not.
        steer, steered = guidance.solve_steer(
            self.env, snap.position, snap.velocity, snap.mass, snap.max_accel,
            self.cfg)
        if steer is not None:
            self.prediction = steered
            self.env.set_descent_profile(steered.profile)
            self.steer = steer
            prediction = steered
            self.aim(guidance.steer_attitude(snap.position, snap.velocity,
                                             steer), snap)
        else:
            self.steer = None
            prediction = self.predict(snap, powered=True)
            self.aim(guidance.coast_attitude(self.env, snap.position,
                                             snap.velocity, prediction,
                                             self.cfg), snap)

        distance = self.miss_distance()
        height, needed, _ = trajectory.landing_burn_state(
            self.env, snap.position, snap.velocity, snap.max_accel, self.cfg,
            height=snap.landing_height, miss=distance)
        self.extra_lines = [
            "miss    : %8.0f m" % distance,
            "burn alt: %8.0f m" % needed,
            "margin  : %8.0f m" % (height - needed),
            "aoa     : %8.1f deg" % (math.degrees(self.steer.aoa)
                                     if self.steer else 0.0),
            "ttl     : %8.0f s" % prediction.time_to_land,
        ]
        if self.cfg.DEPLOY_LAW:
            self.run_deploy(snap)
            self.extra_lines.append("brakes  : %8.0f deg" % (
                self.deploy_angle if self.deploy_angle is not None
                else self.cfg.AIRBRAKE_DEPLOY_ANGLE_DEG))
        if snap.vertical_speed < 0.0 and height <= needed:
            self.enter(LANDING_BURN, snap.ut,
                       "h=%.0f needed=%.0f miss=%.0f"
                       % (height, needed, distance))
        elif self.correction_worthwhile(snap, prediction, distance):
            self.corrections += 1
            self.correction_start_miss = distance
            self.correction_best_miss = distance
            self.enter(CORRECTION, snap.ut, "miss=%.0fm" % distance)

    def run_deploy(self, snap):
        """Range by drag: step the airbrakes' deploy angle onto the pad.

        The only control after boostback that can make the booster land
        *longer* -- correction burns and the landing burn only shorten.
        Every ``DEPLOY_STEP_S`` it takes a Newton step on a gain it works out
        for itself, as the product of two things any vehicle can be
        measured for, so no constant here belongs to one craft:

        * ``S``, metres of along-track miss per unit of ln(Cd*A): the
          propagator flown twice, the second time with the drag curve scaled
          by ``1 + DEPLOY_SENS_EPS`` and the same steering.  Exact for this
          trajectory and this vehicle, and recomputed every step, so it
          follows the sensitivity as it collapses toward the ground.
        * ``k``, ln(Cd*A) per degree of deploy: the drag probe simulates the
          vessel as it is now, so the curve swept after a step against the
          curve before it, bin by bin, is this vehicle's brake authority at
          this angle.  The first step is a fixed ``DEPLOY_PROBE_DEG`` that
          takes the first reading.

        The step is ``-long / (S k)`` times ``DEPLOY_NEWTON_FRACTION``,
        rate-limited by ``DEPLOY_MAX_STEP_DEG``.  ``long`` is measured along
        the horizontal velocity, the axis drag acts on.  The curve is
        re-swept ``DEPLOY_SETTLE_S`` after a step, once the brakes have
        finished moving.  It acts for the whole coast.
        """
        cfg = self.cfg
        if self.deploy_none:
            return
        if (self.deploy_resweep_ut is not None
                and snap.ut >= self.deploy_resweep_ut):
            self.deploy_resweep_ut = None
            self.env.request_resweep()
        if snap.ut < self.deploy_next_ut or self.deploy_resweep_ut is not None:
            return
        up = vec.unit(snap.position)
        along = vec.project_out(snap.velocity, up)
        if vec.norm(along) < 1.0:
            return
        along = vec.unit(along)

        def long_of(prediction):
            miss = vec.project_out(vec.sub(prediction.position,
                                           self.env.target), up)
            return vec.dot(miss, along)

        eps = cfg.DEPLOY_SENS_EPS
        # Which miss the brakes answer.  The lift solve re-nulls the *steered*
        # prediction every tick, so judged on that the brakes see ~0 while
        # the lift creeps to its limit, and only when it saturates does the
        # miss appear -- all at once, late (LOG10637: 0 -> -500 m at 12.7 km,
        # 0 -> +240 at 4.7 km, brakes 30 -> 50 -> 30 -> 50).  Judged on the
        # unsteered prediction, drag carries the range and the lift is left
        # the residual and the cross-track.
        steer = None if cfg.DEPLOY_UNSTEERED else self.steer
        base = trajectory.predict_landing(
            self.env, snap.position, snap.velocity, snap.mass,
            snap.max_accel, cfg, steer=steer)
        more = trajectory.predict_landing(
            self.env, snap.position, snap.velocity, snap.mass,
            snap.max_accel, cfg, steer=steer, drag_scale=1.0 + eps)
        long = long_of(base)
        sens = (long_of(more) - long) / math.log(1.0 + eps)

        angle = (self.deploy_angle if self.deploy_angle is not None
                 else cfg.AIRBRAKE_DEPLOY_ANGLE_DEG)
        curve = self.env.curve_bins()
        swept = self.env.last_sweep_ut()
        if self.deploy_record is not None:
            a0, c0, t0 = self.deploy_record
            da = angle - a0
            if (abs(da) >= cfg.DEPLOY_LEARN_MIN_DEG and swept is not None
                    and swept > t0):
                ratios = sorted(math.log(n / o) for n, o in zip(curve, c0)
                                if n and o and n > 0.0 and o > 0.0)
                if ratios:
                    k = ratios[len(ratios) // 2] / da
                    if k > 0.0:
                        self.deploy_k = (k if self.deploy_k is None else
                                         0.5 * self.deploy_k + 0.5 * k)
        self.deploy_record = (angle, curve, snap.ut)
        self.deploy_next_ut = snap.ut + cfg.DEPLOY_STEP_S

        step = 0.0
        how = "hold"
        if abs(long) > cfg.DEPLOY_DEADBAND_M:
            if self.deploy_k is None:
                # No reading of the brakes yet: a fixed step the right way
                # (more drag shortens, so a long miss opens them).
                step = math.copysign(cfg.DEPLOY_PROBE_DEG, long * -sens)
                how = "probe"
            elif abs(sens * self.deploy_k) > 1e-9:
                step = (-long / (sens * self.deploy_k)
                        * cfg.DEPLOY_NEWTON_FRACTION)
                how = "newton"
            step = max(-cfg.DEPLOY_MAX_STEP_DEG,
                       min(cfg.DEPLOY_MAX_STEP_DEG, step))
        new = max(cfg.DEPLOY_MIN_DEG,
                  min(cfg.DEPLOY_MAX_DEG, angle + step))
        if abs(new - angle) >= 0.5:
            got = set_airbrake_angle(self.vessel, new)
            if not got:
                self.deploy_none = True
                self.log.event(snap.ut, "brakes: no airbrake answered, law off")
                return
            self.deploy_resweep_ut = snap.ut + cfg.DEPLOY_SETTLE_S
            self.log.event(
                snap.ut, "brakes %.0f -> %.0f %s long=%+.0f S=%.0f k=%s"
                " (read %s)" % (angle, new, how, long, sens,
                                "%.4f" % self.deploy_k
                                if self.deploy_k is not None else "-",
                                "/".join(sorted(set(map(str, got))))))
        self.deploy_angle = new

    def measure_tracking(self, snap):
        """How much of the commanded angle of attack is the vehicle holding?

        The command in force is the one the last tick set; the angle held is
        the nose's real angle off retrograde.  A pair counts only once the
        command has stayed within ``AERO_STEER_TRACK_STEADY_DEG`` for
        ``AERO_STEER_TRACK_STEADY_S`` -- a command that just moved has not had
        time to be held, and reading the turn as the vehicle's limit would
        throw away lift it does have.  See ``Environment.lift_slope_at``.
        """
        cfg = self.cfg
        if self.steer is None:
            self.steer_history = []
            return
        commanded = math.degrees(abs(self.steer.aoa))
        self.steer_history.append((snap.ut, commanded))
        window = snap.ut - cfg.AERO_STEER_TRACK_STEADY_S
        while len(self.steer_history) > 1 and self.steer_history[1][0] <= window:
            self.steer_history.pop(0)
        if self.steer_history[0][0] > window:
            return                                  # not steady for long enough
        values = [c for _, c in self.steer_history]
        if max(values) - min(values) > cfg.AERO_STEER_TRACK_STEADY_DEG:
            return
        held = vec.angle_between(vec.scale(snap.direction, -1.0),
                                 snap.velocity)
        if snap.ut - self.phase_start_ut < cfg.AERO_STEER_TRACK_SETTLE_S:
            return              # still swinging back from the last phase
        if held > max(values) + cfg.AERO_STEER_TRACK_STEADY_DEG:
            # Holding *more* than asked is a nose still coming back from
            # somewhere else, not authority: every first sample after a
            # correction read 46-88 deg against a 4-7 deg command and pinned
            # the tracking at 1 (sweep heavy1-1016, ab7-1016).
            return
        before = self.env.tracking_samples
        self.env.record_tracking(sum(values) / len(values), held)
        if before == 0 and self.env.tracking_samples:
            self.log.event(snap.ut, "aoa tracking: commanded %.1f deg, held "
                           "%.1f -> lift scaled by %.2f"
                           % (sum(values) / len(values), held,
                              self.env.aoa_tracking ** 2))

    def alignment_rate(self, ut, align):
        """Degrees per second the attitude error is closing, or None.

        None on the first tick of a burn, when there is nothing to compare
        against yet.
        """
        previous = self.correction_align
        self.correction_align = (ut, align)
        if previous is None or ut <= previous[0]:
            return None
        return (previous[1] - align) / (ut - previous[0])

    def turning_too_slowly(self, rate):
        """Is the vehicle failing to come round, rather than merely mid-turn?

        Unknown (the first tick) counts as turning: give RCS a tick to show
        what it can do before spending propellant on the answer.
        """
        return (self.cfg.CORRECTION_GIMBAL_THROTTLE > 0.0 and rate is not None
                and rate < self.cfg.CORRECTION_STUCK_RATE_DEG_S)

    def correction_making_it_worse(self, ut, distance):
        """Has the running burn actually deepened the miss it was lit to close?

        ``correction_gradient`` is a *prediction* -- one 5 m/s probe, linearised
        through a propagation -- and low in the atmosphere it is linearised
        through drag that goes as v^2, so it holds over the probe and not over
        the 40 m/s the burn goes on to spend.  LOG2 and LOG3 both flew two
        burns that reported the burn helping (``grad=15.99``, ``grad=13.23``)
        every tick while the miss ran 335 -> 855 and 927 -> 1596, and put the
        booster 1.7 km out; nothing in the phase was watching the one number
        that was disagreeing with the gradient the whole way.

        So this is the measurement rather than the prediction: the miss the
        burn started at, against the miss it has now.  The grace period is
        what keeps it from firing on the turn -- a burn that has to swing the
        nose round first legitimately gives ground before it takes any, and in
        the sim a working correction opens by about 0.4% before closing.  At
        1.3x it is far outside that and still cuts both of the real burns
        inside their first few seconds.
        """
        if self.correction_start_miss is None or distance != distance:
            return False
        best = getattr(self, "correction_best_miss", None)
        best = min(self.correction_start_miss if best is None else best,
                   distance)
        self.correction_best_miss = best
        if ut - self.phase_start_ut < self.cfg.CORRECTION_ABORT_GRACE_S:
            return False
        # Against the *best* miss the burn has reached, not the one it began
        # at.  A burn that closes the miss and then keeps going opens it
        # again, and measured against its start it is allowed to undo all of
        # its own work and 30% more: LOG10237's second burn took 900 m to
        # 314, burned on for 25 s at a gradient that still read "helping",
        # and stopped at 1068 having spent 7 t of a heavy booster's landing
        # propellant.  The best-so-far is the start until the burn improves
        # on it, so a burn that never helps is cut as before.  The floor of
        # a floor of CORRECTION_ABORT_FLOOR_M above the best keeps a burn that has nearly closed
        # the miss from being called a backfire on prediction noise.
        return distance > max(best * self.cfg.CORRECTION_ABORT_RATIO,
                              best + self.cfg.CORRECTION_ABORT_FLOOR_M)

    def correction_gradient(self, snap, prediction):
        """Metres of miss closed per m/s burned along the correction attitude.

        Returned positive when the burn helps.  The steering is bounded to what
        the vehicle can actually hold (see guidance.correction_attitude), so
        there are geometries -- an undershoot, mostly -- where nothing reachable
        improves the answer.  Burning then makes it worse, so this is checked
        before entering CORRECTION and every tick while it runs.
        """
        aim = guidance.correction_attitude(self.env, snap.position,
                                           snap.velocity, prediction, self.cfg,
                                           snap.landing_height)
        return -trajectory.miss_gradient(
            self.env, snap.position, snap.velocity, snap.mass, aim, self.cfg,
            self.cfg.CORRECTION_PROBE_DV,
            snap.max_accel if self.cfg.CORRECTION_GRADIENT_POWERED else 0.0)

    def correction_worthwhile(self, snap, prediction, distance):
        """Is there enough miss, altitude and time left to be worth a burn?

        The coast is where the error grows: drag and the rotating frame keep
        acting on the booster for a minute or more after boostback ends, and
        nothing about pointing retrograde corrects for that.  A short, gentle
        burn high up is far cheaper than trying to translate kilometres during
        the landing burn, which has seconds and a tilt limit to work with.
        """
        cfg = self.cfg
        if cfg.CORRECTION_OVERSHOOT_ONLY:
            # CORRECTION steers within CORRECTION_MAX_TILT_DEG of retrograde,
            # so in the game it only ever shortens: entered on a *short* miss
            # it deepened it every time (vert1-1016 LOG10435: 1152 -> 1499,
            # 1003 -> 2320, 1785 -> 2382, splashed 2 km short).
            long, cross = trajectory.miss_components(
                self.env, prediction.position, self.env.target, snap.position)
            if long == long and long < 0.0 and abs(long) >= abs(cross):
                return False
        return (distance > cfg.CORRECTION_ENTER_M
                and snap.max_accel > 0.0
                and snap.landing_height > cfg.CORRECTION_MIN_ALT_M
                and snap.landing_height <= cfg.CORRECTION_MAX_ALT_M
                and prediction.time_to_land > cfg.CORRECTION_MIN_TTL_S
                and self.corrections < cfg.CORRECTION_MAX_BURNS
                and not self.correction_retired
                and snap.ut >= self.correction_ready_ut
                and self.settled_enough_to_correct(snap)
                and self.correction_gradient(snap, prediction)
                > cfg.CORRECTION_MIN_GRADIENT)

    def settled_enough_to_correct(self, snap):
        """Has the drag curve stopped moving under the prediction?

        The first descent re-sweep throws the whole broadside curve away and
        re-probes it, and the prediction jumps when it does.  That jump is
        not the booster moving, and a correction fired on it is failure 11's
        mistake with a newer cause: four flights of one save all entered
        CORRECTION between 2 and 12 s after the re-sweep, on misses of
        350-400 m that the settled curve did not agree with.

        One re-sweep is not a converged curve either -- failure 14 measured
        that the *repeats* are what converge it, because the first one probes
        at altitudes derived from the curve it is replacing.  So the gate is
        time since that first re-sweep, not the re-sweep itself.

        0 disables it, which is the behaviour every flight before this had.
        """
        settle = self.cfg.CORRECTION_SETTLE_S
        if settle <= 0.0:
            return True
        reswept = getattr(self.env, "reswept_ut", None)
        if reswept is None:
            # No descent re-sweep yet, so the curve is still the broadside
            # one.  That is the least trustworthy prediction of the flight.
            return False
        return snap.ut - reswept >= settle

    def run_correction(self, snap):
        """A throttled-down boostback: same law, gentler, mid-flight."""
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        prediction = self.predict(snap, powered=True)
        solution = guidance.boostback_solution(
            self.env, snap.position, snap.velocity, snap.mass, snap.max_accel,
            snap.isp, prediction, self.cfg)
        # Same dv/throttle law as boostback, but steered off retrograde rather
        # than along the horizontal miss -- see guidance.correction_attitude.
        aim = guidance.correction_attitude(self.env, snap.position,
                                           snap.velocity, prediction, self.cfg,
                                           snap.landing_height)
        self.aim(aim, snap)

        align = vec.angle_between(snap.direction, aim)
        distance = self.miss_distance()
        elapsed = snap.ut - self.phase_start_ut
        cfg = self.cfg

        rate = self.alignment_rate(snap.ut, align)
        gradient = self.correction_gradient(snap, prediction)
        backfiring = self.correction_making_it_worse(snap.ut, distance)
        done = (distance <= cfg.CORRECTION_EXIT_M
                or elapsed > cfg.CORRECTION_MAX_BURN_S
                or snap.landing_height <= cfg.CORRECTION_MIN_ALT_M
                or prediction.time_to_land <= cfg.CORRECTION_MIN_TTL_S
                or gradient <= 0.0
                or backfiring)
        if done:
            self.set_throttle(0.0)
            # A backfire retires the phase, it does not merely end the burn.
            # ``correction_making_it_worse`` bounds one burn; nothing bounded
            # the *sequence*, and the sequence is where the damage is.  One
            # qs_hot flight ran eight burns: two that worked (367 -> 38,
            # 554 -> 62), three that achieved nothing (171 -> 148, 153 -> 157,
            # 159 -> 193 -- by then the burn's real effect was smaller than
            # the prediction noise it was chasing), and then three that each
            # aborted as backfiring and each left the miss worse than they
            # found it: 238 -> 560 -> 701 -> 1572, landing 1421 m out against
            # 56 m for the same config on the same save.  Every one of those
            # last three reported a healthy gradient on every tick.
            #
            # So the first backfire is taken as evidence about the phase
            # rather than about the burn: whatever the gradient says next
            # time, it has just been wrong by more than the abort ratio, and
            # re-entering has only ever made things worse.
            if backfiring and cfg.CORRECTION_RETIRE_ON_BACKFIRE:
                self.correction_retired = True
            # Let the trajectory (and the drag estimate) settle before judging
            # the miss again, or the phase flaps in and out on prediction noise.
            self.correction_ready_ut = snap.ut + cfg.CORRECTION_COOLDOWN_S
            self.enter(COAST, snap.ut, "miss=%.0fm burns=%d grad=%.2f%s%s"
                       % (distance, self.corrections, gradient,
                          " BACKFIRING from %.0fm" % self.correction_start_miss
                          if backfiring else "",
                          " -- CORRECTION retired" if self.correction_retired
                          else ""))
        elif align > cfg.CORRECTION_ALIGN_DEG and self.turning_too_slowly(rate):
            # Still a long way off the aim and not getting there: light the
            # engines anyway, gently, and let the gimbal do the turning.  A
            # booster falling through thick air has very little authority on
            # RCS and reaction wheels alone, and holding the throttle shut
            # until it is pointed is how LOG7 spent 36 s and a whole
            # correction burn achieving nothing while the miss grew
            # 341 -> 966 m unopposed.  The thrust is inefficient by
            # definition -- cos(align) useful at best, pushing sideways with
            # the rest -- but a burn that happens beats a burn that waits for
            # an attitude the vehicle cannot reach unpowered.
            #
            # The rate test is what keeps the waste off a vehicle that is
            # simply mid-turn: RCS coming round at a decent clip needs no
            # help, and burning through that turn costs accuracy for nothing.
            # Set CORRECTION_GIMBAL_THROTTLE to 0 to hold thrust regardless.
            self.set_throttle(min(cfg.CORRECTION_GIMBAL_THROTTLE,
                                  cfg.CORRECTION_MAX_THROTTLE))
        elif align > cfg.CORRECTION_ALIGN_DEG:
            self.set_throttle(0.0)
        else:
            self.set_throttle(min(solution.throttle,
                                  cfg.CORRECTION_MAX_THROTTLE))

        self.extra_lines = [
            "miss    : %8.0f m" % distance,
            "dv left : %8.1f m/s" % solution.dv,
            "align   : %8.1f deg" % align,
            "closing : %8.1f deg/s" % (rate if rate is not None else float("nan")),
            "m per dv: %8.2f m" % gradient,
            "ttl     : %8.0f s" % prediction.time_to_land,
        ]

    def log_attitude_tune(self, snap):
        """Say what kRPC's attitude controller is tuned to, once at the burn's
        start and once a second into thrust.  Auto-tune sets the gains from
        the torque kRPC believes is available -- the gimbal's at the present
        thrust, and the RCS only while the valve is open -- so the gains
        swing 30x with the valve (Kp 7.6 at 112 kN m, 209 at 1.5; LOG11023,
        LOG11037) and fall 15x as thrust comes up."""
        elapsed = snap.ut - self.phase_start_ut
        stage = self.tune_logged
        if stage >= 2 or (stage == 1 and (snap.throttle < 0.5
                                          or elapsed < 1.0)):
            return
        self.tune_logged = stage + 1
        try:
            ap = self.autopilot
            self.log.event(snap.ut, "attitude tune (%s, thr %.2f): time_to_peak"
                           " %s gains pitch %s yaw %s roll %s torque %s"
                           % ("burn start" if stage == 0 else "under thrust",
                              snap.throttle,
                              _fmt3(ap.time_to_peak), _fmt3(ap.pitch_pid_gains),
                              _fmt3(ap.yaw_pid_gains), _fmt3(ap.roll_pid_gains),
                              _fmt3([t / 1000.0 for t in
                                     self.vessel.available_torque[0]])))
        except Exception as exc:                            # noqa: BLE001
            self.log.event(snap.ut, "attitude tune unreadable: %r" % exc)

    def run_landing_burn(self, snap):
        self.log_attitude_tune(snap)
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        prediction = self.predict(snap, powered=True)
        # Throttle first: the steering law has to know how much acceleration
        # it is actually going to get, not how much the vehicle could make at
        # full thrust -- see guidance.terminal_attitude.
        command = guidance.landing_throttle(self.env, snap.position,
                                            snap.velocity, snap.max_accel,
                                            self.cfg,
                                            height=snap.landing_height)
        throttle = command.throttle
        # The vertical profile commands zero throttle for the first seconds of
        # the burn, which is exactly when the divert has the most lateral
        # speed to take out and the most time to do it in -- and with the
        # engines shut it can do neither.  On the 30 km/40 km entry state the
        # burn opens 45 m from the pad carrying 24.7 m/s sideways and is 90 m
        # past it by the time the profile lights the engines.  So when the
        # steering law has real work to do, hold enough thrust to steer with;
        # landing_throttle's feedback trim absorbs the early braking.
        zem = (trajectory.miss_vector(self.env, prediction)
               if self.cfg.LANDING_DIVERT_ON_PREDICTION else None)
        budget = self.tilt_budget(throttle)
        steer = guidance.terminal_command(
            self.env, snap.position, snap.velocity, snap.landing_height,
            prediction.time_to_land, throttle * snap.max_accel, self.cfg, zem,
            budget)
        if steer.demand > self.cfg.LANDING_DIVERT_DEMAND_M_S2:
            throttle = max(throttle, self.cfg.LANDING_DIVERT_MIN_THROTTLE)
            steer = guidance.terminal_command(
                self.env, snap.position, snap.velocity, snap.landing_height,
                prediction.time_to_land, throttle * snap.max_accel, self.cfg,
                zem, self.tilt_budget(throttle))
        # Thrust tilted off retrograde brakes with only its cosine, and the
        # vertical profile above sized the throttle as though all of it were
        # braking.  Left alone, a divert quietly under-brakes for exactly as
        # long as it steers -- the trim term only notices once the vehicle is
        # already fast -- so give back what the tilt takes.  Nothing changes
        # when the divert is off, because then the tilt is zero.
        tilt = vec.angle_between(steer.direction, vec.scale(snap.velocity, -1.0))
        if tilt > 1.0:
            throttle = min(self.cfg.LANDING_THROTTLE_CAP,
                           throttle / math.cos(math.radians(tilt)))
        self.set_throttle(throttle)
        self.aim(steer.direction, snap)

        self.extra_lines = [
            "miss    : %8.0f m" % self.miss_distance(),
            "legs alt: %8.1f m" % snap.legs_altitude,
            "v target: %8.1f m/s" % command.target_speed,
            "v error : %8.1f m/s" % (snap.speed - command.target_speed),
        ]
        if self.touched_down(snap, command):
            self.set_throttle(0.0)
            self.enter(TOUCHDOWN, snap.ut, "legs=%.1fm vs=%.1f"
                       % (snap.legs_altitude, snap.vertical_speed))

    def tilt_budget(self, throttle):
        """How far the burn may tilt without stealing braking, in degrees.

        Thrust tilted by theta brakes with only cos(theta) of itself, so
        holding the profile's deceleration while tilted needs the throttle
        opened to ``demanded / cos(theta)``.  That is only possible while
        there is throttle left to open; at the cap there is none, and the
        tilt comes out of the braking instead.  Five round-4 flights flew the
        divert without this and every one of them arrived broken -- 8 to
        44 m/s, carrying 17-42 m/s sideways, one still descending at 36 m/s a
        metre and a half up.  ``acos(demanded / cap)`` is the angle the spare
        throttle can pay for, and it closes to zero exactly when the vehicle
        needs everything it has to stop.

        Returns None when the budget is switched off, which restores the
        fixed ``LANDING_MAX_TILT_DEG`` bound on its own.
        """
        if not self.cfg.LANDING_DIVERT_THROTTLE_BUDGET:
            return None
        cap = self.cfg.LANDING_THROTTLE_CAP
        if cap <= 0.0:
            return 0.0
        return math.degrees(math.acos(vec.clamp(throttle / cap, 0.0, 1.0)))

    def touched_down(self, snap, command):
        """Cut the engines on contact, not on an altitude.

        Any cutoff above the ground is a free fall from that height, and the
        height itself is only an estimate -- terrain-relative, measured at the
        centre of mass, offset by a bounding box.  So the burn is flown until
        the game reports the vehicle down; the profile has it at
        ``TOUCHDOWN_SPEED`` by then.  The other two cases are backstops for a
        contact that never gets reported.
        """
        situation = self.conn.space_center.VesselSituation
        if snap.situation in (situation.landed, situation.splashed):
            return True
        if abs(snap.vertical_speed) > self.cfg.TOUCHDOWN_BACKSTOP_SPEED:
            # The backstops are height comparisons, and a height is an
            # estimate; a vehicle still descending at hundreds of m/s is not
            # about to be on the ground whatever the number says (LOG5 cut
            # the engines at 24 km and 386 m/s on a bad terrain reading).
            #
            # The test is on the *vertical* speed, and that is the whole
            # point.  It used to be on total speed, which cannot tell a
            # booster falling out of the sky from one lying on the pad
            # sliding along it -- and the second is what LOG480 was:
            # `legs=-0.5` (below the pad radius, i.e. down), `vs=+0.5`, and
            # `hs=38.2`, with the throttle at 0.95 because the profile saw
            # no height left and plenty of speed.  The engine drove it across
            # the ground.  LOG5's flight had `vs=-386` and is still caught;
            # a skid has `vs` of nearly nothing and is now let through to the
            # height backstops below, which cut it.
            return False
        if command.height <= self.cfg.TOUCHDOWN_ALT_M:
            return True                       # effectively on the deck already
        # Stopped descending short of the ground: cut rather than hover.
        return (command.height <= self.cfg.HOVER_CUT_ALT_M
                and snap.vertical_speed >= self.cfg.TOUCHDOWN_HOVER_VS)

    def run_touchdown(self, snap):
        self.set_throttle(0.0)
        distance = self.pad_distance(snap)
        self.extra_lines = ["down %0.0f m from pad" % distance]
        if snap.ut - self.phase_start_ut > 2.0:
            # Say what the game thinks the vehicle is doing, not just how far
            # away it is: LOG5 signed off "landed 15706 m from pad" while the
            # booster was still 24 km up doing 386 m/s.
            self.finish("%s %.0f m from pad, alt=%.0f spd=%.1f"
                        % (snap.situation, distance, snap.landing_height,
                           snap.speed))

    # -- main loop ---------------------------------------------------------
    def landing_gear(self, snap):
        if self.gear_deployed:
            return
        # The burn is the last thing that happens, so anything the legs are
        # going to cost -- drag, and a bin of the curve written without them
        # -- is cheaper paid now than a leg that is still folding at
        # touchdown. GEAR_DEPLOY_ALT_M remains as a backstop for a flight
        # that somehow reaches the ground without the phase.
        why = None
        if self.cfg.GEAR_ON_LANDING_BURN and self.state == LANDING_BURN:
            why = "landing burn"
        elif self.state in (COAST, CORRECTION, LANDING_BURN) \
                and snap.height_above_pad <= self.cfg.GEAR_DEPLOY_ALT_M:
            why = "%.0f m" % self.cfg.GEAR_DEPLOY_ALT_M
        if why is None:
            return
        self.control.gear = True
        if self.cfg.GEAR_ACTION_GROUP:
            self.control.set_action_group(self.cfg.GEAR_ACTION_GROUP, True)
        self.gear_deployed = True
        self.log.event(snap.ut, "gear down at %.0f m (%s)"
                       % (snap.height_above_pad, why))

    def finish(self, reason):
        self.finished = True
        self.exit_reason = reason

    def vessel_lost(self, snap):
        """Has the vessel broken up or been destroyed?  Logs it; True ends it.

        A destroyed vessel does not always raise: its streams freeze on the
        last values, so the loop kept flying a dead snapshot -- same position,
        86 m/s, 0 t -- until the harness timed out (LOG10748, LOG10512), and the
        flight read as a hover rather than a crash.  Losing parts in flight
        (LOG10720, the tanks gone in the boostback flip) leaves a live vessel;
        that is only said, so the log shows where the mass went.
        """
        cfg = self.cfg
        last = self.last_sample
        self.last_sample = (snap.position, snap.mass)
        if last is None:
            return False
        if (last[1] > 0.0
                and snap.mass < last[1] * (1.0 - cfg.MASS_LOSS_FRACTION)):
            self.log.event(snap.ut, "mass %.2f -> %.2f t in one tick: parts lost"
                           % (last[1] / 1000.0, snap.mass / 1000.0))
        if snap.position != last[0] or snap.speed < cfg.DEAD_STREAM_MIN_SPEED:
            self.frozen_since = None
            return False
        if self.frozen_since is None:
            self.frozen_since = snap.ut
            return False
        if snap.ut - self.frozen_since < cfg.DEAD_STREAM_S:
            return False
        self.log.event(snap.ut, "vessel lost: position frozen %.1f s at %.1f m/s,"
                       " mass %.2f t" % (snap.ut - self.frozen_since,
                                        snap.speed, snap.mass))
        self.finish("destroyed %.0f m from pad, alt=%.0f spd=%.1f"
                    % (self.pad_distance(snap), snap.landing_height,
                       snap.speed))
        return True

    def tick(self):
        started = time.monotonic()
        state = self.state
        snap = self._tick()
        # What control rate each phase actually got, sampled here rather
        # than in run() because the farm harness drives tick() itself: a
        # burn whose commands land late was flown through a latency, not a
        # vehicle, and the log has to say so.
        if snap is not None:
            self.loop_rate.sample(state, snap.ut, time.monotonic() - started)
        return snap

    def track_nose_rate(self, snap):
        """How fast the nose is swinging, deg/s, from this tick's direction
        and the last one's -- streamed already, so it costs no call.  Beside
        ``err=`` and ``sat=`` it tells a pinned input that is turning the
        vehicle (and will overshoot) from one losing to a standing moment."""
        prev = self.prev_nose
        self.prev_nose = (snap.ut, snap.direction, snap.rotation)
        if prev is not None and snap.ut - prev[0] > 1e-3:
            dt = snap.ut - prev[0]
            self.nose_rate = vec.angle_between(prev[1], snap.direction) / dt
            # Roll: the turn from the last attitude to this one, as a
            # rotation vector, along the nose.
            dq = vec.quat_mul(snap.rotation, vec.quat_conjugate(prev[2]))
            if dq[3] < 0.0:
                dq = tuple(-c for c in dq)
            s = vec.norm(dq[:3])
            if s > 1e-12:
                angle = 2.0 * math.atan2(s, dq[3])
                axis = vec.scale(dq[:3], 1.0 / s)
                self.roll_rate = (math.degrees(angle) / dt
                                  * vec.dot(axis, snap.direction))
            else:
                self.roll_rate = 0.0

    def _tick(self):
        snap = self.telemetry.sample(self.env)
        self.track_nose_rate(snap)
        if self.gui.terminate_pressed():
            self.log.event(snap.ut, "TERMINATE pressed in game")
            self.finish("terminated from GUI")
            return snap

        if self.vessel_lost(snap):
            return snap

        if self.state != STANDBY:
            self.engage_autopilot(snap.ut)   # no-op once it has taken
        self.check_throttle(snap)

        handler = {
            STANDBY: self.run_standby,
            SEPARATION: self.run_separation,
            BOOSTBACK: self.run_boostback,
            COAST: self.run_coast,
            CORRECTION: self.run_correction,
            LANDING_BURN: self.run_landing_burn,
            TOUCHDOWN: self.run_touchdown,
        }[self.state]
        handler(snap)
        self.landing_gear(snap)
        # After the handler, so the error is measured against the attitude
        # this tick actually commanded rather than the last one's.
        self.update_rcs(snap)

        lines = telemetry_lines(self.state, snap, self.extra_lines)
        self.gui.update(lines)
        wrote = self.log.telemetry(snap.ut, compact_line(
            self.state, snap, self.prediction, self))
        if wrote and self.cfg.DIAG_STATE:
            self.log.event(snap.ut, diag_line(snap, self))
        return snap

    def run(self):
        # The sleep is in game seconds when the game is not running at 1x.  A
        # fixed wall-clock sleep would quietly halve the control rate the
        # vehicle sees for every doubling of the time scale, and a flight flown
        # that way is not reproducible at 1x.
        wait = sleeper(self.cfg, lambda: self.conn.space_center.ut, self.conn)
        while not self.finished:
            snap = self.tick()
            wait(self.cfg.LOOP_SLEEP_S,
                 snap.ut if snap is not None else self.conn.space_center.ut)

    def shutdown(self, reason):
        """Hand control back to the player in a safe, predictable state."""
        ut = 0.0
        try:
            ut = self.conn.space_center.ut
            self.control.throttle = 0.0
            if self.cluster is not None:
                self.cluster.release()
            if self.cfg.GIMBAL_ROLL_OFF:
                set_gimbal_roll(self.vessel, True)
            set_autopilot_engaged(self.autopilot, False)
            self.control.sas = True
        except Exception as exc:                     # game closed / vessel gone
            self.log.event(ut, "shutdown control error: %r" % exc)
        self.log.event(ut, "shutdown: %s" % reason)
        self.log.event(ut, self.loop_rate.report())
        self.telemetry.close()
        self.gui.close()
        try:
            self.conn.close()
        except Exception:
            pass


def _fmt3(values):
    return "(" + ", ".join("%.3g" % v for v in values) + ")"


def compact_line(state, snap, prediction, run):
    """One dense telemetry line -- readable, but cheap to scan in bulk."""
    miss = run.miss_distance() if prediction else float("nan")
    ttl = prediction.time_to_land if prediction else float("nan")
    # cda is the Cd*A at the Mach the booster is doing *now*; the propagator
    # reads a whole curve, so logging the Mach alongside it is what makes the
    # column readable -- 50 m^2 at M1.2 and 20 m^2 at M2.5 is the same vehicle.
    mach = run.env.mach(snap.speed, snap.mean_altitude)
    # Signed, because the distance alone cannot say which way: `long` is
    # positive when the prediction lands *beyond* the aim point.
    along, cross = run.miss_components(snap) if prediction else (
        float("nan"), float("nan"))
    return ("%-12s alt=%8.0f legs=%8.1f%s vs=%7.1f hs=%7.1f spd=%7.1f thr=%.2f "
            "m=%7.2ft pad=%8.0f miss=%8.0f long=%+7.0f cross=%+6.0f "
            "ttl=%6.0f cda=%5.2f M=%4.2f aoa=%+5.1f hold=%4.1f cla=%s trk=%.2f%s"
            % (state, snap.height_above_pad, snap.legs_altitude,
               " " if snap.surface_altitude_ok else "?",
               snap.vertical_speed, snap.horizontal_speed, snap.speed,
               run.demand if run.cluster is not None else snap.throttle,
               snap.mass / 1000.0, run.pad_distance(snap),
               miss, along, cross, ttl, run.env.drag_area, mach,
               math.degrees(run.steer.aoa) if run.steer else 0.0,
               # What the vehicle is *actually* holding, against what it was
               # asked for.  A commanded angle of attack is a request; the
               # airstream gets a vote, and a propagator that credits itself
               # with lift the booster is not making predicts a flight nobody
               # flies -- which is how a steered qs_hot held a predicted miss
               # of 0-12 m all the way down and landed 202 m out.
               vec.angle_between(vec.scale(snap.direction, -1.0),
                                 snap.velocity),
               "%6.2f" % run.env.lift_slope
               if run.env.lift_slope is not None else "     -",
               run.env.aoa_tracking,
               " can=%.1f" % run.env.holdable_deg
               if run.env.holdable_deg is not None else "")
            + attitude_fields(snap, run) + engine_fields(state, run))


def engine_fields(state, run):
    """` eng=` each engine group's delivered thrust, a fraction of its own
    maximum, in lighting order -- only while the engines can burn and the
    cluster is split (a single group would just repeat `thr=`)."""
    cluster = run.cluster
    if (cluster is None or len(cluster.groups) < 2
            or state not in ("BOOSTBACK", "CORRECTION", "LANDING_BURN")):
        return ""
    got = cluster.delivered()
    return " eng=%s" % got if got else ""


def attitude_fields(snap, run):
    """The command beside the attitude: ` cmd=` is the commanded nose's angle
    off retrograde, ` err=` how far the real nose is from it, ` sat=` the
    largest control input (1 = an axis pinned).  `hold` alone could not say
    whether a landing burn 50 deg off retrograde was steering there or losing
    the vehicle (LOG10969, LOG10971 tipped over at full throttle)."""
    cmd = getattr(run, "commanded_direction", None)
    if cmd is None or vec.norm(cmd) < 0.5 or snap.speed < 1.0:
        return ""
    roll = roll_error(snap, getattr(run, "roll_up", None))
    rate = getattr(run, "nose_rate", None)
    spin = getattr(run, "roll_rate", None)
    inputs = getattr(snap, "inputs", None)
    return (" cmd=%.1f err=%.1f sat=%s%s%s%s%s"
            % (vec.angle_between(cmd, vec.scale(snap.velocity, -1.0)),
               run.pointing_error(snap),
               "%.2f" % snap.input_saturation
               if snap.input_saturation is not None else "-",
               " roll=%.0f" % roll if roll is not None else "",
               " w=%.1f" % rate if rate is not None else "",
               " r=%+.0f" % spin if spin is not None else "",
               " in=%+.2f/%+.2f/%+.2f" % inputs if inputs else ""))


def roll_error(snap, up):
    """Degrees between the vehicle's roof and the commanded ``up``, both
    taken perpendicular to the nose; None if either is unknowable.

    The roof is the vessel frame's -z (kRPC: x right, y nose, z bottom).
    Which way the attitude quaternion applies is checked, not assumed: the
    convention that carries the body's +y onto the streamed nose is the one
    used.  A booster with 1.5 kN m of roll authority (LOG11023) cannot follow
    a roll reference that swings as the horizontal speed passes through zero
    -- this says whether it was asked to."""
    if up is None or snap.rotation is None or len(snap.rotation) != 4:
        return None
    nose = vec.unit(snap.direction)
    for q in (snap.rotation, vec.quat_conjugate(snap.rotation)):
        if vec.norm(vec.sub(vec.quat_rotate(q, (0.0, 1.0, 0.0)), nose)) < 0.05:
            roof = vec.project_out(vec.quat_rotate(q, (0.0, 0.0, -1.0)), nose)
            want = vec.project_out(up, nose)
            if vec.norm(roof) < 1e-6 or vec.norm(want) < 1e-6:
                return None
            return vec.angle_between(roof, want)
    return None


def diag_line(snap, run):
    """Everything an offline replay needs for this tick -- ``DIAG_STATE`` only.

    The compact line records ``cda`` at the Mach the booster is doing, which
    is not enough to re-fly its prediction: the propagator reads the whole
    curve, and the question these logs kept failing to answer is whether a
    walking prediction is the *curve* being revised or the propagation being
    wrong.  Emitted at telemetry pace, so it re-anchors under time warp with
    the line it belongs to.
    """
    r, v = snap.position, snap.velocity
    curve = " ".join("%.2f:%.1f" % (m, a)
                     for m, a in run.env.curve_snapshot())
    pred = run.prediction
    pr = pred.position if pred else (float("nan"),) * 3
    probe = run.env.diag_probe
    if probe is None:
        probes = ""
    else:
        mach, real, descent, aoa = probe
        probes = (" probe=M%.2f,%s,%s,aoa%.1f"
                  % (mach,
                     "%.1f" % real if real is not None else "-",
                     "%.1f" % descent if descent is not None else "-", aoa))
    return ("diag r=%.1f,%.1f,%.1f v=%.2f,%.2f,%.2f m=%.1f acc=%.3f "
            "pred=%.1f,%.1f,%.1f ttl=%.1f%s curve=[%s]"
            % (r[0], r[1], r[2], v[0], v[1], v[2], snap.mass, snap.max_accel,
               pr[0], pr[1], pr[2],
               pred.time_to_land if pred else float("nan"), probes, curve))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="boosterland", add_help=True,
        description="Boostback and land a KSP booster on the KSC pad.")
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=50000)
    parser.add_argument("--stream-port", type=int, default=50001)
    parser.add_argument("--name", default="boosterland")
    parser.add_argument("--set", action="append", default=[], metavar="K=V",
                        help="override a Config field, repeatable")
    parser.add_argument("--autostart", action="store_true",
                        help="do not wait for the panel's START button")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    run = None
    status = 0
    reason = "shutdown"
    try:
        conn = krpc.connect(name=args.name, address=args.address,
                            rpc_port=args.rpc_port,
                            stream_port=args.stream_port)
        run = Autoland(conn, cfg, log)
        run.autostart = args.autostart
        if args.autostart:
            run.gui.hide_start()
        run.run()
        reason = run.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
    except Exception as exc:
        log.event(0.0, "FATAL %r" % exc)
        reason = "error: %r" % exc
        status = 1
    finally:
        if run is not None:
            run.shutdown(reason)
        log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
