"""Boostback, coast and propulsive landing on the KSC pad.

Phase machine::

    STANDBY -> SEPARATION -> BOOSTBACK -> COAST -> LANDING_BURN -> TOUCHDOWN

Each tick re-predicts the touchdown point from the current state (see
:mod:`boosterland.trajectory`); every phase transition and every steering
command is derived from that single prediction, so there is no stored plan to
go stale.  The script never prints: telemetry goes to ``logs/LOG<n>`` and to
the in-game panel.
"""

import math
import sys
import time
from dataclasses import fields, replace

import krpc

from common import vec
from common import partmodule  # noqa: E402
from common.krpcpilot import vessel_gone
from . import gearcache, guidance, trajectory
from common import rcs as rcs_valve
from .config import Config, apply_overrides, defaults_fingerprint
from .engines import allocate, group_engines
from . import dragcache
from .environment import Environment
from common.panel import ACCENT_BOOSTER, ControlPanel
from .gui import telemetry_lines
from common.idle import IdleGimbals, VacuumSurfaces
from common.krpcpilot import (  # noqa: F401
    set_autopilot_attitude, set_autopilot_engaged)
from common import cli
from common.launchsite import launch_site
from common.attitudelag import LagFit
from common.jitter import SAMPLES as JITTER_SAMPLES, Jitter
from common.touchdown import (landing_limits, read_leg_constants, read_legs,
                              touchdown_limit)
from common.logbook import Logbook
from common.pacing import LoopRate, sleeper
from .proximity import ProximityScan
from .telemetry import Telemetry

STANDBY = "STANDBY"
SEPARATION = "SEPARATION"
BOOSTBACK = "BOOSTBACK"
COAST = "COAST"
LANDING_BURN = "LANDING_BURN"
TOUCHDOWN = "TOUCHDOWN"



AIRBRAKE_MODULE = "ModuleAeroSurface"   # the stock airbrake's part module


def set_airbrake_angle(vessel, angle, cfg=None):
    """Set every airbrake's deploy angle; return the angles read back.  With
    ``cfg.FIN_BRAKES`` the brake fins (``brake_fins``) follow, at the same
    fraction of their own range as ``angle`` is of ``DEPLOY_MAX_DEG``."""
    angles = []
    if cfg is not None and cfg.FIN_BRAKES:
        frac = max(0.0, min(1.0, float(angle) / cfg.DEPLOY_MAX_DEG))
        for surface, _ in _BRAKE_FINS.get(id(vessel), ()):
            try:
                top = surface.max_deploy_angle
                surface.deploy_angle = top * frac
                angles.append("%.1f" % surface.deploy_angle)
            except Exception:       # noqa: BLE001 -- KSP-side failure
                angles.append("refused")
    for surface in vessel.parts.control_surfaces:
        for m in surface.part.modules:
            if m.name == AIRBRAKE_MODULE and partmodule.has_field(m, "Deploy Angle"):
                try:
                    partmodule.set_field_float(m, "Deploy Angle", float(angle))
                    angles.append(partmodule.get_field(m, "Deploy Angle"))
                except Exception:       # noqa: BLE001 -- KSP-side failure
                    angles.append("refused")
    return angles


_BRAKE_FINS = {}     # id(vessel) -> [(ControlSurface, invert)], set at START


def brake_fins(vessel):
    """Control surfaces that are not airbrakes, in rings that can brake
    without turning: per part type and station along the nose, an even
    number, at least 4, spaced evenly round the axis.  Sorted by azimuth,
    alternate fins get the opposite deploy direction: the two halves' roll
    moments cancel, and each half (n/2 fins, 720/n apart) has no net side
    force -- opposite fins deployed the same way make a pure roll couple.
    Two fins cannot: alternated, both push the same way and the pair pitches
    the vehicle.  Stock ``ModuleControlSurface`` adds the
    deploy angle, signed by ``deployInvert``, to the control deflection and
    clamps the sum to 1.5 x ``ctrlSurfaceRange`` (Assembly-CSharp,
    ``CtrlSurfaceUpdate``), so the fins still steer on top.  Returns
    ``[(surface, invert)]``; rings that are odd or uneven are left out."""
    frame = vessel.reference_frame
    rings = {}
    for surface in vessel.parts.control_surfaces:
        part = surface.part
        try:
            if any(m.name == AIRBRAKE_MODULE for m in part.modules):
                continue
            x, y, z = part.position(frame)
        except Exception:               # noqa: BLE001 -- KSP-side failure
            continue
        key = (part.name, round(y * 2.0))       # 0.5 m stations
        rings.setdefault(key, []).append(
            (math.atan2(z, x), math.hypot(x, z), surface))
    found = []
    for ring in rings.values():
        n = len(ring)
        if n < 4 or n % 2:
            continue
        ring.sort(key=lambda r: r[0])
        step = 2.0 * math.pi / n
        gaps = [(ring[(i + 1) % n][0] - ring[i][0]) % (2.0 * math.pi)
                for i in range(n)]
        radii = [r[1] for r in ring]
        if (max(abs(g - step) for g in gaps) > 0.15 * step
                or max(radii) - min(radii) > 0.1 * max(radii)):
            continue
        found += [(r[2], i % 2 == 1) for i, r in enumerate(ring)]
    return found


def symmetry_order(parts):
    """The roll symmetry the vehicle's aerodynamic surfaces have, from
    ``[(key, azimuth rad, radius m)]`` (key: part type and station): the gcd
    of the sizes of the evenly spaced rings, each ring of 2 or more; 1 when
    any off-axis surface is not in such a ring.  Surfaces on the axis do not
    count."""
    rings = {}
    for key, az, radius in parts:
        if radius < 0.1:
            continue
        rings.setdefault(key, []).append((az, radius))
    order = 0
    for ring in rings.values():
        n = len(ring)
        if n < 2:
            return 1
        ring.sort()
        step = 2.0 * math.pi / n
        gaps = [(ring[(i + 1) % n][0] - ring[i][0]) % (2.0 * math.pi)
                for i in range(n)]
        radii = [r[1] for r in ring]
        if (max(abs(g - step) for g in gaps) > 0.15 * step
                or max(radii) - min(radii) > 0.1 * max(radii)):
            return 1
        order = math.gcd(order, n)
    return max(1, order)


def roll_symmetry(vessel):
    """``symmetry_order`` of the vessel's control surfaces and airbrakes."""
    frame = vessel.reference_frame
    parts = []
    for surface in vessel.parts.control_surfaces:
        try:
            part = surface.part
            x, y, z = part.position(frame)
        except Exception:               # noqa: BLE001 -- KSP-side failure
            continue
        parts.append(((part.name, round(y * 2.0)), math.atan2(z, x),
                      math.hypot(x, z)))
    return symmetry_order(parts)


def nearest_slot(angle, slot, current, hysteresis):
    """The multiple of ``slot`` (rad) to hold the roof at, the vehicle's
    roof ``angle`` (rad) from the reference: the nearest one, but the
    ``current`` index is kept until another is nearer by ``hysteresis`` of
    a slot, so the command does not chatter at the midpoint."""
    near = round(angle / slot)
    if current is None:
        return near
    if abs(angle - near * slot) + hysteresis * slot < abs(angle
                                                          - current * slot):
        return near
    return current


def deploy_brake_fins(vessel, cfg):
    """Alternate the fins' deploy direction and deploy them at the start
    angle.  Returns the line to log."""
    fins = brake_fins(vessel)
    _BRAKE_FINS[id(vessel)] = fins
    done, why = 0, None
    for surface, invert in fins:
        try:
            module = next(m for m in surface.part.modules
                          if m.name == "ModuleControlSurface")
            partmodule.set_field_bool_by_id(module, "deployInvert", invert)
            surface.deployed = True
            done += 1
        except Exception as exc:        # noqa: BLE001 -- KSP-side failure
            why = str(exc).splitlines()[0] if str(exc) else repr(exc)
    got = set_airbrake_angle(vessel, 0.0, cfg)
    top = fins[0][0].max_deploy_angle if fins else float("nan")
    return ("brake fins deployed: %d of %d, alternate directions, angle %s "
            "of max %.1f%s"
            % (done, len(fins), "/".join(sorted(set(got))) or "n/a", top,
               " (%s)" % why if why else ""))


def gear_parts(vessel):
    """Every deployable leg and wheel: what the gear group moves."""
    found = []
    for kind in ("legs", "wheels"):
        try:
            found += [p for p in getattr(vessel.parts, kind) if p.deployable]
        except Exception:               # noqa: BLE001 -- not exposed (fake)
            pass
    return found


def part_name(item):
    """The part's internal name (``landingLeg1-2``), the cache's key."""
    try:
        return item.part.name
    except Exception:                   # noqa: BLE001 -- KSP-side failure
        return None


def _name_counts(names):
    counts = {}
    for n in names:
        counts[n] = counts.get(n, 0) + 1
    return ", ".join("%d %s" % (c, n) for n, c in sorted(counts.items(),
                                                          key=str))


def deploy_state(part):
    """'deployed', 'retracted', 'deploying', 'retracting' or 'broken'."""
    try:
        return str(part.state).split(".")[-1].lower()
    except Exception:                   # noqa: BLE001 -- KSP-side failure
        return None


def prepare_vehicle(vessel, cfg):
    """Deploy every airbrake and light every liquid engine, at START.

    This replaces firing an action group, which only worked on a craft whose
    builder had put the right parts on AG2.  Airbrakes are found by their
    part module, set to their range's floor (the late law opens them) and deployed; their
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
            if partmodule.has_field(module, "Deploy Angle"):
                partmodule.set_field_float(module, "Deploy Angle", 0.0)
                angles.append(partmodule.get_field(module, "Deploy Angle"))
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
    lines.append("airbrakes deployed: %d, deploy angle %s (asked 0)"
                 % (brakes, "/".join(sorted(set(angles))) or "n/a"))
    if cfg.FIN_BRAKES:
        lines.append(deploy_brake_fins(vessel, cfg))
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
    if cfg.GIMBAL_ROLL_OFF and not cfg.GIMBAL_ROLL_UNTIL_BURN:
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
    done, refused, why = 0, 0, None
    for engine in vessel.parts.engines:
        try:
            for module in engine.part.modules:
                if module.name == "ModuleGimbal":
                    partmodule.set_field_bool_by_id(module, "enableRoll",
                                                    enabled)
                    done += 1
        except Exception as exc:        # noqa: BLE001 -- KSP-side failure
            refused += 1
            why = why or "%s: %s" % (type(exc).__name__, exc)
    # Say why: from the move to kRPC 0.7 until LOG15102 every booster flew
    # with roll on its gimbals, logged as "0 engines (n refused)" and an
    # AttributeError nobody saw.
    return ("gimbal roll %s on %d engines%s"
            % ("enabled" if enabled else "disabled", done,
               " (%d refused, %s)" % (refused, why) if refused else ""))


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


def flip_done_deg(cfg):
    """The flip is over when thrust along the nose loses no more than
    ``FLIP_THRUST_LOSS`` of itself to the angle off the aim."""
    return math.degrees(math.acos(1.0 - cfg.FLIP_THRUST_LOSS))


class ValveLimits:
    """The RCS valve's relay thresholds, measured rather than fitted; every
    other ``RCS_*`` field reads through to the config.

    - off inside ``NOISE_SIGMAS`` times what a settled nose is off its
      command: the loop's hunt -- the rms scatter of the nose about its
      delayed command (``common.attitudelag``) -- plus the trail of a nose a
      lag behind a moving command (command rate x lag), plus the pointing
      error's own tick scatter (``common.jitter``): closer than that is not a
      turn (a fin booster holds ~1 deg);
    - on past ``NOISE_SIGMAS`` times that (the relay's gap, so it cannot
      switch at the hunt's own frequency);
    - settled after ``SETTLE_LAGS`` measured nose lags inside; an input
      pinned for one lag is a saturation.

    Until the hunt and the lag are measured, both angles are 0: the valve is
    open whenever it is permitted -- an unknown is all the authority there
    is, not none."""

    def __init__(self, cfg):
        self._cfg = cfg
        self.measured = False
        self.RCS_ERROR_OFF_DEG = 0.0
        self.RCS_ERROR_ON_DEG = 0.0
        self.RCS_SETTLE_S = 0.0
        self.RCS_SAT_ON_S = 0.0

    def __getattr__(self, name):
        return getattr(self._cfg, name)

    def measure(self, jitter, error_deg, lag, hunt=None, cmd_rate=None):
        if error_deg is not None and error_deg >= 0.0:
            jitter.update((error_deg, 0.0, 0.0))
        sd = jitter.sd
        if sd is None or not lag or hunt is None:
            return
        k = self._cfg.NOISE_SIGMAS
        self.measured = True
        # A nose a lag behind a command moving at cw is cw * lag off it.
        trail = (cmd_rate or 0.0) * lag
        self.RCS_ERROR_OFF_DEG = k * (math.degrees(hunt) + trail + sd)
        self.RCS_ERROR_ON_DEG = k * self.RCS_ERROR_OFF_DEG
        self.RCS_SETTLE_S = self._cfg.SETTLE_LAGS * lag
        self.RCS_SAT_ON_S = lag


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
        self.idle_gimbals = None
        if getattr(cfg, "IDLE_GIMBAL_LOCK", False):
            self.idle_gimbals = IdleGimbals(
                self.vessel, log=lambda text: self.log.event(
                    self.conn.space_center.ut, text))
        self.fins = None
        if getattr(cfg, "BOOSTBACK_FINS_HELD", False):
            self.fins = VacuumSurfaces(
                self.vessel, 0.0, 0.0, log=lambda text: self.log.event(
                    self.conn.space_center.ut, text),
                keep_deploy=cfg.FINS_HELD_KEEP_DEPLOY)
        site = launch_site(conn, self.vessel)
        if site is None:
            logbook.event(ut, "launch site unknown; pad from PAD_LAT/PAD_LON")
        else:
            dlat = math.radians(site["lat"] - cfg.PAD_LAT)
            dlon = math.radians(site["lon"] - cfg.PAD_LON)
            off = self.body.equatorial_radius * math.hypot(
                dlat, dlon * math.cos(math.radians(cfg.PAD_LAT)))
            use = site["body"] == self.body.name
            logbook.event(ut, "launch site (%s %s on %s): %.6f %.6f, %.1f m "
                          "from PAD_LAT/PAD_LON%s"
                          % (site["source"], site["site"] or "-", site["body"],
                             site["lat"], site["lon"], off,
                             " -- targeted" if use else ""))
            if use:
                cfg = self.cfg = replace(cfg, PAD_LAT=site["lat"],
                                         PAD_LON=site["lon"])
        self.env = Environment(conn, self.vessel, self.body, cfg, logbook, ut)
        # Before the flight: one call per part is a stall no phase can afford.
        self.touchdown_limit = touchdown_limit(self.vessel)
        if self.touchdown_limit is None:
            logbook.event(ut, "touchdown limit unreadable")
        else:
            logbook.event(ut, "touchdown limit %.1f m/s (%s, weakest of %d "
                          "that touch)" % self.touchdown_limit)
            self.env.touchdown_tolerance = self.touchdown_limit[0]
        # The legs' cfg constants, for read_landing_limits (a few hundred
        # batched calls: here, not in the descent).
        self.leg_constants = read_leg_constants(conn, self.vessel)
        logbook.event(ut, "touchdown target %.1f m/s"
                      % trajectory.touchdown_speed(self.env, cfg))
        self.telemetry = Telemetry(conn, self.vessel, self.frame, cfg,
                                   logbook)
        self.proximity = ProximityScan(conn, self.vessel, self.frame, cfg,
                                       logbook)
        self.gui = ControlPanel(conn, "BOOSTERLAND", "START AUTOLAND",
                                ACCENT_BOOSTER)

        self.state = STANDBY
        self.autostart = False   # --autostart: STANDBY acts as if START was pressed
        self.phase_start_ut = ut
        self.prediction = None
        self.prediction_ut = None       # when it was taken
        self.roll_up = None      # last roll reference, for continuity
        self.roll_nose = None    # the commanded nose roll_up was taken for
        self.roll_hold = None    # rad: the roof's angle from roll_up at the
                                 # first aim, held for the whole flight
        self.roll_cmd = None     # the up vector actually commanded
        self.rcs_limits = ValveLimits(cfg)
        self.rcs = rcs_valve.Valve(self.rcs_limits, logbook)
        self.err_jitter = Jitter()      # the pointing error's tick scatter
        self.velocity_jitter = Jitter()  # the horizontal velocity's
        self.cmd_rates = []             # recent command rates (deg/s)
        self.rcs_permitted = False      # STANDBY gets none; START opens it
        self.commanded_direction = None  # what ``aim`` last asked for
        self.tune_logged = 0            # log_attitude_tune's progress
        self.rcs_held = False           # LANDING_RCS_HELD has opened it
        self.nose_block_logged = False  # nose_blocked has said so
        self.autopilot_engaged = False
        self.autopilot_retries = 0
        self.extra_lines = ["press START AUTOLAND"]
        self.gear_deployed = False
        self.touchdown_record = None    # the 2 s sign-off, held for the watch
        self.touchdown_tilt = 0.0       # most the nose leaned off vertical
        self.landing_limits = None      # read once, legs out (False = tried)
        self.gear_down_ut = None
        self.gear_timing = None         # GEAR_TIMED: None, deploying,
                                        # retracting, done or failed
        self.gear_items = []
        self.gear_t0 = None
        self.gear_deploy_s = None       # measured, game seconds
        self.gear_names = []
        self.gear_done = []             # per part: seconds, or False if broken
        self.gear_t1 = None
        self.late_state = None          # see run_late_deploy
        self.late_ut = None
        self.late_next_ut = 0.0
        self.late_resweep_ut = None
        self.steer = None               # the angle of attack the coast flies
        self.steer_history = []         # (ut, commanded aoa deg), for tracking
        self.deploy_angle = None        # DEPLOY_LAW's command, deg
        self.brake_probe = None         # Cd*A probed last tick (brakes_settled)
        self.deploy_none = False        # no airbrake answered: law off
        self.boostback_under = 0        # consecutive ticks inside the tolerance
        self.boostback_under_since = None   # ... and when that run started
        self.boostback_heading = None   # BOOSTBACK_HEADING_HOLD's aim
        self.latch_checked = False      # KRPC_CLEAR_LATCH_ON_THRUST
        self.flip_gimbal = None         # FLIP_THROTTLE_LAW: rad/s^2 per unit
        self.flip_fixed = 0.0           # throttle; RCS + wheels, rad/s^2
        self.flip_done = False
        self.flip_prev = None           # (ut, align deg) last tick
        self.flip_start = None          # (ut, mass) at the first flip tick
        self.flip_u = []                # throttles the law chose
        self.coast_settled = False      # the coast's flip has come round
        self.trim_active = False        # COAST_RCS_TRIM: the flip has settled
        self.trim_pushing = False       # ... and it is translating now
        self.trim_sign = 1.0            # ... -1 once the check flipped it
        self.trim_flips = 0
        self.trim_confirmed = False     # a push has closed the miss: the
                                        # sign is proven, stop checking it
        self.trim_off = False           # ... the check failed twice
        self.trim_run = None            # (ut, |cross|, side) of this push
        self.trim_cmd = (0.0, 0.0, 0.0)  # right, forward, up last sent
        self.commanded_throttle = None  # what set_throttle last asked for
        self.demand = 0.0               # fraction of cluster thrust wanted
        self.last_ut = None             # the last tick's game time
        self.tick_intervals = []        # recent ticks' game-s (median kept)
        self.spool_from = None          # (ut, demand) of an ignition being
                                        # timed (track_spool)
        self.cluster = None             # EngineCluster, built at START
        self.roll_order = 1             # roll symmetry (ROLL_SYMMETRY)
        self.roll_slot = None           # which of its equivalent roofs
        self.diag_restarted = False     # KRPC_AP_DIAG_BELOW_M
        self.prev_nose = None           # (ut, direction) of the last tick
        self.nose_rate = None           # deg/s, see track_nose_rate
        self.prev_cmd = None            # (ut, commanded direction)
        self.burn_cmd = None            # last landing-burn command, slewed
        self.burn_cmd_ut = 0.0
        self.burn_plan_ut = None        # DIAG_BURN: when the plan was logged
        self.drag_diag_next = 0.0       # DIAG_DRAG: next probe
        self.neutral_from = None        # sweep_burn_curve: fins held since
        self.neutral_last = None        # ... the last probe while held
        self.neutral_next_ut = 0.0      # ... the next sweep, not before
        self.neutral_fell = False       # ... its drag has started falling
        self.ladder_machs = [2.0, 1.2, 0.6]     # DIAG_DRAG: burn ladders
        self.drag_key = None            # dragcache's key for this booster
        self.miss_jitter = Jitter()     # the predicted touchdown's tick-to-
                                        # tick scatter: every deadband
        self.rcs_accel = None           # m/s^2 the RCS translates at (read
                                        # while the valve is open)
        self.lag = LagFit()             # the attitude loop's measured
                                        # closing rate (Autoland.null_rate)
        self.coast_lag = LagFit()       # the same, unpowered in the coast
                                        # (fins, RCS, the air): another plant
        self.lag_logged = None          # the rate last logged
        self.cmd_rate = None            # deg/s the command itself moves
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
        # Each phase predicts under its own law: its own scatter.
        self.miss_jitter.reset()
        self.env.miss_band = None
        self.telemetry.remeasure_leg_clearance(ut, state.lower())
        if state in (COAST, LANDING_BURN):
            self.read_dry_mass(ut, state)
        if state == COAST and self.drag_key is None:
            # The coast burns nothing and waits: the part reads are free here.
            self.drag_key = dragcache.booster_key(
                lambda: self.vessel.parts.all) or ""
        if (self.cfg.KRPC_AP_DIAG and state == COAST
                and self.cfg.KRPC_AP_DIAG_BELOW_M <= 0.0):
            # The boostback's record first (LOG<n>.bb.ap.csv): at shutdown
            # a destroyed vessel takes it with it.
            self.save_ap_diag(".bb.ap.csv")
            # kRPC keeps 3000 physics ticks (60 s) from when logging was
            # last turned on: restart it here, so the record is the coast's.
            try:
                self.autopilot.diagnostic_logging = False
                self.autopilot.diagnostic_logging = True
            except Exception:                       # noqa: BLE001
                pass
        if self.fins is not None:
            if state in (SEPARATION, BOOSTBACK):
                self.fins.hold(state.lower())
            else:
                self.fins.release(state.lower())
        if (state == LANDING_BURN and self.cfg.GIMBAL_ROLL_OFF
                and self.cfg.GIMBAL_ROLL_UNTIL_BURN):
            # Boostback rolls cleanly on the gimbals at full thrust; the
            # burn is where kRPC's uncredited gimbal roll spun the heavy.
            self.log.event(ut, set_gimbal_roll(self.vessel, False))
        # A steering command carried through another phase is not a steady
        # one: the first COAST tick after a tilted burn would read the tilt
        # as "held".
        self.steer_history = []

    def predict(self, snap, powered, steer=None):
        max_accel = snap.max_accel if powered else 0.0
        if snap.leg_clearance:
            self.env.leg_clearance = snap.leg_clearance
        self.env.attitude_rate = self.null_rate(snap)
        self.update_burn_env(snap)
        self.prediction_ut = snap.ut
        self.prediction = trajectory.predict_landing(
            self.env, snap.position, snap.velocity, snap.mass, max_accel,
            self.cfg, steer=steer)
        # Hand the descent back to the estimator: the next aerodynamic probe
        # wants to know where the booster will be when it is doing each speed,
        # so that a Mach bin is measured in the air it is going to be used in.
        # See Environment.probe_altitude.
        self.env.set_descent_profile(self.prediction.profile)
        return self.prediction

    def read_dry_mass(self, ut, why):
        """The mass the burn's engines leave when their propellant is gone
        (``env.burn_dry_mass``, kg): the vessel's mass less every resource
        its engines burn.  The fin booster ran its tanks dry at 80-310 m on
        6 flights of 6 (bp1-1116) while the propagation flew its burn to the
        ground.  Read on entering the coast and the burn (a handful of kRPC
        calls; the coast burns nothing the engines use)."""
        try:
            names = set()
            for engine in self.vessel.parts.engines:
                names.update(engine.propellant_names)
            res = self.vessel.resources
            prop = sum(res.amount(n) * res.density(n) for n in names)
            mass = self.vessel.mass
        except Exception as exc:                            # noqa: BLE001
            self.log.event(ut, "propellant unreadable (%s): %r" % (why, exc))
            return
        self.env.burn_dry_mass = mass - prop
        self.log.event(ut, "propellant %.2f t of %s, dry %.2f t (%s)"
                       % (prop / 1000.0, "/".join(sorted(names)),
                          (mass - prop) / 1000.0, why.lower()))

    def update_burn_env(self, snap):
        """What the burn's propagation reads off the vehicle as it is now:
        its Isp, whether the burn's engines already deliver, the nose's
        measured lag and where the nose points."""
        if snap.isp and snap.isp > 0.0:
            self.env.burn_isp = snap.isp
        self.env.burn_lit = bool(self.state == LANDING_BURN and snap.thrust)
        self.env.nose_lag = self.lag.lag
        self.env.nose = snap.direction

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
            if self.cfg.FLIP_VIA_VERTICAL:
                # Big turns are two pitches through the zenith, never a dive
                # through the airstream; releases itself as the nose crosses.
                direction = guidance.flip_through_vertical(
                    snap.direction, direction, snap.position,
                    self.cfg.FLIP_VIA_VERTICAL_DEG,
                    self.cfg.FLIP_VERTICAL_PAST_DEG)
            if self.cfg.ROLL_ALIGN:
                previous, previous_nose = self.roll_up, self.roll_nose
                if previous is None and self.cfg.ROLL_SEED_ACTUAL:
                    # Seed on the side of the plane nearest the roof the
                    # vehicle actually has: never more than 90 deg of roll.
                    previous = vessel_roof(snap)
                    previous_nose = snap.direction
                up = guidance.roll_reference(snap.position, snap.velocity,
                                             direction, previous,
                                             self.velocity_band(snap),
                                             previous_nose
                                             if self.cfg.ROLL_TRANSPORT
                                             or previous is not self.roll_up
                                             else None)
                self.roll_up = up
                self.roll_nose = direction
                if up is not None and self.roll_hold is None:
                    # Hold the roll the vehicle starts with, not the plane's:
                    # the roof stays at its starting angle from the
                    # trajectory plane rather than being rolled into it.
                    self.roll_hold = roof_angle(snap, direction, up)
                    self.log.event(snap.ut, "roll held at %.1f deg from the "
                                   "trajectory plane (the starting roll)"
                                   % math.degrees(self.roll_hold or 0.0))
                if up is not None and self.roll_hold:
                    up = rotate_about(up, direction, self.roll_hold)
                late = self.state in (COAST, LANDING_BURN, TOUCHDOWN)
                if up is not None and late and self.roll_order >= 3:
                    # Three or more equal surfaces round the axis lift and
                    # brake the same at any roll, and the steering is the
                    # nose's: the roll angle is worth nothing, only its
                    # rate.  kRPC with no target roll drives the rate to
                    # zero (the inner loop alone).  Held to an angle, its
                    # attitude loop rang at ~100 deg/s on kRPC 0.7, which
                    # credited the fins with 10-30x their roll
                    # (LOG16195-16198).
                    up = None
                if (up is not None and late and self.roll_order > 1):
                    # The vehicle looks the same every 360/n deg of roll:
                    # hold the equivalent roof nearest its own, so the roll
                    # error is never more than 180/n and nothing winds up
                    # through the air the fins cannot use (LOG16194: input
                    # pinned 8 s at 32 km, error through 180 deg).
                    slot = 2.0 * math.pi / self.roll_order
                    self.roll_slot = nearest_slot(
                        roof_angle(snap, direction, up), slot, self.roll_slot,
                        self.cfg.ROLL_SLOT_HYSTERESIS)
                    up = rotate_about(up, direction, self.roll_slot * slot)
        self.roll_cmd = up
        set_autopilot_attitude(self.autopilot, direction, up,
                               self.cfg.ROLL_OFFSET_DEG)
        # The *routed* direction, not the one asked for: while the flip is
        # going over the top the vehicle is meant to be pointed at the
        # waypoint, and measuring the error against the final aim would call
        # a turn that is going exactly to plan a 100-degree failure.
        self.commanded_direction = tuple(direction)
        if snap is not None:
            prev = self.prev_cmd
            self.prev_cmd = (snap.ut, self.commanded_direction)
            if prev is not None and snap.ut - prev[0] > 1e-3:
                self.cmd_rate = (vec.angle_between(prev[1], direction)
                                 / (snap.ut - prev[0]))

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
        if (self.cfg.COAST_RCS_TRIM and self.trim_pushing
                and not self.trim_off and self.state == COAST
                and self.rcs_permitted):
            # Translating: the valve is the actuator, and relaying it would
            # swing kRPC's auto-tuned gains as it flips (see above).
            if not self.rcs_held:
                self.rcs_held = True
                self._apply_rcs(True)
                self.log.event(snap.ut, "rcs held open for the trim")
            return
        if self.rcs_held:
            # Out of the burn: shut it and give the valve back to the relay,
            # which never saw it open and so would never close it.
            self.rcs_held = False
            self._apply_rcs(False)
            self.log.event(snap.ut, "rcs released after the %s"
                           % ("trim" if self.state == COAST
                              else "landing burn"))
        self.rcs_limits.measure(self.err_jitter, self.pointing_error(snap),
                                self.lag.lag, self.lag.hunt,
                                self.steady_cmd_rate())
        if self.rcs_limits.measured:
            self.env.pointing_band = self.rcs_limits.RCS_ERROR_OFF_DEG
        self.rcs.update(snap.ut, self.rcs_permitted,
                        self.pointing_error(snap), None,
                        apply=self._apply_rcs,
                        saturation=(snap.input_saturation
                                    if self.state == COAST
                                    and (self.coast_settled
                                         or not self.cfg.COAST_FLIP_RELAY)
                                    else None))

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
        if (throttle > 0.0 and self.demand <= 0.0
                and getattr(self, "last_ut", None) is not None
                and self.env.engine_spool_s is None):
            self.spool_from = (self.last_ut, throttle)
        elif throttle <= 0.0 and hasattr(self, "spool_from"):
            self.spool_from = None
        self.demand = throttle
        cluster = self.cluster
        if cluster is not None and len(cluster.groups) > 1:
            # Center engine first, then pairs, each to full: see
            # EngineCluster.  The main throttle only gates the cluster.
            cluster.set(throttle)
            throttle = 1.0 if throttle > 0.0 else 0.0
        self.commanded_throttle = throttle
        self.control.throttle = throttle
        # Nozzles still while nothing burns (``IDLE_GIMBAL_LOCK``).
        idle = getattr(self, "idle_gimbals", None)
        if idle is not None:
            idle.update(throttle > 0.0)

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

    def configure_autopilot(self, ut):
        """krpcLab's settings on kRPC's autopilot: at START, and again after
        ``clear_krpc_latches`` has reset it to kRPC's defaults."""
        self.autopilot.reference_frame = self.frame
        try:
            self.autopilot.limit_angular_velocity = bool(
                self.cfg.KRPC_LIMIT_ANGULAR_VELOCITY)
        except Exception as exc:    # noqa: BLE001 -- kRPC before 0.8
            self.log.event(ut, "kRPC limit_angular_velocity not set (%r)"
                           % (exc,))
        try:
            # kRPC controls roll only while the pointing error is under
            # this; above it the roll input is zero and roll drifts.
            # kRPC 0.7 replaced ``roll_threshold`` by a blend: none
            # above ``roll_start_angle``, full below
            # ``roll_engage_angle`` (start set first, so the pair is
            # never inverted mid-change).
            limit = float(self.cfg.ROLL_THRESHOLD_DEG)
            start = min(180.0, limit + 5.0)
            self.autopilot.roll_start_angle = start
            self.autopilot.roll_engage_angle = min(limit, start - 1.0)
        except Exception as exc:    # noqa: BLE001
            self.log.event(ut, "could not set kRPC's roll gate "
                           "(%r): roll drops above 20 deg of pointing "
                           "error" % exc)

    def clear_krpc_latches(self, snap):
        """KRPC_CLEAR_LATCH_ON_THRUST: when the engines light after the
        flip, clear any axis kRPC latched as structurally flexible.

        Through the unpowered flip an axis can have no authority at all --
        the fin booster's RCS cannot roll it (credited 0, measured ~0) -- and
        auto-tune's gain is the bandwidth over that authority: roll kp
        19,000-244,000.  The output bangs, the integrator winds to the
        stop, and kRPC's wobble detector latches the axis and floors its
        bandwidth.  Lit, the gimbals give 5.4-6 rad/s^2 of roll, and the
        latched loop ran at kp 0.19 against ~1.6 with the integrator still
        at -0.7: 3-4 rad/s of roll took ~5 s to stop (LOG16502 .bb.ap).
        Nothing flexed, so the latch is not a finding: ``AutoPilot.reset``
        clears it with the integrators, and krpcLab's settings go back on.
        A real structural mode latches again under thrust."""
        ap = self.autopilot
        try:
            latched = (bool(ap.roll_oscillation_latched),
                       bool(ap.pitch_yaw_oscillation_latched))
        except Exception as exc:                    # noqa: BLE001
            self.log.event(snap.ut, "kRPC latch unread: %r" % (exc,))
            return
        if not any(latched):
            self.log.event(snap.ut, "kRPC latches at thrust: none")
            return
        if self.cfg.KRPC_AP_DIAG:
            self.save_ap_diag(".flip.ap.csv")
        try:
            ap.reset()
            self.configure_autopilot(snap.ut)
            if self.cfg.KRPC_AP_DIAG:
                ap.diagnostic_logging = True
            error = set_autopilot_engaged(ap, True)
        except Exception as exc:                    # noqa: BLE001
            error = exc
        self.log.event(snap.ut, "kRPC latches at thrust: roll=%s pitch/yaw=%s"
                       " -> reset%s" % (latched[0], latched[1],
                                        "" if error is None
                                        else " FAILED: %r" % (error,)))

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
            if self.cfg.KRPC_AP_DIAG:
                try:
                    self.autopilot.diagnostic_logging = True
                except Exception as exc:            # noqa: BLE001
                    self.log.event(ut, "kRPC diagnostic log refused: %r"
                                   % exc)
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
            if self.cfg.ROLL_SYMMETRY:
                try:
                    self.roll_order = roll_symmetry(self.vessel)
                except Exception as exc:            # noqa: BLE001
                    self.log.event(snap.ut, "roll symmetry unread: %r" % exc)
                self.log.event(snap.ut, "roll symmetry: %d-fold (from the "
                               "coast: %s)" % (self.roll_order,
                                               "roll rate damped, angle free"
                                               if self.roll_order >= 3 else
                                               "the nearest of %d roofs held"
                                               % self.roll_order
                                               if self.roll_order > 1
                                               else "roll held"))
            if self.cfg.ENGINE_ALLOCATION:
                self.cluster = EngineCluster(self.vessel, self.cfg)
                self.log.event(snap.ut, self.cluster.describe())
            self.control.sas = False
            self.configure_autopilot(snap.ut)
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
        only be flown on RCS and reaction wheels.  It turns only while the
        clearance scan holds the phase.  An unpowered flip at separation speed
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
            clear = blocker is None
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
        if blocker is not None:
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
        aim, throttle = solution.aim, solution.throttle
        distance = self.miss_distance()
        long, cross = self.miss_components(snap)
        remaining = None
        if self.cfg.BOOSTBACK_HEADING_HOLD:
            if (self.boostback_heading is None
                    and solution.throttle < 1.0
                    and vec.angle_between(snap.direction, solution.aim)
                    <= flip_done_deg(self.cfg)):
                self.boostback_heading = solution.aim
                self.log.event(snap.ut, "boostback heading held: miss=%.0fm "
                               "long=%+.0f cross=%+.0f dv=%.1f"
                               % (distance, long, cross, solution.dv))
            if self.boostback_heading is not None:
                aim = self.boostback_heading
                throttle, remaining = guidance.held_boostback(
                    self.env, snap.position, prediction,
                    self.boostback_heading, self.cfg, snap.max_accel)
        self.aim(aim, snap)

        align = vec.angle_between(snap.direction, aim)
        elapsed = snap.ut - self.phase_start_ut
        if self.cfg.FLIP_THROTTLE_LAW and not self.flip_done:
            throttle = min(throttle, self.flip_law(snap, align, prediction))

        # Debounce the exit.  The predicted miss is noisy by a couple of
        # hundred metres near the end of boostback -- it is a fresh
        # propagation every tick off a drag estimate that is still settling --
        # and a bare threshold turns that noise into *when the burn stops*.
        # LOG29 and LOG30 flew the same config from the same start state and
        # ended boostback 3 s apart at the same miss (186 and 184); those 3 s
        # of burn are 370 m at the pad, and the two landed 4 m and 362 m out.
        # The phase was not imprecise, it was bimodal.  Requiring the miss to
        # stay inside the tolerance for a measured nose lag makes a
        # single dip unable to end the burn.
        #
        # The test is on the *signed* longitudinal miss, and that half is not
        # cosmetic.  Every actuator after this phase can only make the booster
        # land *shorter*: the landing burn thrusts
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
        # On a held heading the cross-track is left for the coast to trim,
        # so it is the miss *along* the heading that has to be inside the
        # tolerance -- or the burn could never end with a cross-track miss
        # bigger than the tolerance.
        # The tolerance is the prediction's own noise: past the pad by no
        # more than the phases after it would read as a miss at all.  A
        # 190 m tolerance left the fin boosters 150-170 m short at coast
        # entry with nothing after it able to lengthen (nose-1115).
        band = self.env.miss_band
        closed = band is not None and (remaining <= band
                                       if remaining is not None
                                       else distance <= band)
        under = closed
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
        held = (snap.ut - self.boostback_under_since
                if self.boostback_under_since is not None else 0.0)
        if (self.boostback_under >= 2
                and held >= (self.lag.lag or 0.0)):
            self.set_throttle(0.0)
            # `held` and the tick count go in the event because the debounce
            # is the one thing here that is paced by the *host* rather than
            # the vehicle: a tick is wall-clock work, so four of them are a
            # different amount of burn on a loaded machine.  Solo and on four
            # instances the same save exited 2.7 s apart -- 150 m at the pad
            # -- and nothing in the log said so.  Now it does.
            self.enter(COAST, snap.ut,
                       "miss=%.0fm long=%+.0f remaining=%s band=%.1f "
                       "held=%.2fs ticks=%d"
                       % (distance, long,
                          "%+.0f" % remaining if remaining is not None
                          else "-", band, held, self.boostback_under))
        elif elapsed > self.cfg.BOOSTBACK_MAX_BURN_S:
            self.set_throttle(0.0)
            self.enter(COAST, snap.ut, "TIMEOUT miss=%.0fm" % distance)
        elif not self.autopilot_engaged:
            # Nothing is steering: a full-throttle burn along whatever
            # attitude the separation left would only deepen the miss.
            # FLIP_UNDER_POWER lights the engines without waiting for
            # alignment, so this is the check that replaces that wait.
            self.set_throttle(0.0)
        elif align > flip_done_deg(self.cfg) and not self.cfg.FLIP_UNDER_POWER:
            # Still flipping: hold thrust so the flip does not add error.
            self.set_throttle(0.0)
        elif (align > flip_done_deg(self.cfg) and self.cfg.BOOSTBACK_CLEAR_NOSE
              and self.nose_blocked(snap)):
            # Lit mid-flip, the thrust is along the nose, not the aim -- and
            # at the start of the flip the nose points at the stage it just
            # dropped.  SEPARATION's scan cleared the *aim*; three qs_hot
            # boosters then drove into the upper stage at 40 m/s and lost
            # their tanks (LOG10720, LOG10774, LOG11177; KSP.log: airbrake
            # into the ServiceBay, core into the verniers).
            self.set_throttle(0.0)
        else:
            self.set_throttle(throttle)

        self.extra_lines = [
            "miss    : %8.0f m" % distance,
            "long    : %8.0f m" % long,
            "dv left : %8.1f m/s" % solution.dv,
            "burn    : %8.1f s" % solution.burn_time,
            "align   : %8.1f deg" % align,
            "ttl     : %8.0f s" % prediction.time_to_land,
        ]

    def flip_authority(self):
        """Angular acceleration the flip can count on, rad/s^2: the gimbals'
        per unit throttle (geometry: each engine's thrust times the sine of
        its gimbal travel times its lever about the centre of mass, over the
        weaker tilt axis' moment of inertia), and what RCS and wheels give
        without thrust.  RCS reads 0 while the valve is shut, so the most it
        has read is kept."""
        moi = self.vessel.moment_of_inertia
        inertia = max(1.0, min(moi[0], moi[2]))
        if self.flip_gimbal is None:
            frame = self.vessel.reference_frame
            torque = 0.0
            for engine in self.vessel.parts.engines:
                try:
                    if ("LiquidFuel" not in engine.propellant_names
                            or not engine.gimballed):
                        continue
                    travel = math.radians(engine.gimbal_range
                                          * engine.gimbal_limit)
                    lever = abs(engine.part.position(frame)[1])
                    thrust = engine.max_vacuum_thrust or engine.max_thrust
                    torque += thrust * math.sin(travel) * lever
                except Exception:       # noqa: BLE001 -- KSP-side failure
                    continue
            self.flip_gimbal = torque / inertia
        rcs = self.env._control_torque("available_rcs_torque") or 0.0
        wheels = self.env._control_torque(
            "available_reaction_wheel_torque") or 0.0
        self.flip_rcs = max(getattr(self, "flip_rcs", 0.0), rcs)
        self.flip_fixed = (self.flip_rcs + wheels) / inertia
        self.flip_inertia = inertia
        return self.flip_gimbal, self.flip_fixed

    def flip_law(self, snap, align, prediction):
        """FLIP_THROTTLE_LAW: the throttle for this tick of the flip, or 1
        once the nose is within flip_done_deg (the flip is over, and the
        boostback's own throttle rules)."""
        if align <= flip_done_deg(self.cfg):
            self.flip_done = True
            if self.flip_start is not None:
                ut0, m0 = self.flip_start
                self.log.event(snap.ut, "flip done: %.1f s, %.2f t burned, "
                               "throttle mean %.2f (law)"
                               % (snap.ut - ut0, (m0 - snap.mass) / 1e3,
                                  sum(self.flip_u) / max(1, len(self.flip_u))))
            return 1.0
        gimbal, fixed = self.flip_authority()
        omega = 0.0
        if self.flip_prev is not None and snap.ut > self.flip_prev[0]:
            omega = math.radians(self.flip_prev[1] - align) / (
                snap.ut - self.flip_prev[0])
        self.flip_prev = (snap.ut, align)
        up = vec.unit(snap.position)
        to_pad = vec.project_out(vec.sub(self.env.pad, snap.position), up)
        x = vec.norm(to_pad)
        away = vec.scale(vec.unit(to_pad), -1.0) if x > 1.0 else up
        v_away = vec.dot(vec.project_out(snap.velocity, up), away)
        drift = guidance.drift_rate(x, v_away, prediction.time_to_land
                                    if prediction is not None else 100.0)
        u, t, cost = guidance.flip_throttle(math.radians(align), omega,
                                            gimbal, fixed, snap.max_accel,
                                            drift)
        if self.flip_start is None:
            self.flip_start = (snap.ut, snap.mass)
            self.log.event(snap.ut, "flip law: align %.0f deg, gimbal %.3f "
                           "rad/s2 per throttle, rcs+wheels %.3f, inertia "
                           "%.0f t m2, thrust %.1f m/s2, drift %.1f m/s per "
                           "s -> u %.2f, turn %.1f s, cost %.0f m/s"
                           % (align, gimbal, fixed, self.flip_inertia / 1e3,
                              snap.max_accel, drift, u, t, cost))
        self.flip_u.append(u)
        return u

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
        # is a wing, and a wing costs no propellant, and no relight -- see guidance.solve_steer.
        # The angle is *solved* against the propagator and then handed back to
        # it, so the stored prediction is of the trajectory the booster is
        # about to fly rather than of a ballistic one it is not.
        self.set_unwind(snap)
        steer, steered = guidance.solve_steer(
            self.env, snap.position, snap.velocity, snap.mass, snap.max_accel,
            self.cfg)
        if steer is not None:
            self.env.descent_aoa = abs(steer.aoa)
            self.prediction = steered
            self.prediction_ut = snap.ut
            self.env.set_descent_profile(steered.profile)
            self.steer = steer
            prediction = steered
            self.aim(guidance.steer_attitude(
                snap.position, snap.velocity,
                self.unwound(steer, steered)), snap)
        else:
            self.steer = None
            prediction = self.predict(snap, powered=True)
            self.aim(guidance.coast_attitude(snap.velocity), snap)

        distance = self.miss_distance()
        height, needed, _ = trajectory.landing_burn_state(
            self.env, snap.position, snap.velocity, snap.max_accel, self.cfg,
            height=snap.landing_height)
        self.extra_lines = [
            "miss    : %8.0f m" % distance,
            "burn alt: %8.0f m" % needed,
            "margin  : %8.0f m" % (height - needed),
            "aoa     : %8.1f deg" % (math.degrees(self.steer.aoa)
                                     if self.steer else 0.0),
            "ttl     : %8.0f s" % prediction.time_to_land,
        ]
        if self.cfg.DEPLOY_LAW:
            self.run_late_deploy(snap)
            self.extra_lines.append("brakes  : %8.0f deg" % (
                self.deploy_angle if self.deploy_angle is not None else 0.0))
        if (not self.coast_settled and self.rcs_limits.measured
                and 0.0 <= self.pointing_error(snap)
                <= self.rcs_limits.RCS_ERROR_ON_DEG):
            self.coast_settled = True
            self.log.event(snap.ut, "coast settled: err %.1f deg after %.1fs"
                           % (self.pointing_error(snap),
                              snap.ut - self.phase_start_ut))
        if self.cfg.COAST_RCS_TRIM:
            self.run_rcs_trim(snap)
        if snap.vertical_speed < 0.0 and height <= needed:
            # Light the engines on this tick, before the burn's one-off work
            # (dry mass, gimbal roll, gear, RCS, DIAG_BURN's plans): the
            # propagation lights them ``reaction_time`` after the trigger,
            # and the 0.3-0.4 s that work took at full speed was flown
            # unbraked -- the burn landed ~70 m long of its own prediction
            # (noaim-1117).
            self.set_throttle(guidance.landing_throttle(
                self.env, snap.position, snap.velocity, snap.max_accel,
                self.cfg, height=snap.landing_height).throttle)
            if self.cfg.DIAG_BURN:
                self.log_burn_plans(snap)
            self.enter(LANDING_BURN, snap.ut,
                       "h=%.0f needed=%.0f miss=%.0f"
                       % (height, needed, distance))

    def run_rcs_trim(self, snap):
        """Translate on the RCS thrusters to walk the cross-track miss to 0.

        On the heavy nothing else moves it: the wing reads no lift worth
        steering on, and the landing burn is held upright.  So the cross-track the
        boostback ended with -- and whatever the flip's thrusters added --
        was the touchdown miss.  A sideways dv moves the touchdown by dv
        times the time left, so at coast entry 100 m is ~0.7 m/s: a second
        or two of the Vernors, no turn and no relight.

        Starts once the flip has settled (its thrusters are firing for
        rotation and the error is not yet a hold), holds the valve open from
        then on (see ``update_rcs``), and checks the sign of its own effect:
        the translation axes are mapped from the vessel frame, and a wrong
        sign would push the miss the other way for the whole coast.
        """
        if self.trim_off or not self.rcs_permitted or self.prediction is None:
            self.trim_pushing = False
            self.send_translation((0.0, 0.0, 0.0))
            return
        if not self.trim_active:
            if not self.coast_settled:
                return
            self.trim_active = True
            long, cross = self.miss_components(snap)
            self.log.event(snap.ut, "rcs trim: armed at cross=%+.0f long=%+.0f"
                           % (cross, long))
        long, cross = self.miss_components(snap)
        band = self.env.miss_band
        if band is None:
            return                  # the noise unmeasured: nothing to act on
        if (not self.trim_pushing and abs(cross) > band):
            self.trim_pushing = True
            self.log.event(snap.ut, "rcs trim: push at cross=%+.0f long=%+.0f"
                           % (cross, long))
        axes = trajectory.miss_axes(self.env.target, snap.position)
        if self.trim_pushing and self.rcs_accel is None:
            self.rcs_accel = self.read_rcs_accel(snap)
        to_go = (self.prediction.time_to_land - self.prediction.burn_time
                 if self.prediction.powered else self.prediction.time_to_land)
        world = (guidance.rcs_trim_command(
                    cross, axes[1], band / self.cfg.NOISE_SIGMAS,
                    self.rcs_accel, to_go)
                 if axes is not None and self.trim_pushing else None)
        if world is None:
            if self.trim_pushing:
                if not self.trim_confirmed:
                    self.trim_confirmed = True
                    self.log.event(snap.ut, "rcs trim: sign confirmed")
                self.log.event(snap.ut, "rcs trim: done at cross=%+.0f "
                               "long=%+.0f" % (cross, long))
            self.trim_pushing = False
            self.trim_run = None
            self.send_translation((0.0, 0.0, 0.0))
            return
        side = 1.0 if cross > 0.0 else -1.0
        if self.trim_run is None or self.trim_run[2] != side:
            self.trim_run = (snap.ut, abs(cross), side)
        elif (not self.trim_confirmed
              and snap.ut - self.trim_run[0] >= self.trim_check_time(band, to_go)
              and abs(cross) > self.trim_run[1] + band):
            if self.trim_flips == 0:
                self.trim_flips = 1
                self.trim_sign = -self.trim_sign
                self.log.event(snap.ut, "rcs trim: cross %+.0f -> %+.0f in "
                               "%.1fs pushing against it: sign flipped"
                               % (side * self.trim_run[1], cross,
                                  snap.ut - self.trim_run[0]))
                self.trim_run = (snap.ut, abs(cross), side)
            else:
                self.trim_off = True
                self.trim_pushing = False
                self.log.event(snap.ut, "rcs trim: no effect either way "
                               "(cross %+.0f), off" % cross)
                self.send_translation((0.0, 0.0, 0.0))
                return
        body = body_vector(snap, vec.scale(world, self.trim_sign))
        if body is None:
            self.send_translation((0.0, 0.0, 0.0))
            return
        # Vessel frame x right, y nose, z bottom; Control.up pushes roofward.
        self.send_translation((body[0], body[1], -body[2]))

    def read_rcs_accel(self, snap):
        """The weakest sideways RCS acceleration (m/s^2, the vessel's x and
        z, both senses), read once while the valve is open: kRPC counts the
        thrusters only then.  None if unreadable or nothing."""
        try:
            pos, neg = self.vessel.available_rcs_acceleration
        except Exception:                                   # noqa: BLE001
            return None
        lateral = [abs(pos[0]), abs(neg[0]), abs(pos[2]), abs(neg[2])]
        accel = min(lateral)
        if accel <= 0.0:
            return None
        self.log.event(snap.ut, "rcs trim: lateral acceleration %.3f m/s^2"
                       % accel)
        return accel

    def trim_check_time(self, band, to_go):
        """How long a push must run before a growing miss means a wrong sign:
        the time full input takes to move the touchdown by ``band``
        (dv x time left), or one nose lag while the RCS is unread."""
        if self.rcs_accel and to_go and to_go > 0.0:
            return band / (self.rcs_accel * to_go)
        return self.lag.lag or 0.0

    def send_translation(self, command):
        """``(right, forward, up)``, each sent only when it has moved."""
        command = tuple(vec.clamp(c, -1.0, 1.0) for c in command)
        last = self.trim_cmd
        for name, new, old in zip(("right", "forward", "up"), command, last):
            if abs(new - old) > 0.02 or (new == 0.0 and old != 0.0):
                try:
                    setattr(self.control, name, new)
                except Exception:                    # noqa: BLE001
                    pass
        self.trim_cmd = command

    def run_late_deploy(self, snap):
        """Aim long on closed brakes, and open them all where that lands.

        Drag is the only braking that costs nothing, and the landing burn
        pays for every m/s the coast leaves it.  So rather than hold the
        brakes part-open the whole way down, hold them closed (their floor)
        -- long -- and open them to ``DEPLOY_MAX_DEG`` low down, in the
        dense air where they brake hardest, at the altitude whose prediction
        lands on the pad.  The switch is the control, solved by bisection
        on the propagator every ``DEPLOY_LATE_STEP_S``, and the environment
        carries it (``late_switch_alt``) so every prediction -- the lift
        solve's, the correction's, the burn trigger's -- flies the schedule.

        The full-brake curve is measured, not extrapolated: once at the top
        of the coast, where the air is too thin for a few seconds of brakes
        to matter, the brakes open to the top, the whole curve is probed,
        and they close to the bottom with the current curve re-taken
        outright (a blended re-sweep would read a quarter of the change).
        Corrections wait for all of it.  Once open they stay open: the switch
        was solved to land on the pad, and the lift trims what is left.
        Every wait for the brakes to move is ``brakes_settled``: the drag
        probed at this instant until it stops changing.
        """
        cfg = self.cfg
        if self.late_state in ("off", "switched", "handed"):
            return
        if cfg.GEAR_TIMED and self.gear_timing in ("deploying", "retracting"):
            return
        if self.late_state is None:
            # The probe measures the vessel at its real attitude, so the
            # full-brake curve has to wait for the coast's descent attitude
            # and the re-sweep taken there: measured mid-flip it read 0.92 x
            # the closed brakes' drag (LOG11855) and the solve flew on it.
            if not (self.env.reswept_ut is not None
                    and self.env.in_descent_configuration(snap.velocity,
                                                          snap.direction)):
                return
            got = set_airbrake_angle(self.vessel, cfg.DEPLOY_MAX_DEG, self.cfg)
            if not got:
                self.late_state = "off"
                self.deploy_none = True
                self.log.event(snap.ut, "brakes: no airbrake answered, law off")
                return
            self.late_state = "full"
            self.brake_probe = None
            self.log.event(snap.ut, "late brakes: open to %.0f to measure"
                           % cfg.DEPLOY_MAX_DEG)
            return
        if self.late_state == "full":
            if not self.brakes_settled(snap):
                return
            bins = self.env.sweep_bins(snap.position, snap.velocity,
                                       snap.rotation, snap.direction)
            set_airbrake_angle(self.vessel, 0.0, self.cfg)
            self.deploy_angle = 0.0
            if not self.env.set_full_curve(bins):
                # No curve: nothing to solve the switch on; brakes closed.
                self.late_state = "off"
                self.log.event(snap.ut, "late brakes: full curve unreadable,"
                               " brakes closed")
                return
            self.late_state = "settling"
            self.brake_probe = None
            return
        if self.late_state == "settling":
            if not self.brakes_settled(snap):
                return
            if self.late_resweep_ut is None:
                self.env.request_resweep(replace=True)
                self.late_resweep_ut = snap.ut
                return
            swept = self.env.last_sweep_ut()
            if swept is None or swept < self.late_resweep_ut:
                return
            self.late_resweep_ut = None
            ratio = self.env.full_curve_ratio()
            self.log.event(snap.ut, "late brakes: Cd*A open is %s x closed"
                           % ("%.2f" % ratio if ratio else "-"))
            if ratio is None or ratio <= 1.0:
                # Opening the brakes cannot reduce drag: a bad measurement.
                self.late_state = "off"
                self.log.event(snap.ut, "late brakes: not more drag open,"
                               " brakes closed")
                return
            self.late_state = "armed"
            self.late_next_ut = snap.ut
        if snap.ut < self.late_next_ut:
            return
        self.late_next_ut = snap.ut + cfg.DEPLOY_LATE_STEP_S
        self.solve_late_switch(snap)

    def brakes_settled(self, snap):
        """True once the brakes have stopped moving: the drag the game's own
        aerodynamics give for this instant's state (one probe a tick) reads
        the same two ticks running.  Measured, so it holds for any brake's
        animation; a probe that cannot be read is not settled."""
        env = self.env
        altitude = vec.norm(snap.position) - env.equatorial_radius
        cda = env._probe(snap.position, vec.unit(snap.velocity), snap.speed,
                         altitude, snap.rotation)
        last, self.brake_probe = self.brake_probe, cda
        if cda is None or last is None:
            return False
        return abs(cda - last) <= 1e-3 * max(abs(cda), abs(last))

    def solve_late_switch(self, snap):
        """Bisect for the brake-opening altitude that nulls the long miss.

        The unsteered prediction: drag carries the
        range and the lift trims what is left.  Higher opens sooner and
        lands shorter.  Long even opening now: open now.  Short even never
        opening: never (the law cannot lengthen past closed brakes).
        """
        cfg, env = self.cfg, self.env
        up = vec.unit(snap.position)
        along = vec.project_out(snap.velocity, up)
        if vec.norm(along) < 1.0:
            return
        along = vec.unit(along)

        def long_at(switch):
            env.late_switch_alt = switch
            p = trajectory.predict_landing(
                env, snap.position, snap.velocity, snap.mass,
                snap.max_accel, cfg, steer=None)
            miss = vec.project_out(vec.sub(p.position, env.target), up)
            return vec.dot(miss, along)

        lo = env.target_radius - env.equatorial_radius - 1.0
        hi = vec.norm(snap.position) - env.equatorial_radius
        f_lo, f_hi = long_at(lo), long_at(hi)
        if f_hi >= 0.0:
            switch, f = hi, f_hi
        elif f_lo <= 0.0:
            switch, f = None, f_lo
        else:
            switch, f = hi, f_hi
            for _ in range(24):
                mid = 0.5 * (lo + hi)
                f = long_at(mid)
                switch = mid
                band = env.miss_band
                if (band is not None and abs(f) <= band) or hi - lo < 1.0:
                    break
                if f > 0.0:
                    lo = mid
                else:
                    hi = mid
        env.late_switch_alt = switch
        self.log.event(snap.ut, "late brakes: open at %s long=%+.0f"
                       " (closed %+.0f, open now %+.0f)"
                       % ("%.0f m" % switch if switch is not None else "never",
                          f, f_lo, f_hi))

    def late_switch(self, snap):
        """Open the brakes where the solve said; hand over once re-swept."""
        cfg, env = self.cfg, self.env
        if self.late_state == "armed":
            if self.state not in (COAST, LANDING_BURN):
                return
            altitude = vec.norm(snap.position) - env.equatorial_radius
            if env.late_switch_alt is None or altitude > env.late_switch_alt:
                return
            set_airbrake_angle(self.vessel, cfg.DEPLOY_MAX_DEG, self.cfg)
            self.deploy_angle = cfg.DEPLOY_MAX_DEG
            env.late_switch_alt = float("inf")  # all of it at full now
            self.late_state = "switched"
            self.brake_probe = None
            self.log.event(snap.ut, "late brakes %.0f -> %.0f at %.0f m,"
                           " %.0f m/s" % (0.0, cfg.DEPLOY_MAX_DEG,
                                          snap.height_above_pad, snap.speed))
        elif self.late_state == "switched" and self.brakes_settled(snap):
            if self.late_resweep_ut is None:
                env.request_resweep(replace=True)
                self.late_resweep_ut = snap.ut
                return
            swept = env.last_sweep_ut()
            if swept is None or swept < self.late_resweep_ut:
                return
            env.late_switch_alt = None
            self.late_state = "handed"

    def time_gear(self, snap):
        """How long the legs take to deploy: from the cache, or timed once.

        Nothing in kRPC says how long a part's deploy animation is (the
        length is in the model, not the part's config), so it is measured:
        the gear group fired early in the coast, where the air is too thin
        for the legs' drag to matter, each deployable leg and wheel watched
        until it reads deployed, then fired back.  Each part's time goes in
        ``gearcache`` under its part name, so a craft whose parts are all
        there skips the cycle -- every flight after the first.
        ``landing_gear`` drops them the slowest's time, plus a few measured
        reaction times, before the ground.  No legs, or a cycle that
        does not finish: the legs drop when the burn lights, as untimed.
        """
        cfg = self.cfg
        if self.gear_timing in ("done", "failed") or self.gear_deployed:
            return
        if self.gear_timing is None:
            if self.state != COAST:
                return
            self.gear_items = gear_parts(self.vessel)
            if not self.gear_items:
                self.gear_timing = "failed"
                self.log.event(snap.ut, "gear: nothing deployable to time")
                return
            self.gear_names = [part_name(p) for p in self.gear_items]
            cached = (gearcache.lookup(self.gear_names)
                      if None not in self.gear_names else None)
            if cached is not None:
                self.gear_deploy_s = max(cached.values())
                self.gear_timing = "done"
                self.log.event(snap.ut, "gear: %s deploy in %.2f s (cached);"
                               " drop at %.1f s to go"
                               % (_name_counts(self.gear_names),
                                  self.gear_deploy_s, self.gear_lead()))
                return
            self.fire_gear(True)
            self.gear_timing = "deploying"
            self.gear_t0 = snap.ut
            self.gear_done = [None] * len(self.gear_items)
            return
        states = [deploy_state(p) for p in self.gear_items]
        if snap.ut - self.gear_t0 > cfg.GEAR_MEASURE_TIMEOUT_S:
            self.fire_gear(False)
            self.gear_timing = "failed"
            self.gear_deploy_s = None
            self.log.event(snap.ut, "gear: cycle unfinished after %.0f s (%s)"
                           ", untimed" % (cfg.GEAR_MEASURE_TIMEOUT_S,
                                          "/".join(sorted(set(map(str, states))))))
            return
        if self.gear_timing == "deploying":
            for i, st in enumerate(states):
                if self.gear_done[i] is None and st in ("deployed", "broken"):
                    self.gear_done[i] = (snap.ut - self.gear_t0
                                         if st == "deployed" else False)
            if all(d is not None for d in self.gear_done):
                times = [d for d in self.gear_done if d is not False]
                self.gear_deploy_s = max(times) if times else None
                self.read_landing_limits(snap, "gear timing")
                self.fire_gear(False)
                self.gear_timing = "retracting"
                self.gear_t1 = snap.ut
        elif all(st in ("retracted", "broken") for st in states):
            if self.gear_deploy_s is None:
                self.gear_timing = "failed"
                self.log.event(snap.ut, "gear: every part broken, untimed")
                return
            self.gear_timing = "done"
            retract = snap.ut - self.gear_t1
            # Per part name, the slowest of its kind (a tick's resolution).
            entries = {}
            for name, d in zip(self.gear_names, self.gear_done):
                if name is None or d is False:
                    continue
                old = entries.get(name, {}).get("deploy", 0.0)
                entries[name] = {"deploy": round(max(old, d), 3),
                                 "retract": round(retract, 3)}
            try:
                gearcache.record(entries)
                saved = "cached"
            except OSError as exc:
                saved = "not cached: %s" % exc
            self.log.event(snap.ut, "gear timed: %s deploy in %.2f s,"
                           " retract in %.2f s (%s); drop at %.1f s to go"
                           % (_name_counts(self.gear_names),
                              self.gear_deploy_s, retract, saved,
                              self.gear_lead()))

    def gear_lead(self):
        # The legs' measured deploy time, plus SETTLE_LAGS measured
        # reaction times for the command to reach them.
        return (self.gear_deploy_s + self.cfg.SETTLE_LAGS
                * trajectory.reaction_time(self.env))

    def fire_gear(self, down):
        self.control.gear = down
        if self.cfg.GEAR_ACTION_GROUP:
            self.control.set_action_group(self.cfg.GEAR_ACTION_GROUP, down)

    def time_to_ground(self, snap):
        """Seconds to touchdown, the sooner of the prediction's and the
        fall at the present descent rate (which only slows from here)."""
        best = None
        if snap.vertical_speed < -0.1:
            best = snap.landing_height / -snap.vertical_speed
        if self.prediction is not None and self.prediction.time_to_land > 0.0:
            t = self.prediction.time_to_land
            best = t if best is None else min(best, t)
        return best

    def measure_tracking(self, snap):
        """How much of the commanded angle of attack is the vehicle holding?

        The command in force is the one the last tick set; the angle held is
        the nose's real angle off retrograde.  A pair counts only once the
        command has stayed within the pointing noise for ``SETTLE_LAGS``
        nose lags -- a command that just moved has not had
        time to be held, and reading the turn as the vehicle's limit would
        throw away lift it does have.  See ``Environment.lift_slope_at``.
        """
        cfg = self.cfg
        if self.steer is None:
            self.steer_history = []
            return
        # Steady for SETTLE_LAGS measured nose lags, to within the pointing
        # error's own noise (ValveLimits): both measured, or no sample.
        lag = self.lag.lag
        noise = self.rcs_limits.RCS_ERROR_OFF_DEG
        if not lag or not self.rcs_limits.measured:
            return
        settle = cfg.SETTLE_LAGS * lag
        commanded = math.degrees(abs(self.steer.aoa))
        self.steer_history.append((snap.ut, commanded))
        window = snap.ut - settle
        while len(self.steer_history) > 1 and self.steer_history[1][0] <= window:
            self.steer_history.pop(0)
        if self.steer_history[0][0] > window:
            return                                  # not steady for long enough
        values = [c for _, c in self.steer_history]
        if max(values) - min(values) > noise or commanded < noise:
            return
        held = vec.angle_between(vec.scale(snap.direction, -1.0),
                                 snap.velocity)
        if snap.ut - self.phase_start_ut < settle:
            return              # still swinging back from the last phase
        if held > max(values) + noise:
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

    def burn_slew_rate(self):
        """deg/s the landing burn's command may move: what the nose follows
        to within the burn's own tilt allowance, the attitude loop's measured
        closing rate times ``LANDING_MAX_TILT_DEG`` (a first-order follower
        lags a ramp of w by w / rate).  A command that steps is one no vehicle
        flies: unslewed, the old law's command reversed 60 deg in a tick and
        the booster chased it (cmd1-1017).  None (unslewed) until measured."""
        lam = self.lag.rate
        if lam is None:
            return None
        return lam * self.cfg.LANDING_MAX_TILT_DEG

    def steady_cmd_rate(self):
        """deg/s the command moves, the median of the last ``jitter.SAMPLES``
        ticks: a command that steps (a new phase, a flip) is one tick of
        enormous rate, and it is not a rate the nose trails."""
        if self.cmd_rate is not None:
            self.cmd_rates.append(self.cmd_rate)
            if len(self.cmd_rates) > JITTER_SAMPLES:
                self.cmd_rates.pop(0)
        if not self.cmd_rates:
            return None
        return sorted(self.cmd_rates)[len(self.cmd_rates) // 2]

    def velocity_band(self, snap):
        """The horizontal speed below which the trajectory plane cannot be
        read: ``NOISE_SIGMAS`` sd of the horizontal velocity's tick scatter.
        Unmeasured: infinite (no plane)."""
        horizontal = vec.project_out(snap.velocity, vec.unit(snap.position))
        self.velocity_jitter.update(horizontal)
        band = self.velocity_jitter.band(self.cfg.NOISE_SIGMAS)
        return float("inf") if band is None else band

    def unwound(self, steer, prediction):
        """``steer`` with its angle faded to zero over the last
        ``SETTLE_LAGS`` measured nose lags before the predicted
        ignition, so the burn lights with the nose on retrograde.  A nose
        still holding the coast's 5-7 deg when the engines lit kept it ~5 s
        and the tilted thrust carried the touchdown 20-100 m long
        (nodiv-1115).  The coast's predictions still fly the full angle; the
        lift lost over the last second or so is re-solved every tick.

        The lag is the *unpowered* nose's (``coast_lag``): the boostback's
        gimballed one read 0.8 s where the coast's nose took ~3 s, so the
        fade ended with the nose 9-13 deg off retrograde at ignition and the
        burn braked ~10 m/s less sideways than predicted (tick-1117).  The
        coast's predictions fly the same fade (``trajectory.unwind_share``,
        ``set_unwind``)."""
        span = self.unwind_span()
        if (steer is None or not steer.aoa or span is None
                or prediction is None or not prediction.powered):
            return steer
        to_ignition = max(0.0, prediction.time_to_land - prediction.burn_time)
        return replace(steer, aoa=steer.aoa
                       * trajectory.unwind_share(to_ignition, 0.0, span))

    def unwind_span(self):
        """Seconds the coast's angle fades over before ignition:
        ``SETTLE_LAGS`` of the unpowered nose's measured lag; None until it
        is measured."""
        lag = self.coast_lag.lag
        return None if lag is None else self.cfg.SETTLE_LAGS * lag

    def set_unwind(self, snap):
        """Tell the coast's predictions when the fade runs: the last
        prediction's ignition, from now, and the span."""
        p = self.prediction
        span = self.unwind_span()
        if p is None or not p.powered or span is None:
            self.env.unwind_in = self.env.unwind_span = None
            return
        self.env.unwind_in = max(0.0, p.time_to_land - p.burn_time
                                 - (snap.ut - self.prediction_ut))
        self.env.unwind_span = span

    def null_rate(self, snap):
        """The lateral null's highest rate (1/s): ``LOOP_BANDWIDTH_SHARE`` of
        the attitude loop's closing rate as measured in flight (``common.attitudelag``: one over
        the nose's measured delay behind its command), or None until that
        fit is confident.  A lateral loop faster than the nose that executes
        it is the last-20 m oscillation (dv3-1115).  Logged each time it
        moves by a quarter."""
        lam = self.lag.rate
        if lam is None:
            return None
        rate = self.cfg.LOOP_BANDWIDTH_SHARE * lam
        last = self.lag_logged
        if last is None or abs(rate - last) > 0.25 * last:
            self.lag_logged = rate
            self.log.event(snap.ut, "lateral null rate %.2f /s (nose lag "
                           "%.2f s, residual %.0f%% of none, %d ticks, %s)"
                           % (rate, self.lag.lag or 0.0,
                              100.0 * (self.lag.rel_error or 0.0),
                              self.lag.samples, self.state))
        return rate

    def run_landing_burn(self, snap):
        if self.env.taper_rate is None:
            rate = trajectory.one_law_rate(self.env, self.cfg)
            if rate is not None:
                self.env.taper_rate = min(
                    rate, self.cfg.LOOP_BANDWIDTH_SHARE
                    * trajectory.throttle_gain(self.env, self.cfg))
                self.log.event(snap.ut, "taper rate held at %.2f /s"
                               % self.env.taper_rate)
        self.log_attitude_tune(snap)
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        prediction = self.predict(snap, powered=True)
        # Throttle first: the steering law has to know how much acceleration
        # it is actually going to get, not how much the vehicle could make at
        # full thrust -- see guidance.terminal_command.
        command = guidance.landing_throttle(self.env, snap.position,
                                            snap.velocity, snap.max_accel,
                                            self.cfg,
                                            height=snap.landing_height)
        throttle = command.throttle
        zem = trajectory.miss_vector(self.env, prediction)
        budget = self.tilt_budget(throttle)
        steer = guidance.terminal_command(
            self.env, snap.position, snap.velocity, snap.landing_height,
            prediction.time_to_land, throttle * snap.max_accel, self.cfg, zem,
            budget, self.null_rate(snap), mass=snap.mass)
        # Thrust tilted off retrograde brakes with only its cosine, and the
        # vertical profile above sized the throttle as though all of it were
        # braking.  Left alone, a divert quietly under-brakes for exactly as
        # long as it steers -- the trim term only notices once the vehicle is
        # already fast -- so give back what the tilt takes.  Nothing changes
        # when the divert is off, because then the tilt is zero.
        brake = vec.scale(snap.velocity, -1.0)
        rate = trajectory.one_law_rate(self.env, self.cfg)
        if rate is not None:
            brake = trajectory.base_direction(
                snap.velocity, vec.unit(snap.position),
                throttle * snap.max_accel, rate)
        tilt = vec.angle_between(steer.direction, brake)
        if tilt > 1.0:
            throttle = min(self.cfg.LANDING_THROTTLE_CAP,
                           throttle / math.cos(math.radians(tilt)))
        self.set_throttle(throttle)
        direction = steer.direction
        rate = self.burn_slew_rate()
        if rate is not None:
            direction = guidance.slew(
                self.burn_cmd, direction,
                rate * max(0.0, snap.ut - self.burn_cmd_ut))
        self.burn_cmd, self.burn_cmd_ut = direction, snap.ut
        self.aim(direction, snap)
        if self.cfg.DIAG_BURN:
            self.log_burn_diag(snap, throttle, brake)

        self.extra_lines = [
            "miss    : %8.0f m" % self.miss_distance(),
            "legs alt: %8.1f m" % snap.legs_altitude,
            "v target: %8.1f m/s" % command.target_speed,
            "v error : %8.1f m/s" % (snap.speed - command.target_speed),
        ]
        if self.touched_down(snap, command):
            self.set_throttle(0.0)
            self.enter(TOUCHDOWN, snap.ut,
                       "legs=%.1fm vs=%.1f td_pad=%.1fm td_hs=%.1f td_tilt=%.1f"
                       % (snap.legs_altitude, snap.vertical_speed,
                          self.pad_distance(snap), snap.horizontal_speed,
                          vec.angle_between(snap.direction, snap.position)))
            self.log_touchdown(snap)

    # DIAG_BURN's model variants: which of the burn propagation's terms each
    # leaves out (env attribute -> the value that switches it off).
    BURN_PLAN_VARIANTS = (
        # burn_lit True: no dead time (and the nose starts where it is).
        ("all", {}),
        ("base", {"burn_isp": None, "nose_lag": None, "burn_lit": True}),
        ("isp", {"nose_lag": None, "burn_lit": True}),
        ("dead", {"burn_isp": None, "nose_lag": None}),
        ("lag", {"burn_isp": None, "burn_lit": True}),
        ("wet", {"burn_dry_mass": None}),
    )

    def log_burn_plans(self, snap):
        """DIAG_BURN: the burn each propagation model flies from here, one
        ``burn plan`` line each (``all`` first: what the coast aims with)."""
        self.burn_plan_ut = snap.ut
        self.update_burn_env(snap)
        for name, off in self.BURN_PLAN_VARIANTS:
            saved = {k: getattr(self.env, k, None)
                     for k in ("burn_isp", "nose_lag", "burn_lit",
                               "burn_dry_mass")}
            try:
                for k, val in off.items():
                    setattr(self.env, k, val)
                trace = []
                trajectory.predict_landing(
                    self.env, snap.position, snap.velocity, snap.mass,
                    snap.max_accel, self.cfg, trace=trace)
            finally:
                for k, val in saved.items():
                    setattr(self.env, k, val)
            out, next_t = [], 0.0
            for t, h, hs, vs, thr in trace:
                if t >= next_t:
                    out.append("%.2f:%.1f:%.1f:%.1f:%.3f" % (t, h, hs, vs, thr))
                    next_t = t + 0.25
            if trace:
                out.append("%.2f:%.1f:%.1f:%.1f:%.3f" % trace[-1])
            self.log.event(snap.ut, "burn plan %s m=%.2f acc=%.3f isp=%s "
                           "lag=%s rate=%s legs=%.2f samples=[%s]"
                           % (name, snap.mass / 1000.0, snap.max_accel,
                              self.env.burn_isp, self.env.nose_lag,
                              trajectory.one_law_rate(self.env, self.cfg),
                              trajectory.leg_clearance(self.env),
                              " ".join(out)))

    def log_burn_diag(self, snap, throttle, brake):
        """DIAG_BURN: the burn the prediction flies, once at the first burn
        tick, then the flown burn every tick on the same clock and heights,
        so burnplan.py can say which term the two part on."""
        radius = vec.norm(snap.position)
        h_com = (radius - self.env.target_radius
                 - trajectory.leg_clearance(self.env))
        if self.burn_plan_ut is None:
            self.log_burn_plans(snap)
        vs = vec.dot(snap.velocity, vec.unit(snap.position))
        hs = math.sqrt(max(0.0, snap.speed ** 2 - vs * vs))
        delivered = (snap.thrust / snap.mass
                     if snap.thrust is not None and snap.mass > 0 else float("nan"))
        altitude = radius - self.env.equatorial_radius
        self.log.event(snap.ut, "burn tick t=%.2f h=%.1f hl=%.1f hs=%.2f vs=%.2f "
                       "thr=%.3f got=%.3f acc=%.3f max=%.3f m=%.2f off=%.2f "
                       "cda=%.2f M=%.2f"
                       % (snap.ut - self.burn_plan_ut, h_com, snap.landing_height,
                          hs, vs, throttle, snap.throttle, delivered,
                          snap.max_accel, snap.mass / 1000.0,
                          vec.angle_between(snap.direction, brake),
                          self.env.drag_area_at(snap.speed, altitude),
                          self.env.mach(snap.speed, altitude)))
        if self.cfg.DIAG_DRAG:
            self.log_burn_forces(snap, altitude)

    def log_burn_forces(self, snap, altitude):
        """DIAG_BURN: the horizontal force budget, m/s^2, against the
        model's: the game's aerodynamic force at this very state and
        attitude, resolved along the horizontal velocity (``aH``, negative
        braking) and vertical (``aV``), beside the model's drag on the same
        axes (``mH``/``mV``: Cd*A q along -v) and the thrust's horizontal
        part (``tH``).  burnplan.py --forces reads them."""
        if snap.speed < 1.0 or snap.mass <= 0.0:
            return
        force = self.env._probe_force(snap.position, snap.velocity,
                                      snap.rotation)
        if force is None:
            return
        up = vec.unit(snap.position)
        flat = vec.project_out(snap.velocity, up)
        if vec.norm(flat) < 1e-6:
            return
        fwd = vec.unit(flat)
        rho = self.env.density(altitude)
        q = 0.5 * rho * snap.speed * snap.speed
        model = vec.scale(vec.unit(snap.velocity),
                          -q * self.env.drag_area_at(snap.speed, altitude))
        thrust = (snap.thrust or 0.0) / snap.mass
        self.log.event(snap.ut, "burn force aH=%.2f aV=%.2f mH=%.2f mV=%.2f "
                       "tH=%.2f side=%.2f"
                       % (vec.dot(force, fwd) / snap.mass,
                          vec.dot(force, up) / snap.mass,
                          vec.dot(model, fwd) / snap.mass,
                          vec.dot(model, up) / snap.mass,
                          thrust * vec.dot(snap.direction, fwd),
                          vec.norm(vec.project_out(
                              vec.project_out(force, up), fwd)) / snap.mass))

    def log_drag_diag(self, snap):
        """DIAG_DRAG: the game's Cd*A at this very state (``real``), what a
        sweep would record for this speed now (``map``: the same attitude,
        at the bin's probe altitude), and what the propagation flies there
        (``curve``: the coast's curve, the burn's in the burn), twice a
        game-second in the coast and four times in the burn."""
        if self.state not in (COAST, LANDING_BURN) or snap.speed < 20.0:
            return
        env = self.env
        if (self.state == LANDING_BURN and self.ladder_machs
                and env.mach(snap.speed, vec.norm(snap.position)
                             - env.equatorial_radius) < self.ladder_machs[0]):
            self.ladder_machs.pop(0)
            self.log_lift_ladder(snap, "burn")
        if snap.ut < self.drag_diag_next:
            return
        self.drag_diag_next = snap.ut + (0.5 if self.state == COAST else 0.25)
        env = self.env
        altitude = vec.norm(snap.position) - env.equatorial_radius
        unit = vec.unit(snap.velocity)
        real = env._probe(snap.position, unit, snap.speed, altitude,
                          snap.rotation)
        at = env.probe_altitude(snap.speed, altitude)
        mapped = env._probe(env.probe_position(snap.position, at), unit,
                            snap.speed, at, snap.rotation)
        curve = (env.burn_drag_area_at if self.state == LANDING_BURN
                 else env.drag_area_at)(snap.speed, altitude)
        inputs = getattr(snap, "inputs", None)
        self.log.event(snap.ut, "drag diag st=%s M=%.2f alt=%.0f real=%s "
                       "map=%s at=%.0f curve=%.2f aoa=%.1f dep=%.0f "
                       "neutral=%d in=%s"
                       % (self.state, env.mach(snap.speed, altitude), altitude,
                          "%.2f" % real if real is not None else "-",
                          "%.2f" % mapped if mapped is not None else "-",
                          at, curve,
                          vec.angle_between(snap.direction,
                                            vec.scale(unit, -1.0)),
                          self.deploy_angle or 0.0,
                          int(self.neutral_from is not None),
                          "%+.2f/%+.2f/%+.2f" % inputs if inputs else "-"))

    def log_lift_ladder(self, snap, where):
        """DIAG_DRAG: the aerodynamic force at the real state with the nose
        turned 0-15 deg off retrograde in the vertical plane (+ toward the
        vertical), as Cd*A along -v and Cn*A toward the tilt's side."""
        env = self.env
        altitude = vec.norm(snap.position) - env.equatorial_radius
        q = 0.5 * env.density(altitude) * snap.speed ** 2
        retro = vec.scale(vec.unit(snap.velocity), -1.0)
        side = vec.project_out(vec.unit(snap.position), retro)
        if q <= 0.0 or vec.norm(side) < 1e-6:
            return
        side = vec.unit(side)
        out = []
        for deg in (-15, -8, -4, -2, -1, 0, 1, 2, 4, 8, 15):
            a = math.radians(deg)
            target = vec.add(vec.scale(retro, math.cos(a)),
                             vec.scale(side, math.sin(a)))
            rot = vec.rotation_onto(snap.rotation, snap.direction, target)
            f = env._probe_force(snap.position, snap.velocity, rot) \
                if rot is not None else None
            if f is None:
                continue
            out.append("%+d:%.2f/%+.3f" % (deg, vec.dot(f, retro) / q,
                                           vec.dot(f, side) / q))
        self.log.event(snap.ut, "lift ladder (%s) M=%.2f alt=%.0f real aoa "
                       "%.1f: %s" % (where, env.mach(snap.speed, altitude),
                                     altitude, vec.angle_between(
                                         snap.direction, retro),
                                     " ".join(out)))

    def hold_fins(self, hold):
        """Every control surface held at no deflection (kRPC's override,
        which also retracts a deployed one: only with the brakes closed), or
        handed back.  False if the game would not take it."""
        try:
            for cs in self.vessel.parts.control_surfaces:
                cs.deflection_override = hold
                if hold:
                    cs.deflection = 0.0
        except Exception:                               # noqa: BLE001
            return False
        return True

    def sweep_burn_curve(self, snap):
        """The landing burn's Cd*A curve, swept in the coast with the fins
        held neutral, as the burn will fly them.

        The coast holds its angle of attack on the fins and a probe sees
        them as they sit -- pinned, most of the way down -- so its curve is
        the drag of a booster the burn is not (Environment.
        burn_drag_area_at).  Held neutral, the fins' drag falls away over
        their actuators' travel; the sweep is taken the first tick the drag
        at the real state stops falling, and the fins are handed back.
        Taken once the coast has settled on its descent attitude with the
        brakes closed and not being measured, and again every
        ``DRAG_RESWEEP_INTERVAL_S``, as the probe altitudes converge.  If
        the nose wanders past the RCS's own on-threshold while the fins
        are held, they are handed back and the sweep waits for the next
        interval."""
        env = self.env
        if self.state != COAST:
            if self.neutral_from is not None:
                self.release_fins(snap, "coast over")
            return
        if self.neutral_from is None:
            if (snap.ut < self.neutral_next_ut or not self.coast_settled
                    or self.pointing_error(snap)
                    > self.rcs_limits.RCS_ERROR_OFF_DEG
                    or getattr(env, "reswept_ut", None) is None
                    or self.late_state in ("full", "settling")
                    or (self.deploy_angle or 0.0) != 0.0):
                return
            if self.hold_fins(True):
                self.neutral_from, self.neutral_last = snap.ut, None
                self.neutral_fell = False
            else:
                self.neutral_next_ut = float("inf")
                self.log.event(snap.ut, "burn curve: fins cannot be held")
            return
        if self.pointing_error(snap) > self.rcs_limits.RCS_ERROR_ON_DEG:
            self.release_fins(snap, "nose off %.1f deg"
                              % self.pointing_error(snap))
            return
        altitude = vec.norm(snap.position) - env.equatorial_radius
        cda = env._probe(snap.position, vec.unit(snap.velocity), snap.speed,
                         altitude, snap.rotation)
        if cda is None:
            return
        last, self.neutral_last = self.neutral_last, cda
        if last is None:
            return
        if cda < last * (1.0 - 1e-3):
            self.neutral_fell = True
            return
        if not self.neutral_fell:
            # Not moving yet: the override reaches the fins a frame or two
            # after it is sent, and a probe before that reads them pinned.
            return
        bins = env.sweep_bins(snap.position, snap.velocity, snap.rotation,
                              snap.direction)
        held = snap.ut - self.neutral_from
        if env.set_burn_curve(bins):
            self.log.event(snap.ut, "burn curve swept, fins neutral %.2f s "
                           "(Cd*A here %.2f, coast's %.2f): %s"
                           % (held, cda, env.drag_area_at(snap.speed, altitude),
                              " ".join("M%.2f %.1f" % (env._bin_mach(i), a)
                                       for i, a in enumerate(bins)
                                       if a is not None)))
            if self.cfg.DIAG_DRAG:
                self.log_lift_ladder(snap, "coast, fins neutral")
            self.release_fins(snap, None)
            self.neutral_next_ut = float("inf")     # once
            return
        self.release_fins(snap, None)

    def release_fins(self, snap, why):
        self.hold_fins(False)
        self.neutral_from = None
        self.neutral_next_ut = snap.ut + self.cfg.DRAG_RESWEEP_INTERVAL_S
        if why:
            self.log.event(snap.ut, "burn curve: fins handed back (%s)" % why)

    def read_landing_limits(self, snap, why):
        """The legs' live suspension, deployed, as touchdown limits: once.

        One batched kRPC round trip (``common.touchdown.read_legs``) while
        the legs are out -- the gear-timing cycle in the coast, or after gear
        down -- on the constants read before the flight.
        """
        if not self.leg_constants:
            self.leg_constants = read_leg_constants(self.conn, self.vessel)
        legs = read_legs(self.conn, self.vessel, self.leg_constants)
        self.landing_limits = False
        if not legs:
            self.log.event(snap.ut, "landing limits: no legs read (%s)" % why)
            return
        try:
            moi = self.vessel.moment_of_inertia
            transverse = min(moi[0], moi[2])
            g = self.body.surface_gravity
        except Exception:                                   # noqa: BLE001
            self.log.event(snap.ut, "landing limits: no inertia (%s)" % why)
            return
        lim = landing_limits(snap.mass, transverse, g, legs)
        leg = legs[0]
        self.log.event(snap.ut, "legs (%s): %d x %s spring %.0f damper %.0f "
                       "travel %.2f impact %.0f crash %.0f friction %.2f%s"
                       % (why, len(legs), leg["title"], leg["spring"],
                          leg["damper"], leg["travel"], leg["impact_tol"],
                          leg["crash_tol"], leg["friction"],
                          "" if all(l["deployed"] for l in legs)
                          else " (not all deployed)"))
        if lim is None:
            self.log.event(snap.ut, "landing limits: fewer than 3 feet")
            return
        self.landing_limits = lim
        if lim.get("lateral") and lim["lateral"] > 0.0:
            self.env.lateral_tolerance = lim["lateral"]
        if lim.get("h") and lim["h"] > 0.0:
            self.env.tip_angle_deg = math.degrees(math.atan2(lim["b"], lim["h"]))
        if lim.get("vertical") and (self.env.touchdown_tolerance is None
                                    or lim["vertical"]
                                    < self.env.touchdown_tolerance):
            self.env.touchdown_tolerance = lim["vertical"]
            self.log.event(snap.ut, "touchdown target %.1f m/s (the legs)"
                           % trajectory.touchdown_speed(self.env, self.cfg))
        self.log.event(snap.ut, "landing limits: vertical %s m/s (damper %s, "
                       "stroke %.1f), lateral %.1f m/s stuck; COM %.1f m over a "
                       "%.1f m support margin, %.1f deg suspension tilt; %s"
                       % ("%.1f" % lim["vertical"] if lim["vertical"] else "?",
                          "%.1f" % lim["vertical_damper"]
                          if lim.get("vertical_damper") else "?",
                          lim.get("vertical_bottom", float("nan")),
                          lim["lateral"], lim["h"], lim["b"],
                          lim.get("tilt_deg", float("nan")),
                          "a sliding foot can tip it (friction x h > b)"
                          if lim["slide_tips"] else "sliding cannot tip it"))

    def log_touchdown(self, snap):
        """Contact speed against the limit measured before the flight."""
        lim = self.landing_limits or None
        if lim and lim.get("vertical"):
            self.log.event(snap.ut, "touchdown vs %.1f / hs %.1f m/s against legs' "
                           "vertical %.1f / lateral %.1f -- %s"
                           % (snap.vertical_speed, snap.horizontal_speed,
                              lim["vertical"], lim["lateral"],
                              "HARD" if (-snap.vertical_speed > lim["vertical"]
                                         or snap.horizontal_speed > lim["lateral"])
                              else "within"))
            return
        if self.touchdown_limit is None:
            return
        tol = self.touchdown_limit[0]
        self.log.event(snap.ut, "touchdown %.1f m/s (vs %.1f, hs %.1f) against "
                       "limit %.1f m/s -- %s"
                       % (snap.speed, snap.vertical_speed, snap.horizontal_speed,
                          tol, "HARD" if snap.speed > tol else "within"))

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
        the touchdown speed by then.  The other two cases are backstops for a
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
        # Stopped descending short of the ground needs no cut here: the
        # profile gives no thrust while not descending (landing_command).
        return command.height <= self.cfg.TOUCHDOWN_ALT_M

    def run_touchdown(self, snap):
        self.set_throttle(0.0)
        distance = self.pad_distance(snap)
        self.extra_lines = ["down %0.0f m from pad" % distance]
        if self.touchdown_record is None and snap.ut - self.phase_start_ut > 2.0:
            # Say what the game thinks the vehicle is doing, not just how far
            # away it is: LOG5 signed off "landed 15706 m from pad" while the
            # booster was still 24 km up doing 386 m/s.
            self.touchdown_record = ("%s %.0f m from pad, alt=%.0f spd=%.1f"
                                     % (snap.situation, distance,
                                        snap.landing_height, snap.speed))
        if self.touchdown_record is not None:
            # ... and then whether it stays up: a tip-over takes seconds, and
            # the 2 s figures above are taken before one shows.
            tilt = vec.angle_between(snap.direction, snap.position)
            self.touchdown_tilt = max(self.touchdown_tilt, tilt)
            if snap.ut - self.phase_start_ut > 2.0 + self.cfg.TOUCHDOWN_WATCH_S:
                self.finish("%s tilt=%.0f max=%.0f" % (
                    self.touchdown_record, tilt, self.touchdown_tilt))

    # -- main loop ---------------------------------------------------------
    def landing_gear(self, snap):
        if self.gear_deployed:
            if (self.landing_limits is None and self.gear_down_ut is not None
                    and self.gear_deploy_s is not None
                    and snap.ut - self.gear_down_ut > self.gear_deploy_s + 0.5):
                self.read_landing_limits(snap, "gear down")
            return
        # The burn is the last thing that happens, so anything the legs are
        # going to cost -- drag, and a bin of the curve written without them
        # -- is cheaper paid now than a leg that is still folding at
        # touchdown. GEAR_DEPLOY_ALT_M remains as a backstop for a flight
        # that somehow reaches the ground without the phase.
        why = None
        if self.cfg.GEAR_TIMED and self.gear_timing == "done":
            if self.state not in (COAST, LANDING_BURN):
                return
            t_go = self.time_to_ground(snap)
            if t_go is None or t_go > self.gear_lead():
                return
            why = "%.1f s to go" % t_go
        elif self.cfg.GEAR_TIMED and self.gear_timing in ("deploying",
                                                           "retracting"):
            return
        elif self.cfg.GEAR_ON_LANDING_BURN and self.state == LANDING_BURN:
            why = "landing burn"
        elif self.state in (COAST, LANDING_BURN) \
                and snap.height_above_pad <= self.cfg.GEAR_DEPLOY_ALT_M:
            why = "%.0f m" % self.cfg.GEAR_DEPLOY_ALT_M
        if why is None:
            return
        self.fire_gear(True)
        self.gear_deployed = True
        self.gear_down_ut = snap.ut
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
        self.last_snap = snap
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
        # The command in force since the last tick is the one the nose has
        # been following: aim() has not run yet this tick.  Powered ticks
        # only: the burn turns on its gimbals, and the unpowered coast (fins,
        # RCS, the air) is another plant -- its rate, ~1/3 of the boostback's
        # in the sim, predicted a burn nobody flew and lit it 500 m high.
        if self.state != STANDBY and self.demand > 0.0 and (
                snap.thrust is None or snap.thrust > 0.0):
            self.lag.update(snap.ut, snap.direction, self.commanded_direction,
                            snap.input_saturation)
        else:
            self.lag.gap()
        if self.state == COAST and self.demand <= 0.0:
            before = self.coast_lag.lag
            self.coast_lag.update(snap.ut, snap.direction,
                                  self.commanded_direction,
                                  snap.input_saturation)
            if self.coast_lag.lag is not None and (
                    before is None
                    or abs(self.coast_lag.lag - before) > 0.25 * before):
                self.log.event(snap.ut, "coast nose lag %.2f s (residual "
                               "%.0f%% of none, %d ticks)"
                               % (self.coast_lag.lag,
                                  100.0 * (self.coast_lag.rel_error or 0.0),
                                  self.coast_lag.samples))
        else:
            self.coast_lag.gap()
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

    def track_spool(self, snap):
        """Time the engines from the throttle-up to (1 - 1/e) of the thrust
        commanded, once, at the first ignition (the boostback's), for the
        landing burn's trigger (``trajectory.reaction_time``); and keep the
        control interval the trigger is checked at."""
        if self.last_ut is not None and snap.ut > self.last_ut:
            # The loop's interval, not the last tick's: the trigger's tick
            # carries the burn's one-off entry work (0.3-0.4 s), and the
            # burn's taper rate, held from the next tick, was capped by a
            # reaction time read off that one tick -- 0.54 /s against the
            # 0.78 every coast prediction planned with, so the burn flew
            # slower and longer than predicted (lit-1117).
            self.tick_intervals.append(snap.ut - self.last_ut)
            if len(self.tick_intervals) > JITTER_SAMPLES:
                self.tick_intervals.pop(0)
            ordered = sorted(self.tick_intervals)
            self.env.control_interval = ordered[len(ordered) // 2]
        self.last_ut = snap.ut
        start = self.spool_from
        if start is None or snap.thrust is None or snap.available_thrust <= 0:
            return
        if snap.thrust >= (1.0 - math.exp(-1.0)) * start[1] \
                * snap.available_thrust:
            self.spool_from = None
            # Less the tick the reading itself is late by: the throttle went
            # up during the last tick, the thrust is seen on this one.
            self.env.engine_spool_s = max(
                0.0, snap.ut - start[0] - (self.env.control_interval or 0.0))
            self.log.event(snap.ut, "engine spool-up %.2f s (to 63%% of "
                           "throttle %.2f; tick %.2f s)"
                           % (self.env.engine_spool_s, start[1],
                              self.env.control_interval or 0.0))

    def _tick(self):
        snap = self.telemetry.sample(self.env)
        self.track_nose_rate(snap)
        self.track_spool(snap)
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
            LANDING_BURN: self.run_landing_burn,
            TOUCHDOWN: self.run_touchdown,
        }[self.state]
        handler(snap)
        if self.prediction is not None:
            self.miss_jitter.update(self.prediction.position)
            self.env.miss_band = self.miss_jitter.band(self.cfg.NOISE_SIGMAS)
        if (self.cfg.KRPC_AP_DIAG and self.cfg.KRPC_AP_DIAG_BELOW_M > 0.0
                and self.state == COAST and not self.diag_restarted
                and snap.mean_altitude < self.cfg.KRPC_AP_DIAG_BELOW_M):
            self.diag_restarted = True
            try:
                self.autopilot.diagnostic_logging = False
                self.autopilot.diagnostic_logging = True
                self.log.event(snap.ut, "kRPC diagnostic record restarted")
            except Exception:                       # noqa: BLE001
                pass
        if (self.cfg.KRPC_CLEAR_LATCH_ON_THRUST and self.state == BOOSTBACK
                and self.flip_done and not self.latch_checked
                and snap.throttle > 0.5):
            self.latch_checked = True
            self.clear_krpc_latches(snap)
            self.aim(self.commanded_direction or snap.direction, snap)
        if self.state != COAST and any(self.trim_cmd):
            self.send_translation((0.0, 0.0, 0.0))
        if self.cfg.GEAR_TIMED:
            self.time_gear(snap)
        if self.cfg.DEPLOY_LAW:
            self.late_switch(snap)
        self.sweep_burn_curve(snap)
        if self.cfg.DIAG_DRAG:
            self.log_drag_diag(snap)
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

    def save_ap_diag(self, suffix=".ap.csv"):
        """kRPC's diagnostic CSV beside the log (``KRPC_AP_DIAG``)."""
        try:
            text = self.autopilot.diagnostic_log
            path = self.log.path + suffix
            with open(path, "w") as fh:
                fh.write(text)
            self.log.event(0.0, "kRPC diagnostic log: %d rows -> %s"
                           % (text.count("\n"), path))
        except Exception as exc:                    # noqa: BLE001
            self.log.event(0.0, "kRPC diagnostic log unreadable: %r" % exc)

    def record_drag(self, ut):
        """Leave the Cd*A curve this flight measured in ``dragcache``, for the
        ascent of the next stack carrying this booster to budget with."""
        curve = self.env.curve_snapshot()
        if not self.drag_key or len(curve) < 2:
            return
        try:
            dragcache.record(self.drag_key, [m for m, _ in curve],
                             [a for _, a in curve],
                             note=self.log.path.split("/")[-1])
            self.log.event(ut, "drag curve recorded (%s, %d bins, M%.2f-%.2f)"
                           % (self.drag_key, len(curve), curve[0][0],
                              curve[-1][0]))
        except OSError as exc:
            self.log.event(ut, "drag curve not recorded: %r" % exc)

    def shutdown(self, reason):
        """Hand control back to the player in a safe, predictable state."""
        ut = 0.0
        if self.cfg.KRPC_AP_DIAG:
            self.save_ap_diag()
        try:
            ut = self.conn.space_center.ut
            self.control.throttle = 0.0
            if self.cluster is not None:
                self.cluster.release()
            if self.cfg.GIMBAL_ROLL_OFF:
                set_gimbal_roll(self.vessel, True)
            if self.idle_gimbals is not None:
                self.idle_gimbals.update(True)      # freed for the player
            if self.fins is not None:
                self.fins.release("shutdown")
            if any(self.trim_cmd):
                self.send_translation((0.0, 0.0, 0.0))
            set_autopilot_engaged(self.autopilot, False)
            self.control.sas = True
        except Exception as exc:                     # game closed / vessel gone
            self.log.event(ut, "shutdown control error: %r" % exc)
        self.log.event(ut, "shutdown: %s" % reason)
        self.record_drag(ut)
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
               (" can=%.1f" % run.env.holdable_deg
                if run.env.holdable_deg is not None else ""))
            + attitude_fields(snap, run) + engine_fields(state, run)
            + (" tr=%+.2f/%+.2f/%+.2f" % run.trim_cmd
               if any(getattr(run, "trim_cmd", (0.0,))) else ""))


def engine_fields(state, run):
    """` eng=` each engine group's delivered thrust, a fraction of its own
    maximum, in lighting order -- only while the engines can burn and the
    cluster is split (a single group would just repeat `thr=`)."""
    cluster = run.cluster
    if (cluster is None or len(cluster.groups) < 2
            or state not in ("BOOSTBACK", "LANDING_BURN")):
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
    roll = roll_error(snap, getattr(run, "roll_cmd", None))
    rate = getattr(run, "nose_rate", None)
    spin = getattr(run, "roll_rate", None)
    inputs = getattr(snap, "inputs", None)
    crate = getattr(run, "cmd_rate", None)
    return (" cmd=%.1f err=%.1f cv=%.1f%s sat=%s%s%s%s%s"
            % (vec.angle_between(cmd, vec.scale(snap.velocity, -1.0)),
               run.pointing_error(snap),
               vec.angle_between(cmd, snap.position),
               " cw=%.1f" % crate if crate is not None else "",
               "%.2f" % snap.input_saturation
               if snap.input_saturation is not None else "-",
               " roll=%.0f" % roll if roll is not None else "",
               " w=%.1f" % rate if rate is not None else "",
               " r=%+.0f" % spin if spin is not None else "",
               " in=%+.2f/%+.2f/%+.2f" % inputs if inputs else ""))


def body_vector(snap, world):
    """``world`` in the vessel frame (x right, y nose, z bottom), or None.
    The quaternion's convention is checked as ``vessel_roof`` checks it."""
    if snap.rotation is None or len(snap.rotation) != 4:
        return None
    nose = vec.unit(snap.direction)
    for q in (snap.rotation, vec.quat_conjugate(snap.rotation)):
        if vec.norm(vec.sub(vec.quat_rotate(q, (0.0, 1.0, 0.0)), nose)) < 0.05:
            return vec.quat_rotate(vec.quat_conjugate(q), world)
    return None


def vessel_roof(snap):
    """The vessel's roof (its frame's -z; kRPC: x right, y nose, z bottom)
    in the reference frame, or None.  Which way the attitude quaternion
    applies is checked, not assumed: the convention that carries the body's
    +y onto the streamed nose is the one used."""
    if snap.rotation is None or len(snap.rotation) != 4:
        return None
    nose = vec.unit(snap.direction)
    for q in (snap.rotation, vec.quat_conjugate(snap.rotation)):
        if vec.norm(vec.sub(vec.quat_rotate(q, (0.0, 1.0, 0.0)), nose)) < 0.05:
            return vec.quat_rotate(q, (0.0, 0.0, -1.0))
    return None


def rotate_about(v, axis, angle):
    """``v`` rotated by ``angle`` (rad) about ``axis`` (Rodrigues).  The sense
    is whatever ``vec.cross`` gives, which is what ``roof_angle`` measures
    with, so the two agree in any frame handedness."""
    k = vec.unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    return vec.add(vec.add(vec.scale(v, c), vec.scale(vec.cross(k, v), s)),
                   vec.scale(k, vec.dot(k, v) * (1.0 - c)))


def roof_angle(snap, nose, up):
    """Signed angle (rad) about ``nose`` from ``up`` to the vehicle's roof,
    both perpendicular to the nose: ``rotate_about(up, nose, it)`` is the
    roof.  0 when the roof is unknowable."""
    roof = vessel_roof(snap)
    if roof is None:
        return 0.0
    k = vec.unit(nose)
    a = vec.project_out(up, k)
    b = vec.project_out(roof, k)
    if vec.norm(a) < 1e-6 or vec.norm(b) < 1e-6:
        return 0.0
    return math.atan2(vec.dot(k, vec.cross(a, b)), vec.dot(a, b))


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
    p = cli.parser("boosterland",
                   "Boostback and land a KSP booster on the KSC pad.")
    p.add_argument("--load", default=None, metavar="SAVE",
                   help="load this save and fly from the moment its physics "
                        "runs (the farm harness, quickfly.py)")
    p.add_argument("--start-ut", type=float, default=None, metavar="UT",
                   help="take the first tick at this game time, not as soon "
                        "as the process is up (the farm harness: the same "
                        "entry state whatever the interpreter's start-up)")
    p.add_argument("--load-timeout", type=float, default=90.0, metavar="S",
                   help="wall-seconds to wait for the loaded save to settle")
    return p.parse_args(argv)


def load_settled(args, poll=0.5):
    """``--load``: load the save and return a connection the moment the
    booster is flying.

    **In this process, not the harness's**: a harness that loaded the save
    and then started this one would have to stop the game meanwhile, and a
    pause lands a few frames after it is asked for -- 0.2-0.8 game-seconds
    at 4x, a different entry state every flight.  The load drops every
    connection while the scene is rebuilt, so this reconnects; a speed
    above 1 m/s is physics running, not the loading screen.
    """
    conn = cli.connect(args, "boosterland-loader")
    try:
        conn.space_center.load(args.load)
    finally:
        try:
            conn.close()
        except Exception:                   # noqa: BLE001 -- already gone
            pass
    deadline = time.monotonic() + args.load_timeout
    while time.monotonic() < deadline:
        time.sleep(poll)
        conn = None
        try:
            conn = cli.connect(args)
            vessel = conn.space_center.active_vessel
            if vessel.flight(vessel.orbit.body.reference_frame).speed > 1.0:
                return conn
        except Exception:                   # noqa: BLE001 -- still loading
            pass
        if conn is not None:
            try:
                conn.close()
            except Exception:               # noqa: BLE001
                pass
    raise RuntimeError("save %r never settled into a flyable vessel"
                       % args.load)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    cfg = apply_overrides(Config(), args.set)
    log = Logbook(cfg.LOG_DIR, cfg.LOG_INTERVAL_UT)
    run = None
    status = 0
    reason = "shutdown"
    try:
        conn = load_settled(args) if args.load else cli.connect(args)
        run = Autoland(conn, cfg, log)
        run.autostart = args.autostart
        if args.start_ut is not None:
            # **The entry state is a game time, not "once we are up".**
            # How long this process takes to start depends on the
            # interpreter and the machine (PyPy's first pass is ~3x
            # CPython's), and at 4x it moved the first tick 0.2-0.8
            # game-seconds -- a different booster every flight.
            now = conn.space_center.ut
            while now < args.start_ut:
                time.sleep(0.01)
                now = conn.space_center.ut
            log.event(now, "start held to UT %.2f (%+.2f s)"
                      % (args.start_ut, now - args.start_ut))
        if args.autostart:
            run.gui.hide_start()
            # **A harness may hand over a paused game** (``quickfly.py`` on
            # a simulated instance) so the booster does not fall while this
            # process starts: the first tick commands on the state the save
            # recorded, then time moves.
            run.tick()
            if conn.krpc.paused:
                conn.krpc.paused = False
                log.event(conn.space_center.ut,
                          "unpaused: engaged on the state the save recorded")
        run.run()
        reason = run.exit_reason or "complete"
    except KeyboardInterrupt:
        reason = "interrupted (SIGINT)"
    except Exception as exc:
        last = getattr(run, "last_snap", None)
        if vessel_gone(exc) and last is not None:
            # kRPC 0.7 raises on the first call after the vessel is gone.
            log.event(last.ut, "vessel lost: kRPC reports it destroyed")
            reason = ("destroyed %.0f m from pad, alt=%.0f spd=%.1f"
                      % (run.pad_distance(last), last.landing_height,
                         last.speed))
        else:
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
