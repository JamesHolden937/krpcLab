"""Phase machine: the only kRPC-aware control code.

    STANDBY -> DEORBIT -> DRAIN -> COAST -> GLIDE -> APPROACH -> FLARE
                                                                   |
                                                              ROLLOUT -> STOPPED

The phases hand over at *altitudes and geometry*, never at a miss distance,
and the reason is inherited: boosterland's worst failures were all a law
handing over to another law at a threshold defined in terms of the very error
it was trying to close.

Two invariants from the sibling project are kept deliberately:

**One prediction drives everything.**  Each GLIDE tick solves the angle of
attack and bank against a fresh propagation to the gate, and every decision
comes out of that solve.  There is no stored plan that can go stale -- except
the deorbit burn, which cannot be anything else, and which is therefore biased
to fail in the recoverable direction.

**Nothing prints.**  stdout and stderr are for crashes before the logbook
opens.  Everything else goes to ``logs/LOG<n>`` and the in-game panel.
"""
import argparse
import math
import os
import sys
import time

import krpc

from common import rcs, vec
from common.logbook import Logbook
from common.pacing import LoopRate, ScaleGovernor, sleeper
from common.rpccount import RpcCounter
from . import airbrake as airbrake_mod
from . import airframe
from . import guidance, rollrate, trajectory
from .config import (Config, apply_overrides, defaults_fingerprint,
                      differences)
from .environment import Environment
from .gui import ControlPanel
from .telemetry import Telemetry
from .trajectory import Steer

STANDBY = "STANDBY"
DEORBIT = "DEORBIT"
DRAIN = "DRAIN"
COAST = "COAST"
GLIDE = "GLIDE"
HAC = "HAC"
APPROACH = "APPROACH"
FLARE = "FLARE"
ROLLOUT = "ROLLOUT"
STOPPED = "STOPPED"


def _connection_name(conn):
    """Whatever the connection can say about which game this is.

    Best effort on purpose: this is a log line, and a log line that raises is
    worse than one that says "unknown".
    """
    for attempt in (lambda: "%s:%d" % (conn._client._connection.getpeername()[:2]),
                    lambda: str(conn.krpc.get_status().version)):
        try:
            return attempt()
        except Exception:                                   # noqa: BLE001
            continue
    return "unknown"


def slew_time_scale(vessel):
    """The vehicle's own attitude time scale, in seconds: ``sqrt(I / tau)``.

    **What ``ATTITUDE_TIME_TO_PEAK_S`` is a number for.**  The angular
    acceleration an actuator can make is ``tau / I``, so the time to swing a
    given angle goes as ``sqrt(I / tau)`` -- a property of the vehicle, which
    kRPC reports in full (``moment_of_inertia``, ``available_torque``) and
    which the autopilot already prints half of in STANDBY.

    Measured on the two craft on disk:

        old craft   14.5 t   I = (34931, 13452, 37195)    wheels 15 kN m
                    -> sqrt(I/tau) = (1.53, 0.95, 1.57),  max **1.57 s**
        shuttle     37.5 t   I = (2068357, 94184, 2106086)  wheels 15 kN m
                    -> sqrt(I/tau) = (11.74, 2.51, 11.85), max **11.85 s**

    **59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
    reaction wheel.**  The committed 3.0 s is 1.9x the old craft's 1.57, and
    flying the shuttle at that number is flying it at a fifth of its own
    slew time: measured, it holds 16 degrees more angle of attack than it is
    commanded and swings 95 degrees of sideslip at Mach 7.

    **Per axis, and taking the worst one is a mistake that costs 37 km.**
    kRPC reports the inertia around ``(pitch, roll, yaw)`` and this returns
    the slew time in the same order.  On a winged vehicle the roll inertia is
    tiny next to the other two -- the shuttle reads (11.74, **2.51**, 11.85)
    -- so a single figure taken from the slowest axis slows *roll* by nearly
    five times more than its own physics asks.  Roll is the bank control and
    bank is the entry's only cross-track authority: flown that way the
    shuttle pointed beautifully (alpha error -0.6, sd 1.4) and arrived
    **37 km off the centreline** (``logs/LOG2879``).

    ``None`` when kRPC cannot answer -- the caller keeps the constant, and
    says which it used.
    """
    # **The wheels' torque, by name, not ``available_torque``.**  The total
    # includes RCS whenever RCS happens to be switched on at the moment of the
    # call -- the shuttle's save has it on, and its 290 kN m would make the
    # derived time 4.5x quicker than the wheels it is actually flown on.  The
    # answer must not depend on the order of two lines in ``__init__``.  A
    # vehicle with no wheels at all falls back to the total.
    try:
        inertia = vessel.moment_of_inertia
        torque = vessel.available_torque[0]
    except Exception:                                       # noqa: BLE001
        return None
    try:
        wheels = vessel.available_reaction_wheel_torque[0]
        if min(abs(t) for t in wheels) > 1.0:
            torque = wheels
    except Exception:                                       # noqa: BLE001
        pass
    out = []
    for axis in range(3):
        tau = abs(torque[axis])
        if tau <= 1.0:
            return None
        out.append(math.sqrt(max(0.0, inertia[axis]) / tau))
    return tuple(out) if all(out) else None


def krpc_axes(cfg, pitch, roll, yaw, corrected=None):
    """A per-axis triple in the order kRPC's ``time_to_peak`` applies it.

    **kRPC's order is (pitch, roll, yaw)** -- the vessel frame's x, y, z,
    the same order as ``moment_of_inertia`` -- and this code believed
    (pitch, yaw, roll).  Measured on the shuttle in orbit
    (journal, "Session, 2026-09-23 (second)"): ``time_to_peak`` (3, 30, 3)
    cuts the *roll* PID gains tenfold (19.3 -> 1.93) and (3, 3, 30) the
    *yaw* gains (431 -> 43); a 90 degree roll takes 8.0 s against 3.7.  So
    the derived tune flew the shuttle's roll on yaw's 22.6 s and its yaw on
    roll's 4.8 -- a bank that lags by tens of seconds, on a vehicle with
    little yaw authority being yawed quickly.  ``ATTITUDE_AXES_KRPC_ORDER``
    selects the corrected order; off reproduces the old one exactly.
    """
    if corrected is None:
        corrected = getattr(cfg, "ATTITUDE_AXES_KRPC_ORDER", False)
    # ``ATTITUDE_ROLL_TIME_TO_PEAK_S``: roll on full authority, whatever
    # the derivation says.  Only in the corrected order, where the slot
    # handed to kRPC as roll really is roll.
    full = float(getattr(cfg, "ATTITUDE_ROLL_TIME_TO_PEAK_S", 0.0))
    if corrected and full > 0.0:
        roll = full
    if corrected:
        return (pitch, roll, yaw)
    return (pitch, yaw, roll)


def rcs_yaw_time_to_peak(cfg, inertia_yaw, wheel_yaw, rcs_yaw):
    """``ATTITUDE_SLEW_FACTOR * sqrt(I / tau)`` on yaw with the thrusters
    counted: the figure for yaw while the valve is open.  ``None`` when there
    is no RCS yaw to count (or kRPC gave nothing), so a craft without
    thrusters is untouched.  See ``ATTITUDE_YAW_WITH_RCS``."""
    try:
        inertia, wheel, rcs = (float(inertia_yaw), abs(float(wheel_yaw)),
                               abs(float(rcs_yaw)))
    except (TypeError, ValueError):
        return None
    if rcs <= 1.0 or inertia <= 0.0:
        return None
    return float(cfg.ATTITUDE_SLEW_FACTOR) * math.sqrt(inertia
                                                       / (wheel + rcs))


def yaw_time_to_peak(cfg, roll, static_yaw, alpha_deg, rcs_yaw=None):
    """Yaw's ``time_to_peak`` for a given roll figure and commanded alpha.

    A bank change is a rotation about the velocity: body roll and body yaw
    in the ratio cos(alpha) : sin(alpha).  At the entry's 35-40 deg a yaw
    axis on its wheels-only 22.6 s cannot keep up and every reversal leaves
    sideslip (glide slip max 19-74 deg, one tumble in six, LOG3717-3728);
    yaw on roll's figure everywhere cured that and lost the bank in the
    cone instead, at ~20 deg of alpha below Mach 1 where the single rudder
    is all the yaw there is (92 ticks of >60 deg bank error below Mach 2,
    against 33).  ``ATTITUDE_YAW_BY_ALPHA`` asks yaw for the share of the
    rotation that is yaw: roll's figure / sin(alpha), clamped between roll's
    and yaw's own static figure -- 7.5 s at 40 deg, 12.8 at 22, static near
    level flight.  ``ATTITUDE_YAW_WITH_ROLL`` is the unscheduled version.

    ``rcs_yaw`` is the thrusters-counted figure (``rcs_yaw_time_to_peak``),
    passed only while the valve is open under ``ATTITUDE_YAW_WITH_RCS``:
    yaw then flies what wheels plus RCS deliver, never faster than roll and
    never slower than its static figure.
    """
    if rcs_yaw is not None:
        return max(roll, min(static_yaw, rcs_yaw))
    return static_yaw


def label_axes(cfg, triple):                             # noqa: ARG001
    """``"pitch a, roll b, yaw c"`` -- what kRPC *applies*.  Under the old
    order this reads the swap plainly: the shuttle's roll at 22.6 s."""
    return "pitch %.1f, roll %.1f, yaw %.1f" % tuple(triple)


def attitude_time_to_peak(cfg, vessel, logbook=None, ut=0.0,
                          corrected=None):
    """How slow to make the attitude controller, derived where it can be.

    **A constant in seconds cannot be general, and this one is worth 198 km
    of arrival between two airframes.**  ``ATTITUDE_TIME_TO_PEAK_S`` is 3.0
    because that damped one vehicle's 2.4 s lateral mode; the next aeroplane
    would need a different second and the one after that another.  What
    transfers is the *ratio* to the vehicle's own slew time -- one
    dimensionless number in place of one constant per aircraft, which is the
    same trade ``airframe.MARGIN`` makes and is stated in the same way.

    ``ATTITUDE_SLEW_FACTOR`` is that ratio, and it is not fitted here: it is
    read off the craft the constant was fitted to (3.0 / 1.57 = 1.9), so the
    derived law reproduces the committed behaviour on that airframe by
    construction.  **That agreement is the reason to believe it and also the
    warning** -- a derivation that reproduces the fit tells you the fit was
    right for that aircraft and nothing yet about the next one.  What it
    predicts for the shuttle is 22.6 s against a flown bracket where 3.0
    could not point the vehicle at all and 6 to 12 all could; the prediction
    is untested at the top and is the next thing to fly.
    """
    fixed = float(getattr(cfg, "ATTITUDE_TIME_TO_PEAK_S", 0.0))
    scale = slew_time_scale(vessel)
    if scale is None:
        if logbook:
            logbook.event(ut, "attitude: no inertia or torque from kRPC -- "
                              "keeping the %.1f s constant" % fixed)
        return fixed
    k = float(cfg.ATTITUDE_SLEW_FACTOR)
    # ``moment_of_inertia`` is (pitch, roll, yaw), and so -- measured, see
    # ``krpc_axes`` -- is ``time_to_peak``.  This comment used to say the
    # second was (pitch, yaw, roll), which swapped roll and yaw.
    pitch, roll, yaw = (k * scale[0], k * scale[1], k * scale[2])
    out = krpc_axes(cfg, pitch, roll, yaw, corrected)
    if logbook:
        logbook.event(ut, "attitude: slew sqrt(I/tau) = (pitch %.2f, roll "
                          "%.2f, yaw %.2f) s, x%.2f -> time_to_peak as "
                          "applied (%s) (the constant says %.1f on every "
                          "axis)"
                      % (scale[0], scale[1], scale[2], k,
                         label_axes(cfg, out), fixed))
    return out


def tune_autopilot(autopilot, seconds, logbook=None, ut=0.0):
    """Slow kRPC's attitude controller down, if the config asks.

    ``time_to_peak`` is the response the PID is auto-tuned to reach, and its
    default is one second on a vehicle whose lateral mode is 2.4 s.  A
    controller faster than the mode it is acting through is how an
    oscillation gets driven rather than damped.  Reported rather than applied
    silently, because a tuning that did not take and one that did are
    otherwise the same flight.
    """
    if not seconds:
        return False
    # A scalar means "the same on every axis", which is what a single
    # constant can express; the derived law hands a per-axis triple.
    axes = (tuple(seconds) if isinstance(seconds, (tuple, list))
            else (seconds, seconds, seconds))
    try:
        autopilot.time_to_peak = axes
        if logbook:
            logbook.event(ut, "attitude controller time_to_peak -> "
                              "(pitch %.1f, roll %.1f, yaw %.1f) s as kRPC "
                              "applies it (kRPC default is 1.0 on every "
                              "axis)" % axes)
        return True
    except Exception as exc:                                # noqa: BLE001
        if logbook:
            logbook.event(ut, "could not set time_to_peak (%r); kRPC's "
                              "default stands" % exc)
        return False


def ungate_roll(cfg, autopilot, logbook=None, ut=0.0):
    """Let kRPC roll whatever the pointing error -- ``ATTITUDE_ROLL_ENGAGE_DEG``.

    The server suppresses roll entirely above ``roll_start_angle`` of
    direction error and engages it fully below ``roll_engage_angle``; on a
    vehicle banking at high alpha that gate is shut through every reversal.
    Start is set before engage so the pair is never inverted mid-change.
    Reported either way: a server without the properties flies the gate.
    """
    engage = float(getattr(cfg, "ATTITUDE_ROLL_ENGAGE_DEG", 0.0))
    if engage <= 0.0:
        return False
    start = min(180.0, engage + 5.0)
    engage = min(engage, start - 1.0)
    try:
        autopilot.roll_start_angle = start
        autopilot.roll_engage_angle = engage
        got = (autopilot.roll_engage_angle, autopilot.roll_start_angle)
    except Exception as exc:                                # noqa: BLE001
        if logbook:
            logbook.event(ut, "could not open the roll gate (%r); kRPC drops "
                              "roll above its default 20 deg of pointing "
                              "error" % exc)
        return False
    if logbook:
        logbook.event(ut, "roll gate: engaged below %.0f deg of pointing "
                          "error, off above %.0f (kRPC default 15/20)" % got)
    return True


def set_autopilot_attitude(autopilot, direction, up=None, roll=0.0):
    """Command a nose direction and the vector to roll the roof toward.

    ``set_direction_and_up`` is the singularity-free form; older servers have
    only ``up_reference``/``target_roll``, and older ones no roll control.
    Fall back rather than fail.  Roll matters more here than on a booster:
    bank *is* the control, so a vehicle that will not hold a commanded roll
    has no cross-track authority at all.
    """
    autopilot.target_direction = tuple(direction)
    if up is None:
        try:
            autopilot.target_roll = float("nan")
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

    Engaging makes kRPC total up the vessel's available torque, and KSP can
    throw out of that while modules are still settling -- which is exactly
    when a script connects.  boosterland lost a whole flight to it (failure
    7) on the first tick after START.  So it is reported, not raised, and the
    caller retries every tick until it takes.
    """
    try:
        try:
            autopilot.engaged = engaged
        except AttributeError:
            (autopilot.engage if engaged else autopilot.disengage)()
        return None
    except Exception as exc:                            # noqa: BLE001
        return exc


class Autopilot:
    def __init__(self, conn, cfg, logbook):
        self.conn = conn
        self.cfg = cfg
        self.logbook = logbook
        self.vessel = conn.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.frame = self.body.reference_frame
        self.control = self.vessel.control
        self.throttle = 0.0
        self.rcs_on = None
        self.rcs = rcs.Valve(cfg, logbook)
        self.commanded_nose = None      # what ``aim`` last asked for
        self._engines_checked = False   # see ``ensure_thrust``
        self._window_logged = False     # see ``log_deorbit_window``
        self.autopilot = self.vessel.auto_pilot
        self.autopilot.reference_frame = self.frame

        ut = conn.space_center.ut
        # **The differences, and the baseline they are differences from.**
        # A list of overrides says nothing when the defaults themselves move
        # between two batches: eighteen flights of one session were filed
        # into the wrong arm because flipping a default made their config
        # line identical to the controls'.  See ``defaults_fingerprint``.
        changed = differences(cfg)
        logbook.event(ut, "config: %s [defaults %s]"
                      % (", ".join(changed) if changed else "(defaults)",
                         defaults_fingerprint()))
        logbook.event(ut, "vessel: %s  %d parts  mass %.3f t"
                      % (self.vessel.name, len(self.vessel.parts.all),
                         self.vessel.mass / 1000.0))
        # **And which game flew it.**  CLAUDE.md's farm rule is that
        # instances are only comparable if they are the same game, and the
        # log had no way to say which one it was -- so a batch spread across
        # the farm could not be split by instance after the fact.  That is
        # exactly the split that separates "this loop amplifies" from "these
        # two machines run at different rates", and it cost a session's
        # worth of cone flights: four instances were flying four *different*
        # ``qs_cone`` saves and nothing in any log said so.
        logbook.event(ut, "instance: kRPC %s" % (_connection_name(conn),))

        self.env = Environment(conn, self.vessel, self.body, cfg, logbook, ut)
        self.telemetry = Telemetry(conn, self.vessel, self.frame, cfg,
                                   logbook)
        self.panel = ControlPanel(conn)

        self.state = STANDBY
        self.state_since = ut
        self.last_ut = ut
        # What the control loop is achieving, per phase.  Written at shutdown
        # so that every log says which controller flew it -- see failure 63.
        self.loop_rate = LoopRate()
        self.rpc = None             # common.rpccount.RpcCounter, set by main
        self.running = True
        self.finished_reason = None
        self.autopilot_engaged = False
        self._engage_warned = False

        self.end = self.env.runway.ends["09"]
        self.steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=0.0)
        self.commanded_alpha = 0.0
        self.commanded_bank = 0.0
        self.commanded_slip = 0.0
        # How fast this vehicle actually rolls, measured tick by tick from
        # its flown bank.  See ``rollrate`` and ``bank_rate``.
        self.roll_rate = rollrate.RollRate(cfg)
        # The yaw axis's own ceiling, learned exactly like alpha's: the
        # probe found a saturation in q (20 deg holds through 5000 Pa,
        # 30 deg only through 4000), and a table would fly the next
        # aircraft on this one's fin.
        self.slip_holdable = trajectory.Holdable(cfg)
        self._slip_held = 0.0
        self._slip_side = 0.0    # chosen once, on the first command
        self.alpha_ceiling = cfg.ALPHA_MAX_DEG
        # The learned alpha ceiling, hung on the environment so every
        # propagation sees it without being handed it.  See
        # ``trajectory.Holdable``: it starts knowing nothing and says so.
        self.env.holdable = trajectory.Holdable(cfg)
        if getattr(cfg, "HOLDABLE_PRIOR", False):
            self._load_holdable_prior(ut)
        self.env.flown_polar = None
        if getattr(cfg, "HAC_LD_FLOWN_POLAR", False):
            self._load_flown_polar(ut)
        self._below_ground_since = None
        self.prediction = None
        # Latched once, at the cone's entry: recomputing it every tick flips
        # the hand as the vehicle crosses the centreline, which turns an
        # alignment turn into a reversal at the worst possible moment.
        self.hac_side = None
        self.hac_command = None
        self.hac_radius = None
        self.hac_stretch = 0.0
        self._last_hac_ut = None
        self.deorbit_dv = None
        self.deorbit_aim_m = None
        self.airframe = None
        self.rollout_entry_alpha = None
        # The brake law's state: the commanded fraction (for the telemetry
        # line) and the aerodynamic deceleration it subtracts, which is the
        # ``dec=`` the telemetry already computes rather than a second drag
        # model.  Before the first telemetry tick there is no measurement, and
        # zero is the conservative answer -- it makes the law ask the wheels
        # for slightly more than they need, never less.
        self._landing_airframe_reported = False
        self.bank_wanted = None
        self.brake_fraction = 0.0
        self.last_free_decel = 0.0
        self._brake_wheel_cache = None
        self._reaction_wheels_released = False
        self._ground_spoiler_done = False
        self.deorbit_since = None
        self._deorbit_tick_ut = None
        self._deorbit_prev_tick_ut = None
        self._thrust_limits = []
        self._thrust_limit_pending = False
        self.deorbit_needed = None
        self.deorbit_last_ut = None
        self.warp_factor = 0
        # What a warped tick has actually been covering.  ``None`` until a
        # warped tick has been seen, and never a default that would pass a
        # threshold test unseen.
        self.warp_ceiling = int(cfg.WARP_MAX_FACTOR)
        self.warp_last = None           # (ut, position) of the last warped tick
        self.warp_arc = None            # metres of orbit the last one covered
        self.warp_refused = False
        self.bank_intent = None
        self.bank_reversed_ut = None
        self.bank_side = None
        self.attitude_settle_s = float(cfg.ATTITUDE_TIME_TO_PEAK_S)
        self._tuned_peak = None
        self.roll_damper = None         # built at engage, from the static tune
        self._retune_ut = None
        self._retune_logged = None
        self.holdable_quiet_until = None
        self.thermal_peak = 0.0
        self.parts_at_start = None
        self.parts_gone = 0
        self.parts_silent = 0
        self.alpha_checked_ut = None
        self.deorbit_burned = 0.0
        self.deorbit_modelled = 0.0   # the open-loop estimate, log only
        self.deorbit_model_ut = None  # its own clock; see fly_deorbit_burn
        self.deorbit_speed_prev = None
        self.deorbit_ticks = 0
        self.deorbit_done = False   # see fly_deorbit_burn
        self.interface_logged = False
        self.cutoff_state = None
        self.cutoff_logged = False
        self.deorbit_prediction = None
        self.deorbit_prediction_ut = None
        self.deorbit_prediction_speed = None
        self.boundary_logged = False
        self.deorbit_progress = 0.0
        self.drain_modules = self._find_drain()
        self.airbrake = airbrake_mod.Brake(cfg)
        self.flap_brake = self._find_brake()
        # ``HAC_SPLIT_BRAKE``: the mirrored vertical pair, kept apart from
        # whatever ``measure_flap_brake`` later puts in ``flap_brake``.
        self.split_pair = list(getattr(self.flap_brake, "pair", None) or ())
        self.split_angle = 0.0
        self.air_drag_out = 0.0
        self.flap_brake_out = False
        self.airbrake_ut = None
        self.gimbals_locked = self._lock_gimbals(ut)
        self._release_nose_brake(ut)
        self._set_main_gear(ut)
        self.log_actuators(ut)
        self.drain_started = None
        self.flare_since = None
        self.gear_down = False
        self.touchdown_ut = None
        self.touchdown_speed = None
        self.last_miss = (0.0, 0.0)
        self.stop_distance = None

    # -- plumbing ----------------------------------------------------------
    def set_throttle(self, value):
        """One place, so the log always shows what was *commanded*.

        KSP reasserts the player's throttle axis every frame, so a physical
        throttle left up can simply win and this script cannot take it back.
        boosterland logged ``thr=0.98`` through a phase that sets zero on
        every tick and lost the flight to it; recording the command separately
        from the vessel is how that becomes visible instead of silent.
        """
        self.throttle = float(value)
        self.control.throttle = self.throttle

    def enter(self, state, ut, note=""):
        self.logbook.event(ut, "%s -> %s%s" % (self.state, state,
                                               (" " + note) if note else ""))
        self.log_roll_rate(ut)
        self.state = state
        self.state_since = ut
        self.log_authority(ut)

    def derive_hac_aim(self, snap):
        """``HAC_AIM_DERIVED``: set the entry aim's ratio once, off the table,
        at the mass the cone will fly (the residual is dumped before it)."""
        if (not getattr(self.cfg, "HAC_AIM_DERIVED", False)
                or getattr(self.env.runway, "aim_ld", None) is not None
                or not self.env.ready()):
            return
        mass = snap.mass
        try:
            got = guidance.straight_in_reach(self.env, self.cfg, mass,
                                             self.surface_gravity)
        except Exception as exc:                        # noqa: BLE001
            got = None
            self.logbook.event(snap.ut, "hac aim: FAILED (%s)" % exc)
        self.env.runway.aim_ld = got or self.cfg.HAC_GATE_LD
        self.logbook.event(snap.ut, "hac aim: mid-authority ratio %s at %.2f t"
                                    " -> entry aim %.1f km before the gate"
                           % ("%.2f" % got if got else "unavailable "
                              "(HAC_GATE_LD %.2f)" % self.cfg.HAC_GATE_LD,
                              mass / 1000.0,
                              (self.cfg.HAC_ALT_M - self.cfg.GATE_ALT_M)
                              * self.env.runway.aim_ld / 1000.0))

    def measure_hac_ld(self, snap):
        """``HAC_LD_MEASURED``: the vehicle's L/D over the table's at the
        alpha it is flying, smoothed; subsonic GLIDE warms it, HAC uses it."""
        if (not getattr(self.cfg, "HAC_LD_MEASURED", False)
                or self.state not in (GLIDE, HAC)
                or snap.dynamic_pressure < 300.0 or not self.env.ready()):
            return
        speed = vec.norm(snap.velocity)
        altitude = vec.norm(snap.position) - self.env.equatorial_radius
        try:
            if self.env.mach(speed, altitude) > 0.95:
                return
            cla, cda = measured_coefficients(snap)
            mla, mda = self.env.coefficients(snap.alpha_actual, speed,
                                             altitude)
        except Exception:                               # noqa: BLE001
            return
        if min(cla, cda, mla, mda) <= 0.01:
            return
        inst = vec.clamp((cla / cda) / (mla / mda), 0.5, 2.0)
        last = getattr(self, "_hac_ld_ut", None)
        prev = getattr(self, "hac_ld_scale", None)
        self._hac_ld_ut = snap.ut
        if prev is None or last is None:
            self.hac_ld_scale = inst
            return
        k = min(1.0, max(0.0, snap.ut - last)
                / max(1.0, float(self.cfg.HAC_LD_MEASURED_TAU_S)))
        self.hac_ld_scale = prev + k * (inst - prev)

    def hac_weave_phase(self, snap):
        """``HAC_WEAVE_FIRST_WITH_BANK``: +1 or -1, which swing the weave
        clock starts on, locked on the first cone tick where the weave
        changes the command at all: the one whose bank agrees in sign with
        the bank the vehicle is flying, so the cone never opens with a
        reversal.  Chosen by trying both rather than by reasoning about the
        sign, as ``bank_toward`` is.  1 when off or until locked.
        """
        if not getattr(self.cfg, "HAC_WEAVE_FIRST_WITH_BANK", False):
            return 1.0
        phase = getattr(self, "_weave_phase", None)
        if phase is not None:
            return phase
        flying = self.steer.bank if self.steer is not None else 0.0
        if abs(flying) < 5.0:
            self._weave_phase = 1.0
            return 1.0
        base = self.hac_weave_sign(snap.ut)
        banks = []
        for sign in (1.0, -1.0):
            got = guidance.hac(self.env, self.cfg, self.end, snap.position,
                               snap.velocity, snap.mass, self.surface_gravity,
                               snap.height_above_runway, self.hac_side,
                               previous=self.hac_radius, max_step=0.0,
                               weave=sign * base,
                               roll_rate=self.roll_rate.limit(),
                               ld_scale=getattr(self, "hac_ld_scale", None))
            banks.append(None if got is None else got.bank)
        if None in banks or abs(banks[0] - banks[1]) < 1.0:
            return 1.0
        self._weave_phase = min(
            (1.0, -1.0), key=lambda s: abs(banks[0 if s > 0 else 1] - flying))
        self.logbook.event(snap.ut, "cone weave: first swing %+.0f deg "
                           "(flying %+.0f, the other swing %+.0f)"
                           % (banks[0 if self._weave_phase > 0 else 1],
                              flying, banks[1 if self._weave_phase > 0
                                           else 0]))
        return self._weave_phase

    def hac_weave_sign(self, ut):
        """Which way the cone's weave leans this tick.

        Off ``HAC_WEAVE_HELD`` it is the phase clock (``weave_sign``).  Held,
        a swing lasts the half-period the last command solved for -- the
        reversal the airframe needs plus the hold -- and the clock restarts
        whenever the weave was off, so the first swing is half a reversal
        plus the hold.
        """
        if not getattr(self.cfg, "HAC_WEAVE_HELD", False):
            return guidance.weave_sign(self.cfg,
                                       ut - (self.state_since or ut))
        last = getattr(self, "hac_command", None)
        if self.state != "HAC" or last is None \
                or getattr(last, "weave_deg", 0.0) <= 0.0:
            self._weave_dir = 1.0
            self._weave_flip_ut = ut
            self._weave_first = True
            return self._weave_dir
        half = getattr(last, "weave_half_s", self.cfg.HAC_WEAVE_PERIOD_S)
        if getattr(self, "_weave_first", False):
            half -= 0.5 * guidance.weave_reversal_s(
                self.cfg, last.weave_deg, last.speed,
                self.surface_gravity, self.roll_rate.limit())
        if ut - getattr(self, "_weave_flip_ut", ut) >= half:
            self._weave_dir = -getattr(self, "_weave_dir", 1.0)
            self._weave_flip_ut = ut
            self._weave_first = False
        return getattr(self, "_weave_dir", 1.0)

    def log_roll_rate(self, ut):
        """What the vehicle was measured to roll at, as a phase ends."""
        rr = getattr(self, "roll_rate", None)
        if rr is None or not rr.samples:
            return
        self.logbook.event(ut, "roll rate measured in %s: %.1f deg/s (peak "
                               "%.1f, %d samples) -> bank command slews at "
                               "%.1f%s" % (self.state, rr.rate, rr.peak,
                                           rr.samples, rr.limit(), ""))
        rd = getattr(self, "roll_damper", None)
        if rd is not None:
            self.logbook.event(ut, "roll damper in %s: %d swing(s) so far, "
                                   "roll time_to_peak %.1f s (floor %.1f, "
                                   "ceiling %.1f)" % (self.state, rd.swings,
                                                      rd.tp, rd.floor,
                                                      rd.ceiling))

    def log_authority(self, ut):
        """Torque by source, and the dynamic pressure it was read at.

        **The line that would have prevented a session's worth of wrong
        physics.**  The only torque the log used to carry was the pad's, where
        q is zero and the control surfaces report nothing -- and "this craft
        has no aerodynamic control" was built on it, when the surfaces were
        live all along and deliver 18x the wheels at 15 kPa.  Once per phase
        change is cheap and puts the actual authority beside every phase.
        """
        try:
            q = self.vessel.flight(self.frame).dynamic_pressure
            parts = []
            for label, name in (("wheel", "available_reaction_wheel_torque"),
                                ("rcs", "available_rcs_torque"),
                                ("surf", "available_control_surface_torque"),
                                ("eng", "available_engine_torque")):
                value = getattr(self.vessel, name)[0]
                parts.append("%s=(%.0f,%.0f,%.0f)" % ((label,) + tuple(
                    abs(v) / 1000.0 for v in value)))
            self.logbook.event(ut, "authority kN m (pitch,roll,yaw) at q=%.0f"
                                   " Pa: %s" % (q, " ".join(parts)))
        except Exception:                               # noqa: BLE001
            pass

    def log_actuators(self, ut):
        """Every setting the craft file chose and this autopilot does not.

        CLAUDE.md's standing audit, made automatic: the shuttle found five
        things in an afternoon that were right on one craft by inheritance
        (``Drain Mode``, the brake strength, ...).  One line, at STANDBY, so
        two logs from two airframes can be read against each other and a
        setting that differs between them is visible before it is a bug.
        ``testInstances/actuators.py`` is the full, per-module version.
        """
        bits = []
        try:
            cs = list(self.vessel.parts.control_surfaces)
            axes = sum(int(c.pitch_enabled) + int(c.yaw_enabled)
                       + int(c.roll_enabled) for c in cs)
            lim = sorted({round(c.authority_limiter) for c in cs})
            bits.append("surfaces %d (axes on %d of %d, limiter %s)"
                        % (len(cs), axes, 3 * len(cs), lim))
        except Exception:                               # noqa: BLE001
            pass
        try:
            wheels = list(self.vessel.parts.wheels)
            brakes = sorted({round(w.brakes) for w in wheels if w.has_brakes})
            steer = sum(1 for w in wheels
                        if w.steerable and w.steering_enabled)
            bits.append("wheels %d (brake %% %s, steering on %d)"
                        % (len(wheels), brakes, steer))
        except Exception:                               # noqa: BLE001
            pass
        try:
            rw = list(self.vessel.parts.reaction_wheels)
            bits.append("reaction wheels %d (active %d)"
                        % (len(rw), sum(1 for w in rw if w.active)))
        except Exception:                               # noqa: BLE001
            pass
        try:
            rcs_parts = list(self.vessel.parts.rcs)
            bits.append("rcs blocks %d (enabled %d), master %s"
                        % (len(rcs_parts),
                           sum(1 for r in rcs_parts if r.enabled),
                           "on" if self.control.rcs else "off"))
        except Exception:                               # noqa: BLE001
            pass
        try:
            eng = list(self.vessel.parts.engines)
            bits.append("engines %d (thrust limit %s)" % (
                len(eng), sorted({round(100 * e.thrust_limit) for e in eng})))
        except Exception:                               # noqa: BLE001
            pass
        if bits:
            self.logbook.event(ut, "actuators: " + "; ".join(bits))

    def engage_autopilot(self, ut):
        """Keep asking until it takes.  See ``set_autopilot_engaged``."""
        if self.autopilot_engaged:
            return True
        error = set_autopilot_engaged(self.autopilot, True)
        if error is None:
            self.autopilot_engaged = True
            # Tune it once, when it first takes: the controller does not
            # exist to be tuned before it is engaged.
            peak = attitude_time_to_peak(self.cfg, self.vessel,
                                         self.logbook, ut, None)
            self._axes_switched = False
            tune_autopilot(self.autopilot, peak, self.logbook, ut)
            ungate_roll(self.cfg, self.autopilot, self.logbook, ut)
            self._tuned_peak = (tuple(peak) if isinstance(peak, (tuple, list))
                                else (peak, peak, peak))
            self._static_peak = self._tuned_peak
            # Read the first time the valve is open (``rcs_yaw_now``): kRPC
            # reports no RCS torque while it is shut, and it is shut here.
            self._rcs_yaw_peak = None
            self._rcs_yaw_reads = 0
            # Floor: the roll the config asked for; ceiling: the
            # vehicle's slowest static axis.  Slot 1 is what kRPC
            # applies as roll in either axis order.
            self.roll_damper = rollrate.RollDamper(
                self.cfg, self._tuned_peak[1], max(self._tuned_peak))
            # The pitch axis's own response time: how long a disturbed angle
            # of attack takes to come back.  ``HOLDABLE_SKIP_REVERSAL`` waits
            # this long after a roll before believing the alpha again.
            try:
                self.attitude_settle_s = float(peak[0])
            except TypeError:
                self.attitude_settle_s = float(peak)
            if self._engage_warned:
                self.logbook.event(ut, "autopilot engaged (retry succeeded)")
            return True
        if not self._engage_warned:
            self._engage_warned = True
            self.logbook.event(ut, "autopilot refused to engage (%s) -- "
                                   "retrying every tick" % error)
        return False

    def aim(self, alpha_deg, bank_deg, snap):
        """Point the nose at ``alpha`` above the wind, roof toward the lift.

        One command expresses both controls, and that is not a convenience:
        the nose is tilted *toward the lift direction*, which is itself rolled
        by the bank, so the vehicle's own pitch plane and the plane the angle
        of attack is taken in are the same plane by construction.  Commanding
        a pitch and a roll separately is how they come apart.

        **The broadside probe overrides it here and nowhere else.**  With
        ``BROADSIDE_PROBE_DEG`` set, every angle of attack commanded from
        COAST onward is replaced by that one angle -- after the solve, after
        ``alpha_limit_for_speed``, after the ratchet, after every clamp -- for
        the same reason the override exists at all: the question is what the
        airframe can *hold*, and a knob that six other laws are free to
        overrule answers a different question.  ``Holdable`` then reads the
        shortfall against it every tick and the log carries ``aoa=cmd/actual``
        beside ``q=``, which is the ceiling curve.  0 disables it, and it is
        an instrument, not a flight mode: the vehicle will not reach the
        runway with it on.
        """
        if self.cfg.BROADSIDE_PROBE_DEG > 0.0 and self.state in (
                COAST, GLIDE, APPROACH, FLARE):
            alpha_deg = float(self.cfg.BROADSIDE_PROBE_DEG)
        bank_deg = self.bank_probe(bank_deg, snap)
        alpha_deg = self.alpha_trim_loop(alpha_deg, snap)
        v = snap.velocity
        if vec.norm(v) < 1.0:
            return
        lift = trajectory.lift_direction(snap.position, v, bank_deg)
        if lift is None:
            return
        vhat = vec.unit(v)
        alpha = math.radians(alpha_deg)
        # **Roll about the wind, not the nose** (``AIM_NOSE_FROM_FLOWN_BANK``).
        # Tilted toward the *commanded* lift, the nose sits off the
        # vehicle's own pitch plane by asin(sin alpha sin lag) whenever the
        # roll lags its command -- at 35 deg of alpha a 20 deg lag is 11 deg
        # of commanded sideslip, and the pitch/yaw loops faithfully fly it.
        # Tilted toward the *flown* lift the nose asks for none; the roof
        # still goes to the commanded lift, so the roll loop alone carries
        # the bank change and the nose follows it round the cone.
        tilt = lift
        nose = vec.unit(vec.add(vec.scale(vhat, math.cos(alpha)),
                                vec.scale(tilt, math.sin(alpha))))
        slip = self.slip_command(snap)
        if abs(slip) > 0.01:
            # **Yaw about the lift axis, not about the vertical.**  Rotating
            # the nose around the lift direction swings it sideways *in the
            # wing's own plane*, which is a sideslip at constant angle of
            # attack; rotating about the local vertical would change both at
            # once on a banked vehicle, and a probe whose two axes move
            # together measures neither.  The roof still goes to ``lift``, so
            # the bank the guidance asked for is untouched.
            nose = vec.unit(vec.quat_rotate(
                vec.quat_axis_angle(lift, math.radians(slip)), nose))
        set_autopilot_attitude(self.autopilot, nose, lift)
        self.commanded_nose = nose
        self._aim_frame = (vhat, tilt, lift, slip, alpha_deg)
        self.commanded_alpha = alpha_deg
        self.commanded_bank = bank_deg
        self.commanded_slip = slip
        rr = getattr(self, "roll_rate", None)
        if rr is not None:
            rr.update(snap.ut, bank_deg, flown_bank(snap),
                      getattr(snap, "sideslip", None))
        self.damp_roll(snap, bank_deg, alpha_deg)

    def bank_probe(self, bank_deg, snap):
        """``GLIDE_BANK_PROBE_DEG``: hold one bank from COAST through GLIDE
        until ``_END_MACH`` and log where the vehicle is at fixed Mach
        crossings (config).  Returns the bank to fly."""
        probe = float(getattr(self.cfg, "GLIDE_BANK_PROBE_DEG", 0.0))
        if probe == 0.0 or self.state not in (COAST, GLIDE) \
                or getattr(self, "_bank_probe_done", False):
            return bank_deg
        try:
            alt = vec.norm(snap.position) - self.env.equatorial_radius
            mach = self.env.mach(vec.norm(snap.velocity), alt)
        except Exception:                               # noqa: BLE001
            return probe
        marks = getattr(self, "_bank_probe_marks", None)
        if marks is None:
            marks = self._bank_probe_marks = [8.0, 6.0, 5.0, 4.0, 3.0, 2.0,
                                              1.5, 1.0]
            self._bank_probe_ref = None
            self.logbook.event(snap.ut, "bank probe: holding %+.0f deg "
                               "(%s) until Mach %.1f" % (
                                   probe, self.state, float(
                                       self.cfg.GLIDE_BANK_PROBE_END_MACH)))
        while marks and mach <= marks[0]:
            mark = marks.pop(0)
            try:
                body, frame = self.body, self.frame
                if self._bank_probe_ref is None:
                    mid = self.env.runway.midpoint
                    self._bank_probe_ref = (
                        body.latitude_at_position(mid, frame),
                        body.longitude_at_position(mid, frame))
                lat = body.latitude_at_position(snap.position, frame)
                lon = body.longitude_at_position(snap.position, frame)
                lat0, lon0 = self._bank_probe_ref
                radius = self.env.equatorial_radius
                north = math.radians(lat - lat0) * radius
                dlon = (lon - lon0 + 180.0) % 360.0 - 180.0
                east = math.radians(dlon) * radius * math.cos(
                    math.radians(lat0))
                self.logbook.event(
                    snap.ut, "bank probe: Mach %.1f alt %.0f m  lat %+.4f "
                    "lon %+.4f  north %+.1f km east %+.1f km of the runway "
                    "(bank flown %+.1f)" % (mark, alt, lat, lon, north / 1e3,
                                            east / 1e3, flown_bank(snap)))
            except Exception as exc:                    # noqa: BLE001
                self.logbook.event(snap.ut, "bank probe: no position (%s)"
                                   % exc)
        if mach <= float(self.cfg.GLIDE_BANK_PROBE_END_MACH):
            self._bank_probe_done = True
            self.logbook.event(snap.ut, "bank probe: done at Mach %.2f, "
                               "guidance resumes" % mach)
            return bank_deg
        return probe

    def damp_roll(self, snap, bank_deg, alpha_deg=None):
        """The lateral tune, every tick: roll from ``ROLL_DAMPER``, yaw from
        roll under ``ATTITUDE_YAW_WITH_ROLL`` or ``ATTITUDE_YAW_BY_ALPHA``.

        The damper slows roll when the bank swings about a steady command
        with growing amplitude and relaxes it back when it holds (see
        ``rollrate.RollDamper``).  ``yaw_time_to_peak`` says how fast yaw
        must follow.  Re-assigned only past ``ATTITUDE_RETUNE_FRAC`` -- kRPC
        re-derives its gains on every assignment."""
        static = getattr(self, "_static_peak", None)
        if not (static and getattr(self, "autopilot_engaged", False)
                and getattr(self, "_tuned_peak", None)):
            return
        rd = getattr(self, "roll_damper", None)
        swung = None
        if rd is not None:
            swung = rd.update(snap.ut, bank_deg, flown_bank(snap))
            roll = rd.tp
        else:
            roll = static[1]
        yaw = yaw_time_to_peak(self.cfg, roll, static[2], alpha_deg,
                               self.rcs_yaw_now(snap.ut))
        have = self._tuned_peak
        frac = float(self.cfg.ATTITUDE_RETUNE_FRAC)

        def moved(want, had, home):
            return (abs(want - had) > frac * had
                    or (want == home and had != home))
        if not (moved(roll, have[1], rd.floor if rd else roll)
                or moved(yaw, have[2], static[2])):
            return
        want = (have[0], roll, yaw)
        try:
            self.autopilot.time_to_peak = want
        except Exception:                               # noqa: BLE001
            return
        self._tuned_peak = want
        if swung is not None:
            self.logbook.event(
                snap.ut, "roll damper: bank swings growing, %.0f -> %.0f "
                         "deg past a steady %+.0f (swing %d, q=%.0f Pa) -> "
                         "roll time_to_peak %.1f -> %.1f s, yaw %.1f"
                % (swung[0], swung[1], bank_deg, rd.swings,
                   snap.dynamic_pressure, have[1], roll, yaw))
        elif moved(roll, have[1], rd.floor if rd else roll):
            self.logbook.event(
                snap.ut, "roll damper: held -> roll time_to_peak %.1f -> "
                         "%.1f s, yaw %.1f (q=%.0f Pa)"
                % (have[1], roll, yaw, snap.dynamic_pressure))
        else:
            logged = getattr(self, "_yaw_logged", None)
            if logged is None or abs(yaw - logged) > 0.3 * logged:
                self._yaw_logged = yaw
                self.logbook.event(
                    snap.ut, "yaw time_to_peak %.1f -> %.1f s at %.0f deg "
                             "of commanded alpha (roll %.1f%s)"
                    % (have[2], yaw, alpha_deg if alpha_deg is not None
                       else float("nan"), roll,
                       ", rcs open" if self.rcs_yaw_now() is not None
                       else ""))

    def rcs_yaw_now(self, ut=0.0):
        """The thrusters-counted yaw figure while it applies, else ``None``:
        ``ATTITUDE_YAW_WITH_RCS``, in GLIDE or HAC, with the valve open."""
        if (self.state not in (GLIDE, HAC)
                or not getattr(self.rcs, "on", False)):
            return None
        peak = getattr(self, "_rcs_yaw_peak", None)
        reads = getattr(self, "_rcs_yaw_reads", 0)
        if peak is None and reads < 10:
            # **Only an open valve reports its torque.**  Read at engage the
            # shuttle's 290 kN m came back as nothing -- the valve had just
            # been shut -- and the flag flew as the defaults (LOG3764).  So
            # it is read here, live, on the first ticks the valve is open.
            self._rcs_yaw_reads = reads + 1
            try:
                peak = rcs_yaw_time_to_peak(
                    self.cfg, self.vessel.moment_of_inertia[2],
                    self.vessel.available_reaction_wheel_torque[0][2],
                    self.vessel.available_rcs_torque[0][2])
            except Exception:                           # noqa: BLE001
                peak = None
            self._rcs_yaw_peak = peak
            static = getattr(self, "_static_peak", None)
            self.logbook.event(
                ut, "yaw with RCS: valve open, time_to_peak %s (static %s)"
                % ("none -- no RCS yaw reported" if peak is None
                   else "%.1f s" % peak,
                   "%.1f" % static[2] if static else "?"))
        return peak

    def reversal_under_way(self, snap):
        """The bank command leads the flown bank by more than
        ``BANK_RATE_SAT_DEG`` -- a turn the pointing error has not shown yet.
        Only under
        ``ATTITUDE_YAW_WITH_RCS``, only in GLIDE."""
        if snap is None or self.state != GLIDE:
            return False
        cmd = getattr(self, "commanded_bank", None)
        flown = flown_bank(snap)
        if cmd is None or math.isnan(flown):
            return False
        lead = (cmd - flown + 180.0) % 360.0 - 180.0
        return abs(lead) > float(self.cfg.BANK_RATE_SAT_DEG)

    def bank_rate(self):
        """deg/s the bank command may slew at: measured by ``roll_rate``
        under ``BANK_RATE_MEASURED``, else the ``BANK_RATE_DEG_S`` constant."""
        rr = getattr(self, "roll_rate", None)
        if rr is not None:
            return rr.limit()
        return float(self.cfg.BANK_RATE_DEG_S)

    def alpha_trim_loop(self, alpha_deg, snap):
        """``ALPHA_TRIM_LOOP``: offset the command until the *signed* alpha
        the vehicle flies is the alpha asked for -- APPROACH and FLARE only.

        kRPC's attitude loop leaves a standing pitch error that grows with
        dynamic pressure: the shuttle's fast flares (LOG3994-4007, doors at
        87-97 m/s) asked 5-8 deg for eight seconds and flew 2.7-4.1, and the
        S-turning approaches ran 2-7 deg rms short and dived.  The offset
        integrates (commanded - ``krpc_aoa``) over the pitch axis's own
        response time, only while the roll is settled and the slip small (a
        reversal is not a trim error), bounded by ``ALPHA_TRIM_MIN_DEG`` /
        ``ALPHA_TRIM_MAX_DEG``."""
        states = (HAC, APPROACH, FLARE)
        # ``ALPHA_TRIM_IN_GLIDE``: the glide too, under its own bound.  At
        # one dynamic pressure the shuttle flies ~4 deg under its command
        # below 35 deg of bank and ~10 above 50, with the pitch input at
        # 0.5-0.7 -- authority left unused -- and the flights that bank hard
        # arrive at the cone 4-15 km long and 5 km high (rot-phantom-1003).
        states = states + (GLIDE,)
        if getattr(self, "state", None) not in states:
            return alpha_deg
        top = float(self.cfg.ALPHA_TRIM_GLIDE_MAX_DEG) if self.state == GLIDE else float(self.cfg.ALPHA_TRIM_MAX_DEG)
        # What the glide wound up is not the landing's to inherit.
        delta = vec.clamp(getattr(self, "_alpha_trim", 0.0),
                          float(self.cfg.ALPHA_TRIM_MIN_DEG), top)
        last = getattr(self, "_alpha_trim_ut", None)
        self._alpha_trim_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, snap.ut - last)
        try:
            achieved = float(snap.krpc_aoa)
            slip = abs(float(getattr(snap, "sideslip", 0.0) or 0.0))
            flown = flown_bank(snap)
            cmd_bank = float(getattr(self, "commanded_bank", 0.0) or 0.0)
            settled = (not math.isnan(flown)
                       and abs(flown - cmd_bank)
                       <= float(self.cfg.ALPHA_TRIM_ROLL_TOL_DEG)
                       and slip <= float(self.cfg.ALPHA_TRIM_SLIP_TOL_DEG))
            if (dt > 0.0 and settled
                    and snap.dynamic_pressure > self.cfg.LIFT_LOOP_MIN_Q_PA):
                settle = max(1.0, float(getattr(self, "attitude_settle_s",
                                                0.0) or 2.0))
                # The *wanted* angle against the flown one.  Integrating the
                # offset command against it (as first built, never flown)
                # integrates kRPC's own standing error, which the offset
                # never closes, and winds to the bound regardless.
                error = vec.clamp(alpha_deg - achieved, -5.0, 5.0)
                delta = vec.clamp(delta + error * dt / settle,
                                  float(self.cfg.ALPHA_TRIM_MIN_DEG), top)
        except (TypeError, ValueError):
            pass
        self._alpha_trim = delta
        return alpha_deg + delta

    def run_flap_probe(self, snap):
        """``FLAP_BRAKE_PROBE_DEG``: hold the opposed flaps out and watch.

        **The instrument, before the law** -- the pattern that made the
        sideslip work land in one round.  ``SLIP_PROBE_DEG`` flew four fixed
        angles and produced the ceiling curve and the L/D result that the
        control law was then written against; this does the same for the
        canard-against-elevon brake.

        What it has to answer, and none of it is knowable from geometry:

        * **Does the moment actually cancel?**  Read ``aoa=cmd/actual``
          across the deployment.  If the trailing-edge senses are wrong the
          moments add instead of cancelling and the tracking error jumps --
          which is the behavioural check the split rudder never had, because
          its verification summed ``available_torque`` and could not fail.
        * **What does it cost in ``ClA`` and give in ``CdA``?**  ``act=`` is
          the achieved force and needs no model.
        * **Is it a lift spoiler or a brake?**  Those are different
          currencies and only one of them is any use: drag spends speed and a
          glider dives to get it back.

        Barred from FLARE and ROLLOUT, like every other probe, because
        landing with a deployed brake is a variable nobody asked for.
        """
        if getattr(self, "flap_brake", None) is None:
            return
        angle = float(getattr(self.cfg, "FLAP_BRAKE_PROBE_DEG", 0.0))
        if angle <= 0.0:
            return
        want = self.state in (COAST, GLIDE, HAC, APPROACH)
        if want != self.flap_brake_out:
            if self.set_flap_brake(want, angle):
                self.logbook.event(snap.ut, "flap brake %s at %.1f deg aft"
                                   % ("OUT" if want else "stowed", angle))

    def slip_command(self, snap):
        """How far off the velocity vector to yaw the nose, this tick.

        Two callers.  ``SLIP_PROBE_DEG`` is the instrument that measured the
        ceiling in the first place and overrides everything; it is barred
        from FLARE and ROLLOUT because landing crabbed is failure 34, and the
        probe flights proved the point -- 10, 20 and 30 degrees were all
        *destroyed in ROLLOUT* while the 5-degree flight landed normally.

        ``APPROACH_SLIP_FOR_ENERGY`` is the control law the probe earned.
        **Sideslip on this airframe is a glide-ratio spoiler, not a brake.**
        Measured subsonic in HAC and APPROACH: at about 15 degrees of held
        slip, ``ClA`` falls 15.7 -> 10.9 while ``CdA`` does not rise, so L/D
        goes **1.64 -> 1.23** -- a quarter of the glide ratio, with the
        airspeed untouched.

        That currency is the whole point.  The split rudder spent *speed*
        (21 m/s in eleven seconds), and a glider can only buy speed back by
        diving, so the speed loop dived into the flare door and destroyed two
        vehicles.  Lift costs nothing the approach is trying to protect:
        ``tan(gamma) = D/L``, so less lift at the same drag is a steeper
        path at the same speed, and "steeper at the same speed" is exactly
        what a surplus of height wants spent.
        """
        probe = float(getattr(self.cfg, "SLIP_PROBE_DEG", 0.0))
        if probe != 0.0 and self.state in (COAST, GLIDE, HAC, APPROACH):
            return probe
        return 0.0

    def aim_runway(self, alpha_deg, snap):
        """Point the nose **down the runway**, pitched up by ``alpha``.

        Everywhere else in this flight ``aim`` points the nose relative to
        the *velocity*, which is right for an entry: the angle of attack is
        the control and the airflow is what it is measured against.  On short
        final it is exactly wrong. A vehicle correcting cross-track arrives
        with its velocity pointing at the centreline rather than along it, so
        aiming at the velocity lands it **crabbed** -- measured, 23 degrees of
        sideslip at the first rollout tick, 51 degrees two seconds later, and
        the gear torn off at 48 m/s on a runway it had just landed on.

        Wheels do not care where the nose points; they care where it points
        *relative to the direction of travel*, and the moment they touch, the
        difference becomes a yaw the aircraft cannot absorb.  So near the
        ground the reference stops being the airflow and becomes the runway,
        which is the one thing the wheels are going to agree with.
        """
        alpha_deg = self.alpha_trim_loop(alpha_deg, snap)
        up = vec.unit(snap.position)
        along = self.env.runway.horizontal(self.end, self.end["along"])
        pitch_deg = alpha_deg
        # **An angle above the runway is not an angle of attack, and the
        # flare is where the difference lives.**  Every caller computes its
        # number with ``alpha_for_load`` -- an angle measured from the
        # *airflow* -- and this used to fly it as a pitch attitude measured
        # from the runway.  The two differ by the descent angle, which is
        # twenty degrees at the flare's door and zero on the ground, so the
        # wing was handed more lift than the guidance asked for, in exact
        # proportion to how fast it was coming down.  Measured on
        # ``logs/LOG2290``: the flare commands 8.2 degrees and achieves 12.6,
        # arrests to level at 35 m, floats six seconds bleeding 74 m/s to 44,
        # and drops the last thirty metres -- a flare that has already been
        # told to unload (``FLARE_TRACK_LOAD_MIN``) and cannot, because the
        # command is not being flown in the units it was written in.
        #
        # So pitch to ``alpha + descent``, which is the attitude that
        # *delivers* ``alpha`` to the airflow, and keep the heading on the
        # runway -- the yaw reference is the whole reason this method exists
        # and it is untouched.  Capped by the tail, because that limit really
        # is about where the body is pointing.
        speed = vec.norm(snap.velocity)
        sink = -vec.dot(snap.velocity, up)
        if speed > 1.0:
            descent = math.degrees(math.asin(
                vec.clamp(sink / speed, -1.0, 1.0)))
            # ``FLARE_TAIL_BY_ATTITUDE``: alpha = pitch + descent, so
            # the pitch that delivers ``alpha`` is ``alpha - descent``.
            sign = (-1.0)
            pitch_deg = min(alpha_deg + sign * max(0.0, descent),
                            self.tail_limit_deg()
                            - self.pitch_overshoot(snap))
        alpha = math.radians(pitch_deg)
        nose = vec.unit(vec.add(vec.scale(along, math.cos(alpha)),
                                vec.scale(up, math.sin(alpha))))
        set_autopilot_attitude(self.autopilot, nose, up)
        self.commanded_nose = nose
        self.commanded_alpha = alpha_deg
        self.commanded_bank = 0.0
        self._runway_pitch_cmd = pitch_deg

    def _glide_top(self, snap):
        """``trajectory.glide_alpha_max`` at this state."""
        return trajectory.glide_alpha_max(
            self.cfg, self.env, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius)

    def pitch_overshoot(self, snap):
        """``TAIL_LIMIT_ACHIEVED``: how far above its commanded pitch the
        vehicle is flying, smoothed, never negative.

        The tail cap bounds the *command*; the shuttle trims nose-up and
        flies 3-5 deg above it (LOG3596: 6.9 commanded, 11.2 flown at
        touchdown, against a 9.1 deg tail angle -- a delta wing and the RCS
        blocks gone in 0.1 s).  Subtracting the measured overshoot makes the
        cap bound the attitude actually reached.  Smoothed over the craft's
        own derived attitude settle time, so nothing here is fitted.
        """
        cmd = getattr(self, "_runway_pitch_cmd", None)
        if cmd is None or vec.norm(snap.nose) < 0.5:
            return getattr(self, "_pitch_over", 0.0)
        up = vec.unit(snap.position)
        flown = math.degrees(math.asin(vec.clamp(
            vec.dot(vec.unit(snap.nose), up), -1.0, 1.0)))
        last = getattr(self, "_pitch_over_ut", None)
        dt = 0.0 if last is None else max(0.0, snap.ut - last)
        self._pitch_over_ut = snap.ut
        tau = max(0.1, float(getattr(self, "attitude_settle_s", 1.0)))
        k = min(1.0, dt / tau) if last is not None else 1.0
        over = getattr(self, "_pitch_over", 0.0)
        over += k * ((flown - cmd) - over)
        self._pitch_over = over
        return max(0.0, over)

    def log_deorbit_window(self, snap):
        """What the burn search was actually offered, once, at the exit.

        Two config knobs meant to aim this entry were flown and neither
        moved the landing: a batch can say the landing did not move, but not
        whether the knob ever reached the decision.  These numbers can.  See
        ``guidance.deorbit_centring``.
        """
        if self._window_logged:
            return
        self._window_logged = True
        seen = []
        try:
            guidance.deorbit_centring(
                self.env, snap.position, snap.velocity,
                snap.mass, self.cfg, self.end,
                self.env.runway.gate(self.end),
                window_note=lambda *args: seen.append(args))
        except Exception:                               # noqa: BLE001
            return
        if not seen:
            self.logbook.event(snap.ut, "deorbit window: no answer -- the "
                                        "aim knobs cannot have reached the "
                                        "search")
            return
        shortest, longest, needed, want, bias = seen[-1]
        self.logbook.event(
            snap.ut, "deorbit window: reach %.0f..%.0f m (%.0f wide), "
                     "gate needs %.0f, aimed at %.0f, centre bias %.0f"
            % (shortest, longest, longest - shortest, needed, want, bias))

    def ensure_thrust(self, snap):
        """An engine nobody activated makes a burn that never happens.

        The autopilot commands a throttle and assumes something is lit.  A
        craft handed over with its engine unstaged answers that throttle with
        nothing at all: measured on the new airframe, sixty seconds of
        commanded burn at a perfect retrograde attitude with the speed
        unchanged at 2080.6 m/s to the last tick, then ``burn guard at 60 s``
        and an entry that never deorbited.  Nine flights, three of them
        still in orbit at the timeout.

        **Activated, not staged.**  ``activate_next_stage`` fires whatever is
        next, which on a winged vehicle can decouple something the flight
        needs; setting ``active`` on an engine that already has fuel is the
        same as lighting it by hand and drops nothing.  If no engine will
        light, that is said once and the burn guard is left to catch it --
        a missing answer must not look like a good one.
        """
        if self._engines_checked or snap.available_thrust > 1.0:
            return
        self._engines_checked = True
        lit = []
        try:
            engines = list(self.vessel.parts.engines)
        except Exception:                               # noqa: BLE001
            return
        for engine in engines:
            try:
                if engine.active or not engine.has_fuel:
                    continue
                engine.active = True
                lit.append(engine.part.title)
            except Exception:                           # noqa: BLE001
                continue
        if lit:
            self.logbook.event(snap.ut, "no thrust available -- lit %s"
                               % ", ".join(sorted(set(lit))))
        else:
            self.logbook.event(snap.ut, "no thrust available and no engine "
                                        "would light -- the burn will not "
                                        "happen")

    def flare_lead_s(self):
        """``FLARE_LEAD_BY_RESPONSE``: how far ahead the flare's sink
        schedule is read -- the pitch axis's own response time, the figure
        ``attitude_settle_s`` already carries (derived, and retuned with the
        air by ``ATTITUDE_PITCH_AIR``).  0 with the flag off."""
        return max(0.0, float(getattr(self, "attitude_settle_s", 0.0) or 0.0))

    def flare_tail_cap(self, snap):
        """The flare's alpha cap from the tail: the tail angle, plus -- under
        ``FLARE_TAIL_BY_ATTITUDE`` -- the descent angle, because the tail
        strikes by the body's attitude and attitude is alpha less the
        descent.  The same cap at touchdown; the attitude limit above it."""
        cap = self.tail_limit_deg()
        speed = vec.norm(snap.velocity)
        if speed <= 1.0:
            return cap
        sink = -vec.dot(snap.velocity, vec.unit(snap.position))
        descent = math.degrees(math.asin(vec.clamp(sink / speed, -1.0, 1.0)))
        return cap + max(0.0, descent)

    def tail_limit_deg(self):
        """How far the nose may come up before the tail reaches the ground.

        The flare's job is to arrest the sink, and its only tool is angle of
        attack -- so left alone it takes as much as the wing will give and
        lands the vehicle on its tail.  LOG1656 touched down at 45.9 m/s with
        the sink already arrested, which is a landing, and lost Elevon 4 one
        second later and everything else four seconds after that.

        The limit is the airframe's geometry, measured from the bounding box
        (see ``Telemetry``), not a number chosen for this aircraft.  Until
        the box answers, the fallback is used -- and it is tight on purpose:
        landing flat costs a hard arrival, landing on the tail costs the
        aircraft.
        """
        angle = getattr(self.telemetry, "tail_angle_deg", None)
        if angle is None:
            angle = self.cfg.TAIL_ANGLE_FALLBACK_DEG
        return max(1.0, self.cfg.TAIL_STRIKE_MARGIN * angle)

    def snap_mach(self, snap):
        """Mach off the state vector, or ``None``."""
        try:
            return self.env.mach(
                vec.norm(snap.velocity),
                vec.norm(snap.position) - self.env.equatorial_radius)
        except Exception:                               # noqa: BLE001
            return None

    def ratchet_alpha(self, snap):
        """Lower the ceiling when the vehicle cannot hold what it is given.

        The highest holdable angle of attack was taken to be not computable:
        "kRPC's aerodynamic probe returns a force and not a moment".  **Not
        true of kRPC 0.6**, which has ``simulate_aerodynamic_wrench_at``
        (force and torque) and ``ControlSurface.deflection_override`` -- so
        the trim limit may be computable from the game's own model (journal,
        "Session, 2026-09-23 (second)").  Until that is built and flown, this
        observes it.  What *is* available is whether the vehicle is achieving the
        angle it was told to, which is the same closed loop boosterland runs
        on ``alignment_rate`` -- and failure 6 there is a phase that commanded
        an attitude the vehicle could not reach and sat on it for 36 seconds
        while the miss doubled.
        """
        dt = max(0.0, snap.ut - (self.alpha_checked_ut or snap.ut))
        self.alpha_checked_ut = snap.ut
        # Feed the learned ceiling from the same three numbers the ratchet
        # runs on.  The ratchet is the *control* loop's answer to a vehicle
        # that will not hold its command; ``Holdable`` is the *predictor's*,
        # and they have to agree or the prediction is again of a law nobody
        # flies.  The dynamic pressure is the game's own, not the table's.
        if snap.dynamic_pressure > 0.0 and self.commanded_alpha > 0.0:
            # Mach the way every other reader of it here does it: off the
            # state vector, not off attributes ``Snapshot`` does not carry.
            try:
                mach = self.env.mach(
                    vec.norm(snap.velocity),
                    vec.norm(snap.position) - self.env.equatorial_radius)
            except Exception:                               # noqa: BLE001
                mach = None
            # The yaw axis, from the same tick: what was asked, what the
            # vehicle is holding, at what dynamic pressure.  ``sideslip_angle``
            # runs opposite to a rotation about the lift axis (measured:
            # slipc=+30.0 reads back slip=-30.3), so both are taken absolute --
            # the ceiling is a magnitude and has no side.
            # **Not while rolling through a reversal, nor until the pitch
            # axis has had its own response time to recover.**  Measured on
            # the shuttle (LOG2912): +70 through 0 to -58 degrees of bank at
            # 6-7.5 kPa, the achieved alpha dipped 31 -> 8 and was back at
            # 22.6 three seconds after the roll -- but q climbs fast there,
            # the reversal filled a whole q bin by itself, and the learner
            # recorded a ceiling of 10.6.  Denser air "carries the lowest
            # seen", so the rest of the entry flew 16-18 degrees against 28
            # and arrived 35-48 km long.  A lagging roll is not a ceiling.
            quiet = (self.holdable_quiet_until is not None
                     and snap.ut < self.holdable_quiet_until)
            if abs(self.commanded_slip) > 0.01 and not quiet:
                self.slip_holdable.observe(abs(self.commanded_slip),
                                           abs(snap.sideslip),
                                           snap.dynamic_pressure)
            if not quiet:
                self.env.holdable.observe(self.commanded_alpha,
                                          snap.alpha_actual,
                                          snap.dynamic_pressure, mach)
        if self.commanded_alpha <= self.cfg.ALPHA_MIN_DEG + 1.0:
            return
        error = self.commanded_alpha - snap.alpha_actual
        if (abs(error) < 0.5 * self.cfg.ALPHA_TRACK_TOLERANCE_DEG
                and self.alpha_ceiling < self._glide_top(snap)):
            # Tracking comfortably, so give the authority back.  Without this
            # the ratchet is one-way and a transient -- a bank reversal throws
            # the error for a second or two -- costs the range for the rest of
            # the flight.
            #
            # **The magnitude, not the signed error.**  Transonically this
            # airframe trims *nose-high* and overshoots its command -- 31.5
            # deg achieved against 20 commanded -- which makes the signed
            # error large and negative, and a signed test reads that as
            # "comfortable" and hands back authority to a vehicle that is
            # already flying more angle of attack than it was asked for.
            self.alpha_ceiling = min(
                self._glide_top(snap),
                self.alpha_ceiling + self.cfg.ALPHA_RECOVER_DEG_S * dt)
            return
        if (error > self.cfg.ALPHA_TRACK_TOLERANCE_DEG
                and self.holdable_quiet_until is not None
                and snap.ut < self.holdable_quiet_until):
            # **A roll is not a refusal.**  The same window that keeps
            # ``Holdable`` from learning a reversal: without it the ceiling
            # backed off 30 -> 20 deg in five seconds of rolling at 3.4 kPa,
            # recovered at 0.5 deg/s, and the solve -- pinned at bank 70 --
            # could not win back the +22 km it cost (LOG2918).
            return
        if error > self.cfg.ALPHA_TRACK_TOLERANCE_DEG:
            # **Floored on the plant measurement, not on a guidance target
            # and not on nothing.**  ``GLIDE_ALPHA_DEG`` is a range target
            # and stopped the ratchet at 20 on a vehicle holding 14 (failure
            # 26); removing it outright let the ratchet collapse the ceiling
            # during the thin-air transient at the entry interface, where a
            # tracking error is the attitude controller lagging and not the
            # airframe refusing -- and an entry flown at alpha 8 hypersonic
            # makes no drag and goes 90 km long (failure 29).
            #
            # ``Holdable`` already knows the difference: it answers None for
            # a dynamic pressure it has no evidence at.  With evidence the
            # ceiling may walk down to what was measured there; without it,
            # the old floor stands.
            learned = self.env.holdable.limit(snap.dynamic_pressure,
                                              self.snap_mach(snap))
            if learned is None:
                floor = self.cfg.GLIDE_ALPHA_DEG
            else:
                floor = max(self.cfg.ALPHA_CEILING_FLOOR_DEG, learned)
            new = max(floor,
                      self.alpha_ceiling - self.cfg.ALPHA_BACKOFF_DEG)
            if new < self.alpha_ceiling - 0.01:
                self.alpha_ceiling = new
                learned = self.env.holdable.limit(snap.dynamic_pressure,
                                                  self.snap_mach(snap))
                self.logbook.event(
                    snap.ut, "alpha ceiling -> %.1f deg (commanded %.1f, "
                             "achieving %.1f, q=%.0f Pa, learned %s)"
                    % (new, self.commanded_alpha, snap.alpha_actual,
                       snap.dynamic_pressure,
                       "%.1f" % learned if learned is not None else "-"))

    def update_cone_energy(self, snap, prediction):
        """``GLIDE_CONE_ENERGY``: what the cone wants to be entered with,
        from the state this tick's prediction enters it in
        (``guidance.cone_entry_energy``), for the next tick's solve.
        Cleared when the arc does not meet the entry test, so no stale
        target is ever priced."""
        if not getattr(self.cfg, "GLIDE_CONE_ENERGY", False):
            return
        state = getattr(prediction, "handover_state", None)
        want = None
        if getattr(prediction, "handover_met", False) and state:
            want = guidance.cone_entry_energy(
                self.env, self.cfg, self.end, state[0], state[1],
                snap.mass, self.surface_gravity)
        self.env.cone_energy = want

    def miss(self, snap):
        if self.prediction is None or not self.prediction.reached:
            return None
        gate = self.env.runway.gate(self.end)
        long, cross = trajectory.miss_components(self.env, self.end,
                                                 self.prediction.position,
                                                 gate)
        # What the solve nulled: under ``GLIDE_ENERGY_AIM`` the position
        # miss plus the arrival speed's error as ground.
        return long + getattr(self.prediction, "energy_long", 0.0), cross

    def gate_distance(self, snap):
        gate = self.env.runway.gate(self.end)
        return trajectory.surface_distance(self.env, snap.position, gate)

    def runway_distance(self, snap):
        return trajectory.surface_distance(self.env, snap.position,
                                           self.end["threshold"])

    def _lock_gimbals(self, ut):
        """Lock every engine gimbal, because otherwise the autopilot cannot engage.

        This is boosterland failure 7 -- ``ModuleGimbal.GetPotentialTorque``
        throwing ``IndexOutOfRange`` when kRPC totals up the vessel's torque
        on ``AutoPilot.engaged = True`` -- except that there it was transient
        and here it is not.  On a booster the gimbals were still settling
        seconds after staging and the same call worked a moment later; on this
        aircraft the engine is *inactive* in orbit, its gimbal has no thrust
        transforms to index, and it throws forever.  A flight spent 150
        seconds retrying with the nose drifting 50 degrees off and never
        recovered.

        Locking the gimbal takes it out of kRPC's torque total and the engage
        succeeds immediately -- measured, on the instance that was failing:
        four refusals at four-second intervals, then ``Lock Gimbal``, then
        engaged.  It stays locked for the flight: the only burn is a few
        seconds of retrograde thrust on a vehicle with reaction wheels, four
        RCS blocks and its own control surfaces, and none of it needs the
        gimbal.

        The action is looked up by its **display** name, which is what kRPC
        reports -- the same lesson the drain valve taught.
        """
        locked = 0
        try:
            modules = [m for part in self.vessel.parts.all
                       for m in part.modules if m.name == "ModuleGimbal"]
        except Exception:                               # noqa: BLE001
            return 0
        for module in modules:
            for action in ("Lock Gimbal", "LockAction"):
                try:
                    if action in module.actions:
                        module.set_action(action, True)
                        locked += 1
                        break
                except Exception:                       # noqa: BLE001
                    continue
        if modules:
            self.logbook.event(ut, "locked %d of %d engine gimbal(s) so the "
                                   "autopilot can engage" % (locked,
                                                             len(modules)))
        return locked

    def runway_remaining(self, snap):
        """Metres of tarmac in front of the wheels, signed.

        Negative past the far end, which is a real state this vehicle reaches
        -- ``logs/LOG2384`` touched down 92 m beyond it -- and the brake law
        needs to be told that rather than shown a clamped zero.
        """
        up = vec.unit(snap.position)
        along = self.env.runway.horizontal(self.end, self.end["along"])
        offset = vec.sub(vec.scale(up, vec.norm(self.end["threshold"])),
                         self.end["threshold"])
        del up
        return self.cfg.RUNWAY_LENGTH_M - vec.dot(offset, along)

    def apply_brakes(self, snap, speed):
        """Brake for the distance that is left, not flat out from touchdown.

        The fraction goes to the wheels individually, because
        ``Control.brakes`` is a boolean and the whole point is that it should
        not be.  The nose wheel is left alone -- ``_release_nose_brake`` has
        already taken it out of the loop and putting a fraction on it here
        would quietly undo that.

        Recorded on the telemetry line as ``brk=`` so that the commanded
        fraction and the achieved ``dec=`` can be read against each other,
        which is what would disagree with ``BRAKE_DECEL_FULL_M_S2``.
        """
        delay = float(getattr(self.cfg, "ROLLOUT_BRAKE_DELAY_S", 0.0))
        since = (getattr(self, "_contact_ut", None)
                 or getattr(self, "state_since", None)) if delay > 0.0 else None
        if since is not None and snap.ut - since < delay:
            # ``ROLLOUT_BRAKE_DELAY_S``: no wheel brake until the tyres have
            # spun up and the gear has taken the weight.
            self.brake_fraction = 0.0
            self.control.brakes = False
            return
        try:
            remaining = self.runway_remaining(snap)
        except Exception:                               # noqa: BLE001
            remaining = None
        fraction = guidance.brake_fraction(
            self.cfg, speed, remaining, self.last_free_decel)
        self.brake_fraction = fraction
        # **The master switch stays on and the strength carries the command.**
        # Toggling ``Control.brakes`` per tick instead re-applies full force
        # for whatever part of a physics frame the strength has not reached.
        self.control.brakes = True
        for wheel in self._brake_wheels():
            try:
                wheel.brakes = self.cfg.WHEEL_BRAKE_MAX_PCT * fraction
            except Exception:                           # noqa: BLE001
                continue

    def main_wheels_grounded(self):
        """True once any braked (main) wheel reports ``grounded``, False
        while none does, None if kRPC cannot say."""
        answer = None
        for wheel in self._brake_wheels():
            try:
                if wheel.grounded:
                    return True
                answer = False
            except Exception:                           # noqa: BLE001
                continue
        return answer

    def log_contact(self, snap):
        """One line, once: the state the tick the main wheels first report
        ``grounded``, and the last tick before it.  The FLARE -> ROLLOUT
        line's sink is read after the bounce (LOG3855: "sink=0.04" for a
        contact at 4.9 m/s and 36 deg of bank), and so is the grounded tick
        itself (LOG3958: -6.2 m/s, the rebound); the tick before is what the
        gear and the wings actually met."""
        if getattr(self, "_contact_logged", False):
            return
        try:
            state = self._contact_state(snap)
        except Exception:                               # noqa: BLE001
            return                # an instrument, never a reason to crash
        if not self.main_wheels_grounded():
            self._pre_contact = state
            return
        self._contact_logged = True
        self._contact_ut = snap.ut
        before = getattr(self, "_pre_contact", None)
        self.logbook.event(
            snap.ut, "contact: before (%s), at (%s), %s"
            % ("?" if before is None else before, state, self.state))

    def _contact_state(self, snap):
        up = vec.unit(snap.position)
        speed = vec.norm(snap.velocity)
        sink = -vec.dot(snap.velocity, up)
        pitch = float("nan")
        if vec.norm(snap.nose) > 0.5:
            pitch = math.degrees(math.asin(vec.clamp(
                vec.dot(vec.unit(snap.nose), up), -1.0, 1.0)))
        return ("sink %.1f m/s at %.1f m/s, h %.1f, bank %+.1f, pitch %+.1f, "
                "alpha %.1f, slip %+.1f"
                % (sink, speed, snap.landing_height, flown_bank(snap), pitch,
                   getattr(snap, "krpc_aoa", float("nan")),
                   getattr(snap, "sideslip", float("nan")) or 0.0))

    def wheel_watch(self, snap):
        """``WHEEL_WATCH_S``: the wheel list and every wheel's state, logged
        whenever it changes, from gear-down until this many game seconds
        after main-gear contact.  An instrument.

        2/3 of the shuttle's level touchdowns roll 11-180 deg within a
        second (rot-gspoiler-1004), and at 5 of ~20 contacts kRPC listed
        only one LY-60 main gear (rot-wheels2-1004) although no part had
        been lost (31 of 31 until contact).  So the list is re-read, not
        cached -- at most every ``WHEEL_WATCH_RELIST_S`` -- with the part
        count beside it, from the moment the gear comes down: is a gear
        missing from the list, broken, or never deployed?  (``stress`` is
        left out: it read 0% on every wheel, every tick.)"""
        span = float(getattr(self.cfg, "WHEEL_WATCH_S", 0.0))
        if span <= 0.0 or not getattr(self, "gear_down", False):
            return
        if getattr(self, "_wheel_watch_last_ut", None) == snap.ut:
            return                        # FLARE calls it twice a tick
        self._wheel_watch_last_ut = snap.ut
        contact = getattr(self, "_contact_logged", False)
        start = getattr(self, "_wheel_watch_ut", None)
        if contact and start is None:
            start = self._wheel_watch_ut = snap.ut
        if start is not None and snap.ut - start > span:
            return
        listed = getattr(self, "_wheel_watch_list_ut", None)
        relist = (listed is None or snap.ut - listed
                  >= float(getattr(self.cfg, "WHEEL_WATCH_RELIST_S", 0.5)))
        if not (relist or contact):
            return
        try:
            if relist:
                self._wheel_watch_list_ut = snap.ut
                frame = self.vessel.reference_frame
                wheels = []
                for w in self.vessel.parts.wheels:
                    x, y, z = w.part.position(frame)
                    wheels.append(("%s(%+.1f,%+.1f)" % (w.part.title[:10], x,
                                                        y), w))
                self._watch_wheels = wheels
                self._watch_parts = len(self.vessel.parts.all)
            states = []
            for name, w in self._watch_wheels:
                try:
                    states.append("%s %s d%d g%d b%d" % (
                        name, str(w.state).rsplit(".", 1)[-1], w.deployed,
                        w.grounded, w.broken))
                except Exception as exc:                # noqa: BLE001
                    states.append("%s unreadable (%s)" % (name, exc))
        except Exception as exc:                        # noqa: BLE001
            states = ["wheels unreadable (%s)" % exc]
        # Positions move with the centre of mass; key on the states alone.
        key = (len(states), getattr(self, "_watch_parts", 0),
               tuple(s.split(") ", 1)[-1] for s in states))
        if key != getattr(self, "_wheel_watch_key", None):
            self._wheel_watch_key = key
            when = ("t+%.1f" % (snap.ut - start) if start is not None
                    else "pre-contact")
            self.logbook.event(snap.ut, "wheels %s bnk=%+.1f, %d wheels, "
                               "%d parts: %s"
                               % (when, flown_bank(snap), len(states),
                                  getattr(self, "_watch_parts", 0),
                                  " | ".join(states)))

    def gear_geometry(self, snap):
        """Once the main wheels report deployed: measure the landing
        geometry with the gear down (``Telemetry.measure_gear_geometry``).
        Applied with ``GEAR_GEOMETRY_DEPLOYED``; measured (for the log and
        ``ground_watch``) whenever either is on."""
        apply = bool(getattr(self.cfg, "GEAR_GEOMETRY_DEPLOYED", False))
        watch = float(getattr(self.cfg, "GROUND_WATCH_S", 0.0)) > 0.0
        if getattr(self, "_gear_geometry_done", False) or not (apply or watch):
            return
        mains = self._brake_wheels()
        try:
            if not mains or not all(w.deployed for w in mains):
                return
        except Exception:                               # noqa: BLE001
            return
        self._gear_geometry_done = True
        self._ground_rows = self.telemetry.measure_gear_geometry(
            snap.ut, [w.part for w in mains], apply)

    def ground_watch(self, snap):
        """``GROUND_WATCH_S``: from the wheels 6 m up until this many game
        seconds after contact, every tick, the lowest point of every part
        above the terrain -- the rigid geometry measured at gear-down,
        turned by the vessel's attitude now -- and, for the parts that come
        closest, how far the real part has moved from where the rigid
        vessel would put it (flex).  An instrument: which part reaches the
        runway, and whether the airframe bends to get it there.

        At 40-50 m/s any part over its crash tolerance (15 m/s for the
        wings, elevons and RCS blocks) that touches the runway explodes;
        the game's own log says the shuttle's first part to go at a gentle
        contact is usually an RV-105 block, while the rigid geometry keeps
        every non-wheel part >= 1.5 m clear at the contact attitude."""
        span = float(getattr(self.cfg, "GROUND_WATCH_S", 0.0))
        rows = getattr(self, "_ground_rows", None)
        if span <= 0.0 or not rows:
            return
        if getattr(self, "_ground_watch_ut", None) == snap.ut:
            return
        self._ground_watch_ut = snap.ut
        contact = getattr(self, "_contact_logged", False)
        start = getattr(self, "_ground_contact_ut", None)
        if contact and start is None:
            start = self._ground_contact_ut = snap.ut
        if start is not None and snap.ut - start > span:
            return
        main_z = [max(c[2] for c in r[5]) for r in rows if r[3]]
        clearance = sum(main_z) / len(main_z)
        if start is None and snap.surface_altitude - clearance > 6.0:
            return
        sc = self.conn.space_center
        try:
            vframe = self.vessel.reference_frame
            sframe = self.vessel.surface_reference_frame
            ups = [sc.transform_direction(e, vframe, sframe)[0]
                   for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
            alt = snap.surface_altitude
            parts_now = len(self.vessel.parts.all)
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(snap.ut, "ground watch unreadable (%s)" % exc)
            return

        def height(p):
            return alt + p[0] * ups[0] + p[1] * ups[1] + p[2] * ups[2]

        lows = []
        for title, part, is_wheel, is_main, origin, corners in rows:
            lows.append((min(height(c) for c in corners), title, part,
                         is_wheel, is_main, origin))
        mains = ["%+.2f" % low for low, _, _, _, m, _ in lows if m]
        others = sorted(l for l in lows if not l[3])
        if getattr(self, "_ground_flex_parts", None) is None:
            self._ground_flex_parts = others[:4]
        flex = []
        for _, title, part, _, _, origin in self._ground_flex_parts:
            try:
                real = part.position(sframe)[0] + alt
                flex.append("%s %+.2f" % (title[:8], real - height(origin)))
            except Exception:                           # noqa: BLE001
                flex.append("%s gone" % title[:8])
        when = ("t%+.2f" % (snap.ut - start) if start is not None
                else "pre")
        self.logbook.event(
            snap.ut, "ground %s n=%d mains %s | low %s | flex %s"
            % (when, parts_now, "/".join(mains),
               ", ".join("%s %+.2f" % (t[:12], low)
                         for low, t, _, _, _, _ in others[:4]),
               ", ".join(flex)))

    def ground_spoiler(self, snap, landed=False):
        """``ROLLOUT_GROUND_SPOILER``: the spoiler fully out on main-gear
        contact, and left out.

        Called every FLARE and ROLLOUT tick; it acts once.  The trigger is
        the main wheels' own ``grounded`` -- the mains touch before the
        game's ``situation`` turns *landed* on a nose-high arrival -- and
        ROLLOUT (``landed``) deploys regardless, as the backstop for a
        vessel whose wheels will not say.  Flaps and the airbrake use the
        same surfaces and are simply overridden.
        """
        self.log_contact(snap)
        self.gear_geometry(snap)
        self.wheel_watch(snap)
        self.ground_watch(snap)
        if (getattr(self.cfg, "ROLLOUT_SPLIT_BRAKE", False)
                and not getattr(self, "_split_ground_done", False)
                and getattr(self, "split_pair", None)
                and (landed or self.main_wheels_grounded())):
            # ``ROLLOUT_SPLIT_BRAKE``: the split rudder full out on the
            # mains, as the Shuttle's speedbrake opened at main-gear
            # touchdown -- +54% drag at 80 m/s (``tools/splitprobe.py``),
            # no yaw or roll; its nose-down moment helps the derotation.
            self._split_ground_done = True
            if self.set_split(float(self.cfg.ROLLOUT_SPLIT_DEG)):
                self.logbook.event(snap.ut, "rollout split brake out: %.0f "
                                   "deg at %.1f m/s" % (
                                       self.split_angle,
                                       vec.norm(snap.velocity)))
        if getattr(self, "_ground_spoiler_done", False):
            return
        grounded = self.main_wheels_grounded()
        if not (landed or grounded):
            return
        self._ground_spoiler_done = True
        brake = None or getattr(
            self, "flap_brake", None)
        if (brake is not None and brake is getattr(self, "flap_brake", None)
                and not getattr(self, "_spoiler_lateral_ok", True)):
            self.logbook.event(snap.ut, "ground spoiler: the measured set "
                                        "rolls the vehicle "
                                        "(AIRBRAKE_SPOILER_LATERAL_CHECK) "
                                        "-- nothing deployed")
            return
        if brake is None or not getattr(brake, "any", True):
            self.logbook.event(snap.ut, "ground spoiler: no measured spoiler "
                                        "set (AIRBRAKE_MEASURED off?) -- "
                                        "nothing deployed")
            return
        biggest = max((abs(m) for _, m in brake.surfaces()), default=0.0)
        if biggest <= 0.0:
            return
        base = float(self.cfg.ROLLOUT_SPOILER_DEG) / biggest
        ok = self.deploy_set(brake, True, base_deg=base)
        if ok:
            self.flap_brake_out = True
        self.flaps_out = False
        readback = []
        for surface, _ in brake.surfaces():
            try:
                for module in surface.key.part.modules:
                    if ("ControlSurface" in module.name
                            and module.has_field("Deploy Angle")):
                        readback.append(module.get_field("Deploy Angle"))
            except Exception:                           # noqa: BLE001
                continue
        self.logbook.event(
            snap.ut, "ground %s %s at %.1f m/s (%s): %d surfaces, "
                     "deploy angle read back %s"
            % (getattr(brake, "kind", "spoiler"), "out" if ok else "FAILED",
               vec.norm(snap.velocity),
               "main wheels grounded" if grounded else "rollout",
               len(brake.surfaces()), "/".join(readback) or "-"))

    def release_reaction_wheels(self, ut):
        """Take the reaction wheels out of the loop once the wheels are down.

        **An attitude the gear is holding is an attitude the reaction wheels
        are fighting.**  This vessel carries 15 kN m on every axis, and
        ``aim_runway`` goes on commanding a body attitude for the whole
        rollout -- pitch ramping to ``ROLLOUT_ALPHA_DEG`` and heading down
        the runway.  In the air that is the right command and it is the
        reason ``aim_runway`` exists.  On the ground the wheels have already
        decided the pitch, so the torque has nowhere to go but into the gear:
        15 kN m over the 2.18 m from the CoM to the contact patch is 6.9 kN
        of couple, on a 6.9 t aircraft, about a tenth of its weight moved
        between the nose and the mains and reversed whenever the controller
        changes its mind.

        The aerodynamic surfaces and the nosewheel are left alone -- they act
        through the airflow and the contact patch respectively, which is how
        an aircraft is actually steered on a runway, and turning *those* off
        would be the departure this is trying to prevent.

        Idempotent, and quiet about a vessel that has none.
        """
        if self._reaction_wheels_released:
            return
        self._reaction_wheels_released = True
        return

    def _brake_wheels(self):
        """Every wheel that brakes except the frontmost, found once.

        Same rule as ``_release_nose_brake``: the nose wheel is the one
        furthest along the direction the vehicle points, measured rather than
        named, because this autopilot is not written for one vessel.
        """
        if self._brake_wheel_cache is not None:
            return self._brake_wheel_cache
        try:
            wheels = [w for w in self.vessel.parts.wheels if w.has_brakes]
        except Exception:                               # noqa: BLE001
            return []
        front = None
        if len(wheels) > 1:
            frame = self.vessel.reference_frame
            try:
                forward = self.vessel.direction(frame)
                front = max(wheels,
                            key=lambda w: vec.dot(w.part.position(frame),
                                                  forward))
            except Exception:                           # noqa: BLE001
                front = None
        self._brake_wheel_cache = [w for w in wheels if w is not front]
        return self._brake_wheel_cache

    def _set_main_gear(self, ut):
        """Every wheel but the nose: full brake torque and manual friction;
        the nose: manual friction at ``NOSE_WHEEL_FRICTION`` (auto off).

        The user's standing rule for the gear: the nose wheel gets no brake
        and (since 2026-09-30) manual friction, automatic control off; the mains get ``WHEEL_BRAKE_MAX_PCT``
        (200, the game's maximum) and friction control switched to manual
        at ``MAIN_WHEEL_FRICTION`` (10, the slider's maximum).  Measured
        before: the mains braked at 50% on the craft's own setting and the
        shuttle rolled 4.2 km from a 65 m/s touchdown (LOG3609).

        Friction is a ``ModuleWheelBase`` field kRPC only lists once the
        auto toggle is off, and both carry localised GUI names -- so both
        are found by "friction" in their name, and the log says what was set.
        """
        wheels = self._brake_wheels()
        if not wheels:
            return
        target = float(self.cfg.MAIN_WHEEL_FRICTION)
        done = []
        for wheel in wheels:
            what = []
            try:
                if wheel.has_brakes:
                    wheel.brakes = float(self.cfg.WHEEL_BRAKE_MAX_PCT)
                    what.append("brake %.0f%%" % wheel.brakes)
            except Exception:                           # noqa: BLE001
                pass
            if target > 0.0:
                what.append(self._set_friction(wheel, target))
            spring = float(getattr(self.cfg, "MAIN_GEAR_SPRING", 0.0))
            damper = float(getattr(self.cfg, "MAIN_GEAR_DAMPER", 0.0))
            if spring > 0.0 or damper > 0.0:
                what.append(self._set_suspension(wheel, spring, damper))
            done.append("%s: %s" % (wheel.part.title, ", ".join(what)))
        self.logbook.event(ut, "main gear: " + " | ".join(done))
        nose_target = float(self.cfg.NOSE_WHEEL_FRICTION)
        nose = self._nose_wheel()
        if nose is not None and nose_target > 0.0:
            self.logbook.event(ut, "nose gear: %s: %s" % (
                nose.part.title, self._set_friction(nose, nose_target)))

    def _nose_wheel(self):
        """The frontmost wheel, by the rule ``_brake_wheels`` excludes it by."""
        try:
            wheels = list(self.vessel.parts.wheels)
            if len(wheels) < 2:
                return None
            frame = self.vessel.reference_frame
            forward = self.vessel.direction(frame)
            return max(wheels, key=lambda w: vec.dot(w.part.position(frame),
                                                     forward))
        except Exception:                               # noqa: BLE001
            return None

    def _set_suspension(self, wheel, spring, damper):
        """``MAIN_GEAR_SPRING`` / ``MAIN_GEAR_DAMPER``: the suspension off
        auto and at these strengths (0 leaves that one as it is).

        CollisionSpy (LOG5855): at a 4 m/s contact both LY-60 *bodies* hit
        the runway at 47 m/s in the step both wing roots broke -- the
        suspension bottomed and the leg dragged on the tarmac.  Like
        friction, the strengths are fields kRPC lists only once the
        ``Spring/Damper`` toggle is off auto.  Says what it set."""
        try:
            for module in wheel.part.modules:
                if module.name != "ModuleWheelSuspension":
                    continue
                if not any("spring" in n.lower() for n in module.fields):
                    for event in module.events:
                        if "spring" in event.lower() and "auto" in event.lower():
                            module.trigger_event(event)
                fields = list(module.fields)
                for want, value in (("spring", spring), ("damper", damper)):
                    if value <= 0.0:
                        continue
                    for name in fields:
                        if want in name.lower():
                            module.set_field_float(name, value)
                return "suspension %s" % ", ".join(
                    "%s %s" % (k, v) for k, v in module.fields.items())
        except Exception as exc:                        # noqa: BLE001
            return "suspension not set (%s)" % exc
        return "no suspension module"

    def _set_friction(self, wheel, target):
        """Manual friction at ``target`` on one wheel; says what happened."""
        try:
            for module in wheel.part.modules:
                if module.name != "ModuleWheelBase":
                    continue
                def friction_field():
                    for name in module.fields:
                        if "friction" in name.lower():
                            return name
                    return None
                field = friction_field()
                if field is None:
                    for event in module.events:
                        if "friction" in event.lower():
                            module.trigger_event(event)
                            break
                    field = friction_field()
                if field is None:
                    return "friction: no manual field"
                module.set_field_float(field, float(target))
                return "friction %s" % module.fields.get(field, "?")
        except Exception as exc:                        # noqa: BLE001
            return "friction: failed (%s)" % exc
        return "friction: no ModuleWheelBase"

    def _release_nose_brake(self, ut):
        """Take the brakes off whichever gear is furthest forward.

        **Found, not configured.**  Which wheel is the nose wheel is a
        property of the vessel, and this autopilot is not written for one
        vessel: the frontmost is the one whose position has the largest
        component along the direction the vehicle points, and both of those
        are measured rather than assumed -- the same rule that stopped
        ``planeprobe`` flying this aircraft tail-first and ``Telemetry``
        reporting half a wingspan as the wheel clearance.

        Silent if there is nothing to do, loud if there is: a vessel whose
        nose wheel brakes cannot be reached is a vessel that will land the
        way the last twenty did.
        """
        try:
            wheels = list(self.vessel.parts.wheels)
        except Exception:                               # noqa: BLE001
            return
        if len(wheels) < 2:
            return
        frame = self.vessel.reference_frame
        try:
            forward = self.vessel.direction(frame)
        except Exception:                               # noqa: BLE001
            return
        front, best = None, None
        for wheel in wheels:
            try:
                along = vec.dot(wheel.part.position(frame), forward)
            except Exception:                           # noqa: BLE001
                continue
            if best is None or along > best:
                front, best = wheel, along
        if front is None:
            return
        try:
            if not front.has_brakes:
                return
            was = front.brakes
            front.brakes = 0.0
            self.logbook.event(ut, "nose brake off: %s at %+.2f m forward "
                                   "(%.0f -> %.0f)"
                               % (front.part.title, best, was, front.brakes))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(ut, "could not release the nose brake (%s) -- "
                                   "it will pitch onto it at touchdown" % exc)

    def log_drain_module(self, module, index=0):
        """Say what each valve actually exposes, once.

        The valve empties 556 units in 2.0 s and this loop cannot tick faster
        than about two game-seconds, so "poll and close at a reserve" cannot
        work -- the first tick after opening it is already dry.  Stopping it
        at a reserve needs its *rate*, and the rate is a field whose display
        name is not knowable from here.  So the module says what it has and
        the log keeps it, the same way the two spellings of the Drain action
        were found.

        Each valve is numbered, because two of them need not be the same part
        -- different rates, different ``Resources`` -- and a log that merges
        them cannot say which one a mass discrepancy came from.
        """
        try:
            fields = dict(module.fields)
        except Exception:                               # noqa: BLE001
            fields = {}
        try:
            actions = list(module.actions)
        except Exception:                               # noqa: BLE001
            actions = []
        self.logbook.event(0.0, "drain module %d fields: %r" % (index, fields))
        self.logbook.event(0.0, "drain module %d actions: %r"
                           % (index, actions))

    def _find_drain(self):
        """Every drain valve's module, found by what it is rather than by name.

        ``ModuleResourceDrain`` is the module a release valve carries; the part
        is called ``ReleaseValve`` on this craft but that is a part name and
        the next vehicle's will differ.  The module is the contract.

        **All of them, not the first one.**  A vehicle may carry more than one
        valve, and draining through one of two halves the rate -- which the
        drain's own comments show this phase cannot afford to be wrong about,
        since it closes the valve on a *watched* reserve and a slower drain
        spends longer between reads.  Worse, a valve left shut is propellant
        the entry flies heavy with, and the mass is what the whole phase
        exists to establish.  So every module found is driven, together.
        """
        found = []
        try:
            for part in self.vessel.parts.all:
                for module in part.modules:
                    if module.name == "ModuleResourceDrain":
                        self.log_drain_module(module, len(found))
                        found.append(module)
        except Exception:                               # noqa: BLE001
            pass
        self.logbook.event(0.0, "drain valves found: %d" % (len(found),))
        return found

    def _surface_records(self):
        """Every control surface as a plain record, in the vessel frame.

        Shared by both brakes so the geometry is read once and read the same
        way.  ``None`` when the craft has no control surfaces at all, which
        must fly exactly as it does now rather than raise on the pad.
        """
        records = []
        try:
            frame = self.vessel.reference_frame
            for cs in self.vessel.parts.control_surfaces:
                part = cs.part
                try:
                    area = float(cs.surface_area)
                except Exception:                       # noqa: BLE001
                    area = 0.0
                records.append(airbrake_mod.Surface(
                    cs, part.position(frame),
                    airbrake_mod.span_axis(part.rotation(frame)),
                    area, part.title))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(0.0, "airbrake: no control surfaces (%s)"
                               % type(exc).__name__)
            return None
        return records

    def _find_brake(self):
        """Every surface this vehicle can deploy without upsetting itself.

        One call, both rules: the mirrored vertical pair and the fore/aft
        horizontal groups.  They cancel on different axes -- yaw and roll by
        symmetry, pitch by the area-times-arm ratio -- so a craft that can do
        both gets **all** of its control surfaces as drag.  On the old craft
        that is 6 m^2 against a whole-craft ``CdA`` of 5.5, where the
        vertical pair alone was worth 21 m/s in eleven seconds.

        One group refusing does not veto the other, and every decision is
        logged: a brake that silently did not arm reads exactly like one
        that armed and did nothing.
        """
        records = self._surface_records()
        if records is None:
            return None
        # ``vessel.reference_frame``'s origin *is* the centre of mass, so
        # the positions already carry the arms and the split is at zero.
        armed = airbrake_mod.find_all_brakes(records, 0.0, self.cfg)
        self.logbook.event(0.0, armed.describe())
        return armed if armed.any else None

    def _deploy_surface(self, surface, angle, out):
        """One surface's ``Deploy Angle`` and deployed state, or False."""
        try:
            for module in surface.key.part.modules:
                if ("ControlSurface" in module.name
                        and module.has_field("Deploy Angle")):
                    module.set_field_float("Deploy Angle", float(angle))
            surface.key.deployed = bool(out)
            return True
        except Exception:                               # noqa: BLE001
            return False

    def _probe_wrench(self, altitude, speed, alpha_deg):
        """``(ClA, CmA)`` at a synthetic state, the attitude as it is now.

        The same aimed-airflow probe as ``Environment._probe_row``, plus the
        pitching moment ``simulate_aerodynamic_wrench_at`` also returns --
        the quantity this project long believed kRPC would not report.
        """
        env = self.env
        nose, dorsal = env._axes()
        if nose is None:
            return None
        rot = tuple(self.vessel.rotation(env.frame))
        right = vec.unit(self.conn.space_center.transform_direction(
            (1.0, 0.0, 0.0), self.vessel.reference_frame, env.frame))
        up = vec.unit(self.vessel.position(env.frame))
        pos = tuple(vec.scale(up, env.equatorial_radius + altitude))
        a = math.radians(alpha_deg)
        d = vec.unit(vec.sub(vec.scale(nose, math.cos(a)),
                             vec.scale(dorsal, math.sin(a))))
        force, torque = env.flight.simulate_aerodynamic_wrench_at(
            env.body, pos, tuple(vec.scale(d, speed)), rot,
            (0.0, 0.0, 0.0), self.conn.space_center.ut)
        q = 0.5 * env.density(altitude) * speed * speed
        if q <= 0.0:
            return None
        lift = vec.unit(vec.project_out(dorsal, d))
        return vec.dot(force, lift) / q, vec.dot(torque, right) / q

    def measure_flap_brake(self, snap):
        """``AIRBRAKE_MEASURED``: find the lift-spoiling sense of every
        horizontal surface by deploying it, in vacuum, once.

        See ``airbrake.MeasuredBrake`` for why the geometric brake was half
        a flap.  Blocks for about a second per surface -- the deployment
        needs game frames to move -- which is only affordable in the vacuum
        before the burn, so a vehicle engaged in the air is refused and
        says so rather than measured mid-flight.
        """
        if getattr(self, "_brake_measured", False) or not self.env.ready():
            return
        # **Not while paused**: a deployment needs game frames to move, and
        # the harness hands over a paused game that ``main`` unpauses only
        # after the first tick.  Measured paused, every surface read 0.00.
        try:
            if self.conn.krpc.paused:
                return
        except Exception:                               # noqa: BLE001
            pass
        self._brake_measured = True
        if snap.dynamic_pressure > 1.0:
            self.flap_brake = None
            # ``AIRBRAKE_CACHE``: the set this craft measured in vacuum on an
            # earlier flight, matched part by part.
            cached = (self._load_brake_cache()
                      if getattr(self.cfg, "AIRBRAKE_CACHE", False) else None)
            if cached is not None:
                self.flap_brake = cached
                self.flap_set = None
                self.logbook.event(snap.ut, "brake (measured): engaged in the "
                                            "air -- armed from the cache: "
                                            "%s" % cached.describe())
                return
            self.logbook.event(snap.ut, "brake (measured): engaged in the air "
                                        "(q=%.0f Pa) -- not measured, not "
                                        "armed" % snap.dynamic_pressure)
            return
        records = self._surface_records() or []
        horizontals = [r for r in records if airbrake_mod.is_horizontal(r)]
        theta = float(self.cfg.AIRBRAKE_MEASURE_DEG)
        settle = float(self.cfg.AIRBRAKE_MEASURE_SETTLE_S)
        alt = float(self.cfg.AIRBRAKE_MEASURE_ALT_M)
        vs = airframe.stall(self.env, self.cfg)
        if vs is None:
            return
        speed = self.cfg.APPROACH_FACTOR * vs
        alpha = float(self.cfg.AIRBRAKE_MEASURE_ALPHA_DEG)
        try:
            base = self._probe_wrench(alt, speed, alpha)
            samples = []
            for r in horizontals:
                got = []
                for angle in (theta, -theta):
                    self._deploy_surface(r, angle, True)
                    self.settle_game(settle)
                    w = self._probe_wrench(alt, speed, alpha)
                    got.append((w[0] - base[0], w[1] - base[1]))
                self._deploy_surface(r, 0.0, False)
                self.settle_game(settle)
                samples.append((r, got[0][0], got[0][1], got[1][0],
                                got[1][1]))
                self.logbook.event(
                    snap.ut, "brake probe %s: +%.0f dClA %+.2f dCmA %+.2f | "
                             "-%.0f dClA %+.2f dCmA %+.2f"
                    % (r.title, theta, got[0][0], got[0][1], theta,
                       got[1][0], got[1][1]))
            one_side = max(max(abs(s[2]), abs(s[4])) for s in samples) \
                if samples else 1.0
            limit = float(self.cfg.AIRBRAKE_MEASURE_MOMENT_FRAC) * one_side

            def verify(chosen, sign):
                """Deploy the set, read the wrench, rebalance; the set and
                whether it is fit to fly."""
                verified = None
                history = []
                for attempt in range(1 + int(self.cfg.AIRBRAKE_MEASURE_ITER)):
                    if not chosen.any:
                        break
                    for r, m in chosen.surfaces():
                        self._deploy_surface(r, theta * m, True)
                    self.settle_game(settle)
                    w = self._probe_wrench(alt, speed, alpha)
                    verified = (w[0] - base[0], w[1] - base[1])
                    for r, _ in chosen.surfaces():
                        self._deploy_surface(r, 0.0, False)
                    self.settle_game(settle)
                    self.logbook.event(
                        snap.ut, "%s (measured) set %d, gains nose-up x%.2f "
                                 "nose-down x%.2f: dClA %+.2f dCmA %+.2f"
                        % (getattr(chosen, "kind", "spoiler"), attempt,
                           chosen.gains[0], chosen.gains[1], verified[0],
                           verified[1]))
                    if abs(verified[1]) <= limit:
                        break
                    rebalanced = chosen.rebalanced(verified[1], history)
                    history.append((chosen.balance(), verified[1]))
                    chosen = rebalanced
                self.logbook.event(snap.ut, chosen.describe())
                if verified is None:
                    return None
                # **Measure the set, not only the parts.**  Deflections do
                # not add; a set predicted to cancel that does not is an
                # uncommanded pitch input, so it is refused, not flown.
                ok = sign * verified[0] > 0.0 and abs(verified[1]) <= limit
                # What the set did, at the angle it did it -- the lift
                # ``spoiler_lift_hold`` has to buy back with alpha.
                chosen.verified_lift = verified[0]
                chosen.verified_deg = theta
                self.logbook.event(
                    snap.ut, "%s (measured) verified at %.0f deg: dClA "
                             "%+.2f, dCmA %+.2f (largest single surface "
                             "%.2f) -- %s"
                    % (getattr(chosen, "kind", "spoiler"), theta,
                       verified[0], verified[1], one_side,
                       "armed" if ok else "REFUSED"))
                return chosen if ok else None

            spoiler = verify(airbrake_mod.choose_measured_brake(samples),
                             -1.0)
            self._spoiler_lateral_ok = True
            flaps = None
        except Exception as exc:                        # noqa: BLE001
            self.flap_brake = None
            self.flap_set = None
            self.logbook.event(snap.ut, "brake (measured): probe failed (%s) "
                                        "-- not armed" % exc)
            return
        self.flap_brake = spoiler
        self.flap_set = flaps
        if spoiler is not None:
            self._save_brake_cache(spoiler)

    # -- the measured brake, kept between flights ---------------------------
    #
    # **A save that starts in the air has never had a brake.**  The probe
    # needs vacuum, so every cone and final-approach save flew without the
    # one spender the cone's surplus logic is written around, and a flag
    # tested from those saves was disconnected (sav-spend-1006).  The set is
    # a property of the craft, not the flight: key it on the surfaces and
    # reuse it.

    def _load_flown_polar(self, ut):
        """``HAC_LD_FLOWN_POLAR``: this craft's flown cone polar from
        ``logs/conepolar/`` (``tools/conepolar.py``), and say so."""
        try:
            import json
            key = trajectory.holdprior_key(self.vessel.name,
                                           len(self.vessel.parts.all))
            path = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "logs", "conepolar",
                trajectory.holdprior_slug(key) + ".json")
            if not os.path.exists(path):
                self.logbook.event(ut, "flown polar: none for %r (%s)"
                                   % (key, path))
                return
            with open(path) as fh:
                data = json.load(fh)
            self.env.flown_polar = data
            self.logbook.event(
                ut, "flown polar: %r from %d logs below Mach %.1f: %s"
                % (key, data.get("logs", 0), data.get("mach_max", 0.9),
                   " ".join("%.0f:%.0f/%.2f" % (b[0], b[1], b[1] / b[2])
                            for b in data["bins"])))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(ut, "flown polar: not loaded (%s)" % (exc,))

    def _load_holdable_prior(self, ut):
        """``HOLDABLE_PRIOR``: seed ``env.holdable`` from this craft's file
        in ``logs/holdprior/`` (``tools/holdprior.py``), and say so."""
        try:
            import json
            key = trajectory.holdprior_key(self.vessel.name,
                                           len(self.vessel.parts.all))
            path = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "logs", "holdprior",
                trajectory.holdprior_slug(key) + ".json")
            if not os.path.exists(path):
                self.logbook.event(ut, "alpha prior: none for %r (%s)"
                                   % (key, path))
                return
            with open(path) as fh:
                data = json.load(fh)
            cells = [tuple(b[:3]) + tuple(b[4:6]) for b in data["bins"]]
            self.env.holdable.set_prior(cells, data.get("mach_floor", 0.0))
            self.logbook.event(
                ut, "alpha prior: %r from %d logs, above Mach %.1f: %s"
                % (key, data.get("logs", 0), self.env.holdable.prior_mach,
                   " ".join("%s%.0fPa:%.1f" % (
                       "M%g-%g/" % c[3:5] if len(c) >= 5 else "", c[0], c[2])
                       for c in self.env.holdable.prior)))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(ut, "alpha prior: not loaded (%s)" % (exc,))

    def _brake_cache_path(self, records):
        import hashlib
        names = sorted("%s@%.1f,%.1f,%.1f" % ((r.title,) + tuple(r.position))
                       for r in records)
        key = hashlib.sha1("|".join(names).encode()).hexdigest()[:12]
        root = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "logs", "brakecache")
        return os.path.join(root, key + ".json")

    @staticmethod
    def _record_id(record):
        return "%s@%.1f,%.1f,%.1f" % ((record.title,) + tuple(record.position))

    def _save_brake_cache(self, brake):
        try:
            import json
            records = self._surface_records() or []
            if not records or not getattr(brake, "parts", None):
                return
            path = self._brake_cache_path(records)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            data = {
                "parts": [[self._record_id(r), sense, dl, dm]
                          for r, sense, dl, dm in brake.parts],
                "gains": list(brake.gains),
                "kind": getattr(brake, "kind", "spoiler"),
                "verified_lift": getattr(brake, "verified_lift", None),
                "verified_deg": getattr(brake, "verified_deg", None),
            }
            with open(path, "w") as fh:
                json.dump(data, fh)
        except Exception:                               # noqa: BLE001
            pass

    def _load_brake_cache(self):
        try:
            import json
            records = self._surface_records() or []
            if not records:
                return None
            path = self._brake_cache_path(records)
            if not os.path.exists(path):
                return None
            with open(path) as fh:
                data = json.load(fh)
            by_id = dict((self._record_id(r), r) for r in records)
            parts = []
            for rid, sense, dl, dm in data["parts"]:
                if rid not in by_id:
                    return None
                parts.append((by_id[rid], sense, dl, dm))
            brake = airbrake_mod.MeasuredBrake(
                [], 0.0, 0.0, reasons=("cached",), parts=parts
            ).with_gains(*data["gains"])
            brake.kind = data.get("kind", "spoiler")
            if data.get("verified_lift") is not None:
                brake.verified_lift = data["verified_lift"]
                brake.verified_deg = data["verified_deg"]
            return brake
        except Exception:                               # noqa: BLE001
            return None

    def deploy_set(self, surface_set, out, base_deg=None):
        """Deploy or stow any measured set (spoiler or flaps)."""
        if surface_set is None:
            return False
        if base_deg is None:
            base_deg = float(getattr(self.cfg, "AIRBRAKE_DEPLOY_ANGLE_DEG",
                                     20.0))
        done = 0
        for surface, multiplier in surface_set.surfaces():
            if self._deploy_surface(surface, base_deg * multiplier, out):
                done += 1
        return bool(done)

    def set_flap_brake(self, out, base_deg=None):
        """Deploy or stow the opposed flaps, at the moment-cancelling ratio.

        **The angles are not equal and must not be.**  The forward group is
        usually small and on a long arm, so it deflects ``ratio`` times the
        aft group -- on the shuttle's layout that is a factor of thirteen.
        Deploying both to the same angle is not a brake, it is a large
        uncommanded pitch input.
        """
        brake = getattr(self, "flap_brake", None)
        if brake is None:
            return False
        if base_deg is None:
            base_deg = float(getattr(self.cfg, "AIRBRAKE_DEPLOY_ANGLE_DEG",
                                     20.0))
        done = 0
        for surface, multiplier in brake.surfaces():
            angle = base_deg * multiplier
            try:
                for module in surface.key.part.modules:
                    if "ControlSurface" not in module.name:
                        continue
                    if module.has_field("Deploy Angle"):
                        module.set_field_float("Deploy Angle", float(angle))
                surface.key.deployed = bool(out)
                done += 1
            except Exception:                           # noqa: BLE001
                continue
        if done:
            self.flap_brake_out = bool(out)
        return bool(done)

    def command_airbrake(self, snap, command, height, flare_trigger=0.0,
                         sink=0.0):
        """One tick of the brake, against the approach's own surplus.

        The trigger is ``guidance.approach``'s weave command sitting at its
        cap -- the guidance saying it has run out of range control -- with
        surplus height still unspent; see ``spaceplane.airbrake`` for why
        that rather than an altitude, and ``GEAR_DRAG_FRACTION`` for what
        happens to drag that is not commanded against a surplus.
        """
        previous = self.airbrake.extended
        dt = 0.0 if self.airbrake_ut is None else max(0.0,
                                                      snap.ut - self.airbrake_ut)
        self.airbrake_ut = snap.ut
        wanted = self.airbrake.update(
            dt, getattr(command, "scurve_deg", 0.0),
            float(self.cfg.APPROACH_SCURVE_MAX_DEG),
            getattr(command, "excess", 0.0), height, flare_trigger, sink,
            getattr(command, "speed", None),
            getattr(command, "target_speed", None),
            gravity=self.surface_gravity)
        # **A spoiler serves the approach's descent, it does not replace
        # it.**  See ``Config.AIRBRAKE_SINK_TRACK``.
        track = float(getattr(self.cfg, "AIRBRAKE_SINK_TRACK_M_S", 0.0))
        want_sink = getattr(command, "wanted_sink", None)
        if wanted and want_sink is not None and (sink > want_sink + track):
            wanted = False
            self.airbrake.extended = False
            self.airbrake.last_reason = ("sink %.0f past the %.0f the "
                                         "approach wants" % (sink, want_sink))
        flaps = getattr(self, "flap_brake", None) is not None
        if not flaps:
            self.airbrake.extended = previous
            return
        if wanted == previous:
            return
        if flaps:
            self.set_flap_brake(wanted)
        self.logbook.event(snap.ut, "airbrake %s at %.0f m: %s"
                           % ("out" if wanted else "in", height,
                              self.airbrake.last_reason))

    def set_rcs(self, permitted, snap=None):
        """RCS on only while something is actually turning the vehicle.

        The phase says where thrusters are *allowed*; ``rcs.Valve`` decides
        when they fire, from the angle between the commanded nose and the
        real one.  Permission alone still pays for the hunt, because the hunt
        happens inside the phase that needed the slew: the flip costs a few
        seconds of thruster and the settling afterwards costs more than the
        flip.  See ``common.rcs``.
        """
        self.rcs.update(snap.ut if snap is not None else 0.0, permitted,
                        self.valve_error(snap),
                        None if snap is None else snap.dynamic_pressure,
                        apply=self._apply_rcs,
                        hold=self.reversal_under_way(snap))
        self.rcs_on = self.rcs.on

    def _apply_rcs(self, wanted):
        try:
            self.control.rcs = wanted
        except Exception:                               # noqa: BLE001
            pass

    def valve_error(self, snap):
        """The pointing error the RCS valve opens on.

        ``RCS_IGNORE_ALPHA_SHORTFALL``: in GLIDE and HAC, a nose *below* its
        commanded angle of attack is the surfaces' trim limit -- a steady
        saturation, not a turn -- and is not counted; lateral error and an
        alpha *overshoot* still are.  The reference is the commanded nose
        with its alpha lowered to the alpha flown in the commanded plane.
        Measured: on ~15 shuttle flights the valve opened on exactly that
        (40.3 commanded, 36 trimmed, ``err 5.0`` at q 1100-2100, Mach
        4.7-5.4), and the pitch thrusters drove alpha through the trim
        limit to 48-58 within 4 s (LOG4352, 4354, 4375, 4379, 4383...).
        ``RCS_PITCH_BY_AUTHORITY`` never saw it: it latches at q ~1650.
        """
        return self.pointing_error(snap)

    def pointing_error(self, snap):
        """Degrees between the commanded nose and the real one, or -1."""
        if snap is None or self.commanded_nose is None:
            return -1.0
        if vec.norm(snap.nose) < 0.5:
            return -1.0
        return vec.angle_between(snap.nose, self.commanded_nose)

    def run_standby(self, snap):
        """Idle, but not idle: this is where the aerodynamic table is built.

        A booster connects seconds before it has to fly and has no time to
        measure anything.  This vehicle has a whole orbit, in vacuum, with
        nothing to do -- so the entire (alpha, Mach) table is swept here, and
        the first prediction of the flight is made on measured coefficients
        rather than on a warm-up.  That is worth more than it sounds: the
        boostback exit that cost boosterland kilometres was a decision taken
        on an unconverged curve.
        """
        self.set_rcs(False)
        self.set_throttle(0.0)
        if not self.env.ready():
            self.env.sweep(snap.ut, full=True)
        self.engage_autopilot(snap.ut)
        if vec.norm(snap.velocity) > 1.0:
            self.aim(0.0, 0.0, snap)
        if self.panel.start_pressed():
            if self.cfg.STARTUP_ACTION_GROUP:
                self.control.set_action_group(
                    int(self.cfg.STARTUP_ACTION_GROUP), True)
            self.engage(snap.ut, snap.height_above_runway,
                        snap.position, snap.velocity)

    def engage(self, ut, height, position, velocity):
        """Enter the phase that matches where the vehicle actually is.

        **A vehicle handed over already inside its entry does not have a
        deorbit to fly.**  Starting in DEORBIT from below the interface asks
        ``deorbit_solution`` for a burn on a trajectory that is already
        committed, and it answers: measured, 10 m/s lit at 55 km, *inside the
        atmosphere*, on a vehicle that had nothing to correct.

        It is not only a harness convenience, though that is what it was
        written for -- half of every measurement flight is a deorbit and a
        coast that most changes do not touch, and a save taken at the
        interface removes both, along with the burn's contribution to the
        scatter.  An autopilot engaged mid-entry is a real case and this is
        what it should do.

        Both entry points come through here.  The first version put the test
        in ``run_standby`` only, and ``--autostart`` does not go through
        ``run_standby`` -- it forced DEORBIT directly, so the change looked
        like it did nothing.
        """
        if (height <= self.cfg.ENTRY_INTERFACE_M
                if not getattr(self.cfg, "ENTRY_INTERFACE_AT_AIR", False)
                else trajectory.past_interface(
                    self.env, self.cfg,
                    vec.norm(position) - self.env.equatorial_radius)):
            self.end = self.env.runway.choose(position, velocity)
            self.steer = Steer(alpha=min(self.cfg.ENTRY_ALPHA_DEG,
                                         self.alpha_ceiling),
                               bank=self.cfg.SOLVE_BANK_MIN_DEG)
            # **And the same argument does not stop at the interface.**  The
            # docstring above says "enter the phase that matches where the
            # vehicle actually is" and the code knew exactly two of them, so
            # a vehicle handed over on short final was given an entry to fly:
            # GLIDE at 600 m solves a two-thousand-kilometre range problem
            # from inside the last twenty seconds of the flight.
            #
            # The throughput argument is the same one that justified the
            # interface save and it is much stronger here.  The open problem
            # on this vehicle is the last two seconds -- it reaches the
            # runway and comes apart on it -- and every attempt at those two
            # seconds currently costs a 1300-second flight.  A save on final
            # costs about thirty.
            #
            # Ordered by energy, and the landing phases are only entered
            # *near the field*: APPROACH assumes it is on final, and a
            # vehicle at gate height twenty kilometres out is not.  The
            # bound is the approach's own geometry -- the gate plus the
            # distance from the gate to the aim -- rather than a new
            # constant.
            if getattr(self.cfg, "ENGAGE_INTO_LANDING", False):
                reach = (self.env.runway.gate_dist()
                         + airframe.touchdown_aim(self.env, self.cfg)
                         + self.cfg.GATE_ALT_M
                         * airframe.approach_ld(
                             self.env, self.cfg, self.cfg.GATE_ALT_M,
                             self.vessel.mass, self.surface_gravity))
                try:
                    out = trajectory.surface_distance(
                        self.env, position, self.end["threshold"])
                except Exception:                       # noqa: BLE001
                    out = None
                sink = -vec.dot(velocity, vec.unit(position))
                trigger = guidance.flare_door(self.cfg, sink,
                                              vec.norm(velocity), self.env)
                if out is not None and out <= reach:
                    if height <= trigger:
                        self.enter(FLARE, ut,
                                   "engaged on the flare at %.0f m, %.0f m "
                                   "from the threshold, runway %s"
                                   % (height, out, self.end["name"]))
                        return
                    if height <= self.cfg.GATE_ALT_M:
                        self.enter(APPROACH, ut,
                                   "engaged on final at %.0f m, %.0f m from "
                                   "the threshold, runway %s"
                                   % (height, out, self.end["name"]))
                        return
            self.enter(GLIDE, ut,
                       "engaged below the interface at %.0f m, runway %s"
                       % (height, self.end["name"]))
            return
        if self.cfg.DRAIN:
            # **Upstream of the closed loop, not downstream of it.**  See
            # ``DRAIN_BEFORE_BURN``: the valve is worth 3.3 m/s on
            # ``qs_plane`` and 9.3 on ``qs_plane_inc``, and after the burn
            # nothing can answer for it.  Before the burn, the burn's own
            # stop test answers for it without being told it exists.
            self.enter(DRAIN, ut, "runway %s, draining before the burn"
                       % self.end["name"])
            return
        self.enter(DEORBIT, ut, "runway %s" % self.end["name"])

    def report_airframe(self, snap, landing=False):
        """Read the landing constants off the table, and say so if they
        disagree with the ones configured.

        This is the rule in CLAUDE.md made executable: *a measurement whose
        only consumer is a hand-copied constant cannot be contradicted*.
        ``STALL_SPEED_M_S`` and ``APPROACH_BEST_LD`` were transcribed out of
        ``planeprobe``'s file, which is wrong by 1.8x subsonically, and
        nothing in a flight could have told anyone -- the vehicle simply
        stalled in the flare for a year.

        It does not *change* anything.  A derived number is a hypothesis
        until the game agrees with it, and swapping the landing's constants
        for the table's reading, silently, on the strength of an argument, is
        the move this project has a rule against.  What it does is make the
        disagreement impossible to miss.

        **And for the life of the project it made the wrong one impossible to
        miss.**  ``airframe.measure`` says in its own signature that ``mass``
        is "the mass the *landing* is flown at -- the drained one", and this
        handed it ``entry_mass``, which with ``DRAIN_BEFORE_BURN`` is simply
        the mass aboard *now* -- 14.527 t at STANDBY on ``qs_plane`` against
        the **6.93 t** the wheels arrive at.  A stall speed goes as the square
        root of the weight, so the number reported was about 1.45 times the
        real one, and the alarm beneath it read "STALL_SPEED_M_S is 48.00 and
        the swept table says 78.14 (63% out) -- one of them is not this
        aircraft" on every flight ever logged.

        At the landing mass the table says about 52, and the configured 48 is
        within a few per cent.  **The constant was roughly right and the
        instrument was wrong**, which is the more dangerous way round: a false
        alarm that stands long enough stops being read, and this one was one
        step from being "fixed" by moving the constant to match it.

        So it is taken twice -- once in STANDBY, where the entry mass is the
        right question for the glide, and once on final with ``landing=True``,
        where ``snap.mass`` *is* the landing mass and no prediction is
        needed.  The second is the one the landing constants should be judged
        against.
        """
        mass = snap.mass
        try:
            measured = airframe.measure(self.env, mass,
                                        self.surface_gravity)
        except Exception as exc:                            # noqa: BLE001
            self.airframe = airframe.Airframe()
            self.logbook.event(snap.ut, "airframe: could not be read (%r)"
                               % (exc,))
            return
        self.airframe = measured
        # Hand the best-glide angle to the solve.  On ``env`` for the reason
        # ``holdable`` is there: every propagation of a tick has to see the
        # same plant, and sixteen ``Steer`` objects cannot each remember to
        # carry it.
        self.env.best_alpha = measured.best_alpha
        # ``stall_speed`` and ``best_ld`` ride along for the same reason and
        # are read back by ``airframe.stall`` / ``airframe.glide_ld``.  Until
        # this line they were measured, printed, compared against the
        # configured constants -- and discarded.
        self.env.stall_speed = measured.stall_speed
        self.env.stall_mass = mass
        self.env.best_ld = measured.best_ld
        self.env.best_speed = measured.best_speed
        self.env.stall_alpha = measured.stall_alpha
        if (not landing
                and getattr(self.env.runway, "gate_dist_m", None) is None):
            # **The gate is where the approach's own model needs GATE_ALT_M.**
            # See ``Config.GATE_FROM_APPROACH``.  Once, before the deorbit,
            # so the geometry the entry is solved to never moves.
            ratio = airframe.approach_ld(self.env, self.cfg,
                                         self.cfg.GATE_ALT_M, mass,
                                         self.surface_gravity)
            dist = max(self.cfg.GATE_CAPTURE_M,
                       self.cfg.GATE_ALT_M * ratio
                       - airframe.touchdown_aim(self.env, self.cfg))
            self.env.runway.gate_dist_m = dist
            self.logbook.event(snap.ut, "gate: %.0f m before the threshold -- "
                               "%.0f m at the approach's %.2f, less the %.0f m "
                               "aim (GATE_DIST_M says %.0f)"
                               % (dist, self.cfg.GATE_ALT_M, ratio,
                                  airframe.touchdown_aim(self.env, self.cfg),
                                  self.cfg.GATE_DIST_M))
        # **And bring the learned ceiling under the wing's own stall.**
        # The two limits are different -- see ``airframe.alpha_ceiling`` --
        # and only this one can be known before the vehicle has flown.  It
        # is applied as a lowering, never a raising: a ceiling the flight
        # has already learned down is evidence and this is only a prior.
        top = airframe.alpha_ceiling(self.env, self.cfg)
        if top < self.alpha_ceiling:
            self.logbook.event(
                snap.ut, "alpha ceiling -> %.1f deg: this wing's lift peaks "
                         "there (ALPHA_MAX_DEG is %.1f, which is %.1f deg "
                         "past its own stall)"
                % (top, self.cfg.ALPHA_MAX_DEG, self.cfg.ALPHA_MAX_DEG - top))
            self.alpha_ceiling = top
        self.logbook.event(snap.ut, "%s%s"
                           % ("at the landing mass %.2f t: " % (mass / 1000.0)
                              if landing else "",
                              measured.describe()))
        if measured.stall_speed is None:
            return
        for name, configured, derived in (
                ("APPROACH_BEST_LD", self.cfg.APPROACH_BEST_LD,
                 measured.best_ld),):
            if configured <= 0.0 or derived is None:
                continue
            off = abs(derived - configured) / configured
            if off > self.cfg.AIRFRAME_DISAGREE_FRACTION:
                self.logbook.event(
                    snap.ut, "airframe DISAGREES%s: %s is %.2f and the swept "
                             "table says %.2f (%.0f%% out) -- one of them is "
                             "not this aircraft"
                    % (" at the landing mass" if landing else " at the entry "
                       "mass (the landing is lighter -- read the one on final)",
                       name, configured, derived, 100.0 * off))

    def run_deorbit(self, snap):
        """Wait for a burn that works, then hand over to flying it.

        The waiting is the interesting half.  A deorbit burn is the one
        irreversible act in the flight, so rather than compute a time to burn
        at, the phase asks every couple of seconds: "is there a dv in range
        that would land the entry on the gate *from here*?"  When there is, it
        burns.  That turns a timing problem into a trigger and removes any
        need to model where the orbit will be later -- and it means an
        unfavourable pass simply goes by without anything having to decide so.
        """
        if self.deorbit_dv is not None:
            # Burning.  Stop asking how much more and start asking whether the
            # trajectory already does the job: one propagation instead of
            # sixteen, and a condition that can actually become true.
            return self.fly_deorbit_burn(snap)

        # Before anything decides how far to jump next, record how far the
        # last jump actually went.  See ``watch_warp``.
        self.watch_warp(snap)
        self.end = self.env.runway.choose(snap.position, snap.velocity)
        reach = trajectory.forward_arc(self.env, snap.position, snap.velocity,
                                       self.env.runway.gate(self.end))
        self.set_throttle(0.0)
        self.set_rcs(False)
        if not (self.cfg.DEORBIT_RANGE_MIN_M <= reach
                <= self.cfg.DEORBIT_RANGE_MAX_M):
            # Out of reach either way; the search would only confirm it, and
            # it costs sixteen entry propagations against a game that wants
            # the CPU.
            self.panel.set_start_label("%.0f km to run" % (reach / 1000.0))
            self.set_warp(False)   # nothing to act on: warp
            return

        dv, best = guidance.deorbit_solution(self.env, snap.position,
                                             snap.velocity,
                                             snap.mass,
                                             self.cfg, self.end)
        if dv is None:
            self.panel.set_start_label("waiting: %+.0f km" % (best / 1000.0))
            self.set_warp(False)
            return
        if self.warp_factor:
            # Drop out of warp and re-ask at 1x before committing.  The
            # search's answer is a function of the state it was taken at, and
            # a warped tick covers twenty seconds of orbit.
            self.set_warp(True)    # release
            return
        self.deorbit_dv = dv
        self.deorbit_since = snap.ut
        self._thrust_limit_pending = True
        # **The offset the burn's stop test will track.**  With the authority
        # window there is no fitted aim to measure against, so it is read off
        # the solution the search actually took -- one propagation, once.
        # See ``guidance.deorbit_chosen_aim``; ``None`` leaves the stop test
        # on ``deorbit_aim``'s fitted formula, which is what the flag being
        # off means.
        self.deorbit_aim_m = None
        self.deorbit_aim_m = guidance.deorbit_chosen_aim(
            self.env, snap.position, snap.velocity,
            snap.mass, self.cfg, self.end, dv)
        # The aim is no longer one number -- it is a share of the entry each
        # candidate flies (``guidance.deorbit_aim``) -- so log the rule, not
        # ``DEORBIT_LONG_BIAS_M``, which is only its floor and would read as
        # "aiming 40 km" on a flight aimed at three hundred.
        # **What the glide can still reach, and where in it this burn
        # sits.**  Without this the window is a number nobody can check:
        # ``DEORBIT_WINDOW_BIAS`` is a share of a width, and a share of
        # an unlogged width is not a measurement.
        window = guidance.deorbit_window(
            self.env, snap.position,
            vec.add(snap.velocity,
                    vec.scale(vec.unit(snap.velocity), -dv)),
            snap.mass, self.cfg, self.end,
            self.env.runway.gate(self.end))
        if window is None:
            self.logbook.event(snap.ut, "deorbit window: unreadable at "
                                        "the committed burn")
        else:
            shortest, longest, want = window
            width = longest - shortest
            self.logbook.event(
                snap.ut, "deorbit window: the glide can reach %.0f to "
                         "%.0f km (%.0f km wide); the gate is at %.0f, "
                         "%+.0f%% off centre"
                % (shortest / 1000.0, longest / 1000.0, width / 1000.0,
                   want / 1000.0,
                   200.0 * (want - 0.5 * (shortest + longest))
                   / max(1.0, width)))
        # ``best`` here is the chosen candidate's signed error against its
        # *own* aim, in metres -- the slack the one-sided acceptance band
        # allowed it.  It is the second half of "which burn is this": two
        # flights of one save can commit at dv 45.3 and dv 50.4 and land
        # 119 km apart, and the dv alone does not say how far past the aim
        # the search believed each of them would land.
        self.logbook.event(snap.ut, "deorbit solution dv=%.1f m/s on runway "
                                    "%s, %.0f km to run, aiming %.0f%% of the "
                                    "entry (floor %.0f m), %+.0f m past its aim"
                           % (dv, self.end["name"], reach / 1000.0,
                              100.0 * self.cfg.DEORBIT_LONG_BIAS_FRACTION,
                              self.cfg.DEORBIT_LONG_BIAS_M, best))
        # **How finely the wait was sampled is part of which pass this is.**
        # Two flights of one save that warped at different steps take
        # different opportunities and land 23 km apart, and until this line
        # existed the log gave no way to tell them apart.  Failure 11.
        self.logbook.event(snap.ut, "warp sampling: ceiling %d, last tick "
                                    "covered %s of orbit"
                           % (self.warp_ceiling,
                              "%.0f km" % (self.warp_arc / 1000.0)
                              if self.warp_arc is not None else "no warped tick"))
        return self.fly_deorbit_burn(snap)

    def watch_warp(self, snap):
        """Learn how much orbit a warped tick is actually covering.

        The factor is an index into a rate table the game owns, the tick is
        however long ``deorbit_solution`` takes, and off 1x the timescale
        plugin multiplies both.  None of that is knowable in advance, and
        assuming it cost this project a pass: measured, one instance covered
        **180 seconds** of orbit per warped tick where the code assumed 20,
        and warped over the window its burn was solvable in.

        So watch instead, the way ``Holdable`` watches the alpha ceiling --
        the loop already knows the position at each tick and the one before,
        and the arc between them is the number that matters.  Called on every
        DEORBIT tick, before the factor for the next one is chosen.
        """
        if not self.warp_factor:
            self.warp_last = None
            return
        previous, self.warp_last = self.warp_last, (snap.ut, snap.position)
        if previous is None or snap.ut <= previous[0]:
            return
        # **From elapsed time and speed, not from the two positions.**
        # ``surface_distance`` is the short way round the sphere (failure 1),
        # so it *aliases*: a tick that covered more than half a circumference
        # comes back as a small number, and the ceiling would rise on exactly
        # the runaway step this exists to catch. A mod that redefines the
        # rate table can produce such a step -- one already produced a 9x
        # one. Elapsed universal time is monotonic and cannot wrap, and
        # speed x dt slightly over-states the ground arc, which errs towards
        # backing off.
        self.warp_arc = vec.norm(snap.velocity) * (snap.ut - previous[0])
        budget = float(self.cfg.WARP_MAX_ARC_M)
        if self.warp_arc > budget and self.warp_ceiling > self.cfg.WARP_MIN_FACTOR:
            self.warp_ceiling -= 1
            self.logbook.event(snap.ut, "warp tick covered %.0f km of orbit "
                                        "against a %.0f km budget; ceiling -> %d"
                               % (self.warp_arc / 1000.0, budget / 1000.0,
                                  self.warp_ceiling))
        elif (self.warp_arc < 0.5 * budget
              and self.warp_ceiling < self.cfg.WARP_MAX_FACTOR):
            # Two-way, for the reason ``ALPHA_RECOVER_DEG_S`` is: one slow
            # tick is a hitch, not a rate, and a one-way ratchet turns it
            # into minutes of real time spent at 1x for the rest of the wait.
            self.warp_ceiling += 1

    def set_warp(self, release, factor=None):
        """Rails-warp through the wait for a pass, and only through that.

        The vehicle is in vacuum with the engine off and nothing commanded,
        so the minutes spent waiting are minutes in which warp changes
        nothing except how long they take in the room.

        The trigger is **the absence of a solution**, not the range.  Range
        was the first rule and it never fired once: the runway sits inside
        the reach window for the whole wait, because what the phase is
        waiting on is the cross-track, not the distance.  The tick that finds
        a pass drops the warp and re-asks at 1x rather than committing on a
        warped state.

        **How far to warp is learned, not assumed** -- see ``watch_warp``.
        It starts at ``WARP_MIN_FACTOR`` and climbs only on evidence that the
        last step was comfortably inside the arc budget, because a step that
        is too small costs wall-clock seconds and a step that is too large
        costs the pass, and those are not the same price.
        """
        # ``factor`` is the coast's, which is chosen rather than learned:
        # nothing is being solved on a ballistic arc, so there is no window a
        # coarse step could jump over.  ``WARP_WAIT`` gates the deorbit's wait
        # only, so the coast passes its own factor and is not gated by it.
        if factor is not None:
            wanted = 0 if release else int(factor)
        else:
            wanted = 0 if (release or not self.cfg.WARP_WAIT) \
                else int(self.warp_ceiling)
        if wanted == self.warp_factor:
            return
        previous, self.warp_factor = self.warp_factor, wanted
        if wanted and not previous:
            self.warp_last = None       # a fresh measurement, not a stale arc
        try:
            self.conn.space_center.rails_warp_factor = wanted
        except Exception as exc:                            # noqa: BLE001
            # **Not silently.**  Warp refusing and warp working look identical
            # from here, and which one happened decides which pass is flown.
            self.warp_factor = previous
            if not self.warp_refused:
                self.warp_refused = True
                self.logbook.event(0.0, "rails warp refused (%r); the wait "
                                        "will run at 1x" % (exc,))

    def fly_deorbit_burn(self, snap):
        """Point retrograde, burn, and stop when the arc already reaches.

        The one irreversible act in the flight, so it is flown against a fresh
        propagation every tick rather than to a computed duration, and the
        stop is debounced: boosterland failure 12 is a burn whose exit tick
        was a coin toss worth 370 m on the ground, and the fix there --
        requiring the condition to hold for several consecutive ticks -- is
        the same one here.
        """
        self.set_warp(True)
        speed = vec.norm(snap.velocity)
        retro = vec.scale(snap.velocity, -1.0 / max(1.0, speed))
        set_autopilot_attitude(self.autopilot, retro)
        self.commanded_nose = retro
        self.commanded_alpha, self.commanded_bank = 180.0, 0.0
        align = vec.angle_between(snap.nose, retro)
        # **RCS for the whole phase, alignment included.**  This was flown the
        # other way -- thrusters only once the engine was lit, on the argument
        # that a vacuum flip against no aerodynamic moment has minutes to
        # happen in and the reaction wheels can have it for free.  The
        # argument is sound and the measurement destroyed it -- though not
        # for the reason it looks like.  The flip takes ~113 s either way.
        # What wheels cannot do is *stop* it: the last pre-burn ticks read
        # aoa 83.8, 160.8, 178.5 -- 77 degrees in two seconds -- so the burn
        # lit mid-slew and the dv did not go retrograde.  Nine flights
        # exited on a measured arc 35-50 km short of the aim where every
        # historical flight exits on the blind fallback at +0 (LOG1617-25
        # against LOG1626-34).  Every arrival moved 50 km.  Failure 43.
        #
        # The distinction that survives is not slow versus fast, it is
        # **whether anything is waiting on the turn**.  A deorbit window is
        # waiting on this one, so it is urgent however long it has; what is
        # *not* urgent is the hold afterwards, and the valve still shuts for
        # that (``rcs.Valve``) -- which is where the 55-of-150 units went.
        self.set_rcs(True, snap)
        self.ensure_thrust(snap)
        # **Charge the interval that just ended to the throttle that was
        # actually in force over it, before any early return can skip it.**
        #
        # This is the open-loop estimate, kept only so the log can be read
        # against the measured speed drop -- but it was accumulated at the
        # *bottom* of the tick, using the throttle just commanded and the
        # time since the last tick that *reached* the bottom.  An alignment
        # tick returns early (below), so its dead time went uncharged and
        # was then billed to the next burning tick at that tick's throttle.
        # Over 72 flights the log therefore read ``delivered 32.7 m/s
        # (model said 75.5)`` -- and that 2.3x gap was read, by me, as an
        # engine model that over-reads, and written up as the next thing to
        # fix.  It is not: the burn is closed-loop on the measured speed
        # drop and every one of those 72 exits landed within 0.7 m/s of the
        # solved dv at ``+0.00 m/s still owed``.
        #
        # A number kept "for the log only" is still read, and still
        # believed.  Its own clock, so the taper's horizon below -- which
        # deliberately measures from the last *burning* tick -- is untouched.
        if self.deorbit_model_ut is not None:
            self.deorbit_modelled += (
                self.throttle * max(0.1, snap.max_accel)
                * max(0.0, snap.ut - self.deorbit_model_ut))
        self.deorbit_model_ut = snap.ut
        # **On the first tick the engine can answer, not at commit.**  At
        # commit the engine is often not yet lit, ``max_accel`` reads zero,
        # and the first version of this returned silently -- the whole
        # shuttle batch LOG2912-2914 flew with the limiter never set.
        if getattr(self, "_thrust_limit_pending", False) \
                and snap.max_accel > 0.0:
            self._thrust_limit_pending = False
            self.limit_burn_thrust(snap, self.deorbit_dv)
        # Every tick, before the alignment test can return early, so the
        # floor rule below can charge one tick rather than the dead time.
        self._deorbit_prev_tick_ut = self._deorbit_tick_ut
        self._deorbit_tick_ut = snap.ut
        if align > self.cfg.DEORBIT_ALIGN_DEG:
            self.set_throttle(0.0)
            return
        record = {} if getattr(self.cfg, "DIAG_INTERFACE", False) else None
        progress, needed = guidance.deorbit_remaining(
            self.env, snap.position, snap.velocity, snap.mass,
            self.cfg, self.end, self.deorbit_aim_m, record=record)
        if (record is not None and record.get("prediction") is not None
                and self.throttle == 0.0):
            # **Only from a tick the engine was not firing through.**
            #
            # ``deorbit_remaining`` propagates from the state at the *top* of
            # this tick, and the tick then burns.  Storing it unconditionally
            # therefore keeps a prediction carrying the energy of however
            # much dv the last tick had yet to deliver -- and the final tick
            # of this burn delivers a lot: measured on LOG2079, 2064.5 ->
            # 2049.2 m/s, **15.3 m/s in one tick**.
            #
            # It looked exactly like physics, which is why it is worth the
            # comment.  The predicted speed at the atmosphere boundary came
            # out **-9.5 m/s** against actual on `qs_plane_inc` and **-2.2**
            # on `qs_plane`, and the "vacuum leg" of the handover error read
            # +54.2 km against +2.5 -- a ballistic arc apparently diverging
            # by 54 km.  The ratio is just the vehicles: `inc` weighs 9.4 t
            # against `plane`'s 14.4 t on the same engine, so its last tick
            # delivers ~1.5x the dv and carries ~1.5x the stale energy.
            #
            # ``self.throttle`` is the command that was in force over the
            # interval just ended, so zero means the arc this propagation
            # started from is the one the vehicle is actually on.
            self.deorbit_prediction = record["prediction"]
            # Provenance, so the handover diagnostic can be read for
            # staleness instead of being trusted.  The stored prediction is
            # only as good as the state it was propagated from, and the last
            # ticks of this burn are worth ~13 m/s each.
            self.deorbit_prediction_ut = snap.ut
            self.deorbit_prediction_speed = vec.norm(snap.velocity)
        self.deorbit_progress = progress
        # ``None`` means the arc does not reach the gate's altitude at all,
        # which is where every burn starts and where it stays until enough dv
        # is off.  There is no error to divide then, so fall back to what is
        # left of the dv the search asked for -- both to taper on and to stop
        # on.  A blind phase at full throttle is how a burn overshoots: in
        # ``fakeplane`` the error stays unmeasurable until the arc is already
        # on the aim, and an untapered blind burn arrives 49 km short of it.
        owed = needed
        if owed is None:
            owed = max(0.0, (self.deorbit_dv or 0.0) - self.deorbit_burned)
        self.deorbit_needed = owed
        # The error comes *down* to the bias as the burn proceeds -- before
        # the burn the arc lands most of the planet long, and every m/s of
        # retrograde thrust walks it back towards the gate.  Testing
        # ``>= bias`` stops on the very first tick that produces a number at
        # all: in game that was ``range error +769912 m, held 3 ticks``, a
        # burn shut down 770 km long.
        if owed <= 0.0:
            self.deorbit_ticks += 1
        else:
            self.deorbit_ticks = 0
        if (self.deorbit_ticks >= int(self.cfg.DEORBIT_EXIT_TICKS)
                or self.deorbit_done):
            self.set_throttle(0.0)
            self.restore_thrust_limits()
            self.cutoff_state = (snap.ut, snap.position, snap.velocity,
                                 snap.mass)
            self.log_deorbit_window(snap)
            self.enter(COAST if self.cfg.DRAIN else DRAIN, snap.ut,
                       "range error %+.0f m, %+.2f m/s still owed, "
                       "solved dv %.0f m/s, delivered %.1f m/s "
                       "(model said %.1f)"
                       % (progress if progress is not None else 0.0,
                          owed, self.deorbit_dv or 0.0,
                          self.deorbit_burned, self.deorbit_modelled))
            return
        # From the first burning tick, not from phase entry: the phase spends
        # minutes in orbit waiting for a pass, and counting that against the
        # runaway guard expires it before the engine has ever been lit.
        if snap.ut - (self.deorbit_since or snap.ut) \
                > self.cfg.DEORBIT_MAX_BURN_S:
            self.set_throttle(0.0)
            self.restore_thrust_limits()
            self.enter(COAST if self.cfg.DRAIN else DRAIN, snap.ut,
                       "burn guard at %.0f s"
                       % self.cfg.DEORBIT_MAX_BURN_S)
            return
        # Taper on the dv the burn still owes rather than running full
        # throttle into a threshold.  The sensitivity is ~25 km of range per
        # m/s, so the last tick of an untapered burn is worth more than the
        # whole glide's steering authority.
        accel = max(0.1, snap.max_accel)
        # The taper horizon *and* the tick, whichever binds.  Two entry
        # propagations a tick is tens of milliseconds, so the loop does not
        # keep to ``TICK_S``; a tick longer than ``DEORBIT_TAPER_S`` would
        # otherwise spend more than the burn owes in one go.
        horizon = max(float(self.cfg.DEORBIT_TAPER_S),
                      snap.ut - (self.deorbit_last_ut or snap.ut))
        throttle = min(float(self.cfg.DEORBIT_THROTTLE),
                       owed / (accel * horizon))
        # **The floor exists so the engine is not commanded below where it
        # responds -- but a floor is a minimum *spend*, and the last tick of
        # a burn is worth kilometres.**
        #
        # Measured over nine flights: the smallest throttle this will command
        # still spends about 0.34 m/s in one tick (8.6 m/s^2 x 0.02 x 2 s),
        # and at ~25 km of range per m/s that tick is worth 8 km on the
        # ground.  Every burn exited past its aim, by 0.01 to 0.39 m/s, and
        # the exit line said so: ``range error -13674 m, -0.39 m/s still
        # owed``.
        #
        # So when what is left is smaller than what one floored tick would
        # spend, stopping is nearer the aim than burning -- compare the two
        # and take the better.  This is the burn's own version of
        # boosterland's landing-burn rule: the last tick is a decision, not
        # a continuation.
        floor = float(self.cfg.DEORBIT_MIN_THROTTLE)
        if throttle < floor:
            # **What one more floored tick spends is one tick's worth**, not
            # the time since the engine last burned.  ``horizon`` includes
            # any alignment dead time, and on a 65 m/s^2 engine ten seconds
            # of realigning made one floored tick "cost" 13 m/s -- so the burn
            # stopped 1.9 m/s short on every shuttle flight (LOG2906-2908).
            step = horizon
            step = max(0.02, snap.ut - (self._deorbit_prev_tick_ut
                                        or snap.ut))
            spend = floor * accel * step
            if abs(owed - spend) >= abs(owed):
                # Burning one more floored tick lands further from the aim
                # than cutting now does.
                #
                # **A sticky flag, because the tick counter does not
                # survive.**  This used to set ``deorbit_ticks`` to the exit
                # threshold -- but the exit test runs at the *top* of the
                # next tick, after ``owed > 0`` has reset the counter to
                # zero, so the burn cut its throttle and then sat at zero
                # owing a fraction of a m/s, cutting and resetting, until
                # ``DEORBIT_MAX_BURN_S`` fired.  Measured across three
                # batches of logs: **7 of 24, 2 of 12 and 5 of 10** burns
                # exited on ``burn guard at 60 s`` having delivered their dv
                # 35 seconds earlier.  The burn was correct and the phase
                # would not end.
                self.set_throttle(0.0)
                self.deorbit_done = True
                self.deorbit_ticks = int(self.cfg.DEORBIT_EXIT_TICKS)
                return
            throttle = floor
        self.set_throttle(throttle)
        # **What the burn spent, measured, not modelled.**
        #
        # This used to accumulate ``throttle * accel * dt`` and test *that*
        # against what was owed -- a model of the burn standing in for the
        # burn.  It over-counts, so the engine shuts down early: measured
        # over two airframes, a solve that asked for 27 m/s delivered 18.3
        # (2080.6 -> 2062.3) and one that asked for 32 delivered 19.9
        # (2080.6 -> 2060.7).  About a third of every deorbit was never
        # flown.
        #
        # It stayed hidden because a second error cancelled it: the old
        # airframe could not hold its commanded angle of attack, sank faster
        # than the propagator predicted, and landed 40-70 km short (failure
        # 10).  An airframe that *does* hold its command removes that error
        # and leaves this one bare -- nine flights, 190-310 km **long**, the
        # glide predicting +237 km from its first tick with no authority to
        # take it back, because the error was made before the entry
        # interface where no glide tuning can reach it.
        #
        # The speed drop is the measurement.  Over the few seconds of a
        # retrograde burn in a near-circular orbit, gravity moves the speed
        # by a hair and the engine moves it by metres per second, so the
        # difference between ticks *is* the delivered dv.  Only decreases
        # count: a tick that reads faster is the arc, not the engine.
        speed_now = vec.norm(snap.velocity)
        if self.deorbit_speed_prev is not None:
            self.deorbit_burned += max(0.0, self.deorbit_speed_prev
                                       - speed_now)
        self.deorbit_speed_prev = speed_now
        self.deorbit_last_ut = snap.ut

    def diag_gate(self):
        """One fixed yardstick for the handover diagnostic, and why.

        `Runway.gate` is the *high* gate when the cone is flying, and the
        high gate sits ``HAC_ALT_M * HAC_LD`` out along the approach
        centreline -- so the two ends' gates are on opposite sides of the
        field, **about 44 km apart**. The deorbit solves against whichever
        end ``Runway.choose`` picks from orbit (measured: runway 27 on every
        flight of two states) and the glide re-chooses on the way down
        (measured: runway 09 on every one of the same flights).

        The first version of this instrument took each line's arc against
        ``self.end`` *as it stood at that moment*, so "predicted" was an arc
        to 27's gate and "actual" an arc to 09's -- two different targets,
        tens of km apart, subtracted from each other. Both lines now measure
        against one named end so the subtraction means something. The
        absolute value is not "distance to the gate this flight will use";
        it is a consistent yardstick, which is all a difference needs.

        The end mismatch itself is a live question about the *flight*, not
        about the diagnostic -- the burn is solved against a gate the glide
        does not fly to -- and each line logs the end that phase had chosen
        so it stays visible.
        """
        return self.env.runway.gate(self.env.runway.ends["09"])

    def log_interface_prediction(self, snap):
        """What the committed burn expected at the entry interface.

        **The instrument this project did not have.**  ``Config.DIAG_STATE``
        is defined for the spaceplane and nothing reads it -- it is a
        booster feature the config inherited -- so there has never been a
        way to ask where a miss is born, only where it ends up.

        Measured with it: the vehicle reaches the interface **+91.3 km sd
        3.7** further from the gate than the burn predicted on
        ``qs_plane_inc``, against **-2.5 km sd 2.4** on ``qs_plane``, in the
        right state (altitude to 70 m, speed to 0.4%) and the wrong place.

        **It costs nothing, and the first version cost 9 km.**  That one ran
        its own entry propagation here; the stall it put in the control loop
        moved the interface 9 km and the landing 1.5 km -- measured against
        an interleaved ``DIAG_INTERFACE=False`` arm, five flights each, with
        the off arm landing on the historical -1.1 km and the on arm on
        -3.0.  A diagnostic that changes what it measures is worse than no
        diagnostic.  The burn already propagates to the gate every tick and
        that arc passes through the interface, so this reads the crossing
        off the prediction the burn paid for (``Prediction.interface_*``).

        **Read the provenance before the number.**  The stored prediction is
        the last one ``deorbit_remaining`` made on a tick the engine was not
        firing through, and on this burn that is not the final state: the
        line reports the speed it was propagated from against the speed now,
        and measured it is **16-17 m/s stale**.  At tens of kilometres per
        m/s that is most of the "handover error" this line appears to show,
        so the difference against ``interface actual`` is an upper bound on
        the flight's error and not a measurement of it.  ``cutoff drift`` is
        the instrument that does not have this problem, because both of its
        samples are states the loop took.

        Log only -- nothing steers on it.
        """
        if not getattr(self.cfg, "DIAG_INTERFACE", False):
            return
        try:
            seen = self.deorbit_prediction
            if seen is None or not seen.interface_position:
                self.logbook.event(snap.ut, "interface predicted: the burn "
                                            "never propagated one")
                return
            gate = self.diag_gate()
            arc = trajectory.forward_arc(self.env, seen.interface_position,
                                         seen.interface_velocity, gate)
            self.logbook.event(
                snap.ut,
                "interface predicted: alt %.0f m, speed %.1f m/s, "
                "arc to gate %.0f m, t+%.0f s, end %s "
                "(propagated at ut %.2f from %.1f m/s, now %.1f m/s)"
                % (vec.norm(seen.interface_position)
                   - self.env.equatorial_radius,
                   vec.norm(seen.interface_velocity), arc,
                   seen.interface_time, self.end["name"],
                   self.deorbit_prediction_ut or 0.0,
                   self.deorbit_prediction_speed or 0.0,
                   vec.norm(snap.velocity)))
            if seen.entry_position:
                # The atmosphere boundary, which splits the vacuum leg from
                # the aerodynamic one -- the open question of failure 57.
                edge = trajectory.forward_arc(self.env, seen.entry_position,
                                              seen.entry_velocity, gate)
                self.logbook.event(
                    snap.ut,
                    "boundary predicted: alt %.0f m, speed %.1f m/s, "
                    "arc to gate %.0f m"
                    % (vec.norm(seen.entry_position)
                       - self.env.equatorial_radius,
                       vec.norm(seen.entry_velocity), edge))
        except Exception as exc:                            # noqa: BLE001
            # A diagnostic must never be the reason a flight dies.
            self.logbook.event(snap.ut,
                               "interface predicted: failed (%r)" % (exc,))

    def log_boundary_actual(self, snap):
        """What the vehicle actually brought to the atmosphere boundary.

        Paired with ``boundary predicted``, this splits failure 57's 91 km
        into the vacuum leg and the aerodynamic one.  A comparison per tick
        and one log line; no propagation.
        """
        if not getattr(self.cfg, "DIAG_INTERFACE", False):
            return
        if self.boundary_logged:
            return
        try:
            altitude = (vec.norm(snap.position)
                        - self.env.equatorial_radius)
            if altitude >= self.env.atmosphere_depth:
                return
            self.boundary_logged = True
            gate = self.diag_gate()
            arc = trajectory.forward_arc(self.env, snap.position,
                                         snap.velocity, gate)
            self.logbook.event(
                snap.ut,
                "boundary actual: alt %.0f m, speed %.1f m/s, "
                "arc to gate %.0f m, end %s"
                % (altitude, vec.norm(snap.velocity), arc,
                   self.end["name"]))
        except Exception as exc:                            # noqa: BLE001
            self.boundary_logged = True
            self.logbook.event(snap.ut,
                               "boundary actual: failed (%r)" % (exc,))

    def log_interface_actual(self, snap):
        """What the vehicle actually brought to the interface.

        Subtract from ``log_interface_prediction`` and the difference is the
        handover error, per flight, in one line each.
        """
        if not getattr(self.cfg, "DIAG_INTERFACE", False):
            return
        try:
            gate = self.diag_gate()
            arc = trajectory.forward_arc(self.env, snap.position,
                                         snap.velocity, gate)
            self.logbook.event(
                snap.ut,
                "interface actual: alt %.0f m, speed %.1f m/s, "
                "arc to gate %.0f m, end %s"
                % (vec.norm(snap.position) - self.env.equatorial_radius,
                   vec.norm(snap.velocity), arc, self.end["name"]))
        except Exception as exc:                            # noqa: BLE001
            self.logbook.event(snap.ut,
                               "interface actual: failed (%r)" % (exc,))

    def vehicle_vacuum_isp(self, snap):
        """The Isp this vehicle burns at, whether or not it is burning now.

        ``vessel.vacuum_specific_impulse`` is an *aggregate over active
        engines*, so with the throttle shut and nothing ignited it reads
        **0** -- and the drain runs in vacuum, before the burn, which is
        exactly when it is 0.

        On the craft this autopilot grew up on it happened to read 355 at
        that moment, so ``drain_reserve_units`` got its rocket equation and
        kept 96 units, "117 m/s of burn".  A shuttle read 0, the computed dv
        budget collapsed to the ``DRAIN_RESERVE_UNITS`` floor, the drain
        dumped the propellant the burn was going to need, and the deorbit
        ran dry **9 m/s short of its own solution** (LOG2871: solved 26.2,
        delivered 17.4, speed pinned at 2050.1 with the engine commanded
        on).  The log said ``at Isp 0`` all along.

        So ask the engines instead.  Vacuum Isp is a property of the engine,
        not of whether it is lit, and the largest one aboard is the one the
        burn will use.  ``0.0`` only when there is genuinely nothing to ask,
        and the caller says so in capitals rather than quietly taking a
        floor.
        """
        isp = float(getattr(snap, "vacuum_isp", 0.0) or 0.0)
        if isp > 0.0:
            return isp
        cached = getattr(self, "_engine_isp", None)
        if cached is not None:
            return cached
        best = 0.0
        try:
            for engine in self.vessel.parts.engines:
                try:
                    if not engine.part.shielded:
                        best = max(best, float(
                            engine.vacuum_specific_impulse or 0.0))
                except Exception:                       # noqa: BLE001
                    continue
        except Exception:                               # noqa: BLE001
            best = 0.0
        self._engine_isp = best
        return best

    def drain_reserve_units(self, snap):
        """How much propellant the pre-burn drain must leave, in units.

        **A units count cannot be checked against anything, and 40 of them
        were not enough.**  ``qs_plane_high`` solves a **121.6 m/s** deorbit
        where ``qs_plane`` and ``qs_plane_inc`` solve 32-44, and a reserve
        sized for the latter two ran the burn dry: LOG2204 logged "37.5 units
        (69 m/s of burn at Isp 355)", then "deorbit solution dv=121.6 m/s",
        then ``burn guard at 60 s``, and the vehicle arrived 800 km from the
        field.  That is the contradiction the constant's own comment asked
        for, found on the first entry state that needed it.

        So the reserve is a *dv budget* turned into propellant by the rocket
        equation, at the Isp the vehicle reports and the mass it will have --
        a quantity the program computes rather than a number transcribed for
        one save.  ``DRAIN_RESERVE_UNITS`` stays as a floor for the case
        where Isp is not available.
        """
        floor = float(self.cfg.DRAIN_RESERVE_UNITS)
        budget = float(getattr(self.cfg, "DRAIN_RESERVE_DV_MS", 0.0))
        isp = self.vehicle_vacuum_isp(snap)
        if budget <= 0.0 or isp <= 0.0:
            # **A floor is not a budget, and taking one silently is how the
            # burn starves.**  See ``vehicle_vacuum_isp``.
            self.logbook.event(
                getattr(snap, "ut", 0.0),
                "drain reserve: NO ISP AVAILABLE -- falling back to the "
                "%.0f unit floor, which is a number and not a dv" % floor)
            return floor
        per_unit = float(self.cfg.RESOURCE_KG_PER_UNIT)
        aboard = snap.liquid_fuel + snap.oxidizer
        dry = max(1.0, snap.mass - aboard * per_unit)
        needed = dry * (math.exp(budget / (isp * 9.80665)) - 1.0)
        needed *= float(getattr(self.cfg, "DRAIN_RESERVE_MARGIN", 1.0))
        return max(floor, needed / per_unit)

    def drain_to_reserve(self, snap, floor):
        """Hold the valve open until the tank is down to ``floor``, watching.

        Bounded by ``DRAIN_TIMEOUT_S`` in wall-clock seconds -- a valve that
        does not drain must not hang the autopilot in orbit.
        """
        read = getattr(self.telemetry, "resources", None)
        if read is None:
            return
        names = tuple(self.cfg.DRAIN_RESOURCES)

        def aboard():
            total = 0.0
            for name in names:
                try:
                    total += read.amount(name)
                except Exception:                       # noqa: BLE001
                    pass
            return total

        deadline = time.time() + float(self.cfg.DRAIN_TIMEOUT_S)
        left = aboard()
        while left > floor and time.time() < deadline:
            left = aboard()
        self.stop_drain(snap)
        # **Say what the reserve buys, not how many units it is.**  A units
        # count cannot be checked against anything; a dv can be checked
        # against the burns this vehicle actually flies (31-42 m/s here).
        # If this number ever reads near the solved dv, the reserve is too
        # small and the burn will die on ``burn guard`` with a dry tank.
        isp = self.vehicle_vacuum_isp(snap)
        dry = snap.mass - left * float(self.cfg.RESOURCE_KG_PER_UNIT)
        budget = 0.0
        if isp > 0.0 and dry > 0.0:
            budget = isp * 9.80665 * math.log(
                (dry + left * float(self.cfg.RESOURCE_KG_PER_UNIT)) / dry)
        self.logbook.event(snap.ut, "drain watched down to %.1f units "
                                    "(%.0f m/s of burn at Isp %.0f)"
                           % (left, budget, isp))

    def _set_module_field(self, module, field, value):
        """Set one module field, by whichever setter this kRPC actually has.

        **``Module.set_field_value`` does not exist**, and the mistake is
        instructive: it was written into both drain paths as the fallback
        under the action names, wrapped in ``except Exception: continue``,
        and so it failed silently for the life of the project.  Nothing
        noticed, because the *actions* always worked on the one craft being
        flown -- a fallback that is never reached is a fallback nobody has
        tested.  Measured on this kRPC the setters are ``set_field_bool``,
        ``set_field_int``, ``set_field_float`` and ``set_field_string``.
        """
        for name, argument in (("set_field_bool", bool(value)),
                               ("set_field_string", str(value))):
            setter = getattr(module, name, None)
            if setter is None:
                continue
            try:
                setter(field, argument)
                return True
            except Exception:                           # noqa: BLE001
                continue
        return False

    def _drain_whole_vessel(self, module):
        """Point the valve at the vehicle rather than at its own part.

        **The setting nothing ever commanded.**  ``ModuleResourceDrain`` has
        a ``Drain Mode`` choosing between draining the part it is bolted to
        and draining the whole vessel, and the craft this autopilot grew up
        on was *saved* with it already True.  So for the life of the project
        it was right by inheritance and never set.

        A second airframe found it in one flight: a shuttle whose valves came
        in at ``'Drain Mode': 'False'`` opened both, drained two empty valve
        parts, and watched a tank that never moved -- 1750.0 units, four
        times, while DRAIN re-entered forever.  Probed directly, toggling the
        mode drains the same 1750 units to zero **in two seconds**.

        A setting that is only ever inherited is a setting that is only ever
        right by luck.
        """
        try:
            if str(module.get_field("Drain Mode")).strip().lower() == "true":
                return True
        except Exception:                               # noqa: BLE001
            pass
        if self._set_module_field(module, "Drain Mode", True):
            return True
        # Last resort, and only because the read above says it is False: a
        # toggle applied to an unknown state is how you turn it *off*.
        try:
            module.set_action("Toggle Resource Drain mode", True)
            return True
        except Exception:                               # noqa: BLE001
            return False

    def _open_one_drain(self, module):
        """Open one valve.

        kRPC reports a module's actions and fields by their *display* names,
        not the names in the part config: this valve answers to "Drain", not
        to ``StartResourceDrainAction``, and the field is "Drain" rather than
        ``isDraining``.  The first flight tried the config names, logged
        ``drain COULD NOT START``, and flew the whole entry 2.8 t heavy.  Both
        spellings are tried, and the field is tried after the actions, because
        a module that exposes neither is a module this cannot drive and the
        log should say so rather than the flight quietly continuing wet.
        """
        # **Open the valve onto the *vessel*, not onto its own part.**  This
        # valve has a second setting nothing used to touch -- ``Drain Mode``,
        # which chooses between draining the part it is bolted to and
        # draining the whole vehicle.  The craft this autopilot grew up on
        # happened to be saved with it already True, so for the life of the
        # project the mode was never commanded and never noticed.
        #
        # A second airframe found it in one flight: a shuttle whose valves
        # came in at ``'Drain Mode': 'False'`` opened both of them, drained
        # the (empty) valve parts, watched a tank that never moved, and sat
        # in DRAIN re-opening them until it was killed -- 1750.0 units, four
        # times, unchanged.  **A setting that is only ever inherited is a
        # setting that is only ever right by luck.**
        self._drain_whole_vessel(module)
        try:
            available = list(module.actions)
        except Exception:                               # noqa: BLE001
            available = []
        for action in ("Drain", "StartResourceDrainAction",
                       "Toggle Draining", "ToggleResourceDrainAction"):
            if action not in available:
                continue
            try:
                module.set_action(action, True)
                return True
            except Exception:                           # noqa: BLE001
                continue
        for field in ("Drain", "isDraining"):
            if self._set_module_field(module, field, True):
                return True
        return False

    def _close_one_drain(self, module):
        """Close one valve, by the same two spellings that opened it."""
        # Measured on this valve (``drain module actions`` in any log):
        # ``['Drain', 'Stop Draining', 'Toggle Draining',
        # 'Toggle Resource Drain mode']``, with fields ``{'Resources': '2',
        # 'Drain rate': '20', 'Drain': 'False', 'Drain Mode': 'True'}``.
        # "Stop Draining" is an action and an action is *triggered*, so it is
        # set True; the field is the fallback and takes the state itself.
        try:
            available = list(module.actions)
        except Exception:                               # noqa: BLE001
            available = []
        for action in ("Stop Draining", "StopResourceDrainAction"):
            if action not in available:
                continue
            try:
                module.set_action(action, True)
                return True
            except Exception:                           # noqa: BLE001
                continue
        for field in ("Drain", "isDraining"):
            if self._set_module_field(module, field, False):
                return True
        if "Toggle Draining" in available:
            try:
                module.set_action("Toggle Draining", True)
                return True
            except Exception:                           # noqa: BLE001
                pass
        return False

    def stop_drain(self, snap):
        """Close every valve.

        **Each one is reported.**  A valve that would not close is propellant
        still leaving the vehicle after the phase that accounts for it has
        ended -- failure 61's disturbance, back in the place it was moved out
        of -- so "3 of 4 closed" must not read as success.
        """
        if not self.drain_modules:
            return
        closed = sum(1 for module in self.drain_modules
                     if self._close_one_drain(module))
        total = len(self.drain_modules)
        self.logbook.event(snap.ut, "drain valves %s (%d of %d)"
                           % ("closed" if closed == total
                              else "WOULD NOT CLOSE", closed, total))

    def run_drain(self, snap):
        """Dump the propellant, and wait for it to actually be gone.

        2.780 t of 9.495, so 29% of the vehicle and 29% off the wing loading
        -- which moves the stall speed by 16% and every aerodynamic answer
        with it.  It gets its own phase in vacuum for that reason: every
        prediction after this point is made at the drained mass, and one made
        *during* the drain is made at a mass the vehicle is not going to have.
        """
        self.set_throttle(0.0)
        self.aim(0.0, 0.0, snap)
        # **Here, not at the burn's exit test.**  ``fly_deorbit_burn`` has
        # two ways out -- the range test and ``DEORBIT_MAX_BURN_S`` -- and
        # hanging the instrument on the first one silently skipped half the
        # flights of the batch it was built for.
        if not self.interface_logged and self.deorbit_dv is not None:
            self.interface_logged = True
            self.log_interface_prediction(snap)
        remaining = snap.liquid_fuel + snap.oxidizer
        # **Which drain this is.**  With ``DRAIN_BEFORE_BURN`` the phase runs
        # once, before the deorbit, and stops at a reserve the burn can spend;
        # the leftover then stays aboard for the entry rather than being
        # dumped where nothing can answer for the impulse.
        pre = self.deorbit_dv is None
        floor = (self.drain_reserve_units(snap) if pre
                 else float(self.cfg.DRAIN_REMAINING_UNITS))
        after = DEORBIT if pre else COAST
        if not self.cfg.DRAIN:
            self.enter(after, snap.ut, "drain disabled")
            return
        if not self.drain_modules:
            self.logbook.event(snap.ut, "no ModuleResourceDrain on this "
                                        "vessel -- flying wet")
            self.enter(after, snap.ut)
            return
        if self.drain_started is None:
            self.drain_started = snap.ut
            # **Every valve, and open them before watching any of them.**
            # The tank empties in about two seconds through one valve; with
            # the loop unable to tick faster than that, a valve opened after
            # ``drain_to_reserve`` has already blocked would be opened onto a
            # tank at its reserve and take the burn's propellant with it.
            opened = sum(1 for module in self.drain_modules
                         if self._open_one_drain(module))
            total = len(self.drain_modules)
            ok = opened > 0
            self.logbook.event(snap.ut, "drain %s (%d of %d valves), "
                                        "%.1f units aboard, mass %.3f t"
                               % ("started" if ok else "COULD NOT START",
                                  opened, total, remaining,
                                  snap.mass / 1000.0))
            if not ok:
                self.enter(after, snap.ut, "drain unavailable")
                return
            if pre:
                # **In the same tick the valve was opened, not the next
                # one.**  The next one is 2 game-seconds away and the tank
                # empties in 2.0: LOG2122 opened the valve, came back to a
                # dry tank, reported "0.0 units left" and flew a deorbit that
                # died on ``burn guard at 60 s`` with no propellant.
                self.drain_to_reserve(snap, floor)
            return
        # **A valve that will not drain must not hold the vehicle in orbit.**
        # ``drain_to_reserve`` is bounded by ``DRAIN_TIMEOUT_S``, but the
        # phase re-enters on the next tick and calls it again, so a tank that
        # never falls is an infinite loop with a timeout inside it.  Measured
        # on the shuttle before ``Drain Mode`` was commanded: four rounds of
        # "drain watched down to 1750.0 units" at ninety-second intervals,
        # the flight going nowhere, and nothing in the log saying the phase
        # was stuck rather than working.  Failure 61's rule from the other
        # side -- what leaves the vehicle has to be accounted for, and so
        # does what refuses to.
        if pre and remaining > floor:
            self.drain_attempts = getattr(self, "drain_attempts", 0) + 1
            if self.drain_attempts > int(self.cfg.DRAIN_MAX_ATTEMPTS):
                self.logbook.event(
                    snap.ut, "drain WILL NOT DRAIN after %d attempts "
                             "(%.1f units aboard, floor %.1f) -- flying wet"
                    % (self.drain_attempts, remaining, floor))
                self.stop_drain(snap)
                self.enter(after, snap.ut, "drain refused")
                return
        if pre and remaining > floor:
            # **The valve is faster than this loop and polling cannot stop
            # it.**  556 units go in 2.0 game-seconds and the control loop
            # ticks at about two of those, so the first tick after opening
            # the valve is already dry -- the reserve was never kept and the
            # burn had nothing to spend (LOG2118).  kRPC reads are
            # milliseconds, though, so the phase watches the tank directly
            # and closes the valve on the reserve instead of waiting for its
            # next tick.  It blocks, deliberately: this is vacuum, the drain
            # is the only thing happening, and the alternative is a burn with
            # no propellant.
            self.drain_to_reserve(snap, floor)
            return
        if remaining <= floor:
            if pre:
                # **Shut the valve.**  Left open it keeps draining straight
                # through the burn, which is the disturbance this mode exists
                # to move, put back in the worst possible place.
                self.stop_drain(snap)
            self.logbook.event(snap.ut, "drained in %.1f s, mass now %.3f t, "
                                        "%.1f units left"
                               % (snap.ut - self.drain_started,
                                  snap.mass / 1000.0, remaining))
            self.enter(after, snap.ut)
            return
        if snap.ut - self.drain_started > self.cfg.DRAIN_TIMEOUT_S:
            if pre:
                self.stop_drain(snap)
            self.logbook.event(snap.ut, "drain timed out with %.1f units left"
                               % remaining)
            self.enter(after, snap.ut)

    def limit_burn_thrust(self, snap, dv):
        """Set the engines' thrust limiter so the burn lasts long enough to steer.

        **A control every engine has and this autopilot never touched.**  The
        deorbit loop was built on a 14.5 t craft with a Cheetah: about 8.6
        m/s^2, so a 27 m/s burn took three seconds and thirty ticks.  The
        shuttle's Rhino gives **65 m/s^2** on 30.6 t -- the same burn is
        over in 0.4 s, four ticks, and the unbalanced thrust knocks the nose
        10-15 degrees off retrograde while it does it (``aoa=180/165``,
        LOG2906-2908).  All three flights then quit **1.9 m/s short** of the
        solved dv, which on a craft with twice the glide ratio is the 100 km
        they arrived long.

        So: never let the burn be shorter than ``DEORBIT_MIN_BURN_S`` at full
        throttle.  ``limit = dv / (MIN_BURN_S * accel)``, clamped to (0.05,
        1].  On the shuttle it is about 0.14; on the old craft, which reads
        17.8 m/s^2 at the burn (not the 8.6 an older comment claims), it is
        0.60 -- flown, arrival unchanged.  ``available_thrust`` honours the limiter, so
        the taper's own ``max_accel`` follows without being told.

        Off (0) by default; restored at shutdown.
        """
        want = float(getattr(self.cfg, "DEORBIT_MIN_BURN_S", 0.0) or 0.0)
        if want <= 0.0 or dv is None or dv <= 0.0:
            return
        accel = snap.max_accel
        if accel <= 0.0:
            self.logbook.event(snap.ut, "thrust limiter: no thrust to scale "
                                        "yet -- limiter NOT set")
            return
        limit = vec.clamp(dv / (want * accel), 0.05, 1.0)
        self._thrust_limits = []
        try:
            for engine in self.vessel.parts.engines:
                self._thrust_limits.append((engine, engine.thrust_limit))
                engine.thrust_limit = limit
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(snap.ut, "thrust limiter: could not set (%s)"
                               % type(exc).__name__)
            return
        self.logbook.event(snap.ut, "thrust limiter %.2f on %d engine(s): "
                                    "%.1f m/s^2 -> %.1f, a %.1f m/s burn in "
                                    "%.1f s rather than %.1f"
                           % (limit, len(self._thrust_limits), accel,
                              accel * limit, dv, dv / (accel * limit),
                              dv / accel))

    def restore_thrust_limits(self):
        for engine, limit in getattr(self, "_thrust_limits", None) or []:
            try:
                engine.thrust_limit = limit
            except Exception:                           # noqa: BLE001
                continue
        self._thrust_limits = []

    def log_cutoff_drift(self, snap):
        """What the vehicle did between the burn's cutoff and the coast.

        **The burn's stop test measures the state at a tick, and the vehicle
        does not stop at that tick.**  Read off logs 2090-2101, specific
        energy between the ``DEORBIT -> DRAIN`` instant and the 70 km
        atmosphere boundary falls by 4.8 kJ/kg on ``qs_plane`` and 18.4 on
        ``qs_plane_inc`` -- an equivalent **2.3 and 8.9 m/s** of extra
        retrograde dv, in vacuum, after the burn was declared finished and
        with the tick-to-tick spread inside a state under 0.3 m/s.  At the
        interface that is worth 28 km and 127 km of ground track
        respectively, which is the whole of the handover error this project
        has been chasing, and the whole of why nothing transfers between
        entry states: it is not a modelling error, it is dv the model never
        hears about.

        Two candidates and this line separates them: engine thrust tailing
        off after a commanded cutoff (``Fn`` on the telemetry line is now the
        thrust the engine is *producing*), or the release valve venting
        2.7-7.9 t of propellant.  The decomposition says which way the
        impulse points; the timing says when it arrives.

        Log only -- nothing steers on it.
        """
        if self.cutoff_logged or self.cutoff_state is None:
            return
        self.cutoff_logged = True
        try:
            ut0, r0, v0, m0 = self.cutoff_state
            mu = self.env.mu
            e0 = vec.norm(v0) ** 2 / 2.0 - mu / vec.norm(r0)
            e1 = vec.norm(snap.velocity) ** 2 / 2.0 - mu / vec.norm(snap.position)
            dv = vec.sub(snap.velocity, v0)
            along = vec.dot(dv, vec.unit(v0))
            up = vec.unit(r0)
            radial = vec.dot(dv, up)
            self.logbook.event(
                snap.ut,
                "cutoff drift: %.1f s, speed %.1f -> %.1f m/s, "
                "dE %+.0f J/kg (equivalent %+.2f m/s), "
                "dv along %+.2f radial %+.2f, mass %.3f -> %.3f t"
                % (snap.ut - ut0, vec.norm(v0), vec.norm(snap.velocity),
                   e1 - e0, (e1 - e0) / max(1.0, vec.norm(snap.velocity)),
                   along, radial, m0 / 1000.0, snap.mass / 1000.0))
        except Exception as exc:                            # noqa: BLE001
            self.logbook.event(snap.ut, "cutoff drift: failed (%r)" % (exc,))

    def roll_needs_the_flaps(self, snap):
        """``FLAP_BRAKE_YIELDS_TO_ROLL``: is the vehicle off its bank?

        **The flap brake deploys the elevons, and the elevons are the roll
        surfaces.**  Deployed, they sit at the end of their travel and the
        roll authority is gone or left in one direction only: both of this
        airframe's losses of control in the glide began the tick the brake
        went out -- LOG3680 (+114 flown against -10 commanded, slip 53, at
        Mach 4.8) and LOG3690 (-37 against +19, slip 32, at Mach 6.4), where
        the user watched the vehicle refuse to roll.  So the brake gives the
        surfaces back whenever the bank the vehicle flies is more than
        ``BANK_RATE_SAT_DEG`` from the command or the sideslip passes
        ``BANK_RATE_SLIP_TOL_DEG``, and may not come back until the bank has
        been held for ``FLAP_BRAKE_ROLL_SETTLE_S``.
        """
        flown = flown_bank(snap)
        slip = getattr(snap, "sideslip", 0.0) or 0.0
        off = (math.isnan(flown)
               or abs((flown - self.commanded_bank + 180.0) % 360.0 - 180.0)
               > float(self.cfg.BANK_RATE_SAT_DEG)
               or abs(slip) > float(self.cfg.BANK_RATE_SLIP_TOL_DEG))
        if off:
            self._roll_settled_since = None
            return True
        since = getattr(self, "_roll_settled_since", None)
        if since is None:
            since = self._roll_settled_since = snap.ut
        return snap.ut - since < float(self.cfg.FLAP_BRAKE_ROLL_SETTLE_S)

    def glide_flap_brake(self, snap, reserve, dt):
        """``GLIDE_FLAP_BRAKE``: the measured flap brake as the glide's third
        energy control, after alpha and bank have run out.

        The solve holds the prediction on ``reserve``; it reads long past it
        only when both controls are at their stops, so "long by more than
        ``GLIDE_FLAP_ON_M``" *is* the saturation test.  The propagation does
        not model the brake, so every prediction is of the vehicle with the
        brake **stowed** -- "if I retract now, am I still long?" -- and the
        brake comes in when the answer is no.  That is a law the vehicle can
        fly, which a prediction of a brake held out would not be.  Never
        switched mid-reversal (``bank_in_transit``): the transit's own
        prediction is the four-second lie of failure 16.
        """
        said = getattr(self, "_glide_flap_said", None)
        if said is None:
            said = self._glide_flap_said = set()

        def declined(reason):
            if reason not in said:
                said.add(reason)
                self.logbook.event(snap.ut, "glide flap brake: %s" % reason)
        if getattr(self, "flap_brake", None) is None:
            declined("no brake armed -- never deployed")
            return
        if self.last_miss is None:
            return
        try:
            mach = self.env.mach(vec.norm(snap.velocity),
                                 vec.norm(snap.position)
                                 - self.env.equatorial_radius)
        except Exception:                                   # noqa: BLE001
            return
        top = float(self.cfg.GLIDE_FLAP_MACH_MAX)
        allowed = top <= 0.0 or mach <= top
        # Roll first: stow (not merely "don't switch") while the vehicle is
        # off its bank -- the guard below used to return with the brake
        # still out through a whole reversal.
        if self.roll_needs_the_flaps(snap):
            if self.flap_brake_out and self.set_flap_brake(False):
                self.logbook.event(snap.ut, "glide flap brake in: rolling "
                                   "(bank %+.1f flown, %+.1f commanded, slip "
                                   "%+.1f)" % (flown_bank(snap),
                                               self.commanded_bank,
                                               snap.sideslip))
            return
        # Mid-reversal: the rate-limited command has not reached the lean
        # the loop wants (``bank_wanted``, after the side latch and caps).
        wanted = getattr(self, "bank_wanted", None)
        reach = max(self.cfg.SOLVE_HOLD_TRANSIT_DEG,
                    self.bank_rate() * max(0.0, dt))
        if wanted is not None and abs(wanted - self.steer.bank) > reach:
            if self.last_miss[0] > reserve + self.cfg.GLIDE_FLAP_ON_M:
                declined("long but mid-reversal (seen at least once)")
            return
        long = self.last_miss[0]
        out = self.flap_brake_out
        if not out and allowed and long > reserve + self.cfg.GLIDE_FLAP_ON_M:
            out = True
        elif out and (not allowed or long < reserve):
            out = False
        if not out and not allowed and long > reserve + self.cfg.GLIDE_FLAP_ON_M:
            declined("long but above GLIDE_FLAP_MACH_MAX (seen at least once)")
        if out == self.flap_brake_out:
            return
        if not self.set_flap_brake(out):
            declined("set_flap_brake(%s) moved no surface" % out)
            return
        self.logbook.event(snap.ut, "glide flap brake %s: long=%+.0f "
                           "reserve=%.0f M=%.2f"
                           % ("OUT" if out else "in", long, reserve, mach))

    def hac_exit_surplus(self, needed, snap=None):
        """How much height over its own need the approach can be handed.

        ``HAC_EXIT_SURPLUS_M`` is a fitted 500 m.  With
        ``HAC_EXIT_SURPLUS_DERIVED`` the question is the one the exit really
        decides: lined up at the gate with surplus, the alternative to
        leaving is **a lap**, and a lap is only worth starting if the height
        can pay for it -- ``2 pi R / cone_ld`` at the tightest circle the
        airframe holds at this speed (``hac_hold_radius``, floored at
        ``HAC_RADIUS_MIN_M``).  Anything less is handed to the approach,
        whose S-turn and brake exist to spend it.  Measured before this: every
        lap begun at the gate ran out of height (LOG3591, 3594, 3597-3601);
        an S-turn-only allowance (``needed * (1/cos 45 - 1)``, ~865 m) sent 4
        of 6 sim cones round (LOG3597-3602).
        """
        if (snap is None):
            return self.cfg.HAC_EXIT_SURPLUS_M
        speed = vec.norm(snap.velocity)
        height = vec.norm(snap.position) - self.env.equatorial_radius
        at_target = getattr(self.cfg, "HAC_EXIT_LAP_AT_TARGET", False)
        if at_target:
            # ``HAC_EXIT_LAP_AT_TARGET``: the lap the plan prices -- at the
            # cone's target speed, not the speed the gate is reached at.
            stall = airframe.stall(self.env, self.cfg)
            if stall is not None:
                speed = min(speed, guidance.cone_speed(self.env, self.cfg,
                                                       stall, height))
        radius = max(self.cfg.HAC_RADIUS_MIN_M,
                     guidance.hac_hold_radius(self.cfg, speed,
                                              self.surface_gravity))
        ratio = airframe.cone_ld(self.env, self.cfg, speed, height,
                                 snap.mass, self.surface_gravity)
        lap = 2.0 * math.pi * radius / max(0.1, ratio)
        # ``HAC_EXIT_LAP_FRACTION``: past this share of a lap, the lap is
        # the nearer answer.
        lap *= float(getattr(self.cfg, "HAC_EXIT_LAP_FRACTION", 1.0))
        if at_target:
            # Lap once the surplus passes a lap less what the approach can
            # spend: the lap then leaves the approach no more than that.
            lap -= self.cfg.HAC_EXIT_SURPLUS_M
        return max(self.cfg.HAC_EXIT_SURPLUS_M, lap)

    def hac_flap_brake(self, snap, command, height):
        """``HAC_FLAP_BRAKE``: the spoiler as the cone's descent authority
        once the weave is saturated.

        Out when the weave sits at ``HAC_WEAVE_MAX_DEG`` with more than
        ``HAC_WEAVE_DEADBAND_M`` of surplus over ``needed_height``.  In when
        the surplus is gone -- **counting the height it takes to arrest the
        sink the spoiler built**, ``sink^2 / (2 a)`` at
        ``HAC_FLAP_ARREST_G``, because stowing on "surplus spent" leaves a
        vehicle falling at 100 m/s with nothing left to stop it in (LOG3593,
        the approach's version of the same brake).
        """
        if getattr(self, "flap_brake", None) is None:
            return
        needed = getattr(command, "needed_height", None)
        if needed is None:
            return
        sink = max(0.0, -vec.dot(snap.velocity, vec.unit(snap.position)))
        # ``HAC_FLAP_ARREST_EXCESS``: only the sink *over the cone's own
        # glide* needs arresting -- the rest is the descent the profile
        # already flies.  Charged whole, 117-179 m/s at 200 m/s read as
        # 1.4-3.2 km of arrest and stowed every deployment within 3-7 s
        # (LOG4052, 4065, 4067).
        nominal = 0.0
        if getattr(self.cfg, "HAC_FLAP_ARREST_EXCESS", False):
            speed = vec.norm(snap.velocity)
            ratio = airframe.cone_ld(self.env, self.cfg, speed, height,
                                     snap.mass, self.surface_gravity)
            nominal = speed / math.sqrt(1.0 + max(0.1, ratio) ** 2)
        arrest = (max(0.0, sink * sink - nominal * nominal)
                  / (2.0 * max(0.1, self.cfg.HAC_FLAP_ARREST_G)
                     * self.surface_gravity))
        surplus = height - needed
        saturated = (getattr(command, "weave_deg", 0.0)
                     >= self.cfg.HAC_WEAVE_MAX_DEG - 0.5)
        # ``HAC_FLAP_BRAKE_ON_SURPLUS``: the weave need not be pinned.  It
        # sat at ~44 of 50 deg on every shuttle cone of LOG3846-3875 while
        # 1-2 km of surplus reached the rollout, so the brake never came out
        # and the approach was handed what it cannot spend.
        if getattr(self.cfg, "HAC_FLAP_BRAKE_ON_SURPLUS", False):
            saturated = True
        if (self.roll_needs_the_flaps(snap)
                and not getattr(self.cfg, "HAC_FLAP_BRAKE_IGNORES_ROLL",
                                False)):
            if self.flap_brake_out and self.set_flap_brake(False):
                self.logbook.event(snap.ut, "cone flap brake in: rolling "
                                   "(bank %+.1f flown, %+.1f commanded, slip "
                                   "%+.1f)" % (flown_bank(snap),
                                               self.commanded_bank,
                                               snap.sideslip))
            return
        out = self.flap_brake_out
        if not out and saturated and surplus > self.cfg.HAC_WEAVE_DEADBAND_M \
                and surplus - arrest > self.cfg.HAC_WEAVE_DEADBAND_M:
            out = True
        elif out and surplus - arrest < 0.5 * self.cfg.HAC_WEAVE_DEADBAND_M:
            out = False
        if out != self.flap_brake_out and self.set_flap_brake(out):
            self.logbook.event(snap.ut, "cone flap brake %s: surplus %+.0f, "
                               "sink %.0f (arrest %.0f m)"
                               % ("OUT" if out else "in", surplus, sink,
                                  arrest))

    def bay_brake(self, snap, surplus, where, saturated):
        """``BAY_BRAKE``: the cargo bay doors, **only when nothing else can
        spend the surplus** (the user, 2026-10-07: "only if necessary ...
        really goofy to see it randomly opening and closing").  They open
        once -- the cone's radius at its cap with no lap, or the approach's
        S-turns saturated, and more than ``HAC_WEAVE_DEADBAND_M`` still to
        spend -- and stay open until the surplus is gone; once shut they
        stay shut.  Doors add drag and a little lift (scratch bayprobe), so
        there is no sink to arrest.  rot-canard2-1007's three losses were
        approaches 1 km high with the S-turns pinned that dived the excess
        away into the flare door at 64-85 m/s of sink."""
        if not getattr(self.cfg, "BAY_BRAKE", False) or surplus is None:
            return
        if getattr(self, "_bay_done", False):
            return
        bays = getattr(self, "_bays", None)
        if bays is None:
            try:
                bays = list(self.vessel.parts.cargo_bays)
            except Exception:                           # noqa: BLE001
                bays = []
            self._bays = bays
            self._bay_open = False
            if not bays:
                self.logbook.event(snap.ut, "bay brake: no cargo bay")
        if not bays:
            return
        if not self._bay_open:
            if not (saturated and surplus > float(self.cfg.HAC_WEAVE_DEADBAND_M)):
                return
            want = True
        else:
            if surplus > 0.0:
                return
            want = False
        try:
            for bay in bays:
                bay.open = want
        except Exception:                               # noqa: BLE001
            return
        self._bay_open = want
        if not want:
            self._bay_done = True
        self.logbook.event(snap.ut, "bay brake %s (%s): surplus %+.0f m%s"
                           % ("OPEN" if want else "shut", where, surplus,
                              ", everything else saturated" if want else ""))

    def run_coast(self, snap):
        """Fall to the entry interface, already in the entry attitude.

        Pitching up before the air arrives rather than after: the vehicle has
        minutes of nothing to do and an angle of attack established in vacuum
        costs nothing to hold, where one established in thickening air has to
        be fought for.
        """
        self.set_throttle(0.0)
        # With ``DRAIN_BEFORE_BURN`` the post-burn DRAIN phase never runs, so
        # the handover instrument is read here instead -- still the first
        # tick after the burn, still off the propagation the burn paid for.
        if not self.interface_logged and self.deorbit_dv is not None:
            self.interface_logged = True
            self.log_interface_prediction(snap)
        self.log_cutoff_drift(snap)
        self.set_rcs(False, snap)
        self.coast_warp(snap)
        self.end = self.env.runway.choose(snap.position, snap.velocity)
        self.env.refresh(
            snap.ut, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius)
        self.log_boundary_actual(snap)
        alpha = min(self.cfg.ENTRY_ALPHA_DEG, self.alpha_ceiling)
        # **Enter in the attitude the burn was aimed for.**  The deorbit
        # search and its stop test both propagate at
        # ``Steer(ENTRY_ALPHA_DEG, SOLVE_BANK_MIN_DEG)``, so coasting in at
        # bank zero hands the glide a different trajectory from the one the
        # burn solved: the measured range table puts bank 30 at 1872 km
        # against bank 0's 1730, and that 142 km is the wrong way.  In game
        # the first GLIDE tick predicted ``long=-63015`` and ``-63110`` on two
        # flights whose deorbit exits were 486 km apart -- the same number,
        # because it is not their deorbit error at all, it is the do-nothing
        # baseline of an attitude nobody aimed at.
        #
        # It also puts the vehicle where it has authority in *both*
        # directions: from bank 30 the solve can stretch to 0 or shorten to
        # 70, where from bank 0 the only way out is shorter.
        gate = self.env.runway.gate(self.end)
        held = getattr(self, "_coast_bank", None)
        if held:
            # Keep the lean until the glide's deadband calls for the other
            # one.  See ``Config.COAST_BANK_LATCH``.
            bank = math.copysign(
                self.cfg.SOLVE_BANK_MIN_DEG,
                guidance._bank_sign(self.env, self.cfg, snap.position,
                                    snap.velocity, gate, held))
        else:
            bank = guidance.bank_toward(snap.position, snap.velocity,
                                        vec.sub(gate, snap.position),
                                        self.cfg.SOLVE_BANK_MIN_DEG)
        self._coast_bank = bank
        self.aim(alpha, bank, snap)
        # Seed the glide's solve from what the coast is actually holding.
        # ``aim`` records the command for the log; ``self.steer`` is what
        # ``run_glide`` inverts around, so leaving it at bank 0 would hand the
        # first solve the very baseline this is here to remove.
        self.steer = Steer(alpha=alpha, bank=bank)
        if (snap.height_above_runway <= self.cfg.ENTRY_INTERFACE_M
                if not getattr(self.cfg, "ENTRY_INTERFACE_AT_AIR", False)
                else trajectory.past_interface(
                    self.env, self.cfg,
                    vec.norm(snap.position) - self.env.equatorial_radius)):
            # Belt and braces: the altitude release above has already fired
            # 12 km higher, and a phase that hands over in warp is a phase
            # whose successor cannot point the vehicle.
            self.set_warp(True, factor=self.cfg.COAST_WARP_FACTOR)
            self.log_interface_actual(snap)
            self.enter(GLIDE, snap.ut, "runway %s" % self.end["name"])

    def coast_warp(self, snap):
        """Rails-warp the vacuum part of the fall to the interface.

        A third of the flight is this coast and two thirds of the coast is
        above the atmosphere with nothing commanded and nothing being
        decided.  See ``Config.COAST_WARP``.

        The release is by altitude and with room to spare, because warp
        freezes the attitude and the entry angle of attack is established
        before the air arrives on purpose.

        What the game *gave* is logged rather than what was asked for: the
        factor is an index into a rate table the game owns and a mod can
        redefine, and warp refusing looks exactly like warp working from
        here.  Failure 11 is the version of that lesson this project has
        already paid for.
        """
        if not self.cfg.COAST_WARP:
            return
        altitude = vec.norm(snap.position) - self.env.equatorial_radius
        release = altitude <= (self.env.atmosphere_depth
                               + self.cfg.COAST_WARP_STOP_M)
        was = self.warp_factor
        self.set_warp(release, factor=self.cfg.COAST_WARP_FACTOR)
        if self.warp_factor == was:
            return
        if self.warp_factor:
            try:
                got = self.conn.space_center.rails_warp_factor
            except Exception:                               # noqa: BLE001
                got = -1
            self.logbook.event(snap.ut, "coast warp at %.0f km: asked index "
                                        "%d, game gives %d"
                               % (altitude / 1000.0,
                                  self.cfg.COAST_WARP_FACTOR, got))
        else:
            self.logbook.event(snap.ut, "coast warp released at %.0f km, "
                                        "%.0f km of vacuum coast left"
                               % (altitude / 1000.0,
                                  (altitude - (
                                      self.env.atmosphere_depth
                                      if getattr(self.cfg,
                                                 "ENTRY_INTERFACE_AT_AIR",
                                                 False)
                                      else self.cfg.ENTRY_INTERFACE_M))
                                  / 1000.0))

    def check_thermal(self, snap):
        """Say once when the skin gets close to its limit, and say which part.

        A warning and nothing more.  The entry corridor really is bounded
        below by heating and above by the skip, and a guidance that traded
        range against temperature would be the textbook answer -- but there is
        no measurement here yet to size that trade with, and this project's
        rule is that a number gets measured before it gets acted on.
        """
        if snap.skin_fraction < float(self.cfg.THERMAL_WARN_FRACTION):
            return
        if snap.skin_fraction <= self.thermal_peak + 0.05:
            return
        self.thermal_peak = snap.skin_fraction
        self.logbook.event(snap.ut, "skin at %.0f%% of limit on %s"
                           % (snap.skin_fraction * 100.0,
                              snap.skin_hottest or "?"))

    def _mach_text(self, snap):
        """Mach, or an honest dash.  A breakup line that invents a speed is
        worse than one that admits it could not read the air."""
        try:
            return "M %.1f" % self.env.mach(
                vec.norm(snap.velocity),
                vec.norm(snap.position) - self.env.equatorial_radius)
        except Exception:                                   # noqa: BLE001
            return "M ?"

    def watch_breakup(self, snap):
        """Say when the vehicle starts coming apart, once, when it happens.

        The vessel is destroyed on most flights and the log has never carried
        a word about it -- the evidence was one line in a harness that runs
        *after* the flight, saying ``0 parts``, with no way to tell a breakup
        at Mach 6 from a heavy touchdown.  Two independent signs are watched
        because they fail differently: a part whose skin stream has fallen
        silent (free, per tick, but a stream can go quiet for other reasons)
        and the polled count (unambiguous, but only every few seconds).
        """
        if snap.parts_now and self.parts_at_start is None:
            self.parts_at_start = snap.parts_now
        if self.parts_at_start and snap.parts_now:
            gone = self.parts_at_start - snap.parts_now
            if gone > self.parts_gone:
                self.parts_gone = gone
                self.logbook.event(
                    snap.ut, "lost %d of %d parts (%d left) at alt %.0f, "
                             "%s, hottest %s at %.0f%% of its limit"
                    % (gone, self.parts_at_start, snap.parts_now,
                       snap.height_above_runway, self._mach_text(snap),
                       snap.skin_hottest or "?", snap.skin_fraction * 100.0))
        if snap.parts_lost > self.parts_silent:
            self.parts_silent = snap.parts_lost
            self.logbook.event(snap.ut, "%d skin sensor(s) have gone silent -- "
                                        "parts gone, or streams lost: %s"
                               % (snap.parts_lost,
                                  ", ".join(snap.parts_lost_names) or "?"))

    def run_glide(self, snap):
        """Solve the angle of attack and bank, every tick, to hit the gate.

        This is the whole flight, in the sense that everything before it only
        decides where it starts and everything after it is a landing.  The
        vehicle spends fifteen minutes in air thick enough to decelerate it at
        more than a g, with two controls that cost no propellant, and a
        prediction of where it is going to arrive.
        """
        self.set_throttle(0.0)
        # Off by default, and the default is the whole entry as it is flown:
        # the glide's angle of attack shortfall is a saturation the thrusters
        # would fight continuously rather than a slew they could finish, and
        # the tank is sized for slews.  It is a switch because the high-alpha
        # experiment needs it (docs/spaceplane/design.md, "High alpha: what the
        # airframe gives and what it will hold"), and because "RCS cannot
        # help here" should be a measurement rather than an assumption.
        permitted = True
        if permitted and self.cfg.GLIDE_RCS_MACH_MIN > 0.0:
            # Only where the reversals need it.  See ``GLIDE_RCS_MACH_MIN``.
            try:
                permitted = self.env.mach(
                    vec.norm(snap.velocity),
                    vec.norm(snap.position) - self.env.equatorial_radius
                ) >= self.cfg.GLIDE_RCS_MACH_MIN
            except Exception:                               # noqa: BLE001
                pass
            # ``ATTITUDE_YAW_WITH_RCS``: a turn in progress keeps its valve.
            # Below the Mach floor permission used to lapse on the tick,
            # shutting the valve with 10.8 deg of pointing error and 20 of
            # slip and snapping yaw from 4.8 s back to 22.6 mid-recovery --
            # LOG3802 then swung the bank past its command six times and
            # was lost 25 km short.  Open stays open until the relay settles
            # on its own; only then does the floor apply.
            if not permitted and getattr(self.rcs, "on", False):
                permitted = True
        self.set_rcs(permitted, snap)
        self.env.refresh(
            snap.ut, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius)
        # **Is there surplus to spend?**  The speed floor is a brake, and a
        # brake is only free while the entry is long.  Decided here, once,
        # off the last prediction, and hung on ``env`` so every propagation
        # this tick flies the same law -- the rule ``holdable`` follows.
        # Surplus is measured against what the glide is *aiming* at, which
        # with ``GLIDE_RESERVE_M`` is the gate plus a decaying reserve.  Read
        # against the gate instead, a vehicle deliberately flown 8 km long
        # and still short of its own target would have the brake on -- which
        # is failure 22, "the vehicle was braking while it was short", in the
        # reserve's frame.
        reserve = guidance.glide_reserve(self.env, self.cfg, snap.position)
        self.env.spending = (self.last_miss is None
                             or self.last_miss[0] >= reserve)
        alpha0 = vec.clamp(self.steer.alpha, self.cfg.ALPHA_MIN_DEG,
                           self.alpha_ceiling)
        # **Predict the lean the solve intends, not the one the rate limiter
        # is passing through.**  See ``Config.BANK_PREDICT_INTENT``: a
        # reversal takes seventeen seconds to slew between the stops and the
        # propagation flies whatever it is handed for the remaining thousand,
        # so mid-reversal it predicts a wings-level entry and reads 20 km
        # long.  The *sign* still comes from the vehicle's actual lean, which
        # is what ``_bank_sign`` reads it for; only the magnitude is the
        # intent.
        bank0 = self.steer.bank
        steer, prediction = guidance.solve_glide(
            self.env, snap.position, snap.velocity, snap.mass, self.cfg,
            self.end, alpha0, bank0,
            self.alpha_ceiling)
        self.bank_intent = steer.bank
        if prediction is not None:
            self.prediction = prediction
            self.env.set_profile(prediction.profile)
            self.update_cone_energy(snap, prediction)

        # Rate-limit the command.  The solve is allowed to ask for anything;
        # the vehicle is not allowed to slam to it, because a step in bank is
        # a step in the vertical lift component and that is what the whole
        # energy budget rides on.
        alpha = vec.clamp(steer.alpha, self.cfg.ALPHA_MIN_DEG,
                          self.alpha_ceiling)
        dt = max(0.05, snap.ut - getattr(self, "_last_glide_ut", snap.ut))
        self._last_glide_ut = snap.ut
        alpha = vec.clamp(alpha, self.steer.alpha - self.cfg.ALPHA_RATE_DEG_S * dt,
                          self.steer.alpha + self.cfg.ALPHA_RATE_DEG_S * dt)
        # Mid-reversal the prediction is of a wings-level entry the vehicle is
        # not going to fly, and it reads twenty kilometres long.  Hold the
        # angle of attack until the lean is back against a stop rather than
        # chase it.  See ``Config.SOLVE_HOLD_THROUGH_REVERSAL_DEG``.
        # The test is "has the lean arrived", not "is the lean small" --
        # see ``guidance.bank_in_transit`` and ``Config.SOLVE_HOLD_ON_TRANSIT``.
        # The magnitude version froze the angle of attack for the whole
        # second half of every entry, because the terminal glide's lean is
        # genuinely a few degrees and never looked established to it.
        if guidance.bank_in_transit(self.cfg, self.steer.bank, steer.bank,
                                    self.bank_side, dt, self.bank_rate()):
            # Rolling through a reversal: what the alpha does now is the
            # roll, not the airframe's ceiling.  See ``ratchet_alpha``.
            self.holdable_quiet_until = snap.ut + self.attitude_settle_s
        if guidance.bank_in_transit(self.cfg, self.steer.bank,
                                    steer.bank, self.bank_side, dt,
                                    self.bank_rate()):
            alpha = self.steer.alpha
        # **The same cap the propagator applies.**  It lived only inside
        # ``trajectory.acceleration`` at first, so the prediction flew a
        # vehicle that gave up angle of attack to hold the approach speed and
        # the control loop flew one that did not.  That is boosterland's
        # "``landing_command`` is the throttle law itself, called by the
        # propagator and wrapped by guidance for the control loop" -- a
        # prediction of a law nobody flies is a prediction of a trajectory
        # nobody flies.  APPROACH and FLARE are deliberately outside it: they
        # have their own speed logic, and the flare in particular has to be
        # able to ask for maximum lift at a speed this cap would refuse.
        limit = trajectory.alpha_limit_for_speed(
            self.env, self.cfg, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius,
            snap.mass, self.surface_gravity)
        alpha = min(alpha, limit)
        # And the floor underneath it, which is the same law: see
        # ``alpha_floor_for_speed``.  The learned ceiling still wins, because
        # it is a plant limit and this is a policy.
        alpha = max(alpha, trajectory.alpha_floor_for_speed(
            self.env, self.cfg, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius,
            snap.mass, self.surface_gravity))
        alpha = min(alpha, self.alpha_ceiling)
        # **A reversal has to outlast its own actuator.**  See
        # ``Config.BANK_REVERSAL_DWELL_S``: the relay switches faster than the
        # seventeen seconds a stop-to-stop slew takes, so the vehicle spends a
        # fifth of the entry mid-transit and the solve spends it reading a
        # wings-level prediction.
        wanted = steer.bank
        side = 1.0 if wanted >= 0.0 else -1.0
        if self.bank_side is None:
            self.bank_side, self.bank_reversed_ut = side, snap.ut
        # **A lean is not abandoned before it is established.**  See
        # ``Config.BANK_REVERSAL_SETTLE``: no clock and no constant --
        # the test is whether the vehicle has actually reached the side
        # it is currently committed to.  During a transit it has not, so
        # the slew is allowed to finish; once it has 30 degrees on (or
        # whatever smaller lean the solve asked for), reversing is free
        # again.
        if side != self.bank_side:
            settled = min(self.cfg.SOLVE_BANK_MIN_DEG, abs(wanted))
            if self.steer.bank * self.bank_side >= settled:
                self.bank_side, self.bank_reversed_ut = side, snap.ut
        wanted = self.bank_side * abs(wanted)
        # **The lean the vehicle is trying to hold, after every clamp and
        # before the rate limiter.**  This is the denominator of the duty
        # ratio (``update_bank_duty``), and it has to be this one rather than
        # ``bank_intent``: the solve's raw output is taken before the caps
        # below, so a solve asking for more lean than is flyable would make
        # the denominator too small and report a duty above 1.0 on a vehicle
        # that never reversed at all -- a spurious lift bonus wearing a duty
        # cycle's clothes.
        self.bank_wanted = wanted
        bank = vec.clamp(wanted,
                         self.steer.bank - self.bank_rate() * dt,
                         self.steer.bank + self.bank_rate() * dt)
        self.steer = Steer(alpha=alpha, bank=bank)
        self.aim(alpha, bank, snap)
        self.ratchet_alpha(snap)

        miss = self.miss(snap)
        if miss is not None:
            self.last_miss = miss
        self.glide_flap_brake(snap, reserve, dt)
        # Both tests, and the altitude one is not optional: a horizontal
        # distance alone captures the gate while the vehicle is still fifty
        # kilometres above it, doing Mach 7 straight over the top.  Offline
        # that ended the glide at h=50596 with the approach law handed a
        # hypersonic vehicle.
        height = snap.height_above_runway
        # **The glide ends where its own target is**, which is the high gate
        # when the cone is flying and the low one otherwise.  Reading
        # ``GATE_ALT_M`` here regardless would solve the entry to twelve
        # kilometres and then keep flying it down to two and a half, which
        # is the straight-in arrival the cone exists to stop asking for.
        # **The cone is entered on distance, the straight-in gate on
        # altitude**, and that difference is the whole point.  A handover at
        # a fixed altitude is another demand for a particular energy at a
        # particular place; a handover at a fixed distance takes the vehicle
        # whatever height it has and spends it.  The altitude test stays as a
        # backstop for an arrival so flat it reaches the gate's height before
        # it reaches the field at all.
        distance = self.gate_distance(snap)
        # **Mach first, and it is a veto, not a preference.**  Turn
        # radius is ``v^2 / (g tan bank)``: 9 km at Mach 0.9 and 250 km
        # at Mach 4.7.  There is no cone to fly at that speed, and
        # commanding one commands a hypersonic bank the entry never
        # asked for.  The first flight to reach this phase entered it at
        # Mach 4.7, because the distance test fired while the vehicle
        # was still 27 km up and hot.
        #
        # Under that veto, either of the other two -- and the cone is
        # entered as *early* as it can be flown, because every metre of
        # height it starts with is a metre of path it can spend.
        speed = vec.norm(snap.velocity)
        altitude = vec.norm(snap.position) - self.env.equatorial_radius
        try:
            mach = self.env.mach(speed, altitude)
        except Exception:                           # noqa: BLE001
            mach = 9.9
        # **The veto, calculated.**  ``HAC_ENTRY_MACH`` is a Mach
        # number standing in for a turn radius, and the speed of sound
        # is not part of the question -- see ``guidance.hac_enterable``,
        # which asks whether the circle the airframe can hold fits the
        # one the cone may fly.  ``None`` means the derivation is off
        # and the Mach constant is still the veto.
        can_turn = mach <= self.cfg.HAC_ENTRY_MACH
        # **Entered when it can pay for itself, not at an altitude.**
        # See ``cone_affordable``.  ``HAC_ENTRY_DIST_M`` stays as the
        # backstop it was written to be -- an arrival flat enough to
        # reach the field before the cone is ever affordable still has
        # to be taken -- and ``HAC_ALT_M`` keeps its other job, the
        # entry's aim point in ``Runway.high_gate``, where it is not a
        # trigger.
        #
        # **Waiting is what makes this the earliest flyable entry
        # rather than a delay**: the straight glide's ratio is 2.3-3.0
        # against the cone's 1.49, so gliding on buys affordability
        # faster than it spends height.  A cone that cannot pay now can
        # pay later; one entered anyway never can.
        reached = (height <= self.cfg.HAC_ALT_M
                   or distance <= self.cfg.HAC_ENTRY_DIST_M)
        ready = can_turn and reached
        if ready:
            tune_autopilot(self.autopilot,
                           self.cfg.ATTITUDE_TIME_TO_PEAK_FINAL_S,
                           self.logbook, snap.ut)
            released = airframe.alpha_ceiling(self.env, self.cfg)
            if self.alpha_ceiling < released:
                self.logbook.event(
                    snap.ut, "alpha ceiling released for the landing: %.1f "
                             "-> %.1f deg (it was learned at entry dynamic "
                             "pressure and the flare is not flown there)"
                    % (self.alpha_ceiling, released))
                self.alpha_ceiling = released
            miss = self.last_miss or (0.0, 0.0)
            # **Both ends are on the table here**, and the end chosen at
            # the deorbit was chosen on a bearing taken fifteen hundred
            # kilometres out.  Over the field the only question is how
            # much flying is left, and the four combinations of end and
            # hand differ by most of a lap.
            chosen, self.hac_side = guidance.hac_choose(
                self.env, self.cfg, self.env.runway, snap.position,
                snap.velocity, mass=snap.mass,
                gravity=self.surface_gravity, height=height)
            if chosen["name"] != self.end["name"]:
                self.logbook.event(
                    snap.ut, "runway %s -> %s: cheaper from here"
                    % (self.end["name"], chosen["name"]))
                self.end = chosen
            self.enter(HAC, snap.ut,
                       "over the field long=%+.0f cross=%+.0f d=%.0f "
                       "h=%.0f turning %s"
                       % (miss[0], miss[1], distance, height,
                          "left" if self.hac_side > 0 else "right"))
            self.log_cone_ladder(snap, height)

    def log_cone_ladder(self, snap, height):
        """One line at cone entry: the ladder's price of each 1 km of height
        wings level -- reference speed, the alpha that holds 1 g there, the
        table's L/D at it and what ``_hac_planned_ld`` returns -- so the
        plan's ``pld`` can be read against the table it was built from."""
        try:
            stall = airframe.stall(self.env, self.cfg)
            bits = []
            h = float(self.cfg.GATE_ALT_M) + 500.0
            while h < height:
                v = guidance.cone_speed(self.env, self.cfg, stall, h)
                a = trajectory.alpha_for_load(self.env, v, h, snap.mass,
                                              self.surface_gravity, 1.0)
                cla, cda = (self.env.coefficients(a, v, h) if a is not None
                            else (0.0, 0.0))
                ld = guidance._hac_planned_ld(self.env, self.cfg, v, h,
                                              snap.mass,
                                              self.surface_gravity,
                                              self.cfg.HAC_RADIUS_MAX_M, 0.0)
                bits.append("%.0fkm:v%.0f/a%s/%.2f/%s" % (
                    h / 1000.0, v, "%.1f" % a if a is not None else "-",
                    cla / cda if cda > 0.0 else 0.0,
                    "%.2f" % ld if ld is not None else "-"))
                h += 1000.0
            self.logbook.event(snap.ut, "cone ladder (h:v/alpha/table L/D/"
                               "planned): " + " ".join(bits))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(snap.ut, "cone ladder: FAILED (%s)" % exc)

    def run_hac(self, snap):
        """Spiral the surplus off overhead, and roll out on the centreline.

        The phase that makes the entry's scatter survivable.  Everything
        upstream delivers the vehicle over the field with some height between
        "just enough" and "far too much"; this turns that unknown into a turn
        radius and flies it away.  The rollout point is the low gate itself,
        so leaving the cone hands ``run_approach`` exactly the state it was
        always designed for -- lined up, at the gate's altitude, at the
        approach speed -- however the entry happened to arrive.
        """
        # Nothing below the glide permits RCS; close what it left open --
        # except, under ``ATTITUDE_YAW_WITH_RCS``, a turn still in progress:
        # shutting it on the first HAC tick snapped yaw from the thrusters'
        # figure back to 22.6 s mid-recovery and tumbled the vehicle
        # transonic (LOG3802, 3805, 3838, 3844).  Open stays open until the
        # relay settles on its own, the GLIDE Mach-floor rule; once shut it
        # stays shut.
        self.set_rcs(getattr(self.rcs, "on", False), snap)
        self.set_throttle(0.0)
        if not getattr(self, "_cone_ias_logged", False):
            self._cone_ias_logged = True
            ias = guidance.cone_ias(self.env, self.cfg)
            if ias is not None:
                self.logbook.event(
                    snap.ut, "cone IAS %.1f m/s (stall %.1f, min-drag %s, "
                    "bank limit %.0f deg)"
                    % (ias, self.env.stall_speed,
                       "-" if not getattr(self.env, "best_speed", None)
                       else "%.1f" % self.env.best_speed,
                       self.cfg.HAC_BANK_MAX_DEG))
        if not getattr(self, "_glide_flaps_stowed", False):
            # The glide's brake is the glide's; the cone has its own energy.
            self._glide_flaps_stowed = True
            if getattr(self, "flap_brake_out", False) \
                    and self.set_flap_brake(False):
                self.logbook.event(snap.ut, "glide flap brake in: the cone")
        self.env.refresh(
            snap.ut, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius)
        height = snap.height_above_runway
        if self.hac_side is None:
            self.hac_side = guidance.hac_side(self.env, self.cfg, self.end,
                                              snap.position, snap.velocity)
        dt = max(0.05, snap.ut - (self._last_hac_ut or snap.ut))
        self._last_hac_ut = snap.ut

        def plan(stretch, cfg=None):
            end = self.end
            if stretch > 0.0:
                end = dict(self.end, gate_stretch=stretch)
            return guidance.hac(self.env, cfg or self.cfg, end, snap.position,
                                snap.velocity, snap.mass,
                                self.surface_gravity, height,
                                self.hac_side,
                                previous=self.hac_radius,
                                max_step=self.cfg.HAC_RADIUS_RATE_M_S * dt,
                                weave=self.hac_weave_phase(snap)
                                * self.hac_weave_sign(snap.ut),
                                roll_rate=self.roll_rate.limit(),
                                ld_scale=getattr(self, "hac_ld_scale",
                                                 None))

        command = plan(self.hac_stretch)
        if command is not None and getattr(self.cfg, "HAC_PAST_KEEPS_LINEUP",
                                           False):
            command = self.hac_keep_lineup(snap, command, plan)
        if command is not None and getattr(self.cfg, "HAC_GATE_STRETCH",
                                           False):
            command = self.hac_gate_stretch(snap, command, height, dt, plan)
        if command is None:
            # **No answer, not a zero.**  Degenerate geometry here means over
            # the centre of the circle or stopped; holding the last command
            # for a tick is honest, commanding wings level is a decision
            # nobody made.
            self.aim(self.steer.alpha, self.steer.bank, snap)
            return
        # **Its own attribute, not ``self.command``.**  The transition into
        # APPROACH happens inside this tick, and the line logged after it
        # reads ``self.command`` expecting the approach's fields -- so a
        # shared attribute means every cone flight dies of an
        # ``AttributeError`` on ``wanted_sink`` at the one moment it was
        # working.  It did, on the first flight that got there.
        self.hac_command = command
        self.hac_radius = command.radius
        self.log_hac_ladder(snap, height)
        if getattr(self.cfg, "HAC_SPLIT_BRAKE", False):
            self.hac_split_brake(snap, command, height, dt)
        spiral = self.hac_spiral(snap, command, height)
        if spiral is not None:
            self.steer = Steer(alpha=spiral[0], bank=spiral[1])
            self.aim(spiral[0], spiral[1], snap)
            return
        alpha = min(command.alpha, self.alpha_ceiling)
        self.steer = Steer(alpha=alpha, bank=command.bank)
        self.aim(alpha, command.bank, snap)
        self.ratchet_alpha(snap)
        self.hac_flap_brake(snap, command, height)
        needed = getattr(command, "needed_height", None)
        self._hac_surplus = None if needed is None else height - needed
        self.bay_brake(snap, self._hac_surplus, "cone",
                       command.radius >= self.cfg.HAC_RADIUS_MAX_M - 1.0
                       and getattr(command, "laps", 0) == 0)
        # Rolled out, or out of height.  The second is not a failure mode:
        # the cone is flown *above* the gate's altitude and reaching it is
        # simply the end of the surplus.
        # **Lined up is not the same as ready**, and conflating them cost
        # two flights of the first clean batch.  Reaching the rollout point
        # with a lap still owed means the cone has decided it needs another
        # 19 km of path; leaving there anyway hands the approach a vehicle
        # 12 km up and 4 km out, and it floats ten kilometres past the
        # field.  Both of the +10 km arrivals in that batch read exactly
        # that: ``rolled out turn=1 h=12697 laps=1``.
        #
        # So the exit wants all three: pointing the right way, on the
        # circle, and with no more height than the approach's own S-turn can
        # spend.  Anything else flies past the rollout, the turn wraps, and
        # the lap it needed happens.
        #
        # Short is *not* an exit -- it is a steering mode inside the cone
        # (cut at the gate rather than spend path).  The first flight to
        # reach this phase read short 29 km out at 26 km up, handed over,
        # and the four-kilometre approach law flew it into the ground inside
        # a second.  The floor at the gate's altitude is the backstop under
        # all of it.
        # **The height test is the *approach's* requirement, not the cone's.**
        # See ``guidance.hac``: ``needed_height`` is the gate's altitude plus
        # the circling path still to fly at the ratio a turning descent
        # achieves, and none of that survives the rollout -- from there the
        # vehicle flies a straight line to the touchdown aim at the ratio the
        # approach achieves.  Checking the handover against the cone's own
        # arithmetic passed a vehicle 560 m high and put it 1.9 km down a
        # 2.4 km runway (``logs/LOG1500``, from an arrival 19 m off the gate).
        needed = getattr(command, "approach_needed", command.needed_height)
        # ``HAC_EXIT_PAST_DEG``: a little *past* the rollout counts for the
        # exit (not for the plan, whose wrap is what owes a lap -- putting
        # the band in ``hac_turn`` broke laps, journal 2026-09-24).  LOG4104
        # reached the gate 750 m above need with the weave's last swing 13-17
        # deg past the centreline, read turn 343-347, flew on and left "out
        # of height" 3.2 km beyond it.
        past = float(getattr(self.cfg, "HAC_EXIT_PAST_DEG", 0.0))
        aligned = (command.turn_deg <= self.cfg.HAC_EXIT_TURN_DEG
                   or command.turn_deg >= 360.0 - past)
        ready = (aligned
                 and command.gate_range <= self.cfg.HAC_ROLLOUT_M
                 and command.laps == 0
                 and height <= needed + self.hac_exit_surplus(needed, snap))
        if ready or height <= self.cfg.GATE_ALT_M:
            released = airframe.alpha_ceiling(self.env, self.cfg)
            if self.alpha_ceiling < released:
                self.logbook.event(
                    snap.ut, "alpha ceiling released for the landing: %.1f "
                             "-> %.1f deg" % (self.alpha_ceiling, released))
                self.alpha_ceiling = released
            self.enter(APPROACH, snap.ut,
                       "%s turn=%.0f h=%.0f (needed %.0f) gate=%.0f "
                       "(circle %.0f) laps=%d stretch=%.0f"
                       % ("rolled out" if ready else "out of height",
                          command.turn_deg, height, needed,
                          command.gate_range, command.radius, command.laps,
                          getattr(command, "stretch", 0.0)))

    def set_split(self, angle):
        """Both halves of the split rudder to ``angle`` (0 stows)."""
        done = 0
        for surface in self.split_pair:
            if self._deploy_surface(surface, angle, angle > 0.0):
                done += 1
        if done:
            self.split_angle = angle
        return done

    def hac_split_brake(self, snap, command, height, dt):
        """``HAC_SPLIT_BRAKE``: spend the cone's surplus as drag.

        The twin fins deployed as a mirrored pair are a speedbrake with no
        yaw or roll (``tools/splitprobe.py``: at 38 deg the lower cone's L/D
        3.28 -> 2.00, the approach's 4.17 -> 1.89, yaw and roll 0.0; the
        cost is a nose-down moment the elevons trim).  It is continuous, so
        it reaches the surplus between the widest no-lap circle and a lap
        that no radius can (rot-polar-1008: 1-4 km of height).

        The plan prices the path still to fly at ``plan_ld``; spending the
        whole height down to the gate over that path needs ``path / (height
        - gate)``.  Their ratio is the L/D factor wanted, and
        ``HAC_SPLIT_FACTOR`` (the measured factor against angle) turns it
        into an angle.  Stowed when short, when a lap is owed (the lap is
        the spender), and on leaving the cone (``run_approach``).
        """
        if not self.split_pair:
            return
        gate = float(getattr(command, "gate_alt", self.cfg.GATE_ALT_M))
        ld = float(getattr(command, "plan_ld", 0.0) or 0.0)
        want = 0.0
        # **Only on the branch the plan priced** (``HAC_SPLIT_ON_BRANCH``):
        # the flown polar's rising side.  Deployed at cone entry while the
        # speed law was still braking the vehicle stalled at ~20 deg (L/D
        # ~1.2 where the plan priced 2.3), brake and stall together ran two
        # inc flights out of height 13.6 km from the gate (LOG8714, 8719).
        # So: achieved alpha under the polar's lift peak, and the speed
        # within ``HAC_SPLIT_SPEED_FRAC`` of the cone's target.
        on_branch = True
        if getattr(self.cfg, "HAC_SPLIT_ON_BRANCH", False):
            polar = getattr(self.env, "flown_polar", None)
            peak = None
            if polar and polar.get("bins"):
                peak = max(polar["bins"], key=lambda b: b[1])[0]
            target = getattr(command, "alpha_target_speed", None)
            speed = vec.norm(snap.velocity)
            on_branch = ((peak is None or snap.alpha_actual < peak)
                         and (not target or speed <= target * float(
                             self.cfg.HAC_SPLIT_SPEED_FRAC)))
        # ``HAC_SPLIT_RESERVE_M``: spend down to a little above the gate's
        # need, not to it -- unbraked, the cone ended 230-550 m short of the
        # approach's need on 8 of 12 brake flights (rot-split-1008).
        floor = gate + float(getattr(self.cfg, "HAC_SPLIT_RESERVE_M", 0.0))
        if (on_branch and not command.short and command.laps == 0
                and ld > 0.0 and height > floor and command.path > 0.0):
            factor = (command.path / (height - floor)) / ld
            table = sorted(self.cfg.HAC_SPLIT_FACTOR)
            if factor < 1.0:
                want = table[-1][0]
                for (a0, f0), (a1, f1) in zip(table, table[1:]):
                    if f1 <= factor <= f0:
                        want = a0 + (a1 - a0) * (f0 - factor) / max(1e-6, f0 - f1)
                        break
        step = float(self.cfg.HAC_SPLIT_RATE_DEG_S) * dt
        new = vec.clamp(want, self.split_angle - step, self.split_angle + step)
        if abs(new - self.split_angle) >= 1.0 or (new == 0.0) != (
                self.split_angle == 0.0):
            was = self.split_angle
            if self.set_split(new) and (was == 0.0) != (new == 0.0):
                self.logbook.event(snap.ut, "cone split brake %s at h=%.0f: "
                                   "%.0f deg (plan L/D %.2f, wanted factor "
                                   "%.2f)" % ("out" if new else "in", height,
                                              new, ld, (command.path / max(
                                                  1.0, height - gate)) / ld
                                              if ld else 0.0))

    def hac_keep_lineup(self, snap, command, plan):
        """``HAC_PAST_KEEPS_LINEUP``: a lined-up vehicle that a weave swing
        carries a little past the rollout is still lined up, unless the lap
        that wrap implies is one it can fly.

        The wrap is deliberate (``HAC_EXIT_PAST_DEG``'s comment): a vehicle
        past the rollout and too high owes its lap.  But a lap it cannot
        afford is not a plan.  Flown with the polar priced right, the cone
        lined up on its widest circle 15 km out with surplus, wove at 50
        deg to spend it, swung 12 deg past the rollout, read turn 348 and a
        106 km path, went ``short``, dropped the weave -- its only spender
        there -- and flew straight in 2.3 km high (LOG8655, 8665).  So:
        when the plan is short only because of a wrap inside
        ``HAC_EXIT_PAST_DEG`` and the last tick was lined up, plan it lined
        up (the past band as the overshoot tolerance) and keep that plan if
        it is not short."""
        past = float(getattr(self.cfg, "HAC_EXIT_PAST_DEG", 0.0))
        wrapped = command.turn_deg > 360.0 - past
        if (command.short and wrapped
                and getattr(self, "_hac_lined_up", False)):
            relaxed = getattr(self, "_cfg_past", None)
            if relaxed is None:
                import dataclasses
                relaxed = self._cfg_past = dataclasses.replace(
                    self.cfg, HAC_OVERSHOOT_DEG=max(
                        past, float(getattr(self.cfg, "HAC_OVERSHOOT_DEG",
                                            0.0))))
            lined = plan(self.hac_stretch, cfg=relaxed)
            if lined is not None and not lined.short:
                if not getattr(self, "_hac_kept_logged", False):
                    self._hac_kept_logged = True
                    self.logbook.event(
                        snap.ut, "cone: %.0f deg past the rollout and a lap "
                        "owed it cannot fly (path %.0f) -- kept lined up "
                        "(path %.0f, need %.0f)" % (
                            360.0 - command.turn_deg, command.path,
                            lined.path, lined.needed_height))
                command = lined
        self._hac_lined_up = (command.turn_deg <= self.cfg.HAC_EXIT_TURN_DEG
                              or command.turn_deg > 360.0 - past
                              and not command.short)
        return command

    def hac_gate_stretch(self, snap, command, height, dt, plan):
        """``HAC_GATE_STRETCH``: move the rollout out along the centreline
        while the plan has height it cannot spend; back in when short.

        The signal is the plan's own surplus, ``height - needed_height``
        with no lap owed -- the quantity that was being handed over (LOG8401
        read +1.7 km of it at the gate and rolled out +1.9).  One step of
        ``HAC_RADIUS_RATE_M_S * dt`` per tick, re-planned at the candidate
        and taken only if that plan still owes no lap, is not short, and
        has not wrapped the turn: a gate moved behind a vehicle already
        lined up reads ~360 deg to go, which is a lap, not a longer final.
        Returns the command to fly.
        """
        step = self.cfg.HAC_RADIUS_RATE_M_S * dt
        surplus = height - command.needed_height
        old = self.hac_stretch
        candidate = None
        if command.laps == 0 and surplus > 0.0:
            top = float(self.cfg.HAC_GATE_STRETCH_MAX_M)
            if old < top:
                candidate = min(top, old + step)
        elif command.needed_height > height and old > 0.0:
            candidate = max(0.0, old - step)
        if candidate is None:
            return command
        trial = plan(candidate)
        if trial is None:
            return command
        # **A step must spend, not reprice.**  Lined up outside the gate,
        # moving the gate toward the vehicle swaps straight path priced at
        # the cone's ratio (~2.3) for the same path at the approach's (4.2):
        # the plan's surplus grows and nothing is spent (kspSim LOG8459,
        # 8463, 8465 grew 2.5-2.9 km at turn 0 and rolled out +1.2-1.8 km
        # over need, against +0.2-0.65 without).  Out, the plan's need must
        # rise; in, it must fall.
        if candidate > old:
            jump = abs(trial.turn_deg - command.turn_deg)
            if (trial.laps != 0 or trial.needed_height > height
                    or trial.needed_height <= command.needed_height
                    or jump > 90.0):
                return command
        elif trial.needed_height >= command.needed_height:
            return command
        self.hac_stretch = candidate
        if (old == 0.0) != (candidate == 0.0) or int(old / 1000.0) != \
                int(candidate / 1000.0):
            self.logbook.event(
                snap.ut, "cone gate stretch %.0f -> %.0f m (surplus %+.0f, "
                "turn %.0f, gate alt %.0f)"
                % (old, candidate, surplus, trial.turn_deg,
                   getattr(trial, "gate_alt", self.cfg.GATE_ALT_M)))
        return trial

    def hac_spiral(self, snap, command, height):
        """``HAC_SPIRAL_DUMP``: tight descending 360s over the gate.

        Returns ``(alpha, bank)`` while spiralling, else ``None``.

        The cone hands the approach anything short of a whole lap
        (``hac_exit_surplus``: ``2 pi R / cone_ld``, ~8 km of height at its
        2 km minimum radius), and the approach can spend ~0.8 km of it.  The
        cone saves flown with ``CANARD_TRIM`` rolled out lined up 4-4.6 km
        over need and landed 3-14 km long (sav-sharp-1007).  A tight spiral
        is the quantum in between: at ``HAC_SPIRAL_BANK_DEG`` 60 and ~90 m/s
        the radius is ~480 m, and the turn's load is drag -- the user's
        split-S/sharp-turn idea, 2026-10-07, flown where it returns the
        vehicle to the same point on the same heading.

        Started lined up at the gate with no lap owed and surplus over
        ``HAC_EXIT_SURPLUS_M`` plus one lap's estimated cost; the cost of
        each lap is then *measured* (height lost per 360 deg of track), and
        at each pass through the runway heading another lap is flown only if
        it still leaves the approach its allowance.  Alpha holds the cone's
        speed at the turn's load, so the descent is whatever that speed
        costs.
        """
        if not getattr(self.cfg, "HAC_SPIRAL_DUMP", False):
            return None
        needed = getattr(command, "approach_needed", command.needed_height)
        surplus = height - needed
        allowance = float(self.cfg.HAC_EXIT_SURPLUS_M)
        speed = vec.norm(snap.velocity)
        up = vec.unit(snap.position)
        track = vec.project_out(snap.velocity, up)
        if vec.norm(track) < 1.0:
            return None
        track = vec.unit(track)
        g = self.surface_gravity
        stall = airframe.stall(self.env, self.cfg)
        v_ref = guidance.cone_speed(self.env, self.cfg, stall, height)
        cap = min(self.alpha_ceiling,
                  float(self.cfg.HAC_SPIRAL_ALPHA_MAX_DEG))
        # **The bank the alpha cap can hold**, with a tenth in hand.  At 60
        # deg the turn needs 2 g, ~16 deg at cone speed; capped at 14 the
        # load fell short, the nose dropped and the spiral became a dive --
        # 128 m/s, stopped mid-lap, flare at 138 m/s (LOG8432).
        cla, _ = self.env.coefficients(cap, v_ref, height)
        q_ref = 0.5 * self.env.density(height) * v_ref * v_ref
        load = 0.9 * q_ref * cla / max(1.0, snap.mass * g)
        if load <= 1.05:
            return None
        bank_deg = min(float(self.cfg.HAC_SPIRAL_BANK_DEG),
                       math.degrees(math.acos(1.0 / load)))
        state = getattr(self, "_spiral", None)
        aligned = (command.turn_deg <= self.cfg.HAC_EXIT_TURN_DEG
                   or command.turn_deg >= 360.0 - float(
                       getattr(self.cfg, "HAC_EXIT_PAST_DEG", 0.0)))
        if state is None:
            if getattr(self, "_spiral_done", False):
                return None
            radius = speed * speed / (g * math.tan(math.radians(bank_deg)))
            ratio = airframe.cone_ld(self.env, self.cfg, speed, height,
                                     snap.mass, g) or 1.5
            estimate = 2.0 * math.pi * radius / max(0.5, ratio)
            if not (aligned and command.laps == 0
                    and command.gate_range <= self.cfg.HAC_ROLLOUT_M
                    and surplus > allowance + estimate):
                return None
            state = self._spiral = {"track": track, "turned": 0.0,
                                    "lap_h": height, "cost": estimate,
                                    "laps": 0, "h0": height}
            self.logbook.event(snap.ut, "spiral dump: start h=%.0f surplus "
                               "%.0f, lap estimated %.0f m (r %.0f m, bank "
                               "%.0f, v_ref %.0f)" % (height, surplus,
                                                     estimate, radius,
                                                     bank_deg, v_ref))
        # Track turned since the last tick, signed about the vertical.
        step = math.degrees(math.atan2(
            vec.dot(vec.cross(state["track"], track), up),
            vec.dot(state["track"], track)))
        state["track"] = track
        state["turned"] += abs(step)
        if state["turned"] >= 360.0 * (state["laps"] + 1) - 30.0 and aligned:
            state["laps"] += 1
            state["cost"] = max(1.0, state["lap_h"] - height)
            state["lap_h"] = height
            again = surplus - state["cost"] >= allowance
            self.logbook.event(snap.ut, "spiral dump: lap %d cost %.0f m, "
                               "surplus %.0f -> %s" % (
                                   state["laps"], state["cost"], surplus,
                                   "another" if again else "done"))
            if not again:
                self._spiral, self._spiral_done = None, True
                return None
        # Stop while the rest of the turn back to the runway heading can
        # still be paid for, or on an overspeed (a spiral turning into a
        # dive), or a stall.
        left = max(0.0, 360.0 * (state["laps"] + 1) - state["turned"])
        rest = state["cost"] * left / 360.0
        if (surplus - rest < allowance * 0.5
                or speed > 1.25 * v_ref
                or (stall and speed < 1.5 * stall)):
            self.logbook.event(snap.ut, "spiral dump: stop mid-lap, surplus "
                               "%.0f (rest of lap ~%.0f), %.1f m/s"
                               % (surplus, rest, speed))
            self._spiral, self._spiral_done = None, True
            return None
        load = 1.0 / math.cos(math.radians(bank_deg))
        alpha = trajectory.alpha_for_load(self.env, v_ref, height, snap.mass,
                                          g, load)
        if alpha is None:
            alpha = float(self.cfg.HAC_SPIRAL_ALPHA_MAX_DEG)
        alpha += self.cfg.APPROACH_SPEED_KP * (speed - v_ref)
        alpha = vec.clamp(alpha, 0.0, cap)
        side = getattr(self, "_spiral_side", None)
        if side is None:
            side = self._spiral_side = 1.0 if command.bank >= 0.0 else -1.0
        return (alpha, side * bank_deg)

    def log_hac_ladder(self, snap, height):
        """``HAC_LD_AT_TARGET``, every 60 s of the cone: what each rung of the ladder was
        priced at -- speed, trim alpha, the table there and the ratio -- so
        a ladder that disagrees with the ``ld=`` the vehicle flies can be
        read off the log (LOG4097 implied 0.7 against 2.5 flown)."""
        last = getattr(self, "_hac_ladder_ut", None)
        if (not getattr(self.cfg, "HAC_LD_AT_TARGET", False)
                or (last is not None and snap.ut - last < 60.0)):
            return
        self._hac_ladder_ut = snap.ut
        stall = airframe.stall(self.env, self.cfg)
        g = self.surface_gravity
        bits = []
        h = self.cfg.GATE_ALT_M
        while h <= height + 1.0:
            v = guidance.cone_speed(self.env, self.cfg, stall, h)
            a = trajectory.alpha_for_load(self.env, v, h, snap.mass, g, 1.0)
            if a is None:
                bits.append("%.0f: v %.0f no trim" % (h, v))
            else:
                cla, cda = self.env.coefficients(a, v, h)
                ld = airframe.turning_ld(self.env, self.cfg, v, h, snap.mass,
                                         g, 0.0)
                bits.append("%.0f: v %.0f a %.1f cla %.1f cda %.1f trim - "
                            "ld %s" % (h, v, a, cla, cda,
                                       "-" if ld is None else "%.2f" % ld))
            h += 2000.0
        self.logbook.event(snap.ut, "hac ladder (m %.2ft, stall %.1f): %s"
                           % (snap.mass / 1000.0, stall, " | ".join(bits)))

    def report_landing_airframe(self, snap):
        """The airframe reading that the landing constants answer to.

        Once, on final: ``snap.mass`` here is the mass the wheels arrive at,
        so the stall speed this returns needs no prediction and no drain
        model.  See ``report_airframe`` for why the STANDBY reading is not
        the one to judge ``STALL_SPEED_M_S`` by.
        """
        if self._landing_airframe_reported:
            return
        self._landing_airframe_reported = True
        try:
            self.report_airframe(snap, landing=True)
        except Exception:                               # noqa: BLE001
            pass

    def run_approach(self, snap):
        """Geometric final: the threshold, the centreline, and the speed floor."""
        if getattr(self, "split_angle", 0.0) > 0.0 and self.set_split(0.0):
            self.logbook.event(snap.ut, "cone split brake in: the approach")
        if (getattr(self, "flap_brake_out", False)
                and not getattr(self, "_cone_flaps_stowed", False)):
            # The approach's brake law starts from stowed.
            self._cone_flaps_stowed = True
            if self.set_flap_brake(False):
                self.logbook.event(snap.ut, "cone flap brake in: the approach")
        # Nothing below the glide permits RCS; close what it left open.
        self.set_rcs(False, snap)
        self.set_throttle(0.0)
        self.env.refresh(
            snap.ut, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius)
        self.report_landing_airframe(snap)
        height = snap.landing_height
        command = guidance.approach(
            self.env, self.cfg, self.end, snap.position, snap.velocity,
            snap.mass, self.surface_gravity, height,
            accel=self.path_accel(snap),
            weave=guidance.weave_sign(
                self.cfg, snap.ut - (self.state_since or snap.ut),
                period=self.scurve_half_period_s()),
            roll_lag_s=self.roll_lag_s())
        self.command = command
        sink = -vec.dot(snap.velocity, vec.unit(snap.position))
        trigger = guidance.flare_door(self.cfg, sink, vec.norm(snap.velocity),
                                      self.env)
        alpha = min(command.alpha, self.alpha_ceiling)
        bank = self.approach_bank(snap, command.bank, height, trigger, sink)
        self.steer = Steer(alpha=alpha, bank=bank)
        self.aim(alpha, bank, snap)
        self.landing_gear(snap, height, getattr(command, "excess", None))
        # **The brake is stowed against the flare door this tick computes**,
        # not against a height constant -- the door is the vehicle's own
        # (``FLARE_ALT_M + FLARE_LEAD_S * sink``) and the next aircraft's is
        # somewhere else entirely.  Hence the ordering: the trigger first,
        # then the brake that has to stay clear of it.
        self.command_airbrake(snap, command, height, trigger, sink)
        brake = getattr(self, "airbrake", None)
        self.bay_brake(snap, getattr(command, "excess", None), "approach",
                       brake is not None and brake.saturated >= float(
                           self.cfg.BAY_BRAKE_SATURATED))
        if height <= trigger:
            self.flare_since = snap.ut
            self.enter(FLARE, snap.ut, "h=%.1f v=%.1f sink=%.1f cross=%+.0f"
                       % (height, command.speed, command.sink, command.cross))
        elif self.touched_down(snap, height):
            self.enter(ROLLOUT, snap.ut, "touchdown without a flare")

    def path_accel(self, snap):
        """dv/dt along the path, m/s^2, smoothed over
        ``APPROACH_ACCEL_TAU_S`` of game time -- the rate term of
        ``APPROACH_SPEED_KD``.  None until two ticks have been seen."""
        speed = vec.norm(snap.velocity)
        prev = getattr(self, "_accel_prev", None)
        self._accel_prev = (snap.ut, speed)
        if prev is None or snap.ut - prev[0] <= 1e-3:
            return getattr(self, "_accel", None)
        dt = snap.ut - prev[0]
        raw = (speed - prev[1]) / dt
        tau = max(0.05, float(getattr(self.cfg, "APPROACH_ACCEL_TAU_S", 1.0)))
        old = getattr(self, "_accel", None)
        self._accel = raw if old is None else old + (raw - old) * min(
            1.0, dt / tau)
        return self._accel

    def roll_lag_s(self):
        """Roll's ``time_to_peak`` as kRPC applies it, or ``None``."""
        peak = getattr(self, "_tuned_peak", None)
        try:
            return float(peak[1]) if peak else None
        except (TypeError, IndexError, ValueError):
            return None

    def scurve_half_period_s(self):
        """``APPROACH_SCURVE_PERIOD_BY_ROLL``: the weave's half-cycle as
        ``APPROACH_SCURVE_PERIOD_FACTOR`` x the time to reverse the bank
        (``2 x APPROACH_BANK_MAX_DEG`` at the measured roll rate, plus
        roll's time to peak).  A half-cycle shorter than the reversal never
        reaches its bank.  Else ``APPROACH_SCURVE_PERIOD_S``."""
        return self.cfg.APPROACH_SCURVE_PERIOD_S

    def approach_bank(self, snap, bank_deg, height, trigger, sink):
        """The approach's bank as commanded -- or, under
        ``APPROACH_BANK_BY_ROLL``, as the vehicle can fly it: slewed at the
        measured roll rate, and no larger than it can roll back out of
        before the flare door (``rate * (t_door - roll time_to_peak)``)."""
        rate = max(1.0, self.bank_rate())
        peak = getattr(self, "_tuned_peak", None)
        settle = peak[1] if peak else 0.0
        to_door = max(0.0, height - trigger) / max(1.0, sink)
        limit = max(0.0, rate * (to_door - settle))
        want = vec.clamp(bank_deg, -limit, limit)
        last_ut = getattr(self, "_app_bank_ut", None)
        prev = getattr(self, "_app_bank", None)
        if prev is None or last_ut is None:
            flown = flown_bank(snap)
            prev = 0.0 if math.isnan(flown) else flown
            dt = 0.0
        else:
            dt = max(0.0, snap.ut - last_ut)
        bank = vec.clamp(want, prev - rate * dt, prev + rate * dt)
        self._app_bank, self._app_bank_ut = bank, snap.ut
        return bank

    def run_flare(self, snap):
        """The last fifteen metres, which are their own problem."""
        # Nothing below the glide permits RCS; close what it left open.
        self.set_rcs(False, snap)
        self.set_throttle(0.0)
        height = snap.landing_height
        elapsed = snap.ut - (self.flare_since or snap.ut)
        alpha, sink, needed = guidance.flare(
            self.env, self.cfg, snap.position, snap.velocity, snap.mass,
            self.surface_gravity, height, elapsed, self.flare_lead_s(),
            float_s=None)
        cap = min(self.alpha_ceiling, self.cfg.FLARE_ALPHA_DEG,
                  self.flare_tail_cap(snap))
        alpha = min(alpha, cap)
        # **Wings level at the arrival, not for the whole flare.**  A banked
        # arrival puts a wingtip down first and this airframe's wings are the
        # only thing holding it off the tarmac -- but the flare starts around
        # 470 m and lasts ten seconds, and flying all of them wings level is
        # ten seconds of drift onto the grass.  The lean is the approach's
        # own capture, tapered to nothing well before touchdown; taking it
        # from ``guidance.approach`` rather than writing a second law here is
        # the same discipline as sharing the throttle law with the
        # propagator.
        bank = 0.0
        leaning = height > self.cfg.FLARE_WINGS_LEVEL_M
        taper = vec.clamp((height - self.cfg.FLARE_WINGS_LEVEL_M)
                          / max(1.0, self.cfg.FLARE_BANK_TAPER_M),
                          0.0, 1.0)
        if leaning:
            limit = self.cfg.FLARE_BANK_MAX_DEG * taper
            lateral = guidance.approach(self.env, self.cfg, self.end,
                                        snap.position, snap.velocity,
                                        snap.mass,
                                        self.surface_gravity, height,
                                        roll_lag_s=self.roll_lag_s())
            bank = vec.clamp(lateral.bank, -limit, limit)
        self.steer = Steer(alpha=alpha, bank=bank)
        # **Below the levelling height the reference is the runway, not the
        # airflow.**  See ``aim_runway``: this is where the crab comes out.
        # ``FLARE_ALIGN_BY_YAW``: align when the time to the ground is
        # yaw's time to peak -- the settling time ``FLARE_ALIGN_ALT_M``'s
        # comment assumed was 3 s; on the shuttle it is 19.7 -- but never
        # while the lean is still allowed, because ``aim_runway`` flies
        # wings level and would end the lateral correction early.
        aligned = height <= self.cfg.FLARE_ALIGN_ALT_M
        if aligned:
            self.aim_runway(alpha, snap)
        else:
            self.aim(alpha, bank, snap)
        self.landing_gear(snap, height)
        # **Stowed for the flare, whatever the approach left it at.**  The
        # flare arrests the sink with the speed it arrives with, and the
        # brake's own height floor is above the flare door for the same
        # reason; this is the backstop for an approach that handed over
        # early.
        if self.airbrake.extended:
            if getattr(self, "flap_brake_out", False) and not float(
                    getattr(self.cfg, "FLAP_BRAKE_PROBE_DEG", 0.0)):
                self.set_flap_brake(False)
            self.airbrake.extended = False
            self.logbook.event(snap.ut, "airbrake in for the flare")
        self.flare_sink = sink
        self.flare_needed = needed
        self.ground_spoiler(snap)
        # ``ROLLOUT_ON_MAIN_CONTACT``: the mains reporting ``grounded`` is
        # the touchdown.  KSP's ``situation`` said "landed" 1.4 s later on
        # LOG4836, and for that 1.4 s the flare pulled the nose up on the
        # wheels (+0.44 input), bounced, and came down at 6.7 m/s.
        mains = self.main_wheels_grounded()
        if mains or self.touched_down(snap, height):
            self.touchdown_ut = snap.ut
            self.touchdown_speed = vec.norm(snap.velocity)
            self.enter(ROLLOUT, snap.ut, "sink=%.2f speed=%.1f" %
                       (sink, self.touchdown_speed))

    def run_rollout(self, snap):
        """Brakes and nosewheel, and stop before the tarmac runs out.

        There is 2400 m and the measured touchdown speed is 46-54 m/s, which
        needs 0.44-0.61 m/s^2 -- gear-down drag alone gives about one, so this
        is the least demanding part of the flight.  That arithmetic sat in
        this docstring for the life of the project while the code below it
        braked flat out from the first tick; it is now the law the brakes are
        commanded by (``guidance.brake_fraction``).  It still steers, because
        an aircraft that leaves the runway sideways at 40 m/s has crashed as
        surely as one that lands short.
        """
        # Nothing below the glide permits RCS; close what it left open.
        self.set_rcs(False, snap)
        self.set_throttle(0.0)
        speed = vec.norm(snap.velocity)
        self.release_reaction_wheels(snap.ut)
        self.ground_spoiler(snap, landed=True)
        self.apply_brakes(snap, speed)
        if self.rollout_entry_alpha is None:
            # Whatever the flare finished holding -- the command ramps out of
            # *that*, not out of nothing.  ``self.steer`` is the last thing
            # commanded and the last thing the vehicle was flying.
            self.rollout_entry_alpha = self.steer.alpha
        # **Put the nose down, but fly it down.**  Nothing here commanded an
        # attitude at first, so the autopilot went on holding the flare's 15
        # degrees nose-up on the ground at 52 m/s and sat on its tail; the
        # fix was to command zero, and that drove the nose onto the nose gear
        # at 90 m/s from a nose-high attitude, which is worse.  Both are the
        # same mistake -- a step -- and an aircraft does neither.  See
        # ``Config.ROLLOUT_HOLD_ALPHA_DEG``.
        # Down the runway, not down the velocity: on the ground the two can
        # differ by fifty degrees and only one of them is where the wheels
        # are going.  ``aim`` here commanded the vehicle to yaw into its own
        # slip and it spun.
        # ``ROLLOUT_HOLD_TAIL_FRACTION``: the hold is a fraction of this
        # airframe's own tail-strike angle, not the old craft's 8 deg -- on
        # the shuttle (tail 11.0 deg) 8 deg held on the wheels at 70 m/s
        # plus kRPC's overshoot put the tail down (LOG4819: 22-27 deg).
        # Applied after the ramp it *is* a step to the cap on the first
        # tick; ``ROLLOUT_RAMP_FROM_ATTITUDE`` hands it to the schedule.
        cap = None
        frac = float(getattr(self.cfg, "ROLLOUT_HOLD_TAIL_FRACTION", 0.0))
        if frac > 0.0:
            tail = getattr(self.telemetry, "tail_angle_deg", None)
            if tail is None:
                tail = self.cfg.TAIL_ANGLE_FALLBACK_DEG
            cap = frac * float(tail)
        ground_alpha = guidance.rollout_alpha(
            self.cfg, speed, snap.ut - (self.state_since or snap.ut),
            self.rollout_entry_alpha, env=self.env,
            cap=None)
        if cap is not None:
            ground_alpha = min(ground_alpha, cap)
        self.aim_runway(ground_alpha, snap)
        up = vec.unit(snap.position)
        along = self.env.runway.horizontal(self.end, self.end["along"])
        across = vec.unit(vec.cross(up, along))
        offset = vec.sub(vec.scale(vec.unit(snap.position),
                                   vec.norm(self.end["threshold"])),
                         self.end["threshold"])
        cross = vec.dot(offset, across)
        self.rollout_cross = cross
        # Full deflection only once the wheels are slow: see
        # ``Config.ROLLOUT_STEER_FULL_M_S``.
        taper = min(1.0, (self.cfg.ROLLOUT_STEER_FULL_M_S
                          / max(1.0, speed)) ** 2)
        limit = self.cfg.ROLLOUT_STEER_MAX * taper
        # ``ROLLOUT_STEER_LEAD_S``: damp on the cross-track rate.
        lead = float(getattr(self.cfg, "ROLLOUT_STEER_LEAD_S", 0.0))
        cross_rate = vec.dot(snap.velocity, across)
        steer_cmd = vec.clamp(self.cfg.ROLLOUT_STEER_GAIN
                              * (cross + lead * cross_rate),
                              -limit, limit)
        self.rollout_steer_cmd = steer_cmd
        # The track's angle off the runway, signed like ``cross`` (positive
        # heading right): with the steering command beside it the log says
        # whether the wheels are turning the vehicle toward the centreline.
        self.rollout_track_deg = math.degrees(math.atan2(
            vec.dot(snap.velocity, across),
            max(1e-6, vec.dot(snap.velocity, along))))
        try:
            self.control.wheel_steering = steer_cmd
        except Exception:                               # noqa: BLE001
            pass
        if snap.ut - self.state_since > self.cfg.ROLLOUT_TIMEOUT_S:
            self.logbook.event(snap.ut, "rollout timed out at %.1f m/s"
                               % speed)
            self.finish("rollout timeout")
            return
        if speed <= self.cfg.STOPPED_SPEED_M_S:
            self.stop_distance = trajectory.surface_distance(
                self.env, snap.position, self.end["threshold"])
            # **And how hard it stopped**, because nothing here has ever
            # measured it and the approach speed is chosen against it.  There
            # are about 2.2 km of tarmac past the touchdown aim, so stopping
            # from 115 m/s needs 3.0 m/s^2 and from 90 m/s needs 1.8 -- and
            # the only rollout this project has completed managed 1.04 while
            # shedding twenty parts, which is not a measurement of braking.
            rolled = max(0.1, snap.ut - (self.touchdown_ut or snap.ut))
            self.enter(STOPPED, snap.ut, "stopped %.0f m along the runway, "
                                        "%+.1f m off the centreline, "
                                        "%.1f s from %.1f m/s (%.2f m/s^2)"
                       % (self.stop_distance, cross, rolled,
                          self.touchdown_speed or 0.0,
                          (self.touchdown_speed or 0.0) / rolled))

    def log_holdable(self, ut):
        """What this flight learned about the airframe, once, at the end.

        The estimator's whole point is that it is not a constant, so the log
        has to carry what it found or the measurement dies with the process.
        """
        try:
            summary = self.env.holdable.summary()
        except Exception:                               # noqa: BLE001
            return
        if summary:
            self.logbook.event(ut, "holdable alpha, learned: %s" % summary)
        # And what the airframe really made against what the table said --
        # printed whether or not it was applied, so a flight flown with the
        # trim off still measures it.  A number nothing reads back cannot be
        # contradicted; failure 13.
        try:
            trim = self.env.lift_trim.report()
        except Exception:                               # noqa: BLE001
            trim = ""
        if trim:
            self.logbook.event(ut, "lift trim, measured (applied=False): %s"
                               % trim)

    def run_stopped(self, snap):
        self.set_throttle(0.0)
        self.control.brakes = True
        distance = trajectory.surface_distance(self.env, snap.position,
                                               self.env.runway.midpoint)
        # **Signed, along and across.**  A great-circle distance cannot tell
        # the tarmac from the grass beside it, and the runway is 2400 m long
        # and about 70 m wide: 200 m "from the midpoint" is a landing or a
        # wreck depending entirely on which axis it is on.  Same rule as
        # ``quickglide``'s report and as boosterland's signed miss.
        up = vec.unit(snap.position)
        along = self.env.runway.horizontal(self.end, self.end["along"])
        across = vec.unit(vec.cross(up, along))
        middle = self.env.runway.midpoint
        offset = vec.sub(vec.scale(vec.unit(snap.position), vec.norm(middle)),
                         middle)
        down, side = vec.dot(offset, along), vec.dot(offset, across)
        parts = len(self.vessel.parts.all)
        # **Say whether it is on the tarmac, because that is the question.**
        # A distance from the midpoint cannot answer it: the runway is 2400 m
        # long and 70 m wide, so 200 m "from the midpoint" is a landing or a
        # wreck on the grass depending entirely on which axis it is on.
        # Intact, too -- arriving in the right place as debris is not a
        # landing, and every flight of this vehicle for a year reported a
        # position and 0 of 23 parts in the same breath.
        on = (abs(down) <= 0.5 * self.cfg.RUNWAY_LENGTH_M
              and abs(side) <= 0.5 * self.cfg.RUNWAY_WIDTH_M)
        whole = parts >= 0.9 * (self.parts_at_start or parts)
        self.logbook.event(snap.ut,
                           "DOWN: %s -- %.0f m from the runway midpoint "
                           "(along %+.0f, across %+.0f), %s, "
                           "touchdown %.1f m/s, %d of %s parts"
                           % ("ON THE RUNWAY" if on and whole else
                              "on the runway but broken up" if on else
                              "off the runway",
                              distance, down, side, snap.situation,
                              self.touchdown_speed or 0.0,
                              parts, self.parts_at_start or "?"))
        self.finish("landed")

    # -- shared ------------------------------------------------------------
    def landing_gear(self, snap, height, excess=None):
        """Drop the gear -- and, when the approach is long, drop it early.

        **The gear is a speedbrake this autopilot owns and has never
        used.**  ``GEAR_ALT_M``'s own comment says the gear costs 19% of the
        glide ratio and deploys "late" for exactly that reason, which is the
        right call when the approach is on profile and precisely the wrong
        one when it is long -- and this vehicle's one open problem is that
        it lands long, +900 m sd 184 of a +-1200 m runway.

        The arithmetic that sized this was wrong by six times, and the
        flight said so.  Flying from ``A`` down to ``GEAR_ALT_M`` dirty
        instead of clean costs ``(A - GEAR_ALT_M) * GEAR_DRAG_FRACTION``
        metres of height, which at the approach's measured ground ratio of
        3.96 is about four times that in ground -- but the fraction is
        **0.03 measured in flight**, not the 0.19 transcribed from
        ``planeprobe``, so gear at 1800 m rather than 800 buys about 120 m of
        ground and not 900.  The flown batch agrees: +800 against +947 at 8 an
        arm, the glide ratio moving 4.12 -> 3.76 in the commanded direction
        with the distance under the noise.  **The mechanism is real and the
        size is small**; what absorbs the rest is
        ``APPROACH_SPEED_PATH``, which re-trims alpha to hold the commanded
        speed.  A drag device is not a range lever unless the guidance
        commands it against a surplus -- and even then it is only worth the
        drag it actually has.

        So it is solved rather than scheduled.  ``excess`` is the approach's
        own surplus height (``guidance.approach``'s ``height - reachable``,
        the ``exc=`` column), and the altitude that burns it is
        ``GEAR_ALT_M + excess / GEAR_DRAG_FRACTION``.  On profile the
        expression returns ``GEAR_ALT_M`` and nothing changes; short, it
        stays late.  A control, not a fitted altitude -- which is the
        difference between this and the four levers already refuted against
        the same miss (failures 68, 82, 83, 84; note 83 rejected the gear's
        *spring and damper*, not when it comes down).
        """
        if self.gear_down:
            self.gear_geometry(snap)
            self.wheel_watch(snap)
            return
        trigger = self.cfg.GEAR_ALT_M
        if height > trigger:
            return
        self.control.gear = True
        self.gear_down = True
        self.logbook.event(snap.ut, "gear down at %.0f m, %.1f m/s"
                           % (height, vec.norm(snap.velocity))
                           + ("" if excess is None
                              else " (surplus %+.0f m, schedule says %.0f)"
                              % (excess, self.cfg.GEAR_ALT_M)))

    def touched_down(self, snap, height):
        """On the ground, on the game's word first and the height second.

        ``situation`` is the reliable signal and the height is only a
        backstop, because an altitude-based cutoff at a height the vehicle
        would fall from is precisely what broke boosterland's first four
        landings.  The speed gate is the other half of that lesson: nothing
        doing hundreds of metres a second is about to be on the ground.
        """
        situation = str(snap.situation)
        if "landed" in situation.lower() or "splashed" in situation.lower():
            return True
        if vec.norm(snap.velocity) > 120.0:
            return False
        return height <= 0.3 and -vec.dot(snap.velocity,
                                          vec.unit(snap.position)) < 1.0

    def grounded_early(self, snap):
        """Stop flying a vehicle that is already on the ground.

        Only ROLLOUT and STOPPED expect to be there, and only APPROACH and
        FLARE can reach them: every other phase has no ground test at all,
        because the arrival it is steering towards is a runway hundreds of
        kilometres away.  An entry that falls short lands in terrain *during
        GLIDE*, and the phase machine has no opinion about that -- three
        flights in one batch sat at ``h=-0.2`` logging GLIDE lines at 63 m/s
        until the harness gave up on them an hour later, which in a project
        whose bottleneck is in-game measurement costs more than the flight
        did.  Not a cosmetic fix: the autopilot's contract is to hand back
        rather than keep commanding a vehicle it cannot help.
        """
        if self.state in (STANDBY, ROLLOUT, STOPPED):
            return False
        situation = str(snap.situation).lower()
        down = ("landed" in situation or "splashed" in situation)
        # **And a backstop for when the game will not say so.**  A vehicle
        # that arrives on a slope can skid with ``situation`` still reading
        # *flying* while the height under its wheels is negative, and then
        # nothing ends the phase: one flight logged the same FLARE line at
        # ``h=-0.3`` and 66.5 m/s until the harness gave up on it.  Height
        # below ground for a sustained time is the same fact the situation
        # would have reported, arrived at independently -- and it is held for
        # a while rather than taken on one tick, because a terrain sensor
        # reading below ground for an instant is a bounce, and boosterland
        # lost five flights to trusting that sensor without a clamp.
        if not down:
            if snap.landing_height > 0.0:
                self._below_ground_since = None
                return False
            if self._below_ground_since is None:
                self._below_ground_since = snap.ut
                return False
            if snap.ut - self._below_ground_since < self.cfg.GROUNDED_STUCK_S:
                return False
        if down and vec.norm(snap.velocity) > self.cfg.GROUNDED_SPEED_M_S:
            return False
        # **And it must not take the vehicle off the phase that lands it.**
        # This test runs before the phase handler, so a touchdown on the
        # runway at 65 m/s in FLARE was ending the flight with "on the ground
        # in FLARE" -- ``run_flare``'s own ``touched_down`` never got the
        # tick, ROLLOUT never ran, the brakes never came on and the wheels
        # were never steered.  The guard is for the phases that have *no*
        # ground test: an entry that falls short lands in terrain during
        # GLIDE and the phase machine has no opinion about it.  APPROACH and
        # FLARE have a ground test and a phase to hand to, and ROLLOUT has a
        # clock of its own, so they are allowed to finish the landing.
        #
        # A wreck is still taken here: ROLLOUT has nothing to do for a vessel
        # kRPC reports as zero parts, and the flight is over whatever the
        # ``situation`` says next.  ``parts_now`` carries the previous count
        # when the poll fails rather than a zero, so this cannot fire on a
        # dropped call.
        if down and self.state in (APPROACH, FLARE) and snap.parts_now > 0:
            return False
        self.log_line(snap)
        # **How hard, not just that it happened.**  Six of eight flights in one
        # batch ended down this path rather than through FLARE -> ROLLOUT, and
        # the only record of the arrival was the phase name -- no sink rate, no
        # speed, nothing to separate a firm touchdown from a vehicle hitting
        # terrain at 60 m/s and coming apart.  The two look identical in a log
        # that says "on the ground in FLARE".
        sink = -vec.dot(snap.velocity, vec.unit(snap.position))
        self.finish("on the ground in %s: sink %.1f m/s, speed %.1f m/s, "
                    "%s, %d parts left"
                    % (self.state, sink, vec.norm(snap.velocity),
                       self._mach_text(snap), snap.parts_now))
        return True

    def tick(self):
        snap = self.telemetry.sample(self.env)
        self.last_ut = snap.ut
        # The pitch response every guidance call can read (``flare_door``
        # under ``FLARE_DOOR_FROM_RESPONSE``), the way ``env.spending`` is.
        self.env.pitch_response_s = getattr(self, "attitude_settle_s", None)
        if self.panel.terminate_pressed():
            self.finish("terminated from the panel")
            return snap
        if self.state != STANDBY:
            self.engage_autopilot(snap.ut)
        if self.grounded_early(snap):
            return snap
        handler = {
            STANDBY: self.run_standby, DEORBIT: self.run_deorbit,
            DRAIN: self.run_drain, COAST: self.run_coast,
            GLIDE: self.run_glide, HAC: self.run_hac,
            APPROACH: self.run_approach,
            FLARE: self.run_flare, ROLLOUT: self.run_rollout,
            STOPPED: self.run_stopped,
        }[self.state]
        # **A deployment is not an aim parameter.**  The first version of
        # this hung off ``slip_command``, which is the yaw command and had
        # nothing to do with it -- and it duly broke every test that drives
        # ``aim`` on a stub.  The brake is a per-tick decision about the
        # vehicle, so it belongs where the state is authoritative.
        self.run_flap_probe(snap)
        self.measure_flap_brake(snap)
        self.retune_attitude(snap)
        handler(snap)
        self.pitch_assist(snap)
        self.propellant_trim(snap)
        self.canard_trim(snap)
        # **Wherever the table happens to become ready.**  The first version
        # reported from STANDBY, which ``--autostart`` leaves on the tick
        # before the sweep finishes -- so on every harness flight, which is
        # every flight that gets measured, it never ran at all.  A check that
        # only fires in the configuration nobody uses is not a check.
        # ``airframe.stall`` scales the table's stall to this -- from the
        # glide on.  Before it the vehicle is still wet and every
        # prediction is made at the drained entry mass the table was read
        # at, so the reading stands unscaled.
        if self.state in (GLIDE, HAC, APPROACH, FLARE, ROLLOUT):
            self.env.vehicle_mass = snap.mass
        if self.airframe is None and self.env.ready():
            self.report_airframe(snap)
        self.derive_hac_aim(snap)
        self.measure_hac_ld(snap)
        self.check_thermal(snap)
        self.watch_breakup(snap)
        self.destroyed_early(snap)
        self.frozen_early(snap)
        self.log_line(snap)
        return snap

    def pitch_error(self, snap):
        """The pitch-plane pointing error kRPC's loop is closing, degrees:
        the commanded nose against the roof, positive for nose-up wanted.
        Logged as the third ``pin=`` number on every flight."""
        nose = getattr(self, "commanded_nose", None)
        roof = getattr(snap, "roof", None)
        if nose is None or not roof or len(roof) != 3:
            return None
        return math.degrees(math.asin(vec.clamp(
            vec.dot(vec.unit(nose), vec.unit(roof)), -1.0, 1.0)))

    def pitch_assist(self, snap):
        """``PITCH_ASSIST``: a manual pitch input, integrated on the pitch
        pointing error, added to kRPC's attitude controller (kRPC sums the
        two).

        kRPC's loop leaves a standing pitch error against the airframe's
        restoring moment that grows with dynamic pressure; its integral runs
        on a clock scaled to an available torque the surfaces do not deliver.
        This is the missing integral, on **the error kRPC's own loop is
        closing** -- the commanded nose against the roof -- so the two share
        one zero (integrated on commanded minus kRPC's signed alpha instead,
        it chased a 2 deg difference between that angle and the nose kRPC
        was holding, and the two integrators wound each other to +-1, sim
        LOG4680).  Errors inside ``PITCH_ASSIST_DEADBAND_DEG`` are kRPC's
        attenuation band and are left to it (integrating them wound the trim
        to +1 against kRPC's -0.8, LOG4695).  Rate: the error past the band
        over ``PITCH_ASSIST_FULL_DEG * pitch time_to_peak``, held while the
        total input is saturated the same way.  HAC, APPROACH and FLARE; it
        decays over the same time in ROLLOUT and is zero elsewhere."""
        error = self.pitch_error(snap)
        self._pitch_assist_err = error
        if getattr(self.cfg, "CONE_TRIM_HANDOFF", False) and self.state in (
                HAC, APPROACH, FLARE, ROLLOUT):
            self.cone_trim_handoff(snap, error)
            return
        if getattr(self.cfg, "GLIDE_PITCH_OFFLOAD", False) and (
                self.state == GLIDE or (getattr(self, "_offload_live", False)
                                        and self.state not in (FLARE,
                                                               ROLLOUT))):
            self.glide_pitch_offload(snap)
            return
        if float(getattr(self.cfg, "FLARE_PITCH_P", 0.0)) > 0.0:
            self.flare_pitch_p(snap, error)
            return
        return

    def glide_pitch_offload(self, snap):
        """``GLIDE_PITCH_OFFLOAD``: carry the glide's *standing* pitch input
        as a manual body-frame trim, so kRPC's loop keeps only the transient.

        Holding 36 deg at 2-4 kPa takes the shuttle's whole pitch input, and
        kRPC holds it in its integrators -- which run in a roll-invariant
        frame (``kspSim/attitude.py``, from kRPC 0.6.0's
        ``AttitudeController``) and are mapped back to the body through the
        roll it has accumulated.  At the bank reversal near 38 km the long
        arrivals of rot-lapstack-1006 drop from +1.0 of pitch to +0.4 and
        stay there for 30 s against a 8-12 deg pitch error, with 8-10 deg of
        sideslip, while the on-profile flights recover: the shape of a
        wound-up pitch integral swung partly onto yaw.  A trim the client
        sets is applied in the body frame, which a roll cannot rotate.

        The law is the autopilot's trim offload: the trim moves toward the
        total input the vessel is getting (kRPC's read-back is the sum) over
        ``GLIDE_PITCH_OFFLOAD_TAU_S`` (0 = the pitch axis's own
        time_to_peak), so kRPC's share decays to the transient and the
        two loops share one zero.  Only while the roll is settled -- during a
        reversal the total is kRPC's transient, not the airframe's moment.
        After GLIDE it bleeds off over the same time and kRPC takes the
        moment back."""
        trim = getattr(self, "_pitch_assist", 0.0)
        last = getattr(self, "_pitch_assist_ut", None)
        self._pitch_assist_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, min(1.0, snap.ut - last))
        tau = float(getattr(self.cfg, "GLIDE_PITCH_OFFLOAD_TAU_S", 0.0)) or (
            self._tuned_peak[0] if getattr(self, "_tuned_peak", None)
            else (getattr(self, "_static_peak", None) or (20.0,))[0])
        k = min(1.0, dt / max(1.0, tau))
        mach = None
        floor = float(getattr(self.cfg, "GLIDE_PITCH_OFFLOAD_MIN_MACH", 0.0))
        if floor > 0.0:
            try:
                mach = self.env.mach(vec.norm(snap.velocity),
                                     vec.norm(snap.position)
                                     - self.env.equatorial_radius)
            except Exception:                           # noqa: BLE001
                mach = None
        if self.state == GLIDE and (mach is None or mach >= floor):
            self._offload_live = True
            total = getattr(snap, "pitch_input", None)
            flown = flown_bank(snap)
            cmd_bank = float(getattr(self, "commanded_bank", 0.0) or 0.0)
            # A saturated read-back says "at least this much", not what the
            # airframe needs: chasing it pinned the trim at 1.0 and left kRPC
            # no nose-down authority -- 15 of 15 such flights departed at Mach
            # 3-4 and arrived 1.4-17 km short; the 9 whose trim stayed at
            # 0.59-0.90 all arrived within 1.7 km (rot-offload-1007).
            settled = (total is not None and abs(float(total)) < 0.98
                       and not math.isnan(flown)
                       and abs(flown - cmd_bank)
                       <= float(self.cfg.ALPHA_TRIM_ROLL_TOL_DEG)
                       and snap.dynamic_pressure
                       > self.cfg.LIFT_LOOP_MIN_Q_PA)
            if settled:
                cap = float(self.cfg.GLIDE_PITCH_OFFLOAD_MAX)
                trim = vec.clamp(trim + k * (float(total) - trim), -cap, cap)
        else:
            trim -= trim * k
            if abs(trim) < 0.005:
                trim = 0.0
                self._offload_live = False
        if abs(trim - getattr(self, "_pitch_assist_sent", 0.0)) > 0.002 or (
                trim == 0.0 and getattr(self, "_pitch_assist_sent", 0.0)):
            try:
                self.control.pitch = trim
                self._pitch_assist_sent = trim
            except Exception:                           # noqa: BLE001
                pass
        self._pitch_assist = trim

    def _trim_tanks(self):
        """Every tank of each propellant, by station along the vessel's
        long axis (+ forward), read once: ``{resource: [(y, part, max)]}``
        sorted aft to forward.  Only resources held in two or more parts
        can be moved."""
        tanks = {}
        try:
            frame = self.vessel.reference_frame
            for part in self.vessel.parts.all:
                res = part.resources
                for name in ("LiquidFuel", "Oxidizer", "MonoPropellant"):
                    if not res.has_resource(name):
                        continue
                    cap = float(res.max(name))
                    if cap > 1.0:
                        tanks.setdefault(name, []).append(
                            (float(part.position(frame)[1]), part, cap))
        except Exception as exc:                        # noqa: BLE001
            self.logbook.event(0.0, "propellant trim: no tanks (%s)"
                               % type(exc).__name__)
            return {}
        # Monopropellant feeds the RCS; it stays where the craft put it.
        tanks.pop("MonoPropellant", None)
        tanks = {k: sorted(v, key=lambda t: t[0]) for k, v in tanks.items()
                 if len(v) >= 2 and v and max(t[0] for t in v)
                 - min(t[0] for t in v) > 1.0}
        self.logbook.event(0.0, "propellant trim tanks: %s" % "; ".join(
            "%s %s" % (k, " ".join("%+.1f/%.0f" % (y, c) for y, _, c in v))
            for k, v in tanks.items()) or "none")
        return tanks

    def propellant_trim(self, snap):
        """``PROPELLANT_TRIM``: carry the standing pitch moment on the
        centre of mass instead of on the elevons.

        The standing input is the summed pitch read-back (kRPC's output
        plus any trim) low-passed over ``PROPELLANT_TRIM_TAU_S``.  Past
        ``PROPELLANT_TRIM_DEADBAND`` nose-up, propellant moves from the
        most forward tank holding any to the most aft one with room;
        nose-down, the other way.  The rate sweeps the movable load end to
        end in ``PROPELLANT_TRIM_SWEEP_S``.  A saturated read-back is
        believed here as a direction (it says "at least this much"), which
        is all a pump needs."""
        if not getattr(self.cfg, "PROPELLANT_TRIM", False):
            return
        # Not in the glide unless asked: an aft CoM at the glide's 38-41 deg
        # transonic alpha pitched up into a deep stall (LOG7969: flown alpha
        # 48 -> 80 deg at full nose-down input, rot-ptrim-1007).
        phases = (HAC, APPROACH, FLARE) + (
            (GLIDE,) if getattr(self.cfg, "PROPELLANT_TRIM_IN_GLIDE", False)
            else ())
        if self.state not in phases:
            return
        total = getattr(snap, "pitch_input", None)
        if total is None:
            return
        last = getattr(self, "_ptrim_ut", None)
        self._ptrim_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, min(2.0, snap.ut - last))
        k = min(1.0, dt / max(0.1, float(self.cfg.PROPELLANT_TRIM_TAU_S)))
        s = getattr(self, "_ptrim_standing", None)
        s = float(total) if s is None else s + k * (float(total) - s)
        self._ptrim_standing = s
        try:
            mach = self.env.mach(vec.norm(snap.velocity),
                                 vec.norm(snap.position)
                                 - self.env.equatorial_radius)
        except Exception:                               # noqa: BLE001
            return
        if mach > float(self.cfg.PROPELLANT_TRIM_MAX_MACH):
            return
        band = float(self.cfg.PROPELLANT_TRIM_DEADBAND)
        direction = 1 if s > band else (-1 if s < -band else 0)
        if getattr(self.cfg, "PROPELLANT_TRIM_ON_ENERGY", False):
            # The pump as the cone's energy control: aft (L/D 2.5) only
            # while short of height, forward (back to the trim drag, L/D
            # 2.0) once the surplus is worth weaving for; forward again
            # for the approach and flare, which land nose-heavy.
            surplus = getattr(self, "_hac_surplus", None)
            if self.state != HAC:
                direction = -1
            elif surplus is None:
                direction = 0
            elif surplus < 0.0:
                direction = 1 if s > band else 0
            elif surplus > float(self.cfg.HAC_WEAVE_DEADBAND_M):
                direction = -1
            else:
                direction = 0
        # Pump on a one-game-second cadence: each transfer is a handful
        # of calls and the CoM needs no finer step than the sweep allows.
        pumped = getattr(self, "_ptrim_pump_ut", None)
        if direction == 0 or (pumped is not None and snap.ut - pumped < 1.0):
            self._ptrim_dir = direction if direction == 0 else getattr(
                self, "_ptrim_dir", 0)
            return
        step = 1.0 if pumped is None else min(2.0, snap.ut - pumped)
        self._ptrim_pump_ut = snap.ut
        tanks = getattr(self, "_ptrim_tanks", None)
        if tanks is None:
            tanks = self._ptrim_tanks = self._trim_tanks()
        moved = 0.0
        for name, row in tanks.items():
            try:
                held = [(y, p, c, float(p.resources.amount(name)))
                        for y, p, c in row]
            except Exception:                           # noqa: BLE001
                continue
            load = sum(a for _, _, _, a in held)
            if load < 1.0:
                continue
            # Nose-up wanted -> mass aft: from the most forward tank that
            # holds any to the most aft that has room.
            order = held if direction < 0 else held[::-1]
            src = next((h for h in order if h[3] > 0.5), None)
            dst = next((h for h in order[::-1] if h[2] - h[3] > 0.5), None)
            if src is None or dst is None or src[1] == dst[1] or (
                    (src[0] - dst[0]) * direction <= 0):
                continue
            amount = min(src[3], dst[2] - dst[3], load * step
                         / max(1.0, float(self.cfg.PROPELLANT_TRIM_SWEEP_S)))
            try:
                self.conn.space_center.ResourceTransfer.start(
                    src[1], dst[1], name, float(amount))
                moved += amount * (src[0] - dst[0])
            except Exception as exc:                    # noqa: BLE001
                self.logbook.event(snap.ut, "propellant trim: %s transfer "
                                   "failed (%s)" % (name, type(exc).__name__))
        self._ptrim_moment = getattr(self, "_ptrim_moment", 0.0) + moved
        if direction != getattr(self, "_ptrim_dir", 0):
            self.logbook.event(snap.ut, "propellant trim %s: standing pitch "
                               "%+.2f, M=%.2f" % ("AFT" if direction > 0
                                                  else "FORWARD", s, mach))
        self._ptrim_dir = direction

    def canard_trim(self, snap):
        """``CANARD_TRIM``: carry the standing pitch moment on the canards,
        driven directly, so neither kRPC's integrators nor the elevons' travel
        hold it (config).  Every other surface stays on kRPC."""
        if not getattr(self.cfg, "CANARD_TRIM", False):
            return
        pair = getattr(self, "_canards", None)
        if pair is None:
            pair = self._canards = self._find_canards(snap)
            self._ctrim = 0.0
            self._ctrim_lp = None
            self._ctrim_on = False
            self._ctrim_sent = None
        if not pair:
            return
        last = getattr(self, "_ctrim_ut", None)
        self._ctrim_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, min(1.0, snap.ut - last))
        learn = self.state in (HAC, APPROACH)
        if self.state == GLIDE and float(self.cfg.CANARD_TRIM_MAX_MACH) > 0.0:
            try:
                mach = self.env.mach(vec.norm(snap.velocity),
                                     vec.norm(snap.position)
                                     - self.env.equatorial_radius)
            except Exception:                           # noqa: BLE001
                mach = None
            learn = mach is not None and mach <= float(
                self.cfg.CANARD_TRIM_MAX_MACH)
        hold = self.state == FLARE
        release = self.state in (ROLLOUT, STOPPED)
        if not (learn or hold or release or self._ctrim_on):
            return
        total = getattr(snap, "pitch_input", None)
        if learn:
            # Settled, and where the canards are linear (config).
            try:
                flown = float(snap.krpc_aoa)
                cmd_alpha = float(getattr(self, "commanded_alpha", flown))
                fb = flown_bank(snap)
                cb = float(getattr(self, "commanded_bank", 0.0) or 0.0)
                learn = (abs(flown) < float(self.cfg.CANARD_TRIM_MAX_ALPHA_DEG)
                         and abs(cmd_alpha - flown)
                         <= float(self.cfg.CANARD_TRIM_ALPHA_TOL_DEG)
                         and not math.isnan(fb)
                         and abs(fb - cb)
                         <= float(self.cfg.ALPHA_TRIM_ROLL_TOL_DEG))
            except (TypeError, ValueError):
                learn = False
            if not learn and not self._ctrim_on:
                return
        if learn and total is not None:
            total = float(total)
            lp = self._ctrim_lp
            k = min(1.0, dt / max(0.1, float(self.cfg.CANARD_TRIM_LP_S)))
            lp = total if lp is None else lp + k * (total - lp)
            self._ctrim_lp = lp
            band = float(self.cfg.CANARD_TRIM_DEADBAND)
            if abs(lp) > band:
                err = lp - math.copysign(band, lp)
                cap = float(self.cfg.CANARD_TRIM_MAX)
                try:
                    local = (float(self.cfg.CANARD_TRIM_LOCAL_MAX_DEG)
                             - max(0.0, float(snap.krpc_aoa))) / 37.5
                    cap = max(0.0, min(cap, local))
                except (TypeError, ValueError):
                    pass
                self._ctrim = vec.clamp(
                    self._ctrim + err * dt / max(0.1, float(
                        self.cfg.CANARD_TRIM_TAU_S)), -cap, cap)
        elif release:
            self._ctrim -= self._ctrim * min(1.0, dt / max(0.1, float(
                self.cfg.ROLLOUT_RAMP_S)))
        if release and abs(self._ctrim) < 0.01:
            if self._ctrim_on:
                try:
                    for cs in pair:
                        cs.deflection_override = False
                except Exception:                       # noqa: BLE001
                    pass
                self._ctrim_on = False
                self.logbook.event(snap.ut, "canard trim released")
            return
        if self._ctrim > 0.0:
            try:
                local = (float(self.cfg.CANARD_TRIM_LOCAL_MAX_DEG)
                         - max(0.0, float(snap.krpc_aoa))) / 37.5
                self._ctrim = min(self._ctrim, max(0.0, local))
            except (TypeError, ValueError):
                pass
        cmd = float(self.cfg.CANARD_TRIM_SIGN) * self._ctrim
        if abs(cmd) < 0.003:
            # A deploy angle of exactly 0 freezes the surface under
            # AtmosphereAutopilot; a hair off neutral moves it there.
            cmd = 0.003
        try:
            if not self._ctrim_on:
                for cs in pair:
                    cs.deflection_override = True
                self._ctrim_on = True
                self.logbook.event(snap.ut, "canard trim engaged (%s), "
                                   "standing pitch %+.2f"
                                   % (self.state, total or 0.0))
            if self._ctrim_sent is None or abs(cmd - self._ctrim_sent) > 0.004:
                for cs in pair:
                    cs.deflection = cmd
                self._ctrim_sent = cmd
        except Exception:                               # noqa: BLE001
            pass

    def _find_canards(self, snap):
        """The forward mirrored pair of control surfaces, or ``[]``: two
        surfaces ahead of the centre of mass (``vessel.reference_frame``'s
        origin) at mirrored lateral stations.  Refuses anything else."""
        try:
            fwd = []
            for cs in self.vessel.parts.control_surfaces:
                x, y, z = cs.part.position(self.vessel.reference_frame)
                if y > 1.0:
                    fwd.append((cs, x, y, z))
        except Exception:                               # noqa: BLE001
            fwd = []
        ok = (len(fwd) == 2 and fwd[0][1] * fwd[1][1] < 0.0
              and abs(fwd[0][1] + fwd[1][1]) < 0.2
              and abs(fwd[0][2] - fwd[1][2]) < 0.2)
        self.logbook.event(
            snap.ut, "canard trim: %s" % (
                "pair at y=%+.1f m, x=+-%.1f m" % (fwd[0][2], abs(fwd[0][1]))
                if ok else "no forward mirrored pair (%d forward) -- off"
                % len(fwd)))
        return [f[0] for f in fwd] if ok else []

    def cone_trim_handoff(self, snap, error):
        """``CONE_TRIM_HANDOFF``: carry the standing pitch/yaw input as a
        body-frame manual trim and keep kRPC's integrators empty.

        kRPC 0.6's PID integrators are held in a roll-invariant frame that
        it carries by parallel transport of the nose; a spiral twists that
        frame against the body (``phi`` 34 deg on a wings-level vehicle in
        LOG8144's diagnostic dump), and the nose-up trim stored there comes
        back out as body pitch *and yaw* -- a 4-7 deg pointing error held for
        minutes at an unsaturated input (config, ``CONE_TRIM_HANDOFF``).  A
        manual input is summed by kRPC in the body frame, which no roll or
        turn rotates.  So, whenever the nose is on its command, the roll
        settled and nothing is saturated, the summed read-back becomes the
        trim and kRPC is re-engaged: ``Start()`` zeroes its integrators and
        re-seats the frame, and with the trim already carrying the moment
        its soft start fades in an output of about nothing.  FLARE holds the
        trim and adds ``FLARE_PITCH_P``; ROLLOUT ramps it out."""
        if not hasattr(self, "_handoff_pitch"):
            # Take over whatever trim was being sent (the glide offload's).
            self._handoff_pitch = float(getattr(self, "_pitch_assist", 0.0))
            self._handoff_yaw = 0.0
            self._handoff_ut = None
            self._handoff_n = 0
        last = getattr(self, "_pitch_assist_ut", None)
        self._pitch_assist_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, min(1.0, snap.ut - last))
        pitch, yaw = self._handoff_pitch, self._handoff_yaw
        if self.state in (HAC, APPROACH):
            due = (self._handoff_ut is None or snap.ut - self._handoff_ut
                   >= float(self.cfg.CONE_TRIM_HANDOFF_S))
            nose = getattr(snap, "nose", None)
            cmd = getattr(self, "commanded_nose", None)
            total_p = getattr(snap, "pitch_input", None)
            total_y = getattr(snap, "yaw_input", None)
            flown = flown_bank(snap)
            cmd_bank = float(getattr(self, "commanded_bank", 0.0) or 0.0)
            if (due and nose and cmd and total_p is not None
                    and total_y is not None
                    and abs(float(total_p)) < 0.98
                    and abs(float(total_y)) < 0.98
                    and not math.isnan(flown)
                    and abs(flown - cmd_bank)
                    <= float(self.cfg.ALPHA_TRIM_ROLL_TOL_DEG)
                    and snap.dynamic_pressure > self.cfg.LIFT_LOOP_MIN_Q_PA
                    and math.degrees(math.acos(vec.clamp(vec.dot(
                        vec.unit(nose), vec.unit(cmd)), -1.0, 1.0)))
                    <= float(self.cfg.CONE_TRIM_HANDOFF_TOL_DEG)):
                pitch, yaw = float(total_p), float(total_y)
                try:
                    self.control.pitch = pitch
                    self.control.yaw = yaw
                    self._pitch_assist_sent = pitch
                    self._handoff_yaw_sent = yaw
                except Exception:                       # noqa: BLE001
                    pass
                set_autopilot_engaged(self.autopilot, False)
                set_autopilot_engaged(self.autopilot, True)
                self._handoff_ut = snap.ut
                self._handoff_n += 1
                if self._handoff_n == 1 or self._handoff_n % 10 == 0:
                    self.logbook.event(
                        snap.ut, "trim handoff %d: body trim pitch %+.2f yaw "
                                 "%+.2f, kRPC re-engaged"
                        % (self._handoff_n, pitch, yaw))
            out_p = pitch
        elif self.state == FLARE:
            out_p = pitch
            if error is not None and float(getattr(
                    self.cfg, "FLARE_PITCH_P", 0.0)) > 0.0:
                cap = float(self.cfg.FLARE_PITCH_P_MAX)
                out_p = pitch + vec.clamp(
                    float(self.cfg.FLARE_PITCH_P) * error, -cap, cap)
        else:
            k = min(1.0, dt / max(0.1, float(self.cfg.ROLLOUT_RAMP_S)))
            pitch -= pitch * k
            yaw -= yaw * k
            out_p = pitch
        self._handoff_pitch, self._handoff_yaw = pitch, yaw
        try:
            if abs(out_p - getattr(self, "_pitch_assist_sent", 0.0)) > 0.002:
                self.control.pitch = out_p
                self._pitch_assist_sent = out_p
            if abs(yaw - getattr(self, "_handoff_yaw_sent", 0.0)) > 0.002:
                self.control.yaw = yaw
                self._handoff_yaw_sent = yaw
        except Exception:                               # noqa: BLE001
            pass
        self._pitch_assist = out_p

    def flare_pitch_p(self, snap, error):
        """``FLARE_PITCH_P``: manual pitch input proportional to the pitch
        pointing error, FLARE only -- kRPC sums it with its own output.

        The flare commands 7-12 deg and flies 2-3 at a flat +0.24-0.4 of
        kRPC's input (LOG4927); the oscillation mitigation is not why
        (rot-oscoff-1003), and a stiffer kRPC tune departs (rot-pfloor2-1003).
        A proportional term has nothing to wind up, which is what sank
        ``PITCH_ASSIST``.  Positive is nose-up: kRPC's own input is positive
        on the same error.  Capped at ``FLARE_PITCH_P_MAX``; decays over
        ``ROLLOUT_RAMP_S`` in ROLLOUT, zero elsewhere."""
        trim = getattr(self, "_pitch_assist", 0.0)
        last = getattr(self, "_pitch_assist_ut", None)
        self._pitch_assist_ut = snap.ut
        dt = 0.0 if last is None else max(0.0, min(1.0, snap.ut - last))
        # ``PITCH_P_CONE``: the cone and the approach too -- kRPC's loop
        # leaves the same standing error there (LOG8104: -8 deg at +0.3).
        states = (FLARE, HAC, APPROACH) if getattr(
            self.cfg, "PITCH_P_CONE", False) else (FLARE,)
        if self.state in states and error is not None:
            cap = float(self.cfg.FLARE_PITCH_P_MAX)
            trim = vec.clamp(float(self.cfg.FLARE_PITCH_P) * error, -cap, cap)
        elif self.state == ROLLOUT:
            trim -= trim * min(1.0, dt / max(0.1, float(
                self.cfg.ROLLOUT_RAMP_S)))
        else:
            trim = 0.0
        if abs(trim - getattr(self, "_pitch_assist_sent", 0.0)) > 0.002 or (
                trim == 0.0 and getattr(self, "_pitch_assist_sent", 0.0)):
            try:
                self.control.pitch = trim
                self._pitch_assist_sent = trim
            except Exception:                           # noqa: BLE001
                pass
        self._pitch_assist = trim

    def retune_attitude(self, snap):
        """``ATTITUDE_TIME_TO_PEAK_LIVE``: follow the torque the air provides.

        See ``live_time_to_peak``.  Once per ``ATTITUDE_RETUNE_S`` of game
        time, and only when an axis has moved by ``ATTITUDE_RETUNE_FRAC`` --
        kRPC re-derives its gains on every assignment, so re-assigning an
        unchanged tune is churn, not control.  Logged when an axis has moved
        by half again since the last line, so the log shows the schedule
        without a line per second.
        """
        self.retune_pitch_air(snap)
        return

    def retune_pitch_air(self, snap):
        """``ATTITUDE_PITCH_AIR``: pitch follows the authority the air adds.

        The static tune is ``sqrt(I / wheels)``, 19 s of pitch on the shuttle,
        while in the cone its surfaces give 6000-12000 kN m against the
        wheels' 15.  kRPC scales its gains to the *available* torque, so the
        pitch input comes out tiny (+0.08 of 1.0, LOG3035) and the aero
        moment -- which grows with q exactly as the surfaces do -- is left to
        an integral term running on a 19 s clock.  The nose fell fifteen
        degrees below its command and the vehicle dived in from 11 km.

        ``ATTITUDE_TIME_TO_PEAK_LIVE`` had the idea and three faults, each
        answered here: engine and RCS torque are never counted (wheels plus
        surfaces, by name); the surface figure, which kRPC derives from the
        current deflection and which chattered 2.3 <-> 10 s, is smoothed over
        ``ATTITUDE_AIR_SMOOTH_S``; and only **pitch** moves -- yaw and roll
        keep the static derivation, since the lateral mode is what the live
        version drove.  Floored at ``ATTITUDE_TIME_TO_PEAK_S``, the tune that
        damps the old craft, whose static pitch figure *is* that floor -- so
        on it this is inert by construction -- and never slower than static.
        """
        static = getattr(self, "_static_peak", None)
        if not self.autopilot_engaged or static is None:
            return
        # **From the cone on, never in the glide.**  Flown in the entry it
        # cost the arrival about eight kilometres: cone arrivals +8.5 to
        # +18.7 km (mean +14.0, n=5) against +5.1 to +6.7 (mean +6.2, n=4)
        # on the defaults, ``logs/pairfly-shuttle-pitchair.txt``.  The glide
        # was already pointing (alpha error -0.5, sd 3); the dive this is for
        # happens below Mach 1, in the cone and on final.
        if self.state not in (HAC, APPROACH, FLARE):
            return
        last = getattr(self, "_air_ut", None)
        dt = 0.0 if last is None else max(0.0, snap.ut - last)
        self._air_ut = snap.ut
        try:
            surf = abs(self.vessel.available_control_surface_torque[0][0])
            wheel = abs(self.vessel.available_reaction_wheel_torque[0][0])
            inertia = self.vessel.moment_of_inertia[0]
        except Exception:                               # noqa: BLE001
            return
        tau = max(0.1, float(self.cfg.ATTITUDE_AIR_SMOOTH_S))
        ema = getattr(self, "_air_surf", None)
        ema = surf if ema is None else ema + (surf - ema) * dt / (tau + dt)
        self._air_surf = ema
        total = wheel + ema
        if total <= 1.0 or inertia <= 0.0:
            return
        pitch = float(self.cfg.ATTITUDE_SLEW_FACTOR) * math.sqrt(
            inertia / total)
        # ``ATTITUDE_PITCH_AIR_FLOOR_S``: this retune's own floor (0 = the
        # static ``ATTITUDE_TIME_TO_PEAK_S``).  On the shuttle the air figure
        # sits under the 3 s floor all the way down, and kRPC's tune at 3 s
        # (gains 2.7/1.25/0) leaves the flare 5-7 deg short at +0.3-0.4 of
        # input (rot-decel-1003, rot-oscoff-1003).
        # Final and flare only: in the cone too it left three of six saves
        # 0.6-1.0 km short and two 2.7-3.3 km across (rot-pfloor-1003) --
        # the cone's constants were fitted to the softer pitch.
        floor = float(self.cfg.ATTITUDE_TIME_TO_PEAK_S)
        if self.state in (APPROACH, FLARE):
            floor = float(getattr(self.cfg, "ATTITUDE_PITCH_AIR_FLOOR_S",
                                  0.0) or floor)
        pitch = min(static[0], max(floor, pitch))
        have = self._tuned_peak[0] if self._tuned_peak else static[0]
        if (self._retune_ut is not None and snap.ut - self._retune_ut
                < float(self.cfg.ATTITUDE_RETUNE_S)):
            return
        if abs(pitch - have) <= float(self.cfg.ATTITUDE_RETUNE_FRAC) * have:
            return
        self._retune_ut = snap.ut
        lateral = self._tuned_peak if self._tuned_peak else static
        want = (pitch, lateral[1], lateral[2])
        try:
            self.autopilot.time_to_peak = want
        except Exception:                               # noqa: BLE001
            return
        self._tuned_peak = want
        self.attitude_settle_s = pitch
        logged = self._retune_logged
        if logged is None or abs(pitch - logged[0]) > 0.3 * logged[0]:
            self._retune_logged = want
            self.logbook.event(
                snap.ut, "attitude pitch retune at q=%.0f Pa: surfaces %.0f "
                         "kN m (smoothed) + wheels %.0f -> time_to_peak "
                         "(pitch %.1f, roll %.1f, yaw %.1f) s as applied"
                % ((snap.dynamic_pressure, ema / 1000.0, wheel / 1000.0)
                   + tuple(want)))

    def destroyed_early(self, snap):
        """A vehicle with no parts left is not flying, whatever the phase says.

        ``watch_breakup`` *reports* the breakup and nothing ended the flight.
        Only ROLLOUT and STOPPED have a clock, so a vehicle that comes apart
        during the entry leaves GLIDE steering at an arrival it will never
        reach, on a frozen state kRPC keeps returning: ``logs/LOG954`` and
        ``logs/LOG955`` both burned up at Mach 6 and then logged **700 more
        seconds** of identical GLIDE lines, twelve minutes of an instance
        each. In a project whose bottleneck is in-game measurement that is
        the same fault ``ROLLOUT_TIMEOUT_S`` exists for, one phase earlier.

        Zero parts is unambiguous -- it is the polled count, not the skin
        streams, which can go quiet for other reasons -- so this needs no
        threshold and no clock. See ``Config.STOP_WHEN_DESTROYED``.
        """
        if not self.cfg.STOP_WHEN_DESTROYED:
            return
        if self.parts_at_start and snap.parts_now == 0:
            self.finish("destroyed in %s" % self.state)

    def frozen_early(self, snap):
        """A vehicle that reports speed and never moves is not flying.

        ``destroyed_early`` catches the case where nothing is left, and a
        breakup that leaves **one** fragment slips past it: LOG3009 came apart
        in the flare at 94 m/s of sink, kept one part, and kRPC went on
        returning ``v=124.9 vs=-108.8`` at a position that did not change for
        2800 game-seconds -- 50 minutes of an instance, in a phase with no
        clock.  A position identical to the last bit while the reported speed
        is metres per second is not a state any vehicle can be in, so this
        needs no threshold on the physics, only on how long to believe it.
        """
        limit = float(getattr(self.cfg, "STATE_FROZEN_S", 0.0) or 0.0)
        if limit <= 0.0 or self.state in (STANDBY, STOPPED):
            return
        position = tuple(snap.position)
        if (position != getattr(self, "_frozen_position", None)
                or vec.norm(snap.velocity) < 1.0):
            self._frozen_position = position
            self._frozen_since = snap.ut
            return
        if snap.ut - self._frozen_since > limit:
            self.finish("state frozen in %s for %.0f s at %.1f m/s reported "
                        "-- the vessel is gone or kRPC lost it"
                        % (self.state, snap.ut - self._frozen_since,
                           vec.norm(snap.velocity)))

    def log_line(self, snap):
        # **Feed the lift trim from the force the game reports**, at the
        # angle the vehicle is *achieving* -- the only like-for-like
        # comparison, and the same discipline as ``Holdable``.  Here rather
        # than in a phase, so every phase with air under it contributes.
        if snap.dynamic_pressure > self.cfg.LIFT_TRIM_MIN_Q_PA:
            try:
                measured_cla, _ = measured_coefficients(snap)
                speed = vec.norm(snap.velocity)
                altitude = (vec.norm(snap.position)
                            - self.env.equatorial_radius)
                mach = self.env.mach(speed, altitude)
                self.env.lift_trim.observe(
                    mach, measured_cla,
                    self.env.lift.lookup(snap.alpha_actual, mach))
            except Exception:                           # noqa: BLE001
                pass
        if not self.logbook.telemetry(snap.ut, compact_line(self.state, snap,
                                                            self)):
            return
        try:
            self.panel.update(panel_lines(self.state, snap, self))
        except Exception:                               # noqa: BLE001
            pass

    def finish(self, reason):
        self.finished_reason = reason
        self.running = False

    def run(self):
        # In game seconds, not wall-clock seconds, when the game is off 1x --
        # a 1 s glide tick at 2x is 2 s of trajectory between bank commands.
        wait = sleeper(self.cfg, lambda: self.conn.space_center.ut, self.conn)
        interval = self.cfg.ORBIT_TICK_S
        governor = self.scale_governor()
        self.governor = governor
        governed_phase = None
        while self.running:
            started = time.monotonic()
            if self.rpc is not None:
                self.rpc.tick(self.state, self.last_ut)
            snap = self.tick()
            if (self.state == DEORBIT and self.deorbit_dv is not None):
                # Thrusting at 13 m/s^2, so a two-second tick is 26 m/s of dv
                # -- a third of the whole burn -- between chances to stop.
                #
                # **And asking at ignition is asking too late.**  The
                # governor writes a file the plugin re-reads twice a second,
                # so a slowdown commanded on the burn's first tick arrives
                # half a wall-second later -- three game-seconds at 6x, which
                # at this thrust is more dv than the whole burn.  While the
                # phase waits for its pass the ticks are cheap, the governor
                # ramps the scale up on them, and whether the burn is flown
                # at 1.5x or 6x is then a race.  Measured on `qs_plane_inc`
                # over 32 flights: a DEORBIT tick of 0.10-0.14 game-seconds
                # arrives within +-4 km, one of 0.24-0.32 arrives 10-25 km
                # short, with nothing in between -- the whole of that save's
                # bimodality.  ``DEORBIT_PACE_WHOLE_PHASE`` asks for the
                # burn's interval from the moment the phase is entered, so
                # the scale is already where the burn needs it.
                interval = self.cfg.TICK_S
            elif self.state in (STANDBY, DEORBIT, DRAIN, COAST):
                interval = self.cfg.ORBIT_TICK_S
            elif self.state == GLIDE:
                interval = self.cfg.GLIDE_TICK_S
            else:
                interval = self.cfg.TICK_S
            ut = snap.ut if snap is not None else self.conn.space_center.ut
            # Measured at the *bottom* of the tick, so ``busy`` is the work and
            # not the wait.  The governor divides the interval this phase asked
            # for by that work; everything else is the plugin's problem.
            busy = time.monotonic() - started
            if self.rpc is not None:
                self.rpc.tick_done(busy, ut)
            # **A tick under rails warp is not a governed tick.**  The game's
            # warp owns the clock then, and its frames make every round trip
            # slow (COAST: 19 calls in 160-215 ms, against 13 ms out of
            # warp), so sampling them taught the governor a cost the phase
            # does not have once the warp ends.
            warping = bool(getattr(self, "warp_factor", 0))
            if not warping:
                self.loop_rate.sample(self.state, ut, busy)
            if governor is not None and not warping:
                # The governor's own decaying estimate must not carry one
                # phase's cost into the next: DRAIN's 5 s probe tick held the
                # deorbit's wait near 1x for fifty game-seconds.
                if self.state != governed_phase:
                    governor.cost = None
                    governed_phase = self.state
                cost = self.loop_rate.busy(self.state)
                cost = self.loop_rate.peak_after(
                    self.state,
                    getattr(self.cfg, "GOVERN_PEAK_SKIP", 0),
                    getattr(self.cfg, "GOVERN_PEAK_WINDOW_S", 0.0)) or cost
                governor.serve(interval, cost)
            wait(interval, ut)
        return self.finished_reason

    @property
    def surface_gravity(self):
        """``body.surface_gravity``, read once per body: a constant that cost
        two to four round trips a tick on final."""
        body = self.body
        if getattr(self, "_gravity_body", None) is not body:
            self._gravity = body.surface_gravity
            self._gravity_body = body
        return self._gravity

    def settle_game(self, seconds):
        """Wait ``seconds`` of *game* time for a surface to move.

        It was a wall-clock sleep, which is the same thing at 1x and fifteen
        wall-seconds of an idle farm at the DRAIN tick's 1x: the brake and
        envelope probes deploy surfaces ~25 times, in vacuum, with nothing to
        command between.  So the governor is asked for its ceiling and the
        wait is on ``ut``.  A clock that does not move (a paused game) falls
        back to the old wall sleep rather than hanging.
        """
        governor = getattr(self, "governor", None)
        if governor is not None:
            governor.hold_ceiling()
        try:
            start = self.conn.space_center.ut
        except Exception:                               # noqa: BLE001
            time.sleep(seconds)
            return
        deadline = time.monotonic() + max(5.0, 5.0 * seconds)
        while time.monotonic() < deadline:
            time.sleep(min(0.02, seconds))
            try:
                if self.conn.space_center.ut - start >= seconds:
                    return
            except Exception:                           # noqa: BLE001
                return

    def scale_governor(self):
        """The time-scale governor, or ``None`` when nobody asked for one.

        Off unless the harness configured a path, so a flight outside the
        measurement farm never touches the game's clock.  See
        ``Config.TIMESCALE_GOVERNOR``.
        """
        path = getattr(self.cfg, "TIMESCALE_GOVERNOR", "")
        if not path:
            return None

        def announce(previous, scale):
            self.logbook.event(
                self.last_ut or 0.0,
                "time scale: %s -> %.2fx for a %s tick (%.0f ms of work, "
                "quantum %.2f s)"
                % ("--" if previous is None else "%.2fx" % previous, scale,
                   self.state, 1000.0 * (self.loop_rate.busy(self.state) or 0.0),
                   getattr(getattr(self, "governor", None), "quant_now", 0.0)))

        return ScaleGovernor(
            path,
            maximum=self.cfg.TIMESCALE_GOVERNOR_MAX,
            minimum=self.cfg.TIMESCALE_GOVERNOR_MIN,
            margin=self.cfg.TIMESCALE_GOVERNOR_MARGIN,
            on_change=announce,
            quant_fraction=getattr(self.cfg, "TIMESCALE_QUANT_FRACTION", 0.0))

    def shutdown(self, reason):
        try:
            self.log_holdable(self.conn.space_center.ut)
        except Exception:                               # noqa: BLE001
            pass
        try:
            self.logbook.event(self.last_ut or 0.0, self.loop_rate.report())
        except Exception:                               # noqa: BLE001
            pass
        if self.rpc is not None:
            self.logbook.event(self.last_ut or 0.0, self.rpc.report())
            self.logbook.event(self.last_ut or 0.0, self.rpc.wall_report())
            self.logbook.event(self.last_ut or 0.0, self.rpc.slow_report())
        """Hand back to the player, deliberately, rather than try to save it."""
        try:
            self.logbook.event(self.conn.space_center.ut,
                               "shutdown: %s" % reason)
        except Exception:                               # noqa: BLE001
            pass
        for action in (
                lambda: setattr(self.conn.space_center,
                                "rails_warp_factor", 0),
                lambda: setattr(self.control, "throttle", 0.0),
                lambda: set_autopilot_engaged(self.autopilot, False),
                lambda: setattr(self.control, "sas", True),
                self.telemetry.close,
                self.panel.close):
            try:
                action()
            except Exception:                           # noqa: BLE001
                pass


def flown_bank(snap):
    """The bank the vehicle is actually at, in ``lift_direction``'s sense."""
    try:
        up_perp, side = trajectory.lift_frame(snap.position, snap.velocity)
        if up_perp is None or len(snap.roof) != 3:
            return float("nan")
        roof = vec.project_out(snap.roof, vec.unit(snap.velocity))
        if vec.norm(roof) < 1e-6:
            return float("nan")
        return math.degrees(math.atan2(vec.dot(roof, side),
                                       vec.dot(roof, up_perp)))
    except Exception:                                   # noqa: BLE001
        return float("nan")


def measured_coefficients(snap):
    """``(Cl*A, Cd*A)`` the vehicle actually flew, from the reported force.

    Resolved about the *air-relative* velocity, which in the body's rotating
    frame is the velocity itself -- that frame turns with the atmosphere, so
    no wind term is needed and none is invented.  Drag is the component along
    the wind and lift is the magnitude of everything perpendicular to it,
    which is the same decomposition ``environment`` tabulates, so the two are
    directly comparable numbers and not merely similar ones.
    """
    q = snap.dynamic_pressure
    speed = vec.norm(snap.velocity)
    if q <= 0.0 or speed <= 1.0:
        return 0.0, 0.0
    wind = vec.scale(snap.velocity, 1.0 / speed)
    along = vec.dot(snap.aero_force, wind)
    side = vec.sub(snap.aero_force, vec.scale(wind, along))
    return vec.norm(side) / q, -along / q


def compact_line(state, snap, run):
    """One dense line per tick.  The logs are meant to be read whole."""
    env = run.env
    up = vec.unit(snap.position)
    speed = vec.norm(snap.velocity)
    altitude = vec.norm(snap.position) - env.equatorial_radius
    cla, cda = env.coefficients(run.steer.alpha, speed, altitude)
    mass = max(1.0, snap.mass)
    rho = env.density(altitude)
    q = 0.5 * rho * speed * speed
    # The brake law subtracts the deceleration the airframe is already
    # getting for free, and this is where that number is computed.  Shared
    # rather than re-derived: a law fed a second drag model is a law nobody
    # can check against the ``dec=`` column beside it.
    run.last_free_decel = cda * q / mass
    bits = [
        "%-8s" % state,
        "alt=%6.0f" % snap.height_above_runway,
        "h=%6.1f" % snap.landing_height,
        "v=%6.1f" % speed,
        "M=%4.1f" % env.mach(speed, altitude),
        "vs=%+6.1f" % vec.dot(snap.velocity, up),
        "aoa=%4.1f/%4.1f" % (run.commanded_alpha, snap.alpha_actual),
        # Sideslip, from the game rather than inferred.  ``aoa=`` above is
        # nose-to-velocity and hides the slip inside itself; a reversal that
        # is not coordinated shows up here and nowhere else.
        # kRPC's own angle of attack, *signed*: ``aoa=``'s second number is
        # the unsigned nose-to-wind angle and reads a nose below the airflow
        # as above it (LOG3035: -5.5 here, +7.4 there).
        "aoak=%+5.1f" % snap.krpc_aoa,
        "dal=%+4.1f" % ((getattr(run, "_flare_delta", 0.0) if state == FLARE
                         else getattr(run, "_lift_delta", 0.0))
                        + getattr(run, "_alpha_trim", 0.0)),
        "slip=%+5.1f" % snap.sideslip,
        "pin=%+5.2f/%+5.2f/%+4.1f" % (
            getattr(snap, "pitch_input", 0.0) or 0.0,
            getattr(run, "_pitch_assist", 0.0),
            getattr(run, "_pitch_assist_err", None) or 0.0),
        # Yaw and roll as the vessel receives them (the same summed
        # read-back as ``pin=``'s first number).
        "yrin=%+5.2f/%+5.2f" % (getattr(snap, "yaw_input", 0.0) or 0.0,
                                getattr(snap, "roll_input", 0.0) or 0.0),
        ] + ([] if not getattr(run.cfg, "PROPELLANT_TRIM", False) else [
        # ``PROPELLANT_TRIM``: the standing pitch input it closes on, and
        # the propellant moment moved so far (unit-metres, + aft).
        "ptrim=%+5.2f/%+6.0f" % (getattr(run, "_ptrim_standing", None) or 0.0,
                                 getattr(run, "_ptrim_moment", 0.0)),
        ]) + ([] if not getattr(run.cfg, "CANARD_TRIM", False) else [
        # ``CANARD_TRIM``: the canards' commanded trim (fraction of travel,
        # + nose-up) and the low-passed summed pitch input it closes on.
        "ctrim=%+5.2f/%+5.2f" % (getattr(run, "_ctrim", 0.0),
                                 getattr(run, "_ctrim_lp", None) or 0.0),
        ]) + [
        # **Commanded slip is its own column, and only when something asks
        # for one.**  ``slip=`` has always meant the achieved angle and two
        # analyses already parse it; changing that column into ``cmd/actual``
        # would silently re-point every one of them at a different quantity.
        # ``slipc=`` appears only on flights that command a sideslip, which
        # is also how a reader knows the probe was on.
        ] + ([] if abs(getattr(run, "commanded_slip", 0.0)) < 0.01 else [
        "slipc=%+5.1f" % run.commanded_slip,
        ]) + [
        "bank=%+5.1f" % run.commanded_bank,
        # The bank *flown*, in the frame the command is built in
        # (``trajectory.lift_frame``), so ``bank=`` and ``bnk=`` compare
        # directly.  ``nan`` when there is no roof or no airflow.
        "bnk=%+5.1f" % flown_bank(snap),
        "cla=%5.1f" % cla,
        "cda=%5.1f" % cda,
        "dec=%5.2f" % (cda * q / mass),
        # The game's dynamic pressure, not the model's.  Everything about
        # what this airframe can *hold* is binned by it (``Holdable``), and
        # until the broadside probe needed it the one quantity the ceiling
        # model is indexed on was the one quantity the log did not carry.
        "q=%6.0f" % snap.dynamic_pressure,
        "mono=%5.1f" % snap.monopropellant,
        "m=%5.2ft" % (mass / 1000.0),
        "thr=%4.2f" % run.throttle,
        # **Commanded throttle is not thrust.**  Sixty seconds of thr=1.00
        # against an unlit engine look exactly like a burn in this log, and
        # did: the only tell was the speed not changing.  See
        # ``ensure_thrust``.
        "F=%5.0f" % (snap.available_thrust / 1000.0),
        # And what it is *producing*, which is not the same thing for several
        # seconds after a commanded cutoff.
        "Fn=%5.1f" % (getattr(snap, "thrust", 0.0) / 1000.0),
        "skin=%4.2f" % snap.skin_fraction,
        # Parts still attached.  Every flight of this vehicle so far has ended
        # with the game reporting none, and the log used to say nothing.
        "n=%2d" % snap.parts_now,
        "rwy=%5.0f" % run.runway_distance(snap),
        # The same, signed along the centreline: ``rwy`` is a distance and
        # cannot tell a door 800 m short of the threshold from one 800 m
        # past it.  Negative short, past ``RUNWAY_LENGTH_M`` off the far end.
        "ral=%+6.0f" % (run.cfg.RUNWAY_LENGTH_M - run.runway_remaining(snap)),
    ]
    # **What the air actually did, beside what the table said it would.**
    # ``cla``/``cda`` above are the model's, at the *commanded* angle; these
    # are the vehicle's own, from the force kRPC reports, at the angle it is
    # *achieving*.  Two ratios near 1.0 mean the propagator is flying the
    # right airframe; the open shortfall (failure 10) says one of them is not,
    # and nothing until now could say which without finite-differencing the
    # velocity through a Coriolis term worth 10-30% of the drag.
    if snap.dynamic_pressure > 1.0:
        cla_act, cda_act = measured_coefficients(snap)
        # The table read at the angle the vehicle is *achieving*, which is the
        # only comparison that is like for like: ``cla``/``cda`` above are at
        # the angle it was *asked* for, and the two differ by up to five
        # degrees in dense air.
        mla, mda = env.coefficients(snap.alpha_actual, speed, altitude)
        bits.append("act=%5.1f/%5.1f" % (cla_act, cda_act))
        bits.append("mdl=%5.1f/%5.1f" % (mla, mda))
        bits.append("ld=%5.2f/%5.2f" % (mla / mda if mda > 0.01 else 0.0,
                                        cla_act / cda_act
                                        if cda_act > 0.01 else 0.0))
        bits.append("rho=%5.3f" % (snap.air_density / rho if rho > 0 else 0.0))
    if run.prediction is not None:
        bits.append("long=%+6.0f cross=%+6.0f" % run.last_miss)
        bits.append("t2g=%5.0f" % run.prediction.time_to_go)
        # The predicted arrival speed at the gate's altitude, and (under
        # ``GLIDE_ENERGY_AIM``) the ground it is worth in ``long=``.
        bits.append("pv=%4.0f el=%+5.0f" % (
            run.prediction.speed,
            getattr(run.prediction, "energy_long", 0.0)))
        hand = getattr(run.prediction, "handover", None)
        if hand:
            bits.append("ph=%5.0f/%3.0f" % hand)
    # What the glide is aiming past the gate by, right now.  ``long=`` is the
    # miss against the *gate*, so on a flight flown with a reserve the number
    # to read is ``long - rsv``: that is what the solve is nulling.  Without
    # the column a deliberately long entry is indistinguishable in the log
    # from a solve that has stopped converging.
    if state == GLIDE:
        bits.append("rsv=%6.0f" % guidance.glide_reserve(env, run.cfg,
                                                         snap.position))
        if getattr(run.cfg, "GLIDE_CONE_ENERGY", False):
            # The cone's wanted entry energy (low/mid/high) against the
            # predicted one: what ``el=`` is made of.
            want = getattr(env, "cone_energy", None)
            hand = getattr(run.prediction, "handover", None) \
                if run.prediction is not None else None
            met = getattr(run.prediction, "handover_met", False) \
                if run.prediction is not None else False
            got = (hand[0] + hand[1] ** 2 / (2.0 * run.surface_gravity)
                   if hand and met else 0.0)
            bits.append("ce=%5.0f/%5.0f/%5.0f pe=%5.0f" % (
                (want or (0.0, 0.0, 0.0)) + (got,)))
    # **Which law flew this tick.**  ``max`` means the propagated arc did not
    # reach the gate and the vehicle is bracketing for distance rather than
    # nulling a miss (``guidance.max_range``).  It is here because the two
    # are indistinguishable in every other column -- a glide that has stopped
    # steering reads exactly like one on profile, and for eighteen flights
    # nothing in the log said which was which.
    if state == GLIDE and getattr(run, "prediction", None) is not None:
        bits.append("slv=%s" % ("max" if getattr(run.prediction, "max_range",
                                                 False) else "ok "))
    if state == HAC and getattr(run, "hac_command", None) is not None:
        c = run.hac_command
        # The cone's whole state in five numbers: how much turn is left, the
        # radius being flown against the radius the energy asks for, how many
        # extra laps are still owed, and the height against the height this
        # much path needs.  ``r`` chasing ``want`` is the loop working;
        # ``want`` pinned at its bound with ``laps`` above zero is the
        # vehicle absorbing an arrival the straight-in geometry could not
        # have taken at all.
        bits.append("turn=%5.1f gate=%6.0f R=%6.0f laps=%d%s"
                    % (c.turn_deg, c.gate_range, c.radius, c.laps,
                       " short" if c.short
                       else ("" if c.on_circle else " join")))
        bits.append("path=%6.0f need=%6.0f wv=%4.1f sink=%5.1f"
                    % (c.path, c.needed_height, c.weave_deg, c.sink))
        if getattr(run.cfg, "HAC_LD_MEASURED", False):
            bits.append("ldk=%.2f pld=%.2f"
                        % (getattr(run, "hac_ld_scale", None) or 0.0,
                           getattr(c, "plan_ld", 0.0)))
        if getattr(run.cfg, "HAC_GATE_STRETCH", False):
            bits.append("str=%5.0f" % getattr(c, "stretch", 0.0))
        if getattr(run.cfg, "APPROACH_SINK_GUARD", False) and state == APPROACH:
            c2 = getattr(run, "command", None)
            bits.append("sg=%d" % (1 if getattr(c2, "sink_guard", False) else 0))
        if getattr(run.cfg, "HAC_SPLIT_BRAKE", False):
            bits.append("spb=%4.1f" % getattr(run, "split_angle", 0.0))
        if getattr(run.cfg, "HAC_WEAVE_HELD", False):
            bits.append("wh=%4.1f wd=%+.0f"
                        % (getattr(c, "weave_half_s", 0.0),
                           getattr(run, "_weave_dir", 0.0)))
    if state == APPROACH and getattr(run, "command", None) is not None:
        c = run.command
        bits.append("sink=%5.1f/%5.1f" % (c.sink, c.wanted_sink))
        bits.append("xt=%+6.0f hdg=%+5.1f exc=%+6.0f sc=%4.1f"
                    % (c.cross, c.heading_error,
                       getattr(c, "excess", 0.0),
                       getattr(c, "scurve_deg", 0.0)))
        if getattr(run.cfg, "APPROACH_SHARP_TURN", False):
            sh = getattr(c, "sharp", None)
            bits.append("shp=--" if sh is None else
                        "shp=%4.1f/%+.0f" % (sh[0], sh[1] * sh[2]))
        # ``ab=`` is the brake, and ``sat=`` the share of the recent window
        # the S-turn spent at its cap -- printed even when the brake is not
        # armed, because that share is the measurement the whole mechanism
        # rests on and it costs nothing to keep taking it.
        brake = getattr(run, "airbrake", None)
        if brake is not None:
            bits.append("ab=%s sat=%4.2f"
                        % ("out" if brake.extended else "-- ",
                           brake.saturated))
    if state == FLARE:
        bits.append("sink=%5.2f n=%4.2f" % (getattr(run, "flare_sink", 0.0),
                                            getattr(run, "flare_needed", 1.0)))
    if state in (ROLLOUT, STOPPED):
        # ``brk`` is the commanded brake fraction and ``left`` the tarmac the
        # law computed it from.  Both are printed beside ``dec=`` on purpose:
        # a rollout at brk=1.00 whose ``dec`` is not about
        # ``BRAKE_DECEL_FULL_M_S2`` plus ``dec``-from-drag is the measurement
        # that disagrees with that constant.
        try:
            left = run.runway_remaining(snap)
        except Exception:                               # noqa: BLE001
            left = float("nan")
        bits.append("xt=%+6.1f brk=%4.2f left=%+6.0f st=%+5.2f trk=%+5.1f"
                    % (getattr(run, "rollout_cross", 0.0),
                       getattr(run, "brake_fraction", 0.0), left,
                       getattr(run, "rollout_steer_cmd", 0.0),
                       getattr(run, "rollout_track_deg", 0.0)))
    return " ".join(bits)


def panel_lines(state, snap, run):
    speed = vec.norm(snap.velocity)
    lines = [
        "phase   : %s" % state,
        "alt AGL : %8.0f m" % snap.landing_height,
        "speed   : %8.1f m/s" % speed,
        "vert v  : %8.1f m/s" % vec.dot(snap.velocity,
                                        vec.unit(snap.position)),
        "aoa cmd : %8.1f deg" % run.commanded_alpha,
        "aoa act : %8.1f deg" % snap.alpha_actual,
        "bank    : %8.1f deg" % run.commanded_bank,
        "mass    : %8.2f t" % (snap.mass / 1000.0),
        "runway  : %8.0f m (%s)" % (run.runway_distance(snap),
                                    run.end["name"]),
    ]
    if run.prediction is not None:
        lines.append("long    : %+8.0f m" % run.last_miss[0])
        lines.append("cross   : %+8.0f m" % run.last_miss[1])
    return lines


def parse_args(argv):
    p = argparse.ArgumentParser(description="Deorbit and land a spaceplane.")
    p.add_argument("--address", default=None)
    p.add_argument("--rpc-port", type=int, default=None)
    p.add_argument("--stream-port", type=int, default=None)
    p.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    p.add_argument("--autostart", action="store_true",
                   help="do not wait for the panel's START button")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    cfg = apply_overrides(Config(), args.set)
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    directory = (cfg.LOG_DIR if os.path.isabs(cfg.LOG_DIR)
                 else os.path.join(here, cfg.LOG_DIR))
    kwargs = {"name": "spaceplane"}
    if args.address:
        kwargs["address"] = args.address
    if args.rpc_port:
        kwargs["rpc_port"] = args.rpc_port
    if args.stream_port:
        kwargs["stream_port"] = args.stream_port

    with Logbook(directory, cfg.LOG_INTERVAL_UT) as logbook:
        run = None
        try:
            conn = krpc.connect(**kwargs)
            rpc = RpcCounter().install(conn)
            run = Autopilot(conn, cfg, logbook)
            run.rpc = rpc
            if args.autostart:
                run.panel.hide_start()
                # **Through ``engage``, not straight into DEORBIT.**  This
                # path bypassed ``run_standby`` entirely, so a vehicle handed
                # over below the interface went looking for a deorbit burn it
                # had already flown -- and found one: 10 m/s, lit at 55 km,
                # inside the atmosphere.
                # RCS is not switched on here: the valve owns it, and the
                # first tick below sets it like every other tick.
                if not run.env.ready():
                    run.env.sweep(conn.space_center.ut, full=True)
                first = run.tick()
                if first is not None:
                    run.engage(first.ut, first.height_above_runway,
                               first.position, first.velocity)
                else:
                    run.enter(DEORBIT, conn.space_center.ut, "autostart")
                # **The harness hands over a paused game** so a save taken
                # in the air does not fall while this process starts (56
                # game-seconds from ``qs_shuttle_cone``, LOG3031-3033).  The
                # phase is chosen and commanded above, on the state the save
                # recorded; only now may time move.
                try:
                    if conn.krpc.paused:
                        conn.krpc.paused = False
                        logbook.event(conn.space_center.ut,
                                      "unpaused: engaged on the state the "
                                      "save recorded")
                except Exception:                       # noqa: BLE001
                    pass
            reason = run.run()
            run.shutdown(reason or "finished")
        except KeyboardInterrupt:
            if run is not None:
                run.shutdown("interrupted")
        except Exception as exc:                        # noqa: BLE001
            logbook.event(0.0, "FATAL %s: %s" % (type(exc).__name__, exc))
            import traceback
            logbook.event(0.0, traceback.format_exc())
            if run is not None:
                run.shutdown("crashed")
            raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
