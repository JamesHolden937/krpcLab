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
from . import airbrake as airbrake_mod
from . import airframe
from . import guidance, trajectory
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


def attitude_time_to_peak(cfg, vessel, logbook=None, ut=0.0):
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
    if not getattr(cfg, "ATTITUDE_TIME_TO_PEAK_DERIVED", False):
        return fixed
    scale = slew_time_scale(vessel)
    if scale is None:
        if logbook:
            logbook.event(ut, "attitude: no inertia or torque from kRPC -- "
                              "keeping the %.1f s constant" % fixed)
        return fixed
    k = float(cfg.ATTITUDE_SLEW_FACTOR)
    # ``moment_of_inertia`` is (pitch, roll, yaw); ``time_to_peak`` wants
    # (pitch, yaw, roll).  They are not the same order, and getting it wrong
    # swaps the quickest axis for one of the slowest.
    pitch, roll, yaw = (k * scale[0], k * scale[1], k * scale[2])
    if logbook:
        logbook.event(ut, "attitude: slew sqrt(I/tau) = (pitch %.2f, roll "
                          "%.2f, yaw %.2f) s, x%.2f -> time_to_peak "
                          "(%.1f, %.1f, %.1f) (the constant says %.1f "
                          "on every axis)"
                      % (scale[0], scale[1], scale[2], k,
                         pitch, yaw, roll, fixed))
    return (pitch, yaw, roll)


def live_time_to_peak(cfg, vessel):
    """``ATTITUDE_SLEW_FACTOR * sqrt(I / tau)`` on the torque available *now*.

    **The static derivation is right in vacuum and fifteen times too slow in
    thick air.**  ``slew_time_scale`` reads the wheels, because at STANDBY
    that is all there is.  But the control surfaces are live on both craft
    (docs/spaceplane.md, "Session, 2026-09-23"), and their authority grows
    with q: on the shuttle at 4.7 kPa ``available_control_surface_torque`` is
    **2928 kN m of pitch against the wheels' 15**.  Tuned to the wheels' 22 s
    there, a bank reversal at 30 deg of alpha -- which needs the nose to
    swing in pitch and yaw -- dips the alpha to 9 and moves the prediction
    from +0.4 to +22 km in one roll (LOG2918).

    So derive it from the live total, which includes whatever is actually
    acting: wheels always, surfaces as the air thickens, RCS when it is on.
    The wheels are part of the total, so this is never slower than the
    static figure, and at q = 0 with RCS off it *is* the static figure.

    Returned in ``time_to_peak`` order, (pitch, yaw, roll); ``None`` when
    kRPC cannot answer.
    """
    try:
        inertia = vessel.moment_of_inertia
        torque = vessel.available_torque[0]
    except Exception:                                       # noqa: BLE001
        return None
    k = float(cfg.ATTITUDE_SLEW_FACTOR)
    out = []
    for axis in range(3):
        tau = abs(torque[axis])
        if tau <= 1.0:
            return None
        out.append(k * math.sqrt(max(0.0, inertia[axis]) / tau))
    pitch, roll, yaw = out
    return (pitch, yaw, roll)


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
                              "(pitch %.1f, yaw %.1f, roll %.1f) s "
                              "(kRPC default is 1.0 on every axis)" % axes)
        return True
    except Exception as exc:                                # noqa: BLE001
        if logbook:
            logbook.event(ut, "could not set time_to_peak (%r); kRPC's "
                              "default stands" % exc)
        return False


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
        self.running = True
        self.finished_reason = None
        self.autopilot_engaged = False
        self._engage_warned = False

        self.end = self.env.runway.ends["09"]
        self.steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=0.0)
        self.commanded_alpha = 0.0
        self.commanded_bank = 0.0
        self.commanded_slip = 0.0
        # The yaw axis's own ceiling, learned exactly like alpha's: the
        # probe found a saturation in q (20 deg holds through 5000 Pa,
        # 30 deg only through 4000), and a table would fly the next
        # aircraft on this one's fin.
        self.slip_holdable = trajectory.Holdable(cfg)
        self._slip_held = 0.0
        self._slip_ut = None
        self._slip_side = 0.0    # chosen once, on the first command
        self.alpha_ceiling = cfg.ALPHA_MAX_DEG
        # The learned alpha ceiling, hung on the environment so every
        # propagation sees it without being handed it.  See
        # ``trajectory.Holdable``: it starts knowing nothing and says so.
        self.env.holdable = trajectory.Holdable(cfg)
        self._below_ground_since = None
        self.prediction = None
        # Latched once, at the cone's entry: recomputing it every tick flips
        # the hand as the vehicle crosses the centreline, which turns an
        # alignment turn into a reversal at the worst possible moment.
        self.hac_side = None
        self.hac_command = None
        self.hac_radius = None
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
        self._bank_duty_ut = None
        self._bank_cos_num = None
        self._bank_cos_den = None
        self.brake_fraction = 0.0
        self.last_free_decel = 0.0
        self._brake_wheel_cache = None
        self._reaction_wheels_released = False
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
        self.enable_control_surfaces()
        self.airbrake_pair = self._find_airbrake()
        self.airbrake = airbrake_mod.Brake(cfg)
        self.flap_brake = self._find_brake()
        self.flap_brake_out = False
        self.airbrake_ut = None
        self.gimbals_locked = self._lock_gimbals(ut)
        self._release_nose_brake(ut)
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
        self.state = state
        self.state_since = ut
        self.log_authority(ut)

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
                                         self.logbook, ut)
            tune_autopilot(self.autopilot, peak, self.logbook, ut)
            self._tuned_peak = (tuple(peak) if isinstance(peak, (tuple, list))
                                else (peak, peak, peak))
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
        v = snap.velocity
        if vec.norm(v) < 1.0:
            return
        lift = trajectory.lift_direction(snap.position, v, bank_deg)
        if lift is None:
            return
        vhat = vec.unit(v)
        alpha = math.radians(alpha_deg)
        nose = vec.unit(vec.add(vec.scale(vhat, math.cos(alpha)),
                                vec.scale(lift, math.sin(alpha))))
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
        self.commanded_alpha = alpha_deg
        self.commanded_bank = bank_deg
        self.commanded_slip = slip

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
        if (getattr(self.cfg, "APPROACH_SLIP_FOR_ENERGY", False)
                and self.state == APPROACH):
            return self.slip_for_energy(snap)
        return 0.0

    def slip_for_energy(self, snap):
        """Slip in proportion to the surplus, ramped, under the q ceiling.

        Three properties, each of them paid for by a failure in this file:

        * **Proportional to the surplus, not switched on.**  The surplus is
          ``guidance.approach``'s own ``excess`` -- the same quantity the
          S-turn and the gear already spend -- so when there is none the
          command is zero and the vehicle flies exactly as it does now.
        * **Ramped, never stepped.**  ``SLIP_RATE_DEG_S`` limits how fast the
          command moves, in both directions.  A control law that steps is a
          law no vehicle flies (failure 31), and this one steers the nose.
        * **Under the ceiling the flight measures for itself.**  The probe
          found a saturation in dynamic pressure -- 20 degrees holds through
          5000 Pa, 30 degrees only through 4000 -- so the command is capped
          by ``Holdable``, learned live from what the yaw axis achieves
          against what it was asked for.  A table would fly the next
          aircraft on this one's fin.

        And it goes to zero before the flare: ``AIRBRAKE_STOW_LEAD_S``
        seconds of sink above the door, the same rule the brake uses, so the
        vehicle is straight well before the wheels.  The three probe flights
        that held slip into the ROLLOUT were all destroyed.
        """
        cfg = self.cfg
        command = getattr(self, "command", None)
        if snap is None or command is None:
            return self._slip_ramp(snap, 0.0)
        height = snap.landing_height
        sink = max(0.0, -vec.dot(snap.velocity, vec.unit(snap.position)))
        door = (cfg.FLARE_ALT_M + cfg.FLARE_LEAD_S * sink
                + float(getattr(cfg, "AIRBRAKE_STOW_LEAD_S", 4.0)) * sink)
        if height <= door:
            return self._slip_ramp(snap, 0.0)
        excess = max(0.0, getattr(command, "excess", 0.0))
        wanted = float(cfg.SLIP_ENERGY_KP) * max(
            0.0, excess - float(cfg.SLIP_ENERGY_DEADBAND_M))
        ceiling = float(cfg.SLIP_MAX_DEG)
        learned = self.slip_holdable.limit(snap.dynamic_pressure)
        if learned is not None:
            ceiling = min(ceiling, learned)
        # **The sign is a closed loop on the cross-track, and it has to be.**
        # Two sign rules have now been flown and both were wrong in an
        # instructive way.  Taking it from the commanded bank made it reverse
        # twice per S-turn cycle and halved the delivered slip (LOG2836).
        # Choosing the corrective side *once* and holding it was worse: the
        # side force is not small.
        #
        # Measured over four flights, the slip moves the cross-track **in its
        # own sign at about 50 m per degree** across an approach:
        #
        #     mean slip +6.8 -> cross +372     mean slip -7.1 -> cross -477
        #     mean slip +8.1 -> cross +367     mean slip +9.1 -> cross +411
        #
        # So an open-loop sign is a 400 m lateral kick.  ``logs/LOG2846``
        # started at cross -259, correctly chose a positive slip, and rode it
        # through zero to +301 at the flare door -- destroyed, 227 m off the
        # centreline.  The direction was right and the absence of feedback
        # was fatal.
        #
        # With the sign taken from the cross-track *now*, the same side force
        # becomes a lateral control worth ~50 m/deg -- on a phase whose only
        # other lateral authority, the S-turn, is pinned at its cap for half
        # of every approach.  The deadband keeps it from chattering across
        # the centreline, and the ramp below bounds how fast it reverses.
        #
        # **And the offset *now* is still the wrong quantity to steer on.**
        # Nine flights an arm: the slip arm's door cross-track ran a median
        # 26 m against the defaults' 14, and its two wrecks were its two
        # highest-slip flights, arriving at +151 and +176.  A sign chosen
        # from the present offset is a control that *arrives* at the
        # centreline -- it is still leaning when it gets there, and 50 m per
        # degree carries it straight through.  The capture one level up has
        # never reasoned this way: it asks what rate puts the offset at zero
        # *when the flare starts*.  So does this now -- the side comes from
        # the cross-track predicted at the door, ``cross + rate * time``,
        # with both terms published by the capture itself, so the two
        # lateral authorities are solving the same problem instead of two
        # different ones.  Degrades to the plain offset when the capture is
        # off (the fields are zero) or on the first tick.
        cross = getattr(command, "cross", 0.0)
        if getattr(cfg, "SLIP_CROSS_PREDICT", True):
            cross = (cross + float(getattr(command, "cross_rate", 0.0))
                     * float(getattr(command, "cross_time", 0.0)))
        band = float(getattr(cfg, "SLIP_CROSS_DEADBAND_M", 25.0))
        if cross > band:
            self._slip_side = -1.0
        elif cross < -band:
            self._slip_side = 1.0
        elif self._slip_side == 0.0:
            self._slip_side = -1.0 if cross > 0.0 else 1.0
        side = self._slip_side
        return self._slip_ramp(snap,
                               side * vec.clamp(wanted, 0.0,
                                                max(0.0, ceiling)))

    def _slip_ramp(self, snap, wanted):
        """Move the commanded slip toward ``wanted`` at a bounded rate."""
        rate = float(getattr(self.cfg, "SLIP_RATE_DEG_S", 5.0))
        now = snap.ut if snap is not None else 0.0
        dt = 0.0 if self._slip_ut is None else max(0.0, now - self._slip_ut)
        self._slip_ut = now
        # **No elapsed time, no movement.**  The first version fell back to
        # an unbounded step when ``dt`` was zero -- which is the first tick of
        # every APPROACH, so the law opened with the one thing it exists to
        # prevent: 20 degrees of yaw in a single command.  Caught by the
        # ramp's own test before it flew.
        step = rate * dt
        delta = vec.clamp(wanted - self._slip_held, -step, step)
        self._slip_held += delta
        return self._slip_held

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
        if getattr(self.cfg, "AIM_RUNWAY_TRUE_ALPHA", False):
            speed = vec.norm(snap.velocity)
            sink = -vec.dot(snap.velocity, up)
            if speed > 1.0:
                descent = math.degrees(math.asin(
                    vec.clamp(sink / speed, -1.0, 1.0)))
                pitch_deg = min(alpha_deg + max(0.0, descent),
                                self.tail_limit_deg())
        alpha = math.radians(pitch_deg)
        nose = vec.unit(vec.add(vec.scale(along, math.cos(alpha)),
                                vec.scale(up, math.sin(alpha))))
        set_autopilot_attitude(self.autopilot, nose, up)
        self.commanded_nose = nose
        self.commanded_alpha = alpha_deg
        self.commanded_bank = 0.0

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
                self.entry_mass(snap), self.cfg, self.end,
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

    def cone_affordable(self, snap, height):
        """Can the cone the vehicle would fly now be paid for from here?

        **The same expression the cone exits on.**  ``guidance.hac`` already
        computes ``needed`` -- the gate's altitude plus the circling path
        still to fly over the ratio a turning descent achieves -- and
        ``run_hac`` leaves when the height stops covering it.  Asking the
        same question before entering costs one closed-form solve and makes
        the two ends of the phase agree about what a flyable cone is.  They
        did not, and the disagreement is measurable: over 57 flights
        (``conesum.py``, logs/LOG26*-LOG274*), split by whether the planned
        path was affordable at entry --

            affordable      21 rolled out,  3 out of height
            not affordable   9 rolled out, 24 out of height

        -- and the unaffordable entries are the ones ``HAC_ALT_M`` fires,
        at a mean entry height of 11359 against 12992 for the ones the
        distance backstop fires.

        ``None`` when the geometry cannot answer, so the caller falls back
        to the altitude and distance tests rather than reading a failure as
        a refusal.
        """
        try:
            side = guidance.hac_side(self.env, self.cfg, self.end,
                                     snap.position, snap.velocity)
            command = guidance.hac(self.env, self.cfg, self.end,
                                   snap.position, snap.velocity, snap.mass,
                                   self.body.surface_gravity, height, side)
        except Exception:                               # noqa: BLE001
            return None
        if command is None or getattr(command, "needed", None) is None:
            return None
        return height >= command.needed

    def entry_mass(self, snap):
        """The mass the entry will be flown at, not the one aboard now.

        The drain is 2.44 t of 9.13 -- 27% of the vehicle -- and it happens
        *after* the burn.  Predicting the entry at the wet mass is predicting
        a trajectory the vehicle never flies, and it errs in the dangerous
        direction: the same ``Cd*A`` over less mass is more deceleration, so
        the real entry lands *shorter* than the burn aimed for.  Measured in
        game, a burn that exited 46 m from its 50 km long aim reached the
        glide already 38 km **short**, and arrived 68 km short of the runway.

        This is the rule the drain's own phase exists for -- "every prediction
        after this point is made at the drained mass" -- applied one phase
        earlier, where it had been missed.
        """
        if not self.cfg.DRAIN:
            return snap.mass
        if getattr(self.cfg, "DRAIN_BEFORE_BURN", False):
            # **With the valve moved upstream there is no drain still to
            # come.**  The reserve the burn was left is kept for the entry
            # (see ``DRAIN_BEFORE_BURN``), so subtracting it here would
            # predict a vehicle 2-3% lighter than the one that flies -- the
            # same error this method exists to fix, with its sign reversed.
            return snap.mass
        aboard = snap.liquid_fuel + snap.oxidizer
        drained = snap.mass - aboard * float(self.cfg.RESOURCE_KG_PER_UNIT)
        return max(0.1 * snap.mass, drained)

    def ratchet_alpha(self, snap):
        """Lower the ceiling when the vehicle cannot hold what it is given.

        The highest holdable angle of attack is not computable: kRPC's
        aerodynamic probe returns a force and not a moment, so the pitching
        moment at an attitude the vessel is not in is unavailable at any
        price.  What *is* available is whether the vehicle is achieving the
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
                and self.alpha_ceiling < self.cfg.ALPHA_MAX_DEG):
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
                self.cfg.ALPHA_MAX_DEG,
                self.alpha_ceiling + self.cfg.ALPHA_RECOVER_DEG_S * dt)
            return
        if (error > self.cfg.ALPHA_TRACK_TOLERANCE_DEG
                and getattr(self.cfg, "RATCHET_SKIP_REVERSAL", False)
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
            learned = self.env.holdable.limit(snap.dynamic_pressure)
            if learned is None:
                floor = self.cfg.GLIDE_ALPHA_DEG
            else:
                floor = max(self.cfg.ALPHA_CEILING_FLOOR_DEG, learned)
            new = max(floor,
                      self.alpha_ceiling - self.cfg.ALPHA_BACKOFF_DEG)
            if new < self.alpha_ceiling - 0.01:
                self.alpha_ceiling = new
                learned = self.env.holdable.limit(snap.dynamic_pressure)
                self.logbook.event(
                    snap.ut, "alpha ceiling -> %.1f deg (commanded %.1f, "
                             "achieving %.1f, q=%.0f Pa, learned %s)"
                    % (new, self.commanded_alpha, snap.alpha_actual,
                       snap.dynamic_pressure,
                       "%.1f" % learned if learned is not None else "-"))

    def miss(self, snap):
        if self.prediction is None or not self.prediction.reached:
            return None
        gate = self.env.runway.gate(self.end)
        return trajectory.miss_components(self.env, self.end,
                                          self.prediction.position, gate)

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
        if not getattr(self.cfg, "BRAKE_FOR_DISTANCE", False):
            self.control.brakes = speed < self.cfg.BRAKE_SPEED_M_S
            self.brake_fraction = 1.0 if self.control.brakes else 0.0
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
                wheel.brakes = 100.0 * fraction
            except Exception:                           # noqa: BLE001
                continue

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
        if not getattr(self.cfg, "ROLLOUT_REACTION_WHEELS_OFF", False):
            return
        try:
            wheels = list(self.vessel.parts.reaction_wheels)
        except Exception:                               # noqa: BLE001
            return
        stopped = 0
        for wheel in wheels:
            try:
                if wheel.active:
                    wheel.active = False
                    stopped += 1
            except Exception:                           # noqa: BLE001
                continue
        if stopped:
            self.logbook.event(ut, "reaction wheels off for the rollout (%d "
                                   "of %d) -- the gear owns the attitude now"
                               % (stopped, len(wheels)))

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
        if not self.cfg.NOSE_BRAKE_OFF:
            return
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

    def enable_control_surfaces(self):
        """Turn on control axes the craft file left disabled.

        **A no-op on both craft on disk, and the premise below is false.**
        Every surface on ``qs_plane`` and ``qs_shuttle`` already has pitch,
        yaw and roll enabled; the claim that the old craft had "0 of 6" came
        from reading KSP's ``ignorePitch`` flag as if it were an enable, and
        from ``available_torque`` on the pad at q=0.  Kept because a craft
        that really does ship with an axis off is still worth configuring,
        and the log line says how many it changed (on these two: none).

        **The authority that is not merely uncommanded but switched off.**
        The craft this autopilot grew up on carries six control surfaces with
        ``pitch_enabled``, ``yaw_enabled`` and ``roll_enabled`` all False --
        it flies the whole entry on 15 kN m of reaction wheel with its
        aerodynamic surfaces acting as fixed fins.  That one fact is behind
        most of this project's attitude history: the alpha ceiling collapsing
        at about 900 Pa (a wheel is a *constant* torque against a moment that
        grows with ``q``, so it always loses eventually), RCS buying three
        kilometres for an entire tank, and ``ALPHA_TRACKING`` delivering 0.74
        of what is commanded.

        The autopilot already configures the vehicle it is handed -- it locks
        engine gimbals so it can engage, disables the nose brake, sets deploy
        angles, drives the drain valves.  This is the same act.

        **It changes the plant, and every number measured on the old plant
        with it** (failure 23): the stall, ``ALPHA_TRACKING``,
        ``HOLDABLE_PROBE``, ``MARGIN``, the attitude tune.  So it is off by
        default, it is flown as an arm, and it is read with ``oscsum.py`` and
        ``alphaceiling.py`` rather than assumed to be an improvement.

        **And it opens a hole in the derived attitude tune.**
        ``available_torque`` reports reaction wheels and RCS and *never*
        aerodynamic surfaces, so ``slew_time_scale`` still sees a wheels-only
        vehicle and asks for a tune far slower than one with working elevons
        needs.  Flying this with ``ATTITUDE_TIME_TO_PEAK_DERIVED`` is
        therefore two changes, not one.
        """
        if not getattr(self.cfg, "ENABLE_CONTROL_SURFACES", False):
            return 0
        changed, total = 0, 0
        try:
            surfaces = list(self.vessel.parts.control_surfaces)
        except Exception:                               # noqa: BLE001
            self.logbook.event(0.0, "control surfaces: none to enable")
            return 0
        for cs in surfaces:
            total += 1
            was = []
            try:
                for axis in ("pitch", "yaw", "roll"):
                    if not getattr(cs, axis + "_enabled"):
                        setattr(cs, axis + "_enabled", True)
                        was.append(axis)
            except Exception:                           # noqa: BLE001
                continue
            if was:
                changed += 1
                self.logbook.event(0.0, "control surface: %s -- enabled %s"
                                   % (cs.part.title, ", ".join(was)))
        self.logbook.event(
            0.0, "control surfaces: %d of %d had axes switched on "
                 "(the plant is now different from every constant measured "
                 "on it)" % (changed, total))
        return changed

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
        if not (getattr(self.cfg, "AIRBRAKE_OPPOSED_FLAPS", False)
                or float(getattr(self.cfg, "FLAP_BRAKE_PROBE_DEG", 0.0))):
            return None
        records = self._surface_records()
        if records is None:
            return None
        # ``vessel.reference_frame``'s origin *is* the centre of mass, so
        # the positions already carry the arms and the split is at zero.
        armed = airbrake_mod.find_all_brakes(records, 0.0, self.cfg)
        self.logbook.event(0.0, armed.describe())
        return armed if armed.any else None

    def _find_flap_brake_unused(self):
        """Canards against elevons: the opposed-flap brake, armed by geometry.

        See ``airbrake.find_opposed_flaps``.  The forward group deflects by
        ``ratio`` times the aft group's angle so the pitching moments cancel,
        and what is left is drag and a *spoiled* lift -- which is the
        currency the split rudder got wrong and sideslip got right.

        Arms once, logs either way, and refuses rather than guesses.
        """
        if not (getattr(self.cfg, "AIRBRAKE_OPPOSED_FLAPS", False)
                or float(getattr(self.cfg, "FLAP_BRAKE_PROBE_DEG", 0.0))):
            return None
        records = self._surface_records()
        if records is None:
            return None
        # ``vessel.reference_frame``'s origin *is* the centre of mass, so
        # the positions already carry the arms and the split is at zero.
        com_y = 0.0
        forward, aft, ratio, reason = airbrake_mod.find_opposed_flaps(
            records, com_y, self.cfg)
        self.logbook.event(0.0, reason)
        if forward is None:
            return None
        return (forward, aft, ratio)

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

    def _find_airbrake(self):
        """The split-rudder pair, found by geometry, and logged either way.

        Runs once, at construction, because arming a brake is a statement
        about the *vehicle* and not about a phase.  ``spaceplane.airbrake``
        holds the rule and the reasons; everything here is the adapter that
        turns kRPC parts into the plain records it takes, and it is wrapped
        because a craft with no control surfaces at all must fly exactly as
        it does now rather than raise on the pad.

        **It logs the refusal too.**  A brake that silently does not arm is
        indistinguishable in the logs from a brake that armed and did
        nothing, and those two want opposite next steps.
        """
        if not getattr(self.cfg, "AIRBRAKE_SPLIT_RUDDER", False):
            return None
        records = self._surface_records()
        if records is None:
            return None
        pair, reason = airbrake_mod.find_split_rudder(records, self.cfg)
        self.logbook.event(0.0, reason)
        if pair is not None:
            self._set_airbrake_angle(pair)
        return pair

    def _set_airbrake_angle(self, pair):
        """Ask for ``AIRBRAKE_DEPLOY_ANGLE_DEG``, and say whether it took.

        The deploy angle is a *module field* -- there is no typed kRPC
        accessor -- so it goes through the generic interface, and the parts'
        own default is 20 degrees, which is what the constant is set to: on
        this craft the call is meant to change nothing and exists so the
        angle is a configured quantity rather than an editor setting nobody
        can see.  **The log says which happened**, because a constant whose
        only consumer is a hand-set field in the VAB is exactly the
        uncontradictable measurement of failure 13.
        """
        wanted = float(getattr(self.cfg, "AIRBRAKE_DEPLOY_ANGLE_DEG", 20.0))
        done = 0
        for surface in pair:
            try:
                for module in surface.key.part.modules:
                    if "ControlSurface" not in module.name:
                        continue
                    if not module.has_field("Deploy Angle"):
                        continue
                    module.set_field_float("Deploy Angle", wanted)
                    done += 1
            except Exception:                           # noqa: BLE001
                pass
        self.logbook.event(0.0, "airbrake deploy angle %.0f deg set on %d of 2"
                           % (wanted, done))

    def _set_airbrake(self, wanted, snap):
        """Deploy the halves in opposite directions, or stow them.

        ``inverted`` is what makes it a *brake* rather than a rudder: the two
        halves deploy to opposite sides, their side forces act at equal and
        opposite arms, and the yaw and the roll cancel.  Setting it every
        time rather than once at arming is deliberate -- a vehicle that
        reverted to a stock module mid-flight, or a part that was already
        inverted in the editor, would otherwise give a one-sided deflection
        and this autopilot has torn the gear off a vehicle over exactly that
        (failure 34).
        """
        if self.airbrake_pair is None:
            return
        left, right = self.airbrake_pair
        for surface, inverted in ((left, False), (right, True)):
            try:
                surface.key.inverted = inverted
                surface.key.deployed = bool(wanted)
            except Exception as exc:                    # noqa: BLE001
                self.logbook.event(snap.ut, "airbrake: cannot deploy %s (%s)"
                                   % (surface.title, type(exc).__name__))
                self.airbrake_pair = None
                return
        if wanted:
            self._report_airbrake_split(snap)

    def _report_airbrake_split(self, snap):
        """Once, on the first deployment: what the vehicle did about it.

        The geometry says a mirrored pair deployed oppositely produces no net
        yaw or roll, and ``inverted`` is how that opposition is asked for --
        but whether KSP's deploy *direction* on this particular pair obeys it
        is a property of the craft file, not of the argument.

        **The first version of this check measured the wrong quantity, and
        the flight said so.**  It logged each half's ``available_torque`` and
        their sum, expecting a sum near zero -- but ``available_torque`` is
        the torque a surface *could* produce, reported as positive and
        negative magnitudes, so two halves can never cancel in it however
        perfectly they oppose.  ``deflection`` was no better: with every axis
        disabled there is no control-input deflection to read, and it sat at
        +0.00 through a deployment that demonstrably happened (LOG2815).
        Both agreed with themselves and neither could ever have disagreed
        with the mechanism, which is failure 13's shape exactly.

        So this records what a net yaw would actually *do*: the sideslip and
        the yaw rate at the moment the brake comes out.  The comparison that
        settles it is in the telemetry column beside ``ab=`` -- ``slip=``
        with the brake out against ``slip=`` with it in, over a batch -- and
        a clean split leaves that pair indistinguishable.
        """
        if getattr(self, "_airbrake_split_reported", False):
            return
        self._airbrake_split_reported = True
        try:
            yaw_rate = vec.norm(self.vessel.angular_velocity(
                self.vessel.orbital_reference_frame))
        except Exception:                               # noqa: BLE001
            yaw_rate = float("nan")
        self.logbook.event(snap.ut,
                           "airbrake split: slip=%+.2f deg, body rate=%.3f "
                           "rad/s -- compare slip= with ab=out against ab=in"
                           % (snap.sideslip, yaw_rate))

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
            gravity=self.body.surface_gravity)
        # **The window is kept even when there is no brake to command**, so
        # that ``sat=`` is measured on every flight and a craft that refused
        # to arm still says how saturated its S-turn was.  The state is
        # rolled back rather than never computed, because the policy and its
        # thresholds are then exercised identically on both paths.
        # **The opposed flaps are a brake this law drives too.**  They were
        # armed by ``AIRBRAKE_OPPOSED_FLAPS`` and then never deployed by
        # anything but the probe: this method returned on "no split-rudder
        # pair", so the flag armed a brake and flew the committed vehicle --
        # a disconnected knob, found before its batch rather than after.
        flaps = (getattr(self.cfg, "AIRBRAKE_OPPOSED_FLAPS", False)
                 and getattr(self, "flap_brake", None) is not None)
        if self.airbrake_pair is None and not flaps:
            self.airbrake.extended = previous
            return
        if wanted == previous:
            return
        if self.airbrake_pair is not None:
            self._set_airbrake(wanted, snap)
        if flaps:
            self.set_flap_brake(wanted)
        self.logbook.event(snap.ut, "airbrake %s at %.0f m: %s"
                           % ("out" if wanted else "in", height,
                              self.airbrake.last_reason))

    # -- phases ------------------------------------------------------------
    def set_rcs(self, permitted, snap=None):
        """RCS on only while something is actually turning the vehicle.

        The phase says where thrusters are *allowed*; ``rcs.Valve`` decides
        when they fire, from the angle between the commanded nose and the
        real one.  Permission alone still pays for the hunt, because the hunt
        happens inside the phase that needed the slew: the flip costs a few
        seconds of thruster and the settling afterwards costs more than the
        flip.  See ``boosterland.rcs``.
        """
        self.rcs.update(snap.ut if snap is not None else 0.0, permitted,
                        self.pointing_error(snap),
                        None if snap is None else snap.dynamic_pressure,
                        apply=self._apply_rcs)
        self.rcs_on = self.rcs.on

    def _apply_rcs(self, wanted):
        try:
            self.control.rcs = wanted
        except Exception:                               # noqa: BLE001
            pass

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
        if height <= self.cfg.ENTRY_INTERFACE_M:
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
                reach = (self.cfg.GATE_DIST_M + self.cfg.TOUCHDOWN_AIM_M
                         + self.cfg.GATE_ALT_M
                         * airframe.approach_ld(
                             self.env, self.cfg, self.cfg.GATE_ALT_M,
                             self.vessel.mass, self.body.surface_gravity))
                try:
                    out = trajectory.surface_distance(
                        self.env, position, self.end["threshold"])
                except Exception:                       # noqa: BLE001
                    out = None
                sink = -vec.dot(velocity, vec.unit(position))
                trigger = (self.cfg.FLARE_ALT_M
                           + self.cfg.FLARE_LEAD_S * max(0.0, sink))
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
        if getattr(self.cfg, "DRAIN_BEFORE_BURN", False) and self.cfg.DRAIN:
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
        mass = snap.mass if landing else self.entry_mass(snap)
        try:
            measured = airframe.measure(self.env, mass,
                                        self.body.surface_gravity)
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
        self.env.best_ld = measured.best_ld
        self.env.stall_alpha = measured.stall_alpha
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
                ("STALL_SPEED_M_S", self.cfg.STALL_SPEED_M_S,
                 measured.stall_speed),
                ("APPROACH_BEST_LD", self.cfg.APPROACH_BEST_LD,
                 measured.best_ld)):
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
                                             self.entry_mass(snap),
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
        if getattr(self.cfg, "ENTRY_MAX_DRAG", False):
            # **Where the entry stops braking, solved with the burn.**  The
            # broadside entry has no lift, so it has neither range authority
            # nor steering; the switch speed is what it has instead, and it
            # has to be known *now* because the arc it produces is the arc
            # this burn was chosen against.  Hung on ``env`` for the same
            # reason ``holdable`` and ``spending`` are: every propagation
            # from here on has to fly the same law the vehicle does, and a
            # limit nobody passed on is a limit nobody flies.
            switch = guidance.drag_switch_for(
                self.env, snap.position,
                vec.add(snap.velocity,
                        vec.scale(vec.unit(snap.velocity), -dv)),
                self.entry_mass(snap), self.cfg, self.end,
                self.env.runway.gate(self.end), reach)
            self.env.drag_until = switch
            self.logbook.event(
                snap.ut, "drag switch: %s (%.0f km to run; the entry is flown "
                         "for drag above that speed and for lift below)"
                % ("unsolved -- the entry flies for lift" if switch is None
                   else "broadside until %.0f m/s" % switch, reach / 1000.0))
        if self.cfg.DEORBIT_AUTHORITY_WINDOW:
            self.deorbit_aim_m = guidance.deorbit_chosen_aim(
                self.env, snap.position, snap.velocity,
                self.entry_mass(snap), self.cfg, self.end, dv)
        # The aim is no longer one number -- it is a share of the entry each
        # candidate flies (``guidance.deorbit_aim``) -- so log the rule, not
        # ``DEORBIT_LONG_BIAS_M``, which is only its floor and would read as
        # "aiming 40 km" on a flight aimed at three hundred.
        if self.cfg.DEORBIT_AUTHORITY_WINDOW:
            # **What the glide can still reach, and where in it this burn
            # sits.**  Without this the window is a number nobody can check:
            # ``DEORBIT_WINDOW_BIAS`` is a share of a width, and a share of
            # an unlogged width is not a measurement.
            window = guidance.deorbit_window(
                self.env, snap.position,
                vec.add(snap.velocity,
                        vec.scale(vec.unit(snap.velocity), -dv)),
                self.entry_mass(snap), self.cfg, self.end,
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
            self.env, snap.position, snap.velocity, self.entry_mass(snap),
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
            self.shutdown_engines()
            self.restore_thrust_limits()
            self.cutoff_state = (snap.ut, snap.position, snap.velocity,
                                 snap.mass)
            self.log_deorbit_window(snap)
            self.enter(COAST if (getattr(self.cfg, "DRAIN_BEFORE_BURN", False)
                                 and self.cfg.DRAIN) else DRAIN, snap.ut,
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
            self.enter(COAST if (getattr(self.cfg, "DRAIN_BEFORE_BURN", False)
                                 and self.cfg.DRAIN) else DRAIN, snap.ut,
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
            if getattr(self.cfg, "DEORBIT_FLOOR_ON_TICK", False):
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
        pre = (getattr(self.cfg, "DRAIN_BEFORE_BURN", False)
               and self.deorbit_dv is None)
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

    def shutdown_engines(self):
        """Stop the engine, rather than only asking it for nothing.

        **A commanded throttle of zero is not a thrust of zero, and the
        difference is the whole handover error.**  Measured across logs
        2090-2101, the vehicle keeps losing energy after ``DEORBIT -> DRAIN``
        -- an equivalent 2.3 m/s of extra retrograde dv on ``qs_plane`` and
        8.9 on ``qs_plane_inc``, in vacuum, tight within each state.  At
        13 m/s^2 that is under a second of full thrust, which is what an
        engine's decay from a commanded cutoff looks like, and it scales the
        way a fixed impulse does: the lighter vehicle gets the larger dv.

        **Flown, and it is a null -- the engine was never the cause.**  With
        ``thrust`` now streamed, ``Fn`` reads **0.0 kN on the first tick
        after cutoff**, and the shutdown arm drifts -9.05 m/s against the
        control's -8.97 and -9.36 on ``qs_plane_inc`` (LOG2111 against
        LOG2109/2110).  The energy goes somewhere else, and the ``cutoff
        drift`` line says when: entirely inside the DRAIN phase, with the
        engine cold.  Kept, off, because it costs nothing and rules the
        engine out by construction on any future vehicle.

        Off by default: see ``DEORBIT_CUTOFF_SHUTDOWN``.
        """
        if not getattr(self.cfg, "DEORBIT_CUTOFF_SHUTDOWN", False):
            return
        stopped = 0
        try:
            engines = list(self.vessel.parts.engines)
        except Exception:                               # noqa: BLE001
            return
        for engine in engines:
            try:
                if engine.active:
                    engine.active = False
                    stopped += 1
            except Exception:                           # noqa: BLE001
                continue
        self.logbook.event(0.0, "engines shut down at cutoff: %d" % stopped)

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

    def max_drag_alpha(self, snap):
        """The hot entry's angle of attack, or ``None`` when it is not on.

        One call into ``trajectory.max_drag_alpha`` so the control loop and
        the propagator cannot drift apart -- the same reason the approach's
        speed cap lives in ``trajectory`` and is called from both.  The
        vehicle is asked for what ``Holdable`` says it holds, so the command
        is never one the airframe has to argue with.
        """
        mach = None
        speed = vec.norm(snap.velocity)
        altitude = vec.norm(snap.position) - self.env.equatorial_radius
        try:
            sound = self.env.speed_of_sound(altitude)
            if sound > 0.0:
                mach = speed / sound
        except Exception:                                   # noqa: BLE001
            mach = None
        return trajectory.max_drag_alpha(
            self.cfg, snap.dynamic_pressure, mach,
            getattr(self.env, "holdable", None), speed,
            getattr(self.env, "drag_until", None))

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
        self.set_rcs(self.cfg.COAST_RCS, snap)
        self.coast_warp(snap)
        self.end = self.env.runway.choose(snap.position, snap.velocity)
        self.env.refresh(snap.ut)
        self.log_boundary_actual(snap)
        alpha = min(self.cfg.ENTRY_ALPHA_DEG, self.alpha_ceiling)
        hot = self.max_drag_alpha(snap)
        if hot is not None:
            # **Broadside before the air arrives.**  The coast has minutes of
            # nothing to do and an attitude established in vacuum costs
            # nothing to hold, where one established in thickening air has to
            # be fought for -- the same argument this phase already makes for
            # 22 degrees, and it is worth more at 90.
            alpha = hot
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
        bank = guidance.bank_toward(snap.position, snap.velocity,
                                    vec.sub(self.env.runway.gate(self.end),
                                            snap.position),
                                    self.cfg.SOLVE_BANK_MIN_DEG)
        self.aim(alpha, bank, snap)
        # Seed the glide's solve from what the coast is actually holding.
        # ``aim`` records the command for the log; ``self.steer`` is what
        # ``run_glide`` inverts around, so leaving it at bank 0 would hand the
        # first solve the very baseline this is here to remove.
        self.steer = Steer(alpha=alpha, bank=bank)
        if snap.height_above_runway <= self.cfg.ENTRY_INTERFACE_M:
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
                                  (altitude - self.cfg.ENTRY_INTERFACE_M)
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
        # experiment needs it (docs/spaceplane.md, "High alpha: what the
        # airframe gives and what it will hold"), and because "RCS cannot
        # help here" should be a measurement rather than an assumption.
        self.set_rcs(self.cfg.GLIDE_RCS, snap)
        self.env.refresh(snap.ut)
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
        self.update_bank_duty(snap)
        bank0 = self.steer.bank
        if self.cfg.BANK_PREDICT_INTENT and self.bank_intent is not None:
            sign = 1.0 if bank0 >= 0.0 else -1.0
            bank0 = sign * max(abs(bank0), abs(self.bank_intent))
        steer, prediction = guidance.solve_glide(
            self.env, snap.position, snap.velocity, snap.mass, self.cfg,
            self.end, alpha0, bank0,
            self.alpha_ceiling)
        self.bank_intent = steer.bank
        if prediction is not None:
            self.prediction = prediction
            self.env.set_profile(prediction.profile)

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
        if getattr(self.cfg, "HOLDABLE_SKIP_REVERSAL", False) and \
                guidance.bank_in_transit(self.cfg, self.steer.bank,
                                         steer.bank, self.bank_side, dt):
            # Rolling through a reversal: what the alpha does now is the
            # roll, not the airframe's ceiling.  See ``ratchet_alpha``.
            self.holdable_quiet_until = snap.ut + self.attitude_settle_s
        hold = self.cfg.SOLVE_HOLD_THROUGH_REVERSAL_DEG
        if self.cfg.SOLVE_HOLD_ON_TRANSIT:
            if guidance.bank_in_transit(self.cfg, self.steer.bank,
                                        steer.bank, self.bank_side, dt):
                alpha = self.steer.alpha
        elif hold > 0.0 and abs(self.steer.bank) < hold:
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
            snap.mass, self.body.surface_gravity)
        alpha = min(alpha, limit)
        # And the floor underneath it, which is the same law: see
        # ``alpha_floor_for_speed``.  The learned ceiling still wins, because
        # it is a plant limit and this is a policy.
        alpha = max(alpha, trajectory.alpha_floor_for_speed(
            self.env, self.cfg, vec.norm(snap.velocity),
            vec.norm(snap.position) - self.env.equatorial_radius,
            snap.mass, self.body.surface_gravity))
        alpha = min(alpha, self.alpha_ceiling)
        hot = self.max_drag_alpha(snap)
        if hot is not None:
            # **The hot entry does not solve for range; it spends energy.**
            # Applied after the ratchet and the speed laws on purpose: this
            # command is already the ceiling the plant was measured at, so
            # there is nothing for ``alpha_ceiling`` to protect against --
            # and ``ALPHA_MAX_DEG``, which seeds that ratchet, is a lift-curve
            # stop that has no business bounding a drag phase.  The rate
            # limiter below still owns how fast the nose gets there.
            alpha = vec.clamp(hot, self.steer.alpha
                              - self.cfg.ALPHA_RATE_DEG_S * dt,
                              self.steer.alpha
                              + self.cfg.ALPHA_RATE_DEG_S * dt)
        # **A reversal has to outlast its own actuator.**  See
        # ``Config.BANK_REVERSAL_DWELL_S``: the relay switches faster than the
        # seventeen seconds a stop-to-stop slew takes, so the vehicle spends a
        # fifth of the entry mid-transit and the solve spends it reading a
        # wings-level prediction.
        wanted = steer.bank
        side = 1.0 if wanted >= 0.0 else -1.0
        if self.bank_side is None:
            self.bank_side, self.bank_reversed_ut = side, snap.ut
        if self.cfg.BANK_REVERSAL_SETTLE:
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
        else:
            # The clock version, kept for the A/B.  **Latch the side, not the
            # tick**: the first attempt tested the dwell against the command
            # every tick, which gates the *slew* rather than the decision, so
            # a reversal got one tick of travel and was refused for the next
            # twenty seconds and could never cross zero.  Flown, that held
            # one lean for the whole entry -- ``long`` inside 30 m from 55 km
            # to 19 km, and 93 km off the centreline.
            slew = (2.0 * self.cfg.BANK_MAX_DEG
                    / max(0.1, self.cfg.BANK_RATE_DEG_S))
            dwell = max(self.cfg.BANK_REVERSAL_DWELL_S,
                        self.cfg.BANK_REVERSAL_DWELL_SLEWS * slew)
            try:
                if self.env.mach(vec.norm(snap.velocity),
                                 vec.norm(snap.position)
                                 - self.env.equatorial_radius) \
                        < self.cfg.SPEED_FLOOR_MACH:
                    dwell = 0.0
            except Exception:                               # noqa: BLE001
                pass
            if dwell > 0.0:
                if (side != self.bank_side
                        and snap.ut - (self.bank_reversed_ut or -1e9) >= dwell):
                    self.bank_side, self.bank_reversed_ut = side, snap.ut
                wanted = self.bank_side * abs(wanted)
        # **Hold a scheduled lean while the prediction is worthless.**  The
        # solve nulls a predicted miss 1500 km away, and above about Mach 4
        # the vehicle has almost no authority over that miss but enormous
        # leverage on its own energy: bank is what sets how fast it comes
        # down, and how fast it comes down sets how much speed the thin air
        # takes.  Measured over sixteen flights from one byte-identical
        # save, the speed at 45 km spans 41 m/s and predicts the arrival at
        # r=+0.97 -- and the *mean bank above 45 km* spans 14.6 to 43.6
        # degrees.  The commanded lean up there is the whole scatter.
        #
        # So the magnitude is scheduled rather than solved until the air is
        # thick enough for the solve to mean something; the *sign* is still
        # the cross-track's, because that costs nothing and the cone absorbs
        # whatever cross-track is left.  A repeatable bias is worth more than
        # an unbiased scatter: one can be nulled with a number, the other
        # cannot be nulled at all.
        if self.cfg.GLIDE_UPPER_HOLD:
            try:
                mach = self.env.mach(vec.norm(snap.velocity),
                                     vec.norm(snap.position)
                                     - self.env.equatorial_radius)
            except Exception:                               # noqa: BLE001
                mach = None
            if mach is not None and mach > self.cfg.GLIDE_UPPER_UNTIL_MACH:
                side = 1.0 if wanted >= 0.0 else -1.0
                wanted = side * self.cfg.GLIDE_UPPER_BANK_DEG
        # **And the lean has to be gone by the gate.**  Bank is how the glide
        # spends surplus range and it is also the one thing the approach
        # cannot inherit: see ``guidance.align_bank_cap`` and
        # ``Config.GLIDE_ALIGN_RANGE_M``.  Applied to the *command*, so the
        # rate limiter below still governs how fast the lean comes off.
        # Not while the cone is flying: the taper exists so a straight-in
        # arrival is not still mid-turn at the gate, and under the cone the
        # arrival *is* a turn -- stripping the lean on the way in would hand
        # the cone a wings-level vehicle with the whole alignment still to
        # make.
        if not self.cfg.HAC_ON:
            cap = guidance.align_bank_cap(self.env, self.cfg, snap.position,
                                          self.env.runway.gate(self.end))
            wanted = vec.clamp(wanted, -cap, cap)
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
                         self.steer.bank - self.cfg.BANK_RATE_DEG_S * dt,
                         self.steer.bank + self.cfg.BANK_RATE_DEG_S * dt)
        self.steer = Steer(alpha=alpha, bank=bank)
        self.aim(alpha, bank, snap)
        self.ratchet_alpha(snap)

        miss = self.miss(snap)
        if miss is not None:
            self.last_miss = miss
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
        if self.cfg.HAC_ON:
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
            derived = guidance.hac_enterable(
                self.cfg, speed, self.body.surface_gravity)
            can_turn = (mach <= self.cfg.HAC_ENTRY_MACH if derived is None
                        else derived)
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
            afford = (self.cone_affordable(snap, height)
                      if getattr(self.cfg, "HAC_ENTRY_AFFORDABLE", False)
                      else None)
            if afford is None:
                reached = (height <= self.cfg.HAC_ALT_M
                           or distance <= self.cfg.HAC_ENTRY_DIST_M)
            else:
                reached = afford or distance <= self.cfg.HAC_ENTRY_DIST_M
            ready = can_turn and reached
        else:
            ready = (height <= self.cfg.GATE_ALT_M
                     or (height <= 1.15 * self.cfg.GATE_ALT_M
                         and distance <= self.cfg.GATE_CAPTURE_M))
        if ready:
            tune_autopilot(self.autopilot,
                           self.cfg.ATTITUDE_TIME_TO_PEAK_FINAL_S,
                           self.logbook, snap.ut)
            released = airframe.alpha_ceiling(self.env, self.cfg)
            if (self.cfg.RELEASE_CEILING_ON_FINAL
                    and self.alpha_ceiling < released):
                self.logbook.event(
                    snap.ut, "alpha ceiling released for the landing: %.1f "
                             "-> %.1f deg (it was learned at entry dynamic "
                             "pressure and the flare is not flown there)"
                    % (self.alpha_ceiling, released))
                self.alpha_ceiling = released
            miss = self.last_miss or (0.0, 0.0)
            if self.cfg.HAC_ON:
                # **Both ends are on the table here**, and the end chosen at
                # the deorbit was chosen on a bearing taken fifteen hundred
                # kilometres out.  Over the field the only question is how
                # much flying is left, and the four combinations of end and
                # hand differ by most of a lap.
                chosen, self.hac_side = guidance.hac_choose(
                    self.env, self.cfg, self.env.runway, snap.position,
                    snap.velocity)
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
            else:
                self.enter(APPROACH, snap.ut,
                           "gate long=%+.0f cross=%+.0f d=%.0f"
                           % (miss[0], miss[1], distance))

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
        self.set_throttle(0.0)
        self.env.refresh(snap.ut)
        height = snap.height_above_runway
        if self.hac_side is None:
            self.hac_side = guidance.hac_side(self.env, self.cfg, self.end,
                                              snap.position, snap.velocity)
        dt = max(0.05, snap.ut - (self._last_hac_ut or snap.ut))
        self._last_hac_ut = snap.ut
        command = guidance.hac(self.env, self.cfg, self.end, snap.position,
                               snap.velocity, snap.mass,
                               self.body.surface_gravity, height,
                               self.hac_side,
                               previous=self.hac_radius,
                               max_step=self.cfg.HAC_RADIUS_RATE_M_S * dt,
                               weave=guidance.weave_sign(
                                   self.cfg,
                                   snap.ut - (self.state_since
                                              or snap.ut)))
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
        alpha = min(command.alpha, self.alpha_ceiling)
        self.steer = Steer(alpha=alpha, bank=command.bank)
        self.aim(alpha, command.bank, snap)
        self.ratchet_alpha(snap)
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
        ready = (command.turn_deg <= self.cfg.HAC_EXIT_TURN_DEG
                 and command.gate_range <= self.cfg.HAC_ROLLOUT_M
                 and command.laps == 0
                 and height <= needed + self.cfg.HAC_EXIT_SURPLUS_M)
        if ready or height <= self.cfg.GATE_ALT_M:
            released = airframe.alpha_ceiling(self.env, self.cfg)
            if (self.cfg.RELEASE_CEILING_ON_FINAL
                    and self.alpha_ceiling < released):
                self.logbook.event(
                    snap.ut, "alpha ceiling released for the landing: %.1f "
                             "-> %.1f deg" % (self.alpha_ceiling, released))
                self.alpha_ceiling = released
            self.enter(APPROACH, snap.ut,
                       "%s turn=%.0f h=%.0f (needed %.0f) gate=%.0f "
                       "(circle %.0f) laps=%d"
                       % ("rolled out" if ready else "out of height",
                          command.turn_deg, height, needed,
                          command.gate_range, command.radius, command.laps))

    def update_bank_duty(self, snap):
        """How much more vertical lift the reversals are really buying.

        Two time-averages over the same window: the ``cos`` of the bank the
        vehicle was *commanded* -- which is rate-limited, so it passes through
        zero on every reversal -- over the ``cos`` of the lean the solve
        *intended* to hold.  Sitting on the stops the two are the same and the
        ratio is 1.0.  Reversing, the numerator is lifted by every slew and
        the ratio is the duty cycle, which is the quantity the propagation
        has always wanted and never had.

        Exponentially weighted rather than a window, so it needs no history
        buffer, and hung on ``env`` for the reason ``holdable`` is: every
        propagation already carries ``env``, and the sixteen ``Steer``
        objects the search builds cannot each remember to pass it.

        Floored at 1.0: a reversal can only ever *add* vertical lift relative
        to holding the lean, so a ratio below one is noise and believing it
        would make the prediction short for no reason.
        """
        if not getattr(self.cfg, "GLIDE_BANK_DUTY_ON", False):
            return
        previous = self._bank_duty_ut
        self._bank_duty_ut = snap.ut
        intent = getattr(self, "bank_wanted", None)
        if intent is None:
            intent = self.steer.bank
        actual = math.cos(math.radians(self.steer.bank))
        meant = math.cos(math.radians(intent))
        if previous is None:
            self._bank_cos_num, self._bank_cos_den = actual, meant
            return
        dt = max(0.0, snap.ut - previous)
        tau = max(1.0, float(self.cfg.GLIDE_BANK_DUTY_TAU_S))
        w = 1.0 - math.exp(-dt / tau)
        self._bank_cos_num += w * (actual - self._bank_cos_num)
        self._bank_cos_den += w * (meant - self._bank_cos_den)
        if self._bank_cos_den > 1e-3:
            self.env.bank_cos_duty = vec.clamp(
                self._bank_cos_num / self._bank_cos_den,
                1.0, float(self.cfg.GLIDE_BANK_DUTY_MAX))

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
        self.set_throttle(0.0)
        self.env.refresh(snap.ut)
        self.report_landing_airframe(snap)
        height = snap.landing_height
        command = guidance.approach(
            self.env, self.cfg, self.end, snap.position, snap.velocity,
            snap.mass, self.body.surface_gravity, height,
            weave=guidance.weave_sign(
                self.cfg, snap.ut - (self.state_since or snap.ut),
                period=self.cfg.APPROACH_SCURVE_PERIOD_S))
        self.command = command
        alpha = min(command.alpha, self.alpha_ceiling)
        self.steer = Steer(alpha=alpha, bank=command.bank)
        self.aim(alpha, command.bank, snap)
        self.landing_gear(snap, height, getattr(command, "excess", None))
        sink = -vec.dot(snap.velocity, vec.unit(snap.position))
        trigger = (self.cfg.FLARE_ALT_M
                   + self.cfg.FLARE_LEAD_S * max(0.0, sink))
        # **The brake is stowed against the flare door this tick computes**,
        # not against a height constant -- the door is the vehicle's own
        # (``FLARE_ALT_M + FLARE_LEAD_S * sink``) and the next aircraft's is
        # somewhere else entirely.  Hence the ordering: the trigger first,
        # then the brake that has to stay clear of it.
        self.command_airbrake(snap, command, height, trigger, sink)
        if height <= trigger:
            self.flare_since = snap.ut
            self.enter(FLARE, snap.ut, "h=%.1f v=%.1f sink=%.1f cross=%+.0f"
                       % (height, command.speed, command.sink, command.cross))
        elif self.touched_down(snap, height):
            self.enter(ROLLOUT, snap.ut, "touchdown without a flare")

    def run_flare(self, snap):
        """The last fifteen metres, which are their own problem."""
        self.set_throttle(0.0)
        height = snap.landing_height
        elapsed = snap.ut - (self.flare_since or snap.ut)
        alpha, sink, needed = guidance.flare(
            self.env, self.cfg, snap.position, snap.velocity, snap.mass,
            self.body.surface_gravity, height, elapsed)
        alpha = min(alpha, self.alpha_ceiling, self.cfg.FLARE_ALPHA_DEG,
                    self.tail_limit_deg())
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
        if height > self.cfg.FLARE_WINGS_LEVEL_M:
            taper = vec.clamp((height - self.cfg.FLARE_WINGS_LEVEL_M)
                              / max(1.0, self.cfg.FLARE_BANK_TAPER_M),
                              0.0, 1.0)
            limit = self.cfg.FLARE_BANK_MAX_DEG * taper
            lateral = guidance.approach(self.env, self.cfg, self.end,
                                        snap.position, snap.velocity,
                                        snap.mass,
                                        self.body.surface_gravity, height)
            bank = vec.clamp(lateral.bank, -limit, limit)
        self.steer = Steer(alpha=alpha, bank=bank)
        # **Below the levelling height the reference is the runway, not the
        # airflow.**  See ``aim_runway``: this is where the crab comes out.
        if height <= self.cfg.FLARE_ALIGN_ALT_M:
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
            self._set_airbrake(False, snap)
            if getattr(self, "flap_brake_out", False) and not float(
                    getattr(self.cfg, "FLAP_BRAKE_PROBE_DEG", 0.0)):
                self.set_flap_brake(False)
            self.airbrake.extended = False
            self.logbook.event(snap.ut, "airbrake in for the flare")
        self.flare_sink = sink
        self.flare_needed = needed
        if self.touched_down(snap, height):
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
        self.set_throttle(0.0)
        speed = vec.norm(snap.velocity)
        self.release_reaction_wheels(snap.ut)
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
        self.aim_runway(guidance.rollout_alpha(
            self.cfg, speed, snap.ut - (self.state_since or snap.ut),
            self.rollout_entry_alpha, env=self.env), snap)
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
        steer_cmd = vec.clamp(-self.cfg.ROLLOUT_STEER_GAIN * cross,
                              -limit, limit)
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
            self.logbook.event(ut, "lift trim, measured (applied=%s): %s"
                               % (bool(self.cfg.LIFT_TRIM_ON), trim))

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
            return
        trigger = self.cfg.GEAR_ALT_M
        if (getattr(self.cfg, "GEAR_FOR_ENERGY", False)
                and excess is not None and excess > 0.0):
            frac = max(0.01, float(self.cfg.GEAR_DRAG_FRACTION))
            trigger = min(float(self.cfg.GEAR_ALT_MAX_M),
                          self.cfg.GEAR_ALT_M + excess / frac)
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
        self.retune_attitude(snap)
        handler(snap)
        # **Wherever the table happens to become ready.**  The first version
        # reported from STANDBY, which ``--autostart`` leaves on the tick
        # before the sweep finishes -- so on every harness flight, which is
        # every flight that gets measured, it never ran at all.  A check that
        # only fires in the configuration nobody uses is not a check.
        if self.airframe is None and self.env.ready():
            self.report_airframe(snap)
        self.check_thermal(snap)
        self.watch_breakup(snap)
        self.destroyed_early(snap)
        self.frozen_early(snap)
        self.log_line(snap)
        return snap

    def retune_attitude(self, snap):
        """``ATTITUDE_TIME_TO_PEAK_LIVE``: follow the torque the air provides.

        See ``live_time_to_peak``.  Once per ``ATTITUDE_RETUNE_S`` of game
        time, and only when an axis has moved by ``ATTITUDE_RETUNE_FRAC`` --
        kRPC re-derives its gains on every assignment, so re-assigning an
        unchanged tune is churn, not control.  Logged when an axis has moved
        by half again since the last line, so the log shows the schedule
        without a line per second.
        """
        if not (getattr(self.cfg, "ATTITUDE_TIME_TO_PEAK_LIVE", False)
                and self.autopilot_engaged):
            return
        if self.state in (STANDBY, ROLLOUT, STOPPED):
            return
        if (self._retune_ut is not None and snap.ut - self._retune_ut
                < float(self.cfg.ATTITUDE_RETUNE_S)):
            return
        self._retune_ut = snap.ut
        want = live_time_to_peak(self.cfg, self.vessel)
        if want is None:
            return
        have = self._tuned_peak
        frac = float(self.cfg.ATTITUDE_RETUNE_FRAC)
        if have is not None and all(
                abs(w - h) <= frac * h for w, h in zip(want, have)):
            return
        try:
            self.autopilot.time_to_peak = want
        except Exception:                               # noqa: BLE001
            return
        self._tuned_peak = want
        self.attitude_settle_s = want[0]
        logged = self._retune_logged
        if logged is None or any(abs(w - h) > 0.5 * h
                                 for w, h in zip(want, logged)):
            self._retune_logged = want
            self.logbook.event(
                snap.ut, "attitude retune at q=%.0f Pa: time_to_peak "
                         "(pitch %.1f, yaw %.1f, roll %.1f) s"
                % ((snap.dynamic_pressure,) + tuple(want)))

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
        wait = sleeper(self.cfg, lambda: self.conn.space_center.ut)
        interval = self.cfg.ORBIT_TICK_S
        governor = self.scale_governor()
        while self.running:
            started = time.monotonic()
            snap = self.tick()
            if self.state == DEORBIT and (
                    self.deorbit_dv is not None
                    or getattr(self.cfg, "DEORBIT_PACE_WHOLE_PHASE", False)):
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
            self.loop_rate.sample(self.state, ut, time.monotonic() - started)
            if governor is not None:
                cost = self.loop_rate.busy(self.state)
                if getattr(self.cfg, "GOVERN_ON_PEAK", False):
                    cost = self.loop_rate.peak(self.state) or cost
                governor.serve(interval, cost)
            wait(interval, ut)
        return self.finished_reason

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
                "time scale: %s -> %.2fx for a %s tick (%.0f ms of work)"
                % ("--" if previous is None else "%.2fx" % previous, scale,
                   self.state, 1000.0 * (self.loop_rate.busy(self.state) or 0.0)))

        return ScaleGovernor(
            path,
            maximum=self.cfg.TIMESCALE_GOVERNOR_MAX,
            minimum=self.cfg.TIMESCALE_GOVERNOR_MIN,
            margin=self.cfg.TIMESCALE_GOVERNOR_MARGIN,
            on_change=announce)

    def shutdown(self, reason):
        try:
            self.log_holdable(self.conn.space_center.ut)
        except Exception:                               # noqa: BLE001
            pass
        try:
            self.logbook.event(self.last_ut or 0.0, self.loop_rate.report())
        except Exception:                               # noqa: BLE001
            pass
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
        "slip=%+5.1f" % snap.sideslip,
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
    # What the glide is aiming past the gate by, right now.  ``long=`` is the
    # miss against the *gate*, so on a flight flown with a reserve the number
    # to read is ``long - rsv``: that is what the solve is nulling.  Without
    # the column a deliberately long entry is indistinguishable in the log
    # from a solve that has stopped converging.
    if state == GLIDE and run.cfg.GLIDE_RESERVE_ON:
        bits.append("rsv=%6.0f" % guidance.glide_reserve(env, run.cfg,
                                                         snap.position))
    # **Which law flew this tick.**  ``max`` means the propagated arc did not
    # reach the gate and the vehicle is bracketing for distance rather than
    # nulling a miss (``guidance.max_range``).  It is here because the two
    # are indistinguishable in every other column -- a glide that has stopped
    # steering reads exactly like one on profile, and for eighteen flights
    # nothing in the log said which was which.
    if state == GLIDE and getattr(run, "prediction", None) is not None:
        bits.append("slv=%s" % ("max" if getattr(run.prediction, "max_range",
                                                 False) else "ok "))
    # **What the propagation believes the reversals are worth.**  1.00 is a
    # vehicle sitting on its lean and a correction that is doing nothing;
    # above it is the extra vertical lift the slews are really buying
    # (``Autoland.update_bank_duty``).  Logged because without it a null on
    # ``GLIDE_BANK_DUTY_ON`` cannot be told from the term never engaging --
    # CLAUDE.md's "a knob that changes nothing may be disconnected, not
    # powerless", and this file has cost the project a wrong headline finding
    # for want of exactly this column before.
    if state == GLIDE and getattr(run.cfg, "GLIDE_BANK_DUTY_ON", False):
        bits.append("duty=%4.2f" % getattr(env, "bank_cos_duty", 1.0))
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
    if state == APPROACH and getattr(run, "command", None) is not None:
        c = run.command
        bits.append("sink=%5.1f/%5.1f" % (c.sink, c.wanted_sink))
        bits.append("xt=%+6.0f hdg=%+5.1f exc=%+6.0f sc=%4.1f"
                    % (c.cross, c.heading_error,
                       getattr(c, "excess", 0.0),
                       getattr(c, "scurve_deg", 0.0)))
        # ``ab=`` is the brake, and ``sat=`` the share of the recent window
        # the S-turn spent at its cap -- printed even when the brake is not
        # armed, because that share is the measurement the whole mechanism
        # rests on and it costs nothing to keep taking it.
        brake = getattr(run, "airbrake", None)
        if brake is not None:
            bits.append("ab=%s sat=%4.2f"
                        % ("out" if brake.extended else
                           ("in " if run.airbrake_pair is not None else "-- "),
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
        bits.append("xt=%+6.1f brk=%4.2f left=%+6.0f"
                    % (getattr(run, "rollout_cross", 0.0),
                       getattr(run, "brake_fraction", 0.0), left))
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
            run = Autopilot(conn, cfg, logbook)
            if args.autostart:
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
