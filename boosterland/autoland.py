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
from .config import Config, apply_overrides
from .environment import Environment
from .gui import ControlPanel, telemetry_lines
from common.logbook import Logbook
from common.pacing import sleeper
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
        logbook.event(ut, "config: %s" % (" ".join(changed) or "(defaults)"))
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
        self.autopilot_engaged = False
        self.correction_align = None    # (ut, align) from the last tick
        self.autopilot_retries = 0
        self.extra_lines = ["press START AUTOLAND"]
        self.gear_deployed = False
        self.steer = None               # the angle of attack the coast flies
        self.corrections = 0
        self.correction_ready_ut = 0.0
        self.correction_start_miss = None   # the miss the running burn began at
        self.correction_retired = False     # a burn backfired; stop trying
        self.boostback_under = 0        # consecutive ticks inside the tolerance
        self.boostback_under_since = None   # ... and when that run started
        self.commanded_throttle = None  # what set_throttle last asked for
        self.throttle_diverged = False
        self.finished = False
        self.exit_reason = None

    # -- helpers -----------------------------------------------------------
    def enter(self, state, ut, note=""):
        self.log.event(ut, "phase %s -> %s %s" % (self.state, state, note))
        self.state = state
        self.phase_start_ut = ut
        self.correction_align = None    # each burn judges its own turn

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
            if self.cfg.FLIP_VIA_VERTICAL:
                # Big turns go over the top rather than nose-down through the
                # airstream; releases itself as the vehicle comes round.
                direction = guidance.flip_waypoint(
                    snap.direction, direction, snap.position,
                    self.cfg.FLIP_VIA_VERTICAL_DEG)
            if self.cfg.ROLL_ALIGN:
                up = guidance.roll_reference(snap.position, snap.velocity,
                                             direction, self.roll_up)
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
        outnumber the ones where it is turning.  See ``common.rcs``.
        """
        self.rcs.update(snap.ut, self.rcs_permitted,
                        self.pointing_error(snap), None,
                        apply=self._apply_rcs)

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
            self.control.set_action_group(self.cfg.STARTUP_ACTION_GROUP, True)
            self.control.sas = False
            self.autopilot.reference_frame = self.frame
            self.aim(snap.direction, snap)   # hold attitude, but stop the roll
            self.engage_autopilot(snap.ut)
            self.gui.set_start_label("RUNNING")
            self.log.event(snap.ut, "start: rcs=%s ag%d on"
                           % (self.cfg.ENABLE_RCS,
                              self.cfg.STARTUP_ACTION_GROUP))
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

    def run_coast(self, snap):
        self.set_throttle(0.0)
        self.env.refresh_drag(snap.ut, snap.position, snap.velocity,
                              snap.rotation, snap.direction)
        self.env.refresh_lift(snap.position, snap.velocity, snap.rotation,
                              snap.direction)
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
        if snap.vertical_speed < 0.0 and height <= needed:
            self.enter(LANDING_BURN, snap.ut,
                       "h=%.0f needed=%.0f miss=%.0f"
                       % (height, needed, distance))
        elif self.correction_worthwhile(snap, prediction, distance):
            self.corrections += 1
            self.correction_start_miss = distance
            self.enter(CORRECTION, snap.ut, "miss=%.0fm" % distance)

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
        if ut - self.phase_start_ut < self.cfg.CORRECTION_ABORT_GRACE_S:
            return False
        return distance > self.correction_start_miss * self.cfg.CORRECTION_ABORT_RATIO

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

    def run_landing_burn(self, snap):
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

    def tick(self):
        snap = self.telemetry.sample(self.env)
        if self.gui.terminate_pressed():
            self.log.event(snap.ut, "TERMINATE pressed in game")
            self.finish("terminated from GUI")
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
        wait = sleeper(self.cfg, lambda: self.conn.space_center.ut)
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
            set_autopilot_engaged(self.autopilot, False)
            self.control.sas = True
        except Exception as exc:                     # game closed / vessel gone
            self.log.event(ut, "shutdown control error: %r" % exc)
        self.log.event(ut, "shutdown: %s" % reason)
        self.telemetry.close()
        self.gui.close()
        try:
            self.conn.close()
        except Exception:
            pass


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
            "ttl=%6.0f cda=%5.2f M=%4.2f aoa=%+5.1f hold=%4.1f cla=%s"
            % (state, snap.height_above_pad, snap.legs_altitude,
               " " if snap.surface_altitude_ok else "?",
               snap.vertical_speed, snap.horizontal_speed, snap.speed,
               snap.throttle, snap.mass / 1000.0, run.pad_distance(snap),
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
               if run.env.lift_slope is not None else "     -"))


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
