"""All tunables for the boostback-and-land script.

Every number the guidance uses lives here so tuning never means editing
control logic.  Override any of them from the command line, e.g.::

    ./run.sh --set BOOSTBACK_TOLERANCE_M=250 --set LOG_INTERVAL_UT=1
"""

import hashlib
from dataclasses import dataclass, fields


@dataclass
class Config:
    # --- Target -----------------------------------------------------------
    PAD_LAT: float = -0.097162          # KSC launchpad
    PAD_LON: float = -74.557679
    PAD_ALT_OFFSET_M: float = 0.0       # aim point above pad surface
    # Aim off the pad to cancel a *measured*, repeatable miss -- see
    # Environment.__init__.  Reporting stays against PAD_LAT/PAD_LON, so the
    # logs keep saying how far from the real pad the booster actually landed.
    #
    # **It is 0 because there is no longer a systematic miss to cancel.**  The
    # 90 m fitted before was the propagator's integration bias wearing a
    # calibration's clothes: a first-order step understated the overshoot by
    # 60-100 m, boostback stopped on that, and the booster flew past by about
    # as much on every entry state.  RK4 removes it at the source (failure
    # 16), and over ten flights of the five saves at bias 0 the east residual
    # is -33, -2, -132, -98, -23, -0, +12, -24, +162, +142, i.e. a mean of
    # +0.4 m.  There is nothing for a constant to cancel; what is left is
    # per-state (qs_hot lands long, qs_steep short) and a constant makes one
    # of them worse for every metre it helps the other.
    #
    # The method still holds if a new vehicle needs one: fly several entry
    # states with these at 0, read the signed E column, and fit the *slope*
    # from two biases rather than adding the residual to one flight.
    AIM_BIAS_NORTH_M: float = 0.0        # no systematic north miss to cancel
    AIM_BIAS_EAST_M: float = 0.0         # nothing left to cancel; see below

    # --- Sequencing -------------------------------------------------------
    SEPARATION_COAST_S: float = 3.0     # drift before the flip, scan disabled
    SEPARATION_CLEARANCE_SCAN: bool = True  # flip when clear, not on a timer
    SEPARATION_MIN_COAST_S: float = 0.0     # never flip sooner than this
    SEPARATION_MAX_COAST_S: float = 5.0     # ... nor wait longer for clearance
    AIRBRAKE_DEPLOY_ANGLE_DEG: float = 60.0  # every airbrake, deployed at START.
                                            # new1-1016 medians, 90 -> 60:
                                            # qs_hot 247 -> 31, heavy 79 -> 63,
                                            # quicksave 79 -> 55.
                                            # Boostback aims on the drag of
                                            # this angle, so it is also where
                                            # the deploy law starts: at the
                                            # top of its range the law can
                                            # only lengthen
    DEPLOY_LAW: bool = True                 # COAST steps the airbrakes' deploy
                                            # angle against the along-track
                                            # miss; see Autoland.run_deploy.
                                            # ov1-1016 (with OVERSHOOT_ONLY):
                                            # heavy 17-386 m, median 74, v
                                            # 328-619; quicksave 8-245 v two
                                            # 2.3-2.7 km splashes in six
    DEPLOY_STEP_S: float = 10.0             # one Newton step per this
    DEPLOY_SETTLE_S: float = 2.0            # re-sweep drag this long after
                                            # a step (the brakes animate)
    DEPLOY_MAX_STEP_DEG: float = 20.0       # rate limit, not a gain
    DEPLOY_MAX_DEG: float = 90.0
    DEPLOY_MIN_DEG: float = 30.0            # 0 doubled the heavy's cross-track
                                            # scatter (dep1-1016)
    DEPLOY_DEADBAND_M: float = 30.0
    DEPLOY_SENS_EPS: float = 0.05           # drag scale of the sensitivity probe
    DEPLOY_PROBE_DEG: float = 10.0          # first step, before k is measured
    DEPLOY_NEWTON_FRACTION: float = 0.7     # of the full Newton step
    DEPLOY_LEARN_MIN_DEG: float = 3.0       # measure k only from a step this big
    DEPLOY_UNSTEERED: bool = True           # the brakes null the miss of the
                                            # *unsteered* prediction: drag
                                            # carries the range, the lift
                                            # only trims what is left
    DEPLOY_LATE: bool = True                # aim long on brakes held at
                                            # DEPLOY_MIN_DEG, then open them to
                                            # DEPLOY_MAX_DEG at the altitude
                                            # that lands on the pad (solved
                                            # on a measured full-brake curve):
                                            # slower at the burn.  See
                                            # Autoland.run_late_deploy
    DEPLOY_LATE_STEP_S: float = 5.0         # re-solve the switch this often
    DEPLOY_LATE_TOL_M: float = 10.0         # ... to this along-track miss
    ENGINE_ALLOCATION: bool = True          # throttle = center engine first,
                                            # then symmetric pairs, each to
                                            # full; see autoland.EngineCluster.
                                            # alloc2-1016, heavy 12 v 12: miss
                                            # median 47 worst 79 v 90/362;
                                            # touchdown 6.7/14 v 4.2/8.7 m/s
    GIMBAL_ROLL_OFF: bool = True            # engine gimbals ignore the roll
                                            # input; see autoland.set_gimbal_roll.
                                            # groll1-1017 heavy 12 v 12: burn
                                            # spin 0 v 258 deg/s, miss median
                                            # 20 v 58 m, touchdown 5.8 v 8.7
    GIMBAL_ROLL_UNTIL_BURN: bool = True     # ... but only from the landing
                                            # burn: boostback rolls on them
    ENGINE_AXIS_TOL_M: float = 0.3          # this near the cluster axis: center
    ENGINE_RING_TOL_M: float = 0.3          # radii this close: one ring
    ENGINE_PAIR_TOL: float = 0.25           # pairing tolerance, of the radius
    ENABLE_RCS: bool = True
    # **Permission is not demand.**  From START the booster is allowed RCS for
    # the whole flight -- every turn it makes is one time cares about -- but
    # allowed is not the same as open.  Most ticks of a five-minute flight the
    # vehicle is already pointed where it was told and the autopilot is merely
    # hunting, and a thruster held open through the hunt pays for every
    # oscillation.  ``common.rcs.Valve`` makes the valve a relay on the
    # angle between the commanded nose and the real one: on above ``ON``, off
    # once inside ``OFF`` for ``SETTLE_S``.  The deadband is what stops the
    # relay switching at the frequency of the oscillation it is damping.
    #
    # Wide enough not to interfere with the phases that steer: the correction
    # burn lights the gimbal when it is more than ``CORRECTION_ALIGN_DEG``
    # (20) off and not coming round, so the thrusters are long since open by
    # the time that test is reached.
    #
    # **And in COAST demand is saturation, not error** -- the shuttle's air
    # rule, in the booster's air phase (the flip, burns and landing keep
    # the error relay, as the shuttle's non-air phases do).  A pointing error says only that the nose is not where it
    # was told; the wheels, the gimbal and the airbrakes close most of those
    # alone, and on a booster whose RCS is a ring of Vernors every second open
    # is landing propellant.  So the valve opens only when a control axis has
    # been pinned at ``RCS_SAT_INPUT`` for ``RCS_SAT_ON_S`` *and* the error
    # has not closed by ``RCS_SAT_CLOSING_FRACTION`` over that time, and
    # shuts once the input has stayed under ``RCS_SAT_RELEASE`` for
    # ``RCS_SETTLE_S`` (or the error relay settles).  A server that will not
    # stream the input leaves the error relay below in charge.  See
    # ``common.rcs``.
    RCS_SAT_INPUT: float = 0.95
    RCS_SAT_ON_S: float = 0.5
    RCS_SAT_CLOSING_FRACTION: float = 0.2
    RCS_SAT_RELEASE: float = 0.6
    RCS_ERROR_ON_DEG: float = 5.0
    RCS_ERROR_OFF_DEG: float = 1.5
    RCS_SETTLE_S: float = 2.0
    # No dynamic-pressure ceiling on this vehicle: the spaceplane shuts its
    # thrusters off in thick air because its control surfaces own the
    # attitude there, and a booster's do not -- the descent is flown on RCS,
    # reaction wheels and the gimbal, which is the whole reason
    # ``CORRECTION_GIMBAL_THROTTLE`` exists.  0 disables the test, and the
    # snapshot carries no dynamic pressure to make it with anyway.
    RCS_Q_MAX_PA: float = 0.0

    # --- Boostback --------------------------------------------------------
    BOOSTBACK_EXIT_TICKS: int = 4           # consecutive ticks under tolerance
    BOOSTBACK_EXIT_S: float = 0.0           # ... and this much *game* time
                                            # under it as well.  Ticks are
                                            # wall-clock paced, so a loaded
                                            # machine makes a tick count a
                                            # different amount of burn; game
                                            # seconds are the same burn on
                                            # any machine.  0 = ticks only.
    BOOSTBACK_TOLERANCE_M: float = 190.0    # stop burning inside this miss
    BOOSTBACK_UNDERSHOOT_M: float = 1e9     # the deepest undershoot the exit
                                            # will accept, and it is off (a
                                            # number no flight can reach) on
                                            # purpose.  The idea was that
                                            # every phase after boostback can
                                            # only shorten the flight, so the
                                            # burn should never stop while the
                                            # prediction is still short of the
                                            # pad.  That reads the geometry
                                            # backwards: boostback approaches
                                            # the pad *from* the undershoot
                                            # side -- long runs from -113 km
                                            # up towards 0 -- so the tolerance
                                            # band is a deliberate lead, and
                                            # the coast then spends it.
                                            # Requiring long >= 0 makes the
                                            # burn eat the whole lead: the
                                            # taper decays with a ~3 s time
                                            # constant, so the last 190 m cost
                                            # 36 s of burn, boostback ended at
                                            # miss=0 far lower and slower, and
                                            # the coast re-opened 500 m the
                                            # corrections could not close.
                                            # Measured in game at bias 0:
                                            # 236-885 m from the pad on every
                                            # save (671/816/885 quicksave,
                                            # 653/853/449 qs_hot), against
                                            # 5-33 m for the same flights with
                                            # this off.  The sim liked it --
                                            # 4/75/19/5 m over the four entry
                                            # states -- which is the trap
                                            # CLAUDE.md's "Measured in game"
                                            # section is about.
    BOOSTBACK_PITCH_DEG: float = 0.0        # aim above local horizon
    BOOSTBACK_TAPER_DV: float = 60.0        # throttle down under this dv left
    BOOSTBACK_MIN_THROTTLE: float = 0.0     # let the taper reach zero.  At 0.05
                                            # the last tick of a burn is not
                                            # small: 0.05 throttle with ~130 s
                                            # of flight left still moves the
                                            # predicted miss over 100 m per
                                            # second of burn, so *which tick*
                                            # ends the burn is worth tens of
                                            # metres.  With no floor the miss
                                            # decays with a ~3 s time constant
                                            # and the burn ends where the
                                            # throttle is already nothing, so
                                            # the exit tick stops mattering.
                                            # Boostback and CORRECTION share
                                            # this law, so it fixes both.
    BOOSTBACK_MAX_BURN_S: float = 180.0     # abort guard, in-game seconds
    BOOSTBACK_HEADING_HOLD: bool = True     # in the taper, hold the aim
                                            # taken at its first tick and burn
                                            # off the miss *along* it, rather
                                            # than re-aiming opposite the miss
                                            # every tick.  Re-aiming swung the
                                            # nose 30 deg sideways in the last
                                            # 2 s (cw=41 deg/s, LOG12455) as
                                            # the along-track part closed and
                                            # the cross-track one took over the
                                            # direction -- the yaw at the end
                                            # of boostback.
    FLIP_ALIGN_DEG: float = 25.0            # hold throttle until aligned
    FLIP_UNDER_POWER: bool = True           # ignore that gate: light, then flip
    # The user's 2026-10-18 proposal: during the flip, burn only the
    # throttle that makes the turn cheapest -- gimbal authority bought with
    # thrust that mostly points the wrong way, against the drift every
    # second of turning adds (guidance.flip_throttle, solved every tick from
    # the live gimbal geometry, RCS and wheel torque).  Off = full throttle
    # through the flip.  ascent/budget.boostback models the same law.
    FLIP_THROTTLE_LAW: bool = False
    FLIP_GIMBAL_SHARE: float = 0.9          # of the geometric gimbal torque
                                            # the attitude loop delivers (91-
                                            # 93% measured, burn_slew_rate)
    BOOSTBACK_CLEAR_NOSE: bool = True       # ... but not while thrust along
                                            # the nose as it points now would
                                            # close on a neighbour

    # --- Separation clearance (when it is safe to flip) --------------------
    CLEARANCE_RANGE_M: float = 500.0        # ignore craft further off than this
    CLEARANCE_MARGIN_M: float = 5.0         # padding on the two bounding spheres
    CLEARANCE_HORIZON_S: float = 10.0       # how far ahead a drifting craft is checked
    CLEARANCE_BURN_HORIZON_S: float = 2.0   # ... and a thrusting one, see below
    CLEARANCE_STEP_S: float = 0.25          # resolution of that check
    CLEARANCE_RESCAN_UT: float = 2.0        # how often the vessel list is re-read
    CLEARANCE_RADIUS_MAX_M: float = 200.0   # bigger than this is a bad box
    CLEARANCE_RADIUS_FALLBACK_M: float = 15.0   # used while a box is unreadable

    # --- Attitude ---------------------------------------------------------
    FLIP_VIA_VERTICAL: bool = True      # big turns through the zenith: pitch
                                        # up in the nose's vertical plane,
                                        # down in the target's
                                        # (guidance.flip_through_vertical)
    FLIP_VIA_VERTICAL_DEG: float = 90.0     # only turns bigger than this
    FLIP_VERTICAL_PAST_DEG: float = 10.0    # waypoint: zenith tilted this far
                                            # toward the target's side
    ROLL_ALIGN: bool = True             # hold roll, roof in the trajectory plane
    ROLL_OFFSET_DEG: float = 0.0        # roof offset from that plane (fin clocking)
    # Roll is held the whole flight (seed1-1017, 12 a cell, all six on
    # against the defaults: heavy roll error boostback/coast/burn median
    # 0/21/26 v 111/95/96 deg, qs_hot burn p90 6 v 147; miss mean 33 v 43
    # heavy, 39 v 56 qs_hot; worst 101 v 115, 80 v 212).
    ROLL_THRESHOLD_DEG: float = 180.0     # kRPC roll_threshold: roll is held only
                                        # while the pointing error is under it
                                        # (5 = kRPC's default).  The heavy's
                                        # coast sat at 6-10 deg, roll input 0,
                                        # and drifted 142 deg off by ignition
                                        # (LOG11353)
    ROLL_TRANSPORT: bool = True         # the roof's side is the last roof
                                        # carried through the nose's turn:
                                        # a flip over the top is pitch only
                                        # (guidance.transport_roof)
    ROLL_SEED_ACTUAL: bool = True       # first roof on the side nearest the
                                        # vehicle's own, not away from the body
    ROLL_PLANE_MIN_HS_MPS: float = 5.0  # below this horizontal speed the roof
                                        # is carried, not re-taken from the
                                        # trajectory plane (0 = never)

    # --- Aerodynamic steering (flying the coast as a wing) -----------------
    AERO_STEER: bool = True                 # steer the coast with an angle of
                                            # attack instead of holding
                                            # retrograde.  The booster falls
                                            # for a minute through air thick
                                            # enough to brake it at over a g,
                                            # with its grid fins deployed, and
                                            # every other correction in the
                                            # flight costs propellant, an
                                            # engine relight and works in one
                                            # direction only -- which is the
                                            # whole of what is left of the
                                            # miss (failure 16).  The lift
                                            # slope is *measured* in flight by
                                            # the same call that measures
                                            # Cd*A, and the angle is *solved*
                                            # against the propagator rather
                                            # than fitted to a gain; see
                                            # guidance.solve_steer.  Off until
                                            # it is measured in game.
    AERO_STEER_CROSS: bool = True           # roll the lift about the velocity
                                            # as well as tilting it, so the
                                            # cross-track miss has an actuator
                                            # too.  Nothing else in the flight
                                            # can move it: CORRECTION tilts off
                                            # retrograde, which is a downrange
                                            # lever, and the landing burn has
                                            # ~10 m of authority.  Costs one
                                            # more propagation a tick.
    AERO_STEER_MAX_ALT_M: float = 25000.0   # ... and hold retrograde *above*
                                            # this, until the drag re-sweep
                                            # has settled the prediction.  At
                                            # the boostback exit the miss
                                            # reads ~-175 m and an unsteered
                                            # coast walks it to zero on its
                                            # own; steering it out there is
                                            # correcting a walk, not a miss
    AERO_STEER_MIN_ALT_M: float = 0.0       # hold retrograde below this.  An
                                            # altitude cliff is the wrong
                                            # guard and it was measured: at
                                            # 2000 it cost `quicksave` 13 m,
                                            # and guidance.verified catches
                                            # the flights it was meant to
                                            # save.  0 = off
    AERO_STEER_DEADBAND_M: float = 2.0      # ... and do not steer against a
                                            # miss already this small
    AERO_STEER_MIN_AUTHORITY_M: float = 1.0  # metres the air must be able to
                                            # move the touchdown point before
                                            # the solve is trusted; under this
                                            # it holds retrograde rather than
                                            # divide by nothing
    AERO_STEER_MAX_AOA_DEG: float = 10.0    # the most the vehicle is asked to
                                            # hold off retrograde.  It is an
                                            # attitude limit, not a control
                                            # gain: a booster that cannot hold
                                            # it will simply not fly it, and
                                            # the loop re-solves every tick.
    AERO_STEER_SOLVE_AOA_DEG: float = 4.0   # the angle the second propagation
                                            # probes, to measure metres of
                                            # miss per radian
    AERO_STEER_PROBE_AOA_DEG: float = 8.0   # the angle the *aerodynamic* probe
                                            # is taken at, to measure Cl*A per
                                            # radian.  Big enough to be well
                                            # clear of the probe's own noise,
                                            # small enough to stay in the
                                            # linear part of the curve.

    AERO_STEER_TRACK_SETTLE_S: float = 10.0  # no tracking sample this soon
                                            # after entering COAST
    AERO_STEER_TRACK_MIN_DEG: float = 1.0   # measure how much of a commanded
                                            # angle the vehicle holds (see
                                            # Environment.lift_slope_at) only
                                            # from commands at least this big.
                                            # Small commands count: the
                                            # original booster held 0.1 of a
                                            # 1.5 deg command, and a lift it
                                            # does not make must not be
                                            # credited at any size
    AERO_STEER_TRACK_STEADY_S: float = 2.0  # ... that have held within
    AERO_STEER_TRACK_STEADY_DEG: float = 1.0  # this much for this long
    AERO_STEER_TRACK_SMOOTHING: float = 0.1  # EMA weight per sample
    AERO_STEER_TRACK_FLOOR: float = 0.05    # a tracking of 0 would read as a
                                            # vehicle with no air at all
    AERO_STEER_HOLDABLE: bool = True        # compute the angle the booster
                                            # *can* hold before asking for it:
                                            # the aerodynamic restoring moment
                                            # (simulate_aerodynamic_torque_at,
                                            # at the vessel as it is now)
                                            # against the control torque.  The
                                            # solve commands no more, and the
                                            # propagator credits lift only up
                                            # to it at each step's q and Mach.
                                            # The tracking measurement stays
                                            # as the thing that disagrees.
    AERO_STEER_HOLD_PROBES: int = 4         # torque probes per refresh, at
                                            # evenly spaced angles up to
                                            # AERO_STEER_MAX_AOA_DEG
    AERO_STEER_HOLD_PROBE_S: float = 2.0    # game seconds between refreshes
    AERO_STEER_HOLD_MARGIN: float = 0.7     # fraction of the control torque
                                            # a steady hold may spend on the
                                            # aerodynamic moment; the rest is
                                            # left for disturbances

    # --- Coast ------------------------------------------------------------
    COAST_STEER_GAIN: float = 0.0006        # degrees of tilt per metre of miss
    COAST_MAX_TILT_DEG: float = 12.0

    # --- Mid-course correction (trims the miss the coast opens up) ---------
    CORRECTION_ENTER_M: float = 1000.0      # start correcting above this miss.
                                            # Only a disturbance the air cannot
                                            # steer out: a correction starts
                                            # from COAST's *steered* miss and
                                            # is judged on an unsteered one, so
                                            # below this every burn backfired
                                            # (noc1-1016: corrections off v 60,
                                            # 6 a cell -- heavy 0 v 3 lost,
                                            # mean 547 v 741; quicksave mean
                                            # 98 v 342, worst 235 v 1088).  At
                                            # 1000 a 25 m/s mid-coast shove is
                                            # still caught in the sim (151-210 m
                                            # v 1094-2886 with none).  Was 60;
                                            # the history below is of that.
                                            # 1200 was set when the predicted
                                            # miss walked by ~1 km through the
                                            # coast (failure 11) -- the gate
                                            # has to sit above the prediction
                                            # noise.  The re-swept drag curve
                                            # removes that walk, so the gate
                                            # can come back down to where the
                                            # miss it is meant to catch lives;
                                            # see CLAUDE.md failure 14.
    CORRECTION_EXIT_M: float = 15.0         # hysteresis: stop below this miss.
                                            # Must stay well under
                                            # CORRECTION_ENTER_M -- at 100
                                            # against a 60 m gate the phase
                                            # enters and exits on the same
                                            # tick, doing nothing and spending
                                            # a relight to do it.
    CORRECTION_SETTLE_S: float = 0.0        # wait this long after the first
                                            # descent re-sweep before
                                            # correcting.  One re-sweep is not
                                            # a converged curve -- failure 14
                                            # is that the *repeats* are the
                                            # mechanism -- so a burn fired two
                                            # seconds after the first one is
                                            # acting on a prediction that is
                                            # still being replaced.  0 = off.
    CORRECTION_MAX_ALT_M: float = 1e9       # ... nor higher than this.  OFF,
                                            # and measured: see below.  It was
                                            # 25000 for one grid, on the
                                            # reasoning that CORRECTION should
                                            # wait for a settled prediction the
                                            # way the wing does, and it is a
                                            # clear regression -- `qs_north`
                                            # 4/5 m became 118/141/107 and
                                            # `qs_cold` 3/3 became 42/58/20.
                                            # CORRECTION's early burns take the
                                            # first bite out of the ~180 m
                                            # boostback lead even though the
                                            # prediction they act on has not
                                            # settled, and the wing cannot
                                            # close that much on its own: the
                                            # regressed flights read coast
                                            # entry -182 -> burn entry -120.
                                            # One backfire in ten flights is
                                            # cheaper than removing the phase.
                                            # ... nor higher than this, for
                                            # the same reason the wing waits:
                                            # up at the boostback exit the
                                            # predicted miss is still the
                                            # drag curve's walk and not a
                                            # miss, and a burn spent on it is
                                            # a burn spent on nothing.  LOG542
                                            # entered at 238 m up there and
                                            # came out at 795 -- the abort
                                            # guard caught it, but 795 m is
                                            # more than the wing can close in
                                            # the descent that is left, and
                                            # it landed 112 m out where its
                                            # sibling flights landed at 3.
                                            # Keep it at or below
                                            # AERO_STEER_MAX_ALT_M
    CORRECTION_MIN_ALT_M: float = 6000.0    # never correct lower than this
    CORRECTION_MIN_TTL_S: float = 15.0      # nor with less flight time left
    CORRECTION_MAX_THROTTLE: float = 0.40
    CORRECTION_ALIGN_DEG: float = 20.0      # full correction thrust inside this
    CORRECTION_GIMBAL_THROTTLE: float = 0.25    # thrust for gimbal when stuck
    CORRECTION_STUCK_RATE_DEG_S: float = 3.0    # turning slower than this is stuck
    CORRECTION_FREE_AIM_ALT_M: float = 0.0  # above this, a correction aims
                                            # like boostback -- opposite the
                                            # horizontal miss -- instead of
                                            # tilting off retrograde, so it
                                            # can lengthen the flight as well
                                            # as shorten it.  **Off**, and the
                                            # reasoning behind it is still
                                            # sound, which is why the knob is
                                            # here.  Everything after
                                            # boostback is one-sided, so an
                                            # undershoot is a miss nothing can
                                            # touch -- qs_steep lands ~130 m
                                            # short and the prediction says so
                                            # at 34 km, in thin air, with fuel
                                            # aboard, while CORRECTION
                                            # declines because miss_gradient
                                            # correctly reports its own
                                            # steering would make things
                                            # worse.  At 25000 it does fire
                                            # and it is worth a little on
                                            # qs_hot (52, 53 m against 67,
                                            # 47), but on quicksave it
                                            # destroys a flight that was
                                            # landing at 6 and 12 m: 142 m
                                            # west and 223 m *east* on the
                                            # same config, which is a burn
                                            # over-correcting through the pad
                                            # and out the other side.  Full
                                            # horizontal authority with
                                            # boostback's dv law is too much
                                            # gain for a 100 m miss; the idea
                                            # needs a law sized for the job
                                            # rather than the gate opened
                                            # wider.  See failure 16.
    CORRECTION_MAX_TILT_DEG: float = 50.0   # steering authority off retrograde
    CORRECTION_TILT_GAIN: float = 0.12      # degrees of tilt per metre of miss
    CORRECTION_MAX_BURN_S: float = 45.0     # per-burn guard
    CORRECTION_COOLDOWN_S: float = 20.0     # settle before judging the miss again
    CORRECTION_OVERSHOOT_ONLY: bool = True   # never enter on a short miss
    CORRECTION_MAX_BURNS: int = 3           # engines have limited relights
    CORRECTION_PROBE_DV: float = 5.0        # dv used to test which way a burn moves
    CORRECTION_ABORT_RATIO: float = 1.3     # abort if the miss grows this much
    CORRECTION_ABORT_GRACE_S: float = 3.0   # ... after this long, so a turn can settle
    CORRECTION_ABORT_FLOOR_M: float = 60.0  # ... and never on less growth than
                                            # this past the best (prediction
                                            # noise; was the old 60 m gate)
    CORRECTION_MIN_GRADIENT: float = 0.5    # metres of miss closed per m/s, minimum

    # --- Landing burn -----------------------------------------------------
    LANDING_THROTTLE_CAP: float = 0.95      # headroom for the throttle loop
    SUICIDE_MARGIN: float = 1.18            # light the engines this much early
    SUICIDE_REACTION_S: float = 1.5         # engine spool-up allowance
    LANDING_RCS_HELD: bool = True           # RCS valve held open through the
                                            # landing burn, not relayed: keeps
                                            # kRPC's auto-tuned gains steady
    COAST_FLIP_RELAY: bool = True           # the coast's flip gets the error
                                            # relay, not the saturation rule:
                                            # kRPC sent no input at 100-145 deg
                                            # of error with the valve shut, and
                                            # the flip was left to the airflow
                                            # (LOG12465).  The relay had opened
                                            # it only by the luck of the end-
                                            # of-boostback swing (LOG12455).
    COAST_RCS_TRIM: bool = True             # translate on RCS in COAST to null
                                            # the predicted cross-track miss:
                                            # nothing else on the heavy moves
                                            # it (its wing reads aoa 0), and
                                            # the -40..-98 m it leaves boostback
                                            # with is its touchdown miss
    COAST_RCS_TRIM_SETTLE_DEG: float = 10.0  # start once the flip is this close
    COAST_RCS_TRIM_START_M: float = 10.0    # start pushing past this ...
    COAST_RCS_TRIM_DEADBAND_M: float = 3.0  # ... stop inside this; the valve
                                            # is held open only while pushing
                                            # (open all coast: 1.6 t, LOG12465)
    COAST_RCS_TRIM_FULL_M: float = 30.0     # full input at this cross-track
    COAST_RCS_TRIM_CHECK_S: float = 4.0     # after this long translating one
    COAST_RCS_TRIM_CHECK_M: float = 15.0    # way, a cross grown by this much
                                            # means the axis sign is wrong:
                                            # flip it once, then give up
    LANDING_DIVERT_LEAD: float = 0.0        # extra trigger height per m of miss
    LANDING_MAX_EARLY_M: float = 1500.0     # cap on that early start
    LANDING_TERMINAL_GUIDANCE: bool = True   # steer the landing burn; see
                                            # guidance.terminal_command
    LANDING_DIVERT_ON_PREDICTION: bool = True    # steer on the propagator's
                                            # predicted touchdown rather than
                                            # on a linear extrapolation of the
                                            # current state; see
                                            # guidance.terminal_command
    LANDING_ZEM_GAIN: float = 6.0           # position term (double-integrator optimal)
    LANDING_ZEV_GAIN: float = 4.0           # velocity term; 6/4 is the energy solution
    LANDING_TGO_MIN_S: float = 2.5          # floor under time-to-go, or gains blow up
    LANDING_NULL_ALT_M: float = 250.0       # below this the steering stops
                                            # aiming and only nulls the
                                            # lateral speed, so the legs
                                            # arrive going straight down
    LANDING_HVEL_TOL: float = 0.3           # lateral speed that counts as nulled
    LANDING_DIVERT_MIN_ALT_M: float = 4.0   # floor under the nulling: stand on
                                            # vertical for the last few m
    LANDING_DIVERT_MAX_ALT_M: float = 2500.0  # nor above this: see terminal_attitude
    LANDING_DIVERT_MIN_ACCEL: float = 1.0   # thrust needed before steering means anything
    LANDING_DIVERT_MIN_THROTTLE: float = 0.0    # 0: see run_landing_burn, it hovers
    LANDING_DIVERT_DEMAND_M_S2: float = 1.0     # lateral demand worth lighting it for
    LANDING_DIVERT_THROTTLE_BUDGET: bool = True  # never tilt into the
                                            # braking: cap the tilt at the
                                            # angle the *spare* throttle can
                                            # pay for.  Only bites when the
                                            # divert is on at all.
    LANDING_MAX_TILT_DEG: float = 15.0
    LANDING_UPRIGHT_ALT_M: float = 50.0     # leg height below which the
                                            # burn's whole command narrows
    LANDING_UPRIGHT_TILT_DEG: float = 2.0   # ... to this at the ground; see
                                            # guidance.terminal_command
    LANDING_SLEW_SHARE: float = 0.25        # s: the burn's command moves at most
                                            # this many seconds of the vehicle's
                                            # angular acceleration per second
                                            # (0 = unslewed); see burn_slew_rate.
                                            # slew2-1017 v unslewed: qs_hot miss
                                            # mean 13 v 30, worst 25 v 66, burn
                                            # err 15 v 50 deg; heavy err 7 v 37,
                                            # worst 90 v 112 but median 56 v 32
    LANDING_SLEW_MIN_DEG_S: float = 5.0     # floor under the slew rate (none is
                                            # applied before a torque is read)
    LANDING_DIVERT_GAIN: float = 0.12       # degrees of tilt per metre of miss
    LANDING_THROTTLE_GAIN: float = 6.0      # m/s^2 of trim per m/s of speed
                                            # error; the throttle is this over
                                            # max_accel.  It was 0.12 throttle
                                            # per m/s, which is this gain on
                                            # the original booster (~50 m/s^2
                                            # empty) and twice it on a 9 g
                                            # one: five Vectors on 55 t
                                            # flapped 0 <-> 0.95 on the last
                                            # 30 m, overshot into a climb and
                                            # ran the tanks dry (LOG10237)
    TOUCHDOWN_ALT_M: float = 0.2            # legs-above-ground cutoff height
    TOUCHDOWN_SPEED: float = 2.0            # target descent rate at cutoff
    LANDING_THROTTLE_VERTICAL: bool = True   # profile on descent rate, not |v|
                                            # (vert1-1016: heavy 6/6 at 0.9-8.3 m/s
                                            # on the 2 km prediction +-30 m, v 4/6
                                            # drifting 300-750 m off it)
    FINAL_APPROACH_RATE: float = 0.5        # m/s of descent allowed per metre up
    LANDING_FLARE_ALT_M: float = 12.0       # this low and this slow, hover
    LANDING_FLARE_SPEED: float = 8.0        # thrust is all that can be wanted
    LANDING_FLARE_MARGIN: float = 0.12      # ... plus this much to settle with
    LANDING_GOAROUND_ALT_M: float = 60.0    # below this, thrusting while
                                            # ascending is always wrong; see
                                            # trajectory.landing_command and
                                            # LOG429, which left the ground at
                                            # 101 m/s from 11 m up
    HOVER_CUT_ALT_M: float = 3.0            # ... or this low and not descending
    TOUCHDOWN_HOVER_VS: float = -0.5        # what counts as "not descending"
    LEG_CLEARANCE_MARGIN_M: float = 0.5     # padding under the bounding box
    LEG_CLEARANCE_MAX_M: float = 100.0      # longer than this is not a booster
    LEG_CLEARANCE_FALLBACK_M: float = 10.0  # used only while the game will not say
    GEAR_DEPLOY_ALT_M: float = 350.0        # backstop only, see below
    GEAR_ON_LANDING_BURN: bool = True       # drop the legs when the landing
                                            # burn lights, not at an altitude
    GEAR_ACTION_GROUP: int = 0              # extra AG to fire with the legs;
                                            # 0 = none (this booster's gear is
                                            # on the stock Gear group)
    GEAR_TIMED: bool = True                 # cycle the legs once early in the
                                            # coast to time them, then drop
                                            # them this long before touchdown:
    GEAR_LEAD_FACTOR: float = 1.25          # ... x the measured deploy time
    GEAR_LEAD_S: float = 1.0                # ... plus this; see landing_gear.
                                            # The time-to-go is the coast's fall
                                            # rate, which the burn stretches:
                                            # the legs are down ~13.5 s before
                                            # contact for this 6.8 s lead.
                                            # Timed on the burn's prediction
                                            # instead it was worse (def2-1018:
                                            # 22 m, 7/12 stood v 13 m, 10/12)
    GEAR_MEASURE_TIMEOUT_S: float = 20.0    # a cycle not done by now: untimed
    TOUCHDOWN_BACKSTOP_SPEED: float = 15.0  # height backstops only below this
    TOUCHDOWN_WATCH_S: float = 8.0          # stay this long after the 2 s
                                            # sign-off to see if it tips

    # --- Sanity limits on the terrain sensor -------------------------------
    TERRAIN_MAX_M: float = 7000.0           # highest ground the body can have
    TERRAIN_BELOW_SEA_M: float = 100.0      # slack for terrain under sea level

    # --- Trajectory prediction -------------------------------------------
    PREDICT_RK4: bool = True                # integrate the prediction with
                                            # RK4 rather than semi-implicit
                                            # Euler.  The step is not a
                                            # detail: drag goes as v^2 through
                                            # a density that changes by a
                                            # factor of e every 5 km, so a
                                            # first-order step biases a
                                            # hundred-second descent rather
                                            # than adding noise to it.  On
                                            # LOG243 (landed 109 m out) the
                                            # prediction from 20 km read 48 m
                                            # at DT_ATMO 0.5 and 106 m at
                                            # 0.05 with the vehicle doing
                                            # nothing in between -- the
                                            # "coast walk" was the
                                            # integrator.  False restores the
                                            # old scheme; see CLAUDE.md
                                            # failure 16.
    PREDICT_DT_VACUUM: float = 2.0
    PREDICT_DT_ATMO: float = 0.5
    PREDICT_MAX_TIME_S: float = 900.0
    DENSITY_SAMPLE_COUNT: int = 160         # atmosphere profile resolution
    AERO_REFRESH_UT: float = 1.0            # how often to re-probe drag
    DRAG_SMOOTHING: float = 0.25            # EMA weight for each Cd*A sample
    DRAG_OUTLIER_RATIO: float = 2.0         # clamp a sample to this x previous
    DRAG_WARMUP_SAMPLES: int = 1            # a bin averages its first N
                                            # samples with equal weight before
                                            # the clamp and the EMA start
                                            # (1 = off).  Measured at 4 it is
                                            # worse alone: it converges faster
                                            # onto whatever is arriving, and
                                            # during boostback that is the
                                            # broadside samples.  See
                                            # Environment._record.
    DRAG_PROBE_MAX_AOA_DEG: float = 0.0     # ignore Cd*A probes taken this far
                                            # off retrograde (0 = take them
                                            # all).  Measured at 20: it cannot
                                            # help the decision that needs it,
                                            # because nothing has been probed
                                            # at a descent attitude yet when
                                            # boostback exits.  See
                                            # Environment.sample_is_clean.
    DRAG_RESWEEP_AOA_DEG: float = 25.0      # re-probe the *whole* curve the
                                            # first time the vehicle is this
                                            # near the attitude the descent
                                            # is flown at.  The opening sweep
                                            # is taken during boostback, i.e.
                                            # broadside, and every bin it
                                            # writes is a side-on area used
                                            # to fly a nose-on descent; see
                                            # Environment.refresh_drag and
                                            # CLAUDE.md failure 13.  0 = off.
    DRAG_RESWEEP_INTERVAL_S: float = 10.0   # re-probe the whole curve this
                                            # often (in-game seconds) while
                                            # the vehicle is in its descent
                                            # attitude, not just the first
                                            # time it gets there.  0 = once.
                                            # See Environment.refresh_drag.
    DRAG_PROBE_DESCENT_ALTITUDE: bool = True    # probe each Mach bin at the
                                            # altitude the *predicted*
                                            # descent passes that speed,
                                            # instead of at the altitude the
                                            # booster is at now.  KSP scales
                                            # drag by a pseudo-Reynolds term
                                            # as well as by Mach, so a bin
                                            # probed at 30 km reads ~15-20%
                                            # high for the air it is used in;
                                            # see Environment.probe_altitude.
    DRAG_MACH_MAX: float = 6.0              # top of the Cd*A-against-Mach curve
    DRAG_CURVE_BINS: int = 24               # its resolution (0.25 Mach each)
    SOUND_SPEED_FALLBACK_M_S: float = 340.0  # if the body will not say
    DIAG_STATE: bool = False                # log the state vector and the
                                            # whole Cd*A curve each tick, so a
                                            # prediction can be re-flown
                                            # offline; see replay.py

    THROTTLE_DIVERGENCE: float = 0.1        # log when the vessel ignores us
    MASS_LOSS_FRACTION: float = 0.1         # mass down this much in one tick
                                            # is parts lost, not propellant
                                            # (LOG10720: 26.7 -> 0.66 t in 2 s)
    DEAD_STREAM_S: float = 1.0              # game seconds of a bit-identical
                                            # position while moving: the vessel
                                            # is destroyed and the streams have
                                            # frozen (LOG10748 flew a dead
                                            # snapshot for 60 s to the timeout)
    DEAD_STREAM_MIN_SPEED: float = 1.0      # below this a still vessel is
                                            # allowed a constant position

    # --- Loop / logging ---------------------------------------------------
    LOOP_SLEEP_S: float = 0.05              # pacing of the loop, seconds
    LOOP_PACING_GAME_TIME: bool = False     # pace on ut, not wall clock.  Only
                                            # matters when the game is running
                                            # off 1x -- see
                                            # testInstances/timescaleSrc
    LOOP_PACING_MAX_STRETCH: float = 5.0    # give up on a tick after this many
                                            # intervals of wall clock: a paused
                                            # game must not hang the autopilot
    LOG_INTERVAL_UT: float = 2.0            # in-game seconds between log lines
    LOG_DIR: str = "logs"


def defaults_fingerprint():
    """A short hash of every default in ``Config``.

    The config line lists differences from the defaults, which says nothing
    when a default itself moves between two batches; the fingerprint names
    the baseline.  Two logs with different fingerprints are not the same
    experiment, however alike their difference lists look.
    """
    base = Config()
    text = ";".join("%s=%r" % (f.name, getattr(base, f.name))
                    for f in sorted(fields(base), key=lambda f: f.name))
    return hashlib.sha1(text.encode()).hexdigest()[:8]


def apply_overrides(cfg, pairs):
    """Apply ``NAME=value`` strings to ``cfg`` in place, coercing types."""
    types = {f.name: f.type for f in fields(cfg)}
    for pair in pairs:
        name, _, raw = pair.partition("=")
        name = name.strip()
        if name not in types:
            raise KeyError("unknown config field: %s" % name)
        current = getattr(cfg, name)
        if isinstance(current, bool):
            value = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(current, int):
            value = int(raw)
        elif isinstance(current, float):
            value = float(raw)
        else:
            value = raw
        setattr(cfg, name, value)
    return cfg
