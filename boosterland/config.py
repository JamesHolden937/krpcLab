"""All tunables for the boostback-and-land script.

Every number the guidance uses lives here so tuning never means editing
control logic.  Override any of them from the command line, e.g.::

    ./run.sh --set BOOSTBACK_TOLERANCE_M=250 --set LOG_INTERVAL_UT=1
"""

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
    SEPARATION_PRETURN: bool = False    # start the flip during that drift
    SEPARATION_CLEARANCE_SCAN: bool = True  # flip when clear, not on a timer
    SEPARATION_MIN_COAST_S: float = 0.0     # never flip sooner than this
    SEPARATION_MAX_COAST_S: float = 5.0     # ... nor wait longer for clearance
    STARTUP_ACTION_GROUP: int = 2       # AG fired at sequence start (grid fins)
    ENABLE_RCS: bool = True
    # **Permission is not demand.**  From START the booster is allowed RCS for
    # the whole flight -- every turn it makes is one time cares about -- but
    # allowed is not the same as open.  Most ticks of a five-minute flight the
    # vehicle is already pointed where it was told and the autopilot is merely
    # hunting, and a thruster held open through the hunt pays for every
    # oscillation.  ``boosterland.rcs.Valve`` makes the valve a relay on the
    # angle between the commanded nose and the real one: on above ``ON``, off
    # once inside ``OFF`` for ``SETTLE_S``.  The deadband is what stops the
    # relay switching at the frequency of the oscillation it is damping.
    #
    # Wide enough not to interfere with the phases that steer: the correction
    # burn lights the gimbal when it is more than ``CORRECTION_ALIGN_DEG``
    # (20) off and not coming round, so the thrusters are long since open by
    # the time that test is reached.
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
    FLIP_ALIGN_DEG: float = 25.0            # hold throttle until aligned
    FLIP_UNDER_POWER: bool = True           # ignore that gate: light, then flip

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
    FLIP_VIA_VERTICAL: bool = True      # route big turns over the top, not under
    FLIP_VIA_VERTICAL_DEG: float = 90.0     # only turns bigger than this
    ROLL_ALIGN: bool = True             # hold roll, roof in the trajectory plane
    ROLL_OFFSET_DEG: float = 0.0        # roof offset from that plane (fin clocking)

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

    # --- Coast ------------------------------------------------------------
    COAST_STEER_GAIN: float = 0.0006        # degrees of tilt per metre of miss
    COAST_MAX_TILT_DEG: float = 12.0

    # --- Mid-course correction (trims the miss the coast opens up) ---------
    CORRECTION_ENTER_M: float = 60.0        # start correcting above this miss.
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
    CORRECTION_MAX_BURNS: int = 3           # engines have limited relights
    CORRECTION_PROBE_DV: float = 5.0        # dv used to test which way a burn moves
    CORRECTION_GRADIENT_POWERED: bool = False   # fly the landing burn in the
                                            # gradient probes too; see
                                            # trajectory.miss_gradient
    CORRECTION_ABORT_RATIO: float = 1.3     # abort if the miss grows this much
    CORRECTION_RETIRE_ON_BACKFIRE: bool = False  # a burn that backfires ends
                                            # the *phase*, not just itself;
                                            # see run_correction
    CORRECTION_ABORT_GRACE_S: float = 3.0   # ... after this long, so a turn can settle
    CORRECTION_MIN_GRADIENT: float = 0.5    # metres of miss closed per m/s, minimum

    # --- Landing burn -----------------------------------------------------
    LANDING_THROTTLE_CAP: float = 0.95      # headroom for the throttle loop
    SUICIDE_MARGIN: float = 1.18            # light the engines this much early
    SUICIDE_REACTION_S: float = 1.5         # engine spool-up allowance
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
                                            # retrograde for the last few m
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
    LANDING_DIVERT_GAIN: float = 0.12       # degrees of tilt per metre of miss
    LANDING_THROTTLE_KP: float = 0.12       # per m/s of speed error
    TOUCHDOWN_ALT_M: float = 0.2            # legs-above-ground cutoff height
    TOUCHDOWN_SPEED: float = 2.0            # target descent rate at cutoff
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
    TOUCHDOWN_BACKSTOP_SPEED: float = 15.0  # height backstops only below this

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
    DRAG_PROBE_AXIAL: bool = False          # see Environment.probe_direction: measured worse
    DRAG_PROBE_DESCENT: bool = False        # probe the attitude the descent is
                                            # flown at instead of the one the
                                            # booster currently holds; see
                                            # Environment.probe_rotation
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
