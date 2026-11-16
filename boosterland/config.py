"""All tunables for the boostback-and-land script.

Every number the guidance uses lives here so tuning never means editing
control logic.  Override any of them from the command line, e.g.::

    ./run.sh --set NOISE_SIGMAS=3 --set LOG_INTERVAL_UT=1
"""

from dataclasses import dataclass

from common.config import apply_overrides, differences, fingerprint  # noqa: F401


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
    SEPARATION_MAX_COAST_S: float = 5.0     # ... nor wait longer for clearance
    DEPLOY_LAW: bool = True                 # the brakes fly closed (their
                                            # floor) and open fully at the
                                            # height whose prediction lands
                                            # on the pad (Autoland.
                                            # run_late_deploy)
    DEPLOY_MAX_DEG: float = 90.0            # the law's unit: full travel of
                                            # every brake
    FINS_HELD_KEEP_DEPLOY: bool = True      # BOOSTBACK_FINS_HELD holds the
                                            # surfaces by their ignore flags,
                                            # so brake fins stay deployed and
                                            # boostback's drag probe sees the
                                            # descent configuration.
                                            # keep1/gate1-1111: median 34-40
                                            # v 49 (kRPC's override retracts
                                            # them: it drives deployAngle)
    FIN_BRAKES: bool = True                 # alternate fins deployed opposite
                                            # ways as an airbrake, on the
                                            # airbrake law (brake_fins).
                                            # finbrake2-1110, qs_finbooster,
                                            # 10 an arm: median 42 v 460 m,
                                            # 9/10 within 102 m (LOG15574
                                            # 1062: late-brake probe at aoa 21)
    DEPLOY_LATE_STEP_S: float = 5.0         # re-solve the switch this often
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
    RCS_SAT_CLOSING_FRACTION: float = 0.2
    RCS_SAT_RELEASE: float = 0.6
    # No dynamic-pressure ceiling on this vehicle: the spaceplane shuts its
    # thrusters off in thick air because its control surfaces own the
    # attitude there, and a booster's do not -- the descent is flown on RCS,
    # reaction wheels and the gimbal.  0 disables the test, and the
    # snapshot carries no dynamic pressure to make it with anyway.
    RCS_Q_MAX_PA: float = 0.0

    # --- Boostback --------------------------------------------------------
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
    FLIP_THRUST_LOSS: float = 0.1           # the flip is over when thrust
                                            # along the nose loses this
                                            # share to the angle off the aim
                                            # (acos(0.9) = 25.8 deg)
    FLIP_UNDER_POWER: bool = True           # ignore that gate: light, then flip
    # The user's 2026-10-18 proposal: during the flip, burn only the
    # throttle that makes the turn cheapest -- gimbal authority bought with
    # thrust that mostly points the wrong way, against the drift every
    # second of turning adds (guidance.flip_throttle, solved every tick from
    # the live gimbal geometry, RCS and wheel torque).  Off = full throttle
    # through the flip.  ascent/budget.boostback models the same law.
    # flipheavy-1018 (qs_heavy, 6 v 6): boostback 27.9 v 29.5 t, flip 6 s
    # at mean throttle 0.08-0.16, stood 6/6 v 4/6, but lands ~+45 m long
    # (median 43 v 20 m).  flip1-1018 (ascent stacks): boostback -3.3 t on
    # the shuttle stack, all of it spent on the climb.
    FLIP_THROTTLE_LAW: bool = True
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
    ROLL_ALIGN: bool = True             # hold roll: the roof at the angle from
                                        # the trajectory plane it had at the
                                        # first aim (Autoland.roll_hold)
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
                                        # is carried, not re-taken from the
                                        # trajectory plane (0 = never)
    # When the engines light after the flip, reset kRPC's autopilot if it
    # latched an axis as structurally flexible: an axis with no authority
    # through the unpowered flip (the fin booster's roll on RCS) latches on
    # its own runaway gain, and the latch floors that axis for the rest of
    # the flight (Autoland.clear_krpc_latches).
    KRPC_CLEAR_LATCH_ON_THRUST: bool = True
    # From the coast on, the roof is held at the nearest of the n roll
    # orientations the aerodynamic surfaces look the same in (autoland.
    # symmetry_order: 6 for the fin booster), so the roll error stays under
    # 180/n deg; a slot is left once another is nearer by this fraction.
    ROLL_SYMMETRY: bool = True
    ROLL_SLOT_HYSTERESIS: float = 0.15

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
                                            # can move it but the RCS trim, and
                                            # the landing burn has
                                            # ~10 m of authority.  Costs one
                                            # more propagation a tick.

    AERO_STEER_TRACK_SMOOTHING: float = 0.1  # EMA weight per sample
    AERO_STEER_TRACK_FLOOR: float = 0.05    # a tracking of 0 would read as a
                                            # vehicle with no air at all
    AERO_STEER_HOLD_PROBES: int = 8         # torque and lift probes per
                                            # refresh, evenly spaced to 90 deg
                                            # off retrograde: the hold limit,
                                            # and the angle of peak lift that
                                            # caps it (Environment.
                                            # refresh_holdable)
    AERO_STEER_HOLD_PROBE_S: float = 2.0    # game seconds between refreshes
    AERO_STEER_HOLD_MARGIN: float = 0.7     # fraction of the control torque
                                            # a steady hold may spend on the
                                            # aerodynamic moment; the rest is
                                            # left for disturbances

    # --- Landing burn -----------------------------------------------------
    LANDING_THROTTLE_CAP: float = 1.0       # the engines' full thrust; the
                                            # profile plans at cos(LANDING_
                                            # MAX_TILT_DEG) of it, leaving the
                                            # rest to the divert's tilt
                                            # (trajectory.planned_net)
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
    SETTLE_LAGS: float = 3.0                # a nose is settled this many
                                            # measured lags after a command:
                                            # the coast's angle of attack
                                            # fades to zero over them before
                                            # the predicted ignition
                                            # (Autoland.unwound), the RCS
                                            # valve waits them out
                                            # (ValveLimits)
    LANDING_TERMINAL_GUIDANCE: bool = True   # steer the landing burn; see
                                            # guidance.terminal_command
    LANDING_ZEM_GAIN: float = 6.0           # position term (double-integrator optimal)
    LANDING_BURN_AIM: bool = True           # the burn steers at the pad
                                            # (ZEM on top of the base); off:
                                            # it only brakes and nulls the
                                            # sideways speed, the coast aims.
                                            # Measured in aim-1115
    LANDING_DIVERT_THROTTLE_BUDGET: bool = True  # never tilt into the
                                            # braking: cap the tilt at the
                                            # angle the *spare* throttle can
                                            # pay for.  Only bites when the
                                            # divert is on at all.
    LANDING_MAX_TILT_DEG: float = 15.0
    LANDING_HEADROOM: float = 0.85          # the profile is planned at this
                                            # share of the thrust left after
                                            # the tilt: the trim's room to
                                            # correct (policy; the inverse of
                                            # the old 1.18 margin).  At 1 the
                                            # burn had none: gen2-1115, 77 m/s
                                            # at 28 m on full thrust
    NOISE_SIGMAS: float = 2.0               # every deadband is this many
                                            # sd of the prediction's own
                                            # tick-to-tick scatter
                                            # (common.jitter): act on a miss
                                            # only when it is not noise
    LOOP_BANDWIDTH_SHARE: float = 0.5       # an outer loop closes at this
                                            # share of the measured bandwidth
                                            # of the loop that executes it:
                                            # the lateral null of the attitude
                                            # loop's (common.attitudelag), the
                                            # descent-rate trim of the
                                            # throttle's (spool-up + one tick).
                                            # Cascade separation.  0.25 lit
                                            # the fin boosters at 7 km on a
                                            # slow taper and missed ~1 km
                                            # (meas-1115); the old fixed
                                            # 0.5 /s was ~half the loop
    TOUCHDOWN_ALT_M: float = 0.2            # legs-above-ground cutoff height
    SURVIVAL_SHARE: float = 0.25            # the touchdown is flown at this
                                            # share of what the craft
                                            # survives: the descent rate of
                                            # the weakest part that touches
                                            # (trajectory.touchdown_speed,
                                            # 1/16 of the breaking energy),
                                            # the lean of the legs' tip angle
                                            # (guidance.terminal_command)
    LANDING_THROTTLE_VERTICAL: bool = True   # profile on descent rate, not |v|
                                            # (vert1-1016: heavy 6/6 at 0.9-8.3 m/s
                                            # on the 2 km prediction +-30 m, v 4/6
                                            # drifting 300-750 m off it)
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
    DIAG_BURN: bool = False                 # log the landing burn the
                                            # prediction flies at ignition,
                                            # and the flown burn every tick;
                                            # see burnplan.py

    # kRPC 0.8's AutoPilot.limit_angular_velocity (on by kRPC's default):
    # caps the nose's rate at the rate the flight path turns at the
    # incidence the spare torque holds (AerodynamicRateLimit, an aircraft's
    # model).  On the fin booster's burn that is 0.001-0.04 rad/s, and the
    # nose sat 6-12 deg off its command for the first 10 s (LOG16724.ap.csv).
    # nl1-1114 v rt1-1114 (qs_finbooster2, 12 an arm, auto-tune on): off,
    # burn input never pinned (0.00 v 0.12 of ticks), miss median 61 v 253.
    KRPC_LIMIT_ANGULAR_VELOCITY: bool = False
    KRPC_AP_DIAG: bool = False              # kRPC's own per-physics-tick
                                            # autopilot record (errors, rates,
                                            # gains, outputs per axis), saved
                                            # at shutdown as LOG<n>.ap.csv;
                                            # kRPC keeps 3000 ticks (60 s),
                                            # restarted at the coast
    KRPC_AP_DIAG_BELOW_M: float = 0.0       # ... or once the coast is
                                            # below this altitude (0: at
                                            # its start)

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
    # Engine gimbals locked while the commanded throttle is 0 and freed
    # when it rises (``common.idle.IdleGimbals``): a gimbal makes no torque
    # without thrust, and a nozzle swinging on the coast looks broken.
    IDLE_GIMBAL_LOCK: bool = True
    BOOSTBACK_FINS_HELD: bool = True        # control surfaces held neutral
                                            # through SEPARATION and BOOSTBACK,
                                            # handed back at COAST: at aoa
                                            # 50-150 deg kRPC's PID drives the
                                            # fins into a +-40 deg swing.
                                            # fins1-1110, qs_finbooster, 9 an
                                            # arm: boostback err median 2-3 v
                                            # 18-43 deg; 0/9 v 5/9 hit at
                                            # 130-500 m/s (LOG15544-15561)
    LOG_INTERVAL_UT: float = 2.0            # in-game seconds between log lines
    LOG_DIR: str = "logs"


def defaults_fingerprint():
    """A short hash of every default in this ``Config``
    (``common.config.fingerprint``)."""
    return fingerprint(Config)
