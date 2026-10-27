# spaceplane

Deorbits a winged vehicle, flies an atmospheric entry and lands it on the KSC
runway. The root `CLAUDE.md` (conventions, farm, cross-cutting rules) applies
here in full.

```
STANDBY -> DEORBIT -> DRAIN -> COAST -> GLIDE -> HAC -> APPROACH
                                                          |
                                    STOPPED <- ROLLOUT <- FLARE
```

One phase machine (`autopilot.py`) is the only kRPC-aware code; every tick
re-propagates to the ground (`trajectory.py`, RK4) and every phase decision
comes from that fresh prediction.

| read | when |
|---|---|
| this file | the current state and what to do next |
| the root [HANDOFF.md](../HANDOFF.md) | the last session's state -- read it first |
| [docs/spaceplane/journal.md](../docs/spaceplane/journal.md) | what each session measured, oldest first -- **append new sessions here** |
| [docs/spaceplane/design.md](../docs/spaceplane/design.md) | the runway, the airframe measurements, the architecture phase by phase |
| [docs/spaceplane/failures.md](../docs/spaceplane/failures.md) | **before undoing anything** -- numbered failures, cited everywhere by number |

## The target is the shuttle

`qs_shuttle2` (Mk3, twin fins, ~30 t at landing) and its orbits
`qs_shuttle2_rigoff` / `_inc_rigoff` / `_high_rigoff` / `_ecc_rigoff`
(77 x 135 km since 2026-10-09; the earlier ecc save was invalid, periapsis
inside the atmosphere). **The old craft (`qs_plane`) is retired** (the
user, 2026-10-03: a flying brick with no real wings). Don't tune for it or
judge changes on it; its saves stay in `saves/` as history.

The bar is generality across the shuttle family (cargo / big-wing /
mass-distribution variants): per-craft laws and measured curves, not
constants fitted to one save.

**The goal (the user, 2026-10-10): >= 95% stopped on the runway
(|along| <= 1.2 km), 0 lost (no water, no breakup), worst miss under
~3 km** -- on all four orbits and at least two airframes of the family.
Losses matter more than overruns: a 1.4 km overrun is untidy, a water
landing is a failure.  Telling 95% from 85% takes ~60 flights, so judge
it on pooled batches; at 24 an arm read the tails as well as the count.
Each remaining miss is its own failure mode (cone out of energy, glide
arriving high, high orbit long) -- remove them one by one.  Above ~97%
the misses are mostly the farm's own faults, not the autopilot.
**Every divert goes north**: the space center lies south of the runway, so a
flight that cannot make or stop on the runway lands north of it
(`ABORT_NORTH`); a miss to the south counts as a loss.

## Where it stands (2026-10-25, fingerprint `66612c16`)

**The user's goal: neutral/unstable craft to the runway reliably.** The cargo shuttle is dropped (inherently problematic). The test family is the shuttle with its wings moved forward: `savegen.py --wing-forward M`, saves `qs_shuttle2_fwdwingNN_rigoff`.

- **Defaults land through 1.5 m forward** (stock 7-8/8, 1.2 m 6/8, 1.5 m 8/8 on the runway). **1.8 m: 5/16.** 3 m and beyond: 0.
- Stability is set by the **drained** airframe, which has a much more aft CoM than the orbital save. At 3 m it is statically unstable hypersonically; at 4 m the elevons cannot hold it at any alpha.
- The fins move with the wing, so yaw stiffness falls too (1.8 m: -20%).
- **1.8 m loss modes:**
  - a hypersonic lateral limit cycle (slip +-17, roll input saturated);
  - a transonic pitch oscillation (alpha 12-46 on +-0.2 input; the static pitch loop is too slow there).
- **The approach turns:** `APPROACH_REENGAGE` (default) resets kRPC's integrators at the cone's exit. Turn ratio median 0.88 against 0.16.
- The cone saves `qs_s2_hac*` are stale (overshoot 5-14 km): retake them.

Root HANDOFF.md has the numbers.

## Earlier (2026-10-22 night, fingerprint `9eec0e1f`)

The plant has been the same since 2026-10-21: AtmosphereAutopilot removed (stock surfaces), kRPC 0.7.0, and `KRPC_WOBBLE_MITIGATION` off.

`ATTITUDE_SLEW_FACTOR` is 1.0, retuned for 0.7's controller. slew-1021 measured 14/23 intact on the runway against 7/23 at 1.91.

dead-1022 default arm, intact on the runway:
- rigoff 6/6
- inc 4/6
- ecc 5/6
- high 2/5

**Blocker: the approach cannot turn.** Its heading rate is about 0.14 to -0.16 of a coordinated turn (0.82-0.97 on 0.6 + AA, or at 1.91). It hands the flare 200-400 m of cross-track, and the losses are touchdowns beside the runway.

kRPC's pointing deadband is refuted as the cause (dead-1022). Next: log kRPC's target and error on the approach.

The cone saves `qs_s2_hac*` and every derived table predate the plant change. Retake them.

## Earlier (2026-10-21 evening, fingerprint `56d5a616`)

Baseline on the new plant at slew 1.91 (6 each), intact on the runway:
- rigoff 5/6
- inc 2/6
- ecc 3/6
- high 3/6
- cargo 0/6

4/24 broke up on the runway at touchdown.

## Earlier (2026-10-21 morning, fingerprint `fb9a9b61`)

**The user's goals:** fly craft with minimum stability margin, and the
shortest reliable time from deorbit burn to an intact stop.  Burn->stop
is ~1100 s on `qs_shuttle2_rigoff` and 1150-1250 on ecc/high (was ~1560):
the deorbit takes the *latest* window that still leaves the glide 15% of
the first window's reach and costs no more than a surface periapsis
(`DEORBIT_LATEST`); the coast turns to the entry attitude on the RCS relay
(`COAST_RCS_TURN` -- on wheels alone a third of entries met the air
25-62 deg in sideslip); reversals are slewed on wheel + surface torque
only (`BANK_ACCEL_NO_RCS`); the short divert judges reach on the table's
best glide over energy height (`ABORT_SHORT_BEST_GLIDE`).  On the runway:
base 5/6, inc 6/6, ecc 22/26 -- **high 8/19 is the blocker**: clean
entries bank -55 at Mach 7 and the propagator over-predicts range there,
so the glide arrives at 12 km and ~180 m/s.  Score landings by the
`DOWN:` line and by each log's own save/`config:` (arm labels slip when
an instance dies).  `airframe.glide_ld(env, cfg)` returns the *constant*;
the table's numbers are `env.best_ld`/`best_speed`.  The low-margin test
airframe is `qs_shuttle2_fwdwing_rigoff`; `qs_stackorb` is not flown.

## Earlier (2026-10-21 night, fingerprint `cb0ae1d4`)

Burn->stop ~1100 s on rigoff via `DEORBIT_LATEST`, aim 1200,
`ABORT_NORTH_TOLERANT`, the cone's energy speed and signed negative-alpha
lift.

## Earlier (2026-10-11 evening, fingerprint `b5651d39`)

**The cone holds dynamic pressure** (`HAC_SPEED_Q`, promoted 2026-10-11):
its speed target is the gate target's q at every height (~245 m/s true at
12 km), not a constant true airspeed that is below the stall up high.
speedq2-1011: **19/24 v 14/24**, RCS in the cone 15-40 s v 92-99.

## Earlier (2026-10-10 late, fingerprint `bc9ac544`)

**The terminal glide solves on cone-entry energy** (`GLIDE_TAEM_*`,
promoted 2026-10-10): below Mach 4.5 the glide nulls the predicted energy
height at the cone's entry against what the cone plans for (~14.1 km),
with the prediction flying the alpha achieved.  taemfix-1010: **23/24 on
the runway v 20/24, high orbit 6/6, 0 lost**.  Also default: brakes before
weaving, the deorbit burn lined up on stock SAS, the azimuth floor.

**The deorbit wait is planned** (2026-10-10 late, unflown): the window is
found on the propagated orbit, `warp_to` takes the game there, SAS turns to
*surface* retrograde in the lead (Maneuver mode held 4-5 deg off on an
inclined orbit and never committed, LOG9598), and the coast `warp_to`s
its release.

Open: the touchdown bounce at 68-76 m/s (3 of 5 breakups); an inc-orbit
glide energy deficit (one ~8-10 km short per 24, both arms); two
low-speed rollouts that ran off the edge.

## Next, in order

The root [HANDOFF.md](../HANDOFF.md): the approach turn on kRPC 0.7, then retaking the cone saves and derived tables; then `docs/spaceplane/constantsAudit.md` top-down.

## Standing facts (each paid for; the journal has the evidence)

- **The shuttle trims at ~22 deg hypersonically** (lowa-1012): commanding
  16-18 at Mach 4-5 flies ~22 and leaks into yaw (slip > 10 on 22% of
  ticks v 9%); a 10-20 deg plan went 3/12 v 11/12.  Heat is not what
  holds alpha up -- the hottest part plateaus at 0.86-0.88 over a 4x range
  of heating.  To fly lower, move the trim (CoM, glide trim) first.
- **Roll and yaw are one lateral axis at high alpha** (failure 99): at 35
  deg of alpha a body roll *is* sideslip. Roll flies the derived 4.8 s.
  Tune the lateral axes together and fly any change on the farm first.
- **kRPC holds standing moments in integrators that a spiral twists.** The
  glide's pitch is offloaded to a body-frame trim (`GLIDE_PITCH_OFFLOAD`)
  and the cone's standing trim lives on the canards (`CANARD_TRIM`); keep
  standing moments out of kRPC.
- **Below Mach 3 the shuttle holds 0.57-0.75 of the ~40 deg the glide
  commands**, so the glide over-delivers cone energy; `HOLDABLE_PRIOR` by
  Mach fixed the prediction and was null on landings (deleted).
- **The aero probe measures at the surfaces' present deflection**, so
  subsonic rows re-probed mid-glide describe an airframe never flown there;
  the cone's flown lift peaks ~139 m^2 near 12 deg (table ~300).
  `tools/conepolar.py` writes the flown polar to `logs/conepolar/`
  (regenerate after any trim or attitude change).
- **The control surfaces are live on every axis.** KSP's `Pitch=False` is
  the *ignore* flag.
- **Laws, not constants, on final**: `APPROACH_SPEED_PATH` (the descent
  that holds the speed, two-sided), `FLARE_SINK_TRACK`,
  `AIM_RUNWAY_TRUE_ALPHA`.
- **The time scale serves the control interval.** `TIMESCALE_GOVERNOR`
  holds each phase to what its loop can serve; every log ends with a `loop
  rate` line -- read it before comparing two logs.
- **The deorbit aim is geared down 6.4:1** (a knob's gain before a null).
- **`DRAIN_BEFORE_BURN`** (failure 61) closed the generality gap.
- **Touchdown breakups are not a priority** unless a wing/tail strike or a
  genuinely hard arrival (the user, 2026-10-08). `KSP.log` on the farm
  names every exploded part.
- **Batch sizes:** a flight-to-flight scatter this size needs 15-20 flights
  an arm to tell 45% from 65%. Interleave arms (`rotfly.sh`) and restart the
  farm every 2 rounds (swap reaches ~20 GB).

**Removed 2026-10-09/10** (refuted, null, or superseded; history in the
journal and `git log`): `HOLDABLE_PRIOR`/`_BY_MACH`, `COAST_TRIM`, `PITCH_P_CONE`,
`CONE_TRIM_HANDOFF`, `HAC_LD_AT_TARGET`, `GLIDE_CONE_ENERGY`/`_CEILING`,
`ENTRY_INTERFACE_AT_AIR`, `GEAR_GEOMETRY_DEPLOYED`, `AERO_REFRESH_NEAR_MACH`,
`TOUCHDOWN_AIM_DERIVED`, `BAY_BRAKE`, `HAC_SPLIT_ON_BRANCH` (`HAC_SPLIT_BRAKE` rebuilt the same evening),
`HAC_LD_FLOWN_POLAR`, `ROLLOUT_SPLIT_BRAKE`, `FLARE_SPLIT_BRAKE`,
`APPROACH_SPEED_KI`, `APPROACH_ALPHA_MIN_DEG`, `GLIDE_SPLIT_BRAKE`, `GLIDE_SETTLE_ON_FLOWN`, `hac_flap_brake`/`command_airbrake`, `ALPHA_BINS_NEGATIVE`, `HAC_SPEED_EAS`, `HAC_IAS_FROM_STALL`,
`HAC_SPEED_PATH`, `HAC_SHORT_BEST_GLIDE`, `APPROACH_SINK_GUARD`,
`APPROACH_SPLIT_ON_GUARD`, `APPROACH_SHARP_TURN`, `HAC_SPIRAL_DUMP`,
`HAC_GATE_STRETCH`, `HAC_PAST_KEEPS_LINEUP`, `HAC_EXIT_LAP_AT_TARGET`,
`HAC_WRAP_BEFORE_GATE`, `HAC_CHOOSE_BY_ENERGY`, `HAC_WEAVE_HELD`,
`HAC_WEAVE_STRAIGHT_ONLY`, the cone flap-brake variants, `PROPELLANT_TRIM*`,
`AIRFRAME_DERIVED`, `APPROACH_LD_DERIVED`, `GLIDE_ENERGY_AIM`, `HAC_ENTRY_DERIVED`,
`HAC_BANK_FROM_LIFT`, `ALPHA_TRACKING_ON`, `APPROACH_CAPTURE_TAU_SHARE`;
2026-10-18, unflown or never re-flown: `HAC_SHORT_POLAR`, `AIRBRAKE_CACHE`,
`GROUND_SPOILER_ON_NOSE`; 2026-10-20: `HAC_FAST_FACTOR`, `HAC_FAST_TAEM`,
`HAC_ENVELOPE`, `ROLL_DAMPER_SLIP_GATE_DEG`, `RCS_HOLD_IN_AIR`,
`ROLLOUT_DIVERT_SETTLE_S`; 2026-10-26: `FUEL_TO_NOSE` (the user: slightly
tail-heavy craft now fly; the propellant stays where the craft put it).

## Commands

```bash
./run.sh --pilot spaceplane                          # fly it in your game, waiting for START
./spaceplane/tools/quickglide.py -n 1 --instance 0 --save qs_shuttle2_rigoff   # one farm flight
./spaceplane/tools/multirot.sh TAG 3 "save|SET=1;SET2=2" ...   # arms rotated over the farm, restarted every 2 rounds
./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 2 logs/rot-X.txt "$PWD" "save|SET=1" ...  # one farm cycle
./spaceplane/tools/entrysave.py 0                     # quicksave mid-flight (interface, or --alt on final)

./spaceplane/tools/landsum.py logs/LOG88*   # arrival -> cone surplus -> wheels, one line per flight
./spaceplane/tools/oscsum.py logs/LOG88*    # alpha error, sideslip, per phase
./spaceplane/tools/armsum.py --by X --against '(default)' logs/LOG88*   # one line per arm
./spaceplane/tools/conesum.py --settled logs/LOG88*
./spaceplane/tools/conepolar.py             # the cone's flown polar -> logs/conepolar/
./spaceplane/tools/splitprobe.py 0 [--glide] # the split rudder's drag, yaw, roll
./spaceplane/tools/spoilerprobe.py 0         # each surface as a spoiler; the best pitch-neutral lift dump

python3 -m unittest spaceplane.tests.testSpaceplane   # offline, no KSP
./testInstances/actuators.py 0               # every module setting of the craft
```

**Method notes.** Code is frozen during a batch (`rotfly.sh` runs the
tree): develop in a git worktree. Label arms from each log's `config:`
line. `pgrep -f` in a wait loop matches itself.
