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
| [docs/spaceplane/handoff.md](../docs/spaceplane/handoff.md) | the last session's state -- read it first (index: the root HANDOFF.md) |
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

## Where it stands (2026-11-01, fingerprint `07089ff3`)

The approach plans in the middle of each craft's own span (`airframe.approach_ld`: clean reach from the STANDBY sweep at the dry mass, times the root of the split rudder's stop factor); `APPROACH_BEST_LD` is gone, and on qs_shuttle2 the law gives 4.22 where the constant was 4.2. The low-alpha glide is off (`GLIDE_LOW_ALPHA_SPEED_M_S` 0: on, 7/12 ran out of energy short). The Heavy Shuttle has a farm save, `qs_heavyshuttle_rigoff`, which has not yet flown in its stowed form. The root HANDOFF.md has the next batch.

Earlier states (2026-10-10 .. 10-31, with fingerprints): docs/spaceplane/journal.md, "Former where-it-stands
sections (moved 2026-11-18)" at its top.

## Next, in order

[docs/spaceplane/handoff.md](../docs/spaceplane/handoff.md): the approach turn on kRPC 0.7, then retaking the cone saves and derived tables; then `docs/spaceplane/constantsAudit.md` top-down.

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
  cone's standing trim lives on the canards (`CANARD_TRIM`); keep standing
  moments out of kRPC.  Client pitch/roll/yaw do nothing while kRPC's
  autopilot is engaged, so a trim has to go on a surface's deflection.
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
  that holds the speed, two-sided), the flare's sink tracking, and
  `Autopilot.aim_runway` (the nose down the runway, not the airflow).
- **The time scale serves the control interval.** `TIMESCALE_GOVERNOR`
  holds each phase to what its loop can serve; every log ends with a `loop
  rate` line -- read it before comparing two logs.
- **The deorbit aim is geared down 6.4:1** (a knob's gain before a null).
- **`DRAIN_BEFORE_BURN`** (failure 61) closed the generality gap.  The
  reserve the burn leaves is dumped on the first GLIDE tick
  (`DRAIN_LEFTOVER`, the user 2026-10-26), where the closed loop absorbs
  the valve's kick.
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
`ROLLOUT_DIVERT_SETTLE_S`; 2026-10-21: `GLIDE_PITCH_OFFLOAD` (client
pitch never reaches a vessel kRPC flies); 2026-10-26: `FUEL_TO_NOSE` (the user: slightly
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
