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

## Where it stands (2026-10-10 night, fingerprint `101fc0e0`)

**The terminal glide solves on cone-entry energy** (`GLIDE_TAEM_*`,
promoted 2026-10-10): below Mach 4.5 the glide nulls the predicted energy
height at the cone's entry against what the cone plans for (~14.1 km),
with the prediction flying the alpha achieved.  taemfix-1010: **23/24 on
the runway v 20/24, high orbit 6/6, 0 lost**.  Also default: brakes before
weaving, the deorbit burn lined up on stock SAS, the azimuth floor.

Off, built, unflown on the farm: `ABORT_NORTH` (every divert goes north;
sim: 8/8 stopped 158-197 m north) and `GROUND_SPOILER_ON_NOSE` (the
touchdown bounce at 68-76 m/s, 3 of 5 breakups).  Open: an inc-orbit
glide energy deficit (one ~8-10 km short per 24, both arms); two
low-speed rollouts that ran off the edge.

## Next, in order

The root [HANDOFF.md](../HANDOFF.md): fly `ABORT_NORTH` and
`GROUND_SPOILER_ON_NOSE` on the farm and promote or delete; the inc glide
energy deficit; then `docs/spaceplane/constantsAudit.md` top-down.

## Standing facts (each paid for; the journal has the evidence)

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
`HAC_BANK_FROM_LIFT`, `ALPHA_TRACKING_ON`, `APPROACH_CAPTURE_TAU_SHARE`.

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
