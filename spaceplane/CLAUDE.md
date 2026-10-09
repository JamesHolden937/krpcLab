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
`qs_shuttle2_rigoff` / `_inc_rigoff` / `_high_rigoff` (and `_ecc_rigoff`,
which arrives ~17 km long). **The old craft (`qs_plane`) is retired** (the
user, 2026-10-03: a flying brick with no real wings). Don't tune for it or
judge changes on it; its saves stay in `saves/` as history.

The bar is generality across the shuttle family (cargo / big-wing /
mass-distribution variants): per-craft laws and measured curves, not
constants fitted to one save.

## Where it stands (2026-10-09)

Defaults now carry the configuration measured best since 10-07:
`GLIDE_PITCH_OFFLOAD` (max 0.6, above Mach 3), `CANARD_TRIM`,
`HAC_LD_MEASURED`. Over 135 flights of it on 2026-10-08 it landed on the
runway **61/135 (45%)**: rigoff 27/49, inc 16/43, high 18/43. The failures:

- **long, 32** -- 3-9 km past the midpoint, intact, into the sea;
- **lost, 26** -- mostly the approach diving into the flare at 86-112 m/s;
- **off the strip, 12** -- flare entered 90-330 m off the centreline;
- short, 4.

**One number predicts most of it: the height surplus the cone hands the
approach** (`landsum.py`'s `surplus`). Under ~+400 m nearly every flight
lands; over ~+800 m it lands long (rollout-to-wheels ratio ~3.5) or dives
(ratio ~1.9). The cone rolls out 1.2-1.5 km high only ~6 km from the
threshold, which needs a ~28 deg path; the approach had no drag device, so
it banked 40 deg S-turns at alpha ~1 and spiralled in.

**The split rudder is the speedbrake** (`tools/splitprobe.py`): both fins
at Deploy Angle 38 double the approach's drag (L/D 4.17 -> 1.89) with no
yaw or roll. Flags using it (all off, under test):
`APPROACH_SPLIT_BRAKE` (open by geometry on final, alpha holds speed),
`HAC_SPLIT_BRAKE` + `HAC_SPLIT_ON_BRANCH` (in the cone),
`ROLLOUT_SPLIT_BRAKE`.

## Next, in order

1. Whatever `rot-asb-1009` (base v `APPROACH_SPLIT_BRAKE`) says -- see the
   root HANDOFF.md.
2. **Energy management the Shuttle's way**, replacing patches: the
   speedbrake used continuously against an energy-v-range target; a TAEM
   segment (S-turns) *before* the cone; overhead v straight-in chosen by
   energy; a dynamic-pressure schedule instead of a fixed 112 m/s true cone
   target.
3. The flare's lateral entry (off-strip landings).
4. `TOUCHDOWN_AIM_M` (1800) and `APPROACH_AIM_SHIFT_M` (1000) are fitted
   constants; derive the aim from flare float + rollout per vehicle.
5. The eccentric orbit arrives ~17-21 km long.

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
  Mach fixes the prediction (null on landings).
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

**Removed 2026-10-09** (refuted, null, or superseded; history in the
journal and `git log`): `HAC_SPEED_EAS`, `HAC_IAS_FROM_STALL`,
`HAC_SPEED_PATH`, `HAC_SHORT_BEST_GLIDE`, `APPROACH_SINK_GUARD`,
`APPROACH_SPLIT_ON_GUARD`, `APPROACH_SHARP_TURN`, `HAC_SPIRAL_DUMP`,
`HAC_GATE_STRETCH`, `HAC_PAST_KEEPS_LINEUP`, `HAC_EXIT_LAP_AT_TARGET`,
`HAC_WRAP_BEFORE_GATE`, `HAC_CHOOSE_BY_ENERGY`, `HAC_WEAVE_HELD`,
`HAC_WEAVE_STRAIGHT_ONLY`, the cone flap-brake variants, `PROPELLANT_TRIM*`,
`AIRFRAME_DERIVED`, `APPROACH_LD_DERIVED`, `GLIDE_ENERGY_AIM`.

## Commands

```bash
./run.sh --pilot spaceplane                          # fly it in your game, waiting for START
./spaceplane/tools/quickglide.py -n 1 --instance 0 --save qs_shuttle2_rigoff   # one farm flight
./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 2 logs/rot-X.txt "$PWD" "save|SET=1;SET2=2" ...  # arms rotated over the farm
./spaceplane/tools/entrysave.py 0                     # quicksave mid-flight (interface, or --alt on final)

./spaceplane/tools/landsum.py logs/LOG88*   # arrival -> cone surplus -> wheels, one line per flight
./spaceplane/tools/oscsum.py logs/LOG88*    # alpha error, sideslip, per phase
./spaceplane/tools/armsum.py --by X --against '(default)' logs/LOG88*   # one line per arm
./spaceplane/tools/conesum.py --settled logs/LOG88*
./spaceplane/tools/conepolar.py             # the cone's flown polar -> logs/conepolar/
./spaceplane/tools/holdprior.py --by-mach --mach 0.8   # the glide's holdable alpha -> logs/holdprior/
./spaceplane/tools/splitprobe.py 0          # the split rudder's drag, yaw, roll

python3 -m unittest spaceplane.tests.testSpaceplane   # offline, no KSP
./testInstances/actuators.py 0               # every module setting of the craft
```

**Method notes.** Code is frozen during a batch (`rotfly.sh` runs the
tree): develop in a git worktree. Label arms from each log's `config:`
line. `pgrep -f` in a wait loop matches itself.
