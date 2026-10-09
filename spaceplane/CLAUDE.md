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

## Where it stands (2026-10-09 evening, fingerprint `9abe7afd`)

**The split rudder speedbrake on final is the default** (`APPROACH_SPLIT_BRAKE`):
rigoff/inc/high land **46/66 on the runway (70%)**, against 61/135 (45%)
the day before; head to head with it off, 24/30 v 8/30.  The cone hands
over 0.6-1.7 km high ~6 km out (a ~28 deg path); the fins open to the L/D
factor the geometry asks for and alpha holds speed, so the drag buys path
angle.  Gated on 1.45 x stall: **the shuttle cannot fly below ~+1 deg of
alpha on final**, so its steady glide is ~80 m/s and the approach's 115
m/s target is never held.

Remaining on the good orbits: a few long by <1.4 km (fast flare entries),
a few short, off-strip by 40-60 m.  **The eccentric orbit
(`qs_shuttle2_ecc_rigoff`) is the blocker**: it reaches the cone 10-50 km
long, and the miss is made by the ~90 s post-burn flip to entry attitude
at 69 km (vertical speed after it orders the arrivals exactly).  On that
orbit the burn's stop test cannot measure (t2g > 1500 s), so the burn ends
on the solved dv (root HANDOFF.md).  The single-fin `qs_shuttle`
(no pair, brake absent) stops on the runway 5/6.

## Next, in order

1. The eccentric orbit: correct the flip's disturbance with a measurement
   that exists there (not the stop test), or take the disturbance out.
2. Fast flare entries (86-96 m/s float long) -- braking in the flare was
   null; and the flare's lateral entry.
3. **Energy management the Shuttle's way**: a TAEM segment before the cone
   so the handover is not 1-1.7 km high; overhead v straight-in by energy.
4. `TOUCHDOWN_AIM_M` / `APPROACH_AIM_SHIFT_M` per vehicle.

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

**Removed 2026-10-09** (refuted, null, or superseded; history in the
journal and `git log`): `HOLDABLE_PRIOR`/`_BY_MACH`, `COAST_TRIM`, `PITCH_P_CONE`,
`CONE_TRIM_HANDOFF`, `HAC_LD_AT_TARGET`, `GLIDE_CONE_ENERGY`/`_CEILING`,
`ENTRY_INTERFACE_AT_AIR`, `GEAR_GEOMETRY_DEPLOYED`, `AERO_REFRESH_NEAR_MACH`,
`TOUCHDOWN_AIM_DERIVED`, `BAY_BRAKE`, `HAC_SPLIT_BRAKE`, `HAC_SPLIT_ON_BRANCH`,
`HAC_LD_FLOWN_POLAR`, `ROLLOUT_SPLIT_BRAKE`, `FLARE_SPLIT_BRAKE`,
`APPROACH_SPEED_KI`, `APPROACH_ALPHA_MIN_DEG`, `ALPHA_BINS_NEGATIVE`, `HAC_SPEED_EAS`, `HAC_IAS_FROM_STALL`,
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
./spaceplane/tools/multirot.sh TAG 3 "save|SET=1;SET2=2" ...   # arms rotated over the farm, restarted every 2 rounds
./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 2 logs/rot-X.txt "$PWD" "save|SET=1" ...  # one farm cycle
./spaceplane/tools/entrysave.py 0                     # quicksave mid-flight (interface, or --alt on final)

./spaceplane/tools/landsum.py logs/LOG88*   # arrival -> cone surplus -> wheels, one line per flight
./spaceplane/tools/oscsum.py logs/LOG88*    # alpha error, sideslip, per phase
./spaceplane/tools/armsum.py --by X --against '(default)' logs/LOG88*   # one line per arm
./spaceplane/tools/conesum.py --settled logs/LOG88*
./spaceplane/tools/conepolar.py             # the cone's flown polar -> logs/conepolar/
./spaceplane/tools/splitprobe.py 0          # the split rudder's drag, yaw, roll

python3 -m unittest spaceplane.tests.testSpaceplane   # offline, no KSP
./testInstances/actuators.py 0               # every module setting of the craft
```

**Method notes.** Code is frozen during a batch (`rotfly.sh` runs the
tree): develop in a git worktree. Label arms from each log's `config:`
line. `pgrep -f` in a wait loop matches itself.
