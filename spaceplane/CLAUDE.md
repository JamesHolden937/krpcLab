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
| [docs/spaceplane/journal.md](../docs/spaceplane/journal.md) | what each session measured, oldest first -- **append new sessions here**. The last section is the long-form standing summary this file condenses |
| [docs/spaceplane/design.md](../docs/spaceplane/design.md) | the runway, the airframe measurements, the architecture phase by phase, entry states |
| [docs/spaceplane/failures.md](../docs/spaceplane/failures.md) | **before undoing anything** -- numbered failures, cited everywhere by number |

## Priority: the shuttle, not the old craft (the user, 2026-10-03)

**The old craft (`qs_plane`) is a flying brick with no real wings. Don't
spend effort on it.** If it lands badly, assume the airframe's design is the
cause, not the autopilot. Don't tune for it. Fly it only as a regression
check, to confirm a change didn't break something that used to work. The
shuttle (`qs_shuttle2`) is the target. One thing was seen and not chased on
2026-10-03: the old craft's rollouts veer 100-200 m sideways once the nose
comes down at ~33 m/s (LOG5080/5081: xt -6 -> -188 m in 10 s). The shuttle
also stops 300-480 m off the centreline (LOG5075, 5079), so the rollout
steering is worth a look **on the shuttle**.

## Where it stands (2026-10-03 evening, fingerprint `6e854d9f`)

See the root HANDOFF.md and the journal's last section. In short, the
shuttle now lands on the centreline: every stop from the cone saves is
within ~100 m, and from orbit within 181 m. It still lands **long**,
because `TOUCHDOWN_AIM_M` = 2400 is the far threshold. Its touchdowns are
often a stall after a float. The old craft lands 6/6 intact on the strip
from orbit. **Fixed this session:** the nosewheel steered away from the
centreline (left-handed `across`). The approach's lateral capture relayed
against the shuttle's 5.3 s roll. The cone didn't count its entry speed as
energy.

## Where it stood (2026-09-23, fingerprint `64268a58`)

> **Superseded by the root [HANDOFF.md](../HANDOFF.md)** for the latest
> session (2026-09-23 evening): the shuttle's attitude axes, measured
> spoiler/flaps, and the cone as the blocker. The table below is the state
> before that session.

| | arrival at the cone | landing |
|---|---|---|
| old craft, `qs_plane` | +0.1..+0.8 km, pinned at the cone's saturation | usually intact; loses the vehicle when the touchdown is late |
| old craft, `qs_plane_inc` | -0.3..-0.7 km | **breaks up on contact ~11 of 12** (failure 81, unchanged) |
| shuttle, `qs_shuttle` | **+3.2 km sd ~1** (was +111) | **does not land** -- out of height in the cone, flare at 20-94 m/s sink |
| shuttle, `qs_shuttle_inc` | +3.4 km | same |
| shuttle, `qs_shuttle_high` | +6..+12 km (n=2) | same |

Verified on the committed defaults, one flight each
(`logs/ladder-final-defaults.txt`): `qs_plane` cone -0.3 km, landed intact at
+861 m, 12 m off the centreline (LOG3013); `qs_shuttle` cone +5.0 km, burn
closed to 0.00 m/s at limiter 0.14, destroyed in ROLLOUT (LOG3014);
`qs_shuttle_inc` cone +3.7 km, destroyed in ROLLOUT (LOG3015). **Entry and
deorbit work on both craft. Landing works on the old craft on `qs_plane`
only.**

The last full characterisation of `qs_plane`'s landing (fingerprint
`6debf0c5`, two 8-flight batches): n=16, intact 88%, on the runway 88%, on the
strip 75%, stopped along **+939 m sd 455** of a +-1200 m runway. **It lands
late.** Already measured against that and rejected: the aim (failures 68, 84),
the S-turn stop as a time (84), the gear's spring and damper (83); the brake
law and bank compensation were kept (82).

## Next, in order

**Start from the root HANDOFF.md** (2026-10-07 late: the user's split-S -- `APPROACH_SHARP_TURN` spends surplus as drag in the S-turn, `HAC_SPIRAL_DUMP` flies measured tight 360s over the gate; both off, the cone saves roll out 4-6.8 km high with the canard trim and a spiral lap costs ~3.9 km). Before that (2026-10-07 evening: kRPC holds the standing trim in integrators a spiral twists; `CANARD_TRIM` moves it to the canards -- rigoff 31/48 on the runway, but inc/high land long and the eccentric orbit arrives 41 km long; crossrange measured). Before that (2026-10-07 morning: the missing control was the propellant -- `PROPELLANT_TRIM` pumps the nose fuel aft and frees the elevons, but the trim drag was the only speedbrake, so every pump variant lands worse than the offload; open as an energy control). Before that (2026-10-07 early: the glide's long mode is kRPC's roll-invariant pitch integrator; `GLIDE_PITCH_OFFLOAD` fixes the arrival and survival, landings now set by the energy chain and the cone's subsonic speed collapse). Before that (2026-10-06 evening: the glide plans an alpha the shuttle cannot hold -- measured drag 0.59-0.80 of predicted at 38-32 km on every long flight; `HOLDABLE_PRIOR` fixed the arrival position but not the energy, off). Before that (day): the blocker was
`qs_shuttle2_rigoff`'s bimodal cone arrival, traced to the glide's learned
alpha ceiling; the rollout weave is fixed (`ROLLOUT_STEER_LEAD_S`).

0. **Replace `TOUCHDOWN_AIM_M` = 1800 with a per-vehicle derivation**
   (open since 2026-10-06; the user asked it be kept for now and fixed
   later). 1800 was fitted on the shuttle alone (9/18 vs 3/18 at 2400,
   rot-aim-1006) and is a ship bias: the old craft on it lands 0-1/12
   **1.2-1.5 km short of the threshold**, intact (corrected 2026-10-06:
   the earlier "rolls off the end" read an unsigned distance; the signed
   stops are -1.4..-2.6 km from the midpoint, rot-bank-1006) -- its
   approach falls ~3 km short of the aim, so the shuttle-fitted
   `APPROACH_BEST_LD` 4.2 is the first suspect. `APPROACH_AIM_SHIFT_M` 1000 is the same kind of
   constant and goes with it. Derive the aim so that aim + flare float
   (door speed against stall, L/D, from the table) + rollout (touchdown
   speed^2 / 2 x braking deceleration) fits the runway; log predicted
   against flown float and rollout; fly derived vs 1800 on the farm,
   interleaved, on the three shuttle orbits **and** `qs_plane`. Accept
   only if it holds on both craft. kspSim cannot screen it (its landings
   do not follow the game, kspSim gap 7).

1. **The opposed-flap brake and the speed loop sharing the surplus.** The
   user's brake works: wired (`AIRBRAKE_OPPOSED_FLAPS`, disconnected until
   2026-09-23) it moved the old craft's touchdown **~600 m earlier**, the
   first thing that ever has, and broke 3 of 6, reaching the flare at 38-58
   m/s of sink against the ~39 the flare can arrest -- because
   `APPROACH_SPEED_PATH` dives to win back the speed the brake takes.
   Deploy only with speed above target by what the brake will spend, or feed
   its drag into the speed path's `dv/dt = g sin(theta) - D/m`. Read the
   *touchdown point* (`rwy=` on the first ROLLOUT line), not the stop -- the
   stop is the brake law's own +900 m aim.
2. **The shuttle's landing chain, from a final-approach save**
   (`spaceplane/tools/entrysave.py N --alt 3000 --name qs_shuttle_final` during a shuttle
   flight), not from orbit. Its L/D at landing mass is 2.27 at 59 m/s; the
   chain (`HAC_LD`, stall, approach L/D) is sized on the old craft. It also
   wallows +-12 deg of sideslip in the cone. `AIRFRAME_DERIVED` (the table's
   own numbers) was flown and made it worse; it needs a design session, not a
   constant swap.
3. **`qs_shuttle_high`**: the deorbit window called the gate reachable and the
   glide, pinned at both caps, arrived 6-12 km long. A lead on the reach model.
4. **The audit** (root CLAUDE.md, "Every few batches..."). On 2026-09-23 it
   found the surfaces error, the never-commanded thrust limiter, the
   disconnected brake and a frozen kRPC state. Run
   `testInstances/actuators.py` on any new craft first.

**Refuted on 2026-09-23 -- do not re-fly without reading why** (journal,
"Session, 2026-09-23"): `ATTITUDE_TIME_TO_PEAK_LIVE` (-42 km, 2.5x sideslip;
engine torque in the total and a chattering surface torque),
`AIRFRAME_DERIVED` on the shuttle (+9.7 km, 132 m/s into the flare),
`AIRBRAKE_SINK_GUARD` (never deploys: the unbraked approach already sinks
46-49 m/s).

## Standing facts (each paid for; the journal has the evidence)

- **Roll and yaw are one lateral axis at high alpha** (failure 99). kRPC
  roll at its default 1.0 s ("full authority") lost the shuttle's entry
  5/5: at 35 deg of alpha a body roll *is* sideslip, and yaw on 22.6 s
  never takes it out. Roll flies the derived 4.8 s. Yaw on roll's figure
  cures the hypersonic slip and loses the bank in the cone; yaw scheduled
  by sin(alpha) tumbled 2/6. Tune the lateral axes together, and fly any
  change to them on the farm first -- the user flies defaults live.

- **The control surfaces are live on every axis, on both craft.**
  `ignorePitch = False`; 95-276 kN m in flight. An old probe read KSP's
  *ignore* flag as an enable and measured torque on the pad at q=0.
  `ENABLE_CONTROL_SURFACES` is a no-op; the alpha ceiling falling with q is a
  *trim* limit, not wheels losing to the air.
- **Five flags became defaults on 2026-09-23** (+111 -> +3.2 km on the
  shuttle, each null on the old craft in a paired batch):
  `ATTITUDE_TIME_TO_PEAK_DERIVED`, `DEORBIT_FLOOR_ON_TICK` (the burn priced
  re-pointing dead time as thrust and quit 2 m/s short on 15/15 shuttle
  burns), `DEORBIT_MIN_BURN_S=3` (the engine **thrust limiter**, never
  commanded before), `HOLDABLE_SKIP_REVERSAL` and `RATCHET_SKIP_REVERSAL`
  (bank reversals were teaching a false alpha ceiling -- 30 of the km).
- **Laws that replaced constants** (failures 65-72): `APPROACH_SPEED_PATH`
  (the speed loop could only slow down; it now commands the descent angle
  that holds the speed, `dv/dt = g sin(theta) - D/m`, flown with load against
  the descent the vehicle has -- open-loop it ran away); `FLARE_SINK_TRACK`
  (tracks `sqrt(td^2 + 2 a h)`); `AIM_RUNWAY_TRUE_ALPHA` (pitch to
  `alpha + descent`, not alpha); `GLIDE_RESERVE_M` 4000 -> 500 (a margin is a
  fitted constant with a standard deviation baked in); `HAC_ROLLOUT_M` 1500 ->
  900 (the roll-out distance sets the cross-track the flare inherits).
- **The time scale serves the control interval, not the other way round.**
  `--timescale` is a ceiling; `Config.TIMESCALE_GOVERNOR` holds it to what
  each phase's loop can serve, and every log ends with a `loop rate` line.
  Read it before comparing two logs, instead of subtracting timestamps.
  `GOVERN_ON_PEAK` (default) governs on the phase's worst tick: that was
  `qs_plane_inc`'s whole bimodality (failure 91). Bimodal results? Sort by
  the loop-rate line before theorising.
- **The deorbit aim is geared down 6.4:1.** The window moves -19.7 km per m/s
  of dv and the arrival -3.06 km per m/s, so one metre of arrival costs 6.4 m
  of `DEORBIT_CENTRE_BIAS_M`. Every earlier null on the aim used too little
  (failures 88-89). Compute a knob's gain before believing a null on it.
  `qs_e60` lands with `--set DEORBIT_CENTRE_BIAS_M=51000` (16 of 16 on the
  runway, 14 intact); **51000 must never become a default** -- on `qs_plane`
  it is 6-7 km short. `TOUCHDOWN_AIM_M` is not a lever on where the wheels
  touch (a nearer aim is a larger surplus, spent by flying further).
- **`DRAIN_BEFORE_BURN`** (failure 61, default) is what closed the generality
  gap: the drain valve fired after the burn's closed loop stopped looking.
- **The reversal count is a saturation symptom, not a range term** -- its
  sign differs between saves (+2137 m per reversal on `_inc`, negative on
  `qs_e60`). `ALPHA_TRACKING_ON` (77), `GLIDE_BANK_DUTY_ON` (79) and both
  together (85) all measured worse (failure 87).
- **Landing work cannot be measured from an orbital save.** Fly it from
  `spaceplane/tools/entrysave.py` saves (`ENGAGE_INTO_LANDING` takes over on final).
  `GLIDE_RESERVE_M=4000` is worth +5.0 km of arrival on `qs_plane_inc` but
  would land `qs_plane` later still; not committed.
- **Sideslip spoils lift without adding drag** on the old airframe (L/D 1.64
  -> 1.23 at 15 deg). `APPROACH_SLIP_FOR_ENERGY`: -473 m and half the scatter
  on `qs_plane`, did **not** reproduce on `qs_plane_inc`; default off.
- **Built, offline-tested, unflown** (default off): `ENTRY_MAX_DRAG` (fly the
  hot entry at the most alpha `Holdable` says the airframe holds) with
  `DEORBIT_SHALLOWEST` (smallest burn that still captures; dv 43.2 -> 37.0
  offline). Shallowness is bound by `DEORBIT_MAX_TIME_TO_GO_S`=1500, not the
  skip -- raising it is its own arm (removing it once cost 95-132 km).
- **Fly every change on both craft**, and read `oscsum.py` as well as
  `landsum.py`: the shuttle wallowed 96 deg of sideslip at Mach 7 while every
  other summary reported only the 200 km it cost.
- **Batch sizes:** an 8-flight batch on this vehicle resolves a factor of two
  and nothing finer (4/8 and 7/8 from one configuration in consecutive
  batches). Quote a rate with its n; prefer `spaceplane/tools/pairfly.sh`, whose
  interleaving protects a comparison when the absolute level drifts.

**The old airframe** is a capsule with wings: best glide L/D 3.06 at 8 deg,
stall 52.4 m/s, tail strike at 18.7 deg of body attitude, holds its commanded
angle of attack. Entry range is non-monotone in bank (1730 km at 0 deg, 1872 at
30, 1432 at 70) and in entry alpha. **The shuttle** (Mk3, 30 parts, 37.5 t,
seven surfaces) glides at L/D 5.91. Saves: [saves/README.md](../saves/README.md).

## Commands

```bash
./run.sh --pilot spaceplane                          # fly it in your game, waiting for START
./spaceplane/tools/quickglide.py -n 3 --instance 0 --timescale 6   # fly a save on the farm
./spaceplane/tools/quickglide.py -n 1 --save qs_plane_inc --set GATE_ALT_M=1600
OUT=/tmp/base.txt N=8 ./spaceplane/tools/farmfly.sh   # one config, n per instance
./spaceplane/tools/pairfly.sh                         # two arms, interleaved round by round
./spaceplane/tools/ladder.py --arms qs_l0,qs_l16,qs_l24 -n 4   # several saves across the farm
./spaceplane/tools/entrysave.py 0                     # quicksave mid-flight (interface, or --alt on final)

./spaceplane/tools/landsum.py --arrivals-within 5000 logs/LOG16*  # arrival -> handover -> wheels
./spaceplane/tools/oscsum.py logs/LOG287*   # was it pointed? alpha error, sideslip, per phase
./spaceplane/tools/armsum.py --by X --against '(default)' logs/LOG10*   # one line per arm, error bars
./spaceplane/tools/conesum.py --settled logs/LOG16*   # what HAC_LD should be
./spaceplane/tools/glidesum.py --band 33000 26000 logs/LOG7*   # bank reversals, per flight
./spaceplane/tools/slipsum.py logs/LOG282*  # the sideslip the airframe holds, against q
./spaceplane/tools/polar.py --mach 0.45     # the polar the airframe flies, from the logs
./spaceplane/tools/aeroaudit.py logs/LOG601 # model against flight

python3 -m unittest spaceplane.tests.testSpaceplane   # offline, no KSP
python3 -m spaceplane.tests.glidesim --dv 60           # one entry, traced
./testInstances/actuators.py 0               # every module setting of the craft
./testInstances/planeprobe.py 0 --mass 6.715 # probe it -- over-reads subsonic lift ~1.8x (failure 13)
```

**Method notes.** Code is frozen during a batch: develop in a copy of the tree
(`spaceplane common testInstances/planeprobeGearup.txt`). `farmfly.sh` will not
share the farm and writes `BATCH DONE` to its OUT file; `ladder.py` arms
carrying settings are separated by `;`; `pgrep -f` in a wait loop matches
itself. "Inert by construction" claims taken from comments were wrong twice
this project (the old engine is 17.8 m/s^2, not 8.6) -- re-run the tool.
