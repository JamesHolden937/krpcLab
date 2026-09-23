# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## HANDOFF, 2026-09-23 -- READ THIS, ACT ON IT, THEN DELETE THIS SECTION

**Delete this section once you have read it and folded anything lasting into
the docs.** CLAUDE.md is loaded whole into every session. The full account is
docs/spaceplane.md, "Session, 2026-09-23" and everything after it.

### Does it work? Where each craft stands

| | arrival at the cone | landing |
|---|---|---|
| old craft, `qs_plane` | +0.1..+0.8 km, pinned at the cone's saturation | usually intact; loses the vehicle when the touchdown is late |
| old craft, `qs_plane_inc` | -0.3..-0.7 km | **breaks up on contact ~11 of 12** (failure 81, unchanged) |
| shuttle, `qs_shuttle` | **+3.2 km sd ~1** (was +111) | **does not land** -- out of height in the cone, flare at 20-94 m/s sink |
| shuttle, `qs_shuttle_inc` | +3.4 km | same |
| shuttle, `qs_shuttle_high` | +6..+12 km (n=2) | same |

**Verified on the final committed defaults** (`64268a58`, one flight each,
`logs/ladder-final-defaults.txt`): `qs_plane` cone -0.3 km, landed intact
at +861 m, 12 m off the centreline (LOG3013); `qs_shuttle` cone +5.0 km,
burn closed to 0.00 m/s at limiter 0.14, destroyed in ROLLOUT (LOG3014);
`qs_shuttle_inc` cone +3.7 km, destroyed in ROLLOUT (LOG3015). **Entry and
deorbit: working on both craft. Landing: the old craft on `qs_plane` only.**

### What changed, all measured on both craft

- **A headline fact was false.** The old craft's control surfaces are live
  on every axis (`ignorePitch = False`; 95-276 kN m in flight). The old probe
  read KSP's *ignore* flag as an enable and took torque on the pad at q=0.
  `ENABLE_CONTROL_SURFACES` is a no-op; the alpha ceiling is a *trim* limit.
- **Five flags promoted to defaults** (fingerprint `64268a58`), each null on
  the old craft in a paired batch and together worth +111 -> +3.2 km on the
  shuttle: `ATTITUDE_TIME_TO_PEAK_DERIVED`, `DEORBIT_FLOOR_ON_TICK` (the
  burn's floor rule charged re-pointing dead time as thrust; 15/15 shuttle
  burns quit 2 m/s short, 0/870 old-craft ones), `DEORBIT_MIN_BURN_S=3` (the
  engine **thrust limiter**, never commanded before), `HOLDABLE_SKIP_REVERSAL`
  and `RATCHET_SKIP_REVERSAL` (bank reversals were teaching and ratcheting a
  false alpha ceiling -- that one flag was 30 of the km).
- **Your opposed-flap brake** was disconnected (armed, never deployed). Wired:
  subsonic lift spoiler (L/D 2.41 -> 2.11 at 15 deg, moment cancels),
  hypersonic *anti*-brake, and it moved the old craft's touchdown **~600 m
  earlier** -- first thing that ever has -- while breaking 3 of 6, because
  the approach's speed loop dives to win back the speed the brake takes.
- **Instruments:** every log has `actuators:` at STANDBY and `authority kN m
  ... at q=` per phase; `testInstances/actuators.py` dumps a craft's every
  module setting; `STATE_FROZEN_S` ends a flight whose kRPC state has frozen
  (one did for 50 minutes).

### Refuted this session -- do not re-fly without reading why

`ATTITUDE_TIME_TO_PEAK_LIVE` (-42 km, 2.5x sideslip; engine torque in the
total and a chattering surface torque), `AIRFRAME_DERIVED` on the shuttle
(+9.7 km, 132 m/s into the flare), `AIRBRAKE_SINK_GUARD` (never deploys: the
unbraked approach already sinks 46-49 m/s at altitude).

### Next, in the order I would take them

1. **The brake and the speed loop sharing the surplus.** The user's
   mechanism works and is one design away: deploy only with speed above
   target by what the brake will spend, or feed its drag into
   `APPROACH_SPEED_PATH`'s `dv/dt = g sin(theta) - D/m`. Read on the
   *touchdown point* (`rwy=` on the first ROLLOUT line), not the stop -- the
   stop is the brake law's own +900 m aim.
2. **The shuttle's landing chain, from a final-approach save** (`entrysave.py
   N --alt 3000 --name qs_shuttle_final` during a shuttle flight), not from
   orbit. Its table's L/D at landing mass is 2.27 at 59 m/s; the chain is
   sized on the old craft. It also wallows +-12 deg of sideslip in the cone.
3. `qs_shuttle_high`: the deorbit window called the gate reachable and the
   glide, pinned at both caps, arrived 6-12 km long. Lead on the reach model.
4. The audit rule below ("Every few batches..."): this session it found the
   surfaces error, the thrust limiter, the disconnected brake and the frozen
   state. Run `actuators.py` on any new craft first.

### Method notes

- Code frozen during batches; develop in a copy of the tree
  (`cp -r spaceplane tests boosterland` + `testInstances/planeprobeGearup.txt`).
- `farmfly.sh` will not share the farm and writes `BATCH DONE` to its OUT
  file; `ladder.py` arms carrying settings must be separated by `;`;
  `pgrep -f` in a wait loop matches itself.
- "Inert by construction" claims came from comments and were wrong twice
  (the old engine is 17.8 m/s^2, not 8.6). Re-run the tool.

### Machine state as left

Farm stopped, sleep inhibitor released. Logs LOG2906-3015; batch
files listed at the end of docs/spaceplane.md.

## What this is

Two kRPC autopilots in one tree, sharing `boosterland.vec`, `boosterland.logbook`,
the `testInstances/` measurement farm, and every convention below.

- **`boosterland/`** flies a KSP booster back to the KSC launchpad after stage
  separation, Superheavy-style: boostback burn, ballistic coast, suicide burn,
  touchdown. Target pad lat `-0.097162`, lon `-74.557679`
  (`Config.PAD_LAT/PAD_LON`). It lands within a few metres of the pad and is
  largely settled work.
- **`spaceplane/`** deorbits a winged vehicle, flies an atmospheric entry and
  lands it on the KSC runway.

  **Read `docs/spaceplane.md` "Session handoff, 2026-09-20" first** -- the
  committed configuration's measured performance, six mechanisms refuted in
  game (do not re-try them), and the two open problems with their numbers.
  Then the two 2026-09-21 handoffs -- but **the second one's headline fact
  is false**: it said this craft "has no aerodynamic control at all, 0 of 6
  surfaces with any axis enabled". **Every surface on both craft has pitch,
  yaw and roll enabled** (`ignorePitch = False` in the save, `pitch_enabled`
  True through kRPC, and switching them off changes the vehicle's response to
  a pitch input from 0.14 rad/s in a second to nothing, wheels off). The
  probe read the part menu's `Pitch=False`, which is KSP's *ignore* flag, and
  `available_torque` on the pad, where q=0. The alpha ceiling falling with q
  is a *trim* limit (surfaces and airframe moments both scale with q, the
  wheels do not), not wheels losing to the air. docs/spaceplane.md,
  "Session, 2026-09-23".
  Then **"Sideslip, measured end to end"** and **"The slip has two
  authorities"**: the approach's +939 m overshoot is answered by flying it at
  a sideslip (`APPROACH_SLIP_FOR_ENERGY`, **-473 m and half the scatter over
  9 an arm, still default off**), because slip on this airframe spoils lift
  without adding drag -- L/D 1.64 -> 1.23 at 15 degrees, airspeed untouched.
  **It did not reproduce on `qs_plane_inc`** (+913 against +497, 6 an arm):
  the scatter halving carried over, the bias did not, so it stays off.
  The latest state -- second airframe, derived attitude tune, high-alpha
  ceiling per craft, and the queue of built-but-unflown mechanisms -- is
  docs/spaceplane.md, "Session handoff, 2026-09-22 (evening)".
  Then "The farm was measuring its own tick latency, in one phase". The farm's `--timescale 6` multiplies every tick's
  wall-clock cost into game time, and the deorbit burn's tick propagates: at
  6x it closes its throttle loop every **0.30 game-seconds** instead of 0.10.
  The time scale is now the free variable and the *control interval* is the
  constraint — `Config.TIMESCALE_GOVERNOR` asks for the interval the phase
  wants, measures what a tick costs, and commands the fastest scale that
  serves it. Throughput is unchanged (5.0-6.0x). Measured against itself,
  one save, committed defaults, same farm state, `--no-govern` the only
  difference: the arrival at `GLIDE -> HAC` goes **-3226 m sd 806 (n=6) to
  +294 m sd 171 (n=9)**. **Every phase but the burn was already being
  served** — a tick on final costs 3-6 ms — so this buys one thing in one
  place, and it is 3.5 km of bias and 5-8x of scatter.
  Every log now ends with a `loop rate` line saying what each phase actually
  got. **Read it before comparing two logs, and read it instead of
  subtracting log timestamps** — those are `LOG_INTERVAL_UT`, which is the
  trap failure 63 recorded and failure 65 fell into anyway.

  **Where it stands, `qs_plane`, eighteen flights of one configuration --
  which is now the committed default, so there is nothing to pass:**

  ```bash
  .venv/bin/python -m spaceplane.autopilot          # waits for START
  ./quickglide.py -n 3 --instance 0 --timescale 6   # or fly the quicksave
  ```

  **Re-measured, two independent eight-flight batches of this exact
  configuration (fingerprint `6debf0c5`), which agreed at 7 of 8 each:**

      n=16   intact 14 (88%)   on the runway 14 (88%)   on the strip 12 (75%)
      stopped along  +939 m  sd 455        (the runway is +-1200 m)

  **It lands late** -- +939 of a +-1200 m runway is the last quarter of the
  tarmac, and a flight that stops at +1737 has rolled off the end. That is
  the open item, and four things have now been measured against it and
  rejected: the aim (failures 68 and 84), the S-turn stop as a time
  (84), the brake law and bank compensation (kept -- 82), and the landing
  gear's spring and damper in both directions (83). Do not re-try those
  without reading why.

  **Two cautions on numbers like the ones above.** An eight-flight batch on
  this vehicle resolves a factor of two and nothing finer -- the same
  configuration gave 4 of 8 and 7 of 8 in consecutive batches (83) -- so
  quote a rate only with its n, and prefer `pairfly.sh`, whose round-by-round
  interleaving protects a *comparison* even when the absolute level drifts.

  **The laws that changed, each replacing a constant rather than re-fitting
  one** (failures 65-72):
  - `APPROACH_SPEED_PATH` -- the approach's speed loop could only ever *slow
    down*, because its floor was one g and one g is a pull-up on a vehicle
    descending at 18 degrees. It now commands the descent angle that holds
    the speed (`dv/dt = g sin(theta) - D/m`) and flies it with load, against
    the descent the vehicle *has* -- the inner loop is not optional, open-loop
    it ran away to 124 m/s and 72 m/s of sink.
  - `FLARE_SINK_TRACK` -- the flare arrested to level at 50 m, floated four
    seconds and fell the rest. It now tracks `sqrt(td^2 + 2 a h)`, the sink
    it can still arrest in the height left.
  - `AIM_RUNWAY_TRUE_ALPHA` -- `aim_runway` flew a pitch attitude where every
    caller computed an *angle of attack*, and the two differ by the descent
    angle, so the wing was handed extra lift in exact proportion to how fast
    the vehicle was coming down. It now pitches to `alpha + descent`. This is
    what was sustaining the flare's float.
  - `GLIDE_RESERVE_M` 4000 -> 500 -- the reserve was three sigma of an entry
    that scattered kilometres, and the entry now scatters 171 m. **A margin
    is a fitted constant with a standard deviation baked into it.**
  - `HAC_ROLLOUT_M` 1500 -> 900 -- the cone's roll-out distance is what sets
    the cross-track the flare inherits (`gate=778 -> cross=-37`,
    `gate=1324 -> cross=+206`), and that offset is what costs the wing. Two
    sessions on the capture, the flare's bank taper and the touchdown alpha
    moved none of it; one constant in the phase above them moved all of it.

  **Do not re-raise, and do not read the older material as current:**
  `DEORBIT_CENTRE_BIAS_M` is inert (the glide absorbs it -- 7.3 km of aim
  moved the arrival 81 m); `TOUCHDOWN_AIM_M` is not a lever on where the
  wheels touch (2400 -> 400 landed it 900 m *later*, because a nearer aim is
  a larger surplus and the approach spends surplus by flying further). The
  generality result that made all this reachable is failure 61's
  `DRAIN_BEFORE_BURN` -- the release valve fires after the burn's closed loop
  stops looking -- and it is the default.

  **The other two entry states.** See docs/spaceplane.md, "Session handoff,
  2026-09-21", which corrects the 2026-09-20 one in two places.

  **`qs_e60` (a 157 km apoapsis) now lands** -- failure 80 had it 0 of 4 onto
  the runway, "what grows is the scatter, not the bias, so no margin reaches
  it". `pairfly.sh`, 8 an arm, fresh farm:

  | | stopped along | on the runway | intact |
  |---|---|---|---|
  | `--set DEORBIT_CENTRE_BIAS_M=51000` | **+849** (+203..+1162) | **8 of 8** | **8 of 8** |
  | committed defaults | +1189 (-4777..+4192) | 0 of 8 | 0 of 8 |

  -- and over two independent batches of that configuration, **16 flights,
  16 on the runway, 14 intact**. The arms' mean *distance* barely differs:
  the defaults fail on scatter, not bias. Failures 88-89, 93.

  **The aim was never disconnected; it is geared down by a factor nobody had
  measured.** The window moves -19.7 km per m/s of dv and the arrival moves
  -3.06 km per m/s, so **one metre of arrival costs 6.4 m of
  `DEORBIT_CENTRE_BIAS_M`** -- and every null ever recorded against the aim
  (failures 33, 68, 84, 86) used 2.5-17 km, which buys 0.4-2.6 km and hides
  under an eight-flight batch. **Compute an aim knob's gain before believing
  a null on it**; it is one offline command. Failures 88-89.

  The *target* is not fitted: bias until the arrival lands on
  `GLIDE_RESERVE_M` (+500), which is where the cone's surplus saturates
  positive. Only the bias is per-state, and **51000 must never become a
  default** -- on `qs_plane`'s 32 m/s burn it is 2.2 m/s and 6-7 km short.

  **`qs_plane_inc`'s bimodal arrival is fixed, and it was the deorbit's own
  loop rate.** Sorted by the achieved `DEORBIT` interval, every flight of
  that save on disk splits at 0.22 game-s/tick with nothing between:
  **0.10-0.21 arrives within 2.6 km, 0.24-0.32 arrives 6.5-25 km short.**
  `ScaleGovernor` keeps a decaying *maximum* because "what binds is the
  tail" -- and was being fed `LoopRate.busy`, an exponential *average*. Two
  filters in series, and the second never sees what the first removed; the
  phase's cheap waiting ticks then ramp the scale to 6x and the burn ignites
  there. **`GOVERN_ON_PEAK` (now the default) governs on the phase's worst
  tick, undecayed**: `qs_plane_inc` arrival **+199 sd 420** against -4378
  sd 5369 with the coarse mode gone, `qs_plane` 8 of 8 on the runway against
  7 of 8 with a five times tighter spread, for 4-5% of throughput.
  Failure 91. Asking for the interval earlier does *not* fix it
  (`DEORBIT_PACE_WHOLE_PHASE`, flown, null) -- the interval was never the
  problem, the cost estimate was.

  **What is still open on `qs_plane_inc` is the touchdown**: it comes apart
  on contact on most flights, from flare states indistinguishable from
  `qs_plane`'s, which is failure 81 and a craft problem. `GLIDE_RESERVE_M`
  500 -> 4000 is separately worth +5.0 km of arrival on that save (measured,
  8 an arm) but is not committed -- on `qs_plane` it would land later still.
  The reversal count
  correlates with the arrival on every save but **its sign is not the same
  on all of them** (+2137 m per reversal on `inc`, negative on `qs_e60`), so
  it is a symptom of the glide saturating -- pinned at bank 0.9 when short,
  at bank 70 when long -- and not a range term. Three arms have now been
  spent modelling it as one and all three measured worse: `ALPHA_TRACKING_ON`
  (77), `GLIDE_BANK_DUTY_ON` (79), both together (85). Failure 87.
  It also destroys the vehicle on contact 8 of 8 from flare states
  indistinguishable from `qs_plane`'s; **landing work still cannot be
  measured from an orbital save**, so fly it from `entrysave.py`, which
  `ENGAGE_INTO_LANDING` lets you take on final.

  **Two mechanisms are built, offline-tested and unflown** (all default
  off; docs/spaceplane.md, "Flying the entry for drag, and buying the burn
  with it"). `ENTRY_MAX_DRAG` flies the hot entry at the most angle of
  attack `Holdable` says the airframe holds -- up to true broadside, five
  times the drag of the commanded 22 degrees and **zero** lift -- and
  `DEORBIT_SHALLOWEST` takes the smallest burn that still captures, found by
  bisection on a monotone predicate rather than a grid on a non-monotone
  range. They are one design: broadside cannot skip, and it is lift that
  makes a shallow entry bounce. Offline, same arrival, **dv 43.2 -> 37.0
  m/s**. The targeting moves to the ignition *time*, and where the entry
  stops braking is **solved at the burn** (`guidance.drag_switch_for`)
  because the entry's length is what decides where the burn goes -- a fitted
  Mach number there would be the worst constant in the file.

  Two cautions carried from that work. **The shallowness is bound by
  `DEORBIT_MAX_TIME_TO_GO_S` = 1500, not by the skip** -- raising it reaches
  ~12 m/s and is the next arm, not a silent edit (removing it once cost
  three flights and 95-132 km). And **the drag/lift switch is nearly inert
  on this vehicle**: the holdable ceiling collapses from 90 to 25 degrees by
  45 km while the vehicle is still doing 2100-2400 m/s, so the plant makes
  the handover and the parameter is worth 224 km of a 2618 km arc. That is a
  fact about this craft's *trim* ceiling (its surfaces are live -- see the
  correction above), and it is the clearest argument yet for flying a
  different airframe.

  **A second airframe exists and it changes the priorities.**  The user's
  Mk3 shuttle (`qs_shuttle` in the farm; 30 parts, 37.5 t, seven control
  surfaces, every axis enabled -- as on the old craft, whatever older text says -- best
  glide L/D 5.91 against 3.06) found **five** things fitted to one vehicle,
  in an afternoon, none of them visible to 566 offline tests or ~2800
  flights: the drain valve's `Drain Mode` was never commanded (right by
  inheritance); `Module.set_field_value` does not exist in this kRPC and was
  the documented fallback in both drain paths; `vacuum_specific_impulse`
  reads 0 on an unlit vessel so the drain reserve silently took a floor and
  the burn **ran dry 9 m/s short of its own solution**; `GLIDE_RCS` is off
  on the strength of a measurement taken on a craft whose RCS torque is
  literally `(0, 0, 0)`; and `ATTITUDE_TIME_TO_PEAK_S` -- fitted to "a 2.4 s
  lateral mode" this craft does not have -- is worth **198 km** of arrival
  on it.  The first three are fixed.  See docs/spaceplane.md, "A second
  airframe, and the five things it found in an afternoon".
  **Fly every change on both craft from here.**  `oscsum.py` is the new
  reader and the one that found it: alpha error, its spread and peak-to-peak
  sideslip per phase, because nothing in the tree reported whether the
  vehicle was *pointed* -- the shuttle wallowed **96 degrees** of sideslip at
  Mach 7 and every existing summary reported only the 200 km it cost.

  **The shuttle's arrival, 2026-09-23** (docs/spaceplane.md, "Session,
  2026-09-23"): **+111 km sd 9 -> +3.4 km sd 0.8** at the cone, n=6 an arm,
  from four changes, **all four now the defaults**, each flown and cleared
  on both craft (fingerprint `83c8c65f`, which also makes
  `ATTITUDE_TIME_TO_PEAK_DERIVED` the default -- null on the old craft,
  -697 vs -686 m on qs_plane_inc, n=6):
  - `DEORBIT_FLOOR_ON_TICK` (+111 -> +33): the burn's floor rule priced
    alignment dead time as thrust, and on a 65 m/s^2 engine quit 2 m/s short
    on 15 of 15 shuttle burns (0 of ~870 old-craft burns);
  - `DEORBIT_MIN_BURN_S=3` -- the engine **thrust limiter**, never commanded
    before -- and `HOLDABLE_SKIP_REVERSAL` (+33 -> +34, sd 12 -> 3.3);
  - `RATCHET_SKIP_REVERSAL` (+34 -> +3.4): `ratchet_alpha` backed the alpha
    ceiling off during every bank reversal and a saturated glide could not
    win the range back.
  Cleared on the old craft: the first three on `qs_plane` (+477 vs +426,
  n=6), the ratchet gate on `qs_plane_inc` (-558 vs -346, n=6).
  `ATTITUDE_TIME_TO_PEAK_LIVE` was flown and **refuted** (-42 km, 2.5x the
  sideslip wallow). The shuttle's frontier is now its landing chain, which
  still runs on the old airframe's `HAC_LD`, stall and approach L/D --
  and `AIRFRAME_DERIVED` (the table's own) was flown and made it **worse**
  (+9.7 km arrival, 132-135 m/s into the flare). It needs a design session
  from a final-approach save, not a constant swap.
  Every log now carries `actuators:` at STANDBY and `authority kN m ... at
  q=` on each phase change, and `testInstances/actuators.py` dumps every
  module setting of a craft -- run it on any new airframe first.
  `qs_shuttle_inc` and `qs_shuttle_high` exist on the farm (`savegen.py`,
  normal 100 / prograde 80): the same configuration arrives **+3.4 km**
  (n=4) on the inclined orbit and **+6 / +12 km** (n=2) on the high one,
  whose glide is pinned at both caps though the deorbit window called the
  gate reachable -- a lead on the reach model, recorded, not chased.

  **The user's opposed-flap brake, flown** (docs/spaceplane.md, "The
  opposed flaps as a law"): subsonically a lift spoiler (L/D 2.41 -> 2.11 at
  15 deg, moment cancels), hypersonically the reverse (never out in the
  entry). `AIRBRAKE_OPPOSED_FLAPS` was **disconnected** -- armed, never
  deployed -- until 2026-09-23. Wired, it moved the old craft's touchdown
  **~600 m earlier**, the first thing that ever has, and broke 3 of 6 by
  reaching the flare at 38-58 m/s of sink against the ~39 the flare can
  arrest. The extra sink is the speed loop diving to recover the speed the
  brake took (a sink guard, `AIRBRAKE_SINK_GUARD`, flew as a no-op) -- the
  next design is how the brake and the speed loop share the surplus.

  After those, a craft file of one's own.

  The airframe is a capsule with wings: best glide L/D 3.06 at 8 deg, stall
  52.4 m/s, tail strike at 18.7 deg of body attitude, and it holds its
  commanded angle of attack (32.0 commanded, 32.2 achieved). Entry range is
  *non-monotone* in bank (1730 km at 0 deg, 1872 at 30, 1432 at 70) and in
  entry alpha, with a plateau at 20-22 deg.

Both are the same shape: one phase machine is the only kRPC-aware control code,
every tick re-propagates the state to the ground with a local RK4 propagator,
and every phase decision comes from that fresh prediction rather than a stored
plan.

```
boosterland  STANDBY -> SEPARATION -> BOOSTBACK -> COAST -> LANDING_BURN -> TOUCHDOWN
                                                    ^  |
                                                    +--+ CORRECTION

spaceplane   STANDBY -> DEORBIT -> DRAIN -> COAST -> GLIDE -> HAC -> APPROACH
                                                                        |
                                                  STOPPED <- ROLLOUT <- FLARE
```
## Commands

```bash
./run.sh                          # create .venv if needed, install krpc, fly
./run.sh --set BOOSTBACK_TOLERANCE_M=250 --set LOG_INTERVAL_UT=1
./run.sh --address 192.168.1.5    # kRPC server on another machine

python3 -m unittest discover -s tests            # full suite, no KSP needed
python3 -m unittest tests.test_flight_sim        # closed-loop flight only
python3 -m unittest tests.test_offline.TestGuidance.test_boostback_burns_against_the_miss

./quickfly.py -n 3                # fly the quicksave repeatedly, in game
./quickfly.py -n 1 --set DIAG_STATE=True   # ... and log enough to replay it
./replay.py logs/LOG53 --curve-from -1     # re-propagate a real flight offline

./sweep.py -n 2 --saves quicksave,qs_hot --compare CORRECTION_ENTER_M=300,1200
./sweep.py -n 2 --saves quicksave --configs "a:X=1;b:X=1,Y=2" --out raw.json
./savegen.py -o hot --prograde 60   # a different entry state, written to a save
./logsum.py --trace logs/LOG25      # one line per flight, plus the miss trace
./bundle.py booster                 # export one runnable file (plane: ./bundle.py plane)
./timescale.py 5 max               # fly instance 5 as fast as it will sustain
./quickfly.py --instance 5 --timescale 2.0 -n 3   # ... or a fixed multiplier

# **Starting, stopping and restarting the farm never needs permission.**
# It is the measuring instrument, it is always OK to bring up or take down,
# and a session that flies nothing measures nothing.  Standing procedure:
# `cd testInstances && ./nosleep.sh start && ./start.sh 0 1 2`, wait for the
# ports, fly; `./stop.sh` when the suite has to run or the session ends.
#
# **Three instances, and never beside the test suite.**  Four idle instances
# hold 13-16 GB of this box's 30, and a four-flight pairfly round or a
# `unittest discover` on top of them is killed by memory pressure.  Sequence
# it: stop the farm, run the suite, restart the farm, fly.  `pairfly.sh`
# takes INSTANCES="0 1 2" and still balances arms across instances over an
# even ROUNDS.
cd testInstances && ./mkbase.sh   # build a stripped KSP copy (once, ~7.5 GB)
./mkclone.sh 0 && ./kwinRun.sh 0  # an unattended instance, invisible
./watchdog.sh start                # restart any instance that dies
./fly.sh 0 -n 3                    # fly it, on that instance's kRPC ports
./stop.sh                          # stop them all (and the watchdog)
./instancebench.py 0 --quant 0.05,0.1   # how fast, and which ceiling binds
```

**Three things make a farm batch a lie, and none of them announce it.**
`pgrep -af spaceplane.autopilot` should show exactly one per busy instance --
the harness runs the flight as a *child*, so a killed batch used to leave it
flying (fixed, but check). `md5sum testInstances/ksp*/saves/default/<save>.sfs
| sort -u | wc -l` should print 1 -- each clone keeps its own quicksaves and
they drift. And `swapon --show` should be near zero *after* the batch as well
as before: four instances left up for seven hours put 15.5 GB into zram and
biased the spaceplane's arrivals **27 km long** with the spread unchanged.
Restart the farm between sessions. See [docs/test-instances.md](docs/test-instances.md).

**Instances are only comparable if they are the same game.** Three of them
drifted to three different mod sets and it cost a session: one had
`BetterTimeWarp` (which redefines the warp rate and changed which deorbit
pass was flown, worth 23 km), another was missing the four mods `base/` keeps
on purpose because they replace the spaceplane's control-surface modules.
Diff `ksp<N>/GameData` against `base/GameData` before believing a comparison
across instances, and re-clone rather than patch.

The spaceplane:

```bash
.venv/bin/python -m spaceplane.autopilot            # fly it, waiting for START
./quickglide.py -n 3 --instance 0 --timescale 6.0   # fly the quicksave, in game
#   --timescale is a CEILING: the autopilot holds the scale down to what its
#   control loop can serve, per phase (Config.TIMESCALE_GOVERNOR).  --no-govern
#   restores the old fixed-multiplier meaning, which is what fitted the landing
#   chain to the farm's loop rate.  Every log ends with a `loop rate` line.
./quickglide.py -n 1 --save qs_plane_inc --set GATE_ALT_M=1600
./glidesum.py --band 33000 26000 logs/LOG7*         # bank reversals, per flight
OUT=/tmp/base.txt N=8 ./farmfly.sh                   # one config, n per instance
./landsum.py --arrivals-within 5000 logs/LOG16*     # arrival -> handover -> wheels
./conesum.py --settled logs/LOG16*                  # what HAC_LD should be
./ladder.py --arms qs_l0,qs_l16,qs_l24 -n 4   # dose-response across the farm
./entrysave.py 0                # quicksave at the 58 km interface, mid-flight
./quickglide.py --save qs_entry --instance 0   # ... fly the glide alone from it
./armsum.py --by X --against '(default)' logs/LOG10*  # one line per arm, with error bars
./oscsum.py logs/LOG287*        # was the vehicle *pointed*?  alpha error, its
#   spread and peak-to-peak sideslip, per phase.  Nothing else reports it:
#   the shuttle wallowed 96 deg of sideslip at Mach 7 and `landsum.py` said
#   only that it landed 200 km short.
./slipsum.py logs/LOG282*       # what sideslip the airframe holds, against q
#   -- and what it costs in CdA.  The yaw axis's ceiling curve, the same
#   shape Holdable learns for alpha; SLIP_PROBE_DEG flies the probe.
# the configuration that lands (docs/spaceplane.md, "The session that made it land"):
./quickglide.py -n 1 --instance 0 --set CROSS_DEADBAND_PER_KM=20 \
  --set CROSS_DEADBAND_MAX_M=40000 --set HAC_LD=1.86 --set ENTRY_ALPHA_DEG=22 \
  --set APPROACH_BEST_LD=4.2 --set GATE_ALT_M=2000 --set TOUCHDOWN_AIM_M=2400
python3 -m unittest tests.test_spaceplane           # offline, no KSP
python3 -m tests.glidesim --dv 60                   # one entry, traced
./polar.py --mach 0.45          # the polar the airframe flies, from the logs
./aeroaudit.py logs/LOG601      # model against flight, from a log
./testInstances/planeprobe.py 0 --mass 6.715   # probe it -- but it over-reads
                                #   subsonic lift ~1.8x; see failure 13
```

`run.sh` never sources the venv — it runs `.venv/bin/python` by path. It
forwards SIGINT to the script and prints the path of the log the run produced.
`--set FIELD=VALUE` overrides a field of `Config` with type coercion; unknown
fields raise. No linter or formatter is configured. Tests need no third-party
packages (`krpc` is only imported by `autoland.py`, which the tests replace).

## Hard conventions

- **Nothing prints.** stdout/stderr are reserved for crashes before the logbook
  opens (`run.sh` funnels them to `logs/stderr.log`). All output goes to
  `logs/LOG<n>` — a new numbered file per run — and to the in-game panel.
- **Log pacing is in-game time.** `Logbook.telemetry` gates on `SpaceCenter.ut`,
  so a paused game logs nothing and a time warp still produces one line per
  `LOG_INTERVAL_UT` in-game seconds (it re-anchors instead of catching up).
  Phase changes go through `Logbook.event` and are never suppressed. Keep
  telemetry to one dense line per tick — the logs are meant to be read whole.
- **Every tunable lives in `config.py`.** Control code reads `cfg.X`; it never
  hard-codes a number.
- **Every log says which configuration flew it — *and* which defaults it was
  measured against.** `Autoland.__init__` writes a `config:` line listing the
  fields that differ from the defaults, plus `defaults_fingerprint`, a hash of
  the whole default set. Twenty flights of one session were unreadable for
  want of the first half; eighteen were misfiled into the wrong arm for want
  of the second, because a list of *differences* says nothing when the
  baseline itself moves. Spaceplane failure 28.
- **Vectors are plain 3-tuples in the body's rotating reference frame**
  (`body.reference_frame`) everywhere except the GUI. That frame rotates with
  the atmosphere, so velocities in it are already air-relative.

## Cross-cutting rules

These bite in both projects and are the ones a change most often violates.

- **Measure in game, not in the sim.** `tests/fakeksp` models a point mass with
  near-instant pointing and no aerodynamic torque, and integrates its own truth
  with the scheme it is judging. Six changes that improved a sim sweep made the
  real flight worse. Treat a sim improvement as a hypothesis and confirm it with
  `quickfly.py` / `quickglide.py`. See [docs/booster-tuning.md](docs/booster-tuning.md).
- **A knob that changes nothing may be disconnected, not powerless.** Before
  concluding "X is not the lever", check that X moved what it commands. See
  spaceplane failure 10a, which cost this project a wrong headline finding.
- **One flight per configuration is not a measurement, and neither is
  three.** The spaceplane's along-track scatter is **sd 9.6 km over 22
  flights of one configuration** — so the first batch of any comparison is
  n flights of *one* arm, not one flight of n arms. An ordered
  dose-response over three-flight arms is not evidence of a gradient:
  spaceplane failure 27 is that mistake made, adopted, documented and
  retracted inside one session. Also 10d, and boosterland's accuracy
  history.
- **A correlation across flights the autopilot chose is not a sensitivity.**
  Interface speed against landing miss correlated 0.88 over 79 flights at
  11 km per m/s, and the real figure is 0.2-1.2: both variables are
  consequences of the burn, and the regressor was carrying the geometry's
  effect. If the quantity can be *set* — `savegen.py` sets it, on an entry
  save as well as an orbital one — set it and measure the slope directly.
  Spaceplane failure 30.
- **Read the signed miss, not the distance.** A great-circle distance cannot
  tell an overshoot from an undershoot; `long=`/`cross=` and the N/E offsets
  can. That ambiguity has cost this project two sessions.
- **Measure over several entry states, with `AIM_BIAS_*` at 0.** One quicksave
  is one separation state and a number tuned against it proves nothing about
  the next. Fit a calibration from two points, never from one flight's
  residual.
- **The propagator must fly the law the vehicle flies.** `landing_command` and
  the spaceplane's speed cap are shared by the predictor and the control loop
  on purpose: a prediction of a law nobody flies is a prediction of a
  trajectory nobody flies.
- **A missing answer must not be allowed to look like a good one** — return
  `None`, not a sentinel that a threshold test will accept.
- **A quantity measured from flight data describes the vehicle *as it was
  flown*, and every change to how it is flown silently invalidates it.**
  `ALPHA_TRACKING` was taken before the attitude controller was tuned and
  read 0.87 where the tuned vehicle delivers 0.74 — and the deorbit's stretch
  bound was making decisions on it. Re-take a derived table after anything
  that changes the plant, and prefer tables something recomputes.
  Spaceplane failure 23.
- **A guard whose stated reason does not survive inspection may still be the
  only thing enforcing a constraint nobody wrote down.** Removing one whose
  argument was airtight cost three flights and 110 km; what it was really
  doing was keeping the deorbit search out of the shallow regime the
  propagator cannot be trusted in. Find out what it does before removing it,
  and write *that* down. Spaceplane failure 21, boosterland failure 17.
- **A constant that has to be re-fitted after unrelated changes is a missing
  model wearing a constant's clothes.** `DEORBIT_LONG_BIAS_FRACTION` moved
  five times in one session, every time forced by a change elsewhere. The
  question is not "what value?" but "what quantity is this standing in for,
  and can the program compute it?" — here, the span of arrivals the glide can
  still reach (`guidance.deorbit_window`). Spaceplane failure 19.
- **A phase must hand over a state the next phase can fly, and a control
  law that steps is a law no vehicle flies.** Three of this project's
  landing failures are the same shape: the rollout commanded zero angle of
  attack at 90 m/s (a nose-over) after previously commanding none at all (a
  tail-scrape); the flare held wings level for its whole ten seconds and
  landed whatever cross-track it started with; and `aim` points the nose at
  the *velocity*, which on short final lands the vehicle crabbed -- 23
  degrees of sideslip at the first rollout tick, 51 two seconds later, gear
  torn off at 48 m/s on a runway it had just landed on. Ramp the command,
  and near the ground reference it to the runway rather than the airflow
  (`Autopilot.aim_runway`). Spaceplane failures 31 and 34.
- **A clamp can make two configurations fly identically, and the log will
  faithfully report the one that was ignored.** `deorbit_aim` returned
  `max(bias, fraction * arc)`, so with the fraction retired a configured
  `-3000` flew as `0` -- four in-game batches were flown believing they were
  aimed short. Spaceplane failure 33, and `testInstances/HANDOFF.md`'s
  "read the numeric value, never the label" from the other side.
- **A control loop that cannot serve its own commanded interval is flying a
  different vehicle, and the log has to say so.** Every spaceplane log ends
  with a `loop rate` line giving, per phase, the game-seconds between commands
  and the wall-seconds one tick costs. Read it before comparing two logs: a
  phase whose first number is far above the tick it asked for was measured
  through a latency, not a vehicle. `Config.TIMESCALE_GOVERNOR` keeps that
  from happening by making the time scale serve the interval rather than the
  other way round — matched against itself it took the entry's arrival from
  -3226 m sd 806 to +294 m sd 171 at unchanged throughput. Spaceplane
  failures 63 and 65.

- **Ask what the vehicle can do that the autopilot never commands.** Every
  constant in `config.py` rations an authority the code already has, so a
  search over values cannot find one that was never wired. Three were found
  in an afternoon just by asking: `ALPHA_MIN_DEG` is 0 on an airframe whose
  own swept table reads **ClA 8.8 at zero alpha** — it physically cannot
  unload and nothing in the log would ever say so; `HAC_BANK_MAX_DEG` is 45
  while the glide routinely flies **67**, so the cone's descent authority is
  a policy and not a limit; and nothing anywhere commands an airbrake,
  though every brake constant in the file is a wheel brake and the Shuttle
  this cone is borrowed from uses a speedbrake continuously. The table even
  stops at alpha 0, so half the polar of a cambered wing has never been
  measured. When a phase is short of authority, find out whether the
  authority exists and is merely uncommanded **before** re-fitting the
  constant that rations it.

- **A measurement that shares a constant's name is usually a different
  quantity, and the agreement is what makes it dangerous.** Three arms were
  spent on this in one session, each replacing a fitted constant with a
  live measurement that looked like the same thing. `HAC_LD` 1.86 is the
  ratio the cone must *plan* with; the vehicle *achieves* 1.42, and flying
  the achieved value put it 3.9 km from the gate — `HAC_LD`'s own comment
  had already recorded that failure ("1.35 was 25% low", LOG1315, overflew
  by 3.5 km). `MARGIN` discounts the *top* of the lift curve; `LiftTrim`
  measures lift at *whatever alpha is being flown*, and substituting it
  gave a stall speed 25% low (+1405 m against +900, 8 an arm).
  `APPROACH_BEST_LD` spans rollout-to-wheels **including the flat flare**,
  so the glide ratio is not it — and the committed 4.2 measures 3.96 in
  flight while the comment beside it says 2.08. In every case the two
  quantities agreed somewhere, which is exactly why the substitution looked
  safe. Before replacing a constant with a measurement, say what each one
  is a quantity *of*, and check they are the same one.

- **Re-run the tool; do not read the prose.** A number quoted in a comment
  describes the configuration that was flown when it was written.
  `HAC_LD`'s "tracking 1.07-1.11" reads 1.27-1.39 today and
  `APPROACH_BEST_LD`'s "2.08 sd 0.27 over 41 flights" reads 3.96 sd 0.41 —
  both were taken as current, and both were wrong in a way that changed a
  decision. `conesum.py`, `landsum.py` and `armsum.py` recompute all of it
  in seconds. Failure 23's rule, aimed at the comments rather than the
  constants.

- **Every few batches, stop and audit the whole vehicle and the whole
  flight -- not the phase you are in.** Two standing checks, on a schedule
  rather than when stuck: (1) *missing controls* -- list every actuator and
  setting the craft exposes through kRPC (control-surface axes, deploy
  angles, RCS, airbrakes, gimbals, drain valves, gear, brakes, SAS/throttle
  modes) and every one the autopilot never commands or leaves at whatever
  the craft file happened to set; the shuttle found five of those in one
  afternoon after ~2800 flights on one craft, and `Drain Mode` had been right
  only by inheritance. (2) *breadth* -- before the next batch on the phase
  you have been working, glance at the others' summaries (`oscsum.py`,
  `landsum.py`, `loop rate`, touchdown survival) on **both craft and more than
  one orbit**. Sessions here have repeatedly spent a day on the approach's
  last 900 m while the entry scattered 200 km on another airframe, or tuned
  one save while a second save's bimodality sat unread. The goal is accuracy
  across ships and orbits, and the largest error is rarely in the part you
  are looking at.

- **When three attempts on one mechanism come back null, the next thing to
  change is the method, not the fourth value.** A refuted mechanism is
  information about the mechanism; a *run* of refuted mechanisms is
  information about the approach that picked them. So change what is being
  varied: measure a different quantity, act at a different point in the
  chain, or replace the law instead of re-fitting its constant. The
  generality gap survived two hundred flights of propagator work and fell to
  a release valve nobody had thought to look at (failure 61); the approach's
  speed loop was re-tuned all session and what it needed was a law that could
  make speed as well as lose it (failure 65). A constant on its fifth value,
  a diagnostic that keeps agreeing with itself, and a fix that only works on
  the save it was fitted to are all the same signal. Write down which
  approach was abandoned, not only which value was.

- **A measurement whose only consumer is a hand-copied constant cannot be
  contradicted.** `planeprobe`'s subsonic lift was wrong by 1.8x for the life
  of the project because nothing read it except a human transcribing numbers
  into `config.py` — which set the stall speed, the approach speed, the gate
  and the flare for an aircraft that does not exist. Prefer a measurement
  something recomputes (`polar.py` reads it back off the logs, the alpha
  ceiling is learned in flight); where a constant really is transcribed, say
  in its comment what would disagree with it. Spaceplane failure 13.

## Where the detail is

Read the file that covers what you are about to touch; do not load them all.

| doc | read it when |
|---|---|
| [docs/booster-design.md](docs/booster-design.md) | changing boosterland guidance, the propagator, a phase, or the drag/lift estimators |
| [docs/booster-failures.md](docs/booster-failures.md) | **before undoing anything that looks redundant** — 17 numbered rules, each paid for by a lost or broken flight. The other booster docs cite these by number |
| [docs/booster-tuning.md](docs/booster-tuning.md) | measuring a change: `replay.py`, aim bias, sim-vs-game, the accuracy history, flying a different vehicle or body |
| [docs/booster-testing.md](docs/booster-testing.md) | writing or reading `tests/fakeksp` regressions |
| [docs/spaceplane.md](docs/spaceplane.md) | anything in `spaceplane/` — the airframe measurements, the entry design, the runway |
| [docs/spaceplane-failures.md](docs/spaceplane-failures.md) | same, before undoing something; failure 10 is the open frontier (the glide lands 40-70 km short) |
| [docs/test-instances.md](docs/test-instances.md) | running the parallel-instance farm, or when instances die |
| [docs/krpc.md](docs/krpc.md) | any kRPC call whose signature you are not certain of |

`plan.txt` is the booster's original specification and still describes the
intended flight profile.

**Keep this file short.** It is loaded into every session whole; the docs are
not. New material goes in the doc it belongs to, and only a rule that applies
to *every* change belongs here.
