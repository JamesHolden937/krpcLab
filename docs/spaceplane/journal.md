# The spaceplane: session journal

Every session's measurements and conclusions, oldest first.  Append new
sessions at the end.  Numbers here describe the configuration flown *at the
time*; re-run the tools rather than quoting them (CLAUDE.md, "Re-run the tool").
The design reference is [design.md](design.md); the current state is
[`spaceplane/CLAUDE.md`](../../spaceplane/CLAUDE.md).

## State as of 2026-09-17 (formerly the top of spaceplane.md)

The second autopilot: deorbit, entry, and a runway landing. Its failure modes
are in [spaceplane-failures.md](spaceplane-failures.md).

A second autopilot in the same tree: deorbit from orbit, fly an atmospheric
entry, and land a winged vehicle on the KSC runway. `boosterland/` is untouched
by it and its 152 tests still pass; what the two share is `boosterland.vec` and
`boosterland.logbook`, the `testInstances/` farm, and every convention in
`CLAUDE.md`.

```bash
.venv/bin/python -m spaceplane.autopilot            # fly it, waiting for START
./quickglide.py -n 3 --instance 0                   # fly the quicksave, in game
./quickglide.py -n 3 --instance 0 --timescale 4.0   # ... about 3.5x faster
./quickglide.py -n 1 --save qs_plane_inc --set GATE_ALT_M=1600
./glidesum.py --band 33000 26000 logs/LOG7*         # reversals, per flight
OUT=/tmp/base.txt N=8 ./farmfly.sh                   # one config, n per instance
./landsum.py --arrivals-within 5000 logs/LOG16*     # arrival -> handover -> wheels
./conesum.py --settled logs/LOG16*                  # what HAC_LD should be
./armsum.py --by GLIDE_RESERVE_ON --against '(default)' logs/LOG10*
python3 -m unittest tests.test_spaceplane           # offline, no KSP
python3 -m tests.glidesim --dv 60                   # one entry, traced
./testInstances/planeprobe.py 0 --mass 6.715       # measure the airframe
```

An entry is ten minutes of game time and the orbital wait is more, so
`--timescale` is most of what makes a repeatability question affordable here:
it paces the control loop on `ut` (`LOOP_PACING_GAME_TIME`) and prints what
the instance *achieved* rather than what it was asked for. Read
`docs/test-instances.md` before trusting a multiplier.

**Where it is** (2026-09-17, from `qs_plane` on a freshly restarted farm).
Read this with `landsum.py`, which prints all of it. **The two rows are two
different configurations and the difference between them is not committed:**

```
                              arrival mean   sd      inside 5 km
committed defaults   n= 7      -1.5 km     17.0 km     1 of 7
+ CROSS_DEADBAND arm n=30      +2.8 km     12.8 km    17 of 30   (--set, not
                                                                  committed)

both configurations, of the flights that then landed:
landing bias (stopped-arrival) mean +1.75 km  sd 0.80 km
cross-track at rest            |mean|  30 m   8 of 10 inside the 35 m strip
best landing                   20 of 23 parts, stopped +917 m along, -40 across
```

The baseline row is n=7 and the arm is one arm; neither is a measurement by
this project's own standard (failure 27). What the arm is good for is saying
where to point the next proper batch.

**The lateral problem is closed and the longitudinal one is not.** That is
the reverse of what this file said for several sessions, and the reversal is
the cone's doing: it picks which way to turn from where the vehicle actually
is, so it absorbs kilometres of cross-track and delivers tens of metres.

**Judge the entry and the landing separately.** They fail independently and
each hides the other:

- the **entry** on the committed defaults delivers an arrival inside 5 km
  about **one flight in seven**, and nothing downstream can rescue the rest;
  the uncommitted deadband arm made that roughly one in two;
- the **landing chain** adds **+1.75 km** of its own on top of whatever the
  entry did, against a runway that accepts +/-1200 m -- so even a perfect
  arrival runs off the far end today.

Both have a specified next step and neither needs new invention.

**What is left, in the order the evidence points:**

- **The landing bias is one constant measured over the wrong span**, and the
  fix is a *pair* of numbers that have to move together: `APPROACH_BEST_LD`
  1.85 -> ~2.20 and `GATE_ALT_M` 2600 -> ~1950. See "The landing chain, and
  the one number that sizes all of it" below for the arithmetic, the
  predicted result, and the failure mode to watch for (`out of height`
  exits). This is the cheapest remaining kilometre and a half.
- **The entry's scatter is made by a relay.** The glide spends **87% of its
  time with the bank off its stops** -- 24 reversals at 17.5 s per
  stop-to-stop slew -- and entry range is non-monotone in bank, so every
  reversal crosses the branch `SOLVE_BANK_MIN_DEG` exists to avoid. Raising
  `BANK_RATE_DEG_S` is not the lever (the relay just cycles faster: 33
  reversals, no change in arrival). Widening the cross-track deadband is:
  flown at `CROSS_DEADBAND_PER_KM=4, CROSS_DEADBAND_MAX_M=6000`, arrivals
  inside 5 km went **1 of 7 -> 17 of 30**, with the arrival cross-track
  *unharmed* (-138, -35, +16 m on the first three). The knob's original
  rejection was paid for entirely in cross-track, which the cone has since
  made free. **It is not committed**: the 30-flight arm is one arm, the
  baseline it beats is n=7, and the cap wants a sweep (3 km moved the
  reversal count only 24 -> 21; 6 km moved it to 17).
- **The deorbit still sometimes takes a different opportunity**, 2133-2186 km
  to run instead of 2244-2254, and those flights arrive **+36 km** on
  average against +5.3 for the rest -- 13 of 41 flights. A hard floor
  (`DEORBIT_COMMIT_ARC_MIN_M`) is implemented and left at 0: as a veto it
  makes the phase *wait*, and what it waits for may be badly out of plane.
  It wants to be a preference among the opportunities available at one
  moment, not a veto that spends them.

**Before trusting any of the numbers above, read
[test-instances.md](test-instances.md) on the farm.** Three separate faults
were silently corrupting measurements here until 2026-09-17 -- orphaned
autopilots surviving a killed batch, per-instance quicksaves drifting apart,
and a farm left up long enough to swap -- and the last of those biased
arrivals **27 km** with the spread unchanged, which reads exactly like a
guidance regression. All three are fixed or checkable in one line; the
checks are worth running before a batch and after it.

### Start here (2026-09-17)

If you are picking this up cold, in this order:

1. **Check the farm before anything else** -- three one-line checks in
   [test-instances.md](test-instances.md). Until 2026-09-17 all three were
   silently wrong at various points and a whole session's comparisons went
   with them. `farmfly.sh` refuses to start if the first one fails.
2. **Run `./landsum.py logs/LOG16*`** on the newest batch. It prints the two
   numbers that matter -- the *arrival* (the entry's doing) and the *landing
   bias* (everything after it) -- and they have to be read separately.
3. **Take the landing bias first.** It is the cheaper of the two, it is
   fully diagnosed, and the fix is two constants moved together:
   `APPROACH_BEST_LD` 1.85 -> ~2.20 with `GATE_ALT_M` 2600 -> ~1950. The
   arithmetic, the predicted result and the failure mode to watch for are
   under "The landing chain, and the one number that sizes all of it".
4. **Then the entry.** `CROSS_DEADBAND_PER_KM` looks like the lever and is
   not committed; it needs a cap sweep against a proper baseline, not
   another single arm. See "What is left" above and failure 41.

What *not* to spend time on, because it has been measured and written down:
cross-track anywhere (solved by the cone), `BANK_RATE_DEG_S` (connected, not
the lever), a minimum-width requirement on the deorbit window (backwards --
the wider windows are the worse passes), and `DEORBIT_COMMIT_ARC_MIN_M` as a
veto (it makes the phase wait for something worse).


## Session handoff, 2026-09-20: what is measured, what is refuted, what is open

Read this before touching the spaceplane. It is the state a whole session of
in-game measurement left, and most of it is *negative* results -- which is the
expensive kind to re-derive.

### The committed configuration, and what it actually does

Fingerprint `6debf0c5`. On `qs_plane`, two **independent** eight-flight
batches of this exact configuration, which agreed at 7 of 8 each:

    n=16   intact 14 (88%)   on the runway 14 (88%)   on the strip 12 (75%)
    stopped along  +939 m  sd 455        (the runway is +-1200 m)

Two changes were added this session and both are **flown, not argued**
(`pairfly.sh`, 8 an arm, `qs_plane`): intact 4/8 against 2/8, on the strip
6/8 against 3/8, cross-track at rest **27 m against 66**.

- `BRAKE_FOR_DISTANCE` -- the rollout braked flat out from the first tick
  (`brakes = speed < 200 m/s`, i.e. always), measured at **12.3 m/s^2**, which
  is 83 kN on 6.9 t -- more than the vehicle weighs -- for a rollout whose own
  docstring computes a requirement of 0.5 and notes drag alone gives 1.0.
  Now `needed = v^2 / (2 * remaining)` less the drag already present.
- `APPROACH_BANK_COMPENSATION` -- `alpha_for_speed` solves for a load in the
  *vertical plane* and nothing divided by `cos(bank)`, so at the S-turn's 40
  degree limit the wing carried 77% of the load the loop computed, in exactly
  the moments the approach had decided it was high.

### Refuted this session -- do not re-try without reading the failure

| what | result | where |
|---|---|---|
| `TOUCHDOWN_AIM_M` 2400 -> 1200 | lands 245 m earlier, **halves survival** (3/8 vs 7/8) | 84 |
| S-turn stop as a time, not a distance to the aim | wrecks at cross-track +125..+145 m | 84 |
| `CROSS_DEADBAND_MAX_M` 40 km -> 10 km | +3.8 km, t=1.80 at n=8; **looked decisive at 4v4 and dissolved** | 78 |
| `ALPHA_TRACKING_ON` | table is *correct* (re-measured, 25 007 ticks) and enabling it is **worse** | 77 |
| `GLIDE_BANK_DUTY_ON` (propagate the reversal duty cycle) | -1350 m controlling for reversals | 79 |
| landing gear spring/damper, **both directions** | stiffer 1/8, softer 4/8, **stock 7/8** | 83 |

### Open, in the order I would take them

1. **Inclined orbits.** The arrival at `GLIDE -> HAC` is bimodal, `+86..+1005`
   against `-3664..-20562`, and **half its variance is the bank reversal
   count: +2137 m per reversal, r^2 = 0.49 over 35 flights**, monotone from
   six reversals to eleven. `Steer` holds one constant lean while the vehicle
   reverses 6-12 times, spending 88-138 s of a 700 s glide near wings level
   where it sinks less and flies further -- a range term the propagator has no
   representation of (CLAUDE.md's "the propagator must fly the law the vehicle
   flies"). Failure 79 explains why modelling it *alone* makes things worse:
   it cancels another error nobody has found, and **the pair has to be fixed
   together or not at all**.

   The lead I was on when the session ended: the dwell that limits the
   reversal rate (`BANK_REVERSAL_DWELL_SLEWS`) is switched off below
   `SPEED_FLOOR_MACH`, and that constant's own comment says the subsonic
   phase "still turns over every 12-18 s against a 17.5 s slew -- so it is
   permanently in transit, and that is where the flights that still diverge
   lose it." Splitting the dwell's Mach gate out from `SPEED_FLOOR_MACH`
   makes that a one-flag experiment. **Not run.** Note the caveat that sent
   me back to the drawing board: the divergence I measured on `qs_plane_inc`
   and `qs_e80` is at 24-19 km and Mach 3.7-2.5, which is *above* the gate,
   so the dwell was already active there -- the documented subsonic churn is
   a different window from the one the arrivals diverge in.

2. **Higher-energy orbits.** A five-arm ladder (`savegen.py --prograde`
   0/20/40/60/80 m/s, four flights each, every arm on every instance) puts the
   cliff between a **104 km and a 130 km apoapsis**: 3 of 4 on the runway
   becomes 0 of 4 and never recovers. **The failure is dispersion, not bias**
   -- arrival sd 157 m at circular against **5836 m** at 130 km. Peak q does
   not explain it (the worst arm has the *lowest* q), nor does the alpha
   ceiling crossing `SOLVE_ALPHA_MIN_DEG`, nor Kerbin's rotation (checked:
   burn-to-arrival is 1095-1114 s across all five arms, so the surface turns
   the same ~193 km under every one). What survives: the glide shortens
   598 -> 489 s while burn-to-arrival stays fixed, and the deorbit range comes
   out at 1945-1950 km on every arm against a `DEORBIT_RANGE_MAX_M` of
   2300 km it never uses. Failure 80.

3. **It lands late.** +939 of a +-1200 m runway is the last quarter of the
   tarmac; a flight that stops at +1737 has rolled off the end, which is the
   complaint this session opened with. Four levers have been measured against
   it and rejected (the table above). Nothing cheap is left.

### Two method errors this session, both now rules

- **"Off until flown" is not the conservative choice.** Two changes were
  committed as defaults, then turned *off* as unvalidated, then measured and
  found to be worth about half the landing performance. Disabling an unflown
  change swaps in the opposite hypothesis and calls it caution. Failure 82.
- **An eight-flight batch resolves a factor of two and nothing finer.** The
  same configuration gave 4 of 8 and 7 of 8 in consecutive batches; a
  four-flight arm looked "decisive at 4.4 sigma" twice and dissolved both
  times. Quote a rate with its n, and prefer `pairfly.sh`, whose
  round-by-round interleaving protects a *comparison* even when the absolute
  level drifts. Failures 78 and 83.

### Tooling added or repaired

`rollsum.py` (what the rollout was handed vs whether it kept the vehicle),
`appsum.py` (what the approach flew vs the descent it asked for),
`pairfly.sh` (two arms **or** two saves, halves swapped every round, so the
instance and the clock are both controlled; it also records the defaults
fingerprint at launch and re-checks it at the end). `armsum.py` was **silently
dead** since the heading-alignment cone was added -- it greps for
`GLIDE -> APPROACH`, a transition that no longer exists -- and grouped by the
per-instance governor path, making twelve flights into twelve arms of n=1.
Both fixed. Every log now records `SAVE_NAME`, without which a campaign across
three entry states misfiles itself.

## Session handoff, 2026-09-21: the aim's gain, and what it fixes

Read this with the 2026-09-20 handoff above, which it corrects in two places.

### The one number this session found

**One metre of arrival costs 6.4 m of `DEORBIT_CENTRE_BIAS_M`**, because two
gains multiply and neither had been measured:

    the window moves  -19.7 km per m/s of dv   (offline, every energy in the ladder)
    the arrival moves  -3.06 km per m/s        (in game, qs_e60, 16 flights)

Every test this project has run on the deorbit aim used 2.5-17 km of bias,
which buys 0.4-2.6 km of arrival -- at or under the noise of an eight-flight
batch, every time.  So "the aim is not a lever" (CLAUDE.md, failures 33, 68,
84, 86) is what a *correct* knob looks like when it is read at a seventh of
its scale.  Flown at the right scale on `qs_e60` the aim moves the arrival
3-4 km per 20 km of bias, with the delivered dv moving exactly as predicted
beforehand (+1.2 m/s at 28000 against a prediction of +1.1; +0.65 m/s from
28000 to 44000 against 0.66).  Failure 88.

**Check the gain before believing a null on any aim knob.**  The gain is
computable offline in one command (`deorbit_solution` at two biases), and
that check is cheaper than the batch it saves.

### What that buys on the high-energy save

`qs_e60` (a 157 km apoapsis, failure 80's arm that went 0 of 4 onto the
runway) is now a bracketed one-parameter problem:

| `DEORBIT_CENTRE_BIAS_M` | delivered dv | arrival | where it stops |
|---|---|---|---|
| 0 (committed) | 96.2 | +3939 sd 3100 | cone out of height, mostly lost |
| 28000 | 97.4 | -48 sd 3040 | mixed; the two runway landings of that batch |
| 44000 | 97.6 | -1354 | ~+3000, rolled off the end |
| 67000 | 98.6 | ~-7000 | **intact 3 of 6**, 7 km short |

**and then, on a farm restarted for it, 51000 lands it** (8 an arm,
`pairfly.sh`, swap 4.0 GB):

| | stopped along | on the runway | intact | arrival | cone surplus |
|---|---|---|---|---|---|
| `DEORBIT_CENTRE_BIAS_M=51000` | **+793 sd 123** | **8 of 8** | **6 of 8** | +131..+531 | +496..+499 |
| committed defaults | +152 sd 3350 | 1 of 8 | 0 of 8 | +4000..+5064 | -465..-751 |

Re-flown after `GOVERN_ON_PEAK` became the default (below), the same arm is
**8 of 8 on the runway and 8 of 8 intact** against 0 of 8 -- sixteen flights
of the configuration across two batches, sixteen on the runway, fourteen
intact.  Failure 93.

`sd 123 m` is tighter than `qs_plane`'s own committed configuration
(+939 sd 455).  **The three rows above it locate the optimum in the wrong
place** -- they were flown with 10.6 GB in zram after two and a half hours
of farming, which is the drift CLAUDE.md's farm rule is about: `pairfly.sh`
protects the comparison and nothing protects the level.  Failure 89.

**The target is the reserve, and that is not a fitted number.**  The winning
arm arrives at +131..+531, which is `GLIDE_RESERVE_M`, and earns a cone
surplus saturated at +499.  Only the *bias that achieves it* is per-state,
and 51000 must never be committed as a default -- on `qs_plane`'s 32 m/s
burn it is 2.2 m/s and 6-7 km short.

**`conesum.py`'s surplus is the outcome variable on this save**: positive
and the cone rolls out and the vehicle lands, negative and it exits *out of
height* 3-6 km from the gate with the approach having no vote.

### Two corrections to the 2026-09-20 handoff

- **The bank reversal count is a symptom of saturation, not a range term.**
  Its sign is +2137 m per reversal on `qs_plane_inc` and *negative* on
  `qs_e60` (8-9 reversals give +2689..+5003, 13 give -238).  A range term
  cannot change sign between two entry states of one vehicle.  What is the
  same on both is that a saturated glide stops reversing -- pinned at bank
  0.9 when short, at bank 70 when long -- and misses in the direction it is
  pinned.  Failure 87.  This retires the modelling programme of failures 78,
  79 and 85.
- **Failure 79's pre-registered "both together" arm is refuted.**
  `GLIDE_BANK_DUTY_ON` + `ALPHA_TRACKING_ON` on `qs_plane_inc`, 8 an arm:
  arrival -11664 sd 10700 against -4348 sd 4400, and the arm moved the
  reversal count *down* (9.0 to 7.4), which is the mechanism of its own
  failure.  Failure 85.

### The governor was reading a mean where it needed a tail

`ScaleGovernor` keeps a decaying *maximum* of the tick cost -- its own
comment says "Not the mean: what binds is the tail" -- and it was being fed
`LoopRate.busy`, an exponential average.  Two filters in series, and the
second never sees what the first removed.  `DEORBIT`'s search ticks cost
50-80 ms and its waiting ticks 3-12, so while the phase waits the estimate
falls to the cheap ones, the scale ramps to the 6x ceiling, and the burn
ignites there -- 0.3 game-seconds between commands at 17.8 m/s^2, which is
5.3 m/s of dv.

Sorted by achieved `DEORBIT` interval, every `qs_plane_inc` flight on disk
splits at 0.22 game-s/tick **with nothing in between**: 0.10-0.21 arrives
within 2.6 km, 0.24-0.32 arrives 6.5-25 km short.  `GOVERN_ON_PEAK`
(committed) governs on `LoopRate.peak`, the phase's worst tick, undecayed:

| | `DEORBIT` interval | arrival | on the runway |
|---|---|---|---|
| `qs_plane_inc`, peak | **0.10-0.14 every flight** | **+199 sd 420** | 4 of 6 |
| `qs_plane_inc`, mean | 0.10-0.31, bimodal | -4378 sd 5369 | 4 of 8 |
| `qs_plane`, peak | -- | -- | 8 of 8, +838..+971 |
| `qs_plane`, mean | -- | -- | 7 of 8, +554..+1258 |

4-5% of throughput for a thirteenfold reduction in arrival scatter and no
regression anywhere.  Failures 91-92.  Asking for the interval earlier does
*not* fix it (`DEORBIT_PACE_WHOLE_PHASE`, flown, null): the interval was
never the problem, the cost estimate was.

**The rule: a governor must be driven by the worst tick of the phase it
governs, and any smoothing upstream of it is the bug.**

### Open, in the order I would take them

1. **`qs_plane_inc`'s touchdown.**  The arrival is fixed; the vehicle still
   comes apart on contact on most flights, from flare states
   indistinguishable from `qs_plane`'s (h 123-160, v 90-99, sink 31-45,
   touchdown 45-62 m/s), `Elevon 4` first.  Failure 81's reading stands and
   it is a craft change.
2. **The deorbit bias is a per-state calibration and must not become a
   default.**  51000 works on `qs_e60`; on `qs_plane`'s 32 m/s burn it is
   2.2 m/s and 6-7 km short.  A proportional-to-dv law fails on inspection:
   `qs_plane_inc` already arrives at +480 with no bias at all, so 3% of its
   43 m/s burn would push it 3.5 km short.  What the aim needs is a
   *measured* correction, and the one early measurement nobody takes is in
   the coast -- the propagator predicts the interface crossing minutes
   before the glide begins and nothing compares it with what arrives.
   `DIAG_INTERFACE` exists for exactly this.
3. **A superseded lead, recorded so it is not re-taken.**  Failure 90 read
   `qs_plane_inc`'s bimodality as made *during the coast*, because the first
   GLIDE tick's `long=` predicts the arrival 600 s early.  That observation
   is correct and the inference was wrong: what the first tick was reporting
   is the state a burn flown at 0.3 game-seconds per command leaves behind
   (failure 91).  The coast is fine.
4. **`GLIDE_RESERVE_M` on inclined orbits.**  500 -> 4000 measured +5.0 km
   of arrival on `qs_plane_inc`, 8 an arm, and it is *not* committed: on
   `qs_plane` it would land later still, and that save's complaint is
   already that it lands late.  Worth re-measuring now that the arrival
   scatter there is 420 m rather than 5.4 km -- the whole reason the
   reserve was cut to 500 was a save whose entry had stopped scattering.

## Session handoff, 2026-09-21 (second): uncommanded authority, and four refuted arms

Baseline reproduced CLAUDE.md exactly -- **+900 to +1081 m, sd 184-210, 5 of 5
on the runway**, committed configuration, four in-game batches of
`pairfly.sh` at 8 an arm.

| arm | result | verdict |
|---|---|---|
| `AIRFRAME_DERIVED` (batch 1) | +1405 sd 364, 1 of 4 on the runway | worse |
| `HAC_LD` 1.49, the *achieved* ratio (batch 2) | 3.9 km from the gate, stopped 5.2 km | far worse, killed at n=2 |
| `AIRFRAME_DERIVED`, corrected (batch 3) | +2751 sd 639, **0 of 7** on the runway | worse, twice measured |
| `GEAR_FOR_ENERGY` (batch 4) | +800 vs +947, ratio 4.12 -> **3.76**, 6 of 8 vs 4 of 8 intact | mechanism confirmed, size below noise |

`AIRFRAME_DERIVED` is refuted on this craft and stays off. `GEAR_FOR_ENERGY`
is the one arm whose *mechanism* was confirmed -- the glide ratio moved in the
commanded direction -- and its size is under an eight-flight batch, which is
consistent with the 3% gear cost above rather than the 19% it was sized on.

### Corrections to things the project believed

- **`APPROACH_BEST_LD` = 4.2 is right; its comment is stale.** The comment
  says "2.08 sd 0.27 over 41 flights"; `landsum.py` measures the
  rollout-to-wheels ratio at **3.96 sd 0.41** today. Do not "fix" the
  constant. `APPROACH_LD_DERIVED` is documented do-not-use.
- **`HAC_LD`'s "tracking 1.07-1.11" reads 1.27-1.39 today** (`conesum.py`, 58
  flights, the 30 that roll out). The constant 1.86 is what the cone must
  *plan* with and is not the achieved ratio -- flying the achieved ratio
  stopped 5.2 km from the runway, which `HAC_LD`'s own comment had already
  recorded once (LOG1315, "1.35 was 25% low", overflew by 3.5 km).
- **Do not raise `APPROACH_SCURVE_MAX_DEG`.** The S-turn is saturated and it
  is the tempting knob, but its cost is cross-track at the flare door, which
  is already at the wreck threshold: mean **73 m** against a measured
  survivor/wreck split at **70 m**. It trades a late landing for a destroyed
  vehicle.

### Refuted mechanisms, each with its account in the code

Do not re-try these without reading the docstrings, which record why:
`airframe.lift_discount` (`LiftTrim` for `MARGIN` -- different quantities, and
the bin reads 0.74/0.49/2.54 across three flights); `airframe.approach_ld`
(omits the flare's flat 400-500 m); `airframe.cone_ld` at the achieved ratio
(`PLANNING_BIAS` -- the cone must plan *above* what it achieves).

### The next change, now built and unflown: a split-rudder airbrake

The vertical `smallCtrlSrf` pair deployed in *opposing* directions is a
speedbrake: side forces at equal and opposite moment arms, so yaw and roll
both cancel and what is left is drag. **No control authority is lost, because
these surfaces have none** (see "This vehicle has no aerodynamic control at
all" above).

**The identification rule must stay geometric and must refuse rather than
guess.** `testInstances/surfacespan.py` implements it: span axis from the
part rotation quaternion; vertical when `|span.z| > |span.x|` in the vessel
frame; require a pair mirrored about the centreline; **no pair -> no
airbrake**, and the vehicle flies as it does now. Horizontal surfaces are
never candidates -- cancelling a canard against an elevon needs a balance of
areas and moment arms that differs on every aircraft, which fails the
generality bar.

**Trigger it on the S-turn saturating, not on an altitude.** Measured over
batches 3-4: the approach's weave command `sc=` sits at its 45 deg cap for
**52% of approach ticks**, with **+639 m of surplus still unspent**. That is
the guidance's own signal that its only range control has run out, so it is a
principled trigger rather than a fitted constant.

**Do not simply bolt drag on.** The gear result above is the warning: the
speed loop re-trims around added drag and absorbs most of it. The brake has to
be commanded against surplus, the way `GEAR_FOR_ENERGY` is.

**Built as `spaceplane/airbrake.py`, `AIRBRAKE_SPLIT_RUDDER`, default off and
never flown.** `find_split_rudder` is the geometric rule above and returns a
reason on every path, armed or refused, because a brake that silently did not
arm reads in the log exactly like one that armed and did nothing.
`Brake.update` is the policy, and **every threshold in it is derived from a
constant some other phase already owns** -- a brake with three fitted numbers
of its own is a brake that must be re-fitted on the next aircraft:

| what | derived from | not |
|---|---|---|
| the surplus that arms it | `APPROACH_SCURVE_M`, the surplus that started the weave | a second constant for the same quantity |
| the window | `AIRBRAKE_TAU_CYCLES` x 2 x `APPROACH_SCURVE_PERIOD_S` | a fitted number of seconds |
| where it stows | the flare door this tick computes (`FLARE_ALT_M + FLARE_LEAD_S * sink`) plus `AIRBRAKE_STOW_LEAD_S` **seconds of sink** | a height fitted to this craft's ~470 m door |

There is an unconditional stow on entering FLARE as a backstop.

#### Flown once, 8 flights, and stopped early: two defects, both in the
#### measuring rather than the mechanism

The identification worked on the live craft at the first attempt --
`airbrake armed: x=-2.50/+2.50, 1.0 m^2 each`, `deploy angle 20 deg set on 2
of 2` -- and the policy fired as specified: `airbrake out at 702 m: S-turn
saturated 50% with +226 m left`, then `in at 430 m: surplus spent (+98 m)`.
The batch was killed at round 2 of 4 because it could not have answered
anything, and both reasons are worth more than the batch would have been.

**1. The saturation measure was an EMA, and an EMA starts cold.** The window
spanned two weave cycles -- 40 s of a ~75 s approach -- so the average was
still charging when the vehicle reached the stow height. Of four `B` flights,
**two deployed for about seven seconds** and **two peaked at 0.44-0.48 and
never armed at all** (LOG2815-2818). Half the treatment arm was therefore
identical to the control, and the other half carried seven seconds of a 1 m^2
pair. *Fixed:* the share is now **capped seconds over elapsed seconds**,
believed once `AIRBRAKE_MIN_CYCLES` of weave has been sampled -- which is
the quantity the mechanism was built from in the first place ("the cap for
52% of approach ticks"), unbiased from the first tick. Nudging
`AIRBRAKE_SATURATED_FRAC` down to 0.4 would have "fixed" the same symptom by
fitting a constant to this craft.

**2. The check that was supposed to be able to contradict the mechanism could
not.** It logged each half's `available_torque` and their sum, expecting near
zero -- but `available_torque` is the torque a surface *could* produce, given
as positive and negative magnitudes, so two halves can never cancel in it
however perfectly they oppose. `deflection` read `+0.00` through a deployment
that demonstrably happened, because with every axis disabled there is no
control-input deflection to read. Both agreed with themselves and neither
could ever have disagreed: failure 13's shape, rebuilt by accident.
*Fixed:* the log now records the **sideslip and body rate** when the brake
comes out, and the settling comparison is `slip=` with `ab=out` against
`ab=in` over a batch -- a clean split leaves them indistinguishable.

**What the batch did establish**, incidentally: both arms wrecked at a
similar rate (4 of 8 kept 18+ parts), and every wreck entered the flare
outside the 70 m cross-track split (-103, +240) while the survivors were
inside it. That is failure 34's known mechanism and it indicts neither arm.
 The approach line now
carries `ab=` and `sat=`, and `sat=` is logged even when the brake is not
armed -- it is the measurement the whole mechanism rests on and it costs
nothing to keep taking.

**What to fly:** one `pairfly.sh` arm of 8 against the committed default, on
`qs_plane`, reading `stopped along` first and the intact count second. The
prediction is a *smaller* overshoot with `ab=out` on the ticks where `sc=`
was capped; the failure modes to watch for are a yaw the identification was
supposed to make impossible (`xt=` growing while `ab=out`) and the gear
result repeating -- the speed loop absorbing the drag, which would show as
`ab=out` with `exc=` unmoved. 1 m^2 of extra `CdA` against a craft total of
~5.6 is the size of the effect available, so it is not a subtle-enough
question to need more than one batch.

**Two generality questions it does not yet answer**, both worth taking before
any constant in it is touched:

- *Other craft.* The identification refuses on anything but a clean mirrored
  vertical pair, so it is safe by construction -- but "safe" and "works" are
  different claims and only `qs_plane`'s geometry has been run through it,
  offline. `testInstances/mkheavy.py` and a craft file with a centreline fin
  or with two fin pairs are the cheap tests, and the second of those needs no
  game at all.
- *Other situations.* The trigger is the APPROACH's weave saturating, because
  that is where the surplus was measured. The same saturation signal exists
  one phase up -- the cone's `wv=` against `HAC_BANK_MAX_DEG`, and the
  glide's bank pinned at its stops, which is what failure 87 reads the
  reversal count as. A vehicle that arrives with the surplus the *entry*
  could not spend reaches the approach with more than the approach can spend,
  and the brake would be a phase too late. Do not extend it there by
  analogy: each phase's stow constraint is different, and the flare's is the
  one that destroys vehicles.

### Also unflown and still worth a batch

- **`HAC_ENTRY_AFFORDABLE`** -- enter the cone when it can pay for itself.
  Separates 21/3 from 9/24 over 57 flights offline, but note it asks
  `needed`, computed with `HAC_LD`, so it may be inert until that is honest.
- **`testInstances/mkheavy.py`**, which builds `qs_plane_heavy` (+20%
  landing mass via MonoPropellant, which `DRAIN` does not dump) -- the only
  generality test available, since every save in the farm holds the same
  airframe. It writes binary to preserve CRLF; reading a save in text mode
  silently strips 14 kB of line endings.

### State of the tree

**The defaults fingerprint is now `61adf259`, was `6debf0c5`.** Nothing that
flies changed -- `GEAR_DRAG_FRACTION` 0.19 -> 0.03 only bites with
`GEAR_FOR_ENERGY` on, and the airbrake fields are new and off -- but a batch
flown before this session carries the old hash, and `armsum.py` will file it
as a different baseline. That is the mechanism working (failure 28), not a
fault: the eighteen-flight and sixteen-flight results quoted above remain the
reference for this configuration's *flight* behaviour.

New this session, **all default off**: `AIRFRAME_DERIVED`,
`APPROACH_LD_DERIVED`, `HAC_ENTRY_DERIVED`, `HAC_ENTRY_AFFORDABLE`,
`LIFT_TRIM_DISCOUNT`, `GEAR_FOR_ENERGY`; plus `STALL_CALIBRATION_M_S` 55.6 and
`airframe.PLANNING_BIAS` 1.09, which exist so a derived quantity is expressed
in the units its consumers were fitted in and is therefore inert on the
reference craft. New modules: `spaceplane/airframe.py` additions,
`tests/flownpolar.py` (a real swept polar from `logs/LOG2747`),
`testInstances/{ctrlsrf,surfacespan,mkheavy}.py`, and now
`spaceplane/airbrake.py` with `AIRBRAKE_*` in `config.py`. The last four
batches are `logs/LOG2757-2812`. 530 offline tests pass.

### How the session went, which is itself information

Three changes were refuted in flight and one analytic claim was retracted
after being built on truncated shell output. The pattern is now written as
rules in CLAUDE.md: a measurement that shares a constant's name is usually a
different quantity; re-run the tool instead of reading the prose; and ask what
authority the vehicle has that the autopilot never commands. **The user
supplied the two most productive ideas of the session** -- pitch-down and bank
as uncommanded controls, and the split rudder -- and both were *missing
mechanisms* rather than mis-set constants.


## The airbrake flew, worked, and destroyed two vehicles doing it

The corrected estimator did what it was meant to -- `airbrake out at 1077 m:
S-turn saturated 100% with +459 m left`, against 702 m and a seven-second
deployment before -- and the drag is real and large:

```
alt=1238  v=106.3  ab=in     sc=45.0
alt=1023  v=105.6  ab=out    <- brake out
alt= 869  v= 93.4  ab=out
alt= 747  v= 84.1  ab=out    <- 21 m/s of speed gone in ~11 s
alt= 594  v= 85.3  ab=out
alt= 307  v= 99.5  ab=in     <- stowed; the speed loop dives to recover
```

Both braked flights then entered the flare at **h≈250 m with 79-82 m/s of
sink** (LOG2824, LOG2825) against **h 119-139, sink 29-36** for the unbraked
controls of the same round -- and both came apart on contact, while the two
controls landed intact at +900 and +953. The arrival did move the right way
(+186 against +900), which is the mechanism working; the vehicle just could
not fly the state it was handed.

**The cause is not the drag, it is which currency the drag was spent in.** A
glider makes speed only by trading height for it. The brake took the speed,
the speed loop bought it back with a dive, and the dive arrived at the flare
door. The surplus the brake exists to spend is *height*; the speed is the
approach's margin and was never the brake's to take.

*Fixed:* `AIRBRAKE_SPEED_GUARD` -- the brake retracts the moment the airspeed
falls below `guidance.approach`'s own target, which the command now
publishes (`command.target_speed`) rather than a second constant that could
drift from it. Worked through LOG2825's own speed profile the guard fires at
**747 m**, 250 m and eight seconds earlier than the flare-door stow did, with
the speed deficit at stow ~0 instead of 21 m/s. Unflown.

**And this is the same lesson the landing gear taught, from the other side.**
Gear-down drag was absorbed because `APPROACH_SPEED_PATH` re-trims alpha to
hold the commanded speed (3% of glide ratio, not 19%). The airbrake's drag
was large enough to break that loop instead of being absorbed by it -- so the
loop bought the speed back the only way it can. **On a speed-holding path
law, a drag device converts into flight-path angle or into a dive, and into
range only if the guidance spends the surplus deliberately.**

## The cone's dissipation is pinned at both of its caps

Found by asking the airbrake's question one phase earlier -- *what is
saturated, and what authority does the vehicle have that nothing commands?*
-- and measured over 722 circling HAC ticks from 126 logs (`logs/LOG27*`,
`logs/LOG28*`; the script is `coneradius.py` in the session scratchpad, kept
below as arithmetic anyone can redo).

| | ticks | bank flown, median | at `HAC_BANK_MAX_DEG` (45) |
|---|---|---|---|
| weave commanded | 345 | **45.0** | **57%** |
| no weave | 377 | 25.0 | 15% |

and the weave command itself, whenever it is on, reads **median 50.0, p90
50.0, max 50.0** -- that is `HAC_WEAVE_MAX_DEG` exactly, on every tick it
fires. So the cone asks for all the weave it is allowed and all the bank it
is allowed, and still hands a surplus down.

**But only one of those two caps is real**, which the next section works out:
the bank clamp sits downstream of the weave, so the 50 degrees the weave
commands is never flown -- about 32 is. `wv=50.0` in the log is a request,
not a measurement of the path.

That matters because the phase below does the same thing: the approach's
S-turn sits at its own cap for 52% of ticks with +639 m unspent (the
measurement that motivated the airbrake). So the chain is one story, not two
-- **the entry hands over a surplus, the cone cannot dump it, the approach
cannot dump the remainder, and the vehicle lands +939 m down a +-1200 m
runway.** Every lever tried against that overshoot so far has been in the
last phase or on the ground: the aim (68, 84), the S-turn stop (84), the
brake law (82), the gear's suspension (83), the gear's deployment
(`GEAR_FOR_ENERGY`, mechanism confirmed, ~150 m), the split rudder (1 m^2 of
`CdA`). The cone's bank redirects the whole lift vector, which is a
different order of authority.

**Why 45 is a policy and not a limit**: the same airframe flies
`BANK_MAX_DEG` 70 in the glide, one phase earlier, and pins itself there.
The cone is rationed to 45 with nothing in the file saying why.

**What the wing can pay for, measured** (`q * ClA / mg` over 634 HAC ticks):
median **1.06 g**, p90 1.36, max 1.91. Holding *altitude* in a turn needs
`1/cos(bank)` -- 1.41 g at 45 degrees, 2.00 at 60 -- so the vehicle cannot
hold a level 45-degree turn now and does not try to: the cone is a
*descending* spiral and more bank means more sink, which is the point when
the surplus is what you are trying to spend. The load ceiling still bounds
how tight the circle can be, and that bound is what `hold_radius` gets wrong
(below).

### `HAC_WEAVE_MAX_DEG` is decorative above about 32 degrees

Worth doing the arithmetic before touching either cap, because the two are in
series and only one of them binds. The cone's bank is
`clamp(forward + HAC_HEADING_KP * error, +-HAC_BANK_MAX_DEG)` with
`HAC_HEADING_KP` 1.2 and the feed-forward measured at a median 6.9 degrees,
so the heading error that saturates the bank is `(cap - 6.9) / 1.2`:

| `HAC_BANK_MAX_DEG` | saturates at | weave actually flyable | path stretch |
|---|---|---|---|
| **45 (committed)** | 31.8 deg of error | **31.8 deg** | 1.18x |
| 55 | 40.1 | 40.1 | 1.31x |
| 60 | 44.2 | 44.2 | 1.40x |
| 70 | 52.6 | **50.0** (the weave's own cap) | 1.56x |

So the configured `HAC_WEAVE_MAX_DEG` of 50 **cannot be flown at all** at the
committed bank cap -- the weave asks for it, the bank clamp refuses, and the
log dutifully prints `wv=50.0` while the vehicle flies about 32. That is
failure 33's clamp shape for the third time in this project, and it means
raising the *weave* cap alone would change nothing, while raising the *bank*
cap alone unlocks weave the configuration already asks for.

The dissipation this buys is the path stretch column: **1.18x today against
1.56x at bank 70**, on the phase whose surplus the approach then inherits.

### The arm this suggests, and what would refute it

`HAC_BANK_MAX_DEG` 45 -> 60, as a ladder if the farm allows (45/55/65),
read on **two** numbers: the surplus handed to the approach (`exc=` at the
first APPROACH tick) and the cross-track at the flare door. The second is
the failure mode: the cone's roll-out distance is what sets the cross-track
the flare inherits (`HAC_ROLLOUT_M` 1500 -> 900 moved `gate=1324 -> 778` and
`cross=+206 -> -37`), a tighter circle changes that geometry, and the flare
door's 70 m split is where this vehicle is destroyed. A batch that lands
earlier *and* wider has not won anything.

### Two diagnostics that are missing, and one model that is wrong

- **Nothing logs the flown turn rate.** `turn=` is re-solved every tick, so
  `d(turn)/dt` mixes flying with re-planning and the flown radius cannot be
  recovered from the logs: the estimate comes out median 7042 m against a
  planned 10800 with a p10-p90 of 0.5-13x, which is not a measurement. One
  `omega=` column would settle which radius law the vehicle obeys.
- **Nothing logs the cone's heading error**, so the split between the
  feed-forward bank and the proportional correction has to be inferred. It
  is worth knowing: the feed-forward `atan(v^2/gR)` is median **6.9 deg**
  while the bank flown is median 33.8, so **the geometry asks for a seventh
  of the bank the vehicle is using** -- the rest is weave and track
  correction.
- **`hold_radius` uses `tan(bank)`, which assumes the wing pulls
  `1/cos(bank)` g.** It pulls about 1.06. The lateral acceleration actually
  available is `n g sin(bank)`, so the honest radius is
  `v^2 / (n g sin(bank))` with `n` measured -- and the `tan` form is
  optimistic at every bank angle, increasingly so as bank rises. This is the
  same shape as `HAC_LD` having to be planned 30% above what the cone
  achieves: a plan that assumes authority the vehicle does not have, with a
  fitted constant absorbing the difference. **Fix the model before raising
  the cap much past 60**, or the plan gets tighter exactly where it is least
  affordable.

### And why 90 degrees is not the limiting case of this

At 90 degrees the lift vector is horizontal: there is no vertical component
at all, the vehicle stops gliding and falls, and it arrives *low and fast*
rather than high. The turn does not tighten the way the formula suggests
either -- lateral acceleration goes as `n g sin(bank)`, so 45 -> 90 buys a
factor of 1.41 in turn rate (a real radius of 9006 -> 6371 m at 1 g and 250
m/s) while the *planned* radius goes 8282 -> 0 and is caught by
`HAC_RADIUS_MIN_M` = 2000. A 2000 m circle at 250 m/s needs 3.2 g laterally
against the 1.91 this wing has ever delivered, and the log would faithfully
print `R=2000` while the vehicle flew something else -- failure 33's clamp,
again.


## Flying at a sideslip: the prediction, written before the data

A forward slip is how a glider with no spoilers dumps energy, and **nothing
in either autopilot has ever commanded one** -- measured over every log on
disk, the vehicle sits at |slip| median 0.8 deg in GLIDE and 2.1 in APPROACH.
The user proposed it; this section is the prediction, recorded before the
probe flights land, so the measurement can contradict it.

`CdA(b) = CdA(0) + (broadside - CdA(0)) sin^2 b`, with `CdA(0)` 6.1 flown
subsonic at aoa 8 and broadside 29 from the swept table:

| held slip | predicted `CdA` | change |
|---|---|---|
| 5 deg | 6.27 | +3% |
| 10 deg | 6.79 | +11% |
| 20 deg | 8.78 | **+44%, +2.7 m^2** |
| 30 deg | 11.82 | **+94%, +5.7 m^2** |

For scale, the split rudder measured about **+2 m^2 (+36%)** and took 21 m/s
out of the approach in eleven seconds. So 20 degrees of slip is worth more
than the airbrake and 30 degrees is worth three of them -- *if the wheels can
hold it*, which is the only thing `SLIP_PROBE_DEG` measures.

**Two ways this prediction can be wrong, and both matter.** The broadside
figure comes from the same probe table that was wrong by 1.8x subsonically
(failure 13), so the *magnitude* is suspect even though the shape is not.
And `sin^2` is the flat-plate form: a fuselage with a wing on it will not
follow it exactly, particularly where the wing starts shadowing the fin.
Which is why the number that decides is `act=` -- the achieved aerodynamic
force, which needs no model and no density assumption.

**Early result, from the coast (q < 500 Pa):** every commanded angle holds
at 96-98% of command, 30 degrees included. That is the regime where the
wheels have no weathercock moment to fight, so it proves the yaw authority
exists and proves nothing about the approach. The ceiling against `q` is
what the probe flights are for, and `slipsum.py` reads it back.

### First correction, from the glide: hypersonic slip buys nothing

Measured on the probe flights themselves, all four at a commanded alpha of
32 degrees and M > 6.5, so the only difference between them is the slip:

| commanded slip | `ClA` | `CdA` | L/D |
|---|---|---|---|
| 5 deg | 9.30 | 11.50 | 0.81 |
| 10 deg | 9.20 | 11.70 | 0.79 |
| 20 deg | 8.90 | 11.60 | 0.77 |
| 30 deg | 8.50 | 11.20 | 0.76 |

**No drag** (+0/+2/+1/-3%, i.e. nothing), and lift down 9% at 30 degrees, so
L/D falls 0.81 -> 0.76. The prediction above is wrong *here* and the reason
is plain once seen: `sin^2 b` assumes a streamwise baseline, and the
hypersonic glide already flies at **32 degrees of alpha** -- the body is
broadside to the flow before any yaw is added, so yawing it adds almost no
projected area. The prediction's regime is the subsonic one, where alpha is
8-12 and `CdA` about 6, and that is what the rest of the probe flight
measures.

Worth keeping either way: **slip is a cheap lift-spoiler at hypersonic
conditions** (-9% `ClA` for -3% `CdA`), which is a different tool from a
brake and might belong to the entry's range control rather than the
approach's.

**Sign convention, measured:** a commanded `slipc=+30.0` reads back as
`slip=-30.3` -- kRPC's `sideslip_angle` runs opposite to a rotation about the
lift axis. The magnitude tracks to a third of a degree; only the sign is
flipped, and any control law that acts on the sign has to know it.

**And the obvious second use of a lift-spoiler is dead on this save.** If
slip spoils lift without adding drag, the place to spend it is a glide that
is long and already out of bank -- failure 87's "pinned at bank 70 when
long". Measured over 23083 GLIDE ticks: **59 of them (0.3%) are at the
70-degree stop.** The glide is not bank-saturated on `qs_plane`, so there is
no opportunity there, and the saturation this project keeps rediscovering is
specifically the *cone's*. One query, one line, one dead end closed.

### The ceiling, measured: it is a saturation in `q`, like the alpha one

`slipsum.py` over the four probe flights (5/10/20/30 degrees commanded, one
per instance, each sweeping `q` on the way down):

| commanded | holds through | partial | lost |
|---|---|---|---|
| 20 deg | **5000 Pa** (101% of command) | 5-7k (74%) | 7k+ (43%) |
| 30 deg | **4000 Pa** (90%) | -- | 4-5k (23%), above (2-8%) |

So the authority is real and bounded, and the bound is exactly the shape
`Holdable` already learns for alpha: `achieved = min(command, holdable(q))`.
The approach's median `q` is about 5300 Pa, which puts **20 degrees at the
edge of what final can hold and 30 degrees comfortably inside the cone's**
(median `q` 4293). Anything built on this has to learn the ceiling in flight
rather than carry a table -- the same argument as everywhere else in this
file, and now with a measured curve behind it.

**If it holds, it changes the order of the whole session's work.** Slip is
*continuously variable*, where the split rudder is a fixed 20-degree deploy,
so it can be commanded in proportion to the surplus and backed off as the
speed margin closes -- which is exactly the failure that destroyed two
vehicles with the airbrake. And it can be spent in the cone, where the
saturation actually is, with 10 km of path and no flare underneath it.


## Sideslip, measured end to end, and the law it earned

The user proposed flying at a sideslip; four probe flights (5/10/20/30 deg
commanded, one per instance, `SLIP_PROBE_DEG`) settled every part of it in
one round. **The idea works, but not for the reason it was proposed, and not
in the regime it was proposed for.**

**1. The authority is there and nothing has ever used it.** The vehicle holds
what it is asked for -- 98-102% of command -- and the bound is a saturation
in dynamic pressure, the same shape `Holdable` already learns for alpha:

| commanded | holds through | partial | lost |
|---|---|---|---|
| 20 deg | 5000 Pa (101%) | 5-7k (74%) | 7k+ (43%) |
| 30 deg | 4000 Pa (90%) | -- | 4-5k (23%), 5k+ (2-8%) |

**2. It is not a brake, at any speed.** Hypersonic (M>6.5, alpha 32): `CdA`
+0/+2/+1/-3% across 5/10/20/30 degrees -- nothing, because the glide already
flies broadside. Subsonic (HAC and APPROACH): `CdA` flat again.

**3. It is a glide-ratio spoiler, and that is better.** Subsonic, at ~15 deg
of held slip, `ClA` falls **15.7 -> 10.9** with `CdA` flat, so

    L/D  1.64 -> 1.23     a quarter of the glide ratio, airspeed untouched

`tan(gamma) = D/L`: less lift at the same drag is a **steeper path at the
same speed**. Compare the split rudder, which spent *speed* (21 m/s in
eleven seconds) and forced the speed loop to dive into the flare door. The
approach's problem is a surplus of height with too much glide ratio to spend
it; this is a direct control on the glide ratio and the speed loop has
nothing to fight.

**4. And it must be straight before the wheels.** The 10, 20 and 30 degree
probes were all destroyed in ROLLOUT; the 5 degree one landed normally.
Failure 34 again -- a crabbed touchdown tears the gear off.

### `APPROACH_SLIP_FOR_ENERGY`, built, tested offline, unflown

Proportional to `guidance.approach`'s own `excess` above a deadband, so it is
inert on profile; ramped at `SLIP_RATE_DEG_S` in both directions; capped by
`SLIP_MAX_DEG` *and* by the ceiling `Holdable` learns live off the yaw axis;
zero at the flare door plus its lead.

**Its ramp test paid for itself before the arm flew.** The first version fell
back to an unbounded step whenever `dt` was zero -- which is the first tick of
every APPROACH -- so the law opened with twenty degrees of yaw in one command,
which is exactly what the ramp exists to prevent.

### Flown once, stopped at one flight, and the flight found a sign bug

`LOG2836`, the first B flight: **stopped +865 intact** against controls at
+900 and +953, and -- the thing that mattered -- **the flare door was
normal**: `h=137.9 v=93.8 sink=35.9 cross=+65` where the unbraked controls
arrive at h 119-139 and sink 29-36. Speed held 96-105 m/s through the whole
slip with no dive. So the currency is right: this spends height without
touching the speed the flare needs, which is exactly what the split rudder
could not do.

**But the law was running at about half strength, by construction.** The
sign was taken from the commanded bank -- and the bank alternates with the
S-turn weave, so the command reversed twice a cycle and spent the approach
ramping through zero:

```
alt=1925  exc=+857  slipc= -8.3
alt=1835  exc=+813  slipc=-12.3
alt=1741  exc=+762  slipc= -6.7     <- reversing
alt=1649  exc=+715  slipc= +3.5
alt=1569  exc=+679  slipc= +9.6
alt=1065  exc=+423  slipc= -4.5     <- reversing again
```

A law asking for 8-12 degrees delivered a mean of **5.9**. That is
structural, not statistical -- it follows from the code -- so the batch was
stopped at one flight rather than spending eighteen on a half-strength
configuration. *Fixed:* the side is chosen once, away from the centreline
the vehicle is already off, and held for the phase. What spoils lift is the
magnitude; every reversal is authority spent going through nothing.

**What would refute it:** the arrival moving earlier but the *flare door*
state degrading -- cross-track outside 70 m, or sink above the 29-36 m/s the
unbraked controls arrive with. Read `landsum.py` for the stop and the
`APPROACH -> FLARE` line for the door, in that order, and remember that this
save's controls land at +900 to +953 intact.


## The slip has two authorities, and the second one was found by wrecking a vehicle

Flown as a pure lift-spoiler with the sign chosen once per approach
(`logs/LOG2838-2846`, four an arm before the batch was stopped):

| | along, median | \|across\|, median | intact |
|---|---|---|---|
| A defaults | **+1122** | 13 m | 2/4 |
| B slip | **+762** | 59 m | 3/4 |

**The along-track mechanism is real and it is not an artefact of the arms
starting differently.** The surplus each flight was handed at its first
APPROACH tick is matched -- A +919/+864/+875, B +802/+911/+927, means 886
against 880 -- so the ~350 m separation is made in the phase where the slip
acts. Speed was held throughout (96-107 m/s, no dive) and the flare doors
were ordinary, which is the whole difference from the split rudder.

**The second authority is much larger than expected.** Over the four slipping
flights the cross-track moves **in the slip's own sign at about 50 m per
degree**:

    mean slip +6.8 -> cross +372 m      mean slip -7.1 -> cross -477 m
    mean slip +8.1 -> cross +367 m      mean slip +9.1 -> cross +411 m

So a *held* sign is a 400 m lateral kick, and `logs/LOG2846` is what that
costs: it began at cross -259, chose a positive slip -- the correct
direction -- and rode it through zero to **+301 at the flare door**,
destroyed 227 m off the centreline. The direction was right; the absence of
feedback was fatal.

*Fixed:* the sign is now feedback on the current cross-track with a
`SLIP_CROSS_DEADBAND_M` band so it does not chatter at the centreline, and
the existing ramp bounds how fast it reverses.

**And this reframes what the control is.** The approach has exactly two
lateral authorities today -- the capture's bank, capped at 40 degrees, and
the S-turn, which is pinned at its own cap for 52% of ticks. Slip adds a
third worth ~50 m per degree that is *paid for in height rather than in
path*, which is the one currency this phase has a surplus of. A mechanism
built to spend energy turns out to be a lateral control as well, and the
phase is short of both.

**What to watch when it next flies:** the cross-track should now converge
rather than diverge -- the failing signature is the monotone
`+80 +102 +152 +215 +268 +297 +302` of LOG2846 -- and the along-track gain
should survive, because the magnitude law is unchanged and only its sign
moved.

## The slip, flown nine an arm: the result, and the lateral bill it comes with

`APPROACH_SLIP_FOR_ENERGY` against committed defaults, `pairfly.sh`, nine an
arm, `qs_plane`, one fresh farm, fingerprint `86c9c646`:

| | mean along | sd | median | on the runway | intact |
|---|---|---|---|---|---|
| defaults | +1263 | 456 | +1129 | 6 of 9 | 4 of 9 |
| `APPROACH_SLIP_FOR_ENERGY=True` | **+790** | **215** | +860 | **9 of 9** | 7 of 9 |

**-473 m of overshoot and less than half the scatter.** The defaults ran off
the end twice (+2055, +2108, both `splashed`); the slip arm's longest was
+1009. The arms are matched on what they were handed -- surplus at the first
APPROACH tick +903 against +838 -- so the separation is made in the phase
where the slip acts.

**The bill is lateral.** Door cross-track median **26 m against 14**, and the
arm's two wrecks were its two highest-slip flights, arriving at `cross=+151`
and `+176`. At ~50 m per degree a sign taken from the offset *now* is still
leaning when the offset reaches zero, so the control **arrives** at the
centreline instead of arresting at it -- `logs/LOG2846`'s failure one order
smaller. *Fixed, unflown:* `SLIP_CROSS_PREDICT` takes the side from the
cross-track predicted at the flare door (`cross + rate * time`), the way
`APPROACH_LATERAL_CAPTURE` already reasons; the capture now publishes
`cross_rate` and `cross_time` so the two lateral authorities solve the same
problem rather than two different ones.

**Do not answer the lateral cost by shrinking the magnitude.** The magnitude
is the energy control and it delivered a mean of 6-8 degrees where 15 was
measured to be worth a quarter of the glide ratio.

Still default off, and **every slip flight so far is `qs_plane`**. Generality
is unmeasured; the batch to run first is `qs_plane_inc` (whose touchdown is
separately broken -- failure 81 -- so read its *arrival* and door state, not
its wreck count), then `qs_e60` with `--set DEORBIT_CENTRE_BIAS_M=51000`,
then a different airframe.

## Flying the entry for drag, and buying the burn with it

Two changes that are one design, both default off, prompted by the question
*what authority does the vehicle have that nothing commands?* asked of the
entry rather than the approach.

### `ENTRY_MAX_DRAG`: the entry's alpha is bounded by a lift argument

`ENTRY_ALPHA_DEG` is 22 because a *range* sweep plateaus at 20-22;
`ALPHA_MAX_DEG` is 32 because "past 30 the lift curve turns over". Both are
lift arguments bounding a phase whose job is to destroy energy. Drag does not
turn over. This airframe's own swept table at entry Mach:

| alpha | ClA | CdA | L/D |
|---|---|---|---|
| 22 (flown) | 6.9 | 5.7 | 1.21 |
| 35 | 9.3 | 14.4 | 0.65 |
| 65 | 4.1 | 27.7 | 0.15 |
| 90 | **0.0** | 28.2 | 0 |

Five times the drag, never asked for. And at 57.9 km the log reads
`aoa=22.0/22.7` at `q=128 Pa` -- the command is tracked exactly, so up there
the authority is free.

**The guard is the plant, not a constant.** The command is `Holdable`'s
ceiling -- live evidence first, `HOLDABLE_PROBE` (LOG1615's wheels-only
curve) as the prior -- so the vehicle is never asked for an angle it has been
seen to refuse. No standing shortfall means no ratchet fighting it and no
monopropellant spent arguing, which is how the whole tank once went for three
kilometres (LOG1616).

**This looks refuted by "High alpha: what the airframe gives and what it will
hold" above, and is not.** That section closed high alpha *on a committed
entry*, which crosses the thin-air band in seconds. A shallow entry is
defined by living in it. Same airframe, same curve, different trajectory.

### `DEORBIT_SHALLOWEST`: the smallest burn that still captures

Every other rule in this file picks the burn by *range*, which is not
monotone in dv -- hence a grid, a window and a tolerance. "Does this burn
commit?" **is** monotone, so it is a bisection: twelve propagations, no
fitted constant, and the answer is a property of the vehicle and the air.

It buys propellant, which this craft does not need and the next one will.
Measured offline (fakeplane, so a hypothesis until the game agrees), one
state, same arrival: **dv 43.23 -> 37.04 m/s, 14% of the burn.**

And the two halves need each other: broadside has `ClA` *exactly zero at
every Mach in the table*, so a broadside entry has nothing to skip on, and it
is the lift that makes a shallow entry bounce. The smallest committing burn
moves **16 -> 12 m/s** when the entry is flown for drag.

**What actually binds the shallowness today is not the skip.** It is
`DEORBIT_MAX_TIME_TO_GO_S` = 1500, the fitted stretch bound this file already
admits is standing in for "do not commit to an entry so shallow the
propagator cannot be trusted". Raising it drops the threshold to ~12 m/s.
Removing it once cost three flights and 95-132 km; the no-lift argument
undercuts the *reason* for it, which makes raising it the next arm and not a
silent edit.

### Where the drag stops and the lift starts: solved, not configured

The first version switched at a fixed Mach 3. That is the shape this project
has a rule against -- and it is the worst possible constant to fix, because
it sets the entry's length and the entry's length sets **where the burn
goes**. So it is solved at the burn, against the propagator, and carried on
the `Steer` as `drag_until` (`guidance.drag_switch_for`). Monotone: switch
early and the entry is long, late and it is short. Bounded below by the
arrival speed the landing chain is sized on, or the solve walks it to
"broadside all the way to the gate", which is the shortest arc and not a
landing.

A closed form will not do this. The equilibrium-glide range equation reads
**647 km where this vehicle flies 2317**: the arc is flown near circular
speed with centrifugal lift carrying it, nowhere near equilibrium.

Solving it turns the ignition point from an instant into a **window** -- ~4
degrees of orbit in the offline sweep -- because the switch trims inside it.

**And on this airframe the switch is nearly inert, which is the finding.**
Shallowest burn, sweeping the switch speed:

```
until=   0 m/s   arc = 2618 km      until=1500   arc = 2618 km
until= 600       arc = 2618 km      until=2000   arc = 2618 km
until=4500       arc = 2842 km      (never broadside)
```

Everything from 0 to 2000 m/s is the same trajectory. The airframe takes the
decision away: the holdable ceiling collapses from 90 to ~25 degrees between
500 Pa and 4 kPa -- about 45 km -- and the vehicle is still doing 2100-2400
m/s when it gets there. **The handover is a plant limit, not a policy**, and
the plant makes it at 45 km. Total authority in the parameter: 224 km of a
2618 km arc.

That is a statement about *this* vehicle, which has **no aerodynamic control
at all** (0 of 6 surfaces with an axis enabled, 15 kN m of reaction wheel
doing everything). On an airframe with working elevons the holdable band
reaches far deeper and the switch becomes a real control. The code is right
and the test vehicle is the degenerate case -- which is an argument for
`qs_plane_heavy` and for a craft file of one's own.

### What the drag is worth, and how it gears

| burn | arc at alpha 22 | arc at max drag | removed |
|---|---|---|---|
| 30 m/s | 2318 km | 1943 km | **375 km** |
| 60 | 1605 | 1529 | 76 |
| 90 | 1325 | 1290 | 35 |

It gears hard with shallowness, because shallow is what keeps the vehicle in
the band where the angle is free. Which is the same reason the two flags
belong to one design.

### What is general here and what is not

| piece | general? |
|---|---|
| bisection on "does this burn commit?" | yes -- monotone predicate, no constant |
| alpha bounded by `Holdable` | yes -- learned from what the airframe gives back |
| `ENTRY_ALPHA_CEILING_DEG` = 90 | yes -- broadside is broadside |
| the switch solved against the propagator | yes -- and it is why the burn and the entry agree |
| `HOLDABLE_PROBE` | prior only; live evidence overrides it |
| `DEORBIT_MAX_TIME_TO_GO_S` = 1500 | **no**, and it is what currently binds |
| `DEORBIT_SKIP_MARGIN_MS` = 2.0 | **no** -- a placeholder that names its replacement (the log's `cutoff drift`) |

## A second airframe, and the five things it found in an afternoon

The user built a Mk3 shuttle -- 30 parts, 37.5 t against the old craft's
6.7-14.5, seven Big-S control surfaces with **every axis enabled** against 0
of 6, two drain valves, best glide **L/D 5.91** against 3.06 -- and the
quicksave was copied into the farm as `qs_shuttle`. It took four flights to
get an entry out of it, and each failure was a quantity that had been right
*by inheritance* on one vehicle.

**None of these were findable on the old craft**, and none of them were
visible to 566 offline tests or to ~2800 flights. This section is the
argument for a second craft, made in numbers.

### 1. `Drain Mode` was never commanded

`ModuleResourceDrain` chooses between draining the part it is bolted to and
draining the whole vessel. The old craft was *saved* with it True, so the
autopilot never set it. The shuttle's valves came in False: both opened, both
drained their own empty parts, the tank never moved, and DRAIN re-entered
forever -- `drain watched down to 1750.0 units`, four times, unchanged.
Probed directly, toggling the mode empties the same tank in **two seconds**.

Fixed: `_drain_whole_vessel` reads the mode and sets it, toggling only as a
last resort -- a toggle applied to an unknown state is how you turn it off.

### 2. `Module.set_field_value` does not exist in this kRPC

It was the documented fallback in *both* drain paths, under the action names,
wrapped in `except Exception: continue`. It has therefore failed silently for
the life of the project, and nothing noticed because the actions always
worked on the one craft being flown. **A fallback that is never reached is a
fallback nobody has tested.** The setters this kRPC has are
`set_field_bool`, `set_field_int`, `set_field_float`, `set_field_string`.

### 3. Isp read from an unlit vessel is zero, and a floor is not a budget

`vessel.vacuum_specific_impulse` aggregates over *active* engines, so with
the throttle shut it reads 0 -- and the drain runs in vacuum before the burn,
which is exactly when it is 0. The old craft happened to report 355 there, so
`drain_reserve_units` ran its rocket equation and kept 96 units ("117 m/s of
burn"). The shuttle read 0, the budget collapsed to the 40-unit floor, the
drain dumped what the burn needed, and the deorbit **ran dry 9 m/s short of
its own solution** -- solved 26.2, delivered 17.4, speed pinned at 2050.1
with the engine commanded on (`logs/LOG2871`).

The log had been saying `at Isp 0` the whole time. Fixed:
`vehicle_vacuum_isp` asks the *engines* (Isp is a property of the engine, not
of whether it is lit), and a fallback to the floor now says so in capitals.
With the fix the drain keeps **430 units** instead of 30 and the burn
delivers 25.7 of 26.6 m/s.

### 4. `ATTITUDE_TIME_TO_PEAK_S` is worth 198 km on this airframe

The committed 3.0 s was fitted to the old craft, and its own comment says
what for: *"the controller is acting through a 2.4 s lateral mode"*. This
craft has no such mode. Flown as a bracket, one flight per instance,
`qs_shuttle`:

| `ATTITUDE_TIME_TO_PEAK_S` | aoa error | aoa sd | slip p-p | along |
|---|---|---|---|---|
| 1.0 | +34.2 | 14.1 | 68.2 | **-262 km** |
| 3.0 (committed) | +15.9 | 15.6 | 95.1 | -180 km |
| 6.0 | **-3.0** | **4.9** | 48.4 | **+17 km** |

**The mechanism is in the same logs as the outcome**, which is what makes
this more than the ordered dose-response failure 27 warns about: at 3.0 s the
vehicle flies 16-34 degrees of alpha *more than commanded*, with sideslip
swinging 95 degrees peak to peak at Mach 7, and `CdA` at 63-72 where the plan
assumed 45. The range error was never a guidance error. **The airframe was
not being pointed.**

Still n=1 an arm. It does not go near a default until it is replicated.

### 5. "RCS cannot help" was measured on a vehicle with no usable RCS

`GLIDE_RCS` is off because LOG1616 measured the whole monopropellant tank
buying 3 km of altitude. Read the torque line of that craft:

    old craft:  reaction wheel 15 kN m    RCS (0, 0, 0)
    shuttle:    reaction wheel 15 kN m    RCS (290112, 42625, 290134) N m

The shuttle flies the hypersonic glide on **the same 15 kN m of reaction
wheel as a craft a third its mass**, several times the inertia, at dynamic
pressures too low for the surfaces to bite -- with 290 kN m of RCS and 429
units of monopropellant switched off. `mono=419.9` does not move for the
whole glide. And the axis that wallows is **yaw**, which is exactly the weak
RCS axis (42.6 against 290).

Unflown. The general form is not a flag but a decision: compare the torque
available against what the airframe is failing to deliver, which the vehicle
already measures every tick.

### The tool this needed, which did not exist

`oscsum.py` -- alpha error, its spread, and peak-to-peak sideslip, per phase.
Nothing in the tree reported whether the vehicle was *pointed*; `landsum.py`
and `glidesum.py` would both have reported only the 200 km. Measured on the
two craft under the committed configuration:

| | aoa err | aoa sd | slip p-p |
|---|---|---|---|
| old craft, GLIDE | -1.5 | 2.8 | 9.2 |
| shuttle, GLIDE | +20.8 | 16.5 | **96.2** |

### And the fifth one had a general form, which is the point

The first reaction to `ATTITUDE_TIME_TO_PEAK_S` being worth 198 km was to
fly a batch at 9.0 s. That is not a fix -- it is a second airframe's trim,
and the third craft would need a third number. The user said so, and was
right.

**A constant in seconds cannot be general; the ratio to the vehicle's own
slew time can.** Angular acceleration is `tau / I`, so the time to swing a
commanded angle goes as `sqrt(I / tau)` -- and kRPC reports both terms
(`moment_of_inertia`, `available_torque`), one of which this autopilot has
been printing in STANDBY all along without anyone joining them up:

| | mass | I (pitch, roll, yaw) | slew, wheels | slew, +RCS |
|---|---|---|---|---|
| old craft | 14.5 t | (34931, 13452, 37195) | **1.57 s** | 1.57 s |
| shuttle | 37.5 t | (2068357, 94184, 2106086) | **11.85 s** | **2.63 s** |

**59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
reaction wheel.** So 3.0 s on the shuttle is a fifth of its own time scale,
which is the +16 degrees of alpha error and the 95 degree sideslip swing.

`ATTITUDE_TIME_TO_PEAK_DERIVED` computes `ATTITUDE_SLEW_FACTOR * sqrt(I/tau)`
**per axis** (first written on the slowest axis only; that over-slowed roll,
the bank control, and cost 37 km in `logs/LOG2879` -- see the 2026-09-22
evening handoff below; `moment_of_inertia` is `(pitch, roll, yaw)` and
`time_to_peak` wants `(pitch, yaw, roll)`). The
factor is 1.91, **read off the craft the constant was fitted to** (3.0 /
1.57), so the law reproduces committed behaviour on that airframe by
construction. That agreement is the reason to believe it and equally the
warning: a derivation that reproduces a fit says the fit was right for *that*
aircraft, and nothing yet about the next.

**What it predicts is testable and untested**: 22.6 s for the shuttle, above
the whole flown bracket (3.0 could not point the vehicle; 6, 9 and 12 all
could). The batch that tests the *law* is derived-against-committed on
**both** craft. The batch that merely fits it is 9.0 against 3.0 on one, and
that is the batch not to run.

**And the same two numbers say the constant was never the real fault.** With
RCS the shuttle's slew time is 2.63 s, near the old craft's 1.57 -- so 3.0 is
about right for this airframe *if it is allowed the actuators it carries*.
`GLIDE_RCS` is off because of a measurement on a craft whose RCS torque reads
`(0, 0, 0)`. Derive the tune from the actuators in use, or use the actuators
the vehicle has; it is one cause seen from two ends.

## Where the mechanisms have come from, counted

Worth recording because the pattern is now four for four and it should change
how a session is spent.

| mechanism | whose | outcome |
|---|---|---|
| raise the cone's bank | user | found a *double* saturation and the third clamp bug in this project |
| fly the approach at a sideslip | user | **-473 m and half the scatter**; an entire unused control axis |
| canards opposed to elevons | user | built; the split rudder's mechanism with the sideslip's currency |
| the split rudder | mine | weakest of the three brakes, and the only one that destroyed vehicles |
| `ENTRY_MAX_DRAG` / `DEORBIT_SHALLOWEST` | mine | correct, and inert on the airframe they were built against |

The two of mine are not *wrong* -- the drag work is right and found its home
the moment a second airframe existed -- but both were chosen from inside the
code, by asking which constant to vary. The user's four were chosen by asking
what the *vehicle* can do that nothing commands, and every one of them opened
an authority rather than tuning one.

**So when the user proposes a mechanism, build it before the thing already on
the list.** The hit rate is the evidence, and the reason is structural: a
search over `config.py` cannot find an authority that was never wired, and
that is exactly the class of thing this autopilot keeps turning out to be
short of.

## Session handoff, 2026-09-22 (evening): a second airframe, the derived attitude tune, and the slip not generalising

Moved here from CLAUDE.md verbatim; fingerprint at the time `b31b6887`.


### The result: there is a second airframe now, and it found five bugs in an afternoon

The user built a Mk3 shuttle and it is in the farm as **`qs_shuttle`**
(copied from `~/Kerbal Space Program/saves/default/quicksave.sfs`; all parts
stock, all present in `base/`). 30 parts, 37.5 t, **seven control surfaces
with every axis enabled** against the old craft's 0 of 6, best glide L/D
**5.91** against 3.06, two drain valves.

It found five things that were one airframe's trim, none of them visible to
566 offline tests or ~2800 flights. Four are fixed, one is a law:

| | what | state |
|---|---|---|
| 1 | `Drain Mode` never commanded -- right by inheritance on one craft | fixed |
| 2 | `Module.set_field_value` does not exist in this kRPC, and was the documented fallback in both drain paths | fixed |
| 3 | `vacuum_specific_impulse` is 0 on an unlit vessel, so the drain reserve took a floor and the burn **ran dry 9 m/s short of its own solution** | fixed |
| 4 | `ATTITUDE_TIME_TO_PEAK_S` is a constant in seconds and cannot be general | replaced by a law, below |
| 5 | `GLIDE_RCS` is off on the strength of a measurement taken on a craft whose RCS torque is `(0,0,0)` | open, unflown |

Full account: docs/spaceplane.md, "A second airframe, and the five things it
found in an afternoon".

### The attitude tune is now derived, and it beat every value I fitted by hand

`ATTITUDE_TIME_TO_PEAK_DERIVED` (default off) computes
`ATTITUDE_SLEW_FACTOR * sqrt(I / tau)` **per axis** from kRPC's
`moment_of_inertia` and `available_torque`.

    old craft  14.5 t  I=(34931, 13452, 37195)      -> (3.0, 1.8, 3.0) s
    shuttle    37.5 t  I=(2068357, 94184, 2106086)  -> (22.4, 4.8, 22.6) s

**59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
reaction wheel.** The factor 1.91 is read off the craft the constant was
fitted to (3.0 / 1.57), so the law reproduces committed behaviour there *by
construction* -- and in flight the old craft's derived value logged exactly
3.0 and landed +1151 m, an ordinary flight for it.

On the shuttle, attitude quality against the hand-flown bracket:

| `TIME_TO_PEAK` | aoa err | aoa sd | slip p-p |
|---|---|---|---|
| 3.0 (committed) | +15.9 | 15.6 | 95.1 |
| 6 / 9 / 12 | +4.0 / -1.5 / -0.6 | 10.6 / 5.4 / 5.0 | 71 / 54 / 53 |
| **22.6 (derived)** | **-0.6** | **1.4** | **24.8** |

The derived value is better than every number in the bracket and would have
been reached on the first flight instead of the eighth.

**Two cautions.** `available_torque` never reports aerodynamic surfaces, so
the derived slew time describes a wheels-only vehicle -- which matters the
moment `ENABLE_CONTROL_SURFACES` (below) is used. And getting the axis order
wrong costs 37 km: `moment_of_inertia` is `(pitch, roll, yaw)` and
`time_to_peak` wants `(pitch, yaw, roll)`; a single figure taken from the
slowest axis over-slows **roll**, which is the bank control and the entry's
only cross-track authority (`logs/LOG2879`, 37 km off the centreline with the
alpha tracked to 1.4 degrees of spread).

### High alpha: the question is answered, and the answer is "it depends on the craft"

`BROADSIDE_PROBE_DEG=90` on the shuttle (`logs/LOG2881`), read with the new
`./alphaceiling.py`, against the old craft's LOG1615 curve:

| q (Pa) | old craft | shuttle |
|---|---|---|
| 250-500 | 89.6 | 80.4 |
| 750-1000 | 71.1 | 45.3 |
| 1500-2000 | 43.9 | 38.5 |
| 2000-3000 | 36.3 | **50.3** |
| 4000-5000 | 25.6 | **60.4** |
| 8000-10000 | 19.4 | **47.4** |

Below ~2 kPa the shuttle holds *less* (59x the inertia, same wheel, air too
thin for surfaces). Above 2 kPa it holds *far more*, and that is the band
where `q * CdA * v` lives. **`ENTRY_MAX_DRAG` was never wrong, it was
homeless** -- the "switch is inert" finding is a property of the old craft.
One flight, small bins; the level is soft, the trend across six bins is not.

### Built and unflown, in the order I would fly them

1. **The combined brake** (`AIRBRAKE_OPPOSED_FLAPS`, `FLAP_BRAKE_PROBE_DEG`)
   -- **the user's mechanism, and fly it on the OLD craft first.** Canards
   deflected against elevons: the pitching moments cancel at the ratio of
   their area times arm, while both groups spoil lift *and* make drag. That
   is the currency the split rudder got wrong (drag alone spends speed, a
   glider dives to get it back, two vehicles died doing it) and sideslip got
   right (spoils lift at constant drag, spends height).
   `airbrake.find_all_brakes` arms the vertical pair **and** both horizontal
   groups -- they cancel on different axes so they compose -- and on
   `qs_plane`'s measured geometry that is **all six surfaces, 6.0 m^2,
   against a whole-craft `CdA` of 5.5**, with every control axis disabled so
   it costs nothing.
   `FLAP_BRAKE_PROBE_DEG` is the instrument and flies first, the way
   `SLIP_PROBE_DEG` did. It has to answer three things geometry cannot:
   does the moment actually cancel (`aoa=cmd/actual` across the deployment --
   the *behavioural* check the split rudder never had, because its
   verification summed `available_torque` and could not fail), what does it
   cost in `ClA` and give in `CdA` (`act=`), and is it therefore a lift
   spoiler or merely a brake.
   **Why the old craft and not the shuttle**: the shuttle's baseline `CdA`
   is 37.3 against the old craft's 5.5 at M 0.3 / alpha 8, so the same
   surfaces are worth ~6x less proportionally there -- and the old craft is
   the one with the unsolved +939 m overshoot.

2. **`ENABLE_CONTROL_SURFACES`** -- the old craft's six surfaces have pitch,
   yaw *and* roll disabled and kRPC exposes all three as writable. The
   autopilot already configures the vehicle it is handed (it locks gimbals,
   disables the nose brake, sets deploy angles, drives the drain valves);
   this is the same act. **It changes the plant and with it every constant
   measured on the old one** -- the stall, `ALPHA_TRACKING`,
   `HOLDABLE_PROBE`, `MARGIN`, the attitude tune (failure 23). Read it with
   `oscsum.py` and `alphaceiling.py`, not the arrival. Prediction: the alpha
   ceiling stops collapsing at 900 Pa and holds deeper, the way the
   shuttle's does above 2 kPa.
   **Note it interacts with (1)**: once the axes are on, the brake is no
   longer free -- it borrows from surfaces doing real work, as on the
   shuttle.

3. `SLIP_CROSS_PREDICT` (on by default inside the still-off
   `APPROACH_SLIP_FOR_ENERGY`) -- the slip's side is now taken from the
   cross-track predicted at the flare door, `cross + rate * time`, with
   `ApproachCommand` publishing `cross_rate` and `cross_time` so the two
   lateral authorities solve the same problem.

4. `HAC_LOAD_MODEL` -- the cone's radius is `v^2 / (n g sin(bank))` with `n`
   the load the wing can pay for (`airframe.turn_load`), not
   `v^2 / (g tan(bank))` which assumes a level turn the cone never flies.
   **This has to land before `HAC_BANK_MAX_DEG` is raised**, or the plan
   gets optimistic exactly where it is least affordable (1.33x too tight at
   45 degrees, 1.89x at 60, 2.8x at 70). The user has explicitly opened up
   raising that cap.

5. `ATTITUDE_TIME_TO_PEAK_DERIVED` on both craft, as a *law* test: the old
   craft is a null by construction and **that null is the test**.

### Refuted, or parked, and do not spend flights defending them

- **`DEORBIT_SHALLOWEST`** -- the bisection on "does this burn commit?" is
  sound (monotone predicate, no fitted constant) but on `qs_plane` it picks
  a *larger* burn than the committed rule, because its clock test is
  stricter than the one it replaces, and its first flight committed to a
  burn outside its own authority window and landed 484 km short. The window
  check is now in. Still default off and it has no proven value; leave it
  idle rather than feed it flights.
- **`ENTRY_MAX_DRAG`'s drag/lift switch is inert on the old craft** -- 0,
  600, 1500 and 2000 m/s all give the identical 2618 km arc, because the
  holdable ceiling collapses at ~45 km while the vehicle is still at
  2100-2400 m/s. That is a plant limit, not a policy. It should come alive
  on the shuttle; untested.
- The split rudder alone: superseded by (1).

### Method notes, paid for this session

- **I quoted failure 27's warning and then made the mistake.** A 1/3/6 s
  attitude bracket at n=1 an arm looked like a 198 km monotone gradient; the
  6.0 s arm then repeated at -59 km having given +17 km. **76 km of scatter
  on one configuration.** What survived was the *mechanism* -- attitude
  error measured within each flight over 100+ ticks -- not the ordered
  arrivals. Prefer a within-flight measurement to an outcome ordering.
- **I edited `config.py` while a batch was flying** and split it into two
  fingerprints (`f1916f8e` -> `7dafb344`). The first ten flights were
  discarded. The fingerprint caught it, which is the mechanism working; the
  fix is to develop in a copy of the tree (`cp -r spaceplane tests
  boosterland` somewhere, plus `testInstances/planeprobeGearup.txt` which
  `fakeplane` reads) and apply the patch when the farm is free.
- **A batch died to memory pressure at 5.7 GB of zram** and I only read the
  number in the post-mortem. Watch it every round; it dropped to 712 MB the
  instant the farm stopped, so it is the farm accumulating and a restart
  fixes it.
- **The mechanisms that come from the user are four for four**, and the ones
  I chose from inside `config.py` are the weak ones. See docs/spaceplane.md,
  "Where the mechanisms have come from, counted". When the user proposes a
  mechanism, build it before whatever is already on the list.

**595 offline tests pass** (`python3 -m unittest discover -s tests`).

### New tools

- **`oscsum.py`** -- was the vehicle *pointed*? alpha error, its spread and
  peak-to-peak sideslip, per phase. Nothing else reported it: the shuttle
  wallowed **96 degrees** of sideslip at Mach 7 and `landsum.py` said only
  that it landed 200 km short.
- **`alphaceiling.py`** -- what angle of attack the airframe actually held,
  binned against `q`. Reproduces LOG1615's published curve exactly from its
  own log, so it is a like-for-like reader for any `BROADSIDE_PROBE_DEG`
  flight.

### The slip does not generalise, and that is this session's clearest result

`APPROACH_SLIP_FOR_ENERGY` was the tree's one measured win -- **-473 m and
half the scatter over 9 an arm on `qs_plane`** -- and default off only
because it had never been flown on a second entry state. It has now been.
`pairfly.sh`, `qs_plane_inc`, 4 rounds, 6 an arm, fresh farm, fingerprint
`7dafb344`:

| | n | mean along | sd | median | \|across\| med | on the runway |
|---|---|---|---|---|---|---|
| defaults | 6 | +497 | 596 | +354 | 3 m | 5 of 6 |
| `APPROACH_SLIP_FOR_ENERGY=True` | 6 | **+913** | **330** | +918 | 12 m | 3 of 6 |

**It lands later here, not earlier -- the opposite sign.** The difference is
+416 +- 278 (t ~ 1.5), so it is not significant either way; what matters is
that the -473 m does not reproduce. Two things *do* carry over: the scatter
halves (330 against 596, matching 215 against 456) and the lateral cost
appears (12 m against 3, matching 26 against 14).

So the mechanism is real -- sideslip spoils lift at constant drag, measured
end to end -- but **the benefit was a property of one entry state's energy
profile.** A lever worth 473 m on one save and nothing on another is a fitted
lever. It stays off. This is what a second entry state is for, and it is the
same shape as the five airframe constants above: something measured
convincingly on one configuration turning out to describe that configuration.

Read the touchdowns on that save as noise: 11 of 12 flights came apart on
contact, which is failure 81 and a craft problem, not guidance.

### Logs of that session

This session's logs are `logs/LOG2868-2903`; the batch files are
`logs/pairfly-slip-inc.txt` (discarded -- config edited mid-batch) and
`logs/pairfly-slip-inc2.txt` (the result above).

## Session, 2026-09-23: the surfaces were never off, and the audit that found it

### The old craft's control surfaces are live, and always were

The 2026-09-21 handoff's headline -- "this craft has no aerodynamic control
at all, 0 of 6 surfaces with any axis enabled" -- is **false**, and a lot of
reasoning since was built on it. What is actually true, measured this
session:

- The save has `ignorePitch = False`, `ignoreYaw = False`,
  `ignoreRoll = False` and `authorityLimiter = 150` on **all six** surfaces
  of `qs_plane` and all seven of `qs_shuttle`. kRPC's typed
  `ControlSurface.pitch_enabled` reads True on every one.
- In flight (`qs_cone`, ~10-15 kPa) `available_control_surface_torque` is
  **95-276 kN m** in pitch -- 6-18x the 15 kN m of wheels.
- The behavioural check, with the reaction wheels **off**: a pitch input of
  +1 against -1 differs by **0.14 rad/s after one second**; with the axes
  explicitly disabled the two inputs give **identical** rates (-0.1005 both).
  The surfaces have authority, and switching them off removes it.

How the error happened, since it is a trap for the next probe:

1. KSP's part menu shows the *ignore* flags under the names "Pitch", "Yaw",
   "Roll", so `Pitch=False` in `Module.fields` means **active**. Both craft
   read identically there -- the "0 of 6 against 7 of 7" contrast was never
   in the data.
2. `available_torque` was read on the pad or in vacuum, where q=0 and every
   surface reports zero.
3. kRPC's `ControlSurface.deflection` reads **0.00 at all times** under this
   install's `SyncModuleControlSurface`, even at 15 kPa with full input -- so
   the one per-surface number that could have disagreed could not.
4. The "Authority Limiter=30" (old) / "37.5" (shuttle) in the module fields
   is the deflection in *degrees* (150% x 20 deg, 150% x 25 deg); the limiter
   is already at its maximum.

**What it changes.**

- `ENABLE_CONTROL_SURFACES` is a no-op on both craft. Drop it from the queue.
- The alpha ceiling falling with q (LOG1615) is not "a constant wheel losing
  to a moment that grows with q". Surfaces and airframe moments both scale
  with q; the wheels do not. So at low q the wheels dominate and the ceiling
  is high, and at high q it asymptotes to the surfaces' **trim limit** --
  about 20 deg on the old craft, 50-60 deg on the shuttle, which is exactly
  the shape `alphaceiling.py` shows on both. It is a property of the surface
  geometry and deflection range, which is why it is craft-specific.
- **The opposed-flap brake is not free.** A deployed surface's deploy angle
  and its control deflection share one travel; deploying all six borrows
  from the only aerodynamic control the vehicle has. The `FLAP_BRAKE_PROBE_DEG`
  flight still answers the question, but `aoa=cmd/actual` degrading under
  deployment is now the *expected* failure, not a surprise.
- The shuttle's and the old craft's attitude behaviour differ for inertia and
  surface-geometry reasons, not because one of them has surfaces.

### The rollout's "+939 m overshoot" is mostly the brake law's own aim

`guidance.brake_fraction` brakes to stop `ROLLOUT_STOP_RESERVE_M` (300 m)
short of the far end, and the runway is +-1200 m from the midpoint, so its
target stop is **+900 m**. The intact stops of `logs/pairfly-slip3.txt`
cluster at +811..+973. The number that describes where the *approach* puts
the vehicle is the **touchdown**, and it is late: 1558-2274 m down a 2400 m
runway on that batch (`rwy=` on the first ROLLOUT line). Read the touchdown,
not the stop, when judging the approach and flare; read the stop only for the
brake law.

A smaller point on the same law: `apply_brakes` writes
`wheel.brakes = 100 * fraction`, but kRPC's `Wheel.brakes` runs 0-200 and the
old craft ships at 200 (the shuttle at 50). Measured across ~210 rollout
intervals, the wheels give about **7-8 m/s^2 per unit fraction** at partial
braking, against the 11 the law assumes (`BRAKE_DECEL_FULL_M_S2`, taken
flat-out at the craft's 200%). The law is closed-loop on distance, so this
mostly delays the braking rather than moving the stop. Not changed.

### Missing-controls audit, as a tool and as a log line

`testInstances/actuators.py <instance> [save]` lists every part module's
fields, events and actions, the torque by source, and the kRPC auto-pilot's
tuning. Diffing the two craft's inventories is how the above was found.
Other differences it shows, none of them commanded by the autopilot:

| setting | old craft | shuttle | commanded? |
|---|---|---|---|
| main-gear `brakes` % | 200 | 50 | yes, overwritten by the brake law (0-100) |
| RCS master in the save | off | **on** | yes, by the RCS valve |
| nose-wheel steering | on | on | yes |
| main-gear steering | off | (no module) | no; mains do not steer on either |
| engine gimbal | free | free | locked at STANDBY |

The flight log now carries two new lines (from the patch applied after the
2026-09-23 shuttle batch): `actuators:` at STANDBY -- surfaces with axes on
and limiter, wheel brake % and steering, reaction wheels, RCS blocks and
master, engine thrust limits -- and `authority kN m ... at q=` on **every
phase change**, torque by source (wheel, RCS, surfaces, engine) beside the
dynamic pressure. The second is the line that would have prevented the
error above: the surfaces' authority is now in every log at real q.

`slew_time_scale` now reads `available_reaction_wheel_torque` by name. It
used `available_torque`, which includes RCS whenever RCS happens to be on at
the moment of the call; the shuttle's save has RCS on, and the derived tune
would have come out 4.5x quicker than the wheels it flies on if the
initialisation order ever changed.

### The shuttle's 100 km overshoot is its deorbit burn, and the missing control is the thrust limiter

First arrival flights of the per-axis derived attitude tune on `qs_shuttle`
(`logs/farmfly-shuttle-derived.txt`, LOG2906-2911). The pointing is now
good -- alpha error -1..+2 deg, sd 4.5-9 in GLIDE, against +15.9/15.6 under
the committed 3.0 s -- and **every flight arrived at the cone 100-125 km long
and 33-43 km to the right.** A consistent bias, not scatter.

The glide is not at fault: `act=` and `mdl=` agree tick by tick, the solve
predicts +86..+118 km from its first tick, and it flies the whole entry
pinned at alpha 32 (the cap) and bank 70 (the cap). The error is made before
the interface. The exit line says where:

    DEORBIT -> COAST range error +0 m, +1.87 m/s still owed, solved dv 27 m/s, delivered 24.8 m/s

**Six of six burns quit 1.9-2.1 m/s short.** The mechanism:

1. The Rhino gives **65 m/s^2** on 30.6 t (the old craft's Cheetah: 8.6). The
   27 m/s burn is four ticks, and the unbalanced thrust on a locked gimbal
   swings the nose 10-15 deg off retrograde (`aoa=180/165`).
2. While it realigns, throttle is held at zero.
3. The floor rule prices "one more floored tick" as `floor x accel x
   horizon`, and `horizon` is the time since the engine last *burned* --
   which now includes ten seconds of realignment. One floored tick "costs"
   13 m/s, more than twice what is owed, so the rule stops the burn.

On this craft 2 m/s is worth ~50 km of arrival (the window moved from
2148-2429 km at the solve to 2227-2554 at cutoff), against ~3 km per m/s on
the old craft -- which is why the same rule was harmless there.

**Two fixes, both default off until flown** (fingerprint `aa66a7c8`):

- `DEORBIT_MIN_BURN_S` -- the engines' **thrust limiter**, which nothing in
  the autopilot ever set, is set at commit so the burn is never shorter than
  this at full throttle: `limit = dv / (MIN_BURN_S x accel)`. At 3 s it is
  ~0.14 on the shuttle and **0.60 on the old craft** -- whose engine reads
  17.8 m/s^2 at the burn, not the 8.6 an older comment claims, so it is
  *not* inert there (I wrote "inert by construction" from the comment before
  a flight corrected it). Restored at cutoff.
- `DEORBIT_FLOOR_ON_TICK` -- the floor rule charges the length of one real
  tick, not the dead time since the engine last burned.

**Flown** (`logs/farmfly-shuttle-burnfix.txt`, LOG2912-2917,
`ATTITUDE_TIME_TO_PEAK_DERIVED` + both flags): every burn delivered its
solution to **0.00-0.04 m/s** (against 1.9-2.1 short), the gate sat inside the
glide's reach at cutoff, and the first round arrived at the cone **+23, +35,
+48 km** against +100..+125. **But the thrust limiter never fired in this
batch** -- at commit the engine is not yet lit, `max_accel` read 0 and the
first version returned without a word. So this batch measures
`DEORBIT_FLOOR_ON_TICK` alone. Fixed afterwards (applied on the first tick
the engine answers, and a failure to set it is logged); the limiter itself
is still unflown.

### A bank reversal taught the learner a false alpha ceiling

With the burn fixed, the shuttle's glide starts on target -- predicted
+374..+526 m for three minutes -- and then walks out to +62 km between 40 and
28 km altitude. The commanded alpha drops from 28 to 16-18 deg there, and the
event log says why:

    aoa=25.5/14.2 bank=+14.0 q=6609
    alpha ceiling -> 24.0 (commanded 24.5, achieving 8.2, q=7095 Pa, learned 24.0)
    alpha ceiling -> 22.0 (commanded 24.0, achieving 9.6, q=7478 Pa, learned 10.6)
    aoa=20.0/22.6 bank=-58.0 q=8085

A reversal from +70 through 0 to -58 deg of bank dips the achieved alpha from
31 to 8 while the vehicle rolls; three seconds later it is back at 22.6. But q
climbs fast through 7 kPa, so the reversal fills a whole `Holdable` bin by
itself, the bin's "highest saturated sample" is a transient, and denser air
"carries the lowest seen" -- the predictor believes 10.6 deg for the rest of
the entry and the guidance flies less drag than it could.

`HOLDABLE_SKIP_REVERSAL` (default off): don't feed the learner while
`guidance.bank_in_transit` says the lean is between the stops, nor for one
pitch `time_to_peak` after -- the controller's own settle time, so 3 s on the
old craft and 22 s on the shuttle with the derived tune, not a fitted number.
The old craft reverses often too, so this is **not** null there by
construction and needs its own pair.

### The shuttle's arrival, three arms of six

`qs_shuttle`, `ATTITUDE_TIME_TO_PEAK_DERIVED=True` in all three, arrival at
`GLIDE -> HAC` (`long=`):

| arm | n | arrival, km | mean | sd | batch file |
|---|---|---|---|---|---|
| derived tune alone | 6 | +100 +104 +108 +108 +118 +125 | **+111** | 9 | `farmfly-shuttle-derived.txt` |
| + `DEORBIT_FLOOR_ON_TICK` (limiter silently inert) | 6 | +15 +23 +34 +35 +40 +48 | **+33** | 12 | `farmfly-shuttle-burnfix.txt` |
| + thrust limiter working + `HOLDABLE_SKIP_REVERSAL` | 6 | +30 +31 +34 +35 +36 +39 | **+34** | **3.3** | `farmfly-shuttle-all.txt` |

The floor fix is the 78 km. The limiter and the reversal gate together
leave the mean where it was and cut the scatter by nearly four. The
reversal gate works as designed -- every flight of the third arm logs
`learned -` through the whole entry -- so the remaining bias is not the
learner.

**What the remaining +34 km is.** The glide holds its prediction at
+0.4..+0.5 km for the first 20 km of descent, then **one bank reversal
between 34 and 26 km (q 3-8 kPa) moves it to +20 km.** Mid-roll the achieved
alpha falls from 30 to 9.6 deg and the `ratchet_alpha` ceiling backs off from
30 to 20 in five seconds (that ratchet is separate from `Holdable`, and not
gated). At that q the surfaces give 2928 kN m of pitch against 15 of wheel,
and the attitude controller is tuned to the wheels' 22 s: a roll about the
velocity vector at 30 deg alpha swings the nose in pitch and yaw, on axes
tuned to be slow. `ATTITUDE_TIME_TO_PEAK_LIVE` (built, unflown) re-derives
the tune from the live torque once a game-second.

Also seen and not chased yet: in the cone the shuttle wallows +-10-13 deg of
sideslip on a ~4 s cycle with alpha achieved 6-29 against 1-9 commanded,
and runs out of height (`needed 3587, had 1997`). Same cause is the first
suspect. And the landing chain still sizes itself on the old craft's
`STALL_SPEED_M_S`, `HAC_LD` and `APPROACH_BEST_LD` (`AIRFRAME_DERIVED` is
off); the log says so on every flight (`airframe DISAGREES ... 41% out`).

### Why the old craft is coming apart on the runway again

Intact rate on `qs_plane` has slid from 14/16 (fingerprint `6debf0c5`) to
4/9 on last session's defaults arm and 0/4 on today's. Tabulated from the
first ROLLOUT line of 22 flights (LOG2847-2864, LOG2924-2932): **every
flight whose rollout opened at `brk=1.00` at ~47 m/s was destroyed or badly
damaged; every one that opened at `brk=0.00-0.09` stopped intact.** Full
brakes at that speed means `guidance.brake_fraction` saw little runway left,
i.e. a late touchdown -- and flat-out braking at that speed is the 83 kN
through two small gear legs that the brake law's own docstring says tears
the aircraft apart. So the late touchdown is now the *cause of loss*, not
just of a late stop, and the thing to measure on the landing is the
touchdown point (`rwy=` on the first ROLLOUT line). Not a result of this
session's flags: the defaults arm is the worse one.

### The three flags on the old craft: harmless

`pairfly.sh`, `qs_plane`, 4 rounds on 3 instances, defaults against
`DEORBIT_MIN_BURN_S=3.0;DEORBIT_FLOOR_ON_TICK=True;HOLDABLE_SKIP_REVERSAL=True`
(`logs/pairfly-oldcraft-fixes.txt`, fingerprint `d7db6186`). Arrival at
`GLIDE -> HAC`:

| arm | n | arrivals, m | mean |
|---|---|---|---|
| defaults | 6 | +213 +442 +447 +495 +497 +766 | +477 |
| three flags | 6 | +92 +479 +480 +491 +503 +510 | +426 |

Indistinguishable -- on `qs_plane` the arrival is pinned at the cone's
`GLIDE_RESERVE_M` saturation, so this pair can only show harm and shows
none. The landing outcomes of this batch are read in the section above
(late touchdown -> full brakes -> breakup), in both arms alike.

**Every orbit on disk, for the floor bug.** Scanning the exit line of every
log since LOG2000 without `DEORBIT_FLOOR_ON_TICK`: ~870 old-craft burns across
`qs_plane`, `qs_plane_inc`, `qs_e20..e80`, `qs_gear`, `qs_soft` -- **none**
quit more than 0.5 m/s short (mean owed 0.03-0.25 m/s, ~1 km of arrival on
that craft, which the cone absorbs). `qs_shuttle`: **15 of 15**, mean 1.52.
It is a thrust-to-weight bug and no entry state of the old craft could have
shown it.

### `ATTITUDE_TIME_TO_PEAK_LIVE`: flown, refuted as built

`qs_shuttle`, with the three fixes above (LOG2936-2941,
`logs/farmfly-shuttle-live.txt`): arrival **-26, -31, -43, -49, -50, -51 km (n=6, mean -42)** against +34
without it; GLIDE sideslip peak-to-peak **63-69 deg** against 23-27. The
`attitude retune` lines show why. The live total includes engine and RCS
torque (time_to_peak read **0.0** during the burn), and kRPC's
control-surface torque depends on the current deflection, so the yaw figure
**chatters 2.3 <-> 10 s** from one second to the next while roll sits at
0.3-0.5 s. A controller that quick in thick air drives the lateral mode the
committed 3.0 s was chosen to damp. The idea -- the controller's time scale
should follow the authority the air provides -- survives; this
implementation does not. A version worth flying would use wheels plus a
*smoothed* surface torque, exclude engine and RCS, and floor the result at
the lateral mode's own period.

`RATCHET_SKIP_REVERSAL` (built, flying as of LOG2942): the other half of the reversal
problem. `ratchet_alpha` backs its ceiling off 2 deg/s during the roll (30 ->
20 in five seconds at 3.4 kPa, LOG2918) and recovers at 0.5 deg/s, and the
solve, pinned at bank 70, could not win back the +22 km. It skips the
back-off inside the same window `HOLDABLE_SKIP_REVERSAL` marks.

### `RATCHET_SKIP_REVERSAL`: the shuttle reaches the cone

`qs_shuttle`, derived tune + `DEORBIT_MIN_BURN_S=3` + `DEORBIT_FLOOR_ON_TICK`
+ `HOLDABLE_SKIP_REVERSAL` + `RATCHET_SKIP_REVERSAL`
(`logs/farmfly-shuttle-ratchet.txt`, fingerprint `0894b2b0`): arrival at the
cone **+1.9, +3.5, +3.2 km** (round one) against +34 without the ratchet gate
and +111 at the start of the session. The glide now holds its +500 m target
*through* the reversals, at ~47 deg of bank -- with authority in hand rather
than pinned at 70 -- and drifts to +1.8 km only in the last 20 km of altitude.

**Which moves the shuttle's frontier to the landing chain**, flown for the
first time from a reachable arrival (LOG2942-2944):

- the cone runs **out of height**: `needed 2724-3514, h=1998`;
- the flare opens at **28-53 m/s of sink** (the old craft: ~5);
- `landsum.py`'s rollout-to-wheels ratio reads **9.2**, where
  `APPROACH_BEST_LD` is the old craft's 4.2 (the log's `airframe DISAGREES`
  line says 5.91 from the swept table).

Every one of those is sized on the old airframe (`HAC_LD`, `STALL_SPEED_M_S`,
`APPROACH_BEST_LD`, with `AIRFRAME_DERIVED` off), plus the cone's sideslip
wallow noted above.

### Across orbits: two new shuttle entry states

`savegen.py --source qs_shuttle -o qs_shuttle_inc --normal 100` and
`-o qs_shuttle_high --prograde 80`, the same recipe as the old craft's
`qs_plane_inc` / `qs_plane_high`; md5 identical on ksp0-2. Best configuration
(derived tune + the four flags), arrival at the cone
(`logs/ladder-shuttle-breadth.txt`, `farmfly-shuttle-inc.txt`):

| save | n | arrivals, km | mean |
|---|---|---|---|
| `qs_shuttle` | 8 | +1.6 +1.9 +3.2 +3.5 +3.5 +3.6 +3.8 +4.5 | **+3.2** |
| `qs_shuttle_inc` | 4 | +2.4 +3.2 +3.6 +4.2 | **+3.4** |
| `qs_shuttle_high` | 2 | +6.1 +11.6 | **+8.9** |

Every burn closed to 0.00-0.01 m/s, including the high save's 66 m/s. The
inclined orbit is indistinguishable from the base one. The high orbit is
the one still out: its glide holds +0.5 km through the reversals and then
drifts to +8..+11 km below ~26 km altitude with alpha at 32 and bank at 70
-- **both caps** -- reversing on the cross-track every ~15 s. (I first read
it as "only 42-50 deg of bank"; those were mid-reversal samples.) So the
high-energy entry is beyond the glide's authority, the same shape as the
old craft's `qs_e60`, which needed `DEORBIT_CENTRE_BIAS_M` to move the aim
(failures 88-89). Compute that knob's gain on this craft before trying it.
**But read this first:** at the solve the window said the gate *was*
reachable -- `the glide can reach 2217 to 2303 km (86 km wide); the gate is
at 2267, +16% off centre` -- and both flights arrived 6-11 km long anyway,
and the post-cutoff window line returned `no answer`. So on this state the
reach model is optimistic by at least the arrival, or the reversals spend
the margin it counted on. n=2; a lead, not a finding.

`ladder.py` note: if any arm carries settings, separate arms with `;` --
with commas, `af:save=qs_shuttle,AIRFRAME_DERIVED=True` became two arms, the
second a save that does not exist, so the `AIRFRAME_DERIVED` probe on the
shuttle is **still unflown**. The help text now says so. And `farmfly.sh`
refuses to start while any autopilot is flying, so three single-instance
farmflys cannot share the farm: use `ladder.py` or `pairfly.sh`.

### `RATCHET_SKIP_REVERSAL` on the old craft: harmless

`pairfly.sh`, `qs_plane_inc`, 4 rounds, the three cleared flags in both arms,
`RATCHET_SKIP_REVERSAL` in B (`logs/pairfly-oldcraft-ratchet.txt`,
fingerprint `0894b2b0`). Arrival at the cone:

| arm | n | arrivals, m | mean | sd |
|---|---|---|---|---|
| three flags | 6 | -2153 -542 -534 -184 -22 +86 | -558 | 751 |
| + ratchet gate | 6 | -1783 -532 -266 +50 +152 +303 | -346 | 699 |

A difference of +212 against a standard error of ~420: nothing, which is
what an old craft whose reversals do not dip its alpha should show. The
landings are failure 81 unchanged -- 11 of 12 destroyed on contact, one
with 3 parts left, both arms, the same as last session's inc batch.
(I first tabulated these as 12/12 intact by reading the wrong column; the
per-flight lines say `0 parts`.)

**So all four shuttle fixes are now cleared on the old craft** -- the three
on `qs_plane` (arrival +477 vs +426) and the ratchet gate on `qs_plane_inc`
-- and on the shuttle together they take the arrival from +111 km to +3.2
(n=8), +3.4 on a second orbit (n=4). They are the candidates to become
defaults. `ATTITUDE_TIME_TO_PEAK_DERIVED` is still off by default too; on
the old craft it reproduces 3.0 s on pitch and yaw by construction, and it
is part of every shuttle arm above.

### Promoted to defaults, 2026-09-23

After each was flown and cleared on both craft, five flags are now the
committed configuration (fingerprint `83c8c65f`; `7521c020` had the first
four):

| flag | shuttle | old craft |
|---|---|---|
| `ATTITUDE_TIME_TO_PEAK_DERIVED` | precondition: at 3.0 s it cannot be pointed | null by construction, flown: -697 vs -686 (qs_plane_inc, n=6) |
| `DEORBIT_FLOOR_ON_TICK` | +111 -> +33 km | null (qs_plane, n=6) |
| `DEORBIT_MIN_BURN_S = 3` | limiter 0.14; sd 12 -> 3.3 km with the next | limiter 0.60; null |
| `HOLDABLE_SKIP_REVERSAL` | (with the above) | null |
| `RATCHET_SKIP_REVERSAL` | +34 -> +3.4 km | null (qs_plane_inc, n=6) |

The derived tune is per axis, so on the old craft roll is 1.8 s where the
constant gave 3.0; the pair above is what says that is harmless.

### The opposed-flap brake (the user's mechanism), probed

`FLAP_BRAKE_PROBE_DEG` 0 / 8 / 15 on `qs_plane`, two flights each
(`logs/ladder-flapprobe.txt`, LOG2980-2985, fingerprint `83c8c65f`). All six
surfaces arm (the vertical pair plus canards at x0.47 against the elevons,
6.0 m^2). Read within flight, by Mach band and *achieved* alpha, medians of
`act=`:

| Mach | alpha | control ClA / CdA / L/D | 8 deg | 15 deg |
|---|---|---|---|---|
| < 0.6 | 12 | 22.4 / 9.3 / **2.41** | -- | 18.6 / 8.9 / **2.11** |
| 4-8 | 20 | 5.2 / 6.2 / 0.84 | 6.3 / 4.5 / 1.42 | 5.9 / 4.4 / 1.34 |
| 4-8 | 24 | 6.3 / 7.2 / 0.88 | 7.4 / 5.7 / 1.30 | 7.0 / 5.5 / 1.27 |
| 4-8 | 32 | 9.0 / 10.9 / 0.83 | 9.3 / 10.0 / 0.93 | 9.0 / 10.3 / 0.87 |

1. **The moment cancels.** GLIDE alpha error -2.0 vs -1.4, spread 3.0 vs 2.8,
   sideslip p-p unchanged: canards against elevons at the computed ratio do
   not upset the vehicle -- the behavioural check the split rudder never had.
2. **Subsonically it is a lift spoiler**, exactly the currency the user
   argued for: at alpha 12 it costs 17% of ClA and 4% of CdA, L/D 2.41 ->
   2.11. The same shape as sideslip (1.64 -> 1.23 at 15 deg), with no
   lateral bill.
3. **Hypersonically it is the opposite** -- L/D *rises* 0.88 -> 1.27 at alpha
   24 -- so it must never be out during the entry. Holding it from COAST is
   why all four probe flights arrived 2-6 km short or worse and broke up; that
   is the instrument, not the brake.

So the brake belongs on the approach only, which is what
`AIRBRAKE_OPPOSED_FLAPS` does (deployed when the approach's S-turn authority
saturates). That is the arm to fly, against defaults on `qs_plane`, read on
the **touchdown point** -- see "Why the old craft is coming apart".

### The opposed flaps as a law: the first thing to move the touchdown

`AIRBRAKE_OPPOSED_FLAPS` was **disconnected**: it armed the flap brake and
`command_airbrake` returned on "no split-rudder pair", so nothing but the
probe ever deployed it (and the flare's stow backstop only stowed the pair).
Wired, tested, then flown -- `pairfly.sh`, `qs_plane`, 4 rounds, defaults
against `AIRBRAKE_OPPOSED_FLAPS=True` (`logs/pairfly-flapbrake.txt`,
fingerprint `83c8c65f`):

| arm | touchdown, m down the 2400 m runway | flare-door sink, m/s | intact |
|---|---|---|---|
| defaults | 1635 1650 1692 1706 1969 1978 (mean ~1770) | 29-36 | **6 of 6** |
| flap brake | 320 695 975 1478 2485 (one lost before contact) (mean ~1190) | **38-58** | **3 of 6** |

It deploys as designed -- `airbrake out at 1222 m: S-turn saturated 75%
with +286 m left`, back in when the speed guard fires -- and it moves the
touchdown **~600 m earlier**, where the aim, the S-turn stop, the brake law
and the gear spring (failures 68, 82-84) never moved it at all. The user's
mechanism is right. But the steeper path reaches the flare door with more
sink than the flare can arrest, and half the vehicles broke up.

The flare's own schedule says how much it can take: `sqrt(td^2 + 2 (L-1) g
h)` at its door `h = FLARE_ALT_M + FLARE_LEAD_S x sink`, whose fixed point on
the committed constants is **~39 m/s** -- the unbraked 29-36 is inside it,
the braked 38-58 is not. `AIRBRAKE_SINK_GUARD` (default off) stows the
brake whenever the sink exceeds that, every number the flare's.

**`AIRBRAKE_SINK_GUARD` as built is a no-op, and the reason reframes the
problem.** Two rounds (`logs/pairfly-flapguard.txt`, stopped between rounds
3 and 4 with nothing in flight): the guarded arm never deployed. The
*unbraked* approach already sinks 46-49 m/s at 1-2 km and only shallows to
~30 at the door, so a guard on the current sink is always tripped. And the
unguarded arm's worst flights did not need a long deployment to go wrong:
LOG2987 had the brake out for **three seconds** (1222 -> 1101 m, retracted by
the speed guard) and still reached the door at 42.7 m/s. That is not the
brake steepening the path; it is `APPROACH_SPEED_PATH` **diving to recover
the speed the brake took** -- the split rudder's failure (LOG2825), smaller.
So the next design is about how the brake and the speed loop share the
surplus -- e.g. deploy only while speed is *above* target by a margin the
brake can spend, or feed the brake's drag into the speed law's
`dv/dt = g sin(theta) - D/m` so the loop does not read it as a deficit --
not a sink threshold. Left off; the guard stays in the tree, off, with this
account beside it.

### `AIRFRAME_DERIVED` on the shuttle: refuted

`pairfly.sh`, `qs_shuttle`, 2 rounds, committed defaults against
`AIRFRAME_DERIVED=True` (`logs/pairfly-shuttle-airframe.txt`, LOG3007-3012):

| arm | cone arrival, km | flare entry, speed / sink | parts left |
|---|---|---|---|
| defaults | +3.9 +5.7 +6.2 | 50-126 / 20-68 m/s | 18, 11, 0 |
| `AIRFRAME_DERIVED` | +8.4 +10.1 +10.6 | **132-135 / 77-94** | 0, 1, 0 |

Sizing the cone and the stall off the shuttle's own table makes both the
arrival and the approach worse. The shuttle's landing chain -- out of height
in the cone on every flight, reaching the flare at 20-94 m/s of sink -- is
the frontier on that airframe and wants a design session flown from a
repeatable final-approach save (`entrysave.py --alt 3000`), not a constant
swap.

### Housekeeping from this session

- `STATE_FROZEN_S` (default 20): LOG3009 broke up in the flare, kept one
  fragment, and kRPC returned the same position at 124.9 m/s for 2800
  game-seconds -- 50 minutes of an instance in a phase with no clock.
  `frozen_early` ends such a flight. Final fingerprint **`64268a58`**.
- `farmfly.sh` refuses to start beside any flying autopilot (by design), and
  it writes `BATCH DONE` to its `OUT` file, not stdout -- a queue trigger
  that watches stdout waits forever. Use `ladder.py` (arms separated by `;`
  when they carry settings) or `pairfly.sh` to share the farm.
- `pgrep -f <pattern>` inside a wait loop matches the loop's own command
  line; test with `ps -eo args | grep "^python3 ./ladder.py"` or a file.

Logs of the session: LOG2906-3012. Batch files: `farmfly-shuttle-*.txt`,
`pairfly-oldcraft-{fixes,ratchet,derived}.txt`, `ladder-shuttle-breadth.txt`,
`ladder-flapprobe.txt`, `pairfly-flapbrake.txt`, `pairfly-flapguard.txt`,
`pairfly-shuttle-airframe.txt`.

## Standing summary as it stood in CLAUDE.md, 2026-09-23 (moved here verbatim)

Until the 2026-09-23 restructure this lived in the root CLAUDE.md and was
loaded into every session.  Its current, condensed form is
[`spaceplane/CLAUDE.md`](../../spaceplane/CLAUDE.md); this is the full text,
unedited except for file paths.

- **`spaceplane/`** deorbits a winged vehicle, flies an atmospheric entry and
  lands it on the KSC runway.

  **Read `docs/spaceplane/journal.md` "Session handoff, 2026-09-20" first** -- the
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
  wheels do not), not wheels losing to the air. docs/spaceplane/design.md,
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
  docs/spaceplane/journal.md, "Session handoff, 2026-09-22 (evening)".
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
  ./spaceplane/tools/quickglide.py -n 3 --instance 0 --timescale 6   # or fly the quicksave
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

  **The other two entry states.** See docs/spaceplane/journal.md, "Session handoff,
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
  off; docs/spaceplane/journal.md, "Flying the entry for drag, and buying the burn
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
  on it.  The first three are fixed.  See docs/spaceplane/journal.md, "A second
  airframe, and the five things it found in an afternoon".
  **Fly every change on both craft from here.**  `oscsum.py` is the new
  reader and the one that found it: alpha error, its spread and peak-to-peak
  sideslip per phase, because nothing in the tree reported whether the
  vehicle was *pointed* -- the shuttle wallowed **96 degrees** of sideslip at
  Mach 7 and every existing summary reported only the 200 km it cost.

  **The shuttle's arrival, 2026-09-23** (docs/spaceplane/journal.md, "Session,
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

  **The user's opposed-flap brake, flown** (docs/spaceplane/journal.md, "The
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
