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

## Session, 2026-09-23 (second): the shuttle's landing chain, from the attitude layer up

Goal: generality -- the shuttle (`qs_shuttle`) had never landed. Every shuttle
flight at the start of the session left the cone "out of height" and was
destroyed or splashed 2.5-7.8 km from the runway (LOG3028-3030).

### A save in the air fell for 56 s before the autopilot engaged

`quickglide.py` wrote the time-scale ceiling (6x) *before* the load and slept
6 wall-seconds for the scene, so `qs_shuttle_cone` (saved at 11971 m, 221 m/s)
engaged at **2189 m in a 127 m/s dive**; all three flights of the first batch
(LOG3031-3033) broke up within 15 s in both arms. Every earlier in-air save
(`qs_cone`, entry saves) paid some of this. Fixed three ways:

- `common/timescale.hold()`: the load runs at 1x when the autopilot governs.
- The harness pauses (`KRPC.paused`) after a one-second clock check, and the
  autopilot unpauses once it has engaged on the recorded state (`unpaused:`
  in the log). Engagement is now ~3 s after the save (11568 m).
- **The time-scale plugin defeated KSP's pause**: it rewrote `Time.timeScale`
  every frame, so `paused` read back True while game time ran on at 1x or 6x.
  `TimeScale.cs` now stands down while `FlightDriver.Pause`; rebuilt and
  installed on every clone (base/ksp0-4 share one hardlinked DLL, ksp5 has
  its own copy).

### The probe is right; the table is untrimmed

`simulate_aerodynamic_force_at` at the vehicle's *live* state matches the
game's force to 1% from Mach 6 to Mach 0.38 (lift and drag, ratio 1.00). But
the STANDBY table is probed with the surfaces neutral, and in flight the
shuttle trims with its elevons: at Mach 0.38-0.45 the flown lift is
**0.72-0.74x** the table at the same Mach and alpha, drag 1.01x, pitch input
+0.63. The old craft's log reads the opposite way (1.34x). So every
`alpha_for_load` on the landing is off in a craft-specific direction.
LiftTrim's subsonic bin (0.42 on the shuttle) is polluted by flare/rollout
samples; the clean figure is 0.73.

kRPC 0.6 has `simulate_aerodynamic_wrench_at` (force *and* torque) and
`ControlSurface.deflection_override`/`deflection`, so the claim in
`ratchet_alpha` and `Holdable` that a pitching moment "is unavailable at any
price" is out of date. A table swept *at trim* -- per Mach and alpha, the
deflection that zeroes the pitching moment, and where none does, the trim
limit -- is buildable from the game's own model. Probe script written
(scratchpad `trimprobe.py`), not yet run.

### The measured alpha was unsigned

`Telemetry.alpha_actual` is the angle between nose and velocity. In the
shuttle's dive from `qs_shuttle_cone` (LOG3035) kRPC read **-1.5, -4.7,
-5.5 deg** while the log said +4.6..+7.4, so a 15 degree tracking error
looked like 5 -- inside `ALPHA_TRACK_TOLERANCE_DEG` -- and nothing reacted.
The log now carries kRPC's signed angle as `aoak=`. `ALPHA_SIGNED` (feed it to
the ratchet) is built and **harmful as built**: negative samples during an
upset collapsed the ceiling to 8 deg in a second (LOG3036). Off.

### The dive is the pitch controller, not trim saturation

In that dive the pitch input was **+0.08** of 1.0 -- nowhere near saturated.
The static tune is `sqrt(I / wheels)` x 1.91 = 19 s of pitch, and kRPC scales
its gains to the *available* torque (surfaces 6000-12000 kN m against 15 of
wheel), so the aero moment that grows with q is left to a slow integral term.
The drain moves the CoM **aft** 2.4 m (fuel is forward), so it is not a
nose-heavy accident.

`ATTITUDE_PITCH_AIR`: pitch only, `ATTITUDE_SLEW_FACTOR * sqrt(I / (wheels +
EMA(surfaces)))`, never engine or RCS, floored at `ATTITUDE_TIME_TO_PEAK_S`
(so inert on the old craft, whose static pitch *is* the floor), never slower
than static. Yaw and roll keep the static derivation.

**Flown in the glide, it costs the entry ~8 km**
(`logs/pairfly-shuttle-pitchair.txt`, stopped after round 2): cone arrivals
+8.5 +14.9 +12.6 +15.4 +18.7 km (mean +14.0) against defaults +6.5 +6.7 +6.3
+5.1 (mean +6.2). No overlap. Now restricted to HAC/APPROACH/FLARE. Landings
in that batch: no flight on the runway in either arm; survivable verticals
(flare door ~85 m/s, 25-33 m/s sink, touchdown sink ~0) appeared in both.

### The shuttle does not turn when it banks

Heading rate against commanded bank in APPROACH (turn rate = g tan(phi)/v,
scratchpad `bankfly.py`): the old craft flies **22-24 deg** of a 37-39 deg
command in the right direction on 84-100% of ticks; the shuttle flies
**3-16 deg** of 34-40, in no consistent direction -- in both arms. The log
carried only the *commanded* bank; `bnk=` is now the flown one, in
`lift_frame`'s sense. The user's warning: the shuttle's yaw authority is poor
(surface yaw 155 kN m against 679 in pitch at the cone entry) and too much
sideslip flips it retrograde -- so the lateral fix is to ask for less (bank
and bank rate bounded by yaw authority, backed off on |slip|), **not** a
quicker yaw tune.

Saves added: `qs_shuttle_cone` (11971 m, 221 m/s -- taken from LOG3028's
cone under the old controller, already at -8 deg alpha in a 33 deg dive: a
recovery test, not an arrival) and `qs_shuttle_final` (2972 m but 63 m/s).

### Roll and yaw were swapped in the tune (failure 96), and fixing it exposed roll

kRPC applies `time_to_peak` as (pitch, roll, yaw), measured by the autotuned
PID gains; the code handed it (pitch, yaw, roll). Corrected
(`ATTITUDE_AXES_KRPC_ORDER`), the bank follows its command on final (-45
commanded, -43.6 flown, where the heading rate had said 3-16 deg of 40) and
the glide's sideslip halves (<=10 deg against 18.5) -- but a hypersonic bank
reversal between 35 and 28 km overshoots (+2 commanded, -75 flown, LOG3051),
the misses ratchet the alpha ceiling to 15 deg, and the cone arrival goes
to **+35 km** (LOG3051-3052, `logs/pairfly-shuttle-axes.txt`, stopped in
round 0). Roll's wheel-derived 4.8 s is far too quick against ~10000 kN m
of surface roll in thick air; the legacy order had it accidentally at
22.6 s. `ATTITUDE_AXES_FROM_CONE` keeps the legacy order in the entry and
switches on reaching the cone -- a phase split, stated as such in
`switch_axes`, until roll in thick air has a derivation.

### The flap brake was half a flap: `Deploy Angle` direction is per part

Measured with `deflection_override`, the `Deploy Angle` field and
`simulate_aerodynamic_wrench_at` on `qs_shuttle_cone` (M0.4, alpha 5,
+/-15 deg): +15 spoils lift on one main elevon pair (-9.1 ClA) and *adds* it
on the other main pair (+4.7) and the forward pair (+5.5). The geometric
brake deployed every surface at +20. That also explains the old craft's
probe, where the "brake" raised hypersonic L/D (0.88 -> 1.27).

`AIRBRAKE_MEASURED`: in vacuum, once, after the harness unpauses, each
horizontal surface is deployed at +/-15 and probed; each keeps its
lift-spoiling sense; surfaces are grouped by the *sign of the moment they
make* (the shuttle's main elevons sit on its CoM and pitch by camber, so
station is the wrong split); the stronger side is scaled to cancel; and the
set is **re-probed and rebalanced on the measured residual**, because
deflections do not add (a set predicted to cancel measured +55-67 CmA).
On the shuttle it arms at **-33.7 ClA (~40% of the lift) with -27 CmA
residual**, within half a single surface's moment. Verified in the game's
model only; its first flights are in `pairfly-shuttle-chain.txt`.

Group-level probe of the same surfaces in the lift-spoiling sense: L/D 0.62
-> 0.54 at Mach 6/alpha 30, 1.10 -> 1.00 at Mach 2, 3.01 -> 1.96 at Mach 0.4
-- a brake at every Mach, so usable in the glide when it saturates long
(not yet wired).

### With the chain in, the cone is the blocker

Chain arm (corrected axes from the cone, pitch retune from the cone,
measured brake): every mechanism fires as designed (LOG3057-3058), the cone
arrival is +2.7/+3.6 km, the bank tracks -- and both flights leave the cone
"out of height" with 211-215 deg of turn left and dive to the flare door at
180-187 m/s. The cone enters at 18-19 km and 282 m/s and its one-sided speed
law bleeds the excess at 18-22 deg of alpha (sink 175 m/s), undershoots to
105 m/s, then dives. Next: `HAC_SPEED_PATH` on top of the chain
(`pairfly-shuttle-hacspeed2.txt`).

Also: three smoke flights ran together on ksp0 for 12 minutes because a
pattern kill (`quickglide.py --save`) matched nothing; LOG3053-3055 are
contaminated past their brake lines.

## Session, 2026-09-24 (third): the shuttle's entry reversals, then the cone

The user's framing: bank is the entry's sink control and the big shuttle
does not want to bank back and forth at hypersonic speed, for want of yaw
authority to coordinate the roll. Flown in kspSim for screening (qs_shuttle,
~40 s a flight, 2-8 at once) and confirmed on the farm (ksp0-2, `pairfly`,
governed `--timescale 8`, frozen tree copies in the scratchpad).

**Two sim caveats learned the hard way.** (1) Sim flights are less
repeatable while the game farm loads the box: the governor paces on wall
time. (2) The sim cannot judge the flap brake at all -- its probe reads
+15 and -15 deg as the same nose-up spoiler, `AIRBRAKE_MEASURED` arms
nothing, and `ControlSurface.deployed` does not move a surface. A sim
"refutation" of the brake (LOG3550-3555) is void.

### Where the reversals were

Counted per phase (sign changes of the commanded bank; "bad" = bank off its
command by >20 deg or |slip| >8):

| | COAST M>4 | GLIDE M>4 | GLIDE M<4 |
|---|---|---|---|
| game LOG3404/3405 | **8** | 3 | 3 |
| sim LOG3416/3417 | **9** | 3 | 3 |

COAST chose the sign with `bank_toward`, no hysteresis, at q 0-120 Pa
(failure 97). `COAST_BANK_LATCH` -> 0 in both. Game n=3 an arm
(LOG3456-3501): arrival **+4.7 -> +8.9 km** -- it leaned the other way first
and the terminal glide could not absorb it. Game glide slip p-p unchanged
(32 -> 32-33); the sim had said 37 -> 26. So the *glide's* reversals carry
the game's sideslip.

### The terminal glide had no sink left (failure 98)

Below Mach 3.5, bank at 70 and alpha at 32 and `long` running +500 -> +8 km.
`ALPHA_MAX_DEG=40` broke the deorbit (a corner of its box). New
`GLIDE_ALPHA_MAX_DEG`: the glide's own ceiling, the ratchet still bounding
it. Sim, latch on, n=4 each: 36 -> +0.52 km sd 0.03, 40 -> +1.3 sd 0.6
without RCS / +0.46 sd 0.10 with, 45 -> +0.06 sd 0.44; RCS without the
ceiling +13.8. The hypersonic lean fell ~50 -> ~32 deg: alpha doing the
energy work. Cost: below Mach 5 the sim vehicle cannot follow bank at 38 deg
(the stability-axis roll needs yaw ~ p sin(alpha)).

`GLIDE_RCS` with `RCS_Q_MAX_PA=20000`: mistracked ticks below Mach 4 ~80% ->
~31% in the sim, and **it emptied the tank** -- nothing after GLIDE closed
the valve (230 of 429 units subsonic). HAC/APPROACH/FLARE/ROLLOUT now close
it (inert on defaults: no default flight spends mono below the glide), and
`GLIDE_RCS_MACH_MIN=1.0` confines it: 117 units, all supersonic.

**Game, the package** (`COAST_BANK_LATCH; GLIDE_ALPHA_MAX_DEG=40;
GLIDE_RCS; RCS_Q_MAX_PA=20000; GLIDE_RCS_MACH_MIN=1.0`), n=3 an arm,
instance-balanced (`game_pkg`, LOG3506-3508, 3540, 3542-3543):

    defaults  +4.5  +6.9  +6.1   mean +5.8   (with the first batch: +5.3, n=6)
    package   +0.38 -0.61 -2.44  mean -0.9

Hypersonic mistracked ticks 29 -> 12-13 of ~136; glide slip p-p not better
(36 vs 32). Both package flights of round 0 lined up (+-160 m) and landed
~4 km short, as the sim's package flights did (-3.5..-4.3).

### The cone: two geometry defects (sim)

The cone approaches the gate straight (R pinned at 16 km) weaving +-45-50
deg to spend surplus. **The weave is `acos(total/available)`, so it is at
its maximum as the path goes to zero**: LOG3524 passed the gate sideways
(gate distance 442 -> 1738 m) and left "out of height". And `hac_turn`'s
wrap band was `HAC_EXIT_TURN_DEG` (12): LOG3520 overshot the tangent by
14.6 deg and read 345 deg to go. `HAC_WEAVE_WHOLE_CYCLE` (weave only while
`total >= speed * HAC_WEAVE_PERIOD_S`) and `HAC_OVERSHOOT_DEG=60`. Either
alone did not do it (6/6 out of height with the band; with the cycle rule
alone the exits read turn 315-348). Together, on the package plus the
attitude chain (`ATTITUDE_AXES_KRPC_ORDER; ATTITUDE_AXES_FROM_CONE;
ATTITUDE_PITCH_AIR`), 12 sim flights: every cone ends at turn 0, flare at
51-100 m/s and +-200 m, touchdown sink 5-12 m/s; **3 intact, 7 damaged
(19-25 parts), 2 lost** -- against ~all lost before. They float: stopped
+3.5..+5 km on a runway ending at +1.2. `AIRFRAME_DERIVED` on top: worse
(0 intact, 4 lost of 6).

### The flap brake as the glide's third control

`GLIDE_FLAP_BRAKE` (`autopilot.glide_flap_brake`): the measured set out
when the brake-*stowed* prediction reads long past reserve +
`GLIDE_FLAP_ON_M`, in when it is back on the reserve; never switched
mid-reversal; stowed at the cone. Its first transit test used
`bank_in_transit` after the rate limiter and read every tick as
mid-reversal -- caught by making it log why it declines. With the 40 deg
ceiling the sim glide never read long enough to use it (max +963 m).
Game batch `game_flaps` (chain vs chain + flaps, glide and approach):
round 0 (LOG3567-3569) -- **the flaps never deployed in either phase**: the
glide never saturated (40 deg ceiling), and the approach brake's trigger is
surplus height, which a game approach that leaves the cone "out of height"
(300-1100 m short at 2 km) does not have. Arrivals -454/+482/+628 m; all
three flared at ~101 m/s with **60-65 m/s of sink** (the sim floats long on
the same code). LOG3567 touched down at sink 10, 73 m off the centreline,
kept 29/30 and ran off the far end; the other two were destroyed. So the
game's cone and approach are worse than the sim's, and that is the next
question (HANDOFF).

Round 1 (LOG3570-3572): A 28/30 and 28/30, B lost -- all **overran into the
water** (+3.2..+6.8 km). The chain in the game, n=6 (the flaps never fired,
so both arms are the chain): 3 kept 28-29 parts, 1 kept 21, 2 lost.

### The approach started 1 km from the runway at 2 km up

The game's cone and the sim's are the same tick for tick (LOG3567 against
LOG3538: speed, `ld=`, `need=` within a few percent), so the game/sim
difference is after the cone. Both reach the gate ~700 m high and overfly
it; the approach then begins at **rwy=1046 m, h=2000, exc=+1222**
(LOG3567) -- a 63 deg path -- saturates its S-turn, dives to 117 m/s and
lands long (sim) or flares at 60 m/s of sink (game).

Two causes. `GATE_DIST_M` is 4000 at `GATE_ALT_M` 2000, a 2:1 final sized
for the old craft; and **`HAC_OVERSHOOT_DEG` broke the lap**: a vehicle
too high to exit at the gate (`HAC_EXIT_SURPLUS_M` 500) is meant to fly on,
wrap, and lap -- with the band it read "arrived", could not exit, and flew
straight on to 2 km (gate distance 6.7-7.9 km at the handover, 4 of 6).
Without the band the laps came back but started from heights that could not
pay for them (3 of 6 out of height mid-lap at the gate): the gap between
"exit" (500 m of surplus) and "a lap" is where the approach's own brake
belongs, as the Shuttle's speedbrake does.

Sim, chain with `HAC_WEAVE_WHOLE_CYCLE; GATE_DIST_M=8000;
HAC_EXIT_SURPLUS_M=1500` (no overshoot band), LOG3585-3590: **6 of 6
"rolled out"**, the approach starting 7.8-8.8 km out, touchdown 35-37 m/s
at sink 8-15, cross -64..+19 m in five of six, stopped +0.5..+2.3 km (3 of
6 on the runway). All but one kept 19/30: 11 parts go 4 s into the rollout
at 10-22 m/s with no sink -- the sim's spring-damper gear, not the landing
(the game kept 29/30 from a 10 m/s touchdown).

**`GATE_DIST_M=8000` is not a default candidate as a constant**: the old
craft's best L/D is 3.06, so a 4:1 final is beyond it. The gate distance
wants to be the airframe's approach ratio times the gate height --
`APPROACH_BEST_LD` is a hand-copied 4.2 shared by both craft. Game batch
`game_gate`: that chain against it + flaps (LOG3591+).

### Generality pass (2026-09-25): the landing numbers derived

The user plans many shuttle variants (passenger, tanker, ISRU explorer:
different masses, slightly different aero); the bar is "fitted constants
only if they apply to almost everything". Replaced, each behind a flag:

- `GATE_FROM_APPROACH`: the gate where the approach's own model needs
  `GATE_ALT_M` (`GATE_ALT_M * approach_ld - TOUCHDOWN_AIM_M`, 6000 m now).
  At 4000 the cone's exit check (`approach_needed`) wanted 1524 m of a
  2000 m gate: every approach started high by construction.
- `HAC_EXIT_SURPLUS_DERIVED`: exit unless the surplus pays for a lap at the
  tightest circle held (`2 pi R / cone_ld`). Every lap begun at the gate had
  run out of height; an S-turn-only allowance (~865 m) sent 4 of 6 sim
  cones round (LOG3597-3602). With the lap rule, 6 of 6 rolled out and 3 of
  6 kept 30/30 (LOG3603-3608).
- `GLIDE_ALPHA_PLATEAU=0.05`: the glide ceiling at the far edge of the wing's
  lift plateau, per Mach, from the table (shuttle ~41 hypersonic, ~31
  subsonic). Fixed 60/90 doubled-to-tripled alpha sd and lost landings
  (LOG3612-3617); the plateau matched fixed 40 (LOG3618-3623; old craft
  +954 m, within its cone).
- `TAIL_LIMIT_ACHIEVED`: the tail cap less the measured pitch overshoot.
  The shuttle flies 3-5 deg above its pitch command; every game touchdown
  lost parts within 0.5 s (tail fin, wing, RCS blocks).
- Flap laws: `HAC_FLAP_BRAKE` (out when the weave saturates with surplus,
  in counting `sink^2/2a` to arrest what it built) and
  `AIRBRAKE_SINK_TRACK` (the approach brake stows when sinking faster than
  the approach wants; LOG3593 dove to 149 m/s of sink on the old stow).

**Game, the general chain** (`game_v2`, LOG3609-3611, 3639-3641; B arm
+ every flap law): **none destroyed** in 6 -- 29, 28, 28, 25, 22 and 8 parts --
but 4 of 6 overran the far end into the water. All three flap laws fired in
game: glide (+2015 -> +21 m in 8 s at Mach 4.9, overshooting below the 500 m
reserve), cone (surplus 2000 -> 1525 in 10 s, stowed at sink 106 with 1144 m
to arrest), approach (out at 854 m, in at the flare door). LOG3611: sink 5.7
at 34 m/s but 310 m off and 21 deg crabbed -- wing and fin lost.

**Still hand-set and blocking the variants:** `STALL_SPEED_M_S`,
`APPROACH_BEST_LD`, `APPROACH_FACTOR`. `AIRFRAME_DERIVED` (+
`APPROACH_LD_DERIVED`) on the new chain lands +5..+11 km long (LOG3627-3638).
The reason is in its own log: STANDBY reads best L/D 5.87 at 0 deg, the
landing-mass re-read 1.46-3.65 at 2-12 deg -- the re-read is the live
*trimmed* vehicle at whatever trim it holds. The general fix is a table swept
**at trim** in STANDBY (the pitch-moment-zeroing deflection per Mach and
alpha; `simulate_aerodynamic_wrench_at` gives the torque), which the session
of 2026-09-23 designed and never ran.

**Gear (the user's rule, default now):** nose no brake and auto friction;
mains `WHEEL_BRAKE_MAX_PCT=200` (game max) and `MAIN_WHEEL_FRICTION=10`
(slider max, `ModuleWheelBase.frictionMultiplier`, decompiled). Before, the
mains braked at the craft file's 50% and the rollout law scaled 100.
Verified in game (LOG3643: "brake 200%, friction 10"); the sim clamps to
100% and has no friction field. Not yet flown on a full landing.

## Session, 2026-09-25 (LOG3644, a live flight on defaults)

The user flew `qs_shuttle` live on **plain defaults** (`72859ef7`): landed
in the water 9.1 km off the centreline, 28 of 30 parts, flare at 17.5 m/s
sink. Their report: roll seemed locked to 0; wobbly at Mach 1-3; deploy full
spoiler when the main wheels touch.

- **Roll locked: the axis swap, still in the defaults.** Line 50:
  `time_to_peak as applied (pitch 22.4, roll 22.6, yaw 4.8)` -- roll on
  yaw's slew time. From ~Mach 0.4 the bank command sat at +-45 (HAC) and +40
  (all of APPROACH, 80 s) while the achieved bank stayed between -15 and +9.
  The vehicle never turned onto the runway: `rwy` 826 m -> 10.2 km,
  `xt` +8.9 km at the flare. HAC exited "out of height" (h 1996 against
  3401, gate 7883 m). The chain flights with `ATTITUDE_AXES_KRPC_ORDER` +
  `_FROM_CONE` track it (LOG3609/3641: HAC +45 commanded, +52..+54 flown).
  **Defaults still fly the swapped order; the corrected one is in the chain.**
- **Mach 1-3 wobble is not the roll bug.** Over Mach 1-3, LOG3644: sideslip
  sd 5.5 (max 11), bank error rms 32, alpha error min -15.6. The chain
  (LOG3609-3611, 3639-3641): sideslip sd 6.4-8.0 (max 12-20), bank error rms
  18-38. The same or worse. Reversals in that band swing the command
  +70 <-> -70 inside 40 s. Open.
- **Ground spoiler (built, default on, unflown):** `ROLLOUT_GROUND_SPOILER`
  deploys the measured spoiler set (`flap_brake`) the tick any braked wheel
  reports `Wheel.grounded`, ROLLOUT as backstop, the largest surface at
  `ROLLOUT_SPOILER_DEG`=25 (the Big-S elevons' `ctrlSurfaceRange`), the rest
  at their measured ratio. Logs the read-back deploy angle. **Needs
  `AIRBRAKE_OPPOSED_FLAPS` + `AIRBRAKE_MEASURED`**; without them it logs
  "no measured spoiler set". Fingerprint -> `03b1dee1`.
- **LOG3645** (user, live, defaults `03b1dee1`): still no roll -- bank -47
  commanded steady, -82..+23 flown at Mach 5; ended at Mach 4.7 on a broken
  kRPC pipe. **`ATTITUDE_AXES_KRPC_ORDER` + `ATTITUDE_AXES_FROM_CONE`
  promoted to defaults**, fingerprint `80468d1f`. The entry keeps the soft
  legacy roll by design (LOG3051); corrected order from the cone on.
- **Entry-roll pairfly** (`80468d1f`, qs_shuttle, ksp0/1, 4 rounds,
  LOG3647-3654). A = defaults (roll fix from the cone): **4/4 destroyed**,
  6.0-9.4 km off, 126-184 m/s -- HAC out of height, then APPROACH dives
  (flare at 300-400 m with 103-143 m/s sink; LOG3644 on the old defaults
  sank ~20). Roll now banks the vehicle and pitch, still on 22.4 s, cannot
  hold the nose: **the roll fix needs `ATTITUDE_PITCH_AIR` with it** (the
  chain always had both). B = `ATTITUDE_AXES_FROM_CONE=False` (fast roll in
  the entry too): 4/4 destroyed, 1.6, 11.5, **34.7, 34.8 km** -- LOG3051's
  +35 km reproduced. **Fast entry roll stays refuted.**
- **Pitch-air vs the chain** (`80468d1f`, LOG3655-3662, 4 rounds, ksp0/1).
  A = `ATTITUDE_PITCH_AIR` alone: **0/4**, all destroyed 7.6-11.8 km off
  (HAC out of height, dive into the flare at 130-190 m/s sink). B = the
  game_v2 chain + flaps: **4/4 landed** (18, 16, 18, 13 parts), within 60 m
  of the centreline, stopped +1956, +419, -4296, +1679 from the midpoint;
  touchdown ~3100, ~1500, -3100 (short, on the grass), ~2800 m from the
  threshold. The ground spoiler fired on all four on main-wheel contact
  (25 fore / ~14 aft). Stopping from ~36 m/s takes ~300 m.
- **Defaults `4fe7eaf0`**: the chain promoted (every B flag), plus the
  near aim (`TOUCHDOWN_AIM_M` 2400 -> 400, `APPROACH_SCURVE_STOP_BY_TIME`,
  `GATE_FROM_APPROACH`). The chain is **unflown on the old craft**.
- **Drag brake, built, off** (`AIRBRAKE_MAX_DRAG`, `AIRBRAKE_DRAG_IN_FLIGHT`,
  the user's idea): every mirrored surface probed on its own (drag, lift,
  pitch, roll, yaw); `airbrake.choose_max_drag_set` (a small simplex) picks
  each deflection -- ground set: max drag + lift dumped; air set: max drag,
  lift held -- all moments held, verified in game. Never the rudder
  (`mirrored_only`). Air set deploys in APPROACH on speed over
  `target_speed` at a height on profile (`drag_brake_fraction`).
- Touchdown along-track scatter is the limiter now: ~6 km over 4 flights
  on a 2.4 km runway. The aim moves the mean; nothing yet moves the spread.
- **Is the landing scatter systemic from the glide? Partly.** Over the 10
  chain flights (LOG3609-3611, 3639-3641, 3656-3657, 3660-3661) the glide's
  cone arrival is **bimodal**: ~2.5 km short at ~12.1 km (3611, 3640, 3657)
  or ~+500 m long at ~14 km (the other seven). The short ones stopped +420,
  +626, +1173 from the midpoint (all on the runway); the long ones +1675 to
  +3039 (all past the far end), bar 3660 (left the cone slow, 63 m/s, and
  fell 3.1 km short). Within a group the stops agree ~+-600 m; the groups
  differ ~1.7 km -- most of the 6 km spread. **The cone passed it on: laps=0
  in all 10**, handing the approach 1000-2500 m of surplus where it needs
  ~2213. Fix it in the cone (lap or flap brake until the exit surplus is
  spent; why `HAC_EXIT_SURPLUS_DERIVED` never affords a lap), not the glide.
  Grouped post hoc on n=10: re-check on the aim batch (LOG3663+) first.
- **Aim batch** (`4fe7eaf0`, LOG3663-3670). A = `TOUCHDOWN_AIM_M=2400` +
  distance weave stop: **4/4 landed** (22, 6, 14, 14 parts; +1829, +577,
  -4968, -5023 from the midpoint). B = aim 400 + time stop: **3/4 crashed**
  (LOG3664, 3668, 3669), one landed 15 parts 600 m short. B left the cone
  with 3491-3907 m (needs ~2213) and dived: flare at 70 m/s of sink from
  220 m (3665, 3668). The near aim cannot be flown while the cone passes on
  its surplus. **Reverted** to 2400 and the distance stop; fingerprint
  `77fe82b1`. The glide-bimodality grouping: A's 3667/3670 left the cone at
  2439-2527 m and fell ~3.8 km short; the high exits went long -- the cone's
  exit surplus, not the arrival, is the variable that reads through.
- **Surface envelope built** (`AIRBRAKE_ENVELOPE`, off; the user's
  "spectrum"): `airbrake.SurfaceEnvelope` + a two-phase simplex
  (`linprog_max`); requests (dClA, dCdA) solved per change, moments held,
  mirrored surfaces only, corners verified and rescaled in vacuum.
  Consumers: approach (drag for overspeed over `ENVELOPE_SPEED_TAU_S`, lift
  cut when the brake law wants), flare (stow), touchdown ("brake" corner).
  First batch: defaults vs envelope, scratchpad `envelope.txt`, LOG3671+.
- **Envelope batch** (`77fe82b1`, LOG3671-3678). Defaults 4/4 landed (25,
  22, 10, 9 parts). `AIRBRAKE_ENVELOPE` 3/4 landed (20, 10, 2), 1 destroyed
  -- but its two worst (3673, 3676) left the cone out of height and the
  envelope only acted at touchdown; not a verdict on it. Vacuum check: the
  rudder excluded; corners' moments 1.1-1.3x the limit (held); the model is
  ~2.2x low on the brake corner (-20/+10 predicted, -45/+22 measured) and
  "drag with lift held" cut 16 of lift -- deflections do not superpose at
  25 deg from 15 deg probes. Big-S Elevon 2 *reduces* drag one way. In the
  approach the requests chattered on a step boundary (LOG3672, 12 writes);
  fixed with hysteresis. Stays off; next is probing more points (a second
  angle, pairs) so the model is fitted, not assumed linear.


## Session, 2026-09-25 (night, roll)

User, live on defaults (LOG3679): "the craft isn't attempting to roll AT
ALL" (GUI roll indicator at zero). kRPC 0.6's controller gates roll on
pointing error: off above `roll_start_angle` 20 deg, full below
`roll_engage_angle` 15; never set by this code. At 36 deg alpha a reversal
slewing at `BANK_RATE_DEG_S` 8 (unsourced) leads the vehicle 20+ deg within
two ticks. Added `ATTITUDE_ROLL_ENGAGE_DEG` 175 (ungated) and
`BANK_RATE_MEASURED` (`rollrate.RollRate`, runtime peak roll rate with a
sideslip tolerance, the user's idea). Farm, qs_shuttle, ksp0/1:
LOG3680 (ungated) landed +994 m but tumbled at Mach 4.8 (bank +-150, slip
+-50); LOG3681 (gated) splashed +2368. LOG3684/3685: the first estimator
(averaged, lead-gated) ratcheted to 1.1 deg/s in GLIDE while the control arm
measured 7.1; LOG3684 still landed (2452 m, -289 m). Rewritten as a probed,
decaying peak; unflown -- batch stopped for the user's live test. Nothing
concluded; every arm n=1.

Later: user flew LOG3690 live (defaults f283628a): "still not rolling at all".
Cause: the glide flap brake deploys the elevons (the roll surfaces); both
losses of control (LOG3680 61713.8, LOG3690 61568.4) began the tick it went
OUT, and its mid-reversal guard returned with the brake still out. Added
`FLAP_BRAKE_YIELDS_TO_ROLL` (glide and cone), unflown. config.py was
truncated by a scripted edit and restored from the previous session's dev
copy (fingerprint 77fe82b1) plus this session's blocks; new defaults
78d872d1.

**Follow-up (LOG3691, user: "still isn't rolling").** Defaults `78d872d1`.
The attitude line reads `time_to_peak as applied (pitch 22.4, roll 22.6,
yaw 4.8)`: `ATTITUDE_AXES_FROM_CONE` kept the legacy swapped order through
the entry, so roll ran on yaw's slow figure and the reversal (-31 -> +27
flown at ~2 deg/s) was flown by yaw with the roll input near zero.
`ATTITUDE_AXES_FROM_CONE` -> False (fingerprint `44553753`); two legacy-order
tests now pin the old order explicitly. Unflown.

Then `ATTITUDE_ROLL_TIME_TO_PEAK_S=1.0` (user: "give roll full authority"): roll on kRPC's default tune, not the derived 4.8 s. Fingerprint `b55c72f8`. Unflown.

## Session, 2026-09-26 (roll oscillation)

User, live on defaults `b55c72f8` (LOG3692): "the roll kept oscillating".
That default flew roll at kRPC's 1.0 s (`ATTITUDE_ROLL_TIME_TO_PEAK_S`, the
user's "full authority" of the night before), yaw on its wheels-only 22.6 s.
Tracked to ~1 deg up to q ~700 Pa; above it the bank swung about a steady
+30 with growing amplitude (16, 48, 34, 15, 35, 44, 7, 59, -5, 64), then
the alpha went (30 commanded, 4 achieved). Failure 99. All farm batches
qs_shuttle, pairfly on ksp0/1 (2 instances), results in the session
scratchpad (`pair-*.txt`); glide sideslip = max |slip| above Mach 1.5.

1. **1.0 vs derived 4.8** (defaults `b55c72f8`, LOG3693-3696, stopped after
   2 rounds): 1.0 destroyed in the entry 2/2 (bank error 125-179, slip
   41-56); 4.8 survived 2/2 (splashed +2.2 km 25 parts; crashed 6 km
   across).
2. **`ROLL_DAMPER` v1** (every crossing of a steady command by 5 deg each
   way slows roll x1.5, relaxes over 120 s) at 1.0 vs 4.8 without it
   (LOG3699-3702, stopped after 2 rounds): 1.0 + damper destroyed 2/2 --
   the damper reached 19.6 s but only after the departure (LOG3699: a
   reversal at q 2200 made +27..+40 deg of slip that held 30 s at a steady
   command; tumble at Mach 4). 4.8 survived 2/2. **The slip, not the bank,
   goes first: yaw on 22.6 s cannot remove it.**
3. **`ATTITUDE_YAW_WITH_ROLL`** (yaw on roll's figure) at 1.0 vs 4.8, both
   damper v1 (LOG3705-3716, 6 an arm): no entry lost in 12. Glide slip
   26-48 at 1.0, **8-19 at 4.8** (against 21-74 with yaw 22.6 in 1-2).
   Damper v1 ran to its 22 s ceiling in every flight: in LOG3710 a +-7 deg
   swing at q 2300-3300 kept its amplitude as the tune went 4.8 -> 20 s --
   an airframe mode, not the loop -- and the lag it bought cost the bank on
   final (LOG3711: +-40 S-turn commands flown 30-60 deg late, 60 m/s sink
   into the flare). **Damper v2: growth only** (a half-swing past a steady
   command must peak 1.1x the one before).
4. **Defaults: roll 4.8, yaw=roll, damper v2 vs yaw on 22.6**
   (LOG3717-3728): landings 4/6 with parts either way; slip 19-28 against
   19-65 (one tumble, LOG3727, 55 km short) -- but >60 deg of bank error on
   **92 ticks below Mach 2 against 33**, three of the yaw=roll flights
   losing the bank in the cone at Mach 0.8-1.1 (LOG3717, 3720, 3721). Fast
   yaw helps at 35-40 deg of alpha and hurts at ~20 deg below Mach 1, where
   RCS is off and the single rudder is the yaw.
5. **`ATTITUDE_YAW_BY_ALPHA`** (yaw = roll / sin(commanded alpha), clamped
   between roll's and yaw's static figure; logged, the knob moves: 7.4 s at
   40 deg, 12.8 at 22, 18.2 at 15) vs defaults `fbe32132` (roll 4.8, yaw
   22.6, damper v2) (LOG3729-3740): bimodal -- 4 flights at slip 10-17 and
   **2 hypersonic tumbles** (LOG3734, LOG3738: slip +7 -> +30 over 25 s at
   Mach 4 about a steady command). Defaults: no tumble, slip 16-27, 9 ticks
   of >60 deg error, 5/6 with parts. Not adopted.

**Defaults now `fbe32132`**: `ATTITUDE_ROLL_TIME_TO_PEAK_S=0` (derived),
`ROLL_DAMPER` (growth only), `ATTITUDE_YAW_WITH_ROLL`/`_BY_ALPHA` off.
Hypersonic tumbles over the session: yaw=roll 0/12, yaw 22.6 2/16, by alpha
2/6 -- not a dose-response at these n, and the one arm with none pays for it
in the cone. The damper v2 has not been flown against its own absence.
Touchdowns still lose parts on most flights (sink 60+ m/s into the flare on
some) -- the handoff's items 1 and 3, unchanged.

## Session, 2026-09-26 (afternoon: the leftover fuel, the CG, a faster shallower final)

The user asked three things: why the propellant left after the deorbit is
not drained; if not, could fuel transfer move the CG; and would a faster,
shallower landing help.

**The leftover.** `DRAIN_RESERVE_DV_MS` 200 x 1.25 for a 26 m/s burn leaves
~376 units (1.88 t, 6.5% of the shuttle) aboard to the runway. The save's
part list says all 1750 units live in one tank, the Mk3->2.5 m adapter at the
nose; the aft Mk3 tanks (y -13/-15) and wing tanks are empty.

**The CG, priced without flying** (`testInstances/cgProbe.py 0
qs_shuttle_final`, new): part masses/CoMs in the vessel frame (the sum closes
on kRPC's CoM to 0.00 m -- a body-frame first version missed by 1.9 m because
the vessel moves while thirty parts are read), and the pitching moment against
alpha at neutral surfaces from `simulate_aerodynamic_force_at`/`_torque_at`,
as a fraction of the surfaces' pitch authority, for several placements of the
1.88 t:

| placement | CG station | flare (70 m/s), worst | Mach 6 slope per 20 deg |
|---|---|---|---|
| nose tank (as flown) | 0.00 | -0.33 | -0.02..-0.04 |
| drained | -0.65 m | -0.19 | ~0.00 |
| aft Mk3 tank | -0.81 | -0.15 | ~+0.01 |
| aft adapter | -0.95 | -0.12 | ~+0.01 |

So transfer buys only 0.16-0.30 m beyond draining and carries the 1.88 t to
the ground; draining is the better answer on this airframe *for the landing*.

**Drained at the first GLIDE tick (`DRAIN_RESIDUAL`, 1 s, both valves):
departed 3 of 3** (LOG3743-3745, `cb8bcf3d`, pairfly ksp0/1 plus a ksp2
flight) -- alpha 35 -> 45-48 at Mach 4-7 bank reversals, slip 70-100 p-p,
19-29 km short. Defaults beside it: 1 of 2 (LOG3742, flown while a third
instance was busy; LOG3746 clean), and 0 of 6 the night before. The table's
Mach-6 column is why: drained the vehicle is neutrally stable in pitch
exactly where the glide flies. Failure 100. Batch stopped after round 1.

**`DRAIN_RESIDUAL_MACH_MAX` = 0.8** (the valve waits; `entry_mass` then
predicts the entry wet). Opens at ~236 m/s at the end of the glide or in the
cone, empties in <1 s. Flown (`de45abf3`, pairfly ksp0/1, qs_shuttle), round
0 only before the session ended: defaults LOG3749 destroyed (0/30),
drained LOG3751 **24/30 on the runway** (+1491 along, -88 across); the
ksp2 save flight LOG3750 glided clean (alpha sd 4.1). Two clean drained
glides, n=1 pair -- no measurement yet.

**Faster and shallower: yes, on the evidence.** The tail strikes at 9.1 deg,
so the flare is capped at ~7 commanded and speed is its only authority
(spare lift ~0.8 g at 100 m/s, ~2 g at 130). Below ~60 m/s the drained wing
cannot hold 1 g at a tail-safe attitude. On `fbe32132` every flare door was a
30-32 deg path at 43-69 m/s of sink (LOG3729-3739); the user's live LOG3741
reached its door at 54 m/s with 37 of sink and touched at 29 of sink, 13/30
parts. The two gentle farm touchdowns (LOG3737, 3739) lost parts to an alpha
overshooting its command by 6-8 deg in the last 2 s (3.4 commanded, 11.5
achieved), past the tail limit. The steepness is surplus: the approach
spends the cone's excess at the *bottom*, by S-turns, brakes and a dive.

Built (**`FLARE_SHALLOW`, off, unflown except two contaminated flights**):
`guidance.flare_door` -- one function for all six places that computed the
door -- is physical when on: the height to bring the sink to the inner
glide's at `FLARE_SHALLOW_PULL_LOAD` 1.5 g, plus `FLARE_TRACK_TAU_S` of lag,
plus `FLARE_ALT_M` of inner glide (a 55 m/s sink at 115 m/s: ~465 m against
the old 192). The flare's sink schedule is capped at
`speed * sin(FLARE_INNER_GLIDE_DEG=5)` with a gentle bottom
(`FLARE_SHALLOW_FINAL_LOAD` 1.15 to 1.5 m/s); the approach flies
`FLARE_SHALLOW_APPROACH_FACTOR` 2.7 / `_DOOR_FACTOR` 2.4 x stall. Six offline
tests (`TestFlareShallow`). Why 5 deg and not the Shuttle's 1.5: at L/D ~3.25
any inner glide decelerates ~2-2.5 m/s^2, so its length is its speed cost.

**The approach save failed twice.** `qs_shuttle_app` (ksp2 only, not in
`saves/`) was taken at 3493 m from LOG3750, whose cone handed over 6846 m
against 2042 needed: both arms flown from it dove vertically (sink 107-113,
LOG3752/3753) -- and those two also flew **while LOG3750 was still flying on
the same instance** (entrysave finishing is not the flight finishing). Void.
The first attempt (LOG3744) was a departed flight. Do not use it.

## Session, 2026-09-29 (the live spin: the reversal's sideslip was commanded)

User, live on defaults `de45abf3` (LOG3758): "entered a spin at around
2000 m/s". Confirmed from the log, and the figure is the right one. **Every
flight, landed or lost, takes 12-24 deg of sideslip and a 10-15 deg alpha
overshoot (35 -> 45-51) in the first bank reversal, at ~2000 m/s, Mach 6.5,
52 km, q ~300 Pa** (LOG3741, 3742, 3744, 3745, 3749, 3758). Most recover.
LOG3758 recovered from that one, then at the second reversal (1680 m/s,
Mach 5.0, 37 km, cmd +34 -> -30) slip climbed 2 -> 38 deg over 25 s while
the bank rolled *away* from a steady command (+130, -144, +160), alpha to
85 deg; the roll damper ran to 22.6 s and never got it back. Fell out
subsonic 49 km short, HAC -> APPROACH at h 1997 against 15380 needed,
flare at 73 m/s of sink, destroyed. Same mass and leftover fuel as the farm
flights (440 units, 30.6 t) -- not a loading difference. Defaults depart
roughly 1 in 7 (LOG3727, 3742, 3758), always seeded by a reversal.

**Mechanism: `aim` commanded the slip.** The nose target was
`v cos(alpha) + lift(bank_cmd) sin(alpha)` -- tilted toward the
*commanded* lift. While the roll lags its command by Δ, that target sits
asin(sin alpha sin Δ) off the vehicle's own pitch plane: a sideslip
command. Lags in the first reversal are 10-23 deg (LOG3741: cmd +21.5,
flown -2.0), which at alpha 31-35 is 7-13 deg of commanded slip against
12-24 flown. Failure 99 and its four follow-ups re-tuned how fast the
loops *chase* this target (roll 1.0/4.8, yaw with roll, yaw by alpha, the
damper) -- null or bimodal each time. This changes the target.

**`AIM_NOSE_FROM_FLOWN_BANK`** (off, 5 offline tests): the nose from the
*flown* bank, the roof to the commanded one -- a stability-axis roll: the
roll loop alone carries the bank change and the nose follows round the
alpha cone with no slip asked of it. Falls back to the command when the
roof is unreadable.

**Flown and refuted** (pairfly ksp0/1, qs_shuttle, `44e63da5`, stopped
after round 1 on the stopping rule): the flag 2/2 departed -- LOG3760 slip
26 deg in the first reversal, tumble at Mach 3-4, 28 km short, destroyed;
LOG3761 slip 31, 19 km short, destroyed. Defaults LOG3759 beside it: slip
**26** in the same reversal and landed (+456 long). So with *zero*
commanded slip the reversal slip is unchanged or worse: the commanded
component was not the source. It is the body roll outrunning a 22.6 s yaw
at 35 deg of alpha -- and the lead toward the commanded lift was starting
the nose round the cone early, i.e. helping. LOG3762 is a killed partial.
Five attempts on the loop and the target now; **the untried levers act on
the rate or on a different actuator**: slew the bank command in the
hypersonic reversal at what yaw can coordinate (roll rate x sin alpha <=
the measured yaw rate -- `rollrate` measures roll only), or put RCS on yaw
for the reversal instead of waiting for 5.8 deg of pointing error.

## Session, 2026-09-29 evening (yaw on the thrusters; the landing is an energy problem)

**The missing actuator was already open.** The shuttle's RCS makes 290 kN m
of yaw against the reaction wheels' 15 (STANDBY torque lines), and
`GLIDE_RCS` has the valve open through every hypersonic reversal -- but
yaw's `time_to_peak` (22.6 s) is derived from the wheels alone, and kRPC
schedules the nose's target rate from it. So the thrusters were asked for a
wheel's worth of yaw while roll (4.8 s) outran it. **`ATTITUDE_YAW_WITH_RCS`**
(built, off): while the valve is open in GLIDE, yaw's figure is
`ATTITUDE_SLEW_FACTOR * sqrt(I_yaw / (wheel + rcs))` (4.3 s, clamped to
roll's 4.8), static again the moment the valve shuts -- so below Mach 1,
where yaw-on-roll's-figure lost the cone (`ATTITUDE_YAW_WITH_ROLL`), nothing
changes. The valve is held open while the bank command leads the flown bank
by more than `BANK_RATE_SAT_DEG` (`rcs.Valve.update(hold=True)`).

**Trap paid: kRPC reports no RCS torque with the valve shut** (probed on
ksp2: 290 kN m open, 0 shut). The first version read it at engage, where the
valve had just been shut, and flew as the defaults (LOG3764; batch killed).
It is now read on the first ticks the valve is open.

**Flown** (pairfly ksp0/1, qs_shuttle, `e1a9838c`, 6 rounds): peak glide
sideslip above Mach 2 **8-12 deg on all six flag flights, 14-57 on all six
defaults** -- no overlap; departures 0/6 against 1/6 (LOG3787, 56 km short).
Arrivals with the flag: +489..+595 m, all six (defaults +116..+546 and
-1556). With the flag the shuttle crosses the threshold at 26-611 m and
touches down 35-1351 m in -- on the runway -- but steep and fast (sink
22-70 m/s at 82-118 m/s); 5/6 broke up (LOG3781: on the runway, 22/30
parts). Defaults cross at 388-1447 m and land long. Logs LOG3766-3794 (A:
3766 3769 3775 3782 3787 3794; B: 3767 3768 3776 3781 3788 3793).

**The landing misses are energy on final, not the flare.** Over LOG3720-3779
the approach spends a near-fixed 2.4-3.2 km of height in its ~6.7 km however
much the HAC hands it (needed ~2.2 km; the exit tolerates up to a lap's
worth, `hac_exit_surplus`), so HAC exits at ~3.1 km cross the threshold at
150-460 m and land on the runway, at 3.5-4.2 km cross at 600-1400 m and land
1.8-3.6 km in, and above 5 km are lost. The brake cannot help as wired: on
LOG3775 (1.15 km high) it came out four times for 0.2-1.4 s, stowed each
time on "speed 108 below target 108" or "sink 39 above the 39 the flare can
arrest" -- a drag brake can only spend height as speed or as path, and at a
fixed 108 m/s both are forbidden.

**Built, off, offline-tested:**
- `APPROACH_SPEND_AS_SPEED`: surplus past `APPROACH_SCURVE_M` lowers the
  approach's target speed to `sqrt(target^2 - 2 g surplus)`, floored at the
  flare's door speed (`guidance.spend_as_speed`); the floor follows. ~235 m
  spent as speed, and at 84 m/s the same 39 m/s sink limit allows a 28 deg
  path against 21 -- about 1 km more over the final.
- `FLARE_TAIL_BY_ATTITUDE`: the flare capped its *angle of attack* at the
  tail angle (9.1 deg) at every height; the tail strikes by *attitude*, which
  is alpha less the descent (25-35 deg at the door). Cap = tail + descent.
  And `aim_runway` pitched to `alpha + descent` (`AIM_RUNWAY_TRUE_ALPHA`) --
  a sign error delivering alpha plus twice the descent, masked by the tail
  cap on pitch (gentle flares fly 7-9 deg over command, LOG3730, 3737); now
  `alpha - descent`.
- `FLARE_LEAD_BY_RESPONSE`: the sink schedule read at `h - T sink`, T the
  pitch axis's own derived response time (`attitude_settle_s`).

**A new save, and why it did not measure the landing.** `qs_shuttle_low`
(14913 m, 231 m/s, late GLIDE, from defaults LOG3765) is highly repeatable,
but the airbrake is measured in vacuum, so from it the spoiler and flap brake
are **never armed** ("not measured, not armed"): every flight splashed 2.6-4.4
km long with both flare flags and without (LOG3770-3796). It measures the
flare law in isolation only; landing work still has to fly from orbit.

**Yaw flag alone vs yaw flag + the three landing flags** (`2c1fa02b`,
pairfly ksp0/1 6 rounds + ksp2 alternating 4; A: LOG3797 3798 3802 3804 3806
3808 3809 3810 3814 3816, B: 3799 3800 3801 3803 3805 3807 3811 3812 3813
3815). **Five departures in 19**, and in 9 of 19 the slip first passes 15
deg between Mach 0.9 and 1.4 -- where `GLIDE_RCS_MACH_MIN` shuts the valve
(LOG3802: "rcs off (err 10.8 deg)" with 20 deg of slip) and yaw snaps from
4.8 s back to 22.6 mid-recovery; the damper then counts six growing swings.
The defaults had no transonic trouble in the batch before, so **the yaw
flag made this**. Fix (built): under the flag an open valve stays permitted
below the Mach floor until its own relay settles.

The flare's outcome is set by the sink it is entered at, in both arms:
entered at <=25 m/s it touched down at -2..+13; entered at >=45 it hit at
38-80 (LOG3801, 3803: door at 163-169 m, 46-48 m/s, touchdown 3.5 s later
at the same sink). The door (`50 + 2.5 s * sink`) is lower than the pitch
response plus the pull-up. LOG3799 (package) is the gentlest touchdown on
record for this craft: flare at 63 m/s, touchdown sink -0.3, 48 m/s, on
the runway 51 m off the centreline, 24/30 parts.

**`FLARE_DOOR_FROM_RESPONSE`** (built, off): door = `FLARE_ALT_M + T sink +
(sink^2 - td^2) / (2 (FLARE_TRACK_LOAD_MAX - 1) g)`, T = `attitude_settle_s`
published on `env.pitch_response_s` each tick; one function, so the speed
profile, brake stow and S-turn stop move with it.

**Defaults vs the whole package** (yaw + handover, flare attitude, lead,
spend-as-speed, derived door; `baf131f9`; stopped after 8 package flights,
LOG3817-3832): **worse** -- 0/8 package flights usable, six destroyed at
59-149 m/s, three 650-2350 m off the side. LOG3832: out of the HAC short
(299 deg off, 2 km up), dived at 90 m/s of sink, the raised door opened the
flare at 615 m and 149 m/s and the flare asked for 2-4 deg (its table
over-reads trimmed lift) -- no arrest. LOG3824: 2 km high at the approach,
spend-as-speed slowed it to 70 m/s and it still S-turned over the
threshold. LOG3819: the door arrested 62 -> 7.5 m/s of sink (it works when
there is height) but the vehicle was 1.5 km off the centreline. **Lesson:
five changes in one arm; fly them one at a time.**

**Why gentle touchdowns break up.** Wings are within 1-4 deg of level at
the last flare tick; the roll comes *after* contact (+25..+57 deg 1-2 s in,
three ended inverted). The first part lost, 0.1-1.4 s after contact, is at
the back/underside on nearly every flight: aft RCS block 7/12, else tail
fin, engine, docking port. Craft file: the lower aft RCS blocks sit 2.3 m
behind the main gear and *below* its mount (y 8.72 vs 8.92), beside the
3.75 m engine bell. The flare commands ~7.3 deg (the tail cap, 0.8 x 9.1)
but flies 12-18 (the `aim_runway` sign error), and as the sink goes to
zero the attitude becomes that alpha -- above the 9.1 deg tail angle, before
gear compression. **Refuted on the way:** the ground spoiler rolling it --
`AIRBRAKE_SPOILER_LATERAL_CHECK` measured the set at roll -0.00, yaw -0.00
(LOG3836).

**The user's landing rules, built (off):** `ROLLOUT_BRAKE_FULL_ON_CONTACT`
(mains 200% from the tick they are `grounded`, held; nose none -- the
rollout had been giving 0-24% of 200% at contact under
`BRAKE_FOR_DISTANCE`), `ROLLOUT_STEER_PID` (P on cross-track, D on measured
drift = steer on the position `ROLLOUT_STEER_LOOKAHEAD_S` ahead, small
clamped I; the P-only law weaves and stops 30-40 m off). "Flare less, fly
faster" was flown as `FLARE_TAIL_BY_ATTITUDE=True;TAIL_STRIKE_MARGIN=0.5;
APPROACH_FLARE_FACTOR=2.1;APPROACH_FLARE_FLOOR_FACTOR=2.0;
ROLLOUT_BRAKE_FULL_ON_CONTACT=True` -- one flight before the session ended
(LOG3839: destroyed at 80.6 m/s, 1.4 km along; not a result).

**Yaw flag with the Mach-floor fix vs defaults** (`37ba2429`, 2 of 6 rounds,
LOG3837-3844, stopped for the session end): transonic slip still builds on
the flag (LOG3838 33 deg at M1.6, LOG3844 41 at M1.2). Cause found: `run_hac`
calls `set_rcs(False)` on its first tick -- "rcs off (err 22.9 deg, q 2481
Pa)" 0.2 s after GLIDE -> HAC -- and `rcs_yaw_now` applies in GLIDE only, so
yaw snaps back to 22.6 s at HAC entry mid-slip. The Mach-floor fix covered
the GLIDE half of the same handover.

## Session, 2026-09-29 night -> 09-30 (yaw-RCS becomes default; the flare is the common killer)

**`ATTITUDE_YAW_WITH_RCS` is a default** (`c31c3bea` onward). The HAC
handover fixed first: `run_hac` now keeps an open valve permitted (it used
to shut it on the first tick) and `rcs_yaw_now` applies in HAC as well as
GLIDE. Pairfly qs_shuttle 6 v 6 on ksp1/2 (`23e33e7b`; A defaults LOG3846
3850 3852 3856 3858 3862, B flag 3847 3849 3853 3855 3859 3861): hypersonic
peak slip **8-13 deg on all six flag flights, 15-70 on the defaults**; the
defaults departed once (LOG3850, 70 deg, 54 km short -- the user's live
spin), the flag never; two flag flights landed 30/30 (LOG3853 795 m off the
side, LOG3859 1.9 km long). The valve now stays open into the cone until
its relay settles (LOG3847: shut 133 s into HAC). Transonic bank overshoot
(cmd 37, flown 80-106, LOG3859) is present on both arms at similar size --
a separate roll problem, not this flag's.

**The aft-RCS craft variant** (`saves/qs_shuttle_rcsup.sfs`: qs_shuttle with
the two lower aft RV-105 blocks rotated 45 deg about the long axis to the
sides, x=+-1.81 z=0; RCS yaw torque 266 kN m vs 290) flown solo on ksp0
alternating with qs_shuttle, 3 v 3 (A LOG3848 3857 3860, B 3851 3854
3863): **inconclusive** -- every touchdown in both arms was hard (9-17 m/s
sink) and the first part lost was a delta wing, tail fin or the pod, not the
aft RCS. The craft geometry cannot be judged until touchdowns are gentle.

**Measured: why the landings fail, in order.**

1. *The approach dives.* Approaches that S-turn (|bank| > 30 on 50-100% of
   ticks) track commanded alpha 2-7 deg rms worse than straight ones (0.5-1.4)
   and dive: LOG3849, 3852, 3864, 3870 reached the door at 130-190 m/s and
   90-130 m/s of sink. The S-turn's bank is a +-40 relay reversing every 4-8
   s against a ~12 deg/s roll; the vehicle never reaches the command and the
   pitch loop loses alpha in the reversals. (An earlier reading that "the
   table over-reads lift 3x" -- LOG3832 act 20-34 vs mdl 62-73 -- was an
   artifact: `mdl=` is evaluated at the *unsigned* alpha; with sideslip the
   signed alpha was ~0. `LIFT_LOOP` flown, n=2: it learns only +-2 deg.)
2. *The cone hands over 1-2 km high* (conesum, 35 flights): the glide
   arrives over the field at 13.7-15 km (the 3 km distance backstop fires
   before `HAC_ALT_M`), the cone's plan saturates at R=16 km (~16 km of
   path), it flies ~20 km at L/D 1.9 and exits at 3.5-4.4 km against 2.2
   needed. **The cone's flap brake never deployed** on any of these flights:
   `hac_flap_brake` waits for the weave to pin at `HAC_WEAVE_MAX_DEG` (50) and
   the weave sat at ~44. A disconnected brake.
3. *The flare over-lifts, floats and stalls out* -- on every arm and on the
   old craft. `aim_runway` pitches to alpha + descent (the sign error found
   last session): LOG3869 (qs_plane) commanded 6 deg at the door and flew
   15.6, levelled at 65 m, bled 85 -> 45 m/s and fell 17.7 m/s. On the flare
   bench (qs_shuttle_low, 6 per arm, LOG3882-3905) the defaults stalled out
   6/6 (min speed 34-37 m/s, 27-28 parts, docking port first -- a nose slam).
4. *Wings not level at contact.* LOG3855: 0.04 m/s of "sink" on the FLARE ->
   ROLLOUT line, but the vehicle arrived at the door with -51 deg of bank
   flown (0 commanded, S-turn still reversing at 208 m) and touched down at
   -36: wing lost. The FLARE -> ROLLOUT sink is read after the bounce.

**The flare bench** (`qs_shuttle_low`, ~2.5 min a flight; 4-arm rotation
over ksp0-2, `440649ed`):

- defaults: stall-out 6/6, 27-28 parts.
- `FLARE_TAIL_BY_ATTITUDE` (the sign fix + attitude cap): never stalls
  (min speed 58-84). 30/30 on 3/6 -- all three from doors at <= 70 m/s;
  doors at 74-87 m/s fly ~4 deg of alpha to the ground at 20-24 m/s sink,
  16-21 deg banked (22-24 parts).
- + `FLARE_EXP_TAU_S=4` (new: wanted sink <= 2 + h/4): 4/6 intact.
- + `FLARE_LOAD_LOOP` (new): worse (23-29) -- it *reduced* alpha, because the
  schedule asked for less than one g; the loop did what it was told.
- + `APPROACH_BANK_BY_ROLL` (new, bench 2 LOG3906-3926): bank at contact
  -5..+2 (from -12..-20), and lands ~1.6 km along instead of 2.6-4.4 (the
  S-turn can no longer spend late) -- but doors at 92-102 m/s where the
  flare commands 2-6 deg, achieves ~2 and contacts at 8-12 m/s sink, pitch
  -1..+3 (`contact:` line). The flare is too late for a 3 s pitch response:
  at 72 m with 25 m/s of sink the exponential schedule asks 1.24 g.

**Traps paid:** the `mdl=` field is evaluated at the unsigned alpha (compare
`act=` against it only with small sideslip); stopping the farm while a
rotation's last round is still up voided LOG3927-3929; killed partials
LOG3879-3881.

**Flare bench 3** (2x2 on `FLARE_TAIL_BY_ATTITUDE` + `FLARE_EXP_TAU_S=4`,
LOG3930-3953; LOG3951, 3952 void -- clobbered by a rotation started on top
of a running one): base 4/5 intact (with bench 1: **8/11**, against 4/11 for
the sign fix alone and 0/6 for the defaults); + `FLARE_DOOR_FROM_RESPONSE`
3/6; + `FLARE_LEAD_BY_RESPONSE` 4/6 (two at 10-11 parts); both 4/5. Neither
the raised door nor the lead is distinguishable from the base at this n.
The `contact:` line does not fire from `qs_shuttle_low` (a mid-air save
never set the gear up, so no main wheel is known) -- read the FLARE ->
ROLLOUT sink there.

**The old craft had quietly stopped landing.** `qs_plane` had not been flown
since the shuttle chain became the defaults (2026-09-25); on the current
defaults it contacted at 15-21 m/s and kept all 23 parts on 1 flight of 12
(LOG4014; 3877 kept 21; destroyed or broken: 3869, 3875, 3958, 3960, 3962,
4012, 4016, 4024, 4026, 4028). And
`qs_cone` (old craft at 12 km) is no bench any more: the cone runs "out of
height" 5.4 km from the gate on every flight, defaults included (LOG3967
...3984), and dives in at 140 m/s.

**Flare bench 5** (`14dc04c2`, LOG3992-4009): defaults stall out 5/6 (27-28
parts); `FLARE_TAIL_BY_ATTITUDE` + `FLARE_EXP_TAU_S=4` +
`FLARE_DOOR_FROM_SCHEDULE` (new: the door where the exponential schedule
starts to bind, `tau (sink - td) + T sink`, and the schedule read T ahead)
**5/6 intact 30/30** (the sixth 28), arrest at 20-27 m, bank <= 3.4 at the
last tick. + `APPROACH_BANK_BY_ROLL` 0/6: the capped S-turn spends less,
doors at 87-97 m/s, and there the flare never flies more than ~4 deg --
commanded 5-8, a standing 3-4 deg pitch deficit for eight seconds (LOG4003).
The earlier "fast doors don't pull" flights were the same thing, plus (on
the arms without bank-by-roll) doors entered 12-28 deg banked, where raising
the nose in the vertical plane needs body *yaw* -- five times slower than
pitch on this airframe.

**From orbit, both craft** (rotation `14dc04c2`, LOG4010-4033, 6 per arm):
old craft on the three flare fixes **5/6 intact on the runway** (along
+14..+620, across <= 22 m), contact sink 1.0-4.1 m/s at 50-60 m/s; on the
defaults 1/6 (contact 15-21 at 37-41). The one loss (LOG4031) contacted at
4.1 m/s and pitch +8, then the nose fell to -1 in one tick and the nose gear
broke -- a slam at main-gear contact. The shuttle 0/6 on both arms, every
failure upstream of the flare: dives (door at 90-140 m/s, 40-100 m/s of
sink), doors 14-39 deg banked, two departures on the defaults (LOG4018,
4032). **Made default** (`2deb4b71`): `FLARE_TAIL_BY_ATTITUDE`,
`FLARE_EXP_TAU_S=4`, `FLARE_DOOR_FROM_SCHEDULE`. Failure 101.

**Built, off:** `ALPHA_TRIM_LOOP` (an outer integral on commanded minus
signed alpha in APPROACH and FLARE, gated to settled roll and small slip);
`HAC_FLAP_BRAKE_ON_SURPLUS` (the cone's brake on surplus alone -- it waited
for the weave to pin at 50 and the weave sat at 44, so it never deployed);
`APPROACH_BANK_BY_ROLL`; `FLARE_LOAD_LOOP` (refuted on the bench);
the `contact:` log line (state at the last airborne tick and the first
grounded one).

**Last rotation** (`d012a11f`, LOG4051-4071; 4072-4074 void): shuttle
defaults 0/4 intact; + `HAC_FLAP_BRAKE_ON_SURPLUS` + `HAC_FLAP_BRAKE_IGNORES_ROLL`
0/4 (brake really spends -- LOG4052 exit surplus +594 -- but each deployment
builds 30-40 m/s of sink in 3 s, and the approach still dives); no S-turn
(`APPROACH_SCURVE_MAX_DEG=0`) 0/4, mushes to 33-53 m/s at 35 m/s sink
(LOG4053: the speed law's 0.82 g load floor needs 13-16 deg at 57 m/s --
built `APPROACH_MUSH_RECOVERY`, unflown). Old craft defaults 2/3 intact (one
nose-gear loss after a 3.9 m/s contact: the mains are behind the CoM and the
nose pitches +8.5 -> 0 in one tick); with the user's rollout rules
(`ROLLOUT_BRAKE_FULL_ON_CONTACT` + `ROLLOUT_STEER_PID`) 1/3 intact, two lost
an elevon -- but both contacted at 72-74 m/s and 8-9 m/s sink, a different
arrival, so not yet a verdict on the rollout rules.

## Session, 2026-09-30 afternoon (the cone's budget: a phantom path, a constant glide ratio, and a table that changes)

**`APPROACH_MUSH_RECOVERY` is disconnected on the defaults.** Flown 3 rounds
of a 4-arm rotation (`ebd6dbb2`, LOG4075-4083, `logs/rot-mush-0930.txt`,
stopped at the round boundary): on no approach tick of either arm was the
vehicle below 0.8 x target above 800 m. With the S-turn on, the shuttle's
approach *dives* (108 -> 132 m/s at 60-70 m/s of sink on each +-40 bank
reversal, flown bank overshooting to 55 and signed alpha to ~0, LOG4051); it
only mushes with `APPROACH_SCURVE_MAX_DEG=0` (LOG4053). Re-fly it only on
that arm.

**The cone always exits high, on both craft, and it is not the cone's
authority -- it is the cone's arithmetic.** `conesum` over LOG4051-4071: the
first plan is ~16.2 km on every flight, flown ~20 km, exit +1.1..+2.8 km over
what the approach needs; `qs_plane` +1.1..+1.4 km on every flight. The
vehicle arrives *lined up* 16 km before the low gate (`turn=0`; the high gate
is on the extended centreline), so the "cone" is a straight-in and its only
spending authority is the weave and the brake. Two defects in the plan hid
the surplus from both:

1. **Phantom path** (failure 102, `HAC_PATH_WRAP_TO_GATE`, built, off).
   Lined up and a little outside a wide circle, the tangent point sits just
   past the rollout; `hac_turn` calls that arrived but `lead` still runs to
   it, `sqrt(x^2 + 2 R dy)`. Reproduced exactly offline: 190 m outside a
   16 km circle 955 m before the gate costs 2651 m (LOG4056: `gate=955
   path=2650`). The radius scan takes the longest fitting path, so it chose
   those circles: 2.2-3.3 km of phantom through most of the shuttle's cone
   (LOG4051 `gate=6207 path=9484`), weave at 0. Fixed, `path` tracks
   `gate` (LOG4086).
2. **`HAC_LD` 1.86 is a whole-cone average** of a Mach 0.7 entry at 22 deg
   (flown 1.3-1.5, decelerating) and a subsonic straight-in at 6 deg (flown
   2.5-3.0; the table agrees at the flown alpha, `ld=` 2.97/2.98). With the
   phantom gone the cone read itself *short* at 6 km and still rolled out
   1.1 km high (LOG4086).

**The glide-ratio fix, twice, and why it is off.** `HAC_LD_AT_TARGET`
first took the table's ratio at the cone's target speed *where the vehicle
was*: 1.16 at 13 km (thin air), short until 7 km, then weave pinned at 50
with 5 km left, exit +2.0 km (LOG4091). Rebuilt as a ladder
(`guidance.hac_ladder`: path = sum over 500 m slices from the gate of the
ratio at that slice's height). **From `qs_shuttle_low` it reads what it
should** (4.53 at 2 km -> 1.62 at 12 km, LOG4099) and weaves from entry.
**From orbit it reads 0.74-1.68** (LOG4100-4102, the `hac ladder` event):
the in-flight table at M=0.3 needs 5.8-9 deg for the lift STANDBY's table
makes at 1.3, at 54-69 m^2 of CdA against 11.5 -- and the same cell moves
between samples 60 s apart (LOG4102 CdA 41.8 -> 58.9 -> 35.7). STANDBY's
dump is clean and identical on every flight (M=0.3 5 deg 94.4/24.0). So the
rows `sweep` re-probes in flight are wrong for conditions the vehicle has not
reached yet; the `mdl=` field agrees with `act=` only once it is there. Not
explained (candidates: the rows re-aimed by `set_profile` to altitudes where
KSP's pseudo-Reynolds drag multiplier bites; the probe rotation). **Anything
that plans on the table ahead of the vehicle inherits this** -- the
propagator included.

**The cone's other spender is broken by roll.** Flown bank off the command
by >60 deg on 14-35 of 40-90 subsonic ticks on 7 of 17 recent shuttle
flights -- continuous rolls at Mach 0.5-0.9 (LOG4075: -64, -76, +109, -114,
-171, +131 ... at 2 s ticks, commanded 2-22; the damper answers by slowing
roll to 13 s). Four of five `HAC_FLAP_BRAKE_ON_SURPLUS +
HAC_FLAP_BRAKE_IGNORES_ROLL` flights are among them: the brake holds the
elevons the roll needs. `HAC_FLAP_ARREST_EXCESS` (built, off) prices only
the sink over the cone's own glide (whole, 117-179 m/s read as 1.4-3.2 km
of arrest and stowed every deployment in 3-7 s) -- but the roll conflict,
not the arrest, is what has to be solved first. The approach's S-turn has
the same disease: every reversal is where alpha is lost.

`HAC_SPEND_AS_SPEED` (flown n=2 on wrap+ld, LOG4092 4094): exits +1.3/+1.4
km; not a verdict.

**`HAC_PATH_WRAP_TO_GATE` vs defaults** (`264d651f`, `logs/rot-wrap-0930.txt`,
LOG4103-4120; 4121-4123 void, killed when the user stopped the batch):
`qs_plane` exits defaults +1136..+1504 (6, 5 intact 23/23), wrap +704..+1035
(5, 3 intact; LOG4112, the lowest exit, contacted at 71 m/s and broke up;
LOG4110 lost a gear). Shuttle 0 intact on either arm; exits defaults
+1046..+2759, wrap -777..+2317 (LOG4104: at the gate 750 m above need but
13-17 deg past the centreline, `turn` 343-347, not released, out of height
3.2 km beyond -- built `HAC_EXIT_PAST_DEG`, off). The shuttle's result is
set by lateral control, not the plan. **No shuttle flight from orbit has
landed intact on the runway** (28 today; closest LOG4083 +90 m along, 100 m
across, 9 parts). The user notes the shuttle's CG is too far aft to move the
vertical tail further back.

## Session, 2026-09-30 evening: the twin-fin shuttle

The user rebuilt the shuttle (`saves/craft/SPH/shuttle.craft`): the single
centreline Big-S tail fin replaced by **two on the wingtips** (outer
elevons, x = +-5.63 m, z = -15.53 against the old fin's -16.91, canted 15
deg out, mirror-deployed). Everything else identical. Flying it by hand they
put it on the runway at 50-60 m/s, tanks empty. `actuators.py`: every axis
live on both fins, limiter 37.5 as before; MoI roll 94k -> 124k (+32%),
mass 37.50 -> 37.95 t.

**Saves.** `qs_shuttle2` is a splice, not a flight: the craft launched on
ksp0 (`launch_vessel` hangs on a pre-flight dialog if the named crew is
already aboard another vessel -- name an unassigned kerbal), saved, and its
PART blocks put into `qs_shuttle`'s vessel taking structure (parent, attN,
srfN, sym, positions) from the new save and flight state (modules -- gear
retracted --, resources, temperatures, crew) from the old; both rudders
whole from the new. So UT, orbit and fuel equal `qs_shuttle`'s and a pair
measures the craft alone. `_inc`/`_high` by the old recipe; orbits
byte-identical to `qs_shuttle_inc`/`_high`.

**Paired batch, defaults `89b7daaf`** (`logs/rot-shuttle2-0930.txt`,
LOG4124-4135, 6 per arm, ksp0-2, fresh farm). New reader
`spaceplane/tools/lateralsum.py`: per flight, subsonic ticks with flown bank
>60 off the command (`bad60` -- *includes reversal lag*, so it does not
separate the craft), ticks with |sideslip| >10 (`slip10`), peak sideslip,
the cone's exit surplus, touchdown.

| | old `qs_shuttle` | new `qs_shuttle2` |
|---|---|---|
| subsonic `slip10` ticks | 13, 2, 6, 8, 4, 9 | 0, 0, 0\*, 1, 0, 0 |
| peak subsonic slip, deg | 12-30 | 8.8-11.0 (\*47.8 is on the ground, LOG4129 rollout) |
| intact | 1 (LOG4128, splashed 2.9 km long) | **1: LOG4135, 31/31, 6.1 m/s sink at 67 m/s**, +1569 along (past the end) |
| cone exit surplus, m | -6499..+2378 | +959..+5022 |

**The lateral departure is gone on the new craft** -- no continuous rolls,
sideslip inside +-11 deg on every airborne tick of six flights. LOG4135 is
the first intact shuttle landing from orbit.

**What now loses it: the flare inherits bank and cannot remove it.** Every
new-craft loss reaches the flare banked and touches down banked: LOG4129
handed over at -28 (commanded 0 for 5 s, flown -25..-45, contact at -45, a
wingtip -- now a fin -- on the runway, then a 30-48 deg slide); LOG4131 -13
-> -39 -> +87 at contact; LOG4133 +41 at the door. Two sources: the
centreline capture after the S-turn stop oscillates with a ~25 s period
against a vehicle whose roll lags 4-6 s (LOG4129: -17, -30, -4, +20, +18,
-14, -38 commanded down to 130 m) -- measured roll rate 8.5-12.8 deg/s,
roll `time_to_peak` 5.5 s; and the approach dives (LOG4127: +-40 S-turns
at alpha 2-6, 128 m/s and 77 m/s of sink at the flare door, struck at 133).
And the cone exits +1.0 to +5.0 km high (LOG4131 +5022 with laps=0).

## Session, 2026-09-30 night: fuel, and why the shuttle arrives high

**Gear.** `NOSE_WHEEL_FRICTION` (default 1.0): the nose wheel's friction
control switched to manual, the user's rule. Verified in game (LY-35 at 1,
mains 200%/10).

**Fuel.** The pre-burn drain keeps a 200 m/s x 1.25 reserve for the worst
orbit; the shuttle's burns are ~26 m/s, so 1.9 t rode to the wheels (LOG4135:
30.1 t, dry 27.5). In `qs_shuttle2` all of it is in the nose adapter
(+10.8 m) -- the other LF/Ox tanks are empty; 399 mono sit 0.6 m behind the
CoM. Built: `DRAIN_TO_BURN` (drain to the solved burn x1.25 + 10 m/s,
re-solve, then commit; LOG4140: 430 -> 69 units, burn delivered 26.2 of
26.2) and `FUEL_TO_NOSE` (first COAST tick: pump every tank front-to-back;
in-game test moved 700 LF/800 Ox from aft tanks to the nose in 1.4 s).
**`DRAIN_TO_BURN` is refuted as a default**: 3/3 drained flights crashed
27-31 km short with hypersonic departures (LOG4140, 4142, 4144; `rot-drain-0930`)
against 3/3 defaults reaching the field -- the nose fuel is the hypersonic
ballast (as `DRAIN_RESIDUAL_MACH_MAX`'s note recorded). The combination that
works: keep the reserve, `FUEL_TO_NOSE`, dump at Mach 0.8 (`DRAIN_RESIDUAL`):
valves open at 237-258 m/s, empty in 0.4-1.0 s, 30.6 -> 28.7 t
(`rot-ballast-0930`, LOG4149-4156).

**Arriving high: two mechanisms** (all `qs_shuttle2` exits +1.0..+5.4 km,
`laps=0`, planned path ~16 km on every flight whatever the entry height).
1. *The phantom lap* (LOG4152): 16 km before the gate, lined up, the weave's
   first swing put the tangent point 13 deg past the rollout -- one degree
   outside `hac_turn`'s 12 -- and the plan read a 347 deg / 106 km turn
   against 59 km affordable: *short*. Weave off, wings level, 13 km straight
   to the gate, +5.4 km. `HAC_PAST_BEFORE_GATE_DEG` (new): before the gate,
   a tangent point up to N deg past the rollout costs the run to the gate.
2. *Speed the plan cannot see* (bench): the cone dives 13.5 -> 9.7 km
   gaining 258 -> 314 m/s, then bleeds to 77 m/s by *climbing* (5790 ->
   6100 m) -- 4.3 km of height in kinetic energy that `HAC_ENERGY_BUDGET`
   (off since LOG1941-1952, old craft) would count. Bench `qs_shuttle2_low`
   (new: the twin-fin craft spliced into `qs_shuttle_low`, cone in 2.5 min):
   defaults exit +1591 (LOG4157-4159); energy budget +649..+722 (LOG4160-4162).

**Bench stack** (energy budget + wrap + `DRAIN_RESIDUAL` +
`APPROACH_BANK_BY_ROLL` + `APPROACH_FLARE_FACTOR=1.45`): **3 of 5 intact**
(LOG4166: 31/31, 4.8 m/s sink at 51.7 m/s, wings level; 4167, 4169) against
0 of 7 bench flights without it. The losses (LOG4168, 4170) reached the door
at 80-82 m/s with ~30 m/s of sink and contacted nose-down (pitch -13/-16) at
23-24 m/s -- the cone still exits ~+1 km dry, and the approach spends it as
speed. The slowest door (66.7 m/s, LOG4166) landed best.

**From orbit on the farm the stack did not hold** (`rot-stack-0930`,
LOG4171-4174; `rot-gap-0930`, LOG4177-4179): exits +2.0, +5.3, -1.8, -1.2,
-6.4, +4.3, -1.6 km; cone entries 13.4-19.5 km, all on the 3 km *distance
backstop*, never the glide's own `HAC_ALT_M` 12 km target. 0 intact.

**kspSim for the twin-fin craft.** `makecraft.sh 0 qs_shuttle2
qs_shuttle2_low qs_shuttle2` -> `models/qs_shuttle2.json`,
`qs_shuttle2_low.json` (augment needed an instance restart: stale thruster
transforms). Bench in the sim: exits ~-0.4 km where the game's are +1.0 --
biased, relative use only. From orbit it reproduces the scatter (exits
-3.5..+2.3, 1/6 intact) in **2.4 min for 6 flights**. New:
`kspSim/tools/simarms.sh SAVE K "arm sets" ...` (K flights per arm, all at
once) and `spaceplane/tools/armgroup.py` (groups logs by their own config
line -- concurrent flights confuse the harness's LOG attribution).

Sim screens from orbit (stack = energy budget + wrap + past-gate 60 + drain
residual + fuel to nose + bank by roll + flare factor 1.45):
- stack 2/9 intact; + `HAC_SPEED_PATH` 5/9, then 1/6 on a repeat (n=6 is
  noise here). Weave 75 + Rmin 1000: 1/3. `HAC_LAP_AT_TARGET_SPEED` (new,
  laps priced at the cone's speed): 2/6, three out of height -- overspends.
- **Why the entries scatter: the late glide cannot hold alpha.** LOG4171
  (game): commanded 31-34 deg at Mach 1.6-2.6, flown 13-15 -- less drag,
  arrives 18 km up over the field. The 1.9 t nose ballast costs the elevons
  their alpha authority below Mach 3 (cgProbe's 0.33 of pitch authority).
  **`DRAIN_RESIDUAL_MACH_MAX=2.5`** (dump after the hypersonic regime where
  the ballast is needed): sim 3/6 intact vs 0/6, cone entries **11.9-12.5
  km** (the 12 km target) vs 13.2-14.7, exits **+1.0..+1.5** vs
  -3.1..+3.0 (LOG4228-4239). The cone then tracks `need` within 300-500 m.
  Drained, alpha *overshoots* the command by 10-13 deg at Mach 1-3 (aft CG)
  -- watch it in game. The residual +1.2 km: flown cone L/D 2.3-2.6 in the
  sim against `HAC_LD` 1.86 (game measured 1.55-1.65 wet) -- not refitted
  from sim numbers.

**Game check of the Mach 2.5 dump** (`rot-m25-0930`, LOG4240-4243, round 0
only; 4244-4245 void, killed for swap): both 2.5 flights handed over at
h=12.0 km by the *altitude* trigger, 10.8-12.5 km before the field -- but
then ran out of height (-6.0, -2.2), as did the 0.8 arm (-2.7, -2.0) with
`HAC_SPEED_PATH` on (in the game the speed path runs the cone low; the sim
said the opposite). LOG4241 landed 31/31 at 52 m/s / 4.8 sink, 12 km short.
Drained, the game overshoots alpha by **19 deg** at Mach 1-3 (sim 10-13).

**The CG is wrong both ways**: wet flies under its alpha (long, high),
drained flies over (short, low). Built `DRAIN_RESIDUAL_KEEP_UNITS` /
`DRAIN_RESIDUAL_FINAL_MACH` (two-stage: keep N units as trim at the first
opening) and `DRAIN_TRIM_LOOP` (dump 25 units every 6 s while smoothed
alpha is >2 deg short of command, Mach 3.5..`DRAIN_RESIDUAL_MACH_MAX`; the
vehicle picks the amount). Sim (LOG4245-4269): keep 330 tracks alpha within
1-2 deg; the loop leaves 150-250 units with tracking -1..-6. Outcomes
bimodal **by how the glide hands over**: `HAC_ALT_M` trigger (12 km, 4-6 km
before the field) -> exits -3.4..-4.7, out of height; 3 km distance
backstop (~14 km) -> +1.8..+2.5. The same split `cone_affordable`'s
docstring records from 57 old flights (24/33 out of height on the altitude
trigger).

**`HAC_ENTRY_AFFORDABLE` (sim, LOG4270-4281): stack + affordable 4/4
intact, 2 on the runway, exits -191..+990**; + trim loop 0/4 (arrives
low); trim loop alone 2/4. Game confirmation: `rot-afford-1001`.

**Game, the rest of the night** (all n=2-4 per arm; exits in km):
`rot-afford-1001` wet+affordable +1.9 +3.9 +4.0 -2.7 (0/4), +trim loop -2.1
+1.1 -1.0 -0.8 (1/4); `rot-spend-1001` trim+affordable +3.2 +4.1 -1.3 +6.6
(0/4), + spend authority -1.1 +1.3 -1.8 +1.9 (1/4). `conesum` on these:
HAC_LD median 1.89 (configured 1.86) -- not a calibration error. Sim:
`PREDICT_RESIDUAL_DUMP` null (drained arrivals are short from over-rotation
drag, not mass). Across 46 game flights the entry height follows the glide's
`long=` at handover (on target -> 13.5-16.7 km; long -> 18-21; short ->
12 km early, out of height); corr(h, M1-3 alpha deficit) 0.45, (h, hypersonic
slip ticks) 0.15.

**`rot-ballast2-1001`** (LOG4311-4319): keep 200 units below Mach 3.5 ->
exits +2.08 +2.15 +2.22 +2.35 (one -8.0, entered 22 km short); keep 300 ->
+1.8..+4.4. The first consistent arrival of the session; lost in the
approach at 82-96 m/s. Next session starts there (HANDOFF).

## Session, 2026-10-01 morning: old-craft constants in the shuttle's chain

The user's direction this session: "what if there's a fixed constant in the
code you missed", then "replace constants with curves or some general
solution" -- never re-fit a constant for the shuttle.

**Baseline on a clean box** (`rot-base-1001`, LOG4320-4323, keep-200 stack,
no sims beside it): exits -1.8 / +4.2 / +1.8 / +1.5 km, 0/4 on the runway.
Last night's numbers were not only the sim load.

**Over ~150 twin-fin landings** (game and sim): every exit above +1.3 km
lands long, mostly past the far end; every exit below ~-1 km crashes short;
inside +-0.8 km the flare still starts at 75-95 m/s and the craft breaks
above ~70 m/s at touchdown.

Found, each an old-craft constant or a law that never did what it said:

1. **The cone weave did not fly its angle.** Progress per metre flown was
   0.76-0.80 whatever was commanded (cos 0.55-0.81): reversing +-50 deg at
   45 deg of bank takes ~33 s and the clock reversed every 24.
   `HAC_WEAVE_HELD` (swing = reversal + hold, angle solved for the swing's
   effective ratio). Sim (LOG4327): achieved 0.73 vs commanded 0.73, surplus
   closed +1.4 -> +0.3 km, landed 31/31.
2. **`HAC_LD` 1.86 and `HAC_GATE_LD` 1.35 are the old craft's.** The
   shuttle's cones fly 2.1-2.6 per planned metre; at 1.35 it enters the cone
   high by construction (LOG4321: 14.6 km at 20 km, weave pinned, +4.2 km).
   `HAC_LD_MEASURED` (the table ladder scaled by measured/table L/D at the
   flown alpha), `HAC_AIM_DERIVED` (entry aim from the same wings-level
   ladder). A re-fit arm (`HAC_GATE_LD=2.4;HAC_LD=2.4`, LOG4332-4335 round
   0 of `rot-trim-1001`) is a gain check only, not a candidate value.
3. **The approach speed (`APPROACH_FACTOR` 2.25 x stall = 108 m/s) sits
   where the shuttle's L/D is 3.0, against a final sized at 4.2.** LOG4281
   (landed on the runway) read itself 360-620 m low all the way down yet
   flew alpha 1.1-1.6 at 28-35 m/s of sink to hold 105 m/s, and flared at
   91. `APPROACH_POLAR_SPEED`: the speed whose 1 g glide ratio is the ratio
   still needed to the aim, off the table, fast side of best glide.
4. **The glide's alpha ceiling is 40 deg at every Mach** (`GLIDE_ALPHA_MAX_DEG`,
   raised for hypersonic sink). At Mach 0.6-1.3 the glide commands 34-40
   on a wing whose lift peaks at 32; LOG4334 departed: alpha 86-105, slip
   20-28, bank -153. Alpha tracking below Mach 4 is sd 5-24 deg on every
   flight. The curve already existed, sim-only: `GLIDE_ALPHA_PLATEAU=0.05`
   (~41 hypersonic, ~31 subsonic).
5. **The CG is right at one Mach only.** Keep 200 flew 10-20 deg over the
   command below Mach 2.9 (LOG4317: 20 commanded, 33-41 flown), keep 300
   under. `FUEL_TRIM_TRANSFER` pumps LF/Ox nose<->aft (13 m arm) against the
   interval-mean alpha error, Mach 4 to the residual drain. Verified moving
   fuel in game (LOG4332-4335).

Tools: `spaceplane/tools/gatesave.py` (quicksave a farm flight at an
on-profile cone handover); bench saves `qs_shuttle2_low_{m30,p30,p60}`.
Trap repeated and stopped: sims beside the farm (load 23 on 16 cores) --
four sim flights departed in the late glide; don't.

**The chain batches** (all `qs_shuttle2` from orbit; chain = stack + held
weave + fuel trim + plateau ceiling + measured cone ratio + derived aim +
polar approach speed):
- `rot-chain-1001` round 0 (LOG4336-4339): **3/4 intact** (31/31) at 55-66
  m/s, flare 76-85 m/s, cross at flare <100 m; all short or long on energy
  (-4.9, -1.3, +1.1 km): the cone held 108 m/s *true* at 6-10 km (alpha
  17-22, L/D 1.0-1.5) -- short all the way. `ldk` 0.91-1.15: the table is
  right; the speed law was the problem.
- `rot-chain2-1001` (+`HAC_POLAR_SPEED`, LOG4344-4347): **refuted** -- the
  polar is flat near its top; the target stepped tens of m/s, 75-250 m/s
  phugoid in the cone, flares 87-112.
- `rot-chain3-1001` (+`HAC_SPEED_EAS`, `ALPHA_RATCHET_ON_SWING`,
  LOG4351-4358): **4/8 cone exits within +67..+425 m** of what the approach
  needs (baseline: +-1.5-4 km). The other 4 arrived 10-46 km long: LOG4358
  departed hypersonically at a Mach 5.1 bank reversal (alpha 38.7 -> 17,
  long +0.5 -> +11.7 km) and the swing ratchet, at a 6 deg threshold, then
  lowered the ceiling to 28.7 and kept it long. Threshold now 12 deg
  (`ALPHA_SWING_TOL_DEG`). Of the good exits: LOG4354 (+202) intact but
  floated 3.6 km (the polar approach inverted a 4.2 final to ~140 m/s --
  now one-sided), LOG4351 (+103) flared at 87, broke at 84.
- **Hypersonic alpha excursions >15 deg at Mach 3-5 on most shuttle flights**
  (overnight and today): overshoots to 50-58 without reversal, collapses to
  17-24 in reversals. The airframe's pitch margin at high alpha; the craft
  question stands.
- **`qs_shuttle2_gate` is not a valid bench**: loaded in the air the vehicle
  makes half its table lift from the first tick (elevons saturated
  nose-up, flown alpha below command) and dives; 12/12 identical crashes at
  118-175 m/s (`rot-gate-1001`), LOG4371 the same at 1x. The late-glide
  bench shows the same transient right after loading and recovers.

**Afternoon batches** (each 8 from orbit unless noted):
- `rot-chain4-1001` round 0 (swing threshold 12): LOG4375 exited -55 m, on
  the centreline, crossed the midpoint 840 m up and dove into the runway at
  90 m/s -- the final was aimed at `TOUCHDOWN_AIM_M`=2400, the far
  threshold. -> `TOUCHDOWN_AIM_DERIVED` (zone less flare float; 42 m).
- `rot-chain5-1001` round 0: 3/4 glides 10-36 km off; LOG4379 arrived on
  target and ran out of height -- `HAC_AIM_DERIVED` was aiming at best
  glide (3.57), the edge of the cone's authority. -> aim at the harmonic
  mean of steepest (alpha-limit drag x least efficient weave) and flattest
  (best glide): 1.79 for the shuttle.
- `rot-chain6-1001` (midpoint aim): **3/8 intact** (LOG4385 28/31 at 38 m/s,
  199 m from the midpoint; LOG4387 31/31, +78 m along; LOG4388 31/31) but
  416-2503 m off the centreline. LOG4385: the capture commanded level with
  27 deg of bank still on and the track turned on to +21 deg before the
  wings came level.
- `rot-chain7-1001` (+`APPROACH_ENERGY_EXCESS`, `APPROACH_HEADING_LEAD`):
  **0/8 intact**, touchdowns 1.7-2.6 km short of the midpoint, flares 83-95.
  Energy excess refuted; heading lead unproven (confounded).
- The user pointed out the shuttle already has canards: two Big-S Elevon 1
  at z=0 on the nose adapter, ~12 m ahead of the CoM (I had read the part
  list by name, not position). The Mach 3-5 pitch-ups do not follow
  flap-brake deployments (1/28). User approved a forward-ballast test;
  first form needs no craft edit: `DRAIN_RESERVE_DV_MS=1000` keeps all the
  propellant, `FUEL_TO_NOSE` puts ~8.7 t in the nose adapter
  (`rot-ballast3-1001`, chain6 vs chain6 + ballast, 4 v 4).
- Trap repeated: idle kspSim servers from the morning's sim screens sat
  beside every game batch until ~11:00 (1.6 GB swap).

**Evening: the "pitch-up".** The user asked why the twin-fin craft would
pitch up and not the single-fin one: it doesn't -- paired `rot-shuttle2-0930`
has single-fin 4/6, twin-fin 2/6; all game flights 54% vs 28%. Precursors
over 71 overshoots: RCS valve open (most), residual drain dumping the nose
fuel at Mach 3.5 (16), fuel trim aft, reversals. `RCS_PITCH_BY_AUTHORITY`
(`rot-rcsgate-1001`, 4 v 4): 2/4 still departed vs 1/4 -- null. Read on
LOG4409: sideslip 21-27 deg for 20 s with the flown bank going the wrong way
(+7 -> +107 -> -95 vs -31 commanded), roll damper swings to 178, *then*
alpha 66-88. The pitch-up is the end of a high-alpha lateral departure
(failure 99), common to both Mk3 shuttles. Forward ballast
(`rot-ballast3-1001`, round 0 only): 0/2 departures but commanded alpha
only 18-25 -- a different regime. Not fixed; HANDOFF item 1.

## Session, 2026-10-01 evening: the pitch-up is the pitch thrusters

Mined ~320 shuttle logs (scratch scripts). Bank-reversal roll rate, alpha,
Mach and q did **not** separate slipping reversals from clean ones (168
reversals, ~50% "bad" in every bin). The valve's lag did: reversals where
the RCS opened 6-12 s after the command started moving had median peak slip
15.8 deg vs ~9 (358 reversals) -- `reversal_under_way` needs a 5 deg lead,
which a slow slew never builds (LOG4409).

First departure tick per flight (flown alpha > commanded + 15, M>1.2) splits
into three classes: **A** Mach 4.7-7, slip <5, `rcs on` 0-5 s before, alpha
40 -> 56 in 4 s (LOG4352: commanded 40.3, trimmed 36, `err 5.0` from the
shortfall alone opened the valve, pitch thrusters drove alpha through the
trim limit); **B** residual drain at Mach 2-3.3 (LOG4298-4307 are sim
flights -- thinner game evidence than it looked); **C** lateral, roll damper
swings 130-178, slip 20-70 (most chain6 departures).

- `rot-shortfall-1001` (chain6 vs + `RCS_IGNORE_ALPHA_SHORTFALL`, 8 v 8,
  `23a608af`): class A 3/8 vs 0/8, but the control's three opened on
  reversals (err 7.9-8.9) and LOG4415 (flag) opened on lateral error at
  Mach 7.1 and still pitched 26 -> 53. Intact 3/8 vs 0/8 (LOG4426 +742 m).
  Swap 5.3 GB at the end.
- `rot-pitchoff-1001` (chain6 vs + `RCS_PITCH_OFF_IN_GLIDE`, 8 v 8,
  `b87b111e`): glide departures M>2 **6/8 vs 1/8**. Flag arm flew 2-5 deg
  under command at Mach 2-5 (control within ~1) and arrived at the cone
  +2.4..+34 km long, 19-22 km high; 0 intact. The thrusters were trimming
  beyond the surfaces' limit -- which is where it departs. Swap 10.4 GB.
- `Holdable` learned the command ("learned 36.0" flying 31-34): max while
  saturated, and the *command* on any tick within 2.5 deg. ->
  `HOLDABLE_MEAN`.
- `rot-holdmean-1001` (both pitch-off; vs + `HOLDABLE_MEAN`, `5ea40f73`):
  killed after round 1 for the user's CPU. 4 v 4: handover h 13.5-14.5 km
  vs 16-17; LOG4444 long +125, LOG4449 -1872; LOG4446 +9.8 km; LOG4447
  departed laterally at M4.1. Not a result. LOG4451-4454 void.

## Session, 2026-10-01 night: the roll-rate estimator's slip pulldown

- `rot-holdmean-1002` (chain6 + `RCS_PITCH_OFF_IN_GLIDE` vs the same +
  `HOLDABLE_MEAN`, 8 v 8, `5ea40f73`, LOG4455-4470): **null**. Cone
  handover long ctl -1.5..+17.2 km, HM -15.9..+32.3; 4/8 within 2.1 km in
  each. No intact landing (LOG4469 19/31 parts, +2.1 km). Swap 5 GB.
- Mined it instead (scratch `slip.py`, `rev.py`, `replay.py`). **Every
  flight that slipped past 12 deg did it in the same reversal**: Mach 4.5-4.7,
  q 2800-3800, + -> - bank, flown bank lagging the command 15-30 deg, alpha
  collapsing 35 -> 13-23, then the ceiling ratchets down to ~22 and the glide
  runs low-drag 4-32 km long (LOG4459). The 6 that never slipped all reached
  the cone within +-2.1 km. Loop rate, drain residual, mass, nose fuel and
  instance do not separate them.
- The separator is the bank slew limit (`RollRate.limit()`). Replaying the
  estimator over the logged ticks: the slip tolerance (`BANK_RATE_SLIP_TOL_DEG`
  5) has already pulled the limit from 7-10 to 1.3-2.8 deg/s **in COAST and
  the first seconds of GLIDE, at q 0-150 Pa**, where "sideslip" is the nose
  wandering in vacuum. Bad flights then reach the second reversal at 3.2-4.3
  deg/s (clean 5.1-5.9), take 11-17 s over it instead of 7-9, and finish it
  at q ~3000+. The first reversal (M6.6, q 290) slips ~10 deg in half the
  flights and ~2 in most others regardless of rate -- cause not found.
- `rot-sliptol-1002` (chain6 + pitch-off vs + `BANK_RATE_SLIP_TOL_DEG=0`,
  8 v 8, LOG4471-4486): second reversal at q ~2000 vs ~2800, alpha collapse
  >=16 deg at M3.5-5.2 **4/8 vs 7/8**, cone within 4 km 5/8 vs 3/8 (but
  arriving 14.5-23 km high vs ~12), final |along| median ~4 vs ~9 km,
  >10 km misses 2/8 vs 4/8. No intact landing in either (best TOL0 LOG4485
  13/31 at -3.2 km). Direction consistent, p ~0.3; replicate
  `rot-sliptol2-1002` flown next.
- `rot-sliptol2-1002` (replicate, LOG4487-4502): TOL0 **LOG4498 intact
  31/31, -1036 m along** (first intact shuttle landing since chain6),
  LOG4493 30/31 at +2980, LOG4491 31/31 but splashed 4.4 km off the
  centreline; control none intact on land (two splashed 29/31). Pooled 16 v
  16: final |along| <= 5 km **11/16 vs 6/16**, median ~3 vs ~10 km, >=30
  parts 3 vs 0. Alpha collapse at M3.5-5.2 *not* cured (6/8 vs 7/8): the
  second reversal still slips 13-23 deg at q ~2100. TOL0 arrives at the
  cone high (15-22 km) and long, and lands better anyway. Next: the same
  pair on `qs_plane` (`rot-sliptol-plane-1002`) before any default.
- `rot-sliptol-plane-1002` (`qs_plane` defaults vs + TOL0, 8 v 8,
  LOG4503-4518): **null** -- the old craft's limit sits at the 30 deg/s
  ceiling in every phase either way; handover -0.5..+0.9 km in both; 4/8
  vs 3/8 destroyed. -> **`BANK_RATE_SLIP_TOL_DEG` 5 -> 0 is the default**
  (fingerprint `f1c153ba`). Suite 854 OK.
- `RCS_HOLD_MID_REVERSAL` (built, off; `reversal_under_way` also holds
  while the rate-limited command is > `BANK_RATE_SAT_DEG` from
  `bank_wanted`). `rot-revhold-1002` (chain6 + pitch-off on `925b4729` vs +
  flag, 8 v 8, LOG4519-4534): **null**. Connected (LOG4520 opened at glide
  start on err 1.3), but with the pulldown gone the M4.9 command slews
  12-18 deg/s and the valve already opens within a tick; second-reversal
  slip 7.6-20.8 vs 7.7-17.8, alpha collapse >=16 deg 4/8 vs 2/8. No intact
  landing either arm; both arrive at the cone mostly long and high (+4..+29
  km, h 15-23 km). LOG4521: after a slip-11 reversal at M4.8 the bank pins
  at the 70 cap at q ~5000 and flies -66..+117 against it, slip -38.
- Over the 32 pulldown-off flights, the slew limit going into the second
  reversal against its peak slip: r = -0.30 (limit < 10: 8/15 slip >= 15;
  >= 10: 5/17). Faster is weakly better; not a lever. A q-gated pulldown
  (replayed at q >= 500/1000) gives the same limit as off before that
  reversal -- nothing to gain there.
- Authority (logged): at q 2100 the surfaces give roll 617 / yaw 158 kN m,
  RCS yaw 293; yaw inertia is 22x roll's. The airframe out-rolls its own
  yaw at 35 deg alpha (failure 99). Three mechanisms on this reversal now
  (rate pulldown -- fixed; valve hold -- null; rate -- weak): change the
  method. Candidate: unload alpha through the reversal (body roll's slip
  scales with sin alpha), flown by the propagator too.

## Session, 2026-10-02 morning: the slow reversal (`GLIDE_BANK_SWEEP`)

The user's two ideas, in order. **"Fly lower alpha instead of banking"**:
offline from LOG4498's swept table the shuttle's range falls monotonically
with alpha at every Mach (Mach 7 wings level: 1166 km at 0 deg, 439 at 30,
337 at 45); lower alpha *stretches* the glide (L/D 2.4 at 0, 0.63 at 30), and
alpha already flies to the plateau edge (~38-41), so alpha cannot take the
bank's energy job. **"One huge reversal: bank really slowly the whole time,
steeper early"**: built as `GLIDE_BANK_SWEEP` (off), in three versions.

1. *Continuous sweep, solve on the mean model.* Steers beautifully (one sign
   change, 0.1 deg/s, cross at the gate within 200 m) but the solve sat at the
   70 cap while the vehicle flew -30..+20: sim LOG4543-4550 reached the cone
   +57..+64 km long. A sweep slow *throughout* cannot shed the energy:
   +-70 averages cos 0.77 against the ~0.5 the glide asks for.
2. *Planned slow reversal* (`trajectory.BankPlan`): hold a side, cross at
   ~1 deg/s, hold the other side; the crossing's start
   (`guidance.sweep_start`) and, under way, its rate (`guidance.sweep_rate`)
   are solved for the cross-track at the gate; the range solve flies the same
   plan. Root-finding is a warm-started local bracket + Illinois (a global
   regula falsi stalled 116 km off on the step-shaped curve). Held to the
   gate it spirals subsonically (bank 40-54 at 200-250 m/s, ~5 km turn
   radius): sim cone 1 km low, 50 m/s slow, out of height 7/8. Relay below
   Mach 1 (`GLIDE_BANK_SWEEP_UNTIL_MACH=1`, with the plan flying the relay's
   mean there, `BankPlan.until_mach`): sim 5/8 intact vs base 2/8. Below
   Mach 2 it was 11-16 km long at the cone in every flight -- retired.
3. Farm, interleaved vs the best stack (chain6 + `RCS_PITCH_OFF_IN_GLIDE`):
   `logs/rot-sweep-1002.txt` (fingerprint 9bd29b7a, rate cap 2 deg/s) and
   `logs/rot-sweep2-1002.txt` (8c5a05a9, cap 1 deg/s, fallback after 10
   lost ticks). **Base: sideslip 17-51 deg in all 16 flights.** Sweep: the
   crossing (always Mach ~5.8 -> 3.5-4.5, q 1000 -> 3000-5000) is
   **bimodal -- slip 2.8-3.3 in 4 of 13 (LOG4608, 4624, 4634, 4637), 15-28 in
   9**. In the bad ones slip is already -2..-5.5 at q ~1500, lean ~-20, and
   grows steadily even at 0.5 deg/s; the flown bank stalls, runs backwards,
   then overshoots to 77-92. Not the discriminator: loop rate (all 1.0 s),
   instance, RCS valve state, crossing rate, fuel trim. Weakly: the roll rate
   measured in COAST (clean 8.6-14.9, bad 6.1-10.8 deg/s). Three flights in
   batch 1 crossed at Mach 7 on a first-tick search that ran out of
   evaluations (fixed: `GLIDE_BANK_SWEEP_FALLBACK_TICKS`) and then crossed
   back at 0.3 deg/s from q ~700 -- all three clean (<=6 deg), 25 km long.
   Landing outcome: unresolved at n=8 (within 5 km 5/8 and 4/8 vs base 3/8
   and 2/8; intact 1/16 vs 0/16).

The slow crossing is the first thing that has taken the shuttle through Mach
5-4 with slip under 4 deg, and it does so a third of the time; the rest is a
lateral-directional divergence near wings level at q > 1500 that a slow roll
does not prevent.

**`GLIDE_BANK_SWEEP_CROSS_ALPHA_DEG=25`**, `logs/rot-unload-1002.txt`
(d695195b, sweep v sweep+unload, 8 v 8): **null, and costly.** Four of the
unload arm's five "clean" crossings reached the cone +182..+194 km long --
alpha 25 through the crossing cut the drag the plan was counting on (the
plan does not model the cap) -- so their slip is not comparable. The three
that flew a normal range (LOG4640, 4645, 4650) slipped 18-22 deg, earlier
(Mach ~5). Control arm: clean 2/8 (LOG4649, 4654). The sweep's clean
fraction across three batches is **6/21**; base **0/16**. Roll coupling
through sin(alpha) is not the discriminator at this size of unload.

## Session, 2026-10-02 afternoon: the flare does not flare; the farm gets PyPy

**The flare never reaches its alpha in the game.** Over ~80 game shuttle2
flights the flare commands 9-16 deg and flies 3-4 (LOG4610, 4639, 4647);
doors are entered at 20-40 m/s of sink, touchdowns at 65-107 m/s and 7-25 m/s
of sink. The sim tracks the same commands within ~1 deg, so only the farm
can see it. New `pin=total/assist/err` column (kRPC's `Control.pitch` reads
back the *sum* of game, manual and autopilot input -- decompiled
`PilotAddon.OnFlyByWire`): base flares sit at **+0.13..+0.31 total input**
with a 1-7 deg lag -- kRPC is not saturated, it simply under-commands
(rot-assist*, LOG4700-4720).

**`PITCH_ASSIST` (off): refuted as built.** kRPC *adds* client manual input
to its output, so a manual pitch integrated on the pitch pointing error
(commanded nose . roof) was meant to supply the missing integral. Sim: v1 on
(cmd - signed alpha) wound both integrators to +-1 (LOG4680); v2 on the
pointing error wound up on kRPC's ~1-2 deg attenuation band (LOG4695); v3
with a 2 deg deadband landed 2/2 intact on the low bench vs 0/2 (LOG4696-4699).
Game, 8 v 8 from orbit (`rot-assist-1002` CPython rounds 0-1,
`rot-assist2-1002` PyPy rounds 0-1; fingerprint 29664148/aed153ef, fields
added only): **the assist ran to -0.9..-1.0 (full nose-down) in 6/8**, dove
out of the cone 3.5-6.4 km short, doors at 55-91 m/s of sink. Base 8: cone
exits mostly "out of height" too, 3 landed on land with parts left (LOG4700
523 m from the midpoint, 8 parts; LOG4718 959 m along, 6 parts). The
runaway in the game but not the sim says the sign of manual pitch or of the
roof differs between them -- verify directly before any re-fly (HANDOFF).

**kspSim**: `world.py` now sums manual and autopilot inputs as kRPC does;
`Control.pitch/roll/yaw` getters return the total.

**Farm**: PyPy for the autopilot (`rotfly.sh` `PYPY=1`): propagator 20 s vs
120 s offline, cone ticks 21 -> 8 ms; a round from orbit 8 min vs 20.
Textures stripped from every clone (GameData 2.4 -> 1.1 GB; RSS unchanged
at ~3.6-4.0 GB, the 4 GB is anonymous heap). `-nographics` hangs. Six
instances ran together at 6 GB free, cores 23-49% user, swap 1.5 GB after.
Benches `qs_shuttle2_low` and `qs_shuttle2_gate` are invalid in the game
(the former crashes 6-8 km off the centreline into hills, the latter starts
643 m from the gate).

## Session, 2026-10-02 evening: farm speed, round 643 s -> ~450 s

Farm throughput only; no flight-law change. Six instances, PyPy, one arm
(the afternoon's base arm on `qs_shuttle2`), one flight per instance per
round. The practices are written up in `docs/loopCost.md`.

**Instruments added.** `common/rpccount.py` makes four lines at the end of every
log: kRPC calls and wall-ms per tick by phase, wall seconds per phase
(the real farm cost), and each phase's three slowest ticks with their calls.
The `loop rate` line now gives `pk=`, each phase's worst tick.

**Baseline** (`rot-rpc2`, LOG4733-4738, fresh farm): round 643 s, flights
475-627 s (mean 533). COAST ~150 s at 5x, GLIDE ~145 s at 3.5x, HAC 1.1-1.4x,
DEORBIT ~80 s, DRAIN 18 s.

**Changes, each measured on its own round:**

| commit | change | round |
|---|---|---|
| `RPC_BATCH` | aero-table row = one kRPC request (`common.krpcbatch`); 14 calls 6-7 ms vs 10-45 ms, bit-identical | 643 -> 544 |
| `settle_game` | surface probes wait 0.6 game-s at the governor ceiling, not 0.6 wall-s at 1x (DRAIN 18 -> 6 s) | (same round) |
| `GOVERN_PEAK_SKIP=1`, warp ticks ignored, cost reset per phase | one fuel-scan tick held COAST at 4.4x for 320 game-s; DRAIN's 5 s tick held DEORBIT near 1x for 50 game-s | 548 -> 449 |
| `GOVERN_PEAK_WINDOW_S=60` | peak over the last 60 game-s of the phase | 446, **null** (GLIDE unchanged) |
| `TIMESCALE_QUANT_FRACTION=0.2` | frame quantum 0.2 x the control interval | GLIDE unchanged, **null** for speed |
| surface gravity cached | 2-4 round trips a tick | small |

**Final** (`rot-final-1002`, LOG4781-4792, two rounds, fresh farm but 10 GB
zram at boot): rounds 472 and 435 s; flights 318-438 s (mean 381). That is
**1.42x the throughput** (~34 -> ~48 flights an hour). HAC, APPROACH and FLARE
all held 0.10-0.11 game-s per tick.

**What limits it now is the game, not the autopilot.** In GLIDE the governor asks for
9-19x and the instance delivers ~4.5x at ~23 fps on a 0.2-0.5 s quantum:
~235 physics steps a second, ~4 ms each. The KSP main thread is at 75-99% in
`top -H` whether the instance keeps up or not (Unity frames are uncapped),
so that percentage cannot show saturation; fps against the quantum can
(`logs/farmcpu-1002.txt`, 192 samples). HAC's slow ticks are game-side:
`AvailableControlSurfaceTorque`, `AvailableReactionWheelTorque` and
`MomentOfInertia` taking 40-60 ms on some ticks. DEORBIT's ~80 s includes about
23 game-s at 1x slewing to burn attitude, governed by the 0.8 and 0.5 s solve
ticks before ignition. That is failure 91's protection, left alone.

**Seven instances**: `ksp6` was cloned (matches `base/`; kRPC ports
50112/50113) and booted. Seven booting together saturated the CPU (load
31.5 on 16 threads, instances at 90-215% each). The user stopped it, so it
was never flown, and we are back to six. Six already put 8-13 GB in zram
(~4 GB per instance). That contradicts the afternoon's "6 GB free", which was
measured before the instances had grown.

## Session, 2026-10-02 night -> 10-03: landing from cone saves, working backwards

**Method change: landings measured from in-game cone saves, not from orbit.**
`qs_s2_hac0`-`5` (`saves/`, commit f31fb53): six default `qs_shuttle2`
flights (LOG4793-98) saved by `entrysave.py --alt 13500 --low 11500` just
inside the cone. A flight from one costs ~4-5 min wall; six saves are six
cone-entry states (arrivals -0.2..+5.0 km). Every batch below is
`rotfly.sh` over the six saves x 2 rounds = 12 flights, fresh farm each.
Note the orbital defaults arrived within 2 km on all six (LOG4793-98), where
last session's "best stack" from orbit read +17 km sd 10 (LOG4781-92).

| batch (`logs/rot-*.txt`) | LOGs | change on top of the previous arm | intact / 19-30 parts / lost |
|---|---|---|---|
| hacbase-1002 | 4799-4810 | defaults | 0 / 2 / 7 |
| atrim-1002 | 4811-4822 | `ALPHA_TRIM_LOOP` fixed + `ALPHA_TRIM_IN_HAC`, slip tol 15, max 12 | 1 / 0 / 4 |
| slow-1002 | 4823-4834 | + approach 1.8x, floor 1.6x, door 1.45x stall | 1 / 1 / 3 |
| mid-1002 | 4835-4846 | instead 2.0 / 1.8 / 1.8x | 0 / 0 / 9 |
| bankroll-1002 | 4847-4858 | trim + `APPROACH_BANK_BY_ROLL` (default speeds) | 1 / 2 / 4 |
| ground-1002 | 4859-4870 | + `ROLLOUT_ON_MAIN_CONTACT`, `ROLLOUT_HOLD_TAIL_FRACTION=0.4` | 1 / 0 / 0 |
| kd-1002 | 4871-4882 | + `APPROACH_SPEED_KD=1` | 0 / 3 / 0 |
| trim4-1002 | 4883-4894 | + trim bounded -2..+4 | 1 / 3 / 4 |
| attgt-1002 | 4895-4906 | + `APPROACH_ALPHA_AT_TARGET` | 0 / 2 / 2 |
| stop4k-1003 | 4907-4918 | + `APPROACH_SCURVE_STOP_M=4000` | 1 / 3 / 2 |
| **capture-1003** | 4919-4930 | + `APPROACH_CAPTURE_MARGIN=0.15` | **2 / 2 / 4** |
| decel-1003 | 4931-4942 | + `ATTITUDE_PITCH_DECEL_S` -- **disconnected** (no such kRPC attribute): a replicate | 1 / 2 / 2 |
| oscoff-1003 | 4943-4954 | capture + `ATTITUDE_OSC_MITIGATION_OFF` | 3 / 4 / 1 |
| pfloor-1003 | 4955-4966 | capture + `ATTITUDE_PITCH_AIR_FLOOR_S=1` (cone too) | 0 / 5 / 4 |
| pfloor2-1003 | 4967-4978 | same, approach and flare only | 2 / 0 / 8 |

Findings, in the order they were found:

1. **The cone hands over 1.5-4.4 km high and the approach dives it off**
   (baseline: doors at 650-870 m and 95-126 m/s of sink). kRPC flew -3 deg
   of signed alpha against 2-10 commanded at +0.07 input (LOG4803).
2. **`ALPHA_TRIM_LOOP` was built wrong and never flown**: it integrated
   `(alpha + delta) - achieved`, kRPC's error against the offset command,
   which can only wind up. Fixed to `alpha - achieved` (49f3858). With it in
   the cone and on final the handover surplus fell to +0.2..+0.7 km and
   contact sinks to 1.5-10 m/s in half the flights.
3. **Every intact landing entered the flare fast and high**: LOG4816
   110 m/s / 367 m, LOG4823 102/371, LOG4857 122/523. Doors under ~90 m/s
   could not arrest the sink. The stall-factor speeds were swept and are
   noise-dominated (slow, mid); retired as a method.
4. **The approach flies a ~50 s phugoid** (LOG4848: 81-102-80-121-53 m/s;
   pi sqrt(2) v/g = 45 s at 100 m/s), and the trim loop wound to its bounds
   in phase with it. `APPROACH_SPEED_KD` bunched door speeds (44-78) but
   did not stop the cycle; bounding the trim to -2..+4 and
   `APPROACH_ALPHA_AT_TARGET` (alpha at the target speed, not a speed law)
   smoothed it. Door sinks fell to 16-35 m/s.
5. **Touchdowns after a gentle contact broke on the ground**: the craft's
   tail-strike angle is **11.0 deg**, the rollout held 8, and FLARE flew
   1.4 s on the wheels before KSP said "landed" (LOG4836). Two flags.
6. **Doors were 100-600 m off the centreline**: the capture law limit-cycled
   +-500 m with the flown bank far behind the command (LOG4915). The
   S-turn stop did nothing; `APPROACH_CAPTURE_MARGIN` 0.35 -> 0.15 put 11/12
   doors within +-170 m.
7. **The flare still flies 2-3 deg against 7-12 commanded at a flat +0.24
   input** (LOG4927). This kRPC 0.6.0 build has no `deceleration_time` (the
   flag flew disconnected) but has an oscillation detector with automatic
   notch/bandwidth/feedforward mitigation, latched at level 0.93 on a
   landed vessel. Switched off from the cone on: **tracking unchanged**
   (flare error 6.7 deg vs 5.5), so the 3 intact are noise. A stiffer pitch
   (`ATTITUDE_PITCH_AIR_FLOOR_S=1`, applied 1.5 s): approach error halved
   (rms 6.1 -> 3.3) in the cone-too arm, but the cone's fitted constants
   moved three handovers short; restricted to final and flare it **departed
   4 flights in the flare** (pitch +90) -- refuted.

**Pooled, the capture stack lands 6 intact of 36** (capture, decel, oscoff),
against 0/12 on defaults. The rest are: flares that do not pull up (kRPC
tracking), doors low on energy (77-170 m, 15-35 m/s sink), and departures.

**`FLARE_PITCH_P` -- the first lever with a mechanism and an outcome.**
kRPC sums a client's manual pitch with its own output, so a manual input
proportional to the flare's pitch pointing error (no integrator, nothing to
wind) supplies the authority kRPC's tune leaves out. On the capture stack,
cone saves:

| batch | gain | intact (31/31) | flare alpha error mean / rms |
|---|---|---|---|
| capture, decel, oscoff | 0 | 6 / 36 | 5.5-6.7 / 8.3-9.7 |
| rot-flarep-1003 (LOG4979-90) | 0.04 | 5 / 12 | 4.0 / 6.4 |
| rot-flarep2-1003 (LOG4991-5014) | 0.04 | 5 / 12 | 4.2 / 5.8 |
| rot-flarep2-1003 | 0.08 | 6 / 12 | 2.6 / 4.4 |
| save-flarep3-1003 (LOG5015-38) | 0.08 | 7 / 12 | 4.0 / 11.9 |
| save-flarep3-1003 | 0.12 | 4 / 12 (1 lost) | 4.1 / 8.5 |

0.08 pooled: **13/24 intact**. (rot-flarep2 was unbalanced across saves --
`rotfly.sh` with 12 arms on 6 instances flies only arms 0-8; savefly in the
scratchpad pins save i to instance i and alternates the arm.)

**From orbit** (`rot-orbit-1003`, LOG5039-50, 4 each): `qs_plane` on the
defaults **lost 4/4** (touchdowns 61-65 m/s); `qs_plane` with the stack +
`FLARE_PITCH_P=0.08` put 4/4 down with 18-21/23 parts (doors +17..+30 m off
the centreline, contacts 8 m/s at 44 m/s every time, then ~100 m of drift
in the rollout); `qs_shuttle2` with the stack 0/4 intact -- two arrivals
+5.5 and +11 km (the entry, upstream of tonight), the two near ones 16 and
18/31.

**From orbit at n=8, then promoted** (`rot-orbit2-1003`, LOG5051-74):
`qs_shuttle2` on the old defaults 0 intact, **7/8 destroyed**, touchdowns
60-170 m/s; on the stack + `FLARE_PITCH_P=0.08` 1 intact, 3 with 19-23/31,
**none destroyed**, touchdowns 39-67; `qs_plane` on the stack 3/8 intact,
all eight 20+/23. Promoted all twelve settings (bc2c494, fingerprint
**6640ccdc**); three tests of the replaced laws now pin their configuration,
and `alpha_trim_loop` reads `state` with a getattr (a bare test Autopilot has
none). Verified on the committed defaults with no `--set`
(`rot-newdef-1003`, LOG5075-86, 6 each): shuttle 1 intact + 28, 17, 16/31,
two destroyed (one from a +9.5 km arrival); old craft 2 intact + 21, 20/23,
two broken up to 6-7 parts in the rollout. Old-craft rollouts end 100-200 m
off the centreline. Full suite 865 OK.

## Session, 2026-10-03 evening: the rollout steered the wrong way, and the approach relayed

The user, at the start of the session: the old craft is a wingless flying brick.
A bad result there is the design, not the autopilot. It is a regression check
only, and the shuttle is the target (`spaceplane/CLAUDE.md`, "Priority").

**Every "intact" shuttle landing of the last session was off the runway
sideways.** save-flarep3-1003's 31/31 flights stopped 200-600 m from the
centreline, on a 70 m strip. There were two causes, one after the other.

1. **The nosewheel steered away from the centreline.** `across =
   cross(up, along)` points to the vehicle's *right* in kRPC's left-handed
   body frame. At KSC, facing east, it comes out south. `wheel_steering`
   is +1 to the left, so `-gain * cross` steered further out. It showed as
   sideways speed *growing* while the vehicle slowed (LOG5021: 9 -> 24 m/s).
   **`ROLLOUT_STEER_ACROSS_IS_RIGHT`**, save-steer-1003 (LOG5087-98): the
   new `st=`/`trk=` columns show the track turning back (LOG5090, 5093:
   trk -6 -> +33, xt -103 -> -19) where the old sign turns away (LOG5087:
   trk -7 -> -64). **Default** (fingerprint e06a83a7). `ROLLOUT_STEER_PID`
   (save-steerpid-1003, LOG5099-5110) could not be told apart from the
   default, because touchdowns were already 5-390 m off.
2. **The approach handed the flare a vehicle 60-525 m off the centreline.**
   `APPROACH_CAPTURE_KP` 2 deg per m/s gives the rate loop a 2.9 s lag. The
   shuttle's roll arrives in 5.3 s (time_to_peak), at 5-8 deg/s. The result
   was a relay: +-40 deg alternating, the flown bank overshooting to 61. On
   top of that, the S-turn stopped at a distance, still running ~44 m/s
   sideways (LOG5096, door +525 m).
   **`APPROACH_CAPTURE_LAG_AWARE`**: the gain is set so the loop's lag is
   2 x roll's time_to_peak (0.55 on the shuttle), and the S-turn asks for
   no more sideways rate than `lateral * (t_door - lag) / 2` can take back.
   save-lag-1003 (LOG5111-22): **doors 1-22 m off on 5 of 6** (defaults
   4-473), stops 21-61 m off. **But 4 of 6 overshot 1.8-6.7 km into the
   sea.** The relay had been the approach's only dissipation, and at the
   gentle gain the weave banked 15-25 deg against 45 asked.
   **`APPROACH_SCURVE_FULL_GAIN`** gives the weave back its full gain.
   save-fullgain-1003 (LOG5135-46): **every stop within 73 m of the
   centreline, 9 of 12 on the strip**. Along: +0.9..+1.8 km on most, and
   +4.5..+7.4 on hac1/hac4. One flight landed on the runway with 31/31
   parts (LOG5141, +861/-27).

**The energy comes from the cone.** 10 of 12 cones roll out with +840 to
+3935 m of surplus (conesum, LOG5135-46: the fit would be HAC_LD 1.40 sd
0.15 against 1.86 planned). `HAC_EXIT_SURPLUS_DERIVED` hands the approach
anything less than a lap, and a lap is ~4-7 km of height. The approach
spends height only by banking. `HAC_RADIUS_MIN_M=1000` (save-rmin-1003,
LOG5123-34) changed nothing: still laps=0. The airframe's hold radius at
~100 m/s is ~1.3 km, and a lap there still costs more than the surplus.

**Constants replaced by runtime measurement** (the user's request; all off
by default):
- `FLARE_LEAN_BY_ROLL`: `FLARE_WINGS_LEVEL_M` / `FLARE_BANK_TAPER_M` as
  times, from the measured roll rate plus roll's time_to_peak.
- `FLARE_ALIGN_BY_YAW`: `FLARE_ALIGN_ALT_M` as yaw's time_to_peak (19.7 s
  on the shuttle, against the 3 s its comment assumed).
- `APPROACH_SCURVE_PERIOD_BY_ROLL`: the weave's half-cycle as 2 x the bank
  reversal time.

All three together were not separable at n=6 (save-fullgain-1003 arm 1).
`HAC_LD` is not substituted by the achieved ratio, because that failed on
the old craft (journal 2026-09-21). `HAC_LD_MEASURED` scales the table's
ladder by the measured L/D and is in flight now (save-ldmeas-1003).

**From the cone saves the shuttle never has a speedbrake.** The brake is
measured in vacuum, and the saves start at 13 km ("no brake armed", "no
measured spoiler set"). From orbit it is armed and the approach never
deploys it either (`ab=--` on all 50 lines of LOG5075).
