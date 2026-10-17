# boosterland journal

## Session, 2026-10-16 evening: a general deploy law, engine allocation

The user, on the deploy law's 8 km cutoff: "oscilation is a control issue,
not something that can't be fixed"; asked for a controller that works for
ships with different gains, an audit of what else is missing, and engine
allocation (center engine to full first, then pairs) as a general solution.

- **Deploy law, general** (default): every 10 s a Newton step on a computed
  gain. S = metres of along-track miss per unit ln(Cd*A), from two
  propagations (drag x1 and x1.05, same steering); k = ln(Cd*A) per degree,
  from the drag curve swept 2 s after a step against the one before it,
  median over Mach bins (first step a fixed 10 deg probe). Step = 0.7 *
  -long / (S k), rate-limited 20 deg; acts the whole coast; `long` along the
  horizontal velocity (the line to the pad swings round overhead). No
  per-craft gain constant. full1 (whole coast, old fixed gain): airbrakes
  pinned at 90 on quicksave/qs_hot with long misses -- the law had no drag
  left to add.
- **Airbrakes start at 60** (`AIRBRAKE_DEPLOY_ANGLE_DEG`, `DEPLOY_MAX_DEG` 90
  split off): boostback aims on mid-range drag. new1 medians 90 -> 60:
  qs_hot 247 -> 31, heavy 79 -> 63, quicksave 79 -> 55; 72/72 intact.
- **`ENGINE_ALLOCATION`** (default; `engines.py`, `EngineCluster`): engines
  grouped by position across the thrust axis (on-axis, opposite pairs, an
  odd ring whole), filled in order with per-engine `thrust_limit`, main
  throttle at 1; guidance still plans on `max_thrust`. The heavy groups as
  [1 @ 0.0] [2 @ 1.2] [2 @ 1.2]; quicksave/qs_hot are single-engine (their
  new1 "alloc" cells, 24 v 55 and 10 v 31, are scatter between identical
  configurations). alloc2, heavy 12 v 12: miss median 47 worst 79 v 90/362;
  touchdown median 6.7 worst 14 v 4.2/8.7 m/s (open).
- **`RCS_TRANSLATE`** (sideways speed nulled by RCS below 250 m, scaled by
  `available_rcs_force`): refuted, deleted -- heavy authority 0.39 m/s^2,
  harder arrivals (qs_hot 12-13 m/s), misses no better.
- Not built: an engine-latency term in the burn trigger. The qs_hot crashes'
  late throttle was the profile law's own command (the early start puts the
  booster below the profile); the killer was tipping at full throttle. No
  crash in 96 flights since (new1, alloc2, 4 lines/s logged).
- `defaults_fingerprint` on the config line (now `e7e5aa99`); loop-rate line
  sampled in `tick()` (the harness drives tick, so `run()` never sampled):
  landing burn 0.04 game-s per tick.

## Session, 2026-10-16 afternoon: corrections, drag as range, the landing hover

The user asked why the booster needs correction burns at all ("they seem to
just cause trouble"), whether a control was missing, and suggested using the
airbrakes' deploy angle for drag. All batches 1x, aim bias 0, 6 a cell,
logs `rot-<tag>-1016.txt`.

### Corrections

- **noc1** (corrections off v on): heavy 0 v 3 of 6 lost, mean 547 v 741;
  quicksave mean 98 v 342, worst 235 v 1088. Every burn "backfired": COAST's
  miss comes from the *steered* prediction (`solve_steer`), CORRECTION's from
  an unsteered one, so the miss jumps at zero throttle the moment it enters.
- They cannot simply go: the sim's 25 m/s mid-coast shove (failure 1's case)
  lands 1.1-2.9 km out without them, 13-137 m with.
- **`CORRECTION_ENTER_M` 60 -> 1000** (the abort's noise floor split off as
  `CORRECTION_ABORT_FLOOR_M` 60); still catches the shove (151-210 m).
- Still, every flight that made three burns missed by 0.7-2.5 km: at 28 km
  the zero-throttle prediction swings past 1000 m *short*, and a burn within
  50 deg of retrograde only shortens in the game (the sim, with no aero
  torque, can hold the tilt that lengthens). **`CORRECTION_OVERSHOOT_ONLY`**
  (default): never enter on a short miss. The flight-sim kick tests now kick
  long.

### Missing controls: the airbrakes

- `AIRBRAKE_STEER` (pitch/yaw surfaces, re-flown without the confound of
  backfiring corrections, abs1): heavy 2 lost + 1 splash, mean ~1020 v 578;
  quicksave 242 v 160. Deleted.
- Fixed deploy angle from the first COAST tick (dep1; the game takes the
  change in flight): heavy at 90/45/0 lands ~450 short / centred / ~800 long,
  and 0 doubles the cross-track scatter. Probe deleted once the law existed.
- **`DEPLOY_LAW`** (default, `Autoland.run_deploy`): every `DEPLOY_STEP_S`
  10 s the angle moves by long/gain (gain >= 20 m/deg, learned upward), at
  most 10 deg, within 30-90, above `DEPLOY_MIN_ALT_M` 8 km, and the drag curve
  is re-swept (the probe simulates the shape the vessel has now). The first
  version (5 s, 15 deg, gain learned down to 3) limit-cycled 90 -> 20 -> 71 ->
  20 and its late swings threw the cross-track +-400 m (dlaw1, vert1).

### The landing hover (the largest error on both boosters)

Predicted landing point v actual: on quicksave the 2 km prediction was on the
pad and the booster landed 180-720 m off; on the heavy 400-750 m off a
prediction steady from 30 km. Low over the right spot, it hovered at 10-30 m
with the throttle 0 <-> 0.95 every tick and slid sideways. `landing_command`
flew the profile on |v|: 15 m/s sideways read as 15 m/s against a 3 m/s
target -> cap (zeroing the divert's tilt budget) -> climb -> the go-around
guard cuts -> fall -> repeat. **`LANDING_THROTTLE_VERTICAL`** (default): the
profile is on descent rate, with the sideways speed weighted by
height / `LANDING_NULL_ALT_M` (pure descent rate hit at 54 m/s in the sim --
the burn must still brake sideways speed high up). vert1 (pure version):
heavy 6/6 at 0.9-8.3 m/s within ~30 m of the 2 km prediction; DIVERGED lines
88-155 -> 5 a flight.

### Where it stands

ov1 (gate + law v gate alone v neither): heavy 17/69/72/75/112/386 v
354-523 v 328-619; quicksave 8-38 (one 245) v one 1.9 km splash v two
2.3-2.7 km splashes. brd1, defaults on all six saves: 36/36 intact bar one
(below); medians steep 12, north 32, cold 59, quicksave 61, heavy 188,
hot 195. Heavy and qs_hot now land ~200 m **long**: the steered prediction
reads ~0 at 20 km and drifts 200-580 m long by 10 km, after the law stops.

low1 (`DEPLOY_MIN_ALT_M` 3000 v 8000, heavy/hot/quicksave): medians 74/54/55
v 122/43/56, 36/36 intact -- null, 8 km kept.

Open: qs_hot LOG10512 crashed at 145 m/s -- burn entered on time at 697 m
(needed 698) but the throttle read 0 at 584 m and came up 0.23 -> 0.95 only
by ~400 m; not explained without DIAG. The steered prediction's lift credit
is the root of both the old backfires and the late long drift.

## Session, 2026-10-16: the heavy booster

The user flew a second booster live -- the first stage of "shuttle first
complete test flight": 35 parts, 99.4 t at separation, five Vectors on an
engine plate, six airbrakes on the top tank, six legs, twelve Vernors as RCS.
It landed 830 m short and broke up (LOG10236).

**New save `qs_heavy`** (`boosterland/tools/splicebooster.py`): the booster
cut from the stack in the live game's FMRS `before_launch.sfs` (the subtree
under `Decoupler.3`, rooted at the top tank), put in place of `quicksave`'s
vessel with its separation state, propellant set to the logged 99.36 t. The
first farm flight reproduced the live failure (LOG10237).

### What the live flight died of

1. **The predictor credited lift the booster cannot make.** Commanded 8-10
   deg off retrograde, it held 2-3 (`hold=` is the real angle) with RCS
   open: six airbrakes at 90 deg on the trailing end make it a weathervane.
   Lift goes as the angle squared, so the solve's prediction carried ~10x
   the real lift and the miss walked 800 m short below Mach 0.9, unseen.
2. **A correction burned past its best**: 900 -> 314 -> 1068 m, 7 t of
   landing propellant.
3. **The landing burn**: the throttle trim gain (0.12 /(m/s)) is twice as
   hard on a 9 g booster as on the 3.7 g original; it bang-banged into a
   climb, and retrograde -- the attitude reference -- pointed down once it
   climbed. Ran dry on the farm; broke up live.

### Changes (all default, all committed)

| change | evidence |
|---|---|
| `lift_slope_at` scaled by the measured aoa tracking squared (`AERO_STEER_TRACK_*`) | reads 0.1-0.3 on both boosters where it samples cleanly; samples after a correction were polluted (46-88 deg "held") until the 10 s settle + over-hold rejection |
| correction aborts against its **best** miss (and a floor of best + `CORRECTION_ENTER_M`) | replay of LOG10237's 900 -> 314 -> 1068 |
| `LANDING_THROTTLE_GAIN` 6 m/s^2 per m/s, divided by max_accel (replaces `LANDING_THROTTLE_KP`) | equal to the old gain on the original booster |
| landing reference: retrograde held inside `LANDING_MAX_TILT_DEG` of vertical below `LANDING_NULL_ALT_M`, vertical when ascending | pure vertical was tried first and lost the free lateral damping at the throttle cap (qs LOG10246: hs 5 -> 14 m/s) |
| RCS: saturation rule (the shuttle's) in COAST only; error relay elsewhere | saturation everywhere left the valve shut while a landing tipped 41 deg (LOG10251) |
| airbrakes deployed at START but **not** made pitch/yaw surfaces | 1x quicksave: 93/141/262/300 m with pitch/yaw on (rt6-1016) v 16-51 m on 6 of 8 with the action-group deploy (ab7-1016, commit 2984d8d) |

### Refuted, deleted

- **Pumping propellant to the top tanks for the coast** (`COAST_PROPELLANT_UP`,
  the user's idea; branch `pump`, commit 5b57168, deleted). Moved 13.6 t,
  centre of mass +2.2 m toward the airbrakes; tracking still <= 0.29, misses
  679/860/958 m + one lost against ~600 without (ab7-1016).
- **`DRAG_PROBE_DESCENT` on the heavy booster** (pd10-1016): 3 of 4 lost,
  628-884 m. Stays off.

### Where it stands (1x, the only valid numbers)

- heavy: lands 2-9 m/s but ~600 m short (trk9-1016: 605/611/695 + one lost;
  rt6: 586/604/639 + one lost). The live code broke it up.
- original `quicksave`: 19/71/84/431 m with hard arrivals (trk9) -- no better
  than 2984d8d's 16-51 m on 6 of 8.

### The lead for next time

The heavy booster's corrections are **deterministic** across flights
(pd10-1016, four of four): burn 1 at ~31 km 855 -> 1190, burn 2
1240 -> 403, burn 3 420-600 -> 780-840. The ~820 m it carries into the
landing burn is made by them. And entering CORRECTION while COAST steers
jumps the predicted miss from ~0 to 300-1500 m at zero throttle: the steered
prediction still over-credits lift. Read a correction tick by tick with
`DIAG_STATE` (trk9-1016 logs have it: LOG10310-10315) before changing it.

### Traps paid for

- **`--timescale` persists on the instance.** The booster has no governor;
  every batch until ~1010 flew at ~4x (including ones with no `--timescale`
  argument, which inherit the last file) and is worthless. Pass
  `--timescale 1` explicitly.
- `sweep.py` runs one job per (save, config) on one instance, sequentially,
  and its LINE regex no longer matches quickfly's output ("FAILED rc=0").
- A worktree's `fly.sh` looks for `ksp<N>` beside itself: symlink them.
- Six instances reach ~20 GB of zram in a batch from a 10 GB start.
