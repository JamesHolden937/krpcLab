# boosterland journal

## Session, 2026-10-18 evening: the flip's throttle (from ascent)

The user, working on `ascent/`: "do low throttle during flip, just enough to
flip fast; calculate what throttle and therefore gimbal authority would be
most efficient on the fly."  ascent's budget had shown the full-throttle
flip as the biggest single cost in the boostback (955 v 787 m/s pointed
already, heavy at the ascent's separation state).

`FLIP_THROTTLE_LAW` (`guidance.flip_throttle`, `drift_rate`, `turn_time`;
`Autoland.flip_law`, `flip_authority`): cost(u) = u*a*t(u)*(1 - sin th/th)
+ D*t(u), t(u) the bang-bang turn at u*gimbal + RCS/wheels, gimbal from the
engines' geometry (x0.9), RCS/wheels read live (max kept), D = v_away/T +
x/T^2.  Re-solved every tick until the nose is within FLIP_ALIGN_DEG.

- `flipheavy-1018` (qs_heavy, 6 v 6, 1x): boostback 27.9 v 29.5 t (-1.6 t);
  flip 5.8-6.3 s at mean throttle 0.08-0.16, 1.3-1.45 t; boostback exit miss
  identical (~170 m); touchdown mass +0.75 t; stood 6/6 v 4/6; lands long
  +27..+57 m E (median 43 v 20 m).  First tick: gimbal 1.32 rad/s2 per
  throttle, RCS 0 (valve still shut), inertia 5651 t m2, drift 5.3 m/s/s ->
  u 0.15.
- `flip1-1018` (ascent stacks): shuttle stack boostback 20.3 v 23.6 t.
- Promoted to the default.  Open: the +45 m long on qs_heavy (the coast is
  1.6 t heavier); the coast items in HANDOFF (a: steered miss hides the
  saturation; b: brakes for speed, not range; c: aoa as a speed brake).

## Session, 2026-10-17 night to 10-18: timed legs, late brakes, standing up

All batches 1x, aim bias 0, `logs/rot-<tag>.txt`; saves qs_heavy, qs_hot,
quicksave; 12 a cell unless said. The user asked: time how long the legs take
to deploy and drop them that long (plus margin) before the ground instead of
at the burn; cache each leg's time so it is measured once ever; aim long on
closed brakes and open them fully late, for a slower, cheaper burn; improve
accuracy; fix tip-overs.

- **Leg timing** (`GEAR_TIMED`): kRPC exposes neither action-group
  membership nor a part's animation length (it is in the binary model, not
  the cfg), so the coast cycles the legs once and times each part to
  `deployed` and back; `boosterland/gearcache.py` keeps the time per part
  name in `boosterland/data/gearTimes.json` (gitignored, shared by the farm,
  locked, atomic). LT-2 (`landingLeg1-2`): 4.6 s down, 2.2 s up. The legs
  drop when the smaller of h/|vs| and the prediction's time to land is
  below factor x deploy + `GEAR_LEAD_S` (1 s). Factor 2.0 put them down at
  ~3 km, earlier than the old burn-start trigger (~1.35 km): ab1-1017 heavy
  median 74 v 39 m. Factor 1.25 (~1.9-2 km): tip1-1018 38 v 56, fl1-1018
  (18 a cell) 48 v 47, standing 14/18 v 12/18, burn fuel 7.77 v 7.91 t.
  **Promoted at 1.25.**
- **Late brakes** (`DEPLOY_LATE`): after the coast's descent re-sweep, the
  brakes go to full, all Mach bins are probed (the full-brake curve), they
  return to the law's minimum and a *replacing* re-sweep takes the closed
  curve. The env carries `late_switch_alt`, so every prediction flies "closed
  until h, full below"; a bisection on the along-track miss picks h. The
  first try measured mid-flip and read 0.92x (LOG11855, 186 m out): gated on
  the re-sweep and descent attitude, a ratio <= 1 falls back to the ordinary
  law. Full brakes give only 1.17x the drag of 30 deg -- a few hundred metres
  of range. Heavy over three batches: ignition 264 v 287-290 m/s, burn fuel
  7.38/7.49/7.45 v 7.87/7.88/7.91 t, worst miss 65/69/87 v 102/93/141,
  median 51/48/61 v 39/56/47. qs_hot/quicksave: fuel within 0.1 t. **Promoted.**
- **`DEPLOY_RESWEEP_REPLACE`** (the ordinary law's re-sweep replacing rather
  than blending a quarter in): ab1-1017 miss null, 5 touchdowns > 6 m/s v 0.
  Deleted (the replacing sweep stays: `DEPLOY_LATE` uses it).
- **Tip-overs were invisible**: the sign-off came 2 s after touchdown. The
  touchdown is now watched `TOUCHDOWN_WATCH_S` (8 s) more and the sign-off
  says `tilt= max=`. On the old defaults the heavy stood 8/12, 6/12, 5/18
  (several "No such vessel" FATALs are breakups, not harness errors) and the
  18-part legless booster 0-3 of 12.
- **Not the ground hold**: the burn's last command (10-12 deg off vertical)
  stays on the autopilot after touchdown and the valve opens on the error,
  inputs pinned -- it looked like a lever. `TOUCHDOWN_ATTITUDE` vertical /
  release (tip1-1018, standing of 12 on heavy/hot/qs): last 6/3/5, vertical
  2/2/1, release 4/0/2. A released booster at rest fell 5 s later with no
  input. Deleted.
- **The lean at touchdown**: below 250 m the base was retrograde clamped to
  15 deg of vertical *all the way down*, plus the lateral nulling's tilt; as
  the descent stops, retrograde swings toward the horizontal, so the last
  burn tick sat 7-15 deg off vertical on nearly every flight, and the heavy
  tipped at 0.4 m/s sideways (LOG12086, cv 12.7). `LANDING_UPRIGHT_ALT_M`:
  the whole command narrows linearly to `LANDING_UPRIGHT_TILT_DEG` (2) at
  the ground. up1-1018 (18 a cell) heavy standing base 5/18, 20 m 14/18,
  50 m 14/18; last-tick lean 7-13 v 3-7 v 2-5 deg; worst 87/120/87.
  **Promoted at 50.** fl1-1018 base (with it): 12/18.
- **The legless booster cannot stand**: quicksave/qs_hot on any arm 0-3/18.
  It stands on one engine bell; the airbrakes and probe hit the pad as it
  goes. Some flights also hit the launch clamps (debris left on the pad from
  launch, `girder_BOXCOLLIDER` at 12 m in CollisionSpy). qs_hot also arrives
  at 5-8 m/s sideways: its burn runs 11-19 deg off its command at full
  throttle and builds the drift at ~100 m.
- **Two 40-50 km "misses"** (LOG11997, LOG12144) broke up at separation
  (26.7 -> 0.66 t two seconds in), not in the landing.
- **Defaults smoke** (def1-1018, 6 a save, fingerprint 9cba676e): heavy
  standing 5/6, ignition 263 m/s, burn 7.43 t, legs down at ~1.8 km (6.7 s
  to go), median miss 72 (n=6); qs_hot 0/6 standing, median 30.

## Session, 2026-10-17 afternoon: the heavy's burn spin, roll held all flight, the burn's command whip

All batches 1x, aim bias 0, `logs/rot-<tag>.txt`, 12 a cell unless said.
The user asked: why not keep the other engine sets off until the current one
is saturated; force roll throughout the flight; make the flips to and from
boostback pure pitch through the vertical.

- **Staggered throttle**: the code already fills one group to full before
  the next lights (`engines.allocate`); the how-it-works figure made it look
  otherwise (demand is a share of the *whole* cluster). New telemetry `eng=`
  (each group's delivered thrust, of its maximum) confirms it in game
  (diag1-1017): `0.94/0.37/0.00`, `0.94/0.09/0.00`. 0.94 = sea-level thrust
  over the vacuum figure the allocation divides by. Figure redrawn.
- **The heavy spun in the landing burn** (diag2-1017, new `r=` roll rate,
  `in=` inputs, `w=` nose rate): 85-257 deg/s through the whole burn.
  kRPC credits the cluster ~0 roll torque (0.006 kN m unlit, 0.155 lit) and
  tunes roll gains 1e3-1.7e5; the off-axis Vectors' gimbals roll it hard.
  The coast had left roll uncontrolled (kRPC's `roll_threshold` 5 deg, the
  coast flies 6-10 deg off) and 142 deg off by ignition, so the engines lit
  with roll pinned: 0 -> 200 deg/s in 1.3 s (LOG11353), coning, tip-over.
- **`GIMBAL_ROLL_OFF`** (`ModuleGimbal.enableRoll`, field id, verified to
  stick), groll1-1017 heavy: burn spin 0 v 258 deg/s, miss median 20 v 58,
  touchdown 5.8 v 8.7 m/s, > 10 m/s 1 v 4. **Promoted.** qs_hot (one
  on-axis engine) unaffected, as it must be.
- **Forcing roll** (hold1, tr1, seed1-1017). Four things stood in the way:
  1. kRPC's `roll_threshold` (5 deg): above it roll is not controlled.
     `ROLL_THRESHOLD_DEG` 180.
  2. The roll reference kept the roof "up" through a flip over the top,
     which physically carries it down: a half roll commanded mid-flip.
     `ROLL_TRANSPORT`: the last roof carried by the nose's own rotation
     (`guidance.transport_roof`); qs_hot coast roll error after the flip
     82 -> 2 deg.
  3. The seed ("roof away from the body") was 90-170 deg from the roll the
     saves separate in. `ROLL_SEED_ACTUAL`: the side nearest the vehicle's.
  4. With gimbal roll off the heavy has no roll actuator (RCS 0.15 kN m:
     input pinned 15 s, ~1 deg/s). `GIMBAL_ROLL_UNTIL_BURN`: gimbal roll
     through boostback (it rolled 109 -> 1 deg in 4 s, LOG11353), off at the
     landing burn.
  Plus `ROLL_PLANE_MIN_HS_MPS` 5 (the roof carried, not re-taken from the
  plane of the last few m/s of drift). seed1-1017, all v defaults: heavy
  roll error boostback/coast/burn median 0/21/26 v 111/95/96 deg; qs_hot
  burn p90 6 v 147; miss mean 33 v 43 heavy, 39 v 56 qs_hot, worst 101 v
  115, 80 v 212. **All promoted.** The heavy's remaining coast roll error is
  drift with no actuator.
- **Flips through the vertical** (`guidance.flip_through_vertical`, now what
  `FLIP_VIA_VERTICAL` flies; the great-circle `flip_waypoint` deleted): two
  pitches, up in the nose's vertical plane to the zenith tilted 10 deg toward
  the target, then down in the target's. In game the coast flip is pinned
  pitch, roll and yaw input ~0. Accuracy alone: not separable from scatter
  (heavy flip arms 32, 48 v defaults 42, 20 in two batches).
- **Measuring roll mid-flip**: `roll=` projects the commanded roof onto the
  plane of the *actual* nose, meaningless while the nose is 90 deg from its
  command -- the "178 deg in every flip" of three summaries was that. Read
  roll error on ticks with err < 5, and the roll rate.
- **The gimbal delivers what kRPC credits** (torqueprobe, qs_hot in the air,
  pitch/yaw pinned +1 and -1 so the background rotation cancels): 91-93% at
  throttle 0.3 and 1.0 (0.62 v 0.68, 2.09 v 2.28 rad/s^2). Not authority.
- **The single-engine burn tip is the command whipping** (cmd1-1017, new
  `cv=` command off vertical, `cw=` command rate): in the last 60 m the
  command swung 57-115 deg/s across 30 deg (retrograde in a 15 deg cone plus
  a 15 deg velocity tilt whose 4/t_go gain saturates and reverses with the
  sideways speed); the vehicle chased it into a ~3 s limit cycle at
  throttle ~0.3.
- **`LANDING_TOTAL_TILT`** (15 deg on the whole command), slew1-1017:
  heavy median 62 v 25 m -- takes divert authority. Deleted.
- **`LANDING_SLEW_SHARE`** (the command moves at most share x the vehicle's
  angular acceleration per second), slew1-1017 at 0.5 read once a second:
  heavy max burn err 13 v 39 deg, touchdown 3.8 v 6.0, worst 54 v 119 m;
  qs_hot barely bit (command still 80 deg/s at 46 m in LOG11667, destroyed)
  because the full-thrust rate outlived the throttle cut. Now scaled by each
  tick's throttle.
- **slew2-1017** (throttle-scaled slew, 0.5 and 0.25 v unslewed): 0.25
  qs_hot miss median/mean/worst 12/13/25 v 22/30/66 m, touchdowns > 10
  m/s 1 v 4, burn err 15 v 50 deg, command rate in the last 60 m 26 v 96
  deg/s; heavy err 7 v 37, touchdown 0.8 v 5.9, worst 90 v 112 -- but
  median 56 v 32 (open). 0.5 in between on both. **0.25 promoted.** The
  sim (fakeksp, no `available_torque`) then landed at 10.8 m/s: the slew
  had fallen back to its 5 deg/s floor for the whole burn. An unreadable
  torque now means no limit.

## Session, 2026-10-16/17 night: the paired flags promoted, the landing burn's attitude

All batches 1x, aim bias 0, `logs/rot-<tag>.txt`. The user asked to keep
working; mid-session, "how accurate is aim now" and "how would we get the
remaining 40 m".

- **pair1-1017** (4 arms x heavy/quicksave/qs_hot, 12 a cell): both flags
  v off, pooled 35 a side: mean 47 v 82 m, p90 94 v 190, worst 162 v 431,
  touchdowns > 10 m/s 6 v 9. `DEPLOY_UNSTEERED` alone 51/115/199, `HOLDABLE`
  alone 66/154/290. **Both promoted.**
- **Destroyed vessels**: a crashed booster's streams freeze (same position,
  86 m/s, 0 t), so LOG10748 and LOG10512 flew a dead snapshot to the 420 s
  timeout and read as "hovers". Now a bit-identical position for 1 game-s
  while moving ends the flight as `destroyed`; a > 10% mass drop in one tick
  is logged as parts lost. 4 of 4 crashes since were caught within 1 s.
- **LOG10720 and LOG10774** (qs_hot, boostback timeouts, 88 / 52 km): the
  tanks were lost 2 s into the flip (26.7 -> 0.66 t at Mach 2.8-2.9, 146 deg
  off). 2 of ~110 qs_hot flights. KSP.log was gone (farm restarted). Open.
- **`LANDING_FEEDFORWARD_FLOOR`** (refuted, deleted; failure 5 amended):
  every flight idles 0.3-1.9 s after the landing-burn trigger. It is not the
  margin thrown away -- the booster is at terminal velocity then and drag
  brakes it. In the closed-loop sim the floor turned the 320 kN LOG7
  booster's predicted burn into a hover from 20 km (boostback never
  converged, 76 m/s 10 km out v 7.4 m/s 148 m).
- **brd2-1017**, the new defaults on six saves, 12 a cell: median 37 m, mean
  47, p90 90, worst 269; steep 15, cold 31, quicksave 38, heavy 42, hot 47,
  north 73 (sd N 98 m, the outlier). No timeouts.
- **Where the 37 m is made** (brd2): 46/72 lit the burn predicting <= 10 m
  and landed a median 31 m out; landed distance correlates 0.72 with peak
  tilt in the burn (0-15 deg: 17 m; >= 25 deg: 56 m). The other 26 lit the
  burn already 10-250 m off (all 12 heavy flights: 19-95 m; the heavy's miss
  grows from 8 km to 3 km, e.g. 51 -> 101). The deploy law has no altitude
  floor (removed 10-16 evening) -- so that growth is the law's behaviour
  low down, not its absence.
- **The tip-over** (tip1-1017, telemetry now has `cmd= err= sat= roll=`):
  the command stays within 0-15 deg of retrograde and the nose drifts up to
  78 deg off it -- not commanded. kRPC's auto-tune re-derives the gains from
  the available torque: thrust adds the SSME gimbal (112 -> 623 kN m) and
  pitch Kp fell 7.6 -> 0.49, Ki 7.8 -> 0.18 (LOG11023); the crashed burns
  were flown at 0.1-0.4 input while 70-80 deg off.
- **`LANDING_FREEZE_TUNE`** (auto_tune off at the burn's start; refuted,
  deleted), frz1-1017, 36 a side: median 20 v 34 m, on-target-at-ignition
  15 v 27 -- but **4/18 destroyed on qs_north** v 0, touchdowns > 10 m/s 20
  v 8, peak tilt 25 v 15. The frozen gains were whatever the RCS valve left:
  LOG11037 froze at 1.5 kN m (valve shut) -> Kp 209, and oscillated at full
  input. **The RCS valve swings kRPC's gains 30x** every time it flips.
- **`LANDING_RCS_HELD`** (valve open through the burn), rcsh1-1017, 36 a
  side on qs_hot/qs_north: median 23 v 34 m, on-target-at-ignition 17 v 32,
  peak burn tilt max 34 v 81 deg, destroyed 0 v 1, touchdowns > 10 m/s 14 v
  18. **Promoted.** Its two worst (312, 186 m) were coast-made: brakes pinned
  at 90 deg 20 s into the coast, 298 / 193 m off at ignition.
- Still seen with the valve held: pitch Kp ~6 -> 0.5 under thrust, and the
  roll error reaches 110-135 deg in every arm (roll torque 1.5 kN m; the
  roll reference follows the trajectory plane, undefined as hs passes 0).
- **`LANDING_ROLL_FREE`** (damp roll only in the burn; deleted), roll1-1017,
  36 a side: on-target mean 22 v 21 m, tilt and hard touchdowns equal --
  null. The roll error is not what tips the booster.
- **The boostback breakups are collisions with the upper stage.** KSP.log
  (read before restarting, ksp1) for LOG11177, the third qs_hot tank loss:
  3 s after separation the booster's airbrake hit the upper stage's
  ServiceBay at 40 m/s relative, then its core the stage's verniers, and the
  Rockomax32 joint broke. `FLIP_UNDER_POWER` lights the engines on the first
  BOOSTBACK tick with the nose still near prograde -- at the stage -- and the
  SEPARATION scan had cleared thrust along the *aim* (its own docstring said
  so). **`BOOSTBACK_CLEAR_NOSE`**: mid-flip, hold the throttle while full
  thrust along the present nose would close on a neighbour. Sim: holds 1.8 s
  with a stage 40 m ahead, then lands. nose1-1017 (qs_hot/quicksave/heavy, 36
  a side): fired on 4/12 qs_hot flights (0.2-0.5 s), never elsewhere;
  median 33 v 28 m, mean 42 v 39 (scatter); CollisionSpy contacts above
  1000 m: 0 in both arms (the base rate is ~1 in 50 qs_hot flights, so this
  batch could not show it). **Promoted** on the mechanism; the breakup rate
  is not yet re-measured.
- `LANDING_RCS_HELD` left the valve open on the pad (testFlightSim caught it
  after the promotion was pushed): it now closes when the burn ends.
- How-it-works PDF rewritten (defaults `b3b48d4a`).

## Session, 2026-10-16 late: the holdable angle, computed

The user asked (1) what the docs say causes the miss, (2) whether the booster
can work out how far it can angle itself before planning, (3) whether
boostback could be more accurate, (4) why the deploy law oscillates -- "we
need a gentle one ... a hand tuned PID loop is probably bad for generality".

- **Boostback is not the error.** 60 flights (new1, alloc2): it exits at
  long -133..-188 every time (the deliberate tolerance lead). The miss is
  made in the coast: on quicksave the prediction 5 s into the coast reads
  -80..+1500 (the drag curve re-measured at descent attitude) and the deploy
  law still brings most flights to ~0 at the landing burn.
- **The heavy's 300-400 m tail** (LOG10598, 10618, 10667, 10674, 10592): from
  25 km to 10 km the commanded aoa climbs 1.8 -> 10 deg while the booster
  holds 0.3-3; the steered prediction reads ~0, so the brakes sit at 30; at
  10 deg (9 km) the miss jumps 0 -> +280 with 20 s left. Tracking read 1.00
  (commanded 1.6, held 2.4 -- a small-command sample).
- **The deploy-law "oscillation" is not a gain problem** (LOG10637, 10622):
  the lift solve re-nulls the *steered* prediction every tick, so the brakes
  see ~0 until the lift saturates and the hidden miss appears at once
  (-500 at 12.7 km, +240 at 4.7 km), and the brakes take a 20 deg step
  each time (30->50->30->50). Separately, with brakes held at 30, the
  prediction went -1 -> +850 in 8 s as the re-swept curve dropped Cd*A
  18 -> 6.
- **`AERO_STEER_HOLDABLE`** (flag, off): `simulate_aerodynamic_torque_at` at
  a few tilts off retrograde (vessel as it is now), restoring moment per Pa
  per Mach bin, against wheels + RCS (highest seen) + surfaces*q with
  margin 0.7; the solve commands no more and the propagator caps lift at it
  per step. Smoke LOG10686 (heavy, DIAG): moment restoring, ~490 kN m of
  control (all RCS), limit 10 deg at low q falling to 5.5/3.0/1.7/0.9 deg
  at q 16-50 kPa -- the tracking sample in the same flight read 2.7 -> 2.0.
  **hold1-1016** (12 a cell, 1x, aim bias 0, swap 21.6 GB at the end):
  heavy median 49 worst 94 v 87/180 -- better; qs_hot 60/189 v 33/113,
  quicksave 35/243 v 20/236 -- worse on the original booster, unexplained
  (its `can=` reads 10 at 26 km, ~5 at 7 km; the solve rarely asked more).
  Not promoted, not deleted: it is to be flown paired with the next flag.
- **`DEPLOY_UNSTEERED`** (flag, off, unflown): the deploy law nulls the
  *unsteered* prediction, so drag carries the range and the lift only trims
  what is left and the cross-track. No new constant.
- Not the flag: LOG10720 (hold, qs_hot) BOOSTBACK TIMEOUT 180 s, splashed
  88 km -- boostback never converged; LOG10748 (hold, qs_hot) hovered to the
  420 s harness timeout. Both open.
- `boosterland/tools/brot.sh` (the rotated batch, `LAPS=`) moved into the repo.

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

## Housekeeping, 2026-10-18 (fingerprint `9cba676e` -> `c7b78daa`)

No flights. Five flags that sat off by default were deleted with their code
(the user: "delete the flags"); the defaults fly exactly as before, only the
fingerprint moves because the fields are gone:
- `SEPARATION_PRETURN` (turn during the separation drift even with nothing
  in the way; the turn while the clearance scan holds stays).
- `CORRECTION_GRADIENT_POWERED` (the gradient probes fly no landing burn).
- `CORRECTION_RETIRE_ON_BACKFIRE` (a backfire ends the burn, not the phase).
- `DRAG_PROBE_AXIAL` (measured worse: read 10.8-14.2 m^2 against 17.4-18.8,
  landed 3.3 km short) and `DRAG_PROBE_DESCENT` (pd10-1016: 3 of 4 heavies
  lost, 628-884 m); `probe_direction`/`probe_rotation` now return the real
  airflow and attitude.

`gearcache.PATH` fell over in a `tools/bundle.py` single file (no
`__file__`), so `dist/boosterlandAutoland.py` had failed its selftest since
`GEAR_TIMED`; it now sits beside the script there. Unreferenced logs older
than a week are zipped in `logs/archive/LOG<k>xxx.zip`.

## Session, 2026-10-18 day: the cross-track trim, the boostback yaw, the pad

The user asked to keep eliminating the miss, and whether the yaw at the end
of boostback was intentional. Fingerprint `c7b78daa` -> `bb3fa2e6`. Batches
in `logs/rot-<tag>.txt`; all heavy unless said.

### The yaw (not intentional)

Boostback aims opposite the predicted horizontal miss. In its last ~2 s the
along-track part closes and the ~100 m cross-track takes over the direction:
the command swung 30 deg sideways at 41 deg/s at 2% throttle (LOG12455
`cw=41 err=30`, yaw input -0.94). **`BOOSTBACK_HEADING_HOLD`** (default):
the aim at the first taper tick (nose within 25 deg) is held, only the miss
along it is burnt off, and the exit tests that miss, not the distance.

The swing had been opening the RCS valve (error relay) right before the
coast, and the coast's flip ran on that by luck: with the swing gone the
valve stayed shut through the flip (saturation rule) and kRPC sent **zero
input at 100-145 deg of error** -- the airflow turned it (LOG12465).
**`COAST_FLIP_RELAY`** (default): the error relay until the coast has
*settled* (nose within 10 deg), then the saturation rule.

### The heavy's miss was its cross-track

On qs_heavy the cross-track at burn entry equalled the N/S touchdown miss to
~5 m (LOG12453-12462: -74/-98/-90/-44/+8 v N -70/-92/-92/-42/+4); 83
flights of three earlier batches averaged N -30. Nothing after boostback
moves it: the heavy's wing solves aoa 0, CORRECTION is downrange and rarely
flown, the burn is held upright. **`COAST_RCS_TRIM`** (default): once
settled, translate on the Vernors against the cross-track (start > 10 m,
stop < 3 m, input cross/30 m), world direction to vessel frame through the
attitude quaternion, valve held open only while pushing. -63 -> -3 m in
2.2 s (LOG12466). The first version held the valve open all coast: roll
input pinned at -1, 1.6 t of propellant (LOG12465); push-only costs
~0.15-0.2 t.

| batch | arm | heavy miss med / worst | stood |
|---|---|---|---|
| trim1 | base / hold / trim | 35/83, 67/90, 26/69 | 10, 12, 6 of 12 |
| trim2 | base / trim | 47/74, 17/43 | 10, 6 of 12 |
| trim3 | trim | 22/86 (n=10, ksp5 dead) | 7/10 |
| def2 | new defaults | contact 13 / 33 | 10/12 |

`hold` alone is worse (67): it leaves the cross-track for nothing to fix.
qs_hot: trim null (12 v 29, 29 v 18): its correction burn already clears the
cross-track. Its legless booster falls every time anyway.

### Why the heavy falls over: the pad

The reported miss of a fallen booster is where the wreck lies, ~25-35 m
from where it touched -- use the first TOUCHDOWN line's `pad=`. By contact
distance (trim1-4): **0-9 m 16 stood / 5 fell; 10-34 m 18 / 33; 35+ 63 / 6.**
KSP.log CollisionSpy (trim4): only 4 of 30 falls touched a launch clamp
(`girder_BOXCOLLIDER`); at 14-18 m the engine bells land on the pad
model's **"Fuel Tank"** collider. Base stood more often only because it
missed by 40-90 m, off the pad. Standing needs < 10 m.

### Refuted, deleted

- **`COAST_RCS_TRIM_LONG`** (trim the along-track too, once the brakes are
  handed over). The Vernors' axial coupling left trim arms +20-25 m long at
  the burn and the brakes, at 90 already, cannot add drag. It fixed that
  (-2 at the burn, trim3) but not the landing: 26 v 22 (3/10 stood), def2
  12 m but 4/12 stood, worst 59, +0.4 t. First version read a deleted
  field `DEPLOY_MIN_ALT_M` and crashed 22 of trim2's 24 tlong flights.
- **`LANDING_RCS_TRANSLATE`** (below 50 m, translate against hs + offset/3 s).
  The last 30 m drift 9 -> 20 m from the pad as the cone's 2-4 deg of tilt
  pushes ~0.5 m/s^2 sideways (LOG12618: hs 1.4 -> 3.7). Saturated `up`
  input for 4 s did not stop it (LOG12653). def2: 16 m 9/12 v 13 m 10/12.
  The `up` axis sign is unproven -- it took the sim's convention; the
  coast trim only exercises `right` (the roll hold puts cross there).
- **`GEAR_BURN_TTG`** (the user: "legs deploying a bit early?" -- yes: the
  time-to-go is the fall at coast speed, legs down 13.4-14.1 s before
  contact for a 6.8 s lead). Timing on the burn's powered prediction
  still dropped them ~11 s early (its ttl reads short early in the burn:
  4 at 9.7 s actual) and def2 was worse: 22 m, 7/12. The lead's real
  value is now written beside `GEAR_LEAD_S`.

### Traps paid

- A sign check on a *combined* push misread drift as a wrong sign and
  flipped it (LOG12652, 101 m): check only until one push has closed.
- Every new `cfg.X` read must exist: a test that runs the method on a bare
  `object.__new__(Autoland)` catches it; so does grepping the diff.
- `rot` summaries that drop "no exit line" rows drop every fallen booster:
  my first trim1 headline (median 10) was 26 with them in.
- Asked whether the airbrakes could roll the heavy: they cannot -- their
  force lies in a plane through the axis. Roll is gimbal (lit) and Vernors.

## 2026-10-19 afternoon: FMRS boostback button

`boosterland/fmrs.py` + launcher button: load FMRS's separation save paused,
switch to the booster, unpause on boosterland's first tick (no clicking
time spent falling). The live `FMRS_save_0` was a pad split (stack staged
4 s after launch), so the button takes the first `FMRS_save_<n>` with a
booster in flight. Test on ksp0 (LOG12937 load-only, LOG12940 flown): an
ascent-on-`qs_stack` separation at 20.5 km / Mach 3; boostback to 179 m,
then the coast diverged below 17 km (cross -2.2 km, long -8.6 km) and the
booster was destroyed 8.9 km out. One flight; not chased.

## 2026-10-20: FMRS button -- booster active on load, jump back to the main mission

The user asked for two things: no vessel switch after the load (edit the
save so the booster is the active vessel, then load it), and a JUMP BACK TO
MAIN MISSION button once the booster is down or destroyed, which writes the
booster's final state into the main mission's save before loading it.

`common/sfs.py` edits a save's FLIGHTSTATE vessels as text (CRLF kept).
`fmrs.py` now saves the live game as `krpclab_main` (unique name first, so
the save folder can be found on disk), picks the booster in the FMRS file
itself (closest in-flight vessel with an engine module, within 5 km, instead
of the heaviest via kRPC), writes `FMRS/krpclab_boostback` with it active,
and loads that. At the end it offers the jump: `krpclab_booster` is saved,
its booster block replaces the main save's copy (removed if destroyed),
`activeVessel` is kept on the main vessel, and `krpclab_main` is loaded.

ksp0: LOG13114 (main = qs_heavy, whose active vessel *is* that booster --
the jump refused to remove it; now checked before flying) and LOG13115
(main = FMRS_save_0): load to first tick 2.6 s wall, booster destroyed
91 m from the pad (ksp0's old FMRS save; it falls over), removed from the
main mission, which reloaded with the upper stage active at 39 km. The
landed merge is covered only by `tests/testFmrs.py`.

`fakeksp.Vessel` had no `moment_of_inertia`, so FLIP_THROTTLE_LAW made 20
closed-loop tests error since 2026-10-18; added (a cylinder's inertia).

## 2026-10-27: quickfly flies the bundle; a 1x load is a different booster

quickfly now starts the autoland as a child from `dist/krpcLab.py`.  Pausing
the game until the child was up landed 0.2-0.8 game-s late at 4x (a pause
waits for frames); starting "as soon as up" was 0.5 s later under PyPy than
CPython.  Final form: the child loads the save itself, the instance stays at
1x through load and start-up, the first tick waits for save UT + 4.6, the
harness raises the scale afterwards.  Start 16681.78 +-0.02 on every flight.

At that UT the 1x-loaded booster has hs 699.6 m/s on every flight; the old 4x
load gave 701.7-702.8.  Fresh farm, interleaved, 6 v 6: old harness 23, 29,
32, 61, 139, 155 m; new (CPython) 19, 64, 69, 170, 259, 361 m, all west.  The
earlier 6 v 6s flown on a farm at 20-22 GB of swap are discarded.  Under PyPy
the booster serves its 0.05 s tick (~2470 coast ticks) where CPython gave
~0.07 s (~1650); not shown to matter.

