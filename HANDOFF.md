# HANDOFF — read this first, rewrite it last

## 2026-10-19 evening: ascent -- classic MechJeb, follows the upper stage to orbit, turn shape solved per craft (fingerprint `316c85cd`)

All committed and pushed; `docs/ascent/howItWorks.pdf` redrawn for classic
+ the solve. Launcher button renamed "ASCENT TO ORBIT". Farm stopped.

**What the user asked, in order:** (1) follow the ship (upper stage) now that
the FMRS button exists for the booster; (2) use MechJeb's *classic* ascent,
not PVG; (3) make the classic "turn shape" (MechJeb's 40% default) the most
efficient, balancing booster downrange against trajectory efficiency --
**calculated from the craft, per flight, not fitted by sweeping values**
(the user objected twice to sweeps standing in for a calculation, and to
"verifying it lands in a band": the solve's answer is what flies).

**Where it stands (defaults):**
- `ASCENT_TYPE` 0 classic, `HANDOFF` "upper". PVG is not used: it plans the
  full stack and never re-plans after the script stages the booster, so it
  held periapsis at 80 km and burned the upper stage to escape
  (LOG12916-12922). PVG re-engaged after separation: hyperbolic on 4/4
  (rot-upper1-1019, LOG12943-12950). Classic from separation: 80 x 80 km
  on 3/3 (LOG12942, 12949, 12952). `UPPER_GUIDANCE` flag deleted.
- `turnshape.py` + `Ascent.solve_turn_shape` (no flag): at liftoff, sample
  drag (60 `simulate_aerodynamic_force_at` calls, zero AoA), pressure and
  density, read MechJeb's auto path (start 3500 m, end 59500 m on Kerbin),
  fly the ascent model for each shape, set `turn_shape_exponent` to the
  best. Logs `turn shape: solved ...` with the predicted curve.
- `qs_stack` = the user's VAB "shuttle first complete test flight" (70
  parts, the upgraded booster, **two full ore tanks ~34 t** in the shuttle;
  checked part for part against the live craft file).

**Measured:**
- `rot-shape1-1019` (fixed shapes, qs_stack, 6 a cell, *farm at 15-24 GB
  swap*): 0.2 and 0.3 fail 6/6 (flat turn, booster separates at 5 / 11 km,
  upper stage falls back); 0.4 62.13 t sd 0.50 (n=5); 0.5 63.00 sd 0.54;
  0.6 62.89 sd 0.50; 0.7 63.19 sd 0.87 (mass in 80 x 80 km orbit).
- LOG12991 (solve, clean single instance): **solved 0.524, predicted
  64.66 t, flew 64.72 t.** Model curve: 0.10 and 0.28 no orbit (matches the
  sweep's failures), 0.44-0.55 ~64.6, 0.91 63.7. Upper 100.9 t with 37.5 t
  LFO -> **surplus in orbit ~1.3 t LFO, ~70 m/s** (Rhino Isp 340) -- about
  a deorbit burn; the ore payload eats the margin. Told the user.
- Open discrepancy: LOG12991 is 1.7 t above the sweep's 0.5/0.6 cells
  (~63.0, sd 0.5). One flight; the sweep ran on a swapped farm. Fly ~6 of
  the defaults on a freshly restarted farm before reading anything into it.

**Next, in order:**
1. 6 flights of the defaults on qs_stack (clean farm) -- settles the 1.7 t
   question and checks predicted-vs-flown over n, not 1.
2. Run the solve on `qs_stack_small` / `qs_stack_stretch` (they have not
   flown classic or the solve at all; the small booster may not suit the
   separation budget under a classic path).
3. Model gaps worth closing if 1-2 disagree: drag is sampled at zero AoA
   (classic flies a pitch, not prograde -- most off-airflow on flat
   turns); booster throttle-down and coast drag left out.
4. Older items still open from 2026-10-18: learn the budget model's error
   per booster before cutting `RESERVE_FRACTION`; `SEP_ACCEL_RATIO`.

**Traps paid:** `pkill -f <pattern>` from a bash tool call kills the tool's
own shell (pattern in its command line) -- kill by PID. A farm restart
interrupted mid-`start.sh` leaves an instance that answers its port but
never loads a save; a harness then waits 30 min silently (`./stop.sh` and
start again). kRPC `space_center.load("FMRS/x")` reads a subfolder; `save`
does not (writes `FMRS_x`). In MechJeb classic, a *low* turn shape is the
flat early turn.

## 2026-10-19 afternoon: boosterland -- FMRS boostback button; flight code as 2026-10-18 (fingerprint `bb3fa2e6`)

**New this session: `boosterland/fmrs.py`, launcher button "BOOSTBACK FROM
FMRS SAVE"** (`./run.sh --pilot fmrs`). Loads FMRS's separation save
(`FMRS/FMRS_save_<n>`, the first one holding a booster in flight -- the
live `FMRS_save_0` of 2026-10-19 is a stack split on the pad, so it is
skipped), pauses as soon as the game answers, switches to the heaviest
engined vessel in flight near the active one, builds `Autoland` paused and
unpauses: zero game time lost between the load and the first tick
(LOG12940: save_0 skipped, save_1 loaded in 8.7 s wall, BOOSTBACK 0.5 s
after). kRPC's `load` takes the `FMRS/` subfolder; its `save` does not
(writes `FMRS_<name>` in the save folder). FMRS's own `save.txt` is not
touched -- untested whether FMRS objects in the live game. The launcher now
rebuilds its panel when a pilot's load destroyed it.

Test state: ascent on `qs_stack` on ksp0, paused and saved at separation
(20.5 km, Mach 3, 26.7 km from the pad, booster 99 t). Boostback fine (miss
179 m at COAST, 27 m at 20 km) but the coast lost it below 17 km: cross
-2.2 km by 11 km, long -8.6 km at the burn, destroyed 8.9 km out. One
flight, a booster lighter (62 t at COAST) than `qs_heavy`; the save is not
kept (rebuild with ascent + pause at the second engined vessel). Worth a
look as boosterland's next item if FMRS-state flights keep missing.


Full write-up: `docs/boosterland/journal.md`, "Session, 2026-10-18 day".
How-it-works PDF rewritten. Committed and pushed; farm stopped; sleep
inhibitor released.

**The user asked:** keep eliminating the miss; is the yaw at the end of
boostback intentional (no -- fixed); can anything but the gimbal roll the
heavy (only the Vernors; airbrakes cannot); should a reaction wheel go on
(not yet: it would only buy roll, and roll is not costing anything
measured); are the legs deploying early (yes, ~13.5 s before contact for a
6.8 s lead -- a fix was flown and lost, below).

**Defaults changed** (batches `logs/rot-trim1..4-1018`, `rot-def2-1018`):
- `BOOSTBACK_HEADING_HOLD`: the taper flies the aim held at its first tick
  and exits on the miss *along* it. Removes the 30-deg end-of-burn swing.
- `COAST_FLIP_RELAY`: the coast's flip gets the RCS error relay until the
  nose is within 10 deg (it had RCS only by the luck of that swing).
- `COAST_RCS_TRIM`: once settled, push sideways on the Vernors against the
  cross-track (>10 m start, <3 m stop), valve open only while pushing,
  sign self-checked until the first successful push. ~0.15-0.2 t.
  Heavy miss 26/17/22 v 35/47 over three batches; def2 defaults: contact
  median 13 m, worst 33, stood 10/12. qs_hot: null.
- Deleted after measuring: `COAST_RCS_TRIM_LONG`, `LANDING_RCS_TRANSLATE`,
  `GEAR_BURN_TTG` (journal). No flags open.

**Where it stands:** heavy contact median ~13 m (def2), cross-track ~0.
**The remaining miss is along-track, set before the burn**: def2's defaults
reach burn entry +14..+54 m long (median +28); the burn halves it and the
last 30 m add only 1-7 m. The two falls were the two longest (+49, +54 ->
contact 30-33 m). Standing ~50% when it touches 10-34 m from centre: the
KSC pad model's colliders ("Fuel Tank" at 14-18 m) and the launch clamps.

**Next, in order:**
**DONE this evening -- `FLIP_THROTTLE_LAW` (now default; boosterland
fingerprint now `7aa71525`):** the boostback flip burns the throttle that makes the turn
cheapest (`guidance.flip_throttle`: waste u*a*t(u)*(1-sin th/th) against the
drift `guidance.drift_rate` buys back, t from a bang-bang turn at
u*gimbal + RCS/wheels; gimbal from geometry, RCS/wheels read live). Logged
as `flip law:` (inputs) and `flip done:` (seconds, tonnes, mean throttle).
`flipheavy-1018` (qs_heavy, 6 v 6): boostback 27.9 v 29.5 t, flip ~6 s at
mean throttle 0.08-0.16, stood 6/6 v 4/6, lands ~+45 m long (median 43 v
20 m; boostback exit miss identical, so the shift is downstream -- the
coast is ~1.6 t heavier). `flip1-1018` (ascent stacks): -3.3 t boostback on
the shuttle stack, all spent on the climb (ascent models the same law).
How-it-works PDF: the law is page 3; the phase table's BOOSTBACK row (page
2, "full throttle before the turn is finished") predates it -- fold it in at
the next rewrite. Journal: "Session, 2026-10-18 evening".

**Next for the coast (the user, 2026-10-18 evening; another agent will
take these):**

a. **Why the miss reads 0-3 m above ~14 km and jumps to ~40 below.** The
   displayed miss is of the *steered* prediction, re-nulled every tick by
   `solve_steer`, so it reads ~0 while the commanded aoa creeps up (LOG12905:
   0.4 deg at 21 km -> 3.0 at 15 km). The late brakes went 30 -> 90 at
   20.4 km; six open brakes at the tail are a weathervane, the holdable aoa
   is ~3 deg, the solve saturates at ~14 km and the residual appears at once
   (-36..-45 m), partly recovered lower (landed -18). Same mechanism as
   `DEPLOY_UNSTEERED`'s comment (LOG10637). On the ascent's hotter
   separations it is part of the shuttle stack's ~250 m-short landings
   (flip1-1018, law arm: -25..-608).

b. **Brakes for speed, not range.** The user: "do you need to deploy full so
   early? I just want it near terminal velocity by the landing burn." Full
   deploy buys little drag: the measured `Cd*A at 90 deg is N x at 30` is
   1.07-1.20 on most of this session's flights (one 1.62) -- ~6% terminal
   velocity, ~15 m/s at the burn, ~0.3 t. It opens early (4-23 km this
   session) because `DEPLOY_LATE` makes it the *range* actuator: boostback
   aims long on 30 deg and the switch height is solved to pull it back (the
   flight above was +731 m long closed -> opened at 20.4 km). Proposal:
   boostback aims at the pad on the 30-deg profile, lift steering trims
   range all the way down (it keeps authority with the brakes at 30), and
   90 only low -- at a height computed for ~terminal at the burn -- or
   never. Risk: `DEPLOY_LATE` won its comparison (journal), so the brakes do
   real range work; measure against the defaults on qs_heavy + the ascent
   stacks.

c. **Angle of attack as a speed brake** (the user: "lift creates drag, so
   use aoa to make it slower"). Shuttle-style: fly an aoa for drag and roll
   the lift vector round (coning / bank reversals) so the lift cancels.
   Unknown is the drag curve: at the 2-5 deg the booster holds now there is
   no measurable Cd*A rise in the logs (LOG12906: 18.1 at 0.4 deg v 14.7 at
   4.5, Mach-dominated), but a long cylinder shows ~10x its tail area side-on
   (heavy: ~112 v ~11 m2), so 10-30 deg may matter -- only holdable with the
   brakes at 30 or in. Step 1: add a drag-vs-aoa probe beside
   `Environment.refresh_holdable`'s torque probe (same rotated-attitude
   machinery), 0-30 deg at brakes 30 and 90, logged with the holdable angle
   per brake setting; fly one batch. Step 2, if aoa buys drag: brakes at
   30 for authority, aoa for speed with the lift rolled to cancel, a lift
   offset for range, brakes 90 only where the curve says. Whatever the
   coast flies, `ascent/budget.landing_dv` models the landing from the
   terminal velocity -- keep it in step.

1. **The +28 m along-track at the burn.** It comes with the trim (trim arms
   +20..+28 v -7..+1 without, three batches) and grows after the late brakes
   open fully (LOG12746: +24 at the switch, +73 eight seconds later), where
   no drag is left to add. Candidates: the Vernors' push has an axial
   component (axis ~ velocity in the coast), or the open valve shifts the
   held attitude. Log the along-track change across each push and after
   the switch, trim on v off; then cancel on `forward` or stop late
   re-pushes.
2. The leg lead: the time-to-go is the coast-speed fall, so the legs are
   down ~13.5 s early. A burn-aware estimate that is right early in the
   burn (the powered prediction reads 4 s at 9.7 actual) -- or measure the
   burn's duration per vehicle. Earlier legs measured worse (factor 2.0).
3. Pad debris: the launch clamps stay on the pad (live too). Ask the user
   before recovering vessels; an aim point chosen clear of them is the
   alternative.
4. Older: qs_hot sideways arrival, a legged single-engine save, separation
   breakups (LOG12500 again: 0.66 t at 2 s), the coast's roll drift.

**Traps paid:** a fallen booster's reported miss is where the wreck lies
(+25-35 m): use the first TOUCHDOWN line's `pad=`. Summaries that drop "no
exit line" rows drop every fallen booster. Every new `cfg.X` must exist --
grep the diff; a bare-`Autoland` test runs the method. ksp5 returned
nothing for all of trim3 (12 flights lost silently): check every instance
has rows. Swap reached ~20 GB per 40-min batch: restart between batches.
Readers (scratchpad, easy to rewrite): per-arm contact distance from the
first TOUCHDOWN `pad=`, stood from `max=`, gear-to-contact seconds,
burn-entry mass; KSP.log CollisionSpy matched to flights by wall clock.

## 2026-10-15 night: lowThrust RCS/gimbal on rails; the attitude model leaves other flights alone

Defaults fingerprint `8deb8110` (lowThrust, C# `Config`). Committed and
pushed to origin/main. Farm stopped (swap 0.85 GB after), sleep inhibitor
released. How-it-works PDF rewritten (new section 3a). History:
`docs/lowThrust/journal.md`, "Session, 2026-10-15 night". `build.sh`
installed the dll in the farm and the **live game** (restart KSP to load it).

The user asked:
- **Does the mod work like Persistent Rotation?** In part: it integrates
  rotation on rails, but only to steer its own burns (answered).
- **RCS and gimbal steering**: done. `RCS_MODE` (assist default / always /
  off), `RCS_RATE_DEG` 2. Assist uses wheels + gimbal, RCS only where they
  give nothing or a hold exceeds them; respects the RCS action group; pure
  torque. Measured on `qs_shuttle2_lt*` (new saves, Kerbin 200x150 i3 from
  86.7): wheels present -> 0 kg RCS; always 187 kg; no wheel 114 kg (holds
  during burns all on the gimbal); every arm arrives, dv 207.8-208.4.
- **"Make sure persistent rotation can't change my orbit"** -- the live
  shuttle "plunged" (spaceplane LOG10208). Findings: orbit was 77.6 x 80 at
  22:37 (2.3 m/s above the air); the autopilot planned a 31.6 m/s deorbit,
  `warp_to` 739 s, was Ctrl-C'd 2 s into the warp, and KSP.log ends there.
  No log shows an orbit change. But the mod *did* rotate the shuttle through
  that warp: `Attitude.Step` ran on every active vessel. Now: it turns a
  vessel only while a mission flies it; rigid hand-back (w x r, momentum
  unchanged); every rotation on rails restores the elements if they moved
  (`orbit restores`: 0 on all flights). Persistent Thrust is enabled live on
  the shuttle's Rhino and *would* burn on rails with throttle up during a
  warp -- told the user; not evidenced in this flight.

Bugs found and fixed this session (in the journal): measuring stopped with
the gate (a mission starting on rails never turned); RCS costed at full
nozzle thrust; **a new command read as the target turning** (spun the vessel
to 30 deg/s and back: 480 -> 114 kg; the wheels wasted EC the same way).

Regressions with the final build: Kerbin 150x150 99.6 m/s (LOG10232),
Kerbin 400x150 i3 361.4 (LOG10233) -- same as 10-14.

Open, in order:
1. **Mun route stalls in WAIT in game** (46 days, LOG10220/10225/10226; the
   pre-session build f2841c5 stalls identically; the stand-in arrives). The
   last good in-game Mun flight (LOG10196) predates 7111c0e (WAIT's search
   spread over physics steps). Fly 7111c0e^ against 7111c0e on the farm (copy
   a worktree build into one clone, as this session did), then read the
   spread search / `SearchFor`.
2. Fly multi-pass aerobraking on the farm (Duna 200, Duna 80 i0) -- still
   unflown in game since 10-15's changes; ESCAPE's Lambert burn in game.
3. A pass still flown on rails (Kerbin pass 5): who sets the rate.
4. Kerbin aerobraking heat-bound by RTGs; Laythe stand-in time limit; weak
   departure caught by the Mun (LOG10202).

Traps paid: a gate on the attitude model must not gate *measuring* (a
mission can start on rails); an angle difference between two *commands* is
not a rate -- restart rate estimates on a command change; `ltfly` batches:
an instance whose kRPC refuses (ksp5 once) loses its arm silently -- check
every arm has a LOG; KSP's `Orbit` clashes with `LowThrust.Orbit` in the
plugin (`global::Orbit`).

## 2026-10-12 evening: low alpha flown and refuted; the live tumble not reproducible (fingerprint `01de6e69`)

Defaults unchanged all session (every arm was a flag; all deleted).
Committed and pushed to origin/main.  Farm stopped, sleep inhibitor
released.  How-it-works PDF current (the only default change is logging).
Full write-up: `docs/spaceplane/journal.md`, "Session, 2026-10-12 evening".

### Low alpha -- what was found (the user asked for it in the handoff)

The user asked why every plan flies 20-40 deg and whether a lower-alpha
trajectory would be better, thermally checked.

- **Why it is high**: `solve_glide` uses alpha as the primary range control
  (bank is the residual, floored at 30), the deorbit plans inside the box
  (`SOLVE_ALPHA_MIN_DEG` 20, bank 70) to (`ALPHA_MAX_DEG` 32, bank 30), and
  the glide solves in 20-40.  Every plan is high-alpha by construction.
- **Thermal is not the limit.**  Heating rate H = sqrt(rho) v^3 / 1e6 (KSP's
  high-Mach convective coefficient, from Physics.cfg).  The hottest part is
  always the RV-105 RCS block; at v >= 1600 m/s it plateaus at **0.86-0.88
  of its skin limit over H 128-512**, peaks 0.70-0.80 on i90/rigoff and
  0.85-0.88 on e500i45 (H up to 611).  The plateau is not the shock-temp
  ceiling (21 v^0.75 ~ 7000 K), so it is not known past H ~610.  A point
  mass on the logged aero table (calibrated: 220 v 243 flown) gives peak H
  220/257/298/364 at 30/20/15/10 deg, bank 30 -- circular orbits fit inside
  the flown envelope even at 10; e500i45 would exceed it.
- **The shuttle cannot fly low alpha hypersonically by command alone.**
  lowa-1012 (LOG10081-10104; plan 10-20 deg via `SOLVE_ALPHA_MIN_DEG=10` and
  a deorbit band top of 20): **3/12 on the runway v 11/12**, misses to
  -165 km.  At Mach 4-5 it was commanded 16-18 and flew ~22: **its
  hypersonic trim point**.  Fighting the trim leaked into yaw: slip > 10 deg
  on 22% of Mach-4 ticks v 9% on the defaults (LOG10084: 9 commanded, 34
  flown, 28 deg of slip at Mach 3.8).  Heat never bound (skin 0.87 v 0.85).
  The plan also stretched entries to ~2080 km (v ~1500), the shallow regime
  the propagator over-predicts.  Below Mach 2 low alpha is held fine.
- **What would make it work -- move the trim point, then re-plan:**
  1. CoM forward by pumping the nose LFO (`ResourceTransfer`, already used
     as a cone trim) and/or a standing glide trim on the canards/elevons
     (like `CANARD_TRIM` in the cone).
  2. Measure the trimmed alpha by Mach with the broadside probe
     (`BROADSIDE_PROBE_DEG`) before and after.
  3. Only if the trim reaches 15-20 at Mach 4-6, re-fly a lower band --
     and keep the deorbit window out of the >1500 km shallow entries.
  Expected upsides if trimmed: bank changes become body roll (yaw share
  sin(alpha) 57% at 35 deg v 26% at 15), fins out of the wake, brakes
  more effective; downside: higher L/D amplifies model error, more energy
  to dump in the cone, q 2-3x (the low arm's glides reached 12-30 kPa and
  survived).  The code for the band/thermal limits was deleted
  (`git show e145a51`); `deorbit heating` stays in the log.

### The live tumble (LOG10020) -- dropped

The user: "probably a freak accident."  Sideslip -3 -> -24 deg at Mach 3.7,
roll input pinned at -1, the roll damper slowing roll and yaw to 24 s, then a
rolling tumble.  **Not reproducible on the farm**: 72 default flights
(tumble-1012, low70-1012) including 16 from the live orbit
(`qs_shuttle2_i90low_rigoff`, new save: 70.23 km circular) -- glide slip p-p
<= 11 on every one.  Two fixes flown and deleted: `ATTITUDE_YAW_RCS_ARMED`
(17/24 v 21/24, rigoff arrivals +3.4 km long) and `ROLL_DAMPER_SLIP_MAX_DEG`
(never cleanly measured).  The live install differs from the farm by
mods, notably **ksp-advanced-flybywire with an SDL controller enabled**
(pitch/roll/yaw/throttle bound; its input is summed into `yrin=`), KJR,
MechJeb, BetterTimeWarp.  If a live flight departs again, ask whether a
stick was connected before re-opening this.

### Added (default, logging only)

- `nz=` load factor on every telemetry line; a `load factor` event at each
  new peak (q, Mach, alpha).  Defaults' glides peak at 1.4-2.4 g.
- `deorbit heating: alpha A peak H, ...` at deorbit commit: each window
  corner's predicted peak heating rate (`trajectory.peak_heating`).

### Where each orbit stands (defaults, this session's farm batches)

tumble-1012 + lowa-1012 defaults: rigoff 12/12, i90 7/8, i90low 3/4 (+ 11/12
in low70-1012, both arms), e500i45 10/12.  Misses: long splashes
(+6.3, +12.7 km), rollout breakups (19-24 parts left), one -2.3 km.

## Next, in order

1. **Landing accuracy on the default band** (the goal: >= 95% stopped,
   0 lost, all orbits): the remaining misses are long overruns after the
   cone (surplus) and rollout breakups.  The handoff of 2026-10-11 item 1
   still applies: measure the split brake by Mach (`splitprobe.py --glide`),
   `HAC_SPLIT_FACTOR` over-credits the upper cone.
2. Low alpha, if wanted: the trim-point measurement above, first.
3. A load-factor guard / destructive probe (the user asked about runtime q
   tolerance): the `load factor` events build the envelope from every log;
   a guard on commanded alpha/bank and a farm-only breaking probe are the
   next steps (not built).
4. `HAC_SHORT_POLAR` (off, unflown) -- re-fly on the current default or
   delete.  Older: `ABORT_NORTH`/`GROUND_SPOILER_ON_NOSE` farm measurement.

## Traps paid this session

- **A farm that won't reproduce a live failure: diff the GameData first.**
  The live install runs ~50 mods the farm does not, one of which feeds a
  joystick into the control state.
- **Lowering the commanded alpha below the airframe's trim is not flying
  lower alpha** -- it is a standing moment the attitude loop holds badly,
  and on this shuttle it couples into yaw.  Move the trim, not the command.
- Batch round length depends on the arm: the low-alpha cycles took 27 min
  v 18 -- estimate from the slowest arm.
- `multirot` arm numbers rotate per cycle: join results to each log's
  `config:` line, never to the `armN` label.
- A source-text test (`TestTheWindowClockIsLoadBearing`) bans the string
  `index == 0` in `deorbit_window`.

## Traps from before (still true)

- Read the save name off each log before comparing.
- A `&&` chain ending in `| tail` commits on a failing test -- check the
  suite's own exit status.
- landsum drops some logs; for outcome counts use the harness lines in
  `logs/rot-*.txt` joined to each log's `config:` line.
- kspSim has no `qs_shuttle2_rigoff` model; its subsonic lift is wrong (gap
  8) -- don't screen cone energy there.
- Six fresh instances put 10-17 GB into zram at boot; 4 cycles reach ~22.
- Kill sims with `pkill -f "kspSim[.]run"`.
- `warp_to` is blocking: the panel does not answer during it.

A global SessionStart hook (`~/.claude/hooks/pushOnStart.sh`) pushes
unpushed commits at session start; read its `pushOnStart:` message.
