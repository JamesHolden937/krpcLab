# boosterland HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 night (2): boosterland -- live LOG17979: miss jump is live-only (M1.5 coast); coast settle gate fixed (unflown)

State: defaults b3480ed9 unchanged. Code 585b5b1 (coast settle dwell + brake measurement gated on it), **not flown**.
boosterland offline tests 206 OK. Farm down. mirror-1118 was read in f0a4403 (section below).

**LOG17979** (live, FMRS_save_1 'Heavy Shuttle Probe', 274 t, sep 16.5 km M2.75): landed 5.5 m/s 1411 m short, then
tipped (tilt 159) and broke up after touchdown (rcs on, hold ~100 deg).

1. **Miss jump (top priority, untraced).** Predicted miss 39 m -> 1047 m in one tick (983 -> 985 s, 11.5 km); ut+ttl
   stepped 2.0 s earlier, and the *post*-jump prediction came true (touchdown 1034.0 v 1034.0 predicted). So the
   pre-jump prediction was wrong. **LOG17952 (live) has the same: 203 -> 1370 m, ut+ttl -2 s, at 7.0 km.** None of 48
   farm flights (nsep/mirror-1118) jumps > 400 m below 14 km. **Both live jumps happen as Mach crosses 1.50**
   (1.48 -> 1.51; 1.52 -> 1.50); farm coasts below 14 km run at M~2.2, so the farm saves never fly this regime. 1.5 is a
   0.25-Mach bin edge (`DRAG_CURVE_BINS` 24): look for a nearest-bin (stepped) lookup in what the coast prediction
   reads at the current Mach (hold/`can`, lift slope, the burn's aero tables, `BURN_LIFT`). Ruled out: nose lag
   (burn only, span change ~1.6 s), cda (smooth), the rcs trim (cross-track only, its physics can't move 1 km in 2 s).
   DIAG_STATE was off and doesn't log steering state, so no offline replay.
2. **Reproduce on the farm:** `saves/qs_fin2live1118` = the live `FMRS/FMRS_save_1.sfs` (LOG17979's separation), with
   `activeVessel = 10` (the booster). FMRS saves keep the main mission active and live `fmrs.py` switches to the
   booster; the farm harness flies whatever is active. The first batch (live1118, LOG17980-17985: 'Heavy Shuttle',
   259 t, destroyed 119 km E) flew the **orbiter**. Ignore it. **RUNNING: detachjob `live1118b`** (started 20:00, ETA
   ~20:04): `SAVES_LIST=qs_fin2live1118 LAPS=1 brot.sh live1118b "diag:DIAG_STATE=True"` -> `logs/rot-live1118b.txt`,
   6 flights, defaults + DIAG_STATE. Read:
   - Check the first SEPARATION line reads ~274 t, 16.5 km, M2.75, pad ~20.3 km, like LOG17979. If pad is off by
     km, shift the Kerbin LANs as `qs_finbooster` did.
   - Does the coast miss jump as Mach crosses 1.50? Use the awk scan of ut+ttl steps, which found -2.0 s in both live
     logs.
   - If yes: `boosterland/tools/replay.py` on the tick before and after the jump, to find which table steps at the
     Mach 1.5 bin edge, and fix it at its source.
   - If no: compare the coast's env lines (`holdable`, `torque limit`, `lift peak`) with LOG17979's. The farm and
     live then differ in what the probes read.
   Then add the save to saves/README.md.
3. **Coast oscillation (fixed, unflown, 585b5b1):** "coast settled" latched on the first tick inside RCS_ERROR_ON_DEG
   (17.5 deg while turning at 47 deg/s). Then the rcs trim pushed and the late-brake measurement held the fins neutral
   (the coast's attitude authority) mid-swing: 25 -> 75 deg after the brakes opened, and open/closed read 0.83 (bogus).
   Now settled = inside for RCS_SETTLE_S (SETTLE_LAGS x measured lag) without leaving, and the brake measurement waits
   for it. Whether the loop itself also diverges (sat=1.00 for 30 s) still has to be measured.
4. **Boostback flip:** every flight (farm too) finishes the flip in 1-4 s against the law's planned 9-12 s
   (`flip_authority` reads 0.056 rad/s^2 rcs+wheels; RCS reads 0 while the valve is shut, so it is probably read
   before the valve opens). It arrives at 46-54 deg/s. The farm settles in one tick; live overshoots (err 26 deg, 4 s).
   "flip done" is angle-only (`flip_done_deg` 25.8 deg). GimbalTorqueFix **is** installed live now (HANDOFF item 7 is
   stale). Proposed: read flip authority after the valve opens, and end the flip on the rate as well as the angle.

**Next, in order:** 1 (reproduce, then find the M1.5 step) -> fly 585b5b1 on the farm (nsep settings,
`LOOP_PACING_GAME_TIME=True`, so it compares with nsep-1118) -> 4 -> the older items below.

**Traps:** a live FMRS save has the main mission active: set `activeVessel` to the booster before the farm flies it. The context-limit
hook now fires only right after a batch launch (claudeHooks.py `BATCH_LAUNCH`).

## 2026-11-18 late night: boosterland -- the live "10x less torque" was a misreading; the real fault was the hold probe reading the fins' deflection (fixed, unflown)

State: defaults b3480ed9 unchanged (no config change); code change in `environment.refresh_holdable`, committed, **not flown**.
boosterland offline tests 206 OK. Farm down.

**LOG17952** (live, FMRS_save_1 of 'Heavy Shuttle'): 1200 m short, broke up; below 9 km `can` 16-18 deg, held 7-12.

**The torque is the same on farm and live.** Fixed (wheels+RCS): flip law 0.056 v 0.055 rad/s^2 (x inertia ~1.9 MN m).
Surface/q ~140 m^3 on both at 18 km (q from kspSim/atmosphere.py). budget/q = 0.7(fixed/q + surf/q) is dominated by
fixed/q, so it scales as 1/q: farm's first probe was at 29 km (q ~550 Pa) -> 2734, live's at 18 km (q ~6500) -> 304.
Farm at 18 km read 230, at 12 km 116 -- same as live. The live booster just reached the coast handback lower
(18 km v 29 km): a different separation, not a different install. Old next step 1 (fly FMRS save on farm) is moot.

**Real fault (old item 2, `can` bouncing 5.6-26 deg):** `refresh_holdable`'s torque and lift probes through
`simulate_aerodynamic_torque_at/force_at` include the fins' *current* deflection (saturated in the coast, sat=1.00):
the 11 deg moment/q read 159 then 243 eight seconds apart at the same Mach. Same trap fixed in probe_lift_slope
(4a09ace). **Fix:** each angle probed at +aoa and -aoa; restoring moment = half-sum along each side's own axis, lift =
half-difference of the side force, drag = mean. Doubles the hold probe's calls (16 -> 32 per 2 s refresh).

**How LOG17952 was lost (re-read 2026-11-18 19:05):** boostback exit 35 m; the flip took the prediction to 2.8 km
short, steering recovered it to 45 m by 11 km. Below 9 km cmd 17-18 deg = `can`, but held 9-12, pitch input pinned
(`in=+0.51/+1.00`, sat=1.00), trk 0.90 -> 0.57; at 7.7 -> 7.0 km the miss jumped 203 -> 1370 m and stayed. So: `can`
overstated what the fins could hold (the probe read saturated fins) -> asked for lift it could not make -> short.

**mirror-1118 READ (2026-11-18 night): not comparable with nsep-1118, the arms differ in pacing.** `logs/rot-mirror-1118.txt`,
LOG17953-17976, 24 flights on fb61b3b, pure defaults (fingerprint a68b023d). nsep-1118 flew `--set
LOOP_PACING_GAME_TIME=True AIM_BIAS_EAST_M=0.0` (fingerprint b3480ed9). Miss median ~20 m (nsep ~6), 14/24 within 36 m
(nsep 20/24), worst 103 m, 2 destroyed (LOG17953, LOG17968). The loss is upstream of the coast: 9/24 boostbacks exit
early (ut ~17030, miss 61-212 m, long +13..+196) against 3/24 in nsep, and those 9 are the long landings (E +34..+113).
The hold probe is not used in boostback, so the mirror fix cannot be blamed for these. The likely cause is wall-clock
pacing (the default, and what the user flies live). The fingerprint moved too, so check why first (`git log` on configs
after b3480ed9).

**Next, in order:**
1. Re-fly mirror with nsep's settings (`--set LOOP_PACING_GAME_TIME=True`) to isolate the probe fix. Separately, read
   why boostback exits early under wall-clock pacing (LOG17970, LOG17973 v LOG17959): this matters live.
   Test LOG17952's state: would the mirrored probe have given `can` 9-12 deg rather than 17-18?
2. Old items from the evening section below (outliers LOG17937/17942, coast spend split, howItWorks.pdf, DIAG_DRAG).

**Trap:** never compare budget/q across probes at different q -- the fixed term divides by q. Compare torque (N m).

## 2026-11-18 evening: boosterland -- coast steering fixed at its source (lift probe + solve root); new fin2 separation; median miss ~6 m

State: **defaults fingerprint b3480ed9.** main committed through 0206e9d + this handoff (not pushed). Farm up, idle.
howItWorks.pdf (boosterland) stale. `docs/krpcFeatureRequests.md`, `tools/instancebench.py`, `.wt-pytune`,
`.claude/worktrees/part-prune-*` are not ours. `.wt-band` worktree: merged, can be removed.

**Result: nsep-1118** (`logs/rot-nsep-1118.txt`, LOG17927-17950, 24 flights, defaults, 12 qs_finbooster2 + 12 _legs, the
new save): miss median ~6 m (fin2 ~5, legs ~8), 20 of 24 within 36 m, outliers 135/187 m (LOG17937, LOG17942 -- read
why next), 18 of 24 "landed" mostly at 1-4 m/s, none ran dry. Coast RCS spend median 1.2 t (0.73-2.04) against ~2.5
(0.7-4.8) in band-1118/fp-1118.

**What was wrong (the coast's 0.7-5 t RCS limit cycle), in order found:**
1. `band` (merged, 85ce0c3 + ca2d6af): the deadband now measures the *unsteered* prediction's jitter
   (`env.unsteered`); band fell to 1-3 m but the toggle stayed (band-1118, 18 of 24 flown -- killed by my farmlaunch
   wrapping, see trap). Kept: it is the right series for the band.
2. `sw=` on the coast line (which return `solve_steer` took). sw-1118 (6): every kept command was the 0.5x retry,
   still +80..+234 m past a -205 m miss; other ticks `worse` -> 8 <-> 0 deg. Fix aed89e1: `verified()` finds an
   overshooting command's root by false position (`STEER_ROOT_STEPS` 2, numerics). fp-1118 (24): 814 accepted v 26
   worse -- but spend unchanged, because:
3. **The lift probe read the fins' deflection** (4a09ace): `probe_lift_slope` tilts the vessel to the commanded angle
   (>= 1 deg) and divides the side force by q a|a|; the force includes the fins as deflected now (saturated, flipping);
   at ~1 deg it swamped the lift: `cla` -3800 <-> -225 tick to tick (LOG17905), command 0.8 <-> 7 deg. Now probed at
   +aoa and -aoa, differenced: the deflection cancels. Flown in asc-1118 and nsep-1118 (the results above).
4. Leg clearance "2 m low at gear down" (old item): **not a bug** -- the box clearance shrinks with propellant burned
   (CoM moves to the engines): coast -0.2..-0.4 m for 1-5 t, burn ~-1.8 m for ~25 t; `legs=-1.4` at contact = 1.19 m
   leg travel compressed. Stale only from ignition to gear down (re-read 5 s before contact). Closed.

**Open, in order:**
1. Read the two outliers (LOG17937 135 m E+142, LOG17942 187 m E+192, both long east): logsum --trace.
2. Re-fly the comparison with more flights only if a change is proposed; E bias now: read predvsland on nsep-1118.
3. Coast spend 0.7-2 t remains: is it the flip (17 s) or the hold? budgetcheck splits only coast/burn.
4. howItWorks.pdf regenerate; `DIAG_DRAG` keep/delete; old items in journal.md.

**Traps paid for:** never wrap brot.sh in `farmlaunch` yourself (`detachjob start N -- ./boosterland/tools/brot.sh`):
FARMLAUNCH set skips farmGuard's `--started-glob`, brot prints nothing, killed at 601 s mid-flight. The offline suite
is refused by a hook while the farm is up: stop.sh, suite, start.sh. A probe through `simulate_aerodynamic_force_at`
includes the control surfaces' *current* deflection -- difference mirrored probes.

## 2026-11-18 morning: boosterland -- back on qs_finbooster2; brake settle read from the game; BURN_LIFT promoted

State: **boosterland defaults fingerprint 080e48b5** (was d8a430b8). Everything committed on main (last: 304dbb5 + this
handoff), pushed. **Farm down.** launcherMod rebuilt (new `SurfaceTravel` procedure), installed in the farm and the live
game. howItWorks.pdf (boosterland) **stale** (not regenerated since 11-17). `tools/instancebench.py`,
`docs/krpcFeatureRequests.md`, `.wt-pytune`, `.claude/worktrees/part-prune-*` are not ours: leave them.
Full measurements: `docs/boosterland/journal.md`, 2026-11-18.

**Fly booster work on `qs_finbooster2` only** (the user, 2026-11-18: finbooster4 was the wrong save; in
boosterland/CLAUDE.md). Not finbooster3/4.

**Done this session:**
1. **The late brakes' open/closed measurement settled mid-travel** (ratio 1.55-3.65 on one craft; low ratios = settled
   0.4-0.7 s into a ~1.7 s opening). Now `brakes_settled` reads the game's own surfaces: `launcherMod`
   `SurfaceTravel` (max |action - deflection| / range, by reflection), travel seen then <= 1e-3; drag stillness is
   only the fallback without the mod (kspSim, tests). fin2trv-1118: open settles 1.5-1.9 s every flight, ratio
   3.29-3.35 on 17/23.
2. **BURN_LIFT promoted** (flag deleted): landed-pred E -36 sd 84 against -106 sd 88 (12 a side, fin2trv-1118).
3. `predvsland.py` splits by brake opening and prints each flight's open/closed ratio.

**Open, in order:**
0. **NEXT PRIORITY (the user, 2026-11-18): the coast's attitude limit cycle burns 0.65-5.2 t of LFO on RCS.**
   Not valve chatter: the valve (a relay on pointing error) opens and *stays* open while the pitch/yaw loop
   oscillates under it -- inputs saturated, flipping sign every few seconds (`in=-0.86/+1.00` <-> `+0.86/-1.00`,
   sat=1.00), err swinging ~2-10 deg and never falling below the valve's off threshold. LOG17812: valve open
   3640-3688 s, mass falling a steady ~60 kg/s (this craft's RCS burns the main LFO), 5.18 t spent in the coast.
   Good case LOG17830: settled in 14 s, RCS off, mass flat (1.37 t). Spread over fin2trv-1118 + fin2rec-1118 (same
   save): 0.65-5.2 t. A loop that will not converge (gain or lag against the RCS's authority) -- fix the controller
   (`rolltune.py` / the coast's pitch-yaw bandwidth, `docs/boosterland/design.md`), not a budget margin. Measure
   the coast's spend with `ascent/tools/budgetcheck.py` (column "coast spent").
1. **fin2 runs dry**: 14/24 ended in the air, tanks empty (38.7 t at the coast over 122.65 t dry; the burn needs more
   than it has). Every miss on fin2 is mixed with a free fall. The user (2026-11-18): when the boostback's budget
   changes, re-fly the ascent for a new separation with the right propellant (boosterland/CLAUDE.md) -- **not done
   yet, on purpose: the user said not now.** (Afternoon: the budget's fall drag fixed, it now says fin2's separation
   is 1.5 t short before the coast's RCS -- see the ascent section.) Next session: re-fly the ascent with the booster's current budget
   (`ascent/CLAUDE.md`, FMRS save -> `boosterland/fmrs.py`), make the new separation the fin2 save, then re-read the
   E bias on flights that land under power.
2. Remaining E bias ~-36 m (short) with BURN_LIFT: not inside its standard error (~24 m)? borderline; re-read once
   flights stop running dry.
3. Six ratio outliers (2.03-3.77) in fin2trv and LOG17811 (no brake measurement at all, 353 m off): read why.
4. Old list: the coast's own lift model probed with fins deflected; the burn's aim (`LANDING_AERO_DIVERT`,
   `LANDING_STAND_UP`, unflown); regenerate howItWorks.pdf; `DIAG_DRAG` -- delete or keep as a tool now BURN_LIFT
   is decided.

**Traps paid for:** still drag is not still surfaces (a brake's drag barely moves near the ends of its travel, and
ticks inside one physics frame read identical drag); the farm's first restart can fail with every instance's nested
X server timing out ("XIO fatal IO error 110" in ksp<n>-boot.log): stop.sh and start again; swap reaches 15 GB after
one 24-flight batch on six instances -- restart before each comparison; the offline suite has 2 kspSim wire-codec
errors (krpc client type code 203), not boosterland's.
