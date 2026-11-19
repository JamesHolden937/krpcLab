# boosterland HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 night (4): boosterland -- the coast's overswing is kRPC pumping the air's pendulum; COAST_RATE_CAP (default, unflown)

State: code + config change committed (COAST_RATE_CAP=True, new default fingerprint), **unflown** on farm and live.
boosterland offline tests 206 OK. Farm down. The user asked: "that save [qs_fin2live1118] overswings, oscillates,
then lands a kilometer short. fix it."

**Diagnosis (live LOG17979, farm LOG17986-17990, all the same):** the coast starts with the nose 137-152 deg off
retrograde (hold=). kRPC's stopping profile `speed = sqrt(2 e alpha)` (~/krpc AttitudeController.cs ~2395) counts only
the control torque (wheels+RCS 2.05 MN m, surfaces 0.2-0.7). The air's restoring moment is 2-3x that at 45-90 deg
(moment/q 870-1470 m^3 x q ~4.5 kPa, `diag hold`). So in every fall toward retrograde kRPC pushes *with* the air,
and the swing is pumped back up: hold 151 -> 56 -> 45 -> 100 -> 67 -> 24 -> 74 -> 36 (LOG17979), sat=1.00, w=30-47
deg/s, 20-30 s to settle. The broadside drag over those 15-25 s walks the miss from +90 to -1083 (live) / -1179
(LOG17986) short. The steering then runs at its holdable limit (sw=v1.00, aoa=can) and cannot win it back: live
landed 1411 m short, matching the coast-end prediction (-1417). The live jump 39 -> -1047 at 770985 is still
unexplained (only event: coast nose lag 0.08 -> 0.62 s, which only stretches the unwind 0.24 -> 1.9 s, worth ~100-200 m).

**Fix (COAST_RATE_CAP, autoland.coast_rate_cap/set_rate_cap):** in COAST, kRPC's `max_angular_velocity` (pitch,
yaw) = sqrt(2 theta_on alpha). theta_on is the valve's measured RCS_ERROR_ON_DEG, and alpha is
(env._hold_fixed + env._hold_surface q) / min(moi x, z). Whenever the air drives the nose faster than that, kRPC
brakes, instead of pushing toward the target. It is re-set only when the value moves 10%, and the saved value is
restored on leaving COAST. Event lines: "coast rate cap X deg/s", "coast rate cap released". My energy estimate:
the first overshoot is still ~40-50 deg (the air beats full braking from 150 deg), then it settles in ~1.5 swings.

**RUNNING: detachjob `cap1118b`** (launched 21:08, ETA ~2125; the first launch, `cap1118`, flew nothing: ksp5 hung mid-load and start.sh hit its 900 s deadline -- start.sh now relaunches a hung instance, c004501), the A/B below.
On its end: `detachjob status cap1118b --tail 150`, read `logs/rot-cap1118.txt`, compare arms as in 1.

**Next, in order:**
1. A/B on the farm (flying as cap1118): `SAVES_LIST=qs_fin2live1118 LAPS=2 brot.sh cap1118 "cap:COAST_RATE_CAP=True,DIAG_STATE=True"
   "nocap:COAST_RATE_CAP=False,DIAG_STATE=True"` (under detachjob; ~2.5 min a lap). Read: time to "coast settled",
   peak hold= after the first crossing, coast peak_miss, pad distance, coast spend (budgetcheck).
   Also qs_finbooster2 (the standing target).
2. If it settles but the first swing is still the bulk: start braking before the fall gathers speed. The cap
   cannot help while the nose hangs near prograde.
3. Then the old list: the steered-prediction steps (section (3) below), the boostback flip.
4. The user may fly it live first: the default is on.

## 2026-11-18 night (3): boosterland -- live1118b: the farm reproduces coast miss swings; the steered prediction credits lift it drops a tick later

State: defaults b3480ed9 unchanged. Code 585b5b1 (coast settle dwell + brake measurement gated on it) **flown** in
live1118b (5 flights, defaults + DIAG_STATE). Farm down. Commits this session: 505290a (batch-end notify-send),
ecd19b3 (Stop hook), this handoff.

**live1118b** (`logs/rot-live1118b.txt`, LOG17986-17990, qs_fin2live1118 = live LOG17979's separation, booster active):
all landed, 312/361/491/539/595 m from the pad, 0.3-2.1 m/s, mostly east (+361..+578). bb_exit miss 38-135 m, so the
error grows in the coast. Coast peak_miss 648-1179 m. Live LOG17979 for comparison: 1411 m short.

1. **The coast miss swings on the farm too (not live-only).** LOG17986 (M rising 1.46 -> 1.8 through the coast):
   logged miss 557 -> 888 -> 1179 -> **20** (770971, aoa steering engages, sw=v0.67) -> 131 -> 15 -> 96 -> **838**
   (770979). `replay.py logs/LOG17986` (unsteered re-propagation, its own curve) is smooth through those ticks:
   318, 667, 967, 951, 769, 776, 927, 841. `--curve-from -1` gives the same shape, so the drag curve's re-sweep
   (770969.80) is innocent. **So the steered prediction claimed ~900 m of lift correction for four ticks and then
   gave it up.** The drop at 770977.6 is "aoa tracking: commanded 7.0, held 3.0 -> lift scaled by 0.19":
   `lift_slope_at` scales by tracking squared, so the lift credit stepped by 27x in one event.
   - Two defects: (a) the steering commanded lift the vehicle doesn't make (held 3 deg of 7) and the prediction
     believed it until one tracking measurement; (b) that measurement is a single-event step, so the prediction
     steps with it.
2. **Live LOG17979's jump (770983 -> 770985, M1.48 -> 1.51, miss 39 -> 1047) is a different trigger:** trk=1.00
   throughout, no tracking event, no first-probe event (bins at M1.49/M1.50 were probed at 770955/770959). At
   M1.48 the steer claimed 39 m with aoa 5.5. One tick later even 13.6 leaves -1046 (can 13.6 -> 12.3, cla
   -379 -> -473). The likely common cause is the same: the steered prediction's lift credit is wrong, and something
   that steps at the bin edge revealed it. `holdable_aoa`/`torque_limit_aoa`/`_nearest` take the **nearest filled
   bin** (stepped), and `_hold[index]`/`_lift_peak[index]` are **replaced wholesale** by each probe (no EMA). Those
   are the stepped reads at a bin edge. Lift slope and drag interpolate.
3. **Can't settle it offline yet:** DIAG_STATE logs the state, drag curve and atmosphere, so `replay.py` re-flies
   only the **unsteered** prediction. Needed: DIAG_STATE also logs the steering tables (`_lift` bins,
   `aoa_tracking`, `_hold` curves, `_lift_peak`, `_hold_fixed/_hold_surface`), and `replay.py` re-flies the steered
   prediction (`guidance` solve) from them. Then re-fly LOG17986 770969-770981 and find which input moves the
   steered miss. Fix at the source (principles: prediction error first, no bias/margin).
4. **Coast settle (585b5b1) flew:** "coast settled: err 0.9 deg after 30.0s" (LOG17986). The coast started at
   err 94 deg, sat=1.00, and took 30 s to settle. The steering's 900 m swing happened during that unsettled stretch,
   so the commanded aoa wasn't being held. Whether the steer should command at all before "settled" is a question
   for the fix in 3.
5. **Boostback flip** (unchanged): finishes in 1-4 s against the law's planned 9-12 s; `flip_authority` reads RCS
   before the valve opens. Proposed: read it after the valve opens, and end the flip on rate as well as angle.

**Next, in order:** 3 (log steering tables in DIAG_STATE, extend replay.py to the steered solve, replay LOG17986)
-> fix 1/2 at the source -> fly qs_fin2live1118 again (`SAVES_LIST=qs_fin2live1118 LAPS=1 brot.sh <name>
"diag:DIAG_STATE=True"`, ~2.5 min) -> 5.

**Traps:** `ttl=` is logged rounded to whole seconds, so ut+ttl "steps" of 1-2 s are rounding, not events (the
earlier "ut+ttl -2.0 s" in both live logs is just that). A live FMRS save has the main mission active: set
`activeVessel` to the booster before the farm flies it.

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
