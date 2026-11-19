# boosterland HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 night: boosterland -- live flight LOG17952 missed 1200 m short: the live booster has ~10x less control torque than the farm's

State: defaults b3480ed9 unchanged, no code changed. The farm section below (nsep-1118, ~6 m) is still the farm's state.

**LOG17952** (the user, live, FMRS_save_1 of 'Heavy Shuttle', 276.7 t at separation): boostback exit miss 35 m. The flip
cost ~2.7 km of predicted miss (err 40-100 deg for 25 s at M1.5) and the aoa steering won it back (+44 m at 11 km).
Below ~9 km it commanded 16-18 deg (`can`), held only 9-11 (`hold`); `trk` fell to 0.57 (lift x0.32) and the predicted
miss jumped -203 -> -1370 m at 7.7-7.0 km. The burn started 1294 m off; touchdown 1200 m short at 13 m/s on terrain,
broke up (144 -> 1.6 t). Brief "throttle DIVERGED" at boostback end (user's throttle?) -- not the cause.

**Why live differs:** the control budget. `torque limit` line: farm LOG17950 budget/q 2734 (limit 90 deg); live
budget/q 304 then 203 (limit 23.8, then 9.4 deg). The aero moment/q curves are about the same (22 deg: 341 farm, 289
live). The fins' save settings are identical (6 fins, authorityLimiter 150, deployAngle 22.5, nothing ignored;
FMRS_save_1 vs saves/qs_finbooster2). So it's the game's reported available torque (wheels + RCS + surfaces; the
surface term dominates on the farm), not the craft's settings. Live mods that could change it: ReStock, KSPCommunityFixes,
ModularFlightIntegrator, KJR, AFBW, Kopernicus expansion (live GameData list).

**Next, in order:**
1. Copy the live `~/KSP-Retooled/saves/default/FMRS/FMRS_save_1.sfs` into a farm save and fly it on the farm with
   DIAG_STATE=True (the `diag hold` line splits fixed (wheels+RCS) from surface). Budget ~2700 -> it's the install
   (diff GameData, bisect mods); ~300 -> it's the state (propellant, CoM, Mach, the separation).
2. Separately: `can` (holdable_aoa) read 16-26 deg while the 770892 probe said 9.4 deg and the booster held ~10.
   Find why it bounced between 5.6 and 26 deg in the coast (a prediction error: fix it at its source).

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
