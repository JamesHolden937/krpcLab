# boosterland HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 late afternoon: boosterland -- constants audit; legs on fin2; the coast's RCS limit cycle traced to the deadband

State: defaults fingerprint changed (LEG_CLEARANCE_MARGIN_M and TOUCHDOWN_BACKSTOP_SPEED deleted; read it off the next
log's `config:` line). main committed through c9fa8d3 + docs + this handoff. **Branch `band` (worktree `.wt-band`,
commit 85ce0c3): the coast deadband fix, WIP, untested, unflown -- not merged.** Farm stopped (the user stopped the
session mid-batch, 16:20). howItWorks.pdf (boosterland) still stale. `docs/krpcFeatureRequests.md`,
`tools/instancebench.py`, `.wt-pytune`, `.claude/worktrees/part-prune-*` are not ours.

**Done:**
1. **Audit for fitted constants** (the user asked first thing): `config.py` and every numeric literal in
   autoland/guidance/trajectory/environment/telemetry/proximity and the common modules they use. Nothing fitted to one
   craft shapes the nominal flight. Two leftovers that acted on every landing are replaced (c9fa8d3):
   - `LEG_CLEARANCE_MARGIN_M` (0.5 m padding under the bounding box) -> `Telemetry.set_feet`: the legs' lowest
     `Leg.contact_position` (read in `read_landing_limits`), clearance = max(box tail, feet).
   - `TOUCHDOWN_BACKSTOP_SPEED` (15 m/s) -> the weakest touching part's crash tolerance (`touchdown_limit[0]`, else
     KSP's default 9).
   Left as allowed (guards/numerics): the 20 m/s floor under aero probes (open, low value), roll gate +5/-1 deg,
   `GEAR_DEPLOY_ALT_M` backstop, `SEPARATION_MAX_COAST_S`, `TOUCHDOWN_ALT_M` 0.2. `docs/boosterland/constants.md`
   updated; boosterland/CLAUDE.md's attitude paragraph corrected (auto-tune on, RollTune deleted in 67ad1f3).
2. **`saves/qs_finbooster2_legs`** (the user: fin2 has no legs, attach some to test the leg code): six LT-2s copied from
   qs_heavy onto EnginePlate4, r 2.70, y -31.77; `splicebooster.py --legs-from` (saves/README.md has the command).
3. **legs-1118** (`logs/rot-legs-1118.txt`, LOG17837-17854, 18 of 24 flown, one default arm with DIAG_BURN, both
   saves, stopped by the user -- partial, not read beyond this): fin2 9 flights, legs 9; legs landed 2 (4 m/4.0 m/s,
   21 m/3.4), destroyed 3; most end "flying" = ran dry (item 2).
   **Leg clearance looks wrong:** on the legged save it reads 19.42 at connect (whole stack), 15.6-15.7 in the coast
   (box), then **13.4-13.8 at gear down ("feet")** -- 2 m *less* than the box although the code takes the max. So at
   that tick the box itself read lower (legs deploying in the box?) or the vessel frame is not CoM-centred for
   contact_position. Read LOG17839/17844/17846 around 3720 s and find out before trusting it. With the hovering
   suspicion: a too-small clearance means the burn thinks it is 2 m higher than it is -> hits harder.

**Open, in order:**
0. **The coast's RCS limit cycle (the user's priority) -- cause found, fix unflown.** Not the attitude loop: the
   commanded aoa toggles ~10 deg <-> 0 every ~6 s (LOG17812 3640-3688: at 0 the miss is -255 m, at 10 deg +110). The
   deadband `env.miss_band` is the jitter of `self.prediction`, which in the coast is the *steered* prediction; every
   flip of the command jumps it ~365 m, the band swells past the 255 m miss, `solve_steer` returns zero ("already on
   the pad"), the next tick steers again. The nose chases each 10 deg step saturated with the valve open = the
   0.65-5.2 t. Branch `band`: `solve_steer` stores the zero-angle prediction as `env.unsteered`; the jitter is fed
   from it in the coast; the coast line logs `band=`. Next: merge `band` into main, run the suite (farm down),
   restart the farm, fly ~12 a side on qs_finbooster2 (+ the legs save), read `band=` against |miss| and the coast's
   spend (`ascent/tools/budgetcheck.py`, "coast spent"). Also check the solve then settles at ~7 deg rather than
   toggling: if it still bang-bangs, the quadratic per_square model vs the verified() halving is next.
1. Leg clearance at gear down (item 3 above).
2. fin2 runs dry (carried; the user said re-fly the ascent later, not now).
3. Carried from the morning section: E bias ~-36 m, ratio outliers, howItWorks.pdf, DIAG_DRAG.
4. Older carried items (the 11-14 boostback oscillation / roll latch / gimbalTorqueFix, the 11-13 landing-burn
   margins, PITCH_YAW_TUNE): journal.md, "Former handoff sections (moved 2026-11-18)" at its top.

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
