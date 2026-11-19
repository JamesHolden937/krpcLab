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

## 2026-11-14 (+ 11-13 night): boosterland -- the boostback oscillation (gimbal lag, the roll latch, gimbalTorqueFix)

**2026-11-14 later: kRPC main (0.8.0-dev, commit 34c70de) installed on the farm, stockref and the live game,
and its client in both venvs** -- `tools/krpcmain.sh` rebuilds/reinstalls it (bazelisk in ~/.local/bin, source
~/krpc); 0.7.0 backups in `.krpc070/` (gitignored). What changed for us: one port (`connect(port=)`; settings
keys `tcpip_address`/`tcpip_port`, `setports.py` writes them; `--stream-port`/`.stream_port` are ignored), every
`krpc.connect` call rewritten, `krpcbatch` on `Session.call`, `fastclient` off (0.7-only: PyPy coast ticks ~30%
dearer, 0.08 -> 0.105 wall-s), launcherMod rebuilt, KRPC.MechJeb now ships with kRPC. The backport stands down
itself ("kRPC models the gimbal lag itself"); boostback identical to it (Kp*alpha 3.29, nose p90 5.7-6.2).
**0.8 also carries #1191: MoI measured about the CoM, ControlSurface torque fixed, `LimitAngularVelocity` on by
default -- a new plant.** k08-1114 (LOG16667-16690, qs_finbooster2, 12 a arm): pace on median miss 480 m, 7/12
destroyed (the coast's held bandwidth reads 0.6 v 1.9 rad/s and kRPC latches 12/12); pace off 26 m, 5/12, td
median 12 m/s (0.7+backport pace on: 66 m, 7.5 m/s). **`ATTITUDE_LAG_PACE` default now False.** kspSim is
broken until ported to the 0.8 protocol (2 suite errors are its 0.7 codec tests).

**2026-11-14: kRPC's own fix (krpc/krpc#1348, merged 11-14, unreleased 0.8.0) backported into
`KrpcGimbalLag.cs` (b53a1d8)**: exact per-tick lag -dt/ln(1-speed dt), cap 2 zeta w0 + w0 <= 1/lag, PI gains
placed for the lagged plant; the patch refuses itself on a kRPC with `gimbalLag` (delete it then). 0.8.0 itself is
not usable early: hashed procedure ids and ~30 other breaking changes need the new Python client and a launcherMod
check; no milestone/date (0.6->0.7 took 91 days). lag4-1114 (LOG16643-16666, qs_finbooster2, same arms as
lag2-1113): boostback lit nose p90 **5.6-6.1 deg/s on all 24** (old patch: trimodal 3.3/10/21), Kp*alpha 3.29
(was 5.33), pace on = off in the boostback; landings 2/12 destroyed pace-on (= lag2), pace-off misses still
+80..+200 E. Burn nose p90 (pace on) median ~5.5 v ~8. On every farm instance; not live. **Trap paid:** the
default `quicksave` (18-part single-Vector booster) is destroyed 24/24 on the Retooled farm with either patch
(lag3-1114, ctlold-1114) -- fly qs_finbooster2 for booster comparisons until that is looked at.

Journal: `docs/boosterland/journal.md` "2026-11-14" and "2026-11-13 night". The user's live LOG16371: the nose swung 55 deg at
full thrust in the boostback (4.3 km of cross). **Two kRPC defects and one stock one, all now handled:**
- **Gimbal lag (kRPC; the user filed an issue).** Auto-tune assumes instant torque; ModuleGimbal Lerps at
  gimbalResponseSpeed (0.125 s, Vector) -> limit cycle under thrust. `ATTITUDE_LAG_PACE` **on by default**
  (rolltune.RollTune.lag_ttp/lag_bandwidth): every phase, from the measured torque-weighted lag.
  Boostback nose p90 15-29 -> 3-25, burn 16-35 -> 4-13 deg/s; coast-entry cross -68..+54 m (was -189..+397).
- **Zero-authority roll latch (kRPC).** No roll authority in the unpowered flip (the fins' RCS can't roll)
  -> auto-tune roll kp 1e4-1e5 -> kRPC latches roll as flexible -> roll at 1/8 gain after ignition, 133-258
  deg/s spin. `KRPC_CLEAR_LATCH_ON_THRUST` **on by default** (Autoland.clear_krpc_latches: AutoPilot.reset +
  configure_autopilot at ignition when latched). Roll p90 133 -> 2 (fin), 173 -> 17 (heavy).
- **Stock ModuleGimbal.GetPotentialTorque over-credits 1.25-1.5x.** `gimbalTorqueFix/` (Harmony mod, build
  with `gimbalTorqueFix/build.sh [--live]`): torque from stock's own GimbalRotation deflection; probe
  measured/credited 0.67-0.80 stock -> 0.85-1.00. **Installed on every farm instance since latch-1113 (the
  farm's plant changed then); NOT in the live game -- the user decides.** Its `KrpcGimbalLag.cs` caps kRPC's
  DoAutoTuneAxis bandwidth by gimbal lag (any kRPC client); lag-1113 showed it never bound (kN m v N m, fixed
  c56c76e); lag2-1113 (fixed): pace off + patch caps kRPC's bandwidth at 5.33 exactly
  as pace does, boostback equally calm in both arms -- **the patch alone fixes the boostback for any kRPC
  client**. Pace is still needed for the burn: from the coast krpcLab sets kRPC's gains itself (auto-tune off),
  so the patch cannot act there (pace off: burn saturated 1.00, fin misses 201-241 m).
- Tools: `testInstances/actuators.py N SAVE --probe` (credited v measured per axis and source);
  `KRPC_AP_DIAG` now also writes LOG<n>.bb.ap.csv (boostback) and .flip.ap.csv (before a latch reset).
  kRPC source: `~/krpc` (git clone, master).
- Deleted after measuring: ROLL_FREE_BOOSTBACK (null), ROLL_TUNE_BOOSTBACK (worked around the latch, hurt
  pitch/yaw). Fixed: holdable probe reading RCS 0 on the valve's reopening tick.

**Open, in order (everything pending as of 2026-11-14 end):**

*kRPC 0.8 (main) -- consequences of the install:*
1. **The 0.8 default rests on an accident.** With pace off, RollTune.engage still turns auto-tune off at coast
   entry but reads pitch/yaw authority 0 at that tick (`held at kRPC's bandwidth - -`, every k08 pace-off
   flight), so the coast and burn fly kRPC's last auto-tuned pitch/yaw gains, unmanaged. Find why accel reads 0
   (0.8's MoI/torque, #1191, or the tick) and decide what the coast should hold -- this replaces old item 4.
2. **Touchdown on 0.8**: median 12 m/s and 5/12 lost v 7.5 m/s, 2/12 on 0.7+backport. Re-take everything
   derived from kRPC torque/MoI (#1191 moved both): RollTune, the probes' tables, `holdable aoa`, flip law.
3. **Second batch, then delete the pace code**: ATTITUDE_LAG_PACE, lag_ttp/lag_bandwidth/pace/unpace,
   ROLL_LAG_FACTOR, ROLL_TUNE_ENGAGE_DELAY_S, and `gimbalTorqueFix/KrpcGimbalLag.cs` (inert under 0.8). Fly
   k08's design again on a fresh farm, plus qs_finbooster and qs_heavy, before deleting.
4.-9. Moved to the top-level HANDOFF.md, "Cross-pilot open items (kRPC 0.8)".

*Booster, carried:*
10. Why the 18-part `quicksave` booster dies 24/24 on the Retooled farm (lag3/ctlold-1114: parts lost mid-burn,
    ~567 m/s at 5.9 km, either patch). Fly qs_finbooster2 for comparisons until then.
11. Heavy booster's unpowered flip stalls ~120 s -> boostback TIMEOUT, 2-5 km: 4/4 on instance 2, 0 on 5
    (identical GameData and save). Separation roll + no RCS roll authority; find why only ksp2.
12. Flip law reads gimbal authority 0 (gimbal_limit is 0 while locked; cached). Inert today.
13. The landing-burn items of the 2026-11-13 section below (ignition margin, throttle margin, PITCH_YAW_TUNE).
14. Farm memory: six Retooled instances swap 20-30 GB (28.8 GB after k08-1114); consider five.
15. howItWorks.pdf (boosterland) not redone for the latch reset and pace off.

## 2026-11-13: boosterland + ascent -- high-aoa descent refuted and deleted (earlier the same day; still current for the descent, burn and ascent); the fin booster's roll; the ascent's landing budget

**Later the same day (journal "2026-11-13: high-aoa descent", "... the cone"):** item 1b built behind
`HIGH_AOA_DESCENT` (off; defaults fly as before) and flown in 5 batches. **Refuted twice; deleted** (commit after 8aad4ea; code in a0e5f4d), keeping the probe's `lift peak` (polar `aoa:lift/drag`) and `torque limit` log lines.
- Polar (fin boosters): drag/q 7-14 nose-first, 60-85 at 22 deg, 170-375 at 90; lift peaks 45-50 deg, ~0 at 90.
  Torque limit is 90 deg only above ~35 km (q < 400 Pa), 7-15 deg by M2.
- Law 1 (hiaoa1-4, LOG16331-16354, commits 125d60b..e75fdd2): rest broadside, lift on the back branch.
  Commanded 49-70, held 2-42 saturated; burn still lit at 690-720 m/s.
- Law 2 (cone1-1113, LOG16355-16366, commit a0e5f4d, **the code now in the tree**): angle = `holdable_aoa`
  at each propagation step's q; lift direction coned about v at a solved rate/phase (`guidance.solve_high_aoa`,
  `Steer.schedule/cone_rate`); late brakes held while flying. Result vs default, 6 a arm: **ignition speed
  683-747 (median ~701) v 657-754 (~694): no gain; miss median 426 v 44, all long east.** Cause: the cone rate
  is capped by the nose's turn rate (coast flip measured ~9 deg/s, so <= ~0.1 rad/s at 48 deg) and the solve
  pinned it at 0.05-0.09: a minute per revolution, the lift never averages; best miss > 500 m, so the law
  toggled on/off every few s and the coast inherited km-sized errors. At the torque limit there is no torque
  left to move the nose: coning is infeasible on this airframe, not mistuned.
- Deleted: config `HIGH_AOA_*`, the guidance solve, `Steer`'s polar/schedule/cone fields, autoland's high-aoa state and
  brake gate, environment's `drag_nose_first`/`polar_at`, `tests/testHighAoa.py`. Kept: the probe's polar and
  `torque limit` events (`Environment.torque_limit_aoa`, log only).
- **Ascent comparison (the user asked): not flown** -- nothing changes the burn speed, so separation would not
  move. The fin-booster stack on the pad is the live `FMRS/before_launch.sfs` of 11-12 21:49 ("Heavy Shuttle",
  129 parts, all part types present on the farm; untested there): copy it to `saves/` if an ascent needs it.
  `stackfly.py --booster-set` passes booster flags.
- **The user's direction at wrap-up:** asked whether ~5.5 km ignition is high and whether the burn fine-tunes
  touchdown. Facts: LOG16351 lit at 5.8 km / 735 m/s, ~4.2 km of real stop plus ~1.6 km of margin
  (`SUICIDE_MARGIN` 1.18 + `SUICIDE_REACTION_S` 1.5 s), throttle 0.66 at first, 0.95 near the bottom. The burn
  does steer (throttle loop; tilt <= 15 deg + divert, coned upright under 250 m) but on the fin boosters it
  adds error: predicted 1 m at 10 km, 36 m at ignition, landed 71 m short. **Best next lever: the burn's margin
  and its terminal accuracy (item 1a below), not the descent.**
- Farm stopped; swap was 24.3 GB after cone1 (restart before any batch). Defaults fingerprint back to **7cd7531a**.

The user (live LOG16183/16184, a modified fin booster, now `saves/qs_finbooster2`): it oscillates a lot, mostly
roll in the unpowered descent; it ran short of fuel in the burn; it must work with every engine gimballing (they
had turned the outer ring's gimbals down -- not in any save, all 13 flew at 100). Journal:
`docs/boosterland/journal.md` "2026-11-12 night". **Defaults fingerprint 7cd7531a.** All committed and pushed;
**howItWorks.pdf (boosterland, ascent) not redone**. Farm stopped.

**Fuel (ascent, unflown in an ascent):** the separation budget assumed a sea-level terminal-speed landing (405
m/s); the booster lit at 680 m/s and needed ~850. `ascent/budget.fall` falls through the body's density on the
stack's Cd*A table (taken at liftoff, every ascent type) and lights where boosterland's law does: 833 m/s for
that state. `LANDING_CD` deleted. The farm saves start from the old separation and still land at dry mass.

**Roll (defaults now):** stock credits the tail fins with ~100x their roll (kRPC 100-116 rad/s^2 per unit v
fitted 0.75-1.6), so auto-tune's roll loop barely pushed: 70-110 deg/s spins below 20 km.
- `ROLL_TUNE`: from the coast, auto-tune off; roll gains from `common.rateplant.Plant`'s fitted effect
  (`pi_gains`, closing time slowed to the fins' slew, re-read at 1 Hz); pitch/yaw keep the bandwidth kRPC had
  at coast entry (gain x credited accel), dropped to kRPC's oscillation floor while it latches pitch/yaw.
- `ROLL_SYMMETRY`: n from the surface rings (6); n >= 3: roll angle free from the coast, only the rate damped
  (kRPC target roll NaN); n = 2: nearest of 2 roofs.
- Result (attitude4/5-1113, 18 an arm over qs_finbooster/qs_finbooster2): coast/burn roll rate p90 ~0-18 v
  50-105 deg/s, coast nose rate ~1-7 v 5-25; **miss median 51.5 v 39.5** (one 272 m). The fin2 save's own
  6-flight old arms span 28-70.

**Decision (the user, 2026-11-13): keep this controller as the default; fix accuracy later, on top of it.**
Do not revert ROLL_TUNE/ROLL_SYMMETRY to win back the miss.
- `common/rateplant.py`: `Plant`, `actuators`, `closing_time`, `pi_gains`, `authority`, `clamp` moved from
  airplane/control.py (airplane imports them back). launcherMod's `SurfaceActuators` also lists gimbals
  (`gimbalResponseSpeed`, kind 2: lag 1/speed; built and installed in farm + live).
- Tools: `logsum.py --roll` (per phase/4 km: roll error, |r| p90, nose w p90, input, fit); `KRPC_AP_DIAG`
  (kRPC's per-tick record -> LOG<n>.ap.csv, 3000 ticks from the coast or `KRPC_AP_DIAG_BELOW_M`).
- Deleted after measuring: `ATTITUDE_LAG_PACE` (time to peak from actuator lag: boostback cross very
  repeatable but holdable aoa fell to ~12 and the miss 201 v 88), `ROLL_FREE_IN_BURN` (held in the burn 65
  v 44).

**What limits accuracy now** (read off attitude4/5-1113, `logsum.py --segments`, both arms):
- The error is made in the landing burn, not before it. At burn entry most flights are within +-40 m along and
  cross; they land 30-70 m out.
- The burn ends saturated. It lights at ~4-5 km at ~690 m/s; from ~250 m the throttle sits at
  `LANDING_THROTTLE_CAP` 0.95 until touchdown. With no throttle margin left, the horizontal speed is not nulled:
  touchdown hs is 1-22 m/s, median ~8, in both arms. That is 20-50 m of drift in the last seconds, and it is
  the floor both controllers share.
- What the new controller adds: on 5 of 18 flights the nose swings 30-80 deg/s in the burn (old max 38),
  pitch/yaw saturated (`sat=1.00`). LOG16306 (272 m) arrived at the burn 160 m short, then lost pointing
  while it closed. That is the ~12 m median cost and the outliers.

**Open, in order:**
0. **Next (the user chose it, 2026-11-13 end): the landing burn.** Its ignition margin (`SUICIDE_MARGIN` 1.18 +
   `SUICIDE_REACTION_S` 1.5 s = ~1.6 of LOG16351's 5.8 km; throttle opens at 0.66) and why it adds error on the fin
   boosters (predicted 36 m at ignition, landed 71 short; median ~45-90). Restart the farm first (swap 24.3 GB).
1. Accuracy, on the kept controller: (a) leave throttle margin at the end of the burn -- light it earlier, or
   size `SUICIDE_MARGIN` from the craft's thrust/weight -- so the last ~250 m can still steer and null hs;
   (b) the burn's pitch/yaw swings (`PITCH_YAW_TUNE`, item 2, is the candidate); (c) the `holdable aoa`
   probe now runs at whatever roll the booster has (12 v 45 deg between flights).
1b. **High-AOA final descent** (the user, 2026-11-13): first round done, see the top of this section.
   The original notes, kept for context: arrive at the burn slower than ~690 m/s. What the
   fin boosters can hold (36 flights, attitude4/5-1113): the `holdable aoa` probe reads **44-49 deg** on 29
   flights. That is the lift peak (`lift peak at M1.2-1.8: 44-48 deg`, lift/q 135-158 m^2 at 45 deg against
   Cd*A ~15-30 nose-first), so the torque margin is not what binds. On 7 flights, in both arms, it reads 11.5-13.5
   deg: the probe's turn plane against the fin ring's roll, so measure it at the roll the booster flies.
   The coast now flies only 1-5 deg (it only nulls a miss that is already small) and holds 1-3 deg of actual
   aoa below 20 km. Steps: (i) log the probe's drag as well as its lift per angle -- the gain is Cd*A at 30-45
   deg against nose-first; (ii) a law that flies a large aoa for speed, with its lift steered by bank so the
   range still closes (`AERO_STEER`'s propagator has to fly the same law); (iii) on the farm, compare ignition
   speed, burn dv, throttle-cap time and miss against the defaults. Risk: the tracking was 0.1 of a command on
   the original booster; check trk= at the big angles before trusting the model.
2. `PITCH_YAW_TUNE` (branch `pyTune`, unflown): pitch/yaw fitted and capped as roll is.
3. qs_finbooster's boostback roll (|r| p90 115-195, before the coast: gimbals roll there).
4. Fly an ascent on the new budget (live: the user's Heavy Shuttle); re-fly qs_heavy/qs_heavyshuttlebooster
   with the roll tune (airbrakes, no fins: n from the airbrake ring).
5. Carried from 2026-11-12: boostback departure (LOG16157: err 65-77 deg after the flip, held heading took
   cross -2860; gate throttle on pointing error, or re-take the heading); the wing saturating below 9 km after
   a slow flip settle (LOG16045); `DRAG_PROBE_DESCENT_TILT_DEG` 16 stands in for the boostback-exit drag
   (derive it); landing heights by time-to-go (`LANDING_NULL_ALT_M`, `LANDING_DIVERT_MAX_ALT_M`,
   `GEAR_DEPLOY_ALT_M`, `LANDING_UPRIGHT_ALT_M`), `BOOSTBACK_MAX_BURN_S`, `PREDICT_MAX_TIME_S`, `TERRAIN_MAX_M`;
   `BOOSTBACK_TOLERANCE_M` 190 is insensitive over 20-190; re-fly qs_hot/qs_heavy on MiniSystem.

**Traps:** kRPC's diagnostic buffer keeps the *first* 3000 ticks after it is enabled. kRPC resets
time_to_peak after it is set (read it back). With auto-tune off kRPC's mitigation no longer reaches the gains.
The roll rate `r=` has the opposite sense to kRPC's roll input. Swap sat at 7.5-8 GB even after a farm
restart this session. sweep.py must run under `farmlaunch` with its output tee'd to the job's stdout. The
full suite won't run from a git worktree (kspSim tests import the main tree): merge, then run it in main.
