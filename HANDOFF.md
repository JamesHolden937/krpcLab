# HANDOFF — read this first, rewrite it last

## 2026-10-21 morning: spaceplane -- breadth over four orbits; coast RCS, divert on energy height, latest-burn fixes (fingerprint `fb9a9b61`)

Committed and pushed.  How-it-works PDF updated.  Journal: "Morning,
2026-10-21".  Farm stopped, sleep inhibitor released at the end.

**The user's goals (memory `spaceplane-goals-2026-10-20`):** (1) craft
flyable with minimum stability margin; (2) reliable landings in the
shortest burn-to-intact-stop time.  Don't size burns for nose ballast;
don't fly `qs_stackorb`; 500 km saves out until 80 km is reliable.

**Defaults now** (on top of 10-20's `DEORBIT_LATEST` share 0.15, aim
1200, cone energy speed, `ABORT_NORTH_TOLERANT`):
- **Latest-burn fixes** (c29d7ff): the walk's first hit no longer goes to
  the live search (ecc: SAS held a node for *now* 950 s, 0/6 burned); the
  walk refuses burns dearer than a surface periapsis (high: 75.6 m/s, 24
  km window).
- **`ABORT_SHORT_BEST_GLIDE`**: the short divert judges reach on
  max(4.2, the table's best glide at the landing mass) over *energy*
  height.  The first cut read `airframe.glide_ld(env, cfg)` = the
  configured 4.2 and was inert -- with a cfg, `_derived` always returns
  the constant.
- **`COAST_RCS_TURN`**: RCS relay permitted in the coast.  The warp holds
  the burn's retrograde attitude to ~70 km; on wheels alone a third of
  entries met the air 25-62 deg in sideslip and lost roll at Mach 5-7.
  coastrcs-1021 ecc 9/9, entry sideslip <=1 deg, no glide departures.
- **`BANK_ACCEL_NO_RCS`**: the reversal slew is sized on wheels +
  surfaces, not `available_torque` (which counts RCS while open).
  bankrcs-1021 high 4/10 v 1/9.
- Deleted: `HAC_TRIM_AT_TARGET` (30/47 v 31/47).  No flags open.

**Where it stands (on the runway, latest batches):** base 5/6, inc 6/6
(orbits-1021); ecc 9/9 + 6/8 + 7/9; **high 4/9, 4/10** -- the blocker.
Burn->stop 1150-1250 s on ecc/high, ~1100 on base.

**The blocker -- high orbit, energy-poor arrivals:** with clean entries
the high glide banks -46..-56 at Mach 7, predicts on aim through Mach 5,
then from Mach 4.5 its own predicted cone entry sits at the 12.07 km
floor against the 14.14 km target and it arrives at 12 km and 157-197 m/s
(LOG13601; good arrivals 12-15 km at 200-280).  The propagator
over-predicts range at high bank on this steep entry (60.8 m/s, 42 km
window); the departing entries used to hide it.

**Next, in order:**
1. **Score by each log's own save and `config:` line**, never `armN`:
   when an instance dies at a cycle restart the arm labels stop matching
   (trim2-1021, ksp2).  And by the `DOWN:` line, not `along`.
2. **High orbit:** measure the propagator's range error against bank on
   `qs_shuttle2_high_rigoff` (predicted vs flown energy at Mach 4.5,
   split by the bank flown above Mach 5).  If the model can't be fixed,
   derive the latest burn's window floor from that error rather than
   re-fitting `DEORBIT_LATEST_WINDOW_SHARE`.
3. Re-fly base and inc on the new defaults (coast RCS changes every
   entry).
4. Spurious short diverts: one in ~24 remains (LOG13462: 105 m "short"
   at 102 m/s, touched down mid-runway).
5. Goal 2: rollout braking (~1.3-2 m/s^2, wheels only), a touchdown-aim
   law, the terminal glide's energy at ~40 deg alpha below Mach 1.5.
6. Goal 1: `qs_shuttle2_fwdwing_rigoff` on the new defaults -- coast RCS
   and the torque-sized reversal may change where it departs; then a yaw
   damper on sideslip/yaw rate.

**Traps paid:** `airframe.glide_ld(env, cfg)`/`stall(env, cfg)` return
the *configured* constant whenever a cfg is passed -- read `env.best_ld`
etc. for the table's number.  A batch that loses an instance mislabels
its arms.  `&&` after `| tail` commits even when the tests fail (the
pipe's status is tail's).  `pkill -f` from a shell holding the pattern
kills that shell.

## 2026-10-20: boosterland -- FMRS button: booster active on load, JUMP BACK TO MAIN MISSION; flight code as 2026-10-18 (fingerprint `7aa71525`)

**`boosterland/fmrs.py`, launcher button "BOOSTBACK FROM FMRS SAVE"**
(`./run.sh --pilot fmrs`), reworked 2026-10-20 at the user's request:
1. It first saves the live game (the main mission) as `krpclab_main` --
   under a unique name, found on disk among `~/Kerbal Space Program/saves/*`
   and `testInstances/ksp*/saves/*`, then renamed; that is how it learns
   the save folder (`--save-dir` overrides; a remote `--address` needs it).
   Refuses if the live game's active vessel *is* the booster.
2. It reads the `FMRS/FMRS_save_<n>` files as text (`common/sfs.py`), takes
   the first with a booster (closest vessel to the active one, in flight,
   with an engine module, within 5 km), writes a copy with `activeVessel`
   set to it (`FMRS/krpclab_boostback`) and loads that: **no vessel switch**
   (ksp0: 2.6 s wall from button to the first tick, LOG13114/13115).
3. When the flight ends (touchdown settled, destroyed, or TERMINATE) the
   panel shows **JUMP BACK TO MAIN MISSION** (TERMINATE stays with the
   booster). Jump: saves the booster's game as `krpclab_booster`, replaces
   every VESSEL with the booster's persistentId in `krpclab_main` by the
   booster's final block (or removes it if destroyed), keeps
   `activeVessel` on the main vessel, loads `krpclab_main`.
   Tested on ksp0 (LOG13115, main = FMRS_save_0 itself): destroyed booster
   removed, main mission reloaded with the upper stage active. The
   **landed** merge is unit-tested only (`tests/testFmrs.py`) -- ksp0's
   FMRS save crashes its booster (91-316 m out, falls over); not chased.
   Untested: whether FMRS itself objects to the reloaded main mission.
Also: `fakeksp.Vessel` gained `moment_of_inertia`/`reference_frame`
(FLIP_THROTTLE_LAW read them; 20 closed-loop tests had errored since
2026-10-18). Suite 656 OK.

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

## 2026-10-19 night: ascent -- all three stacks reach 80 x 80; the shuttle keeps its wings (fingerprint `5a3c681f`)

All committed and pushed. Farm stopped. The user stopped the session and
will fly the current defaults live themselves (no re-fly asked for).

**Where it stands (defaults):** classic MechJeb, turn shape solved at
liftoff, booster dropped by budget, upper stage followed to orbit.
- qs_stack (shuttle, gears now autostrut Grandparent): 12/12 to 80 x 80,
  **64.48 t sd ~0.1** in orbit (gs1-1019, LOG13070-13081), 0 joint breaks.
  That leaves ~1.0 t LFO = **~51 m/s** (Rhino 340 s) -- the 34 t ore payload
  eats the margin.
- qs_stack_small 12/12 10.17 t; qs_stack_stretch 12/12 10.11 t
  (fix1-1019, LOG13034-13069; Jebediah now in the pod).

**This session changed:**
1. Saves: small/stretch had an uncrewed pod -- no control source after
   separation (all 24 lost). qs_stack gears Heaviest -> Grandparent: the
   struts ran to the booster tank and tore the delta wings off at
   separation (~11/12). Craft file fixed too. **The user's live VAB craft
   still has Heaviest on the three gears** -- told them.
2. Liftoff warnings (no flag, logging only): `upper stage: ... control X`
   / `WARNING upper stage has no control source`; `WARNING autostruts
   cross the booster's decoupler`.
3. `check_settings`: MechJeb ascent settings re-read after separation and
   re-applied if lost (they were not lost in any flight; a guard).
4. turnshape: downrange along the ground (was inertial); the turn starts
   at MechJeb's start speed or height, curve measured from there
   (`Path.start_speed`). **Not flown yet** -- model2-1019 was stopped
   part-way; its partial logs are not a result.
5. `turn shape track:` log line + `ascent/tools/shapetrack.py` (model vs
   telemetry). Before item 4 the model ran 1.4-3.6 km short in downrange
   at separation on all three stacks.
6. Hot staging tried and deleted (did not prevent the wing loss).

**Next, in order:**
1. Whatever the user's live flights report.
2. One batch of the defaults on all three stacks (srot LAPS=2): does the
   turn-start model move the solved shape, and do predicted mass and
   shapetrack agree with the flights?
3. Model gaps still open: drag at zero AoA, booster throttle-down and coast
   drag left out; `RESERVE_FRACTION` per booster; `SEP_ACCEL_RATIO`.

**Traps paid:** `pgrep -f`/`pkill -f` with a pattern that also appears in
your own command line kills the tool's shell (again) -- kill by PID from a
separate listing. The farm restart overwrites every KSP.log: read the
CollisionSpy / explosion lines before restarting. A farm launch with every
kerbal Missing produces an uncrewed pod silently. Swap read 10-15 GB right
after each farm restart tonight (other memory pressure on the machine).

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

### Earlier spaceplane snapshot -- 2026-10-12 evening: low alpha flown and refuted; the live tumble not reproducible (fingerprint `01de6e69`)

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

### The live tumble -- superseded by 2026-10-20 above (aft CoM, `FUEL_TO_NOSE`)

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
