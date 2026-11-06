# HANDOFF — read this first, rewrite it last

## 2026-11-05 evening: airplane -- tested on the Electric Warbird; gains measured in flight (fingerprint `b3fd31af`)

`airplane/` (read `airplane/CLAUDE.md`; history in
`docs/airplane/journal.md`). The user: both modes "very oscillation
prone"; test on the **Electric Warbird** from now on, spawned at 100 m,
200 m/s, full throttle, propPitch on the blades, with stick input.

**Built:** `saves/qs_warbird` (flown there from the runway by
`airplane/tools/airstart.py` -- an edited air-start carries a stopped rotor
that never spins up); `airplane/tools/airfly.py` (load paused, pilot with
`PROP_PLANE`, stick pulses through the window mod's new **`TestStick`**,
trim gusts, per-axis summary). `launcherMod` rebuilt and installed into
the farm and live: **the live game must be restarted** to load it.

**Cause:** gains came from kRPC's torque model, wrong ~4x both ways on the
Warbird (roll 3.7x low: rang, 46 deg overshoot; pitch 4.4x high: sat 10 deg
off). **Fix (default):** `control.Plant` fits each axis's `w' = k accel u -
d w + c` in flight (RLS, 10 s forgetting, only in the 1 s after an input
step, trusted as far as it explains the rate); `control.pi_gains` places
the rate loop's poles at `-1/tau` on it. Old law and `PLANT_FIT` deleted.
Batch 5 (3 an arm, LOG15363-15382): roll reversals 0.36 v 0.98/s CRUISE,
0.70 v 1.23/s HOLD; pitch error 0.5-0.9 v 2.9 deg; HOLD yaw 1.1 v 2.1 deg.
Confirmation of the merged defaults: see the journal's last entry.

**Open, in order:** the user flies it live; HOLD pitch small reversals
(~1/s); softer gust rejection (out-of-trim roll peaks 12-16 deg, no swing);
a second airframe; roll-yaw coupling is hidden by the quality weighting,
not measured.

**Traps paid:** stick pulses must be a player's size -- 0.15 for 0.6 s
banks the Warbird 85 deg and CRUISE spirals into the sea from 100 m (a
4 g turn cannot hold altitude there). A killed `airstart` left instances
whose next `launch_vessel` hangs: restart the farm. `pkill -f` from a Bash
call matches its own shell.

## 2026-11-04 late: the window mod is the only UI; the launcher survives scene changes (airplane `570902c5`, propPitch `ca612601`)

The user's request, all done and committed; **nothing here was flown or
run in a game** (the user: "no need to test this"; they test live).

- **`launcherMod` draws everything**: the launcher's menu and every pilot's
  panel (`common/panel.py` is now a client of it: title, telemetry text,
  and controls -- buttons, checkboxes, number fields, choices -- sent as
  `kind|id|label|value` strings and read back by one kRPC stream of
  `PanelRead`). The kRPC-UI panel (`panel.window`, the glyph-drawn frame)
  and `LauncherPanel` are deleted. Without the mod a pilot's panel is
  headless (kspSim; START never pressed, so fly with `--autostart`).
- **Scene changes**: the mod's `EpochWatcher` (an `Instantly` addon kept
  across scenes) counts `onGameSceneLoadRequested` and `onVesselChange`;
  the launcher polls `Epoch()` while a pilot flies and, on a change, stops
  it (SIGINT, SIGKILL after 3 s), takes its panel down and shows the menu.
  The window and toolbar button exist in flight only; the launcher keeps
  running through the space centre / SPH. A pilot's own load or switch is
  announced first with `panel.expect_scene_change(conn, s)` (fmrs: 90 s per
  load; ascent's booster handoff: 10 s).
- **START SCRIPT** (the window with no launcher running): starts kRPC's
  servers (`KRPC.Core.Instance.StartAll`) and runs `env -i HOME=.. PATH=..
  KRPCLAB_GAME_PID=<pid> /bin/bash -lc "cd <root> && exec ./run.sh"`
  (`Z:\usr\bin\env` under Wine; the pid from `/proc/self/stat`).
  `build.sh` writes `GameData/KrpcLabLauncher/PluginData/launch.cfg`
  (`root=`, `home=`). **Unverified**: that Wine's CreateProcess execs a
  Unix binary from Mono's `Process.Start` -- if START SCRIPT says it could
  not run, that is the place; the window shows the exception text.
- **The game's death**: the launcher exits when a kRPC call raises a
  connection error or `KRPCLAB_GAME_PID` is gone, killing the pilot first;
  pilots also get `PR_SET_PDEATHSIG` SIGTERM from the launcher.

**Traps paid:** `tools/bundle.py` only counts identifiers in *git-tracked*
files as reachable from outside a module; an untracked new package's
imports get renamed away ("cannot import name ACCENT_AIRPLANE"). `git add`
a new package before building the bundle.

## 2026-11-04: propPitch -- the prop brake's tip limit; props stopped when still; torque counter deleted (fingerprint `ca612601`)

The tip limit and the stop when still are defaults (below); **the
reaction-torque counter (`TORQUE_COUNTER`, the spin-sense probe, the trims,
the log's `trim=`) is deleted** at the user's request (it read 0.00 on the
only craft it flew). `PropPilot` takes a `gui` so the airplane pilot can run
it. Read `propPitch/CLAUDE.md` and `docs/propPitch/journal.md`.

**What changed (all defaults):**
- `BRAKE_TIP_LIMIT` (`propPitch/tipping.py`): on the wheels, the braking
  thrust is bounded so every wheel stays down with `BRAKE_TIP_MARGIN` 0.5 of
  gravity's restoring moment, from the moment about the foremost/rearmost
  contacts (gravity, measured aero force, the thrust line's height over the
  CoM). Past the balance point the bound is the balance. Hill-climb
  backstop: halves on a contact off the ground 0.25 s or pitch rate > 10
  deg/s, rises 0.25/s; where no bound can be computed it climbs from 0 at
  0.1 g/s. Log `tip=`; events `tip:`.
- Still (|v| < 0.5 m/s): every rotor at 0 rpm with its brake latch set,
  until 2 m/s (`BRAKE_RESUME_SPEED`); `mode=brake-stop`.
- `propfly --wheel-brakes P`; `tools/brakesum.py`.

**Measured (farm, qs_prop_noon):** bound ~-17..-25 kN at low speed, ~0 from
55 m/s. Braked from 33-38 m/s it stopped intact in 8-9 s (LOG15245/15246).
From 55-75 m/s it slides and hops whatever the props do; a plain throttle
cut there breaks up 4/6 (LOG15249-15254): the airframe under the harness.

**Not measured:** the tip limit on a wide wheelbase or props below the
CoM; the brake live.

**Next:** the user flies it live. Then: re-find after a lost part (still
ObjectDestroyedException); a harness that can fly this plane past 55 m/s.

**Traps paid:** "no bound on this side" must never mean "unlimited"
(LOG15225-15230). A bound extrapolated through a filtered acceleration lies
after a 1000 kN thrust swing (LOG15233). The fallback must climb from zero
(LOG15221). A propfly launch of six with an inner `&` returns at once: wait
on the `ROT DONE` marker.

## 2026-11-01: boosterland -- gimbal roll really off again; correction when the brakes are spent; the FMRS Heavy Shuttle booster (fingerprint `56a3d2b9`)

Started from the user's live FMRS flight (LOG15102: Heavy Shuttle booster,
596 m out, broke up). Committed and pushed; farm stopped. Journal:
"2026-11-01".

**Fixed (defaults):**
- `set_gimbal_roll` -> `common.partmodule.set_field_bool_by_id`. kRPC 0.7
  has no `Module.set_field_bool_by_id`; **from 2026-10-21 to today every
  booster burned with roll on its gimbals** (logged "0 engines (n
  refused)"). LOG15102 spun 100-227 deg/s in the landing burn. Refusals now
  log the exception. Roll-off works while the gimbal's toggles are hidden.
- `CORRECTION_SATURATED_ENTER_M` 150: with the brakes at `DEPLOY_MAX_DEG`
  and the steered prediction long, correct above 150 m (not 1000). And no
  correction while `late_state == "switched"` (before the opened brakes are
  re-swept). 9 an arm, gate v off: FMRS booster median **70 v 292 m**;
  qs_heavy 112 v 41 median, means 139 v 133, destroyed at contact 2/9 v 6/9.
- `saves/qs_heavyshuttlebooster`: the live FMRS boostback save, booster
  active, Kerbin orbits' LAN -9.8916 (the farm put it 10 deg east of the
  live game; see saves/README).

**Where it stands:** both boosters leave boostback on a curve swept at aoa
20-25 deg (inside `DRAG_RESWEEP_AOA_DEG` 25); the first descent sweep says
+400..+1200 m long *with the brakes full open*, and the first correction
takes it to ~100-200 m. qs_heavy then ends ~120 m west on 6/9 (just under
the gate; the landing burn does not divert it). Pre-0.7 the heavy landed
median ~13 m. **Tip-overs are not scored**: the Heavy Shuttle booster has no
legs and the user will fix the craft.

**Next, in order:**
1. The boostback's drag curve: probe the descent attitude (aoa ~0, what
   `holdable` says) instead of the real one while boostback is at 20-25 deg,
   or tighten `DRAG_RESWEEP_AOA_DEG` -- the boostback exit should land
   inside [closed, open] of the late brakes. pd10-1016 lost boosters probing
   at retrograde; that was before the late brakes, re-measure.
2. The ~120 m residual after the first correction on qs_heavy.
3. Older, still open (the 2026-10-18 list): brakes for speed not range;
   aoa as a speedbrake; the +28 m along-track from the RCS trim; legs down
   ~13.5 s early; pad debris; qs_hot sideways arrival.
4. FMRS on live: LOG15102's `FATAL jump back: ConnectionError` came while
   the panel waited -- the game closed under it, not chased.

**FMRS button** (`./run.sh --pilot fmrs`, `boosterland/fmrs.py`): saves the
live game as `krpclab_main`, finds the first `FMRS/FMRS_save_<n>` of this
launch with a booster in flight, writes `FMRS/krpclab_boostback` with it
active, loads it paused, flies it, then offers JUMP BACK TO MAIN MISSION
(booster's block merged into `krpclab_main`). Landed merge unit-tested only.

**Traps paid:** a swallowed exception in a mode switch reads as "0 done"
for ten days -- log the reason. A live save on the farm may load somewhere
else: check `pad=` on the first log line against the live log. Swap reaches
27 GB per 10-min batch: restart between batches. Readers: `logsum.py`;
per-flight corrections from `phase COAST -> CORRECTION`.

## 2026-11-01: spaceplane -- the approach plans in the middle of each craft's own span; the low-alpha glide is off (fingerprint `07089ff3`)

Committed and pushed; farm stopped (the last batch, heavy-1101, was killed by Claude Code for low memory before it flew -- LOG15091-15095 are part-batch, don't read them). Journal: "Session, 2026-10-31 late" and "2026-11-01". **howItWorks.pdf NOT updated for the approach ratio** -- do it next wrap-up.

**`APPROACH_BEST_LD` is deleted (the user: "remove the hardcoded value").** It was qs_shuttle2's 4.2; the live Heavy Shuttle (best glide 3.0) planned at 4.2 got a 7.2 km gate and landed 765 m short (LOG15006). Now `airframe.approach_ld` = clean reach x sqrt(braked span), at `GATE_ALT_M`:
- clean reach = glide x (h + float)/h: glide is `cla/cda` at 1 g at `APPROACH_FACTOR` x stall, read from the STANDBY sweep **at the dry mass** (`Vessel.dry_mass`, `airframe.stall_at`) and kept on `env.approach_glide_ld`, scaled by `env.approach_ld_scale` (= `hac_ld_scale`, the flown/table ratio, 0.94-1.16 on both craft); float = `(v_app^2 - v_stall^2)/2g`.
- braked span = the split rudder's last `APPROACH_SPLIT_FACTOR` (0.45), or cos(`APPROACH_SCURVE_MAX_DEG`) with no pair.
- Consumers' formulas unchanged (gate, handover `approach_needed`, excess, S-turn, split brake, short divert). The DISAGREES check is gone.
- **qs_shuttle2: 5.11 x 1.23 x 0.67 = 4.22** (gate 7242 m, was 7200) -- inert, by design. Heavy, from LOG15006's live table: L/D 2.65 at 115 m/s, reach 3.37 (flew 3.41), **planned 2.26, gate ~3.3 km**. Unflown on the heavy: see below.
- **Refuted on the way:** planning at the clean reach (bec678c, reverted 6fb047b): reach-1031 0/12, every flight short at 6.3. The table *is* right where flown; the shuttle simply can't be planned with no margin.

**`GLIDE_LOW_ALPHA_SPEED_M_S` 1000 -> 0 (off).** lowa-1101, rigoff 12 an arm: on 2/12 on the runway, **7/12 out of energy 56-82 km short** ("on the ground in HAC"); off 7/12, none. This was last session's unflown default and the user's live flights carried it. The rule says delete or promote: it was the user's idea, so ask before deleting the code.

**`saves/qs_heavyshuttle_rigoff`** (new): the live Heavy Shuttle on rigoff's orbit, LOG15006's propellant (70.37 t), surfaces/gear stowed. Two traps cost two batches: the first copy had full tanks (182.6 t, entry breakup, 0/8 at 100-300 km) and was taken from a persistent saved *after* landing (4 surfaces deployed, gear down: the farm table read CdA 41/ClA 12 at 0 deg against live 25/56). **The stowed version has not flown.**

**Added after the wrap-up (2026-11-01 evening, the user, default, unflown -- they fly it live):** `APPROACH_HOT_BRAKES` -- past the threshold with energy excess (height over best glide to the aim + (v^2-held^2)/2g) > 0, the split rudder goes to its stop and the opposed flaps open at once; latched; still stowed at the flare door. Log line `hot over the runway: every brake out`. Cause: LOG15100 (live Heavy Shuttle) crossed the threshold 460 m up at 95 m/s, rudder 24/38 deg, flaps never opened (they wait for the rudder's stop), rudder retracted at h=400 once the aim was behind it, flare 1.1 km down, touchdown 1726 m along, then an overrun divert north 0.8 s after touchdown on a 2.1 m/s^2 first reading. Also: the overrun divert now waits 1 s after touchdown. Root cause still open: the approach's `exc=` is height-only, and the speed schedule (127 -> 91 m/s) hides ~360 m of energy, so it read 430 m low at the cone exit and opened the brakes 900 m out. Fix candidate: count speed over the held speed in the split brake's factor.

**Next, in order:**
1. Fly heavy-1101: `multirot.sh heavy-1101 2 "qs_heavyshuttle_rigoff|" "qs_stackorb_live1028|"`. Read the `approach at ... planned at` line and the gate (expect ~2.3 / ~3.3 km), and whether the farm table now matches LOG15006's (`aero M=0.3` row). If the heavy still breaks up in the entry, that's its own failure (RCS blocks at 100% skin, then 74 deg alpha at 46 km in LOG15043).
2. Ask the user: delete `GLIDE_LOW_ALPHA_SPEED_M_S`'s code, or find why bank-first runs out of energy.
3. Rigoff off-arm still had 2 ROLLOUT losses and 3 misses (-2449, +1898, -8813) in 12.
4. The brake probe's quiet-inputs version (a3a64cf) still unflown; read `spoiler (measured) ... armed/REFUSED` live.
5. Carried: `APPROACH_SPLIT_FACTOR` is qs_shuttle2's splitprobe table and now also sets the approach's span; LOG14999's 118 m off-centreline touchdown; the stack orbiter's cone outliers; kspSim probe models.

**Traps paid:** a background batch launched from `testInstances/` (the Bash cwd resets there) silently found no `spaceplane/tools` -- `cd /home/holden/krpcLab &&` in every launch. The telemetry's `cla=`/`cda=` are the *table at the commanded alpha*, not flown values. The swept table re-read at the landing mass is noisy (2.4-5.6 best glide on one craft). Zram at 28 GB after two cycles is now normal; restart between batches.

## 2026-10-27: tooling and the farm harnesses; boosterland's farm entry state (bundle 369 kB)

No pilot's flight laws changed.  What did:

- **Bundle 379 -> 369 kB**: C# `name=` no longer pins parameters, test strings
  no longer pin `vec`/`math`, annotations read builtins through aliases,
  `SimpleNamespace`/`Thread` keywords no longer pin parameters.  Post-commit
  hook: its suite bundle was written into the root and failed every commit
  ("came from the tree"); fixed, `post-merge` calls `post-commit`.
- **Dead code deleted**: spaceplane `hac_available`, `airframe.glide_ld`;
  boosterland `landing_attitude`, `terminal_attitude`; `idle.rescan`.
- **Every farm harness flies the bundle** through `tools/farmflight.py`
  (launch in its own process group, reaper, `claim_log` by the pid now in
  every log header); `--tree` flies the modules.  `fly.sh` runs PyPy.
- **From the bundle, `paths.ROOT` was `dist/`**: the launcher's children, the
  spaceplane's logs and the gear cache would have gone into `dist/`.  Fixed
  (a checkout's `dist/` roots in the tree; a copied bundle keeps its own dir).
  Nobody had flown live from the bundle before this fix.
- **quickfly** no longer runs the autoland in-process: the child loads the save
  (`autoland --load`), the instance is held at 1x through the load and the
  child's start-up, the first tick waits for save UT + 4.6 (`--start-ut`), then
  the harness raises the scale.  Every flight engages at 16681.78 +-0.02.

**Boosterland finding (open, not chased):** a save loaded at 1x is a different
booster from one loaded at 4x -- hs 699.6 m/s at 16681.78 every flight, against
701.7-702.8 for the old 4x load -- and on `quicksave` it lands **100-360 m
west** (e1-cpy: 19/64/69/170/259/361 m) where the 4x-loaded state landed
23-155 m, mostly within 60 (e1-old, interleaved on a fresh farm).  Neither the
bundle nor PyPy is the cause (same result under `--cpython`).  The 1x state is
what a live game flies, so the farm booster numbers before today were measured
on a 4x artefact.  Next for the booster: measure over several saves at the new
state; find what a 2.6 m/s entry difference does that the boostback does not
null (the coast miss is ~180 m in both).  Logs: `logs/harness-1027/`.

**Trap paid:** the farm reached 22 GB of swap after an hour of six-instance
booster flights and every comparison flown on it was worthless -- restart
between comparisons, as the farm section says.

## 2026-10-19 night: ascent -- all three stacks reach 80 x 80; the shuttle keeps its wings (fingerprint `5a3c681f`)

**2026-10-21 (farm-wide, see the spaceplane section):** kRPC 0.7.0;
ascent's staging on `Vessel.stage_at`/`decouple_stage_at` (0.7 removed
`Parts.in_stage`/`in_decouple_stage`/`resources_in_decouple_stage`).  One
smoke flight (qs_stack): 81 x 79 km.  KRPC.MechJeb 0.8.1 still loads under
0.7.


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
