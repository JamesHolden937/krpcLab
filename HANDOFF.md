# HANDOFF — read this first, rewrite it last

> **⚠ The farm runs a MODIFIED Kerbol system (MiniSystem, since 2026-11-06), not stock.**
> - Changed: planets' orbits x0.2, Kerbol GM and radius x0.2, moons' orbits x0.5 (Mun at 6 Mm, Minmus at 23.5 Mm).
> - Kerbin's mass, radius, atmosphere and 6 h solar day are stock. Its **sidereal rotation is 21349.47 s**
>   (stock 21549.43) and the year is ~85 days.
> - Kopernicus now actually loads on the farm. Before, it silently didn't, because `KSPTextureLoader` was
>   stripped. It brings its own patches, e.g. solar panels.
> - Farm results from before 2026-11-06 are not comparable with later ones.
> - kspSim models probed before then are stale: re-probe (`kspSim/world.py` warns).
> - The live game (`~/Kerbal Space Program`) and KSP-VSE's `vse/` instance are still stock.
> - Install: `testInstances/miniSystem.sh`; back to stock: `testInstances/miniSystem.sh --stock`. Design: `~/minisystem/CLAUDE.md`.


## 2026-11-13: boosterland + ascent -- the fin booster's roll; the ascent's landing budget

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
1. Accuracy, on the kept controller: (a) leave throttle margin at the end of the burn -- light it earlier, or
   size `SUICIDE_MARGIN` from the craft's thrust/weight -- so the last ~250 m can still steer and null hs;
   (b) the burn's pitch/yaw swings (`PITCH_YAW_TUNE`, item 2, is the candidate); (c) the `holdable aoa`
   probe now runs at whatever roll the booster has (12 v 45 deg between flights).
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

## 2026-11-08: airplane -- crash guard deleted; phase margin; test jets; pitch stiffness (fingerprint after `a8e0e27`)

`airplane/` (read `airplane/CLAUDE.md`, "Where it stands" / "Next";
journal 2026-11-08). The user: delete the crash guard from production;
reduce oscillation; work for the vast majority of aircraft; build jets
and screenshot them before flying. All committed; **not pushed;
howItWorks.pdf not redone (crash guard still in it). Farm stopped.**

**Done:** crash guard deleted (`GUARD*`, `pull_up_loss`). Rate-loop tau
from a phase margin (`RATE_PHASE_MARGIN_DEG`, default 32.12 = the old 4
ticks; 47 is the candidate). `control.Stiffness` (pitch input per rad of
alpha, fitted in flight) and `ATTITUDE_TRIM` (off): the trim integrates the
attitude error at g p. `RATE_SETPOINT_WEIGHT` (1 = off). Three stock jets
(`airplane/tools/craftgen.py`, screenshots `logs/shots/craftgen/`),
Seahawk and Suborbital hopper craft in `saves/craft/SPH`.
`airfly --rotate`, runway launch from a loaded flight.

**Unmeasured:** every flag above -- one flight per cell so far. The
trainer's sag after rotation (both margins) is the model's missing
stability spring, which `ATTITUDE_TRIM` targets.

**Next:** re-fly the killed screening batch on **4 instances** (6 put
21-28 GB in swap within two rounds and the reaper killed the batch), then
3+ flights a cell, promote/delete `RATE_PHASE_MARGIN_DEG` 47,
`ATTITUDE_TRIM`, `RATE_SETPOINT_WEIGHT`; PDF; push.

**Traps:** `git commit -a` swept the user's four uncommitted doc edits
(CLAUDE.md, docs/farmSetup.md, docs/testInstances.md,
testInstances/README.md) into a commit once -- undone; they are still
uncommitted, not ours. Commits take ~2 min (post-commit hook). A fresh
instance sits in the space centre and `launch_vessel` hangs there.

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
