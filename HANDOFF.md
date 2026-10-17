# HANDOFF — read this first, rewrite it last

## 2026-10-16 late: boosterland -- holdable angle computed; deploy law judged on the unsteered miss (fingerprint `89628946`)

Full write-up: `docs/boosterland/journal.md`, "Session, 2026-10-16 late".
**Defaults unchanged** this session (both new mechanisms are flags, off);
how-it-works PDF still current. Committed and pushed; farm stopped (swap
was 21.6 GB at the end of the batch -- restart before the next comparison);
sleep inhibitor released.

**Where it stands (defaults, 1x):** heavy median ~50-90, qs_hot ~30,
quicksave ~20-55, worst 100-240; every save has an occasional hard or
tipped arrival. Boostback is *not* the error (exits at long -133..-188
every flight); the miss is made in the coast.

**The diagnosis (read the journal entry before changing anything):**
1. The lift solve re-nulls the *steered* prediction every tick and credits
   lift the booster cannot hold (commanded up to 10 deg, held 0.3-3), so
   the predicted miss reads ~0 and the deploy law (airbrakes) sees nothing;
   when the lift saturates the hidden miss appears at once (heavy +280 m at
   9 km, LOG10598; quicksave -500 / +240, LOG10637). That is the deploy
   law's "oscillation" -- not a gain. The user wants a gentle law with no
   hand-tuned gains (generality).
2. Early in the coast the drag curve is re-measured at descent attitude and
   the prediction swings hundreds of m (quicksave -1 -> +850 in 8 s with the
   brakes still).

**Flags (both off; each must end promoted or deleted):**
- `AERO_STEER_HOLDABLE`: holdable aoa from `simulate_aerodynamic_torque_at`
  against wheels+RCS+surface torque; the solve and the propagator cap at
  it. Computes 10 deg at low q down to ~1 deg at 50 kPa on the heavy
  (matches what it holds). hold1-1016 (`logs/rot-hold1-1016.txt`, 12 a
  cell): heavy median 49 v 87 (better); qs_hot 60 v 33, quicksave 35 v 20
  (worse, unexplained).
- `DEPLOY_UNSTEERED`: the deploy law nulls the *unsteered* prediction --
  drag carries the range, lift only trims. **Unflown.**

**Next, in order:**
1. Restart the farm, then fly one batch, 4 arms x qs_heavy/quicksave/qs_hot,
   `LAPS=2` (12 a cell, ~45 min):
   `SAVES_LIST="qs_heavy quicksave qs_hot" LAPS=2 boosterland/tools/brot.sh
   pair1-1017 "off:" "uns:DEPLOY_UNSTEERED=True" "hold:AERO_STEER_HOLDABLE=True"
   "both:AERO_STEER_HOLDABLE=True,DEPLOY_UNSTEERED=True"`
   (arms of brot.sh take comma-separated sets). Expect `both` to be the
   intended design. Promote the winner(s) to default, delete the losers,
   update the how-it-works PDF if a default changes.
2. If the original booster still loses with HOLDABLE: compare its `can=`
   column with `hold=` (the angle actually held) in the quicksave logs --
   is the limit too low there, or is the solve now leaving range to a
   deploy law that cannot take it?
3. The early-coast prediction swing (re-sweep at descent attitude): don't
   step the brakes before the descent re-sweep, or take that sweep before
   boostback exits (a more accurate boostback cut).
4. Open failures: LOG10720 (qs_hot, BOOSTBACK TIMEOUT 180 s, splashed 88 km
   -- boostback never converged; separation took 2.3 s); LOG10748 (qs_hot,
   hovered to the 420 s timeout). Touchdown speed with engine allocation
   (heavy worst 14 m/s) from the previous handoff.
5. Breadth on the five original saves; `docs/boosterland/failures.md` still
   lacks entries for corrections-only-shorten and the |v| landing hover.

**Traps paid:** don't run the unittest suite while the farm is up (it was,
for 5 s, no harm); a tracking sample from a small command reads 1.00 and
licenses full lift credit at 10 deg; read `long` from the *unsteered*
prediction when asking what the brakes should do.

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
