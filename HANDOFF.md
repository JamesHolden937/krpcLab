# HANDOFF — read this first, rewrite it last

## 2026-10-14 evening: `lowThrust/` is a standalone C# mod; the lag is gone; a dv/time budget

Defaults fingerprint `d4db5b79` (lowThrust, C# `Config`). Committed and
pushed to origin/main; farm stopped, sleep inhibitor released. How-it-works
PDF rewritten for the mod. History: `docs/lowThrust/journal.md`, "Session,
2026-10-14 afternoon -- evening". Read `lowThrust/CLAUDE.md` before touching it.

What changed (the user asked for all three):
- **Standalone mod** (`lowThrust/mod/`): solver ported to C# (parity on the
  270-flight grid: 270/270, mean dv 4914.3 v 4914.7, 176 byte-identical);
  the autopilot runs in the game's physics step -- thrust integrated on
  rails with the engines' own propellants (ElectricCharge from the part
  config), orbit written through KSP's elements. No kRPC/Python/Persistent
  Thrust in flight. Python implementation deleted; launcher button removed
  (the mod has its own toolbar window). Farm-only kRPC bridge
  `LowThrustKRPC.dll` for `ltfly.py`.
- **Lag**: no Unity time-scale games; warp up to 100000x governed by frame
  budget, propellant per step, turning, altitude (limits relaxed to the
  atmosphere top while flying). Mun 125-137 s wall (was 545-557), 87 ->
  150 km 6 s (120). Left: a search in WAIT freezes one frame up to ~0.4 s a
  few times a flight.
- **Dv/time budget**: Max dv / Max days fields, PLAN shows the frontier
  (13 arc widths flown in parallel, ~7 s), clickable; impossible budgets
  refused (instantly below 0.8x the impulsive estimate, else after the scan,
  with the achievable range). `simmission --frontier/--max-dv/--max-days`.

`build.sh` installed the mod into the **live game** too and removed the old
`GameData/LowThrustWarp` there. Persistent Thrust can stay installed.

Measured in game (`qs_lowthrust`): Mun 9/9 inline arrive, 35.1 d where
planned 35.0 and 20.8-20.9 d where planned 26.9, 1332-1381 m/s (plan
1297-1311); budget 1350 -> 1351 m/s; max 10 d refused in 14 s; Duna arrived
586 d / 2695 m/s (plan 528 / 2256); 400x150 i3 argpe90 arrived 16 s wall.
Flag measured and deleted: decisions on a worker thread with game time held
(THINK_ASIDE): 5/5 captures failed in its first form, then worse alignment
and later arrivals than inline (LOG10170-74 v LOG10176-81).

Open, in order:
1. Re-fly Minmus inline (both Minmus flights used the deleted aside path:
   1533 and 2786 m/s v plan ~1380); then dv over plan (Mun +3-6%, Duna +19%).
2. The WAIT frame freeze: make the push search incremental across steps.
3. RCS, gimbal and steering losses -- the user: "another time" (design in
   `lowThrust/CLAUDE.md` Open 3).
4. More planets in game, routes from a moon, aerobraking.

Traps paid: PT strips EC from the engine module at run time; simulated
`RequestResource` on rails offers ~nothing (use connected totals); the farm
loads the dll at boot (restart after build); never use world-space or
inertial world vectors below the inverse-rotation altitude.

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
