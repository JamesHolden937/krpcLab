# HANDOFF — read this first, rewrite it last

## 2026-10-15: lowThrust escape inside the SOI; multi-pass aerobraking

Defaults fingerprint `feec62da` (lowThrust, C# `Config`). Committed and
pushed to origin/main; farm not started this session (everything was
measured in the stand-in, `lowthrust.exe mission`); sleep inhibitor
released. How-it-works PDF rewritten. History: `docs/lowThrust/journal.md`,
"Session, 2026-10-15" and "Aerobraking: the tail, and Ike". `build.sh`
installed the dll in the farm and the **live game** (restart KSP to load it).

The user asked:
- **Why the departure raised the periapsis / didn't finish the transfer
  inside Kerbin's SOI**: the raise was the DEPART law; the correction was left
  to the cruise. Now ESCAPE, when the exit would need the cruise's Lambert
  re-target, aims it from the exit and burns it inside the SOI (Oberth).
  Live Duna state in the stand-in: 1963 -> 1796 m/s; default craft Jool -89,
  Dres -109, Duna/Ike unchanged (gated: a small miss is left to the B-plane).
- **Aerobraking at 32x**: physics warp was an index; BetterTimeWarp's index 3
  is 32x. Now a rate (`AERO_PHYSICS_WARP` 4.0).
- **Multi-pass aerobraking (deep, shallower, skim, lift out)**: the tail crept
  (12-13 passes, ~45% of the remaining apoapsis each) because the floor check
  used 2.25x drag. Now `AERO_RA_MARGIN` 1.25 or the worst observed miss;
  ends when a periapsis burn finishes it under `AERO_MIN_PASS_DV` 2 m/s.
  Plus five Ike fixes and a cost ledger. Stand-in sweep (Duna 200 and 80 i0,
  cdA 12 and 0.58, 10 dates): 40/40 arrive (24/40 before); Duna 200 cdA 12
  6-7 passes, dv equal or -75..-103.
- **"13 arc widths"**: `PLAN_GRID` -- explained (planner frontier).

New/changed config (all default): `AERO_RA_MARGIN` 1.25, `AERO_MIN_PASS_DV`
2.0, `AERO_PHYSICS_WARP` 4.0 (a rate, was index 3). No flags.

Open, in order:
1. **Fly multi-pass aerobraking on the farm** (Duna 200, Duna 80 i0 saves) --
   nothing of this session has flown in the game. Watch: observed miss vs
   1.25, the last skim's exit lift, Ike trims before passes.
2. ESCAPE's Lambert burn in game (a live Duna departure).
3. A pass still flown on rails (Kerbin pass 5, 2026-10-14): find who sets the
   rate (log TimeWarp.CurrentRateIndex after Warp.Set near the air).
4. Kerbin aerobraking heat-bound by RTGs (~250 passes from 3000 km): higher
   `AERO_HEAT_TARGET`, `AERO_HOLD` Retrograde, less soak.
5. Laythe hits the stand-in time limit (both builds). Weak-engine departure
   caught by the Mun (LOG10202). RCS/gimbal, routes from a moon.

Traps paid: an ungated ESCAPE Lambert cost Duna +45 / Ike +100 m/s (a small
miss is cheaper fixed over the cruise); trims inside AEROBRAKE without a
ledger blew low-drag runs to 10,837 m/s / 300 passes; the floor guard must
count a TRIM resuming AEROBRAKE as aerobraking; mono's "Write fault" is
only `| head` closing the pipe.

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
