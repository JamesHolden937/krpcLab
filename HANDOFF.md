# HANDOFF — read this first, rewrite it last

## 2026-10-14 morning: `lowThrust/` flies in game -- LKO -> Mun arrives

Defaults fingerprint `42882c64` (lowThrust). All committed and pushed to
origin/main; farm stopped, sleep inhibitor released. How-it-works PDF
rewritten (solver + pilot + plugin). History: `docs/lowThrust/journal.md`,
"Session, 2026-10-13 night".

The user: "implement the low thrust autopilot into the game", plugin
allowed, "just don't make it cheaty in any way". Built `lowThrust/pilot.py`
(panel: target, PLAN, START, TERMINATE; launcher button LOW THRUST
TRANSFER), `live.py`, `tools/ltfly.py`, and the plugin
`lowThrust/plugin/` (kRPC service `LowThrust`: PT throttle on rails,
instant altitude-capped rails warp that climbs back as altitude allows,
RAILS_PACE 20 with exact holds, reaction-wheel attitude on rails with EC).

**The user must copy `testInstances/base/GameData/LowThrustWarp/` into the
live install's GameData** (outside krpcLab: not done by Claude).

Measured in game, `qs_lowthrust` (29.9 t, Dawn, 0.067 m/s^2, 12 RTGs):
- 87 -> 150 km: 150.1 x 150.0, 97.9 m/s, 1.6 d (stand-in 97.9, 1.55 d),
  90-140 s wall (LOG10130).
- Mun 50 x 50: arrived 3/3 -- 1392/1372/1388 m/s, 77/69/83 days, 15 min
  wall (LOG10131-10133; stand-in 1319 m/s, 44 d).
- LOG10134: the Mun flight logged every tick (`LOG_INTERVAL_UT=1`), stopped
  at wrap-up mid-PUMP (ap 2448 km, UT 213k-424k): the pump's arcs for the
  PUMP gap below.

Open, in order:
1. **In-game PUMP ~54 days v the stand-in's 35** for the same dv; tracking
   the orbit frame fixed FINAL (26 -> 12-19 d) but not this. Read LOG10134's
   arcs against `simmission --thrust 2000 --mass 29875 --isp 4200 --ut
   213522`'s burn list.
2. MATCH drops Kerbin's periapsis to 66 km in every Mun flight before the
   SOI entry rescues it: check the floor guard during MATCH.
3. RCS torque and gimbal misalignment on rails (the user asked; not built;
   the craft has neither).
4. Duna in game, then aerobraking (the craft has a heat-resistant cone).

Traps paid for: `pkill -f` matching its own shell; a test burn with no
attitude held burned the ksp2 copy into the atmosphere; **count parts, not
titles** (a set of titles showed one RTG of twelve and a false power
shortage was reported to the user).

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
