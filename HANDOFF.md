# HANDOFF — read this first, rewrite it last

## 2026-10-14 late: lowThrust aerobraking; capture flip-flop; Minmus push

Defaults fingerprint `5822bb01` (lowThrust, C# `Config`). Committed and pushed
to origin/main; farm stopped, sleep inhibitor released. How-it-works PDF
rewritten. History: `docs/lowThrust/journal.md`, "Session, 2026-10-14
late". Read `lowThrust/CLAUDE.md` before touching it. `build.sh` installed
the current dll in the farm and the **live game** (restart KSP to load it).

The user asked (about their live Duna flight, flight-20261014-183836.log):
- **Why braking started late**: the capture law flip-flopped between
  retrograde and a periapsis-gradient burn ~100 deg away (the planned
  periapsis sat exactly on the safety floor), losing every second 250 s tick
  to turning; half the capture was burned after periapsis. Fixed (smooth law,
  aim 5% above the floor, unaligned ticks coast only while turning).
- **Burns away from periapsis in FINAL**: by design -- +-60 deg arcs while
  crossing Ike's band, a capped burn, apoapsis touch-ups of the periapsis.
- **Aerobraking, aggressive with a safe margin**: built and flown. Duna
  200 x 200: 14 passes, 1944 m/s, 546 d (LOG10206); Duna 80 x 80 i0: 15
  passes, 1946 m/s, 536 d (LOG10207; live flight without it 2462 / 561).
  Hottest part 0.57 of its limit (target 0.6, every limit checked at 1.5x
  drag and density). Kerbin from 3000 km: heat-limited by the RTGs at
  3 km/s -- passes at ~65 km, hundreds of them (one reached 0.638).
- **Exit lift with this TWR**: was a real risk (Kerbin lift ~300 s vs an
  85 s arc); now no pass may leave a periapsis the engine cannot lift
  within +-60 deg of apoapsis, and the exit arc is widened to fit.
- **Handoff items**: Minmus over plan fixed (the push's re-solve horizon cut
  off its own encounter; open-loop pushes missed by 29,000 km); WAIT's
  search spread over steps.

New config (all default): `AEROBRAKE`, `AERO_DRAG_MARGIN` 1.5,
`AERO_HEAT_FIRST` 30 / `_TARGET` 0.6 / `_GROWTH` 3 / `_PLAN` 100,
`AERO_TERRAIN_MARGIN_M`, `AERO_MIN_ALT_SHARE`, `AERO_ACCEL_MAX`,
`AERO_MIN_GAIN` 0.005, `AERO_LIFT_MARGIN_M`, `AERO_PHYSICS_WARP` 3,
`AERO_HOLD` Prograde, `AERO_SOAK_S`. New save `qs_lowthrust_kell`.

Kerbin 3000 x 100 -> 100 (qs_lowthrust_kell, last flight on ksp0): 19
passes (63-68 km, RTG to 0.638 once) took the apoapsis to 2550 km, then a
pass gained < 0.5% and FINAL burned the rest: 675 m/s v 683 without -- for
this craft Kerbin aerobraking from high orbit does not pay (heat-bound).

Open, in order:
1. **A pass still flown on rails** (Kerbin pass 5): the governor trace
   wanted 5x for the last frames before the air, but the game was at
   100000x and 2098 s ahead at entry (after unaligned AEROBRAKE ticks at
   100000x, err 120 deg). 1 of 48 passes; it teaches nothing and costs a
   revolution. Find who sets the rate (log TimeWarp.CurrentRateIndex after
   Warp.Set each frame near the air).
2. Kerbin aerobraking heat-bound for this craft: a higher
   `AERO_HEAT_TARGET`, `AERO_HOLD` Retrograde (RTGs shielded?), less soak.
3. Ike vs equatorial aerobraking: 0 encounters in game so far; watch.
4. Weak-engine departure caught by the Mun on a fast transfer (LOG10202).
5. RCS/gimbal, planets beyond Duna, routes from a moon.

Traps paid: `pkill -f ltfly.py` kills its own shell; a farm watch loop must
pick the log newer than the launch (`ls -t` picked a stale one); a time
"a hair negative" from an anomaly difference must not wrap a revolution;
the stand-in cannot show what a law's direction jumps cost (instant
pointing) -- the game can.

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
