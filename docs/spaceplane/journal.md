# The spaceplane: session journal

Every session's measurements and conclusions, oldest first.  Append new
sessions at the end.  Numbers here describe the configuration flown *at the
time*; re-run the tools rather than quoting them (CLAUDE.md, "Re-run the tool").
The design reference is [design.md](design.md); the current state is
[`spaceplane/CLAUDE.md`](../../spaceplane/CLAUDE.md).

## 2026-09-17 .. 09-23: the old craft (`qs_plane`), condensed

Pruned 2026-10-09.  These sessions flew the retired 23-part `qs_plane`; the
full text (~2300 lines, every arm and table) is in git:
`git show f8d63c5:docs/spaceplane/journal.md`, lines 9-2322.  Section titles
cited elsewhere ("Session, 2026-09-23", "Session handoff, 2026-09-22
(evening)", "Flying the entry for drag ...", "A second airframe ...") are
there.  What carried forward, with the failure numbers that hold the
evidence:

- **Farm hygiene** (09-17): orphaned autopilots, drifting per-instance
  saves and a long-running farm's swap each silently biased results; the
  last one 27 km with the spread unchanged.  Now the root CLAUDE.md farm
  rules.
- **Statistics** (09-20): one configuration gave 4/8 and 7/8 in consecutive
  batches; four-flight arms looked "decisive" twice and dissolved.  Pool,
  interleave arms, quote n.  Failures 27, 78, 83.  "Off until flown" is not
  conservative: two disabled changes were worth half the landings (82).
- **The aim's gain** (09-21): one metre of arrival costs 6.4 m of
  `DEORBIT_CENTRE_BIAS_M`; every earlier "the aim is not a lever" was read at
  1/7 of its scale.  Check a knob's gain before a null (88).  A per-state
  bias (51000 on `qs_e60`) must never become a default (89, 93).
- **Governor on the worst tick** (09-21): driven by a mean, the time scale
  ramped during the burn and split arrivals bimodally at 0.22 game-s/tick;
  governing on the phase's peak tick cut arrival sd 13x (91-92).
- **Bank reversal count is a saturation symptom**, not a range term: its
  sign differs between entry states (87); retires failures 78, 79, 85.
- **Uncommanded authority** (09-21/22): split-rudder airbrake, sideslip for
  energy, max-drag entry -- the slip worked on one airframe and did not
  generalise; the split rudder later became the shuttle's speedbrake.
- **A second airframe found five bugs in an afternoon** (09-22): `Drain
  Mode` never commanded, `set_field_value` absent in this kRPC, Isp of an
  unlit vessel reads 0, `ATTITUDE_TIME_TO_PEAK_S` worth 198 km (now derived
  from inertia/torque), RCS "cannot help" measured on a vehicle without
  usable RCS.  Write craft files to test generality.
- **The surfaces were never off** (09-23): `Pitch=False` is KSP's *ignore*
  flag; the missing-controls audit became a tool and a log line.  The
  shuttle's 100 km overshoot was its burn (thrust limiter added); a bank
  reversal taught the alpha learner a false ceiling.

## 2026-09-23 .. 09-30 afternoon: the single-fin shuttle (`qs_shuttle`), condensed

Pruned 2026-10-09; full text in `git show f8d63c5:docs/spaceplane/journal.md`
(the sessions from "Session, 2026-09-23 (second)" to "Session, 2026-09-30
afternoon").  What carried forward:

- **Harness**: the time-scale plugin defeated KSP's pause, so an in-air save
  fell 56 s before the autopilot engaged (`timescale.hold()`, pause after a
  clock check).  Every in-air save before that paid some of it.
- **Attitude layer**: the measured alpha was unsigned; roll and yaw were
  swapped in the tune (failure 96) -- fixing it exposed roll, now the derived
  4.8 s with yaw tied to it; a dive was the pitch controller, not trim
  saturation.  `ATTITUDE_YAW_WITH_RCS` became the default (sideslip 8-13 deg
  against 15-70); kRPC reports no RCS torque while the valve is shut.
- **Actuators**: `Deploy Angle` direction is per part (the flap brake was
  half a flap); gear rules (nose no brake, mains 200%, friction 10, full
  main brakes from touchdown) are the user's and are default.
- **Entry**: the terminal glide had no sink left (failure 98); the live spin
  (LOG3644) was a commanded sideslip -- `aim` pointed the nose with the
  commanded bank, fixed by aiming from the flown bank.
- **Landing**: misses were energy on final, not the flare; `HAC_LD` 1.86 is
  a whole-cone average and the flown ratio varies 0.74-1.68 from orbit; the
  cone's flap brake never deployed as built; gentle touchdowns broke up
  (later traced to rigid attachment, 10-04/05).
- The landing numbers were derived from the airframe (09-25), the first
  step of "constants to curves".

## Session, 2026-09-30 evening: the twin-fin shuttle

The user rebuilt the shuttle (`saves/craft/SPH/shuttle.craft`): the single
centreline Big-S tail fin replaced by **two on the wingtips** (outer
elevons, x = +-5.63 m, z = -15.53 against the old fin's -16.91, canted 15
deg out, mirror-deployed). Everything else identical. Flying it by hand they
put it on the runway at 50-60 m/s, tanks empty. `actuators.py`: every axis
live on both fins, limiter 37.5 as before; MoI roll 94k -> 124k (+32%),
mass 37.50 -> 37.95 t.

**Saves.** `qs_shuttle2` is a splice, not a flight: the craft launched on
ksp0 (`launch_vessel` hangs on a pre-flight dialog if the named crew is
already aboard another vessel -- name an unassigned kerbal), saved, and its
PART blocks put into `qs_shuttle`'s vessel taking structure (parent, attN,
srfN, sym, positions) from the new save and flight state (modules -- gear
retracted --, resources, temperatures, crew) from the old; both rudders
whole from the new. So UT, orbit and fuel equal `qs_shuttle`'s and a pair
measures the craft alone. `_inc`/`_high` by the old recipe; orbits
byte-identical to `qs_shuttle_inc`/`_high`.

**Paired batch, defaults `89b7daaf`** (`logs/rot-shuttle2-0930.txt`,
LOG4124-4135, 6 per arm, ksp0-2, fresh farm). New reader
`spaceplane/tools/lateralsum.py`: per flight, subsonic ticks with flown bank
>60 off the command (`bad60` -- *includes reversal lag*, so it does not
separate the craft), ticks with |sideslip| >10 (`slip10`), peak sideslip,
the cone's exit surplus, touchdown.

| | old `qs_shuttle` | new `qs_shuttle2` |
|---|---|---|
| subsonic `slip10` ticks | 13, 2, 6, 8, 4, 9 | 0, 0, 0\*, 1, 0, 0 |
| peak subsonic slip, deg | 12-30 | 8.8-11.0 (\*47.8 is on the ground, LOG4129 rollout) |
| intact | 1 (LOG4128, splashed 2.9 km long) | **1: LOG4135, 31/31, 6.1 m/s sink at 67 m/s**, +1569 along (past the end) |
| cone exit surplus, m | -6499..+2378 | +959..+5022 |

**The lateral departure is gone on the new craft** -- no continuous rolls,
sideslip inside +-11 deg on every airborne tick of six flights. LOG4135 is
the first intact shuttle landing from orbit.

**What now loses it: the flare inherits bank and cannot remove it.** Every
new-craft loss reaches the flare banked and touches down banked: LOG4129
handed over at -28 (commanded 0 for 5 s, flown -25..-45, contact at -45, a
wingtip -- now a fin -- on the runway, then a 30-48 deg slide); LOG4131 -13
-> -39 -> +87 at contact; LOG4133 +41 at the door. Two sources: the
centreline capture after the S-turn stop oscillates with a ~25 s period
against a vehicle whose roll lags 4-6 s (LOG4129: -17, -30, -4, +20, +18,
-14, -38 commanded down to 130 m) -- measured roll rate 8.5-12.8 deg/s,
roll `time_to_peak` 5.5 s; and the approach dives (LOG4127: +-40 S-turns
at alpha 2-6, 128 m/s and 77 m/s of sink at the flare door, struck at 133).
And the cone exits +1.0 to +5.0 km high (LOG4131 +5022 with laps=0).

## Session, 2026-09-30 night: fuel, and why the shuttle arrives high

**Gear.** `NOSE_WHEEL_FRICTION` (default 1.0): the nose wheel's friction
control switched to manual, the user's rule. Verified in game (LY-35 at 1,
mains 200%/10).

**Fuel.** The pre-burn drain keeps a 200 m/s x 1.25 reserve for the worst
orbit; the shuttle's burns are ~26 m/s, so 1.9 t rode to the wheels (LOG4135:
30.1 t, dry 27.5). In `qs_shuttle2` all of it is in the nose adapter
(+10.8 m) -- the other LF/Ox tanks are empty; 399 mono sit 0.6 m behind the
CoM. Built: `DRAIN_TO_BURN` (drain to the solved burn x1.25 + 10 m/s,
re-solve, then commit; LOG4140: 430 -> 69 units, burn delivered 26.2 of
26.2) and `FUEL_TO_NOSE` (first COAST tick: pump every tank front-to-back;
in-game test moved 700 LF/800 Ox from aft tanks to the nose in 1.4 s).
**`DRAIN_TO_BURN` is refuted as a default**: 3/3 drained flights crashed
27-31 km short with hypersonic departures (LOG4140, 4142, 4144; `rot-drain-0930`)
against 3/3 defaults reaching the field -- the nose fuel is the hypersonic
ballast (as `DRAIN_RESIDUAL_MACH_MAX`'s note recorded). The combination that
works: keep the reserve, `FUEL_TO_NOSE`, dump at Mach 0.8 (`DRAIN_RESIDUAL`):
valves open at 237-258 m/s, empty in 0.4-1.0 s, 30.6 -> 28.7 t
(`rot-ballast-0930`, LOG4149-4156).

**Arriving high: two mechanisms** (all `qs_shuttle2` exits +1.0..+5.4 km,
`laps=0`, planned path ~16 km on every flight whatever the entry height).
1. *The phantom lap* (LOG4152): 16 km before the gate, lined up, the weave's
   first swing put the tangent point 13 deg past the rollout -- one degree
   outside `hac_turn`'s 12 -- and the plan read a 347 deg / 106 km turn
   against 59 km affordable: *short*. Weave off, wings level, 13 km straight
   to the gate, +5.4 km. `HAC_PAST_BEFORE_GATE_DEG` (new): before the gate,
   a tangent point up to N deg past the rollout costs the run to the gate.
2. *Speed the plan cannot see* (bench): the cone dives 13.5 -> 9.7 km
   gaining 258 -> 314 m/s, then bleeds to 77 m/s by *climbing* (5790 ->
   6100 m) -- 4.3 km of height in kinetic energy that `HAC_ENERGY_BUDGET`
   (off since LOG1941-1952, old craft) would count. Bench `qs_shuttle2_low`
   (new: the twin-fin craft spliced into `qs_shuttle_low`, cone in 2.5 min):
   defaults exit +1591 (LOG4157-4159); energy budget +649..+722 (LOG4160-4162).

**Bench stack** (energy budget + wrap + `DRAIN_RESIDUAL` +
`APPROACH_BANK_BY_ROLL` + `APPROACH_FLARE_FACTOR=1.45`): **3 of 5 intact**
(LOG4166: 31/31, 4.8 m/s sink at 51.7 m/s, wings level; 4167, 4169) against
0 of 7 bench flights without it. The losses (LOG4168, 4170) reached the door
at 80-82 m/s with ~30 m/s of sink and contacted nose-down (pitch -13/-16) at
23-24 m/s -- the cone still exits ~+1 km dry, and the approach spends it as
speed. The slowest door (66.7 m/s, LOG4166) landed best.

**From orbit on the farm the stack did not hold** (`rot-stack-0930`,
LOG4171-4174; `rot-gap-0930`, LOG4177-4179): exits +2.0, +5.3, -1.8, -1.2,
-6.4, +4.3, -1.6 km; cone entries 13.4-19.5 km, all on the 3 km *distance
backstop*, never the glide's own `HAC_ALT_M` 12 km target. 0 intact.

**kspSim for the twin-fin craft.** `makecraft.sh 0 qs_shuttle2
qs_shuttle2_low qs_shuttle2` -> `models/qs_shuttle2.json`,
`qs_shuttle2_low.json` (augment needed an instance restart: stale thruster
transforms). Bench in the sim: exits ~-0.4 km where the game's are +1.0 --
biased, relative use only. From orbit it reproduces the scatter (exits
-3.5..+2.3, 1/6 intact) in **2.4 min for 6 flights**. New:
`kspSim/tools/simarms.sh SAVE K "arm sets" ...` (K flights per arm, all at
once) and `spaceplane/tools/armgroup.py` (groups logs by their own config
line -- concurrent flights confuse the harness's LOG attribution).

Sim screens from orbit (stack = energy budget + wrap + past-gate 60 + drain
residual + fuel to nose + bank by roll + flare factor 1.45):
- stack 2/9 intact; + `HAC_SPEED_PATH` 5/9, then 1/6 on a repeat (n=6 is
  noise here). Weave 75 + Rmin 1000: 1/3. `HAC_LAP_AT_TARGET_SPEED` (new,
  laps priced at the cone's speed): 2/6, three out of height -- overspends.
- **Why the entries scatter: the late glide cannot hold alpha.** LOG4171
  (game): commanded 31-34 deg at Mach 1.6-2.6, flown 13-15 -- less drag,
  arrives 18 km up over the field. The 1.9 t nose ballast costs the elevons
  their alpha authority below Mach 3 (cgProbe's 0.33 of pitch authority).
  **`DRAIN_RESIDUAL_MACH_MAX=2.5`** (dump after the hypersonic regime where
  the ballast is needed): sim 3/6 intact vs 0/6, cone entries **11.9-12.5
  km** (the 12 km target) vs 13.2-14.7, exits **+1.0..+1.5** vs
  -3.1..+3.0 (LOG4228-4239). The cone then tracks `need` within 300-500 m.
  Drained, alpha *overshoots* the command by 10-13 deg at Mach 1-3 (aft CG)
  -- watch it in game. The residual +1.2 km: flown cone L/D 2.3-2.6 in the
  sim against `HAC_LD` 1.86 (game measured 1.55-1.65 wet) -- not refitted
  from sim numbers.

**Game check of the Mach 2.5 dump** (`rot-m25-0930`, LOG4240-4243, round 0
only; 4244-4245 void, killed for swap): both 2.5 flights handed over at
h=12.0 km by the *altitude* trigger, 10.8-12.5 km before the field -- but
then ran out of height (-6.0, -2.2), as did the 0.8 arm (-2.7, -2.0) with
`HAC_SPEED_PATH` on (in the game the speed path runs the cone low; the sim
said the opposite). LOG4241 landed 31/31 at 52 m/s / 4.8 sink, 12 km short.
Drained, the game overshoots alpha by **19 deg** at Mach 1-3 (sim 10-13).

**The CG is wrong both ways**: wet flies under its alpha (long, high),
drained flies over (short, low). Built `DRAIN_RESIDUAL_KEEP_UNITS` /
`DRAIN_RESIDUAL_FINAL_MACH` (two-stage: keep N units as trim at the first
opening) and `DRAIN_TRIM_LOOP` (dump 25 units every 6 s while smoothed
alpha is >2 deg short of command, Mach 3.5..`DRAIN_RESIDUAL_MACH_MAX`; the
vehicle picks the amount). Sim (LOG4245-4269): keep 330 tracks alpha within
1-2 deg; the loop leaves 150-250 units with tracking -1..-6. Outcomes
bimodal **by how the glide hands over**: `HAC_ALT_M` trigger (12 km, 4-6 km
before the field) -> exits -3.4..-4.7, out of height; 3 km distance
backstop (~14 km) -> +1.8..+2.5. The same split `cone_affordable`'s
docstring records from 57 old flights (24/33 out of height on the altitude
trigger).

**`HAC_ENTRY_AFFORDABLE` (sim, LOG4270-4281): stack + affordable 4/4
intact, 2 on the runway, exits -191..+990**; + trim loop 0/4 (arrives
low); trim loop alone 2/4. Game confirmation: `rot-afford-1001`.

**Game, the rest of the night** (all n=2-4 per arm; exits in km):
`rot-afford-1001` wet+affordable +1.9 +3.9 +4.0 -2.7 (0/4), +trim loop -2.1
+1.1 -1.0 -0.8 (1/4); `rot-spend-1001` trim+affordable +3.2 +4.1 -1.3 +6.6
(0/4), + spend authority -1.1 +1.3 -1.8 +1.9 (1/4). `conesum` on these:
HAC_LD median 1.89 (configured 1.86) -- not a calibration error. Sim:
`PREDICT_RESIDUAL_DUMP` null (drained arrivals are short from over-rotation
drag, not mass). Across 46 game flights the entry height follows the glide's
`long=` at handover (on target -> 13.5-16.7 km; long -> 18-21; short ->
12 km early, out of height); corr(h, M1-3 alpha deficit) 0.45, (h, hypersonic
slip ticks) 0.15.

**`rot-ballast2-1001`** (LOG4311-4319): keep 200 units below Mach 3.5 ->
exits +2.08 +2.15 +2.22 +2.35 (one -8.0, entered 22 km short); keep 300 ->
+1.8..+4.4. The first consistent arrival of the session; lost in the
approach at 82-96 m/s. Next session starts there (HANDOFF).

## Session, 2026-10-01 morning: old-craft constants in the shuttle's chain

The user's direction this session: "what if there's a fixed constant in the
code you missed", then "replace constants with curves or some general
solution" -- never re-fit a constant for the shuttle.

**Baseline on a clean box** (`rot-base-1001`, LOG4320-4323, keep-200 stack,
no sims beside it): exits -1.8 / +4.2 / +1.8 / +1.5 km, 0/4 on the runway.
Last night's numbers were not only the sim load.

**Over ~150 twin-fin landings** (game and sim): every exit above +1.3 km
lands long, mostly past the far end; every exit below ~-1 km crashes short;
inside +-0.8 km the flare still starts at 75-95 m/s and the craft breaks
above ~70 m/s at touchdown.

Found, each an old-craft constant or a law that never did what it said:

1. **The cone weave did not fly its angle.** Progress per metre flown was
   0.76-0.80 whatever was commanded (cos 0.55-0.81): reversing +-50 deg at
   45 deg of bank takes ~33 s and the clock reversed every 24.
   `HAC_WEAVE_HELD` (swing = reversal + hold, angle solved for the swing's
   effective ratio). Sim (LOG4327): achieved 0.73 vs commanded 0.73, surplus
   closed +1.4 -> +0.3 km, landed 31/31.
2. **`HAC_LD` 1.86 and `HAC_GATE_LD` 1.35 are the old craft's.** The
   shuttle's cones fly 2.1-2.6 per planned metre; at 1.35 it enters the cone
   high by construction (LOG4321: 14.6 km at 20 km, weave pinned, +4.2 km).
   `HAC_LD_MEASURED` (the table ladder scaled by measured/table L/D at the
   flown alpha), `HAC_AIM_DERIVED` (entry aim from the same wings-level
   ladder). A re-fit arm (`HAC_GATE_LD=2.4;HAC_LD=2.4`, LOG4332-4335 round
   0 of `rot-trim-1001`) is a gain check only, not a candidate value.
3. **The approach speed (`APPROACH_FACTOR` 2.25 x stall = 108 m/s) sits
   where the shuttle's L/D is 3.0, against a final sized at 4.2.** LOG4281
   (landed on the runway) read itself 360-620 m low all the way down yet
   flew alpha 1.1-1.6 at 28-35 m/s of sink to hold 105 m/s, and flared at
   91. `APPROACH_POLAR_SPEED`: the speed whose 1 g glide ratio is the ratio
   still needed to the aim, off the table, fast side of best glide.
4. **The glide's alpha ceiling is 40 deg at every Mach** (`GLIDE_ALPHA_MAX_DEG`,
   raised for hypersonic sink). At Mach 0.6-1.3 the glide commands 34-40
   on a wing whose lift peaks at 32; LOG4334 departed: alpha 86-105, slip
   20-28, bank -153. Alpha tracking below Mach 4 is sd 5-24 deg on every
   flight. The curve already existed, sim-only: `GLIDE_ALPHA_PLATEAU=0.05`
   (~41 hypersonic, ~31 subsonic).
5. **The CG is right at one Mach only.** Keep 200 flew 10-20 deg over the
   command below Mach 2.9 (LOG4317: 20 commanded, 33-41 flown), keep 300
   under. `FUEL_TRIM_TRANSFER` pumps LF/Ox nose<->aft (13 m arm) against the
   interval-mean alpha error, Mach 4 to the residual drain. Verified moving
   fuel in game (LOG4332-4335).

Tools: `spaceplane/tools/gatesave.py` (quicksave a farm flight at an
on-profile cone handover); bench saves `qs_shuttle2_low_{m30,p30,p60}`.
Trap repeated and stopped: sims beside the farm (load 23 on 16 cores) --
four sim flights departed in the late glide; don't.

**The chain batches** (all `qs_shuttle2` from orbit; chain = stack + held
weave + fuel trim + plateau ceiling + measured cone ratio + derived aim +
polar approach speed):
- `rot-chain-1001` round 0 (LOG4336-4339): **3/4 intact** (31/31) at 55-66
  m/s, flare 76-85 m/s, cross at flare <100 m; all short or long on energy
  (-4.9, -1.3, +1.1 km): the cone held 108 m/s *true* at 6-10 km (alpha
  17-22, L/D 1.0-1.5) -- short all the way. `ldk` 0.91-1.15: the table is
  right; the speed law was the problem.
- `rot-chain2-1001` (+`HAC_POLAR_SPEED`, LOG4344-4347): **refuted** -- the
  polar is flat near its top; the target stepped tens of m/s, 75-250 m/s
  phugoid in the cone, flares 87-112.
- `rot-chain3-1001` (+`HAC_SPEED_EAS`, `ALPHA_RATCHET_ON_SWING`,
  LOG4351-4358): **4/8 cone exits within +67..+425 m** of what the approach
  needs (baseline: +-1.5-4 km). The other 4 arrived 10-46 km long: LOG4358
  departed hypersonically at a Mach 5.1 bank reversal (alpha 38.7 -> 17,
  long +0.5 -> +11.7 km) and the swing ratchet, at a 6 deg threshold, then
  lowered the ceiling to 28.7 and kept it long. Threshold now 12 deg
  (`ALPHA_SWING_TOL_DEG`). Of the good exits: LOG4354 (+202) intact but
  floated 3.6 km (the polar approach inverted a 4.2 final to ~140 m/s --
  now one-sided), LOG4351 (+103) flared at 87, broke at 84.
- **Hypersonic alpha excursions >15 deg at Mach 3-5 on most shuttle flights**
  (overnight and today): overshoots to 50-58 without reversal, collapses to
  17-24 in reversals. The airframe's pitch margin at high alpha; the craft
  question stands.
- **`qs_shuttle2_gate` is not a valid bench**: loaded in the air the vehicle
  makes half its table lift from the first tick (elevons saturated
  nose-up, flown alpha below command) and dives; 12/12 identical crashes at
  118-175 m/s (`rot-gate-1001`), LOG4371 the same at 1x. The late-glide
  bench shows the same transient right after loading and recovers.

**Afternoon batches** (each 8 from orbit unless noted):
- `rot-chain4-1001` round 0 (swing threshold 12): LOG4375 exited -55 m, on
  the centreline, crossed the midpoint 840 m up and dove into the runway at
  90 m/s -- the final was aimed at `TOUCHDOWN_AIM_M`=2400, the far
  threshold. -> `TOUCHDOWN_AIM_DERIVED` (zone less flare float; 42 m).
- `rot-chain5-1001` round 0: 3/4 glides 10-36 km off; LOG4379 arrived on
  target and ran out of height -- `HAC_AIM_DERIVED` was aiming at best
  glide (3.57), the edge of the cone's authority. -> aim at the harmonic
  mean of steepest (alpha-limit drag x least efficient weave) and flattest
  (best glide): 1.79 for the shuttle.
- `rot-chain6-1001` (midpoint aim): **3/8 intact** (LOG4385 28/31 at 38 m/s,
  199 m from the midpoint; LOG4387 31/31, +78 m along; LOG4388 31/31) but
  416-2503 m off the centreline. LOG4385: the capture commanded level with
  27 deg of bank still on and the track turned on to +21 deg before the
  wings came level.
- `rot-chain7-1001` (+`APPROACH_ENERGY_EXCESS`, `APPROACH_HEADING_LEAD`):
  **0/8 intact**, touchdowns 1.7-2.6 km short of the midpoint, flares 83-95.
  Energy excess refuted; heading lead unproven (confounded).
- The user pointed out the shuttle already has canards: two Big-S Elevon 1
  at z=0 on the nose adapter, ~12 m ahead of the CoM (I had read the part
  list by name, not position). The Mach 3-5 pitch-ups do not follow
  flap-brake deployments (1/28). User approved a forward-ballast test;
  first form needs no craft edit: `DRAIN_RESERVE_DV_MS=1000` keeps all the
  propellant, `FUEL_TO_NOSE` puts ~8.7 t in the nose adapter
  (`rot-ballast3-1001`, chain6 vs chain6 + ballast, 4 v 4).
- Trap repeated: idle kspSim servers from the morning's sim screens sat
  beside every game batch until ~11:00 (1.6 GB swap).

**Evening: the "pitch-up".** The user asked why the twin-fin craft would
pitch up and not the single-fin one: it doesn't -- paired `rot-shuttle2-0930`
has single-fin 4/6, twin-fin 2/6; all game flights 54% vs 28%. Precursors
over 71 overshoots: RCS valve open (most), residual drain dumping the nose
fuel at Mach 3.5 (16), fuel trim aft, reversals. `RCS_PITCH_BY_AUTHORITY`
(`rot-rcsgate-1001`, 4 v 4): 2/4 still departed vs 1/4 -- null. Read on
LOG4409: sideslip 21-27 deg for 20 s with the flown bank going the wrong way
(+7 -> +107 -> -95 vs -31 commanded), roll damper swings to 178, *then*
alpha 66-88. The pitch-up is the end of a high-alpha lateral departure
(failure 99), common to both Mk3 shuttles. Forward ballast
(`rot-ballast3-1001`, round 0 only): 0/2 departures but commanded alpha
only 18-25 -- a different regime. Not fixed; HANDOFF item 1.

## Session, 2026-10-01 evening: the pitch-up is the pitch thrusters

Mined ~320 shuttle logs (scratch scripts). Bank-reversal roll rate, alpha,
Mach and q did **not** separate slipping reversals from clean ones (168
reversals, ~50% "bad" in every bin). The valve's lag did: reversals where
the RCS opened 6-12 s after the command started moving had median peak slip
15.8 deg vs ~9 (358 reversals) -- `reversal_under_way` needs a 5 deg lead,
which a slow slew never builds (LOG4409).

First departure tick per flight (flown alpha > commanded + 15, M>1.2) splits
into three classes: **A** Mach 4.7-7, slip <5, `rcs on` 0-5 s before, alpha
40 -> 56 in 4 s (LOG4352: commanded 40.3, trimmed 36, `err 5.0` from the
shortfall alone opened the valve, pitch thrusters drove alpha through the
trim limit); **B** residual drain at Mach 2-3.3 (LOG4298-4307 are sim
flights -- thinner game evidence than it looked); **C** lateral, roll damper
swings 130-178, slip 20-70 (most chain6 departures).

- `rot-shortfall-1001` (chain6 vs + `RCS_IGNORE_ALPHA_SHORTFALL`, 8 v 8,
  `23a608af`): class A 3/8 vs 0/8, but the control's three opened on
  reversals (err 7.9-8.9) and LOG4415 (flag) opened on lateral error at
  Mach 7.1 and still pitched 26 -> 53. Intact 3/8 vs 0/8 (LOG4426 +742 m).
  Swap 5.3 GB at the end.
- `rot-pitchoff-1001` (chain6 vs + `RCS_PITCH_OFF_IN_GLIDE`, 8 v 8,
  `b87b111e`): glide departures M>2 **6/8 vs 1/8**. Flag arm flew 2-5 deg
  under command at Mach 2-5 (control within ~1) and arrived at the cone
  +2.4..+34 km long, 19-22 km high; 0 intact. The thrusters were trimming
  beyond the surfaces' limit -- which is where it departs. Swap 10.4 GB.
- `Holdable` learned the command ("learned 36.0" flying 31-34): max while
  saturated, and the *command* on any tick within 2.5 deg. ->
  `HOLDABLE_MEAN`.
- `rot-holdmean-1001` (both pitch-off; vs + `HOLDABLE_MEAN`, `5ea40f73`):
  killed after round 1 for the user's CPU. 4 v 4: handover h 13.5-14.5 km
  vs 16-17; LOG4444 long +125, LOG4449 -1872; LOG4446 +9.8 km; LOG4447
  departed laterally at M4.1. Not a result. LOG4451-4454 void.

## Session, 2026-10-01 night: the roll-rate estimator's slip pulldown

- `rot-holdmean-1002` (chain6 + `RCS_PITCH_OFF_IN_GLIDE` vs the same +
  `HOLDABLE_MEAN`, 8 v 8, `5ea40f73`, LOG4455-4470): **null**. Cone
  handover long ctl -1.5..+17.2 km, HM -15.9..+32.3; 4/8 within 2.1 km in
  each. No intact landing (LOG4469 19/31 parts, +2.1 km). Swap 5 GB.
- Mined it instead (scratch `slip.py`, `rev.py`, `replay.py`). **Every
  flight that slipped past 12 deg did it in the same reversal**: Mach 4.5-4.7,
  q 2800-3800, + -> - bank, flown bank lagging the command 15-30 deg, alpha
  collapsing 35 -> 13-23, then the ceiling ratchets down to ~22 and the glide
  runs low-drag 4-32 km long (LOG4459). The 6 that never slipped all reached
  the cone within +-2.1 km. Loop rate, drain residual, mass, nose fuel and
  instance do not separate them.
- The separator is the bank slew limit (`RollRate.limit()`). Replaying the
  estimator over the logged ticks: the slip tolerance (`BANK_RATE_SLIP_TOL_DEG`
  5) has already pulled the limit from 7-10 to 1.3-2.8 deg/s **in COAST and
  the first seconds of GLIDE, at q 0-150 Pa**, where "sideslip" is the nose
  wandering in vacuum. Bad flights then reach the second reversal at 3.2-4.3
  deg/s (clean 5.1-5.9), take 11-17 s over it instead of 7-9, and finish it
  at q ~3000+. The first reversal (M6.6, q 290) slips ~10 deg in half the
  flights and ~2 in most others regardless of rate -- cause not found.
- `rot-sliptol-1002` (chain6 + pitch-off vs + `BANK_RATE_SLIP_TOL_DEG=0`,
  8 v 8, LOG4471-4486): second reversal at q ~2000 vs ~2800, alpha collapse
  >=16 deg at M3.5-5.2 **4/8 vs 7/8**, cone within 4 km 5/8 vs 3/8 (but
  arriving 14.5-23 km high vs ~12), final |along| median ~4 vs ~9 km,
  >10 km misses 2/8 vs 4/8. No intact landing in either (best TOL0 LOG4485
  13/31 at -3.2 km). Direction consistent, p ~0.3; replicate
  `rot-sliptol2-1002` flown next.
- `rot-sliptol2-1002` (replicate, LOG4487-4502): TOL0 **LOG4498 intact
  31/31, -1036 m along** (first intact shuttle landing since chain6),
  LOG4493 30/31 at +2980, LOG4491 31/31 but splashed 4.4 km off the
  centreline; control none intact on land (two splashed 29/31). Pooled 16 v
  16: final |along| <= 5 km **11/16 vs 6/16**, median ~3 vs ~10 km, >=30
  parts 3 vs 0. Alpha collapse at M3.5-5.2 *not* cured (6/8 vs 7/8): the
  second reversal still slips 13-23 deg at q ~2100. TOL0 arrives at the
  cone high (15-22 km) and long, and lands better anyway. Next: the same
  pair on `qs_plane` (`rot-sliptol-plane-1002`) before any default.
- `rot-sliptol-plane-1002` (`qs_plane` defaults vs + TOL0, 8 v 8,
  LOG4503-4518): **null** -- the old craft's limit sits at the 30 deg/s
  ceiling in every phase either way; handover -0.5..+0.9 km in both; 4/8
  vs 3/8 destroyed. -> **`BANK_RATE_SLIP_TOL_DEG` 5 -> 0 is the default**
  (fingerprint `f1c153ba`). Suite 854 OK.
- `RCS_HOLD_MID_REVERSAL` (built, off; `reversal_under_way` also holds
  while the rate-limited command is > `BANK_RATE_SAT_DEG` from
  `bank_wanted`). `rot-revhold-1002` (chain6 + pitch-off on `925b4729` vs +
  flag, 8 v 8, LOG4519-4534): **null**. Connected (LOG4520 opened at glide
  start on err 1.3), but with the pulldown gone the M4.9 command slews
  12-18 deg/s and the valve already opens within a tick; second-reversal
  slip 7.6-20.8 vs 7.7-17.8, alpha collapse >=16 deg 4/8 vs 2/8. No intact
  landing either arm; both arrive at the cone mostly long and high (+4..+29
  km, h 15-23 km). LOG4521: after a slip-11 reversal at M4.8 the bank pins
  at the 70 cap at q ~5000 and flies -66..+117 against it, slip -38.
- Over the 32 pulldown-off flights, the slew limit going into the second
  reversal against its peak slip: r = -0.30 (limit < 10: 8/15 slip >= 15;
  >= 10: 5/17). Faster is weakly better; not a lever. A q-gated pulldown
  (replayed at q >= 500/1000) gives the same limit as off before that
  reversal -- nothing to gain there.
- Authority (logged): at q 2100 the surfaces give roll 617 / yaw 158 kN m,
  RCS yaw 293; yaw inertia is 22x roll's. The airframe out-rolls its own
  yaw at 35 deg alpha (failure 99). Three mechanisms on this reversal now
  (rate pulldown -- fixed; valve hold -- null; rate -- weak): change the
  method. Candidate: unload alpha through the reversal (body roll's slip
  scales with sin alpha), flown by the propagator too.

## Session, 2026-10-02 morning: the slow reversal (`GLIDE_BANK_SWEEP`)

The user's two ideas, in order. **"Fly lower alpha instead of banking"**:
offline from LOG4498's swept table the shuttle's range falls monotonically
with alpha at every Mach (Mach 7 wings level: 1166 km at 0 deg, 439 at 30,
337 at 45); lower alpha *stretches* the glide (L/D 2.4 at 0, 0.63 at 30), and
alpha already flies to the plateau edge (~38-41), so alpha cannot take the
bank's energy job. **"One huge reversal: bank really slowly the whole time,
steeper early"**: built as `GLIDE_BANK_SWEEP` (off), in three versions.

1. *Continuous sweep, solve on the mean model.* Steers beautifully (one sign
   change, 0.1 deg/s, cross at the gate within 200 m) but the solve sat at the
   70 cap while the vehicle flew -30..+20: sim LOG4543-4550 reached the cone
   +57..+64 km long. A sweep slow *throughout* cannot shed the energy:
   +-70 averages cos 0.77 against the ~0.5 the glide asks for.
2. *Planned slow reversal* (`trajectory.BankPlan`): hold a side, cross at
   ~1 deg/s, hold the other side; the crossing's start
   (`guidance.sweep_start`) and, under way, its rate (`guidance.sweep_rate`)
   are solved for the cross-track at the gate; the range solve flies the same
   plan. Root-finding is a warm-started local bracket + Illinois (a global
   regula falsi stalled 116 km off on the step-shaped curve). Held to the
   gate it spirals subsonically (bank 40-54 at 200-250 m/s, ~5 km turn
   radius): sim cone 1 km low, 50 m/s slow, out of height 7/8. Relay below
   Mach 1 (`GLIDE_BANK_SWEEP_UNTIL_MACH=1`, with the plan flying the relay's
   mean there, `BankPlan.until_mach`): sim 5/8 intact vs base 2/8. Below
   Mach 2 it was 11-16 km long at the cone in every flight -- retired.
3. Farm, interleaved vs the best stack (chain6 + `RCS_PITCH_OFF_IN_GLIDE`):
   `logs/rot-sweep-1002.txt` (fingerprint 9bd29b7a, rate cap 2 deg/s) and
   `logs/rot-sweep2-1002.txt` (8c5a05a9, cap 1 deg/s, fallback after 10
   lost ticks). **Base: sideslip 17-51 deg in all 16 flights.** Sweep: the
   crossing (always Mach ~5.8 -> 3.5-4.5, q 1000 -> 3000-5000) is
   **bimodal -- slip 2.8-3.3 in 4 of 13 (LOG4608, 4624, 4634, 4637), 15-28 in
   9**. In the bad ones slip is already -2..-5.5 at q ~1500, lean ~-20, and
   grows steadily even at 0.5 deg/s; the flown bank stalls, runs backwards,
   then overshoots to 77-92. Not the discriminator: loop rate (all 1.0 s),
   instance, RCS valve state, crossing rate, fuel trim. Weakly: the roll rate
   measured in COAST (clean 8.6-14.9, bad 6.1-10.8 deg/s). Three flights in
   batch 1 crossed at Mach 7 on a first-tick search that ran out of
   evaluations (fixed: `GLIDE_BANK_SWEEP_FALLBACK_TICKS`) and then crossed
   back at 0.3 deg/s from q ~700 -- all three clean (<=6 deg), 25 km long.
   Landing outcome: unresolved at n=8 (within 5 km 5/8 and 4/8 vs base 3/8
   and 2/8; intact 1/16 vs 0/16).

The slow crossing is the first thing that has taken the shuttle through Mach
5-4 with slip under 4 deg, and it does so a third of the time; the rest is a
lateral-directional divergence near wings level at q > 1500 that a slow roll
does not prevent.

**`GLIDE_BANK_SWEEP_CROSS_ALPHA_DEG=25`**, `logs/rot-unload-1002.txt`
(d695195b, sweep v sweep+unload, 8 v 8): **null, and costly.** Four of the
unload arm's five "clean" crossings reached the cone +182..+194 km long --
alpha 25 through the crossing cut the drag the plan was counting on (the
plan does not model the cap) -- so their slip is not comparable. The three
that flew a normal range (LOG4640, 4645, 4650) slipped 18-22 deg, earlier
(Mach ~5). Control arm: clean 2/8 (LOG4649, 4654). The sweep's clean
fraction across three batches is **6/21**; base **0/16**. Roll coupling
through sin(alpha) is not the discriminator at this size of unload.

## Session, 2026-10-02 afternoon: the flare does not flare; the farm gets PyPy

**The flare never reaches its alpha in the game.** Over ~80 game shuttle2
flights the flare commands 9-16 deg and flies 3-4 (LOG4610, 4639, 4647);
doors are entered at 20-40 m/s of sink, touchdowns at 65-107 m/s and 7-25 m/s
of sink. The sim tracks the same commands within ~1 deg, so only the farm
can see it. New `pin=total/assist/err` column (kRPC's `Control.pitch` reads
back the *sum* of game, manual and autopilot input -- decompiled
`PilotAddon.OnFlyByWire`): base flares sit at **+0.13..+0.31 total input**
with a 1-7 deg lag -- kRPC is not saturated, it simply under-commands
(rot-assist*, LOG4700-4720).

**`PITCH_ASSIST` (off): refuted as built.** kRPC *adds* client manual input
to its output, so a manual pitch integrated on the pitch pointing error
(commanded nose . roof) was meant to supply the missing integral. Sim: v1 on
(cmd - signed alpha) wound both integrators to +-1 (LOG4680); v2 on the
pointing error wound up on kRPC's ~1-2 deg attenuation band (LOG4695); v3
with a 2 deg deadband landed 2/2 intact on the low bench vs 0/2 (LOG4696-4699).
Game, 8 v 8 from orbit (`rot-assist-1002` CPython rounds 0-1,
`rot-assist2-1002` PyPy rounds 0-1; fingerprint 29664148/aed153ef, fields
added only): **the assist ran to -0.9..-1.0 (full nose-down) in 6/8**, dove
out of the cone 3.5-6.4 km short, doors at 55-91 m/s of sink. Base 8: cone
exits mostly "out of height" too, 3 landed on land with parts left (LOG4700
523 m from the midpoint, 8 parts; LOG4718 959 m along, 6 parts). The
runaway in the game but not the sim says the sign of manual pitch or of the
roof differs between them -- verify directly before any re-fly (HANDOFF).

**kspSim**: `world.py` now sums manual and autopilot inputs as kRPC does;
`Control.pitch/roll/yaw` getters return the total.

**Farm**: PyPy for the autopilot (`rotfly.sh` `PYPY=1`): propagator 20 s vs
120 s offline, cone ticks 21 -> 8 ms; a round from orbit 8 min vs 20.
Textures stripped from every clone (GameData 2.4 -> 1.1 GB; RSS unchanged
at ~3.6-4.0 GB, the 4 GB is anonymous heap). `-nographics` hangs. Six
instances ran together at 6 GB free, cores 23-49% user, swap 1.5 GB after.
Benches `qs_shuttle2_low` and `qs_shuttle2_gate` are invalid in the game
(the former crashes 6-8 km off the centreline into hills, the latter starts
643 m from the gate).

## Session, 2026-10-02 evening: farm speed, round 643 s -> ~450 s

Farm throughput only; no flight-law change. Six instances, PyPy, one arm
(the afternoon's base arm on `qs_shuttle2`), one flight per instance per
round. The practices are written up in `docs/loopCost.md`.

**Instruments added.** `common/rpccount.py` makes four lines at the end of every
log: kRPC calls and wall-ms per tick by phase, wall seconds per phase
(the real farm cost), and each phase's three slowest ticks with their calls.
The `loop rate` line now gives `pk=`, each phase's worst tick.

**Baseline** (`rot-rpc2`, LOG4733-4738, fresh farm): round 643 s, flights
475-627 s (mean 533). COAST ~150 s at 5x, GLIDE ~145 s at 3.5x, HAC 1.1-1.4x,
DEORBIT ~80 s, DRAIN 18 s.

**Changes, each measured on its own round:**

| commit | change | round |
|---|---|---|
| `RPC_BATCH` | aero-table row = one kRPC request (`common.krpcbatch`); 14 calls 6-7 ms vs 10-45 ms, bit-identical | 643 -> 544 |
| `settle_game` | surface probes wait 0.6 game-s at the governor ceiling, not 0.6 wall-s at 1x (DRAIN 18 -> 6 s) | (same round) |
| `GOVERN_PEAK_SKIP=1`, warp ticks ignored, cost reset per phase | one fuel-scan tick held COAST at 4.4x for 320 game-s; DRAIN's 5 s tick held DEORBIT near 1x for 50 game-s | 548 -> 449 |
| `GOVERN_PEAK_WINDOW_S=60` | peak over the last 60 game-s of the phase | 446, **null** (GLIDE unchanged) |
| `TIMESCALE_QUANT_FRACTION=0.2` | frame quantum 0.2 x the control interval | GLIDE unchanged, **null** for speed |
| surface gravity cached | 2-4 round trips a tick | small |

**Final** (`rot-final-1002`, LOG4781-4792, two rounds, fresh farm but 10 GB
zram at boot): rounds 472 and 435 s; flights 318-438 s (mean 381). That is
**1.42x the throughput** (~34 -> ~48 flights an hour). HAC, APPROACH and FLARE
all held 0.10-0.11 game-s per tick.

**What limits it now is the game, not the autopilot.** In GLIDE the governor asks for
9-19x and the instance delivers ~4.5x at ~23 fps on a 0.2-0.5 s quantum:
~235 physics steps a second, ~4 ms each. The KSP main thread is at 75-99% in
`top -H` whether the instance keeps up or not (Unity frames are uncapped),
so that percentage cannot show saturation; fps against the quantum can
(`logs/farmcpu-1002.txt`, 192 samples). HAC's slow ticks are game-side:
`AvailableControlSurfaceTorque`, `AvailableReactionWheelTorque` and
`MomentOfInertia` taking 40-60 ms on some ticks. DEORBIT's ~80 s includes about
23 game-s at 1x slewing to burn attitude, governed by the 0.8 and 0.5 s solve
ticks before ignition. That is failure 91's protection, left alone.

**Seven instances**: `ksp6` was cloned (matches `base/`; kRPC ports
50112/50113) and booted. Seven booting together saturated the CPU (load
31.5 on 16 threads, instances at 90-215% each). The user stopped it, so it
was never flown, and we are back to six. Six already put 8-13 GB in zram
(~4 GB per instance). That contradicts the afternoon's "6 GB free", which was
measured before the instances had grown.

## Session, 2026-10-02 night -> 10-03: landing from cone saves, working backwards

**Method change: landings measured from in-game cone saves, not from orbit.**
`qs_s2_hac0`-`5` (`saves/`, commit f31fb53): six default `qs_shuttle2`
flights (LOG4793-98) saved by `entrysave.py --alt 13500 --low 11500` just
inside the cone. A flight from one costs ~4-5 min wall; six saves are six
cone-entry states (arrivals -0.2..+5.0 km). Every batch below is
`rotfly.sh` over the six saves x 2 rounds = 12 flights, fresh farm each.
Note the orbital defaults arrived within 2 km on all six (LOG4793-98), where
last session's "best stack" from orbit read +17 km sd 10 (LOG4781-92).

| batch (`logs/rot-*.txt`) | LOGs | change on top of the previous arm | intact / 19-30 parts / lost |
|---|---|---|---|
| hacbase-1002 | 4799-4810 | defaults | 0 / 2 / 7 |
| atrim-1002 | 4811-4822 | `ALPHA_TRIM_LOOP` fixed + `ALPHA_TRIM_IN_HAC`, slip tol 15, max 12 | 1 / 0 / 4 |
| slow-1002 | 4823-4834 | + approach 1.8x, floor 1.6x, door 1.45x stall | 1 / 1 / 3 |
| mid-1002 | 4835-4846 | instead 2.0 / 1.8 / 1.8x | 0 / 0 / 9 |
| bankroll-1002 | 4847-4858 | trim + `APPROACH_BANK_BY_ROLL` (default speeds) | 1 / 2 / 4 |
| ground-1002 | 4859-4870 | + `ROLLOUT_ON_MAIN_CONTACT`, `ROLLOUT_HOLD_TAIL_FRACTION=0.4` | 1 / 0 / 0 |
| kd-1002 | 4871-4882 | + `APPROACH_SPEED_KD=1` | 0 / 3 / 0 |
| trim4-1002 | 4883-4894 | + trim bounded -2..+4 | 1 / 3 / 4 |
| attgt-1002 | 4895-4906 | + `APPROACH_ALPHA_AT_TARGET` | 0 / 2 / 2 |
| stop4k-1003 | 4907-4918 | + `APPROACH_SCURVE_STOP_M=4000` | 1 / 3 / 2 |
| **capture-1003** | 4919-4930 | + `APPROACH_CAPTURE_MARGIN=0.15` | **2 / 2 / 4** |
| decel-1003 | 4931-4942 | + `ATTITUDE_PITCH_DECEL_S` -- **disconnected** (no such kRPC attribute): a replicate | 1 / 2 / 2 |
| oscoff-1003 | 4943-4954 | capture + `ATTITUDE_OSC_MITIGATION_OFF` | 3 / 4 / 1 |
| pfloor-1003 | 4955-4966 | capture + `ATTITUDE_PITCH_AIR_FLOOR_S=1` (cone too) | 0 / 5 / 4 |
| pfloor2-1003 | 4967-4978 | same, approach and flare only | 2 / 0 / 8 |

Findings, in the order they were found:

1. **The cone hands over 1.5-4.4 km high and the approach dives it off**
   (baseline: doors at 650-870 m and 95-126 m/s of sink). kRPC flew -3 deg
   of signed alpha against 2-10 commanded at +0.07 input (LOG4803).
2. **`ALPHA_TRIM_LOOP` was built wrong and never flown**: it integrated
   `(alpha + delta) - achieved`, kRPC's error against the offset command,
   which can only wind up. Fixed to `alpha - achieved` (49f3858). With it in
   the cone and on final the handover surplus fell to +0.2..+0.7 km and
   contact sinks to 1.5-10 m/s in half the flights.
3. **Every intact landing entered the flare fast and high**: LOG4816
   110 m/s / 367 m, LOG4823 102/371, LOG4857 122/523. Doors under ~90 m/s
   could not arrest the sink. The stall-factor speeds were swept and are
   noise-dominated (slow, mid); retired as a method.
4. **The approach flies a ~50 s phugoid** (LOG4848: 81-102-80-121-53 m/s;
   pi sqrt(2) v/g = 45 s at 100 m/s), and the trim loop wound to its bounds
   in phase with it. `APPROACH_SPEED_KD` bunched door speeds (44-78) but
   did not stop the cycle; bounding the trim to -2..+4 and
   `APPROACH_ALPHA_AT_TARGET` (alpha at the target speed, not a speed law)
   smoothed it. Door sinks fell to 16-35 m/s.
5. **Touchdowns after a gentle contact broke on the ground**: the craft's
   tail-strike angle is **11.0 deg**, the rollout held 8, and FLARE flew
   1.4 s on the wheels before KSP said "landed" (LOG4836). Two flags.
6. **Doors were 100-600 m off the centreline**: the capture law limit-cycled
   +-500 m with the flown bank far behind the command (LOG4915). The
   S-turn stop did nothing; `APPROACH_CAPTURE_MARGIN` 0.35 -> 0.15 put 11/12
   doors within +-170 m.
7. **The flare still flies 2-3 deg against 7-12 commanded at a flat +0.24
   input** (LOG4927). This kRPC 0.6.0 build has no `deceleration_time` (the
   flag flew disconnected) but has an oscillation detector with automatic
   notch/bandwidth/feedforward mitigation, latched at level 0.93 on a
   landed vessel. Switched off from the cone on: **tracking unchanged**
   (flare error 6.7 deg vs 5.5), so the 3 intact are noise. A stiffer pitch
   (`ATTITUDE_PITCH_AIR_FLOOR_S=1`, applied 1.5 s): approach error halved
   (rms 6.1 -> 3.3) in the cone-too arm, but the cone's fitted constants
   moved three handovers short; restricted to final and flare it **departed
   4 flights in the flare** (pitch +90) -- refuted.

**Pooled, the capture stack lands 6 intact of 36** (capture, decel, oscoff),
against 0/12 on defaults. The rest are: flares that do not pull up (kRPC
tracking), doors low on energy (77-170 m, 15-35 m/s sink), and departures.

**`FLARE_PITCH_P` -- the first lever with a mechanism and an outcome.**
kRPC sums a client's manual pitch with its own output, so a manual input
proportional to the flare's pitch pointing error (no integrator, nothing to
wind) supplies the authority kRPC's tune leaves out. On the capture stack,
cone saves:

| batch | gain | intact (31/31) | flare alpha error mean / rms |
|---|---|---|---|
| capture, decel, oscoff | 0 | 6 / 36 | 5.5-6.7 / 8.3-9.7 |
| rot-flarep-1003 (LOG4979-90) | 0.04 | 5 / 12 | 4.0 / 6.4 |
| rot-flarep2-1003 (LOG4991-5014) | 0.04 | 5 / 12 | 4.2 / 5.8 |
| rot-flarep2-1003 | 0.08 | 6 / 12 | 2.6 / 4.4 |
| save-flarep3-1003 (LOG5015-38) | 0.08 | 7 / 12 | 4.0 / 11.9 |
| save-flarep3-1003 | 0.12 | 4 / 12 (1 lost) | 4.1 / 8.5 |

0.08 pooled: **13/24 intact**. (rot-flarep2 was unbalanced across saves --
`rotfly.sh` with 12 arms on 6 instances flies only arms 0-8; savefly in the
scratchpad pins save i to instance i and alternates the arm.)

**From orbit** (`rot-orbit-1003`, LOG5039-50, 4 each): `qs_plane` on the
defaults **lost 4/4** (touchdowns 61-65 m/s); `qs_plane` with the stack +
`FLARE_PITCH_P=0.08` put 4/4 down with 18-21/23 parts (doors +17..+30 m off
the centreline, contacts 8 m/s at 44 m/s every time, then ~100 m of drift
in the rollout); `qs_shuttle2` with the stack 0/4 intact -- two arrivals
+5.5 and +11 km (the entry, upstream of tonight), the two near ones 16 and
18/31.

**From orbit at n=8, then promoted** (`rot-orbit2-1003`, LOG5051-74):
`qs_shuttle2` on the old defaults 0 intact, **7/8 destroyed**, touchdowns
60-170 m/s; on the stack + `FLARE_PITCH_P=0.08` 1 intact, 3 with 19-23/31,
**none destroyed**, touchdowns 39-67; `qs_plane` on the stack 3/8 intact,
all eight 20+/23. Promoted all twelve settings (bc2c494, fingerprint
**6640ccdc**); three tests of the replaced laws now pin their configuration,
and `alpha_trim_loop` reads `state` with a getattr (a bare test Autopilot has
none). Verified on the committed defaults with no `--set`
(`rot-newdef-1003`, LOG5075-86, 6 each): shuttle 1 intact + 28, 17, 16/31,
two destroyed (one from a +9.5 km arrival); old craft 2 intact + 21, 20/23,
two broken up to 6-7 parts in the rollout. Old-craft rollouts end 100-200 m
off the centreline. Full suite 865 OK.

## Session, 2026-10-03 evening: the rollout steered the wrong way, and the approach relayed

The user, at the start of the session: the old craft is a wingless flying brick.
A bad result there is the design, not the autopilot. It is a regression check
only, and the shuttle is the target (`spaceplane/CLAUDE.md`, "Priority").

**Every "intact" shuttle landing of the last session was off the runway
sideways.** save-flarep3-1003's 31/31 flights stopped 200-600 m from the
centreline, on a 70 m strip. There were two causes, one after the other.

1. **The nosewheel steered away from the centreline.** `across =
   cross(up, along)` points to the vehicle's *right* in kRPC's left-handed
   body frame. At KSC, facing east, it comes out south. `wheel_steering`
   is +1 to the left, so `-gain * cross` steered further out. It showed as
   sideways speed *growing* while the vehicle slowed (LOG5021: 9 -> 24 m/s).
   **`ROLLOUT_STEER_ACROSS_IS_RIGHT`**, save-steer-1003 (LOG5087-98): the
   new `st=`/`trk=` columns show the track turning back (LOG5090, 5093:
   trk -6 -> +33, xt -103 -> -19) where the old sign turns away (LOG5087:
   trk -7 -> -64). **Default** (fingerprint e06a83a7). `ROLLOUT_STEER_PID`
   (save-steerpid-1003, LOG5099-5110) could not be told apart from the
   default, because touchdowns were already 5-390 m off.
2. **The approach handed the flare a vehicle 60-525 m off the centreline.**
   `APPROACH_CAPTURE_KP` 2 deg per m/s gives the rate loop a 2.9 s lag. The
   shuttle's roll arrives in 5.3 s (time_to_peak), at 5-8 deg/s. The result
   was a relay: +-40 deg alternating, the flown bank overshooting to 61. On
   top of that, the S-turn stopped at a distance, still running ~44 m/s
   sideways (LOG5096, door +525 m).
   **`APPROACH_CAPTURE_LAG_AWARE`**: the gain is set so the loop's lag is
   2 x roll's time_to_peak (0.55 on the shuttle), and the S-turn asks for
   no more sideways rate than `lateral * (t_door - lag) / 2` can take back.
   save-lag-1003 (LOG5111-22): **doors 1-22 m off on 5 of 6** (defaults
   4-473), stops 21-61 m off. **But 4 of 6 overshot 1.8-6.7 km into the
   sea.** The relay had been the approach's only dissipation, and at the
   gentle gain the weave banked 15-25 deg against 45 asked.
   **`APPROACH_SCURVE_FULL_GAIN`** gives the weave back its full gain.
   save-fullgain-1003 (LOG5135-46): **every stop within 73 m of the
   centreline, 9 of 12 on the strip**. Along: +0.9..+1.8 km on most, and
   +4.5..+7.4 on hac1/hac4. One flight landed on the runway with 31/31
   parts (LOG5141, +861/-27).

**The energy comes from the cone.** 10 of 12 cones roll out with +840 to
+3935 m of surplus (conesum, LOG5135-46: the fit would be HAC_LD 1.40 sd
0.15 against 1.86 planned). `HAC_EXIT_SURPLUS_DERIVED` hands the approach
anything less than a lap, and a lap is ~4-7 km of height. The approach
spends height only by banking. `HAC_RADIUS_MIN_M=1000` (save-rmin-1003,
LOG5123-34) changed nothing: still laps=0. The airframe's hold radius at
~100 m/s is ~1.3 km, and a lap there still costs more than the surplus.

**Constants replaced by runtime measurement** (the user's request; all off
by default):
- `FLARE_LEAN_BY_ROLL`: `FLARE_WINGS_LEVEL_M` / `FLARE_BANK_TAPER_M` as
  times, from the measured roll rate plus roll's time_to_peak.
- `FLARE_ALIGN_BY_YAW`: `FLARE_ALIGN_ALT_M` as yaw's time_to_peak (19.7 s
  on the shuttle, against the 3 s its comment assumed).
- `APPROACH_SCURVE_PERIOD_BY_ROLL`: the weave's half-cycle as 2 x the bank
  reversal time.

All three together were not separable at n=6 (save-fullgain-1003 arm 1).
`HAC_LD` is not substituted by the achieved ratio, because that failed on
the old craft (journal 2026-09-21). `HAC_LD_MEASURED` scales the table's
ladder by the measured L/D and is in flight now (save-ldmeas-1003).

**From the cone saves the shuttle never has a speedbrake.** The brake is
measured in vacuum, and the saves start at 13 km ("no brake armed", "no
measured spoiler set"). From orbit it is armed and the approach never
deploys it either (`ab=--` on all 50 lines of LOG5075).

**`HAC_LD_MEASURED`** (save-ldmeas-1003, LOG5147-58) is wired (`ldk`
0.5-0.9) but moves the plan the wrong way: `pld` 1.6-2.7 against the 1.40
that fits. The fit's gap is a tracking factor of 1.3-1.4, not the L/D, so
the exit surplus was unchanged. **The surplus is kinetic.** The cone is
entered at ~270 m/s against a ~100 m/s reference, ~3.2 km of energy height
that the height-only plan never priced. Near the gate it shows as a climb
(LOG5155: 4403 -> 4774 m). **`HAC_ENERGY_BUDGET`** prices it. Two batches,
arms swapped (save-energy-1003 LOG5159-70, save-energy2-1003): exit surplus
median +1200 -> +500 m, **on the runway 6/12 vs 3/12, splashed 2 vs 4**,
hac1 landed both times where the stack alone splashed both. On the old
craft (LOG1941-52, old approach) it was null; the shuttle enters the cone
at three times its reference speed.

**Promoted** (5e30f7a, fingerprint **6e854d9f**):
`APPROACH_CAPTURE_LAG_AWARE`, `APPROACH_SCURVE_FULL_GAIN`,
`HAC_ENERGY_BUDGET`. Two cone tests pinned to `HAC_ENERGY_BUDGET=False`
(they build a state on profile in height at a speed above the reference).
**From orbit on the committed defaults** (rot-newdef2-1003, LOG5183-94):
`qs_plane` **6/6 intact, all on the strip** (across within 27 m, along
+494..+686; last session 2 broken up and 100-200 m off: the steering
sign). `qs_shuttle2`: 1 intact on the runway (+928/+28), 2 splashed long
(+2.2 km), 2 damaged, 1 broken up 1.8 km short. Sideways within 181 m.

**Still long, and the reason is the aim.** `TOUCHDOWN_AIM_M` = 2400 is the
far threshold, so the flare door is ~2.2 km down the runway (LOG5183:
`rwy=2201` at 127 m) and the float runs off the end over falling ground. Its
comment says to move it once the cone spends the surplus.
**`TOUCHDOWN_AIM_DERIVED`** (save-aim-1003, LOG5195-5206) is **refuted**:
0 intact, 2 destroyed, doors 1.7-4.8 km off the centreline at 44-50 m/s.
That is the same mechanism as 400 on 2026-09-25: a near aim leaves too
little final to spend what is left. Defaults in that batch: 2/6 on the
runway. Several shuttle touchdowns are a stall onto the ground at 37-40
m/s and 14-24 m/s of sink, after a float.

## Session, 2026-10-03 night: the long landing is the approach, and the cone's phantom lap

Short session; the user asked to wrap up before the second batch finished.
No defaults changed (fingerprint 6e854d9f).

**`rot-aim-1004`** (LOG5207-5242, shuttle from orbit, 3 arms x 12):
defaults (aim 2400) 3 intact, 1 lost, stops median ~2200 m past the
threshold; `TOUCHDOWN_AIM_M=1200` 3 intact, 1 lost, ~1960;
`TOUCHDOWN_AIM_DERIVED` (computes to 0) 1 intact, 3 lost at 57-76 m/s
touchdown, ~1250. Nothing promoted.

**Last session's refutation of the derived aim was a save artifact.**
`GATE_FROM_APPROACH` re-places the gate at engage (`2000 * 4.2 - aim`), so
from `qs_s2_hac*` an aim of 0 moved the gate 6000 -> 8400 m (LOG5196) under
a cone planned for 6000. Aim, gate and approach-ratio changes have to be
flown from orbit.

**The decomposition** (signed door positions read off the rwy trend; now
logged as `ral=`):
- Float, door to wheels: 1.0-1.3 km, matching `(h + (v^2-v_td^2)/2g) * L/D`
  with L/D ~3 (LOG5183: 133 m, 84.6 -> 37 m/s, predicted ~1.25 km, flew
  1.16-1.24).
- Door relative to the aim: -2.1..+0.9 km (aim 2400), -4.1..+2.1 km
  (aim 0). The approach gets +800..+1260 m from the cone and its weave is
  held to `APPROACH_SCURVE_CROSS_M`=+-300 m. At the shuttle's ~1.2 km turn
  radius that allows ~28 deg of track (LOG5233: `sc=45 sat=1.00`, hdg
  +-15..28). It crossed the threshold 1100 m up and landed 1.9 km past
  the aim.
- The cone's phantom lap: `turn=` alternating 0 / ~347 deg, `need` 40-50
  km, ` short `, `wv=0`, in 13 of 36 flights. Counted with:
  `grep -E '^\[.*\] HAC ' LOG | grep -cE 'turn= *3[0-9][0-9]\.'`.
  `HAC_PATH_WRAP_TO_GATE` and `HAC_PAST_BEFORE_GATE_DEG` (2026-09-30)
  were never separated or promoted.
- Separately, 7 of 36 reach the cone 4-8 km long and exit 6-10 km high.

**`rot-phantom-1003-void`** (LOG5243-5248): defaults against WRAP+PAST=60,
killed after round 0 when the session was wrapped up. 3 per arm, not
read. Re-fly it in full (HANDOFF). `60` is a tolerance, not a fit. The
constant-free form is "any wrap before the gate costs the run to the gate".

Telemetry: `ral=` (2888089), the signed along-runway position.

## Session, 2026-10-03/04 night: the glide's long arrivals are bank-coupled alpha, and the phantom batch

Defaults unchanged (fingerprint 6e854d9f). Two flags added, both off:
`HAC_WRAP_BEFORE_GATE` (4e5af16) and `ALPHA_TRIM_IN_GLIDE` (6c3d68c).

**`rot-phantom-1003`** (LOG5255-5278, shuttle from orbit, 12 per arm):
defaults vs `HAC_PATH_WRAP_TO_GATE` + `HAC_PAST_BEFORE_GATE_DEG=60`. Can't be
read. 9 of 24 flights reached the cone 4-15 km long and ~5 km high, and the
flag arm drew more of them. Intact (31 parts) 2 vs 1 (+1 splashed whole),
lost (0 parts) 0 vs 3, exits "out of height" 4 vs 6. Also, the HANDOFF's
phantom count (`turn= 3xx`) includes real laps flown past the gate after
a high arrival (LOG5265, 5275). Check `distance*cos(angle)` (before or past
the gate) before calling a line a phantom.

**The high arrivals.** The deorbit is identical on every flight (26.2 m/s,
same window). The cone handover splits into two groups, `long` ~+500 at
h ~15 km, or +3..+15 km at h ~19-21 km. Loop rates are clean (1.00 s
everywhere). The split happens below Mach 4 and comes from **bank**. Flights
that arrive long fly |bank| 42-60 between Mach 1.2 and 4, against 20-30 for
the good ones. Pooled over 630 glide ticks at fixed q, the alpha shortfall is
~4-5 deg below 35 deg of bank and ~10 above 50. The pitch input in the long
flights is 0.5-0.7 and rarely saturated, against 0.8-0.93 in the good ones.
So the controller isn't using the authority it has. This is in the code, not
the craft. It amends the 2026-10-01 "pitch trim / CG" diagnosis.

**`rot-glidetrim-1003`** (LOG5279-5314, 18 per arm): defaults vs
`ALPHA_TRIM_IN_GLIDE` (the alpha trim loop extended into GLIDE, bound
`ALPHA_TRIM_GLIDE_MAX_DEG` 10). The trim wound to +6..+9 deg at high bank.
Achieved alpha went from 26-33 to 33-35 there, the same as at low bank, and
the pitch input then saturates. So ~34-35 deg is the real ceiling below
Mach 4. The long tail shrank: long arrivals (>1.5 km) 10/18 averaging
+6.4 km (worst +12.5) became 8/18 averaging +2.6 (worst +3.6). The landing
didn't improve: intact 6 vs 5, lost (0 parts) 0 vs 3. The three lost flights
left the cone 2.3-4 km above the height they needed. Not promoted.

**Refuted on the same data: late reversals as the cause.** Long arrivals
reverse *fewer* times in the last 150 km (5-6 vs 7-13), with the same peak
cross (~3.2 km). This is the "reversal count is a saturation symptom"
finding again. The real difference is that the range solve asks for 45-70
deg of bank because the prediction expects drag at high bank that the
vehicle doesn't make. LOG5308: `long=` holds +500 down to 30 km out, then
climbs to +3.5 km.

**`save-wrap-1004`** (LOG5315-5350, cone saves, 18 per arm): defaults vs
`HAC_WRAP_BEFORE_GATE`. A wash. On the runway (|along from the midpoint| <
1200) 10 vs 9, intact 5 vs 5 (+1 vs +3 splashed whole). The per-save
effect changes sign: hac5 lands ~1 km nearer (-466..-4 vs +885..+1216),
hac1 and hac3 hand over 100-160 m higher and splash +2.1..+2.5 km where
defaults landed. hac0 *gains* a phantom with the flag (LOG5321). That one
is **inside the circle** (`distance <= radius`, the `hac_turn` branch),
which the flag doesn't cover. Three tries on the phantom have now come
back null (WRAP+PAST60 from orbit, swamped; WRAP_BEFORE_GATE from saves, a
wash). Whether the cone reads a phantom matters less than what the
approach does with any surplus. hac4 hands over 5.8 km high and splashes
+6.6..+7.8 km in both arms. Note: the defaults fingerprint is now
**83592105**. Adding the off flags changed the hash, not the behaviour.

**The approach's brake is a disconnected knob.** From orbit it arms (the
measured opposed-flap set) and deploys on "S-turn saturated 100% with
+1253..+4780 m left". The **speed guard stows it on the next tick**:
"speed 108 below target 108" (7 of 7 deployments in rot-glidetrim-1003).
`APPROACH_SPEED_PATH` holds speed *at* the target, so a guard at 1.0x trips
on rounding. Under the speed path, the brake's drag should become a steeper
descent at the held speed, which is the dissipation the approach lacks.
Probe: `AIRBRAKE_SPEED_GUARD=0.9` (rot-brakeguard-1004).

**`rot-brakeguard-1004`** (LOG5351-5386, from orbit, 18 per arm, both arms
`ALPHA_TRIM_IN_GLIDE`): adding `AIRBRAKE_SPEED_GUARD=0.9` doesn't connect
the brake. It now **relays against `AIRBRAKE_SINK_TRACK`**. Out on "S-turn
saturated", and within ~1.5 s the sink is 5-6 m/s past what the approach
wants, so it's stowed. Out again 2 s later (LOG5361: eight cycles from 2.5 km
down to the door). The measured set is a lift spoiler: it buys sink, not
drag. To make it a speedbrake, the alpha has to rise while it's out (lift
held, induced drag up). That's a design job. Arm 1 landed worse (1 intact vs
9), but it also drew more long glide arrivals (11/18 vs 7/18), so it can't
be read cleanly. Not promoted.

Pooled glide arrivals (the brake can't reach the glide). Defaults, 42
flights: 55% arrive >1.5 km long, those average ~7 km. `ALPHA_TRIM_IN_GLIDE`,
54 flights: 48%, averaging ~3 km. Arm 0 here landed 6 intact on land, 5 on
the runway. Confirmation batch: rot-glidetrim2-1004.

**`rot-glidetrim2-1004`** (LOG5387-5422, 18 per arm, paired, fresh farm):
defaults vs `ALPHA_TRIM_IN_GLIDE`. Intact on land **3 vs 6**, on the runway
(|along| < 1200, not lost) **4 vs 8**, lost (0 parts) 4 vs 2, arrivals
>1.5 km long 11 vs 7. Over both paired batches (36 a side): intact on land
5 vs 9, long arrivals 21 vs 15, lost 4 vs 5. **Promoted**, fingerprint
**cdb701a0**. Three geometry tests that call `aim` in GLIDE with a minimal
snapshot are pinned to `ALPHA_TRIM_IN_GLIDE=False`.

## Session, 2026-10-04 afternoon: the brake costs pitch authority, and the shuttle rolls over after a level touchdown

All shuttle, `qs_shuttle2`, from orbit with `rotfly.sh`, 36 flights a batch.
The farm was restarted before every batch; swap still reached 15-17 GB by
each batch's end.

**The approach brake.** On brake-out ticks the pitch input saturates
(+1.00) and the vehicle flies 3-4 deg of alpha against a commanded 11-13.
The measured spoiler set is the main elevons, the pitch control, deployed
20 of their 25 deg. The "lift spoiler" seen on 2026-10-04 morning was the
nose dropping. Added `AIRBRAKE_HOLD_LIFT` (off; e7ed115): while the
spoiler is out, raise alpha until the table's lift covers the verified
set's dClA, scaled to the deploy angle; `hold=` in the log.
- rot-holdlift-1004 (5423-5458), guard 0.9 vs guard 0.9 + HOLD_LIFT: hold
  averaged 1.5 deg. The brake still stowed in a median 1.1 s. A null on
  the mechanism; landings were swamped by four 6-8 km long arrivals in arm 0.
- rot-deploy10-1004 (5459-5494), both HOLD_LIFT + guard 0.9, deploy 20 vs
  10: brake-out ticks read `aoak` 1.3 vs 5.0. More authority left at 10.
  Still stowed in ~1.3 s, now mostly on the speed guard (dec 6-10 m/s^2).
- rot-pkg-1004 (5495-5530), defaults vs guard 0.9 + HOLD_LIFT + deploy 10:
  intact on land 4 vs 4, runway 8/17 vs 5/16. **Null.** The brake is out
  a few seconds a flight; nothing it does can show. Three nulls; the
  method changes next time (split rudder on the tail fins, or guards
  referenced to the braked path).

**The rollover.** Over ~140 orbital flights, level touchdowns (sink 4-10,
38-48 m/s, pitch 0-6, bank < 5) split cleanly: every intact flight stays
within 5 deg of bank for 4 s; the broken ones roll 11-180 deg within ~1 s
and lose the wingtip RCS block, tail fin or elevon. The contact states are
indistinguishable. Touchdown position doesn't explain it (11 of 15 broke
inside the first 1000 m). Brakes don't (rolls at brk 0.00, intact at 1.00).
- rot-gspoiler-1004 (5531-5566), defaults vs ROLLOUT_GROUND_SPOILER=False:
  level touchdowns rolled 4/6 vs 6/8. **Not the spoiler.**
- rot-wheels-1004 (5567-5602), WHEEL_WATCH_S=6 vs + MAIN_WHEEL_FRICTION=1:
  the instrument read every wheel "gone" (it asked for `Wheel.deflated`,
  which kRPC 0.6.0 doesn't have; fixed a8f4927). Friction 1 was worse:
  intact 5/18 vs 12/19. Friction 10 stays.
- rot-wheels2-1004 (5603-5638), defaults + WHEEL_WATCH_S=6: **5 of ~20
  contacts list only one LY-60 main** at contact (LOG5608, 5611, 5620,
  5623, 5627), and those roll toward the missing side. Nothing logs a gear
  being lost earlier (gear parts carry no skin sensor), so whether it
  detached or kRPC just doesn't list it is open. In two-main rollovers
  (LOG5612, 5619, 5626) one main lifts (`g0`), unbroken, as the bank
  builds to 13-20 deg within 1 s. `stress_percentage` reads 0% on every
  wheel every tick, so that column is dead.

Also seen, not chased: ~half of all touchdowns are past the runway's far
end (`TOUCHDOWN_AIM_M` 2400 is the far threshold); ~12 flights stalled in
the flare at 50-97 m, 51-57 m/s, sinking 37-45 (nose first).

## Session, 2026-10-04 evening: it is not a rollover -- the wings come off

Fingerprints e4679374 / db95a67f (two off flags added; defaults unchanged
in behaviour). All from the cone saves `qs_s2_hac0-5` with `savefly.sh`,
`WHEEL_WATCH_S=6;LOG_INTERVAL_UT=0.1` on every arm.

**The "one main gear" lead is closed.** `wheel_watch` now starts at
gear-down and re-reads the wheel list and part count every 0.5 s
(cbed8f0). sav-wheels3-1004 (LOG5639-5674): 3 wheels and 31 parts on
every flight until contact; the "2 wheels, 26 parts" listings are read on
the contact tick itself, after the parts have gone. Nothing is lost
before touchdown.

**What breaks is the wing.** In `saves/qs_s2_hac1.sfs` each LY-60
(`GearMedium`) is attached to a `wingShuttleDelta` at x = +-5.6 m, and
each wing carries elevon 2, elevon 1, the tail fin (on elevon 1) and the
gear: 5 parts a side. "Lost 10 of 31" in 0.4 s (LOG5658) is both wings;
"lost 5" is one. The bank that follows is the fuselage with one wing or
none. The farm's own `ksp<N>/KSP.log` says how: parts **"Exploded!!"**
(a collision over crash tolerance, not a joint break). The first part to
go is usually an RV-105 RCS block, then the outboard elevon, then the
wing; sometimes the wing first. Neither wing is favoured (kept + 10, - 15).

**Two populations, set by the save.**
- hac1/hac3: door at ~75 m/s (hac3 at 100 m with 15 m/s of sink), +7 deg
  of slip through the whole flare on hac1, a float at ~20 m while the
  speed bleeds 47 -> 38, alpha at the tail cap with full pitch input, and
  contact at 37-39 m/s and **8-10 m/s of sink**: broken ~10 of 12.
- hac0/hac5 (and hac2 when it lands): contact at 47-50 m/s, 3-5 m/s,
  pitch +5. Intact 13/14 in the first batch, but only ~half in the next
  two -- gentle touchdowns also lose a wing. hac4 mostly never contacts
  (no `contact:` line) or arrives nose-down.

**Two fixes, both null:**
- `FLARE_SPEED_BUDGET` (sav-budget-1004, LOG5675-5710): raise the
  exponential flare's touchdown sink when the time to the stall floor
  (craft's own landing-mass stall in factor units, 45.5 m/s; deceleration
  measured over the last second) is shorter than the schedule's time to
  the ground. Wired (`td=` rises to 5 from h 28 m), but on hac1/hac3
  contact moved only 37.5 -> 38.5 m/s, sink 10 -> 9. The deceleration
  grows from 1.4 to 3.6 m/s^2 as the speed falls (induced drag, ~1/v^2),
  so the budget is optimistic early, and from a 76 m/s door there isn't
  the energy anyway. **The door speed is the lever, not the flare.**
- `ROLLOUT_RAMP_FROM_ATTITUDE` (sav-ramp-1004, LOG5711-5746): found that
  `ROLLOUT_HOLD_TAIL_FRACTION` caps the rollout alpha *after* the ramp, so
  the first ground tick steps 10-14 -> 4.4 deg (failure 31's step again).
  The flag caps the schedule inside the ramp and starts from the contact
  pitch. Wired (LOG5717 "ramps from the pitch at contact, 5.5 deg"), but
  gentle touchdowns intact 3/8 vs 5/9 on defaults. **Null.** The signed
  alpha (`aoak`, not `aoa`'s unsigned second figure) falls +9 -> -2..-6
  within 0.4 s of contact on every touchdown, with the pitch input at full
  nose-up: the mains are 4.0 m behind the centre of mass, and the contact
  pitches the nose onto its wheel whatever the rollout commands.

Two nulls on the wing loss from the autopilot side. Next is a different
method (HANDOFF).

## Session, 2026-10-04 night / 10-05: the wings come off because of rigid attachment

Cone saves `qs_s2_hac0-5`, `savefly.sh`, fingerprint 1d14ddc8 (all new
flags off).  Instruments added: `GROUND_WATCH_S` (each part's lowest point
above the runway, every tick near contact), `contactsum.py`, `gearProbe.py`,
`shoot.py` (screenshots through kRPC's `SpaceCenter.screenshot`; F1 over
XTEST does not reach the game under Wine), and the **CollisionSpy** plugin
(`testInstances/collisionSpySrc`, on every instance): every collision of the
active vessel's parts, every joint break, and per physics step near the
ground the wing attach joints' force/torque against their break limits
(`jointsum.py`).

**The finding.** With rigid attachment off on the two `wingShuttleDelta`
parts (`saves/qs_s2_rigoff0-5`, two lines per save), **22 of 24 touchdowns
kept both wings** (14/14 gentle, 8/10 hard at 8-10 m/s), against ~8/24 on
the stock saves in the same interleaved batches (sav-rigoff-1005 LOG5932-
5967, sav-rigconf-1005 LOG5968-6003).  Rigid off on every wing part
(`qs_s2_rigwing`) was no better (7/9).

How it was found, and what was wrong on the way (each retracted):
- The geometry: nothing but the wheels reaches the runway at contact
  (GROUND_WATCH, LOG5747; screenshots LOG5815, LOG5852/5854): the wing
  separates in one piece and stands on its own gear.
- Brake at contact: 5/5 intact delayed (sav-root-1004), then 6/9 lost with
  the brakes confirmed off (sav-geom-1004).  Small-n luck.
- Contact state: over 90 gentle contacts sink/pitch/bank/slip/nose-drop
  /time scale all AUC 0.49-0.56; only speed leans, mostly between saves.
- Autostrut on the wings (qs_s2_strut): null.  Suspension spring/damper
  (MAIN_GEAR_SPRING/DAMPER 3/2 and 0.6/0.5): null.
- "The gear body hits the runway" and "the gear collides with the elevon"
  (CollisionSpy): the first has impulse 0 and happens on intact landings
  too; the second happens after the delta has already gone (its children
  become separate debris).  Same-vessel collisions are off (the user).
- Gear moved to the fuselage (qs_s2_gfuse): the wings *still* came off,
  now without their gear -- the touchdown jolt alone breaks a rigid wing
  root.  That pointed at rigid attachment.
- CollisionSpy: both wing joints break 3-5 physics steps after the wheels
  touch, at logged force/torque well under the 1760 limits (the spike is
  inside the breaking step).

Also found: the landing geometry (wheel clearance 1.82 m, tail strike
11.0 deg) is measured with the gear up; gear-down it is 3.72 m and ~25 deg
(engine bell).  `GEAR_GEOMETRY_DEPLOYED` (off) fixes it: contact height
reads 0.1 instead of 2.2.  Unpromoted: flown one batch, wing loss swamped it.

### 2026-10-05 midday: the touchdown point, on the rigoff saves

Scored with `rwysum.py` (on the runway = intact and stopped inside
+-1200 m along, +-35 m across).  All on `SAVE=qs_s2_rigoff`.
- `GEAR_GEOMETRY_DEPLOYED`: null (3/12 vs 4/12, sav-rig3-1005).
- `TOUCHDOWN_AIM_DERIVED`: crashes 5-9 km short on four saves.  Off.
- **`FLARE_ALIGN_ALT_M` 140 -> 30**: the flare aligned the nose from 140 m
  and flew wings level, so the velocity's track error drifted the vehicle
  25-55 m sideways (slip to +8).  Median touchdown |xt| 27 -> 13-20 m over
  48 flights (sav-flarelat/aim/sstop-1005).  `FLARE_LEAN_BY_ROLL` worse.
- Every remaining miss was **long**: the approach flies its profile to the
  aim (the far threshold) and the flare floats 300-500 m past.
  `TOUCHDOWN_AIM_M=1800` fixed hac0/1/2 (9/9) but moved the cone, which ran
  out of height mid-turn on hac5 (crashes 2 km off).  `APPROACH_SCURVE_STOP_M
  =4600` fixed hac1's -100 m capture but landed it 1 km long.
- **`APPROACH_AIM_SHIFT_M` (new) = 1000**: only the approach's aim moves;
  the cone and gate keep `TOUCHDOWN_AIM_M`, and (after rot-orbit-1005 showed
  the weave ending 1 km early on big-surplus arrivals) the S-turn stop stays
  measured from the unshifted aim.  13/15 vs 5/15 on the runway, hac4
  excluded (sav-shift3-1005, LOG6244-6279).
- Both promoted (fingerprint 9fb6cdb8).  Regression (rot-regress-1005): old
  craft 8/8 intact, on runway 4/4 vs 3/4; shuttle from orbit 2/8 vs 2/8.

**From orbit the shuttle still lands ~1 in 8 on the runway** -- the cone
hands over anything from 6.7 km of surplus (LOG6226: arrived over the field
at 20.5 km, rolled out with laps=0 because `HAC_EXIT_SURPLUS_DERIVED` lets
through any surplus under a lap's cost, ~8 km on a 3.9 km circle) to "out of
height" exits that the approach then reads as +530 m surplus (LOG6224: the
cone prices the path via its gate 6 km out while its 9.3 km circle has the
vehicle 1.1 km from the threshold).  Two models of one quantity.

### 2026-10-05 afternoon: the cone from orbit

Three rigoff orbits (`qs_shuttle2{,_inc,_high}_rigoff`), 12 flights an arm,
`rotfly.sh`, scored with `rwysum.py`.  From orbit ~2-4 of 12 land on the
runway whatever the arm; the cone dominates.
- `HAC_EXIT_SURPLUS_DERIVED=False` (fixed 500 m): intact 10/12 vs 5/12 but
  on the runway 1/12 vs 2/12 -- the "out of height" exits it produces land
  2.7-5.7 km long (rot-exit-1005).  Again in rot-surplus-1005: 1/12 vs 3/12.
  Derived stays.
- **`HAC_EXIT_PAST_DEG` 0 -> 25, promoted**: "out of height" exits 0/12 vs
  4/12 (rot-past-1005).  Those exits had overshot the rollout 14-23 deg
  (``turn`` is the angle still to turn: 344 = 16 deg past), read a lap
  owed, widened the circle chasing the receding rollout point (R
  8576 -> 9296 m in 2 s at `HAC_RADIUS_RATE_M_S` 400) and left at the 2 km
  floor beside the field with 0.5-1.2 km of surplus (LOG6224, 6305, 6309,
  6314).  Overshoots beyond `HAC_ROLLOUT_M` (900 m) on 10-14 km circles
  still do it.
- `HAC_RADIUS_MAX_M=8000`: 2/12 vs 4/12, laps still 0 on every flight
  (rot-rmax-1005).  The cone is not short of path; **it arrives at the
  rollout 300-3500 m high on almost every orbital flight** (laps=0, circle
  2-16 km).  It burns less height than it plans -- measure planned against
  flown turning L/D (`conesum.py`) before changing anything else.

## Session, 2026-10-05 evening: the cleanup, and the cone's gap

Ended early (usage limit). HANDOFF.md has the snapshot.

**From orbit, defaults:** rot-hacld-1005 rounds 0-2 3/9 on the runway;
rot-cone-1005 5/12; rot-ias-1005 1/12 (same defaults -- the run-to-run
scatter is that wide). Misses are mostly long; cone exits +0.6..+1.9 km
over what the approach needs (conesum: plan pinned at ~16.4 km, R at the
16 km cap, laps 0, LD flown 1.8-1.9 ~ HAC_LD).

**Arms (all 12/arm over the three rigoff orbits):**
- `HAC_SPEED_EAS + HAC_LD_AT_TARGET + HAC_LD_MEASURED + HAC_WEAVE_HELD`
  (rot-cone-1005): 0/12 vs 5/12. Held at 108 IAS the true speed is 280 m/s
  at 12 km; the energy budget counted excess speed against the target *at
  the current height*, so the ~3 km of height released as TAS falls was
  invisible: rolled out 1.5 km high, overran, lapped, out of height 7 km
  from the gate (LOG6476). Fixed: budget against the gate's target.
- `HAC_IAS_FROM_STALL` + the same chain (rot-ias-1005): 0/12. IAS came out
  133 m/s (V_md gear-up at alpha 0 is 112 on the shuttle's table) -- too
  fast to spend anything.

**The cone's gap** (see HANDOFF): arrival on the centreline 15-19 km out at
14-17 km; laps=0 path ~16-25 km, a lap >= +12.6 km, energy wants 26-30 km.
`HAC_GATE_LD` 1.35 (old craft) puts the entry aim too close; `hac_choose`
always takes the cheapest end/hand. kspSim reproduces it (12 flights,
surplus median +1.9 km sd 1.9): screen cone work there.

**Cleanup** (commits 7fd5096, 8636f9d, c7e0eed, + fix): 139 flags baked,
~11k lines removed, verified by a guidance fingerprint, pylint call checks,
the suite and kspSim flights. Two bugs reached a farm batch first
(rot-clean-1005, all 12 crashed at table-ready: a one-row tuple from the
stall-constant removal) and the sim found a third (`run.airbrake_pair`).
`STALL_SPEED_M_S`/`STALL_CALIBRATION_M_S` removed: unflown on the farm.

Also: panel frame (thin red, rounded); `ENTRY_INTERFACE_AT_AIR` (unflown);
`spaceplane/tools/conexit.py` (cone handover per flight).

## Session, 2026-10-06 (overnight): the cone's handover fixed, the aim moved

Fingerprint at start `01c06c20`, at the checkpoint `d91cd0ad` (defaults
`f31c4cbe` + the off flag `HAC_SHORT_BEST_GLIDE`).

**Old defaults, first farm flights since the stall constants went**
(rot-base-1006, 36 from orbit over the three rigoff orbits): **3/36** intact
on the runway.  Cone handover high on 34/36 (median +974 m, laps 0 on
every flight); the approach lands it 1-5 km long.

**kspSim, by config line** (`conexit.py` now groups by each log's own
`config:` line -- last session's arm mapping was wrong; defaults were 0/4
within +-500, not 3/8).  sim-cone-1006, 15 an arm: defaults +2492 m, 0/15
within +-500; `HAC_AIM_DERIVED` -1156, 4/14, **9 of 14 departed**;
`+ HAC_LAP_AT_TARGET_SPEED` -87, 9/14.

**The departures were the weave.**  The cone's first command is the
weave's first swing, and the phase clock always started on the same one:
-45 deg on every flight, sim and game, whether the log said "turning left"
or "right" (the flip 24 s later is `HAC_WEAVE_PERIOD_S`).  A glide handing
over at +50..+70 deg was commanded a 100-115 deg reversal at alpha ~42,
Mach 0.9 -- failure 99's regime -- and departed 11 of 17 times in the sim
(2 of ~24 otherwise); the farm's 3 departures were all from a positive
glide bank.  `HAC_WEAVE_FIRST_WITH_BANK` starts the clock on the swing
that agrees with the bank being flown (tried both, picked by sign).  Sim
(sim-weave-1006, 10 an arm): the AIM+LAP sd 1616 -> 569, no reversal
departures.  New tool `hacentry.py` (glide bank, first cone commands,
departures, stops, by arm).

**Farm, rot-weave-1006** (18 an arm, interleaved): AIM+LAP+WEAVE_FIRST
handover within +-500 **9/18 vs 1/18**, departures 0 vs 2 -- but runway
4/18 vs 4/18.  A handover on profile still touched down 1.4-2.3 km in:
`TOUCHDOWN_AIM_M` 2400 is the far threshold.

**The aim, rot-aim-1006** (cone flags on, 18 an aim): 2400 **3/18**, 1800
**9/18**, 1200 5/18.  At 1800 every handover between -200 and +630 m
stopped +650..+850 along.  kspSim cannot screen this: the same
configuration stops 1.2-3.5 km *short* there (sim-aim-1006) -- kspSim
gap 7.

**Promoted** (e584a8a): the three cone flags and `TOUCHDOWN_AIM_M` 1800.
rot-newdef-1006: shuttle **19/36** on the runway (rigoff 5/12, inc 4/12,
high 10/12), handover within +-500 23/36.  **Old craft 1/12**: centreline
touchdowns 1.3-1.6 km in, ~2 km of rollout off the end -- 1800 is a ship
bias.  The user: keep it for now, replace it later (spaceplane/CLAUDE.md
"Next" 0, and the constant's comment).

**Refuted, `HAC_SHORT_BEST_GLIDE`** (off): a cone short of height caps its
alpha at the table's best-L/D alpha (LOG6827 flew its deficit at alpha
18-26, L/D 1.2-1.5 against 3.1).  All-or-nothing: median +907 vs +138
(sim-short-1006).  Proportional to the deficit: 8/15 vs 7/16 on the
runway, sd 866 vs 482, high outliers (sim-short2-1006).  Two versions,
both null on the sim; the farm tail it was for (`_inc`) is upstream.

**What is left (rot-newdef-1006):**
- `qs_shuttle2_rigoff` is **bimodal at the glide**: 5/12 reach the cone
  +500..+860 long at ~16 km, 7/12 arrive 3-10 km long at 19-21 km.  Same
  burn (range error 0), same loop rate.  The glide's own predicted miss
  holds +500 until ~31 km / Mach 4.3, then jumps: in the long flights the
  commanded bank has ramped to 66-70 deg (pinned) before a reversal, which
  swings through wings level with no authority left (LOG6845: predicted
  +532 -> +1493 in 15 s before the reversal, +11 km after).  The normal
  flights hold ~30 deg there.  The energy difference arises earlier.
- `_inc`: the cone loses height it planned to have (entry margin
  +700..+1800 -> handover -700..-1000) -- its planned L/D is above what it
  flies on that orbit.
- sim runaways (deorbit ending on the 60 s burn guard, coasting an orbit,
  600 MB logs) held two screens for 100 min; `simarms.sh` now times a
  flight out at 600 s.

## Session, 2026-10-06 morning: the rollout weave, and the cone's gap measured on the cone saves

Fingerprint at start `d91cd0ad`.

**The rollout weaved off the tarmac.**  rot-newdef-1006's losses read
again: 5 of the 17 shuttle failures touched down within 30 m of the
centreline and stopped 40-55 m off it (runway half-width 35).  Every
shuttle rollout weaved: LOG6831 +22 -> -41 -> +51 m with the track
swinging -17..+26 deg, period ~20 s, amplitude growing.  The law was
proportional on cross-track alone acting on a double integrator, which has
no damping.  `ROLLOUT_STEER_LEAD_S` adds the cross-track rate times a lead
time.  Measured on `qs_s2_rigoff2` copied to every instance as
`qs_s2_rwy0-5` (it lands on the runway), sav-lead2-1006, 12 an arm: lead 0
weave spans 24-176 m; lead 3 and 6 close monotonically and stop within 7 m.
**Promoted, lead 3** (fingerprint `6a226a64`).  The five off-axis crashes in
the lead arms were cone exits out of height 1.3-1.8 km from the gate after
roll swings to 179 deg early in the cone, upstream of the flag (LOG7060,
7083, 7089, 7061).  The `qs_s2_rwy*` copies live only in the instances
(regenerate: copy `saves/qs_s2_rigoff2.sfs`).

**The cone saves are no longer cone saves.**  With `HAC_AIM_DERIVED` the
`qs_s2_rigoff0-5` states (glide at 12.7 km, 12.8 km out) arrive ~2 km and
more high: handover +1.6..+5.0 km, **sd < 150 m per save**, landing 3-13
km long (sav-lead-1006, sav-spend-1006, sav-bank-1006).  Not what orbit
delivers now, but a deterministic testbed for a cone with surplus.  Flown
against it, 12 an arm:
- `HAC_FLAP_BRAKE_ON_SURPLUS + _IGNORES_ROLL`: **disconnected** -- the flap
  brake is measured only in vacuum, and from an air-start save it is
  never armed ("glide flap brake: no brake armed").  Null by construction.
- `+ HAC_WEAVE_HELD`: null to worse.
- `HAC_EXIT_LAP_FRACTION` 0.5 (new, off): only `rigoff4` (+4.9 km) lapped,
  and the lap came out -1.6 km and crashed.  A lap costs ~6.5 km of height
  at the gate; the surplus that strands the cone is 1.5-5 km.
- `HAC_WEAVE_MAX_DEG` 70: two saves tipped into laps ending -1.5 km.
- `HAC_BANK_MAX_DEG` 60: every save's handover lower by 0.1-1.2 km
  (rigoff5 +2286 -> +1047, rigoff4 +4948 -> +3898).  Partial; in flight
  from orbit now (rot-bank-1006).

**What the cone cannot do.**  An arrival lined up with the runway is costed
the run to the gate at any radius (no turn to lengthen), and the next
answer is a lap (12.6 km of path at R 2000, ~6.5 km of height).  Between
them only the weave (capped 50 deg, 1/cos = 1.55x) and drag spend, and the
weave stops inside one period of the gate.  LOG6964: the plan held laps=1
and fitting at 10.7 km, dropped it a tick later as the entry's
deceleration spent ~2 km of energy for 1 km of path, pinned the weave at
50 and reached the gate +4.5 km.  `HAC_CHOOSE_BY_ENERGY` (new, off): choose
the end/hand by the planner's fit, not the cheapest -- offline, the other
end from a lined-up arrival costs more than a lap, so it does not fill the
gap; unflown.

**From orbit the same disease** (rot-newdef-1006 re-read): LOG6826, 6850,
6855 left the cone 6-8 km high (rolled out with the plan reading laps=1 and
fitting, because the exit's allowance is a whole lap priced at the current
speed's hold radius -- 9-14 km); the approach dived at alpha 20-25 to 50
m/s and all three flared at 39-41 m/s of sink.  `_inc`: the short cones fly
L/D 1.43-1.55 against 1.69-1.92 for the ones that land, at 1-2 deg more
alpha and twice the sideslip (|slip| 3.8-4.9 vs 1.6-3.3).

## Session, 2026-10-06 afternoon: the glide's split, and nulls on the cone

Fingerprint `63ef8321` (`6a226a64` + off flags).  All from orbit, farm
restarted before every batch.

- rot-bank-1006 (12 an arm): defaults with the rollout lead 22/36 on the
  shuttle (rigoff 4, inc 9, high 9), **no rollout losses**; `HAC_BANK_MAX_DEG`
  60 22/36.  Old craft 0/12 -- **landing 1.2-1.5 km short of the
  threshold**, not long (the previous handoff misread an unsigned
  distance).
- rot-lapt-1006: `HAC_EXIT_LAP_AT_TARGET` (new) null, 4/12 vs 4/12 and
  6/12 vs 7/12; `HAC_SPEED_PATH` on inc **0/12**, all 7.5-12 km short.
- rot-galpha-1006 (24 an arm): `GLIDE_ALPHA_MAX_DEG` 40 / 37 / 34 -> 8, 7,
  0.  Cone entries are trimodal (11.8 / 15-17 / 19-21 km); the caps push
  flights into the 11.8 km (short) mode.
- rot-fbrake-1006 (24): cone flap brake from orbit 9 vs 11 -- it deploys
  only on the cone's last tick (LOG7342: OUT at +7853, in 0.1 s later).
- rot-lapstack-1006 (24): `LAP_AT_TARGET + BANK 60 + RADIUS_MIN 1200`
  handover +2012 sd 2430 -> +66 sd 1249, landings 10 vs 11; half exit low.

**The glide's split (rigoff).**  LOG7092/7101/7129/7161 (long) against
7096/7116/7119/7138: identical to 55 km; at 50-40 km the long ones command
36-37 deg against 39-40 and bank 34-37 against 31; Mach 4.9 vs 4.7 at 36
km; at q ~3500 they achieve 23-28 against commands of 37-44, the ceiling
collapses (`alpha ceiling -> 32`, achieving 26) and the predicted miss
jumps +500 -> +5-11 km at 33-27 km.  The normal flights saturated once at
q ~1600 and planned the rest with a ~37 ceiling.  Over 264 shuttle logs,
the alpha actually held while saturated above Mach 1.5 is 31-34 deg for q
1-4 kPa (30-31 at 45-60 deg of bank), 25-28 above 5 kPa
(`spaceplane/tools/holdmap.py`, new).

## Session, 2026-10-06 evening: the glide planned an alpha it cannot hold

**Asked by the user: is something fundamental missing?**  Yes, upstream of
the cone.  landsum over rot-lapstack-1006 (48 flights): almost every flight
reaching the cone within ~1 km of profile landed intact on the runway;
of ~20 arriving 4-14 km long, 2 did, and every miss is long.
Reliability is the fraction of glides that arrive on profile.

**The measurement** (log columns `cda=` model at the commanded alpha,
`act=` measured, `mdl=` model at the achieved alpha), averaged per altitude
band: at 38-32 km measured/commanded drag is **0.59-0.80 on every long
flight and 0.83-1.07 on every on-profile one**; measured/achieved is
0.96-1.04 on all of them.  The table is right; the alpha the propagator
assumes is not.  `Holdable` learns a ceiling only by saturating, and
tracking 40-44 deg in the first kilopascal leaves denser air unlimited
(offline test: one saturated sample at 1.2 kPa plus tracking -> limit at
5 kPa above 40).

**`HOLDABLE_PRIOR`** (new, off; 65c3fc3 + 592ea75): `tools/holdprior.py`
writes, per Holdable q bin, the median alpha this craft's glide held while
saturated (Mach >= 1.5, >= 100 samples) to `logs/holdprior/<vessel>_<parts>.json`
(shuttle from 360 logs: 36.8 below 1 kPa, 31.9 at 2-4.6, 28.7 at 4.6-6.8);
the propagator uses it wherever the vehicle has no evidence of its own.
v1 (65c3fc3), rot-prior-1006, 8 an arm: rigoff long arrivals 6/8 -> 2/8,
but inc handed over 0.4-1.7 km low and landed short 6/8 -- the prior is
conditioned on saturating, and a flight tracking above it never overrode
it.  v2 (592ea75): tracking above the prior makes the vehicle's own bin.
rot-prior2-1006 (12 an arm): **rigoff arrivals 2.7-10.5 km long 5/12 ->
0/12** (all -700..+582), intact on the runway 6 -> 7; inc 7 -> 9.  Under
the prior rigoff's cone hands over low on every flight (-96..-1007): the
next limit, previously hidden by the long arrivals.

**`HAC_WEAVE_STRAIGHT_ONLY`** (new, off): no weave on the circle, where it
walked the vehicle off it (7 of 12 lap-stack "out of height" exits wove on
the circle, 1 of 12 rolled-out).  rot-straight-1006 (24 an arm, lap
stack): 11 vs 11 intact on the runway; out of height 10 -> 7.  Null.

**rot-prior3-1006 (8 an arm), v2: not promotable.**  high 7/8 -> **4/8**
(arrivals on profile, handover -700..-930 on most, 3-5 km short);
`qs_plane` 1/8 -> 0/8, arrivals **5.5-8.5 km short** (its prior is from
12 logs); rigoff 3/8 -> 2/8 with 2 of 8 long again.  Over v2, rigoff long
arrivals 10/20 -> 2/20 but landings 9/20 each.  The prior fixes where the
glide arrives and not the energy it arrives with: low on every shuttle
orbit.  A q-only median of *saturated* samples is biased low for flights
that would have held more, and `GLIDE_ALPHA_MAX_DEG` 34 (short, 0/24)
already said lower planned alpha means a shorter glide on this vehicle.
Off.

## Session, 2026-10-06 night / 10-07: kRPC's pitch integrator, the offload, and the glide's energy

Fingerprints `c53353bf` -> `8f16d628` (off flags only; no default
changed).  All rigoff unless said, farm restarted before every batch.

**The long mode is a controller fault.**  Over rot-lapstack + rot-straight
(96 flights), at 38-32 km the long arrivals command ~41 deg and achieve
28-32 with 5-10 deg of slip; on-profile ones command ~35 and hold 35-40
with ~3.  At the ~38 km bank reversal the long flights' pitch input drops
1.0 -> 0.4 and stays there 30 s against an 8-12 deg pitch error.  kRPC
0.6's attitude controller keeps its pitch/yaw integrators in a
roll-invariant frame (`kspSim/attitude.py`); the shuttle needs its whole
pitch input to hold 36 deg at 2-4 kPa, mostly integral, and a reversal
swings it.  New column `yrin=` (yaw/roll read-back).

**`GLIDE_PITCH_OFFLOAD`** (new, off): the standing pitch input carried as a
body-frame manual trim (moves toward the summed read-back over the pitch
time_to_peak while the roll is settled; frozen in reversals; bleeds off
after).
- rot-offload-1007 (24 an arm), v1 cap 1.0: long (>1.5 km) 17/24 -> 1/24,
  but 15/24 short (to -17 km), 5 vs 8 landed.  Every short flight's trim
  hit 1.0 (chasing a saturated read-back pins the elevons nose-up, lateral
  departure at Mach 3-4); all 9 at 0.59-0.90 arrived within 1.7 km.
- v2: no learning from a saturated read-back, cap `_MAX`.
  rot-offload2-1007 (16): cap 0.8 13/16 short; cap 0.6 arrivals 16/16
  within 5 km (mean -0.8 sd 1.5; defaults +4.7 sd 3.8), but 2/16 landed:
  handed over ~2 km low.
- `GLIDE_PITCH_OFFLOAD_MIN_MACH` (new): rot-offmach-1007 (16): cap 0.6
  above Mach 3 only -> arrival **+834 sd 696**, 16/16 kept, 7/16 intact on
  the runway (defaults 8/16).  Landing split exactly on the cone's exit
  surplus: > -450 m 8/8 on the runway, < -570 m 8/8 3.5-6.5 km short.
- Repeat in rot-hmach-1007: +870 sd 973, 5/16 (defaults 5/16).
- rot-offorb-1007 (12): inc 4 vs 3, high 8 vs 8 -- neutral on the other
  orbits (inc's arrivals are already +491 sd 170 on defaults).

**`HOLDABLE_BY_MACH`** (new, off): Holdable bins by q alone, so the 36 deg
held at Mach 4 / 3 kPa answered for Mach 0.7 / 3 kPa where the vehicle
holds 18 (LOG7727: "commanded 26, achieving 18.7, learned 38.5").
rot-hmach-1007: 3/16 vs 5/16, same deficits.  Null.

**The deficit is speed.**  Arrivals on the aim point at the same place
(30 km out, h 14-17 km): landing flights 246-270 m/s, short ones 211-229
-- ~1 km of the cone's energy height.  The glide solves position at the
12 km crossing and leaves speed to the alpha history; the cone budgets
height + v^2/2g.  In the cone every flight is pitch-saturated (alpha 18
against 24): the airframe's subsonic trim limit, not the discriminator.

**`GLIDE_ENERGY_AIM`** (new, off, **refuted in the sim by construction**):
the predicted speed at HAC_ALT_M counted as ground.  sim-energy-1007:
`pv=` is 114-147 m/s on every flight, pinned by the propagator's
gate-speed alpha cap; the vehicle hands over at 14-17 km doing 210-270.
It measures the propagator, not the arrival.  Not flown on the farm.

**Instrument `ph=`** (the prediction's own handover state, where the arc
meets the cone's entry test).  Sim: 100 km out it predicts 13.3 km / 155
m/s against an actual 16.9 km / 261 -- ~6 km of energy height pessimistic
until the last ~35 km.  rot-ph-1007 flies it on the farm.

**rot-ph-1007 (24 an arm):** defaults 10/24 intact on the runway, 17/24
kept; offload (0.6, Mach >= 3) 10/24, **24/24 kept**.  Pooled over three
rigoff batches: offload 22/56 landed, 55/56 kept; defaults 23/56, 46/56.
`ph=` in the game: the predicted handover is pinned at 12.9 km of energy
height (the 12 km crossing) until ~60 km out, then correlates with the
actual, biased low.  Landing threshold within the offload arm: energy
height at handover ~18.5 km (h + v^2/2g); the aim promises 12.9;
defaults deliver ~24 because their propagator is pessimistic.

**rot-hacalt-1007 (offload, 16 an arm), `HAC_ALT_M` 12000/13500/15000:**
6 / 4 / 9 landed (inside the noise of 16).  Delivered energy rises (median
~19 / ~19 / ~25) but in every arm a cluster exits the cone "out of height"
at -1.0..-1.36 km, 4-5.7 km from the gate on the 2000 m circle, laps=0 --
the cone's own failure.  Those flights leave the cone at **68-92 m/s**
against 106-115 for the ones that roll out, at 1-3 deg more alpha and a
flight-path L/D of 1.70-1.80 against 2.02-2.19 (back of the drag curve).
LOG7930: speed 221 -> 107 m/s between 10.5 and 8.5 km with the bank 30-40
deg off its command (yaw input -0.46), then alpha driven to ~0 to recover
speed and out of height.

## Session, 2026-10-07 morning: the missing control was the propellant, and it is not yet a gain

Fingerprints `8f16d628` -> `db682413` (new off flags only; no default
changed).  All `qs_shuttle2_rigoff` from orbit, every arm on the
offload (`GLIDE_PITCH_OFFLOAD` 0.6, Mach >= 3), farm restarted before
every batch.  The user asked: "you might be missing a control, not just a
method -- have you been using flaps/spoilers?"

**Flaps/spoilers: armed, never deployed in flight.**  Every orbital
flight measures a split rudder (0.3 m^2 a side) and an opposed-flap
"spoiler" set at startup; in the 40 logs before this session no in-flight
deployment happened (every in-flight brake flag off; the cone was short of
energy, which a brake cannot fix).  Only the ground spoiler runs.

**The cone flies on its pitch stop.**  Over rot-hacalt-1007 (61 flights)
every cone has a standing summed pitch input of +0.55..+0.76, pinned at +1
for 7-49% of ticks, flown L/D 1.3-1.6 -- good and out-of-height flights
alike.

**The control nobody commanded: the propellant.**  After the drain all the
LFO the shuttle keeps (~390 units, ~2 t on the cone saves) is in the nose
tank (`adapterMk3-Size2`, +9.9 m), 13-15 m ahead of four empty tanks.
kRPC's `ResourceTransfer` moves it at ~110 LF units per game second
(bench, `scratchpad/xferbench.py`); the whole orbital load moved the CoM
3.0 m, the cone's load ~0.6 m (~175 kN m at 1 g).

**`PROPELLANT_TRIM`** (new, off): pump fore/aft on the low-passed summed
pitch input (deadband 0.15, tau 5 s, sweep 20 s, at or below Mach 1.5),
tanks found by station.  New column `ptrim=` (standing input / unit-m
moved, + aft).  Smoke from `qs_s2_hac0` (LOG7962): standing input
+0.65 -> +0.15 within seconds, bank tracks its command, no speed collapse
-- and 11.8 km long, intact.

| batch | arms (8 an arm unless said) | intact / intact on runway |
|---|---|---|
| rot-ptrim-1007 | offload / +pump (glide too) / +pump +cone flap brake | 8/3, 3/0, 3/1 |
| rot-ptrim2-1007 | offload / +pump (HAC on) / +pump +brake +arrest-excess | 7/3, 5/0, 5/1 |
| rot-ptrim3-1007 | offload / +pump +`HAC_LD_MEASURED` / `HAC_LD_MEASURED` | 7/3, 5/1, 8/5 |
| rot-ptrim4-1007 | same three | 8/5, 4/0, 7/2 |
| rot-ptrim5-1007 | offload / pump+LDm `HAC_ALT_M` 10500 / 9000 | 8/4, 4/2, 2/2 |
| rot-ptrim6-1007 (12) | offload / `PROPELLANT_TRIM_ON_ENERGY` | 12/5, 8/1 |

Offload pooled over the session: **50/52 intact, 23/52 on the runway**;
every miss short (-1.6..-6.7 km, the cone out of height).

What each step showed:
1. **Pumping in the glide departs.**  An aft CoM at the transonic glide's
   38-41 deg pitched up into a deep stall: LOG7969 flown alpha 48 -> 80
   deg at full nose-down input at Mach 0.8, 29 km short (also LOG7985).
   Now cone-on only; `PROPELLANT_TRIM_IN_GLIDE` opts back in.
2. **Trimmed, the cone flies glide ratio 2.60 against 2.01** (conesum),
   plans with `HAC_LD` 1.86, rolls out 2.2-2.8 km high and never laps;
   the approach floats 7-15 km long or hits at 116-123 m/s.
3. **The flap brake cannot spend it.**  The measured set dumps lift (dClA
   -30): each deployment doubled the sink in 4-8 s with the surplus
   unchanged, and the arrest guard stowed it within 3 s.
   `HAC_FLAP_ARREST_EXCESS` (re-added, off; retired in 7fd5096 leaving
   `nominal = 0`) charges only the sink over the cone's glide: the brake
   then stays out 4-14 s and still spends nothing.
4. **`HAC_LD_MEASURED` alone: null** (5/8 then 2/8 against 3/8, 5/8).
   With the pump it takes the exit surplus to +0.7..+2.7 km, still long.
5. **The handover is not the lever.**  `HAC_ALT_M` 10500/9000 lowered the
   handover to 11-13 km, the cone shortened its path to match (flown 17.8
   km against 27) and exited 2.5-4.8 km high all the same.
6. **The trim drag is this craft's only speedbrake.**  That is why the
   offload misses are all short and the pump's all long.
   `PROPELLANT_TRIM_ON_ENERGY` (new, off): aft only while the cone is
   short of height, forward past `HAC_WEAVE_DEADBAND_M` surplus and from
   the approach on.  Stops -1.6..+3.4 km (one +10.3) -- centred, the first
   pump arm to straddle the runway -- but 8/12 intact and ~1/12 on it,
   losses off the centreline (+179..+790 m across).

**Not adopted.**  The pump is a real, wired, two-sided energy control
(L/D 2.0 <-> 2.6 in the cone) but no variant beats the offload on
landings.  Abandoned *as a trim*; open *as an energy control* -- the next
look is why its centred arrivals lose the centreline (the cone's exit
with fuel forward again just before the approach is a CoM step at the
handover, the shape of failure 31).  Traps: ksp3/ksp4 once each "could
not pause after the load" (LOG7966, LOG8069 suspect; GameData matches
`base/` apart from the shared CollisionSpy/icons).

## Session, 2026-10-07 afternoon/evening: kRPC's twisting integrators, and trim on the canards

Fingerprints `db682413` -> `0d31fb12` (off flags only; no default
changed).  All `qs_shuttle2_rigoff` from orbit, every arm on the offload
(`GLIDE_PITCH_OFFLOAD` 0.6, Mach >= 3), farm restarted before every
batch.  The user: "not just tune things, think about missing things".

**Where the short misses come from.**  The offload's short flights reach
the cone as well as the good ones (+0.4..+0.6 km, same height) and lose it
inside: late in the cone the speed law drops the alpha command to ~0-2 and
the vehicle holds 6-11 deg with a 7-9 deg nose-down pointing error and an
*unsaturated* pitch input of +0.3..+0.5 for 30-90 s (LOG8104 vs LOG8083).
Over 480 rigoff flights of 2026-10-07, that state on >= 20% of cone and
approach ticks: **3/173 on the runway, 145 short**; under 5%: 114/185.
kspSim never shows it (max 6% over 31 sim flights).

**What it is, from kRPC's own diagnostic log** (scratch `apwatch.py`: a
passive observer reading `AutoPilot.current_attitude_error`, oscillation
levels and latches, PID gains, and dumping `AutoPilot.diagnostic_log` --
kRPC 0.6's 50 Hz internals -- when the nose sits off target): no
oscillation mitigation involved (latch 0, level 0), effective target equal
to ours.  The error sits in kRPC's *roll-invariant yaw* axis, and `phi` =
33-35 deg on a wings-level vehicle: kRPC carries that frame by parallel
transport of the nose, and the spiral twists it against the body.  The
nose-up trim its integrators hold (RI yaw -0.62) comes out as body pitch
+0.29 and **body yaw -0.53** -- the pitch and heading error and the 3.6
deg slip -- and unwinds at ~0.002/s.  The glide's long mode (previous
session) is the same fault.

**Three ways of holding the standing input elsewhere, all refuted:**
- `PITCH_P_CONE` (rot-pitchp-1007, 18 an arm): 8/18 vs 6/18.  kRPC wound
  its own output up against the term (LOG8123: P -0.35, total +0.30).
- `CONE_TRIM_HANDOFF` (rot-handoff-1007): carry the summed input as a body
  trim at converged moments and re-engage kRPC (its `Start()` zeroes the
  integrators and re-seats the frame).  4/18 vs 5/19, 2 lost, 5 far misses:
  the first handoff captured +0.6..+0.98 at high q and the vehicle never
  converged again -- a stale trim is the same fault.
- (The glide offload, previous session.)

**Measured, never commanded before:**
- *Cargo bay doors* (scratch `bayprobe.py`, `simulate_aerodynamic_force_at`):
  open adds 10-30% drag and a little lift (L/D 5.5 -> 4.75 at 0 deg, 3.9 ->
  3.4 at 5, 90 m/s).  A real speedbrake.
- *Trim cost* (scratch `tabprobe.py`, 12 km, alpha 5, 90 m/s): full nose-up
  input on every surface 285 kN m for **-50 kN lift (-34%) and +28 kN drag
  (+160%)**; canards deployed +10 deg 60 kN m with **+6 kN lift**, +2 drag.
  That, with the stuck controller, is the cone's L/D 1.2.
- *Canard stall* (`tabprobe2.py`, 120 m/s): +15 deg deployed gives -129 kN
  m at alpha 5, -65 at 18, 0 at 24, **+56 (nose-down) at 30**.  The
  cone's early 22-26 deg command, flown at 15-17 with the input pinned, is
  this ceiling.
- *Surfaces under AtmosphereAutopilot* (decompiled
  `SyncModuleControlSurface`): pitch+roll+yaw share one travel per surface
  (clamp of the sum); a *deployed* surface ignores input entirely (stock
  adds the deploy angle to the control deflection, clamped at 1.5x range).
  kRPC's `ControlSurface.deflection_override` takes a surface off the input
  and sets its deploy angle (-1..1 -> +-37.5 deg here) -- direct per-surface
  control, no more travel or rate than stock; a deploy angle of exactly 0
  freezes the surface (never command 0).
- *Pump vs stuck state*: in last session's pump arms the 35-47% stuck
  cluster of every offload arm is gone (max ~24%).

**`BAY_BRAKE`** v1 (deadband law) with `PROPELLANT_TRIM` +
`HAC_LD_MEASURED` (rot-bay-1007): 4/18 vs 4/18, 4 lost -- the measured
ratio swings ~3 km with the pump changing the trim, doors cycled.  The user:
doors only if necessary, no cycling (memory bay-doors-last-resort).  v2:
open once when the cone's radius is at its cap with no lap or the
approach's S-turns are saturated (`BAY_BRAKE_SATURATED` 0.8 of the window),
shut when spent, never reopened.

**`CANARD_TRIM`** (new, off): the forward mirrored pair is taken off kRPC
with `deflection_override` and integrates the low-passed summed pitch input
to zero (tau 5 s), engaging and learning only settled and under 15 deg of
alpha (first smoke LOG8224 engaged at the cone's entry at alpha 22-30,
learned the entry transient and departed at alpha 86), capped by the
canard's local incidence (deflection + alpha <= 28 deg).  Column `ctrim=`.
LOG8225: standing input +-0.08 from engagement, yaw ~0, flown L/D 3.0-3.7
late cone, 3.7-4.2 approach (was 1.2-2.5); +5.1 km long.

| batch | arms (12 or 18 an arm) | runway / intact |
|---|---|---|
| rot-canard-1007 (12) | offload / +canard / +canard +LD_MEASURED | 4/12, 12 / 5/12, 11 / 6/12, 10 |
| rot-canard2-1007 (18, incidence cap) | offload / +canard +LD_MEASURED | 5/18, 18 / **13/18**, 15 |

Pooled canard + `HAC_LD_MEASURED`: **19/30 against 9/30**; the offload's
-5..-6.5 km short cluster is gone, runway stops within +-930 m along.  The
losses (3 + 2) are approaches handed over ~1 km high that pin the S-turns
for a minute and then dive the excess away into the flare door at 52-93
m/s of sink (LOG8265, 8268, 8282).  rot-canbay-1007 flies the last-resort
doors against that.

**Later the same evening.**  rot-canbay-1007 (canard + `HAC_LD_MEASURED`,
18 an arm): last-resort doors 11/18 vs 12/18 -- null, kept off; pooled
canard + LD_MEASURED on rigoff **31/48**.  rot-xrange-1007
(`GLIDE_BANK_PROBE_DEG`, the user's crossrange question): +bank = left;
at Mach 1, 45 deg held 57-72 km sideways, 60 deg 92-95 km (downrange -34
and -86 km; bought back by a later burn).  `attitudeProbe.py`: inverted at
1 g only 8-47% more drag than upright -- not a brake; upright needs
700-830 kN m nose-down trimmed out at approach speeds.  rot-orbits-1007
(same configuration, 6 each): inc 1/6, ecc 0/6 (arrival +41 km --
upstream), high 2/6; inc/high land long.  The rigoff gain does not carry
over yet.

## Session, 2026-10-07 late (~2200-0015): the user's split-S

The user: "to bleed altitude, maybe you could do a split S. the sharp
turns should kill most of the speed."  Taken as two separate claims.
**A literal split-S was not flown**, and here is why. Inverted, lift and
gravity both point down, so it turns height into *speed*, and speed is
what was destroying the high handovers (LOG8253/8265/8268/8282: flare door
at 52-93 m/s of sink).  The shuttle rolls at 14 deg/s (~13 s to get
inverted) with pitch time_to_peak 22.5 s.  And it reverses the heading: the
turn back costs ~1-2 km, about twice the ~850 m surplus those flights
carried.
**The sharp-turn half is right**: the subsonic polar from ~40k ticks
(LOG820x-829x) reads CdA 18.6 at 2 deg, 49 at 10, 92 at 16 (L/D 3.8 ->
1.3), and the approach flew alpha 1-3 with 40-deg S-turns (1.3 g).

**`APPROACH_SHARP_TURN`** (off; `guidance.sharp_turn`): during the S-turn,
the drag that dissipates the energy surplus by the weave's stop
(`CdA = m g E / s / q`) chooses the alpha (cap 14).  The vertical lift is
what a steady descent at the present speed needs (`sin gamma = D/W`, dive
cap 25 deg, pull toward max(door + 5, v)), and the rest is banked off.  The
lean is predicted over the roll reversal against a 45-deg heading limit and
the 300 m band.  Three references for the vertical lift were flown:
- the speed law's lift: it chases a held 110 m/s against ~90 flown and asks
  for almost none, giving 30-54 deg of bank at 1-4 deg of alpha.  LOG8374
  ran out 1.8 km, the capture turned it 176 deg and it landed backwards
  7 km short;
- one g: flew level and spent speed (101 -> 75 in 10 s).  LOG8377 kept
  1150 m of excess and floated 2.8 km into the sea;
- steady descent: rot-sharp3-1007 (rigoff from orbit, 6 an arm, all intact)
  **6/6 on the runway vs 5/6**.  But it engaged in only 3 flights, for
  4-15 ticks: these handovers were barely high.  Not a measurement.
The first gate (held speed - 5) engaged on 1 tick in 3 flights; it now
fades in above the flare's door speed (rot-sharp0/1-1007).

**The cone saves showed the real gap** (sav-sharp-1007, 23 flights,
canard + LD_MEASURED): `qs_s2_hac0-5` arrive ~13 km long at Mach 1, and
with `CANARD_TRIM` the cone **rolls out lined up 4-6.8 km high** (need
2.2): `hac_exit_surplus` hands over anything short of a whole lap
(`2 pi R / cone_ld` ~8 km at R = 2 km), and the approach can spend ~0.8.
Both arms land +3..+14 km long; runway intact 1/11 vs 0/11.  The low
handovers ("out of height", hac0/hac5) break up in both.

**`HAC_SPIRAL_DUMP`** (off; `Autopilot.hac_spiral`): tight descending 360s
over the gate, lined up at the gate with surplus over the allowance plus one
estimated lap.  Each lap's cost is measured, another lap is flown only if it
leaves the allowance, and it stops mid-lap only while the rest of the lap is
affordable.  The bank is what the alpha cap can hold, with 10% in hand.
- sav-spiral-1007: fixed 60 deg at cap 14 is short of load -- a spiral
  dive, 128 m/s, flare at 138 m/s (LOG8432, destroyed).
- sav-spiral2-1007: sustainable 45 deg, cost **~3.9 km a lap**.  LOG8444
  (hac4) spent 4139 -> 651 m in 0.9 lap, handed over 2.84 km, stopped
  **+62 m** against +14.0 km without it -- broke on touchdown (15 parts).
- sav-spiral3-1008: cap 20 (bank 58, estimate 2.1 km): still >3.5 km a lap,
  sped to 127 m/s, stopped mid-lap misaligned, flare at 133 m/s
  (LOG8450, destroyed).
hac1/hac3 (~1.9 km over need) never start: a lap costs more than they
have.  At the alpha cap the radius is ~v^2/(g k v^2), the same ~1 km at
any speed, so the lap quantum is set by the airframe, not by speed.
**Three engagements, one right handover, two dives: the mechanism is not
ready.**  The method to change next: spend the cone's surplus continuously
(a measured lap at a tight radius planned *before* rollout, or the exit
allowance cut so the cone itself flies the lap), not a quantum after it.

Also seen: touchdown breakups at 64-68 m/s flare entry are common on the
cone saves in every arm (15-25 parts), and the approach often commands
alpha -2 while flying +4.5 (LOG8359) -- the kRPC tracking problem again.

## Session, 2026-10-08 morning (~0700-0800): the cone's exit quantum -- `HAC_GATE_STRETCH`, null

The user: "fix the cone handover".  The last session's finding: on the cone
saves the cone rolls out lined up 2-6.8 km over the approach's need with
`laps=0` (LOG8401 +1.9 km, 8404 +4.1, 8416 +4.6) and lands +3..+14 km.
Read: the cone's continuous spending devices are the radius and the weave;
once both pin (`R=16000`, `wv=50`), anything short of a lap (~5-8 km of
height, `hac_exit_surplus`) is handed to an approach that spends ~0.5.

**Built: `HAC_GATE_STRETCH`** (off; `Autopilot.hac_gate_stretch`,
`guidance.gate_alt`, `end["gate_stretch"]` read by `low_gate`).  While the
plan reads surplus, the rollout moves out along the extended centreline at
`HAC_RADIUS_RATE_M_S`, the gate rising by the approach's glide over the
move (`approach_needed` carries it), so a sub-lap surplus becomes a longer
final.  A step is taken only if the re-planned cone owes no lap, is not
short, does not wrap the turn, and **raises the plan's need**.

- **kspSim screen 1** (sim-stretch-1008, qs_shuttle2, 4 v 4, best config):
  without the last test the stretch grew 2.5-2.9 km at turn 0 and rolled
  out +1.2-1.8 km over need against +0.2-0.65 (LOG8459, 8463, 8465).
  Lined up outside the gate, moving the gate toward the vehicle only swaps
  straight path priced at the cone's ratio (~2.3) for the approach's (4.2):
  surplus appears, nothing is spent.  Fixed by the need-must-rise test.
- **kspSim screen 2** (sim-stretch2-1008): barely engaged.  The sim's cone
  plan read *short* most of the way down (`pld` 0.85-0.97, LOG8473) and
  rolled out high anyway -- the cone's L/D estimate, a separate problem.
- **Farm** (sav-stretch-1008, savefly 4 rounds, `GLIDE_PITCH_OFFLOAD` +
  `CANARD_TRIM` + `HAC_LD_MEASURED`, off v on, 12 each): **null**.
  Engaged on 4 flights (hac1, hac3), stretch 0.2-1.35 km; rollout surplus
  hac3 +2.33/+2.15 km on against +1.83/+1.83 off, hac1 +1.93/+1.22 on
  against +1.87/+1.93.  Landings: hac3 +6.2/+4.8 km on against +3.4/+3.1;
  the rest flew stretch 0 and differ by noise (hac2 on: -267, +621, 31
  parts both).  hac0/hac5 hand over "out of height" in both arms; hac4
  rolls out at turn 336 with +4.1 km in both.

**Why:** the high saves enter the cone **straight in** (LOG8477: turn
1.6 deg, gate 11.3 km, on the centreline outside the gate, surplus
+2.7 km).  The entry aim (`high_gate`, `HAC_AIM_DERIVED`) puts them on the
straight-in profile by design.  There, no gate position adds path -- only a
turn away from the runway does, and the cheapest such turn is the lap.  The
stretch only helps a vehicle that is on the far side of the circle.

This is the third mechanism on spending the cone's sub-lap surplus (sharp
turn, spiral dump, stretch).  Per the root rule, change the method: the
remaining continuous devices are **drag at the cone's speed** (flap brake on
surplus, the bay doors, propellant trim -- each previously off for its own
reasons) and **upstream**: an entry aim that delivers the high arrivals
*onto the circle* rather than onto the straight-in line, where both the
radius and the stretch have authority.  Swap after the batch: 24.6 GB.

## Session, 2026-10-08 afternoon/evening (~1530-1800): the glide's handover energy -- found, fixed upstream, null on landings

The user: "the cone's struggling to manage energy, but it could be that the
glide is giving the cone an energy profile that's impossible to use."

**An unrecorded session before this one** built `GLIDE_CONE_ENERGY` (off;
59f04d7, 21288c5): the cone's wanted entry energy (`cone_entry_energy`, a
low/mid/high band off the cone's ladder) against the predicted entry
energy, as ground added to the miss.  kspSim sim-cone-energy(2)-1008: null.

**What the cone does with an entry** (new scratch tool, per-flight
energy height `h + v^2/2g` at cone entry and rollout, flown path; 24 farm
orbit flights, rot-orbits/rot-sharp1-1007): the cone flies **25.9-28.8 km
of path from every entry**, whatever the radius.  Its one energy control is
how long it stays at the alpha cap: the default cone target is 108 m/s
*true*, ~65 indicated at 8-14 km, held at 20-26 deg of alpha (path per
metre of energy ~1.5); below ~5.5 km alpha drops to 0-8 and it is ~3.1.
`ldk` 0.71 -> 1.44 over one cone (LOG8340).  The plan reads short through
the middle (`pld` 0.87-0.97, R pinned 2 km), then +1-4 km of surplus
appears below 5 km with straight-in geometry left.  Ceiling: ~27 km /
1.5 + ~2.8 at the gate = **~20-21 km of entry energy**; entries above it
landed 4-6 km long (LOG8340, 8348, 8351).

- **`HAC_SPEED_EAS` alone** (kspSim sim-eas-1008, 4 v 4): rollout +570..
  +1191 against +139..+1075; radius and weave pin at 16 km / 35-41 deg and
  it still rolls out high; 2 of 4 broke.  With EAS+`GLIDE_CONE_ENERGY`
  (sim-easce-1008): CE never engaged, EAS lost 2/4 (one at 273 m/s).
- **The ladder band is noise**: low edge 14-35 km for near-identical entry
  states.  **`GLIDE_CONE_CEILING`** (new, off; e464ad1): the band replaced
  by the ceiling -- gate energy + longest no-lap path at the table's L/D
  at `HAC_ALPHA_MAX_DEG`.  Computes 19.2-22.9 km in the sim, matching the
  farm.  But where it engaged (sim-ceil-1008, LOG8531/8533) it was too late:
  **the predicted entry energy `pe` reads 15-17 km from 45 to 25 km of
  altitude against 22 delivered**, catching up only in the last ~60 s at
  70 deg of bank.
- **Why `pe` is low**: below Mach 3 the glide commands ~40 deg and the
  shuttle holds 0.57-0.75 of it (farm, best config: M2-3 0.75/0.58
  rigoff/inc, M1.2-2 0.64/0.57; Mach 3-5 0.84-0.94; sim 0.67-0.70).  The
  10-06 finding again, and the reason inc/high land long: they hold least.
- **`HOLDABLE_PRIOR` by Mach band** (756d9ef; `holdprior.py --by-mach
  --mach 0.8`, regenerated from 138 current-stack farm logs into the
  untracked `logs/holdprior/`; needs `HOLDABLE_BY_MACH`).  The q-only prior
  answered Mach 4 with Mach 2's ceiling (at 4.4 kPa: 32 against 25-27).
  kspSim sim-prior-1008 (3 an arm): entry energy 21.5-21.9 -> **17.6-17.9
  km**, arrivals +2.2..+3.9 -> +0.2..+0.5 km, rollout -0.4 km.
- **Farm, rot-prior/prior2/prior3a/prior3b-1008** (36 an arm, 12 per orbit
  rigoff/inc/high, best config v + prior + BY_MACH + CONE_ENERGY +
  CEILING): entry energy 18.9 -> 17.4 km mean (first 24), entries >20 km
  4/12 -> 1/12 (first batch).  **Landings null: 21/36 v 21/36 on the
  runway**; long 9 v 8, short 5 v 4, hard losses 2 v 3.  Pooled (65): by
  entry energy <16 / 16-18 / 18-20 / >20 km the runway rate is 3/5, 22/31,
  13/20, 2/9; the stop tracks the **rollout surplus** (r 0.74) more than
  the entry energy (0.36) -- correlations across flights, a pointer only.
  **The handover energy is a real cause of the >20 km tail and not the
  main one: the cone mis-spends energy it can absorb.**

Also: rigoff on the best config 6/12 today against 15/19 last night
(rot-sharp1-3 controls) on identical config strings; every change since
78f93e7 is flag-gated (read in full).  Not a missing flag.  Batch 3 ran 2
rounds per farm start (as last night) and swap still reached 20 GB.  The
"65% / 14 of 18" figures are rigoff only; inc/high landed 3/12 last night
and 14/24 today -- the blend is ~50% either day.  The user, mid-session:
touchdown breakups are not a concern unless a strike or a hard arrival.

**Later (~1800-1850): why the cone mis-prices -- the probe sees today's
deflections.**  New log line `cone ladder` at cone entry (per km: reference
speed, 1 g alpha, table L/D, planned).  On the farm (rot-ladder-1008,
rigoff best config, 5/6 on the runway -- today's rigoff base 11/18) the
table at entry prices 2.5 km / 112 m/s at **L/D 0.85-1.72** on five flights
of one craft, where late in the same flights it reads 3.3-3.7 at ~2 deg and
the vehicle measures 3.9.  `Environment.sweep` re-probes the current Mach
row *and one more in turn* with `simulate_aerodynamic_force_at`, which
evaluates the airframe as deflected now: subsonic rows re-probed mid-glide
(elevons saturated, canard trim) describe an airframe never flown there.
(kspSim's ladder at entry read 1.57-1.71 and its `LiftTrim` M0-0.5 1.51x
against the game's 0.54-0.63x: the sim's subsonic lift does not follow the
game -- likely the old "sim cone reads short" gap.)

**`AERO_REFRESH_NEAR_MACH`** (new, off): re-probe only the rows the vehicle
is flying (+ the one below).  rot-near-a/b-1008, 12 an arm over
rigoff/inc/high: **7/12 v 8/12 (base) -- half connected**: 6/12 entries
priced 2.5 km at 3.9-4.5, the rest (entry Mach 0.8-1.0) still contaminated
via the 0.3/0.6 rows; and on every flight `pld` still sinks to ~1.0
mid-cone, because the cone's own 20+ deg upper part re-contaminates the
rows it will fly at low alpha.  Rollout still +0.1..+1.8 km, laps=0.
**The probe gives L/D at the present deflection; the plan needs L/D
trimmed at the alpha it will fly.**  The logs measure that polar
(subsonic, measured L/D: 3.9 at 0-3 deg, 3.3 at 3-6, 2.8 at 6-9, 2.5 at
9-12, ~1.3 at 15+; `polar.py`).  Next: price the ladder from the flown
polar (recomputed from logs like the alpha prior), not the probe.

**Evening (~1850-1940): the flown polar, and the two fixes it suggests.**
`tools/conepolar.py` (new) writes the subsonic polar the cone *flew*
(measured ClA/CdA by achieved alpha, HAC, M<0.9) to untracked
`logs/conepolar/`.  From 207 current-stack farm logs: ClA 69/95/129/**139**/
139/103/94/90/103 at 1/5/9/11/13/15/17/21/25 deg, L/D 3.9 -> 2.4 at 11 ->
**1.2 at 15-17**.  **The flown lift peaks at ~139 m^2 near 12 deg** against
the table's ~300 (210 after `MARGIN`); the cone spends most ticks at 15-17
deg (~32k) -- stalled, braking from 250-350 m/s to its 112 m/s true target
at the 22 deg cap.  (kspSim's subsonic lift is 1.5x the table where the
game's is ~0.6x: this afternoon's sim EAS screen says nothing about the
game.)

rot-polar-1008 (36 flights, 4 per config per orbit; ksp3 refused 3 base
flights; labelled from each log's `config:` line -- `HAC_LD_MEASURED`
contains "EAS"):
- best config 5/9 on the runway (3 long, 1 lost).
- **`HAC_LD_FLOWN_POLAR`** (new, off): the ladder's subsonic rungs off the
  flown polar's rising branch, no `ld_scale`.  Connected: `pld` 2.3-2.5
  from entry (was ~1.0), R pinned 16 km from the start -- and **3/12**:
  rollout -1.3..+2.3 km, laps=0 on every flight.  The surplus it now sees
  falls in **the cone's gap** (10-05): more than the widest no-lap circle
  spends, less than a minimum lap (~12.6 km of path).
- **`HAC_SPEED_EAS`**: **1/12, 4 lost** -- three out of height mid-turn
  (turn 192-197, h 1998 against 3400 needed), others 1.2-2.7 km high.
  Refuted on the farm.
- Both new arms lost two to the approach diving: flare entered at 102-132
  m/s with ~90 m/s of sink (the approach-speed item; base had one today).

**Where this leaves the cone:** its mis-pricing is understood (probe at
present deflection; stalled upper cone) and fixable, but correct pricing
exposes a geometric gap no constant closes.  Fourth attempt on spending a
sub-lap surplus would be the same method again (sharp turn, spiral, stretch
all null): change the method -- e.g. plan the lap at entry, where the
flown polar now says it is needed, or give the entry aim the gap to avoid.

**Night (~2000-2330): the cone's lap gap, the split rudder, the approach
dive.**  All flags below off; best config = offload + canard trim +
measured cone L/D.  Farm restarted every 2 rounds; swap ~20 GB at the end
of each half.
- `HAC_PAST_KEEPS_LINEUP` (new): with the flown polar the cone lined up on
  its widest circle 15 km out, wove at 50, swung 12 deg past the rollout,
  read turn 348 / a 106 km "lap", went short, dropped the weave and flew in
  2.3 km high (LOG8655, 8665).  Keep it lined up unless that lap is
  affordable.  rot-keep-a/b-1008: 3/12 v 2/12 -- swamped: 10 of 24 flights
  entered the flare at 95-133 m/s (the approach dive), ~1 in 5 all day.
- **Online (the user asked):** nothing general-purpose exists in kRPC.
  Closest: kyooni18/KSL (kRPC + C, one shuttle, prior-flight aero evidence
  with airbrake-state separation), giuliodondi/kOS-ShuttleEntrySim (kOS,
  RO).  The Shuttle's TAEM fixes energy *before* the HAC (S-turns, MEP,
  energy-v-range corridor, dynamic-pressure profile), chooses
  overhead/straight-in by energy, and uses a speedbrake throughout.
- **The split rudder** (the user: "we have 2 rudders, you can split
  them"), `tools/splitprobe.py` (new): both Big-S fins at Deploy Angle 38
  -> drag +101% on approach (L/D 4.17 -> 1.89), +52% lower cone (3.28 ->
  2.00), +54% flare; **yaw and roll 0.0**; nose-down ~60 m^3 q; 45 adds
  ~2%.  The old approach "airbrake" only ever moved the flaps -- the pair
  was armed and never deployed.  **Sideslip** (the user's skid idea):
  +68% drag at 10 deg, but one fin makes ~50 m^3 q of yaw (less past 20
  deg) against ~28 m^3 q per degree of weathercock: ~3.5 deg holdable,
  ~+17%, with roll coupling.  The split wins.
- `HAC_SPLIT_BRAKE` (new): angle from the measured L/D factor so the
  remaining path spends the height to the gate.  rot-split-1008 (with
  polar + keep): **7/12 v 5/12, long 1 v 5, high 4/4** -- but two inc
  flights opened it at cone entry still stalled and ran out of height
  13.6 km out (LOG8714, 8719).  `HAC_SPLIT_ON_BRANCH` gates it to the
  flown polar's rising branch with the deceleration done.
- **Rollout** (the user asked): over 161 intact landings the roll was
  already ~380 m median (55 m/s, 3.4 m/s^2; p90 ~870); the 19 "ROLLOUT"
  losses are mostly hard arrivals that die on contact (>= 11).  The
  measured elevon+canard ground spoiler already deploys from orbit (LOG8687:
  6 surfaces, +-14.4/-25); `ROLLOUT_SPLIT_BRAKE` (new) adds the fins.
- **`APPROACH_SINK_GUARD`** (new): the approach's speed law may dive 35 deg
  (~63 m/s of sink) to make speed while the S-turn banks 40 at alpha ~1
  (LOG8699: wanted 36, flew 86 at the door, 2.2 km short).  v1 capped at
  the path to the aim and never fired (handed over high, the path is
  itself steep): **stack 5/12 v 9/12** (rot-stack-1008; a 400 m brake
  reserve also handed over higher -- retired).  v2 caps at 1.5 x the
  approach's design sink: rot-guard-1008 (4 per orbit): **guard + rollout
  brake 0 dives, 0 lost, but 7/12 long** (2/12 on the runway, base 6/12);
  full stack 4/12.
- `APPROACH_SPLIT_ON_GUARD` (new): the split rudder spends the guard's
  surplus as speed.  rot-gsplit-1008: **6/12 v 5/12, long 3 v 6, inc 4/4**;
  but four reached the flare at 44-54 m/s and LOG8801 stalled 2.8 km short
  -> stowed below the flare's target speed (e2a21b2).
- rot-gsplit2-1008 (repeat, speed floor): **guard+split 2/12 v base
  7/12**; the approach brake never deployed (`split=0` on all 12), yet 7
  reached the flare at 38-46 m/s and 6 landed short.  **The slow flares
  are the guard's own**: capping sink floors alpha at the path load, the
  alpha is draggy, the speed bleeds to the stall.  It trades dives for
  stalls.  Pooled over rot-guard / gsplit / gsplit2 it is worse than the
  base; off.  Capping a symptom (sink) without managing the energy only
  moves the failure -- the dives start from high handovers (+0.8-1.1 km
  over need), which is where to act.
- Base (best config) on the three orbits tonight: 9/12, 6/12, 5/12,
  7/12.

## Session, 2026-10-09 morning: the approach's surplus, the cleanup, and the missing half of the polar

**What decides a landing** (135 flights of the best configuration on
2026-10-08, `landsum.py`): the height surplus the cone hands the approach.
Under ~+400 m nearly every flight lands (stops cluster at the brake law's
+850); over ~+800 m it lands 3-9 km long (rollout-to-wheels ratio ~3.5) or
dives into the flare at 86-112 m/s (ratio ~1.9).  Runway 61/135; long 32,
lost 26, off-strip 12 (flare entered 90-330 m off the centreline), short 4.
The dives (LOG8699, 8710, 8583) handed over 1.2-1.4 km high ~6 km out --
a ~28 deg path -- and spiralled at 40 deg of S-turn bank, alpha ~0-8, to
90-110 m/s of sink.

- **`APPROACH_SPLIT_BRAKE`** (new, off): the split rudder opened on final
  to the L/D factor `distance / (height x APPROACH_BEST_LD)` (re-solved
  every tick), the S-turn sized on the braked ratio, sink-capped at 1.5x
  design, speed-gated, stowed before the door.  rot-asb-1009 (17 an arm):
  **8/17 v 8/17**, long 6 -> 3, short 1 -> 3.  Why null: the brake came out
  and stowed within seconds on "slow" or "sink"; LOG8850 had it out 70 s
  and the speed fell 113 -> 81 instead of the path steepening.  **The
  shuttle cannot unload**: at 105 m/s it carries ~1 g at alpha 0 (cla ~75,
  q 4.1 kPa: 310 kN under 294) and `ALPHA_MIN_DEG` is 0, so drag can only
  cost speed.  The root CLAUDE.md had this written down ("the table even
  stops at alpha 0") and nothing had acted on it.
- **`ALPHA_BINS_NEGATIVE`** (built, deleted the same morning): the vacuum
  probe's rows below zero read lift *rising* as alpha falls (M0.3: -6
  16.5, -3 15.6, 0 45.7); the 1 g trim came out at -6, the cone's ladder
  priced its low rungs at -6, and the approach flew -7 into a 90 m/s-sink
  dive (LOG8879, destroyed).  The table cannot be trusted below zero.
- **`APPROACH_ALPHA_MIN_DEG`** (new, off): trim stays on the measured rows;
  the floor only lets the speed loop's proportional term go below zero,
  within 8% of the held speed and only while the brake is out (LOG8880
  flew -2.5 deg handed over 260 m *low*).
- Two bugs the smoke flights caught: the split brake's slew never moved on
  short ticks (it compared each sub-degree step against the *applied*
  angle; LOG8881 held the brake out 113 -> 79 m/s) -- fixed in the cone's
  brake too; and a `HAC_WEAVE_HELD` leftover from the cleanup (below)
  NameError'd every flight in the cone (LOG8877) -- the offline tests do
  not reach `guidance.hac`'s bank cap.  pyflakes now clean of undefined
  names.

**The cleanup** (the user: "remove any ideas for old craft and just outdated
ideas ... maybe old policies are holding us back").  Twenty flags deleted
with their code (~900 lines): `HAC_SPEED_EAS`, `HAC_IAS_FROM_STALL`,
`HAC_SPEED_PATH`, `HAC_SHORT_BEST_GLIDE`, `APPROACH_SINK_GUARD`,
`APPROACH_SPLIT_ON_GUARD`, `APPROACH_SHARP_TURN`, `HAC_SPIRAL_DUMP`,
`HAC_GATE_STRETCH`, `HAC_PAST_KEEPS_LINEUP`, `HAC_EXIT_LAP_AT_TARGET`,
`HAC_WRAP_BEFORE_GATE`, `HAC_CHOOSE_BY_ENERGY`, `HAC_WEAVE_HELD`,
`HAC_WEAVE_STRAIGHT_ONLY`, three cone flap-brake variants,
`PROPELLANT_TRIM*`, `AIRFRAME_DERIVED`, `APPROACH_LD_DERIVED`,
`GLIDE_ENERGY_AIM`.  **The best configuration became the default**
(`GLIDE_PITCH_OFFLOAD` 0.6 above Mach 3, `CANARD_TRIM`, `HAC_LD_MEASURED`;
fingerprint then `dc281ee4`).  New root rule: a flag ends as the default or
deleted.  `spaceplane/CLAUDE.md` rewritten around the shuttle; the old craft
is retired.

**Late morning / afternoon: the speedbrake lands it.**  Gating the split
rudder on the held speed was the null: the shuttle cannot fly below ~+1
deg of alpha on final (commanded -2..-5 flew +0.3..+4: LOG8880, 8887,
8924 -- the canard trim freezes nose-up once the alpha error passes its 3
deg learning tolerance, and kRPC adds ~-0.1 of input), so its steady
glide at the lift it needs is ~80 m/s and the 115 m/s target is never
held; the brake read "slow" and stowed.  `APPROACH_SPEED_KI` (integral on
the speed) wound to its bound against the untracked alpha and released a
phugoid, 133 -> 48 m/s at 900 m (LOG8920, 8924): deleted, with
`APPROACH_ALPHA_MIN_DEG`.  **Gated on 1.45 x stall instead**, at fixed
alpha the brake steepens the path (L/D ~2: ~27 deg at ~80 m/s):

- rot-asb2-1009 (18 an arm): **15/18 v 6/18**, lost 1 v 7, 18/18 intact;
  surpluses of +800..+1667 m stopped at +838..+1673; rollout-to-wheels
  ratio on those ~3.5 -> ~2.2.  Promoted to default.
- rot-conf-1009 (12 an arm): default **9/12**, brake off **2/12**, default +
  the gated cone split brake + flown polar 6/12 (deleted; its history 7/12,
  5/12, 6/12).
- rot-roll-1009: `ROLLOUT_SPLIT_BRAKE` 9/18 v 10/18, lost 5 v 2 (deleted).
- `FLARE_SPLIT_BRAKE` (new, deleted): every long landing of the defaults
  entered the flare at 86-96 m/s, the good ones 61-81; holding the brake
  into the flare while fast engaged on 14/18 and bled to ~79 m/s mid-flare
  but changed neither the touchdown speed nor the stop: 11/18 v 12/18
  (rot-flare-1009).  The float is not a speed problem.
- Pooled defaults (asb2/conf/roll/flare): **46/66 on the runway**; misses
  spread -- long <1.4 km, a few short, off-strip 40-60 m, 2-3 partial
  breakups.

**Breadth** (rot-breadth-1009): single-fin `qs_shuttle` (no mirrored pair,
brake absent) 5/6 stopped on the runway (rigid-attach save: parts lost on
touchdown, known).  **`qs_shuttle2_ecc_rigoff` 1/6**: cone arrival +32..+36
km long, 7-20 km cross, glide pinned at alpha 40 / bank 70 from the
interface.  rot-ecc-1009 (6 v 6): with `HOLDABLE_PRIOR` + `_BY_MACH` the
arrival halves on 4/6 (+7..+15 km) and lands 2/6 v 1/6.  On all 12 the
window at the burn read the max-drag short end at 2,150-2,165 km with the
gate at 2,205, while the glide's own solve read +17 km long at the
interface: **the deorbit window and the glide disagree about the drag the
shuttle makes** -- next session's entry problem.

## Session, 2026-10-09 late afternoon (~1430-1600): the prior deleted, 13 flags deleted, and the eccentric orbit's error is the post-burn flip

**`HOLDABLE_PRIOR` + `HOLDABLE_BY_MACH`, deleted** (rot-prior-1009, 6 an
arm, ecc/inc/high): ecc 1/6 v 2/6 (pooled with rot-ecc-1009: 3/12 v 3/12),
cone arrivals +11..+48 km either way; inc/high 12/12 v 8/12 (pooled with
10-08: 33/48 v 29/48).  Null where it was meant to help.

**Cleanup, second pass** (the root rule: measured flags end as default or
deleted): `PITCH_P_CONE`, `CONE_TRIM_HANDOFF` (refuted 1007),
`HAC_LD_AT_TARGET` (superseded by `HAC_LD_MEASURED`), `GLIDE_CONE_ENERGY` /
`_CEILING` (null in the sim), `ENTRY_INTERFACE_AT_AIR` (never flown),
`GEAR_GEOMETRY_DEPLOYED` (null), `AERO_REFRESH_NEAR_MACH` (null),
`TOUCHDOWN_AIM_DERIVED` (refuted), `BAY_BRAKE` (superseded by the split
rudder), the prior and `tools/holdprior.py`.  Suite 559 pass; smoke
rot-smoke-1009b 4/6 landed, no errors.  Defaults fingerprint `9abe7afd`.

**The eccentric orbit's miss is made in the coast, not the burn.**  The six
default ecc flights of rot-prior-1009 burned identically (34.0-34.1 m/s,
window and aim within 2 km of each other) and arrived +11, +12, +21, +44,
+48, +50 km long.  They separate in the ~90 s after the burn, when the
vehicle swings from retrograde to the entry attitude at full input (pointing
error 100-160 deg) at 69 km, q ~1 Pa.  Vertical speed ~200 s later: -6.8,
-6.8, -6.6, -6.6, -6.4, -6.5 m/s -- in arrival order.  The shallow ones
reach the interface 10-17 s later and ~30 km further along (`rwy=` 540 v 570
km).  Coast energy is the same on all six to ~0.2 m/s; it is the path angle.

**`COAST_TRIM` (built, smoked, deleted)**: re-run the burn's stop test after
the flip settles and close it with RCS translation on the nose axis.  The
first reading said -290..-330 km, -3 m/s owed; its one prograde pulse made
three flights worse (+69..+78 km at the cone).  **`COAST_WATCH_S`** (an
instrument, kept) logged the stop test every 5 s through the coast
(rot-watch-1009): **on the ecc orbit the stop test cannot measure at all**
-- t2g 1830 s against `DEORBIT_MAX_TIME_TO_GO_S` 1500 straight after
cutoff, readable only mid-flip, against an aim of +760 km.  So the ecc burn
was never closed-loop: it ends on the solved dv, and its exit line printed
"range error +0 m" for "unmeasured" (a missing answer looking like a good
one -- fixed: it now says "unmeasured").  inc/high also return no
measurement from the coast (t2g 1580-1790).

Next for ecc: either (a) a correction measured by a quantity that exists on
this orbit (the glide's own solve from the coast state, or the window's
corners -- not the stop test), or (b) take the disturbance out: flip on the
wheels slowly, or before the drag matters, or do the flip *before* the
burn's final tick.

## Session, 2026-10-09 evening (~1630-): the eccentric save was invalid; the approach brake's relay

**`qs_shuttle2_ecc_rigoff` had its periapsis at 69 km, inside the 70 km
atmosphere** (the user caught it).  A retrograde burn at periapsis cannot
lower it, so every "deorbit" from that save was a 34 m/s apoapsis trim and
then ~1800 s of drag decay at q 1-10 Pa -- which is why the burn's stop
test could never measure (t2g > 1500 s) and why a 0.4 m/s vertical-speed
spread from the post-burn flip made 40 km.  **Everything the 10-08/10-09
sessions concluded about "the eccentric orbit" is about that invalid
orbit** (the coast-flip mechanism, `COAST_TRIM`, the window/glide drag
disagreement).  Regenerated at **77 x 135 km** (`savegen.py --source
qs_shuttle2_rigoff --prograde 30 --radial -70`).

Baseline on the new save, defaults `9abe7afd` (rot-ecc2-1009, 3 cycles):
ecc **12/18** stopped within +-1200 m (10/18 also on the strip), against
~1/4 on the old save; rigoff 5/6, inc 5/6, high 5/6.  Pooled 27/36; the
commonest miss is long (9/36: +1.2..+4.8 km).

**Why long** (78 default flights of rot-asb2/conf/roll/flare/prior-1009,
`landsum.py`): 14 long, 4 short.  Short ones handed over with a negative
surplus (-300..-530 m).  Long ones mostly handed over +650..+1200 m high
and crossed the threshold still hundreds of metres up: LOG9013 at 1000 m
over the threshold, touching down past the runway end into the sea.  The
approach's split brake is a relay: out -> path steepens -> sink passes the
cap (1.5 x design) -> **stowed** -> the clean airframe accelerates down the
same steep path (92 -> 118 m/s) -> the speed-path law pulls up to bleed it
(vs -21 -> -5) and the vehicle levels at 1000 m.  13 of 14 long flights
stowed on sink at least once (44 of 60 good ones did too).
**`APPROACH_SPLIT_SINK_FADE`** (new, off): over the cap the brake fades
linearly across `fade x cap` instead of stowing, and does not back off
while faster than the held speed.  Flown in rot-fade-1009 (below).

Housekeeping: journal pruned 315 -> 130 KB (old-craft and single-fin eras
condensed, full text at `f8d63c5`); regenerable files untracked (plugin
DLLs, 11 savegen saves via `saves/derived.txt`, run output); `setup.sh`,
GPL-3.0-or-later, Code of Conduct, CONTRIBUTING.md.
