# kspSim journal

## 2026-09-23 — built, first validation

Asked for: a physics-only KSP that can share resources, tested against the
real game. Built as a kRPC-compatible server so the autopilots fly it
unmodified.

Measurements (all against the live game):

- **Call surface**: a recording proxy on one qs_plane flight (LOG3079,
  trace `logs/kspsim/trace_plane.jsonl`) — ~100 distinct procedures, three
  frames (body, non-rotating, vessel). `Flight_SimulateAerodynamicForceAt`
  21k calls, `get_UT` 7k, `Resources_Amount` 14k.
- **Atmosphere**: 180 points (lat, lon, alt) — temperature 2.6e-5 K,
  density 1.4e-6 relative. The sun term uses the vertical projected on the
  equator and led 45°; a plain dot product with the vertical was 5 K out.
- **Frames**: rotating→inertial = rotation about +y by `rotation_angle`;
  position and velocity round-trip exactly; body spin (0,-ω,0).
- **Drain valve**: 8 t vented in ~10 s gave 2.95 m/s² at 13 t = 39 kN =
  800 kg/s × 5 s × g0 (`drainForceISP = 5`, ResourcesGeneric.cfg); it spins
  qs_plane to 6 rad/s in pitch.
- **Control surfaces**: pitch/yaw/roll input deflects nothing in orbit.
  In the air (qs_cone): pitch+1 & roll+1 together = 45% of the sum of each
  alone. Solo per-surface responses sum to the all-surfaces response within
  0.7%; weights are exactly -1/0/+1; per-surface clamp predicts the unseen
  combined input to 2.2% (plane) / 0.5% (shuttle). `deflection_override`
  tables are on a different (deploy-angle) scale — the override-based fit
  was 44% out and was abandoned.
- **Actuator**: pitch step, oracle sampled per frame: linear ramp, full
  swing 0.46 s.
- **First sim flights of qs_plane** (LOG3080-3090): after UI/stub fixes the
  whole chain flies. Against LOG3079: deorbit solved 32 / delivered 31.6
  (both), cutoff drift radial -12.76 (both), COAST→GLIDE 39916 vs 39921,
  first alpha ceiling q 3452 vs 3721 achieving 25.8 vs 25.9, GLIDE→HAC
  long +2801 vs -1074. Landings: game +715 along; sim +0.6..+3.6 km over 9
  flights, 2 into the water past the east end.
- **Without surface tables** (first flight, LOG3081) the sim lost alpha at
  high q and diverged — the reason the control work above was needed.
- **Speed**: 122 s wall per qs_plane sim flight (physics 0.2 ms/step);
  6 parallel in 133 s. The game ran the same flight at ~1.1x (COAST tick
  262 ms of work vs 8 ms in the sim) — about 25 min.
- **Booster**: flies end to end in 19 s wall. Game reference LOG3094-3097
  (landed 85, 45, 973, 27 m). First sim run started 2.5 km high because
  the probe let air saves run 3+ s before pausing — fixed, not re-flown.
- **Thermal**: Physics.cfg gives KSP's shock-temperature and convective
  coefficient formulas; saves have ReentryHeatScale 1.2. One-node per-part
  fit from LOG3079's skin streams: rms 70-130 K, peaks ~150 K low (nose
  1426 K game vs 1317 fit). Not yet in the sim.

Abandoned approaches: per-axis control tables with superposition (119%
out on combined input); per-surface override tables with a fitted mixing
matrix (44%); least-squares on finite-difference dT/dt (stream staleness
made coefficients negative).

## 2026-09-24 — fidelity harness; eight physics defects found and fixed

Method (new, generic, the measuring instrument from here on):
`tools/flighttest.py` flies a scripted open-loop input schedule on a save in
the game at 1x and records **every physics frame** (0.02 s) -- state in the
non-rotating frame, the inputs the vessel received, the aero force.
`tools/fidelity.py` replays a recording through `world.py` in-process:
(1) *one step*: the sim put in the game's exact state, its wrench against the
game's (from the change in velocity and rate between frames); (2)
*restarts*: from the game's state every 1 s, fly the recorded inputs for
0.5-10 s, report position/attitude/rate error. `tools/fidsum.py` gives one
line per recording. `tools/oracle.py` asks the game's
`simulate_aerodynamic_wrench_at` about a recording's states (separates "the
table is wrong" from "the oracle is not the physics"). 38 recordings in
`logs/kspsim/ft/` (g_<save>_<script>.jsonl): qs_cone, qs_entry, qs_plane,
qs_shuttle, qs_shuttle_cone, qs_shuttle_final, quicksave x free / pitch /
roll / yaw / mix / gear / brakes / throttle / inertia / rcs.

Found and fixed, each measured:

- **Reporting offset**: `aerodynamic_force` in frame i+1 is the force from
  frame i's state (normal-force rms 3.4 kN -> 0.7 kN aligned). Convention,
  not physics; the harness aligns it.
- **Inertia**: kRPC's `inertia_tensor` puts each part's mass at its
  transform (a wing's root). Wheels-only steps in orbit: true roll inertia
  1.1433x kRPC's on the shuttle, 1.025x on the old plane; computing it at
  each part's centre of mass predicts 1.1435x / 1.023x (and the products).
  `world.inertia_at_part_coms`. Shuttle 5 s attitude error, wheel steps:
  10 deg -> 0.12 deg.
- **Reaction wheels lag**: torque is Lerp(input, command, 30 * dt) --
  0.60, 0.84, 0.94 of full on successive frames. It had also inflated the
  first inertia fit by 1.7%.
- **Surface rate**: AtmosphereAutopilot's SyncModuleControlSurface moves
  every surface at 2.0/s (best of 1.2-4.0 on three craft; was 2.2).
- **RCS was a single torque number** (kRPC's `available_rcs_torque` x input,
  2-5x too strong). Now per nozzle, KSP's law: level = max(e . unit(w x p) |w|,
  0) + max(e . l, 0), clamped. The lever enters as a direction only
  (with p/|p| roll was 5.6x weak on the shuttle, 1.5x on the plane). Nozzle
  geometry had been recorded wrong (every nozzle along the part's axis --
  the probe's fallback when `thrust_direction` threw after a load); and
  ReStock's RV-105 lists all 18 nozzles of its five variants: the live ones
  come from the save's variant, the config's GAMEOBJECTS and the `.mu`
  hierarchy (`partcfg.Variants`). Shuttle forward 1.0: game 7875 N, sim
  7877; roll 0.5: -15367 / -15366 N m; mixed input -28471 / -28634.
  Shuttle 5 s attitude error under RCS: 29.7 deg -> 0.14.
- **Gimbal was a torque only**: now the thrust vector deflects (lateral
  force 4316 N game / 4318 sim), lerps at `gimbalResponseSpeed` (AA's patch
  turns it on for every gimbal), and **swings about the gimbal pivot** --
  `Thruster.gimbal_position`, -1.78 m on qs_plane's LV-T91, exactly the
  lever the game's torque/lateral-force ratio gave (nozzle -3.50). Plane
  gimbal torque -12270 / -12249 N m; 5 s attitude 36 deg -> 0.8.
- **Integration**: PhysX-order (position from the new velocity) sank 0.37 m
  in 5 s against the game in orbit; the mean of old and new velocity
  matches to 3 mm.
- **Pseudo-Reynolds drag**: each Mach probed at one altitude bakes in that
  altitude's DRAG_PSEUDOREYNOLDS multiplier (qs_shuttle_cone flies M0.75 at
  10 km, x1.00, on a table taken at 4 km, x0.86: axial force 15% light).
  The probe now takes the base table at a second density per Mach and the
  sim separates cube drag: base = rest + m(rho V) cube.
- **Wrong craft's tables**: two variants of the old spaceplane share a name
  and part list (wings at z 0.70 vs 0.00; saves/README.md lists which save
  is which). qs_cone/qs_entry/qs_plane_inc were flying qs_plane's tables and
  qs_plane its control tables from qs_cone. The game's own oracle: roll due
  to 2 deg sideslip -0.035 vs +0.003 N m/Pa between them. The probe now
  refuses `--aero-from` across craft.
- **Gear table corrupt on the shuttle** (-270 kN of lift): taken after 20 s
  of stepped inputs from a save near the stall. Now on a fresh load.
- **Table grid**: alpha stepped 30 deg outside +-35 and beta stopped at 30;
  the tumbling booster's one-step axial error was 100%. Now alpha every 5
  deg (2.5 in -30..40) round the circle, beta to +-90.

Not in the oracle: the Mk3 shuttle's flying roll and yaw due to sideslip
exceed the paused oracle's by up to 60 kN m, growing with alpha (the old
plane: 20 N m). kRPC's oracle is a careful per-part reimplementation that
uses the parts' current positions; the likely cause is wing flex under
~350 kN of lift. `tools/calibrate.py` fits (game - sim)/q per craft from
flight tests on smooth state terms (x Mach), keeps only components that
improve **held-out** recordings, and stores `aero_residual`, applied inside
its Mach range. Shuttle, leave-one-recording-out: roll 24 -> 8, yaw 23 ->
6 (N m/Pa rms); pitch and axial not improved, so not applied.

Thermal (`thermal.py`, new): two nodes per part (skin, internal), KSP's
shock temperature and convective coefficient (Physics.cfg via
ModuleManager.Physics), areas from drag cubes (`PartDatabase.cfg`),
radiation, skin-internal conduction; per-part `exposure` and conductance
fitted by `tools/thermfit.py` from an observer recording. qs_plane to 56 km
(skin 1001 K): 7-60 K rms per part fitted (in sample). The coefficient's
units were fitted, not derived (1e-3 of the first guess). Parts past
`skinMaxTemp`/`maxTemp` burn up with everything attached below them; the
root's loss destroys the vessel.

### 2026-09-24 (continued) — closed loop: electric charge, airbrakes, ground, pacing

The scripted tests could not see what a whole flight does to a vehicle, so
`flighttest.py --attach` records an autopilot flight in the game frame by
frame (now with charge, CoM, action groups, brakes, RCS, SAS) and
`regimes.py` bins the one-step error by Mach. Each defect below was found
that way, then confirmed closed loop:

- **Electric charge**: qs_entry's pod runs dry holding the nose up; the
  game's wheels then give nothing (ModuleReactionWheel: no torque unless 90%
  of the charge asked for arrives, none at zero input). The sim gave 15 kN m
  from Mach 3 down. qs_entry landing: sim -11.4 km -> **-20.5 km (sd 1.0, n=8)
  vs game -18.1 (sd 2.0, n=4)**; the glide tracks the game within 100-600 m
  and 25 m/s to the cone.
- **Replay CoM**: the shuttle's apparent +100-170 kN m pitch bias was the
  replay spreading the mass loss over the monopropellant tank; with the
  recorded CoM it is +-1-8 kN m.
- **Booster**: action groups (Custom02 deploys the four airbrakes and makes
  them pitch/yaw surfaces; Brakes sets deploy), ModuleAeroSurface (one-sided
  opening, MoveTowards 20 deg/s, tables against angle via the "Deploy Angle"
  field); RCS switched off and on by the autopilot (recorded); the start
  5.8 s after the save as the game's harness does; the time scale reset to
  1x on Load; a flow catch-up burst after Load removed.
- **Ground**: the old contact launched a booster dropped at 5 m/s 120 m
  back up. Now every part's box corners, stiffness and damping from the
  effective mass at the point (2 Hz, zeta 0.8), friction that cannot
  reverse the slip, and parts past crashTolerance destroyed. The KSC pad is
  a 5 m mound the 260 m terrain grid smoothed away: `data/ksc_terrain.json`
  (pad 2 m, runway 5 m) from `tools/kscterrain.py`.
- **Pacing**: a wall-clock-paced loop sleeps its interval *after* its work
  in the game (period = work + interval); lock-step now does the same when
  the autopilot is wall-paced (`common.pacing.lockstep(after_work=)`).
  Booster: sim **median 83 m (n=12)** vs game **median 80 m (n=15)**,
  both with a few 0.7-1.4 km misses; before: 258.

Speed: `--pypy` (autopilot under PyPy) + server under PyPy: qs_entry 155 s
-> 31 s wall; 8 flights in parallel in 60 s.

### 2026-09-24 (continued) — the shuttle closed loop: four general laws

The shuttle flown on the sim arrived at the HAC -44..+153 km long at h 11.9
km, three flights ending "burn guard at 60 s"; the game arrives +3.7..+5.5
km at 18.5 km (LOG3110/3115/3118). Four defects, each general, each read
off the game's own code or oracle:

- **Drain valves** (ModuleResourceDrain): each valve takes its rate of the
  vessel's *whole* capacity per second and vents it as thrust along its own
  `part.transform.forward`. The sim lumped them into one +z thruster at the
  first valve: 2 m/s and a slow yaw into the shuttle's circular orbit (its
  mirrored pair cancels). Coast orbit now matches the game to a metre.
- **`CelestialBody.DensityAt`** is kRPC's equator day-average (latitude bias
  at 0, half the sun term, axial curve at 0), not the bare temperature curve:
  c0 340 -> 353 m/s, the game's number. Every autopilot that tables the
  atmosphere reads it.
- **Rails warp** (TimeWarp via kRPC's non-instant SetRate): the rate lerps
  over one second of real time, and the farm's time-scale plugin stands down
  under warp. The sim multiplied the governed scale into the warp rate and
  switched instantly: the deorbit solve (one long tick at 10x) came 35-45 s
  late. Now 16 s early (dv 26.2 vs 26.6); the rest is the solve's wall time.
- **Control surfaces at sideslip.** Frame by frame on the recorded game
  entry (`a_qs_shuttle_2`), yaw error regressed on beta*q with slope
  -1.6..-2.4 at every Mach 1.2-8; at alpha 30 the game's yaw stiffness is
  -0.49 N m/Pa/deg and the sim's was -2.09. The game's oracle: full
  nose-up elevon takes it from -2.92 to -0.58 at Mach 5 -- a deflected
  surface's moment depends on sideslip, and the solo tables were beta 0
  only. `probe.py --solo-beta` adds each surface's dominant-axis levels at
  beta -15, -4, 4, 15 (~4 min a craft); `aero.Surface` interpolates. The
  oracle is now reproduced across beta with every input to ~1% (M5) - 3%
  (M1.4); the recorded flight's yaw error rms fell 5-15x (M4-5: 59k -> 5k
  N m), slope now -0.33 vs game -0.49. **Not for airbrakes**: their input
  opens them from shut in the probe while in flight they are already open,
  and stacked on the brake table they flew the booster 355 m median against
  70; they stay beta 0 until the brake tables get a sideslip axis.
- The shuttle's old residual ("roll and yaw due to sideslip, growing with
  alpha -- wing flex?") was this same missing effect: refitted against the
  new tables (calibrate.py now fits against the bare tables, not the old
  residual), only pitch still improves held out (7.48 -> 6.46 /q).

Result: sim shuttle HAC arrivals **+0.5..+5.4 km at h 16-21 km in 6 of 8**
(game +3.7..+5.5 at 18.5); two still arrive at 11.9 km. The approach still
differs: the sim holds alpha 13-14 where the game gets 7 at 90 m/s (a
subsonic nose-up pitch residual of the *flying* vehicle; the paused oracle
matches the tables to 1% there) and reaches the flare at 46-55 m/s instead
of 103-128. Both crash at touchdown.

Also: **crash rule** is KSP's (Part.OnCollisionEnter -> HandleCollision):
a part breaks when it *first* meets the ground at a relative speed, the
whole of it, past crashTolerance; wheel colliders exempt. The sim tested the
normal speed every step, so qs_entry's 60 m/s touchdown kept its pod where
all five game flights were "destroyed in ROLLOUT"; now all eight sim flights
are too. qs_entry lands -21.4..-22.9 km (game -18.1); its entry tracks the
game within a few hundred metres to 50 km to go and its in-flight forces
agree to 1-2%. Booster (wall-paced, so a sim batch depends on machine load):
**median ~100 m (n=20)** vs game 80 (n=15), tails alike. Sim server logs now
carry part losses with the UT.

## 2026-09-24 (afternoon) — surface levels, harness start lag, client speed

Found by recording a game flight of `qs_shuttle_final` frame by frame
(`a_qs_shuttle_final_{1,2,3}.jsonl`, `--attach` now records gear too) and
replaying it:

- **Control-surface drag was interpolated linearly between 0 and +-0.5.**
  A deflected surface's drag goes as deflection squared and its lift
  saturates past half travel; the shuttle trims its elevons at 0.17 on
  final, where the chord gave ~4.2 m^2 of extra axial force against the
  whole vehicle's 7-8 (the static table alone agreed with the game to 2-4%).
  `probe.py --solo-levels` adds +-0.05, 0.15, 0.3, 0.75 on each surface's
  dominant axis at beta 0 and every probed sideslip (~4 min a surface);
  qs_shuttle, qs_cone (and siblings), qs_plane re-probed. Axial error on
  the recording -3.6..-5.0 -> -0.07..-0.30 /q. Scripted shuttle tests, 5 s
  restarts: position 7-20 m -> 3-5 m, attitude 9-101 deg -> 3-30 deg.
- **Shuttle residual refitted** on the new tables (calibrate.py): pitch no
  longer survives held-out (dropped); roll/yaw now do (24 -> 8, 23 -> 5 /q)
  and use the body rates: the game departs from its own oracle in
  proportion to rate while the sim follows the oracle -- the damping table
  (0.1 rad/s, linear) is the suspect. Closed-loop rates stay < 0.12 rad/s
  90% of the time, so this matters mostly for the tumbling scripted tests.
- **Start lag**: the game's governed harness engages 1.15 s of game time
  after the save's UT; the sim engaged after 0.25 s (42 m high on
  qs_shuttle_final). `quickglide.py` now advances a sim to save UT + 1.15 s
  (`SIM_START_LAG_S`), as quickfly already did for the booster.
  (`--timescale off` in the game skips the paused handover altogether --
  don't compare against that.)

Closed loop, sim vs game after the fixes:
- qs_shuttle_final: every phase change matches (HAC->APPROACH 62073.56 vs
  .54, gate 4521 vs 4518; FLARE at h 376.6 / v 191.0 / sink 132.6 vs 377 /
  191.0 / 132.5); cross-track at the flare -1281 vs -1569. Both crash.
- qs_entry: sim -19.1 km (sd 1.0, n=8) vs game -18.1 (n=4, earlier) and
  -17.7, -20.3, -19.7 today (LOG3400-3402).
- qs_shuttle: HAC arrival sim +3.8..+5.7 km at 19.1-20.0 km (8/8; before,
  two of eight at 11.9 km), game +4.0..+6.8 at 19.1-19.9 (LOG3404-3406).
  Downstream both scatter km-scale (the autopilot's approach).

Speed:
- Parsed models cached per server (`world.load_model`): Load 0.95 -> 0.11 s.
  Models are now 150-170 MB of JSON.
- Server codec by hand (`fastpb.py`): stream push 0.32 -> 0.13 ms.
- **The PyPy client was the cost, not the server**: krpc's pure-Python
  protobuf plus stubs that rebuild their parameter types per call.
  `fastclient.py` (loaded by a .pth in .venv-pypy, `tools/pypysetup.sh`;
  byte-identical, tested on every signature): aero call 0.35 -> 0.22 ms,
  property read 0.05 -> 0.024; a qs_entry flight 34.8 -> 29.9 s wall.
- The autopilot's propagator is ~84% of its own time: the floor stays there.
  Physics is ~15% of a flight, thermal occlusion ~60% of physics (a cache
  on flow direction is the next speed item; not done, it touches a
  validated port).
