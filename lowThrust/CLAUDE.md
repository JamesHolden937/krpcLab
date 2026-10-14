# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in** -- about the same body, one of its moons, or another
planet (or that planet's moon) -- optionally within a time budget. The
solver works out every burn; the pilot (`pilot.py`) flies them in the game
under rails warp with Persistent Thrust and this package's plugin
(`plugin/`, the kRPC service `LowThrust`). The root `CLAUDE.md` applies here.

**State (2026-10-14): the solver arrives at all 15 destinations at every
thrust 0.02-10 m/s^2 in the stand-in (630/630). In the game, on the user's
craft (`qs_lowthrust`, 29.9 t, one Dawn, 0.067 m/s^2): LKO 87 -> 150 km
arrives 150.1 x 150.0 km, 97.9 m/s, 1.6 days, 90-140 s wall (stand-in 97.9,
1.55 d); LKO -> Mun 50 x 50 km arrives every time (three flights,
1372-1392 m/s against the stand-in's 1319, 69-83 days against 44, 15 min
wall). Open: the in-game PUMP takes ~54 days where the stand-in takes 35
for the same dv.**
History, every refuted approach and why: `docs/lowThrust/journal.md`.
What the code does and what triggers what: `docs/lowThrust/howItWorks.pdf`.

```bash
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "mun: ap=50 pe=50"
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "duna: ap=200 pe=200 i=10"
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "ike: ap=50 pe=50"
./lowThrust/tools/simmission.py ... --thrust 200 --mass 5000 --isp 4200 --ut 3000000
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500 i=5" --within 2d   # one SOI, with a deadline
lowThrust/tools/sweep.sh /tmp/sw "100 200 500 2000" "0 3000000 6000000 9000000"   # the grid: every destination x thrust x date
.venv-pypy/bin/python -m unittest lowThrust.tests.testOrbit lowThrust.tests.testGuidance lowThrust.tests.testMission lowThrust.tests.testLive

./run.sh --pilot lowthrust                     # in game: panel with target, PLAN, START, TERMINATE
lowThrust/tools/ltfly.py --instance 0 --target "mun: ap=50 pe=50" --timeout 3600   # farm: load qs_lowthrust, fly, report
lowThrust/plugin/build.sh                      # build the plugin into base/ and every farm instance (restart them)
```

**Live install**: Persistent Thrust 1.7.5 and
`GameData/LowThrustWarp/LowThrustWarp.dll` (built by `plugin/build.sh`;
copy it from `testInstances/base/GameData/LowThrustWarp/`). A save made
before PT was installed has no `PersistentEngine` on its engines (PT finds
them only in OnLoad): rebuild or relaunch the craft.

Orbits: `body: ap= pe= [i= lan= argpe=]`, km and degrees; anything left out
of the target is free. `simmission` prints the phase timeline (Kerbin dates),
every burn (phase, body, start, duration, dv, prograde/normal/radial share),
dv by phase and the totals. Default vessel: 5 t, one 2 kN Dawn, Isp 4200
(0.4 m/s^2, TWR 0.04); `--mass`, `--thrust`. The user's craft: TWR ~0.004
(0.04 m/s^2) -- the case that has to work.

## Files

| file | what |
|---|---|
| `orbit.py` | two-body maths: elements, any conic, universal-variable propagation, time to periapsis, Lambert, radius toward a direction, impulsive references |
| `guidance.py` | the within-SOI law: Hohmann in many small burns + plane change at the nodes (+ a held apse line) |
| `plan.py` | within-SOI deadline planner: the narrowest arcs that meet a time budget |
| `bodies.py` | the stand-in system: stock bodies on Keplerian rails, SOIs |
| `transfer.py` | targeting: closest approach / B-plane miss, mid-course correction, moon push, porkchop window, parking orientation, capture design and the capture law, Lambert re-target, SOI hazards |
| `mission.py` | the phase machine for a whole transfer (patched conics), routing and leg chaining |
| `sim.py` | the single-body stand-in (`fly`) and the shared RK4 / tick |
| `tools/simfly.py`, `tools/simmission.py`, `tools/sweep.sh` | command lines; `sweep.sh` flies the grid in parallel |
| `live.py` | the game as the planner sees it: bodies rebuilt from kRPC state vectors (rails checked against `Orbit.position_at`), frame swap (kRPC non-rotating is left-handed, y north: `to_ours` swaps y and z) |
| `pilot.py` | the only kRPC-aware code: panel, `Mission.step` flown against the game, holds, ledger from propellant used |
| `plugin/LowThrustWarp.cs` | kRPC service `LowThrust`: PT throttle on rails, instant altitude-capped rails warp, the pace (Unity timeScale) and exact holds, reaction-wheel attitude on rails |
| `tools/ltfly.py` | farm harness: loads a save, runs the pilot as a reaped child, prints the result |

## The pilot (in game)

- Each tick: read the state (SOI change -> `Mission._soi_change`), `step()`
  gives (throttle, direction, tick) or the result.
- **Thrust tick**: the direction is held fixed against the orbit (kRPC's
  orbital frame = (-radial, along-track, normal)), so the plugin tracks it
  through the tick; fired only if the thrust axis is within `ALIGN_DEG`
  (2) at the tick's start; throttle set through PT; warp = the largest
  rate <= `WARP_PER_TICK` (5) x tick; held to exactly t + tick; dv booked
  from the xenon used (Isp g0 ln(m0/m1); KSP freezes mass on rails, so
  mass = load mass less resources used).
- **Coast**: the mission is run ahead on the exact two-body coast until it
  next wants thrust, an SOI boundary, or `COAST_AHEAD` (0.5) of a period;
  the vessel turns to the pending direction and the game warps there in
  one hold.
- **Holds** (plugin `RunUntil`): rails warp runs at `RAILS_PACE` (20) x
  KSP's pace (Unity timeScale sized from the longest recent frame x 1.5,
  at most half the steps left, under one step a frame at the end: 0-3 s
  late); the rate climbs to the one asked for as the altitude allows and
  steps down near the end, not below `WARP_FLOOR_S`'s (10 s tick) rate;
  at the end the game is held (timeScale 0, kRPC served from the
  plugin's Update). A hold nobody watches for 30 s (`Held` is streamed)
  is released; a hold found dropped is re-issued after `HOLD_LOST_S`.
- **Attitude on rails** (plugin `Attitude`): the wheels' torque (x
  authority) and MOI measured unpacked; integrated in 0.02 s substeps;
  rate limited to sqrt(2 a angle) and angle/4 s, rate error closed over
  0.5 s; EC drawn per |input| (a wheel short of EC loses torque); the
  angular rate handed back to the rigidbodies on unpack. **RCS torque
  and gimbal misalignment are not modelled** (the user asked for both;
  the user's craft has neither RCS nor a gimballed engine).
- PT draws the Dawn's EC on rails (9.2 EC/s at full thrust with the
  wheels); the craft's 12 RTGs make 9.0, so its battery stays full.

## The phases

```
within SOI:   FINAL
to a moon:    PUMP -> WAIT -> PUSH -> COAST [-> MATCH -> COAST] -> APPROACH -> CAPTURE -> FINAL
to a planet:  PARK -> WAIT -> DEPART -> ESCAPE -> CRUISE [-> MATCH -> CRUISE] -> APPROACH -> CAPTURE -> FINAL
a planet's moon: the planet leg ends in the moon leg's waiting orbit, then the moon leg
anywhere:     TRIM (phasing, to miss a moon: at the next periapsis, or now), evasive burns in an unplanned SOI
```

- **Within SOI** (`guidance`): prograde/retrograde arcs centred on a fixed
  apse line (re-locked between burns), each fixing the opposite apsis;
  outward-first order; plane change at the nodes on the biggest orbit;
  overlapping burns one vector. Within +0.0-3.1% of the impulsive Hohmann
  on apsides changes, +0.1-8.4% with plane changes.
- **Moon**: pump to a waiting orbit in the moon's plane, wait for the point
  whose prograde push puts a *direct* encounter on the aim, push, correct
  mid-course, nudge the periapsis, capture (below), FINAL.
  - **Waiting orbit** (`waiting_orbit`): apoapsis 0.6 of the way, periapsis
    as low as possible but high enough that the engine captures the
    arrival -- in one pass, or by the long capture inside the SOI up to its
    limit (`moon_vcap_long`). Arriving from above (a planet leg first), the
    whole orbit sits in the highest gap between other moons' bands under
    the moon (Vall at 0.04 m/s^2: 39 x 37 Mm, between Laythe and Vall);
    from below with a moon inside the target's (Minmus), it stays under that
    moon. Nothing capturable: the slowest arrival, the highest clear one.
  - **Push points**: the periapsis, and every 30 deg if the push there costs
    no more than `PUSH_OFF_PERIAPSIS` (1.1) of the periapsis Hohmann; each
    decided far enough ahead to start the burn half its duration early, the
    point locked once decided. The push is planned and re-solved as the
    finite burn it is (`transfer.prograde_burn`). Points whose coast meets
    another moon, or that arrive faster than the capture takes (a bar
    rising 25% a skip, at most once a synodic period), are skipped.
- **Aim** (`transfer.aim_error`): measured at the SOI entry when the coast
  enters (patched conics: the entry state sets the periapsis), at most 0.95
  of the SOI; at the closest approach (straight-line) when it does not.
  Push, mid-course correction and MATCH share it.
- **Planet**: porkchop window after the pump's estimated end; a parking
  ellipse whose periapsis push leaves on the departure asymptote (plane
  chosen so its change is cheapest); push at the periapsis nearest the
  window, each pass checked for moon hazards along the burn DEPART will
  actually fly (multi-pass if the engine is too weak to escape in one);
  any SOI exit during a departure phase hands over to CRUISE. B-plane
  corrections in the cruise, or a Lambert re-target (aimed at the B-plane
  point) when the miss is too big to linearise, the encounter looked for
  round the planned arrival.
- **Polar capture**: at a planet with moons and a target plane left free,
  the aim point is over a pole (correction, re-target and MATCH alike), so
  the descent crosses each moon's orbit far out of its plane.
- **Capture** (`transfer.capture_step`, shared by flight and planning's
  prediction `fly_capture`): it only has to end *bound* (below 0.3 SOI, or
  below the moon-clear ceiling once past periapsis, periapsis above the
  floor); FINAL shapes the orbit. Inside the SOI only. A capture that fits
  one pass burns centred on periapsis. A longer one: thrust along the
  velocity removes energy at `a` per metre of path, so with q = energy to
  shed / (a x path left to periapsis) it starts at q =
  `CAPTURE_PATH_SHARE` (0.8) and throttles at q, finishing at periapsis
  with a steady throttle. Direction: retrograde, less its part along the
  periapsis gradient once at or below the aimed periapsis (it sheds energy
  without lowering it); a gradient term holds the band (floor .. 1.5x aim).
- **MATCH** (planned when the arrival is faster than a capture starting
  inside 0.8 of the SOI handles): sheds the excess about the star (or, for
  a moon, the planet) -- zero-effort-miss/zero-effort-velocity guidance onto
  an aim point just outside the SOI, arriving at 0.8 of the fastest speed
  the capture takes; started when braking at 0.8 of the engine must start,
  ended 600 s before the aim point, or given up once it has spent twice the
  speed it set out to shed.
- **Approach dodge**: inbound to a planet with moons, if the coast to
  periapsis, flown through the moon's SOI in patched conics, would hit
  it, leave the system, or leave the periapsis under the floor, APPROACH
  burns the cheapest dodge across the velocity (periapsis put back on the
  aim; costed with the plane change it adds).
- **Deep wells with moons** (Jool): planning compares a tight capture with a
  loose one (0.5 SOI) plus the impulsive lowering after, by predicted dv.
- **Moon hazards in orbit shaping**: while the orbit crosses another moon's
  band the arcs are the widest (cutoff 0.5, +-60 deg: fewest revolutions
  there); else they widen until the apsis changes left fit 30 passes. Each
  arc burn's first tick flies the burn in prediction by the guidance law
  itself (`_fly_arc`, closed loop) and caps it at the largest tenth whose
  next revolution meets no moon (none: the latest encounter), the cap
  delivered exactly. Once a revolution a phasing trim -- now if the rest of
  this revolution meets one, else at the next periapsis -- the smallest of
  +-0.5..8 m/s or of the burn that slips the pass past a moon's SOI.
- **Floor guard**: outside CAPTURE/DEPART, a bound periapsis inside the
  atmosphere or surface is lifted along its gradient first.
- **Thrust ticks** change the velocity by at most 2% (`THRUST_DV_SHARE`).
- **The stand-in vessel** has constant mass and unlimited propellant
  (`Vessel(mass, thrust, isp)`): it plans paths; xenon is reported from dv.

## Measured (stand-in, from LKO 100 km, 5 t vessel)

Sweep of 2026-10-13 evening (`sweep.sh`, 630 flights: 15 destinations x 7
thrusts x 6 start dates UT 0-9e6 s): **all 630 arrive**. Mean dv (m/s):

| m/s^2 | Mun | Minmus | Duna | Ike | Eve | Gilly | Dres | Moho | Jool | Laythe | Vall | Tylo | Bop | Pol | Eeloo |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.02 | 1654 | 1698 | 3068 | 3176 | 4291 | 3521 | 6186 | 8874 | 8328 | 8562 | 7038 | 7870 | 6825 | 6734 | 7676 |
| 0.04 | 1377 | 1517 | 2450 | 2537 | 3840 | 3123 | 5413 | 7551 | 7334 | 7541 | 6544 | 7231 | 5913 | 5943 | 6208 |
| 0.1 | 1232 | 1299 | 2502 | 2542 | 4187 | 3245 | 5026 | 7197 | 6613 | 7479 | 6177 | 6364 | 5333 | 5438 | 5615 |
| 0.2 | 1113 | 1245 | 2120 | 2384 | 3585 | 2926 | 4937 | 6615 | 6341 | 6908 | 5762 | 5875 | 5207 | 5211 | 5289 |
| 0.4 | 1126 | 1234 | 1829 | 1853 | 3068 | 2774 | 4705 | 6326 | 6127 | 6497 | 5300 | 5613 | 4810 | 4899 | 4973 |
| 2 | 1135 | 1231 | 1838 | 1814 | 2863 | 2733 | 4085 | 5807 | 5495 | 6240 | 4802 | 5068 | 4372 | 4495 | 4282 |
| 10 | 1135 | 1324 | 1844 | 1861 | 2860 | 2738 | 3811 | 5507 | 5451 | 6017 | 4859 | 5025 | 4235 | 4408 | 4153 |

0.04 m/s^2 is the user's craft (TWR ~0.004). Grid for quick checks (270
flights, ~4-5 min): `sweep.sh OUT "100 200 2000" "0 1800000 3600000
5400000 7200000 9000000" moho mun minmus duna eve ike dres jool eeloo gilly
pol tylo laythe vall bop`.

## Open, in order

1. **In-game PUMP 54 days v the stand-in's 35** for the same dv (820 v
   808 m/s): less thrust per revolution in game. LOG10134 (Mun,
   `LOG_INTERVAL_UT=1`) logs every tick: compare its arcs (start, length,
   fired or not, err) with `simmission`'s burn list for the same craft
   (`--thrust 2000 --mass 29875 --isp 4200 --ut 213522`). MATCH in game
   took the periapsis to 66 km before the Mun SOI rescued it (all three
   flights) -- check the floor guard during MATCH.
2. **RCS and gimbal on rails** in the plugin (the user asked): RCS torque
   when the RCS group is on, burning its propellant; gimbal torque only
   while thrusting, with the thrust deflected by the gimbal angle.
3. **A planet transfer in game** (Duna), then **aerobraking** (below; the
   craft has a heat-resistant nose cone for it).
4. Routes not yet supported: from a moon (Mun -> Kerbin, Mun -> Minmus,
   moon -> planet).
5. **Cheaper moon dodges**: APPROACH dodges a moon on the inbound leg late
   and dear (a 10 m/s^2 Duna arrival through Ike: 238 m/s). Choosing the
   aim point round the B-plane circle in the cruise, where it is cheap,
   would avoid it.
6. **Eve at 0.1 m/s^2 dearer than at 0.04** (4187 v 3840), Minmus at 10
   m/s^2 than at 2 (1324 v 1231): not failures, unexplained.
7. **Weak-engine departures**: a 0.04 m/s^2 escape leaves with ~760 m/s of
   a 2.9 km/s window v_inf, and the cruise re-target builds the rest
   (Jool: CRUISE 2.3-2.6 km/s of the total). Limiting DEPART to its
   +-60 deg arc was flown and lost (v_inf 765 even at 0.4 m/s^2, a Mun
   hit); spiralling out and leaving on the asymptote might not.

## Aerobraking (planned, not built -- the user, 2026-10-13: "think about it")

Where it pays: captures and orbit-lowering at bodies with an atmosphere
(Kerbin, Eve, Duna, Laythe, Jool). For a weak engine it is the biggest
saving there is: Duna's capture plus FINAL is ~1 km/s at 0.04 m/s^2, Jool's
~2.5 km/s, and both are slow (long burns, many passes through moon bands).
What it needs:

1. **An atmosphere in the stand-in**: density by altitude per body. KSP
   defines pressure and temperature curves per body; density =
   p M / (R T). Take the curves from the game (kRPC
   `body.atmospheric_density_at_position` sampled on a save, the way
   `kspSim/data/` holds terrain) rather than transcribing constants.
2. **The vessel's drag**: a ballistic coefficient (m / (Cd A)) measured in
   the game (kRPC `flight.drag` at known q), and the heating limit (KSP
   skin temperature; the spaceplane's `trajectory.peak_heating` uses
   sqrt(rho) v^3). Prograde-pointing drag only -- no lift.
3. **Propagation through it**: the two-body coast is no longer exact below
   the atmosphere top; integrate the atmospheric arc with drag (RK4, like
   the thrust arcs), exact conics everywhere else.
4. **The law**: aerocapture = aim the arrival periapsis at the altitude
   whose one pass sheds exactly enough to be bound (a 1-D search on the
   predicted pass, re-solved every tick of the approach, the B-plane
   correction steering the periapsis); aerobraking = after any capture, a
   periapsis in the upper atmosphere lowering the apoapsis pass by pass,
   lifted out at the end by a burn at apoapsis. Heating caps how deep.
   Both fit the phase machine as an alternative CAPTURE and a FINAL
   variant, chosen at planning when the body has an atmosphere.
5. **The pilot**: KSP will not rails-warp inside an atmosphere, so each
   pass is physics-warped; Persistent Thrust is irrelevant there (no thrust
   during a pass).

## Traps known

- **Under PyPy, `is` compares floats by value** (`goal is target.rp`
  deadlocked every circular target).
- **Ticks must be uniform in angle**, not time: near the periapsis of a long
  ellipse a time-uniform tick skipped the whole arc and every pump past
  ~4000 km stalled.
- **Near-circular, the osculating apsides are not places**; any law chasing
  them surfs its arc's edge. Hence the fixed, re-locked apse line.
- **A throttle against a sliding window never finishes**.
- **A closest approach at the edge of the search window is no encounter**;
  measure the whole distance, not its across-track part.
- **Multi-revolution intercepts are hypersensitive**: moon pushes look only
  within 0.75 of a revolution.
- **Linear corrections only near an encounter**: far out they ask km/s;
  re-target with Lambert instead.
- **Every orbit past a moon crosses it twice a revolution**: SOI hazards
  are the norm, in the game as in the stand-in.
- **Live install differs from the farm**; the live install needs PT 1.7.5
  and the plugin dll; it has BetterTimeWarp (rates read from the game).
- **kRPC's warp and throttle fight PT**: `RailsWarpFactor` zeroes the
  FlightInputHandler throttle and `MaximumRailsWarpFactor` is 0 while it is
  open, so the plugin sets PT's throttle (`SetThrottleAfterKey`) and the
  warp itself. PT's throttle set while unpacked does not survive going on
  rails: set it after the warp.
- **kRPC serves RPCs from FixedUpdate**, which stops at timeScale 0: the
  plugin pumps kRPC from Update while a hold has the game stopped.
- **An inertial command is not a law's direction for a whole tick**: near
  a high periapsis prograde moves 6 deg in a 26 s tick, past ALIGN_DEG.
- **A warp set at periapsis's limit stays there** unless something raises
  it (KSP only lowers): the pump flew its 5000 km orbits at 50x.
- **Count parts, not titles**: a set of part titles read one RTG where the
  craft has twelve, and a power shortage was reported that does not exist.
- **Energy per metre of path is the same wherever a burn is flown**; dv
  is not -- it is energy over speed, so the slow outbound tail after
  periapsis is the dearest place to finish a capture.
- **A long capture flown retrograde to the end has a knife-edge start**
  (too early stops the vessel dead far out and it hovers as it falls).
  Decide each tick from the energy and the path left, not from a
  predicted start time.
- **Size a law's main term to one tick's dv** before adding a correction
  to it, or the correction drowns (a periapsis term under a 2600 m/s
  retrograde vector let a capture sink into Moho).
- **`orbit.propagate` on a hyperbola**: the long-arc starting guess
  diverges for a short coast far out; it now takes the better of two.
- **A horizon counted from a stale plan reads a phantom miss**: the
  cruise's re-target measured its miss up to the *window's* arrival on an
  arc re-targeted 170 days later, found the minimum on the search's edge,
  and re-targeted Eeloo into a failed capture.
- **Check a pass along the burn that will be flown**, not an impulse: a
  weak escape checked at the window's v_inf met the Mun at its real 229.
- **Two gates measuring the same quantity differently deadlock**: the
  plane burn waited on the osculating periapsis, the apsis arc on the
  radius toward the held apse line; PARK coasted 4000 days.
- **A prediction must fly what the flight flies**: three of this
  session's failures were predictions of a different burn -- the pass
  guard's impulse at the arc start (days off on the next pass), its
  first-tick dv estimate (42 m/s predicted, 121 burned), and a push
  planned as an impulse that took 2.4 h. Predict with the law itself,
  closed loop, from when it will actually start.
- **Aim where the conic is decided**: the patched-conic periapsis is set by
  the SOI-entry state. A straight-line closest approach is right only
  for fast arrivals; at 75 m/s the aim fell outside Ike's SOI and the
  correction steered away.
- **A check keyed by something its own action moves re-fires at once**:
  phasing trims keyed by the periapsis time moved the key with every
  trim (+-0.01 m/s for years).
- **Sweep thrusts and dates, not one flight**: every fix here was found by
  a 270-432 flight grid (`simmission` takes 1-20 s), and two "fixes" that
  looked right on one case lost 10-20% dv or robustness on the grid.
