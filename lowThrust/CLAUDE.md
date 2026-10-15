# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in** -- about the same body, one of its moons, or another
planet (or that planet's moon) -- within a **dv / time budget** the user
sets. It is a **standalone KSP mod in C#** (`lowThrust/mod/`): the solver,
the autopilot and its window run inside the game; no kRPC, Python or
Persistent Thrust in flight. A farm-only kRPC bridge (`LowThrustKRPC.dll`)
lets the harness start flights. The root `CLAUDE.md` applies here.

**State (2026-10-14 late):** **aerobraking** works in game: Duna 200 x 200
in 14 passes for 1944 m/s, 546 d (LOG10206); Duna 80 x 80 equatorial in 15
for 1946 m/s, 536 d (LOG10207; the live flight without it 2462 / 561); the
Duna part ~580-610 m/s where it was ~1050. Heat-limited at Kerbin (3 km/s:
passes at ~65 km, many of them). The capture law no longer flip-flops
(the live flight lost every second tick turning); the Minmus push's re-solve
sees its encounter (3 of 7 flights missed by 29,000 km); WAIT's push search
is spread over steps. Earlier: steering costs (0.01-0.02 m/s a flight), the
dv/time frontier with fast planet transfers, the warp clamp. In game on
`qs_lowthrust` (29.9 t, one Dawn, 0.067 m/s^2, 12 RTGs).
History: `docs/lowThrust/journal.md`. What the code does:
`docs/lowThrust/howItWorks.pdf`.

```bash
lowThrust/mod/build.sh                 # build all; install the mod in the farm (base/, clones) and the live game
lowThrust/mod/build.sh test            # ... and run the solver's tests (42, ~10 s)
lowThrust/tools/simmission --from "kerbin: ap=100 pe=100" --to "mun: ap=50 pe=50"
lowThrust/tools/simmission ... --thrust 2000 --mass 29875 --ut 213522 --set NAME=VALUE
lowThrust/tools/simmission ... --frontier                 # the dv/time options
lowThrust/tools/simmission ... --max-dv 1350 --max-days 20  # a budget: the option it picks, or why impossible
lowThrust/tools/simfly --start "ap=100 pe=100" --target "ap=500 pe=500 i=5" --within 2d
lowThrust/tools/sweep.sh OUT "100 200 2000" "0 1800000 3600000 5400000 7200000 9000000" mun duna ...
lowThrust/tools/ltfly.py --instance 0 --target "mun: ap=50 pe=50" [--max-dv 1350] [--max-days 20] [--set K=V]
```

The farm instances load the mod at boot: **restart them after a build**
(`testInstances/stop.sh; start.sh`) or they fly the old dll.

**Live install**: `build.sh` copies `LowThrust.dll` to
`~/Kerbal Space Program/GameData/LowThrust/Plugins/` (and removes the old
`LowThrustWarp`). Persistent Thrust may stay installed: the mod holds its
throttle at 0 and draws every propellant itself. Settings overrides:
`GameData/LowThrust/PluginData/settings.cfg` (`NAME=VALUE` lines); flight
logs: `GameData/LowThrust/PluginData/logs/` (ltfly copies them to
`logs/LOG<n>`).

## Files (`mod/`)

| file | what |
|---|---|
| `core/V3.cs`, `Orbit.cs`, `Bodies.cs` | vectors (Python's op order), two-body maths, Lambert, the stand-in system |
| `core/Guidance.cs`, `Sim.cs` | the within-SOI law; the stand-in, RK4, tick, the within-SOI deadline planner |
| `core/Transfer.cs`, `Mission.cs` | targeting and capture; the phase machine, routing, waiting orbit |
| `core/Tradeoff.cs` | the dv/time frontier, budgets, the impulsive estimate |
| `core/Config.cs` | every tunable; `config:` line and fingerprint |
| `plugin/Controller.cs` | the autopilot: decisions, thrust integrated on rails, the warp governor, PLAN/START/TERMINATE, the log |
| `plugin/Game.cs` | the system and states through KSP's orbital elements; the engines (thrust, Isp, propellants from the part config, draws); warp limits |
| `plugin/Attitude.cs` | rotation on rails from the wheels' torque and MOI |
| `plugin/Window.cs` | the toolbar window: target, budget, options, max warp, status |
| `plugin/FlightLog.cs` | the flight log |
| `bridge/Bridge.cs` | farm-only kRPC service `LowThrustBridge` (Start/Status/Lines/Terminate) |
| `tools/Tools.cs`, `tests/Tests.cs` | `lowthrust.exe mission|fly`, the tests (mono) |

## In the game

- **Every physics step on rails** the controller brings the vessel from
  where it left it (`tSim`) to the game's time: commands are decided as they
  fall due (several per step at high warp), a thrust command is integrated
  (RK4 substeps of tick/THRUST_SUBSTEPS, gravity plus thrust along the axis
  as the vessel points it, held against the orbit), propellant drawn, mass
  reduced, and the orbit written back **through its elements**
  (`Orbit.SetOrbit`); a coast is left to the game. Nothing changes Unity's
  time scale except a decision made aside (below).
- **Frames**: states are read from and written to KSP's orbital elements,
  which are inertial; world space turns with the body below its
  inverse-rotation altitude. The planner's LAN is KSP's (no offset). Each log
  opens with a `frame check` (inclination from elements v world) and a
  `rails check` (bodies 1e6 s ahead: 0 m).
- **Engines**: lit `ModuleEngines`, vacuum thrust x limiter, Isp at 0 atm;
  propellants and ratios **from the part config** (Persistent Thrust removes
  ElectricCharge from the module at run time); each substep draws them via
  the part's flow rules; thrust cut to the scarcest (`propellants:` line).
- **Pointing**: the attitude target is always held against the orbit (R, T,
  N components): a burn's direction, or during a coast the next burn's
  direction where the coast ends (coast run ahead in prediction,
  `COAST_AHEAD`). A tick fires only if the axis is within `ALIGN_DEG` (2).
  The attitude model runs first in the controller's step; a thrust tick
  thrusts along the axis as that step leaves it (behind the command while
  the wheels still turn). Once on target it holds exactly, **charged**: the
  wheels' input for I dw/dt + w x Iw, drawn as EC; a hold the wheels cannot
  give is let go. The `steering:` line logs thrust dv, the cosine loss, dv
  flown > `STEER_OFF_DEG` off, the worst angle (LOG10182-87: 0.01-0.02 m/s
  lost, 48-90 m/s > 0.5 deg, worst 2.0; holding 9-21 EC).
- **Warp governor**: the highest rails rate within: the frame budget
  (`FRAME_BUDGET_MS`, running average), one step drawing at most
  `STEP_RESOURCE_SHARE` (0.5) of any propellant held (EC buffers the
  generators), 5 s steps while the vessel turns, the altitude limits (relaxed to the atmosphere top or terrain +
  `TERRAIN_MARGIN_M` while flying, `WARP_BELOW_LIMITS`), the game's own
  SOI-transition clamp (`TimeWarp.ClampRateToOrbitTransitions`: above index
  3 only if the change is beyond (rate + rate3)/2 + 50 s from the rate it is
  at; mirrored with margin, and a drop by the game held on that body until
  the change has passed), the window's max warp.
- **Decisions are made inside the physics step**, as they fall due. A
  search (the moon push in WAIT, up to ~0.4 s; PUSH ~0.13 s) freezes that one
  frame: a few times a flight. (Made on a worker thread with game time held
  instead, flown 2026-10-14: more unaligned ticks, later arrivals; deleted.)
- **Budget** (`Tradeoff`): PLAN and START fly the mission in the stand-in at
  `PLAN_GRID` (13) arc cutoffs in parallel (~7 s to the Mun) and keep the
  frontier (no option both faster and cheaper). On a route to another
  planet every cutoff is also flown at each `TRANSFER_SHARES` (1, .85, .7,
  .55, .4): the window is the cheapest Lambert arriving within that share of
  the cheapest window's wait + flight (65 plans, ~6 s to Duna; Duna's
  frontier 314 d / 7813 m/s .. 512 d / 2138). The window lists at most 10. Max dv -> the fastest within
  it; max days -> the cheapest within it; none -> the fastest within
  `BALANCE_DV_SHARE` (2%) of the cheapest; a clicked option -> that one.
  Refused at once below `IMPULSIVE_MARGIN` (0.8) of the impulsive estimate,
  else after the scan (flights stop at `BUDGET_SCAN_FACTOR` x the budget),
  saying what is possible. In flight the window shows dv used against plan
  and budget; going over is logged (not enforced).

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
  any SOI exit during a departure phase hands over to CRUISE. ESCAPE (the
  hyperbola out): if the exit would miss by as much as CRUISE re-targets
  for, it burns the Lambert re-target inside the SOI instead (impulse
  through the exit velocity's Jacobian, every tick); else it coasts. B-plane
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

## Measured in game (2026-10-14, `qs_lowthrust`; Minmus rows before the push fix)

| target | flights | wall | days (plan) | dv (plan) |
|---|---|---|---|---|
| Kerbin 150 x 150 | LOG10151 | 6 s | 0.4 (0.4) | 99.6 (99) |
| Kerbin 400 x 150 i3 lan40 argpe90 | LOG10153 | 16 s | 1.2 (1.2) | 361.5 (361) |
| Mun 50 x 50 | LOG10155-57, LOG10176-81, LOG10183-85, 10188-89, 10195-96 | 125-137 s | 35.0-35.1 (35.0); 20.7-20.9 (26.9) | 1332-1381 (1297-1311) |
| Mun, budget 1350 m/s | LOG10154 | 123 s | 14.2 (13.8) | 1351 (1349) |
| Mun, max 10 days | LOG10152 | 14 s | refused: fastest 13.8 d | |
| Minmus 30 x 30 (aside, deleted) | LOG10168, LOG10175 | 174-460 s | 113-215 (154) | 1533-2786 (1371-1383) |
| Duna 200 x 200 (polar) | LOG10163 (before transfer shares) | 417 s | 586 (528) | 2695 (2256) |
| Duna 200 x 200 | LOG10192, LOG10198 | 280 s | 505.8-511.9 (509.5-511.6) | 2165-2239 (2151-2173) |
| Duna, max dv 4000 | LOG10193, LOG10199 | 286-294 s | 404.8-433.0 (406.7-433.3) | 3833-3896 (3842-3906) |
| Minmus 30 x 30 inline | LOG10182, 10186-87, 10190-91, 10194, 10197 | 115-254 s | 59.7-310 | 1358-1742 (1362-1384) |
| Duna 200 x 200, aerobraking (14 passes) | LOG10206 | 1649 s | 545.8 (521.2) | 1944 (1909) |
| Duna 80 x 80 i0, aerobraking (15 passes) | LOG10207 | 2933 s | 536.0 (523.5) | 1946 (1857) |

## Measured (stand-in)


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

0. **A pass still flown on rails** (1 of 48: Kerbin pass 5): the game was
   at 100000x and 2098 s past the entry though the governor asked 5x. Find
   who sets the rate near the air.
1. **Kerbin aerobraking is slow for this craft**: the RTGs (1200 K) bind the
   heating near H 100 at 3 km/s; from 3000 km it is hundreds of ~45 s
   passes. Options: a higher `AERO_HEAT_TARGET` (0.6 now; one pass reached
   0.638), turning the RTGs out of the flow (`AERO_HOLD` Retrograde --
   unmeasured), fewer soak seconds. Measure a whole Kerbin run.
2. **Ike and equatorial targets**: depths are chosen (or a revolution
   skipped) so the next revolution clears Ike at the modelled drag and the
   margin either way; the stand-in still met Ike twice on one run. Watch for
   it in game.
3. **Weak-engine departures with a fast transfer** (LOG10202): the escape
   was captured by the Mun and never left Kerbin.
4. The capture-direction test allows one ~23 deg step near periapsis (the
   lift term ramping in); the old law flipped ~90 every tick.
5. RCS and gimbal steering (unmeasured need: 0.001% steering loss); planets
   beyond Duna in game; routes from a moon.

## Aerobraking (`core/Aero.cs`; Mission `AEROBRAKE`; Controller passes)

At a body with air (Kerbin, Eve, Duna, Laythe, Jool) and a vessel with a
drag area, on by default (`AEROBRAKE`).

- **Air**: density every 1 km (log-linear), sampled from the game's body
  (`GetDensity(GetPressure, GetTemperature)`, base temperature) in flight; the
  stand-in's tables (`Aero.Air`) were sampled from the game by kRPC
  `density_at` (2026-10-14). The air turns with the body (`Body.spin`).
- **Pass** (`Aero.Pass`): RK4 from the top down and out, gravity plus drag
  q x cdA x k / m, no thrust. Heating index H = sqrt(rho) v^3 / 1e6 (KSP's
  high-Mach convective coefficient less its constant), its peak and integral
  (load); peak deceleration; lowest altitude.
- **Depth** (`Aero.Deepest`): the lowest periapsis whose pass flown at
  `AERO_DRAG_MARGIN` (1.5) x drag and density keeps every limit: above the
  highest terrain + 3 km and 0.2 of the air's depth; H and load under the
  learned limits; deceleration under `AERO_ACCEL_MAX`; the apoapsis it leaves
  above the final one; **and a periapsis the engine can lift out of the air
  (+5 km) within +-60 deg of the next apoapsis** (a weak engine must not be
  left plunging back in).
- **Capture**: on SOI entry the arrival periapsis is aimed at the deepest
  depth safe both for the arrival and for the orbit the capture burn leaves
  (`AeroCaptureRp`); the capture law counts on the pass's energy at the low
  end (drag / margin) and burns less.
- **AEROBRAKE** (after capture, or in FINAL whenever the apoapsis is above
  the final one): after each pass the next depth is chosen; the guidance sets
  it in the arc at apoapsis (target ra free, rp = depth, the target plane
  carried, so the plane change happens at the high apoapsis); among depths
  from the deepest up in tenths, the first whose next revolution -- at the
  modelled drag and at the margin either way -- meets no moon; none: if the
  orbit as it is stays clear a revolution more, the periapsis is kept out of
  the air for one ("no pass this revolution"). Ends when a pass would lower
  the apoapsis by less than `AERO_MIN_GAIN` (0.005) of what is left -- but
  not while the heating limit is still growing -- then the periapsis is lifted
  out (the arc at apoapsis widened to fit the lift) and FINAL finishes.
- **Learning** (in game): pass 1 at `AERO_HEAT_FIRST` (30); after each pass,
  every part's rise of max(skin / skin max, interior / max) is extrapolated
  linearly to `AERO_HEAT_TARGET` (0.6) of its maximum, the H and load allowed
  next = the pass's x that factor (at most `AERO_HEAT_GROWTH` 3); the drag
  scale k = energy shed / modelled at k=1. The stand-in plans at
  `AERO_HEAT_PLAN` (100) and learns nothing.
- **In the game**: the mission stops at the top, inbound (the warp's steps
  shrink to a quarter of the time left to the air); the game flies the pass
  off rails at the fastest physics warp rate up to `AERO_PHYSICS_WARP` (4x, chosen by rate: mods renumber the indices), SAS holding
  `AERO_HOLD` (Prograde); every step H, q, the parts' drag (sum of
  `dragScalar`) and temperatures are measured; out of the air the
  temperatures are watched until they stop rising (`AERO_SOAK_S` at most),
  then the orbit is read back and the mission told. If the game drops off
  rails just above the air on its way down, the pass starts there.
- **Measured on `qs_lowthrust`** (LOG10201-, 2026-10-14): drag area 18.6-23.7
  m^2 at Kerbin, 12.9 at Duna, against 4.74 from the drag cubes (the prior is
  2.7-5x low; k learns it in one pass). The hottest part is always an RTG
  (max 1200 K): at Kerbin 0.28 -> 0.47 of its limit at H 64, so the limit
  settles near H 110 (passes at ~65 km at 3 km/s); at Duna 0.23 -> 0.26 at
  H 14.

## Traps known

- **mcs binds to the system Mono's newest overloads**: `s.Split(',')`
  compiled to `Split(char, StringSplitOptions)`, which KSP's Mono lacks;
  every START failed with a MissingMethodException from the bridge. Use
  `Split(new[] { ',' })`; check `monodis --memberref bin/LowThrust.dll`.
- **The game clamps warp before an SOI change** from the rate it is at;
  asking for more each step makes it drop each step (a log line 50x a
  second, the "lag" -- not BetterTimeWarp, whose EC toggle is off).
- **`TimeWarp.fixedDeltaTime` is the step's game time** (scaled by the
  rate); `Time.fixedDeltaTime` is the real 0.02 s.

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
