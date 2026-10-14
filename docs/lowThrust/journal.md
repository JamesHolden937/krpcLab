# lowThrust journal

## Session, 2026-10-12: the project planned

The user wants a low-thrust (ion) autopilot: type in a target orbit, the
solver works out the maneuvers, later interplanetary. Hand-flying ion craft
is impractical and MechJeb does not do it.

- Persistent Thrust 1.7.5 + PersistentThrustNavigator are already in the
  farm's `base/GameData` (PT replaces the engine PartModule); the live
  install's copy is not yet checked.
- Plan written into `lowThrust/CLAUDE.md`: Q-law (with Edelbaum and the
  single-element laws as special cases) as a per-tick feedback law, a
  propagator that flies the same law for the PLAN preview, and a kRPC
  executor under rails warp. The executor's design waits on a probe of what
  attitude PT holds on rails (SAS maneuver mode, plain SAS modes, or
  segmented warp).
- Difficulty, as told to the user: the maths is more abstract than either
  existing autopilot but textbook, the plant is exact (no aero, no attitude
  dynamics on rails), so it should be far less empirical than the shuttle.
  Stage 1 a session or two; stage 3 (precise arrival) open-ended.

## Session, 2026-10-12 (later): the stand-in, and apsides changes solved

The user: build a simple sim with a stand-in planet, get apoapsis/periapsis
changes working first, with a "within x minutes" input -- "the most
efficient way is to split a Hohmann transfer into a billion tiny burns".

Built `sim.py` (Kerbin numbers, exact coasting, RK4 thrust), `orbit.py`,
`guidance.py`, `plan.py`, `tools/simfly.py`, 15 tests. The law that won is
the user's: Hohmann in many small burns. The route there, each step
measured on 100 -> 500 km circular unless said:

1. **Q-law on the apsis radii**: chattered +-20 km round a circular target
   for 9 km/s -- ra/rp are singular at e = 0 (the apse flips as they
   cross). Switched to a and e (e^2 is smooth through circular).
2. **Per-element throttle limits** pinned the throttle at 0 whenever one
   element sat on its target; replaced by an exact line search on Q.
3. **Low effectivity cutoffs trap**: radial thrust at periapsis drags the
   apse line round with the vessel, which never reaches apoapsis (eta 0:
   9 km/s, never arrived). Floored at 0.2.
4. **The tail was 80% of the time**: the effectivity gate admits a whole
   arc and the throttle nulled the error on its first tick -- the arc's
   edge, its worst point. A 5 km circularisation burned at nu 120/300 for
   8.5 m/s against 2.0 impulsive.
5. **Narrowing the arc to the remaining need** fixed that case (one burn at
   nu 180, 2.05 m/s) but on a flat near-circular profile picked slices at
   eta 0.998, and the vessel never landed in them.
6. **A deadline-driven arc (need / orbits left)** -- same failure, plus a
   circular orbit's flat profile tied eta == cutoff and never burned.
7. **Spreading the need over the orbit's arcs** is a sliding horizon: the
   remainder decays exponentially (a factor 3 per two orbits) and the bulk
   flew at half throttle. Over "the arc the vessel is in" -- the same, since
   a low cutoff's arc is the whole orbit.
8. **The apsis law** (prograde/retrograde at arcs round each apsis, each arc
   fixing the opposite apsis -- decoupled, deadbeat). First version failed
   with wide arcs: **surfing** -- near-circular, a burn short of apoapsis
   turns the osculating apse as fast as the vessel moves, so it rode the
   arc's edge (nu frozen at 137 deg) at ever lower throttle for weeks.
9. **Fixed inertial apse line**, controlling the radius toward its two
   directions: no surfing, but the burns cannot correct e across the line
   (500 circular ended 628 x 396). **Re-locked between burns, held through
   each**: everything arrives.
10. **Hohmann order** (outward first, inward first): 100 -> 3000 from +15%
    (spiral cost) to +6.7%.
11. **Burn starts half its duration before the apsis**: 100 -> 3000 to
    +2.7%, but 2000x100 -> 2000 to +20%: one pass took e 0.58 -> 0.14 and
    overshot the far apsis by 390 km.
12. **Narrow arcs by default (eta 0.99, +-8 deg)** cap each pass: that case
    +1.6% in the same time. Planner grid spaced in half-width.

Result (default): nine transfers within +0.0-3.1% of the impulsive Hohmann,
arrivals inside 100 m; deadlines trade dv for time (0.04 m/s^2 engine,
100 -> 500: 41 days at +0.0% down to 1.1 days at +14%). Q-law deleted (git
history), apsis law the only guidance. Prograde/retrograde only, so plain
SAS modes can fly it in the game.

Also paid: under PyPy `is` compares floats by value -- `goal is target.rp`
deadlocked every circular target (dv 0, orbit limit).

## Session, 2026-10-12 (night): the spiral question, plane change

**Spiral** (asked: how inefficient would a spiral be?). Ideal cost
|v1 - v2|: +1.3% on Hohmann for 100 -> 500, +16% for 100 -> 3000, +43% for
100 -> 12000. Flown naively (continuous prograde until a matches, then the
law trims): at TWR 0.004, 100 -> 500 took 3.1 h at the ideal cost but left
e 0.029, and the trim made it +7.1% in 5.9 h (default small burns: +0.2% in
72 h). At TWR 0.04 it is not a spiral -- the burn ends inside one orbit
with e 0.28 (+47% after trim). 500 -> 100 at TWR 0.04 put the periapsis in
the atmosphere. Found on the way: **100 -> 12000 km never arrives** with
the default law (20000-orbit limit) -- open.

**Interplanetary** (the user: launch early, pump with tiny burns, final
push at the window -- "get there as soon as anyone else"). Right in
principle: every burn at periapsis, so the total equals a one-burn
departure and the arrival date is the same. The final push must fit one
periapsis pass: from a 100 x 9000 km ellipse, Eve 204 m/s, Duna 240, Jool
1111 -- 8/10/46 min at TWR 0.04, against ~10 min within +-60 deg of
periapsis. Table in lowThrust/CLAUDE.md.

**Plane change.** Normal/anti-normal arcs at the nodes with the target
plane, the arc/centring machinery of the apsides. Steps:
1. First version: pure plane changes +0.1-0.8% at once; combined raise +
   5 deg +11.8%, lower 3000 -> 100 + 20 deg +15%.
2. Overlapping burns were each clipped to full throttle before combining,
   so an overlap always pointed at 45 deg. In proportion to need: no
   change -- the arcs were not overlapping.
3. Started half the *combined* duration early: no change, same reason.
4. **Why not overlapping**: on an equatorial start the node line was
   chosen at the start; 15 orbits of apoapsis raising turned the apse line
   off it. Chosen when the plane change begins, on the apse line: raise +
   5 deg to +1.2%.
5. **Lowering**: on a circular orbit both nodes are equally good, and half
   the change went uncombined at the periapsis-side node. While the
   periapsis still changes, only the apoapsis-side node: +15% to +6.7%.
Open: 100 -> 3000 with i 6 -> 0 is +8.4% -- an inclined start's nodes are
fixed, and the apse drifts off them over 72 orbits of raising.
The impulsive reference for case "nodes at +-90 deg" read +133% until it
was computed at the actual nodes (only the horizontal velocity turns):
-0.6%.

## Session, 2026-10-12 (late night): transfers between bodies

The user: keep working until the solver generates maneuvers for
interplanetary, to-moon and within-SOI transfers, then push.

Built `bodies.py` (stock system on rails), universal-variable propagation,
hyperbolic conics, Lambert, `transfer.py`, `mission.py` (patched-conic phase
machine), `tools/simmission.py`, `tests/testMission.py`. In order, what broke
and what fixed it:

1. **Pump stalled past ~4000 km** (the 100 -> 12000 km bug): time-uniform
   ticks swept 20 deg near periapsis, wider than the +-8 deg arc. Ticks are
   now uniform in angle.
2. **Held apse line** for the departure parking ellipse: periapsis pinned to
   a direction, +-90 deg arcs correct the cross-line eccentricity at the end
   (within 0.02 deg at 0/60/135/250).
3. Kerbin -> Mun and -> Duna flew end to end on the first runs; Duna's
   capture ellipse (14000 km) crossed Ike and FINAL "arrived" about Ike.
   Capture apoapsis kept under the destination's moons; arrival requires
   the right body.
4. **Minmus: COAST spent 1654 m/s.** The push never burned (a failed
   re-solve read as "done") and the linear correction chased a 46000 km
   miss. Push ledger; a moon COAST with no real encounter goes back to WAIT.
5. **Unplanned Mun flybys** on the way to Minmus (every orbit past the Mun
   crosses it twice a revolution): passes whose coast meets another moon
   are skipped; flyby recovery re-pumps; pass caps and phasing trims.
6. **Pushes kept missing**: (a) two-revolution intercepts, hypersensitive,
   re-solves jumping roots -- direct transfers only (0.75 revolution), local
   re-solve; (b) a B-plane branch inside the push flickered with the SOI test
   and oscillated the apoapsis 350 km a tick -- removed; (c) the decisive
   one: the push solver scored a minimum at the search window's edge by its
   across-track part (557 km for a pass 10400 km away). Whole distance, edge
   minima rejected: every first push hit (Mun 1108, Minmus 1188).
7. Lower waiting orbit (0.6 of the way to the moon): more passes, a wider
   push, no dv cost (both at periapsis).
8. **Chained legs**: Kerbin -> Ike via a Duna capture straight into the moon
   leg's waiting orbit. Periapsis margin 10 -> 30 km (approach error plus a
   long capture burn put Duna's periapsis in the atmosphere); capture holds
   the periapsis above the floor only (holding it on the aim cost 100 m/s).
9. **Weak engine (TWR 0.004)**: capture high and loose (rc up to 0.2-0.5
   SOI), joint waiting-orbit/capture design, waiting periapsis kept out of
   other moons' bands (11988 km sat on the Mun's orbit), multi-pass
   departure, slow-arrival cruise (3x3 Jacobian with the arrival speed),
   evasive burns in unplanned SOIs, re-binding after a slingshot (to 0.6
   SOI, not merely bound). Mun arrives; Minmus and Duna remain fragile.
10. **High-v_inf targets ran out of propellant**: a short departure push
    left a Moho transfer 4 million km off and the linear correction asked
    5-17 km/s a tick. Lambert re-target when the miss exceeds min(20 SOI, 1%
    of the target's distance) -- Jool's SOI let a 40 Gm miss through 20 SOI
    -- or the correction exceeds 150 m/s; aimed at the B-plane point (aimed
    at the centre it fought the correction sixty times). Excluding near-180
    deg arcs changed nothing (not the cause).
11. **Moho**: the slow-arrival correction (cut v_inf 3497 -> 1379) is a
    different trajectory class; tried as a correction it spent 7.5 km/s.
    Captures needing more than a 30% cut are refused at planning, with the
    acceleration that would capture (Moho: TWR 0.177).

Result: strong engine (TWR 0.04) arrives at Mun and Minmus over 6 start
dates, Duna/Eve/Ike/Dres over 3, Jool and Eeloo; Moho refused. Numbers in
lowThrust/CLAUDE.md.

## Session, 2026-10-13: the capture starts when it must

The user: Moho "supposedly is refusing to capture ... don't [start only
on SOI entry], start it when needed"; later, "just get it in a stable
orbit then adjust" (it already did: capture to 0.3 SOI, FINAL shapes).

The refusal was the one-pass model (`CAPTURE_ARC_DEG` 60, `CAPTURE_FILL`
0.8): at 0.4 m/s^2 Moho's v_inf 3497 needs ~2.4 h and ~15 Mm of braking;
the SOI is 9.6 Mm. Approaches, in order:

1. **Retrograde to the end, started at the latest feasible moment**
   (searched in prediction). Ran on past periapsis and sat at apoapsis
   just above the target ellipse (retrograde there lowers only the
   periapsis) -- fixed by accepting any bound orbit below the moon-clear
   ceiling once past periapsis. But the feasible starts were a window that
   shrank to a knife edge as the engine weakened (+-12% of the burn at
   0.4 m/s^2, one point at 0.04), fragmented, and the "later fails / now
   works" trigger fired on a hole 41 Mm out; started early, retrograde
   stopped the vessel dead 12 Mm out and then burned 5 km/s hovering as it
   fell back. **Abandoned: deciding from a predicted start time.**
2. **The energy law** (kept): thrust along the velocity removes specific
   energy at a per metre of path wherever it is flown, so with q = energy
   to shed / (a x path left to periapsis) it starts at q = 0.8 and throttles
   at q (holding q, finishing at periapsis, one steady burn instead of 111
   pulses). Short captures still burn centred on periapsis.
   - Path budget out to `ra_ok` as well as in: Moho 3580-3910 m/s over
     0.02-0.8 m/s^2; to periapsis only: 3060-3430, flat in k 0.7-0.9 (the
     slow outbound tail is the dearest place to shed energy).
   - The periapsis term drowned under a retrograde vector sized at the whole
     need (2600 m/s): a capture sank 40 km into Moho. Retrograde sized to one
     tick's dv; below the floor, lift only; steps <= 5% of the speed (a 370 s
     step near apoapsis was 150 m/s, past the gradient's reach).
   - "Captured" with the periapsis at -25 km: now only above the floor.
   - Acting on the two-body periapsis from 14 SOI out started the capture
     138 Mm early: before the start, the cruise corrections own the aim.
3. `orbit.propagate` diverged (to 1e17 m) on a 63 s coast 72 Mm out on a
   hyperbola: the long-arc starting guess; now the better of two.

Result (default vessel): Moho arrives, 6371 m/s, capture 3053 from 17.8 Mm
(1.8 SOI). Jool 6311 -> 5922; others within +-17. At 0.04 m/s^2 Dres, Duna,
Eeloo, Ike newly arrive; at 0.1 Dres and Moho. Moho at 0.04 is refused: its
capture would start 272 Mm out, past the 249 Mm where the Sun's tide is 10%
of the engine; with that cap lifted it flew on plan and failed mid-capture
(0.04, 0.02), never arrived (0.01) -- the cap is where the method ends.
Very weak engines need a heliocentric rendezvous leg (open item 1).

## Session, 2026-10-13 (continued): TWR 0.004 has to work

The user's craft has a Kerbin TWR ~0.004 (0.04 m/s^2): "this really needs
to work". Then: the stand-in's finite propellant is unnecessary for
pathfinding (deleted); test 10 m/s^2; think about aerobraking (notes in
lowThrust/CLAUDE.md). Method: grid sweeps (9 destinations x 5-8 thrusts x
6 start dates, 270-432 flights, ~10 min on 14 cores), read every failure.

1. **MATCH** (kept): an arrival too fast for an in-SOI capture is braked
   about the star in the cruise (ZEM/ZEV onto a point just outside the SOI
   with the capture's impact parameter). Timing from the predicted
   encounter -- the instantaneous relative velocity 19000 Mm out read
   20 km/s and started it at once -- plus the standoff (without it, 0.4 m/s^2
   shed 181 of 1766 m/s). The capture law now runs only inside the SOI:
   started outside on a planet-only prediction, a 0.04 m/s^2 capture 11 SOI
   from Moho was wrecked by the Sun's tide. Deleted: the tidal-reach
   constant, the 30% arrive-slow correction (vinf_max). Moho 0.04: 4/4.
2. **Minmus past the Mun** (kept): the waiting orbit stays below the Mun's
   band, periapsis raised until the push fits one pass. 30/30 over
   0.02-0.4. Applied only when the default orbit's push did not fit: -40
   m/s on strong engines but lost 3 of 12 dates (refuted).
3. **Finite propellant deleted** (the user): Vessel(mass, thrust, isp),
   constant mass.
4. **Departure** (kept): a multi-pass departure ending bound with its
   apoapsis past the SOI left in WAIT and "departed" from the Sun's
   periapsis (3.7 km/s, once into Jool's SOI); any departure-phase SOI exit
   now hands to CRUISE. Departure passes checked for moon hazards along the
   RK4-flown burn (an impulse check let a 229 m/s escape meet the Mun).
   Signed true anomaly in DEPART's arc test (unsigned, a long weak push
   ended every minute). Refuted: limiting each pass to the arc (the default
   engine's v_inf fell 1623 -> 765, Moho +650 m/s); a one-pass escape from
   a raised parking periapsis (same 9-10 failures over 108 weak flights,
   10-20% dearer once the hand-over existed).
5. **Bugs the grid found**: guidance's plane gate and apsis arc measured
   the periapsis differently (4000-day PARK deadlock); the cruise's
   re-target horizon counted from the window's arrival (phantom 2 Gm miss,
   Eeloo re-targeted into a failed capture); Lambert overflowed before its
   limit check; APPROACH and CAPTURE crashed or gave up inside a moon's SOI
   (they coast it now); moon passes arriving faster than the capture takes
   are skipped (bar +25% a skip; a fixed bar skipped Mun passes for ten
   years at 0.02).
6. **Refuted at the end**: limiting the in-SOI capture to start inside the
   moons (MATCH the rest) -- Ike at 0.04 fell 6/6 -> 2/6, a slow MATCH
   arrival into Duna not steered well enough.

Result (432 flights): 0.04 m/s^2 54/54, 0.1-10 m/s^2 all arrive; failures
only at 0.02 m/s^2 in moon systems (Duna 4/6, Ike 0/6, Jool 0/6). Table in
lowThrust/CLAUDE.md.

## Session, 2026-10-14: 10 m/s^2, Gilly and Jool's moons

1. **10 m/s^2** (the user asked): all destinations arrived but dearer
   than 2 m/s^2 (Ike 2191 v 1933): the linear throttle over a 300 m/s
   tick overshot the waiting orbit into the Mun's band. A thrusting tick
   now changes the velocity by at most 2% -- Ike 1795, Minmus 1278.
2. **Gilly and Jool's moons added to the stand-in** -- a 64-flight grid
   then broke a lot: a TypeError in planning (capture feasibility not
   monotone in arrival speed), every Laythe pass rejected by a fixed
   400 m/s push cap (now 2x the push to the moon's orbit), Jool captures
   predicted at 36 km/s (the path budget in a deep well; a loose capture
   with a low periapsis instead, chosen by dv), a Jool leg flying 10 km/s
   of a "feasible" tight capture (now compared by dv), a Tylo flyby putting
   Jool's periapsis at -284 km unguarded (floor guard), Gilly's 400 m/s
   arrivals into a 126 km SOI (MATCH for moons), a stale encounter starting
   MATCH 59 h in the past, years of FINAL at +-8 deg arcs (widening arcs).
3. **LOOP** for a weak engine at a many-moon planet: the classic route's
   descent from a loose capture to a deep waiting orbit crossed every
   other band for years and ended in Laythe or Jool. Loose capture on the
   moon's orbit, plane only, one apoapsis burn a revolution to time a pass,
   MATCH in Jool's frame. Laythe 0 -> 2/6; Vall 0/6 (MATCH deep in the well
   starts too late). Applied also to strong engines (whenever loose was
   cheaper) it broke Laythe 6/6 -> 3/4: only when loose is forced.
   MATCH_SHARE 0.5-0.6: no better (refuted).

Grid (180 flights, 15 destinations x 0.04/0.4 m/s^2 x 6 dates): 170 arrive.

## Session, 2026-10-13 (evening): every destination at every thrust

The user: "keep working on the solver ... make sure all issues are fixed.
if some manoeuvres truly cannot be executed with really low thrust
reasonably, then that's fine. but if it's a control issue, fix it."
Method: the 270-flight grid (15 destinations x 0.02/0.04/0.4 m/s^2 x 6
dates, ~4-5 min on 14 jobs) after every change, the failures read whole;
a 630-flight sweep (0.02-10 m/s^2) at the end. Baseline (new dates):
217/270. Every failure turned out to be control, not physics.

Kept, in the order found:

1. **A capped pass delivers its cap exactly**: the cap was not applied on
   the burn's first tick (5.3 m/s flown as 6.1; high up 1 m/s moves the
   next pass by days -- straight back onto Ike).
2. **Widest arcs while the orbit crosses another moon's band**: a Duna
   descent at +-8 deg spent 300 days crossing Ike's orbit.
3. **Aim at the SOI entry** (`aim_error`): the B-plane aim was an arrival
   from infinity; a 75 m/s arrival at Ike aimed 1565 km out, outside the
   1050 km SOI, and the correction steered away (ten pushes undone, ~450
   m/s). A straight-line test (`bt.py`) showed neither formula predicts the
   periapsis of a slow arrival -- the coast curves through the planet's
   field. Now the patched-conic entry state is aimed for rc. Mun 0.04
   1630 -> 1355 on one date (no MATCH, no APPROACH).
4. **Capture sheds energy without lowering the periapsis** once at or
   below the aim (retrograde less its periapsis-gradient part): Duna 612
   -> 566 m/s, periapsis held at 266 km instead of the 80 km floor.
   ("89% radial" was a misreading: far out, retrograde *is* radial.)
5. **Waiting orbits judged by the capture actually flown**: one-pass only,
   Laythe and Vall at 0.04 had no waiting orbit at any height and fell back
   to 230 km over Jool -- the fastest arrival, under every other moon.
   Now one pass or the long capture; from above, the highest gap between
   other moons' bands under the moon; nothing capturable: the slowest.
6. **LOOP deleted**: without it Laythe 0.04 arrived 4/4 (7467 m/s mean)
   against 2/4 with it (one at 15.7 km/s).
7. **MATCH gives up** at twice the speed it set out to shed (co-orbital
   with Vall it chased a receding encounter for 3807 days, 37 km/s).
8. **Polar capture** for a free plane at a planet with moons: Jool 0.04
   5/6 -> 6/6 (8008 -> 7353 m/s), 0.02 2/6 -> 5/6.
9. **Push points every 30 deg**, finite pushes, the point locked and far
   enough ahead to start half the burn early: from periapsis alone a
   near-circular waiting orbit met the Hohmann phase once a moon
   revolution and arrived 120-320 m/s where 80 would do (Mun 0.02 1828 ->
   1662 m/s, 136 -> 43 days); a 2.4 h push to Laythe 6 h away, planned as
   an impulse, chased a growing remainder into the moon.
10. **The pass guard flies the arc by the guidance law, closed loop**:
    the impulse at the arc's start was days off on the next pass; the
    law's first-tick estimate said 42 m/s where 121 burned.
11. **Phasing trims**: also for the rest of the current revolution
    (trimmed now); sized by the shift that slips past a moon's SOI (8 m/s
    moved a pass 5 min where 25 were needed); at least 0.1 m/s and at most
    once per half revolution (keyed by the periapsis time, each trim moved
    the key: +-0.01 m/s for years).
12. **APPROACH dodge**, only for a harmful flyby (patched-conic
    prediction), costed with the plane change it adds (a 10 m/s^2 Duna
    arrival ran through Ike into Duna; dodging every flyby cost Ike
    arrivals 900 m/s turning back into Ike's plane).
13. The too-fast pass bar rises 25% a skip, at most once a synodic period.

Refuted: the hazard predictor (time-uniform sampling of an eccentric
orbit, suspected of jumping periapsis passes) -- 600 random Jool orbits,
identical detections; a fallback push re-solve over the whole range
(re-rooted the push into a chase); limiting a weak departure to its
+-60 deg arc (v_inf 765 even at 0.4 m/s^2, a Mun hit -- the long burn is
what builds the escape energy); the 2%-speed rule for off-periapsis push
points (a stand-in for the late start, fixed at the root).

Result: 270-flight grid 217 -> 269/270; the 630-flight sweep over
0.02-10 m/s^2: 630/630 arrive (table in lowThrust/CLAUDE.md).

## Session, 2026-10-13 night -- 14 morning: the pilot in game

The user: "implement the low thrust autopilot into the game", on their
craft (`qs_lowthrust`: 29.9 t, one Dawn 2 kN, Isp 4200, 12 RTGs, 1 kEC
battery, two inline wheels, heat-resistant nose cone). Plugin allowed,
"just don't make it cheaty in any way".

Built: `live.py` (bodies from kRPC state vectors, frame swap), `pilot.py`
(`Mission.run` split into `step`/`account`, the 270-flight grid
identical), `tools/ltfly.py`, `testLive`, a PILOTS line, and the plugin
`LowThrustWarp` (kRPC service `LowThrust`).

What the game forced, in order:

1. **PT did nothing** on the first save: made before PT was installed, no
   `PersistentEngine` (PT finds engines only in OnLoad). The user rebuilt
   the craft.
2. **kRPC warp zeroes the throttle and refuses warp while it is open**:
   the plugin sets PT's throttle and the rails warp itself.
3. **PT steers only to SAS modes** on rails: the plugin turns the vessel
   with its wheels (torque, MOI, EC measured unpacked). A bang-bang law
   chattered at 1.7 EC/s holding still; the proportional zone (angle/4 s,
   0.5 s response) holds at 0.
4. **Warp rates learned from the easing `warp_rate` were wrong** (10x where
   50x was allowed): read from the game (`RailsWarpRates`). Easing between
   rates every tick ran a 50x warp at 33x: set at once. A Mun flight
   crashed when kRPC's MaximumRailsWarpFactor read 0: `SetRailsWarp`.
5. **The pace**: rails frames run at RAILS_PACE x KSP's (Unity timeScale)
   with exact holds. The game froze at timeScale 0 (kRPC serves from
   FixedUpdate): the plugin pumps it from Update. Holds overshot 40-160 s
   (sized from the last frame; frames swing 2-3x; `maximumDeltaTime` had
   no effect) and cut burn arcs: sized from the longest recent frame
   x 1.5, half the steps left at most -- 0-3 s. The 150 km raise: 724 s
   wall -> 90 s (373x). RAILS_PACE 50 gave 447x v 411x at 10: per-tick
   overhead (~0.1 s) is the limit, not the CPU.
6. **The ledger read 6% low** (throttle x time): dv from the xenon used,
   after the hold has settled.
7. **A Mun flight hung** (LOG10129): a coast hold over 30 s wall tripped
   the orphan release and the pilot waited for a hold nobody would set.
   `Held` (streamed) now keeps a hold alive; a dropped hold is re-issued.
8. **The pump flew at 50x all the way round a 5000 km orbit** (rate set at
   periapsis's limit, KSP only lowers it): the plugin climbs back to the
   rate asked for. The old code timed out at 2 h still pumping
   (LOG10120); the new arrived in 15.6 min (LOG10131).
9. **Burn ticks skipped near a high periapsis**: an inertial command
   fell ~6 deg behind prograde in a 26 s tick, past ALIGN_DEG 2. Burns now
   point fixed against the orbit (kRPC orbital frame). FINAL 26 -> 12-19
   days; PUMP unchanged (~54 d), so that gap is something else.

Results (Mun 50 x 50 from LKO 87 km; stand-in for this craft: 1319 m/s,
44 days -- PUMP 808 m/s in 35 days, FINAL 230 in 7):
LOG10131 1392 m/s 77 d; LOG10133 1372 m/s 69 d; LOG10132 1388 m/s 83 d;
all 15 min wall, PUMP at 1400-1640x. In all three MATCH dropped the Kerbin
periapsis to 66 km ("lifting it") before the Mun SOI took over.

Retracted: "the craft cannot power the Dawn". PT draws its EC on rails
(9.2 EC/s with the wheels, xenon at the right 0.049 kg/s) and the battery
stayed full because the craft has **twelve** RTGs (9.0 EC/s); a set of
part titles had shown one. The user caught it.

EC audit (the user: "are you sure the plugin isn't cheaty?"), battery set
to 500 in a scratch save: RTGs +8.97 EC/s in physics and on rails coasting;
at full throttle on rails EC rose +3.1-3.6 EC/s while PT gave only
1231-1364 N (0.62-0.68 of full) with xenon in proportion -- PT ramps the
thrust after the throttle is set. Energy balances both ways: PT draws EC
with the xenon, generators credit on rails. The flat 996.3 seen before was
the battery full less one 1 s frame of draw at 50x.
