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

## Session, 2026-10-14 afternoon -- evening: a standalone mod

The user: the pilot "is still kind of laggy ... do we even need kRPC or
should we just turn it into a standalone mod?" -- then "make sure low thrust
becomes standalone, and do the lag fix", and later "make sure I can adjust
how much total amount of delta v is expended ... quickly realizes if I ask
for an impossible number ... or maybe there's a better way".

(Earlier the same afternoon, commit 72a8062, unjournalled: the Python pilot
queued each command with the plugin while the one before flew, so the game
no longer stopped every tick, and the plugin got a stock KSP window.)

**Where the lag was.** The pipelined pilot was almost never late (2 stalls in
2122 commands, LOG10147). The lag was `RAILS_PACE` 20: Unity's time scale at
~20, so each rendered frame ran up to ~40 physics steps (kRPC server,
attitude substeps, Persistent Thrust each step). Speed was bought with frame
rate because the burn advanced one physics frame at a time.

**The port.** The solver (orbit, guidance, planner, transfer, mission; 3900
lines) went to C# operation for operation (`common/vec`'s order, Python's
modulo and round). Against the Python on the 270-flight grid (15
destinations x 100/200/2000 N x 6 dates): 270/270 arrive in both, mean dv
4914.3 v 4914.7 m/s, 176 outputs byte-identical, the rest diverging late from
rounding (median dv difference 0.00%, worst 2.4%: a Gilly pass 3 min later),
2.4x faster than PyPy. The 42 Python tests ported and pass. Python retired.

**The mod.** The controller runs in the game's physics step and brings the
vessel to the game's time every step: commands decided as they fall due,
thrust integrated on rails (RK4, the engines' real thrust, Isp and
propellants), the orbit written through its elements. No time-scale change,
no holds, no other process. Results on `qs_lowthrust`:
- 87 -> 150 km: 6 s wall (Python pilot 120 s), 0.4 d as planned (1.6 d).
- Mun: 125-137 s wall (545-557 s), 35.0-35.1 d where planned 35.0, and 20.8
  d where planned 26.9 (73-80 d before): the old "in-game PUMP 54 d v 35"
  gap is gone. Dv 1332-1381 v plan 1297-1311 (+3-6%).
- Duna (first planet transfer in game): arrived, 586 d, 2695 m/s v 2256.

**Traps paid for.**
- Persistent Thrust removes ElectricCharge from the Dawn's `ModuleEngines`
  propellants at run time: the first flights drew no EC. The mod reads the
  propellants from the part config (`propellants:` line in every log).
- `RequestResource(..., simulate=true)` on rails offered next to nothing:
  10% thrust, warp held at 5x. Availability is the vessel's connected
  totals; draws are real requests.
- The farm loads the dll at boot: a rebuilt mod flies only after a restart.
- World space turns with the body below its inverse-rotation altitude: every
  state goes through KSP's orbital elements (frame check 0.000 deg, rails
  check 0 m) and every attitude target is held against the orbit (R, T, N),
  never as a world or inertial vector.
- Frames of 2000 s at 100000x: the attitude model holds a settled target
  exactly instead of integrating 100000 substeps; the governor limits a
  thrusting step to half of any propellant held (the RTGs refill the battery
  between steps) and a turning one to 5 s.

**Refuted: slow decisions on a worker thread with game time held.** The
moon-push search freezes a frame up to ~0.4 s a few times a flight. Made
aside instead: one decision per step let the autopilot's clock fall behind
the game's -- 5/5 Mun captures failed (LOG10158-62); holding time until it
caught up, chained decisions over a long step drained the battery and fired
unaligned (thrust share 0.53-0.95, Mun 40-260 d, LOG10164-68); one decision
per step: 5/5 arrive but 150-344 unaligned ticks and arcs-0.827 plans 53-92
d (LOG10170-74) against inline's 36-41 and 20.8-20.9 d (6/6, LOG10176-81).
Deleted. The next approach to the freeze: make the search incremental.

**The dv/time trade.** The arc cutoff (ETA_DEFAULT) is the lever, and it is
lumpy: to the Mun for this craft 0.9 gives 1295 m/s in 20 d where the
default 0.99 gave 1319 in 44 -- the old default was not on the frontier.
So the mission is flown at 13 cutoffs in parallel (~7 s to the Mun) and the
frontier kept; a budget picks from it (max dv: fastest within; max days:
cheapest within; none: fastest within 2% of the cheapest), or is refused --
at once below 0.8 of an impulsive estimate (1139 m/s to the Mun; the stand-in
flies 1115, so the estimate is not a strict bound), else after the scan with
the achievable range. In game: budget 1350 m/s flew 1351 (plan 1349); max 10
days refused in 14 s ("fastest 13.8 days").

## Session, 2026-10-14 night: steering costs, fast planet transfers, the warp fight

The user asked for steering costs after alignment, and for a high dv limit
to buy high-energy transfers; and suspected BetterTimeWarp's EC toggling of
the lag.

**The lag was the warp governor fighting the game.** The live KSP.log of the
17:50 Minmus flight had no BetterTimeWarp EC messages (its
`DisableECAtHighWarp` is False in every preset) but 1132 "Orbit event
imminent. Dropping TimeWarp to max limit" lines, 50 a second for ~23 s:
KSP's `ClampRateToOrbitTransitions` (decompiled) drops above index 3 once
the patch's SOI change is closer than (TimeScale + rate3)/2 + 50 s, and the
governor raised it back every step. Mirroring the rule cut the farm's drops
from 685/684/706/2096/243/1 (rot-steer-1014) to 288-805 (rot-fast-1014,
still 25/s): the game re-checks a step later (a step's game time gone) and
the hold keyed on an exact EndUT that thrust moves. With a two-step margin
and the drop held by body and time: 4/4/4/1/13/9 (rot-fast2-1014). Mun
wall 127-129 s (was 135-141).

**Steering after alignment** (c72301c): thrust along the axis the attitude
leaves each step (was frozen at the decision), the hold charged as wheel
input for I dw/dt + w x Iw. LOG10182-87: cosine loss 0.01-0.02 m/s a flight
(0.001%), 48-90 m/s flown > 0.5 deg off, worst 2.0 deg (the align gate),
holding 9-21 EC, 0-3 holds lost. Not the cause of dv over plan.

**Fast planet transfers** (5d71966): `TRANSFER_TIME_SHARE` -- the cheapest
Lambert window arriving within a share of the cheapest window's wait +
flight (tof from 0.15 of the Hohmann's); the scan flies each arc width at
`TRANSFER_SHARES` 1/.85/.7/.55/.4 on planet routes (65 plans, ~6 s Duna,
~15 s Ike). Stand-in Duna frontier 314 d / 7813 m/s .. 512 d / 2138 (was
527-542 d, all at the one window). In game: Duna max-dv 4000 433.0 d / 3833
(plan 433.3 / 3842, LOG10193) and 404.8 / 3896 (plan 406.7 / 3906,
LOG10199); defaults 505.8 / 2239 (plan 509.5 / 2173, LOG10192) and 511.9 /
2165 (511.6 / 2151, LOG10198) -- last session's default Duna was 586 d /
2695.

**Minmus inline** (open item 1's re-fly): 1404, 1362, 1742, 1496, 1714,
1433, 1358 m/s (plans 1362-1384) -- three of seven far over, with WAITs of
3-5.6 Ms.

**Paid**: mcs bound `value.Split(',')` to `Split(char, StringSplitOptions)`,
absent from KSP's Mono: every START failed (the first fast-1014 batch flew
nothing, and the live game had that dll for ~10 min). `TimeWarp.fixedDeltaTime`
is scaled by the rate; `Time.fixedDeltaTime` is not.

## Session, 2026-10-14 late: aerobraking, the capture's flip-flop, the Minmus push

The user asked: why, in their live Duna flight
(`flight-20261014-183836.log`), braking did not start until after
periapsis, and why the circularisation burned away from periapsis; then
aerobraking "as aggressively as possible while keeping a safe margin so
nothing burns up"; and the handoff's open items.

**Late braking at Duna: the capture law flip-flopped.** CAPTURE ran 2.2 h
before periapsis but every second tick was unaligned (`thr=0.00/0.23
err=94-107`): the planned periapsis sat exactly on the safety floor
(80 km = atmosphere + 30 km), the retrograde burn lowered it under, the law
then commanded a pure periapsis-gradient burn ~100 deg away (whole 250 s tick
lost turning), lifted it to 100 km, went back to retrograde... Only 382 of
678 m/s were burned before periapsis; 296 m/s after, where dv is dearest.
Fix: the direction now varies smoothly -- retrograde with its pull on the
periapsis faded out toward the aim, a lift term closing the error over the
time left to periapsis, the floor term ramped in over 5% of it, the aim kept
5% above the floor (at the floor exactly, Gilly captures burned forever:
737,000 m/s in the grid); and an unaligned thrust tick now coasts only while
the wheels turn (bang-bang at 0.8 of pitch/yaw + 5 s), not the whole tick.
Stand-in Duna capture 601 -> 542 m/s; grid (270 flights) 268/270 arrive
(the two Laythe flights fail by chaotic Tylo/Vall flybys), mean dv +0.2% --
in the stand-in pointing is instant, so the old law's jumps cost nothing
there; they cost ticks only in the game.

**Burns away from periapsis in FINAL** were by design: arcs widen to +-60 deg
while the orbit crosses Ike's band (fewest revolutions there), a pass's burn
was capped for an Ike encounter, and small apoapsis burns put back the
periapsis a finite periapsis arc lowered (71-78 km -> 80). Aerobraking
removes most of that phase.

**Aerobraking** (`core/Aero.cs`, AEROBRAKE, the Controller's passes): see
lowThrust/CLAUDE.md for the design. What it took to work in the game:
- the first in-game passes were all flown **on rails through the air** (no
  drag, "unpacked at 1168 km"): `TimeToRadius` a hair short of the top
  read a tiny negative time and added a revolution, so the mission never
  stopped at the air; and when the game dropped off rails at its own
  altitude limit, the off-rails branch put the full warp back. Fixed both;
  a pass started off rails close above the air starts there.
- aerobraking ended at once at Kerbin (first-pass heat limit 30 grazes at
  3 km/s): it now keeps passing while the heating limit is still growing.
- a pass must leave a periapsis the engine can lift out within +-60 deg of
  apoapsis (the user's question: with 0.067 m/s^2 a Kerbin exit lift of
  ~20 m/s is 300 s, the default +-8 deg arc 85 s); the exit arc is widened.
- the moon check placed the planned pass at the decision time, not the next
  periapsis; skipping a revolution when no depth is clear of Ike.

Measured (qs_lowthrust): drag area 12.9 m^2 at Duna, 18.6-23.7 at Kerbin,
against 4.74 from the drag cubes; the drag scale settles 1.7-3.6 (higher on
shallow passes). Temperatures: the RTGs and the probe core run hottest.
- Duna 200 x 200 (LOG10206): 14 passes, 1944 m/s, 545.8 d (plan 1909 /
  521); capture 524, AEROBRAKE 14, FINAL 55; hottest 0.44.
- Duna 80 x 80 i0 (LOG10207): 15 passes, 1946 m/s, 536.0 d (the live flight
  without it: 2462 / 561); hottest probe core 0.57.
- Kerbin 3000 x 100 -> 100 (qs_lowthrust_kell): heat-limited near H 100 at
  3 km/s (passes at 63-66 km, ~1-3 m/s each); one pass reached 0.638 of an
  RTG's limit against the 0.6 target (limit cut x0.89). ~45 s wall a pass;
  19 passes took 3000 -> 2550 km, then a pass gained < 0.5% of what was
  left and FINAL burned the rest: 675 m/s against 683 without aerobraking.
  For this craft (RTGs, 1200 K) aerobraking at Kerbin from high orbit does
  not pay. Pass 5 was still flown on rails: the game at 100000x and 2098 s
  past the entry while the governor asked 5x -- unexplained (1 of 48).
- Duna 200 with the fast-transfer plan, earlier (LOG10202): stuck at Kerbin
  after the Mun captured the weak escape -- a departure problem, not this.

**Minmus over plan (handoff item 1): the push's re-solve could not see the
encounter.** On 3 of 7 flights the push ended ~90 s early and COAST found no
encounter (29,000-31,000 km): every re-solve during PUSH returned nothing,
because the horizon (0.75 of the post-push revolution) cut the encounter WAIT
had planned off, so the push flew open loop. The stand-in did the same at arcs
0.988 (155 d planned). The re-solve now looks at least 1.2x as far as WAIT's
encounter: 57 d / 1411 m/s.

**The frame freeze (handoff item 2)**: WAIT's moon-push search is an iterator
run while the step has half its frame budget left, the coast going on and the
warp slowed meanwhile; same evaluations, same answer (stand-in outputs
byte-identical on Mun, Minmus, Ike).
