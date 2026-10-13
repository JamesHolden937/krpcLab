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
