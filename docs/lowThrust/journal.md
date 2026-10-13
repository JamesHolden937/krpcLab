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
