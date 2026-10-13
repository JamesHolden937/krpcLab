# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in**, optionally **within a time budget**: the solver works
out where to burn, which way and how hard, and (once the game side exists)
the executor flies it under rails warp with Persistent Thrust so a transfer
takes wall-minutes. The root `CLAUDE.md` (conventions, farm, cross-cutting
rules) applies here in full.

**State (2026-10-12): apsides and plane changes solved in the stand-in
simulator; no kRPC code yet.** History and every refuted approach: `docs/lowThrust/journal.md`.

```bash
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500"
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500" --within 2d --table
./lowThrust/tools/simfly.py --start "ap=1000 pe=80" --target "pe=300"      # one apsis, other free
./lowThrust/tools/simfly.py --start "ap=100 pe=100 i=6 lan=30 argpe=0" --target "ap=500 pe=500 i=0"
./lowThrust/tools/simfly.py ... --thrust 200 --dry 4000 --prop 1000 --isp 4200
.venv-pypy/bin/python -m unittest lowThrust.tests.testOrbit lowThrust.tests.testGuidance
```

Altitudes in km, angles in degrees (start: ap pe i lan argpe; target: ap
pe i lan; anything left out of the target is free); `--within` takes
s/m/h/d (d = a 6 h Kerbin day). Output
gives the dv against the two-burn impulsive (Hohmann) reference, the final
apsides, and the burn list.

## The stand-in (why a sim first)

Vacuum two-body plus a constant-thrust engine is physics a small simulator
gets *exactly* -- unlike the spaceplane's aero -- so the guidance is built
and regression-tested offline, and the farm will only be needed for what
the sim cannot know (Persistent Thrust's warp behaviour, EC, attitude
holding). `sim.py`: Kerbin's mu/radius/atmosphere, a 5 t probe with one
Dawn (2 kN, Isp 4200 -> 0.4 m/s^2). Guidance is asked once per tick
(period/180) and held, as the live pilot will hold a command between kRPC
calls; coasting ticks are propagated exactly (`orbit.coast`), thrusting
ones by RK4.

## The law (`guidance.py`): Hohmann in many small burns

- **Decoupled**: a tangential burn at periapsis moves the apoapsis and, to
  first order, not the periapsis; at apoapsis, the reverse. The periapsis
  arc drives the apoapsis to its target, the apoapsis arc the periapsis;
  the last pass nulls exactly.
- **Fixed apse line**, held through each burn and re-locked to the real
  periapsis between burns (`lock_apse`); the law controls the radius
  *toward* those fixed directions, `p / (1 + e.u)`, smooth through circular.
- **Arcs** of half-width `acos(eta)` round each apsis; a burn starts half
  its duration before the apsis and runs at full throttle until done or the
  arc ends.
- **Order**: outward first, inward first (raise pe only once ap is up;
  lower ap only once pe is down).
- **Plane change**: normal/anti-normal arcs at the nodes with the target
  plane (fixed once: with `lan` free it keeps the current node line, or on
  an equatorial start puts the nodes on the apse line when the plane change
  begins). Changed on the biggest orbit -- after the apoapsis is raised,
  before it is lowered -- at the high node only (the low one skipped when
  below eta of it), and while the periapsis still changes only at the
  apoapsis-side node, where the two burns overlap.
- **Overlapping burns are one vector** in proportion to what each needs,
  started half the *combined* duration early.
- Pure apsis burns are prograde/retrograde and pure plane burns
  normal/anti-normal -- plain SAS modes; a combined burn points between.

`plan.py` flies the law in the sim over a grid of arc widths (even in
half-width, `ETA_MIN`..`ETA_MAX`) and takes the narrowest that meets the
deadline; no deadline flies `ETA_DEFAULT` (0.99, +-8 deg). Infeasible
deadlines are reported with the fastest transfer. The plan *is* the law
flown forward, so its dv/time/xenon predict what the vessel will fly.

Measured (default cutoff; `testGuidance` pins the first six):

| transfer | dv vs impulsive | time |
|---|---|---|
| 100 -> 500 circ | +0.2% | 14 orbits |
| 500 -> 100 circ | +0.2% | 7 |
| 2000x100 -> 2000 circ | +1.6% | 4 |
| 100 -> 2000x100 | +0.5% | 38 |
| ap only 100 -> 1000 | +0.1% | 18 |
| pe only 80x1000 -> 300 | +0.2% | 0.5 |
| 100 -> 3000 circ | +3.1% | 72 |
| 300x800 -> 600x200 | +0.0% | 2 |
| same, 0.04 m/s^2 engine | +0.2% | 132 |

Plane changes (impulsive reference: at the actual nodes for a pure plane
change; Hohmann with the plane folded into the larger-radius burn
otherwise, a lower bound when the nodes are off the apse line):

| transfer | dv vs impulsive |
|---|---|
| 500 circ, i 0 -> 10 | +0.6% |
| 2000x100, i 0 -> 5 (one burn at apoapsis) | +0.1% |
| 2000x100 i 10, nodes at +-90 deg -> i 0 | -0.6% (at its nodes) |
| 500 circ i 30, lan 0 -> 40 | +0.8% |
| 100 -> 500 circ, i 0 -> 5 | +1.2% |
| 3000 -> 100 circ, i 0 -> 20 | +6.7% |
| 100 -> 3000 circ, i 6 -> 0 | +8.4% (open: the apse drifts off the fixed nodes) |
| 100 -> 500 + 5 deg, 0.04 m/s^2 | +1.1% |

The time/dv trade on the 0.04 m/s^2 engine, 100 -> 500: cheapest 41 days
at +0.0%, 1.1 days at +14%; a 2-day budget picks +6.7%.

## Interplanetary: the plan (the user's goal)

Pump the parking orbit with small periapsis burns well before the window
(e.g. 100 x 9000 km, under the Mun's SOI), orient it for the departure
asymptote of the window date (KSP has no precession, so an early setup
stays put), then give the final push at periapsis. Every burn is at
periapsis, so the total equals a one-burn chemical departure (Duna: 821
pump + 240 push = 1061 against 1060) and the arrival date is the same. The
catch: **the final push must fit one periapsis pass** (~10 min within +-60
deg on that ellipse):

| target | final push | TWR 0.04 | TWR 0.004 |
|---|---|---|---|
| Eve | 204 m/s | 8 min | 85 min |
| Duna | 240 | 10 | 100 |
| Dres | 735 | 31 | 306 |
| Jool | 1111 | 46 | 463 |

Past what one pass allows, push what fits and make up the v_inf with the
ion engine early in the heliocentric cruise. Capture is the mirror problem
(Duna can aerocapture).

## Next

1. **Argument of periapsis** in the target (sets the apse line) -- needed
   to orient the parking ellipse for a departure.
2. **The 100 -> 12000 km transfer never arrives** (20000-orbit limit) at
   the default cutoff: a bug in the law for big ratios, found by the spiral
   test; find it before trusting large transfers.
3. **A real spiral as the planner's fast end** (continuous tangential,
   ending circular, a periapsis floor): ideal cost |v1 - v2| (+1.3% for
   100 -> 500, +16% for 100 -> 3000), 3 h instead of 72 at TWR 0.004. The
   naive version left e 0.03 and cost +7%.
4. **The PT probe** (`tools/ptprobe.py`, an ion craft, an LKO save): does a
   kRPC throttle persist into rails warp; do SAS prograde/retrograde/normal
   hold on rails; delivered dv per warp rate; EC through a shadow pass. A
   combined burn needs an arbitrary direction (SAS maneuver mode + a node,
   or drop out of warp to point), so probe that too.
5. **The pilot** (`pilot.py`, the only kRPC-aware code), as planned: the
   measured acceleration into `guidance.command`, SAS mode / throttle /
   warp, panel with target fields, PLAN, START/TERMINATE, `config:` line,
   fingerprint, `rpccount`, a `PILOTS` line. Re-plan against
   `deadline - ut` every few orbits.
6. Interplanetary: departure planning (window, v_inf, asymptote ->
   parking-ellipse orientation), the final push, heliocentric correction,
   capture.

## Traps known

- **Under PyPy, `is` compares floats by value**: `goal is target.rp`
  deadlocked every circular target. Use explicit flags.
- **Near-circular, the osculating apsides are not places**: a burn short of
  apoapsis turns the apse line as fast as the vessel moves, so any law
  chasing osculating apsides (or the Q-law's effectivity peak) surfs the
  arc's edge. Hence the fixed, re-locked line.
- **A throttle re-evaluated against a sliding window never finishes** (the
  remainder decays exponentially). Any spreading must be against a fixed
  endpoint; the half-duration-before-apsis start needs none.
- **Wide arcs are not just faster, they are each a big change of orbit**:
  eta 0.95 cost +20% on one transfer where 0.99 cost +2. Time per dv is not
  monotonic in the cutoff -- the planner scans, it does not bisect.
- **Power and warp** (game side): an ion engine draws ~8.7 EC/s, shadow
  passes on batteries; PT integrates once per frame. The guidance must use
  the thrust the vessel *gets*.
- **Live install differs from the farm** (~50 extra mods); the live install
  needs PT 1.7.5.
