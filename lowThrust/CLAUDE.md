# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in**, optionally **within a time budget**: the solver works
out where to burn, which way and how hard, and (once the game side exists)
the executor flies it under rails warp with Persistent Thrust so a transfer
takes wall-minutes. The root `CLAUDE.md` (conventions, farm, cross-cutting
rules) applies here in full.

**State (2026-10-12): apsides changes solved in the stand-in simulator; no
kRPC code yet.** History and every refuted approach: `docs/lowThrust/journal.md`.

```bash
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500"
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500" --within 2d --table
./lowThrust/tools/simfly.py --start "ap=1000 pe=80" --target "pe=300"      # one apsis, other free
./lowThrust/tools/simfly.py ... --thrust 200 --dry 4000 --prop 1000 --isp 4200
.venv-pypy/bin/python -m unittest lowThrust.tests.testOrbit lowThrust.tests.testGuidance
```

Altitudes in km; `--within` takes s/m/h/d (d = a 6 h Kerbin day). Output
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
- **Prograde / retrograde only** -- so plain SAS modes can fly it.

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

The time/dv trade on the 0.04 m/s^2 engine, 100 -> 500: cheapest 41 days
at +0.0%, 1.1 days at +14%; a 2-day budget picks +6.7%.

## Next

1. **The Persistent Thrust probe** (`tools/ptprobe.py`, an ion craft, an
   LKO save in `saves/`): does a kRPC throttle persist into rails warp; do
   SAS prograde / retrograde hold on rails; delivered dv against the rocket
   equation per warp rate; EC through a shadow pass; does kRPC's orbit read
   track PT. The law needs only prograde/retrograde, so executor option 2
   (plain SAS modes) is enough for stage 1.
2. **The pilot** (`pilot.py`, the only kRPC-aware code): reads the orbit,
   calls `guidance.command` with the *measured* acceleration, sets SAS mode,
   throttle and warp rate; panel with target fields, PLAN, START/TERMINATE;
   `config:` line, fingerprint, `rpccount`, a `PILOTS` line in
   `common/launcher.py`. Warp rate from the tick the guidance needs (an arc
   of +-8 deg needs several ticks inside it), and drop out of warp to change
   SAS mode if PT does not follow it on rails.
3. **Re-planning**: the live pilot should re-plan against `deadline - ut`
   every few orbits, so a weak engine or a shadow pass widens the arcs.
4. **Inclination** (and lan/argpe): the next elements. Normal/anti-normal at
   the nodes, the same small-burn structure; `argpe` given by the user just
   sets the apse line. Q-law (in git history, e5f7bea's parent) is the
   general fallback, with the traps listed in the journal.
5. Stage 2/3 (escape spirals, heliocentric, arrival) as in the journal's
   plan.

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
