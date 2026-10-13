# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in** -- the solver works out the steering, the throttle and
when to coast, and the executor flies it under rails warp with Persistent
Thrust so a transfer takes wall-minutes, not hours. The root `CLAUDE.md`
(conventions, farm, cross-cutting rules) applies here in full.

**State (2026-10-12): planned, nothing written.** This file is the plan.
`docs/lowThrust/journal.md` holds the history.

## What the user asked for

> type in a target orbit and the solver should figure out what maneuvers to
> execute

Then, later: interplanetary transfers. Sitting at 1x or physics warp for an
hour per maneuver is not acceptable -- everything must work under rails warp.

## The input: a target orbit

Any subset of the classical elements around the vessel's current body;
an element left out is free (its weight in the solver is zero):

| field | meaning |
|---|---|
| `ap`, `pe` (or `a`, `e`) | apoapsis / periapsis altitude, km |
| `i` | inclination, deg |
| `lan` | longitude of the ascending node, deg |
| `argpe` | argument of periapsis, deg |

e.g. `--target "ap=500 pe=500 i=6"` (circular 500 km, 6 deg, node free).
The in-game panel gets the same fields as text boxes, plus PLAN (preview
dv, time, xenon) and START/TERMINATE. A target the vessel cannot reach on
its remaining xenon must say so at PLAN time -- never fly into a half-done
transfer (root rule: a missing answer must not look like a good one).

## Architecture (the repo's shape)

```
STANDBY -> PLAN -> THRUST <-> COAST -> TRIM -> DONE
                     (warp throughout; re-decided every tick)
```

- **`guidance.py` -- the solver, pure functions, no kRPC.** A feedback law
  on the osculating elements, not a stored schedule: each tick it returns
  the thrust direction in the RTN frame (radial / tangential / normal) and
  whether thrusting *here* is worth it. Candidate laws, simplest first:
  - single-element: tangential for `a`, normal/anti-normal switched at the
    nodes for `i`
  - **Edelbaum** for circular-to-circular with a plane change (closed form)
  - **Q-law** (Petropoulos 2004) for any element subset: a Lyapunov
    function over the element errors, Gauss's variational equations give
    each element's best achievable rate, steer to descend Q fastest, coast
    when the effectivity (achieved rate / best rate on the orbit) is below a
    threshold. This is the target law; the others are its special cases.
- **`trajectory.py` -- the propagator.** Two-body + thrust (+ mass flow,
  + shadow / EC if needed), integrating *the same guidance function* the
  vessel flies. PLAN is just this run to convergence: its result is the
  dv / time / xenon preview. Root rule: the propagator must fly the law the
  vehicle flies.
- **`pilot.py` -- the phase machine, the only kRPC-aware code.** Reads the
  orbit, asks `guidance`, maps the RTN direction to something the vessel
  will hold under warp, sets throttle and warp rate, logs.
- `config.py`, `Logbook`, `config:` line + defaults fingerprint, `rpccount`,
  one line in `common/launcher.py` -- the root "Adding an autopilot" list.

## The executor: steering under rails warp

kRPC does not drive Persistent Thrust; PT (1.7.5, FreeThinker's fork,
already in `testInstances/base/GameData`) applies the thrust on rails and
updates the orbit, kRPC sets throttle / SAS / warp and reads the orbit back.
**The open question is what attitude PT holds during warp.** In order of
preference:

1. **SAS `maneuver` mode + a node placed in the desired direction** -- any
   RTN direction, re-aimed by moving the node. Needs PT to follow it.
2. **SAS modes only** (prograde / retrograde / normal / anti-normal /
   radial) -- enough for `a`, `e` and `i` with switching; Q-law's direction
   would be quantised to the nearest.
3. **Segmented warp** -- drop to 1x every N game-seconds, point, re-warp.
   Works whatever PT does; costs a few wall-seconds per segment.

## First job: the probe (before any guidance)

`lowThrust/tools/ptprobe.py` + a simple ion craft (Dawn engine(s), xenon,
solar + batteries, probe core with SAS) and a save in LKO (`saves/`,
`saves/README.md`). On one farm instance it answers:

1. does a kRPC throttle setting persist into rails warp under PT?
2. which SAS modes does PT hold on rails -- including `maneuver`?
3. delivered dv against the rocket equation, per warp rate (PT integrates
   once per frame; at 100,000x a frame is thousands of game-seconds)
4. EC through a shadow pass: does thrust stop, and does it resume?
5. does kRPC's orbit read track PT's changes during warp?

Its answers pick the executor above and so most of `pilot.py`.

## Stages

1. **One SOI**: raise / lower, circularise, plane change, then any element
   subset by Q-law. Done when typed targets are reached within tolerance on
   several start orbits and the flown dv matches PLAN's.
2. **Interplanetary**: spiral to escape, heliocentric Q-law to the target
   planet's orbit, spiral in. New problem is phasing (the planet must be
   there): a departure-date search over the propagator.
3. **Precise arrival / rendezvous**: trajectory optimisation offline (in
   Python, CPU is plentiful), Q-law as the in-flight correction. Acceptable
   intermediate: get close, finish with a small chemical burn.

## Testing

Vacuum two-body plus thrust is a model a small offline sim gets *exactly*,
unlike the spaceplane's aero: the guidance is developed and regression-
tested offline (`lowThrust/tests/`, no KSP), and the farm is for what the
sim cannot know -- PT's warp behaviour, EC, SOI transitions. Still: one
flight is not a measurement, read signed element errors, and set start
orbits with `tools/savegen.py` / hand-edited saves rather than relying on
one (check periapsis is above the atmosphere).

## Traps known in advance

- **Power.** An ion engine draws ~8.7 EC/s; shadow passes on batteries.
  The guidance must use the thrust the vessel *gets*, read live, never the
  rated figure.
- **High warp coarsens PT's integration.** Pick warp from the guidance's
  needed update interval (as the spaceplane's governor does), and measure
  the dv lost against the rate.
- **Q-law degenerates near e=0 and i=0** (argpe / lan undefined). Use
  equinoctial elements internally and weight undefined elements to zero.
- **Live install differs from the farm** (~50 extra mods); diff GameData
  before chasing a live-only failure, and the live install needs PT 1.7.5.
