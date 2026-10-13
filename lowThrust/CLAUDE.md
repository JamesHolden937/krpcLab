# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in** -- about the same body, one of its moons, or another
planet (or that planet's moon) -- optionally within a time budget. The
solver works out every burn; the executor (not written yet) will fly them
under rails warp with Persistent Thrust. The root `CLAUDE.md` applies here.

**State (2026-10-13): the solver works end to end in a stand-in solar
system (patched conics, stock bodies on rails); no kRPC code yet.**
History, every refuted approach and why: `docs/lowThrust/journal.md`.
What the code does and what triggers what: `docs/lowThrust/howItWorks.pdf`.

```bash
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "mun: ap=50 pe=50"
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "duna: ap=200 pe=200 i=10"
./lowThrust/tools/simmission.py --from "kerbin: ap=100 pe=100" --to "ike: ap=50 pe=50"
./lowThrust/tools/simmission.py ... --thrust 200 --dry 4000 --prop 1000 --isp 4200 --ut 3000000
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500 i=5" --within 2d   # one SOI, with a deadline
.venv-pypy/bin/python -m unittest lowThrust.tests.testOrbit lowThrust.tests.testGuidance lowThrust.tests.testMission
```

Orbits: `body: ap= pe= [i= lan= argpe=]`, km and degrees; anything left out
of the target is free. `simmission` prints the phase timeline (Kerbin dates),
every burn (phase, body, start, duration, dv, prograde/normal/radial share),
dv by phase and the totals. Default vessel: 5 t, one 2 kN Dawn, Isp 4200
(0.4 m/s^2, TWR 0.04).

## Files

| file | what |
|---|---|
| `orbit.py` | two-body maths: elements, any conic, universal-variable propagation, time to periapsis, Lambert, radius toward a direction, impulsive references |
| `guidance.py` | the within-SOI law: Hohmann in many small burns + plane change at the nodes (+ a held apse line) |
| `plan.py` | within-SOI deadline planner: the narrowest arcs that meet a time budget |
| `bodies.py` | the stand-in system: stock bodies on Keplerian rails, SOIs |
| `transfer.py` | targeting: closest approach / B-plane miss, mid-course correction, moon push, porkchop window, parking orientation, capture design, Lambert re-target, SOI hazards |
| `mission.py` | the phase machine for a whole transfer (patched conics), routing and leg chaining |
| `sim.py` | the single-body stand-in (`fly`) and the shared RK4 / tick |
| `tools/simfly.py`, `tools/simmission.py` | command lines |

## The phases

```
within SOI:   FINAL
to a moon:    PUMP -> WAIT -> PUSH -> COAST -> APPROACH -> CAPTURE -> FINAL
to a planet:  PARK -> WAIT -> DEPART -> ESCAPE -> CRUISE -> APPROACH -> CAPTURE -> FINAL
a planet's moon: the planet leg ends in the moon leg's waiting orbit, then the moon leg
anywhere:     TRIM (phasing, to miss a moon), evasive burns in an unplanned SOI
```

- **Within SOI** (`guidance`): prograde/retrograde arcs centred on a fixed
  apse line (re-locked between burns), each fixing the opposite apsis;
  outward-first order; plane change at the nodes on the biggest orbit;
  overlapping burns one vector. Within +0.0-3.1% of the impulsive Hohmann
  on apsides changes, +0.1-8.4% with plane changes.
- **Moon**: pump to a waiting orbit in the moon's plane (apoapsis 0.6 of the
  way; periapsis raised only as far as a weak engine needs to capture), wait
  for the pass whose prograde push puts a *direct* encounter on the B-plane
  aim point (passes whose coast meets another moon are skipped), push on a
  ledger, correct mid-course, nudge the hyperbola's periapsis, capture
  (below), then FINAL.
- **Planet**: porkchop window after the pump's estimated end; a parking
  ellipse whose periapsis push leaves on the departure asymptote (plane
  chosen so its change is cheapest); push at the periapsis nearest the
  window (multi-pass if the engine is too weak to escape in one); B-plane
  corrections in the cruise, or a Lambert re-target (aimed at the B-plane
  point) when the miss is too big to linearise; capture designed from the
  window's arrival speed -- high and loose if the engine is weak; if no
  one-pass capture exists, a long capture (below), flown in prediction at
  planning; only if that would start past the star's tidal reach, the
  cruise cuts v_inf by up to 30%, refused at planning beyond that.
- **Capture** (`transfer.capture_step`, shared by flight and prediction):
  it only has to end *bound* (below 0.3 SOI, or below the moon-clear
  ceiling once past periapsis, periapsis above the floor); FINAL shapes the
  orbit. A capture that fits one pass burns centred on periapsis. A longer
  one starts when it must -- from COAST/CRUISE, outside the SOI if need
  be: thrust along the velocity removes energy at `a` per metre of path,
  so with q = energy to shed / (a x path left to periapsis) it starts at
  q = `CAPTURE_PATH_SHARE` (0.8) and throttles at q, finishing at
  periapsis with a steady throttle. The periapsis is held in a band
  (floor .. 1.5x aim). The prediction ignores the parent's tide, so a start
  farther out than where it reaches 10% of the engine is refused.

## Measured (stand-in, default vessel, from LKO 100 km)

| destination | dv | trip | notes |
|---|---|---|---|
| Mun 50 km | 1108-1123 m/s | 11-17 d | 6 start dates |
| Minmus 30 km | 1195-1240 m/s | 69-117 d | 6 start dates; one 1513 (a Mun slingshot survived) |
| Duna 200 km | 1810 m/s | 364-503 d | 3 dates; the latest catches the next window (2184) |
| Eve 150 km | 2952 m/s | 481-759 d | parking plane change ~185 m/s (window declination -27 deg) |
| Ike 50 km | 1869-2254 m/s | via Duna | |
| Dres 50 km | 4778 m/s | | departure push short at 0.4 m/s^2; re-targeted |
| Jool | 5922 m/s | | re-targeted after a short push; long capture |
| Eeloo | 4953 m/s | | re-targeted after a short push |
| Moho 50 km | 6371 m/s | 135 d | long capture 3053 m/s, from 17.8 Mm (1.8 SOI), one 2 h 15 m burn |
| 0.1 m/s^2 | Mun 1211, Minmus 1519, Eve 3792, Dres 5157, Jool 6533, Eeloo 5636, Moho 7512 | | Duna, Ike **fail** (departure, see Open) |
| 0.04 m/s^2 (TWR 0.004) | Mun 1478, Eve 4731, Duna 2929, Ike 3640, Dres 5686, Jool 7542, Eeloo 6233 | | Minmus **fails** (Mun crossings); Moho refused (capture would start 272 Mm out, reach 249) |

Long-capture law alone at Moho (v_inf 3497, impulsive 2617 m/s): 3030-3400
m/s over 0.8-0.02 m/s^2, starting 10-385 Mm out.

## Open, in order

1. **Very weak engines to a fast-arrival planet** (Moho at <= 0.04 m/s^2):
   the capture would have to start 270+ Mm out, where the star's tide
   breaks the destination-only prediction -- flown anyway (cap lifted) it
   started on plan and failed mid-capture at 0.04 and 0.02, never arrived
   at 0.01. Physically reachable; the method is wrong for it. Needs a
   continuous-thrust heliocentric leg that matches the planet's orbit
   during the cruise (spiral rendezvous, arrival v_inf ~0), as real ion
   probes reach Mercury -- a new phase, not the 30% "arrive slow"
   correction (which spent 7.5 km/s on Moho).
2. **Weak engines crossing a moon's orbit** (TWR 0.004 to Minmus past the
   Mun, Duna capture spiralling through Ike's orbit): dozens of crossings;
   the pass caps and trims cannot always find a clear revolution, and the
   recovery after a slingshot is too slow for the engine. Either time the
   pump's crossings (choose the waiting orbit's period to be non-resonant
   with the moon) or cross the band in fewer, larger passes.
3. **Multi-pass departures that leave during WAIT** (0.1 m/s^2 to Duna and
   Ike): a pass leaves the apoapsis at ~92000 km, past Kerbin's SOI, the
   vessel drifts out while waiting for the next pass, and DEPART then
   burns about the Sun (3.8 km/s, out of propellant). Cap each pass's
   apoapsis inside the SOI, or let the last pass escape.
4. **Short departure pushes** at 0.4 m/s^2 for high v_inf (Dres, Jool,
   Eeloo, Moho): the 60 deg arc cuts the push and the cruise re-targets
   expensively (Dres 4778 m/s). A longer push, or a parking ellipse beyond
   the moons (guarded) to shrink the periapsis push.
5. **The Persistent Thrust probe** (`tools/ptprobe.py`, an ion craft and an
   LKO save): throttle persistence into rails warp, which SAS modes hold on
   rails (prograde/retrograde/normal and the maneuver mode -- combined burns
   and corrections need arbitrary directions), dv per warp rate, EC in
   shadow, kRPC orbit reads during warp.
6. **The pilot** (`pilot.py`, the only kRPC-aware code): run `Mission`'s
   phase machine against kRPC (live bodies and orbits, measured
   acceleration), panel with target fields, PLAN (the stand-in flight as
   the preview), START/TERMINATE, `config:` line, fingerprint, `rpccount`,
   a `PILOTS` line. Warp from the tick each phase needs (lower near
   periapsis: ticks are uniform in angle).
7. Routes not yet supported: from a moon (Mun -> Kerbin, Mun -> Minmus,
   moon -> planet).

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
- **Live install differs from the farm**; the live install needs PT 1.7.5.
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
