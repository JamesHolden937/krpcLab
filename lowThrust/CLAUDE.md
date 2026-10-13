# lowThrust

Flies a low-thrust (ion) vessel from its current orbit to a **target orbit
the user types in** -- about the same body, one of its moons, or another
planet (or that planet's moon) -- optionally within a time budget. The
solver works out every burn; the executor (not written yet) will fly them
under rails warp with Persistent Thrust. The root `CLAUDE.md` applies here.

**State (2026-10-12): the solver works end to end in a stand-in solar
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
  retrograde centred on periapsis (holding the periapsis above the safe
  floor), then FINAL.
- **Planet**: porkchop window after the pump's estimated end; a parking
  ellipse whose periapsis push leaves on the departure asymptote (plane
  chosen so its change is cheapest); push at the periapsis nearest the
  window (multi-pass if the engine is too weak to escape in one); B-plane
  corrections in the cruise, or a Lambert re-target (aimed at the B-plane
  point) when the miss is too big to linearise; capture designed from the
  window's arrival speed -- high and loose if the engine is weak, the cruise
  cutting v_inf by up to 30% if needed, refused at planning beyond that.

## Measured (stand-in, default vessel, from LKO 100 km)

| destination | dv | trip | notes |
|---|---|---|---|
| Mun 50 km | 1108-1123 m/s | 11-17 d | 6 start dates |
| Minmus 30 km | 1195-1240 m/s | 69-117 d | 6 start dates; one 1513 (a Mun slingshot survived) |
| Duna 200 km | 1810 m/s | 364-503 d | 3 dates; the latest catches the next window (2184) |
| Eve 150 km | 2952 m/s | 481-759 d | parking plane change ~185 m/s (window declination -27 deg) |
| Ike 50 km | 1869-2254 m/s | via Duna | |
| Dres 50 km | 4778 m/s | | departure push short at 0.4 m/s^2; re-targeted |
| Jool, Eeloo | 6313, 4953 m/s | | re-targeted after a short push |
| Moho | refused | | needs TWR 0.177 to capture in one pass; flies at 4 m/s^2 (5698 m/s) |
| weak engine (TWR 0.004) | Mun 1477 m/s | 97 d | Minmus and Duna **fail** (see Open) |

## Open, in order

1. **Weak engines crossing a moon's orbit** (TWR 0.004 to Minmus past the
   Mun, Duna capture spiralling through Ike's orbit): dozens of crossings;
   the pass caps and trims cannot always find a clear revolution, and the
   recovery after a slingshot is too slow for the engine. Either time the
   pump's crossings (choose the waiting orbit's period to be non-resonant
   with the moon) or cross the band in fewer, larger passes.
2. **Short departure pushes** at 0.4 m/s^2 for high v_inf (Dres, Jool,
   Eeloo, Moho): the 60 deg arc cuts the push and the cruise re-targets
   expensively (Dres 4778 m/s). A longer push, or a parking ellipse beyond
   the moons (guarded) to shrink the periapsis push.
3. **The Persistent Thrust probe** (`tools/ptprobe.py`, an ion craft and an
   LKO save): throttle persistence into rails warp, which SAS modes hold on
   rails (prograde/retrograde/normal and the maneuver mode -- combined burns
   and corrections need arbitrary directions), dv per warp rate, EC in
   shadow, kRPC orbit reads during warp.
4. **The pilot** (`pilot.py`, the only kRPC-aware code): run `Mission`'s
   phase machine against kRPC (live bodies and orbits, measured
   acceleration), panel with target fields, PLAN (the stand-in flight as
   the preview), START/TERMINATE, `config:` line, fingerprint, `rpccount`,
   a `PILOTS` line. Warp from the tick each phase needs (lower near
   periapsis: ticks are uniform in angle).
5. Routes not yet supported: from a moon (Mun -> Kerbin, Mun -> Minmus,
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
