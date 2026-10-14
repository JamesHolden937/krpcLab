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
./lowThrust/tools/simmission.py ... --thrust 200 --mass 5000 --isp 4200 --ut 3000000
./lowThrust/tools/simfly.py --start "ap=100 pe=100" --target "ap=500 pe=500 i=5" --within 2d   # one SOI, with a deadline
lowThrust/tools/sweep.sh /tmp/sw "100 200 500 2000" "0 3000000 6000000 9000000"   # the grid: every destination x thrust x date
.venv-pypy/bin/python -m unittest lowThrust.tests.testOrbit lowThrust.tests.testGuidance lowThrust.tests.testMission
```

Orbits: `body: ap= pe= [i= lan= argpe=]`, km and degrees; anything left out
of the target is free. `simmission` prints the phase timeline (Kerbin dates),
every burn (phase, body, start, duration, dv, prograde/normal/radial share),
dv by phase and the totals. Default vessel: 5 t, one 2 kN Dawn, Isp 4200
(0.4 m/s^2, TWR 0.04); `--mass`, `--thrust`. The user's craft: TWR ~0.004
(0.04 m/s^2) -- the case that has to work.

## Files

| file | what |
|---|---|
| `orbit.py` | two-body maths: elements, any conic, universal-variable propagation, time to periapsis, Lambert, radius toward a direction, impulsive references |
| `guidance.py` | the within-SOI law: Hohmann in many small burns + plane change at the nodes (+ a held apse line) |
| `plan.py` | within-SOI deadline planner: the narrowest arcs that meet a time budget |
| `bodies.py` | the stand-in system: stock bodies on Keplerian rails, SOIs |
| `transfer.py` | targeting: closest approach / B-plane miss, mid-course correction, moon push, porkchop window, parking orientation, capture design and the capture law, Lambert re-target, SOI hazards |
| `mission.py` | the phase machine for a whole transfer (patched conics), routing and leg chaining |
| `sim.py` | the single-body stand-in (`fly`) and the shared RK4 / tick |
| `tools/simfly.py`, `tools/simmission.py`, `tools/sweep.sh` | command lines; `sweep.sh` flies the grid in parallel |

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
  any SOI exit during a departure phase hands over to CRUISE. B-plane
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

## Measured (stand-in, from LKO 100 km, 5 t vessel)

Mean dv (m/s) over 6 start dates (UT 0 - 9e6 s), and arrivals where not 6/6
(sweep of 2026-10-13, 432 flights):

| thrust (accel) | Mun | Minmus | Duna | Eve | Ike | Dres | Jool | Eeloo | Moho |
|---|---|---|---|---|---|---|---|---|---|
| 100 N (0.02) | 1700 | 1540 | 3698 (4/6) | 4157 | **0/6** | 6321 | **0/6** | 7177 | 9063 |
| 200 N (0.04, TWR 0.004) | 1328 | 1450 | 2902 | 3951 | 3434 | 5482 | 7151 | 6209 | 7397 |
| 500 N (0.1) | 1222 | 1330 | 2798 | 3807 | 2788 | 5035 | 6483 | 5690 | 7090 |
| 1 kN (0.2) | 1178 | 1248 | 2332 | 3319 | 2377 | 4939 | 6183 | 5293 | 6791 |
| 2 kN (0.4, default) | 1167 | 1250 | 1805 | 2944 | 1862 | 4720 | 5944 | 4981 | 6527 |
| 10 kN (2) | 1155 | 1239 | 1905 | 2904 | 1933 | 4153 | 5510 | 4228 | 5854 |
| 25 kN (5) | 1133 | 1255 | 1882 | 2900 | 1898 | 3945 | 5479 | 4141 | 5881 |
| 50 kN (10) | 1154 | 1449 | 1894 | 2914 | 2191 | 3923 | 5463 | 4115 | 5652 |

### Second grid (2026-10-14): 15 destinations x 0.04 / 0.4 m/s^2 x 6 dates

180 flights, 170 arrive. Mean dv (m/s), 0.04 / 0.4: Mun 1412 / 1167,
Minmus 1494 / 1248, Duna 2812 / 1801, Ike 3652 / 1859, Eve 3880 / 3047,
Gilly 4033 / 2782, Dres 5531 / 4705, Eeloo 6303 / 4972, Moho 7453 / 6522,
Jool 8146 / 6001, Pol 5923 / 5228, Bop 5838 / 6753, Tylo 7455 / 7996,
Laythe 9547 (**2/6**) / 6779, Vall **0/6** / 6333. (Gilly and Jool's moons
were added to the stand-in this session.)

## Open, in order

1. **The Persistent Thrust probe** (`tools/ptprobe.py`, an ion craft and an
   LKO save): throttle persistence into rails warp, which SAS modes hold on
   rails (prograde/retrograde/normal and the maneuver mode -- combined burns
   and corrections need arbitrary directions), dv per warp rate, EC in
   shadow, kRPC orbit reads during warp.
2. **The pilot** (`pilot.py`, the only kRPC-aware code): run `Mission`'s
   phase machine against kRPC (live bodies and orbits, measured
   acceleration), panel with target fields, PLAN (the stand-in flight as
   the preview), START/TERMINATE, `config:` line, fingerprint, `rpccount`,
   a `PILOTS` line. Warp from the tick each phase needs (lower near
   periapsis: ticks are uniform in angle).
3. **Aerobraking** (below).
4. Routes not yet supported: from a moon (Mun -> Kerbin, Mun -> Minmus,
   moon -> planet).
5. **Cheaper moon dodges**: APPROACH dodges a moon on the inbound leg late
   and dear (a 10 m/s^2 Duna arrival through Ike: 238 m/s). Choosing the
   aim point round the B-plane circle in the cruise, where it is cheap,
   would avoid it.
6. **Weak-engine departures**: a 0.04 m/s^2 escape leaves with ~760 m/s of
   a 2.9 km/s window v_inf, and the cruise re-target builds the rest
   (Jool: CRUISE 2.3-2.6 km/s of the total). Limiting DEPART to its
   +-60 deg arc was flown and lost (v_inf 765 even at 0.4 m/s^2, a Mun
   hit); spiralling out and leaving on the asymptote might not.

## Aerobraking (planned, not built -- the user, 2026-10-13: "think about it")

Where it pays: captures and orbit-lowering at bodies with an atmosphere
(Kerbin, Eve, Duna, Laythe, Jool). For a weak engine it is the biggest
saving there is: Duna's capture plus FINAL is ~1 km/s at 0.04 m/s^2, Jool's
~2.5 km/s, and both are slow (long burns, many passes through moon bands).
What it needs:

1. **An atmosphere in the stand-in**: density by altitude per body. KSP
   defines pressure and temperature curves per body; density =
   p M / (R T). Take the curves from the game (kRPC
   `body.atmospheric_density_at_position` sampled on a save, the way
   `kspSim/data/` holds terrain) rather than transcribing constants.
2. **The vessel's drag**: a ballistic coefficient (m / (Cd A)) measured in
   the game (kRPC `flight.drag` at known q), and the heating limit (KSP
   skin temperature; the spaceplane's `trajectory.peak_heating` uses
   sqrt(rho) v^3). Prograde-pointing drag only -- no lift.
3. **Propagation through it**: the two-body coast is no longer exact below
   the atmosphere top; integrate the atmospheric arc with drag (RK4, like
   the thrust arcs), exact conics everywhere else.
4. **The law**: aerocapture = aim the arrival periapsis at the altitude
   whose one pass sheds exactly enough to be bound (a 1-D search on the
   predicted pass, re-solved every tick of the approach, the B-plane
   correction steering the periapsis); aerobraking = after any capture, a
   periapsis in the upper atmosphere lowering the apoapsis pass by pass,
   lifted out at the end by a burn at apoapsis. Heating caps how deep.
   Both fit the phase machine as an alternative CAPTURE and a FINAL
   variant, chosen at planning when the body has an atmosphere.
5. **The pilot**: KSP will not rails-warp inside an atmosphere, so each
   pass is physics-warped; Persistent Thrust is irrelevant there (no thrust
   during a pass).

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
