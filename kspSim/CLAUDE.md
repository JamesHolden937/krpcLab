# kspSim — a headless KSP behind a real kRPC server

A Python stand-in for the game that speaks the kRPC wire protocol, so the
**unmodified** autopilots and harnesses fly it:
`quickglide.py --instance sim0 [--pypy]`, `quickfly.py --instance sim0
--rpc-port 50200 --stream-port 50201`. Physics is Kerbin, one vessel, KSP's
0.02 s step. Everything vehicle-specific is **read out of the real game**
(probe, part configs, the save) and every law is **checked against the game
frame by frame** (flight tests). Journal: `docs/kspSim/journal.md`.

**Rule (root CLAUDE.md "measure in game"):** the sim is for screening and
volume. Nothing is promoted on sim evidence alone; confirm on the farm.

## Layout

| file | what |
|---|---|
| `server.py` | kRPC server: RPC + stream ports, streams, `Sim.AdvanceTo` lock-step, real-time flow at the governed time scale (1x after a Load until the governor writes), per-procedure time table in `logs/kspsimN.log` |
| `fastpb.py`, `fastclient.py` | kRPC messages by hand (server side); the krpc client patched for PyPy (loaded by `.venv-pypy`'s .pth) |
| `protocol.py`, `wire.py` | kRPC codec; signatures from `data/services.bin` plus a `Sim` service |
| `api.py` | ~250 procedure handlers, reference frames, UI stubs; action groups and brakes drive the vessel |
| `world.py` | Body, Vessel (mass/inertia, aero, surfaces, airbrakes, wheels, RCS, engines + gimbal, drain valve, electric charge, ground, thermal hook), rails warp, clock |
| `aero.py` | tables: base split into rest + cube drag x pseudo-Reynolds; per-surface solo tables; airbrake angle tables; damping; gear |
| `thermal.py` | port of KSP's FlightIntegrator thermal model: drag-cube areas with stack occlusion, shock-cone occlusion, exposed/unexposed skins, conduction, convection, radiation (Sun, planet), burn-up |
| `partcfg.py` | ModuleManager ConfigCache (patched part configs), variants (`.mu` hierarchy), drag cubes, save parsing |
| `attitude.py` | port of kRPC 0.6.0 `AttitudeController` |
| `atmosphere.py`, `curves.py` | Kerbin atmosphere; KSP FloatCurve |
| `run.py`, `restart.sh` | start instance N (ports 50200+2N, dir `testInstances/kspsimN/`); `restart.sh` uses PyPy when `.venv-pypy` exists |
| `tests/testPhysics.py` | offline tests of each law against the number the game gave |
| **tools** | |
| `probe.py` | **makes `models/<save>.json`** (see below) |
| `augment.py` | adds thruster geometry, live nozzles, part configs, drag cubes, action bindings to a model (`--offline`: no game) |
| `flighttest.py` | scripted inputs on a save, **every frame** recorded (`--torques`, `--thermal`, `--oracle-every`); `--attach` records an autopilot flight passively |
| `fidelity.py`, `fidsum.py`, `regimes.py` | replay a recording through the sim: one-step wrench and multi-step restarts; one line per recording; a long flight binned by Mach |
| `oracle.py` | the game's `simulate_aerodynamic_wrench_at` at a recording's states (table vs physics) |
| `calibrate.py` | per-craft aero residual from flight tests, kept only where held-out recordings improve (fits against the bare tables) |
| `thermcheck.py` | the thermal model replayed along an `observe.py` game flight, per-part skin/interior errors; `--fluxes` compares every heat flow and thermal mass term by term (needs `observe.py --fluxes`) |
| `airsave.py` | make an in-air save by deorbiting an orbital one |
| `makecraft.sh` | the whole new-craft procedure (below) |
| `observe.py`, `krpcproxy.py`, `simfly.sh` | passive temperature observer (`--fluxes`: heat flows and thermal masses too); recording proxy; N sims at once (`PILOT=booster` flies quickfly) |

## Regenerated, not tracked

`kspSim/data/ksc_terrain.json` (the KSC ground grid): `./kspSim/tools/kscterrain.py
--instance 0` with a farm instance up. `models/<save>.json`: `probe.py` (below).

## A new craft

    ./kspSim/tools/makecraft.sh 0 <main save> <air save> [<orbit save>]

probe (tables on the main save, control/gear/airbrake tables on the air
save, per-surface tables at sideslip), the flight-test battery, and
`fidsum.py`. Then: one autopilot flight
recorded with `flighttest.py --attach` and `regimes.py` on it. A bias in one
test is a defect to find (`fidelity.py --segments`, `oracle.py`);
`calibrate.py` is the last resort; thermal needs no fit (`thermcheck.py`
scores it). Another save of the same
craft: `probe.py --save X --aero-from <main> --no-terrain` (refuses a
different craft).

## What is measured (2026-09-24; numbers in the journal)

- Vacuum: wheels to 0.1%, RCS per nozzle to <1% (shuttle), gimbal force
  and torque to <1%, integration 3 mm in 5 s.
- Old plane (mid-wing): scripted tests ~1 deg / ~1 m after 5 s (roll 15
  deg); a whole game entry, frame by frame: pitch within 0.1-0.6 kN m and
  roll within 0.03 at every Mach, yaw offset 0.5-0.9. Closed loop qs_entry:
  sim -20.5 km (sd 1.0, n=8) vs game -18.1 (sd 2.0, n=4); glide within
  100-600 m / 25 m/s.
- Shuttle: vacuum exact; air 5-10% of large moments (unstable airframe);
  roll/yaw residual (rate-dependent) fitted M0.2-0.8; supersonic yaw
  +-15-70 kN m unfitted.  Scripted subsonic tests after 5 s: 3-8 deg / 3-5
  m (were 8-100 deg / 7-20 m before the surface levels, 2026-09-24).
  Closed loop qs_shuttle_final: every phase change within 0.2 s, 5 m, 3 m
  of gate and 0.1 m/s of the game's; cross-track at the flare 290 m off.
- Booster: forces, gimbal, airbrakes (grid fins), action groups; closed loop
  median ~100 m (n=20) vs game 80 (n=15), tails alike.
- kRPC-reported torques (what kRPC's attitude controller tunes on): surfaces
  1.5-2% at low alpha, 4-6 kN m at high; wheels ignore charge, as kRPC does.
- Thermal (the FlightIntegrator port, nothing fitted): skin rms over parts
  median 68 K on a whole qs_entry flight (obsf_entry), 46 K on the shuttle's
  (obsf_shuttle); peaks within 0-90 K.  Worst: the shuttle's nose interiors
  150-215 K low, the old plane's engine plate skin 174 K.

## Known gaps, in order

1. The shuttle's base pitch table is ~30 kN m nose-down against the flying
   game at zero input near the stall (g_qs_shuttle_final_pitch); a pitch
   residual does not survive held-out validation, so it is not applied.
2. The shuttle's roll/yaw residual depends on body rates: the game departs
   from its own oracle by -48..+160 kN m per rad/s while the sim follows the
   oracle, so the damping table (the oracle at 0.1 rad/s, beta 0) is the
   suspect -- a missing law wearing a fit.  Supersonic yaw unfitted.
3. Thermal: the shuttle's nose interiors run 150-215 K low.
4. Aero tables stop at the craft as probed: a part lost (burn-up) keeps its
   aero; deployable parts other than airbrakes (cargo doors, flaps via
   Deploy) are not modelled.
5. Terrain outside the KSC grid is sea level; wheels are spring-dampers.
6. Fuel flow draws evenly from all tanks (KSP has flow priorities).

## Speed

A qs_entry flight (~900 game s): **~30 s wall** with `--pypy`, the server
under PyPy and the fast client; 8 at once in ~65 s. The autopilot's own
propagator is ~84% of its time -- that is the floor. Setup:
`./kspSim/tools/pypysetup.sh` (`.pypy/` portable PyPy, `.venv-pypy/` with
krpc 0.6.0 and `kspsim_fastclient.pth`, all git-ignored). The .pth patches
krpc's client (`fastclient.py`: same bytes, 1.6x faster calls);
`KSPSIM_FASTCLIENT=0` turns it off. Models are parsed once per server
process (two cached).

## Reading the game's code

`ilspycmd` decompiles `KSP_x64_Data/Managed/Assembly-CSharp.dll` (and mod
DLLs): `dotnet tool install --tool-path <scratch>/ilspy ilspycmd`, then
`ilspycmd -t ModuleGimbal -r <Managed dir> <dll>`. Read, cite the class in a
comment, **never copy the source into the repo**. It settled the gimbal law,
wheel charge, surface potential torque, airbrakes, convection units.

## Traps

- **kRPC hands stale thruster transforms after many scene reloads** (every
  `thrust_position` throws): restart the instance; `augment.py` borrows
  geometry from a sibling model of the same craft meanwhile.
- The recorded mass cannot place the CoM (which tank emptied): recordings
  carry `root` (CoM) and `ec`; replays use them.
- `aerodynamic_force` in frame i+1 is frame i's force.
- `pkill -f` from a shell whose command line holds the pattern kills the
  shell (exit 144): put kill loops in a script file.
- A loaded sim keeps simulating when idle: stop sims after a batch.
- Surface tables need levels near zero (`probe.py --solo-levels`): drag
  is quadratic in deflection, and a new craft probed without them flies
  with too much trim drag.
- Compare against game flights made with the governed harness
  (`--timescale 8`), never `--timescale off`: that skips the paused
  handover and the save falls for seconds first.
- `ControlSurface.deflection` override drives `deployAngle`, which an
  airbrake ignores (`aeroDeployAngle`): set the "Deploy Angle" field.
- The sim does not advance while paused: unpause, then `advance_to`.
