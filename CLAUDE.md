# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **⚠ The farm runs a MODIFIED Kerbol system (MiniSystem, since 2026-11-06), not stock.**
> - Changed: planets' orbits x0.2, Kerbol GM and radius x0.2, moons' orbits x0.5 (Mun at 6 Mm, Minmus at 23.5 Mm).
> - Kerbin's mass, radius, atmosphere and 6 h solar day are stock. Its **sidereal rotation is 21349.47 s**
>   (stock 21549.43) and the year is ~85 days.
> - Kopernicus now actually loads on the farm. Before, it silently didn't, because `KSPTextureLoader` was
>   stripped. It brings its own patches, e.g. solar panels.
> - Farm results from before 2026-11-06 are not comparable with later ones.
> - kspSim models probed before then are stale: re-probe (`kspSim/world.py` warns).
> - The live game (`~/KSP-Retooled`, KSP Retooled 1.13 on Unity 6.7 since 2026-11-13) runs MiniSystem too; KSP-VSE's `vse/` instance is stock.
> - Install: `testInstances/miniSystem.sh`; back to stock: `testInstances/miniSystem.sh --stock`. Design: `~/minisystem/CLAUDE.md`.


## Session procedure (standing, every session)

**Work autonomously.** Don't ask whether to go on to the next batch, fix or
experiment. **Never ask before starting the farm or flying a batch on it**
-- not even when an earlier session or HANDOFF.md says the user wanted it
left down (the user, 2026-10-11: "don't ever ask me again ... I'll tell you
to stop when I need my CPU"). If a test needs the farm, start it and fly;
the user interrupts when they need the CPU, and that stop lasts only until
the next time a test needs it. **Only ask permission for things that affect disk space outside
this directory**; anything confined to krpcLab (craft/save variants, farm
instances, venv installs, commits) needs no asking. Pick the best next step, do it, and report what you did and why.
Interrupting is the user's job, not asking permission. **Commit before
anything risky**: a default change, a batch on new code, a refactor or a
scripted edit, so it can always be undone.

**Never sit waiting for a batch (or a save, a probe, a single flight) to
finish before doing the next piece of work** (the user, 2026-10-25): build
what you will do next anyway in a git worktree (`git worktree add
.wt-<name>`; the batch flies the main tree, which stays frozen), and use idle
instances for the next question. Don't poll or read mid-flight logs:
part-batch data gets misread as a result. Every time you stop to wait, tell
the user roughly when it will finish, in 24-hour time ("done ~1845"), from the
previous batch's measured length.

**Procedures tied to an event are not written here: the event delivers them.**
Each arrives in the output you read at that moment, so follow it when it
appears:
- *Session start* (`.claude/settings.json` -> `tools/claudeHooks.py session`):
  HANDOFF.md injected, and the farm's state (up since, swap, restart advice).
- *Batch launched from a session* (`~/.local/bin/farmlaunch`, the user's): exit
  76 with the detach / handoff / relay / clear steps (`~/.claude/docs/autoContinue.md`).
- *Batch start and end* (farmlaunch runs `.farmlaunch/pre|post` ->
  `testInstances/batchHooks.py`): saves in sync, same GameData, nothing stray,
  farm restarted, code committed; then swap after, leftovers, what to do now
  (read, handoff, commit, clear) and the periodic whole-vehicle audit.
  When woken by an AUTO-CONTINUE message, `detachjob status NAME --tail 150` shows it.
- *Suite run beside the farm, or a code edit in the main tree mid-batch*
  (`claudeHooks.py bash|edit`): refused, with what to do instead.
- *Post-commit bundle/suite failure* (`claudeHooks.py notice`): reported once.
- *A farm batch launched with the context over 100k tokens* (`claudeHooks.py notice`, on the
  launching Bash call only): a hard order to write the handoff, commit and clear self.
- *New save or craft committed* (post-commit) and *push with a pilot's code
  newer than its howItWorks.pdf* (`tools/githooks/pre-push`): reminders with
  the procedure.
Change a procedure in the script that prints it, not here.

0. **The sleep inhibitor is automatic** (global Claude hooks): taken at each
   prompt, released at the end of a turn unless a farm instance is still
   running, then released once the farm is idle. Don't run nosleep by hand.
1. **[HANDOFF.md](HANDOFF.md)** (the index: every pilot's status in one line,
   cross-pilot open items, general traps) arrives at session start; read
   `docs/<pilot>/handoff.md` for the pilot you will work on -- the last
   session's state: what changed, which flags exist and what they measured,
   the blocker, what's next, and the traps. Then the pilot's own `CLAUDE.md`.
   Read another pilot's handoff only when the work touches it.
   `testInstances/HANDOFF.md` is the farm's build history, not this.
1a. **Screen in kspSim whenever the question is one it models** (~10x
   faster, many at once): guidance, propagator, phase logic, anything a
   learned table or law decides. Fly the farm to confirm a sim result, and
   go straight to it only where the sim is known not to follow the game
   (`kspSim/CLAUDE.md`, "Known gaps" -- e.g. the shuttle's high-alpha
   lateral moments). Say which it was and why. Never sims beside the farm.
2. **End by rewriting the handoff.** Also do it before any long pause or when
   the user asks for a summary. Each pilot's state is in
   `docs/<pilot>/handoff.md`, the most recent section first: rewrite the
   section of the pilot(s) this session touched in place (move it to the
   top) and leave the other pilots' files as they are -- they are the last
   word on pilots nobody has flown since. Then update that pilot's row in
   HANDOFF.md's table (date, one-line state/next) and HANDOFF.md's own
   sections only for cross-pilot items or general traps.
   Record the date, the defaults fingerprint, commit state, where each
   airframe stands, every flag added and what it measured (with log/batch
   names), the blocker and the proposed next step, open items in order, and
   traps paid for. Each section is a snapshot: replace it, don't append to
   it. The history goes in `docs/<pilot>/journal.md`.
3. **On wrap-up: commit and push to GitHub** (`git push origin main`), so the
   next session -- on any machine -- starts from it. `docs/<pilot>/howItWorks.pdf`
   is the only tracked copy of each pilot's how-it-works page; the push says
   when one is older than its code, and how to remake it.
   **Nothing that the repo plus a KSP instance can regenerate is tracked**
   (e.g. `kspSim/data/ksc_terrain.json`, `kspSim/models/`): gitignore it and
   say in the owning CLAUDE.md how to rebuild it.

## What this is

**krpcLab**: kRPC autopilots for Kerbal Space Program, and the infrastructure
to measure them -- a farm of headless game instances, reference saves, log
readers. New autopilots go here and plug into all of it.

| directory | what |
|---|---|
| `boosterland/` | flies a booster back to the KSC pad after stage separation. Largely settled. Has its own `CLAUDE.md` |
| `ascent/` | MechJeb classic ascent (over KRPC.MechJeb), turn shape solved from the craft at liftoff; drops the booster when it has just enough propellant to fly home and follows the upper stage to orbit (the booster is flown from FMRS's save by `boosterland/fmrs.py`). Read `ascent/CLAUDE.md` |
| `spaceplane/` | deorbits a winged vehicle, flies the entry, lands on the KSC runway. The active frontier. **Read `spaceplane/CLAUDE.md` before touching it** |
| `propPitch/` | a propeller plane's collective blade pitch held at the setting that goes fastest (extremum seeking on forward aero force); flies only the blades. Read `propPitch/CLAUDE.md` |
| `airplane/` | an attitude autopilot for winged craft: CRUISE (sideslip nulled, bank held to 89 deg, pitch attitude held through coordinated turns) and HOLD (pitch, heading, bank held), the player's stick on top; optional constant body rates (RPM) and propPitch beside it. Read `airplane/CLAUDE.md` |
| `launcherMod/` | **krpcLab's only UI** (C#, kRPC service `KrpcLabLauncher`): one stock KSP window and toolbar button, in flight only, drawing the launcher's menu and every pilot's panel (`common.panel`); counts scene changes and vessel switches (the launcher stops a pilot on one); reports the control surfaces' actuator speed/range/torque (`SurfaceActuators`, which kRPC lacks); START SCRIPT starts the kRPC server and `./run.sh`. `launcherMod/build.sh` builds it and installs it into the farm and the live game. Without it a pilot's panel is headless (kspSim) |
| `kspSim/` | a headless KSP behind a real kRPC server: the autopilots fly it unmodified (`--instance sim0`), ~10x faster per flight and many at once. Screening only -- confirm on the farm. **Read `kspSim/CLAUDE.md` first** |
| `common/` | shared by every autopilot: `vec`, `logbook`, `pacing` (loop pacing and the time-scale governor), `rcs` (the RCS valve), `timescale` (the farm plugin's file), `paths`, `launcher`, `cli` (the standard flags and connect), `config` (`--set`, the defaults fingerprint), `panel` (`ControlPanel`), `krpcpilot` (engaging and aiming kRPC's autopilot), `geo` (gravity, great-circle distance), `idle` (gimbals and surfaces held when useless). **Anything two pilots both need goes here, not into a second copy** |
| `tools/` | tools for any autopilot: `bundle.py`, `savegen.py`, `timescale.py`, `instancebench.py` |
| `<pilot>/tests/`, `<pilot>/tools/` | each autopilot's offline tests and its own harness and log readers |
| `saves/` | **the reference quicksaves and craft files** -- tracked; the farm copies drift. `saves/README.md` says which save is which vehicle |
| `testInstances/` | the measurement farm's scripts. The 15 GB of game copies are in the shared farm `~/kspfarm` (`base/`, `krpclab/`; see its README.md), with git-ignored symlinks here |
| `docs/` | `boosterland/`, `spaceplane/`, `propPitch/`, `airplane/` (design, failures, session journal), `testInstances.md`, `krpc.md` |
| `logs/` | every flight's `LOG<n>`, numbered across all autopilots; git-ignored, cited by number in the docs |

Every autopilot has the same shape: one phase machine is the only kRPC-aware
control code, every tick re-propagates the state to the ground with a local
RK4 propagator, and every phase decision comes from that fresh prediction
rather than a stored plan.

## Adding an autopilot

1. A package `<name>/` with a module that accepts `--address`,
   `--rpc-port`, `--stream-port`, `--set FIELD=VALUE` and `--autostart`, and
   shows an in-game panel with START and TERMINATE (`common.panel.ControlPanel`,
   drawn by `launcherMod`). If it loads a save or switches the active vessel
   itself, it calls `panel.expect_scene_change` first, or the launcher stops it.
2. Its tunables in `<name>/config.py`, a `Logbook` from `common.logbook`, the
   `config:` line and a defaults fingerprint in every log (hard conventions
   below).
3. One line in `PILOTS` in `common/launcher.py`: the launcher gets a button
   and `./run.sh --pilot <name>` works.
4. `<name>/tests/` (a package with `__init__.py`, files named `test*.py`):
   `python3 -m unittest` finds them.
5. A harness in `<name>/tools/` that loads a save on a farm instance and flies
   it (`spaceplane/tools/quickglide.py` is the model: subprocess, reaper,
   `--instance`, `--timescale`), and the saves it flies in `saves/`.
6. `<name>/CLAUDE.md` for its current state, `docs/<name>/` for the rest.
7. Write its loop and its scripts by [docs/loopCost.md](docs/loopCost.md):
   a tick's wall cost is the farm's speed. Batch reads, cache constants,
   wait in game time, nothing under rails warp, keep one-off work out of
   fine-interval phases, and install `common.rpccount` so every log says
   what each phase cost.

## Commands

```bash
./run.sh                               # in-game launcher: a button per autopilot
./run.sh --pilot spaceplane            # one autopilot; its panel waits for START
./run.sh --pilot booster --autostart --set LOG_INTERVAL_UT=1
./run.sh --address 192.168.1.5         # kRPC server on another machine

.venv-pypy/bin/python -m unittest      # every offline test, no KSP needed: 19 s (CPython: 131)
.venv-pypy/bin/python -m unittest boosterland.tests.testFlightSim   # one module

./tools/bundle.py                      # every autopilot in one file, comments stripped -> dist/krpcLab.py
./tools/savegen.py -o hot --prograde 60   # a different entry state, written to a save
./tools/timescale.py 0 max             # an instance as fast as it will sustain
./tools/instancebench.py 0 --quant 0.05,0.1   # how fast, and which ceiling binds

# **Starting, stopping and restarting the farm never needs permission.**
# It is the measuring instrument, it is always OK to bring up or take down,
# and a session that flies nothing measures nothing.  Standing procedure:
# `cd testInstances && ./start.sh` (six, 0-5; it returns
# when every port is up, relaunching boot crashes), fly; `./stop.sh` when the
# suite has to run or the session ends (exit 1 = something survived it).
# Every batch script runs under farmlaunch (testInstances/farmGuard.sh): exit 75
# = another batch holds the farm (launch after its notice, never a wait loop on
# its name); 124 = no LOG<n> within 900 s.  Never wait for the bundle or the
# post-commit hook: bundle.ensure() waits on its lock.
#
# **Six instances (0-5), and never beside the test suite.**  Six put 8-13 GB
# into zram at ~4 GB each; a seventh saturated the CPU at boot (2026-10-02).
# Harness defaults: PyPy (PYPY=0 / --cpython for CPython), time-scale
# ceiling 20, six instances.
cd testInstances && ./mkbase.sh        # build a stripped KSP copy into ~/kspfarm/base (once, ~7.5 GB)
./mkclone.sh 0 && ./kwinRun.sh 0       # an unattended instance, invisible
./syncSaves.sh check                   # do the instances fly the reference saves?
./syncSaves.sh push                    # ... make them
./watchdog.sh start                    # restart any instance that dies
./stop.sh                              # stop them all (and the watchdog)
```

**Every autopilot flies from `dist/krpcLab.py`**, the single-file bundle (`tools/bundle.py`): `run.sh`, the launcher's buttons and the farm harnesses rebuild it when a source is newer and run it, and the `post-commit` hook in `tools/githooks` rebuilds it on every commit (`run.sh` sets `core.hooksPath`), then, when the farm is down, runs the offline suite against it in the background; `.git-bundle-error.log` appears in the root if either fails, and the session is told. Minified plain text, never compressed; a name any test or shell script could reach stays. `tools/bundle.py --suite` runs the offline tests against it. Every farm harness starts its flight through `tools/farmflight.py`; `--tree` flies the modules instead. How it is minified: [docs/bundle.md](docs/bundle.md).

Tools are run from the root. Each puts the root on `sys.path` itself and
re-executes under the project venv if the interpreter has no `krpc`, so
`./spaceplane/tools/quickglide.py` works from any shell. **Everything runs
under PyPy** when `.venv-pypy` exists (`kspSim/tools/pypysetup.sh`): `run.sh`,
the launcher's autopilots, every tool through `common.paths.use_venv`, and
the farm harnesses. `KRPCLAB_CPYTHON=1` (or `--cpython` / `PYPY=0` on the
harnesses) falls back to `.venv`, which `run.sh` creates on first use (`krpc`
built from kRPC main by `tools/krpcmain.sh`, the server mod's version) and runs by path. The flight runs in the foreground, so Ctrl-C reaches it and it hands
the vessel back. `--set FIELD=VALUE` overrides a field of `Config` with type
coercion; unknown fields raise. No linter or formatter is configured. Tests
need no third-party packages.


## The farm

**Where the game copies live:** in the shared farm `~/kspfarm`. Read `~/kspfarm/README.md` first.
- `base/` is `~/kspfarm/base`, and the instances `ksp<n>` and `prefix<n>` are in `~/kspfarm/krpclab/`.
- **KSP Retooled 1.13 (Unity 6.7) since 2026-11-13**, like the live game; earlier farm results are not
  comparable. Plugins compile against `~/kspfarm/stockref` (stock 1.12.5), never `base/`. The game process
  is named `Unity Main Thre`: use `procs.sh` (`game_pid`, `game_count`), not `pgrep -x KSP_x64.exe`.
- `testInstances/{base,ksp<n>,prefix<n>}` are symlinks to them, so every path in this repo still
  works. `mkbase.sh` and `mkclone.sh` create the real directories there plus the links.
  `KSPFARM=<dir>` moves the farm.
- Other projects' instances may share the farm (`~/kspfarm/<project>/`). Never run them while
  these are up.

**What makes a farm batch a lie without announcing it** -- a killed batch's
flight still flying, a clone's quicksave drifted from `saves/`, swap grown
over hours (seven hours biased the spaceplane's arrivals 27 km long, spread
unchanged), an instance with a different mod set (`BetterTimeWarp` once moved
a deorbit pass 23 km) -- is checked by `testInstances/batchHooks.py` at every
batch's start and end. Re-clone rather than patch. See
[docs/testInstances.md](docs/testInstances.md).

## Reading code and logs (token cost)

- **Never Read a source file over ~500 lines whole** (`boosterland/autoland.py` is 3.3k, ~40k tokens):
  `grep -n "def \|^class " FILE` (or grep the name you need), then Read with `offset`/`limit` around it.
- Read logs through the pilot's summary tool (`boosterland/tools/logsum.py`, the spaceplane's `*sum.py`),
  and only then the lines of one log around the event it points to.

## Hard conventions

- **Nothing prints.** stdout/stderr are reserved for crashes before the logbook
  opens (`run.sh` funnels them to `logs/stderr.log`). All output goes to
  `logs/LOG<n>` — a new numbered file per run — and to the in-game panel.
- **Log pacing is in-game time.** `Logbook.telemetry` gates on `SpaceCenter.ut`,
  so a paused game logs nothing and a time warp still produces one line per
  `LOG_INTERVAL_UT` in-game seconds (it re-anchors instead of catching up).
  Phase changes go through `Logbook.event` and are never suppressed. Keep
  telemetry to one dense line per tick — the logs are meant to be read whole.
- **Every tunable lives in `config.py`.** Control code reads `cfg.X`; it never
  hard-codes a number.
- **Every log says which configuration flew it — *and* which defaults it was
  measured against.** Each autopilot writes a `config:` line listing the
  fields that differ from the defaults, plus `defaults_fingerprint`, a hash of
  the whole default set. Twenty flights of one session were unreadable for
  want of the first half; eighteen were misfiled into the wrong arm for want
  of the second, because a list of *differences* says nothing when the
  baseline itself moves. Spaceplane failure 28.
- **Vectors are plain 3-tuples in the body's rotating reference frame**
  (`body.reference_frame`) everywhere except the GUI. That frame rotates with
  the atmosphere, so velocities in it are already air-relative.

## Cross-cutting rules

One line each; the reasoning and the case behind every one is in
[docs/principles.md](docs/principles.md) -- read it before designing a change to a law, a
constant, a measurement or a comparison.

- **Never try ANYTHING that involves a ship-fitted constant** (the user, 2026-11-15): no flag, batch, arm or
  intermediate step resting on a number fitted to one craft or save; replace it with a measured or physical
  quantity first, or the work waits. Stop a running batch the moment it is found to rest on one.
- **Fine-tuned methods are not permitted** (the user, 2026-11-15): no mode switches at fitted altitudes,
  distances, speeds or times, no gains or margins sized on one craft; every threshold and gain comes from a
  quantity measured on the craft in flight or from physics, a mode change falls out of a continuous law.
  Dimensionless policies are allowed.
- **A flag ends its life as the default or deleted -- never parked off.**
- **Measure in game, not in the sim.**
- **A knob that changes nothing may be disconnected, not powerless.**
- **One flight per configuration is not a measurement, and neither is three.**
- **A correlation across flights the autopilot chose is not a sensitivity.**
- **Read the signed miss, not the distance.**
- **Measure over several entry states, with `AIM_BIAS_*` at 0.**
- **A prediction error found is the top priority, and it is fixed at its source -- never hidden** (the user,
  2026-11-17): stop and find which term, delay or tick; never absorb it with a bias, margin, gain or late loop.
- **The propagator must fly the law the vehicle flies.**
- **A missing answer must not be allowed to look like a good one**
- **A quantity measured from flight data describes the vehicle *as it was flown*, and every change to how it is flown silently invalidates it.**
- **A guard whose stated reason does not survive inspection may still be the only thing enforcing a constraint nobody wrote down.**
- **A constant that has to be re-fitted after unrelated changes is a missing model wearing a constant's clothes.**
- **A phase must hand over a state the next phase can fly, and a control law that steps is a law no vehicle flies.**
- **A clamp can make two configurations fly identically, and the log will faithfully report the one that was ignored.**
- **A control loop that cannot serve its own commanded interval is flying a different vehicle, and the log has to say so.**
- **Ask what the vehicle can do that the autopilot never commands.**
- **A measurement that shares a constant's name is usually a different quantity, and the agreement is what makes it dangerous.**
- **Re-run the tool; do not read the prose.**
- **When three attempts on one mechanism come back null, the next thing to change is the method, not the fourth value.**
- **A measurement whose only consumer is a hand-copied constant cannot be contradicted.**

## Where the detail is

Read the file that covers what you are about to touch; do not load them all.

| doc | read it when |
|---|---|
| [spaceplane/CLAUDE.md](spaceplane/CLAUDE.md) | anything in `spaceplane/`: where it stands, what is next, what is refuted |
| [boosterland/CLAUDE.md](boosterland/CLAUDE.md) | anything in `boosterland/` |
| [ascent/CLAUDE.md](ascent/CLAUDE.md) | anything in `ascent/`, or MechJeb on the farm |
| [docs/testInstances.md](docs/testInstances.md) | running the parallel-instance farm, or when instances die |
| [docs/farmSetup.md](docs/farmSetup.md) | building the farm from scratch on a new machine (the game copies are not in git) |
| [docs/loopCost.md](docs/loopCost.md) | writing or changing a control loop, a kRPC call in one, or a farm/harness script |
| [docs/krpc.md](docs/krpc.md) | any kRPC call whose signature you are not certain of |
| [propPitch/CLAUDE.md](propPitch/CLAUDE.md) | anything in `propPitch/` |
| [airplane/CLAUDE.md](airplane/CLAUDE.md) | anything in `airplane/` |
| [kspSim/CLAUDE.md](kspSim/CLAUDE.md) | flying anything in the simulator, or making a model of a new save |
| [saves/README.md](saves/README.md) | which save is which vehicle and state; adding a save |

**Keep this file short.** It is loaded into every session whole; the docs are
not. New material goes in the doc it belongs to -- an autopilot's state in its
own `CLAUDE.md`, a session's measurements in its `docs/<name>/journal.md` --
and only a rule that applies to *every* change belongs here.
