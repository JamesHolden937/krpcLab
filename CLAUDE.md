# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session procedure (standing, every session)

**Work autonomously.** Don't ask whether to go on to the next batch, fix or
experiment. Pick the best next step, do it, and report what you did and why.
Interrupting is the user's job, not asking permission. **Commit before
anything risky**: a default change, a batch on new code, a refactor or a
scripted edit, so it can always be undone.

**Once a batch starts, stop and wait until it ends.** Launch one background
waiter that prints the summary when the batch finishes (`ROT DONE`), end the
turn, and do nothing with the batch until that notification arrives. Don't
poll, don't chain wait calls, don't read mid-flight logs: it wastes tokens,
and part-batch data (one round, half the arms, a flight still in the glide)
gets misread as a result. **Every time you stop to wait, tell the user
roughly when it will finish, in 24-hour time** ("done ~1845"), estimated
from the previous batch's round length times the rounds left.

0. **Take the sleep inhibitor first thing**: `testInstances/nosleep.sh start`
   (idempotent; `status` checks it). **Release it last thing**, after the
   farm is stopped and HANDOFF.md is written: `testInstances/nosleep.sh
   stop`. Both, every session, whether or not anything flies -- a suspend
   mid-batch reads as a crash, and an inhibitor left held is the user's
   sleep setting overridden.
1. **Start by reading [HANDOFF.md](HANDOFF.md)**, the last session's state:
   what changed, which flags exist and what they measured, the blocker,
   what's next, and the traps. Then the pilot's own `CLAUDE.md`.
   `testInstances/HANDOFF.md` is the farm's build history, not this.
1a. **Screen in kspSim whenever the question is one it models** (~10x
   faster, many at once): guidance, propagator, phase logic, anything a
   learned table or law decides. Fly the farm to confirm a sim result, and
   go straight to it only where the sim is known not to follow the game
   (`kspSim/CLAUDE.md`, "Known gaps" -- e.g. the shuttle's high-alpha
   lateral moments). Say which it was and why. Never sims beside the farm.
2. **End by rewriting HANDOFF.md.** Also do it before any long pause or when
   the user asks for a summary. Record the date, the defaults fingerprint,
   commit state, where each airframe stands, every flag added and what it
   measured (with log/batch names), the blocker and the proposed next step,
   open items in order, and traps paid for. It's a snapshot: replace it,
   don't append. The history goes in `docs/<pilot>/journal.md`.

## What this is

**krpcLab**: kRPC autopilots for Kerbal Space Program, and the infrastructure
to measure them -- a farm of headless game instances, reference saves, log
readers. New autopilots go here and plug into all of it.

| directory | what |
|---|---|
| `boosterland/` | flies a booster back to the KSC pad after stage separation. Largely settled. Has its own `CLAUDE.md` |
| `spaceplane/` | deorbits a winged vehicle, flies the entry, lands on the KSC runway. The active frontier. **Read `spaceplane/CLAUDE.md` before touching it** |
| `kspSim/` | a headless KSP behind a real kRPC server: the autopilots fly it unmodified (`--instance sim0`), ~10x faster per flight and many at once. Screening only -- confirm on the farm. **Read `kspSim/CLAUDE.md` first** |
| `common/` | shared by every autopilot: `vec`, `logbook`, `pacing` (loop pacing and the time-scale governor), `rcs` (the RCS valve), `timescale` (the farm plugin's file), `paths`, `launcher` |
| `tools/` | tools for any autopilot: `bundle.py`, `savegen.py`, `timescale.py`, `instancebench.py` |
| `<pilot>/tests/`, `<pilot>/tools/` | each autopilot's offline tests and its own harness and log readers |
| `saves/` | **the reference quicksaves and craft files** -- tracked; the farm copies drift. `saves/README.md` says which save is which vehicle |
| `testInstances/` | the measurement farm's scripts (the 15 GB of game copies beside them are git-ignored) |
| `docs/` | `boosterland/`, `spaceplane/` (design, failures, session journal), `testInstances.md`, `krpc.md` |
| `logs/` | every flight's `LOG<n>`, numbered across all autopilots; git-ignored, cited by number in the docs |

Every autopilot has the same shape: one phase machine is the only kRPC-aware
control code, every tick re-propagates the state to the ground with a local
RK4 propagator, and every phase decision comes from that fresh prediction
rather than a stored plan.

## Adding an autopilot

1. A package `<name>/` with a module that accepts `--address`,
   `--rpc-port`, `--stream-port`, `--set FIELD=VALUE` and `--autostart`, and
   shows an in-game panel with START and TERMINATE (see either existing one).
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
   Run `testInstances/actuators.py` on the new craft before anything else.
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

./tools/bundle.py plane                # one runnable file -> dist/ (or: booster)
./tools/savegen.py -o hot --prograde 60   # a different entry state, written to a save
./tools/timescale.py 0 max             # an instance as fast as it will sustain
./tools/instancebench.py 0 --quant 0.05,0.1   # how fast, and which ceiling binds

# **Starting, stopping and restarting the farm never needs permission.**
# It is the measuring instrument, it is always OK to bring up or take down,
# and a session that flies nothing measures nothing.  Standing procedure:
# `cd testInstances && ./nosleep.sh start && ./start.sh` (six, 0-5; it returns
# when every port is up, relaunching boot crashes), fly; `./stop.sh` when the
# suite has to run or the session ends.
#
# **Six instances (0-5), and never beside the test suite.**  Six put 8-13 GB
# into zram at ~4 GB each; a seventh saturated the CPU at boot (2026-10-02).
# Restart the farm before every comparison -- swap grows round by round and
# biases the results.  Harness defaults: PyPy (PYPY=0 / --cpython for
# CPython), time-scale ceiling 20, six instances.  Check `swapon --show`
# after each batch.  The unittest suite on top of the farm
# is killed by memory pressure: stop the farm, run the suite, restart, fly.
cd testInstances && ./mkbase.sh        # build a stripped KSP copy (once, ~7.5 GB)
./mkclone.sh 0 && ./kwinRun.sh 0       # an unattended instance, invisible
./syncSaves.sh check                   # do the instances fly the reference saves?
./syncSaves.sh push                    # ... make them
./watchdog.sh start                    # restart any instance that dies
./stop.sh                              # stop them all (and the watchdog)
```

Tools are run from the root. Each puts the root on `sys.path` itself and
re-executes under the project venv if the interpreter has no `krpc`, so
`./spaceplane/tools/quickglide.py` works from any shell. **Everything runs
under PyPy** when `.venv-pypy` exists (`kspSim/tools/pypysetup.sh`): `run.sh`,
the launcher's autopilots, every tool through `common.paths.use_venv`, and
the farm harnesses. `KRPCLAB_CPYTHON=1` (or `--cpython` / `PYPY=0` on the
harnesses) falls back to `.venv`, which `run.sh` creates on first use (`krpc`
pinned to the server mod's 0.6.0) and runs by path. The flight runs in the foreground, so Ctrl-C reaches it and it hands
the vessel back. `--set FIELD=VALUE` overrides a field of `Config` with type
coercion; unknown fields raise. No linter or formatter is configured. Tests
need no third-party packages.


## The farm

**Three things make a farm batch a lie, and none of them announce it.**
`pgrep -af spaceplane.autopilot` should show exactly one per busy instance --
the harness runs the flight as a *child*, so a killed batch used to leave it
flying (fixed, but check). `testInstances/syncSaves.sh check` should say every
instance matches `saves/` -- each clone keeps its own quicksaves and they drift
(on 2026-09-23 three clones were missing up to 17 saves and two held an older
`qs_plane`). And `swapon --show` should be near zero *after* the batch as well
as before: four instances left up for seven hours put 15.5 GB into zram and
biased the spaceplane's arrivals **27 km long** with the spread unchanged.
Restart the farm between sessions. See [docs/testInstances.md](docs/testInstances.md).

**Instances are only comparable if they are the same game.** Three of them
drifted to three different mod sets and it cost a session: one had
`BetterTimeWarp` (which redefines the warp rate and changed which deorbit
pass was flown, worth 23 km), another was missing the four mods `base/` keeps
on purpose because they replace the spaceplane's control-surface modules.
Diff `ksp<N>/GameData` against `base/GameData` before believing a comparison
across instances, and re-clone rather than patch.

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

These bite in both projects and are the ones a change most often violates.

- **Measure in game, not in the sim.** `boosterland/tests/fakeksp` models a point mass with
  near-instant pointing and no aerodynamic torque, and integrates its own truth
  with the scheme it is judging. Six changes that improved a sim sweep made the
  real flight worse. Treat a sim improvement as a hypothesis and confirm it with
  `quickfly.py` / `quickglide.py`. See [docs/boosterland/tuning.md](docs/boosterland/tuning.md).
- **A knob that changes nothing may be disconnected, not powerless.** Before
  concluding "X is not the lever", check that X moved what it commands. See
  spaceplane failure 10a, which cost this project a wrong headline finding.
- **One flight per configuration is not a measurement, and neither is
  three.** The spaceplane's along-track scatter is **sd 9.6 km over 22
  flights of one configuration** — so the first batch of any comparison is
  n flights of *one* arm, not one flight of n arms. An ordered
  dose-response over three-flight arms is not evidence of a gradient:
  spaceplane failure 27 is that mistake made, adopted, documented and
  retracted inside one session. Also 10d, and boosterland's accuracy
  history.
- **A correlation across flights the autopilot chose is not a sensitivity.**
  Interface speed against landing miss correlated 0.88 over 79 flights at
  11 km per m/s, and the real figure is 0.2-1.2: both variables are
  consequences of the burn, and the regressor was carrying the geometry's
  effect. If the quantity can be *set* — `savegen.py` sets it, on an entry
  save as well as an orbital one — set it and measure the slope directly.
  Spaceplane failure 30.
- **Read the signed miss, not the distance.** A great-circle distance cannot
  tell an overshoot from an undershoot; `long=`/`cross=` and the N/E offsets
  can. That ambiguity has cost this project two sessions.
- **Measure over several entry states, with `AIM_BIAS_*` at 0.** One quicksave
  is one separation state and a number tuned against it proves nothing about
  the next. Fit a calibration from two points, never from one flight's
  residual.
- **The propagator must fly the law the vehicle flies.** `landing_command` and
  the spaceplane's speed cap are shared by the predictor and the control loop
  on purpose: a prediction of a law nobody flies is a prediction of a
  trajectory nobody flies.
- **A missing answer must not be allowed to look like a good one** — return
  `None`, not a sentinel that a threshold test will accept.
- **A quantity measured from flight data describes the vehicle *as it was
  flown*, and every change to how it is flown silently invalidates it.**
  `ALPHA_TRACKING` was taken before the attitude controller was tuned and
  read 0.87 where the tuned vehicle delivers 0.74 — and the deorbit's stretch
  bound was making decisions on it. Re-take a derived table after anything
  that changes the plant, and prefer tables something recomputes.
  Spaceplane failure 23.
- **A guard whose stated reason does not survive inspection may still be the
  only thing enforcing a constraint nobody wrote down.** Removing one whose
  argument was airtight cost three flights and 110 km; what it was really
  doing was keeping the deorbit search out of the shallow regime the
  propagator cannot be trusted in. Find out what it does before removing it,
  and write *that* down. Spaceplane failure 21, boosterland failure 17.
- **A constant that has to be re-fitted after unrelated changes is a missing
  model wearing a constant's clothes.** `DEORBIT_LONG_BIAS_FRACTION` moved
  five times in one session, every time forced by a change elsewhere. The
  question is not "what value?" but "what quantity is this standing in for,
  and can the program compute it?" — here, the span of arrivals the glide can
  still reach (`guidance.deorbit_window`). Spaceplane failure 19.
- **A phase must hand over a state the next phase can fly, and a control
  law that steps is a law no vehicle flies.** Three of this project's
  landing failures are the same shape: the rollout commanded zero angle of
  attack at 90 m/s (a nose-over) after previously commanding none at all (a
  tail-scrape); the flare held wings level for its whole ten seconds and
  landed whatever cross-track it started with; and `aim` points the nose at
  the *velocity*, which on short final lands the vehicle crabbed -- 23
  degrees of sideslip at the first rollout tick, 51 two seconds later, gear
  torn off at 48 m/s on a runway it had just landed on. Ramp the command,
  and near the ground reference it to the runway rather than the airflow
  (`Autopilot.aim_runway`). Spaceplane failures 31 and 34.
- **A clamp can make two configurations fly identically, and the log will
  faithfully report the one that was ignored.** `deorbit_aim` returned
  `max(bias, fraction * arc)`, so with the fraction retired a configured
  `-3000` flew as `0` -- four in-game batches were flown believing they were
  aimed short. Spaceplane failure 33, and `testInstances/HANDOFF.md`'s
  "read the numeric value, never the label" from the other side.
- **A control loop that cannot serve its own commanded interval is flying a
  different vehicle, and the log has to say so.** Every spaceplane log ends
  with a `loop rate` line giving, per phase, the game-seconds between commands
  and the wall-seconds one tick costs. Read it before comparing two logs: a
  phase whose first number is far above the tick it asked for was measured
  through a latency, not a vehicle. `Config.TIMESCALE_GOVERNOR` keeps that
  from happening by making the time scale serve the interval rather than the
  other way round — matched against itself it took the entry's arrival from
  -3226 m sd 806 to +294 m sd 171 at unchanged throughput. Spaceplane
  failures 63 and 65.

- **Ask what the vehicle can do that the autopilot never commands.** Every
  constant in `config.py` rations an authority the code already has, so a
  search over values cannot find one that was never wired. Three were found
  in an afternoon just by asking: `ALPHA_MIN_DEG` is 0 on an airframe whose
  own swept table reads **ClA 8.8 at zero alpha** — it physically cannot
  unload and nothing in the log would ever say so; `HAC_BANK_MAX_DEG` is 45
  while the glide routinely flies **67**, so the cone's descent authority is
  a policy and not a limit; and nothing anywhere commands an airbrake,
  though every brake constant in the file is a wheel brake and the Shuttle
  this cone is borrowed from uses a speedbrake continuously. The table even
  stops at alpha 0, so half the polar of a cambered wing has never been
  measured. When a phase is short of authority, find out whether the
  authority exists and is merely uncommanded **before** re-fitting the
  constant that rations it.

- **A measurement that shares a constant's name is usually a different
  quantity, and the agreement is what makes it dangerous.** Three arms were
  spent on this in one session, each replacing a fitted constant with a
  live measurement that looked like the same thing. `HAC_LD` 1.86 is the
  ratio the cone must *plan* with; the vehicle *achieves* 1.42, and flying
  the achieved value put it 3.9 km from the gate — `HAC_LD`'s own comment
  had already recorded that failure ("1.35 was 25% low", LOG1315, overflew
  by 3.5 km). `MARGIN` discounts the *top* of the lift curve; `LiftTrim`
  measures lift at *whatever alpha is being flown*, and substituting it
  gave a stall speed 25% low (+1405 m against +900, 8 an arm).
  `APPROACH_BEST_LD` spans rollout-to-wheels **including the flat flare**,
  so the glide ratio is not it — and the committed 4.2 measures 3.96 in
  flight while the comment beside it says 2.08. In every case the two
  quantities agreed somewhere, which is exactly why the substitution looked
  safe. Before replacing a constant with a measurement, say what each one
  is a quantity *of*, and check they are the same one.

- **Re-run the tool; do not read the prose.** A number quoted in a comment
  describes the configuration that was flown when it was written.
  `HAC_LD`'s "tracking 1.07-1.11" reads 1.27-1.39 today and
  `APPROACH_BEST_LD`'s "2.08 sd 0.27 over 41 flights" reads 3.96 sd 0.41 —
  both were taken as current, and both were wrong in a way that changed a
  decision. `conesum.py`, `landsum.py` and `armsum.py` recompute all of it
  in seconds. Failure 23's rule, aimed at the comments rather than the
  constants.

- **Every few batches, stop and audit the whole vehicle and the whole
  flight -- not the phase you are in.** Two standing checks, on a schedule
  rather than when stuck: (1) *missing controls* -- list every actuator and
  setting the craft exposes through kRPC (control-surface axes, deploy
  angles, RCS, airbrakes, gimbals, drain valves, gear, brakes, SAS/throttle
  modes) and every one the autopilot never commands or leaves at whatever
  the craft file happened to set; the shuttle found five of those in one
  afternoon after ~2800 flights on one craft, and `Drain Mode` had been right
  only by inheritance. (2) *breadth* -- before the next batch on the phase
  you have been working, glance at the others' summaries (`oscsum.py`,
  `landsum.py`, `loop rate`, touchdown survival) on **both craft and more than
  one orbit**. Sessions here have repeatedly spent a day on the approach's
  last 900 m while the entry scattered 200 km on another airframe, or tuned
  one save while a second save's bimodality sat unread. The goal is accuracy
  across ships and orbits, and the largest error is rarely in the part you
  are looking at.

- **When three attempts on one mechanism come back null, the next thing to
  change is the method, not the fourth value.** A refuted mechanism is
  information about the mechanism; a *run* of refuted mechanisms is
  information about the approach that picked them. So change what is being
  varied: measure a different quantity, act at a different point in the
  chain, or replace the law instead of re-fitting its constant. The
  generality gap survived two hundred flights of propagator work and fell to
  a release valve nobody had thought to look at (failure 61); the approach's
  speed loop was re-tuned all session and what it needed was a law that could
  make speed as well as lose it (failure 65). A constant on its fifth value,
  a diagnostic that keeps agreeing with itself, and a fix that only works on
  the save it was fitted to are all the same signal. Write down which
  approach was abandoned, not only which value was.

- **A measurement whose only consumer is a hand-copied constant cannot be
  contradicted.** `planeprobe`'s subsonic lift was wrong by 1.8x for the life
  of the project because nothing read it except a human transcribing numbers
  into `config.py` — which set the stall speed, the approach speed, the gate
  and the flare for an aircraft that does not exist. Prefer a measurement
  something recomputes (`polar.py` reads it back off the logs, the alpha
  ceiling is learned in flight); where a constant really is transcribed, say
  in its comment what would disagree with it. Spaceplane failure 13.


## Where the detail is

Read the file that covers what you are about to touch; do not load them all.

| doc | read it when |
|---|---|
| [spaceplane/CLAUDE.md](spaceplane/CLAUDE.md) | anything in `spaceplane/`: where it stands, what is next, what is refuted |
| [boosterland/CLAUDE.md](boosterland/CLAUDE.md) | anything in `boosterland/` |
| [docs/testInstances.md](docs/testInstances.md) | running the parallel-instance farm, or when instances die |
| [docs/loopCost.md](docs/loopCost.md) | writing or changing a control loop, a kRPC call in one, or a farm/harness script |
| [docs/krpc.md](docs/krpc.md) | any kRPC call whose signature you are not certain of |
| [kspSim/CLAUDE.md](kspSim/CLAUDE.md) | flying anything in the simulator, or making a model of a new save |
| [saves/README.md](saves/README.md) | which save is which vehicle and state; adding a save |

**Keep this file short.** It is loaded into every session whole; the docs are
not. New material goes in the doc it belongs to -- an autopilot's state in its
own `CLAUDE.md`, a session's measurements in its `docs/<name>/journal.md` --
and only a rule that applies to *every* change belongs here.
