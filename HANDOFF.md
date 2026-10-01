# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-09-30 evening: the twin-fin shuttle" and "... night: fuel,
and why the shuttle arrives high"), failures in `docs/spaceplane/failures.md`
(latest 102).

Last written **2026-10-01 ~04:30**, spaceplane, unattended overnight session.
Defaults fingerprint **`b5de541e`**; flown behaviour = this morning's except
**`NOSE_WHEEL_FRICTION=1.0`** (the user's rule: nose-wheel friction control
off, manual at 1; verified in game). **Everything else built tonight is off.**
Committed through `c3825a5` (plus the final docs commit). Offline suite 828
OK (spaceplane 656 OK after the last change). Farm **stopped**, inhibitor
**released**. The suspend was interrupted by the user's return.

**The goal set by the user -- "mostly reliable landings" -- was NOT reached.**
Best game result tonight is ~1 in 4 intact from orbit for any configuration.
What was reached is a causal chain, below, that says why, and that the
shuttle's remaining problem is largely the airframe's pitch trim.

## Where it stands

- **New craft: twin wingtip fins** (`saves/craft/SPH/shuttle.craft`),
  saves `qs_shuttle2`, `_inc`, `_high` (spliced into qs_shuttle's orbit), and
  bench `qs_shuttle2_low` (spiral entry, 2.5 min). **Lateral departures
  cured** (6v6). First intact orbital shuttle landing (LOG4135).
- **kSpSim model of the new craft** exists (`kspSim/models/qs_shuttle2*.json`,
  flight-test battery not run). Sim screens: `kspSim/tools/simarms.sh`
  (6-12 orbital flights in 2.5-3.5 min); group with
  `spaceplane/tools/armgroup.py`. **Sim and game disagree on the late glide**
  (sim wet tracks alpha within ~2 deg, game falls 2-20 deg short; sim cone
  L/D ~2.4 vs game ~1.9): the sim's "4/4 with HAC_ENTRY_AFFORDABLE" did not
  carry over (game 0/4). Treat sim results on the entry/cone as hypotheses.
- **The causal chain (game):**
  1. Every landing loss is energy at the spiral exit (-6..+6.6 km).
  2. The spiral's exit tracks its *entry*, which varies 8-21 km on one save
     and one configuration -- set by the glide's miss at handover (`long=`):
     on target -> 13.5-16.7 km; long 1.7-5.5 -> 18-21; short -> 12 km at the
     altitude trigger, then out of height.
  3. The glide's miss is set by **pitch trim below Mach 4**: with the 1.9 t
     nose fuel aboard it flies 2-20 deg *under* its commanded alpha (no drag,
     bank pinned at 70 -> long/high, LOG4171); dumped, it flies up to 19 deg
     *over* (tail-heavy, more drag -> short/low, LOG4241). Dumping in steps
     against alpha tracking (`DRAIN_TRIM_LOOP`) tracks alpha but entries
     still scatter 8-21 km (n=8).
  4. The spiral cannot absorb more than ~+-1.5 km: the 50 deg weave adds
     <=1.56x path, a lap at the entry-speed radius costs 6-46 km of height.
     More authority (`HAC_LAP_AT_TARGET_SPEED`, `HAC_RADIUS_MIN_M=1000`,
     `HAC_WEAVE_MAX_DEG=75`) narrowed exits to -1.8..+1.9 (n=4, 1 intact).
- **The landing itself works when it is handed a good state**: LOG4166
  (bench) 31/31 at 52 m/s / 4.8 m/s sink; LOG4241, 4296, 4285 intact at
  49-63 m/s. Touchdown *speed* matters on this craft: >70 m/s breaks it even
  at 3-6 m/s sink.

## The last batch: a consistent arrival, for the first time

`rot-ballast2-1001` (LOG4311-4319, ksp1-3; ksp0 never booted): the stack +
affordable entry + spend authority (`HAC_LAP_AT_TARGET_SPEED`, Rmin 1000,
weave 75) + `DRAIN_RESIDUAL_MACH_MAX=3.5` + **`DRAIN_RESIDUAL_KEEP_UNITS`**:

| kept below Mach 3.5 | alpha error M1-3 | entry h | spiral exits | intact |
|---|---|---|---|---|
| **200** | -6..-13 | 12.4-13.1 km (one 22 km short) | **+2075 +2148 +2223 +2349** (-7998) | 1/5 |
| 300 | -2..+8 | 12.0-17.1 km | +1785..+4380 | 2/4 |

**Keep 200 delivers 4 of 5 to the gate with the same +2.1-2.3 km** --
consistent, so a calibration, not scatter. Those four died in the approach
(82-96 m/s, 31-47 m/s sink): it cannot spend 2 km. **Start here**: with keep
200, find why the cone exits a constant +2.2 (the exit allowance
`HAC_EXIT_SURPLUS_DERIVED` = a lap's cost ~6 km lets it go; weave saturates)
and spend it before the gate; confirm n>=6.

## Recommended next step: the airframe, then the glide

**Ask the user about a fixed pitch-trim fix in the craft.** The fuel can't be
both hypersonic ballast and transonic trim. Candidates: move the 1.6 t
monopropellant tank (now 0.6 m behind the CoM) forward toward the nose as
permanent ballast and fly the LF/Ox fully drained at Mach 3.5; or a
canard/forward lifting surface. Result of `rot-ballast2-1001` (fixed partial
ballast 200 vs 300 units, below) says which ballast level the glide wants.
Then: re-fly the glide's arrival scatter (`long=` at handover) as THE metric,
6+ per arm, before touching the spiral again.

## Built tonight (all off unless noted)

| flag | status |
|---|---|
| `NOSE_WHEEL_FRICTION` | **default 1.0**: nose wheel friction control manual (user rule) |
| `DRAIN_TO_BURN` | drain to the solved burn x1.25+10 m/s before committing. Works (430->69 units, burn exact) but **refuted**: 3/3 crashed short, hypersonic departures (the nose fuel is ballast) |
| `FUEL_TO_NOSE` | first COAST tick: pump all tanks front-to-back. Verified in game (1.4 s). Use it on the live flight, tanks start full |
| `DRAIN_RESIDUAL` (+`_MACH_MAX`) | existing; dump works (0.4-1 s). 0.8 = wet glide (long); 2.5 = drained glide (short) |
| `DRAIN_RESIDUAL_KEEP_UNITS`, `_FINAL_MACH` | two-stage dump: keep N units as trim, rest at the final Mach. Batch `rot-ballast2-1001` |
| `DRAIN_TRIM_LOOP` (+5 constants) | dump 25 units every 6 s while alpha is >2 deg short (Mach 3.5..`DRAIN_RESIDUAL_MACH_MAX`). Tracks alpha; entries still scatter (game 1/8 intact) |
| `PREDICT_RESIDUAL_DUMP` | predictor drops the residual's mass at the dump Mach. **Null** in the sim (short arrivals are drag from over-rotation, not mass) |
| `HAC_PAST_BEFORE_GATE_DEG` | before the gate, a tangent point <=N deg past the rollout costs the run to the gate, not a lap (LOG4152's 13-deg phantom lap). In the flown stacks at 60 |
| `HAC_LAP_AT_TARGET_SPEED` | laps priced at the cone's speed. Sim: overspends when entries are low; game (with Rmin 1000, weave 75): exits -1.8..+1.9 |
| `HAC_ENERGY_BUDGET` | existing, off since LOG1941. Counts kinetic energy: bench exits +1.6 -> +0.7 km. In every stack flown tonight |
| `HAC_ENTRY_AFFORDABLE` | existing. Sim 4/4; game 0/4 wet, 1/8 with trim |
| `HAC_SPEED_PATH` | existing. Stops zoom climbs (bench LOG4168 zoomed to 37 m/s). Sim helped, game ran the cone low (n=2) |
| tools | `lateralsum.py`, `armgroup.py`, `kspSim/tools/simarms.sh` |

The "stack" in every batch: `HAC_ENERGY_BUDGET HAC_PATH_WRAP_TO_GATE
HAC_PAST_BEFORE_GATE_DEG=60 DRAIN_RESIDUAL FUEL_TO_NOSE APPROACH_BANK_BY_ROLL
APPROACH_FLARE_FACTOR=1.45`. Batches: `rot-shuttle2-0930`, `rot-drain-0930`,
`rot-ballast-0930`, `rot-stack-0930`, `rot-gap-0930`, `rot-m25-0930`,
`rot-afford-1001`, `rot-spend-1001`, `rot-ballast2-1001`; sim
`sim-screen1..8.txt`. Void: LOG4121-4123, 4145-4148, 4175-4176, 4244-4245,
`rot-ballast-0930-void`, `rot-ballast2-1001-void`.

### Older (2026-09-30 afternoon), unchanged

| flag | status |
|---|---|
| `HAC_PATH_WRAP_TO_GATE` | **Real bug fix** (failure 102): the cone's plan credited path to a tangent point past the rollout; the scan chose those circles; cone read on-profile while 1.1-2.8 km high. Flown 6v6 both craft (`rot-wrap-0930`): `qs_plane` exit +1136..+1504 -> +704..+1035, 3/5 intact vs 5/6 (LOG4112 broke up at 71 m/s contact). Shuttle unchanged (lateral). Candidate default once the old craft's landing is re-checked at the lower exit |
| `HAC_LD_AT_TARGET` (+ `HAC_LADDER_STEP_M`) | Plans the cone's path on the swept table, slice by slice to the gate. **Unusable**: from orbit the in-flight re-probed subsonic rows read 2x drag and 40% less lift than STANDBY (LOG4100-4102 `hac ladder` event; bench LOG4099 fine). Cause unknown -- see Next 2 |
| `HAC_FLAP_ARREST_EXCESS` | cone brake prices only sink over the cone's glide. Unflown. The brake's real problem is `HAC_FLAP_BRAKE_IGNORES_ROLL` taking the elevons from roll (4/5 brake flights departed) |
| `HAC_EXIT_PAST_DEG` | exit test (only) accepts a little past the rollout (LOG4104 stranded 13-17 deg past). Unflown; not offline-tested (kRPC-bound) |
| `APPROACH_MUSH_RECOVERY` | **disconnected on defaults**: never fires with the S-turn on (approach dives, never mushes). Only meaningful with `APPROACH_SCURVE_MAX_DEG=0` |
| `hac ladder` log event | every 60 s of cone under `HAC_LD_AT_TARGET`: per-slice speed, trim alpha, ClA/CdA, lift trim, ratio |

Older flags from last handoff unchanged: `APPROACH_BANK_BY_ROLL`,
`ALPHA_TRIM_LOOP` (harmful as built), `FLARE_LOAD_LOOP` (refuted),
`HAC_FLAP_BRAKE_ON_SURPLUS`/`IGNORES_ROLL`, `HAC_SPEND_AS_SPEED` (n=2, not a
verdict).

## Next, in order

1. The airframe trim question (above) -- the user's call; offer the options.
2. Keep 200 + spend the constant +2.2 km in the cone (see above).
3. The glide's arrival scatter as the metric: `long=` and `h=` at
   `GLIDE -> HAC`, 6+ per arm, on the farm (the sim's late glide is wrong for
   this craft: run `makecraft.sh`'s flight-test battery and fix that first if
   the sim is to be used for it).
4. Then the spiral's authority (`HAC_LAP_AT_TARGET_SPEED` + Rmin 1000 +
   weave 75) on consistent entries.
5. Older open items: `HAC_LD_AT_TARGET`'s bad subsonic rows; wrap fix
   toward default on qs_plane; the user's rollout rules
   (`ROLLOUT_BRAKE_FULL_ON_CONTACT` still off).

## How to run

`spaceplane/tools/rotfly.sh "0 1 2 3" ROUNDS OUT TREE "save|A=1;B=2" ...`
from a frozen tree (copy `spaceplane common tools run.sh`, symlink the rest).
Sim: `kspSim/tools/simarms.sh qs_shuttle2 K "A=1 B=2" "..."` from a frozen
tree (also copy `kspSim/*.py tools data restart.sh`, symlink `kspSim/models`,
`.venv-pypy`, `.pypy`). Never run >6 sims beside the farm (~1 GB each).

## Traps paid


- A flag's fire condition can be unreachable on the defaults
  (`APPROACH_MUSH_RECOVERY`): count the ticks where it could fire after
  round 0.
- A scan that maximises a model finds the model's errors (failure 102).
- The in-flight aero table is not the STANDBY table at conditions not yet
  flown; `mdl=` agrees with `act=` only where the vehicle is.
- Restart the farm about hourly: swap reached 2.6 GB in 45 min.
- Void logs: LOG4121-4123 (killed). Batches stopped at round boundaries:
  `rot-mush-0930` (3 rounds), `rot-conewrap-0930` (2), `rot-coneld-0930`
  (2), `rot-ladder-0930` (1).
- `launch_vessel` hangs forever on a pre-flight dialog if the named crew is
  aboard another vessel in that save; name an unassigned kerbal, and use
  keyword arguments (positional `recover` is mis-coerced).
- `bad60` (bank >60 off command) counts reversal *lag* too -- it did not
  separate the craft; sideslip did.
- The farm reached 4.5 GB of swap after one 50-minute batch from a fresh
  start: restart it between batches.
- **rotfly launches round N's flights the moment it prints `-- round N`**:
  killing the driver after seeing the header orphans them (LOG4175-4176
  flew under a kspSim probe on the same instance).
- `start.sh` reported ksp0 started; it never opened its port (04:00): check
  every port before launching, and pass only live instances.
- **Twelve kspSim servers ran all night unnoticed** (started ~23:10 and
  ~00:40, found 07:10): the cleanup grepped `kspSim/run.py` but they run as
  `-m kspSim.run --instance N`, so it killed nothing and printed 0. They are
  the 7-11 GB of zram, and **every game batch from `rot-m25-0930` on flew
  beside them** (CPU load 17 on 16 cores, swap) -- arms were paired, but
  treat those batches' absolute numbers (including the keep-200 +2.2 km)
  as suspect until re-flown on a clean box. Kill sims with
  `ps -eo pid,args | grep "[k]spSim\.run --instance"` from a script file.
  KDE's `baloo_file` also holds ~2.9 GB indexing the 100 MB sim models.
- Re-run `conesum` before quoting `HAC_LD`: I quoted 1.55-1.65 from memory;
  the night's batches read 1.89 median (configured 1.86).
- Sim exits on the shuttle are biased (~1.4 km on the bench) and its late
  glide tracks alpha far better than the game: a sim win on the entry or
  cone is a hypothesis.
