# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-11 ~1705.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-11 afternoon (the plan's dear windows)".

Defaults fingerprint **`bbcacd22`**.  Committed locally, **not pushed**
(the user: "do not push").  Farm stopped, sleep inhibitor released.
The how-it-works PDF was **not** updated this session (see Next, 3).

**Standing (the user, 2026-10-11): never ask before starting the farm or
flying a batch** -- in root `CLAUDE.md`.

## FIRST THING: fly the round that was stopped

The user stopped the session as `short-1011` started (killed in round 0;
`logs/rot-short-1011-c0.txt` and any LOG99xx from it are partial --
ignore them).  Re-fly it as is, on a freshly restarted farm:

```bash
cd testInstances && ./nosleep.sh start && ./start.sh && cd ..
spaceplane/tools/multirot.sh short-1011 4 \
  "qs_shuttle2_i90_rigoff|" "qs_shuttle2_i90_rigoff|HAC_SHORT_POLAR=True" \
  "qs_shuttle2_e500i45_rigoff|" "qs_shuttle2_e500i45_rigoff|HAC_SHORT_POLAR=True" \
  "qs_shuttle2_rigoff|" "qs_shuttle2_rigoff|HAC_SHORT_POLAR=True"
```

~19 min a cycle, ~76 min.  Group by each log's `config:` line (arm numbers
rotate).  Promote `HAC_SHORT_POLAR` if i90/e500i45 short landings fall
without rigoff moving; delete it otherwise.  It needs
`logs/conepolar/Untitled_Space_Craft_31.json` (present; regenerate with
`spaceplane/tools/conepolar.py logs/LOG9[6-9]??` -- not tracked).

## What changed today (all default unless marked)

- **Cheap deorbit windows.**  The plan was deterministic all along (the
  morning's "four windows on one save" were four saves).  It took the
  first solvable window whatever it cost: e500 244 m/s at periapsis, e500i45
  180 m/s.  Now a window must cost no more than the apoapsis burn to a
  surface periapsis (`deorbit_dv_floor` at `DRAIN_RESERVE_PERIAPSIS_M`);
  dearer ones are skipped half an orbit at a time, cheapest taken at the
  horizon.  cheap-1011 (LOG9746-9793): **e500 12/12 v 4/12 (5 lost),
  e500i45 8/12 v 1/12** (arrivals -0.35..+1.8 km v -17..-31).
- **The burn node.**  The node carries surface retrograde x dv at its time
  resolved into prograde/normal/radial (`guidance.node_components`; normal
  sign read back from `burn_vector` once, logged as `burn node: ...`).  SAS
  holds Maneuver to commit, then **stability assist through the burn**.
  `DEORBIT_SAS_SURFACE` deleted.  node3-1011 (LOG9890-9913): 0/24
  stranded burns, e500i45 7/12 v 5/12, i90 10/12 v 9/12.
- **Burn stop through a drifting nose**: once owed reached 0 the stop test
  runs on misaligned ticks (it used to return early forever -- node-1011's
  4/8 tail-first water landings).
- **`HAC_LOAD_MAX` 3 g**: cone alpha capped at the lift of 3 weights at the
  q flown (LOG9641's 441 m/s / 28.5 kPa entry: alpha 32 is ~8 g on the
  table).  Never bound in 16 i90 flights; unmeasured on a fast entry.
- **`GLIDE_PITCH_RESCUE` deleted**: no dive on the farm; i90 4/8 and 5/8
  v 6/8 (node-1011, LOG9794-9841).
- **Off, unmeasured: `HAC_SHORT_POLAR`** (the round above).

## The finding behind HAC_SHORT_POLAR

Every inclined-orbit miss is the cone running short, not the deorbit or
glide: arrival within ~1 km, cone entered with the usual ~14.0 km energy
height, exit `out of height` 0.4-3 km low, 4-6 km from the gate, landing
1.3-7.7 km short (i90 in the water).  conesum: short flights flew the cone
at L/D 1.94-2.25, alpha 8-9 flown, 99-108 m/s; good ones 2.25-2.75, alpha
~7, 104-113 m/s.  Flown polar: L/D 2.45 at 11 deg, 2.67 at 9, 2.87 at 7,
2.97 at 5.  The law: once even the tightest circle is too long for the
height, cap alpha where the polar gives `total/available` times the ratio.
It is aggressive (10% short: 9 -> 5.6 deg) -- watch for long arrivals.

## Where each orbit stands (latest farm, default)

rigoff 13/13; e500 12/12; i90 ~9-10/12 (losses: cone short -> water);
e500i45 7-8/12 (cone short, 1-3.5 km).  ~87% pooled against the goal of
>= 95% and 0 lost.

## Next, in order

1. Re-fly `short-1011` (above); promote or delete.
2. If the cone is still short on inclined arrivals: check whether the
   deficit scales with the heading change the cone flies (the glide's
   TAEM target is a constant, `guidance.taem_target`) and price the turn.
3. **How-it-works PDF** (`docs/spaceplane/howItWorks.pdf`): the DEORBIT
   section is stale -- cheap-window plan, burn node + stability-assist
   hold, the stop test, the cone's load cap (and HAC_SHORT_POLAR if
   promoted).  Then push when the user allows it.
4. Older items: `ABORT_NORTH`, `GROUND_SPOILER_ON_NOSE` (both off,
   unflown on the farm), `docs/spaceplane/constantsAudit.md`.
5. The live craft (`shuttle|31`) has no flown polar of its own
   (`HAC_SHORT_POLAR` would do nothing live until it does).

## Traps paid this session

- **Read the save name off each log before comparing.**  The handoff's
  headline was four saves misread as one.
- A `&&` chain ending in `| tail` commits on a failing test -- check the
  suite's own exit status.
- landsum drops some logs (44 of 48); for outcome counts use the
  harness lines in `logs/rot-*.txt` joined to each log's `config:` line.
- kspSim has no `qs_shuttle2_rigoff` model: orbit specs go on
  `qs_shuttle2@pe=..`, and it needs `DEORBIT_SAS_ALIGN=False` (SAS modes
  not simulated -- the burn node can only be judged on the farm).
- Six fresh instances put 10-17 GB into zram at boot; normal.

## Traps from before (still true)

- Kill sims with `pkill -f "kspSim[.]run"`.
- kspSim lands short of the game (gap 7); its subsonic lift is wrong
  (gap 8) -- don't screen cone energy there.
- multirot: arm counts must divide 12; arm numbers rotate per cycle.
- `warp_to` is blocking: the panel does not answer during it.
