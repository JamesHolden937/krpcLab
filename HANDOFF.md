# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-09-30 evening: the twin-fin shuttle", and the afternoon
before it), failures in `docs/spaceplane/failures.md` (latest 102).

Last written **2026-09-30 late evening**, spaceplane. Defaults fingerprint
**`89b7daaf`** (unchanged; flown behaviour is the morning's `ebd6dbb2`).
**Nothing committed.** No code changed this session; offline suite not
re-run (828 passed last session). Farm **stopped**; sleep inhibitor
**released**. The user's live defaults fly as this morning.

## Where it stands

- **The user rebuilt the shuttle: twin wingtip fins** replace the single
  tail fin (`saves/craft/SPH/shuttle.craft`). In the farm as
  **`qs_shuttle2`** (+ `_inc`, `_high`), spliced into `qs_shuttle`'s orbital
  state so a pair measures the craft alone (`saves/README.md`). Audited
  (`actuators.py`): both fins live on every axis; roll MoI +32%.
- **It cured the subsonic roll departures.** Paired 6v6 on defaults
  (`logs/rot-shuttle2-0930.txt`, LOG4124-4135): sideslip >10 deg on 0-1
  airborne ticks per flight against 2-13 on the old craft; peak <=11 against
  12-30. **LOG4135 is the first intact shuttle landing from orbit** (31/31,
  6 m/s sink at 67 m/s, +1569 m along -- past the runway end).
- **What loses it now: the flare inherits bank and touches down banked**
  (LOG4129 -45 at contact, LOG4131, LOG4133), with wingtip fins now the
  first thing to hit. Sources: the centreline capture after the S-turn stop
  rings (~25 s period against a 4-6 s roll lag; roll 8.5-12.8 deg/s,
  `time_to_peak` 5.5 s), and the approach dives (LOG4127, 133 m/s into the
  ground). The cone exits +1.0..+5.0 km high on the new craft.
- **Old craft (`qs_plane`) lands** intact on nearly every flight; the old
  shuttle (`qs_shuttle`) is now a reference, not a target.

## Flags (built 2026-09-30 afternoon, all off; none flown this evening)

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

1. **Hand the flare a level vehicle on `qs_shuttle2`.** Candidates,
   autopilot-side: stop the S-turn and the capture's bank early enough to
   level at the *measured* roll rate (`roll rate measured in APPROACH` is
   logged: bank/rate + settle, in seconds, before the flare door -- a law,
   not a distance); cut the centreline capture's gain to what the roll lag
   supports. Metric: |bnk| at the last FLARE tick and at contact, plus
   intact, 6+ per arm, `lateralsum.py`. Remember the fins are now on the
   wingtips: bank at contact is a fin strike.
2. **The cone's high exit on the new craft** (+1..+5 km; LOG4131 +5022,
   laps=0). `HAC_PATH_WRAP_TO_GATE` (below) is the existing candidate; fly
   it on `qs_shuttle2` with the old craft.
3. **The approach dive** (LOG4127): S-turns at alpha 2-6 spend height as
   speed. `APPROACH_MUSH_RECOVERY` exists but never fires with the S-turn on.
4. From last session, still open: why the in-flight table's subsonic rows go
   bad (blocks `HAC_LD_AT_TARGET`); `HAC_PATH_WRAP_TO_GATE` toward default on
   `qs_plane`; the user's rollout rules; `DRAIN_RESIDUAL`, `FLARE_SHALLOW`.
5. Breadth: `qs_shuttle2_inc`/`_high` unflown.

## How to run

`spaceplane/tools/rotfly.sh "0 1 2" ROUNDS OUT TREE "save|SETS" ...` from a
frozen copy of the tree (copy `spaceplane common tools run.sh`, symlink
`logs saves testInstances .venv boosterland kspSim docs`). To stop at a
round boundary, kill only the `bash spaceplane/tools/rotfly.sh` driver (the
lowest PID); the round's flights finish. Readers: `flightsum.py`,
`conesum.py`, `flaresum.py`, `appenergy.py`, `lateralsum.py` (new: subsonic sideslip/bank
error, cone exit, touchdown per flight). `qs_shuttle_low` reaches the
cone in ~2.5 min (bench; its table state differs from an orbital flight's).

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
