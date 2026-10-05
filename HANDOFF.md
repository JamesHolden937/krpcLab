# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-04 evening: it is not a rollover -- the wings come
off").

Last written **2026-10-04 ~20:20**. Defaults fingerprint **`db95a67f`**
(was 6e5178e5; it changed only because off flags were added, with no
behaviour change). Full offline suite OK (887). Everything is committed.
Farm **stopped**, inhibitor **released**.

## What changed

Nothing was promoted. Instrument rebuilt and two off flags added, both null:
- **`WHEEL_WATCH_S`** (cbed8f0): now from gear-down; re-reads the wheel
  list and part count every `WHEEL_WATCH_RELIST_S` (0.5), logs each
  wheel's `state`/`deployed`/`grounded`/`broken`. Use `WHEEL_WATCH_S=6`.
- **`FLARE_SPEED_BUDGET`**: raises the exp flare's touchdown sink (up to
  `FLARE_SPEED_TD_MAX_M_S` 5) when the speed would reach the craft's own
  landing-mass stall (45.5 m/s on the shuttle) before the schedule lands.
  `bud= fdc= td=` in FLARE telemetry. Null.
- **`ROLLOUT_RAMP_FROM_ATTITUDE`**: the tail cap bounds the rollout
  schedule inside the ramp (it was applied after it, a step 10-14 -> 4.4
  deg on the first ground tick) and the ramp starts from the contact
  pitch. Null on survival -- but the step it removes is real (failure 31's
  shape); keep it in mind before promoting anything else in the rollout.

## The finding: it is not a rollover, the wings come off

Each LY-60 main gear is mounted **on a delta wing**, 5.6 m out; each wing
carries two elevons, a tail fin and the gear (5 parts). "Lost 10 of 31" in
0.4 s at contact is both wings. The farm's `testInstances/ksp<N>/KSP.log`
records the parts as **"Exploded!!"** (collision over crash tolerance,
not a joint break), usually an RV-105 RCS block first, then the outboard
elevon, then the wing. The "one main gear listed" lead was this: the list
is read after the wing has gone. No part is lost before contact.

Two populations, set by the save (cone saves `qs_s2_hac0-5`):
- **hac1/hac3**: door at ~75 m/s, a float at ~20 m while the speed falls
  47 -> 38, contact at 37-39 m/s and **8-10 m/s of sink**: ~all broken.
  The flare can't fix it (FLARE_SPEED_BUDGET); the door speed is short.
  hac1 also flies the whole flare at +7 deg of slip, 250 m off centreline.
- **hac0/hac5**: contact 47-50 m/s, 3-5 m/s sink, pitch +5. About half
  still lose a wing. The signed alpha (`aoak`) falls +9 -> -2..-6 within
  0.4 s of contact with full nose-up input: the mains are **4.0 m behind
  the CoM** and the contact pitches the nose onto its wheel.

| batch | logs | arms | read |
|---|---|---|---|
| sav-wheels3-1004 | 5639-5674 | defaults + WATCH | no gear lost pre-contact; split by save |
| sav-budget-1004 | 5675-5710 | defaults / FLARE_SPEED_BUDGET | null (38.5 vs 37.5 m/s on hac1/3) |
| sav-ramp-1004 | 5711-5746 | defaults / ROLLOUT_RAMP_FROM_ATTITUDE | null (gentle intact 3/8 vs 5/9) |

## Next, in order

1. **Change the method: instrument the game, not the autopilot.** Two
   autopilot-side fixes on the wing loss came back null. Write a tool that
   merges each instance's `KSP.log` "Exploded!!" lines (wall-clock) into
   the matching `LOG<n>` by flight start/end time, so every log says which
   part hit first and when, relative to `contact:`. Then sort the gentle
   touchdowns by first part: RCS block (tail strike / bounce), elevon
   (deflected surface on the ground), or wing.
2. **A craft variant** (`write-craft-files-for-generality`): the mains on
   the fuselage, or moved forward toward the CoM, in a copy of
   `qs_shuttle2` -- *ask the user before touching the reference craft*.
   If the variant keeps its wings at 3-5 m/s, the airframe is the limit
   (as the user said of the old craft) and the autopilot's job is the door
   speed.
3. **The door speed on hac1/hac3**: they reach the flare at 75 m/s against
   the 84 the approach schedules (`APPROACH_FLARE_FACTOR` 1.75 x 48).
   Why the approach delivers less there (hac3: door at 100 m, sink 15).
4. Carried over: the split-rudder brake; touchdown aim (~half land past
   the far end); the 45-70 deg glides; the weave; the phantom lap.

## Traps paid this session

- `aoa=`'s second number is **unsigned**; read `aoak=` for nose-down.
- `pin=` is pitch input / pitch assist / assist error (deg), not three
  axes.
- `airframe.stall` returns the transcribed 48 for any craft while
  `AIRFRAME_DERIVED` is off; the craft's own stall is `env.stall_speed`.
- KSP's own log (`testInstances/ksp<N>/KSP.log`) has every part explosion
  with a wall-clock time; it was never read before and it answered in one
  grep what a day of kRPC instruments couldn't.
- Swap still reaches 16-17 GB per 36-flight batch; restart between.

## How to measure

Cone saves: `./spaceplane/tools/savefly.sh 6 logs/sav-X.txt "WHEEL_WATCH_S=6;LOG_INTERVAL_UT=0.1" "...;FLAG=..."`
(~18 min for 36 flights; every save sees every arm 3 times). Per-contact
table: the `contact:` line (sink/speed/bank/pitch/alpha/slip) and the
`DOWN:` parts count.
