# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-03 night: the long landing is the approach, and the
cone's phantom lap").

Last written **2026-10-03 ~21:15**. Defaults fingerprint **`6e854d9f`**
(unchanged this session). Full offline suite OK (875). Everything is
committed; this file is in the last commit. Farm **stopped**, inhibitor
**released**. Code change this session: telemetry only (`ral=`, below).

## FIRST THING: run this batch (the user asked for it)

The session was wrapped up early, so this batch was started and killed
after round 0. Fly it in full on a fresh farm, then analyse it and act on
the result:

```bash
./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 4 logs/rot-phantom-<date>.txt . \
    "qs_shuttle2|" "qs_shuttle2|HAC_PATH_WRAP_TO_GATE=True;HAC_PAST_BEFORE_GATE_DEG=60"
```

From orbit, 12 flights per arm, ~29 min (round ~7 min). **Read the cone's
exit surplus first** (`HAC -> APPROACH ... h=X (needed Y)`, X-Y), and how
many HAC lines read `turn=` > 300 or ` short `. Only then read the landing.
The grep that counts them is in the journal section. Round 0 of the killed
run is in `logs/rot-phantom-1003-void.txt` (LOG5243-5248, 3 per arm). That
is too few to read. Don't pool it.

**`60` is a tolerance, not a fitted value, but it is still an arbitrary
number.** The user asked about this. If the mechanism works, don't promote
"60". Promote the constant-free form: **before the gate (`x < 0`), any
tangent point past the rollout costs the run to the gate.** The code's
`distance * cos(angle) < 0` test already tells "past the gate" (a real
lap) apart. The bound reduces to "any wrap before the gate", so the
constant should go. Write that version in `guidance.hac_path`, add an
offline test (there is one for WRAP and none for PAST), and fly it.

## Where it stands

From orbit on defaults (`rot-aim-1004` arm0 + `rot-newdef2-1003`), the
shuttle lands intact about 3 times in 12. Sideways is solved: stops on
the runway are within ~40 m of the centreline. Along-track is not.

**This session's finding: the long landing is not the aim.** It breaks
into three parts, measured on LOG5207-5242:

1. **The flare's float is steady**, 1.0-1.3 km from the door to the
   wheels. That is the door's energy height `h + (v^2 - v_td^2)/2g` (about
   430 m) times L/D ~3, and it predicts LOG5183's float to within 100 m.
2. **The approach's door scatters +-1.5-2 km around its aim**, on every
   aim tried. The cone hands the approach +800..+1260 m of surplus from
   orbit, and the approach can't spend it. The weave is held to
   `APPROACH_SCURVE_CROSS_M` = +-300 m of the centreline, a constant from
   the old craft. The shuttle's turn radius at 40 deg bank is about 1.2 km
   (`v^2 / (g tan bank)`, 100 m/s), so inside that band the track reaches
   only ~28 deg (LOG5233: hdg +-15..28, `sc=45`, `sat=1.00`). That spends
   ~100 m of height. LOG5233 crossed the threshold 1100 m up with `exc`
   +480 and touched down 1.9 km past the aim.
3. **The cone's phantom lap** (failure-102 family, journal 2026-09-30). In
   13 of 36 flights, `turn=` flips between 0 and ~347 deg. The tangent
   point is a few degrees past the rollout and wraps to a full lap. The
   plan reads `need` 40-50 km against ~3 km and calls itself `short`. It
   then stops weaving and flies straight at the gate, high. The two fixes
   (`HAC_PATH_WRAP_TO_GATE`, `HAC_PAST_BEFORE_GATE_DEG`) were built on
   2026-09-30. They were only flown in large stacks, or on the old
   single-fin shuttle when its lateral control was broken. Neither was
   separated or promoted. **That's the batch above.**

Also: **7 of 36 flights reach the cone 4-8 km long from the glide and roll
out 6-10 km high** (LOG5210, 5211, 5219, 5224, 5236, 5237, 5239: short HAC
phases of 30-47 lines). That's an upstream glide/arrival problem, separate
from the phantom.

## Measured this session

**`rot-aim-1004.txt`** (LOG5207-5242; 6 rounds, 3 arms x 12, shuttle from
orbit, fingerprint 6e854d9f). Stop is metres past the threshold.

| arm | intact 31/31 | lost (0 parts) | stop median, good arrivals |
|---|---|---|---|
| defaults (aim 2400) | 3 (+2 splashed whole) | 1 | ~2200 |
| `TOUCHDOWN_AIM_M=1200` | 3 (+2 splashed whole) | 1 | ~1960 |
| `TOUCHDOWN_AIM_DERIVED` (computes 0) | 1 | 3 (td 57-76 m/s) | ~1250 |

The aim shifts the stop as expected, but **none of the arms is promotable**.
The derived aim loses more vehicles: LOG5240's door was at 370 m with
60 m/s of sink, the "near aim dives" mechanism. `TOUCHDOWN_AIM_M=1200` was
a probe value, not a candidate.

**Last session's refutation of `TOUCHDOWN_AIM_DERIVED` was invalid.** It
was flown from the cone saves (`save-aim-1003`). `GATE_FROM_APPROACH`
re-places the gate at engage as `GATE_ALT_M * 4.2 - aim`. With aim 0 the
gate moved from 6000 to 8400 m, away from the gate the saved cone had been
planned to (LOG5196: "gate: 8400 m"). That explains the negative surplus,
the slow doors and the doors 1.5-4 km off the centreline. **Any change to
the aim, the gate or `APPROACH_BEST_LD` moves the gate, so it can't be
measured from `qs_s2_hac*`. Fly it from orbit.**

**Telemetry `ral=`** (commit 2888089): the signed distance along the runway
from the threshold, on every line. `rwy=` is unsigned and hid doors up to
4 km *short* of the threshold (LOG5217, 5227: landed before the runway).

## Next, in order

1. **The phantom batch above**, then its constant-free form.
2. **The approach's dissipation authority.** Replace
   `APPROACH_SCURVE_CROSS_M` 300 (a constant) with a band scaled by the
   turn radius `v^2/(g tan APPROACH_BANK_MAX_DEG)`, and check the S-turn
   stop (`APPROACH_SCURVE_STOP_M` 4000) against the capture's time to
   recentre from that band. Read `exc`, `hdg`, `sc` and `sat` on the
   approach lines. Goal: the door within ~300 m of the aim.
3. **Then the aim** = the touchdown zone minus the *predicted* float.
   Predict the float from the door energy times the measured L/D, as in
   point 1 above, not from `APPROACH_FLARE_FACTOR * stall` as
   `airframe.touchdown_aim` does. Move it only once the door is controlled.
4. **`APPROACH_BEST_LD` 4.2 is the old craft's.** The shuttle flies
   2.9-3.6 to the wheels, float included (LOG5207: 2.93, LOG5233: 3.55,
   landsum `ratio`). It sets `exc`, the gate (via `GATE_FROM_APPROACH`) and
   the cone's `needed`. Make it a measured quantity, but read
   CLAUDE.md's "a measurement that shares a constant's name" first.
5. The glide's 4-8 km long cone arrivals (7/36).
6. Carried over: hac4 in the sea, the approach can't reach its commanded
   alpha (pitch trim), the three roll/yaw-timing flags, `ROLLOUT_STEER_PID`.

## Traps paid this session

- **`pkill -f <pattern>` kills your own shell** when the pattern is in the
  command line (exit 144, the flights kept going). Use the bracket trick
  (`pgrep -af "quickglid[e]"`) and kill by PID.
- **A cone-save test of anything that moves the gate is void** (above).
- Read `ral=`, not `rwy=`. A door 800 m short and one 800 m past read the
  same in `rwy=`.
- `start.sh` worked first time on both restarts this session. Swap reached
  16 GB after one 36-flight batch; restart between batches.

## How to measure

From orbit: `rotfly.sh` as above (~7 min a round of 6). Landing-only work
that doesn't move the gate: `spaceplane/tools/savefly.sh ROUNDS OUT "setsA"
"setsB"` from `qs_s2_hac0-5`. Door/touchdown/stop per flight: the loop in
the journal section (door `h`/`v`/`ral`, first ROLLOUT `ral`, the stop).
