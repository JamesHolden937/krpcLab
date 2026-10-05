# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-04 night / 10-05: the wings come off because of rigid
attachment").

Last written **2026-10-05 ~14:50** (machine clock). Defaults fingerprint
**`9e64c23f`**: `FLARE_ALIGN_ALT_M` 140 -> 30, `HAC_EXIT_PAST_DEG` 0 -> 25 and `APPROACH_AIM_SHIFT_M`
(new) 0 -> 1000 promoted this session. Offline
spaceplane suite OK. Everything committed. Farm **up** (0-5), ksp6 is the
camera instance (1280x720, its own settings.cfg).

## The finding: rigid attachment breaks the wings at touchdown

With `rigidAttachment = False` on the two `wingShuttleDelta` parts
(`saves/qs_s2_rigoff0-5`, two lines per save), **22 of 24 touchdowns kept
both wings** (14/14 gentle, 8/10 hard), against ~8/24 stock, interleaved in
two batches (sav-rigoff-1005 LOG5932-5967, sav-rigconf-1005 LOG5968-6003).
Off on every wing part (`qs_s2_rigwing`) is no better. **The user's real
craft still has it on** -- they were told to turn it off on the deltas in
the SPH (FullAutoStrut's `RigidAttachment = True` re-applies it to new
parts). The reference saves are unchanged; fly landing work on
`SAVE=qs_s2_rigoff` until the user's craft is updated.

Refuted on the way (journal has the logs): the brake at contact, contact
attitude/sink within the gentle band (random), autostrut on the wings,
suspension spring/damper, gear-into-runway and gear-into-elevon
collisions (artefacts: impulse 0 / after the break), gear on the fuselage
(wings still go -- the clue).

## Instruments added (all off by default / outside the flight code)

- `GROUND_WATCH_S` (e.g. 3): lowest point of every part above the runway
  near contact, rigid geometry + flex. `contactsum.py` summarises.
- **CollisionSpy** plugin, `testInstances/collisionSpySrc`, installed in
  every instance's GameData: collisions, joint breaks, and per physics step
  near the ground the wing joints' force/torque vs break limits (1760).
  `spaceplane/tools/jointsum.py` (its `--batch` labelling is unreliable
  when a flight has no touchdown -- align by UT instead).
- `testInstances/shoot.py N --watch --cam-mode locked --cam-heading 90`:
  screenshot burst through touchdown via `SpaceCenter.screenshot` into
  `logs/shots/LOG<n>/` (F1 over XTEST does not reach the game). Run on ksp6
  alone, at time scale 1.
- `testInstances/gearProbe.py N SAVE`: geometry with the gear down.
- `savefly.sh` arms may carry `SAVE=<prefix>`.

## Flags added, all off

- `GEAR_GEOMETRY_DEPLOYED`: wheel clearance and tail-strike angle measured
  gear-down (3.72 m / 25 deg; gear-up they read 1.82 / 11.0, so the flare
  flew 1.9 m of phantom height and capped pitch at 8.8). Wired (LOG5747:
  contact h 0.1 not 2.2). Unmeasured on landing quality -- the wing loss
  swamped it. **Re-fly on rigoff saves.**
- `ROLLOUT_BRAKE_DELAY_S`: null on wing loss.
- `MAIN_GEAR_SPRING` / `MAIN_GEAR_DAMPER`: null.

## Promoted 2026-10-05 midday (journal, "the touchdown point")

- `FLARE_ALIGN_ALT_M=30`: touchdown cross-track halved (median 27 -> 13-20).
- `APPROACH_AIM_SHIFT_M=1000`: the approach aims 1 km nearer; cone/gate
  keep `TOUCHDOWN_AIM_M`; S-turn stop measured from the unshifted aim.
  Rigoff cone saves on the runway **13/15 vs 5/15** (hac4 excluded).
  Regression: old craft 4/4 on runway; orbit 2/8 vs 2/8.

**Now the blocker is the cone from orbit**: ~1 in 8 orbital shuttle
flights lands on the runway. Two mechanisms (journal): the derived exit
allowance hands over up to a lap's height (~8 km) of surplus; and the cone
prices its exit via a gate 6 km out while a 9 km circle has the vehicle
beside the field, so "out of height" exits read +500 m to the approach.
Make the cone's exit ask `guidance.approach` for the excess (one model).
Since then (journal, "the cone from orbit"): `HAC_EXIT_PAST_DEG` 0 -> 25
promoted (out-of-height exits 0/12 vs 4/12). Fixed exit allowance and an
8 km radius cap both null/worse. **Next: the cone arrives at the rollout
300-3500 m high on almost every orbital flight with laps=0 -- measure the
planned vs flown turning L/D (`conesum.py --settled`) on rot-*-1005 logs.**

## Where the landing stood before those (rigoff saves, 30 flights)

On the runway and intact: hac0 5/5, hac2 3/5, hac5 2/5, hac1 0/5, hac3
0/5, hac4 0/5 -- **10/30**. What misses now is the arrival, not the gear:
- hac1: the approach's S-turns end ~1.6 km out and the centreline capture
  overshoots (+321 -> -378 m); flare hands over 200-260 m off and stops
  75-115 m to the side. The flare has ~1.5 s of lateral authority.
- hac3/hac5: touch down near or past the far end (`TOUCHDOWN_AIM_M` is the
  far threshold, 2400; read its comment before moving it).
- hac4: cone surplus 3.5 km (lands 6-8 km long) or stalls on final.
- One flare departure (LOG6003): cone out of height, 127 deg bank at 49 m.

## Next, in order

1. Batch on `SAVE=qs_s2_rigoff`: defaults / `GEAR_GEOMETRY_DEPLOYED` /
   `TOUCHDOWN_AIM_DERIVED` (exists, off; the cone exits closer now).
2. hac1's lateral capture: end the S-turns by the time the capture needs
   (roll lag), or capture before the S-turn's last swing.
3. hac4's cone energy.

## Traps paid this session

- Small n flipped three "findings" tonight (brake 5/5, the spring, the
  gear-collision story). Interleave arms, and read 18+ per arm.
- CollisionSpy ENTER lines with `impulse=0.0` are the wheel touching, not
  a part strike; collisions after a JOINTBREAK are debris.
- KSP overrides `-screen-width` with settings.cfg; ksp6's is 1280x720.
- `date` on this machine reads UTC-ish morning times; KSP.log too.
