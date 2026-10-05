# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-04 afternoon: the brake costs pitch authority, and the
shuttle rolls over after a level touchdown").

Last written **2026-10-04 ~17:35**. Defaults fingerprint **`6e5178e5`**
(was cdb701a0; it changed only because two off flags were added, with no
behaviour change). Full offline suite OK. Everything is committed. Farm
**stopped**, inhibitor **released**.

## What changed

Nothing was promoted. Two off flags:
- **`AIRBRAKE_HOLD_LIFT`** (e7ed115): while the approach spoiler is out,
  raise alpha until the table lift covers the verified set's dClA (scaled
  to the deploy angle). `hold=` column. Null (below).
- **`WHEEL_WATCH_S`** (aa4a058, fixed a8f4927): log every wheel's
  `grounded`/`broken`/`stress_percentage` on change, for N game-s from
  main-gear contact. An instrument; `WHEEL_WATCH_S=6` to use it.

## The finding: the rollover after a level touchdown is the biggest loss

Over ~140 orbital flights, most vehicles that land level are destroyed in
the next second. Level touchdowns (sink 4-10, 38-48 m/s, pitch 0-6, bank
<5): every intact one stays within 5 deg of bank for 4 s, and about 2/3
roll 11-180 deg within ~1 s, losing the wingtip RCS block / tail fin first.
The contact states are indistinguishable.
- **Not:** the ground spoiler (rot-gspoiler-1004, 4/6 vs 6/8 rolled), the
  wheel brakes (rolls at brk 0, intact at brk 1), touchdown position, or a
  tail strike (pitch 0-6 at contact). Main-wheel friction 1 is *worse*
  than 10 (intact 5/18 vs 12/19).
- **Lead** (rot-wheels2-1004): **5 of ~20 contacts list only one LY-60
  main gear** at contact (LOG5608, 5611, 5620, 5623, 5627). Those roll
  toward the missing side. Is the gear lost before contact, or just
  unlisted by kRPC? Unknown: gear parts have no skin sensor.
  In two-main rollovers (LOG5612, 5619, 5626) one main lifts (`g0`),
  unbroken, while the bank builds 13-20 deg in 1 s.

## The approach brake costs pitch authority

The measured spoiler set is the main elevons, which are also the pitch
control. Deployed at 20 of 25 deg, pitch input saturates and alpha falls
from 11-13 commanded to 3-4. At 10 deg, `aoak` while out is 5.0 vs 1.3,
and it really decelerates. But the speed and sink guards stow it in
~1.3 s every time, so it is out a few seconds per flight, and landings
don't move (rot-pkg-1004: intact on land 4 vs 4). Three nulls. Don't fly a
fourth deploy angle or guard value.

## Measured this session (shuttle, `qs_shuttle2`, from orbit)

| batch | logs | arms | read |
|---|---|---|---|
| rot-holdlift-1004 | 5423-5458 | guard 0.9 / +HOLD_LIFT | hold 1.5 deg; still stows in 1.1 s; null |
| rot-deploy10-1004 | 5459-5494 | HOLD_LIFT+guard 0.9, deploy 20 / 10 | aoak while out 1.3 vs 5.0 |
| rot-pkg-1004 | 5495-5530 | defaults / guard 0.9+HOLD_LIFT+deploy 10 | null on landing |
| rot-gspoiler-1004 | 5531-5566 | defaults / ROLLOUT_GROUND_SPOILER=False | not the spoiler |
| rot-wheels-1004 | 5567-5602 | WATCH / WATCH+MAIN_WHEEL_FRICTION=1 | instrument broken; friction 1 worse |
| rot-wheels2-1004 | 5603-5638 | defaults + WHEEL_WATCH_S=6 | one main gear listed at 5 of ~20 contacts |

## Next, in order

1. **Is a main gear gone before touchdown?** For the one-gear flights, log
   the part count and the wheel list at gear-down and every few seconds
   after (extend `wheel_watch` to start at gear-down, or log
   `len(vessel.parts.wheels)` in telemetry). If the gear goes at
   deployment (gear down at 52-92 m/s, 800 m), check the LY-60's deploy
   speed limit or impact tolerance against those speeds.
2. **The two-main rollover.** One main lifts within 0.5 s of contact while
   the bank grows. Read the roll input (`pin=` 2nd field) and `bnk=` at
   10 Hz around contact (`LOG_INTERVAL_UT` small, from `qs_s2_hac*` or an
   `entrysave.py --alt 300` save). Suspects: the roll loop still active on
   the ground, nosewheel steering at 40 m/s, the craft's gear track (mains
   at x = +-5.6 m).
3. **The brake, by a different method:** the split rudder on the twin tail
   fins (already armed, `airbrake armed: Big-S Spaceplane Tail Fin` pair)
   costs no pitch authority. Or guards referenced to the braked path.
4. **Touchdown aim**: ~half of touchdowns are past the far end;
   `TOUCHDOWN_AIM_M` 2400 is the far threshold. Carried-over item: aim =
   touchdown zone minus predicted float.
5. Carried over: why some glides need 45-70 deg of bank (LOG5308 vs 5297);
   the weave band; the phantom lap (don't fly a fourth value).

## Traps paid this session

- A unit test with a fake object accepted an attribute kRPC doesn't have
  (`Wheel.deflated`) and a whole batch's instrument read "gone". Check new
  kRPC attributes against `dir(krpc.services.spacecenter.<Class>)`.
- `Wheel.stress_percentage` reads 0% on every wheel, every tick: not a
  usable signal on this install.
- Swap reaches 15-17 GB by the end of **every** 36-flight batch, even
  restarted fresh. Restart between batches.
- The arrival scatter (4 of 18 arrivals 6-8 km long in one arm) swamps any
  landing comparison of 18. Compare landings on matched arrivals, or
  per-contact (level touchdowns only), as the scratch scripts did.

## How to measure

From orbit: `./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 6 logs/rot-X.txt .
"qs_shuttle2|" "qs_shuttle2|FLAG=..."` (~40 min for 36 flights). Rollover
analysis: grep each log's `contact:` line (sink/speed/bank/pitch at
contact) and the max `|bnk=|` of the ROLLOUT ticks in the next 4 s;
"level" = sink < 12, |bank| < 10, |pitch| < 10.
