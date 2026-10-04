# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-03/04 night: the glide's long arrivals are bank-coupled
alpha, and the phantom batch").

Last written **2026-10-04 ~02:20**. Defaults fingerprint **`cdb701a0`**
(was 6e854d9f; 83592105 in between only because two off flags were
added). Full offline suite OK (877). Everything is committed; this file is
in the last commit. Farm **stopped**, inhibitor **released**.

## What changed

- **Promoted `ALPHA_TRIM_IN_GLIDE`** (1c088ae). The existing alpha trim
  loop (`Autopilot.alpha_trim_loop`) now runs in GLIDE too, bounded by
  `ALPHA_TRIM_GLIDE_MAX_DEG` 10 (a safety bound, not a fit). On leaving the
  glide it is clamped back to the landing's +4. Paired from orbit, 36 a
  side (rot-glidetrim-1003 + rot-glidetrim2-1004): intact on land 9 vs 5,
  arrivals at the cone >1.5 km long 15 vs 21, vehicles lost 5 vs 4.
- **Added, off: `HAC_WRAP_BEFORE_GATE`** (4e5af16): the version of
  `HAC_PAST_BEFORE_GATE_DEG` with no tolerance angle (before the gate, any
  wrap past the rollout costs the run to the gate). It has a test. Flown
  from the cone saves: a wash (below).

## The finding: the glide's long arrivals are bank-coupled alpha

The deorbit is identical on every flight. The cone handover still splits
into two groups: `long` ~+500 at h ~15 km, or +3..+15 km at h ~19-21 km
(about 40-55% of flights from orbit). Loop rates are clean. The split is set
below Mach 4 by **bank**:
- The long flights bank 42-60 deg between Mach 1.2 and 4, against 20-30
  for the good ones.
- At fixed q, the alpha shortfall is ~4-5 deg below 35 deg of bank and ~10
  above 50 (630 ticks pooled).
- Pitch input on the long flights is 0.5-0.7, unsaturated. So it's the
  controller, not the airframe. This amends the 2026-10-01 "pitch trim /
  CG" note.

The trim raises achieved alpha at high bank from 26-33 to 33-35 and then
saturates the pitch input. **~34-35 deg is the real ceiling below Mach 4**;
`GLIDE_ALPHA_MAX_DEG` is 40.

**Still open: why some flights need 45-70 deg of bank.** Refuted on the
same data: late reversals. Long arrivals reverse *fewer* times in the last
150 km (5-6 vs 7-13), with the same ~3.2 km peak cross. The range solve
banks hard because energy is surplus. The long flights already run 1-3 deg
short at Mach 4-6 (LOG5308: a reversal at Mach 4.8 drops achieved alpha to
31). `long=` reads +500 until ~30 km out, then climbs as bank saturates at
70. The learned holdable ceiling does know the shortfall (30.5 vs 34.4 at
3.2 kPa), so the prediction isn't simply ignoring it.

## Measured this session (all shuttle, `qs_shuttle2`)

| batch | logs | arms | read |
|---|---|---|---|
| rot-phantom-1003 | 5255-5278 | defaults / WRAP_TO_GATE+PAST60 | swamped by 9/24 high arrivals; `turn= 3xx` also counts real laps past the gate |
| rot-glidetrim-1003 | 5279-5314 | defaults / ALPHA_TRIM_IN_GLIDE | long tail +6.4 -> +2.6 km mean; landing even |
| save-wrap-1004 | 5315-5350 | defaults / HAC_WRAP_BEFORE_GATE (cone saves) | wash: runway 10 vs 9, intact 5 vs 5; per-save sign flips |
| rot-brakeguard-1004 | 5351-5386 | GLIDETRIM / +AIRBRAKE_SPEED_GUARD=0.9 | brake now relays against SINK_TRACK; arm 0 landed 6 intact |
| rot-glidetrim2-1004 | 5387-5422 | defaults / ALPHA_TRIM_IN_GLIDE | intact on land 3 vs 6, runway 4 vs 8, lost 4 vs 2 |

**The approach brake has never braked.** From orbit it deploys on "S-turn
saturated 100% with +1.3..+4.8 km left", and the speed guard stows it the
next tick ("speed 108 below target 108"). `APPROACH_SPEED_PATH` holds speed
*at* the target. With the guard at 0.9 it relays against
`AIRBRAKE_SINK_TRACK` instead: out, sink +5-6 m/s over wanted in ~1.5 s,
in, out again 2 s later (LOG5361, eight cycles). The measured set is a
**lift spoiler**, so it buys sink, not drag.

## Next, in order

1. **Make the spoiler a speedbrake.** While it's out, raise alpha to hold
   lift, so the lost lift becomes induced drag at the held speed. That's
   the dissipation the approach is missing: hand-overs 1-6 km over profile
   are where the losses are. Candidates: `LIFT_LOOP` (built, off) acting
   while `ab=out`, or a feed-forward alpha offset from the measured set's
   ClA (-33). Then the sink-track relay should stop on its own. Fly it from
   orbit, because the cone saves have no brake (measured in vacuum).
2. **Why some glides need 45-70 deg of bank** (above). Start at the Mach
   4-6 reversal: LOG5308 vs 5297. Read `aoa=`, `bank=`, `long=` through
   each reversal.
3. **The approach's weave band** (`APPROACH_SCURVE_CROSS_M` 300). The
   binding caps are `sqrt(2 lateral room)` and the lag cap
   `0.5 lateral (cross_time - lag)`, both on `APPROACH_CAPTURE_MARGIN` 0.15
   (lateral ~1.2 m/s^2): ~12-16 deg of track. Even 45 deg over the 4.6 km it
   may weave spends only ~600 m of LOG5233's +1139, so this is secondary to
   item 1.
4. The phantom: three nulls. Don't fly a fourth value. If it comes back,
   `HAC_WRAP_BEFORE_GATE` misses the inside-the-circle branch (LOG5321).
5. Carried over: aim = touchdown zone minus predicted float;
   `APPROACH_BEST_LD` 4.2 is the old craft's; hac4 in the sea.

## Traps paid this session

- A batch's `turn= 3xx` count includes real laps flown past the gate after a
  high arrival. Check `ral=` against the gate before calling it a phantom.
- Adding an off flag changes the defaults fingerprint (6e854d9f ->
  83592105) with no behaviour change. Note it, so logs aren't misfiled.
- Promoting a GLIDE-phase flag broke tests that call `aim` in GLIDE with a
  minimal snapshot (no `ut`). They are pinned off now.

## How to measure

From orbit: `./spaceplane/tools/rotfly.sh "0 1 2 3 4 5" 6 logs/rot-X.txt .
"qs_shuttle2|" "qs_shuttle2|FLAG=..."` (~7.5 min a round). Landing-only
work that neither moves the gate nor needs the brake: `savefly.sh` from
`qs_s2_hac0-5`. The per-flight table this session used (glide `long=`,
exit `h (needed)`, parts, along) is in the journal section's commands.
Read glide alpha vs bank with `aoa=cmd/act`, `bank=`, `pin=` on GLIDE lines
between Mach 1.2 and 4.
