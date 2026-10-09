# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-09, ~0700-1430.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-09".

Defaults fingerprint **`4590c8ce`**.  Committed and pushed.

## The headline

**The split rudder as the approach's speedbrake, now the default
(`APPROACH_SPLIT_BRAKE`), lands the shuttle 46/66 on the runway (70%)**
over rigoff/inc/high, against 61/135 (45%) for 10-08's best configuration.
Head to head with the brake off, in the same batches: **24/30 v 8/30**
(rot-asb2-1009 15/18 v 6/18, rot-conf-1009 9/12 v 2/12).  Losses fell
from ~1 in 5 to ~1 in 15, and all 48 brake flights of the first three
batches kept 18+ parts.

How it works: the cone hands over 0.6-1.7 km high, ~6 km out, a ~28 deg
path; the approach had no drag device, so it floated long (L/D ~4.4) or
spiralled into the flare at 86-112 m/s.  The fins open to the L/D factor
`distance / (height x APPROACH_BEST_LD)` (re-solved every tick); alpha
still holds speed, so the drag buys path angle.  Gated on **1.45 x stall**,
not on the held speed: the shuttle cannot fly below ~+1 deg of alpha on
final, so its steady glide is ~80 m/s and the 115 m/s target is never held
(gated on a fraction of it, the brake stowed on "slow" within seconds and
the first batch was null, 8/17 v 8/17).

## What else changed today

- **Cleanup** (the user asked): ~20 refuted/null flags deleted with their
  code; the 10-08 best config (offload, canard trim, measured cone L/D)
  promoted to defaults; `spaceplane/CLAUDE.md` rewritten around the
  shuttle (old craft retired); **new root rule: a flag ends as the default
  or deleted, never parked off.**  The cleanup left one NameError in
  `guidance.hac`, caught by a smoke flight and fixed; pyflakes is clean.
- **Flown and deleted today**: `ALPHA_BINS_NEGATIVE` (the vacuum probe's
  rows below zero are junk: lift rising as alpha falls; the trim came out
  -6 and LOG8879 dived), `APPROACH_ALPHA_MIN_DEG` (commanded -2..-5, flown
  +0.3..+4), `APPROACH_SPEED_KI` (a phugoid, 133 -> 48 m/s), the cone split
  brake + flown polar (6/12 v 9/12 stacked), `ROLLOUT_SPLIT_BRAKE` (9/18 v
  10/18), `FLARE_SPLIT_BRAKE` (engaged 14/18, null 11/18 v 12/18).
- Two split-brake bugs fixed: the slew never moved on short ticks; it now
  runs on its own state.

## Where it stands

| | runway | note |
|---|---|---|
| rigoff/inc/high (defaults) | 46/66 | misses spread: long 0.04-1.4 km, a few short, 2-3 partial breakups |
| `qs_shuttle` (single fin, no pair: brake absent) | 5/6 stop on the runway | rigid-attach save, parts lost at touchdown (known) |
| `qs_shuttle2_ecc_rigoff` | 1/6 (2/6 with `HOLDABLE_PRIOR`) | **arrives at the cone 30-45 km long, glide pinned at alpha 40 / bank 70 from the interface** |

## The blocker: the eccentric orbit's entry

At the burn the deorbit window says max drag reaches 2,150-2,165 km with
the gate at 2,205 (rot-ecc-1009, all 12 flights); by the interface the
glide's own solve, pinned at both limits, already reads +17 km long and it
arrives +30-45 km.  **The window's short end and the glide's prediction
disagree about how much drag the shuttle makes.**  `HOLDABLE_PRIOR` +
`HOLDABLE_BY_MACH` roughly halves the arrival error (+7..+15 km on 4/6)
but does not close it.  Next: find which model the window uses for its
short end (`guidance.deorbit_window` / `max_range`) against the glide's
(`solve_glide`), and make them the same -- "the propagator must fly the
law the vehicle flies".  Do not fit `DEORBIT_CENTRE_BIAS_M` (the 51000 trap).

## Next, in order

1. The eccentric orbit (above).
2. `HOLDABLE_PRIOR` + `_BY_MACH`: decide -- promote (it helps ecc, was null
   on the others 21/36 v 21/36) or delete.  It needs `logs/holdprior/`
   regenerated after any attitude/trim change (`holdprior.py --by-mach`).
3. Remaining misses on the good orbits: a few long by <1.4 km (fast flare
   entries, 86-96 m/s -- *not* fixed by braking in the flare), and the
   flare's lateral entry (off-strip by 40-60 m).
4. TAEM before the cone (handovers still 0.6-1.7 km high; the brake now
   absorbs it, but a lower handover is margin).
5. Carried: `TOUCHDOWN_AIM_M`/`APPROACH_AIM_SHIFT_M` per vehicle.

## Traps paid today

- **Smoke-fly before a batch after any refactor**: 3 smoke flights caught
  a NameError, a slew bug and a dive that a 70-minute batch would have
  spent.  The offline tests do not reach `guidance.hac`'s bank cap.
- **`multirot.sh` (scratch) restarted the rotation each farm cycle**: with
  9 arms on 6 instances some arms were never flown.  Rotate the arm list
  by 2 per cycle (fixed in the scratch copy -- promote it to
  `spaceplane/tools/` if reused).
- A command alpha below ~+1 deg on final is not flown -- check `aoa=`
  cmd/actual before building on a commanded angle.
- Batches of one configuration swing 56-83% (10/18, 15/18): pool across
  batches before believing a level; compare only interleaved arms.
