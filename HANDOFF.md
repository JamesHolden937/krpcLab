# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-09, ~0700-1600.  History: `docs/spaceplane/journal.md`,
"Session, 2026-10-09" (morning, afternoon, late afternoon).

Defaults fingerprint **`9abe7afd`**.  Committed and pushed.  Farm stopped.

## The headline

**The split rudder as the approach's speedbrake (`APPROACH_SPLIT_BRAKE`,
default) lands the shuttle 46/66 on the runway (70%)** over
rigoff/inc/high, against 61/135 (45%) for 10-08's best configuration;
head to head with it off, 24/30 v 8/30.  Rigoff alone: 16/22 on today's
defaults.  Losses ~1 in 15.

How it works: the cone hands over 0.6-1.7 km high ~6 km out (a ~28 deg
path); the fins open to the L/D factor `distance / (height x
APPROACH_BEST_LD)` every tick while alpha holds speed.  Gated on 1.45 x
stall (the shuttle cannot fly below ~+1 deg alpha on final; its steady glide
is ~80 m/s, the 115 m/s target is never held).

## What else changed today

- **Cleanup** (the user asked): ~35 flags measured-and-off deleted with
  their code (list in `spaceplane/CLAUDE.md`); the 10-08 best config
  promoted.  **Root rule: a flag ends as the default or deleted.**
  Suite 559 pass, pyflakes clean on spaceplane, smoke 4/6 landed.
- Deleted late: `HOLDABLE_PRIOR` + `_BY_MACH` (rot-prior-1009: ecc 3/12 v
  3/12 pooled, null), `COAST_TRIM` (below), and 11 older parked flags.
- New instrument `COAST_WATCH_S` (0 = off): the burn's stop test re-run
  every N s of COAST, with pointing error, vertical speed and q.
- The burn's exit line now says `range error unmeasured (stopped on the
  solved dv)` instead of printing `+0 m` when the stop test had no answer.
- `spaceplane/tools/multirot.sh`: arms over the farm, restart every 2
  rounds, arm list rotated by 2 per cycle.

## Where it stands

| | runway | note |
|---|---|---|
| rigoff/inc/high (defaults) | 46/66 | long <1.4 km (flare entries 86-96 m/s), off-strip 40-60 m, a few short / partial breakups |
| `qs_shuttle` (single fin, brake absent) | 5/6 stop on the runway | rigid-attach save |
| `qs_shuttle2_ecc_rigoff` | ~1 in 4 | **cone arrival +10..+50 km long** |

## The blocker: the eccentric orbit's coast

All ecc burns are identical (34 m/s).  **The miss is made in the ~90 s
post-burn flip to entry attitude** (full input, pointing error 100-160 deg,
at 69 km, q ~1 Pa): vertical speed ~200 s later orders the arrivals
exactly (-6.8 m/s -> +11 km ... -6.4 -> +50 km, rot-prior-1009).  Same
energy to 0.2 m/s; it is the path angle on a grazing entry.

**On that orbit the burn's stop test cannot measure** (rot-watch-1009,
`COAST_WATCH_S=5`): t2g 1830 s > `DEORBIT_MAX_TIME_TO_GO_S` 1500 straight
after cutoff, so the burn has always ended on the solved dv, open loop;
readable only mid-flip, against an aim of +760 km.  `COAST_TRIM` (RCS
translation closed on that stop test) therefore closed on nothing: its first
prograde pulse took three flights to +69..+78 km.  Deleted.

Next, either:
1. **Correct with a measurement that exists there**: the glide's own solve
   (or the window's corners) propagated from the settled coast state,
   trimmed with RCS along the nose before the air (q < ~20 Pa).  Check
   first, with `COAST_WATCH_S`-style logging, that it reads the same miss
   the glide later sees at the interface.
2. **Or remove the disturbance**: a slower wheel-only flip, the flip done
   before the drag matters, or the burn's last measurement taken after it.
Do not fit `DEORBIT_CENTRE_BIAS_M` (the 51000 trap).

## Next, in order

1. The eccentric orbit (above).
2. Remaining misses on the good orbits: fast flare entries (86-96 m/s
   float long; braking in the flare was null) -- likely TAEM before the
   cone so the handover is not 0.6-1.7 km high; the flare's lateral entry
   (off-strip 40-60 m).
3. Classify the short landings / partial breakups by cause (not done).
4. Carried: `TOUCHDOWN_AIM_M`/`APPROACH_AIM_SHIFT_M` per vehicle.

## Traps paid today

- **Smoke-fly after any refactor**: smoke flights caught a NameError, a
  slew bug and a dive that a 70-minute batch would have spent.
- **"range error +0 m" was a None** on the ecc orbit (fixed in the log).
  Check a stop test actually measured before building on it.
- A batch label (`LOG_INTERVAL_UT=0.5`) means nothing across a day in which
  the defaults moved: group by `defaults=` fingerprint as well.
- A command alpha below ~+1 deg on final is not flown -- check `aoa=`
  cmd/actual.
- Batches of one configuration swing 56-83%: pool before believing a level.
