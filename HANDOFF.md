# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-01 night: the roll-rate estimator's slip pulldown").

Last written **2026-10-02 ~05:00**, spaceplane. Defaults fingerprint
**`925b4729`** (one default changed this session, below; one new flag, off).
Full offline suite OK (854 before the flag; spaceplane 683 after).
Committed. Farm **stopped**, inhibitor **released**, no sims running.

## Where it stands

Goal ("lands mostly reliably") **not reached**, but the first intact
shuttle landings since chain6 came this session. Best measured stack:
**chain6 + `RCS_PITCH_OFF_IN_GLIDE`** on the new defaults (exact string:
`arm0` of `logs/rot-revhold-1002.txt`). Over 16 flights with the new default:
final |along| <= 5 km 11/16, intact 3/16 (LOG4498 31/31 at -1036 m).

**The entry scatter is the second bank reversal** (Mach 4.5-5, q 2000-3000):
the flown bank lags, sideslip reaches 13-23 deg, alpha collapses 15-30 deg
below command, the ceiling ratchets down and the glide runs long -- or, at
q ~5000, the bank pins at the 70 cap and departs laterally (LOG4521). Flights
whose reversals stay under ~12 deg of slip reach the cone within +-2 km.

## Changed this session

| what | measured | status |
|---|---|---|
| **`BANK_RATE_SLIP_TOL_DEG` 5 -> 0 (default)** | the pulldown fired mostly in COAST at q 0-150 (vacuum "sideslip"), cutting the shuttle's bank slew 7-10 -> 1.3-2.8 deg/s, so the second reversal crawled into q ~3000. Off, 16 v 16 (`rot-sliptol-1002`, `rot-sliptol2-1002`): within 5 km along 11/16 vs 6/16, intact 3 vs 0. Null on `qs_plane` (limit pinned at 30 either way, `rot-sliptol-plane-1002`) | **default**, `8198273` |
| `RCS_HOLD_MID_REVERSAL` | holds the yaw valve from the first tick of a reversal. Connected, but with the pulldown off the valve already opens within a tick: `rot-revhold-1002` null (collapse 4/8 vs 2/8) | built, **off** |
| `HOLDABLE_MEAN` | re-flown whole (`rot-holdmean-1002`, 8 v 8): null on arrival and departures | off |

Over 32 pulldown-off flights the slew limit going into the second reversal
correlates only weakly with its slip (r -0.30): rate is not the lever.

## Next, in order

1. **Change the method on the reversal** (three mechanisms tried: pulldown
   fixed, valve hold null, rate weak). The airframe out-rolls its own yaw:
   at q 2100 surfaces give roll 617 / yaw 158 kN m, yaw inertia 22x roll's
   (failure 99). Candidate: **unload alpha through the reversal** (body
   roll's sideslip scales with sin alpha) -- a guidance change the
   propagator must fly too; `SOLVE_HOLD_THROUGH_REVERSAL_DEG` and failure 16
   are the history of alpha-in-reversal. Or reverse only below a q the
   measured lateral authority can serve. Also worth asking the user about a
   craft fix (yaw authority: bigger/more fins), as with the pitch trim.
2. The cone arrival is now mostly **long and high** (+4..+29 km, h 15-23
   km) on this stack, even without a departure -- read why once the
   reversal is tamed (pitch-off flies 2-5 deg under command at Mach 2-5).
3. Why the first reversal (Mach 6.6, q ~250) slips ~10 deg in some flights
   and ~2 in others regardless of rate: unexplained.
4. The landing chain on good arrivals (unchanged): lateral capture with bank
   still on (LOG4385), flare entry 66-95 m/s.

## Traps paid this session

- **Swap** still grows ~4-5 GB per 80-min batch; restart the farm before
  every batch (done each time this session, swap back to ~1-3 GB).
- Scratch `rev.py`/`slip.py`/`dep.py` open `LOG<n>` relative to `logs/`;
  `replay.py`/`replayq.py` replay `RollRate` over logged ticks from the root.
  `aoa=` is **cmd/actual** -- I read it backwards once.
- `syncSaves.sh check` reports ksp5 missing saves; only 0-3 are the farm.
