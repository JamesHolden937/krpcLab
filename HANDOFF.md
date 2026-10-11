# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-10 late (~2230).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-10 late".

Defaults fingerprint **`bc9ac544`** (was `101fc0e0`).  Committed and
pushed.  Farm stopped (not started this session); sleep inhibitor
released; no worktrees.  **The user is flying the defaults live and asked
that no tests be flown** -- this session flew nothing.

## The headline

**The deorbit burn never lit on the user's live flight (LOG9598)**: SAS
in Maneuver mode held 4.2-5.2 deg off the rotating-frame retrograde (the
node is built on *orbital* retrograde; their orbit is inclined, 160 km),
outside the 3 deg commit test.  Fixed, and the wait redesigned as asked
("warp more aggressive, preplan the maneuver and preorient"), all default
on and **unflown**:

| flag | what |
|---|---|
| `DEORBIT_SAS_SURFACE` | navball surface mode + SAS Retrograde (Maneuver is the fallback) |
| `DEORBIT_ALIGN_STALL_S` 6 | a hold inside 8 deg that stops improving commits |
| `DEORBIT_PLAN` (+ `_STEP_S` 60, `_RESOLVE_S` 8, `_HORIZON_S` 14000, `_TICK_WALL_S` 1.5, `_LEAD_S` 60, `_GRACE_S` 120, `_DT_S` 10) | scan the future orbit for the window, node there, `warp_to` 60 s before, preorient, live search commits |
| `COAST_WARP_TO` (+ `_STEP_S`, `_MARGIN_S`, `_MIN_S`, `_HORIZON_S`) | `warp_to` the 72 km release instead of polling index 3 |

Read in a log: `deorbit plan: window at UT ...`, `deorbit plan: warping
...`, `SAS has the attitude: surface retrograde mode ...`, `SAS
settled|stalled|timed out after ...`, `coast warp_to: ...`.

## Next, in order

1. **The user's live result.**  If the plan or the surface-mode SAS
   misbehaves, the log lines above say where.  Then fly the four farm
   orbits (24 an arm, defaults v `DEORBIT_PLAN=False`) -- the plan picks
   the first solvable state, as the live trigger does, so arrivals should
   not move; check that, and that the farm's speed-up is real.
2. **Rewrite `docs/spaceplane/howItWorks.pdf`'s DEORBIT and COAST
   sections** once (1) confirms the behaviour -- it still describes the
   polled warp and the Maneuver-mode line-up.
3. Last session's list, unchanged: farm defaults v `ABORT_NORTH` v
   `GROUND_SPOILER_ON_NOSE` (both still off, built, unmeasured on the
   farm); the inc glide energy deficit (9577, 9543: 8-10 km short); two
   low-speed rollouts off the edge (9542, 9545); `constantsAudit.md`;
   `AIRBRAKE_CACHE` promote or delete; glide bank overshoot past 90 deg;
   LOG9375.

## Traps paid this session

- **The farm's saves are all near-equatorial**, so anything that confuses
  orbital and surface frames passes there and fails on an inclined orbit.
  A save on an inclined orbit (~45 deg) belongs in `saves/`.
- `warp_to` is blocking: the panel does not answer during it (Ctrl-C does).

## Traps from before (still true)

- kspSim lands short of the game with TAEM on; don't judge range there.
- kspSim needs `DEORBIT_SAS_ALIGN=False` (stub SAS).
- `pkill -f "<pattern>"` in the same command line kills the calling shell.
- multirot's arm numbers rotate per cycle: group by the log's config line.
- Swap ends a 4-cycle multirot at ~21-23 GB; restart before comparing.
