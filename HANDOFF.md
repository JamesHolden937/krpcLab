# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-10 night (~1800 -> 1955).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-10 night".

Defaults fingerprint **`101fc0e0`**.  Flight behaviour = `b9eba0f3`
(TAEM promoted); `101fc0e0` only adds two off-flags.  Committed and pushed.
Farm stopped, sleep inhibitor released, no worktrees.  `howItWorks.pdf`
rewritten (terminal glide).  **The user is flying the defaults live** --
they stopped the session to play with it.

## The headline

**The terminal glide now solves on cone-entry energy, by default.**
`GLIDE_TAEM_ENERGY` + `GLIDE_TAEM_ALPHA` at `GLIDE_TAEM_MACH` 4.5, with
last session's two defects fixed: in surplus the alpha search may only
raise alpha (no ratchet through the flown-alpha cap), and out of reach of
the entry circle the miss is the shorter of energy and range.

Farm **taemfix-1010** (24 an arm, four orbits, LOG9536-9583):

| | old defaults | TAEM (now default) |
|---|---|---|
| on the runway (\|along\| <= 1.2 km) | 20/24 | **23/24** |
| high orbit on the runway | 3/6 | **6/6** |
| lost | 1 (9560, +2.2 km, water) | 0 |
| cone entry | 12-17.6 km, 210-330 m/s | 12-15 km, 195-275 m/s |

The goal (in spaceplane/CLAUDE.md, the user's): **>= 95% on the runway, 0
lost, worst miss < 3 km, four orbits, two airframes; every divert north**
(the space center is south of the runway).

## Built, off, not yet flown on the farm

- **`ABORT_NORTH`** (+ `ABORT_NORTH_OFFSET_M` 150, `ABORT_SHORT_M` 30):
  on final, once height over best glide to the threshold < -30 m; on the
  rollout, once the stop at the achieved deceleration passes the far end
  -> the tracked line moves 150 m north (`guidance.north_side`, off the
  pole).  kspSim sim-north-1010 (LOG9584-9591): 8/8 diverted, stopped
  158-197 m north by KSP latitude -- but all sim flights were short, so
  the **overrun trigger never fired and false triggers are unmeasured**.
  The farm batch north-1010 was stopped in round 0: nothing measured.
- **`GROUND_SPOILER_ON_NOSE`** (+ `_NOSE_TIMEOUT_S` 4): the ground spoiler
  waits for the nose wheel.  3 of taemfix-1010's 5 part-loss landings
  (9567, 9578, 9580) were a bounce: touchdown 68-76 m/s, alpha 2-4 -> 11-15
  in the spoiler's tick against full nose-down elevon, 17-22 m up, second
  contact at 10-15 m/s sink.  Bounce scales with airspeed, not sink.

## Next, in order

1. **Farm: defaults v `ABORT_NORTH` v `GROUND_SPOILER_ON_NOSE`**, four
   orbits, 24 an arm (restart the farm first).  For ABORT_NORTH read the
   `divert north` events: any on a flight that would have stopped on the
   runway is a false trigger; the DOWN line's "m north/south of the runway
   line" says where the misses went.  The overrun trigger may need a
   forced test (`APPROACH_AIM_SHIFT_M=-1500`, a few flights).  For the
   spoiler: bounce height (max h in ROLLOUT) against touchdown speed.
   Promote or delete each.
2. **The inc glide energy deficit**: one inc flight per 24 lands 8-10 km
   short in both arms (9577, 9543): 12 km reached 10-12 km before the
   circle; the TAEM solve saturates at alpha ~20, bank ~0 while the short
   grows from -1 km (Mach 4) to -10 km (Mach 0.9).  Is the stretch alpha
   right (best glide transonic), or is the deficit set at the burn?
3. Two rollouts ran off the edge at 47-57 m/s without bouncing (9542,
   9545): 28-49 m off the centreline.
4. `docs/spaceplane/constantsAudit.md` top-down; `AIRBRAKE_CACHE` (never
   flown) promote or delete.
5. Glide bank overshoot past 90 deg (roll loop); kspSim shows bank -70
   commanded, -90 flown.
6. LOG9375 (old): exited the cone needing 6.6 km more height.

## Traps paid this session

- **kspSim lands short of the game now**: with TAEM on, sim flights end
  1.5-5.7 km short where the farm's stop ~+0.8 km.  Don't judge range in
  the sim; it showed the ratchet gone and the divert's direction.
- kspSim still needs `DEORBIT_SAS_ALIGN=False` (stub SAS).
- `pkill -f "<pattern>"` with the pattern in the same command line kills
  the calling shell (exit 144).  Kill by PID from `ps -eo pid,args | awk
  '/[p]attern/'`.
- multirot's arm numbers rotate per cycle: group by the log's own config
  line (`armsum.py --by`), never by the `armN` label.
- Swap ends a 4-cycle multirot at ~21-23 GB; restart before comparing.
