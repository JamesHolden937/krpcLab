# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-09 night (~2200 -> 2026-10-10 0020).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-09 night".

Defaults fingerprint **`ca5fef06`** (was `b9f653fc`).  Committed and
pushed.  Farm stopped, sleep inhibitor released.  No worktrees open.
`howItWorks.pdf` rewritten for the brake order.

## The headline

1. **Brakes before weaving is the default.**  Split rudder first, spoiler
   second (yields to roll), weave / S-turn last, in the cone and on final.
   rot-v2-1010 (24 an arm, interleaved, four orbits, fresh farm per
   cycle): **22/24 on the strip and intact v 16/24**, long 1 v 5, parts
   kept 24 v 22, ecc 8/8 v 4/9.
2. **`GLIDE_SPLIT_BRAKE` deleted**: 4 of 6 departed within ~15 s of the
   fins deploying at Mach 2.2 (rot-smoke-v2-1010, LOG9273-9278).
3. **Every glide ends in a lateral limit cycle** (found in all 36 flights
   of rot-ecc2-1009): ~6 bank reversals in the last 60 s, flown bank
   85-180 deg, slip 10-25 deg.  The azimuth deadband in
   `guidance._bank_sign` is at its 0.5 deg floor near the field against a
   ~3 s roll lag.  It has not lost a flight on the defaults, but it is
   the closest thing to a departure the glide flies.

## Flags (all off) and what they measured

| flag | what | measured |
|---|---|---|
| `AZIMUTH_FLOOR_FROM_TURN` | azimuth band >= g tan\|bank\|/v x roll damper tp (capped 12 deg) | smoke rot-smoke-azf-1010 on `ca5fef06`: 6/6 intact, 5/6 on the strip; reversals in the last 60 s mean 3.3 (2-5) v ~6; flown bank still peaks 92-111. **Uncompared** |
| `APPROACH_CAPTURE_TAU_SHARE` | lateral capture as an exponential instead of constant rate to the wheels | unflown; try 0.35 |

Deleted this session: `GLIDE_SPLIT_BRAKE` (departures),
`GLIDE_SETTLE_ON_FLOWN` (null on its mechanism: reversals 6 v 6, the
flown bank overshoots past 30 deg in ~2 s so the guard never held),
`hac_flap_brake`, `command_airbrake`, `HAC_FLAP_ARREST_G` (replaced).

Probes: split rudder at 38 deg, +32% drag at Mach 0.8, +27% at 1.0, +16%
at 1.5, +12% at 2, +3% at 4.  Hypersonic pitch-neutral lift dump 4-5% of
lift (`spoilerprobe.py`): not built.

## Next, in order

0. **First thing (the user, 2026-10-10): replace the cone-entry
   constants with derived tests.**  `run_glide` enters HAC on
   `mach <= HAC_ENTRY_MACH` (1.5, hand-set) and `height <= HAC_ALT_M or
   distance <= HAC_ENTRY_DIST_M` (3 km).  Both are arbitrary:
   - Mach 1.5 stands in for a turn radius and contradicts the cone's own
     limit: the holdable circle at Mach 1.5 is ~20 km
     (`guidance.hac_hold_radius`, 45 deg bank) against
     `HAC_RADIUS_MAX_M` 16 km -- the LOG2756 failure shape.  The derived
     test it was meant to defer to (`guidance.hac_enterable`,
     `HAC_ENTRY_DERIVED`) no longer exists; comments in `run_glide`
     (~autopilot.py:4357) and `hac_radius` (~guidance.py:1489) still cite
     it -- fix them.  It decides nothing today (arrivals are Mach 0.8-0.9)
     but would admit an unflyable circle on a fast arrival.
     Replace with: enterable when `hac_hold_radius(speed) <=` the radius
     the plan wants (at most `HAC_RADIUS_MAX_M`).
   - The 3 km distance test is what actually fires, while the glide dives
     at ~37 deg, so entry is at 14-18 km instead of the 12 km aimed for.
     Replace with: enter when the glide's own prediction reaches the gate
     height (or the vehicle is at/past the high gate), keeping a backstop
     only for a genuinely flat arrival.
   Build behind one flag, smoke, then fly against the defaults (24 an arm,
   four orbits); judge on entry-height scatter (conesum `h` at entry), cone
   exit surplus and landings.  Then item 1.
1. **Fly `AZIMUTH_FLOOR_FROM_TURN` against the defaults**, 24 an arm:
   `spaceplane/tools/multirot.sh azf-1010 4` with 12 arm strings
   (defaults / `AZIMUTH_FLOOR_FROM_TURN=True` on rigoff, inc, high, ecc,
   rigoff, ecc, each with `LOG_INTERVAL_UT=0.5`).  ~65 min.  Judge on
   landings *and* the glide-end reversal count / peak flown bank / slip
   (journal describes the count: bank-sign changes with |bank|>5 in the
   last 60 s before GLIDE -> HAC).  Promote or delete.
2. If the overshoot past 90 deg survives the floor, it is the roll loop
   itself (command slew 30 deg/s against a ~3 s response) -- look at the
   glide's bank slew and roll damper, not the sign law.
3. **The cone's lap gap.**  laps=0 in every flight: a lap is ~22 km of
   path, so a surplus between ~23 and ~45 km cannot be planned and is left
   to the brakes.  Entry fires 3 km from the high gate (`HAC_ENTRY_DIST_M`)
   in a ~37 deg dive, at 13.8-18 km instead of 12; exit height follows
   entry at ~0.65.  Options: enter on the predicted gate height, or a
   partial lap / wider circle range.
4. `APPROACH_CAPTURE_TAU_SHARE=0.35` (off-strip).

## Traps paid this session

- **A background waiter can be killed for memory** with six instances up
  (~3 GB available); the nohup'd batch survives it.  Re-read the batch's
  rot file rather than re-launching.
- `armsum.py --by FLAG` printed along in km and was unreadable for a
  package comparison; a per-arm table off the rot files (save, along,
  across, parts, and the log's `config:` line for the arm) was what
  decided it.
- Swap reaches 20-21 GB per two-round cycle, as before; multirot restarts
  the farm per cycle.
- `git worktree add .wt-x` needs `-b name` here (the bare form fails on
  the branch name).
- Carried over: check a save's periapsis against the 70 km atmosphere
  before trusting it; "could not pause after the load" hits ~1 in 15
  flights -- exclude them; diff after every scripted edit.
