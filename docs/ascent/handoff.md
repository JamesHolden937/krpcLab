# ascent HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 afternoon: ascent -- the separation budget measured against 30 fin2 flights; its error was the fall's drag

State: ascent defaults unchanged; boosterland defaults fingerprint unchanged (080e48b5). Committed on main, pushed.
Farm down. howItWorks.pdf (ascent, boosterland) not regenerated (stale since 11-17). Journal: `docs/ascent/journal.md`
2026-11-18. kRPC 0.8 MechJeb split API (652811c) and PersistentThrust stripped from the farm (11-14) still stand.

**How accurate the budget is now** (`ascent/tools/budgetcheck.py LOG...`: `budget.recovery` from each boosterland
log's separation, on the dragcache curve and on the log's own fall, against the flight):
- Boostback: coast mass 160.2-160.3 t against 161.1-161.5 flown (~1 t pessimistic). The "~3 t optimistic
  boostback" of 11-16/17 was wrong.
- Fall: on the old cache it lit at 729 m/s every flight (flown 735-786) and charged the landing 36.75 t, ~3-4 t
  low. Cause: `record_drag` saved the env's curve at shutdown, after the gear-down burn's and the wreck's probes had
  walked it to 2x (9.9 m^2 at M2.4 against the fall's 4-5). **Fixed (843438e):** boosterland records the curve as
  it stood at the landing burn's ignition. fin2rec-1118 (6 flights, defaults) rewrote the cache (5.3 at M2.4): the
  budget now lights at 763 m/s (flown 742-764) and charges 39.1 t, within ~1 t of the flights' own fall.
- Not modelled: the coast's RCS spends 0.65-5.2 t of LFO (LOG17812: ~50 s saturated, err 5-10 deg): a
  boosterland coast-attitude limit cycle, to fix there, not to budget.
- On fin2's separation the budget now says it lands **1.5 t short before the coast's RCS** (was "0.85 t spare").

**Open, in order:**
1. **NEXT PRIORITY (the user, 2026-11-18): the coast's RCS limit cycle (boosterland)** -- 0.65-5.2 t a flight on
   one save; the largest unpredicted term now. Details in the boosterland section's open item 1.
2. Re-fly the ascent for a new fin2 separation on the corrected budget (deferred by the user, 2026-11-18).
3. The landing burn's Isp is budgeted at the pad's (295); the flight reads ~307 at ignition. Small, check.

**Traps:** `dragcache` holds one entry per booster key; `budgetcheck` picks the entry whose note is one of the given
logs (`--key` otherwise). The offline suite's 2 kspSim wire-codec errors (krpc type code 203) are pre-existing.
A kRPC object takes any `setattr` silently: grep assignments after an API change, not only reads.

The ascent side of 2026-11-13 (separation budget, `ascent/budget.fall`, Heavy Shuttle stack): docs/boosterland/handoff.md, section 2026-11-13.

## 2026-10-19 night: ascent -- all three stacks reach 80 x 80; the shuttle keeps its wings (fingerprint `5a3c681f`)

**2026-10-21 (farm-wide, see the spaceplane section):** kRPC 0.7.0;
ascent's staging on `Vessel.stage_at`/`decouple_stage_at` (0.7 removed
`Parts.in_stage`/`in_decouple_stage`/`resources_in_decouple_stage`).  One
smoke flight (qs_stack): 81 x 79 km.  KRPC.MechJeb 0.8.1 still loads under
0.7.


All committed and pushed. Farm stopped. The user stopped the session and
will fly the current defaults live themselves (no re-fly asked for).

**Where it stands (defaults):** classic MechJeb, turn shape solved at
liftoff, booster dropped by budget, upper stage followed to orbit.
- qs_stack (shuttle, gears now autostrut Grandparent): 12/12 to 80 x 80,
  **64.48 t sd ~0.1** in orbit (gs1-1019, LOG13070-13081), 0 joint breaks.
  That leaves ~1.0 t LFO = **~51 m/s** (Rhino 340 s) -- the 34 t ore payload
  eats the margin.
- qs_stack_small 12/12 10.17 t; qs_stack_stretch 12/12 10.11 t
  (fix1-1019, LOG13034-13069; Jebediah now in the pod).

**This session changed:**
1. Saves: small/stretch had an uncrewed pod -- no control source after
   separation (all 24 lost). qs_stack gears Heaviest -> Grandparent: the
   struts ran to the booster tank and tore the delta wings off at
   separation (~11/12). Craft file fixed too. **The user's live VAB craft
   still has Heaviest on the three gears** -- told them.
2. Liftoff warnings (no flag, logging only): `upper stage: ... control X`
   / `WARNING upper stage has no control source`; `WARNING autostruts
   cross the booster's decoupler`.
3. `check_settings`: MechJeb ascent settings re-read after separation and
   re-applied if lost (they were not lost in any flight; a guard).
4. turnshape: downrange along the ground (was inertial); the turn starts
   at MechJeb's start speed or height, curve measured from there
   (`Path.start_speed`). **Not flown yet** -- model2-1019 was stopped
   part-way; its partial logs are not a result.
5. `turn shape track:` log line + `ascent/tools/shapetrack.py` (model vs
   telemetry). Before item 4 the model ran 1.4-3.6 km short in downrange
   at separation on all three stacks.
6. Hot staging tried and deleted (did not prevent the wing loss).

**Next, in order:**
1. Whatever the user's live flights report.
2. One batch of the defaults on all three stacks (srot LAPS=2): does the
   turn-start model move the solved shape, and do predicted mass and
   shapetrack agree with the flights?
3. Model gaps still open: drag at zero AoA, booster throttle-down and coast
   drag left out; `RESERVE_FRACTION` per booster; `SEP_ACCEL_RATIO`.

**Traps paid:** `pgrep -f`/`pkill -f` with a pattern that also appears in
your own command line kills the tool's shell (again) -- kill by PID from a
separate listing. The farm restart overwrites every KSP.log: read the
CollisionSpy / explosion lines before restarting. A farm launch with every
kerbal Missing produces an uncrewed pod silently. Swap read 10-15 GB right
after each farm restart tonight (other memory pressure on the machine).
