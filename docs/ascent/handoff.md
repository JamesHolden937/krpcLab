# ascent HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-18 evening: ascent -- re-flown on the corrected budget; new fin2 separation

asc-1118 (`logs/rot-asc-1118.txt`, 6 x `qs_stack_fin`, `HANDOFF=booster,SEP_SAVE=fin2sep`): separations 18.5-19.0 km
(one 20.2), 866-874/440-456 m/s, 166-169 t propellant, predicted 21.5-23.4 t at touchdown (reserve ~21); boosters
landed 2-96 m (LOG17921-17926), finishing at 21.5-23.4 t over dry -- the budget's end state matches. budgetcheck: coast
mass flown ~3 t above the budget's (boostback pessimistic). ksp3's (LOG17918) is the new `qs_finbooster2`
(saves/README.md). Open: the boostback's ~3 t pessimism; landing-burn Isp (295 budgeted, ~307 flown).

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

Older section (2026-10-19 night: all three stacks reach 80 x 80, the shuttle keeps its wings) -- the last word on
the stacks: journal.md, "Former handoff section (moved 2026-11-18)" at its top.
