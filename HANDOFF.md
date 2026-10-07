# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-06 (evening, ~1550-2015).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-06 evening".

Defaults fingerprint **`42d938f0`** (= the day's `63ef8321` + two new
off flags; no default changed this session).  Everything committed and
pushed.  Farm **stopped**, sims not used, sleep inhibitor released.

## The finding (the user asked: is something fundamental missing?)

**Reliability is set by the glide's arrival, not the cone.**  rot-lapstack-1006
(48 flights): nearly every flight reaching the cone within ~1 km of profile
landed intact on the runway; of ~20 arriving 4-14 km long, 2 did.

**The glide is planned at an alpha the vehicle does not hold.**  Per altitude
band, from the logs' `cda=` (model at the commanded alpha), `act=`
(measured) and `mdl=` (model at the achieved alpha): at 38-32 km
measured/commanded drag is **0.59-0.80 on every long flight, 0.83-1.07 on
every on-profile one**; measured/achieved is 0.96-1.04 on all.  The table is
right; the propagator's alpha is wrong.  `Holdable` knows a ceiling only once
the vehicle saturates, and tracking 40-44 early leaves denser air unlimited.
(Scratch script for the table: re-derive with the three columns; worth
turning into a tool, see Next 1.)

## Flags added (off) and what they measured

- `HOLDABLE_PRIOR` (65c3fc3, 592ea75): seeds Holdable's bins from the craft's
  own logs (`spaceplane/tools/holdprior.py` -> `logs/holdprior/<vessel>_<parts>.json`,
  untracked; regenerate with `./spaceplane/tools/holdprior.py logs/LOG7[0-4]??`).
  v2 lets tracking above the prior override it.  rot-prior-1006 (v1),
  rot-prior2-1006 and rot-prior3-1006 (v2): rigoff long arrivals **10/20 ->
  2/20**, but landings level (rigoff 9/20 each); inc 7/12 -> 9/12; high
  **7/8 -> 4/8**; `qs_plane` 1/8 -> 0/8 with arrivals 5.5-8.5 km **short**.
  Under the prior every shuttle orbit hands the cone over low (-100..-1000).
  It fixes where the glide arrives, not the energy it arrives with.
- `HAC_WEAVE_STRAIGHT_ONLY` (2e4d2aa): no weave on the circle (on the lap
  stack it walked the vehicle off the circle: 7 of 12 out-of-height exits).
  rot-straight-1006, 24 an arm: 11 vs 11.  Null.

## Next

1. **Same mechanism, change what is measured** (fifth try on the alpha
   ceiling -- the rule says change the method): instead of predicting the
   alpha, correct the prediction by the *measured* aero.  A filtered
   ratio of measured to predicted drag and lift (the `act`/`cda` columns,
   already computed every tick) applied in the propagator, decaying with
   q distance -- the standard entry-guidance navigation correction.  It
   sees the shortfall the tick it starts (38 km) instead of planning a
   ceiling, and is not biased toward flights that saturate.  Caveat from
   the data: by 38 km the long flights' bank is already 64-70, so check
   first (logs above) whether the solve still has bank authority when the
   ratio first drops below ~0.85; if not, the correction has to come from
   the energy plan above 40 km.
2. Why does a lower planned alpha make this vehicle arrive *short*
   (`GLIDE_ALPHA_MAX_DEG` 34: 0/24 short; the prior: low handovers)?  Less
   lift sinks it into denser air; the propagator should already know that,
   so find which part of the energy chain disagrees (glidesum on
   rot-prior3 arms 0 vs 1).
3. The cone's handover low on rigoff/high under the prior -- after 1.
4. Carried over: TOUCHDOWN_AIM_M per vehicle; spaceplane/CLAUDE.md "Next".

## Traps paid this session

- `rotfly` outputs: score with fields `along`=$11, `across`=$13, parts=$17,
  and dedupe by LOG (one arm printed 13 lines for 12 flights).
- The PyPy venv's .pth puts the main repo on the path: the suite cannot run
  in a git worktree (develop there, test after merging).
- Swap reaches 18-20 GB after every ~1 h batch; restarted every time.
