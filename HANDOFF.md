# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 morning: the slow reversal").

Last written **2026-10-02 ~13:15**, spaceplane. Defaults fingerprint
**`d695195b`** (no default changed this session; the hash moved only because
new fields were added). Offline suite OK (858). Committed (this file is the last commit).
Farm **stopped**, inhibitor **released**, no sims running.

## Where it stands

Goal ("lands mostly reliably") **not reached**. The entry scatter is still
the bank reversal at Mach 5-4 (q 1500-5000): on the best stack
(chain6 + `RCS_PITCH_OFF_IN_GLIDE`, exact string in `arm0` of
`logs/rot-sweep-1002.txt`) **every one of 16 base flights today slipped
17-51 deg**. The farm's absolute level drifted down from last session (within
5 km 5/16 today vs 11/16 then; intact 0/16 vs 3/16).

## Built this session (all off)

| flag | what | measured |
|---|---|---|
| **`GLIDE_BANK_SWEEP`** (+`_UNTIL_MACH=1.0` to fly it) | the user's "one huge reversal": hold a side, cross *slowly* (<=1 deg/s, `_RATE_MAX`) to the other, hold; the crossing's start and rate are solved every tick for the cross-track at the gate; the range solve flies the same `trajectory.BankPlan`; relay below Mach 1 | farm 13 crossings (`rot-sweep-1002`, `rot-sweep2-1002`): **slip 2.8-3.3 in 4, 15-28 in 9** (base: 17-51 in all 16). Bimodal, no discriminator found (loop rate, instance, RCS, rate, fuel trim). Landing unresolved at n=8 |
| `GLIDE_BANK_SWEEP_CROSS_ALPHA_DEG` | cap alpha while crossing (sin alpha coupling) | **null at 25**, `rot-unload-1002`: the three normal-range flights slipped 18-22; four others ran +182..+194 km long (the plan does not model the cap). Sweep clean fraction over three batches **6/21**, base 0/16 |
| `GLIDE_SINGLE_REVERSAL` | one fast flip, timed by a signed propagation | offline: 21 km off at Mach 2 (the post-flip magnitude differs from the planned one). Not flown |
| `GLIDE_BANK_MIN_DEG` | a glide-only bank floor (the 30 is the old craft's) | not flown; the floor binds 2-27% of hypersonic ticks |

Settled along the way: **lower alpha does not shorten the shuttle's glide**
-- range falls monotonically with alpha (offline from LOG4498's table), alpha
already flies to the plateau edge. A sweep slow *throughout* cannot shed the
energy (+-70 averages cos 0.77 vs ~0.5 needed: sim +57 km long). Holding one
side subsonically spirals (5 km turn radius). Handback below Mach 2: 11-16 km
long at the cone in the sim -- retired.

## Next, in order

0. **The user's call, try first: a single *fast* reversal, timed by the
   slow reversal's planner.** The slow crossing starts at Mach ~5.8, q ~1000
   and lingers near level ~100 s while q climbs to 3000-5000; the bad
   crossings begin slipping at q ~1500, part-way through. A fast flip
   started at the same moment is done in 10-15 s, before q 1500 -- and the
   three crossings that started at q ~700 were all clean. No new code: the
   `GLIDE_BANK_SWEEP` planner at full roll rate *is* the single reversal,
   with the plan-aware range solve that the offline `GLIDE_SINGLE_REVERSAL`
   lacked (it flipped on a 30 deg plan and the solve then flew 18: 21 km off
   at Mach 2). Fly, interleaved against the 1 deg/s version (8 v 8, both on
   the best stack + `GLIDE_BANK_SWEEP=True;GLIDE_BANK_SWEEP_UNTIL_MACH=1.0`):
   `GLIDE_BANK_SWEEP_RATE_DEG_S=8;GLIDE_BANK_SWEEP_RATE_MIN_DEG_S=6;GLIDE_BANK_SWEEP_RATE_MAX_DEG_S=10`.
   Screen 8 in kspSim first (~2 min) to check the planner still converges
   at that rate. Read slip in the crossing (the journal's awk), the q at
   which the lean passes zero, and the cone arrival.
1. **The bimodal crossing** (three mechanisms now refuted on it: rate,
   RCS state, alpha unload -- change the method: record the crossing with
   `kspSim/tools/flighttest.py --attach` on a farm flight and look at the
   lateral moments, rather than a fifth flag). Clean ones hold slip within +-2 from q 1000 to
   2400; bad ones are already -2..-5 at q ~1500, lean ~-20, always the same
   sign (crossing to +1 every time). Something is different as the crossing
   starts. Weak lead: the roll rate measured in COAST (clean 8.6-14.9, bad
   6.1-10.8 deg/s). Worth asking the user about yaw authority on the craft
   (the twin fins were their fix for departures) -- the vehicle cannot hold
   slip near wings level at q > 1500 in 70% of crossings.
2. Crossings that started at q ~700 (three, after an accidental Mach-7
   crossing) were all clean. The start is set by geometry (~0.3 of the
   glide) and cannot simply be moved; a deliberate early pre-crossing costs
   ~25 km of range unless the plan models it.
3. The landing chain on good arrivals (unchanged).

## Traps paid this session

- `farm start` again left two instances without ports after 10 min;
  `./start.sh 0 1` on just those brought them up in a minute.
- Swap reached 8.4 GB after an 85-minute batch: restart before every batch.
- `&& tail` after `unittest` hides a failure from the chain: one commit went
  in with a failing source-inspection test (fixed next commit).
- Sim screening was ~2 min per 8-flight arm and caught three design errors
  (energy, the subsonic spiral, the Mach-2 handback) before any farm time.
