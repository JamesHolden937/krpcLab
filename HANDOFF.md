# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 morning: the slow reversal").

Last written **2026-10-02 ~11:50**, spaceplane. Defaults fingerprint
**`d695195b`** (no default changed this session; the hash moved only because
new fields were added). Offline suite OK (858). Committed through `138e032`.
**A farm batch was running at the time of writing**
(`logs/rot-unload-1002.txt`, below); if this file was not rewritten after it,
read that log, stop the farm and release the inhibitor.

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
| `GLIDE_BANK_SWEEP_CROSS_ALPHA_DEG` | cap alpha while crossing (sin alpha coupling) | **in flight** at 25: `rot-unload-1002` (sweep vs sweep+unload, 8 v 8) |
| `GLIDE_SINGLE_REVERSAL` | one fast flip, timed by a signed propagation | offline: 21 km off at Mach 2 (the post-flip magnitude differs from the planned one). Not flown |
| `GLIDE_BANK_MIN_DEG` | a glide-only bank floor (the 30 is the old craft's) | not flown; the floor binds 2-27% of hypersonic ticks |

Settled along the way: **lower alpha does not shorten the shuttle's glide**
-- range falls monotonically with alpha (offline from LOG4498's table), alpha
already flies to the plateau edge. A sweep slow *throughout* cannot shed the
energy (+-70 averages cos 0.77 vs ~0.5 needed: sim +57 km long). Holding one
side subsonically spirals (5 km turn radius). Handback below Mach 2: 11-16 km
long at the cone in the sim -- retired.

## Next, in order

1. **Read `rot-unload-1002`** (the per-flight table in the journal's awk is
   the tool: max slip above Mach 1, min alpha Mach 2-6, crossings, cone
   long/cross/h). If unloading moves the 4/13 clean fraction, keep going on
   the crossing's alpha (and make the plan model it); if not, the slip is
   not roll coupling.
2. **The bimodal crossing.** Clean ones hold slip within +-2 from q 1000 to
   2400; bad ones are already -2..-5 at q ~1500, lean ~-20, always the same
   sign (crossing to +1 every time). Something is different as the crossing
   starts. Weak lead: the roll rate measured in COAST (clean 8.6-14.9, bad
   6.1-10.8 deg/s). Worth asking the user about yaw authority on the craft
   (the twin fins were their fix for departures) -- the vehicle cannot hold
   slip near wings level at q > 1500 in 70% of crossings.
3. Crossings that started at q ~700 (three, after an accidental Mach-7
   crossing) were all clean. The start is set by geometry (~0.3 of the
   glide) and cannot simply be moved; a deliberate early pre-crossing costs
   ~25 km of range unless the plan models it.
4. The landing chain on good arrivals (unchanged).

## Traps paid this session

- `farm start` again left two instances without ports after 10 min;
  `./start.sh 0 1` on just those brought them up in a minute.
- Swap reached 8.4 GB after an 85-minute batch: restart before every batch.
- `&& tail` after `unittest` hides a failure from the chain: one commit went
  in with a failing source-inspection test (fixed next commit).
- Sim screening was ~2 min per 8-flight arm and caught three design errors
  (energy, the subsonic spiral, the Mach-2 handback) before any farm time.
