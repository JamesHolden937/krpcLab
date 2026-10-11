# HANDOFF — read this first, rewrite it last

Snapshot of 2026-10-10 evening (~1600 -> 1830).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-10 evening".

Defaults fingerprint **`a4b58a35`** (was `66f98be6`; only new off-flags
added, default behaviour unchanged).  **Committed, not pushed** (the
user: "don't push").  Farm stopped, sleep inhibitor released.  No
worktrees open.  `howItWorks.pdf` unchanged (no default changed).

## The headline

The user's redirect: **fix the glide so it hands the cone the energy the
cone plans for, instead of making the cone absorb 5-9 km of surplus
height.**  Built as two off-flags (commit 6b60b36):

- `GLIDE_TAEM_ENERGY` (+ `GLIDE_TAEM_MACH`, `GLIDE_TAEM_MARGIN_M`): below
  `GLIDE_TAEM_MACH` the glide solve nulls the predicted **cone-entry
  energy height** (h + v^2/2g where the arc meets the entry test) against
  `HAC_ALT_M` at cone speed + margin (~14.1 km), not the 12 km crossing.
  `guidance.taem_target` / `taem_miss`; reserve 0 in TAEM; telemetry
  `tE=`, and `long=` is the energy surplus in metres.
- `GLIDE_TAEM_ALPHA` (+ `GLIDE_TAEM_ALPHA_TAU_S`): the propagator flies
  min(command, EMA of achieved alpha + `HOLDABLE_MARGIN_DEG`)
  (`env.taem_alpha_cap`, `taf=`).

**Result, farm `taem-1010`** (4 cycles, defaults v TAEM at Mach 4.5 with
both flags, rigoff/inc/high/ecc, 24 an arm, LOG9480-9527):

| | defaults | TAEM |
|---|---|---|
| cone-entry energy, typical | 19-21 km | **14-15 km** (target 14.1) |
| on the runway (\|along\| <= 1.2 km) | 18/23 | 19/24 |
| along mean / sd | +1.0 / 1.2 km | +2.8 / 4.2 km |
| tails | -27 km broke up (9496), -5.6, -4.9, +1.8 | **+16 water (9490), +7.5 water (9505), -12 (9483), -6.4 lost (9510)**, -2.1 |

It does what it aims at on 19/24 (those all stop ~+0.8 km, high orbit
included), but **not promoted**: two mechanism defects made the tails.

1. **Alpha ratchet through the solver** (9490, 9505, 9510, high orbit).
   With the propagated alpha capped at flown+margin, commands above the
   cap predict identically, so the solve can't see alpha helping; with
   +12 km surplus it walked the command 36 -> 21 deg, the vehicle
   followed, the cap followed the vehicle (30 -> 21), cone entered at
   437 m/s, 17 km.
2. **No range constraint** (9483, inc).  Solving energy from Mach 4.5
   left it 4.5 km energy-short; min alpha wings-level still crossed 12 km
   16.6 km out of the gate, 12 km short.

Sim screen before it (logs/sim-taem-1010.txt, sim-taem2-1010.txt): at
Mach 3 the switch-over changes nothing (solve saturated below Mach 3:
alpha 40-42, bank 50-70 with reversals); at 4.5 / 6 entry energy fell
from ~20 to 16-17 km.  Sim needs `DEORBIT_SAS_ALIGN=False` (stub SAS).

## Next, in order

1. **Fix the two TAEM defects, then re-fly the same batch**:
   (a) in surplus the solve may only *raise* alpha (or: don't let the cap
   bind the solver's alpha search -- cap the prediction's alpha but keep
   the command at max while surplus > 0); (b) the miss is the shorter of
   range and energy (energy can't override a range short).  Screen in
   kspSim (`DEORBIT_SAS_ALIGN=False`), then farm 24 an arm, four orbits,
   **restart the farm first** (swap ended at 23.6 GB).  Promote or delete
   both flags on that result -- no parking.
2. If TAEM wins: the cone's surplus-height work (old item 1) mostly goes
   away; re-read conesum on the new entries before touching the cone.
   If it loses: the cone spends surplus as path (wider circle / partial
   lap / S-turn, alpha capped at the lift peak).
3. LOG9375: exited the cone needing 6.6 km more height, lost 27 km short.
4. `docs/spaceplane/constantsAudit.md` top-down; parked-off
   `AIRBRAKE_CACHE`, `ALPHA_TRACKING_ON`, `APPROACH_CAPTURE_TAU_SHARE`:
   promote or delete.
5. Glide bank overshoot past 90 deg: the roll loop, not the sign law.
6. Deorbit "model said" log figure wrong on first burn tick (log-only).

## Traps paid this session

- **kspSim's SAS is a stub**: every deorbit times out on the 60 s guard
  under today's `DEORBIT_SAS_ALIGN` default.  Fly sims with
  `DEORBIT_SAS_ALIGN=False`.
- **Idle kspSim servers hold ~400 MB swap each** (22.6 GB before the
  farm).  Kill them: `kill $(pgrep -f "m kspSim[.]run")`.
  `pkill -f "kspSim/run.py"` matched and killed the calling shell.
- Capping the *propagated* alpha at what is flown re-creates the command
  ratchet through the solver unless the solver's alpha search is fenced.
- Swap still ends a 4-cycle multirot at ~23 GB; restart before comparing.
- "could not pause after the load": 2 this batch (LOG9502, 9504), kept.
