# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-01 evening: the pitch-up is the pitch thrusters").

Last written **2026-10-01 ~20:50**, spaceplane. Code defaults unchanged
apart from new flags, **all off**. Offline spaceplane suite OK (681).
Committed. Farm **stopped** (the user needed the CPU; the last batch was
killed mid-round), inhibitor **released**, no sims running.

**The user's rules from this session** (in CLAUDE.md):
- whenever you stop to wait for a batch, give an approximate finish time in
  24-hour form ("done ~2120");
- screen in kspSim whenever the question is one it models; farm-only where
  its known gaps say so (CLAUDE.md step 1a).
Also: `testInstances/` is now excluded from KDE's baloo indexer (user OK'd).

## Where it stands

Goal ("lands mostly reliably") **not reached**. Best measured stack is still
**chain6** (exact string: `arm0` of `logs/rot-shortfall-1001.txt`): ~3/8
intact from orbit, ~half of flights depart in the glide.

**The glide pitch-ups are three classes, and the biggest is the pitch
thrusters.** Whenever the RCS valve opens in GLIDE (for yaw: reversals,
bank lag, or a steady trim shortfall reading as 5 deg of error), every block
also thrusts in pitch and drives alpha *through the surfaces' trim limit*
into 48-80 deg. Class A (Mach 4.7-7, little slip, `rcs on` 0-5 s before):
~15 flights before today, LOG4352 the clean case. Class B: residual drain
at Mach 2-3.3 (note: LOG4298-4307 in that evidence are *sim* flights).
Class C: lateral departure, roll damper `bank swings growing`, slip 20-70.

## Flags built this session (all off)

| flag | what | status |
|---|---|---|
| `RCS_IGNORE_ALPHA_SHORTFALL` | valve error ignores alpha *below* command in GLIDE/HAC | built for the wrong trigger: `rot-shortfall-1001` 0/8 vs 3/8 class A, but the control arm's valve opened on reversals, and LOG4415 (flag) still pitched 26->53 at Mach 7.1. Superseded by the next |
| `RCS_PITCH_OFF_IN_GLIDE` | pitch thrusters off from the first GLIDE tick, latched (`rcs_pitch_gate`) | **cures the departures**: `rot-pitchoff-1001` glide departures M>2 **1/8 vs 6/8** (p~0.02). **But** the vehicle then flies 2-5 deg under a 36-37 command at Mach 2-5 and every glide arrived **+2.4..+34 km long, 19-22 km high** at the cone; 0 intact |
| `HOLDABLE_MEAN` (+`HOLDABLE_MEAN_SAMPLES`=20) | `Holdable` learns the running mean of alpha held on ticks that saturated or asked >= the estimate (was: max while saturated, and *the command* on any tick within 2.5 deg -- so a 3 deg shortfall learned the command, "learned 36.0" flying 31-34) | **half-flown** (`rot-holdmean-1001`, both arms pitch-off, killed after round 1, 4 v 4): handover h 13.5-14.5 km vs 16-17; long +125, -1872 m on two, but LOG4446 +9.8 km and LOG4447 departed at M4.1 (lateral). Not a result |

`RCS_PITCH_BY_AUTHORITY` (yesterday's "null") never tested class A: it
latches at q ~1650 Pa, after those events (q 1100-1600).

## Next, in order

1. **Re-fly `rot-holdmean-1001`** whole (restart the farm first): chain6 +
   `RCS_PITCH_OFF_IN_GLIDE` vs the same + `HOLDABLE_MEAN`, 4 rounds. Read
   `GLIDE -> HAC` long/h per arm and departures (scratch `dep.py`-style:
   first tick flown alpha > commanded + 15 at M>1.2).
2. **Before that, if the farm is busy: kspSim screen** of the same two arms
   -- first check the sim reproduces the 2-5 deg shortfall with pitch
   thrusters off (that depends on its pitch trim, known gap 1). If it does,
   tune `HOLDABLE_MEAN` there.
3. Class C (lateral) is now the main departure: the valve opens late in slow
   reversals (`reversal_under_way` needs a 5 deg lead; LOG4409 opened 8 s
   in, slip already -14). Hold the valve open while the bank command is
   *moving*, not only leading.
4. The landing chain on good arrivals (unchanged from last session): lateral
   capture with bank still on (LOG4385), flare entry 66-95 m/s.

## Traps paid this session

- **Swap grows with batch length**: 5.3 GB after one 80-min 4-round batch,
  10.4 GB after the next. The KSP processes themselves (2.4-2.8 GB each).
  Restart the farm before *every* batch; baloo was only ~0.4 GB anon.
- Void logs: LOG4451-4454 (killed in round 2 of `rot-holdmean-1001`).
- `dep.py`/`revs*.py` were scratch; the logic is in the journal entry.
