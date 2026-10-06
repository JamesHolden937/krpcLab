# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-06 (overnight, ~0000-0750).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-06 (overnight)".

Defaults fingerprint **`d91cd0ad`** (= the promoted `f31c4cbe` plus the
off flag `HAC_SHORT_BEST_GLIDE`).  Everything committed and pushed.
Farm **stopped**, sims stopped.

## What changed this session

1. **Promoted to defaults** (e584a8a), measured on the farm:
   `HAC_AIM_DERIVED` (the cone's entry aim from the table, replacing the
   old craft's `HAC_GATE_LD` 1.35), `HAC_LAP_AT_TARGET_SPEED`,
   `HAC_WEAVE_FIRST_WITH_BANK` (new: the weave's first swing agrees with
   the bank being flown -- the old clock opened every cone with -45 deg,
   a 100-115 deg reversal at alpha 42 / Mach 0.9 when the glide handed
   over banked positive, and departed), and **`TOUCHDOWN_AIM_M` 2400 ->
   1800**.
   - Shuttle, rot-newdef-1006: **19/36 intact on the runway** (rigoff
     5/12, inc 4/12, high 10/12) against 3/36 on the old defaults
     (rot-base-1006).  Cone handover within +-500 m 23/36 (was 2/36).
   - Old craft, same batch: **1/12** -- see the blocker.
2. **`TOUCHDOWN_AIM_M` 1800 is a ship bias, kept on purpose (the user).**
   Recorded in its config comment and as item 0 of `spaceplane/CLAUDE.md`
   "Next": derive the aim per vehicle (aim + flare float + rollout fits
   the runway), fly it against 1800 on both craft.
3. Tools: `conexit.py` groups by each log's own config line (exact arm
   mapping); **`hacentry.py`** (new) shows the glide-to-cone handover --
   glide bank, first cone commands, departures, stops -- by arm.
4. kspSim: **gap 7, its landings do not follow the game** (stops short
   where the farm lands long) -- screen the cone there, never the aim /
   approach / flare.  `simarms.sh` times a flight out at 600 s (runaway
   flights coasting an orbit held two screens for 100 min).

## Flags added (off), and what they measured

- `HAC_SHORT_BEST_GLIDE`: a cone short of height moves alpha toward the
  table's best-L/D alpha, in proportion to the deficit.  Null in kspSim
  twice (sim-short-1006, sim-short2-1006): handovers higher, sd up.

## Blockers, in order

1. **`qs_shuttle2_rigoff` is bimodal in the glide.**  7/12 arrive 3-10 km
   long at 19-21 km (the rest +500 at ~16 km), same burn, same loop rate.
   The glide's predicted miss holds +500 until ~31 km / Mach 4.3; in the
   long flights the commanded bank has ramped to 66-70 deg (pinned)
   before a reversal, and the reversal through wings-level leaves no
   authority (LOG6845, 6826; normal LOG6812, 6816 hold ~30 deg there).
   Find where the energy state splits, earlier in the glide.
2. **`_inc`: the cone flies a lower L/D than it plans** -- entry margins
   +700..+1800 become handovers -700..-1000 and land 3-7 km short
   (LOG6813, 6817, 6822, 6827).  Compare planned against flown turning
   L/D on that orbit (`conesum.py`).
3. **The old craft rolls off the end** on the 1800 aim (touchdown 1.3-1.6
   km in, ~2 km rollout).  Part of item 0 in `spaceplane/CLAUDE.md`.

## Next

1. Blocker 1, narrowed after the checkpoint: every rigoff glide looks
   identical down to ~36 km (bank +-30-36, alpha 35-39, predicted +0.5
   km).  The split is the reversal at 32-36 km: normal flights come out
   at +-30 and stay; long ones come out needing 44-51 deg and are pinned
   at 70 within 30 s (LOG6845, 6826, 6836 against 6812, 6816, 6831).  The
   propagator models a reversing entry as a constant cos(bank) share
   (`trajectory.py` ~690) -- the lift-up time of a slow Mach-4 reversal is
   invisible to it.  **But that model was corrected three times and
   refuted three times** (failures 79, 85: `GLIDE_BANK_DUTY_ON`,
   `ALPHA_TRACKING_ON`, both) -- on the old craft, while the drain valve
   dominated.  So change the method (failure 85's own advice): keep bank
   authority in reserve late in the glide so a reversal cannot pin it
   (e.g. cap the solve's bank below `BANK_MAX` by the reversal's cost, or
   forbid reversals below a q / Mach where the remaining authority cannot
   absorb one), rather than a fourth propagator correction.  Check first
   whether kspSim reproduces the split on `qs_shuttle2` (it has no rigoff
   model; its sim cone entries were all 16-18 km, so maybe not).
2. Blocker 2 with `conesum.py` on rot-newdef-1006's `_inc` logs.
3. The derived touchdown aim (spaceplane/CLAUDE.md "Next" 0).
4. Carried over: the other fitted constants in the chain (`HAC_LD` 1.86,
   `APPROACH_BEST_LD`, `APPROACH_AIM_SHIFT_M`, `APPROACH_FACTOR`,
   `MARGIN`); untracking `savegen` save variants behind a regeneration
   script; ~100 comment lines naming removed flags.

## Traps paid this session

- **The sim's landings are not the game's** (kspSim gap 7): a sim screen
  of the aim said the opposite of the farm.
- **A long simarms screen is a runaway, not slow sims**: a normal screen
  of 30 flights takes ~2-4 minutes.
- **Kill the kspSim servers before the farm** (`kspSim/stopall.sh`):
  16 idle servers were left up and swap read 12 GB at farm start.
- `rotfly.sh ... &` chained after `start.sh` with `&&` still backgrounds
  the whole chain (it works, but the start output is lost).
- `rwysum.py`'s `td rwy` is a distance: read the signed stop to tell a
  short landing from a long one.
