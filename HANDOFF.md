# HANDOFF — read this first, rewrite it last

Snapshot of the last session (2026-10-05 evening, wrapped up early at the
user's request -- usage limit). History: `docs/spaceplane/journal.md`,
"Session, 2026-10-05 evening: the cleanup, and the cone's gap".

Defaults fingerprint **`01c06c20`**. Everything committed and pushed;
how-it-works PDFs rebuilt (status table updated).
Farm **stopped**, sims stopped, inhibitor **released**.

## What changed this session

1. **Big cleanup of `spaceplane/`** (the user's request): 79 retired
   off-by-default flags and 60 settled default-on flags baked in, their dead
   code / stubs / config fields / tests removed. Package 21.5k -> 14.9k
   lines, config 580 -> 341 fields, ~11k lines gone in total. Proven
   behaviour-preserving by a guidance-law fingerprint (cone, approach,
   flare, glide solve, propagator over a state grid, byte-identical) plus
   pyflakes/pylint call checks, the full suite (560 OK) and kspSim full
   flights. Resurrect anything from **`bbfd1e9`** (last pre-cleanup commit).
   Kept switchable: the live candidates (below), `DIAG_INTERFACE`, and the
   operational switches (warp, RCS master, drain, both ends, ...).
2. **`STALL_SPEED_M_S` and `STALL_CALIBRATION_M_S` removed** (the user: the
   48 was the old craft's, measured as TAS). `airframe.stall` is always the
   vehicle's own table, scaled by sqrt(mass) from the glide on. **This moves
   every speed on final ~6% up on the shuttle (approach ~115 vs 108) and
   has NOT been measured on the farm yet** -- first thing to fly.
3. In-game panel: thin even red frame, rounded corners (common/panel.py).
4. CLAUDE.md: push to GitHub on wrap-up.

## Flags added (off), and what they measured

- `HAC_IAS_FROM_STALL`: cone at one IAS = max(1.3 Vs, V_md) x sqrt(load at
  45 deg). On the shuttle V_md (gear up, alpha 0) is 112 -> IAS 133: too
  fast, 0/12 (rot-ias-1005, with the chain below). Wants a different speed
  rule (the user asked for "constant IAS derived from stall").
- `ENTRY_INTERFACE_AT_AIR`: glide takes over at the atmosphere top instead
  of the arbitrary 58 km. Unflown.
- Cone energy budget now counts excess speed against the *gate's* target
  (inert under the default constant-TAS cone; fixes the constant-IAS case).
- Chain `HAC_SPEED_EAS + HAC_LD_AT_TARGET + HAC_LD_MEASURED + HAC_WEAVE_HELD`
  before that fix: 0/12 vs 5/12 (rot-cone-1005).

## The blocker: the cone's structural gap

From orbit the cone rolls out **+0.6..+4 km high** (game and sim alike;
sim baseline surplus median +1.9 km sd 1.9, 1/12 within +-500,
`spaceplane/tools/conexit.py`). Mechanism: the entry delivers the vehicle
on the centreline ~15-19 km out at 14-17 km, so the circle has ~0 turn;
laps=0 offers ~16 km of path (+weave, ~25 km at 50 deg), a lap costs
>= 2 pi R more (~12.6 km at R 2 km, far more at entry speed), and the
energy wants 26-30 km -- the gap. `hac_choose` always picks the cheapest
end/hand. Root cause upstream: `HAC_GATE_LD` 1.35 (old craft's) places the
entry aim too close for the shuttle's glide.

**Sim screen at wrap-up** (simarms, 4 per arm, qs_shuttle2, LOG6535-6550;
small n, and the arm-to-log mapping from simarms output is loose):
cone surplus median / sd / within +-500 m --
defaults -206 / 2398 / 3 of 8; `HAC_AIM_DERIVED` +186 / 943 / 4 of 5;
**`HAC_AIM_DERIVED` + `HAC_LAP_AT_TARGET_SPEED` +340 / 280 / 3 of 5** (the
lead); + `HAC_FLAP_BRAKE_ON_SURPLUS`+`IGNORES_ROLL` -193 / 2306 / 2 of 4.
One flight (LOG6548) coasted round another orbit and was killed.

## Next, in order

1. Farm: defaults (new, stall removed) on the three rigoff orbits,
   12+ per save -- the regression nobody has flown yet.
2. Cone: re-screen `HAC_AIM_DERIVED + HAC_LAP_AT_TARGET_SPEED` in kspSim
   with 16+ flights per arm (map logs to arms by instance, not the fuzzy
   LOG column), then fly it on the farm against defaults.  Then screen in kspSim (8-16 flights/minute, farm stopped), confirm on
   the farm. Candidates: `HAC_AIM_DERIVED` (replaces HAC_GATE_LD),
   `HAC_LAP_AT_TARGET_SPEED`, the cone flap brake on surplus, choosing the
   end/hand by energy fit rather than min cost, a larger weave cap.
3. Then the constants still standing in the chain: `HAC_LD` 1.86,
   `APPROACH_BEST_LD` 4.2, `TOUCHDOWN_AIM_M` 2400, `APPROACH_AIM_SHIFT_M`,
   `APPROACH_FACTOR` 2.25, `HAC_GATE_LD`, `MARGIN` 0.70.
4. Repo hygiene (the user's rule, now in CLAUDE.md): `saves/` variants
   made by `savegen.py` / hand splices are still tracked -- write a
   regeneration script before untracking them (they are the farm's
   reference via `syncSaves.sh`).  `ksc_terrain.json` is untracked now.
5. Comments still naming removed flags (~100 lines, mostly autopilot.py
   and config.py) -- trim.

## Traps paid this session

- **A flag switched per call (`replace(cfg, X=True)`) is not a default**:
  baking `ALPHA_TRACKING_ON` silently disabled the deorbit window's
  tracking corners. Check `replace`/`--set` users before baking a flag.
- The guidance fingerprint does not cover `autopilot.py`: a one-row tuple
  (from the stall removal) and a `run.attr` read both crashed every flight.
  **Fly one kspSim flight (`./kspSim/tools/simfly.sh qs_shuttle2 2 1`)
  after any code change before a farm batch** -- it takes a minute.
- Scripted multi-file tools emptied `gui.py` and `__init__.py` once:
  compare line counts against HEAD after any scripted edit.
- `rotfly.sh ... &` chained after `start.sh` backgrounds the whole chain.
