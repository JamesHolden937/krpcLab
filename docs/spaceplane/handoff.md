# spaceplane HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-01: spaceplane -- the approach plans in the middle of each craft's own span; the low-alpha glide is off (fingerprint `07089ff3`)

Committed and pushed; farm stopped (the last batch, heavy-1101, was killed by Claude Code for low memory before it flew -- LOG15091-15095 are part-batch, don't read them). Journal: "Session, 2026-10-31 late" and "2026-11-01". **howItWorks.pdf NOT updated for the approach ratio** -- do it next wrap-up.

**`APPROACH_BEST_LD` is deleted (the user: "remove the hardcoded value").** It was qs_shuttle2's 4.2; the live Heavy Shuttle (best glide 3.0) planned at 4.2 got a 7.2 km gate and landed 765 m short (LOG15006). Now `airframe.approach_ld` = clean reach x sqrt(braked span), at `GATE_ALT_M`:
- clean reach = glide x (h + float)/h: glide is `cla/cda` at 1 g at `APPROACH_FACTOR` x stall, read from the STANDBY sweep **at the dry mass** (`Vessel.dry_mass`, `airframe.stall_at`) and kept on `env.approach_glide_ld`, scaled by `env.approach_ld_scale` (= `hac_ld_scale`, the flown/table ratio, 0.94-1.16 on both craft); float = `(v_app^2 - v_stall^2)/2g`.
- braked span = the split rudder's last `APPROACH_SPLIT_FACTOR` (0.45), or cos(`APPROACH_SCURVE_MAX_DEG`) with no pair.
- Consumers' formulas unchanged (gate, handover `approach_needed`, excess, S-turn, split brake, short divert). The DISAGREES check is gone.
- **qs_shuttle2: 5.11 x 1.23 x 0.67 = 4.22** (gate 7242 m, was 7200) -- inert, by design. Heavy, from LOG15006's live table: L/D 2.65 at 115 m/s, reach 3.37 (flew 3.41), **planned 2.26, gate ~3.3 km**. Unflown on the heavy: see below.
- **Refuted on the way:** planning at the clean reach (bec678c, reverted 6fb047b): reach-1031 0/12, every flight short at 6.3. The table *is* right where flown; the shuttle simply can't be planned with no margin.

**`GLIDE_LOW_ALPHA_SPEED_M_S` 1000 -> 0 (off).** lowa-1101, rigoff 12 an arm: on 2/12 on the runway, **7/12 out of energy 56-82 km short** ("on the ground in HAC"); off 7/12, none. This was last session's unflown default and the user's live flights carried it. The rule says delete or promote: it was the user's idea, so ask before deleting the code.

**`saves/qs_heavyshuttle_rigoff`** (new): the live Heavy Shuttle on rigoff's orbit, LOG15006's propellant (70.37 t), surfaces/gear stowed. Two traps cost two batches: the first copy had full tanks (182.6 t, entry breakup, 0/8 at 100-300 km) and was taken from a persistent saved *after* landing (4 surfaces deployed, gear down: the farm table read CdA 41/ClA 12 at 0 deg against live 25/56). **The stowed version has not flown.**

**Added after the wrap-up (2026-11-01 evening, the user, default, unflown -- they fly it live):** `APPROACH_HOT_BRAKES` -- past the threshold with energy excess (height over best glide to the aim + (v^2-held^2)/2g) > 0, the split rudder goes to its stop and the opposed flaps open at once; latched; still stowed at the flare door. Log line `hot over the runway: every brake out`. Cause: LOG15100 (live Heavy Shuttle) crossed the threshold 460 m up at 95 m/s, rudder 24/38 deg, flaps never opened (they wait for the rudder's stop), rudder retracted at h=400 once the aim was behind it, flare 1.1 km down, touchdown 1726 m along, then an overrun divert north 0.8 s after touchdown on a 2.1 m/s^2 first reading. Also: the overrun divert now waits 1 s after touchdown. Root cause still open: the approach's `exc=` is height-only, and the speed schedule (127 -> 91 m/s) hides ~360 m of energy, so it read 430 m low at the cone exit and opened the brakes 900 m out. Fix candidate: count speed over the held speed in the split brake's factor.

**Next, in order:**
1. Fly heavy-1101: `multirot.sh heavy-1101 2 "qs_heavyshuttle_rigoff|" "qs_stackorb_live1028|"`. Read the `approach at ... planned at` line and the gate (expect ~2.3 / ~3.3 km), and whether the farm table now matches LOG15006's (`aero M=0.3` row). If the heavy still breaks up in the entry, that's its own failure (RCS blocks at 100% skin, then 74 deg alpha at 46 km in LOG15043).
2. Ask the user: delete `GLIDE_LOW_ALPHA_SPEED_M_S`'s code, or find why bank-first runs out of energy.
3. Rigoff off-arm still had 2 ROLLOUT losses and 3 misses (-2449, +1898, -8813) in 12.
4. The brake probe's quiet-inputs version (a3a64cf) still unflown; read `spoiler (measured) ... armed/REFUSED` live.
5. Carried: `APPROACH_SPLIT_FACTOR` is qs_shuttle2's splitprobe table and now also sets the approach's span; LOG14999's 118 m off-centreline touchdown; the stack orbiter's cone outliers; kspSim probe models.

**Traps paid:** a background batch launched from `testInstances/` (the Bash cwd resets there) silently found no `spaceplane/tools` -- `cd /home/holden/krpcLab &&` in every launch. The telemetry's `cla=`/`cda=` are the *table at the commanded alpha*, not flown values. The swept table re-read at the landing mass is noisy (2.4-5.6 best glide on one craft). Zram at 28 GB after two cycles is now normal; restart between batches.

### Earlier spaceplane snapshot -- 2026-10-12 evening: low alpha flown and refuted; the live tumble not reproducible (fingerprint `01de6e69`)

Defaults unchanged all session (every arm was a flag; all deleted).
Committed and pushed to origin/main.  Farm stopped, sleep inhibitor
released.  How-it-works PDF current (the only default change is logging).
Full write-up: `docs/spaceplane/journal.md`, "Session, 2026-10-12 evening".

### Low alpha -- what was found (the user asked for it in the handoff)

The user asked why every plan flies 20-40 deg and whether a lower-alpha
trajectory would be better, thermally checked.

- **Why it is high**: `solve_glide` uses alpha as the primary range control
  (bank is the residual, floored at 30), the deorbit plans inside the box
  (`SOLVE_ALPHA_MIN_DEG` 20, bank 70) to (`ALPHA_MAX_DEG` 32, bank 30), and
  the glide solves in 20-40.  Every plan is high-alpha by construction.
- **Thermal is not the limit.**  Heating rate H = sqrt(rho) v^3 / 1e6 (KSP's
  high-Mach convective coefficient, from Physics.cfg).  The hottest part is
  always the RV-105 RCS block; at v >= 1600 m/s it plateaus at **0.86-0.88
  of its skin limit over H 128-512**, peaks 0.70-0.80 on i90/rigoff and
  0.85-0.88 on e500i45 (H up to 611).  The plateau is not the shock-temp
  ceiling (21 v^0.75 ~ 7000 K), so it is not known past H ~610.  A point
  mass on the logged aero table (calibrated: 220 v 243 flown) gives peak H
  220/257/298/364 at 30/20/15/10 deg, bank 30 -- circular orbits fit inside
  the flown envelope even at 10; e500i45 would exceed it.
- **The shuttle cannot fly low alpha hypersonically by command alone.**
  lowa-1012 (LOG10081-10104; plan 10-20 deg via `SOLVE_ALPHA_MIN_DEG=10` and
  a deorbit band top of 20): **3/12 on the runway v 11/12**, misses to
  -165 km.  At Mach 4-5 it was commanded 16-18 and flew ~22: **its
  hypersonic trim point**.  Fighting the trim leaked into yaw: slip > 10 deg
  on 22% of Mach-4 ticks v 9% on the defaults (LOG10084: 9 commanded, 34
  flown, 28 deg of slip at Mach 3.8).  Heat never bound (skin 0.87 v 0.85).
  The plan also stretched entries to ~2080 km (v ~1500), the shallow regime
  the propagator over-predicts.  Below Mach 2 low alpha is held fine.
- **What would make it work -- move the trim point, then re-plan:**
  1. CoM forward by pumping the nose LFO (`ResourceTransfer`, already used
     as a cone trim) and/or a standing glide trim on the canards/elevons
     (like `CANARD_TRIM` in the cone).
  2. Measure the trimmed alpha by Mach with the broadside probe
     (`BROADSIDE_PROBE_DEG`) before and after.
  3. Only if the trim reaches 15-20 at Mach 4-6, re-fly a lower band --
     and keep the deorbit window out of the >1500 km shallow entries.
  Expected upsides if trimmed: bank changes become body roll (yaw share
  sin(alpha) 57% at 35 deg v 26% at 15), fins out of the wake, brakes
  more effective; downside: higher L/D amplifies model error, more energy
  to dump in the cone, q 2-3x (the low arm's glides reached 12-30 kPa and
  survived).  The code for the band/thermal limits was deleted
  (`git show e145a51`); `deorbit heating` stays in the log.

### The live tumble -- superseded by 2026-10-20 above (aft CoM, `FUEL_TO_NOSE`)

### Added (default, logging only)

- `nz=` load factor on every telemetry line; a `load factor` event at each
  new peak (q, Mach, alpha).  Defaults' glides peak at 1.4-2.4 g.
- `deorbit heating: alpha A peak H, ...` at deorbit commit: each window
  corner's predicted peak heating rate (`trajectory.peak_heating`).

### Where each orbit stands (defaults, this session's farm batches)

tumble-1012 + lowa-1012 defaults: rigoff 12/12, i90 7/8, i90low 3/4 (+ 11/12
in low70-1012, both arms), e500i45 10/12.  Misses: long splashes
(+6.3, +12.7 km), rollout breakups (19-24 parts left), one -2.3 km.

## Next, in order

1. **Landing accuracy on the default band** (the goal: >= 95% stopped,
   0 lost, all orbits): the remaining misses are long overruns after the
   cone (surplus) and rollout breakups.  The handoff of 2026-10-11 item 1
   still applies: measure the split brake by Mach (`splitprobe.py --glide`),
   `HAC_SPLIT_FACTOR` over-credits the upper cone.
2. Low alpha, if wanted: the trim-point measurement above, first.
3. A load-factor guard / destructive probe (the user asked about runtime q
   tolerance): the `load factor` events build the envelope from every log;
   a guard on commanded alpha/bank and a farm-only breaking probe are the
   next steps (not built).
4. `HAC_SHORT_POLAR` (off, unflown) -- re-fly on the current default or
   delete.  Older: `ABORT_NORTH`/`GROUND_SPOILER_ON_NOSE` farm measurement.

## Traps paid this session

- **A farm that won't reproduce a live failure: diff the GameData first.**
  The live install runs ~50 mods the farm does not, one of which feeds a
  joystick into the control state.
- **Lowering the commanded alpha below the airframe's trim is not flying
  lower alpha** -- it is a standing moment the attitude loop holds badly,
  and on this shuttle it couples into yaw.  Move the trim, not the command.
- Batch round length depends on the arm: the low-alpha cycles took 27 min
  v 18 -- estimate from the slowest arm.
- `multirot` arm numbers rotate per cycle: join results to each log's
  `config:` line, never to the `armN` label.
- A source-text test (`TestTheWindowClockIsLoadBearing`) bans the string
  `index == 0` in `deorbit_window`.
