# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-07 morning (~0700-1110).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-07 morning".

Defaults fingerprint **`db682413`** (= `8f16d628` + new off flags;
**no default changed**).  Everything committed and pushed.  Farm
**stopped**, sleep inhibitor released.

## Where it stands (rigoff, the bimodal save)

The offload (`GLIDE_PITCH_OFFLOAD` 0.6, `_MIN_MACH` 3, still off by
default) pooled over this session's six batches: **50/52 intact, 23/52
intact on the runway**; every miss is short (-1.6..-6.7 km), the cone
running out of height.  Previous session: 22/56 landed, 55/56 kept.

## What was found

1. **Flaps/spoilers are armed on every orbital flight and never deploy in
   flight** (the user asked).  The measured "spoiler" set dumps lift
   (dClA -30) rather than adding drag; the split rudder is 0.3 m^2 a side.
   Deployed in the cone they doubled the sink and spent nothing.
2. **Every cone flies on its pitch stop**: standing input +0.55..+0.76,
   pinned 7-49% of ticks, L/D 1.3-1.6.
3. **The missing control is the propellant.**  ~2 t of LFO sits in the
   nose tank 13-15 m ahead of four empty ones; `ResourceTransfer` moves it
   in ~2 s.  `PROPELLANT_TRIM` takes the standing input to +0.15, the bank
   tracks, the cone's glide ratio goes 2.01 -> 2.60.
4. **But the trim drag is this craft's only speedbrake**, so the trimmed
   vehicle lands long (7-15 km) and nothing downstream can spend it:
   not the flap brake, not `HAC_LD_MEASURED`, not a lower `HAC_ALT_M`
   (the cone shortens its path to match and exits high anyway).
5. **As an energy control** (`PROPELLANT_TRIM_ON_ENERGY`: aft only when
   the cone is short) the stops centre on the runway (-1.6..+3.4 km) for
   the first time, but 8/12 intact and ~1/12 on the runway, losses off
   the centreline.

## Flags added (all off)

- `PROPELLANT_TRIM` (+ `_MAX_MACH` 1.5, `_DEADBAND` 0.15, `_TAU_S` 5,
  `_SWEEP_S` 20): pump on the standing pitch input, HAC/APPROACH/FLARE.
  Column `ptrim=standing/unit-m moved`.  rot-ptrim..ptrim5-1007: worse
  than the offload in every variant.
- `PROPELLANT_TRIM_IN_GLIDE`: departs (deep stall at Mach 0.8, alpha 80,
  LOG7969, 7985).  Never.
- `PROPELLANT_TRIM_ON_ENERGY`: rot-ptrim6-1007, 8/12 intact vs 12/12.
- `HAC_FLAP_ARREST_EXCESS` (re-added; retired in 7fd5096 leaving
  `nominal = 0`): keeps the brake out 4-14 s instead of 3; spends nothing.

## Next

1. **Why the energy-mode pump loses the centreline** (rot-ptrim6-1007:
   LOG8084, 8091, 8096, 8098 at +341..+790 across).  Suspect the CoM step
   at the cone's exit: fuel pumped forward from the approach on, a plant
   change during the capture (failure 31's shape).  Try holding the fuel
   where the cone left it and pumping forward only in the flare, or
   ramping.  Read `oscsum.py` on those logs first.
2. **The offload's short misses** are the cone out of height, the
   pump's aft half's job alone: an arm with `PROPELLANT_TRIM_ON_ENERGY`
   and the forward pump disabled (aft when short, otherwise hold).
3. Carried from 2026-10-07 early: the glide's energy target from the
   cone's need; promote the offload (all three orbits + `qs_plane`);
   `TOUCHDOWN_AIM_M` per vehicle; CG-shifted `qs_shuttle2` copies.
4. The user's idea, not flown: pump into the **wing tanks** for roll
   inertia (LF only, mid-station, must split evenly).  Not needed unless
   the cone wallows in roll.

## Traps paid this session

- 8 flights an arm told a false story again: `HAC_LD_MEASURED` 5/8 then
  2/8.  16 is the minimum for a landing rate.
- ksp3 and ksp4 once each "could not pause after the load" (LOG7966,
  LOG8069 suspect); GameData matches `base/` -- one-offs.
- kspSim cannot screen anything about pitch trim near the stall (gap 1)
  or propellant location; the pump went straight to the farm.
- Swap 20-21 GB after every ~35 min batch; restarted before each.
