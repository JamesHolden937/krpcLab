# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-07 late (~2200 to 0015 on 10-08).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-07 late: the
user's split-S"; the evening before it is the section above that.

Defaults fingerprint **`d5e851b4`** (= `386549dd` + new off flags; **no
default changed**).  Everything committed and pushed.  Farm **stopped**,
sleep inhibitor released.

## Best configuration (not yet a default) -- unchanged

    GLIDE_PITCH_OFFLOAD=True GLIDE_PITCH_OFFLOAD_MAX=0.6
    GLIDE_PITCH_OFFLOAD_MIN_MACH=3 CANARD_TRIM=True HAC_LD_MEASURED=True

`qs_shuttle2_rigoff` from orbit: 31/48 on the runway (65%) over the evening
batches; tonight's three control rounds of the same configuration
(rot-sharp1/2/3-1007, 18 flights) landed 14/18 on the runway, two lost.
Inclined/high orbits land long with it, and the eccentric orbit arrives
41 km long (upstream) -- see the journal's evening section.

## What this session did (the user: "do a split S, the sharp turns should kill the speed")

**A literal split-S was not flown.**  Inverted, it turns height into speed,
and speed is what destroys the high handovers.  The shuttle rolls 14 deg/s
(~13 s to get inverted), and the heading reversal costs ~1-2 km more height
than the surplus.  The sharp-turn half was built twice:

1. **`APPROACH_SHARP_TURN`** (off, `guidance.sharp_turn`): in the S-turn,
   the drag that spends the surplus by the weave's stop chooses alpha
   (polar: CdA 18.6 at 2 deg -> 92 at 16).  The vertical lift is that of a
   steady descent at the present speed (`sin gamma = D/W`, cap 25 deg), and
   the rest is banked off.  The lean is predicted over the roll reversal,
   heading limit 45.  Final version, rot-sharp3-1007: 6/6 vs 5/6 on the
   runway, but it engaged in only 3 flights, for a few ticks --
   **unmeasured**.  Two refuted vertical-lift references are recorded in
   its docstring (LOG8374 landed backwards; LOG8377 spent speed, not
   height).
2. **`HAC_SPIRAL_DUMP`** (off, `Autopilot.hac_spiral`): measured-cost tight
   360s over the gate when the cone lines up with surplus between the
   approach's allowance and a lap.  **One right handover (LOG8444, hac4:
   +62 m vs +14 km without; broke on touchdown) and two spiral dives
   (LOG8432, LOG8450, destroyed).**  A lap costs ~3.9 km at any speed (the
   radius at the alpha cap is speed-independent).  `HAC_SPIRAL_ALPHA_MAX_DEG`
   20 did not shrink it and dived.  hac1/hac3 (~1.9 km over) never start.
   Three engagements on one mechanism: change the method, not the value.

**The finding that matters more:** on the cone saves `qs_s2_hac0-5` with
`CANARD_TRIM`, the cone **rolls out lined up 4-6.8 km high** (need 2.2)
and both arms land +3..+14 km long (sav-sharp-1007: runway intact 1/11 vs
0/11).  `hac_exit_surplus` hands over anything short of a whole lap
(`2 pi R / cone_ld` ~8 km at R 2 km), and the approach spends ~0.8.  This is
the same shape as the inc/high orbits landing long with the canard trim.

## Batches (all in logs/)

| file | what | result |
|---|---|---|
| rot-sharp0-1007 | rigoff, base vs sharp (gate on held speed) | sharp engaged 1 tick; 5/6 intact on runway overall |
| rot-sharp1-1007 | sharp, door gate, speed-law lift | 4/6 vs 4/6; LOG8374 backwards 7 km short |
| rot-sharp2-1007 | sharp, one-g lift | 4/6 vs 5/6; LOG8377 +2.8 km into the sea |
| rot-sharp3-1007 | sharp, steady-descent lift (final) | 6/6 vs 5/6, barely engaged |
| sav-sharp-1007 | cone saves, 4 rounds | cone rolls out 4-6.8 km high; ~nothing lands |
| sav-spiral-1007 | + spiral, 60 deg fixed | 1 engagement, spiral dive (LOG8432) |
| sav-spiral2-1007 | spiral, sustainable bank | LOG8444 +62 m (broke) |
| sav-spiral3-1008 | spiral cap 20 vs 14 | LOG8450 spiral dive |

## Next, in order

1. **The cone's exit quantum.**  Spend the cone's surplus *before*
   rollout, continuously: let the cone's own plan include a tight measured
   lap (or part of one) when surplus exceeds the approach's allowance,
   instead of handing over up to a lap's worth.  Or cut
   `hac_exit_surplus` so the cone flies the lap at its own radius.  Fly it
   on `qs_s2_hac1/3/4` (the high ones) and rigoff from orbit.  This is
   probably also why inc/high land long with the canard trim.
2. **Touchdown breakups on the cone saves** (flare entered at 50-68 m/s,
   15-25 parts left, every arm).  Read `KSP.log` for which parts explode
   first (memory: ksp-log-explosions); the rigoff orbit flights do not
   show it.
3. **The approach's speed target is above what the shuttle flies**
   (held 110-115, flown 73-100) and it often commands alpha -2 while flying
   +4.5 (LOG8359): the kRPC tracking problem in the approach.  The sharp
   turn is gated on the door speed because of this.
4. Carried from the evening: runtime crossrange footprint (the user's
   request); the other orbits with the canard trim; cap the propellant pump
   at the stock transfer rate; `TOUCHDOWN_AIM_M` per vehicle.

## Traps paid this session

- **A gate on the speed the law *holds* is a gate on a speed this vehicle
  never flies**: `APPROACH_SHARP_TURN` engaged on 1 tick of 3 flights.
  Read the flown speed against the target before gating on either.
- **"Keep the speed law's lift vertical" is not neutral** when the speed
  law is chasing an unreachable target: it asks for no lift, and the whole
  wing went sideways (LOG8374).
- **A turn whose bank is above what the alpha cap can hold is a spiral
  dive** (LOG8432, 8450): derive the bank from the cap's lift.
- savefly/rotfly parse: the grep pipeline in this session's notes misaligned
  arm/LOG pairs once; read each `arm ksp ... LOGn` line whole.
- Swap ~19-20 GB after every 2-round batch; restarted before each.
