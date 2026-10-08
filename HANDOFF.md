# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-08 morning (~0700-0800).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-08 morning: the
cone's exit quantum"; the 2026-10-07 late split-S session is above it.

Defaults fingerprint **`38b4f3b6`** (= `d5e851b4` + `HAC_GATE_STRETCH`
off; **no default changed**).  Everything committed and pushed.  Farm
**stopped**, sleep inhibitor released.  Swap after the last batch: 24.6 GB
(restart before any comparison, as always).

## This session (the user: "fix the cone handover") -- not fixed

**`HAC_GATE_STRETCH`** (off; `Autopilot.hac_gate_stretch`,
`guidance.gate_alt`, `end["gate_stretch"]`): while the cone plan reads
surplus, the rollout moves out along the extended centreline and the gate
rises by the approach's glide, so a sub-lap surplus becomes a longer
final.  Steps only if the re-plan owes no lap, isn't short, doesn't wrap,
and **raises the plan's need** (without that last test it grew at turn 0,
only repricing cone path at the approach's 4.2 -- kspSim LOG8459/8463/8465,
rollout +1.2-1.8 km worse).

**Farm, sav-stretch-1008 (12 v 12, qs_s2_hac0-5): null.**  Engaged on 4,
stretch 0.2-1.35 km, rollout surplus unchanged (+1.2..+2.3 km).  **Why:**
the high saves enter the cone *straight in* -- on the centreline outside
the gate, turn ~2 deg, 11 km out, +2.7 km surplus (LOG8477).  No gate
position adds path there; only a turn away does, and that is the lap.
Third null mechanism on the sub-lap surplus (sharp turn, spiral, stretch):
**change the method.**

## Best configuration (not yet a default) -- unchanged

    GLIDE_PITCH_OFFLOAD=True GLIDE_PITCH_OFFLOAD_MAX=0.6
    GLIDE_PITCH_OFFLOAD_MIN_MACH=3 CANARD_TRIM=True HAC_LD_MEASURED=True

`qs_shuttle2_rigoff` from orbit: 31/48 on the runway (65%) over the evening
batches; tonight's three control rounds of the same configuration
(rot-sharp1/2/3-1007, 18 flights) landed 14/18 on the runway, two lost.
Inclined/high orbits land long with it, and the eccentric orbit arrives
41 km long (upstream) -- see the journal's evening section.

## Next, in order

1. **The cone's sub-lap surplus, by another method.**  (a) Upstream: the
   entry aim (`high_gate`, `HAC_AIM_DERIVED`) puts arrivals on the
   straight-in line by design; aim high arrivals *onto the circle's far
   side*, where radius and stretch both have authority.  (b) Drag at the
   cone's speed: `HAC_FLAP_BRAKE_ON_SURPLUS` (+`_IGNORES_ROLL`,
   `HAC_FLAP_ARREST_EXCESS`), `BAY_BRAKE`, propellant trim -- each off for
   its own recorded reason; re-read why before flying.  hac4 (rolls out at
   turn 336 with +4.1 km) and hac0/hac5 ("out of height" at turn 190-320)
   are separate shapes -- read them first.
2. **kspSim's cone reads short where the game reads high** (sim LOG8473:
   plan L/D 0.85-0.97 through the cone, rolled out +1.3 km).  The cone's
   `HAC_LD_MEASURED` scale or the sim's subsonic polar; matters before
   screening any cone work in the sim.

3. **Touchdown breakups on the cone saves** (flare entered at 50-68 m/s,
   15-25 parts left, every arm).  Read `KSP.log` for which parts explode
   first (memory: ksp-log-explosions); the rigoff orbit flights do not
   show it.
4. **The approach's speed target is above what the shuttle flies**
   (held 110-115, flown 73-100) and it often commands alpha -2 while flying
   +4.5 (LOG8359): the kRPC tracking problem in the approach.  The sharp
   turn is gated on the door speed because of this.
5. Carried from the evening: runtime crossrange footprint (the user's
   request); the other orbits with the canard trim; cap the propellant pump
   at the stock transfer rate; `TOUCHDOWN_AIM_M` per vehicle.

## Traps paid (this session first, then 2026-10-07 late)

- **A gate moved toward a lined-up vehicle "spends" nothing**: it swaps
  path priced at the cone's ratio for the approach's.  Any re-plan that
  changes which ratio prices a leg must be judged by the plan's need, not
  its surplus.
- `pgrep -f KSP_x64` matches its own shell; count with
  `ps -eo args | grep -c "^[^ ]*KSP_x64"`.

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
