# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-01 morning: old-craft constants in the shuttle's chain").

Last written **2026-10-01 ~10:50**, spaceplane, session in progress (the
user asked: keep going until the spaceplane lands mostly reliably). Code
defaults unchanged from `b5de541e` apart from new flags, **all off**.
Offline spaceplane suite OK. Farm **up** (4 instances, restarted 08:28),
batch `rot-chain2-1001` flying.

**The user's standing direction this session: replace fitted constants with
curves or general solutions — never re-fit a constant for the shuttle**
(memory `constants-to-curves`).

## Where it stands

Goal not reached yet. Twin-fin shuttle (`qs_shuttle2`) from orbit:
- clean-box baseline (keep-200 stack, `rot-base-1001`, LOG4320-4323): 0/4 on
  the runway, exits -1.8/+4.2/+1.8/+1.5 km.
- the derived chain (`rot-chain-1001` round 0, LOG4336-4339): **3/4 intact**,
  touchdown 55-66 m/s, flare 76-85 m/s, cross at flare within 100 m — but
  off the runway on energy (-4.9, -1.3, +1.1 km): the cone ran short at
  1.0-1.5 L/D because it held 108 m/s *true* airspeed at 6-10 km (alpha
  17-22). Fixed by `HAC_POLAR_SPEED` + density-scaled `polar_speed`, now
  flying as `rot-chain2-1001`.

**Latest (10:50):** `rot-chain3-1001` put 4/8 cone exits within +425 m
(baseline +-1.5-4 km). `rot-chain4-1001` round 0 (swing threshold 12):
LOG4375 exited -55 m, on the centreline, touched down inside the runway's
length -- and broke at 90 m/s because the final aims at
`TOUCHDOWN_AIM_M`=2400, the far threshold (crossed the midpoint 840 m up).
`TOUCHDOWN_AIM_DERIVED` (zone less flare float, 42 m for the shuttle) is in
`rot-chain5-1001`, flying now. LOG4372 landed 31/31, 854 m from the
midpoint but 342 m off the centreline.

## Flags built this session (all off)

| flag | what | status |
|---|---|---|
| `HAC_WEAVE_HELD` (+`_HOLD_S`, `_BANK_DEG`, `_ROLL_RATE_DEG_S`) | weave swing = reversal + hold; angle from the swing's effective ratio | old weave flew 0.76-0.80 whatever it commanded; sim 0.73 vs 0.73 |
| `FUEL_TRIM_TRANSFER` (+5) | pump LF/Ox nose<->aft against interval-mean alpha error, Mach 4 -> residual drain | works in game; Mach 1.5-4 tracking sd 1-3.5 on 2 of 4 (was 5-10) |
| `HAC_LD_MEASURED` (+`_TAU_S`) | cone ratio = table ladder x measured/table L/D | ldk 0.91-1.15: table is right for this craft |
| `HAC_AIM_DERIVED` | entry aim from the wings-level ladder (3.20 -> 32 km before the gate, was 13.5) | in chain |
| `APPROACH_POLAR_SPEED` (+`_STEP_M_S`) | approach speed whose 1 g L/D = ratio still needed | flare 76-85 m/s (was 85-95) |
| `HAC_POLAR_SPEED` | the same law in the cone | flying in chain2 |
| `ALPHA_RATCHET_ON_SWING` (+`_TAU_S`, `ALPHA_SWING_TOL_DEG`=12) | ceiling backs off on a sustained swing, not only a deficit | at 6 deg it ate hypersonic drag; 12 in chain4/5 |
| `TOUCHDOWN_AIM_DERIVED` (+`TOUCHDOWN_ZONE_FRACTION`) | final aimed at the touchdown zone less the flare float | in chain5 |
| `HAC_POLAR_SPEED` | cone speed off the polar | **refuted** (stepping target, phugoid) |
| `GLIDE_ALPHA_PLATEAU=0.05` (existing, was sim-only) | ceiling at far edge of lift plateau | on this craft it is 41 deg even transonic — see open item 1 |

Chain = stack (`HAC_ENERGY_BUDGET HAC_PATH_WRAP_TO_GATE HAC_PAST_BEFORE_GATE_DEG=60
DRAIN_RESIDUAL FUEL_TO_NOSE APPROACH_BANK_BY_ROLL APPROACH_FLARE_FACTOR=1.45
HAC_ENTRY_AFFORDABLE HAC_LAP_AT_TARGET_SPEED HAC_RADIUS_MIN_M=1000
HAC_WEAVE_MAX_DEG=75`) + all of the above. Exact string in
`logs/rot-chain2-1001.txt`.

## Open, in order

1. **Transonic glide departures.** Mach 0.6-1.5 the glide sits on its +500 m
   reserve by pinning alpha at 41 and reversing bank +-40; the shuttle
   wallows 3-66 deg, slip to 22. Lift is flat 24-50 deg here (measured and
   table agree), so the lift plateau is the wrong criterion; control is.
   `ratchet_alpha` only learns from falling *short*, never from overshoot or
   oscillation. Candidate: a learned ceiling that also lowers on oscillation
   amplitude / sideslip, so the energy is left to the cone.
2. Landing chain inside +-0.8 km exits: flare speed still 76-85; touchdown
   >70 m/s breaks the craft.
3. Re-check the old craft (`qs_plane`) on anything adopted.
4. **`qs_shuttle2_gate` is not a valid bench** (loads into half lift,
   elevons saturated; 12/12 identical dives, `rot-gate-1001`). Don't use it;
   a valid on-final bench is still missing.
5. Hypersonic alpha excursions >15 deg at Mach 3-5 on most flights: 5/20
   follow an RCS valve opening (Mach 5+, unreachable 40 deg command); 15/20
   are pitch-ups with the bank steady -- the airframe's pitch margin. **Ask
   the user about a craft fix** (forward ballast / canard).

## Traps paid this session

- **Sims beside the farm**: load 23 on 16 cores; 4 of 4 sim flights departed.
  Don't. And **simarms.sh leaves its servers running**: four idle kspSim
  servers sat beside every game batch from ~07:45 to 11:00 today (1.6 GB of
  swap; killed before `rot-chain6-1001`).
- A re-fit arm (`HAC_GATE_LD=2.4;HAC_LD=2.4`, LOG4333/4335) was flown once as
  a gain check; not a candidate value.
- Void: LOG4340-4343 (killed at ~30 s, rot-chain-1001 round 1).
- Killing the rotfly *driver* only (not its subshells) stops further rounds
  and lets the running flights land.
