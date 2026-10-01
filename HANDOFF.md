# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-01 morning: old-craft constants in the shuttle's chain"
and the afternoon/evening entries after it).

Last written **2026-10-01 ~17:15**, spaceplane. Code defaults unchanged from
`b5de541e` apart from new flags, **all off** (fingerprint of the flown tree
`d02fcccb`). Offline spaceplane suite OK (674). Committed. Farm **stopped**,
inhibitor **released**, no sims running.

**The user's rules from this session** (also in memory / CLAUDE.md):
- replace fitted constants with curves or general solutions -- never re-fit
  one for the shuttle (`constants-to-curves`);
- once a batch starts, end the turn and wait for its end notification; no
  polling, no mid-flight reads (CLAUDE.md, session procedure);
- before blaming the airframe, compare the other craft on the same code and
  show the actuator saturated (`compare-craft-before-blaming-airframe`).

## Where it stands

Goal ("lands mostly reliably") **not reached**. Best configuration so far is
**chain6** (exact string in `logs/rot-chain6-1001.txt`): 3/8 intact from
orbit (LOG4385 28/31 at 38 m/s, 199 m from the midpoint; LOG4387, 4388
31/31), but all off the centreline (416-2503 m). Clean-box baseline
(`rot-base-1001`, last night's stack): 0/4 on the runway.

Two things now dominate, in this order:

1. **High-alpha lateral departures in the glide, Mach 2.5-6** (the "pitch-up").
   On ~half of all flights, **both shuttles** (paired `rot-shuttle2-0930`:
   single-fin 4/6, twin-fin 2/6). Mechanism read on LOG4409/4404: a bank
   reversal or roll overshoot at 35-40 deg of alpha builds 20-50 deg of
   sideslip, the flown bank goes the wrong way (+7 -> +107 -> -95 against
   -31 commanded), the roll damper flags swings to 178 deg, and only then
   does alpha go to 50-88. Failure 99's coupling. It throws the glide 10-40 km
   off and nothing downstream recovers that. Precursors over 71 flights: RCS
   valve open (most), residual drain dumping nose fuel (16, only in configs
   that dump at Mach 3.5), fuel trim aft, reversals.
   **Not the fix:** pitch RCS (`RCS_PITCH_BY_AUTHORITY`, 2/4 still departed
   vs 1/4, `rot-rcsgate-1001`); forward ballast (8.4 t of fuel kept in the
   nose, `rot-ballast3-1001` round 0: no departures in 2, but too nose-heavy
   to reach the commanded alpha -- a different regime).
   **Next:** find, from the logs, the alpha (vs Mach) where roll couples into
   sideslip on this body (slip build-up against alpha), make the glide
   ceiling and the reversal law respect it, and look at why the RCS valve
   opens there and what yaw/roll thrust does to the roll.
2. **The landing chain on good arrivals**: lateral capture near the ground
   (LOG4385 touched down 20 deg off the runway heading: the capture
   commanded level with 27 deg of bank still on), and flare entry speed
   (66-95 m/s; the craft breaks above ~70 at touchdown).

## Flags built this session (all off)

| flag | what | status |
|---|---|---|
| `HAC_WEAVE_HELD` (+3) | weave swing = reversal + hold, angle from the swing's effective ratio | **keep**: delivered ratio now matches commanded (was 0.78 whatever) |
| `HAC_LD_MEASURED` (+1) | cone ratio = table ladder x measured/table L/D | in chain6; `ldk` 0.91-1.15 |
| `HAC_AIM_DERIVED` | entry aim at the harmonic mean of the cone's steepest and flattest ratios (1.79 shuttle) | in chain6 |
| `HAC_SPEED_EAS` (existing) | cone speed as equivalent airspeed | in chain6; fixed the cone flying near stall at altitude |
| `APPROACH_POLAR_SPEED` (+1) | approach slows toward best glide when low (one-sided) | in chain6 |
| `TOUCHDOWN_AIM_DERIVED` (+1) | final aimed at the touchdown zone less the flare float (42 m) | in chain6; replaced the far-threshold aim that dove LOG4375 into the runway |
| `FUEL_TRIM_TRANSFER` (+5) | pump LF/Ox nose<->aft against interval-mean alpha error, Mach 4 -> drain | in chain6; mechanically verified |
| `ALPHA_RATCHET_ON_SWING` (+2, tol 12) | ceiling backs off on sustained swing | in chain6; at tol 6 it ate hypersonic drag |
| `GLIDE_ALPHA_PLATEAU=0.05` (existing) | ceiling at the far edge of the lift plateau | in chain6; 41 deg even transonic on this body |
| `RCS_PITCH_BY_AUTHORITY` (+1, latched) | pitch thrusters off once surfaces out-torque them | **null** (n=4) |
| `HAC_POLAR_SPEED` | cone speed off the polar | **refuted**: stepping target, phugoid |
| `APPROACH_ENERGY_EXCESS` | approach counts kinetic energy over the door | **refuted**: lands 1.7-2.6 km short |
| `APPROACH_HEADING_LEAD` (+1) | capture leads by the roll-out | unproven (confounded with the above) |

Tools: `spaceplane/tools/gatesave.py` (save at an on-profile cone
handover; `--after N`). Bench saves `qs_shuttle2_low_{m30,p30,p60}`,
`qs_shuttle2_gate(_p15,_m15)`.

## Traps paid this session

- **`qs_shuttle2_gate` is not a valid bench**: loaded in the air the vehicle
  makes half its table lift (elevons saturated) and dives -- 12/12 identical
  (`rot-gate-1001`).
- **kspSim servers left running** by `simarms.sh` sat beside game batches
  ~07:45-11:00 (paid twice now; the memory's check would have caught it).
- I claimed the shuttle had no canards (read part *names*; two Big-S Elevon 1
  sit on the nose) and blamed the airframe for the pitch-ups before comparing
  the single-fin craft. Both wrong; see the memory above.
- Void logs: LOG4340-4343, LOG4380-4383 (killed at ~25 s).
- Killing a rotfly *driver* (not its subshells) stops later rounds and lets
  the running flights land. `pkill -f` patterns can match your own shell.
