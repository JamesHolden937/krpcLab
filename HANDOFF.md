# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-07 afternoon/evening (~1430-2200).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-07
afternoon/evening".

Defaults fingerprint **`386549dd`** (= `db682413` + new off flags and one
instrument; **no default changed**).  Everything committed and pushed.
Farm **stopped**, sleep inhibitor released.

## Best configuration (not yet a default)

    GLIDE_PITCH_OFFLOAD=True GLIDE_PITCH_OFFLOAD_MAX=0.6
    GLIDE_PITCH_OFFLOAD_MIN_MACH=3 CANARD_TRIM=True HAC_LD_MEASURED=True

`qs_shuttle2_rigoff` from orbit, three batches pooled: **31/48 on the
runway (65%)**, against same-day offload baselines of 9/30 and ~1/3.  The
offload's -5..-6.5 km short cluster is gone; runway stops within +-930 m
along.  Stage by stage (rot-canard2-1007, 18 flights): arrival at the cone
18/18 within 5 km (+0.8 sd 0.46); cone handover 15/18 within +-0.7 km;
the approach is where the rest is lost.

**It does not carry over to the other orbits yet** (rot-orbits-1007, 6
each, same configuration; measured only, not acted on):

| orbit | runway | misses | arrival at cone | intact |
|---|---|---|---|---|
| `_inc_rigoff` | 1/6 | 5 long +2..+6.4 km | +1.65 sd 0.86 km | 6/6 |
| `_ecc_rigoff` (new) | 0/6 | 5 long +17..+21 km, 3-13 km across | **+41 sd 10 km** | 5/6 |
| `_high_rigoff` | 2/6 | 3 long, 1 short | +0.8 sd 0.6 km | 5/6 |

Previous session's offload-only on these orbits: inc 4/12, high 8/12 --
so on inc/high this configuration looks *worse* (6 vs 12 flights: a
warning, not a measurement).  The more efficient (canard-trimmed) vehicle
lands long there: the energy chain under-plans its L/D.  The eccentric
orbit is an upstream failure (deorbit/entry arrive 41 km long) independent
of today's work.

## What was found (and why)

1. **The cone's short misses were kRPC, not the plan.**  The offload's
   short flights reach the cone like the good ones and lose it inside: alpha
   held 6-11 deg above a ~0-2 command, 7-9 deg nose-down error, pitch input
   unsaturated at +0.3..+0.5 for 30-90 s.  Over 480 flights: that state on
   >=20% of cone/approach ticks -> 3/173 on the runway.  **kRPC's own
   diagnostic log** (`spaceplane/tools/apWatch.py`; `AutoPilot.diagnostic_log`,
   `current_attitude_error`, oscillation latches -- all readable from the
   client) says why: its PID integrators live in a roll-invariant frame
   carried by parallel transport of the nose, and the spiral twists it
   (`phi` 34 deg on a wings-level vehicle).  The standing nose-up trim comes
   out as body pitch +0.29 **and yaw -0.53**, unwinding at 0.002/s.  No
   oscillation mitigation involved.  kspSim does not reproduce it.
2. **Trimming through control travel is this airframe's largest drag.**
   Full nose-up input at alpha 5: -34% lift, +160% drag.  The airframe needs
   700-830 kN m of nose-down moment trimmed out at approach speeds (2 km,
   80-120 m/s, `spaceplane/tools/attitudeProbe.py`).  That and (1) are the cone's L/D 1.2.
3. **The canards are the efficient trim surface** (+10 deg: 60 kN m
   nose-up with +6 kN *more* lift) **and stall**: nose-up effect gone by
   alpha ~24 deg, reversed past it.  That is also the cone's alpha ceiling
   (commands 22-26, flies 15-17 pinned).
4. **Per-surface control exists**: kRPC `ControlSurface.deflection_override`
   + `deflection` (-1..1 -> the deploy-angle limits, +-37.5 deg here).
   Under AtmosphereAutopilot a deployed surface ignores input; stock adds
   the deploy angle to the control deflection.  Never command exactly 0
   (freezes the surface).  No more travel or rate than stock.
5. **Cargo bay doors** are a real speedbrake (+10-30% drag, slight lift
   gain) -- flown as a last resort, null on landings.
6. **Not-cheaty audit** (the user asked): every part write is within stock
   slider limits (brakes 200%, friction 10 = stock max, deploy angles
   <=25 deg, authority 150).  kRPC's `ResourceTransfer` could move 10%/s of
   a tank (stock ~5%/s); our pump asks less -- **add a hard cap at the stock
   rate**.  AtmosphereAutopilot makes surfaces slew ~75 deg/s vs stock 40
   (the user's mod, every flight benefits).

## Flags added (all off unless said)

- `CANARD_TRIM` (+ `_TAU_S` 5, `_LP_S` 2, `_DEADBAND` 0.05, `_MAX` 0.6,
  `_MAX_ALPHA_DEG` 15, `_ALPHA_TOL_DEG` 3, `_LOCAL_MAX_DEG` 28,
  `_MAX_MACH` 0, `_SIGN` +1): the forward mirrored pair off kRPC, driven as
  a trim that integrates the standing pitch input to zero; engages only
  settled under 15 deg of alpha; capped by the canard's local incidence.
  Column `ctrim=trim/standing`.  **The win.**  Smoke LOG8225: standing
  input +-0.08, L/D late cone 3.0-3.7, approach 3.7-4.2.
- `BAY_BRAKE` (+ `_SATURATED` 0.8): doors open **once**, only with the
  cone's radius at its cap (no lap) or the approach's S-turns saturated and
  >800 m to spend; shut when spent, never reopened (**the user: only if
  necessary, no cycling** -- memory bay-doors-last-resort).
  rot-canbay-1007: 11/18 vs 12/18.  Null; keep off.
- `PITCH_P_CONE` -- refuted (8/18 vs 6/18; kRPC winds up against it).
- `CONE_TRIM_HANDOFF` -- refuted (4/18 vs 5/19, 2 lost: a captured trim
  goes stale).
- `GLIDE_BANK_PROBE_DEG` (+ `_END_MACH` 1): **instrument**: hold one bank
  COAST..Mach 1 and log north/east of the runway at Mach 8..1.

## Crossrange, measured (rot-xrange-1007, rigoff, LOG8334-8339)

**+bank = left** (north on this eastbound orbit).  At Mach 1:

| bank held | sideways | downrange (east of rwy) | alt |
|---|---|---|---|
| 45 left (2) | 57, 66 km | -34 km | 16-20 km |
| 45 right (2) | 70, 72 km | -41 km | 15 km |
| 60 left | 94.5 km | -88 km | 14.6 km |
| 60 right | 92.5 km | -84 km | 14.4 km |

Plus the subsonic glide from 14-20 km (L/D ~3-4 now): total lateral reach
probably ~130-150 km.  Downrange is bought back by burning later (the
user's point; the deorbit search needs to plan with the steep-bank
profile).

## Next

1. **Runtime crossrange** (the user's request for next session): propagate
   from the current state with the bank held at the left/right limit to
   Mach 1 (and on to the ground) and report the reachable footprint --
   before the burn (which pass of an inclined orbit is reachable, so it
   need not wait for a near-perfect one) and during entry (lateral authority
   left).  Log prediction beside the `GLIDE_BANK_PROBE_DEG` flights and
   measure its accuracy against LOG8334-8339 and the inclined orbit.
2. **Spend surplus height without making speed** (the approach): 4 of 17
   misses in the canard arm are losses (handed over ~1 km high, S-turns
   pinned a minute, then the approach *dives* the excess into the flare
   door at 52-93 m/s of sink: LOG8253, 8265, 8268, 8282), 7 are long.  The
   drag control is alpha (L/D ~4 at 3 deg, ~2 at 15-20) now that the
   canards carry the trim -- let excess select a draggier alpha (inside the
   canard's stall) before a steeper path, and never dive past what the
   flare can arrest.  Inverted flight was probed and is **not** a brake
   (only 8-47% more drag at 1 g).
3. **The other orbits first**: why inc/high land long with the canard
   trim (cone handover surplus? `HAC_LD_MEASURED` vs flown ratio? approach
   float?) -- 12+ per arm against offload-only on the same orbits; and the
   eccentric orbit's +41 km arrival (deorbit window / propagator on a
   radial-perturbed orbit; never flown before).  Promote candidates once
   the other orbits agree: `CANARD_TRIM` +
   `HAC_LD_MEASURED` (with the offload).  Fly `qs_plane` as the regression
   check (it has no canards: `CANARD_TRIM` logs "no forward mirrored pair"
   and stays off).  Generality: the canard sign and stall limits were
   measured on this craft -- derive them (probe at engage) before the
   cargo/big-wing variant.
4. Cap the propellant pump at the stock transfer rate.
5. Carried: `TOUCHDOWN_AIM_M` per vehicle; the glide's energy target from
   the cone's need.

## Traps paid this session

- **The live tree was edited mid-batch** (the probe, rot-canbay-1007):
  inert, but it split the fingerprints (12 logs `0d31fb12`, 24
  `386549dd`).  Develop in a `git worktree` while a batch flies.
- kRPC's override maps +deflection to the deploy-angle limits; the first
  canard smoke (LOG8224) departed at alpha 86 by learning the cone-entry
  transient as trim -- not a sign error.  Probe the surface across alpha
  before driving it.
- `simulate_aerodynamic_torque_at` takes a 5th argument (angular
  velocity).  Probes that unpause on the farm must set the instance to 1x
  first (`tools/timescale.py N 1`), or the vessel falls 8 km mid-probe.
- The knife-edge part of `spaceplane/tools/attitudeProbe.py` read zero side force: sign error in
  the probe, no result.
- Swap 20-21 GB after every ~45 min batch; restarted before each.

## Diagnostic tools added (spaceplane/tools/, scratch-grade)

- `apWatch.py N OUT [secs]`: passive observer of kRPC's auto-pilot on
  instance N below 15 km; dumps `diagnostic_log` CSVs when the nose sits
  >3 deg off target.  Run beside a flight.
- `surfaceProbe.py N SAVE`: a surface group (`ONLY`, default canards)
  deployed +-15 deg against alpha 5..38 at 120 m/s (set the instance to 1x).
- `attitudeProbe.py N SAVE`: upright vs inverted (vs knife-edge, broken)
  drag at 1 g.
- `bayProbe.py N SAVE`: cargo bay closed vs open.
- `stuckSum.py logs/rot-*.txt`: the stuck-pitch share per flight against
  the outcome bucket.
