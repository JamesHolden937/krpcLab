# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-08 afternoon/evening (~1530-1940).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-08
afternoon/evening: the glide's handover energy"; the morning's
`HAC_GATE_STRETCH` session is above it.

Defaults fingerprint **`ae54ff05`** (`c9fedee0` + `AERO_REFRESH_NEAR_MACH`,
`HAC_LD_FLOWN_POLAR` off, + `GLIDE_CONE_ENERGY`, `GLIDE_CONE_CEILING` off; **no default changed**).  Farm **stopped**; swap ~20 GB after every batch
(restart before any comparison).

## This session (the user: is the glide handing the cone an unusable energy profile?)

**Partly yes, and it is not the main cause.**
- The cone flies ~27 km of path from *every* entry (26-29 km, 24 farm
  orbit flights); its one energy control is how long it holds the alpha
  cap (path/energy ~1.5 there, ~3.1 below ~5.5 km).  So it absorbs at most
  ~20-21 km of entry energy height; entries above that land 4-6 km long.
- The glide over-delivers because **below Mach 3 it commands ~40 deg and
  the shuttle holds 0.57-0.75 of it** (inc/high least), so the propagator
  predicts too much drag and the entry energy reads 5-7 km low until the
  last minute.
- Built (both off): `GLIDE_CONE_CEILING` (the cone's ceiling replaces the
  ladder band, which was noise: 14-35 km for one state) and a **Mach-banded
  `HOLDABLE_PRIOR`** (`holdprior.py --by-mach --mach 0.8`, file regenerated
  from 138 current-stack logs in untracked `logs/holdprior/`; fly with
  `HOLDABLE_BY_MACH`).
- **Farm, 36 an arm** (rot-prior, prior2, prior3a, prior3b-1008; rigoff/
  inc/high x12): entry energy 18.9 -> 17.4 km, >20 km entries 4/12 ->
  1/12 -- but **landings 21/36 v 21/36**.  Pooled, the stop tracks the
  cone's **rollout surplus** (r 0.74) more than entry energy (0.36);
  16-18 km entries still hand over +250 m mean and 9/31 miss.

## Best configuration (not a default) -- unchanged

    GLIDE_PITCH_OFFLOAD=True GLIDE_PITCH_OFFLOAD_MAX=0.6
    GLIDE_PITCH_OFFLOAD_MIN_MACH=3 CANARD_TRIM=True HAC_LD_MEASURED=True

Today: 21/36 over rigoff/inc/high (6/12, 7/12, 7/12).  Last night rigoff
15/19 on the identical config string, inc/high 3/12.  **The "65%" in old
notes is rigoff only.**  Every code change since 78f93e7 is flag-gated
(read in full); today's rigoff dip is unexplained -- if it persists,
bisect on rigoff alone.

## Later this session: why the cone mis-prices, and what fixing it exposes

- The aero table is re-probed in flight at the surfaces' *present*
  deflection: subsonic rows read L/D 0.85-1.72 at cone entry where the cone
  flies 3.3-3.7 (new `cone ladder` log line, rot-ladder-1008).
  `AERO_REFRESH_NEAR_MACH` (off) only half connects: 7/12 v 8/12.
- **The flown polar** (`tools/conepolar.py` -> untracked `logs/conepolar/`,
  207 logs): lift peaks at **~139 m^2 near 12 deg** (table ~300); the cone
  flies most ticks at 15-17 deg, **stalled**, L/D ~1.2, braking to its 112
  m/s *true* target.
- rot-polar-1008 (4 per config per orbit): base 5/9; **`HAC_LD_FLOWN_POLAR`
  3/12** -- connected (`pld` 2.3-2.5 from entry, R pinned 16 km), but the
  surplus it now sees falls in **the cone's gap** (more than the widest
  no-lap circle, less than a ~12.6 km minimum lap), rollout -1.3..+2.3 km,
  laps=0; **`HAC_SPEED_EAS` 1/12, 4 lost** (out of height mid-turn) --
  refuted on the farm.  Two losses per new arm were the approach diving
  (flare at 102-132 m/s, ~90 m/s sink).
- kspSim cannot screen any of this: its subsonic lift is 1.5x the table,
  the game's ~0.6x.  Today's rigoff base: 11/18 (the 3/8 dip was noise).

## Next, in order

1. **The cone's gap, by a different method** (fourth attempt; sharp
   turn, spiral dump, gate stretch were the same method).  With the flown
   polar the cone knows at entry that it owes more than its widest circle
   and less than a minimum lap.  Candidates: plan the lap at entry when the
   flown-polar plan says the surplus is over the no-lap ceiling (a lap at a
   radius sized to the surplus), or make the entry aim (`high_gate`,
   `HAC_AIM_DERIVED`) deliver energy that lands inside the no-lap band.
   Price with `HAC_LD_FLOWN_POLAR`; fly on the farm only.
2. The approach diving to make speed (flare at 102-132 m/s, ~90 m/s
   sink; lost 5 flights today across arms) -- the approach speed target
   sits above what the shuttle flies (110-115 v 73-100).
3. The prior + ceiling stack for the >20 km tail, once the cone half is
   fixed (it removes most >20 km entries; null alone).
4. Carried: eccentric orbit ~17-21 km long, runtime crossrange footprint,
   `TOUCHDOWN_AIM_M` per vehicle, kspSim's cone L/D.

## Traps paid (this session first)

- **Touchdown breakups don't count** unless a wing/tail strike or a hard
  arrival (the user).  Score on signed stops.
- **Read per-orbit rates, never a blended one against a single-orbit
  figure**: "65%" (rigoff) against 50% (three orbits) looked like a
  regression and wasn't.
- A background command ending in `&` returns at once: the notification is
  the launcher, not the batch -- put a waiter on `ROT DONE`.
- `GLIDE_CONE_ENERGY`'s `long=` at handover includes the energy term: an
  "arrival +8 km" there is partly the term, not position.
- **`HAC_LD_MEASURED` contains the substring "EAS"**: label arms from
  each log's `config:` line, never by grepping the arm string.
- `rotfly.sh` with more arms than instances flies them unevenly (arm =
  round + instance): rotate the arm list yourself (rot-polar-1008).
- `pgrep -f KSP_x64` matches its own shell; count with
  `ps -eo args | grep -c "^[^ ]*KSP_x64"`.
- "could not pause after the load" (ksp5, ~1 flight in 5 batches) is the
  intermittent pause race, not a mod mismatch (GameData diff = ksp0's).
