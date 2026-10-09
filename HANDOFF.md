# HANDOFF — read this first, rewrite it last

Snapshot of the session of 2026-10-08 afternoon/evening (~1530-1800).
History: `docs/spaceplane/journal.md`, "Session, 2026-10-08
afternoon/evening: the glide's handover energy"; the morning's
`HAC_GATE_STRETCH` session is above it.

Defaults fingerprint **`80930f0e`** (`c9fedee0` + `AERO_REFRESH_NEAR_MACH`
off, + `GLIDE_CONE_ENERGY`, `GLIDE_CONE_CEILING` off; **no default changed**).  Farm up at the time of
writing (stop it at wrap-up); swap ~20 GB after every batch.

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

## Later this session: the cone's mis-pricing is the aero probe

The table is re-probed in flight at the surfaces' *present* deflection, so
the subsonic rows read L/D 0.85-1.72 at cone entry where the cone flies
3.3-3.7 (new `cone ladder` log line; rot-ladder-1008).  The cone plans its
descent on that, reads short, pins the 2 km circle, finds 1-4 km of surplus
below 5 km.  `AERO_REFRESH_NEAR_MACH` (off) only half connects (7/12 v 8/12,
rot-near-a/b-1008): the cone's own high-alpha upper part re-contaminates the
rows.  **Next is item 1 below, re-aimed: price the ladder from the flown
trimmed polar** (`polar.py`; subsonic measured L/D 3.9 / 3.3 / 2.8 / 2.5 /
~1.3 at 0-3 / 3-6 / 6-9 / 9-12 / 15+ deg), not the probe.  kspSim cannot
screen it: its subsonic lift is 1.51x the table where the game's is 0.6x.
Today's rigoff base: 11/18 (the 3/8 dip was noise).

## Next, in order

1. **The cone mis-spends energy it can absorb.**  The default target is
   108 m/s *true* (~65 IAS at 8-14 km): it flies the upper cone at the
   alpha cap (L/D ~1), prices the rest at that (`ldk` 0.7-0.9, `pld`
   <1), reads short, pins R at 2 km; below ~5.5 km alpha drops, `ldk` ->
   1.3-1.45, and 1-4 km of surplus appears with straight-in geometry left
   (LOG8340, 8364, 8345, 8350; scratch `conetrace`).  Make the plan price
   the two regimes it actually flies -- or choose the cone speed so there
   is one regime -- without the EAS failure (sim: radius/weave pin and it
   still rolls out high, 2/4 broke).
2. The prior + ceiling stack for the >20 km tail, once the cone half is
   fixed (it removes most >20 km entries; null alone).
3. Carried: approach speed target above what the shuttle flies (110-115 vs
   73-100), eccentric orbit ~17-21 km long, runtime crossrange footprint,
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
- `pgrep -f KSP_x64` matches its own shell; count with
  `ps -eo args | grep -c "^[^ ]*KSP_x64"`.
- "could not pause after the load" (ksp5, ~1 flight in 5 batches) is the
  intermittent pause race, not a mod mismatch (GameData diff = ksp0's).
