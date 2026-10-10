# HANDOFF — read this first, rewrite it last

**Interim snapshot, 2026-10-10 ~0000 (session in progress).**  Defaults
fingerprint **`ca5fef06`**, committed and pushed.  Farm UP; the smoke
batch `logs/rot-smoke-azf-1010.txt` (AZIMUTH_FLOOR_FROM_TURN, 6 flights,
4 orbits) was flying when the session paused -- read it with
`/tmp/.../glideend.py`-style reversal counts (last-60-s bank sign changes,
max |bnk|, max |slip| before GLIDE -> HAC) and landsum.

## This session (2026-10-09 night)

- Probes: split rudder at 38 deg adds +32% drag at Mach 0.8, +27% at 1.0,
  +16% at 1.5, +12% at 2, +3% at 4 (`splitprobe.py 0 --glide`).  The
  pitch-neutral lift dump at hypersonic speed is 4-5% of lift
  (`spoilerprobe.py`): not worth building.
- `GLIDE_SPLIT_BRAKE` **deleted**: 4 of 6 departed (alpha 85, slip 43,
  bank 165) within ~15 s of the fins deploying at Mach 2.2
  (rot-smoke-v2-1010, LOG9273-9278).
- **Promoted** the brakes-before-weave package (HAC_SPLIT_BRAKE,
  BRAKES_BEFORE_WEAVE, HAC_WEAVE_AFTER_BRAKE, approach fades 0.3/0.15):
  rot-v2-1010, 24 an arm interleaved over 4 orbits, **22/24 v 16/24 on the
  strip and intact**, long 1 v 5, intact 24 v 22; ecc 8/8 v 4/9.  Deleted
  `hac_flap_brake`, `command_airbrake`, `HAC_FLAP_ARREST_G`.
- Found: **every glide ends in a lateral limit cycle** -- ~6 bank
  reversals in the last 60 s, flown bank 85-180 deg, slip 10-25 deg (36/36
  default flights).  Driver: `_bank_sign`'s azimuth deadband at its 0.5
  deg floor against a ~3 s roll lag.  `GLIDE_SETTLE_ON_FLOWN` (settle the
  reversal on the flown bank) was null on the mechanism (6 v 6) and is
  deleted.  `AZIMUTH_FLOOR_FROM_TURN` (off; band >= g tan(bank)/v x roll
  damper tp, capped 12 deg) is in smoke.
- Found: the cone **never plans a lap** (laps=0 in 36/36): a lap costs
  ~22 km of path, so available path between ~23 and ~45 km cannot be
  planned and the surplus goes to brakes/weave.  Cone entry fires on
  `HAC_ENTRY_DIST_M` 3 km out while the glide dives at ~37 deg, i.e. at
  14.6-18 km instead of 12; exit height follows entry height at ~0.65.

## Next

1. Read the azf smoke; if clean, `multirot.sh azf-1010 4` with arms
   defaults v `AZIMUTH_FLOOR_FROM_TURN=True` x 4 orbits (12 arm strings,
   rigoff and ecc doubled).  Promote or delete.
2. The cone's lap gap: a fractional lap / wider radius range, or enter
   the cone on the glide's predicted gate height rather than 3 km out.
3. `APPROACH_CAPTURE_TAU_SHARE=0.35` (off-strip), still unflown.

---
Previous snapshot (2026-10-09 evening) follows for its traps and history.

Snapshot of 2026-10-09 evening (~1630-2010).  History:
`docs/spaceplane/journal.md`, "Session, 2026-10-09 evening".

Defaults fingerprint **`b9f653fc`** (no default changed today: every new
mechanism is a flag, off).  Committed and pushed.  Farm stopped, sleep
inhibitor released.  No worktrees open.

## The headline

1. **The old `qs_shuttle2_ecc_rigoff` was an invalid orbit** (periapsis 69
   km, inside the 70 km atmosphere -- the user caught it).  Everything 10-08
   and 10-09 said about "the eccentric orbit's post-burn flip" is about
   that invalid save.  Regenerated at **77 x 135 km**; it now lands like the
   others (rot-ecc2-1009: 12/18 within the runway's length).
2. **Brakes before weaving** (the user's direction): spend surplus with the
   split rudder first (no attitude change), the spoiler second (yields to
   roll), and bank/weave/S-turn last.  Built as flags, partly flown.

## Where it stands (defaults)

rot-ecc2-1009 (fresh farm, 36 flights): **27/36 within the runway's length,
22/36 also on the strip and intact**; ecc 12/18, rigoff/inc/high 5/6 each.
rot-fade-1009: defaults 21/24 (18/24 strip+intact).  rot-pkg-1009 (late,
swap 21-22 GB): defaults 12/18.  **Batch-to-batch drift is large; compare
only arms interleaved in one batch.**

Failure causes (114 default flights, `landsum.py` + log reading):
- **Long** (~20%): the cone hands over +650..+1200 m high; the approach's
  split brake stowed on its sink cap -> clean airframe accelerated (92 ->
  118 m/s) -> speed law pulled up -> crossed the threshold ~1000 m up
  (LOG9013).  The cone's on/off flap brake never fired (0/36).
- **Short** (~4%): handed over 300-530 m low; nothing can recover it.
- **Off-strip** (~7%): flare entered 100-200 m off the centreline.  The
  approach capture closes at a constant rate timed to finish *at the
  wheels*; the S-turn puts the offset there (no S-turn -> 35 m mean
  cross at the flare, 0 off-strip; heavy S-turn -> ~100 m).

## Flags added today (all off) and what they measured

| flag | what | measured |
|---|---|---|
| `APPROACH_SPLIT_SINK_FADE` | approach brake fades over its sink cap instead of stowing; holds while fast | rot-fade-1009, 24 an arm: 19/24 v 18/24 strip+intact, shorts 3 v 1 -- **null alone**; kept only inside the package |
| `APPROACH_SPLIT_SLOW_FADE` | same for its speed gate | only inside the package |
| `HAC_SPLIT_BRAKE` | the cone's split rudder, throttled to the L/D factor that spends the height to the gate; cone L/D measured clean, plan flies the braked one | package v1 |
| `BRAKES_BEFORE_WEAVE` | spoiler throttled on the surplus the rudder leaves (yields to roll, stowed for the flare); approach S-turn only once both brakes are at their stops | package v1 |
| `HAC_WEAVE_AFTER_BRAKE` | the cone's weave waits for the rudder to saturate (or fade out at low speed) | **unflown** (v2) |
| `GLIDE_SPLIT_BRAKE` | split rudder below Mach 2.2 in the glide, angle integrating the bank the solve spends past 30 deg; every propagation prices it | **unflown**; `GLIDE_SPLIT_DRAG` below Mach 2 is interpolated -- measure first |
| `APPROACH_CAPTURE_TAU_SHARE` | lateral capture as an exponential (share of time left) instead of constant rate to the wheels | **unflown**; try 0.35 |

**Package v1** (`HAC_SPLIT_BRAKE`, `BRAKES_BEFORE_WEAVE`, both fades 0.3 /
0.15), rot-pkg-1009, 18 an arm, stopped after 3 of 4 cycles: **12/18 v
12/18 strip+intact** (13 v 12 within the runway's length); every package
flight stopped within 28 m of the centreline (defaults up to 109 m), but
long 3 v 4.  Smoke (rot-smoke-pkg-1009) 6/6.  Why not more: in the cone
the weave keyed on 800 m of *path* surplus (~270 m of height) and spent it
before the rudder's slew got there (weave 34-95 ticks, rudder 0-26 deg) --
which is what `HAC_WEAVE_AFTER_BRAKE` fixes.

Probe: `splitprobe.py --glide` -- the split rudder at 38 deg adds +1% drag
at Mach 6-7.5 (alpha 35-40), +3% at Mach 4, **+12% at Mach 2**; yaw/roll
0.  Shadowed at high alpha; worth something only below ~Mach 2.

## Next, in order

1. **Probes (farm up, one instance, minutes):**
   `spaceplane/tools/splitprobe.py 0 --glide --angles 0,20,38` (now
   includes Mach 0.8/1.0/1.5 -> fix `GLIDE_SPLIT_DRAG`) and
   `spaceplane/tools/spoilerprobe.py 0` (is a pitch-neutral lift dump
   worth anything at hypersonic speed against the elevon margin the glide
   lacks? the user's question; expectation: no).
2. **Smoke v2** on all four orbits: package v1 + `HAC_WEAVE_AFTER_BRAKE`
   (+ `GLIDE_SPLIT_BRAKE` once its table is measured).  Check the logs:
   cone `spb=` should reach the 30s before `wv=` goes non-zero; `glide
   split brake out` below Mach 2.2; no Tracebacks.
3. **Fly v2 against the defaults**, 4 orbits interleaved,
   `spaceplane/tools/multirot.sh TAG 4 ...` (24 an arm, ~80 min on a
   fresh farm).  Promote the package if it wins (and delete
   `hac_flap_brake`/`command_airbrake`, which it replaces); delete it if
   not.
4. `APPROACH_CAPTURE_TAU_SHARE=0.35` on the winner (off-strip).
5. Upstream: the cone exit height scatter (+-1 km) is the root of long
   and short.

Arm string for v2:
`LOG_INTERVAL_UT=0.5;HAC_SPLIT_BRAKE=True;BRAKES_BEFORE_WEAVE=True;HAC_WEAVE_AFTER_BRAKE=True;APPROACH_SPLIT_SINK_FADE=0.3;APPROACH_SPLIT_SLOW_FADE=0.15`

## Repo changes today (not flight code)

- Journal pruned 315 -> 130 KB (full text at `f8d63c5`).
- Untracked what regenerates: plugin DLLs (`mkbase.sh` builds all three
  with `mcs`), 11 savegen saves (`saves/derived.txt`; `syncSaves.sh
  check|push` rebuilds them byte-identical), `verify*.txt`.
- `setup.sh` (asks for the KSP install, checks mods, venvs, derived saves,
  optional farm), GPL-3.0-or-later `LICENSE`, `CODE_OF_CONDUCT.md`,
  `CONTRIBUTING.md`.  Claude pushes directly to `main`; branch
  protection was not set (the classifier blocked it; the user can do it
  in GitHub settings).
- The user's live KSP install lacks `PersistentThrust`, which the farm's
  `base/` has (setup.sh warns).

## Traps paid today

- **Check a save's orbit before believing anything flown from it**
  (periapsis vs atmosphere depth): two sessions went into an invalid save.
- "could not pause after the load" hits ~1 flight in 15; all instances run
  the same time-scale DLL (md5 checked), so the message's "old plugin"
  guess is wrong.  Exclude those flights.
- Swap still reaches 21-22 GB within 2 rounds of six; restart per cycle
  (multirot does) and compare only within one batch.
- A scripted edit landed in the wrong function (`enter` has no
  docstring); diff after every splice.
- `pkill -f` patterns with quotes do not match the process's argv.
