# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 night -> 10-03: landing from cone saves").

Last written **2026-10-03 ~03:05**, end of session. Defaults fingerprint is
**`6640ccdc`**: **the landing stack became the default** (bc2c494). Full
offline suite OK (865). Everything is committed; this file is the last
commit. Farm **stopped**, inhibitor **released**, no sims running.

## Where it stands

The goal (land on the runway reliably, both craft) is **closer, and not
reached**.

| from orbit, current defaults | intact | damaged, on the ground | destroyed |
|---|---|---|---|
| shuttle `qs_shuttle2` (n=14: rot-orbit2 stack arm + rot-newdef) | 2 | 10 (6-28/31 parts) | 2 (one from a +9.5 km arrival) |
| old craft `qs_plane` (n=14) | 5 | 9 (6-22/23 parts) | 0 |
| *both, on the defaults before tonight* | 0 | 1 | 11 of 12 |

From the shuttle's cone saves, the stack lands **13/24 intact**. So the
landing chain works about half the time once the arrival is near. From orbit
the arrival still scatters (+3 to +11 km on a third of flights).

## What became default (all measured; journal has the batches)

- `ALPHA_TRIM_LOOP` (**bug fixed**: it integrated kRPC's error against its own
  offset), `ALPHA_TRIM_IN_HAC`, slip tol 15, bounds -2..+4 (unbounded it
  drove the approach phugoid).
- `FLARE_PITCH_P=0.08`: manual pitch input proportional to the flare's pitch
  pointing error, summed with kRPC's output. The flare used to fly 2-3 deg
  against 7-12 commanded. **This is the lever**: 6/36 -> 13/24.
- `APPROACH_ALPHA_AT_TARGET`: the one-g alpha at the *target* speed plus a P
  pull-up, in place of the speed-path law (a ~50 s phugoid swung door speeds
  43-122 m/s).
- `APPROACH_CAPTURE_MARGIN` 0.35 -> 0.15. The capture limit-cycled
  +-500 m on a lagging roll; doors are now within +-170 m.
  `APPROACH_SCURVE_STOP_M` 1500 -> 4000. `APPROACH_BANK_BY_ROLL`.
- `ROLLOUT_ON_MAIN_CONTACT` and `ROLLOUT_HOLD_TAIL_FRACTION=0.4`. The
  shuttle's tail strikes at 11.0 deg, the rollout held 8, and FLARE flew
  1.4 s on the wheels.

Built and **not** default: `APPROACH_SPEED_KD` (superseded),
`ATTITUDE_OSC_MITIGATION_OFF` (null on tracking),
`ATTITUDE_PITCH_AIR_FLOOR_S` (departs). `ATTITUDE_PITCH_DECEL_S` was removed:
there is no `deceleration_time` in this kRPC build.

## Next, in order

1. ~~Old craft rollout~~ -- deprioritised by the user 2026-10-03 (a wingless brick; see spaceplane/CLAUDE.md). **Shuttle rollout steering instead**: it stops 300-480 m off the centreline. It ends 100-200 m off the centreline, and two of
   six broke up to 6-7 parts on the ground. Contacts are 8 m/s at 44 m/s
   every flight, which suggests its flare is saturated at alpha 14-15. Read
   `oscsum.py` and the ROLLOUT lines. A cone/final save for the old craft
   would make this fast. Make it in game with `entrysave.py`.
2. **Shuttle's remaining landing losses** from the cone saves (11/24). They
   are doors low on energy (77-170 m, 15-35 m/s sink) and touchdowns at
   60-75 m/s. Use `spaceplane/tools/savefly.sh`.
3. **The arrival from orbit** (old blocker 2, the hypersonic reversal; +3 to
   +11 km on a third of shuttle flights).
4. The same proportional pitch assist in APPROACH is untested. Stiffening
   kRPC's tune departs; a manual P term might not.

## How to measure landing work now

`spaceplane/tools/savefly.sh ROUNDS OUT "setsA" "setsB"`: instance i flies
`qs_s2_hac<i>` (or `SAVE=<prefix>`), and arms alternate by round, so every
save sees every arm. ~5 min per round of six. Restart the farm between
batches: swap reaches 12-15 GB after 2-4 rounds.

## Traps paid this session

- **`start.sh` hangs one instance at texture load** about every third
  restart. The log stops and the port never opens. Kill it with
  `pgrep -f "testInstances/ks[p]N/" | xargs kill` (the bracket keeps pgrep
  off its own shell, which gave exit 144), then `setsid ./kwinRun.sh N`.
- **A kRPC attribute that doesn't exist fails at runtime, not import.** One
  arm flew disconnected. Read the flag's own log line before the batch.
  `dir(vessel.auto_pilot)` lists what this build has (oscillation
  mitigations, attenuation angles, `max_angular_velocity`; no
  `deceleration_time`).
- **`rotfly.sh` with more arms than instances flies only some arms.** Use
  `savefly.sh`.
- **A stiffer pitch in the cone broke the cone.** Its constants were fitted
  to the soft pitch. Fix a phase only below a phase that is already settled.
- **Today's defaults no longer land the old craft at all** (4/4 destroyed
  before the promotion). Every change made for the shuttle since
  2026-09-23 went in untested on it. Fly both craft.
