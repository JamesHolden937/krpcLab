# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 night -> 10-03: landing from cone saves").

Last written **2026-10-03 ~01:05**, mid-session checkpoint. Defaults
fingerprint is **`f6a13fa5`**. It changed only by new fields (all off) and one
bug fix in an off flag (`ALPHA_TRIM_LOOP`). No default flight law changed.
Offline spaceplane suite OK. Committed. The farm may still be **up**, and the
inhibitor may still be **held**. Check `testInstances/nosleep.sh status`.

## Where it stands

The goal (land the shuttle on the runway reliably) is **not reached**. It is
now measured from **in-game cone saves** `qs_s2_hac0`-`5`, not from orbit:
six cone-entry states, ~5 min per round of six. Work backwards from the
landing to the arrival.

**Best stack** ("capture"), 6 intact of 36 (3 batches) against 0/12 on
defaults:

```
ALPHA_TRIM_LOOP=True;ALPHA_TRIM_IN_HAC=True;ALPHA_TRIM_SLIP_TOL_DEG=15;
ALPHA_TRIM_MAX_DEG=4;ALPHA_TRIM_MIN_DEG=-2;APPROACH_BANK_BY_ROLL=True;
ROLLOUT_ON_MAIN_CONTACT=True;ROLLOUT_HOLD_TAIL_FRACTION=0.4;
APPROACH_ALPHA_AT_TARGET=True;APPROACH_SCURVE_STOP_M=4000;
APPROACH_CAPTURE_MARGIN=0.15
```

None of it is a default yet. It has not been flown from orbit, nor on the old
craft (`qs_plane`). Both are required before promoting any of it.

## Flags added this session (all off by default)

| flag | measured |
|---|---|
| `ALPHA_TRIM_LOOP` (**bug fixed**: integrated kRPC's error against the offset command) + `ALPHA_TRIM_IN_HAC` | the one clear win: handover surplus +0.8..4.4 -> +0.2..0.7 km, contact sink halved (rot-atrim-1002). Unbounded it drives the phugoid; bound -2..+4 |
| `ROLLOUT_ON_MAIN_CONTACT`, `ROLLOUT_HOLD_TAIL_FRACTION` | tail-strike angle is 11.0 deg; the rollout held 8 and FLARE flew 1.4 s on the wheels (LOG4836). No total losses in that batch |
| `APPROACH_SPEED_KD` | bunched door speeds, did not stop the ~50 s phugoid; superseded by `APPROACH_ALPHA_AT_TARGET` |
| `APPROACH_ALPHA_AT_TARGET` | one-g alpha *at the target speed* plus P pull-up. No speed law on final. Smoother |
| `ATTITUDE_OSC_MITIGATION_OFF` | this kRPC build's oscillation detector (latched 0.93 on a landed vessel). Off: **tracking unchanged**. Null |
| `ATTITUDE_PITCH_AIR_FLOOR_S` | stiffer pitch halved approach alpha error, but in the cone it broke the handover. On final and flare only: **4 departures**. Refuted |
| ~~`ATTITUDE_PITCH_DECEL_S`~~ | removed: `deceleration_time` does not exist in this kRPC 0.6.0 build. rot-decel-1003 flew it disconnected (a replicate) |

Also: `APPROACH_CAPTURE_MARGIN` 0.35 -> 0.15 took the door cross-track from
100-600 m to within +-170 m. The capture limit-cycled on a roll that lags.

## Blocker and next step

**The flare does not pull up.** It commands 7-12 deg and flies 2-3 deg at a
flat +0.24-0.4 of pitch input (LOG4927). This is not the oscillation
mitigation (null), and a stiffer kRPC tune departs. Next, in order:

1. **Own the pitch axis in the flare** (method change). kRPC adds client
   manual input to its output, so `PITCH_ASSIST`-style feedforward is
   possible. A *feedforward* (input proportional to commanded alpha minus
   trim alpha) has no integrator to wind; `PITCH_ASSIST` wound to -1.
   Alternatively disengage kRPC for the flare's ~8 s and fly PD on pitch,
   holding roll level and yaw to the runway. Screen whatever is built in
   kspSim first.
2. Door energy: doors at 77-170 m and 15-35 m/s sink. The intact ones
   entered at 95-120 m/s from 310-520 m.
3. Then: the cone's high handover on defaults, and the stack from orbit and
   on `qs_plane`.

## Traps paid this session

- **`start.sh` leaves an instance hung at texture load** roughly every
  third restart (log stops; port never opens). Kill it with
  `pgrep -f "testInstances/ks[p]N/" | xargs kill` (the bracket stops
  pgrep matching its own shell; plain `ksp4/` returned exit 144). Then run
  `setsid ./kwinRun.sh N`.
- **A kRPC attribute that does not exist fails at runtime, not import.**
  The flag logged "not set" in every flight. Read the log line before
  reading the batch. List the client's attributes with
  `dir(vessel.auto_pilot)`: this build has oscillation, attenuation and
  `max_angular_velocity` controls, and no `deceleration_time`.
- **Fit a phase only after the phase above it is fixed.** A stiffer pitch in
  the cone moved its handover 0.6-1 km short, because the cone's constants
  were fitted to the soft pitch.
- n=12 resolves "0 vs 3 of 12" weakly. Read the mechanism (alpha tracking,
  door cross-track, door speed) before the intact count.
