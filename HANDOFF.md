# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-03 evening: the rollout steered the wrong way, and the
approach relayed").

Last written **2026-10-03 ~20:25**. Defaults fingerprint **`6e854d9f`**.
Full offline suite OK (875). Everything is committed; this file is in the
last commit. Farm **stopped**, inhibitor **released**.

**The user, this session:** the old craft (`qs_plane`) is a wingless flying
brick. A bad result there is design, not autopilot. Use it as a regression
check only; the shuttle is the target (`spaceplane/CLAUDE.md`, "Priority").
They also asked for fitted constants to be replaced by runtime
measurements wherever they appear.

## Where it stands

| from orbit, current defaults (rot-newdef2-1003) | intact on runway | damaged | broken up | in the sea |
|---|---|---|---|---|
| shuttle `qs_shuttle2` (n=6) | 1 | 2 | 1 (1.8 km short) | 2 (+2.2 km long) |
| old craft `qs_plane` (n=6) | **6, all on the strip** | 0 | 0 | 0 |

From the shuttle cone saves `qs_s2_hac0-5` (n=12, save-energy*): 6/12 on
the runway, every stop within ~110 m of the centreline, 2 in the sea (both
hac4). **The shuttle lands on the centreline now and lands long**, usually
stopping +0.9..+1.6 km, past the far end.

## What became default this session (all measured)

1. `ROLLOUT_STEER_ACROSS_IS_RIGHT`: **a sign bug.** `across = cross(up,
   along)` points *right* in kRPC's left-handed frame, and `wheel_steering`
   is +1 left, so the nosewheel steered away from the centreline. Every
   "intact" landing last session stopped 200-600 m off. Rollout lines now
   log `st=` (the command) and `trk=` (the track angle off the runway).
2. `APPROACH_CAPTURE_LAG_AWARE`: the capture gain is set from roll's
   time_to_peak (5.3 s on the shuttle, so 0.55 against 2.0). The old gain
   relayed +-40 deg and left the flare door 60-525 m off. The S-turn's
   sideways rate is capped by what the time to the door can take back.
3. `APPROACH_SCURVE_FULL_GAIN`: the weave keeps the full gain. It is the
   approach's only dissipator; at the gentle gain it spent nothing and 4/6
   went into the sea.
4. `HAC_ENERGY_BUDGET`: the cone counts speed over its reference as height.
   The shuttle enters at ~270 m/s against ~100, about 3 km of energy height
   that the plan never priced and that surfaced as a climb near the gate.
   Exit surplus median +1200 -> +500 m.

## Built, not default

- **Runtime-measured replacements** (the user's request), unseparated at
  n=6 (save-fullgain-1003 arm 1):
  - `FLARE_LEAN_BY_ROLL` (wings-level and taper heights as roll times)
  - `FLARE_ALIGN_BY_YAW` (align height as yaw's time_to_peak, 19.7 s on
    the shuttle)
  - `APPROACH_SCURVE_PERIOD_BY_ROLL` (weave half-cycle from the bank
    reversal time)

  Fly them again on the new defaults, n=12.
- `ROLLOUT_STEER_PID`: null at n=6. Touchdowns were 5-390 m off before
  steering mattered. Worth re-flying now that touchdowns are near the line.

## Refuted this session

- `HAC_RADIUS_MIN_M=1000`: still laps=0, nothing changed.
- `HAC_LD_MEASURED`: wired, but raises the plan ratio to 2.2-2.7 where 1.4
  fits. The gap is tracking (flown/planned path 1.3-1.4), not L/D.
- `TOUCHDOWN_AIM_DERIVED`: 0 intact, 2 destroyed, doors 1.7-4.8 km off at
  44-50 m/s. A near aim leaves the approach too little final.

## Next, in order

1. **The long landing.** The flare door is ~2.2 km past the threshold
   because the aim is the far threshold (2400). The derived aim crashed,
   so move it in steps (1800, 1200) **now that the cone exits with ~500 m**,
   and read the door's `rwy=` and speed as well as the stop. Several
   touchdowns are a float, then a stall onto the ground at 37-40 m/s and
   14-24 m/s of sink. The flare may be too high or too long; read the
   FLARE lines' `h`/`v`/`vs`.
2. **hac4**: exits the cone at +3.6-3.9 km with no lap that fits, and is in
   the sea every time. Laps are priced at the 2 km floor; check what a lap
   costs at the hold radius with the energy budget on.
3. **The approach can't reach its commanded alpha** (asks 20-29, flies
   12-15; LOG5139): the shuttle's pitch trim (memory: shuttle-pitch-trim).
4. Re-fly the three roll/yaw-timing flags and `ROLLOUT_STEER_PID` on the new
   defaults.
5. Rollout steering gain from a measured ground-turn response (not built).

## How to measure landing work

`spaceplane/tools/savefly.sh ROUNDS OUT "setsA" "setsB"` from
`qs_s2_hac0-5`: ~10 min per 2-round batch of 12. Restart the farm between
batches (swap 10-12 GB after one). Verify promotions from orbit with
`rotfly.sh "0 1 2 3 4 5" 2 OUT . "qs_shuttle2|" "qs_plane|"` (~25 min).

## Traps paid this session

- **A sign that only shows in the right column.** The steering sign was
  wrong for weeks. "Stopped 30-40 m off" read as weave, not divergence.
  Log the command beside the response (`st=`/`trk=`) for any actuator.
- **The cone saves have no speedbrake.** It is measured in vacuum, and
  they start at 13 km. From orbit it is armed but the approach never
  deploys it either (`ab=--`).
- **A dissipator can hide inside a bug.** The approach's +-40 deg relay was
  spending the cone's surplus. Fixing the relay alone put 4/6 into the sea.
- `start.sh` still hangs one instance at texture load about every third
  restart. Kill it with `pgrep -f "testInstances/ks[p]N/" | xargs -r kill`,
  relaunch with `setsid ./kwinRun.sh N`. Wrap `start.sh` in `timeout 300`.
- `git cherry-pick -q` is not an option in this git.
