# Booster: failure modes already paid for

Short entries: the rule, and enough evidence to stop someone undoing it. The
logs named are on disk if more is needed. Referenced by number from the other
booster docs.

1. **Nothing acted on the coast drift** (`LOG3`, 2002 m, booster broken).
   Boostback finished at 497 m; the coast walked the prediction to ~2000 m and
   the booster landed there. Pointing retrograde with an aero bias corrects
   nothing. Hence CORRECTION.

2. **The landing burn had no margin and no authority** (same flight). It
   triggered at `h=876 needed=878` — the last possible metre — then saturated
   at 0.95 for 7 s. A burn that starts exactly on the stopping distance is
   vertical-only. `SUICIDE_MARGIN`/`SUICIDE_REACTION_S` were raised, and
   `landing_burn_state` adds `LANDING_DIVERT_LEAD` metres of early start per
   metre of miss, capped by `LANDING_MAX_EARLY_M`.

3. **The engines were cut 16 m up and the booster fell** (same flight). The
   cutoff compared the *pad-radius* height against `TOUCHDOWN_ALT_M` and
   ignored the centre-of-mass-to-legs offset. `touched_down` now cuts on
   `situation == landed/splashed`, with a sub-metre height and a
   stopped-descending hover guard (`HOVER_CUT_ALT_M`) as backstops only, both
   ignored above `TOUCHDOWN_BACKSTOP_SPEED` — nothing doing hundreds of m/s is
   about to be on the ground. **Do not reintroduce an
   altitude-based cutoff at a height the vehicle would fall from.**

4. **One bad bounding box, measured once, became every landing height**
   (`LOG4/5/6`). `Vessel.bounding_box` returned a lower corner of ~-7e17 m —
   the script connects during staging, which is exactly when KSP will not
   answer — and `leg_clearance` was read once in `Telemetry.__init__` and
   cached. The landing-burn trigger fired the moment boostback ended. The tell
   in a log is `legs=` tracking `alt=` offset by a constant.
   `Telemetry._refresh_leg_clearance` checks the box against
   `LEG_CLEARANCE_MAX_M` and **re-measures every tick until it gets a
   believable one**, flying on `LEG_CLEARANCE_FALLBACK_M` meanwhile and logging
   that it is. **Do not go back to measuring it once at startup.**
   `Telemetry._surface_altitude` applies the same plausibility test to
   `surface_altitude` itself (`TERRAIN_MAX_M`/`TERRAIN_BELOW_SEA_M`), marking
   the line with `?` after `legs=`; precautionary, no flight has needed it.
   LOG5/LOG6 also show why the log carries `pad=` (the real great-circle
   distance) and why the exit line reports `Vessel.situation`, altitude and
   speed rather than asserting "landed": LOG5 signed off `landed 15706 m from
   pad` from 24 km up.

5. **The landing burn diverted itself into the ground** (`LOG7`). A textbook
   profile that arrived doing **10 m/s sideways**: with a 1190 m miss,
   `LANDING_DIVERT_GAIN` saturated `LANDING_MAX_TILT_DEG` at 20° for the whole
   nine-second burn, closed 36 m of miss and spent the rest building lateral
   velocity nothing was left to cancel. `LANDING_MAX_TILT_DEG` is now 0 — the
   burn thrusts straight retrograde, which nulls horizontal velocity as a side
   effect of stopping — and `LANDING_DIVERT_LEAD` went to 0 with it. The trade,
   over four entry states in atmosphere: 56/199/14/921 m with the divert,
   99/294/18/972 without; vacuum is much worse (81/87 -> 1313/1424) because
   there is no drag to bleed what the coast opens. The sim's autopilot points
   instantly, so it flatters the divert badly.
   **Do not "fix" the burn timing by adding margin to `landing_throttle`'s
   profile.** Planning on `net / SUICIDE_MARGIN` was tried: every thrust-limited
   sim case got worse (impact 4.1 -> 7.2 m/s, 11.3 -> 26.3, one atmospheric case
   385 -> 1043 m out).

6. **CORRECTION commanded an attitude the booster could not reach** (also
   LOG7). It reused `boostback_solution`'s aim, which points along the
   *horizontal* miss vector; during the coast retrograde is 45-72° off
   horizontal, so `align > CORRECTION_ALIGN_DEG` held the throttle at zero for
   36 s, the burn timed out on `CORRECTION_MAX_BURN_S` having done nothing, and
   the miss grew 341 -> 966 m unopposed. The 20 s `CORRECTION_COOLDOWN_S` then
   ran out the clock: by the time a third burn was allowed the vehicle was
   below `CORRECTION_MIN_ALT_M`.
   `guidance.correction_attitude` tilts off **retrograde** instead, bounded by
   `CORRECTION_MAX_TILT_DEG` (50°).
   The second half matters as much: steering near retrograde can only land the
   booster *shorter*, so for an undershoot every reachable direction is wrong —
   and the phase happily burned anyway (a 25 m/s mid-coast kick took the miss
   2591 -> 4503 -> 17277 m over three burns). `trajectory.miss_gradient`
   propagates twice, once with `CORRECTION_PROBE_DV` added along the aim, and
   reports signed metres of miss per m/s; CORRECTION is entered only above
   `CORRECTION_MIN_GRADIENT` and exits when the gradient goes non-positive.
   **Do not remove that gate to make a correction "try harder".**

7. **KSP threw out of the engage call and took the flight with it** (`LOG10`,
   four lines long). `AutoPilot.engaged = True` makes kRPC total up available
   torque and `ModuleGimbal.GetPotentialTorque` raised `IndexOutOfRange` — the
   gimbals were still settling after staging. kRPC surfaces it as a plain
   `ValueError`. It is transient, so `set_autopilot_engaged` *returns* the
   exception and `Autoland.engage_autopilot` retries every tick. Same lesson as
   failure 4: startup is the worst moment to ask, so ask again. Second half:
   with `FLIP_UNDER_POWER` nothing stood between a dead autopilot and full
   thrust along whatever attitude separation left, so BOOSTBACK holds the
   throttle at zero while `autopilot_engaged` is False.

8. **One Cd\*A for the whole descent made boostback overshoot by kilometres**
   (`LOG15`). Boostback ended sitting exactly on the transonic peak, so every
   prediction it stopped on flew the coming supersonic descent with more than
   twice the real drag; over-stated drag lands the prediction short, boostback
   stops when the prediction reaches the pad, and the booster flew past —
   crossing over the pad at 9 km still doing 27 m/s horizontally. The sensor
   was fine (`cda=` reads a normal transonic curve, 25 at Mach 2.5, 50 at 1.2,
   15 at 0.5); the propagator froze it. Hence the curve against Mach: over the
   24-flight sweep grid, 100 m mean / 313 worst -> 50 / 124. **Do not go back
   to a scalar to save the kRPC calls** — the whole curve is probed from one
   attitude.
   The same log's `cda=` swinging 12-60 m² between ticks is why
   `Environment.refresh_drag` clamps outliers to `DRAG_OUTLIER_RATIO` and
   low-passes with `DRAG_SMOOTHING`, per Mach bin.

9. **The clearance scan refused to let the booster flip at all** (`LOG16`, nine
   lines; **the vehicle lost an airbrake** sitting broadside at Mach 2.4). The
   upper stage 19 m away reported a 44.5 m bounding radius and the booster's
   own box was unreadable, so keep-out was 64.5 m against a 19 m gap and could
   never clear. Fixes: `keep_out_distance` caps the keep-out at the gap the
   craft are demonstrably sitting at, and `SEPARATION_MAX_COAST_S` is 5 s with
   the booster turning while it holds. **Do not reintroduce a keep-out the
   geometry cannot satisfy at t=0**, and do not let a hold be unbounded.
   The same log shows `thr=0.98` through a phase that commands zero every tick:
   kRPC's `Control.throttle` is the player's throttle axis, which KSP reasserts
   every frame, so a physical throttle left up from the ascent wins.
   `Autoland.set_throttle` records every command and `check_throttle` logs once
   per episode past `THROTTLE_DIVERGENCE`. **Cut the throttle before pressing
   START.**

10. **The engines were cut 48 m up, over ground lower than the pad**
   (`LOG2`/`LOG3`). `landing_height` took `min(height_above_pad,
   legs_altitude)`; the pad-relative height hit zero while the legs were still
   48 m up (terrain 57.6 m below the pad radius, ordinary ground 1.6 km out).
   The terrain sensor was right and was overruled. `landing_height` now clamps
   into a band on **both** sides; `TestLandingHeight` covers all four corners.
   Both logs also flew the whole way on `LEG_CLEARANCE_FALLBACK_M` — set it to
   your booster's half-length.

11. **A correction burn reported "helping" all the way out to 1.6 km** (same
   two logs). `correction_gradient` is one 5 m/s probe through a propagation,
   and at 7 km the drag decel is the same order as the burn itself, so the
   linearisation does not hold over the ~40 m/s the burn goes on to spend.
   Every `done` condition was a *prediction*; nothing watched the miss.
   `Autoland.correction_making_it_worse` compares the miss the burn started at
   against the miss now, aborting above `CORRECTION_ABORT_RATIO` (1.3) after
   `CORRECTION_ABORT_GRACE_S` (3 s) — the grace matters because a burn that has
   to swing the nose round gives ground before it takes any.
   **Do not respond to this by disabling CORRECTION.** Kicked mid-coast on the
   45 km entry state it is the difference between ~2700 m and ~80 m (+25 m/s:
   77 vs 2711; -25: 88 vs 2699; ±120 @ 25 km: ~125 vs ~3100), and the guard
   changes none of those to the metre.
   The gate's own lesson: **it has to sit above the prediction noise and below
   the disturbance it exists for.** At 300 m it was inside the transonic drag
   estimate's re-settling band and the phase chased its own prediction.

12. **Boostback's exit was a coin flip worth 370 m** (`LOG29`/`LOG30`: same
   config, same save, 4 m and 362 m). Same predicted miss at the exit, 3 s
   apart — the phase was not imprecise but **bimodal**, and 3 s of burn is
   370 m at the pad. `BOOSTBACK_EXIT_TICKS` (4) requires the miss to stay
   inside the tolerance for that many consecutive ticks: four flights then
   exited within 40 ms of each other and landed within 5 m. Do not raise it
   much — at 6 the extra second takes the `min_throttle` vehicle from 5.x to
   7.9 m/s on arrival and
   `test_it_lands_with_an_engine_that_cannot_throttle_deep` fails.

13. **The drag curve was written broadside and used nose-on** (diagnosed with
   `replay.py`, superseded by 14). Through a *ballistic* coast the predicted
   miss walked out monotonically and did not come back: re-flying LOG53's coast
   with the curve as it stood at the boostback exit errs by 1297 m at +99 s,
   with the settled curve by 6 m. The difference is three transonic bins reading
   ~1.35x high. It is **not** the rotating-frame terms — the measured omega
   beats zeroed and negated at every horizon against the flown path; zeroing it
   makes the touchdown land on the truth by cancelling two errors, which is the
   trap. Measured with the aim bias at zero, the overshoot was 757 m mean.
   Three fixes were flown and **all three are worse**; they are in the tree,
   defaulted off:

   | change | flag | result at zero bias |
   |---|---|---|
   | reject probes off retrograde | `DRAG_PROBE_MAX_AOA_DEG=20` | 676 median, but **one booster lost** |
   | probe the descent attitude | `DRAG_PROBE_DESCENT` | **splashed 3603 m short** |
   | converge a fresh bin faster | `DRAG_WARMUP_SAMPLES=4` | 951 mean, worse |

   Together the last two flip the sign (648 m *east* against 757 m west),
   which brackets the answer: the broadside probe reads ~20% high at the
   transonic peak and the turned probe ~13% low. They also land within 11 m of
   each other against 85 m — precision the aim bias cannot buy. Do not reach
   for a scalar correction factor: the estimator is accurate at each instant
   (1.00-1.12 of the drag actually experienced); the samples are taken in the
   wrong configuration.

14. **The answer to 13 was to re-sweep the curve repeatedly, and let CORRECTION
   act on the honest prediction it buys.** Neither half is worth anything
   alone, and that is the whole result. The **re-sweep**
   (`DRAG_RESWEEP_AOA_DEG` 25°, `DRAG_RESWEEP_INTERVAL_S` 10 s,
   `DRAG_PROBE_DESCENT_ALTITUDE`) throws the curve away and re-probes it in the
   descent attitude; once is not enough, because the descent-altitude probe
   asks where the booster will be using each bin and that comes from a
   prediction made with the curve being replaced. The **gate**
   (`CORRECTION_ENTER_M` 1200 -> 300) is the other half: remove the walk and
   the noise floor moves.

   ```
   base        miss 80 -> 251 -> 461 -> 676 -> 743 -> 778   walks, monotone   -> 801 m
   re-swept    miss 480 -> 573 -> 540 -> 529 -> 519 -> 555  flat from 20 s in -> 568 m
   + gate 300  miss 138 -> 281 -> 255 -> 254 -> 246 -> 278  one burn at 33 km -> 288 m
   ```

   The middle row is the point: the prediction is *right* and nothing acts on
   it. Over 5 entry states, bias zeroed: committed-before 663-902 mean / 946
   worst; re-sweep alone 791/1081; gate alone 653/779; **both 230/473**.
   Decomposing the drag half (gate 300 throughout): gate alone 653, + one
   re-sweep 572, + the repeat 351, + the descent-altitude probe 327 — **the
   repeat is the mechanism**.

15. **The last tick of a burn was worth tens of metres, because the throttle
   taper had a floor.** `BOOSTBACK_MIN_THROTTLE` was 0.05, an unexamined
   default, and it is the quantum at the end of *both* powered phases since
   CORRECTION flies boostback's law. Near the end of a burn the taper commands
   far less than the floor, so the throttle sits *at* 0.05 and stops tapering —
   over 100 m of predicted miss per second of burn still being commanded, on
   correction burns that last under a second (LOG102: 465 -> 137 m in 0.78 s).
   It was mistaken for failure 12 until the `held=`/`ticks=` diagnostic showed
   four flights exiting within 0.08 s and still landing 214-466 m out. With the
   floor removed the miss decays smoothly (~3 s time constant) so which tick
   ends the burn stops mattering. **Do not put the floor back to keep the
   engine lit** — the demand there is zero by construction.
   `CORRECTION_ENTER_M` went 300 -> 60 and `CORRECTION_EXIT_M` 100 -> 15 with
   it; **the exit threshold has to move with the gate**, or the phase enters and
   exits on the same tick and spends a relight doing it. Over five entry states
   at zeroed bias: 220/186/259 (median/mean/worst) -> **93/110/220**. A 30 m
   gate buys nothing at the median and produces a 329 m outlier — failure 11's
   lesson at a smaller scale. `CORRECTION_MAX_BURNS` was then measured, not
   guessed: 3 beats 6 and 10 on all three statistics (99/172 mean/worst against
   106/295 and 116/273); the saturation is the phase converging, not starving.

16. **The whole systematic overshoot was the integration step.** Every
   `AIM_BIAS_EAST_M` in this file's history — 628, then 170, then 90 — was
   cancelling one bug: `predict_landing` integrated with semi-implicit Euler at
   `PREDICT_DT_ATMO` = 0.5 s, which over a hundred-second descent through
   `rho v^2` *biases* the answer rather than adding noise to it. `replay.py`
   settles it in one line — LOG243 landed 109 m out, and the tick at 20 km
   re-flown with nothing changed but the step:

   | step | predicted, from 20 km |
   |---|---|
   | Euler, 0.5 (committed) | 48 m |
   | Euler, 0.1 | 99 m |
   | **RK4, 0.5** | **114 m** |
   | RK4, 0.1 | 113 m |
   | *what the booster did* | *109 m* |

   The naming symptom: the old propagation **was not self-consistent** —
   propagate 60 s forward with the same model and re-predict and it answered
   80 m where the start answered 48. Only a discretisation can do that; under
   RK4 the same test is flat. In game over five saves at `AIM_BIAS_EAST_M` 0:
   median 82 / mean 87 / worst 259 (Euler) -> **30 / 66 / 162** (RK4), with the
   east residuals centred at +0.4 m. **`AIM_BIAS_EAST_M` is therefore 0.**
   RK4 costs 2.4 ms a propagation against 0.8, and is *converged* at the step
   the loop already affords; refining Euler instead needs `DT_ATMO` 0.05 for a
   worse answer. `PREDICT_RK4=False` restores the old scheme;
   `test_the_step_is_fine_enough_to_have_stopped_mattering` holds the property
   rather than the scheme.
   **`tests/fakeksp` cannot judge this change and says the opposite** — it
   integrates its own truth with semi-implicit Euler at `SIM_DT` 0.2, so it
   rewards a predictor sharing that bias (71/45/50/49 m for Euler against
   144/125/144/60 for RK4), and sub-stepping its physics 8x does not change the
   verdict.

   **The obvious follow-up is wrong and it is worth 700 m.** With an honest
   prediction `qs_steep` lands short, and everything after boostback can only
   shorten the flight — so the exit was gated on the *signed* miss
   (`BOOSTBACK_UNDERSHOOT_M`, never stop while still short). Clean in the sim
   (4/75/19/5 m), a disaster in game (236-885 m on every save). One exit line
   says why: `BOOSTBACK -> COAST miss=3m long=+2`. **Boostback approaches the
   pad from the undershoot side**, so `BOOSTBACK_TOLERANCE_M` is not slop, it
   is a deliberate *lead*, and the coast spends it. Demanding `long >= 0` makes
   the burn eat the lead — the last 190 m cost 36 s of extra burn, ending lower
   and slower, and the coast re-opened 500 m. Left in at 1e9 (off).

   What is left is *not* a constant: `qs_hot` lands ~95 m long and `qs_steep`
   ~130 m short. Two attempts at that asymmetry, both in the tree at their
   defaults:

   | attempt | flag | in game |
   |---|---|---|
   | tighter boostback lead | `BOOSTBACK_TOLERANCE_M=120` | median 58 vs 38, worst 124 vs 155 — keep 190 |
   | two-sided correction up high | `CORRECTION_FREE_AIM_ALT_M=25000` | `qs_hot` 52/53 (vs 67/47), `quicksave` **142 and 223** (vs 6 and 12) |

   The second is worth picking up again: the diagnosis holds (above 25 km the
   vehicle can hold a horizontal attitude and the gradient gate agrees) and
   only the law is wrong. It borrowed boostback's dv law, sized to null tens of
   kilometres, and pointed it at a 100 m miss — one flight went 142 m west and
   the next 223 m *east*. A correction with full horizontal authority needs a
   law sized for a hundred metres, and probably a tighter cap than
   `CORRECTION_MAX_THROTTLE`.

17. **The landing burn flew the booster back into the sky** (`LOG429`,
   `LOG431`, both `qs_hot`). At 13 m and quarter throttle it went to full and
   **left the ground at 101 m/s**, coasted to 1021 m, fell back, did it again,
   and ran the tanks dry 1697 m out. The mechanism is one floor:
   `landing_command` sizes its feedforward on `span = max(1.0, remaining)`, so
   a height reading near zero while the vehicle is still doing several m/s asks
   for 26 m/s^2 of deceleration, and `LANDING_THROTTLE_CAP` is all that stands
   between that and a nearly empty booster with 3.5 g in hand.
   **Do not try to establish which input lied** — a height is a terrain sample
   offset by a bounding box, and bad ones have ended five flights. What is
   unambiguous from one tick is the *sign of the vertical speed*: below
   `LANDING_GOAROUND_ALT_M` (60 m) a booster going up has finished its burn, so
   cutting the throttle trades nothing away. Both `guidance.landing_throttle`
   and `_fly_landing_burn` pass the signed vertical speed into
   `landing_command`, so propagator and control loop still fly the same law.
   Incidence: **2 of 9 flights**, and both were that grid's entire worst case.
   Invisible in `fakeksp`, which has no terrain sensor to misread.

   **The same floor bites a metre up, with no ascent to give it away, and there
   it is commoner** (`LOG446`). At **1.6 m descending at 3.5 m/s** — the
   gentlest arrival this profile can produce — the feedforward asked for 0.95
   where the whole approach had flown at 0.30 (which *is* hover for it). It
   tipped, the lit engine drove it sideways at 30 m/s, and it circled the pad
   until the tanks ran dry. The vertical-speed guard cannot catch this one.
   Hence `LANDING_FLARE_ALT_M`/`LANDING_FLARE_SPEED`/`LANDING_FLARE_MARGIN`:
   below 12 m and under 8 m/s the throttle is capped at a hover plus a little.
   A *fast* arrival is deliberately left alone — it needs everything it has.

   **Read `hs=` on the last three landing-burn lines of every flight in a sweep
   before believing the distances.** In the grid that found this, six of
   fourteen flights arrived above 25 m/s sideways, and all six were in the
   columns that landed *nearest* — they were the only ones that got close
   enough to reach the flare. A bug that punishes whichever configuration is
   winning inverts a sweep's conclusion, and this one did.
