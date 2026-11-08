# airplane -- journal

## 2026-11-04: written

The user's spec: an airplane autopilot with two modes. CRUISE minimises
sideslip, holds roll (capped at 89 deg) and holds pitch attitude through
coordinated turns ("if you are going up 30 degrees and bank 45 degrees it
will pull back on the stick so that 30 degrees is maintained"). HOLD holds
pitch, yaw and roll attitude. In both, the player's input adds to what the
pilot does. A "prop plane" box runs the prop-pitch script beside it, and a
"constant rates" box limits full stick at high speed to a rate per axis
the player sets, in RPM.

Built as Euler-rate outer loops into body-rate PI inner loops, gains scaled
by dynamic pressure. The stick read-back was taken from kRPC 0.7's IL
(`PilotAddon.OnFlyByWire` adds the client's `manualInputs` to the game's
state; `Control.pitch` reads the sum) and KSP's (`FlightInputHandler` adds
the trim into `pitch`/`roll`/`yaw`). Under "constant rates" the stick
becomes a body-rate command; otherwise an axis the player moves is let go
and re-held where it is left. Offline tests pin the geometry; never flown.

## 2026-11-05: gains from the craft

The user: "why are you using handpicked gains? shouldn't you use something
that adapts to the plane?" The first version had per-axis KP/KI and a
dynamic-pressure scale about a hand-set 5000 Pa: constants for a plane
that does not exist. Replaced: each axis's angular acceleration per unit
input from kRPC's `available_torque` / `moment_of_inertia` (it counts the
surfaces at the present q -- the spaceplane's logs read surf pitch 28 /
1928 / 3958 kN m at q 127 / 2903 / 5400 Pa), the loop closing in 4 of its
own measured ticks, the attitude loops 3x slower. A ringing detector halves
an axis's gain where kRPC's figure is high. Offline (1-tick latency,
damping and a bias in the plant), the real authority at 0.25/0.5/1/2/4/8x
kRPC's: settled error 0.05/0.036/0.010/0.006/0.006/0.007 rad/s on 0.2
steps, the backstop at 1/0.9/0.39/0.33. Weaker than reported is the slow
case (no gain increase is attempted). INDI on measured angular
acceleration was not used: `SURFACE_OWN` (spaceplane, 2026-10-21) flew it
at kRPC's tick and went 0/12.

## 2026-11-05: the surfaces' travel time

The user: "also factor in things like control surface deflection time".
It was not: tau counted only the loop's tick. KSP (Assembly-CSharp
`ModuleControlSurface`) slews each surface's input by `actuatorSpeed x dt /
ctrlSurfaceRange` a physics step (`MoveTowards`; `Lerp` when
`useExponentialSpeed`) and its angle at `actuatorSpeed` deg/s; the farm's
base GameData has actuatorSpeed 25/30/35/40 over ranges 10-60 deg (no
exponential stock part). kRPC exposes neither, so the window mod gained
`SurfaceActuators` (every ITorqueProvider's potential torque, and per
control surface its speed, range, mode and torque). The loop's closing time
per axis is now also `sqrt(2 x cap / (accel x slew))` and 3 x any lag, and
the command is slew-limited. Offline grid (accel 1/3/8/20 rad/s^2, slew
0.4/0.6/1.67/3 travel/s, 0.2 rad/s steps, one tick latency): overshoot
sized on the tick 0.044-0.763 rad/s, slew-aware 0.000-0.048; the command
slew alone is most of it at fast settings. Unflown.

## 2026-11-05 evening: first flights -- the Warbird, the farm harness, gains measured in flight (fingerprint `b3fd31af`)

The user: "both hold and cruise is very oscillation prone. fix the issue"
and test on the "Electric Warbird" from now on, spawned flying at 200 m/s,
100 m, full throttle; then "use the prop pitch autopilot" and "make sure to
apply some input during cruise".

**Test bed.** `saves/qs_warbird` (level, 101.8 m, 200.6 m/s, heading 090
over the water, full throttle). Made by `airplane/tools/airstart.py`, which
*flies* it there from runway 09 (kRPC's autopilot on the controls,
propPitch on the blades, the throttle holding 200 m/s while it levels, full
throttle set at the paused save point). Editing a runway save into the air
was tried first and abandoned: the rotor starts at rest, its spin-up rolls
the plane (-24 deg before the first paused frame) and the stopped blades
brake it at ~50 m/s^2; spawned at 150 m/s it slowed to 23 m/s in 18 s and
fell in. Without propPitch the plane does not move at full throttle (the
blades are not at a thrusting pitch).

**Harness.** `airplane/tools/airfly.py` loads the save paused, runs the
pilot with `PROP_PLANE`, unpauses and flies pulses. A second kRPC client
cannot play the stick (kRPC 0.7: one `manualInputs` per vessel, so its
writes replace the pilot's), so the window mod gained `TestStick`: it adds
to `ctrlState` in `OnPreAutopilotUpdate`, which `Vessel.FeedInputFeed` runs
before `OnFlyByWire` (where kRPC adds its input) -- the pilot reads it
exactly as the player's stick. Trims are the hidden disturbance. Schedule:
bank right, reverse, level, pitch up, pitch down, rudder kick, an
out-of-trim roll. Summary: attitude error rms/peak, rate rms, rate
reversals per second, quiet windows and overall.

**The user's live LOG15261** (defaults `570902c5`): pitch kp 0.07 at 300
m/s (kRPC: 2958 deg/s^2 per unit), the nose 8 deg off its target for 10 s;
after stick releases the bank swung -89 -> -91 -> -65 -> -50 against -41,
roll rate -216 -> +180 deg/s, `roll ringing` repeatedly.

**Measured (LOG15265, regression of rate change on input at 0.1 s):** roll
2164 deg/s^2 per unit, damping 1.2/s (kRPC 585: **3.7x low**, the gain too
high, it rang and overshot 46 deg); pitch 675, damping 4.8/s (kRPC 2958:
**4.4x high**, gain too low, and the old integral `kp/(4 tau)` 15x short
of what the damped axis needs). Yaw unidentifiable at that resolution.

**What was tried, in order (each an interleaved batch, restarted farm):**
1. Batch 1, stick pulses of 0.15 for 0.6 s: banked the Warbird **85 deg**
   (it rolls ~180 deg/s at 0.15). CRUISE held the bank in its 4 g turn
   (11.5 g needed at 85) and spiralled into the sea from 100 m -- 5 of 6
   CRUISE flights, both laws. A test problem, not oscillation; pulses cut
   to 0.05 x 0.5 s (~30 deg). The RLS fit on every tick also showed yaw
   sliding to k 0.06 under feedback (no excitation) and ringing.
2. Batch 2 (fit gated to ticks whose input stepped): pitch error 3-5x
   down, roll reversals 2-4x down; roll settled slowly, HOLD yaw rang more.
3. Batch 3 (fit over a 1 s window after each step, mean-rate regressor,
   covariance cap 100 -- the cap of 10 acted as a prior, k 0.20 for 0.25
   offline): same picture; roll peaks traced to the out-of-trim gust only
   (13-17 deg against 5 on the old law; after stick inputs both 3-6 deg).
4. Batch 4: `RATE_TAU_TICKS` 2.5 against 4: no faster roll, more pitch/yaw
   ringing. Stays 4.
5. Batch 5: the fit trusted as far as it explains the rate (`k^quality`,
   `quality x d`). HOLD's cross-controlled yaw had fitted k 0.1-0.3
   explaining little. Result, 3 flights an arm (LOG15363-15382):
   CRUISE roll err 1.4 v 1.6, reversals 0.36 v 0.98/s, pitch 0.5 v 2.9,
   yaw 0.1 v 0.3; HOLD roll 1.3 v 1.3, reversals 0.70 v 1.23/s, pitch 0.9 v
   2.9, yaw 1.1 v 2.1 (reversals 0.69 v 0.64). Promoted; `PLANT_FIT` and
   `rate_gains` deleted.

**Known costs:** the out-of-trim roll gust (0.05 trim, ~90 deg/s worth)
peaks 12-16 deg against ~5-7 on the old, stiffer law, with no swing; HOLD
pitch reversals 0.96 v 0.80/s (small). The pole placement's stronger
integral overshoots a slow-surface step by 0.058 rad/s offline where the
old law did 0.048.

**Traps paid:** `pkill -f <pattern>` from a Bash call matches the call's
own shell (exit 144); a killed `airstart` left two instances whose next
`launch_vessel` never returned -- restart the farm. Setting
`krpc.paused` outside flight throws (PauseMenu.Close).

**Confirmation of the merged defaults** (`b3fd31af`, 3 CRUISE + 3 HOLD,
restarted farm). First try (LOG15387-15396) had the surfaces' closing time
on the *measured* effect: roll tau 0.37 -> 0.26 s, roll reversals back to
0.91/s CRUISE and 1.33/s HOLD -- reverted to kRPC's figure, which is what
flew in batch 5. Second (LOG15399-15409): CRUISE roll err 1.3 deg, rev
0.45/s, pitch 1.0, yaw 0.1; HOLD roll 2.1 / 0.89/s, pitch 0.9, yaw 1.1 /
0.56/s. Swap 18-21 GB through the session (six instances).

## 2026-11-06: the runway, the stick gate, long ticks, the crash guard (fingerprint `b1f8fe9b`)

The user: drift (the Warbird rolls left on take-off, pitches up by
itself); "maintain attitude on runway, reduce oscillations, general
solution"; then "make a crash guard".

**Read from the live LOG15411:** CRUISE's slip loop ran at rest (slip
-179.6 from 0.5 m/s) and held full left rudder before the roll began; the
live stick read-back crossed the 0.05 deadband 834 times in 400 s, mostly
one tick wide (the farm: 3-6 a minute), and every crossing let the hold go
and re-took its target where the plane had drifted.

**Farm, `airfly --runway` (new: runway 09, full throttle, hands off).**
Old code LOG15423, LOG15451: on the ground the pitch integral wound to
+0.13 nose-up against the tail wheel and the nose went to 25.5 deg after
lift-off (target 12.5); roll rate flipping sign tick to tick in bursts
(rev 1.2-1.8/s) on 0.15-0.2 s ticks, gains sized on the 0.10 s mean.
First fix (LOG15425-15431: pitch left free on the ground, heading held,
no fit) made the ground worse: bank 9-19, heading 12-18 deg, a ground loop
at +/-40 deg/s yaw -- kp hit KP_MAX 20 at walking pace (kRPC's authority
~0) and the steering is not in kRPC's yaw figure; with pitch free the
tail lifted and the nose went to -0.6 deg.

**Kept** (LOG15435-15442, 5 flights): kp held to full input at the axis's
rate cap (KP_MAX deleted, critical ki when held), the loop closed on the
current tick when it is longer than the mean (`tick_tau`), on the ground
pitch held without a trim, wings level, heading held, yaw plant fitted
there; `StickGate` (0.15 s confirm, 0.2 firm). Ground bank max 2.1-3.2,
heading drift 3.3-3.4, pitch rate 5-6 deg/s (old: 5.7-7.5 / 4.8-6.5 /
4.6-6.0); lift-off at 32 m/s pitch 9.5-9.9; 15 s after it pitch error
0.6-0.9 rms (old 1.0-3.9), roll rev 0.7-1.2/s (old 1.2-1.8).

**qs_warbird is broken on the MiniSystem farm:** it loads at 155 m/s
pitching -56 deg/s (2026-11-05: 195 m/s, level). Flown on the new code
without a guard, 3/3 went into the sea (LOG15445-15447). `airstart.py`
could not re-make it: kRPC's autopilot sat 6 m off the altitude
(tolerance widened 3 -> 8 m, unverified), then flipped the plane on the
runway, then the plane never moved (prop not spun up).

**Crash guard** (`GUARD`, default on): `pull_up_loss` (react 3 pitch taus
+ tick at the sink, roll level at the roll cap with the bank's lost lift,
then an arc at min(pitch cap, g(n-1)/V)); pulls when the recovery ends
within 20 m + 2 s of sink of `surface_altitude`; overrides holds and stick;
gear down or on the ground disarms it. On the broken save 6/6 survived
(LOG15453-15462, 3 CRUISE + 3 HOLD): 7 pulls at 62-108 m, sink 12-22 m/s,
cleared climbing at 52-84 m; all ended 400-680 m up.

## 2026-11-08: the crash guard deleted

The user: "get rid of the crash guard in the production version". `GUARD`,
`GUARD_*`, `control.pull_up_loss`, its tests and the `hgt=`/`vs=`/`guard=`
log fields are gone (code in `git log`, commit after 6e06ba8).
