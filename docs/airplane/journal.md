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
