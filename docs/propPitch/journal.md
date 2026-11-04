# propPitch journal

## 2026-11-02: built

The user asked for a new autopilot for the prop plane in their newest live
save (`prop.sfs`): adjust the prop pitch continuously so the plane goes
as fast as possible.

- Craft: 4 EM-32S rotors (2 CW, 2 CCW) with 4 R-25 blades each, 16
  blades in all. A KAL-1000 drives the blades' Deploy Angle and the
  rotors' RPM from the throttle. Rotor brakes are on the Brakes group.
  It holds 55 EC.
- Farm save `qs_prop` (the live file) is at sunrise: 1.5 EC/s, and the
  rotors starve and stall (LOG15157, 15161). `qs_prop_noon` = +5100 s,
  53 EC/s.
- Objective: forward aero force per unit mass. It agrees with the
  measured specific force along the nose (`acc=`).
- Seeker v1 (3 s dither, linear trend, gain on the raw slope, 2 deg/s):
  LOG15158 sat on the rate cap the whole takeoff roll. At 10 deg/s,
  LOG15163-15167 sat at pitch 0 for ~150 s, because the slope at rest is
  0.0001-0.002. On LOG15167 a false negative slope at 90 m/s (J rising
  25 m/s^2 in 2 s, seen through a 6 s linear fit) took the pitch 68 -> 23.
  The blades windmilled at -9 m/s^2 and the plane crashed.
- Seeker v2: 1 s dither, quadratic trend, sin2/cos2 for the curvature,
  Newton steps on a smoothed curvature, the t-statistic before there is
  one, 5 deg/s. The offline plant: hunting +-0.5 deg (v1-style t-stepping
  hunted +-5), and a 4 m/s^2 takeoff ramp tracked within 3.5 deg.
- Farm, v2, 6 flights (LOG15168-15173): every roll climbed 0 -> 60-70 deg
  with 2-3 g of acceleration. Every plane was destroyed once airborne:
  `propfly.py`'s kRPC attitude/height hold could not fly it (vertical
  speed +25/-74 m/s). Harness or airframe: unresolved.
- Fixed pitch 30/45/60/75 (LOG15159-15162): 45 tops out at 36 m/s on the
  ground. 60 and 75 at rest stall the blades and drain the EC.

## 2026-11-02 evening: the optimum computed, not searched

The user: "could you make it calculate the optimum instead of hill climb?
conditions change fast sometimes", then "blade load reduces rpm".

- Read KSP's blade code (Assembly-CSharp, monodis): one lifting surface per
  blade evaluated at `velocityOffset` (-5.8 m along the R-25's span: ~6.0 m
  radius, ~290 m/s at 460 rpm), lift along the deflected normal (not
  perpendicular to the flow), drag along the flow; q and Mach of that point.
  Deploy 0 = blade normal tangential (a flat plate into the rotation: the
  stall the seeker sat in at rest); 90 = flat in the disc.
- `bladeModel.py` replaces `seeker.py` (deleted). LOG15177-15183.
  Bug on the way: thrust kN over mass kg in the log (fixed).
- Rotor code: velocity drive, damper 1000, cap maxTorque x limit; EC draw
  load-independent. Load slows the rotor past the cap or without power;
  the motor torque is learned from rpm. Bug: the rpm readout refreshes
  every 0.3 s, so spin-up read as a limit (70 -> 54); fixed with a 0.6 s
  flat-window rule.
- Probe agreement and the harness crashes: propPitch/CLAUDE.md.

## 2026-11-03: START in flight

The user: "the prop autopilot doesn't seem to work when you turn it on mid
flight, change that".

- Live LOG15188 ("Probe", 91 parts, START at 7.7 km, 164 m/s): every rotor
  read 0 rpm for the whole run, EC flat at 5 (no draw), motors engaged and
  unlocked per kRPC; model +24 m/s^2 against J -10.
- Assembly-CSharp: `ModuleRoboticServoRotor.brakingMode` is private, set by
  `BrakeAction` (the Brakes group) and latched on power loss
  (`lockPartOnPowerLoss`, default true). FixedUpdate's drain is skipped
  while braking, matching the flat EC. Only `BrakeAction` with Deactivate
  clears it. The farm harness always sets `brakes = False`, so no farm
  flight had seen it.
- Fix: release every rotor's brake at START; at each re-assert release a
  rotor below `BRAKE_RPM` while the group is off. Farm LOG15189 (on the
  runway, Brakes group on): 0 -> 239-285 rpm 2 s after START. LOG15193 (a
  harness wreck braked to 0): 460 rpm in ~2 s.
- Second bug on the way (LOG15189): `Rotor.observe` stamped the reading
  before advancing its clock, so one tick longer than the 0.6 s window (the
  4.7 game-s START tick) emptied it and raised IndexError. Stamped after.
- Live LOG15185/15187 from the runway: model and J agree to ~1 m/s^2 up to
  75 m/s; LOG15187 zoomed to 57 km at up to 894 m/s, EC out at 35 km.
  Re-sets of "motor off" on all four rotors at 256888/257006 (user input or
  a power latch); the re-assert re-engaged them.
- The farm harness still cannot fly the plane past ~100 m/s (two more
  break-ups, LOG15191/15193).

## 2026-11-03: throttle sets the rpm; the prop brake

The user: "make it so my throttle will control RPM limit", then "when [I]
hit brake, you should spin the motor at full RPM and angle the blades in a
way to stop the motion of the craft ... read whether the velocity vector is
closer to forwards or backwards first".

- `THROTTLE_RPM` (default): RPM limit = throttle x `ROTOR_RPM`. Farm,
  runway, throttle held: LOG15208 (1.0) 460 rpm, LOG15206 (0.5) 230 rpm
  (reached 173 m/s on the runway without breaking up -- at 460 every farm
  run breaks up by ~130), LOG15207 (0) 0 rpm. `propfly --throttle`.
- `PROP_BRAKE` (default): Deploy Angle limits read from Assembly-CSharp
  (`OnStart`: +/-1.5 x ctrlSurfaceRange, field default 15) and the cfgs
  (60 fans/props, 15 heli), now in `bladeTable.py`. Offline (R-25 rotor,
  460 rpm): most reverse thrust -85 deg at rest, 77-89 deg at 30-100 m/s
  (flat in the disc, the inflow from the front), -76 at 150; zero thrust at
  40-45 deg. `settled` now overspeeds a windmilling rotor.
- Farm `propfly --brake-at`: LOG15211 at 40 m/s: 40 -> 0 in ~1 s
  (J -42), then hold at ~0 thrust, still. LOG15210 / 15209 (0.5 throttle)
  at 80 m/s: ~4.5 g reverse reared the plane up and over (ksp2 lost 7
  parts), and the tumbling flipped fwd/rev. A deceleration cap was written
  and reverted: the user releases Brakes if it starts to flip.
- The user: "feather the prop on zero throttle to increase glide ratio".
  The model already picked Deploy 0 at 0 rpm (the most thrust at rest is
  the least drag); made explicit: `THROTTLE_FEATHER` 0.02 (a joystick seldom
  rests at 0) -> 0 rpm, search over the blades' whole range; log `mode=`
  replaces `brake=`. Farm `propfly --cut-at 60`: LOG15214 0 rpm / 0.0 deg
  within ~1 s, coast J ~-2 m/s^2 against the model's prop drag 0.07.
  LOG15212 (THROTTLE_FEATHER=-1) is the same arm: a throttle of exactly 0
  is 0 rpm anyway. LOG15213 (DISABLE_KAL=False as a baseline) is unusable:
  the KAL fought the pilot and never reached the cut. Both coasts
  ground-looped ~5 s after the cut.
