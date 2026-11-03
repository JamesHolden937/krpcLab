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
