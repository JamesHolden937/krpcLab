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
