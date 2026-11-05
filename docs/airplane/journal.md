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
