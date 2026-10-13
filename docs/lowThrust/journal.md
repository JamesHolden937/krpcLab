# lowThrust journal

## Session, 2026-10-12: the project planned

The user wants a low-thrust (ion) autopilot: type in a target orbit, the
solver works out the maneuvers, later interplanetary. Hand-flying ion craft
is impractical and MechJeb does not do it.

- Persistent Thrust 1.7.5 + PersistentThrustNavigator are already in the
  farm's `base/GameData` (PT replaces the engine PartModule); the live
  install's copy is not yet checked.
- Plan written into `lowThrust/CLAUDE.md`: Q-law (with Edelbaum and the
  single-element laws as special cases) as a per-tick feedback law, a
  propagator that flies the same law for the PLAN preview, and a kRPC
  executor under rails warp. The executor's design waits on a probe of what
  attitude PT holds on rails (SAS maneuver mode, plain SAS modes, or
  segmented warp).
- Difficulty, as told to the user: the maths is more abstract than either
  existing autopilot but textbook, the plant is exact (no aero, no attitude
  dynamics on rails), so it should be far less empirical than the shuttle.
  Stage 1 a session or two; stage 3 (precise arrival) open-ended.
