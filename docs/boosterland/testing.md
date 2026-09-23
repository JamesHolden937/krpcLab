# Booster: testing without KSP

`boosterland/tests/fakeksp` and what its regression fixtures are for.
Numbered failures cited here are in [docs/boosterland/failures.md](failures.md).

`boosterland/tests/fakeksp.py` installs a fake `krpc` module in `sys.modules` and provides
a point-mass flight model (instant pointing, optional exponential atmosphere).
`boosterland/tests/testFlightSim.py` runs the *real* `Autoland` loop against it
(`time.sleep` is patched to advance the simulated vessel) and asserts the
booster completes all phases, lands near the pad, and arrives slowly enough to
keep its legs (`Vessel.impact_speed`, recorded when the *bottom* of the vessel
reaches the ground). `boosterland/tests/testOffline.py` covers the propagator, guidance
laws, logbook gating and config overrides against a `FakeEnv`.

Pass `atmosphere=True` to `fakeksp.Vessel` for the version that matters: the
fake's `simulate_aerodynamic_force_at` returns the true drag times a
deterministic wobble, so the script has to *estimate* a `Cd*A` with a transonic
hump, as it does in game. Vacuum runs cannot reproduce coast drift and never
exercise smoothing. Note `fakeksp.Body.pressure_at` is isothermal, i.e. a
constant speed of sound, so the fake's hump sits at one Mach number all flight
— an assumption the real curve does not get to make.

Regression fixtures worth knowing about:

- a mid-coast velocity kick (`test_coast_drift_is_corrected...`) and the
  atmospheric flight, both for failure 1;
- `fakeksp.Vessel.broken_bounding_box` reproduces failure 4's -7e17 corner (two
  tests fly through it, one where it never recovers), and
  `broken_terrain_sensor` covers the precautionary `surface_altitude` check;
- `test_it_lands_close_from_the_log7_entry_state` flies LOG7's actual
  separation state, for failure 6;
- `fakeksp.AutoPilot.engage_failures` makes `engaged = True` throw failure 7's
  `ModuleGimbal` ValueError that many times — one test refuses ~2 s and still
  lands, another refuses forever and asserts the boostback throttle stays shut;
- `fakeksp.OtherCraft`/`add_craft` put a second vessel in the sky for the
  clearance scan; with `follow=` it tracks the booster exactly, which is the
  stage that never separates and the only way to reach
  `SEPARATION_MAX_COAST_S`.
