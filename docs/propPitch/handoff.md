# propPitch HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-04: propPitch -- the prop brake's tip limit; props stopped when still; torque counter deleted (fingerprint `ca612601`)

The tip limit and the stop when still are defaults (below); **the
reaction-torque counter (`TORQUE_COUNTER`, the spin-sense probe, the trims,
the log's `trim=`) is deleted** at the user's request (it read 0.00 on the
only craft it flew). `PropPilot` takes a `gui` so the airplane pilot can run
it. Read `propPitch/CLAUDE.md` and `docs/propPitch/journal.md`.

**What changed (all defaults):**
- `BRAKE_TIP_LIMIT` (`propPitch/tipping.py`): on the wheels, the braking
  thrust is bounded so every wheel stays down with `BRAKE_TIP_MARGIN` 0.5 of
  gravity's restoring moment, from the moment about the foremost/rearmost
  contacts (gravity, measured aero force, the thrust line's height over the
  CoM). Past the balance point the bound is the balance. Hill-climb
  backstop: halves on a contact off the ground 0.25 s or pitch rate > 10
  deg/s, rises 0.25/s; where no bound can be computed it climbs from 0 at
  0.1 g/s. Log `tip=`; events `tip:`.
- Still (|v| < 0.5 m/s): every rotor at 0 rpm with its brake latch set,
  until 2 m/s (`BRAKE_RESUME_SPEED`); `mode=brake-stop`.
- `propfly --wheel-brakes P`; `tools/brakesum.py`.

**Measured (farm, qs_prop_noon):** bound ~-17..-25 kN at low speed, ~0 from
55 m/s. Braked from 33-38 m/s it stopped intact in 8-9 s (LOG15245/15246).
From 55-75 m/s it slides and hops whatever the props do; a plain throttle
cut there breaks up 4/6 (LOG15249-15254): the airframe under the harness.

**Not measured:** the tip limit on a wide wheelbase or props below the
CoM; the brake live.

**Next:** the user flies it live. Then: re-find after a lost part (still
ObjectDestroyedException); a harness that can fly this plane past 55 m/s.

**Traps paid:** "no bound on this side" must never mean "unlimited"
(LOG15225-15230). A bound extrapolated through a filtered acceleration lies
after a 1000 kN thrust swing (LOG15233). The fallback must climb from zero
(LOG15221). A propfly launch of six with an inner `&` returns at once: wait
on the `ROT DONE` marker.
