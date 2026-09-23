# kRPC API notes

Signatures and behaviours that differ from older tutorials, for both projects.

Docs (v0.6.0): <https://krpc.github.io/krpc/latest/python/api/>. Look up
anything uncertain — several signatures differ from older tutorials.

- **`Flight` reports the achieved aerodynamic force and the air it was made
  in**, which removes any need to reconstruct them: `aerodynamic_force` (and
  `lift`/`drag` separately) in the frame the `Flight` was created with, plus
  `atmosphere_density`, `dynamic_pressure`, `mach`, `speed_of_sound` and
  `static_air_temperature`. Resolve the force about the velocity and the
  vehicle's own `Cl*A`/`Cd*A` fall out, with no density assumption and no
  Coriolis term to worry about. The spaceplane went three sessions inferring
  these from finite differences of the velocity — see its failure 10b — so
  before deriving an aerodynamic quantity, check this list first. All are
  streamable, so they cost one setup and nothing per tick.
- `Flight.simulate_aerodynamic_force_at(body, position, velocity, rotation)`
  takes **four** arguments (the rotation is the orientation to evaluate at) and
  works in the frame the `Flight` object was created with.
- `AutoPilot` exposes an `engaged` property in current versions and
  `engage()`/`disengage()` in older ones; `autoland.set_autopilot_engaged`
  handles both, and also catches the KSP-side `IndexOutOfRange` that engaging
  can throw while the gimbals settle (failure 7). kRPC reports server-side
  exceptions as ordinary Python builtins — that one arrives as a `ValueError` —
  so catching by type is not an option.
- `AutoPilot.set_direction_and_up(direction, up, roll=0)` points the nose and
  rolls the roof toward `up`. It is the singularity-free form: `target_roll`
  alone is measured against `up_reference`, which defaults to the frame's
  zenith and so is undefined with the nose vertical. `target_roll` left at NaN
  does not hold any roll — it only damps the rate.
- `Vessel.available_thrust` is zero when the engines are not active, which
  would silently disable the landing-burn trigger; `Snapshot.max_accel` falls
  back to `max_thrust`.
- Buttons: read `clicked` through a stream and set `button.clicked = False`
  after handling. `button.text` is a `Text` object — set `.content`.
- **`ControlSurface` exists even when the surface controls nothing**, and it
  is the route to a speedbrake. `vessel.parts.control_surfaces` returns typed
  wrappers despite the four mods this install keeps that replace the stock
  module (`SyncModuleControlSurface`), and each exposes `pitch_enabled`,
  `yaw_enabled`, `roll_enabled`, `deployed`, `inverted`, `authority_limiter`,
  `deflection`, `deflection_override`, `surface_area` and `available_torque`.
  `deployed` plus `inverted` is what makes a split rudder: the pair deploys to
  opposite sides at the part's *deploy angle*, which is not a typed property
  — it is the module field `"Deploy Angle"`, reached through
  `module.has_field(name)` / `module.set_field_float(name, value)` on the
  part's `ControlSurface` module. **The part menu's `Pitch`/`Yaw`/`Roll`
  fields are KSP's *ignore* flags** (`ignorePitch` in the save), not enables:
  `Pitch=False` means the axis is live. An earlier reading took them the other
  way round, and `available_torque` on the pad (q=0) seemed to agree; every
  surface on both spaceplane craft has every axis live (docs/spaceplane/journal.md,
  "Session, 2026-09-23"). `spaceplane/airbrake.py` identifies a rudder
  geometrically, from `part.position(frame)` and the span axis of
  `part.rotation(frame)`, which does not depend on those flags.
