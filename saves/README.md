# saves

The reference copies of every quicksave the farm flies, and the craft files
they were built from.  **These are the source of truth, not the copies inside
`testInstances/ksp*/saves/default/`**, which drift.

    testInstances/syncSaves.sh check           # does every instance match?
    testInstances/syncSaves.sh push            # make them match
    testInstances/syncSaves.sh pull 0 qs_new   # adopt a save made on ksp0

A new save made on one instance (`spaceplane/tools/entrysave.py`,
`tools/savegen.py`) is `pull`ed here, committed, and `push`ed to the farm.
`mkclone.sh` pushes these into every new clone.

Read off each save's active vessel (part count, situation, altitude):

| save | vehicle | state |
|---|---|---|
| `quicksave`, `qs_hot`, `qs_cold`, `qs_steep`, `qs_north` | booster, 18 parts | flying at 21 km, after stage separation |
| `qs_plane`, `qs_plane_{inc,high,low,ecc,south,lit,v1}`, `qs_e20..qs_e80`, `qs_soft`, `qs_gear` | old spaceplane, 23 parts (`craft/SPH/spaceplane.craft`) | orbit, 80 km |
| `qs_entry`, `qs_b*`, `qs_l*`, `qs_m*` | old spaceplane | flying at the 58 km interface |
| `qs_cone` | old spaceplane | flying at 12 km |
| `qs_plane_air` | old spaceplane, **low-wing** variant (qs_plane's craft) | flying at 25 km, 1413 m/s, periapsis 29 km: made by `kspSim/tools/airsave.py` from `qs_plane` so its control tables could be probed on its own craft |
| `qs_shuttle`, `qs_shuttle_inc`, `qs_shuttle_high` | Mk3 shuttle, 30 parts (`craft/SPH/Untitled Space Craft.craft`) | orbit, 87 km |
| `qs_shuttle2`, `qs_shuttle2_inc`, `qs_shuttle2_high` | Mk3 shuttle **with twin wingtip fins**, 31 parts, 37.95 t (`craft/SPH/shuttle.craft`, the user's 2026-09-30 rebuild): the single tail fin replaced by two Big-S tail fins on the outer elevons, canted 15 deg out, mirror-deployed | orbit, 87 km. **Spliced**, not flown up: the new craft's parts (launched on ksp0, saved) put into `qs_shuttle`'s vessel with its flight state (fuel 787.5/962.5, gear retracted, temperatures, crew), so UT, orbit and fuel equal `qs_shuttle`'s; `_inc`/`_high` by the same `savegen.py` recipe, orbits identical to the old shuttle's |
| `qs_shuttle_cone` | Mk3 shuttle | flying at 11971 m, 221 m/s, already at -8 deg alpha in a 33 deg dive (taken from LOG3028's cone): an upset-recovery state, not a cone arrival |
| `qs_shuttle_final` | Mk3 shuttle | flying at 2972 m, 63 m/s -- near the stall; taken from LOG3029's cone |
| `qs_shuttle_low` | Mk3 shuttle | flying at 14913 m, 231 m/s, 150 m/s down, Mach 0.8, 23 km from the runway in late GLIDE: the landing chain (cone, approach, flare) from one repeatable state. Taken 2026-09-29 from defaults LOG3765 (`e1a9838c`), which landed 742 m from the midpoint with 23 parts |

**The old spaceplane exists in two variants that share a name and a part
list** (found 2026-09-24 by kspSim, whose tables for one were being used for
the other): the *low-wing* craft (wings at z = +0.70 on the service module)
is `qs_plane`, `qs_plane_lit`, `qs_e20`..`qs_e80`, `qs_gear`, `qs_soft`; the
*mid-wing* craft (z = 0.00, gear moved with the wings) is `qs_cone`,
`qs_entry`, `qs_plane_inc`, `qs_plane_{high,low,ecc,south,v1}`, `qs_b*`,
`qs_l*`, `qs_m*`.  Wing height sets the roll due to sideslip (the game's own
wrench oracle: -0.035 vs +0.003 N m/Pa at Mach 0.88, 2 deg of sideslip), so
a result on one is not automatically a result on the other.  Check with

    grep -A8 'name = structuralWing' saves/<save>.sfs | grep position

`craft/` is copied from the live install's `saves/default/Ships`.  The VAB
file's role is not recorded; check its parts before relying on it.
