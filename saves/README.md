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

**Derived saves are not tracked.**  The saves `derived.txt` lists are
`tools/savegen.py` outputs from a tracked source save (byte-identical on
regeneration); `syncSaves.sh check` and `push` rebuild them first, so they
appear here after either.  Add a savegen-made save as a line there, not as a
file.

Read off each save's active vessel (part count, situation, altitude):

| save | vehicle | state |
|---|---|---|
| `quicksave`, `qs_hot`, `qs_cold`, `qs_steep`, `qs_north` | booster, 18 parts | flying at 21 km, after stage separation |
| `qs_shuttle`, `qs_shuttle_inc`, `qs_shuttle_high` | Mk3 shuttle, 30 parts (`craft/SPH/Untitled Space Craft.craft`) | orbit, 87 km |
| `qs_shuttle2`, `qs_shuttle2_inc`, `qs_shuttle2_high` | Mk3 shuttle **with twin wingtip fins**, 31 parts, 37.95 t (`craft/SPH/shuttle.craft`, the user's 2026-09-30 rebuild): the single tail fin replaced by two Big-S tail fins on the outer elevons, canted 15 deg out, mirror-deployed | orbit, 87 km. **Spliced**, not flown up: the new craft's parts (launched on ksp0, saved) put into `qs_shuttle`'s vessel with its flight state (fuel 787.5/962.5, gear retracted, temperatures, crew), so UT, orbit and fuel equal `qs_shuttle`'s; `_inc`/`_high` by the same `savegen.py` recipe, orbits identical to the old shuttle's |
| `qs_shuttle2_{,inc_,high_,ecc_}rigoff` | twin-fin shuttle, rigid attachment off on the delta wings | orbit: 87 circular / inc 2.5 deg / 87x193 km / **ecc 77x135 km** (`savegen.py --source qs_shuttle2_rigoff --prograde 30 --radial -70`, 2026-10-09; the earlier ecc save had its periapsis at 69 km, inside the atmosphere, and was replaced) |
| `qs_shuttle2_fwdwing_rigoff`, `_fwdwing09_rigoff` | `qs_shuttle2_rigoff` with both delta wings (and their elevons, fins and main gear) moved **0.6 m / 0.9 m forward** -- the same parts, less stability margin in pitch and yaw | orbit as `qs_shuttle2_rigoff`; the low-margin test airframe (the user, 2026-10-20: "make craft flyable with minimum stability margin"). Rebuild: shift `position` y by +0.6 on every `wingShuttleDelta` and its children in the vessel block |
| `qs_shuttle2_i90low_rigoff` | twin-fin shuttle, rigid attachment off | orbit: **70.23 km circular**, inclination 90 -- the live LOG10020 orbit (grazing entry); `qs_shuttle2_i90_rigoff` with the vessel's SMA and `alt` hand-edited (2026-10-12) |
| `qs_shuttle_cone` | Mk3 shuttle | flying at 11971 m, 221 m/s, already at -8 deg alpha in a 33 deg dive (taken from LOG3028's cone): an upset-recovery state, not a cone arrival |
| `qs_shuttle_final` | Mk3 shuttle | flying at 2972 m, 63 m/s -- near the stall; taken from LOG3029's cone |
| `qs_shuttle_low` | Mk3 shuttle | flying at 14913 m, 231 m/s, 150 m/s down, Mach 0.8, 23 km from the runway in late GLIDE: the landing chain (cone, approach, flare) from one repeatable state. Taken 2026-09-29 from defaults LOG3765 (`e1a9838c`), which landed 742 m from the midpoint with 23 parts |
| `qs_stack` | "shuttle first complete test flight", 70 parts, 366.8 t: the heavy booster (five Vectors, 264 t, legs, airbrakes) under the Mk3 shuttle (Rhino, 101 t) | on the pad, PRELAUNCH. The live game's FMRS `before_launch.sfs` of 2026-10-18, **with the three gears' autostrut Grandparent instead of Heaviest** (2026-10-19: Heaviest strutted the wing-mounted gear to the booster's S3-14400 and the delta wings broke off at separation on ~11 of 12 flights; 0 of 12 after). `ascent/` flies it |
| `qs_stackorb` | the `qs_stack` orbiter alone, 32 parts, 30.51 t: flown to orbit by `ascent/tools/stackfly.py` on ksp0, cargo bay opened and the TD-18 fired to drop the two Large Holding Tanks (30 t of ore) as the user did live, debris vessel deleted from the save | orbit 81 x 79 km, equatorial (2026-10-20). The airframe of the live LOG12759 tumble; `spaceplane/` flies it |
| `qs_stack_small` | "Untitled Space Craft" (`craft/VAB/`), 41 parts, 73.9 t: the 18-part booster (one Vector, 59.5 t, no legs) under a Mk1-3 pod with a Cheetah (14.4 t); Jebediah in the pod (launched uncrewed, the upper stage had no control source once the booster's probe core left) | on the pad; `launch_vessel` + `save` on ksp1, 2026-10-18 |
| `qs_stack_stretch` | `craft/VAB/stack small stretched.craft`, 42 parts, 91.9 t: `qs_stack_small` with a second X200-32 in the booster (77.5 t), liftoff TWR ~1.04 | on the pad; `ascent/tools/stretchcraft.py`, launched and saved the same way |
| `qs_lowthrust` | ion probe "lower thrust test": OKTO2 core (SAS level 2), one Dawn 2 kN, 12 RTGs, battery bank (1005 EC), 21 t Ore ballast, nosecone | orbit, 86.75 km circular equatorial. The user's live "low thrust" save (2026-10-13, after PT was installed live), other vessels removed. **A save made before PT was installed has no `PersistentEngine` node, and PT then silently does nothing** (it finds its engine only in `OnLoad`) |
| `qs_lowthrust_kell` | the same vessel | Kerbin 3000 x 100 km equatorial, at apoapsis (`qs_lowthrust` with SMA 2150000, ECC 0.6744, MNA 3.1): the aerobraking test |
| `qs_s2_e42`, `qs_cargo_e42` | twin-fin shuttle (`qs_shuttle2_rigoff`) / cargo shuttle | flying at 42 km, Mach 6, in the glide: `spaceplane/tools/stepbench.py --air` bench saves, taken with `entrysave.py --alt 42000 --low 40000` on 2026-10-21 (0.7 + stock surfaces) |
| `qs_shuttle2_lt`, `qs_shuttle2_lt_nowheel`, `qs_shuttle2_lt_wheelonly` | twin-fin shuttle (`qs_shuttle2_rigoff`), Rhino thrust limiter 5% (100 kN) | orbit 86.7 circular: the lowThrust mod's RCS/gimbal test. `_lt`: gimbal free, RCS group on; `_nowheel`: + reaction wheel `Disabled`; `_wheelonly`: RCS group off, gimbal locked (2026-10-15) |

The old spaceplane (`qs_plane` and its family: entry, cone, ladder and
orbit variants, `craft/SPH/spaceplane.craft`) was retired 2026-10-03 and its
saves removed 2026-10-18; `git log -- saves/qs_plane.sfs` recovers them.

`craft/` is copied from the live install's `saves/default/Ships`.  The VAB
file's role is not recorded; check its parts before relying on it.
