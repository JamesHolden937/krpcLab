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
| `qs_shuttle`, `qs_shuttle_inc`, `qs_shuttle_high` | Mk3 shuttle, 30 parts (`craft/SPH/Untitled Space Craft.craft`) | orbit, 87 km |

`craft/` is copied from the live install's `saves/default/Ships`.  The VAB
file's role is not recorded; check its parts before relying on it.
