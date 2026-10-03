# HANDOFF — read this first, rewrite it last

Snapshot of the last session; history is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 afternoon").

Last written **2026-10-02 ~17:50**, spaceplane + farm. Defaults fingerprint
**`aed153ef`** (no default changed; fields added). Offline suite OK (861).
Committed (this file is the last commit). Farm **stopped**, inhibitor
**released**, no sims running.

## Where it stands

Goal (lands on the runway reliably) **not reached**. Two blockers, in order
of what caps the success rate:

1. **The landing chain.** Even arrivals within 1-2 km break up: the flare
   commands 9-16 deg of alpha and the game flies 3-4 with only +0.13..+0.31
   of pitch input -- kRPC under-commands against the airframe's restoring
   moment (the sim does not show it). Doors 20-40 m/s of sink.
2. **The hypersonic bank reversal** (unchanged from the morning, see the
   journal): slip 17-51 deg on every base flight, `GLIDE_BANK_SWEEP` clean
   6/21.

## Built this session

| flag / tool | what | measured |
|---|---|---|
| `PITCH_ASSIST` (off) | manual pitch, integrated on (commanded nose . roof) past a 2 deg band, added to kRPC's output (kRPC sums them) | sim fine; **game: runs to -1 (full nose-down) in 6/8**, dives out of the cone. Refuted as built -- sign suspect |
| `pin=total/assist/err` | log column on every flight | base flares +0.13..+0.31 total |
| kspSim | manual + autopilot inputs summed; getters return the total | |

## Next, in order

0. **Check the sign before anything else** (10 min, one instance): on a
   flight, autopilot engaged holding a fixed attitude, set
   `vessel.control.pitch = +0.3` and read whether alpha rises or falls; and
   check `snap.roof` against the nose's up component.  The sim says + is
   nose-up; the game's runaway to -1 says one of them disagrees.  If it is
   the sign, fix it and fly 8 v 8 again.
1. If the sign is right: replace the trim with a direct gain change --
   kRPC's pitch PID gains (`auto_tune` off, kp/ki computed for the measured
   surface torque) in APPROACH/FLARE only -- or a full own pitch loop.
2. Then the approach: doors at 20-40 m/s sink are a design choice (steep
   final); with a working flare it may be enough.
3. The hypersonic reversal (morning's list, unchanged).

## Farm: how to make it faster (state and next steps)

Now: **PyPy** for the autopilot (`PYPY=1 ./spaceplane/tools/rotfly.sh ...`;
6x cheaper propagation, a round from orbit **8 min vs 20**), textures
stripped from clones (`mkclone.sh` does it), and `rotfly.sh`'s time-scale
ceiling is `TS` (default **20**; the governor holds each phase to its control
interval -- read the `loop rate` line). **Six instances (0-5) ran together**
with 6 GB free, cores 23-49% user, swap 1.5 GB after: use six.
Speeds achieved per flight: 1.0-5.6x, ~3x mean; the game always delivered
what was asked (`common/timescale.status`), so the **governor is the limit**:
ticks cost 13-83 ms wall against a 0.1 game-s interval on final.

To make it faster, in order:
1. **kRPC calls per tick.** The tick cost is now round trips, not maths --
   approach ticks went 9 -> 20 ms going from 4 to 6 instances (frames slower
   under load).  Count the non-stream RPCs per tick in each phase (wrap the
   connection) and turn repeated reads into streams, batch the writes, skip
   writes that do not change (`set_direction_and_up`, `time_to_peak`, gear,
   brakes, flaps, `control.*`).  kRPC server settings are already
   `oneRPCPerUpdate=False`, `blockingRecv`, `maxTimePerUpdate=10000`.
2. **The one-off stalls**: a ~15 s DRAIN tick every flight and DEORBIT ticks
   of 180-260 ms that drop the governor to 1x (it governs on the worst tick).
3. **A seventh instance** if cores stay <90% (each KSP ~2 cores, ~3.8 GB).
4. RAM per instance: the 4 GB is anonymous heap (Mono + Unity), not DLLs or
   textures; `-nographics` hangs; part stripping (`partstrip.py`, 4.3 -> 2.0
   GB) needs the cross-mod texture fix in docs/testInstances.md.
5. A compiled propagator only once 1-2 leave the maths as the limit again.

## Traps paid this session

- A tool call the user "rejected" had already run (kwinRun.sh edit, ksp4
  booted with -nographics, mkclone.sh edited) -- check state after a reject.
- Editing `rotfly.sh` while a batch runs it can garble the batch's tail:
  bash reads scripts incrementally. Edit a copy.
- Idle kspSim servers again sat beside the farm; kill them before a batch.
- `kwinRun.sh`'s exec line: the comment inside the continuation ends the
  command, so instances run with `-force-d3d11` only (left as is).
