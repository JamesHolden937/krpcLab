# HANDOFF — read this first, rewrite it last

Snapshot of the last session. History is in `docs/spaceplane/journal.md`
("Session, 2026-10-02 evening: farm speed"), and the practices are in
`docs/loopCost.md`.

Last written **2026-10-02 ~20:25**, farm speed only. Defaults fingerprint is
**`03f34fd6`**: new fields only, and no flight law changed. The new defaults
govern the farm: `RPC_BATCH`, `GOVERN_PEAK_SKIP`, `GOVERN_PEAK_WINDOW_S` and
`TIMESCALE_QUANT_FRACTION`. Offline suite OK (865). Committed; this file is
the last commit. Farm **stopped**, inhibitor **released**, no sims running.

## Where it stands

The spaceplane goal (land on the runway reliably) is **not reached**. Its
blockers are unchanged from the afternoon, in order:

1. **The landing chain.** Even arrivals within 1-2 km break up. The flare
   commands 9-16 deg of alpha and gets 3-4; kRPC under-commands against the
   restoring moment. `PITCH_ASSIST` is refuted as built (it runs to -1). The
   sign check comes first (see Next).
2. **The hypersonic bank reversal:** 17-51 deg of slip on every base flight.

This session's base-arm flights broke up in 3-5 of 6 per round, the same rate
as before, and that is still blocker 1.

## Farm: what changed this session

A round of six flights from orbit went **643 s -> ~453 s**, from 1.42x the
throughput (~34 -> ~48 flights an hour). Per flight the mean went from 533 s
to 381 s. HAC, APPROACH and FLARE still get 0.10-0.11 game-s per tick
(`rot-rpc2` against `rot-final-1002`, LOG4733-38 against LOG4781-92).

| what | measured |
|---|---|
| `common/rpccount.py`: four end-of-log lines (calls per tick, wall per phase, slowest ticks; `pk=` in `loop rate`) | the instrument; read these first |
| `RPC_BATCH` (`common/krpcbatch.py`): an aero row is one request | 14 calls take 6-7 ms against 10-45, bit-identical; COAST 150 -> 98 s |
| `settle_game`: probes wait in game time at the ceiling | DRAIN 18 -> 6 s |
| `GOVERN_PEAK_SKIP=1`, warp ticks ignored, cost reset per phase | COAST 98 -> 72 s, HAC 1.1-1.4x -> 2-2.8x |
| `GOVERN_PEAK_WINDOW_S=60` | **null** for speed. Kept: harmless, and the DEORBIT burn is still governed |
| `TIMESCALE_QUANT_FRACTION=0.2` | **null** for speed. GLIDE is physics-bound |
| `kspSim/fastclient.py`: `call_bytes` factored out for batching | bytes unchanged (`testPhysics`) |

**The limit now is the game's main thread, not the autopilot.**
- GLIDE: asked for 9-19x, delivers ~4.5x at ~23 fps, about 4 ms per physics
  step.
- `top -H` shows the main thread at 75-99% whether or not the instance keeps
  up, so that figure can't show saturation. Compare `achieved` with
  `commanded` in `testInstances/kspN/timescale-status.txt`.

Per flight now:

| phase | wall s | speed |
|---|---|---|
| GLIDE | ~125 | 4.2x |
| HAC | 25-120 | ~2x |
| DEORBIT | 80 | 6x (23 game-s of it at 1x, slewing) |
| COAST | 75 | 10x |
| the rest | 20-40 | — |

## Farm: next, if more speed is wanted (the user said this is good enough unless a fix is cheap)

1. **Fewer parts / less RAM** (`partstrip.py`, 4.3 -> 2.0 GB; needs the
   cross-mod texture fix in docs/testInstances.md). It cuts physics cost per
   step and frees RAM for more instances.
2. **HAC's torque reads**: `AvailableControlSurfaceTorque`,
   `AvailableReactionWheelTorque` and `MomentOfInertia` are read every tick,
   and some ticks take 40-60 ms game-side. Reading them every N ticks is a
   behaviour change, so put it behind a flag and fly 8 v 8.
3. **The DEORBIT slew** (~20 s per flight at 1x before ignition). Only with
   the scale dropped *before* ignition: failure 91.
4. Fuel reads after DRAIN, and the user's COAST warp-to-drop-out idea. Both
   are small now (rails warp is already ~20 s of COAST).

## Spaceplane: next, in order (unchanged)

0. **Check the sign before anything else** (10 min, one instance). On a
   flight with the autopilot holding a fixed attitude, set
   `vessel.control.pitch = +0.3` and read whether alpha rises. Check
   `snap.roof` against the nose's up component. If the sign is wrong, fix it
   and fly `PITCH_ASSIST` 8 v 8 again.
1. If the sign is right, change kRPC's pitch PID gains in APPROACH/FLARE only,
   or write our own pitch loop.
2. The approach's 20-40 m/s sink at the doors.
3. The hypersonic reversal.

## Traps paid this session

- **Six instances swap.** zram holds 8-13 GB right after boot (~4 GB per
  instance), and it grew to 14 GB over three rounds without a restart; that
  round ran 514 s against ~430. **Restart the farm before every comparison.**
- **A seventh instance** (`ksp6` exists: cloned, matches `base/`, ports
  50112/50113) saturated the CPU while booting with the others (load 31 on
  16 threads). The user stopped it. Don't start seven without asking.
- **A rejected tool call had already run** (the 7-instance start). Check state
  after a reject, again.
- **A background script that `cd`s needs absolute paths.** The CPU sampler
  wrote nowhere and would have looped forever. `pkill -f` matched its own
  shell (exit 144).
- **The suite was run once beside an idle farm.** Don't: stop the farm first.
- **Old waiters from earlier sessions fire task notifications.** Check the
  file name before reading anything into one.
