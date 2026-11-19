# HANDOFF -- read this first, rewrite it last

**Split per pilot (2026-11-18, to cut tokens):** this file holds only what every session needs. Each pilot's
state is in `docs/<pilot>/handoff.md`: read the one for the pilot you will work on, and **write that pilot's
section there, not here.** Only cross-pilot items go here.

| pilot | file | last worked | where it stands / next |
|---|---|---|---|
| boosterland | docs/boosterland/handoff.md | 2026-11-18 eve | coast steering fixed (mirrored lift probe, solve root); new fin2 separation; nsep-1118 median miss ~6 m, 18/24 landed; next: read 2 outliers |
| ascent | docs/ascent/handoff.md | 2026-11-18 eve | re-flown: asc-1118 separations on the corrected budget land 2-96 m with 21.5-23.4 t as predicted; ksp3's is the new qs_finbooster2 |
| recon | docs/recon/handoff.md | 2026-11-18 | QUDRAC flies the mission; blocker: nose slam at main-gear contact; not pushed |
| airplane | docs/airplane/handoff.md | 2026-11-08 | crash guard deleted; screening batch to re-fly on 4 instances; not pushed |
| propPitch | docs/propPitch/handoff.md | 2026-11-04 | tip limit + stop-when-still defaults; the user flies it live |
| spaceplane | docs/spaceplane/handoff.md | 2026-11-01 | APPROACH_BEST_LD deleted; next: fly heavy-1101 |
| tooling | docs/handoffTooling.md | 2026-11-18 | event-triggered procedures moved out of the docs into hooks (CLAUDE.md "Procedures tied to an event"): batchHooks.py via farmlaunch pre/post, claudeHooks.py, githooks pre-push; 4 howItWorks PDFs stale (pre-push lists them) |

Farm state: MiniSystem and Retooled since 2026-11-06 / 11-13 (banner at the top of CLAUDE.md). Retooled flights
start at LOG16368; earlier ones (stock 1.12.5) are not comparable.

## 2026-11-18 evening: logs written unpadded (common/logbook.py)

`Logbook` now strips column padding from every line (`[3575.76] COAST alt=16697 vs=-574.8`, not
`[   3575.76] COAST        alt=   16697 vs= -574.8`): ~6 % fewer bytes, nothing lost. The user never reads the
logs; they are for Claude. Every reader in the repo already matched `key=\s*value`; logsum, budgetcheck, burnplan,
replay, shapetrack, flightsum give identical output on tightened copies. **Readers must not assume fixed columns.**
Key=value per line was kept on purpose (grep by key, no column counting). Read logs through the pilots' summary
tools, not raw.

## Cross-pilot open items (kRPC 0.8, from boosterland 2026-11-14)

4. **kspSim is broken**: port `server.py`/`protocol.py`/`fastpb.py`/`wire.py` to 0.8 (Message envelope,
   ConnectionRequest/Response, request ids, one port, streams as uint64, Error.exception, Status ->
   ServerStatus struct), re-record `data/services.bin` (`tools/services.py`), port `attitude.py` to 0.8's
   AttitudeController (#1191 + #1348). Until then the suite has 2 errors (testPhysics wire codec) and the
   post-commit hook will report them.
5. **PyPy fast client**: `kspSim/fastclient.py` stands down under 0.8; coast ticks cost ~30% more wall time
   (0.08 -> 0.105 s). Port it to 0.8's Session (or measure whether CPython+C protobuf is now faster).
6. **Other pilots on 0.8, unflown**: spaceplane, airplane, propPitch (LimitAngularVelocity on by default, surface
   torque and MoI fixed -- their tables and kRPC-dependent laws), ascent (KRPC.MechJeb now ships with kRPC; one
   smoke flight of qs_stack). Fly each pilot's standard batch before trusting its defaults.
7. **Live**: the user flies the 0.8 defaults (latch reset, pace off). START SCRIPT / launcherMod under 0.8 not
   exercised live yet. Live game has no GimbalTorqueFix (stock GetPotentialTorque over-credits 1.25-1.5x):
   offer `gimbalTorqueFix/build.sh --live` (its torque patch still applies under 0.8).
8. **When 0.8.0 is released**: switch `tools/krpcmain.sh` to the release (mod zip + `pip install krpc==0.8.0`),
   diff it against 34c70de, re-fly.
9. **Disk outside krpcLab (the user's call)**: Bazel's cache is 17 GB in `~/.cache/bazel` (`cd ~/krpc && bazel
   clean --expunge` frees it; the next `krpcmain.sh` re-downloads, ~4 min); bazelisk in `~/.local/bin/bazel`;
   `.krpc070/` (50 MB, gitignored) holds the 0.7.0 installs for a rollback.

## Traps from before (still true)

- Read the save name off each log before comparing.
- A `&&` chain ending in `| tail` commits on a failing test -- check the
  suite's own exit status.
- landsum drops some logs; for outcome counts use the harness lines in
  `logs/rot-*.txt` joined to each log's `config:` line.
- kspSim has no `qs_shuttle2_rigoff` model; its subsonic lift is wrong (gap
  8) -- don't screen cone energy there.
- Six fresh instances put 10-17 GB into zram at boot; 4 cycles reach ~22.
- Kill sims with `pkill -f "kspSim[.]run"`.
- `warp_to` is blocking: the panel does not answer during it.

A global SessionStart hook (`~/.claude/hooks/pushOnStart.sh`) pushes
unpushed commits at session start; read its `pushOnStart:` message.
