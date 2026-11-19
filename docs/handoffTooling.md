# Tooling HANDOFF (older cross-cutting sections) -- read when touching launcherMod, the bundle or the farm harnesses

## 2026-11-18 night (2): batch-end notice, Stop hook, relay timing

- **Why no notification after live1118b:** the relay was armed at ETA+3 (20:07), the batch ended 20:03, and the
  PushNotification step runs only in the woken session. Fixes: relays now fire at launch+1 min and `detachjob wait`
  (farmlaunch's exit-76 text, autoContinue.md). `batchHooks.post` also runs `notify-send` at once (505290a).
  **Desktop notifications are down system-wide:** nobody owns `org.freedesktop.Notifications` on the session bus
  (plasmashell lost it; `gdbus ... GetServerInformation` hangs). The user's desktop, not touched. PushNotification
  (phone via Remote Control) is the working path.
- **Stop hook** (`claudeHooks.py stop`, ecd19b3; the user: "make sure you don't stop when there's work left"):
  blocks the end of a turn when tracked files are uncommitted, a batch ended with no handoff commit since, or
  nothing is flying and the turn ends neither on a question nor on `DONE: <why>`. Passes relay runs and the wait
  on a running detachjob; gives up after 4 blocks in a row (`logs/.stopHook.json`).
- **Context-limit order false trigger:** `BATCH_LAUNCH` matched `brot.sh` inside a heredoc. `batch_launched` now
  ignores heredoc bodies and quoted strings.

## 2026-11-18 night: the end of a batch with no next batch notifies before the clear (`bedeec3`)

- nsep1118 finished at 18:01. The waiting session woke, wrote the boosterland handoff (`d9815b2`) and cleared
  itself at 18:05. The clear erased its report and left the stale title `⏳ ... ~18:03`, so the user could not
  tell the batch was done. The wake itself worked: it was an in-conversation `run_in_background` wait, so no
  relay task was armed.
- `testInstances/batchHooks.py` batch-end step 4 (no next batch) now says: PushNotification with the result
  and "nothing flying", re-title without ⏳, then clear. Not exercised yet: the next batch end is its test.

## 2026-11-04 late: the window mod is the only UI; the launcher survives scene changes (airplane `570902c5`, propPitch `ca612601`)

The user's request, all done and committed; **nothing here was flown or
run in a game** (the user: "no need to test this"; they test live).

- **`launcherMod` draws everything**: the launcher's menu and every pilot's
  panel (`common/panel.py` is now a client of it: title, telemetry text,
  and controls -- buttons, checkboxes, number fields, choices -- sent as
  `kind|id|label|value` strings and read back by one kRPC stream of
  `PanelRead`). The kRPC-UI panel (`panel.window`, the glyph-drawn frame)
  and `LauncherPanel` are deleted. Without the mod a pilot's panel is
  headless (kspSim; START never pressed, so fly with `--autostart`).
- **Scene changes**: the mod's `EpochWatcher` (an `Instantly` addon kept
  across scenes) counts `onGameSceneLoadRequested` and `onVesselChange`;
  the launcher polls `Epoch()` while a pilot flies and, on a change, stops
  it (SIGINT, SIGKILL after 3 s), takes its panel down and shows the menu.
  The window and toolbar button exist in flight only; the launcher keeps
  running through the space centre / SPH. A pilot's own load or switch is
  announced first with `panel.expect_scene_change(conn, s)` (fmrs: 90 s per
  load; ascent's booster handoff: 10 s).
- **START SCRIPT** (the window with no launcher running): starts kRPC's
  servers (`KRPC.Core.Instance.StartAll`) and runs `env -i HOME=.. PATH=..
  KRPCLAB_GAME_PID=<pid> /bin/bash -lc "cd <root> && exec ./run.sh"`
  (`Z:\usr\bin\env` under Wine; the pid from `/proc/self/stat`).
  `build.sh` writes `GameData/KrpcLabLauncher/PluginData/launch.cfg`
  (`root=`, `home=`). **Unverified**: that Wine's CreateProcess execs a
  Unix binary from Mono's `Process.Start` -- if START SCRIPT says it could
  not run, that is the place; the window shows the exception text.
- **The game's death**: the launcher exits when a kRPC call raises a
  connection error or `KRPCLAB_GAME_PID` is gone, killing the pilot first;
  pilots also get `PR_SET_PDEATHSIG` SIGTERM from the launcher.

**Traps paid:** `tools/bundle.py` only counts identifiers in *git-tracked*
files as reachable from outside a module; an untracked new package's
imports get renamed away ("cannot import name ACCENT_AIRPLANE"). `git add`
a new package before building the bundle.

## 2026-10-27: tooling and the farm harnesses; boosterland's farm entry state (bundle 369 kB)

No pilot's flight laws changed.  What did:

- **Bundle 379 -> 369 kB**: C# `name=` no longer pins parameters, test strings
  no longer pin `vec`/`math`, annotations read builtins through aliases,
  `SimpleNamespace`/`Thread` keywords no longer pin parameters.  Post-commit
  hook: its suite bundle was written into the root and failed every commit
  ("came from the tree"); fixed, `post-merge` calls `post-commit`.
- **Dead code deleted**: spaceplane `hac_available`, `airframe.glide_ld`;
  boosterland `landing_attitude`, `terminal_attitude`; `idle.rescan`.
- **Every farm harness flies the bundle** through `tools/farmflight.py`
  (launch in its own process group, reaper, `claim_log` by the pid now in
  every log header); `--tree` flies the modules.  `fly.sh` runs PyPy.
- **From the bundle, `paths.ROOT` was `dist/`**: the launcher's children, the
  spaceplane's logs and the gear cache would have gone into `dist/`.  Fixed
  (a checkout's `dist/` roots in the tree; a copied bundle keeps its own dir).
  Nobody had flown live from the bundle before this fix.
- **quickfly** no longer runs the autoland in-process: the child loads the save
  (`autoland --load`), the instance is held at 1x through the load and the
  child's start-up, the first tick waits for save UT + 4.6 (`--start-ut`), then
  the harness raises the scale.  Every flight engages at 16681.78 +-0.02.

**Boosterland finding (open, not chased):** a save loaded at 1x is a different
booster from one loaded at 4x -- hs 699.6 m/s at 16681.78 every flight, against
701.7-702.8 for the old 4x load -- and on `quicksave` it lands **100-360 m
west** (e1-cpy: 19/64/69/170/259/361 m) where the 4x-loaded state landed
23-155 m, mostly within 60 (e1-old, interleaved on a fresh farm).  Neither the
bundle nor PyPy is the cause (same result under `--cpython`).  The 1x state is
what a live game flies, so the farm booster numbers before today were measured
on a 4x artefact.  Next for the booster: measure over several saves at the new
state; find what a 2.6 m/s entry difference does that the boostback does not
null (the coast miss is ~180 m in both).  Logs: `logs/harness-1027/`.

**Trap paid:** the farm reached 22 GB of swap after an hour of six-instance
booster flights and every comparison flown on it was worthless -- restart
between comparisons, as the farm section says.
