# testInstances -- parallel KSP instances for unattended measurement flights

Independent, graphics-stripped copies of the KSP install, each with its own
kRPC port, so several `quickfly.py` runs can fly at once.  The live install at
`~/Kerbal Space Program` is only ever **read**.

## Layout

    HANDOFF.md     start here: what works, what is measured, what is not
    keepNotes.md  why it is built this way, and the traps already paid for
    strip.txt      the 31 GameData directories that are not copied

    mkbase.sh      build base/ from the live install (once)
    mkclone.sh N   make ksp<N> + prefix<N> + its launcher
    kwinRun.sh N  run instance N unattended in its own virtual compositor
    fly.sh N ...   run the project's quickfly.py against instance N
    stop.sh [N...] stop instances (all of them if given no argument)

    setports.py    rewrite kRPC's nested port nodes (used by mkclone.sh)
    spawn.sh N...  older gamescope-based starter; prefer kwinRun.sh
    autoloadSrc/  the main-menu autoload plugin, built by mkbase.sh

## Setup

    ./mkbase.sh              # ~7.5 GB, a few minutes
    ./mkclone.sh 0
    ./mkclone.sh 1           # ... as many as you want

## Use

    ./kwinRun.sh 0 &        # unattended, invisible; ~2 min to the space centre
    ./kwinRun.sh 1 &
    # wait for the ports:  ss -ltn | grep 5010
    ./fly.sh 0 -n 3 &
    ./fly.sh 1 -n 3 --set AIM_BIAS_EAST_M=0 &
    wait
    ./stop.sh

`kwinRun.sh N` is the launcher for real work: it gives the instance its own
`kwin_wayland --virtual` compositor, so there is no window, no display
contention and no focus to lose.  `run-ksp.sh N` uses gamescope
(`BACKEND=sdl|headless|none`) and is for watching one instance interactively.

An instance takes roughly a minute to the main menu and another minute for the
plugin to load the quicksave into the space centre; `fly.sh` refuses to run
until that instance's port is listening.  First boot on a fresh clone is slower
because its DXVK and NVIDIA shader caches are empty.

`spawn.sh` predates `kwinRun.sh` and starts instances through `run-ksp.sh`;
prefer `kwinRun.sh` for unattended runs.

Logs still land in the project's own `logs/LOG<n>`, and the counter is shared,
so two instances flying at once interleave their log numbers.  Read the header
of each log to see which config produced it.

## Disk

`base/` is 7.5 GB of real files.  Each clone is `cp -al` hardlinked against it
and then has the link broken on every file KSP might rewrite in place, so a
clone costs about 10 MB.  `base/` is never launched, so nothing writes to the
shared inodes.  Each clone does get its own 1.3 GB wine prefix, because wine
rewrites `system.reg`/`user.reg` in place and a shared prefix would have N
processes racing on them -- and, separately, a second `umu-run` against a busy
prefix simply blocks on `wineserver -w` instead of starting.

## Ports

Instance N listens on rpc `50100 + 2N`, stream `50101 + 2N`.  The normal
install's 50000/50001 is deliberately left alone, so you can play while these
fly.  `mkbase.sh` also sets `autoStartServers = True`, which the interactive
install does not need but an unattended one does.

## What was stripped

31 rendering/audio/skybox/IVA/UI mods, 5.9 GB: GameData drops from 9.4 GB to
3.1 GB.  Every one was checked against the 20 part names in the booster's
craft file for patches touching mesh, MODEL, DRAG_CUBE, mass or
lifting-surface keys.  **ReStock is deliberately kept** -- it rewrites mass and
drag cubes on 15 of those 20 parts, so removing it would change the vehicle the
calibration was measured on.  See `keepNotes.md`.

Removing these cannot break the `.craft` file: every part in it is stock.  What
is dropped is cosmetic PartModule state (recolouring, plumes, sounds), which
KSP skips with a log line.  See `keepNotes.md` for the one real caveat --
write-back.

## Why there is a plugin in here

`autoloadSrc/` builds `BoosterlandAutoLoad.dll`, ~40 lines of C#, installed
into every instance as `GameData/BoosterlandAutoLoad/`.  It exists because of
a hard constraint in kRPC, found by disassembling `KRPC.dll`:

    scene = CurrentGameScene()
    if (scene == GameScene.None) { core.StopAll(); return; }
    if (config.AutoStartServers)  core.StartAll();

The **main menu is `GameScene.None`**, so kRPC deliberately stops its servers
there.  `autoStartServers = True` is necessary but not sufficient: an instance
parked at the menu never opens its port, and nothing can connect to tell it to
load a save.  Chicken and egg.

So the plugin loads `default/quicksave` from the main menu, which puts the game
in `FLIGHT`, which is when kRPC starts listening.  `quickfly.py` then reloads
the save itself for every flight, exactly as it does interactively.

It is `[KSPAddon(KSPAddon.Startup.MainMenu, true)]` -- it runs once, at the
menu, and never again -- so it cannot influence a flight.  The save it loads is
configurable in `GameData/BoosterlandAutoLoad/autoload.cfg`.

One subtlety already paid for: **do not call `game.Start()` from inside the
addon's `Start()`**.  The menu is still constructing at that point and the
flight scene load stalls silently.  The plugin defers to a coroutine and waits
for the menu to settle first.

Rebuild it with:

    mcs -target:library -out:autoloadSrc/BoosterlandAutoLoad.dll \
        -r:base/KSP_x64_Data/Managed/Assembly-CSharp.dll \
        -r:base/KSP_x64_Data/Managed/UnityEngine.dll \
        -r:base/KSP_x64_Data/Managed/UnityEngine.CoreModule.dll \
        autoloadSrc/AutoLoadSave.cs

## Rendering

Each instance renders at 640x360 with `QUALITY_PRESET 0`, no AA, no shadows and
`FRAMERATE_LIMIT 60`.

**`FRAMERATE_LIMIT` is a floor as much as a ceiling, and must not go low.**
`PHYSICS_FRAME_DT_LIMIT = 0.04` becomes Unity's `Time.maximumDeltaTime`, and
`fixedDeltaTime` is `0.02`, so KSP runs **at most 2 physics steps per rendered
frame**.  Real-time physics therefore needs >= 25 fps.  At 1 fps you would get
0.04 game-seconds per real second -- a 3-minute flight would take 75 minutes --
and kRPC, which is serviced from the frame loop, would drop the autoland's
20 Hz control loop to 1 Hz and change the flight.  60 leaves 2.4x headroom.

`PHYSICS_FRAME_DT_LIMIT` itself is never touched: it sets the timestep, and
the timestep is the physics.

## Measured: the strip costs nothing detectable

Three flights on a stripped instance at the committed config
(`AIM_BIAS_EAST_M = 628`, quicksave UT 16677), against the known result for
that save of **12 / 115 / 131 / 10 m** from the pad:

| | log | from pad | N | E | arrival |
|---|---|---|---|---|---|
| 1 | LOG67 | 74 m | +4 | -74 | 0.5 m/s, `legs=0.1m vs=-2.1` |
| 2 | LOG68 | 66 m | -5 | -66 | 0.2 m/s |
| 3 | LOG69 | 52 m | +0 | -52 | 0.4 m/s |

Median 66 m, mean 64 m against the reference group's mean of 67 m -- the centre
did not move.  All three intact.

The scatter is much tighter than the reference (52-74 m against 10-131 m),
which would be worth a great deal if it holds -- see CLAUDE.md failure 12,
where the spread is set by the boostback exit tick.  Three flights against
four, so treat it as a lead, not a result.

## Parallel instances work, and contention may not be free

Two instances ran at once, each in its own `kwin_wayland --virtual`, own wine
prefix, own kRPC port -- ksp1 on 50102 flying while ksp0 loaded, auto-loaded
and then flew on 50100.  Machine load stayed under 1.7 with 16 cores and about
9 GB of 30 GB used, so four instances look comfortable (untested).

The one concurrent flight is a warning, though:

| | log | from pad | arrival |
|---|---|---|---|
| ksp1 solo x3 | LOG67-69 | 52 / 66 / 74 m | 0.2-0.5 m/s |
| ksp0 concurrent x1 | LOG70 | 111 m | 1.5 m/s |

111 m is inside the original reference spread but outside the tight solo group,
and it arrived three times faster.  **n=1, so this is a hint, not a result** --
but it is the hint to expect.  A flight is fixed-timestep, so contention cannot
change the physics; what it can do is slow the game below real time, and the
autoland's control loop sleeps on wall time, so a loaded machine gives the loop
more ticks per game second.

So: **compare parallel flights only against other parallel flights**, and
before trusting a parallel sweep, fly the same config solo and concurrently a
few times each and see whether the landing point moves.  If it does, either
pin the instance count for every comparison or go back to running one at a
time.

## Verifying an instance

Do not assume the stripped instance flies the same profile -- measure it, the
same way everything else in this project is measured.  Fly three at the
committed config and compare against the known result for this quicksave
(UT 16677, `AIM_BIAS_EAST_M = 628`): **12, 115, 131 and 10 m from the pad**,
all intact.  Landing inside that spread means the strip changed nothing that
matters.  Landing outside it means something in `strip.txt` did, and the log
will say where the trajectory diverged.

## First boot in a new instance is slow, and that is not a hang

A fresh clone has an empty DXVK state cache and an empty NVIDIA shader cache
(both live in `prefix<N>/`), so the first launch compiles shaders as it goes.
The visible symptom is the log going quiet for a minute or two at around
`[MechJeb2] Loaded Shaders Bundles` while the process sits at about half a
core.  It is working; leave it.

Telling the two apart is worth knowing, because the FastLoader dialog in
`keepNotes.md` looks superficially identical:

    pid=$(pgrep -f "ksp<N>/KSP_x64.exe")
    a=$(awk '{print $14+$15}' /proc/$pid/stat); sleep 10
    b=$(awk '{print $14+$15}' /proc/$pid/stat); echo $((b-a))

Roughly 500 ticks per 10 s means it is compiling and will finish.  Under about
50, with nothing being written to disk, it is blocked on something -- check
`GameData/*/PluginData` against the live install first.

Subsequent boots on the same instance reuse the caches and reach the flight
scene in well under a minute.
