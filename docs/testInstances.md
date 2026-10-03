# The test-instance farm (`testInstances/`)

Independent graphics-stripped KSP copies for flying several measurements at
once, and the operational failures that farm has produced.

In-game measurement is the bottleneck: a flight is about three minutes and the
useful experiments are three flights a configuration. `testInstances/` holds
independent, graphics-stripped copies of the KSP install, each with its own
kRPC port. The live install at `~/Kerbal Space Program` is only ever **read**;
`testInstances/README.md` and `keepNotes.md` carry the detail.

`mkbase.sh` builds `base/` by copying the install without the mods in
`strip.txt` — GameData drops 9.4 GB to 2.4 GB, kRPC open in 39 s against 61 s.
A second pass took the last 1.2 GB out of mods that *stay*: the surface
textures and plume models of ReStock and Squad, whose part meshes — and so
the drag cubes and masses — are untouched. Three mods that look like GUI tools
were put back because they replace stock PartModules on the flying vessel;
`keepNotes.md` has the table. `mkclone.sh N` hardlinks a clone off `base/` and
breaks the links on every file KSP rewrites, so an instance costs ~10 MB plus
its own 1.3 GB wine prefix. Ports are `50100 + 2N`, clear of 50000/50001.

Three things about this are load-bearing.

**`ReStock` is not a texture mod and stays.** It carries `!mesh = DELETE`, a
replacement `MODEL`, `DRAG_CUBE`/`!DRAG_CUBE` and `@mass` on **15 of the 20**
parts this booster uses. KSP derives a part's drag cube from its mesh, so
removing it changes the vehicle's mass and drag area — the very quantity the
estimator measures. Everything else in `strip.txt` was checked against the
craft's 20 part names for patches touching mesh/MODEL/DRAG_CUBE/mass and
touches none. Parallax only adds `subdivisionLevel = 7` on Kerbin.

**Removing mods cannot break the `.craft`, but re-saving it can.** Every part
is stock, so nothing can be orphaned; what is dropped is cosmetic PartModule
state (`ModuleReforged`, `ModuleWaterfallFX`, `RSE_*`). But a stripped instance
that *re-saves* a craft writes those modules out of the file permanently. Hence
the private `saves/` and `Ships/` per clone, and `base/` is never launched.

**An unattended instance has to load a save by itself.** `KRPC.Addon` calls
`Core.StopAll()` whenever `CurrentGameScene()` is `GameScene.None`, and the
main menu is `None` — so `autoStartServers = True` is necessary but not
sufficient, and an instance parked at the menu never opens its port.
`GameData/BoosterlandAutoLoad` (built from `autoloadSrc/` by `mkbase.sh`)
loads the quicksave from the menu into `SPACECENTER`. It is
`[KSPAddon(Startup.MainMenu, true)]`, so it cannot influence a flight. Two
traps in it, both paid for:

- **`HighLogic.CurrentGame = game` before `game.Start()`.** Without it
  `Game.Start()` throws a `NullReferenceException` internally, Unity swallows
  it per callback, and the game renders the main menu at 60 fps forever. That
  looks exactly like a frozen process and is not one.
- **`SPACECENTER`, not `FLIGHT`.** `Game.Start()` into `FLIGHT` needs
  `FlightDriver` primed with a valid active vessel and silently does not
  transition without it.

And one in the harness: `mkbase.sh` must **not** wipe `GameData/*/PluginData`.
It holds mods' answers to first-run prompts, and KSPCommunityFixes asks about
its FastLoader texture cache with a modal dialog *during the loading screen*,
which an off-screen instance can never click. The stall is silent — no error,
no CPU, no log line. Before bisecting a mod list for a hang, diff
`GameData/*/PluginData` against the live install.

`kwinRun.sh N` runs an instance inside its own `kwin_wayland --virtual`
compositor: no visible window, no display contention. `run-ksp.sh N`
(gamescope, `BACKEND=sdl|headless|none`) is for watching one interactively.
`start.sh` launches each in its own `systemd-run --user --scope`.

**Flights can run faster than real time without coarsening the physics step.**
`testInstances/timescaleSrc` raises `Time.timeScale` while leaving
`Time.fixedDeltaTime` at 0.02, which is the opposite of what KSP's own physics
warp does; `./tools/timescale.py N max` turns it on and reports what was *achieved*
rather than what was commanded. The ceiling is control quantization
(`timeScale / fps`), not the CPU, so `SYNC_VBL = 0` and `FRAMERATE_LIMIT = 90`
matter more than the machine does — and frames above what the quantum needs
come straight out of the speedup. Measured: 175 s/flight to 47 s. The control
loops must be paced on `ut` for this to be honest (`LOOP_PACING_GAME_TIME`);
see `keepNotes.md`, "Flying faster than real time".

**And pacing the loop on `ut` is not enough, because the loop cannot always
keep up with it.** A tick costs 20-60 ms of wall clock whatever the game
clock is doing, so at 6x a phase that asks for a command every 0.1 game-second
gets one every two. `Config.TIMESCALE_GOVERNOR` closes that: the autopilot
measures its own tick cost and writes back the fastest scale that still serves
the interval the current phase asked for — the ceiling passed to
`--timescale` is then a ceiling and not a setting. `quickglide.py` wires it up
whenever `--timescale` is given; `--no-govern` turns it off. Measured against
itself on `qs_plane` — same save, same defaults, same farm state — it costs
nothing in throughput (5.0-6.0x achieved) and takes the entry's arrival from
−3226 m sd 806 to +294 m sd 171. The whole effect is in the **deorbit burn**,
whose tick goes 0.30 → 0.10 game-seconds; every other phase was already being
served at 6x on this machine. See docs/spaceplane/design.md, "The farm was measuring
its own tick latency, in one phase".

### The saves are per-instance, and they drift

CLAUDE.md's farm rule is about `GameData`. It applies to `saves/default/`
just as hard, and that half is easier to get wrong because nothing warns you:
`savegen.py` and `entrysave.py` write into **one** instance, and every clone
keeps its own copy of every quicksave.

Measured on 2026-09-16: `qs_plane.sfs` was byte-identical on all six
instances (the flights from it are comparable), while the four `qs_cone.sfs`
files had **four different md5s** — one had been re-taken three minutes after
the others. A session's worth of cone flights was read as "the glide scatters
8 km from one state" when it was four different states.

So before a batch:

```bash
md5sum testInstances/ksp*/saves/default/<save>.sfs | awk '{print $1}' | sort -u | wc -l
```

One line of output means one experiment. More than one means copy the
canonical instance's save over the rest and start again — a save is a file,
not a measurement, and copying it is free.

**Re-measured 2026-09-21, and `qs_plane` has since drifted apart**: ksp0-3
carry one file and ksp4-5 another. Everything that session flew used 0-3
and is safe, but a batch that reaches for a fifth instance is not comparing
one state. `qs_plane_inc` is identical on all six; `qs_e60` and the rest of
the `savegen.py` ladder exist on **four only**, so `--instances 0,4` on
those silently flies a different save or none. Check both things — the md5s
*and* whether the file is on every instance you asked for.

The log now carries an `instance:` line for the same reason: a batch spread
across the farm could not be split by instance after the fact, and that split
is what separates "this loop amplifies" from "these two games are not the
same".

### Killing the harness does not kill the flight

`quickglide.py` runs the autopilot as a **subprocess**, so `pkill -f
quickglide.py` matches the parent and leaves the child flying -- still
connected, still commanding the vessel. Two killed batches on 2026-09-16
left **eight** autopilots alive across four instances, three of them on one
port, fighting each other over one aircraft.

Fixed at the source: the child is started in its own session and reaped in a
`finally`, so an interrupted harness takes its flight with it. The check, if
an instance is behaving strangely:

```bash
pgrep -af spaceplane.autopilot        # should be one per busy instance
```

More than one on a port means a batch was killed and the old flight is still
running; `pkill -9 -f spaceplane.autopilot` before measuring anything.

**Three ways a batch lies about itself, all paid for in one session:**

- `pkill -f spaceplane.autopilot` **matches the shell running it.** The
  pattern is compared against every command line including your own, so a
  cleanup line inside a compound command kills the command. Twice in one
  session this silently killed the launcher and the batch never started --
  and "no output" looks exactly like "nothing to clean up". List the pids
  with `ps -eo pid,args | grep "[s]paceplane.autopilot"` and kill those.
- **One `OUT` file per batch.** Reusing it appends the next batch to the
  last one's summary, and nothing in the file says where one ends and the
  next begins -- three attempts merged into one table of nine "results"
  that were three different configurations.
- **`farmfly.sh` refuses to start on top of live flights and then prints
  `done` seconds later.** A batch that never ran looks exactly like one that
  finished, unless the log is read rather than the timestamp.

**And do not edit the flight code while a batch is running.** `farmfly.sh`
starts a *fresh process per flight*, so a change saved halfway through is
flown by the flights after it and not by the ones before -- one arm, two
configurations, and nothing in the summary says which flight got which. The
`config:` line would only show it if the change moved a field, and a change
to a control law moves nothing: the `defaults_fingerprint` shifts, which is
the one column that would notice, and only if someone thinks to compare it
across flights *within* a batch rather than between batches.

The batch is minutes. Wait for it, and keep doc edits for the gap.

**What it produces is not a crash, it is plausible telemetry.** The next
batch is *wrong without looking wrong*. Twice on 2026-09-16:

- a cone batch started four seconds after a kill produced `vessel: 0 parts
  mass 0.000 t` and flights frozen at `alt=122 v=221 vs=-211`, which at least
  announces itself;
- a deorbit arm started four minutes after a kill produced three flights
  arriving **+30, +42 and +61 km**, on the normal pass, against a baseline of
  +2.7 km with sd 10.7. Nothing in those logs is malformed. They read as a
  catastrophic regression in the change under test, and the change was
  innocent -- rerun clean, the same arm's first flights arrived **+19 m,
  -2012 m, +19 m**.

The second kind is the dangerous one: it is a 4-sigma result that will be
believed, and it points at whatever was changed most recently. If a batch has
to be killed, let the instances load a quicksave from an *idle* run before
measuring on them again, or restart them.

### A farm that has been up all day is a different farm

Four instances hold 3.0-3.5 GB each when fresh. Measured on 2026-09-16 after
**seven hours** of continuous flying they were still reporting the same RSS
while the box held **15.5 GB in zram** and the load average sat above 10.
Stopping them dropped memory used from 23 GB to 4.5 and swap from 15.5 GB to
1.9.

What that does to a measurement is not subtle and is not noise:

```
                       arrivals over the field, n>=4, same configuration
after 7 h, swapping      mean +25 km    (+10.8, +18.6, +32.0, +58.9)
restarted, 1.9 GB swap   mean  -2 km    (-25.0,  -1.0,  +6.4, +10.8)
```

The spread is unchanged; the **bias** moves 27 km. So a swapping farm does not
merely add scatter, it shifts the answer, and it shifts it in the direction
that looks like a guidance regression. Two arms flown either side of the
degradation are not comparable however carefully the configuration was
controlled.

Check `free -m` and `swapon --show` before a batch *and after it* -- the
before-check passes on a farm that will be thrashing by round six. Restart
the instances between long sessions; they are cheap to restart and the
alternative is a day of measurements that disagree with each other.

### How fast an instance goes, and what is stopping it

`./tools/instancebench.py N --save qs_plane --quant 0.05,0.1,0.2` reports, per
setting, the multiplier achieved **and which of the two ceilings is binding**.
That distinction is the whole point: from inside the game both look like "it
is going at 4x", and they want opposite fixes.

- **Quantization.** Unity applies control input once a frame, so
  `timeScale / fps` is the grid every autopilot command lands on. Binding
  here means the instance is frame-starved and wants *more frames*.
- **CPU.** Physics is single-threaded and runs `timeScale / fixed_dt` steps a
  second. Binding here means more frames would make it *worse*, because
  frames and physics share the core.

Tell them apart by **physics steps per real second**, not by the multiplier.
Past the CPU ceiling the frame rate collapses and drags the computed
allowance (`quant_s * fps`) down with it, so a CPU-bound instance reports a
multiplier far *above* its own allowance and reads as frame-starved. That is
what `instancebench.py` said first, on every row that mattered.

Three things were found by measuring rather than assuming, and together they
took an instance from 4x to 7x with a *finer* control grid than before:

- **`FRAMERATE_LIMIT = 90` was the binding constraint, and it is free to
  raise.** At `quant_s = 0.05` the allowance is `0.05 x 90 = 4.5`, which is
  exactly where instances sat. Frames on a 900x520 off-screen instance are
  cheap; at 500 the same instance holds 6x at 177 fps, and every frame above
  what the quantum needs converts directly into speedup. `base/settings.cfg`
  carries 500 now.
- **The plugin's own governor was unstable, and asking for more made it
  slower.** `Apply()` held `timeScale = min(requested, quant_s * fps)` -- but
  fps is a *function of* timeScale, because a frame simulating more game time
  costs more to render. Below the boundary the clamp never binds; the moment
  it binds it lowers timeScale, which raises fps, which raises the allowance,
  which raises timeScale again. Measured, holding a fixed multiplier:

  | asked | held | fps |
  |---|---|---|
  | 4 | 4.00 | 253 |
  | 6 | 6.01 | 177 |
  | 8 | **4.10** | 248 |
  | 10 | **1.33** | 397 |

  and the status file reported the collapsed number as though it were the
  machine's limit. The clamp was also redundant: `Time.maximumDeltaTime =
  quant_s` already caps game-time-per-frame, so the grid holds whatever
  timeScale is. It is gone; if the main thread cannot keep up, `achieved`
  falls below `commanded` and says so, which is what the caller needs.
- **`Time.maximumDeltaTime` is in *unscaled* seconds, so setting it to
  `quant_s` never bounded the grid at all.** Unity clamps the *real* frame
  time to it and then multiplies by timeScale, so the game-time grid is
  `min(realFrame, maximumDeltaTime) * timeScale`. At 8x and 80 fps the real
  frame is 0.0125 s, nowhere near a 0.05 clamp, and the grid came out at
  `0.0125 * 8 = 0.0997` -- twice what was asked -- while the status file
  reported `max_dt=0.05` as though the guarantee were holding. It is set to
  `quant_s / timeScale` now, which does bound it. Holding a grid needs
  `fps >= target / quant_s`; below that the game runs slow, and that is the
  honest half of the trade.
- **Benchmark the phase you care about, not the cheap one.**
  `instancebench.py` parks the vessel in orbit, where it is on rails with no
  aerodynamics, and an instance holds 7.9x there with a 0.133 s grid. Flying
  an actual entry the *same* instance holds **6.1x at a 0.211 s grid** --
  23 parts in thickening air cost a third of the multiplier. The orbital
  number is the one that is easy to measure and the atmospheric one is the
  one the flight spends its time in, so size a multiplier against a real
  entry before believing it.
- **`quant_s` is per-*project*, not per-machine.** 0.05 s is sized for
  boosterland, whose command interval is 0.05 s. The spaceplane's glide tick
  is ~2.4 game-seconds, so it can afford ten times that. Always read the
  `grid` column back rather than trusting the request.

**The plugin is a `KSPAddon.Startup.Flight` addon, and KSP loads plugins
once, at startup.** Two consequences, both of which have quietly produced
flights that ran at 1x while the harness believed 4x:

- An instance booted **before** `BoosterlandTimeScale.dll` was installed
  ignores `timescale.txt` forever. Restart it; nothing else works, and
  nothing anywhere logs an error.
- It writes `timescale-status.txt` only with a **vessel in the scene**, so a
  freshly booted instance parked at the space centre has no status yet and
  that is not a fault. Check after loading the save, not before.

`quickfly.py` and `quickglide.py` both print the *achieved* multiplier beside
the result for this reason, and `quickglide.py` refuses to fly at all when a
timescale was asked for and the plugin is silent with a vessel loaded — an
hour of flights that silently ran at 1x is an hour of measurements that lie in
the direction you were hoping for.

### Stripping parts saves RAM and does not work yet

Instances compile **489 parts** at load against the **26** that every vehicle
in the farm's thirteen quicksaves put together uses, and RAM is what caps the
instance count -- which is the thing that actually multiplies throughput.
`partstrip.py` derives the keep-list from a clone's own saves and prunes the
rest. **It currently wedges the instance**, and the reason is worth knowing
before anyone tries again.

A stripped ksp4 never finished loading: the log froze at *Compiling Internal
Prop*, the port never opened, and the process spun at 290% CPU for four
minutes. Ten lines earlier:

    PartCompiler: Cannot replace texture as cannot find texture 'blank'
    PartCompiler: Cannot replace texture as cannot find texture 'blank-n'

**The dependency graph crosses directories through ModuleManager patches.**
`ReStock/PatchesMH/Coupling/restock-mh-engineplates.cfg` does `texture = blank`
against a part that was *kept*, naming a texture that lived in a directory
that was removed. A directory-level keep-list cannot see that edge, because
the edge is not in either directory -- it is in a third mod's patch.

So a working strip has to resolve what the patches reference, not just what
the saves reference. Until then `partstrip.py` is a loaded gun: RSS did drop
from 4.26 GB to 1.97 GB, but that is the footprint of a **wedged** instance
and proves nothing about a working one.

Two things make it safe to experiment with anyway, and both are the point:
`mkclone.sh` hardlinks, so deleting inside a clone drops only that clone's
directory entries and `base/` keeps all 489 parts; and the script refuses to
run against `base/` at all. A bad strip costs one `mkclone.sh`.

**And the failure mode is the real objection.** Not that it broke -- that it
broke by *spinning silently* rather than erroring, on a farm whose whole
purpose is unattended runs. Anything that prunes assets needs a positive
check that the instance still reaches its port and still flies, not just that
it started.

**Hold a sleep inhibitor while flying** (`./nosleep.sh start|stop|status`). A
flight is minutes of unattended wall clock with nobody at the keyboard, which
is exactly what an idle timer waits for.

## When every instance dies at once

Four independent processes ending in the same second, each with an orderly
Unity `OnDestroy` cascade, is **not** four crashes — and it has four known
causes with the same signature. The check has to be *positive*; do not spend a
session hunting a reaper that does not exist.

```bash
journalctl --user -b -u systemd-suspend.service   # suspend?
last -x | head                                    # reboot?
coredumpctl list --since "-10min"                 # something crashed?
journalctl -b --since "-10min" | grep -iE "oomd|killed|plasma|baloo"
journalctl --user -b --since "-10min" | grep -E "systemd\[.*\]:.*(Started|Consumed)"
```

- **A suspend.** Before it, every process dies together; after the resume the
  game runs ~10x slower than real time until `quickfly.py` times out with the
  booster still at 32 km (`shutdown: TIMEOUT after 420s` on a flight that
  covered forty seconds of game time). Both signatures were misdiagnosed once
  each — as an external SIGTERM, and as CPU pressure.
- **The graphical session restarting.** `systemd-run --user --scope` isolates
  an instance from the *calling process tree*, not from the session: a browser
  taking a SIGSEGV, or merely being restarted, took all four KSP clients with
  it while the nested compositors survived, so the instances looked healthy
  from outside. The last line of the checklist is what turns "unexplained" into
  "explained". Cost: 15 flights of a 60-flight sweep, which is the argument for
  reading a sweep's partial table rather than re-flying it.
- **A GPU context loss, which wears the browser's costume.** The entry above
  blamed the browser, and at least once the browser was a fellow victim. The
  core dump's stack is the tell: `EGL_CONTEXT_LOST` through
  `libGLX_nvidia.so` / `libnvidia-glcore.so`, with the browser's own GPU
  process exiting 139 in the same second. A driver-level context loss
  invalidates **every** GL context on the machine at once, so every KSP
  client dies with it and the `--virtual` compositors -- which can rebuild
  theirs -- do not. Read the stack, not just the fact that something dumped
  core: "the browser crashed" and "the GPU reset and took the browser with
  it" produce the same `coredumpctl` line and have different fixes. Nothing
  in the farm can prevent this one; the instances are NVIDIA GL clients
  because there is no software Vulkan ICD installed
  (`/usr/share/vulkan/icd.d/` holds only `nvidia_icd.json`). Installing
  `vulkan-swrast` and pointing `VK_ICD_FILENAMES` at lavapipe would decouple
  them and has not been tried.
- **Alt+F4 at the desktop, which is what all of these actually were.** The
  nested compositors register their shortcuts with KGlobalAccel as *global*
  ones, and every instance shared the session bus at
  `/run/user/1000/bus` -- so all four sat alongside the desktop's own KWin on
  `Window Close=Alt+F4` (it is in `~/.config/kglobalshortcutsrc`). One
  keypress was delivered to every nested compositor as well, each closed the
  one client it had, and four KSP processes exited within ten seconds. The
  compositors survived because losing a client does not end them, which is
  exactly the "instances looked healthy from outside" signature blamed on the
  browser above.

  **The tell was written down twice before it was read: an orderly Unity
  `OnDestroy` cascade is a clean quit request, not a crash.** So is
  `Server 'Default Server' stopped` in `KSP.log`. Nothing was reaping these
  processes -- they were being politely asked to close, and they were saying
  yes. A reaper leaves a `SIGKILL` and a truncated log; this leaves a tidy
  shutdown, and the difference is the whole diagnosis.

  `kwinRun.sh` runs each compositor under `dbus-run-session` now, so its
  global shortcuts are global only to itself. **Check this first** when
  instances die together, before the suspend and the GPU: it is free to rule
  out (did anyone touch the keyboard?) and it was the cause of every
  occurrence recorded in this file.
- **Unexplained.** One occurrence with every box ticked negative and the KSP
  `NullReferenceException`s *following* the `OnDestroy` cascade -- which, per
  the entry above, means it was probably an Alt+F4 too.
- **Orphaned compositors, which wear the same costume and are mundane.** Every
  instance launched after one such restart died ~75 s after its own kRPC server
  came up, with no client ever connecting. `stop.sh` had killed
  `KSP_x64.exe`, gamescope and wineserver but **never `kwin_wayland
  --virtual`**, so each stop/start cycle orphaned one per instance, each still
  holding the `kspN-wl` socket the next needs. `stop.sh` kills them now and
  reports a compositor count — **check it is 0 before calling a restart
  clean.** The general lesson: a failure that resembles a known unexplained one
  is not thereby explained by it.

**`watchdog.sh` is the answer to all of them, because it does not ask which
one it is.** All four causes share one signature -- the process is gone, or
its kRPC port is closed -- and none of them announces itself to the harness:
a flight in the air simply stops producing telemetry and the loop sits on it
until its timeout, which for `quickglide.py` is an hour. So supervision is
kept separate from diagnosis.

```bash
./watchdog.sh start          # supervise every instance, restart what dies
./watchdog.sh status         # who is up, and the last few restarts
./watchdog.sh stop           # (also done by a bare ./stop.sh)
```

It polls "process alive *and* port listening", relaunches through `stop.sh N`
so the compositor corpse is cleared first, and writes the positive check's
verdict into `watchdog.log` at the moment it is still findable. A booting
instance is not a dead one, so nothing is judged until `WATCHDOG_BOOT_GRACE`
(480 s) after its launch. What it does **not** do is save the flight that was
in the air; it saves the farm, and the next batch.

Two bugs it found in its first minute, both worth keeping in mind before
writing anything else that stops an instance:

- **`stop.sh N` used to kill every instance's wineserver, not that one's.**
  The global sweep at the bottom matches `gamescope` and `wineserver` by
  name across the machine, so clearing one corpse took down the three
  healthy instances beside it -- which the watchdog then restarted, and so
  on. The sweep now runs only for a bare `stop.sh`.
- **`journalctl ... | grep -q .` is always true.** It prints
  `-- No entries --` on stdout when there are none, so the first restart
  this watchdog ever performed was logged as "a suspend" on a machine that
  had not suspended. The positive check needs `-q` and a grep for the thing
  itself.

In the flights this all surfaces as `FATAL BrokenPipeError` on every instance
at once. And in the readiness check, `ss -ltn | grep :$port` can go true before
the instance is usable; `grep -c "Server 'Default Server' started"
kspN/KSP.log` is the authoritative signal.

## What the instances are allowed to cost

Nothing here reads a pixel. The flight software works from kRPC telemetry and
`simulate_aerodynamic_force_at`, so rendering is pure overhead -- and it was
being paid at full price:

| | was | is | why |
|---|---|---|---|
| render size | 900x520 | **160x100** | 29x fewer pixels |
| `TEXTURE_QUALITY` | 0 (full) | **3 (eighth)** | VRAM, see below |

**The render size was not what `settings.cfg` said.** `mkbase.sh` set
`SCREEN_RESOLUTION_*` to 640x360 and `kwinRun.sh` passed
`-screen-width 900 -screen-height 520` on the command line, which wins -- so
the number in the config file had never been the one in use. Both are set
together now.

**It cannot go to 1x1, and that was measured rather than assumed.** Four
instances booted at 1x1, 32x32, 160x100 and 320x200: the first two reach
`Server 'Default Server' started` and then die *without* a shutdown cascade
-- a real crash, unlike the Alt+F4 signature above -- while 160x100 and
320x200 are fine. The floor is somewhere between 32x32 and 160x100 and there
was no reason to find it more precisely. KSP is frame-driven and everything
after the loading screen needs frames, so the target is the smallest size
that still renders, not the smallest that is readable. `RES=WxH ./kwinRun.sh
N` overrides it.

**VRAM is a real constraint and it used to be the binding one.** Four
instances at full-res textures measured 4673 + 4685 + 2879 + 2659 MiB --
14.9 GB of a 16.3 GB card, leaving ~700 MB for the whole desktop. That is not
a comfort problem: a card that full loses GL contexts, and a lost context
takes every GL client with it. `mkbase.sh` carried a comment asserting the
opposite ("not the constraint on a 16 GB card"); it was wrong, and the
measurement is why. `watchdog.sh` logs a line whenever the card passes
`WATCHDOG_VRAM_WARN_PCT` (88%) so the next shortage arrives as a number.

Texture resolution is a load-time mipmap choice. It touches no mesh, no
`DRAG_CUBE` and no mass -- which is the reason `ReStock` is kept at all -- so
it cannot move the quantity these instances exist to measure.

## What the strip and the concurrency cost

Three flights on a stripped instance at the then-committed config landed 74/66/
52 m from the pad against a reference group mean of 67 m, so the centre did not
move. The scatter looked better (52-74 against 10-131) but that is three
flights against four. Against a ~120 m reference spread this rules out a gross
change in the vehicle or the air, not a 20 m one.

Concurrency, measured properly at `AIM_BIAS_EAST_M=0` on `quicksave`, is not
visible, and the mechanism it was supposed to act through is identical:

| | flights | `BOOSTBACK -> COAST` | `held=` |
|---|---|---|---|
| solo | 286, 125 | 16712.48-16712.62 | 0.20-0.24 s |
| four instances | 214, 238, 267, 273, 466 | 16712.46-16712.54 | 0.20-0.22 s |

Compare parallel flights only against parallel flights as a precaution, but do
not attribute a difference to instance count without this table's equivalent.

**The trap that produced the wrong version of that is worth more than the
result.** `(defaults)` in a `config:` line means *the defaults of the day*, so
`config: (defaults)` and `config: AIM_BIAS_EAST_M=170.0` were the *same*
flight. Four solo flights logged that way landed 45, 148, 49, 155 m; read as
two configs of two they look like a clean 100 m bias effect and are in fact one
config scattering by 110 m. That then became a 220 m "concurrency effect" that
does not exist. **Read the numeric value, never the label**, and prefer sweeps
that set every field they depend on explicitly.

**Four flights into one `logs/` can be three files.** `logs/` is shared, and
`Logbook` used to scan for `highest + 1` and *then* open it, so two flights
starting together chose the same name and the second truncated the first —
leaving one file holding an interleaving of two flights, which still looks
exactly like a log. The claim is now `O_CREAT | O_EXCL`
(`test_concurrent_logbooks_never_share_a_file`). Be sceptical of any
parallel-flight log read before this.

### RAM: where an instance's 4 GB is (2026-10-02)

Measured on ksp4 in the flight scene with `qs_shuttle2` loaded (`/proc/PID/smaps`):
**4.07 GB anonymous** (Mono heap plus Unity's native allocations) and ~250 MB
file-backed, of which every DLL -- Unity's `Managed/` is 29 MB on disk -- is a
few tens of MB.  Deleting DLLs is not a lever.

- **Part textures deleted** from every clone (`*.dds`, non-plugin `*.png`, the
  KSPCommunityFixes texture cache; `mkclone.sh` now does it): GameData 2.4 ->
  1.1 GB on disk, boots to its port in 40 s, flies the bench as before (LOG4712),
  same eight ReStock exceptions as an unstripped clone -- but RSS is ~unchanged
  (4.0 GB): at `TEXTURE_QUALITY 3` the textures were never the weight.
- **`-nographics`** (`KSP_EXTRA=-nographics ./start.sh N`, `kwinRun.sh` passes
  `KSP_EXTRA` through): **hangs** -- the process sits at 76 MB for 5+ min and
  never writes KSP.log.  Dead end.
- What is left is the 489 compiled parts (`partstrip.py`, above: 4.26 -> 1.97
  GB but wedged on a cross-mod texture reference).

Note `kwinRun.sh`'s exec line: the comment between its continuation lines ends
the command, so instances launch with `-force-d3d11` alone and the
`-screen-*`/`-popupwindow` arguments after it are never passed.  Left as is --
every measurement on the farm was taken that way.
