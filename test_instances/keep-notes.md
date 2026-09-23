# What was removed, what was kept, and why

The measurement instances exist to fly `quickfly.py` unattended, in parallel,
without the GPU cost of the visual mods.  The constraint is that **the flight
must be physically identical to the one the calibration was measured on** --
`AIM_BIAS_EAST_M = 628` is a number measured on this vehicle in this air, and
anything that changes mass, drag or terrain height invalidates it.

So every removal candidate was checked against the 20 part names that appear
in `Untitled Space Craft.craft`, for patches touching
`mesh` / `MODEL` / `DRAG_CUBE` / `dragCube` / `mass` / lifting-surface keys.

## Removed (31 directories, 5.9 GB)

See `strip.txt`.  All of them are rendering, audio, skybox, IVA-prop or UI
mods.  None patches any part this booster uses.

## Removed, second round (1.2 GB, 2026-09-13)

A second pass over `base/GameData`, measured and re-flown afterwards.  93% of
the 1.2 GB is **texture data inside mods that stay**, which is where the win
actually was:

| what | size | why it is inert |
|---|---|---|
| `ReStock` `.dds` + `FX/*.mu` | 600 MB | the part `.mu` **meshes stay**; only the surface textures and the engine-plume models go.  KSP derives the drag cube from the mesh, so mass and drag area are untouched |
| `Squad` agency logos, IVA `Spaces/`, `FX/*.mu`, tutorials | 308 MB | no part model or part `.cfg` was touched; IVAs are never entered and the craft flies uncrewed |
| `SquadExpansion` banners, missions, stock craft | 203 MB | content, not parts |
| `KSPCommunityFixes/PluginData/TextureCache` | 74 MB | a regenerated cache.  `PNGTextureCache.cfg` and the other four answered prompts **stayed** -- deleting *those* is the silent hang below |
| `ModuleManager.Config{Cache,SHA}`, `.Physics`, `.TechTree` | 4 MB | MM regenerates them; deleting one only costs a slow first load |
| MechJeb2, WindTunnel, FMRS, BetterTimeWarp, ksp-advanced-flybywire, KSPBurnPlanner, CommNetOptimization, FlightRecorder, HUDReplacer, RecoveryController, StationKeeping | 20 MB | GUI and planning tools.  None flies the vehicle; `quickfly.py` drives it over kRPC |

**Three were put back**, and they are the ones worth knowing about:

- **AtmosphereAutopilot** (348 KB) and **PersistentThrust** /
  **PersistentThrustNavigator** (172 KB).  They look like GUI mods and are
  not: the quicksave carries `SyncModuleControlSurface` on six control
  surfaces and `PersistentEngine` on both engines, i.e. they *replace* the
  stock control-surface and engine modules on the flying vessel.  Removing
  them reverts six surfaces to stock `ModuleControlSurface` with the part
  config's authority instead of the save's -- an unmeasured change to the
  spaceplane's pitch and roll authority, on the project whose open problem is
  glide repeatability.  Half a megabyte is not worth that variable.
- **Shabby** (32 KB).  Already on the keep list above; ReStock and
  B9PartSwitch load shaders through it.

**What the log says when a PartModule's mod is gone**, and why it is survivable
here:

    [WRN] [KSPCF:ModuleIndexingMismatch] Persisted module
          "SyncModuleControlSurface" at index [0] has been removed,
          no matching module in the part config

That is KSPCommunityFixes re-matching persisted modules **by name** instead of
by index.  Without it the save's module list would shift by one and every
module after the missing one would be loaded with the wrong state -- silently.
KSPCF is on the keep list for other reasons; this is another.

**Verified, not assumed** (clone `ksp4`, built from the stripped `base/`):
loaded to kRPC in 39 s, autoload reached `SPACECENTER`, 9 exceptions and 1426
errors -- *all* of them `Texture '...' not found` or
`ParticleModelFX: Cannot find model` from the deleted `.dds`/FX.  Two booster
flights landed 3 m and 5 m from the pad, at **27.25 t separation mass and
`cda` within 1% of LOG676/677 on an unstripped instance from the same
quicksave**.  Identical mass and identical drag area is the check that
matters: it says the stripped instance is flying the same vehicle through the
same air.  A spaceplane entry from `qs_plane` flew normally too.

## Kept, against the obvious instinct

**ReStock** -- the one that looks like a texture mod and is not.  Its patches
carry `!mesh = DELETE`, `!MODEL {}`, a replacement `MODEL`, `DRAG_CUBE` /
`!DRAG_CUBE` and `@mass`, on **15 of the 20 parts**, including all three
Rockomax tanks, the SSME, the engine plate, the service bay and the vernier
RCS.  KSP generates a part's drag cube from its mesh, so removing ReStock
changes both the mass and the drag area of the booster.  On this project that
is not a cosmetic difference -- it is the quantity the whole estimator is
measuring.  690 MB is worth paying to keep the vehicle the same vehicle.

**Shabby** -- 32 KB shader shim.  Nothing to gain by removing it, and ReStock
and B9PartSwitch both load shaders through the same path.

**Parallax** -- removed, and the user's instinct was right.  Checked: for
Kerbin it adds only

    Parallax { subdivisionLevel = 7  subdivisionRadius = 700 }

to the PQS mod list.  That subdivides the terrain *mesh* within 700 m of the
camera; it does not alter the height field.  The `VertexHeightMap` override in
`Parallax_StockPlanetTextures` is scoped to **Eeloo**, and the `deformity`
values in `IncreaseSubdivisionCount.cfg` belong to Laythe -- Kerbin's are
commented out.  Terrain height at the pad is unchanged.

**Everything that attaches a PartModule to this vessel** -- FullAutoStrut
(struts), PersistentThrust (`PersistentEngine`, on both engines),
AtmosphereAutopilot (`SyncModuleControlSurface`, on all six control
surfaces), plus KSPCommunityFixes, Kopernicus, ModularFlightIntegrator,
B9PartSwitch, BurstPQS, KSPBurst.  Any of these could plausibly move the
flight, so they stay.  One variable at a time.  The ones that only add a
*window* went in the second round below.

## Does removing a mod break the .craft file?

No, for this craft, and the reason is worth knowing because it is not general.

A craft file breaks when it names a **part** the game can no longer find --
KSP cannot instantiate the vessel and refuses the craft.  All 20 parts in
`Untitled Space Craft.craft` resolve to `Squad/Parts`, `SquadExpansion/
MakingHistory` or `SquadExpansion/Serenity`.  Not one modded part.  So nothing
in the strip list can orphan a part.

What *is* orphaned is **PartModule** state.  The craft carries
`ModuleReforged` on all 41 parts, plus `ModuleWaterfallFX`,
`ModuleRestockDepthMask`, `ModuleRestockRCSGlow`, `RSE_RCS`, `RSE_Engines`,
`ShipEffectsCollisions` and so on.  With the mod gone KSP logs that it cannot
find the module and skips the node; the craft still loads, with stock plumes,
stock sounds and no recolouring.  Geometry, mass, staging and action groups are
untouched.

The one real loss is **write-back**.  If a stripped instance re-saves the
craft, the `ModuleReforged` colour data is gone from the file for good, and
reinstalling ReforgedRedux will not bring it back.  That is why each clone gets
its own private `saves/` and `Ships/` (see `mkclone.sh`) and why `base/` is
never run: nothing here can write to the live install's craft files.

If you later add a mod that ships **parts** and build them into the booster,
this analysis has to be redone -- rerun the part-name check before stripping.

## Flying faster than real time

A booster flight is ~175 s of wall clock and a spaceplane entry is ~12 min, and
the rules in CLAUDE.md require several flights per configuration before a
difference is believable.  That arithmetic is the real bottleneck on this
project, so it is worth knowing exactly what can and cannot be bought.

**KSP's own physics warp is not the answer.**  It multiplies
`Time.fixedDeltaTime` (0.02 -> 0.08 at 4x) so the CPU cost per real second
stays flat.  That is a straight integration-fidelity loss, and it is worst
precisely where this vehicle lives: dense air, under thrust, with a controller
closing on a target.

`timescale-src/` does the opposite trade.  `Time.fixedDeltaTime` is left alone,
so every physics step is the step the vehicle flies at 1x; `Time.timeScale`
goes up, which makes Unity run *more* of those steps per real second.  Same
trajectory, more of it per wall-clock second, linear CPU cost.  Driven by
`<instance>/timescale.txt`, off by default, and `../timescale.py` is the other
end:

    ./timescale.py 5 2.0     # fixed 2x
    ./timescale.py 5 max     # as fast as the main thread will sustain
    ./timescale.py 5         # what it actually ACHIEVED, not what it was told

### The ceiling is control quantization, not the CPU

Unity applies control input once a frame, so the autopilot's commands land on a
grid of `timeScale / fps` game-seconds.  Push the time scale up without pushing
the frame rate up and you get faithful physics flown by a degraded controller
-- the same failure as running the game at 10 fps, arriving from the other
direction.  `quant_s` (default 0.05 s, the booster's own command interval) is
the ceiling on that grid, and the achievable speed follows as `quant_s * fps`.

Two things fall out of that, and both were measured rather than reasoned:

- **`SYNC_VBL` was the real cap.**  Vsync pinned every instance to the virtual
  output's 60 Hz, which pinned the top speed to 3.0x.  With vsync off the same
  flight reached 4.2x.
- **More frames is not better.**  196 fps reached 3.38x; 89 fps reached 4.22x
  in the same wall clock.  Rendering and physics share the one main thread and
  KSP's physics is single-threaded, so frames above what the quantum needs are
  taken straight out of the speedup.  The optimum is the *lowest* frame rate
  that still meets the quantum: `fps ~= target_speed / quant_s`.  Hence
  `FRAMERATE_LIMIT = 90` and `SYNC_VBL = 0` in `mkbase.sh`.

### The half of it that is not in the plugin

Both autopilots paced their loops with `time.sleep(LOOP_SLEEP_S)` -- wall-clock
seconds.  At 4x that is a 5 Hz loop *in the air* for a 20 Hz command interval,
and the flight it produces is not reproducible at 1x.  `boosterland/pacing.py`
paces on `space_center.ut` instead, behind `LOOP_PACING_GAME_TIME`, which
`quickfly.py` turns on automatically whenever `--timescale` is not off.

**`quickfly.py` drives `Autoland.tick()` itself rather than calling
`Autoland.run()`.**  A pacer added only to `run()` would not be in the path any
measurement actually flies.  Both were changed; if a third harness appears,
it needs the same treatment.

### What it bought, and what it cost

| config | wall/flight | achieved | fps | quantum | miss |
|---|---|---|---|---|---|
| 1x | 175 s | 1.00x | 60 | 0.017 s | 6 m |
| 2x fixed | 87 s | 1.99x | 60 | 0.033 s | 7 m |
| max, vsync on | 62 s | 3.03x | 60 | 0.050 s | 4 m |
| max, 240 fps cap | 47 s | 3.38x | 196 | 0.017 s | 6 m |
| max, 90 fps cap | 47 s | 4.22x | 89 | 0.047 s | 2 m |

`fixed_dt` stayed at 0.0200 throughout, which is the property the whole design
exists to preserve.  **Commanded is not achieved**: asking for 8x on a machine
that sustains 4.2x gets 4.2x, and from inside the game the two are
indistinguishable, which is why the plugin measures game seconds against real
seconds and writes the answer back for the harness to report per flight.

## The first-run dialog that looks exactly like a hang

Building the instances, every clone stalled at the same point in the loading
screen: the log stopped, CPU dropped to ~0.2 of a core, nothing was written to
disk, and the process sat there indefinitely.  It reproduced with gamescope's
headless backend, with the sdl backend, without gamescope at all, at 640x360
and at 2560x1440, with a copied wine prefix and with the original one, and
with the full unstripped GameData symlinked back in.  It was none of those.

It was **KSPCommunityFixes' FastLoader opt-in dialog** -- "KSPCommunityFixes
can cache converted PNG textures on disk to speed up loading time ... Do you
want to enable this optimization?" -- a modal popup raised *during* the
loading screen.  An unattended instance rendering into a 640x360 off-screen
surface has nobody to click it, so the loader waits forever.

The answer is stored in `GameData/KSPCommunityFixes/PluginData/
PNGTextureCache.cfg` as `userOptInChoiceDone = True`, alongside four other
answered prompts (`OptionalMakingHistoryDLCFeatures`, `NoIVA`,
`DisableManeuverTool`, `AltimeterHorizontalPosition`).  **`mkbase.sh` used to
wipe every `GameData/*/PluginData` directory**, which deleted all five, so the
clone asked again where the live install never would.  It no longer wipes
them; it copies them, and that is load-bearing rather than incidental.

Two consequences worth keeping:

- **The 370 MB `KSPCommunityFixes/PluginData/TextureCache` is copied too**, and
  hardlinked, so it costs nothing.  A *partial* cache is worse than none: the
  first debugging runs were killed mid-write and left truncated entries, and
  FastLoader blocked on those the same silent way.  If a clone ever stalls
  there again, delete its `TextureCache` and re-link the live one rather than
  letting it rebuild.
- **Any mod that asks something on first run will do this.** The symptom is
  always the same and always misleading: no error, no CPU, no log line. Before
  bisecting the mod list, check `GameData/*/PluginData` against the live
  install for a settings file the clone is missing.

## The stall was one missing line, and every environmental theory was wrong

An unattended instance loaded to the main menu and stopped there.  Three
explanations were advanced and all three were wrong:

1. **The session lock.**  `loginctl` did report `LockedHint=yes`, and on KWin
   Wayland a locked session does stop frame callbacks.  Unlocking changed
   nothing.
2. **Window focus.**  Unity does stop `Update()` on an unfocused window, and
   `Awake()`/`Start()` running while `Update()` never did fitted perfectly.
   Wrong: the heartbeat that "proved" it sat *after* an `if (done) return;`
   guard, so it could not log once the save had loaded.  Moving it above the
   guard showed frames running at a steady 60 a second the whole time.
3. **`-popupwindow`** being unfocusable.  Removing it was strictly worse -- the
   load then stalled at 1208 lines instead of reaching the menu.

The actual cause was one missing assignment in this plugin:

    HighLogic.CurrentGame = game;      // before game.Start()

`Game.Start()` reads `HighLogic.CurrentGame` internally.  Without it, it throws
a `NullReferenceException` *inside* `Game.Start()`; Unity catches exceptions per
callback, so `Update()` carried on at 60 fps and the menu simply stayed up
forever.  The log had said so all along:

    [EXC] NullReferenceException
            Game.Start ()
            BoosterlandAutoLoad.AutoLoadSave.Load ()

**The lesson is about instrumentation, not about KSP.**  A heartbeat placed
where it cannot fire is worse than none, because its silence got read as
evidence and sent the search into the compositor, the session lock and three
gamescope backends.  Check that a diagnostic can actually produce output in the
failing case before trusting its absence -- and grep the log for `[EXC]` before
theorising about the environment.

Two things did come out of the detour and are worth keeping:

- `kwin-run.sh` runs each instance inside its own `kwin_wayland --virtual`
  (already installed, no sudo, no visible window).  Not needed to fix the
  stall, but a clean way to run several instances with no display contention.
- Target `SPACECENTER`, not `FLIGHT`.  `Game.Start()` into `FLIGHT` needs
  `FlightDriver` primed with a valid active vessel.  The space centre is a real
  scene as far as kRPC is concerned -- `KRPC.Addon` only calls `Core.StopAll()`
  for `GameScene.None`, which is the main menu -- so the server starts there and
  `quickfly.py` loads the quicksave through its normal path.

## Choosing a backend

`kwin-run.sh <n>` is the launcher for unattended runs: it puts each instance in
its own `kwin_wayland --virtual` compositor, which has no visible output and no
contention with anything else on the desktop, so several can run at once.
kwin_wayland ships with the desktop session here -- nothing to install.

`run-ksp.sh <n>` (gamescope) is for watching a single instance interactively:

    BACKEND=sdl  ./ksp0/run-ksp.sh      # a real window you can see
    BACKEND=none ./ksp0/run-ksp.sh      # no gamescope, uses your DISPLAY

An earlier version of this file claimed the backend choice was load-bearing --
that only a focused window would run, and that `headless` was mandatory. That
was wrong; see the section above.  The stall it was trying to explain was a
missing `HighLogic.CurrentGame` assignment, and it reproduced identically under
every backend because it had nothing to do with the display.
