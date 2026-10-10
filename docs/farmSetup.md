# Setting up the measurement farm

The farm is a set of headless copies of Kerbal Space Program, each with its
own kRPC port, that the harnesses fly unattended and in parallel. **The game
copies are not in this repository and never will be**: KSP is commercial
software, so you build them from your own install. This page covers going
from a clean checkout to six instances. To find out why the farm is built this
way, read [testInstances.md](testInstances.md),
[`testInstances/README.md`](../testInstances/README.md) and
[`testInstances/keepNotes.md`](../testInstances/keepNotes.md).

You don't need the farm for the offline tests (`python3 -m unittest`) or for
flying a single autopilot against the game you already play
(`./run.sh --pilot spaceplane`). You need it for the farm harnesses
(`quickglide.py`, `savefly.sh`, rotations) and for probing new `kspSim`
models.

## What you need

**A legal KSP install.** The farm was built from KSP **1.12.3 (build
03190)**, the Windows build (`KSP_x64.exe`), run under Proton. By default
`mkbase.sh` reads it from `~/Kerbal Space Program`; set `KSP_SRC=/path` to
point somewhere else. The install is only ever read.

**The mods in it.** These are the mods the reference saves were flown with.
The craft depend on them for mass, drag and control-surface behaviour, so
leaving one out gives you a different vehicle (see the trap below). Get them
from CKAN or from each mod's own release page:

| mod | version | why |
|---|---|---|
| kRPC | **0.6.0** | the server. `requirements.txt` pins the client to match |
| ModuleManager | 4.2.3 | |
| ReStock | 1.5.1 | rewrites mesh, drag cube and mass on most stock parts. **Not cosmetic** |
| AtmosphereAutopilot | 1.6.1 | replaces the control-surface PartModule |
| PersistentThrust (+ Navigator) | 1.7.5 | replaces the engine PartModule |
| FullAutoStrut | | adds struts to the vessel |
| AxisGroupController | | |
| B9PartSwitch | 2.21.0 | |
| Kopernicus, ModularFlightIntegrator | 1.12.1.247, 1.2.10 | |
| KSPCommunityFixes, Harmony, KSPBurst, BurstPQS, Shabby, ClickThroughBlocker, ToolbarControl | 1.41.1, 2.2.1, 1.7.4, 0.1.24, 0.4.2.0, 2.1.10, 0.1.9 | dependencies |

You can have any other mods installed. `mkbase.sh` leaves out every directory
listed in [`testInstances/strip.txt`](../testInstances/strip.txt) (visual,
audio and UI mods). A mod that is in neither list gets copied, and if it
touches the vessel it changes the measurement.

**Tools.** The farm was developed on Linux (KDE/Wayland, 16 cores, 30 GB RAM
plus zram). You need:

- [`umu-run`](https://github.com/Open-Wine-Components/umu-launcher) and a
  GE-Proton in `~/.steam/root/compatibilitytools.d/` (the newest one is used)
- `kwin_wayland`, which gives each instance its own invisible
  `--virtual` compositor. `gamescope` is only needed for `spawn.sh`/`run-ksp.sh`
- `mcs` (Mono) to build the two farm plugins, `tar`, `ss`, `systemd-inhibit`
- Python 3 with a venv. `./run.sh` creates `.venv` on first use. PyPy is
  optional but about 7x faster for the test suite:
  `kspSim/tools/pypysetup.sh`

**Disk and memory.** `base/` is about 7.5 GB. Each clone is a hardlink farm of
about 10 MB plus its own 1.3 GB wine prefix. Each running instance uses about
4 GB of RAM. Six instances is the tested maximum; a seventh saturated the CPU
at boot.

**A wine prefix to copy (optional).** `mkclone.sh` copies
`~/.local/share/ksp-prefix` (or `KSP_PREFIX=`) when it exists. Otherwise umu
builds a fresh prefix on first launch, which is slower.

## Build it

`./setup.sh` at the root does all of this: it asks for your install's path
and the instance count, checks the tools below, then runs the same steps.
By hand:

```bash
cd testInstances
./mkbase.sh                 # base/ from your install: stripped, kRPC autostart,
                            # ports, and the two plugins built with mcs
for n in 0 1 2 3 4 5; do ./mkclone.sh $n; done
./syncSaves.sh push         # copy saves/ into every instance
./syncSaves.sh check        # ... and confirm it
```

`mkbase.sh` installs two small plugins, both from source in this repository:

- `autoloadSrc/` loads a quicksave from the main menu. kRPC deliberately does
  not listen while the game is at the menu, so an unattended instance would
  otherwise never open its port.
- `timescaleSrc/` sets the physics time scale from a file that the
  harness writes (`common/timescale.py`, `tools/timescale.py`).

The `.dll` files are not tracked: `mkbase.sh` builds all three (the two
above and `collisionSpySrc/`) with `mcs` (mono) against your own
`base/KSP_x64_Data/Managed/`, so install mono first.

## Run it

```bash
./nosleep.sh start          # a suspend mid-batch reads as a crash
./start.sh                  # instances 0-5; returns when every port is up
./watchdog.sh start         # optional: restart any instance that dies
...                         # fly: see spaceplane/CLAUDE.md, boosterland/CLAUDE.md
./stop.sh
./nosleep.sh stop
```

Instance N listens on rpc `50100 + 2N` and stream `50101 + 2N`, which leaves
the normal install's 50000/50001 free to play on. The first boot of a fresh
clone is slow: it compiles its shader caches, and the log goes quiet for a
minute or two. That is not a hang (`keepNotes.md` has the one that is).

## Check it before you trust it

- `diff -r testInstances/ksp0/GameData testInstances/base/GameData` should
  show nothing but per-instance config. Instances that drifted apart in mod
  set once cost a whole session.
- `testInstances/actuators.py` on a craft before any other work with it.
- `swapon --show` should be near zero before and after a batch. Restart the
  farm between sessions: hours of flying bias the results.
- `pgrep -af spaceplane.autopilot` should show one process per busy instance
  and none once the batch is over.
