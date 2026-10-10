# krpcLab

[kRPC](https://krpc.github.io/krpc/) autopilots for Kerbal Space Program, and
the infrastructure to measure them: a farm of headless game instances, a
headless simulator that the same autopilots fly unmodified, reference saves,
and log readers.

| directory | what |
|---|---|
| `boosterland/` | flies a booster back to the KSC pad after stage separation |
| `spaceplane/` | deorbits a winged vehicle, flies the entry and lands it on the KSC runway |
| `kspSim/` | a headless KSP model behind a real kRPC server, for fast screening |
| `common/`, `tools/` | shared by every autopilot |
| `saves/` | the reference quicksaves and craft files |
| `testInstances/` | the measurement farm's scripts (the game copies are **not** included) |
| `docs/` | design notes, failure histories, session journals |

## Setup

```bash
git clone https://github.com/JamesHolden937/krpcLab.git && cd krpcLab
./setup.sh
```

`setup.sh` asks for the path of your KSP install (the folder holding
`GameData`; it is only ever read), checks that kRPC 0.6.0 and the mods the
reference saves were flown with are there, builds the Python environment,
regenerates the derived saves, and offers to build the measurement farm.
Unattended: `./setup.sh --ksp "/path/to/Kerbal Space Program" --no-farm -y`
(or `--farm 6`). You need KSP 1.12 with [kRPC
0.6.0](https://github.com/krpc/krpc/releases) and Python 3; the farm's extra
requirements are in [docs/farmSetup.md](docs/farmSetup.md).

## Quick start

```bash
./run.sh                          # in-game launcher, against a KSP running kRPC 0.6.0
./run.sh --pilot spaceplane       # one autopilot; its in-game panel waits for START
python3 -m unittest               # offline tests, no KSP needed
```

To set up the parallel measurement farm, see
[docs/farmSetup.md](docs/farmSetup.md). It is built from your own copy of
KSP; no game files are distributed here.

[CLAUDE.md](CLAUDE.md) and each autopilot's own `CLAUDE.md` describe the
project's conventions and current state in detail.

## Contributing and licence

Contributions are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) and the
[Code of Conduct](CODE_OF_CONDUCT.md). Pull requests are reviewed and merged
by the maintainer.

krpcLab is free software: you can redistribute it and/or modify it under the
terms of the [GNU General Public License](LICENSE) as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version. It is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. Copyright (C) 2026 JamesHolden937 and the
krpcLab contributors.

Kerbal Space Program is a trademark of its owners; this project is not
affiliated with them and distributes no game files.
