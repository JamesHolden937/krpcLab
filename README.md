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
