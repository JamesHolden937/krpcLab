# Contributing to krpcLab

Thanks for wanting to help. Bug reports, measurements, new autopilots, farm
fixes and documentation are all welcome. Everyone taking part agrees to the
[Code of Conduct](CODE_OF_CONDUCT.md).

## The short version

1. Fork, branch, change, open a pull request against `main`.
2. Run the offline tests (`python3 -m unittest`, or `.venv-pypy/bin/python
   -m unittest`); they need no KSP.
3. If the change affects how a vehicle flies, include the flights that
   measured it (see "Changes to an autopilot" below).
4. Sign off your commits (`git commit -s`, explained below).

The maintainer reviews every pull request and has the final say on what is
merged. Nobody pushes to `main` directly. That is what keeps the project
coherent while it stays open to everyone.

## Licence of contributions

krpcLab is licensed under the **GNU General Public License, version 3 or
(at your option) any later version** ([LICENSE](LICENSE)). By contributing
you agree that your contribution is licensed under the same terms
(inbound = outbound). You keep the copyright to what you wrote; there is
no copyright assignment and no CLA.

**Sign-off (Developer Certificate of Origin).** Add a `Signed-off-by:`
line to each commit (`git commit -s`). It certifies that you wrote the
change or otherwise have the right to submit it under this licence, as set
out at <https://developercertificate.org/>. Use your real name or your
usual handle.

**Never commit game files.** KSP is commercial software. Nothing from a KSP
install (GameData, `KSP_x64_Data`, mods' binaries) may enter this
repository. Craft files and quicksaves that you made yourself are fine.

## Forks and redistribution

You may fork, modify and redistribute krpcLab under the GPL. In return the
GPL asks that you:

- keep the licence and the copyright notices;
- mark a modified version as modified (GPLv3 section 5a). Please also give
  a public fork that is not tracking this project its own name, so that
  users can tell the two apart;
- give the people you distribute it to the source, under the same licence.

## How to set up

`./setup.sh` asks for the path of your KSP install, checks the mods, builds
the Python environment and, if you want it, the measurement farm. See
[README.md](README.md) and [docs/farmSetup.md](docs/farmSetup.md).

## Changes to an autopilot

The project's conventions are in [CLAUDE.md](CLAUDE.md) ("Hard conventions"
and "Cross-cutting rules"), and each autopilot's own `CLAUDE.md` gives its
current state. The ones a pull request is checked against:

- **Nothing prints.** Output goes to the `logs/LOG<n>` logbook and the
  in-game panel.
- **Every tunable lives in `<pilot>/config.py`**, with a comment that says
  what it is a quantity *of*, and what measured it.
- **A behaviour change goes in behind a flag, is measured, and then becomes
  the default or is deleted.** Do not leave a flag parked off.
- **Measure in the game, with enough flights.** One flight per
  configuration is not a measurement. Interleave the arms over the farm
  (`spaceplane/tools/multirot.sh`) and quote counts with their n, for
  example "15/18 on the runway against 6/18". Name the saves you flew,
  more than one orbit where there is one.
- **Read the signed miss** (`along=` / `across=`), not the distance.
- Don't track anything the repository plus a KSP install can regenerate
  (built DLLs, `savegen.py` outputs: add those to `saves/derived.txt`).

Small fixes (typos, tooling, documentation, a crash) need none of the
measurement. Just say what you checked.

## New autopilots

See "Adding an autopilot" in [CLAUDE.md](CLAUDE.md): a package with
`--address`/`--rpc-port`/`--set`/`--autostart`, its own `config.py` and
logbook, one line in `common/launcher.py`, offline tests and a farm harness.

## Reporting bugs

Open an issue with the `logs/LOG<n>` file of the flight (it records the
configuration and the defaults fingerprint), the save it was flown from,
and your KSP and kRPC versions.
