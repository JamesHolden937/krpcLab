# boosterland

Flies a KSP booster back to the KSC launchpad after stage separation,
Superheavy-style: boostback burn, ballistic coast, suicide burn, touchdown.
Target pad lat `-0.097162`, lon `-74.557679` (`Config.PAD_LAT/PAD_LON`). It
lands within a few metres of the pad and is largely settled work. The root
`CLAUDE.md` (conventions, farm, cross-cutting rules) applies here in full.

```
STANDBY -> SEPARATION -> BOOSTBACK -> COAST -> LANDING_BURN -> TOUCHDOWN
                                       ^  |
                                       +--+ CORRECTION
```

`autoland.py` is the phase machine and the only kRPC-aware control code;
`trajectory.py` propagates, `guidance.py` holds the laws as pure functions.
`docs/boosterland/plan.txt` is the original specification and still describes
the intended flight profile.

| read | when |
|---|---|
| [docs/boosterland/design.md](../docs/boosterland/design.md) | changing guidance, the propagator, a phase, or the drag/lift estimators |
| [docs/boosterland/failures.md](../docs/boosterland/failures.md) | **before undoing anything that looks redundant** -- 17 numbered rules, each paid for by a lost or broken flight |
| [docs/boosterland/tuning.md](../docs/boosterland/tuning.md) | measuring a change: `replay.py`, aim bias, sim-vs-game, the accuracy history, a different vehicle or body |
| [docs/boosterland/testing.md](../docs/boosterland/testing.md) | writing or reading `tests/fakeksp` regressions |

Saves: `quicksave`, `qs_hot`, `qs_cold`, `qs_steep`, `qs_north` (18 parts,
flying at 21 km after separation; `tools/savegen.py` makes more).

**The booster has no time-scale governor** -- only the spaceplane does
(`common/pacing.ScaleGovernor`). On the farm, `--timescale` is a fixed
multiplier for it, and what the loop achieved is not logged.

```bash
./run.sh --pilot booster                       # fly it in your game, waiting for START
./boosterland/tools/quickfly.py -n 3           # fly the quicksave repeatedly (live game)
./testInstances/fly.sh 0 -n 3 --timescale 4    # ... on farm instance 0
./boosterland/tools/quickfly.py -n 1 --set DIAG_STATE=True   # log enough to replay it
./boosterland/tools/replay.py logs/LOG53 --curve-from -1     # re-propagate a real flight offline
./boosterland/tools/sweep.py -n 2 --saves quicksave,qs_hot --compare CORRECTION_ENTER_M=300,1200
./boosterland/tools/sweep.py -n 2 --saves quicksave --configs "a:X=1;b:X=1,Y=2" --out raw.json
./boosterland/tools/logsum.py --trace logs/LOG25   # one line per flight, plus the miss trace
./boosterland/tools/padfix.py                  # measure the pad's coordinates from the game

python3 -m unittest boosterland.tests.testFlightSim   # closed-loop flight only
python3 -m unittest boosterland.tests.testOffline.TestGuidance.test_boostback_burns_against_the_miss
```
