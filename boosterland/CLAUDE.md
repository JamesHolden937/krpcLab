# boosterland

Flies a KSP booster back to the KSC launchpad after stage separation,
Superheavy-style: boostback burn, ballistic coast, suicide burn, touchdown.
Target pad lat `-0.097162`, lon `-74.557679` (`Config.PAD_LAT/PAD_LON`). It
lands a median ~12-30 m (qs_hot) to ~13-20 m (qs_heavy, at first contact)
from the pad. Boostback holds its heading through the taper; the coast's
flip runs on the RCS error relay; once settled, the coast pushes sideways on
the RCS to null the cross-track (`COAST_RCS_TRIM`); what is left is
along-track, ~+28 m long at the burn (HANDOFF). The legged heavy stands
~10 times in 12, but falls about half the time it touches 10-34 m out: the
KSC pad model's own colliders and the launch clamps are there. The 18-part
booster has no legs and falls over on its engine bell. The legs are timed
once per part type and dropped by time-to-go (in practice ~13.5 s early);
the airbrakes fly closed and long, then open fully at a solved height --
see the root HANDOFF.md. The root
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
| [docs/boosterland/failures.md](../docs/boosterland/failures.md) | **before undoing anything that looks redundant** -- 19 numbered rules, each paid for by a lost or broken flight |
| [docs/boosterland/tuning.md](../docs/boosterland/tuning.md) | measuring a change: `replay.py`, aim bias, sim-vs-game, the accuracy history, a different vehicle or body |
| [docs/boosterland/testing.md](../docs/boosterland/testing.md) | writing or reading `tests/fakeksp` regressions |

Saves: `quicksave`, `qs_hot`, `qs_cold`, `qs_steep`, `qs_north` (18 parts,
no legs, flying at 21 km after separation; `tools/savegen.py` makes more);
`qs_heavy` (99 t, five engines, six LT-2 legs).

`boosterland/data/gearTimes.json` is the leg deploy-time cache (gitignored,
written in flight; delete it and the next flight re-times its legs).

**The booster has no time-scale governor** -- only the spaceplane does
(`common/pacing.ScaleGovernor`). On the farm, `--timescale` is a fixed
multiplier for it, and what the loop achieved is not logged.

```bash
./run.sh --pilot booster                       # fly it in your game, waiting for START
./run.sh --pilot fmrs                          # FMRS separation save, booster made active in the file, flown; then JUMP BACK TO MAIN MISSION (fmrs.py)
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
