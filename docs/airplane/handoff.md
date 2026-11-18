# airplane HANDOFF -- read with ../../HANDOFF.md, rewrite in place

This pilot's sections (most recent first). Replace a section rather than append; history goes in journal.md.

## 2026-11-08: airplane -- crash guard deleted; phase margin; test jets; pitch stiffness (fingerprint after `a8e0e27`)

`airplane/` (read `airplane/CLAUDE.md`, "Where it stands" / "Next";
journal 2026-11-08). The user: delete the crash guard from production;
reduce oscillation; work for the vast majority of aircraft; build jets
and screenshot them before flying. All committed; **not pushed;
howItWorks.pdf not redone (crash guard still in it). Farm stopped.**

**Done:** crash guard deleted (`GUARD*`, `pull_up_loss`). Rate-loop tau
from a phase margin (`RATE_PHASE_MARGIN_DEG`, default 32.12 = the old 4
ticks; 47 is the candidate). `control.Stiffness` (pitch input per rad of
alpha, fitted in flight) and `ATTITUDE_TRIM` (off): the trim integrates the
attitude error at g p. `RATE_SETPOINT_WEIGHT` (1 = off). Three stock jets
(`airplane/tools/craftgen.py`, screenshots `logs/shots/craftgen/`),
Seahawk and Suborbital hopper craft in `saves/craft/SPH`.
`airfly --rotate`, runway launch from a loaded flight.

**Unmeasured:** every flag above -- one flight per cell so far. The
trainer's sag after rotation (both margins) is the model's missing
stability spring, which `ATTITUDE_TRIM` targets.

**Next:** re-fly the killed screening batch on **4 instances** (6 put
21-28 GB in swap within two rounds and the reaper killed the batch), then
3+ flights a cell, promote/delete `RATE_PHASE_MARGIN_DEG` 47,
`ATTITUDE_TRIM`, `RATE_SETPOINT_WEIGHT`; PDF; push.

**Traps:** `git commit -a` swept the user's four uncommitted doc edits
(CLAUDE.md, docs/farmSetup.md, docs/testInstances.md,
testInstances/README.md) into a commit once -- undone; they are still
uncommitted, not ours. Commits take ~2 min (post-commit hook). A fresh
instance sits in the space centre and `launch_vessel` hangs there.
