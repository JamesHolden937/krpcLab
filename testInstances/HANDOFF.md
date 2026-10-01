# testInstances -- state at handoff

> The farm's build history. **The current session handoff is the root
> [HANDOFF.md](../HANDOFF.md)**, read at the start of every session.

Read `README.md` for how to use it and `keepNotes.md` for why it is built the
way it is.  This file is the short version plus what is and is not verified.

## Works, verified in game

- `mkbase.sh` builds `base/` from the live install without the 31 mods in
  `strip.txt`: GameData 9.4 GB -> 3.1 GB, main menu in 43 s against 61 s.
- `mkclone.sh N` makes an instance for ~10 MB (hardlinked to `base/`, links
  broken on everything KSP rewrites) plus a 1.3 GB wine prefix.
- `kwinRun.sh N` runs it unattended and invisibly in its own
  `kwin_wayland --virtual` compositor.  Nothing to install, no sudo.
- `BoosterlandAutoLoad` loads the quicksave from the main menu into
  SPACECENTER, which is what lets kRPC start at all.
- `fly.sh N ...` runs the project's `quickfly.py` on that instance's ports.

## The measurement that matters

Three flights on the stripped instance, committed config
(`AIM_BIAS_EAST_M = 628`, quicksave UT 16677), against the known result for
that save of **12 / 115 / 131 / 10 m** from the pad:

| flight | log | from pad | N | E | arrival |
|---|---|---|---|---|---|
| 1 | LOG67 | 74 m | +4 | -74 | 0.5 m/s, `legs=0.1m vs=-2.1` |
| 2 | LOG68 | 66 m | -5 | -66 | 0.2 m/s |
| 3 | LOG69 | 52 m | +0 | -52 | 0.4 m/s |

Median 66 m, mean 64 m, all intact.  The reference group's mean was 67 m, so
the centre did not move.  Treat the strip as physics-neutral only to the
precision of that reference spread (~120 m): this rules out a gross change,
not a 20 m one.

**Lead worth chasing:** the scatter is 52-74 m against the reference 10-131 m.
Per CLAUDE.md failure 12 the spread is set by which tick boostback exits on,
and a stripped install has fewer mods perturbing frame timing -- so a tighter
install may buy precision, which is the half the aim bias cannot fix.  Three
flights against four; measure it before believing it.

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

## Two things measured since, both about this rig

**The concurrency worry above did not survive measurement.** Flown properly --
`quicksave`, `AIM_BIAS_EAST_M=0` on both sides -- solo and four-instance
flights are the same population, and the boostback exit they were supposed to
differ through is identical:

| | flights | `BOOSTBACK -> COAST` | `held=` |
|---|---|---|---|
| solo | 286, 125 m | 16712.48-16712.62 | 0.20-0.24 s |
| four instances | 214, 238, 267, 273, 466 m | 16712.46-16712.54 | 0.20-0.22 s |

The earlier "220 m concurrency effect" was an artefact of comparing solo
flights flown at the *default* aim bias against loaded ones flown at 0 --
`config: (defaults)` and `config: AIM_BIAS_EAST_M=170.0` are the same flight,
because 170 is the committed default. **Read the numeric value, never the
label.** Keep comparing parallel against parallel as a precaution, but the
instance count is not currently known to cost anything.

**`logs/` is shared, and log naming used to race.** `Logbook` picked
`highest + 1` and *then* opened it, so two flights starting together claimed
one name and the second truncated the first -- one file holding two
interleaved flights, which still looks exactly like a log. It surfaced only
as a sweep reporting the same LOG number under two different configurations.
Fixed with `O_CREAT | O_EXCL`; be sceptical of any parallel-flight log read
before that.

## Not yet established

- How many instances fit.  One uses about a core and ~5 GB; the machine has 16
  cores and 30 GB, so four looks comfortable, untested.
- Whether the aim bias calibrated on the live install still holds exactly on a
  stripped instance beyond the scatter above.

## All four games can die in the same second, and it is not a suspend

On 2026-09-09 at 22:12:42 every KSP process exited inside 70 ms, with the
orderly Unity `OnDestroy` cascade in each `KSP.log` -- the same signature
`keepNotes.md` warns about.  It was **not** a suspend this time: the machine
had been up since 21:13, `systemd-suspend.service` had no entries, and 24 of
30 GB were free.  The `ksp<N>-boosterland` scopes stayed `active running`; only
the games inside them died, and a browser scope and the sweep's own scope are
logged terminating in the same second.

So this reads as a session-wide SIGTERM rather than anything the harness did.
What it costs is a sweep's last cells (`FAILED rc=1 ... Connection refused`),
which is recoverable -- restart with `./start.sh` and re-run those cells.
**Check three things before theorising**, in this order: whether all four died
in the same second (external kill, not a crash), `journalctl --user -b -u
systemd-suspend.service` (the suspend story), and whether other user scopes
ended at the same moment (session-wide).

## Traps that already cost a session

All detailed in `keepNotes.md`; the headlines:

1. `mkbase.sh` must not wipe `GameData/*/PluginData` -- it holds mods' answers
   to first-run prompts, and KSPCommunityFixes' FastLoader dialog blocks the
   loading screen where nobody can click it.  Silent: no error, no CPU, no log.
2. `HighLogic.CurrentGame = game` before `game.Start()`, or it throws inside
   `Game.Start()`, Unity swallows it per callback, and the menu renders at
   60 fps forever looking exactly like a freeze.
3. Load into `SPACECENTER`, not `FLIGHT`.
4. ReStock is kept deliberately: it rewrites `@mass` and `DRAG_CUBE` on 15 of
   the booster's 20 parts.
5. Never let a stripped instance re-save a `.craft` you care about -- the
   ReforgedRedux colour data is written out of the file permanently.

## Wrong theories, recorded so they are not retried

The main-menu stall was blamed, in order, on gamescope's headless backend, the
session lock (`LockedHint=yes`), window focus, and `-popupwindow`.  All four
were wrong; it was trap 2 above.  The focus theory was the most convincing and
was built on a heartbeat log placed after an early `return`, so its silence
proved nothing.  Check a diagnostic can fire in the failing case before
trusting its absence, and grep for `[EXC]` first.
