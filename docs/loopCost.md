# Writing fast control loops and farm scripts

Every autopilot here runs under the time-scale governor
(`common.pacing.ScaleGovernor`): the farm runs the game as fast as each tick's
**wall cost** allows while still delivering a command every control interval.
So a tick's wall cost *is* the farm's throughput, and anything that makes one
tick slow, even once, slows a whole phase. On a live flight at 1x the same cost
is latency between reading the state and the command landing.

Measured on the spaceplane on 2026-10-02: a round of six flights from orbit
went from 643 s to about 450 s using only the practices below, with no change
to what the vehicle flies. The numbers come from `logs/rot-rpc*-1002.txt` and
`LOG4727`-`4774`. The journal has the history.

## Measure before changing anything

- **Every spaceplane log ends with four lines.** Read them before guessing:
  - `loop rate`: game-s per tick and wall-s of work per phase, `pk=` the worst
    tick.
  - `rpc per tick`: calls per tick, wall seconds spent in kRPC, and the top
    procedures.
  - `wall per phase`: the real wall seconds per phase, and the speed achieved.
  - `slowest ticks`: each phase's three worst ticks and the calls they made.

  They come from `common/rpccount.py` (`RpcCounter().install(conn)`) and
  `common.pacing.LoopRate`. A new autopilot should install the same.
- **Commanded is not achieved.** `testInstances/kspN/timescale-status.txt`
  says what the game actually delivered. If the governor asks for 15x and gets
  4.5x, the game's main thread is the limit, not the autopilot. A thread at
  90%+ in `top -H` is the confirmation.

## In the control loop

- **Read once and cache what never changes.** Examples are
  `body.surface_gravity`, radii, part lists and module names. A property
  costs a round trip on every read, and `surface_gravity` was 2-4 of them a
  tick on final. Cache per object (see `Autopilot.surface_gravity`), not in
  `__init__`, so tests that swap the object still work.
- **Batch independent reads into one request** (`common.krpcbatch.ask`). The
  protocol carries a list of calls, and the server answers them in one frame.
  - Fourteen `SimulateAerodynamicForceAt` calls cost 10-45 ms one at a time
    and 6-7 ms batched, with bit-identical results.
  - The answers come from the same game moment, which separate calls do not.
  - Only reads go in a batch; setters can't.
- **Stream what you read every tick; poll what you need now and then.** A
  stream costs no round trip, but it pushes every frame whether or not anyone
  reads it.
- **Stop reading what can no longer change.** Fuel totals are fixed after the
  drain; transfers move fuel between tanks but don't change the totals. Track
  it yourself instead.
- **Skip writes that change nothing.** Throttle, brakes, gear and target
  direction resent with the same value are a round trip each.
- **Do nothing under rails warp.** The trajectory is a Kepler orbit and every
  call is slow there: COAST's 19 calls a tick cost 160-215 ms under warp
  against 13 ms out of it. Compute when the warp must end, warp to it, and
  tick again afterwards.

## Waiting

- **Wait in game time, never with `time.sleep`.** A surface deflecting, a
  transfer completing and an engine spooling all happen in game time.
  - `Autopilot.settle_game(seconds)` waits on `ut`, asks the governor for its
    ceiling (nothing is commanded meanwhile), and falls back to a wall sleep
    if the clock does not move (a paused game).
  - The control-surface probe slept 0.6 wall-s about 25 times at 1x: a
    15-second tick on every flight. It now takes about 5 s.
- **Never let a wait run unbounded.** Put a wall-clock deadline on every poll
  loop, and say in the log when it fired.

## One-off work and the governor

- **A one-off expensive tick slows its whole phase** unless it is kept out of
  the governor's estimate:
  - the fuel-to-nose scan took 300 ms and 457 calls once;
  - that one tick held COAST at 4.4x for 320 game-s, on ticks that cost
    13 ms.
- The governor now sets aside each phase's single worst tick
  (`GOVERN_PEAK_SKIP`), ignores ticks under rails warp, and resets its
  carried cost at a phase change. A slow tick that *recurs* still governs,
  which is what protects the deorbit burn (spaceplane failure 91).
- **Prefer making setup work cheap, or doing it before it's needed.** Put it
  at a phase boundary where the scale is already low, or batch its reads.
  Don't put it inside a phase that needs a fine control interval.
- **The frame quantum follows the control interval**
  (`TIMESCALE_QUANT_FRACTION`, 0.2 of the interval, floor 0.05 s). A phase
  that commands once a second does not need commands landing every 0.05 s.

## Farm and harness scripts

- **Use absolute paths in anything that `cd`s or runs in the background.** A
  CPU sampler that changed into `testInstances/` wrote nowhere and waited
  forever for a done-marker it couldn't find.
- **Bound every wait loop.** Use a timeout or `$SECONDS` ceiling, so a typo
  costs one round, not an orphan that lives all night.
- **Match processes precisely.** `pkill -f pattern` also matches the shell
  running it (exit 144). Check with `ps -eo pid,args | grep [p]attern`.
- **Don't edit a script while a batch runs it.** Bash reads scripts
  incrementally, so edit a copy.
- **Report what was achieved, not what was asked for.** That applies to time
  scale, warp index and quantum alike.
