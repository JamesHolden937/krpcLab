# Cross-cutting rules, in full

CLAUDE.md carries one line per rule; this is each rule with the case that paid for it. Read the rule here before designing a change to a law, a constant, a measurement or a comparison.

These bite in both projects and are the ones a change most often violates.

- **Never try ANYTHING that involves a ship-fitted constant** (the user,
  2026-11-15). Not a flag, not a batch, not an arm, not an intermediate
  step, not "measure it first and replace it later": if a mechanism, a
  comparison or a flight depends on a number fitted to one craft or one
  save, replace that number with a measured or physical quantity *before*
  building or flying anything on it. If that cannot be done yet, the work
  waits. Stop a running batch the moment it is found to rest on one.
- **Fine-tuned methods are not permitted** (the user, 2026-11-15). A law
  must work on every vehicle of its kind without re-fitting: no mode
  switches at fitted altitudes, distances, speeds or times (the booster
  burn's 2500/250/4 m steering bands are the example), no gains or margins
  sized on one craft or one save. Every threshold and gain is derived from
  a quantity the program measures on the craft in flight (thrust, attitude
  bandwidth, torque, mass, drag) or from physics, and where a mode change is
  needed it falls out of a continuous law (a gain capped by the measured
  bandwidth) rather than a switch. A dimensionless policy (a tilt share, a
  gain ratio from optimal control) is allowed; a length, time or speed
  fitted to make one flight land is not. Don't fly a batch whose arms are
  re-fits of such a constant: replace the constant first.

- **A flag ends its life as the default or deleted -- never parked off.**
  A mechanism goes in behind a flag so it can be measured; once measured,
  it is promoted to the default (it won) or its code is deleted (refuted,
  null, or superseded), with the result in the journal and the code in
  `git log`. The user flies the defaults live, so a winner left off never
  reaches them -- the spaceplane's best configuration sat behind three
  flags for two days -- and every loser left in makes the next change
  harder to read: by 2026-10-09 the spaceplane had 48 off-by-default flags
  and ~20 were dead. "Off until paired" is not a state; pair it or delete
  it.

- **Measure in game, not in the sim.** `boosterland/tests/fakeksp` models a point mass with
  near-instant pointing and no aerodynamic torque, and integrates its own truth
  with the scheme it is judging. Six changes that improved a sim sweep made the
  real flight worse. Treat a sim improvement as a hypothesis and confirm it with
  `quickfly.py` / `quickglide.py`. See [docs/boosterland/tuning.md](docs/boosterland/tuning.md).
- **A knob that changes nothing may be disconnected, not powerless.** Before
  concluding "X is not the lever", check that X moved what it commands. See
  spaceplane failure 10a, which cost this project a wrong headline finding.
- **One flight per configuration is not a measurement, and neither is
  three.** The spaceplane's along-track scatter is **sd 9.6 km over 22
  flights of one configuration** — so the first batch of any comparison is
  n flights of *one* arm, not one flight of n arms. An ordered
  dose-response over three-flight arms is not evidence of a gradient:
  spaceplane failure 27 is that mistake made, adopted, documented and
  retracted inside one session. Also 10d, and boosterland's accuracy
  history.
- **A correlation across flights the autopilot chose is not a sensitivity.**
  Interface speed against landing miss correlated 0.88 over 79 flights at
  11 km per m/s, and the real figure is 0.2-1.2: both variables are
  consequences of the burn, and the regressor was carrying the geometry's
  effect. If the quantity can be *set* — `savegen.py` sets it, on an entry
  save as well as an orbital one — set it and measure the slope directly.
  Spaceplane failure 30.
- **Read the signed miss, not the distance.** A great-circle distance cannot
  tell an overshoot from an undershoot; `long=`/`cross=` and the N/E offsets
  can. That ambiguity has cost this project two sessions.
- **Measure over several entry states, with `AIM_BIAS_*` at 0.** One quicksave
  is one separation state and a number tuned against it proves nothing about
  the next. Fit a calibration from two points, never from one flight's
  residual.
- **A prediction error found is the top priority, and it is fixed at its
  source -- never hidden** (the user, 2026-11-17). The moment a prediction
  and the flight disagree (landed minus predicted, plan minus flown), stop
  the work in hand and find *why*: which term, which delay, which tick. Fix
  that cause in the code or the vehicle's handling. Do not absorb it with a
  bias, a margin, a gain, a closed loop that steers it out late, or a
  re-fitted constant -- each of those leaves the error in place and only
  hides it. The booster burn landed ~70 m long of its own ignition
  prediction for weeks while the divert spent its authority steering it
  out; the cause was the throttle opening 0.3-0.4 s after the trigger
  (noaim-1117).
- **The propagator must fly the law the vehicle flies.** `landing_command` and
  the spaceplane's speed cap are shared by the predictor and the control loop
  on purpose: a prediction of a law nobody flies is a prediction of a
  trajectory nobody flies.
- **A missing answer must not be allowed to look like a good one** — return
  `None`, not a sentinel that a threshold test will accept.
- **A quantity measured from flight data describes the vehicle *as it was
  flown*, and every change to how it is flown silently invalidates it.**
  `ALPHA_TRACKING` was taken before the attitude controller was tuned and
  read 0.87 where the tuned vehicle delivers 0.74 — and the deorbit's stretch
  bound was making decisions on it. Re-take a derived table after anything
  that changes the plant, and prefer tables something recomputes.
  Spaceplane failure 23.
- **A guard whose stated reason does not survive inspection may still be the
  only thing enforcing a constraint nobody wrote down.** Removing one whose
  argument was airtight cost three flights and 110 km; what it was really
  doing was keeping the deorbit search out of the shallow regime the
  propagator cannot be trusted in. Find out what it does before removing it,
  and write *that* down. Spaceplane failure 21, boosterland failure 17.
- **A constant that has to be re-fitted after unrelated changes is a missing
  model wearing a constant's clothes.** `DEORBIT_LONG_BIAS_FRACTION` moved
  five times in one session, every time forced by a change elsewhere. The
  question is not "what value?" but "what quantity is this standing in for,
  and can the program compute it?" — here, the span of arrivals the glide can
  still reach (`guidance.deorbit_window`). Spaceplane failure 19.
- **A phase must hand over a state the next phase can fly, and a control
  law that steps is a law no vehicle flies.** Three of this project's
  landing failures are the same shape: the rollout commanded zero angle of
  attack at 90 m/s (a nose-over) after previously commanding none at all (a
  tail-scrape); the flare held wings level for its whole ten seconds and
  landed whatever cross-track it started with; and `aim` points the nose at
  the *velocity*, which on short final lands the vehicle crabbed -- 23
  degrees of sideslip at the first rollout tick, 51 two seconds later, gear
  torn off at 48 m/s on a runway it had just landed on. Ramp the command,
  and near the ground reference it to the runway rather than the airflow
  (`Autopilot.aim_runway`). Spaceplane failures 31 and 34.
- **A clamp can make two configurations fly identically, and the log will
  faithfully report the one that was ignored.** `deorbit_aim` returned
  `max(bias, fraction * arc)`, so with the fraction retired a configured
  `-3000` flew as `0` -- four in-game batches were flown believing they were
  aimed short. Spaceplane failure 33, and `testInstances/HANDOFF.md`'s
  "read the numeric value, never the label" from the other side.
- **A control loop that cannot serve its own commanded interval is flying a
  different vehicle, and the log has to say so.** Every spaceplane log ends
  with a `loop rate` line giving, per phase, the game-seconds between commands
  and the wall-seconds one tick costs. Read it before comparing two logs: a
  phase whose first number is far above the tick it asked for was measured
  through a latency, not a vehicle. `Config.TIMESCALE_GOVERNOR` keeps that
  from happening by making the time scale serve the interval rather than the
  other way round — matched against itself it took the entry's arrival from
  -3226 m sd 806 to +294 m sd 171 at unchanged throughput. Spaceplane
  failures 63 and 65.

- **Ask what the vehicle can do that the autopilot never commands.** Every
  constant in `config.py` rations an authority the code already has, so a
  search over values cannot find one that was never wired. Three were found
  in an afternoon just by asking: `ALPHA_MIN_DEG` is 0 on an airframe whose
  own swept table reads **ClA 8.8 at zero alpha** — it physically cannot
  unload and nothing in the log would ever say so; `HAC_BANK_MAX_DEG` is 45
  while the glide routinely flies **67**, so the cone's descent authority is
  a policy and not a limit; and nothing anywhere commands an airbrake,
  though every brake constant in the file is a wheel brake and the Shuttle
  this cone is borrowed from uses a speedbrake continuously. The table even
  stops at alpha 0, so half the polar of a cambered wing has never been
  measured. When a phase is short of authority, find out whether the
  authority exists and is merely uncommanded **before** re-fitting the
  constant that rations it.

- **A measurement that shares a constant's name is usually a different
  quantity, and the agreement is what makes it dangerous.** Three arms were
  spent on this in one session, each replacing a fitted constant with a
  live measurement that looked like the same thing. `HAC_LD` 1.86 is the
  ratio the cone must *plan* with; the vehicle *achieves* 1.42, and flying
  the achieved value put it 3.9 km from the gate — `HAC_LD`'s own comment
  had already recorded that failure ("1.35 was 25% low", LOG1315, overflew
  by 3.5 km). `MARGIN` discounts the *top* of the lift curve; `LiftTrim`
  measures lift at *whatever alpha is being flown*, and substituting it
  gave a stall speed 25% low (+1405 m against +900, 8 an arm).
  `APPROACH_BEST_LD` spans rollout-to-wheels **including the flat flare**,
  so the glide ratio is not it — and the committed 4.2 measures 3.96 in
  flight while the comment beside it says 2.08. In every case the two
  quantities agreed somewhere, which is exactly why the substitution looked
  safe. Before replacing a constant with a measurement, say what each one
  is a quantity *of*, and check they are the same one.

- **Re-run the tool; do not read the prose.** A number quoted in a comment
  describes the configuration that was flown when it was written.
  `HAC_LD`'s "tracking 1.07-1.11" reads 1.27-1.39 today and
  `APPROACH_BEST_LD`'s "2.08 sd 0.27 over 41 flights" reads 3.96 sd 0.41 —
  both were taken as current, and both were wrong in a way that changed a
  decision. `conesum.py`, `landsum.py` and `armsum.py` recompute all of it
  in seconds. Failure 23's rule, aimed at the comments rather than the
  constants.

- **When three attempts on one mechanism come back null, the next thing to
  change is the method, not the fourth value.** A refuted mechanism is
  information about the mechanism; a *run* of refuted mechanisms is
  information about the approach that picked them. So change what is being
  varied: measure a different quantity, act at a different point in the
  chain, or replace the law instead of re-fitting its constant. The
  generality gap survived two hundred flights of propagator work and fell to
  a release valve nobody had thought to look at (failure 61); the approach's
  speed loop was re-tuned all session and what it needed was a law that could
  make speed as well as lose it (failure 65). A constant on its fifth value,
  a diagnostic that keeps agreeing with itself, and a fix that only works on
  the save it was fitted to are all the same signal. Write down which
  approach was abandoned, not only which value was.

- **A measurement whose only consumer is a hand-copied constant cannot be
  contradicted.** `planeprobe`'s subsonic lift was wrong by 1.8x for the life
  of the project because nothing read it except a human transcribing numbers
  into `config.py` — which set the stall speed, the approach speed, the gate
  and the flare for an aircraft that does not exist. Prefer a measurement
  something recomputes (`polar.py` reads it back off the logs, the alpha
  ceiling is learned in flight); where a constant really is transcribed, say
  in its comment what would disagree with it. Spaceplane failure 13.
