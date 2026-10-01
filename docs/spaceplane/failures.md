# Spaceplane: failure modes already paid for

Referenced by number from [design.md](design.md) and [journal.md](journal.md). Citations of
*boosterland* failures point at [docs/boosterland/failures.md](../boosterland/failures.md).

**Entry 10 is long, partly wrong, and kept anyway.** It was the open frontier
for several sessions and most of what it concluded has since been overturned
by measurement — its headline ("the deorbit aim cannot fix it") was an
artefact of a disconnected knob, and the aerodynamic table it argues from was
contaminated. The corrections are 10a-10d and each says which part of 10 it
replaces. It is not deleted because *how* those conclusions were reached is
the more expensive lesson: every one of them was a plausible inference from a
derived quantity, drawn from one flight per configuration, in a system whose
scatter nobody had measured. **Read 10 as a case study, and 10a-10d as the
results.** The current frontier is 10d.

1. **The deorbit search solved a 335 m/s burn on the first tick**, with the
   runway 1306 km away and **receding**. Two bugs, both about measuring
   distance on a sphere:

   - `surface_distance` is the *short way round*, and in orbit the vehicle is
     heading the other way half the time. `trajectory.forward_arc` measures the
     angle about the orbit normal, taken forwards.
   - `miss_components` was a **chord projection**, fine within tens of
     kilometres and catastrophic at a thousand — and silent in the worst way. A
     predicted landing point on the far side of the planet has an offset from
     the gate that is almost entirely *radial*, so both tangential components
     come back near zero and the guidance reads a perfect hit. The first
     in-game flight logged `long=+154 cross=-1140` while 1533 km out. It is now
     an arc length through the tangent plane at the gate, exact at any
     separation.

2. **The deorbit burn's stop test could not become true.** It watched for the
   *search* to stop finding a burn above `DEORBIT_DV_MIN`; as the burn proceeds
   the dv still required falls towards zero, the floor is 10 m/s, and the
   condition only trips once the requirement has fallen through it — about ten
   seconds of extra thrust at 13 m/s^2, or 130 m/s of overburn on a 60 m/s
   burn. `guidance.deorbit_progress` asks the right question instead, in one
   propagation rather than sixteen: "does the trajectory I am already on do the
   job?"

3. **And then its sentinel value stopped the burn immediately.**
   `deorbit_progress` returned `+1e7` for "the arc never got down to the gate",
   which is precisely the state every burn starts in — and the stop test is
   `progress >= bias`, which a large sentinel satisfies. In game: `DEORBIT ->
   DRAIN range error +10000000 m` after 35 kg of propellant. It returns `None`
   now. **A missing answer must not be allowed to look like a good one.**

3a. **And with the sentinel gone, the same stop test was still wrong three more
   ways.** In the order they hid each other:

   - **The runaway guard was timed from phase entry.** DEORBIT spends minutes
     in orbit waiting for a favourable pass, so the 60 s `DEORBIT_MAX_BURN_S`
     had expired before the engine was ever lit. Timed from the first burning
     tick now.
   - **The sense was inverted.** The range error approaches the aim from
     *above* — an unburned orbit lands most of the planet long — so `progress
     >= bias` stops on the first tick that produces a number at all: a burn
     shut down 770 km long.
     `test_the_burn_stops_by_falling_to_the_bias_not_by_reaching_it` holds the
     direction rather than the threshold.
   - **A debounce cannot end this burn**, and that is boosterland failure 12
     failing to transfer. Here the range sensitivity is ~25 km per m/s, so
     three ticks of full throttle is ~100 km on the ground: four flights exited
     at -85, -99, -128 and **-1957** km.

   `guidance.deorbit_remaining` is the answer, and it is boosterland's
   `miss_gradient` doing the same job: two propagations, one with
   `DEORBIT_PROBE_DV` more taken off, give metres of range per m/s; the
   remaining dv is the error divided by that, the throttle tapers it over
   `DEORBIT_TAPER_S`, and the exit is the first tick that owes nothing
   (`DEORBIT_EXIT_TICKS` is 1).

   | | exit error against a 50 km aim |
   |---|---|
   | debounced threshold, 3 ticks | -85, -99, -128, -1957 km |
   | tapered on the measured gradient | **+49842, +49954, +49976 m** |

   **The probe has to be sized to the step, not to a constant.** The next batch
   split two and two — `+49954`/`+49818` against **-73 km** and **-183 km** —
   and the exit lines differ in one column: the good pair owed 0.01-0.04 m/s,
   the bad pair 0.5-0.6. The sensitivity is not fixed (~25 km/(m/s) shallow,
   **~290 km** steep), so a fixed 2 m/s probe is a tangent measured across
   nearly 600 km of range. `deorbit_remaining` now **re-probes at the step it
   is about to take**: once the first tangent says the burn owes less than one
   probe, the probe is set to that amount and the gradient re-measured, turning
   a tangent into a secant across exactly the step contemplated. The taper
   horizon is also bounded below by the *measured* tick, since two entry
   propagations a tick means the loop does not keep to `TICK_S`.
   The lesson is not that debouncing is wrong but that it is a *noise* fix: it
   buys repeatability where the last tick is cheap, and here the last tick was
   worth more than the glide's entire steering authority.

3b. **RCS left on through the vacuum coast emptied the monopropellant** — 3
   kg/s of hunting by kRPC's autopilot against 600 kg aboard, the whole
   attitude budget before the entry that needs it. `COAST_RCS` is off: an angle
   of attack established in vacuum has nothing to fight and minutes to get
   there. The tell is `m=` falling on a `thr=0.00` line.

3c. **The orbital wait is rails-warped, and the first rule for it never fired
   once.** Keyed on the runway coming inside the reach window, warp never
   engaged, because the runway is inside that window for the whole wait — what
   the phase waits on is the *phasing*. It warps on the absence of a solution
   instead, and the tick that finds one drops to 1x and re-asks rather than
   committing on a warped state. Worth about 10x on the minutes before the
   burn; the entry itself is not warpable.

4. **The gate capture fired 50 km overhead.** It was a horizontal distance test
   alone, so it triggered while the vehicle was still fifty kilometres above
   the gate doing Mach 7, handing a hypersonic vehicle to the approach law. It
   needs the altitude test too.

5. **The autopilot could not be engaged at all, and retrying did not help.**
   `AutoPilot.engaged = True` threw `IndexOutOfRange` from
   `ModuleGimbal.GetPotentialTorque` — boosterland failure 7 exactly, except
   that there it is *transient* and here it is not: the engine is inactive in
   orbit, its gimbal has no thrust transforms to index, and it throws forever.
   One flight spent 150 seconds retrying while the nose drifted 50 degrees off
   prograde. The fix is `Lock Gimbal`, which takes it out of kRPC's torque
   total; it stays locked for the flight, the only burn being a few seconds of
   retrograde thrust on a vehicle with reaction wheels, four RCS blocks and
   control surfaces. **Do not assume a KSP-side exception is transient just
   because the sibling project's was**; retry *and* have a way to remove the
   cause.

6. **A patch removed `trajectory.predict` and nothing noticed** until a flight
   died eight minutes in with `AttributeError` — every import still succeeded,
   because the missing name is only looked up when a propagation runs.
   `spaceplane/tests/testSpaceplane.py` now calls every guidance entry point once on a
   plausible state.

7. **The entry was aimed at a mass the vehicle would not have, and it cost
   68 km.** `logs/LOG599` exited its deorbit burn **46 m** from a 50 km aim and
   arrived 68.5 km short, hitting the ground at 44.5 m/s of sink. The predicted
   `long` swings 88 km between the burn's exit (+49954) and the early glide
   (-37949) across a coast in which nothing happens — two different
   trajectories: `deorbit_solution` and `deorbit_remaining` were both called
   with `snap.mass`, which during DEORBIT is **9.132 t**, and the drain then
   takes the vehicle to **6.693 t** before it flies a metre of the entry. The
   same `Cd*A` over 27% less mass is more deceleration, so the real entry flies
   shorter than the burn aimed for, every time and in the dangerous direction.
   `Autoland.entry_mass` subtracts the propellant aboard at
   `RESOURCE_KG_PER_UNIT` and both deorbit calls use it. This is the rule the
   drain's own phase exists for, applied one phase too late.

8. **The entry stretched itself out of the atmosphere.** `logs/LOG598` went
   through periapsis at 23.5 km at Mach 4.1 and **climbed at +71 m/s**, with
   `bank=-0.1` — wings level, every bit of the lift vertical. When the entry is
   short the solve wants lift and the cheapest lift is bank zero; that deep in,
   it is over a g on this airframe. Failure 7 is why it was short, so the two
   are one bug at two altitudes — but the glide should not be able to command
   this whatever the reason.

   **No threshold on the miss can catch it, and that is the point.** A
   ballooning arc does come back down, so `predict` reports a range for it and
   `deorbit_progress` reports a plausible error — for a landing one or more
   passes later. It is a property of the *trajectory*, not of the miss.
   `Prediction.skipped` is that property (into the air by
   `SKIP_ENTER_MARGIN_M`, then back out of it), and three places use it:
   `deorbit_solution` scores a skipping burn like an undershoot so the search
   walks towards *more* dv, which is what lowers the periapsis;
   `deorbit_progress` returns `None` so the burn keeps thrusting; and
   `verified` refuses a glide command that balloons, falling back to
   `SOLVE_BANK_MIN_DEG` of bank, the one control that always sinks the vehicle.
   **Arriving short having stayed in the air beats arriving on a later pass
   having left it.**

9. **Nothing had ever measured the heating**, on a Mach 7 entry flown at
   maximum lift. It is instrumented now rather than reasoned about: `Telemetry`
   streams `skin_temperature` for every part with a real skin limit — streamed
   and not polled, because twenty-odd streams cost one setup and nothing per
   tick where `parts.all` every tick is twenty-odd remote calls inside the
   control loop — and `skin=` on every line is the hottest part as a fraction
   of *its own* limit, with `THERMAL_WARN_FRACTION` naming it. Deliberately a
   **warning and not a control input**: the corridor really is bounded below by
   heating and above by the skip, but there is no measurement here yet to size
   that trade. Baseline in vacuum is 0.23. Note failures 8 and 9 are the same
   question — a vehicle that balloons off a Mach 4 pass re-enters hotter.

10. **The glide is ~40-70 km short and the deorbit aim cannot fix it.** This is
   the frontier, and the useful part is the negative results.

   > **Read 10a first.** The headline of this entry is wrong. The aim *is* the
   > lever; the sweep that said otherwise was measuring a disconnected knob.
   > The rest of the entry is kept because its other findings stand and
   > because the way this one failed is the lesson.

   Everything upstream is solved and confirmed in flight: the deorbit exits
   4-300 m from its aim across a dozen burns and four entry states; `long` at
   the first GLIDE tick reads -1753 m and is solved to +45 m within three
   ticks; cross-track holds inside a kilometre where it used to reach 23 km.
   And the vehicle still lands 35-78 km short, every time.

   **The aim was swept and it is not the lever.**

   | `DEORBIT_LONG_BIAS_M` | `long` at glide entry | landed |
   |---|---|---|
   | -60 km | +30 m | -65 km |
   | -25 km | -1416 m | -78 km |
   | +25 km | +241 m | -53 km |
   | +50 km | -1753 m | -35 to -62 km |
   | +90 km | **-25000 m** | -57 km |

   Across a 110 km span of aim the state at the 58 km interface moves less than
   2 km: the upper atmosphere equalises the entry energy whatever the burn
   aimed at, so a bigger or smaller burn changes *where* the vehicle enters and
   not how much energy it has there. Only +90 km breaks that, and in the
   **wrong direction** — a smaller burn is a shallower entry that bleeds its
   energy high in thin air, so aiming 40 km longer arrived 23 km shorter.
   Raising the bias on one flight's residual was a mistake this file's own rule
   (fit from two points) exists to prevent.

   **So what is left is a propagator that over-predicts the glide's range by
   about 10%** while knowing the entry state correctly. Two candidates are
   ruled out by measurement:

   - *Not the alpha ceiling.* The miss re-opens (+59 m at 54 km to -17 km at
     40 km) while alpha tracks 31.2 deg against 30.6 commanded. The ratchet was
     also made two-way (`ALPHA_RECOVER_DEG_S`), because a one-way ratchet turns
     a transient — a bank reversal throws the tracking error for a second or
     two — into a permanent loss of the range authority, and on this airframe
     the ceiling *is* the range control. Worth having; not the cause.
   - *Not a stale aero table.* Unlike boosterland failure 13, the table is
     re-swept two rows a second and `set_profile` re-aims each Mach row at the
     altitude the descent will use it at.

   **`aeroaudit.py` is what to reach for, and it found two things in one pass
   over a log already on disk.** It recovers the drag and normal acceleration
   the vehicle actually achieved — finite differences of `v`, `vs` and the
   timestamp, less gravity and the sphere's own `v^2/R` — and compares them
   against the model the guidance was flying on, which the log also carries
   (`dec`, `cla`, `cda`, `bank`, and `q/m = dec/cda`, so no density is needed).
   Read the **L/D ratio** and not the absolute columns: Coriolis is neglected,
   which at 2 km/s is 10-30% of the drag, and it is common-mode so it cancels.

   | alt | Mach | L/D model | L/D actual |
   |---|---|---|---|
   | 52357 | 6.6 | 1.07 | 4.62 |
   | 23926 | 3.0 | 0.98 | 0.96 |
   | 12921 | 0.9 | 1.06 | **0.41** |
   | 2036 | 0.2 | 1.63 | 1.56 |

   0.41 against an airframe measured at **3.39** subsonic best glide: the
   vehicle was not gliding, it was falling.

   > **That table is wrong, and the warning above it was not strong enough.**
   > See 10b: the aerodynamic force is available directly from kRPC, and read
   > that way the model and the vehicle agree to 0.3%. The Coriolis term the
   > caveat says "cancels in the ratio" does not cancel -- it is along-track,
   > so it lands almost entirely on drag, and the L/D ratio is exactly where
   > it does the most damage. `aeroaudit.py` reconstructs forces it does not
   > have to reconstruct; prefer the `act=`/`mdl=` columns.

   **The solver was parked in the range *minimum*.** Range against alpha has an
   interior optimum (1610 km at 5 deg, a minimum of 1265 at 20, 1823 at 32) and
   `SOLVE_ALPHA_MIN_DEG` sat *at the bottom of the bowl*, so a solve walking
   downhill settled there and stayed: a flight 23 km short at 31 km and Mach
   5.7 was commanding exactly 20.0 deg, the worst range it can fly, while
   trying to stretch. `_solve_range` now **brackets** — it propagates the
   current command, both ends of the span and the Newton step, and takes
   whichever lands nearest — which is immune to an optimum wherever it sits,
   and it moves with the state. **A guard that protects a bad algorithm from
   its own failure mode can be worse than fixing the algorithm, because it
   makes the failure *stable*, and a stable wrong answer looks like a converged
   one.**

   **What is left is the airframe, and it is not a guidance bug.** With the
   bracketing solve flying, the vehicle is still 45-63 km short and the `aoa=`
   column says why:

   ```
   57 km, Mach 6.8   aoa=30.0/30.7    holds what it is asked
   26 km, Mach 3.4   aoa=24.2/19.6    delivers 19 whatever it is asked
   16 km, Mach 1.5   aoa=20.0/31.5    trims nose-high, overshoots
   ```

   In dense air it drifts to its own trim point regardless of command. The
   measured polar therefore describes attitudes the vehicle **cannot hold**:
   `planeprobe` aims the airflow and reads a *force*, and kRPC will not report
   a moment, so nothing in this project has ever measured trim.

   **That measurement was then taken from the logs, and the obvious fix made
   things worse.** The `aoa=commanded/achieved` column of every GLIDE line of
   16 flights is an achievable-alpha table already on disk — 5467 samples, no
   flight needed:

   | q (Pa) | n | mean cmd | achieved | ratio |
   |---|---|---|---|---|
   | 0-200 | 209 | 29.2 | 29.8 | 1.02 |
   | 500-1000 | 206 | 30.2 | 30.8 | 1.02 |
   | 1000-2000 | 348 | 25.3 | 24.6 | 0.97 |
   | 2000-4000 | 1251 | 21.3 | 18.6 | 0.87 |
   | 4000-8000 | 2685 | 22.2 | 19.2 | 0.87 |
   | 8000+ | 462 | 20.8 | 16.7 | 0.81 |

   A **gain, not a ceiling**: thin air holds the command, above 2 kPa the
   vehicle delivers ~85%. `trajectory.tracked_alpha` applies it, which makes
   the propagation honest — and in game took the shortfall from 45-63 km to
   **99-118 km**. `ALPHA_TRACKING` is therefore `()` by default, with the table
   kept in the config.

   The reason is worth more than the change. Less effective alpha is less lift
   and so a *shorter* predicted range, so the dv that still lands on the aim is
   a **smaller** one — the search solved 75 m/s where the same state previously
   solved 140 — and that is a shallow entry whose extra range does not
   materialise. **Making one half of a model honest, while the search that
   consumes it is free to walk into the regime where the other half is least
   reliable, is not an improvement.** The shallow end is also where the skip
   lives (failure 8).

   **The steepness bound was flown and does not rescue it.** A 2x2 on one save:
   the bound demonstrably closed the shallow escape (with tracking on it solved
   156 m/s against 75 unbounded) and the result is still twice as bad.

   | tracking | `DEORBIT_MAX_TIME_TO_GO_S` | `long` at ~25 km |
   |---|---|---|
   | off | 1500 (default) | -48506 |
   | off | 1200 | -45362 |
   | **on** | 1200 | **-109106** |
   | **on** | 1100 | **-108341** |

   **A proportional gain is the wrong shape for this plant.** The solve simply
   *undoes* it: told that 0.85 of its command will be flown, it commands more,
   the factor cancels the inflation, and the prediction believes the inflated
   number while the real vehicle saturates near 19 deg whatever it is asked.
   The plant is a **saturation**: `achievable = min(command, holdable(q))`, and
   a ceiling cannot be undone by asking for more. That is what `ratchet_alpha`
   already does from live telemetry without a table — so the thing to improve
   is the ratchet's speed and recovery (`ALPHA_BACKOFF_DEG`,
   `ALPHA_RECOVER_DEG_S`), not the propagator's idea of the command.
   `ALPHA_TRACKING_ON` is left in at False. Note also what the bound costs on
   its own: at 1100 s the phase waited an extra orbit for a pass it would
   accept and then needed 205 m/s. Steepness is bought with waiting.

10a. **The aim was never connected to the outcome, and that is why it looked
   like it was not the lever.** `deorbit_solution` accepts every burn whose
   predicted arrival is within a tolerance of the aim and then picks one.
   The tolerance was `max(500, abs(DEORBIT_LONG_BIAS_M))` -- **the aim and
   the width of the band underneath it were the same number**. So raising the
   aim by 50 km widened the acceptable set by 50 km in the same direction,
   and the "take the smallest burn" rule went on selecting the same shallow
   candidates. Five values of the bias produced five nearly identical
   flights, which read as "the upper atmosphere equalises the entry energy"
   and was really "nothing downstream of this knob is listening to it".

   Three things were wrong at once and each hid the others:

   - **The band was two-sided.** It accepted arrivals up to a tolerance
     *short* of the aim -- the one outcome the whole design exists to avoid,
     since nothing after the burn can add energy. Aiming "50 km long" quietly
     permitted "50 km short". It is `wanted <= m <= wanted + tolerance` now.
   - **The tolerance was derived from the aim.** It is its own constant,
     `DEORBIT_TOLERANCE_M`.
   - **The search could not resolve the band.** One refinement pass leaves
     the dv grid at ~16 m/s, and the range sensitivity runs 25-290 km per
     m/s, so the arrival was resolved to hundreds of kilometres and then
     tested against a 25 km window. `DEORBIT_REFINE_PASSES` quarters the
     width five times.

   With those fixed, the same sweep on one save:

   | aim | 25 km | 100 km | 200 km | 300 km |
   |---|---|---|---|---|
   | landed | -85.1 km | -57.8 km | -18.9 km | **-6.0 km** |

   and a fixed 300 km aim across four entry states, repeated:

   | save | flights | landed |
   |---|---|---|
   | `qs_plane` | 3 | -4.2, -19.5, -10.5 km |
   | `qs_plane_inc` | 2 | -4.8, -9.5 km |
   | `qs_plane_south` | 1 | -14.1 km |
   | `qs_plane_high` | 2 | **+60.4, +56.8 km**, splashed |

   against -57 to -75 km for all four before. **The overshoot is not noise
   and not a failure of the aim's sign**: the elliptical save's glide was
   flying alpha at its stop with 60 degrees of bank on and still arrived
   long, so the aim was simply past what its glide could spend. That state
   also has the shortest entry (728 km to run against ~1100), which is the
   clue: what the bias compensates is a *relative* over-prediction of the
   glide's range, so it is expressed as `DEORBIT_LONG_BIAS_FRACTION` of the
   arc still to fly, with `DEORBIT_LONG_BIAS_M` as a floor.

   **The lesson is about the shape of a negative result.** "I changed X over a
   wide range and nothing happened" is *evidence that X is disconnected* at
   least as often as it is evidence that X does not matter -- and the two
   look identical from the outside. The check is cheap and was never done:
   log the quantity the knob is supposed to move (here, the solved dv) and
   confirm it moved. It did not; 140 m/s came back for every value of the
   aim.

10b. **Three sessions of aero theories died to one kRPC call.** Entry 10
   spends most of its length on candidate causes for a propagator that
   over-predicts the glide's range: the alpha ceiling, a stale table, the
   trim point, the range bowl. Every one of them was argued from
   `aeroaudit.py`, which recovers the achieved forces by **finite-differencing
   the velocity** and subtracting gravity and `v^2/R`.

   It never had to. `Flight` reports the aerodynamic force itself, along with
   the density and dynamic pressure the game used to make it:

   ```
   flight.aerodynamic_force     the whole force, in the body's rotating frame
   flight.atmosphere_density    the game's, not the table's
   flight.dynamic_pressure
   ```

   Three more streams, nothing per tick, and the telemetry line now carries
   `act=` (the vehicle's own `Cl*A`/`Cd*A`, from that force resolved about the
   air-relative velocity), `mdl=` (the table at the angle the vehicle is
   **achieving**, not the one it was asked for), `ld=` and `rho=`. Over 194
   glide samples of one flight:

   | | median actual / model |
   |---|---|
   | `Cd*A` | 1.013 |
   | `Cl*A` | 1.000 |
   | **L/D** | **1.000** |
   | air density | **1.000** |

   **The aero table is exact and the atmosphere model is exact.** Both were
   under suspicion for three sessions. What `aeroaudit` was reporting as an
   airframe gliding at L/D 0.41 was Coriolis landing on the drag term, and
   what looked like table error in the `cla`/`cda` columns was the comparison
   being made at the *commanded* angle while the vehicle flew five degrees
   less.

   Two lessons, and the second is the expensive one:

   - **Compare like with like.** The model's coefficients at a commanded
     attitude tell you nothing about an airframe that is not in it. `mdl=`
     exists because `cla=`/`cda=` had been quietly answering a different
     question on every line of every log.
   - **Before reconstructing a quantity, check whether the simulator will
     simply hand it to you.** Everything in entry 10 upstream of this point
     was inference from a derived number with a known 10-30% contaminant, and
     the undisturbed measurement was one property access away the whole time.
     The same question is worth asking of anything else this project derives.

10c. **Where it stands, and what the frontier is now.** On the committed
   defaults, two flights of each of four entry states:

   | save | landed | was |
   |---|---|---|
   | `qs_plane` | -3.4, -38.3 km | -75.0 |
   | `qs_plane_inc` | -33.3, -6.5 km | -62.7 |
   | `qs_plane_high` | -7.0, -6.4 km | -57.4 |
   | `qs_plane_south` | -22.9, -3.1 km | ~-58 |

   Mean **-15.1 km** against about -63 before, every flight on the ground
   under control, none splashed. **The systematic shortfall is mostly gone
   and what is left is scatter** -- five of these eight are inside 7 km and
   the other three are 23-38, with no pattern by entry state (`qs_plane` has
   both the best and the worst). That is the reverse of the situation entry
   10 describes, where every flight missed by 40-70 km in the same direction.

   So the open problem is no longer "the glide is short". It is **10d: the
   glide does not repeat**, and nothing here has yet looked for the cause.
   Two candidates worth separating before any more tuning, because both are
   visible in logs already on disk:

   - *The phugoid.* Several entries level off near 30 km and climb again --
     `vs` swinging from -200 to +60 m/s over a minute. Where a flight happens
     to be in that oscillation when the air thickens plausibly decides tens
     of kilometres, and it is not something the mean-of-a-reversing-entry
     propagator models.
   - *Bank-reversal timing.* The sign is chosen by an azimuth deadband, so
     two flights from near-identical states can reverse at different moments
     and spend different amounts of their lift vertically.

   Until that is understood, **any single-flight comparison on this vehicle
   is measuring the scatter and not the change**, and several of the
   conclusions above this line were drawn from exactly that.

   **The scale has to be the entry's own length, not the distance to the
   runway.** Taken as a fraction of the arc *still to fly*, the aim shrinks as
   the vehicle coasts closer -- so **waiting makes the target easier**, a pass
   that cannot be solved becomes solvable by doing nothing, and one flight
   duly sat until 410 km to run and committed on `DEORBIT_DV_MAX` exactly.
   `Prediction.entry_arc` is the arc flown *inside the atmosphere*, which is a
   property of the trajectory being judged and does not move while the phase
   waits. With it the same states commit at ~1100 km to run on 85-87 m/s:

   | save | fraction x range-to-run | fraction x entry arc |
   |---|---|---|
   | `qs_plane` | -11.3, -13.1 km | **-3.4 km** |
   | `qs_plane_inc` | -25.5, -65.3 km | -33.3 km |
   | `qs_plane_high` | -39.1, -37.8 km | **-7.0 km** |
   | `qs_plane_south` | -25.9, -25.6 km | -22.9 km |

   **And making the aim a function created a way for two callers to disagree
   about it.** `deorbit_solution` picks the dv whose arrival sits on the aim;
   `deorbit_remaining` decides when the engine has delivered it. While the aim
   was a single constant they could not come apart, and the first version of
   this change had the search aiming at a fraction of the entry while the
   burn still terminated at the old floor -- tens of kilometres of overburn,
   silently, on a vehicle that cannot get any of it back. `guidance.deorbit_aim`
   is the one definition both call, `deorbit_progress` reports its error
   already net of it so "owes nothing" is zero for both, and
   `TestTheDeorbitAimIsShared` holds it. The general shape:
   **when a constant becomes a function, look for everyone who was relying on
   it being the same number everywhere.**

10d. **The glide does not repeat, and the divergence has been located.**
   `logs/LOG706` and `logs/LOG710` are the cleanest pair this project has:
   same save, same config, deorbit solutions identical to 0.1 m/s (86.4),
   burn exits 70 m and 162 m from the aim, GLIDE entered 0.2 s apart at the
   same altitude and speed. They arrive at the gate **-28.9 km and -3.2 km**.

   Laid side by side they are the same flight down to about 31 km -- `long`
   within 400 m, `alt`, `v` and `vs` within a percent -- and then:

   ```
   31 km  vs=-58   LOG706 bank=-38.4 long=+5      LOG710 bank=-18.6 long=-423
   29 km  vs=-62   LOG706 bank= -1.7 long=+3658   LOG710 bank=+15.5 long=+5421
   26 km  vs=-100  LOG706 bank=+12.9 long=+5642   LOG710 bank=+58.5 long=-102
   20 km           LOG706 v=812 long=-179         LOG710 v=901 long=+3
   12 km           LOG706 v=223 long=-16770       LOG710 (14 km) v=397 long=-1124
   ```

   Both level off near 30 km (`vs` -58 against -160 higher up) and both then
   see the predicted miss swing several kilometres between ticks. **One rolls
   wings-level to stretch and the other banks to 58 degrees**, from states
   that differ by a few hundred metres of predicted miss -- and from there
   the energy histories separate for good. The vehicle that stayed banked
   arrived on the gate; the one that unbanked was 5 km long in prediction,
   then dived, lost 590 m/s in seven kilometres of descent, and finished
   17 km short and falling.

   So the scatter is not noise in the plant. It is a **bifurcation in the
   solve**, at the level-off, where the range is briefly insensitive to the
   controls and the miss changes sign between ticks -- and the bank magnitude
   the solve picks there is worth tens of kilometres at the ground. That is
   the thing to fix next, and it is probably a rate limit or a hysteresis on
   bank magnitude through the level-off rather than anything in the model.

   The original observation, for the record: two flights of the same save and
   config,
   whose deorbit solutions agreed to 1 m/s and whose ranges to run agreed to
   4 km, landed 18.9 km and 57.5 km short. Three flights at a 300 km aim
   spread -4.2 to -19.5 km. So the glide carries something like 15-40 km of
   run-to-run scatter, which is **larger than most of the effects this file
   records as measured** -- including several single-flight comparisons in
   entry 10 above, and the 5-9 km attributed to the learned alpha ceiling.
   Nothing here is safe to conclude from one flight per configuration, and
   the sweeps that predate this note mostly were.

10e. **The reversal was a relay on a rate, and it limit-cycled.** This
   corrects 10d. The scatter 10d located at the level-off is real and the
   pair of logs it names is the right pair, but its reading of them —
   "a bifurcation in the solve", one flight choosing to unbank and stretch
   while the other chooses to bank — is wrong. Both flights are doing the
   same thing, eighteen seconds out of phase with each other.

   Three measurements off `logs/LOG706` and `logs/LOG710`, both already on
   disk, through the 34-26 km band 10d points at:

   ```
   max |d bank|/dt        8.03 deg/s  -- BANK_RATE_DEG_S exactly, both flights
   ticks slewing >=7 deg/s     57%, 62%
   reversal interval      18 18 18 20 17 s   and   18 18 20 17 17 15 s
   |cross| peak            3378 m, 3144 m   against a 500 m deadband
   ```

   The bank command through the level-off is **not the solve's answer**. It
   is the rate limiter slewing between the stops, saturated for six ticks in
   ten, on a period so regular it can be read off the log. The `bank=-1.7`
   and `bank=+15.5` that 10d reads as two different decisions are two
   samples of one roll: LOG706 runs `-21.9, -1.7, +18.8, +39.1, +54.3` and
   LOG710 runs `-24.8, -4.6, +15.5, +35.7, +54.1`, both at 8 deg/s, in
   quadrature. **A control that is against its rate limit is not expressing
   a preference, and a log column cannot tell you which it is doing without
   the tick before it.**

   The oscillator is the cross-track test. `CROSS_DEADBAND_M` was an
   absolute 500 m at every range, and what it tested is
   `Prediction.cross` — the offset at the gate of a propagation that models
   a *reversing* entry, so one carrying no lateral lift at all. That makes
   it the vehicle's present lateral velocity carried forward: **a rate
   wearing a distance's units.** Relay-testing a rate against a fixed
   threshold, with an actuator that needs thirteen seconds to roll from one
   stop to the other, is a bang-bang loop with lag, and it produced exactly
   the textbook result — a fixed period and an overshoot set by the lag,
   here 6.8 times the band.

   That costs the range budget twice over. The propagator flies the mean of
   a reversing entry at `cos(bank)` of the commanded magnitude; a vehicle
   spending 60% of its ticks slewing through wings-level is making far more
   vertical lift than that, so it stops sinking — which is the level-off
   itself, `vs` pinned at -57 m/s through 6 km of descent. And the
   *phase* of that cycle when the air finally thickens is not a property of
   the entry state at all. Two flights identical to 0.1 m/s of deorbit are
   in opposite halves of it, which is 10d's 26 km.

   The fix is the shape the azimuth band has had all along: scale with range
   to run, floor at the runway's own width, cap far below the 23 km the
   azimuth test alone permitted. Three kilometres of predicted cross-track
   with 250 km left to fly is not worth a reversal — the vehicle has tens of
   degrees of crossrange authority to spend on it — and three kilometres at
   the gate is the entire miss. `CROSS_DEADBAND_PER_KM` /`_MIN_M`/`_MAX_M`.

   **Why the fixed threshold looked right when it was written.** It went in
   to fix the opposite failure: the azimuth band widens with range, so it
   permitted a cross-track proportional to range and the lean held one sign
   while the offset grew to 23 km. An absolute threshold does fix that, and
   the flight it was measured on got better. What nobody looked at was the
   *count* — that the same flight now reversed eighteen times, on an 18 s
   period, with the roll rate-limited most of the way. **A control fix
   measured only at the endpoint cannot tell a converged loop from an
   oscillating one**, and both entries 10 and 10d then spent their effort on
   the solve, which was not the thing moving the control.

11. **The deorbit warped over the pass it was waiting for, and which pass you
   got was luck.** Two instances loaded the *same save file* (same md5) at the
   same UT and flew different entries: one committed at UT 39687 on 51 m/s
   with 1425 km to run, the other at UT 39938 on 105 m/s with 975 km. They
   landed 23 km apart. Nothing about the glide differed; they were never on
   the same trajectory to begin with.

   `set_warp` warps on the absence of a solution and drops to 1x on the tick
   that finds one, which is right. What was wrong was the sentence justifying
   how far it may jump: *"at 10x a 2 s tick is 20 s of orbit, ~44 km of arc
   against a window hundreds of kilometres wide, so no pass the search could
   have acted on is skipped."* Every term in it is an assumption, and each is
   independently false:

   - **`WARP_MAX_FACTOR` is an index, not a rate.** What index 2 means is the
     game's to decide, and `BetterTimeWarp` on one instance made it nine
     times what the comment assumed.
   - **The tick is not 2 s.** A DEORBIT tick runs `deorbit_solution` --
     sixteen entry propagations -- so the more precisely the search resolves
     the burn, the more sparsely it samples the orbit. `DEORBIT_REFINE_PASSES`
     (raised to five by 10a) and the warp step are coupled, and nothing said
     so. **Making one half of a search more precise made the other half
     blind.**
   - **Off 1x the timescale plugin multiplies it again**, and the whole point
     of that plugin is that flights do not run at 1x any more.

   Measured, the product was **180 seconds of orbit per warped tick** against
   the 20 assumed. `logs/LOG736` has no telemetry line at all between UT
   39670 and 39705 -- the window its burn was solvable in, which the other
   instance found at 39687.

   So the step is watched rather than derived, the way `Holdable` watches the
   alpha ceiling: the loop already knows the position at each tick and the one
   before. `WARP_MAX_ARC_M` is the budget (50 km: 44 km is what the factor
   that worked achieved, 390 km is what the one that skipped did, so take the
   side that cannot skip), and the ceiling starts at `WARP_MIN_FACTOR` and
   climbs only on evidence -- a step that is too small costs wall-clock
   seconds, a step that is too large costs the pass, and those are not the
   same price. Confirmed in game: the same instance now commits at UT 39698
   on 52.8 m/s with 1406 km to run, which is the pass the others take.

   **The arc is measured from elapsed time and speed, not from the two
   positions.** `surface_distance` is the short way round (failure 1), so it
   aliases: a tick covering more than half a circumference returns a *small*
   number and the ceiling would rise on exactly the runaway step this exists
   to catch -- and a mod redefining the rate table is what produces such a
   step. Universal time is monotonic and cannot wrap.

   **And warp failing looked exactly like warp working.** `set_warp` wrapped
   the kRPC call in `except Exception: pass`, so an instance that refused to
   warp and one that warped 180 s a tick were indistinguishable from the log,
   while the difference decided which pass was flown. It is logged once now.

   The general lesson is the one 10a already paid for in a different place:
   **a comment that derives a safety margin from three assumed quantities is
   a measurement nobody took.** The check is cheap -- log the quantity the
   margin is about -- and the log line (`warp sampling:`) now carries it on
   every flight, because two flights that sampled the wait differently are
   not replicates and there was no way to tell them apart afterwards.

12. **The airframe goes laterally unstable below 34 km, and nothing here had
   ever measured sideslip.** `aim` builds a *coordinated* command by
   construction -- the nose is placed at alpha above the relative wind inside
   the plane already rolled by the bank, so the wind lies in the plane of
   symmetry and sideslip should be zero -- and it works. Above 35 km the
   vehicle rolls through +-70 degrees, reversal after reversal, holding
   sideslip under one degree:

   ```
   55-40 km   |slip| median 0.5-0.8 deg, max 1.0
   30-25 km   |slip| median 10-12 deg,  max 24.8
    5-0 km    |slip| median 12-16 deg,  max 24.9
   ```

   Lower down it falls apart, and the signature says what it is:

   ```
   32135  bank +70.0  slip -20.5
   31788  bank +70.0  slip +24.8     <- bank pinned, slip swinging +-25 deg
   31469  bank +70.0  slip -11.3
   30974  bank +70.0  slip -10.4
   ```

   The bank command is **not moving** across those rows. That is not an
   uncoordinated roll; it is an undamped lateral-directional oscillation that
   appears once the dynamic pressure builds, aliased by a 2.4 s telemetry
   interval into alternating signs.

   **Why it went unseen for the whole project.** `alpha_actual` is computed
   here as the angle between the nose and the velocity, which is the *total*
   angle and folds the slip inside itself: 20 degrees of alpha with 10 of
   slip reads identically to 22.4 with none. Every log this project has ever
   written could show a vehicle 25 degrees sideways and would report it as a
   slightly high angle of attack. kRPC has `Flight.sideslip_angle` and
   `Flight.angle_of_attack` as plain properties the whole time -- failure 10b
   is the entry about exactly this, and the lesson did not transfer to the
   second quantity.

   **Why it is the best current lead.** Twenty degrees of sideslip puts the
   fuselage broadside to a Mach 5 flow: a large unmodelled drag term arriving
   precisely where the trajectory is decided, and a lateral force that
   wanders the vehicle off the centreline. Those are the two open problems --
   arrives short, drifts sideways -- and one mechanism produces both. It also
   explains why 10e's change failed: the slip is not caused by the reversals,
   so cutting the reversal count could not help it.

   **Resampled at 0.55 s, it is a clean mode.** `logs/LOG765`, 706 samples
   taken while the bank command was *not moving* (so the airframe is on its
   own), 180 zero crossings:

   ```
   period      2.40 s median (2.52 mean, n=165)
   |slip|      median 5.0, p90 17.3, max 28.1 deg
   ```

   with the trace an unmistakable sinusoid at a pinned +70 bank:

   ```
   -19.7  -5.0  +11.8  +16.2  +6.1  -4.5  -15.4  -4.1  +8.4  +10.7  +1.4  -9.0
   ```

   So it is Dutch roll, undamped or nearly so, and it is not aliasing and not
   the reversals.

   **The first place to look is that this project has never tuned the
   attitude controller.** `set_autopilot_attitude` calls
   `set_direction_and_up` and leaves kRPC's `stopping_time`, `time_to_peak`
   and `overshoot` at their defaults, which are derived from the vessel's
   available torque -- and on this airframe the control-surface authority
   varies by orders of magnitude between 50 km and 10 km, which is precisely
   where the mode appears and precisely where a fixed tune would be worst.
   A controller can damp this mode or drive it, and nothing here has ever
   checked which.

   **It is the controller, confirmed, and the cure is worse than the
   disease.** kRPC's attitude PID was sitting at its default
   `time_to_peak = 1.0 s` -- a one-second controller acting through a 2.4 s
   mode. Raising it to 3.0 s, everything else identical:

   | | \|slip\| median | p90 | max | \|aoa cmd-act\| median | p90 |
   |---|---|---|---|---|---|
   | `time_to_peak` 1.0 | 7.3 | 13.5 | 16.7 | 2.8 | 10.9 |
   | `time_to_peak` 3.0 | **2.5** | **3.0** | **3.6** | **0.6** | **5.2** |

   and the *character* changes, not just the amplitude: the sinusoid is
   replaced by a steady 2.5 deg trim offset holding one sign for eighteen
   consecutive samples at a pinned -70 bank. So the oscillation was
   pilot-induced, and **the attitude tracking error that `Holdable` models as
   an aerodynamic ceiling is substantially a tuning artefact** -- "asked for
   20 degrees at Mach 4 it delivers 15-18" is partly a controller that was
   never tuned.

   **And the flight went 84 km long.** A vehicle that achieves the angle of
   attack it is commanded makes the lift the table says it should, and 32 deg
   is *maximum range* on this airframe (1823 km against 1265 at 20). The aim
   bias and the learned ceiling are both calibrated against a vehicle that
   under-delivers, so fixing the delivery leaves the guidance flying a
   trajectory it no longer predicts: `long` at glide entry +25 km against
   -20 km, +119 km by 30 km altitude, and alpha already at its 20 deg floor
   with nothing left to spend.

   That is failure 10's `ALPHA_TRACKING` lesson from the other side. There,
   making the *model* honest about a vehicle that under-delivers walked the
   search into a worse regime; here, making the *vehicle* honest leaves the
   model calibrated for one that no longer exists. **A plant improvement is
   not a system improvement while the calibration downstream of it still
   describes the old plant.**

   `ATTITUDE_TIME_TO_PEAK_S` is therefore 0 by default -- kRPC's tuning
   untouched -- alongside `ALPHA_TRACKING` and `CROSS_DEADBAND_PER_KM` in the
   category of measured-real, not-yet-adoptable. Adopting it means re-fitting
   `DEORBIT_LONG_BIAS_FRACTION` and re-taking the `Holdable` measurement
   against the better-behaved vehicle, which is the work, not a constant.

   **Adopted.** The re-fit is done and it is a straight line. On one entry
   state, three instances per point, every flight on the same deorbit pass
   (dv 50.9-51.3 m/s):

   ```
   DEORBIT_LONG_BIAS_FRACTION   0.27     0.22     0.15
   along-track landing         +68 km   +36 km   +1.4 km
   ```

   about 0.43 of the aim reaching the ground, which is itself the answer to
   the question the constant's own comment asks: the aim is compensating an
   over-prediction that is *most* of what it adds, not a fixed offset. The
   defaults are now `ATTITUDE_TIME_TO_PEAK_S = 3.0` and
   `DEORBIT_LONG_BIAS_FRACTION = 0.15`.

   And what the tuning was actually buying is not ride quality. At the gate:

   ```
                        time_to_peak 1.0     3.0
   speed                   96.7 m/s        63 m/s
   flight path angle      -82 deg         -33 deg
   ```

   At kRPC's default tuning the vehicle does not arrive *at* the gate, it
   arrives above it, nose down, having converted its whole energy surplus
   into speed -- commanded 24.6 degrees of angle of attack in the last 70 m
   it delivered 5.3. The oscillation was the reason the airframe could not
   be pointed.

13. **`planeprobe` and the airframe disagree by a factor of two subsonically,
   and every terminal constant was copied out of `planeprobe`.**

   `planeprobe` aims the airflow and asks
   `simulate_aerodynamic_force_at` what the force would be. Subsonically it
   over-reads the lift by about 1.8x, and the *ratio* is right while the
   magnitude is not -- it reports L/D 1.70 at 30 degrees against 1.6 flown,
   and ClA 84.8 against 45-50. So the numbers derived from its ratios
   survived and the numbers derived from its magnitudes did not: the stall
   speed, the approach speed, the gate's height and the flare's trigger were
   all set for an aircraft that does not exist.

   The airframe's own answer, from `act=ClA/CdA` -- the force kRPC reports,
   resolved about the relative wind -- on every subsonic GLIDE and APPROACH
   line the project has flown, binned on the angle of attack *achieved*:

   ```
   aoa      n     L/D    ClA   1g speed      planeprobe said
    10    186    1.75   15.3    83.7 m/s     L/D 3.39, ClA 28.4
    12    519    2.10   22.9    68.4         L/D 3.26, ClA 36.1
    14   1013    1.52   24.6    66.0         L/D 3.07, ClA 43.6
    16   2882    1.46   26.8    63.2         L/D 2.86, ClA 50.7
    18    669    1.48   24.5    66.1         L/D 2.65, ClA 57.6
   ```

   Restricted to the flights flown with the attitude controller tuned -- a
   vehicle that holds what it is asked, so these are not transients -- the
   same shape reads tighter, quartiles inside 0.13: L/D 2.12 at 10, 2.11 at
   12, 1.91 at 14, 1.49 at 16. **Best glide is L/D 2.1, not 3.39.** The most
   lift the airframe has been measured making subsonically is ClA 45-50
   around 22-26 degrees, which at 6.69 t is a stall near 48 m/s and not 37.1.

   What the error did, in order:

   * `APPROACH_FACTOR * STALL_SPEED_M_S` = 1.8 x 37.1 = 67 m/s is **1.4 times
     the real stall speed**, where holding 1 g needs 16-17 degrees of angle
     of attack -- past best glide, at L/D 1.46. So the approach descended at
     33 degrees instead of 17.
   * The flare then had about 1.5 g available against the 1.7 the arrest
     needed, and touched down at 25 m/s of sink. Every flight of this
     airframe has ended as a wreck reporting 0 parts and 0.00 t, *including*
     the ones that arrived within two kilometres of the runway.
   * The gate at 4 km out and 1400 m up has 2.9 km of glide in hand at L/D
     2.1, against the 4.6 km to the touchdown aim. **The gate was placed
     where the vehicle cannot reach the runway from**, so an approach flown
     perfectly from it still lands 1.7 km short. That is most of the "arrives
     short" at the terminal end, and it was structural rather than a control
     error.

   The propagator's own model is not implicated and that is the thing to
   notice: swept in flight, it reads ClA 23.3 at 12 and 26.2 at 16 against
   the airframe's 22.9 and 26.8, and `act`/`mdl` sit on top of each other in
   every log. Two instruments agreed with each other and with the vehicle's
   behaviour -- it stalls in the flare at 57 m/s, which a 37 m/s stall does
   not do -- and the third, which nothing had checked since the day it was
   written, was wrong. `planeprobe`'s failure mode is not noise: it is a
   clean, nearly constant factor, which is exactly the kind that reads as a
   plausible measurement.

   **The general rule.** `planeprobeGearup.txt` is a file of measurements
   and the constants in `config.py` were transcribed from it by hand, which
   made it a source with no consumer that could contradict it. A measurement
   that only ever feeds constants can be wrong for as long as the project
   lasts. The flown polar is now the authority; `polar.py`'s table can be
   re-taken from the logs at any time and costs no flight at all.


14. **The autopilot could not complete a landing, and had never had the
   chance to prove it.**

   `tick` calls `grounded_early` *before* the phase handler, and
   `grounded_early` fires on any phase but STANDBY, ROLLOUT and STOPPED as
   soon as the game says "landed" below `GROUNDED_SPEED_M_S`. So a touchdown
   on the runway at 65 m/s during FLARE ends the flight with "on the ground
   in FLARE": `run_flare`'s own `touched_down` never gets the tick, ROLLOUT
   never runs, the brakes never come on and the wheels are never steered.

   The guard is right for the phases that have *no* ground test — an entry
   that falls short lands in terrain during GLIDE and the phase machine has
   no opinion about it, which cost a harness hour per flight. It is wrong
   for the two phases whose job is to arrive. ROLLOUT reached 14 times in
   the project's history and STOPPED zero, and the reason was not only that
   the vehicle kept crashing: those 14 are the flights where kRPC's
   `situation` stream happened to lag a tick behind the handler.

   The lesson is the ordering one. A guard that runs before the thing it is
   guarding cannot tell the failure it exists for from the success it
   forbids, and a project whose successes are all in the future will not
   notice which one it is catching.

   APPROACH and FLARE are now exempt while the vessel still has its parts; a
   wreck is still taken here, because ROLLOUT has nothing to do for one.

15. **The glide hands the approach a vehicle pointing the wrong way.**

   With the aim re-fitted and the polar honest, `logs/LOG784` arrived 466 m
   from the runway midpoint — the first flight of this project to reach the
   tarmac's own length — and still broke up, because it was 213 m to the side
   of it on grass. The handover explains both:

   ```
   GLIDE -> APPROACH   gate long=+2136 cross=+834 d=2283
   first APPROACH tick  alt 2514   v 219.2 m/s   heading error +74 deg
   ```

   Position is the only thing the glide solves for, and it nulled it
   reasonably: 2.1 km long, 834 m off. Everything else about the arrival is
   whatever the last bank reversal left behind — **219 m/s against a 77 m/s
   approach speed, and 74 degrees off the runway heading.** The approach then
   spends its whole length flying a turn onto final: `xt` grows from 1071 m
   to 1918 before the turn bites, and comes back only to 708 by the flare.

   Neither number is an accident of that flight. The bank is the range
   control and the range needs all of it, so the vehicle is in a 70 degree
   turn when it crosses the gate altitude whichever way it happens to be
   pointing; and above `SPEED_HOLD_FACTOR` times the approach speed there is
   no speed control at all, so between Mach 1 and 123 m/s the vehicle simply
   accelerates. A gate is a point, and arriving at a point says nothing about
   arriving *on a course*.

   The shuttle's answer to exactly this is the heading alignment cone, and
   the cheaper half of it is available here: make the speed hold two-sided
   (`SPEED_FLOOR_ON`) so the vehicle is not 140 m/s fast, and give the
   cross-track a bank magnitude of its own (`CROSS_BANK_ON`) so it is not
   relying on the range solve to lend it one.

   **Flown. The speed half is most of it and the bank half is nothing.**
   Three flights each, one save, same deorbit pass:

   ```
                       along mean   sd    |across| median   max
   cap only             +11.7 km   6.5        8.8 km       12.7
   + speed floor         +2.2      6.5        3.2           5.8
   + floor and lean      +1.8      6.2        5.1           5.6
   ```

   The floor halves the cross-track and takes 9.5 km off the along-track
   bias; the cross-track bank floor on top of it does nothing measurable
   (5.1 against 3.2 at n=3 is inside the scatter), so it stays off. That the
   *speed* fixes the *cross-track* is the point: at 219 m/s a 35 degree bank
   has a 7 km turn radius and the runway is 5 km away, so the approach
   cannot align however hard it steers. One of the three arrived at the gate
   29 m off the centreline and landed 13 m off it.

   What the floor does **not** touch is the along-track scatter, which is
   unchanged at +-6.5 km. That has a different cause: failure 16.

16. **Mid-reversal, the solve is shown a wings-level vehicle and believes it
   for the rest of the entry.**

   `solve_glide` is handed the bank the vehicle is currently holding and
   propagates it to the gate. A reversal slews between the stops at
   `BANK_RATE_DEG_S`, about seventeen seconds, and for four of them the
   command is near wings level — which on this airframe is a *much* longer
   flight. So four seconds in every reversal, the propagation is of an entry
   nobody is going to fly. Measured in the 42-28 km band, where reversals
   come every twenty seconds, on a flight tracking its command to a tenth of
   a degree:

   ```
   bank  -60.5  -35.3  -14.8   +8.1  +30.5  +51.8  +61.5
   long    -15     +1  +3661 +20049 +11350   -175    +23
   ```

   `long` is inside 50 m at the stops and spikes past **20 km** every time
   the command crosses zero. Usually the spike passes before the solve acts
   on it. Sometimes it does not: in `logs/LOG789` the angle of attack was
   pulled off 32 degrees to the 20 degree floor during one of them, and the
   flight never came back — it held +10 to +22 km with the bank against its
   stop for the rest of the entry and landed 17.7 km long. Five flights of
   one configuration off one save landed **-0.5, -0.9, +4.8, +12.7 and
   +17.7 km**, and the ones that diverged are the ones that diverged here.

   This is the same error the propagator already fixed for the bank's *sign*
   and did not notice it had left the magnitude behind: `acceleration`
   carries no lateral lift at all, deliberately, because "a propagation of
   one constant lean predicts an 80 km curving miss that is an artefact of
   the model rather than anything the vehicle would do". The magnitude is
   the other half of that sentence. `BANK_PREDICT_INTENT` shows the solve
   the lean it intends to hold rather than the transient the rate limiter is
   passing through.

   **Flown, and the naive fix is worse than the artefact.** Two flights
   against three of the control, everything else identical:

   ```
                     along          glide inherited
   control        -1.4  -1.1  +17.3      -7.0 km
   predict intent       +11.1  +39.8    -22.1 km
   ```

   and the second column says why. Mid-reversal the vehicle really *is* near
   wings level and really is sinking less; telling the solve it is at 70
   degrees makes the prediction too *short*, so the glide asks for more
   range, and it lands long. The entry spends a real fraction of its time in
   the transit, and pretending it spends none is the same error with the
   sign flipped. `BANK_PREDICT_INTENT` stays off.

   So the measurement in the table above stands and the fix does not: what
   the propagation wants is the **mean of the reversing entry including its
   duty cycle** -- neither the instantaneous lean nor the intended one, but
   the one that flies the same distance as the vehicle does. That is a
   different and larger piece of work, and it is the open one.

   **How much of the flight this is: a fifth of it.** Counted over three
   entries below 60 km:

   ```
   LOG835   24 reversals, 23% of ticks between the stops
   LOG832   26 reversals, 23%
   LOG830   23 reversals, 21%
   ```

   and the reading splits cleanly on which side of that the tick is:

   ```
                        |long| median, at a bank stop   mid-transit
   LOG830 (landed well)              26 m                  1857 m
   LOG832 (landed well)              54 m                  2429 m
   LOG835 (diverged)               5852 m                 38398 m
   ```

   A stop-to-stop slew takes `2 * BANK_MAX_DEG / BANK_RATE_DEG_S` = 17.5 s,
   and the relay is switching faster than that — the textbook way to make a
   lagged relay chatter, which is the limit cycle failure 10e measured
   without naming the *rate* as the cause. So there are two handles and they
   are different: `SOLVE_HOLD_THROUGH_REVERSAL_DEG` stops the solve acting on
   the spike, and `BANK_REVERSAL_DWELL_S` stops the spike happening so often
   by requiring a lean to outlast its own actuator before the next reversal
   is considered.

   **And the range solve is excellent when it is not being fed this.** The
   first version of the dwell had a bug -- it tested the dwell against the
   command every tick, which gates the *slew* rather than the decision, so a
   reversal got one tick of travel and was refused for the next twenty
   seconds and could never cross zero. The result is an accidental
   experiment worth more than the feature: `logs/LOG838` held **one lean for
   the entire entry**, and

   ```
   alt  55327  bank -47.1  long   -22        alt  38367  bank -36.5  long  -30
   alt  52778  bank -47.3  long    +0        alt  35169  bank -42.8  long  -14
   alt  50334  bank -46.3  long    -0        alt  31012  bank -42.4  long +329
   alt  47975  bank -44.7  long   -15        alt  26598  bank -39.9  long   +3
   alt  45780  bank -42.6  long    +7        alt  22769  bank -30.2  long  -57
   ```

   `long` inside 30 m from 55 km to 23 km, on a vehicle whose along-track
   scatter with reversals is **plus or minus 8 km**. (It finished 93 km off
   the centreline, which is what never reversing costs.) So the range half of
   this guidance is not approximately right, it is right to metres, and
   essentially all of the along-track scatter this project has been chasing
   since failure 10 is the reversal transient corrupting its input.

   The general shape is worth more than any of them: **a control loop that
   propagates its own rate-limited command is propagating its actuator, not
   its plan.** Anywhere an actuator takes a meaningful fraction of the
   prediction horizon to move, the instantaneous command is the wrong input,
   and the error is largest exactly when the vehicle is doing something —
   which is when the answer matters most.

17. **The flare was clamped at 20.5 degrees by a ceiling learned at Mach 5.**

   `logs/LOG803` touched down **on the runway, 13 m off the centreline**, and
   broke up: the flare arrested 40.4 m/s of sink to 12.65. Reading the trace,
   the flare asked for the load it needed and was commanded 20.5 degrees at
   67 m and still 20.5 at 4 m — pinned, for its whole length.

   That is `ratchet_alpha`'s learned ceiling, and the number came from 20 kPa
   of dynamic pressure at Mach 5, where the control surfaces are fighting a
   trim the propagator cannot see. On final at Mach 0.2 and 2 kPa none of it
   applies. `ALPHA_RECOVER_DEG_S` gives the ceiling back at 0.5 deg/s while
   the vehicle is tracking comfortably, and on a flight with any sideslip on
   final it never is, so the ceiling simply stays where the entry left it.

   `alpha_limit_for_speed` already makes this exact argument for itself —
   "APPROACH and FLARE are deliberately outside it, because the flare has to
   be able to ask for maximum lift at a speed this cap would refuse" — and
   the exemption was never extended to the other cap. Two limits, the same
   reason, one of them fixed. **When a rule is written down as a comment on
   one mechanism, nothing carries it to the next one that needs it.**

   `RELEASE_CEILING_ON_FINAL` hands the whole range back on entering
   APPROACH, and says so in the log.

   The related mistake, recorded because it cost a sweep: the slow flare
   pitch-up *looked* like a controller problem, so the attitude controller
   was handed back to kRPC's 1.0 s tune on final. That re-introduced failure
   12 one phase later — the achieved angle of attack oscillating +-8 degrees
   about a smooth command — and was not the cause at all. A symptom that two
   mechanisms can explain is not evidence for either.

18. **Flying the approach fast fixes the flare and breaks the centreline,
   because the lateral gains are not speed-aware.**

   The flare study in `planeprobe` asks for 1.7-1.9 times the stall speed,
   and those are multiples of a stall speed that is wrong by 28% (failure
   13). The real constraint points the other way: the load factor available
   to arrest the sink goes as **V squared** while the sink to arrest goes
   only as V, so a faster approach makes the flare strictly easier, and what
   it costs is runway — of which there is 2.4 km. Horizontal speed is the
   cheap axis. (This was pointed out by the user, who had been landing the
   craft by hand at 100+ m/s while the autopilot was trying to arrive at 67.)

   Measured, going from a 77 m/s approach to 115:

   ```
   approach at the flare   sink at the flare   sink at touchdown
        66.4 m/s                22.9                ballooned, 0 of 23 parts
        78.8                    31.2                -5.5 (stopped on the runway)
       111.4                    49.8                12.2
       116.1                    59.5                 7.7
       115.7                    61.0                 3.7
   ```

   The flare stops being marginal. What breaks instead is the lateral: turn
   rate for a given bank is `g tan(bank) / v`, so the same gains that gave a
   900 m turn radius at 77 m/s give 1.9 km at 115. Handed over at -847,
   -2427 and +5300 m of cross-track, the approach turned through the
   centreline and settled on the **other side** at +616, +644 and +1751 —
   the same overshoot every time, which is a gain and not noise. The lead
   term in the sign test was a bare `12.0` metres per degree, a distance
   that should scale as `v^2`.

   So the lateral law is now a capture rather than two proportional terms:
   the vehicle is asked for the lateral closing rate it can still *arrest*
   in the offset it has left (`v^2 = 2 a s`, with `a` the lateral
   acceleration the bank limit affords) and banks on the error between that
   and the rate it has. Every term is a speed or an acceleration, so it does
   not need re-fitting when the approach speed changes.

   **The rule.** A gain fitted at one operating point is a measurement of
   that operating point. This project has now re-fitted
   `DEORBIT_LONG_BIAS_FRACTION` five times in one session for exactly this
   reason. Where a law can be written in physical quantities instead —
   accelerations, speeds, distances that the geometry supplies — it should
   be, because then the operating point is an input rather than an
   assumption.

19. **Five re-fits of one constant in one session is the constant telling you
   it is not a constant.**

   `DEORBIT_LONG_BIAS_FRACTION` went 0.27 -> 0.22 -> 0.15 -> 0.10 -> 0.13 in
   a single afternoon, and every move was forced by a change somewhere else:
   tuning the attitude controller, correcting `alpha_for_load`, splitting the
   glide's arrival speed from the touchdown speed. None of those are
   properties of the aim. What the aim stands in for is the difference
   between the entry the deorbit search propagates -- a fixed alpha-30,
   bank-30 schedule -- and the entry the glide actually flies, so it moves
   whenever either does. A value for it is a measurement of whatever the rest
   of the configuration happened to be that afternoon.

   `guidance.deorbit_window` replaces it with something the search computes.
   It propagates the **two corners of the solve's own search box** -- the
   shortest entry `solve_glide` is allowed to fly (`SOLVE_ALPHA_MIN_DEG` at
   `BANK_MAX_DEG`) and the longest (`ALPHA_MAX_DEG` at `SOLVE_BANK_MIN_DEG`)
   -- and takes the burn that puts the gate in the *middle* of what is then
   reachable. The corners are not a guess about the airframe: they are
   exactly the span of commands the glide may issue, so the window is the
   authority the glide actually has.

   That deletes the aim, its floor, its tolerance band and the one-sided
   "take the steepest acceptable" rule, and replaces four fitted numbers with
   one computation. It costs one extra propagation per candidate: 2.9 s
   against 1.9 for a whole search, in a phase that has an orbit to spare.

   The burn's stop test still needs an offset to measure against, and
   `deorbit_aim`'s sharing discipline still applies -- so it is read off the
   solution the search took (`deorbit_chosen_aim`), one propagation at commit
   time, rather than recomputed from a formula that might disagree.

   **Flown, and the repeatability is not close.** Three flights each off one
   save, everything else identical:

   ```
                  along mean    sd      |across| median   max
   fitted aim      +19.5 km   24.9         5.20 km      21.45
   window          -19.7       0.9         0.23          0.23
   ```

   The window's *scatter* is 0.9 km where the fitted aim's is 24.9, and its
   cross-track is a fifth of a kilometre where the fitted aim's is five. What
   it has instead is a clean **19.7 km bias**, which is the tractable kind of
   wrong: the long corner of the window assumes `ALPHA_MAX_DEG` and in dense
   air this airframe does not hold it (`Holdable`), so the span is shorter at
   the far end than the propagation believes.

   The correction for that is `DEORBIT_WINDOW_BIAS` -- where to sit in the
   window, as a **share of its own width**. That is the one-sided principle
   the old aim expressed (surplus is spendable, a shortfall is not) stated as
   a fraction of the authority the vehicle has rather than as a distance in
   metres, so it carries to a different airframe, orbit and runway unchanged.
   A dimensionless number is a policy; a distance is a fit.

   **The general rule.** A constant that has to be re-fitted after unrelated
   changes is a missing model wearing a constant's clothes. The question to
   ask of it is not "what value?" but "what quantity is this standing in
   for, and can the program compute that instead?" And when a correction is
   genuinely needed on top, prefer to write it as a fraction of something the
   program already knows over a number in metres or seconds.

20. **The landing constants could not be contradicted, so they were wrong for
   a year.**

   This is failure 13 stated as a process failure rather than a measurement
   one. `planeprobe` wrote a file, a human read four numbers out of it into
   `config.py`, and nothing downstream could ever disagree: the stall speed
   was 28% low, and the only symptom was that the vehicle stalled in the
   flare, which read as a flare problem for the whole life of the project.

   `spaceplane/airframe.py` closes it. The autopilot already sweeps the real
   airframe's coefficients in STANDBY -- and that table *does* agree with
   what the vehicle then does (ClA 23.3 against 22.9 at 12 degrees, 26.2
   against 26.8 at 16) -- so the stall speed and the best glide ratio can be
   read off the aircraft that is about to be flown. Every flight now logs

   ```
   airframe, from its own swept table: stall 55.4 m/s (ClA max 50.2,
   discounted 30%), best glide L/D 2.01 at 12 deg and 68.9 m/s
   airframe DISAGREES: STALL_SPEED_M_S is 48.00 and the swept table says
   55.43 (15% out) -- one of them is not this aircraft
   ```

   It deliberately does **not** adopt the derived numbers. A derived number
   is a hypothesis until the game agrees with it, and silently swapping the
   landing's constants on the strength of an argument is the move this
   project has a rule against. What it does is make the disagreement
   impossible to miss -- which is the whole of what was missing.

   The one honest weakness is stated in the module: the table over-reads the
   top of the lift curve (50 against the 34 the airframe delivers), and
   maximum lift is exactly what a stall speed is. `airframe.MARGIN` is that
   discount, and it is one stated assumption in place of four transcribed
   constants.

21. **The guard whose stated reason was wrong and whose real job was
   essential.**

   `deorbit_window` rejects a candidate whose arrival is more than
   `DEORBIT_MAX_TIME_TO_GO_S` away, on both of its corners. Read literally
   that is a mistake: the constant exists to reject an arrival a revolution
   away, and the long corner is *by construction* the slowest entry the glide
   could choose -- a trajectory nobody is going to fly. And it was visibly
   costing something: the long corner takes 1593 s against the 1500 s limit
   at the burns that would have centred the gate, so the search sat pinned at
   the shallowest burn that happened to pass, landing 19.9 km short, with
   `DEORBIT_WINDOW_BIAS` connected to nothing -- two arms at 0.0 and 0.4
   chose the identical dv to 0.1 m/s.

   Removing it, with the argument above written out in full in the commit,
   made it **much** worse. Free to centre the gate, the search committed
   300 km earlier on 28 m/s of dv, and three flights landed **95, 103 and
   132 km short**.

   What the clock was actually doing was marking the line the project has
   known about since failure 8 and never written down: *do not commit to an
   entry so shallow that the propagator cannot be trusted on it.* The shallow
   end is where the skip lives and where the range over-prediction is worst,
   and on this vehicle a 1500 s stretch bound happens to sit on that line.

   This is boosterland failure 17 arriving in the other project: **a guard
   whose stated reason does not survive inspection may still be the only
   thing enforcing a constraint nobody wrote down.** The fix is not to keep
   it out of superstition -- it is to find out what it is really doing and
   write *that* down, which is now done, both in the code and here.

22. **The vehicle was braking while it was short, and the solve never tried
   its own best glide.**

   Two separate faults, found in the same trace, both costing range in the
   terminal descent and both invisible because the controls *looked* busy.

   **The brake.** `SPEED_FLOOR_ON` exists to stop a *long* vehicle buying
   range by diving (failure 15). Nothing said it should only apply while
   there is surplus to spend, so it applied to short entries too. In
   `logs/LOG867` the last 25 km were flown at a pinned 20 degrees of angle of
   attack -- where this airframe glides at L/D 1.33, against 2.10 at its
   best-glide 12 -- and the predicted miss bled from -6.4 km to -14.1 km over
   exactly that stretch. Gated on the glide's own prediction
   (`SPEED_FLOOR_WHEN_LONG`, hung on `env` for the reason `holdable` is),
   the along-track went from **-19.9 km to -9.5**, and the miss the glide
   *inherits* from -5.3 km to +1.6.

   Turning the floor off entirely is worse than gating it -- -19.6, -24.1,
   -28.4 km against -9.5 -- so this is a gate, not a retraction.

   **The stop it chose.** `_solve_range` samples both ends of the allowed
   alpha range and a Newton step. That is deliberate and it is proof against
   the *hypersonic* range minimum at 20 degrees which `SOLVE_ALPHA_MIN_DEG`
   exists to guard. Subsonically the curve turns the other way: there is an
   interior **maximum** at best glide -- L/D 2.1 at 12 degrees against 1.19
   at 8 and 1.33 at 20 -- so a short entry sampling only the ends picks
   whichever stop is less bad and never tries the one angle that would
   stretch it.

   `logs/LOG880` is the picture: 7 km short at 24 km altitude, angle of
   attack at exactly 20.0 and bank at 1.6 for the whole terminal glide,
   arriving 15.5 km short with neither control moving. It was not saturated.
   It was sitting at a stop it had chosen, twice a tick, all the way down.

   The fix adds best glide as a candidate, and the angle is not a constant:
   `airframe.measure` reads it off the table the vehicle swept for itself, so
   it follows the aircraft (failure 20).

   **The shared lesson.** `SOLVE_ALPHA_MIN_DEG`'s comment says the range
   curve has an interior optimum and the search must not walk into it. That
   is true at Mach 5 and it is true at Mach 0.5 -- *with the sign reversed* --
   and the guard written for one was silently wrong for the other. A
   statement about the shape of a curve is a statement about a regime.

23. **The stretch bound believed a tracking table that was measured on a
   different vehicle -- this one, before it could be pointed.**

   `ALPHA_TRACKING` is the measured fraction of its commanded angle of attack
   this airframe delivers against dynamic pressure. It was taken over 16
   flights and 5467 samples, and it is off by default because the *solve*
   exploits it (failure 10). `deorbit_window` flies its long corner with it
   deliberately on, because a bound is not a search and cannot exploit
   anything (failure 19).

   Which means the bound is only as honest as the table -- and the table was
   measured before `ATTITUDE_TIME_TO_PEAK_S` (failure 12), on a vehicle whose
   angle of attack was a pilot-induced oscillation. Re-taken over 31323
   samples from every flight since:

   ```
   q <= Pa      n     achieved/commanded      the table said
     1000   10221          1.02                   1.00
     2000    4423          1.01                   0.97
     4000    5889          0.89                   0.87
     8000    5279        **0.74**                 0.87
    16000    4963        **0.72**                 0.81
    32000     548          0.72                   --
   ```

   Thin air got slightly *better* with the controller tuned and dense air got
   much worse, which is the shape you would expect once the oscillation stops
   hiding a trim limit behind an averaged overshoot. Believing 0.87 at 8 kPa
   where the vehicle delivers 0.74 makes the stretch bound too long, the
   window too optimistic, and the burn it chooses too steep -- the entry then
   arrives short and cannot recover, because at that dynamic pressure it
   cannot hold the angle that would stretch it.

   `logs/LOG891` shows the moment: the learned ceiling ratchets 30 to 20
   degrees in **nine seconds** as q crosses 7 kPa, with the vehicle achieving
   13-16 against 28 commanded -- 0.54, worse than even the new table's worst
   bin -- and the predicted miss then bleeds from -3.2 km to -13.5 km with
   the angle of attack pinned at 20 and the bank at 1.7 for twenty kilometres
   of descent. Neither control was saturated against a limit it could argue
   with; the airframe simply could not do it.

   **The lesson is about derived quantities, not about this table.** A
   measurement taken from flight data describes the vehicle *as it was
   flown*, and every change to how it is flown silently invalidates it.
   `ALPHA_TRACKING`, `Holdable`, the flown polar in `polar.py` and the
   `DEORBIT_LONG_BIAS_FRACTION` fits are all in this category. The tracking
   table is the one that had a consumer downstream making decisions on it,
   so it is the one that cost something.

24. **The gate was placed where the vehicle could not reach the runway from.
   Again. For the same reason. Four hours later.**

   Failure 13 found the gate sitting at 4 km out and 1400 m up against a
   glide ratio of 3.39 that `planeprobe` had invented, when the airframe
   flies 2.1 -- so an approach flown perfectly from it landed 1.7 km short.
   It was re-placed at 2600 m against the measured 2.05 and the test was
   rewritten to assert the inequality against `APPROACH_BEST_LD` rather than
   a written-down angle, so that re-measuring the polar could not leave the
   gate behind again.

   Then the approach speed went from 67 m/s to 115 (failure 18), and 2.05
   stopped being the number. **The L/D a glider flies is a function of the
   speed it is flown at**, and fast is draggy: measured at the flare entry,
   127.3 m/s with 69.3 m/s of sink is L/D **1.54**, and 120.6 with 67.7 is
   1.50. At 1.54 the 4.2 km from the gate to the touchdown aim needs 2.7 km
   of height and the gate had 2.6.

   The symptom was a flight that did everything else right: `logs/LOG903`
   handed over 626 m short of the gate with 20 m of cross-track, flared, and
   touched down at -6.9 m/s of sink 7 m off the centreline -- 861 m short of
   the threshold.

   The test did not catch it because `APPROACH_BEST_LD` is an input to the
   test, and the input was stale. Asserting a relationship between two
   constants only helps while both describe the same thing; the guard has to
   be against a *measurement*, and the measurement here lives in the flare
   entry of every log.

   **And the obvious fix cost 11 km.** Raising `GATE_ALT_M` from 2600 to
   3200 m closes the inequality, and it is the wrong handle: the gate's
   *radius* is the altitude the deorbit's own propagations stop at, so
   lifting the gate moves the burn. The search committed 35 km earlier on
   1 m/s less dv -- the shallow regime, again -- the reachable window halved
   from 120 km to 56, and five flights landed 15 to 36 km short against the
   4 km the same configuration managed at 2600.

   `GATE_DIST_M` closes the same inequality and touches nothing upstream: the
   gate stays at the altitude the deorbit is solved against and simply sits
   nearer the threshold. 4000 -> 3400 m.

   **The rule.** Failure 13's lesson was "do not transcribe a constant from a
   file nothing can contradict". This is the sharper version: a constant is
   not a property of the airframe, it is a property of the airframe *in a
   regime*, and changing the regime silently invalidates every geometry
   placed against it. The same sentence covers `ALPHA_TRACKING` (failure 23),
   `STALL_SPEED_M_S`, `DEORBIT_LONG_BIAS_FRACTION` and this.

   And a second rule, from the fix rather than the fault: **when a geometric
   inequality fails, check which of its terms something else is also reading
   before choosing which one to move.** Two constants closed this one; one of
   them was shared with the deorbit and one was not.
25. **The angle of attack was frozen for the second half of every entry, by
   a guard whose premise stopped being true at 27 km.**

   `SOLVE_HOLD_THROUGH_REVERSAL_DEG` holds the angle of attack while the
   bank is mid-reversal, and it is right to exist: failure 16 is the
   four-second wings-level lie a transit tells the propagation, and the
   damage there was not the reading but the angle of attack being yanked
   from 32 degrees to its floor in response to it.

   Its test was `abs(bank) < 25`, on a stated argument: `SOLVE_BANK_MIN_DEG`
   is 30, so a commanded magnitude below 25 can only be a reversal in
   progress. That argument is sound hypersonically and false below about
   27 km, where `cross_bank_floor` narrows the band with the range to run
   and the solve asks for a few degrees of lean and *holds* it. Every tick
   of the terminal glide then reads as mid-reversal, and the angle of
   attack — the range solve's primary control — is frozen.

   Measured over the last five default flights, as the share of GLIDE ticks
   with the hold active:

   | log | held | from | alpha over that stretch |
   |---|---|---|---|
   | LOG935 | 83% | 27365 m | 22.0 -> 14.3 |
   | LOG930 | 60% | 27680 m | 20.0 -> 14.5 |
   | LOG929 | 72% | 43024 m | 31.0 -> 14.5 |
   | LOG927 | 57% | 26418 m | 20.0 -> 14.2 |
   | LOG925 | 59% | 28511 m | 24.7 -> 15.3 |

   The alpha trace is **monotone decreasing in every one of them**, which is
   the signature: with the solve's answer discarded, the only thing left
   touching the command is `alpha_limit_for_speed` pulling it down. The
   solve ran, propagated its four candidates, returned an angle, and was
   thrown away, twice a second, for 27 km of descent.

   That is the same stretch failures 22 and 23 both examined, and both
   described the same way — "the predicted miss bled from -3.2 km to -13.5
   km with the angle of attack pinned at 20 and the bank at 1.7 for twenty
   kilometres of descent", "neither control was saturated against a limit it
   could argue with". Both concluded the airframe could not do it. It was
   never asked: the commands in those traces are the *held* value, not a
   choice.

   **What it costs.** The flown polar (`./spaceplane/tools/polar.py`, 935 logs) is
   L/D 2.08-2.12 flat from 8 to 12 degrees and 1.43 at 16, and the flights
   spend 7539 subsonic samples at 16 against 972 at 12. The terminal glide
   is flown at two thirds of the glide ratio the airframe has, in the one
   phase where range is the only thing left to win.

   **The fix is the test, not the guard** — boosterland failure 17's rule,
   and failure 21's one project over. "Is the vehicle between the stops" is
   a question about whether the rate-limited command has *arrived* at the
   lean it is committed to, and the loop already holds both numbers:
   `guidance.bank_in_transit` compares the command against
   `side * abs(wanted)`, so a reversal is a transit for its whole slew and a
   settled lean is not, at any magnitude. The threshold is floored on one
   tick's worth of slew, because a command that cannot be reached this tick
   will still be moving next tick.

   **The rule.** CLAUDE.md already says *a knob that changes nothing may be
   disconnected, not powerless*. This is its mirror image and it is the more
   dangerous one: **a guard that fires more often than its argument allows
   is not conservative, it is in control.** The argument here named a
   threshold (`SOLVE_BANK_MIN_DEG`) that another part of the flight was free
   to go underneath, and nothing connected the two. Where a guard's test is
   justified by a constant it does not own, say so in the comment and assert
   the relationship — or ask the question the guard actually means, which
   usually needs neither.

26. **The learned alpha ceiling could not fall below 20 degrees, because its
   floor was a guidance target.**

   Failure 25 freed the angle of attack from the reversal hold and the
   command still sat at exactly 20.0 for the whole terminal glide -- from
   26 km down through M 0.9, 0.5 and 0.3 to about 3 km, achieving 14.5.
   Both flights of that A/B did it, so the hold was not what was pinning it.

   `ratchet_alpha` lowers the ceiling when the vehicle will not hold what it
   is told, and it computes the new one as
   `max(GLIDE_ALPHA_DEG, ceiling - ALPHA_BACKOFF_DEG)`. `GLIDE_ALPHA_DEG` is
   20 and its comment says what it is: *max L/D, for range*. It is what the
   guidance would **like** to fly. Used as the floor of a plant-limit
   estimator it says the vehicle can always hold 20 degrees, which on this
   airframe above about 7 kPa is false.

   `logs/LOG937`, over fourteen seconds:

   ```
   alpha ceiling -> 28.0 (commanded 21.2, achieving 13.2, q=7451, learned 23.4)
   alpha ceiling -> 26.0 (commanded 21.2, achieving 13.6, q=7672, learned 17.8)
   alpha ceiling -> 24.0 (commanded 21.2, achieving 14.8, q=7889, learned 17.8)
   alpha ceiling -> 22.0 (commanded 21.2, achieving 15.2, q=8103, learned 17.8)
   alpha ceiling -> 20.0 (commanded 21.2, achieving 14.0, q=8317, learned 17.8)
   ```

   and then nothing, for 27 km of descent. The ratchet knew. `Holdable`
   knew -- it is printing 17.8 in the same line. The floor forbade acting on
   either. So the loop commanded 20, the vehicle flew 14, and `solve_glide`
   propagated its candidates at angles the airframe was not going to hold:
   the model's polar at 20 is L/D 1.68 and what the vehicle delivered at the
   14.5 it could actually reach is 1.43. That is *predict the law you fly*
   violated in the one place where the plant limit is known by measurement.

   **Flown.** Floor moved to `ALPHA_CEILING_FLOOR_DEG` (8, the lowest angle
   the subsonic solve may command), with failure 25's fix in place:

   | | gate long | gate cross | touchdown |
   |---|---|---|---|
   | before, `qs_plane` | -11.0 km, -15.2 km | -100 m, -82 m | 11 and 15 km short |
   | after, `qs_plane` | **-0.8 km** | **-16 m** | 120 m short of the threshold |
   | after, `qs_plane_inc` | +4.4 km | +4.4 km | 280 m from the runway midpoint |

   The ceiling now walks 20 -> 18 and keeps going toward what `Holdable`
   learned. Along-track went from tens of kilometres to hundreds of metres
   on the equatorial state; what is left there is the flare (10.3 m/s of
   sink) and, on the inclined state, a 4.4 km cross-track at the gate that
   the approach cannot null.

   **The rule, and it is failure 24's with the terms swapped.** That one
   said a constant is a property of the airframe *in a regime*. This one:
   **a constant is a property of a role, and a target is not a limit.** The
   same number, 20 degrees, is the angle the guidance wants, the hypersonic
   solve's floor (`SOLVE_ALPHA_MIN_DEG`) and -- until now -- the plant
   estimator's floor. Three different claims wearing one value, so moving it
   for any one of them silently moved the other two, and using it for the
   third was simply wrong. Where a constant appears in a second role, give
   the second role its own name even if the value is identical.

27. **The glide spends its surplus with bank right up to the gate, and hands
   the approach a vehicle that cannot turn back.**

   With failures 25 and 26 fixed the along-track came home and the landing
   itself became good -- `logs/LOG950` arrived +1.4 km at the gate and
   touched down at **1.1 m/s of sink** -- and it still finished 347 m off
   the centreline, on the grass, broken up.

   The trace says why in one column. The last 1.2 km of the glide:

   ```
   alt 4002  bank=-70.0  long=+1517  cross=-1503
   alt 3398  bank=-70.0  long=+1608  cross= -835
   alt 2785  bank=-70.0  long=+1430  cross= -605
   GLIDE -> APPROACH gate long=+1422 cross=-578
   ```

   and then the approach, entered 81 degrees off the runway heading:

   ```
   xt= -396 hdg=+81.2 bank=+25.2
   xt=   +7 hdg=+66.8 bank=+40.0
   xt= +680 hdg=+30.5 bank=+40.0
   xt= +910 hdg= -0.0 bank=+40.0     <- the overshoot, at the bank limit
   APPROACH -> FLARE cross=+772
   ```

   **The capture law is not at fault and no gain can fix it.** At 140 m/s and
   `APPROACH_BANK_MAX_DEG` the turn radius is `v^2/(g tan 40)` = 2.4 km, so
   81 degrees of heading takes 3.4 km of arc and the approach is handed 2.6.
   The law did the right thing -- it went to the stop and stayed there -- and
   the geometry still beat it. That is arithmetic about the airframe, not a
   tuning question, and a session could be lost re-tuning
   `APPROACH_CAPTURE_KP` against it.

   **The cause is that bank has two jobs and they collide at the gate.** It
   is how the glide sheds surplus range (the range table runs 1730 km at
   bank 0 to 1432 at 70) and it is the only thing that must be *small* at
   handover. Flying 70 degrees of it to the last moment is optimal for the
   first job and fatal to the second.

   The fix is a taper, not a ban: `guidance.align_bank_cap` holds full
   authority until `GLIDE_ALIGN_RANGE_M` from the gate and then walks the cap
   down to `GLIDE_ALIGN_BANK_DEG`. Fifteen degrees at 140 m/s is still about
   1.1 deg/s of heading change, so the last stretch aligns tens of degrees --
   what it removes is the *rate the vehicle is still rotating at* when the
   approach inherits it. Surplus the taper will not let the glide spend goes
   to the approach's S-turn, which exists for that, and arrivals are long now
   rather than short, so there is surplus to hand over.

   **The rule.** When one actuator serves two objectives and the second one
   only binds at a boundary, the boundary has to be written into the first
   objective's authority -- otherwise the first spends the actuator right up
   to the moment the second needs it. The same shape as the gate's
   one-sidedness: *arrive with the thing you cannot make more of*.

28. **`watch_breakup` reported the breakup and nothing ended the flight.**

   Failure 27's entry (28) is not about guidance at all; it is about the
   farm. `watch_breakup` was added so the log would say when the vehicle
   came apart rather than leaving "0 parts" to a harness running afterwards.
   It says it, and then the flight carries on.

   Only ROLLOUT and STOPPED have a clock (`ROLLOUT_TIMEOUT_S`). An entry that
   burns up leaves GLIDE steering at a gate a thousand kilometres away, on a
   state kRPC keeps returning unchanged forever. `logs/LOG954` and
   `logs/LOG955` both lost all 23 parts at Mach 6 and then logged **700 more
   seconds of identical GLIDE lines** -- twelve minutes of a test instance
   each, in a project whose stated bottleneck is in-game measurement.

   Zero parts needs no threshold and no clock to interpret, and it is the
   *polled* count rather than the skin streams, which can go quiet for other
   reasons. `destroyed_early` ends the flight wherever it happens.

   **The rule.** A detector that only writes to the log is half a check. The
   question to ask of every new one is "and what acts on it?" -- the same
   question failure 13 asks of a measurement whose only consumer is a human
   transcribing it.

29. **Failure 26's fix, applied bare, flew the entry at alpha 8 and went
   90 km long.**

   Failure 26 found the ratchet's floor was `GLIDE_ALPHA_DEG` -- a range
   *target* -- and that this stopped the learned ceiling at 20 degrees on a
   vehicle holding 14. Replacing it with a floor at the solve's own lowest
   subsonic command (8) fixed that and took the gate arrival from -11 km to
   -0.8 km. Then the sweep flew the same configuration again and produced
   +87 km, +89 km and -137 km. `logs/LOG962`:

   ```
   alt 57819  M 7.2  cmd 30.1  achieved 13.2      long=  +2664
   alt 51640  M 6.9  cmd 11.2  achieved  5.8      long=   -354
   alt 20591  M 3.5  cmd 11.6  achieved  9.5      long= +70237
   ```

   At the entry interface the air is thin, the attitude controller has not
   caught up, and a 17-degree tracking error means **nothing about trim**.
   The ratchet read it as a plant limit, walked the ceiling to 11, and the
   vehicle flew the whole hypersonic entry at an angle of attack that makes
   almost no drag. Range against alpha runs 1265 km at 20 to 1823 at 32;
   below the range minimum it climbs again, and at 8 it never decelerates.

   **What the old floor was really doing** was keeping the ratchet out of
   the one regime where a tracking error is not evidence -- exactly
   boosterland failure 17 and this file's failure 21, and the rule CLAUDE.md
   already carries: *a guard whose stated reason does not survive inspection
   may still be the only thing enforcing a constraint nobody wrote down.*
   Failure 26 read the stated reason correctly and removed the guard anyway.

   The fix keeps both facts. `Holdable` answers `None` for a dynamic
   pressure it has no evidence at, so `ratchet_alpha` takes the low floor
   only where there is a measurement and `GLIDE_ALPHA_DEG` where there is
   not -- and never walks below the angle actually measured at that `q`. At
   8 kPa that is 17.8 and the ceiling reaches it, which is failure 26's win;
   at 200 Pa there is no evidence and the ceiling holds at 20, which is what
   this entry needed.

   **The rule, and it is about how to retire a guard.** Finding that a
   guard's justification is wrong licenses *rewriting the justification*,
   not deleting the guard. Before removing one, fly the regime it was
   quietest in -- the damage here was not at the dense end the change was
   reasoned about, but at the thin end nobody looked at.

27. **Three flights per arm, against a scatter nobody had measured, and the
   result reversed itself twice in one session.**

   `GLIDE_RESERVE_M` was measured at n=3 per arm on one save, then n=3 on a
   second save, and both said the same thing:

   ```
                     f1        f2        f3      mean
   off (qs_plane) -20.1 km  -34.5 km  -12.1 km  -22.2
   8 km            -2.0      +0.2      -1.7      -1.2
   ```

   Twelve flights, two entry states, a clean dose-response (8 better than
   12 better than 16 better than off), the arms flown *simultaneously* on
   matched instances so contention could not explain it. It was adopted,
   written into `config.py` with its table, and documented.

   Then nine flights of that same configuration landed at -5.4, -48.1,
   -2.3, -21.1, -4.6, -28.0, -12.2, -43.9 and -5.4 km. Classified properly
   across every flight since `LOG1000`:

   ```
   reserve off    n=22   mean -9.1 km   median -7.2   sd  9.6
   reserve 8 km   n=23   mean -8.9 km   median -2.8   sd 11.2
   ```

   A difference of 0.2 km against a standard error of 3.2. **The change does
   nothing measurable**, and the -1.2 km arm that looked like a seven-fold
   improvement was three draws from a distribution 46 km wide.

   CLAUDE.md already carries the rule -- *one flight per configuration is
   not a measurement; establish the scatter before believing a difference
   smaller than it* -- and none of the usual defences helped. The arms were
   randomised against instance and contention. The dose-response was
   ordered. It held on two entry states. All of that is what n=3 sampling
   from a wide distribution looks like a reasonable fraction of the time,
   and *the ordering of three noisy means is not evidence of a gradient*.

   **What the scatter actually is, and it was cheap to measure**: 22 flights
   of one configuration, mean -9.1 km, sd 9.6. That is the number every
   comparison in this project needs before it starts, it costs one batch,
   and this session ran four batches of tuning before taking it.

   The rule, sharpened: **the first batch of any comparison is n flights of
   one arm**, not one flight of n arms. Until the scatter is known, an arm
   is not a measurement, it is an anecdote with error bars nobody drew.

28. **The config line is a list of *differences*, so changing a default
   makes every log on both sides of the change say the same thing.**

   `Autoland.__init__` writes the fields that differ from `Config`'s
   defaults, which is exactly right for a sweep and silently wrong across a
   code change. `GLIDE_RESERVE_ON` was flipped to `True` mid-session; the
   eighteen flights after that flew *with* the reserve and logged a config
   line identical to the controls flown before it, because there was no
   longer any difference to report.

   The analysis built on those logs put eighteen reserve flights in the
   control group and concluded the opposite of the truth -- twice, in both
   directions, before the misfiling was found.

   `config.defaults_fingerprint` is the fix: an 8-character hash of the
   whole default set, appended to the config line. Two logs with different
   fingerprints are not the same experiment however identical their
   difference lists look, and a batch that straddles a code change now says
   so.

   This is CLAUDE.md's "every log says which configuration flew it" with the
   loophole closed. The convention was doing its job; what it never claimed
   to pin down was the *baseline*, and a baseline that moves is exactly what
   a long session does to it.

30. **"11 km of along-track per m/s at the interface" is a correlation
   between two consequences, and it was read as a plant sensitivity.**

   Seventy-nine full flights off one save put interface specific energy
   against landing miss at correlation 0.88, slope ~11 km per m/s. The
   conclusion drawn from it -- written into `docs/spaceplane/design.md` and into
   CLAUDE.md, and steering a session's worth of work -- was that the burn
   must deliver ±0.1 m/s to land inside a kilometre, and that everything
   measured inside the glide was a 1-2 km knob underneath an 11 km input
   error.

   `savegen.py` turns out to work on an entry-interface save, not just an
   orbital one: same position, same flight path angle, `--prograde N` more
   speed. That makes interface energy a dial with nothing else attached.
   Turned, over 63 flights on two states, the plant answers **0.2 km per m/s
   inside the plateau and 1.2 near the operating point** -- not 11.

   The 79 flights were not wrong, they were *observational*. A smaller burn
   arrives at 58 km faster **and further downrange**, so it has less distance
   left to cover; interface speed is a proxy for the whole entry geometry and
   the regression attributes the geometry's effect to the speed. Both
   variables are consequences of the burn; neither was manipulated.

   **The rule.** A correlation across flights the autopilot chose is not a
   sensitivity, however many flights it has and however tight it is. Before
   sizing a requirement off a slope, ask what *else* moves with the
   regressor -- and if the quantity can be set directly, set it. Here the
   manipulation cost 63 glide flights and about an hour, against a session
   already spent trying to make a burn ten times more accurate than it needs
   to be.

   `ladder.py` is the harness: a set of saves flown many times each across
   the farm, arms assigned to instances up front rather than raced for, and
   the table printed after every round.

31. **Every landing was a nose-over, and twenty flights of sink-rate tuning
   were aimed at the wrong thing.**

   Arrivals ended with 0 of 23 parts whatever the touchdown looked like.
   The sink rate was blamed, and the approach speed was raised to 2.40 times
   the stall to give the flare more margin. Then a flight touched down at
   **1.8 m/s of sink** and still lost every part, which no sink rate
   explains.

   `run_rollout` commanded `ROLLOUT_ALPHA_DEG = 0.0` on its first tick, at
   60-100 m/s, from the nose-high attitude the flare has to end in. That
   drives the nose onto the nose gear at flying speed. The brakes came on in
   the same tick (`BRAKE_SPEED_M_S` is 200, so they are on at every
   touchdown this vehicle makes), which on a braked nose wheel is a moment
   that holds it there.

   The zero was itself a fix: before it, nothing commanded an attitude at
   all and the autopilot held the flare's 15 degrees on the ground, sat on
   its tail and scraped twenty parts off (the note at `ROLLOUT_ALPHA_DEG`
   records it). **Both are the same mistake -- a step -- and an aircraft
   does neither.** It derotates: the main gear takes the vehicle, the nose
   is held off while the elevator still has the authority to hold it, and it
   comes down as the speed decays. `ROLLOUT_HOLD_ALPHA_DEG` and
   `ROLLOUT_DEROTATE_M_S` blend between the two.

   `NOSE_BRAKE_OFF` releases the brakes on whichever gear is furthest
   forward, **found and not configured** -- the frontmost is the one whose
   position has the largest component along the direction the vehicle
   points, and both of those are measured. This craft turns out to be built
   with its nose brake already at zero, which is exactly why it has to be
   found: that is a property of one `.craft` file and the autopilot is not
   written for one craft.

   The rule: **a control law that ends a phase has to hand over a state the
   next phase can fly, and "the wheels are down" is not the end of the
   landing.** Two phases of this project have now been fixed by noticing
   that a quantity was stepped where the vehicle can only ramp.

32. **One latitude for two thresholds, on a runway that is not east-west.**

   `RUNWAY_LAT` was a single number and the note beside it said why that was
   tolerable: "worth 10-20 m at the far threshold on a strip 70 m wide, so
   it will matter once the cross-track is inside that -- and it is not what
   is costing kilometres today". Both halves were right. The second stopped
   being true the moment the cone brought the cross-track inside 250 m.

   Measured on this install, scanning latitude in half-metre steps at each
   threshold's longitude and taking the centre of the raised, flat band:

   ```
   lon -74.724466   centre lat -0.0485761   plateau 69.17 m   width 78.5 m
   lon -74.495283   centre lat -0.0501756   plateau 68.76 m   width 79.0 m
   ```

   The centreline drifts **16.9 m south** over the runway's length, a tilt
   of 0.4 degrees, so the shared -0.0486 was half a runway-width out at the
   east end. Worse, `Runway.along` is taken *from* the two thresholds, so
   sharing a latitude made the modelled centreline exactly east-west -- and
   the extended centreline the approach tracks diverged from the real one by
   29 m at the 4 km gate.

   Two things about the measurement are worth keeping. The raised PQS
   plateau **extends well past the tarmac** (still 79 m wide at -74.729 and
   -74.488), so terrain height locates the centreline and says nothing about
   where the runway stops -- the longitudes must not be taken from it. And
   the answer agrees to about a metre with a ten-year-old forum post giving
   (-0.0485997, -74.724375) and (-0.0502119, -74.489998), which is the sort
   of corroboration worth recording because it also dates the geometry.

33. **`max(floor, fraction * arc)` is a floor under the *fraction*, and it
   silently refuses to aim short.**

   With `DEORBIT_LONG_BIAS_FRACTION` retired to zero, `DEORBIT_LONG_BIAS_M`
   is the whole aim -- and `deorbit_aim` returned `max(bias, 0)`, so a
   configured `-3000` flew as `0`. Four in-game batches were flown believing
   they were aimed three kilometres short of the gate. They were aimed at
   it, and the two configurations are indistinguishable in the log because
   the `config:` line faithfully reports the value that was ignored.

   This is `testInstances/HANDOFF.md`'s "read the numeric value, never the
   label" from the other side: there the label hid the value, here the value
   was read and then thrown away. A clamp that can make two configurations
   fly identically belongs in a test, and now is one.

34. **Three ways to lose the vehicle on a runway it had already reached, and
   all three were the same mistake.**

   With the cone bringing arrivals inside a kilometre, the flight stopped
   failing at range and started failing in the last four hundred metres.
   Three causes, found in order, each visible only once the one before it was
   gone:

   **The flare ballooned.** `APPROACH_FACTOR` was 2.40 -- 115 m/s, 2.4 times
   the stall -- raised at some point to stop the flare stalling. It does,
   by arriving with so much speed that pulling any useful angle of attack
   *climbs*: measured, the flare ended at sink **-7.5 m/s** at 93 m/s with
   15.7 degrees achieved, and the vehicle skipped down the runway on its tail
   shedding parts. The project's own integration in `docs/spaceplane/design.md` already
   said 1.90 arrests with a 54 m/s touchdown; flown, 1.90 gives 37-51 m/s at
   3-5 m/s of sink.

   **The rollout stepped the attitude.** Covered in failure 31.

   **And the vehicle landed crabbed.** `Autopilot.aim` points the nose at
   `alpha` above the *velocity*, which is exactly right for an entry and
   exactly wrong on short final: a vehicle correcting cross-track has its
   velocity pointing at the centreline rather than along it, so aiming at the
   velocity lands it sideways. Measured, 23 degrees of sideslip at the first
   rollout tick and 51 two seconds later, with the gear torn off at 48 m/s on
   a runway it had just landed on -- and the rollout's own `aim` then
   commanded the vehicle to *yaw into its own slip*, because the velocity is
   what it was referenced to.

   `Autopilot.aim_runway` points the nose down the runway instead, below
   `FLARE_ALIGN_ALT_M` and through the whole rollout. Wheels do not care
   where the nose points; they care where it points relative to the direction
   of travel, and the moment they touch, the difference becomes a yaw the
   aircraft cannot absorb.

   **The rule.** Near the ground the reference frame changes. Everything
   above it is flown against the airflow, because the angle of attack is the
   control and the airflow is what it is measured against; everything on and
   just above the tarmac is flown against the *runway*, because that is the
   one thing the wheels are going to agree with. A phase boundary that does
   not change the reference is a phase boundary that lands the entry's
   conventions on a runway.

   With all three: the first intact landing this project has made -- 19 of 23
   parts, 51 m/s, stopped in 7.5 s at 6.84 m/s^2, 43 m off the centreline.
   The three that did not survive that batch were 16, 52 and 24 m off, and
   the strip is 35 m either side: **it survives when it touches the tarmac
   and breaks when it touches the grass beside it**, which is now the whole
   remaining problem and is a cross-track problem, not a landing one.

35. **The cone's whole energy model divided by a number that was 25% low,
   and the same number was also placing the entry's aim.**

   `HAC_LD` is what `(height - GATE_ALT_M) * HAC_LD` multiplies: the path
   the cone believes the height can pay for. It was **1.35**, a figure
   written as "the achieved glide ratio discounted about 20% for tracking"
   from five flights. Both halves are in the logs and `conesum.py` now reads
   them back, over 22 flights:

   ```
   glide ratio flown over the phase   1.86 - 1.90   (sd 0.05)
   path flown / path planned          1.07 - 1.11
   -> HAC_LD                          1.68 - 1.76, median 1.73
   ```

   Wrong *low* the cone plans a shorter circle than the height can pay for,
   so it arrives over the gate high — and, worse, the weave never fires,
   because the surplus `hac` watches is measured against the same wrong
   budget. `logs/LOG1315` planned 15.0 km, flew 17.5, reached the rollout
   **1499 m** above profile with `wv=0.0` on every single tick, and
   overflew the aim by 3.5 km into the sea. Corrected to 1.70 the same
   configuration rolls out **-597 and +213 m** off profile and the weave
   runs at 35-47 degrees for most of the descent.

   **And correcting it moved the landing, through a path nothing in the
   cone mentions.** `Runway.high_gate` — where the *entry* aims — was
   `(HAC_ALT_M - GATE_ALT_M) * HAC_LD` back from the threshold, so the
   correction pushed the aim 3.3 km further out and every arrival with it.
   They are not the same quantity: `HAC_LD` is path per metre of height,
   the gate's is *ground* per metre of height, and the cone spends much of
   its path going round rather than along. Split into `HAC_GATE_LD`, which
   keeps the aim where every arrival to date was flown to. Failure 19's
   shape — one constant standing in for two things — caught before it cost
   a session, and only because the arrivals moved 17 km in one batch.

36. **The S-turn set the bank angle and never once set the track**, which
   is the difference between 41% of extra path and 1.5%.

   `guidance.approach` computed an S-turn magnitude from the surplus height
   and then handed the *direction* to the centreline capture — which points
   at the centreline, by construction. So a vehicle with height to spend
   banked to its 40 degree limit, crossed the centreline, banked 40 the
   other way, and flew a limit cycle: measured on `logs/LOG1315`, twenty
   consecutive ticks at +/-40 degrees of bank with the cross-track never
   outside **+/-30 m** and the track never more than ten degrees off. That
   is `1/cos(10 deg)` — 1.5% — for a manoeuvre the guidance was costing at
   41%, and the vehicle overflew the aim by 2.5 km with 1.7 km of height
   still in hand.

   What an S-turn commands is a *track angle*, `theta = acos(straight path
   / path the height can pay for)` — the cone's weave one phase later and
   the same arithmetic. It is asked for as a lateral **rate** rather than a
   bank so the capture law underneath is untouched, and so that the same
   `sqrt(2 a s)` that stops the vehicle reaching the centreline too fast
   also stops the weave building a cross-track it cannot get back from.
   Flown, the cross-track swings +328, -49, -288 m with the track 33
   degrees off, and `sc=` reads 34-45 — and it reads **0.0 on every flight
   that was not high**, which is the check failure 10a's rule asks for.

   CLAUDE.md's rule about a knob that changes nothing, for the third time
   in this project. The tell was in the log all along: a bank column pinned
   at its limit beside a cross-track column that never moves is not a
   manoeuvre, it is a limit cycle.

37. **The derotation ramp was scheduled off a touchdown speed the vehicle
   stopped making, so the fix for failure 31 never ran once.**

   `ROLLOUT_DEROTATE_M_S` was 55 with a 15 m/s band — written when
   `APPROACH_FACTOR` was 2.40 and the docstring recorded touchdowns at
   "60-100 m/s". `APPROACH_FACTOR` is now 1.90 and the measured touchdowns
   are **41.7, 47.9, 50.5 and 51.2 m/s**, every one of them below the speed
   at which the blend has already reached zero. So every landing since has
   commanded `ROLLOUT_ALPHA_DEG` — zero — on its first tick, out of the
   flare's 16 degrees: precisely the step the ramp exists to prevent,
   delivered by the ramp. On the first tick after touchdown those four
   flights read an *achieved* angle of attack of **74, 87, 92 and 99
   degrees**. That is not a vehicle derotating.

   Two changes. The schedule is now a fraction of `STALL_SPEED_M_S`, which
   is what the touchdown speed actually depends on — the flare ends at
   about the stall by construction — so a re-tuned approach speed cannot
   leave it behind again. And the command *ramps out of the flare's own
   last angle* over `ROLLOUT_RAMP_S`, because arriving at the right
   schedule in one tick is still a step.

   The schedule moved into `guidance.rollout_alpha` in the same change, and
   that is the part worth keeping. `TestTheRolloutDerotatesRatherThanStepping`
   passed throughout: it re-implemented the arithmetic instead of calling
   it, and then asked about 95 m/s — a speed this aircraft does not land
   at. A test that shares the flight's own function and is asked about the
   speeds in the logs would have failed on the day the approach speed
   changed. Failure 23's rule, arriving at the one phase that had already
   been fixed for it.

38. **The landing was never a cross-track problem at the gate; it was a
   cross-track problem in the eight seconds after it.**

   Failure 34 left the project believing the strip's 35 m half-width was the
   whole remaining question. The approach delivers far better than that --
   measured over a dozen flights, `cross` at the APPROACH -> FLARE line is
   **+1, -7, -8, -11, +6, +14, +18, +29 m**. What happens next is that the
   flare flies wings level and the vehicle keeps whatever *lateral rate* it
   was given. `logs/LOG1366` handed over at `cross=+14`, reached the first
   rollout tick at **-72 m** and stopped at -86, both elevons and a wing
   gone in the grass beside a runway it had just landed on.

   The capture was asking for a closure it could arrest *in the offset*
   (`sqrt(2 a s)` -- 9 m/s fourteen metres out) and the flare was
   interrupting the arrest. It now asks for the closure that puts the
   cross-track at zero **when the wheels arrive**: `|cross| / time left`.
   The drift through the flare is then the last of the correction rather
   than a departure from it. That is the open item recorded at
   `APPROACH_CAPTURE_MARGIN` since `logs/LOG912`, and the answer was a
   time rather than a gain.

   **And the time is not the height over the sink rate**, which is the same
   mistake one level down: the flare spends its last two hundred metres
   arresting exactly that sink, so dividing by the approach's rate runs the
   closure about twice as long as it was sized for. `logs/LOG1398` did that
   -- handed over near the centreline, drew 6.8 m/s for it, touched down at
   **+45 m** still going, and came to rest at +80 with both elevons gone.
   The estimate is now written against the flare's own trigger
   (`FLARE_ALT_M + FLARE_LEAD_S * sink`) and an average sink of half, so the
   two laws cannot drift apart.

39. **Every landing touches down at about 1.1 times the stall, and that is
   the one attitude the airframe cannot land in.**

   The flare's arrest demand is `sink^2 / 2 g h` and `h` goes to zero, so
   the last two ticks of every flare ask for maximum lift whatever the
   approach did. Measured achieved angle of attack at touchdown, across
   every landing this project has made and every configuration it has flown:
   **13.9, 15.3, 15.5, 15.6, 16.0, 16.2, 16.3, 17.2, 17.4 degrees.** Nine
   flights, a degree and a half of spread, and the first part lost is
   `Elevon 4` on every single one of them.

   Three things changed together and the part count went 0 of 23 to 21:

   - `FLARE_ARREST_FLOOR_M` stops the demand growing in the last twenty
     metres, where asking for 2.5 g does not arrest anything -- it only
     sets the attitude the wheels arrive in.
   - `FLARE_TOUCHDOWN_ALPHA_DEG` says what attitude to arrive in, tapered
     in over `FLARE_TOUCHDOWN_ALT_M` and **yielding to a sink rate that is
     still there** -- capping a vehicle twenty metres up and coming down at
     30 m/s is not a gentler landing, it is an arrival at 21 (flown, twice).
   - `APPROACH_FACTOR` back up to 2.25, because the load a given angle makes
     goes as the square of the speed and 1.90 leaves none.

   **What does not work is buying the attitude with approach speed alone.**
   2.45 was flown and touches down *slower* than 2.25 -- 51-54 m/s against
   51-58 -- because the extra speed is spent in a longer flare rather than
   carried to the wheels. The touchdown speed is set by how long the flare
   lasts, which is `FLARE_LEAD_S`, and that is where the next attempt on it
   belongs. Failure 34's own measurement said as much and was read the other
   way: 2.40 touched down at 93 m/s and still achieved 15.7 degrees.

   The diagnostic that made all of this visible was three lines in
   `telemetry.py`: the breakup line had always counted the parts and never
   named them, and "10 skin sensors have gone silent" is the same line
   whether the tail scraped or the nose gear folded -- which want opposite
   fixes. It names them now.

40. **Killing the harness does not kill the flight, and the flights that
   result are not malformed — they are plausible.**

   `quickglide.py` runs the autopilot as a subprocess. `pkill -f
   quickglide.py` matches the parent only, so the child goes on flying:
   still connected to that instance, still commanding the vessel, still
   writing its log. Two abandoned batches on one afternoon left **eight**
   autopilots alive across four instances, three of them on the same kRPC
   port, flying one aircraft between them.

   The first symptom is obvious and harmless — `vessel: 0 parts mass
   0.000 t`, telemetry frozen at a fixed altitude. The second is neither:

   - a deorbit arm read **+30, +42 and +61 km** against a baseline of
     +2.7 km with sd 10.7, on the normal pass, with nothing wrong in any
     log. That is a 4-sigma result pointing squarely at the change under
     test. Re-run after the orphans were cleared, the same arm's first
     flights arrived **+19 m and -2012 m**;
   - another logged `APPROACH` at 80 km and Mach 7, which is a phase
     machine that cannot have got there — two autopilots were driving the
     same vessel.

   The arm that produced the first of those was judged harmful on it and
   very nearly written off. **A contaminated measurement here does not look
   like noise; it looks like the thing you just changed.**

   Fixed at the source: the child is started with `start_new_session=True`
   and reaped through the process group in a `finally`, so an interrupted
   harness takes its flight with it. The check before trusting a batch is
   one line — `pgrep -af spaceplane.autopilot` should show exactly one per
   busy instance.

   **And the dates matter.** Everything measured in game between the first
   abandoned batch and the fix is suspect to some degree. What survives is
   the findings that are visible *inside a single flight* — a bank column
   pinned at its limit beside a cross-track that never moves, a derotation
   ramp whose schedule starts above every touchdown speed this aircraft
   makes, a flare whose arrest demand divides by a height going to zero.
   What does not survive unexamined is any of the between-batch arithmetic.
   Entries 35-39 are written from the first kind; their part counts and
   scatter figures are from the second and want re-taking.

41. **The scatter is made by a relay, and speeding the actuator does not
   slow the relay.**

   Measured over fourteen flights, the glide spends **87% of its time with
   the bank off its stops**: 24 reversals in 595 s, at 17.5 s per
   stop-to-stop slew (`BANK_MAX_DEG` 70, `BANK_RATE_DEG_S` 8). The bank
   magnitude is the energy control and it is almost never settled.

   That matters because of a fact already in `config.py` and not previously
   joined to it: **entry range is non-monotone in bank.** Offline, holding
   30 degrees of angle of attack, the range is 1730 km at 0 degrees of bank,
   *up* to 1872 at 30, then down to 1679 at 50 and 1432 at 70 -- which is
   why `SOLVE_BANK_MIN_DEG` floors the solve at 30, to keep it on the
   monotone branch. Every reversal slews the vehicle straight through the
   branch the solve refuses to operate on, twenty-four times a flight, for
   most of the flight.

   It also rescues an observation that looked like failure 30's trap. Mean
   bank above 45 km correlates with the landing miss at **r = +0.97** over
   sixteen flights, which reads as reverse causation -- the guidance banks
   because it predicts long. The offline table says a little bank really
   does lengthen the entry, so the correlation has the sign the physics
   predicts as well as the sign the feedback predicts, and the two cannot be
   separated by watching.

   **`BANK_RATE_DEG_S = 24` was flown and it is not the lever.** The knob is
   connected -- mid-slew falls from 87% to 70% -- but the reversal count
   rises from 24 to 33, because what sets the reversals is the cross-track
   relay, not how long a reversal takes. Five flights: arrival mean +8.8 km
   against a baseline of -1.5, sd 17.6 against 17.0. No change worth having.

   So the quantity to attack is the reversal *count*, and the thing that
   sets it is a cross-track loop the cone has made unnecessary -- see
   `CROSS_DEADBAND_PER_KM`, whose original rejection was paid for entirely
   in cross-track. A first attempt at 2 m/km capped at 3 km moved the count
   only 24 -> 21; the documented 5 km cap moved it to 5.2. The effect is
   strongly non-linear in the cap and that is where the next batch goes.

42. **One glide ratio, measured over three different spans, none of them the
   one it sizes.**

   `APPROACH_BEST_LD` decides where the wheels touch. It sets the altitude
   the cone is allowed to hand over at (`approach_needed`) and the surplus
   the approach's S-turn measures itself against, and the handover altitude
   becomes runway at about two metres per metre. So the span it has to
   describe is fixed by what it is aimed at: **the rollout to first wheel
   contact**, the flare included, because the flare is flat and long.

   It has been measured over everything except that:

   ```
   3.39   planeprobe, transcribed        an aircraft that does not exist (13)
   1.55   at the flare entry, at 127 m/s one regime, one instant
   1.95   over the APPROACH phase        stops at the flare
   2.08   ditto, called "41 flights"     same span, larger n, same error
   2.55   rollout to the wheels          the span the aim ends
   ```

   At 1.85 against a flown 2.55 the handover from a 1500 m rollout is 846 m
   too high, which is 2157 m of ground. Measured landing bias: **+1754 m, sd
   803**, against a runway that accepts +/-1200. The arithmetic closes to
   within the scatter, which is how the span was finally identified.

   **And the two ends cannot be corrected independently.** The cone descends
   on its own profile -- `GATE_ALT_M` plus the circling path at `HAC_LD` --
   and the exit compares that profile against `approach_needed`. Correcting
   only the constant makes the exit unsatisfiable at the rollout: the
   vehicle stays in the cone, falls to the `GATE_ALT_M` floor, and leaves
   "out of height" from wherever it is. Flown at 2.08, **eight of eight**
   exits were out of height, one 2435 m from the gate at 2597 m, and none
   landed. The pair to move is `APPROACH_BEST_LD` **and** `GATE_ALT_M`, and
   the log column that says whether the pair is consistent is the exit
   reason.

   Two rules meet here. Failure 23's -- a quantity measured from flight data
   describes the vehicle *as it was flown* -- explains why 1.55 went stale
   when `APPROACH_FACTOR` moved. The new one is narrower and sharper: **a
   constant that sizes a distance must be measured over the whole distance
   it sizes**, and the way to keep that true is to have the tool that
   measures it end where the requirement ends. `landsum.py` measures from
   the rollout to the wheels for exactly this reason, and prints the bias
   beside it so the two can never drift apart again.

43. **A turn with nothing waiting on it is free; a deorbit burn is waiting
    on this one.** RCS was made demand-driven -- a relay on the pointing
    error, so thrusters fire for turns and not for holds -- and the deorbit
    alignment was taken off it entirely, on the argument that a vacuum flip
    against no aerodynamic moment has minutes to happen in and the reaction
    wheels can have it for nothing. Nine flights of one config later, every
    arrival had moved from the known 40-70 km short to **75-118 km short**.
    Nothing else changed.

    **The mechanism is not the one the phase timings suggest**, and the first
    write-up of this failure got it wrong. The flip takes about 113 s to
    reach retrograde either way -- wheels or thrusters -- so "the flip is
    slower" is false and the earlier claim that it took 114-169 s against 15
    was comparing against a flight that entered DEORBIT 164 s further round
    its orbit. What changes is *how the vehicle arrives at the gate*. On
    wheels it swings through: LOG1617's last three pre-burn ticks read
    `aoa=83.8`, `160.8`, `178.5` -- 77 degrees in two seconds -- so the burn
    lights while the airframe is still slewing hard through
    `DEORBIT_ALIGN_DEG`, and thrust spent mid-swing does not go retrograde.
    With RCS damping it, the same ticks read `170.6`, `174.2`, `179.2`, and
    the burn is lit pointed.

    The signature is in the exit line, not the landing distance:

        pre-fix   DEORBIT -> DRAIN range error -46409 m, -0.97 m/s still owed
        post-fix  DEORBIT -> DRAIN range error     +0 m, +0.00 m/s still owed

    Those are two different *branches*. `+0` is the blind fallback -- the arc
    does not reach the gate altitude yet, so there is no measured error and
    the exit is on dv spent, which is what every historical flight did. The
    pre-fix flights exited on a measured arc that was 35-50 km short, meaning
    the burn had been declared complete while pointing somewhere else.

    Three rules meet here. CLAUDE.md's, that a guard whose stated reason does
    not survive inspection may still enforce a constraint nobody wrote down
    -- the constraint was "the burn is lit pointed", the guard was "RCS is
    on". The rule about reading the phase line rather than the miss: this
    vehicle lands short for its own reasons (failure 10), so 100 km looks
    like a bad day and only `range error` said otherwise. And "measure in
    game": `fakeksp`'s `control.rcs` is a bare bool its slew never reads, so
    the closed-loop test was byte-identical before and after, 58.2986 m both
    ways.

44. **The deorbit counted a model of its burn instead of the burn, and shut
    down a third early -- on every flight this project has flown.**
    `deorbit_burned` accumulated `throttle * accel * dt` and the exit test
    compared *that* against the dv still owed. Measured across two airframes:
    a solve asking 27 m/s delivered 18.3 (2080.6 -> 2062.3), one asking 32
    delivered 19.9 (2080.6 -> 2060.7), and with the fix in place the same
    solve reported `delivered 32.0 m/s (model said 42.6)` -- a 33%
    over-count, every time.

    **It hid behind failure 10.** The airframe could not hold its commanded
    angle of attack, so it sank faster than the propagator predicted and
    landed 40-70 km short; the underburn pushed the other way by a
    comparable amount. Two large errors of opposite sign, and sessions of
    tuning went into their difference. A new airframe that *holds* its
    command removed one of them and left this one bare: nine flights,
    190-310 km **long**, the glide predicting +237 km on its first tick with
    no authority to take it back.

    The fix is the project's own rule -- the speed drop is the measurement,
    and over a few seconds of retrograde burn in a near-circular orbit
    gravity moves the speed by a hair while the engine moves it by metres per
    second. Only decreases count: a tick that reads faster is the arc, not
    the engine. The modelled figure is kept beside it in the exit line so the
    two can never silently diverge again.

45. **A throttle floor is a minimum *spend*, and the last tick of a deorbit
    is worth kilometres.** `DEORBIT_MIN_THROTTLE` is 0.02, which sounds
    small and is not: `8.6 m/s^2 x 0.02 x 2 s` is 0.34 m/s, and at ~25 km of
    range per m/s that final tick moves the landing 8 km. All nine flights of
    one batch exited past their aim -- `range error -13674 m, -0.39 m/s still
    owed` -- and the exit line had been saying so all along.

    The fix is not a smaller floor. When what is owed is less than one
    floored tick would spend, *stopping is nearer the aim than burning*, so
    the burn compares the two and cuts. Arrival scatter over nine flights
    went from sd 1592 m to **sd 565 m**, and the burn's own exit error from
    -10543 m to -1246 m.


46. **The angle-of-attack solve had a search bracket one point wide for the
   whole part of the entry that decides the landing -- and this is the third
   mechanism to freeze that command.**

   Failure 25 freed the command from the reversal hold. Failure 26 let the
   learned ceiling fall below 20 degrees by moving its floor off
   `GLIDE_ALPHA_DEG` to `ALPHA_CEILING_FLOOR_DEG` (8). **Fixing 26 created
   this one.** `solve_glide` clamps its search with

   ```
   top   = min(ALPHA_MAX_DEG, alpha_ceiling)      # what the airframe holds
   floor = min(alpha_floor(...), top)             # SOLVE_ALPHA_MIN_DEG = 20
   ```

   so the moment the ceiling ratchets under 20 -- which it now may, all the
   way to 8 -- `floor` collapses onto `top` and the bracket is a single
   point. The solve runs twice a second, propagates, and can return only the
   angle it was handed. `verified` then finds no improvement and holds, which
   looks like a decision and is not.

   `logs/LOG1877`: **thirty consecutive ticks at alpha 18.0 and bank +2.3**,
   byte-identical, from 23.5 km to 15.5 km, while the predicted arrival walks
   from -0.7 km to -4.1 km at L/D 0.75. `logs/LOG1893` does it at 19.0 from
   22.6 km to 14.7 km while `long` runs -6.7 km to -9.7 km. Every flight of
   both batches does it.

   **What it costs is the whole landing bias.** Over the eighteen flights of
   the configuration that lands (`logs/LOG1860-1877`), touchdown is
   `+118 m + 0.324 * arrival` with a residual of **258 m** -- so the miss is
   inherited from the arrival and an arrival centred on zero lands on the
   runway with nothing else changed. The arrival bias is not made over the
   2000 km entry as `docs/spaceplane/design.md` assumed: `long` holds within tens of
   metres down to 26 km and loses 5.4 km below it.

   **The obvious fix is nearly worthless, and that is the useful part.**
   `SOLVE_MAX_RANGE_ON` brackets the whole permitted span when the vehicle is
   short and no candidate improves, and flies whatever arc the propagator
   says goes furthest -- a law rather than a number, since the best-glide
   angle comes from the swept table at the current Mach. Flown, fifteen
   flights, three arms rotated across four instances: control -7.1 km sd 2.7,
   max-range -5.9 km sd 4.0. **0.6 standard errors.**

   `logs/LOG1891` says why: `slv=max` engages and then commands a *constant*
   18.8 degrees from 22.6 km to 15.7 km, because the bracket's optimum is at
   its **top**, against the ceiling. The vehicle wants *more* angle of attack
   to stretch and the airframe will not hold it. Opening the bracket downward
   offers nothing it wants; the frozen command was already at the constrained
   optimum. It is kept, defaulted off, for the bank it does buy (being short
   now spends no lean it was not asked for) and for a vehicle whose ceiling
   is not the binding constraint.

   **The rule.** A bracket that has collapsed and a solve that has converged
   are indistinguishable in every column a log carries -- both show a command
   that stops moving. `slv=` now says which law flew the tick, but the deeper
   point is the one failure 25 already made and this repeats: *where a guard
   or a clamp is justified by a constant it does not own, assert the
   relationship.* `SOLVE_ALPHA_MIN_DEG` and `ALPHA_CEILING_FLOOR_DEG` are
   free to cross and nothing connected them. And the second rule, paid for
   twice now: **fixing a freeze is not the same as fixing the range.** Three
   mechanisms have frozen this command and the range was lost anyway, because
   by 22 km it is already gone. The lever is upstream, where the solve still
   has authority -- see `HOLDABLE_EXTRAPOLATE`.

47. **The landing configuration was fitted against an arrival five kilometres
   short, so centring the arrival broke it -- and the cone cannot absorb what
   is now left over, because its budget is in height and the surplus is in
   speed.**

   Fixing the along-track (`GLIDE_RESERVE_M`, failure 46's aftermath) took the
   touchdown from -3.2 km sd 1.3 to **-0.4 km sd 0.5, 5 of 6 inside the
   +/-1200 m window**. The binding constraint immediately moved to
   cross-track, which went from +5.4 m mean without the reserve to
   **-25.3 m sd 17.0** with it, against a strip that is +/-35 m.

   The cross-track is not a lateral problem. Over the reserve flights:

   ```
   corr(cross-track at flare entry, final cross-track)  -0.99
   corr(flare entry speed, |final cross-track|)         +0.88
   corr(arrival, flare entry speed)                     +0.77
   ```

   The flights split cleanly in two and nothing in between:

   | arrival | flare entry | final across |
   |---|---|---|
   | -4931, -4809, -4092 | 99-133 m, 68-90 m/s | -2, -4, -5 |
   | -1585 to -3636 | ~152 m, 100-104 m/s, 42 m/s sink | -29 to -44 |

   So **the vehicle only arrives with the right energy when it arrives
   4.5 km short.** Every constant downstream -- `HAC_LD`,
   `APPROACH_BEST_LD`, `GATE_ALT_M`, `TOUCHDOWN_AIM_M` -- was fitted while
   the entry was delivering a 5 km shortfall, and they were absorbing it.
   Take the shortfall away and they are wrong by exactly what they were
   absorbing. CLAUDE.md's rule, collected: *a quantity measured from flight
   data describes the vehicle as it was flown, and every change to how it is
   flown silently invalidates it.* This is failure 23's shape at the scale of
   a whole phase chain rather than one table.

   The lateral miss is then a consequence: a flare entered at 103 m/s with
   42 m/s of sink and 50-67 m of cross-track has no time to finish the
   lateral capture, and the flare flies wings level, so the vehicle crosses
   the centreline and lands on the far side of it -- which is what a
   correlation of -0.99 between entry offset and final offset means. Failures
   31 and 34 are the same mechanism; what is new is that the *energy* now
   causes it rather than a gain.

   **Why the cone does not spend it.** `guidance.hac` budgets in **height**:
   `needed = GATE_ALT_M + path / HAC_LD`, and the exit tests height against
   what the approach needs. A vehicle arriving at the right height and 35 m/s
   too fast reads as *on profile*. Every one of these flights flew `laps=0`
   and rolled out. The cone was built to absorb an arbitrary arrival and it
   does -- in one of the two variables that matter.

   **The fix is the budget, not another constant.** The quantity the cone
   should be spending is specific energy, `h + v^2 / 2g`, which is the same
   arithmetic it already does with one more term, is what a heading alignment
   cone manages on the vehicle it was borrowed from, and needs nothing
   measured about this airframe. `HAC_EXIT_SURPLUS_M` was swept 500 against
   250 first (`logs/LOG1929-1940`) on the theory that the handover height was
   the cause: along-track identical at -0.4 km in both arms and cross-track
   scattered -2 to -37 in both. It is not the height.

48. **The flare has a speed window, the chain has no way to hit it, and the
   obvious knob cannot be moved because it sets two things.**

   With the along-track centred (failure 47), every remaining miss is the
   speed at `APPROACH -> FLARE`. Nine flights of one configuration, ordered
   by it:

   | flare entry | cross-track | outcome |
   |---|---|---|
   | 67.1 m/s | +6 | destroyed |
   | 83.2 | -3 | intact |
   | 86.0 | -4 | intact |
   | 91.5 | -5 | intact |
   | 94.7 | -13 | destroyed |
   | 99.2 | -22 | destroyed |
   | 99.4 | -26 | destroyed |
   | 99.6 | -32 | intact |
   | 101.9 | -34 | intact |

   **83-91 m/s lands it on the centreline with every part attached.** Above
   94 the lateral miss grows monotonically with speed, because a flare
   entered fast has no time to finish the lateral capture and then flies
   wings level through what is left (`corr(cross at flare entry, final
   cross) = -0.99`). Below about 82 -- 1.7 times the stall, which
   `APPROACH_FACTOR`'s own comment records as where the flare stops
   completing -- it stalls.

   `APPROACH_FACTOR` commands 2.25 x stall = **108 m/s** and
   `APPROACH_SPEED_FLOOR_FACTOR` refuses below 2.0 x = **96**. Both are
   outside the window. Every good landing this project has made happened
   where the vehicle was too energy-starved to obey its own command -- which
   is why 2.25 survived being tuned: it was never delivered.

   **Lowering it does not work, and the reason is structural.** Flown at
   1.85 with the floor at 1.7, three flights of three: **-3.9, -4.0, -3.9 km
   and flare entry 56.6, 56.7, 56.6 m/s, all three destroyed.** Remarkably
   repeatable and 25 m/s under the stall floor.

   `HAC_SPEED_FACTOR` is *deliberately* a multiple of the approach speed --
   see its comment, so the cone cannot roll out below the floor of the phase
   it hands to -- so cutting `APPROACH_FACTOR` slows the **cone** by the same
   fraction, and the whole chain arrives starved: the cone left "out of
   height" on every one of those flights where the control arm "rolled out".
   And a glider that is slow cannot get the speed back; the speed floor is a
   guard (`alpha = min(alpha, trim)`), not a controller.

   **What this actually asks for.** The flare needs a speed *at one point*,
   not a speed held throughout, and the approach currently controls the only
   thing it can -- a constant. The general form is a speed *profile*: command
   the speed that arrives at the flare at the airframe's own flare-entry
   requirement (about 1.75 x stall, and the stall is measured), letting it be
   higher earlier where the energy is needed for range. That decouples the
   flare's requirement from the cone's without letting the two drift apart,
   which is the property `HAC_SPEED_FACTOR` exists to protect.

   **The rule.** Two of this session's four failed fixes failed the same way:
   `HAC_ENERGY_BUDGET` shed energy with no target, and this commanded a
   target with no authority to reach it. *A quantity that matters at one
   point in the flight needs something that holds it at that point* -- not an
   earlier phase spending in its general direction, and not a constant
   applied everywhere.

49. **Controlling the flare's entry speed controls the cross-track -- and a
   glider can only hold a speed it is able to fall to.**

   Failure 48 established the window (83-91 m/s lands on the centreline) and
   that cutting `APPROACH_FACTOR` starves the cone. `APPROACH_SPEED_PROFILE`
   schedules the commanded speed instead: `APPROACH_FACTOR` at the gate,
   falling to `APPROACH_FLARE_FACTOR` at the height the flare actually
   triggers (the same expression the flare's own trigger uses), with the
   speed floor tracking down to the measured stall edge. `APPROACH_FACTOR`
   is untouched, so `HAC_SPEED_FACTOR` and the cone do not move.

   **It works, and the lateral miss is a consequence of the speed exactly as
   the correlation said.** Six flights per arm, rotated across three
   instances:

   | | flare entry | cross-track | on runway & intact |
   |---|---|---|---|
   | constant 108 | 87.2 sd 14.0 | -12.2 sd 15.7 | 4 of 5 |
   | schedule to 1.75 x stall | 70.7 **sd 5.3** | **-1.0 sd 6.6** | 1 of 5 |
   | schedule to 2.05 x stall | 86.4 sd 11.4 | -18.4 sd 7.2 | 4 of 6 |

   At 1.75 the cross-track is **-1.0 m with sd 6.6** against a 35 m
   half-width -- the lateral problem solved outright -- and two of five were
   destroyed, entering the flare at 65 m/s.

   **Why the target cannot simply be re-centred.** A glider sheds speed at
   will and cannot make it. A target *below* the minimum arrival energy is
   therefore always reachable and is held tightly (1.75: sd 5.3); a target
   above it is often unreachable and the control evaporates (2.05: sd 11.4,
   back to the constant's scatter). The band that is both controllable and
   survivable is narrow, and the ramp as written reaches its target *at* the
   trigger height while still decelerating, so it overshoots downward into
   the stall.

   **What it wants.** The schedule should reach its target *above* the
   trigger and hold it, so the vehicle arrives at the flare on speed rather
   than still slowing through it -- and the target itself should be the
   airframe's own flare requirement rather than a factor chosen here. The
   vehicle can measure that: the flare either completes or it does not, and
   the log already records the entry speed beside the part count.

   **And the 2.05 result did not replicate, which is the other lesson.**
   The first six flights per arm read 4 of 6 on the runway against the
   constant's 2 of 6 and were reported as the best configuration measured.
   A second identical six, pooled, wiped it out: **4 of 11 against 4 of 11,
   exactly level**, with the along-track scatter three times worse (sd 1270
   against 297). CLAUDE.md says three flights are not a measurement and
   neither are six against this scatter; here is that happening to a result
   that had already been written down.

   **The rule, and it is this session's third instance of it.**
   `HAC_ENERGY_BUDGET` spent energy with no target; cutting `APPROACH_FACTOR`
   set a target with no authority; this holds a target only where the plant
   can reach it. *Before changing what a phase spends, check that something
   holds the quantity at the point it matters, and that the plant can move it
   in both directions.*

50. **`GLIDE_RESERVE_M` is not a property of the airframe. It is a property of
   one deorbit geometry, and nothing in the session that fitted it could have
   noticed.**

   Every flight of the session that produced it flew `qs_plane` -- about two
   hundred of them. CLAUDE.md says in as many words that *one quicksave is one
   separation state and a number tuned against it proves nothing about the
   next*, and the rule was broken for the whole session before anyone checked.

   Flown across three entry states, same configuration, four flights each,
   rotated across four instances:

   | entry state | along | sd | destroyed |
   |---|---|---|---|
   | `qs_plane` | **-1.1 km** | 0.4 | 2 of 4 |
   | `qs_plane_high` | **-17.3 km** | 1.4 | 4 of 4 |
   | `qs_plane_inc` | **-56.0 km** | 6.0 | 4 of 4 |

   The miss is at the *arrival*, not the landing: `qs_plane_inc` reaches the
   cone at `long=-45320 cross=-4355` and runs out of sky inside it. A four
   kilometre reserve is not within an order of magnitude of what that entry
   needs.

   **And each state is internally consistent** -- `qs_plane_inc` gives -50.7,
   -50.9, -62.1, -60.3 against a spread of 0.4 km on `qs_plane`. So the
   deficit is a systematic, repeatable function of the entry geometry, not
   scatter. That is the important half: the propagator's optimism is
   *deterministic given the entry*, and varies by more than an order of
   magnitude across entries.

   **What that means for the fix.** A constant cannot express a quantity that
   moves 4 km to 50 km with the deorbit, and no amount of re-fitting will
   make one do it -- which is CLAUDE.md's "a constant that has to be
   re-fitted is a missing model wearing a constant's clothes", with the
   missing model now measured rather than suspected. The reserve has to be
   computed per flight, and the place it comes from is named at
   `HOLDABLE_EXTRAPOLATE`: `Holdable.limit` returns `None` -- *no limit* --
   for any dynamic pressure the vehicle has not reached, so the propagator
   predicts this airframe holding angles it cannot, by an amount that depends
   on how much of the entry is spent in air it has not yet met. A steeper or
   longer entry meets more of it. That is exactly the shape of the table
   above.

   **The mechanism generalises; only the magnitude does not.** Crossed with
   reserve on/off on both failing states, three flights per cell:

   | state | reserve on | reserve off | gain |
   |---|---|---|---|
   | `qs_plane` | -0.4 km | -3.2 km | +2.8 km |
   | `qs_plane_high` | -12.2 sd 6.0 | -18.6 sd 2.1 | +6.4 km |
   | `qs_plane_inc` | -62.0 sd 3.2 | -68.7 sd 3.2 | +6.7 km |

   Aiming long helps on every entry state, and a 4 km reserve buys 6-7 km --
   more than one for one, because aiming long earlier changes the arc the
   solve flies rather than only adding a bias at the end. So the design is
   right and `GLIDE_RESERVE_ON` is worth defaulting on; what cannot be
   defaulted is `GLIDE_RESERVE_M`, because the amount *needed* runs from
   3 km to more than 60 across entries.

   **The rule.** A fitted constant is a fine instrument and a poor product,
   and the way to tell which you have is to vary the thing it was fitted
   against. Doing that costs one batch. Not doing it cost this session the
   right to call any of its numbers a result -- and doing it turned a number
   that looked like a tuning into a measurement of the model that is
   missing.

51. **The learned alpha ceiling is a real measurement in dense air and a
   record of its own commands in thin air -- which closes the whole family of
   in-flight estimators the previous four failures pointed at.**

   Failures 47 and 50 both ended by naming the same fix: `Holdable.limit`
   returns `None` -- *no limit* -- for any dynamic pressure the vehicle has
   not reached, so the propagator assumes the airframe holds whatever it is
   commanded in air it has never met, and the remedy is a pessimistic prior.
   Two were built. Both are refuted, and the second refutes the family.

   **The prior that cancels its unknowns.** The angle is lost where
   `q * Cn(alpha) * arm = T`, so one observed saturation fixes the product
   and the ceiling everywhere denser follows from the swept table, with the
   moment arm and the torque dividing out -- no constant about the vehicle
   anywhere. It is wrong because `T` is not constant: this airframe holds
   attitude partly with *elevons*, whose torque also goes as `q`. Fitted
   against the curve the vehicle learned:

   ```
   Cn = 25369 * (1/q) + 5.44        R^2 = 0.892
   ```

   A constant-torque law needs that intercept to be zero. Anchored on one
   sample the prior predicts 12.5, 9.1 and 8.0 degrees where the vehicle held
   26.4, 23.2 and 21.0. The true law has two terms and needs two anchors at
   different `q` -- more evidence, later, which is the problem it was built to
   solve.

   **And the curve both priors would learn from is half fiction.**
   `Holdable` records a ceiling only where the vehicle was commanded past
   one. Where the solve asked for less than the airframe could hold, the bin
   records the *command*. Over 98 flights of one airframe and one save:

   | q (Pa) | mean | sd | range |
   |---|---|---|---|
   | 2154 | 23.4 | **5.7** | 17.3 - 32.0 |
   | 3162 | 24.0 | **4.5** | 14.6 - 29.0 |
   | 4642 | 23.0 | 1.1 | |
   | 6813 | 20.2 | 0.7 | |
   | 10000 | 14.7 | **0.4** | |

   Rock solid exactly where the vehicle is genuinely pinned -- below about
   28 km -- and noise everywhere above it. `LOG1877` and `LOG1905`, same
   configuration and same save, disagree by **12 degrees** in the 3162 Pa
   bin. That is failure 13's shape again, and it is why
   `HOLDABLE_EXTRAPOLATE` measured inert *and* destroyed five of six: it fits
   a trend through the half of the curve that is noise.

   **The conclusion, which is the useful one.** The vehicle cannot learn its
   ceiling until it is deep enough to be pinned against it, and by then the
   range is already gone. *No estimator over entry telemetry can be honest in
   time*, because the information is not there yet -- so stop building them.

   The ceiling is a two-parameter property of the airframe (`A/q + B`, wheels
   and elevons) and it has to be **probed**, not picked up incidentally. This
   project already knows how: `BROADSIDE_PROBE_DEG` flies exactly that sweep
   and `logs/LOG1615` and `LOG1616` already contain exactly this curve. Two
   probe flights per vehicle fix `A` and `B`; after that the propagator is
   honest from the first tick of every entry and `GLIDE_RESERVE_M` retires.
   A probe is general -- any vehicle can fly one, and it measures rather than
   transcribes, which is the standard `planeprobe` and the swept aero table
   are already held to.

52. **The propagator's optimism is load-bearing: the deorbit was calibrated
   against it, so making the glide honest moves the burn and costs more than
   it buys. That is why all three attempts to fix the ceiling failed.**

   Failure 51 concluded that the ceiling has to be *probed* rather than
   learned in flight, and `logs/LOG1615` already held the probe:
   `BROADSIDE_PROBE_DEG=90` commands 90 degrees all the way down, so every
   bin in it is a real saturation instead of a record of the command.

   **The probe curve is good.** Against 98 landing flights, in the three bins
   where a landing genuinely pins the vehicle and can measure the ceiling
   itself:

   | q (Pa) | probe | 98 landings | landing sd |
   |---|---|---|---|
   | 4642 | 23.5 | 23.0 | 1.1 |
   | 6813 | 20.0 | 20.2 | 0.7 |
   | 10000 | 14.8 | 14.7 | 0.4 |

   **0.5, 0.2 and 0.1 degrees apart.** It is a measurement with independent
   confirmation, and above those bins the landings carry 4.5-5.7 degrees of
   scatter so there is nothing there to disagree with.

   **And feeding it to the propagator makes the landing worse.** Twelve
   flights, probe against control on two entry states, rotated across three
   instances, no reserve anywhere:

   | | probe on | probe off | cost |
   |---|---|---|---|
   | `qs_plane` | -17.8 sd 1.1 | -3.5 sd 1.2 | **14.3 km worse** |
   | `qs_plane_inc` | -88.8 sd 10.1 | -61.3 sd 9.8 | **27.5 km worse** |

   **The mechanism is the deorbit, and it is visible in the burn.** The glide
   propagator is the same one `deorbit_solution` searches with. Told the
   truth about the ceiling it predicts less glide range, and the burn search
   answers with more delta-v -- 33.3 and 33.5 m/s against 32.7 and 32.3 --
   which is a steeper entry and therefore a *shorter* one. The docs already
   record that burns identical to 0.1 m/s arrive 5 km apart; a full 1 m/s
   accounts for the whole 14 km.

   So the deorbit has been tuned, over this project's history, against an
   optimistic glide model. The optimism is not a defect sitting on top of a
   correct system; it is a term the burn search has absorbed. Correcting it
   in one place is not a fix, it is a perturbation.

   **The rule, and it is the one that explains failures 47, 50 and 51 as
   well.** *An error that everything downstream has been fitted against is
   load-bearing, and removing it is a change to everything that was fitted.*
   The same shape cost this session the cross-track when the along-track was
   centred (47). Here it runs *upstream*: the entry's model error is
   structural to the burn that sets up the entry.

   **What that leaves.** An honest propagator is still the right destination,
   but it cannot be dropped in. It has to arrive together with a deorbit that
   does not depend on the glide's aero model being wrong -- either re-derived
   against the honest model, or decoupled from it. Until then
   `HOLDABLE_PROBE_ON`, `HOLDABLE_PRIOR` and `HOLDABLE_EXTRAPOLATE` all stay
   off, and the reason is this entry rather than any doubt about the curve.

53. **The probe ceiling is pessimistic exactly where it binds, and failure
    52 read the mechanism backwards.** Offline, on the logs already flown --
    no new flights.

    Failure 52 recorded the fact (probe on costs 14 km on `qs_plane`, 27 km
    on `qs_plane_inc`) and then explained it: *"told the truth about the
    ceiling it predicts less glide range, and the burn search answers with
    more delta-v -- which is a steeper entry and therefore a shorter one."*
    The delta-v direction is right. The range direction is **wrong**, and so
    the conclusion drawn from it was wrong.

    **Capping alpha lengthens the predicted entry, it does not shorten it.**
    The real `deorbit_solution` and the real `Holdable`, run offline against
    four states, fixed delta-v, swept over `ENTRY_ALPHA_DEG`:

    | entry alpha | probe off | probe on | delta |
    |---|---|---|---|
    | 18 | 1887 km | 1892 km | +5 |
    | 20 | 1874 | 1882 | +8 |
    | **22 (flown)** | **1821** | **1844** | **+23** |
    | 26 | 1709 | 1766 | +57 |
    | 32 | 1412 | 1677 | +265 |

    Same sign and the same shape on all four states. The reason is that at
    entry angles of attack the dominant term is **drag, not lift**: capping
    alpha takes braking away, so the arc carries further. The burn search
    then asks for more delta-v to pull it back in -- +2.8 to +7.2 m/s across
    five states, which is the direction 52 measured in game.

    So the optimism is not "a term the burn search has absorbed". The
    uncapped propagator predicts a *shorter* entry than the capped vehicle
    really flies, which biases the arrival **long**; the flights land
    **short**; therefore the ceiling was *cancelling* a larger error of the
    opposite sign, and turning it on removed the cancellation. That is why
    probe-on is worse, and it is a different fault with a different fix.

    **And the curve is mildly pessimistic where it binds -- but the first
    way I measured that was confounded, so both numbers go in.** 89 landing
    logs, every tick short of its command by more than
    `HOLDABLE_SATURATED_DEG`: 6383 saturated samples.

    Binned the way `Holdable` bins (6 per decade) and reduced the way
    `Holdable` reduces (highest angle seen while saturated), the 8254 Pa bin
    reads max 20.5 / p90 19.2 against a probe of 17.4, and I wrote that up
    as the probe being 1.8-3.1 degrees pessimistic. **It is not a sound
    comparison.** A 6-per-decade bin is 1.47x wide, the ceiling falls
    monotonically across it, and a `max` therefore reports the value at the
    bin's *thin-air edge* while `probe` is evaluated at its centre. On a
    curve running 20.0 at 6813 Pa to 14.8 at 10000, that artefact alone
    manufactures most of the gap.

    Compared per sample, at each sample's own q, with no binning at all:

    | q (Pa) | n | mean | p90 | p99 | max |
    |---|---|---|---|---|---|
    | 316-1000 | 13 | -55.6 | -55.5 | -55.5 | -55.5 |
    | 1000-3162 | 161 | -15.3 | -2.5 | -2.2 | -2.0 |
    | **3162-10000** | **4831** | -3.2 | **+1.3** | **+2.1** | **+2.6** |
    | 10000-31623 | 1378 | -0.9 | -0.5 | -0.1 | +0.1 |
    | all | 6383 | -3.2 | +0.8 | +2.1 | +2.6 |

    (Negative is the vehicle sitting below the ceiling, which is the normal
    case -- most saturated ticks are bank transients, not the plant's
    limit.) **935 of 6383 samples exceed the probe ceiling, by at most 2.6
    degrees, and they are concentrated in 3162-10000 Pa.** A ceiling that is
    right is never exceeded, so the probe is genuinely a little low in the
    binding band -- about a degree at p90, not three. The three bins the
    table's own comment cites as agreeing to 0.1-0.5 degrees survive.

    So the probe curve is close to right, and the 14-27 km is **not** mostly
    a bad curve. It is that applying *any* honest cap lengthens the
    predicted entry (the table above), and the entry was already being
    predicted too short.

    **What this changes.** The suspect is no longer "an honest propagator
    cannot be dropped in". It is:

    * `HOLDABLE_PROBE` is close enough that re-measuring it is not the
      lever. It is about a degree low at p90 in 3162-10000 Pa, worth
      re-flying eventually, worth nothing now.
    * **The uncapped propagator biases the arrival long, and the flights
      land short.** Those are opposite signs, so there is a *larger*
      short-side error still unaccounted for, and the missing cap was
      partly cancelling it. That error is the open question, and it is not
      the ceiling. It is also not the burn: across 72 flights the deorbit
      delivered within 0.7 m/s of its solved dv and every one exited at
      `+0.00 m/s still owed`, so the burn is cleared as a suspect by
      measurement rather than by assumption.

    **Two rules, and the second is the one I actually broke.**

    *Check the sign of a mechanism against a measurement before building
    the conclusion on it.* Failure 52's fact survives intact and its
    prescription does not; the prescription was the part nobody had
    measured.

    *A wide log-spaced bin reduced by `max` reports its thin-air edge, not
    its centre -- so comparing it against a curve evaluated at the centre
    invents a discrepancy.* `Holdable`'s bins are 1.47x wide and its rule
    is `max`; that is correct for learning a conservative ceiling in
    flight, and wrong for validating a curve against one. Compare per
    sample at its own q. I published the binned version of this finding
    before checking it, and the unbinned number is a third of it.

54. **The deficit is present at the entry interface, not made during the
    entry -- and the guard that should have refused the burn shares the
    propagator that is wrong.** Offline, on logs already flown.

    **Where the miss is made.** Bucketing each flight's own predicted
    `long=` by altitude looks at first like an answer:

    | band (km) | `qs_plane` | `qs_plane_high` | `qs_plane_inc` |
    |---|---|---|---|
    | 60-45 | +3.5 | +0.5 | +0.1 |
    | 45-35 | +2.2 | +0.1 | -0.1 |
    | **35-26** | +2.8 | -1.1 | **-49.9** |
    | 26-20 | -0.3 | -8.3 | -61.6 |
    | 20-12 | -2.7 | -13.3 | -65.0 |

    It is not. **`long=` is censored from above**: while the solve still has
    authority it drives the predicted miss to the reserve by construction,
    so it reads ~0 whatever the true margin is, and only becomes informative
    once the solve saturates. The bank trace proves it -- in 35-26 km
    `qs_plane_inc` flies mean `|bank|` **10.0 deg** against `qs_plane`'s
    28.5, i.e. it is already pinned at `SOLVE_BANK_MIN_DEG` stretching for
    range. The band where `long=` collapses is where the vehicle *runs out
    of margin*, not where the energy went.

    **The energy says it plainly.** Speed and range-to-runway at matched
    altitudes, converted to specific energy:

    | state | at 58 km | at 30 km |
    |---|---|---|
    | `qs_plane` | 1002 km to go, E 2816 kJ/kg | 159 km to go, E 1352 |
    | `qs_plane_high` | 858 km to go, E 2827 | 170 km to go, E 1304 |
    | `qs_plane_inc` | **1168 km to go**, E 2810 | **243 km to go**, E 1351 |

    At the entry interface `qs_plane_inc` is **166 km further from the
    runway on the same energy** as `qs_plane`. At 30 km it is 84 km further
    on the same energy again. It never had the range; the glide spends the
    entry discovering that, and lands 65-85 km short.

    Nor is it flying badly to get there -- measured between 45 and 30 km it
    converts **0.401 km of range per kJ/kg** against `qs_plane`'s 0.328,
    because it is gliding flat and stretching. At its own best observed rate
    1168 km needs ~2913 kJ/kg and it was committed with 2810.

    **So the fault is the commit, and the guard on the commit is blind in
    the same way.** `deorbit_window` reports the span the glide can reach,
    and both the search and the window propagate with the same optimistic
    model:

    | flight | window | gate | landed |
    |---|---|---|---|
    | LOG1973 `qs_plane` | 1972-2014 km (41 wide) | 1989, -21% off centre | -1.1 km |
    | LOG2009 `qs_plane_inc` | 2105-2135 km (**30 wide**) | 2117, -19% off centre | **-69 km** |

    Both look healthy and centred. The second is wrong by **more than twice
    its own window width**. A reachability test whose error is 70x larger on
    one entry state than another is not a safety net, it is a confident
    wrong answer, and it is why "stay in orbit another pass" never fires on
    the states that need it.

    **And it is not a fraction of the entry, so do not re-fit
    `DEORBIT_LONG_BIAS_FRACTION`.** The over-prediction implied by each
    state's landing is 1.1 km on a 1002 km entry (0.11%), 18.6 km on 858 km
    (2.2%), 68.7 km on 1168 km (5.9%). It does not scale with entry length
    -- the *shortest* entry of the three carries the middle error -- so the
    "relative error" the fraction was built to express is not the law.
    Failure 19 retired that constant after five re-fits in one session; this
    is the measurement that says why it could never have held.

    **What to do with it.** The lever is the commit, not the glide, and the
    quantity to compute is *reachable range at the interface state*, honestly
    -- which is the same propagator problem as everything else, but asked
    where it can still be acted on (stay up another pass) rather than where
    it cannot. Confirming it needs `DIAG_STATE=True` flights on at least two
    states so the interface state can be replayed against the propagator;
    no such log exists yet.

55. **The glide's propagator is accurate at the interface. The error is
    entirely in the deorbit's prediction from orbit -- and five other
    suspects are now cleared by measurement.** Offline, on logs already
    flown.

    Failure 54 located the deficit at the entry interface and inferred an
    optimistic propagator. The first half is right; the second is not, and
    the logs already contained the disproof. **The first `long=` a glide
    prints is the propagator's own answer from the interface state**, before
    the solve has converged on the reserve:

    | state | first `long=` | landed | propagator error |
    |---|---|---|---|
    | `qs_plane` | +7.3 km | -0.9 | +8.2 |
    | `qs_plane_high` | -44.2 km | -18.6 | -25.6 |
    | `qs_plane_inc` | **-73.6 km** | **-66 to -69** | **~-5** |

    On the state that fails worst the glide announces, on its very first
    tick with the controls, that it is 73.6 km short -- and it lands 66.
    **The propagator is right to about 5 km.** It is not optimistic, it is
    not hiding anything, and there is nothing for an honest ceiling to fix.
    The vehicle is simply handed a trajectory from which the runway is not
    reachable, and it says so immediately.

    So the same propagator is accurate over the 1168 km from the interface
    and wrong by ~66 km when run from orbit over ~2100 km to the same
    endpoint. That is an **internal inconsistency between two runs of one
    model**, not a model-versus-reality error, and it is the whole remaining
    fault.

    **Cleared by measurement in the course of getting here**, each one a
    suspect this project has spent flights on:

    * *The burn.* 72 flights delivered within 0.7 m/s of the solved dv,
      every one exiting `+0.00 m/s still owed`.
    * *Vehicle mass at the entry.* `entry_mass` already subtracts the
      drained propellant; checked against LOG1973 it predicts 6.665 t
      against an actual post-drain 6.511 t.
    * *`HAC_ALT_M` as a fitted constant.* All three states fall subsonic at
      the **same** altitude -- 14.3-14.5 km at M<1, 18.0-18.4 at M<2 -- so
      12 km is a property of the airframe and the atmosphere, not of the hot
      arrival. Flown at 18 km on `qs_plane` the vehicle splashed 2.6 km long
      and broke up, which is the supersonic cone failing exactly as the
      constant's comment says it does.
    * *Cross-track cost.* Removing `qs_plane_inc`'s 22.7 km of cross-track
      over a ~700 s entry needs a mean lateral acceleration of
      2*22700/700^2 = **0.09 m/s^2**, one to two degrees of bank against the
      ~9 m/s^2 of lift available. Cross-track is very nearly free and cannot
      be the 66 km.
    * *The alpha ceiling*, already, by failures 51-53.

    **What is left, and how to measure it rather than guess.** The suspects
    that survive all live between the deorbit's propagation and the
    interface: the assumed entry schedule (`Steer(ENTRY_ALPHA_DEG,
    SOLVE_BANK_MIN_DEG)` held constant for 2100 km, where the real entry
    flies mean `|bank|` of 42.8 deg on `qs_plane` and 30.1 on
    `qs_plane_inc`), and integration error over the long arc. The assumed
    bank is enormously load-bearing -- measured offline on four states, 30
    deg against 43 deg is worth **139 to 179 km** of entry range -- so an
    assumption wrong by ten degrees is worth more than the whole miss.

    **The instrument this needs does not exist.** `DIAG_STATE` is defined in
    the spaceplane's `Config` and **nothing reads it**; it is a booster
    feature the config inherited, which is why failure 54 could not be
    settled by replay. The cheap fix is not a replay harness: log the
    *predicted* interface state at deorbit commit and the *actual* one at
    GLIDE entry, and the handover error is one subtraction per flight. That
    measures where the 66 km is born instead of hypothesising about it --
    which, after six refuted mechanisms, is the change of method this needs.

56. **A burn that had finished would not end, on a seventh of every batch,
    and the test that was meant to catch it read the source instead of the
    behaviour.**

    Found while checking why the new interface instrument fired on only one
    flight in ten. `LOG2050`: the burn delivers its dv correctly
    (2080.7 -> 2047.3 m/s, 33.4 m/s), cuts the throttle at 39419.9 -- and
    then sits at `thr=0.00` for **35 seconds** until
    `DEORBIT_MAX_BURN_S` fires and the phase leaves on
    `DEORBIT -> DRAIN burn guard at 60 s`.

    **The loop.** The last-tick rule decides that burning one more floored
    tick lands further from the aim than cutting does, so it sets
    `deorbit_ticks` to the exit threshold and returns. But the exit test
    runs at the *top* of the next tick, after

        if owed <= 0.0: self.deorbit_ticks += 1
        else:           self.deorbit_ticks = 0

    has already reset the counter -- and `owed` is a small **positive**
    sliver, which is exactly the state the rule fires in. So the burn cuts,
    the counter is cleared, the burn cuts again, until the runaway guard
    ends the phase. The counter can never carry a decision made below it.

    **How common.** Counted over three batches of logs: **7 of 24** burns in
    batches 9-10, **2 of 12** in batch 13, **5 of 10** in batches 15-16.
    Between a seventh and a half of every batch this project has measured.

    **What it cost, honestly: almost nothing on the ground.** The burn is
    already complete when the idling starts and the idle is in vacuum, where
    dumping propellant applies no force, so the trajectory is set either
    way. Measured on `qs_plane`, 20 flights split by exit path:
    guard-cut **-0.7 km sd 0.6** (n=5) against normal **-1.0 km sd 0.5**
    (n=15). The cost is farm throughput -- 35 s of game time per affected
    flight -- and the noise of a scary log line on half a batch.

    **The test that should have caught it.**
    `test_it_exits_rather_than_idling_at_the_floor` asserted that
    `deorbit_ticks` appears within 400 characters of the cut. It did. The
    test passed for the entire life of the bug, because it checked that a
    *name was mentioned* rather than that the burn ends. It now drives the
    law with `owed` stuck just above zero and asserts termination in at most
    two ticks -- which fails against the old code.

    **Two rules.**

    *A decision made in one tick must be carried by something the next tick
    cannot clear.* A sticky `deorbit_done` flag, checked alongside the
    counter; the counter stays for the ordinary debounced exit.

    *An instrument must not hang on one of two exit paths.* The interface
    prediction was called from the burn's range-test exit, so every
    guard-cut flight -- half of that batch -- silently produced an "actual"
    line with no "predicted" line to subtract from. It is taken on the first
    DRAIN tick now, which every route out of DEORBIT passes through.

57. **Measured at last: the deorbit hands the inclined entry a trajectory
    92 km out of position, and the glide inherits it.** Eleven flights with
    `DIAG_INTERFACE=True`, two entry states, rotated across three instances.

    The instrument logs what the committed burn predicts for the 58 km entry
    interface and what the vehicle actually brings there. Subtract:

    | arm | n | arc error | time error | speed error |
    |---|---|---|---|---|
    | `qs_plane` | 5 | **-2.5 km sd 2.4** | -18.0 s sd 1.6 | -2.9 m/s |
    | `qs_plane_inc` | 6 | **+91.3 km sd 3.7** | -66.4 s sd 1.9 | -7.8 m/s |

    Roughly **50 standard errors apart**, and reproducible to under 4 km
    within each arm. Positive arc error means the vehicle is *further from
    the gate* than the burn predicted. `qs_plane_inc` then lands
    **-66.4 km sd 2.8**, so the glide claws back ~25 km of a 91 km deficit
    and no downstream tuning can do better -- which is why six successive
    attempts to fix the *glide* all came back null or negative.

    **The vehicle arrives in the right state, in the wrong place.** Altitude
    matches to 4-70 m and speed to 0.4%; it is simply 66 s early and 91 km
    short of the predicted ground track. The real flight path is steeper
    than the model's.

    **Where it is not.** Each of these was checked rather than assumed:

    * *The vacuum arc.* The propagator's apoapsis-to-58 km time is 453.2 s
      against LOG2049's measured 453 s. Exact.
    * *Propagator self-consistency.* Splitting a propagation at the
      interface reproduces the one-shot answer to **0.0 km** on four states.
    * *Entry mass.* `entry_mass` gives 6.805 t against an actual 6.696 for
      `inc` and 6.666 against 6.511 for `plane` -- two saves with very
      different loads (537.9 and 1572.1 units), both right.
    * *The attitude flown into the air.* From 70 km down both states fly
      **identically**: 22.7 deg achieved against 22.0 commanded, 0.6 deg of
      sideslip. The upper-atmosphere entry is not flown differently.
    * *The lift model.* The `lift trim, measured` line in every log reads
      **0.96-1.00 in every Mach bin from M0 to M7**. Lift is not
      over-predicted at entry speed.

    So the vacuum leg is exact, the aero model is good, the mass is right
    and the attitude matches -- and the real legs take nearly the same total
    time on both states (479 s against 486 s) while the propagator predicts
    `inc` takes **40 s longer** than `plane` when it actually takes 10 s
    *less*.

    **How small the culprit has to be.** Measured off the logs, the real
    flight path angle at 70 km is **-1.66 deg** on `qs_plane` and
    **-1.84 deg** on `qs_plane_inc`, and at 58 km -1.78 against -2.08. A
    12 km descent at 1.72 deg covers 400 km of ground; at 1.96 deg it covers
    351 km. **So 0.2 degrees of flight path angle at 70 km is worth ~50 km
    of ground track**, and the measured leg durations follow (207 s against
    186 s).

    That is the constraint worth carrying: the 91 km does not need a large
    modelling error. It needs about **7 m/s of vertical speed out of 2078**
    -- 0.3% -- at the top of the atmosphere. Look for something that small,
    not something big, and note the burn is already accurate to 0.7 m/s so
    it cannot supply it.

    **What is left.** The propagator's flight path angle through 70-58 km,
    inherited from a post-burn state that is right to 4 m of altitude and
    3 m/s. The next instrument is the obvious one: a second probe at the
    70 km atmosphere boundary, splitting the vacuum leg from the
    aerodynamic one. Everything above says the error must be in the second.

59. **RETRACTION: failure 57's numbers measured arcs to two different
    runway ends, 44 km apart.** The conclusion may survive; the numbers do
    not, and they are withdrawn until re-flown.

    `Runway.gate` returns the **high** gate while the cone is flying, and
    the high gate sits `HAC_ALT_M * HAC_LD` (~22 km) out along the approach
    centreline -- so the two ends' gates lie on *opposite* sides of the
    field, about **44 km apart**. `log_interface_prediction` ran at the
    DRAIN tick against `self.end` as it then stood; `log_interface_actual`
    ran after `run_coast` had re-chosen. Measured on every flight of both
    entry states: the deorbit solves **`on runway 27`** and the glide flies
    **`runway 09`**. The instrument was subtracting an arc to 27's gate from
    an arc to 09's.

    **The tell was a number that could not be true.** Splitting the error at
    the 70 km boundary gave a *vacuum* leg of **-34.9 km on `qs_plane` and
    +19.0 km on `qs_plane_inc`** -- opposite signs on a ballistic arc, with
    the two crossings sampled 43 and 144 m apart in altitude so it was not
    sampling noise. A ballistic leg cannot do that. Chasing the impossible
    number rather than explaining it away is what found the bug.

    **What survives, because it never used the diagnostic's arc:** the
    landing figures; the timing split (`qs_plane_inc` predicted 545 s to the
    interface and took 479, against 506 and 490 on `qs_plane`); the flight
    path angles (-1.66 deg against -1.84 at 70 km) and the ~50 km-per-0.2
    deg sensitivity; and every suspect cleared in failure 57 -- the burn,
    entry mass, the attitude flown into the air, the lift model, and the
    propagator's self-consistency, all of which were measured off the
    flights or offline.

    **What is withdrawn:** the +91.3 km and -2.5 km interface errors, and
    the vacuum/aero split built on them.

    **And a real question is left behind, about the flight and not the
    instrument.** The burn is solved against a gate the glide does not fly
    to, on every flight, 44 km away. `qs_plane` lands anyway, so the glide
    absorbs it there; whether `qs_plane_inc` can is exactly the sort of
    thing that fails state-dependently, and it is now the first thing to
    measure once the instrument is trustworthy.

    **The rule.** *A diagnostic that reads a moving target must pin the
    target, not sample it twice.* Both lines now measure against one named
    end and each records the end its phase had chosen, so the mismatch stays
    visible instead of being silently differenced away.

58. **The instrument moved what it was built to measure, and I blamed the
    farm for it.** Two mistakes, one flown test that caught both.

    `qs_plane` landed **-1.1 km sd 0.3** across batches 9-10 (n=24) and
    **-2.7 sd 0.9** after `DIAG_INTERFACE` was added. Three candidates: the
    `deorbit_done` fix (failure 56), the diagnostic, or the farm.

    **First conclusion, and it was wrong.** Splitting the old logs by exit
    path gave 1005.0 km sd 4.5 for normal exits and 1001.6 sd 3.1 for
    guard-cut ones, against 1015.9 sd 6.5 for "the new code" -- outside both
    old populations, so the fix could not explain it, and I concluded farm
    degradation over a long session. I was about to write that up as a
    reusable farm-health canary. **The "new code" population was almost
    entirely `DIAG_INTERFACE=True` flights**, so I had attributed the
    instrument's own effect to the environment.

    **The interleaved A/B, five flights each, same farm, same code:**

    | | interface (km to runway) | landing |
    |---|---|---|
    | batches 9/10, no diagnostic (n=24) | 1004.0 sd 4.4 | -1.1 sd 0.3 |
    | `DIAG_INTERFACE=False` (n=5) | **1006.4 sd 5.3** | **-1.5 sd 0.6** |
    | `DIAG_INTERFACE=True` (n=5) | **1015.5 sd 5.4** | **-3.0 sd 1.3** |

    The off arm sits on the historical value (0.8 se). The on arm is 11.5 km
    beyond it (4.4 se). **The farm was fine and the `deorbit_done` fix was
    fine; the diagnostic was the whole effect.** A ~400-step entry
    propagation at the first DRAIN tick stalls the control loop, and this
    project already records that loop rate drives entry scatter
    (sd 0.3 -> 2.7 km).

    **The fix costs nothing.** Every entry propagation the burn already runs
    passes through both the atmosphere top and `ENTRY_INTERFACE_M` on its
    way to the gate. `Prediction` now records both crossings (two
    comparisons per step), `deorbit_progress` hands the prediction back on
    request, and the instrument reads the crossing instead of propagating.
    The same change buys the 70 km probe failure 57 asked for, free -- and
    it is taken at `atmosphere_depth`, not at `entry_r`'s skip threshold
    `SKIP_ENTER_MARGIN_M` below it, which would put 5 km of air on the wrong
    side of the vacuum/aero line.

    **Two rules.**

    *A diagnostic must not change what it measures -- and on a vehicle flown
    by a control loop, cost is a side effect.* Prefer reading a quantity
    something already computes over computing it again.

    *When a measurement shifts and a change of yours is in the frame, test
    the change against itself, interleaved, before reasoning about the
    environment.* Batch-to-batch drift and a code change are confounded by
    construction; only the interleaved arm separates them. Failure 57's
    result survives precisely because both its arms were flown with the
    diagnostic on.

60. **The deorbit aims at a runway end the glide does not fly to, and that
    mismatch is load-bearing on both entry states.** Twelve flights,
    `RUNWAY_BOTH_ENDS` pinned against free, two states, interleaved.

    `Runway.choose` picks the end from the chord to the threshold. From
    orbit that resolves to **runway 27**; by the time the glide has the
    controls it resolves to **runway 09** -- on every flight of both states.
    The two *high* gates sit `HAC_ALT_M * HAC_LD` (~22 km) out along
    opposite approach centrelines, about **44 km apart**, so the burn is
    solved against a gate the glide never flies to.

    `RUNWAY_BOTH_ENDS=False` pins 09 and removes the mismatch outright:

    | arm | n | along |
    |---|---|---|
    | `qs_plane`, free (27 -> 09) | 4 | **-1.4 km sd 0.6** |
    | `qs_plane`, pinned 09 | 4 | **-4.9 km sd 0.3** (~10 se worse) |
    | `qs_plane_inc`, free | 4 | **-66.8 km sd 3.3** |
    | `qs_plane_inc`, pinned 09 | 4 | **-77.1 km sd 2.2** (~5 se worse) |

    **Both states get worse.** Pinning takes `qs_plane` from inside the
    +/-1200 m window to 5 km short of it, so the mismatch is not a defect
    sitting on top of a working system -- it is holding one up.

    **And the 44 km is a distance between two points, not a range error.**
    Removing it costs 3.5 km on `qs_plane` and 10.3 km on `qs_plane_inc`,
    because the glide re-solves against 09's gate the moment it takes the
    controls and absorbs most of the difference. Do not reason from the gate
    separation to a miss; measure it.

    **This is the seventh mechanism tried against generality and the seventh
    refuted**, after `SOLVE_MAX_RANGE_ON`, `HOLDABLE_EXTRAPOLATE`,
    `HOLDABLE_PROBE_ON`, `HAC_ENERGY_BUDGET`, the `APPROACH_FACTOR` cut and
    `HAC_ALT_M` -- and like several of those it failed *because it removed a
    compensation*. That recurrence is now the finding rather than the
    footnote: **the flown configuration is a stack of mutually-cancelling
    errors fitted against `qs_plane`, and removing any single one makes the
    landing worse.** It explains both why nothing transfers and why every
    principled local fix has come back negative.

    **What that implies about method.** Individual corrections cannot work
    here; the root error and everything fitted against it have to move
    together, or the mechanism has to be one that does not depend on the
    fit at all. The single mechanism that ever generalised is
    `GLIDE_RESERVE_M` -- +2.8 to +6.7 km on *every* entry state -- and it
    works by **carrying margin rather than predicting more accurately**.
    Prefer that shape.

61. **The propellant drain fires after the burn's closed loop has finished,
    and it is worth 3.3 m/s on one entry state and 9.3 on another. That gap
    is the generality gap.** Four arms, interleaved, two entry states.

    Every mechanism tried against "nothing transfers" (failures 50-60) asked
    how to *predict* the entry better. None of them could work, because the
    thing that differs between entry states is not a prediction at all: it
    is a real impulse, delivered in vacuum, after the last measurement that
    could have answered for it.

    **The instrument.** `cutoff drift` records the vehicle's specific energy
    at the `DEORBIT -> DRAIN` tick and again at the first COAST tick, and
    reports the difference as an equivalent dv. Both are states the loop
    already samples; there is no propagation and nothing steers on it.

    | arm | `qs_plane` | `qs_plane_inc` |
    |---|---|---|
    | as flown | **-3.1 to -3.5 m/s** | **-9.0 to -9.4 m/s** |
    | `DRAIN=False` | **-0.27** | **-0.36** |

    In vacuum, with the engine cold. Tight inside each state (sd under
    0.2 m/s over six flights each) and different between them by 6 m/s.

    **Three suspects, two eliminated by the same batch.** The engine is not
    it: `thrust` is now streamed and `Fn` reads **0.0 kN on the first tick
    after cutoff**, and `DEORBIT_CUTOFF_SHUTDOWN` -- deactivating the engine
    at cutoff rather than commanding zero throttle -- drifts -9.05 against
    the control's -8.97 and -9.36 (LOG2111 against LOG2109/2110). RCS is not
    it either: `qs_plane` spends no monopropellant at all across the window
    and still drifts 3.3 m/s. `DRAIN=False` takes the drift to zero. **It is
    the release valve.**

    **Why it is exactly the shape of the open failure.** The burn's stop
    test is closed-loop on the state -- it propagates from where the vehicle
    *is* and stops when that arc is right -- so it is immune to everything
    except an impulse arriving after it has stopped looking. The valve
    arrives there, every flight, with a magnitude set by the vehicle's own
    propellant load. At the entry's ~12 km of ground track per m/s the 6 m/s
    difference between these two states is ~70 km of arrival, which is the
    difference between `qs_plane` landing and `qs_plane_inc` arriving 66 km
    short. No constant can absorb that, because it is not an error in a
    model: it is dv the model never hears about.

    **The fix is not to model it, it is to move it upstream of the loop.**
    `DRAIN_BEFORE_BURN` runs the valve *before* the deorbit, stopping at
    `DRAIN_RESERVE_UNITS`; the burn is then solved and flown at the drained
    mass and its stop test measures a state no valve will change afterwards.
    The reserve stays aboard for the entry -- 40 units is 200 kg on a 6.7 t
    vehicle, 3% against the 29% the drain exists to shed.

    **And the first version of that did not keep a reserve at all.** The
    valve empties 556 units in 2.0 game-seconds; this control loop cannot
    tick faster than about two of those, so the first tick after opening it
    reads zero and the burn had nothing to spend (LOG2118: "drained in 2.0 s,
    mass now 6.715 t, 0.0 units left"). kRPC *reads* are milliseconds, so the
    phase now holds the valve open and watches the tank directly, closing it
    on the reserve rather than waiting for its own next tick. The rule:
    **a control loop's tick rate is the resolution of everything it closes a
    loop on, and a valve can be faster than the loop.**

    **Also fixed by the same change, and it would have bitten the other
    way:** `entry_mass` subtracts the propellant aboard because the drain is
    still to come. With the drain moved it is not, so the subtraction would
    predict a vehicle 3% lighter than the one that flies -- the same error
    this method exists to remove, with its sign reversed.

    **Flown, four flights per cell, interleaved, two entry states.** The
    arrival is the predicted along-track miss at `GLIDE -> HAC`:

    | | as flown | `DRAIN_BEFORE_BURN=True` |
    |---|---|---|
    | `qs_plane` | -8.6 km sd 1.0 | **-0.6 km sd 3.2** |
    | `qs_plane_inc` | -68.1 km sd 2.6 | **-11.3 km sd 3.6** |
    | **spread between the states** | **59.5 km** | **10.7 km** |

    Both states move long, by the amount their own valve was costing them
    (+8.0 and +56.8 km, against ~3.3 and ~9.3 m/s of drift -- 2.4 and
    6.1 km per m/s), and the gap between them closes by 5.6x. This is the
    first mechanism since `GLIDE_RESERVE_M` to help every entry state, and
    the first of any size to close the *difference* between them. `qs_plane`
    landed **+97 m along, -56 m across, 23 of 23 parts** on the first flight
    of the new arm, from an arrival of -0.6 km.

    **And the landing bias is not a constant either.** It was measured at
    +3881 m sd 96 over flights that arrived 5-9 km short and spent the
    entry stretching; the flight that arrived on the gate added **+680 m**.
    A chain that has to stretch lands differently from one that does not, so
    "arrival" and "landing bias" are not merely separate numbers -- the
    second is a function of the first.

    **What is left, and it is now visible because the valve stopped hiding
    it.** The residual `cutoff drift` is -0.04 to -0.66 m/s, mass falls 2 kg
    across it, and that is the engine's own tail-off after a commanded
    cutoff -- the thing `DEORBIT_CUTOFF_SHUTDOWN` was built for and measured
    as a null while the valve was worth fifteen times as much. At ~6 km per
    m/s its 0.6 m/s of spread is the 3-4 km of scatter the new arm shows,
    where the old one ran sd 1.0-2.6. **A null measured against a dominant
    disturbance is a null about the disturbance, not about the mechanism**;
    every refuted mechanism in failures 50-60 was measured that way and is
    worth re-flying now.

62. **The approach's speed schedule works once it stops ramping into the
    flare, and the knob's useful end is the opposite one from the airframe
    measurement.** Four arms, four flights each, `qs_plane`, all with
    `DRAIN_BEFORE_BURN`.

    Failure 49 left `APPROACH_SPEED_PROFILE` off with a diagnosis: the ramp
    reached its target *at* the trigger height, so the vehicle arrived at the
    flare still decelerating and carried on through the target into the
    stall. `APPROACH_PROFILE_HOLD_M` finishes the ramp 200 m above the
    trigger, leaving a stretch of constant command for the speed loop to
    settle on.

    | `APPROACH_FLARE_FACTOR` | flare entry (m/s) | intact | `\|along\| <= 1200` | on the strip |
    |---|---|---|---|---|
    | profile off | 106.9 107.7 106.8 102.7 | 2/4 | 1 of 2 | 1 of 2 |
    | 1.85 | 76.0 77.7 72.3 80.9 | 3/4 | 2 of 3 | 3 of 3 |
    | 1.95 | 83.4 92.2 90.9 83.9 | 2/4 | 1 of 2 | 2 of 2 |
    | **2.05** | 101.0 91.1 93.6 100.2 | **3/4** | **4 of 4** | 3 of 4 |

    **The schedule does command the flare's entry speed** -- 72-81 at 1.85
    against 103-108 with it off, monotone in the factor -- which is what the
    constant could never do. What it does not do is *hold* the number it is
    given: 1.85 asks for 89 m/s and delivers 77, 1.95 asks for 94 and
    delivers 88. A glider on a fixed geometry cannot choose speed and path
    independently, so the commanded speed is a bias on the schedule and not
    a setpoint, and the factor has to be calibrated against the speed
    delivered rather than read off the airframe.

    That is why the useful value is **above** the measured 83-91 m/s window
    rather than inside it, and why failure 49's reading of 2.05 as "the
    control evaporates" was a reading of the un-held ramp. With the hold,
    2.05 put **4 of 4 inside the along-track window at sd 302 m**, against
    1020-1988 m for every other arm including the one it was fitted against.

    n=4 per arm, so the ordering is a bracket and not a result -- see the
    rule about three-flight arms. What the bracket does establish is that
    the *mechanism* connects, which is the thing a knob has to do first.

63. **The landing configuration is fitted to the farm's loop rate, and the
    instrument that first suggested so was the log's own cadence.** Two
    mistakes worth keeping, in that order.

    **The misreading.** The FLARE telemetry lines sit **2.02 game-seconds
    apart** across sixteen flights, which reads as a control loop far too
    coarse for a six-second manoeuvre: a hundred metres up, thirty metres a
    second of sink, five to eight control actions in the whole flare. It is
    the logging cadence. `Logbook.telemetry` gates on `LOG_INTERVAL_UT`,
    which is **2.0**, and the same 2.02 appears at `--timescale 1.5` where
    the loop has four times the wall clock per game-second. *To measure the
    loop, set `LOG_INTERVAL_UT` below the tick you are asking for, or count
    something the loop itself writes.*

    **The real effect, flown.** The same configuration at `--timescale 1.5`
    against `--timescale 6`, `qs_plane`, four flights each:

    | | flare entry (m/s) | at the ground | stopped |
    |---|---|---|---|
    | timescale 6 | 90.7 93.5 99.5 102.2 | 35-64 m/s | 1995-2057 m |
    | timescale 1.5 | **78.5 68.6 69.8 51.5** | 48.7-50.3 m/s | **3881-5441 m** |

    Flare entry falls by 25 m/s and the vehicle rolls out one to three
    kilometres further -- off the end of the runway. Nothing about the
    vehicle changed; what changed is how many times a second the speed loop
    gets to act on `APPROACH_SPEED_PROFILE`'s command, and at 1.5x it tracks
    a command that at 6x it merely leans towards. **A configuration tuned
    against a loop that cannot keep up is tuned against the shortfall.**

    This is the same shape as fitting to one entry state, one layer down,
    and it means CLAUDE.md's *compare nothing across that boundary* is not
    only about scatter: the mean moves too, by more than the runway is long.
    Anything that would be flown at 1x by a person -- which is every
    configuration this project intends to ship -- has to be confirmed there.

64. **The pre-burn reserve was a units count, and the first entry state that
    needed a bigger burn ran it dry.** Caught by the line that was written to
    catch it.

    `DRAIN_BEFORE_BURN` leaves `DRAIN_RESERVE_UNITS` aboard for the deorbit.
    40 units is about 69 m/s on this vehicle, which covers the 32-44 m/s that
    `qs_plane` and `qs_plane_inc` solve. `qs_plane_high` solves **121.6 m/s**.
    LOG2204, in order:

    ```
    drain watched down to 37.5 units (69 m/s of burn at Isp 355)
    deorbit solution dv=121.6 m/s on runway 27, 2250 km to run
    DEORBIT -> COAST burn guard at 60 s
    GLIDE -> HAC over the field long=+812329
    ```

    Four of four on that state, the same way. The reserve is now a **dv
    budget** put through the rocket equation at the Isp the vehicle reports
    and the mass it will have (`DRAIN_RESERVE_DV_MS`, 200 m/s, margin 1.25),
    with the units figure kept only as a floor.

    **The useful part is that the diagnosis took one line of one log.** The
    constant's comment named what would contradict it -- "a burn that runs
    out of propellant" -- and the reserve prints the dv it buys rather than
    the units it keeps, so the contradiction and the evidence for it are the
    same string. That is the rule about measurements whose only consumer is a
    hand-copied constant (failure 13), applied before the fact instead of
    after it.

65. **The control loop's rate was treated as a property of the farm, and it
    is a property of the *controller*.  The measured effect is all in the
    burn -- and the first account of it written here was wrong.**

    Failure 63 measured that the landing chain is fitted to the farm's loop
    rate and left CLAUDE.md saying *compare nothing across that boundary*.
    That rule is expensive: it says a landing measured at 6x tells you
    nothing about the landing a person flies, so the farm cannot measure the
    phase that is binding.

    The way out is to notice that the **time scale is the free variable and
    the control interval is the constraint**.  Ask for a fixed interval in
    *game* seconds, measure what one tick costs in wall seconds, and the
    fastest honest scale is the ratio.  `common.pacing.ScaleGovernor`
    writes it to the plugin's control file; `Config.TIMESCALE_GOVERNOR` is
    the path, `quickglide --timescale` becomes a ceiling, `--no-govern`
    restores the old meaning.  Throughput is unchanged: 5.0-6.0x achieved.

    **Measured against itself** -- one save, the committed defaults, the same
    farm in the same state, the only difference being `--no-govern`:

    | | ungoverned 6x (n=6) | governed (n=18) |
    |---|---|---|
    | `DEORBIT` tick | **0.30 game-s** | **0.10 game-s** |
    | `HAC` / `APPROACH` / `FLARE` tick | 0.11 | 0.11 |
    | arrival at `GLIDE -> HAC` | **-3226 m, sd 806** | **+294 sd 171, +409 sd 100** |

    **Every phase but the burn was already being served.**  A tick costs
    3-6 ms of wall clock on final, so even at 6x the approach gets its 0.1
    game-second command interval; the burn's tick costs 44-62 ms because it
    propagates, and 6x turns that into 0.3 game-seconds of a throttle taper.
    So the governor buys one thing, it buys it in one place, and it is worth
    3.5 km of bias and a five-to-eightfold cut in the entry's scatter.

    **The retraction.**  This entry first claimed the flare "got four
    commands and now gets forty-six", from FLARE telemetry lines two seconds
    apart.  That is `LOG_INTERVAL_UT`.  It is *exactly* the misreading
    failure 63 records and warns about, made again eighteen entries later by
    the person who wrote the warning -- and it survived because it was
    plausible and because nothing was measured against it until a matched
    control arm was flown.  The `loop rate` line now written at every
    shutdown is the answer to that: it reports what the loop did, per phase,
    from the loop rather than from the log.

    ```
    loop rate, game-s per tick / wall-s of work: DEORBIT 0.10/0.062 n=97
      COAST 2.01/0.016 n=144  GLIDE 1.01/0.029 n=568  HAC 0.11/0.003 n=1508
      APPROACH 0.12/0.003 n=437  FLARE 0.11/0.002 n=99
    ```

    Read it before comparing two logs. *A number you can only get by
    subtracting two log timestamps is a number about the log.*

66. **The approach's speed loop could only ever slow down, and one g is a
    pull-up.**

    `alpha = trim + APPROACH_SPEED_KP * (speed - target)`, floored at `trim`
    by `APPROACH_TRIM_FLOOR`.  Faced with a surplus it adds angle and bleeds;
    faced with a *deficit* it reaches its floor and waits.  And the floor is
    not neutral: a vehicle descending at its best glide angle of 18 degrees
    flies a straight path at `cos(18) = 0.95` g, so holding a full g is a
    standing pull-up.  It flattens, it slows, `trim` rises with the slowing,
    and the drag that comes with the higher `trim` slows it again.

    `LOG2246`, every other tick from 830 m:

    ```
    h= 833 v= 96.6 sink=29.5 aoa=7.5   rwy=2499
    h= 654 v= 88.1 sink=26.2 aoa=8.2   rwy=3109
    h= 465 v= 80.0 sink=23.5 aoa=9.2   rwy=3708
    h= 282 v= 72.2 sink=20.4 aoa=10.4  rwy=4275
    h= 128 v= 63.1 sink=16.8 aoa=12.4  rwy=4791
    APPROACH -> FLARE h=88.4 v=59.9
    ```

    Monotone in every column: slower, shallower, more angle, further down the
    runway.  Nothing was fighting it -- the law had reached its floor and
    stopped.  **This is failure 63 from the other end**: the coarse loop hid
    it, because a controller that cannot obey its own command cannot chase
    itself into the corner either.

    The fix is not a gain.  A glider's speed *is* its descent angle, so
    command the descent:

        dv/dt = g sin(theta) - D/m          L = m g cos(theta)

    -- the first says which angle holds the speed, the second what load flies
    that angle, and `alpha_for_load` turns the load into an angle out of the
    swept table.  One time constant (`APPROACH_SPEED_TAU_S`), no gain, and
    the old speed floor becomes a floor under the *target* instead of a clamp
    on the angle.  `APPROACH_TRIM_FLOOR`'s caution -- 27 flights that unloaded
    to 4-6 degrees and arrived 40-65 degrees below the horizon -- moves onto
    the path as `APPROACH_DIVE_MAX_DEG`, where no speed or mass can move it.

67. **An equilibrium commanded open-loop is a positive feedback.**

    `L = m g cos(theta)` is a statement about a *steady* glide.  The first
    version of the law above computed the steady-state load for the descent
    it wanted and commanded it, every tick, with no reference to the descent
    the vehicle actually had.  `LOG2247`:

    ```
    h=1919 v=104.6 sink=43.8 aoa=8.1
    h=1353 v=114.1 sink=63.7 aoa=7.3
    h= 780 v=123.4 sink=71.9 aoa=6.4
    h= 350 v=123.5 sink=69.7 aoa=5.2
    FLARE at 222 m, 123.6 m/s, 72 m/s of sink -- into the ground at 98.5
    ```

    This airframe delivers about 85% of the angle it is commanded, so the
    load arrives short, the path steepens past the target, the speed rises,
    and the table answers the higher speed with a *lower* angle still.  Every
    step of that is the law working as written.

    So the load is the steady-state one **plus a gain per radian of the
    difference between the descent the vehicle has and the one it was asked
    for** (`APPROACH_PATH_KN`): the ordinary inner loop of a two-loop
    controller, and the reason the outer one is allowed to be a slow energy
    argument.  The lesson generalises past this law -- three of this
    project's control failures are a command derived from an equilibrium the
    vehicle was not in.

68. **The glide's reserve was insurance against scatter that no longer
    exists, and it was being paid every flight.**

    `GLIDE_RESERVE_M = 4000` aims the entry four kilometres long and asks the
    cone and the approach to give it back.  It was set when the arrival's own
    spread was kilometres (failure 63's loop rate, measured at sd **6.5 km**),
    and against that spread a four kilometre cushion is the difference
    between landing long and landing in the sea short of the field.

    With the loop governed the arrival lands on its commanded point to
    **sd 42 m**, and the cushion is simply four kilometres of height nothing
    downstream can spend.  `logs/LOG2273`, the approach in one column -- the
    vehicle crosses the threshold at **803 m** and is still **130 m** above
    its profile at `rwy=3012`, past the far end:

    ```
    rwy=  691  alt= 803  v=95.5  exc= +865
    rwy= 1846  alt= 440  v=80.1  exc= +494
    rwy= 2875  alt= 102  v=71.7  exc= +169
    ```

    **And the aim point cannot fix it.**  `TOUCHDOWN_AIM_M` 2400 -> 400 moved
    the aim two kilometres *earlier* and the vehicle stopped nine hundred
    metres *later* (+1991 -> +2900 over six flights each): a nearer aim is a
    larger computed surplus, and the approach spends surplus by flying
    further, not by landing sooner.  A knob that is upstream of a phase with
    no authority over the quantity you are moving is not a lever.  Compare
    failure 54's `long=`.

    Cutting the reserve to 1500 m instead: arrival +1524 sd 42, landing bias
    **-39 m sd 595** -- the chain now hands the wheels exactly the arrival it
    was given -- and **5 of 6 kept 18 parts or more** against 2 of 16 before.

    The general shape is worth keeping: **a margin is a fitted constant with a
    standard deviation baked into it, and it has to be re-cut whenever that
    deviation moves.**  Nothing in the program knew that 4000 meant "three
    sigma of an entry that scattered 6.5 km", so nothing objected when the
    scatter fell by two orders of magnitude.

69. **``aim_runway`` flew a pitch attitude where every caller computed an
    angle of attack, and the error was exactly the descent angle.**

    ``guidance.flare`` and ``guidance.approach`` both build their command with
    ``alpha_for_load`` -- an angle measured from the *airflow*.  Below
    ``FLARE_ALIGN_ALT_M`` the autopilot flies it with ``aim_runway``, which
    pitches the nose that many degrees above the *runway*.  The two differ by
    the flight path angle: twenty degrees at the flare's door, zero on the
    ground.  So the wing was handed more lift than the guidance asked for, in
    exact proportion to how fast the vehicle was coming down -- and the flare
    is the one phase where both of those are large.

    ``logs/LOG2290``, with the flare already told to unload:

    ```
    h= 107  v=95.6  vs=-31.1  aoa= 6.3 commanded / 11.0 achieved
    h=  41  v=73.8  vs= -5.1  aoa= 8.2 commanded / 12.6 achieved
    h=  35  v=64.4  vs= -1.8  aoa=10.1 commanded / 12.4 achieved   <- float
    h=  24  v=49.2  vs= -4.8  aoa=13.0 commanded / 17.1 achieved
    FLARE -> ROLLOUT speed=40.9, destroyed
    ```

    ``FLARE_TRACK_LOAD_MIN`` was commanding 0.85 g and the airframe was
    flying about 1.05, so the vehicle levelled at thirty-five metres, bled
    74 m/s to 44 in six seconds and fell the rest.  **A knob that changes
    nothing may be disconnected rather than powerless** -- this one was
    connected to the wrong quantity.

    ``AIM_RUNWAY_TRUE_ALPHA`` pitches to ``alpha + descent`` instead, capped
    by the tail, and leaves the heading on the runway -- the crab fix
    (failure 34) is what the method exists for and is untouched.  The float
    disappears: the same flare now descends 80 -> 50 -> 29 -> 8 m at 21, 11,
    10, 12 m/s of sink and touches down at 48.3 m/s, intact.

70. **It was the wing, not the tail, and the number that predicted it was
    the cross-track at the flare's door.**

    Every wreck of the working arm is ``destroyed in ROLLOUT``, and for a
    long time that was read as a touchdown-speed problem.  It is not: across
    eighteen flights the touchdown speeds of the survivors and the wrecks
    overlap completely (47-50 m/s in both).  What separates them is where the
    vehicle was when the flare started.

    | cross-track at `APPROACH -> FLARE` | outcome |
    |---|---|
    | +16, -36, -45, -68, -69 m | kept its parts |
    | -75, -77, -79, +127, +189, +206, +233 m | destroyed |

    And the first parts lost say why: ``Structural Wing Type A`` and an
    elevon, every time.  The flare is eight seconds long and the only way to
    move sideways in it is to put a tip down -- at fifty metres, on an
    airframe whose wings are the lowest thing on it.

    The lateral capture was *designed* to finish at the wheels
    (``|cross| / (to_flare + in_flare)``, failure 34), which is right about
    the cross-track at rest -- it holds 33 m -- and wrong about the wing.
    Finishing at the door instead (``APPROACH_CAPTURE_BY_FLARE``) enters the
    flare at +1 and +0 m, and then *drifts* 60-80 m during it, because
    arriving at zero offset is not arriving at zero rate.  Interpolating
    between the two (``APPROACH_CAPTURE_FLARE_SHARE``) did not beat either.

    What actually fixed it was upstream: see failure 71.

71. **The cross-track at the flare is made by the cone's roll-out distance,
    and it is nearly a straight line.**

    Nine flights of one configuration, the cone's exit against the offset the
    flare inherited:

    ```
    gate= 778 -> cross= -37   landed        gate=1209 -> cross=+127  landed
    gate= 835 -> cross= -10   landed        gate=1288 -> cross=+189  destroyed
    gate=1115 -> cross= +40   landed        gate=1323 -> cross=+233  destroyed
    gate=1145 -> cross= +70   destroyed     gate=1324 -> cross=+206  destroyed
    ```

    A cone that rolls out further from the gate hands the approach more
    height than the straight-in geometry can spend, the S-turn takes the
    surplus -- that is what it is for -- and the excursion it flies is the
    cross-track the flare inherits.  So the offset at the flare's door is not
    a lateral-control failure at all; it is the cone's energy error wearing a
    lateral costume, and no amount of capture tuning reaches it.

    ``HAC_ROLLOUT_M`` 1500 -> 900 keeps the cone turning until it is closer.
    Nine flights, one configuration, `qs_plane`:

    | | before | after |
    |---|---|---|
    | on the runway (\|along\| <= 1200) | 5 of 9 | **9 of 9** |
    | inside the strip (\|across\| <= 35) | 4 of 7 | **9 of 9** |
    | kept 18+ parts | 4-6 of 9 | **8 of 9** |
    | stopped along | +744 sd 777 | **+485 sd 444** |
    | across, \|mean\| / max | 41 / 79 | **10 / 20** |

    Two sessions of work on the capture, the flare's bank taper and the
    touchdown alpha moved none of this; one constant in the phase *above*
    them moved all of it. **When a phase keeps handing over a state the next
    phase cannot fly, the bug is usually in the handover and not in the
    flying.**

72. **``id()`` is an address, not an identity, and a cache keyed on it served
    one environment's answers to another.**

    ``trajectory.alpha_for_load`` memoises on ``(id(env), generation)``.
    CPython hands a freed object's id to the next allocation, so after a test
    dropped one ``FakeEnv`` and built another, the new one inherited the old
    one's cached angles.  Measured: ``guidance.approach`` returned an angle of
    attack **2.2 degrees below the trim its own floor guarantees** -- an
    assertion that cannot fail by construction, failing.

    It only reproduced in a whole-suite run, never in the single test, which
    is the signature: the first object has to have been collected. Keyed on a
    serial now, which is never reused. Nothing in flight had two environments
    at once, so no flight was wrong -- but "no flight was wrong" was luck
    about object lifetimes, not a property of the code.

73. **The tail limit is not the lever on the tail strike.**

    Four of the eighteen flights of the landing configuration are
    `destroyed in ROLLOUT` losing an `LV-T91` and an elevon first -- the
    engine bell, which is the tail.  The obvious reading is that the flare
    ends too nose-high, and the obvious knob is `TAIL_STRIKE_MARGIN`, which
    turns the measured 19.9 degree strike angle into the cap the flare may
    command.

    0.8 -> 0.65 (a 15.9 degree cap down to 12.9), eight flights on a freshly
    restarted farm: **3 of 8 kept their parts against 6 of 9** at the
    default, with the along-track and the strip unchanged (+924 sd 369, 4 of
    4 inside 35 m).  Worse, not better, and not by a margin worth splitting.

    The measurement that says why is in the logs either way: the last flare
    tick of a wreck and of a survivor read the *same* achieved attitude
    (16.1 and 16.0 degrees, against a 15.9 cap).  Whatever separates them is
    not how high the nose is, and a cap that both of them are already sitting
    on cannot be what distinguishes them.  Lowering it only makes the wing
    carry less at touchdown, which is why it costs flights rather than saving
    them.

74. **The rollout is where the vehicle is destroyed, and nothing about the
    arrival predicts it.**

    The framing that survived a long time is that the landing is a touchdown
    problem -- `qs_plane_inc` "comes apart on contact at 48-53 m/s after a
    textbook approach" -- and every candidate mechanism was therefore a
    property of the arrival.  Measured across 41 flights that came to rest
    within 2 km of the runway midpoint, grouped by whether the flight lost a
    part at all (`./spaceplane/tools/rollsum.py --near 2000`):

    |  | n | sink at handover | touchdown speed | decel to first loss |
    |---|---|---|---|---|
    | kept everything | 32 | -2.1 sd 2.4 | 46.5 sd 9.8 | 12.3 sd 2.8 |
    | lost on contact | 2 | -2.7 sd 5.8 | 41.8 sd 7.7 | -- |
    | lost in rollout | 7 | -2.7 sd 3.0 | 44.6 sd 3.7 | 8.4 sd 3.2 |

    **Three mechanisms, three nulls.**  The sink the flare hands over, the
    speed it hands over, and the deceleration that follows all fail to
    separate the flights that keep the aircraft from the ones that do not.
    The first read of the sink column looked like a result -- across *all* 81
    flights it reads -3.0 intact against +3.3 for the losses -- and it
    dissolves the moment the flights that never reached the field are
    excluded, because that +3.3 sd 14.9 is three `qs_plane_high` crashes
    arriving at 15, 48 and 53 m/s of sink and is not a measurement of a
    touchdown at all.  **Condition on arriving before reading anything about
    the arrival.**

    Two things follow.  The first is CLAUDE.md's rule about a run of
    refutations: three nulls on one mechanism is information about the
    *approach*, not the mechanism, and the approach here was correlating
    across flights the autopilot chose.  What varies between these flights is
    not the arrival; so look instead at what does *not* vary -- what is
    applied identically on every flight regardless of the arrival.

    The second is `run_rollout`'s own docstring, which has computed the right
    answer for the life of the project and been ignored by the code beneath
    it: *"There is 2400 m and the measured touchdown speed is 46-54 m/s,
    which needs 0.44-0.61 m/s^2 -- gear-down drag alone gives about one, so
    this is the least demanding part of the flight."*  The line under it was
    `self.control.brakes = speed < self.cfg.BRAKE_SPEED_M_S` with that
    constant at **200 m/s**, which is "always, flat out, from the first
    tick".  The intact column above is the measurement of what that costs:
    **12.3 m/s^2**, which on 6.9 t is 83 kN -- more than the vehicle's own
    weight -- through two small gear legs, for a requirement of 0.5.

    So the brakes are now commanded by the distance that is left
    (`guidance.brake_fraction`), which is the same shape as boosterland's
    landing burn: `needed = v^2 / (2 * remaining)`, less the drag the
    airframe already has, as a fraction of what the wheels can do.  It is
    feedback rather than a schedule -- as `remaining` shrinks the demand
    rises on its own -- so a vehicle that floated to the far end still gets
    everything, which is the one case the old behaviour was accidentally
    right about.  `BRAKE_SPEED_M_S` goes back to the meaning its comment
    always claimed, a floor under which the brakes go full because the energy
    left is small and nothing else will stop it.

    **This also says why the landing position and the survival are one
    problem and not two.**  Braking gently is only available to a vehicle
    that touched down with runway in front of it, and the chain currently
    puts the wheels down 2.4 km along a 2.4 km runway (`TOUCHDOWN_AIM_M` is
    the far threshold).  On the arrivals that land early the new law asks the
    wheels for nothing at all; on the ones that land at the far end it
    changes nothing, by construction.

75. **The bounding box's z was range-checked and its y was not.**

    `WHEEL_CLEARANCE_MAX_M` rejects an impossible wheel clearance, so a box
    that answers with a usable z beside a garbage y passes as a good
    measurement.  `logs/LOG2384` -- flown by hand -- reports **24.95 m of
    tail behind the centre of mass** on an aircraft that measures 5.65 m from
    its docking port to its engine bell, which is a tail strike angle of
    **5.0 deg** where every other flight on disk reads 18.7 or 19.9.
    `TAIL_STRIKE_MARGIN` takes that to 4.0, and `aim_runway` clamps the
    flare's *pitch* command to it: the phase whose only tool is angle of
    attack, held to four degrees of it, for the whole landing, silently.

    The shape is CLAUDE.md's: a *too large* aft extent produces a *small*
    angle, which sails through a lower bound of 1.0 deg looking conservative.
    A missing answer must not be allowed to look like a good one.

    The check is the parts, the way `_release_nose_brake` finds the nose
    wheel -- the furthest-aft part along the direction the vehicle points,
    which reads ~3.5 m and agrees with the good boxes.  It is used as a
    *veto* on the box rather than as the source: believing it instead would
    take the limit from 4.0 deg straight to 28.8 in one step, on the phase
    that owns the touchdown, and the rotation really happens about the main
    wheels rather than the centre of mass.  A rejected box falls back to
    `TAIL_ANGLE_FALLBACK_DEG`.

    **Scope, honestly: this has fired once in 2400 logs.**  It is a guard
    against a rare hazard and not an explanation of the rollout breakups --
    the first draft of this entry claimed it might be the hidden two-
    population split behind failure 74's nulls, and scanning every log on
    disk refuted that in one command.

76. **The instrument built to contradict a transcribed constant was itself
    measuring the wrong aircraft, and it cried wolf on every flight ever
    flown.**

    `report_airframe` exists because of failure 13 -- *a measurement whose
    only consumer is a hand-copied constant cannot be contradicted* -- and it
    does its job loudly.  Every log on disk carries:

    ```
    airframe DISAGREES: STALL_SPEED_M_S is 48.00 and the swept table
    says 78.14 (63% out) -- one of them is not this aircraft
    ```

    `airframe.measure` says in its own signature that `mass` is *"the mass the
    landing is flown at -- the drained one"*.  It was handed `entry_mass`,
    which with `DRAIN_BEFORE_BURN` (the default since failure 61) returns
    `snap.mass` unchanged -- and at STANDBY that is **14.527 t on `qs_plane`
    against the 6.93 t the wheels arrive at.**  A stall speed goes as the
    square root of the weight:

    | mass | stall from the same table |
    |---|---|
    | 14 527 kg (what is measured) | 75.4 m/s |
    | 6 930 kg (what lands) | **52.1 m/s** |

    So the configured 48 is about **8% low**, not 63% out, and the alarm has
    been wrong -- in the alarming direction -- since it was written.

    **The constant was roughly right and the instrument was wrong, which is
    the more dangerous way round.**  A false alarm that stands long enough
    stops being read; this one was one step from being "fixed" by moving
    `STALL_SPEED_M_S` up to match a number derived at twice the landing
    weight, which would have raised the approach speed, the gate, the flare
    target and the rollout schedule together, all four in the wrong
    direction.  The rule in CLAUDE.md is that a derived table must be re-taken
    after anything that changes the plant; the corollary this cost is that
    **a derived number is also wrong if it is taken at the wrong point in the
    flight**, and "which mass" is exactly that.

    It is now taken twice: in STANDBY at the entry mass, which is the right
    question for the glide, and again on the first APPROACH tick with
    `landing=True`, where `snap.mass` *is* the landing mass and no drain
    model or burn prediction is needed.  The second is the one the landing
    constants answer to, and the log says which is which.

    **What survives once the instrument is honest.**  The touchdowns on
    record run **37.4 to 52.1 m/s against a stall of ~52** -- so a good
    number of them are at or below the speed at which the wing can hold the
    aircraft up, which is consistent with the signature in the handover line:
    `sink=-7.91` at `FLARE -> ROLLOUT` is not a gentle arrival, it is a
    rebound, the vehicle having already hit between ticks.  That is the open
    item, and it is a *flare* question -- the flare bleeds 84 m/s down to the
    forties arresting the sink, and a glider cannot get it back.

77. **"Inclined orbits don't work" is an entry problem, not a landing one,
    and the propagator is told the vehicle holds an angle of attack it
    cannot hold.**

    The framing to discard first: `qs_plane_inc` was recorded as reaching the
    runway 6 of 6 and keeping its parts 1 of 6 -- "a *touchdown* problem and
    not a guidance one".  Twelve fresh flights say the arrival is not
    reliable at all.  The along-track miss at `GLIDE -> HAC`, which is what
    the entry delivers:

    ```
    inc:      +86  +524  +536  +683 +1005 | -3664 -6142 -6934 -11847 -12957 -14822 -14958
    qs_plane: +0 +135 +158 +279 +484 +489 | -1804 -3143 -3181 -3215 -3965 -4047
    ```

    **Bimodal on both saves, and on `inc` the short mode is six to fifteen
    kilometres.**  A landing change cannot be measured against that: two
    batches were started and stopped here for exactly that reason, one on the
    brake law and one on the bank compensation, rather than bank a null taken
    against a disturbance that dominates it.  Landing work belongs on a save
    whose arrival is repeatable -- `qs_entry`, or better a save on final.

    **The solver never knows.**  `slv=ok` on all 272 GLIDE ticks of
    `logs/LOG2401`, a flight that arrives **14.8 km short**.  So
    `SOLVE_MAX_RANGE_ON` cannot help even switched on: the "did not reach"
    branch it hangs off never fires, because the propagation does reach.  The
    prediction is wrong, not absent, and it goes wrong progressively:

    | altitude | predicted miss | commanded/achieved alpha |
    |---|---|---|
    | 54-40 km | +525 .. +475 | 29.9 / 30.4 |
    | 37 km | -2145 | 27.5 / 28.3 |
    | 24.7 km | -4544 | 21.9 / **13.9** |
    | 19.9 km | -11804 | 19.9 / **15.1** |

    The two columns go wrong together.  The vehicle stops holding what it is
    commanded at about 6 kPa, and that is where the prediction starts to
    leave.

    **`tracked_alpha` exists for this and is switched off.**  Its own
    docstring says *"A propagation that believes the command is a propagation
    of a trajectory the vehicle does not fly, which is the error this project
    has now made in five places"* -- and `ALPHA_TRACKING_ON` is `False`, so it
    returns the command unchanged and every propagation in the program, the
    glide's solve and the burn's alike, assumes 100%.

    CLAUDE.md's rule says a table taken before the plant was tuned has to be
    re-taken, so it was -- read straight back off the logs, achieved over
    commanded, binned by dynamic pressure, **25 007 GLIDE ticks**:

    | q (Pa) | median | mean | p25 | n |
    |---|---|---|---|---|
    | 0-1500 | 1.02 | 1.01 | 1.02 | 13250 |
    | 1500-3000 | 1.02 | 1.00 | 1.00 | 3130 |
    | 3000-6000 | 0.84 | 0.86 | 0.81 | 2648 |
    | 6000-9000 | 0.80 | 0.82 | 0.75 | 2083 |
    | 9000-12000 | 0.74 | 0.75 | 0.71 | 3753 |
    | 12000+ | 0.77 | 0.77 | 0.73 | 143 |

    against the configured 1.02, 1.01, 0.89, 0.74, 0.72.  **Within 0.06
    everywhere: the table was right and nobody was using it.**  That is the
    opposite of failure 13's shape and worth naming as its own trap -- there,
    a measurement had no consumer and the constant was wrong; here the
    measurement is correct, has a consumer, and the consumer is behind a flag
    set to `False`.  A disabled correct model and a missing one produce the
    same flights.

    The reversal count correlates with the arrival (5-6 reversals give -12 to
    -15 km, 11-12 give about +500), which is what `CROSS_DEADBAND_MAX_M`
    sets: at 20 m/km and 2000 km to run the deadband saturates at its 40 km
    ceiling, which is **wider than the 20 km of cross-track an inclined entry
    starts with**, so the relay does not fire at all until the range has come
    down. That constant was fitted on a save whose cross-track is small.  It
    is being measured against 10 km as a set variable rather than read off
    the correlation, per CLAUDE.md on failure 30.

78. **The cross-track deadband is not the lever on the arrival, and the
    reversal count is a consequence rather than a control.**

    Failure 77 left the reversal count correlating tightly with the arrival
    (5-6 reversals give -12 to -15 km, 11-12 give about +500), and
    `CROSS_DEADBAND_MAX_M` is what sets the band the relay fires on -- at
    20 m/km and 2000 km to run it saturates at 40 km, wider than the 20 km of
    cross-track an inclined entry starts with.  So: set it, do not correlate
    it (CLAUDE.md on failure 30).  `pairfly.sh`, `qs_plane_inc`, 10 km against
    the committed 40 km, arms swapped across instances every round.

    **The first four flights of each arm looked decisive and it dissolved.**

    ```
    rounds 0-1   A (10 km)  +844  +567  +483 -1430     mean  +116 sd 1023
                 B (40 km) -5802 -10447 -3484 -6692    mean -6606 sd 2891
    round 2      A (10 km) -5783 -8418
                 B (40 km)  +691 -13113
    ```

    At 4 v 4 the separation was complete -- the worst A flight beat the best
    B flight, 4.4 standard errors -- and two rounds later the arms are inside
    a standard error of each other.  **This is failure 27 happening again**,
    and the only reason it was not adopted is that the batch was allowed to
    finish.  Four flights of one arm against four of another is not a
    measurement on this vehicle either.

    **And the knob does not do what the correlation implied.**  The reversal
    counts, by arm and round:

    ```
    A (10 km):  11  11  11  11 |  8   7
    B (40 km):   9   7   9  10 | 10   7
    ```

    Rounds 0-1 the tight band gave 11 every time; round 2 it gave 8 and 7,
    *fewer* than the wide band in the same round.  The deadband influences
    the count weakly and something else dominates it.  The count still
    predicts the arrival -- that relation is not in doubt -- but it is a
    symptom, and tuning the band does not reach it.

    **Where the mechanism actually is, and it is already written down.**  See
    the comment at `Config.BANK_RATE_DEG_S`: a reversal is about seventeen
    seconds of slew at 8 deg/s, and `Steer` is *"a commanded angle of attack
    and bank, held for one propagation"* -- the propagator flies a constant
    lean while the vehicle flies a relay.  Eleven reversals is 138 s of
    rolling in a 700 s glide (20%); seven is 88 s (12%).  The vehicle near
    wings level sinks less and flies further, so the reversal count *is* a
    range term, and it is one the prediction has no representation of at all.
    That is a violation of CLAUDE.md's "the propagator must fly the law the
    vehicle flies", and it explains the bimodality directly: the arrival has
    two modes because the duty cycle has two modes and the predictor models
    neither.

    Both single-bank repairs have been flown and both are worse -- the
    transient bank spikes `long` by +20 km mid-reversal, the intended stop
    bank lands it long -- and that entry already states the answer: *"what
    the propagation wants is the mean of the reversing entry **including its
    duty cycle**, which is neither of the two banks available here."*

    **The design that follows, and the reason it is one quantity and not
    two.**  Keep a time-average of `cos(bank)` over the commanded,
    rate-limited bank -- an EWMA over a couple of reversal periods -- and
    propagate the *effective* lean `acos(mean cos)` rather than the
    instantaneous one.  A single measured number then fixes both known
    artefacts at once: it cannot spike mid-reversal, because the mean does
    not care where in the slew the tick landed, and it carries the duty cycle,
    because that is exactly what the mean of `cos` over a reversing entry
    *is*.  It is self-measuring in the way this project prefers -- a flight
    that holds its lean reports its lean, a flight that reverses constantly
    reports the lower effective bank it is really flying -- so it needs no
    constant beyond the averaging window.  Not flown yet; the constants
    exist (`BANK_RATE_DEG_S`) and the history is already in the log.

    **How big the term is, measured.**  Every `qs_plane_inc` flight on disk
    with an arrival, binned by the reversal count `glidesum.py` reports:

    | reversals | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
    |---|---|---|---|---|---|---|---|---|
    | n | 1 | 4 | 3 | 5 | 5 | 5 | 9 | 3 |
    | mean arrival (m) | -11847 | -13904 | -13142 | -5655 | -4192 | -1537 | **+29** | -5400 |

    **+2137 m per reversal, r = 0.70, r^2 = 0.49 over 35 flights**, and
    monotone from six through eleven.  Half the arrival's variance on this
    save is a quantity the prediction does not represent, and the size of it
    is about what the geometry says it should be: a reversal is
    `2 x BANK_MAX_DEG / BANK_RATE_DEG_S` of slew, and the lift the vehicle
    picks up passing through wings level is the difference between `cos(50)`
    and 1.

    This is a correlation across flights the autopilot chose, so by CLAUDE.md
    it is **not** a sensitivity and the count is not a knob -- which the
    deadband arms above confirm from the other side.  But the response to a
    confounded correlation with a physical mechanism behind it is to *model*
    the quantity, not to steer it, and that is what `GLIDE_BANK_DUTY_ON`
    does: it makes the prediction right for whatever count the flight turns
    out to have, instead of trying to fix the count.

79. **The duty-cycled lean is the right physics and it does not help, because
    the reversals were compensating for an error nobody has found.**

    Failure 78 ends with a design: propagate `acos(mean cos(bank))` rather
    than one constant lean, measured by the vehicle as the ratio of the
    time-averaged `cos` of what it commanded to the time-averaged `cos` of
    the lean it meant to hold.  Built (`GLIDE_BANK_DUTY_ON`), flown on
    `qs_plane_inc`, arms swapped across instances every round.

    **The term engages and is the right size.**  The `duty=` column runs
    1.00 on the stops and peaks at 1.06-1.11 through the reversals; a hand
    estimate for a fifth of the glide near level against a 50 degree lean
    gives 1.07.  So a null here is a null about the *mechanism*, not about
    whether the knob reached the decision -- which is why the column exists.
    It was added only after the first batch had been launched without it and
    killed two flights in: a null on a term that cannot be seen is worth
    nothing.

    | arm | n | arrival | reversals |
    |---|---|---|---|
    | duty on | 6 | -5372 sd 3991 | 9.8 |
    | default | 6 | -6090 sd 5538 | 8.8 |

    Raw difference +718 m, which is nothing.  **Controlling for the reversal
    count the arm effect is -1350 m** -- slightly *worse* -- and the
    regression puts the reversal effect at **+2068 m each**, against the
    +2137 measured independently over 35 flights.

    **Why it goes the wrong way, which was predicted before the batch
    finished.**  A reversing entry flies *further* than a constant lean, so
    the unmodelled reversals make the propagation predict too *short* for a
    reversing flight.  But the flights land short, so the propagator's net
    error is the other way: it over-predicts range and the reversals have
    been quietly offsetting it.  Model the reversals honestly and the
    prediction gets longer, the solve steepens to null it, and the arrival
    goes shorter still.

    This is CLAUDE.md's guard rule in a new costume.  There the argument is
    about a *guard* whose stated reason does not survive inspection; here it
    is an **omission** doing the same job -- the missing term was the only
    thing offsetting a bug nobody has found, and correcting it in isolation
    is a regression.  **A model error that cancels another model error is
    load-bearing, and the pair has to be fixed together or not at all.**

    The first four flights said the opposite, for the record: `-1739, -1974`
    against `-13105, -9177`, which read as a decisive win until the reversal
    counts beside them (12, 11 against 7, 7) showed the arm was simply
    drawing more reversals.  Two arms of four on this vehicle have now
    produced a spurious "decisive" result twice in one session (failure 78).

    **What this leaves.** The propagator over-predicts range and the size of
    it is the 6-15 km the short mode lands by.  `ALPHA_TRACKING_ON` pushes
    the arrival the *other* way (less predicted lift, so the solve asks for
    more range) and measured worse alone -- so the honest next arm is both
    together, two independent corrections whose range effects partly cancel,
    where each alone is worse than neither.

80. **Higher-energy orbits fail by dispersion, not by bias, and the cliff is
    between a 104 km and a 130 km apoapsis.**

    `qs_plane_high` was two anecdotes (+2460 and +6577 m) and "the miss is
    made between the burn and the interface".  Energy is something
    `savegen.py` can *set*, so it was set: a five-arm ladder from the
    committed circular save, `--prograde` 0/20/40/60/80 m/s, four flights
    each, every arm on every instance (`ladder.py`).  `qs_e80` reproduces
    `qs_plane_high`'s orbit exactly -- SMA 732383, ECC 0.0715 -- differing
    only in argument of periapsis, and it fails the same way, so the ladder
    is measuring the real thing.

    | apoapsis | dv | arrival mean sd | on runway | intact | glide s |
    |---|---|---|---|---|---|
    | 80 km | 0 | +412 sd **157** | **3 of 4** | 3 | 598 |
    | 104 km | +20 | +1074 sd **326** | **3 of 4** | 3 | 571 |
    | 130 km | +40 | -3008 sd **5836** | 0 of 4 | 1 | 577 |
    | 157 km | +60 | +3289 sd **1671** | 0 of 4 | 0 | 516 |
    | 185 km | +80 | +6179 sd **2590** | 0 of 4 | 0 | 489 |

    **It is a cliff, not a slope.** Three of four on the runway becomes none
    of four between +20 and +40 m/s, and never recovers.

    **And the quantity that moves is the scatter.**  The arrival is
    repeatable to **sd 157 m** from a circular orbit and to **sd 5836 m** from
    a 130 km apoapsis, in both directions.  That is why no margin fixes it:
    `GLIDE_RESERVE_M` is a constant fitted against one save's sd (failure 68),
    and a sd that moves by a factor of thirty is not something a constant can
    follow.

    **Three mechanisms proposed and refuted inside this one batch**, each
    worth recording because each was plausible:

    - *Peak dynamic pressure.*  The first four flights suggested a ~12 kPa
      threshold.  `qs_e40` has the **lowest** peak q of all five arms
      (10617 Pa) and the **worst** dispersion.  Dead.
    - *The alpha ceiling crossing `SOLVE_ALPHA_MIN_DEG`.*  The ceiling really
      does ratchet below the solve's 20 degree floor on `qs_e80`, and the
      U-shaped range curve makes that a genuine sign reversal -- but a
      `qs_plane` flight crossed to 19.8 and arrived **+408**, while two
      `qs_e80` flights both crossed at 19.6 and arrived +8101 and +2398.
      Neither sufficient nor predictive.
    - *Kerbin's rotation.*  Raised from outside; checked rather than assumed.
      `omega` takes its magnitude from `body.rotational_speed` and *measures*
      its sign (`Environment._measure_omega`), the propagator carries Coriolis
      and centrifugal, and the runway is fixed in the rotating frame.  It is
      also not a differential confound here: burn-to-arrival is **1095-1114 s
      across all five arms**, so the surface turns the same ~193 km under
      every one.  Had the arms differed by 100 s that would have been 17.5 km
      of runway travel, which is the size of the misses being attributed to
      energy -- so this was worth the one command it cost.

    **What survives.** The *glide* shortens monotonically, 598 -> 489 s,
    while burn-to-arrival stays fixed: the coast absorbs the difference and
    the closed loop gets 18% less time as energy rises.  The deorbit range
    meanwhile comes out at **1945-1950 km on every arm** against a
    `DEORBIT_RANGE_MAX_M` of 2300 km, so the search has headroom it is not
    using.  A longer, shallower arc for a high-energy orbit would give the
    glide back its time.  **Indicated, not demonstrated** -- it is one arm
    (bias the deorbit range up, re-fly `qs_e60`), and the last thing this
    file needs is a fourth mechanism asserted from the same batch that
    refuted three.

81. **The airframe has no landing margin, and that is why no touchdown
    parameter has ever predicted the breakups.**

    Failure 74 tried three touchdown parameters against survival -- sink at
    the handover, touchdown speed, deceleration -- and all three came back
    null once the flights that never reached the field were excluded.  Three
    nulls were read as "look at what does not vary" and pointed at the
    braking.  There is a simpler reading that was never checked, and it is in
    the part configs:

    | part | `crashTolerance` |
    |---|---|
    | LY-10 Small Landing Gear | **50 m/s** |
    | Structural Wing Type A | **15 m/s** |
    | Elevon 2 / Elevon 4 / control surfaces | **15 m/s** |
    | Mk1-3 command pod | 20 m/s |

    The vehicle touches down at **40-60 m/s**.  The gear is built for that.
    **Nothing else on the aircraft is, by a factor of three or more** -- and
    the parts that go first in every breakup on record are exactly the 15 m/s
    ones: `Structural Wing Type A`, `Elevon 2`, `Elevon 4`.

    So the landing has no tolerance at all for anything but a clean
    wheels-first, wings-level touchdown.  Any contact by a wing, an elevon or
    the tail is not a hard landing, it is instant destruction.

    **That explains the shape of the null results rather than adding a fourth
    mechanism to them.**  Survival depends on a near-binary *geometric* event
    -- did a non-wheel part touch -- which small variations in attitude, bank
    and terrain decide.  A binary outcome driven by geometry will not
    correlate with any bulk continuous parameter of the touchdown, which is
    precisely what failure 74 measured and could not explain: intact -2.1 m/s
    of sink against broken -2.7, 46.5 m/s against 44.6, deceleration equal
    inside a sigma.  **The discriminator was never going to be in those
    columns.**

    It also re-reads failure 73.  `TAIL_STRIKE_MARGIN` 0.8 -> 0.65 made
    things *worse* and the last flare tick of a wreck and a survivor read the
    same attitude (16.1 against 16.0) -- consistent with the cap not being
    the binding constraint and the real event being a contact the cap does
    not describe.

    **What follows is a craft change, not a control change**, and it is the
    one the spaceplane notes already anticipate ("after those, a craft file
    of one's own"): taller or larger gear so the wing and elevons clear the
    runway at any attitude the flare can produce, or a touchdown slow enough
    that a 15 m/s part can survive brushing it -- and 15 m/s is below the
    stall, so it is the gear.  Until then the autopilot is being asked to be
    perfect on every flight because the airframe gives it no margin to be
    anything else, and *that* is the honest reason the touchdown has resisted
    three sessions of guidance work.

82. **Disabling an unmeasured change is not the neutral act it looks like.**

    `BRAKE_FOR_DISTANCE` (failure 74) and `APPROACH_BANK_COMPENSATION` (the
    `1/cos(bank)` the speed loop never applied) were both written, tested
    offline, and committed as defaults -- and neither had been flown, because
    both of their batches were stopped when it became clear the entry's
    arrival scatter swamps any landing effect (failure 77).  They were
    therefore turned *off*, on CLAUDE.md's first rule: a change argued rather
    than measured is a hypothesis.

    That reasoning is right and the action was wrong.  Measured afterwards on
    `qs_plane`, eight flights an arm, arms swapped across instances every
    round:

    | | intact | on the 35 m strip | \|cross\| at rest |
    |---|---|---|---|
    | both on | **4 of 8** | **6 of 8** | **27 m** |
    | both off (the "safe" default) | 2 of 8 | 3 of 8 | 66 m |

    Every metric doubles the same way, and the cross-track column is the one
    the mechanism predicted in advance: uncompensated bank hands the wing
    `cos(40) = 77%` of the load the speed loop computed, in exactly the
    moments the approach has decided it is high, so it dives, arrives fast,
    and the lateral capture runs out of time.

    **The general point.** "Off until flown" *feels* like the conservative
    choice and it is not -- it is a different untested configuration, chosen
    over another untested one on the strength of which came first.  There is
    no neutral default here; there is only the one that has been measured.
    The correct response to an unflown change is to fly it, and if the farm
    cannot answer the question yet (failure 77), to say so and leave the
    change where the reasoning put it rather than swap in the opposite
    hypothesis and call that caution.

    Worth reading beside failure 21 and boosterland 17 -- a guard whose
    stated reason does not survive inspection may still be enforcing
    something real.  Here it is the same lesson with the sign reversed: a
    change whose *justification* is only an argument may still be carrying
    the flight.

83. **The landing gear is not the constraint, in either direction.**

    Failure 81 established that the airframe has no landing margin -- the
    gear tolerates 50 m/s of contact and the wings, elevons and control
    surfaces tolerate **15** -- and inferred that the destruction must be a
    geometric contact event the autopilot cannot reach.  The obvious
    follow-up is the suspension, and the save persists it per-part
    (`springTweakable`, `damperTweakable`, `autoSpringDamper`), so it can be
    varied without touching `GameData` and diverging the instances.

    Two variants, each 8 flights against 8 of stock, `pairfly.sh` with
    `SAVE_A`/`SAVE_B` so every save flies twice on every instance:

    | gear | intact | on runway |
    |---|---|---|
    | spring x1.6, damper x2.5 (stiffer) | **1 of 8** | -- |
    | **stock** | **7 of 8** | 7 of 8 |
    | spring x0.5, damper x2.0 (softer) | 4 of 8 | 7 of 8 |

    **Stock wins decisively and both modifications hurt.**  Stiffer was
    reasoned from clearance (less compression keeps the elevons off the
    tarmac) and softer from compliance (absorb the contact instead of
    transmitting it); the airframe prefers neither.  So the suspension is not
    what decides survival, and failure 81's *explanation* is retired even
    though the part tolerances it rests on are facts.

    **A farm-state explanation was proposed here and does not hold up.**
    The same configuration on `qs_plane`, 8 flights each, gave 4 of 8 intact
    in one batch and 7 of 8 in the next, and the second was read as "a fresh
    farm is worth as much as the code" on swap figures of 5.6 GB against
    0.8 GB.  **The 0.8 GB was measured at the wrong moment** -- after
    `stop.sh` and *before* the instances loaded.  Measured once four
    instances are actually up, a freshly started farm sits at **3.1 GB** and
    climbs to about 5.6 over a session: a real drift, but a much smaller one
    than the comparison implied.

    And 7 of 8 against 4 of 8 at n=8 is p ~ 0.28.  Ordinary variance accounts
    for it without any farm effect at all, and the honest reading is that
    this project's landing outcome has a binomial spread wide enough that two
    eight-flight batches of *the same configuration* differ by that much
    routinely.  Which is the same lesson as failures 27 and 78, arriving by a
    third route: **an eight-flight batch resolves a factor of two, and
    nothing finer.**  `pairfly.sh`'s round-by-round interleaving protects a
    *comparison* from drift; nothing protects an absolute rate, and an
    absolute rate quoted from one batch is a number with +-15 points on it.

84. **The aim is still not a lever, and the S-turn stop condition is not why.**

    Failure 78 proposed that `TOUCHDOWN_AIM_M` behaves backwards (failure 68:
    2400 -> 400 landed it *later*) because `APPROACH_SCURVE_STOP_M` measures
    from the aim, so moving the aim nearer also moves the point where
    dissipation stops further out.  The repair was to express that stop as a
    *time to the flare* (`APPROACH_SCURVE_STOP_BY_TIME`, calibrated at 7.0 s
    to be neutral at the old operating point) and then move the aim.

    Flown on `qs_plane`, 8 an arm, balanced, fresh farm:

    | arm | intact | on runway | stopped along |
    |---|---|---|---|
    | aim 1200 + time-based stop | **3 of 8** | 7 of 8 | +685 |
    | committed defaults | **7 of 8** | 7 of 8 | +930 |

    It moves the landing 245 m earlier and **halves the survival**.  The
    cross-track says why: the four wrecks in the changed arm came to rest at
    **+125, +135, +130 and +145 m**, against a 35 m strip.  Stopping the
    weave on a time leaves the vehicle further off the centreline than
    stopping it on distance-to-aim did, and this airframe does not survive
    that (failure 81's part tolerances are still facts even though the gear
    was not the constraint).

    So failure 68's verdict stands on its own terms: **the aim is not a lever
    on where the wheels touch**, and the mechanism proposed for why is wrong
    too.  `APPROACH_SCURVE_STOP_BY_TIME` stays at `False`.

    **What this leaves as the committed state**, measured across two
    independent eight-flight batches of the same configuration on `qs_plane`
    (fingerprint `6debf0c5`), which agreed at 7 of 8 each:

        n=16   intact 14 (88%)   on the runway 14 (88%)   on the strip 12 (75%)
        stopped along  +939 m  sd 455        (the runway is +-1200 m)

    That is the working configuration.  It lands late -- +939 of a +-1200 m
    runway, which is the last quarter of the tarmac and is exactly the
    complaint this session opened with -- and nothing tried here moved it
    earlier without costing more than it bought.  The aim, the S-turn stop,
    the brake law and the gear have all now been measured against it.

85. **Failure 79's pre-registered "both together" arm is refuted, and it
    fails by the mechanism it was meant to model.**

    Failure 79 ends with a plan: `GLIDE_BANK_DUTY_ON` alone is worse because
    it corrects one model error while another, unfound, error was cancelling
    it, so "the honest next arm is both together" -- the duty-cycled lean
    *and* `ALPHA_TRACKING_ON`, two corrections whose range effects partly
    cancel.  Flown: `pairfly.sh`, `qs_plane_inc`, 8 an arm, halves swapped
    every round, fresh farm, fingerprint `6debf0c5`.

    | arm | arrival at `GLIDE -> HAC` | reversals | intact | stopped |
    |---|---|---|---|---|
    | duty + tracking | **-11664 sd 10700** | 7.4 | 2 of 8 | 4 within 400 m |
    | committed defaults | **-4348 sd 4400** | 9.0 | 0 of 8 | 3 within 1.1 km |

    The mean is 7.3 km worse (t = 1.8 at n=8, so not decisive on its own)
    but the **scatter is 2.4x worse**, which at this n is the factor-of-two
    a batch can resolve: the arm produced four flights at -13.9, -22.8,
    -23.1 and -24.9 km and the control produced none beyond -9.1.

    **Why, and it is the same quantity both failures are about.**  The
    reversal count is monotone against the arrival *within each arm of this
    batch*, which is the third independent measurement of it:

    ```
    defaults    6 rev -9120 | 8 rev -8977 -8288 -6660 | 9 rev -3179 | 11 rev +520 +481 +442
    duty+track  4 rev -13886 | 6 rev -24923 -23077 | 7 rev -22785 | 8 rev -2171 | 9 rev -2965 -2459 | 10 rev -1042
    ```

    and **the arm moved the count down**, 9.0 to 7.4.  Modelling the duty
    cycle makes the propagation of a reversing entry longer, the solve
    steepens to null it, and the vehicle both flies shorter *and* reverses
    less -- the two effects compound instead of cancelling.  Failure 79
    predicted the first half and the second half is new.

    **What this closes.**  Three attempts now on one mechanism -- the glide's
    lift-and-lean model: `ALPHA_TRACKING_ON` alone (77), `GLIDE_BANK_DUTY_ON`
    alone (79), both together (here).  By CLAUDE.md's own rule that is the
    signal to change the method rather than try a fourth value, and the
    method to change is *what is being corrected*: every one of these tries
    to make the prediction right, and the prediction is re-solved every
    second by a loop that cannot use a correction it has no authority to
    act on.  By 22 km on a short flight the bank is at 0.9 degrees, the
    angle of attack is pinned, and `verified`'s hold branch is flying
    whatever was commanded when the shortfall began -- the state
    `SOLVE_MAX_RANGE_ON`'s comment describes and measured 0.6 standard
    errors against.

    **What the batch says to try instead.**  `GLIDE_RESERVE_M` is the one
    mechanism this project has measured to *work* on exactly this failure,
    and its own comment says why -- "open-loop and therefore early, which on
    this problem beats being correct and late".  It was cut 4000 -> 500
    because the entry on `qs_plane` now scatters 171 m.  On `qs_plane_inc`
    it scatters 4.4 km and is centred 4.3 km short, so the reserve is
    sized against the wrong save.  A margin is a fitted constant with a
    standard deviation baked into it (CLAUDE.md), and the two saves do not
    share one.

86. **On a high-energy orbit the arrival is set by the *delivered* dv, at
    3 km per m/s -- and the aim knob that ought to set it is disconnected.**

    Failure 80 left `qs_e60` failing by dispersion with three mechanisms
    refuted and one indication (the glide loses 18% of its time).  Two
    eight-flight `pairfly.sh` batches on `qs_e60` this session say what it
    actually is, and the first of them says it by coming back null.

    **`DEORBIT_CENTRE_BIAS_M` does not reach the decision.**  Arm
    `DEORBIT_CENTRE_BIAS_M=6000` (positive aims short) against committed
    defaults, 8 an arm:

    | arm | stopped along | arrival at `GLIDE -> HAC` |
    |---|---|---|
    | centre bias 6000 | +1292 sd 2050 | +3.4 km |
    | defaults | +1578 sd 1470 | +2.6 km |

    286 m, in the wrong direction, on a 6 km aim.  And the log says why in
    one column, which is the whole reason that column exists: **`solved dv`
    is 96.1-96.5 m/s on all sixteen flights, in both arms.**  The knob moved
    nothing because the search is scoring `deorbit_centring` and arriving at
    the same burn either way.  CLAUDE.md's failure-10a rule, and the third
    time this project has confirmed the aim is not a lever -- now including
    the case where the glide is *saturated*, which was the reason to expect
    it would be.

    **What does set the arrival is the dv the engine actually delivered**,
    and it is not the dv that was solved:

    ```
    delivered 95.8-95.9  ->  +3241 +3647 +4120 +4244 +5003 +3771
    delivered 96.1-96.3  ->  +2689 +3392 +3419 +3640 +4121 +4184 +4975
    delivered 96.7       ->   +604
    delivered 97.5       ->   -238
    delivered 98.2       ->  -3147
    ```

    **-3.06 km of arrival per m/s of delivered dv**, monotone over 2.4 m/s
    of spread, on flights whose *solved* dv agreed to 0.4 m/s and whose
    decision-time windows were identical to a kilometre (1907-2002 km, gate
    at 1946, -10 to -26% off centre).  So the scatter is not in the search,
    the pass, or the glide: it is cutoff scatter on a 96 m/s burn, amplified
    by a sensitivity three times steeper than anything the design allows for.

    And the *bias* is the same quantity: at the solved dv the arrival is
    +3.9 km, so the solution is about **1.3 m/s too small**.  The two
    flights that overshot their own solution by 1.4 and 2.0 m/s are the two
    that arrived at -238 and -3147, and one of them
    (`logs/LOG2576`, +420 along) is one of only two flights in sixteen to
    keep all 23 parts.  Arrivals inside about +-1 km land intact on this
    save; everything past +2 km leaves the cone **out of height** 4-5 km
    from the gate (`conesum.py`: surplus -469 to -993 on thirteen of
    sixteen) and the approach never gets a vote.

    Why the burn is loose here and tight on `qs_plane`: 96 m/s against
    32 m/s at the same thrust-to-mass, closed at a `DEORBIT` tick of
    0.10-0.30 game-seconds, and at 17.8 m/s^2 a 0.30 s tick is 5.3 m/s of
    dv.  `qs_plane` delivers 31.6-31.7 against a solved 32; `qs_e60`
    delivers 95.8-98.2 against a solved 96.

    **The lever that is connected.**  `DEORBIT_LONG_BIAS_M` is shared by the
    search and the stop test (`deorbit_aim`), and on the authority-window
    path the search ignores it -- so it reaches the *stop test* alone, which
    is exactly the quantity that was just measured to matter.  Failure 33
    recorded that it "reaches only the burn's stop test" as a limitation; on
    this save that is the whole mechanism.

87. **The bank reversal count is a symptom of saturation, and its sign is
    not the same on every save -- which retires the range-term reading of
    failures 78, 79 and 85.**

    Three sessions have read the reversal count as a *range term*: the
    vehicle spends 12-20% of the glide near wings level, sinks less there
    and flies further, so more reversals means a longer flight.  That
    reading is supported on `qs_plane_inc` (+2137 m per reversal over 35
    flights, failure 78) and on `qs_e40` (7-8 reversals give -8.0 km,
    10-11 give +0.9 to +3.1).

    It is **reversed** on `qs_e60`, measured this session over sixteen
    flights of one configuration:

    | reversals | 8-9 (n=13) | 11 (n=2) | 13 (n=1) |
    |---|---|---|---|
    | arrival | +2689 .. +5003 | -3147, +604 | -238 |

    More reversals, *shorter*.  A range term cannot change sign between two
    entry states of the same vehicle, so it is not one.

    **What is the same on all of them.**  A glide that ends up on target
    reverses freely; a glide that saturates stops reversing, because the
    bank is pinned at a stop and the sign test is the only thing that could
    move it.  `qs_plane_inc`'s short flights pin at **bank 0.9** trying to
    stretch (`logs/LOG2461`, fifty consecutive ticks, arrival walking from
    -1.9 to -4.4 km) and `qs_e60`'s long flights pin at **bank 70** trying
    to shed (`logs/LOG2452`, -70.0 / +67.8 / +66.2 on the last three
    samples, `long` climbing +507 -> +4757).  Both pinned states reverse
    less, and they miss in opposite directions -- which is precisely the
    sign reversal above.

    So the count is a *diagnostic of saturation*, and a good one: it is
    11-12 on every `qs_plane` flight, where the arrival scatters 157 m.
    What it is not is a control, or a term the propagator is missing.
    Failure 78 said "consequence rather than control" and then spent two
    batches modelling it as a term anyway (79, 85); this is the third
    reading and the one that survives both signs.

88. **The deorbit aim *is* a lever, and every null against it was measured at
    a seventh of the scale the gain requires.**

    Failure 86 left `qs_e60` arriving +3.9 km long with the aim knobs
    apparently disconnected: `DEORBIT_CENTRE_BIAS_M=6000` moved the landing
    286 m and left `solved dv` unchanged to 0.4 m/s, and
    `DEORBIT_LONG_BIAS_M` -3000 -> -7000 (8 an arm, `pairfly.sh`) moved the
    delivered dv not at all -- A +2310 against B +1645, another null.  Three
    nulls, so by CLAUDE.md the thing to change is the method.  What changed
    was asking **how big the knob's gain actually is**, which nobody had
    computed:

    - the *window* moves **-19.7 km per m/s** of dv (offline, both corner
      pairings, at every energy in the ladder), so a centre bias expressed
      in metres buys dv at 1/19700 of itself;
    - the *arrival* moves **-3.06 km per m/s** (failure 86, measured in
      game over 2.4 m/s of cutoff spread).

    So **one metre of arrival costs 6.4 m of `DEORBIT_CENTRE_BIAS_M`**, and
    every test this project has run on the aim used 2.5-17 km, which buys
    0.4-2.6 km of arrival -- at or under the noise of an eight-flight batch
    every time.  The knob was never disconnected.  It was geared down by a
    factor nobody had measured, and "the aim is not a lever" is what a
    correct knob looks like when it is read at the wrong scale.

    **Flown at the right scale.**  `DEORBIT_CENTRE_BIAS_M=28000` against
    committed defaults, `qs_e60`, 8 an arm, halves swapped every round:

    | arm | delivered dv | arrival at `GLIDE -> HAC` | cone surplus |
    |---|---|---|---|
    | bias 28000 | 97.4 m/s | **-48 sd 3040** | -401 .. +75 |
    | defaults | 96.2 m/s | **+3939 sd 3100** | -1013 .. +499 |

    -3987 m, t = 2.6, and the delivered dv moved +1.2 m/s against a
    prediction of +1.1 made before the batch.  The arm holds the two
    flights that stopped *on* the runway (-1642 and -1277, 23 parts each).

    **And the batch says where to aim, which is not zero.**  `conesum.py`'s
    surplus is the whole outcome on this save and it is a clean function of
    the arrival:

    ```
    arrival  -5064 -5029 -2957 |  +526 | +1608 +1636 +1913 +2234 | +4294 .. +5312
    surplus    +14   +75  +499 |  +299 |  -360  -344  -353  -401 |  -718 .. -1013
    outcome  intact intact intact| lost | 6 of 23 parts, stopped +3000 | lost
    ```

    Zero surplus is the cliff: above it the cone rolls out and the vehicle
    lands, below it the cone exits **out of height** 3-6 km from the gate
    and the approach never gets a vote.  The arrival that buys surplus on
    `qs_e60` is about **-2500 m**, so the aim wants about 44000 rather than
    28000.

    **What is left, and it is one number.**  The residual scatter in the
    28000 arm is not continuous: delivered dv comes out at 97.0-97.3 (six
    flights, arrivals +526 to +2234) or 98.2-98.4 (two flights, -5064 and
    -5029).  One extra tick of the burn's terminal low-throttle trim is
    1.2 m/s, and 1.2 m/s is 3.7 km of arrival.  **The burn's cutoff
    quantisation is now the whole of the high-energy dispersion**, which is
    a far more tractable statement than failure 80's "sd 157 m to sd
    5836 m and no margin reaches it".  `qs_plane` delivers 31.6-31.7 m/s
    against a solved 32 and does not have this problem, because its burn is
    a third the size and its glide has the authority to absorb what is left.

89. **`qs_e60` lands, 8 of 8 on the runway, and the target was the reserve
    all along.**

    Failure 88 established the aim's gain and left the sweet spot at about
    -2500 m of arrival, read off a bracket flown between 44000 and 67000 of
    `DEORBIT_CENTRE_BIAS_M`.  **That reading was contaminated and the number
    is wrong.**  Those batches ran on a farm that had been up two and a half
    hours with **10.6 GB in zram**; CLAUDE.md's farm rule says restart
    between sessions and this is what ignoring it costs -- not a reversed
    result, but a mis-located optimum, because the *comparison* is protected
    by `pairfly.sh`'s interleaving and the *absolute level* is not.

    Restarted (swap 4.0 GB), one more bracket point, `qs_e60`, 8 an arm,
    halves swapped every round:

    | | stopped along | on the runway | intact | arrival | cone surplus |
    |---|---|---|---|---|---|
    | `DEORBIT_CENTRE_BIAS_M=51000` | **+793 sd 123** | **8 of 8** | **6 of 8** | +131..+531 | +496..+499 |
    | committed defaults | +152 sd 3350 | 1 of 8 | 0 of 8 | +4000..+5064 | -465..-751 |

    Eight of eight inside +585..+988 on a +-1200 m runway, every one inside
    96 m of the centreline, **sd 123 m** -- tighter than `qs_plane`'s own
    committed configuration manages (+939 sd 455).  This is the entry state
    failure 80 recorded as 0 of 4 onto the runway with "what grows is the
    scatter, not the bias -- so no margin reaches it".

    **And the target is not a fitted number.**  The winning arm arrives at
    +131 to +531 m, which is `GLIDE_RESERVE_M` -- the glide's own design
    target -- and the cone surplus it earns is +496 to +499, saturated
    positive.  The defaults arrive 4.5 km past the reserve and the cone
    exits *out of height* every time.  So the rule is "aim until the
    arrival lands on the reserve", and only the *bias that achieves it* is
    per-state.

    **What must not be done with this.**  51000 is a calibration for one
    orbit and committing it as a default would put `qs_plane` 6-7 km short
    (2.2 m/s on a 32 m/s burn).  Fly it with `--set` on high-energy entry
    states, and read failure 88's gain before choosing a value for a new
    one: compute the bias offline from `deorbit_solution` at two biases,
    then check the first flight's arrival against +500 and scale.

    **What would retire it.**  The bias is standing in for the propagator's
    range error on this state, in exactly failure 19's sense -- a missing
    model wearing a constant's clothes.  The measurement that would replace
    it exists and is unused: the propagator predicts the interface crossing
    during the coast, minutes before the glide begins, and nothing ever
    compares that prediction with what arrives.

90. **`qs_plane_inc`'s bimodality is decided before the glide starts, and the
    first glide tick already knows which mode the flight is in.**

    `GLIDE_RESERVE_M` 500 -> 4000 on `qs_plane_inc`, 8 an arm, `pairfly.sh`,
    fresh farm -- the mechanism failure 85 nominated, and the one this
    project has previously measured to *work* on a short arrival:

    | arm | arrival | stopped along | on the runway | parts kept |
    |---|---|---|---|---|
    | reserve 4000 | -1506 | **-839** | 4 of 8 | 23, 16, 16, 3 |
    | committed (500) | -6504 | -4641 | 4 of 8 | 5 |

    **+5.0 km of arrival**, the stopped distribution centred rather than
    4.6 km short, and the first intact landing this save has produced in
    two sessions.  It is a real improvement and it is **not** a fix: the
    arrival is still bimodal (`-92 +1324 +3983 +4035` against
    `-2156 -2555 -6546 -10037`), because a uniform shift cannot close a gap
    between two modes.

    **Where the modes are made, which is the useful part.**  The `long=`
    column on the *first* GLIDE tick, at 58 km and Mach 7.2, predicts the
    arrival 600 seconds later, on the committed arm:

    ```
    first-tick long   -7707  -1946   -530  +1024 | +3077  +3216  +3353  +4403
    arrival          -15017 -12119 -11875 -12115 |  +513   +118   +479  -2015
    ```

    A cliff between +1024 and +3077, and nothing in between.  So the
    divergence is **not** in the glide, the bank relay, or the reversal
    count -- it is already present in the state the coast hands over, and
    the glide is merely a threshold detector on it.  Twelve kilometres of
    spread at the interface, from one save, on flights whose *delivered* dv
    agrees to 0.1 m/s (`logs/LOG2673` +3216 and `logs/LOG2680` -530 both
    delivered 42.8 against a solved 43.0-43.1).

    That rules the burn out as well, on this save, which is the opposite of
    `qs_e60` (failure 86, where the burn is the whole of it).  What is left
    between the two is the **coast**: minutes of vacuum flight with the
    engine off in which nothing is commanded, nothing is logged but
    telemetry, and no prediction is ever checked against what arrives.
    That is where to instrument next, and `DIAG_INTERFACE` already exists
    to do it.

91. **The inclined save's bimodality was the deorbit's own loop rate, and the
    governor was being fed a mean where its docstring says a tail.**

    Failure 90 localised `qs_plane_inc`'s two modes to the state the coast
    hands over -- the first GLIDE tick's `long=` already knows which mode a
    flight is in, 600 s early -- and ruled out the burn's delivered dv and
    the glide alike.  The column that separates them is the one CLAUDE.md
    says to read before comparing two logs, and it separates them
    *completely*.  Every `qs_plane_inc` flight on disk with an arrival,
    sorted by the achieved `DEORBIT` interval:

    ```
    0.10-0.21 game-s/tick (n=18)   arrival -2555 .. +4035, |arrival| <= 2.6 km on 16
    0.24-0.32 game-s/tick (n=17)   arrival -6546 .. -24923, every one short
    ```

    Nothing between 0.21 and 0.24, and nothing in the good group past
    +4 km.  A burn at 17.8 m/s^2 commanded every 0.3 game-seconds instead of
    0.1 is 5.3 m/s of dv between chances to stop.

    **Why the governor allowed it, and it is a two-filter bug.**
    `ScaleGovernor` keeps a decaying *maximum* of the tick cost, and its
    comment is explicit: "Not the mean: what binds is the tail."  It was
    being handed `LoopRate.busy`, which is an exponential average with
    `FORGET = 0.75`.  Two filters in series, and the second never sees what
    the first removed.  `DEORBIT`'s search ticks cost 50-80 ms and its
    waiting ticks 3-12; while the phase waits for its pass the average falls
    to the cheap ones, the scale ramps to the 6x ceiling
    (`1.00x -> 1.45x -> 2.21x -> 3.37x` in `logs/LOG2445`), and the burn then
    ignites at whatever the ramp reached.  The plugin re-reads the scale file
    twice a second, so a slowdown commanded on the burn's first tick arrives
    three game-seconds late at 6x -- more dv than the whole burn.

    **Asking earlier does not fix it** and was tried first:
    `DEORBIT_PACE_WHOLE_PHASE=True` requests the burn's interval from the
    moment the phase is entered, and the coarse mode survives it unchanged
    (arrivals -12199, -8846, -10428 at 0.29-0.30 in the arm).  Of course it
    does -- the interval was never the problem, the cost estimate was.

    **`GOVERN_ON_PEAK` governs on `LoopRate.peak`, the phase's worst tick,
    undecayed.**  `pairfly.sh`, `qs_plane_inc`, halves swapped every round:

    | arm | DEORBIT interval | arrival | on the runway |
    |---|---|---|---|
    | `GOVERN_ON_PEAK=True` | **0.10-0.14, every flight** | **+199 sd 420** | 4 of 6 |
    | committed defaults | 0.10-0.31, bimodal | -4378 sd 5369 | 4 of 8 |

    **The coarse mode is gone and the arrival scatter falls thirteenfold**,
    for 5% of throughput (mean 5.08x against 5.34x).  This is the entry
    state that was "0 of 15, every one destroyed 60-90 km short" two
    sessions ago and "bimodal, +86..+1005 against -3664..-20562" one
    session ago; the bimodality was never in the entry at all.

    **What it does not fix**, and the reason `qs_plane_inc` is still not a
    landing: the vehicle still comes apart on contact, 0-3 parts kept on
    most flights, from flare states indistinguishable from `qs_plane`'s.
    Failure 81 stands.

    **The general rule.** *A governor must be driven by the worst tick of
    the phase it governs, and any smoothing upstream of it is the bug.*
    Read beside failure 63, which is the same lesson before anyone had
    built the governor: the log now says what interval flew each phase, and
    this is the first time that column has been used to *find* something
    rather than to disqualify a comparison.

92. **The same loop-rate signature is on the high-energy save, which
    re-reads failures 86 and 88 without overturning them.**

    Failure 91 found the achieved `DEORBIT` interval to be the whole of
    `qs_plane_inc`'s bimodality.  Every `qs_e60` flight of this session
    (n=96, five bias arms) carries the same signature, in the same
    direction:

    | bias | tick 0.10-0.12 | tick 0.24-0.31 |
    |---|---|---|
    | 0 | +3238 .. +5511 (n=34) | -3147 .. +1885 (n=8) |
    | 28000 | +1218 .. +2723 (n=11) | -5064, -5029, +526 (n=3) |
    | 44000 | +322 .. +1288 (n=6) | -5794, -5126 (n=2) |

    A coarse tick overshoots the burn's cutoff -- delivered dv 97.5-98.2
    against 95.7-96.1 at a fine one -- and 1.5 m/s is 4.5 km of arrival.
    So failure 86's "-3.06 km per m/s of delivered dv" is right and its
    attribution of the spread to "cutoff quantisation" is right in
    mechanism but wrong in cause: the quantisation is the *time scale*, not
    the throttle trim, and `GOVERN_ON_PEAK` removes it.

    **Failure 88 and 89's bias is unaffected.**  The 51000 arm that landed
    8 of 8 was already 7 of 8 at a fine tick, and at a fine tick the
    committed defaults arrive +4000 to +5000 every time.  The bias is
    answering the propagator's range error, which is a different quantity
    from the loop rate and does not go away with it.

    **One discontinuity worth recording before someone extrapolates the
    gain.**  The bias-to-dv relation is linear to about 51000 and then is
    not: at 67000 the search jumps branch and delivers **109.7-112.5 m/s**
    instead of ~98, arriving -9.5 to -10.8 km on all eight flights.  So the
    6.4 m-per-metre gain of failure 88 holds over roughly one and a half
    m/s of dv and no further; past that, re-measure rather than scale.

93. **Confirmed, with both fixes in: `qs_e60` is 8 of 8 on the runway and
    8 of 8 intact.**

    `GOVERN_ON_PEAK` committed as the default (failure 91), and
    `DEORBIT_CENTRE_BIAS_M=51000` on top of it (failures 88-89).  An
    independent `pairfly.sh` batch on `qs_e60`, 8 an arm, halves swapped
    every round:

    | arm | stopped along | on the runway | intact |
    |---|---|---|---|
    | `--set DEORBIT_CENTRE_BIAS_M=51000` | **+849** (+203..+1162) | **8 of 8** | **8 of 8** |
    | committed defaults | +1189 (-4777..+4192) | 0 of 8 | 0 of 8 |

    Across the two batches of this configuration: **16 flights, 16 on the
    runway, 14 intact.**  The mean *distance* barely differs between the
    arms and that is the point -- the defaults are 0 of 8 because they
    scatter ±4 km, not because they are biased.

    This is the entry state failure 80 measured at 0 of 4 on the runway with
    "what grows is the scatter, not the bias -- so no margin reaches it".
    Both halves of that were true and neither was the obstacle: the scatter
    was the time-scale governor and the bias was the aim's unmeasured gain.

94. **A near-miss worth the same page as a failure: `SOLVE_ALPHA_MIN_DEG` is
    right, and the offline propagator says it is wrong.**

    Chasing `qs_plane_inc`'s short mode, `logs/LOG2461` shows the glide
    pinned at alpha 21.8 and bank 0.9 from 22 km down while the arrival
    walks out to -4.4 km -- saturated against the solve's alpha floor.  The
    obvious question is whether the floor is in the right place, and the
    offline propagator answers it clearly.  Range against alpha, wings
    level, propagated to the runway from eight points down the entry:

    | start | peak range at alpha |
    |---|---|
    | 70 km, M 7.2 | **16** |
    | 57 km, M 7.1 | **16** |
    | 45 km, M 6.0 | **16** |
    | 32 km, M 4.9 | **16** |
    | 25 km, M 3.7 | **16** |
    | 21 km, M 2.7 | **14** |

    Monotone falling above 16 at every altitude, and no interior *minimum*
    at 20 anywhere -- so the floor at 20 sits past the peak and costs 3-5 km
    of reachable range per probe.  That is a clean, quantitative, and
    entirely wrong result, and the arm was one command from being flown.

    **What stopped it was reading the same question off the game.**  `ld=`
    against achieved alpha from 96 flights, binned by altitude so the two
    are not confounded:

    | altitude | 12 deg | 16 | 20 | 24 | 28 | 32 |
    |---|---|---|---|---|---|---|
    | 20-25 km | 0.71 | 0.79 | **0.85** | -- | -- | -- |
    | 25-30 km | 0.70 | 0.78 | 0.85 | **0.87** | -- | -- |
    | 30-35 km | -- | -- | 0.86 | **0.88** | 0.88 | -- |
    | 35-40 km | -- | -- | 0.91 | **0.93** | 0.88 | 0.80 |
    | 40-50 km | -- | -- | -- | -- | **1.00** | 0.85 |

    L/D *rises* to alpha 20-26 and falls below it.  The propagator's peak at
    16 is an artefact of `spaceplane/tests/fakeplane`'s tables, which is failure 13
    exactly: `planeprobe` over-reads subsonic lift ~1.8x, nothing reads the
    table back except a human, and the error survives because it is never
    contradicted.

    **The floor stays at 20.**  Recorded because the offline result is
    persuasive, reproducible, and will be found again by whoever next looks
    at a saturated glide -- and because it is a worked example of CLAUDE.md's
    first cross-cutting rule catching something *before* the batch rather
    than after it.  The check cost two shell commands against the logs.

95. **`DEORBIT_WINDOW_CORNERS_FIXED` is unflown, and it cannot be flown
    alone.**

    Looked at while hunting the high-energy bias, and recorded because the
    next person will find it the same way.  The flag is written, tested
    offline, documented at length in `config.py`, defaults to `False`, and
    appears in **no** write-up -- so by failure 82 it is an untested
    configuration chosen over another untested one.

    Its argument is sound: the window is built from two corners of
    `solve_glide`'s command box, the committed pairing is
    `(alpha_min, bank_max)` / `(alpha_max, bank_min)`, and on this airframe
    range falls with alpha over the whole solve range, so both of those are
    mid-box trajectories and the "window" they span is a sliver of the real
    authority.

    **What is not in that comment is the coupling.**  Measured offline on
    the ladder states, turning it on takes the window from **85-128 km wide
    to 540-1860 km**, and `DEORBIT_WINDOW_BIAS` is *a share of the width* --
    so a constant fitted at 0.25 against an 85 km window becomes a 125 km
    aim offset against the honest one.  `DEORBIT_WINDOW_TIME_ON_LONG` has
    to come off in the same breath (its own comment says the honest long
    corner is vetoed by the 1500 s clock for every burn, and the deorbit
    then never commits at all).

    So the arm is three flags, not one: `DEORBIT_WINDOW_CORNERS_FIXED=True`,
    `DEORBIT_WINDOW_TIME_ON_LONG=False`, `DEORBIT_WINDOW_BIAS=0.0`, and the
    bias wants re-fitting afterwards because its units have changed meaning.
    It was not flown this session -- the aim's gain (failure 88) turned out
    to be the cheaper route to the same symptom -- and it is left at `False`
    with the coupling written down rather than left to be rediscovered.

96. **The per-axis attitude tune handed kRPC roll and yaw swapped, and the
    comment beside it said the opposite of what kRPC does.**
    `attitude_time_to_peak` returned `(pitch, yaw, roll)` with the comment
    "`time_to_peak` wants (pitch, yaw, roll)"; kRPC applies it as **(pitch,
    roll, yaw)**, the vessel frame's x, y, z and the order of
    `moment_of_inertia`. Measured on the shuttle in orbit: `time_to_peak`
    (3, 30, 3) cuts the *roll* PID gains tenfold (19.3 -> 1.93) and
    (3, 3, 30) the *yaw* gains (431 -> 43); a 90 degree roll takes 8.0 s
    against 3.7. So since `ATTITUDE_TIME_TO_PEAK_DERIVED` became the default
    the shuttle has flown **roll on 22.6 s** and **yaw on 4.8 s** -- a bank
    that lags its command by tens of seconds (`bnk=` against `bank=`: -45
    commanded, +17 flown; the heading rate says 3-16 deg of a 40 deg command,
    against 22-24 on the old craft), and a quick yaw on its weakest axis. The
    offline test enshrined the wrong order because it tested the code against
    its own comment. The read-back of `time_to_peak` returns what was
    written, so nothing in a log could show it; the autotuned *gains* can.
    Fix behind `ATTITUDE_AXES_KRPC_ORDER`. **Check an interface's axis order
    against its behaviour, not its documentation and not our comment.**

97. **The coast reversed the bank on every azimuth crossing, at no dynamic
    pressure.** `run_coast` leaned through `guidance.bank_toward`, which has no
    hysteresis, so with the track near the bearing to the gate the sign
    flipped every time they crossed: 8-9 full +-30 deg reversals in COAST at
    Mach 7 and q 0-120 Pa on the shuttle (game LOG3404/3405, sim LOG3416+),
    against 3 in the whole hypersonic glide. `COAST_BANK_LATCH` routes the
    sign through the glide's own deadband: 0 reversals, in the sim and the
    game. **It is not a fix on its own**: the entry then leaned the other way
    first, and the arrival went +4.7 -> +8.9 km in the game (n=3 an arm,
    `pairfly` 2026-09-24, LOG3456-3501) -- because the terminal glide had no
    sink left to absorb any change in cross-track history. See 98.

98. **Below Mach 3.5 the glide ran out of sink with drag untouched.** Bank at
    `BANK_MAX_DEG` 70, alpha at `ALPHA_MAX_DEG` 32, and `long` climbing
    +500 -> +8000 m through the chattering terminal reversals (LOG3416,
    LOG3440), while the shuttle's own table offers CdA 145 at 40 deg against
    86 at 30 (Mach 3). Raising `ALPHA_MAX_DEG` itself broke the deorbit (it
    is a corner of the search box: 6 of 8 sim flights never reached the
    interface). `GLIDE_ALPHA_MAX_DEG` is the glide's own ceiling. With it the
    hypersonic lean fell from ~50 to ~32 deg -- alpha doing the energy work
    bank had been doing -- and the arrival came in; see the journal,
    2026-09-24 (third).

99. **"Full authority" on roll lost the shuttle's entry every time it was
    flown, and the loss looked like a roll oscillation, not a departure.**
    `ATTITUDE_ROLL_TIME_TO_PEAK_S=1.0` (kRPC's default tune, the user's
    request the night before) replaced the derived 4.8 s. It tracked to a
    degree up to q ~700 Pa; above that the bank swung about a *steady* +30
    command with growing amplitude -- 16, 48, 34, 15, 35, 44, 7, 59, -5, 64
    -- until the nose was lost (the user, live, LOG3692: "the roll kept
    oscillating"). On the farm, qs_shuttle, it was **5/5 destroyed in the
    entry** (LOG3693/3696/3699/3702 plus the live one), bank error 125-179,
    sideslip 41-64, against no entry lost at 4.8. The mechanism is not the
    roll loop alone: at 35 deg of alpha a body roll *is* sideslip, and yaw
    sat on its wheels-only 22.6 s, so a fast roll made slip that nothing
    took out (LOG3699: +27..+40 deg of slip for 30 s about a steady command).
    **Roll and yaw are one lateral axis at high alpha; tune them apart and
    the faster one makes the slower one's error.** Default back to the
    derived figure (fingerprint in HANDOFF.md). Two follow-ups measured:
    yaw on roll's figure everywhere (`ATTITUDE_YAW_WITH_ROLL`) halved the
    hypersonic slip and lost the bank in the cone instead (see the journal,
    2026-09-26); a damper that slowed on every crossing of the command ran
    to its ceiling on a +-7 deg wobble the tune did not change (LOG3710) --
    an airframe mode, not a loop one -- and was cut back to growth only.

100. **The propellant the deorbit left over was the shuttle's hypersonic
     pitch stability, and dumping it lost the entry 3 of 3.** 250 m/s of
     reserve for a 26 m/s burn left 376 units (1.88 t, 6.5%) aboard to the
     runway, all of it in the only tank that holds any -- the Mk3 adapter at
     the nose, station +9.8 m -- which is what the user asked about ("why
     aren't you draining all the fuel?"). `DRAIN_RESIDUAL` dumped it on the
     first GLIDE tick (1 s, both valves, mass 30.48 -> 28.72 t). Flown
     against the defaults (pairfly ksp0/1, qs_shuttle, fingerprint
     `cb8bcf3d`), the drained vehicle **departed 3 of 3** (LOG3743, 3744,
     3745): alpha overshooting its 35 deg command to 45-48 at a Mach 4-7
     bank reversal, sideslip 70-100 deg peak to peak, arrival 19-29 km
     short; the defaults beside it 1 of 2 (LOG3742, flown while a third
     instance was busy; LOG3746 clean) and 0 of 6 the night before
     (LOG3729-3739). `testInstances/cgProbe.py` shows the mechanism
     without a flight: the pitching moment about each candidate CG, at
     neutral surfaces, as a fraction of the elevons' authority. Drained,
     the CG moves 0.65 m aft and the Mach-6 moment curve goes flat
     (-0.00 per 20 deg of alpha against -0.02 to -0.04 with the nose
     tank): neutrally stable exactly where the glide holds 35 deg and
     reverses its bank. Subsonic it stays stable (-0.11 per 20 deg), and
     there the nose fuel is pure cost -- up to 0.33 of the pitch authority
     spent on trim in the flare against 0.19 drained. **Mass you did not put
     aboard on purpose can still be doing a job; price the CG before you
     dump it.** `DRAIN_RESIDUAL_MACH_MAX` holds the valve to Mach 0.8.

101. **The flare flew its attitude with the descent added twice, and every
     arrest it made became a float and a stall.** `aim_runway` turned the
     flare's angle of attack into a pitch above the runway as `alpha +
     descent` (`AIM_RUNWAY_TRUE_ALPHA`, written to cure a flare that had
     flown `alpha` as a pitch); attitude is `alpha - descent`, so the vehicle
     was handed alpha plus *twice* the descent angle. On the old craft
     (LOG3869) the door asked 6 deg and the vehicle flew 15.6 the tick the
     runway reference took over, levelled at 65 m, bled 85 -> 45 m/s and fell
     the last 60 m at 17.7 m/s. Every default flare on both craft did the
     same thing -- the shuttle bench (`qs_shuttle_low`) stalled out 11 of 12
     (minimum speed 34-37 m/s, docking port first: a nose slam), the old
     craft from orbit contacted at 15-21 m/s on 6 of 6 (LOG4012-4028). The
     error was *masked* by the thing it broke: the extra lift is what
     arrested the sink at all, so every other flare constant had been fitted
     to it. Fixed together with the two things it had been hiding
     (`FLARE_TAIL_BY_ATTITUDE`, `FLARE_EXP_TAU_S`, `FLARE_DOOR_FROM_SCHEDULE`,
     default 2026-09-30): the sink schedule `sqrt(td^2 + 2 a h)` was
     *designed* to touch down at 8 m/s (wings and elevons tolerate 15, the
     shuttle's engine 7), and the door opened 1-2 s after the schedule began
     to bind. Old craft from orbit 5/6 intact on the runway, contact 1-4 m/s.
     **A sign error that the rest of the law has been tuned around is not
     fixed by flipping the sign; find what else was leaning on it.**

102. **The cone's plan credited path the vehicle would never fly, and the
     radius scan went looking for it.** Lined up a little outside a wide
     circle, the tangent point lies a few degrees *past* the rollout.
     `hac_turn` correctly reads that as arrived (turn 0), but `hac_path`
     still costed the straight run to the tangent point, `sqrt(x^2 + 2 R
     dy)`: 190 m outside a 16 km circle and 955 m before the gate is 2651 m
     of "path" (LOG4056 logged `gate=955 path=2650`; reproduced to the
     metre offline). `hac_radius` picks the *longest* path that fits the
     height, so it steered the plan onto exactly those circles -- R walked
     13 -> 16 km in the cone's last kilometres -- and the cone read itself
     on profile while 1.1-2.8 km high on every flight of both craft, with
     the weave at 0 (LOG4051 carried 2.2-3.3 km of phantom through most of
     its cone). `HAC_PATH_WRAP_TO_GATE` costs such a tangent point as the
     distance to the gate. **A scan that maximises a model will find the
     model's errors before it finds anything real; check the maximiser's
     choice against the geometry, not just the model's value.**
