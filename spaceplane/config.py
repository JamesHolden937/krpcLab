"""Every tunable in one place.

Same rule as ``boosterland.config``: control code reads ``cfg.X`` and never
hard-codes a number, and ``Autopilot.__init__`` writes a ``config:`` line
listing what differs from these defaults, so a log says which configuration
flew it.

The aerodynamic numbers here are *measured*, not guessed --
``testInstances/planeprobe.py`` against the real airframe, recorded in
``planeprobeGearup.txt``.  Where a default came out of that file the
measurement is quoted next to it, because the next vehicle will have different
ones and there has to be a way to tell a measurement from a guess.
"""
import hashlib
from dataclasses import dataclass, fields


@dataclass
class Config:
    # -- the runway --------------------------------------------------------
    # Threshold coordinates and the heading to land on.  Two ends, and the
    # guidance picks whichever suits the arrival: see Runway.choose.
    # **A latitude per threshold, because the strip is not east-west.**  It
    # used to share one, and the previous note here said so and left it:
    # "worth 10-20 m at the far threshold on a strip 70 m wide, so it will
    # matter once the cross-track is inside that -- and it is not what is
    # costing kilometres today".  Both halves were right and the second half
    # stopped being true.
    #
    # Measured on this install, by scanning latitude in half-metre steps at
    # each threshold's longitude and taking the centre of the raised, flat
    # band (79 m wide, the tarmac plus its shoulders):
    #
    #     lon -74.724466   centre lat -0.0485761   plateau 69.17 m
    #     lon -74.495283   centre lat -0.0501756   plateau 68.76 m
    #
    # So the centreline drifts **16.9 m south** over the runway's length, a
    # tilt of 0.4 degrees, and the single -0.0486 was half the runway's width
    # out at the east end.  It also made ``Runway.along`` -- which is taken
    # from the two thresholds -- exactly east-west, which the runway is not,
    # so the extended centreline the approach tracks diverged from the real
    # one by 29 m at the 4 km gate.
    #
    # Independently corroborated: a ten-year-old forum post gives
    # (-0.0485997, -74.724375) and (-0.0502119, -74.489998), which this probe
    # reproduces to about a metre at both ends.
    #
    # The longitudes are *not* from the same probe and must not be: the
    # raised PQS plateau extends well past the tarmac at both ends (still
    # 79 m wide at -74.729 and -74.488), so terrain height locates the
    # centreline but says nothing about where the runway stops.  Those come
    # from the earlier measurement of the tarmac itself.
    RUNWAY_09_LAT: float = -0.0485761       # west end, at RUNWAY_09_LON
    RUNWAY_27_LAT: float = -0.0501756       # east end, at RUNWAY_27_LON
    RUNWAY_09_LON: float = -74.724466       # west end, land heading 090
    RUNWAY_27_LON: float = -74.495283       # east end, land heading 270
    RUNWAY_LENGTH_M: float = 2400.0         # 2.5 km of tarmac, 2.4 recorded
    # **How wide the tarmac is, which is the tolerance the whole flight is
    # actually being judged against and was nowhere in the code.**  About
    # 70 m, so 35 either side of the centreline -- and a landing 25 m off is
    # *inside* that by ten metres, which is not the margin it sounds like
    # once the aircraft has a wingspan and a rollout to go.  Carried here so
    # ``run_stopped`` can say whether the vehicle is on the runway instead of
    # leaving that to be scored by hand afterwards.
    RUNWAY_WIDTH_M: float = 70.0
    RUNWAY_BOTH_ENDS: bool = True           # False pins runway 09

    # **Which entry state flew this, recorded because the log could not say.**
    # The harness loads a quicksave and the autopilot never learns its name,
    # so a campaign flying three saves against one another produced logs that
    # were identical in the header and different in the vehicle -- failure 28
    # exactly, with the save in place of the defaults.  ``quickglide.py``
    # passes it; nothing reads it.  It is carried in ``Config`` so that it
    # rides the ``config:`` difference line for free.
    SAVE_NAME: str = ""

    # -- the approach gate -------------------------------------------------
    # The glide is solved to arrive *here*, not at the threshold: a point on
    # the extended centreline at a fixed altitude.  Everything from the gate
    # in is flown geometrically rather than predicted, because a predictor
    # that flies the flare is predicting a manoeuvre it cannot model.
    # Measured best glide is L/D 3.39, i.e. a 16.4 degree path, so a gate
    # 4 km out has to be about 1200 m up for the vehicle to reach it at all.
    # It is put slightly *above* that (19 degrees) on purpose: arriving with
    # surplus height is recoverable by raising the nose, and arriving low is
    # not recoverable by anything.  The same one-sided logic as the deorbit
    # bias, one phase later.
    # **Re-placed against the polar the airframe actually flies.**  At the
    # L/D of 3.39 that ``planeprobe`` reported, a gate 4 km out at 1400 m had
    # 3.4 km of glide in hand against the 4.6 km to the touchdown aim.  At
    # the measured 2.1 it has 2.9 km against 4.6: **the gate was placed where
    # the vehicle cannot reach the runway from**, so an approach flown
    # perfectly from it still lands 1.7 km short.  That is most of the
    # "arrives short" this project has been chasing at the terminal end, and
    # it was structural rather than a control error.
    # The margin has to be taken against the *gear-down* polar, and it is
    # worse than the probe said: flown, best glide is L/D 2.19 gear up and
    # **1.78 gear down** (n=333 and 175 subsonic samples), a 19% penalty
    # rather than the 9% ``planeprobe`` reported.  From the gate the vehicle
    # flies ``GATE_ALT_M - GEAR_ALT_M`` clean and ``GEAR_ALT_M`` dirty, so at
    # 2400 m over a 1500 m gear height the reach was 4.93 km against the
    # 4.6 km to the aim -- 300 m of margin, which is none.  Raising the gate
    # and dropping the gear later buys 766 m, and surplus height is the side
    # to be on: it is spendable by the S-turn and a shortfall is not
    # recoverable by anything.
    # **Closed by moving the gate *in*, not up, and that distinction was
    # measured.**  At the L/D the fast approach flies (1.55, see
    # ``APPROACH_BEST_LD``) the 4.2 km from a 4 km gate to the touchdown aim
    # needs 2.7 km of height and 2600 m did not close it: the glide handed
    # over 626 m short of the gate and the approach lost another 435 m,
    # arriving 861 m short of the threshold on a flight whose touchdown was
    # otherwise perfect (``logs/LOG903``, sink -6.9 m/s, 7 m off centre).
    #
    # Raising the gate to 3200 m closes the geometry and **costs 11 km**.
    # ``GATE_ALT_M`` sets the gate's *radius*, which is the altitude the
    # deorbit's own propagations stop at, so lifting it moves the burn: the
    # search committed 35 km earlier on 1 m/s less dv -- the shallow regime
    # again -- the reachable window halved from 120 km to 56, and five
    # flights landed 15 to 36 km short against the 4 km the same
    # configuration managed at 2600.
    #
    # Shortening ``GATE_DIST_M`` to 3400 was the other candidate, on the
    # reasoning that it closes the same inequality without touching the
    # gate's radius.  **Also worse** -- two flights at -16.3 and -21.5 km --
    # because it moves the gate *horizontally*, which moves the arc to the
    # gate and therefore the deorbit's target just as surely.
    #
    # So both handles on this inequality run back into the burn, and the
    # measured-best geometry is the one that does not close it: 2600 m over
    # 4.2 km of flying is 4.03 km of reach against 4.2 needed, about 4%
    # short, which is most of the ~1 km the good flights land short of the
    # aim.  Left as it is because every attempt to close it has cost ten
    # times what it buys; the way out is to stop the gate's placement feeding
    # the deorbit, not to keep moving it.
    # **The gate sits above the best glide to the aim on purpose, and the
    # margin has to be one the approach can actually spend.**  4200 m of
    # ground to the touchdown aim at the 1.95 the approach flies is 2150 m
    # of height; 2600 was 450 m over that, and the approach converts every
    # metre it cannot spend into about two metres of runway.  ``LOG1410``
    # landed intact and on the centreline and rolled 724 m off the far end
    # for exactly that reason.  2350 keeps 200 m of one-sided margin -- the
    # side that is recoverable -- which is about what the S-turn is worth.
    #
    # **2350 was flown and is below the line.**  Three of six arrivals hit
    # the ground during APPROACH with 26 m/s of sink and no flare at all:
    # 1.95 is the ground ratio the approach *achieves on average*, not the
    # one it can rely on when it has to stretch, and the gate's margin is
    # one-sided precisely because the low side has no recovery. Back to
    # 2600, and the surplus that creates is the S-turn's job rather than the
    # gate's -- which is the split ``APPROACH_BEST_LD`` at its measured
    # value now makes visible, since ``excess`` at the gate reads 446 m
    # where it used to read none.
    # The approach may not unload the wing below one g.  See
    # ``guidance.approach``: the path term used to be free to command zero
    # alpha to shed surplus height, which converts height into sink rather
    # than spending it, and every one of 27 flights entered the flare at 40
    # to 65 degrees nose-down and broke up.  Left as a switch only so the
    # two can be flown against each other.
    APPROACH_TRIM_FLOOR: bool = True
    GATE_ALT_M: float = 2000.0
    GATE_DIST_M: float = 4000.0             # before the threshold
    # **Put the gate where the approach's own model needs ``GATE_ALT_M``.**
    # The cone checks its exit against ``approach_needed = (gate_range +
    # GATE_DIST_M + TOUCHDOWN_AIM_M) / approach_ld``, which at a 4 km gate is
    # (4000 + 2400) / 4.2 = 1524 m -- against a gate at 2000.  So every
    # approach began ~500 m high by construction, more with the cone's own
    # exit allowance on top (the shuttle: APPROACH at rwy=1046 m, h=2000,
    # exc=+1222, LOG3567).  True solves ``GATE_ALT_M * approach_ld -
    # TOUCHDOWN_AIM_M`` once, before the deorbit: 6000 m with today's
    # constants, and per craft wherever ``approach_ld`` is.
    GATE_FROM_APPROACH: bool = True     # the shuttle chain, 4/4 landed (LOG3656-3661)
    GATE_CAPTURE_M: float = 1500.0          # hand over to APPROACH within this

    # -- the heading alignment cone ----------------------------------------
    # **The entry is not asked to arrive with the right energy any more.**
    # Everything above about the gate is the geometry of a *straight-in*
    # arrival, where the glide has to deliver the vehicle to one point with
    # one energy from fifteen hundred kilometres away.  Measured, it delivers
    # it with 14.8 km of along-track scatter and 97.6% of that variance is
    # made before the entry interface, where nothing in the glide can reach
    # it.  Every attempt to shrink it -- reserves, schedules, gate placement,
    # a burn ten times more accurate -- has come back inside the noise.
    #
    # So stop trying.  Aim the entry at a point *over* the field with ten
    # kilometres of height in hand, and spend whatever surplus arrives by
    # turning: a circle tangent to the extended centreline, flown round as
    # many times as the energy needs, rolling out on final when the height
    # that is left matches the distance that is left.  That is a shuttle's
    # heading alignment cone, and the reason is the same one that produced it
    # there -- an unpowered vehicle cannot choose its arrival energy, so the
    # approach has to be able to absorb any of it.
    #
    # What this buys is not accuracy, it is *tolerance*: a lap of a 12 km
    # circle is 75 km of path, so an arrival 30 km long and one 30 km short
    # both reach the same gate, and the requirement on the deorbit drops from
    # "hit a 16 m/s plateau" to "do not be short".  That is the side the bias
    # was already on.
    HAC_ON: bool = True
    HAC_ALT_M: float = 12000.0              # the entry's target, over the gate
    # The turn circle.  Tangent to the extended centreline at the low gate,
    # so rolling out of it *is* being lined up -- the alignment and the energy
    # management are the same manoeuvre, which is the whole point of the cone.
    # The radius is the energy knob: the guidance flies whatever radius makes
    # the remaining arc match the remaining height, between these bounds, and
    # adds a whole lap when even the widest circle is not enough path.
    HAC_RADIUS_M: float = 8000.0            # nominal
    HAC_RADIUS_MIN_M: float = 2000.0
    # 40 km was tried and is too much rope: the scan took a 20 km circle
    # and an 87 km path and flew away from the field.  The width that
    # matters is supplied instead by ``HAC_HOLD_MARGIN`` raising the *floor*
    # with speed, which is the cone's shape and comes from the airframe.
    HAC_RADIUS_MAX_M: float = 16000.0
    HAC_MAX_LAPS: int = 3
    # How much better an extra lap has to fit before it is worth flying.  A
    # lap is minutes at the bank limit with the ground getting closer, so it
    # is not bought to shave a few hundred metres off a fit the approach's
    # own S-turn would absorb.
    HAC_LAP_MARGIN_M: float = 1500.0
    # The glide ratio the cone is *planned* at, and it is deliberately not
    # the one it achieves.  It is the achieved glide ratio discounted for
    # tracking: the vehicle does not fly the ideal circle, it joins,
    # overshoots, re-captures and chases a radius re-solved every tick, and
    # all of that is path the geometry did not cost.  Both halves are in the
    # logs and ``conesum.py`` reads them back:
    #
    #     LD flown over the phase   1.86 - 1.90   (22 flights, sd 0.05)
    #     path flown / path planned 1.07 - 1.11
    #     -> HAC_LD                 1.68 - 1.76, median 1.73
    #
    # **Those three lines describe the configuration flown when they were
    # written.**  Re-run today (``conesum.py``, 58 flights, the 30 that roll
    # out) the tracking factor reads **1.27 - 1.39**, so the committed 1.86
    # is not "a little under the median" of anything current -- it is the
    # number that lands.  The achieved ratio is 1.42, and a batch flown at
    # 1.49 stopped **5.2 km short, 3.9 km from the gate**, which is LOG1315's
    # failure again from the other direction.  This constant is what the cone
    # must *plan* with; it is not a measurement of what the cone achieves,
    # and the two agreeing once is what makes substituting one for the other
    # look safe.
    #
    # **1.35 was the transcribed number and it was 25% low, which is what
    # was putting the vehicle in the sea.**  Wrong low the cone plans a
    # shorter circle than the height can pay for, so it arrives over the
    # gate high -- and, worse, ``hac``'s weave never fires, because the
    # surplus it watches is measured against the same wrong budget.
    # ``logs/LOG1315`` planned 15.0 km, flew 17.5, reached the rollout
    # **1499 m** above profile with the weave never once commanded, and
    # overflew the aim by 3.5 km into the sea.  Wrong high it arrives short,
    # which nothing absorbs, so this sits a little under the median.
    #
    # Re-run ``conesum.py --settled`` after anything that changes how the
    # cone is flown -- the attitude controller, ``HAC_BANK_MAX_DEG``,
    # ``HAC_SPEED_FACTOR`` -- because all of those move both halves of it.
    # Failure 23's rule; failure 13's remedy.
    HAC_LD: float = 1.86
    # **Where the entry aims, which is a different measurement.**
    # ``Runway.high_gate`` puts the entry's target this many metres of
    # ground back per metre of height between ``HAC_ALT_M`` and
    # ``GATE_ALT_M``; ``HAC_LD`` above is *path* per metre of height.  They
    # were one constant until the cone's budget was corrected and silently
    # moved the aim 3.3 km with it.  This value keeps the aim exactly where
    # every arrival to date was flown to, so that correcting one does not
    # re-tune the other; it is the knob to move when the arrivals are
    # centred on the wrong side, and they are currently centred about 5 km
    # short (nine flights, spread sd ~11 km).
    HAC_GATE_LD: float = 1.35
    HAC_BANK_MAX_DEG: float = 45.0
    HAC_HEADING_KP: float = 1.2             # deg of bank per deg of track error
    HAC_CAPTURE_M: float = 4000.0           # radial error that saturates the cut
    # Speed in the turn, as a multiple of **the approach speed** at that
    # load -- not of the stall speed.  Written against the stall it was 84
    # m/s while ``APPROACH_FACTOR`` asks for 115 and
    # ``APPROACH_SPEED_FLOOR_FACTOR`` refuses anything under 96, so the cone
    # would have rolled out below the floor of the phase it hands to and the
    # approach's only way to make the speed back is to dive -- which is
    # exactly the failure ``SPEED_FLOOR_ON`` exists to stop.  Expressed as a
    # multiple of the approach speed the two cannot drift apart when either
    # is retuned.
    HAC_SPEED_FACTOR: float = 1.00          # x the approach speed
    HAC_SPEED_KP: float = 0.20              # deg of alpha per m/s of excess
    # Hold the cone's speed with ``guidance.alpha_for_speed`` (the
    # approach's two-sided law, ``APPROACH_SPEED_PATH``) rather than ``trim +
    # HAC_SPEED_KP * error``, which cannot make speed.  The shuttle decays to
    # 55-80 m/s in the cone under the old law (LOG3029).  Off until paired.
    HAC_SPEED_PATH: bool = False
    # **An inner loop on lift** (``Autopilot.lift_loop``): offset the
    # commanded alpha until the measured lift matches what the table
    # promised the law.  The landing's table is untrimmed (shuttle 0.73x,
    # old craft 1.34x subsonically); a 1.6 g pull-up arrived at ~1.1 g and
    # the cone's climb never happened (LOG3065-3066).  Cone and approach
    # only; the flare has its own sink loop and a tail-strike cap.
    # **The cone's speed target as equivalent airspeed** (``guidance.
    # eas_scale``): a stall speed is a sea-level quantity and the cone flew
    # it as true airspeed at 9 km -- ~66 m/s equivalent, near the stall,
    # held with 19-22 deg of alpha (LOG3071).
    HAC_SPEED_EAS: bool = False
    # **The cone at one IAS derived from the stall** (``guidance.cone_ias``,
    # the user's, 2026-10-05): ``max(1.3 x stall, minimum-drag speed)``
    # off this airframe's table, times ``sqrt(1/cos(HAC_BANK_MAX_DEG))``,
    # held as indicated airspeed from the top of the cone to the gate.
    # Implies the EAS scaling.  Off until paired.
    HAC_IAS_FROM_STALL: bool = False
    # **An outer loop on the angle itself** (``Autopilot.alpha_trim_loop``),
    # APPROACH and FLARE: kRPC's attitude loop holds a standing pitch error
    # that grows with q -- fast shuttle flares asked 5-8 deg and flew 2.7-4.1
    # for eight seconds (LOG3994-4007), S-turning approaches ran 2-7 deg rms
    # short and dived (LOG3810-3849).  Integrates commanded - signed alpha
    # while the roll is settled and the slip small.  Off until paired.
    ALPHA_TRIM_LOOP: bool = True  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # Manual pitch input per degree of pitch pointing error in the FLARE,
    # summed with kRPC's (0 = off).  See ``Autopilot.flare_pitch_p``.
    FLARE_PITCH_P: float = 0.08  # default 2026-10-03: the landing stack, rot-orbit2-1003
    FLARE_PITCH_P_MAX: float = 0.5
    ALPHA_TRIM_MIN_DEG: float = -2.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
    ALPHA_TRIM_MAX_DEG: float = 4.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
    ALPHA_TRIM_ROLL_TOL_DEG: float = 10.0
    ALPHA_TRIM_SLIP_TOL_DEG: float = 15.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # ``ALPHA_TRIM_LOOP`` in the cone as well: the dive that hands the
    # approach 100+ m/s of sink starts there (LOG4803: commanded 2-10 deg,
    # kRPC's signed alpha -3, pitch input +0.07).
    ALPHA_TRIM_IN_HAC: bool = True  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # ``ALPHA_TRIM_LOOP`` in the GLIDE as well (``Autopilot.alpha_trim_loop``).
    # The shuttle's alpha shortfall below Mach 4 is a function of *bank*, not
    # only of q: at 2.5-4.5 kPa it flies 4-5 deg under its command below 35
    # deg of bank and ~10 above 50, with the pitch input at 0.5-0.7 -- not
    # saturated.  The hard-banking flights reach the cone 4-15 km long and
    # 5 km high, 9 of 24 in rot-phantom-1003.  ``ALPHA_TRIM_GLIDE_MAX_DEG``
    # bounds the offset there: a safety bound, not a fit.
    ALPHA_TRIM_IN_GLIDE: bool = True  # default 2026-10-04: rot-glidetrim-1003 + rot-glidetrim2-1004, intact on land 9 vs 5 of 36
    ALPHA_TRIM_GLIDE_MAX_DEG: float = 10.0
    LIFT_LOOP_MIN_Q_PA: float = 500.0
    SPEED_PATH_CLIMB_MAX_DEG: float = 20.0
    HAC_ALPHA_MAX_DEG: float = 22.0
    # Roll out when there is this little turn left, or when the height is
    # gone -- whichever comes first.  The second is not a fallback, it is the
    # floor the cone is flown above: below the low gate's altitude there is
    # no surplus left to spend and the approach's own geometry takes over.
    HAC_EXIT_TURN_DEG: float = 12.0
    # **How far past the rollout still counts as arrived.**  ``hac_turn``
    # wraps into ``[0, 2pi)``, so a vehicle a few degrees past the rollout
    # reads a whole lap to go; the band that catches that was
    # ``HAC_EXIT_TURN_DEG``, 12 degrees, on a cone that weaves +-50.  Flown in
    # the sim (LOG3520): turn 5.1 -> 0.0 -> 6.3 -> 1.6 -> **345.4** as the
    # weave carried the track 14.6 degrees past the tangent at 3.1 km, then
    # a lap it could not afford and out of the cone at 2 km pointing away;
    # the one flight of eight that landed intact (LOG3519) never left turn 0.
    # 0 keeps the old band.
    HAC_OVERSHOOT_DEG: float = 0.0
    # The same band for the **exit test only** (``Autopilot.run_hac``): the
    # plan keeps its wrap, so a vehicle too high still owes its lap, but one
    # at the gate with the height the approach can take is let go even with
    # the weave's last swing past the centreline.  LOG4104: 829 m from the
    # gate, 750 m above need, turn 343-347 -- flew on and ran out of height
    # 3.2 km past it.  0 is the committed behaviour.  Off until paired.
    # **25, default 2026-10-05.**  Paired at last (rot-past-1005, three
    # rigoff orbits): "out of height" exits 0/12 against 4/12 -- each of
    # those had overshot the rollout 14-23 deg, read a lap owed, widened the
    # circle chasing it and left at the 2 km floor beside the field with
    # 0.5-1.2 km of surplus (LOG6224, 6305, 6309, 6314).  On the runway 2/12
    # vs 0/12.
    HAC_EXIT_PAST_DEG: float = 25.0
    # How much height the cone may still hand the approach when it rolls
    # out.  Not zero: the approach has an S-turn for exactly this and a
    # whole extra lap to shed 700 m is a nineteen kilometre answer to a one
    # kilometre question.  Above it, the vehicle flies past the rollout and
    # the turn wraps round to another lap, which is what the surplus needs.
    # **1500 is three times what the approach can spend and twice what the
    # runway can absorb.**  Measured, the approach turns a metre of surplus
    # at the rollout into about two metres of touchdown point -- its ground
    # glide ratio is 1.8-2.1 whether it is high or low, and the S-turn moves
    # that by a few per cent, not by a third.  ``logs/LOG1410`` rolled out
    # **634 m** high, landed intact and on the centreline (22 of 23 parts,
    # -6.6 m), and rolled 724 m off the far end because the touchdown was
    # 1.3 km late.  There are about 1200 m of tarmac either side of the aim,
    # so the surplus that can be absorbed is about 500.
    #
    # **And now that the test is against the approach's own requirement**
    # (see ``guidance.hac``'s ``approach_needed``) it is measured against the
    # right thing and buys runway directly: every metre of it is 2.08 metres
    # of touchdown point.  500 pins the handover 500 m high on every flight
    # -- the surplus column reads +493 to +499 whatever the arrival was --
    # and puts the wheels a kilometre late.
    #
    # **150 was flown and is worse, for a reason worth keeping.**  The exit
    # wants *both* the rollout geometry (``gate_range <= HAC_ROLLOUT_M``) and
    # the height; tightening the height alone means the height test stops
    # being satisfiable at the rollout, so the vehicle stays in the cone and
    # leaves through the ``GATE_ALT_M`` floor instead -- "out of height",
    # from wherever it happens to be.  Flown at 150, three of three exits
    # were out of height, one of them **2435 m from the gate at 2597 m**,
    # which hands the approach a geometry no glide ratio can fly.  The
    # surplus cannot be tightened below what the cone's own descent profile
    # delivers at the rollout; lowering *that* is a change to ``GATE_ALT_M``
    # and to what the cone plans against, not to this.
    # **Budget the cone in specific energy rather than height.**
    #
    # The cone was built to absorb an arbitrary arrival, and it does -- in
    # one of the two variables that matter.  ``guidance.hac`` plans against
    # ``(height - GATE_ALT_M) * HAC_LD``, so a vehicle at the right altitude
    # and 35 m/s too fast plans no extra path at all.
    #
    # That did not show while the entry was five kilometres short, because
    # the shortfall was absorbing the energy.  With the along-track centred
    # (``GLIDE_RESERVE_M``) it is the binding constraint: seventeen flights
    # give ``corr(arrival, flare entry speed) = +0.77`` and
    # ``corr(cross-track at flare entry, final cross-track) = -0.99``, so the
    # vehicle that arrives *on target* arrives hot, enters the flare at
    # 103 m/s with 42 m/s of sink and 50-67 m of offset, and lands 29-44 m
    # off a strip that is 35 m wide.  See spaceplane failure 47.
    #
    # On, the surplus kinetic energy over the speed the approach wants is
    # added to the height the cone has to spend, so being fast buys path --
    # a wider circle, a weave, or a lap -- which is the one thing that sheds
    # it.  No constant about this airframe: the reference speed is the one
    # the cone already trims its angle of attack to.
    #
    # **Flown, and it is worse: off because it was measured, not because the
    # reasoning was wrong.**  Twelve flights, two arms rotated across three
    # instances (``logs/LOG1941-1952``): along-track identical at -0.6 and
    # -0.8 km, and **four of six destroyed in ROLLOUT against one of six**.
    #
    # The log says why, and it is the same shape as much else in this file:
    # *it sheds energy without a target*.  The cone dumps whatever is over
    # the reference speed and hands the vehicle to an approach that flies its
    # own speed law, so the flare is entered at 86.0, 99.4, 67.1 and 94.7 m/s
    # on four consecutive flights -- scattered across the whole usable range
    # rather than converged on any part of it, and 67.1 is *below* the
    # flare's measured stall floor.  Spending energy earlier does not help if
    # nothing downstream holds the quantity that matters.
    #
    # What the same batch did establish is where that quantity has to land:
    # flare entry at **83-91 m/s** gives -3, -4 and -5 m of cross-track with
    # every part attached; above 94 the lateral miss grows monotonically with
    # speed; below about 82 the flare stalls.  The approach commands
    # ``APPROACH_FACTOR`` x stall = 108 and refuses below
    # ``APPROACH_SPEED_FLOOR_FACTOR`` x stall = 96 -- both outside that
    # window -- so the good landings happened only where the vehicle was too
    # starved to obey.  That is the knob to move, and it is an existing one.
    HAC_ENERGY_BUDGET: bool = True  # default 2026-10-03: on the shuttle, exit surplus median +1200 -> +500 m, on the runway 6/12 vs 3/12, splashed 2 vs 4 (save-energy-1003, save-energy2-1003)

    HAC_EXIT_SURPLUS_M: float = 500.0
    # Exit unless the surplus can pay for a lap at the tightest circle the
    # airframe holds -- ``2 pi R / cone_ld`` -- because a lap it cannot pay
    # for is the only alternative to leaving.  See
    # ``Autopilot.hac_exit_surplus``.
    HAC_EXIT_SURPLUS_DERIVED: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # How near the gate counts as being at it.  ``GATE_CAPTURE_M`` is the
    # straight-in gate's own answer to the same question and this is
    # deliberately the same size.
    HAC_ROLLOUT_M: float = 900.0
    # -- spending a surplus the circle cannot ---------------------------
    # **The circle aligns; the weave spends.**  Measured, the vehicle
    # arrives over the gate already *aligned* -- cross-track a couple of
    # hundred metres, turn within a few degrees of the rollout -- and
    # several kilometres too high.  From there the circle's only offer is a
    # full orbit, and a full orbit costs ``2 pi R`` where ``R`` is set by the
    # speed (``v^2 / g tan(bank)``): 53 km at the 250 m/s the cone is
    # entered at, against a height budget of 13.  There is no entry altitude
    # that fixes that, because the orbit's cost scales with ``v^2`` and the
    # budget only with height.
    #
    # Weaving does scale: serpentining at ``theta`` off the intended track
    # multiplies the path by ``1/cos(theta)`` -- 41% more at 45 degrees --
    # for no net heading change and no extra radius.  So the angle is solved
    # from the surplus directly, ``theta = acos(planned / affordable)``, and
    # goes to zero by construction the moment the vehicle is on profile.
    HAC_WEAVE_ON: bool = True
    HAC_WEAVE_MAX_DEG: float = 50.0
    # **The same phantom, one degree outside the band** (``guidance.hac_path``):
    # before the gate, a tangent point up to this far past the rollout costs
    # the run to the gate, not a lap.  LOG4152: 13 deg past read as a 347
    # deg turn, the plan called itself short, stopped weaving and handed
    # over 5.4 km high.  0 is the committed behaviour.
    HAC_PAST_BEFORE_GATE_DEG: float = 0.0
    # **The same rule with no tolerance** (``guidance.hac_path``): before the
    # gate, any tangent point past the rollout -- the wrap itself, not an
    # angle -- costs the run to the gate.  The constant-free form of
    # ``HAC_PAST_BEFORE_GATE_DEG``.  Off until flown.
    HAC_WRAP_BEFORE_GATE: bool = False
    # **Price a lap at the speed it will be flown** (``guidance.hac_radius``):
    # laps take their radius floor from the cone's target speed, not the
    # entry speed.  At 270 m/s the floor is 7.4 km and no lap ever fits, so
    # 5-7 km of surplus went out of the gate (LOG4152, 4171).  Off.
    HAC_LAP_AT_TARGET_SPEED: bool = False
    # **The cone's glide ratio at the speed and bank it will be flown at**
    # (``guidance._hac_planned_ld``): the swept table at the cone's target
    # speed, wings level on the straight legs and at the circle's bank on
    # the arc, divided by ``airframe.PLANNING_BIAS``.  ``HAC_LD`` 1.86 is a
    # whole-cone average of a Mach 0.7 entry at 22 deg of alpha (flown 1.4)
    # and a subsonic straight-in at 6 (2.7-3.0 flown, and the table agrees:
    # ``ld=`` act/mdl 2.97/2.98 on LOG4051).  With the phantom path gone
    # (above) the cone read itself short at 6.1 km and still rolled out
    # 1.1 km high (LOG4086).
    # **Priced slice by slice down to the gate** (``guidance.hac_ladder``),
    # each at the target speed at *that* height: the first version took the
    # ratio where the vehicle was, read 1.16 at 13 km (thin air, high trim
    # alpha), called itself short, and found its 2 km of surplus below 7 km
    # with 5 km of path left -- weave pinned at 50, out +2.0 km (LOG4091).
    # Off until paired.
    HAC_LD_AT_TARGET: bool = False
    # **The cone's ratio as a measured curve** (``guidance.hac``,
    # ``Autopilot.hac_ld_scale``): the ladder above, every rung scaled by
    # the vehicle's measured L/D (kRPC's force, ``act=``) over the table's
    # at the alpha it is flying, smoothed over ``HAC_LD_MEASURED_TAU_S``.
    # Replaces ``HAC_LD`` without re-fitting it: the table supplies how the
    # ratio changes with height, the flight supplies the scale, on whatever
    # airframe flies.  Off.
    HAC_LD_MEASURED: bool = False
    HAC_LD_MEASURED_TAU_S: float = 20.0
    # **The entry aim off the same table** (``guidance.straight_in_reach``):
    # ``HAC_GATE_LD`` 1.35 is the old craft's; this computes the ground a
    # straight-in at the cone's speed covers per metre of height, once,
    # when the table is ready.  Off.
    HAC_AIM_DERIVED: bool = False
    HAC_LADDER_STEP_M: float = 500.0
    # **The cone's flap brake on surplus alone** (``hac_flap_brake``).  It
    # waited for the weave to pin at ``HAC_WEAVE_MAX_DEG``, which on the
    # shuttle it never does (~44 deg), and so never deployed in LOG3846-3875
    # while the cone exited 1-2 km above what the approach needed on most
    # flights (conesum: R saturated at 16 km, flown 20 km at L/D 1.9).  With
    # this on the brake comes out whenever the surplus over the cone's own
    # profile, less the arrest height of the sink it builds, exceeds
    # ``HAC_WEAVE_DEADBAND_M``.  Off until paired.
    HAC_FLAP_BRAKE_ON_SURPLUS: bool = False
    # ...and **not stowed for the roll** in the cone.  ``FLAP_BRAKE_YIELDS_
    # TO_ROLL`` was written for the hypersonic glide (LOG3680, 3690: the
    # elevon brake out, roll authority gone, departure at Mach 5-6); in the
    # subsonic cone the vehicle is always adjusting its bank, and under
    # ``HAC_FLAP_BRAKE_ON_SURPLUS`` the brake came out and was stowed 0.4-0.6 s
    # later "rolling" on every flight (LOG4035, 4037) -- it never spent
    # anything.  Off until paired.
    HAC_FLAP_BRAKE_IGNORES_ROLL: bool = False
    HAC_WEAVE_DEADBAND_M: float = 800.0     # surplus worth weaving for
    # The reversal is on a clock rather than on a cross-track band, because
    # the quantity a band would watch -- the offset from the intended path --
    # is what the weave is deliberately creating.
    HAC_WEAVE_PERIOD_S: float = 24.0
    # **Weave only while a whole cycle of path is left.**  The weave angle
    # is ``acos(total / available)``, so as the path to the gate goes to
    # zero with height still in hand it goes to its *maximum*: flown in the
    # sim, +-50 deg with 1.4 km to the gate at 120 m/s (a 1.2 km turn
    # radius), the vehicle passed the gate sideways, the gate distance went
    # 442 -> 1738 m and the cone left "out of height" (LOG3524; all six of
    # LOG3523-3528).  One cycle is ``speed * HAC_WEAVE_PERIOD_S`` of track;
    # below that the surplus is left to the approach's speed path.
    HAC_WEAVE_WHOLE_CYCLE: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **A weave that is flown, not only commanded** (``guidance.weave_angle``).
    # Over the eight cones of ``rot-ballast2-1001`` the gate-distance gained
    # per metre flown was **0.76-0.80 whatever the weave commanded** (cos
    # 0.55-0.81): reversing the track from +50 to -50 deg at 45 deg of bank
    # and 125 m/s takes ~23 s of turn plus ~10 s of roll, and the clock
    # reversed every 24 s, so the vehicle spent each swing turning and never
    # reached the angle.  The surplus over the profile stayed at +1.3-1.8 km
    # from cone entry to the gate.  With this on, each swing lasts the
    # reversal the airframe needs (turn at the weave's bank, roll at the
    # measured roll rate) plus ``HAC_WEAVE_HOLD_S`` held at the angle, and
    # the angle is solved for the *effective* path ratio of that swing --
    # ``(H cos t + T sin(t)/t) / (H + T)`` -- not ``1/cos``.  Near the gate
    # the hold shrinks and the angle is limited to a swing that still fits.
    HAC_WEAVE_HELD: bool = False
    HAC_WEAVE_HOLD_S: float = 20.0
    # The bank the weave reverses at; 0 is ``HAC_BANK_MAX_DEG``.  A
    # steeper bank reverses faster and costs lift, both of which spend.
    HAC_WEAVE_BANK_DEG: float = 0.0
    # Roll rate the reversal is timed with when none has been measured.
    HAC_WEAVE_ROLL_RATE_DEG_S: float = 8.0
    # **The spoiler as the cone's descent authority** once the weave is
    # saturated (``autopilot.hac_flap_brake``).  LOG3591 weaved at the full
    # 50 deg for the last 5 km and still reached the gate 2400 m high, where
    # a lap at 15-16 km radius costs ~94 km of path: out of height mid-lap.
    # Needs the measured brake (``AIRBRAKE_OPPOSED_FLAPS``,
    # ``AIRBRAKE_MEASURED``).
    HAC_FLAP_BRAKE: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # The pull-up assumed when pricing the height a sink costs to arrest.
    HAC_FLAP_ARREST_G: float = 0.5
    # How fast the commanded circle may change size.  The radius is the
    # cone's plan, and an unrated plan chatters between two manoeuvres that
    # have nothing in common -- see ``guidance.hac``.  At 400 m/s a full
    # sweep of the radius range takes 35 seconds, which is long enough to
    # fly an arc of the circle it is currently on.
    HAC_RADIUS_RATE_M_S: float = 400.0
    # How much wider than the theoretical minimum turn radius the tightest
    # offered circle is.  ``v^2 / (g tan(bank))`` is the radius at the bank
    # limit with the lift exactly balancing; flying at the limit leaves
    # nothing for the heading loop, and this airframe holds 0.74 of a
    # commanded angle in dense air.
    HAC_HOLD_MARGIN: float = 1.30
    # Close enough to the circle to stop flying the join leg and start
    # tracking the turn.  Not zero: the join is a straight line at a tangent,
    # so the two laws agree in the limit and a band avoids chattering between
    # them at the crossover.
    HAC_JOIN_M: float = 1200.0
    # **The cone is entered on distance to the field, not on altitude**, and
    # that is the whole reason it tolerates a bad entry.  A fixed handover
    # altitude asks the glide to arrive at one energy again; a fixed distance
    # asks it only to arrive, and hands whatever height is left to the one
    # phase that can spend an arbitrary amount of it.
    # **Small on purpose: this is a backstop, not the handover.**  At 60 km
    # it fires while the glide still has tens of kilometres of range
    # authority left and has not finished using it -- measured, arrivals
    # handed over 38-51 km short because the distance test beat the altitude
    # one.  The glide owns the vehicle until its own target altitude; this
    # only catches an arrival so flat it reaches the field first.
    HAC_ENTRY_DIST_M: float = 3000.0
    # **The veto on entering the cone at all.**  Turn radius is
    # ``v^2 / (g tan(bank))``: 9 km at Mach 0.9, 30 km at Mach 1.8, 250 km
    # at Mach 4.7.  Above this there is no cone to fly and the glide keeps
    # the vehicle.
    # Raised from 1.10 after a batch that entered the cone at 14 km with
    # 18 km of path in hand and a 340 degree turn to fly, which needs 19.
    # A cone is a cone and not a cylinder: entering it faster means
    # entering it *higher*, flying the wide end of it while still quick and
    # letting the radius shrink as the speed comes off.  The radius the
    # airframe can hold at Mach 1.5 is 21 km at 45 degrees of bank, which
    # is the wide end.
    HAC_ENTRY_MACH: float = 1.50

    # -- angle of attack ---------------------------------------------------
    # Measured: max lift at 30 deg at every altitude, max L/D at 20 deg, and
    # *zero* lift at 90 -- a cylinder in crossflow has none of that structure.
    ALPHA_MAX_DEG: float = 32.0             # past 30 the lift curve turns over
    # **The glide's own ceiling, separate from the one the deorbit is aimed
    # with.**  ``ALPHA_MAX_DEG`` is a lift argument and it is also a corner of
    # the deorbit search's box, so raising it moves the burn (flown at 40 in
    # the sim: 6 of 8 never reached the interface).  But below Mach ~3.5 the
    # shuttle's glide runs out of sink with bank pinned at ``BANK_MAX_DEG``
    # and alpha pinned at 32 -- ``long`` climbing +500 -> +8000 m in the last
    # 60 km (LOG3416, LOG3440) -- while its own table offers CdA 145 at 40
    # deg against 86 at 30 (Mach 3).  Drag is the sink control that needs no
    # roll, and the learned ceiling (``ratchet_alpha``) still bounds it by what
    # the vehicle has shown it holds.  0 means ``ALPHA_MAX_DEG``.
    GLIDE_ALPHA_MAX_DEG: float = 40.0  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **The ceiling from the wing's own lift plateau**, per Mach: the
    # highest alpha whose ClA is still within this fraction of the row's
    # peak.  The shuttle's table is flat from 30 to 40 deg at Mach 3-5
    # (within 4%) while CdA grows ~70%, and falls 18% by 50: drag for free up
    # to the plateau's edge, lift and control given away past it.  Sim:
    # fixed 36/40/45 all nulled the arrival; 60 and 90 doubled-to-tripled
    # the alpha spread and lost landings (LOG3612-3617).  0.05 puts the
    # shuttle at ~41 hypersonic and ~31 subsonic; a dimensionless fraction
    # of any wing's own peak.  0 keeps ``GLIDE_ALPHA_MAX_DEG``.
    GLIDE_ALPHA_PLATEAU: float = 0.0
    # **The measured flap brake as the glide's third energy control.**
    # ``autopilot.glide_flap_brake``.  Needs ``AIRBRAKE_OPPOSED_FLAPS`` and
    # ``AIRBRAKE_MEASURED`` (the set it deploys is the one measured in vacuum
    # at the start of the flight).  The journal's probe of that set on the
    # shuttle: L/D 0.62 -> 0.54 at Mach 6, 1.10 -> 1.00 at Mach 2 -- a brake
    # at every Mach, and a drag control that asks nothing of the yaw axis,
    # unlike the alpha that ``GLIDE_ALPHA_MAX_DEG`` spends.
    GLIDE_FLAP_BRAKE: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **The flap brake yields to roll** (``Autopilot.roll_needs_the_flaps``):
    # it deploys the elevons, which are the roll surfaces, and both glide
    # losses of control began the tick it went out (LOG3680, LOG3690 -- the
    # user's "not rolling at all").  Stowed while the flown bank is more than
    # ``BANK_RATE_SAT_DEG`` off the command or the sideslip passes
    # ``BANK_RATE_SLIP_TOL_DEG``; not redeployed until the bank has held for
    # ``FLAP_BRAKE_ROLL_SETTLE_S``.  Applies to the glide and the cone.
    FLAP_BRAKE_YIELDS_TO_ROLL: bool = True
    FLAP_BRAKE_ROLL_SETTLE_S: float = 5.0
    # Long past the reserve by this much before it deploys; it stows when the
    # brake-stowed prediction is back on the reserve.
    GLIDE_FLAP_ON_M: float = 1000.0
    # Only at or below this Mach (0: any).  The trouble it is for is the
    # terminal glide's (failure 98); the hypersonic one is untested.
    GLIDE_FLAP_MACH_MAX: float = 0.0
    ALPHA_MIN_DEG: float = 0.0
    # The *solve's* floor, which is not the same number and is the single most
    # important line in this file.  Range against angle of attack is U-shaped:
    # measured offline from an 80 km orbit it reads 1610 km at 5 deg, falls to
    # a minimum of 1265 km at 20 deg, and climbs to 1823 km at 32.  Max L/D
    # gives the *shortest* flight, because a high angle of attack makes enough
    # lift to hold the vehicle up where the air is thin and it glides on.
    # A Newton solve on a function with an interior minimum will walk downhill
    # away from the answer as happily as towards it, and the first closed-loop
    # run did exactly that -- saturating at 32 deg while 62 km short.
    # Floored at the minimum, the whole usable range is monotone increasing,
    # and there is still 560 km of authority between the corners.
    SOLVE_ALPHA_MIN_DEG: float = 20.0
    # **But that floor is a hypersonic fact and it is wrong subsonically.**
    # 20 deg is the bottom of the *range* minimum at Mach 5-7, so a Newton
    # step below it walks the wrong way -- that is what the floor is for.
    # Subsonically the polar has no such interior optimum: `planeprobe`
    # measured best glide at **10 deg, L/D 3.39**, and 20 deg is well past
    # it, where extra angle of attack is only drag.
    #
    # Measured in flight, against the deceleration and flight-path-angle
    # rate the vehicle actually achieved: at Mach 0.9 and 13 km it was
    # gliding at **L/D 0.41** against a modelled 1.06 and an airframe
    # capable of 3.39.  It was not gliding, it was falling, and that is
    # where the 40-70 km shortfall is spent.  Below ``SOLVE_ALPHA_SUB_MACH``
    # the floor relaxes to this.
    SOLVE_ALPHA_MIN_SUB_DEG: float = 8.0
    SOLVE_ALPHA_SUB_MACH: float = 1.2
    # **An instrument, not a flight mode.**  Set non-zero, every angle of
    # attack commanded from COAST onward is replaced by this one, downstream
    # of the solve and every clamp (see ``Autopilot.aim``).  It exists to
    # measure one thing the project has never measured: what this airframe
    # can hold at high alpha, against dynamic pressure, read off
    # ``aoa=cmd/actual`` and ``q=`` in the log.  A flight flown with it on
    # does not reach the runway and is not a landing measurement.
    BROADSIDE_PROBE_DEG: float = 0.0
    # **The same instrument on the yaw axis, and the same reasoning.**  A
    # forward slip is how a glider with no spoilers dumps energy: yaw the
    # nose off the velocity vector and the fuselage's side area does the
    # braking.  This airframe is the extreme case of that -- the swept table
    # reads ``CdA`` **29-33 m^2 broadside against 0.8 at zero**, on a craft
    # whose whole approach ``CdA`` is about 5.6 -- and **nothing in either
    # autopilot has ever commanded a sideslip**.  Flown, the vehicle sits
    # within 1-2 degrees of zero slip in every phase (median 0.8 in GLIDE,
    # 2.1 in APPROACH over 2337 ticks), so the authority is entirely unused.
    #
    # Whether it can be *held* is the open question and the only one this
    # measures.  (Correction 2026-09-23: the surfaces are live, not
    # disabled -- see ``ENABLE_CONTROL_SURFACES``.)  With every control surface's axes disabled this vehicle
    # yaws on reaction wheels alone (15 kN m), against a weathercock moment
    # that grows with ``q`` -- exactly the shape ``Holdable`` already learns
    # for alpha.  So: set non-zero and every ``aim`` from COAST onward yaws
    # the nose this far off the velocity vector, downstream of everything,
    # and the log carries ``slip=cmd/actual`` beside ``q=``.  **An
    # instrument, not a flight mode.**  If the wheels cannot hold five
    # degrees at approach ``q`` the idea is dead and this is how it dies
    # cheaply; if they hold twenty, it is a bigger lever than the split
    # rudder by an order of magnitude.
    SLIP_PROBE_DEG: float = 0.0
    ENTRY_ALPHA_DEG: float = 22.0# hot phase: maximum drag with lift
    GLIDE_ALPHA_DEG: float = 20.0           # max L/D, for range
    ALPHA_RATE_DEG_S: float = 3.0           # how fast the command may move
    # Above this multiple of the approach speed the angle of attack is free to
    # brake with everything it has; below it, it becomes a speed hold on the
    # approach speed.  Without the hold the entry arrives at the gate at the
    # entry angle's equilibrium glide speed -- measured, 37 m/s, against a
    # 37.1 m/s stall -- and cannot flare.
    # **The glide's arrival speed and the approach's touchdown speed are two
    # different jobs and were one constant.**  ``alpha_limit_for_speed`` caps
    # the angle of attack so the entry arrives *able to land*; ``approach``
    # targets the speed to touch down at.  Both read
    # ``APPROACH_FACTOR x STALL_SPEED_M_S``, so raising the touchdown speed
    # to 115 m/s also told the glide to hold 115 -- and holding 115 in a glide
    # means about six degrees of angle of attack, where this airframe's L/D
    # is 1.15 and the path is 41 degrees. Measured: six flights at that
    # setting landed 1.0 to 9.4 km **short**, hitting terrain at 107-116 m/s
    # without ever reaching the gate.
    #
    # There is no conflict once they are separate. The glide arrives near
    # best glide, which is what reaching the runway needs, and the approach
    # spends the gate's surplus *height* on speed: from 2600 m with 766 m in
    # hand, 86 -> 115 m/s costs 261 m of it.  Trading height for speed is
    # the one thing a glider on final can do cheaply, and it is the thing the
    # old shared constant forbade for the whole entry.
    GLIDE_ARRIVAL_FACTOR: float = 1.80      # x stall: 86 m/s at the gate
    SPEED_HOLD_FACTOR: float = 1.6
    SPEED_HOLD_KP: float = 0.30             # deg of alpha per m/s of excess
    # **A ceiling is only half of a speed hold, and the missing half is where
    # the flight is lost.**  ``alpha_limit_for_speed`` returns
    # ``ALPHA_MAX_DEG`` -- no constraint at all -- whenever the vehicle is
    # above the hold threshold, which is exactly the state a diving vehicle
    # is in: the hold disengages precisely when it is needed.
    #
    # What the solve does with that freedom is measured.  The glide is solved
    # on the along-track miss at the gate's *altitude*, and with a surplus in
    # hand the cheapest way to null it is to **lower** the angle of attack:
    # less lift sinks the arc onto that altitude sooner, and the predicted
    # miss converges nicely.  In ``logs/LOG753`` it converged to +1.9 km
    # while the vehicle arrived over the gate at 1381 m doing 96.7 m/s with
    # **95.7 of it straight down** -- an 82 degree dive, at the angle of
    # attack floor, with the bank pinned at 70.  The energy was never
    # dissipated; it was moved out of height and into speed, and nothing
    # after the gate can spend speed.  Every flight of the current default
    # ends this way and the landing "miss" that results is where a falling
    # brick happened to land.
    #
    # So subsonically the hold is two-sided: the same law used as a floor as
    # well as a cap.  Diving is range authority the solve should not have,
    # and taking it away leaves the division of labour the entry design
    # already claims -- angle of attack holds the speed, bank does the
    # ranging.  Only subsonically, because above ``SPEED_FLOOR_MACH`` the
    # range curve has the interior optimum ``SOLVE_ALPHA_MIN_DEG`` exists to
    # guard and a floor there would fight it.
    # **Flown and adopted.**  Three flights each off one save, everything
    # else identical, on the same deorbit pass:
    #
    #   |            | along mean | sd  | \|across\| median | max |
    #   |------------|-----------|-----|-----------------|-----|
    #   | cap only   | +11.7 km  | 6.5 |     8.8 km      | 12.7|
    #   | two-sided  | **+2.2**  | 6.5 |   **3.2**       | 5.8 |
    #
    # -- and the mechanism shows up where it was predicted to.  Without the
    # floor the glide handed the approach a vehicle at 219 m/s and 74 degrees
    # off the runway heading, where a 35 degree bank has a **7 km** turn
    # radius against 5 km of runway to go; with it, one of the three arrived
    # at the gate 29 m off the centreline and landed 13 m off it.  The
    # along-track scatter is untouched, because that has a different cause
    # (failure 16).
    SPEED_FLOOR_ON: bool = True
    SPEED_FLOOR_MACH: float = 1.2
    # **And only while there is surplus to spend.**  The floor exists to stop
    # a *long* vehicle buying range by diving.  Applied to a short one it
    # does the opposite of what is wanted: measured in ``logs/LOG867``, the
    # last 25 km of the entry were flown at a pinned 20 degrees of angle of
    # attack -- where this airframe glides at L/D 1.33 against 2.10 at its
    # best-glide 12 -- and the predicted miss bled from -6.4 km to -14.1 km
    # over exactly that stretch.  The vehicle was braking while it was short.
    #
    # So the floor is gated on the glide's own prediction, and the gate is
    # hung on ``env`` rather than passed through ``Steer`` for the reason
    # ``holdable`` is: sixteen propagations cannot each forget an attribute
    # the environment carries.  A limit nobody passed on is a limit nobody
    # flies.
    SPEED_FLOOR_WHEN_LONG: bool = True
    # How much of the dive to leave the solve, in degrees below the hold.
    # Zero pins the angle of attack subsonically and leaves bank the only
    # control; a little slack keeps a usable gradient for the Newton step.
    SPEED_FLOOR_SLACK_DEG: float = 2.0
    # A gentler gain than the cap's.  ``SPEED_HOLD_KP`` is sized for a hold
    # that only ever *removes* angle of attack, where being wrong costs a
    # little range; as a floor the same 0.30 pins the command at
    # ``ALPHA_MAX_DEG`` from about 130 m/s up, which leaves the solve no
    # alpha authority at all in the fast subsonic regime and the trajectory
    # fully determined.  A floor wants to make diving expensive, not to fly
    # the vehicle.
    SPEED_FLOOR_KP: float = 0.15

    # -- bank --------------------------------------------------------------
    # Bank is the energy and range control; its sign is the cross-track one.
    BANK_MAX_DEG: float = 70.0
    # And a floor, for the same reason ``SOLVE_ALPHA_MIN_DEG`` exists: range
    # against bank is not monotone at the shallow end either.  Measured
    # offline, holding 30 deg of angle of attack, the entry range goes
    # 1730 km at 0 deg of bank, *up* to 1872 km at 30, then down to 1679 at 50
    # and 1432 at 70.  A little bank lengthens the flight, because the lift it
    # gives up vertically is more than repaid by the drag it stops making.
    # Floored at 30 the whole usable span is monotone decreasing, the reachable
    # maximum is *larger* than it was at zero bank, and a Newton step on it
    # means something.
    SOLVE_BANK_MIN_DEG: float = 30.0
    # **The glide's own floor** (0: ``SOLVE_BANK_MIN_DEG``).  The 30 above
    # is the old airframe's non-monotone range; the shuttle's is monotone
    # in bank at every Mach (offline from LOG4498's swept table, wings
    # level always goes furthest), and the floor makes it carry 30 deg
    # where the solve wants less: LOG4498 sits on it at Mach 5.5-5.2, so
    # the second reversal there is a +-30 flip energy never asked for.
    # Glide only; the deorbit keeps its aim.  ``guidance.glide_bank_min``.
    GLIDE_BANK_MIN_DEG: float = 0.0
    BANK_RATE_DEG_S: float = 8.0
    # **Measured at runtime instead** (``rollrate.RollRate``): the 8 above
    # has no derivation, was set on the old capsule, and the shuttle cannot
    # follow it -- the command outran the vehicle, kRPC's controller stopped
    # rolling at 20 deg of pointing error, and the GUI showed no roll input
    # (LOG3679).  On, the glide's bank command slews at the rate the vehicle
    # was last seen to deliver, and 8 is only the prior before the first
    # sample.
    BANK_RATE_MEASURED: bool = True
    # A tick counts as a sample only when the command led the flown bank by
    # at least this -- the vehicle was being asked for more than it gave.
    BANK_RATE_SAT_DEG: float = 5.0
    # The airframe's roll tolerance: sideslip past this during a roll means
    # it was rolled faster than it can take, and the estimate is pulled down
    # by (tol/|slip|) per BANK_RATE_TAU_S.  Steady holds read ~1 deg;
    # reversals that went wrong 15-50.
    #
    # **0 (off) since 2026-10-02.**  At 5 it did most of its pulling in COAST
    # and the first GLIDE seconds, at q 0-150 Pa, where "sideslip" is the
    # nose wandering in vacuum: replayed over the logs it cut the shuttle's
    # limit from 7-10 to 1.3-2.8 deg/s before the first reversal, so the
    # second one crawled into q ~3000 and departed (LOG4459).  Off, on chain6
    # + RCS_PITCH_OFF_IN_GLIDE, 16 v 16 (`rot-sliptol-1002`,
    # `rot-sliptol2-1002`): final |along| <= 5 km 11/16 vs 6/16, intact 3 vs
    # 0.  Null on `qs_plane` (limit pinned at the 30 ceiling either way,
    # `rot-sliptol-plane-1002`).  A tolerance that only counts sideslip once
    # the air can make it would be the law to try if it is wanted back.
    BANK_RATE_SLIP_TOL_DEG: float = 0.0
    # The held peak decays over this much *sampled* time, and a sideslip
    # past the tolerance pulls it down on the same scale -- a couple of
    # reversals, so it follows q without one slow tick discarding it.
    BANK_RATE_TAU_S: float = 20.0
    # How far above the held peak the command may slew, so the vehicle is
    # always asked slightly more than it last gave and the peak can climb.
    BANK_RATE_PROBE: float = 1.25
    BANK_RATE_MIN_DEG_S: float = 1.0
    BANK_RATE_MAX_DEG_S: float = 30.0
    # Two samples further apart than this are not a rate (a warp, a pause).
    BANK_RATE_MAX_GAP_S: float = 5.0
    # **The other way to answer failure 16: do not act on the spike.**
    # Replacing the bank the propagation is shown was tried and is worse (see
    # above), because the transit is real and pretending it away biases the
    # prediction short.  But the *prediction made during the transit* is
    # still one of a trajectory nobody flies, and the damage measured in
    # ``logs/LOG789`` is not the reading -- it is the angle of attack being
    # yanked from 32 degrees to its 20 degree floor in response to it, which
    # then throws the tracking error, drops the learned ceiling, and takes
    # the range authority away for the rest of the entry.
    #
    # So hold the angle of attack while the lean is in transit and let the
    # bank finish its reversal first.  ``SOLVE_BANK_MIN_DEG`` is 30, so a
    # commanded magnitude below this threshold only happens mid-reversal:
    # the test is "is the vehicle between the stops", not a guess.  The bank
    # is left alone -- it is the thing doing the reversing.
    #
    # 0 disables it.  This is a *rate* limit in disguise and the honest
    # version of the same idea would be to slow ``ALPHA_RATE_DEG_S``, except
    # that the angle of attack genuinely needs to move quickly elsewhere in
    # the entry; what is wrong is moving it in response to one particular
    # four-second lie.
    SOLVE_HOLD_THROUGH_REVERSAL_DEG: float = 25.0
    # **And the premise of that threshold is false for most of the entry.**
    # The paragraph above says a commanded magnitude below 25 degrees "only
    # happens mid-reversal", because ``SOLVE_BANK_MIN_DEG`` is 30.  It is not
    # the solve's floor that decides the late lean: ``cross_bank_floor``
    # narrows the band with the range to run, and below about 27 km the solve
    # asks for a few degrees of lean and holds it.  Measured over the last
    # five default flights, the fraction of GLIDE ticks with |bank| < 25 --
    # every one of which freezes the angle of attack:
    #
    #     LOG935  83%   frozen from 27365 m, alpha 22.0 -> 14.3
    #     LOG930  60%   frozen from 27680 m, alpha 20.0 -> 14.5
    #     LOG929  72%   frozen from 43024 m, alpha 31.0 -> 14.5
    #     LOG927  57%   frozen from 26418 m, alpha 20.0 -> 14.2
    #     LOG925  59%   frozen from 28511 m, alpha 24.7 -> 15.3
    #
    # So the range solve's primary control is disconnected for the whole
    # second half of every entry, and the only thing still moving the angle
    # of attack is ``alpha_limit_for_speed`` pulling it *down* -- which is
    # why the trace is monotone in every flight.  That is exactly the stretch
    # where failures 22 and 23 watched the predicted miss bleed from -3 km to
    # -13 km with "neither control moving".  It was not saturated and it was
    # not at a stop it had chosen: it was held.
    #
    # The guard's real job is failure 16's and is worth keeping -- do not
    # yank the angle of attack in response to the four-second wings-level lie
    # a transit tells the propagation.  What is wrong is the *test*.  "Is the
    # vehicle between the stops" is a question about whether the rate-limited
    # command has arrived at the lean it is committed to, not about how big
    # that lean is, and the loop already knows both numbers.  See
    # ``guidance.bank_in_transit``.  Set False for the old magnitude test.
    SOLVE_HOLD_ON_TRANSIT: bool = True
    # How close to the committed lean counts as arrived.  Floored on one
    # tick's worth of slew, because a command that cannot be reached this
    # tick will still be moving next tick; the constant is the floor under
    # that, so a fast tick does not read every lean as a transit.  The
    # dangerous direction is *too small* -- that is the frozen-alpha bug
    # above wearing a different threshold -- so it is a few degrees and not
    # a few tenths.
    SOLVE_HOLD_TRANSIT_DEG: float = 5.0
    # **How often the lean may reverse at all.**  The other half of the same
    # problem, and the measurement is stark: an entry makes 23-26 reversals
    # and spends **21-23% of its ticks between the stops**, which is a fifth
    # of the flight during which the propagation is of a wings-level vehicle
    # and reads tens of kilometres long.  A reversal takes seventeen seconds
    # to slew at ``BANK_RATE_DEG_S``, so the relay is switching faster than
    # its own actuator can follow -- the textbook way to make a lagged relay
    # chatter, and failure 10e measured the resulting limit cycle without
    # naming the rate as the problem.
    #
    # A minimum dwell is the textbook answer: having committed to a lean,
    # hold it long enough for the vehicle to actually get there and for the
    # cross-track to respond, before considering the next one.  It costs
    # cross-track authority in principle and the measurement will say whether
    # it costs any in practice.  0 disables it.
    BANK_REVERSAL_DWELL_S: float = 0.0
    # **Stated in the actuator's own units, because that is what it is about.**
    # The quantity that matters is whether a lean outlasted the slew that
    # established it, and a stop-to-stop slew is
    # ``2 * BANK_MAX_DEG / BANK_RATE_DEG_S`` -- 17.5 s on this airframe and a
    # different number on the next one.  A dwell in seconds is this craft's
    # roll rate hard-coded into a policy; in slews it is the policy.  The two
    # combine with ``max`` so a seconds value still overrides for one
    # experiment; 0 in both disables it.
    #
    # **A faster actuator is not an alternative.**  Doubling
    # ``BANK_RATE_DEG_S`` to 16 was tried, on the reasoning that a shorter
    # slew is a smaller duty cycle.  Measured, it made **29** reversals
    # against 23-26 at the old rate and left the transit fraction at 20%:
    # the loop simply chattered faster.  A relay's duty cycle is set by the
    # loop, not by how quickly the actuator can answer, and the only thing
    # that changes it is limiting the switching rate itself.
    # **Adopted at 2.5 slews, and only while supersonic.**  Measured against
    # the same configuration without it: along-track -3.4 km +- **0.3** on
    # two flights, where the control scatters +-8 to +-11 km.  The cost is
    # cross-track -- one lean held for 44 s lets the offset grow, and those
    # two came down 3.3 km off the centreline.
    #
    # That cost is avoidable because it is not uniform: far out, a
    # cross-track error is cheap (there is crossrange authority and time to
    # spend it) and the divergence this guards against happens at 30-50 km;
    # near the gate the cross-track *is* the miss and a reversal is the only
    # thing that can fix it.  So the dwell is lifted subsonically -- the same
    # boundary ``SPEED_FLOOR_MACH`` draws, deliberately reusing it rather
    # than introducing a second definition of where the terminal glide
    # begins.
    BANK_REVERSAL_DWELL_SLEWS: float = 2.5
    # **The dwell in its honest form: not a clock at all.**  What "outlast
    # your own actuator" actually means is *have you got there yet* -- a lean
    # should not be abandoned before it has been established -- and that is a
    # question about the bank angle, not about seconds.  A reversal away from
    # the latched side is refused until the vehicle has actually reached that
    # side, meaning ``SOLVE_BANK_MIN_DEG`` of lean on it, or whatever smaller
    # magnitude the solve asked for.
    #
    # It needs no constant, it cannot be mis-tuned, and it scales itself:
    # supersonically the solve commands 40-70 degrees and settling takes most
    # of a slew, while subsonically it commands much less and settling is
    # quick -- which is exactly the behaviour the clock version had to be
    # *switched off* subsonically to imitate.  Measured with the clock: the
    # supersonic reversal count fell from ~24 to 8-11 and ``|long|`` at a
    # bank stop came down to 6-46 m, but the subsonic phase, where the clock
    # was lifted, still turns over every 12-18 s against a 17.5 s slew -- so
    # it is permanently in transit, and that is where the flights that still
    # diverge lose it.
    BANK_REVERSAL_SETTLE: bool = True
    # **The coast reversed on every azimuth crossing, at no dynamic pressure.**
    # ``run_coast`` leaned toward whichever side the gate was on through
    # ``bank_toward``, which has no hysteresis: with the ground track near
    # the bearing to the gate the sign flipped every time they crossed.
    # Measured on the shuttle (LOG3404/3405 game, LOG3416/3417 sim): **8-9
    # full +-30 degree reversals in COAST at Mach 7 and q 0-120 Pa**, against
    # 3 in the whole hypersonic glide -- rolls the air cannot coordinate
    # (its yaw axis is 15 kN m of wheel and nothing else up there), so the
    # vehicle reached the interface mid-wallow.  They buy nothing: at that q
    # the lean moves nothing but the attitude.  True routes the coast's sign
    # through the glide's own ``_bank_sign`` deadband, seeded from the lean
    # it already holds, so it is chosen once and reversed only for a real
    # azimuth error.
    COAST_BANK_LATCH: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    GLIDE_BANK_SWEEP_RATE_DEG_S: float = 1.0      # the planned crossing
    GLIDE_BANK_SWEEP_RATE_MIN_DEG_S: float = 0.3  # the crossing's trim range
    # 2.0 lost it: the crossings that trimmed up to 2 deg/s at q 2500-3600
    # slipped 25-37 deg, those that stayed under 1 held 6 (farm, LOG4608-4621).
    GLIDE_BANK_SWEEP_RATE_MAX_DEG_S: float = 1.0
    GLIDE_BANK_SWEEP_HORIZON_S: float = 1500.0    # latest start searched
    GLIDE_BANK_SWEEP_START_STEP_S: float = 20.0   # the start bracket's first step
    GLIDE_BANK_SWEEP_STEP_DEG_S: float = 0.1      # the rate bracket's first step
    GLIDE_BANK_SWEEP_TOL_M: float = 200.0
    GLIDE_BANK_SWEEP_ITERATIONS: int = 6   # propagations per tick, at most
    GLIDE_BANK_SWEEP_UNTIL_MACH: float = 0.0  # below it the relay; 0: the whole glide
    GLIDE_BANK_SWEEP_FALLBACK_M: float = 10000.0  # no start nulls it: the relay picks the side
    # ...for this many ticks running: the first tick's search can run out of
    # evaluations short of the root, and acting on that crossed three farm
    # flights at Mach 7 for a '64 km' miss that was 260 s of hold away.
    GLIDE_BANK_SWEEP_FALLBACK_TICKS: int = 10
    # **Unload alpha through the crossing** (0: off).  A body roll at alpha
    # a is sideslip in proportion to sin a (failure 99), and 9 of 13 farm
    # crossings slipped 15-28 deg even at 0.5-1 deg/s.  Caps the command
    # while the lean is crossing; the plan's propagations do not model it
    # (~100 s of the glide), so read the arrival with that in mind.
    GLIDE_BANK_SWEEP_CROSS_ALPHA_DEG: float = 0.0
    GLIDE_SIGN_LAW_LOG_S: float = 20.0  # game s between the sign laws' prediction lines
    # The azimuth error a reversal waits for, shrinking with range to go.
    # Wide early, so the entry does not chase cross-track it will fly out of
    # anyway; tight at the end, where azimuth *is* the miss.
    AZIMUTH_DEADBAND_PER_KM: float = 0.012  # deg of error per km still to run
    AZIMUTH_DEADBAND_MIN_DEG: float = 0.5
    AZIMUTH_DEADBAND_MAX_DEG: float = 12.0
    # **The reversal has to be triggered on the cross-track, not only on the
    # azimuth.**  A deadband that widens with range to go permits a
    # cross-track *proportional* to range: at 500 km the azimuth deadband is
    # 6 deg, and 23 km of offset at 500 km is 2.6 deg -- inside it.  Measured
    # in game, the bank held one sign for the whole upper glide while the
    # predicted cross-track grew 469 m -> 4 km -> 11 km -> 23 km, and by the
    # time the azimuth test finally fired there was no authority left to
    # undo it.  This is an absolute threshold on the quantity that actually
    # matters -- the propagator reports it every tick -- and the runway is
    # 2.4 km long, so a few hundred metres is already off the centreline.
    #
    # **But it cannot be absolute at every range, and as one it drove a limit
    # cycle that is most of the glide's run-to-run scatter.**  See failure
    # 10e.  ``Prediction.cross`` is the offset at the *gate* of an entry the
    # propagator models as reversing -- no lateral lift at all -- so it is
    # the current lateral velocity carried forward, which is a *rate* wearing
    # a distance's units.  Relay-testing a rate against a fixed 500 m, with
    # an actuator that needs thirteen seconds to roll from one stop to the
    # other, is a bang-bang loop with lag: measured in game it reversed on an
    # 18-second period through the level-off, overshooting the band nearly
    # sevenfold to 3.4 km, with the roll rate-limited at ``BANK_RATE_DEG_S``
    # for six ticks in ten.  Half the entry was spent mid-reversal, where the lift is nearly
    # all vertical and the vehicle stops sinking.
    #
    # So the band scales with range to run, like the azimuth one, and for the
    # same reason: three kilometres of predicted cross-track with 250 km left
    # is nothing -- the vehicle has tens of degrees of crossrange authority
    # to spend -- while three kilometres at the gate is the whole miss.  The
    # floor is the runway's own scale, which is absolute and does not scale
    # with anything; the cap keeps it far below the 23 km the azimuth test
    # alone permitted.
    #
    # The two tests stay commensurate rather than one doing all the work:
    # at 250 km to run the azimuth band of 3.0 deg permits about 13 km of
    # cross-track, so a 5 km cap still leads by 2.6x.  At the old 500 m it
    # led by 26x, which is why the azimuth test never fired and the cross
    # test chattered.
    # **Flown, and it does not work.  Default 0, which is the fixed 500 m
    # band this replaced.**  Nine flights against eight of the old band, one
    # entry state, homogeneous farm, same deorbit pass:
    #
    #   |                | along mean | sd  | |cross| median | reversals |
    #   |----------------|-----------|-----|----------------|-----------|
    #   | fixed 500 m    | -1.8 km   | 9.1 |     24 m       |    8.2    |
    #   | scaled, 5 km cap| -4.8 km  | 5.8 |   1114 m       |    5.2    |
    #
    # The scaling does exactly what it was built to do -- the limit cycle is
    # halved, and the roll spends 22% of ticks against its rate limit instead
    # of 36% -- and it buys nothing.  Along-track is a wash; cross-track is
    # forty-five times worse at the median, because a 5 km cap permits an
    # offset the vehicle then has to fly out late.  That is a smaller version
    # of the failure the fixed band was introduced to fix.
    #
    # So the *observation* in failure 10e stands (an 18 s relay cycle
    # overshooting its band sevenfold, measured) and the *hypothesis* it was
    # built on does not: the cycle is not what makes the glide scatter.
    # Left in at 0 the way ``ALPHA_TRACKING`` is left in at (), because the
    # knob is worth keeping for a better-targeted experiment -- a cap nearer
    # 1 km, say -- and the measurement is worth not repeating.
    #
    # **And the objection that killed it has since been removed by a phase
    # downstream.**  What sank the wide band was cross-track: 1114 m at the
    # median against 24, "an offset the vehicle then has to fly out late".
    # The heading alignment cone flies it out *as its own manoeuvre* -- it
    # picks which way to turn from where the vehicle actually is, and the
    # measured cross-track at the flare is tens of metres from arrivals
    # kilometres off the centreline.  The cost side of that trade is now
    # near zero, and the benefit side was never nothing: the same table
    # shows the along-track sd going **9.1 -> 5.8 km** and the reversals
    # 8.2 -> 5.2, which was read as "a wash" on the mean.
    #
    # It matters because of what the reversals cost. At ``BANK_RATE_DEG_S``
    # of 8 a stop-to-stop reversal is 17.5 s, and measured over 14 flights
    # the glide spends **87% of its time with the lean off its stops** --
    # 24 reversals in 595 s. The bank magnitude is the energy control and it
    # is almost never settled, which is the one thing a drag-reference
    # entry needs it to be. CLAUDE.md's rule about a guard whose reason no
    # longer survives inspection, running the other way: here it is a
    # *rejection* whose reason no longer survives.
    CROSS_DEADBAND_PER_KM: float = 20.0# m of predicted cross per km to run
    CROSS_DEADBAND_MIN_M: float = 500.0     # the runway's own scale
    CROSS_DEADBAND_MAX_M: float = 40000.0

    # **The lean has to be off before the gate, because the approach cannot
    # turn.**  The glide spends surplus range with bank, and bank is also the
    # one thing that must be small at handover: a vehicle arriving in a hard
    # turn arrives pointing the wrong way.  Measured, ``logs/LOG950`` -- a
    # flight that did everything else right, +1.4 km at the gate and 1.1 m/s
    # of sink at touchdown -- held **bank -70 for the last 1.2 km of
    # altitude** to shed 1.5 km of surplus, and handed the approach a vehicle
    # 81 degrees off the runway heading with 138 m/s of lateral velocity and
    # 2.6 km to run.
    #
    # The approach's capture law is not at fault and re-tuning it cannot
    # help: at 140 m/s and its 40 degree limit the turn radius is 2.4 km, so
    # 81 degrees of heading needs 3.4 km of arc against the 2.6 it has. It
    # swept through the centreline to +910 m and touched down 347 m off it.
    # The arithmetic is a property of the airframe, not of a gain.
    #
    # So the glide gives the lean up on a taper: full authority until
    # ``GLIDE_ALIGN_RANGE_M`` from the gate, then linearly down to
    # ``GLIDE_ALIGN_BANK_DEG`` at it.  At 15 degrees and 140 m/s the heading
    # rate is about 1.1 deg/s, so the last stretch is still worth tens of
    # degrees of alignment -- it is a cap on the *rate* the vehicle can be
    # left rotating at, not a ban on turning.  Surplus range that the taper
    # will not let the glide spend is handed to the approach's S-turn, which
    # exists for exactly that (``APPROACH_SCURVE_M``), and arrivals are now
    # long rather than short, so there is surplus to hand over.
    GLIDE_ALIGN_RANGE_M: float = 15000.0
    GLIDE_ALIGN_BANK_DEG: float = 15.0

    # -- the glide solve ---------------------------------------------------
    # Same shape as boosterland's solve_steer: propagate, measure the
    # sensitivity with probes, invert, and verify the answer lands nearer
    # before committing it.
    SOLVE_ALPHA_PROBE_DEG: float = 4.0
    SOLVE_BANK_PROBE_DEG: float = 15.0
    SOLVE_DEADBAND_M: float = 60.0          # metres of height over the gate
    SOLVE_MIN_AUTHORITY_M: float = 50.0

    # How finely to bracket.  A bracket rather than a gradient because range
    # against angle of attack has an interior optimum on this airframe and a
    # Newton step walks away from the answer as happily as towards it -- the
    # same reason ``SOLVE_ALPHA_MIN_DEG`` exists.  Six intervals over the
    # permitted span is about 2 degrees on this vehicle, which is inside the
    # angle it holds its command to anyway.
    SOLVE_MAX_RANGE_PROBES: int = 6

    # **Crossed against the entry state, which is what settles it.**  Three
    # flights per cell, reserve on against off, on three saves:
    #
    #     qs_plane        -0.4 km   against  -3.2 km       +2.8
    #     qs_plane_high  -12.2 sd 6.0        -18.6 sd 2.1  +6.4
    #     qs_plane_inc   -62.0 sd 3.2        -68.7 sd 3.2  +6.7
    #
    # Aiming long helps on **every** entry state, and 4 km of reserve buys
    # 6-7 km -- more than one for one, because aiming long earlier changes
    # the arc the solve flies rather than only biasing its end.  So the
    # design is sound and this defaults on.
    #
    # What must not be defaulted with it is the *magnitude*.  The amount
    # needed runs from about 3 km to more than 60 across those three
    # entries, deterministically per entry (the spread within a state is
    # 0.4 to 6 km).  ``GLIDE_RESERVE_M`` below is a calibration for
    # ``qs_plane`` and is wrong everywhere else -- spaceplane failure 50 --
    # so it is left at the older 8000 rather than pinned to one save's
    # answer, and a flight that means the fitted value must say so with
    # ``--set``.  A constant cannot express a quantity that moves by an
    # order of magnitude with the deorbit, and no re-fit will make it; that
    # is the clearest statement this project has of why the prior at
    # ``HOLDABLE_EXTRAPOLATE`` has to be built.
    #
    # **Left off, and the reason is not the measurement.**  What was flown is
    # the reserve held *constant* to the gate -- ``GLIDE_RESERVE_FROM_ALT_M``
    # at the cone's altitude -- at ``GLIDE_RESERVE_M`` 4000.  Switching this
    # on alone ships neither: the schedule below still decays from 45 km and
    # the magnitude is still 8000, a combination nobody has flown.  Turning
    # all three on would instead make one save's calibration a silent
    # default, which is the thing the entry above exists to warn about.  So
    # the landing configuration stays an explicit ``--set`` list, as it
    # already is for the other seven constants, and the flown form is:
    #
    #     --set GLIDE_RESERVE_ON=True --set GLIDE_RESERVE_M=4000 \
    #       --set GLIDE_RESERVE_FROM_ALT_M=12000
    #
    # The decay is a straight-in-era requirement -- "a reserve held to the
    # end is just a long landing" -- and the cone removes it, because long is
    # the side the cone absorbs.  That is why the flown form does not decay.
    GLIDE_RESERVE_ON: bool = True
    GLIDE_RESERVE_M: float = 500.0
    GLIDE_RESERVE_FROM_ALT_M: float = 12000.0
    GLIDE_RESERVE_TO_ALT_M: float = 12000.0

    # -- entry -------------------------------------------------------------
    ENTRY_INTERFACE_M: float = 58000.0      # where to stop coasting and fly
    # **The interface at the top of the air** (``trajectory.past_interface``):
    # the glide takes over where the body's atmosphere begins, as the game
    # reports it, instead of at 58 km -- a Kerbin altitude in the initial
    # commit with no recorded reason, which means nothing on another body.
    # The glide's solve propagates the whole remaining entry, so it is well
    # posed from the first air.  Off until paired.
    ENTRY_INTERFACE_AT_AIR: bool = False
    # -- the upper entry, where the solve has leverage but no authority ----
    # Holdability is discovered, not computed: simulate_aerodynamic_force_at
    # returns a force and not a moment, so the pitching moment at an attitude
    # the vessel is not in is not available at any price.  The command is
    # ratcheted against what the vehicle actually achieves.
    ALPHA_TRACK_TOLERANCE_DEG: float = 6.0
    ALPHA_BACKOFF_DEG: float = 2.0
    # **And the floor under it has to be a statement about the plant.**  It
    # was ``GLIDE_ALPHA_DEG`` -- 20 degrees, the angle the *guidance* wants
    # for range -- so the ratchet walked the ceiling down 28, 26, 24, 22, 20
    # and stopped, with the vehicle achieving 13-15 against 21 commanded and
    # ``Holdable`` already reporting 17.8 for that dynamic pressure
    # (``logs/LOG937``, q = 7.5-8.3 kPa at about 30 km).
    #
    # Everything below 27 km then flew pinned at exactly 20.0 commanded and
    # 14.5 achieved, all the way to 3 km and through M 0.9, 0.5 and 0.3 --
    # and the solve propagated alpha 20 the whole way, which is a prediction
    # of a law nobody flies.  The model's polar at 20 is L/D 1.68; what the
    # vehicle delivered at the 14.5 it could actually hold is 1.43.  The
    # predicted miss bled from -1.0 km at 26 km altitude to -15.3 km at the
    # gate with both controls apparently steady, which is the picture
    # failures 22 and 23 both described and neither explained.
    #
    # A target is not a limit.  The floor here is the lowest angle the
    # subsonic solve is allowed to *command* (``SOLVE_ALPHA_MIN_SUB_DEG``),
    # because a ceiling below that would leave the solve no legal value; the
    # ratchet is two-way (``ALPHA_RECOVER_DEG_S``), so it cannot get stuck
    # low the way a one-way one could.  Failure 26.
    # **And it is a floor under the *measurement*, not a floor on its own.**
    # Used bare it let the ratchet collapse the ceiling at the entry
    # interface, where the air is thin, the attitude controller is still
    # catching up, and a 17-degree tracking error means nothing about trim:
    # ``logs/LOG962`` commanded 30.1 and achieved 13.2 at 57 km, ratcheted to
    # 11 degrees, flew the whole hypersonic entry there making no drag and
    # arrived **90 km long**.  ``ratchet_alpha`` therefore takes this floor
    # only where ``Holdable`` has evidence at the current dynamic pressure,
    # and ``GLIDE_ALPHA_DEG`` where it has none.  Failure 29.
    ALPHA_CEILING_FLOOR_DEG: float = 8.0
    # **And it has to recover.**  A ratchet that only ever lowers turns a
    # transient into a permanent loss of range authority, and on this
    # airframe the ceiling *is* the range control: the measured table runs
    # 1265 km at alpha 20 to 1823 km at 32.  The vehicle holds 30.7 deg
    # against 30.0 commanded at 57 km and only 21 deg at 30 km, so the
    # ceiling legitimately walks down through the dense part -- but a bank
    # reversal also throws the tracking error transiently, and with
    # reversals now firing on cross-track those are common.  Measured in
    # game, the miss grew monotonically as the ceiling fell: -14.9, -24.1,
    # -27.0, -28.3 km.  So the ceiling climbs back at this rate while the
    # vehicle is tracking comfortably -- inside half the tolerance, which it
    # cannot be while failing to hold what it is already commanded.
    ALPHA_RECOVER_DEG_S: float = 0.5
    # **What the vehicle actually holds, against dynamic pressure.**  Nothing
    # here had ever measured trim -- ``planeprobe`` aims the airflow and
    # reads a *force*, and kRPC will not report a moment -- so the guidance
    # was predicting angles of attack the airframe cannot fly.  This is that
    # measurement, taken from the ``aoa=commanded/achieved`` column of every
    # GLIDE line of 16 flights (5467 samples):
    #
    #   q (Pa)        n    mean cmd   achieved   ratio
    #   0-200       209      29.2       29.8      1.02
    #   500-1000    206      30.2       30.8      1.02
    #   1000-2000   348      25.3       24.6      0.97
    #   2000-4000  1251      21.3       18.6      0.87
    #   4000-8000  2685      22.2       19.2      0.87
    #   8000+       462      20.8       16.7      0.81
    #
    # It is a **gain, not a ceiling**: in thin air the vehicle holds what it
    # is asked, and above ~2 kPa it delivers about 85% of it.  The propagator
    # applies this so a prediction is of a trajectory the vehicle can fly,
    # and the solve then asks for more to compensate -- which is the same
    # rule as the arrival-speed cap and ``landing_command`` in the sibling
    # project: predict the law you fly.  Set to () to model perfect tracking.
    # **Flown, and it is worse -- so it is off.**  The measurement above is
    # sound and the propagator applying it is self-consistent, but the
    # *search* then exploits it: less effective alpha is less lift and
    # therefore a shorter predicted range (1265 km at 20 deg against 1823 at
    # 32), so the dv that still lands on the aim is a **smaller** one -- 75
    # m/s where the same state used to solve 140 -- and that is a shallow
    # entry whose extra range does not materialise.  In game it took the
    # shortfall from 45-63 km to **99-118 km**.
    #
    # The lesson is not that the measurement is wrong.  It is that making one
    # half of a model honest, while the thing that consumes it is free to
    # search into the regime where the *other* half is least reliable, is not
    # an improvement.  What this wants is the tracking model **and** a bound
    # on how shallow an entry the search may choose -- the shallow end is
    # also where the skip lives (failure 8), which is the same warning twice.
    # **Re-measured on the attitude-tuned vehicle, 31323 samples over every
    # flight since ``ATTITUDE_TIME_TO_PEAK_S`` went to 3.0**, and the old
    # table is optimistic exactly where the range is decided:
    #
    #   q <= Pa      n     achieved/commanded      was
    #     1000   10221          1.02              1.00
    #     2000    4423          1.01              0.97
    #     4000    5889          0.89              0.87
    #     8000    5279          **0.74**          0.87
    #    16000    4963          **0.72**          0.81
    #    32000     548          0.72              --
    #
    # It matters because the deorbit's authority window flies its long corner
    # with this table on (``guidance.deorbit_window``): believing the vehicle
    # holds 87% of its command at 8 kPa when it holds 74% makes the stretch
    # bound too long, the window too optimistic, and the burn it picks too
    # steep.  Measured in ``logs/LOG891``, the ceiling ratcheted 30 -> 20 deg
    # in nine seconds as q crossed 7 kPa, with the vehicle achieving 13-16
    # against 28 commanded -- a factor of 0.54, worse than even the new
    # table's 0.74 at its worst bin.
    # **Finer bins, and the conservative quartile rather than the median,
    # because its consumer is a *bound*.**  ``deorbit_window`` flies its long
    # corner on this table to ask how far the glide could stretch; a median
    # is the answer the vehicle beats half the time, and half the time it
    # falls short of a bound it was promised.  p25 is the number a bound
    # wants.  The median is in the right-hand column so the cost of the
    # choice is visible:
    #
    #   q <= Pa      n      p25    median     p75
    #     1000   10674     1.01     1.02     1.02
    #     2000    4566     1.00     1.01     1.02
    #     4000    6202     0.80     0.89     0.99
    #     6000    2945     0.71     0.79     0.86
    #     8000    2605     0.68     0.72     0.81
    #    12000    2842     0.71     0.72     0.78
    #    16000    2332     0.71     0.72     0.73
    #    24000     554     0.70     0.72     0.72
    #
    # The spread narrows as the air thickens, which is itself the finding:
    # up high the vehicle holds what it is asked and the variation is noise,
    # and by 12 kPa it is against a trim limit and every flight meets it.
    # **The median, not the conservative quartile, and that is measured.**
    # p25 looks like the right choice for a *bound* and it moves the answer
    # the wrong way: a harsher factor pushes the short corner's 20 degrees
    # down toward 14, and hypersonically that is *below* the range minimum,
    # so the short corner gets **longer** while the long one gets shorter.
    # The window narrows from both ends and its centre does not move the way
    # the intuition says.  Offline it asked for 59.3 m/s against 57.2 -- a
    # steeper burn, which is the opposite of what a vehicle landing short
    # needs.  The residual bias belongs in ``DEORBIT_WINDOW_BIAS``, which is
    # dimensionless and does not have this coupling.
    # **Re-measured over 25 007 GLIDE ticks, and it holds.**  CLAUDE.md
    # warns that this table was taken before the attitude controller was
    # tuned, so it was read back off the logs directly -- the achieved over
    # commanded angle of attack, binned by dynamic pressure, across every
    # flight on disk:
    #
    #     q (Pa)        median   mean    p25      n
    #     0-1500          1.02   1.01   1.02   13250
    #     1500-3000       1.02   1.00   1.00    3130
    #     3000-6000       0.84   0.86   0.81    2648
    #     6000-9000       0.80   0.82   0.75    2083
    #     9000-12000      0.74   0.75   0.71    3753
    #     12000+          0.77   0.77   0.73     143
    #
    # against the table's 1.02, 1.01, 0.89, 0.74, 0.72 -- within 0.06
    # everywhere.  **The table is not the problem.  ALPHA_TRACKING_ON is.**
    # With the flag off ``tracked_alpha`` returns the command unchanged, so
    # every propagation in the program -- the glide's solve and the burn's
    # alike -- believes the vehicle holds 100% of the angle it is given
    # through the whole dense entry, where it actually holds 74-84%.
    #
    # That is a systematic over-estimate of lift in exactly the phase that
    # sets the arrival, and the shape of the resulting miss is visible in
    # ``logs/LOG2401``: the predicted along-track miss sits at the commanded
    # +500 m from 54 km down to 40 km, and then goes -2145 at 37 km, -4544 at
    # 24.7 km and -11804 at 19.9 km, with ``aoa=21.9/13.9`` beside it.  The
    # solve reports ``slv=ok`` on all 272 ticks of a flight that arrives
    # **14.8 km short**, because nothing it propagates knows the vehicle
    # cannot hold what it is being told to fly.
    #
    # ``tracked_alpha``'s own docstring already says this: "A propagation
    # that believes the command is a propagation of a trajectory the vehicle
    # does not fly, which is the error this project has now made in five
    # places."  It is made here too, by a flag.
    ALPHA_TRACKING: tuple = ((1000.0, 1.02), (2000.0, 1.01),
                             (4000.0, 0.89), (8000.0, 0.74),
                             (16000.0, 0.72), (32000.0, 0.72))
    # Gated by a flag rather than by emptying the table, so the experiment
    # above can be run with ``--set ALPHA_TRACKING_ON=True`` without having
    # to express a tuple on a command line.  Off by default: see the numbers.
    ALPHA_TRACKING_ON: bool = False
    # **The saturation, learned in flight rather than tabulated.**  Failure
    # 10 ends by saying the plant is ``achievable = min(command, holdable(q))``
    # and that a proportional gain is the wrong shape, because the solve
    # undoes it: told 0.85 of its command will be flown it commands more, the
    # factor cancels, and the prediction believes the inflated number.  A
    # ceiling cannot be talked round.  What was missing was ``holdable``,
    # because ``planeprobe`` aims the airflow and reads a *force* -- kRPC will
    # not report a moment at any price.
    #
    # It could be fitted offline: every GLIDE line carries
    # ``aoa=commanded/achieved`` and enough to recover the dynamic pressure
    # (``q = dec * m / cda``), and 663 logs of this craft give 11280 samples
    # of a clean saturation.  **That number belongs to this airframe and
    # nothing else**, which is the same objection that makes the aero table
    # swept in STANDBY instead of written down, the vessel axes measured
    # instead of assumed, and the frame's handedness computed instead of
    # reasoned about.  A constant here would fly the next vehicle on this
    # one's trim.
    #
    # So ``trajectory.Holdable`` learns it from the vehicle's own telemetry
    # during the entry it is flying.  These are the estimator's shape, not
    # the airframe's: how wide a bin is, how much command-minus-achieved
    # counts as saturated, and how many samples a bin needs before the
    # propagator is allowed to believe it.
    # **On by default: it makes the prediction honest and never measured
    # worse.**  Three saves with it on land 5-9 km nearer than with it off --
    # which is inside the glide's own run-to-run scatter (failure 10d) and so
    # not a result on its own.  It is on because of what it *is*, not that
    # margin: the propagator was flying attitudes the airframe demonstrably
    # cannot hold, and "predict the law you fly" is the rule that the
    # arrival-speed cap and boosterland's ``landing_command`` already exist
    # to keep.  Unlike ``ALPHA_TRACKING``'s gain, a ceiling cannot be undone
    # by the solve asking for more.
    HOLDABLE_ON: bool = True
    HOLDABLE_Q_DECADE_BINS: int = 6         # bins per decade of dynamic pressure
    HOLDABLE_SATURATED_DEG: float = 2.5     # command - achieved, to count
    HOLDABLE_MIN_SAMPLES: int = 4           # before a bin is trusted
    HOLDABLE_MIN_Q: float = 500.0           # below this the air holds nothing back
    HOLDABLE_MARGIN_DEG: float = 1.0        # believe the vehicle by this much

    # -- the ceiling, measured by a probe instead of inferred in flight ----


    # -- approach and flare ------------------------------------------------
    # Measured, at 6.715 t: stall 37.1 m/s at 30 deg, and the flare *stalls*
    # at 1.3 and 1.5 Vs -- arresting 16 m/s of sink needs a high angle of
    # attack, which drags at 50 m^2, which eats the airspeed the lift is made
    # from.  1.7 Vs arrests in 12 m and touches down at 46 m/s.  So the
    # approach is fast and the rollout is long, which 2.4 km affords easily
    # (46 m/s in 2400 m is 0.44 m/s^2, and gear-down drag alone gives ~1).
    # **These four numbers came from ``planeprobe`` and the airframe does not
    # agree with it.**  ``planeprobe`` aims the airflow and asks
    # ``simulate_aerodynamic_force_at`` what the force would be; subsonically
    # that over-reads the lift by about a factor of two, and every constant
    # derived from it -- the stall speed, the approach speed, the glide
    # angle, the gate's height, the flare's trigger -- was set for an
    # aircraft that does not exist.
    #
    # The airframe's own answer, recovered from ``act=ClA/CdA`` (the force
    # kRPC reports, resolved about the relative wind) on every subsonic
    # GLIDE and APPROACH line this project has flown, binned on the angle of
    # attack the vehicle *achieved*:
    #
    #   aoa      n     L/D    ClA   1g speed      probe said
    #    10    186    1.75   15.3    83.7 m/s     L/D 3.39, ClA 28.4
    #    12    519    2.10   22.9    68.4         L/D 3.26, ClA 36.1
    #    14   1013    1.52   24.6    66.0         L/D 3.07, ClA 43.6
    #    16   2882    1.46   26.8    63.2         L/D 2.86, ClA 50.7
    #    18    669    1.48   24.5    66.1         L/D 2.65, ClA 57.6
    #
    # and on the nine flights that were flown with the attitude controller
    # tuned -- a vehicle that holds what it is asked, so the samples are not
    # transients -- the same shape reads tighter: L/D 2.12 at 10, 2.11 at 12,
    # 1.91 at 14, 1.49 at 16, with quartiles inside 0.13.  **Best glide is
    # L/D 2.1, not 3.39.**  The highest lift the airframe has been measured
    # making subsonically is ClA 45-50 around 22-26 degrees, which at 6.69 t
    # is a stall speed near 48 m/s and not 37.1.
    #
    # What that error did.  At ``1.8 x 37.1 = 67 m/s`` the approach was being
    # flown at **1.4 times the real stall speed**, where holding 1 g needs 16
    # to 17 degrees of angle of attack -- past best glide, at L/D 1.46.  So
    # the vehicle descended at 33 degrees instead of 17, reached the flare
    # with a load factor of about 1.5 available against the 1.7 the arrest
    # needed, and touched down at 25 m/s of sink.  Every flight of this
    # airframe has ended as a wreck reporting 0 parts and 0.00 t, including
    # the ones that arrived within two kilometres of the runway, and this is
    # why.
    #
    # The model the *propagator* uses is not implicated: swept in flight, it
    # reads ClA 23.3 at 12 and 26.2 at 16 against the airframe's 22.9 and
    # 26.8.  It is ``planeprobe``'s file alone that is wrong, and the
    # constants that were copied out of it by hand.
    # **The approach is fast on purpose, and the first version of this was
    # not fast enough.**  The load factor available to arrest the sink goes
    # as the *square* of the approach speed -- ``n_max = (V/Vs)^2`` -- while
    # the sink to be arrested goes only as ``V``, so flying faster makes the
    # flare strictly easier.  What it costs is runway, and there are 2.4 km
    # of it: stopping from 91 m/s in the 1.8 km past the touchdown aim needs
    # 2.3 m/s^2 against about 1 from gear-down drag alone, before the brakes.
    # Horizontal speed is the cheap axis and vertical speed is the one that
    # breaks the vehicle.
    #
    # At 1.60 x stall the flare had 2.56 g in hand against 38 m/s of sink and
    # it was not enough: measured, it arrested 40.4 m/s to 12.65 and broke
    # the vehicle up **on the runway, 13 m off the centreline**
    # (``logs/LOG803``).  At 1.90 there are 3.6 g, which is the difference
    # between a manoeuvre with margin and one that stalls partway through.
    # (``STALL_SPEED_M_S``, the old craft's 48 m/s taken in flight as a true
    # airspeed, was removed 2026-10-05: ``airframe.stall`` is the table's.)
    # **Flown by hand at 100+ m/s, and that is the number to design to.**
    # 1.90 was still anchored on the flare study in ``planeprobe``, whose
    # multiples are of a 37.1 m/s stall that does not exist -- and every
    # flight that arrived under about 70 m/s ballooned and stalled out of
    # the flare rather than landing out of it:
    #
    #   LOG816  flare at 66.4 m/s -> arrests, then vs +8.8 (off the ground
    #                               again), -8.9, and 0 of 23 parts left
    #   LOG815  flare at 78.8 m/s -> touches at 54.6, stops 1628 m along the
    #                               runway and 4.7 m off the centreline
    #
    # The faster one is the one that stayed down.  Stopping from 110 m/s in
    # the 2.0 km past the touchdown aim needs 3.0 m/s^2, which is what the
    # brakes are for; ballooning at 66 m/s has no answer at all.
    # **1.90, not 2.40, and the project's own flare table said so.**  2.40
    # was set to stop the flare stalling, and it did -- by arriving with so
    # much speed that the flare *balloons* instead: measured, the flare ended
    # at sink **-7.5 m/s** (climbing) at 93 m/s with 15.7 degrees of alpha
    # achieved, and the vehicle skipped along the runway on its tail shedding
    # parts.  Every arrival of that configuration ended with 0 of 23.
    #
    # The integration in ``docs/spaceplane/design.md`` puts 1.90 at "arrested, flare
    # uses 11.1 m, touchdown 54.4 m/s", and 1.30-1.50 at "stalled".  Flown at
    # 1.90 the touchdowns are 37-51 m/s at 3-5 m/s of sink, which is the
    # table's number, and the first intact landing this project has made came
    # off it: 19 of 23 parts, stopped in 7.5 s at 6.84 m/s^2.
    # **And 1.90 lands it on its tail, because the attitude at touchdown is
    # set by the speed there and not by the flare.**  1.90 arrives at the
    # flare at 90 m/s and touches down at 47-54 -- 1.0 to 1.1 times the
    # stall -- which is the one angle of attack that will hold the vehicle
    # up, about 16 degrees achieved on every landing this project has made.
    # The fuselage stack is what comes off (``Mk1-3 Command Pod, Service
    # Bay, FL-R120, Rockomax X200-16`` in that order, a part every fifth of
    # a second), which is a body strike and not a gear failure.
    #
    # Raising it alone does not help and failure 34 has the measurement:
    # 2.40 touched down at 93 m/s and still achieved 15.7 degrees, because
    # the flare's arrest demand divides by the height left and saturates at
    # maximum lift whatever the speed.  It is the *pair* that works --
    # ``FLARE_TOUCHDOWN_ALPHA_DEG`` says what attitude to arrive in and this
    # buys the airspeed to arrive in it, since the load a given angle makes
    # goes as the square of the speed.
    #
    # **And the speed it takes is set by the attitude, not chosen.**  To
    # hold 1 g at angle ``a`` the vehicle needs ``v = Vs sqrt(CLmax/CL(a))``:
    # at 58 m/s -- 1.2 times the stall -- the wing needs about 0.7 of its
    # maximum lift and that *is* 15 degrees, whatever the flare commands.
    # 2.25 touched down at 58.4 m/s and 15.5 degrees achieved, on the
    # centreline at 0.03 m/s of sink, and still took both elevons off
    # (``logs/LOG1373``, 20 of 23 parts, +11.4 m off the centreline, stopped
    # in 8.3 s). The elevons are the first part lost on every landing this
    # project has made, which is a trailing edge striking the runway.
    # **2.45 was tried, to buy a wheel landing at 85 m/s, and it does not
    # buy one.**  The flare bleeds to about 1.1 times the stall whatever it
    # is given -- 2.25 touches down at 51-58 m/s and 2.45 at 51-54, because
    # the extra speed is spent in a longer flare rather than carried to the
    # wheels.  Flown, four good handovers at 2.45 with an 8 degree touchdown
    # cap were destroyed where the same handovers at 2.25 and 13 degrees
    # kept 20 and 23 of 23 parts.  The touchdown speed is set by how long
    # the flare lasts (``FLARE_LEAD_S``), not by how fast the approach is,
    # and that is where the next attempt on it belongs.
    APPROACH_FACTOR: float = 2.25           # x stall: 108 m/s
    # **What the flare needs at its own door, which is not what the approach
    # should hold all the way down.**  Measured over nine flights of one
    # configuration, by the speed at ``APPROACH -> FLARE``:
    #
    #     67.1 m/s  destroyed        94.7  destroyed
    #     83.2      -3 m, intact     99.2  destroyed
    #     86.0      -4 m, intact     99.4  destroyed
    #     91.5      -5 m, intact     99.6  -32 m
    #                                101.9 -34 m
    #
    # 1.75 sits in the middle of the band that works and clears the stall
    # floor below.  It is a number about *this airframe* and it says so --
    # but it is one the vehicle can measure on itself (the flare either
    # completes or it does not, and the log records both the entry speed and
    # the parts), which is the difference between this and a transcription.
    APPROACH_FLARE_FACTOR: float = 1.75     # x stall: 84 m/s at the flare
    # The floor at the flare's door.  ``APPROACH_FACTOR``'s own comment
    # records where this came from: the flare *stalls* from 1.3 and 1.5 times
    # the stall speed and only completes from 1.7 up.  So this is the
    # measured edge, not a margin chosen here, and the schedule must not take
    # the vehicle under it.
    APPROACH_FLARE_FLOOR_FACTOR: float = 1.70
    # Schedule the approach's commanded speed from ``APPROACH_FACTOR`` at the
    # gate to ``APPROACH_FLARE_FACTOR`` at the height the flare triggers,
    # instead of holding one speed throughout.  Off by default until it is
    # flown; ``APPROACH_FACTOR`` is deliberately left alone by it, so the
    # cone's speed (``HAC_SPEED_FACTOR``, a multiple of it) does not move and
    # the starvation that killed the constant-cut experiment cannot recur.
    # **Flown at 1.75 and at 2.05, and neither is a configuration to ship.**
    #
    # At 1.75 the schedule does exactly what it was written for: flare entry
    # held to **sd 5.3 m/s** where the constant scatters 14, and cross-track
    # of **-1.0 m sd 6.6** against a 35 m half-width -- the lateral problem
    # solved outright.  It also enters the flare at 65-76 m/s and destroyed
    # two of five.
    #
    # At 2.05 it survives and the flare entry mean lands inside the window,
    # but the control evaporates: the first six flights read 4 of 6 on the
    # runway against the constant's 2 of 6, and **pooling a second identical
    # six wiped it out -- 4 of 11 against 4 of 11, exactly level**, with the
    # along-track scatter three times worse (sd 1270 against 297).  The
    # apparent win was noise at n=6, which is the thing CLAUDE.md says three
    # flights cannot answer, measured happening.
    #
    # **Why the two ends behave differently is the useful part.**  A glider
    # sheds speed at will and cannot make it, so a target *below* the minimum
    # arrival energy is always reachable and is held tightly, and a target
    # above it is often unreachable and is not held at all.  The band that is
    # both controllable and survivable is narrow, and the ramp as written
    # reaches its target *at* the trigger height while still decelerating --
    # so it arrives still slowing, and overshoots into the stall.
    #
    # What it wants is to reach the target *above* the trigger and hold it.
    # ``APPROACH_PROFILE_HOLD_M`` does that, and with it the schedule works:
    # four arms of four flights on ``qs_plane``, flare entry runs 72-81 m/s
    # at a factor of 1.85 against 103-108 with the schedule off, monotone in
    # between, and at **2.05** the landing put 4 of 4 inside the along-track
    # window at **sd 302 m** against 1020-1988 for every other arm.
    #
    # **The factor is a bias, not a setpoint, and its useful end is above
    # the airframe's window rather than inside it.**  1.85 asks for 89 m/s
    # and delivers 77; 1.95 asks for 94 and delivers 88.  A glider on a
    # fixed geometry cannot choose speed and path independently, so what the
    # schedule sets is how hard the approach is asked to decelerate and not
    # the speed it arrives at -- calibrate it against the speed *delivered*.
    # Failure 62; failure 49 is the same knob read before the hold existed.
    APPROACH_SPEED_PROFILE: bool = True
    # How far *above* the flare's trigger height the schedule finishes.  See
    # ``guidance.approach``: a ramp that lands on its target at the trigger
    # arrives still decelerating and overshoots into the stall, which is what
    # failure 49 measured at 1.75.  With a hold there is a stretch of
    # constant command for the speed loop to settle on, and the flare gets
    # the speed the schedule was asked for rather than the speed the
    # deceleration happened to be passing through.  200 m is about five
    # seconds at the measured sink rates.
    APPROACH_PROFILE_HOLD_M: float = 200.0
    # Measured: the flare needs 12 m gear-up and 14.6 m gear-down from a
    # 1.7 Vs approach.  Started at a fixed height it is late whenever the
    # arrival is faster than nominal, so the trigger leads on the sink rate
    # the vehicle actually has.
    # The standalone integration says 12 m is enough from a 1.7 Vs approach,
    # and that is the height needed once the angle of attack is *already*
    # there.  Getting it there costs time -- the ramp -- during which the
    # vehicle is still descending at the approach rate, so the trigger has to
    # lead by roughly the sink rate times the ramp plus the arrest itself.
    # At 16 m plus half a second of lead the offline arrival was 18.8 m/s,
    # i.e. barely flared at all.
    # Re-derived against the measured stall and the measured polar.  The
    # arrest itself needs ``sink^2 / (2 g (n - 1))`` of height, and at 77 m/s
    # against a 48 m/s stall there are about 2.5 g available against a 30 m/s
    # sink -- some 40 m.  What the old numbers left out is the *pitch-up*:
    # measured, a flare commanded 30 degrees and was achieving 16.1 a second
    # and a third later, and ``ATTITUDE_TIME_TO_PEAK_S`` at 3.0 s makes that
    # deliberately slower.  Three seconds of pitch-up at 30 m/s of sink is 90
    # m of height that has to be there before the arrest starts.
    # Flown at 40 + 4.5 x sink: triggered at 198 m against 35.4 m/s of sink
    # and arrested it to **7.5 m/s** by touchdown -- the first flare this
    # project has completed, and still a hard enough arrival to break the
    # vehicle up on grass.  What is left is the pitch rate: commanded 26.3
    # degrees it was achieving 15.9 four seconds later even with the
    # controller handed back to 1.0 s.  An early flare costs nothing, because
    # the law is closed on the load the arrest actually needs -- at 280 m and
    # 35 m/s that is 1.32 g, which is a few degrees above trim -- so it
    # starts gently and tightens as the ground comes up, which is what a
    # slow-pitching aircraft wants.
    FLARE_ALT_M: float = 50.0
    # **And then it led by so much that the vehicle floated onto its tail.**
    # 6.5 puts the flare at 310 m with a 40 m/s sink where the arrest itself
    # needs about 140: the sink is gone by 100 m and the rest is float, and
    # a float is a deceleration.  Measured, the vehicle enters the flare at
    # 90-93 m/s and touches down 900 m later at **50**, which is 1.04 times
    # the stall -- so it arrives at the only angle of attack that will hold
    # it there, about **16 degrees achieved, every single flight**, and
    # scrapes its tail along the runway shedding a part every fifth of a
    # second.  ``logs/LOG1336`` did that on the centreline (xt +6.6 m),
    # 1909 m along a 2400 m runway, at 1.1 m/s of sink: nothing about that
    # arrival is wrong except the attitude it is in.
    #
    # The one landing this project has kept (``logs/LOG1309``, 19 of 23
    # parts) touched down in the same attitude and lost four parts doing it.
    # A full-stall landing is not available to this airframe, so the flare
    # is triggered near the height the arrest actually needs -- ``sink^2 /
    # (2 g (n-1))``, about 140 m at 1.6 g -- and the vehicle touches down
    # faster, at the smaller angle a faster wing needs.
    # 3.5 still floats: the flare is entered at 99-108 m/s and the wheels
    # arrive at 51-58, which is 1.1 times the stall and therefore about 15
    # degrees of angle of attack -- and the elevons are the first part lost
    # on every landing this project has made, in every configuration, which
    # is a trailing edge on the tarmac.  The touchdown speed is set by how
    # long the flare lasts and not by how fast the approach is (2.45 was
    # tried on the approach and touched down *slower*), so this is the knob.
    # 2.5 puts the flare at about 155 m against the ~140 the arrest itself
    # needs at 1.6 g, which the approach speed now affords.
    FLARE_LEAD_S: float = 2.5               # extra height per m/s of sink
    # **The flare is ten seconds long, and it used to fly all of them wings
    # level.**  At the sink rates this approach arrives with, the trigger is
    # ``50 + 6.5 x 65`` -- about 470 m -- so "wings level from here" hands
    # the runway ten seconds of uncorrected drift.  Measured, the cross-track
    # at the flare and the cross-track at touchdown are the same number, and
    # it is the number that has been putting the vehicle on the grass beside
    # a runway it is otherwise hitting: +101 m and +103 m on the best
    # arrival of one batch, on a strip 70 m wide.
    #
    # The rule it was protecting is real -- a banked *arrival* puts a wingtip
    # down and the wings are the only thing holding this airframe off the
    # tarmac -- but that is a rule about the last few metres, not about 470.
    # So the lean is tapered to nothing by ``FLARE_WINGS_LEVEL_M`` instead of
    # cut at the phase boundary, and the lateral law is the approach's own:
    # one capture law, not two that can disagree.
    # Below this the nose is aimed along the *runway* rather than along the
    # airflow, which is how the crab comes out before the wheels touch.  See
    # ``Autopilot.aim_runway``.  High enough that the yaw has time to settle
    # -- the attitude controller is tuned to a 3 s time to peak -- and low
    # enough that the flare is still flying the airflow where that is what
    # matters.
    # **30, default 2026-10-05**: aligning from 140 m left the velocity's
    # track error to drift the shuttle 25-55 m sideways through the flare
    # (sideslip to +8 deg); aligned only in the last 30 m the touchdown
    # cross-track halved, median 27 -> 13-20 m over 48 flights
    # (sav-flarelat-1005, sav-aim-1005, sav-sstop-1005; rigoff cone saves).
    FLARE_ALIGN_ALT_M: float = 30.0
    FLARE_BANK_MAX_DEG: float = 12.0
    FLARE_WINGS_LEVEL_M: float = 60.0
    FLARE_BANK_TAPER_M: float = 150.0
    FLARE_MARGIN: float = 1.3               # ask for more load than the sum says
    FLARE_ALPHA_DEG: float = 30.0           # maximum lift
    # **The tail is a hard limit and the flare did not know about it.**
    # Measured on LOG1656, the best landing this project has flown: touchdown
    # at 45.9 m/s with the sink already arrested (-3.8 m/s), and then Elevon 4
    # went one second later, followed by everything else.  The flare had
    # rotated to 17.7 degrees of achieved alpha by 10 m, and the docs already
    # knew 15.2 on the main gear scrapes twenty parts off -- but only the
    # *rollout* was capped for it (``ROLLOUT_HOLD_ALPHA_DEG``), and the flare
    # that sets the touchdown attitude was not.
    #
    # ``Telemetry`` measures the angle from the bounding box: the wheels sit
    # under the centre of mass, the tail sits behind them, and the nose-up
    # angle that brings the two together is ``atan(clearance / aft)``.  This
    # is the fraction of it the flare may use.  Geometry per airframe, not a
    # number per airframe -- the vehicle after this one has its own tail.
    TAIL_STRIKE_MARGIN: float = 0.8
    # Cap the attitude the vehicle *reaches*, not the one it is commanded:
    # the tail cap less the measured pitch overshoot (``pitch_overshoot``).
    # The shuttle flew 11.2 deg against 6.9 commanded at touchdown on a 9.1
    # deg tail angle and lost a wing and the RCS blocks in 0.1 s (LOG3596);
    # every game touchdown of 2026-09-24 lost parts within 0.5 s of contact.
    TAIL_LIMIT_ACHIEVED: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # Used only while the box has not answered.  Deliberately tight: too
    # little rotation lands hard, too much removes the tail, and only one of
    # those is recoverable.
    TAIL_ANGLE_FALLBACK_DEG: float = 12.0
    # **The attitude the vehicle is allowed to touch down in**, which is a
    # geometry limit and not an aerodynamic one -- and which the flare had
    # no term for.  See ``guidance.touchdown_alpha_cap``: the arrest demand
    # divides by the height left, so the last two ticks of every flare ask
    # for maximum lift and the vehicle arrives nose-high whatever the
    # approach did.  Measured achieved angles at touchdown, across every
    # landing this project has made: 13.9, 15.3, 15.6, 16.0, 16.2, 17.2 --
    # and 3 to 23 parts lost each time, on the tarmac and on the centreline.
    # This airframe holds about 0.8 of a commanded angle down here, so 13
    # commanded is about 11 achieved.
    # This airframe *overshoots* its command down here rather than lagging
    # it -- 15.5 achieved against 13.9 commanded, 17.4 against 13.0 -- so
    # the commanded figure sits a couple of degrees under the attitude
    # wanted.  8 was tried and is below what the wing needs to hold itself
    # up at a touchdown speed of 1.1 times the stall: the flare stops
    # arresting and the arrival is harder, not flatter.
    FLARE_TOUCHDOWN_ALPHA_DEG: float = 13.0
    FLARE_TOUCHDOWN_ALT_M: float = 25.0     # capped below this, tapered to 2x
    # Below this height the arrest demand stops growing.  It is roughly the
    # height at which there is no longer time to change the outcome: asking
    # for 2.5 g three metres up does not arrest anything, it only sets the
    # attitude the wheels arrive in.
    FLARE_ARREST_FLOOR_M: float = 20.0
    # The sink rate below which the vehicle is settling rather than still
    # arresting, and the cap above may apply.  See ``touchdown_alpha_cap``:
    # capping a vehicle that is still coming down at 30 m/s does not land it
    # gently, it lands it at 21.
    FLARE_TOUCHDOWN_SINK_M_S: float = 8.0
    # **The flare arrests to level and then falls out of the float.**  See
    # ``guidance.flare``: the arrest demand is an open-loop equilibrium and
    # nothing corrects it once the descent has already stopped.  With this on
    # the flare tracks the sink rate it can still arrest in the height left
    # -- ``sqrt(touchdown^2 + 2 a h)`` -- rather than arresting all of it at
    # whatever height it happens to succeed.
    FLARE_SINK_TRACK: bool = True
    # The vertical deceleration the schedule is drawn with, as a load factor.
    # Higher is a steeper schedule, a shorter flare and a faster touchdown.
    FLARE_TRACK_LOAD: float = 1.5
    # How long the loop is given to put the sink back on the schedule.
    FLARE_TRACK_TAU_S: float = 2.0
    # And what it may ask for while doing it.  The floor is a push-over, not
    # a dive: ``FLARE_TRACK_LOAD_MIN`` is what lets an early arrest come back
    # down instead of floating, and it is close to one on purpose.
    FLARE_TRACK_LOAD_MIN: float = 0.85
    FLARE_TRACK_LOAD_MAX: float = 2.5
    # **An exponential bottom under the sink schedule** (0 = off).  The
    # schedule ``sqrt(td^2 + 2 a h)`` is designed to touch down at
    # ``FLARE_TOUCHDOWN_SINK_M_S`` (8) and the loop's lag adds to it: the
    # shuttle contacts at 9-17 m/s (LOG3770-3796, 3846-3869) against wings
    # and elevons of crashTolerance 15 and an engine of 7, and loses them.
    # With this set the wanted sink is also capped at
    # ``FLARE_EXP_TOUCHDOWN_M_S + h / FLARE_EXP_TAU_S`` -- the classic
    # autoland flare, arresting earlier and settling rather than arriving.
    # **Default 2026-09-30**, with ``FLARE_EXP_TAU_S=4`` and
    # ``FLARE_DOOR_FROM_SCHEDULE`` (the three flare fixes together): from
    # orbit on the old craft (qs_plane, rotation `14dc04c2`, LOG4010-4033)
    # 5/6 intact on the runway within 22 m of the centreline, contact sink
    # 1.0-4.1 m/s at 50-60 m/s, against 1/6 on the old defaults (contact
    # 15-21 m/s at 37-41 -- every flare stalled out); on the shuttle bench
    # (qs_shuttle_low, LOG3992-4009) 5/6 intact 30/30 against 1/6.  The
    # shuttle from orbit is 0/6 on both arms -- its failures are upstream.
    FLARE_EXP_TAU_S: float = 4.0
    FLARE_EXP_TOUCHDOWN_M_S: float = 2.0
    FLARE_SPEED_TD_MAX_M_S: float = 5.0
    # **The door where that schedule starts to bind** (needs
    # ``FLARE_EXP_TAU_S``): ``tau (sink - td) + T sink``, T the pitch axis's
    # response (``attitude_settle_s``), and the schedule read T ahead
    # (``flare_lead_s``) so the two agree.  The old door (50 m + 2.5 s x
    # sink) opened the old craft at 126 m with 31 m/s of sink (LOG3961); the
    # schedule still allowed 28 m/s at 107 m, asked 1.1 g, and the pull came
    # at 45 m -- contact at 74 m/s, nose 4 deg down.  Off until paired.
    FLARE_DOOR_FROM_SCHEDULE: bool = True   # default 2026-09-30, see FLARE_EXP_TAU_S
    # **The tail strikes by attitude, not by angle of attack.**  The flare
    # capped its *angle of attack* at the tail angle (9.1 deg on the
    # shuttle) at every height, and the body's attitude is angle of attack
    # less the descent angle -- at the door the shuttle is 25-35 deg nose
    # down on its path (sink 35-107 m/s at 55-160 m/s, LOG3720-3759), so the
    # tail is nowhere near the ground and the wing was being held to ~1.6 g
    # it could have made 3 at.  The sink tracker asks for up to
    # ``FLARE_TRACK_LOAD_MAX`` when it is behind and never got it: every
    # shuttle flare touched down at 9-102 m/s of sink.  With this on:
    #   * the flare's alpha cap is the tail angle *plus the descent angle*,
    #     which is the same cap at touchdown (descent ~0) and the attitude
    #     limit everywhere above it; and
    #   * ``aim_runway`` pitches to ``alpha - descent``, the attitude that
    #     delivers ``alpha``.  ``AIM_RUNWAY_TRUE_ALPHA`` pitches to
    #     ``alpha + descent`` -- a sign error that delivers alpha plus
    #     *twice* the descent, masked because the tail cap on the pitch
    #     binds (gentle flares fly 7-9 deg above command: LOG3730, 3737).
    # What would contradict it: flares that balloon (sink going negative
    # well above the runway), or touchdowns on the tail (the pitch cap is
    # unchanged, so this should not happen).
    FLARE_TAIL_BY_ATTITUDE: bool = True   # default 2026-09-30, see FLARE_EXP_TAU_S
    # **A fast, shallow final** (the user's proposal, 2026-09-26): the
    # Shuttle's own profile -- steep and fast outside, a preflare high up
    # into a shallow inner glide that bleeds the speed, and a small final
    # flare -- in place of a door at ``FLARE_ALT_M + FLARE_LEAD_S * sink``
    # followed by one hard arrest.
    #
    # Why this airframe needs it: the shuttle's tail strikes at 9.1 deg, so
    # the flare is capped near 7 deg of alpha and its only authority is
    # speed (spare lift at that alpha is ~0.8 g at 100 m/s, ~2 g at 130).
    # On ``fbe32132`` (LOG3729-3739) every flare door was a 30-32 deg path at
    # 43-69 m/s of sink, arrested with the wing at the tail limit, and the
    # touchdowns ran 40-87 m/s with 2-43 m/s of sink; the two gentle ones
    # (LOG3737, 3739) still lost parts to an alpha that overshot its command
    # by 6-8 deg in the last two seconds of an arrest that ended at the
    # ground.  Below ~60 m/s this wing cannot hold 1 g at a tail-safe
    # attitude at all, so every touchdown under that falls in.
    #
    # With this on:
    #   * the door is *physical*: the height to bring the sink down to the
    #     inner glide's at ``FLARE_SHALLOW_PULL_LOAD``, plus
    #     ``FLARE_TRACK_TAU_S`` of lag at the current sink, plus
    #     ``FLARE_ALT_M`` of inner glide under it (``flare_door``);
    #   * the flare's sink schedule is capped at the inner glide's
    #     ``speed * sin(FLARE_INNER_GLIDE_DEG)``, and its bottom is drawn at
    #     ``FLARE_SHALLOW_FINAL_LOAD`` to ``FLARE_SHALLOW_TOUCHDOWN_SINK_M_S``
    #     -- so the arrest happens hundreds of metres up and the last part is
    #     a glide, not a pull;
    #   * the approach flies faster (``FLARE_SHALLOW_APPROACH_FACTOR``,
    #     ``FLARE_SHALLOW_DOOR_FACTOR`` x stall in place of
    #     ``APPROACH_FACTOR``/``APPROACH_FLARE_FACTOR``): the inner glide is
    #     shallower than the polar's best and pays for it in speed, so the
    #     door has to carry it.
    #
    # What would contradict it: touchdowns no faster than today's, a flare
    # that still reaches the ground in its pull-up (the ``flare:`` line's
    # sink at the door vs at touchdown), or rollouts running off the far end
    # -- stopping from 80 m/s needs ~1.3 km at the spoiler's 2.5 m/s^2.
    FLARE_INNER_GLIDE_DEG: float = 5.0
    FLARE_SHALLOW_APPROACH_FACTOR: float = 2.7   # x stall at the gate
    FLARE_SHALLOW_DOOR_FACTOR: float = 2.4       # x stall at the door
    # **``aim_runway`` flies a pitch attitude and the guidance computes an
    # angle of attack.**  They differ by the descent angle -- twenty degrees
    # at the flare's door, zero on the ground -- so the wing was handed more
    # lift than was asked for, in proportion to how fast the vehicle was
    # coming down.  ``logs/LOG2290``: commanded 8.2 degrees, achieved 12.6,
    # arrested to level at 35 m, floated six seconds from 74 m/s to 44 and
    # fell the last thirty.  With this on the nose is pitched to
    # ``alpha + descent``, which is the attitude that delivers ``alpha``, and
    # the heading stays on the runway -- the crab fix is untouched.
    AIM_RUNWAY_TRUE_ALPHA: bool = True
    # How much of the flare the lateral capture may still use, 0 to 1.  The
    # two ends are both measured and both wrong: 1.0 (finish at the wheels)
    # lands within 33 m of the centreline and sheds a wing getting there,
    # 0.0 (finish at the door) enters the flare on the centreline and drifts
    # 60-80 m during it.  ``APPROACH_CAPTURE_BY_FLARE`` is the hard 0.
    APPROACH_CAPTURE_FLARE_SHARE: float = 1.0
    # Late, because the gear costs 19% of the glide ratio (see GATE_ALT_M)
    # and 800 m is still 25 seconds of descent to deploy in.
    GEAR_ALT_M: float = 800.0
    # What the gear costs, as a fraction of the glide ratio.  **Measured in
    # flight at 0.03, not the 0.19 this was transcribed from.**  Controlled
    # for altitude and phase (the 900-1700 m band, APPROACH only, a gear-down
    # arm against a gear-up one) ``CdA`` rises 16% but the ground-over-height
    # ratio moves only 1.79 -> 1.74, because ``APPROACH_SPEED_PATH`` commands
    # the descent angle that holds the speed and so absorbs the extra drag by
    # re-trimming alpha (9.5 -> 9.2 deg).  0.19 came from ``GEAR_ALT_M``'s own
    # comment, which took it off ``planeprobe`` -- the same probe that is
    # wrong by 1.8x subsonic (failure 13).
    # **What would disagree with it:** ``polar.py`` reading a clean/dirty
    # ratio pair further apart than 3% in the same altitude band and phase.
    # ``GEAR_FOR_ENERGY`` divides by this, so the honest value makes its
    # trigger saturate at ``GEAR_ALT_MAX_M`` for any surplus over about 30 m
    # -- which is the correct reading of the flown batch (+800 vs +947, 8 an

    # -- rollout -----------------------------------------------------------
    # **From touchdown, not from 60 m/s.**  The rollout is where the
    # horizontal speed is meant to go, and waiting until 60 m/s throws away
    # the first third of the runway at the speed where drag is doing most
    # work anyway.  Left as a threshold rather than removed because a vehicle
    # that touches down far too fast should still be braking, not a special
    # case.
    # **It was set to "always", and that is what has been destroying the
    # vehicle on the ground.**  See ``guidance.brake_fraction``: flat-out
    # braking from touchdown is 12.3 m/s^2 measured over 41 arrivals, which
    # is 83 kN on 6.9 t -- more than the vehicle weighs -- through two small
    # gear legs, for a rollout that needs 0.5 and gets 1.0 from drag alone.
    # Back to the meaning the comment always claimed: below this speed the
    # brakes go full, because the energy left is small and nothing else will
    # stop it.  Above it, the distance that is left decides.
    BRAKE_SPEED_M_S: float = 25.0           # full brakes below this
    # What the wheels deliver at 100%, over and above drag.  Measured as the
    # 12.3 m/s^2 mean of the first two seconds of 41 rollouts less the ~1.3
    # of aerodynamic deceleration at that speed.  **What would disagree with
    # it:** a rollout whose braking phase decelerates at some other rate with
    # ``brake_fraction`` reading 1.0 -- the telemetry prints both, so the
    # ``brk=`` column and ``dec=`` can be read against each other directly.
    BRAKE_DECEL_FULL_M_S2: float = 11.0
    # How much tarmac to have left over when stopped.  This is a margin, and
    # CLAUDE.md's rule about margins applies: it is three-ish sigma of where
    # the rollout currently stops, and it has to be re-cut if that moves.
    ROLLOUT_STOP_RESERVE_M: float = 300.0
    # **Off until it has been flown, which it has not been.**  The argument
    # for it is strong -- 12.3 m/s^2 measured over 41 rollouts is 83 kN on
    # 6.9 t, more than the vehicle weighs, for a requirement of 0.5 -- but
    # CLAUDE.md's first cross-cutting rule is that a change argued rather
    # than measured is a hypothesis, and six changes that improved a sim
    # sweep made the real flight worse.  Its batch was started twice and
    # stopped twice, both times because the entry's arrival scatter swamps
    # any landing effect (failure 77); it needs a run from `entrysave.py`,
    # not from orbit.  `--set BRAKE_FOR_DISTANCE=True` is the arm.
    # **Flown, and restored.**  Turned off as unvalidated, then measured
    # against the committed default on `qs_plane`, 8 flights an arm, arms
    # swapped across instances every round: intact 4/8 against 2/8, on the
    # 35 m strip 6/8 against 3/8, and cross-track at rest **27 m against
    # 66** -- every metric doubling the same way, with the mechanism
    # predicted before the batch.  Paired with APPROACH_BANK_COMPENSATION,
    # which is what the cross-track column is really reporting.
    BRAKE_FOR_DISTANCE: bool = True
    # **Ground spoiler: the measured lift-spoiling set, fully out, the tick
    # a main wheel reports ``grounded``.**  The user's rule (2026-09-25).
    # Dumping lift puts the weight on the braked mains -- the brake can only
    # use the normal force it is given -- and stops a skip off the mains
    # onto the nose.  It deploys the set ``AIRBRAKE_MEASURED`` found (the
    # moment-cancelling one, so no pitch kick onto the nose gear); with
    # that flag off there is no set and it logs so once.
    # ``ROLLOUT_SPOILER_DEG`` is the angle for the surface that deflects
    # most; the rest keep their measured ratio.  25 is the Big-S elevons'
    # ``ctrlSurfaceRange`` (``deployAngle`` in the save); the log reads the
    # field back, which is what would disagree with it.
    ROLLOUT_GROUND_SPOILER: bool = True
    ROLLOUT_SPOILER_DEG: float = 25.0
    AIR_DRAG_ON_M_S: float = 10.0       # over target_speed to deploy
    AIR_DRAG_OFF_M_S: float = 3.0       # ...and to stow (hysteresis)
    AIR_DRAG_FULL_M_S: float = 25.0     # fully out at this much over
    AIR_DRAG_LOW_M: float = 100.0       # not when this far under profile
    AIR_DRAG_MIN_H_M: float = 150.0     # nor below this height
    ENVELOPE_QUANTUM: float = 0.05         # of a corner span, per step
    ENVELOPE_SPEED_TAU_S: float = 10.0     # shed the overspeed in this time
    # **The attitude to hold on the ground, which ROLLOUT was not holding at
    # all.**  ``run_rollout`` commanded brakes and nosewheel and never called
    # ``aim``, so kRPC's autopilot went on holding whatever the flare had
    # last asked for -- measured, 15.2 degrees of angle of attack -- while
    # the vehicle was on the runway.  A 23-part aircraft held nose-up on its
    # main gear at 52 m/s is sitting on its tail, and in ``logs/LOG815`` it
    # stopped 1628 m along the runway and 4.7 m off the centreline having
    # shed twenty of its twenty-three parts doing it: the mass falls 6.69 t
    # to 2.80 t over four seconds of rollout while the deceleration reads
    # 8 m/s^2, which is not braking, it is scraping.
    #
    # The flare *has* to end nose-high -- that is what arrests the sink -- so
    # the fix is not a gentler flare, it is putting the nose down once the
    # wheels are down.  This is the angle of attack ROLLOUT commands.
    ROLLOUT_ALPHA_DEG: float = 0.0
    # **But not immediately, and that omission is what has been destroying
    # the vehicle.**  Touchdown is at 60-100 m/s and this commanded zero on
    # the first ROLLOUT tick, so the nose was driven onto the nose gear from
    # a nose-high attitude at flying speed.  Every arrival ended with
    # 0 of 23 parts, including one that touched down at **1.8 m/s of sink**
    # -- which is not a landing a sink rate can explain.
    #
    # An aircraft derotates: the main gear takes the vehicle, the nose is
    # held off while the elevator still has the authority to hold it, and it
    # comes down as the speed decays.  That is also free aerodynamic braking
    # at the end of the flight where the runway is shortest.  Held too high
    # it sits on its tail instead -- 15.2 degrees did that and scraped
    # twenty parts off -- so this is deliberately shallow.
    ROLLOUT_HOLD_ALPHA_DEG: float = 8.0
    # The hold capped at this fraction of the measured tail-strike angle
    # (0 = off).  See ``Autopilot.run_rollout``.
    ROLLOUT_HOLD_TAIL_FRACTION: float = 0.4  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # Enter ROLLOUT when the main wheels report ``grounded`` rather than
    # waiting for KSP's ``situation`` (see ``Autopilot.run_flare``).
    ROLLOUT_ON_MAIN_CONTACT: bool = True  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # **And the ramp that was supposed to deliver it never ran once**, which
    # is the same fix failing twice.  It was written against a touchdown at
    # "60-100 m/s" -- the speeds an ``APPROACH_FACTOR`` of 2.40 produced --
    # and the vehicle now touches down at **41 to 51 m/s**, below the 55
    # where the blend has already reached zero.  So every landing since has
    # commanded ``ROLLOUT_ALPHA_DEG`` on its first tick from the flare's 16
    # degrees: the step the ramp exists to prevent, delivered by the ramp.
    # Measured on the first tick after touchdown, four flights read an
    # *achieved* angle of attack of 74, 87, 92 and 99 degrees -- the vehicle
    # is not derotating, it is going over.
    #
    # So the schedule is written against the speed it actually depends on.
    # Touchdown is at about the stall speed by construction (the flare ends
    # there), the nose is held while the elevator can hold it, and it comes
    # down well below that.  Expressed as a fraction, a re-tuned approach
    # speed or a re-measured stall cannot leave it behind again -- which is
    # exactly how it was left behind.  Spaceplane failure 31, and failure
    # 23's rule about a constant measured from a flight nobody flies now.
    ROLLOUT_DEROTATE_FACTOR: float = 0.65   # x stall: nose fully down below
    ROLLOUT_DEROTATE_BAND_FACTOR: float = 0.30      # x stall, blended over
    # **And the first tick is still a step unless it is ramped in time too.**
    # Even with the schedule right, a vehicle that touches at 16 degrees and
    # is asked for 8 on the next tick is asked for an 8 degree pitch input
    # at 50 m/s with the wheels down.  The command leaves the flare's last
    # angle and reaches the schedule over this long.
    ROLLOUT_RAMP_S: float = 1.5
    # **The frontmost gear never brakes.**  A braked nose wheel at landing
    # speed is a moment that pitches the vehicle onto it and keeps it there.
    # This craft happens to be built with the nose brake already at zero,
    # which means nothing: the autopilot is not written for one craft, and a
    # gear that is found rather than configured costs one query at startup.
    NOSE_BRAKE_OFF: bool = True
    # **The gear, always: nose wheel no brake (friction: below); every
    # other wheel full brake torque and manual friction at the maximum.**
    # The user's rule (2026-09-25).  200% is the game's brake-torque
    # maximum and 10 the friction slider's (``ModuleWheelBase``'s
    # ``frictionMultiplier``, 0.01-10); ``apply_brakes``' fraction now
    # scales 200 rather than 100.  The mains braked at 50% before, and the
    # shuttle rolled 4.2 km from a 65 m/s touchdown (LOG3609).
    WHEEL_BRAKE_MAX_PCT: float = 200.0
    # ``Autopilot.wheel_watch``: log the wheel list and every wheel's state
    # from gear-down until this many game seconds after main-gear contact
    # (0 = off), re-reading the list every ``WHEEL_WATCH_RELIST_S``.  An
    # instrument.
    WHEEL_WATCH_S: float = 0.0
    WHEEL_WATCH_RELIST_S: float = 0.5
    # **The landing geometry measured with the gear down**
    # (``Telemetry.measure_gear_geometry``).  The first-sample box is read
    # with the gear up on every save, so on the shuttle the "wheels" were
    # the belly (1.82 m; the tyres are 3.72) and the tail-strike angle 11.0
    # deg (the engine bell reaches the runway at ~25 about the mains).  On:
    # re-measured from every part's box once the mains report deployed, and
    # the clearance and the tail angle replaced.
    GEAR_GEOMETRY_DEPLOYED: bool = False
    # ``Autopilot.ground_watch``: every part's lowest point above the
    # terrain, and the flex of the four closest, from wheels-6-m until this
    # many game seconds after contact (0 = off).  An instrument.
    GROUND_WATCH_S: float = 0.0
    # **No wheel brake for this long after main-gear contact** (0 = off).
    # GROUND_WATCH (LOG5747) saw both wings leave at the root within 0.1 s
    # of a 4 m/s contact with every other part >1 m clear: is it the brake
    # torque arriving with the load?
    ROLLOUT_BRAKE_DELAY_S: float = 0.0
    MAIN_WHEEL_FRICTION: float = 10.0
    # **The main gear's suspension, off auto** (``_set_suspension``; 0 =
    # leave the game's auto spring/damper, 1.11 / 1.0 on the shuttle).
    # CollisionSpy saw both LY-60 bodies hit the runway at 47 m/s in the
    # physics step both wing roots broke (LOG5855): the 0.5 m of travel
    # bottoms at 3-5 m/s of sink on 30 t.
    MAIN_GEAR_SPRING: float = 0.0
    MAIN_GEAR_DAMPER: float = 0.0
    # **And the nose wheel's friction control off** (the user, 2026-09-30,
    # after hand-landing the twin-fin shuttle): its automatic friction is
    # switched to manual at this multiplier.  1.0 is the slider's value on
    # both shuttle craft files, so this changes the mode, not the number.
    # 0 leaves the craft's automatic friction alone (the old behaviour).
    NOSE_WHEEL_FRICTION: float = 1.0
    ROLLOUT_STEER_GAIN: float = 0.02        # per metre off the centreline
    ROLLOUT_STEER_MAX: float = 0.4
    # **And less of it the faster the wheels are turning.**  A nosewheel
    # deflection is a lateral acceleration that goes as the *square* of the
    # speed, so the 0.4 above -- sized for the end of a rollout -- is a
    # violent swerve at a 110 m/s touchdown, which is what the approach now
    # aims for.  The taper is ``(reference / speed)^2`` so the lateral
    # acceleration the command asks for is roughly constant, full authority
    # below the reference and a tenth of it at three times the reference.
    #
    # This is a guard against a risk the faster approach introduces rather
    # than a fix for anything measured, and it costs nothing: the vehicle
    # now touches down within a few metres of the centreline, so there is
    # nothing for hard steering at speed to do.
    ROLLOUT_STEER_FULL_M_S: float = 30.0
    # **The steering's sign** (2026-10-03).  ``across`` points to the
    # vehicle's right in kRPC's left-handed frame and ``wheel_steering`` is
    # +1 left, so ``-gain * cross`` steered *away* from the centreline: every
    # shuttle rollout from the cone saves stopped 200-600 m off it, sideways
    # speed growing as it slowed.  See ``Autopilot.steer_sign``.
    ROLLOUT_STEER_ACROSS_IS_RIGHT: bool = True  # default 2026-10-03: save-steer-1003, track turns back (LOG5090, 5093) where defaults turn away (LOG5087)
    STOPPED_SPEED_M_S: float = 1.0
    # **A rollout that cannot end must still end.**  The stop test is a speed
    # threshold, and a vehicle that arrived 50 km short is not on a runway:
    # it is wedged in terrain, or destroyed with kRPC reporting a 0.00 t mass
    # and a frozen velocity forever.  ``logs/LOG646`` logged the same ROLLOUT
    # line at 64.3 m/s for the rest of its life and held a test instance for
    # the harness's whole hour.  Measurement throughput is the bottleneck in
    # this project, so a phase that can wedge is a phase that needs a clock.
    # **A vehicle with no parts left is not flying.**  ``watch_breakup``
    # reported the breakup and nothing acted on it, so an entry that came
    # apart left GLIDE steering at a gate it could never reach against a
    # frozen state: ``logs/LOG954`` and ``logs/LOG955`` each logged 700 more
    # seconds of identical lines after losing all 23 parts at Mach 6.  Twelve
    # minutes of an instance, twice, in a project whose bottleneck is in-game
    # measurement -- the same fault ``ROLLOUT_TIMEOUT_S`` exists for, one
    # phase earlier and without a clock, because zero parts needs no
    # threshold to interpret.
    STOP_WHEN_DESTROYED: bool = True
    # End a flight whose reported position has not changed for this long
    # while it reports metres per second of speed (``frozen_early``).  A
    # one-fragment breakup in FLARE, a phase with no clock, held an instance
    # for 50 minutes (LOG3009).  0 disables.
    STATE_FROZEN_S: float = 20.0
    ROLLOUT_TIMEOUT_S: float = 180.0
    # Above this the game's "landed" is a bounce or a skid worth flying out;
    # below it the vehicle is down wherever it is.  See ``grounded_early``.
    # Raised with the approach speed: a touchdown at 91 m/s is a landing and
    # not a bounce, and at 90 this test would have called it one.
    GROUNDED_SPEED_M_S: float = 130.0
    # How long the wheels may read below ground before the flight is over
    # whatever the game's ``situation`` says.  See ``grounded_early``.
    GROUNDED_STUCK_S: float = 10.0

    # -- deorbit -----------------------------------------------------------
    DEORBIT_THROTTLE: float = 1.0
    DEORBIT_DV_MIN: float = 10.0
    DEORBIT_DV_MAX: float = 400.0
    DEORBIT_SEARCH_STEPS: int = 26
    DEORBIT_AUTHORITY_WINDOW: bool = True
    # **Where in that window to sit, as a fraction of its own width.**  0 is
    # the centre; **+1 puts the gate at the *short* edge of what the glide
    # can reach**, which is the aim-long side -- from there the vehicle
    # overflies unless it shortens, and shortening is the thing it can always
    # do.  -1 is the other edge, where it can only just reach and a shortfall
    # has no answer.  (The sign is worth reading twice: a gate near the
    # *long* edge of the window is aiming *short*.)  This is the one-sided
    # principle the old aim expressed -- surplus energy is spendable and a
    # shortfall is not -- but stated as a share of the authority the vehicle
    # has rather than as a distance in metres, so it carries to a different
    # airframe, a different orbit and a different runway unchanged.  A
    # dimensionless number is a policy; a distance is a fit.
    #
    # It is also the correction for the one place the window is optimistic:
    # the long corner assumes ``ALPHA_MAX_DEG``, and in dense air this
    # airframe does not hold it (``Holdable``), so the reachable span is
    # shorter at the far end than the propagation believes.  Measured, the
    # centred window landed **20.2 and 20.2 km short** -- a bias, not
    # scatter, which is the tractable kind of wrong.
    # **Measured, and it is not monotone.**  Three flights at each setting,
    # one save, everything else equal:
    #
    #   bias     0.00     0.25     0.50
    #   along  -14.7 km  -4.3 km  -9.7 km
    #   sd       5.7      3.6      8.0
    #
    # 0.25 is the measured point and it is what is set; the peak is somewhere
    # between it and 0.50 and has not been resolved.
    #
    # More bias buys a shallower burn, and past a point that is the regime
    # the propagator is least reliable in -- the same lesson failure 21 paid
    # for from the other direction.  So this is a peak to sit on rather than
    # a gain to turn up, and the scatter agrees: 3.6 km at the peak against
    # 8.0 either side of it.
    DEORBIT_WINDOW_BIAS: float = 0.25
    # How far inside the gate to aim, in metres.  See ``deorbit_centring``:
    # the share-of-the-width knob above cannot reach past half a kilometre
    # on a 2-4 km window, and the bias it has to cancel is +5.3 km.  Zero
    # until an arm says what it should be.
    DEORBIT_CENTRE_BIAS_M: float = 0.0
    # The long corner is by construction the slowest entry in the box, so
    # ``DEORBIT_MAX_TIME_TO_GO_S`` vetoes it for every burn once the corners
    # are honest and the deorbit never commits at all.  The clock's real job
    # (failure 21) is to keep the *commitment* out of the shallow regime the
    # propagator cannot be trusted in -- a bound is not a commitment -- so
    # with the corners fixed it is applied to the short corner only.
    DEORBIT_WINDOW_TIME_ON_LONG: bool = True
    DEORBIT_MIN_WINDOW_M: float = 0.0
    # **The shortest entry the glide may be committed to.**  See
    # ``deorbit_centring``: the phase waits for a pass and takes the first it
    # can centre, and some passes put the runway 115 km nearer than the usual
    # one.  Measured over 41 flights from one quicksave:
    #
    #   pass                  n   arrival mean    sd     window
    #   2244-2254 km to run  28      +5.3 km   10.3 km   2-4 km
    #   2133-2184 km to run  13     +35.7 km   20.7 km  11-22 km
    #
    # Every short-pass flight landed long, because the vehicle cannot make
    # an entry that short and the propagator thinks it can.  Zero disables
    # the floor.  This is a stand-in for a missing model and says so; the
    # value is the split in the data and wants re-measuring on any change to
    # the entry, the airframe or the interface altitude.
    # **Flown, and it does more harm than the thing it prevents.**  The
    # short passes it refuses do land badly (13 of 41 flights, mean +36 km),
    # but refusing them is not free: a veto makes the phase *wait*, and what
    # it waits for is not a better version of the same pass -- it is a later
    # opportunity that may be badly out of plane.  At 2200 km, two of seven
    # flights committed on a normal-looking 2251 km arc and arrived
    # **+375 km long with 58 km of cross-track**, which is a pass whose
    # ground track does not go near the field.
    #
    # So the shape is wrong: this wants to be a *preference* among the
    # opportunities available at one moment, not a veto that spends
    # opportunities to get one. Left at zero, with the measurement recorded
    # in ``deorbit_centring`` for whoever writes the preference.
    DEORBIT_COMMIT_ARC_MIN_M: float = 0.0
    DEORBIT_TOLERANCE_M: float = 25000.0
    # How many times the dv search narrows around its best candidate.  The
    # band above is in metres of arrival and the search is in m/s of burn, and
    # at 25-290 km per m/s they are only commensurable after a few passes.
    DEORBIT_REFINE_PASSES: int = 5
    # Bias the burn so the entry arrives *long*.  This is the one-sided
    # constraint the whole design turns on: surplus energy can be dissipated
    # with angle of attack and bank, and a shortfall cannot be recovered at
    # any price, because nothing after the burn can add energy.
    #
    # **It is the lever after all, and the record here said otherwise for a
    # long time.**  A five-point sweep once concluded "the upper atmosphere
    # equalises the entry energy whatever the burn aimed at" -- across -60 to
    # +90 km the landings sat at -53, -78, -65, -57 km with no trend.  That
    # was an artefact of the acceptance band in ``deorbit_solution``, whose
    # tolerance was ``max(500, abs(DEORBIT_LONG_BIAS_M))``: **the aim and the
    # width of the band underneath it were the same number**, so raising the
    # aim widened the set of acceptable burns by exactly as much, and the
    # "take the smallest" rule stayed on the same shallow candidates. The aim
    # was never connected to the outcome, so of course it did not move it.
    #
    # With the band one-sided and its width its own constant, the same sweep
    # on one save reads:
    #
    #   aim      25 km   100 km   200 km   300 km
    #   landed  -85.1    -57.8    -18.9     -6.0
    #
    # and 300 km flown across four entry states gives -4.2, -4.8, -14.1 km
    # and one **+60 km overshoot** on the elliptical save, which is the state
    # with the most energy to start with.  So the aim is real, large, and not
    # a constant: what it is compensating is the propagator's over-prediction
    # of the glide's range, and that error scales with the entry rather than
    # sitting still.  A single fitted number is a calibration to *this*
    # airframe and *this* orbit, which is what
    # ``trajectory.Holdable``'s comment argues against; the value below is
    # fitted across the entry states in ``savegen.py`` and should be replaced
    # by an estimate of the over-prediction itself.
    # The entire aim now (see ``DEORBIT_LONG_BIAS_FRACTION``), and it is
    # small and slightly *short* because the cone moved the entry's target
    # 12.7 km further out along the centreline -- ``Runway.high_gate`` is on
    # the straight-in profile, not over the gate.  At the old 40 km the
    # vehicle crosses the field 52 km long, too far out to turn back.
    DEORBIT_LONG_BIAS_M: float = -3000.0
    # The aim as a share of the arc still to fly, which is the form the error
    # it compensates actually takes.  See ``deorbit_solution``.  0.27 is where
    # the three ~1100 km entry states land nearest at a fixed aim, applied as
    # a proportion so the shorter entries are asked for proportionally less.
    # **Re-fitted against the vehicle that can hold its attitude.**  0.27 was
    # right for a plant that under-delivered its commanded angle of attack by
    # a third; with ``ATTITUDE_TIME_TO_PEAK_S`` at 3.0 the same aim overflies
    # by 52-84 km.  Flown on one entry state, three instances per point, all
    # on the same deorbit pass (dv 50.9-51.3 m/s, 1490 km to run):
    #
    #   fraction   0.27    0.22    0.15
    #   along     +68 km  +36 km  +1.4 km
    #
    # -- a clean line at about 0.43 of the aim reaching the ground, which is
    # itself worth recording: the aim is compensating an over-prediction that
    # is *most* of what it adds, not a fixed offset.
    #
    # **And re-fitted again when the terminal glide was fixed**, which is the
    # thing to notice about this constant rather than its value.  Correcting
    # ``alpha_for_load`` at the fast end (failure 13) took the subsonic tail
    # of every propagation from L/D 1.46 to 2.1, and the same 0.15 that had
    # just been fitted then overflew by 58, 92 and 113 km -- the glide
    # arriving saturated at alpha 32 and 70 degrees of bank from 48 km down,
    # with nothing left to spend.  At 0.10 the same save lands 466 m and
    # 924 m from the runway midpoint with the bank sitting at +-42.
    #
    # The aim is the propagator's error wearing a constant's clothes, and it
    # has to be re-fitted after anything that changes what the vehicle flies.
    # Two flights is not a scatter; what it is, is a lever whose sign and
    # rough size are now measured four times.
    #
    # **And a fifth time, when the approach speed went up.**  Raising
    # ``APPROACH_FACTOR`` from 1.60 to 1.90 raises the target of the glide's
    # own speed cap by the same 14 m/s, which takes about four degrees of
    # angle of attack out of the terminal glide -- so the vehicle sinks
    # sooner and lands about 12 km shorter.  0.10 -> 0.13.
    #
    # That is five re-fits in one session, every one of them forced by a
    # change somewhere else, and the pattern is the finding: **this constant
    # is not a property of anything.**  What it stands in for is the
    # difference between the entry the deorbit search propagates (a fixed
    # alpha-30, bank-30 schedule) and the entry the glide actually flies, so
    # it moves whenever either does.  The thing worth building is an estimate
    # of that difference, not a better value for this.
    # **Zero, and the fraction is retired rather than re-fitted.**  It was a
    # share of ``Prediction.entry_arc`` calibrated against the *low* gate,
    # and under the cone the entry's target is the high one -- so the arc it
    # is a fraction of is a different arc, and 13% of it came to about 39 km
    # of aim that nobody chose.
    #
    # Worse than wrong: it made the aim a function of the candidate entry,
    # so two flights of one configuration accepted *different deorbit
    # opportunities* -- 2249 km to run against 2186, sixty-five kilometres
    # apart -- and landed 39 km apart because of it.  Set to zero the four
    # flights of the next batch all committed within 4 km of each other.
    # ``DEORBIT_LONG_BIAS_M`` is then the whole aim: one number, chosen.
    DEORBIT_LONG_BIAS_FRACTION: float = 0.0
    # How far below the atmosphere's top an arc has to get before it counts
    # as having *entered*, so that coming back out of it counts as a skip.
    # A margin rather than the boundary itself: an arc that grazes the very
    # top and leaves has not skipped, it has missed the atmosphere.  See
    # ``Prediction.skipped`` and spaceplane failure 8.
    #
    # NOTE: this field was accidentally deleted while rewriting the comment
    # above it and restored at 5000.0, which is a reconstruction and not
    # necessarily the number that flew the results recorded in this file.
    # Nothing on disk records the original -- no test pins it, no log carries
    # it, and it was never ``--set``.  It is plausible on its face (5 km
    # below a 70 km atmosphere) and the skip tests pass at it, but if skip
    # behaviour looks different from the write-ups, start here.
    SKIP_ENTER_MARGIN_M: float = 5000.0
    DEORBIT_MAX_TIME_TO_GO_S: float = 1500.0

    # -- thermal -----------------------------------------------------------
    # Nothing in this project had ever measured entry heating, which for a
    # Mach 7 entry held at maximum lift is an odd thing to have left out.
    # ``skin=`` on every telemetry line is the hottest part as a fraction of
    # its own skin limit, and crossing this threshold logs which part it is.
    # It is a *warning*, not a control input: the guidance does not yet trade
    # range against heat, and it should not start doing so on no data.
    THERMAL_WARN_FRACTION: float = 0.75

    # Kilogrammes per unit of LiquidFuel and Oxidizer -- 5 for both in stock
    # KSP.  It is here so the deorbit can predict at the mass the entry will
    # actually be flown at: the drain is 27% of the vehicle and it happens
    # *after* the burn, so a search run at ``snap.mass`` is aiming a
    # trajectory the vehicle never flies.
    RESOURCE_KG_PER_UNIT: float = 5.0
    # **Govern the time scale on the phase's worst tick, not its average.**
    # See ``common.pacing.LoopRate.peak``.  ``ScaleGovernor`` keeps a
    # decaying maximum because "what binds is the tail", and it was being fed
    # an exponential *average*, so the tail was gone before it looked.  On
    # ``DEORBIT`` that is the difference between a burn flown at 1.3x and one
    # flown at 6x, which on ``qs_plane_inc`` is the difference between
    # arriving within 2.2 km and arriving 9-12 km short.
    #
    # **Flown, and it is the default because it was measured, not argued.**
    # `pairfly.sh`, 8 an arm, halves swapped every round:
    #
    #   qs_plane_inc  DEORBIT interval 0.10-0.14 every flight against
    #                 0.10-0.31 bimodal; arrival **+199 sd 420** against
    #                 -4378 sd 5369, and the coarse mode gone entirely.
    #   qs_plane      8 of 8 on the runway against 7 of 8, intact 5 and 5,
    #                 stopped +838..+971 against +554..+1258 -- no
    #                 regression, and a five times tighter spread.
    #
    # It costs 4-5% of throughput (mean scale 5.25x against 5.49x), which is
    # the whole price of a thirteenfold reduction in arrival scatter.
    # Spaceplane failure 91.
    GOVERN_ON_PEAK: bool = True
    # How many of a phase's slowest ticks the peak sets aside: a one-off (the
    # fuel-to-nose scan, a phase's setup tick) pinned whole phases at 1-4x
    # when every other tick could serve 20x.  0 is the old undecayed peak.
    # ``LoopRate.peak_after``.
    GOVERN_PEAK_SKIP: int = 1
    # ... over only the last this-many game-seconds of the phase; 0 is the
    # whole phase.  Long enough that the deorbit's pre-ignition solve (0.8 s,
    # ~2 game-s before ignition) still governs the burn after it.
    GOVERN_PEAK_WINDOW_S: float = 60.0
    DEORBIT_ALIGN_DEG: float = 8.0
    # A burn that never satisfies its stop test must still end.  At 13 m/s^2
    # this is 780 m/s, well past anything the search can ask for.
    DEORBIT_MAX_BURN_S: float = 60.0
    # Do not even run the search while the runway is out of reach.  Offline
    # this airframe's entry range spans 590-1880 km depending on the burn, so
    # outside a band around that the answer is known in advance and the
    # sixteen propagations a search costs are pure waste -- and they are being
    # spent every couple of seconds against a game that wants the CPU.
    DEORBIT_RANGE_MIN_M: float = 400000.0
    DEORBIT_RANGE_MAX_M: float = 2300000.0
    # Waiting for a favourable pass is minutes of game time in which nothing
    # is commanded and nothing can go wrong -- the vehicle is in vacuum, on
    # rails, with the engine off.  So warp through it.  Warp is dropped the
    # moment a solution appears, and always before anything is commanded.
    #
    # **What bounds the factor is the arc a warped tick covers, and that has
    # to be measured rather than derived.**  This read "at 10x a 2 s tick is
    # 20 s of orbit, ~44 km of arc against a window hundreds of kilometres
    # wide, so the search still sees every pass it could act on", and every
    # term in that sentence is an assumption:
    #
    # - ``WARP_MAX_FACTOR`` is an *index* into KSP's rails-warp rate table,
    #   not a rate.  What index 2 means is the game's to decide, and a mod
    #   may redefine it -- ``BetterTimeWarp`` on one instance of this farm
    #   made it nine times what the comment assumed.
    # - The tick is not 2 s of wall clock.  A DEORBIT tick runs
    #   ``deorbit_solution``, which is sixteen entry propagations, so the
    #   more precisely the search resolves the burn the more sparsely it
    #   samples the orbit.  ``DEORBIT_REFINE_PASSES`` and this are coupled
    #   and nothing said so.
    # - Off 1x the timescale plugin multiplies it again.
    #
    # Measured, the product came to **180 s of orbit per tick** where 20 was
    # assumed, which warped straight over the window a pass was solvable in:
    # two instances loading the same save at the same UT took different
    # passes, 51 m/s against 105, and landed 23 km apart.  See failure 11.
    #
    # So the loop watches the arc each warped tick actually covered and backs
    # the factor off when it exceeds the budget, the way ``Holdable`` learns
    # the alpha ceiling.  It starts at ``WARP_MIN_FACTOR`` and climbs on
    # evidence rather than starting at the top and correcting after the
    # damage: a step that is too small costs wall-clock seconds, and a step
    # that is too large costs the pass.
    WARP_WAIT: bool = True
    WARP_MAX_FACTOR: int = 2
    WARP_MIN_FACTOR: int = 1                # where it starts, and the floor
    # The most orbit one warped tick may cover.  44 km is what the factor
    # that worked actually achieved, and 390 km is what the one that skipped
    # the pass did, so the window is somewhere between; take the side that
    # cannot skip.
    # **And it is not the repeatability of the commit**, which is worth
    # writing down because it looked exactly as though it were.  Flights of
    # one save commit anywhere in a ~55 second window and solve dv from 44.8
    # to 51.3 m/s there, which reads as a warp step deciding which burn is
    # flown.  It is not: measured at the 65 km interface, the entries those
    # burns produce are the same entry -- 899, 904, 930, 939, 942, 955 km of
    # arc still to run, across flights that landed anywhere from 3 km short
    # to 113 km long.  A later commit needs more dv and gets the same
    # trajectory, which is what a one-parameter family does.  So the burn
    # point is *not* the large term, the aim is, and a finer warp step buys
    # wall-clock cost for nothing.  Left at 50 km.
    WARP_MAX_ARC_M: float = 50000.0

    # **And warp the ballistic fall as well, which is a third of the flight.**
    # Measured on ``logs/LOG1015``: the deorbit wait is 275 s of game time
    # (already warped), the coast from the drain to the 58 km interface is
    # **470 s**, and the glide is 745 s.  Of that coast, 295 s -- 63% of it --
    # is spent above the atmosphere, in vacuum, with the engine off, nothing
    # commanded and no decision being taken.  Those are the cheapest seconds
    # in the flight to delete, and the farm's throughput is the standing
    # constraint on every question this project asks.
    #
    # It is a different risk from the deorbit's warp and the difference is
    # the reason this is a separate knob.  There, a warped tick that covers
    # too much orbit can step over the window a pass is solvable in, which
    # costs the flight (failure 11).  Here nothing is being solved: the
    # vehicle is on a ballistic arc it has already committed to.  Above
    # ``atmosphere_depth`` KSP's on-rails propagation is the same two-body
    # problem the propagator integrates, so this should not move the
    # trajectory at all -- *should*, which is an argument, and this project
    # does not adopt arguments.  Measured against the same save with it off
    # before it is turned on.
    #
    # **Flown: 973 s against 1163 s for three flights, both warped arms
    # agreeing to the second.  63 s a flight, 16%.**  The accuracy side is
    # *not* resolved -- six warped flights against the unwarped ones differ
    # by 4 km against a combined standard error of 3.3, which on a vehicle
    # with a 10 km scatter means "no evidence either way" and not "no
    # effect".  It is on because the throughput is certain, the mechanism
    # for harm is hard to construct (above ``atmosphere_depth`` KSP's
    # on-rails conic is the same two-body problem the propagator
    # integrates), and every future measurement is 16% cheaper.  If an
    # entry-state comparison ever comes out strangely, this is a knob to
    # turn off first.
    #
    # The release is ``COAST_WARP_STOP_M`` above the atmosphere rather than
    # at it, because rails warp freezes the attitude: the entry angle of
    # attack is established in vacuum on purpose (see ``run_coast``) and has
    # to be re-established by the autopilot before the air arrives.  At the
    # 55 m/s the vehicle is falling, 2 km is 36 seconds, and there are a
    # further 12 km of vacuum-ish coast under that.
    COAST_WARP: bool = True
    COAST_WARP_FACTOR: int = 3              # an index into KSP's rate table
    COAST_WARP_STOP_M: float = 2000.0       # release this far above the air

    # -- the drain ---------------------------------------------------------
    # 2.780 t of the 9.495 t on board, so 29% of the vehicle and a 29% cut in
    # wing loading.  It is not housekeeping, it is the largest single
    # aerodynamic change in the flight, and every prediction after it has to
    # be made at the drained mass -- hence its own phase, in vacuum, before
    # anything aerodynamic depends on the answer.
    DRAIN: bool = True
    DRAIN_RESOURCES: tuple = ("LiquidFuel", "Oxidizer")
    # How many times the phase may watch a tank that does not move before
    # it gives up and flies wet.  ``DRAIN_TIMEOUT_S`` bounds one attempt;
    # without this the phase simply re-enters and tries again forever, which
    # is what a valve in the wrong mode actually did (see
    # ``_open_one_drain``).  Flying wet is bad -- it is failure 61's
    # disturbance back in the flight -- but it is a flight, and it is logged
    # in capitals.
    DRAIN_MAX_ATTEMPTS: int = 3
    DRAIN_TIMEOUT_S: float = 90.0
    DRAIN_REMAINING_UNITS: float = 0.5
    # **The release valve is worth 3.3 to 9.3 m/s, it fires after the burn's
    # closed loop has finished, and it is the whole of why nothing
    # transfers.**  Measured with ``cutoff drift``, four arms interleaved,
    # specific energy between the burn's cutoff and the first COAST tick:
    #
    #     |              | qs_plane      | qs_plane_inc  |
    #     | drain on     | -3.1 to -3.5  | -9.0 to -9.4  |  m/s equivalent
    #     | ``DRAIN=False`` | **-0.27**  | **-0.36**     |
    #
    # The engine is cold through all of it (``Fn`` reads 0.0 kN on the first
    # tick after cutoff, and shutting the engine down changes nothing --
    # ``DEORBIT_CUTOFF_SHUTDOWN``).  It is the valve.
    #
    # Downstream that is the handover error this project has spent a dozen
    # mechanisms on: the vehicle reaches the entry interface **+28 km**
    # further from the gate than the burn predicted on ``qs_plane`` and
    # **+127 km** on ``qs_plane_inc``, in the right state and the wrong
    # place, and the difference between those two is the difference between
    # a landing and a wreck.  No constant can express it, because it is not
    # a modelling error at all -- it is dv the model never hears about,
    # delivered after the last thing that could have answered for it.
    #
    # **So move it upstream of the loop instead of modelling it.**  With this
    # on the valve runs *before* the deorbit burn and stops at
    # ``DRAIN_RESERVE_UNITS``; the burn is then solved and flown at the
    # drained mass, and its stop test measures a state no valve is going to
    # change afterwards.  The reserve stays aboard for the entry -- 40 units
    # is 200 kg on a 6.7 t vehicle, 3% of the mass against the 29% the drain
    # exists to shed -- rather than being dumped where nothing can answer
    # for the impulse.
    #
    # What would contradict it: a burn that runs out of propellant (the log
    # shows ``F`` falling to zero mid-burn, or ``burn guard``), which means
    # the reserve is too small.  The burns flown here are 31-33 m/s and
    # spend ~13 units.
    # **Defaulted on.**  This is a bug fix and not a fitted constant: it
    # removes an impulse the burn's closed loop cannot see, on any vehicle
    # with a release valve, and it needs no number chosen against any save.
    # Measured across three entry states it is the largest single change
    # this project has made -- see the table in ``DRAIN_RESERVE_DV_MS``.
    DRAIN_BEFORE_BURN: bool = True
    DRAIN_RESERVE_UNITS: float = 40.0
    # **And 40 units was not enough, on the first entry state that asked.**
    # ``qs_plane_high`` solves a 121.6 m/s deorbit where the other two solve
    # 32-44; the reserve bought 69 and the burn died on ``burn guard at
    # 60 s``, 800 km from the field (LOG2204).  So the reserve is sized from
    # a *dv budget* through the rocket equation, at the Isp the vehicle
    # reports and the mass it will have -- the units figure above is only a
    # floor for the case where Isp is not readable.  200 m/s covers every
    # burn this vehicle has ever solved with room over, and costs about
    # 480 kg on a 6.5 t entry (7%) against the 29% the drain sheds.
    #
    # What would contradict it: a ``burn guard`` exit, or the "N m/s of burn
    # at Isp M" line reading near the solved dv.  Both are in every log.
    #
    # **With the reserve sized this way, all three entry states arrive at the
    # field for the first time.**  Arrival (predicted along-track miss at
    # ``GLIDE -> HAC``), same configuration throughout:
    #
    #     |                  | before      | after            |
    #     | ``qs_plane``     | -1.1 sd 0.4 | **+1.1**         |
    #     | ``qs_plane_high``| -17.3 sd 1.4| **+3.1 sd 0.9**  |
    #     | ``qs_plane_inc`` | -56.0 sd 6.0| **-5.3**         |
    #
    # The spread across the three states goes **74 km -> 8.4 km**, and the
    # state that had never once reached the field is now the tightest of the
    # three.  Deorbit dv is 32, 43 and 121 m/s respectively, which is what
    # the units-based reserve could not cover.
    DRAIN_RESERVE_DV_MS: float = 200.0
    DRAIN_RESERVE_MARGIN: float = 1.25

    # -- the aerodynamic table ---------------------------------------------
    # Cl*A and Cd*A against (alpha, Mach), both probed.  Two dimensions and
    # not one because this airframe's lift varies ninefold between subsonic
    # and Mach 5 (Cl*A at 12 deg: 36 m^2 at M0.15, 5.5 at M2.35) while a
    # booster could hold one scalar against Mach alone.
    ALPHA_BINS: tuple = (0.0, 2.0, 5.0, 8.0, 12.0, 16.0, 20.0, 25.0, 30.0,
                         35.0, 40.0, 50.0, 65.0, 90.0)
    MACH_BINS: tuple = (0.0, 0.3, 0.6, 0.9, 1.2, 1.6, 2.0, 2.5, 3.0, 4.0,
                        5.0, 6.0, 8.0)
    # Print the swept table into the log once, in STANDBY.  Thirteen lines
    # where nothing is happening, and they carry every alpha bin out to 90
    # degrees -- which the vehicle has been measuring every flight and no
    # log has ever shown.  ``Config`` is where the *bins* live; what the
    # airframe answers in them belongs in the log, not in a constant here.
    AERO_DUMP: bool = True
    LIFT_TRIM_MACH_BIN: float = 0.5
    LIFT_TRIM_SMOOTHING: float = 0.15
    LIFT_TRIM_MIN_SAMPLES: int = 8
    # A ratio outside these is a bad sample, not a discovery: a bank
    # reversal mid-slew, or a tick whose force read stale.
    LIFT_TRIM_MIN: float = 0.4
    LIFT_TRIM_MAX: float = 3.0
    # Below this the table's own lift is too small to divide by.
    LIFT_TRIM_MIN_CLA: float = 0.5
    # Below this dynamic pressure the reported force is mostly noise and the
    # ratio taken from it says nothing about the airframe.
    LIFT_TRIM_MIN_Q_PA: float = 200.0
    # The altitude each Mach bin is probed at.  KSP scales drag by a
    # pseudo-Reynolds term as well as Mach, so the same Mach reads lower down
    # low; probing each bin in the air it will be *used* in is what
    # boosterland's DRAG_PROBE_DESCENT_ALTITUDE is for.  Refined from the
    # prediction's own profile once there is one.
    PROBE_ALTITUDES: tuple = (200.0, 1000.0, 3000.0, 5000.0, 8000.0, 11000.0,
                              14000.0, 18000.0, 22000.0, 30000.0, 38000.0,
                              45000.0, 55000.0)
    PROBE_SPEED_FLOOR: float = 30.0
    AERO_REFRESH_UT: float = 1.0
    AERO_ROWS_PER_REFRESH: int = 2
    AERO_SMOOTHING: float = 0.35            # new sample's weight
    SOUND_SPEED_FALLBACK_M_S: float = 340.0
    DENSITY_TABLE_STEP_M: float = 250.0

    # -- the approach and rollout laws -------------------------------------
    # Angle of attack tracks the path through the sink rate, on a feedforward
    # of the angle that carries the weight at the current speed -- read out of
    # the measured table rather than trimmed by hand, so a different airframe
    # gets a different answer from the same code.
    # **Speed hold, not path hold.**  The first version commanded angle of
    # attack from the sink-rate error, which is the law an aircraft with a
    # throttle uses and is exactly backwards for a glider: arriving high, it
    # asked for *less* alpha, which unloads the wing, and the vehicle traded
    # its excess height for speed rather than for drag.  Offline it reached
    # the threshold at 102 m/s with a 36 m/s sink rate -- on the runway, and
    # a crater.  A glider descends faster by adding drag; the only way to
    # spend height safely is to arrive at the right speed and S-turn off
    # whatever is left.
    APPROACH_SPEED_KP: float = 0.22         # deg of alpha per m/s of excess
    APPROACH_PATH_KP: float = 0.004         # deg of alpha per m of excess height
    APPROACH_PATH_LIMIT_M: float = 250.0    # how much height the trim may chase
    APPROACH_SCURVE_M: float = 200.0        # excess height that starts S-turns
    APPROACH_SCURVE_KP: float = 0.06        # deg of bank per m of excess
    # **The S-turn commands a track angle, not a bank magnitude.**  The
    # version this replaces set only the magnitude and left the direction to
    # the centreline capture, which points at the centreline -- so the bank
    # sat on its 40 degree limit while the *track* never left the
    # centreline by more than ten degrees, and ten degrees is 1.5% of extra
    # path.  ``logs/LOG1315`` flew twenty ticks of that, stayed inside
    # +/-30 m of cross-track throughout, and overflew the aim by 2.5 km.
    # See ``guidance.approach``; ``False`` restores the old law together
    # with ``APPROACH_SCURVE_KP``.
    APPROACH_SCURVE_TRACK: bool = True
    # The widest the weave may point off the centreline.  ``1/cos(45)`` is
    # 41% more path, which over the 2.5-3 km of final the stop distance
    # leaves is 700-900 m of height -- about the surplus the cone is allowed
    # to hand over (``HAC_EXIT_SURPLUS_M``), which is not a coincidence:
    # a phase may only tolerate what the next one can fly.
    APPROACH_SCURVE_MAX_DEG: float = 45.0
    # How far the weave may stray from the centreline before the band, not
    # the clock, reverses it.  The capture arrests this much offset in about
    # 1.4 km at the approach speed, which is why the S-turn stops there.
    APPROACH_SCURVE_CROSS_M: float = 300.0
    APPROACH_SCURVE_STOP_M: float = 4000.0  # no weaving inside this of the aim [default 2026-10-03, rot-orbit2-1003]
    APPROACH_SCURVE_PERIOD_S: float = 10.0  # half-cycle of the weave clock

    # -- the split-rudder airbrake ----------------------------------------
    AIRBRAKE_OPPOSED_FLAPS: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # Stow the brake whenever the sink exceeds what the flare can arrest
    # from its door (``airbrake.Brake.update``).  The opposed flaps moved the
    # touchdown ~600 m earlier on `qs_plane` and broke 3 of 6 vehicles doing
    # it, all at 38-58 m/s of sink into a flare that can take ~39.
    # **Flown: a no-op.**  The unbraked approach already sinks 46-49 m/s at
    # 1-2 km, so the guard is always tripped and the brake never deploys
    # (`pairfly-flapguard.txt`).  And the extra sink was the speed loop
    # diving to recover the speed the brake took, not the brake itself --
    # see docs/spaceplane/design.md.  Left off.
    AIRBRAKE_SINK_GUARD: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **Stow when the vehicle sinks faster than the approach wants.**  The
    # shuttle's measured set is a lift spoiler (-33 ClA, ~40% of the lift):
    # flown on a 1300 m surplus (LOG3593) it took the sink from 36 to 106
    # m/s against a wanted 25 then 11, spent the surplus in 20 s, stowed on
    # "surplus spent" at 1877 m and left a 110 m/s sink that needs ~1100 m
    # to arrest -- flare at 162 m/s, sink 149.  The approach already computes
    # the sink its path wants (``ApproachCommand.wanted_sink``); the brake is
    # out only while the vehicle is not sinking faster than that plus this.
    AIRBRAKE_SINK_TRACK: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    AIRBRAKE_SINK_TRACK_M_S: float = 5.0
    # **Choose each surface's deploy sense by deploying it**, in vacuum,
    # once, against the game's own wrench (``Autopilot.measure_flap_brake``,
    # ``airbrake.MeasuredBrake``).  Positive ``Deploy Angle`` is a per-part
    # direction: on the shuttle it spoils lift on one elevon pair and adds it
    # on the other two, so the geometric brake was half a flap.  Needs
    # ``AIRBRAKE_OPPOSED_FLAPS``; replaces its geometry.
    AIRBRAKE_MEASURED: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    AIRBRAKE_MEASURE_DEG: float = 15.0      # the probe deployment
    AIRBRAKE_MEASURE_SETTLE_S: float = 0.6  # wall-s for a surface to move
    AIRBRAKE_MEASURE_ALT_M: float = 1000.0  # the approach's air
    AIRBRAKE_MEASURE_ALPHA_DEG: float = 5.0
    # The verified set's residual pitching moment may be at most this share
    # of the largest single surface's -- beyond it the set is refused.
    AIRBRAKE_MEASURE_MOMENT_FRAC: float = 0.5
    # Deflections do not add (a set predicted to cancel measured +55 CmA on
    # the shuttle), so the balance is re-probed and corrected this often.
    AIRBRAKE_MEASURE_ITER: int = 5
    # **The instrument, and it flies before the law does.**  Deploy the
    # opposed flaps at this angle (the aft group; the forward group gets
    # ``angle * ratio``) from COAST down, and read back three things the
    # geometry cannot tell you: whether the moment really cancels (``aoa=``
    # tracking across the deployment), what it costs in ``ClA`` and gives in
    # ``CdA`` (``act=``, which needs no model), and therefore whether it is a
    # lift spoiler or merely a brake.  The same pattern as
    # ``SLIP_PROBE_DEG``, which settled the sideslip question in one round.
    FLAP_BRAKE_PROBE_DEG: float = 0.0
    # How far the halves deploy.  The parts' own ``Deploy Angle`` default is
    # 20 degrees and that is what this asks for; the module is driven through
    # the generic interface, the same route ``_find_drain`` uses, because the
    # four mods this install keeps replace the stock control-surface module.
    AIRBRAKE_DEPLOY_ANGLE_DEG: float = 20.0
    # **The trigger is the guidance's own signal that it is out of
    # authority, not an altitude.**  Measured over the batches of
    # 2026-09-21, the approach's weave command sits at
    # ``APPROACH_SCURVE_MAX_DEG`` for 52% of approach ticks with +639 m of
    # surplus unspent.  So: the cap held for this share of a
    # window, with this much surplus still to spend.
    AIRBRAKE_SATURATED_FRAC: float = 0.5
    AIRBRAKE_CAP_EPS_DEG: float = 0.5       # "at the cap", in degrees
    # **The minimum sample, in weave cycles -- not a time constant.**  This
    # was an exponential average over two cycles and it measured nothing:
    # the average starts cold at the phase boundary, two cycles is half the
    # approach, and it was still charging when the vehicle reached the stow
    # height.  Flown (LOG2815-2818): two flights deployed for seven seconds
    # at 702 m, two peaked at 0.44-0.48 and never armed.  The share is now
    # capped-seconds over elapsed-seconds, which is the quantity the
    # mechanism was built from ("the cap for 52% of approach ticks") and is
    # unbiased from the first tick.  This is only how much phase must have
    # elapsed before the share is believed, because three ticks at the cap
    # is where the weave *is*, not evidence that it is stuck there.
    AIRBRAKE_MIN_CYCLES: float = 1.0
    # **The surplus that arms the brake is the surplus that started the
    # weave**: ``APPROACH_SCURVE_M``, read directly rather than copied.  The
    # brake exists for the case where the weave is running and has run out,
    # so a second constant here would be the same quantity under a second
    # name -- the failure this project has paid for repeatedly.  The release
    # is a fraction of it, which is the hysteresis and nothing else.
    AIRBRAKE_RETRACT_FRAC: float = 0.5
    # **Never into the flare, expressed as a time.**  The flare arrests the
    # sink with the speed it arrives with, and a brake still out turns a
    # float into a drop.  The door is ``FLARE_ALT_M + FLARE_LEAD_S * sink``
    # on whatever vehicle this is -- about 470 m here -- and the brake stows
    # this many seconds of sink above it, so the margin scales with the
    # aircraft's own flare constants and its sink rate instead of being a
    # height fitted to this one.
    AIRBRAKE_STOW_LEAD_S: float = 4.0
    # **The brake may spend height; it may not spend speed.**  Flown, and it
    # destroyed two vehicles: the pair took 21 m/s out of the approach
    # (105.6 -> 84.1 m/s, LOG2825), and a glider can only make speed by
    # trading height for it, so the speed loop dived to recover and reached
    # the flare door at 250 m with **79-82 m/s of sink** against 29-36
    # unbraked (LOG2824-2825, both destroyed on contact; the unbraked
    # controls of the same round landed intact at +900 and +953).
    # As a multiple of ``guidance.approach``'s own target speed, so it costs
    # nothing to state and needs no per-craft number: 1.0 means the brake
    # retracts the moment the vehicle is slower than the approach wants.
    AIRBRAKE_SPEED_GUARD: float = 1.0
    # The geometry tolerances the identification refuses outside of: how
    # closely the pair must mirror about the centreline and share a station,
    # how far off it neither may sit, and how unequal their areas may be.
    # Wide enough for a hand-built craft, narrow enough that an unbalanced
    # pair is rejected rather than flown -- an uncommanded yaw on final is
    # failure 34, which tore the gear off at 48 m/s.
    AIRBRAKE_PAIR_TOL_M: float = 0.20
    AIRBRAKE_MIN_OFFSET_M: float = 0.20
    AIRBRAKE_AREA_TOL: float = 0.25
    # How far the configured landing constants may sit from what the swept
    # table says before the log calls it out.  See
    # ``Autopilot.report_airframe``: this does not change the flight, it
    # makes a disagreement impossible to miss, which is the whole lesson of
    # failure 13 -- those constants were wrong by 1.8x for the life of the
    # project because nothing could contradict them.
    AIRFRAME_DISAGREE_FRACTION: float = 0.15
    # **Fly the swept table's numbers instead of the transcribed ones.**
    # The line above only complains; this is what acts on the complaint.
    # With it on, ``STALL_SPEED_M_S``, ``APPROACH_BEST_LD`` and ``HAC_LD``
    # stop being flown and become fallbacks for when the sweep fails --
    # the landing chain is sized on the aircraft that is actually about to
    # be flown, which is the only version of this autopilot that can fly a
    # craft file it was not fitted to.  ``airframe.stall``, ``glide_ld``
    # and ``cone_ld`` are the three readers.
    #
    # It is a switch and not an unconditional change because the committed
    # configuration lands 88% intact and this moves three numbers under it
    # at once: on ``qs_plane`` the derived stall is 55.0 against the
    # configured 48.0, which raises the approach floor from 96 to 110 m/s
    # and moves the gate, the flare and the touchdown with it.  That has to
    # be flown as an arm, not assumed.  ``pairfly.sh`` with
    # ``--set AIRFRAME_DERIVED=True`` is the measurement.
    # Flying the vehicle's own numbers stopped being safe to bundle with the
    # lift discount, which was refuted in flight -- see
    # ``airframe.lift_discount``.  This flag now covers the cone's glide
    # ratio and the alpha ceiling; the stall is always the table's.
    AIRFRAME_DERIVED: bool = False
    # **The approach's ground-per-height, on a switch of its own, because it
    # is the one that moves.**  ``AIRFRAME_DERIVED`` above replaces two
    # numbers the derivation reproduces -- the cone's ratio derives to 1.81
    # against a fitted 1.86, and best glide to 3.07 against the 3.06 in
    # CLAUDE.md -- so on this airframe it is nearly inert and only bites on
    # a different one, which is the point of it.
    #
    # ``APPROACH_BEST_LD`` is not like that.  It is 4.2; its own comment
    # below says it is ground from the rollout to the wheels over 41
    # flights, "mean 2.08 sd 0.27", low quartile 1.85; and the derivation at
    # the speed the approach is actually flown at says 2.33.  The constant
    # is twice everything that claims to measure it, and it is nonetheless
    # in the configuration that lands 88% intact.  Either it is carrying
    # something nobody wrote down -- failure 21's shape, a guard whose
    # stated reason does not survive inspection but which is the only thing
    # enforcing a constraint -- or it is a fit that happens to work.
    #
    # Halving it is therefore a real change with a documented failure mode
    # on exactly this constant (failure 23, twice), so it does not ride
    # along with the others.  Fly it as its own arm and find out which.
    # **Do not turn this on.**  The batch of 2026-09-21 measured the
    # rollout-to-wheels ratio at 3.96 sd 0.41, so the committed 4.2 is
    # right and this derivation's 2.33 is not -- it omits the flare, which
    # is flat and covers 400-500 m of ground for 150 of height.  The
    # "2.08 over 41 flights" below describes a configuration no longer
    # flown.  See ``airframe.approach_ld``.
    APPROACH_LD_DERIVED: bool = False
    # **The L/D the approach actually flies, which is not best glide.**  2.05
    # is what the airframe manages at its best-glide angle around 70 m/s.
    # The approach is deliberately flown fast now (see ``APPROACH_FACTOR``),
    # and fast is draggy: measured at the flare entry, v 127.3 m/s with
    # 69.3 of sink is **L/D 1.54**, and 120.6 with 67.7 is 1.50.  Using the
    # best-glide figure here is the same mistake failure 13 made with
    # ``planeprobe``'s -- a number measured in one regime used to place
    # geometry in another.
    # **Re-measured, because the approach is not flown at 127 m/s any
    # more.**  1.55 was taken at the flare entry when ``APPROACH_FACTOR``
    # was 2.40; the ground distance per metre of height over the whole
    # APPROACH phase, read off fourteen logs at the speeds it is flown at
    # now, is **1.65 to 2.07, mean 1.87** -- and it barely moves whether the
    # vehicle is high or low, which is the same measurement that says the
    # S-turn is worth a few per cent rather than a third. Understating it
    # understates ``excess``, which is what the S-turn is sized from and
    # what the gate's margin is checked against. Failure 23's rule: a
    # quantity measured from flight data describes the vehicle *as it was
    # flown*.
    # **And measured from the rollout to the wheels, not over APPROACH.**
    # What this number sizes is where the vehicle touches down, so the span
    # it has to describe is the whole of it -- the approach *and* the flare,
    # which is flat and covers 400-500 m of ground for 150 of height.  Over
    # 41 flights, ground from the rollout to the first wheel contact divided
    # by the height there:
    #
    #     mean 2.08   sd 0.27   [1.67 .. 3.03]
    #
    # 1.55 was taken at the flare entry when the approach was flown at
    # 2.40 times the stall; 1.95 was this measurement taken over the
    # APPROACH phase alone and stopped at the flare. Understating it makes
    # the cone hand over high, and the cone converts a metre of that into
    # two metres of runway: at 1.95 the handover is 340 m high and the
    # wheels land 700 m late. Failure 23, twice on the same constant.
    # **And 2.08 -- the mean -- is the wrong end of the spread to use.**
    # This number has two consumers that pull opposite ways: it sets what
    # the *approach* is asked to reach, and through ``approach_needed`` it
    # sets when the cone is allowed to hand over.  At 2.08 the handover test
    # asks for a height the cone's own descent profile never reaches at the
    # rollout, so the exit stops firing there and the vehicle leaves through
    # the ``GATE_ALT_M`` floor instead -- flown, **eight of eight** exits
    # were "out of height", 2.0 to 3.2 km below profile, and none of them
    # landed.  A margin constant wants the pessimistic end of its own
    # measurement, and the low quartile of the 41 flights is about 1.85.
    # **4.2 is current; the "2.08 sd 0.27" below is not.**  ``landsum.py``
    # measures the rollout-to-wheels ratio at 3.96 sd 0.41 today, on the
    # configuration that lands 88% intact.  Do not "correct" the constant
    # towards the stale comment, and see ``APPROACH_LD_DERIVED`` for why the
    # derivation's 2.33 is a different quantity (it omits the flare, which is
    # flat and covers 400-500 m of ground for 150 of height).
    APPROACH_BEST_LD: float = 4.2# rollout to wheels, low end of 41
    # **The approach's speed loop could only ever slow down.**
    #
    # ``trim + APPROACH_SPEED_KP * (speed - target)``, floored at ``trim`` by
    # ``APPROACH_TRIM_FLOOR``, has no way to make speed: the floor is one g,
    # and one g on a vehicle descending at its best glide angle of 18 degrees
    # is a standing *pull-up* -- a straight path there needs ``cos(18) =
    # 0.95``.  So a vehicle that arrives at the gate a little slow flattens,
    # slows further, and ``trim`` climbs with the slowing, which adds the drag
    # that slows it again.  Measured on ``logs/LOG2246``, at an honest control
    # rate: 96.6 m/s at 830 m, decaying monotonically to **59.9 at the flare**
    # against the flare's measured 83-91 window, three kilometres past the aim
    # point, and destroyed.
    #
    # It never showed at 6x because the loop was too coarse to obey its own
    # command (failure 63) -- the two are the same finding from two ends.
    #
    # With this on, the command is the *descent angle that holds the speed*,
    # flown as the load that flies that angle:
    # ``dv/dt = g sin(theta) - D/m`` against ``L = m g cos(theta)``.  See
    # ``guidance.alpha_for_speed``.  ``APPROACH_SPEED_KP`` and
    # ``APPROACH_TRIM_FLOOR`` are then unused, and the speed floor becomes a
    # floor under the *target* rather than a clamp on the angle.
    APPROACH_SPEED_PATH: bool = True
    # A rate term on the speed law, per unit of measured dv/dt over g (0 =
    # off): damps the approach's phugoid.  See ``guidance.alpha_for_speed``.
    APPROACH_SPEED_KD: float = 0.0
    # Fly the one-g angle at the target speed on final rather than a speed
    # law (see ``guidance.approach``).
    APPROACH_ALPHA_AT_TARGET: bool = True  # default 2026-10-03: the landing stack, rot-orbit2-1003
    APPROACH_ACCEL_TAU_S: float = 1.0
    # How long a speed error is given to disappear.  A time, not a gain: it
    # multiplies nothing that has to be re-fitted when the mass, the air or
    # the approach speed changes.
    APPROACH_SPEED_TAU_S: float = 6.0
    # The steepest descent the speed law may command, as a dive angle.  This
    # is ``APPROACH_TRIM_FLOOR``'s caution moved to where it belongs: the 27
    # flights it was written for unloaded to 4-6 degrees of alpha and arrived
    # 40-65 degrees below the horizon, and a bound on the *path* says that
    # cannot happen whatever the speed or the mass.  ``cos(35)`` is 0.82 g.
    APPROACH_DIVE_MAX_DEG: float = 35.0
    # The inner loop's gain, in g per radian of path error.  ``L = m g cos
    # theta`` describes a *steady* glide, and commanding it open-loop is a
    # positive feedback on an airframe that delivers 85% of the angle it is
    # asked for: the path steepens past the target and the table answers the
    # resulting speed with a lower angle still (``logs/LOG2247``, 104 m/s and
    # 44 of sink running away to 124 and 72, into the ground at 98).  At 1.5
    # a ten degree excess dive buys a quarter of a g of pull-up.
    APPROACH_PATH_KN: float = 1.5
    # And a ceiling on what the speed law may ask the wing for, so it cannot
    # start flying the flare's manoeuvre a kilometre early.
    APPROACH_LOAD_MAX: float = 1.6
    APPROACH_ALPHA_MAX_DEG: float = 28.0
    APPROACH_SPEED_FLOOR_FACTOR: float = 2.00   # x stall, before the flare
    # 55 was tried and it is too much lift to give away: at 55 degrees the
    # wing needs 1.74 g merely to hold the path, and this airframe on final
    # does not have it, so the vehicle sinks instead of turning.  Measured,
    # the flights that ran the wider limit arrived at the flare doing 138 m/s
    # with **97 m/s of sink**, against 108 m/s and 49 on the same
    # configuration at 35 degrees -- which touched down at -0.68 m/s of sink,
    # the best arrival this project has flown.  40 is a small widening for
    # the faster approach, not a licence to bank.
    APPROACH_BANK_MAX_DEG: float = 40.0
    RCS_PITCH_GATE_S: float = 2.0
    APPROACH_BANK_BY_ROLL: bool = True  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # **Whether the speed loop knows it is in a turn.**  See
    # ``guidance.alpha_for_speed``: the load it solves for is vertical and
    # the vehicle is banked, so the wing must carry ``1/cos(bank)`` to fly
    # the same descent.  At the 40 degree S-turn limit that is 1.31, and
    # without it the approach dives exactly when it has decided it is high.
    # **Whether ``engage`` may start in APPROACH or FLARE.**  See
    # ``Autoland.engage``: a vehicle handed over on short final should not be
    # given an entry to solve.  It is also what makes a save on final
    # useful, and a save on final is thirty seconds a data point against
    # thirteen hundred -- which is the difference between iterating the
    # touchdown and guessing at it.
    ENGAGE_INTO_LANDING: bool = True
    # Off for the same reason as ``BRAKE_FOR_DISTANCE``: the physics is not
    # in doubt (a banked wing carries 1/cos(phi) to fly the same descent, and
    # nothing divided by it) but the flight is, and its batch was stopped
    # before it had an answer.
    APPROACH_BANK_COMPENSATION: bool = True
    # ``1/cos`` runs away at the vertical, so the compensation is computed on
    # a bounded bank.  Above this the vehicle is not flying an approach any
    # more and a larger load demand is not the answer.
    APPROACH_BANK_COMP_MAX_DEG: float = 60.0
    APPROACH_CROSS_KP: float = 0.06         # deg of bank per m off centreline
    APPROACH_HEADING_KP: float = 1.1        # deg of bank per deg of track error
    # **The old pair above are not speed-aware, and the approach is now flown
    # 50% faster than they were fitted at.**  Turn rate for a given bank goes
    # as ``g tan(bank) / v``, so the same gains give a 1.9 km turn radius at
    # 115 m/s where they gave 900 m at 77 -- and the lead term, the ``12.0``
    # metres-per-degree in the sign test, is a distance that should scale as
    # ``v^2``.  Measured on three flights at 115 m/s: the glide handed over
    # at -847, -2427 and +5300 m of cross-track and the approach turned
    # through the centreline and settled on the *other* side at +616, +644
    # and +1751.  Same overshoot every time, which is a gain, not noise.
    #
    # So the lateral law becomes a capture rather than two proportional
    # terms: the vehicle is asked for the lateral closing *rate* it can still
    # arrest in the offset it has left -- ``sqrt(2 a |cross|)`` with ``a`` the
    # lateral acceleration the bank limit affords -- and the bank commands
    # the error between that and the rate it has. One physical number
    # instead of two fitted ones, and it is speed-aware by construction:
    # every term in it is a speed or an acceleration.
    APPROACH_LATERAL_CAPTURE: bool = True
    APPROACH_CAPTURE_KP: float = 2.0        # deg of bank per m/s of rate error
    # **The capture and the S-turn against the roll axis's lag** (2026-10-03).
    # ``guidance.approach``: the rate loop's gain is set so its own lag is
    # ``APPROACH_CAPTURE_LAG_FACTOR`` x roll's time_to_peak (KP above becomes
    # a cap), and the S-turn asks for no more sideways rate than
    # ``lateral * (time to the door - that lag) / 2`` can take back.  On the
    # shuttle (roll 5.3 s) KP 2.0 relayed +-40 deg and left the flare door
    # 60-525 m off the centreline (save-steer-1003, LOG5087-98).
    APPROACH_CAPTURE_LAG_AWARE: bool = True  # default 2026-10-03: with SCURVE_FULL_GAIN + HAC_ENERGY_BUDGET, save-energy-1003 + save-energy2-1003
    APPROACH_CAPTURE_LAG_FACTOR: float = 2.0
    # ...but not for the S-turn, which is a dissipator and needs its bank
    # (``guidance.approach``).  Lag-aware alone: doors 1-22 m off, 4 of 6
    # long by 1.8-6.7 km (save-lag-1003, save-rmin-1003).
    APPROACH_SCURVE_FULL_GAIN: bool = True  # default 2026-10-03: save-fullgain-1003, save-energy-1003
    # **1.0 -- inert, and the story of why is worth more than the knob.**
    # ``logs/LOG912`` is a flight that did everything right: touchdown at
    # 1.35 m/s of sink, on the runway's length, from a gate handover 7 m off
    # the centreline.  It came to rest **117 m** to the side, off the 70 m
    # strip, and broke up on the grass.
    #
    # The cross-track reads +7 m at the gate, -56 at the flare and +117 at
    # rest, which looks like a capture oscillating about a centreline it has
    # already found -- so this was set to 0.5 to make it undershoot instead.
    # The first ROLLOUT tick reads **+110.7 m**: the swing happened during
    # the *flare*, not the rollout and not the capture.  ``run_flare``
    # commands bank 0, deliberately, so for its ten seconds the vehicle flies
    # straight and converts whatever lateral rate it inherited into drift --
    # 16 m/s for 10 s is 166 m.
    #
    # So the capture is not overshooting, it is being interrupted, and a
    # gentler one would hand over *more* rate rather than less.  What the
    # approach needs is a rate target that also respects the time left before
    # the flare (``|cross| / time to touchdown`` is 5.6 m/s where the
    # arrest-in-the-offset rule permits 30), and that is the open item.
    # **Not 1.0, because the bank limit is not available instantly.**  The
    # stoppable closing rate is ``sqrt(2 a s)`` with ``a`` from the bank
    # limit, and at full margin that is 36 m/s of lateral closure a hundred
    # metres out -- three seconds of travel against two or three seconds of
    # roll reversal.  Flown, the capture went -106 m, -75, -24, +39, +84 in
    # six seconds with the bank pinned at its limit throughout, and the
    # flare then froze it wings-level at +101.  The margin is what accounts
    # for the roll the law's own algebra assumes away.
    APPROACH_CAPTURE_MARGIN: float = 0.15  # default 2026-10-03: the landing stack, rot-orbit2-1003
    # Past the threshold.  The offline landing touches down about 340 m short
    # of wherever this points, so 600 puts the wheels ~260 m in -- clear of
    # the threshold with 2.1 km of tarmac left, which at 53 m/s needs
    # 0.66 m/s^2 of deceleration against about 1 from gear-down drag alone.
    # Nearer the threshold now that the arrival is faster: every metre of aim
    # past it is a metre of rollout given away, and the flare no longer needs
    # the float it was allowing for.
    # **Tried at -1500, to bias out a measured 2-3 km overshoot, and the
    # offline suite refused it**: the aim is not a free bias, it is the
    # point the gate's geometry is checked against
    # (``test_the_gate_sits_on_a_path_the_vehicle_can_fly``) and the point
    # the approach's S-turn measures its surplus from, so moving it 1.7 km
    # moves four constraints at once.  The overshoot is real -- the
    # rollout-to-wheels ratio is 2.4-2.5 where ``APPROACH_BEST_LD`` says
    # 2.08, and part of it is not a ratio at all but the flare's roughly
    # fixed float -- and the honest fix is to model those two separately
    # rather than to aim at the grass.
    # **400 was flown and crashed 3 of 4 -- back to 2400 (2026-09-25).**
    # The user's rule is right (aim at the near end so the rollout has the
    # room) but it cannot be flown yet: the cone hands the approach
    # 1300-1700 m of surplus height (laps=0 on every flight), and a nearer
    # aim leaves less final to spend it in, so the approach dives -- flare
    # at 70 m/s of sink from 220 m (LOG3665, 3668) against 20-35 at 2400;
    # 4/4 landed at 2400 (LOG3663, 3666, 3667, 3670), 1/4 at 400 (LOG3664,
    # 3665, 3668, 3669).  Failure 68's mechanism, not cured by the
    # time-based weave stop or ``GATE_FROM_APPROACH``.  **Make the cone
    # spend the surplus first (HANDOFF, "SECOND THING"), then move this.**
    TOUCHDOWN_AIM_M: float = 2400.0
    # **The aim derived** (``airframe.touchdown_aim``): the touchdown zone
    # (this fraction of ``RUNWAY_LENGTH_M`` -- the user's rule, aim at the
    # near end so the rollout has the room) less the flare's float at best
    # glide.  The cone now exits within a few hundred metres (rot-chain3),
    # which is the condition the comment above set for moving it.  Off.
    TOUCHDOWN_AIM_DERIVED: bool = False
    # **The approach's aim nearer, the cone's left alone** (metres; 0 =
    # off).  Flown 2026-10-05 on the rigoff cone saves with
    # FLARE_ALIGN_ALT_M=30: every save that missed missed *long* -- they
    # fly the profile to the aim, which is the far threshold, and the flare
    # floats 300-500 m past it (sav-sstop-1005).  ``TOUCHDOWN_AIM_M`` 1800
    # put hac0/1/2 9/9 on the runway but moved the cone, which ran out of
    # height mid-turn on hac5 (sav-aim-1005).  This moves only
    # ``guidance.approach``'s aim -- its profile and its S-turn stop.
    # **1000, default 2026-10-05** (with the S-turn stop measured from the
    # unshifted aim): on the rigoff cone saves, on the runway intact 13/15
    # against 5/15, interleaved (sav-shift3-1005, LOG6244-6279).
    APPROACH_AIM_SHIFT_M: float = 1000.0
    TOUCHDOWN_ZONE_FRACTION: float = 0.25
    FLARE_RAMP_S: float = 0.4

    # -- the propagator ----------------------------------------------------
    PREDICT_DT_ATMO: float = 0.5
    PREDICT_DT_UPPER: float = 2.0           # above PREDICT_UPPER_ALT_M
    PREDICT_UPPER_ALT_M: float = 32000.0
    PREDICT_DT_VACUUM: float = 5.0
    PREDICT_MAX_TIME_S: float = 3000.0
    PREDICT_RK4: bool = True
    DEORBIT_EXIT_TICKS: int = 1
    # The burn is tapered against a measured sensitivity rather than stopped
    # on a threshold: at ~25 km of range per m/s, a debounce of a few ticks at
    # full throttle is tens of m/s of overburn and hundreds of kilometres on
    # the ground.  ``DEORBIT_PROBE_DV`` is the second propagation's offset,
    # ``DEORBIT_TAPER_S`` the horizon the remaining dv is spread over, and
    # ``DEORBIT_MIN_GRADIENT`` the metres per m/s below which the two probes
    # are not far enough apart to divide by.
    DEORBIT_PROBE_DV: float = 2.0
    DEORBIT_TAPER_S: float = 1.5
    DEORBIT_MIN_GRADIENT: float = 200.0
    DEORBIT_MIN_THROTTLE: float = 0.02
    # **Charge the floor rule one tick, not the dead time.**  See the floor
    # test in ``Autopilot.run_deorbit``: its "what one more floored tick
    # spends" used the time since the engine last burned, which includes
    # alignment dead time -- on the shuttle's 65 m/s^2 that made a tick
    # "cost" 13 m/s and every burn quit 1.9 m/s short.
    # **Default since 2026-09-23**, flown and cleared on both craft: on
    # the shuttle the four together take the cone arrival from +111 km sd 9
    # to +3.2 sd 1.0 (n=8) and +3.4 on a second orbit; on the old craft,
    # `pairfly-oldcraft-fixes.txt` (+477 vs +426) and
    # `pairfly-oldcraft-ratchet.txt` (-558 vs -346) show nothing either way.
    DEORBIT_FLOOR_ON_TICK: bool = True
    # **The engine's thrust limiter, set so the burn is never shorter than
    # this at full throttle.**  ``Autopilot.limit_burn_thrust``.  At 3 s it
    # brings the shuttle's Rhino from 65 m/s^2 to about 9.  **Not inert on
    # the old craft**, whatever an older comment says about 8.6 m/s^2: it
    # reads 17.8 at the burn and gets a 0.60 limit (LOG2925-2926), with the
    # arrival unchanged.  0 = never touch the limiter.
    DEORBIT_MIN_BURN_S: float = 3.0
    # **Do not learn an alpha ceiling from a bank reversal.**  While the
    # lean is between the stops, and for one pitch ``time_to_peak`` after,
    # ``Holdable`` is not fed.  On the shuttle a reversal at 7 kPa dipped
    # the alpha 31 -> 8 for three seconds, filled a whole q bin, and taught
    # the predictor a ceiling of 10.6 that cost 35-48 km (LOG2912-2914).
    # The settle time is the controller's own, so it scales per airframe.
    HOLDABLE_SKIP_REVERSAL: bool = True
    # **And do not back the alpha ceiling off during one either.**  The
    # control loop's ratchet (``Autopilot.ratchet_alpha``) is separate from
    # ``Holdable`` and was not gated: one reversal at 3.4 kPa took it from
    # 30 to 20 deg, recovery is 0.5 deg/s, and the solve pinned at bank 70
    # could not recover the +22 km (LOG2918).  Needs HOLDABLE_SKIP_REVERSAL,
    # which marks the window.
    RATCHET_SKIP_REVERSAL: bool = True
    GLIDE_TICK_S: float = 1.0

    # -- the loop ----------------------------------------------------------
    TICK_S: float = 0.1
    ORBIT_TICK_S: float = 2.0               # nothing happens fast up there
    LOOP_PACING_GAME_TIME: bool = False     # pace on ut, not wall clock.  Only
                                            # matters when the game is running
                                            # off 1x -- see
                                            # testInstances/timescaleSrc
    LOOP_PACING_MAX_STRETCH: float = 5.0    # give up on a tick after this many
                                            # intervals of wall clock: a paused
                                            # game must not hang the autopilot
    # **The control interval is the quantity that must be constant, not the
    # time scale.**  The farm runs the game off 1x for throughput and the
    # controller used to pay for it: at 6x this loop delivers a command every
    # ~2 game-seconds where the approach wants one every 0.1, so the landing
    # chain was fitted to a vehicle nobody flies -- same configuration, flare
    # entered at 91-102 m/s with the coarse loop and 52-79 with a fine one,
    # rolling out three kilometres further (failure 63).
    #
    # Turning the farm down to 1x fixes that and costs the throughput the farm
    # exists for.  Neither is necessary: ask for a fixed interval in *game*
    # seconds, measure what one tick costs in wall seconds, and the fastest
    # honest scale is the ratio -- tens of x in orbit, one or two on final,
    # decided per phase by the loop itself.  ``common.pacing.ScaleGovernor``
    # writes it to the plugin's control file; the path is the instance's
    # ``timescale.txt`` and the harness passes it (``quickglide.py``).
    #
    # Empty is off, which is what a flight outside the farm gets.
    TIMESCALE_GOVERNOR: str = ""
    TIMESCALE_GOVERNOR_MAX: float = 8.0     # never ask for more than this
    TIMESCALE_GOVERNOR_MIN: float = 1.0     # never slow the game below real time
    # The ceiling is computed from a mean tick cost and what binds is the tail:
    # a tick twice as expensive as the mean must still fit inside the interval.
    TIMESCALE_GOVERNOR_MARGIN: float = 0.7
    # The plugin's frame quantum as a fraction of the phase's control
    # interval (never below 0.05 s): the glide's 1 s tick gets 0.2 s per
    # frame, the 0.1 s phases keep 0.05.  0 is a fixed 0.05 s.
    # ``ScaleGovernor.quant_fraction``.
    TIMESCALE_QUANT_FRACTION: float = 0.2
    # **Read-only kRPC calls asked together go in one round trip**
    # (``common.krpcbatch``): an aero-table row is fourteen
    # ``SimulateAerodynamicForceAt`` calls, and one at a time they cost 10-45
    # ms of a loaded farm's frames against 6-7 batched -- the same numbers to
    # the bit (measured on ksp0, CPython and PyPy).  The row refresh was every
    # HAC tick's peak, and the governor serves the peak.  False: one at a time.
    RPC_BATCH: bool = True
    # How often to count the parts still attached.  Polled, not streamed:
    # ``parts.all`` is a list transfer and this loop does not carry those.
    # Seconds is plenty -- a breakup takes seconds, and what the log needs is
    # that it happened and roughly when.
    # **How hard kRPC's attitude controller is allowed to chase.**  0 leaves
    # its default alone, which is `time_to_peak = 1.0 s` with 1% overshoot,
    # auto-tuned from the vessel's torque.  That is a one-second controller
    # acting on an airframe whose lateral mode is measured at **2.4 s**
    # (failure 12), with control authority that varies by orders of magnitude
    # across the entry -- the shape that drives a pilot-induced oscillation
    # rather than damping one.  Raising it makes the controller gentler than
    # the mode instead of faster than it.
    #
    # **Flown, adopted, and it is the largest single change this airframe
    # has had.**  Failure 12 measured what it does to the attitude -- sideslip
    # median 7.3 -> 2.5 deg, angle-of-attack tracking error 2.8 -> 0.6 -- and
    # then declined to adopt it, because the three flights that used it went
    # 52-84 km *long*: the aim and the learned ceiling downstream of it were
    # calibrated against a vehicle that under-delivers, and a plant
    # improvement is not a system improvement while the calibration still
    # describes the old plant.  That re-fit is done (see
    # ``DEORBIT_LONG_BIAS_FRACTION``), and what it uncovered is that the
    # tuning was not buying attitude quality, it was buying *the landing*:
    #
    #   | at the gate            | 1.0 s (kRPC default) | 3.0 s |
    #   |------------------------|----------------------|-------|
    #   | speed                  | 96.7 m/s             | 63 m/s|
    #   | flight path angle      | **-82 deg**          | -33   |
    #
    # At the default tuning the vehicle does not arrive at the gate, it
    # arrives *above* it, nose down, having converted its whole surplus into
    # speed; commanded 24.6 deg of angle of attack in the last 70 m it
    # delivered 5.3, and every flight of the old default ended as a wreck
    # reporting 0 parts and 0.00 t.  The oscillation was not a ride-quality
    # problem.  It was the reason the airframe could not be pointed.
    ATTITUDE_TIME_TO_PEAK_S: float = 3.0
    # **And a constant in seconds cannot be general.**  See
    # ``autopilot.attitude_time_to_peak``.  3.0 damped one vehicle's 2.4 s
    # lateral mode; a second airframe on the same farm needs something else
    # entirely, and between them that constant is worth **198 km** of
    # arrival.  What transfers is not the second, it is the ratio to the
    # vehicle's own slew time ``sqrt(I / tau)`` -- which kRPC reports in
    # full and which this autopilot already logs half of in STANDBY.
    #
    #     old craft  14.5 t   I=(34931, 13452, 37195)     -> 1.57 s
    #     shuttle    37.5 t   I=(2068357, 94184, 2106086) -> 11.85 s
    #
    # **59x the pitch inertia for 2.6x the mass, on the identical 15 kN m of
    # reaction wheel** -- so flying the shuttle at 3.0 s is flying it at a
    # fifth of its own slew time, and measured it holds 16 degrees more
    # alpha than commanded with 95 degrees of sideslip swing at Mach 7.
    #
    # Off until flown.  On, the constant above is only the fallback for a
    # vehicle kRPC cannot answer for, and the log says which was used.
    # **Default since 2026-09-23.**  Null on the old craft, as built to be
    # (`pairfly-oldcraft-derived.txt`, qs_plane_inc, n=6 an arm: arrival
    # -686 vs -697, alpha sd 2.7 both, slip p-p 9.3 vs 8.9), and the
    # precondition for every shuttle result -- at the constant 3.0 s that
    # airframe could not be pointed (+16 deg of alpha error, 95 deg slip).
    ATTITUDE_TIME_TO_PEAK_DERIVED: bool = True
    # The ratio, and it is **read off the craft the constant was fitted to**
    # rather than fitted again here: 3.0 / 1.57 = 1.91, so on that airframe
    # the derived law reproduces the committed behaviour by construction.
    # That agreement is the reason to believe it and equally the warning --
    # a derivation that reproduces a fit says the fit was right for *that*
    # aircraft and nothing yet about the next.  It predicts 22.6 s for the
    # shuttle, against a flown bracket where 3.0 could not point the vehicle
    # and 6, 9 and 12 all could; the top of that range is untested.
    ATTITUDE_SLEW_FACTOR: float = 1.91
    # **Hand ``time_to_peak`` over in kRPC's order, (pitch, roll, yaw).**
    # The derivation above handed it (pitch, yaw, roll), so the shuttle has
    # flown roll on 22.6 s and yaw on 4.8 since the derived tune became the
    # default -- measured by the autotuned gains and a timed roll in orbit.
    # See ``autopilot.krpc_axes``.  Off reproduces the old flights exactly.
    # **Default since 2026-09-25**, with ``_FROM_CONE``: the user flew
    # defaults live twice (LOG3644-3645) and the shuttle would not roll --
    # +40 commanded through all of APPROACH, -15..+9 flown, 9 km off the
    # centreline.  Flown before in the chain (game_v2, n=6: HAC +45 -> +52).
    ATTITUDE_AXES_KRPC_ORDER: bool = True
    # **Roll on full authority** (the user, 2026-09-25): kRPC's roll
    # ``time_to_peak`` in seconds, replacing the derived 4.8 s (1.91 x the
    # wheels-only slew time -- a figure that never counted the surfaces).
    # 1.0 is kRPC's own default: gains high enough that any real bank error
    # drives the roll input to its stop.  The bank *command* is still
    # slewed at the measured rate (``BANK_RATE_MEASURED``), which is what
    # keeps a fast roll at high alpha from turning alpha into sideslip
    # (LOG3680).  0 leaves the derived figure.
    # **Back to 0 (derived, 4.8 s on the shuttle) on 2026-09-26.**  Flown at
    # 1.0 the shuttle lost control in the entry every time: the user's live
    # LOG3692 and 4/4 on the farm (LOG3693/3696/3699/3702, qs_shuttle, with
    # and without the damper) -- bank error 125-179 deg, sideslip 41-64.
    # With yaw on roll's figure (``ATTITUDE_YAW_WITH_ROLL``) 1.0 survived
    # the entry, but at glide sideslip 31-44 deg against 8-14 at 4.8
    # (LOG3705-3716).  Full authority is more than this airframe can use at
    # 35 deg of alpha; the derived figure (1.91 x its own sqrt(I/tau)) is
    # the general law.
    ATTITUDE_ROLL_TIME_TO_PEAK_S: float = 0.0
    # **kRPC drops the roll target outright while the nose is off its own.**
    # The server's attitude controller blends roll in only below
    # ``roll_start_angle`` (default 20 deg of *direction* error) and fully
    # below ``roll_engage_angle`` (15); above that it sends **zero** roll and
    # swings the nose with pitch and yaw alone.  On a winged vehicle the nose
    # sits at alpha from the velocity, so a bank change *is* a direction
    # change of ~2 alpha sin(dbank/2): at the shuttle's 36 deg of entry alpha
    # any reversal past ~33 deg of bank starts outside the gate, and the
    # vehicle was yawed round the velocity cone on its weakest axis -- the
    # user watched the GUI's roll indicator sit at zero (LOG3679: +33 -> -1
    # commanded, +52 flown, 23 deg of sideslip).  Bank is the control; roll
    # must never be gated on pointing.  This is the error, in degrees, below
    # which roll is fully engaged (``roll_start_angle`` is set 5 above it,
    # capped at 180).  0 leaves kRPC's 15/20.
    ATTITUDE_ROLL_ENGAGE_DEG: float = 175.0
    # **Yaw on the authority that is actually acting** -- the thrusters.
    # The static yaw figure (22.6 s on the shuttle) is derived from the
    # reaction wheels alone: 15 kN m.  The shuttle's RCS makes **290 kN m
    # of yaw** (STANDBY's ``torque available_rcs_torque`` line), and
    # ``GLIDE_RCS`` has the valve open through every hypersonic reversal
    # (LOG3758-3760: on within a tick of the reversal starting) -- but kRPC
    # schedules the nose's target rate from ``time_to_peak``, so a 22.6 s
    # yaw asks the thrusters for a wheel's worth of yaw and the body roll
    # (4.8 s) outruns it into 12-31 deg of sideslip.  With this on, while
    # the valve is open in GLIDE yaw's figure is ``ATTITUDE_SLEW_FACTOR *
    # sqrt(I_yaw / (wheel + rcs))`` (5.0 s on the shuttle), never faster
    # than roll's, and it goes back to the static figure the moment the
    # valve shuts -- so below ``GLIDE_RCS_MACH_MIN``, where yaw on roll's
    # figure lost the bank in the cone (``ATTITUDE_YAW_WITH_ROLL``), nothing
    # changes.  Derived from what kRPC reports, so a craft with no RCS gets
    # nothing.  And because the tune now follows the valve, the valve is
    # held open while the bank command leads the flown bank by more than
    # ``BANK_RATE_SAT_DEG``: a relay on pointing error alone would shut
    # mid-reversal and hand the turn back to the wheels' figure.
    # **Default 2026-09-29 (night)**, with the HAC handover fixed (the valve
    # stays open into the cone until its relay settles): pairfly qs_shuttle
    # 6 v 6 (`23e33e7b`, LOG3846-3862), hypersonic peak slip 8-13 deg on all
    # six against 15-70 on the defaults, which departed once (LOG3850, 70
    # deg, 54 km short -- the user's live spin); no flag flight departed and
    # two landed 30/30.  Transonic bank overshoot (cmd 37, flown 80-106) is
    # present on both arms at similar size -- a separate roll problem.
    ATTITUDE_YAW_WITH_RCS: bool = True
    # **Slow the lateral axes when the bank diverges, from the vehicle's
    # own swings.**  ``rollrate.RollDamper``.  Full authority (roll 1.0)
    # tracked the shuttle to a degree up to q ~700 Pa and then swung about
    # a steady +30 command with growing amplitude -- 8, 12, 14, 16, 22, 29,
    # 35 deg past it -- until the nose was lost (the user, live, LOG3692);
    # the roll figure has been 3.0, 22.6, 4.8 and 1.0 on this one airframe,
    # so no constant is the answer.  A half-swing is the flown bank's
    # excursion past a command that did not move, beyond
    # ``BANK_RATE_SAT_DEG``; when one peaks more than ``ROLL_DAMPER_GROWTH``
    # times the one before (the loop is diverging), the roll
    # ``time_to_peak`` -- and yaw's, under ``ATTITUDE_YAW_WITH_ROLL`` -- is
    # multiplied by ``ROLL_DAMPER_STEP`` (capped at the vehicle's slowest
    # static axis); while none grows it relaxes back toward the floor over
    # ``ROLL_DAMPER_RECOVER_S``.  **Growth only**: the first version counted
    # every crossing and ran to its 22 s ceiling on the shuttle's +-7 deg
    # lateral wobble, whose amplitude did not change as the tune went
    # 4.8 -> 20 s (LOG3710) -- an airframe mode, not a loop one, which a
    # slower loop only makes lag.  Logged as ``roll damper:`` lines.
    ROLL_DAMPER: bool = True
    ROLL_DAMPER_GROWTH: float = 1.1
    ROLL_DAMPER_STEP: float = 1.5
    ROLL_DAMPER_RECOVER_S: float = 120.0
    # **Pitch only, from wheels plus smoothed surface torque, floored at
    # ``ATTITUDE_TIME_TO_PEAK_S``.**  See ``Autopilot.retune_pitch_air``:
    # the three faults of the live version above, each answered.  The
    # shuttle's static pitch tune is 19 s against 6000+ kN m of surface in
    # the cone; it dives in from ``qs_shuttle_cone`` with the pitch input at
    # +0.08 and the nose 15 deg under its command (LOG3035).  Inert on the
    # old craft by construction (its static pitch *is* the floor).
    ATTITUDE_PITCH_AIR: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    ATTITUDE_AIR_SMOOTH_S: float = 5.0      # game-seconds, the surface EMA
    # The floor under ``ATTITUDE_PITCH_AIR``'s pitch time_to_peak, seconds
    # (0 = ``ATTITUDE_TIME_TO_PEAK_S``).  See ``Autopilot.retune_pitch_air``.
    ATTITUDE_PITCH_AIR_FLOOR_S: float = 0.0
    ATTITUDE_RETUNE_S: float = 1.0          # game-seconds between checks
    ATTITUDE_RETUNE_FRAC: float = 0.15      # re-assign only past this change
    # **And it is handed back for the landing.**  3.0 s is right for the
    # entry, where the controller is acting through a 2.4 s lateral mode and
    # a faster one drives it; on final at Mach 0.2 it is simply a slow
    # aircraft.  Measured in the flare: commanded 17.3 deg and achieving
    # 11.5, then commanded 24.6 and achieving 13.7 two seconds later -- about
    # **1.1 deg/s**, so reaching flare attitude from the approach trim takes
    # some twelve seconds, which at 31 m/s of sink is 370 m of height that
    # is not there.  The flare arrested 31.4 m/s to 19.8 and hit the water.
    #
    # 0 leaves whatever ``ATTITUDE_TIME_TO_PEAK_S`` set; a number re-tunes on
    # entering APPROACH, by which point the Dutch roll the slow tune exists
    # for is long behind the vehicle.
    # **Flown at 1.0 and put back to 0.** Handing the controller back to
    # kRPC's default on final does exactly what failure 12 says it does, one
    # phase later: measured on the approach, the achieved angle of attack
    # oscillates +-8 degrees about a command moving smoothly from 18.9 to
    # 12.1 (15.4, 6.9, 17.1, 13.5, 26.4, 21.1, 13.4, 19.8, 9.5, 27.0) where
    # the same airframe tracks its command to a degree in the glide at 3.0 s.
    #
    # It was introduced because the flare was pitching up at about 1 deg/s
    # and looked controller-limited.  It was not: it was clamped at 20.5
    # degrees by the learned alpha ceiling for its whole length (see
    # ``RELEASE_CEILING_ON_FINAL``), which is the real fix and makes this one
    # unnecessary.  0 leaves ``ATTITUDE_TIME_TO_PEAK_S`` alone.
    ATTITUDE_TIME_TO_PEAK_FINAL_S: float = 0.0
    # **The learned alpha ceiling is handed back at the same moment, and for
    # the same reason.**  ``trajectory.Holdable`` and ``ratchet_alpha`` learn
    # what the airframe can hold *against dynamic pressure in the entry* --
    # 20 kPa at Mach 5, where the control surfaces are fighting a trim the
    # propagator cannot see.  On final at Mach 0.2 and 2 kPa none of that
    # applies, and the ceiling is simply a number left over from a different
    # flight regime.
    #
    # Measured, in a flight that touched down **on the runway, 13 m off the
    # centreline** (``logs/LOG803``): the flare asked for the load it needed
    # and was clamped at 20.5 degrees for its whole length -- commanded 20.5
    # at 67 m and still 20.5 at 4 m -- because that is where the ratchet had
    # left the ceiling at 20 kPa. It arrested 40.4 m/s of sink to 12.65 and
    # broke the vehicle up on the tarmac.
    #
    # This is the argument ``alpha_limit_for_speed`` already makes one line
    # at a time: "APPROACH and FLARE are deliberately outside it, because the
    # flare has to be able to ask for maximum lift at a speed this cap would
    # refuse."  The ceiling is the same kind of cap and the exemption was
    # never extended to it.
    RELEASE_CEILING_ON_FINAL: bool = True
    PART_COUNT_INTERVAL_UT: float = 5.0
    LOG_INTERVAL_UT: float = 2.0
    LOG_DIR: str = "logs"
    # **Where a miss is born, as opposed to where it lands.**  Two log lines
    # per flight: what the committed burn predicts for the entry interface,
    # and what the vehicle actually brings there.  Their difference is the
    # handover error.
    #
    # It exists because the two numbers already in every log disagree and
    # nothing could say why.  The burn exits reporting ``range error +0 m``
    # on every flight; the same propagator, re-run from the vehicle's own
    # state 22 km lower, reports +7 km on ``qs_plane``, **-47 km** on
    # ``qs_plane_high`` and **-74 km** on ``qs_plane_inc``.  Offline that
    # propagator is self-consistent across the same split to **0.0 km** on
    # four states, so it is neither integration error nor the coast.
    #
    # Off by default: it costs one extra entry propagation at commit, which
    # is affordable but not free, and nothing steers on either line.
    DIAG_INTERFACE: bool = False

    # -- the vehicle -------------------------------------------------------
    STARTUP_ACTION_GROUP: int = 0           # 0 = none; this craft needs none
    ENABLE_RCS: bool = True
    # RCS through the glide as well -- off, for the reason ``RCS_Q_MAX_PA``
    # gives.  Raising one without the other changes nothing: the ceiling
    # shuts the valve at 500 Pa whatever the phase permits.
    GLIDE_RCS: bool = True  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **Where the glide's RCS is for: the supersonic reversals.**  Flown in
    # the sim with the q ceiling lifted (LOG3463-3466), the valve took the
    # shuttle's arrival from +1.3 to +0.3 km and its mistracked ticks below
    # Mach 4 from ~80% to ~31% -- and then emptied the tank, 230 of 429
    # units subsonic in the cone, because nothing after GLIDE ever closed it.
    # Below this Mach the glide does not permit it (0: everywhere); the later
    # phases now close the valve themselves.
    GLIDE_RCS_MACH_MIN: float = 1.0  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # **Permission is not demand.**  ``COAST_RCS`` and the burn say where RCS
    # is *allowed*; what actually opens the valve is a pointing error that
    # nothing else is closing.  kRPC's autopilot hunts, and a thruster held
    # on through the hunt pays for every oscillation -- 55 of 150 units in
    # two minutes of holding prograde.  So the valve is a relay on the angle
    # between the commanded nose and the real one: on above ``ON``, off below
    # ``OFF`` once it has stayed there for ``SETTLE_S``.  The gap between the
    # two thresholds is what stops the relay chattering, which is the same
    # shape of mistake as the bank reversals.
    RCS_ERROR_ON_DEG: float = 5.0
    RCS_ERROR_OFF_DEG: float = 1.5
    RCS_SETTLE_S: float = 2.0
    # And not against the air, in the entry this vehicle flies today.
    #
    # **The reason is not the one it looks like.**  "Four blocks cannot matter"
    # was the assumption, and it is wrong by measurement: this craft's RCS
    # reports **37.5 kN m** of pitch/yaw torque against the reaction wheels'
    # 15 (LOG1615, ``dump_torque``).  What keeps the ceiling here is cheaper
    # and duller -- 150 units of monopropellant does not hold an attitude for
    # a three-minute entry, the glide's alpha shortfall is a saturation the
    # thrusters would be fighting continuously rather than a slew they could
    # finish, and nothing downstream of COAST permits RCS at all, so this
    # number has never had to be right.
    #
    # It is also the knob the high-alpha experiment has to move: the measured
    # holdability curve is a *wheels-only* curve because this shuts the valve
    # at 500 Pa.  See docs/spaceplane/design.md, "High alpha: what the airframe
    # gives and what it will hold".
    RCS_Q_MAX_PA: float = 20000.0  # default 2026-09-25: the shuttle chain, 4/4 landed (LOG3656-3661) vs 0/4
    # No knob for the deorbit flip any more.  It was flown on reaction wheels
    # to save monopropellant, on the argument that nothing was waiting on it;
    # the measurement says a deorbit window is (114-169 s of flip, and nine
    # arrivals 50 km further short).  Failure 43.
    WHEEL_CLEARANCE_M: float = 1.5          # centre of mass to the tyres
    WHEEL_CLEARANCE_MAX_M: float = 12.0     # a box bigger than this is broken
    # The same check, for the other axis of the same box.  It had none, and
    # the box answered 24.95 m of tail on a 5.65 m aircraft -- see
    # ``Telemetry._refresh_wheel_clearance``.  Generous, because it is a
    # sanity bound and not a model of this airframe: anything under it is
    # believed, anything over it falls back.
    TAIL_EXTENT_MAX_M: float = 20.0
    # How much longer than the furthest-aft *part* the bounding box may claim
    # to be before it is not describing this vehicle.  The box is measured
    # from the centre of mass and so are the parts, so they should agree
    # closely; 2x is slack, not a tolerance.  Flown, the box said 24.95 m
    # against the parts' ~3.
    TAIL_EXTENT_SLACK: float = 2.0
    # Off restores the unchecked box.  **Rare, and not the farm's breakup
    # mechanism** -- scanned over every log on disk, the bad reading appears
    # exactly once (``logs/LOG2384``, flown by hand) against 18.7 or 19.9 deg
    # on every farm flight, so this is a guard against a hazard that has
    # fired once, not an explanation of anything.  Worth having because when
    # it does fire nothing says so and the flare silently loses three
    # quarters of its authority for the whole landing.
    TAIL_EXTENT_CHECK: bool = True
    TERRAIN_MAX_M: float = 7000.0
    TERRAIN_BELOW_SEA_M: float = 200.0
    RUNWAY_ALT_OFFSET_M: float = 0.0


def apply_overrides(cfg, overrides):
    """``--set FIELD=VALUE``, with type coercion.  Unknown fields raise."""
    kinds = {f.name: f.type for f in fields(cfg)}
    for item in overrides:
        name, _, raw = item.partition("=")
        name = name.strip()
        if name not in kinds:
            raise SystemExit("unknown config field %r" % name)
        current = getattr(cfg, name)
        if isinstance(current, bool):
            value = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(current, int) and not isinstance(current, bool):
            value = int(raw)
        elif isinstance(current, tuple):
            value = tuple(float(x) for x in raw.split(","))
        elif isinstance(current, str):
            value = raw
        else:
            value = float(raw)
        setattr(cfg, name, value)
    return cfg


def defaults_fingerprint():
    """A short hash of every default in this ``Config``.

    **The config line is a list of differences, which makes it silent about
    the thing the differences are measured against.**  Change a default
    between two batches and the logs of both say the same thing while the
    flights differ -- measured, and it cost a session's analysis: eighteen
    flights flown with ``GLIDE_RESERVE_ON`` defaulted to ``True`` were
    indistinguishable in the log from the controls they were being compared
    with, and the comparison drawn from them was backwards.

    So the line carries the fingerprint of the whole default set as well.
    Two logs with different fingerprints are not the same experiment, however
    identical their difference lists look.
    """
    base = Config()
    text = ";".join("%s=%r" % (f.name, getattr(base, f.name))
                    for f in sorted(fields(base), key=lambda f: f.name))
    return hashlib.sha1(text.encode()).hexdigest()[:8]


def differences(cfg):
    """The fields that differ from the defaults, for the log's config line."""
    base = Config()
    out = []
    for f in fields(cfg):
        mine, theirs = getattr(cfg, f.name), getattr(base, f.name)
        if mine != theirs:
            out.append("%s=%s" % (f.name, mine))
    return out
