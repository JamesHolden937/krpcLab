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
    GATE_ALT_M: float = 2000.0
    GATE_DIST_M: float = 4000.0             # before the threshold
    GATE_CAPTURE_M: float = 1500.0          # hand over to APPROACH within this

    # -- the heading alignment cone ----------------------------------------
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
    # Manual pitch input per degree of pitch pointing error in the FLARE,
    # summed with kRPC's (0 = off).  See ``Autopilot.flare_pitch_p``.
    FLARE_PITCH_P: float = 0.08  # default 2026-10-03: the landing stack, rot-orbit2-1003
    FLARE_PITCH_P_MAX: float = 0.5
    ALPHA_TRIM_MIN_DEG: float = -2.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
    ALPHA_TRIM_MAX_DEG: float = 4.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
    ALPHA_TRIM_ROLL_TOL_DEG: float = 10.0
    ALPHA_TRIM_SLIP_TOL_DEG: float = 15.0  # default 2026-10-03: the landing stack, rot-orbit2-1003
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

    HAC_EXIT_SURPLUS_M: float = 500.0
    # **Hand over only what is nearer the approach than a lap would leave
    # it** (``Autopilot.hac_exit_surplus``).  The derived allowance is a
    # whole lap's height (2 pi R / cone ratio at the circle the airframe
    # holds now): any surplus short of a lap was handed to the approach,
    # whose own capacity is the ~500 m above.  On the shuttle a lap there
    # prices at 9-14 km, so rigoff cones rolled out 6-8 km high with the
    # plan still reading laps=1 and fitting (LOG6826 h 9784 needing 2101,
    # 6850, 6855); the approach dived at alpha 20-25 to 50 m/s and all
    # three flared at 39-41 m/s of sink and broke.  A lap leaves the
    # surplus minus the lap; not lapping leaves the surplus, so the lap is
    # the nearer answer once the surplus passes this fraction of it --
    # 0.5 is the symmetric choice, and a short lap can still cut to the
    # gate where a high rollout cannot lose height.  1.0 is the old rule.
    HAC_EXIT_LAP_FRACTION: float = 1.0
    # **Price the exit's lap as the plan prices it.**  The exit took the
    # hold radius at the speed the gate is reached at (~140 m/s, 2.7 km)
    # where the plan flies laps at the cone's target speed (2.0 km): 9-14
    # km of height against the ~6.5 a lap at the gate costs (cone save
    # rigoff4: +4.9 km lapped to -1.6).  rot-bank-1006: 6 of the 8 rigoff
    # losses rolled out 4-8 km high and broke at the flare.  With this on
    # the exit allowance is that lap less ``HAC_EXIT_SURPLUS_M``.  Off.
    HAC_EXIT_LAP_AT_TARGET: bool = False
    # How near the gate counts as being at it.  ``GATE_CAPTURE_M`` is the
    # straight-in gate's own answer to the same question and this is
    # deliberately the same size.
    HAC_ROLLOUT_M: float = 900.0
    # -- spending a surplus the circle cannot ---------------------------
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
    HAC_LAP_AT_TARGET_SPEED: bool = True  # default 2026-10-06: rot-weave-1006, rot-aim-1006
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
    HAC_AIM_DERIVED: bool = True  # default 2026-10-06: rot-weave-1006, rot-aim-1006
    HAC_LADDER_STEP_M: float = 500.0
    # At cone entry, choose the runway end and hand whose path the budget
    # fits with the least left over, not the cheapest one
    # (``guidance.hac_choose``): an arrival lined up with a threshold has
    # nothing between the run to the gate and a whole lap.  Off.
    HAC_CHOOSE_BY_ENERGY: bool = False
    # **The cone's flap brake on surplus alone** (``hac_flap_brake``).  It
    # waited for the weave to pin at ``HAC_WEAVE_MAX_DEG``, which on the
    # shuttle it never does (~44 deg), and so never deployed in LOG3846-3875
    # while the cone exited 1-2 km above what the approach needed on most
    # flights (conesum: R saturated at 16 km, flown 20 km at L/D 1.9).  With
    # this on the brake comes out whenever the surplus over the cone's own
    # profile, less the arrest height of the sink it builds, exceeds
    # ``HAC_WEAVE_DEADBAND_M``.  Off until paired.
    HAC_FLAP_BRAKE_ON_SURPLUS: bool = False
    # Arm the measured brake from this craft's last vacuum measurement
    # when engaged in the air (``Autopilot._load_brake_cache``,
    # ``logs/brakecache/``, written by every vacuum probe).  Off: an
    # air-start save has no brake.
    AIRBRAKE_CACHE: bool = False
    # ...and **not stowed for the roll** in the cone.  ``FLAP_BRAKE_YIELDS_
    # TO_ROLL`` was written for the hypersonic glide (LOG3680, 3690: the
    # elevon brake out, roll authority gone, departure at Mach 5-6); in the
    # subsonic cone the vehicle is always adjusting its bank, and under
    # ``HAC_FLAP_BRAKE_ON_SURPLUS`` the brake came out and was stowed 0.4-0.6 s
    # later "rolling" on every flight (LOG4035, 4037) -- it never spent
    # anything.  Off until paired.
    HAC_FLAP_BRAKE_IGNORES_ROLL: bool = False
    HAC_WEAVE_DEADBAND_M: float = 800.0     # surplus worth weaving for
    # Weave only on the join leg, never on the circle (``guidance.hac``):
    # on the circle the weave fights the turn's standing bank and walks the
    # vehicle off it (rot-lapstack-1006: 7 of 12 "out of height" exits wove
    # on the circle, 1 of 12 rolled-out ones).  Off until flown.
    HAC_WEAVE_STRAIGHT_ONLY: bool = False
    # The reversal is on a clock rather than on a cross-track band, because
    # the quantity a band would watch -- the offset from the intended path --
    # is what the weave is deliberately creating.
    HAC_WEAVE_PERIOD_S: float = 24.0
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
    # The weave's first swing leans the way the vehicle is already banked.
    # Off, the phase clock starts every cone on the same swing, which on
    # qs_shuttle2 is a -45 bank whatever the glide handed over: an arrival
    # banked +50..+70 is commanded a 100-115 deg reversal at alpha ~42, Mach
    # 0.9, and departed 11 of 17 times in kspSim (2 of ~24 otherwise; the
    # farm, 3 departures, all from a positive glide bank -- rot-base-1006).
    HAC_WEAVE_FIRST_WITH_BANK: bool = True  # default 2026-10-06: rot-weave-1006, rot-aim-1006
    # Short of height (the tightest circle unaffordable), cap the cone's
    # alpha at the table's best-L/D alpha (``guidance.hac``).  Off.
    HAC_SHORT_BEST_GLIDE: bool = False
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
    SPEED_FLOOR_MACH: float = 1.2
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
    # How close to the committed lean counts as arrived.  Floored on one
    # tick's worth of slew, because a command that cannot be reached this
    # tick will still be moving next tick; the constant is the floor under
    # that, so a fast tick does not read every lean as a transit.  The
    # dangerous direction is *too small* -- that is the frozen-alpha bug
    # above wearing a different threshold -- so it is a few degrees and not
    # a few tenths.
    SOLVE_HOLD_TRANSIT_DEG: float = 5.0
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
    HOLDABLE_Q_DECADE_BINS: int = 6         # bins per decade of dynamic pressure
    HOLDABLE_SATURATED_DEG: float = 2.5     # command - achieved, to count
    HOLDABLE_MIN_SAMPLES: int = 4           # before a bin is trusted
    HOLDABLE_MIN_Q: float = 500.0           # below this the air holds nothing back
    HOLDABLE_MARGIN_DEG: float = 1.0        # believe the vehicle by this much
    # **Seed the learned alpha ceiling from this craft's earlier flights**
    # (``Holdable.set_prior``; written by ``spaceplane/tools/holdprior.py``
    # to ``logs/holdprior/``, keyed by vessel name and part count).  The
    # learner knows a ceiling only once the vehicle hits it, so a glide that
    # has not saturated yet plans 40-44 deg into air where it holds 23-28:
    # every long rigoff arrival of rot-lapstack-1006 made 20-40% less drag
    # at 38-32 km than its prediction.  No file for the craft: no prior.
    # Off until flown.
    HOLDABLE_PRIOR: bool = False

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
    FLARE_SHALLOW_APPROACH_FACTOR: float = 2.7   # x stall at the gate
    FLARE_SHALLOW_DOOR_FACTOR: float = 2.4       # x stall at the door
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
    ROLLOUT_SPOILER_DEG: float = 25.0
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
    # **Steer on where the wheels are going, not only where they are.**
    # ``ROLLOUT_STEER_GAIN`` alone is a proportional law on cross-track
    # acting on a double integrator (steer -> heading -> cross), which has
    # no damping at all: every shuttle rollout in rot-newdef-1006 weaved
    # +-25-80 m with a ~20 s period and growing amplitude (LOG6831: +22 ->
    # -41 -> +51 m, track swinging -17..+26 deg), and 5 of the 17 losses
    # touched down within 30 m of the centreline and stopped 40-55 m off
    # it.  This adds the cross-track *rate* (``v . across``) times this
    # lead time to the error the gain acts on; ~2/omega of the observed
    # weave is critical damping.  0 is the old law.  Measured from one
    # runway save x6 (sav-lead2-1006, 12 an arm): lead 0 weaved span 24-176
    # m and stopped up to 85 m off; lead 3 closed monotonically, every
    # rollout that stayed on the tarmac stopped within 7 m (spans 11-23).
    ROLLOUT_STEER_LEAD_S: float = 3.0  # default 2026-10-06: sav-lead2-1006
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
    AIRBRAKE_SINK_TRACK_M_S: float = 5.0
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
    # A rate term on the speed law, per unit of measured dv/dt over g (0 =
    # off): damps the approach's phugoid.  See ``guidance.alpha_for_speed``.
    APPROACH_SPEED_KD: float = 0.0
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
    # ``1/cos`` runs away at the vertical, so the compensation is computed on
    # a bounded bank.  Above this the vehicle is not flying an approach any
    # more and a larger load demand is not the answer.
    APPROACH_BANK_COMP_MAX_DEG: float = 60.0
    APPROACH_CAPTURE_KP: float = 2.0        # deg of bank per m/s of rate error
    APPROACH_CAPTURE_LAG_FACTOR: float = 2.0
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
    # **1800, default 2026-10-06**, now that the cone spends its surplus
    # (the three HAC flags above, same day): with them on, intact on the
    # runway 9/18 at 1800 against 3/18 at 2400 and 5/18 at 1200, over the
    # three rigoff orbits interleaved (rot-aim-1006).  At 2400 a handover
    # within 200 m of profile still touched down 1.4-2.3 km in.  At 1800
    # every handover between -200 and +630 m stopped +650..+850 along; the
    # misses are the cone's tails (out of height below -600, or 20 km
    # arrivals 1-2 km high).  kspSim cannot screen this (gap 7).
    # **A ship bias, kept on purpose for now (the user, 2026-10-06) -- to be
    # replaced.**  Fitted on the shuttle only; the old craft on the same
    # defaults lands 1/12 (rot-newdef-1006: centreline touchdowns 1.3-1.6
    # km in, then ~2 km of rollout off the end).  Together with
    # ``APPROACH_AIM_SHIFT_M`` (also fitted) it means "800 m past the
    # threshold, on the shuttle".  The general form derives it per vehicle:
    # touchdown = aim + flare float (door speed vs stall, L/D) and that plus
    # the rollout (v_td^2 / 2 a_brake) must fit the runway.
    # ``TOUCHDOWN_AIM_DERIVED`` has the float half and no rollout term, and
    # has never been flown on the farm.  spaceplane/CLAUDE.md, "Next".
    TOUCHDOWN_AIM_M: float = 1800.0
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
    # **The engine's thrust limiter, set so the burn is never shorter than
    # this at full throttle.**  ``Autopilot.limit_burn_thrust``.  At 3 s it
    # brings the shuttle's Rhino from 65 m/s^2 to about 9.  **Not inert on
    # the old craft**, whatever an older comment says about 8.6 m/s^2: it
    # reads 17.8 at the burn and gets a 0.60 limit (LOG2925-2926), with the
    # arrival unchanged.  0 = never touch the limiter.
    DEORBIT_MIN_BURN_S: float = 3.0
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
    ROLL_DAMPER_GROWTH: float = 1.1
    ROLL_DAMPER_STEP: float = 1.5
    ROLL_DAMPER_RECOVER_S: float = 120.0
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
