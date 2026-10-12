"""The monopropellant valve: a relay on the pointing error, shared by both craft.

**Permission is not demand.**  A phase knows where RCS is *allowed* -- under
thrust, through a flip, in the thin air where nothing else has authority --
and that is all it knows.  What it cannot see is that most of a permitted
phase is not turning at all: kRPC's autopilot hunts around whatever it has
been given, and a thruster held open through the hunt pays for every
oscillation.  Measured on the spaceplane, holding prograde in orbit doing
nothing: 55 of 150 units of monopropellant in two minutes, which is the whole
attitude budget for the entry.

So the phase sets ``permitted`` and this decides when the valve opens: on when
the nose is more than ``RCS_ERROR_ON_DEG`` from the command, off again once it
is inside ``RCS_ERROR_OFF_DEG`` and has stayed there for ``RCS_SETTLE_S``.

Three properties, each of which is the whole point of one of the constants:

- **It is a relay, not a threshold.**  One angle, compared twice a second,
  switches the thrusters at the frequency of the oscillation they are damping
  -- which spends propellant to make the hunt worse.  The gap between ``ON``
  and ``OFF`` is what makes that impossible, and it is the same shape as the
  bank reversal deadband on the spaceplane's glide.
- **It does not drop out mid-oscillation.**  ``SETTLE_S`` asks the error to
  *stay* small, because a swing through zero is not an arrival.
- **A missing answer is not a good one.**  No command yet, or a nose that
  cannot be read, holds the valve where it is rather than deciding it is
  settled.  This is boosterland's rule and it applies to propellant too.

**In air the trigger is saturation, not error** (``saturation`` passed to
``update``).  A pointing error is only indirect evidence that the surfaces
are out of authority: every step in the command makes one, and the surfaces
close most of them alone.  So a caller that can read the summed control
input (kRPC's read-back of pitch, yaw and roll, autopilot included) passes
its largest magnitude, and the valve opens when an axis has been pinned at
``RCS_SAT_INPUT`` for ``RCS_SAT_ON_S`` *and* the pointing error has not
closed by ``RCS_SAT_CLOSING_FRACTION`` over that time -- a saturated input
that is winning is not a turn the thrusters need to join.  It closes once
the input has stayed under ``RCS_SAT_RELEASE`` for ``RCS_SETTLE_S``, or on
the error relay's own settle.  A caller with no input to report passes
``None`` and gets the error relay.

``RCS_Q_MAX_PA`` is the other half, for a vehicle in air: above it the control
surfaces own the attitude and the valve stays shut whatever the error says.
Not because the thrusters are weak -- measured on the spaceplane, RCS is
37.5 kN m of pitch torque against 15 from the reaction wheels -- but because
what they would be fighting there is a continuous aerodynamic saturation
rather than a slew they could finish, and the tank is sized for slews.  A
caller with no air to report (the booster's snapshot has no dynamic pressure
in it) passes ``None`` and gets no ceiling test.
"""


class Valve:
    """One vehicle's RCS switch.  Owns the hysteresis state, nothing else.

    It does not touch kRPC: ``update`` returns the state wanted and calls
    ``apply`` only when that differs from what is already set, so the caller
    keeps the one place that talks to the game and the valve stays testable
    without one.
    """

    def __init__(self, cfg, log=None):
        self.cfg = cfg
        self.log = log
        self.on = None              # None = the hardware has never been set
        self.permitted = False
        self.settled_since = None   # when the error last came inside OFF
        self.sat_since = None       # (ut, error) when the input pinned
        self.released_since = None  # when the input last came off the stop

    def demand(self, ut, error_deg, q=None):
        """Is there a turn the wheels, the gimbal and the air are not closing?"""
        ceiling = getattr(self.cfg, "RCS_Q_MAX_PA", 0.0)
        if q is not None and ceiling > 0.0 and q > ceiling:
            self.settled_since = None
            return False
        if error_deg is None or error_deg < 0.0:
            return bool(self.on)
        if error_deg > self.cfg.RCS_ERROR_ON_DEG:
            self.settled_since = None
            return True
        if error_deg > self.cfg.RCS_ERROR_OFF_DEG:
            # Inside the deadband: neither edge of the relay has been crossed,
            # so whatever it is doing now is what it goes on doing.
            self.settled_since = None
            return bool(self.on)
        if self.settled_since is None:
            self.settled_since = ut
        if ut - self.settled_since < self.cfg.RCS_SETTLE_S:
            return bool(self.on)
        return False

    def demand_saturated(self, ut, saturation, error_deg, q=None):
        """Is an axis pinned, and is the turn not closing without help?"""
        cfg = self.cfg
        ceiling = getattr(cfg, "RCS_Q_MAX_PA", 0.0)
        if q is not None and ceiling > 0.0 and q > ceiling:
            self.sat_since = self.released_since = None
            self.settled_since = None
            return False
        if error_deg is None or error_deg < 0.0:
            return bool(self.on)
        pinned = saturation >= cfg.RCS_SAT_INPUT
        if pinned:
            self.released_since = None
            if self.sat_since is None:
                self.sat_since = (ut, error_deg)
        else:
            self.sat_since = None
        if not self.on:
            if not pinned or error_deg <= cfg.RCS_ERROR_OFF_DEG:
                return False
            start_ut, start_err = self.sat_since
            if ut - start_ut < cfg.RCS_SAT_ON_S:
                return False
            if error_deg < (1.0 - cfg.RCS_SAT_CLOSING_FRACTION) * start_err:
                # Closing on the surfaces alone: restart the window here.
                self.sat_since = (ut, error_deg)
                return False
            self.settled_since = None
            return True
        # Open: close once the input is off the stop and stays off, or the
        # error relay settles on its own.
        if saturation < cfg.RCS_SAT_RELEASE:
            if self.released_since is None:
                self.released_since = ut
            if ut - self.released_since >= cfg.RCS_SETTLE_S:
                return False
        elif not pinned:
            self.released_since = None
        if error_deg <= cfg.RCS_ERROR_OFF_DEG:
            if self.settled_since is None:
                self.settled_since = ut
            if ut - self.settled_since >= cfg.RCS_SETTLE_S:
                return False
        else:
            self.settled_since = None
        return True

    def update(self, ut, permitted, error_deg, q=None, apply=None,
               hold=False, saturation=None):
        """Set the valve for this tick.  Returns what it is now.

        ``permitted`` false is absolute and immediate -- a phase that says no
        is not overruled by a large error -- and the whole mechanism stays
        behind ``ENABLE_RCS``, so a config that has never wanted RCS still
        never gets it.  ``hold`` is a caller that knows a turn is under way
        before the pointing error shows it (a bank reversal): the valve is
        wanted whatever the error, and the settle clock restarts, so it
        closes only ``RCS_SETTLE_S`` after the turn is over.  It does not
        override ``RCS_Q_MAX_PA``.  ``saturation`` (the largest control
        input magnitude, 0-1) switches demand from the error relay to
        ``demand_saturated``.
        """
        permitted = bool(permitted) and bool(self.cfg.ENABLE_RCS)
        self.permitted = permitted
        if saturation is None:
            wanted = permitted and self.demand(ut, error_deg, q)
        else:
            wanted = permitted and self.demand_saturated(
                ut, float(saturation), error_deg, q)
        if permitted and hold and not wanted:
            ceiling = getattr(self.cfg, "RCS_Q_MAX_PA", 0.0)
            if q is None or ceiling <= 0.0 or q <= ceiling:
                self.settled_since = None
                wanted = True
        if wanted != self.on:
            was = self.on
            self.on = wanted
            if apply is not None:
                apply(wanted)
            # Say so.  The valve is no longer a property of the phase, so a
            # log that prints only phases can no longer be read back to find
            # out when it was open or what opened it.
            if was is not None and self.log is not None:
                self.log.event(ut, "rcs %s (err %s deg%s%s)"
                               % ("on" if wanted else "off",
                                  "?" if error_deg is None or error_deg < 0.0
                                  else "%.1f" % error_deg,
                                  "" if q is None else ", q %.0f Pa" % q,
                                  "" if saturation is None
                                  else ", input %.2f" % saturation))
        return self.on
