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

    def update(self, ut, permitted, error_deg, q=None, apply=None):
        """Set the valve for this tick.  Returns what it is now.

        ``permitted`` false is absolute and immediate -- a phase that says no
        is not overruled by a large error -- and the whole mechanism stays
        behind ``ENABLE_RCS``, so a config that has never wanted RCS still
        never gets it.
        """
        permitted = bool(permitted) and bool(self.cfg.ENABLE_RCS)
        self.permitted = permitted
        wanted = permitted and self.demand(ut, error_deg, q)
        if wanted != self.on:
            was = self.on
            self.on = wanted
            if apply is not None:
                apply(wanted)
            # Say so.  The valve is no longer a property of the phase, so a
            # log that prints only phases can no longer be read back to find
            # out when it was open or what opened it.
            if was is not None and self.log is not None:
                self.log.event(ut, "rcs %s (err %s deg%s)"
                               % ("on" if wanted else "off",
                                  "?" if error_deg is None or error_deg < 0.0
                                  else "%.1f" % error_deg,
                                  "" if q is None else ", q %.0f Pa" % q))
        return self.on
