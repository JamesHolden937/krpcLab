"""The roll rate this vehicle delivers, measured while it flies.

``BANK_RATE_DEG_S`` was 8 deg/s with no derivation, set on the old capsule
and never checked against the shuttle, whose roll inertia is 7x on the same
15 kN m of wheel.  A command that slews faster than the vehicle rolls runs
ahead of it, and kRPC's attitude controller stops rolling *altogether* once
the nose is ~20 deg off its target (``roll_start_angle``) -- at 36 deg of
alpha that is a bank lead of about 33 deg, reached in two ticks (LOG3679:
the GUI's roll indicator sat at zero through the reversal).

So the vehicle is asked, every tick, how fast it actually rolled.  A tick
is a sample when the vehicle was being *asked* to roll: the command was
slewing at the current limit, or it led the flown bank by more than
``BANK_RATE_SAT_DEG``.  The rate delivered toward the command then says what
the vehicle can do *now* -- at this q, this Mach, on whatever mix of wheel,
surface and RCS is acting.  A settled vehicle says nothing and is not used.

**Capability is a peak, not an average.**  The start of every roll delivers
nothing while the controller winds up, and an average of those ticks
ratcheted the first version down to 1.1 deg/s on a vehicle seen rolling at
20 (LOG3684).  So the estimate holds the best rate delivered and lets it
decay over ``BANK_RATE_TAU_S`` of sampled time, which follows q as the air
changes without one slow tick throwing it away.  ``BANK_RATE_PROBE`` above
1 keeps the command asking a little more than the last peak, so the vehicle
is always being measured at its limit and the estimate can climb.

**Sideslip is the tolerance.**  Rolling about the body axis at high alpha
turns alpha into sideslip; rolled too fast, the shuttle tumbled (LOG3680,
bank +-150, slip +-50 at Mach 4.8).  While a roll holds the sideslip past
``BANK_RATE_SLIP_TOL_DEG`` the estimate is pulled down by the excess: that
rate was more than the airframe tolerates, whatever the actuators could do.
"""


import math


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class RollRate:
    def __init__(self, cfg):
        self.cfg = cfg
        self.rate = None            # deg/s, measured; None until a sample
        self.samples = 0
        self.peak = 0.0
        self._last = None           # (ut, commanded, flown)

    def update(self, ut, commanded, flown, slip):
        """One tick: the command just sent, the bank flown, the sideslip."""
        if any(x is None or math.isnan(x) for x in (commanded, flown)):
            self._last = None
            return
        last, self._last = self._last, (ut, commanded, flown)
        if last is None:
            return
        dt = ut - last[0]
        if not 0.0 < dt <= float(self.cfg.BANK_RATE_MAX_GAP_S):
            return
        lead = _wrap(last[1] - last[2])
        moved = _wrap(commanded - last[1])
        slewing = abs(moved) / dt >= 0.5 * self.limit()
        if abs(lead) >= float(self.cfg.BANK_RATE_SAT_DEG):
            sense = math.copysign(1.0, lead)
        elif slewing:
            sense = math.copysign(1.0, moved)
        else:
            return
        # Toward the command counts; away from it is zero, not negative.
        # Clamped to the ceiling: one tick across a flown-bank discontinuity
        # read 249 deg/s in the cone.
        toward = _wrap(flown - last[2]) * sense / dt
        sample = min(float(self.cfg.BANK_RATE_MAX_DEG_S), max(0.0, toward))
        self.samples += 1
        self.peak = max(self.peak, sample)
        decay = math.exp(-dt / float(self.cfg.BANK_RATE_TAU_S))
        self.rate = sample if self.rate is None else max(sample,
                                                         self.rate * decay)
        tol = float(self.cfg.BANK_RATE_SLIP_TOL_DEG)
        if slip is not None and not math.isnan(slip) and abs(slip) > tol > 0:
            self.rate *= (tol / abs(slip)) ** (dt / float(
                self.cfg.BANK_RATE_TAU_S))

    def limit(self):
        """deg/s the bank command may slew at.  The configured rate until
        the vehicle has been measured."""
        if self.rate is None:
            return float(self.cfg.BANK_RATE_DEG_S)
        return min(float(self.cfg.BANK_RATE_MAX_DEG_S),
                   max(float(self.cfg.BANK_RATE_MIN_DEG_S),
                       float(self.cfg.BANK_RATE_PROBE) * self.rate))


class RollDamper:
    """Slow the lateral axes when the bank *diverges* about a steady command.

    ``ATTITUDE_ROLL_TIME_TO_PEAK_S`` 1.0 gave the shuttle full roll
    authority and it tracked to a degree up to q ~700 Pa, then swung about a
    *steady* +30 command with growing amplitude -- 8, 12, 14, 16, 22, 29, 35
    deg past it, alternately -- until the nose itself was lost (the user,
    live, LOG3692).  A loop that has gone unstable says so by its swings
    growing, and slowing it is the cure.  How fast is too fast depends on
    the airframe, the alpha and the q, so no constant in seconds is right
    (this one airframe has flown 3.0, 22.6, 4.8 and 1.0).

    **Only growth counts.**  The first version slowed on every crossing of
    a steady command by ``BANK_RATE_SAT_DEG`` each way, and on the shuttle
    it ran to its ceiling on a +-7 deg lateral wobble at q 2300-3300 whose
    amplitude did not change as the tune went 4.8 -> 20 s (LOG3710): a mode
    of the airframe, not of the loop, which a slower loop only makes lag.
    So a *half-swing* is the excursion past a steady command on one side,
    from entering beyond ``BANK_RATE_SAT_DEG`` until it crosses to beyond
    that on the other side; a swing counts when its peak exceeds the peak of
    the half-swing before it by ``ROLL_DAMPER_GROWTH``.  Each counted swing
    multiplies the roll ``time_to_peak`` by ``ROLL_DAMPER_STEP``, up to the
    vehicle's slowest static axis; while none is counted it relaxes back
    toward the floor over ``ROLL_DAMPER_RECOVER_S``.  A crossing made while
    the *command* was still moving (mid-reversal) breaks the chain: that is
    the command's doing, not the loop's.
    """

    def __init__(self, cfg, floor, ceiling):
        self.cfg = cfg
        self.floor = float(floor)
        self.ceiling = max(self.floor, float(ceiling))
        self.tp = self.floor
        self.swings = 0
        self._side = 0              # -1/+1: which side of the command, 0 none
        self._side_cmd = None       # command when that side was entered
        self._peak = 0.0            # |err| peak of the current half-swing
        self._prev_peak = None      # ... and of the one before, if chained
        self._last_ut = None

    def update(self, ut, commanded, flown):
        """One tick.  Returns ``(previous peak, this peak)`` on a counted
        swing, else ``None``; ``tp`` holds the roll ``time_to_peak``."""
        if any(x is None or math.isnan(x) for x in (commanded, flown)):
            self._side, self._prev_peak, self._last_ut = 0, None, None
            return None
        last, self._last_ut = self._last_ut, ut
        if last is None:
            return None
        dt = ut - last
        if not 0.0 < dt <= float(self.cfg.BANK_RATE_MAX_GAP_S):
            self._side, self._prev_peak = 0, None
            return None
        tol = float(self.cfg.BANK_RATE_SAT_DEG)
        err = _wrap(flown - commanded)
        side = 1 if err >= tol else (-1 if err <= -tol else 0)
        counted = None
        if side and side != self._side:
            steady = (self._side_cmd is not None
                      and abs(_wrap(commanded - self._side_cmd)) < tol)
            if self._side and steady:
                done = self._peak
                if (self._prev_peak is not None and done > float(
                        self.cfg.ROLL_DAMPER_GROWTH) * self._prev_peak):
                    self.swings += 1
                    self.tp = min(self.ceiling, self.tp * float(
                        self.cfg.ROLL_DAMPER_STEP))
                    counted = (self._prev_peak, done)
                self._prev_peak = done
            else:
                self._prev_peak = None
            self._side, self._side_cmd, self._peak = side, commanded, 0.0
        elif side and side == self._side:
            # Still out on the same side: a command that walks with it (a
            # slewing reversal) re-anchors and breaks the chain, so only
            # swings about a command that stayed put are compared.
            if abs(_wrap(commanded - self._side_cmd)) >= tol:
                self._side_cmd, self._prev_peak = commanded, None
        if self._side:
            self._peak = max(self._peak, abs(err))
        if counted is None:
            self.tp = max(self.floor, self.tp * math.exp(
                -dt / float(self.cfg.ROLL_DAMPER_RECOVER_S)))
        return counted
