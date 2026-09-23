"""Pace a control loop on in-game seconds instead of wall-clock seconds.

Both autopilots tick on ``time.sleep(interval)``, which is the right thing
while the game runs at 1x and the wrong thing the moment it does not.  Run the
flight at 2x -- see ``testInstances/timescaleSrc`` -- and a 20 Hz loop
becomes a 10 Hz loop *in the air*, because the vehicle covered twice as much
trajectory between two commands.  The physics would still be faithful; the
controller flying it would not be, and the flight that came out would be a
flight nobody can reproduce at 1x.

So when the game clock is not the wall clock, the loop has to follow the game
clock.  This is the same rule ``Logbook.telemetry`` already follows for log
pacing, and for the same reason.

The cost is one ``space_center.ut`` round trip per sleep, which is why this
estimates the current time ratio and sleeps most of the way in one shot rather
than polling in a tight loop.
"""
import os
import time


class GameClockPacer:
    """Wait until the game clock has advanced by ``interval`` seconds.

    ``ut_fn`` is called to read the current universal time.  The ratio of game
    seconds to real seconds is estimated as it goes, so the first sleep of each
    wait lands close even at 3x, and the poll loop is a correction rather than
    the mechanism.
    """

    # Never sleep shorter than this: at 3x a 0.05 s game interval is 17 ms of
    # wall clock, and a sleep much below that costs more in syscalls and RPC
    # round trips than it buys in responsiveness.
    MIN_SLEEP_S = 0.004

    # How much of the running rate estimate survives each wait.  High enough
    # to ride out one slow tick, low enough to follow the adaptive time scale
    # while it is still ramping up.
    FORGET = 0.8

    def __init__(self, ut_fn, max_stretch=5.0):
        self.ut_fn = ut_fn
        # A paused game advances ut not at all, and a loop that waits for it
        # unconditionally never returns.  This is the ceiling on how long one
        # wait may block in wall-clock seconds, expressed as a multiple of the
        # interval asked for: a missing tick must not look like a slow one.
        self.max_stretch = max_stretch
        # Unknown until measured.  Seeding it at 1.0 makes the first wait of a
        # flight overshoot by exactly the speedup factor -- 0.05 game-seconds
        # asked for, 0.17 delivered at 4.2x -- and the first ticks of a flight
        # are the separation ones.
        self.ratio = None
        # Exponentially forgotten totals rather than a per-wait average: the
        # ratio is a property of the machine under load, and a single
        # scheduling hiccup should not move it far.
        self._game = 0.0
        self._wall = 0.0
        self.timeouts = 0

    def wait(self, interval, from_ut):
        """Block until ``ut >= from_ut + interval``; return the ut reached.

        Returns the last ut actually read, so the caller can tell how far the
        game really got -- which is not always what was asked for.
        """
        if interval <= 0.0:
            return from_ut
        target = from_ut + interval
        wall_deadline = time.monotonic() + interval * self.max_stretch
        wall_start = time.monotonic()
        ut = from_ut
        while True:
            remaining = target - ut
            if remaining <= 0.0:
                break
            if time.monotonic() >= wall_deadline:
                # The game is paused, or so far behind that waiting for it is
                # indistinguishable from hanging.  Give the tick back to the
                # caller rather than stalling the autopilot.
                self.timeouts += 1
                break
            if self.ratio is None:
                # No estimate yet: probe with the shortest useful sleep and
                # learn the rate from what the game did, rather than guessing.
                time.sleep(self.MIN_SLEEP_S)
            else:
                # Approach the target from BELOW.  Sleeping the whole estimated
                # remainder in one shot means a stale ratio overshoots and
                # cannot correct inside this wait -- and overshoot is the very
                # thing this class exists to prevent, since it lowers the
                # control rate the vehicle sees.  Undershooting costs one more
                # cheap poll.
                time.sleep(max(self.MIN_SLEEP_S, 0.8 * remaining / self.ratio))
            ut = self.ut_fn()

        wall = time.monotonic() - wall_start
        if wall > 0.0 and ut > from_ut:
            self._game = self._game * self.FORGET + (ut - from_ut)
            self._wall = self._wall * self.FORGET + wall
            if self._wall > 0.0:
                self.ratio = max(0.05, self._game / self._wall)
        return ut


def sleeper(cfg, ut_fn):
    """Return ``wait(interval, from_ut)`` honouring ``LOOP_PACING_GAME_TIME``.

    Wall-clock pacing stays the default: at 1x the two are the same thing, and
    the version that costs no extra RPC per tick is the one to run when there
    is nothing to gain from the other.
    """
    if not getattr(cfg, "LOOP_PACING_GAME_TIME", False):
        def wall(interval, from_ut):
            time.sleep(interval)
            return from_ut + interval
        return wall
    pacer = GameClockPacer(
        ut_fn, max_stretch=getattr(cfg, "LOOP_PACING_MAX_STRETCH", 5.0))
    return pacer.wait


class LoopRate:
    """What the control loop is *achieving*, measured rather than assumed.

    Two numbers per phase and the second one is the one that governs:

    ``interval`` is the game time between one command and the next -- what the
    vehicle experiences, and the only honest description of the controller
    that flew it.  ``busy`` is the wall time one tick costs -- what the machine
    can do, which is a property of kRPC round trips and the propagation, not
    of the game clock.

    A flight run off 1x has a third number, the time scale, and it is exactly
    ``interval / busy`` when the loop is saturated.  That is why the two are
    kept apart: the scale is the free variable and ``interval`` is the thing
    that must be held constant if a landing flown in the farm is to be the
    landing a person flies at 1x.  Spaceplane failure 63 is what happens when
    it is not: the same configuration entered the flare at 91-102 m/s with a
    coarse loop and 52-79 with a fine one, and rolled out three kilometres
    further.
    """

    # Exponential forgetting, per phase.  A phase's first tick carries the
    # previous phase's wait inside it, and a propagation that occasionally
    # takes twice as long should not move the estimate far.
    FORGET = 0.75

    def __init__(self):
        self.phases = {}            # phase -> [interval, busy, ticks]
        self._last_ut = None
        self._last_phase = None

    def sample(self, phase, ut, busy):
        """One tick: ``ut`` at the top of it, ``busy`` wall seconds of work."""
        row = self.phases.setdefault(phase, [None, None, 0, None])
        row[2] += 1
        row[1] = _blend(row[1], busy, self.FORGET)
        row[3] = busy if row[3] is None else max(row[3], busy)
        if self._last_ut is not None and phase == self._last_phase:
            step = ut - self._last_ut
            # A phase change, a reverted save or a warp can put anything here;
            # only plausible steps are allowed to move the estimate.
            if 0.0 < step < 60.0:
                row[0] = _blend(row[0], step, self.FORGET)
        self._last_ut, self._last_phase = ut, phase

    def busy(self, phase):
        row = self.phases.get(phase)
        return row[1] if row else None

    def peak(self, phase):
        """The most expensive tick this phase has ever had, undecayed.

        **What a governor must serve is the worst tick, not the mean one**,
        and ``busy`` above cannot supply it: it is an exponential average, so
        by the time ``ScaleGovernor`` applies its own decaying-maximum the
        tail has already been smoothed away.  Two filters in series, and the
        second one never sees what the first removed -- which is how a
        governor whose docstring says "not the mean: what binds is the tail"
        came to be driven by a mean.

        The phase that showed it is ``DEORBIT``.  Its search ticks cost
        50-80 ms and its waiting ticks 3-12, and while it waits for a pass
        the blended estimate falls to the cheap ones and the scale ramps to
        the ceiling; the burn then ignites at 6x and delivers its first
        command 0.3 game-seconds late.  Measured on ``qs_plane_inc`` over 44
        flights, the achieved interval is bimodal -- 0.10-0.21 against
        0.29-0.32 -- and the arrival follows it with nothing in between
        (+-2.2 km against -8.7 to -12.2).  Spaceplane failure 91.

        Undecayed on purpose.  A decay is what let the estimate forget, and
        the phase is short enough that one pathological tick pinning the
        scale for the rest of it is the cheap mistake to make.
        """
        row = self.phases.get(phase)
        return row[3] if row and len(row) > 3 else None

    def interval(self, phase):
        row = self.phases.get(phase)
        return row[0] if row else None

    def report(self):
        """One line, in flight order of first appearance.

        Written at shutdown so that *every* log says what control rate flew
        it.  Failure 63 cost a session because no log said, and a
        configuration fitted against a loop that cannot keep up is fitted
        against the shortfall.
        """
        bits = []
        for phase, (interval, busy, ticks, _peak) in self.phases.items():
            bits.append("%s %s/%s n=%d"
                        % (phase,
                           "--" if interval is None else "%.2f" % interval,
                           "--" if busy is None else "%.3f" % busy,
                           ticks))
        return "loop rate, game-s per tick / wall-s of work: " + " ".join(bits)


def _blend(old, new, forget):
    if old is None:
        return new
    return forget * old + (1.0 - forget) * new


class ScaleGovernor:
    """Hold the game's time scale down to what the control loop can serve.

    The measurement farm runs the game off 1x for throughput, and the price
    used to be paid by the controller: at 6x this loop delivers a command
    every ~2 game-seconds where the approach wants one every 0.1, so the
    landing chain was fitted to a vehicle nobody flies (spaceplane failure
    63).  Turning the speed down fixes that and costs the throughput the farm
    exists for.

    Neither is necessary, because the quantity that has to be constant is the
    *control interval*, not the time scale.  Ask for ``wanted`` game-seconds
    between commands, measure what one tick actually costs in wall time, and
    the fastest honest scale is the ratio.  In orbit that is tens of x and the
    governor asks for the ceiling; on final it is one or two.  The phase that
    needs the fidelity gets it, the phases that do not stay fast, and the same
    binary flies both without a flag saying which.

    ``path`` is the plugin's control file (``testInstances/kspN/timescale.txt``
    -- see ``timescale.py``).  This is the one place flight software writes to
    the measurement harness, and it is off unless a path is configured.
    """

    # How the tick cost is estimated from a noisy sequence.  Not the mean:
    # what binds is the *tail*, because one 700 ms tick inside a 0.1 s control
    # interval is one command the vehicle did not get, and the mean of a
    # sequence with a few of those in it says the loop is fine.  So the
    # estimate jumps to any tick more expensive than itself and decays back
    # over about fifteen cheap ones.
    DECAY = 0.9

    def __init__(self, path, maximum=8.0, minimum=1.0, margin=0.8,
                 quant_s=0.05, on_change=None):
        self.path = path
        self.maximum = float(maximum)
        self.minimum = float(minimum)
        # The ceiling is computed from a *mean* tick cost and what matters is
        # the tail: a tick twice as expensive as the mean must still land
        # inside the interval.  This is that headroom.
        self.margin = float(margin)
        self.quant_s = float(quant_s)
        self.on_change = on_change
        self.commanded = None
        self.announced = None
        self.cost = None

    def serve(self, wanted, busy):
        """Command the fastest scale that still puts a command every ``wanted``."""
        if not self.path or busy is None or busy <= 0.0 or wanted <= 0.0:
            return self.commanded
        self.cost = (busy if self.cost is None
                     else max(busy, self.DECAY * self.cost))
        scale = self.margin * wanted / self.cost
        scale = max(self.minimum, min(self.maximum, scale))
        # Only rewrite on a real change: the plugin re-reads twice a second
        # and a file being rewritten every tick is a file it reads half
        # written.  (It tolerates that -- see ``TimeScale.ReadConfig`` -- but
        # the tolerance is not a licence.)
        if (self.commanded is not None
                and abs(scale - self.commanded) < 0.15 * self.commanded):
            return self.commanded
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w") as fh:
                fh.write("mode = fixed\nscale = %.4f\nmax_scale = %.4f\n"
                         "quant_s = %.4f\n"
                         % (scale, self.maximum, self.quant_s))
            os.replace(tmp, self.path)
        except OSError:
            # A farm concern must never take the flight down with it.
            self.path = None
            return self.commanded
        previous, self.commanded = self.commanded, scale
        # Announce a *change*, not a wobble: the scale tracks a noisy cost and
        # a line per tick is a log nobody reads.
        if (self.on_change is not None
                and (self.announced is None
                     or abs(scale - self.announced) > 0.4 * self.announced)):
            self.on_change(self.announced, scale)
            self.announced = scale
        return scale
