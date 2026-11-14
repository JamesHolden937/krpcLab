"""Roll gains for kRPC's attitude controller from the roll the booster
actually gets per unit input, measured in flight (``Config.ROLL_TUNE``).

kRPC's auto-tune sets each axis's rate PID from the game's
``available_torque`` over ``moment_of_inertia`` -- the stock modules'
``GetPotentialTorque``.  For the Heavy Shuttle booster's six tail fins,
falling tail first, that roll figure is a couple of hundred times what the
fins deliver: auto-tune's roll Kp 0.027 implies ~340 rad/s^2 per unit
input, while the flights measure 1-2.3 (LOG16185/16186, a least-squares fit
of the roll acceleration on the roll input).  So the roll loop barely
pushed: the booster spun at 70-110 deg/s below 20 km with the roll input at
0.2.  And the bang-bang profile that turns the roll error into a target
rate plans its braking on the same figure, asking for rates the fins cannot
stop.

From the coast on (the gimbals do not roll there, ``GIMBAL_ROLL_UNTIL_BURN``,
and there is no thrust until the burn) the gains are set here, auto-tune
off:

- **roll**: ``common.rateplant.Plant`` fits the effect ``k x`` kRPC's
  figure from the input flown and the roll rate's answer; ``pi_gains`` puts
  both poles at ``-1/tau``, ``tau`` kRPC's own (``1 / omega`` of its
  time-to-peak and overshoot) slowed to what the fins' actuators can serve
  (``closing_time``).  The target-rate cap is the rate the measured effect
  stops within ``ROLL_BRAKE_DEG``.
- **pitch, yaw**: kRPC's own formula on kRPC's own figure, exactly what
  auto-tune would set (``Kp = 2 zeta omega I / T``, ``Ki = omega^2 I / T``).

Until the fit has seen anything (``Plant.quality`` 0) ``k`` is 1 and the
gains equal auto-tune's.
"""

import math

from common.rateplant import Plant, actuators, authority, closing_time, pi_gains


def damping_and_frequency(time_to_peak, overshoot):
    """(zeta, omega) kRPC derives from an axis's time to peak (s) and
    overshoot fraction (AttitudeController.UpdatePIDParameters)."""
    ln = math.log(max(1e-9, min(0.999, overshoot)))
    zeta = math.sqrt(ln * ln / (math.pi * math.pi + ln * ln))
    omega = math.pi / (max(1e-3, time_to_peak) * math.sqrt(1.0 - zeta * zeta))
    return zeta, omega


def time_to_peak_for(tau, overshoot):
    """The time to peak (s) that gives kRPC's tuned loop a decay time of
    ``tau``: its poles decay at ``zeta omega``, and ``omega = pi / (ttp
    sqrt(1 - zeta^2))``."""
    zeta, _ = damping_and_frequency(1.0, overshoot)
    return math.pi * zeta * tau / math.sqrt(1.0 - zeta * zeta)


def krpc_gains(accel, zeta, omega):
    """kRPC auto-tune's (kp, ki, kd) for an axis of ``accel`` rad/s^2 per
    unit input."""
    if accel is None or accel <= 0.0:
        return None
    return (2.0 * zeta * omega / accel, omega * omega / accel, 0.0)


def rate_cap(effect, brake_deg, default):
    """The largest target rate (rad/s) an axis of ``effect`` rad/s^2 per
    unit input stops within ``brake_deg``; ``default`` without one."""
    if effect is None or effect <= 0.0:
        return default
    return min(default, math.sqrt(2.0 * effect * math.radians(brake_deg)))


def _fmt(values):
    return "(" + ", ".join("%.2f" % v for v in values) + ")"


class RollTune:
    """The kRPC glue: engage once, then ``update`` every tick."""

    def __init__(self, autopilot, vessel, actuator_call, cfg, log):
        self.ap, self.vessel, self.cfg, self.log = autopilot, vessel, cfg, log
        self.actuator_call = actuator_call
        self.plant = Plant(cfg.ROLL_PLANT_FORGET_S, cfg.ROLL_PLANT_P_MAX,
                           cfg.ROLL_PLANT_K_MIN, cfg.ROLL_PLANT_K_MAX,
                           cfg.ROLL_PLANT_D_MAX, cfg.ROLL_PLANT_EXCITE,
                           cfg.ROLL_PLANT_WINDOW_S)
        self.active = False
        self.zw = None              # (zeta, omega) per axis
        self.cap0 = None            # kRPC's max angular velocity per axis
        self.accel = (None, None, None)
        self.acts = ((None, 0.0),) * 3
        self.next_read = -1e9
        self.prev = None            # (ut, roll rate rad/s)
        self.sent = None            # gains last written
        self.kp = self.tau = self.cap = None
        self.ttp0 = self.over = None    # kRPC's own time to peak, overshoot
        self.started = False
        self.held_bw = None             # kRPC's pitch/yaw (Kp a, Ki a)
        self.latched = False            # kRPC's pitch/yaw oscillation latch
        self.floor = None               # ... and its bandwidth floor, rad/s
        self.paced = False              # ATTITUDE_LAG_PACE's time to peak set
        self.ttp_sent = None
        self.lag_logged = [0.0, 0.0, 0.0]
        self.engage_at = None           # engage() deferred to this ut

    def start(self, ut):
        """From START: read kRPC's own tuning (time to peak, overshoot, caps)."""
        try:
            self.ttp0 = tuple(self.ap.time_to_peak)
            self.over = tuple(self.ap.overshoot)
            self.cap0 = tuple(self.ap.max_angular_velocity)
        except Exception as exc:                    # noqa: BLE001
            self.log.event(ut, "attitude tune: kRPC tuning unread (%r)"
                           % (exc,))
            return
        self.zw = [damping_and_frequency(self.ttp0[i], self.over[i])
                   for i in range(3)]
        self.started = True

    def engage(self, ut):
        """From the coast: auto-tune off, roll from the measured effect.

        With the time to peak paced, kRPC's own is put back first
        and the gains are read ``ROLL_TUNE_ENGAGE_DELAY_S`` later: auto-tune
        re-derives them on its next physics tick, and read at once they
        would be the paced ones."""
        if not self.started or self.active:
            return
        if self.paced:
            self.unpace(ut)
            self.engage_at = ut + self.cfg.ROLL_TUNE_ENGAGE_DELAY_S
            return
        self.engage_at = None
        # Pitch and yaw keep the tuning kRPC had reached: its gains times
        # the acceleration it credits are the loop's bandwidths (Kp a =
        # 2 zeta omega, Ki a = omega^2), oscillation mitigation included --
        # 0.59/0.43 at the fin booster's burn, ~1.2 rad/s where its plain
        # formula gives 5.6 (LOG16184).  Written from the formula instead,
        # the coast's pitch rang at 20-47 deg/s and the miss went from 34 to
        # 201 m (attitude2-1112).
        try:
            self.accel = authority(self.vessel.available_torque,
                                   self.vessel.moment_of_inertia)
            pid = (tuple(self.ap.pitch_pid_gains), None,
                   tuple(self.ap.yaw_pid_gains))
            self.held_bw = [None, None, None]
            for i in (0, 2):
                if self.accel[i]:
                    self.held_bw[i] = (pid[i][0] * self.accel[i],
                                       pid[i][1] * self.accel[i])
            self.ap.auto_tune = False
        except Exception as exc:                    # noqa: BLE001
            self.log.event(ut, "roll tune: not engaged (%r)" % (exc,))
            return
        self.active = True
        self.log.event(ut, "roll tune: kRPC auto-tune off, roll from the "
                       "measured effect; pitch/yaw held at kRPC's bandwidth "
                       "%s" % " ".join(
                           "%.2f/%.2f" % b if b else "-"
                           for b in (self.held_bw[0], self.held_bw[2])))

    def lag_ttp(self, i):
        """ATTITUDE_LAG_PACE: the time to peak at which kRPC's loop on axis
        ``i`` decays no faster than ``ROLL_LAG_FACTOR`` x its actuators'
        torque-weighted lag (``gimbalResponseSpeed``: 0.125 s on the
        Vector; exponential surfaces).  kRPC tunes as if the torque arrived
        at once; on the gimbals at full thrust that is a limit cycle --
        inputs pinned at +-1 flipping every tick, nose 10-40 deg/s, roll
        60-190 deg/s (LOG16371, LOG16376/16377).  The lag is weighted by
        torque, so an unlit gimbal adds none and the pace lifts itself when
        the engines stop.  Never below kRPC's own."""
        lag = self.acts[i][1]
        need = time_to_peak_for(self.cfg.ROLL_LAG_FACTOR * lag,
                                self.over[i]) if lag else 0.0
        return max(self.ttp0[i], need)

    def lag_bandwidth(self, i):
        """(Kp a, Ki a) of ``lag_ttp`` -- the most the actuators can serve
        while auto-tune is off -- or None when nothing lags."""
        if not self.acts[i][1]:
            return None
        zeta, omega = damping_and_frequency(self.lag_ttp(i), self.over[i])
        return (2.0 * zeta * omega, omega * omega)

    def pace(self, ut):
        """Auto-tune's time to peak, per axis, from ``lag_ttp``."""
        ttp = []
        for i in range(3):
            lag = self.acts[i][1]
            ttp.append(self.lag_ttp(i))
            if lag and abs(lag - self.lag_logged[i]) > 0.25 * max(
                    lag, self.lag_logged[i]):
                self.lag_logged[i] = lag
                self.log.event(ut, "attitude pace: %s lag %.3f s -> time to "
                               "peak %.2f s" % (("pitch", "roll", "yaw")[i],
                                                lag, ttp[i]))
        self.set_ttp(ut, ttp)
        self.paced = True

    def unpace(self, ut):
        self.set_ttp(ut, self.ttp0)
        self.paced = False
        self.log.event(ut, "attitude pace: off, kRPC's time to peak %s"
                       % _fmt(self.ttp0))

    def set_ttp(self, ut, ttp):
        # Read back, not remembered: something resets kRPC's time to peak
        # after it is set (LOG16199-16202).
        try:
            now = tuple(self.ap.time_to_peak)
            if any(abs(a - b) > 0.02 * a for a, b in zip(ttp, now)):
                if self.ttp_sent is not None and tuple(ttp) == tuple(
                        self.ttp_sent):
                    self.log.event(ut, "attitude pace: time to peak found "
                                   "at %s, set again" % (_fmt(now),))
                self.ap.time_to_peak = tuple(ttp)
            self.ttp_sent = tuple(ttp)
        except Exception as exc:                    # noqa: BLE001
            self.log.event(ut, "attitude pace: time to peak not set (%r)"
                           % (exc,))

    def update(self, ut, roll_rate_deg, roll_input):
        if not self.started:
            return
        pace = self.cfg.ATTITUDE_LAG_PACE
        if self.engage_at is not None and ut >= self.engage_at:
            self.engage(ut)
        if ut >= self.next_read:
            # Faster while pacing: a gimbal's torque (and so its lag) is
            # zero until it is lit, and the limit cycle starts with thrust.
            self.next_read = ut + (0.25 if pace else self.cfg.ROLL_TUNE_READ_S)
            try:
                self.accel = authority(self.vessel.available_torque,
                                       self.vessel.moment_of_inertia)
            except Exception:                       # noqa: BLE001
                pass
            # The surfaces' slew, weighted by their torque -- which is ~0
            # in the thin air the coast starts in, so it is re-read too.
            if self.actuator_call is not None:
                try:
                    acts = actuators(list(self.actuator_call()))
                except Exception:                   # noqa: BLE001
                    acts = None
                if acts:
                    if acts[1][0] and not self.acts[1][0]:
                        self.log.event(ut, "roll tune: roll surfaces slew "
                                       "%.2f full travel/s" % acts[1][0])
                    self.acts = acts
            if self.active:
                self.read_latch(ut)
            if pace and not self.active and self.engage_at is None:
                self.pace(ut)
        if not self.active:
            return
        # kRPC's roll input and our roll rate have opposite senses (the
        # boostback's gimbals: -209..-270 deg/s^2 per unit, corr -0.93/-0.97).
        w = (-math.radians(roll_rate_deg)
             if roll_rate_deg is not None else None)
        prev, self.prev = self.prev, ((ut, w) if w is not None else None)
        if (prev is not None and w is not None and roll_input is not None
                and self.accel[1]):
            self.plant.update(self.accel[1], roll_input, prev[1], w,
                              ut - prev[0])
        self.write(ut)

    def read_latch(self, ut):
        """kRPC's structural-oscillation latch on pitch/yaw and the floor it
        would drop their bandwidth to: auto-tune applies it, and auto-tune
        is off.  Read once a game second."""
        try:
            latched = bool(self.ap.pitch_yaw_oscillation_latched)
            floor = float(self.ap.oscillation_bandwidth_floor)
        except Exception:                           # noqa: BLE001
            return
        if latched != self.latched:
            self.log.event(ut, "roll tune: kRPC pitch/yaw oscillation %s "
                           "(floor %.2f rad/s)" % ("latched" if latched
                                                   else "released", floor))
        self.latched, self.floor = latched, floor

    def py_bandwidth(self, i):
        """(Kp a, Ki a) for pitch or yaw: kRPC's at the coast's start, no
        higher than its oscillation floor while that is latched, nor (with
        ATTITUDE_LAG_PACE) than the actuators' lag serves -- the landing
        burn's gimbals."""
        bw = self.held_bw[i] if self.held_bw else None
        if bw and self.latched and self.floor:
            zeta, _ = damping_and_frequency(1.0, self.over[i])
            bw = (min(bw[0], 2.0 * zeta * self.floor),
                  min(bw[1], self.floor * self.floor))
        lag = self.lag_bandwidth(i) if self.cfg.ATTITUDE_LAG_PACE else None
        if bw and lag:
            bw = (min(bw[0], lag[0]), min(bw[1], lag[1]))
        return bw

    def write(self, ut):
        gains = [krpc_gains(self.accel[i], *self.zw[i]) for i in range(3)]
        for i in (0, 2):
            bw = self.py_bandwidth(i)
            if bw and self.accel[i]:
                gains[i] = (bw[0] / self.accel[i], bw[1] / self.accel[i], 0.0)
        roll = None
        if self.accel[1]:
            effect = self.plant.k * self.accel[1]
            _, omega = self.zw[1]
            cap = rate_cap(effect, self.cfg.ROLL_BRAKE_DEG, self.cap0[1])
            slew, lag = self.acts[1]
            tau = closing_time(1.0 / omega, effect, slew, lag, cap,
                               self.cfg.ROLL_SLEW_FACTOR,
                               self.cfg.ROLL_LAG_FACTOR)
            kp, ki = pi_gains(effect, self.plant.d, tau, 1.0 / cap)
            roll = (kp, ki, 0.0)
            self.kp, self.tau, self.cap = kp, tau, cap
        gains[1] = roll
        if self.sent is not None and all(
                g is None or s is None or all(
                    abs(a - b) <= 0.02 * max(abs(a), abs(b), 1e-9)
                    for a, b in zip(g, s))
                for g, s in zip(gains, self.sent)):
            return
        try:
            if gains[0]:
                self.ap.pitch_pid_gains = gains[0]
            if gains[2]:
                self.ap.yaw_pid_gains = gains[2]
            if roll:
                self.ap.roll_pid_gains = roll
                caps = list(self.cap0)
                caps[1] = self.cap
                self.ap.max_angular_velocity = tuple(caps)
            self.sent = gains
        except Exception as exc:                    # noqa: BLE001
            self.log.event(ut, "roll tune: gains not written (%r)" % (exc,))

    def restart(self, ut):
        """Auto-tune back on at kRPC's own time to peak, and engage again
        ``ROLL_TUNE_ENGAGE_DELAY_S`` later, so the held pitch/yaw bandwidth
        is the one kRPC reaches here rather than the last phase's."""
        self.release()
        self.set_ttp(ut, self.ttp0)
        self.paced = False
        self.sent = None
        self.engage_at = ut + self.cfg.ROLL_TUNE_ENGAGE_DELAY_S
        self.log.event(ut, "roll tune: re-engaging for the %s" % "coast")

    def release(self):
        if not self.active:
            return
        self.active = False
        try:
            self.ap.auto_tune = True
            self.ap.max_angular_velocity = tuple(self.cap0)
        except Exception:                           # noqa: BLE001
            pass

    def fields(self):
        """` rk= rkp= rtau=` for the telemetry line."""
        if not self.active or self.kp is None:
            return ""
        return (" rk=%.3g rq=%.2f rkp=%.3g rtau=%.2f rcap=%.0f"
                % (self.plant.k, self.plant.quality, self.kp, self.tau,
                   math.degrees(self.cap)))
