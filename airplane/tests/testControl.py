"""The airplane pilot's laws (airplane/control.py)."""

import math
import unittest

from airplane import control as law

# Surface frame: x up, y north, z east.
UP, NORTH, EAST = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def cross(a, b):
    return law._cross(a, b)


def axes(phi, theta, psi):
    """Body axes (nose, right, roof) in the surface frame for these Euler
    angles, built by turning a north-pointing, wings-level body."""
    # heading, then pitch, then bank: in (up, north, east) components
    ch, sh = math.cos(psi), math.sin(psi)
    ct, st = math.cos(theta), math.sin(theta)
    cp, sp = math.cos(phi), math.sin(phi)
    fwd = (0.0, ch, sh)                 # level nose
    rgt = (0.0, -sh, ch)                # level right wing
    nose = tuple(ct * f + st * u for f, u in zip(fwd, UP))
    roof0 = tuple(-st * f + ct * u for f, u in zip(fwd, UP))
    right = tuple(cp * r - sp * o for r, o in zip(rgt, roof0))
    roof = tuple(cp * o + sp * r for r, o in zip(rgt, roof0))
    return nose, right, roof


class TestAttitude(unittest.TestCase):
    def test_round_trip(self):
        for phi, theta, psi in ((0, 0, 0), (0.5, 0.2, 1.0), (-1.2, -0.4, -2.5),
                                (1.5, 0.6, 3.0)):
            got = law.attitude(*axes(phi, theta, psi))
            for g, w in zip(got, (phi, theta, psi)):
                self.assertAlmostEqual(law.wrap(g - w), 0.0, places=9)

    def test_right_wing_down_is_positive_bank(self):
        nose, right, roof = axes(0.3, 0.0, 0.0)
        self.assertLess(right[0], 0.0)
        self.assertGreater(law.attitude(nose, right, roof)[0], 0.0)


class TestRates(unittest.TestCase):
    """``d(axis)/dt = w x axis``: a rotation vector that moves the nose up
    reads as q > 0, one that drops the right wing as p > 0, one that swings
    the nose right as r > 0."""

    def setUp(self):
        self.nose, self.right, self.roof = axes(0.0, 0.0, 0.0)

    def omega_for(self, axis_from, axis_to):
        # w x a = b for unit perpendicular a, b: w = a x b
        return cross(axis_from, axis_to)

    def test_pitch_up(self):
        w = self.omega_for(self.nose, self.roof)
        p, q, r = law.body_rates(w, self.nose, self.right, self.roof)
        self.assertAlmostEqual(q, 1.0)
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(r, 0.0)

    def test_roll_right(self):
        belly = tuple(-x for x in self.roof)
        w = self.omega_for(self.right, belly)
        p, q, r = law.body_rates(w, self.nose, self.right, self.roof)
        self.assertAlmostEqual(p, 1.0)
        self.assertAlmostEqual(q, 0.0)
        self.assertAlmostEqual(r, 0.0)

    def test_yaw_right(self):
        w = self.omega_for(self.nose, self.right)
        p, q, r = law.body_rates(w, self.nose, self.right, self.roof)
        self.assertAlmostEqual(r, 1.0)
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(q, 0.0)

    def test_kinematics_match_finite_difference(self):
        """Euler rates turned into body rates move the axes the way the
        Euler angles say."""
        phi, theta, psi = 0.7, 0.4, 0.3
        dphi, dtheta, dpsi = 0.1, -0.05, 0.2
        p, q, r = law.euler_to_body(phi, theta, dphi, dtheta, dpsi)
        h = 1e-6
        a0 = axes(phi, theta, psi)
        a1 = axes(phi + dphi * h, theta + dtheta * h, psi + dpsi * h)
        nose, right, roof = a0
        belly = tuple(-x for x in roof)
        d = [tuple((x1 - x0) / h for x0, x1 in zip(v0, v1))
             for v0, v1 in zip(a0, a1)]
        self.assertAlmostEqual(law._dot(d[1], belly), p, places=4)
        self.assertAlmostEqual(law._dot(d[0], roof), q, places=4)
        self.assertAlmostEqual(law._dot(d[0], right), r, places=4)


class TestTurn(unittest.TestCase):
    def test_banked_turn_pulls_back(self):
        """Banked 45 at 30 deg up: holding pitch needs a pull (q > 0) and a
        yaw into the turn (r > 0)."""
        phi, theta = math.radians(45), math.radians(30)
        dpsi = law.turn_rate(phi, 100.0, 4.0, 30.0)
        self.assertAlmostEqual(dpsi, law.G0 / 100.0)
        p, q, r = law.euler_to_body(phi, theta, 0.0, 0.0, dpsi)
        self.assertGreater(q, 0.0)
        self.assertGreater(r, 0.0)
        self.assertLess(p, 0.0)

    def test_load_cap(self):
        rate = law.turn_rate(math.radians(89), 100.0, 4.0, 30.0)
        self.assertAlmostEqual(rate, law.G0 * math.sqrt(15.0) / 100.0)


class TestStick(unittest.TestCase):
    def test_player_input_recovered(self):
        self.assertAlmostEqual(law.stick(0.5, 0.2, 0.1), 0.2)
        self.assertIsNone(law.stick(None, 0.2, 0.0))

    def test_sideslip_sense(self):
        # air from the right (velocity has a +x, rightward, component)
        self.assertGreater(law.sideslip((5.0, 100.0, 0.0)), 0.0)
        self.assertIsNone(law.sideslip((0.0, 0.0, 3.0)))


class TestRateLoop(unittest.TestCase):
    def test_closed_loop_settles(self):
        """A rate plant (rate' = accel u - damping + bias) flown on gains
        from its own authority settles on the command, the integral holding
        the bias -- for authorities 50x apart, with no retuning."""
        for accel in (0.4, 2.0, 20.0):
            tau, dt = 0.2, 0.05
            kp, ki = law.pi_gains(accel, 0.2, tau, 1e9)
            loop = law.RateLoop(0.6)
            rate = 0.0
            for _ in range(600):
                u = loop.step(0.1, rate, kp, ki, dt)
                rate += (accel * u - 0.2 * rate - 0.1 * accel) * dt
            self.assertAlmostEqual(rate, 0.1, places=3, msg=accel)
            self.assertAlmostEqual(loop.i, 0.1 + 0.2 * 0.1 / accel, places=3)

    def test_no_windup_when_pinned(self):
        loop = law.RateLoop(0.6)
        for _ in range(100):
            loop.step(5.0, 0.0, 10.0, 10.0, 0.05)
        self.assertLessEqual(loop.i, 0.6)


class TestGains(unittest.TestCase):
    def test_authority(self):
        acc = law.authority(((100.0, 300.0, 50.0), (-100.0, -100.0, -50.0)),
                            (10.0, 20.0, 0.0))
        self.assertAlmostEqual(acc[0], 10.0)
        self.assertAlmostEqual(acc[1], 10.0)
        self.assertIsNone(acc[2])
        self.assertEqual(law.authority(None, (1, 1, 1)), (None, None, None))



class TestRinging(unittest.TestCase):
    def ring(self):
        return law.Ringing(4, 0.15, 2.0, 0.05, 0.125)

    def test_oscillation_halves(self):
        r, t, cut = self.ring(), 0.0, False
        for k in range(40):
            t += 0.05
            cut |= r.update(t, 0.5 if (k // 3) % 2 else -0.5, 0.05)
        self.assertTrue(cut)
        self.assertLess(r.mult, 1.0)

    def test_steady_and_slow_moves_do_not(self):
        r, t = self.ring(), 0.0
        for k in range(200):
            t += 0.05
            self.assertFalse(r.update(t, 0.3 + 0.002 * k, 0.05))
        r2 = self.ring()
        for k in range(200):        # small dither under the swing
            self.assertFalse(r2.update(k * 0.05, 0.05 * (-1) ** k, 0.05))
        self.assertEqual(r2.mult, 1.0)

    def test_recovers_when_quiet(self):
        r = self.ring()
        r.mult = 0.5
        for k in range(100):
            r.update(k * 0.05, 0.2, 0.05)
        self.assertAlmostEqual(r.mult, 0.75, places=6)


class TestActuators(unittest.TestCase):
    def test_slew_and_lag(self):
        """Pitch: one linear surface (25 deg/s over 15 deg = 1.67 travel/s)
        carrying 90 of 100; roll: wheels only; yaw: an exponential surface
        (speed 10 over 20: lag 2.0 + 0.1 s) carrying all of it."""
        lines = ["total 100 50 40 100 50 40",
                 "25 15 0 90 0 0 90 0 0",
                 "10 20 1 0 0 40 0 0 40"]
        (sp, lp), (sr, lr), (sy, ly) = law.actuators(lines)
        self.assertAlmostEqual(sp, 100.0 / (90.0 / (25.0 / 15.0)))
        self.assertEqual(lp, 0.0)
        self.assertIsNone(sr)
        self.assertEqual(lr, 0.0)
        self.assertIsNone(sy)
        self.assertAlmostEqual(ly, 2.1)
        self.assertIsNone(law.actuators([]))

    def test_closing_time(self):
        # fast surfaces: the tick decides
        self.assertEqual(law.closing_time(0.28, 10.0, 100.0, 0.0, 0.26,
                                          2.0, 3.0), 0.28)
        # slow surfaces and little authority: the surfaces decide
        tau = law.closing_time(0.28, 0.5, 0.5, 0.0, 0.26, 2.0, 3.0)
        self.assertAlmostEqual(tau, math.sqrt(2.0 * 0.26 / (0.5 * 0.5)))
        # a lagged surface
        self.assertAlmostEqual(law.closing_time(0.28, 10.0, None, 0.5, 0.26,
                                                2.0, 3.0), 1.5)

    def test_loop_never_outruns_its_surfaces(self):
        loop = law.RateLoop(0.6)
        us = [loop.step(1.0, 0.0, 10.0, 0.0, 0.05, slew=1.0)
              for _ in range(10)]
        steps = [b - a for a, b in zip([0.0] + us, us)]
        self.assertTrue(all(st <= 0.05 + 1e-9 for st in steps))
        self.assertAlmostEqual(us[-1], 0.5)


def slewed_plant(accel, slew, tau, dt=0.07, seconds=30.0, model_slew=None):
    """A rate step response flown through a surface that moves at ``slew``
    full travel per second, one tick of latency; the loop's gains from
    ``tau``.  Returns the worst overshoot past the 0.2 rad/s steps after
    the first 12 s."""
    loop = law.RateLoop(0.6)
    kp, ki = law.pi_gains(accel, 0.3, tau, 20.0)
    rate, cmd, surf, t, over = 0.0, 0.0, 0.0, 0.0, 0.0
    while t < seconds:
        t += dt
        want = 0.2 if (t % 6) < 3 else -0.2
        u = loop.step(want, rate, kp, ki, dt, slew=model_slew)
        for _ in range(int(round(dt / 0.02))):
            surf += law.clamp(cmd - surf, -slew * 0.02, slew * 0.02)
            rate += (accel * surf - 0.3 * rate) * 0.02
        cmd = u
        if t > 12.0:
            over = max(over, abs(rate) - 0.2)
    return over


class TestSlowSurfaces(unittest.TestCase):
    def test_closing_time_from_the_surfaces_stops_the_ringing(self):
        """Slow surfaces (0.4 travel/s: 60 deg at 25 deg/s) on a 3 rad/s^2
        axis: a loop sized on its tick alone overshoots a 0.2 rad/s step by
        ~0.76 rad/s; sized by ``closing_time`` and slewed, by under 0.07."""
        accel, slew = 3.0, 0.4
        tick_tau = 4 * 0.07
        naive = slewed_plant(accel, slew, tick_tau)
        tau = law.closing_time(tick_tau, accel, slew, 0.0, 0.2, 2.0, 3.0)
        aware = slewed_plant(accel, slew, tau, model_slew=slew)
        self.assertGreater(naive, 0.2)
        # 0.048 under the old kp = 1/(accel tau), ki = kp/(4 tau); the pole
        # placement's integral is 4x stronger and its zero overshoots 0.058.
        self.assertLess(aware, 0.07)


class TestPlant(unittest.TestCase):
    def _fly(self, true_k, d, accel=10.0, n=2000, dt=0.08, excite=0.01):
        """A rate plant w' = true_k accel u - d w + 0.3, driven by a loop
        on kRPC's (wrong) figure plus steps of the player's stick."""
        import random
        rnd = random.Random(1)
        pl = law.Plant(10.0, 100.0, 1 / 16, 16, 20, excite, 1.0)
        w, u_stick = 0.0, 0.0
        for k in range(n):
            if k % 40 == 0:
                u_stick = rnd.choice((-0.05, 0.0, 0.05))
            u = -0.02 * w + u_stick
            w1 = w
            for _ in range(8):
                w1 += (true_k * accel * u - d * w1 + 0.3) * dt / 8
            pl.update(accel, u, w, w1, dt)
            w = w1
        return pl

    def test_finds_a_wrong_figure(self):
        """kRPC 4x low (roll) and 4x high (pitch): the fit finds both."""
        for true_k, d in ((4.0, 1.2), (0.25, 5.0)):
            pl = self._fly(true_k, d)
            self.assertAlmostEqual(pl.k / true_k, 1.0, delta=0.15)
            self.assertAlmostEqual(pl.d, d, delta=0.3 * d)

    def test_holds_without_excitation(self):
        """An input that never steps teaches nothing: k stays where it was."""
        pl = law.Plant(10.0, 100.0, 1 / 16, 16, 20, 0.01, 1.0)
        w = 0.0
        for _ in range(500):
            w1 = w + (0.3 - 2.0 * w) * 0.08
            pl.update(10.0, 0.001, w, w1, 0.08)
            w = w1
        self.assertEqual(pl.n, 0)
        self.assertEqual(pl.k, 1.0)

    def test_a_fit_that_explains_nothing_is_not_trusted(self):
        """The rate driven by a disturbance the model does not have (a
        coupled axis): the fit may land anywhere, and the k used stays
        near kRPC's 1."""
        import random
        rnd = random.Random(2)
        pl = law.Plant(10.0, 100.0, 1 / 16, 16, 20, 0.01, 1.0)
        w = 0.0
        for k in range(2000):
            u = rnd.choice((-0.05, 0.0, 0.05)) if k % 5 == 0 else u
            w1 = w + (0.2 * 10.0 * u + rnd.gauss(0.0, 3.0)) * 0.08
            pl.update(10.0, u, w, w1, 0.08)
            w = 0.9 * w1
        self.assertLess(pl.quality, 0.2)
        self.assertLess(abs(math.log(pl.k)), 0.4)


class TestPiGains(unittest.TestCase):
    def test_poles_at_one_over_tau(self):
        """s^2 + (d + B kp) s + B ki = (s + 1/tau)^2."""
        b, d, tau = 12.0, 1.0, 0.3
        kp, ki = law.pi_gains(b, d, tau, 20.0)
        self.assertAlmostEqual(d + b * kp, 2.0 / tau)
        self.assertAlmostEqual(b * ki, 1.0 / tau ** 2)

    def test_heavily_damped_axis_gets_no_proportional(self):
        kp, ki = law.pi_gains(12.0, 10.0, 0.3, 20.0)
        self.assertEqual(kp, 0.0)
        self.assertGreater(ki, 0.0)

    def test_no_effect(self):
        self.assertEqual(law.pi_gains(None, 1.0, 0.3, 20.0)[0], 20.0)

    def test_little_authority_is_held_to_full_input_at_the_cap(self):
        """At walking pace (effect 0.05 rad/s^2 per unit) the poles would
        ask for 2/(tau effect); held to 1/cap, the loop is critically
        damped on what that gain gives: one overshoot, the PI zero's
        ~14%, and no ring."""
        effect, cap, dt = 0.05, 1.0, 0.05
        kp, ki = law.pi_gains(effect, 0.0, 2.0, 1.0 / cap)
        self.assertEqual(kp, 1.0)
        self.assertAlmostEqual(ki, 0.25 * effect * kp * kp)
        loop, rate, peak, rates = law.RateLoop(0.6), 0.0, 0.0, []
        for _ in range(8000):
            u = loop.step(0.01, rate, kp, ki, dt)
            rate += effect * u * dt
            peak = max(peak, rate)
            rates.append(rate)
        self.assertLess(peak, 0.0115)
        self.assertGreater(min(rates[len(rates) // 3:]), 0.0099)


class TestLongTicks(unittest.TestCase):
    def fly(self, bound):
        """Gains sized on 0.05 s ticks (tau = 4 of them), flown on 0.2 s
        ticks with the command a tick late: the rate's peak error over the
        last five ticks, from a 0.1 rad/s step."""
        accel, dt = 20.0, 0.2
        tau = law.tick_tau(0.2, dt, 4.0) if bound else 0.2
        kp, ki = law.pi_gains(accel, 0.0, tau, 1e9)
        loop = law.RateLoop(0.6)
        rate, u_late, errs = 0.0, 0.0, []
        for _ in range(100):
            u = loop.step(0.1, rate, kp, ki, dt)
            rate += accel * u_late * dt
            u_late = u
            errs.append(abs(rate - 0.1))
        return max(errs[-5:])

    def test_long_ticks_ring_without_the_bound(self):
        self.assertGreater(self.fly(False), 0.1)
        self.assertLess(self.fly(True), 0.01)

    def test_bound_leaves_a_short_tick_alone(self):
        self.assertEqual(law.tick_tau(0.3, 0.05, 4.0), 0.3)
        self.assertAlmostEqual(law.tick_tau(0.3, 0.1, 4.0), 0.4)


class TestPhaseMargin(unittest.TestCase):
    def overshoot(self, ticks):
        """A 0.1 rad/s step on a pure-acceleration axis, the command a tick
        late and held a tick (the farm's loop): the rate's peak over the
        step, relative."""
        accel, dt = 20.0, 0.1
        kp, ki = law.pi_gains(accel, 0.0, ticks * dt, 1e9)
        loop = law.RateLoop(0.6)
        rate, u_late, peak = 0.0, 0.0, 0.0
        for _ in range(200):
            u = loop.step(0.1, rate, kp, ki, dt)
            rate += accel * u_late * dt
            u_late = u
            peak = max(peak, rate)
        return peak / 0.1 - 1.0

    def test_margin_is_what_the_formula_says(self):
        """The loop's phase at its crossover, with the delay, leaves the
        margin asked for."""
        for margin in (40.0, 47.0, 55.0):
            tau = law.tau_ticks(margin, 1.5)
            lo, hi = 1e-3, 100.0          # |L(jw)| = 1 by bisection
            for _ in range(200):
                w = 0.5 * (lo + hi)
                mag = math.hypot(2.0 * w / tau, 1.0 / tau ** 2) / w ** 2
                lo, hi = (w, hi) if mag > 1.0 else (lo, w)
            phase = (-180.0 + math.degrees(math.atan(2.0 * w * tau))
                     - math.degrees(w * 1.5))
            self.assertAlmostEqual(phase + 180.0, margin, places=6)

    def test_the_margin_takes_the_ringing_out(self):
        """At 4 ticks (32 deg of margin) the step overshoots by more than
        half; at the 47 deg default's ~6 ticks by under a third."""
        self.assertGreater(self.overshoot(4.0), 0.5)
        self.assertLess(self.overshoot(law.tau_ticks(47.0, 1.5)), 0.33)

    def test_no_margin_past_the_undelayed_loop(self):
        with self.assertRaises(ValueError):
            law.tau_ticks(80.0, 1.5)


class TestStickGate(unittest.TestCase):
    def gate(self):
        return law.StickGate(0.05, 0.2, 0.15)

    def test_one_tick_flicker_is_not_the_player(self):
        g = self.gate()
        for s in (0.0, 0.09, 0.0, -0.08, 0.0, 0.1, 0.0) * 5:
            self.assertFalse(g.update(s, 0.06))

    def test_held_stick_is_the_player_then_released(self):
        g = self.gate()
        got = [g.update(0.1, 0.06) for _ in range(4)]
        self.assertEqual(got, [False, False, True, True])
        got = [g.update(0.0, 0.06) for _ in range(4)]
        self.assertEqual(got, [True, True, False, False])

    def test_firm_stick_at_once(self):
        self.assertTrue(self.gate().update(0.5, 0.06))

    def test_unreadable_keeps_state(self):
        g = self.gate()
        self.assertFalse(g.update(None, 0.06))


class TestGround(unittest.TestCase):
    def test_situations(self):
        class S:
            def __init__(self, name):
                self.name = name
        for n in ("landed", "splashed", "pre_launch"):
            self.assertTrue(law.on_ground(S(n)))
        for n in ("flying", "sub_orbital"):
            self.assertFalse(law.on_ground(S(n)))
        self.assertTrue(law.on_ground("VesselSituation.landed"))
        self.assertFalse(law.on_ground(None))


if __name__ == "__main__":
    unittest.main()
