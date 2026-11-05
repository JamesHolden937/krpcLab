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
        """A first-order rate plant (rate' = k u - d rate + bias) settles
        on the command with the integral holding the bias."""
        loop = law.RateLoop(1.5, 1.5, 0.6)
        rate, dt = 0.0, 0.05
        for _ in range(400):
            u = loop.step(0.2, rate, 1.0, dt)
            rate += (2.0 * u - 1.0 * rate - 0.3) * dt
        self.assertAlmostEqual(rate, 0.2, places=2)
        self.assertGreater(loop.i, 0.0)

    def test_no_windup_when_pinned(self):
        loop = law.RateLoop(10.0, 10.0, 0.6)
        for _ in range(100):
            loop.step(5.0, 0.0, 1.0, 0.05)
        self.assertLessEqual(loop.i, 0.6)

    def test_gain_scale(self):
        self.assertEqual(law.gain_scale(None, 5000.0, 0.1, 5.0), 5.0)
        self.assertEqual(law.gain_scale(10000.0, 5000.0, 0.1, 5.0), 0.5)


if __name__ == "__main__":
    unittest.main()
