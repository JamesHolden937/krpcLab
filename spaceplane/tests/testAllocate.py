"""``spaceplane.allocate``: attitude error, wanted torque, bounded
allocation over any set of control surfaces."""

import math
import unittest

from common import vec
from spaceplane import allocate

X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def rotate(v, axis, deg):
    return vec.quat_rotate(vec.quat_axis_angle(axis, math.radians(deg)), v)


class TestAttitudeError(unittest.TestCase):
    def test_zero_when_aligned(self):
        e = allocate.attitude_error((X, Y, Z), (X, Y, Z))
        self.assertAlmostEqual(vec.norm(e), 0.0)

    def test_rotation_vector_matches_the_rate_convention(self):
        # A target reached by rotating about +x: the error points along +x,
        # the axis the rate w must take for d(axis)/dt = w x axis.
        tgt = tuple(rotate(a, X, 5.0) for a in (X, Y, Z))
        e = allocate.attitude_error((X, Y, Z), tgt)
        self.assertGreater(e[0], 0.0)
        self.assertAlmostEqual(e[0], math.sin(math.radians(5.0)), places=6)
        dn = vec.cross(e, Y)          # the nose moves toward the target nose
        self.assertGreater(vec.dot(dn, tgt[1]), 0.0)

    def test_each_axis_separately(self):
        for k, axis in enumerate((X, Y, Z)):
            tgt = tuple(rotate(a, axis, -3.0) for a in (X, Y, Z))
            e = allocate.attitude_error((X, Y, Z), tgt)
            self.assertLess(e[k], 0.0)
            for j in range(3):
                if j != k:
                    self.assertAlmostEqual(e[j], 0.0, places=9)


class TestTargetAxes(unittest.TestCase):
    def test_roof_is_orthogonalised_and_right_is_nose_cross_belly(self):
        right, nose, belly = allocate.target_axes(Y, (0.0, 0.3, -1.0))
        self.assertAlmostEqual(vec.dot(nose, belly), 0.0)
        self.assertAlmostEqual(vec.dot(right, nose), 0.0)
        self.assertAlmostEqual(vec.dot(belly, (0.0, 0.0, 1.0)), 1.0)
        self.assertEqual(allocate.target_axes(Y, Y), None)


class TestWantedTorque(unittest.TestCase):
    def test_spring_and_damper_per_axis(self):
        t = allocate.wanted_torque((2.0, 1.0, 3.0), (0.1, 0.0, 0.0),
                                   (0.0, 0.2, 0.0), 0.5, 0.8)
        self.assertAlmostEqual(t[0], 2.0 * 0.25 * 0.1)
        self.assertAlmostEqual(t[1], -1.0 * 2 * 0.8 * 0.5 * 0.2)
        self.assertAlmostEqual(t[2], 0.0)


class TestSolve(unittest.TestCase):
    def test_canards_and_elevons_share_pitch_elevons_carry_roll(self):
        # Two canards (pitch only), two elevons (pitch and opposite roll).
        cols = [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                (0.5, 0.4, 0.0), (0.5, -0.4, 0.0)]
        want = (1.0, 0.2, 0.0)
        d, made = allocate.solve(cols, want, [-1] * 4, [1] * 4, reg=1e-9)
        self.assertAlmostEqual(made[0], 1.0, places=6)
        self.assertAlmostEqual(made[1], 0.2, places=6)
        self.assertGreater(d[2], d[3])

    def test_one_set_of_elevons_flies_pitch_and_roll(self):
        cols = [(0.5, 0.4, 0.0), (0.5, -0.4, 0.0)]
        d, made = allocate.solve(cols, (0.4, -0.1, 0.0), [-1, -1], [1, 1],
                                 reg=1e-9)
        self.assertAlmostEqual(made[0], 0.4, places=6)
        self.assertAlmostEqual(made[1], -0.1, places=6)

    def test_a_saturated_actuator_hands_its_share_to_the_others(self):
        cols = [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        d, made = allocate.solve(cols, (1.5, 0.0, 0.0), [-1, -0.2],
                                 [1, 0.2], reg=1e-9)
        self.assertAlmostEqual(d[1], 0.2)
        self.assertAlmostEqual(d[0], 1.0)
        self.assertAlmostEqual(made[0], 1.2, places=6)

    def test_nothing_to_move_with(self):
        d, made = allocate.solve([(0.0, 0.0, 0.0)], (1.0, 0.0, 0.0),
                                 [-1], [1])
        self.assertEqual(d, [0.0])


if __name__ == "__main__":
    unittest.main()
