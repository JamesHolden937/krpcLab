import math
import unittest

from spaceplane import vacuumown


class TestVacuumOwn(unittest.TestCase):
    # The cargo shuttle's rates per 1 s pulse (body right/nose/belly):
    # yaw makes more pitch than yaw, roll mostly pitch and yaw.
    B = [(-0.018, 0.0, 0.0), (0.023, 0.0, -0.016), (-0.024, -0.018, 0.027)]

    def test_matrix_cancels_the_drift(self):
        drift = (0.01, -0.02, 0.005)
        plus = [tuple(d + c for d, c in zip(drift, col)) for col in self.B]
        minus = [tuple(d - c for d, c in zip(drift, col)) for col in self.B]
        got = vacuumown.matrix(plus, minus, 1.0)
        for g, b in zip(got, self.B):
            for x, y in zip(g, b):
                self.assertAlmostEqual(x, y)

    def test_inputs_make_the_acceleration_asked(self):
        want = (0.0, 0.0, 0.01)                 # pure yaw (belly axis)
        u = vacuumown.inputs_for(self.B, want)
        made = [sum(self.B[c][r] * u[c] for c in range(3)) for r in range(3)]
        for m, w in zip(made, want):
            self.assertAlmostEqual(m, w)
        self.assertNotAlmostEqual(u[0], 0.0)    # pitch cancels yaw's pitch

    def test_command_saturates_together(self):
        auth = vacuumown.axis_authority(self.B)
        u = vacuumown.command(self.B, auth, (math.radians(90), 0.0, 0.0),
                              (0.0, 0.0, 0.0), 1.0, 0.5, 0.2)
        self.assertLessEqual(max(abs(x) for x in u), 1.0 + 1e-9)
        made = [sum(self.B[c][r] * u[c] for c in range(3)) for r in range(3)]
        self.assertGreater(abs(made[0]), 10 * abs(made[1]))
        self.assertGreater(abs(made[0]), 10 * abs(made[2]))

    def test_no_error_no_rate_no_input(self):
        auth = vacuumown.axis_authority(self.B)
        u = vacuumown.command(self.B, auth, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              1.0, 0.5, 0.2)
        self.assertEqual(tuple(round(x, 9) for x in u), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
