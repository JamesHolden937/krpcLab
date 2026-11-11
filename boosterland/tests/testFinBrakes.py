"""brake_fins: which control surfaces make a roll-free airbrake."""
import math
import unittest

from boosterland import autoland


class _Module:
    def __init__(self, name):
        self.name = name


class _Part:
    def __init__(self, name, pos, modules=("ModuleControlSurface",)):
        self.name = name
        self._pos = pos
        self.modules = [_Module(m) for m in modules]

    def position(self, frame):
        return self._pos


class _Surface:
    def __init__(self, part):
        self.part = part


class _Vessel:
    def __init__(self, surfaces):
        self.reference_frame = object()

        class _Parts:
            control_surfaces = surfaces
        self.parts = _Parts()


def ring(n, y=-0.3, r=2.47, name="airlinerTailFin", phase=0.0):
    out = []
    for i in range(n):
        a = phase + 2.0 * math.pi * i / n
        out.append(_Surface(_Part(name, (r * math.cos(a), y,
                                         r * math.sin(a)))))
    return out


class TestBrakeFins(unittest.TestCase):
    def test_six_alternate(self):
        fins = autoland.brake_fins(_Vessel(ring(6)))
        self.assertEqual(len(fins), 6)
        self.assertEqual(sum(1 for _, inv in fins if inv), 3)
        # neighbours in azimuth have opposite directions
        az = [math.atan2(s.part._pos[2], s.part._pos[0]) for s, _ in fins]
        self.assertEqual(az, sorted(az))
        for i in range(6):
            self.assertNotEqual(fins[i][1], fins[(i + 1) % 6][1])

    def test_odd_and_airbrakes_left_out(self):
        brakes = [_Surface(_Part("airbrake1", (2.0, 5.0, 0.0),
                                 ("ModuleAeroSurface",)))]
        fins = autoland.brake_fins(_Vessel(ring(3) + brakes))
        self.assertEqual(fins, [])

    def test_two_fins_left_out(self):
        # alternated, a pair pushes one way: a pitching moment, not a brake
        self.assertEqual(autoland.brake_fins(_Vessel(ring(2))), [])

    def test_four_alternate(self):
        fins = autoland.brake_fins(_Vessel(ring(4)))
        self.assertEqual([inv for _, inv in fins], [False, True, False, True])

    def test_uneven_spacing_left_out(self):
        s = ring(4)
        s[1].part._pos = (1.24, -0.3, 2.14)   # 30 deg toward its neighbour
        self.assertEqual(autoland.brake_fins(_Vessel(s)), [])

    def test_two_rings(self):
        fins = autoland.brake_fins(_Vessel(ring(4) + ring(6, y=-25.0,
                                                         name="fin2")))
        self.assertEqual(len(fins), 10)


if __name__ == "__main__":
    unittest.main()
