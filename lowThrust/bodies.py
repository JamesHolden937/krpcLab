"""The stand-in solar system: stock KSP bodies on fixed Keplerian rails.

The numbers are the stock game's (from memory of the wiki, epoch UT 0), good
enough for a stand-in; the pilot must read the live ones through kRPC
(``CelestialBody.orbit``, ``gravitational_parameter``, ``sphere_of_influence``)
and never these.  All bodies' equators are parallel in KSP (no axial tilt),
so every orbit here is in one frame: x-y the reference plane, longitudes of
the node from +x.

Positions are relative to the parent, in that parent's non-rotating frame.
"""

import math
from dataclasses import dataclass, field

from common import vec
from lowThrust import orbit


@dataclass
class Body:
    name: str
    mu: float
    radius: float
    atmosphere: float = 0.0
    parent: "Body" = None
    sma: float = 0.0
    ecc: float = 0.0
    inc: float = 0.0          # rad
    lan: float = 0.0          # rad
    argpe: float = 0.0        # rad
    mean_anomaly0: float = 0.0   # rad, at UT 0
    children: list = field(default_factory=list)

    @property
    def soi(self):
        """Sphere of influence radius, a (m / M)^0.4 -- as KSP computes it."""
        if self.parent is None:
            return math.inf
        return self.sma * (self.mu / self.parent.mu) ** 0.4

    @property
    def period(self):
        return 2 * math.pi * math.sqrt(self.sma ** 3 / self.parent.mu)

    def state(self, t):
        """(r, v) relative to the parent at UT ``t``."""
        M = self.mean_anomaly0 + 2 * math.pi * t / self.period
        nu = orbit.true_from_mean(M % (2 * math.pi), self.ecc)
        return orbit.state_from_elements(self.parent.mu, self.sma, self.ecc,
                                         self.inc, self.lan, self.argpe, nu)

    def position(self, t):
        return self.state(t)[0]

    @property
    def normal(self):
        """Unit normal of this body's orbit about its parent."""
        return orbit.plane_normal(self.inc, self.lan)


def _deg(x):
    return math.radians(x)


def stock():
    """The stock system (the bodies the transfers use), by name."""
    sun = Body("Sun", 1.1723328e18, 261_600_000.0)
    bodies = {"Sun": sun}

    def add(name, parent, mu, radius, atm, sma, ecc, inc, lan, argpe, m0):
        b = Body(name, mu, radius, atm, bodies[parent], sma, ecc, _deg(inc),
                 _deg(lan), _deg(argpe), m0)
        bodies[parent].children.append(b)
        bodies[name] = b

    add("Moho", "Sun", 1.6860938e11, 250e3, 0, 5_263_138_304, 0.2, 7.0,
        70.0, 15.0, 3.14)
    add("Eve", "Sun", 8.1717302e12, 700e3, 90e3, 9_832_684_544, 0.01, 2.1,
        15.0, 0.0, 3.14)
    add("Kerbin", "Sun", 3.5316e12, 600e3, 70e3, 13_599_840_256, 0.0, 0.0,
        0.0, 0.0, 3.14)
    add("Mun", "Kerbin", 6.5138398e10, 200e3, 0, 12_000_000, 0.0, 0.0, 0.0,
        0.0, 1.7)
    add("Minmus", "Kerbin", 1.7658e9, 60e3, 0, 47_000_000, 0.0, 6.0, 78.0,
        38.0, 0.9)
    add("Duna", "Sun", 3.0136321e11, 320e3, 50e3, 20_726_155_264, 0.051,
        0.06, 135.5, 0.0, 3.14)
    add("Ike", "Duna", 1.8568369e10, 130e3, 0, 3_200_000, 0.03, 0.2, 0.0,
        0.0, 1.7)
    add("Gilly", "Eve", 8_289_449.8, 13e3, 0, 31_500_000, 0.55, 12.0,
        80.0, 10.0, 0.9)
    add("Dres", "Sun", 2.1484489e10, 138e3, 0, 40_839_348_203, 0.145, 5.0,
        280.0, 90.0, 3.14)
    add("Jool", "Sun", 2.82528e14, 6_000e3, 200e3, 68_773_560_320, 0.05,
        1.304, 52.0, 0.0, 0.1)
    add("Laythe", "Jool", 1.962e12, 500e3, 50e3, 27_184_000, 0.0, 0.0, 0.0,
        0.0, 3.14)
    add("Vall", "Jool", 2.074815e11, 300e3, 0, 43_152_000, 0.0, 0.0, 0.0,
        0.0, 0.9)
    add("Tylo", "Jool", 2.82528e12, 600e3, 0, 68_500_000, 0.0, 0.025, 0.0,
        0.0, 3.14)
    add("Bop", "Jool", 2.4868349e9, 65e3, 0, 128_500_000, 0.235, 15.0, 10.0,
        25.0, 0.9)
    add("Pol", "Jool", 7.2170208e8, 44e3, 0, 179_890_000, 0.171, 4.25, 2.0,
        15.0, 0.9)
    add("Eeloo", "Sun", 7.4410815e10, 210e3, 0, 90_118_820_000, 0.26, 6.15,
        50.0, 260.0, 3.14)
    return bodies


def position_in(body, frame_body, t):
    """``body``'s position relative to ``frame_body`` (an ancestor, or the
    body itself) at UT ``t``."""
    pos = (0.0, 0.0, 0.0)
    b = body
    while b is not frame_body:
        pos = vec.add(pos, b.position(t))
        b = b.parent
    return pos
