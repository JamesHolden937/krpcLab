"""Kerbin's atmosphere as KSP computes it.

Pressure is a function of altitude alone.  Temperature is not: KSP adds a
latitude bias and a day/night term to the base curve, scaled by altitude, so
density -- and the speed of sound, hence Mach -- depend on where the vessel is
and where the sun is.  ``FlightIntegrator`` evaluates the sun term against the
local vertical turned 45 degrees about the spin axis, which puts the hottest
air in the afternoon.
"""
import json
import math
import os

from kspSim.curves import FloatCurve

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
GAS_CONSTANT = 8.3144598   # KSP's PhysicsGlobals.IdealGasConstant


class Atmosphere:
    def __init__(self, cfg=None):
        cfg = cfg or json.load(open(os.path.join(DATA, "kerbin.json")))
        self.depth = cfg["atmosphereDepth"]
        self.gamma = cfg["adiabaticIndex"]
        self.molar = cfg["atmosphereMolarMass"]
        self.pressure_curve = FloatCurve(cfg["pressureCurve"])
        self.temperature_curve = FloatCurve(cfg["temperatureCurve"])
        self.sun_mult = FloatCurve(cfg["temperatureSunMultCurve"])
        self.lat_bias = FloatCurve(cfg["temperatureLatitudeBiasCurve"])
        self.lat_sun_mult = FloatCurve(cfg["temperatureLatitudeSunMultCurve"])
        self.axial_bias = FloatCurve(cfg["temperatureAxialSunBiasCurve"])
        self.axial_mult = FloatCurve(cfg["temperatureAxialSunMultCurve"])
        self.ecc_bias = FloatCurve(cfg["temperatureEccentricityBiasCurve"])
        self.sun_lead_deg = 45.0

    def pressure(self, alt):
        """kPa.  Below sea level the curve is held at its first key."""
        if alt >= self.depth:
            return 0.0
        return max(0.0, self.pressure_curve(max(alt, 0.0)))

    def sun_dot(self, position, sun_dir):
        """KSP's day/night fraction at ``position`` (body frame, any
        length), for a unit ``sun_dir`` in the same frame: the vertical is
        projected onto the equator and led 45 degrees about the spin axis.
        Measured against the game's ``TemperatureAt`` to 1e-4."""
        x, z = position[0], position[2]
        n = math.hypot(x, z)
        if n == 0.0:
            return 0.5
        x, z = x / n, z / n
        c = math.cos(math.radians(self.sun_lead_deg))
        s = math.sin(math.radians(self.sun_lead_deg))
        # rotation about +y by the lead, Unity's AngleAxis convention
        lx, lz = c * x + s * z, -s * x + c * z
        return (1.0 + lx * sun_dir[0] + lz * sun_dir[2]) * 0.5

    def temperature_offset(self, lat_deg, sun_dot):
        a = abs(lat_deg)
        return (self.lat_bias(a) + self.lat_sun_mult(a) * sun_dot
                + self.axial_mult(a) * self.axial_bias(0.0) + self.ecc_bias(0.0))

    def temperature(self, alt, lat_deg, sun_dot):
        """sun_dot: (1 + cos(angle between the led vertical and the sun))/2."""
        alt_c = min(max(alt, 0.0), self.depth)
        return (self.temperature_curve(alt_c)
                + self.sun_mult(alt_c) * self.temperature_offset(lat_deg, sun_dot))

    def density(self, pressure_kpa, temperature):
        if pressure_kpa <= 0.0:
            return 0.0
        return pressure_kpa * 1000.0 * self.molar / (GAS_CONSTANT * temperature)

    def sound_speed(self, pressure_kpa, density):
        if density <= 0.0:
            return 0.0
        return math.sqrt(self.gamma * pressure_kpa * 1000.0 / density)
