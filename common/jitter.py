"""How much a re-computed quantity jitters from one tick to the next.

A prediction re-propagated every tick moves by its own noise (a drag
estimate settling, a tick's worth of state) even when nothing real changed.
A deadband smaller than that acts on noise; one larger is a dead zone the
law cannot see into.  So deadbands are made of this.

It is the scatter about the quantity's own trend: second differences
``p[k] - 2 p[k-1] + p[k-2]`` (a steady drift -- a burn walking the
prediction ~750 m a tick, gen-1115 -- has none), ``sd = median|d2| /
sqrt(6)`` over the last ``SAMPLES`` (white noise of sd s gives d2 of sd
s sqrt(6)); the median ignores the occasional real jump (a drag re-sweep, a
re-solve).  None until ``SAMPLES`` are in: an unknown noise is not a small
one.
"""

import math

from common import vec

SAMPLES = 20            # differences the average spans


class Jitter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.prev = []          # the last two values
        self.diffs = []

    def update(self, value):
        """``value`` is a vector (tuple); returns :attr:`sd`."""
        if value is None:
            return self.sd
        value = tuple(value)
        if len(self.prev) == 2:
            a, b = self.prev
            d2 = vec.add(vec.sub(value, vec.scale(b, 2.0)), a)
            self.diffs.append(vec.norm(d2))
            if len(self.diffs) > SAMPLES:
                self.diffs.pop(0)
        self.prev = (self.prev + [value])[-2:]
        return self.sd

    @property
    def count(self):
        return len(self.diffs)

    @property
    def sd(self):
        if len(self.diffs) < SAMPLES:
            return None
        d = sorted(self.diffs)
        return d[len(d) // 2] / math.sqrt(6.0)

    def band(self, sigmas):
        """``sigmas`` standard deviations, or None while unmeasured."""
        sd = self.sd
        return None if sd is None else sigmas * sd
