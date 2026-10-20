"""Read and edit a KSP save's vessels as text, without loading it.

A save is a ConfigNode tree, tab-indented, one node or ``key = value`` per
line.  Everything here works on the FLIGHTSTATE's top-level ``VESSEL``
blocks (two tabs deep) and leaves every other line byte for byte as it was,
line endings included -- KSP writes CRLF.

    save = Save.read(path)
    for v in save.vessels(): v.name, v.sit, v.persistent_id, v.has_engines
    save.active = 3                      # FLIGHTSTATE's activeVessel index
    save.remove_vessel(pid); save.add_vessel(block)
    save.write(path)
"""

import math

VESSEL = "\t\tVESSEL"
VESSEL_END = "\t\t}"
ACTIVE = "\t\tactiveVessel = "
FLYING = ("FLYING", "SUB_ORBITAL")


class Vessel:
    """One top-level VESSEL block: its lines and the header fields."""

    def __init__(self, lines):
        self.lines = lines
        self.fields = {}
        for line in lines:
            if (line.startswith("\t\t\t") and not line.startswith("\t\t\t\t")
                    and " = " in line):
                key, value = line[3:].split(" = ", 1)
                self.fields.setdefault(key, value)

    name = property(lambda self: self.fields.get("name", ""))
    sit = property(lambda self: self.fields.get("sit", ""))
    type = property(lambda self: self.fields.get("type", ""))
    persistent_id = property(lambda self: self.fields.get("persistentId"))
    pid = property(lambda self: self.fields.get("pid"))

    @property
    def has_engines(self):
        return any(line.strip().startswith("name = ModuleEngines")
                   for line in self.lines)

    def position(self, radius):
        """Cartesian position from lat/lon/alt on a sphere of ``radius``;
        None when the block has no usable surface position."""
        try:
            lat = math.radians(float(self.fields["lat"]))
            lon = math.radians(float(self.fields["lon"]))
            r = radius + float(self.fields["alt"])
        except (KeyError, ValueError):
            return None
        return (r * math.cos(lat) * math.cos(lon),
                r * math.cos(lat) * math.sin(lon), r * math.sin(lat))


class Save:
    def __init__(self, lines, newline):
        self.lines = lines
        self.newline = newline

    @classmethod
    def read(cls, path):
        with open(path, newline="") as fh:
            text = fh.read()
        newline = "\r\n" if "\r\n" in text else "\n"
        return cls(text.split(newline), newline)

    def write(self, path):
        with open(path, "w", newline="") as fh:
            fh.write(self.newline.join(self.lines))

    # -- vessels ----------------------------------------------------------
    def _spans(self):
        out = []
        i = 0
        while i < len(self.lines):
            if self.lines[i] == VESSEL:
                j = i
                while self.lines[j] != VESSEL_END:
                    j += 1
                out.append((i, j + 1))
                i = j
            i += 1
        return out

    def vessels(self):
        return [Vessel(self.lines[s:e]) for s, e in self._spans()]

    def _active_line(self):
        for k, line in enumerate(self.lines):
            if line.startswith(ACTIVE):
                return k
        raise ValueError("save has no activeVessel")

    @property
    def active(self):
        return int(self.lines[self._active_line()][len(ACTIVE):])

    @active.setter
    def active(self, index):
        self.lines[self._active_line()] = ACTIVE + str(int(index))

    def remove_vessel(self, persistent_id):
        """Drop every vessel with this persistentId, keeping activeVessel on
        the vessel it named.  Returns how many were removed."""
        spans = self._spans()
        active = self.active
        removed = 0
        for index in reversed(range(len(spans))):
            s, e = spans[index]
            if Vessel(self.lines[s:e]).persistent_id != persistent_id:
                continue
            if index == active:
                raise ValueError("cannot remove the active vessel")
            del self.lines[s:e]
            removed += 1
            if index < active:
                active -= 1
        self.active = active
        return removed

    def add_vessel(self, vessel):
        """Append a vessel block after the last one (activeVessel unmoved)."""
        spans = self._spans()
        if not spans:
            raise ValueError("save has no VESSEL to add after")
        end = spans[-1][1]
        self.lines[end:end] = list(vessel.lines)
