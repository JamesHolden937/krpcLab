"""An autopilot's in-game panel, drawn by the krpcLab window mod
(``launcherMod/``, kRPC service ``KrpcLabLauncher``): a title in the
autopilot's accent colour, a block of telemetry text and its controls --
START and TERMINATE, and whatever buttons, checkboxes, number fields and
choices the autopilot adds.  The mod is krpcLab's only UI: the launcher's
menu is the same window, so pressing a launcher button reads as the window
switching from the menu to the flight.

The controls are described to the mod as strings and read back through one
kRPC stream of ``PanelRead`` ("id=value" per control: a button's click
count, a checkbox 1/0, a field's text, a choice's index), so polling them
costs no call.  The service is reached by name strings because the
bundle's minifier knows only the services in the game's
``GameData/kRPC/*.json``.

A game without the mod (``kspSim``, the offline fakes without one) gets a
headless panel: nothing is drawn, START is never pressed (fly it with
``--autostart``), TERMINATE never, and every control reads its default.
"""

SERVICE = "krpc_lab_launcher"

# Each autopilot's title colour.
ACCENT_BOOSTER = (1.0, 0.75, 0.2)
ACCENT_SPACEPLANE = (0.4, 0.8, 1.0)
ACCENT_PROP = (0.5, 1.0, 0.5)
ACCENT_AIRPLANE = (0.9, 0.6, 1.0)

START = "start"
TERMINATE = "terminate"


def service(conn):
    """The window mod's kRPC service, or None when the game has no mod."""
    return getattr(conn, SERVICE, None)


def expect_scene_change(conn, seconds=30.0):
    """Tell the launcher that a save load or vessel switch in the next
    ``seconds`` is this autopilot's own doing, so it is not terminated for
    it (the launcher stops an autopilot on any other)."""
    svc = service(conn)
    if svc is None:
        return
    try:
        getattr(svc, "expect_scene_change")(float(seconds))
    except Exception:                                   # noqa: BLE001
        pass


def button(cid, label):
    return ("button", cid, label)


def toggle(cid, label, on=False):
    return ("toggle", cid, label, bool(on))


def field(cid, label, value):
    return ("field", cid, label, str(value))


def choice(cid, label, options, index=0):
    return ("choice", cid, label, int(index), tuple(options))


def note(text):
    return ("label", "", text)


def _spec(c):
    kind, cid, label = c[0], c[1], c[2]
    for s in (cid, label):
        if "|" in s:
            raise ValueError("'|' in a panel control: %r" % (c,))
    if kind == "toggle":
        return "toggle|%s|%s|%d" % (cid, label, 1 if c[3] else 0)
    if kind == "field":
        return "field|%s|%s|%s" % (cid, label, c[3])
    if kind == "choice":
        return "choice|%s|%s|%d|%s" % (cid, label, c[3], ";".join(c[4]))
    return "%s|%s|%s" % (kind, cid, label)


def _default(c):
    kind = c[0]
    if kind == "toggle":
        return "1" if c[3] else "0"
    if kind in ("field",):
        return c[3]
    if kind == "choice":
        return str(c[3])
    return "0"


class ControlPanel:
    """START and TERMINATE under a title in the autopilot's ``accent``
    colour, a body of telemetry lines, and ``controls`` (built with
    ``toggle``/``field``/``choice``/``button``/``note``) above them.
    ``draw=False`` is the headless panel (an autopilot run inside another
    one, whose panel is the one drawn)."""

    def __init__(self, conn, title, start_label, accent, controls=(),
                 draw=True):
        self.conn = conn
        self.svc = service(conn) if draw else None
        self.start_label = start_label
        self.start_shown = True
        self.controls = list(controls)
        self.seen = {}
        self.body = None
        self.sent = None
        self.stream = None
        if self.svc is None:
            return
        self._body = getattr(self.svc, "panel_body")
        self._controls = getattr(self.svc, "panel_controls")
        self._close = getattr(self.svc, "panel_close")
        self.sent = self._specs()
        getattr(self.svc, "panel_open")(
            title, " ".join("%.3f" % x for x in accent), self.sent)
        self.stream = conn.add_stream(getattr(self.svc, "panel_read"))
        # Clicks before this panel opened are not presses.
        for cid, value in self._values().items():
            self.seen[cid] = value

    def _specs(self):
        out = [_spec(c) for c in self.controls]
        if self.start_shown:
            out.append(_spec(button(START, self.start_label)))
        out.append(_spec(button(TERMINATE, "TERMINATE SCRIPT")))
        return out

    def _send(self):
        if self.svc is None:
            return
        specs = self._specs()
        if specs != self.sent:
            self._controls(specs)
            self.sent = specs

    def _values(self):
        if self.stream is None:
            return {}
        try:
            got = self.stream()
        except Exception:                               # noqa: BLE001
            return {}
        out = {}
        for item in got or ():
            k, _, v = item.partition("=")
            out[k] = v
        return out

    # -- input -------------------------------------------------------------
    def pressed(self, cid):
        """Whether button ``cid`` was clicked since the last call."""
        values = self._values()
        if cid not in values:
            return False
        try:
            count = int(values[cid])
        except ValueError:
            return False
        last = int(self.seen.get(cid, count))
        self.seen[cid] = count
        return count > last

    def value(self, cid):
        """A control's value: a checkbox's bool, a field's text, a choice's
        index; its default when the player has not set it (or no mod)."""
        spec = next((c for c in self.controls if c[1] == cid), None)
        if spec is None:
            raise KeyError(cid)
        raw = self._values().get(cid, _default(spec))
        if spec[0] == "toggle":
            return raw == "1"
        if spec[0] == "choice":
            try:
                return int(raw)
            except ValueError:
                return spec[3]
        return raw

    def number(self, cid, default):
        """A field's value as a float, ``default`` when it does not parse."""
        try:
            return float(self.value(cid))
        except (TypeError, ValueError):
            return default

    def start_pressed(self):
        return self.pressed(START)

    def terminate_pressed(self):
        return self.pressed(TERMINATE)

    def hide_start(self):
        """``--autostart``, or started: the flight is running, so no START."""
        self.start_shown = False
        self._send()

    def set_start_label(self, text):
        self.start_label = text
        if self.start_shown:
            self._send()

    def set_controls(self, controls):
        self.controls = list(controls)
        self._send()

    # -- output ------------------------------------------------------------
    def update(self, lines):
        if self.svc is None:
            return
        body = "\n".join(lines)
        if body != self.body:
            self._body(body)
            self.body = body

    def close(self):
        if self.stream is not None:
            try:
                self.stream.remove()
            except Exception:                           # noqa: BLE001
                pass
        if self.svc is not None:
            try:
                self._close()
            except Exception:                           # noqa: BLE001
                pass
