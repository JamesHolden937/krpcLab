#!/usr/bin/env python3
"""ctrlsrf.py <rpc_port> <stream_port> -- can the control surfaces be deployed?

**The Shuttle's speedbrake is a split rudder**, and this airframe carries four
``smallCtrlSrf``, two of them mounted as vertical stabilisers.  Deploying that
pair in opposite directions is drag with no net yaw -- an airbrake made of
hardware the vehicle already has, on an autopilot whose only dissipation
controls are bank and the landing gear.

Before any of that can be written, one thing has to be checked rather than
assumed: this install deliberately keeps four mods that **replace the
spaceplane's control-surface modules** (CLAUDE.md), and the save shows
``SyncModuleControlSurface`` where stock would have ``ModuleControlSurface``.
kRPC's typed ``ControlSurface`` wrapper looks for the stock module, so
``vessel.parts.control_surfaces`` may come back empty on a vehicle covered in
control surfaces.  If it does, the generic ``Module`` interface is the way in
-- the same route ``Autopilot.log_drain_module`` already uses for the release
valve.

Read-only: it reports, it does not deploy anything.
"""
import sys
sys.path.insert(0, "..")
import krpc

rpc, stream = int(sys.argv[1]), int(sys.argv[2])
conn = krpc.connect(name="ctrlsrf", address="127.0.0.1",
                    rpc_port=rpc, stream_port=stream)
vessel = conn.space_center.active_vessel
print("vessel: %s, %d parts" % (vessel.name, len(vessel.parts.all)))

try:
    surfaces = vessel.parts.control_surfaces
    print("\nkRPC typed ControlSurface: %d found" % len(surfaces))
    for cs in surfaces:
        bits = []
        for attr in ("deployed", "inverted", "pitch_enabled", "yaw_enabled",
                     "roll_enabled", "surface_area", "authority_limiter"):
            try:
                bits.append("%s=%s" % (attr, getattr(cs, attr)))
            except Exception as exc:                        # noqa: BLE001
                bits.append("%s=<%s>" % (attr, type(exc).__name__))
        print("   %-24s %s" % (cs.part.title, " ".join(bits)))
except Exception as exc:                                    # noqa: BLE001
    print("\nkRPC typed ControlSurface unavailable: %r" % (exc,))

print("\nwhich pair is the rudder, by what the game says they do:")
try:
    surfaces = vessel.parts.control_surfaces
except Exception:                                           # noqa: BLE001
    surfaces = []
verticals = []
for cs in surfaces:
    try:
        if cs.yaw_enabled and not cs.pitch_enabled:
            verticals.append(cs)
    except Exception:                                       # noqa: BLE001
        pass
print("  yaw-only surfaces: %d" % len(verticals))
for cs in verticals:
    try:
        pos = cs.part.position(vessel.reference_frame)
        print("     %-22s x=%+.2f y=%+.2f z=%+.2f  area=%s"
              % (cs.part.title, pos[0], pos[1], pos[2],
                 getattr(cs, "surface_area", "?")))
    except Exception as exc:                                # noqa: BLE001
        print("     %-22s <%s>" % (cs.part.title, type(exc).__name__))
if len(verticals) == 2:
    try:
        a = verticals[0].part.position(vessel.reference_frame)
        b = verticals[1].part.position(vessel.reference_frame)
        print("  mirrored about the centreline? x sum = %+.3f m "
              "(near zero means splitting them cancels net yaw)"
              % (a[0] + b[0]))
    except Exception:                                       # noqa: BLE001
        pass
else:
    print("  NOT a clean pair -- the brake must refuse to arm rather than "
          "guess, or it yaws the vehicle on final (failure 34)")

print("\nhow much deflection is there to share between brake and yaw?")
for cs in surfaces[:8]:
    bits = []
    for attr in ("deployed", "inverted", "authority_limiter", "surface_area"):
        try:
            bits.append("%s=%s" % (attr, getattr(cs, attr)))
        except Exception:                                   # noqa: BLE001
            bits.append("%s=<none>" % attr)
    print("   %-22s %s" % (cs.part.title, " ".join(bits)))

print("\nmodules on every part that looks like a control surface:")
for part in vessel.parts.all:
    names = [m.name for m in part.modules]
    if not any("ControlSurface" in n or "Aero" in n for n in names):
        continue
    print("  %s  [%s]" % (part.title, part.name))
    for m in part.modules:
        if "ControlSurface" not in m.name and "Aero" not in m.name:
            continue
        try:
            fields = m.fields
        except Exception:                                   # noqa: BLE001
            fields = {}
        try:
            actions = m.actions
        except Exception:                                   # noqa: BLE001
            actions = []
        print("     %s" % m.name)
        print("       fields:  %s" % fields)
        print("       actions: %s" % actions)
