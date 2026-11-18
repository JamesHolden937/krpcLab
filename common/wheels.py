"""Wheel settings any pilot may change from the part menu (within the
slider's range): friction control.  ``spaceplane`` keeps its own copy in
``Autopilot._set_friction`` (to be moved here)."""

from common import partmodule


def set_friction(wheel, target):
    """Manual friction at ``target`` (the slider: 0.01-10) on one wheel,
    switching its automatic friction off first; says what happened.
    Friction is a ``ModuleWheelBase`` field kRPC lists only once the auto
    toggle is off, found by "friction" in its localised name."""
    try:
        for module in wheel.part.modules:
            if module.name != "ModuleWheelBase":
                continue

            def field_name():
                for name in partmodule.fields(module):
                    if "friction" in name.lower():
                        return name
                return None
            field = field_name()
            if field is None:
                for event in partmodule.events(module):
                    if "friction" in event.lower():
                        partmodule.trigger_event(module, event)
                        break
                field = field_name()
            if field is None:
                return "friction: no manual field"
            partmodule.set_field_float(module, field,
                                       max(0.01, min(10.0, float(target))))
            return "friction %s" % partmodule.fields(module).get(field, "?")
    except Exception as exc:                            # noqa: BLE001
        return "friction: failed (%s)" % exc
    return "friction: no ModuleWheelBase"
