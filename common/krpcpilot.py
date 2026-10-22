"""kRPC's attitude autopilot, driven the same way by every pilot: commanded
by a direction and the vector to roll the roof toward, and engaged without
letting a KSP-side exception end the flight."""


def set_autopilot_engaged(autopilot, engaged):
    """Engage or disengage, returning the exception instead of raising it.

    kRPC has used both an ``engaged`` property and engage/disengage(), which
    is the AttributeError below.  The outer catch is for KSP itself: engaging
    the autopilot makes kRPC total up the vessel's available torque, and
    ``ModuleGimbal.GetPotentialTorque`` throws ``IndexOutOfRange`` on an
    engine whose gimbal is not initialised yet -- which is exactly when this
    script connects, moments after staging.  LOG10 died there on the first
    tick after START, before the throttle had ever been touched.

    So it is reported, not raised.  Losing the attitude command is bad;
    losing the flight because the game was not ready to answer a question is
    worse, and the answer changes a second later.  The caller retries.
    """
    try:
        try:
            autopilot.engaged = engaged
        except AttributeError:
            (autopilot.engage if engaged else autopilot.disengage)()
        return None
    except Exception as exc:            # noqa: BLE001 -- KSP-side failure
        return exc


def set_autopilot_attitude(autopilot, direction, up=None, roll=0.0):
    """Command a nose direction and the vector to roll the roof toward.

    ``set_direction_and_up`` is the singularity-free form; older servers have
    only ``up_reference``/``target_roll``, and older ones no roll control.
    Fall back rather than fail.  Roll matters more here than on a booster:
    bank *is* the control, so a vehicle that will not hold a commanded roll
    has no cross-track authority at all.
    """
    autopilot.target_direction = tuple(direction)
    if up is None:
        try:
            autopilot.target_roll = float("nan")
        except AttributeError:
            pass
        return
    try:
        autopilot.set_direction_and_up(tuple(direction), tuple(up), roll)
        return
    except AttributeError:
        pass
    try:
        autopilot.up_reference = tuple(up)
        autopilot.target_roll = roll
    except AttributeError:
        pass


def vessel_gone(exc):
    """Is ``exc`` kRPC saying the vessel no longer exists?  0.7 raises
    ``ObjectDestroyedException`` on the next call after a vessel is
    destroyed (0.6 froze its streams instead), so a crash ends the flight
    with that exception rather than with the pilot's own destroyed check.
    By name: the class lives in a generated service module."""
    return type(exc).__name__ == "ObjectDestroyedException"


def wobble_mitigation(autopilot, on, log=None, ut=0.0):
    """kRPC 0.7's structural-oscillation mitigations (bandwidth floor,
    feedforward cut, output smoothing, rate-feedback filters) to their
    automatic default (``on``) or off.  They exist for a flexible vessel's
    wobble; whether they mistake an airframe's aerodynamic oscillation for
    one is what turning them off measures.  A server without them (0.6) is
    left alone and says so."""
    names = ("oscillation_bandwidth_floor_mode", "oscillation_feedforward_mode",
             "oscillation_output_filter_mode", "pitch_yaw_rate_filter_mode",
             "roll_rate_filter_mode")
    try:
        for name in names:
            enum = type(getattr(autopilot, name))
            setattr(autopilot, name, enum.automatic if on else enum.off)
    except Exception as exc:                                # noqa: BLE001
        if log is not None:
            log.event(ut, "kRPC wobble mitigation not set (%r)" % (exc,))
        return False
    if log is not None:
        log.event(ut, "kRPC wobble mitigation %s"
                  % ("automatic (kRPC's default)" if on else "off"))
    return True
