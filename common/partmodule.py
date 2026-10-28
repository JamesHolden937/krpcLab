"""A part module's fields, events and actions by their in-game names.

kRPC 0.7 removed ``Module.fields`` / ``events`` / ``actions`` (name -> value
dictionaries and lists) and ``has_field``, ``get_field``, ``set_field_*``,
``has_event``, ``trigger_event``, ``has_action`` and ``set_action``.  What
replaces them is ``Module.field_list`` / ``event_list`` / ``action_list``:
lists of ``PartField`` (``gui_name``, ``visible``, ``value`` and typed
setters), ``PartEvent`` (``trigger()``) and ``PartAction`` (``activated``).
These functions keep the old operations, keyed by the name the part menu
shows, on either API: a module with ``field_list`` is 0.7, anything else
(0.6, the offline tests' fakes) is called the old way.

Each list costs a call plus one per element read, so a module's fields are
looked up once and the ``PartField`` kept: setting one in a flight loop is a
single call, as ``set_field_float`` was.
"""

_CACHE = {}


def _key(module):
    return getattr(module, "_object_id", None) or id(module)


def _new(module):
    return hasattr(module, "field_list")


def _table(module, kind):
    """``{gui name: object}`` of a module's visible fields, events or
    actions (``kind`` 'field', 'event', 'action'), cached per module."""
    key = (_key(module), kind)
    got = _CACHE.get(key)
    if got is None:
        items = getattr(module, kind + "_list")
        got = {}
        for item in items:
            visible = True if kind == "action" else bool(item.visible)
            name = item.gui_name or item.name
            if visible and name and name not in got:
                got[name] = item
        _CACHE[key] = got
    return got


def fields(module):
    """``{gui name: value as a string}`` of the visible fields (the values
    read now, not cached)."""
    if not _new(module):
        return dict(module.fields)
    return {name: str(f.value) for name, f in _table(module, "field").items()}


def has_field(module, name):
    if not _new(module):
        return module.has_field(name)
    return name in _table(module, "field")


def get_field(module, name):
    if not _new(module):
        return module.get_field(name)
    return str(_table(module, "field")[name].value)


def set_field_float(module, name, value):
    if not _new(module):
        return module.set_field_float(name, value)
    _table(module, "field")[name].float_value = float(value)


def set_field_int(module, name, value):
    if not _new(module):
        return module.set_field_int(name, value)
    _table(module, "field")[name].int_value = int(value)


def set_field_string(module, name, value):
    if not _new(module):
        return module.set_field_string(name, value)
    _table(module, "field")[name].value = str(value)


def events(module):
    """Names of the events the part menu offers now (re-read: an event
    shows and hides with the part's state)."""
    if not _new(module):
        return list(module.events)
    return [e.gui_name or e.name for e in module.event_list
            if e.visible and e.active]


def has_event(module, name):
    return name in events(module)


def trigger_event(module, name):
    if not _new(module):
        return module.trigger_event(name)
    for e in module.event_list:
        if (e.gui_name or e.name) == name and e.visible and e.active:
            e.trigger()
            return
    raise KeyError(name)


def actions(module):
    if not _new(module):
        return list(module.actions)
    return list(_table(module, "action"))


def has_action(module, name):
    return name in actions(module)


def set_action(module, name, value=True):
    if not _new(module):
        return module.set_action(name, value)
    _table(module, "action")[name].activated = bool(value)
