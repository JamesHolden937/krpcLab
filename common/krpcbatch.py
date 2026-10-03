"""Many kRPC calls in one round trip.

The protocol's ``Request`` carries a *list* of procedure calls and the server
answers them all in one ``Response``; the Python client only ever sends one.
A round trip costs a server frame's attention whatever it carries, so a loop
that asks fourteen questions in a row (a row of the aero table) pays fourteen
waits for what one could answer -- and on a loaded farm the waits are most of
a tick.  The answers also come from the same moment of the game, which
fourteen separate calls do not.

``batch(conn, [(fn, args), ...])`` returns the results in order, with
``None`` for a call the server refused: a missing answer must not look like a
good one, and the caller's per-call ``except: continue`` becomes ``if r is
None: continue``.

Only reads with no side effects belong in one batch; a setter cannot be built
into a call this way (``Client.get_call`` refuses it).
"""


def batch(conn, calls):
    """``calls``: ``(bound_method, args_tuple)`` pairs; results in order."""
    if not calls:
        return []
    # Imported here: the offline tests import this module and have no krpc.
    import krpc.schema.KRPC_pb2 as KRPC
    from krpc.decoder import Decoder
    fast = _fastclient()
    types = []
    if fast is not None:
        # Encode by hand, as ``fastclient`` does for a single call: PyPy's
        # pure-Python protobuf would spend what the batch saves.  The stubs
        # look ``_build_call`` up on the client at call time, so borrowing it
        # yields each call's service, procedure, arguments and types.
        parts = []

        def capture(service, procedure, args, _names, param_types, _ret):
            parts.append(fast._CALL_BYTES[0](
                conn, service, procedure, args, param_types))
        conn._build_call = capture
        try:
            for fn, args in calls:
                conn.get_call(fn, *args)
                types.append(conn._get_return_type(fn, *args))
        finally:
            del conn._build_call
        payload = b"".join(fast.fastpb._ld(1, part) for part in parts)
    else:
        request = KRPC.Request()
        for fn, args in calls:
            request.calls.extend([conn.get_call(fn, *args)])
            types.append(conn._get_return_type(fn, *args))
    with conn._rpc_connection_lock:
        if fast is not None:
            conn._rpc_connection.send(fast.fastpb.frame(payload))
            raw = fast._read_frame(conn._rpc_connection._socket)
        else:
            conn._rpc_connection.send_message(request)
            response = conn._rpc_connection.receive_message(KRPC.Response)
    if fast is not None:
        # PyPy has no C protobuf; ``kspSim.fastclient`` parses by hand.
        error, results = fast._parse_response(raw)
        if error is not None:
            raise conn._build_error(KRPC.Error.FromString(error))
        pairs = results
    else:
        if response.HasField("error"):
            raise conn._build_error(response.error)
        pairs = [(r.error if r.HasField("error") else None, r.value)
                 for r in response.results]
    out = []
    for (err, value), typ in zip(pairs, types):
        if err is not None or typ is None:
            out.append(None)
        else:
            out.append(Decoder.decode(conn, value, typ))
    counter = getattr(conn, "_rpc_counter", None)
    if counter is not None:
        counter.batched(calls)
    return out


def ask(conn, calls, batched=True):
    """``batch`` on a real client, else one call at a time with the same
    ``None``-for-a-refusal contract -- for ``batched=False`` and for the
    offline tests' fakes, which have no wire to put a batch on."""
    if batched and hasattr(conn, "_rpc_connection"):
        return batch(conn, calls)
    out = []
    for fn, args in calls:
        try:
            out.append(fn(*args))
        except Exception:                               # noqa: BLE001
            out.append(None)
    return out


def _fastclient():
    """``kspSim.fastclient`` when it has patched this client, else ``None``."""
    try:
        from kspSim import fastclient
    except ImportError:
        return None
    return fastclient if fastclient._ORIGINAL else None
