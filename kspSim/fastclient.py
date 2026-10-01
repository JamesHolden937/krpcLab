"""The krpc Python client, faster under PyPy -- same bytes, same values.

The autopilots run under PyPy against the simulator (``--pypy``) because
their propagators are several times faster there, but PyPy has no C
protobuf: krpc 0.6.0's client builds every request, argument and response
through protobuf's pure-Python runtime and reads each response's length
prefix a byte at a time behind a ``select``.  A qs_entry flight makes
~18,000 ``SimulateAerodynamicForceAt`` calls at 0.42 ms each from PyPy
against 0.17 from CPython -- about seven seconds of a 34-second flight.

``install()`` replaces functions of the installed krpc package, not
anything in an autopilot: ``Encoder.encode`` and ``Decoder.decode`` (per
type, compiled once, through ``kspSim.fastpb``; anything unusual falls
through to the originals), ``Client._invoke`` (the request built and the
response read by hand) and ``Types``' type lookups (remembered).  Aero-force
call 0.35 -> 0.22 ms, a property read 0.05 -> 0.024.  What reaches the wire is byte for byte what the
original client sends (``tests/testPhysics.py``), so a server -- the game's
or the simulator's -- cannot tell the difference.

It is loaded by a ``.pth`` file in ``.venv-pypy`` (``kspSim/tools/pypysetup.sh``)
and only there; set ``KSPSIM_FASTCLIENT=0`` to fly the stock client.
"""
import os
import socket

from kspSim import fastpb

_ORIGINAL = {}


def _compile_encoder(typ, orig):
    from krpc.types import (ValueType, ClassType, EnumerationType, TupleType,
                            ListType, SetType)
    import krpc.schema.KRPC_pb2 as KRPC
    if isinstance(typ, ValueType):
        code = typ.protobuf_type.code
        if code == KRPC.Type.DOUBLE:
            return fastpb.enc_double
        if code == KRPC.Type.FLOAT:
            return fastpb.enc_float
        if code == KRPC.Type.BOOL:
            return lambda v: b"\x01" if v else b"\x00"
        if code in (KRPC.Type.SINT32, KRPC.Type.SINT64):
            return fastpb.enc_zigzag
        if code == KRPC.Type.STRING:
            return fastpb.enc_string
        return None
    if isinstance(typ, ClassType):
        return lambda v: fastpb.varint(v._object_id) if v is not None else b"\x00"
    if isinstance(typ, EnumerationType):
        return lambda v: fastpb.enc_zigzag(v.value)
    if isinstance(typ, TupleType):
        encs = [encoder(t, orig) for t in typ.value_types]
        n = len(encs)

        def tup(v):
            if len(v) != n:
                return orig(v, typ)             # the original's error
            return fastpb.enc_items([f(x) for f, x in zip(encs, v)])
        return tup
    if isinstance(typ, (ListType, SetType)):
        f = encoder(typ.value_type, orig)
        return lambda v: fastpb.enc_items([f(x) for x in v])
    return None


_ENC = {}
_DEC = {}


def encoder(typ, orig):
    f = _ENC.get(typ)
    if f is None:
        f = _compile_encoder(typ, orig) or (lambda v, _t=typ: orig(v, _t))
        _ENC[typ] = f
    return f


def _compile_decoder(typ, orig):
    from krpc.types import (ValueType, ClassType, EnumerationType, TupleType,
                            ListType, SetType)
    import krpc.schema.KRPC_pb2 as KRPC
    if isinstance(typ, ValueType):
        code = typ.protobuf_type.code
        f = {KRPC.Type.DOUBLE: fastpb.dec_double, KRPC.Type.FLOAT: fastpb.dec_float,
             KRPC.Type.SINT32: fastpb.dec_zigzag, KRPC.Type.SINT64: fastpb.dec_zigzag,
             KRPC.Type.UINT32: fastpb.dec_uvarint, KRPC.Type.UINT64: fastpb.dec_uvarint,
             KRPC.Type.BOOL: lambda d: bool(fastpb.dec_uvarint(d)),
             KRPC.Type.STRING: fastpb.dec_string}.get(code)
        return None if f is None else (lambda client, d: f(d))
    if isinstance(typ, ClassType):
        cls = typ.python_type

        def obj(client, d):
            oid = fastpb.dec_uvarint(d)
            return cls(client, oid) if oid != 0 else None
        return obj
    if isinstance(typ, EnumerationType):
        cls = typ.python_type
        return lambda client, d: cls(fastpb.dec_zigzag(d))
    if isinstance(typ, TupleType):
        decs = [decoder(t, orig) for t in typ.value_types]
        return lambda client, d: None if d == b"\x00" else tuple(
            f(client, i) for f, i in zip(decs, fastpb.dec_items(d)))
    if isinstance(typ, ListType):
        f = decoder(typ.value_type, orig)
        return lambda client, d: None if d == b"\x00" else [
            f(client, i) for i in fastpb.dec_items(d)]
    if isinstance(typ, SetType):
        f = decoder(typ.value_type, orig)
        return lambda client, d: None if d == b"\x00" else set(
            f(client, i) for i in fastpb.dec_items(d))
    return None


def decoder(typ, orig):
    f = _DEC.get(typ)
    if f is None:
        f = _compile_decoder(typ, orig) or (lambda client, d, _t=typ: orig(client, d, _t))
        _DEC[typ] = f
    return f


def _read_frame(sock):
    size, shift = 0, 0
    while True:
        b = sock.recv(1)
        if not b:
            raise socket.error("Connection closed")
        b = b[0]
        size |= (b & 0x7F) << shift
        if not b & 0x80:
            break
        shift += 7
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise socket.error("Connection closed")
        data += chunk
    return data


def _parse_response(buf):
    """Response bytes -> (error bytes or None, [(error bytes or None, value)])."""
    error, results = None, []
    pos, end = 0, len(buf)
    while pos < end:
        tag, pos = fastpb.read_varint(buf, pos)
        n, pos = fastpb.read_varint(buf, pos)
        chunk = buf[pos:pos + n]
        pos += n
        if tag == 0x0A:
            error = chunk
        elif tag == 0x12:
            e, v = None, b""
            p, m = 0, len(chunk)
            while p < m:
                t, p = fastpb.read_varint(chunk, p)
                k, p = fastpb.read_varint(chunk, p)
                if t == 0x0A:
                    e = chunk[p:p + k]
                elif t == 0x12:
                    v = chunk[p:p + k]
                p += k
            results.append((e, v))
    return error, results


def install():
    """Patch the installed krpc client (idempotent)."""
    if _ORIGINAL or os.environ.get("KSPSIM_FASTCLIENT") == "0":
        return
    import krpc.client as kclient
    import krpc.schema.KRPC_pb2 as KRPC
    from krpc.decoder import Decoder
    from krpc.encoder import Encoder
    from krpc.event import Event
    from krpc.types import DefaultArgument

    orig_encode = Encoder.encode.__func__
    orig_decode = Decoder.decode.__func__
    _ORIGINAL.update(encode=orig_encode, decode=orig_decode,
                     invoke=kclient.Client._invoke)

    def orig_enc(v, t):
        return orig_encode(Encoder, v, t)

    def orig_dec(client, d, t):
        return orig_decode(Decoder, client, d, t)

    def encode(cls, x, typ):
        return encoder(typ, orig_enc)(x)

    def decode(cls, client, data, typ):
        return decoder(typ, orig_dec)(client, data)

    Encoder.encode = classmethod(encode)
    Decoder.decode = classmethod(decode)

    def _invoke(self, service, procedure, args, param_names, param_types, return_type):
        body = [fastpb._ld(1, service.encode("utf-8")), fastpb._ld(2, procedure.encode("utf-8"))]
        for i, (value, typ) in enumerate(zip(args, param_types)):
            if isinstance(value, DefaultArgument):
                continue
            if not isinstance(value, typ.python_type):
                try:
                    value = self._types.coerce_to(value, typ)
                except ValueError as exc:
                    raise TypeError(
                        "%s.%s() argument %d must be a %s, got a %s"
                        % (service, procedure, i, typ.python_type, type(value))) from exc
            data = encoder(typ, orig_enc)(value)
            arg = (b"\x08" + fastpb.varint(i) if i else b"") + \
                (fastpb._ld(2, data) if data else b"")
            body.append(fastpb._ld(3, arg))
        request = fastpb._ld(1, b"".join(body))
        conn = self._rpc_connection
        with self._rpc_connection_lock:
            conn.send(fastpb.frame(request))
            raw = _read_frame(conn._socket)
        error, results = _parse_response(raw)
        if error is not None:
            raise self._build_error(KRPC.Error.FromString(error))
        err, value = results[0]
        if err is not None:
            raise self._build_error(KRPC.Error.FromString(err))
        result = None
        if return_type is not None:
            result = Decoder.decode(self, value, return_type)
            if isinstance(result, KRPC.Event):
                result = Event(self, result)
        return result

    kclient.Client._invoke = _invoke
    _memoize_types()


def _memoize_types():
    """krpc's generated stubs ask ``Types`` for every parameter's type on
    every call (``class_type("SpaceCenter", "Flight")``, ``double_type``,
    ``tuple_type(...)``), and each answer builds and serialises protobuf
    ``Type`` messages to find a type it already holds: 57,000 of them for
    3,000 aerodynamic-force calls, a third of the call.  The types are
    canonical objects, so the answer for the same arguments is the same
    object; it is remembered per ``Types``."""
    from krpc.types import Types

    def memo(name, orig):
        def method(self, *args, **kwargs):
            cache = self.__dict__.get("_kf_memo")
            if cache is None:
                cache = self.__dict__["_kf_memo"] = {}
            key = (name,) + tuple(a if isinstance(a, str) else id(a) for a in args)
            typ = cache.get(key)
            if typ is None:
                typ = cache[key] = orig(self, *args, **kwargs)
            return typ
        return method

    for name in ("class_type", "enumeration_type", "tuple_type", "list_type", "set_type",
                 "dictionary_type"):
        setattr(Types, name, memo(name, getattr(Types, name)))
    for name, value in list(vars(Types).items()):
        if isinstance(value, property) and name.endswith("_type"):
            setattr(Types, name, property(memo(name, value.fget)))
