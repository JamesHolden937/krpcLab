"""kRPC's type system, server side.

The procedure signatures come from the game itself: ``data/services.bin`` is the
``KRPC.GetServices`` response recorded from a real instance, served back
verbatim (plus the ``Sim`` service below).  So a client cannot tell the stubs
apart, and every argument is decoded with the type the real server declared.

Class instances cross the wire as uint64 ids.  ``Registry`` hands them out;
an id of 0 is null.  Enumerations are sint32.
"""
import os

import krpc.schema.KRPC_pb2 as KRPC
from kspSim import fastpb
from krpc.decoder import Decoder
from krpc.encoder import Encoder
from krpc.types import (Types, ValueType, ClassType, EnumerationType,
                        MessageType, TupleType, ListType, SetType,
                        DictionaryType)

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
_TYPES = Types()


class Registry:
    """Object ids for everything a client may hold a handle to."""

    def __init__(self):
        self._by_id = {}
        self._next = 1

    def id_of(self, obj):
        oid = getattr(obj, "_object_id", None)
        if oid is None:
            oid = self._next
            self._next += 1
            obj._object_id = oid
            self._by_id[oid] = obj
        return oid

    def get(self, oid):
        return self._by_id.get(oid)


def _sim_service():
    """The one service the game does not have: lock-step time.

    ``AdvanceTo(ut)`` runs the physics to ``ut`` and returns the stream
    generation that carries the new state; ``Generation`` is streamed so a
    client can wait until its cached streams are at least that fresh.
    """
    svc = KRPC.Service(name="Sim")
    dbl = KRPC.Type(code=KRPC.Type.DOUBLE)
    u64 = KRPC.Type(code=KRPC.Type.UINT64)
    p = svc.procedures.add(name="AdvanceTo")
    p.parameters.add(name="ut", type=dbl)
    p.return_type.CopyFrom(u64)
    p = svc.procedures.add(name="get_Generation")
    p.return_type.CopyFrom(u64)
    p = svc.procedures.add(name="get_LatencyScale")
    p.return_type.CopyFrom(dbl)
    p = svc.procedures.add(name="set_LatencyScale")
    p.parameters.add(name="value", type=dbl)
    return svc


class Signatures:
    def __init__(self):
        raw = open(os.path.join(DATA, "services.bin"), "rb").read()
        services = KRPC.Services()
        services.ParseFromString(raw)
        services.services.append(_sim_service())
        self.services_message = services
        self.procs = {}
        for svc in services.services:
            for proc in svc.procedures:
                params = [_TYPES.as_type(p.type) for p in proc.parameters]
                defaults = [p.default_value if p.default_value else None
                            for p in proc.parameters]
                ret = (_TYPES.as_type(proc.return_type)
                       if proc.return_type.code != KRPC.Type.NONE
                       else None)
                self.procs[(svc.name, proc.name)] = (params, defaults, ret)

        # Each procedure's codecs, compiled once from its types (``decoder`` /
        # ``encoder``): the per-call path does no type dispatch at all.
        self.codecs = {}
        for key, (params, defaults, ret) in self.procs.items():
            self.codecs[key] = ([decoder(t) for t in params], defaults,
                                encoder(ret) if ret is not None else None)

    def decode_args(self, service, procedure, arguments, registry):
        """``arguments``: [(position, bytes)] (``fastpb.Call.arguments``)."""
        decs, defaults, _ = self.codecs[(service, procedure)]
        values = [None] * len(decs)
        given = 0
        for position, data in arguments:
            values[position] = decs[position](data, registry)
            given |= 1 << position
        for i, d in enumerate(defaults):
            if d is not None and not given & (1 << i):
                values[i] = decs[i](d, registry)
        return values

    def encode_result(self, service, procedure, value):
        enc = self.codecs[(service, procedure)][2]
        return b"" if enc is None else enc(value)


def decoder(typ):
    """A function (bytes, registry) -> value for kRPC type ``typ``: the same
    values ``decode`` gives, through ``fastpb``."""
    if isinstance(typ, ClassType):
        return lambda d, reg: reg.get(fastpb.dec_uvarint(d)) if d else None
    if isinstance(typ, EnumerationType):
        return lambda d, reg: fastpb.dec_zigzag(d)
    if isinstance(typ, ValueType):
        code = typ.protobuf_type.code
        f = {KRPC.Type.DOUBLE: fastpb.dec_double, KRPC.Type.FLOAT: fastpb.dec_float,
             KRPC.Type.SINT32: fastpb.dec_zigzag, KRPC.Type.SINT64: fastpb.dec_zigzag,
             KRPC.Type.UINT32: fastpb.dec_uvarint, KRPC.Type.UINT64: fastpb.dec_uvarint,
             KRPC.Type.BOOL: lambda d: bool(fastpb.dec_uvarint(d)),
             KRPC.Type.STRING: fastpb.dec_string, KRPC.Type.BYTES: fastpb.dec_bytes}[code]
        return lambda d, reg: f(d)
    if isinstance(typ, MessageType):
        if typ.python_type is KRPC.ProcedureCall:
            return lambda d, reg: fastpb.parse_call(d)
        return lambda d, reg: Decoder.decode_message(d, typ.python_type)
    if isinstance(typ, TupleType):
        decs = [decoder(t) for t in typ.value_types]
        return lambda d, reg: tuple(f(i, reg) for i, f in zip(fastpb.dec_items(d), decs))
    if isinstance(typ, (ListType, SetType)):
        f = decoder(typ.value_type)
        return lambda d, reg: [f(i, reg) for i in fastpb.dec_items(d)]
    if isinstance(typ, DictionaryType):
        fk, fv = decoder(typ.key_type), decoder(typ.value_type)
        return lambda d, reg: {fk(k, reg): fv(v, reg) for k, v in fastpb.dec_dict(d)}
    raise TypeError("cannot decode %s" % typ)


def encoder(typ):
    """A function value -> bytes for kRPC type ``typ``: the same bytes
    ``encode`` gives, through ``fastpb``."""
    if isinstance(typ, ClassType):
        return lambda v: fastpb.varint(v._object_id) if v is not None else b"\x00"
    if isinstance(typ, EnumerationType):
        return lambda v: fastpb.enc_zigzag(int(v))
    if isinstance(typ, ValueType):
        code = typ.protobuf_type.code
        if code == KRPC.Type.DOUBLE:
            return lambda v: fastpb.enc_double(float(v))
        if code == KRPC.Type.FLOAT:
            return lambda v: fastpb.enc_float(float(v))
        if code == KRPC.Type.BOOL:
            return lambda v: b"\x01" if v else b"\x00"
        if code in (KRPC.Type.SINT32, KRPC.Type.SINT64):
            return lambda v: fastpb.enc_zigzag(int(v))
        if code in (KRPC.Type.UINT32, KRPC.Type.UINT64):
            def uint(v):
                if v < 0:
                    raise ValueError("Value must be non-negative, got %d" % v)
                return fastpb.varint(int(v))
            return uint
        if code == KRPC.Type.STRING:
            return fastpb.enc_string
        if code == KRPC.Type.BYTES:
            return fastpb.enc_bytes
        raise TypeError("cannot encode %s" % typ)
    if isinstance(typ, MessageType):
        return lambda v: v.SerializeToString()
    if isinstance(typ, TupleType):
        encs = [encoder(t) for t in typ.value_types]
        return lambda v: fastpb.enc_items([f(x) for x, f in zip(v, encs)])
    if isinstance(typ, (ListType, SetType)):
        f = encoder(typ.value_type)
        return lambda v: fastpb.enc_items([f(x) for x in v])
    if isinstance(typ, DictionaryType):
        fk, fv = encoder(typ.key_type), encoder(typ.value_type)
        return lambda v: fastpb.enc_dict([(fk(k), fv(v[k])) for k in sorted(v)])
    raise TypeError("cannot encode %s" % typ)


# The reference codec, through protobuf: what ``decoder``/``encoder`` must
# agree with byte for byte (tests/testPhysics.py).

def decode(data, typ, registry):
    if isinstance(typ, ClassType):
        oid = Decoder._decode_value(data, _TYPES.uint64_type)
        return registry.get(oid) if oid else None
    if isinstance(typ, EnumerationType):
        return Decoder._decode_value(data, _TYPES.sint32_type)
    if isinstance(typ, ValueType):
        return Decoder._decode_value(data, typ)
    if isinstance(typ, MessageType):
        return Decoder.decode_message(data, typ.python_type)
    if isinstance(typ, TupleType):
        msg = KRPC.Tuple()
        msg.ParseFromString(data)
        return tuple(decode(i, t, registry) for i, t in zip(msg.items, typ.value_types))
    if isinstance(typ, (ListType, SetType)):
        msg = KRPC.List() if isinstance(typ, ListType) else KRPC.Set()
        msg.ParseFromString(data)
        return [decode(i, typ.value_type, registry) for i in msg.items]
    if isinstance(typ, DictionaryType):
        msg = KRPC.Dictionary()
        msg.ParseFromString(data)
        return {decode(e.key, typ.key_type, registry):
                decode(e.value, typ.value_type, registry) for e in msg.entries}
    raise TypeError("cannot decode %s" % typ)


def encode(value, typ):
    if isinstance(typ, ClassType):
        oid = value._object_id if value is not None else 0
        return Encoder._encode_value(oid, _TYPES.uint64_type)
    if isinstance(typ, EnumerationType):
        return Encoder._encode_value(int(value), _TYPES.sint32_type)
    if isinstance(typ, ValueType):
        if typ.protobuf_type.code in (KRPC.Type.DOUBLE, KRPC.Type.FLOAT):
            value = float(value)
        elif typ.protobuf_type.code == KRPC.Type.BOOL:
            value = bool(value)
        return Encoder._encode_value(value, typ)
    if isinstance(typ, MessageType):
        return value.SerializeToString()
    if isinstance(typ, TupleType):
        msg = KRPC.Tuple()
        msg.items.extend(encode(v, t) for v, t in zip(value, typ.value_types))
        return msg.SerializeToString()
    if isinstance(typ, ListType):
        msg = KRPC.List()
        msg.items.extend(encode(v, typ.value_type) for v in value)
        return msg.SerializeToString()
    if isinstance(typ, SetType):
        msg = KRPC.Set()
        msg.items.extend(encode(v, typ.value_type) for v in value)
        return msg.SerializeToString()
    if isinstance(typ, DictionaryType):
        msg = KRPC.Dictionary()
        for k in sorted(value):
            e = msg.entries.add()
            e.key = encode(k, typ.key_type)
            e.value = encode(value[k], typ.value_type)
        return msg.SerializeToString()
    raise TypeError("cannot encode %s" % typ)
