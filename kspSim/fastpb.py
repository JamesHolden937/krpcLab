"""kRPC's wire messages without protobuf's Python runtime.

Under PyPy ``google.protobuf`` is its pure-Python implementation, and the
server spent more wall time building, parsing and serialising Request,
Response and StreamUpdate messages than it spent on the physics: a request
round trip cost 0.47 ms against CPython's 0.13 (whose protobuf is C).  The
messages the protocol carries per call are fixed and small (KRPC.proto),
so they are read and written here by hand: the same bytes, byte for byte
(``tests/testPhysics.py`` checks it against protobuf), at a fraction of the
cost on either interpreter.  Everything rare (GetServices, Status, the
connection handshake) still goes through protobuf.

Field numbers (KRPC.proto): Request{calls=1}; ProcedureCall{service=1,
procedure=2, arguments=3, service_id=4, procedure_id=5}; Argument{position=1,
value=2}; Response{error=1, results=2}; ProcedureResult{error=1, value=2};
Error{service=1, name=2, description=3, stack_trace=4};
StreamUpdate{results=1}; StreamResult{id=1, result=2}; Tuple/List/Set
{items=1}; Dictionary{entries=1}; DictionaryEntry{key=1, value=2}.
"""
import struct

_D = struct.Struct("<d")
_F = struct.Struct("<f")


class Call:
    """A ProcedureCall: names, and the arguments as (position, bytes)."""
    __slots__ = ("service", "procedure", "service_id", "procedure_id", "arguments")

    def __init__(self, service="", procedure="", arguments=None, service_id=0,
                 procedure_id=0):
        self.service = service
        self.procedure = procedure
        self.service_id = service_id
        self.procedure_id = procedure_id
        self.arguments = arguments if arguments is not None else []


class Result:
    """A ProcedureResult: the encoded value, or an error
    (service, name, description, stack_trace)."""
    __slots__ = ("value", "error")

    def __init__(self, value=b"", error=None):
        self.value = value
        self.error = error


# -- varints -------------------------------------------------------------

def varint(n):
    """Unsigned varint; a negative value is its 64-bit two's complement."""
    if n < 0:
        n += 1 << 64
    if n < 0x80:
        return bytes((n,))
    out = bytearray()
    while n >= 0x80:
        out.append((n & 0x7F) | 0x80)
        n >>= 7
    out.append(n)
    return bytes(out)


def read_varint(buf, pos):
    b = buf[pos]
    if b < 0x80:
        return b, pos + 1
    result = b & 0x7F
    shift = 7
    pos += 1
    while True:
        b = buf[pos]
        result |= (b & 0x7F) << shift
        pos += 1
        if b < 0x80:
            return result, pos
        shift += 7


def _skip(buf, pos, wire):
    if wire == 0:
        return read_varint(buf, pos)[1]
    if wire == 1:
        return pos + 8
    if wire == 2:
        n, pos = read_varint(buf, pos)
        return pos + n
    if wire == 5:
        return pos + 4
    raise ValueError("wire type %d" % wire)


def _ld(field, data):
    """A length-delimited field: tag, length, bytes."""
    return bytes(((field << 3) | 2,)) + varint(len(data)) + data


# -- messages ------------------------------------------------------------

def parse_call(buf):
    call = Call()
    pos, end = 0, len(buf)
    while pos < end:
        tag, pos = read_varint(buf, pos)
        field, wire = tag >> 3, tag & 7
        if wire == 2:
            n, pos = read_varint(buf, pos)
            chunk = buf[pos:pos + n]
            pos += n
            if field == 1:
                call.service = chunk.decode("utf-8")
            elif field == 2:
                call.procedure = chunk.decode("utf-8")
            elif field == 3:
                call.arguments.append(_parse_argument(chunk))
        elif wire == 0:
            v, pos = read_varint(buf, pos)
            if field == 4:
                call.service_id = v
            elif field == 5:
                call.procedure_id = v
        else:
            pos = _skip(buf, pos, wire)
    return call


def _parse_argument(buf):
    position, value = 0, b""
    pos, end = 0, len(buf)
    while pos < end:
        tag, pos = read_varint(buf, pos)
        field, wire = tag >> 3, tag & 7
        if field == 1 and wire == 0:
            position, pos = read_varint(buf, pos)
        elif field == 2 and wire == 2:
            n, pos = read_varint(buf, pos)
            value = bytes(buf[pos:pos + n])
            pos += n
        else:
            pos = _skip(buf, pos, wire)
    return position, value


def parse_request(buf):
    """Request bytes -> [Call]."""
    calls = []
    pos, end = 0, len(buf)
    while pos < end:
        tag, pos = read_varint(buf, pos)
        field, wire = tag >> 3, tag & 7
        if field == 1 and wire == 2:
            n, pos = read_varint(buf, pos)
            calls.append(parse_call(buf[pos:pos + n]))
            pos += n
        else:
            pos = _skip(buf, pos, wire)
    return calls


def serialize_call(call):
    out = []
    if call.service:
        out.append(_ld(1, call.service.encode("utf-8")))
    if call.procedure:
        out.append(_ld(2, call.procedure.encode("utf-8")))
    for position, value in call.arguments:
        body = (b"\x08" + varint(position) if position else b"") + \
            (_ld(2, value) if value else b"")
        out.append(_ld(3, body))
    if call.service_id:
        out.append(b"\x20" + varint(call.service_id))
    if call.procedure_id:
        out.append(b"\x28" + varint(call.procedure_id))
    return b"".join(out)


def serialize_error(error):
    out = []
    for field, text in enumerate(error, 1):
        if text:
            out.append(_ld(field, text.encode("utf-8")))
    return b"".join(out)


def serialize_result(result):
    """ProcedureResult bytes."""
    if result.error is not None:
        body = _ld(1, serialize_error(result.error))
        return body + _ld(2, result.value) if result.value else body
    return _ld(2, result.value) if result.value else b""


def serialize_response(results):
    """Response bytes from ProcedureResult bytes."""
    return b"".join(_ld(2, r) for r in results)


def serialize_stream_update(items):
    """StreamUpdate bytes from [(stream id, ProcedureResult bytes)]."""
    out = []
    for sid, result in items:
        body = (b"\x08" + varint(sid) if sid else b"") + _ld(2, result)
        out.append(_ld(1, body))
    return b"".join(out)


def frame(data):
    """kRPC framing: a varint length, then the message."""
    return varint(len(data)) + data


# -- values --------------------------------------------------------------

def enc_double(x):
    return _D.pack(x)


def dec_double(b):
    return _D.unpack(b)[0] if len(b) == 8 else 0.0


def enc_float(x):
    return _F.pack(x)


def dec_float(b):
    return _F.unpack(b)[0] if len(b) == 4 else 0.0


def enc_zigzag(n):
    return varint((n << 1) ^ (n >> 63))


def dec_zigzag(b):
    n = read_varint(b, 0)[0] if b else 0
    return (n >> 1) ^ -(n & 1)


def dec_uvarint(b):
    return read_varint(b, 0)[0] if b else 0


def enc_string(s):
    data = s.encode("utf-8")
    return varint(len(data)) + data


def dec_string(b):
    if not b:
        return ""
    n, pos = read_varint(b, 0)
    return bytes(b[pos:pos + n]).decode("utf-8")


def enc_bytes(data):
    return varint(len(data)) + data


def dec_bytes(b):
    if not b:
        return b""
    n, pos = read_varint(b, 0)
    return bytes(b[pos:pos + n])


def enc_items(items):
    """Tuple / List / Set bytes: repeated bytes items = 1."""
    return b"".join(_ld(1, i) for i in items)


def dec_items(buf):
    items = []
    pos, end = 0, len(buf)
    while pos < end:
        tag, pos = read_varint(buf, pos)
        if tag == 0x0A:
            n, pos = read_varint(buf, pos)
            items.append(bytes(buf[pos:pos + n]))
            pos += n
        else:
            pos = _skip(buf, pos, tag & 7)
    return items


def enc_dict(pairs):
    """Dictionary bytes from [(key bytes, value bytes)]."""
    out = []
    for k, v in pairs:
        body = (_ld(1, k) if k else b"") + (_ld(2, v) if v else b"")
        out.append(_ld(1, body))
    return b"".join(out)


def dec_dict(buf):
    pairs = []
    for entry in dec_items(buf):
        k = v = b""
        pos, end = 0, len(entry)
        while pos < end:
            tag, pos = read_varint(entry, pos)
            if tag in (0x0A, 0x12):
                n, pos = read_varint(entry, pos)
                if tag == 0x0A:
                    k = bytes(entry[pos:pos + n])
                else:
                    v = bytes(entry[pos:pos + n])
                pos += n
            else:
                pos = _skip(entry, pos, tag & 7)
        pairs.append((k, v))
    return pairs
