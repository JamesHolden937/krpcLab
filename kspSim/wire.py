"""kRPC framing: a protobuf varint length, then the message."""
from google.protobuf.internal import decoder as _dec
from google.protobuf.internal import encoder as _enc


def frame(message):
    body = message.SerializeToString()
    return _enc._VarintBytes(len(body)) + body


def read_frame(sock):
    """One length-delimited message's bytes, or None at end of stream."""
    head = b""
    while True:
        byte = sock.recv(1)
        if not byte:
            return None
        head += byte
        if not byte[0] & 0x80:
            break
    size, _ = _dec._DecodeVarint(head, 0)
    body = b""
    while len(body) < size:
        chunk = sock.recv(size - len(body))
        if not chunk:
            return None
        body += chunk
    return body


def recv(sock, typ):
    body = read_frame(sock)
    if body is None:
        return None
    msg = typ()
    msg.ParseFromString(body)
    return msg


class FrameReader:
    """``read_frame`` over a buffer: one recv per burst, not one per byte of
    the length prefix."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def _fill(self):
        chunk = self.sock.recv(65536)
        if not chunk:
            return False
        self.buf += chunk
        return True

    def read(self):
        """One message's bytes, or None at end of stream."""
        while True:
            buf = self.buf
            size, shift, pos = 0, 0, 0
            complete = False
            while pos < len(buf):
                b = buf[pos]
                size |= (b & 0x7F) << shift
                pos += 1
                if not b & 0x80:
                    complete = True
                    break
                shift += 7
            if complete and len(buf) - pos >= size:
                self.buf = buf[pos + size:]
                return buf[pos:pos + size]
            if not self._fill():
                return None
