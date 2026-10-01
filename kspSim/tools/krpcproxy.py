#!/usr/bin/env python3
"""Record every kRPC call between a client and a real game.

    ./kspSim/tools/krpcproxy.py --instance 0 --listen 50300 -o trace.jsonl

Clients connect to --listen (rpc) and --listen+1 (stream).  Each line of the
trace is one request/response pair: wall time, the calls (service,
procedure, argument bytes) and the results, base64.  The simulator's API
surface and its conformance tests both come from these traces.
"""
import argparse
import base64
import json
import os
import socket
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths  # noqa: E402
paths.use_venv()

import krpc.schema.KRPC_pb2 as KRPC  # noqa: E402
from kspSim import wire  # noqa: E402

LOCK = threading.Lock()


def b64(data):
    return base64.b64encode(data).decode()


def pipe_rpc(client, upstream, out, tag):
    try:
        req = wire.recv(client, KRPC.ConnectionRequest)
        upstream.sendall(wire.frame(req))
        resp = wire.recv(upstream, KRPC.ConnectionResponse)
        client.sendall(wire.frame(resp))
        with LOCK:
            out.write(json.dumps({"t": time.time(), "conn": tag,
                                  "hello": req.client_name,
                                  "id": b64(resp.client_identifier)}) + "\n")
        while True:
            body = wire.read_frame(client)
            if body is None:
                return
            t0 = time.time()
            upstream.sendall(wire._enc._VarintBytes(len(body)) + body)
            rbody = wire.read_frame(upstream)
            if rbody is None:
                return
            t1 = time.time()
            client.sendall(wire._enc._VarintBytes(len(rbody)) + rbody)
            req = KRPC.Request()
            req.ParseFromString(body)
            resp = KRPC.Response()
            resp.ParseFromString(rbody)
            calls = []
            for i, c in enumerate(req.calls):
                r = resp.results[i] if i < len(resp.results) else None
                calls.append({
                    "s": c.service, "p": c.procedure,
                    "a": [b64(a.value) for a in c.arguments],
                    "ap": [a.position for a in c.arguments],
                    "r": b64(r.value) if r is not None else None,
                    "e": (r.error.description[:300]
                          if r is not None and r.HasField("error") else None)})
            with LOCK:
                out.write(json.dumps({"t": t0, "dt": t1 - t0, "conn": tag,
                                      "calls": calls}) + "\n")
    except OSError:
        return
    finally:
        client.close()
        upstream.close()


def pipe_stream(client, upstream, out, tag, record):
    try:
        req = wire.recv(client, KRPC.ConnectionRequest)
        upstream.sendall(wire.frame(req))
        resp = wire.recv(upstream, KRPC.ConnectionResponse)
        client.sendall(wire.frame(resp))
        while True:
            body = wire.read_frame(upstream)
            if body is None:
                return
            client.sendall(wire._enc._VarintBytes(len(body)) + body)
            if record:
                upd = KRPC.StreamUpdate()
                upd.ParseFromString(body)
                with LOCK:
                    out.write(json.dumps({"t": time.time(), "conn": tag,
                                          "stream": [[r.id, b64(r.result.value)]
                                                     for r in upd.results]}) + "\n")
    except OSError:
        return
    finally:
        client.close()
        upstream.close()


def serve(port, target, handler, out, *extra):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(16)
    n = 0
    while True:
        c, _ = srv.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        u = socket.create_connection(("127.0.0.1", target))
        u.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        n += 1
        threading.Thread(target=handler, args=(c, u, out, "%d:%d" % (port, n)) + extra,
                         daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--listen", type=int, default=50300)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--streams", action="store_true", help="record stream updates too")
    args = ap.parse_args()
    base = os.path.join(ROOT, "testInstances", "ksp%d" % args.instance)
    rpc = int(open(os.path.join(base, ".rpc_port")).read())
    stream = int(open(os.path.join(base, ".stream_port")).read())
    out = open(args.out, "a", buffering=1)
    threading.Thread(target=serve, args=(args.listen + 1, stream, pipe_stream, out,
                                         args.streams), daemon=True).start()
    serve(args.listen, rpc, pipe_rpc, out)


if __name__ == "__main__":
    main()
