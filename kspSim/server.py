"""A kRPC server in front of the simulated world.

Speaks the real protocol on two ports, like the game: requests on one, stream
updates pushed on the other.  Procedures are dispatched to ``api.HANDLERS``;
anything not implemented comes back as an RPC error *and* is appended to
``logs/kspsim-missing.txt``, so a new autopilot's first flight lists what it
still needs instead of failing mysteriously.

Time.  The world only moves when something moves it:

- ``Sim.AdvanceTo(ut)`` (lock-step): runs the physics to ``ut`` at once.
  ``common.pacing`` uses it when connected here, so a wait costs no wall
  time at all.
- the real-time flow: while unpaused, game time also advances at
  ``latency_scale`` times the wall clock -- the time scale a real game would
  be running at.  That is what makes the autopilot's own compute cost game
  time, as it does in KSP, and it is what an unmodified client that sleeps
  sees.  The spaceplane's governor writes the scale it wants to
  ``timescale.txt`` in the instance directory; the flow follows it, so a
  flight here pays the same latency the farm would have made it pay.
"""
import os
import socket
import threading
import time
import traceback
import uuid

import krpc.schema.KRPC_pb2 as KRPC

from kspSim import fastpb, protocol, wire

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ClientState:
    def __init__(self, name):
        self.name = name
        self.ident = uuid.uuid4().bytes
        self.streams = {}          # id -> [ProcedureCall, started, last bytes]
        self.stream_sock = None
        self.send_lock = threading.Lock()
        self.connected = True


class Server:
    def __init__(self, world, api, rpc_port, stream_port, instance_dir=None,
                 latency_scale=1.0, address="127.0.0.1"):
        self.world = world
        self.api = api
        self.sig = protocol.Signatures()
        self.registry = protocol.Registry()
        self.lock = threading.RLock()
        self.clients = {}
        self.next_stream = 1
        self.stream_calls = {}     # (client ident, serialized call) -> id
        self.generation = 0
        self.latency_scale = latency_scale
        self.instance_dir = instance_dir
        self.rpc_port = rpc_port
        self.stream_port = stream_port
        self.address = address
        self.missing = set()
        self._stop = threading.Event()
        # Where the wall time goes: (service, procedure) -> [calls, seconds],
        # plus physics and stream pushes; written to the log per client.
        self.stats = {}
        self._ts_checked = 0.0
        self._ts_mtime = None
        self._loaded_at = 0.0

    # -- time -------------------------------------------------------------

    def _read_timescale(self):
        """Follow the governor's file, like the farm plugin does."""
        if not self.instance_dir:
            return
        now = time.monotonic()
        if now - self._ts_checked < 0.25:
            return
        self._ts_checked = now
        path = os.path.join(self.instance_dir, "timescale.txt")
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if mtime == self._ts_mtime:
            return
        if mtime < self._loaded_at:
            # Written for the flight before this load: a loaded save starts
            # at 1x, as the game does, until someone asks otherwise.
            return
        self._ts_mtime = mtime
        fields = {}
        try:
            for line in open(path):
                if "=" in line:
                    k, v = line.split("=", 1)
                    fields[k.strip()] = v.strip()
        except OSError:
            return
        mode = fields.get("mode", "off")
        try:
            if mode == "fixed":
                self.latency_scale = float(fields.get("scale", 1.0))
            elif mode == "adaptive":
                self.latency_scale = float(fields.get("max_scale", 1.0))
            else:
                self.latency_scale = 1.0
        except ValueError:
            pass

    def _write_status(self):
        if not self.instance_dir:
            return
        try:
            with open(os.path.join(self.instance_dir, "timescale-status.txt"), "w") as fh:
                fh.write("achieved = %.4f\nscale = %.4f\nfps = 500\nwall = %.3f\n"
                         % (self.latency_scale, self.latency_scale, time.time()))
        except OSError:
            pass

    def _flow(self):
        last = time.monotonic()
        last_status = 0.0
        while not self._stop.is_set():
            time.sleep(0.005)
            now = time.monotonic()
            # One iteration sleeps 5 ms; a longer gap is the flow thread
            # having waited on the lock (a Load holds it for a second), and
            # catching that up would run the world through time nobody saw
            # pass -- a save loaded paused would start a second late.
            wall = min(now - last, 0.05)
            last = now
            self._read_timescale()
            if now - last_status > 0.5:
                self._write_status()
                last_status = now
            with self.lock:
                if self.world.loaded and not self.world.paused:
                    # The farm's time-scale plugin stands down under rails
                    # warp (KSP holds Time.timeScale at 1 there): the rate
                    # is the warp's alone.
                    rate = (self.world.warp_rate() if self.world.on_rails
                            else self.latency_scale)
                    before = self.world.t
                    self.world.advance(wall * rate)
                    self._log_events()
                    if self.world.t != before:
                        self.push_streams()

    def _log_events(self):
        """Parts lost and the like, to the instance's log with the UT."""
        ev = self.world.events
        if ev:
            for e in ev:
                print("[%10.2f] %s" % (self.world.t, e), flush=True)
            del ev[:]

    def _stat(self, key, t0):
        e = self.stats.get(key)
        if e is None:
            e = self.stats[key] = [0, 0.0]
        e[0] += 1
        e[1] += time.perf_counter() - t0

    def report_stats(self, label):
        """The time table, heaviest first, to the instance's log."""
        rows = sorted(self.stats.items(), key=lambda kv: -kv[1][1])
        total = sum(v[1] for v in self.stats.values())
        print("time table (%s): %.2f s accounted" % (label, total))
        for (svc, proc), (n, t) in rows[:25]:
            print("  %-45s %7d calls %8.3f s  %6.3f ms/call" % (svc + "." + proc, n, t,
                                                                 1e3 * t / max(n, 1)))
        self.stats = {}

    def advance_to(self, ut):
        with self.lock:
            t0 = time.perf_counter()
            if self.world.loaded and not self.world.paused:
                self.world.advance_to(ut)
                self._log_events()
            self._stat(("physics", "AdvanceTo"), t0)
            t0 = time.perf_counter()
            self.push_streams()
            self._stat(("streams", "push"), t0)
            return self.generation

    # -- streams ----------------------------------------------------------

    def push_streams(self):
        """Send every started stream whose value changed, generation last."""
        self.generation += 1
        for client in list(self.clients.values()):
            if client.stream_sock is None or not client.connected:
                continue
            items = []
            gen_ids = []
            for sid, entry in client.streams.items():
                call, started, last = entry
                if not started:
                    continue
                if call.service == "Sim" and call.procedure == "get_Generation":
                    gen_ids.append(sid)
                    continue
                data = fastpb.serialize_result(self.execute(call, client))
                if data != last:
                    entry[2] = data
                    items.append((sid, data))
            for sid in gen_ids:
                items.append((sid, self._generation_result()))
            if items:
                self._send_stream(client, fastpb.serialize_stream_update(items))

    def _generation_result(self):
        return fastpb.serialize_result(fastpb.Result(
            self.sig.encode_result("Sim", "get_Generation", self.generation)))

    def _send_stream(self, client, update):
        """``update``: StreamUpdate bytes."""
        try:
            with client.send_lock:
                client.stream_sock.sendall(fastpb.frame(update))
        except OSError:
            client.connected = False

    # -- dispatch ---------------------------------------------------------

    def execute(self, call, client):
        """One ``fastpb.Call`` -> a ``fastpb.Result``."""
        t0 = time.perf_counter()
        try:
            return self._execute(call, client)
        finally:
            if not (call.service == "Sim" and call.procedure == "AdvanceTo"):
                self._stat((call.service, call.procedure), t0)

    def _execute(self, call, client):
        result = fastpb.Result()
        key = (call.service, call.procedure)
        if key == ("SpaceCenter", "Load"):
            self.latency_scale = 1.0
            self._loaded_at = time.time()
            self._ts_mtime = None
        try:
            if call.service == "KRPC":
                value = self._krpc(call, client)
            elif call.service == "Sim":
                value = self._sim(call, client)
            else:
                handler = self.api.HANDLERS.get(key)
                if handler is None:
                    raise NotImplementedError("%s.%s is not simulated" % key)
                args = self.sig.decode_args(call.service, call.procedure,
                                            call.arguments, self.registry)
                value = handler(self.world, *args)
            if key in self.sig.procs:
                result.value = self.sig.encode_result(call.service, call.procedure,
                                                      self._register(value))
        except NotImplementedError as exc:
            if key not in self.missing:
                self.missing.add(key)
                try:
                    with open(os.path.join(ROOT, "logs", "kspsim-missing.txt"), "a") as fh:
                        fh.write("%s.%s\n" % key)
                except OSError:
                    pass
            result.error = ("KRPC", "", str(exc), "")
        except Exception as exc:                           # noqa: BLE001
            result.error = ("", "", "%s: %s" % (type(exc).__name__, exc),
                            traceback.format_exc())
        return result

    def _register(self, value):
        """Give every object in a result an id before it is encoded."""
        if isinstance(value, (list, tuple, set)):
            for v in value:
                self._register(v)
        elif isinstance(value, dict):
            for k, v in value.items():
                self._register(k)
                self._register(v)
        elif hasattr(value, "__sim_object__"):
            self.registry.id_of(value)
        return value

    def _krpc(self, call, client):
        p = call.procedure
        args = self.sig.decode_args("KRPC", p, call.arguments, self.registry)
        if p == "GetServices":
            return self.sig.services_message
        if p == "GetStatus":
            return KRPC.Status(version="0.6.0")
        if p == "GetClientID":
            return client.ident
        if p == "GetClientName":
            return client.name
        if p == "get_Paused":
            return self.world.paused
        if p == "set_Paused":
            self.world.paused = bool(args[0])
            return None
        if p == "AddStream":
            pc, start = args[0], args[1]
            sid = self.next_stream
            self.next_stream += 1
            client.streams[sid] = [pc, False, None]
            if start is None or start:
                self._start_stream(client, sid)
            return KRPC.Stream(id=sid)
        if p == "StartStream":
            self._start_stream(client, args[0])
            return None
        if p == "RemoveStream":
            client.streams.pop(args[0], None)
            return None
        if p == "SetStreamRate":
            return None
        if p == "get_CurrentGameScene":
            return 0  # SpaceCenter.GameScene.flight
        if p == "get_Clients":
            return []
        raise NotImplementedError("KRPC.%s is not simulated" % p)

    def _start_stream(self, client, sid):
        entry = client.streams.get(sid)
        if entry is None:
            return
        entry[1] = True
        pc = entry[0]
        if pc.service == "Sim" and pc.procedure == "get_Generation":
            data = self._generation_result()
        else:
            data = fastpb.serialize_result(self.execute(pc, client))
            entry[2] = data
        update = fastpb.serialize_stream_update([(sid, data)])
        # Sent from a thread: the client holds the stream's condition while it
        # calls StartStream and only then waits, so the update must not race
        # ahead of the response on a single thread's ordering assumptions.
        threading.Thread(target=self._send_stream, args=(client, update),
                         daemon=True).start()

    def _sim(self, call, client):
        p = call.procedure
        args = self.sig.decode_args("Sim", p, call.arguments, self.registry)
        if p == "AdvanceTo":
            return self.advance_to(args[0])
        if p == "get_Generation":
            return self.generation
        if p == "get_LatencyScale":
            return self.latency_scale
        if p == "set_LatencyScale":
            self.latency_scale = float(args[0])
            return None
        raise NotImplementedError("Sim.%s" % p)

    # -- sockets ----------------------------------------------------------

    def _rpc_conn(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        hello = wire.recv(sock, KRPC.ConnectionRequest)
        if hello is None:
            sock.close()
            return
        client = ClientState(hello.client_name)
        with self.lock:
            self.clients[client.ident] = client
        sock.sendall(wire.frame(KRPC.ConnectionResponse(
            status=KRPC.ConnectionResponse.OK, client_identifier=client.ident)))
        reader = wire.FrameReader(sock)
        try:
            while True:
                body = reader.read()
                if body is None:
                    break
                results = []
                advance = None
                with self.lock:
                    for call in fastpb.parse_request(body):
                        if call.service == "Sim" and call.procedure == "AdvanceTo":
                            advance = call
                            continue
                        results.append(fastpb.serialize_result(self.execute(call, client)))
                if advance is not None:
                    # Outside the per-call lock so the stream push inside it
                    # is ordered before the response the client waits on.
                    results.append(fastpb.serialize_result(self.execute(advance, client)))
                sock.sendall(fastpb.frame(fastpb.serialize_response(results)))
        except OSError:
            pass
        finally:
            client.connected = False
            with self.lock:
                self.clients.pop(client.ident, None)
                self.world.client_gone(client)
                if sum(v[1] for v in self.stats.values()) > 0.5:
                    self.report_stats(client.name or "client")
            try:
                sock.close()
            except OSError:
                pass

    def _stream_conn(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        hello = wire.recv(sock, KRPC.ConnectionRequest)
        if hello is None:
            sock.close()
            return
        with self.lock:
            client = self.clients.get(hello.client_identifier)
        if client is None:
            sock.sendall(wire.frame(KRPC.ConnectionResponse(
                status=KRPC.ConnectionResponse.WRONG_TYPE, message="unknown client")))
            sock.close()
            return
        sock.sendall(wire.frame(KRPC.ConnectionResponse(
            status=KRPC.ConnectionResponse.OK, client_identifier=client.ident)))
        client.stream_sock = sock

    def _listen(self, port, handler):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.address, port))
        srv.listen(32)
        while not self._stop.is_set():
            sock, _ = srv.accept()
            threading.Thread(target=handler, args=(sock,), daemon=True).start()

    def serve_forever(self):
        threading.Thread(target=self._listen, args=(self.stream_port, self._stream_conn),
                         daemon=True).start()
        threading.Thread(target=self._flow, daemon=True).start()
        self._listen(self.rpc_port, self._rpc_conn)
