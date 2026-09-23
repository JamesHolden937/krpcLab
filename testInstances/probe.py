#!/usr/bin/env python3
"""probe.py <rpc_port> <stream_port> -- can we drive this instance yet?

Reports whether the kRPC server answers and whether SpaceCenter.load works
from wherever the game currently is.  quickfly.py calls space_center.load()
as its first act, and kRPC's SpaceCenter service is only usable once the game
is in a scene -- so an instance sitting at the main menu may answer the socket
and still refuse the load.
"""
import sys
sys.path.insert(0, "..")
import krpc

rpc, stream = int(sys.argv[1]), int(sys.argv[2])
conn = krpc.connect(name="probe", address="127.0.0.1",
                    rpc_port=rpc, stream_port=stream)
print("connected; server version", conn.krpc.get_status().version)
try:
    scene = conn.krpc.current_game_scene
    print("scene:", scene)
except Exception as exc:
    print("current_game_scene failed:", exc)
    scene = None
try:
    conn.space_center.load("quicksave")
    print("space_center.load('quicksave'): OK")
except Exception as exc:
    print("space_center.load('quicksave') FAILED: %s: %s"
          % (type(exc).__name__, exc))
