#!/usr/bin/env python3
"""setports.py <kRPC settings.cfg> <rpc_port> [<stream_port>]

kRPC 0.8 serves RPCs and streams on one TCP port, ``tcpip_port`` (and
``tcpip_address``); the stream port argument is accepted and ignored.  The
settings are not flat keys.  They live as

    Item { key = tcpip_port
           value = 50000 }

inside servers/Item/settings, so the value line has to be rewritten by its
position after the matching key line rather than by name.
"""
import re
import sys

path, rpc = sys.argv[1], sys.argv[2]
want = {"tcpip_port": rpc}

lines = open(path).read().split("\n")
pending = None
changed = {}
for i, line in enumerate(lines):
    m = re.match(r"(\s*)key = (\S+)\s*$", line)
    if m:
        pending = m.group(2)
        continue
    m = re.match(r"(\s*)value = (.*)$", line)
    if m and pending in want:
        lines[i] = "%svalue = %s" % (m.group(1), want[pending])
        changed[pending] = want[pending]
    pending = None

missing = sorted(set(want) - set(changed))
if missing:
    sys.exit("setports: could not find %s in %s (kRPC 0.8 keys)" % (missing, path))

open(path, "w").write("\n".join(lines))
print("  kRPC   : tcpip_port=%s" % changed["tcpip_port"])
