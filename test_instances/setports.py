#!/usr/bin/env python3
"""setports.py <kRPC settings.cfg> <rpc_port> <stream_port>

The ports in kRPC's settings.cfg are not flat keys.  They live as

    Item { key = rpc_port
           value = 50000 }

inside servers/Item/settings, so the value line has to be rewritten by its
position after the matching key line rather than by name.
"""
import re
import sys

path, rpc, stream = sys.argv[1], sys.argv[2], sys.argv[3]
want = {"rpc_port": rpc, "stream_port": stream}

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
    sys.exit("setports: could not find %s in %s" % (missing, path))

open(path, "w").write("\n".join(lines))
print("  kRPC   : rpc_port=%s stream_port=%s" % (changed["rpc_port"],
                                                 changed["stream_port"]))
