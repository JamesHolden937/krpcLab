#!/usr/bin/env python3
"""Write ``kspSim/data/services.bin``: the kRPC ``GetServices`` response the
sim serves back verbatim.

    ./kspSim/tools/services.py --instance 0     # recorded from a farm instance
    ./kspSim/tools/services.py --json           # built from base/GameData/kRPC/*.json
    ./kspSim/tools/services.py --compare        # the two, procedure by procedure

The recording is the reference; ``--json`` needs no game and gives the same
signatures (the server's own service definitions, which the build also reads).
"""
import argparse
import base64
import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common.paths import use_venv  # noqa: E402

use_venv()

import krpc.schema.KRPC_pb2 as KRPC  # noqa: E402

OUT = os.path.join(ROOT, "kspSim", "data", "services.bin")
GAMEDATA = os.path.join(ROOT, "testInstances", "base", "GameData", "kRPC")


def _type(msg, spec):
    msg.code = getattr(KRPC.Type, spec["code"])
    if spec.get("service"):
        msg.service = spec["service"]
    if spec.get("name"):
        msg.name = spec["name"]
    for sub in spec.get("types", []):
        _type(msg.types.add(), sub)
    if spec.get("nullable"):
        msg.nullable = True


def from_json(gamedata=GAMEDATA):
    services = KRPC.Services()
    files = sorted(glob.glob(os.path.join(gamedata, "*.json")))
    defs = {}
    for path in files:
        with open(path) as fh:
            defs.update(json.load(fh))
    # GetServices lists the services in the server's id order.
    for name, svc in sorted(defs.items(), key=lambda kv: kv[1].get("id", 0)):
        s = services.services.add(name=name)
        for pname, proc in svc.get("procedures", {}).items():
            p = s.procedures.add(name=pname)
            for par in proc.get("parameters", []):
                q = p.parameters.add(name=par["name"])
                _type(q.type, par["type"])
                if "default_value" in par:
                    q.has_default_value = True
                    if par["default_value"] is None:
                        q.default_value_is_null = True
                    else:
                        q.default_value = base64.b64decode(par["default_value"])
            if "return_type" in proc:
                _type(p.return_type, proc["return_type"])
            for scene in proc.get("game_scenes", []):
                p.game_scenes.append(getattr(KRPC.Procedure, scene))
        for cname in svc.get("classes", {}):
            s.classes.add(name=cname)
        for ename, enum in svc.get("enumerations", {}).items():
            e = s.enumerations.add(name=ename)
            for v in enum.get("values", []):
                e.values.add(name=v["name"], value=v["value"])
        for xname in svc.get("exceptions", {}):
            s.exceptions.add(name=xname)
        for stname, st in svc.get("structs", {}).items():
            t = s.structs.add(name=stname)
            for f in st.get("fields", []):
                _type(t.fields.add(name=f["name"]).type, f["type"])
    return services


def from_instance(n):
    import krpc
    from common import paths
    rpc, _ = paths.ports(n)
    conn = krpc.connect(name="services", rpc_port=rpc, stream_port=None)
    try:
        return conn._invoke("KRPC", "GetServices", [], [], conn._types.services_type)
    finally:
        conn.close()


def _signatures(services):
    out = {}
    for s in services.services:
        for p in s.procedures:
            out[(s.name, p.name)] = (
                [(q.name, q.type.SerializeToString(), q.default_value, q.default_value_is_null)
                 for q in p.parameters], p.return_type.SerializeToString())
        for st in s.structs:
            out[(s.name, "struct " + st.name)] = [
                (f.name, f.type.SerializeToString()) for f in st.fields]
    return out


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--instance", type=int)
    g.add_argument("--json", action="store_true")
    g.add_argument("--compare", type=int, metavar="INSTANCE")
    ap.add_argument("--gamedata", default=GAMEDATA, help="the kRPC mod's GameData folder")
    args = ap.parse_args()
    if args.compare is not None:
        a, b = _signatures(from_json(args.gamedata)), _signatures(from_instance(args.compare))
        diff = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
        for k in diff:
            print("%-60s json=%s game=%s" % ("%s.%s" % k, k in a, k in b))
        print("%d signatures, %d differ" % (len(b), len(diff)))
        return
    services = from_json(args.gamedata) if args.json else from_instance(args.instance)
    with open(OUT, "wb") as fh:
        fh.write(services.SerializeToString())
    print("%s: %d services, %d procedures" % (
        OUT, len(services.services), sum(len(s.procedures) for s in services.services)))


if __name__ == "__main__":
    main()
