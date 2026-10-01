"""Start one simulated instance.

    .venv/bin/python -m kspSim.run --instance 0

It looks like a farm instance to every harness: ``testInstances/kspsim<N>/``
holds ``.rpc_port`` and ``.stream_port`` (50200 + 2N and the next port up),
so ``quickglide.py --instance sim0`` flies it unchanged, and the time-scale
governor's ``timescale.txt`` lands there too.  ``SpaceCenter.Load(name)``
loads ``kspSim/models/<name>.json``.

Nothing prints: the server's own log is ``logs/kspsim<N>.log``.
"""
import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kspSim import api  # noqa: E402
from kspSim.server import Server  # noqa: E402
from kspSim.world import World  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--save", default=None, help="load this model at start")
    ap.add_argument("--latency-scale", type=float, default=1.0)
    args = ap.parse_args()
    inst = os.path.join(ROOT, "testInstances", "kspsim%d" % args.instance)
    os.makedirs(inst, exist_ok=True)
    rpc = 50200 + 2 * args.instance
    stream = rpc + 1
    with open(os.path.join(inst, ".rpc_port"), "w") as fh:
        fh.write(str(rpc))
    with open(os.path.join(inst, ".stream_port"), "w") as fh:
        fh.write(str(stream))
    log = open(os.path.join(ROOT, "logs", "kspsim%d.log" % args.instance), "a", buffering=1)
    sys.stdout = sys.stderr = log
    print("kspsim%d up %s rpc %d stream %d" % (args.instance, time.ctime(), rpc, stream))
    world = World()
    if args.save:
        world.load(args.save)
    server = Server(world, api, rpc, stream, instance_dir=inst,
                    latency_scale=args.latency_scale)
    api.install_ui(server.sig)
    server.serve_forever()


if __name__ == "__main__":
    main()
