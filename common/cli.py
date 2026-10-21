"""The command line every krpcLab pilot accepts, and the connection it
describes (root ``CLAUDE.md``, "Adding an autopilot"): ``--address``,
``--rpc-port``, ``--stream-port``, ``--name``, ``--set FIELD=VALUE`` and
``--autostart``.  Unset connection flags fall to kRPC's own defaults."""

import argparse


def parser(name, description, pilot=True):
    """An ``ArgumentParser`` with the connection flags, and (``pilot``) the
    ``--set`` and ``--autostart`` every autopilot takes.  Add the pilot's
    own flags to what it returns."""
    p = argparse.ArgumentParser(prog=name, description=description)
    p.add_argument("--address", default=None,
                   help="kRPC server address (default: kRPC's, localhost)")
    p.add_argument("--rpc-port", type=int, default=None)
    p.add_argument("--stream-port", type=int, default=None)
    p.add_argument("--name", default=name, help="kRPC client name")
    if pilot:
        p.add_argument("--set", action="append", default=[],
                       metavar="FIELD=VALUE",
                       help="override a Config field, repeatable")
        p.add_argument("--autostart", action="store_true",
                       help="do not wait for the panel's START button")
    return p


def connect_kwargs(args, name=None):
    """``krpc.connect`` keywords for the parsed connection flags."""
    kwargs = {"name": name or getattr(args, "name", None) or "krpclab"}
    for key in ("address", "rpc_port", "stream_port"):
        value = getattr(args, key, None)
        if value is not None:
            kwargs[key] = value
    return kwargs


def connect(args, name=None):
    import krpc
    return krpc.connect(**connect_kwargs(args, name))
