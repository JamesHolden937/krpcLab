#!/usr/bin/env python3
"""Prune a clone's part catalogue to the parts its saves actually reference.

An instance compiles every part in GameData at load: 489 of them, against the
26 that every vehicle in this farm's thirteen quicksaves put together uses.
That is ~2.3 GB of resident memory and most of a cold load spent on parts
nothing will ever spawn, and RAM is what caps the number of instances --
which is the thing that actually multiplies measurement throughput.

    ./partstrip.py 4 --dry-run      # what would go, and what it would save
    ./partstrip.py 4                # do it
    ./partstrip.py 4 --report       # what this clone currently keeps

**It runs on a clone and never on base/.**  `mkclone.sh` hardlinks against
base, so deleting inside a clone drops only that clone's directory entries and
the master keeps all 489 parts.  That is what makes this safe to get wrong:
a bad keep-list costs one `mkclone.sh`, and nothing propagates.  A strip
applied to base/ would instead be permanent and would take every future
instance with it, so this refuses to touch it.

**The keep-list is derived, never written down.**  It is read out of the
clone's own `saves/*.sfs` every time, so adding a vehicle means re-cloning and
re-running rather than editing a list somebody has to remember to update.  The
closed-world assumption is real and it is the price: an instance stripped for
today's craft cannot fly a craft built from a 27th part, and the failure mode
is a vessel that silently does not load rather than a clean error.  Re-clone
when the fleet changes.

One trap, paid for: **KSP writes `.` in a save where the part config spells
`_`** -- `Decoupler.0` in the save is `name = Decoupler_0` in
`Decoupler_0.cfg`.  Seven of the twenty-six resolve only after normalising
that, and a `grep "name = Decoupler.0"` appears to find them anyway because
`.` is a regex wildcard that happily matches the underscore.  That false
confirmation would have deleted six parts the vehicles do use.
"""
import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PART_ROOTS = ("GameData/Squad/Parts", "GameData/SquadExpansion")
NAME = re.compile(r"^\s*name\s*=\s*([A-Za-z0-9_.\-]+)", re.M)


def norm(name):
    """A part's identity, independent of how the file spells it."""
    return name.strip().replace(".", "_")


def parts_in_saves(instance_dir):
    """Every part name any save in this instance references.

    Read out of the `PART {}` blocks rather than from a list in this file:
    the saves are the ground truth for what the instance has to be able to
    spawn, and they change without anyone remembering to tell a script.
    """
    used, saves = set(), []
    root = os.path.join(instance_dir, "saves")
    for dirpath, _, files in os.walk(root):
        for f in files:
            if not f.endswith(".sfs"):
                continue
            path = os.path.join(dirpath, f)
            saves.append(path)
            want = False
            for line in open(path, errors="replace"):
                stripped = line.strip()
                if stripped == "PART":
                    want = True
                elif want and stripped.startswith("name = "):
                    used.add(norm(stripped[7:]))
                    want = False
    return used, saves


def keep_dirs(instance_dir, used):
    """Directories holding a config for a part in ``used``."""
    keep, found = set(), set()
    for root in PART_ROOTS:
        base = os.path.join(instance_dir, root)
        for dirpath, _, files in os.walk(base):
            for f in files:
                if not f.endswith(".cfg"):
                    continue
                text = open(os.path.join(dirpath, f), errors="replace").read()
                for m in NAME.finditer(text):
                    key = norm(m.group(1))
                    if key in used:
                        keep.add(dirpath)
                        found.add(key)
    return keep, found


def protected(path, keep):
    """Kept itself, an ancestor of something kept, or inside something kept."""
    return any(k == path or k.startswith(path + os.sep)
               or path.startswith(k + os.sep) for k in keep)


def megabytes(path):
    total = 0
    for dirpath, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(dirpath, f))
            except OSError:
                pass
    return total / 1048576.0


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", help="instance number (e.g. 4), or a path")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--report", action="store_true",
                   help="list what is referenced and where it lives")
    a = p.parse_args()

    # A number names a clone; a path is taken as given, so that the guard
    # below is reachable rather than decorative.
    directory = a.instance if os.path.isdir(a.instance) \
        else os.path.join(HERE, "ksp" + str(a.instance))
    if not os.path.isdir(directory):
        raise SystemExit("no such instance: %s" % directory)
    # base/ is the master every clone hardlinks against.  Stripping it would
    # be permanent and would take every future instance with it.
    if os.path.realpath(directory) == os.path.realpath(os.path.join(HERE, "base")):
        raise SystemExit("refusing to strip base/ -- strip a clone")

    used, saves = parts_in_saves(directory)
    if not saves:
        raise SystemExit("no saves in %s -- nothing to derive a keep-list from"
                         % directory)
    keep, found = keep_dirs(directory, used)
    missing = sorted(used - found)

    print("%d saves, %d distinct parts referenced, %d resolved to %d directories"
          % (len(saves), len(used), len(found), len(keep)))
    if missing:
        # **Not a warning to scroll past.**  An unresolved name is a part the
        # saves need and this run could not find a home for, so proceeding
        # would delete nothing for it and prove nothing -- or worse, the part
        # lives in a directory about to go.
        print("unresolved, refusing to strip: %s" % ", ".join(missing))
        return 1
    if a.report:
        for d in sorted(keep):
            print("  keep %s" % os.path.relpath(d, directory))
        return 0

    before = sum(megabytes(os.path.join(directory, r)) for r in PART_ROOTS)
    removed = []
    for root in PART_ROOTS:
        base = os.path.join(directory, root)
        for dirpath, dirnames, _ in os.walk(base, topdown=True):
            for d in list(dirnames):
                full = os.path.join(dirpath, d)
                if protected(full, keep):
                    continue
                removed.append(os.path.relpath(full, directory))
                dirnames.remove(d)
                if not a.dry_run:
                    shutil.rmtree(full)
    after = sum(megabytes(os.path.join(directory, r)) for r in PART_ROOTS)
    print("%s %d part directories: %.0f MB -> %.0f MB"
          % ("would remove" if a.dry_run else "removed", len(removed),
             before, after if not a.dry_run else before))
    return 0


if __name__ == "__main__":
    sys.exit(main())
