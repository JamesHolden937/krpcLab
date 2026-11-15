#!/usr/bin/env python3
"""Prune a clone's part catalogue to the parts its vehicles actually use.

An instance compiles every PART node in its game database at load (498 on
KSP Retooled with this farm's mods) against the few dozen the reference saves
and craft files use, and RAM is what caps the number of instances -- the thing
that multiplies measurement throughput.

    ./partstrip.py 6 --dry-run      # what would be pruned
    ./partstrip.py 6                # write the patch
    ./partstrip.py 6 --undo         # remove it
    ./partstrip.py 6 --report       # the keep-list and where each part lives

**It deletes nothing on disk.**  It writes one ModuleManager patch,
`GameData/zzzKrpcLabPrune/prune.cfg`, that removes every PART node not in the
keep-list -- and every INTERNAL and PROP node no kept part reaches -- at
`:FINAL`, after every other mod's patches have run.  Every file anything
references is still there; only the nodes are gone from the database before
PartLoader compiles it.  The earlier version deleted part directories and
wedged the instance at "Compiling Internal Prop": a ReStock patch applied to a
*kept* part named a texture in a *deleted* directory, an edge no
directory-level keep-list can see.  Undoing it is deleting one directory.

The patch is a new file in the clone's own GameData/ directory (mkclone.sh
recreates directories, so new files are private) and never touches base/.

**The keep-list is derived, never written down.**  Read every time from the
tracked reference saves and craft files (../saves/*.sfs, ../saves/craft/) and
from the clone's own saves/default/ (its .sfs and Ships/*/*.craft), plus the
parts the game spawns by itself (EVA kerbals, flags, asteroids, comets).
start.sh re-runs it before every launch, so a new save or craft file is never
flown on an instance that pruned its parts.  A save made in this instance on a
part outside the list cannot load until the next launch -- re-run and restart.

One trap, paid for: **KSP writes `.` in a save where the part config spells
`_`** -- `Decoupler.0` in a save is `name = Decoupler_0` in its cfg -- and
craft files write `part = <name>_<flightID>`.  Names are compared normalised
(`.` -> `_`) and the patch names the config's own spelling.
"""
import argparse
import fnmatch
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PATCH_DIR = "zzzKrpcLabPrune"
PATCH = os.path.join(PATCH_DIR, "prune.cfg")
# Spawned by the game itself, never named in a save we fly.
ALWAYS = ("kerbalEVA*", "flag", "PotatoRoid", "PotatoComet")
NAME = re.compile(r"^\s*name\s*=\s*(\S+)\s*$")
CRAFT_PART = re.compile(r"^\s*part\s*=\s*(\S+?)_\d+\s*$")


def norm(name):
    """A part's identity, independent of how the file spells it."""
    return name.strip().replace(".", "_")


def walk(root, ext):
    for dirpath, _, files in os.walk(root):
        for f in sorted(files):
            if f.endswith(ext):
                yield os.path.join(dirpath, f)


def parts_in_sfs(path, used):
    want = False
    for line in open(path, errors="replace"):
        s = line.strip()
        if s == "PART":
            want = True
        elif want and s.startswith("name = "):
            used.add(norm(s[7:]))
            want = False


def parts_in_craft(path, used):
    for line in open(path, errors="replace"):
        m = CRAFT_PART.match(line)
        if m:
            used.add(norm(m.group(1)))


def sources(instance_dir):
    """Every save and craft file whose parts this instance must spawn."""
    sfs = list(walk(os.path.join(ROOT, "saves"), ".sfs"))
    # Only the farm's own save game: the clone's saves/scenarios and
    # saves/training are stock missions nothing here flies.
    own = os.path.join(instance_dir, "saves", "default")
    sfs += [f for f in walk(own, ".sfs")
            if os.sep + "Ships" + os.sep not in f]
    craft = list(walk(os.path.join(ROOT, "saves", "craft"), ".craft"))
    craft += walk(os.path.join(own, "Ships"), ".craft")
    return sfs, craft


def parse(path):
    """A ConfigNode file as nested (name, values, children) tuples."""
    root = ("root", [], [])
    stack, pending = [root], None
    for raw in open(path, errors="replace"):
        line = raw.split("//", 1)[0].strip()
        while line:
            if line.startswith("{"):
                node = (pending or "", [], [])
                stack[-1][2].append(node)
                stack.append(node)
                pending, line = None, line[1:].strip()
            elif line.startswith("}"):
                if len(stack) > 1:
                    stack.pop()
                pending, line = None, line[1:].strip()
            else:
                cut = min([i for i in (line.find("{"), line.find("}")) if i >= 0]
                          or [len(line)])
                head, line = line[:cut].strip(), line[cut:]
                if "=" in head:
                    k, v = head.split("=", 1)
                    stack[-1][1].append((k.strip(), v.strip()))
                    pending = None
                elif head:
                    pending = head
    return root


def value(node, key):
    for k, v in node[1]:
        if k == key:
            return v
    return None


def database(instance_dir):
    """The patched game database: ModuleManager's cache of it.

    The cache is what PartLoader compiles, so it names every part (including
    ones a patch made with +PART) under its final name, and every INTERNAL and
    PROP.  Returns {kind: {name: node}}.
    """
    cache = os.path.join(instance_dir, "GameData", "ModuleManager.ConfigCache")
    db = {"PART": {}, "INTERNAL": {}, "PROP": {}, "url": {}, "other": []}
    if not os.path.isfile(cache):
        return None     # a fresh clone: never booted
    for url in parse(cache)[2]:
        parent = value(url, "parentUrl") or ""
        for node in url[2]:
            n = value(node, "name")
            if node[0] in ("PART", "INTERNAL", "PROP") and n:
                db[node[0]][n] = node
                db["url"][id(node)] = parent
            else:
                db["other"].append(node)
    return db


# Directories whose .mu files are part, IVA or prop models.  Anything else
# (Squad/FX, Kopernicus scatter, the KSC) is loaded by code by name and is
# never a candidate.
MODEL_ROOTS = re.compile(r"^(Squad/(Parts|Props|zDeprecated)|SquadExpansion/[^/]+/(Parts|Props|Spaces)"
                         r"|ReStock/(Assets|Props|Spaces)|[^/]+/Parts)/")


def values(node, out):
    for _, v in node[1]:
        out.add(v.strip().replace("\\", "/").lower())
    for child in node[2]:
        values(child, out)


def model_refs(db, kept):
    """Every .mu path (lower case, no extension) a surviving node can name:
    any value of a kept PART/INTERNAL/PROP or of any other root node, a
    `mesh =` resolved against its cfg's directory, and every model in the
    directory of a part that names none (KSP then takes the first .mu there)."""
    tokens, dirs = set(), set()
    for node in db["other"]:
        values(node, tokens)
    for kind in ("PART", "INTERNAL", "PROP"):
        for name in kept[kind]:
            node = db[kind].get(name)
            if node is None:
                continue
            here = os.path.dirname(db["url"][id(node)])
            mine = set()
            values(node, mine)
            tokens |= mine
            mesh = value(node, "mesh")
            if mesh:
                tokens.add(os.path.join(here, mesh).lower())
            if not any(c[0] == "MODEL" for c in node[2]):
                dirs.add(here.lower())
    refs = set()
    for t in tokens:
        for word in re.split(r"[\s,;]+", t):
            refs.add(word[:-3] if word.endswith(".mu") else word)
    return refs, dirs


def prune_models(directory, db, kept, dry):
    """Delete the clone's .mu files no surviving node references.

    The clone's files are hardlinks into base/, so this drops only the
    clone's entries; `--undo` links them back.  Returns (deleted, bytes)."""
    gd = os.path.join(directory, "GameData")
    refs, dirs = model_refs(db, kept)
    gone, size = [], 0
    for path in walk(gd, ".mu"):
        rel = os.path.relpath(path, gd).replace(os.sep, "/")
        if not MODEL_ROOTS.match(rel) or "/fx/" in rel.lower():
            continue
        key = rel[:-3].lower()
        if key in refs or os.path.dirname(key) in dirs:
            continue
        gone.append(rel)
        size += os.path.getsize(path)
        if not dry:
            os.remove(path)
    return gone, size


def restore_models(directory):
    """Link back every .mu base/ has and this clone lacks."""
    gd = os.path.join(directory, "GameData")
    base = os.path.join(HERE, "base", "GameData")
    n = 0
    for path in walk(base, ".mu"):
        rel = os.path.relpath(path, base)
        dst = os.path.join(gd, rel)
        if not os.path.exists(dst) and os.path.isdir(os.path.dirname(dst)):
            os.link(path, dst)
            n += 1
    return n


def raw_parts(instance_dir):
    """PART names defined in GameData's cfg files, before any patch: the
    fallback when the cache is a pruned one."""
    names = set()
    for path in walk(os.path.join(instance_dir, "GameData"), ".cfg"):
        if os.sep + PATCH_DIR + os.sep in path:
            continue
        want = False
        for line in open(path, errors="replace"):
            s = line.split("//", 1)[0].strip().lstrip("\ufeff")
            if s == "PART" or s.startswith("PART{") or s.startswith("PART {"):
                want = True
            elif want:
                m = NAME.match(s)
                if m:
                    names.add(m.group(1))
                    want = False
    return names


def internals_and_props(db, keep_parts):
    """INTERNAL nodes kept parts name, and PROP nodes those internals name."""
    internals = set()
    for p in keep_parts:
        for child in db["PART"][p][2]:
            if child[0] == "INTERNAL" and value(child, "name"):
                internals.add(value(child, "name"))
    props = set()
    for i in internals:
        node = db["INTERNAL"].get(i)
        if node:
            for child in node[2]:
                if child[0] == "PROP" and value(child, "name"):
                    props.add(value(child, "name"))
    return internals, props


def has_filter(names):
    return ",".join("~name[%s]" % n for n in sorted(names))


def patch_text(keep_parts, internals, props, prune_spaces=True):
    lines = [
        "// Written by krpcLab testInstances/partstrip.py: removes every part this",
        "// instance's saves and craft files do not use, after all other patches.",
        "// Undo: delete this directory (or ./partstrip.py N --undo).",
        "!PART[*]:HAS[%s]:FINAL {}" % has_filter(set(keep_parts) | set(ALWAYS)),
    ]
    if prune_spaces:
        lines += [
            "!INTERNAL[*]:HAS[%s]:FINAL {}" % has_filter(internals or {"-none-"}),
            "!PROP[*]:HAS[%s]:FINAL {}" % has_filter(props or {"-none-"}),
        ]
    return "\n".join(lines) + "\n"


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("instance", help="instance number (e.g. 6), or a path")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--report", action="store_true")
    g.add_argument("--undo", action="store_true")
    p.add_argument("--models", action="store_true",
                   help="also delete the .mu models no surviving node uses")
    a = p.parse_args()

    directory = a.instance if os.path.isdir(a.instance) \
        else os.path.join(HERE, "ksp" + str(a.instance))
    if not os.path.isdir(directory):
        raise SystemExit("no such instance: %s" % directory)
    if os.path.realpath(directory) == os.path.realpath(os.path.join(HERE, "base")):
        raise SystemExit("refusing to prune base/ -- prune a clone")
    target = os.path.join(directory, "GameData", PATCH)

    if a.undo:
        if os.path.exists(target):
            os.remove(target)
            os.rmdir(os.path.dirname(target))
            print("removed %s" % target)
        n = restore_models(directory)
        if n:
            print("relinked %d models from base/" % n)
        return 0

    sfs, craft = sources(directory)
    used = set()
    for f in sfs:
        parts_in_sfs(f, used)
    for f in craft:
        parts_in_craft(f, used)

    db = database(directory)
    if db is None:
        # Never booted: no cache to read internals from, so prune parts only
        # (keep every INTERNAL and PROP) and let the next launch refine it.
        print("no ModuleManager cache yet: pruning parts only this boot")
        db = {"PART": {}, "INTERNAL": {}, "PROP": {}, "url": {}, "other": []}
    # The cache may be a pruned one, missing a part a new save needs: the
    # raw configs name every part a patch did not create.
    defined = set(db["PART"]) | raw_parts(directory)
    spelling = {norm(n): n for n in defined}
    missing = sorted(u for u in used if u not in spelling)
    keep = sorted(spelling[u] for u in used if u in spelling)

    print("%d saves, %d craft files: %d parts used, %d of %d defined kept"
          % (len(sfs), len(craft), len(used), len(keep), len(defined)))
    if missing:
        # An unresolved name is a part a vehicle needs that no config
        # defines.  Pruning cannot cause or cure it, but say so: such a
        # vehicle will not load on any instance.
        print("not defined anywhere (vehicles using them will not load): %s"
              % ", ".join(missing))
    in_cache = [k for k in keep if k in db["PART"]]
    stale = len(in_cache) < len(keep)
    if stale:
        print("the cache was built pruned; %d newly kept part(s) not in it: "
              "their internals are kept on the next run, after a boot"
              % (len(keep) - len(in_cache)))
    internals, props = internals_and_props(db, in_cache)
    if stale:
        # Keep whatever survived the last prune too: a newly kept part's
        # internal is not in this cache.
        internals |= set(db["INTERNAL"])
        props |= set(db["PROP"])
    print("internals kept %d of %d, props kept %d of %d (counts from the cache)"
          % (len(internals & set(db["INTERNAL"])), len(db["INTERNAL"]),
             len(props & set(db["PROP"])), len(db["PROP"])))
    if a.report:
        for k in keep:
            print("  keep %s" % k)
        return 0
    if a.models and (stale or not db["PART"]):
        print("cache lacks kept parts: models not pruned this boot")
    elif a.models:
        kept = {"PART": set(keep) | {n for n in db["PART"] if any(
                    fnmatch.fnmatch(n, g) for g in ALWAYS)},
                "INTERNAL": internals, "PROP": props}
        gone, size = prune_models(directory, db, kept, a.dry_run or a.report)
        print("models: %s %d (%.0f MB on disk)" % (
            "would delete" if a.dry_run or a.report else "deleted",
            len(gone), size / 1048576.0))
    text = patch_text(keep, internals, props, prune_spaces=bool(db["PROP"]))
    if a.dry_run:
        sys.stdout.write(text)
        return 0
    old = open(target).read() if os.path.exists(target) else None
    if old == text:
        print("unchanged %s" % target)
        return 0
    os.makedirs(os.path.dirname(target), exist_ok=True)
    tmp = target + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, target)
    print("wrote %s" % target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
