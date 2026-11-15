#!/usr/bin/env python3
"""Shrink a clone to what its vehicles use: parts, their models, and TPR's
visual-only planet textures.

An unpruned instance compiles 498 parts, loads 721 models and keeps ~1.8 GB of
TPR colour/normal/cloud/flare textures that no physics reads.  RAM caps the
number of instances, which is what multiplies measurement throughput.
Measured alone on ksp6 (docs/testInstances.md, "Pruning"): peak RSS+swap
5.26 -> 4.74 GB, flying qs_finbooster2 the same.

    ./partstrip.py 6 --dry-run      # what it would do
    ./partstrip.py 6                # do it (start.sh runs this before every launch)
    ./partstrip.py 6 --undo         # back to the full clone
    ./partstrip.py 6 --report       # the keep-list

Every run first restores the clone from base/ (relinks every model and TPR
file), then prunes again from a keep-list derived afresh, so a newly needed
part always gets its model back.  Three steps:

1. **Parts, IVAs and props: a ModuleManager patch, never a deletion.**
   `GameData/zzzKrpcLabPrune/prune.cfg` removes, at `:FINAL`, every PART node
   not in the keep-list and every INTERNAL/PROP no kept part reaches.  An
   earlier version deleted part directories and wedged the instance at
   "Compiling Internal Prop" (a ReStock patch on a kept part named a texture
   in a deleted directory).  Alone this saves no RAM: compiling a part is not
   the cost, loading its files is.
2. **Models: delete the .mu files no surviving node names.**  References come
   from ModuleManager's cache -- the patched database PartLoader compiles --
   so every value of every kept PART/INTERNAL/PROP and of every other root
   node counts, plus a part's own directory when it names no MODEL.  Only
   part/IVA/prop model directories are candidates (never Squad/FX).  Needs a
   cache that holds every kept part; otherwise models stay until the next run.
3. **TPR: replace each visual-only texture with a 4x4 stub of the same name
   and format**, and delete Configs/NewParallaxConfigs (Parallax is not
   installed; GameDatabase loads its 45 textures anyway).  Kept real: every
   body's biome map and every terrain input of Kerbin, Mun and Minmus (the
   height maps and multiplier maps their PQS mods read), so the ground is the
   live game's.  **Deleting instead of stubbing breaks the game**: Kopernicus
   retries a missing map every frame (2.8 M exceptions, the game at 0.27x).

Clone files are hardlinks into base/; this removes only the clone's entries
(stubs are new private files) and refuses to run on base/ itself.

**The keep-list is derived, never written down.**  From the tracked saves and
craft files (../saves/*.sfs, ../saves/craft/), the clone's own saves/default/
(.sfs and Ships/*/*.craft), and the parts the game spawns itself (EVA kerbals,
flags, asteroids, comets).  A save made on a part outside it cannot load until
the next launch.

One trap, paid for: **KSP writes `.` in a save where the part config spells
`_`** -- `Decoupler.0` in a save is `name = Decoupler_0` in its cfg -- and
craft files write `part = <name>_<flightID>`.  Names are compared normalised
(`.` -> `_`) and the patch names the config's own spelling.
"""
import argparse
import fnmatch
import os
import re
import shutil
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PATCH_DIR = "zzzKrpcLabPrune"
PATCH = os.path.join(PATCH_DIR, "prune.cfg")
BASE = os.path.join(os.environ.get("KSPFARM", os.path.expanduser("~/kspfarm")), "base")
TPR_DATA = os.path.join("TPR", "PluginData")
PARALLAX = os.path.join("TPR", "Configs", "NewParallaxConfigs")
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


def restore(directory):
    """Undo steps 2 and 3: relink from base/ every model the clone lacks and
    every TPR file it lacks or holds a stub for.  Returns the count."""
    gd = os.path.join(directory, "GameData")
    base = os.path.join(BASE, "GameData")
    n = 0
    targets = [(path, True) for path in walk(base, ".mu")]
    for sub in (TPR_DATA, PARALLAX):
        targets += [(path, False) for path in walk(os.path.join(base, sub), "")]
    for path, model in targets:
        dst = os.path.join(gd, os.path.relpath(path, base))
        if os.path.exists(dst) and os.path.samefile(path, dst):
            continue
        if model and os.path.exists(dst):
            continue          # a model the clone has a private copy of: leave it
        if model and not os.path.isdir(os.path.dirname(dst)):
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.lexists(dst):
            os.remove(dst)
        os.link(path, dst)
        n += 1
    return n


def png_stub(src):
    """A 4x4 PNG in the source's colour type (8 bits), mid grey, opaque."""
    with open(src, "rb") as f:
        head = f.read(33)
    ctype = head[25] if len(head) == 33 else 6
    chans = {0: 1, 2: 3, 4: 2, 6: 4}.get(ctype)
    if len(head) < 33 or head[24] != 8 or chans is None:
        ctype, chans = 6, 4
    px = bytes([128] * (chans - 1) + [255]) if chans in (2, 4) else bytes([128] * chans)
    raw = b"".join(b"\0" + px * 4 for _ in range(4))

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
                + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", 4, 4, 8, ctype, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def dds_stub(src):
    """A 4x4 one-mip DDS in the source's FourCC (DXT1, else a 16-byte block)."""
    with open(src, "rb") as f:
        h = bytearray(f.read(128))
    block = 8 if bytes(h[84:88]) == b"DXT1" else 16
    struct.pack_into("<I", h, 8, 0x1 | 0x2 | 0x4 | 0x1000 | 0x80000)
    struct.pack_into("<II", h, 12, 4, 4)
    struct.pack_into("<I", h, 20, block)
    struct.pack_into("<I", h, 28, 1)
    struct.pack_into("<I", h, 108, 0x1000)
    return bytes(h) + bytes(block)


TPR_REF = re.compile(r"^\s*(map|heightMap|multiplierMap|biomeMap)\s*=\s*TPR/PluginData/(\S+)")
PQS_MOD = re.compile(r"^\t\t\t\t([A-Za-z]\w*)\s*$")
HOME = re.compile(r"^0?3(-[12])?_")      # 03_Kerbin, 03-1_Mun, 03-2_Minmus


def tpr_keep(gd):
    """TPR files the physics reads: every biome map, and Kerbin's, the Mun's
    and Minmus's height and multiplier maps (stem, lower case)."""
    keep = set()
    kdir = os.path.join(gd, "TPR", "KopernicusFiles")
    for f in sorted(os.listdir(kdir)):
        home = HOME.match(f) is not None
        mod = ""
        for line in open(os.path.join(kdir, f), errors="replace"):
            line = line.split("//", 1)[0].rstrip() + "\n"
            m = PQS_MOD.match(line)
            if m:
                mod = m.group(1)
            r = TPR_REF.match(line)
            if not r:
                continue
            key, name = r.groups()
            if key == "biomeMap" or (home and (key in ("heightMap", "multiplierMap")
                                               or "Height" in mod)):
                keep.add(os.path.splitext(name)[0].lower())
    return keep


def stub_tpr(directory, dry):
    """Step 3.  Returns (files stubbed or deleted, MB they held)."""
    gd = os.path.join(directory, "GameData")
    data = os.path.join(gd, TPR_DATA)
    if not os.path.isdir(data):
        return 0, 0.0
    keep = tpr_keep(gd)
    n, size = 0, 0
    for f in sorted(os.listdir(data)):
        path = os.path.join(data, f)
        stem, ext = os.path.splitext(f)
        if not os.path.isfile(path) or stem.lower() in keep \
                or ext.lower() not in (".png", ".dds"):
            continue
        if os.path.getsize(path) < 1024:
            continue          # already a stub
        n += 1
        size += os.path.getsize(path)
        if not dry:
            stub = png_stub(path) if ext.lower() == ".png" else dds_stub(path)
            os.remove(path)   # the hardlink into base/; the stub is private
            with open(path, "wb") as out:
                out.write(stub)
    par = os.path.join(gd, PARALLAX)
    if os.path.isdir(par):
        for p in walk(par, ""):
            size += os.path.getsize(p)
            n += 1
        if not dry:
            shutil.rmtree(par)
    return n, size / 1048576.0


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
    a = p.parse_args()

    directory = a.instance if os.path.isdir(a.instance) \
        else os.path.join(HERE, "ksp" + str(a.instance))
    if not os.path.isdir(directory):
        raise SystemExit("no such instance: %s" % directory)
    if os.path.realpath(directory) == os.path.realpath(BASE):
        raise SystemExit("refusing to prune base/ -- prune a clone")
    target = os.path.join(directory, "GameData", PATCH)

    if a.undo:
        if os.path.exists(target):
            os.remove(target)
            os.rmdir(os.path.dirname(target))
            print("removed %s" % target)
        print("relinked %d files from base/" % restore(directory))
        return 0
    if not a.dry_run and not a.report:
        restore(directory)

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
        print("TPR files kept real: %s" % ", ".join(sorted(tpr_keep(
            os.path.join(directory, "GameData")))))
        for k in keep:
            print("  keep %s" % k)
        return 0
    verb = "would " if a.dry_run else ""
    if stale or not db["PART"]:
        print("cache lacks kept parts: models kept this boot")
        gone = []
    else:
        kept = {"PART": set(keep) | {n for n in db["PART"] if any(
                    fnmatch.fnmatch(n, g) for g in ALWAYS)},
                "INTERNAL": internals, "PROP": props}
        gone, size = prune_models(directory, db, kept, a.dry_run)
        print("models: %sdelete %d (%.0f MB on disk)" % (verb, len(gone), size / 1048576.0))
    ntex, mb = stub_tpr(directory, a.dry_run)
    print("TPR: %sstub %d visual-only files (%.0f MB)" % (verb, ntex, mb))
    text = patch_text(keep, internals, props, prune_spaces=bool(db["PART"]))
    if a.dry_run:
        sys.stdout.write(text)
        return 0
    old = open(target).read() if os.path.exists(target) else None
    summary = "%d parts, %d models deleted, %d TPR files stubbed" % (
        len(keep), len(gone), ntex)
    if old == text:
        print("patch unchanged; %s" % summary)
        return 0
    os.makedirs(os.path.dirname(target), exist_ok=True)
    tmp = target + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, target)
    print("patch written; %s" % summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
