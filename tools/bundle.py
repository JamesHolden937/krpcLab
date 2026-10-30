#!/usr/bin/env python3
"""Export every autopilot as one runnable file, comments stripped.

TEMPORARY TEST HARNESS -- not part of the flight software, and it does not
change any of it: the bundle is the same modules, comments and docstrings
removed (``tools/pystrip.py``, which proves each one parses to the same
program), with an importer in front of them.

    ./tools/bundle.py                         # -> dist/krpcLab.py
    ./tools/bundle.py --if-stale -q           # ... only if a source changed
    ./tools/bundle.py -o /tmp/fly.py          # -> /tmp/fly.py

**Everything flies the bundle.**  ``run.sh``, the launcher's buttons and the
farm harnesses run ``dist/krpcLab.py``, each calling ``ensure`` first so an
edited source rebuilds it; the ``post-commit`` hook in ``tools/githooks``
rebuilds and selftests it on every commit (``run.sh`` points git at that
directory).
    python3 dist/krpcLab.py                   # the in-game launcher
    python3 dist/krpcLab.py --pilot spaceplane --set LOG_INTERVAL_UT=1

``--pilot`` takes the names ``run.sh`` does (``PILOTS`` in
``common/launcher.py``); with none, the launcher's buttons start each
autopilot from this same file.  Logs go to ``logs/`` beside the file.

**Why a loader and not a concatenation.**  Pasting the modules end to end
means choosing an order, and the order is a second copy of the dependency
graph that nothing checks -- it goes stale the first time a module grows an
import.  Here every module's source is carried and served by a
``sys.meta_path`` finder, so Python resolves the imports itself, in whatever
order they actually happen, exactly as it does from the tree.  ``from . import
vec`` keeps working because the package modules are real packages with a
``__path__``, and each module's ``__file__`` is where it would sit beside the
bundle, so paths derived from it land beside the file.

Nobody reads the file -- the tree is the readable copy -- so it is as small
as it can be made: comments and docstrings stripped (``pystrip``), every
name nothing outside the bundle can reach renamed (``pyminify``; ``outsiders``
and ``foreign`` say what can), builtins and other modules' functions
read through one-letter aliases (``pyalias``; ``hoistable`` says which
functions), whitespace squeezed (``pycompact``), each proven by the
parser -- and every word worth it written as a short code from a
dictionary the loader expands in memory before anything compiles
(``dictionary``), so the file is a quarter smaller and what runs is the
same plain text.  Not compressed: zip is the last resort.

What the bundle still needs from the outside world is ``krpc`` (and, for the
ascent, MechJeb with KRPC.MechJeb in the game).  The tests and tools are not
carried: they belong to the tree.
"""
import argparse
import ast
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pyalias  # noqa: E402
import pycompact  # noqa: E402
import pyminify  # noqa: E402
import pyshort  # noqa: E402
import pystrip  # noqa: E402

# Every package an autopilot in ``common.launcher.PILOTS`` imports.
PACKAGES = ["common", "spaceplane", "boosterland", "ascent"]
NAME = "krpcLab.py"


def collect(packages):
    """Every module of every package, as ``name -> (is_package, source)``."""
    modules = {}
    for package in packages:
        directory = os.path.join(ROOT, package)
        if not os.path.isdir(directory):
            raise SystemExit("no such package: %s" % package)
        for entry in sorted(os.listdir(directory)):
            if not entry.endswith(".py"):
                continue
            path = os.path.join(directory, entry)
            with open(path, encoding="utf-8") as handle:
                source = handle.read()
            if entry == "__init__.py":
                modules[package] = (True, source)
            else:
                modules["%s.%s" % (package, entry[:-3])] = (False, source)
        # A package with no ``__init__.py`` still has to exist as one.
        modules.setdefault(package, (True, ""))
    return modules


ASSIGNED = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=(?!=)")
# Files whose words cannot name code: prose, saves, craft, binaries.
NOT_CODE = (".md", ".pdf", ".sfs", ".craft", ".loadmeta", ".bin", ".u8",
            ".png", ".jpg", ".svg")


def outsiders(modules):
    """``(name -> every identifier some other tracked file mentions, every
    identifier a file outside the bundle mentions, every word of the shell
    scripts among them, every ``name=`` in those that are not Python, and the Python ones'
    sources -- which
    ``pyminify.free_parameters`` reads for the calls they make)``: what ``pyminify`` must
    not rename in that module, and in any, since a test, a shell script or
    another module may reach it by name.  Python outside the bundled
    packages and their tests is not read (see below)."""
    try:
        listed = subprocess.run(["git", "ls-files"], cwd=ROOT, check=True,
                                capture_output=True, text=True).stdout.split()
    except (OSError, subprocess.CalledProcessError):
        raise SystemExit("bundle: git ls-files failed; cannot tell which "
                         "names other files use")
    bundled = {name.replace(".", "/") + ("/__init__.py" if is_package
                                         else ".py")
               for name, (is_package, _) in modules.items()}
    tests = [rel for rel in listed if "/tests/" in rel and rel.endswith(".py")]
    tested = set()
    for rel in tests:
        with open(os.path.join(ROOT, rel), encoding="utf-8") as handle:
            tested |= set(pyminify.WORD.findall(handle.read()))
    words = {}
    for rel in listed:
        if rel.endswith(NOT_CODE):
            continue
        # A tool imports the tree, never the bundle, so its names do not
        # bind it -- unless a test imports the tool: under ``--suite`` that
        # tool then reads the bundle.
        stem = os.path.splitext(os.path.basename(rel))[0]
        if (rel.endswith(".py") and rel not in bundled and rel not in tests
                and stem not in tested):
            continue
        try:
            with open(os.path.join(ROOT, rel), encoding="utf-8") as handle:
                words[rel] = set(pyminify.WORD.findall(handle.read()))
        except (OSError, UnicodeDecodeError):
            continue
    beyond, beyond_other, keywords, callers = set(), set(), set(), []
    for rel, found in words.items():
        if rel in bundled:
            continue
        beyond |= found
        script = rel.endswith(".sh") or "." not in os.path.basename(rel)
        if script:
            beyond_other |= found   # a script can run Python
        if rel.endswith(".py"):
            with open(os.path.join(ROOT, rel), encoding="utf-8") as handle:
                callers.append(handle.read())
        elif script:
            # Only Python inlined in a script can pass a keyword: name=
            # (the mod's C# assigns names too, and runs no Python).
            with open(os.path.join(ROOT, rel), encoding="utf-8") as handle:
                keywords |= set(ASSIGNED.findall(handle.read()))
    every = {}
    for found in words.values():
        for w in found:
            every[w] = every.get(w, 0) + 1
    out = {}
    for name, (is_package, _) in modules.items():
        rel = name.replace(".", "/") + ("/__init__.py" if is_package else ".py")
        own = words.get(rel, set())
        out[name] = {w for w, n in every.items() if n - (w in own) > 0}
    return out, beyond, beyond_other, keywords, callers


# Where kRPC's service definitions are: the farm's game, else the live one.
KRPC_DEFINITIONS = [os.path.join(ROOT, "testInstances", "base", "GameData",
                                 "kRPC"),
                    os.path.expanduser("~/Kerbal Space Program/GameData/kRPC")]


def _snake(word):
    return "".join("_" + c.lower() if c.isupper() and i else c.lower()
                   for i, c in enumerate(word))


def foreign(sources):
    """``(names, complete)``: every attribute name an object we did not
    write may carry -- kRPC's services (from the game's ``KRPC*.json``, in
    the client's snake_case), the ``krpc`` client's classes, and every class
    of every module the sources import, the standard library's included.
    ``complete`` is False when no service definitions were found: then only
    private attributes may be renamed."""
    import importlib
    import json
    out = set()

    def members(obj):
        for k in dir(obj):
            out.add(k)
            try:
                v = getattr(obj, k)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(v, type):
                out.update(dir(v))

    for t in (object, int, float, complex, str, bytes, list, dict, tuple, set,
              frozenset, type, BaseException, Exception, KeyError,
              property, staticmethod, classmethod):
        members(t)
    wanted = {"io", "threading", "logging", "socketserver", "collections",
              "collections.abc", "enum", "dataclasses", "krpc", "krpc.client",
              "krpc.stream", "krpc.event", "krpc.types", "krpc.service"}
    for source in sources:
        for n in ast.walk(ast.parse(source)):
            if isinstance(n, ast.Import):
                wanted |= {al.name for al in n.names}
            elif isinstance(n, ast.ImportFrom) and n.module and not n.level:
                wanted.add(n.module)
    for mod in sorted(wanted):
        try:
            members(importlib.import_module(mod))
        except Exception:  # noqa: BLE001  (our own packages, absent extras)
            continue
    complete = False
    for directory in KRPC_DEFINITIONS:
        if not os.path.isdir(directory):
            continue
        for entry in sorted(os.listdir(directory)):
            if not entry.endswith(".json"):
                continue
            with open(os.path.join(directory, entry), encoding="utf-8") as h:
                text = json.load(h)

            def walk(v):
                if isinstance(v, dict):
                    for k, x in v.items():
                        if k != "documentation":
                            out.update(_snake(p) for p in k.split("_"))
                            walk(x)
                elif isinstance(v, list):
                    for x in v:
                        walk(x)
                elif isinstance(v, str):
                    out.update(_snake(p) for p in v.split("_"))
            walk(text)
            complete = True
        if complete:
            break
    return out, complete


def stripped(modules):
    """``collect``'s modules with comments and docstrings removed, every
    function's locals and nested functions renamed a, b, ..., and every
    module name and private attribute no other file mentions
    (``pyminify``), statements rewritten shorter (``pyshort``), builtins
    and repeated literals read through one-letter aliases (``pyalias``), as are other modules' functions where
    ``hoistable`` allows, and the whitespace squeezed out (``pycompact``):
    what the bundle carries.  Each raises
    rather than change a program."""
    outside, beyond, beyond_other, keywords, callers = outsiders(modules)
    try:
        bare = {name: pystrip.strip(source, name)
                for name, (_, source) in modules.items()}
        names, complete = foreign(bare.values())
        attributes = pyminify.attribute_plan(
            bare, beyond, names, private_only=not complete)
        free = pyminify.free_parameters(bare.values(), keywords,
                                         callers)
        # What can reach ``module.name``: an attribute read, a ``from module``
        # import or a string (``mock.patch("m.name")``, not a docstring) in
        # any Python, and any word of a shell script.
        reached, imported = set(beyond_other), {}
        sources = [(name, source) for name, source in bare.items()]
        sources += [(None, source) for source in callers]
        for importer, source in sources:
            tree = ast.parse(source)
            docs = {id(n.value) for n in ast.walk(tree)
                    if isinstance(n, ast.Expr)}
            for n in ast.walk(tree):
                if isinstance(n, ast.Attribute):
                    reached.add(n.attr)
                elif isinstance(n, ast.ImportFrom):
                    origin = _origin(modules, importer, n)
                    names = {al.name for al in n.names}
                    if "*" in names:
                        raise SystemExit("bundle: %s star-imports %s"
                                         % (importer, origin))
                    imported.setdefault(origin, set()).update(names)
                elif (isinstance(n, ast.Constant) and isinstance(n.value, str)
                      and id(n) not in docs):
                    reached |= set(pyminify.WORD.findall(n.value))
        minified = {name: pyminify.minify(bare[name], name, outside[name],
                                          attributes, free,
                                          reached | imported.get(name, set()))
                    for name in modules}
        hoist = hoistable(modules, minified, callers, beyond_other)
        return {name: (is_package, compacted(name, pyalias.alias(
                    pyshort.shorten(minified[name], name), name,
                    hoist.get(name))))
                for name, (is_package, _) in modules.items()}
    except (pystrip.StripError, pyminify.MinifyError,
            pyshort.ShortError, pyalias.AliasError) as exc:
        raise SystemExit("cannot strip: %s" % exc)


def _origin(modules, importer, node):
    """The module a ``from ... import`` reads, its dots resolved against
    ``importer`` (``None``: a file outside the bundle, read as written)."""
    origin = node.module or ""
    if node.level and importer:
        base = importer.split(".")
        if not modules[importer][0]:
            base = base[:-1]
        base = base[:len(base) - node.level + 1]
        origin = ".".join(base + ([origin] if origin else []))
    return origin


def _imported_modules(modules, name, tree):
    """``({bound name: module}, every module named)`` for ``name``'s
    imports: the first only for top-level ones, the second anywhere (with
    the packages each passes through) -- what ``hoistable`` needs to know
    which modules ``name`` reaches while it is itself being imported."""
    import importlib
    import types
    bound, named = {}, set()

    def module_named(full):
        if full in modules:
            return True
        parent, _, leaf = full.rpartition(".")
        if not parent or parent.split(".")[0] in modules:
            return False
        try:
            return isinstance(getattr(importlib.import_module(parent), leaf,
                                      None), types.ModuleType)
        except Exception:  # noqa: BLE001
            return False
    top = {id(st) for st in tree.body}
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            for al in n.names:
                parts = al.name.split(".")
                named |= {".".join(parts[:i + 1]) for i in range(len(parts))}
                if id(n) in top:
                    bound[al.asname or parts[0]] = (
                        al.name if al.asname else parts[0])
        elif isinstance(n, ast.ImportFrom) and n.module != "__future__":
            origin = _origin(modules, name, n)
            parts = origin.split(".")
            named |= {".".join(parts[:i + 1]) for i in range(len(parts))}
            for al in n.names:
                full = "%s.%s" % (origin, al.name) if origin else al.name
                if module_named(full):
                    named.add(full)
                    if id(n) in top:
                        bound[al.asname or al.name] = full
    return bound, named


def hoistable(modules, sources, callers, script_words):
    """``{module: {bound name: attributes}}``: the ``module.attr`` reads
    ``pyalias`` may make once, on the line after the import, instead of at
    every use.  That is the same object only if nothing ever rebinds it, so
    an attribute is refused when anything stores or deletes an attribute of
    that name, or names it in a string (``mock.patch.object(m, "attr")``,
    ``hasattr(os, "attr")``) or a shell script.  Beyond that it must be

    - of a bundled module: a function or class it defines once at its top
      level, and that module must not import this one, even through others
      (a cycle would read it half built);
    - of the standard library or ``krpc``: public, and a function, class or
      module when the bundler imports it."""
    import importlib
    import types
    trees = {name: ast.parse(source) for name, source in sources.items()}
    refused = set(script_words)
    for tree in list(trees.values()) + [ast.parse(c) for c in callers]:
        for n in ast.walk(tree):
            if isinstance(n, ast.Attribute) and not isinstance(n.ctx,
                                                               ast.Load):
                refused.add(n.attr)
            elif isinstance(n, ast.Constant) and isinstance(n.value, str):
                refused |= set(pyminify.WORD.findall(n.value))
    imports = {name: _imported_modules(modules, name, tree)
               for name, tree in trees.items()}
    defined = {}
    for name, tree in trees.items():
        counts = pyalias.bind_counts(tree)
        defined[name] = {st.name for st in tree.body
                         if isinstance(st, (ast.FunctionDef,
                                            ast.AsyncFunctionDef,
                                            ast.ClassDef))
                         and counts.get(st.name) == 1}

    def reaches(start):
        seen, stack = set(), [start]
        while stack:
            m = stack.pop()
            if m in seen or m not in imports:
                continue
            seen.add(m)
            stack.extend(imports[m][1])
        return seen
    reach = {name: reaches(name) for name in trees}
    stdlib = set(getattr(sys, "stdlib_module_names", ())) | {"krpc"}
    foreign_attrs = {}

    def foreign(full):
        if full not in foreign_attrs:
            ok = set()
            if full.split(".")[0] in stdlib:
                try:
                    mod = importlib.import_module(full)
                except Exception:  # noqa: BLE001
                    mod = None
                for k in dir(mod) if mod else ():
                    v = getattr(mod, k, None)
                    if not k.startswith("_") and isinstance(v, (
                            types.FunctionType, types.BuiltinFunctionType,
                            type, types.ModuleType)):
                        ok.add(k)
            foreign_attrs[full] = ok
        return foreign_attrs[full]
    out = {}
    for name in trees:
        plan = {}
        for local, full in imports[name][0].items():
            if full in modules:
                if name in reach.get(full, {name}):
                    continue
                attrs = defined.get(full, set())
            else:
                attrs = foreign(full)
            attrs = attrs - refused
            if attrs:
                plan[local] = attrs
        if plan:
            out[name] = plan
    return out


def compacted(name, source):
    """``pycompact``, raising ``SystemExit`` like the other two."""
    try:
        return pycompact.compact(source, name)
    except pycompact.CompactError as exc:
        raise SystemExit("cannot compact: %s" % exc)


# The bundle is plain text: a ``#!`` line, this loader (compacted like
# everything else), then ``_P``, one raw string holding every module as
# ``name `is_package` source``, joined by ``$`` -- two printable characters
# no compacted source contains (``payload`` refuses one that does) -- with
# every word the dictionary ``_D`` lists written as ``&`` and a one- or
# two-character code (``dictionary``); ``_X`` writes the words back, in
# memory, before anything is split or compiled, so what runs is the plain
# text.  Not compressed: unpacking would cost every start.  ``_B`` is a ``sys.meta_path`` finder
# over the sources: packages get a ``__path__`` so relative imports
# resolve, each module a ``__file__`` beside the bundle, and ``get_source``
# serves tracebacks and ``inspect``.  ``_main`` is ``run.sh``: ``--pilot
# NAME`` or ``-m MODULE`` runs that module's ``main``, nothing runs the
# launcher, and ``KRPCLAB_BUNDLE`` tells the launcher's buttons to start
# each autopilot from this same file.  ``_P`` is assigned at the end, so
# the loader's functions read it only once it exists.
LOADER = r"""import importlib.abc as A,importlib.machinery as M,os,sys
Z=os.path.abspath(globals().get("__file__") or sys.argv[0])
H=os.path.dirname(Z)
S={}
class _B(A.MetaPathFinder,A.Loader):
    def find_spec(s,n,p=None,t=None):
        if n not in S:return None
        x=M.ModuleSpec(n,s,is_package=S[n][0])
        if S[n][0]:x.submodule_search_locations=[]
        return x
    def create_module(s,x):return None
    def exec_module(s,m):
        k,c=S[m.__name__];q=m.__name__.split(".")
        m.__file__=os.path.join(H,*(q+["__init__.py"] if k else q[:-1]+[q[-1]+".py"]))
        exec(compile(c,m.__file__,"exec"),m.__dict__)
    def get_source(s,n):return S[n][1]
def _X(t):
    a,k=_A,_K;m=dict(zip(list(a[:k])+[x+y for x in a[k:] for y in a],_D.split(" ")));o=t.split("&");r=[o[0]];f=set(a[:k])
    for p in o[1:]:
        n=1 if p[0] in f else 2;r.append(m[p[:n]]+p[n:])
    return "".join(r)
def _main(a):
    import importlib
    from common import launcher
    os.environ["KRPCLAB_BUNDLE"]=Z;m,r="common.launcher",list(a)
    for f in("--pilot","-m"):
        for i,g in enumerate(r):
            v=None
            if g==f and i+1<len(r):v,d=r[i+1],2
            elif g.startswith(f+"="):v,d=g.split("=",1)[1],1
            if v is None:continue
            m=launcher.module_for(v) if f=="--pilot" else v
            if m is None:sys.stderr.write("unknown pilot: %s\n"%v);return 2
            del r[i:i+d];break
    sys.argv=[Z]+r;return importlib.import_module(m).main(r)
"""
TAIL = r"""for e in _X(_P).split("$"):
    n,k,c=e.split("`",2);S[n]=(k=="1",c)
sys.meta_path.insert(0,_B())
if __name__=="__main__":sys.exit(_main(sys.argv[1:]))
"""
SHEBANG = "#!/usr/bin/env python3\n"


def pilots():
    """``(name, module)`` per autopilot, read from the launcher's source."""
    tree = __import__("ast").parse(open(os.path.join(
        ROOT, "common", "launcher.py"), encoding="utf-8").read())
    for node in tree.body:
        if (isinstance(node, __import__("ast").Assign)
                and getattr(node.targets[0], "id", "") == "PILOTS"):
            return [(e.elts[1].value, e.elts[2].value)
                    for e in node.value.elts]
    raise SystemExit("no PILOTS in common/launcher.py")


DELIMITER = "\x27" * 3
RECORD, FIELD = "$", "`"       # as ``TAIL`` splits them
ESCAPE = "&"                   # opens a dictionary code (``_X``)
# A code's characters: printable, never the escape, a single quote
# (three would close ``_P``) or a backslash.
ALPHABET = "".join(c for c in map(chr, range(33, 127))
                   if c not in ESCAPE + "\x27\\")
WORDS = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def payload(modules):
    """Every module in one string, in the format ``TAIL`` splits."""
    parts = []
    for name in sorted(modules):
        is_package, source = modules[name]
        if (RECORD in source or FIELD in source or DELIMITER in source
                or ESCAPE in source):
            raise SystemExit("%s contains %r, %r, %r or %r, which separate "
                             "the bundle's modules or open a dictionary "
                             "code: pick others"
                             % (name, RECORD, FIELD, DELIMITER, ESCAPE))
        parts.append("%s%s%d%s%s" % (name, FIELD, is_package, FIELD, source))
    # Ends in the last module's newline, so no backslash meets the quotes.
    return RECORD.join(parts)


def codes(k):
    """Every dictionary code in order: ``k`` single characters, then two
    for each of the rest (``_X`` builds the same list)."""
    return list(ALPHABET[:k]) + [x + y for x in ALPHABET[k:]
                                 for y in ALPHABET]


def dictionary(text):
    """``(k, words)``: the words worth a code in ``text``, most frequent
    first, and how many of them get one character.  A word earns its entry
    when its uses save more than the entry costs (its letters and a space);
    the split between one- and two-character codes is the one that saves
    most."""
    count = {}
    for w in WORDS.findall(text):
        if len(w) > 2:
            count[w] = count.get(w, 0) + 1
    ranked = sorted(count, key=lambda w: (-count[w], -len(w), w))
    best = (0, 0, [])
    for k in range(1, len(ALPHABET)):
        room = k + (len(ALPHABET) - k) * len(ALPHABET)
        chosen, saved = [], 0
        for w in ranked:
            if len(chosen) >= room:
                break
            size = 1 if len(chosen) < k else 2
            gain = count[w] * (len(w) - 1 - size) - (len(w) + 1)
            if gain > 0:
                chosen.append(w)
                saved += gain
        if saved > best[0]:
            best = (saved, k, chosen)
    return best[1], best[2]


def encode(text, k, words):
    """``text`` with each listed word written as its code."""
    table = dict(zip(words, (ESCAPE + c for c in codes(k))))
    return WORDS.sub(lambda m: table.get(m.group(0), m.group(0)), text)


def expand(text, k, words):
    """``_X``'s work, for reading a bundle back."""
    table = dict(zip(codes(k), words))
    single = set(ALPHABET[:k])
    parts = text.split(ESCAPE)
    out = [parts[0]]
    for p in parts[1:]:
        n = 1 if p[0] in single else 2
        out.append(table[p[:n]] + p[n:])
    return "".join(out)


def unpack(path):
    """``{name: (is_package, source)}`` and the loader's text, read back out
    of a written bundle the way the loader reads them."""
    import ast
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    found, want = {}, {"_P", "_D", "_K", "_A"}
    for node in ast.parse(text).body:
        if (isinstance(node, ast.Assign)
                and getattr(node.targets[0], "id", "") in want):
            found[node.targets[0].id] = node.value.value
    if set(found) != want:
        raise SystemExit("bundle lacks %s" % ", ".join(sorted(want
                                                          - set(found))))
    if found["_A"] != ALPHABET:
        raise SystemExit("bundle's code alphabet is not this bundler's")
    blob = expand(found["_P"], found["_K"], found["_D"].split(" "))
    modules = {}
    for entry in blob.split(RECORD):
        name, kind, source = entry.split(FIELD, 2)
        modules[name] = (kind == "1", source)
    return modules, text[:text.index("_P=r")]


def build():
    """The bundle's text, its file name and its module count."""
    modules = stripped(collect(PACKAGES))
    for name, module in pilots():
        if module not in modules:
            raise SystemExit("pilot %s needs %s, which no bundled package "
                             "has: add its package to PACKAGES"
                             % (name, module))
    text = payload(modules)
    k, words = dictionary(text)
    body = (SHEBANG + compacted("loader", LOADER) + "_P=r" + DELIMITER
            + encode(text, k, words) + DELIMITER + "\n"
            + "_D=%r\n_K=%d\n_A=r%s%s%s\n" % (" ".join(words), k, DELIMITER,
                                              ALPHABET, DELIMITER)
            + compacted("tail", TAIL))
    return body, NAME, len(modules)


def verify(path, modules):
    """Read the written bundle back and check every module came out intact
    and no comment went in.

    ``modules`` is what was meant to go in (``stripped(collect(...))``).
    Each source is compared byte for byte: a claim nothing checks is the
    kind this project keeps paying for.
    """
    import ast
    import io
    import tokenize
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    if not text.startswith(SHEBANG):
        raise SystemExit("bundle does not start with its #! line")
    hidden = [c for c in set(text) if c != "\n" and (
        ord(c) < 32 or 127 <= ord(c) < 160)]
    if hidden:
        raise SystemExit("bundle carries control characters %r, which "
                         "editors show as boxes" % sorted(hidden))
    seen, loader = unpack(path)
    for module, (is_package, source) in modules.items():
        got = seen.get(module)
        if got is None:
            raise SystemExit("%s is missing from the bundle" % module)
        if got != (is_package, source):
            raise SystemExit("%s came out of the bundle changed" % module)
    extra = set(seen) - set(modules)
    if extra:
        raise SystemExit("bundle carries modules that are not in the tree: %s"
                         % ", ".join(sorted(extra)))
    comments = [t for t in tokenize.generate_tokens(io.StringIO(text).readline)
                if t.type == tokenize.COMMENT and t.start != (1, 0)]
    if comments:
        raise SystemExit("bundle carries a comment at line %d"
                         % comments[0].start[0])
    for name, (_, source) in seen.items():
        if any(t.type == tokenize.COMMENT for t in tokenize.generate_tokens(
                io.StringIO(source).readline)):
            raise SystemExit("%s carries a comment" % name)
        if pystrip._docstring_nodes(ast.parse(source), keep_read=False):
            raise SystemExit("%s carries a docstring (does it read "
                             "__doc__?)" % name)
    return len(seen)


def selftest(path):
    """Run the bundle with a stub ``krpc`` and check it really assembled.

    A bundler that emits a file nobody runs is a bundler that emits a broken
    file.  This flies nothing: it imports every pilot's module out of the
    bundle, checks each has ``main``, that the paths land beside the file (or, in a
    checkout's ``dist/``, in the tree above it),
    and that ``--pilot NAME --help`` gets as far as each one's arguments.
    """
    probe = """import io, os, sys, types, contextlib
sys.modules['krpc'] = types.ModuleType('krpc')
sys.argv = [{path!r}]
import runpy
space = runpy.run_path({path!r}, run_name='bundled')
from common import launcher, paths
here = os.path.dirname({path!r})
if (os.path.basename(here) == 'dist' and os.path.exists(
        os.path.join(os.path.dirname(here), 'common', 'paths.py'))):
    here = os.path.dirname(here)    # a checkout's dist/: the tree is the root
assert paths.ROOT == here, paths.ROOT
for _, name, module in launcher.PILOTS:
    mod = __import__(module, fromlist=['x'])
    assert hasattr(mod, 'main'), module + ' has no main()'
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            space['_main'](['--pilot', name, '--help'])
    except SystemExit as exc:
        assert exc.code in (0, None), (name, exc.code)
    assert 'usage' in out.getvalue(), name + ': no --help'
loaded = [m for m in sys.modules if m.split('.')[0] in
          ('common', 'boosterland', 'spaceplane', 'ascent')]
print('selftest ok: %d pilots, %d modules loaded'
      % (len(launcher.PILOTS), len(loaded)))
""".format(path=path)
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as tmp:
        tmp.write(probe)
        probe_path = tmp.name
    try:
        done = subprocess.run([sys.executable, probe_path],
                              capture_output=True, text=True, cwd="/")
    finally:
        os.unlink(probe_path)
    if done.returncode != 0:
        sys.stderr.write(done.stdout + done.stderr)
        raise SystemExit("bundle selftest failed")
    return done.stdout.strip()


DEFAULT = os.path.join(ROOT, "dist", NAME)


def sources():
    """Every file the bundle is made from, the bundler included."""
    files = [os.path.abspath(m.__file__) for m in (
        sys.modules[__name__], pystrip, pyminify, pyshort, pyalias,
        pycompact)]
    for package in PACKAGES:
        directory = os.path.join(ROOT, package)
        files += [os.path.join(directory, e) for e in os.listdir(directory)
                  if e.endswith(".py")]
    return files


def stale(out=DEFAULT):
    """Is ``out`` missing or older than any file it is made from?"""
    try:
        built = os.path.getmtime(out)
    except OSError:
        return True
    return any(os.path.getmtime(f) > built for f in sources())


def write(out=DEFAULT, test=True):
    """Build, write atomically, read back and (``test``) selftest.

    Under a lock: a farm batch starts six harnesses at once, and each one
    asks for a fresh bundle."""
    import fcntl
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out + ".lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        body, _, count = build()
        tmp = "%s.%d.tmp" % (out, os.getpid())
        with open(tmp, "w", encoding="utf-8") as handle:
            handle.write(body)
        os.chmod(tmp, 0o755)
        try:
            verify(tmp, stripped(collect(PACKAGES)))
            note = selftest(tmp) if test else ""
        except BaseException:
            os.unlink(tmp)
            raise
        # Renamed into place, so a flight starting now reads the old file
        # or the new one, never half of one.
        os.replace(tmp, out)
    return out, count, note


def ensure(out=DEFAULT):
    """The bundle's path, rebuilt first if any source is newer: what every
    launcher flies, so an uncommitted edit flies too."""
    if stale(out):
        write(out, test=False)
    return out


def suite(path):
    """Every bundled package's offline tests with the bundle's modules in
    place of the tree's: the tests come from the tree, everything they test
    from ``path``.  What proves the renaming kept every program the same."""
    if os.path.dirname(os.path.abspath(path)) == ROOT:
        raise SystemExit("bundle: a suite bundle in the root would load the "
                         "tree's modules as its own; write it elsewhere")
    probe = """import os, sys, types, unittest
sys.path.insert(0, {root!r})
import runpy
runpy.run_path({path!r}, run_name='bundled')
import importlib
for pkg in {packages!r}:
    mod = importlib.import_module(pkg)
    mod.__path__.append(os.path.join({root!r}, pkg))
    assert mod.__file__.startswith(os.path.dirname({path!r})), mod.__file__
tests = unittest.TestSuite(
    unittest.defaultTestLoader.discover(os.path.join({root!r}, pkg, 'tests'),
                                        top_level_dir={root!r})
    for pkg in {packages!r} if os.path.isdir(os.path.join({root!r}, pkg, 'tests')))
result = unittest.TextTestRunner(verbosity=0).run(tests)
for pkg in {packages!r}:
    for name, mod in list(sys.modules.items()):
        if (name == pkg or name.startswith(pkg + '.')) and '.tests' not in name:
            f = getattr(mod, '__file__', '') or ''
            assert not f.startswith({root!r} + os.sep + pkg), (
                name + ' came from the tree')
import inspect
text_tests, real = [], []
for test, trace in result.failures + result.errors:
    try:
        body = inspect.getsource(type(test))
    except (OSError, TypeError):
        body = ''
    reads_source = any(k in body for k in ('getsource', '__file__', 'open('))
    (text_tests if reads_source else real).append(test.id())
for name in real:
    print('REAL FAILURE', name)
sys.stderr.write('%d tests, %d behaviour failures, %d source-text tests '
                 'excused\\n' % (result.testsRun, len(real), len(text_tests)))
sys.exit(1 if real else 0)
""".format(root=ROOT, path=path, packages=PACKAGES)
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as tmp:
        tmp.write(probe)
        probe_path = tmp.name
    try:
        done = subprocess.run([sys.executable, probe_path], cwd=ROOT,
                              capture_output=True, text=True,
                              env=dict(os.environ, KRPCLAB_BUNDLE_SUITE="1"))
    finally:
        os.unlink(probe_path)
    tail = (done.stderr.strip().splitlines() or ["?"])
    if done.returncode != 0:
        sys.stderr.write(done.stdout + done.stderr)
        raise SystemExit("the suite fails against the bundle")
    return "suite ok against the bundle: " + " ".join(tail[-3:])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-o", "--out", default=None,
                        help="output file (default: dist/%s)" % NAME)
    parser.add_argument("--no-selftest", action="store_true",
                        help="skip running the result to check it works")
    parser.add_argument("--if-stale", action="store_true",
                        help="only if a source is newer than the bundle")
    parser.add_argument("-q", "--quiet", action="store_true")
    parser.add_argument("--suite", action="store_true",
                        help="also run every offline test against the bundle")
    args = parser.parse_args(argv)

    out = os.path.abspath(args.out or DEFAULT)
    if args.if_stale and not stale(out):
        if not args.quiet:
            print("%s  up to date" % out)
        return 0
    out, count, note = write(out, test=not args.no_selftest)
    if args.suite:
        note = (note + "  " if note else "") + suite(out)
    if not args.quiet:
        print("%s  %d modules  %.0f kB%s" % (
            out, count, os.path.getsize(out) / 1024.0,
            "  " + note if note else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
