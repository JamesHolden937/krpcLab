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
as it can be made: comments and docstrings stripped (``pystrip``), locals
renamed (``pyminify``), whitespace squeezed (``pycompact``), each proven by
the parser.  Not compressed: unpacking would cost every start.

What the bundle still needs from the outside world is ``krpc`` (and, for the
ascent, MechJeb with KRPC.MechJeb in the game).  The tests and tools are not
carried: they belong to the tree.
"""
import argparse
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pycompact  # noqa: E402
import pyminify  # noqa: E402
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


def stripped(modules):
    """``collect``'s modules with comments and docstrings removed, every
    function's locals renamed a, b, ... (``pyminify``) and the whitespace
    squeezed out (``pycompact``): what the bundle carries.  Each raises
    rather than change a program."""
    try:
        return {name: (is_package, compacted(name, pyminify.minify(
                    pystrip.strip(source, name), name)))
                for name, (is_package, source) in modules.items()}
    except (pystrip.StripError, pyminify.MinifyError) as exc:
        raise SystemExit("cannot strip: %s" % exc)


def compacted(name, source):
    """``pycompact``, raising ``SystemExit`` like the other two."""
    try:
        return pycompact.compact(source, name)
    except pycompact.CompactError as exc:
        raise SystemExit("cannot compact: %s" % exc)


# The bundle is plain text: a ``#!`` line, this loader (compacted like
# everything else), then ``_P``, one raw string holding every module as
# ``name \x1f is_package \x1f source``, joined by ``\x1e``.  Not compressed:
# unpacking would cost every start.  ``_B`` is a ``sys.meta_path`` finder
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
TAIL = r"""for e in _P.split("\x1e"):
    n,k,c=e.split("\x1f",2);S[n]=(k=="1",c)
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


def payload(modules):
    """Every module in one string, in the format ``TAIL`` splits."""
    parts = []
    for name in sorted(modules):
        is_package, source = modules[name]
        if "\x1e" in source or "\x1f" in source or DELIMITER in source:
            raise SystemExit("%s contains a separator the bundle's raw "
                             "string cannot carry" % name)
        parts.append("%s\x1f%d\x1f%s" % (name, is_package, source))
    # Ends in the last module's newline, so no backslash meets the quotes.
    return "\x1e".join(parts)


def unpack(path):
    """``{name: (is_package, source)}`` and the loader's text, read back out
    of a written bundle the way the loader reads them."""
    import ast
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    blob = None
    for node in ast.parse(text).body:
        if (isinstance(node, ast.Assign)
                and getattr(node.targets[0], "id", "") == "_P"):
            blob = node.value.value
    if blob is None:
        raise SystemExit("bundle has no _P")
    modules = {}
    for entry in blob.split("\x1e"):
        name, kind, source = entry.split("\x1f", 2)
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
    body = (SHEBANG + compacted("loader", LOADER) + "_P=r" + DELIMITER
            + payload(modules) + DELIMITER + "\n" + compacted("tail", TAIL))
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
    bundle, checks each has ``main``, that the paths land beside the file,
    and that ``--pilot NAME --help`` gets as far as each one's arguments.
    """
    probe = """import io, os, sys, types, contextlib
sys.modules['krpc'] = types.ModuleType('krpc')
sys.argv = [{path!r}]
import runpy
space = runpy.run_path({path!r}, run_name='bundled')
from common import launcher, paths
assert paths.ROOT == os.path.dirname({path!r}), paths.ROOT
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
        sys.modules[__name__], pystrip, pyminify, pycompact)]
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
