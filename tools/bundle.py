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

The sources travel as one dict of string literals (``repr``), so the file
carries no comment at all -- nobody reads it; the tree is the readable copy.

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
    """``collect``'s modules with comments and docstrings removed: what the
    bundle carries.  ``pystrip`` raises rather than change a program."""
    try:
        return {name: (is_package, pystrip.strip(source, name))
                for name, (is_package, source) in modules.items()}
    except pystrip.StripError as exc:
        raise SystemExit("cannot strip: %s" % exc)


# The bundle's own code, carried uncommented like the modules it serves
# (and not %-formatted).  ``_Bundled`` is a ``sys.meta_path`` finder over
# ``_SOURCES``, ``{name: (is_package, source)}``:
# packages get a ``__path__`` so relative imports resolve, and each module a
# ``__file__`` beside the bundle.  ``_main`` (in ``EPILOGUE``) is ``run.sh``:
# ``--pilot NAME`` or ``-m MODULE`` runs that module's ``main``, nothing runs
# the launcher, and ``KRPCLAB_BUNDLE`` tells the launcher's buttons to start
# each autopilot from this same file.
PREAMBLE = '''#!/usr/bin/env python3
import importlib.abc
import importlib.machinery
import os
import sys

_HERE = os.path.dirname(os.path.abspath(
    globals().get("__file__") or sys.argv[0]))


class _Bundled(importlib.abc.MetaPathFinder, importlib.abc.Loader):

    def __init__(self, modules):
        self.modules = modules

    def find_spec(self, name, path=None, target=None):
        if name not in self.modules:
            return None
        is_package, _ = self.modules[name]
        spec = importlib.machinery.ModuleSpec(name, self,
                                              is_package=is_package)
        if is_package:
            spec.submodule_search_locations = []
        return spec

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        is_package, source = self.modules[module.__name__]
        parts = module.__name__.split(".")
        module.__file__ = os.path.join(
            _HERE, *(parts + ["__init__.py"] if is_package
                     else parts[:-1] + [parts[-1] + ".py"]))
        exec(compile(source, "<bundled %s>" % module.__name__, "exec"),
             module.__dict__)


def _install():
    finder = _Bundled(_SOURCES)
    sys.meta_path.insert(0, finder)
    return finder


'''

EPILOGUE = '''

_install()


def _main(argv):
    import importlib
    from common import launcher
    os.environ["KRPCLAB_BUNDLE"] = os.path.join(_HERE, os.path.basename(
        globals().get("__file__") or sys.argv[0]))
    module, rest = "common.launcher", list(argv)
    for flag in ("--pilot", "-m"):
        for i, arg in enumerate(rest):
            value = None
            if arg == flag and i + 1 < len(rest):
                value, drop = rest[i + 1], 2
            elif arg.startswith(flag + "="):
                value, drop = arg.split("=", 1)[1], 1
            if value is None:
                continue
            module = (launcher.module_for(value) if flag == "--pilot"
                      else value)
            if module is None:
                sys.stderr.write("unknown pilot: %s\\n" % value)
                return 2
            del rest[i:i + drop]
            break
    sys.argv = [sys.argv[0]] + rest
    return importlib.import_module(module).main(rest)


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
'''


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


def build():
    modules = stripped(collect(PACKAGES))
    for name, module in pilots():
        if module not in modules:
            raise SystemExit("pilot %s needs %s, which no bundled package "
                             "has: add its package to PACKAGES"
                             % (name, module))
    lines = ["_SOURCES = {"]
    for name in sorted(modules):
        is_package, source = modules[name]
        lines.append("    %r: (%r, %r)," % (name, is_package, source))
    lines.append("}")
    body = PREAMBLE + "\n".join(lines) + "\n" + EPILOGUE
    return body, NAME, len(modules)


def verify(path, modules):
    """Read the written file back and check every module came out intact,
    and that the file carries no comment but its ``#!`` line.

    ``modules`` is what was meant to go in (``stripped(collect(...))``).
    Each source is compared byte for byte: a claim nothing checks is the
    kind this project keeps paying for.
    """
    import ast
    import io
    import tokenize
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    seen = None
    for node in ast.parse(text).body:
        if (isinstance(node, ast.Assign)
                and getattr(node.targets[0], "id", "") == "_SOURCES"):
            seen = ast.literal_eval(node.value)
    if seen is None:
        raise SystemExit("bundle has no _SOURCES")
    for module, (_, source) in modules.items():
        got = seen.get(module)
        if got is None:
            raise SystemExit("%s is missing from the bundle" % module)
        if got[1] != source:
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
source = open({path!r}, encoding='utf-8').read()
space = {{'__name__': 'bundled', '__file__': {path!r}}}
exec(compile(source, {path!r}, 'exec'), space)
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
    files = [os.path.abspath(__file__), os.path.abspath(pystrip.__file__)]
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-o", "--out", default=None,
                        help="output file (default: dist/%s)" % NAME)
    parser.add_argument("--no-selftest", action="store_true",
                        help="skip running the result to check it works")
    parser.add_argument("--if-stale", action="store_true",
                        help="only if a source is newer than the bundle")
    parser.add_argument("-q", "--quiet", action="store_true")
    args = parser.parse_args(argv)

    out = os.path.abspath(args.out or DEFAULT)
    if args.if_stale and not stale(out):
        if not args.quiet:
            print("%s  up to date" % out)
        return 0
    out, count, note = write(out, test=not args.no_selftest)
    if not args.quiet:
        print("%s  %d modules  %.0f kB%s" % (
            out, count, os.path.getsize(out) / 1024.0,
            "  " + note if note else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
