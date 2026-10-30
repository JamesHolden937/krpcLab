#!/usr/bin/env python3
"""Give each builtin a module uses often a one-letter alias, and each
function it reads off another module (``vec.norm``, ``math.radians``).

    ./tools/pyalias.py module.py            # the aliased source on stdout

``alias(source)`` puts ``G=getattr;F=float;...`` at the top of the module and
reads each of those builtins through its alias, where that saves bytes.  A
builtin is aliased only if nothing in the module binds its name (so every
read of it is the builtin), and never ``super`` (the zero-argument form
needs its own name) or one of the builtins that read the calling scope.
Reads inside f-strings, and inside annotations a module postpones, keep
the builtin's name.  Alias
names are capitals no part of the module uses.

``alias(source, name, hoist)`` also reads ``module.attr`` through an alias
(``N=vec.norm``, on the line after the import) for each attribute ``hoist``
(``{bound name: attributes}``) allows that bound name; ``tools/bundle.py``
says which can be read once at import (``hoistable``).  Only a name a lone
top-level ``import`` binds and nothing rebinds, and never in an f-string
or a postponed annotation.

A string or number literal spelled the same way often enough is read
through a capital too (``Q="nan"``), but never one in an f-string, a
postponed annotation or a ``match`` pattern.  ``getters`` names
attributes read through a getter: ``o.reference_frame`` -> ``R(o)``, with
``R=lambda o:o.reference_frame`` -- the same read of any object, at the
cost of a call (``tools/bundle.py`` passes only kRPC's remote
attributes, whose every read is a call to the game).

**Proven**: delete the alias lines from the result, put the builtins' names
and the attribute reads back, and the tree must be exactly the original's,
or ``alias`` raises.
"""
import ast
import builtins
import string
import sys

NEVER = {"super", "locals", "vars", "globals", "eval", "exec", "dir",
         "__import__", "print"}
BUILTINS = {n for n in dir(builtins) if not n.startswith("_")} - NEVER


class AliasError(ValueError):
    pass


def bind_counts(tree):
    """``{name: how many places bind it}``, over every scope."""
    out = {}

    def bind(k):
        out[k] = out.get(k, 0) + 1
    for n in ast.walk(tree):
        if isinstance(n, ast.Name) and not isinstance(n.ctx, ast.Load):
            bind(n.id)
        elif isinstance(n, ast.arg):
            bind(n.arg)
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef,
                            ast.ClassDef)):
            bind(n.name)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            for a in n.names:
                bind((a.asname or a.name).split(".")[0])
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            for k in n.names:
                bind(k)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            bind(n.name)
        elif isinstance(n, (ast.MatchAs, ast.MatchStar)) and n.name:
            bind(n.name)
    return out


def _bound(tree):
    """Every name the module binds anywhere, in any scope."""
    return set(bind_counts(tree))


def _homes(tree):
    """``{bound name: top-level statement index}`` for each name a
    top-level ``import`` binds that has its lines to itself, so an alias
    line can follow it."""
    out = {}
    body = tree.body
    for i, st in enumerate(body):
        if not isinstance(st, (ast.Import, ast.ImportFrom)) or (
                isinstance(st, ast.ImportFrom) and st.module == "__future__"):
            continue
        if (i and body[i - 1].end_lineno >= st.lineno) or (
                i + 1 < len(body) and body[i + 1].lineno <= st.end_lineno):
            continue
        for a in st.names:
            out[(a.asname or a.name).split(".")[0]] = i
    return out


def _offset(lines, starts, row, col_b):
    """Character offset of a parser (row, utf-8 column)."""
    text = lines[row - 1]
    return starts[row] + len(text.encode("utf-8")[:col_b].decode("utf-8"))


def _skipped(tree):
    """ids of Name nodes inside f-strings, and inside annotations when the
    module postpones them (``from __future__ import annotations``: then they
    are text some later reader resolves).  An evaluated annotation is an
    ordinary read, made after the alias line -- a dataclass field's type is
    the same object either way."""
    postponed = any(isinstance(n, ast.ImportFrom) and n.module == "__future__"
                    and any(a.name == "annotations" for a in n.names)
                    for n in tree.body)
    roots = []
    for n in ast.walk(tree):
        if isinstance(n, ast.JoinedStr):
            roots.append(n)
        elif not postponed:
            continue
        elif isinstance(n, ast.arg) and n.annotation is not None:
            roots.append(n.annotation)
        elif isinstance(n, ast.AnnAssign):
            roots.append(n.annotation)
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and n.returns is not None:
            roots.append(n.returns)
    return {id(m) for r in roots for m in ast.walk(r)}


def _insert_at(tree):
    """The statement index after any ``from __future__`` imports."""
    i = 0
    while (i < len(tree.body) and isinstance(tree.body[i], ast.ImportFrom)
           and tree.body[i].module == "__future__"):
        i += 1
    return i


def alias(source, name="<source>", hoist=None, getters=()):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise AliasError("%s does not parse: %s" % (name, exc)) from None
    counts = bind_counts(tree)
    bound = set(counts)
    skipped = _skipped(tree)
    lines = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines:
        starts.append(starts[-1] + len(line))
    homes = _homes(tree) if hoist else {}
    # Each candidate: its reads as ``(start, end)`` spans and the text each
    # span holds; builtins by name, attributes by ``(bound name, attr)``.
    # A constant in a ``match`` pattern must stay a literal.
    patterns = {id(m) for n in ast.walk(tree)
                if isinstance(n, ast.match_case) for m in ast.walk(n.pattern)}
    reads = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                and n.id in BUILTINS and n.id not in bound
                and id(n) not in skipped):
            a = _offset(lines, starts, n.lineno, n.col_offset)
            reads.setdefault(n.id, []).append((a, a + len(n.id)))
        elif (isinstance(n, ast.Constant) and id(n) not in skipped
                and id(n) not in patterns
                and type(n.value) in (str, int, float)):
            a = _offset(lines, starts, n.lineno, n.col_offset)
            b = _offset(lines, starts, n.end_lineno, n.end_col_offset)
            if len(source[a:b]) > 2:
                reads.setdefault(("@", source[a:b]), []).append((a, b))
        elif (hoist and isinstance(n, ast.Attribute)
                and isinstance(n.ctx, ast.Load)
                and isinstance(n.value, ast.Name)
                and n.attr in hoist.get(n.value.id, ())
                and counts.get(n.value.id) == 1 and n.value.id in homes
                and id(n) not in skipped):
            reads.setdefault((n.value.id, n.attr), []).append((
                _offset(lines, starts, n.lineno, n.col_offset),
                _offset(lines, starts, n.end_lineno, n.end_col_offset)))
        elif (getters and isinstance(n, ast.Attribute)
                and isinstance(n.ctx, ast.Load) and n.attr in getters
                and id(n) not in skipped
                and not (isinstance(n.value, ast.Call)
                         and getattr(n.value.func, "id", "") == "super")):
            # (start of the object, its end, the end of ``.attr``)
            reads.setdefault((".", n.attr), []).append((
                _offset(lines, starts, n.value.lineno, n.value.col_offset),
                _offset(lines, starts, n.value.end_lineno,
                        n.value.end_col_offset),
                _offset(lines, starts, n.end_lineno, n.end_col_offset)))

    def text(key):
        if isinstance(key, str):
            return key
        if key[0] == ".":
            return "lambda o:o." + key[1]
        return key[1] if key[0] == "@" else "%s.%s" % key

    def getter(key):
        return not isinstance(key, str) and key[0] == "."

    def headed(key):
        return isinstance(key, str) or key[0] in "@."
    # A read spelled across lines or with spaces is not the plain text.
    def plain(k, v):
        if getter(k):
            return all(source[r[1]:r[2]] == "." + k[1] for r in v)
        return all(source[a:b] == text(k) for a, b in v)
    reads = {k: v for k, v in reads.items() if plain(k, v)}
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | bound
    free = [c for c in string.ascii_uppercase if c not in used]
    free += [a + b for a in string.ascii_uppercase
             for b in string.ascii_uppercase if a + b not in used]
    plan = {}
    for old in sorted(reads, key=lambda k: (-len(reads[k]) * len(text(k)),
                                            str(k))):
        if not free:
            break
        new = free[0]
        cost = len(new) + 1 + len(text(old)) + 1
        each = (len(old[1]) + 1 - len(new) - 2 if getter(old)
                else len(text(old)) - len(new))
        if len(reads[old]) * each > cost:
            plan[old] = free.pop(0)
    if not plan:
        return source
    builtin = {old: new for old, new in plan.items() if headed(old)}
    spans, opens = [], {}
    for old, new in plan.items():
        for r in reads[old]:
            if getter(old):
                # ``o.attr`` -> ``X(o)``; nested reads open outermost first.
                opens.setdefault(r[0], []).append((-r[2], new + "("))
                spans.append((r[1], r[2], ")"))
            else:
                spans.append((r[0], r[1], new))
    for p, parts in opens.items():
        spans.append((p, p, "".join(t for _, t in sorted(parts))))
    at = _insert_at(tree)
    if builtin:
        head = ";".join("%s=%s" % (new, text(old))
                        for old, new in builtin.items()) + "\n"
        where = (starts[tree.body[at].lineno] if at < len(tree.body)
                 else len(source))
        spans.append((where, where, head))
    after = {}
    for old, new in plan.items():
        if not headed(old):
            after.setdefault(homes[old[0]], []).append(
                "%s=%s" % (new, text(old)))
    for i, parts in after.items():
        where = starts[tree.body[i].end_lineno + 1]
        line = ";".join(parts) + "\n"
        if where == len(source) and not source.endswith("\n"):
            line = "\n" + line
        spans.append((where, where, line))
    out = source
    for a, b, new in sorted(spans, reverse=True):
        out = out[:a] + new + out[b:]
    # The proof: drop the alias lines, put the reads back, compare.
    try:
        got = ast.parse(out)
    except SyntaxError as exc:
        raise AliasError("%s: aliased source does not parse (%s)"
                         % (name, exc)) from None
    back = {new: old for old, new in plan.items()}
    head_stmts = got.body[at:at + len(builtin)]
    def literal(key):
        return ast.parse(key[1], mode="eval").body

    def heads(s):
        if not (isinstance(s, ast.Assign) and len(s.targets) == 1):
            return False
        old = back.get(s.targets[0].id)
        if isinstance(old, str):
            return getattr(s.value, "id", None) == old
        if old is not None and old[0] == ".":
            return ast.dump(s.value) == ast.dump(
                ast.parse(text(old), mode="eval").body)
        return (old is not None and old[0] == "@"
                and ast.dump(s.value) == ast.dump(literal(old)))
    if not all(heads(s) for s in head_stmts):
        raise AliasError("%s: alias line not where it was put" % name)
    del got.body[at:at + len(builtin)]
    hoisted = [s for s in got.body if isinstance(s, ast.Assign)
               and len(s.targets) == 1 and isinstance(s.targets[0], ast.Name)
               and s.targets[0].id in back
               and not headed(back[s.targets[0].id])]
    if len(hoisted) != len(plan) - len(builtin) or not all(
            isinstance(s.value, ast.Attribute)
            and isinstance(s.value.value, ast.Name)
            and back[s.targets[0].id] == (s.value.value.id, s.value.attr)
            for s in hoisted):
        raise AliasError("%s: attribute aliases not where they were put"
                         % name)
    got.body = [s for s in got.body if all(s is not h for h in hoisted)]

    class Back(ast.NodeTransformer):
        def visit_Call(self, n):
            old = back.get(getattr(n.func, "id", None))
            if (old is not None and not isinstance(old, str)
                    and old[0] == "." and len(n.args) == 1
                    and not n.keywords):
                return ast.Attribute(value=self.visit(n.args[0]),
                                     attr=old[1], ctx=ast.Load())
            return self.generic_visit(n)

        def visit_Name(self, n):
            old = back.get(n.id)
            if old is None:
                return n
            if isinstance(old, str):
                n.id = old
                return n
            if old[0] == "@":
                return literal(old)
            if old[0] == ".":
                return n        # a getter stands only as a call's callee
            return ast.Attribute(value=ast.Name(id=old[0], ctx=ast.Load()),
                                 attr=old[1], ctx=ast.Load())
    got = Back().visit(got)
    if ast.dump(got) != ast.dump(ast.parse(source)):
        raise AliasError("%s: aliased source is a different program" % name)
    return out


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(alias(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
