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


def alias(source, name="<source>", hoist=None):
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
    reads = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                and n.id in BUILTINS and n.id not in bound
                and id(n) not in skipped):
            a = _offset(lines, starts, n.lineno, n.col_offset)
            reads.setdefault(n.id, []).append((a, a + len(n.id)))
        elif (hoist and isinstance(n, ast.Attribute)
                and isinstance(n.ctx, ast.Load)
                and isinstance(n.value, ast.Name)
                and n.attr in hoist.get(n.value.id, ())
                and counts.get(n.value.id) == 1 and n.value.id in homes
                and id(n) not in skipped):
            reads.setdefault((n.value.id, n.attr), []).append((
                _offset(lines, starts, n.lineno, n.col_offset),
                _offset(lines, starts, n.end_lineno, n.end_col_offset)))

    def text(key):
        return key if isinstance(key, str) else "%s.%s" % key
    # A read spelled across lines or with spaces is not the plain text.
    reads = {k: v for k, v in reads.items()
             if all(source[a:b] == text(k) for a, b in v)}
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
        if len(reads[old]) * (len(text(old)) - len(new)) > cost:
            plan[old] = free.pop(0)
    if not plan:
        return source
    builtin = {old: new for old, new in plan.items() if isinstance(old, str)}
    spans = [(a, b, new) for old, new in plan.items() for a, b in reads[old]]
    at = _insert_at(tree)
    if builtin:
        head = ";".join("%s=%s" % (new, old)
                        for old, new in builtin.items()) + "\n"
        where = (starts[tree.body[at].lineno] if at < len(tree.body)
                 else len(source))
        spans.append((where, where, head))
    after = {}
    for old, new in plan.items():
        if not isinstance(old, str):
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
    if not all(isinstance(s, ast.Assign) and len(s.targets) == 1
               and back.get(s.targets[0].id) == s.value.id
               for s in head_stmts):
        raise AliasError("%s: alias line not where it was put" % name)
    del got.body[at:at + len(builtin)]
    hoisted = [s for s in got.body if isinstance(s, ast.Assign)
               and len(s.targets) == 1 and isinstance(s.targets[0], ast.Name)
               and not isinstance(back.get(s.targets[0].id, ""), str)]
    if len(hoisted) != len(plan) - len(builtin) or not all(
            isinstance(s.value, ast.Attribute)
            and isinstance(s.value.value, ast.Name)
            and back[s.targets[0].id] == (s.value.value.id, s.value.attr)
            for s in hoisted):
        raise AliasError("%s: attribute aliases not where they were put"
                         % name)
    got.body = [s for s in got.body if all(s is not h for h in hoisted)]

    class Back(ast.NodeTransformer):
        def visit_Name(self, n):
            old = back.get(n.id)
            if old is None:
                return n
            if isinstance(old, str):
                n.id = old
                return n
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
