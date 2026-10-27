#!/usr/bin/env python3
"""Give each builtin a module uses often a one-letter alias.

    ./tools/pyalias.py module.py            # the aliased source on stdout

``alias(source)`` puts ``G=getattr;F=float;...`` at the top of the module and
reads each of those builtins through its alias, where that saves bytes.  A
builtin is aliased only if nothing in the module binds its name (so every
read of it is the builtin), and never ``super`` (the zero-argument form
needs its own name) or one of the builtins that read the calling scope.
Reads inside f-strings and annotations keep the builtin's name.  Alias
names are capitals no part of the module uses.

**Proven**: delete the alias line from the result, put the builtins' names
back, and the tree must be exactly the original's, or ``alias`` raises.
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


def _bound(tree):
    """Every name the module binds anywhere, in any scope."""
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Name) and not isinstance(n.ctx, ast.Load):
            out.add(n.id)
        elif isinstance(n, ast.arg):
            out.add(n.arg)
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef,
                            ast.ClassDef)):
            out.add(n.name)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            out |= {(a.asname or a.name).split(".")[0] for a in n.names}
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            out |= set(n.names)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            out.add(n.name)
    return out


def _skipped(tree):
    """ids of Name nodes inside f-strings and annotations."""
    roots = []
    for n in ast.walk(tree):
        if isinstance(n, ast.JoinedStr):
            roots.append(n)
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


def alias(source, name="<source>"):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise AliasError("%s does not parse: %s" % (name, exc)) from None
    bound = _bound(tree)
    skipped = _skipped(tree)
    reads = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                and n.id in BUILTINS and n.id not in bound
                and id(n) not in skipped):
            reads.setdefault(n.id, []).append(n)
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | bound
    free = [c for c in string.ascii_uppercase if c not in used]
    free += [a + b for a in string.ascii_uppercase
             for b in string.ascii_uppercase if a + b not in used]
    plan = {}
    for old in sorted(reads, key=lambda k: -len(reads[k]) * len(k)):
        if not free:
            break
        new = free[0]
        cost = len(new) + 1 + len(old) + 1
        if len(reads[old]) * (len(old) - len(new)) > cost:
            plan[old] = free.pop(0)
    if not plan:
        return source
    lines = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines:
        starts.append(starts[-1] + len(line))
    spans = []
    for old, new in plan.items():
        for n in reads[old]:
            text = lines[n.lineno - 1]
            col = len(text.encode("utf-8")[:n.col_offset].decode("utf-8"))
            a = starts[n.lineno] + col
            if source[a:a + len(old)] != old:
                raise AliasError("%s: %s is not at line %d" % (name, old,
                                                              n.lineno))
            spans.append((a, a + len(old), new))
    out = source
    for a, b, new in sorted(spans, reverse=True):
        out = out[:a] + new + out[b:]
    head = ";".join("%s=%s" % (new, old) for old, new in plan.items()) + "\n"
    at = _insert_at(tree)
    if at < len(tree.body):
        where = starts[tree.body[at].lineno]
    else:
        where = len(out)
    out = out[:where] + head + out[where:]
    # The proof: drop the alias line, rename back, compare.
    try:
        got = ast.parse(out)
    except SyntaxError as exc:
        raise AliasError("%s: aliased source does not parse (%s)"
                         % (name, exc)) from None
    back = {new: old for old, new in plan.items()}
    head_stmts = got.body[at:at + len(plan)]
    if not all(isinstance(s, ast.Assign) and len(s.targets) == 1
               and back.get(s.targets[0].id) == s.value.id
               for s in head_stmts):
        raise AliasError("%s: alias line not where it was put" % name)
    del got.body[at:at + len(plan)]
    for n in ast.walk(got):
        if isinstance(n, ast.Name) and n.id in back:
            n.id = back[n.id]
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
