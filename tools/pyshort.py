#!/usr/bin/env python3
"""Rewrite statements into their shorter equivalents.

    ./tools/pyshort.py module.py            # the rewritten source on stdout

``shorten(source)`` makes, wherever it applies:

- ``return None`` -> ``return``;
- consecutive ``import a`` / ``import b`` -> ``import a,b``, and the same
  for ``from m import ...`` of one module;
- a top-level ``if __name__ == "__main__":`` is dropped (bundled, a module
  is never ``__main__``: the bundle's loader calls ``main`` itself);
- ``n = v`` followed by ``if`` whose test reads ``n`` first ->
  ``if (n:=v)...`` (only ``if``: a ``while`` test runs again each time
  round, and never in a class body);
- ``if c: return a`` followed by ``return b`` -> ``return a if c else b``;
- consecutive ``x = 0`` / ``y = 0`` (one constant, same type and spelling)
  -> ``x=y=0``, assigned in the same order.  A tuple (``x,y=0,1``) saves
  nothing once statements share a line, and evaluates every value first.

**Proven**: each rewrite has a canonical form (``return None``, one import
per statement, no ``__main__`` block, the assignment before the ``if``, the
``if`` before the ``return``), the same for the original's own walruses and
ternaries, and one assignment per target of a constant; the result and the original, both canonicalised, must parse to
the same tree, or ``shorten`` raises.
"""
import ast
import copy
import sys

# What cannot stand bare as a walrus' value or a ternary's operand.
LOOSE = (ast.Tuple, ast.Lambda, ast.IfExp, ast.NamedExpr, ast.Yield,
         ast.YieldFrom, ast.Starred, ast.Await)


class ShortError(ValueError):
    pass


def _leftmost(e):
    """The node of ``e`` evaluated first, down the left edge."""
    while True:
        if isinstance(e, ast.Compare):
            e = e.left
        elif isinstance(e, ast.BoolOp):
            e = e.values[0]
        elif isinstance(e, ast.BinOp):
            e = e.left
        elif isinstance(e, (ast.Attribute, ast.Subscript)):
            e = e.value
        elif isinstance(e, ast.Call):
            e = e.func
        else:
            return e


def _is_main_guard(st):
    t = st.test if isinstance(st, ast.If) else None
    return (isinstance(t, ast.Compare) and len(t.ops) == 1
            and isinstance(t.ops[0], ast.Eq)
            and isinstance(t.left, ast.Name) and t.left.id == "__name__"
            and isinstance(t.comparators[0], ast.Constant)
            and t.comparators[0].value == "__main__")


def _bodies(tree):
    """``(owner, statement list)`` for every block."""
    for node in ast.walk(tree):
        for field in ("body", "orelse", "finalbody"):
            body = getattr(node, field, None)
            if isinstance(body, list) and body and isinstance(body[0],
                                                              ast.stmt):
                yield node, body


class _Canon(ast.NodeTransformer):
    """The canonical form both sides are compared in."""

    def generic_visit(self, node):
        super().generic_visit(node)
        for field in ("body", "orelse", "finalbody"):
            body = getattr(node, field, None)
            if isinstance(body, list) and body and isinstance(body[0],
                                                              ast.stmt):
                setattr(node, field, self._block(node, body))
        return node

    def _block(self, owner, body):
        out, todo = [], list(body)
        while todo:
            st = todo.pop(0)
            if isinstance(owner, ast.Module) and _is_main_guard(st):
                continue
            if isinstance(st, (ast.Import, ast.ImportFrom)) and len(
                    st.names) > 1:
                for al in st.names:
                    one = copy.copy(st)
                    one.names = [al]
                    out.append(one)
                continue
            if (isinstance(st, ast.Assign) and len(st.targets) > 1
                    and isinstance(st.value, ast.Constant)):
                out += [ast.Assign(targets=[t], value=st.value)
                        for t in st.targets]
                continue
            if isinstance(st, ast.Return):
                if st.value is None:
                    st.value = ast.Constant(value=None)
                elif isinstance(st.value, ast.IfExp):
                    e = st.value
                    out.append(ast.If(test=e.test,
                                      body=[ast.Return(value=e.body)],
                                      orelse=[]))
                    todo.insert(0, ast.Return(value=e.orelse))
                    continue
            if isinstance(st, ast.If):
                parent, left = None, st.test
                while True:
                    nxt = _leftmost_step(left)
                    if nxt is None:
                        break
                    parent, left = left, nxt
                if (isinstance(left, ast.NamedExpr)
                        and isinstance(left.target, ast.Name)):
                    name = ast.Name(id=left.target.id, ctx=ast.Load())
                    if parent is None:
                        st.test = name
                    else:
                        _replace_child(parent, left, name)
                    out.append(ast.Assign(
                        targets=[ast.Name(id=left.target.id, ctx=ast.Store())],
                        value=left.value))
            out.append(st)
        return out


def _leftmost_step(e):
    if isinstance(e, ast.Compare):
        return e.left
    if isinstance(e, ast.BoolOp):
        return e.values[0]
    if isinstance(e, ast.BinOp):
        return e.left
    if isinstance(e, (ast.Attribute, ast.Subscript)):
        return e.value
    if isinstance(e, ast.Call):
        return e.func
    return None


def _replace_child(parent, old, new):
    for field, value in ast.iter_fields(parent):
        if value is old:
            setattr(parent, field, new)
            return
        if isinstance(value, list):
            for i, v in enumerate(value):
                if v is old:
                    value[i] = new
                    return


def canonical(tree):
    tree = _Canon().visit(tree)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            node.type_comment = None
    return ast.dump(tree)


def shorten(source, name="<source>"):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise ShortError("%s does not parse: %s" % (name, exc)) from None
    lines = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines:
        starts.append(starts[-1] + len(line))

    def at(row, col_b):
        text = lines[row - 1]
        return starts[row] + len(text.encode("utf-8")[:col_b].decode(
            "utf-8"))

    def begin(n):
        return at(n.lineno, n.col_offset)

    def end(n):
        return at(n.end_lineno, n.end_col_offset)

    def seg(n):
        return source[begin(n):end(n)]

    def wrap(n):
        # A value that spans lines did so inside brackets it no longer has.
        if isinstance(n, LOOSE) or "\n" in seg(n):
            return "(%s)" % seg(n)
        return seg(n)
    groups = []         # [(start, end, text)], applied whole or not at all

    def alone(body, i):
        """Statement ``body[i]`` has its lines to itself."""
        st = body[i]
        return ((i == 0 or body[i - 1].end_lineno < st.lineno)
                and (i + 1 == len(body) or body[i + 1].lineno > st.end_lineno))
    for owner, body in _bodies(tree):
        i = 0
        while i < len(body):
            st = body[i]
            nxt = body[i + 1] if i + 1 < len(body) else None
            if (isinstance(owner, ast.Module) and _is_main_guard(st)
                    and not st.orelse and alone(body, i)):
                a = starts[st.lineno]
                b = starts[st.end_lineno + 1] if st.end_lineno + 1 < len(
                    starts) else len(source)
                groups.append([(a, b, "")])
                i += 1
                continue
            if isinstance(st, ast.Return) and isinstance(
                    st.value, ast.Constant) and st.value.value is None \
                    and seg(st.value) == "None":
                groups.append([(begin(st) + len("return"), end(st), "")])
            group = [st]
            if isinstance(st, (ast.Import, ast.ImportFrom)) and not (
                    isinstance(st, ast.ImportFrom)
                    and st.module == "__future__"):
                j = i + 1
                while (j < len(body) and type(body[j]) is type(st)
                       and alone(body, j) and alone(body, j - 1)
                       and (isinstance(st, ast.Import)
                            or (body[j].module == st.module
                                and body[j].level == st.level))):
                    group.append(body[j])
                    j += 1
            if len(group) > 1:
                names = ",".join(
                    al.name + (" as " + al.asname if al.asname else "")
                    for g in group for al in g.names)
                if isinstance(st, ast.Import):
                    text = "import " + names
                else:
                    text = "from %s%s import %s" % ("." * st.level,
                                                    st.module or "", names)
                groups.append([(begin(st), end(group[-1]), text)])
                i += len(group)
                continue
            run = [st]
            while (isinstance(st, ast.Assign) and len(st.targets) == 1
                   and isinstance(st.value, ast.Constant)
                   and not isinstance(st.targets[0], (ast.Tuple, ast.List,
                                                      ast.Starred))
                   and i + len(run) < len(body)):
                o = body[i + len(run)]
                if not (isinstance(o, ast.Assign) and len(o.targets) == 1
                        and isinstance(o.value, ast.Constant)
                        and type(o.value.value) is type(st.value.value)
                        and seg(o.value) == seg(st.value)
                        and not isinstance(o.targets[0], (
                            ast.Tuple, ast.List, ast.Starred))
                        and o.type_comment is None
                        and run[-1].end_lineno < o.lineno):
                    break
                run.append(o)
            if len(run) > 1:
                text = "=".join(seg(r.targets[0]) for r in run) + "=" + wrap(
                    st.value)
                groups.append([(begin(st), end(run[-1]), text)])
                i += len(run)
                continue
            if (isinstance(st, ast.Assign) and len(st.targets) == 1
                    and isinstance(st.targets[0], ast.Name)
                    and not isinstance(owner, ast.ClassDef)
                    and isinstance(nxt, ast.If)
                    and not isinstance(st.value, LOOSE)
                    and st.end_lineno < nxt.lineno
                    and st.type_comment is None):
                left = _leftmost(nxt.test)
                if (isinstance(left, ast.Name) and left.id == st.targets[0].id
                        and isinstance(left.ctx, ast.Load)):
                    walrus = "%s:=%s" % (left.id, wrap(st.value))
                    if left is not nxt.test:
                        walrus = "(%s)" % walrus
                    groups.append([(begin(st), begin(nxt), ""),
                                   (begin(left), end(left), walrus)])
                    i += 2
                    continue
            if (isinstance(st, ast.If) and not st.orelse and len(st.body) == 1
                    and isinstance(st.body[0], ast.Return)
                    and st.body[0].value is not None
                    and isinstance(nxt, ast.Return) and nxt.value is not None
                    and st.end_lineno < nxt.lineno
                    and not isinstance(st.test, ast.NamedExpr)):
                text = "return %s if %s else %s" % (
                    wrap(st.body[0].value), wrap(st.test), wrap(nxt.value))
                groups.append([(begin(st), end(nxt), text)])
                i += 2
                continue
            i += 1
    # Outer blocks come first (``ast.walk``), so a rewrite that spans a
    # statement wins over one inside it.
    kept = []
    for group in groups:
        if not any(a < d and c < b or a == b == c for a, b, _ in group
                   for c, d, _ in kept):
            kept += group
    if not kept:
        return source
    out = source
    for a, b, text in sorted(kept, reverse=True):
        out = out[:a] + text + out[b:]
    try:
        got = ast.parse(out)
    except SyntaxError as exc:
        raise ShortError("%s: rewritten source does not parse (%s)"
                         % (name, exc)) from None
    if canonical(got) != canonical(ast.parse(source)):
        raise ShortError("%s: rewritten source is a different program" % name)
    return out


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(shorten(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
