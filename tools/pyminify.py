#!/usr/bin/env python3
"""Rename every function's local variables to a, b, ..., z, aa, ab, ...,
and every method's ``self`` to ``s``.

    ./tools/pyminify.py module.py           # the renamed source on stdout

``minify(source)`` renames, per function, the names the function binds
itself and nothing outside it can see.  Left alone:

- parameters (a caller may pass them by keyword), module and class names
  (other modules and ``--set`` reach them), attributes, keyword arguments;
- a name bound by ``import``, ``except ... as``, ``global``/``nonlocal``, or a
  ``match`` pattern (the function is skipped);
- a name any nested function, lambda or class reads (a closure);
- a name read inside an f-string (older parsers misplace those);
- every local of a function that calls ``locals``, ``vars``, ``eval``,
  ``exec`` or ``dir``.

A comprehension's own targets are its own scope and keep their names.  New
names never collide with any name the function or anything inside it uses.

**Checked against the parser.**  The result must parse to exactly the
original tree with the same renaming applied to its ``Name`` nodes, or
``minify`` raises.  That proves the text edit; that the renaming is
*semantically* safe rests on the rules above, and ``tools/bundle.py
--suite`` runs the offline tests against the renamed modules to check it.
"""
import ast
import builtins
import keyword
import sys

FUNCS = (ast.FunctionDef, ast.AsyncFunctionDef)
SCOPES = FUNCS + (ast.Lambda, ast.ClassDef)
COMPS = (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)
DYNAMIC = {"locals", "vars", "eval", "exec", "dir"}
# ``SELF`` is what a method's ``self`` becomes, so no local may take it.
SELF = "s"
RESERVED = (set(keyword.kwlist) | set(getattr(keyword, "softkwlist", ()))
            | set(dir(builtins)) | {"_", "self", "cls", SELF})


class MinifyError(ValueError):
    pass


def names(index):
    """a .. z, aa .. az, ba .. -- the index-th short name.  Lowercase only:
    the capitals are ``pyalias``'s, and a local that took one would cost a
    module-wide alias its single letter (measured: 52 letters made the
    bundle 2 kB larger)."""
    letters = "abcdefghijklmnopqrstuvwxyz"
    out = ""
    index += 1
    while index:
        index, rest = divmod(index - 1, 26)
        out = letters[rest] + out
    return out


def _own_nodes(func):
    """Every node of ``func``'s body that belongs to its scope -- not inside
    a nested function, lambda or class (whose decorators, defaults and
    bases do belong to it)."""
    stack = list(func.body)
    while stack:
        node = stack.pop()
        yield node
        if isinstance(node, SCOPES):
            outer = list(getattr(node, "decorator_list", []))
            if isinstance(node, ast.ClassDef):
                outer += node.bases + [k.value for k in node.keywords]
            else:
                a = node.args
                outer += a.defaults + [d for d in a.kw_defaults if d]
            stack.extend(outer)
            continue
        stack.extend(ast.iter_child_nodes(node))


def _plan(func):
    """``{old: new}`` for one function, or ``{}``."""
    own = list(_own_nodes(func))
    every = [n for n in ast.walk(func)]
    used = {n.id for n in every if isinstance(n, ast.Name)}
    used |= {n.name for n in every if isinstance(n, SCOPES[:2] + SCOPES[3:])}
    used |= {a.arg for n in every if isinstance(n, ast.arguments)
             for a in n.posonlyargs + n.args + n.kwonlyargs
             + [x for x in (n.vararg, n.kwarg) if x]}
    for n in own:
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) \
                and n.func.id in DYNAMIC:
            return {}
        if isinstance(n, ast.Match):
            return {}
    a = func.args
    params = {x.arg for x in a.posonlyargs + a.args + a.kwonlyargs
              + [v for v in (a.vararg, a.kwarg) if v]}
    comp_targets = set()
    for n in own:
        if isinstance(n, COMPS):
            for g in n.generators:
                comp_targets |= {t.id for t in ast.walk(g.target)
                                 if isinstance(t, ast.Name)}
    bound = set()
    excluded = set(params) | comp_targets
    for n in own:
        if isinstance(n, ast.Name) and isinstance(n.ctx, (ast.Store, ast.Del)):
            bound.add(n.id)
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            excluded |= set(n.names)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            excluded |= {(al.asname or al.name).split(".")[0]
                         for al in n.names}
        elif isinstance(n, ast.ExceptHandler) and n.name:
            excluded.add(n.name)
        elif isinstance(n, ast.JoinedStr):
            excluded |= {m.id for m in ast.walk(n) if isinstance(m, ast.Name)}
        elif isinstance(n, SCOPES):
            # Anything a nested scope mentions might be a closure over ours.
            inner = [n] if isinstance(n, ast.Lambda) else n.body
            for part in inner:
                excluded |= {m.id for m in ast.walk(part)
                             if isinstance(m, ast.Name)}
                excluded |= {m.arg for m in ast.walk(part)
                             if isinstance(m, ast.arg)}
            if not isinstance(n, ast.Lambda):
                excluded.add(n.name)
    rename = sorted(bound - excluded)
    if not rename:
        return {}
    # The most used names get the shortest replacements.
    count = {}
    for n in own:
        if isinstance(n, ast.Name) and n.id in rename:
            count[n.id] = count.get(n.id, 0) + 1
    rename.sort(key=lambda k: (-count.get(k, 0), k))
    taken = used | RESERVED
    plan, i = {}, 0
    for old in rename:
        while names(i) in taken:
            i += 1
        new = names(i)
        i += 1
        if len(new) < len(old):
            plan[old] = new
    return plan


def _apply(tree):
    """``[(node, new)]`` for every ``Name`` to rename, innermost function's
    plan first, comprehension targets shadowing it."""
    edits = []
    for func in ast.walk(tree):
        if not isinstance(func, FUNCS):
            continue
        plan = _plan(func)
        if not plan:
            continue

        def visit(node, shadow):
            if isinstance(node, SCOPES) and node is not func:
                outer = list(getattr(node, "decorator_list", []))
                if isinstance(node, ast.ClassDef):
                    outer += node.bases + [k.value for k in node.keywords]
                else:
                    outer += node.args.defaults + [
                        d for d in node.args.kw_defaults if d]
                for o in outer:
                    visit(o, shadow)
                return
            if isinstance(node, COMPS):
                inner = set(shadow)
                for g in node.generators:
                    inner |= {t.id for t in ast.walk(g.target)
                              if isinstance(t, ast.Name)}
                visit(node.generators[0].iter, shadow)
                for child in ast.iter_child_nodes(node):
                    if child is node.generators[0]:
                        for part in (child.target, *child.ifs):
                            visit(part, inner)
                    else:
                        visit(child, inner)
                return
            if (isinstance(node, ast.Name) and node.id in plan
                    and node.id not in shadow):
                edits.append((node, plan[node.id]))
            for child in ast.iter_child_nodes(node):
                visit(child, shadow)
        for stmt in func.body:
            visit(stmt, frozenset())
    return edits


def _self_edits(tree):
    """``[(node, new)]`` renaming each method's ``self`` to ``SELF``: the
    parameter (an ``ast.arg``) and every ``Name`` that reads it, nested
    functions and lambdas included.  Nobody passes ``self`` by keyword.  A
    method is left alone if anything in it already uses ``SELF``, if a
    nested scope binds ``self`` again, if ``self`` appears in an f-string,
    or if it is a ``staticmethod``."""
    edits = []
    for cls in ast.walk(tree):
        if not isinstance(cls, ast.ClassDef):
            continue
        for func in cls.body:
            if not isinstance(func, FUNCS):
                continue
            if any(getattr(d, "id", "") == "staticmethod"
                   for d in func.decorator_list):
                continue
            first = (func.args.posonlyargs + func.args.args)[:1]
            if not first or first[0].arg != "self":
                continue
            inner = [n for stmt in func.body for n in ast.walk(stmt)]
            inner += [n for d in func.args.defaults + func.args.kw_defaults
                      if d for n in ast.walk(d)]
            if any((isinstance(n, ast.Name) and n.id == SELF)
                   or (isinstance(n, ast.arg) and n.arg in (SELF, "self"))
                   or (isinstance(n, SCOPES) and n.name == SELF
                       if not isinstance(n, ast.Lambda) else False)
                   or (isinstance(n, ast.Name) and n.id == "self"
                       and not isinstance(n.ctx, ast.Load))
                   for n in inner):
                continue
            if any(isinstance(m, ast.Name) and m.id == "self"
                   for n in inner if isinstance(n, ast.JoinedStr)
                   for m in ast.walk(n)):
                continue
            # Defaults are evaluated outside the method: a ``self`` there is
            # the enclosing scope's, so not this one's to rename.
            outside = {id(n) for d in func.args.defaults + func.args.kw_defaults
                       if d for n in ast.walk(d)}
            edits.append((first[0], SELF))
            edits += [(n, SELF) for n in inner
                      if isinstance(n, ast.Name) and n.id == "self"
                      and id(n) not in outside]
    return edits


def _ident(node):
    return node.id if isinstance(node, ast.Name) else node.arg


def minify(source, name="<source>"):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise MinifyError("%s does not parse: %s" % (name, exc)) from None
    edits = _apply(tree) + _self_edits(tree)
    lines = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines:
        starts.append(starts[-1] + len(line))
    spans = []
    for node, new in edits:
        old = _ident(node)
        row = node.lineno
        text = lines[row - 1]
        col = len(text.encode("utf-8")[:node.col_offset].decode("utf-8"))
        a = starts[row] + col
        if source[a:a + len(old)] != old:
            raise MinifyError("%s: %s is not at line %d col %d"
                              % (name, old, row, col))
        spans.append((a, a + len(old), new))
    out = source
    for a, b, new in sorted(set(spans), reverse=True):
        out = out[:a] + new + out[b:]
    # The proof of the text edit: the original tree, renamed in the tree.
    want = ast.parse(source)
    by_pos = {(type(n), n.lineno, n.col_offset): new for n, new in edits}
    for node in ast.walk(want):
        if isinstance(node, (ast.Name, ast.arg)):
            new = by_pos.get((type(node), node.lineno, node.col_offset))
            if new is not None:
                setattr(node, "id" if isinstance(node, ast.Name) else "arg",
                        new)
    try:
        got = ast.parse(out)
    except SyntaxError as exc:
        raise MinifyError("%s: renamed source does not parse (%s)"
                          % (name, exc)) from None
    if ast.dump(got) != ast.dump(want):
        raise MinifyError("%s: renamed source is a different program" % name)
    return out


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(minify(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
