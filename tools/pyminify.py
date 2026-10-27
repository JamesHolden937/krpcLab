#!/usr/bin/env python3
"""Rename every function's local variables and nested functions to a, b,
..., z, aa, ab, ..., every method's ``self`` to ``s``, and every
module-level function nothing outside its module names.

Parameters stay (a caller may pass them by keyword) unless ``free`` lists
them: ``free_parameters`` over every module shipped together.

    ./tools/pyminify.py module.py           # the renamed source on stdout

``minify(source)`` renames, per function, the names the function binds
itself and nothing outside it can see.  Left alone:

- parameters not in ``free``, module and class names
  (other modules and ``--set`` reach them), attributes, keyword arguments;
- a name bound by ``import``, ``except ... as``, ``global``/``nonlocal``, or a
  ``match`` pattern (the function is skipped);
- a name any nested function, lambda or class reads (a closure);
- a name read inside an f-string (older parsers misplace those);
- every local of a function that calls ``locals``, ``vars``, ``eval``,
  ``exec`` or ``dir``.

A comprehension's own targets are its own scope and keep their names.  New
names never collide with any name the function or anything inside it uses.
A nested ``def`` is a local like any other (and, like one, keeps its name
if a nested scope -- itself included -- reads it).

``minify(source, name, outside)`` also renames a module-level function,
class, variable or import (``import math as a``) when ``outside`` (every identifier any *other* file
mentions) lacks its name and the module binds that name nowhere else, never
reads it in a string or an f-string, and never calls ``globals``/``vars``/
``eval``/``exec``.  ``minify(..., attributes=attribute_plan(...))`` renames
attributes, planned once over every module shipped together (see
``attribute_plan``).  Without them (the command line) only locals move.
The passes run attributes, module names, locals, each proven on its own;
each later pass sees the earlier names and avoids them.

**Checked against the parser.**  The result must parse to exactly the
original tree with the same renaming applied to its ``Name`` nodes, or
``minify`` raises.  That proves the text edit; that the renaming is
*semantically* safe rests on the rules above, and ``tools/bundle.py
--suite`` runs the offline tests against the renamed modules to check it.
"""
import ast
import builtins
import keyword
import re
import sys

FUNCS = (ast.FunctionDef, ast.AsyncFunctionDef)
SCOPES = FUNCS + (ast.Lambda, ast.ClassDef)
COMPS = (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)
DYNAMIC = {"locals", "vars", "eval", "exec", "dir"}
WORD = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
DEF = re.compile(r"(?:async\s+)?(?:def|class)\s+")
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


def _plan(func, free=frozenset()):
    """``{old: new}`` for one function, or ``{}``.  ``free`` holds the
    ``(function, parameter)`` pairs no caller can pass by keyword; those
    move like locals."""
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
    movable = set()
    if not (func.name.startswith("__") and func.name.endswith("__")):
        movable = {p for p in params if (func.name, p) in free} - {
            "self", "cls"}
        movable |= {v.arg for v in (a.vararg, a.kwarg) if v}
    bound = set(movable)
    excluded = (set(params) - movable) | comp_targets
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
            if isinstance(n, FUNCS):
                bound.add(n.name)
            elif isinstance(n, ast.ClassDef):
                excluded.add(n.name)
    rename = sorted(bound - excluded)
    if not rename:
        return {}
    # The most used names get the shortest replacements.
    count = {}
    for n in own:
        k = n.id if isinstance(n, ast.Name) else getattr(n, "name", None)
        if k in rename:
            count[k] = count.get(k, 0) + 1
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


def _apply(tree, free=frozenset()):
    """``[(node, new)]`` for every ``Name`` to rename, innermost function's
    plan first, comprehension targets shadowing it."""
    edits = []
    for func in ast.walk(tree):
        if not isinstance(func, FUNCS):
            continue
        plan = _plan(func, free)
        if not plan:
            continue
        a = func.args
        edits += [(x, plan[x.arg]) for x in a.posonlyargs + a.args
                  + a.kwonlyargs + [v for v in (a.vararg, a.kwarg) if v]
                  if x.arg in plan]

        def visit(node, shadow):
            if isinstance(node, SCOPES) and node is not func:
                if (isinstance(node, FUNCS) and node.name in plan
                        and node.name not in shadow):
                    edits.append((node, plan[node.name]))
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


def _module_edits(tree, outside, imports_outside=None):
    """``[(node, new)]`` renaming what only this module can name: its
    functions, classes and variables bound once at the top level.  See the
    module docstring for what is refused."""
    if outside is None:
        return []
    every = list(ast.walk(tree))
    called = [n for n in every if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Name)]
    if any(n.func.id in DYNAMIC | {"globals"} for n in called):
        return []
    top = {}            # name -> its one top-level binding node
    binds = {}          # name -> how many bindings, any scope
    class_level = {}    # name -> class-body bindings (def or Name)
    reads, idents, refused = {}, set(), set()

    def bind(k):
        binds[k] = binds.get(k, 0) + 1
        idents.add(k)

    top_aliases = set()
    for stmt in tree.body:
        if isinstance(stmt, (ast.Import, ast.ImportFrom)) and not (
                isinstance(stmt, ast.ImportFrom)
                and stmt.module == "__future__"):
            for al in stmt.names:
                k = al.asname or al.name
                if "." not in k and k != "*":
                    top[k] = al
                    top_aliases.add(id(al))
        if isinstance(stmt, FUNCS + (ast.ClassDef,)):
            top[stmt.name] = stmt
            if isinstance(stmt, ast.ClassDef) and any(
                    _ident_of(x).endswith(("Error", "Exception", "Warning"))
                    for x in stmt.bases):
                refused.add(stmt.name)  # a log prints its __name__
        elif isinstance(stmt, (ast.Assign, ast.AnnAssign)):
            targets = stmt.targets if isinstance(stmt, ast.Assign) \
                else [stmt.target]
            for t in targets:
                for m in [t] + (list(t.elts) if isinstance(t, ast.Tuple)
                                else []):
                    if isinstance(m, ast.Name):
                        top[m.id] = m
    for cls in every:
        if isinstance(cls, ast.ClassDef):
            for stmt in cls.body:
                if isinstance(stmt, FUNCS + (ast.ClassDef,)):
                    class_level.setdefault(stmt.name, []).append(stmt)
                elif isinstance(stmt, ast.Assign):
                    for t in stmt.targets:
                        if isinstance(t, ast.Name):
                            class_level.setdefault(t.id, []).append(t)
    in_class = {id(x) for v in class_level.values() for x in v}
    for n in every:
        if isinstance(n, ast.Name):
            idents.add(n.id)
            if isinstance(n.ctx, ast.Load):
                reads.setdefault(n.id, []).append(n)
            elif id(n) not in in_class:
                bind(n.id)
        elif isinstance(n, ast.arg):
            bind(n.arg)
            refused.add(n.arg)
        elif isinstance(n, FUNCS + (ast.ClassDef,)):
            if id(n) not in in_class:
                bind(n.name)
            idents.add(n.name)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            for al in n.names:
                idents |= set(al.name.split("."))
                k = (al.asname or al.name).split(".")[0]
                if id(al) in top_aliases:
                    bind(k)
                else:
                    refused.add(k)
                if al.asname:
                    idents.add(al.asname)
        elif isinstance(n, (ast.Global, ast.Nonlocal)):
            refused |= set(n.names)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            refused.add(n.name)
        elif isinstance(n, ast.Attribute):
            idents.add(n.attr)
        elif isinstance(n, ast.keyword) and n.arg:
            idents.add(n.arg)
            refused.add(n.arg)
        elif isinstance(n, ast.Constant) and isinstance(n.value, str):
            words = set(WORD.findall(n.value))
            idents |= words
            refused |= words
        elif isinstance(n, ast.JoinedStr):
            refused |= {m.id for m in ast.walk(n) if isinstance(m, ast.Name)}
        elif isinstance(n, (ast.MatchAs, ast.MatchStar)) and n.name:
            refused.add(n.name)

    def dunder(k):
        return k.startswith("__")

    # An import is reached from outside only as ``module.name`` or
    # ``from module import name``: ``imports_outside`` says which can be.
    plain = [k for k, node in top.items()
             if binds.get(k) == 1 and k not in refused
             and k not in (imports_outside if id(node) in top_aliases
                           and imports_outside is not None else outside)
             and k not in class_level and not dunder(k)]
    count = {k: len(reads.get(k, ())) + 1 for k in plain}
    rename = sorted(count, key=lambda k: (-count[k], k))
    taken = idents | RESERVED
    edits, i = [], 0
    for old in rename:
        while names(i) in taken:
            i += 1
        new = names(i)
        i += 1
        if len(new) < len(old):
            edits += [(top[old], new)] + [(n, new) for n in reads.get(old, ())]
    return edits


def _class_body(cls):
    """Nodes directly in ``cls``'s own scope (not inside its methods)."""
    stack = list(cls.body)
    while stack:
        node = stack.pop()
        yield node
        if isinstance(node, SCOPES):
            outer = list(getattr(node, "decorator_list", []))
            if isinstance(node, ast.ClassDef):
                outer += node.bases + [k.value for k in node.keywords]
            else:
                outer += node.args.defaults + [
                    d for d in node.args.kw_defaults if d]
            stack.extend(outer)
            continue
        stack.extend(ast.iter_child_nodes(node))


def _class_bindings(cls):
    """``cls``'s methods and nested classes, and the ``Name`` targets of
    its plain assignments: what an attribute rename renames in its body."""
    out = []
    for stmt in cls.body:
        if isinstance(stmt, FUNCS + (ast.ClassDef,)):
            out.append(stmt)
        elif isinstance(stmt, ast.Assign):
            out += [t for t in stmt.targets if isinstance(t, ast.Name)]
    return out


def attribute_plan(sources, outside, foreign, private_only=False):
    """``{old: new}`` for attributes every module in ``sources`` (``name ->
    source``) renames together.  An attribute is renamed only if a class
    here defines it -- a method, a class-body assignment, or a store on
    ``self`` -- and it is not:

    - named by ``outside`` (any other file) or ``foreign`` (the attribute
      names of kRPC's services, its client and the standard library), so
      no object we did not write is reached under a new name;
    - a dataclass/NamedTuple field (annotated in a class body), a keyword
      argument anywhere (``SimpleNamespace(ut=...)``), a word in any string
      (``getattr(x, "name")``), read inside an f-string, a top-level name
      of any module (``module.name``), or read as a plain name in a class
      body (``@name.setter``);
    - a dunder or ``_sunder_``; or, with ``private_only``, public.

    New names are no identifier any source, ``outside`` or ``foreign``
    uses."""
    trees = {k: ast.parse(v) for k, v in sources.items()}
    defined, refused, count, idents = set(), set(), {}, set()
    for name, tree in trees.items():
        refused |= set(name.split("."))
        idents |= set(WORD.findall(sources[name]))
        for stmt in tree.body:
            if isinstance(stmt, FUNCS + (ast.ClassDef,)):
                refused.add(stmt.name)
            elif isinstance(stmt, (ast.Import, ast.ImportFrom)):
                refused |= {(al.asname or al.name).split(".")[0]
                            for al in stmt.names}
            for n in ast.walk(stmt) if not isinstance(
                    stmt, FUNCS + (ast.ClassDef,)) else ():
                if isinstance(n, ast.Name) and not isinstance(n.ctx, ast.Load):
                    refused.add(n.id)
        for n in ast.walk(tree):
            if isinstance(n, ast.Attribute):
                count[n.attr] = count.get(n.attr, 0) + 1
                if isinstance(n.value, ast.Name) and n.value.id == "self" \
                        and not isinstance(n.ctx, ast.Load):
                    defined.add(n.attr)
            elif isinstance(n, ast.keyword) and n.arg:
                refused.add(n.arg)
            elif isinstance(n, ast.Constant) and isinstance(n.value, str):
                refused |= set(WORD.findall(n.value))
            elif isinstance(n, ast.JoinedStr):
                refused |= {m.attr for m in ast.walk(n)
                            if isinstance(m, ast.Attribute)}
            elif isinstance(n, ast.ClassDef):
                bound = _class_bindings(n)
                for b in bound:
                    k = _ident(b)
                    defined.add(k)
                    count[k] = count.get(k, 0) + 1
                ids = {id(b) for b in bound}
                for m in _class_body(n):
                    if isinstance(m, ast.Name) and id(m) not in ids:
                        refused.add(m.id)
                    elif isinstance(m, ast.AnnAssign) and isinstance(
                            m.target, ast.Name):
                        refused.add(m.target.id)
    rename = sorted(
        k for k in defined
        if k not in refused and k not in outside and k not in foreign
        and not k.startswith("__") and not (k.startswith("_")
                                            and k.endswith("_"))
        and (k.startswith("_") or not private_only))
    rename.sort(key=lambda k: (-count.get(k, 0), k))
    taken = idents | set(outside) | set(foreign) | RESERVED
    plan, i = {}, 0
    for old in rename:
        while names(i) in taken:
            i += 1
        new = names(i)
        i += 1
        if len(new) < len(old):
            plan[old] = new
    return plan


def _attribute_edits(tree, plan):
    """``[(node, new)]`` for ``attribute_plan``'s renames in one module."""
    if not plan:
        return []
    edits = [(n, plan[n.attr]) for n in ast.walk(tree)
             if isinstance(n, ast.Attribute) and n.attr in plan]
    for cls in ast.walk(tree):
        if isinstance(cls, ast.ClassDef):
            edits += [(b, plan[_ident(b)]) for b in _class_bindings(cls)
                      if _ident(b) in plan]
    return edits


def _ident_of(node):
    """The last name in a dotted expression, or ''."""
    if isinstance(node, ast.Name):
        return node.id
    return node.attr if isinstance(node, ast.Attribute) else ""


def _ident(node):
    if isinstance(node, ast.alias):
        return node.asname or node.name
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return node.arg if isinstance(node, ast.arg) else node.name


def _key(node):
    return (type(node), node.lineno, node.col_offset,
            node.end_lineno, node.end_col_offset)


def _start(node, lines, starts, source):
    """Offset of ``node``'s identifier in the source."""
    if isinstance(node, ast.Attribute) or (isinstance(node, ast.alias)
                                           and node.asname):
        row, col_b = node.end_lineno, node.end_col_offset
        text = lines[row - 1]
        col = len(text.encode("utf-8")[:col_b].decode("utf-8"))
        return starts[row] + col - len(_ident(node))
    row = node.lineno
    text = lines[row - 1]
    col = len(text.encode("utf-8")[:node.col_offset].decode("utf-8"))
    a = starts[row] + col
    if isinstance(node, FUNCS + (ast.ClassDef,)):
        m = DEF.match(text, col)
        a = starts[row] + m.end() if m else a
    return a


def _rewrite(source, edits, name):
    """``source`` with ``edits`` applied, proven against the parser."""
    if not edits:
        return source
    lines = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines:
        starts.append(starts[-1] + len(line))
    spans = []
    for node, new in edits:
        old = _ident(node)
        a = _start(node, lines, starts, source)
        if source[a:a + len(old)] != old:
            raise MinifyError("%s: %s is not at line %d"
                              % (name, old, node.lineno))
        if isinstance(node, ast.alias) and not node.asname:
            new = "%s as %s" % (old, new)   # import x -> import x as a
        spans.append((a, a + len(old), new))
    out = source
    for a, b, new in sorted(set(spans), reverse=True):
        out = out[:a] + new + out[b:]
    # The proof of the text edit: the original tree, renamed in the tree.
    want = ast.parse(source)
    by_pos = {_key(n): new for n, new in edits}
    field = {ast.Name: "id", ast.arg: "arg", ast.Attribute: "attr",
             ast.alias: "asname"}
    for node in ast.walk(want):
        if isinstance(node, (ast.Name, ast.arg, ast.Attribute, ast.ClassDef,
                             ast.alias) + FUNCS):
            new = by_pos.get(_key(node))
            if new is not None:
                setattr(node, field.get(type(node), "name"), new)
    try:
        got = ast.parse(out)
    except SyntaxError as exc:
        raise MinifyError("%s: renamed source does not parse (%s)"
                          % (name, exc)) from None
    if ast.dump(got) != ast.dump(want):
        raise MinifyError("%s: renamed source is a different program" % name)
    return out


def minify(source, name="<source>", outside=None, attributes=None,
           free=frozenset(), imports_outside=None):
    """``attributes`` is ``attribute_plan``'s, made over every module this
    one ships with; ``free``, ``free_parameters``'; ``imports_outside``,
    every name some module reads as an attribute or imports by name, and
    every word outside the bundle."""
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise MinifyError("%s does not parse: %s" % (name, exc)) from None
    source = _rewrite(source, _attribute_edits(tree, attributes), name)
    tree = ast.parse(source)
    source = _rewrite(source, _module_edits(tree, outside, imports_outside),
                      name)
    tree = ast.parse(source)
    return _rewrite(source, _apply(tree, free) + _self_edits(tree), name)


# Callees whose keywords stay their own: attributes of the namespace made,
# or ``Thread``'s parameters.  They reach a function of ours only through a
# namespace splatted back out (``f(**vars(ns))``, ``f(**ns.__dict__)``).
OWN_KEYWORDS = {"SimpleNamespace", "Thread"}


def _splats_a_namespace(call):
    return any(k.arg is None and (
        (isinstance(k.value, ast.Call) and _ident_of(k.value.func) == "vars")
        or (isinstance(k.value, ast.Attribute)
            and k.value.attr == "__dict__")) for k in call.keywords)


def free_parameters(sources, outside, callers=()):
    """``(function name, parameter)`` pairs no call can pass by keyword.  A
    keyword at a call to a function the sources define (by its last name:
    ``f(...)``, ``x.f(...)``) pins that parameter in every function so
    named; one at any other call (``partial``, ``dict``, a name some
    variable or attribute also holds) pins it everywhere -- ``OWN_KEYWORDS``
    excepted while no namespace is splatted into a function -- as does a
    word of a string that can become a dict key (``f(**{"name": ...})``)
    or of ``outside``.  ``callers`` (the tests) pin the same way but
    have no parameters of their own to free.  ``*args``/``**kwargs`` always move; a
    dunder's parameters never do."""
    trees = [ast.parse(source) for source in sources]
    every = trees + [ast.parse(source) for source in callers]
    defined, classes, held = set(), set(), set()
    for tree in every:
        for n in ast.walk(tree):
            if isinstance(n, FUNCS):
                defined.add(n.name)
            elif isinstance(n, ast.ClassDef):
                classes.add(n.name)
            elif isinstance(n, ast.Name) and not isinstance(n.ctx, ast.Load):
                held.add(n.id)
            elif isinstance(n, ast.Attribute) and not isinstance(
                    n.ctx, ast.Load):
                held.add(n.attr)
            elif isinstance(n, ast.arg):
                held.add(n.arg)
    known = defined - held - classes
    # A call to a class reaches only ``__init__``, whose parameters stay.
    constructors = classes - held - defined
    if not any(isinstance(n, ast.Call) and _splats_a_namespace(n)
               and _ident_of(n.func) not in constructors | OWN_KEYWORDS
               for tree in every for n in ast.walk(tree)):
        constructors |= OWN_KEYWORDS - held - defined
    everywhere, pinned = set(outside), set()
    for tree in every:
        # Only a string that can become a dict key reaches ``f(**d)``.
        keys = {id(k) for n in ast.walk(tree) if isinstance(n, ast.Dict)
                for k in n.keys if k is not None}
        keys |= {id(m) for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and _ident_of(n.func) in ("dict", "fromkeys", "zip")
                 for a in n.args for m in ast.walk(a)}
        for n in ast.walk(tree):
            if isinstance(n, ast.Call):
                callee = _ident_of(n.func)
                if callee in constructors:
                    continue
                for k in n.keywords:
                    if k.arg is None:   # f(**kw): any of f's
                        pinned.add((callee, "*"))
                    elif callee in known:
                        pinned.add((callee, k.arg))
                    else:
                        everywhere.add(k.arg)
            elif (isinstance(n, ast.Constant) and isinstance(n.value, str)
                  and id(n) in keys):
                everywhere |= set(WORD.findall(n.value))
    out = set()
    for tree in trees:
        for f in ast.walk(tree):
            if isinstance(f, FUNCS):
                a = f.args
                for x in a.posonlyargs + a.args + a.kwonlyargs:
                    if x.arg not in everywhere and not pinned & {
                            (f.name, x.arg), (f.name, "*")}:
                        out.add((f.name, x.arg))
    return frozenset(out)


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(minify(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
