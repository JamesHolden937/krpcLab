#!/usr/bin/env python3
"""Strip comments and docstrings from Python source, provably.

    ./tools/pystrip.py module.py            # the stripped source on stdout

``strip(source)`` returns the source with every ``#`` comment, every
docstring (the string literal that opens a module, class or function body)
and every line those leave blank removed.  A body that was only a docstring
gets ``pass``.  Everything else -- code, string literals, line structure
inside brackets and multi-line strings -- is left as it was.

**Built on the tokenizer, checked against the parser.**  A regex cannot tell a
``#`` in a string from a comment, nor a blank line inside a triple-quoted
string from one between statements; the tokenizer can.  And the result is
then compared with the original's syntax tree (docstrings removed on that
side too): if the two differ in anything at all, ``strip`` raises rather
than return source that means something else.  A stripper whose output is
only *probably* the same program is not one to put in front of an autopilot.

A module that reads ``__doc__`` keeps its module docstring (it is the
``--help`` text), and one that reads a class's ``__doc__`` in the class body
is refused: removing that docstring would change what it does, and no tree
comparison can see it.
"""
import ast
import io
import sys
import tokenize


class StripError(ValueError):
    pass


def _reads_doc(nodes):
    return any(isinstance(n, ast.Name) and n.id == "__doc__"
               for node in nodes for n in ast.walk(node))


def _class_body_reads_doc(tree):
    """``__doc__`` read in a class body itself (not in a method), where it
    is the class's own docstring rather than the module's."""
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            direct = [s for s in node.body if not isinstance(
                s, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))]
            if _reads_doc(direct):
                return True
    return False


def _docstring_nodes(tree, keep_read=True):
    """``(node, alone)`` for every docstring: the ``Expr`` holding it, and
    whether it is its body's only statement (which then needs a ``pass``).
    A module that reads ``__doc__`` keeps its own (``--help`` text)."""
    found = []
    keep_module = keep_read and _reads_doc([tree])
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
            continue
        if isinstance(node, ast.Module) and keep_module:
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            found.append((body[0], len(body) == 1))
    return found


def _without_docstrings(tree):
    """The reference the stripped source must parse back to."""
    for expr, alone in _docstring_nodes(tree):
        for node in ast.walk(tree):
            body = getattr(node, "body", None)
            if isinstance(body, list) and body and body[0] is expr:
                node.body = ([ast.Pass()] if alone else body[1:])
                break
    return tree


def _offsets(lines):
    """Character offset of the start of each 1-based line."""
    starts, total = [0, 0], 0
    for line in lines:
        total += len(line)
        starts.append(total)
    return starts


def strip(source, name="<source>"):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise StripError("%s does not parse: %s" % (name, exc)) from None
    if _class_body_reads_doc(tree):
        raise StripError("%s reads a class's __doc__; stripping it would "
                         "change what it does" % name)
    lines = source.splitlines(keepends=True)
    starts = _offsets(lines)

    def at(row, col):
        return starts[row] + col

    # Character ranges to replace, and the rows nothing may touch: every row
    # a multi-line string (or f-string part) runs across, except the last --
    # their blank lines and trailing spaces are the string's contents.
    edits = []
    docstrings = _docstring_nodes(tree)
    doc_rows = set()
    for expr, alone in docstrings:
        # ``col_offset`` counts UTF-8 bytes; convert to characters.
        def col(row, byte_col):
            return len(lines[row - 1].encode("utf-8")[:byte_col]
                       .decode("utf-8"))
        a = at(expr.lineno, col(expr.lineno, expr.col_offset))
        b = at(expr.end_lineno, col(expr.end_lineno, expr.end_col_offset))
        edits.append((a, b, "pass" if alone else ""))
        doc_rows.update(range(expr.lineno, expr.end_lineno + 1))
    protected = set()
    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(source).readline))
    except (tokenize.TokenError, SyntaxError) as exc:
        raise StripError("%s does not tokenize: %s" % (name, exc)) from None
    for tok in tokens:
        (r0, c0), (r1, c1) = tok.start, tok.end
        if tok.type == tokenize.COMMENT:
            edits.append((at(r0, c0), at(r1, c1), ""))
        elif r1 > r0 and tok.type not in (tokenize.NL, tokenize.NEWLINE):
            if not (tok.type == tokenize.STRING and r0 in doc_rows):
                protected.update(range(r0, r1))
    out = source
    for a, b, text in sorted(edits, reverse=True):
        out = out[:a] + text + out[b:]

    # Line by line, now that the edits kept every row in place: drop what
    # they left blank, trim what they left trailing -- except inside strings.
    kept = []
    for row, line in enumerate(out.splitlines(keepends=True), start=1):
        if row in protected:
            kept.append(line)
            continue
        body = line.rstrip()
        if body.strip():
            kept.append(body + "\n")
    stripped = "".join(kept)

    try:
        got = ast.parse(stripped)
    except SyntaxError as exc:
        raise StripError("%s: stripped source does not parse (%s)"
                         % (name, exc)) from None
    want = _without_docstrings(ast.parse(source))
    if ast.dump(got) != ast.dump(want):
        raise StripError("%s: stripped source is a different program" % name)
    return stripped


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(strip(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
