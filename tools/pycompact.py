#!/usr/bin/env python3
"""Rewrite Python source in as few bytes as its tokens allow.

    ./tools/pycompact.py module.py          # the compacted source on stdout

One space per indentation level, every line continued inside brackets joined
onto its first, a space between two tokens only where they would otherwise
run together, consecutive simple statements joined with ``;``, and a block
whose body is simple statements put on its header's line (``if x:return``).
Comments must already be gone (``pystrip``); strings, f-strings included,
are copied from the source exactly; adjacent plain string literals are
joined (``"a" "b"`` -> ``"ab"``) and float literals take their shortest
spelling (``0.5`` -> ``.5``, ``1e-05`` -> ``1e-5``).

**Checked against the parser**: the result must parse to the identical tree
or ``compact`` raises.
"""
import ast
import io
import sys
import tokenize

COMPOUND = {"if", "elif", "else", "for", "while", "try", "except", "finally",
            "with", "def", "class", "async", "match", "case"}
WORDY = (tokenize.NAME, tokenize.NUMBER)


class CompactError(ValueError):
    pass


def _lines(source):
    """Logical lines as ``(depth, [token text], first word)``, with each
    f-string as one piece of text."""
    lines_src = source.splitlines(keepends=True)
    starts = [0, 0]
    for line in lines_src:
        starts.append(starts[-1] + len(line))

    def off(pos):
        row, col = pos
        return starts[row] + col

    toks = list(tokenize.generate_tokens(io.StringIO(source).readline))
    out, depth, cur, i = [], 0, [], 0
    fstart = getattr(tokenize, "FSTRING_START", None)
    fend = getattr(tokenize, "FSTRING_END", None)
    tstart = getattr(tokenize, "TSTRING_START", None)
    tend = getattr(tokenize, "TSTRING_END", None)
    while i < len(toks):
        t = toks[i]
        if t.type in (fstart, tstart) and t.type is not None:
            # Copy the whole f-string from the source: its tokens do not
            # round-trip (``{{`` comes back as ``{``).
            close = fend if t.type == fstart else tend
            level, j = 0, i
            while True:
                if toks[j].type == t.type:
                    level += 1
                elif toks[j].type == close:
                    level -= 1
                    if level == 0:
                        break
                j += 1
            cur.append((tokenize.STRING, source[off(t.start):off(toks[j].end)]))
            i = j + 1
            continue
        if t.type == tokenize.INDENT:
            depth += 1
        elif t.type == tokenize.DEDENT:
            depth -= 1
        elif t.type == tokenize.NEWLINE:
            if cur:
                out.append((cur_depth, cur))
            cur = []
        elif t.type in (tokenize.NL, tokenize.COMMENT, tokenize.ENDMARKER):
            pass
        else:
            if not cur:
                cur_depth = depth
            cur.append((t.type, t.string))
        i += 1
    return out


def _plain(text):
    """The value of a plain (not f-, not bytes) string literal, else None."""
    prefix = text[:len(text) - len(text.lstrip("rRuUbBfFtT"))].lower()
    if any(c in prefix for c in "bft"):
        return None
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return None
    return value if isinstance(value, str) else None


def _number(text):
    """The shortest spelling of a float literal (``0.5`` -> ``.5``,
    ``1.0`` -> ``1.``, ``0.00001`` -> ``1e-5``); ints and complex as they are."""
    lower = text.lower()
    if lower.endswith("j") or lower.startswith("0x") or lower.startswith(
            "0o") or lower.startswith("0b") or not ("." in lower or "e" in lower):
        return text
    value = float(text.replace("_", ""))
    best = text
    r = repr(value)
    for cand in (r, r[:-1] if r.endswith(".0") else r,
                 r[1:] if r.startswith("0.") else r):
        if cand and float(cand) == value and ("." in cand or "e" in cand) \
                and len(cand) < len(best):
            best = cand
    # Exponent forms: 1e-05 -> 1e-5, 1000000.0 -> 1e6.
    for digits in range(1, 18):
        cand = ("%." + str(digits) + "g") % value
        cand = cand.replace("e-0", "e-").replace("e+0", "e").replace("e+", "e")
        if "." not in cand and "e" not in cand:
            continue
        if float(cand) == value and len(cand) < len(best):
            best = cand
            break
    return best


def _merge(tokens):
    """Adjacent plain string literals become one (``"a" "b"`` -> ``"ab"``),
    numbers their shortest spelling."""
    out = []
    for kind, s in tokens:
        if kind == tokenize.NUMBER:
            s = _number(s)
        elif (kind == tokenize.STRING and out and out[-1][0] == tokenize.STRING
              and _plain(out[-1][1]) is not None and _plain(s) is not None):
            joined = repr(_plain(out[-1][1]) + _plain(s))
            if len(joined) < len(out[-1][1]) + len(s) + 1:
                out[-1] = (kind, joined)
                continue
        out.append((kind, s))
    return out


def _join(tokens):
    """One logical line's tokens with only the spaces that are needed."""
    tokens = _merge(tokens)
    text = ""
    prev = None
    for kind, s in tokens:
        if prev is not None:
            pk, ps = prev
            space = False
            if pk in WORDY and kind in WORDY:
                space = True
            elif pk in WORDY and kind == tokenize.STRING:
                space = True                # ``r 'x'`` must not become r'x'
            elif pk == tokenize.NUMBER and s.startswith("."):
                space = True                # ``1 .real``
            elif pk == tokenize.STRING and kind == tokenize.STRING:
                space = True                # ``"a" """b"""`` stays two
            if pk == tokenize.NUMBER and kind == tokenize.NAME:
                space = True                # ``1if`` warns
            text += " " if space else ""
        text += s
        prev = (kind, s)
    return text


def _simple(tokens):
    return tokens[0][1] not in COMPOUND and tokens[0][1] != "@"


def compact(source, name="<source>"):
    try:
        want = ast.dump(ast.parse(source))
    except SyntaxError as exc:
        raise CompactError("%s does not parse: %s" % (name, exc)) from None
    lines = _lines(source)
    out = []
    i = 0
    while i < len(lines):
        depth, toks = lines[i]
        text = _join(toks)
        if not _simple(toks) and text.endswith(":"):
            # A body of simple statements, one level in, ending where the
            # block ends: onto the header's line.
            j = i + 1
            body = []
            while (j < len(lines) and lines[j][0] == depth + 1
                   and _simple(lines[j][1])):
                body.append(_join(lines[j][1]))
                j += 1
            ends = j >= len(lines) or lines[j][0] <= depth
            if body and ends:
                out.append((depth, text + ";".join(body), False))
                i = j
                continue
        out.append((depth, text, _simple(toks)))
        i += 1
    # Consecutive simple statements at one depth share a line.
    merged = []
    for depth, text, simple in out:
        if (merged and simple and merged[-1][2]
                and merged[-1][0] == depth):
            merged[-1] = (depth, merged[-1][1] + ";" + text, True)
        else:
            merged.append((depth, text, simple))
    result = "".join(" " * d + t + "\n" for d, t, _ in merged)
    try:
        got = ast.dump(ast.parse(result))
    except SyntaxError as exc:
        raise CompactError("%s: compacted source does not parse (%s)"
                           % (name, exc)) from None
    if got != want:
        raise CompactError("%s: compacted source is a different program"
                           % name)
    return result


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            sys.stdout.write(compact(handle.read(), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
