#!/usr/bin/env python3
"""Write text shorter with a phrase dictionary, as plain printable text.

    ./tools/pydict.py file.txt               # sizes, and a round-trip check

``pack(text)`` returns ``(packed, header)``: ``text`` with every phrase the
dictionary lists replaced by a code, and the strings the loader needs to
write it back (``expand(packed, header)``, and ``tools/bundle.py``'s
``_X``, which does the same in the bundle).

**Codes.**  A code is one, two or three printable characters:

- ``header["1"]``: characters too rare in ``text`` to be worth keeping as
  themselves -- each is a whole code, one byte;
- ``header["2"]``: lead characters, each followed by any character of
  ``header["A"]`` -- two bytes; ``&`` is one too, followed by one of the
  first ``header["K"]`` characters of the alphabet, or by two of the rest
  -- two or three bytes.

A character that is a code, a lead or the entry separator ``header["S"]``
is never written as itself: where ``text`` holds one, a code stands for it,
and ``header["L"]`` lists those characters, in code order.  The dictionary
``header["D"]`` lists every entry in code order, joined by the separator,
each written with the codes of shorter entries (an entry contains only
entries shorter than itself, so writing back always ends); a character
entry is empty there.  Line breaks are ordinary characters.

**Which phrases.**  Candidates are runs of up to ``TOKENS`` tokens (a word
or one other character) that recur.  ``text`` is cut at the cheapest set
of codes (a shortest path over the text, codes priced by their length);
every phrase whose uses no longer pay for its entry is dropped, the most
used get the shortest codes, the rarest characters become codes and leads
where that saves more than escaping them costs, and this repeats until the
size stops falling.
"""
import re
import sys

# Code characters: printable ASCII but the escape, a quote and a backslash
# (``_P`` is a raw string between triple quotes).
ESCAPE = "&"
PRINTABLE = "".join(c for c in map(chr, range(33, 127))
                    if c not in ESCAPE + "'\\")
TOKEN = re.compile(r"[A-Za-z0-9_]+|.", re.S)
TOKENS = 12
LONGEST = 100
CANDIDATES = 100000
ROUNDS = 30


def _trie(entries):
    trie = {}
    for e in entries:
        node = trie
        for ch in e:
            node = node.setdefault(ch, {})
        node[None] = e
    return trie


def _cut(s, trie, cost, lit, skip=None):
    """``s`` as the cheapest run of entries and characters: a list whose
    items are entries (keys of ``cost``) or single characters."""
    n = len(s)
    best = [0] * (n + 1)
    pick = [None] * n
    for i in range(n - 1, -1, -1):
        b, p = lit(s[i]) + best[i + 1], None
        node, j = trie, i
        while j < n:
            node = node.get(s[j])
            if node is None:
                break
            j += 1
            hit = node.get(None)
            if hit is not None and hit != skip:
                c = cost[hit] + best[j]
                if c < b:
                    b, p = c, hit
        best[i], pick[i] = b, p
    out, i = [], 0
    while i < n:
        if pick[i] is not None:
            out.append(pick[i])
            i += len(pick[i])
        else:
            out.append(s[i])
            i += 1
    return out


def _codes(alphabet, singles, leads, k):
    """Every code, shortest first: the order entries are ranked in."""
    return (list(singles) + [ESCAPE + c for c in alphabet[:k]]
            + [lead + c for lead in leads for c in alphabet]
            + [ESCAPE + c + d for c in alphabet[k:] for d in alphabet])


def _layout(ranked, uses, alphabet, singles, leads):
    """``(k, {entry: code length})``: the split of ``&``'s codes between two
    and three bytes that makes ``ranked``'s codes cheapest, every entry
    past the last code priced at its length."""
    best = None
    for k in range(1, len(alphabet)):
        two = k + len(leads) * len(alphabet)
        room = len(singles) + two + (len(alphabet) - k) * len(alphabet)
        cost, total = {}, 0
        for r, e in enumerate(ranked):
            if r >= room:
                total += uses.get(e, 0) * len(e)
                continue
            c = 1 if r < len(singles) else 2 if r < len(singles) + two else 3
            cost[e] = c
            total += uses.get(e, 0) * c
        if best is None or total < best[0]:
            best = (total, k, cost)
    return best[1], best[2]


def _candidates(text):
    count = {}
    toks = TOKEN.findall(text)
    for n in range(1, TOKENS + 1):
        for i in range(len(toks) - n + 1):
            p = "".join(toks[i:i + n])
            if 2 <= len(p) <= LONGEST:
                count[p] = count.get(p, 0) + 1

    def score(p):
        return count[p] * (len(p) - 2) - len(p) - 1
    return sorted((p for p in count if count[p] >= 3 and score(p) > 0),
                  key=lambda p: (-score(p), p))[:CANDIDATES]


def choose(text):
    """``(entries ranked, singles, leads, separator, k)``."""
    if ESCAPE in text:
        raise ValueError("text holds the escape %r" % ESCAPE)
    chars = {}
    for ch in text:
        chars[ch] = chars.get(ch, 0) + 1
    # Rarest first; the rarest of all separates the dictionary's entries.
    pool = sorted(PRINTABLE, key=lambda c: (chars.get(c, 0), c))
    sep, pool = pool[0], pool[1:]
    alphabet = "".join(c for c in PRINTABLE if c != sep)
    entries = _candidates(text)
    nsingle = nlead = 0
    cost = {e: 2 for e in entries}
    previous = None
    kept = None
    for _ in range(ROUNDS):
        freed = set([sep] + pool[:nsingle + nlead])

        def lit(ch):
            return 2 if ch in freed else 1
        trie = _trie(entries)
        cut = _cut(text, trie, cost, lit)
        uses, size = {}, 0
        for x in cut:
            if len(x) > 1:
                uses[x] = uses.get(x, 0) + 1
                size += cost[x]
            else:
                size += lit(x)
        own = {}
        for e in entries:
            b = 1                               # its separator
            for x in _cut(e, trie, cost, lit, skip=e):
                if len(x) > 1:
                    uses[x] = uses.get(x, 0) + 1
                    b += cost[x]
                else:
                    b += lit(x)
            own[e] = b
            size += b
        size += 3 * (nsingle + nlead + 1)
        if previous is not None and size >= previous[0]:
            break
        previous = (size, list(entries), nsingle, nlead)
        ranked = sorted(entries, key=lambda e: (-uses.get(e, 0), e))
        # Singles: the rarest characters, while a code of one byte for the
        # next most used entry saves more than escaping the character.
        gain, nsingle, top = 0, 0, 0
        for f in range(1, len(pool) // 2):
            gain += uses.get(ranked[f - 1], 0) - chars.get(pool[f - 1], 0) - 3
            if gain > top:
                nsingle, top = f, gain
        # Leads: the next rarest, while 91 more two-byte codes save more.
        nlead = 0
        for f in range(1, len(pool) - nsingle):
            lo = nsingle + len(alphabet) + (f - 1) * len(alphabet)
            g = (sum(uses.get(e, 0) for e in ranked[lo:lo + len(alphabet)])
                 - chars.get(pool[nsingle + f - 1], 0) - 3)
            if g > 0:
                nlead = f
            else:
                break
        _, lengths = _layout(ranked, uses, alphabet, pool[:nsingle],
                             pool[nsingle:nsingle + nlead])
        kept = [e for e in ranked if e in lengths
                and uses.get(e, 0) * (len(e) - lengths[e]) > own[e]]
        entries = kept
        cost = {e: lengths[e] for e in kept}
    _, entries, nsingle, nlead = previous
    return entries, pool[:nsingle], pool[nsingle:nsingle + nlead], sep, \
        alphabet


def pack(text):
    entries, singles, leads, sep, alphabet = choose(text)
    freed = set(singles) | set(leads) | {sep}
    # Rank by use, characters standing for themselves among them.
    trie = _trie(entries)
    flat = {e: 2 for e in entries}

    def lit(ch):
        return 2 if ch in freed else 1
    uses = {}
    for x in _cut(text, trie, flat, lit):
        uses[x] = uses.get(x, 0) + 1
    for e in entries:
        for x in _cut(e, trie, flat, lit, skip=e):
            uses[x] = uses.get(x, 0) + 1
    ranked = sorted(entries + sorted(c for c in freed if c in uses),
                    key=lambda e: (-uses.get(e, 0), e))
    k, lengths = _layout(ranked, uses, alphabet, singles, leads)
    ranked = [e for e in ranked if e in lengths]
    if any(c in freed and c not in lengths for c in uses if len(c) == 1):
        raise ValueError("a freed character has no code")
    table = dict(zip(ranked, _codes(alphabet, singles, leads, k)))
    words = {e: len(table[e]) for e in ranked if len(e) > 1}
    trie = _trie(list(words))

    def lit_exact(ch):
        return len(table[ch]) if ch in freed else 1

    def write(s, skip=None):
        return "".join(table[x] if (len(x) > 1 or x in freed) else x
                       for x in _cut(s, trie, words, lit_exact, skip))
    header = {
        "A": alphabet, "K": k, "1": "".join(singles), "2": "".join(leads),
        "S": sep,
        "D": sep.join("" if len(e) == 1 else write(e, skip=e)
                      for e in ranked),
        "L": "".join(e for e in ranked if len(e) == 1),
    }
    return write(text), header


def expand(packed, header):
    """What the loader's ``_X`` does, for reading a bundle back."""
    a, k = header["A"], header["K"]
    codes = _codes(a, header["1"], header["2"], k)
    entries = header["D"].split(header["S"])
    done, chars = {}, iter(header["L"])
    table = {}
    for code, e in zip(codes, entries):
        if e:
            table[code] = e
        else:
            done[code] = next(chars)
    parts = []
    if header["1"]:
        parts.append("[%s]" % re.escape(header["1"]))
    if header["2"]:
        parts.append("[%s]." % re.escape(header["2"]))
    parts.append(re.escape(ESCAPE) + "(?:[%s]%s)" % (
        re.escape(a[:k]), "|[%s]." % re.escape(a[k:]) if a[k:] else ""))
    pattern = re.compile("|".join(parts), re.S)

    def run(s):
        def one(m):
            code = m.group(0)
            if code not in done:
                done[code] = run(table[code])
            return done[code]
        return pattern.sub(one, s)
    return run(packed)


def main(argv=None):
    for path in (argv if argv is not None else sys.argv[1:]):
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
        packed, header = pack(text)
        size = len(packed) + sum(len(str(v)) for v in header.values())
        ok = expand(packed, header) == text
        print("%s: %d -> %d  round trip %s" % (path, len(text), size,
                                               "ok" if ok else "FAILED"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
