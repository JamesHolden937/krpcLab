"""``tools/bundle.py`` and ``tools/pystrip.py``: one file carrying every
autopilot, comments stripped without changing a program."""

import ast
import io
import os
import sys
import tempfile
import tokenize
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import bundle    # noqa: E402
import pyalias   # noqa: E402
import pycompact  # noqa: E402
import pyminify  # noqa: E402
import pyshort   # noqa: E402
import pystrip   # noqa: E402


class TestTheStripper(unittest.TestCase):
    """Comments and docstrings out; everything else exactly as it was."""

    def test_comments_go_and_hashes_in_strings_stay(self):
        src = ('x = "a # b"  # gone\n'
               "# gone too\n"
               "y = f'{x}#'\n"
               "z = (1,  # inside brackets\n"
               "     2)\n")
        out = pystrip.strip(src)
        self.assertEqual(out, 'x = "a # b"\ny = f\'{x}#\'\nz = (1,\n     2)\n')

    def test_docstrings_go_and_a_bare_body_gets_pass(self):
        src = ('"""module"""\n'
               "class E(Exception):\n"
               '    """only a docstring"""\n'
               "def f():\n"
               "    '''doc'''\n"
               "    return 1\n")
        out = pystrip.strip(src)
        self.assertEqual(out, "class E(Exception):\n    pass\n"
                              "def f():\n    return 1\n")

    def test_multiline_string_contents_are_untouched(self):
        src = ('s = """line one   \n'
               "\n"
               "    # not a comment\n"
               '"""\n'
               "t = s  # gone\n")
        out = pystrip.strip(src)
        self.assertEqual(out, 's = """line one   \n\n    # not a comment\n'
                              '"""\nt = s\n')

    def test_a_string_statement_that_is_not_a_docstring_stays(self):
        src = "def f():\n    x = 1\n    'kept'\n    return x\n"
        self.assertEqual(pystrip.strip(src), src)

    def test_backslash_continuations_survive(self):
        src = "x = 1 + \\\n    2  # c\n"
        self.assertEqual(pystrip.strip(src), "x = 1 + \\\n    2\n")

    def test_a_module_reading_its_doc_keeps_it(self):
        src = '"""Usage text."""\n# c\nprint(__doc__)\n'
        self.assertEqual(pystrip.strip(src), '"""Usage text."""\nprint(__doc__)\n')

    def test_a_class_reading_its_own_doc_is_refused(self):
        src = 'class A:\n    """doc"""\n    d = __doc__\n'
        with self.assertRaises(pystrip.StripError):
            pystrip.strip(src)

    def test_the_tree_check_can_fail(self):
        """The proof has to be able to say no, or it is decoration."""
        original = pystrip._without_docstrings
        pystrip._without_docstrings = lambda tree: ast.parse("x = 2")
        try:
            with self.assertRaises(pystrip.StripError):
                pystrip.strip("x = 1  # c\n")
        finally:
            pystrip._without_docstrings = original


class TestTheRenamer(unittest.TestCase):
    """Locals become a, b, ...; nothing anyone outside can see moves."""

    def run_both(self, src, call):
        before, after = {}, {}
        exec(src, before)
        exec(pyminify.minify(src), after)
        self.assertEqual(eval(call, before), eval(call, after))

    def test_locals_shrink_and_params_stay(self):
        src = ("def f(speed, height=2):\n"
               "    total = speed + height\n"
               "    doubled = total * 2\n"
               "    return doubled\n")
        out = pyminify.minify(src)
        self.assertIn("def f(speed, height=2):", out)
        self.assertNotIn("total", out)
        self.run_both(src, "f(3, height=4)")

    def test_a_closure_moves_unless_a_nested_scope_binds_it(self):
        src = ("def f():\n"
               "    read = 5\n"
               "    gone = 1\n"
               "    shadowed = 2\n"
               "    def g():\n        shadowed = 7\n        return shadowed\n"
               "    class K:\n        z = gone\n"
               "    return (lambda: read)() + gone + shadowed + g() + K.z\n")
        out = pyminify.minify(src)
        self.assertNotIn("read", out)
        self.assertIn("shadowed", out)
        self.assertIn("gone", out)          # a nested class mentions it
        self.run_both(src, "f()")

    def test_no_new_name_hides_a_global(self):
        src = ("a = 10\n"
               "def f():\n"
               "    local = 1\n"
               "    return local + a\n")
        self.run_both(src, "f()")

    def test_comprehension_targets_shadow(self):
        src = ("def f():\n"
               "    value = 3\n"
               "    out = [value + i for i in range(value)]\n"
               "    return out, [x for x in out]\n")
        self.run_both(src, "f()")

    def test_fstrings_and_dynamic_scopes_are_left_alone(self):
        src = ("def f():\n    width = 4\n    return f'{width}'\n"
               "def g():\n    height = 2\n    return locals()\n")
        out = pyminify.minify(src)
        self.assertIn("width", out)
        self.assertIn("height", out)
        self.run_both(src, "(f(), g())")

    def test_self_becomes_s(self):
        src = ("class A:\n"
               "    def __init__(self, v):\n        self.v = v\n"
               "    def get(self):\n        return (lambda: self.v)()\n"
               "    @staticmethod\n    def same(self):\n        return self\n")
        out = pyminify.minify(src)
        self.assertIn("def get(s):", out)
        self.assertIn("def same(self):", out)
        self.run_both(src, "(A(3).get(), A.same(4))")

    def test_a_nested_def_is_a_local(self):
        src = ("def f(n):\n"
               "    def helper(x):\n        return x * 2\n"
               "    def recurse(x):\n        return x and recurse(x - 1) + 1\n"
               "    return helper(n) + recurse(n)\n")
        out = pyminify.minify(src)
        self.assertNotIn("helper", out)
        self.assertNotIn("recurse", out)
        self.run_both(src, "f(3)")

    def test_names_move_only_when_nothing_outside_names_them(self):
        src = ("LIMIT = 4\n"
               "class Holder:\n"
               "    def __init__(self, v):\n        self._value = v\n"
               "    def _double(self):\n        return self._value * 2\n"
               "    def total(self):\n        return self._double() + LIMIT\n"
               "def helper(x):\n    return Holder(x).total()\n"
               "def public(x):\n    return helper(x) + len('helper')\n")
        plan = pyminify.attribute_plan({"m": src}, {"public"}, {"total"})
        self.assertEqual(set(plan), {"_value", "_double"})
        out = pyminify.minify(src, outside={"public"}, attributes=plan)
        self.assertIn("def public(", out)
        self.assertIn(".total()", out)  # foreign: kept
        self.assertIn("helper", out)  # read in a string: kept
        self.assertNotIn("LIMIT", out)
        self.assertNotIn("Holder", out)
        self.assertNotIn("_double", out)
        before, after = {}, {}
        exec(src, before)
        exec(out, after)
        self.assertEqual(before["public"](3), after["public"](3))
        self.assertEqual(pyminify.minify(src).count("LIMIT"), 2)

    def test_a_keyword_or_a_field_keeps_an_attribute(self):
        src = ("from types import SimpleNamespace\n"
               "class A:\n"
               "    width: int = 1\n"
               "    def __init__(self):\n"
               "        self.ut = 1\n        self.width = 2\n"
               "        self.depth = 3\n"
               "    def read(self, x):\n        return x.ut + self.depth\n"
               "def f():\n    return A().read(SimpleNamespace(ut=4))\n")
        plan = pyminify.attribute_plan({"m": src}, set(), {"read"})
        self.assertEqual(set(plan), {"depth"})
        before, after = {}, {}
        exec(src, before)
        exec(pyminify.minify(src, outside=set(), attributes=plan), after)
        self.assertEqual(before["f"](), after["f"]())

    def test_a_parameter_moves_unless_some_call_names_it(self):
        src = ("from types import SimpleNamespace\n"
               "def area(width, height):\n    return width * height\n"
               "def volume(width, depth):\n    return width * depth\n"
               "def ns(depth):\n    return SimpleNamespace(depth=depth)\n"
               "def run():\n"
               "    return area(2, height=3) + volume(4, 5) + ns(1).depth\n")
        free = pyminify.free_parameters([src], set())
        self.assertIn(("area", "width"), free)
        self.assertNotIn(("area", "height"), free)   # area(height=...)
        self.assertIn(("volume", "depth"), free)  # SimpleNamespace's own
        splat = src + "def again():\n    return volume(1, **vars(ns(2)))\n"
        self.assertNotIn(("volume", "depth"),
                         pyminify.free_parameters([splat], set()))
        out = pyminify.minify(src, outside={"run", "area", "volume", "ns"},
                              free=free)
        self.assertIn("height", out)
        self.assertNotIn("def area(width", out)
        before, after = {}, {}
        exec(src, before)
        exec(out, after)
        self.assertEqual(before["run"](), after["run"]())

    def test_an_init_parameter_moves_unless_a_construction_names_it(self):
        src = ("class Base:\n    def __init__(self, depth):\n"
               "        self.depth = depth\n"
               "class Box(Base):\n    def __init__(self, width, height):\n"
               "        super().__init__(2)\n        self.v = width * height\n"
               "class Free:\n    def __init__(self, weight):\n"
               "        self.w = weight\n"
               "def run():\n    return Box(1, height=3).v + Base(4).depth\n")
        free = pyminify.free_parameters([src], set())
        self.assertIn(("Box.__init__", "width"), free)
        self.assertNotIn(("Box.__init__", "height"), free)
        self.assertIn(("Base.__init__", "depth"), free)
        self.assertIn(("Free.__init__", "weight"), free)
        # A subclass built with a keyword reaches its bases' __init__ too,
        # and a class handed on as a value keeps all of them.
        more = src + "def k():\n    return Box(1, 2, depth=3)\n"
        self.assertNotIn(("Base.__init__", "depth"),
                         pyminify.free_parameters([more], set()))
        handed = src + "FACTORY = Free\n"
        self.assertNotIn(("Free.__init__", "weight"),
                         pyminify.free_parameters([handed], set()))
        # A test's own helper called by keyword pins nothing.
        helper = "def helper(width):\n    return width\nhelper(width=1)\n"
        self.assertIn(("Box.__init__", "width"), pyminify.free_parameters(
            [src], set(), [helper]))
        out = pyminify.minify(src, outside={"run", "Box", "Base", "Free"},
                              free=free)
        self.assertNotIn("weight", out)
        before, after = {}, {}
        exec(src, before)
        exec(out, after)
        self.assertEqual(before["run"](), after["run"]())

    def test_an_import_takes_a_short_alias(self):
        # cmath, not math: a word of any test's string pins that name in
        # every bundled module, and the bundle imports math everywhere.
        src = ("import collections\nimport cmath\n"
               "def f(x):\n    return cmath.sqrt(x) + len(collections.deque())\n")
        out = pyminify.minify(src, outside={"cmath", "collections"},
                              imports_outside={"collections"})
        self.assertIn("import cmath as ", out)
        self.assertNotIn("collections as", out)
        before, after = {}, {}
        exec(src, before)
        exec(out, after)
        self.assertEqual(before["f"](4), after["f"](4))

    def test_names_count_up(self):
        self.assertEqual([pyminify.names(i) for i in (0, 25, 26, 27, 701)],
                         ["a", "z", "aa", "ab", "zz"])


class TestTheCompactor(unittest.TestCase):
    """Fewest bytes, same tree."""

    def same(self, src, call):
        before, after = {}, {}
        exec(src, before)
        exec(pycompact.compact(src), after)
        self.assertEqual(eval(call, before), eval(call, after))

    def test_indent_brackets_and_spaces(self):
        src = ("def f(a,\n      b):\n    if a:\n        return [a,\n"
               "                b]\n    return b\n")
        self.assertEqual(pycompact.compact(src),
                         "def f(a,b):\n if a:return[a,b]\n return b\n")

    def test_simple_statements_share_a_line(self):
        self.assertEqual(pycompact.compact("x = 1\ny = 2\n"), "x=1;y=2\n")

    def test_tokens_that_would_fuse_keep_their_space(self):
        src = ('def f(x):\n    r = 1 if x else 2\n    s = r .real\n'
               '    t = 1 .real\n    u = "a" """b"""\n    return r, s, t, u\n')
        self.same(src, "f(0)")

    def test_literals_take_their_shortest_spelling(self):
        self.assertEqual(pycompact.compact('x = ("a "\n     "b")\ny = 0.5\n'),
                         "x=('a b');y=.5\n")
        for t in ("1.0", "0.00001", "1000000.0", "3.14159", "0x1F", "10"):
            self.assertEqual(float(pycompact._number(t)) if "x" not in t
                             else int(t, 16), float(t) if "x" not in t
                             else int(t, 16))

    def test_a_bare_tuple_loses_its_parentheses(self):
        src = ("def f(d, a, b):\n    x, y = (a, b)\n"
               "    for (i, j) in ((1, 2), (3, 4)):\n"
               "        d[(i, j)] = (i), (j)\n    return (x, y)\n")
        out = pycompact.compact(src)
        self.assertIn("x,y=a,b", out)
        self.assertIn("for i,j in(1,2),(3,4):d[i,j]=(i),(j)", out)
        self.assertIn("return x,y", out)

    def test_fstrings_survive_exactly(self):
        src = 'def f(x):\n    return f"{{x}} {x!r:>5} {\'a\'}"\n'
        self.same(src, "f(3)")


class TestTheAliaser(unittest.TestCase):
    """Builtins read through one letter, where nothing rebinds them."""

    def test_aliases_save_and_behave(self):
        src = ("def f(o):\n    return [getattr(o, 'real', 0), getattr(o, 'imag', 0),"
               " getattr(o, 'x', 1), getattr(o, 'y', 2)]\n")
        out = pyalias.alias(src)
        self.assertTrue(out.startswith("A=getattr\n"))
        a, b = {}, {}
        exec(src, a)
        exec(out, b)
        self.assertEqual(a["f"](3), b["f"](3))

    def test_a_rebound_builtin_and_super_are_left(self):
        src = ("len = 3\nclass K(object):\n    def m(self):\n"
               "        return super().__init__, len, len, len, len\n")
        self.assertEqual(pyalias.alias(src), src)

    def test_a_dataclass_keeps_its_field_types(self):
        src = ("from dataclasses import dataclass, fields\n@dataclass\n"
               "class C:\n" + "".join("    X%d: float = %d.5\n" % (i, i)
                                       for i in range(6)))
        out = pyalias.alias(src)
        self.assertIn("X0: A = 0.5", out)
        a, b = {}, {}
        exec(src, a)
        exec(out, b)
        self.assertEqual([f.type for f in b["fields"](b["C"])],
                         [float] * 6)

    def test_postponed_annotations_are_left(self):
        src = ("from __future__ import annotations\n"
               + "".join("X%d: float = %d.5\n" % (i, i) for i in range(6)))
        self.assertEqual(pyalias.alias(src), src)

    def test_a_repeated_literal_is_read_through_a_capital(self):
        src = ("def f(d):\n    return [d.get('threshold'), 'threshold',"
               " f'{d}threshold', 'threshold', 2.5e3, 2.5e3, 2.5e3, 2.5e3]\n"
               "def g(x):\n    match x:\n        case 'threshold':"
               " return 1\n")
        out = pyalias.alias(src)
        self.assertTrue(out.startswith("A='threshold';B=2.5e3\n"), out)
        self.assertIn("f'{d}threshold'", out)
        self.assertIn("case 'threshold'", out)
        a, b = {}, {}
        exec(src, a)
        exec(out, b)
        self.assertEqual(a["f"]({"threshold": 1}), b["f"]({"threshold": 1}))

    def test_a_module_function_is_read_once_after_its_import(self):
        src = ("import math\nX = 1\ndef f(v):\n    return [math.radians(v),"
               " math.radians(v + 1), math.radians(v + 2), math.radians(2)]\n")
        out = pyalias.alias(src, hoist={"math": {"radians"}})
        self.assertTrue(out.startswith("import math\nA=math.radians\nX = 1\n"),
                        out)
        self.assertNotIn("math.radians(v", out)
        a, b = {}, {}
        exec(src, a)
        exec(out, b)
        self.assertEqual(a["f"](3), b["f"](3))
        # With no hoist given, nothing moves.
        self.assertEqual(pyalias.alias(src), src)

    def test_a_rebound_module_or_an_fstring_read_is_left(self):
        reads = "    return [math.radians(v)] * 3 + [math.radians(1)] * 3\n"
        rebound = "import math\ndef f(v):\n    math = 3\n" + reads
        self.assertEqual(pyalias.alias(rebound, hoist={"math": {"radians"}}),
                         rebound)
        fstring = ("import math\ndef f(v):\n    return [f'{math.radians(v)}',"
                   " f'{math.radians(1)}', f'{math.radians(2)}']\n")
        self.assertEqual(pyalias.alias(fstring, hoist={"math": {"radians"}}),
                         fstring)


class TestTheShortener(unittest.TestCase):
    """Statements rewritten into shorter ones that do the same."""

    SRC = ("import os\nimport sys\nfrom a import b\nfrom a import c as d\n"
           "def f(x, g, h):\n    y = g(x)\n    if y is None:\n"
           "        return None\n    z = h(y)\n    if z:\n        return 1\n"
           "    if x > 2:\n        return x\n    return y\n"
           "def k(s):\n    s.a = 0\n    s.b = 0\n    s.c = 0.0\n    return s\n"
           "class K:\n    v = 1\n    if v:\n        pass\n"
           "if __name__ == '__main__':\n    sys.exit(f())\n")

    def test_each_rewrite(self):
        out = pyshort.shorten(self.SRC)
        for want in ("import os,sys\n", "from a import b,c as d\n",
                     "if (y:=g(x)) is None:\n        return\n",
                     "if z:=h(y):", "return x if x > 2 else y",
                     "s.a=s.b=0\n    s.c = 0.0"):
            self.assertIn(want, out)
        self.assertIn("    v = 1\n    if v:", out)     # no walrus in a class
        self.assertNotIn("__main__", out)

    def test_the_rewritten_functions_behave(self):
        src = self.SRC.replace("from a import b\nfrom a import c as d\n", "")
        a, b = {"__name__": "m"}, {"__name__": "m"}
        exec(src, a)
        exec(pyshort.shorten(src), b)
        for x in (0, 1, 3):
            for g in (lambda v: None, lambda v: v):
                args = (x, g, lambda v: v * 2)
                self.assertEqual(a["f"](*args), b["f"](*args))

    def test_a_conditional_assignment_pass_and_while_true(self):
        src = ("def f(c, g):\n    if c:\n        x = 1\n    else:\n"
               "        x = g(2)\n    while True:\n        if x:\n"
               "            pass\n        break\n    if c is not None:\n"
               "        y = 0 if c else 1\n    else:\n"
               "        y = 2 if x else 3\n    if c:\n        z = 1\n"
               "    elif g:\n        z = 2\n    else:\n        z = 3\n"
               "    return x, y, z\n")
        out = pyshort.shorten(src)
        self.assertIn("x=1 if c else g(2)", out)
        self.assertIn("while 1:", out)
        self.assertIn("            0\n", out)
        self.assertIn("y=(0 if c else 1) if c is not None else (2 if x", out)
        self.assertIn("elif g:", out)     # an elif chain stays
        a, b = {}, {}
        exec(src, a)
        exec(out, b)
        for c in (None, 0, 1):
            self.assertEqual(a["f"](c, lambda v: v), b["f"](c, lambda v: v))

    def test_a_while_test_and_a_multiline_value(self):
        src = ("def f(g):\n    n = g()\n    while n:\n        n = g()\n"
               "    if n:\n        return 1\n    return ('a'\n"
               "            'b')\n")
        out = pyshort.shorten(src)
        self.assertIn("n = g()\n    while n:", out)
        self.assertIn("return 1 if n else ('a'", out)


class TestTheSingleFileExport(unittest.TestCase):
    """The file someone copies to another machine is the software this
    repository tested, less its comments, with every autopilot in it."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        body, name, cls.count = bundle.build()
        cls.path = os.path.join(cls.tmp.name, name)
        with open(cls.path, "w", encoding="utf-8") as handle:
            handle.write(body)
        cls.modules = bundle.stripped(bundle.collect(bundle.PACKAGES))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_every_module_comes_out_as_it_went_in(self):
        self.assertEqual(bundle.verify(self.path, self.modules), self.count)

    def test_every_pilot_is_carried(self):
        for name, module in bundle.pilots():
            self.assertIn(module, self.modules, name)

    def test_no_comment_is_carried(self):
        for name, (_, source) in self.modules.items():
            comments = [t for t in tokenize.generate_tokens(
                io.StringIO(source).readline) if t.type == tokenize.COMMENT]
            self.assertEqual(comments, [], name)

    def test_a_changed_module_fails_the_check(self):
        modules = dict(self.modules)
        is_package, source = modules["common.geo"]   # not vec: see above
        modules["common.geo"] = (is_package, source + "\nx = 1\n")
        with self.assertRaises(SystemExit):
            bundle.verify(self.path, modules)

    def test_the_loader_and_every_module_compile(self):
        modules, loader = bundle.unpack(self.path)
        compile(loader, "__main__", "exec")
        for name, (_, source) in modules.items():
            compile(source, name, "exec")

    def test_it_is_plain_text_behind_its_shebang(self):
        with open(self.path, encoding="utf-8") as handle:
            text = handle.read()
        self.assertTrue(text.startswith(bundle.SHEBANG))
        compile(text, self.path, "exec")

    def test_the_dictionary_round_trips_and_shrinks(self):
        text = ("def reference_frame(position, velocity):\n"
                "    return position, velocity, reference_frame\n") * 30
        k, entries = bundle.dictionary(text)
        packed, table = bundle.encode(text, k, entries)
        self.assertLess(len(packed) + len("\n".join(table)), len(text) / 4)
        self.assertNotIn("reference_frame", packed)
        self.assertEqual(bundle.expand(packed, k, table), text)
        # The loader's own expander says the same.
        space = {"_A": bundle.ALPHABET, "_K": k, "_D": "\n".join(table)}
        loader = bundle.LOADER[bundle.LOADER.index("def _X"):
                               bundle.LOADER.index("def _main")]
        exec(loader, space)
        self.assertEqual(space["_X"](packed), text)

    def test_a_module_using_the_escape_is_refused(self):
        with self.assertRaises(SystemExit):
            bundle.payload({"m": (False, "x = 1 & 2\n")})

    def test_the_same_sources_give_the_same_bytes(self):
        self.assertEqual(bundle.build()[0],
                         open(self.path, encoding="utf-8").read())


if __name__ == "__main__":
    unittest.main()
