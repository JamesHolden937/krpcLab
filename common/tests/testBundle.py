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

    def test_a_closure_keeps_its_name(self):
        src = ("def f():\n"
               "    kept = 5\n"
               "    gone = 1\n"
               "    return (lambda: kept)() + gone\n")
        out = pyminify.minify(src)
        self.assertIn("kept", out)
        self.assertNotIn("gone", out)
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
        is_package, source = modules["common.vec"]
        modules["common.vec"] = (is_package, source + "\nx = 1\n")
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

    def test_the_same_sources_give_the_same_bytes(self):
        self.assertEqual(bundle.build()[0],
                         open(self.path, encoding="utf-8").read())


if __name__ == "__main__":
    unittest.main()
