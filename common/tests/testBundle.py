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

    def test_the_bundle_compiles(self):
        with open(self.path, encoding="utf-8") as handle:
            compile(handle.read(), self.path, "exec")


if __name__ == "__main__":
    unittest.main()
