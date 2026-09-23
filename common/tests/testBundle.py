"""``tools/bundle.py``: the single-file export carries the tree verbatim."""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestTheSingleFileExport(unittest.TestCase):
    """``bundle.py`` carries the tree out verbatim, or it is worthless.

    The point of the export is that the file someone copies to another
    machine is the software this repository tested.  A bundler that reformats
    or drops a module gives them something else with the same name, and they
    would find out in flight.
    """

    def setUp(self):
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import bundle
        self.bundle = bundle
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def build(self, target):
        body, name, count = self.bundle.build(target)
        path = os.path.join(self.tmp.name, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)
        return path, count

    def test_the_booster_bundle_is_the_booster(self):
        path, count = self.build("booster")
        modules = self.bundle.collect(["common", "boosterland"])
        self.assertEqual(self.bundle.verify(path, modules), count)

    def test_the_spaceplane_bundle_carries_both_packages(self):
        path, count = self.build("plane")
        modules = self.bundle.collect(["common", "spaceplane"])
        self.assertEqual(self.bundle.verify(path, modules), count)
        self.assertIn("spaceplane.guidance", modules)
        self.assertIn("common.vec", modules)

    def test_a_changed_module_fails_the_check(self):
        """The check has to be able to fail, or it is decoration."""
        path, _ = self.build("booster")
        modules = self.bundle.collect(["common", "boosterland"])
        is_package, source = modules["common.vec"]
        modules["common.vec"] = (is_package, source + "\n# tampered\n")
        with self.assertRaises(SystemExit):
            self.bundle.verify(path, modules)

    def test_the_bundle_compiles(self):
        for target in ("booster", "plane"):
            path, _ = self.build(target)
            with open(path, encoding="utf-8") as handle:
                compile(handle.read(), path, "exec")


if __name__ == "__main__":
    unittest.main()
