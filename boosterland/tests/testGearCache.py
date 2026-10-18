import os
import tempfile
import unittest

from boosterland import gearcache


class TestGearCache(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(tempfile.mkdtemp(), "data", "gearTimes.json")

    def test_unknown_part_means_no_answer(self):
        self.assertIsNone(gearcache.lookup(["landingLeg1-2"], self.path))
        gearcache.record({"landingLeg1-2": {"deploy": 1.5, "retract": 1.1}},
                         self.path)
        self.assertIsNone(gearcache.lookup(["landingLeg1-2", "other"],
                                           self.path))

    def test_records_merge(self):
        gearcache.record({"a": {"deploy": 1.5, "retract": 1.0}}, self.path)
        gearcache.record({"b": {"deploy": 2.0, "retract": 1.0}}, self.path)
        self.assertEqual(gearcache.lookup(["a", "b", "a"], self.path),
                         {"a": 1.5, "b": 2.0})

    def test_corrupt_file_reads_empty(self):
        os.makedirs(os.path.dirname(self.path))
        with open(self.path, "w") as fh:
            fh.write("{not json")
        self.assertEqual(gearcache.load(self.path), {})


if __name__ == "__main__":
    unittest.main()
