"""``tools/farmflight.py``: which log a harness's flight wrote."""
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, ROOT)
import farmflight  # noqa: E402
from common.logbook import Logbook  # noqa: E402


class TestAFlightFindsItsOwnLog(unittest.TestCase):
    """Three harnesses launched within a second could not tell their logs
    apart by time; the header's pid can."""

    def setUp(self):
        self.saved = farmflight.LOGS
        farmflight.LOGS = tempfile.mkdtemp()

    def tearDown(self):
        farmflight.LOGS = self.saved

    def write(self, number, header):
        with open(os.path.join(farmflight.LOGS, "LOG%d" % number), "w") as fh:
            fh.write(header + "\n")

    def test_the_pid_picks_the_log_and_the_first_of_a_handover(self):
        self.write(1, "# krpcLab log, opened 2026-10-27 17:02:00, pid 111")
        self.write(2, "# krpcLab log, opened 2026-10-27 17:02:00, pid 222")
        self.write(3, "# krpcLab log, opened 2026-10-27 17:09:00, pid 222")
        claim = farmflight.claim_log
        self.assertTrue(claim(set(), 0.0, 222).endswith("LOG2"))
        self.assertTrue(claim(set(), 0.0, 111).endswith("LOG1"))
        # Same second, no pid of ours: admitted, not guessed.
        self.assertTrue(claim(set(), 0.0, 999).startswith("?"))

    def test_a_logbook_header_carries_its_pid(self):
        log = Logbook(farmflight.LOGS, 1.0)
        log.close()
        self.assertEqual(farmflight.opened_by(log.path), os.getpid())


if __name__ == "__main__":
    unittest.main()
