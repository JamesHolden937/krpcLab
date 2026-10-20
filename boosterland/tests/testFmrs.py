"""fmrs.py's save edits: pick the booster, make it active, merge it back."""

import os
import tempfile
import unittest

from boosterland import fmrs
from common.logbook import Logbook
from common.sfs import Save


def vessel(name, pid, sit, lat, lon, alt, engines):
    lines = ["\t\tVESSEL", "\t\t{", "\t\t\tpersistentId = %s" % pid,
             "\t\t\tname = %s" % name, "\t\t\tsit = %s" % sit,
             "\t\t\tlandedAt =",
             "\t\t\tlat = %r" % lat, "\t\t\tlon = %r" % lon,
             "\t\t\talt = %r" % alt, "\t\t\tPART", "\t\t\t{",
             "\t\t\t\tname = probeCore"]
    if engines:
        lines += ["\t\t\t\tMODULE", "\t\t\t\t{",
                  "\t\t\t\t\tname = ModuleEnginesFX", "\t\t\t\t}"]
    return lines + ["\t\t\t}", "\t\t}"]


def save_text(vessels, active):
    lines = ["GAME", "{", "\tFLIGHTSTATE", "\t{", "\t\tUT = 100",
             "\t\tactiveVessel = %d" % active]
    for v in vessels:
        lines += v
    return "\r\n".join(lines + ["\t}", "}", ""])


PAD = (-0.0972, -74.5577)
UPPER = vessel("upper", 1, "FLYING", -0.0976, -72.3037, 28245.0, True)
BOOSTER = vessel("booster", 2, "FLYING", -0.0976, -72.3053, 28234.0, True)
FAIRING = vessel("fairing", 3, "FLYING", -0.0976, -72.3038, 28245.0, False)
FAR = vessel("far", 4, "FLYING", -0.0976, -71.0, 28000.0, True)
ROCK = vessel("rock", 5, "ORBITING", 2.4, -172.9, 1.5e10, False)


class TestFmrsSaves(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        os.makedirs(os.path.join(self.dir, "FMRS"))
        self.log = Logbook(self.dir, 1.0)

    def tearDown(self):
        self.log.close()
        self.tmp.cleanup()

    def write(self, name, text):
        with open(os.path.join(self.dir, name + ".sfs"), "w",
                  newline="") as fh:
            fh.write(text)

    def test_booster_is_the_nearest_engined_vessel_in_flight(self):
        self.write("s", save_text([ROCK, FAR, UPPER, FAIRING, BOOSTER], 2))
        save = Save.read(os.path.join(self.dir, "s.sfs"))
        self.assertEqual(fmrs.find_booster(save), 4)

    def test_pad_split_is_skipped_and_the_copy_has_the_booster_active(self):
        pad_upper = vessel("upper", 1, "PRELAUNCH", PAD[0], PAD[1], 72.0, True)
        pad_rest = vessel("rest", 2, "PRELAUNCH", PAD[0], PAD[1], 72.0, True)
        self.write("FMRS/FMRS_save_0", save_text([ROCK, pad_upper, pad_rest],
                                                 1))
        text = save_text([ROCK, UPPER, BOOSTER], 1)
        self.write("FMRS/FMRS_save_1", text)
        source, main_name, booster = fmrs.prepare_boostback(
            self.dir, fmrs.SAVES, self.log, "1")
        self.assertEqual((source, main_name, booster.name),
                         ("FMRS/FMRS_save_1", "upper", "booster"))
        out = os.path.join(self.dir, fmrs.BOOSTBACK + ".sfs")
        with open(out, newline="") as fh:
            written = fh.read()
        self.assertEqual(written, text.replace("activeVessel = 1",
                                               "activeVessel = 2"))

    def test_no_booster_anywhere_raises(self):
        self.write("FMRS/FMRS_save_0", save_text([ROCK, UPPER, FAIRING], 1))
        with self.assertRaises(RuntimeError):
            fmrs.prepare_boostback(self.dir, fmrs.SAVES, self.log, "1")

    def test_another_launchs_save_is_skipped(self):
        old_upper = vessel("upper", 7, "FLYING", -0.0976, -72.3037, 28245.0,
                           True)
        self.write("FMRS/FMRS_save_0", save_text([ROCK, old_upper, BOOSTER],
                                                 1))
        with self.assertRaises(RuntimeError):
            fmrs.prepare_boostback(self.dir, fmrs.SAVES, self.log, "1")
        self.assertFalse(os.path.exists(
            os.path.join(self.dir, fmrs.BOOSTBACK + ".sfs")))

    def test_merge_replaces_the_booster_and_keeps_the_active_vessel(self):
        landed = vessel("booster", 2, "LANDED", PAD[0], PAD[1], 70.0, True)
        self.write(fmrs.LANDED, save_text([ROCK, UPPER, landed], 2))
        self.write(fmrs.MAIN, save_text([ROCK, BOOSTER, FAIRING, UPPER], 3))
        fmrs.merge_booster(self.dir, "2", self.log)
        main = Save.read(os.path.join(self.dir, fmrs.MAIN + ".sfs"))
        names = [(v.name, v.sit) for v in main.vessels()]
        self.assertEqual(names, [("rock", "ORBITING"), ("fairing", "FLYING"),
                                 ("upper", "FLYING"), ("booster", "LANDED")])
        self.assertEqual(main.vessels()[main.active].name, "upper")
        self.assertEqual(main.newline, "\r\n")

    def test_destroyed_booster_is_removed_from_the_main_mission(self):
        self.write(fmrs.LANDED, save_text([ROCK, UPPER], 1))
        self.write(fmrs.MAIN, save_text([ROCK, BOOSTER, UPPER], 2))
        fmrs.merge_booster(self.dir, "2", self.log)
        main = Save.read(os.path.join(self.dir, fmrs.MAIN + ".sfs"))
        self.assertEqual([v.name for v in main.vessels()], ["rock", "upper"])
        self.assertEqual(main.active, 1)


if __name__ == "__main__":
    unittest.main()
