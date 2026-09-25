import os
import unittest
from unittest import mock

import numpy as np

import serial_test
from LaneDetection import steering
from utlis import load_calibration


class SerialLinkTests(unittest.TestCase):
    def setUp(self):
        serial_test._serial, serial_test._connected, serial_test._last_sent = None, False, None

    def test_dry_run_without_board(self):
        with mock.patch.dict(os.environ, {"ARDUINO_PORT": ""}), \
             mock.patch.object(serial_test, "_find_port", return_value=None):
            self.assertFalse(serial_test.Send("S"))
        self.assertEqual(serial_test._last_sent, "S")

    def test_repeats_are_not_resent(self):
        fake = mock.Mock()
        serial_test._serial, serial_test._connected = fake, True
        self.assertTrue(serial_test.Send("F"))
        self.assertFalse(serial_test.Send("F"))
        self.assertTrue(serial_test.Send("S"))
        self.assertEqual(fake.write.call_count, 2)

    def test_rejects_unknown_command(self):
        with self.assertRaises(ValueError):
            serial_test.Send("X")


class LaneTests(unittest.TestCase):
    def test_steering_thresholds(self):
        self.assertEqual(steering(-100), "L")
        self.assertEqual(steering(0), "F")
        self.assertEqual(steering(100), "R")

    def test_calibration_loads_without_pickle(self):
        mtx, dist = load_calibration()
        self.assertEqual(mtx.shape, (3, 3))
        self.assertEqual(dist.shape, (1, 5))
        self.assertTrue(np.isfinite(mtx).all())


if __name__ == "__main__":
    unittest.main()
