"""実機に接続せず、ID4の負荷検知を確認する。"""

import struct
import unittest
from unittest.mock import patch

import test as app


class GripperLoadTests(unittest.TestCase):
    def setUp(self):
        self.usb = object()
        with patch.object(app, "read_data", side_effect=[struct.pack("<HH", 0, 4095), bytes([0])]), \
                patch.object(app, "read_position", return_value=2300):
            self.gripper = app.create_gripper(self.usb)
        self.reader = patch.object(app, "read_data").start()
        self.move = patch.object(app, "set_position").start()
        self.enable = patch.object(app, "enable_torque").start()
        self.addCleanup(patch.stopall)

    def close(self):
        with patch.object(app.time, "monotonic", return_value=100):
            app.toggle_gripper(self.usb, self.gripper)

    def feedback(self, position, load, now):
        self.reader.return_value = struct.pack("<HHH", position, 0, load)
        app.update_gripper(self.usb, self.gripper, now)
        self.reader.assert_called_with(self.usb, app.GRIPPER_ID, 56, 6)

    def test_close_starts_monitoring_without_full_close_command(self):
        self.close()
        self.enable.assert_called_once_with(self.usb, 4)
        self.move.assert_not_called()
        self.feedback(2300, 50, 100.1)
        self.move.assert_called_once_with(self.usb, 4, 2290)

    def test_contact_holds_measured_position_and_next_click_opens(self):
        self.close()
        self.feedback(2110, app.GRIPPER_LOAD_THRESHOLD, 100.1)
        self.move.assert_not_called()  # 確認中も閉じ進めない
        self.feedback(2107, app.GRIPPER_LOAD_THRESHOLD + 30, 100.2)
        self.move.assert_called_once_with(self.usb, 4, 2107)
        self.assertFalse(self.gripper["closing"])
        self.assertFalse(self.gripper["next_closed"])
        self.feedback(2107, 0, 100.3)
        self.assertEqual(self.move.call_count, 1)  # 負荷が下がっても閉じ直さない
        app.toggle_gripper(self.usb, self.gripper)
        self.move.assert_called_with(self.usb, 4, 2300)

    def test_direction_bit_is_not_load_and_spike_resets(self):
        self.close()
        self.feedback(2200, 1024 + 50, 100.1)
        self.assertEqual(self.gripper["load"], 50)
        self.move.assert_called_with(self.usb, 4, 2190)
        self.feedback(2190, 1024 + app.GRIPPER_LOAD_THRESHOLD, 100.2)
        self.assertEqual(self.gripper["load_hits"], 1)
        self.feedback(2185, 10, 100.3)
        self.assertEqual(self.gripper["load_hits"], 0)
        self.feedback(2180, 1024 + app.GRIPPER_LOAD_THRESHOLD, 100.4)
        self.assertTrue(self.gripper["closing"])
        self.feedback(2178, 1024 + app.GRIPPER_LOAD_THRESHOLD, 100.5)
        self.move.assert_called_with(self.usb, 4, 2178)
        self.assertFalse(self.gripper["closing"])

    def test_no_contact_reaches_closed_position_without_overshoot(self):
        self.close()
        self.feedback(1755, 0, 100.1)
        self.move.assert_called_once_with(self.usb, 4, 1750)
        self.feedback(1750, 0, 100.2)
        self.assertFalse(self.gripper["closing"])
        self.assertEqual(self.move.call_count, 1)

    def test_opening_does_not_stop_on_high_load(self):
        self.close()
        app.toggle_gripper(self.usb, self.gripper)
        self.move.assert_called_once_with(self.usb, 4, 2300)
        self.feedback(2000, 900, 100.1)
        self.feedback(2100, 900, 100.2)
        self.assertEqual(self.move.call_count, 1)

    def test_timeout_holds_instead_of_continuing(self):
        self.close()
        self.feedback(2200, 0, 100 + app.GRIPPER_CLOSE_TIMEOUT)
        self.move.assert_called_once_with(self.usb, 4, 2200)
        self.assertFalse(self.gripper["closing"])

    def test_feedback_rate_and_invalid_position(self):
        self.close()
        self.feedback(2200, 0, 100.1)
        self.reader.reset_mock()
        app.update_gripper(self.usb, self.gripper, 100.11)
        self.reader.assert_not_called()
        self.reader.return_value = struct.pack("<HHH", 5000, 0, 0)
        count = self.move.call_count
        with self.assertRaises(RuntimeError):
            app.update_gripper(self.usb, self.gripper, 100.2)
        self.assertEqual(self.move.call_count, count)


if __name__ == "__main__":
    unittest.main()
