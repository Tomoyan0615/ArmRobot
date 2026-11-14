import math
import random
import struct
import unittest
from unittest.mock import patch

import test as app


class JointLimitsTests(unittest.TestCase):
    def controls(self, id2=2048, id3=2048, id3_hardware=(0, 4095)):
        positions = {1: 2048, 2: id2, 3: id3}

        def read(usb, sid, address, size):
            if address == 33:
                return bytes([0])
            return struct.pack("<HH", *(id3_hardware if sid == 3 else (0, 4095)))

        with patch.object(app, "read_data", side_effect=read), \
                patch.object(app, "read_position", side_effect=lambda usb, sid: positions[sid]):
            return app.create_controls(None)

    def test_reference_range_and_no_startup_motion(self):
        with patch.object(app, "set_position") as move:
            controls = self.controls()
        move.assert_not_called()
        self.assertEqual((controls[2]["minimum"], controls[2]["maximum"]), (1536, 2446))

    def test_id2_stops_at_135_when_id3_is_260(self):
        controls = self.controls(id2=1423, id3=2958)
        second, third = controls[1:]
        self.assertEqual((second["minimum"], second["maximum"]), (1423, 1536))
        second.update(target=2389, pending=True)
        with patch.object(app, "read_position", side_effect=lambda usb, sid: controls[sid - 1]["current"]), \
                patch.object(app, "enable_torque") as enable, patch.object(app, "set_position") as move:
            app.send_arm_target(None, controls, second, 100)
        move.assert_called_once_with(None, 2, 1536)
        enable.assert_called_once_with(None, 2)
        self.assertEqual(third["current"], 2958)
        self.assertEqual(third["sent_target"], 2958)
        self.assertFalse(third["pending"])

    def test_in_flight_target_is_included(self):
        controls = self.controls()
        second, third = controls[1:]
        second.update(enabled=True, sent_target=2389)
        app.update_joint_limits(controls)
        self.assertEqual((third["minimum"], third["maximum"]), (1536, 2105))
        # 両軸の目標だけを見ればよいのではなく、現在位置も考慮する。
        second["sent_target"] = 1423
        app.update_joint_limits(controls)
        self.assertEqual((third["minimum"], third["maximum"]), (2048, 2446))

    def test_range_update_does_not_change_another_target_or_send(self):
        controls = self.controls()
        third = controls[2]
        old_target = third["target"]
        controls[1]["current"] = 2389
        with patch.object(app, "set_position") as move:
            app.update_joint_limits(controls)
        move.assert_not_called()
        self.assertEqual(third["target"], old_target)
        self.assertFalse(third["pending"])

    def test_hardware_limits_are_intersected(self):
        third = self.controls(id3_hardware=(1600, 2200))[2]
        self.assertEqual((third["minimum"], third["maximum"]), (1600, 2200))

    def test_no_feasible_range_does_not_move_and_can_recover(self):
        controls = self.controls(id2=1423, id3=0)
        second = controls[1]
        self.assertTrue(second["blocked"])
        second["pending"] = True
        with patch.object(app, "read_position", side_effect=lambda usb, sid: controls[sid - 1]["current"]), \
                patch.object(app, "enable_torque") as enable, patch.object(app, "set_position") as move:
            app.send_arm_target(None, controls, second, 100)
        move.assert_not_called()
        enable.assert_not_called()
        self.assertFalse(second["pending"])
        controls[2]["current"] = 2048
        app.update_joint_limits(controls)
        self.assertFalse(second["blocked"])

    def test_simultaneous_commands_keep_all_intermediate_sums_in_range(self):
        rng = random.Random(42)
        sum_min = math.ceil(305 * 4096 / 360)
        sum_max = math.floor(395 * 4096 / 360)
        for _ in range(150):
            current2 = rng.randint(1423, 2389)
            current3 = rng.randint(max(1536, sum_min - current2), min(2560, sum_max - current2))
            controls = self.controls(current2, current3)
            for control in controls[1:]:
                control.update(enabled=True, target=rng.randint(0, 4095), pending=True)
            with patch.object(app, "read_position", side_effect=lambda usb, sid: controls[sid - 1]["current"]), \
                    patch.object(app, "set_position"):
                for control in controls[1:]:
                    app.send_arm_target(None, controls, control, 100)
                    second, third = controls[1:]
                    for p2 in (second["current"], second["sent_target"]):
                        for p3 in (third["current"], third["sent_target"]):
                            self.assertGreaterEqual(p2 + p3, sum_min)
                            self.assertLessEqual(p2 + p3, sum_max)
                    self.assertGreaterEqual(third["sent_target"], 1536)
                    self.assertLessEqual(third["sent_target"], 2560)

    def test_id3_absolute_limits_intersect_relative_limits(self):
        # ID2=180度: 相対125～215度と絶対135～225度の共通範囲。
        third = self.controls()[2]
        self.assertEqual((third["base_minimum"], third["base_maximum"]), (1536, 2560))
        self.assertEqual((third["minimum"], third["maximum"]), (1536, 2446))
        self.assertEqual(app.slider_position(-100, third), 1536)
        # ID2が下限に近いときは絶対上限225度で止める。
        third = self.controls(id2=1423)[2]
        self.assertEqual((third["minimum"], third["maximum"]), (2048, 2560))
        self.assertEqual(app.slider_position(2000, third), 2560)

    def test_id3_commands_are_clamped_without_moving_id2(self):
        for id2, requested, expected in [(2048, 0, 1536), (1423, 4095, 2560)]:
            controls = self.controls(id2=id2)
            third = controls[2]
            third.update(target=requested, pending=True)
            with patch.object(app, "read_position", side_effect=lambda usb, sid: controls[sid - 1]["current"]), \
                    patch.object(app, "enable_torque"), patch.object(app, "set_position") as move:
                app.send_arm_target(None, controls, third, 100)
            move.assert_called_once_with(None, 3, expected)

    def test_send_checks_new_feedback_instead_of_old_display_range(self):
        controls = self.controls()
        second = controls[1]
        second.update(target=2389, pending=True)
        positions = {2: 2048, 3: 2500}
        with patch.object(app, "read_position", side_effect=lambda usb, sid: positions[sid]), \
                patch.object(app, "enable_torque"), patch.object(app, "set_position") as move:
            app.send_arm_target(None, controls, second, 100)
        move.assert_called_once_with(None, 2, 1994)


if __name__ == "__main__":
    unittest.main()
