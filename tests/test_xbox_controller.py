import struct
import unittest
from unittest.mock import patch

import pygame
import test as app
import xbox_controller as xbox


class FakeController:
    name = "Xbox 360 test controller"

    def __init__(self):
        self.axes = {}
        self.buttons = {}
        self.connected = True

    def attached(self):
        return self.connected

    def get_axis(self, axis):
        return self.axes.get(axis, 0)

    def get_button(self, button):
        return self.buttons.get(button, False)

    def quit(self):
        pass


class ControllerInputTests(unittest.TestCase):
    def setUp(self):
        self.device = FakeController()
        for name, value in [("init", None), ("get_count", 1), ("is_controller", True),
                            ("Controller", self.device)]:
            patch.object(xbox.controller, name, return_value=value).start()
        self.addCleanup(patch.stopall)
        self.pad = xbox.Gamepad()

    def test_deadzone_and_axis_mapping(self):
        self.assertEqual(xbox.stick_value(5000), 0)
        self.assertEqual(xbox.stick_value(-32768), -1)
        self.pad.read()
        self.device.axes = {pygame.CONTROLLER_AXIS_LEFTX: 32767,
                            pygame.CONTROLLER_AXIS_LEFTY: -32768,
                            pygame.CONTROLLER_AXIS_RIGHTY: -32768}
        self.assertEqual(self.pad.read()["axes"], {1: 1, 2: 1, 3: 1})

    def test_startup_requires_neutral(self):
        self.device.axes[pygame.CONTROLLER_AXIS_LEFTX] = 32767
        self.assertFalse(self.pad.ready)
        self.assertFalse(any(self.pad.read()["axes"].values()))
        self.device.axes.clear()
        self.pad.read()
        self.assertTrue(self.pad.ready)

    def test_a_is_triggered_once_per_press(self):
        self.pad.read()
        self.device.buttons[pygame.CONTROLLER_BUTTON_A] = True
        self.assertTrue(self.pad.read()["toggle"])
        self.assertFalse(self.pad.read()["toggle"])
        self.device.buttons.clear()
        self.pad.read()
        self.device.buttons[pygame.CONTROLLER_BUTTON_A] = True
        self.assertTrue(self.pad.read()["toggle"])

    def test_b_stops_and_requires_neutral_before_resuming(self):
        self.pad.read()
        self.device.axes[pygame.CONTROLLER_AXIS_LEFTX] = 32767
        self.device.buttons[pygame.CONTROLLER_BUTTON_B] = True
        sample = self.pad.read()
        self.assertTrue(sample["stop"])
        self.assertFalse(any(sample["axes"].values()))
        self.assertFalse(any(self.pad.read()["axes"].values()))
        self.device.buttons.clear()
        self.assertFalse(any(self.pad.read()["axes"].values()))
        self.device.axes.clear()
        self.pad.read()
        self.assertTrue(self.pad.ready)

    def test_focus_loss_and_disconnect_interrupt(self):
        self.pad.read()
        self.assertTrue(self.pad.read(focused=False)["interrupted"])
        self.assertFalse(self.pad.read(focused=False)["interrupted"])
        self.pad.read()
        self.device.connected = False
        self.assertTrue(self.pad.read()["interrupted"])
        self.device.connected = True
        self.device.axes[pygame.CONTROLLER_AXIS_LEFTX] = 32767
        self.assertFalse(any(self.pad.read()["axes"].values()))

    def test_no_controller(self):
        with patch.object(xbox.controller, "get_count", return_value=0):
            self.assertFalse(any(self.pad.read()["axes"].values()))


class ControllerMotionTests(unittest.TestCase):
    def setUp(self):
        self.controls = [{"id": sid, "current": 1000, "target": 1000,
                          "minimum": 0, "maximum": 4095, "enabled": True,
                          "pending": False, "gamepad_active": False,
                          "gamepad_target": 1000.0} for sid in [1, 2, 3]]
        self.gripper = {"enabled": False}
        self.inputs = {"axes": {1: 0, 2: 0, 3: 0}, "toggle": False,
                       "stop": False, "interrupted": False}
        self.read = patch.object(app, "read_position", return_value=1000).start()
        self.move = patch.object(app, "set_position").start()
        self.addCleanup(patch.stopall)

    def apply(self, elapsed=0.01):
        app.apply_gamepad(None, self.controls, self.gripper, self.inputs, elapsed)

    def test_one_axis_moves_relative_to_current_and_release_holds(self):
        self.inputs["axes"][2] = 1
        self.apply()
        self.assertEqual([c["target"] for c in self.controls], [1000, 1003, 1000])
        self.read.return_value = 1001
        self.inputs["axes"][2] = 0
        self.apply()
        self.move.assert_called_once_with(None, 2, 1001)
        self.assertFalse(self.controls[1]["pending"])

    def test_time_jump_and_stalled_motor_limit_target(self):
        self.inputs["axes"][1] = app.GAMEPAD_ID1_DIRECTION
        self.apply(elapsed=10)
        self.assertEqual(self.controls[0]["target"], 1015)
        for _ in range(100):
            self.apply(elapsed=0.05)
        self.assertEqual(self.controls[0]["target"], 1000 + app.GAMEPAD_LOOKAHEAD)
        self.controls[0]["maximum"] = 1005
        self.apply()
        self.assertEqual(self.controls[0]["target"], 1005)

    def test_idle_does_not_send_commands(self):
        self.apply()
        self.move.assert_not_called()
        self.read.assert_not_called()

    def test_disconnect_holds_all_axes_and_gripper(self):
        self.inputs["interrupted"] = True
        self.gripper.update(enabled=True, closing=True)
        with patch.object(app, "hold_gripper") as hold:
            self.apply()
        self.assertEqual(self.move.call_count, 3)
        hold.assert_called_once_with(None, self.gripper, "操作停止")
        self.assertFalse(any(c["pending"] for c in self.controls))

    def test_a_uses_existing_gripper_logic(self):
        self.inputs["toggle"] = True
        with patch.object(app, "toggle_gripper") as toggle:
            self.apply()
        toggle.assert_called_once_with(None, self.gripper)


class ConfiguredRangeTests(unittest.TestCase):
    def controls(self, minimum=0, maximum=4095):
        def read(usb, sid, address, size):
            return bytes([0]) if address == 33 else struct.pack("<HH", minimum, maximum)
        with patch.object(app, "read_data", side_effect=read), \
                patch.object(app, "read_position", return_value=2048):
            return app.create_controls(None)

    def test_id1_slider_is_90_to_270_degrees(self):
        controls = self.controls()
        first = controls[0]
        self.assertEqual((first["minimum"], first["maximum"]), (1024, 3072))
        self.assertEqual(app.slider_position(-100, first), 1024)
        self.assertEqual(app.slider_position(2000, first), 3072)
        self.assertEqual((controls[2]["minimum"], controls[2]["maximum"]), (1536, 2446))

    def test_id2_slider_is_125_to_210_degrees(self):
        second = self.controls()[1]
        self.assertEqual((second["minimum"], second["maximum"]), (1423, 2389))
        self.assertEqual(app.slider_position(-100, second), 1423)
        self.assertEqual(app.slider_position(2000, second), 2389)
        self.assertGreaterEqual(second["minimum"] * 360 / 4096, 125)
        self.assertLessEqual(second["maximum"] * 360 / 4096, 210)

    def test_id3_mirrored_range_calculation(self):
        minimum, maximum = app.id3_position_limits(2048)  # ID2=180度
        self.assertEqual((minimum, maximum), (1423, 2446))
        self.assertGreaterEqual(minimum * 360 / 4096, 125)
        self.assertLessEqual(maximum * 360 / 4096, 215)
        # ID2の値が増えると、両端の位置値は同じだけ減る。
        self.assertEqual(app.id3_position_limits(2148), (minimum - 100, maximum - 100))
        for id2_position in range(1423, 2390):
            lower, upper = app.id3_position_limits(id2_position)
            self.assertGreaterEqual((id2_position + lower) * 360 / 4096, 305)
            self.assertLessEqual((id2_position + upper) * 360 / 4096, 395)

    def test_hardware_limits_are_respected(self):
        first = self.controls(1500, 2800)[0]
        self.assertEqual((first["minimum"], first["maximum"]), (1500, 2800))
        with self.assertRaises(RuntimeError):
            self.controls(100, 900)

    def test_gamepad_cannot_go_past_either_endpoint(self):
        for servo_id, position, value in [(1, 1024, -1), (1, 3072, 1),
                                          (2, 1423, -1), (2, 2389, 1)]:
            controls = self.controls()
            first = controls[servo_id - 1]
            first.update(current=position, target=position, gamepad_target=float(position))
            inputs = {"axes": {servo_id: value}, "toggle": False, "stop": False, "interrupted": False}
            if servo_id == 1:
                inputs["axes"][1] *= app.GAMEPAD_ID1_DIRECTION
            with patch.object(app, "read_position", return_value=position), patch.object(app, "set_position"):
                for _ in range(10):
                    app.apply_gamepad(None, controls, {"enabled": False}, inputs, 0.05)
            self.assertEqual(first["target"], position)


if __name__ == "__main__":
    unittest.main()
