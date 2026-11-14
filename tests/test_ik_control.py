import math
import struct
import unittest
from unittest.mock import patch

import ik_control as app
import test as hardware
from inverse_kinematics import forward_kinematics


class SimulatedArm:
    def __init__(self):
        self.positions = {1: 2048, 2: 2048, 3: 2048, 4: 2300}
        self.targets = self.positions.copy()
        self.torque = {sid: 0 for sid in self.positions}
        self.commands = []
        self.instant = True

    def read(self, usb, sid, address, size):
        if address == 9:
            return struct.pack("<HH", 0, 4095)
        if address == 33:
            return bytes([0])
        if address == 40:
            return bytes([self.torque[sid]])
        if address == 56:
            return (struct.pack("<HHH", self.positions[sid], 0, 0) if size == 6
                    else struct.pack("<H", self.positions[sid]))
        raise AssertionError((sid, address, size))

    def write(self, usb, sid, address, data):
        self.commands.append((sid, address, bytes(data)))
        if address == 41:
            self.targets[sid] = int.from_bytes(bytes(data[1:3]), "little")
        if address == 40:
            self.torque[sid] = data[0]
        if self.instant and self.torque[sid]:
            self.positions[sid] = self.targets[sid]


class IKControlTests(unittest.TestCase):
    def setUp(self):
        self.arm = SimulatedArm()
        patch.object(hardware, "read_data", side_effect=self.arm.read).start()
        patch.object(hardware, "write_data", side_effect=self.arm.write).start()
        self.addCleanup(patch.stopall)
        self.controls = hardware.create_controls(None)
        self.gripper = hardware.create_gripper(None)
        self.state = app.make_state(self.controls)
        self.inputs = {"axes": {1: 0, 2: 0, 3: 0}, "toggle": False,
                       "stop": False, "interrupted": False}

    def test_startup_does_not_move(self):
        app.update_ik(None, self.controls, self.gripper, self.state, 100)
        self.assertFalse(self.arm.commands)
        self.assertFalse(self.state["moving"])
        self.assertAlmostEqual(self.state["current"][0], 120 * math.sqrt(2))

    def test_reachable_target_moves_in_small_steps_and_arrives(self):
        goal = (155, 20, 12)
        self.assertTrue(app.request_target(goal, self.controls, self.state, 100))
        previous = self.state["current"]
        for tick in range(400):
            app.update_ik(None, self.controls, self.gripper, self.state, 100 + tick * 0.06)
            positions = tuple(self.arm.positions[sid] for sid in (1, 2, 3))
            point = forward_kinematics(positions, app.GEOMETRY)
            self.assertLessEqual(math.dist(point, previous), 2.0)
            self.assertTrue(self.state["limits"].allows(positions))
            previous = point
            if not self.state["moving"]:
                break
        self.assertFalse(self.state["moving"])
        self.assertLessEqual(math.dist(self.state["current"], goal), app.ARRIVAL_TOLERANCE_MM)
        self.assertTrue(all(self.arm.torque[sid] == 1 for sid in (1, 2, 3)))
        self.assertFalse(any(sid == 4 for sid, address, data in self.arm.commands))

    def test_invalid_target_does_not_issue_a_motor_command(self):
        original = self.state["goal"]
        self.assertFalse(app.request_target((300, 0, 0), self.controls, self.state, 100))
        self.assertEqual(self.state["goal"], original)
        self.assertFalse(self.arm.commands)

    def test_invalid_current_pose_is_not_automatically_corrected(self):
        self.controls[2]["current"] = 3000
        self.assertFalse(app.request_target((170, 0, 0), self.controls, self.state, 100))
        self.assertFalse(self.arm.commands)

    def test_unreachable_intermediate_step_stops(self):
        app.request_target((160, 0, 0), self.controls, self.state, 100)
        with patch.object(app, "inverse_kinematics", return_value=[]):
            app.update_ik(None, self.controls, self.gripper, self.state, 101)
        self.assertFalse(self.state["moving"])
        self.assertFalse(self.arm.commands)

    def test_stalled_motion_times_out_and_holds(self):
        self.arm.instant = False
        app.request_target((155, 0, 0), self.controls, self.state, 100)
        app.update_ik(None, self.controls, self.gripper, self.state, 100.1)
        app.update_ik(None, self.controls, self.gripper, self.state, 131)
        self.assertFalse(self.state["moving"])
        for sid in (1, 2, 3):
            self.assertEqual(self.arm.targets[sid], self.arm.positions[sid])

    def test_gamepad_xyz_mapping_and_release_holds(self):
        self.inputs["axes"] = {1: 0.5, 2: 0.5, 3: 0.5}
        original = self.state["current"]
        app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 0.02, 100)
        for index in range(3):
            self.assertAlmostEqual(self.state["goal"][index] - original[index], 0.3)
        self.inputs["axes"] = {}
        app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 0.02, 100.1)
        self.assertFalse(self.state["moving"])
        self.assertFalse(self.state["gamepad_active"])

    def test_small_stick_input_accumulates_instead_of_sticking(self):
        self.inputs["axes"][1] = 0.1
        for tick in range(120):
            now = 100 + tick / 60
            app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 1 / 60, now)
            app.update_ik(None, self.controls, self.gripper, self.state, now)
        self.assertLess(self.arm.positions[1], 2048)

    def test_configured_yaw_is_reversed_for_both_fk_and_ik(self):
        self.assertEqual(app.GEOMETRY.yaw_direction, -1)
        point = forward_kinematics((2058, 2048, 2048), app.GEOMETRY)
        self.assertLess(point[1], 0)
        solutions = app.inverse_kinematics((169, 10, 0), app.GEOMETRY, self.state["limits"], (2048, 2048, 2048))
        self.assertTrue(solutions)
        self.assertLess(solutions[0].positions[0], 2048)

    def test_gamepad_lead_is_bounded_when_motor_stalls(self):
        self.inputs["axes"][2] = 1
        for tick in range(100):
            app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 10, 100 + tick)
        self.assertLessEqual(math.dist(self.state["goal"], self.state["current"]), 3.0000001)

    def test_stop_and_disconnect_cancel_arm_and_hand_motion(self):
        for field in ("stop", "interrupted"):
            self.inputs[field] = True
            self.gripper.update(enabled=True, closing=True)
            app.request_target((160, 0, 0), self.controls, self.state, 100)
            app.update_ik(None, self.controls, self.gripper, self.state, 100.1)
            app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 0.02, 100.2)
            self.assertFalse(self.state["moving"])
            self.assertFalse(self.gripper["closing"])
            self.assertFalse(any(address == 40 and data == bytes([0])
                                 for sid, address, data in self.arm.commands))
            self.inputs[field] = False

    def test_a_uses_existing_load_monitored_gripper(self):
        self.inputs["toggle"] = True
        app.apply_gamepad(None, self.controls, self.gripper, self.state, self.inputs, 0.02, 100)
        self.assertTrue(self.gripper["closing"])
        self.assertFalse(self.gripper["next_closed"])
        self.assertFalse(any(sid in (1, 2, 3) for sid, address, data in self.arm.commands))


if __name__ == "__main__":
    unittest.main()
