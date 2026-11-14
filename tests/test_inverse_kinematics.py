import math
import random
import unittest

from inverse_kinematics import ArmGeometry, ServoLimits, forward_kinematics, inverse_kinematics


class InverseKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.geometry = ArmGeometry(120, 120, 45, -45, 1, -1)
        self.limits = ServoLimits(((90, 270), (125, 210), (135, 225)), (305, 395))

    def test_reference_pose(self):
        point = forward_kinematics((2048, 2048, 2048), self.geometry)
        self.assertAlmostEqual(point[0], 120 * math.sqrt(2))
        self.assertAlmostEqual(point[1], 0)
        self.assertAlmostEqual(point[2], 0)
        solutions = inverse_kinematics(point, self.geometry, self.limits, (2048, 2048, 2048))
        self.assertEqual(solutions[0].positions, (2048, 2048, 2048))

    def test_calibrated_directions(self):
        self.assertGreater(forward_kinematics((2048, 2058, 2048), self.geometry)[2], 0)
        self.assertLess(forward_kinematics((2048, 2048, 2058), self.geometry)[2], 0)
        self.assertGreater(forward_kinematics((2058, 2048, 2048), self.geometry)[1], 0)

    def test_forward_inverse_round_trip_through_allowed_workspace(self):
        rng = random.Random(27)
        for _ in range(350):
            second = rng.randint(1423, 2389)
            third = rng.randint(max(1536, 3471 - second), min(2560, 4494 - second))
            original = (rng.randint(1024, 3072), second, third)
            xyz = forward_kinematics(original, self.geometry)
            solutions = inverse_kinematics(xyz, self.geometry, self.limits, original)
            self.assertTrue(solutions, original)
            self.assertEqual(solutions[0].positions, original)
            self.assertLess(solutions[0].error_mm, 1e-8)

    def test_quantized_solutions_respect_all_limits(self):
        for xyz in [(169.7, 1, 1), (160, -15, 10), (150, 30, -10)]:
            solutions = inverse_kinematics(xyz, self.geometry, self.limits)
            self.assertTrue(solutions)
            for solution in solutions:
                self.assertTrue(self.limits.allows(solution.positions))
                self.assertLess(solution.error_mm, 0.5)
                self.assertEqual(forward_kinematics(solution.positions, self.geometry), solution.xyz_mm)

    def test_absolute_and_relative_outside_targets_are_rejected(self):
        for positions in [(2048, 1300, 2048), (2048, 2048, 2700), (2048, 2389, 2560)]:
            xyz = forward_kinematics(positions, self.geometry)
            self.assertFalse(inverse_kinematics(xyz, self.geometry, self.limits))

    def test_unreachable_and_folded_origin_are_rejected(self):
        for point in [(241, 0, 0), (500, 500, 500), (0, 0, 0), (-170, 0, 0)]:
            self.assertFalse(inverse_kinematics(point, self.geometry, self.limits))

    def test_invalid_geometry_and_coordinates(self):
        for arguments in [(0, 120, 45, -45, 1, -1), (120, 120, float('nan'), -45, 1, -1),
                          (120, 120, 45, -45, 0, -1)]:
            with self.assertRaises(ValueError):
                ArmGeometry(*arguments)
        with self.assertRaises(ValueError):
            inverse_kinematics((float('nan'), 0, 0), self.geometry, self.limits)


if __name__ == "__main__":
    unittest.main()
