from __future__ import print_function

import math
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.navigation import (NavigationConfig, Navigator,
                                              sector_distances)


class NavigationTest(unittest.TestCase):
    def setUp(self):
        self.navigator = Navigator(NavigationConfig())

    def test_clear_path_uses_bounded_proportional_guidance(self):
        command = self.navigator.compute(goal=(2.0, 0.0, 1.3),
                                         pose=(0.0, 0.0, 1.0),
                                         yaw=0.0,
                                         sectors=(3.0, 3.0, 3.0))

        self.assertAlmostEqual(0.55, command.x)
        self.assertAlmostEqual(0.0, command.y)
        self.assertAlmostEqual(0.24, command.z)

    def test_turns_toward_clearer_left_sector(self):
        command = self.navigator.compute(goal=(2.0, 0.0, 1.3),
                                         pose=(0.0, 0.0, 1.3),
                                         yaw=0.0,
                                         sectors=(0.50, 1.40, 0.60))

        self.assertGreater(command.y, 0.0)
        self.assertLessEqual(command.x, 0.08)

    def test_turns_toward_clearer_right_sector(self):
        command = self.navigator.compute(goal=(2.0, 0.0, 1.3),
                                         pose=(0.0, 0.0, 1.3),
                                         yaw=0.0,
                                         sectors=(0.50, 0.55, 1.30))

        self.assertLess(command.y, 0.0)

    def test_emergency_distance_commands_reverse(self):
        command = self.navigator.compute(goal=(2.0, 0.0, 1.3),
                                         pose=(0.0, 0.0, 1.3),
                                         yaw=0.0,
                                         sectors=(0.29, 0.8, 0.8))

        self.assertLess(command.x, 0.0)
        self.assertEqual('RETREAT', command.status)

    def test_goal_inside_tolerance_is_reached(self):
        command = self.navigator.compute(goal=(0.05, 0.04, 1.31),
                                         pose=(0.0, 0.0, 1.30),
                                         yaw=0.0,
                                         sectors=(3.0, 3.0, 3.0))

        self.assertEqual('REACHED', command.status)
        self.assertEqual((0.0, 0.0, 0.0), (command.x, command.y, command.z))

    def test_scan_is_split_into_front_left_and_right_sectors(self):
        ranges = [5.0] * 360
        ranges[180] = 0.45
        ranges[225] = 0.65
        ranges[135] = 0.75

        front, left, right = sector_distances(
            ranges, angle_min=-math.pi, angle_increment=math.pi / 180.0)

        self.assertAlmostEqual(0.45, front)
        self.assertAlmostEqual(0.65, left)
        self.assertAlmostEqual(0.75, right)


if __name__ == '__main__':
    unittest.main()
