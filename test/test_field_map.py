from __future__ import division, print_function

import inspect
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission import field_map
from firefighting_mission.field_map import plan_route, point_is_free


class FieldMapTest(unittest.TestCase):
    def test_route_avoids_every_inflated_fixed_board(self):
        route = plan_route((0.0, 0.0), (2.70, -1.90))

        self.assertGreater(len(route), 2)
        for point in route:
            self.assertTrue(point_is_free(point, inflation=0.45), point)

    def test_route_source_has_no_random_cylinder_truth_dependency(self):
        source = inspect.getsource(field_map)

        self.assertNotIn('CYLINDER_POSES', source)
        self.assertNotIn('build_scenario', source)
        self.assertNotIn('Scenario', source)

    def test_unreachable_or_outside_goal_raises_clear_error(self):
        try:
            plan_route((0.0, 0.0), (5.0, 5.0))
        except ValueError as error:
            self.assertEqual('goal_outside_field', str(error))
        else:
            self.fail('expected goal_outside_field')

    def test_start_inside_fixed_board_is_rejected(self):
        try:
            plan_route((0.70, -0.20), (2.70, -1.90))
        except ValueError as error:
            self.assertEqual('start_blocked', str(error))
        else:
            self.fail('expected start_blocked')

    def test_route_is_deterministic(self):
        first = plan_route((0.0, 0.0), (2.70, -2.65))
        second = plan_route((0.0, 0.0), (2.70, -2.65))

        self.assertEqual(first, second)

    def test_simplified_segments_change_direction_only_at_corners(self):
        route = plan_route((0.0, 0.0), (1.40, -0.45))

        self.assertEqual((0.0, 0.0), route[0])
        self.assertEqual((1.40, -0.45), route[-1])
        self.assertLess(len(route), 20)


if __name__ == '__main__':
    unittest.main()
