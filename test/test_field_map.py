from __future__ import division, print_function

import inspect
import itertools
import math
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission import field_map
from firefighting_mission.field_map import plan_route, point_is_free


class FieldMapTest(unittest.TestCase):
    def test_planned_route_keeps_airframe_diagonal_boundary_margin(self):
        route = plan_route(
            (0.00, 0.00), (1.50, -1.45),
            dynamic_circles=((0.70, -1.45, 0.20),))

        self.assertGreaterEqual(min(point[0] for point in route), -0.07)
        self.assertLessEqual(max(point[0] for point in route), 2.77)
        self.assertGreaterEqual(min(point[1] for point in route), -2.77)
        self.assertLessEqual(max(point[1] for point in route), 0.07)

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

    def test_task_zone_next_to_a_drawn_cylinder_stays_reachable(self):
        """Rescue zone 2 sits 0.602 m from cylinder 2 in placement 2.

        Both placements are drawn by lot, so the pair is a legal draw and the
        planner must still deliver the aircraft onto the zone instead of
        rejecting the goal.
        """
        route = plan_route(
            (0.00, 0.00), (2.70, -1.90),
            dynamic_circles=((0.70, -1.45, 0.20), (2.10, -1.95, 0.20)))

        self.assertEqual((2.70, -1.90), route[-1])

    def test_every_drawn_field_layout_plans_the_full_mission(self):
        from firefighting_mission.world_generator import (
            CYLINDER_POSES, HAZARD_POSES, PERSON_POSES)

        for first, second, hazard, person in itertools.product(
                (1, 2), (1, 2), (1, 2), (1, 2, 3)):
            circles = (CYLINDER_POSES[first][0] + (0.20,),
                       CYLINDER_POSES[second][1] + (0.20,))
            legs = [(0.0, 0.0), HAZARD_POSES[hazard],
                    PERSON_POSES[person], (0.0, 0.0)]
            for origin, destination in zip(legs, legs[1:]):
                try:
                    plan_route(origin, destination,
                               dynamic_circles=circles)
                except ValueError as error:
                    self.fail('layout %s leg %s->%s rejected: %s' % (
                        (first, second, hazard, person), origin,
                        destination, error))

    def test_goal_relief_never_cuts_inside_the_airframe_radius(self):
        route = plan_route(
            (0.00, 0.00), (2.70, -1.90),
            dynamic_circles=((2.10, -1.95, 0.20),))

        for point in route:
            self.assertGreater(
                math.hypot(point[0] - 2.10, point[1] + 1.95), 0.40, point)

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

    def test_route_centres_the_1_3_m_entry_corridor(self):
        route = plan_route((0.0, 0.0), (1.50, -1.45))

        first_transit = route[1]
        self.assertLessEqual(abs(first_transit[0]), 0.05, route)
        self.assertLess(first_transit[1], -1.05, route)

    def test_real_hazard_goal_survives_grid_rounding_near_board(self):
        route = plan_route((0.0, 0.0), (1.25, -0.10))

        self.assertEqual((1.25, -0.10), route[-1])
        self.assertTrue(all(point_is_free(point, inflation=0.45)
                            for point in route))

    def test_dynamic_circle_blocks_inflated_points(self):
        circles = ((0.70, -1.45, 0.10),)

        self.assertFalse(point_is_free(
            (0.20, -1.45), inflation=0.45,
            dynamic_circles=circles))
        self.assertTrue(point_is_free(
            (0.00, -2.40), inflation=0.45,
            dynamic_circles=circles))

    def test_route_avoids_dynamic_circle(self):
        circle = (0.70, -1.45, 0.10)

        route = plan_route(
            (0.00, -1.90), (1.50, -1.45),
            dynamic_circles=(circle,))

        for first, second in zip(route, route[1:]):
            for index in range(21):
                ratio = index / 20.0
                x_value = first[0] + (second[0] - first[0]) * ratio
                y_value = first[1] + (second[1] - first[1]) * ratio
                self.assertGreater(
                    math.hypot(x_value - circle[0],
                               y_value - circle[1]),
                    circle[2] + 0.44)

    def test_dynamic_route_uses_safe_visibility_shortcuts(self):
        circles = ((0.70, -1.45, 0.10),
                   (2.10, -1.95, 0.10))

        route = plan_route(
            (0.00, -1.00), (1.50, -1.45),
            dynamic_circles=circles)

        self.assertLessEqual(len(route), 7, route)
        for first, second in zip(route, route[1:]):
            distance = math.hypot(second[0] - first[0],
                                  second[1] - first[1])
            sample_count = max(1, int(math.ceil(distance / 0.025)))
            for index in range(sample_count + 1):
                ratio = index / float(sample_count)
                point = (first[0] + (second[0] - first[0]) * ratio,
                         first[1] + (second[1] - first[1]) * ratio)
                self.assertTrue(point_is_free(
                    point, inflation=0.45,
                    dynamic_circles=circles), (first, second, point))

    def test_route_can_escape_dynamic_inflation_at_current_pose(self):
        circle = (0.48, -1.41, 0.15)

        route = plan_route(
            (0.21, -1.93), (1.50, -1.45),
            dynamic_circles=(circle,))

        self.assertEqual((0.21, -1.93), route[0])
        self.assertEqual((1.50, -1.45), route[-1])
        self.assertGreater(len(route), 2)


if __name__ == '__main__':
    unittest.main()
