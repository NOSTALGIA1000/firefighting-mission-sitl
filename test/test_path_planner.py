from __future__ import division, print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.path_planner import (
    VisualPathPlanner, VisualPlannerConfig, corridor_clearances, ramp_setpoint)
from firefighting_mission.stereo_obstacles import ObstacleClusterData


POSE = (0.0, 0.0, 1.2, 0.0)
OBSTACLE = ObstacleClusterData(0.80, 0.0, 0.75, 0.10, -0.10, 1.0)


def straight_route(start, goal):
    return (tuple(start), tuple(goal))


def planner_with_goal(config=None):
    planner = VisualPathPlanner(
        config=config, route_provider=straight_route,
        path_validator=lambda start, side, passing, rejoin: True)
    planner.set_goal((2.0, 0.0, 1.2), POSE)
    return planner


def drive_to_select(planner, pose=POSE, obstacle=OBSTACLE):
    planner.update(pose, (obstacle,), True, 1.0)
    planner.update(pose, (obstacle,), True, 1.1)
    planner.update(pose, (obstacle,), True, 1.2)
    planner.update(pose, (obstacle,), True, 1.3)
    return planner.update(pose, (obstacle,), True, 1.4)


class VisualPathPlannerTest(unittest.TestCase):
    def test_follow_route_never_commands_23_metres(self):
        planner = planner_with_goal()

        command = planner.update(POSE, (), True, 0.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertEqual(1.2, command.target[2])
        self.assertNotEqual(2.3, command.target[2])

    def test_obstacle_runs_brake_observe_and_select_sequence(self):
        planner = planner_with_goal()

        states = [planner.update(POSE, (OBSTACLE,), True, 1.0).state,
                  planner.update(POSE, (OBSTACLE,), True, 1.1).state,
                  planner.update(POSE, (OBSTACLE,), True, 1.2).state,
                  planner.update(POSE, (OBSTACLE,), True, 1.3).state,
                  planner.update(POSE, (OBSTACLE,), True, 1.4).state]

        self.assertEqual(('BRAKE', 'OBSERVE', 'OBSERVE', 'OBSERVE',
                          'SELECT_SIDE'), tuple(states))

    def test_selects_only_valid_right_corridor(self):
        planner = planner_with_goal(VisualPlannerConfig(minimum_corridor=0.90))
        planner.clearance_override = (0.60, 1.30)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('RIGHT', command.selected_side)
        self.assertEqual('SIDESTEP', command.state)

    def test_selects_wider_left_corridor(self):
        planner = planner_with_goal()
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('LEFT', command.selected_side)

    def test_side_hysteresis_keeps_previous_safe_side_on_near_tie(self):
        planner = planner_with_goal()
        planner.last_selected_side = 'RIGHT'
        planner.clearance_override = (1.10, 1.05)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('RIGHT', command.selected_side)

    def test_neither_corridor_holds_position(self):
        planner = planner_with_goal()
        planner.clearance_override = (0.70, 0.80)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('no_safe_corridor', command.reason)
        self.assertEqual(POSE[:3], command.target)

    def test_hold_unsafe_recovers_when_obstacle_leaves_view(self):
        planner = planner_with_goal()
        planner.clearance_override = (0.70, 0.80)
        drive_to_select(planner)
        planner.update(POSE, (OBSTACLE,), True, 1.5)

        command = planner.update(POSE, (), True, 1.6)

        self.assertEqual('FOLLOW_ROUTE', command.state)

    def test_visual_loss_holds_current_position(self):
        command = planner_with_goal().update(POSE, (), False, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual(POSE[:3], command.target)
        self.assertEqual('perception_not_ready', command.reason)

    def test_altitude_error_holds_xy_and_recovers_12(self):
        command = planner_with_goal().update(
            (0.3, -0.2, 1.36, 0.0), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual((0.3, -0.2, 1.2), command.target)
        self.assertEqual('altitude_out_of_band', command.reason)

    def test_full_avoidance_sequence_rejoins_route(self):
        planner = planner_with_goal()
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)
        sidestep = planner.update(POSE, (OBSTACLE,), True, 1.5)
        at_side = sidestep.target + (sidestep.target_yaw,)
        passing = planner.update(at_side, (OBSTACLE,), True, 1.6)
        at_pass = passing.target + (passing.target_yaw,)
        rejoin = planner.update(at_pass, (), True, 1.7)
        at_rejoin = rejoin.target + (rejoin.target_yaw,)
        following = planner.update(at_rejoin, (), True, 1.8)

        self.assertEqual('SIDESTEP', sidestep.state)
        self.assertEqual('PASS', passing.state)
        self.assertEqual('REJOIN', rejoin.state)
        self.assertEqual('FOLLOW_ROUTE', following.state)
        for command in (sidestep, passing, rejoin, following):
            self.assertAlmostEqual(1.2, command.target[2], places=6)

    def test_goal_arrival_reports_reached_at_12(self):
        command = planner_with_goal().update(
            (2.0, 0.0, 1.2, 0.0), (), True, 3.0)

        self.assertEqual('REACHED', command.state)
        self.assertEqual((2.0, 0.0, 1.2), command.target)

    def test_goal_arrival_wins_over_unrelated_visible_obstacle(self):
        planner = VisualPathPlanner(route_provider=straight_route)
        planner.set_goal((0.0, 0.0, 1.2), POSE)

        command = planner.update(POSE, (OBSTACLE,), True, 3.0)

        self.assertEqual('REACHED', command.state)

    def test_visual_avoidance_waits_until_camera_faces_route(self):
        def turning_route(start, goal):
            return (tuple(start), (0.0, -0.5), tuple(goal))
        planner = VisualPathPlanner(route_provider=turning_route)
        planner.set_goal((1.0, -0.5, 1.2), POSE)

        command = planner.update(POSE, (OBSTACLE,), True, 3.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertAlmostEqual(-1.5708, command.target_yaw, places=3)

    def test_setpoint_ramp_limits_translation_lead_and_yaw_rate(self):
        output = ramp_setpoint(
            last=(0.0, 0.0, 1.2, 0.0),
            desired=(1.0, -1.0, 1.2, -1.5708),
            current=(0.0, 0.0, 1.2, 0.0), dt=0.10)

        self.assertLessEqual((output[0] ** 2 + output[1] ** 2) ** 0.5,
                             0.030001)
        self.assertAlmostEqual(-0.06, output[3], places=6)

    def test_fixed_wall_blocks_side_selection_that_would_cross_it(self):
        planner = VisualPathPlanner(route_provider=straight_route)
        planner.set_goal((2.0, 0.0, 1.2), POSE)
        planner.clearance_override = (1.30, 1.30)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('no_safe_corridor', command.reason)

    def test_corridor_measurement_uses_field_boundary(self):
        left, right = corridor_clearances(
            POSE, OBSTACLE, sample_step=0.05)

        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertNotEqual(left, right)


if __name__ == '__main__':
    unittest.main()
