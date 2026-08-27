from __future__ import division, print_function

import math
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


def planner_with_goal(config=None, dynamic_route_provider=None):
    config = config or VisualPlannerConfig()
    config.known_static_tolerance = -1.0
    kwargs = {
        'config': config,
        'route_provider': straight_route,
        'path_validator': lambda start, side, passing, rejoin: True,
    }
    if dynamic_route_provider is not None:
        kwargs['dynamic_route_provider'] = dynamic_route_provider
    planner = VisualPathPlanner(**kwargs)
    planner.set_goal((2.0, 0.0, 1.2), POSE)
    return planner


def drive_to_select(planner, pose=POSE, obstacle=OBSTACLE):
    planner.update(pose, (obstacle,), True, 1.0)
    planner.update(pose, (obstacle,), True, 1.1)
    planner.update(pose, (obstacle,), True, 1.2)
    planner.update(pose, (obstacle,), True, 1.3)
    return planner.update(pose, (obstacle,), True, 1.4)


def drive_through_rejoin(planner, obstacle=OBSTACLE):
    drive_to_select(planner, obstacle=obstacle)
    sidestep = planner.update(POSE, (obstacle,), True, 1.5)
    passing = planner.update(
        sidestep.target + (sidestep.target_yaw,),
        (obstacle,), True, 1.6)
    rejoin = planner.update(
        passing.target + (passing.target_yaw,), (), True, 1.7)
    return planner.update(
        rejoin.target + (rejoin.target_yaw,), (), True, 1.8)


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

    def test_obstacle_outside_flight_corridor_does_not_trigger(self):
        planner = planner_with_goal()
        side_obstacle = OBSTACLE._replace(left_m=0.70)

        command = planner.update(POSE, (side_obstacle,), True, 1.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)

    def test_known_safety_net_seen_by_forward_camera_is_not_replanned(self):
        pose = (0.20, -1.90, 1.20, math.pi)
        safety_net = ObstacleClusterData(
            0.55, 0.0, 0.53, 0.20, -0.20, 1.0)
        planner = VisualPathPlanner(
            VisualPlannerConfig(sensor_forward_offset=0.32),
            route_provider=straight_route)
        planner.set_goal((-0.40, -1.90, 1.20), pose)

        command = planner.update(pose, (safety_net,), True, 1.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)

    def test_known_fixed_board_is_left_to_global_route(self):
        pose = (0.0, -0.20, 1.20, 0.0)
        fixed_board = ObstacleClusterData(
            0.65, 0.0, 0.63, 0.05, -0.05, 1.0)
        planner = VisualPathPlanner(
            VisualPlannerConfig(), route_provider=straight_route,
            path_validator=lambda start, side, passing, rejoin: True)
        planner.set_goal((1.50, -0.20, 1.20), pose)

        command = planner.update(pose, (fixed_board,), True, 1.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)

    def test_unknown_cylinder_still_triggers_after_camera_offset_transform(self):
        pose = (0.0, -1.45, 1.20, 0.0)
        cylinder = ObstacleClusterData(
            0.30, 0.0, 0.28, 0.10, -0.10, 1.0)
        planner = VisualPathPlanner(
            VisualPlannerConfig(sensor_forward_offset=0.32),
            route_provider=straight_route)
        planner.set_goal((1.50, -1.45, 1.20), pose)

        command = planner.update(pose, (cylinder,), True, 1.0)

        self.assertEqual('BRAKE', command.state)

    def test_selects_only_valid_right_corridor(self):
        planner = planner_with_goal(VisualPlannerConfig(minimum_corridor=0.90))
        planner.clearance_override = (0.60, 1.30)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('RIGHT', command.selected_side)
        self.assertEqual('SIDESTEP', command.state)

    def test_right_sidestep_uses_signed_obstacle_edge(self):
        planner = planner_with_goal(VisualPlannerConfig(minimum_corridor=0.90))
        planner.clearance_override = (0.60, 1.30)
        obstacle_left_of_camera = OBSTACLE._replace(
            left_m=0.30, left_edge_m=0.35, right_edge_m=0.25)
        drive_to_select(planner, obstacle=obstacle_left_of_camera)

        command = planner.update(
            POSE, (obstacle_left_of_camera,), True, 1.5)

        self.assertEqual('RIGHT', command.selected_side)
        self.assertAlmostEqual(-0.20, command.target[1], places=6)

    def test_selected_cluster_is_remembered_as_world_circle(self):
        planner = planner_with_goal()
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)

        planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual(1, len(planner.temporary_obstacles))
        circle = planner.temporary_obstacles[0]
        self.assertAlmostEqual(0.80, circle[0], places=6)
        self.assertAlmostEqual(0.00, circle[1], places=6)
        self.assertAlmostEqual(0.10, circle[2], places=6)

    def test_duplicate_visual_observations_merge(self):
        planner = planner_with_goal()

        planner._remember_obstacle((0.70, -1.45, 0.10))
        planner._remember_obstacle((0.74, -1.43, 0.11))

        self.assertEqual(1, len(planner.temporary_obstacles))
        self.assertAlmostEqual(0.11, planner.temporary_obstacles[0][2])

    def test_rejoin_replans_with_temporary_obstacles(self):
        calls = []

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner.clearance_override = (1.30, 1.00)

        command = drive_through_rejoin(planner)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertEqual(1, len(calls))
        self.assertEqual(planner.temporary_obstacles, calls[0][2])

    def test_stable_far_cylinder_replans_before_local_trigger(self):
        calls = []
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.0, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)

        first = planner.update(POSE, (far_obstacle,), True, 1.0)
        self.assertEqual(0, len(calls))
        second = planner.update(POSE, (far_obstacle,), True, 1.1)

        self.assertEqual('FOLLOW_ROUTE', first.state)
        self.assertEqual('FOLLOW_ROUTE', second.state)
        self.assertEqual((0.0, 0.50, 1.2), second.target)
        self.assertEqual(1, len(calls))
        self.assertEqual(1, len(planner.temporary_obstacles))
        self.assertAlmostEqual(1.40, planner.temporary_obstacles[0][0])

    def test_far_cylinder_memory_uses_physical_radius_not_projected_width(self):
        calls = []
        wide_far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35,
            left_edge_m=1.20, right_edge_m=-1.20)

        def dynamic_route(start, goal, circles):
            calls.append(tuple(circles))
            return (tuple(start), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)

        planner.update(POSE, (wide_far_obstacle,), True, 1.0)
        planner.update(POSE, (wide_far_obstacle,), True, 1.1)

        self.assertEqual(1, len(calls))
        self.assertAlmostEqual(0.10, calls[0][0][2])

    def test_dynamic_replan_failure_latches_hold_instead_of_resuming_far_route(self):
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)

        def unreachable(_start, _goal, _circles):
            raise ValueError('route_unreachable')

        planner = planner_with_goal(dynamic_route_provider=unreachable)

        planner.update(POSE, (far_obstacle,), True, 1.0)
        failed = planner.update(POSE, (far_obstacle,), True, 1.1)
        following = planner.update(POSE, (far_obstacle,), True, 1.2)

        self.assertEqual('HOLD_UNSAFE', failed.state)
        self.assertEqual('HOLD_UNSAFE', following.state)
        self.assertEqual('dynamic_route_unreachable', following.reason)

    def test_dynamic_replan_failure_holds(self):
        def unreachable(_start, _goal, _circles):
            raise ValueError('route_unreachable')

        planner = planner_with_goal(dynamic_route_provider=unreachable)
        planner.clearance_override = (1.30, 1.00)

        command = drive_through_rejoin(planner)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('dynamic_route_unreachable', command.reason)

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

    def test_hold_unsafe_does_not_resume_when_obstacle_leaves_view(self):
        planner = planner_with_goal()
        planner.clearance_override = (0.70, 0.80)
        drive_to_select(planner)
        planner.update(POSE, (OBSTACLE,), True, 1.5)

        command = planner.update(POSE, (), True, 1.6)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('obstacle_temporarily_unseen', command.reason)

    def test_visual_loss_holds_current_position(self):
        command = planner_with_goal().update(POSE, (), False, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual(POSE[:3], command.target)
        self.assertEqual('perception_not_ready', command.reason)

    def test_visual_loss_latches_first_hold_position(self):
        planner = planner_with_goal()
        planner.update(POSE, (), False, 2.0)

        command = planner.update((0.2, -0.3, 1.2, 0.0), (), False, 2.1)

        self.assertEqual(POSE[:3], command.target)

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
        self.assertGreaterEqual(
            passing.target[0], OBSTACLE.forward_m + 0.80)
        for command in (sidestep, passing, rejoin, following):
            self.assertAlmostEqual(1.2, command.target[2], places=6)

    def test_rejoin_skips_route_waypoints_left_behind(self):
        def staged_route(start, goal):
            return (tuple(start), (0.5, 0.0), (1.0, 0.0), tuple(goal))

        config = VisualPlannerConfig()
        config.known_static_tolerance = -1.0
        planner = VisualPathPlanner(
            config=config, route_provider=staged_route,
            path_validator=lambda start, side, passing, rejoin: True)
        planner.set_goal((2.0, 0.0, 1.2), POSE)
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)
        sidestep = planner.update(POSE, (OBSTACLE,), True, 1.5)
        passing = planner.update(
            sidestep.target + (sidestep.target_yaw,),
            (OBSTACLE,), True, 1.6)
        rejoin = planner.update(
            passing.target + (passing.target_yaw,), (), True, 1.7)

        following = planner.update(
            rejoin.target + (rejoin.target_yaw,), (), True, 1.8)

        self.assertEqual('FOLLOW_ROUTE', following.state)
        self.assertEqual((2.0, 0.0, 1.2), following.target)

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
        self.assertEqual(POSE[:3], command.target)
        self.assertAlmostEqual(-1.5708, command.target_yaw, places=3)

    def test_route_yaw_alignment_latches_first_xy(self):
        def turning_route(start, goal):
            return (tuple(start), (0.0, -0.5), tuple(goal))
        planner = VisualPathPlanner(route_provider=turning_route)
        planner.set_goal((1.0, -0.5, 1.2), POSE)
        planner.update(POSE, (), True, 3.0)

        command = planner.update((0.2, -0.3, 1.2, -0.1), (), True, 3.1)

        self.assertEqual(POSE[:3], command.target)

    def test_brake_and_observe_latch_detection_xy(self):
        planner = planner_with_goal()
        brake = planner.update(POSE, (OBSTACLE,), True, 1.0)

        observe = planner.update(
            (0.2, -0.3, 1.2, 0.0), (OBSTACLE,), True, 1.1)

        self.assertEqual(POSE[:3], brake.target)
        self.assertEqual(POSE[:3], observe.target)

    def test_setpoint_ramp_limits_translation_lead_and_yaw_rate(self):
        output = ramp_setpoint(
            last=(0.0, 0.0, 1.2, 0.0),
            desired=(1.0, -1.0, 1.2, -1.5708),
            current=(0.0, 0.0, 1.2, 0.0), dt=0.10)

        self.assertLessEqual((output[0] ** 2 + output[1] ** 2) ** 0.5,
                             0.030001)
        self.assertAlmostEqual(-0.06, output[3], places=6)

    def test_setpoint_ramp_can_keep_latched_hold_during_pose_drift(self):
        output = ramp_setpoint(
            last=(0.0, 0.0, 1.2, 0.0),
            desired=(0.0, 0.0, 1.2, -1.0),
            current=(0.3, -0.4, 1.2, -0.1), dt=0.10,
            maximum_lead=None)

        self.assertEqual((0.0, 0.0), output[:2])

    def test_fixed_wall_blocks_side_selection_that_would_cross_it(self):
        planner = VisualPathPlanner(
            VisualPlannerConfig(known_static_tolerance=-1.0),
            route_provider=straight_route)
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
