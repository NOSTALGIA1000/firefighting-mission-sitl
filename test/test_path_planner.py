from __future__ import division, print_function

import math
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.path_planner import (
    VisualPathPlanner, VisualPlannerConfig, corridor_clearances,
    map_target_to_local, ramp_setpoint)
from firefighting_mission.stereo_obstacles import ObstacleClusterData


POSE = (0.0, 0.0, 1.2, 0.0)
OBSTACLE = ObstacleClusterData(0.80, 0.0, 0.75, 0.10, -0.10, 1.0)


def straight_route(start, goal):
    return (tuple(start), tuple(goal))


def planner_with_goal(config=None, dynamic_route_provider=None):
    config = config or VisualPlannerConfig(geofence_warning_margin=-10.0)
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
    def test_map_target_is_shifted_into_current_px4_local_frame(self):
        target = (0.0, -1.0, 1.2, -1.0)
        map_pose = (0.35, -0.44, 1.6, 0.2)
        local_pose = (-0.10, -0.05, 1.37, 0.1)

        converted = map_target_to_local(target, map_pose, local_pose)

        self.assertAlmostEqual(-0.45, converted[0], places=6)
        self.assertAlmostEqual(-0.61, converted[1], places=6)
        self.assertAlmostEqual(1.20, converted[2], places=6)
        self.assertAlmostEqual(-1.10, converted[3], places=6)

    def test_map_target_xy_stays_on_fixed_world_axes_during_yaw(self):
        target = (1.0, 0.0, 1.2, 0.0)
        map_pose = (0.0, 0.0, 1.2, 0.0)
        local_pose = (10.0, 20.0, 1.2, math.pi / 2.0)

        converted = map_target_to_local(target, map_pose, local_pose)

        self.assertAlmostEqual(11.0, converted[0], places=6)
        self.assertAlmostEqual(20.0, converted[1], places=6)
        self.assertAlmostEqual(math.pi / 2.0, converted[3], places=6)

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

    def test_visible_side_cylinder_is_used_for_global_replan(self):
        calls = []
        side_cylinder = OBSTACLE._replace(
            forward_m=1.05, left_m=-0.70, nearest_range_m=0.95,
            left_edge_m=-0.60, right_edge_m=-0.80)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.0, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)

        first = planner.update(POSE, (side_cylinder,), True, 1.0)
        second = planner.update(POSE, (side_cylinder,), True, 1.1)

        self.assertEqual('FOLLOW_ROUTE', first.state)
        self.assertEqual('FOLLOW_ROUTE', second.state)
        self.assertEqual('dynamic_route_replanned', second.reason)
        self.assertEqual(1, len(calls))
        self.assertEqual(1, len(planner.temporary_obstacles))

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
        """Local selection must remember the same circle model as replanning.

        The cluster reports the near surface of a 200 mm cylinder, so the
        centre lies one cylinder radius further along the sight line and the
        stored radius carries the localisation margin.
        """
        planner = planner_with_goal()
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)

        planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual(1, len(planner.temporary_obstacles))
        circle = planner.temporary_obstacles[0]
        self.assertAlmostEqual(0.90, circle[0], places=6)
        self.assertAlmostEqual(0.00, circle[1], places=6)
        self.assertAlmostEqual(0.20, circle[2], places=6)

    def test_duplicate_visual_observations_merge(self):
        planner = planner_with_goal()

        planner._remember_obstacle((0.70, -1.45, 0.10))
        planner._remember_obstacle((0.74, -1.43, 0.11))

        self.assertEqual(1, len(planner.temporary_obstacles))
        self.assertAlmostEqual(0.11, planner.temporary_obstacles[0][2])

    def test_noisy_observations_half_metre_apart_merge_as_one_cylinder(self):
        planner = planner_with_goal()

        planner._remember_obstacle((0.70, -1.45, 0.20))
        planner._remember_obstacle((1.15, -1.45, 0.20))

        self.assertEqual(1, len(planner.temporary_obstacles))

    def test_dynamic_memory_is_capped_so_noise_cannot_seal_the_field(self):
        """The rules place exactly two random cylinders.

        Every extra remembered circle is a false positive that shrinks the
        free space until A* reports ``route_unreachable``, so the memory keeps
        only the most recent few observations.
        """
        planner = planner_with_goal()

        for index in range(12):
            planner._remember_obstacle((0.20 * index, -1.45 + 0.9 * index,
                                        0.20))

        self.assertLessEqual(len(planner.temporary_obstacles), 4)
        newest = planner.temporary_obstacles[-1]
        self.assertAlmostEqual(2.20, newest[0], places=6)
        self.assertAlmostEqual(8.45, newest[1], places=6)

    def test_cylinders_nine_tenths_apart_remain_distinct(self):
        planner = planner_with_goal()

        planner._remember_obstacle((0.70, -1.45, 0.20))
        planner._remember_obstacle((1.60, -1.45, 0.20))

        self.assertEqual(2, len(planner.temporary_obstacles))

    def test_close_observation_replans_with_temporary_obstacles(self):
        calls = []

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner.clearance_override = (1.30, 1.00)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

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
        self.assertEqual('dynamic_route_replanned', second.reason)
        self.assertEqual((0.0, 0.50, 1.2), second.target)
        self.assertEqual(1, len(calls))
        self.assertEqual(1, len(planner.temporary_obstacles))
        self.assertAlmostEqual(1.50, planner.temporary_obstacles[0][0])
        self.assertAlmostEqual(0.20, planner.temporary_obstacles[0][2])

    def test_far_cylinder_replans_while_route_yaw_is_still_aligning(self):
        calls = []
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35, left_m=0.70)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.50, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        turning_pose = (0.0, 0.0, 1.2, math.pi / 2.0)

        first = planner.update(turning_pose, (far_obstacle,), True, 1.0)
        second = planner.update(turning_pose, (far_obstacle,), True, 1.1)

        self.assertEqual('aligning_route_yaw', first.reason)
        self.assertEqual('dynamic_route_replanned', second.reason)
        self.assertEqual(1, len(calls))
        self.assertEqual(1, len(planner.temporary_obstacles))

    def test_remembered_far_cylinder_does_not_replan_every_two_frames(self):
        calls = []
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.50, 0.0), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)

        for index in range(8):
            planner.update(POSE, (far_obstacle,), True, 1.0 + index * 0.1)

        self.assertEqual(1, len(calls))
        self.assertEqual(1, len(planner.temporary_obstacles))

    def test_remembered_replanned_cylinder_does_not_trigger_local_stop(self):
        calls = []
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)
        near_obstacle = OBSTACLE._replace(
            forward_m=0.40, nearest_range_m=0.35)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.50, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner.update(POSE, (far_obstacle,), True, 1.0)
        planner.update(POSE, (far_obstacle,), True, 1.1)

        command = planner.update(
            (1.0, 0.0, 1.2, 0.0), (near_obstacle,), True, 1.2)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertNotEqual('obstacle_detected', command.reason)
        self.assertEqual(1, len(calls))

    def test_next_mission_leg_is_planned_around_remembered_cylinders(self):
        """Learned cylinders must survive the switch to the next task zone.

        Suppressing the repeated local stop is only safe while the active
        route already accounts for every remembered cylinder, so a new goal
        has to be planned with that memory instead of the bare fixed map.
        """
        calls = []

        def dynamic_route(start, goal, circles):
            calls.append(tuple(circles))
            return (tuple(start), (0.50, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner._remember_obstacle((1.50, 0.00, 0.20))

        planner.set_goal((2.0, -1.0, 1.2), POSE)

        self.assertEqual([((1.50, 0.00, 0.20),)], calls)

    def test_first_goal_without_memory_uses_the_fixed_map_route(self):
        calls = []

        def dynamic_route(start, goal, circles):
            calls.append(tuple(circles))
            return (tuple(start), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)

        self.assertEqual([], calls)

    def test_remembered_cylinder_still_brakes_inside_emergency_range(self):
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)
        emergency_obstacle = OBSTACLE._replace(
            forward_m=0.30, nearest_range_m=0.25)

        def dynamic_route(start, goal, circles):
            return (tuple(start), (0.50, 0.50), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner.update(POSE, (far_obstacle,), True, 1.0)
        planner.update(POSE, (far_obstacle,), True, 1.1)

        command = planner.update(
            (1.20, 0.0, 1.2, 0.0), (emergency_obstacle,), True, 1.2)

        self.assertEqual('BRAKE', command.state)

    def test_unremembered_cylinder_still_brakes_at_trigger_range(self):
        planner = planner_with_goal()

        command = planner.update(POSE, (OBSTACLE,), True, 1.0)

        self.assertEqual('BRAKE', command.state)

    def test_default_emergency_range_is_inside_trigger_range(self):
        config = VisualPlannerConfig()

        self.assertAlmostEqual(0.35, config.emergency_range, places=6)
        self.assertLess(config.emergency_range, config.trigger_range)

    def test_one_metre_camera_detection_replans_before_close_range_brake(self):
        calls = []
        observed_cylinder = OBSTACLE._replace(
            forward_m=0.71, nearest_range_m=0.66,
            left_m=0.48, left_edge_m=0.61, right_edge_m=0.42)

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), tuple(goal))

        config = VisualPlannerConfig(sensor_forward_offset=0.32)
        planner = planner_with_goal(
            config=config, dynamic_route_provider=dynamic_route)

        first = planner.update(POSE, (observed_cylinder,), True, 1.0)
        second = planner.update(POSE, (observed_cylinder,), True, 1.1)

        self.assertEqual('FOLLOW_ROUTE', first.state)
        self.assertEqual('FOLLOW_ROUTE', second.state)
        self.assertEqual(1, len(calls))
        surface_range = math.hypot(
            observed_cylinder.forward_m + config.sensor_forward_offset,
            observed_cylinder.left_m)
        remembered_range = math.hypot(calls[0][2][0][0],
                                      calls[0][2][0][1])
        self.assertGreater(remembered_range, surface_range)

    def test_far_cylinder_memory_uses_bounded_uncertainty_not_projected_width(self):
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
        self.assertAlmostEqual(0.20, calls[0][0][2])

    def test_far_dynamic_replan_failure_defers_without_poisoning_route(self):
        far_obstacle = OBSTACLE._replace(
            forward_m=1.40, nearest_range_m=1.35)

        def unreachable(_start, _goal, _circles):
            raise ValueError('route_unreachable')

        planner = planner_with_goal(dynamic_route_provider=unreachable)

        planner.update(POSE, (far_obstacle,), True, 1.0)
        failed = planner.update(POSE, (far_obstacle,), True, 1.1)

        self.assertEqual('FOLLOW_ROUTE', failed.state)
        self.assertEqual('dynamic_route_deferred', failed.reason)
        self.assertEqual((), planner.temporary_obstacles)

    def test_dynamic_replan_failure_holds(self):
        def unreachable(_start, _goal, _circles):
            raise ValueError('route_unreachable')

        planner = planner_with_goal(dynamic_route_provider=unreachable)
        planner.clearance_override = (1.30, 1.00)

        command = drive_through_rejoin(planner)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('dynamic_route_unreachable', command.reason)

    def test_close_cylinder_replans_after_brake_and_observation(self):
        calls = []

        def dynamic_route(start, goal, circles):
            calls.append((tuple(start), tuple(goal), tuple(circles)))
            return (tuple(start), (0.0, 0.5), tuple(goal))

        planner = planner_with_goal(dynamic_route_provider=dynamic_route)
        planner.clearance_override = (1.30, 1.30)
        drive_to_select(planner)

        command = planner.update(POSE, (OBSTACLE,), True, 1.5)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertEqual('dynamic_route_replanned', command.reason)
        self.assertEqual((0.0, 0.5, 1.2), command.target)
        self.assertEqual(1, len(calls))
        self.assertAlmostEqual(0.20, calls[0][2][0][2])

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

    def test_visual_loss_latches_first_hold_yaw(self):
        planner = planner_with_goal()
        first = planner.update((0.0, 0.0, 1.2, -1.2), (), False, 2.0)

        second = planner.update((0.2, -0.3, 1.2, 0.8), (), False, 2.1)

        self.assertAlmostEqual(-1.2, first.target_yaw, places=6)
        self.assertAlmostEqual(-1.2, second.target_yaw, places=6)

    def test_geofence_warning_commands_recovery_inside_field(self):
        planner = VisualPathPlanner(
            config=VisualPlannerConfig(known_static_tolerance=-1.0),
            route_provider=straight_route)
        planner.set_goal((2.0, 0.0, 1.2), POSE)
        near_west_net = (-0.31, -1.50, 1.20, -1.2)

        command = planner.update(near_west_net, (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('geofence_recovery', command.reason)
        self.assertAlmostEqual(-0.10, command.target[0], places=6)
        self.assertAlmostEqual(-1.50, command.target[1], places=6)
        self.assertAlmostEqual(-1.2, command.target_yaw, places=6)

    def test_small_start_area_yaw_drift_does_not_trigger_geofence(self):
        planner = VisualPathPlanner(
            config=VisualPlannerConfig(known_static_tolerance=-1.0),
            route_provider=straight_route)
        planner.set_goal((2.0, 0.0, 1.2), POSE)

        command = planner.update((0.0, 0.20, 1.20, -1.2), (), True, 2.0)

        self.assertNotEqual('geofence_recovery', command.reason)

    def test_geofence_recovery_stays_latched_until_recentred(self):
        planner = VisualPathPlanner(
            config=VisualPlannerConfig(known_static_tolerance=-1.0),
            route_provider=straight_route)
        planner.set_goal((2.0, 0.0, 1.2), POSE)
        planner.update((-0.31, -1.50, 1.20, -1.2), (), True, 2.0)

        still_recovering = planner.update(
            (-0.20, -1.50, 1.20, -1.2), (), True, 2.1)
        recentred = planner.update(
            (-0.08, -1.50, 1.20, -1.2), (), True, 2.2)

        self.assertEqual('geofence_recovery', still_recovering.reason)
        self.assertNotEqual('geofence_recovery', recentred.reason)

    def test_altitude_error_holds_xy_and_recovers_12(self):
        command = planner_with_goal().update(
            (0.3, -0.2, 1.36, 0.0), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual((0.3, -0.2, 1.2), command.target)
        self.assertEqual('altitude_out_of_band', command.reason)

    def test_small_altitude_overshoot_stops_before_scoring_limit(self):
        command = planner_with_goal().update(
            (0.0, 0.0, 1.305, 0.0), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('altitude_out_of_band', command.reason)

    def test_altitude_sag_stops_horizontal_route_early(self):
        command = planner_with_goal().update(
            (0.0, -1.0, 1.04, -1.57), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('altitude_out_of_band', command.reason)

    def test_default_altitude_guard_acts_before_one_point_one_metres(self):
        self.assertAlmostEqual(
            0.08, VisualPlannerConfig().altitude_tolerance, places=6)
        command = planner_with_goal().update(
            (0.0, -1.0, 1.11, -1.57), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('altitude_out_of_band', command.reason)

    def test_non_finite_pose_never_reports_goal_reached(self):
        command = planner_with_goal().update(
            (float('nan'), 0.0, 1.2, 0.0), (), True, 2.0)

        self.assertEqual('HOLD_UNSAFE', command.state)
        self.assertEqual('invalid_pose', command.reason)
        self.assertIsNone(command.target)

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

        config = VisualPlannerConfig(geofence_warning_margin=-10.0)
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

    def test_visual_avoidance_turns_while_creeping_toward_route(self):
        def turning_route(start, goal):
            return (tuple(start), (0.0, -0.5), tuple(goal))
        planner = VisualPathPlanner(route_provider=turning_route)
        planner.set_goal((1.0, -0.5, 1.2), POSE)

        command = planner.update(POSE, (OBSTACLE,), True, 3.0)

        self.assertEqual('FOLLOW_ROUTE', command.state)
        self.assertEqual((0.0, -0.5, 1.2), command.target)
        self.assertAlmostEqual(-1.5708, command.target_yaw, places=3)

    def test_default_yaw_alignment_waits_for_rotation_to_settle(self):
        self.assertAlmostEqual(
            0.10, VisualPlannerConfig().yaw_alignment_tolerance, places=6)

    def test_route_yaw_alignment_keeps_advancing_to_waypoint(self):
        def turning_route(start, goal):
            return (tuple(start), (0.0, -0.5), tuple(goal))
        planner = VisualPathPlanner(route_provider=turning_route)
        planner.set_goal((1.0, -0.5, 1.2), POSE)
        planner.update(POSE, (), True, 3.0)

        command = planner.update((0.2, -0.3, 1.2, -0.1), (), True, 3.1)

        self.assertEqual((0.0, -0.5, 1.2), command.target)

    def test_follow_route_skips_waypoint_passed_without_entering_radius(self):
        def staged_route(start, goal):
            return (tuple(start), (0.0, -1.0), (1.0, -1.0), tuple(goal))
        planner = VisualPathPlanner(route_provider=staged_route)
        planner.set_goal((2.0, -1.0, 1.2), POSE)

        command = planner.update((0.10, -1.20, 1.2, 0.0), (), True, 3.0)

        self.assertEqual((1.0, -1.0, 1.2), command.target)
        self.assertEqual('', command.reason)
        self.assertAlmostEqual(0.0, command.target_yaw, places=6)

    def test_intermediate_waypoint_yaw_uses_stable_segment_direction(self):
        def staged_route(start, goal):
            return (tuple(start), (0.0, -1.0), (1.0, -1.0), tuple(goal))
        planner = VisualPathPlanner(route_provider=staged_route)
        planner.set_goal((2.0, -1.0, 1.2), POSE)

        command = planner.update((0.10, -0.90, 1.2, 0.0), (), True, 3.0)

        self.assertEqual('aligning_route_yaw', command.reason)
        self.assertEqual((0.0, -1.0, 1.2), command.target)
        self.assertAlmostEqual(-math.pi / 2.0,
                               command.target_yaw, places=6)

    def test_close_obstacle_brakes_before_route_yaw_alignment(self):
        planner = planner_with_goal()
        misaligned_pose = (POSE[0], POSE[1], POSE[2], math.pi / 2.0)

        command = planner.update(misaligned_pose, (OBSTACLE,), True, 3.0)

        self.assertEqual('BRAKE', command.state)
        self.assertEqual(misaligned_pose[:3], command.target)

    def test_follow_route_never_projection_skips_final_goal(self):
        planner = planner_with_goal()

        command = planner.update((3.0, 0.0, 1.2, 0.0), (), True, 3.0)

        self.assertNotEqual('REACHED', command.state)
        self.assertEqual('aligning_route_yaw', command.reason)
        self.assertAlmostEqual(math.pi, abs(command.target_yaw), places=6)

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
            maximum_lead=None, lock_xy=True)

        self.assertEqual((0.0, 0.0), output[:2])

    def test_setpoint_ramp_snaps_to_new_latched_hold_before_turning(self):
        output = ramp_setpoint(
            last=(0.0, -1.40, 1.2, -1.57),
            desired=(0.05, -1.58, 1.2, -0.90),
            current=(0.05, -1.58, 1.2, -1.57), dt=0.10,
            maximum_lead=None, lock_xy=True)

        self.assertEqual((0.05, -1.58), output[:2])
        self.assertAlmostEqual(-1.51, output[3], places=6)

    def test_setpoint_lead_limit_never_drags_route_toward_pose_drift(self):
        output = ramp_setpoint(
            last=(0.0, -0.5, 1.2, -1.57),
            desired=(0.0, -1.0, 1.2, -1.57),
            current=(0.3, -0.5, 1.2, -1.57), dt=0.10,
            horizontal_speed=0.20, maximum_lead=0.08)

        self.assertEqual((0.0, -0.5), output[:2])

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
