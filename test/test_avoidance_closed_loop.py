"""Closed-loop avoidance regression over every drawn field layout.

This runs the real planner against a synthetic stereo sensor and a lagging
vehicle model, so the whole avoidance chain can be checked without ROS,
Gazebo or the VM.  It is the fast gate that has to stay green before a SITL
run is worth starting.
"""
from __future__ import division, print_function

import itertools
import math
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.path_planner import (
    VisualPathPlanner, VisualPlannerConfig, ramp_setpoint)
from firefighting_mission.stereo_obstacles import ObstacleClusterData
from firefighting_mission.world_generator import (
    CYLINDER_POSES, FIXED_OBSTACLES, HAZARD_POSES, PERSON_POSES)


CONTROL_PERIOD = 0.05
CRUISE_SPEED = 0.18
TURNING_SPEED = 0.12
SETPOINT_LEAD = 0.25
YAW_RATE = 0.35
SENSOR_OFFSET = 0.32
CAMERA_FIELD_OF_VIEW = math.radians(70.0)
CAMERA_MAX_RANGE = 3.0
CYLINDER_RADIUS = 0.10
AIRFRAME_RADIUS = 0.20
TRACKING_GAIN = 2.5
MISSION_TIME_LIMIT = 120.0
LOCKED_STATES = ('BRAKE', 'OBSERVE', 'SELECT_SIDE', 'HOLD_UNSAFE', 'REACHED')


def _board_distance(point):
    closest = float('inf')
    for _, center_x, center_y, yaw, length, width in FIXED_OBSTACLES:
        dx = point[0] - center_x
        dy = point[1] - center_y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_length = cosine * dx + sine * dy
        local_width = -sine * dx + cosine * dy
        closest = min(closest, math.hypot(
            max(0.0, abs(local_length) - length / 2.0),
            max(0.0, abs(local_width) - width / 2.0)))
    return closest


def _observe(pose, cylinders, range_bias):
    """Report camera-frame clusters for every cylinder inside the field of view."""
    clusters = []
    for center_x, center_y in cylinders:
        dx = center_x - pose[0]
        dy = center_y - pose[1]
        cosine = math.cos(-pose[3])
        sine = math.sin(-pose[3])
        forward = cosine * dx - sine * dy - SENSOR_OFFSET
        left = sine * dx + cosine * dy
        distance = math.hypot(forward, left)
        if distance < 1e-6 or distance > CAMERA_MAX_RANGE:
            continue
        if abs(math.atan2(left, forward)) > CAMERA_FIELD_OF_VIEW / 2.0:
            continue
        surface = distance - CYLINDER_RADIUS
        if surface <= 0.05:
            continue
        scale = surface / distance
        half_width = math.atan2(CYLINDER_RADIUS, distance) * surface
        clusters.append(ObstacleClusterData(
            forward * scale + range_bias, left * scale,
            surface + range_bias,
            left * scale + half_width, left * scale - half_width, 1.0))
    return tuple(sorted(clusters, key=lambda value: value.nearest_range_m))


def fly_mission(cylinders, goals, range_bias=0.0):
    """Return (elapsed, closest cylinder centre, closest board surface).

    Raises AssertionError-friendly RuntimeError when a leg never reaches its
    goal inside the mission time limit.
    """
    planner = VisualPathPlanner(VisualPlannerConfig(
        altitude=1.25, altitude_tolerance=0.15,
        yaw_alignment_tolerance=0.20, geofence_warning_margin=0.45,
        geofence_recovery_margin=0.65, minimum_corridor=0.90,
        trigger_range=0.85, sensor_forward_offset=SENSOR_OFFSET))
    pose = [0.0, 0.0, 1.25, 0.0]
    setpoint = None
    elapsed = 0.0
    closest_cylinder = float('inf')
    closest_board = float('inf')
    for goal in goals:
        planner.set_goal((goal[0], goal[1], 1.25), tuple(pose))
        deadline = elapsed + MISSION_TIME_LIMIT
        while True:
            command = planner.update(
                tuple(pose), _observe(tuple(pose), cylinders, range_bias),
                True, elapsed)
            if command.state == 'REACHED':
                break
            if elapsed > deadline:
                raise RuntimeError(
                    'goal %s never reached: state=%s reason=%s'
                    % (goal, command.state, command.reason))
            if command.target is not None:
                locked = command.state in LOCKED_STATES
                turning = command.reason == 'aligning_route_yaw'
                setpoint = ramp_setpoint(
                    setpoint,
                    (command.target[0], command.target[1], command.target[2],
                     command.target_yaw),
                    tuple(pose), CONTROL_PERIOD,
                    horizontal_speed=(
                        TURNING_SPEED if turning else CRUISE_SPEED),
                    maximum_lead=(None if locked else SETPOINT_LEAD),
                    yaw_rate=YAW_RATE, lock_xy=locked)
                lag = TRACKING_GAIN * CONTROL_PERIOD
                pose[0] += (setpoint[0] - pose[0]) * lag
                pose[1] += (setpoint[1] - pose[1]) * lag
                pose[3] += lag * math.atan2(
                    math.sin(setpoint[3] - pose[3]),
                    math.cos(setpoint[3] - pose[3]))
            elapsed += CONTROL_PERIOD
            for center_x, center_y in cylinders:
                closest_cylinder = min(closest_cylinder, math.hypot(
                    pose[0] - center_x, pose[1] - center_y))
            closest_board = min(closest_board,
                                _board_distance((pose[0], pose[1])))
    return elapsed, closest_cylinder, closest_board


def drawn_layouts():
    for first, second, hazard, person in itertools.product(
            (1, 2), (1, 2), (1, 2), (1, 2, 3)):
        yield (
            (CYLINDER_POSES[first][0], CYLINDER_POSES[second][1]),
            [HAZARD_POSES[hazard], PERSON_POSES[person], (0.0, 0.0)],
            'cylinders(%d,%d) hazard%d person%d' % (
                first, second, hazard, person),
        )


class ClosedLoopAvoidanceTest(unittest.TestCase):
    def test_every_drawn_layout_flies_the_mission_without_collision(self):
        for cylinders, goals, label in drawn_layouts():
            try:
                elapsed, cylinder, board = fly_mission(cylinders, goals)
            except RuntimeError as error:
                self.fail('%s: %s' % (label, error))
            self.assertGreater(
                cylinder, CYLINDER_RADIUS + AIRFRAME_RADIUS,
                '%s hit a cylinder' % label)
            self.assertGreater(
                board, AIRFRAME_RADIUS, '%s hit a board' % label)

    def test_worst_layout_leaves_headroom_in_the_three_minute_run(self):
        worst = 0.0
        for cylinders, goals, _ in drawn_layouts():
            elapsed, _, _ = fly_mission(cylinders, goals)
            worst = max(worst, elapsed)

        self.assertLess(worst, 120.0)

    def test_avoidance_tolerates_a_tenth_metre_of_stereo_range_bias(self):
        for bias in (0.10, -0.10):
            for cylinders, goals, label in drawn_layouts():
                try:
                    _, cylinder, board = fly_mission(
                        cylinders, goals, range_bias=bias)
                except RuntimeError as error:
                    self.fail('%s at bias %+.2f: %s' % (label, bias, error))
                self.assertGreater(
                    cylinder, CYLINDER_RADIUS + AIRFRAME_RADIUS,
                    '%s hit a cylinder at bias %+.2f' % (label, bias))
                self.assertGreater(
                    board, AIRFRAME_RADIUS,
                    '%s hit a board at bias %+.2f' % (label, bias))


if __name__ == '__main__':
    unittest.main()
