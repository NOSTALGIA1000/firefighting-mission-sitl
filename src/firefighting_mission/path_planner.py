from __future__ import division, print_function

import math
from collections import namedtuple

from firefighting_mission.field_map import (plan_route,
                                            point_is_free,
                                            point_matches_field_boundary)


PlanCommand = namedtuple(
    'PlanCommand',
    'state target target_yaw selected_side left_clearance right_clearance reason')


class VisualPlannerConfig(object):
    def __init__(self, altitude=1.20, altitude_tolerance=0.10,
                 trigger_range=1.00, minimum_corridor=0.90,
                 aircraft_radius=0.20, external_clearance=0.25,
                 pass_distance=0.80, waypoint_tolerance=0.12,
                 observation_frames=3, side_hysteresis=0.15,
                 yaw_alignment_tolerance=0.35, lateral_trigger=0.55,
                 sensor_forward_offset=0.0,
                 known_static_tolerance=0.18):
        self.altitude = float(altitude)
        self.altitude_tolerance = float(altitude_tolerance)
        self.trigger_range = float(trigger_range)
        self.minimum_corridor = float(minimum_corridor)
        self.aircraft_radius = float(aircraft_radius)
        self.external_clearance = float(external_clearance)
        self.pass_distance = float(pass_distance)
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.observation_frames = int(observation_frames)
        self.side_hysteresis = float(side_hysteresis)
        self.yaw_alignment_tolerance = float(yaw_alignment_tolerance)
        self.lateral_trigger = float(lateral_trigger)
        self.sensor_forward_offset = float(sensor_forward_offset)
        self.known_static_tolerance = float(known_static_tolerance)


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def ramp_setpoint(last, desired, current, dt, horizontal_speed=0.30,
                  maximum_lead=0.12, yaw_rate=0.60):
    if last is None:
        last = (current[0], current[1], desired[2], current[3])
    seconds = max(0.0, min(float(dt), 0.10))
    max_step = float(horizontal_speed) * seconds
    delta_x = desired[0] - last[0]
    delta_y = desired[1] - last[1]
    distance = math.hypot(delta_x, delta_y)
    scale = min(1.0, max_step / distance) if distance > 1e-9 else 1.0
    x_value = last[0] + delta_x * scale
    y_value = last[1] + delta_y * scale
    lead_x = x_value - current[0]
    lead_y = y_value - current[1]
    lead = math.hypot(lead_x, lead_y)
    if lead > maximum_lead:
        lead_scale = float(maximum_lead) / lead
        x_value = current[0] + lead_x * lead_scale
        y_value = current[1] + lead_y * lead_scale
    yaw_delta = angle_difference(desired[3], last[3])
    yaw_step = max(-yaw_rate * seconds, min(yaw_rate * seconds, yaw_delta))
    yaw_value = last[3] + yaw_step
    return (x_value, y_value, desired[2], yaw_value)


def _body_to_world(pose, forward, left):
    cosine = math.cos(pose[3])
    sine = math.sin(pose[3])
    return (pose[0] + cosine * forward - sine * left,
            pose[1] + sine * forward + cosine * left)


def corridor_clearances(pose, obstacle, sample_step=0.05,
                        maximum_width=4.0):
    left_width = 0.0
    lateral = obstacle.left_edge_m
    while left_width < maximum_width:
        lateral += sample_step
        point = _body_to_world(pose, obstacle.forward_m, lateral)
        if not point_is_free(point, inflation=0.0):
            break
        left_width = lateral - obstacle.left_edge_m

    right_width = 0.0
    lateral = obstacle.right_edge_m
    while right_width < maximum_width:
        lateral -= sample_step
        point = _body_to_world(pose, obstacle.forward_m, lateral)
        if not point_is_free(point, inflation=0.0):
            break
        right_width = obstacle.right_edge_m - lateral
    return round(left_width, 3), round(right_width, 3)


def segment_is_free(first, second, inflation=0.20, sample_step=0.05):
    distance = math.hypot(second[0] - first[0], second[1] - first[1])
    count = max(1, int(math.ceil(distance / float(sample_step))))
    for index in range(count + 1):
        ratio = index / float(count)
        point = (first[0] + (second[0] - first[0]) * ratio,
                 first[1] + (second[1] - first[1]) * ratio)
        if not point_is_free(point, inflation=inflation):
            return False
    return True


def avoidance_path_is_free(start, side, passing, rejoin):
    return (segment_is_free(start, side) and
            segment_is_free(side, passing) and
            segment_is_free(passing, rejoin))


class VisualPathPlanner(object):
    def __init__(self, config=None, route_provider=None, path_validator=None,
                 dynamic_route_provider=None):
        self.config = config or VisualPlannerConfig()
        self.route_provider = route_provider or plan_route
        if dynamic_route_provider is not None:
            self.dynamic_route_provider = dynamic_route_provider
        elif route_provider is None:
            self.dynamic_route_provider = self._dynamic_route
        else:
            self.dynamic_route_provider = None
        self.path_validator = path_validator or avoidance_path_is_free
        self.goal = None
        self.route = ()
        self.waypoint_index = 0
        self.state = 'IDLE'
        self.selected_side = ''
        self.last_selected_side = ''
        self.left_clearance = 0.0
        self.right_clearance = 0.0
        self.observation_count = 0
        self.active_obstacle = None
        self.side_target = None
        self.pass_target = None
        self.rejoin_target = None
        self.route_yaw = 0.0
        self.interrupted_state = None
        self.clearance_override = None
        self.temporary_obstacles = ()

    @staticmethod
    def _dynamic_route(start, goal, circles):
        return plan_route(start, goal, dynamic_circles=circles)

    def set_goal(self, goal, pose):
        self.goal = (float(goal[0]), float(goal[1]), self.config.altitude)
        self.route = tuple(self.route_provider(
            (float(pose[0]), float(pose[1])), self.goal[:2]))
        self.waypoint_index = 1 if len(self.route) > 1 else 0
        self.state = 'FOLLOW_ROUTE'
        self.selected_side = ''
        self.active_obstacle = None
        self.interrupted_state = None

    @staticmethod
    def _horizontal_distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _remember_obstacle(self, circle):
        circles = list(self.temporary_obstacles)
        for index, current in enumerate(circles):
            separation = math.hypot(
                circle[0] - current[0], circle[1] - current[1])
            if separation <= max(circle[2], current[2]):
                circles[index] = (
                    (circle[0] + current[0]) / 2.0,
                    (circle[1] + current[1]) / 2.0,
                    max(circle[2], current[2]))
                self.temporary_obstacles = tuple(circles)
                return
        circles.append(tuple(float(value) for value in circle))
        self.temporary_obstacles = tuple(circles)

    def _command(self, state, target, yaw, reason=''):
        return PlanCommand(
            state,
            (float(target[0]), float(target[1]), self.config.altitude),
            float(yaw), self.selected_side, self.left_clearance,
            self.right_clearance, reason)

    def _hold(self, pose, reason, remember=True):
        if remember and self.state != 'HOLD_UNSAFE':
            self.interrupted_state = self.state
        return self._command('HOLD_UNSAFE', pose[:2], pose[3], reason)

    def _nearest_obstacle(self, pose, obstacles):
        candidates = []
        for value in obstacles:
            shifted = value._replace(
                forward_m=(value.forward_m +
                           self.config.sensor_forward_offset),
                nearest_range_m=(value.nearest_range_m +
                                 self.config.sensor_forward_offset))
            surface = _body_to_world(pose, shifted.forward_m,
                                     shifted.left_m)
            if point_matches_field_boundary(
                    surface, self.config.known_static_tolerance):
                continue
            if (shifted.forward_m > 0.0 and
                    abs(shifted.left_m) <= self.config.lateral_trigger):
                candidates.append(shifted)
        if not candidates:
            return None
        return min(candidates, key=lambda value: value.nearest_range_m)

    def _follow_route(self, pose):
        while self.waypoint_index < len(self.route):
            waypoint = self.route[self.waypoint_index]
            if self._horizontal_distance(pose, waypoint) > self.config.waypoint_tolerance:
                break
            self.waypoint_index += 1
        if self.waypoint_index >= len(self.route):
            self.state = 'REACHED'
            return self._command('REACHED', self.goal[:2], pose[3])
        waypoint = self.route[self.waypoint_index]
        self.route_yaw = math.atan2(waypoint[1] - pose[1],
                                    waypoint[0] - pose[0])
        return self._command('FOLLOW_ROUTE', waypoint, self.route_yaw)

    def _skip_passed_waypoints(self, pose):
        while 0 < self.waypoint_index < len(self.route):
            previous = self.route[self.waypoint_index - 1]
            waypoint = self.route[self.waypoint_index]
            segment_x = waypoint[0] - previous[0]
            segment_y = waypoint[1] - previous[1]
            segment_squared = segment_x * segment_x + segment_y * segment_y
            if segment_squared <= 1e-9:
                self.waypoint_index += 1
                continue
            progress = ((pose[0] - previous[0]) * segment_x +
                        (pose[1] - previous[1]) * segment_y)
            if progress < segment_squared:
                break
            self.waypoint_index += 1

    def _select_side(self, pose):
        if self.active_obstacle is None:
            self.state = 'FOLLOW_ROUTE'
            return self._follow_route(pose)
        if self.clearance_override is None:
            clearances = corridor_clearances(pose, self.active_obstacle)
        else:
            clearances = self.clearance_override
        self.left_clearance = float(clearances[0])
        self.right_clearance = float(clearances[1])
        forward_to_clear = (self.active_obstacle.forward_m +
                            self.config.pass_distance)

        def candidate(sign):
            clearance = (self.config.aircraft_radius +
                         self.config.external_clearance)
            if sign > 0.0:
                lateral = self.active_obstacle.left_edge_m + clearance
            else:
                lateral = self.active_obstacle.right_edge_m - clearance
            side_xy = _body_to_world(pose, 0.0, lateral)
            pass_xy = _body_to_world(pose, forward_to_clear, lateral)
            rejoin_xy = _body_to_world(pose, forward_to_clear, 0.0)
            return side_xy, pass_xy, rejoin_xy

        left_candidate = candidate(1.0)
        right_candidate = candidate(-1.0)
        left_valid = (self.left_clearance >= self.config.minimum_corridor and
                      self.path_validator(pose[:2], *left_candidate))
        right_valid = (self.right_clearance >= self.config.minimum_corridor and
                       self.path_validator(pose[:2], *right_candidate))
        if not left_valid and not right_valid:
            self.state = 'HOLD_UNSAFE'
            return self._hold(pose, 'no_safe_corridor', remember=False)
        if left_valid and not right_valid:
            side = 'LEFT'
        elif right_valid and not left_valid:
            side = 'RIGHT'
        elif (self.left_clearance - self.right_clearance >
              self.config.side_hysteresis):
            side = 'LEFT'
        elif (self.right_clearance - self.left_clearance >
              self.config.side_hysteresis):
            side = 'RIGHT'
        elif self.last_selected_side in ('LEFT', 'RIGHT'):
            side = self.last_selected_side
        else:
            side = 'LEFT'
        self.selected_side = side
        self.last_selected_side = side
        obstacle_center = _body_to_world(
            pose, self.active_obstacle.forward_m,
            self.active_obstacle.left_m)
        obstacle_radius = max(
            0.10,
            abs(self.active_obstacle.left_edge_m -
                self.active_obstacle.right_edge_m) / 2.0)
        self._remember_obstacle(
            (obstacle_center[0], obstacle_center[1], obstacle_radius))
        selected = left_candidate if side == 'LEFT' else right_candidate
        self.side_target, self.pass_target, self.rejoin_target = selected
        self.state = 'SIDESTEP'
        return self._command('SIDESTEP', self.side_target, self.route_yaw)

    def update(self, pose, obstacles, perception_ready, now):
        del now
        pose = tuple(float(value) for value in pose)
        if self.goal is None:
            return PlanCommand('IDLE', None, pose[3], '', 0.0, 0.0,
                               'goal_missing')
        if not perception_ready:
            return self._hold(pose, 'perception_not_ready')
        if abs(pose[2] - self.config.altitude) > self.config.altitude_tolerance:
            return self._hold(pose, 'altitude_out_of_band')
        if self.interrupted_state is not None:
            self.state = self.interrupted_state
            self.interrupted_state = None

        nearest = self._nearest_obstacle(pose, obstacles)
        if self.state == 'FOLLOW_ROUTE':
            route_command = self._follow_route(pose)
            if route_command.state == 'REACHED':
                return route_command
            if abs(angle_difference(pose[3], route_command.target_yaw)) > \
                    self.config.yaw_alignment_tolerance:
                return self._command(
                    'FOLLOW_ROUTE', pose[:2], route_command.target_yaw,
                    'aligning_route_yaw')
            if (nearest is not None and
                    nearest.nearest_range_m < self.config.trigger_range):
                self.active_obstacle = nearest
                self.state = 'BRAKE'
                return self._command('BRAKE', pose[:2], pose[3])
            return route_command

        if self.state == 'BRAKE':
            self.state = 'OBSERVE'
            self.observation_count = 0
            return self._command('OBSERVE', pose[:2], pose[3])

        if self.state == 'OBSERVE':
            if nearest is not None:
                self.active_obstacle = nearest
            self.observation_count += 1
            if self.observation_count >= self.config.observation_frames:
                self.state = 'SELECT_SIDE'
                return self._command('SELECT_SIDE', pose[:2], pose[3])
            return self._command('OBSERVE', pose[:2], pose[3])

        if self.state == 'SELECT_SIDE':
            return self._select_side(pose)

        if self.state == 'SIDESTEP':
            if self._horizontal_distance(pose, self.side_target) <= self.config.waypoint_tolerance:
                self.state = 'PASS'
                return self._command('PASS', self.pass_target, self.route_yaw)
            return self._command('SIDESTEP', self.side_target, self.route_yaw)

        if self.state == 'PASS':
            if self._horizontal_distance(pose, self.pass_target) <= self.config.waypoint_tolerance:
                self.state = 'REJOIN'
                return self._command('REJOIN', self.rejoin_target, self.route_yaw)
            return self._command('PASS', self.pass_target, self.route_yaw)

        if self.state == 'REJOIN':
            if self._horizontal_distance(pose, self.rejoin_target) <= self.config.waypoint_tolerance:
                if self.dynamic_route_provider is not None:
                    try:
                        self.route = tuple(self.dynamic_route_provider(
                            pose[:2], self.goal[:2],
                            self.temporary_obstacles))
                    except ValueError:
                        self.state = 'HOLD_UNSAFE'
                        return self._hold(
                            pose, 'dynamic_route_unreachable',
                            remember=False)
                    self.waypoint_index = 1 if len(self.route) > 1 else 0
                else:
                    self._skip_passed_waypoints(pose)
                self.state = 'FOLLOW_ROUTE'
                self.selected_side = ''
                self.active_obstacle = None
                return self._follow_route(pose)
            return self._command('REJOIN', self.rejoin_target, self.route_yaw)

        if self.state == 'HOLD_UNSAFE':
            if nearest is None:
                return self._hold(
                    pose, 'obstacle_temporarily_unseen', remember=False)
            if nearest.nearest_range_m >= self.config.trigger_range:
                self.state = 'FOLLOW_ROUTE'
                self.selected_side = ''
                self.active_obstacle = None
                return self._follow_route(pose)
            self.active_obstacle = nearest
            self.state = 'SELECT_SIDE'
            return self._select_side(pose)
        if self.state == 'REACHED':
            return self._command('REACHED', self.goal[:2], pose[3])
        return self._hold(pose, 'invalid_planner_state')
