from __future__ import division, print_function

import math
from collections import namedtuple

from firefighting_mission.field_map import plan_route, point_is_free


PlanCommand = namedtuple(
    'PlanCommand',
    'state target target_yaw selected_side left_clearance right_clearance reason')


class VisualPlannerConfig(object):
    def __init__(self, altitude=1.20, altitude_tolerance=0.10,
                 trigger_range=1.00, minimum_corridor=0.90,
                 aircraft_radius=0.20, external_clearance=0.25,
                 pass_distance=0.55, waypoint_tolerance=0.12,
                 observation_frames=3, side_hysteresis=0.15):
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


class VisualPathPlanner(object):
    def __init__(self, config=None, route_provider=None):
        self.config = config or VisualPlannerConfig()
        self.route_provider = route_provider or plan_route
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

    def _nearest_obstacle(self, obstacles):
        candidates = [value for value in obstacles
                      if value.forward_m > 0.0 and abs(value.left_m) <= 0.80]
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
        left_valid = self.left_clearance >= self.config.minimum_corridor
        right_valid = self.right_clearance >= self.config.minimum_corridor
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
        sign = 1.0 if side == 'LEFT' else -1.0
        obstacle_edge = (self.active_obstacle.left_edge_m if sign > 0.0
                         else abs(self.active_obstacle.right_edge_m))
        lateral = sign * (obstacle_edge + self.config.aircraft_radius +
                          self.config.external_clearance)
        forward_to_clear = (self.active_obstacle.forward_m +
                            self.config.pass_distance)
        side_xy = _body_to_world(pose, 0.0, lateral)
        pass_xy = _body_to_world(pose, forward_to_clear, lateral)
        rejoin_xy = _body_to_world(pose, forward_to_clear, 0.0)
        self.side_target = side_xy
        self.pass_target = pass_xy
        self.rejoin_target = rejoin_xy
        self.state = 'SIDESTEP'
        return self._command('SIDESTEP', side_xy, self.route_yaw)

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

        nearest = self._nearest_obstacle(obstacles)
        if self.state == 'FOLLOW_ROUTE':
            if (nearest is not None and
                    nearest.nearest_range_m < self.config.trigger_range):
                self.active_obstacle = nearest
                self.state = 'BRAKE'
                return self._command('BRAKE', pose[:2], pose[3])
            return self._follow_route(pose)

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
                self.state = 'FOLLOW_ROUTE'
                self.selected_side = ''
                self.active_obstacle = None
                return self._follow_route(pose)
            return self._command('REJOIN', self.rejoin_target, self.route_yaw)

        if self.state == 'HOLD_UNSAFE':
            return self._hold(pose, 'no_safe_corridor', remember=False)
        if self.state == 'REACHED':
            return self._command('REACHED', self.goal[:2], pose[3])
        return self._hold(pose, 'invalid_planner_state')
