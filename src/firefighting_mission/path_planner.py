from __future__ import division, print_function

import math
from collections import namedtuple

from firefighting_mission.field_map import (plan_route,
                                            point_is_free,
                                            point_matches_known_static)
from firefighting_mission.world_generator import FIELD_BOUNDS


PlanCommand = namedtuple(
    'PlanCommand',
    'state target target_yaw selected_side left_clearance right_clearance reason')


class VisualPlannerConfig(object):
    def __init__(self, altitude=1.20, altitude_tolerance=0.08,
                 trigger_range=0.85, minimum_corridor=0.90,
                 aircraft_radius=0.20, external_clearance=0.25,
                 pass_distance=0.80, waypoint_tolerance=0.12,
                 observation_frames=3, side_hysteresis=0.15,
                 yaw_alignment_tolerance=0.10, lateral_trigger=0.55,
                 sensor_forward_offset=0.0,
                 emergency_range=0.35,
                 known_static_tolerance=0.18,
                 global_replan_range=2.00, global_replan_frames=2,
                 global_replan_merge_distance=0.55,
                 dynamic_obstacle_radius=0.10,
                 dynamic_localization_margin=0.10,
                 maximum_dynamic_obstacles=4,
                 blocked_route_retry_limit=40,
                 geofence_warning_margin=0.45,
                 geofence_recovery_margin=0.50):
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
        self.emergency_range = float(emergency_range)
        self.known_static_tolerance = float(known_static_tolerance)
        self.global_replan_range = float(global_replan_range)
        self.global_replan_frames = int(global_replan_frames)
        self.global_replan_merge_distance = float(global_replan_merge_distance)
        self.dynamic_obstacle_radius = float(dynamic_obstacle_radius)
        self.dynamic_localization_margin = float(dynamic_localization_margin)
        self.maximum_dynamic_obstacles = int(maximum_dynamic_obstacles)
        self.blocked_route_retry_limit = int(blocked_route_retry_limit)
        self.geofence_warning_margin = float(geofence_warning_margin)
        self.geofence_recovery_margin = float(geofence_recovery_margin)


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def map_target_to_local(target, map_pose, local_pose):
    heading_offset = angle_difference(local_pose[3], map_pose[3])
    return (
        float(target[0]) + float(local_pose[0]) - float(map_pose[0]),
        float(target[1]) + float(local_pose[1]) - float(map_pose[1]),
        float(target[2]),
        float(target[3]) + heading_offset,
    )


def ramp_setpoint(last, desired, current, dt, horizontal_speed=0.30,
                  maximum_lead=0.12, yaw_rate=0.60, lock_xy=False):
    if last is None:
        last = (current[0], current[1], desired[2], current[3])
    seconds = max(0.0, min(float(dt), 0.10))
    max_step = float(horizontal_speed) * seconds
    delta_x = desired[0] - last[0]
    delta_y = desired[1] - last[1]
    distance = math.hypot(delta_x, delta_y)
    scale = min(1.0, max_step / distance) if distance > 1e-9 else 1.0
    x_value = desired[0] if lock_xy else last[0] + delta_x * scale
    y_value = desired[1] if lock_xy else last[1] + delta_y * scale
    lead_x = x_value - current[0]
    lead_y = y_value - current[1]
    lead = math.hypot(lead_x, lead_y)
    if maximum_lead is not None and lead > float(maximum_lead):
        x_value = last[0]
        y_value = last[1]
    yaw_delta = angle_difference(desired[3], last[3])
    yaw_step = max(-yaw_rate * seconds, min(yaw_rate * seconds, yaw_delta))
    yaw_value = last[3] + yaw_step
    return (x_value, y_value, desired[2], yaw_value)


def setpoint_stream_target(command_target, last_output):
    """Return what to publish so OFFBOARD never loses its setpoint stream.

    A planner hold on an invalid pose used to publish nothing at all, and
    PX4 answered with "Failsafe enabled: no RC and no offboard" followed by
    a blind descend.  Holding the last commanded setpoint keeps the stream
    alive through a transient estimator glitch instead of turning it into a
    guaranteed landing; the aircraft is already tracking that point.
    """
    if command_target is not None:
        return command_target
    if last_output is None:
        return None
    return (last_output[0], last_output[1], last_output[2])


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
        self.hold_target = None
        self.hold_yaw = None
        self.global_obstacle_candidate = None
        self.global_observation_count = 0
        self.hold_reason = ''
        self.blocked_route_retries = 0

    @staticmethod
    def _dynamic_route(start, goal, circles):
        return plan_route(start, goal, dynamic_circles=circles)

    def set_goal(self, goal, pose):
        self.goal = (float(goal[0]), float(goal[1]), self.config.altitude)
        start = (float(pose[0]), float(pose[1]))
        self.selected_side = ''
        self.active_obstacle = None
        self.interrupted_state = None
        self.hold_target = None
        self.hold_yaw = None
        self._reset_global_candidate()
        try:
            self.route = tuple(self._plan_with_memory(start))
        except ValueError:
            if not self.temporary_obstacles:
                raise
            # Remembered geometry blocks the new leg.  Hold and let the
            # retry path work it out instead of losing the memory or the
            # goal outright.
            self.route = ()
            self.waypoint_index = 0
            self.state = 'HOLD_UNSAFE'
            self.hold_reason = 'dynamic_route_unreachable'
            self.blocked_route_retries = 0
            return
        self.waypoint_index = 1 if len(self.route) > 1 else 0
        self.state = 'FOLLOW_ROUTE'
        self.hold_reason = ''
        self.blocked_route_retries = 0

    def _plan_with_memory(self, start):
        """Plan to the active goal around every cylinder learned so far.

        Ignoring the memory here would leave the new leg following a route
        that passes through a known cylinder, which is exactly the case the
        suppressed local stop assumes cannot happen, so a blocked route is
        raised to the caller rather than quietly downgraded.
        """
        if self.dynamic_route_provider is None or not self.temporary_obstacles:
            return self.route_provider(start, self.goal[:2])
        return self.dynamic_route_provider(
            start, self.goal[:2], self.temporary_obstacles)

    def _shrink_remembered_margin(self):
        """Trim the localisation margin off remembered circles.

        Forgetting a circle outright once drove the aircraft into a real
        cylinder, so a wedge is worked out by giving back margin instead:
        the circles keep their centres and never shrink below the physical
        cylinder radius, so a route that is genuinely blocked stays blocked.
        """
        floor = self.config.dynamic_obstacle_radius
        self.temporary_obstacles = tuple(
            (x_value, y_value, max(floor, radius * 0.8))
            for x_value, y_value, radius in self.temporary_obstacles)

    def _retry_blocked_route(self, pose):
        """Try to leave a blocked-route hold, or return None to keep holding.

        Holding forever loses the run, so the hold keeps replanning, and
        after ``blocked_route_retry_limit`` consecutive failures it gives
        back some of the remembered localisation margin.
        """
        try:
            route = tuple(self._plan_with_memory(
                (float(pose[0]), float(pose[1]))))
        except ValueError:
            self.blocked_route_retries += 1
            if (self.blocked_route_retries >=
                    self.config.blocked_route_retry_limit):
                self.blocked_route_retries = 0
                self._shrink_remembered_margin()
            return None
        self.blocked_route_retries = 0
        self.route = route
        self.waypoint_index = 1 if len(self.route) > 1 else 0
        self.state = 'FOLLOW_ROUTE'
        self.selected_side = ''
        self.active_obstacle = None
        self._clear_hold()
        self._reset_global_candidate()
        return self._follow_route(pose, 'blocked_route_recovered')

    @staticmethod
    def _horizontal_distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _remember_obstacle(self, circle):
        circles = list(self.temporary_obstacles)
        for index, current in enumerate(circles):
            separation = math.hypot(
                circle[0] - current[0], circle[1] - current[1])
            if separation <= self.config.global_replan_merge_distance:
                circles[index] = (
                    (circle[0] + current[0]) / 2.0,
                    (circle[1] + current[1]) / 2.0,
                    max(circle[2], current[2]))
                self.temporary_obstacles = tuple(circles)
                return
        circles.append(tuple(float(value) for value in circle))
        limit = self.config.maximum_dynamic_obstacles
        if limit > 0:
            circles = circles[-limit:]
        self.temporary_obstacles = tuple(circles)

    def _dynamic_obstacle_circle(self, pose, obstacle):
        surface_range = math.hypot(obstacle.forward_m, obstacle.left_m)
        scale = (self.config.dynamic_obstacle_radius / surface_range
                 if surface_range > 1e-9 else 0.0)
        center = _body_to_world(
            pose,
            obstacle.forward_m * (1.0 + scale),
            obstacle.left_m * (1.0 + scale))
        radius = (self.config.dynamic_obstacle_radius +
                  self.config.dynamic_localization_margin)
        return (center[0], center[1], radius)

    def _reset_global_candidate(self):
        self.global_obstacle_candidate = None
        self.global_observation_count = 0

    def _track_global_candidate(self, circle):
        if self.global_obstacle_candidate is None:
            self.global_obstacle_candidate = circle
            self.global_observation_count = 1
            return
        separation = math.hypot(circle[0] - self.global_obstacle_candidate[0],
                                circle[1] - self.global_obstacle_candidate[1])
        if separation <= self.config.global_replan_merge_distance:
            self.global_obstacle_candidate = (
                (circle[0] + self.global_obstacle_candidate[0]) / 2.0,
                (circle[1] + self.global_obstacle_candidate[1]) / 2.0,
                max(circle[2], self.global_obstacle_candidate[2]))
            self.global_observation_count += 1
            return
        self.global_obstacle_candidate = circle
        self.global_observation_count = 1

    def _matches_remembered_obstacle(self, circle):
        for remembered in self.temporary_obstacles:
            separation = math.hypot(circle[0] - remembered[0],
                                    circle[1] - remembered[1])
            if separation <= self.config.global_replan_merge_distance:
                return True
        return False

    def _local_brake_required(self, pose, obstacle):
        """Return whether a close obstacle still needs the local stop cycle.

        A cylinder that already produced a dynamic replan is part of the
        active route geometry, so braking for it again only repeats the
        brake/observe/select cycle that was already paid for.  Anything
        closer than ``emergency_range`` still stops, because at that range
        localisation error alone can put the aircraft on the obstacle.
        """
        if obstacle.nearest_range_m < self.config.emergency_range:
            return True
        return not self._matches_remembered_obstacle(
            self._dynamic_obstacle_circle(pose, obstacle))

    def _maybe_replan_for_far_obstacle(self, pose, obstacle):
        if (self.dynamic_route_provider is None or obstacle is None or
                obstacle.nearest_range_m >= self.config.global_replan_range):
            self._reset_global_candidate()
            return None
        circle = self._dynamic_obstacle_circle(pose, obstacle)
        if self._matches_remembered_obstacle(circle):
            self._reset_global_candidate()
            return None
        self._track_global_candidate(circle)
        if self.global_observation_count < self.config.global_replan_frames:
            return None
        previous_obstacles = self.temporary_obstacles
        self._remember_obstacle(self.global_obstacle_candidate)
        try:
            self.route = tuple(self.dynamic_route_provider(
                pose[:2], self.goal[:2], self.temporary_obstacles))
        except ValueError:
            self.temporary_obstacles = previous_obstacles
            self._reset_global_candidate()
            return self._follow_route(pose, 'dynamic_route_deferred')
        self.waypoint_index = 1 if len(self.route) > 1 else 0
        self._reset_global_candidate()
        return self._follow_route(pose, 'dynamic_route_replanned')

    def _command(self, state, target, yaw, reason=''):
        return PlanCommand(
            state,
            (float(target[0]), float(target[1]), self.config.altitude),
            float(yaw), self.selected_side, self.left_clearance,
            self.right_clearance, reason)

    def _hold(self, pose, reason, remember=True):
        if remember and self.state != 'HOLD_UNSAFE':
            self.interrupted_state = self.state
        self.hold_reason = reason
        target = self._lock_hold(pose)
        return self._command(
            'HOLD_UNSAFE', target, self.hold_yaw, reason)

    def _lock_hold(self, pose):
        if self.hold_target is None:
            self.hold_target = (float(pose[0]), float(pose[1]))
            self.hold_yaw = float(pose[3])
        return self.hold_target

    def _clear_hold(self):
        self.hold_target = None
        self.hold_yaw = None
        self.hold_reason = ''

    def _geofence_recovery_target(self, pose):
        warning = self.config.geofence_warning_margin
        minimum_x = FIELD_BOUNDS[0] + warning
        maximum_x = FIELD_BOUNDS[1] - warning
        minimum_y = FIELD_BOUNDS[2] + warning
        maximum_y = FIELD_BOUNDS[3] - warning
        if (minimum_x <= pose[0] <= maximum_x and
                minimum_y <= pose[1] <= maximum_y):
            return None
        recovery = self.config.geofence_recovery_margin
        return (max(FIELD_BOUNDS[0] + recovery,
                    min(FIELD_BOUNDS[1] - recovery, pose[0])),
                max(FIELD_BOUNDS[2] + recovery,
                    min(FIELD_BOUNDS[3] - recovery, pose[1])))

    def _inside_recovery_box(self, pose):
        """Return whether the aircraft is back inside the deeper safe box.

        Releasing on the recovery margin rather than on proximity to a single
        point gives the hysteresis the two margins were meant to provide, and
        cannot wedge: the old release needed the aircraft within
        a fixed tolerance of one point, and any steady-state offset
        held the run there for good.
        """
        recovery = self.config.geofence_recovery_margin
        return (FIELD_BOUNDS[0] + recovery <= pose[0] <=
                FIELD_BOUNDS[1] - recovery and
                FIELD_BOUNDS[2] + recovery <= pose[1] <=
                FIELD_BOUNDS[3] - recovery)

    def _geofence_hold(self, pose, target):
        if self.interrupted_state is None and self.state != 'HOLD_UNSAFE':
            self.interrupted_state = self.state
        self.hold_reason = 'geofence_recovery'
        if self.hold_target is None:
            self.hold_target = tuple(target)
            self.hold_yaw = float(pose[3])
        return self._command(
            'HOLD_UNSAFE', self.hold_target, self.hold_yaw,
            'geofence_recovery')

    def _visible_unknown_obstacles(self, pose, obstacles):
        candidates = []
        for value in obstacles:
            shifted = value._replace(
                forward_m=(value.forward_m +
                           self.config.sensor_forward_offset),
                nearest_range_m=(value.nearest_range_m +
                                 self.config.sensor_forward_offset))
            surface = _body_to_world(pose, shifted.forward_m,
                                     shifted.left_m)
            if point_matches_known_static(
                    surface, self.config.known_static_tolerance):
                continue
            if shifted.forward_m > 0.0:
                candidates.append(shifted)
        return tuple(candidates)

    def _nearest_obstacle(self, pose, obstacles, corridor_only=True):
        candidates = self._visible_unknown_obstacles(pose, obstacles)
        if corridor_only:
            candidates = tuple(
                value for value in candidates
                if abs(value.left_m) <= self.config.lateral_trigger)
        if not candidates:
            return None
        return min(candidates, key=lambda value: value.nearest_range_m)

    def _follow_route(self, pose, reason=''):
        self._skip_passed_waypoints(pose)
        while self.waypoint_index < len(self.route):
            waypoint = self.route[self.waypoint_index]
            if self._horizontal_distance(pose, waypoint) > self.config.waypoint_tolerance:
                break
            self.waypoint_index += 1
        if self.waypoint_index >= len(self.route):
            self.state = 'REACHED'
            return self._command('REACHED', self.goal[:2], pose[3], reason)
        waypoint = self.route[self.waypoint_index]
        if 0 < self.waypoint_index < len(self.route) - 1:
            previous = self.route[self.waypoint_index - 1]
            self.route_yaw = math.atan2(waypoint[1] - previous[1],
                                        waypoint[0] - previous[0])
        else:
            self.route_yaw = math.atan2(waypoint[1] - pose[1],
                                        waypoint[0] - pose[0])
        return self._command('FOLLOW_ROUTE', waypoint, self.route_yaw, reason)

    def _skip_passed_waypoints(self, pose):
        while 0 < self.waypoint_index < len(self.route) - 1:
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
            self._clear_hold()
            return self._follow_route(pose)
        if self.dynamic_route_provider is not None:
            self._remember_obstacle(
                self._dynamic_obstacle_circle(pose, self.active_obstacle))
            try:
                self.route = tuple(self.dynamic_route_provider(
                    pose[:2], self.goal[:2], self.temporary_obstacles))
            except ValueError:
                self.state = 'HOLD_UNSAFE'
                return self._hold(
                    pose, 'dynamic_route_unreachable', remember=False)
            self.waypoint_index = 1 if len(self.route) > 1 else 0
            self.state = 'FOLLOW_ROUTE'
            self.selected_side = ''
            self.active_obstacle = None
            self._clear_hold()
            self._reset_global_candidate()
            return self._follow_route(pose, 'dynamic_route_replanned')
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
        self._remember_obstacle(
            self._dynamic_obstacle_circle(pose, self.active_obstacle))
        selected = left_candidate if side == 'LEFT' else right_candidate
        self.side_target, self.pass_target, self.rejoin_target = selected
        self.state = 'SIDESTEP'
        self._clear_hold()
        return self._command('SIDESTEP', self.side_target, self.route_yaw)

    def update(self, pose, obstacles, perception_ready, now):
        del now
        pose = tuple(float(value) for value in pose)
        if any(math.isnan(value) or math.isinf(value) for value in pose):
            self.state = 'HOLD_UNSAFE'
            self.hold_reason = 'invalid_pose'
            return PlanCommand('HOLD_UNSAFE', None, 0.0, '',
                               self.left_clearance, self.right_clearance,
                               'invalid_pose')
        if self.goal is None:
            return PlanCommand('IDLE', None, pose[3], '', 0.0, 0.0,
                               'goal_missing')
        if not perception_ready:
            return self._hold(pose, 'perception_not_ready')
        if (self.hold_reason == 'geofence_recovery' and
                self.hold_target is not None):
            if not self._inside_recovery_box(pose):
                return self._geofence_hold(pose, self.hold_target)
            self._clear_hold()
        geofence_target = self._geofence_recovery_target(pose)
        if geofence_target is not None:
            return self._geofence_hold(pose, geofence_target)
        if abs(pose[2] - self.config.altitude) > self.config.altitude_tolerance:
            return self._hold(pose, 'altitude_out_of_band')
        if self.interrupted_state is not None:
            self.state = self.interrupted_state
            self.interrupted_state = None
            self._clear_hold()

        nearest = self._nearest_obstacle(pose, obstacles)
        global_obstacle = self._nearest_obstacle(
            pose, obstacles, corridor_only=False)
        if self.state == 'FOLLOW_ROUTE':
            route_command = self._follow_route(pose)
            if route_command.state == 'REACHED':
                self._clear_hold()
                return route_command
            if (nearest is not None and
                    nearest.nearest_range_m < self.config.trigger_range and
                    self._local_brake_required(pose, nearest)):
                self._reset_global_candidate()
                self.active_obstacle = nearest
                self.state = 'BRAKE'
                return self._command(
                    'BRAKE', self._lock_hold(pose), self.hold_yaw)
            dynamic_command = self._maybe_replan_for_far_obstacle(
                pose, global_obstacle)
            if dynamic_command is not None:
                self._clear_hold()
                return dynamic_command
            if abs(angle_difference(pose[3], route_command.target_yaw)) > \
                    self.config.yaw_alignment_tolerance:
                self._clear_hold()
                return self._command(
                    'FOLLOW_ROUTE', route_command.target,
                    route_command.target_yaw,
                    'aligning_route_yaw')
            self._clear_hold()
            return route_command

        if self.state == 'BRAKE':
            self.state = 'OBSERVE'
            self.observation_count = 0
            return self._command(
                'OBSERVE', self._lock_hold(pose), self.hold_yaw)

        if self.state == 'OBSERVE':
            if nearest is not None:
                self.active_obstacle = nearest
            self.observation_count += 1
            if self.observation_count >= self.config.observation_frames:
                self.state = 'SELECT_SIDE'
                return self._command(
                    'SELECT_SIDE', self._lock_hold(pose), self.hold_yaw)
            return self._command(
                'OBSERVE', self._lock_hold(pose), self.hold_yaw)

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
            if self.hold_reason == 'dynamic_route_unreachable':
                recovered = self._retry_blocked_route(pose)
                if recovered is not None:
                    return recovered
                return self._hold(
                    pose, 'dynamic_route_unreachable', remember=False)
            if nearest is None:
                return self._hold(
                    pose, 'obstacle_temporarily_unseen', remember=False)
            if nearest.nearest_range_m >= self.config.trigger_range:
                self.state = 'FOLLOW_ROUTE'
                self.selected_side = ''
                self.active_obstacle = None
                self._clear_hold()
                return self._follow_route(pose)
            self.active_obstacle = nearest
            self.state = 'SELECT_SIDE'
            return self._select_side(pose)
        if self.state == 'REACHED':
            return self._command('REACHED', self.goal[:2], pose[3])
        return self._hold(pose, 'invalid_planner_state')
