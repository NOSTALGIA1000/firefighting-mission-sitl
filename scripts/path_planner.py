#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from firefighting_mission.msg import (AvoidanceStatus, ObstacleArray)
from firefighting_mission.path_planner import (
    VisualPathPlanner, VisualPlannerConfig, ramp_setpoint,
    setpoint_stream_target)
from firefighting_mission.stereo_obstacles import ObstacleClusterData


def quaternion_yaw(orientation):
    numerator = 2.0 * (orientation.w * orientation.z +
                       orientation.x * orientation.y)
    denominator = 1.0 - 2.0 * (orientation.y * orientation.y +
                               orientation.z * orientation.z)
    return math.atan2(numerator, denominator)


class PathPlannerNode(object):
    def __init__(self):
        self.planner = VisualPathPlanner(VisualPlannerConfig(
            altitude=rospy.get_param('~transit_altitude', 1.25),
            altitude_tolerance=rospy.get_param('~altitude_tolerance', 0.15),
            yaw_alignment_tolerance=rospy.get_param(
                '~yaw_alignment_tolerance', 0.20),
            # Warning sets when the hold triggers and when it releases;
            # recovery only sets how deep the hold aims.  Deep is protective,
            # so it stays at 0.65 even though the mission points sit on that
            # boundary - the release uses the warning box instead.
            geofence_warning_margin=rospy.get_param(
                '~geofence_warning_margin', 0.45),
            geofence_recovery_margin=rospy.get_param(
                '~geofence_recovery_margin', 0.65),
            minimum_corridor=rospy.get_param('~minimum_corridor', 0.90),
            trigger_range=rospy.get_param('~trigger_range', 0.85),
            sensor_forward_offset=rospy.get_param(
                '~sensor_forward_offset', 0.32)))
        self.pose = None
        self.map_pose = None
        self.use_gazebo_ground_truth = rospy.get_param(
            '~use_gazebo_ground_truth', False)
        self.gazebo_model_name = rospy.get_param(
            '~gazebo_model_name', 'iris_0')
        self.requested_goal = None
        self.active_goal = None
        self.obstacles = ()
        self.perception_ready = False
        self.perception_reason = 'depth_stale'
        self.output_setpoint = None
        self.last_output_time = None
        self.target_pub = rospy.Publisher(
            '/fire_mission/path_setpoint', PoseStamped,
            queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            '/fire_mission/path_status', String,
            queue_size=1, latch=True)
        self.avoidance_pub = rospy.Publisher(
            '/fire_mission/avoidance_status', AvoidanceStatus,
            queue_size=1, latch=True)
        prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        rospy.Subscriber(prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber('/fire_mission/goal', PoseStamped, self._goal)
        rospy.Subscriber('/fire_mission/point_goal', PoseStamped, self._goal)
        rospy.Subscriber('/fire_mission/obstacles', ObstacleArray,
                         self._obstacles)
        if self.use_gazebo_ground_truth:
            rospy.Subscriber('/gazebo/model_states', ModelStates,
                             self._model_states)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    @staticmethod
    def _point(message):
        point = message.pose.position
        return (point.x, point.y, point.z)

    def _pose(self, message):
        self.pose = message

    def _goal(self, message):
        self.requested_goal = self._point(message)

    def _obstacles(self, message):
        self.obstacles = tuple(ObstacleClusterData(
            value.forward_m, value.left_m, value.nearest_range_m,
            value.left_edge_m, value.right_edge_m, value.confidence)
            for value in message.obstacles)
        self.perception_ready = bool(message.ready)
        self.perception_reason = message.reason

    def _model_states(self, message):
        try:
            index = message.name.index(self.gazebo_model_name)
        except ValueError:
            return
        value = message.pose[index]
        self.map_pose = (
            value.position.x, value.position.y, 0.0,
            quaternion_yaw(value.orientation))

    @staticmethod
    def _pose_tuple(message):
        point = message.pose.position
        return (point.x, point.y, point.z,
                quaternion_yaw(message.pose.orientation))

    def _publish_target(self, command, planning_pose, now):
        dt = (0.05 if self.last_output_time is None else
              max(0.0, now - self.last_output_time))
        desired = (command.target[0], command.target[1], command.target[2],
                   command.target_yaw)
        turning = command.reason == 'aligning_route_yaw'
        locked_xy = command.state in (
            'BRAKE', 'OBSERVE', 'SELECT_SIDE', 'HOLD_UNSAFE', 'REACHED')
        cruise_speed = rospy.get_param('~maximum_horizontal_speed', 0.18)
        turning_speed = rospy.get_param('~maximum_turning_speed', 0.12)
        self.output_setpoint = ramp_setpoint(
            self.output_setpoint, desired, planning_pose, dt,
            horizontal_speed=(turning_speed if turning else cruise_speed),
            maximum_lead=(None if locked_xy else rospy.get_param(
                '~maximum_setpoint_lead', 0.25)),
            yaw_rate=rospy.get_param('~maximum_yaw_rate', 0.35),
            lock_xy=locked_xy)
        self.last_output_time = now
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.pose.position.x = self.output_setpoint[0]
        target.pose.position.y = self.output_setpoint[1]
        target.pose.position.z = self.output_setpoint[2]
        target.pose.orientation.z = math.sin(self.output_setpoint[3] / 2.0)
        target.pose.orientation.w = math.cos(self.output_setpoint[3] / 2.0)
        self.target_pub.publish(target)

    def _publish_status(self, command):
        status = AvoidanceStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = 'map'
        status.state = command.state
        status.selected_side = command.selected_side
        status.left_clearance_m = command.left_clearance
        status.right_clearance_m = command.right_clearance
        status.reason = command.reason or self.perception_reason
        if command.target is not None:
            status.target.x, status.target.y, status.target.z = command.target
        status.target_yaw = command.target_yaw
        self.avoidance_pub.publish(status)
        self.status_pub.publish(command.state)

    def _tick(self, _event):
        if (self.pose is None or self.requested_goal is None or
                (self.use_gazebo_ground_truth and self.map_pose is None)):
            self.status_pub.publish('IDLE')
            return
        local_pose = self._pose_tuple(self.pose)
        planning_pose = local_pose
        if self.use_gazebo_ground_truth:
            planning_pose = (
                self.map_pose[0], self.map_pose[1], local_pose[2],
                self.map_pose[3])
        if self.requested_goal != self.active_goal:
            try:
                self.planner.set_goal(self.requested_goal, planning_pose)
            except ValueError as error:
                rospy.logerr_throttle(2.0, 'route rejected: %s', error)
                self.status_pub.publish('HOLD_UNSAFE')
                return
            self.active_goal = self.requested_goal
        now = rospy.Time.now().to_sec()
        command = self.planner.update(
            planning_pose, self.obstacles, self.perception_ready,
            now)
        target = setpoint_stream_target(command.target, self.output_setpoint)
        if target is not None:
            self._publish_target(command._replace(target=target),
                                 planning_pose, now)
        self._publish_status(command)


if __name__ == '__main__':
    rospy.init_node('firefighting_path_planner')
    PathPlannerNode()
    rospy.spin()
