#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from firefighting_mission.msg import (AvoidanceStatus, ObstacleArray)
from firefighting_mission.path_planner import (
    VisualPathPlanner, VisualPlannerConfig)
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
            altitude=rospy.get_param('~transit_altitude', 1.20),
            minimum_corridor=rospy.get_param('~minimum_corridor', 0.90),
            trigger_range=rospy.get_param('~trigger_range', 1.00)))
        self.pose = None
        self.requested_goal = None
        self.active_goal = None
        self.obstacles = ()
        self.perception_ready = False
        self.perception_reason = 'depth_stale'
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

    @staticmethod
    def _pose_tuple(message):
        point = message.pose.position
        return (point.x, point.y, point.z,
                quaternion_yaw(message.pose.orientation))

    def _publish_target(self, command):
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.pose.position.x = command.target[0]
        target.pose.position.y = command.target[1]
        target.pose.position.z = command.target[2]
        target.pose.orientation.z = math.sin(command.target_yaw / 2.0)
        target.pose.orientation.w = math.cos(command.target_yaw / 2.0)
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
        if self.pose is None or self.requested_goal is None:
            self.status_pub.publish('IDLE')
            return
        pose = self._pose_tuple(self.pose)
        if self.requested_goal != self.active_goal:
            try:
                self.planner.set_goal(self.requested_goal, pose)
            except ValueError as error:
                rospy.logerr_throttle(2.0, 'route rejected: %s', error)
                self.status_pub.publish('HOLD_UNSAFE')
                return
            self.active_goal = self.requested_goal
        command = self.planner.update(
            pose, self.obstacles, self.perception_ready,
            rospy.Time.now().to_sec())
        if command.target is not None:
            self._publish_target(command)
        self._publish_status(command)


if __name__ == '__main__':
    rospy.init_node('firefighting_path_planner')
    PathPlannerNode()
    rospy.spin()
