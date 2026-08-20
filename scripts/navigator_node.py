#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from firefighting_mission.navigation import (NavigationConfig, Navigator,
                                              sector_distances)


def quaternion_yaw(orientation):
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny, cosy)


class NavigatorNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix',
                                              '/iris_0/mavros').rstrip('/')
        self.navigator = Navigator(NavigationConfig(
            max_xy=rospy.get_param('~max_xy', 0.55),
            max_z=rospy.get_param('~max_z', 0.30),
        ))
        self.goal = None
        self.pose = None
        self.sectors = (float('inf'),) * 3
        self.command_pub = rospy.Publisher(
            '/xtdrone/iris_0/cmd_vel_flu', Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/fire_mission/nav_status', String, queue_size=1, latch=True)
        rospy.Subscriber('/fire_mission/goal', PoseStamped, self._goal)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(rospy.get_param('~scan_topic', '/scan'), LaserScan, self._scan)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    def _goal(self, message):
        self.goal = message

    def _pose(self, message):
        self.pose = message

    def _scan(self, message):
        self.sectors = sector_distances(message.ranges, message.angle_min,
                                        message.angle_increment)

    def _tick(self, _event):
        if self.goal is None or self.pose is None:
            self.status_pub.publish('STALE')
            return
        goal = self.goal.pose.position
        pose = self.pose.pose.position
        command = self.navigator.compute(
            (goal.x, goal.y, goal.z), (pose.x, pose.y, pose.z),
            quaternion_yaw(self.pose.pose.orientation), self.sectors)
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = command.x, command.y, command.z
        message.angular.z = command.yaw_rate
        self.command_pub.publish(message)
        self.status_pub.publish(command.status)


if __name__ == '__main__':
    rospy.init_node('firefighting_navigator')
    NavigatorNode()
    rospy.spin()
