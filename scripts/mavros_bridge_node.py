#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from firefighting_mission.mavros_bridge import command_actions, flu_to_enu


def quaternion_yaw(orientation):
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny, cosy)


class MavrosBridgeNode(object):
    """Adapts the package's established XTDrone topics to one MAVROS instance."""

    def __init__(self):
        self.prefix = rospy.get_param('~mavros_prefix', '/iris_0/mavros').rstrip('/')
        self.pose = None
        self.velocity = Twist()
        self.velocity_pub = rospy.Publisher(
            self.prefix + '/setpoint_velocity/cmd_vel_unstamped', Twist,
            queue_size=1)
        self.arm = rospy.ServiceProxy(self.prefix + '/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy(self.prefix + '/set_mode', SetMode)
        rospy.Subscriber('/xtdrone/iris_0/cmd', String, self._command)
        rospy.Subscriber('/xtdrone/iris_0/cmd_vel_flu', Twist, self._velocity)
        rospy.Subscriber(self.prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._publish_velocity)

    def _pose(self, message):
        self.pose = message

    def _velocity(self, message):
        self.velocity = message

    def _publish_velocity(self, _event):
        output = Twist()
        yaw = quaternion_yaw(self.pose.pose.orientation) if self.pose else 0.0
        output.linear.x, output.linear.y = flu_to_enu(
            self.velocity.linear.x, self.velocity.linear.y, yaw)
        output.linear.z = self.velocity.linear.z
        output.angular.z = self.velocity.angular.z
        self.velocity_pub.publish(output)

    def _command(self, message):
        for action in command_actions(message.data):
            try:
                if action == 'ARM':
                    self.arm(True)
                elif action == 'DISARM':
                    self.arm(False)
                else:
                    self.set_mode(0, action)
            except rospy.ServiceException as error:
                rospy.logwarn('MAVROS %s request failed: %s', action, error)


if __name__ == '__main__':
    rospy.init_node('firefighting_mavros_bridge')
    MavrosBridgeNode()
    rospy.spin()
