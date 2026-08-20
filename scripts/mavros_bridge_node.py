#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from firefighting_mission.mavros_bridge import (arm_actions, command_actions,
                                                 raw_velocity_setpoint,
                                                 RAW_SETPOINT_RATE_HZ)


class MavrosBridgeNode(object):
    """Adapts the package's established XTDrone topics to one MAVROS instance."""

    def __init__(self):
        self.prefix = rospy.get_param('~mavros_prefix', '/iris_0/mavros').rstrip('/')
        self.target = self._raw_target(0.0, 0.0, 0.0, 0.0)
        self.setpoint_count = 0
        self.arm_requested = False
        self.offboard_sent = False
        self.arm_sent = False
        self.velocity_pub = rospy.Publisher(
            self.prefix + '/setpoint_raw/local', PositionTarget,
            queue_size=1)
        self.arm = rospy.ServiceProxy(self.prefix + '/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy(self.prefix + '/set_mode', SetMode)
        rospy.Subscriber('/xtdrone/iris_0/cmd', String, self._command)
        rospy.Subscriber('/xtdrone/iris_0/cmd_vel_flu', Twist, self._velocity)
        self.timer = rospy.Timer(rospy.Duration(1.0 / RAW_SETPOINT_RATE_HZ),
                                 self._publish_velocity)

    @staticmethod
    def _raw_target(forward, left, upward, yaw_rate):
        frame, mask, forward, left, upward, yaw_rate = raw_velocity_setpoint(
            forward, left, upward, yaw_rate)
        target = PositionTarget()
        target.coordinate_frame = frame
        target.type_mask = mask
        target.velocity.x = forward
        target.velocity.y = left
        target.velocity.z = upward
        target.yaw_rate = yaw_rate
        return target

    def _velocity(self, message):
        self.target = self._raw_target(message.linear.x, message.linear.y,
                                       message.linear.z, message.angular.z)

    def _publish_velocity(self, _event):
        self.velocity_pub.publish(self.target)
        self.setpoint_count += 1
        if not self.arm_requested:
            return

        for action in arm_actions(self.setpoint_count, self.offboard_sent):
            try:
                if action == 'OFFBOARD':
                    response = self.set_mode(0, action)
                    self.offboard_sent = bool(response.mode_sent)
                elif action == 'ARM' and not self.arm_sent:
                    response = self.arm(True)
                    self.arm_sent = bool(response.success)
            except rospy.ServiceException as error:
                rospy.logwarn('MAVROS %s request failed: %s', action, error)

    def _command(self, message):
        if message.data.strip().upper() == 'ARM':
            self.arm_requested = True
            return
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
