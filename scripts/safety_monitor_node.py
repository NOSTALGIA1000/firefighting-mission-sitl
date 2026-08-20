#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from firefighting_mission.safety import SafetyMonitor


def quaternion_roll_pitch(orientation):
    sinr = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
    cosr = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


class SafetyMonitorNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix',
                                              '/iris_0/mavros').rstrip('/')
        self.monitor = SafetyMonitor()
        self.pose = None
        self.pose_stamp = rospy.Time(0)
        self.scan_stamp = rospy.Time(0)
        self.minimum_obstacle = float('inf')
        self.status_pub = rospy.Publisher('/fire_mission/safety_status', String,
                                          queue_size=1, latch=True)
        self.override_pub = rospy.Publisher('/fire_mission/safety_override', Twist,
                                            queue_size=1)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(rospy.get_param('~scan_topic', '/scan'), LaserScan, self._scan)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)

    def _pose(self, message):
        self.pose = message
        self.pose_stamp = rospy.Time.now()

    def _scan(self, message):
        valid = [value for value in message.ranges
                 if not math.isnan(value) and not math.isinf(value) and value > 0.02]
        self.minimum_obstacle = min(valid) if valid else float('inf')
        self.scan_stamp = rospy.Time.now()

    def _tick(self, _event):
        now = rospy.Time.now()
        if self.pose is None:
            self.status_pub.publish('LAND:pose_missing')
            return
        position = self.pose.pose.position
        roll, pitch = quaternion_roll_pitch(self.pose.pose.orientation)
        boundary_margin = min(position.x + 0.65, 3.35 - position.x,
                              position.y + 3.35, 0.65 - position.y)
        status = self.monitor.evaluate(
            (now - self.pose_stamp).to_sec(), (now - self.scan_stamp).to_sec(),
            roll, pitch, position.z, self.minimum_obstacle, boundary_margin)
        self.status_pub.publish('%s:%s' % (status.action, status.reason))
        if status.action != 'CLEAR':
            override = Twist()
            if status.action == 'RETREAT':
                override.linear.x = -0.15
            elif status.action == 'LAND':
                override.linear.z = -0.20
            self.override_pub.publish(override)


if __name__ == '__main__':
    rospy.init_node('firefighting_safety_monitor')
    SafetyMonitorNode()
    rospy.spin()
