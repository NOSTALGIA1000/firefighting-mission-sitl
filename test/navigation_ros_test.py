#!/usr/bin/env python
from __future__ import division, print_function

import math
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class NavigationRosContractTest(unittest.TestCase):
    def setUp(self):
        self.command = None
        self.status = None
        self.goal_pub = rospy.Publisher('/fire_mission/goal', PoseStamped,
                                        queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('/iris_0/mavros/local_position/pose',
                                        PoseStamped, queue_size=1, latch=True)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1, latch=True)
        rospy.Subscriber('/xtdrone/iris_0/cmd_vel_flu', Twist, self._command)
        rospy.Subscriber('/fire_mission/nav_status', String, self._status)

    def _command(self, message):
        self.command = message

    def _status(self, message):
        self.status = message.data

    def test_front_obstacle_produces_left_avoidance_command(self):
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while (self.goal_pub.get_num_connections() == 0 or
               self.pose_pub.get_num_connections() == 0 or
               self.scan_pub.get_num_connections() == 0) and rospy.Time.now() < deadline:
            rospy.sleep(0.05)

        goal = PoseStamped()
        goal.pose.position.x = 2.0
        goal.pose.position.z = 1.3
        pose = PoseStamped()
        pose.pose.position.z = 1.3
        pose.pose.orientation.w = 1.0
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 180.0
        scan.ranges = [5.0] * 360
        for index in range(160, 201):
            scan.ranges[index] = 0.50
        for index in range(201, 251):
            scan.ranges[index] = 1.40
        for index in range(110, 160):
            scan.ranges[index] = 0.60

        while self.command is None and rospy.Time.now() < deadline:
            self.goal_pub.publish(goal)
            self.pose_pub.publish(pose)
            self.scan_pub.publish(scan)
            rospy.sleep(0.05)

        self.assertIsNotNone(self.command)
        self.assertGreater(self.command.linear.y, 0.0)
        self.assertLessEqual(self.command.linear.x, 0.08)
        self.assertEqual('AVOIDING', self.status)


if __name__ == '__main__':
    rospy.init_node('navigation_ros_contract_test')
    rostest.rosrun('firefighting_mission', 'navigation_ros_contract',
                   NavigationRosContractTest)
