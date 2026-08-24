#!/usr/bin/env python
from __future__ import division, print_function

import math
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from firefighting_mission.msg import (AvoidanceStatus, ObstacleArray)


class PathPlannerRosContractTest(unittest.TestCase):
    def setUp(self):
        self.status = None
        self.target = None
        self.avoidance = None
        self.pose_pub = rospy.Publisher(
            '/test_mavros/local_position/pose', PoseStamped,
            queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(
            '/fire_mission/goal', PoseStamped, queue_size=1, latch=True)
        self.obstacle_pub = rospy.Publisher(
            '/fire_mission/obstacles', ObstacleArray, queue_size=1, latch=True)
        rospy.Subscriber('/fire_mission/path_status', String, self._status)
        rospy.Subscriber('/fire_mission/path_setpoint', PoseStamped,
                         self._target)
        rospy.Subscriber('/fire_mission/avoidance_status', AvoidanceStatus,
                         self._avoidance)

    def _status(self, message):
        self.status = message.data

    def _target(self, message):
        self.target = message

    def _avoidance(self, message):
        self.avoidance = message

    @staticmethod
    def _pose(x, y, z):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        return message

    def _drive(self, pose, goal, expected_status):
        ready = ObstacleArray()
        ready.header.stamp = rospy.Time.now()
        ready.ready = True
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while rospy.Time.now() < deadline:
            self.pose_pub.publish(pose)
            self.goal_pub.publish(goal)
            ready.header.stamp = rospy.Time.now()
            self.obstacle_pub.publish(ready)
            if (self.status == expected_status and self.target is not None and
                    self.avoidance is not None):
                return
            rospy.sleep(0.05)
        self.assertEqual(expected_status, self.status)
        self.assertIsNotNone(self.target)
        self.assertIsNotNone(self.avoidance)

    def test_constant_altitude_follow_and_reached_contract(self):
        goal = self._pose(0.0, -1.0, 1.2)
        self._drive(self._pose(0.0, 0.0, 1.2), goal, 'FOLLOW_ROUTE')

        target = self.target.pose.position
        orientation = self.target.pose.orientation
        self.assertEqual(1.2, target.z)
        self.assertNotEqual(2.3, target.z)
        self.assertAlmostEqual(1.0, math.hypot(orientation.z, orientation.w),
                               places=5)
        self.assertEqual('FOLLOW_ROUTE', self.avoidance.state)

        self._drive(self._pose(0.0, -1.0, 1.2), goal, 'REACHED')
        target = self.target.pose.position
        self.assertEqual((0.0, -1.0, 1.2),
                         (target.x, target.y, target.z))


if __name__ == '__main__':
    rospy.init_node('path_planner_ros_contract_test')
    rostest.rosrun('firefighting_mission', 'path_planner_ros_contract',
                   PathPlannerRosContractTest)
