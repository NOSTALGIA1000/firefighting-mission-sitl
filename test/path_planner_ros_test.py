#!/usr/bin/env python
from __future__ import division, print_function

import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class PathPlannerRosContractTest(unittest.TestCase):
    def setUp(self):
        self.status = None
        self.target = None
        self.pose_pub = rospy.Publisher(
            '/test_mavros/local_position/pose', PoseStamped,
            queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(
            '/fire_mission/point_goal', PoseStamped,
            queue_size=1, latch=True)
        rospy.Subscriber('/fire_mission/path_status', String, self._status)
        rospy.Subscriber('/fire_mission/path_setpoint', PoseStamped,
                         self._target)

    def _status(self, message):
        self.status = message.data

    def _target(self, message):
        self.target = message

    @staticmethod
    def _pose(x, y, z):
        message = PoseStamped()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        return message

    def _drive(self, pose, expected_status, expected_target, goal=None):
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while rospy.Time.now() < deadline:
            self.pose_pub.publish(pose)
            if goal is not None:
                self.goal_pub.publish(goal)
            if self.status == expected_status and self.target is not None:
                target = self.target.pose.position
                if (target.x, target.y, target.z) == expected_target:
                    return target
            rospy.sleep(0.05)
        self.assertEqual(expected_status, self.status)
        self.assertIsNotNone(self.target)
        target = self.target.pose.position
        self.assertEqual(expected_target, (target.x, target.y, target.z))
        return target

    def test_ordered_climb_cruise_descend_and_reached_targets(self):
        goal = self._pose(2.0, -1.0, 1.2)
        target = self._drive(self._pose(0.0, 0.0, 1.2), 'CLIMB',
                             (0.0, 0.0, 2.3), goal)
        self.assertEqual((0.0, 0.0, 2.3),
                         (target.x, target.y, target.z))

        target = self._drive(self._pose(0.0, 0.0, 2.3), 'CRUISE',
                             (2.0, -1.0, 2.3))
        self.assertEqual((2.0, -1.0, 2.3),
                         (target.x, target.y, target.z))

        target = self._drive(self._pose(2.0, -1.0, 2.3), 'DESCEND',
                             (2.0, -1.0, 1.2))
        self.assertEqual((2.0, -1.0, 1.2),
                         (target.x, target.y, target.z))

        target = self._drive(self._pose(2.0, -1.0, 1.2), 'REACHED',
                             (2.0, -1.0, 1.2))
        self.assertEqual((2.0, -1.0, 1.2),
                         (target.x, target.y, target.z))


if __name__ == '__main__':
    rospy.init_node('path_planner_ros_contract_test')
    rostest.rosrun('firefighting_mission', 'path_planner_ros_contract',
                   PathPlannerRosContractTest)
