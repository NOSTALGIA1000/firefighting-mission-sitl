#!/usr/bin/env python
from __future__ import division, print_function

import json
import math
import os
import unittest

import rospkg
import rospy
import rostest
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from firefighting_mission.msg import AvoidanceStatus, ObstacleArray
from firefighting_mission.orchestration import contacts_indicate_collision


class VisualAvoidanceSmokeTest(unittest.TestCase):
    def setUp(self):
        self.seed = int(rospy.get_param('~seed', 1))
        self.controller_state = ''
        self.path_state = ''
        self.avoidance_reason = ''
        self.selected_side = ''
        self.last_pose = None
        self.last_yaw = None
        self.clearances = None
        self.obstacles = []
        self.contact_pairs = []
        self.event_trace = []
        self.last_avoidance_state = ''
        self.states = set()
        self.collision = False
        self.transit = False
        self.altitudes = []
        self.goal_pub = rospy.Publisher('/fire_mission/point_goal', PoseStamped,
                                        queue_size=1, latch=True)
        rospy.Subscriber('/competition_main/state', String, self._controller)
        rospy.Subscriber('/fire_mission/path_status', String, self._path)
        rospy.Subscriber('/fire_mission/avoidance_status', AvoidanceStatus,
                         self._avoidance)
        rospy.Subscriber('/fire_mission/contacts', ContactsState, self._contacts)
        rospy.Subscriber('/fire_mission/obstacles', ObstacleArray,
                         self._obstacles)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose)

    def _controller(self, message):
        self.controller_state = message.data

    def _path(self, message):
        self.path_state = message.data

    def _avoidance(self, message):
        self.avoidance_reason = message.reason
        self.selected_side = message.selected_side
        self.clearances = (message.left_clearance_m,
                           message.right_clearance_m)
        if message.state != self.last_avoidance_state:
            self.event_trace.append({
                'state': message.state,
                'reason': message.reason,
                'selected_side': message.selected_side,
                'clearances': self.clearances,
                'pose': self.last_pose,
                'yaw': self.last_yaw,
                'obstacles': list(self.obstacles),
            })
            self.last_avoidance_state = message.state
        if self.transit:
            self.states.add(message.state)

    def _contacts(self, message):
        self.collision = (self.collision or
                          contacts_indicate_collision(message.states))
        for state in message.states:
            pair = (state.collision1_name, state.collision2_name)
            if pair not in self.contact_pairs:
                self.contact_pairs.append(pair)

    def _obstacles(self, message):
        self.obstacles = [
            (value.forward_m, value.left_m, value.nearest_range_m,
             value.left_edge_m, value.right_edge_m, value.confidence)
            for value in message.obstacles]

    def _pose(self, message):
        point = message.pose.position
        self.last_pose = (point.x, point.y, point.z)
        orientation = message.pose.orientation
        self.last_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z))
        if self.transit:
            self.altitudes.append(point.z)

    @staticmethod
    def _goal(x_value, y_value):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x = x_value
        message.pose.position.y = y_value
        message.pose.position.z = 1.2
        message.pose.orientation.w = 1.0
        return message

    def _wait(self, predicate, timeout, label):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail('timeout_waiting_for_%s' % label)

    def _drive(self, goal, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        left_previous_reached = self.path_state != 'REACHED'
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            if self.path_state != 'REACHED':
                left_previous_reached = True
            if left_previous_reached and self.path_state == 'REACHED':
                return
            rate.sleep()
        self.fail('goal_not_reached state=%s reason=%s pose=%r controller=%s' %
                  (self.path_state, self.avoidance_reason, self.last_pose,
                   self.controller_state))

    def tearDown(self):
        package = rospkg.RosPack().get_path('firefighting_mission')
        output_dir = os.path.join(package, 'artifacts', 'avoidance_matrix',
                                  str(self.seed))
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        evidence = {
            'seed': self.seed,
            'states': sorted(self.states),
            'collision': self.collision,
            'minimum_transit_altitude': (min(self.altitudes) if
                                         self.altitudes else None),
            'maximum_transit_altitude': (max(self.altitudes) if
                                         self.altitudes else None),
            'reached_goal': self.path_state == 'REACHED',
            'path_state': self.path_state,
            'avoidance_reason': self.avoidance_reason,
            'selected_side': self.selected_side,
            'last_pose': self.last_pose,
            'controller_state': self.controller_state,
            'last_yaw': self.last_yaw,
            'clearances': self.clearances,
            'obstacles': self.obstacles,
            'contact_pairs': self.contact_pairs,
            'event_trace': self.event_trace,
        }
        with open(os.path.join(output_dir, 'smoke.json'), 'w') as handle:
            json.dump(evidence, handle, indent=2, sort_keys=True)

    def test_constant_altitude_visual_pass(self):
        self._wait(lambda: self.controller_state == 'HOVER', 35.0,
                   'offboard_hover')
        self.states.clear()
        self.altitudes = []
        self.transit = True
        self._drive(self._goal(1.50, -1.45), 55.0)
        self.transit = False

        required = set(('BRAKE', 'OBSERVE', 'SELECT_SIDE', 'SIDESTEP',
                        'PASS', 'REJOIN'))
        self.assertTrue(self.states.issuperset(required), sorted(self.states))
        self.assertFalse(self.collision)
        self.assertTrue(self.altitudes)
        self.assertLessEqual(max(self.altitudes), 1.30)
        self.assertGreaterEqual(min(self.altitudes), 1.10)
        self.assertEqual('REACHED', self.path_state)


if __name__ == '__main__':
    rospy.init_node('visual_avoidance_smoke_test')
    rostest.rosrun('firefighting_mission', 'visual_avoidance_smoke',
                   VisualAvoidanceSmokeTest)
