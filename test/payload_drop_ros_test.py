#!/usr/bin/env python
from __future__ import division, print_function

import unittest

import rospy
import rostest
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool


class PhysicalPayloadDropTest(unittest.TestCase):
    def setUp(self):
        self.states = None
        self.publisher = rospy.Publisher('/fire_iris/drop_fire', Bool,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._states)

    def _states(self, message):
        self.states = message

    def _z(self, link_name):
        if self.states is None or link_name not in self.states.name:
            return None
        return self.states.pose[self.states.name.index(link_name)].position.z

    def test_fire_payload_falls_while_rescue_payload_remains_attached(self):
        fire_name = 'payload_test::fire_payload_link'
        rescue_name = 'payload_test::rescue_payload_link'
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while (self._z(fire_name) is None or self.publisher.get_num_connections() == 0):
            if rospy.Time.now() > deadline:
                self.fail('payload fixture or plugin did not become ready')
            rospy.sleep(0.05)

        initial_fire = self._z(fire_name)
        initial_rescue = self._z(rescue_name)
        self.publisher.publish(True)
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while self._z(fire_name) > initial_fire - 0.20 and rospy.Time.now() < deadline:
            self.publisher.publish(True)
            rospy.sleep(0.05)

        self.assertLess(self._z(fire_name), initial_fire - 0.20)
        self.assertAlmostEqual(initial_rescue, self._z(rescue_name), delta=0.03)


if __name__ == '__main__':
    rospy.init_node('physical_payload_drop_test')
    rostest.rosrun('firefighting_mission', 'physical_payload_drop',
                   PhysicalPayloadDropTest)
