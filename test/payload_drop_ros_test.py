#!/usr/bin/env python
from __future__ import division, print_function

import unittest

import rospy
import rostest
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool

from firefighting_mission.srv import DropSupply


class PhysicalPayloadDropTest(unittest.TestCase):
    def setUp(self):
        self.states = None
        self.rescue_topic = rospy.Publisher('/fire_iris/drop_rescue', Bool,
                                            queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._states)

    def _states(self, message):
        self.states = message

    def _z(self, link_name):
        if self.states is None or link_name not in self.states.name:
            return None
        return self.states.pose[self.states.name.index(link_name)].position.z

    def test_service_release_duplicate_rejection_and_legacy_topic(self):
        fire_name = 'payload_test::fire_payload_link'
        rescue_name = 'payload_test::rescue_payload_link'
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while self._z(fire_name) is None:
            if rospy.Time.now() > deadline:
                self.fail('payload fixture or plugin did not become ready')
            rospy.sleep(0.05)

        rospy.wait_for_service('/fire_iris/drop_supply', timeout=10.0)
        drop = rospy.ServiceProxy('/fire_iris/drop_supply', DropSupply)
        initial_fire = self._z(fire_name)
        initial_rescue = self._z(rescue_name)
        released = drop(1)
        self.assertTrue(released.success)
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while self._z(fire_name) > initial_fire - 0.20 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)

        self.assertLess(self._z(fire_name), initial_fire - 0.20)
        self.assertAlmostEqual(initial_rescue, self._z(rescue_name), delta=0.03)

        duplicate = drop(1)
        self.assertFalse(duplicate.success)
        self.assertEqual('already_released', duplicate.reason)

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.rescue_topic.get_num_connections() == 0:
            if rospy.Time.now() > deadline:
                self.fail('legacy rescue topic did not connect')
            rospy.sleep(0.05)
        self.rescue_topic.publish(True)
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while (self._z(rescue_name) > initial_rescue - 0.20 and
               rospy.Time.now() < deadline):
            self.rescue_topic.publish(True)
            rospy.sleep(0.05)
        self.assertLess(self._z(rescue_name), initial_rescue - 0.20)


if __name__ == '__main__':
    rospy.init_node('physical_payload_drop_test')
    rostest.rosrun('firefighting_mission', 'physical_payload_drop',
                   PhysicalPayloadDropTest)
