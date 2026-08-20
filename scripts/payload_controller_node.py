#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, UInt8

from firefighting_mission.msg import DropResult
from firefighting_mission.payload import PayloadController


class PayloadControllerNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix',
                                              '/iris_0/mavros').rstrip('/')
        self.controller = PayloadController()
        self.pose = None
        self.velocity = None
        self.aligned = False
        self.fire_pub = rospy.Publisher('/fire_iris/drop_fire', Bool,
                                        queue_size=1, latch=True)
        self.rescue_pub = rospy.Publisher('/fire_iris/drop_rescue', Bool,
                                          queue_size=1, latch=True)
        self.result_pub = rospy.Publisher('/fire_mission/drop_result', DropResult,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/fire_mission/drop_request', UInt8, self._request)
        rospy.Subscriber('/fire_mission/aligned', Bool, self._aligned)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(self.mavros_prefix + '/local_position/velocity_local',
                         TwistStamped, self._velocity)

    def _pose(self, message):
        self.pose = message

    def _velocity(self, message):
        self.velocity = message

    def _aligned(self, message):
        self.aligned = message.data

    def _request(self, message):
        result = DropResult()
        result.header.stamp = rospy.Time.now()
        result.channel = message.data
        if self.pose is None or self.velocity is None:
            result.reason = 'flight_state_missing'
            self.result_pub.publish(result)
            return
        linear = self.velocity.twist.linear
        speed = math.hypot(linear.x, linear.y)
        decision = self.controller.request(
            message.data, self.aligned, speed, self.pose.pose.position.z)
        result.released = decision.accepted
        result.reason = decision.reason
        result.release_position = self.pose.pose.position
        self.result_pub.publish(result)
        if decision.accepted:
            publisher = self.fire_pub if message.data == 1 else self.rescue_pub
            publisher.publish(True)


if __name__ == '__main__':
    rospy.init_node('firefighting_payload_controller')
    PayloadControllerNode()
    rospy.spin()
