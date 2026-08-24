#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

from firefighting_mission.srv import DropSupply, DropSupplyResponse
from firefighting_mission.supply_drop import SupplyDropController


class SupplyDropNode(object):
    def __init__(self):
        self.controller = SupplyDropController()
        self.pose = None
        self.velocity = None
        self.aligned = False
        prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        self.low_level = rospy.ServiceProxy('/fire_iris/drop_supply', DropSupply)
        self.service = rospy.Service('/fire_mission/drop_supply', DropSupply,
                                     self._drop)
        rospy.Subscriber(prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(prefix + '/local_position/velocity_local', TwistStamped,
                         self._velocity)
        rospy.Subscriber('/fire_mission/aligned', Bool, self._aligned)

    def _pose(self, message):
        self.pose = message

    def _velocity(self, message):
        self.velocity = message

    def _aligned(self, message):
        self.aligned = bool(message.data)

    def _release(self, channel):
        try:
            response = self.low_level(channel)
            return bool(response.success), response.reason
        except rospy.ServiceException as error:
            return False, 'plugin_service_failed:%s' % error

    def _drop(self, request):
        if self.pose is None or self.velocity is None:
            return DropSupplyResponse(False, 'flight_state_missing')
        linear = self.velocity.twist.linear
        speed = math.hypot(linear.x, linear.y)
        decision = self.controller.request(
            request.channel, self.aligned, speed,
            self.pose.pose.position.z, self._release)
        return DropSupplyResponse(decision.success, decision.reason)


if __name__ == '__main__':
    rospy.init_node('firefighting_supply_drop')
    SupplyDropNode()
    rospy.spin()
