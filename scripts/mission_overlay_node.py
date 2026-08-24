#!/usr/bin/env python
from __future__ import print_function

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from firefighting_mission.avoidance_overlay import compose_mission_view
from firefighting_mission.msg import AvoidanceStatus, ObstacleArray


class MissionOverlayNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.target_image = None
        self.front_image = None
        self.latest_status = None
        self.latest_obstacles = ()
        self.publisher = rospy.Publisher('/fire_mission/mission_view', Image,
                                         queue_size=1)
        rospy.Subscriber(rospy.get_param(
            '~front_topic', '/fire_stereo/rgb/image_raw'), Image,
            self._front, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber(rospy.get_param(
            '~target_topic', '/fire_mission/annotated'), Image,
            self._target, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber('/fire_mission/avoidance_status', AvoidanceStatus,
                         self._status, queue_size=1)
        rospy.Subscriber('/fire_mission/obstacles', ObstacleArray,
                         self._obstacles, queue_size=1)

    def _convert(self, message):
        try:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, 'mission overlay conversion failed: %s',
                                   error)
            return None

    def _front(self, message):
        self.front_image = self._convert(message)
        self._publish(message.header)

    def _target(self, message):
        self.target_image = self._convert(message)
        self._publish(message.header)

    def _status(self, message):
        self.latest_status = message

    def _obstacles(self, message):
        self.latest_obstacles = tuple(message.obstacles)

    def _publish(self, header):
        output = compose_mission_view(
            self.target_image, self.front_image, self.latest_status,
            self.latest_obstacles)
        if output is None:
            return
        message = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        message.header = header
        self.publisher.publish(message)


if __name__ == '__main__':
    rospy.init_node('firefighting_mission_overlay')
    MissionOverlayNode()
    rospy.spin()
