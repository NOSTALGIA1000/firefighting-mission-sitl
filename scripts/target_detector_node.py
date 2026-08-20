#!/usr/bin/env python
from __future__ import division, print_function

import os

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from firefighting_mission.msg import TargetDetection
from firefighting_mission.perception import (HAZARD_CLASSES, StableDetector,
                                             TemplatePerception, annotate,
                                             load_templates)


class TargetDetectorNode(object):
    def __init__(self):
        package_root = rospkg.RosPack().get_path('firefighting_mission')
        template_dir = rospy.get_param(
            '~template_dir', os.path.join(package_root, 'assets', 'templates'))
        self.bridge = CvBridge()
        self.detector = TemplatePerception(
            load_templates(template_dir),
            threshold=rospy.get_param('~threshold', 0.72))
        self.stable = StableDetector(window=5, required=4)
        self.phase = 'WAIT_READY'
        self.start = rospy.Time.now()
        self.detection_pub = rospy.Publisher('/fire_mission/detection',
                                             TargetDetection, queue_size=1)
        self.annotated_pub = rospy.Publisher('/fire_mission/annotated', Image,
                                             queue_size=1)
        rospy.Subscriber('/fire_mission/phase', String, self._phase)
        rospy.Subscriber(rospy.get_param('~image_topic', '/camera/image_raw'),
                         Image, self._image, queue_size=1, buff_size=2 ** 22)

    def _phase(self, message):
        self.phase = message.data

    def _image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, 'image conversion failed: %s', error)
            return
        result = self.detector.detect(image, self.phase)
        matchable = (result.target_class == 'person' or
                     (result.target_class in HAZARD_CLASSES and
                      result.target_class != 'distractor'))
        confirmed = self.stable.update(result.target_class, matchable)

        detection = TargetDetection()
        detection.header = message.header
        detection.target_class = result.target_class
        detection.confidence = result.confidence
        if result.box:
            detection.x, detection.y, detection.width, detection.height = result.box
        detection.confirmation_count = self.stable.confirmation_count
        detection.confirmed = confirmed
        self.detection_pub.publish(detection)

        output = annotate(
            image, result.box, result.target_class, result.confidence,
            self.phase, (rospy.Time.now() - self.start).to_sec())
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(output, encoding='bgr8'))


if __name__ == '__main__':
    rospy.init_node('firefighting_target_detector')
    TargetDetectorNode()
    rospy.spin()
