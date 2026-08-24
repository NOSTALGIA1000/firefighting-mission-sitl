#!/usr/bin/env python
from __future__ import division, print_function

from collections import deque

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from firefighting_mission.msg import (ObstacleArray, ObstacleCluster)
from firefighting_mission.stereo_obstacles import (
    clusters_from_depth, clusters_from_points, stable_clusters)


class StereoObstacleNode(object):
    def __init__(self):
        self.input_mode = rospy.get_param('~input_mode', 'depth')
        self.depth_topic = rospy.get_param(
            '~depth_topic', '/fire_stereo/depth/image_raw')
        self.info_topic = rospy.get_param(
            '~camera_info_topic', '/fire_stereo/depth/camera_info')
        self.points_topic = rospy.get_param(
            '~points_topic', '/fire_stereo/points')
        self.stale_seconds = float(rospy.get_param('~stale_seconds', 0.30))
        self.bridge = CvBridge()
        self.history = deque(maxlen=int(rospy.get_param('~history_frames', 3)))
        self.last_stamp = None
        self.publisher = rospy.Publisher(
            '/fire_mission/obstacles', ObstacleArray, queue_size=1)

        if self.input_mode == 'depth':
            depth_sub = message_filters.Subscriber(self.depth_topic, Image)
            info_sub = message_filters.Subscriber(self.info_topic, CameraInfo)
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [depth_sub, info_sub], queue_size=5, slop=0.08)
            self.synchronizer.registerCallback(self._depth)
        elif self.input_mode in ('points', 'raw_stereo'):
            self.points_sub = rospy.Subscriber(
                self.points_topic, PointCloud2, self._points, queue_size=1)
        else:
            rospy.logerr('unsupported stereo input mode: %s', self.input_mode)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._watchdog)

    @staticmethod
    def _cluster_message(value):
        message = ObstacleCluster()
        message.forward_m = value.forward_m
        message.left_m = value.left_m
        message.nearest_range_m = value.nearest_range_m
        message.left_edge_m = value.left_edge_m
        message.right_edge_m = value.right_edge_m
        message.confidence = value.confidence
        return message

    def _publish(self, stamp, clusters, ready, reason=''):
        message = ObstacleArray()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        message.obstacles = [self._cluster_message(value) for value in clusters]
        message.ready = bool(ready)
        message.reason = reason
        self.publisher.publish(message)

    def _accept(self, stamp, clusters):
        self.last_stamp = rospy.Time.now()
        self.history.append(tuple(clusters))
        required = min(3, self.history.maxlen)
        stable = stable_clusters(tuple(self.history), required=required)
        self._publish(stamp, stable, True)

    def _depth(self, image_message, info_message):
        if not info_message.K or float(info_message.K[0]) <= 0.0:
            self._publish(image_message.header.stamp, (), False,
                          'camera_info_missing')
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, 'depth conversion failed: %s', error)
            self._publish(image_message.header.stamp, (), False,
                          'unsupported_encoding')
            return
        if image_message.encoding == '16UC1':
            depth_m = np.asarray(depth, dtype=np.float32) / 1000.0
        elif image_message.encoding == '32FC1':
            depth_m = np.asarray(depth, dtype=np.float32)
        else:
            self._publish(image_message.header.stamp, (), False,
                          'unsupported_encoding')
            return
        clusters = clusters_from_depth(depth_m, info_message.K[0],
                                       info_message.K[2])
        self._accept(image_message.header.stamp, clusters)

    def _points(self, message):
        body_points = []
        for x_value, y_value, z_value in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True):
            body_points.append((z_value, -x_value, -y_value))
        self._accept(message.header.stamp,
                     clusters_from_points(body_points))

    def _watchdog(self, _event):
        now = rospy.Time.now()
        if self.input_mode not in ('depth', 'points', 'raw_stereo'):
            self._publish(now, (), False, 'unsupported_input_mode')
            return
        if self.last_stamp is None:
            reason = ('pointcloud_stale' if self.input_mode in
                      ('points', 'raw_stereo') else 'depth_stale')
            self._publish(now, (), False, reason)
            return
        if (now - self.last_stamp).to_sec() > self.stale_seconds:
            reason = ('pointcloud_stale' if self.input_mode in
                      ('points', 'raw_stereo') else 'depth_stale')
            self._publish(now, (), False, reason)


if __name__ == '__main__':
    rospy.init_node('firefighting_stereo_obstacles')
    StereoObstacleNode()
    rospy.spin()
