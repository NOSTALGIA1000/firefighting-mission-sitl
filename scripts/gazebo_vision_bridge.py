#!/usr/bin/env python
from __future__ import print_function

import copy

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

from firefighting_mission.external_vision import (due_for_publish, model_state,
                                                  sample_is_usable,
                                                  world_vector_to_body)


class GazeboVisionBridge(object):
    def __init__(self):
        self.model_name = rospy.get_param('~model_name', 'iris_0')
        output_topic = rospy.get_param('~output_topic',
                                       '/mavros/odometry/out')
        publish_rate = float(rospy.get_param('~publish_rate', 50.0))
        if publish_rate <= 0.0:
            raise ValueError('~publish_rate must be positive')

        self.period = 1.0 / publish_rate
        self.last_sample_time = None
        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                           self._model_states, queue_size=1)

    def _model_states(self, message):
        state = model_state(message, self.model_name)
        if state is None:
            return
        if not sample_is_usable(state[0], state[1]):
            rospy.logwarn_throttle(
                5.0, 'dropping non-finite Gazebo sample for %s',
                self.model_name)
            return
        # Stamp and throttle here, so the timestamp belongs to this sample.
        # Republishing the latest sample from a timer instead offsets the
        # stamp by up to one period, and PX4 fuses external vision with
        # EKF2_EV_DELAY at zero, so that offset becomes innovation.
        stamp = rospy.Time.now()
        if not due_for_publish(self.last_sample_time, stamp.to_sec(),
                               self.period):
            return
        self.last_sample_time = stamp.to_sec()
        self._publish(state, stamp)

    def _publish(self, state, stamp):
        pose, twist = state

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose = copy.deepcopy(pose)

        linear = world_vector_to_body(pose.orientation, twist.linear)
        angular = world_vector_to_body(pose.orientation, twist.angular)
        message.twist.twist.linear.x = linear[0]
        message.twist.twist.linear.y = linear[1]
        message.twist.twist.linear.z = linear[2]
        message.twist.twist.angular.x = angular[0]
        message.twist.twist.angular.y = angular[1]
        message.twist.twist.angular.z = angular[2]

        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0001
            message.twist.covariance[index] = 0.01
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.01
            message.twist.covariance[index] = 0.01
        self.publisher.publish(message)


def main():
    rospy.init_node('gazebo_vision_bridge')
    GazeboVisionBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
