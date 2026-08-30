#!/usr/bin/env python
from __future__ import print_function

import copy

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

from firefighting_mission.external_vision import model_state, world_vector_to_body


class GazeboVisionBridge(object):
    def __init__(self):
        self.model_name = rospy.get_param('~model_name', 'iris_0')
        output_topic = rospy.get_param('~output_topic',
                                       '/mavros/odometry/out')
        publish_rate = float(rospy.get_param('~publish_rate', 50.0))
        if publish_rate <= 0.0:
            raise ValueError('~publish_rate must be positive')

        self.latest_state = None
        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                           self._model_states, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / publish_rate),
                                 self._publish)

    def _model_states(self, message):
        state = model_state(message, self.model_name)
        if state is not None:
            self.latest_state = copy.deepcopy(state)

    def _publish(self, _event):
        if self.latest_state is None:
            return
        pose, twist = self.latest_state
        stamp = rospy.Time.now()

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
