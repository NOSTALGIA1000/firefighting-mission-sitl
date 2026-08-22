#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from firefighting_mission.competition_main import CompetitionMain


class CompetitionMainNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        self.controller = CompetitionMain(
            takeoff_altitude=rospy.get_param('~takeoff_altitude', 1.2),
            prestream_count=rospy.get_param('~prestream_count', 40),
            hover_tolerance=rospy.get_param('~hover_tolerance', 0.08),
            hover_hold_seconds=rospy.get_param('~hover_hold_seconds', 2.0),
        )
        self.state = State()
        self.pose = None
        self.setpoint_pub = rospy.Publisher(
            self.mavros_prefix + '/setpoint_position/local',
            PoseStamped, queue_size=10)
        self.phase_pub = rospy.Publisher(
            '/competition_main/state', String, queue_size=1, latch=True)
        self.arm = rospy.ServiceProxy(self.mavros_prefix + '/cmd/arming',
                                      CommandBool)
        self.set_mode = rospy.ServiceProxy(self.mavros_prefix + '/set_mode',
                                           SetMode)
        rospy.Subscriber(self.mavros_prefix + '/state', State, self._state)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose',
                         PoseStamped, self._pose)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    def _state(self, message):
        self.state = message

    def _pose(self, message):
        self.pose = message

    def _altitude(self):
        if self.pose is None:
            return 0.0
        return self.pose.pose.position.z

    def _publish_setpoint(self, point):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def _tick(self, _event):
        outputs = self.controller.tick(
            rospy.Time.now().to_sec(),
            connected=self.state.connected,
            armed=self.state.armed,
            mode=self.state.mode,
            altitude=self._altitude(),
            local_pose_available=self.pose is not None,
        )
        self.phase_pub.publish(outputs.state)
        for point in outputs.setpoints:
            self._publish_setpoint(point)
        for request in outputs.mode_requests:
            try:
                self.set_mode(custom_mode=request.mode)
            except rospy.ServiceException as exc:
                rospy.logwarn('set_mode failed: %s', exc)
        if outputs.arm_request:
            try:
                self.arm(True)
            except rospy.ServiceException as exc:
                rospy.logwarn('arming failed: %s', exc)


if __name__ == '__main__':
    rospy.init_node('competition_main')
    CompetitionMainNode()
    rospy.spin()
