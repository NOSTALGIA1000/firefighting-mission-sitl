#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from firefighting_mission.path_planner import StagedPathPlanner


class PathPlannerNode(object):
    def __init__(self):
        self.planner = StagedPathPlanner()
        self.pose = None
        self.requested_goal = None
        self.active_goal = None
        self.target_pub = rospy.Publisher(
            '/fire_mission/path_setpoint', PoseStamped,
            queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            '/fire_mission/path_status', String,
            queue_size=1, latch=True)
        prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        rospy.Subscriber(prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber('/fire_mission/point_goal', PoseStamped, self._goal)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    @staticmethod
    def _point(message):
        point = message.pose.position
        return (point.x, point.y, point.z)

    def _pose(self, message):
        self.pose = message

    def _goal(self, message):
        self.requested_goal = self._point(message)

    def _tick(self, _event):
        if self.pose is None or self.requested_goal is None:
            self.status_pub.publish('IDLE')
            return
        pose = self._point(self.pose)
        if self.requested_goal != self.active_goal:
            self.planner.set_goal(self.requested_goal, pose)
            self.active_goal = self.requested_goal
        command = self.planner.update(pose)
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.pose.position.x = command.target[0]
        target.pose.position.y = command.target[1]
        target.pose.position.z = command.target[2]
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)
        self.status_pub.publish(command.stage)


if __name__ == '__main__':
    rospy.init_node('firefighting_path_planner')
    PathPlannerNode()
    rospy.spin()
