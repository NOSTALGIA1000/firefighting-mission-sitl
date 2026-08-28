#!/usr/bin/env python
from __future__ import division, print_function

import math

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from firefighting_mission.competition_main import (CompetitionMain,
                                                    PositionSetpoint,
                                                    select_active_setpoints)
from firefighting_mission.path_planner import map_target_to_local


def quaternion_yaw(orientation):
    numerator = 2.0 * (orientation.w * orientation.z +
                       orientation.x * orientation.y)
    denominator = 1.0 - 2.0 * (orientation.y * orientation.y +
                               orientation.z * orientation.z)
    return math.atan2(numerator, denominator)


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
        self.map_pose = None
        self.imu = None
        self.use_gazebo_ground_truth = rospy.get_param(
            '~use_gazebo_ground_truth', False)
        self.gazebo_model_name = rospy.get_param(
            '~gazebo_model_name', 'iris_0')
        self.path_setpoint = None
        self.path_control_enabled = False
        self.terminal_command = ''
        self.safety_action = 'CLEAR'
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
        rospy.Subscriber(self.mavros_prefix + '/imu/data', Imu, self._imu)
        rospy.Subscriber('/fire_mission/path_setpoint', PoseStamped,
                         self._path_setpoint)
        rospy.Subscriber('/xtdrone/iris_0/cmd', String,
                         self._flight_command)
        rospy.Subscriber('/fire_mission/safety_status', String,
                         self._safety_status)
        if self.use_gazebo_ground_truth:
            rospy.Subscriber('/gazebo/model_states', ModelStates,
                             self._model_states)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    def _state(self, message):
        self.state = message

    def _pose(self, message):
        self.pose = message

    def _imu(self, message):
        self.imu = message

    def _model_states(self, message):
        try:
            index = message.name.index(self.gazebo_model_name)
        except ValueError:
            return
        value = message.pose[index]
        self.map_pose = (
            value.position.x, value.position.y, 0.0,
            quaternion_yaw(value.orientation))

    @staticmethod
    def _pose_tuple(message):
        point = message.pose.position
        return (point.x, point.y, point.z,
                quaternion_yaw(message.pose.orientation))

    def _path_setpoint(self, message):
        point = message.pose.position
        self.path_setpoint = PositionSetpoint(
            point.x, point.y, point.z,
            quaternion_yaw(message.pose.orientation))

    def _flight_command(self, message):
        command = message.data.strip().upper()
        if command in ('AUTO.LAND', 'DISARM'):
            self.terminal_command = command

    def _safety_status(self, message):
        self.safety_action = message.data.split(':', 1)[0].upper()
        airborne = bool(self.state.armed and self.pose is not None and
                        self.pose.pose.position.z > 0.25)
        if self.safety_action == 'LAND' and airborne:
            self.terminal_command = 'AUTO.LAND'
        elif self.safety_action in ('HOVER', 'RETREAT') and airborne:
            point = self.pose.pose.position
            if self.use_gazebo_ground_truth and self.map_pose is not None:
                self.path_setpoint = PositionSetpoint(
                    self.map_pose[0], self.map_pose[1], point.z,
                    self.map_pose[3])
            else:
                self.path_setpoint = PositionSetpoint(
                    point.x, point.y, point.z,
                    quaternion_yaw(self.pose.pose.orientation))
            self.path_control_enabled = True

    def _altitude(self):
        if self.pose is None:
            return 0.0
        return self.pose.pose.position.z

    def _publish_setpoint(self, point):
        published = point
        if self.use_gazebo_ground_truth:
            if self.map_pose is None or self.pose is None:
                return
            values = map_target_to_local(
                (point.x, point.y, point.z, point.yaw),
                (self.map_pose[0], self.map_pose[1],
                 self.pose.pose.position.z, self.map_pose[3]),
                self._pose_tuple(self.pose))
            published = PositionSetpoint(*values)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = published.x
        pose.pose.position.y = published.y
        pose.pose.position.z = published.z
        pose.pose.orientation.z = math.sin(published.yaw / 2.0)
        pose.pose.orientation.w = math.cos(published.yaw / 2.0)
        self.setpoint_pub.publish(pose)

    def _tick(self, _event):
        if self.terminal_command == 'AUTO.LAND':
            self.phase_pub.publish('AUTO.LAND')
            try:
                self.set_mode(custom_mode='AUTO.LAND')
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(1.0, 'AUTO.LAND failed: %s', exc)
            return
        if self.terminal_command == 'DISARM':
            self.phase_pub.publish('DISARM')
            try:
                self.arm(False)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(1.0, 'disarming failed: %s', exc)
            return
        if self.use_gazebo_ground_truth and self.map_pose is None:
            self.phase_pub.publish('WAIT_MAP')
            return
        outputs = self.controller.tick(
            rospy.Time.now().to_sec(),
            connected=self.state.connected,
            armed=self.state.armed,
            mode=self.state.mode,
            altitude=self._altitude(),
            sensor_ready=self.imu is not None,
            local_pose_available=self.pose is not None,
        )
        if outputs.state == 'HOVER' and self.path_setpoint is not None:
            self.path_control_enabled = True
        self.phase_pub.publish(outputs.state)
        setpoints = select_active_setpoints(
            outputs, self.path_setpoint, self.path_control_enabled)
        for point in setpoints:
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
