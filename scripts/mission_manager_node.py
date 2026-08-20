#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String, UInt8

from firefighting_mission.msg import DropResult, MissionEvent, TargetDetection
from firefighting_mission.orchestration import (completion_should_shutdown,
                                                 validated_alignment)
from firefighting_mission.state_machine import Inputs, MissionStateMachine


GOALS = {
    'ARM': (0.0, 0.0, 1.30),
    'TAKEOFF': (0.0, 0.0, 1.30),
    'SEARCH_HAZARD': (1.25, -0.10, 1.30),
    'ALIGN_HAZARD': (1.25, -0.10, 1.30),
    'DROP_FIRE': (1.25, -0.10, 1.30),
    'SEARCH_PERSON': (2.65, -1.65, 1.30),
    'ALIGN_PERSON': (2.65, -1.65, 1.30),
    'DROP_RESCUE': (2.65, -1.65, 1.30),
    'RETURN_HOME': (0.0, 0.0, 1.30),
    'LAND': (0.0, 0.0, 0.08),
    'EMERGENCY_LAND': (0.0, 0.0, 0.08),
}


class MissionManagerNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix',
                                              '/iris_0/mavros').rstrip('/')
        self.machine = MissionStateMachine(rospy.Time.now().to_sec())
        self.pose = None
        self.state = State()
        self.nav_status = 'STALE'
        self.safety_status = 'LAND:missing'
        self.detection = TargetDetection()
        self.drop_result = DropResult()
        self.model_states = None
        self.last_phase = None
        self.last_drop_phase = None
        self.completion_timer = None
        self.completion_deadline = None
        self.recorder_finalized = False
        self.phase_pub = rospy.Publisher('/fire_mission/phase', String,
                                         queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/fire_mission/goal', PoseStamped,
                                        queue_size=1, latch=True)
        self.drop_pub = rospy.Publisher('/fire_mission/drop_request', UInt8,
                                        queue_size=1)
        self.aligned_pub = rospy.Publisher('/fire_mission/aligned', Bool,
                                           queue_size=1, latch=True)
        self.command_pub = rospy.Publisher('/xtdrone/iris_0/cmd', String,
                                           queue_size=1)
        self.event_pub = rospy.Publisher('/fire_mission/event', MissionEvent,
                                         queue_size=10, latch=True)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(self.mavros_prefix + '/state', State, self._state)
        rospy.Subscriber('/fire_mission/nav_status', String, self._nav)
        rospy.Subscriber('/fire_mission/safety_status', String, self._safety)
        rospy.Subscriber('/fire_mission/detection', TargetDetection,
                         self._detection)
        rospy.Subscriber('/fire_mission/drop_result', DropResult, self._drop)
        rospy.Subscriber('/fire_mission/recorder_finalized', Bool,
                         self._recorder_finalized)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._models)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._tick)

    def _pose(self, message):
        self.pose = message

    def _state(self, message):
        self.state = message

    def _nav(self, message):
        self.nav_status = message.data

    def _safety(self, message):
        self.safety_status = message.data

    def _detection(self, message):
        self.detection = message

    def _drop(self, message):
        self.drop_result = message

    def _recorder_finalized(self, message):
        self.recorder_finalized = bool(message.data)

    def _models(self, message):
        self.model_states = message

    def _ready(self):
        return bool(self.state.connected and self.pose is not None and
                    self.safety_status.startswith('CLEAR'))

    def _inputs(self):
        altitude = self.pose.pose.position.z if self.pose else 0.0
        target_class = self.detection.target_class
        if target_class in ('flammable', 'explosive', 'toxic'):
            target_class = 'hazard'
        safety_action = self.safety_status.split(':', 1)[0]
        drop_matches = self.drop_result.released and self.drop_result.channel in (1, 2)
        return Inputs(
            ready=self._ready(),
            armed=self.state.armed,
            offboard=self.state.mode == 'OFFBOARD',
            airborne=altitude > 0.25,
            goal_reached=self.nav_status == 'REACHED',
            detection_class=target_class,
            detection_confirmed=self.detection.confirmed,
            aligned=validated_alignment(self.machine.phase,
                                        self.detection.confirmed,
                                        self.nav_status),
            drop_channel=self.drop_result.channel if drop_matches else 0,
            drop_succeeded=bool(drop_matches),
            home_reached=self.nav_status == 'REACHED',
            landed=altitude <= 0.12,
            disarmed=not self.state.armed,
            pose_stale=safety_action in ('HOVER', 'LAND'),
            recovered=safety_action == 'CLEAR',
        )

    def _publish_goal(self, phase):
        if phase not in GOALS:
            return
        point = GOALS[phase]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = point
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _event(self, event_type, detail=''):
        event = MissionEvent()
        event.header.stamp = rospy.Time.now()
        event.phase = self.machine.phase
        event.event_type = event_type
        event.detail = detail
        event.elapsed_seconds = event.header.stamp.to_sec() - self.machine.start_time
        self.event_pub.publish(event)

    def _command_for_phase(self, phase):
        commands = {
            'ARM': 'ARM', 'TAKEOFF': 'OFFBOARD', 'LAND': 'AUTO.LAND',
            'EMERGENCY_LAND': 'AUTO.LAND', 'DISARM': 'DISARM',
        }
        if phase in commands:
            self.command_pub.publish(commands[phase])

    def _shutdown_after_completion(self, _event):
        if self.completion_deadline is None:
            return
        now = rospy.Time.now().to_sec()
        if completion_should_shutdown(self.recorder_finalized, now,
                                      self.completion_deadline):
            rospy.signal_shutdown('mission complete')

    def _tick(self, _event):
        inputs = self._inputs()
        command = self.machine.tick(rospy.Time.now().to_sec(), inputs)
        aligned = validated_alignment(command.phase, self.detection.confirmed,
                                      self.nav_status)
        self.aligned_pub.publish(aligned)
        if command.phase != self.last_phase:
            self.last_phase = command.phase
            self.phase_pub.publish(command.phase)
            self._publish_goal(command.phase)
            self._command_for_phase(command.phase)
            self._event('phase_changed', command.reason)
            if command.phase == 'COMPLETE' and self.completion_timer is None:
                timeout = float(rospy.get_param('~recorder_finalize_timeout',
                                                15.0))
                self.completion_deadline = (rospy.Time.now().to_sec() + timeout)
                self.completion_timer = rospy.Timer(
                    rospy.Duration(0.10), self._shutdown_after_completion)
        if command.drop_channel and command.phase != self.last_drop_phase:
            self.last_drop_phase = command.phase
            self.drop_pub.publish(command.drop_channel)
            self._event('drop_requested', str(command.drop_channel))


if __name__ == '__main__':
    rospy.init_node('firefighting_mission_manager')
    MissionManagerNode()
    rospy.spin()
