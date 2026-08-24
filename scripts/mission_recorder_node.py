#!/usr/bin/env python
from __future__ import division, print_function

import csv
import math
import os
import signal
import subprocess
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ContactsState, LinkStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from firefighting_mission.msg import DropResult, MissionEvent, TargetDetection
from firefighting_mission.orchestration import (contacts_indicate_collision,
                                                 payload_link_position,
                                                 recording_topics)
from firefighting_mission.scoring import Score, write_score
from firefighting_mission.world_generator import HAZARD_POSES, PERSON_POSES, build_scenario


BAG_FLUSH_TIMEOUT_SECONDS = 5.0


class MissionRecorderNode(object):
    def __init__(self):
        self.mavros_prefix = rospy.get_param('~mavros_prefix',
                                              '/iris_0/mavros').rstrip('/')
        self.seed = int(rospy.get_param('~seed', 4501))
        self.record = bool(rospy.get_param('~record', True))
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.output_dir = rospy.get_param(
            '~output_dir', os.path.join(package_root, 'artifacts', str(self.seed)))
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.scenario = build_scenario(self.seed)
        self.started = rospy.Time.now()
        self.phase = 'WAIT_READY'
        self.state = State()
        self.pose = None
        self.minimum_clearance = float('inf')
        self.hazard_identified = False
        self.person_identified = False
        self.drop_results = {}
        self.link_positions = {}
        self.collision = False
        self.writer = None
        self.bridge = CvBridge()
        self.finalized = False
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.event_file = open(os.path.join(self.output_dir, 'events.log'), 'a')
        self.trajectory_file = open(os.path.join(self.output_dir, 'trajectory.csv'), 'w')
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow(('elapsed_seconds', 'x', 'y', 'z', 'phase'))
        self.finalized_pub = rospy.Publisher('/fire_mission/recorder_finalized',
                                             Bool, queue_size=1, latch=True)
        self.bag_process = self._start_bag() if self.record else None
        rospy.Subscriber('/fire_mission/phase', String, self._phase)
        rospy.Subscriber('/fire_mission/event', MissionEvent, self._event)
        rospy.Subscriber('/fire_mission/detection', TargetDetection, self._detection)
        rospy.Subscriber('/fire_mission/drop_result', DropResult, self._drop)
        rospy.Subscriber('/fire_mission/mission_view', Image, self._image,
                         queue_size=1)
        rospy.Subscriber(self.mavros_prefix + '/local_position/pose', PoseStamped,
                         self._pose)
        rospy.Subscriber(self.mavros_prefix + '/state', State, self._state)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._links)
        rospy.Subscriber('/fire_mission/contacts', ContactsState, self._contacts)
        rospy.on_shutdown(self._shutdown)

    def _start_bag(self):
        path = os.path.join(self.output_dir, 'mission.bag')
        topics = recording_topics(self.mavros_prefix, self.scan_topic)
        try:
            return subprocess.Popen(['rosbag', 'record', '-O', path] + topics)
        except OSError as error:
            rospy.logwarn('unable to start rosbag recorder: %s', error)
            return None

    def _phase(self, message):
        self.phase = message.data
        if self.phase == 'COMPLETE':
            self._finalize()

    def _event(self, message):
        self.event_file.write('%0.3f %s %s %s\n' % (
            message.elapsed_seconds, message.phase, message.event_type,
            message.detail))
        self.event_file.flush()

    def _detection(self, message):
        if not message.confirmed:
            return
        if message.target_class in ('flammable', 'explosive', 'toxic'):
            self.hazard_identified = True
        elif message.target_class == 'person':
            self.person_identified = True

    def _drop(self, message):
        if message.released:
            self.drop_results[message.channel] = message

    def _pose(self, message):
        self.pose = message
        point = message.pose.position
        self.trajectory.writerow(('%0.3f' % (rospy.Time.now() - self.started).to_sec(),
                                '%.4f' % point.x, '%.4f' % point.y,
                                '%.4f' % point.z, self.phase))
        self.trajectory_file.flush()

    def _state(self, message):
        self.state = message

    def _scan(self, message):
        valid = [value for value in message.ranges
                 if not math.isnan(value) and not math.isinf(value) and value > 0.02]
        if valid:
            self.minimum_clearance = min(self.minimum_clearance, min(valid))

    def _links(self, message):
        self.link_positions = dict((name, pose.position)
                                   for name, pose in zip(message.name, message.pose))

    def _contacts(self, message):
        self.collision = self.collision or contacts_indicate_collision(message.states)

    def _image(self, message):
        if not self.record:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, 'annotation conversion failed: %s', error)
            return
        if self.writer is None:
            height, width = image.shape[:2]
            path = os.path.join(self.output_dir, 'mission_view.mp4')
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'),
                                          20.0, (width, height))
        self.writer.write(image)

    def _drop_error(self, channel, target):
        result = self.drop_results.get(channel)
        point = result.landing_position if result else None
        if point is None or (point.x == 0.0 and point.y == 0.0 and point.z == 0.0):
            point = payload_link_position(self.link_positions, channel)
        if point is None:
            return float('inf')
        return math.hypot(point.x - target[0], point.y - target[1])

    def _score(self):
        landing_error = float('inf')
        if self.pose is not None:
            point = self.pose.pose.position
            landing_error = math.hypot(point.x, point.y)
        return Score(
            seed=self.seed,
            runtime_seconds=(rospy.Time.now() - self.started).to_sec(),
            minimum_clearance_m=(self.minimum_clearance if
                                 self.minimum_clearance != float('inf') else 0.0),
            hazard_identified=self.hazard_identified,
            person_identified=self.person_identified,
            fire_drop_error_m=self._drop_error(1, HAZARD_POSES[self.scenario.hazard_index]),
            rescue_drop_error_m=self._drop_error(2, PERSON_POSES[self.scenario.person_position]),
            landing_error_m=landing_error,
            disarmed=not self.state.armed,
            collision=self.collision,
            completed=self.phase == 'COMPLETE',
        )

    def _finalize(self):
        if self.finalized:
            return
        self.finalized = True
        self.event_file.close()
        self.trajectory_file.close()
        if self.writer is not None:
            self.writer.release()
        if self.bag_process is not None:
            self._stop_bag()
        write_score(self._score(), os.path.join(self.output_dir, 'score.json'))
        self.finalized_pub.publish(Bool(data=True))

    def _stop_bag(self):
        if self.bag_process.poll() is not None:
            return
        self.bag_process.send_signal(signal.SIGINT)
        deadline = time.time() + BAG_FLUSH_TIMEOUT_SECONDS
        while self.bag_process.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if self.bag_process.poll() is None:
            rospy.logwarn('rosbag did not flush in %.1f seconds; terminating',
                          BAG_FLUSH_TIMEOUT_SECONDS)
            self.bag_process.terminate()

    def _shutdown(self):
        self._finalize()


if __name__ == '__main__':
    rospy.init_node('firefighting_mission_recorder')
    MissionRecorderNode()
    rospy.spin()
