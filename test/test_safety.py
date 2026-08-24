from __future__ import print_function

import math
import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.safety import SafetyMonitor


class SafetyMonitorTest(unittest.TestCase):
    def setUp(self):
        self.monitor = SafetyMonitor()

    def test_fresh_nominal_inputs_are_clear(self):
        result = self.monitor.evaluate(pose_age=0.1, scan_age=0.1,
                                       roll=0.0, pitch=0.0, altitude=1.3,
                                       minimum_obstacle=1.0,
                                       boundary_margin=0.8)
        self.assertEqual('CLEAR', result.action)

    def test_pose_stale_for_half_second_requests_hover(self):
        result = self.monitor.evaluate(pose_age=0.51, scan_age=0.1,
                                       roll=0.0, pitch=0.0, altitude=1.3,
                                       minimum_obstacle=1.0,
                                       boundary_margin=0.8)
        self.assertEqual('HOVER', result.action)
        self.assertEqual('pose_stale', result.reason)

    def test_pose_stale_for_two_seconds_requests_land(self):
        result = self.monitor.evaluate(pose_age=2.01, scan_age=0.1,
                                       roll=0.0, pitch=0.0, altitude=1.3,
                                       minimum_obstacle=1.0,
                                       boundary_margin=0.8)
        self.assertEqual('LAND', result.action)

    def test_excessive_attitude_requests_land(self):
        result = self.monitor.evaluate(pose_age=0.1, scan_age=0.1,
                                       roll=math.radians(31), pitch=0.0,
                                       altitude=1.3, minimum_obstacle=1.0,
                                       boundary_margin=0.8)
        self.assertEqual('LAND', result.action)
        self.assertEqual('attitude_limit', result.reason)

    def test_close_obstacle_requests_retreat(self):
        result = self.monitor.evaluate(pose_age=0.1, scan_age=0.1,
                                       stereo_age=0.1,
                                       roll=0.0, pitch=0.0, altitude=1.3,
                                       minimum_obstacle=0.29,
                                       boundary_margin=0.8)
        self.assertEqual('RETREAT', result.action)

    def test_stereo_stale_requests_hover_before_land(self):
        monitor = SafetyMonitor(stale_hover=0.30, stale_land=1.00)

        hover = monitor.evaluate(
            pose_age=0.1, scan_age=0.1, stereo_age=0.31,
            roll=0.0, pitch=0.0, altitude=1.2,
            minimum_obstacle=1.0, boundary_margin=1.0)
        land = monitor.evaluate(
            pose_age=0.1, scan_age=0.1, stereo_age=1.01,
            roll=0.0, pitch=0.0, altitude=1.2,
            minimum_obstacle=1.0, boundary_margin=1.0)

        self.assertEqual(('HOVER', 'stereo_stale'),
                         (hover.action, hover.reason))
        self.assertEqual(('LAND', 'stereo_stale'),
                         (land.action, land.reason))


if __name__ == '__main__':
    unittest.main()
