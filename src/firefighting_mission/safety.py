from __future__ import print_function

import math
from collections import namedtuple


SafetyStatus = namedtuple('SafetyStatus', 'action reason')


class SafetyMonitor(object):
    def __init__(self, stale_hover=0.5, stale_land=2.0,
                 attitude_limit_degrees=30.0, altitude_limit=2.5,
                 retreat_distance=0.30, boundary_limit=0.20):
        self.stale_hover = float(stale_hover)
        self.stale_land = float(stale_land)
        self.attitude_limit = math.radians(attitude_limit_degrees)
        self.altitude_limit = float(altitude_limit)
        self.retreat_distance = float(retreat_distance)
        self.boundary_limit = float(boundary_limit)

    def evaluate(self, pose_age, scan_age, roll, pitch, altitude,
                 minimum_obstacle, boundary_margin, stereo_age=0.0):
        if pose_age > self.stale_land:
            return SafetyStatus('LAND', 'pose_stale')
        if scan_age > self.stale_land:
            return SafetyStatus('LAND', 'scan_stale')
        if stereo_age > self.stale_land:
            return SafetyStatus('LAND', 'stereo_stale')
        if abs(roll) > self.attitude_limit or abs(pitch) > self.attitude_limit:
            return SafetyStatus('LAND', 'attitude_limit')
        if altitude > self.altitude_limit:
            return SafetyStatus('LAND', 'altitude_limit')
        if boundary_margin < self.boundary_limit:
            return SafetyStatus('RETREAT', 'boundary_limit')
        if minimum_obstacle < self.retreat_distance:
            return SafetyStatus('RETREAT', 'obstacle_too_close')
        if pose_age > self.stale_hover:
            return SafetyStatus('HOVER', 'pose_stale')
        if scan_age > self.stale_hover:
            return SafetyStatus('HOVER', 'scan_stale')
        if stereo_age > self.stale_hover:
            return SafetyStatus('HOVER', 'stereo_stale')
        return SafetyStatus('CLEAR', '')
