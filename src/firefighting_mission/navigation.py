from __future__ import division, print_function

import math
from collections import namedtuple


VelocityCommand = namedtuple('VelocityCommand', 'x y z yaw_rate status')


class NavigationConfig(object):
    def __init__(self, max_xy=0.55, max_z=0.30, position_gain=0.8,
                 vertical_gain=0.8, reached_xy=0.12, reached_z=0.08,
                 slow_distance=0.70, detour_distance=0.45,
                 retreat_distance=0.30):
        self.max_xy = float(max_xy)
        self.max_z = float(max_z)
        self.position_gain = float(position_gain)
        self.vertical_gain = float(vertical_gain)
        self.reached_xy = float(reached_xy)
        self.reached_z = float(reached_z)
        self.slow_distance = float(slow_distance)
        self.detour_distance = float(detour_distance)
        self.retreat_distance = float(retreat_distance)


def _bounded(value, limit):
    return max(-limit, min(limit, value))


def _valid_range(value):
    return (value is not None and not math.isnan(value) and
            not math.isinf(value) and value > 0.02)


def sector_distances(ranges, angle_min, angle_increment):
    sectors = {'front': [], 'left': [], 'right': []}
    for index, distance in enumerate(ranges):
        if not _valid_range(distance):
            continue
        angle = angle_min + index * angle_increment
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        degrees = math.degrees(wrapped)
        if -20.0 <= degrees <= 20.0:
            sectors['front'].append(distance)
        elif 20.0 < degrees <= 70.0:
            sectors['left'].append(distance)
        elif -70.0 <= degrees < -20.0:
            sectors['right'].append(distance)
    return tuple(min(sectors[name]) if sectors[name] else float('inf')
                 for name in ('front', 'left', 'right'))


class Navigator(object):
    def __init__(self, config=None):
        self.config = config or NavigationConfig()

    def compute(self, goal, pose, yaw, sectors):
        gx, gy, gz = goal
        px, py, pz = pose
        dx, dy, dz = gx - px, gy - py, gz - pz
        horizontal_error = math.hypot(dx, dy)
        if horizontal_error <= self.config.reached_xy and abs(dz) <= self.config.reached_z:
            return VelocityCommand(0.0, 0.0, 0.0, 0.0, 'REACHED')

        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        vertical = _bounded(self.config.vertical_gain * dz, self.config.max_z)
        front, left, right = sectors

        if front < self.config.retreat_distance:
            return VelocityCommand(-0.15, 0.0, vertical, 0.0, 'RETREAT')
        if front < self.config.detour_distance:
            lateral = 0.25 if left >= right else -0.25
            return VelocityCommand(0.0, lateral, vertical, 0.0, 'AVOIDING')
        if front < self.config.slow_distance:
            lateral = 0.18 if left >= right else -0.18
            return VelocityCommand(0.08, lateral, vertical, 0.0, 'AVOIDING')

        return VelocityCommand(
            _bounded(self.config.position_gain * body_x, self.config.max_xy),
            _bounded(self.config.position_gain * body_y, self.config.max_xy),
            vertical,
            0.0,
            'ACTIVE',
        )
