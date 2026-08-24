from __future__ import division, print_function

import math
from collections import namedtuple


PlanCommand = namedtuple('PlanCommand', 'stage target')


class PathPlannerConfig(object):
    def __init__(self, safe_altitude=2.30, reached_xy=0.12, reached_z=0.08):
        self.safe_altitude = float(safe_altitude)
        self.reached_xy = float(reached_xy)
        self.reached_z = float(reached_z)


class StagedPathPlanner(object):
    def __init__(self, config=None):
        self.config = config or PathPlannerConfig()
        self.goal = None
        self.start_xy = None
        self.stage = 'IDLE'

    def set_goal(self, goal, pose):
        self.goal = tuple(float(value) for value in goal)
        self.start_xy = (float(pose[0]), float(pose[1]))
        self.stage = 'CLIMB'

    def update(self, pose):
        if self.goal is None:
            return PlanCommand('IDLE', None)

        px, py, pz = (float(value) for value in pose)
        gx, gy, gz = self.goal
        safe = self.config.safe_altitude

        if self.stage == 'CLIMB':
            if abs(pz - safe) > self.config.reached_z:
                return PlanCommand('CLIMB', self.start_xy + (safe,))
            self.stage = 'CRUISE'

        if self.stage == 'CRUISE':
            if math.hypot(gx - px, gy - py) > self.config.reached_xy:
                return PlanCommand('CRUISE', (gx, gy, safe))
            self.stage = 'DESCEND'

        if self.stage == 'DESCEND':
            if abs(gz - pz) > self.config.reached_z:
                return PlanCommand('DESCEND', self.goal)
            self.stage = 'REACHED'

        return PlanCommand('REACHED', self.goal)
