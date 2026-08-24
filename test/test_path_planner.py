from __future__ import print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.path_planner import StagedPathPlanner


class StagedPathPlannerTest(unittest.TestCase):
    def test_climbs_without_horizontal_motion(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))

        command = planner.update((0.0, 0.0, 1.2))

        self.assertEqual('CLIMB', command.stage)
        self.assertEqual((0.0, 0.0, 2.3), command.target)

    def test_cruises_at_safe_altitude_after_climb(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))

        command = planner.update((0.01, -0.01, 2.24))

        self.assertEqual('CRUISE', command.stage)
        self.assertEqual((2.0, -1.0, 2.3), command.target)

    def test_descends_only_after_horizontal_arrival(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        planner.update((0.0, 0.0, 2.3))

        command = planner.update((1.91, -0.94, 2.3))

        self.assertEqual('DESCEND', command.stage)
        self.assertEqual((2.0, -1.0, 1.2), command.target)

    def test_reaches_and_holds_final_goal(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        planner.update((0.0, 0.0, 2.3))
        planner.update((2.0, -1.0, 2.3))

        command = planner.update((2.02, -1.01, 1.24))

        self.assertEqual('REACHED', command.stage)
        self.assertEqual((2.0, -1.0, 1.2), command.target)

    def test_new_goal_restarts_from_current_xy(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, 0.0, 1.2), (0.0, 0.0, 1.2))

        planner.set_goal((1.0, -2.0, 1.2), (0.5, -0.4, 1.3))
        command = planner.update((0.5, -0.4, 1.3))

        self.assertEqual('CLIMB', command.stage)
        self.assertEqual((0.5, -0.4, 2.3), command.target)

    def test_idle_without_goal(self):
        command = StagedPathPlanner().update((0.0, 0.0, 1.2))

        self.assertEqual('IDLE', command.stage)
        self.assertIsNone(command.target)


if __name__ == '__main__':
    unittest.main()
