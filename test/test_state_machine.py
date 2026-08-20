from __future__ import print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.state_machine import Inputs, MissionStateMachine


class MissionStateMachineTest(unittest.TestCase):
    def test_nominal_mission_reaches_complete_in_order(self):
        sm = MissionStateMachine(start_time=0.0)
        expected = [
            ('ARM', Inputs(ready=True)),
            ('TAKEOFF', Inputs(armed=True, offboard=True)),
            ('SEARCH_HAZARD', Inputs(airborne=True, goal_reached=True)),
            ('ALIGN_HAZARD', Inputs(detection_class='hazard', detection_confirmed=True)),
            ('DROP_FIRE', Inputs(detection_class='hazard', detection_confirmed=True,
                                 aligned=True)),
            ('SEARCH_PERSON', Inputs(drop_channel=1, drop_succeeded=True)),
            ('ALIGN_PERSON', Inputs(detection_class='person', detection_confirmed=True)),
            ('DROP_RESCUE', Inputs(detection_class='person', detection_confirmed=True,
                                   aligned=True)),
            ('RETURN_HOME', Inputs(drop_channel=2, drop_succeeded=True)),
            ('LAND', Inputs(home_reached=True)),
            ('DISARM', Inputs(landed=True)),
            ('COMPLETE', Inputs(disarmed=True)),
        ]

        for now, (phase, inputs) in enumerate(expected, start=1):
            command = sm.tick(float(now), inputs)
            self.assertEqual(phase, command.phase)

    def test_return_deadline_preempts_search(self):
        sm = MissionStateMachine(start_time=0.0, phase='SEARCH_PERSON')

        command = sm.tick(165.0, Inputs(airborne=True))

        self.assertEqual('RETURN_HOME', command.phase)
        self.assertEqual('return_deadline', command.reason)

    def test_hard_deadline_preempts_return_with_emergency_land(self):
        sm = MissionStateMachine(start_time=0.0, phase='RETURN_HOME')

        command = sm.tick(175.0, Inputs(airborne=True))

        self.assertEqual('EMERGENCY_LAND', command.phase)
        self.assertEqual('hard_deadline', command.reason)

    def test_drop_requires_matching_stable_detection_and_alignment(self):
        sm = MissionStateMachine(start_time=0.0, phase='ALIGN_HAZARD')

        not_confirmed = sm.tick(10.0, Inputs(detection_class='hazard', aligned=True))
        wrong_class = sm.tick(11.0, Inputs(detection_class='person',
                                          detection_confirmed=True, aligned=True))

        self.assertEqual(0, not_confirmed.drop_channel)
        self.assertEqual('ALIGN_HAZARD', not_confirmed.phase)
        self.assertEqual(0, wrong_class.drop_channel)
        self.assertEqual('ALIGN_HAZARD', wrong_class.phase)

    def test_stale_pose_enters_hover_recovery(self):
        sm = MissionStateMachine(start_time=0.0, phase='SEARCH_HAZARD')

        command = sm.tick(12.0, Inputs(airborne=True, pose_stale=True))

        self.assertEqual('HOVER_RECOVERY', command.phase)
        self.assertEqual('pose_stale', command.reason)

    def test_recovery_returns_to_interrupted_phase(self):
        sm = MissionStateMachine(start_time=0.0, phase='SEARCH_HAZARD')
        sm.tick(12.0, Inputs(airborne=True, pose_stale=True))

        command = sm.tick(12.5, Inputs(airborne=True, recovered=True))

        self.assertEqual('SEARCH_HAZARD', command.phase)


if __name__ == '__main__':
    unittest.main()
