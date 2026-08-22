from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.competition_main import (CompetitionMain,
                                                    ModeRequest,
                                                    PositionSetpoint,
                                                    mission_interface_topics)


class CompetitionMainTest(unittest.TestCase):
    def test_waits_for_fcu_before_publishing_takeoff_control(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        outputs = controller.tick(0.0, connected=False, armed=False,
                                  mode='', altitude=0.0)

        self.assertEqual('WAIT_FCU', outputs.state)
        self.assertEqual([], outputs.setpoints)
        self.assertEqual([], outputs.mode_requests)
        self.assertFalse(outputs.arm_request)

    def test_prestreams_position_setpoints_before_offboard_and_arm(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        first = controller.tick(0.0, connected=True, armed=False,
                                mode='', altitude=0.0)
        second = controller.tick(0.1, connected=True, armed=False,
                                 mode='', altitude=0.0)
        third = controller.tick(0.2, connected=True, armed=False,
                                mode='', altitude=0.0)

        self.assertEqual('PRESTREAM_SETPOINTS', first.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), first.setpoints[0])
        self.assertEqual([], first.mode_requests)
        self.assertEqual([], second.mode_requests)
        self.assertEqual([ModeRequest('OFFBOARD')], third.mode_requests)
        self.assertFalse(third.arm_request)

    def test_requests_arm_after_offboard_mode_is_active(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1)
        controller.tick(0.0, connected=True, armed=False, mode='',
                        altitude=0.0)

        outputs = controller.tick(0.1, connected=True, armed=False,
                                  mode='OFFBOARD', altitude=0.0)

        self.assertEqual('ARM', outputs.state)
        self.assertTrue(outputs.arm_request)

    def test_retries_offboard_request_until_mode_becomes_active(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1,
                                     mode_retry_seconds=1.0)

        first = controller.tick(0.0, connected=True, armed=False, mode='',
                                altitude=0.0)
        too_soon = controller.tick(0.5, connected=True, armed=False, mode='',
                                   altitude=0.0)
        retry = controller.tick(1.1, connected=True, armed=False, mode='',
                                altitude=0.0)

        self.assertEqual([ModeRequest('OFFBOARD')], first.mode_requests)
        self.assertEqual([], too_soon.mode_requests)
        self.assertEqual([ModeRequest('OFFBOARD')], retry.mode_requests)

    def test_takeoff_reaches_hover_when_target_altitude_is_stable(self):
        controller = CompetitionMain(takeoff_altitude=1.2, hover_tolerance=0.08,
                                     hover_hold_seconds=0.3,
                                     prestream_count=1)
        controller.tick(0.0, connected=True, armed=False, mode='',
                        altitude=0.0)
        controller.tick(0.1, connected=True, armed=False, mode='OFFBOARD',
                        altitude=0.0)

        takeoff = controller.tick(0.2, connected=True, armed=True,
                                  mode='OFFBOARD', altitude=0.8)
        hover_pending = controller.tick(0.3, connected=True, armed=True,
                                        mode='OFFBOARD', altitude=1.18)
        hover = controller.tick(0.7, connected=True, armed=True,
                                mode='OFFBOARD', altitude=1.19)

        self.assertEqual('TAKEOFF', takeoff.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), takeoff.setpoints[0])
        self.assertEqual('TAKEOFF', hover_pending.state)
        self.assertEqual('HOVER', hover.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), hover.setpoints[0])

    def test_state_machine_exposes_placeholder_integration_topics(self):
        topics = mission_interface_topics()

        self.assertEqual('/hazard_detected', topics['hazard_detected'])
        self.assertEqual('/person_detected', topics['person_detected'])
        self.assertEqual('/drop_fire_payload', topics['drop_fire_payload'])
        self.assertEqual('/drop_rescue_payload', topics['drop_rescue_payload'])

    def test_takeoff_launch_runs_only_sitl_and_competition_main(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        node_types = [node.attrib.get('type') for node in root.findall('node')]

        self.assertIn('start_sitl.sh', node_types)
        self.assertIn('competition_main.py', node_types)
        self.assertNotIn('mission_manager_node.py', node_types)
        self.assertNotIn('navigator_node.py', node_types)
        competition = next(node for node in root.findall('node')
                           if node.attrib.get('type') == 'competition_main.py')
        self.assertEqual('true', competition.attrib.get('required'))

    def test_takeoff_launch_uses_px4_native_iris_sdf_for_flight_smoke(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        sitl = next(node for node in root.findall('node')
                    if node.attrib.get('type') == 'start_sitl.sh')

        self.assertIn('$(arg sdf)', sitl.attrib.get('args'))
        sdf_arg = next(arg for arg in root.findall('arg')
                       if arg.attrib.get('name') == 'sdf')
        self.assertIn('Tools/sitl_gazebo/models/iris/iris.sdf',
                      sdf_arg.attrib.get('default'))


if __name__ == '__main__':
    unittest.main()
