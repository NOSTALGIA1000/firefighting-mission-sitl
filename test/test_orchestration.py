from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.orchestration import (contacts_indicate_collision,
                                                 payload_link_names,
                                                 validated_alignment)
from firefighting_mission.mavros_bridge import command_actions, flu_to_enu


class OrchestrationTest(unittest.TestCase):
    def test_validated_alignment_requires_target_confirmation_and_reached_goal(self):
        self.assertTrue(validated_alignment('ALIGN_HAZARD', True, 'REACHED'))
        self.assertTrue(validated_alignment('DROP_RESCUE', True, 'REACHED'))
        self.assertFalse(validated_alignment('SEARCH_HAZARD', True, 'REACHED'))
        self.assertFalse(validated_alignment('ALIGN_PERSON', False, 'REACHED'))
        self.assertFalse(validated_alignment('ALIGN_PERSON', True, 'ACTIVE'))

    def test_payload_links_use_the_spawned_iris_model_name(self):
        self.assertEqual(('iris_0::fire_payload_link', 'fire_payload_link'),
                         payload_link_names(1))
        self.assertEqual(('iris_0::rescue_payload_link', 'rescue_payload_link'),
                         payload_link_names(2))

    def test_nonempty_contact_states_are_a_collision(self):
        self.assertFalse(contacts_indicate_collision([]))
        self.assertTrue(contacts_indicate_collision([object()]))

    def test_flu_velocity_is_rotated_into_mavros_enu(self):
        east, north = flu_to_enu(1.0, 0.0, 1.5707963267948966)

        self.assertAlmostEqual(0.0, east, places=6)
        self.assertAlmostEqual(1.0, north, places=6)

    def test_xtdrone_commands_have_mavros_actions(self):
        self.assertEqual(('OFFBOARD', 'ARM'), command_actions('ARM'))
        self.assertEqual(('OFFBOARD',), command_actions('OFFBOARD'))
        self.assertEqual(('AUTO.LAND',), command_actions('AUTO.LAND'))
        self.assertEqual(('DISARM',), command_actions('DISARM'))

    def test_vehicle_contact_sensor_publishes_mission_contacts(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'models', 'fire_iris',
                                     'fire_iris.sdf')).getroot()
        sensor = root.find(".//link[@name='base_link']/sensor[@type='contact']")

        self.assertIsNotNone(sensor)
        plugin = sensor.find("plugin[@filename='libgazebo_ros_bumper.so']")
        self.assertIsNotNone(plugin)
        self.assertEqual('/fire_mission/contacts',
                         plugin.find('bumperTopicName').text)

    def test_launch_uses_required_completion_owner_and_synchronous_sitl_wrapper(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'firefighting.launch')).getroot()
        nodes = root.findall('node')
        manager = next(node for node in nodes
                       if node.attrib.get('type') == 'mission_manager_node.py')
        sitl = next(node for node in nodes
                    if node.attrib.get('type') == 'start_sitl.sh')

        self.assertEqual('true', manager.attrib.get('required'))
        self.assertEqual('true', sitl.attrib.get('required'))
        self.assertFalse(any(node.attrib.get('type') == 'generate_world.py'
                             for node in nodes))

        with open(os.path.join(PROJECT_ROOT, 'scripts', 'start_sitl.sh'), 'r') as handle:
            wrapper = handle.read()
        self.assertLess(wrapper.index('generate_world.py'), wrapper.index('exec roslaunch'))
        self.assertIn('PX4_FIRMWARE_DIR', wrapper)
        self.assertIn('Tools/sitl_gazebo', wrapper)
        self.assertIn('Tools/setup_gazebo.bash', wrapper)
        self.assertIn('build/px4_sitl_default', wrapper)
        self.assertIn('GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"', wrapper)
        self.assertIn('GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"', wrapper)
        self.assertIn('mavros_posix_sitl.launch', wrapper)

    def test_completion_waits_for_recorder_before_required_manager_exits(self):
        with open(os.path.join(PROJECT_ROOT, 'scripts',
                               'mission_manager_node.py'), 'r') as handle:
            manager = handle.read()

        self.assertIn('rospy.Duration(1.0), self._shutdown_after_completion', manager)
        self.assertIn("rospy.signal_shutdown('mission complete')", manager)

    def test_launch_wires_the_iris_scan_topic_to_every_scan_consumer(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'firefighting.launch')).getroot()
        scan_nodes = [node for node in root.findall('node')
                      if node.attrib.get('type') in (
                          'navigator_node.py', 'safety_monitor_node.py',
                          'mission_recorder_node.py')]

        self.assertEqual(3, len(scan_nodes))
        for node in scan_nodes:
            scan = node.find("param[@name='scan_topic']")
            self.assertIsNotNone(scan)
            self.assertEqual('$(arg scan_topic)', scan.attrib.get('value'))

    def test_sitl_wrapper_preserves_upstream_mavros_udp_defaults(self):
        with open(os.path.join(PROJECT_ROOT, 'scripts', 'start_sitl.sh'), 'r') as handle:
            wrapper = handle.read()

        self.assertNotIn('fcu_url:=tcp://localhost:4560', wrapper)
        self.assertNotIn('respawn_mavros:=true', wrapper)


if __name__ == '__main__':
    unittest.main()
