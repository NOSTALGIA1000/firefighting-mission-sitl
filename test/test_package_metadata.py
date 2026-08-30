from __future__ import print_function

import os
import re
import stat
import subprocess
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class PackageMetadataTest(unittest.TestCase):
    @staticmethod
    def _read(relative_path):
        with open(os.path.join(PROJECT_ROOT, relative_path), 'r') as handle:
            return handle.read()

    def test_maintainer_email_has_valid_fully_qualified_domain(self):
        package = ET.parse(os.path.join(PROJECT_ROOT, 'package.xml')).getroot()
        email = package.find('maintainer').attrib['email']
        self.assertIsNotNone(re.match(r'^[^@\s]+@[^@\s]+\.[^@\s]+$', email))

    def test_catkin_python_setup_precedes_message_generation(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        self.assertLess(cmake.index('catkin_python_setup()'),
                        cmake.index('generate_messages('))

    def test_competition_main_is_installed_and_tested(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()

        self.assertIn('scripts/competition_main.py', cmake)
        self.assertIn('catkin_add_nosetests(test/test_competition_main.py)',
                      cmake)

    def test_competition_node_connects_preflight_health_gate(self):
        node = self._read('scripts/competition_main.py')

        self.assertIn('from mavros_msgs.msg import EstimatorStatus, State', node)
        self.assertIn("'/estimator_status'", node)
        self.assertIn('self.estimator_received_at =', node)
        self.assertIn('self.imu_received_at =', node)
        self.assertIn('PreflightHealthGate(', node)
        self.assertIn('PreflightSample(', node)
        self.assertIn('sensor_ready=preflight_ready', node)

    def test_launches_configure_preflight_health_gate(self):
        expected = {
            'health_stable_seconds': '3.0',
            'health_max_message_age': '1.5',
            'health_accel_min': '5.0',
            'health_accel_max': '15.0',
        }
        for launch_name in ('competition_takeoff.launch',
                            'firefighting.launch'):
            root = ET.parse(os.path.join(
                PROJECT_ROOT, 'launch', launch_name)).getroot()
            args = {item.attrib['name']: item.attrib.get('default')
                    for item in root.findall('arg')}
            self.assertEqual(
                expected, {name: args[name] for name in expected})
            node = next(
                item for item in root.findall('node')
                if item.attrib.get('type') == 'competition_main.py')
            params = {item.attrib['name']: item.attrib.get('value')
                      for item in node.findall('param')}
            for name in expected:
                self.assertEqual('$(arg %s)' % name, params[name])

        smoke = ET.parse(os.path.join(
            PROJECT_ROOT, 'test', 'visual_avoidance_smoke.test')).getroot()
        competition = next(
            item for item in smoke.findall('node')
            if item.attrib.get('type') == 'competition_main.py')
        smoke_params = {item.attrib['name']: item.attrib.get('value')
                        for item in competition.findall('param')}
        self.assertEqual(
            expected, {name: smoke_params[name] for name in expected})

    def test_external_vision_bridge_is_installed_and_tested(self):
        cmake = self._read('CMakeLists.txt')
        start_sitl = self._read('scripts/start_sitl.sh')
        px4_post = self._read('config/px4/10016_iris.post')

        self.assertIn('scripts/gazebo_vision_bridge.py', cmake)
        self.assertIn('catkin_add_nosetests(test/test_external_vision.py)',
                      cmake)
        self.assertIn('config', cmake)
        self.assertIn('config/px4/10016_iris.post', start_sitl)
        self.assertIn('cmp -s', start_sitl)
        self.assertIn('EKF2_AID_MASK 264', px4_post)
        self.assertIn('EKF2_MAG_TYPE 0', px4_post)
        self.assertIn('EKF2_MAG_ACCLIM 5.0', px4_post)
        self.assertIn('EKF2_MAGBIAS_ID 0', px4_post)
        self.assertIn('EKF2_MAGBIAS_X 0', px4_post)
        self.assertIn('EKF2_MAGBIAS_Y 0', px4_post)
        self.assertIn('EKF2_MAGBIAS_Z 0', px4_post)
        self.assertIn('EKF2_EV_NOISE_MD 1', px4_post)
        self.assertIn('EKF2_EVP_NOISE 0.03', px4_post)
        self.assertIn('EKF2_EVV_NOISE 0.03', px4_post)
        self.assertIn('EKF2_EVP_GATE 10', px4_post)
        self.assertIn('EKF2_EVV_GATE 10', px4_post)
        self.assertIn('EKF2_HGT_MODE 3', px4_post)
        self.assertIn('EKF2_EV_DELAY 0', px4_post)

        bridge = self._read('scripts/gazebo_vision_bridge.py')
        self.assertIn('/mavros/odometry/out', bridge)
        self.assertIn("get_param('~publish_rate', 50.0)", bridge)
        self.assertIn('nav_msgs.msg import Odometry', bridge)
        self.assertIn("message.header.frame_id = 'odom'", bridge)
        self.assertIn("message.child_frame_id = 'base_link'", bridge)
        self.assertNotIn('/mavros/vision_pose/pose', bridge)
        self.assertNotIn('/mavros/vision_speed/', bridge)

        for launch_name in ('competition_takeoff.launch',
                            'firefighting.launch'):
            launch = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                           launch_name)).getroot()
            bridges = [node for node in launch.findall('node')
                       if node.attrib.get('type') ==
                       'gazebo_vision_bridge.py']
            self.assertEqual(1, len(bridges), launch_name)
            self.assertEqual('$(arg use_gazebo_ground_truth)',
                             bridges[0].attrib.get('if'))
            publish_rates = [param.attrib.get('value')
                             for param in bridges[0].findall('param')
                             if param.attrib.get('name') == 'publish_rate']
            self.assertEqual(['50.0'], publish_rates, launch_name)

        smoke = ET.parse(os.path.join(
            PROJECT_ROOT, 'test', 'visual_avoidance_smoke.test')).getroot()
        smoke_bridges = [node for node in smoke.findall('node')
                         if node.attrib.get('type') ==
                         'gazebo_vision_bridge.py']
        self.assertEqual(1, len(smoke_bridges))
        smoke_rates = [param.attrib.get('value')
                       for param in smoke_bridges[0].findall('param')
                       if param.attrib.get('name') == 'publish_rate']
        self.assertEqual(['50.0'], smoke_rates)

    def test_roslaunch_nodes_are_checked_in_as_executable(self):
        for relative_path in ('scripts/start_sitl.sh',
                              'scripts/competition_main.py',
                              'scripts/gazebo_vision_bridge.py'):
            if os.path.exists(os.path.join(PROJECT_ROOT, '.git')):
                output = subprocess.check_output(
                    ['git', 'ls-files', '--stage', relative_path],
                    cwd=PROJECT_ROOT).decode('utf-8')
                mode = output.split()[0]
                self.assertEqual('100755', mode, relative_path)
            else:
                mode = os.stat(os.path.join(PROJECT_ROOT, relative_path)).st_mode
                self.assertTrue(mode & stat.S_IXUSR, relative_path)

    def test_roslaunch_scripts_are_exported_with_lf_line_endings(self):
        attributes_path = os.path.join(PROJECT_ROOT, '.gitattributes')
        self.assertTrue(os.path.exists(attributes_path), '.gitattributes')
        with open(attributes_path, 'r') as handle:
            attributes = handle.read()

        self.assertTrue(re.search(r'^\*\s+text=auto\s+eol=lf$',
                                  attributes, re.MULTILINE))
        self.assertTrue(re.search(r'scripts/\*\s+text\s+eol=lf', attributes))
        self.assertTrue(re.search(r'\*\.py\s+text\s+eol=lf', attributes))

    def test_drop_supply_service_is_generated(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        service = os.path.join(PROJECT_ROOT, 'srv', 'DropSupply.srv')

        self.assertTrue(os.path.isfile(service))
        self.assertIn('add_service_files(FILES\n  DropSupply.srv', cmake)

    def test_path_planner_node_is_installed(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        launch = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                       'competition_takeoff.launch')).getroot()

        self.assertIn('scripts/path_planner.py', cmake)
        args = [node.attrib.get('name') for node in launch.findall('arg')]
        self.assertIn('enable_path_planner', args)

    def test_supply_drop_node_is_installed_and_launched(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        launch = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                       'firefighting.launch')).getroot()

        self.assertIn('scripts/supply_drop.py', cmake)
        node_types = [node.attrib.get('type') for node in launch.findall('node')]
        self.assertIn('supply_drop.py', node_types)

    def test_stereo_avoidance_messages_are_generated(self):
        cmake = self._read('CMakeLists.txt')
        package = self._read('package.xml')
        for name in ('ObstacleCluster.msg', 'ObstacleArray.msg',
                     'AvoidanceStatus.msg'):
            self.assertIn(name, cmake)
            self.assertTrue(os.path.isfile(os.path.join(PROJECT_ROOT, 'msg', name)))
        self.assertIn('<depend>stereo_msgs</depend>', package)
        self.assertIn('<exec_depend>stereo_image_proc</exec_depend>', package)


if __name__ == '__main__':
    unittest.main()
