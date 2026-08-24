from __future__ import print_function

import os
import re
import stat
import subprocess
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class PackageMetadataTest(unittest.TestCase):
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

    def test_roslaunch_nodes_are_checked_in_as_executable(self):
        for relative_path in ('scripts/start_sitl.sh',
                              'scripts/competition_main.py'):
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

        self.assertTrue(re.search(r'scripts/\*\s+text\s+eol=lf', attributes))

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


if __name__ == '__main__':
    unittest.main()
