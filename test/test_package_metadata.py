from __future__ import print_function

import os
import re
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


if __name__ == '__main__':
    unittest.main()
