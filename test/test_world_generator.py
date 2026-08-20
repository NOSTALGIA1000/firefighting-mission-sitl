from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.world_generator import build_scenario, generate_world


def writable_tempdir():
    root = os.path.join(PROJECT_ROOT, '.test-tmp')
    if not os.path.isdir(root):
        os.makedirs(root)
    return root


class WorldGeneratorTest(unittest.TestCase):
    def test_seed_is_reproducible(self):
        root = writable_tempdir()
        first_path = os.path.join(root, 'first.world')
        second_path = os.path.join(root, 'second.world')

        first = generate_world(4501, first_path)
        second = generate_world(4501, second_path)

        self.assertEqual(first, second)
        with open(first_path, 'rb') as handle:
            first_xml = handle.read()
        with open(second_path, 'rb') as handle:
            second_xml = handle.read()
        self.assertEqual(first_xml, second_xml)

    def test_supported_seeds_cover_randomized_choices(self):
        scenarios = [build_scenario(seed) for seed in (4501, 4502, 4503, 4504)]

        self.assertGreater(len(set(item.cylinder_positions for item in scenarios)), 1)
        self.assertGreater(len(set(item.person_position for item in scenarios)), 1)
        for scenario in scenarios:
            self.assertEqual((4.0, 4.0, 3.0), scenario.bounds)
            self.assertEqual(2, len(scenario.cylinder_positions))
            self.assertTrue(all(position in (1, 2)
                                for position in scenario.cylinder_positions))
            self.assertIn(scenario.hazard_index, (1, 2))
            self.assertIn(scenario.hazard_symbol,
                          ('flammable', 'explosive', 'toxic'))
            self.assertIn(scenario.person_position, (1, 2, 3))

    def test_generated_world_contains_competition_geometry(self):
        output = os.path.join(writable_tempdir(), 'scene.world')
        generate_world(4501, output)
        root = ET.parse(output).getroot()
        models = {model.attrib['name']: model for model in root.findall('.//model')}

        self.assertTrue(all('fixed_obstacle_%d' % index in models
                            for index in range(1, 5)))
        self.assertTrue(all('random_cylinder_%d' % index in models
                            for index in range(1, 3)))
        self.assertTrue(all('hazard_zone_%d' % index in models
                            for index in range(1, 3)))
        self.assertIn('person_zone', models)
        self.assertIn('start_zone', models)

        cylinder_size = models['random_cylinder_1'].find(
            './/geometry/cylinder')
        self.assertAlmostEqual(0.1, float(cylinder_size.find('radius').text))
        self.assertAlmostEqual(2.0, float(cylinder_size.find('length').text))

        hazard_size = models['hazard_zone_1'].find('.//geometry/box/size').text
        self.assertEqual('0.4 0.4 0.01', hazard_size)

    def test_start_zone_is_five_hundred_millimeters_diameter(self):
        output = os.path.join(writable_tempdir(), 'scene.world')
        generate_world(4502, output)
        root = ET.parse(output).getroot()
        start = root.find(".//model[@name='start_zone']")

        self.assertEqual('0.25', start.find('.//geometry/cylinder/radius').text)


if __name__ == '__main__':
    unittest.main()
