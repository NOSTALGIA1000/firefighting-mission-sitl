from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.world_generator import (
    CYLINDER_POSES, FIELD_BOUNDS, FIXED_OBSTACLES,
    HAZARD_POSES, PERSON_POSES, build_scenario, generate_world,
    physical_side_clearance)


def writable_tempdir():
    root = os.path.join(PROJECT_ROOT, '.test-tmp')
    if not os.path.isdir(root):
        os.makedirs(root)
    return root


class WorldGeneratorTest(unittest.TestCase):
    def test_metric_layout_matches_figure_one(self):
        self.assertEqual((-0.65, 3.35, -3.35, 0.65, 3.0), FIELD_BOUNDS)
        expected = (
            ('fixed_obstacle_1', 0.70, -0.20, 0.0, 0.10, 1.70),
            ('fixed_obstacle_2', 2.72, 0.04, 0.785398, 1.60, 0.10),
            ('fixed_obstacle_3', 0.70, -3.10, 0.0, 0.10, 0.50),
            ('fixed_obstacle_4', 2.10, -3.10, 0.0, 0.10, 0.50),
        )
        self.assertEqual(expected, FIXED_OBSTACLES)
        self.assertEqual({1: ((0.70, -1.45), (2.10, -1.45)),
                          2: ((0.70, -2.45), (2.10, -1.95))}, CYLINDER_POSES)
        self.assertEqual({1: (1.40, 0.00), 2: (1.40, -0.45)}, HAZARD_POSES)
        self.assertEqual({1: (2.70, -1.10), 2: (2.70, -1.90),
                          3: (2.70, -2.65)}, PERSON_POSES)

    def test_each_random_cylinder_has_a_1300mm_side_passage(self):
        for candidates in CYLINDER_POSES.values():
            for x, _y in candidates:
                self.assertGreaterEqual(
                    physical_side_clearance(x, 0.10, FIELD_BOUNDS), 1.30)

    def test_physical_side_clearance_uses_outer_surface_and_wider_side(self):
        self.assertAlmostEqual(2.55, physical_side_clearance(0.70, 0.10),
                               places=10)
        custom_bounds = (-2.0, 4.0, -3.0, 3.0, 2.0)
        self.assertAlmostEqual(3.25, physical_side_clearance(0.50, 0.25,
                                                              custom_bounds),
                               places=10)

    def test_fixed_obstacles_touch_the_required_edges(self):
        obstacle_1 = FIXED_OBSTACLES[0]
        obstacle_3 = FIXED_OBSTACLES[2]
        obstacle_4 = FIXED_OBSTACLES[3]
        self.assertAlmostEqual(0.65, obstacle_1[2] + obstacle_1[5] / 2.0)
        self.assertAlmostEqual(-3.35, obstacle_3[2] - obstacle_3[5] / 2.0)
        self.assertAlmostEqual(-3.35, obstacle_4[2] - obstacle_4[5] / 2.0)

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

    def test_task_zones_reference_rendered_recognition_materials(self):
        output = os.path.join(writable_tempdir(), 'textured.world')
        scenario = generate_world(4501, output)
        root = ET.parse(output).getroot()
        names = [node.text for node in root.findall('.//material/script/name')]
        expected_hazard = 'FireTargets/%s' % scenario.hazard_symbol.title()
        self.assertIn(expected_hazard, names)
        self.assertIn('FireTargets/Distractor', names)
        self.assertIn('FireTargets/Person', names)
        for model_name in ('hazard_zone_1', 'hazard_zone_2', 'person_zone'):
            model = root.find(".//model[@name='%s']" % model_name)
            self.assertEqual(2, len(model.findall('.//visual')))

    def test_seeded_layout_has_one_correct_hazard_and_one_distractor(self):
        output = os.path.join(writable_tempdir(), 'seeded-zones.world')
        scenario = generate_world(4501, output)
        root = ET.parse(output).getroot()
        hazard_materials = []
        for index in (1, 2):
            zone = root.find(".//model[@name='hazard_zone_%d']" % index)
            hazard_materials.append(zone.find('.//material/script/name').text)
            self.assertEqual('0.9 0.05 0.05 1',
                             zone.find('.//visual[@name="border"]/material/ambient').text)
        self.assertEqual(sorted(['FireTargets/%s' % scenario.hazard_symbol.title(),
                                 'FireTargets/Distractor']),
                         sorted(hazard_materials))

        person = root.find(".//model[@name='person_zone']")
        self.assertEqual('FireTargets/Person',
                         person.find('.//material/script/name').text)
        self.assertEqual('0.05 0.75 0.95 1',
                         person.find('.//visual[@name="border"]/material/ambient').text)

    def test_every_perception_template_is_packaged_for_gazebo(self):
        labels = ('flammable', 'explosive', 'toxic', 'distractor', 'person')
        for label in labels:
            source = os.path.join(PROJECT_ROOT, 'assets', 'templates', label + '.png')
            texture = os.path.join(PROJECT_ROOT, 'models', 'targets',
                                   'materials', 'textures', label + '.png')
            self.assertTrue(os.path.isfile(texture))
            with open(source, 'rb') as source_file:
                source_bytes = source_file.read()
            with open(texture, 'rb') as texture_file:
                self.assertEqual(source_bytes, texture_file.read())

    def test_sitl_exports_package_models_to_gazebo(self):
        wrapper = os.path.join(PROJECT_ROOT, 'scripts', 'start_sitl.sh')
        with open(wrapper, 'r') as handle:
            text = handle.read()
        self.assertIn('$package_root/models', text)
        self.assertIn('GAZEBO_MODEL_PATH', text)


if __name__ == '__main__':
    unittest.main()
