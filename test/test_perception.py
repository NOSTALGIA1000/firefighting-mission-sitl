from __future__ import print_function

import os
import sys
import unittest

import cv2
import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.perception import (StableDetector, TemplatePerception,
                                             annotate, annotation_color)


def icon(kind):
    image = np.zeros((40, 40), dtype=np.uint8)
    if kind == 'flammable':
        points = np.array([[20, 3], [35, 34], [5, 34]], dtype=np.int32)
        cv2.polylines(image, [points], True, 255, 3)
        cv2.circle(image, (20, 24), 6, 255, -1)
    elif kind == 'person':
        cv2.circle(image, (20, 12), 7, 255, -1)
        cv2.rectangle(image, (10, 21), (30, 37), 255, -1)
    elif kind == 'distractor':
        cv2.line(image, (5, 5), (35, 35), 255, 4)
        cv2.line(image, (35, 5), (5, 35), 255, 4)
    return image


def scene(template, origin=(60, 50)):
    image = np.zeros((160, 180), dtype=np.uint8)
    x, y = origin
    image[y:y + template.shape[0], x:x + template.shape[1]] = template
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


class PerceptionTest(unittest.TestCase):
    def setUp(self):
        self.templates = {
            'flammable': icon('flammable'),
            'person': icon('person'),
            'distractor': icon('distractor'),
        }
        self.perception = TemplatePerception(self.templates, threshold=0.80,
                                             scales=(1.0,))

    def test_hazard_template_is_detected_in_hazard_phase(self):
        result = self.perception.detect(scene(self.templates['flammable']),
                                        'SEARCH_HAZARD')

        self.assertEqual('flammable', result.target_class)
        self.assertGreaterEqual(result.confidence, 0.99)
        self.assertEqual((60, 50, 40, 40), result.box)

    def test_person_is_ignored_during_hazard_search(self):
        result = self.perception.detect(scene(self.templates['person']),
                                        'SEARCH_HAZARD')

        self.assertEqual('none', result.target_class)

    def test_distractor_is_reported_but_not_as_hazard(self):
        result = self.perception.detect(scene(self.templates['distractor']),
                                        'SEARCH_HAZARD')

        self.assertEqual('distractor', result.target_class)

    def test_confirmation_requires_four_matches_in_five_frames(self):
        stable = StableDetector(window=5, required=4)

        states = [stable.update('flammable', matched)
                  for matched in (True, True, False, True, True)]

        self.assertFalse(any(states[:4]))
        self.assertTrue(states[4])
        self.assertEqual(4, stable.confirmation_count)

    def test_class_change_resets_confirmation(self):
        stable = StableDetector(window=5, required=4)
        for _ in range(3):
            stable.update('flammable', True)

        confirmed = stable.update('person', True)

        self.assertFalse(confirmed)
        self.assertEqual(1, stable.confirmation_count)

    def test_annotations_use_required_red_and_blue_bgr_colors(self):
        hazard = annotate(np.zeros((80, 80, 3), dtype=np.uint8),
                          (10, 10, 20, 20), 'flammable', 1.0)
        person = annotate(np.zeros((80, 80, 3), dtype=np.uint8),
                          (10, 10, 20, 20), 'person', 1.0)

        self.assertEqual((0, 0, 255), annotation_color('flammable'))
        self.assertEqual((255, 0, 0), annotation_color('person'))
        self.assertTrue(np.array_equal(hazard[10, 10], np.array([0, 0, 255])))
        self.assertTrue(np.array_equal(person[10, 10], np.array([255, 0, 0])))


if __name__ == '__main__':
    unittest.main()
