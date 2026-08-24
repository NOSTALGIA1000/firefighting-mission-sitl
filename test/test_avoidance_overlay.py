from __future__ import print_function

from collections import namedtuple
import os
import sys
import unittest

import cv2
import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.avoidance_overlay import (compose_mission_view,
                                                     draw_avoidance_overlay)


Status = namedtuple('Status', (
    'state selected_side left_clearance_m right_clearance_m reason'))


class AvoidanceOverlayTest(unittest.TestCase):
    def setUp(self):
        self.status = Status('SIDESTEP', 'LEFT', 1.30, 0.65, 'cylinder_ahead')

    def test_overlay_adds_machine_state_without_mutating_input(self):
        image = np.zeros((120, 240, 3), dtype=np.uint8)
        original = image.copy()

        output = draw_avoidance_overlay(image, self.status, [object()])

        self.assertTrue(np.array_equal(image, original))
        self.assertGreater(int(np.count_nonzero(output)), 0)
        self.assertEqual(image.shape, output.shape)

    def test_target_view_stays_primary_with_front_picture_in_picture(self):
        target = np.full((200, 300, 3), (0, 0, 180), dtype=np.uint8)
        front = np.full((100, 160, 3), (0, 180, 0), dtype=np.uint8)

        output = compose_mission_view(target, front, self.status, ())

        self.assertEqual(target.shape, output.shape)
        self.assertGreater(int(output[60, 280, 1]), int(output[60, 280, 2]))
        self.assertGreater(int(output[170, 20, 2]), int(output[170, 20, 1]))

    def test_front_view_becomes_primary_when_target_view_missing(self):
        front = np.full((100, 160, 3), 40, dtype=np.uint8)

        output = compose_mission_view(None, front, self.status, ())

        self.assertEqual(front.shape, output.shape)


if __name__ == '__main__':
    unittest.main()
