from __future__ import print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.payload import PayloadController


class PayloadControllerTest(unittest.TestCase):
    def test_each_channel_can_release_exactly_once(self):
        controller = PayloadController()

        first = controller.request(1, aligned=True, horizontal_speed=0.05,
                                   altitude=1.30)
        repeated = controller.request(1, aligned=True, horizontal_speed=0.0,
                                      altitude=1.30)
        second = controller.request(2, aligned=True, horizontal_speed=0.05,
                                    altitude=1.30)

        self.assertTrue(first.accepted)
        self.assertFalse(repeated.accepted)
        self.assertEqual('already_released', repeated.reason)
        self.assertTrue(second.accepted)

    def test_invalid_channel_is_rejected(self):
        result = PayloadController().request(3, aligned=True,
                                             horizontal_speed=0.0,
                                             altitude=1.30)
        self.assertFalse(result.accepted)
        self.assertEqual('invalid_channel', result.reason)

    def test_unaligned_aircraft_cannot_release(self):
        result = PayloadController().request(1, aligned=False,
                                             horizontal_speed=0.0,
                                             altitude=1.30)
        self.assertFalse(result.accepted)
        self.assertEqual('not_aligned', result.reason)

    def test_horizontal_speed_above_ten_centimeters_per_second_is_rejected(self):
        result = PayloadController().request(1, aligned=True,
                                             horizontal_speed=0.101,
                                             altitude=1.30)
        self.assertFalse(result.accepted)
        self.assertEqual('moving_too_fast', result.reason)

    def test_altitude_outside_release_window_is_rejected(self):
        low = PayloadController().request(1, aligned=True,
                                          horizontal_speed=0.0,
                                          altitude=1.14)
        high = PayloadController().request(2, aligned=True,
                                           horizontal_speed=0.0,
                                           altitude=1.46)

        self.assertFalse(low.accepted)
        self.assertFalse(high.accepted)
        self.assertEqual('altitude_out_of_range', low.reason)
        self.assertEqual('altitude_out_of_range', high.reason)


if __name__ == '__main__':
    unittest.main()
