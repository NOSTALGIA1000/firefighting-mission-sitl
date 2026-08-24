from __future__ import print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.supply_drop import SupplyDropController


class SupplyDropControllerTest(unittest.TestCase):
    @staticmethod
    def accepted(_channel):
        return True, ''

    def test_each_channel_can_release_once(self):
        controller = SupplyDropController()

        fire = controller.request(1, True, 0.0, 1.25, self.accepted)
        rescue = controller.request(2, True, 0.0, 1.25, self.accepted)

        self.assertTrue(fire.success)
        self.assertTrue(rescue.success)
        repeated = controller.request(1, True, 0.0, 1.25, self.accepted)
        self.assertFalse(repeated.success)
        self.assertEqual('already_released', repeated.reason)

    def test_invalid_channel_is_rejected(self):
        result = SupplyDropController().request(
            3, True, 0.0, 1.25, self.accepted)

        self.assertFalse(result.success)
        self.assertEqual('invalid_channel', result.reason)

    def test_unaligned_aircraft_is_rejected(self):
        result = SupplyDropController().request(
            1, False, 0.0, 1.25, self.accepted)

        self.assertFalse(result.success)
        self.assertEqual('not_aligned', result.reason)

    def test_horizontal_speed_above_limit_is_rejected(self):
        result = SupplyDropController().request(
            1, True, 0.101, 1.25, self.accepted)

        self.assertFalse(result.success)
        self.assertEqual('moving_too_fast', result.reason)

    def test_altitude_outside_window_is_rejected(self):
        controller = SupplyDropController()

        low = controller.request(1, True, 0.0, 1.149, self.accepted)
        high = controller.request(2, True, 0.0, 1.451, self.accepted)

        self.assertEqual('altitude_out_of_range', low.reason)
        self.assertEqual('altitude_out_of_range', high.reason)

    def test_low_level_failure_does_not_consume_channel(self):
        controller = SupplyDropController()
        failed = controller.request(
            1, True, 0.0, 1.25,
            lambda _channel: (False, 'plugin_failed'))

        retried = controller.request(
            1, True, 0.0, 1.25,
            lambda _channel: (True, ''))

        self.assertFalse(failed.success)
        self.assertEqual('plugin_failed', failed.reason)
        self.assertTrue(retried.success)


if __name__ == '__main__':
    unittest.main()
