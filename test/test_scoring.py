from __future__ import print_function

import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.scoring import Score, write_score


def passing_score(**overrides):
    values = dict(
        seed=4501,
        runtime_seconds=142.0,
        minimum_clearance_m=0.41,
        hazard_identified=True,
        person_identified=True,
        fire_drop_error_m=0.11,
        rescue_drop_error_m=0.13,
        landing_error_m=0.17,
        disarmed=True,
        collision=False,
        completed=True,
    )
    values.update(overrides)
    return Score(**values)


class ScoringTest(unittest.TestCase):
    def test_all_hard_conditions_produce_pass(self):
        score = passing_score()

        self.assertTrue(score.passed)
        self.assertEqual([], score.failure_reasons)
        self.assertTrue(score.to_dict()['passed'])

    def test_runtime_over_180_seconds_fails(self):
        score = passing_score(runtime_seconds=180.01)

        self.assertFalse(score.passed)
        self.assertIn('runtime_over_180_seconds', score.failure_reasons)

    def test_clearance_below_35_centimeters_fails(self):
        score = passing_score(minimum_clearance_m=0.349)

        self.assertFalse(score.passed)
        self.assertIn('clearance_below_0_35_m', score.failure_reasons)

    def test_drop_error_above_20_centimeters_fails(self):
        score = passing_score(fire_drop_error_m=0.201)

        self.assertFalse(score.passed)
        self.assertIn('fire_drop_outside_zone', score.failure_reasons)

    def test_collision_or_armed_finish_fails(self):
        score = passing_score(collision=True, disarmed=False)

        self.assertFalse(score.passed)
        self.assertIn('collision', score.failure_reasons)
        self.assertIn('not_disarmed', score.failure_reasons)

    def test_score_is_written_atomically_as_json(self):
        temporary_root = os.path.join(PROJECT_ROOT, '.test-tmp')
        if not os.path.isdir(temporary_root):
            os.makedirs(temporary_root)
        output = os.path.join(temporary_root, 'score-atomic.json')
        temporary = output + '.tmp'
        for path in (output, temporary):
            if os.path.exists(path):
                os.unlink(path)

        write_score(passing_score(), output)

        self.assertTrue(os.path.isfile(output))
        self.assertFalse(os.path.exists(output + '.tmp'))
        with open(output, 'r') as handle:
            payload = handle.read()
        self.assertIn('"passed": true', payload)


if __name__ == '__main__':
    unittest.main()
