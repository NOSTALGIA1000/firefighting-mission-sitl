from __future__ import print_function

import json
import os


class Score(object):
    """Machine-readable evaluation of the mission's non-negotiable rules."""

    def __init__(self, seed, runtime_seconds, minimum_clearance_m,
                 hazard_identified, person_identified, fire_drop_error_m,
                 rescue_drop_error_m, landing_error_m, disarmed, collision,
                 completed):
        self.seed = int(seed)
        self.runtime_seconds = float(runtime_seconds)
        self.minimum_clearance_m = float(minimum_clearance_m)
        self.hazard_identified = bool(hazard_identified)
        self.person_identified = bool(person_identified)
        self.fire_drop_error_m = float(fire_drop_error_m)
        self.rescue_drop_error_m = float(rescue_drop_error_m)
        self.landing_error_m = float(landing_error_m)
        self.disarmed = bool(disarmed)
        self.collision = bool(collision)
        self.completed = bool(completed)

    @property
    def failure_reasons(self):
        reasons = []
        if self.runtime_seconds > 180.0:
            reasons.append('runtime_over_180_seconds')
        if self.minimum_clearance_m < 0.35:
            reasons.append('clearance_below_0_35_m')
        if not self.hazard_identified:
            reasons.append('hazard_not_identified')
        if not self.person_identified:
            reasons.append('person_not_identified')
        if self.fire_drop_error_m > 0.20:
            reasons.append('fire_drop_outside_zone')
        if self.rescue_drop_error_m > 0.20:
            reasons.append('rescue_drop_outside_zone')
        if self.landing_error_m > 0.25:
            reasons.append('landing_outside_start_zone')
        if not self.disarmed:
            reasons.append('not_disarmed')
        if self.collision:
            reasons.append('collision')
        if not self.completed:
            reasons.append('mission_not_complete')
        return reasons

    @property
    def passed(self):
        return not self.failure_reasons

    def to_dict(self):
        return {
            'seed': self.seed,
            'runtime_seconds': self.runtime_seconds,
            'minimum_clearance_m': self.minimum_clearance_m,
            'hazard_identified': self.hazard_identified,
            'person_identified': self.person_identified,
            'fire_drop_error_m': self.fire_drop_error_m,
            'rescue_drop_error_m': self.rescue_drop_error_m,
            'landing_error_m': self.landing_error_m,
            'disarmed': self.disarmed,
            'collision': self.collision,
            'completed': self.completed,
            'failure_reasons': self.failure_reasons,
            'passed': self.passed,
        }


def valid_score(runtime, all_other_conditions=True):
    """Compatibility helper for simple hard-deadline checks."""
    return float(runtime) <= 180.0 and bool(all_other_conditions)


def write_score(score, output_path):
    """Write a complete score without exposing a partial JSON file."""
    directory = os.path.dirname(output_path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    payload = score.to_dict()
    temporary = output_path + '.tmp'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
    os.rename(temporary, output_path)
