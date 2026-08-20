from __future__ import print_function

from collections import namedtuple


class Inputs(object):
    _defaults = {
        'ready': False,
        'armed': False,
        'offboard': False,
        'airborne': False,
        'goal_reached': False,
        'detection_class': '',
        'detection_confirmed': False,
        'aligned': False,
        'drop_channel': 0,
        'drop_succeeded': False,
        'home_reached': False,
        'landed': False,
        'disarmed': False,
        'pose_stale': False,
        'recovered': False,
    }

    def __init__(self, **values):
        unknown = set(values) - set(self._defaults)
        if unknown:
            raise TypeError('unknown mission inputs: %s' % sorted(unknown))
        for name, default in self._defaults.items():
            setattr(self, name, values.get(name, default))


Command = namedtuple('Command', 'phase drop_channel reason')


class MissionStateMachine(object):
    RETURN_PHASES = frozenset((
        'RETURN_HOME', 'LAND', 'DISARM', 'COMPLETE', 'EMERGENCY_LAND'
    ))

    def __init__(self, start_time, phase='WAIT_READY'):
        self.start_time = float(start_time)
        self.phase = phase
        self.interrupted_phase = None
        self.reason = ''

    def _command(self):
        channel = 1 if self.phase == 'DROP_FIRE' else 2 if self.phase == 'DROP_RESCUE' else 0
        return Command(self.phase, channel, self.reason)

    def _transition(self, phase, reason=''):
        self.phase = phase
        self.reason = reason
        return self._command()

    def tick(self, now, inputs):
        elapsed = float(now) - self.start_time
        if elapsed >= 175.0 and inputs.airborne:
            return self._transition('EMERGENCY_LAND', 'hard_deadline')
        if elapsed >= 165.0 and self.phase not in self.RETURN_PHASES:
            return self._transition('RETURN_HOME', 'return_deadline')

        if inputs.pose_stale and inputs.airborne and self.phase != 'HOVER_RECOVERY':
            self.interrupted_phase = self.phase
            return self._transition('HOVER_RECOVERY', 'pose_stale')
        if self.phase == 'HOVER_RECOVERY':
            if inputs.recovered and self.interrupted_phase:
                phase = self.interrupted_phase
                self.interrupted_phase = None
                return self._transition(phase)
            return self._command()

        if self.phase == 'WAIT_READY' and inputs.ready:
            return self._transition('ARM')
        if self.phase == 'ARM' and inputs.armed and inputs.offboard:
            return self._transition('TAKEOFF')
        if self.phase == 'TAKEOFF' and inputs.airborne and inputs.goal_reached:
            return self._transition('SEARCH_HAZARD')
        if (self.phase == 'SEARCH_HAZARD' and inputs.detection_confirmed and
                inputs.detection_class == 'hazard'):
            return self._transition('ALIGN_HAZARD')
        if (self.phase == 'ALIGN_HAZARD' and inputs.detection_confirmed and
                inputs.detection_class == 'hazard' and inputs.aligned):
            return self._transition('DROP_FIRE')
        if self.phase == 'DROP_FIRE' and inputs.drop_channel == 1 and inputs.drop_succeeded:
            return self._transition('SEARCH_PERSON')
        if (self.phase == 'SEARCH_PERSON' and inputs.detection_confirmed and
                inputs.detection_class == 'person'):
            return self._transition('ALIGN_PERSON')
        if (self.phase == 'ALIGN_PERSON' and inputs.detection_confirmed and
                inputs.detection_class == 'person' and inputs.aligned):
            return self._transition('DROP_RESCUE')
        if self.phase == 'DROP_RESCUE' and inputs.drop_channel == 2 and inputs.drop_succeeded:
            return self._transition('RETURN_HOME')
        if self.phase == 'RETURN_HOME' and inputs.home_reached:
            return self._transition('LAND')
        if self.phase == 'LAND' and inputs.landed:
            return self._transition('DISARM')
        if self.phase == 'DISARM' and inputs.disarmed:
            return self._transition('COMPLETE')
        self.reason = ''
        return self._command()
