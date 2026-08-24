from __future__ import print_function

from collections import namedtuple


DropDecision = namedtuple('DropDecision', 'success channel reason')


class SupplyDropController(object):
    def __init__(self, maximum_speed=0.10, minimum_altitude=1.15,
                 maximum_altitude=1.45):
        self.maximum_speed = float(maximum_speed)
        self.minimum_altitude = float(minimum_altitude)
        self.maximum_altitude = float(maximum_altitude)
        self.released = set()

    def request(self, channel, aligned, horizontal_speed, altitude, release):
        channel = int(channel)
        if channel not in (1, 2):
            return DropDecision(False, channel, 'invalid_channel')
        if channel in self.released:
            return DropDecision(False, channel, 'already_released')
        if not aligned:
            return DropDecision(False, channel, 'not_aligned')
        if float(horizontal_speed) > self.maximum_speed:
            return DropDecision(False, channel, 'moving_too_fast')
        if not self.minimum_altitude <= float(altitude) <= self.maximum_altitude:
            return DropDecision(False, channel, 'altitude_out_of_range')

        success, reason = release(channel)
        if not success:
            return DropDecision(False, channel, reason or 'plugin_failed')
        self.released.add(channel)
        return DropDecision(True, channel, '')
