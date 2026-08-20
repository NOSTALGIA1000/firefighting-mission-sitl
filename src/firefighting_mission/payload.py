from __future__ import print_function

from collections import namedtuple


PayloadDecision = namedtuple('PayloadDecision', 'accepted channel reason')


class PayloadController(object):
    def __init__(self, maximum_speed=0.10, minimum_altitude=1.15,
                 maximum_altitude=1.45):
        self.maximum_speed = float(maximum_speed)
        self.minimum_altitude = float(minimum_altitude)
        self.maximum_altitude = float(maximum_altitude)
        self.released = set()

    def request(self, channel, aligned, horizontal_speed, altitude):
        channel = int(channel)
        if channel not in (1, 2):
            return PayloadDecision(False, channel, 'invalid_channel')
        if channel in self.released:
            return PayloadDecision(False, channel, 'already_released')
        if not aligned:
            return PayloadDecision(False, channel, 'not_aligned')
        if float(horizontal_speed) > self.maximum_speed:
            return PayloadDecision(False, channel, 'moving_too_fast')
        if not self.minimum_altitude <= float(altitude) <= self.maximum_altitude:
            return PayloadDecision(False, channel, 'altitude_out_of_range')
        self.released.add(channel)
        return PayloadDecision(True, channel, '')
