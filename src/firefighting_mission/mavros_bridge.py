from __future__ import division, print_function

import math


def flu_to_enu(forward, left, yaw):
    east = math.cos(yaw) * forward - math.sin(yaw) * left
    north = math.sin(yaw) * forward + math.cos(yaw) * left
    return east, north


def command_actions(command):
    command = command.strip().upper()
    if command == 'ARM':
        return ('OFFBOARD', 'ARM')
    if command in ('OFFBOARD', 'AUTO.LAND', 'DISARM'):
        return (command,)
    return ()
