from __future__ import division, print_function

import math


# These are the values used by XTDrone's multirotor_communication.py for
# cmd_vel_flu.  Keep them here (rather than depending on that upstream script)
# so the mission package owns the small ROS/MAVROS adaptation it launches.
FRAME_BODY_NED = 8
VELOCITY_ONLY_TYPE_MASK = 1479
MINIMUM_PRESTREAM_SETPOINTS = 40
RAW_SETPOINT_RATE_HZ = 30.0


def flu_to_enu(forward, left, yaw):
    east = math.cos(yaw) * forward - math.sin(yaw) * left
    north = math.sin(yaw) * forward + math.cos(yaw) * left
    return east, north


def flu_to_mavros_velocity(forward, left, upward, yaw):
    """Map XTDrone FLU velocity into the MAVROS velocity setpoint topic."""
    east, north = flu_to_enu(forward, left, yaw)
    return east, north, upward


def raw_velocity_setpoint(forward, left, upward, yaw_rate):
    """Return the XTDrone cmd_vel_flu PositionTarget velocity contract.

    MAVROS converts FRAME_BODY_NED to the FCU frame.  In particular, these
    values must not be rotated into ENU before publishing to setpoint_raw.
    """
    return (FRAME_BODY_NED, VELOCITY_ONLY_TYPE_MASK,
            forward, left, upward, yaw_rate)


def arm_actions(setpoint_count, offboard_sent):
    """Require a raw-setpoint prestream before requesting OFFBOARD and arm."""
    if setpoint_count < MINIMUM_PRESTREAM_SETPOINTS:
        return ()
    if not offboard_sent:
        return ('OFFBOARD',)
    return ('ARM',)


def command_actions(command):
    command = command.strip().upper()
    if command == 'ARM':
        return ('OFFBOARD', 'ARM')
    if command in ('OFFBOARD', 'AUTO.LAND', 'DISARM'):
        return (command,)
    return ()
