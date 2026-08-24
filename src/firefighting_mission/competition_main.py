from __future__ import division, print_function

from collections import namedtuple


PositionSetpoint = namedtuple('PositionSetpoint', 'x y z')
ModeRequest = namedtuple('ModeRequest', 'mode')
ControllerOutputs = namedtuple(
    'ControllerOutputs', 'state setpoints mode_requests arm_request')


def mission_interface_topics():
    return {
        'hazard_detected': '/hazard_detected',
        'person_detected': '/person_detected',
        'drop_fire_payload': '/drop_fire_payload',
        'drop_rescue_payload': '/drop_rescue_payload',
    }


def select_active_setpoints(outputs, planned_setpoint,
                            path_control_enabled=False):
    if path_control_enabled and planned_setpoint is not None:
        return [planned_setpoint]
    return outputs.setpoints


class CompetitionMain(object):
    """Minimal MAVROS takeoff/hover controller for the team A task."""

    def __init__(self, takeoff_altitude=1.2, prestream_count=40,
                 hover_tolerance=0.08, hover_hold_seconds=2.0,
                 mode_retry_seconds=1.0):
        self.takeoff_altitude = float(takeoff_altitude)
        self.prestream_count = int(prestream_count)
        self.hover_tolerance = float(hover_tolerance)
        self.hover_hold_seconds = float(hover_hold_seconds)
        self.mode_retry_seconds = float(mode_retry_seconds)
        self.state = 'INIT'
        self._setpoint_count = 0
        self._last_mode_request_time = None
        self._hover_since = None

    def _takeoff_setpoint(self):
        return PositionSetpoint(0.0, 0.0, self.takeoff_altitude)

    def _outputs(self, mode_requests=None, arm_request=False):
        return ControllerOutputs(
            self.state,
            [self._takeoff_setpoint()],
            mode_requests or [],
            bool(arm_request),
        )

    def tick(self, now, connected, armed, mode, altitude,
             sensor_ready=True, local_pose_available=True):
        if not connected:
            self.state = 'WAIT_FCU'
            return ControllerOutputs(self.state, [], [], False)

        if not sensor_ready:
            self.state = 'WAIT_SENSOR'
            return ControllerOutputs(self.state, [], [], False)

        if mode != 'OFFBOARD':
            self.state = 'PRESTREAM_SETPOINTS'
            self._setpoint_count += 1
            can_request = (
                self._last_mode_request_time is None or
                float(now) - self._last_mode_request_time >= self.mode_retry_seconds)
            if self._setpoint_count >= self.prestream_count and can_request:
                self._last_mode_request_time = float(now)
                return self._outputs([ModeRequest('OFFBOARD')], False)
            return self._outputs()

        if not armed:
            self.state = 'ARM'
            return self._outputs([], True)

        if abs(float(altitude) - self.takeoff_altitude) <= self.hover_tolerance:
            if self._hover_since is None:
                self._hover_since = float(now)
            if float(now) - self._hover_since >= self.hover_hold_seconds:
                self.state = 'HOVER'
                return self._outputs()
        else:
            self._hover_since = None

        self.state = 'TAKEOFF'
        return self._outputs()
