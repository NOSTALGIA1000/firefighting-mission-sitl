from __future__ import division, print_function

import math
from collections import namedtuple


class PositionSetpoint(namedtuple('_PositionSetpoint', 'x y z yaw')):
    __slots__ = ()

    def __new__(cls, x, y, z, yaw=0.0):
        return super(PositionSetpoint, cls).__new__(
            cls, float(x), float(y), float(z), float(yaw))
ModeRequest = namedtuple('ModeRequest', 'mode')
ControllerOutputs = namedtuple(
    'ControllerOutputs', 'state setpoints mode_requests arm_request')
PreflightSample = namedtuple(
    'PreflightSample',
    'connected armed system_status estimator_received_at '
    'estimator_attitude_valid estimator_accel_error imu_received_at '
    'imu_orientation imu_angular_velocity imu_linear_acceleration')


class PreflightHealthGate(object):
    MAV_STATE_STANDBY = 3

    def __init__(self, stable_seconds=3.0, max_message_age=1.5,
                 accel_min=5.0, accel_max=20.0):
        self.stable_seconds = float(stable_seconds)
        self.max_message_age = float(max_message_age)
        self.accel_min = float(accel_min)
        self.accel_max = float(accel_max)
        self._healthy_since = None
        self.reason = 'not_checked'

    @staticmethod
    def _values_are_finite(values):
        return all(not math.isnan(value) and not math.isinf(value)
                   for value in values)

    def _rejection_reason(self, now, sample):
        if not sample.connected:
            return 'disconnected'
        if (not sample.armed and
                sample.system_status != self.MAV_STATE_STANDBY):
            return 'px4_not_standby'
        if (sample.estimator_received_at is None or
                now - sample.estimator_received_at > self.max_message_age):
            return 'estimator_stale'
        if not sample.estimator_attitude_valid:
            return 'attitude_invalid'
        if sample.estimator_accel_error:
            return 'accelerometer_error'
        if (sample.imu_received_at is None or
                now - sample.imu_received_at > self.max_message_age):
            return 'imu_stale'
        values = (sample.imu_orientation + sample.imu_angular_velocity +
                  sample.imu_linear_acceleration)
        if not self._values_are_finite(values):
            return 'imu_non_finite'
        acceleration = math.sqrt(sum(
            value * value for value in sample.imu_linear_acceleration))
        if not self.accel_min <= acceleration <= self.accel_max:
            return 'acceleration_out_of_range'
        return None

    def update(self, now, sample):
        now = float(now)
        reason = self._rejection_reason(now, sample)
        if reason is not None:
            self._healthy_since = None
            self.reason = reason
            return False
        if self._healthy_since is None:
            self._healthy_since = now
        if now - self._healthy_since < self.stable_seconds:
            self.reason = 'stabilizing'
            return False
        self.reason = 'ready'
        return True


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

        if not sensor_ready and not armed:
            self._setpoint_count = 0
            self._last_mode_request_time = None
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

        if self.state == 'HOVER':
            return self._outputs()

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
