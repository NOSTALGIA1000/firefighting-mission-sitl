# PX4 Preflight Health Gate Design

## Context

The 50 Hz Gazebo external-vision experiment produced one clean run and two unsafe
runs. One failed run reported `Preflight Fail: Accel Range, hold still on arming`
and `Primary accelerometer not found`, but the current ROS node still progressed
from receiving one IMU message to OFFBOARD and arming. A received IMU message is
therefore not a sufficient preflight condition.

The VM uses ROS Melodic MAVROS. This version does not provide
`mavros_msgs/SysStatus`; available health inputs are:

- `/mavros/state` (`mavros_msgs/State.system_status`)
- `/mavros/estimator_status` (`mavros_msgs/EstimatorStatus`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)

## Goal

Prevent OFFBOARD prestreaming and arming until PX4 and its estimator have remained
healthy for a continuous stability window. Never force takeoff when health data is
missing, stale, invalid, or unstable.

## Health Conditions

Preflight is healthy only when all conditions hold:

1. MAVROS is connected and PX4 reports `MAV_STATE_STANDBY` while disarmed.
2. A recent estimator-status message exists.
3. `attitude_status_flag` is true.
4. `accel_error_status_flag` is false.
5. A recent IMU message exists and all quaternion, angular-velocity, and
   linear-acceleration values are finite.
6. Acceleration magnitude is within a configurable stationary range suitable for
   gravity at rest.
7. Conditions remain true continuously for a configurable stability window,
   default 3.0 seconds.

Any failed condition resets the stability timer. Missing estimator-status data is
unhealthy (fail closed).

## State-Machine Behavior

- Before arming, failed health keeps `competition_main` in `WAIT_SENSOR`.
- `WAIT_SENSOR` publishes no position setpoint, OFFBOARD request, or arm request.
- Loss of health before arming resets the OFFBOARD prestream counter, so stale
  samples cannot satisfy the prestream requirement.
- After arming, the preflight gate no longer suppresses flight setpoints. Existing
  runtime pose/geofence safety logic remains responsible for hover, retreat, and
  landing. This avoids cutting OFFBOARD control because of one transient health
  message during flight.

## Code Shape

- Add a pure, ROS-independent `PreflightHealthGate` in
  `src/firefighting_mission/competition_main.py`.
- Subscribe to `mavros_msgs/EstimatorStatus` in
  `scripts/competition_main.py` and feed current PX4 state, estimator status, and
  IMU sample into the gate.
- Expose launch parameters for stability duration, message freshness, and
  stationary acceleration bounds.
- Publish/log the current rejection reason at a throttled rate for diagnosis.

## Tests

Unit tests cover:

- healthy inputs must remain continuous for the full stability window;
- missing/stale IMU and estimator data fail closed;
- attitude invalid and accelerometer error each block readiness;
- NaN/Inf IMU values and out-of-range acceleration block readiness;
- one unhealthy sample resets the stability window;
- health loss before arming resets OFFBOARD prestream count;
- armed flight continues publishing setpoints despite later preflight-health loss;
- launch files expose and pass all health-gate parameters.

VM validation sequence:

1. Run local unit suite.
2. Sync to VM and run VM unit suite plus `catkin_make`.
3. Run three identical SITL trials.
4. Confirm no arm request occurs during PX4 calibration/preflight failure.
5. Accept only if all three trials avoid collision and reach the planned target
   while holding approximately 1.2 m altitude.

## Non-goals

- Replacing PX4 EKF or MAVROS.
- Adding in-flight IMU failover.
- Treating Gazebo ground truth as an onboard sensor for the final real aircraft.
- Changing visual obstacle-avoidance behavior in this patch.
