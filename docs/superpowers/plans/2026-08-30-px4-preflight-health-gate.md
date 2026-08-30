# PX4 Preflight Health Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Block OFFBOARD prestreaming and arming until PX4 estimator and IMU health remain valid for three continuous seconds.

**Architecture:** Add a ROS-independent health sample and stability gate beside the existing pure competition state machine. The ROS adapter converts MAVROS messages into that sample; the existing `sensor_ready` input becomes the preflight gate output. Flight control ignores this preflight-only gate after arming, while pre-arm health loss resets accumulated setpoints.

**Tech Stack:** Python 2.7/3 compatible code, ROS Melodic, MAVROS `State` and `EstimatorStatus`, `sensor_msgs/Imu`, XML launch files, `unittest`.

## Global Constraints

- Fail closed when state, estimator, or IMU health data is missing or stale.
- Require continuous health for 3.0 seconds by default.
- Require stationary acceleration magnitude between 7.0 and 12.0 m/s² by default.
- Publish no setpoint, OFFBOARD request, or arm request while preflight is unhealthy.
- Never suppress armed-flight setpoints because of preflight-health loss.
- Keep ROS-independent logic testable without ROS imports.

---

### Task 1: Pure Preflight Health Gate

**Files:**
- Modify: `src/firefighting_mission/competition_main.py`
- Test: `test/test_competition_main.py`

**Interfaces:**
- Produces: `PreflightSample` named tuple containing connection, PX4 state, estimator flags/timestamps, and IMU vectors.
- Produces: `PreflightHealthGate.update(now, sample) -> bool` and `PreflightHealthGate.reason`.

- [ ] **Step 1: Write failing health-gate tests**

```python
def healthy_sample(now, **changes):
    values = dict(connected=True, armed=False, system_status=3,
                  estimator_received_at=now,
                  estimator_attitude_valid=True,
                  estimator_accel_error=False,
                  imu_received_at=now,
                  imu_orientation=(0.0, 0.0, 0.0, 1.0),
                  imu_angular_velocity=(0.0, 0.0, 0.0),
                  imu_linear_acceleration=(0.0, 0.0, 9.81))
    values.update(changes)
    return PreflightSample(**values)

def test_requires_continuous_health_window(self):
    gate = PreflightHealthGate(stable_seconds=3.0)
    self.assertFalse(gate.update(10.0, healthy_sample(10.0)))
    self.assertFalse(gate.update(12.9, healthy_sample(12.9)))
    self.assertTrue(gate.update(13.0, healthy_sample(13.0)))

def test_unhealthy_sample_resets_health_window(self):
    gate = PreflightHealthGate(stable_seconds=3.0)
    gate.update(10.0, healthy_sample(10.0))
    gate.update(12.0, healthy_sample(
        12.0, estimator_accel_error=True))
    self.assertFalse(gate.update(14.0, healthy_sample(14.0)))
    self.assertTrue(gate.update(17.0, healthy_sample(17.0)))
```

```python
def test_rejects_each_invalid_preflight_input(self):
    cases = (
        ('disconnected', dict(connected=False)),
        ('px4_not_standby', dict(system_status=2)),
        ('estimator_stale', dict(estimator_received_at=9.0)),
        ('attitude_invalid', dict(estimator_attitude_valid=False)),
        ('accelerometer_error', dict(estimator_accel_error=True)),
        ('imu_stale', dict(imu_received_at=9.0)),
        ('imu_non_finite', dict(imu_orientation=(0.0, 0.0, 0.0,
                                                  float('nan')))),
        ('acceleration_out_of_range',
         dict(imu_linear_acceleration=(0.0, 0.0, 1.0))),
    )
    for reason, changes in cases:
        gate = PreflightHealthGate(stable_seconds=0.0,
                                   max_message_age=0.5)
        self.assertFalse(gate.update(10.0,
                                     healthy_sample(10.0, **changes)))
        self.assertEqual(reason, gate.reason)
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest test.test_competition_main
```

Expected: import failure for missing `PreflightHealthGate` or `PreflightSample`.

- [ ] **Step 3: Implement minimal pure gate**

```python
PreflightSample = namedtuple(
    'PreflightSample',
    'connected armed system_status estimator_received_at '
    'estimator_attitude_valid estimator_accel_error imu_received_at '
    'imu_orientation imu_angular_velocity imu_linear_acceleration')

class PreflightHealthGate(object):
    MAV_STATE_STANDBY = 3

    def __init__(self, stable_seconds=3.0, max_message_age=0.5,
                 accel_min=7.0, accel_max=12.0):
        self.stable_seconds = float(stable_seconds)
        self.max_message_age = float(max_message_age)
        self.accel_min = float(accel_min)
        self.accel_max = float(accel_max)
        self._healthy_since = None
        self.reason = 'not_checked'

    def update(self, now, sample):
        reason = self._rejection_reason(float(now), sample)
        if reason is not None:
            self._healthy_since = None
            self.reason = reason
            return False
        if self._healthy_since is None:
            self._healthy_since = float(now)
        if float(now) - self._healthy_since < self.stable_seconds:
            self.reason = 'stabilizing'
            return False
        self.reason = 'ready'
        return True
```

```python
def _rejection_reason(self, now, sample):
    if not sample.connected:
        return 'disconnected'
    if not sample.armed and sample.system_status != self.MAV_STATE_STANDBY:
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
    if not all(not math.isnan(value) and not math.isinf(value)
               for value in values):
        return 'imu_non_finite'
    acceleration = math.sqrt(sum(
        value * value for value in sample.imu_linear_acceleration))
    if not self.accel_min <= acceleration <= self.accel_max:
        return 'acceleration_out_of_range'
    return None
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run same command. Expected: all `test_competition_main` tests pass.

- [ ] **Step 5: Commit pure gate**

```bash
git add src/firefighting_mission/competition_main.py test/test_competition_main.py
git commit -m "feat: add PX4 preflight health gate"
```

### Task 2: Safe State-Machine and MAVROS Adapter Integration

**Files:**
- Modify: `src/firefighting_mission/competition_main.py`
- Modify: `scripts/competition_main.py`
- Test: `test/test_competition_main.py`
- Test: `test/test_package_metadata.py`

**Interfaces:**
- Consumes: `PreflightSample`, `PreflightHealthGate.update`, and `.reason` from Task 1.
- Produces: ROS subscriptions and callback timestamps for `/mavros/estimator_status` and `/mavros/imu/data`.

- [ ] **Step 1: Write failing state-machine and adapter tests**

```python
def test_health_loss_before_arm_resets_prestream_count(self):
    controller = CompetitionMain(prestream_count=2)
    controller.tick(0.0, True, False, '', 0.0, sensor_ready=True)
    controller.tick(0.1, True, False, '', 0.0, sensor_ready=False)
    result = controller.tick(0.2, True, False, '', 0.0,
                             sensor_ready=True)
    self.assertEqual([], result.mode_requests)

def test_armed_flight_keeps_setpoints_after_preflight_health_loss(self):
    controller = CompetitionMain(prestream_count=1)
    result = controller.tick(1.0, True, True, 'OFFBOARD', 1.0,
                             sensor_ready=False)
    self.assertEqual('TAKEOFF', result.state)
    self.assertEqual(1, len(result.setpoints))
```

```python
def test_competition_node_connects_preflight_health_gate(self):
    node = self._read('scripts/competition_main.py')
    self.assertIn('from mavros_msgs.msg import EstimatorStatus, State', node)
    self.assertIn("'/estimator_status'", node)
    self.assertIn('self.estimator_received_at =', node)
    self.assertIn('self.imu_received_at =', node)
    self.assertIn('PreflightHealthGate(', node)
    self.assertIn('PreflightSample(', node)
    self.assertIn('sensor_ready=preflight_ready', node)
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest test.test_competition_main test.test_package_metadata
```

Expected: prestream-reset assertion fails and adapter contract strings are absent.

- [ ] **Step 3: Implement safe integration**

Change state-machine guard to:

```python
if not sensor_ready and not armed:
    self._setpoint_count = 0
    self._last_mode_request_time = None
    self.state = 'WAIT_SENSOR'
    return ControllerOutputs(self.state, [], [], False)
```

ROS node changes:

```python
from mavros_msgs.msg import EstimatorStatus, State

self.preflight_gate = PreflightHealthGate(...)
self.estimator_status = None
self.estimator_received_at = None
self.imu_received_at = None
rospy.Subscriber(self.mavros_prefix + '/estimator_status',
                 EstimatorStatus, self._estimator_status)
```

Callbacks store receive time. `_tick` constructs `PreflightSample`, evaluates gate,
logs `preflight health blocked: <reason>` with throttling, and passes result to
`CompetitionMain.tick`.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run same command. Expected: focused suites pass.

- [ ] **Step 5: Commit integration**

```bash
git add src/firefighting_mission/competition_main.py scripts/competition_main.py test/test_competition_main.py test/test_package_metadata.py
git commit -m "feat: gate OFFBOARD arming on estimator health"
```

### Task 3: Launch Configuration and VM Validation

**Files:**
- Modify: `launch/competition_takeoff.launch`
- Modify: `launch/firefighting.launch`
- Modify: `test/visual_avoidance_smoke.test`
- Test: `test/test_package_metadata.py`
- Modify: `docs/CURRENT_HANDOFF.zh-CN.md`

**Interfaces:**
- Consumes: ROS parameters `health_stable_seconds`, `health_max_message_age`, `health_accel_min`, and `health_accel_max`.
- Produces: launch defaults `3.0`, `0.5`, `7.0`, and `12.0` respectively.

- [ ] **Step 1: Write failing launch-contract test**

```python
expected = {
    'health_stable_seconds': '3.0',
    'health_max_message_age': '0.5',
    'health_accel_min': '7.0',
    'health_accel_max': '12.0',
}
```

```python
for launch_name in ('competition_takeoff.launch', 'firefighting.launch'):
    root = ET.parse(os.path.join(PROJECT_ROOT, 'launch', launch_name)).getroot()
    args = {item.attrib['name']: item.attrib.get('default')
            for item in root.findall('arg')}
    self.assertEqual(expected, {name: args[name] for name in expected})
    node = next(item for item in root.findall('node')
                if item.attrib.get('type') == 'competition_main.py')
    params = {item.attrib['name']: item.attrib.get('value')
              for item in node.findall('param')}
    for name in expected:
        self.assertEqual('$(arg %s)' % name, params[name])
```

- [ ] **Step 2: Run launch-contract test and verify RED**

Run:

```powershell
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest test.test_package_metadata
```

Expected: missing `health_stable_seconds` argument.

- [ ] **Step 3: Add launch parameters and handoff note**

Add these exact arguments and matching node parameters to both launch files:

```xml
<arg name="health_stable_seconds" default="3.0"/>
<arg name="health_max_message_age" default="0.5"/>
<arg name="health_accel_min" default="7.0"/>
<arg name="health_accel_max" default="12.0"/>
```

Add direct values `3.0`, `0.5`, `7.0`, and `12.0` as node parameters in
`test/visual_avoidance_smoke.test`. Add handoff section titled
`2026-08-30 PX4 起飞健康门禁` stating: preflight fails closed, defaults above,
VM build/test evidence, and three consecutive seed-1 trials remain acceptance gate.

- [ ] **Step 4: Run local and VM verification**

Local focused run:

```powershell
& 'C:\Users\ASUS\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest test.test_competition_main test.test_package_metadata
```

VM full run:

```bash
cd ~/catkin_ws/src/firefighting_mission
python -m unittest discover -s test -p 'test_*.py'
cd ~/catkin_ws
catkin_make
```

Expected: all tests pass and build exits 0.

- [ ] **Step 5: Run three seed-1 SITL trials**

Run three times, clearing ROS/Gazebo processes between runs:

```bash
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
```

For each generated smoke result, require `collision=false`, terminal state `REACHED`,
and altitude within the existing 1.1–1.3 m test band. When PX4 reports an unhealthy
startup, verify `/mavros/state.armed` stays false and
`/competition_main/state` stays `WAIT_SENSOR`.

- [ ] **Step 6: Commit verified configuration and handoff**

```bash
git add launch/competition_takeoff.launch launch/firefighting.launch test/visual_avoidance_smoke.test test/test_package_metadata.py docs/CURRENT_HANDOFF.zh-CN.md
git commit -m "test: configure preflight health gate in SITL"
```
