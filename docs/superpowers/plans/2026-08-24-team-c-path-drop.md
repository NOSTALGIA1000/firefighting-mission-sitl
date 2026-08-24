# Team C Path Planning and Supply Drop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add staged climb-cruise-descend point navigation and safety-gated dual-channel Gazebo payload release through ROS services.

**Architecture:** Team C planner publishes internal position targets while Team A remains sole MAVROS setpoint owner. High-level supply service validates flight state, then calls low-level Gazebo plugin service that detaches selected payload joint; existing drop topics remain compatible.

**Tech Stack:** Python 2.7-compatible ROS Melodic, `rospy`, `geometry_msgs`, custom ROS service generation, Gazebo 9 C++14 model plugin, PX4 SITL, `unittest`, `rostest`.

## Global Constraints

- Safe cruise altitude is exactly `2.30 m`.
- Horizontal reach tolerance is `0.12 m`; vertical reach tolerance is `0.08 m`.
- Payload channel `1` is firefighting material; channel `2` is rescue material.
- High-level release requires alignment, horizontal speed at most `0.10 m/s`, and altitude `1.15–1.45 m`.
- `competition_main.py` remains sole publisher to `/mavros/setpoint_position/local`.
- Existing `/fire_iris/drop_fire` and `/fire_iris/drop_rescue` Bool topics remain supported.
- All Python code remains Python 2.7 compatible.

---

### Task 1: Add dual-channel ROS service contract

**Files:**
- Create: `srv/DropSupply.srv`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Modify: `test/test_package_metadata.py`

**Interfaces:**
- Produces: `firefighting_mission/DropSupply` with request `uint8 channel` and response `bool success`, `string reason`.

- [ ] **Step 1: Write failing metadata test**

Add to `test/test_package_metadata.py`:

```python
    def test_drop_supply_service_is_generated(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        service = os.path.join(PROJECT_ROOT, 'srv', 'DropSupply.srv')

        self.assertTrue(os.path.isfile(service))
        self.assertIn('add_service_files(FILES\n  DropSupply.srv', cmake)
```

- [ ] **Step 2: Run test and verify RED**

Run:

```bash
python -m unittest test.test_package_metadata.PackageMetadataTest.test_drop_supply_service_is_generated -v
```

Expected: FAIL because `srv/DropSupply.srv` does not exist.

- [ ] **Step 3: Add service and build metadata**

Create `srv/DropSupply.srv`:

```text
uint8 FIRE=1
uint8 RESCUE=2
uint8 channel
---
bool success
string reason
```

Add to `CMakeLists.txt` before `generate_messages`:

```cmake
add_service_files(FILES
  DropSupply.srv
)
```

Keep `message_generation` as build dependency and `message_runtime` as runtime dependency in `package.xml`.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
python -m unittest test.test_package_metadata -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add srv/DropSupply.srv CMakeLists.txt package.xml test/test_package_metadata.py
git commit -m "feat: add dual-channel drop service contract"
```

---

### Task 2: Implement staged path-planning core

**Files:**
- Create: `src/firefighting_mission/path_planner.py`
- Create: `test/test_path_planner.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `PathPlannerConfig(safe_altitude=2.30, reached_xy=0.12, reached_z=0.08)`.
- Produces: `StagedPathPlanner.set_goal(goal, pose)` where tuples are `(x, y, z)`.
- Produces: `StagedPathPlanner.update(pose)` returning `PlanCommand(stage, target)`.

- [ ] **Step 1: Write failing planner tests**

Create `test/test_path_planner.py`:

```python
from __future__ import print_function

import os
import sys
import unittest

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.path_planner import StagedPathPlanner


class StagedPathPlannerTest(unittest.TestCase):
    def test_climbs_without_horizontal_motion(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        command = planner.update((0.0, 0.0, 1.2))
        self.assertEqual('CLIMB', command.stage)
        self.assertEqual((0.0, 0.0, 2.3), command.target)

    def test_cruises_at_safe_altitude_after_climb(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        command = planner.update((0.01, -0.01, 2.24))
        self.assertEqual('CRUISE', command.stage)
        self.assertEqual((2.0, -1.0, 2.3), command.target)

    def test_descends_only_after_horizontal_arrival(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        planner.update((0.0, 0.0, 2.3))
        command = planner.update((1.91, -0.94, 2.3))
        self.assertEqual('DESCEND', command.stage)
        self.assertEqual((2.0, -1.0, 1.2), command.target)

    def test_reaches_and_holds_final_goal(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, -1.0, 1.2), (0.0, 0.0, 1.2))
        planner.update((0.0, 0.0, 2.3))
        planner.update((2.0, -1.0, 2.3))
        command = planner.update((2.02, -1.01, 1.24))
        self.assertEqual('REACHED', command.stage)
        self.assertEqual((2.0, -1.0, 1.2), command.target)

    def test_new_goal_restarts_from_current_xy(self):
        planner = StagedPathPlanner()
        planner.set_goal((2.0, 0.0, 1.2), (0.0, 0.0, 1.2))
        planner.set_goal((1.0, -2.0, 1.2), (0.5, -0.4, 1.3))
        command = planner.update((0.5, -0.4, 1.3))
        self.assertEqual((0.5, -0.4, 2.3), command.target)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
python -m unittest test.test_path_planner -v
```

Expected: ERROR with missing `firefighting_mission.path_planner`.

- [ ] **Step 3: Implement minimal planner**

Create `src/firefighting_mission/path_planner.py`:

```python
from __future__ import division, print_function

import math
from collections import namedtuple

PlanCommand = namedtuple('PlanCommand', 'stage target')


class PathPlannerConfig(object):
    def __init__(self, safe_altitude=2.30, reached_xy=0.12, reached_z=0.08):
        self.safe_altitude = float(safe_altitude)
        self.reached_xy = float(reached_xy)
        self.reached_z = float(reached_z)


class StagedPathPlanner(object):
    def __init__(self, config=None):
        self.config = config or PathPlannerConfig()
        self.goal = None
        self.start_xy = None
        self.stage = 'IDLE'

    def set_goal(self, goal, pose):
        self.goal = tuple(float(value) for value in goal)
        self.start_xy = (float(pose[0]), float(pose[1]))
        self.stage = 'CLIMB'

    def update(self, pose):
        if self.goal is None:
            return PlanCommand('IDLE', None)
        px, py, pz = (float(value) for value in pose)
        gx, gy, gz = self.goal
        safe = self.config.safe_altitude
        if self.stage == 'CLIMB':
            if abs(pz - safe) > self.config.reached_z:
                return PlanCommand('CLIMB', self.start_xy + (safe,))
            self.stage = 'CRUISE'
        if self.stage == 'CRUISE':
            if math.hypot(gx - px, gy - py) > self.config.reached_xy:
                return PlanCommand('CRUISE', (gx, gy, safe))
            self.stage = 'DESCEND'
        if self.stage == 'DESCEND':
            if abs(gz - pz) > self.config.reached_z:
                return PlanCommand('DESCEND', self.goal)
            self.stage = 'REACHED'
        return PlanCommand('REACHED', self.goal)
```

- [ ] **Step 4: Register test and verify GREEN**

Add to `CMakeLists.txt` testing block:

```cmake
catkin_add_nosetests(test/test_path_planner.py)
```

Run:

```bash
python -m unittest test.test_path_planner test.test_navigation -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/firefighting_mission/path_planner.py test/test_path_planner.py CMakeLists.txt
git commit -m "feat: add staged fixed-height path planner"
```

---

### Task 3: Add ROS planner adapter and Team A setpoint handoff

**Files:**
- Create: `scripts/path_planner.py`
- Create: `test/path_planner_ros_test.py`
- Create: `test/path_planner.test`
- Modify: `scripts/competition_main.py`
- Modify: `launch/competition_takeoff.launch`
- Modify: `CMakeLists.txt`
- Modify: `.gitattributes`

**Interfaces:**
- Consumes: `/fire_mission/point_goal`, MAVROS local pose.
- Produces: `/fire_mission/path_setpoint`, `/fire_mission/path_status`.
- Team A consumes `/fire_mission/path_setpoint` only after initial `HOVER`.

- [ ] **Step 1: Write failing ROS contract and metadata tests**

Create `test/path_planner_ros_test.py` that publishes pose and final goal, then asserts ordered staged targets and statuses. Use four controlled pose updates: takeoff pose, safe-altitude pose, destination-XY pose, final pose.

Add to `test/test_package_metadata.py`:

```python
    def test_path_planner_node_is_installed(self):
        with open(os.path.join(PROJECT_ROOT, 'CMakeLists.txt'), 'r') as handle:
            cmake = handle.read()
        launch = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                       'competition_takeoff.launch')).getroot()
        self.assertIn('scripts/path_planner.py', cmake)
        args = [node.attrib.get('name') for node in launch.findall('arg')]
        self.assertIn('enable_path_planner', args)
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
python -m unittest test.test_package_metadata -v
```

Expected: FAIL because planner script and launch integration are missing.

- [ ] **Step 3: Implement ROS node**

Create executable `scripts/path_planner.py`:

```python
#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from firefighting_mission.path_planner import StagedPathPlanner


class PathPlannerNode(object):
    def __init__(self):
        self.planner = StagedPathPlanner()
        self.pose = None
        self.requested_goal = None
        self.active_goal = None
        self.target_pub = rospy.Publisher('/fire_mission/path_setpoint', PoseStamped,
                                          queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/fire_mission/path_status', String,
                                          queue_size=1, latch=True)
        prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        rospy.Subscriber(prefix + '/local_position/pose', PoseStamped, self._pose)
        rospy.Subscriber('/fire_mission/point_goal', PoseStamped, self._goal)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    @staticmethod
    def _point(message):
        point = message.pose.position
        return (point.x, point.y, point.z)

    def _pose(self, message):
        self.pose = message

    def _goal(self, message):
        self.requested_goal = self._point(message)

    def _tick(self, _event):
        if self.pose is None or self.requested_goal is None:
            self.status_pub.publish('IDLE')
            return
        pose = self._point(self.pose)
        if self.requested_goal != self.active_goal:
            self.planner.set_goal(self.requested_goal, pose)
            self.active_goal = self.requested_goal
        command = self.planner.update(pose)
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.pose.position.x = command.target[0]
        target.pose.position.y = command.target[1]
        target.pose.position.z = command.target[2]
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)
        self.status_pub.publish(command.stage)


if __name__ == '__main__':
    rospy.init_node('firefighting_path_planner')
    PathPlannerNode()
    rospy.spin()
```

- [ ] **Step 4: Add sole-publisher handoff**

Modify `scripts/competition_main.py`:

```python
from firefighting_mission.competition_main import CompetitionMain, PositionSetpoint

# In __init__:
self.path_setpoint = None
self.path_control_enabled = False
rospy.Subscriber('/fire_mission/path_setpoint', PoseStamped,
                 self._path_setpoint)

def _path_setpoint(self, message):
    point = message.pose.position
    self.path_setpoint = PositionSetpoint(point.x, point.y, point.z)

# In _tick, immediately after controller.tick(...):
if outputs.state == 'HOVER' and self.path_setpoint is not None:
    self.path_control_enabled = True
setpoints = outputs.setpoints
if self.path_control_enabled and self.path_setpoint is not None:
    setpoints = [self.path_setpoint]

# Replace the existing loop source:
for point in setpoints:
    self._publish_setpoint(point)
```

Add to `competition_takeoff.launch`:

```xml
<arg name="enable_path_planner" default="false"/>
<node pkg="firefighting_mission" type="path_planner.py" name="path_planner"
      if="$(arg enable_path_planner)" output="screen">
  <param name="mavros_prefix" value="$(arg mavros_prefix)"/>
</node>
```

- [ ] **Step 5: Register scripts/tests and verify GREEN**

Add `scripts/path_planner.py` to `catkin_install_python`, add `test/path_planner.test` to `add_rostest`, and keep LF export through `.gitattributes` existing `scripts/* text eol=lf` rule.

Run:

```bash
python -m unittest test.test_path_planner test.test_competition_main test.test_package_metadata -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add scripts/path_planner.py scripts/competition_main.py launch/competition_takeoff.launch test/path_planner_ros_test.py test/path_planner.test test/test_competition_main.py test/test_package_metadata.py CMakeLists.txt .gitattributes
git commit -m "feat: connect staged planner to offboard controller"
```

---

### Task 4: Implement safety-gated high-level supply service

**Files:**
- Create: `src/firefighting_mission/supply_drop.py`
- Create: `scripts/supply_drop.py`
- Create: `test/test_supply_drop.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `SupplyDropController.request(channel, aligned, horizontal_speed, altitude, release)`.
- Provides: `/fire_mission/drop_supply` and calls `/fire_iris/drop_supply`.

- [ ] **Step 1: Write failing policy tests**

Create `test/test_supply_drop.py` covering valid channels, invalid channel, duplicate release, alignment, speed, altitude, low-level rejection, and retry after low-level rejection. Use a callable release stub returning `(success, reason)`.

Critical test:

```python
    def test_low_level_failure_does_not_consume_channel(self):
        controller = SupplyDropController()
        failed = controller.request(1, True, 0.0, 1.25,
                                    lambda _channel: (False, 'plugin_failed'))
        retried = controller.request(1, True, 0.0, 1.25,
                                     lambda _channel: (True, ''))
        self.assertFalse(failed.success)
        self.assertTrue(retried.success)
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
python -m unittest test.test_supply_drop -v
```

Expected: ERROR with missing `firefighting_mission.supply_drop`.

- [ ] **Step 3: Implement policy and ROS node**

Create `src/firefighting_mission/supply_drop.py`:

```python
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
```

Create executable `scripts/supply_drop.py`:

```python
#!/usr/bin/env python
from __future__ import division, print_function

import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from firefighting_mission.srv import (DropSupply, DropSupplyResponse)
from firefighting_mission.supply_drop import SupplyDropController


class SupplyDropNode(object):
    def __init__(self):
        self.controller = SupplyDropController()
        self.pose = None
        self.velocity = None
        self.aligned = False
        prefix = rospy.get_param('~mavros_prefix', '/mavros').rstrip('/')
        self.low_level = rospy.ServiceProxy('/fire_iris/drop_supply', DropSupply)
        self.service = rospy.Service('/fire_mission/drop_supply', DropSupply,
                                     self._drop)
        rospy.Subscriber(prefix + '/local_position/pose', PoseStamped, self._pose)
        rospy.Subscriber(prefix + '/local_position/velocity_local', TwistStamped,
                         self._velocity)
        rospy.Subscriber('/fire_mission/aligned', Bool, self._aligned)

    def _pose(self, message):
        self.pose = message

    def _velocity(self, message):
        self.velocity = message

    def _aligned(self, message):
        self.aligned = bool(message.data)

    def _release(self, channel):
        try:
            response = self.low_level(channel)
            return bool(response.success), response.reason
        except rospy.ServiceException as error:
            return False, 'plugin_service_failed:%s' % error

    def _drop(self, request):
        if self.pose is None or self.velocity is None:
            return DropSupplyResponse(False, 'flight_state_missing')
        linear = self.velocity.twist.linear
        speed = math.hypot(linear.x, linear.y)
        decision = self.controller.request(
            request.channel, self.aligned, speed,
            self.pose.pose.position.z, self._release)
        return DropSupplyResponse(decision.success, decision.reason)


if __name__ == '__main__':
    rospy.init_node('firefighting_supply_drop')
    SupplyDropNode()
    rospy.spin()
```

- [ ] **Step 4: Register and verify GREEN**

Add to `CMakeLists.txt`:

```cmake
catkin_add_nosetests(test/test_supply_drop.py)

# Inside catkin_install_python(PROGRAMS ...):
scripts/supply_drop.py
```

Run:

```bash
python -m unittest test.test_supply_drop test.test_payload -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/firefighting_mission/supply_drop.py scripts/supply_drop.py test/test_supply_drop.py CMakeLists.txt
git commit -m "feat: add safety-gated supply drop service"
```

---

### Task 5: Expose Gazebo joint-detach service

**Files:**
- Modify: `include/firefighting_mission/payload_plugin.hpp`
- Modify: `src/payload_plugin.cpp`
- Modify: `test/payload_drop_ros_test.py`
- Modify: `test/payload_drop.test`

**Interfaces:**
- Provides: `/fire_iris/drop_supply` using `DropSupply`.
- Preserves: `/fire_iris/drop_fire`, `/fire_iris/drop_rescue` Bool topics.

- [ ] **Step 1: Convert Gazebo test to service-first RED test**

Update `test/payload_drop_ros_test.py` service test body:

```python
rospy.wait_for_service('/fire_iris/drop_supply', timeout=10.0)
drop = rospy.ServiceProxy('/fire_iris/drop_supply', DropSupply)
fire_before = self._z('payload_test::fire_payload_link')
rescue_before = self._z('payload_test::rescue_payload_link')

released = drop(1)
self.assertTrue(released.success)
deadline = rospy.Time.now() + rospy.Duration(10.0)
while self._z('payload_test::fire_payload_link') > fire_before - 0.20:
    if rospy.Time.now() > deadline:
        self.fail('fire payload did not fall after service release')
    rospy.sleep(0.05)
self.assertAlmostEqual(
    rescue_before, self._z('payload_test::rescue_payload_link'), delta=0.03)

duplicate = drop(1)
self.assertFalse(duplicate.success)
self.assertEqual('already_released', duplicate.reason)

self.rescue_topic.publish(True)
deadline = rospy.Time.now() + rospy.Duration(10.0)
while self._z('payload_test::rescue_payload_link') > rescue_before - 0.20:
    if rospy.Time.now() > deadline:
        self.fail('legacy rescue topic did not release payload')
    rospy.sleep(0.05)
```

- [ ] **Step 2: Build/run and verify RED**

Run in VM:

```bash
cd /home/ss/catkin_ws
catkin_make
source devel/setup.bash
rostest firefighting_mission payload_drop.test
```

Expected: FAIL because `/fire_iris/drop_supply` is unavailable.

- [ ] **Step 3: Implement plugin service**

Implement shared release and service callback in `src/payload_plugin.cpp`:

```cpp
bool PayloadPlugin::Release(unsigned channel, std::string* reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (channel != 1 && channel != 2) {
    *reason = "invalid_channel";
    return false;
  }
  Slot& slot = channel == 1 ? fire_ : rescue_;
  if (slot.released) {
    *reason = "already_released";
    return false;
  }
  if (!slot.payload_joint || !slot.door_joint || !slot.payload_link) {
    *reason = "model_not_ready";
    return false;
  }
  slot.door_joint->SetPosition(0, channel == 1 ? 1.15 : -1.15);
  slot.payload_joint->Detach();
  slot.payload_link->SetGravityMode(true);
  slot.released = true;
  reason->clear();
  return true;
}

bool PayloadPlugin::DropService(
    firefighting_mission::DropSupply::Request& request,
    firefighting_mission::DropSupply::Response& response) {
  response.success = Release(request.channel, &response.reason);
  return true;
}
```

In `Load`, advertise after model validation:

```cpp
drop_service_ = node_->advertiseService("drop_supply",
                                        &PayloadPlugin::DropService, this);
```

Topic callbacks call `Release(channel, &reason)` with a local string. Update header signature to `bool Release(unsigned channel, std::string* reason);` and retain the service declaration shown below.

Header adds:

```cpp
#include <firefighting_mission/DropSupply.h>

bool DropService(firefighting_mission::DropSupply::Request& request,
                 firefighting_mission::DropSupply::Response& response);
ros::ServiceServer drop_service_;
```

- [ ] **Step 4: Verify GREEN**

Run:

```bash
cd /home/ss/catkin_ws
catkin_make
source devel/setup.bash
rostest firefighting_mission payload_drop.test
```

Expected: PASS with service release, duplicate rejection, and legacy topic release.

- [ ] **Step 5: Commit**

```bash
git add include/firefighting_mission/payload_plugin.hpp src/payload_plugin.cpp test/payload_drop_ros_test.py test/payload_drop.test
git commit -m "feat: expose gazebo payload detach service"
```

---

### Task 6: Document, integrate, and verify Team C in VM

**Files:**
- Create: `docs/TEAM_C_HANDOFF.zh-CN.md`
- Modify: `README.zh-CN.md`
- Modify: `docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md`

**Interfaces:**
- Documents exact topics, services, channel mapping, launch commands, and acceptance evidence.

- [ ] **Step 1: Add handoff documentation**

Document:

```text
/fire_mission/point_goal      final point request
/fire_mission/path_setpoint   current staged target
/fire_mission/path_status     CLIMB/CRUISE/DESCEND/REACHED
/fire_mission/drop_supply     safety-gated high-level service
/fire_iris/drop_supply        Gazebo low-level detach service
channel 1                     firefighting material
channel 2                     rescue material
```

Include example `rostopic pub` goal and `rosservice call` release commands.

- [ ] **Step 2: Run fresh full unit suite**

Run in VM:

```bash
cd /home/ss/catkin_ws/src/firefighting_mission
python -m unittest discover -s test -p 'test_*.py'
```

Expected: all tests pass, zero failures/errors.

- [ ] **Step 3: Run fresh ROS build and contracts**

Run:

```bash
cd /home/ss/catkin_ws
catkin_make
source devel/setup.bash
rostest firefighting_mission path_planner.test
rostest firefighting_mission payload_drop.test
```

Expected: build and both rostests pass.

- [ ] **Step 4: Run actual path flight**

Launch competition world with planner enabled, wait for initial `HOVER`, then publish a target on the clear side of the field:

```bash
roslaunch firefighting_mission competition_takeoff.launch gui:=true enable_path_planner:=true
rostopic pub -1 /fire_mission/point_goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.70, y: -1.90, z: 1.20}, orientation: {w: 1.0}}}"
```

Acceptance evidence: statuses occur in order `CLIMB`, `CRUISE`, `DESCEND`, `REACHED`; altitude exceeds `2.22 m` before horizontal travel; final pose is within configured tolerances.

- [ ] **Step 5: Run actual service drop fixture**

Run:

```bash
rostest firefighting_mission payload_drop.test
```

Acceptance evidence: requested cube falls, non-requested cube remains attached until its trigger, duplicate call is rejected.

- [ ] **Step 6: Commit documentation**

```bash
git add docs/TEAM_C_HANDOFF.zh-CN.md README.zh-CN.md docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md
git commit -m "docs: add team c handoff and verification"
```

- [ ] **Step 7: Push verified branch**

```bash
git status --short --branch
git push origin feature/firefighting-sitl
```

Expected: clean worktree and remote branch updated to local HEAD.
