# Stereo Visual Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace 2.30 m overflight with map-guided, stereo-visual random-cylinder avoidance while holding 1.20 m transit altitude.

**Architecture:** Fixed boards and field boundaries feed a small-grid global route planner. A sensor adapter converts metric depth or point clouds into body-frame obstacle clusters; a local state machine selects a safe side, generates short-horizon position/yaw targets, and rejoins the global route. `competition_main.py` remains sole MAVROS setpoint publisher.

**Tech Stack:** ROS Melodic, Python 2.7, rospy, MAVROS/PX4 OFFBOARD, Gazebo 9, OpenCV, NumPy, `sensor_msgs`, `stereo_image_proc`, catkin, unittest/rostest.

## Global Constraints

- Transit altitude is `1.20 m`; allowed horizontal-flight band is `1.10--1.30 m`.
- Fixed boards use measured map coordinates from `world_generator.py`.
- Runtime planning must not read random-cylinder scenario truth.
- Raw corridor acceptance starts at `0.90 m`; competition guarantees at least `1.30 m` on one side.
- Maximum aircraft diameter is `0.40 m`; initial external clearance is `0.25 m` per side.
- Normal route speed is at most `0.45 m/s`; avoidance speed is at most `0.30 m/s`.
- Stale stereo data or insufficient corridor commands hover, never forced passage.
- ROS nodes remain Python 2.7 compatible: no dataclasses, f-strings, or Python-3-only APIs.
- Only `scripts/competition_main.py` publishes MAVROS local setpoints in active competition launch.
- Existing payload, target-recognition, mission-state, manual-takeover, and recording behavior must remain available.

## File Structure

- Create `msg/ObstacleCluster.msg`: one metric body-frame obstacle cluster.
- Create `msg/ObstacleArray.msg`: synchronized set of obstacle clusters.
- Create `msg/AvoidanceStatus.msg`: state, side decision, clearance, reason, target.
- Create `src/firefighting_mission/stereo_obstacles.py`: pure depth/point-cloud filtering and clustering.
- Create `scripts/stereo_obstacle_node.py`: ROS sensor adapter.
- Create `launch/stereo_input.launch`: camera-driver output normalization and optional calibrated raw-stereo processing.
- Create `src/firefighting_mission/field_map.py`: fixed-map inflation and A* route generation.
- Replace `src/firefighting_mission/path_planner.py`: constant-altitude global/local planner.
- Replace `scripts/path_planner.py`: ROS adapter for goal, pose, obstacles, target, and status.
- Modify `src/firefighting_mission/competition_main.py`: yaw-bearing setpoint type.
- Modify `scripts/competition_main.py`: forward planner yaw and accept land/disarm commands.
- Create `models/fire_stereo_camera/model.config` and `model.sdf`: forward metric-depth simulation contract.
- Modify `models/fire_iris/fire_iris.sdf`: mount forward stereo simulation model.
- Create `src/firefighting_mission/avoidance_overlay.py`: front-camera decision overlay.
- Create `scripts/mission_overlay_node.py`: picture-in-picture mission recording view.
- Modify launch, recorder, safety, package, and focused tests listed below.

---

### Task 1: Typed Stereo-Avoidance ROS Contracts

**Files:**
- Create: `msg/ObstacleCluster.msg`
- Create: `msg/ObstacleArray.msg`
- Create: `msg/AvoidanceStatus.msg`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Modify: `test/test_package_metadata.py`

**Interfaces:**
- Produces: `firefighting_mission/ObstacleArray` on `/fire_mission/obstacles`.
- Produces: `firefighting_mission/AvoidanceStatus` on `/fire_mission/avoidance_status`.
- `ObstacleCluster` coordinates use aircraft `base_link`: `forward_m` positive forward, `left_m` positive left.

- [ ] **Step 1: Write failing metadata tests**

```python
def test_stereo_avoidance_messages_are_generated(self):
    cmake = self._read('CMakeLists.txt')
    package = self._read('package.xml')
    for name in ('ObstacleCluster.msg', 'ObstacleArray.msg',
                 'AvoidanceStatus.msg'):
        self.assertIn(name, cmake)
    self.assertIn('<depend>stereo_msgs</depend>', package)
    self.assertIn('<exec_depend>stereo_image_proc</exec_depend>', package)
```

- [ ] **Step 2: Run test and verify failure**

Run: `python -m unittest test.test_package_metadata`

Expected: FAIL because message names and stereo dependencies are absent.

- [ ] **Step 3: Add exact message definitions**

`msg/ObstacleCluster.msg`:

```text
float32 forward_m
float32 left_m
float32 nearest_range_m
float32 left_edge_m
float32 right_edge_m
float32 confidence
```

`msg/ObstacleArray.msg`:

```text
std_msgs/Header header
firefighting_mission/ObstacleCluster[] obstacles
bool ready
string reason
```

`msg/AvoidanceStatus.msg`:

```text
std_msgs/Header header
string state
string selected_side
float32 left_clearance_m
float32 right_clearance_m
string reason
geometry_msgs/Point target
float32 target_yaw
```

Add all three files to `add_message_files`. Add `stereo_msgs` to catkin components/dependencies, `stereo_image_proc` as runtime dependency, and keep `generate_messages(DEPENDENCIES geometry_msgs std_msgs)`.

- [ ] **Step 4: Build and run metadata tests**

Run in VM:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src/firefighting_mission
python -m unittest test.test_package_metadata
```

Expected: build succeeds; metadata suite passes.

- [ ] **Step 5: Commit**

```bash
git add msg CMakeLists.txt package.xml test/test_package_metadata.py
git commit -m "feat: define stereo avoidance ROS contracts"
```

---

### Task 2: Metric Stereo Obstacle Extraction

**Files:**
- Create: `src/firefighting_mission/stereo_obstacles.py`
- Create: `scripts/stereo_obstacle_node.py`
- Create: `launch/stereo_input.launch`
- Create: `test/test_stereo_obstacles.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `sensor_msgs/Image + CameraInfo` or `sensor_msgs/PointCloud2`.
- Produces: `ObstacleArray`; pure core returns tuple of `ObstacleClusterData`.
- Public pure signatures:
  - `clusters_from_depth(depth_m, fx, cx, min_depth=0.25, max_depth=3.0) -> tuple`
  - `clusters_from_points(points_xyz, min_depth=0.25, max_depth=3.0) -> tuple`
  - `stable_clusters(history, required=3) -> tuple`

- [ ] **Step 1: Write failing pure tests**

```python
def test_depth_patch_becomes_metric_body_cluster(self):
    depth = np.full((60, 80), np.nan, dtype=np.float32)
    depth[20:45, 36:44] = 1.0
    clusters = clusters_from_depth(depth, fx=80.0, cx=40.0)
    self.assertEqual(1, len(clusters))
    self.assertAlmostEqual(1.0, clusters[0].forward_m, places=2)
    self.assertAlmostEqual(0.0, clusters[0].left_m, places=2)

def test_floor_band_and_single_pixel_noise_are_rejected(self):
    depth = np.full((60, 80), np.nan, dtype=np.float32)
    depth[58, :] = 0.5
    depth[25, 40] = 0.8
    self.assertEqual((), clusters_from_depth(depth, 80.0, 40.0))

def test_point_cloud_and_depth_use_same_forward_left_convention(self):
    points = [(1.0, 0.20, 0.0), (1.0, 0.21, 0.02),
              (1.02, 0.19, -0.02)]
    cluster = clusters_from_points(points)[0]
    self.assertGreater(cluster.left_m, 0.0)
```

- [ ] **Step 2: Run test and verify import failure**

Run: `python -m unittest test.test_stereo_obstacles`

Expected: FAIL with `ImportError: No module named stereo_obstacles`.

- [ ] **Step 3: Implement focused pure extraction core**

Use Python-2-compatible named tuples and deterministic bin clustering:

```python
ObstacleClusterData = namedtuple(
    'ObstacleClusterData',
    'forward_m left_m nearest_range_m left_edge_m right_edge_m confidence')

def _cluster_horizontal_samples(samples, bin_width=0.08, min_samples=8):
    bins = {}
    for forward, left in samples:
        key = int(math.floor(left / bin_width))
        bins.setdefault(key, []).append((forward, left))
    groups = []
    for key in sorted(bins):
        if groups and key <= groups[-1][-1][0] + 1:
            groups[-1].append((key, bins[key]))
        else:
            groups.append([(key, bins[key])])
    return tuple(_summarize(group, min_samples) for group in groups
                 if sum(len(values) for _, values in group) >= min_samples)
```

`clusters_from_depth` crops vertical rows `20%--80%`, samples every second pixel, converts `u` to left distance with `left = -(u-cx)*depth/fx`, and rejects invalid range. `clusters_from_points` accepts `(forward,left,up)` tuples and keeps `-0.45 <= up <= 0.45`. `_summarize` uses nearest-range percentile and lateral extrema; confidence is capped sample count divided by `50.0`.

- [ ] **Step 4: Add ROS adapter with explicit input modes**

`scripts/stereo_obstacle_node.py` parameters:

```python
self.input_mode = rospy.get_param('~input_mode', 'depth')
self.depth_topic = rospy.get_param('~depth_topic', '/fire_stereo/depth/image_raw')
self.info_topic = rospy.get_param('~camera_info_topic', '/fire_stereo/depth/camera_info')
self.points_topic = rospy.get_param('~points_topic', '/fire_stereo/points')
self.output = rospy.Publisher('/fire_mission/obstacles', ObstacleArray,
                              queue_size=1)
```

Depth mode synchronizes `Image` and `CameraInfo`, converts `16UC1` millimetres or `32FC1` metres, then calls `clusters_from_depth`. Point-cloud mode reads `(x,y,z)` with `sensor_msgs.point_cloud2.read_points`, maps optical coordinates to body coordinates as `(forward=z, left=-x, up=-y)`, then calls `clusters_from_points`. Publish `ready=false` with exact reasons `camera_info_missing`, `unsupported_encoding`, `depth_stale`, or `pointcloud_stale`.

`launch/stereo_input.launch` exposes `input_mode`, left/right namespaces, depth,
camera-info, and point-cloud topic arguments. In `raw_stereo` mode it starts the
standard calibrated `stereo_image_proc` chain and routes its point-cloud output
to the adapter. In `depth` and `points` modes it starts only the adapter. Raw
mode requires synchronized rectified images and nonzero baseline from both
`CameraInfo` messages.

- [ ] **Step 5: Register node/tests and run**

Add `scripts/stereo_obstacle_node.py` to `catkin_install_python` and `test/test_stereo_obstacles.py` to `catkin_add_nosetests`.

Run: `python -m unittest test.test_stereo_obstacles`

Expected: all extraction tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/firefighting_mission/stereo_obstacles.py scripts/stereo_obstacle_node.py launch/stereo_input.launch test/test_stereo_obstacles.py CMakeLists.txt
git commit -m "feat: extract metric obstacles from stereo depth"
```

---

### Task 3: Fixed-Map Global Route Planner

**Files:**
- Create: `src/firefighting_mission/field_map.py`
- Create: `test/test_field_map.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `FIELD_BOUNDS`, `FIXED_OBSTACLES`, start `(x,y)`, goal `(x,y)`.
- Produces: `plan_route(start, goal, resolution=0.10, inflation=0.45) -> tuple[(x,y)]`.
- Does not import `CYLINDER_POSES`, `Scenario`, or seed data.

- [ ] **Step 1: Write failing route tests**

```python
def test_route_avoids_every_inflated_fixed_board(self):
    route = plan_route((0.0, 0.0), (2.70, -1.90))
    self.assertGreater(len(route), 2)
    for point in route:
        self.assertTrue(point_is_free(point, inflation=0.45))

def test_route_source_has_no_random_cylinder_truth_dependency(self):
    source = inspect.getsource(field_map)
    self.assertNotIn('CYLINDER_POSES', source)
    self.assertNotIn('build_scenario', source)

def test_unreachable_or_outside_goal_raises_clear_error(self):
    with self.assertRaisesRegexp(ValueError, 'goal_outside_field'):
        plan_route((0.0, 0.0), (5.0, 5.0))
```

- [ ] **Step 2: Run and verify module import failure**

Run: `python -m unittest test.test_field_map`

Expected: FAIL because `field_map.py` is absent.

- [ ] **Step 3: Implement deterministic 8-connected A***

Define rotated-board occupancy from `FIXED_OBSTACLES`: transform point into each board local frame and reject when `abs(local_x) <= length/2 + inflation` and `abs(local_y) <= width/2 + inflation`. Shrink `FIELD_BOUNDS` by `inflation`. Use `0.10 m` cells, Euclidean heuristic, diagonal cost `sqrt(2)`, and reconstruct route from parent links.

Expose exact result simplification:

```python
def simplify_route(points):
    if len(points) < 3:
        return tuple(points)
    result = [points[0]]
    for index in range(1, len(points) - 1):
        before = points[index - 1]
        current = points[index]
        after = points[index + 1]
        first = (current[0] - before[0], current[1] - before[1])
        second = (after[0] - current[0], after[1] - current[1])
        if first != second:
            result.append(current)
    result.append(points[-1])
    return tuple(result)
```

Reject start/goal with exact errors `start_outside_field`, `goal_outside_field`, `start_blocked`, `goal_blocked`, or `route_unreachable`.

- [ ] **Step 4: Run route tests and existing field tests**

Run: `python -m unittest test.test_field_map test.test_world_generator`

Expected: all pass; generated world geometry remains unchanged.

- [ ] **Step 5: Commit**

```bash
git add src/firefighting_mission/field_map.py test/test_field_map.py CMakeLists.txt
git commit -m "feat: plan routes around fixed field obstacles"
```

---

### Task 4: Constant-Altitude Visual Local Planner

**Files:**
- Replace: `src/firefighting_mission/path_planner.py`
- Replace: `test/test_path_planner.py`

**Interfaces:**
- Consumes: map route, pose `(x,y,z,yaw)`, body-frame clusters, timestamp/readiness.
- Produces: `PlanCommand(state, target, target_yaw, selected_side, left_clearance, right_clearance, reason)`.
- Public methods:
  - `set_goal(goal_xyz, pose_xyzyaw)`
  - `update(pose_xyzyaw, obstacles, perception_ready, now)`

- [ ] **Step 1: Replace old overflight tests with failing competition tests**

```python
def test_follow_route_never_commands_23_metres(self):
    planner = VisualPathPlanner(route_provider=straight_route)
    planner.set_goal((2.0, 0.0, 1.2), (0.0, 0.0, 1.2, 0.0))
    command = planner.update((0.0, 0.0, 1.2, 0.0), (), True, 1.0)
    self.assertEqual('FOLLOW_ROUTE', command.state)
    self.assertEqual(1.2, command.target[2])

def test_selects_only_valid_right_corridor(self):
    planner = configured_planner(left_boundary=0.35, right_boundary=1.40)
    drive_to_select_side(planner, obstacle_at(0.80, 0.0))
    command = planner.update(POSE, OBSTACLES, True, 1.4)
    self.assertEqual('RIGHT', command.selected_side)

def test_visual_loss_holds_current_position(self):
    command = active_planner().update(POSE, (), False, 2.0)
    self.assertEqual('HOLD_UNSAFE', command.state)
    self.assertEqual(POSE[:3], command.target)
    self.assertEqual('perception_not_ready', command.reason)

def test_horizontal_states_hold_altitude_band(self):
    for command in drive_complete_avoidance_sequence():
        self.assertAlmostEqual(1.20, command.target[2], places=6)
```

- [ ] **Step 2: Run and verify failures against old staged planner**

Run: `python -m unittest test.test_path_planner`

Expected: FAIL because `VisualPathPlanner` and avoidance states do not exist.

- [ ] **Step 3: Implement state/config types**

```python
PlanCommand = namedtuple(
    'PlanCommand',
    'state target target_yaw selected_side left_clearance right_clearance reason')

class VisualPlannerConfig(object):
    def __init__(self, altitude=1.20, altitude_tolerance=0.10,
                 trigger_range=1.00, minimum_corridor=0.90,
                 avoidance_step=0.25, pass_distance=0.55,
                 waypoint_tolerance=0.12, observation_frames=3,
                 side_hysteresis=0.15):
        self.altitude = float(altitude)
        self.altitude_tolerance = float(altitude_tolerance)
        self.trigger_range = float(trigger_range)
        self.minimum_corridor = float(minimum_corridor)
        self.avoidance_step = float(avoidance_step)
        self.pass_distance = float(pass_distance)
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.observation_frames = int(observation_frames)
        self.side_hysteresis = float(side_hysteresis)
```

`VisualPathPlanner` implements `FOLLOW_ROUTE`, `BRAKE`, `OBSERVE`, `SELECT_SIDE`, `SIDESTEP`, `PASS`, `REJOIN`, `HOLD_UNSAFE`, and `REACHED`. It transforms body cluster edges to map geometry using current yaw. Corridor clearance considers inflated fixed boards and shrunken field boundary. Side remains latched until `REJOIN` completes. Any non-ready update returns current-position hold at altitude target.

- [ ] **Step 4: Implement deterministic transition rules**

Use these exact gates:

```python
if not perception_ready:
    return self._hold(pose, 'perception_not_ready')
if abs(pose[2] - self.config.altitude) > self.config.altitude_tolerance:
    return self._hold_xy_recover_z(pose, 'altitude_out_of_band')
if self.state == 'FOLLOW_ROUTE' and nearest < self.config.trigger_range:
    self.state = 'BRAKE'
if self.state == 'BRAKE':
    self.state = 'OBSERVE'
    self.observation_count = 0
    return self._hold(pose, '')
if self.state == 'OBSERVE':
    self.observation_count += 1
    if self.observation_count >= self.config.observation_frames:
        self.state = 'SELECT_SIDE'
```

`SELECT_SIDE` chooses sole valid side, otherwise wider side only when clearance difference exceeds hysteresis; a prior safe side breaks near-ties. Neither valid produces `HOLD_UNSAFE` with `no_safe_corridor`. `SIDESTEP`, `PASS`, and `REJOIN` advance only after waypoint tolerance is reached.

- [ ] **Step 5: Run planner tests**

Run: `python -m unittest test.test_path_planner`

Expected: all constant-altitude, side-selection, hysteresis, rejoin, and failure tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/firefighting_mission/path_planner.py test/test_path_planner.py
git commit -m "feat: add constant-altitude visual avoidance planner"
```

---

### Task 5: ROS Planner Adapter and Sole MAVROS Command Owner

**Files:**
- Replace: `scripts/path_planner.py`
- Modify: `src/firefighting_mission/competition_main.py`
- Modify: `scripts/competition_main.py`
- Replace: `test/path_planner_ros_test.py`
- Modify: `test/test_competition_main.py`
- Modify: `test/path_planner.test`

**Interfaces:**
- Planner consumes `/fire_mission/goal`, MAVROS pose, `/fire_mission/obstacles`.
- Planner publishes `/fire_mission/path_setpoint`, `/fire_mission/path_status`, `/fire_mission/avoidance_status`.
- Competition main consumes `/xtdrone/iris_0/cmd` for `AUTO.LAND` and `DISARM` compatibility.
- Competition main alone publishes `/mavros/setpoint_position/local`.

- [ ] **Step 1: Write failing yaw and command-owner tests**

```python
def test_planned_setpoint_preserves_yaw(self):
    point = PositionSetpoint(1.0, 2.0, 1.2, 1.5708)
    self.assertAlmostEqual(1.5708, point.yaw, places=4)

def test_active_launch_has_one_mavros_setpoint_owner(self):
    root = ET.parse('launch/firefighting.launch').getroot()
    node_types = [node.attrib.get('type') for node in root.findall('node')]
    self.assertIn('competition_main.py', node_types)
    self.assertNotIn('mavros_bridge_node.py', node_types)
    self.assertNotIn('navigator_node.py', node_types)
```

Update ROS contract test to publish ready `ObstacleArray`, send `/fire_mission/goal`, and expect target `z == 1.2` plus `FOLLOW_ROUTE`; remove climb/cruise/descend expectations.

- [ ] **Step 2: Run and verify old-interface failures**

Run: `python -m unittest test.test_competition_main`

Run in VM: `rostest firefighting_mission path_planner.test`

Expected: failures show missing yaw and old 2.30 m stages.

- [ ] **Step 3: Add yaw-bearing controller setpoint**

Replace named tuple with Python-2-compatible class:

```python
class PositionSetpoint(object):
    def __init__(self, x, y, z, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)
```

In `scripts/competition_main.py`, extract and publish yaw:

```python
def quaternion_yaw(orientation):
    return math.atan2(2.0 * (orientation.w * orientation.z +
                            orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 +
                                   orientation.z ** 2))

def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
```

Set `pose.pose.orientation.z/w` from planner yaw. Preserve existing prestream, OFFBOARD, arm, takeoff, and manual-command service behavior.

- [ ] **Step 4: Replace planner ROS adapter**

Subscribe to `ObstacleArray`; convert messages to pure `ObstacleClusterData`. Convert MAVROS quaternion to yaw. Publish `AvoidanceStatus` every tick. Publish `path_status='REACHED'` only when planner returns `REACHED`; otherwise publish current avoidance state. Require a ready obstacle message before horizontal movement.

- [ ] **Step 5: Run unit, ROS, and control ownership tests**

Run:

```bash
python -m unittest test.test_competition_main test.test_path_planner
rostest firefighting_mission path_planner.test
```

Expected: all pass; target altitude is 1.20 m and target quaternion carries route yaw.

- [ ] **Step 6: Commit**

```bash
git add scripts/path_planner.py scripts/competition_main.py src/firefighting_mission/competition_main.py test/path_planner_ros_test.py test/path_planner.test test/test_competition_main.py
git commit -m "feat: connect visual planner to sole offboard controller"
```

---

### Task 6: Gazebo Forward Stereo/Depth Contract

**Files:**
- Create: `models/fire_stereo_camera/model.config`
- Create: `models/fire_stereo_camera/model.sdf`
- Modify: `models/fire_iris/fire_iris.sdf`
- Modify: `models/fire_iris/model.config`
- Create: `test/test_stereo_model.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces `/fire_stereo/depth/image_raw`, `/fire_stereo/depth/camera_info`, `/fire_stereo/points`, and `/fire_stereo/rgb/image_raw`.
- Optical frame: `fire_stereo_optical_frame`; body mount faces aircraft positive X.

- [ ] **Step 1: Write failing SDF contract test**

```python
def test_forward_sensor_exposes_metric_depth_contract(self):
    root = ET.parse('models/fire_stereo_camera/model.sdf').getroot()
    sensor = root.find(".//sensor[@type='depth']")
    self.assertIsNotNone(sensor)
    plugin = sensor.find("plugin[@filename='libgazebo_ros_depth_camera.so']")
    self.assertEqual('/fire_stereo/depth/image_raw',
                     plugin.find('depthImageTopicName').text)
    self.assertEqual('/fire_stereo/points',
                     plugin.find('pointCloudTopicName').text)

def test_sensor_mount_faces_forward(self):
    iris = ET.parse('models/fire_iris/fire_iris.sdf').getroot()
    joint = iris.find(".//joint[@name='fire_stereo_joint']")
    self.assertIsNotNone(joint)
```

- [ ] **Step 2: Run and verify missing-model failure**

Run: `python -m unittest test.test_stereo_model`

Expected: FAIL because model files and mount are absent.

- [ ] **Step 3: Add simulation sensor model**

Create one lightweight depth sensor at `320x180` (chosen after VM profiling to preserve safety watchdog cadence), horizontal FOV about `1.40 rad`, update rate `15 Hz`, near clip `0.20 m`, far clip `4.0 m`. Use `libgazebo_ros_depth_camera.so` with exact topics above. Add two small lens visuals separated by a configurable-looking `0.08 m` baseline; visuals document stereo hardware while Gazebo depth supplies deterministic metric output equivalent to calibrated stereo processing.

Mount at front of `base_link`, above payload, with no collision geometry. Update description to “lidar, downward target camera, forward stereo-depth camera, and dual payload bay.”

- [ ] **Step 4: Run XML/model tests**

Run: `python -m unittest test.test_stereo_model test.test_orchestration`

Expected: all pass and SDF parses.

- [ ] **Step 5: Validate live topics in VM**

```bash
roslaunch firefighting_mission competition_takeoff.launch sdf:=$(rospack find firefighting_mission)/models/fire_iris/fire_iris.sdf
rostopic hz /fire_stereo/depth/image_raw
rostopic echo -n 1 /fire_stereo/depth/camera_info
rostopic echo -n 1 /fire_stereo/points
```

Expected: depth near 15 Hz; camera info has nonzero focal length; point cloud frame is present.

- [ ] **Step 6: Commit**

```bash
git add models/fire_stereo_camera models/fire_iris test/test_stereo_model.py CMakeLists.txt
git commit -m "feat: add forward stereo depth simulation sensor"
```

---

### Task 7: Active Mission Launch, Safety, and Recording Overlay

**Files:**
- Modify: `launch/firefighting.launch`
- Modify: `launch/competition_takeoff.launch`
- Modify: `scripts/mission_manager_node.py`
- Modify: `src/firefighting_mission/safety.py`
- Modify: `scripts/safety_monitor_node.py`
- Create: `src/firefighting_mission/avoidance_overlay.py`
- Create: `scripts/mission_overlay_node.py`
- Modify: `scripts/mission_recorder_node.py`
- Modify: `src/firefighting_mission/orchestration.py`
- Modify: `test/test_safety.py`
- Modify: `test/test_orchestration.py`
- Create: `test/test_avoidance_overlay.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Active launch starts stereo adapter, path planner, competition main, mission manager, target detector, payload nodes, overlay, safety, and recorder.
- Removes active `navigator_node.py` and `mavros_bridge_node.py`.
- Recorder consumes `/fire_mission/mission_view` and records obstacles/status in bag.

- [ ] **Step 1: Write failing safety and launch tests**

```python
def test_stereo_stale_requests_hover_before_land(self):
    monitor = SafetyMonitor(stale_hover=0.30, stale_land=1.00)
    self.assertEqual('HOVER', monitor.evaluate(
        pose_age=0.1, scan_age=0.1, stereo_age=0.31,
        roll=0.0, pitch=0.0, altitude=1.2,
        minimum_obstacle=1.0, boundary_margin=1.0).action)

def test_launch_starts_visual_avoidance_chain(self):
    root = ET.parse('launch/firefighting.launch').getroot()
    types = [node.attrib.get('type') for node in root.findall('node')]
    for expected in ('stereo_obstacle_node.py', 'path_planner.py',
                     'competition_main.py', 'mission_overlay_node.py'):
        self.assertIn(expected, types)
    self.assertNotIn('navigator_node.py', types)
    self.assertNotIn('mavros_bridge_node.py', types)
```

- [ ] **Step 2: Run and verify failures**

Run: `python -m unittest test.test_safety test.test_orchestration test.test_avoidance_overlay`

Expected: failures show missing stereo age, nodes, and overlay.

- [ ] **Step 3: Extend safety and mission readiness**

Safety monitor subscribes `/fire_mission/obstacles`, records message age/readiness, and returns `HOVER:stereo_stale` after `0.30 s`; configured controlled-land action after `1.00 s`. Mission manager treats `HOLD_UNSAFE` as not reached and leaves current mission phase unchanged. It continues publishing mission goals on `/fire_mission/goal`.

Change all airborne entries in `mission_manager_node.py::GOALS` from `1.30` to
`1.20`; keep `LAND` and `EMERGENCY_LAND` at `0.08`. Target alignment may request
small explicit vertical adjustment later, but this feature introduces none.

- [ ] **Step 4: Implement recording overlay**

Pure overlay signature:

```python
def draw_avoidance_overlay(image, status, obstacles):
    output = image.copy()
    cv2.putText(output, '%s %s' % (status.state, status.selected_side),
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(output, 'L %.2fm  R %.2fm' %
                (status.left_clearance_m, status.right_clearance_m),
                (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 255), 1, cv2.LINE_AA)
    return output
```

ROS overlay node subscribes forward RGB, `ObstacleArray`, `AvoidanceStatus`, and existing target-annotated image. It publishes a fixed-size picture-in-picture `/fire_mission/mission_view`; target view remains primary during task alignment, front avoidance view remains visible in inset.

- [ ] **Step 5: Rewire launch and recorder**

Add launch args `stereo_input_mode`, `depth_topic`, `points_topic`, and `transit_altitude=1.2`. Start new nodes. Remove active navigator/bridge nodes. Subscribe recorder to mission view; extend `recording_topics` with obstacle array, avoidance status, forward RGB/depth, and path status. Keep lidar safety input as secondary independent proximity check.

- [ ] **Step 6: Run unit and launch-contract suites**

Run:

```bash
python -m unittest discover -s test -p 'test_*.py'
rostest firefighting_mission path_planner.test
```

Expected: all tests pass; no old 2.30 m assertion remains in active planner tests.

- [ ] **Step 7: Commit**

```bash
git add launch scripts src/firefighting_mission test CMakeLists.txt
git commit -m "feat: activate stereo avoidance mission chain"
```

---

### Task 8: Scenario Matrix and VM Flight Acceptance

**Files:**
- Create: `scripts/run_avoidance_matrix.sh`
- Create: `test/visual_avoidance_smoke.test`
- Create: `test/visual_avoidance_smoke.py`
- Modify: `CMakeLists.txt`
- Modify: `docs/TEAM_C_HANDOFF.zh-CN.md`
- Modify: `docs/CODE_INVENTORY_AND_PROGRESS.zh-CN.md`

**Interfaces:**
- Matrix drives deterministic seeds covering all two-cylinder position combinations.
- Acceptance consumes trajectory, avoidance status, contact topic, and mission completion.

- [ ] **Step 1: Write failing smoke-test assertions**

```python
def test_constant_altitude_visual_pass(self):
    self.assertTrue(self.seen_states.issuperset(
        set(('BRAKE', 'OBSERVE', 'SELECT_SIDE', 'SIDESTEP', 'PASS', 'REJOIN'))))
    self.assertFalse(self.collision)
    self.assertLessEqual(self.maximum_transit_altitude, 1.30)
    self.assertGreaterEqual(self.minimum_transit_altitude, 1.10)
    self.assertTrue(self.reached_goal)
```

Add a source inspection assertion that active planner and launch do not import/read `CYLINDER_POSES` or `/gazebo/model_states` for avoidance.

- [ ] **Step 2: Run smoke test and verify missing-test support**

Run in VM: `rostest firefighting_mission visual_avoidance_smoke.test`

Expected: FAIL because launch/test files are absent.

- [ ] **Step 3: Implement deterministic four-combination matrix**

`run_avoidance_matrix.sh` iterates four known seeds selected by `build_scenario(seed).cylinder_positions`, runs headless launch with a `150 s` timeout, and verifies `score.json`, collision false, altitude band, selected-side evidence, and goal completion. The test may inspect scenario selection only to choose coverage seeds; runtime planner inputs remain vision-only.

- [ ] **Step 4: Run complete automated validation**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src/firefighting_mission
python -m unittest discover -s test -p 'test_*.py'
rostest firefighting_mission path_planner.test
rostest firefighting_mission visual_avoidance_smoke.test
rosrun firefighting_mission run_avoidance_matrix.sh
```

Expected: build and all unit/ROS/matrix tests pass; every run reports no collision and horizontal altitude within `1.10--1.30 m`.

- [ ] **Step 5: Perform visible Gazebo acceptance**

```bash
roslaunch firefighting_mission firefighting.launch gui:=true record:=true seed:=4501
```

Observe: takeoff to 1.20 m, fixed-map routing, visual brake at cylinder, side selection, lateral pass, route rejoin, task continuation, return, landing. Confirm overlay displays obstacle distance and selected side.

- [ ] **Step 6: Validate sensor-loss safety**

Pause stereo publication during guarded simulation.

Expected: horizontal movement stops within configured stale timeout; status becomes `HOLD_UNSAFE`/`stereo_stale`; flight does not continue blind.

- [ ] **Step 7: Update handoff documentation**

Document exact files by responsibility, launch command, stereo input modes, calibration prerequisite, topic contract, tunable clearance values, test evidence, remaining real-aircraft steps, and rollback command to previous commit. Mark real-flight acceptance incomplete until final camera calibration and guarded aircraft tests pass.

- [ ] **Step 8: Commit final evidence and documentation**

```bash
git add scripts/run_avoidance_matrix.sh test/visual_avoidance_smoke.test test/visual_avoidance_smoke.py CMakeLists.txt docs
git commit -m "test: verify stereo avoidance scenario matrix"
```

## Final Verification Gate

- [ ] `git diff --check` returns no output.
- [ ] Full Python 2.7 unit suite passes in VM.
- [ ] `catkin_make` and both rostests pass.
- [ ] Four random-cylinder combinations pass headless matrix.
- [ ] Visible Gazebo run stays near 1.20 m and shows visual avoidance overlay.
- [ ] Stereo interruption causes safe hover.
- [ ] Active launch has one MAVROS setpoint owner.
- [ ] No runtime cylinder truth dependency exists.
- [ ] Payload, hazard/person detection, return, landing, and recording regressions pass.
- [ ] Branch is pushed only after evidence is recorded and reviewed.
