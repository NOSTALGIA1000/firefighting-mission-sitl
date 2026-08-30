# PX4 External Vision SITL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace unstable PX4 SITL GPS horizontal aiding with Gazebo-backed external vision while preserving one localization interface for future stereo VIO.

**Architecture:** A dedicated ROS bridge publishes `iris_0` Gazebo pose and velocity to MAVROS external-vision inputs. PX4 `10016_iris.post` selects vision position, velocity, and height before boot completes; GPS remains present but is not fused. Existing planner keeps map coordinates and `competition_main.py` remains sole OFFBOARD position-setpoint publisher.

**Tech Stack:** ROS Melodic, Python 2.7/3 compatible `rospy`, Gazebo 9 `ModelStates`, MAVROS `vision_pose/pose`, PX4 v1.11 EKF2.

## Runtime findings (2026-08-30)

- PX4 now fuses Gazebo-backed external-vision position, velocity, and yaw through `/mavros/odometry/out` (`EKF2_AID_MASK=280`); SITL magnetometer fusion is disabled.
- Seed 1 passed once end-to-end: no collision, `REACHED`, altitude `1.158-1.211m`, maximum pose disagreement `0.042m`.
- Repeat runs still fail near safety nets with about `0.22m` MAVROS/Gazebo horizontal lag.
- Bridge rate is 50 Hz. ULog diagnosis found output predictor tracking error up to about `0.223m` position and `0.344m/s` velocity during in-place route-yaw alignment.
- Position+yaw-only mask `24` increased Gazebo/MAVROS disagreement to about `0.43m`; experiment was reverted.
- Next experiment: reduce `EKF2_TAU_POS` and `EKF2_TAU_VEL` within PX4-supported bounds, then repeat seed 1.
- Four-seed matrix has not run. Treat this branch as a checkpoint, not completed acceptance.

## Global Constraints

- Keep flight target altitude at `1.20m`; acceptance band remains `1.10–1.30m`.
- Do not use random-cylinder Gazebo truth for avoidance.
- Keep external-vision bridge simulation-only; real hardware must replace source with calibrated stereo VIO.
- Do not edit PX4 source files manually; install package-owned `.post` content idempotently.
- `competition_main.py` remains sole publisher to `/mavros/setpoint_position/local`.

---

### Task 1: Gazebo External-Vision Bridge

**Files:**
- Create: `scripts/gazebo_vision_bridge.py`
- Create: `src/firefighting_mission/external_vision.py`
- Create: `test/test_external_vision.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `/gazebo/model_states` (`gazebo_msgs/ModelStates`).
- Produces: `/mavros/vision_pose/pose` (`geometry_msgs/PoseStamped`) at 30 Hz.
- Produces pure helper: `model_pose(message, model_name) -> pose or None`.

- [ ] **Step 1: Write failing helper and package-contract tests**

```python
def test_selects_named_model_pose():
    message = FakeModelStates(['ground', 'iris_0'], [GROUND, IRIS])
    self.assertIs(IRIS, model_pose(message, 'iris_0'))

def test_bridge_is_installed():
    self.assertIn('scripts/gazebo_vision_bridge.py', cmake_text)
```

- [ ] **Step 2: Run tests and confirm missing module/script failure**

```bash
PYTHONPATH=src python -m unittest test.test_external_vision test.test_package_metadata
```

Expected: import or installation assertion fails.

- [ ] **Step 3: Implement minimal selector and bridge**

```python
def model_pose(message, model_name):
    try:
        return message.pose[message.name.index(model_name)]
    except (ValueError, IndexError):
        return None
```

Bridge republishes latest pose with current ROS stamp and `frame_id='map'`; it publishes nothing before model appears.

- [ ] **Step 4: Run helper, metadata, and syntax tests**

```bash
PYTHONPATH=src python -m unittest test.test_external_vision test.test_package_metadata
python -m py_compile scripts/gazebo_vision_bridge.py
```

Expected: PASS.

### Task 2: PX4 EKF2 Vision Configuration

**Files:**
- Create: `config/px4/10016_iris.post`
- Modify: `scripts/start_sitl.sh`
- Modify: `test/test_orchestration.py`

**Interfaces:**
- Installs package file into `$PX4_FIRMWARE_DIR/ROMFS/px4fmu_common/init.d-posix/10016_iris.post` before ROS launch.
- Sets `EKF2_AID_MASK 8`, `EKF2_HGT_MODE 3`, and `EKF2_EV_DELAY 0`.

- [ ] **Step 1: Write failing configuration-contract test**

```python
def test_sitl_configures_px4_for_external_vision():
    self.assertIn('param set EKF2_AID_MASK 8', post_text)
    self.assertIn('param set EKF2_HGT_MODE 3', post_text)
    self.assertIn('10016_iris.post', start_script)
```

- [ ] **Step 2: Run test and confirm missing file failure**

```bash
PYTHONPATH=src python -m unittest test.test_orchestration
```

Expected: missing post file/assertion failure.

- [ ] **Step 3: Add post file and idempotent installer**

```sh
param set EKF2_AID_MASK 8
param set EKF2_HGT_MODE 3
param set EKF2_EV_DELAY 0
```

Installer creates target only when absent or package content matches; conflicting pre-existing content fails with clear error.

- [ ] **Step 4: Run orchestration tests**

```bash
PYTHONPATH=src python -m unittest test.test_orchestration
```

Expected: PASS.

### Task 3: Launch Integration and SITL Evidence

**Files:**
- Modify: `launch/competition_takeoff.launch`
- Modify: `launch/firefighting.launch`
- Modify: `test/visual_avoidance_smoke.test`
- Modify: `test/visual_avoidance_smoke.py`
- Modify: `test/test_competition_main.py`

**Interfaces:**
- Launches `gazebo_vision_bridge.py` only when `use_gazebo_ground_truth=true`.
- Smoke evidence records maximum horizontal disagreement between MAVROS and Gazebo after hover.

- [ ] **Step 1: Write failing launch-contract tests**

```python
def test_takeoff_launch_starts_external_vision_bridge():
    self.assertIn('gazebo_vision_bridge.py', node_types)
```

- [ ] **Step 2: Run test and confirm launch assertion failure**

```bash
PYTHONPATH=src python -m unittest test.test_competition_main
```

Expected: bridge node missing.

- [ ] **Step 3: Add bridge nodes and smoke pose-alignment evidence**

Bridge uses `model_name=iris_0`, `output_topic=/mavros/vision_pose/pose`, and `publish_rate=30.0`.

- [ ] **Step 4: Run local regression**

```bash
PYTHONPATH=src python -m unittest test.test_competition_main test.test_orchestration test.test_path_planner test.test_external_vision test.test_field_map test.test_stereo_obstacles test.test_stereo_model
```

Expected: all tests PASS.

- [ ] **Step 5: Build and run seed 1 in VM**

```bash
cd /home/ss/catkin_ws
catkin_make
source devel/setup.bash
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
```

Expected: no GPS position reset after takeoff, no collision, altitude inside `1.10–1.30m`, terminal path state `REACHED`.

- [ ] **Step 6: Repeat seed 1, then run matrix**

```bash
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
rosrun firefighting_mission run_avoidance_matrix.sh
```

Expected: repeat seed 1 passes before matrix execution.

- [ ] **Step 7: Commit only after verified evidence**

```bash
git add CMakeLists.txt config/px4/10016_iris.post scripts/gazebo_vision_bridge.py scripts/start_sitl.sh src/firefighting_mission/external_vision.py launch test docs/superpowers/plans/2026-08-28-px4-external-vision-sitl.md
git commit -m "feat: fuse external vision in PX4 SITL"
```
