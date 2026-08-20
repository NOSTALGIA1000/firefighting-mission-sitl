# Firefighting SITL Mission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a one-command PX4/XTDrone/Gazebo simulation that autonomously completes the intelligent low-altitude firefighting mission in under 180 seconds.

**Architecture:** A new ROS Melodic catkin package owns a finite-state mission manager and separate navigation, perception, payload, safety, scoring, and world-generation components. The package reuses XTDrone's Iris/PX4 command bridge but keeps all competition behavior and Gazebo assets isolated from upstream repositories.

**Tech Stack:** Ubuntu 18.04, Python 2.7-compatible ROS nodes, ROS Melodic, Gazebo 9 model plugin in C++14, PX4 SITL, MAVROS, XTDrone, OpenCV, rostest, unittest.

## Global Constraints

- Runtime workspace: `/home/ss/catkin_ws/src/firefighting_mission` in the VM at `192.168.46.128`.
- Editable source mirror: `F:\VM\firefighting_mission`.
- Do not modify `/home/ss/PX4_Firmware` or `/home/ss/XTDrone` tracked files.
- Use the existing Iris/PX4 SITL first; do not convert the SolidWorks 450 model in this phase.
- No UWB, motion capture, SLAM, YOLO training, or operator input after the one-command start.
- Normal flight altitude is 1.2–1.4 m; do not fly over 2 m obstacles.
- Total mission runtime must not exceed 180 seconds.
- Hazard boxes are red; person boxes are blue.
- Both payloads must become dynamic Gazebo bodies and fall under gravity after release.
- Four deterministic random seeds must pass the end-to-end scenario suite.

---

### Task 1: Package Contract and Mission State Machine

**Files:**
- Create: `CMakeLists.txt`
- Create: `package.xml`
- Create: `setup.py`
- Create: `msg/TargetDetection.msg`
- Create: `msg/DropResult.msg`
- Create: `msg/MissionEvent.msg`
- Create: `src/firefighting_mission/__init__.py`
- Create: `src/firefighting_mission/state_machine.py`
- Create: `test/test_state_machine.py`

**Interfaces:**
- Produces: `MissionStateMachine.tick(now, inputs) -> Command`; message definitions consumed by all later tasks.
- Consumes: no project code.

- [ ] **Step 1: Write the failing state-machine tests**

```python
def test_nominal_transition_sequence():
    sm = MissionStateMachine(start_time=0.0)
    for phase, inputs in nominal_inputs():
        command = sm.tick(inputs.now, inputs)
        assert command.phase == phase

def test_forces_return_at_165_seconds():
    sm = MissionStateMachine(start_time=0.0, phase='SEARCH_PERSON')
    assert sm.tick(165.0, Inputs.ready()).phase == 'RETURN_HOME'

def test_never_drops_without_stable_detection_and_alignment():
    sm = MissionStateMachine(start_time=0.0, phase='ALIGN_HAZARD')
    command = sm.tick(10.0, Inputs(detection_confirmed=False, aligned=True))
    assert command.drop_channel == 0
```

- [ ] **Step 2: Run the focused tests and confirm failure**

Run: `python -m unittest test.test_state_machine -v`

Expected: import failure for `firefighting_mission.state_machine`.

- [ ] **Step 3: Implement the minimal explicit transition table**

```python
NOMINAL_NEXT = {
    'WAIT_READY': 'ARM', 'ARM': 'TAKEOFF', 'TAKEOFF': 'SEARCH_HAZARD',
    'SEARCH_HAZARD': 'ALIGN_HAZARD', 'ALIGN_HAZARD': 'DROP_FIRE',
    'DROP_FIRE': 'SEARCH_PERSON', 'SEARCH_PERSON': 'ALIGN_PERSON',
    'ALIGN_PERSON': 'DROP_RESCUE', 'DROP_RESCUE': 'RETURN_HOME',
    'RETURN_HOME': 'LAND', 'LAND': 'DISARM', 'DISARM': 'COMPLETE',
}

def tick(self, now, inputs):
    if now - self.start_time >= 175.0 and inputs.airborne:
        return self._transition('EMERGENCY_LAND', reason='hard_deadline')
    if now - self.start_time >= 165.0 and self.phase not in TERMINAL_OR_RETURN:
        return self._transition('RETURN_HOME', reason='return_deadline')
    return self._evaluate_phase(inputs)
```

- [ ] **Step 4: Run all Task 1 tests**

Run: `python -m unittest discover -s test -p 'test_state_machine.py' -v`

Expected: all state-machine tests pass.

- [ ] **Step 5: Commit the package contract**

Run: `git add CMakeLists.txt package.xml setup.py msg src test && git commit -m "feat: define firefighting mission state machine"`

---

### Task 2: Deterministic Competition World and Vehicle Model

**Files:**
- Create: `scripts/generate_world.py`
- Create: `config/scenarios.yaml`
- Create: `worlds/firefighting.world.in`
- Create: `models/fire_iris/model.config`
- Create: `models/fire_iris/fire_iris.sdf`
- Create: `models/fire_payload/model.config`
- Create: `models/fire_payload/model.sdf`
- Create: `models/rescue_payload/model.config`
- Create: `models/rescue_payload/model.sdf`
- Create: `models/targets/materials/scripts/targets.material`
- Create: `test/test_world_generator.py`

**Interfaces:**
- Produces: `generate_world(seed, output_path) -> Scenario`; a world containing fixed obstacles, randomized cylinders, randomized task targets, and spawn metadata.
- Consumes: existing XTDrone Iris SDF as the base copied into `models/fire_iris/fire_iris.sdf` before applying package-owned sensor/payload additions.

- [ ] **Step 1: Test dimensions and determinism**

```python
def test_seed_is_reproducible():
    root = tempfile.mkdtemp(prefix='fire-world-')
    a = generate_world(4501, os.path.join(root, 'a.world'))
    b = generate_world(4501, os.path.join(root, 'b.world'))
    assert a == b

def test_competition_geometry():
    scenario = build_scenario(4501)
    assert scenario.bounds == (4.0, 4.0, 3.0)
    assert len(scenario.fixed_obstacles) == 4
    assert all(o.height == 2.0 for o in scenario.fixed_obstacles)
    assert all(o.diameter == 0.2 for o in scenario.random_cylinders)
```

- [ ] **Step 2: Verify generator tests fail**

Run: `python -m unittest test.test_world_generator -v`

Expected: missing `generate_world` import.

- [ ] **Step 3: Implement seeded scenario selection and strict template replacement**

```python
def build_scenario(seed):
    rng = random.Random(int(seed))
    return Scenario(
        seed=int(seed),
        cylinder_positions=(rng.choice((1, 2)), rng.choice((1, 2))),
        hazard_index=rng.choice((1, 2)),
        hazard_symbol=rng.choice(('flammable', 'explosive', 'toxic')),
        person_position=rng.choice((1, 2, 3)),
    )
```

- [ ] **Step 4: Generate and parse all four worlds**

Run: `for seed in 4501 4502 4503 4504; do rosrun firefighting_mission generate_world.py --seed $seed --output /tmp/fire-$seed.world; gz sdf -k /tmp/fire-$seed.world; done`

Expected: each `gz sdf -k` exits 0 and reports no XML/SDF errors.

- [ ] **Step 5: Commit the world and vehicle assets**

Run: `git add scripts config worlds models test && git commit -m "feat: add deterministic firefighting Gazebo world"`

---

### Task 3: Navigator and Safety Monitor

**Files:**
- Create: `src/firefighting_mission/navigation.py`
- Create: `src/firefighting_mission/safety.py`
- Create: `scripts/navigator_node.py`
- Create: `scripts/safety_monitor_node.py`
- Create: `config/mission.yaml`
- Create: `test/test_navigation.py`
- Create: `test/test_safety.py`

**Interfaces:**
- Consumes: `/fire_mission/goal`, `/iris_0/mavros/local_position/pose`, `/scan`, `/iris_0/mavros/state`.
- Produces: `/xtdrone/iris_0/cmd_vel_flu`, `/fire_mission/nav_status`, `/fire_mission/safety_override`.

- [ ] **Step 1: Test sector selection, limits, and stale-input behavior**

```python
def test_turns_to_clearer_side_when_front_blocked():
    nav = Navigator(NavigationConfig())
    cmd = nav.compute(goal=(2, 0), pose=(0, 0), sectors=(0.5, 1.4, 0.6))
    assert cmd.y > 0.0

def test_emergency_stop_inside_thirty_centimeters():
    cmd = Navigator(NavigationConfig()).compute((2, 0), (0, 0), (0.29, 1, 1))
    assert cmd.x <= 0.0

def test_stale_pose_requests_land():
    assert SafetyMonitor().evaluate(pose_age=0.6).action == 'LAND'
```

- [ ] **Step 2: Run and observe failure**

Run: `python -m unittest test.test_navigation test.test_safety -v`

Expected: missing navigation and safety modules.

- [ ] **Step 3: Implement bounded proportional guidance and sector overrides**

```python
if front < 0.30:
    return Velocity(x=-0.15, y=0.0, z=hold_z)
if front < 0.45:
    return Velocity(x=0.0, y=0.25 if left > right else -0.25, z=hold_z)
if front < 0.70:
    lateral = 0.18 if left > right else -0.18
    return Velocity(x=0.08, y=lateral, z=hold_z)
return bounded_position_command(goal, pose, max_xy=0.55, max_z=0.30)
```

- [ ] **Step 4: Run unit and synthetic ROS integration tests**

Run: `python -m unittest test.test_navigation test.test_safety -v && rostest firefighting_mission navigation.test`

Expected: unit tests pass; rostest observes a nonzero lateral correction and no unsafe forward command.

- [ ] **Step 5: Commit navigation and safety**

Run: `git add src scripts config test && git commit -m "feat: add lidar navigation and safety monitor"`

---

### Task 4: Template Perception and Annotated Video

**Files:**
- Create: `src/firefighting_mission/perception.py`
- Create: `scripts/target_detector_node.py`
- Create: `assets/templates/flammable.png`
- Create: `assets/templates/explosive.png`
- Create: `assets/templates/toxic.png`
- Create: `assets/templates/person.png`
- Create: `assets/templates/distractor.png`
- Create: `test/images/hazard_scene.png`
- Create: `test/images/person_scene.png`
- Create: `test/test_perception.py`

**Interfaces:**
- Consumes: configured downward camera `sensor_msgs/Image` and `/fire_mission/phase`.
- Produces: `/fire_mission/detection` and `/fire_mission/annotated`.

- [ ] **Step 1: Test class filtering, confirmation, and box colors**

```python
def test_hazard_requires_four_of_five_frames():
    detector = StableDetector(window=5, required=4)
    for result in [True, True, False, True, True]:
        confirmed = detector.update('flammable', result)
    assert confirmed

def test_distractor_never_confirms_as_hazard():
    result = TemplatePerception(fixtures()).detect(load('distractor.png'), 'SEARCH_HAZARD')
    assert result.target_class == 'distractor'

def test_annotation_colors():
    assert annotation_color('flammable') == (0, 0, 255)
    assert annotation_color('person') == (255, 0, 0)
```

- [ ] **Step 2: Verify perception tests fail**

Run: `python -m unittest test.test_perception -v`

Expected: missing perception module.

- [ ] **Step 3: Implement normalized multi-scale matching and overlay**

```python
for scale in (0.75, 1.0, 1.25):
    candidate = cv2.resize(template, None, fx=scale, fy=scale)
    scores = cv2.matchTemplate(gray, candidate, cv2.TM_CCOEFF_NORMED)
    _, confidence, _, origin = cv2.minMaxLoc(scores)
    best = max(best, Detection(label, confidence, box(origin, candidate)))
```

- [ ] **Step 4: Run fixture tests and save annotated evidence images**

Run: `python -m unittest test.test_perception -v && rosrun firefighting_mission target_detector_node.py --fixture-dir test/images --output-dir /tmp/fire-perception`

Expected: tests pass and output contains a red-box hazard image and blue-box person image.

- [ ] **Step 5: Commit perception**

Run: `git add src scripts assets test && git commit -m "feat: add template target perception"`

---

### Task 5: Physical Payload Release Simulation

**Files:**
- Create: `include/firefighting_mission/payload_plugin.hpp`
- Create: `src/payload_plugin.cpp`
- Create: `src/firefighting_mission/payload.py`
- Create: `scripts/payload_controller_node.py`
- Create: `test/test_payload.py`
- Modify: `CMakeLists.txt`
- Modify: `models/fire_iris/fire_iris.sdf`

**Interfaces:**
- Consumes: `/fire_mission/drop_request`, current pose/velocity, Gazebo model lifecycle.
- Produces: `/fire_mission/drop_result`; plugin topics `/fire_iris/drop_fire` and `/fire_iris/drop_rescue`.

- [ ] **Step 1: Test one-shot interlock and release conditions**

```python
def test_each_channel_releases_once():
    controller = PayloadController()
    assert controller.request(1, aligned=True).accepted
    assert not controller.request(1, aligned=True).accepted
    assert controller.request(2, aligned=True).accepted

def test_rejects_release_while_moving():
    controller = PayloadController()
    assert not controller.request(1, aligned=True, horizontal_speed=0.11).accepted
```

- [ ] **Step 2: Verify tests fail**

Run: `python -m unittest test.test_payload -v`

Expected: missing payload controller.

- [ ] **Step 3: Implement Gazebo joint animation and detach**

```cpp
void PayloadPlugin::Release(unsigned channel) {
  auto &slot = slots_.at(channel - 1);
  if (slot.released) return;
  slot.door_joint->SetPosition(0, 1.15);
  slot.payload_joint->Detach();
  slot.payload_link->SetGravityMode(true);
  slot.released = true;
}
```

- [ ] **Step 4: Build and run gravity/landing test**

Run: `catkin_make && rostest firefighting_mission payload_drop.test`

Expected: both links lose at least 0.8 m altitude after release, contact the ground, and report separate final positions.

- [ ] **Step 5: Commit payload simulation**

Run: `git add CMakeLists.txt include src scripts models test && git commit -m "feat: simulate dual servo payload release"`

---

### Task 6: Mission Orchestration, Recorder, and One-Command Launch

**Files:**
- Create: `scripts/mission_manager_node.py`
- Create: `scripts/mission_recorder_node.py`
- Create: `launch/firefighting.launch`
- Create: `launch/firefighting_headless.launch`
- Create: `scripts/run_mission.sh`
- Create: `src/firefighting_mission/scoring.py`
- Create: `test/test_scoring.py`
- Create: `README.zh-CN.md`

**Interfaces:**
- Consumes: every package status topic and Gazebo model states.
- Produces: `/fire_mission/phase`, `/fire_mission/goal`, `/fire_mission/drop_request`, event log, annotated MP4, bag, trajectory CSV, and `score.json`.

- [ ] **Step 1: Test hard scoring conditions**

```python
def test_pass_requires_every_hard_condition():
    score = Score(runtime=142.0, min_clearance=0.41, hazard=True, person=True,
                  fire_drop_error=0.11, rescue_drop_error=0.13,
                  landing_error=0.17, disarmed=True, collision=False)
    assert score.passed

def test_runtime_over_180_fails():
    assert not valid_score(runtime=180.01, all_other_conditions=True)
```

- [ ] **Step 2: Verify scoring tests fail**

Run: `python -m unittest test.test_scoring -v`

Expected: missing scoring module.

- [ ] **Step 3: Implement launch orchestration and atomic score output**

```python
payload = score.to_dict()
temporary = output_path + '.tmp'
with open(temporary, 'w') as handle:
    json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
os.rename(temporary, output_path)
```

- [ ] **Step 4: Run a one-command smoke mission**

Run: `roslaunch firefighting_mission firefighting_headless.launch seed:=4501 record:=true`

Expected: launch exits after disarm and writes `artifacts/4501/score.json`, logs, bag, trajectory, and annotated video.

- [ ] **Step 5: Commit orchestration and documentation**

Run: `git add scripts launch src test README.zh-CN.md && git commit -m "feat: orchestrate and record firefighting mission"`

---

### Task 7: Fault Injection and Four-Seed End-to-End Verification

**Files:**
- Create: `test/firefighting_e2e.test`
- Create: `test/fault_injection.test`
- Create: `scripts/run_scenario_suite.sh`
- Create: `scripts/summarize_suite.py`
- Create: `config/fault_cases.yaml`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: one-command launch and `score.json` from Task 6.
- Produces: `artifacts/suite-summary.json` and a nonzero exit code when any hard requirement fails.

- [ ] **Step 1: Add a suite assertion over all required seeds**

```python
def assert_suite(scores):
    required = {4501, 4502, 4503, 4504}
    assert set(item['seed'] for item in scores) == required
    assert all(item['passed'] for item in scores)
    assert max(item['runtime_seconds'] for item in scores) <= 180.0
    assert min(item['minimum_clearance_m'] for item in scores) >= 0.35
```

- [ ] **Step 2: Run the suite before final tuning**

Run: `rosrun firefighting_mission run_scenario_suite.sh 4501 4502 4503 4504`

Expected: any incomplete integration fails with the exact failing hard condition in the summary.

- [ ] **Step 3: Run fault injections**

Run: `rostest firefighting_mission fault_injection.test`

Expected: pose timeout, scan timeout, blocked route, and detection timeout each produce hover/recovery or safe landing without a collision or blind drop.

- [ ] **Step 4: Re-run all tests and the four-seed suite**

Run: `catkin_make run_tests && catkin_test_results && rosrun firefighting_mission run_scenario_suite.sh 4501 4502 4503 4504`

Expected: all unit/rostest cases pass and `suite-summary.json` reports `passed: true` for all four seeds.

- [ ] **Step 5: Inspect the generated video and scores, then commit**

Run: `git add test scripts config README.zh-CN.md && git commit -m "test: verify firefighting mission scenarios"`

Expected: the annotated video visibly shows red and blue detections, two physical drops, obstacle avoidance, return, landing, and stopped rotors; the repository is clean after commit.
