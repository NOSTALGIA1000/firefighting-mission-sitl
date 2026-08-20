# Firefighting Field Layout Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the package-owned Gazebo field so its metric layout matches Figure 1, its task zones render the real recognition images, and its 4 x 4 x 3 m boundary has visible physical safety netting.

**Architecture:** Keep `world_generator.py` as the single deterministic source of scenario geometry. Express all placements as named metric constants, render textures through a package-owned Gazebo material model, and render the enclosure as four static collision/visual models. Verify the generated SDF structurally before syncing only the mission package to the VM for a GUI comparison.

**Tech Stack:** Python 2.7-compatible `unittest`, SDF 1.6, Gazebo Classic/OGRE materials, ROS Melodic/catkin, PX4 SITL.

## Global Constraints

- Keep the takeoff-point centre at `(0.0, 0.0)` and the field bounds at `x=-0.65..3.35`, `y=-3.35..0.65`, `z=0..3.0` metres.
- Change only the `firefighting_mission` package; do not modify `/home/ss/PX4_Firmware` or `/home/ss/XTDrone`.
- Preserve Python 2.7 compatibility, deterministic seeded selection, and all existing ROS topic interfaces.
- Keep both payloads at 0.06 m cubes and no larger than the 0.08 m competition maximum.
- Do not run the autonomous mission during this work; stop after the corrected GUI field is visually accepted.

## File Structure

- Modify `src/firefighting_mission/world_generator.py`: authoritative metric constants, textured task-zone SDF, safety-net SDF.
- Modify `test/test_world_generator.py`: exact geometry, texture, clearance, enclosure, and payload-size regressions.
- Modify `models/targets/materials/scripts/targets.material`: OGRE materials for five recognition textures.
- Create `models/targets/materials/textures/{flammable,explosive,toxic,distractor,person}.png`: Gazebo copies of the existing perception templates.
- Modify `scripts/start_sitl.sh`: expose the package model directory to Gazebo.
- Modify `CMakeLists.txt`: install package models and perception assets with the package share.

---

### Task 1: Correct the Metric Field Geometry

**Files:**
- Modify: `src/firefighting_mission/world_generator.py:13-31`
- Modify: `test/test_world_generator.py`

**Interfaces:**
- Consumes: `build_scenario(seed) -> Scenario` and `generate_world(seed, output_path) -> Scenario`.
- Produces: `FIELD_BOUNDS`, `FIXED_OBSTACLES`, `CYLINDER_POSES`, `HAZARD_POSES`, `PERSON_POSES`, and generated SDF with exact metric poses.

- [ ] **Step 1: Write failing exact-coordinate and clearance tests**

Add imports and helpers to `test/test_world_generator.py`:

```python
from firefighting_mission.world_generator import (
    CYLINDER_POSES, FIELD_BOUNDS, FIXED_OBSTACLES,
    HAZARD_POSES, PERSON_POSES, physical_side_clearance)


def numbers(text):
    return tuple(float(value) for value in text.split())
```

Add tests:

```python
def test_metric_layout_matches_figure_one(self):
    self.assertEqual((-0.65, 3.35, -3.35, 0.65, 3.0), FIELD_BOUNDS)
    expected = (
        ('fixed_obstacle_1', 0.70, -0.20, 0.0, 0.10, 1.70),
        ('fixed_obstacle_2', 2.72, 0.04, 0.785398, 1.60, 0.10),
        ('fixed_obstacle_3', 0.70, -3.10, 0.0, 0.10, 0.50),
        ('fixed_obstacle_4', 2.10, -3.10, 0.0, 0.10, 0.50),
    )
    self.assertEqual(expected, FIXED_OBSTACLES)
    self.assertEqual({1: ((0.70, -1.45), (2.10, -1.45)),
                      2: ((0.70, -2.45), (2.10, -1.95))}, CYLINDER_POSES)
    self.assertEqual({1: (1.40, 0.00), 2: (1.40, -0.45)}, HAZARD_POSES)
    self.assertEqual({1: (2.70, -1.10), 2: (2.70, -1.90),
                      3: (2.70, -2.65)}, PERSON_POSES)

def test_each_random_cylinder_has_a_1300mm_side_passage(self):
    for candidates in CYLINDER_POSES.values():
        for x, _y in candidates:
            self.assertGreaterEqual(
                physical_side_clearance(x, 0.10, FIELD_BOUNDS), 1.30)

def test_fixed_obstacles_touch_the_required_edges(self):
    obstacle_1 = FIXED_OBSTACLES[0]
    obstacle_3 = FIXED_OBSTACLES[2]
    obstacle_4 = FIXED_OBSTACLES[3]
    self.assertAlmostEqual(0.65, obstacle_1[2] + obstacle_1[5] / 2.0)
    self.assertAlmostEqual(-3.35, obstacle_3[2] - obstacle_3[5] / 2.0)
    self.assertAlmostEqual(-3.35, obstacle_4[2] - obstacle_4[5] / 2.0)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```text
C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe -m unittest test.test_world_generator -v
```

Expected: import failure for `FIELD_BOUNDS`/`physical_side_clearance`, followed by coordinate assertion failures once the API exists.

- [ ] **Step 3: Implement the metric constants and clearance helper**

Replace the placement constants in `world_generator.py` with:

```python
FIELD_BOUNDS = (-0.65, 3.35, -3.35, 0.65, 3.0)

CYLINDER_POSES = {
    1: ((0.70, -1.45), (2.10, -1.45)),
    2: ((0.70, -2.45), (2.10, -1.95)),
}
PERSON_POSES = {
    1: (2.70, -1.10),
    2: (2.70, -1.90),
    3: (2.70, -2.65),
}
HAZARD_POSES = {
    1: (1.40, 0.00),
    2: (1.40, -0.45),
}
FIXED_OBSTACLES = (
    ('fixed_obstacle_1', 0.70, -0.20, 0.0, 0.10, 1.70),
    ('fixed_obstacle_2', 2.72, 0.04, 0.785398, 1.60, 0.10),
    ('fixed_obstacle_3', 0.70, -3.10, 0.0, 0.10, 0.50),
    ('fixed_obstacle_4', 2.10, -3.10, 0.0, 0.10, 0.50),
)


def physical_side_clearance(x, radius, bounds=FIELD_BOUNDS):
    left, right = bounds[0], bounds[1]
    return max((x - radius) - left, right - (x + radius))
```

Change `build_scenario` to derive its bounds from `FIELD_BOUNDS`:

```python
bounds=(FIELD_BOUNDS[1] - FIELD_BOUNDS[0],
        FIELD_BOUNDS[3] - FIELD_BOUNDS[2], FIELD_BOUNDS[4]),
```

- [ ] **Step 4: Run focused and complete tests and verify GREEN**

Run:

```text
C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe -m unittest test.test_world_generator -v
C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe -m unittest discover -s test -p test_*.py -v
```

Expected: all world tests and the complete suite pass.

- [ ] **Step 5: Commit the geometry correction**

```text
git add src/firefighting_mission/world_generator.py test/test_world_generator.py
git commit -m "fix: align firefighting field geometry"
```

---

### Task 2: Render Recognition Images and Zone Borders

**Files:**
- Modify: `src/firefighting_mission/world_generator.py:83-118`
- Modify: `models/targets/materials/scripts/targets.material`
- Create: `models/targets/materials/textures/flammable.png`
- Create: `models/targets/materials/textures/explosive.png`
- Create: `models/targets/materials/textures/toxic.png`
- Create: `models/targets/materials/textures/distractor.png`
- Create: `models/targets/materials/textures/person.png`
- Modify: `scripts/start_sitl.sh:7-14`
- Modify: `CMakeLists.txt`
- Modify: `test/test_world_generator.py`

**Interfaces:**
- Consumes: seeded `hazard_symbol`, `hazard_index`, and `person_position`.
- Produces: `_zone_model(name, x, y, border_color, material_name, metadata)` with an outer border and upward-facing textured plane.

- [ ] **Step 1: Write failing material and generated-SDF tests**

Add to `test/test_world_generator.py`:

```python
def test_task_zones_reference_rendered_recognition_materials(self):
    output = os.path.join(writable_tempdir(), 'textured.world')
    scenario = generate_world(4501, output)
    root = ET.parse(output).getroot()
    names = [node.text for node in root.findall('.//material/script/name')]
    expected_hazard = 'FireTargets/%s' % scenario.hazard_symbol.title()
    self.assertIn(expected_hazard, names)
    self.assertIn('FireTargets/Distractor', names)
    self.assertIn('FireTargets/Person', names)
    for model_name in ('hazard_zone_1', 'hazard_zone_2', 'person_zone'):
        model = root.find(".//model[@name='%s']" % model_name)
        self.assertEqual(2, len(model.findall('.//visual')))

def test_every_perception_template_is_packaged_for_gazebo(self):
    labels = ('flammable', 'explosive', 'toxic', 'distractor', 'person')
    for label in labels:
        source = os.path.join(PROJECT_ROOT, 'assets', 'templates', label + '.png')
        texture = os.path.join(PROJECT_ROOT, 'models', 'targets',
                               'materials', 'textures', label + '.png')
        self.assertTrue(os.path.isfile(texture))
        with open(source, 'rb') as source_file:
            source_bytes = source_file.read()
        with open(texture, 'rb') as texture_file:
            self.assertEqual(source_bytes, texture_file.read())

def test_sitl_exports_package_models_to_gazebo(self):
    wrapper = os.path.join(PROJECT_ROOT, 'scripts', 'start_sitl.sh')
    with open(wrapper, 'r') as handle:
        text = handle.read()
    self.assertIn('$package_root/models', text)
    self.assertIn('GAZEBO_MODEL_PATH', text)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run the focused world suite. Expected: missing texture files, missing material references, and missing package model path assertions fail.

- [ ] **Step 3: Package the existing PNGs**

Create `models/targets/materials/textures` and copy the five PNG files byte-for-byte from `assets/templates`. Do not regenerate or rescale them.

```powershell
New-Item -ItemType Directory -Force models\targets\materials\textures
Copy-Item assets\templates\flammable.png models\targets\materials\textures\flammable.png
Copy-Item assets\templates\explosive.png models\targets\materials\textures\explosive.png
Copy-Item assets\templates\toxic.png models\targets\materials\textures\toxic.png
Copy-Item assets\templates\distractor.png models\targets\materials\textures\distractor.png
Copy-Item assets\templates\person.png models\targets\materials\textures\person.png
```

- [ ] **Step 4: Define all Gazebo texture materials**

Replace `targets.material` with five entries following this complete pattern:

```text
material FireTargets/Flammable
{
  technique { pass { lighting off texture_unit { texture flammable.png } } }
}
material FireTargets/Explosive
{
  technique { pass { lighting off texture_unit { texture explosive.png } } }
}
material FireTargets/Toxic
{
  technique { pass { lighting off texture_unit { texture toxic.png } } }
}
material FireTargets/Distractor
{
  technique { pass { lighting off texture_unit { texture distractor.png } } }
}
material FireTargets/Person
{
  technique { pass { lighting off texture_unit { texture person.png } } }
}
```

- [ ] **Step 5: Render bordered textured task zones**

Change `_zone_model` so each model has a 0.40 m coloured border box and a 0.34 m upward-facing texture plane:

```python
def _zone_model(name, x, y, border_color, material_name, metadata):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.006 0 0 0</pose>
      <link name="link">
        <visual name="border">
          <geometry><box><size>0.4 0.4 0.01</size></box></geometry>
          <material><ambient>{border}</ambient><diffuse>{border}</diffuse></material>
        </visual>
        <visual name="target_image">
          <pose>0 0 0.006 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>0.34 0.34</size></plane></geometry>
          <material><script>
            <uri>model://targets/materials/scripts</uri>
            <uri>model://targets/materials/textures</uri>
            <name>{material}</name>
          </script></material>
        </visual>
      </link>
      <!-- target_class: {metadata} -->
    </model>'''.format(name=name, x=x, y=y, border=border_color,
                       material=material_name, metadata=metadata)
```

Call it with red (`0.9 0.05 0.05 1`) hazard borders, cyan (`0.05 0.75 0.95 1`) person borders, `FireTargets/<TitleCaseClass>`, and the existing seeded class metadata.

- [ ] **Step 6: Export and install package-owned assets**

In `start_sitl.sh`, after computing `package_root`, prepend the package model path:

```bash
export GAZEBO_MODEL_PATH="$package_root/models:${GAZEBO_MODEL_PATH:-}"
```

In `CMakeLists.txt`, install runtime resources:

```cmake
install(DIRECTORY launch models assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Replace the existing launch-only install block rather than installing `launch` twice.

- [ ] **Step 7: Run focused/full tests and XML validation**

Run the world suite, complete suite, and parse a generated seed-4501 world with `xml.etree.ElementTree`. Expected: all pass and every referenced material name is present.

- [ ] **Step 8: Commit the textured zones**

```text
git add CMakeLists.txt scripts/start_sitl.sh src/firefighting_mission/world_generator.py test/test_world_generator.py models/targets
git commit -m "feat: render firefighting task markers"
```

---

### Task 3: Add the Physical Safety Enclosure

**Files:**
- Modify: `src/firefighting_mission/world_generator.py`
- Modify: `test/test_world_generator.py`
- Test: `models/fire_iris/fire_iris.sdf`

**Interfaces:**
- Produces: `_safety_net_model(name, x, y, size)` and four generated models named `safety_net_north`, `south`, `west`, and `east`.

- [ ] **Step 1: Write failing enclosure and payload-limit tests**

```python
def test_field_has_four_three_metre_physical_safety_nets(self):
    output = os.path.join(writable_tempdir(), 'enclosed.world')
    generate_world(4501, output)
    root = ET.parse(output).getroot()
    expected = {
        'safety_net_north': ((1.35, 0.65, 1.50), (4.00, 0.02, 3.00)),
        'safety_net_south': ((1.35, -3.35, 1.50), (4.00, 0.02, 3.00)),
        'safety_net_west': ((-0.65, -1.35, 1.50), (0.02, 4.00, 3.00)),
        'safety_net_east': ((3.35, -1.35, 1.50), (0.02, 4.00, 3.00)),
    }
    for name, (pose, size) in expected.items():
        model = root.find(".//model[@name='%s']" % name)
        self.assertIsNotNone(model)
        self.assertEqual(pose, numbers(model.find('pose').text))
        self.assertEqual(size, numbers(model.find('.//collision/geometry/box/size').text))
        self.assertGreaterEqual(float(model.find('.//visual/transparency').text), 0.70)

def test_payload_cubes_remain_within_eighty_millimetres(self):
    sdf = os.path.join(PROJECT_ROOT, 'models', 'fire_iris', 'fire_iris.sdf')
    root = ET.parse(sdf).getroot()
    for link_name in ('fire_payload_link', 'rescue_payload_link'):
        link = root.find(".//link[@name='%s']" % link_name)
        size = numbers(link.find('.//collision/geometry/box/size').text)
        self.assertLessEqual(max(size), 0.08)
```

- [ ] **Step 2: Run focused tests and verify RED**

Expected: all four safety-net model lookups fail; the existing payload-size regression passes.

- [ ] **Step 3: Implement the enclosure renderer**

```python
def _safety_net_model(name, x, y, size):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 1.5000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.25 0.45 0.70 0.20</ambient><diffuse>0.25 0.45 0.70 0.20</diffuse></material>
          <transparency>0.80</transparency>
        </visual>
      </link>
    </model>'''.format(name=name, x=x, y=y, size=size)
```

Append exactly these four calls in `render_world`:

```python
models.append(_safety_net_model('safety_net_north', 1.35, 0.65, '4.0 0.02 3.0'))
models.append(_safety_net_model('safety_net_south', 1.35, -3.35, '4.0 0.02 3.0'))
models.append(_safety_net_model('safety_net_west', -0.65, -1.35, '0.02 4.0 3.0'))
models.append(_safety_net_model('safety_net_east', 3.35, -1.35, '0.02 4.0 3.0'))
```

- [ ] **Step 4: Run focused and complete tests and verify GREEN**

Run the world suite and full test discovery. Expected: all pass with no XML parse errors.

- [ ] **Step 5: Commit the enclosure**

```text
git add src/firefighting_mission/world_generator.py test/test_world_generator.py
git commit -m "feat: add firefighting field safety enclosure"
```

---

### Task 4: Validate in the VM and Present the Corrected Field

**Files:**
- Regenerate: `/home/ss/catkin_ws/src/firefighting_mission/artifacts/4501/firefighting.world`
- Inspect: `/tmp/firefighting_view.log`

**Interfaces:**
- Consumes: the completed mission package from Tasks 1-3.
- Produces: a running Gazebo GUI field for user visual acceptance; no autonomous mission run.

- [ ] **Step 1: Run final host verification**

```text
C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe -m unittest discover -s test -p test_*.py -v
git diff --check
```

Expected: zero failures and no whitespace errors.

- [ ] **Step 2: Stop only the currently verified visualization launch**

Read the active process list over SSH, identify the `mavros_posix_sitl.launch` process whose world path is the mission package's seed-4501 artifact, and send SIGINT to that exact process group. Do not kill unrelated ROS, Gazebo, PX4, or user processes.

- [ ] **Step 3: Sync only the mission package**

Copy the tracked package files into `/home/ss/catkin_ws/src/firefighting_mission`, excluding `.git`, `.superpowers`, local test scratch, worktrees, and prior artifacts. Do not write anywhere under PX4_Firmware or XTDrone.

- [ ] **Step 4: Run VM unit/build verification**

In the VM's sourced ROS Melodic shell:

```text
cd /home/ss/catkin_ws/src/firefighting_mission
python -m unittest discover -s test -p 'test_*.py' -v
cd /home/ss/catkin_ws
catkin_make
```

Expected: the Python 2.7 suite passes and `catkin_make` exits zero.

- [ ] **Step 5: Regenerate and launch GUI-only seed 4501**

Set `DISPLAY=:0`, unset stale `ROS_HOSTNAME`, set loopback `ROS_IP`/`ROS_MASTER_URI`, and run only:

```text
/home/ss/catkin_ws/src/firefighting_mission/scripts/start_sitl.sh 4501 \
  /home/ss/catkin_ws/src/firefighting_mission/artifacts/4501/firefighting.world true
```

Do not launch `firefighting_headless.launch` or mission nodes.

- [ ] **Step 6: Check runtime evidence**

Verify the log contains successful world generation, `gzserver`, `gzclient`, vehicle spawn, and no missing material/texture/model error. Confirm the generated world contains all four safety nets and the selected seed-4501 task materials.

- [ ] **Step 7: Hand visual control to the user**

Tell the user the corrected scene is open. Ask them to compare obstacle 2 over obstacle 4, the bottom-aligned obstacles, marker images, and enclosure from an overhead view. Leave the GUI running and do not resume autonomous mission work until they approve.

- [ ] **Step 8: Commit any verification-only documentation update if required**

If no tracked file changed during VM validation, make no empty commit. If a tracked operator note is updated with exact verified commands, commit only that note with message `docs: record corrected field verification`.
