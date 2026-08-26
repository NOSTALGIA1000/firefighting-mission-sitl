# Temporary Visual Obstacle Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent post-avoidance route segments from crossing visually detected random cylinders.

**Architecture:** Store each selected stereo cluster as a world-frame circle. Extend fixed-map A* occupancy checks with optional circles, then replan from `REJOIN` using stored circles. Keep reactive vision active for new obstacles and hold safely when no dynamic route exists.

**Tech Stack:** Python 2.7, ROS Melodic, MAVROS, PX4 SITL, Gazebo 9, `unittest`.

## Global Constraints

- Flight altitude remains 1.20 m; accepted integration band is 1.10–1.30 m.
- Runtime obstacle input remains stereo/depth only; no seeded coordinates, Gazebo model state, UWB, motion capture, or external positioning aid.
- Temporary circles live only for one planner process and are never removed during one mission.
- Existing two-argument route-provider test doubles remain compatible.
- Dynamic replanning failure commands `HOLD_UNSAFE`; it never resumes an unsafe old route.

---

### Task 1: Dynamic-Circle Occupancy in Field Map

**Files:**
- Modify: `src/firefighting_mission/field_map.py`
- Test: `test/test_field_map.py`

**Interfaces:**
- Consumes: circle tuple `(center_x, center_y, radius_m)`.
- Produces: `point_is_free(point, inflation=0.45, dynamic_circles=())` and `plan_route(start, goal, resolution=0.10, inflation=0.45, dynamic_circles=())`.

- [ ] **Step 1: Write failing occupancy test**

```python
def test_dynamic_circle_blocks_inflated_points(self):
    circle = ((0.70, -1.45, 0.10),)
    self.assertFalse(point_is_free((0.20, -1.45), 0.45, circle))
    self.assertTrue(point_is_free((-0.40, -1.45), 0.45, circle))
```

- [ ] **Step 2: Run RED test**

Run: `python test/test_field_map.py FieldMapTest.test_dynamic_circle_blocks_inflated_points`

Expected: `TypeError` because `point_is_free` lacks `dynamic_circles`.

- [ ] **Step 3: Implement circle occupancy and propagate through A***

```python
def _inside_dynamic_circle(point, circle, inflation):
    center_x, center_y, radius = circle
    return math.hypot(point[0] - center_x,
                      point[1] - center_y) <= radius + inflation

def point_is_free(point, inflation=0.45, dynamic_circles=()):
    if not _inside_field(point, inflation):
        return False
    if any(_inside_dynamic_circle(point, value, inflation)
           for value in dynamic_circles):
        return False
    return not any(_inside_board(point, obstacle, inflation)
                   for obstacle in FIXED_OBSTACLES)
```

Add `dynamic_circles=()` to `plan_route`; pass it into every `point_is_free` call inside `valid`, start validation, and goal validation.

- [ ] **Step 4: Add route-detour test and run GREEN suite**

```python
def test_route_avoids_dynamic_circle(self):
    circle = ((0.70, -1.45, 0.10),)
    route = plan_route((0.10, -1.90), (1.50, -1.45),
                       dynamic_circles=circle)
    self.assertTrue(all(math.hypot(x - 0.70, y + 1.45) > 0.54
                        for x, y in route))
```

Run: `python test/test_field_map.py`

Expected: all field-map tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/firefighting_mission/field_map.py test/test_field_map.py
git commit -m "feat: route around temporary visual obstacles"
```

### Task 2: Visual Circle Memory and Safe Replanning

**Files:**
- Modify: `src/firefighting_mission/path_planner.py`
- Test: `test/test_path_planner.py`

**Interfaces:**
- Consumes: selected `ObstacleClusterData`, vehicle pose `(x, y, z, yaw)`, dynamic route provider `(start, goal, circles) -> route`.
- Produces: `temporary_obstacles` tuple of world circles and `HOLD_UNSAFE/dynamic_route_unreachable` on replan failure.

- [ ] **Step 1: Write failing world-transform and merge tests**

```python
def test_selected_cluster_is_remembered_as_world_circle(self):
    planner = planner_with_goal()
    planner.clearance_override = (1.30, 1.00)
    drive_to_select(planner)
    planner.update(POSE, (OBSTACLE,), True, 1.5)
    self.assertEqual(1, len(planner.temporary_obstacles))
    self.assertAlmostEqual(0.80, planner.temporary_obstacles[0][0], places=6)

def test_duplicate_visual_observations_merge(self):
    planner = planner_with_goal()
    planner._remember_obstacle((0.70, -1.45, 0.10))
    planner._remember_obstacle((0.74, -1.43, 0.11))
    self.assertEqual(1, len(planner.temporary_obstacles))
```

- [ ] **Step 2: Run RED tests**

Run: `python test/test_path_planner.py VisualPathPlannerTest.test_selected_cluster_is_remembered_as_world_circle VisualPathPlannerTest.test_duplicate_visual_observations_merge`

Expected: failure because `temporary_obstacles` and `_remember_obstacle` do not exist.

- [ ] **Step 3: Implement visual memory**

Initialize `self.temporary_obstacles = ()`. During successful `_select_side`, transform adjusted cluster center with `_body_to_world`, estimate radius as `max(0.10, (left_edge_m - right_edge_m) / 2.0)`, and merge overlapping observations by retaining weighted center and maximum radius.

```python
def _remember_obstacle(self, circle):
    circles = list(self.temporary_obstacles)
    for index, current in enumerate(circles):
        if math.hypot(circle[0] - current[0],
                      circle[1] - current[1]) <= max(circle[2], current[2]):
            circles[index] = ((circle[0] + current[0]) / 2.0,
                              (circle[1] + current[1]) / 2.0,
                              max(circle[2], current[2]))
            self.temporary_obstacles = tuple(circles)
            return
    circles.append(circle)
    self.temporary_obstacles = tuple(circles)
```

- [ ] **Step 4: Write failing REJOIN dynamic-provider tests**

```python
def test_rejoin_replans_with_temporary_obstacles(self):
    calls = []
    def dynamic_route(start, goal, circles):
        calls.append((start, goal, circles))
        return (tuple(start), tuple(goal))
    planner = planner_with_goal(dynamic_route_provider=dynamic_route)
    # Drive BRAKE through REJOIN using returned command targets.
    self.assertEqual(planner.temporary_obstacles, calls[-1][2])

def test_dynamic_replan_failure_holds(self):
    def unreachable(start, goal, circles):
        raise ValueError('route_unreachable')
    planner = planner_with_goal(dynamic_route_provider=unreachable)
    # Drive to completed REJOIN.
    command = planner.update(at_rejoin, (), True, 1.8)
    self.assertEqual('HOLD_UNSAFE', command.state)
    self.assertEqual('dynamic_route_unreachable', command.reason)
```

- [ ] **Step 5: Implement dynamic route provider and REJOIN behavior**

Keep existing `route_provider(start, goal)`. Add optional `dynamic_route_provider`; default it to a wrapper calling `plan_route(start, goal, dynamic_circles=circles)`. On completed `REJOIN`, call dynamic provider using current pose, goal, and stored circles. Replace route and waypoint index only on success. On `ValueError`, hold current position with `dynamic_route_unreachable`.

- [ ] **Step 6: Run GREEN planner suite**

Run: `python test/test_path_planner.py`

Expected: all planner tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/firefighting_mission/path_planner.py test/test_path_planner.py
git commit -m "feat: remember visual cylinders during replanning"
```

### Task 3: SITL Verification and Handoff

**Files:**
- Modify: `docs/CURRENT_HANDOFF.zh-CN.md`
- Modify: `docs/TEAM_C_HANDOFF.zh-CN.md`
- Evidence: `artifacts/avoidance_matrix/*/smoke.json` (generated, commit only if already tracked by repository policy)

**Interfaces:**
- Consumes: unit-tested field map and planner.
- Produces: four-seed safety evidence and updated teammate instructions.

- [ ] **Step 1: Sync changed source and tests into VM**

Copy exact files to `/home/ss/catkin_ws/src/firefighting_mission`, preserving package-relative paths.

- [ ] **Step 2: Run VM unit suites**

```bash
python test/test_field_map.py
python test/test_path_planner.py
python test/test_stereo_obstacles.py
```

Expected: zero failures.

- [ ] **Step 3: Run seed 1 twice**

```bash
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
rostest firefighting_mission visual_avoidance_smoke.test seed:=1
```

Expected each run: `RESULT: SUCCESS`, collision false, altitude within 1.10–1.30 m.

- [ ] **Step 4: Run four-layout matrix**

```bash
rosrun firefighting_mission run_avoidance_matrix.sh
```

Expected: seeds `1 4 10 2` all pass.

- [ ] **Step 5: Update handoff documents**

Document temporary visual obstacle memory, dynamic A* replan, exact commands, evidence paths, remaining hardware limitation, and stereo calibration requirement.

- [ ] **Step 6: Verify diff and commit**

```bash
git diff --check
git status --short
git add docs/CURRENT_HANDOFF.zh-CN.md docs/TEAM_C_HANDOFF.zh-CN.md
git commit -m "docs: hand off dynamic visual replanning"
```

- [ ] **Step 7: Push only after complete verification**

```bash
git push origin feature/firefighting-sitl
```

Expected: remote branch advances and contains only verified code plus handoff documentation.
