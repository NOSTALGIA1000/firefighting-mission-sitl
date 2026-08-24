# Team C Path Planning and Supply Drop Design

## Goal

Complete Team C's two deliverables:

1. Deterministic point-to-point movement using climb, cruise, and descend stages.
2. Dual-channel Gazebo payload release through ROS services.

The design must integrate with the existing Team A MAVROS controller without
creating multiple publishers that compete for flight control.

## Scope

### Included

- `path_planner.py` pure planning logic.
- ROS adapter for path requests, staged setpoints, and planner status.
- Team A setpoint adapter so only `competition_main.py` publishes MAVROS local
  position setpoints.
- `supply_drop.py` high-level, safety-gated dual-channel drop service.
- Gazebo model-plugin service that opens the matching door, detaches the fixed
  payload joint, and enables gravity.
- Backward compatibility with existing drop topics.
- Unit, ROS contract, Gazebo physics, and VM flight verification.

### Excluded

- Dynamic global path search.
- Mapping or SLAM.
- Vision-based target selection.
- Full competition mission sequencing.
- Real 450-frame hardware validation.

## Existing System Constraints

- ROS Melodic and Python 2.7 compatibility remain required.
- PX4 SITL uses MAVROS OFFBOARD position setpoints from
  `scripts/competition_main.py`.
- Fixed obstacles are 2.0 m high; field ceiling is 3.0 m.
- Payload channels remain `1` for firefighting material and `2` for rescue
  material.
- Existing topic-based drop triggers remain supported.

## Architecture

### Flight command ownership

`competition_main.py` remains the only publisher to
`/mavros/setpoint_position/local`. Team C planner publishes staged targets on an
internal mission topic. Team A adapter consumes those targets after takeoff and
forwards them through its existing OFFBOARD stream.

This prevents two nodes from sending conflicting MAVROS setpoints.

### Path-planning flow

Input:

- Current local pose.
- Requested destination `(x, y, z)`.

Configuration:

- Safe cruise altitude: `2.30 m`.
- Horizontal reach tolerance: `0.12 m`.
- Vertical reach tolerance: `0.08 m`.

Stages:

1. `CLIMB`: hold current `x/y`, command `z=2.30`.
2. `CRUISE`: command destination `x/y`, hold `z=2.30`.
3. `DESCEND`: hold destination `x/y`, command requested destination `z`.
4. `REACHED`: hold final destination.

No horizontal target change occurs during `CLIMB`. No descent target occurs
before destination horizontal tolerance is met. A new destination resets the
planner from current pose.

ROS interface:

- Input goal: `/fire_mission/point_goal` (`geometry_msgs/PoseStamped`).
- Input pose: MAVROS local pose.
- Output staged target: `/fire_mission/path_setpoint`
  (`geometry_msgs/PoseStamped`).
- Output status: `/fire_mission/path_status` (`std_msgs/String`).

### Supply-drop flow

High-level service:

- Name: `/fire_mission/drop_supply`.
- Type: `firefighting_mission/DropSupply`.
- Request: `uint8 channel`.
- Response: `bool success`, `string reason`.

Validation before release:

- Channel is `1` or `2`.
- Aircraft pose and velocity are available.
- Mission alignment is true.
- Horizontal speed is at most `0.10 m/s`.
- Altitude is within `1.15–1.45 m`.
- Selected channel has not already been released.

Low-level Gazebo service:

- Name: `/fire_iris/drop_supply`.
- Same `DropSupply` service type.
- Channel `1` opens fire door and detaches `fire_payload_joint`.
- Channel `2` opens rescue door and detaches `rescue_payload_joint`.
- Detached payload link has gravity enabled.
- Repeated release returns `success=false`, `reason=already_released`.

Existing `/fire_iris/drop_fire` and `/fire_iris/drop_rescue` Bool topics remain
accepted by the plugin for compatibility.

## Files

- Create `src/firefighting_mission/path_planner.py`: pure staged planner.
- Create `scripts/path_planner.py`: ROS planner adapter.
- Create `src/firefighting_mission/supply_drop.py`: service validation policy.
- Create `scripts/supply_drop.py`: high-level ROS service node.
- Create `srv/DropSupply.srv`: dual-channel service contract.
- Modify `scripts/competition_main.py`: consume internal staged targets.
- Modify `src/payload_plugin.cpp`: expose low-level Gazebo service.
- Modify `include/firefighting_mission/payload_plugin.hpp`: service callback and
  server declarations.
- Modify `CMakeLists.txt` and `package.xml`: service generation, installation,
  and tests.
- Add focused unit and ROS/Gazebo tests.

## Error Handling

- Missing pose or velocity: `flight_state_missing`.
- Invalid channel: `invalid_channel`.
- Repeated channel: `already_released`.
- Missing alignment: `not_aligned`.
- Excess speed: `moving_too_fast`.
- Invalid altitude: `altitude_out_of_range`.
- Gazebo service unavailable or rejected: response reports low-level failure;
  high-level channel remains eligible for retry.
- Missing model joints or links: plugin logs exact missing-model error and does
  not advertise a usable release path.

## Verification

### Unit tests

- Planner commands climb target first.
- Planner does not move horizontally before safe altitude.
- Planner cruises at `2.30 m`.
- Planner descends only after horizontal arrival.
- Planner reaches and holds final target.
- New goal resets stages from current pose.
- Supply policy accepts each valid channel once.
- Every validation failure returns exact reason.

### ROS and Gazebo tests

- Planner topics produce ordered `CLIMB`, `CRUISE`, `DESCEND`, `REACHED`
  targets.
- Low-level service drops selected cube while other cube stays attached.
- Duplicate service call is rejected.
- Legacy Bool topic still releases its channel.

### VM acceptance

- `python -m unittest discover -s test -p 'test_*.py'` passes under Python 2.7.
- `catkin_make` passes.
- Native Iris performs climb, horizontal cruise, and descent in the competition
  world without hitting 2.0 m obstacles.
- Payload fixture visibly releases requested cube through ROS service.

## Completion Boundary

Team C is complete when both VM acceptance demonstrations pass and all related
code is committed and pushed to `feature/firefighting-sitl`. Full autonomous
competition sequencing remains separate follow-up work.
