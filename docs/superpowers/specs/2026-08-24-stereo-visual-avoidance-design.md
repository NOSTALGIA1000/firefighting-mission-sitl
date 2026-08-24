# Stereo Visual Avoidance at Constant Altitude

## Goal

Replace the 2.30 m obstacle-overflight planner with a competition-oriented
planner that keeps the aircraft near 1.20 m, uses the known fixed field layout
for global routing, and uses a forward stereo camera to locate and pass the two
random cylinders through the available side corridor.

The design must work before the final stereo camera model and output format are
known. It must also preserve single ownership of MAVROS OFFBOARD setpoints.

## Competition Constraints

- Field volume is approximately 4.0 x 4.0 x 3.0 m.
- Four fixed boards are known before flight and are 2.0 m high.
- Two random cylinders are 0.20 m in diameter and 2.0 m high.
- Each random cylinder has two possible positions, selected on site.
- At least one side of each random cylinder provides a passage at least 1.30 m
  wide.
- Aircraft rotor-axis diagonal is 0.30--0.40 m.
- External UWB and motion-capture positioning are forbidden.
- Mission flight must be autonomous after one-key start, with manual takeover
  retained for abnormal conditions.

## Selected Approach

Use a hybrid planner:

1. A global route follows the measured field layout and avoids all fixed boards.
2. A stereo visual local planner detects unexpected occupancy caused by either
   random cylinder.
3. The local planner selects the safer side passage, temporarily leaves the
   global route, passes the cylinder, and rejoins the route.
4. Altitude control remains independent and holds 1.20 m throughout transit.

This avoids the setup and tuning cost of full stereo SLAM while still making
random-obstacle decisions from onboard vision rather than hard-coded cylinder
positions.

## Sensor Compatibility Layer

The final camera model is unknown, so perception begins behind one normalized
obstacle-depth interface.

Supported inputs, in priority order:

1. Registered depth image plus `CameraInfo`.
2. Organized or unorganized `PointCloud2`.
3. Calibrated left/right images plus both `CameraInfo` topics. ROS
   `stereo_image_proc` produces disparity/depth or a point cloud before the
   normalized interface.

Raw uncalibrated stereo images are not accepted for autonomous flight. Missing
intrinsics, baseline, synchronization, or rectification data produces a
configuration fault and prevents takeoff.

Simulation will expose the same normalized interface as the real camera. No
planner behavior may depend on Gazebo-only ground-truth model positions.

## Visual Obstacle Extraction

Per frame:

1. Reject invalid, stale, and out-of-range depth samples.
2. Crop to the forward flight-height band so the floor, ceiling, rotors, and
   payload do not create false obstacles.
3. Transform valid points into the aircraft body frame.
4. Cluster connected foreground points in the horizontal plane.
5. Estimate each cluster's center, nearest range, lateral span, and confidence.
6. Track clusters over several frames to reject single-frame noise.

The planner does not need semantic classification to avoid collision. A
geometry-consistent vertical cluster is treated as an obstacle. Cylinder-like
clusters may be labelled for recording, but unknown clusters receive the same
safety treatment.

The annotated return image shows cluster box, estimated distance, chosen side,
free-corridor width, and avoidance state. This provides visible evidence of
autonomous visual avoidance in the submitted screen recording.

## Map and Clearance Model

The existing field coordinates and dimensions remain the global-map source.
Fixed boards and field boundaries are inflated in the horizontal plane before
route generation.

Initial configurable geometry:

- Maximum aircraft diameter: 0.40 m.
- Desired obstacle clearance outside the aircraft envelope: 0.25 m per side.
- Minimum accepted raw corridor width: 0.90 m.
- Competition-provided corridor: at least 1.30 m.

Thus the guaranteed passage exceeds the initial minimum by 0.40 m. Final
clearance values must be measured against the built aircraft, stereo blind
zone, propeller guards, and flight tracking error.

Random-cylinder positions are never read from the generated world or scenario
configuration at runtime. Vision-derived occupancy is fused with the fixed
map. Field boundaries participate in left/right clearance scoring, preventing
the planner from choosing an apparently open direction outside the arena.

## Global Route

The global route contains mission waypoints for:

1. Takeoff and 1.20 m hover.
2. Hazard observation/drop region.
3. Person observation/drop region.
4. Return and landing approach.

Route segments are generated around known fixed boards with the inflated map.
The exact hazard and person goal is selected from the competition draw, while
the fixed-board avoidance geometry remains unchanged.

The global planner supplies a route corridor, not only a straight destination.
This gives the local planner a defined direction to rejoin after passing a
random cylinder.

## Local Avoidance State Machine

States:

1. `FOLLOW_ROUTE`: track the next global waypoint at 1.20 m.
2. `BRAKE`: when a stable obstacle enters the forward trigger range, reduce
   horizontal speed while holding altitude and yaw.
3. `OBSERVE`: hover and gather multiple depth frames; if needed, perform a
   bounded yaw scan to expose both side corridors.
4. `SELECT_SIDE`: compare left and right corridor widths after accounting for
   the obstacle, fixed map, and field boundary. Prefer the wider valid corridor.
5. `SIDESTEP`: move laterally toward a clearance waypoint at reduced speed.
6. `PASS`: move forward past the tracked obstacle while maintaining lateral
   clearance.
7. `REJOIN`: return smoothly to the global route corridor.
8. `HOLD_UNSAFE`: hover if neither side meets minimum clearance or perception
   becomes unreliable.

A new obstacle during `SIDESTEP`, `PASS`, or `REJOIN` returns control to
`BRAKE`. Side selection uses hysteresis, so noisy frame-to-frame width changes
cannot make the aircraft oscillate between left and right.

Initial tunable limits:

- Transit altitude: 1.20 m.
- Altitude tolerance: +/-0.10 m.
- Forward avoidance trigger: approximately 1.00 m.
- Avoidance horizontal speed: at most 0.30 m/s.
- Normal route speed: at most 0.45 m/s until flight tests justify an increase.
- Side decision: several consecutive synchronized stereo frames.

## Flight-Control Integration

`competition_main.py` remains the only node publishing MAVROS local-position
setpoints. Global and local planners publish internal short-horizon position
targets and status; the controller forwards the selected target at the required
OFFBOARD rate.

Altitude target is generated separately from horizontal planning and remains
1.20 m during `FOLLOW_ROUTE` through `REJOIN`. Vertical motion is allowed only
for takeoff, task-specific fine adjustment explicitly requested by the mission,
landing, or a safety action.

The existing 2.30 m staged planner is removed from the active competition
launch path. It may remain temporarily as a legacy demo until migration tests
pass, but must not compete for command ownership.

## Safety and Failure Handling

- Depth/point-cloud age exceeds short timeout: stop horizontal motion and hold.
- Data remains unavailable: report fault and wait for manual takeover or invoke
  configured controlled landing policy.
- Stereo configuration invalid before launch: block autonomous start.
- Neither side has required clearance: `HOLD_UNSAFE`; never force passage.
- Altitude exits tolerance: prioritize altitude recovery while braking
  horizontal motion.
- Pose, attitude, or MAVROS connection becomes stale: existing safety monitor
  retains authority to hold or land.
- Collision proximity crosses emergency threshold: retreat along the last safe
  motion vector, then hold.

All abnormal transitions publish a machine-readable reason and appear in the
recorded overlay.

## ROS Boundaries

Planned logical interfaces:

- Stereo adapter input: depth image, point cloud, or calibrated stereo topics.
- Normalized local obstacles: tracked horizontal obstacle clusters with
  timestamp, range, lateral span, and confidence.
- Global route input: mission destination and current phase.
- Local planner output: short-horizon position target, avoidance state, selected
  side, and clearance.
- Controller input: current MAVROS local pose and selected planner target.
- Recorder input: annotated image, planner state, obstacle data, and MAVROS
  flight state.

Exact message definitions and topic names belong to the implementation plan.
They must support ROS Melodic and Python 2.7 where Python nodes remain in use.

## Test Strategy

### Pure logic

- Select left when only left corridor is valid.
- Select right when only right corridor is valid.
- Select wider side when both are valid.
- Hold when both are narrower than minimum.
- Side hysteresis prevents oscillation.
- State sequence completes `BRAKE` through `REJOIN`.
- Altitude target stays 1.20 m in every horizontal-navigation state.
- Stale perception produces `HOLD_UNSAFE`.

### Sensor adapter

- Registered depth and point cloud produce equivalent obstacle clusters.
- Calibrated stereo pipeline produces valid metric range.
- Invalid calibration and unsynchronized images block readiness.
- Floor and isolated depth noise do not create stable obstacles.

### Gazebo scenarios

- Each cylinder is tested at both allowed positions.
- Both-cylinder combinations are tested.
- Planner never reads scenario truth at runtime.
- Aircraft passes through available corridor without contact.
- Height remains 1.10--1.30 m during horizontal transit.
- Planner rejoins route and completes hazard, person, return phases.
- Stereo stream interruption causes hover.

### Real-aircraft progression

1. Bench calibration and static distance validation.
2. Propellers-off perception and overlay validation in a mock corridor.
3. Tethered/guarded hover with stationary obstacle.
4. Low-speed single-cylinder pass.
5. Both random cylinders and fixed-board route.
6. Full timed mission with manual takeover tested beforehand.

## Acceptance Criteria

- Autonomous transit stays near 1.20 m instead of flying over obstacles.
- Fixed boards are avoided from the known map.
- Random cylinders are located from stereo data, not scenario truth.
- Aircraft chooses a valid side based on measured free width.
- All allowed random-cylinder placements complete without contact.
- Avoidance returns to the planned mission route.
- Visual loss and blocked passages produce a safe hover.
- Recorded view clearly shows visual detection and avoidance decisions.
- Hazard recognition/drop, person recognition/drop, return, and landing remain
  reachable after planner integration.

## Completion Boundary

This feature is complete after unit, ROS, Gazebo, and guarded real-flight
acceptance pass. Camera-specific installation remains configurable through the
sensor adapter; acquiring and calibrating the final stereo camera is required
before real-flight acceptance.
