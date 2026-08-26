# Temporary Visual Obstacle Map Design

## Problem

The reactive planner can visually pass a random cylinder, then forget it while
returning to the global route. A subsequent route segment can cross the same
cylinder. Camera forward offset and minimum depth range can make the cylinder
invisible during the turn, causing a collision.

Increasing `pass_distance` alone is not safe: a value large enough for one
cylinder placement can push a detour into the south safety net for another
placement.

## Decision

Keep a mission-local map of visually confirmed circular obstacles. Convert the
selected body-frame cluster to a world-frame circle, then include that circle in
global A* replanning after `REJOIN`.

The temporary map is in-memory only. It resets with the planner process and does
not use seeded world coordinates, Gazebo model state, UWB, motion capture, or any
external positioning aid.

## Data Flow

1. Stereo adapter publishes body-frame obstacle clusters.
2. Planner selects the blocking cluster during `SELECT_SIDE`.
3. Planner converts cluster center to world coordinates using current pose.
4. Planner estimates radius from lateral cluster edges, with a 0.10 m minimum.
5. Near-duplicate circles merge instead of accumulating repeated observations.
6. After `REJOIN`, A* replans from current pose to goal using fixed boards plus
   stored circles.
7. A* rejects grid points inside `circle_radius + route_inflation`.
8. Existing reactive vision remains active for new or moved cylinders.

## Interfaces

- `field_map.point_is_free(..., dynamic_circles=())`
- `field_map.plan_route(..., dynamic_circles=())`
- `VisualPathPlanner` owns temporary circles and calls dynamic route provider
  only after a completed avoidance maneuver.
- Existing two-argument injected route providers remain compatible. Tests may
  inject a dedicated dynamic route provider to inspect stored circles.

Circle representation: `(center_x, center_y, radius_m)`.

## Safety Behavior

- Use aircraft route inflation already configured by global A*.
- Never remove a circle during one mission run; arena contains static random
  cylinders after draw placement.
- Merge observations whose centers overlap after radius allowance.
- If dynamic replanning fails, command `HOLD_UNSAFE` with
  `dynamic_route_unreachable`; never resume old route through the obstacle.
- Safety-net filtering remains separate; only outer nets may be ignored by the
  visual reactive trigger.

## Testing

Unit tests must prove:

- Dynamic circles block `point_is_free` and alter A* routes.
- Selected visual cluster becomes a world-frame circle.
- `REJOIN` passes stored circles to dynamic replanning.
- Replanning failure produces `HOLD_UNSAFE`.
- Duplicate observations merge.
- Existing path-planner, field-map, and stereo suites remain green.

Integration acceptance:

- Seed 1 smoke passes repeatedly without cylinder contact.
- Seeds `1, 4, 10, 2` pass matrix checks.
- Required states include `BRAKE`, `OBSERVE`, `SELECT_SIDE`, `SIDESTEP`,
  `PASS`, and `REJOIN`.
- Transit altitude stays within 1.10–1.30 m.
- No contact with cylinders, fixed boards, floor, or safety nets.

## Out of Scope

- SLAM, occupancy grids, DWA, learned depth models, moving-obstacle tracking,
  real-camera calibration, and hardware validation.
