# Firefighting Field Layout Correction Design

## Goal

Correct the Gazebo field so that its metric geometry follows the competition dimensions and its undimensioned placements visually follow Figure 1. The field must also render recognizable task images and a visible, physical safety enclosure.

## Coordinate System

- Use metres internally.
- Keep the takeoff-point centre at `(0.0, 0.0)`.
- The 4 m square field extends from `x=-0.65` to `x=3.35` and from `y=0.65` to `y=-3.35`.
- Positive `x` points right in Figure 1; negative `y` points down.
- The physical ceiling is 3 m above the floor.

## Geometry

### Takeoff area and floor

- Keep the takeoff marker at `(0.0, 0.0)` with diameter 0.50 m.
- Keep the floor at 4.0 x 4.0 m and near-matte white.

### Fixed obstacles

All fixed obstacles remain 2.0 m high and 0.10 m thick.

| Obstacle | Centre `(x, y)` | Length | Yaw | Placement intent |
|---|---:|---:|---:|---|
| 1 | `(0.70, -0.20)` | 1.70 m | 0 degrees | Top edge flush with field top |
| 2 | `(2.72, 0.04)` | 1.60 m | 45 degrees | Lower-left end above obstacle 4; upper-right end near top boundary |
| 3 | `(0.70, -3.10)` | 0.50 m | 0 degrees | Bottom edge flush with field bottom |
| 4 | `(2.10, -3.10)` | 0.50 m | 0 degrees | Bottom edge flush with field bottom |

### Random cylinders

- Keep both cylinders at diameter 0.20 m and height 2.0 m.
- Cylinder 1 uses `x=0.70`, with candidate `y` positions `-1.45` and `-2.45`.
- Cylinder 2 uses `x=2.10`, with candidate `y` positions `-1.45` and `-1.95`.
- Preserve seeded deterministic selection.
- Add a geometric assertion that each selected cylinder leaves at least 1.30 m of physical clear width on one side between its outer surface and the field perimeter. This verifies the rule without reinterpreting the drawing's separate centre/reference dimensions.

## Task Zones

### Hazard zones

- Keep two 0.40 x 0.40 m zones.
- Place their centres at `(1.40, 0.00)` and `(1.40, -0.45)`.
- Render a red outer border with a white inner image surface.
- The seeded correct zone displays one of `flammable`, `explosive`, or `toxic`; the other displays `distractor`.

### Person-rescue zone

- Keep one active 0.40 x 0.40 m zone selected deterministically from three candidates.
- Use centres `(2.70, -1.10)`, `(2.70, -1.90)`, and `(2.70, -2.65)`.
- Render a cyan outer border and the person image on the inner surface.

### Texture packaging

- Reuse the existing PNG templates in `assets/templates`.
- Package Gazebo material definitions and textures under `models/targets`.
- Add the package model directory to `GAZEBO_MODEL_PATH` in the package-owned SITL wrapper; do not modify PX4 or XTDrone.
- Ensure the downward camera sees the same rendered symbols used by template perception.

## Safety Enclosure

- Add four 3.0 m high perimeter barriers at the exact 4 m field boundaries.
- Give each barrier a thin collision surface so the vehicle cannot leave the arena.
- Use a highly transparent blue-grey visual so the enclosure reads as safety netting without obscuring task zones or camera views.
- Keep the top open; the 3 m height is the competition flight ceiling.

## Payloads

- Keep both payloads at 0.06 m cubes, within the 0.08 m maximum dimension.
- No payload geometry change is part of this field correction.

## Tests and Verification

Use test-driven development for every geometry change.

Automated tests must verify:

1. Field bounds and floor dimensions.
2. Exact fixed-obstacle centres, dimensions, yaw, and bottom/top alignment.
3. Exact random-cylinder candidate coordinates, diameter, and height.
4. Seed reproducibility and candidate coverage.
5. Hazard and person zone dimensions and candidate coordinates.
6. Material-script/texture references for every task class.
7. Four perimeter barriers, 3 m height, collision geometry, and transparency.
8. Both payload cubes remain no larger than 0.08 m.

VM verification must include:

- Python 2.7 unit tests.
- `catkin_make`.
- Regeneration of seed 4501.
- Gazebo GUI relaunch with no missing-material or missing-model errors.
- Visual comparison from an overhead view against Figure 1 before autonomous mission work resumes.

## Scope and Safety

- Change only the `firefighting_mission` package and its tests/assets.
- Do not edit PX4_Firmware or XTDrone.
- Preserve deterministic seed behaviour and existing mission topic interfaces.
- Do not run the full autonomous mission during this correction; stop after the user visually accepts the corrected field.
