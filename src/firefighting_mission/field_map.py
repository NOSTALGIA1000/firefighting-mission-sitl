from __future__ import division, print_function

import heapq
import math

from firefighting_mission.world_generator import FIELD_BOUNDS, FIXED_OBSTACLES


def _inside_field(point, inflation):
    x_value, y_value = point
    return (FIELD_BOUNDS[0] + inflation <= x_value <=
            FIELD_BOUNDS[1] - inflation and
            FIELD_BOUNDS[2] + inflation <= y_value <=
            FIELD_BOUNDS[3] - inflation)


def _inside_board(point, obstacle, inflation):
    _, center_x, center_y, yaw, length, width = obstacle
    dx = float(point[0]) - center_x
    dy = float(point[1]) - center_y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_length = cosine * dx + sine * dy
    local_width = -sine * dx + cosine * dy
    return (abs(local_length) <= length / 2.0 + inflation and
            abs(local_width) <= width / 2.0 + inflation)


def _inside_dynamic_circle(point, circle, inflation):
    center_x, center_y, radius = circle
    return math.hypot(point[0] - center_x,
                      point[1] - center_y) <= radius + inflation


def point_is_free(point, inflation=0.45, dynamic_circles=()):
    inflation = float(inflation)
    if not _inside_field(point, inflation):
        return False
    if any(_inside_dynamic_circle(point, circle, inflation)
           for circle in dynamic_circles):
        return False
    return not any(_inside_board(point, obstacle, inflation)
                   for obstacle in FIXED_OBSTACLES)


def point_matches_known_static(point, tolerance=0.18):
    """Return whether a measured surface belongs to mapped wall or field net."""
    x_value, y_value = point
    tolerance = float(tolerance)
    if (abs(x_value - FIELD_BOUNDS[0]) <= tolerance or
            abs(x_value - FIELD_BOUNDS[1]) <= tolerance or
            abs(y_value - FIELD_BOUNDS[2]) <= tolerance or
            abs(y_value - FIELD_BOUNDS[3]) <= tolerance):
        return True
    for obstacle in FIXED_OBSTACLES:
        _, center_x, center_y, yaw, length, width = obstacle
        dx = x_value - center_x
        dy = y_value - center_y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_length = cosine * dx + sine * dy
        local_width = -sine * dx + cosine * dy
        outside_length = max(0.0, abs(local_length) - length / 2.0)
        outside_width = max(0.0, abs(local_width) - width / 2.0)
        if math.hypot(outside_length, outside_width) <= tolerance:
            return True
    return False


def point_matches_field_boundary(point, tolerance=0.18):
    """Return whether a measured surface belongs to the outer safety net."""
    x_value, y_value = point
    tolerance = float(tolerance)
    return (abs(x_value - FIELD_BOUNDS[0]) <= tolerance or
            abs(x_value - FIELD_BOUNDS[1]) <= tolerance or
            abs(y_value - FIELD_BOUNDS[2]) <= tolerance or
            abs(y_value - FIELD_BOUNDS[3]) <= tolerance)


def _segment_is_free(first, second, inflation, dynamic_circles,
                     sample_step=0.025, extra=0.0):
    """Sample a segment for clearance.

    ``inflation`` may be a float or a callable returning the required
    clearance at a point, which lets the caller taper the margin along the
    route.  ``extra`` is added on top of whichever value applies.
    """
    resolve = inflation if callable(inflation) else (lambda _point: inflation)
    distance = math.hypot(second[0] - first[0], second[1] - first[1])
    sample_count = max(1, int(math.ceil(distance / float(sample_step))))
    for index in range(1, sample_count + 1):
        ratio = index / float(sample_count)
        point = (first[0] + (second[0] - first[0]) * ratio,
                 first[1] + (second[1] - first[1]) * ratio)
        if not point_is_free(point, resolve(point) + extra, dynamic_circles):
            return False
    return True


def simplify_route(points, inflation=0.45, dynamic_circles=()):
    if len(points) < 3:
        return tuple(points)
    result = [points[0]]
    anchor = 0
    final = len(points) - 1
    while anchor < final:
        candidate = final
        while candidate > anchor + 1:
            if _segment_is_free(points[anchor], points[candidate],
                                inflation, dynamic_circles, extra=0.12):
                break
            candidate -= 1
        if candidate == anchor + 1:
            candidate = final
            while candidate > anchor + 1:
                if _segment_is_free(points[anchor], points[candidate],
                                    inflation, dynamic_circles):
                    break
                candidate -= 1
        result.append(points[candidate])
        anchor = candidate
    return tuple(result)


def _heuristic(first, second):
    return math.hypot(second[0] - first[0], second[1] - first[1])


def _reconstruct(parents, node):
    result = [node]
    while node in parents:
        node = parents[node]
        result.append(node)
    result.reverse()
    return result


GOAL_INFLATION = 0.35
GOAL_RELIEF_RADIUS = 0.60


def plan_route(start, goal, resolution=0.10, inflation=0.45,
               dynamic_circles=(), goal_inflation=GOAL_INFLATION,
               goal_relief_radius=GOAL_RELIEF_RADIUS):
    """Plan a route on the fixed field map.

    Task zones are placed by lot and can sit closer to a drawn cylinder than
    the cruise inflation allows, so the required clearance tapers from
    ``inflation`` down to ``goal_inflation`` over the last
    ``goal_relief_radius`` metres.  ``goal_inflation`` still keeps the whole
    airframe outside the obstacle; only the extra cruise margin is given up,
    and only where the rules force the aircraft to hover.
    """
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    resolution = float(resolution)
    inflation = float(inflation)
    goal_inflation = min(float(goal_inflation), inflation)
    goal_relief_radius = max(0.0, float(goal_relief_radius))

    def clearance_at(point):
        if goal_relief_radius <= 0.0:
            return inflation
        distance = _heuristic(point, goal)
        if distance >= goal_relief_radius:
            return inflation
        ratio = distance / goal_relief_radius
        return goal_inflation + (inflation - goal_inflation) * ratio

    if not _inside_field(start, inflation):
        raise ValueError('start_outside_field')
    if not _inside_field(goal, goal_inflation):
        raise ValueError('goal_outside_field')
    if not point_is_free(start, inflation):
        raise ValueError('start_blocked')
    if not point_is_free(goal, goal_inflation, dynamic_circles):
        raise ValueError('goal_blocked')
    if resolution <= 0.0:
        raise ValueError('invalid_resolution')

    minimum_x = FIELD_BOUNDS[0] + goal_inflation
    minimum_y = FIELD_BOUNDS[2] + goal_inflation
    maximum_x = FIELD_BOUNDS[1] - goal_inflation
    maximum_y = FIELD_BOUNDS[3] - goal_inflation
    columns = int(round((maximum_x - minimum_x) / resolution))
    rows = int(round((maximum_y - minimum_y) / resolution))

    def grid(point):
        return (int(round((point[0] - minimum_x) / resolution)),
                int(round((point[1] - minimum_y) / resolution)))

    def world(node):
        return (round(minimum_x + node[0] * resolution, 6),
                round(minimum_y + node[1] * resolution, 6))

    def valid(node):
        if not (0 <= node[0] <= columns and 0 <= node[1] <= rows):
            return False
        point = world(node)
        return point_is_free(point, clearance_at(point), dynamic_circles)

    def nearest_valid(node, exact_point):
        if valid(node):
            return node
        candidates = []
        for radius in range(1, 6):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (node[0] + dx, node[1] + dy)
                    if valid(candidate):
                        candidates.append(candidate)
            if candidates:
                return min(candidates,
                           key=lambda value: _heuristic(world(value),
                                                        exact_point))
        return None

    start_node = nearest_valid(grid(start), start)
    goal_node = nearest_valid(grid(goal), goal)
    if start_node is None:
        raise ValueError('start_blocked')
    if goal_node is None:
        raise ValueError('goal_blocked')

    directions = ((1, 0), (0, 1), (-1, 0), (0, -1),
                  (1, 1), (-1, 1), (-1, -1), (1, -1))
    frontier = [(0.0, 0.0, start_node)]
    costs = {start_node: 0.0}
    parents = {}
    closed = set()
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal_node:
            nodes = _reconstruct(parents, current)
            points = [world(node) for node in nodes]
            points[0] = start
            points[-1] = goal
            return simplify_route(points, clearance_at, dynamic_circles)
        closed.add(current)
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if not valid(neighbor):
                continue
            if dx and dy:
                if not (valid((current[0] + dx, current[1])) and
                        valid((current[0], current[1] + dy))):
                    continue
            step_cost = resolution * (math.sqrt(2.0) if dx and dy else 1.0)
            neighbor_point = world(neighbor)
            if not point_is_free(neighbor_point,
                                 clearance_at(neighbor_point) + 0.12,
                                 dynamic_circles):
                step_cost += resolution * 2.0
            candidate_cost = current_cost + step_cost
            if candidate_cost >= costs.get(neighbor, float('inf')):
                continue
            costs[neighbor] = candidate_cost
            parents[neighbor] = current
            priority = candidate_cost + _heuristic(world(neighbor), goal)
            heapq.heappush(frontier, (priority, candidate_cost, neighbor))
    raise ValueError('route_unreachable')
