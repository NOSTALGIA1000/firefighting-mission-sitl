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


def simplify_route(points):
    if len(points) < 3:
        return tuple(points)
    result = [points[0]]
    for index in range(1, len(points) - 1):
        before = points[index - 1]
        current = points[index]
        after = points[index + 1]
        first = (round(current[0] - before[0], 6),
                 round(current[1] - before[1], 6))
        second = (round(after[0] - current[0], 6),
                  round(after[1] - current[1], 6))
        if first != second:
            result.append(current)
    result.append(points[-1])
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


def plan_route(start, goal, resolution=0.10, inflation=0.45,
               dynamic_circles=()):
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    resolution = float(resolution)
    inflation = float(inflation)
    if not _inside_field(start, inflation):
        raise ValueError('start_outside_field')
    if not _inside_field(goal, inflation):
        raise ValueError('goal_outside_field')
    if not point_is_free(start, inflation, dynamic_circles):
        raise ValueError('start_blocked')
    if not point_is_free(goal, inflation, dynamic_circles):
        raise ValueError('goal_blocked')
    if resolution <= 0.0:
        raise ValueError('invalid_resolution')

    minimum_x = FIELD_BOUNDS[0] + inflation
    minimum_y = FIELD_BOUNDS[2] + inflation
    maximum_x = FIELD_BOUNDS[1] - inflation
    maximum_y = FIELD_BOUNDS[3] - inflation
    columns = int(round((maximum_x - minimum_x) / resolution))
    rows = int(round((maximum_y - minimum_y) / resolution))

    def grid(point):
        return (int(round((point[0] - minimum_x) / resolution)),
                int(round((point[1] - minimum_y) / resolution)))

    def world(node):
        return (round(minimum_x + node[0] * resolution, 6),
                round(minimum_y + node[1] * resolution, 6))

    def valid(node):
        return (0 <= node[0] <= columns and 0 <= node[1] <= rows and
                point_is_free(world(node), inflation, dynamic_circles))

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
            return simplify_route(points)
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
            candidate_cost = current_cost + step_cost
            if candidate_cost >= costs.get(neighbor, float('inf')):
                continue
            costs[neighbor] = candidate_cost
            parents[neighbor] = current
            priority = candidate_cost + _heuristic(world(neighbor), goal)
            heapq.heappush(frontier, (priority, candidate_cost, neighbor))
    raise ValueError('route_unreachable')
