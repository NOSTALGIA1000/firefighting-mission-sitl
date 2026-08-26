from __future__ import division, print_function

import math
from collections import namedtuple

import numpy as np


ObstacleClusterData = namedtuple(
    'ObstacleClusterData',
    'forward_m left_m nearest_range_m left_edge_m right_edge_m confidence')


def _finite(value):
    return not math.isnan(value) and not math.isinf(value)


def _summarize(entries, min_samples):
    samples = []
    for _, values in entries:
        samples.extend(values)
    if len(samples) < min_samples:
        return None
    forwards = np.asarray([value[0] for value in samples], dtype=np.float32)
    lefts = np.asarray([value[1] for value in samples], dtype=np.float32)
    return ObstacleClusterData(
        float(np.median(forwards)),
        float(np.median(lefts)),
        float(np.percentile(forwards, 10.0)),
        float(np.max(lefts)),
        float(np.min(lefts)),
        min(1.0, len(samples) / 50.0),
    )


def _cluster_horizontal_samples(samples, bin_width=0.08, min_samples=8,
                                depth_bin_width=0.15):
    bins = {}
    for forward, left in samples:
        key = (int(math.floor(left / float(bin_width))),
               int(math.floor(forward / float(depth_bin_width))))
        bins.setdefault(key, []).append((forward, left))
    groups = []
    remaining = set(bins)
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        pending = [seed]
        group = []
        while pending:
            key = pending.pop()
            group.append((key, bins[key]))
            for lateral_delta in (-1, 0, 1):
                for depth_delta in (-1, 0, 1):
                    neighbor = (key[0] + lateral_delta,
                                key[1] + depth_delta)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        pending.append(neighbor)
        groups.append(group)
    clusters = []
    for group in groups:
        cluster = _summarize(group, min_samples)
        if cluster is not None:
            clusters.append(cluster)
    return tuple(sorted(clusters, key=lambda value: value.nearest_range_m))


def clusters_from_depth(depth_m, fx, cx, min_depth=0.25, max_depth=3.0):
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError('depth_image_must_be_2d')
    if float(fx) <= 0.0:
        raise ValueError('invalid_focal_length')
    height, width = depth.shape
    first_row = int(round(height * 0.20))
    last_row = int(round(height * 0.80))
    samples = []
    for row in range(first_row, last_row, 2):
        for column in range(0, width, 2):
            forward = float(depth[row, column])
            if (not _finite(forward) or forward < min_depth or
                    forward > max_depth):
                continue
            left = -(float(column) - float(cx)) * forward / float(fx)
            samples.append((forward, left))
    return _cluster_horizontal_samples(samples)


def clusters_from_points(points_xyz, min_depth=0.25, max_depth=3.0):
    samples = []
    for point in points_xyz:
        if len(point) < 3:
            continue
        forward, left, up = (float(point[0]), float(point[1]), float(point[2]))
        if not (_finite(forward) and _finite(left) and _finite(up)):
            continue
        if forward < min_depth or forward > max_depth:
            continue
        if up < -0.45 or up > 0.45:
            continue
        samples.append((forward, left))
    return _cluster_horizontal_samples(samples)


def _matching_cluster(reference, frame, lateral_tolerance=0.12,
                      range_tolerance=0.15):
    candidates = [cluster for cluster in frame
                  if abs(cluster.left_m - reference.left_m) <= lateral_tolerance
                  and abs(cluster.forward_m - reference.forward_m) <= range_tolerance]
    if not candidates:
        return None
    return min(candidates, key=lambda cluster:
               abs(cluster.left_m - reference.left_m) +
               abs(cluster.forward_m - reference.forward_m))


def stable_clusters(history, required=3):
    frames = tuple(history)
    if required <= 0:
        raise ValueError('required_must_be_positive')
    if len(frames) < required or not frames:
        return ()
    reference_frame = frames[-1]
    stable = []
    for reference in reference_frame:
        matches = []
        for frame in frames[-required:]:
            match = _matching_cluster(reference, frame)
            if match is None:
                matches = []
                break
            matches.append(match)
        if len(matches) != required:
            continue
        stable.append(ObstacleClusterData(
            sum(value.forward_m for value in matches) / required,
            sum(value.left_m for value in matches) / required,
            min(value.nearest_range_m for value in matches),
            max(value.left_edge_m for value in matches),
            min(value.right_edge_m for value in matches),
            min(value.confidence for value in matches),
        ))
    return tuple(sorted(stable, key=lambda value: value.nearest_range_m))
