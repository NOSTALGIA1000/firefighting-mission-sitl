from __future__ import division, print_function

import os
import sys
import unittest

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.stereo_obstacles import (
    clusters_from_depth, clusters_from_points, stable_clusters)


class StereoObstacleTest(unittest.TestCase):
    def test_depth_patch_becomes_metric_body_cluster(self):
        depth = np.empty((60, 80), dtype=np.float32)
        depth.fill(np.nan)
        depth[20:45, 36:44] = 1.0

        clusters = clusters_from_depth(depth, fx=80.0, cx=40.0)

        self.assertEqual(1, len(clusters))
        self.assertAlmostEqual(1.0, clusters[0].forward_m, places=2)
        self.assertAlmostEqual(0.0, clusters[0].left_m, delta=0.02)
        self.assertLess(clusters[0].right_edge_m, clusters[0].left_edge_m)

    def test_floor_band_and_single_pixel_noise_are_rejected(self):
        depth = np.empty((60, 80), dtype=np.float32)
        depth.fill(np.nan)
        depth[58, :] = 0.5
        depth[25, 40] = 0.8

        self.assertEqual((), clusters_from_depth(depth, 80.0, 40.0))

    def test_point_cloud_uses_forward_left_up_convention(self):
        points = [(1.0, 0.20, 0.0), (1.0, 0.21, 0.02),
                  (1.02, 0.19, -0.02)] * 4

        cluster = clusters_from_points(points)[0]

        self.assertGreater(cluster.left_m, 0.0)
        self.assertAlmostEqual(1.0, cluster.nearest_range_m, places=2)

    def test_invalid_ranges_are_rejected(self):
        points = [(0.10, 0.0, 0.0), (4.50, 0.0, 0.0),
                  (float('nan'), 0.0, 0.0)] * 10

        self.assertEqual((), clusters_from_points(points))

    def test_three_consistent_frames_form_stable_cluster(self):
        frame = clusters_from_points([(1.0, 0.2, 0.0)] * 10)

        self.assertEqual((), stable_clusters((frame, frame), required=3))
        stable = stable_clusters((frame, frame, frame), required=3)
        self.assertEqual(1, len(stable))
        self.assertAlmostEqual(0.2, stable[0].left_m, places=2)

    def test_inconsistent_lateral_clusters_are_not_stable(self):
        left = clusters_from_points([(1.0, 0.4, 0.0)] * 10)
        right = clusters_from_points([(1.0, -0.4, 0.0)] * 10)

        self.assertEqual((), stable_clusters((left, right, left), required=3))


if __name__ == '__main__':
    unittest.main()
