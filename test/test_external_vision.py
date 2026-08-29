from __future__ import print_function

import unittest

from firefighting_mission.external_vision import (
    model_pose,
    model_state,
    world_vector_to_body,
)


class FakeModelStates(object):
    def __init__(self, names, poses, twists=None):
        self.name = names
        self.pose = poses
        self.twist = [] if twists is None else twists


class ExternalVisionTest(unittest.TestCase):
    def test_returns_pose_for_named_model(self):
        iris_pose = object()
        message = FakeModelStates(['ground_plane', 'iris_0'],
                                  [object(), iris_pose])

        self.assertIs(iris_pose, model_pose(message, 'iris_0'))

    def test_returns_none_when_model_is_missing(self):
        message = FakeModelStates(['ground_plane'], [object()])

        self.assertIsNone(model_pose(message, 'iris_0'))

    def test_returns_none_when_pose_list_is_shorter_than_name_list(self):
        message = FakeModelStates(['ground_plane', 'iris_0'], [object()])

        self.assertIsNone(model_pose(message, 'iris_0'))

    def test_returns_paired_pose_and_twist_for_named_model(self):
        iris_pose = object()
        iris_twist = object()
        message = FakeModelStates(
            ['ground_plane', 'iris_0'],
            [object(), iris_pose],
            [object(), iris_twist])

        self.assertEqual((iris_pose, iris_twist),
                         model_state(message, 'iris_0'))

    def test_returns_none_when_twist_list_is_shorter_than_name_list(self):
        message = FakeModelStates(
            ['ground_plane', 'iris_0'],
            [object(), object()],
            [object()])

        self.assertIsNone(model_state(message, 'iris_0'))

    def test_rotates_world_vector_into_body_frame(self):
        half_root = 2.0 ** -0.5
        orientation = FakeQuaternion(0.0, 0.0, half_root, half_root)

        body = world_vector_to_body(orientation, FakeVector(0.0, 1.0, 0.0))

        self.assertAlmostEqual(1.0, body[0])
        self.assertAlmostEqual(0.0, body[1])
        self.assertAlmostEqual(0.0, body[2])


class FakeQuaternion(object):
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class FakeVector(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


if __name__ == '__main__':
    unittest.main()
