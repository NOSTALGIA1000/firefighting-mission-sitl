from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class StereoModelTest(unittest.TestCase):
    def test_forward_sensor_exposes_metric_depth_contract(self):
        path = os.path.join(PROJECT_ROOT, 'models', 'fire_stereo_camera',
                            'model.sdf')
        root = ET.parse(path).getroot()
        sensor = root.find(".//sensor[@type='depth']")

        self.assertIsNotNone(sensor)
        plugin = sensor.find(
            "plugin[@filename='libgazebo_ros_depth_camera.so']")
        self.assertIsNotNone(plugin)
        self.assertEqual('/fire_stereo/depth/image_raw',
                         plugin.find('depthImageTopicName').text)
        self.assertEqual('/fire_stereo/depth/camera_info',
                         plugin.find('depthImageCameraInfoTopicName').text)
        self.assertEqual('/fire_stereo/points',
                         plugin.find('pointCloudTopicName').text)
        self.assertEqual('/fire_stereo/rgb/image_raw',
                         plugin.find('imageTopicName').text)

    def test_depth_range_covers_complete_competition_field(self):
        root = ET.parse(os.path.join(
            PROJECT_ROOT, 'models', 'fire_stereo_camera', 'model.sdf')).getroot()
        sensor = root.find(".//sensor[@type='depth']")

        self.assertEqual('0.20', sensor.find('camera/clip/near').text)
        self.assertEqual('4.00', sensor.find('camera/clip/far').text)
        self.assertEqual('15', sensor.find('update_rate').text)
        self.assertEqual('320', sensor.find('camera/image/width').text)
        self.assertEqual('180', sensor.find('camera/image/height').text)

    def test_sensor_mount_faces_forward(self):
        iris = ET.parse(os.path.join(PROJECT_ROOT, 'models', 'fire_iris',
                                     'fire_iris.sdf')).getroot()
        joint = iris.find(".//joint[@name='fire_stereo_joint']")
        include = next(value for value in iris.findall('.//include')
                       if value.find('uri') is not None and
                       value.find('uri').text == 'model://fire_stereo_camera')

        self.assertIsNotNone(joint)
        self.assertEqual('fire_stereo_camera::link', joint.find('child').text)
        self.assertEqual('base_link', joint.find('parent').text)
        self.assertEqual('0.32 0 0.03 0 0 0', include.find('pose').text)

    def test_visual_sensor_does_not_shift_px4_airframe_inertia(self):
        root = ET.parse(os.path.join(
            PROJECT_ROOT, 'models', 'fire_stereo_camera', 'model.sdf')).getroot()

        self.assertLess(float(root.find('.//link/inertial/mass').text), 0.001)

    def test_sitl_map_frame_disables_gps_random_walk(self):
        root = ET.parse(os.path.join(
            PROJECT_ROOT, 'models', 'fire_iris', 'fire_iris.sdf')).getroot()
        gps = root.find(".//plugin[@name='gps_plugin']")

        self.assertIsNotNone(gps)
        self.assertEqual('0', gps.find('gpsNoise').text)

    def test_px4_rotor_and_imu_joint_axes_use_parent_model_frame(self):
        """Joint axes must stay in the parent model frame, as PX4 ships them.

        Setting these to 0 makes Gazebo 9 report every ``iris_0`` link at
        exactly (0,0,0) with NaN velocity while static models in the same
        world stay clean.  PX4 then receives no IMU at all, EKF2 resets to
        external vision every 40 ms, ``/mavros/local_position`` publishes a
        NaN altitude and the aircraft never arms.  Observed on the VM on
        2026-08-31; restoring 1 restored normal flight immediately.
        """
        root = ET.parse(os.path.join(
            PROJECT_ROOT, 'models', 'fire_iris', 'fire_iris.sdf')).getroot()
        joint_names = ('/imu_joint', 'rotor_0_joint', 'rotor_1_joint',
                       'rotor_2_joint', 'rotor_3_joint')

        for name in joint_names:
            joint = root.find(".//joint[@name='%s']" % name)
            self.assertIsNotNone(joint)
            self.assertEqual(
                '1', joint.find('axis/use_parent_model_frame').text,
                name)

    def test_sitl_loader_exposes_gazebo_depth_plugin_dependencies(self):
        with open(os.path.join(PROJECT_ROOT, 'scripts', 'start_sitl.sh'),
                  'r') as handle:
            wrapper = handle.read()

        self.assertIn('/usr/lib/x86_64-linux-gnu/gazebo-9/plugins', wrapper)
        self.assertIn('/opt/ros/melodic/lib:$gazebo_system_plugin_path', wrapper)
        self.assertIn('gazebo_system_plugin_path', wrapper)


if __name__ == '__main__':
    unittest.main()
