from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from firefighting_mission.competition_main import (CompetitionMain,
                                                    ModeRequest,
                                                    PreflightHealthGate,
                                                    PreflightSample,
                                                    PositionSetpoint,
                                                    mission_interface_topics,
                                                    select_active_setpoints)


def healthy_preflight_sample(now, **changes):
    values = dict(
        connected=True,
        armed=False,
        system_status=3,
        estimator_received_at=now,
        estimator_attitude_valid=True,
        estimator_accel_error=False,
        imu_received_at=now,
        imu_orientation=(0.0, 0.0, 0.0, 1.0),
        imu_angular_velocity=(0.0, 0.0, 0.0),
        imu_linear_acceleration=(0.0, 0.0, 9.81),
    )
    values.update(changes)
    return PreflightSample(**values)


class CompetitionMainTest(unittest.TestCase):
    def test_preflight_gate_requires_continuous_health_window(self):
        gate = PreflightHealthGate(stable_seconds=3.0)

        self.assertFalse(gate.update(10.0, healthy_preflight_sample(10.0)))
        self.assertFalse(gate.update(12.9, healthy_preflight_sample(12.9)))
        self.assertTrue(gate.update(13.0, healthy_preflight_sample(13.0)))
        self.assertEqual('ready', gate.reason)

    def test_unhealthy_sample_resets_preflight_health_window(self):
        gate = PreflightHealthGate(stable_seconds=3.0)

        gate.update(10.0, healthy_preflight_sample(10.0))
        gate.update(12.0, healthy_preflight_sample(
            12.0, estimator_accel_error=True))

        self.assertFalse(gate.update(14.0, healthy_preflight_sample(14.0)))
        self.assertTrue(gate.update(17.0, healthy_preflight_sample(17.0)))

    def test_default_freshness_accepts_one_hertz_estimator_cadence(self):
        gate = PreflightHealthGate(stable_seconds=3.0)

        gate.update(10.0, healthy_preflight_sample(10.0))
        accepted = gate.update(
            11.1,
            healthy_preflight_sample(11.1, estimator_received_at=10.0))

        self.assertFalse(accepted)
        self.assertEqual('stabilizing', gate.reason)

    def test_default_acceleration_range_accepts_observed_sitl_noise(self):
        gate = PreflightHealthGate(stable_seconds=0.0)

        accepted = gate.update(
            10.0,
            healthy_preflight_sample(
                10.0, imu_linear_acceleration=(0.0, 0.0, 15.1)))

        self.assertTrue(accepted)
        self.assertEqual('ready', gate.reason)

    def test_preflight_gate_rejects_each_invalid_input(self):
        cases = (
            ('disconnected', dict(connected=False)),
            ('px4_not_standby', dict(system_status=2)),
            ('estimator_stale', dict(estimator_received_at=9.0)),
            ('attitude_invalid', dict(estimator_attitude_valid=False)),
            ('accelerometer_error', dict(estimator_accel_error=True)),
            ('imu_stale', dict(imu_received_at=9.0)),
            ('imu_non_finite', dict(imu_orientation=(
                0.0, 0.0, 0.0, float('nan')))),
            ('acceleration_out_of_range', dict(
                imu_linear_acceleration=(0.0, 0.0, 1.0))),
            ('acceleration_out_of_range', dict(
                imu_linear_acceleration=(0.0, 0.0, 21.0))),
        )

        for reason, changes in cases:
            gate = PreflightHealthGate(stable_seconds=0.0,
                                       max_message_age=0.5)
            self.assertFalse(gate.update(
                10.0, healthy_preflight_sample(10.0, **changes)))
            self.assertEqual(reason, gate.reason)

    def test_planned_setpoint_preserves_yaw(self):
        point = PositionSetpoint(1.0, 2.0, 1.2, 1.5708)

        self.assertAlmostEqual(1.5708, point.yaw, places=4)

    def test_waits_for_fcu_before_publishing_takeoff_control(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        outputs = controller.tick(0.0, connected=False, armed=False,
                                  mode='', altitude=0.0)

        self.assertEqual('WAIT_FCU', outputs.state)
        self.assertEqual([], outputs.setpoints)
        self.assertEqual([], outputs.mode_requests)
        self.assertFalse(outputs.arm_request)

    def test_waits_for_sensor_data_before_prestreaming_offboard_setpoints(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        outputs = controller.tick(0.0, connected=True, armed=False,
                                  mode='', altitude=0.0,
                                  sensor_ready=False)

        self.assertEqual('WAIT_SENSOR', outputs.state)
        self.assertEqual([], outputs.setpoints)
        self.assertEqual([], outputs.mode_requests)
        self.assertFalse(outputs.arm_request)

    def test_health_loss_before_arm_resets_prestream_count(self):
        controller = CompetitionMain(takeoff_altitude=1.2,
                                     prestream_count=2)

        controller.tick(0.0, connected=True, armed=False, mode='',
                        altitude=0.0, sensor_ready=True)
        blocked = controller.tick(
            0.1, connected=True, armed=False, mode='', altitude=0.0,
            sensor_ready=False)
        restarted = controller.tick(
            0.2, connected=True, armed=False, mode='', altitude=0.0,
            sensor_ready=True)

        self.assertEqual('WAIT_SENSOR', blocked.state)
        self.assertEqual([], blocked.setpoints)
        self.assertEqual([], restarted.mode_requests)

    def test_armed_flight_keeps_setpoints_after_preflight_health_loss(self):
        controller = CompetitionMain(takeoff_altitude=1.2,
                                     prestream_count=1)

        outputs = controller.tick(
            1.0, connected=True, armed=True, mode='OFFBOARD', altitude=1.0,
            sensor_ready=False)

        self.assertEqual('TAKEOFF', outputs.state)
        self.assertEqual(1, len(outputs.setpoints))
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2),
                         outputs.setpoints[0])

    def test_can_prestream_before_local_pose_is_available_after_sensor_ready(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        outputs = controller.tick(0.0, connected=True, armed=False,
                                  mode='', altitude=0.0,
                                  sensor_ready=True,
                                  local_pose_available=False)

        self.assertEqual('PRESTREAM_SETPOINTS', outputs.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), outputs.setpoints[0])

    def test_prestreams_position_setpoints_before_offboard_and_arm(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=3)

        first = controller.tick(0.0, connected=True, armed=False,
                                mode='', altitude=0.0)
        second = controller.tick(0.1, connected=True, armed=False,
                                 mode='', altitude=0.0)
        third = controller.tick(0.2, connected=True, armed=False,
                                mode='', altitude=0.0)

        self.assertEqual('PRESTREAM_SETPOINTS', first.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), first.setpoints[0])
        self.assertEqual([], first.mode_requests)
        self.assertEqual([], second.mode_requests)
        self.assertEqual([ModeRequest('OFFBOARD')], third.mode_requests)
        self.assertFalse(third.arm_request)

    def test_requests_arm_after_offboard_mode_is_active(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1)
        controller.tick(0.0, connected=True, armed=False, mode='',
                        altitude=0.0)

        outputs = controller.tick(0.1, connected=True, armed=False,
                                  mode='OFFBOARD', altitude=0.0)

        self.assertEqual('ARM', outputs.state)
        self.assertTrue(outputs.arm_request)

    def test_retries_offboard_request_until_mode_becomes_active(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1,
                                     mode_retry_seconds=1.0)

        first = controller.tick(0.0, connected=True, armed=False, mode='',
                                altitude=0.0)
        too_soon = controller.tick(0.5, connected=True, armed=False, mode='',
                                   altitude=0.0)
        retry = controller.tick(1.1, connected=True, armed=False, mode='',
                                altitude=0.0)

        self.assertEqual([ModeRequest('OFFBOARD')], first.mode_requests)
        self.assertEqual([], too_soon.mode_requests)
        self.assertEqual([ModeRequest('OFFBOARD')], retry.mode_requests)

    def test_takeoff_reaches_hover_when_target_altitude_is_stable(self):
        controller = CompetitionMain(takeoff_altitude=1.2, hover_tolerance=0.08,
                                     hover_hold_seconds=0.3,
                                     prestream_count=1)
        controller.tick(0.0, connected=True, armed=False, mode='',
                        altitude=0.0)
        controller.tick(0.1, connected=True, armed=False, mode='OFFBOARD',
                        altitude=0.0)

        takeoff = controller.tick(0.2, connected=True, armed=True,
                                  mode='OFFBOARD', altitude=0.8)
        hover_pending = controller.tick(0.3, connected=True, armed=True,
                                        mode='OFFBOARD', altitude=1.18)
        hover = controller.tick(0.7, connected=True, armed=True,
                                mode='OFFBOARD', altitude=1.19)

        self.assertEqual('TAKEOFF', takeoff.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), takeoff.setpoints[0])
        self.assertEqual('TAKEOFF', hover_pending.state)
        self.assertEqual('HOVER', hover.state)
        self.assertEqual(PositionSetpoint(0.0, 0.0, 1.2), hover.setpoints[0])

    def test_path_setpoint_replaces_hover_hold_after_handoff(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1)
        outputs = controller.tick(1.0, connected=True, armed=True,
                                  mode='OFFBOARD', altitude=1.2)
        planned = PositionSetpoint(0.4, -0.2, 1.2, 0.7)

        selected = select_active_setpoints(outputs, planned,
                                           path_control_enabled=True)

        self.assertEqual([planned], selected)

    def test_default_takeoff_setpoint_remains_before_path_handoff(self):
        controller = CompetitionMain(takeoff_altitude=1.2, prestream_count=1)
        outputs = controller.tick(1.0, connected=True, armed=True,
                                  mode='OFFBOARD', altitude=0.8)

        selected = select_active_setpoints(
            outputs, PositionSetpoint(0.0, 0.0, 1.2),
            path_control_enabled=False)

        self.assertEqual(outputs.setpoints, selected)

    def test_state_machine_exposes_placeholder_integration_topics(self):
        topics = mission_interface_topics()

        self.assertEqual('/hazard_detected', topics['hazard_detected'])
        self.assertEqual('/person_detected', topics['person_detected'])
        self.assertEqual('/drop_fire_payload', topics['drop_fire_payload'])
        self.assertEqual('/drop_rescue_payload', topics['drop_rescue_payload'])

    def test_takeoff_launch_runs_only_sitl_and_competition_main(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        node_types = [node.attrib.get('type') for node in root.findall('node')]

        self.assertIn('start_sitl.sh', node_types)
        self.assertIn('competition_main.py', node_types)
        self.assertNotIn('mission_manager_node.py', node_types)
        self.assertNotIn('navigator_node.py', node_types)
        competition = next(node for node in root.findall('node')
                           if node.attrib.get('type') == 'competition_main.py')
        self.assertEqual('true', competition.attrib.get('required'))

    def test_takeoff_launch_uses_stereo_equipped_fire_iris(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        sitl = next(node for node in root.findall('node')
                    if node.attrib.get('type') == 'start_sitl.sh')

        self.assertIn('$(arg sdf)', sitl.attrib.get('args'))
        sdf_arg = next(arg for arg in root.findall('arg')
                       if arg.attrib.get('name') == 'sdf')
        self.assertIn('models/fire_iris/fire_iris.sdf',
                      sdf_arg.attrib.get('default'))

    def test_takeoff_launch_spawns_iris_above_field_floor_for_imu_startup(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        spawn_z_arg = next(arg for arg in root.findall('arg')
                           if arg.attrib.get('name') == 'spawn_z')
        sitl = next(node for node in root.findall('node')
                    if node.attrib.get('type') == 'start_sitl.sh')

        self.assertEqual('0.2', spawn_z_arg.attrib.get('default'))
        self.assertIn('$(arg spawn_z)', sitl.attrib.get('args'))

    def test_takeoff_launch_waits_for_sitl_estimator_before_arming(self):
        root = ET.parse(os.path.join(PROJECT_ROOT, 'launch',
                                     'competition_takeoff.launch')).getroot()
        prestream_arg = next(arg for arg in root.findall('arg')
                             if arg.attrib.get('name') == 'prestream_count')
        competition = next(node for node in root.findall('node')
                           if node.attrib.get('type') == 'competition_main.py')
        prestream_param = next(
            param for param in competition.findall('param')
            if param.attrib.get('name') == 'prestream_count')

        self.assertEqual('120', prestream_arg.attrib.get('default'))
        self.assertEqual('$(arg prestream_count)',
                         prestream_param.attrib.get('value'))

    def test_safety_takeover_is_gated_until_vehicle_is_airborne(self):
        with open(os.path.join(PROJECT_ROOT, 'scripts',
                               'competition_main.py'), 'r') as handle:
            node = handle.read()

        self.assertIn('self.state.armed and self.pose is not None', node)
        self.assertIn("self.safety_action == 'LAND' and airborne", node)


if __name__ == '__main__':
    unittest.main()
