#!/usr/bin/env python
from __future__ import division, print_function

import argparse
import math

import rosbag

from firefighting_mission.field_map import point_matches_known_static


def yaw_from_quaternion(value):
    return math.atan2(
        2.0 * (value.w * value.z + value.x * value.y),
        1.0 - 2.0 * (value.y * value.y + value.z * value.z))


def finite_pose(message):
    point = message.pose.position
    return all(not math.isnan(value) and not math.isinf(value)
               for value in (point.x, point.y, point.z))


def obstacle_summary(pose, message, sensor_offset=0.32,
                     lateral_trigger=0.55, trigger_range=1.0):
    if pose is None:
        return '-'
    point = pose.pose.position
    yaw = yaw_from_quaternion(pose.pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    values = []
    for obstacle in message.obstacles:
        forward = obstacle.forward_m + sensor_offset
        left = obstacle.left_m
        world_x = point.x + cosine * forward - sine * left
        world_y = point.y + sine * forward + cosine * left
        static = point_matches_known_static((world_x, world_y), 0.18)
        corridor = abs(left) <= lateral_trigger
        trigger = (corridor and not static and
                   obstacle.nearest_range_m + sensor_offset < trigger_range)
        values.append(
            'f=%.2f,l=%.2f,w=(%.2f,%.2f),static=%d,corridor=%d,trigger=%d' %
            (forward, left, world_x, world_y, static, corridor, trigger))
    return '; '.join(values) if values else '-'


def analyze(path):
    topics = (
        '/mavros/local_position/pose',
        '/mavros/setpoint_position/local',
        '/mavros/state',
        '/fire_mission/avoidance_status',
        '/fire_mission/obstacles',
    )
    pose = None
    setpoint = None
    obstacles = None
    mode = ''
    avoidance = ''
    next_second = None
    with rosbag.Bag(path) as bag:
        for topic, message, _stamp in bag.read_messages(topics=topics):
            stamp = message.header.stamp.to_sec()
            if topic == '/mavros/local_position/pose':
                pose = message
            elif topic == '/mavros/setpoint_position/local':
                setpoint = message
            elif topic == '/mavros/state':
                mode = '%s/armed=%d' % (message.mode, message.armed)
            elif topic == '/fire_mission/avoidance_status':
                value = '%s:%s' % (message.state, message.reason)
                if value != avoidance:
                    print('EVENT %.2f %s' % (stamp, value))
                    avoidance = value
            elif topic == '/fire_mission/obstacles':
                obstacles = message

            if pose is None or setpoint is None or obstacles is None:
                continue
            if next_second is None:
                next_second = math.floor(stamp) + 1.0
            if stamp < next_second:
                continue
            point = pose.pose.position
            target = setpoint.pose.position
            print(
                'T %.1f mode=%s pose=(%.2f,%.2f,%.2f,yaw=%.2f,finite=%d) '
                'sp=(%.2f,%.2f,%.2f,yaw=%.2f) obstacles=[%s]' % (
                    stamp, mode, point.x, point.y, point.z,
                    yaw_from_quaternion(pose.pose.orientation),
                    finite_pose(pose), target.x, target.y, target.z,
                    yaw_from_quaternion(setpoint.pose.orientation),
                    obstacle_summary(pose, obstacles)))
            next_second += 1.0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    analyze(parser.parse_args().bag)
