"""Fly the competition avoidance chain and report one machine-readable line.

Seed 1 draws cylinder 2 into placement 2, which puts the rescue zone 0.602 m
from a cylinder centre.  Goals go straight to the planner so the run does not
depend on the target detection module.
"""
from __future__ import division, print_function

import math
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

from firefighting_mission.msg import AvoidanceStatus

CYLINDERS = ((0.70, -1.45), (2.10, -1.95))
CYLINDER_RADIUS = 0.10
AIRFRAME_RADIUS = 0.20
COLLISION_DISTANCE = CYLINDER_RADIUS + AIRFRAME_RADIUS
LEGS = (('hazard', (1.25, -0.10)),
        ('rescue', (2.70, -1.90)),
        ('home', (0.00, 0.00)))
LEG_TIMEOUT = 100.0
HOVER_TIMEOUT = 120.0

state = {}


def remember(key):
    def callback(message):
        state[key] = message
    return callback


def drone_position():
    models = state.get('gz')
    if models is None or 'iris_0' not in models.name:
        return None
    return models.pose[models.name.index('iris_0')].position


def wait_for_hover():
    deadline = rospy.Time.now().to_sec() + HOVER_TIMEOUT
    while rospy.Time.now().to_sec() < deadline:
        point = drone_position()
        if point is not None and point.z > 1.10:
            return True
        rospy.sleep(0.5)
    return False


def fly_leg(publisher, goal):
    """Return (seconds or None, closest cylinder approach, why it stopped).

    The stop reason is sampled when the leg gives up, not accumulated over
    the leg: a transient reason seen early says nothing about what the
    aircraft was actually stuck on.
    """
    target = PoseStamped()
    target.header.frame_id = 'map'
    target.pose.position.x, target.pose.position.y = goal
    target.pose.position.z = 1.25
    target.pose.orientation.w = 1.0

    started = rospy.Time.now().to_sec()
    closest = float('inf')
    while rospy.Time.now().to_sec() - started < LEG_TIMEOUT:
        target.header.stamp = rospy.Time.now()
        publisher.publish(target)
        rospy.sleep(0.2)
        point = drone_position()
        status = state.get('av')
        if point is None or status is None:
            continue
        for cylinder in CYLINDERS:
            closest = min(closest, math.hypot(point.x - cylinder[0],
                                              point.y - cylinder[1]))
        if status.state == 'REACHED':
            return rospy.Time.now().to_sec() - started, closest, ''
    status = state.get('av')
    point = drone_position()
    stuck = 'no_status'
    if status is not None:
        stuck = '%s/%s' % (status.state, status.reason or 'none')
        if point is not None:
            stuck += '@%.2f,%.2f,%.2f' % (point.x, point.y, point.z)
    return None, closest, stuck


def main(label):
    rospy.init_node('leg_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, remember('gz'))
    rospy.Subscriber('/fire_mission/avoidance_status', AvoidanceStatus,
                     remember('av'))
    publisher = rospy.Publisher('/fire_mission/point_goal', PoseStamped,
                                queue_size=1, latch=True)
    rospy.sleep(3.0)

    fields = ['run=%s' % label]
    if not wait_for_hover():
        print('RESULT %s outcome=no_hover' % ' '.join(fields))
        return 1

    total = 0.0
    worst = float('inf')
    for name, goal in LEGS:
        elapsed, closest, reason = fly_leg(publisher, goal)
        worst = min(worst, closest)
        if elapsed is None:
            fields.append('%s=timeout' % name)
            fields.append('closest=%.3f' % worst)
            fields.append('outcome=stuck_at_%s' % name)
            fields.append('stuck=%s' % (reason or 'none'))
            print('RESULT %s' % ' '.join(fields))
            return 1
        total += elapsed
        fields.append('%s=%.1f' % (name, elapsed))

    fields.append('total=%.1f' % total)
    fields.append('closest=%.3f' % worst)
    collided = worst <= COLLISION_DISTANCE
    fields.append('outcome=%s' % ('collision' if collided else 'complete'))
    fields.append('within_180=%s' % ('yes' if total <= 180.0 else 'no'))
    print('RESULT %s' % ' '.join(fields))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else '0'))
