from __future__ import print_function


ALIGNMENT_PHASES = frozenset((
    'ALIGN_HAZARD', 'DROP_FIRE', 'ALIGN_PERSON', 'DROP_RESCUE',
))
MISSION_OBSTACLE_PREFIXES = (
    'fixed_obstacle_', 'random_cylinder_', 'safety_net_')


def validated_alignment(phase, detection_confirmed, nav_status):
    return bool(phase in ALIGNMENT_PHASES and detection_confirmed and
                nav_status == 'REACHED')


def payload_link_names(channel, vehicle_name='iris_0'):
    channel = int(channel)
    link = 'fire_payload_link' if channel == 1 else 'rescue_payload_link'
    return ('%s::%s' % (vehicle_name, link), link)


def payload_link_position(link_positions, channel):
    """Return a detached payload position without assuming Gazebo's model name."""
    names = payload_link_names(channel)
    suffix = '::' + names[1]
    for name in sorted(link_positions):
        if name == names[1] or name.endswith(suffix):
            return link_positions[name]
    return None


def contacts_indicate_collision(contact_states):
    for state in contact_states:
        first = getattr(state, 'collision1_name', '')
        second = getattr(state, 'collision2_name', '')
        pair = first + ' ' + second
        if any(prefix in pair for prefix in MISSION_OBSTACLE_PREFIXES):
            return True
    return False


def recording_topics(mavros_prefix, scan_topic):
    prefix = mavros_prefix.rstrip('/')
    return (
        '/fire_mission/phase', '/fire_mission/event',
        '/fire_mission/detection', '/fire_mission/drop_result',
        '/fire_mission/annotated', '/fire_mission/mission_view',
        '/fire_mission/obstacles', '/fire_mission/avoidance_status',
        '/fire_mission/path_status', '/fire_stereo/rgb/image_raw',
        '/fire_stereo/depth/image_raw', prefix + '/local_position/pose',
        prefix + '/state', scan_topic, '/gazebo/model_states',
        '/gazebo/link_states', '/fire_mission/contacts',
    )


def completion_should_shutdown(recorder_finalized, now_seconds, deadline_seconds):
    return bool(recorder_finalized or now_seconds >= deadline_seconds)
