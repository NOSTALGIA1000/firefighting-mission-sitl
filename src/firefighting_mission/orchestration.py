from __future__ import print_function


ALIGNMENT_PHASES = frozenset((
    'ALIGN_HAZARD', 'DROP_FIRE', 'ALIGN_PERSON', 'DROP_RESCUE',
))


def validated_alignment(phase, detection_confirmed, nav_status):
    return bool(phase in ALIGNMENT_PHASES and detection_confirmed and
                nav_status == 'REACHED')


def payload_link_names(channel, vehicle_name='iris_0'):
    channel = int(channel)
    link = 'fire_payload_link' if channel == 1 else 'rescue_payload_link'
    return ('%s::%s' % (vehicle_name, link), link)


def contacts_indicate_collision(contact_states):
    return bool(contact_states)
