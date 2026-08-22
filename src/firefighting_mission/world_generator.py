from __future__ import print_function

import random
from collections import namedtuple


Scenario = namedtuple(
    'Scenario',
    'seed bounds cylinder_positions hazard_index hazard_symbol person_position'
)

FIELD_BOUNDS = (-0.65, 3.35, -3.35, 0.65, 3.0)

CYLINDER_POSES = {
    1: ((0.70, -1.45), (2.10, -1.45)),
    2: ((0.70, -2.45), (2.10, -1.95)),
}
PERSON_POSES = {
    1: (2.70, -1.10),
    2: (2.70, -1.90),
    3: (2.70, -2.65),
}
HAZARD_POSES = {
    1: (1.40, 0.00),
    2: (1.40, -0.45),
}
FIXED_OBSTACLES = (
    ('fixed_obstacle_1', 0.70, -0.20, 0.0, 0.10, 1.70),
    ('fixed_obstacle_2', 2.72, 0.04, 0.785398, 1.60, 0.10),
    ('fixed_obstacle_3', 0.70, -3.10, 0.0, 0.10, 0.50),
    ('fixed_obstacle_4', 2.10, -3.10, 0.0, 0.10, 0.50),
)


def physical_side_clearance(x, radius, bounds=FIELD_BOUNDS):
    left, right = bounds[0], bounds[1]
    return max((x - radius) - left, right - (x + radius))


def build_scenario(seed):
    rng = random.Random(int(seed))
    return Scenario(
        seed=int(seed),
        bounds=(FIELD_BOUNDS[1] - FIELD_BOUNDS[0],
                FIELD_BOUNDS[3] - FIELD_BOUNDS[2], FIELD_BOUNDS[4]),
        cylinder_positions=(rng.choice((1, 2)), rng.choice((1, 2))),
        hazard_index=rng.choice((1, 2)),
        hazard_symbol=rng.choice(('flammable', 'explosive', 'toxic')),
        person_position=rng.choice((1, 2, 3)),
    )


def _box_model(name, x, y, z, yaw, size, color, static=True):
    static_text = 'true' if static else 'false'
    return '''
    <model name="{name}">
      <static>{static}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''.format(
        name=name, static=static_text, x=x, y=y, z=z, yaw=yaw,
        size=size, color=color,
    )


def _cylinder_model(name, x, y, radius, length, color, collision=True):
    collision_xml = ''
    if collision:
        collision_xml = '''
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>'''.format(
            radius=radius, length=length)
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">{collision}
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''.format(
        name=name, x=x, y=y, z=length / 2.0, radius=radius,
        length=length, color=color, collision=collision_xml,
    )


def _zone_model(name, x, y, border_color, material_name, metadata):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.006 0 0 0</pose>
      <link name="link">
        <visual name="border">
          <geometry><box><size>0.4 0.4 0.01</size></box></geometry>
          <material><ambient>{border}</ambient><diffuse>{border}</diffuse></material>
        </visual>
        <visual name="target_image">
          <pose>0 0 0.006 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>0.34 0.34</size></plane></geometry>
          <material><script>
            <uri>model://targets/materials/scripts</uri>
            <uri>model://targets/materials/textures</uri>
            <name>{material}</name>
          </script></material>
        </visual>
      </link>
      <!-- target_class: {metadata} -->
    </model>'''.format(name=name, x=x, y=y, border=border_color,
                       material=material_name, metadata=metadata)


def _safety_net_model(name, x, y, size):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 1.5000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.25 0.45 0.70 0.20</ambient><diffuse>0.25 0.45 0.70 0.20</diffuse></material>
          <transparency>0.80</transparency>
        </visual>
      </link>
    </model>'''.format(name=name, x=x, y=y, size=size)


def render_world(scenario):
    models = []
    models.append(_safety_net_model('safety_net_north', 1.35, 0.65,
                                    '4.0 0.02 3.0'))
    models.append(_safety_net_model('safety_net_south', 1.35, -3.35,
                                    '4.0 0.02 3.0'))
    models.append(_safety_net_model('safety_net_west', -0.65, -1.35,
                                    '0.02 4.0 3.0'))
    models.append(_safety_net_model('safety_net_east', 3.35, -1.35,
                                    '0.02 4.0 3.0'))
    models.append(_cylinder_model('start_zone', 0.0, 0.0, 0.25, 0.01,
                                  '0.1 0.7 0.9 1', collision=False))
    for name, x, y, yaw, length, width in FIXED_OBSTACLES:
        models.append(_box_model(name, x, y, 1.0, yaw,
                                 '%.2f %.2f 2.0' % (length, width),
                                 '0.45 0.45 0.45 1'))
    for index, selection in enumerate(scenario.cylinder_positions, start=1):
        x, y = CYLINDER_POSES[selection][index - 1]
        models.append(_cylinder_model('random_cylinder_%d' % index, x, y,
                                      0.1, 2.0, '0.9 0.2 0.55 1'))
    for index in (1, 2):
        x, y = HAZARD_POSES[index]
        correct = index == scenario.hazard_index
        metadata = scenario.hazard_symbol if correct else 'distractor'
        models.append(_zone_model('hazard_zone_%d' % index, x, y,
                                  '0.9 0.05 0.05 1',
                                  'FireTargets/%s' % metadata.title(), metadata))
    person_x, person_y = PERSON_POSES[scenario.person_position]
    models.append(_zone_model('person_zone', person_x, person_y,
                              '0.05 0.75 0.95 1', 'FireTargets/Person', 'person'))
    return '''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="firefighting_seed_{seed}">
    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.92 0.92 0.92 1</background></scene>
    <include><uri>model://sun</uri></include>
    <model name="field_floor">
      <static>true</static>
      <pose>1.35 -1.35 -0.03 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>4 4 0.05</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>4 4 0.05</size></box></geometry><material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      </link>
    </model>
{models}
  </world>
</sdf>
'''.format(seed=scenario.seed, models='\n'.join(models))


def generate_world(seed, output_path):
    scenario = build_scenario(seed)
    rendered = render_world(scenario)
    with open(output_path, 'wb') as handle:
        handle.write(rendered.encode('utf-8'))
    return scenario
