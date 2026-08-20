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
    return round(max((x - radius) - left, right - (x + radius)), 10)


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


def _cylinder_model(name, x, y, radius, length, color):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''.format(
        name=name, x=x, y=y, z=length / 2.0, radius=radius,
        length=length, color=color,
    )


def _zone_model(name, x, y, color, metadata):
    return '''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.006 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.4 0.4 0.01</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
      <!-- target_class: {metadata} -->
    </model>'''.format(name=name, x=x, y=y, color=color, metadata=metadata)


def render_world(scenario):
    models = []
    models.append(_cylinder_model('start_zone', 0.0, 0.0, 0.25, 0.01,
                                  '0.1 0.7 0.9 1'))
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
        color = '0.95 0.75 0.05 1' if correct else '0.25 0.55 0.95 1'
        models.append(_zone_model('hazard_zone_%d' % index, x, y, color, metadata))
    person_x, person_y = PERSON_POSES[scenario.person_position]
    models.append(_zone_model('person_zone', person_x, person_y,
                              '0.2 0.8 0.35 1', 'person'))
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
