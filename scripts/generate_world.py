#!/usr/bin/env python
from __future__ import print_function

import argparse

from firefighting_mission.world_generator import generate_world


def main():
    parser = argparse.ArgumentParser(description='Generate a seeded firefighting world')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    scenario = generate_world(args.seed, args.output)
    print('generated seed=%d cylinders=%s hazard=%d person=%d' % (
        scenario.seed, scenario.cylinder_positions, scenario.hazard_index,
        scenario.person_position,
    ))


if __name__ == '__main__':
    main()
